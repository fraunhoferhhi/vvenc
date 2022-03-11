/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD

License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

     * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

     * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */
/** \file     Quant.h
    \brief    base quantization class (header)
*/

#pragma once

#include "CommonDef.h"
#include "Unit.h"
#include "Contexts.h"
#include "ContextModelling.h"
#include "UnitPartitioner.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Constants
// ====================================================================================================================

#define QP_BITS                 15

static inline int getTransformShift(const int channelBitDepth, const Size size, const int maxLog2TrDynamicRange)
{
  return maxLog2TrDynamicRange - channelBitDepth - ( ( Log2(size.width) + Log2(size.height) ) >> 1 );
}

static inline int getScalingListType(const PredMode predMode, const ComponentID compID)
{
  return ((predMode == MODE_INTRA) ? 0 : MAX_NUM_COMP) + compID;
}

// ====================================================================================================================
// Class definition
// ====================================================================================================================
struct TrQuantParams
{
  int     rightShift;
  int     qScale;
};

/// QP struct
class QpParam
{
public:
  short  Qps[2];
  int8_t pers[2];
  int8_t rems[2];

public:
  QpParam(const TransformUnit& tu, const ComponentID &compID, const bool allowACTQpoffset = true);
  
  int Qp ( const bool ts ) const { return Qps [ts?1:0]; }
  int per( const bool ts ) const { return pers[ts?1:0]; }
  int rem( const bool ts ) const { return rems[ts?1:0]; }
}; // END STRUCT DEFINITION QpParam

/// transform and quantization class
class Quant
{
public:
  Quant( const Quant* other, bool useScalingLists );
  virtual ~Quant();

  // initialize class
  virtual void init( int rdoq = 0, bool useRDOQTS = false, bool useSelectiveRDOQ = false, int thrVal = 8 );

public:

  void  setLambdas                ( const double lambdas[MAX_NUM_COMP] )   { for (uint32_t component = 0; component < MAX_NUM_COMP; component++) m_lambdas[component] = lambdas[component]; }
  void  selectLambda              ( const ComponentID compId )             { m_dLambda = m_lambdas[ compId ]; }
  void  getLambdas                ( double (&lambdas)[MAX_NUM_COMP]) const { for (uint32_t component = 0; component < MAX_NUM_COMP; component++) lambdas[component] = m_lambdas[component]; }
  void  scaleLambda               ( const double scale)                    { m_dLambda *= scale;}

  int*  getQuantCoeff             ( uint32_t list, int qp, uint32_t sizeX, uint32_t sizeY ) { return m_quantCoef            [sizeX][sizeY][list][qp]; };  //!< get Quant Coefficent
  int*  getDequantCoeff           ( uint32_t list, int qp, uint32_t sizeX, uint32_t sizeY ) { return m_dequantCoef          [sizeX][sizeY][list][qp]; };  //!< get DeQuant Coefficent

  bool  getUseScalingList         ( const uint32_t width, const uint32_t height, const bool isTransformSkip, const bool lfnstApplied )
  {
    return (m_scalingListEnabled && !isTransformSkip && (!lfnstApplied));
  };
  bool getScalingListEnabled      ()   { return m_scalingListEnabled; }
  virtual void setFlatScalingList ( const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths );

  // quantization
  virtual void quant              ( TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx& ctx );
  // de-quantization
  virtual void dequant            ( const TransformUnit& tu, CoeffBuf& dstCoeff, const ComponentID compID, const QpParam& cQP );

protected:
  bool    xNeedRDOQ               ( TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, const QpParam& cQP );
private:
  void    xInitScalingList        ( const Quant* other, bool useScalingLists );
  void    xDestroyScalingList     ();
  void    xSetFlatScalingList     ( uint32_t list, uint32_t sizeX, uint32_t sizeY, int qp );
  void    xSignBitHidingHDQ       ( TCoeffSig* pQCoef, const TCoeff* pCoef, TCoeff* deltaU, const CoeffCodingContext& cctx, const int maxLog2TrDynamicRange);
  void    ( *DeQuant)             (const int maxX,const int maxY,const int scale,const TCoeffSig*const piQCoef,const size_t piQCfStride,TCoeff   *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum);

#ifdef TARGET_SIMD_X86
  void initQuantX86();
  template <X86_VEXT vext>
  void _initQuantX86();
#endif

protected:
  int      m_RDOQ;
  bool     m_useRDOQTS;
  bool     m_useSelectiveRDOQ;
  double   m_dLambda;
  TCoeffSig m_tmpBdpcm[1 << ( MAX_TB_LOG2_SIZEY << 1 )];
  int      m_thrVal;

private:
  double   m_lambdas[MAX_NUM_COMP];
  bool     m_scalingListEnabled;
  bool     m_isScalingListOwner;

  int      *m_quantCoef            [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
  int      *m_dequantCoef          [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of dequantization matrix coefficient 4x4
};// END CLASS DEFINITION Quant

} // namespace vvenc

//! \}


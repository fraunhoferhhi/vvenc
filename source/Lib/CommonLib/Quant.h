/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVenc

(c) Copyright (2019-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVenc (“Fraunhofer Versatile Video Encoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Encoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Encoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Encoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Encoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Encoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Encoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Encoding Library. If You use the Fraunhofer Versatile Video Encoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Encoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Encoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Encoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Encoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
----------------------------------------------------------------------------- */
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
  Quant( const Quant* other );
  virtual ~Quant();

  // initialize class
  virtual void init( int rdoq = 0, bool useRDOQTS = false, bool useSelectiveRDOQ = false, int dqThrVal = 8 );

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
  virtual void setFlatScalingList ( const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths);

  // quantization
  virtual void quant              ( TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx& ctx );
  // de-quantization
  virtual void dequant            ( const TransformUnit& tu, CoeffBuf& dstCoeff, const ComponentID compID, const QpParam& cQP );

protected:
  bool    xNeedRDOQ               ( TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, const QpParam& cQP );
private:
  void    xInitScalingList        ( const Quant* other );
  void    xDestroyScalingList     ();
  void    xSetFlatScalingList     ( uint32_t list, uint32_t sizeX, uint32_t sizeY, int qp );
  void    xSignBitHidingHDQ       ( TCoeff* pQCoef, const TCoeff* pCoef, TCoeff* deltaU, const CoeffCodingContext& cctx, const int maxLog2TrDynamicRange);
  void ( *DeQuant) (const int maxX,const int maxY,const int scale,const TCoeff*const piQCoef,const size_t piQCfStride,TCoeff   *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum);

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

private:
  double   m_lambdas[MAX_NUM_COMP];
  bool     m_scalingListEnabled;
  bool     m_isScalingListOwner;

  int      *m_quantCoef            [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
  int      *m_dequantCoef          [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of dequantization matrix coefficient 4x4
};// END CLASS DEFINITION Quant

} // namespace vvenc

//! \}


/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software,
especially patent licenses, a separate Agreement needs to be closed. 
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */
/** \file     QuantRDOQ2.h
    \brief    RDOQ class (header)
*/

#pragma once

#include "CommonDef.h"
#include "Unit.h"
#include "Contexts.h"
#include "ContextModelling.h"
#include "QuantRDOQ.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

  typedef int64_t cost_t ;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// transform and quantization class
class QuantRDOQ2 : public QuantRDOQ
{
public:
  QuantRDOQ2( const Quant* other, bool useScalingLists );
  ~QuantRDOQ2();

public:
  virtual void setFlatScalingList      ( const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths );
  virtual void quant                   ( TransformUnit &tu, const ComponentID compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx );

private:
  int* xGetErrScaleCoeffSL             ( unsigned list, unsigned sizeX, unsigned sizeY, int qp ) { return m_errScale[sizeX][sizeY][list][qp]; };  //!< get Error Scale Coefficent
  int  xGetErrScaleCoeff               ( const bool needsSqrt2, SizeType width, SizeType height, int qp, const int maxLog2TrDynamicRange, const int channelBitDepth);
  int& xGetErrScaleCoeffNoScalingList  ( unsigned list, unsigned sizeX, unsigned sizeY, int qp ) { return m_errScaleNoScalingList[sizeX][sizeY][list][qp]; };  //!< get Error Scale Coefficent

  void xInitScalingList                ( const QuantRDOQ2* other );
  void xDestroyScalingList             ();
  void xSetErrScaleCoeff               ( unsigned list, unsigned sizeX, unsigned sizeY, int qp, const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths );
  void xSetErrScaleCoeffNoScalingList  ( unsigned list, unsigned wIdx, unsigned hIdx, int qp, const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths );
  void xInitLastPosBitsTab             ( const CoeffCodingContext& cctx, const unsigned uiWidth, const unsigned uiHeight, const ChannelType chType, const FracBitsAccess& fracBits );

  inline cost_t xiGetICost              ( int iRate ) const;
  inline cost_t xGetIEPRate             () const;
  inline cost_t xiGetICRateCost ( const unsigned     uiAbsLevel,
                                  const BinFracBits& fracBitsPar,
                                  const BinFracBits& fracBitsGt1,
                                  const BinFracBits& fracBitsGt2,
                                  const int          remRegBins,
                                  unsigned           goRiceZero,
                                  const uint16_t     ui16AbsGoRice,
                                  const bool         useLimitedPrefixLength,
                                  const int          maxLog2TrDynamicRange ) const;
  inline cost_t xiGetCostSigCoeffGroup  ( const BinFracBits& fracBitsSigCG, unsigned uiSignificanceCoeffGroup ) const;
  inline cost_t xLevelCost              ( const uint32_t uiAbsLevel, const int iScaledLevel, const int iQBits, const cost_t iErrScale, const cost_t iErrScaleShift, const cost_t costSig, const BinFracBits& fracBitsPar, const BinFracBits& fracBitsGt1, const BinFracBits& fracBitsGt2, const int remRegBins, unsigned goRiceZero, const uint16_t goRiceParam, const bool extendedPrecision, const int maxLog2TrDynamicRange ) const;
  inline cost_t xiGetCostLast           ( const unsigned uiPosX, const unsigned uiPosY, const ChannelType chType ) const;
  inline cost_t xiGetCostSigCoef        ( const BinFracBits& fracBitsSig, unsigned uiSignificance ) const;

  template< bool bSBH, bool bUseScalingList >
  int xRateDistOptQuantFast( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx &ctx );
  int xRateDistOptQuant    ( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx &ctx, bool bUseScalingList );


private:
  bool    m_isErrScaleListOwner;
  int64_t m_iLambda;

  //QuantErrScale m_quantErrScale;
  int     m_lastBitsX             [MAX_NUM_CH][LAST_SIGNIFICANT_GROUPS];
  int     m_lastBitsY             [MAX_NUM_CH][LAST_SIGNIFICANT_GROUPS];
  int*    m_errScale             [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
  int     m_errScaleNoScalingList[SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
};// END CLASS DEFINITION QuantRDOQ2

} // namespace vvenc
//! \}


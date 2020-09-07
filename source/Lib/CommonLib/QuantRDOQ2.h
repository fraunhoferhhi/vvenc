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
  QuantRDOQ2( const Quant* other );
  ~QuantRDOQ2();

public:
  virtual void setFlatScalingList      ( const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths );
  virtual void quant                   ( TransformUnit &tu, const ComponentID compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx );
#ifdef TARGET_SIMD_X86
  void initQuantX86();
  template <X86_VEXT vext>
  void _initQuantX86();
#endif

private:
  int  (*m_pQuantToNearestInt[4])      ( short* piQCoef, short* piCbf, const short* piCoef, const int* piQuantCoeff, const int iAdd, const int iShift, const int iSize );

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


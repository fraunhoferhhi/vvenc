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
/** \file     QuantRDOQ.h
    \brief    RDOQ class (header)
*/

#pragma once

#include "CommonDef.h"
#include "Unit.h"
#include "Contexts.h"
#include "ContextModelling.h"
#include "Quant.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// transform and quantization class
class QuantRDOQ : public Quant
{
public:
  QuantRDOQ( const Quant* other );
  ~QuantRDOQ();

public:
  void setFlatScalingList   ( const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths );
  // quantization
  void quant                ( TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx& ctx );
  void forwardRDPCM         ( TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx &ctx );
  void rateDistOptQuantTS   ( TransformUnit& tu, const ComponentID compID, const CCoeffBuf& coeffs, TCoeff &absSum, const QpParam& qp, const Ctx &ctx );

private:
  double* xGetErrScaleCoeffSL            ( uint32_t list, uint32_t sizeX, uint32_t sizeY, int qp ) { return m_errScale[sizeX][sizeY][list][qp]; };  //!< get Error Scale Coefficent
  double  xGetErrScaleCoeff              ( const bool needsSqrt2, SizeType width, SizeType height, int qp, const int maxLog2TrDynamicRange, const int channelBitDepth, bool bTransformSkip);
  double& xGetErrScaleCoeffNoScalingList ( uint32_t list, uint32_t sizeX, uint32_t sizeY, int qp ) { return m_errScaleNoScalingList[sizeX][sizeY][list][qp]; };  //!< get Error Scale Coefficent
  void    xInitScalingList               ( const QuantRDOQ* other );
  void    xDestroyScalingList            ();
  void    xSetErrScaleCoeff              ( uint32_t list, uint32_t sizeX, uint32_t sizeY, int qp, const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths );
  void    xDequantSample                 ( TCoeff& pRes, TCoeff& coeff, const TrQuantParams& trQuantParams );
  // RDOQ functions
  void    xRateDistOptQuant              ( TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx &ctx);

  inline uint32_t xGetCodedLevel( double&            rd64CodedCost,
                                  double&            rd64CodedCost0,
                                  double&            rd64CodedCostSig,
                                  Intermediate_Int   lLevelDouble,
                                  uint32_t           uiMaxAbsLevel,
                                  const BinFracBits* fracBitsSig,
                                  const BinFracBits& fracBitsPar,
                                  const BinFracBits& fracBitsGt1,
                                  const BinFracBits& fracBitsGt2,
                                  const int          remRegBins,
                                  unsigned           goRiceZero,
                                  uint16_t           ui16AbsGoRice,
                                  int                iQBits,
                                  double             errorScale,
                                  bool               bLast,
                                  bool               useLimitedPrefixLength,
                                  const int          maxLog2TrDynamicRange ) const;
  inline int xGetICRate     ( const uint32_t         uiAbsLevel,
                              const BinFracBits& fracBitsPar,
                              const BinFracBits& fracBitsGt1,
                              const BinFracBits& fracBitsGt2,
                              const int          remRegBins,
                              unsigned           goRiceZero,
                              const uint16_t       ui16AbsGoRice,
                              const bool         useLimitedPrefixLength,
                              const int          maxLog2TrDynamicRange  ) const;
  inline double xGetRateLast         ( const int* lastBitsX, const int* lastBitsY,
                                       unsigned        PosX, unsigned   PosY                              ) const;

  inline double xGetRateSigCoeffGroup( const BinFracBits& fracBitsSigCG,   unsigned uiSignificanceCoeffGroup ) const;

  inline double xGetRateSigCoef      ( const BinFracBits& fracBitsSig,     unsigned uiSignificance           ) const;

  inline double xGetICost            ( double dRate                                                      ) const;
  inline double xGetIEPRate          (                                                                   ) const;

  inline uint32_t xGetCodedLevelTSPred( double&             rd64CodedCost,
                                        double&             rd64CodedCost0,
                                        double&             rd64CodedCostSig,
                                        Intermediate_Int    levelDouble,
                                        int                 qBits,
                                        double              errorScale,
                                        uint32_t            coeffLevels[],
                                        double              coeffLevelError[],
                                        const BinFracBits*  fracBitsSig,
                                        const BinFracBits&  fracBitsPar,
                                        CoeffCodingContext& cctx,
                                        const FracBitsAccess& fracBitsAccess,
                                        const BinFracBits&  fracBitsSign,
                                        const BinFracBits&  fracBitsGt1,
                                        const uint8_t       sign,
                                        int                 rightPixel,
                                        int                 belowPixel,
                                        uint16_t            ricePar,
                                        bool                isLast,
                                        bool                useLimitedPrefixLength,
                                        const int           maxLog2TrDynamicRange,
                                        int&                numUsedCtxBins
                                      ) const;

  inline int xGetICRateTS   ( const uint32_t            absLevel,
                              const BinFracBits&        fracBitsPar,
                              const CoeffCodingContext& cctx,
                              const FracBitsAccess&     fracBitsAccess,
                              const BinFracBits&        fracBitsSign,
                              const BinFracBits&        fracBitsGt1,
                              int&                      numCtxBins,
                              const uint8_t             sign,
                              const uint16_t            ricePar,
                              const bool                useLimitedPrefixLength,
                              const int                 maxLog2TrDynamicRange  ) const;
private:
  bool    m_isErrScaleListOwner;

  double* m_errScale              [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
  double  m_errScaleNoScalingList [SCALING_LIST_SIZE_NUM][SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM][SCALING_LIST_REM_NUM]; ///< array of quantization matrix coefficient 4x4
  // temporary buffers for RDOQ
  double  m_pdCostCoeff           [MAX_TB_SIZEY * MAX_TB_SIZEY];
  double  m_pdCostSig             [MAX_TB_SIZEY * MAX_TB_SIZEY];
  double  m_pdCostCoeff0          [MAX_TB_SIZEY * MAX_TB_SIZEY];
  double  m_pdCostCoeffGroupSig   [(MAX_TB_SIZEY * MAX_TB_SIZEY) >> MLS_CG_SIZE]; // even if CG size is 2 (if one of the sides is 2) instead of 4, there should be enough space
  int     m_rateIncUp             [MAX_TB_SIZEY * MAX_TB_SIZEY];
  int     m_rateIncDown           [MAX_TB_SIZEY * MAX_TB_SIZEY];
  int     m_sigRateDelta          [MAX_TB_SIZEY * MAX_TB_SIZEY];
  TCoeff  m_deltaU                [MAX_TB_SIZEY * MAX_TB_SIZEY];
  TCoeff  m_fullCoeff             [MAX_TB_SIZEY * MAX_TB_SIZEY];
  int     m_bdpcm;
  int     m_testedLevels;
};// END CLASS DEFINITION QuantRDOQ

} // namespace vvenc

//! \}


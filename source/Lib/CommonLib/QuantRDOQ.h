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
  QuantRDOQ( const Quant* other, bool useScalingLists );
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
  void    xDequantSample                 ( TCoeff& pRes, TCoeffSig& coeff, const TrQuantParams& trQuantParams );
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


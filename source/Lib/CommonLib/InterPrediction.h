/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     InterPrediction.h
    \brief    inter prediction class (header)
*/

#pragma once

#include "InterpolationFilter.h"
#include "Unit.h"
#include "Picture.h"
#include "RdCost.h"
#include "ContextModelling.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if ENABLE_SIMD_OPT_BDOF && defined( TARGET_SIMD_X86 )
using namespace x86_simd;
#endif
#if ENABLE_SIMD_OPT_BDOF && defined( TARGET_SIMD_ARM )
using namespace arm_simd;
#endif

// forward declaration
class Mv;


// ====================================================================================================================
// Class definition
// ====================================================================================================================
class InterPredInterpolation
{
  Pel*                 m_gradX0;
  Pel*                 m_gradY0;
  Pel*                 m_gradX1;
  Pel*                 m_gradY1;
  Pel                  m_gradBuf[2][(AFFINE_MIN_BLOCK_SIZE + 2) * (AFFINE_MIN_BLOCK_SIZE + 2)];
  int                  m_dMvBuf[2][16 * 2];
  Mv*                  m_storedMv;

protected:
  bool                 m_skipPROF;
  bool                 m_encOnly;
  bool                 m_isBi;
  InterpolationFilter  m_if;
  Pel*                 m_filteredBlock        [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][MAX_NUM_COMP];
  Pel*                 m_filteredBlockTmp     [LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL][MAX_NUM_COMP];
  int                  m_ifpLines;

  void xApplyBDOF             ( PelBuf& yuvDst, const ClpRng& clpRng );

public:
  void(*xFpBiDirOptFlow)      ( const Pel* srcY0, const Pel* srcY1, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel* gradY1, const int width, const int height, Pel* dstY, const ptrdiff_t dstStride, const int shiftNum, const int  offset, const int  limit, const ClpRng& clpRng, const int bitDepth ) = nullptr;
  void(*xFpBDOFGradFilter)    ( const Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, const int bitDepth );
  void(*xFpProfGradFilter)    ( const Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, const int bitDepth );
  void(*xFpApplyPROF)         ( Pel* dst, int dstStride, const Pel* src, int srcStride, int width, int height, const Pel* gradX, const Pel* gradY, int gradStride, const int* dMvX, const int* dMvY, int dMvStride, const bool& bi, int shiftNum, Pel offset, const ClpRng& clpRng );
  void(*xFpPadDmvr)           ( const Pel* src, const int srcStride, Pel* dst, const int dstStride, int width, int height, int padSize );

protected:
#if ENABLE_SIMD_OPT_BDOF && defined( TARGET_SIMD_X86 )
  void initInterPredictionX86();
  template <X86_VEXT vext>
  void _initInterPredictionX86();
#endif

#if ENABLE_SIMD_OPT_BDOF && defined( TARGET_SIMD_ARM )
  void initInterPredictionARM();
  template <ARM_VEXT vext>
  void _initInterPredictionARM();
#endif

  void xWeightedAverage       ( const CodingUnit& cu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const bool bdofApplied, PelUnitBuf *yuvPredTmp = NULL );
  void xPredAffineBlk         ( const ComponentID compID, const CodingUnit& cu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool bi, const ClpRng& clpRng, const RefPicList refPicList = REF_PIC_LIST_X);
  void xPredInterBlk( const ComponentID compID, const CodingUnit& cu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool bi, const ClpRng& clpRng
                              , const bool bdofApplied
                              , const bool isIBC
                              , const RefPicList refPicList = REF_PIC_LIST_X
                              , const SizeType dmvrWidth = 0
                              , const SizeType dmvrHeight = 0
                              , const bool bilinearMC = false
                              , const Pel* srcPadBuf = NULL
                              , const int32_t srcPadStride = 0
                              );

public:
  InterPredInterpolation();
  virtual ~InterPredInterpolation();
  void destroy();
  void init( bool enableOpt = true );

  void    weightedGeoBlk  ( const ClpRngs &clpRngs, CodingUnit& cu, const uint8_t splitDir, int32_t channel,
                            PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1);

  static bool isSubblockVectorSpreadOverLimit(int a, int b, int c, int d, int predType);
  bool xIsAffineMvInRangeFPP (const CodingUnit& cu, const Mv* _mv, const int ifpLines, const int mvPrecShift = MV_FRACTIONAL_BITS_INTERNAL);
};

class DMVR : public InterPredInterpolation
{
  RdCost*                 m_pcRdCost;

  PelStorage              m_yuvPred[NUM_REF_PIC_LIST_01];
  PelStorage              m_yuvTmp[NUM_REF_PIC_LIST_01];
  PelStorage              m_yuvPad[NUM_REF_PIC_LIST_01];
  const Mv m_pSearchOffset[25] = { Mv(-2,-2), Mv(-1,-2), Mv(0,-2), Mv(1,-2), Mv(2,-2),
                                   Mv(-2,-1), Mv(-1,-1), Mv(0,-1), Mv(1,-1), Mv(2,-1),
                                   Mv(-2, 0), Mv(-1, 0), Mv(0, 0), Mv(1, 0), Mv(2, 0),
                                   Mv(-2, 1), Mv(-1, 1), Mv(0, 1), Mv(1, 1), Mv(2, 1),
                                   Mv(-2, 2), Mv(-1, 2), Mv(0, 2), Mv(1, 2), Mv(2, 2) };
private:
  void     xCopyAndPad          ( const CodingUnit& cu, PelUnitBuf& pcPad, RefPicList refId, bool forLuma);
  void     xFinalPaddedMCForDMVR( const CodingUnit& cu, PelUnitBuf* dstBuf, const PelUnitBuf *refBuf, const bool bioApplied, const Mv startMV[NUM_REF_PIC_LIST_01], const Mv& refMV );

protected:
  DMVR();
  virtual ~DMVR();
  void destroy();
  void init                    ( RdCost* pcRdCost, const ChromaFormat chFormat );
  void xProcessDMVR            ( const CodingUnit& cu, PelUnitBuf& pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied );
};

class InterPrediction : public DMVR
{
protected:
  ChromaFormat m_currChromaFormat;

private:
  PelStorage   m_yuvPred[NUM_REF_PIC_LIST_01];
  bool         m_subPuMC;
  PelStorage   m_geoPartBuf[2]; 
  int          m_IBCBufferWidth;
  PelStorage   m_IBCBuffer;
  void xIntraBlockCopyIBC       ( CodingUnit& cu, PelUnitBuf& predBuf, const ComponentID compID );

  void xPredInterUni            ( const CodingUnit& cu, const RefPicList& refPicList, PelUnitBuf& pcYuvPred, const bool bi, const bool bdofApplied );
  void xPredInterBi             ( const CodingUnit& cu, PelUnitBuf& yuvPred, const bool bdofApplied = false, PelUnitBuf *yuvPredTmp = NULL );
  void xSubPuBDOF               ( const CodingUnit& cu, PelUnitBuf& predBuf, const RefPicList& refPicList = REF_PIC_LIST_X );
  bool xCheckIdenticalMotion    ( const CodingUnit& cu ) const;

public:
  InterPrediction();
  virtual ~InterPrediction();

  void    init                  ( RdCost* pcRdCost, ChromaFormat chromaFormatIDC, const int ctuSize, const int ifpLines = 0 );
  void    destroy               ();

  // inter
  bool    motionCompensation    ( CodingUnit& cu, PelUnitBuf& predBuf, const RefPicList& eRefPicList = REF_PIC_LIST_X, PelUnitBuf* predBufDfltWght = NULL );
  void    motionCompensationIBC ( CodingUnit& cu, PelUnitBuf& predBuf );
  void    xSubPuMC              ( CodingUnit& cu, PelUnitBuf& predBuf, const RefPicList& eRefPicList = REF_PIC_LIST_X );
  void    motionCompensationGeo ( CodingUnit& cu, PelUnitBuf& predBuf, const MergeCtx& geoMrgCtx );
  void    xFillIBCBuffer        ( CodingUnit& cu);
  void    resetIBCBuffer        ( const ChromaFormat chromaFormatIDC, const int ctuSize);
  void    resetVPDUforIBC       ( const ChromaFormat chromaFormatIDC, const int ctuSize, const int vSize, const int xPos, const int yPos);
  bool    isLumaBvValidIBC      ( const int ctuSize, const int xCb, const int yCb, const int width, const int height, const int xBv, const int yBv);
};

} // namespace vvenc

//! \}


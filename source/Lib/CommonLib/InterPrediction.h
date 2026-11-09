/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
  // the whole BDOF of a 16-wide block in one pass (AVX2; nullptr: xApplyBDOF's gradient filter, padding and
  // xFpBiDirOptFlow): pred0 / pred1 point at the block's first sample, inside its unpadded one-sample border
  void(*xFpBDOF16)            ( const Pel* pred0, const Pel* pred1, const ptrdiff_t predStride, const int height, Pel* dst, const ptrdiff_t dstStride, const int shiftNum, const int offset, const int limit, const ClpRng& clpRng ) = nullptr;

  // The luma prediction of one affine CU, everything xPredAffineBlk's sub-block loop needs for it (its MV spread within
  // the limit: the other CUs, whose sub-blocks all take one MV, are predicted in one filter call before)
  struct AffineLumaCU
  {
    const Pel*  ref;                          // reference picture at the CU's position
    int         refStride;
    Pel*        dst;                          // prediction of the CU
    int         dstStride;
    int         width, height;                // CU size, powers of 2 from 8
    int         mvScaleHor, mvScaleVer;       // affine model: MV at the top-left corner, << MAX_CU_DEPTH
    int         dMvHorX, dMvHorY;             // its change per sample to the right
    int         dMvVerX, dMvVerY;             // and per sample down
    int         roundShift;                   // to 1/16 sample, for roundAffineMv
    int         horMin, horMax, verMin, verMax;  // window the MVs are clipped to for the reference fetch
    Mv*         storedMv;                     // sub-block MVs, kept for the chroma pass
    int         storedMvStride;
    const int16_t* dMvXY;                     // PROF: the CU's (dMvX, dMvY) pairs, interleaved
    int         shiftNum;                     // PROF: border shift and output shift (they are equal)
    Pel         offset;                       // PROF: output offset
    ClpRng      clpRng;
  };
  // The per-sub-block loop of xPredAffineBlk for luma in one call, by [PROF][bi]: same MVs and m_storedMv writes,
  // same samples as m_if's 4x4 filters and the PROF functions. nullptr where no SIMD version exists.
  void(*xFpAffineLumaCU[2][2])( const AffineLumaCU& cu ) = {};

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

  void xWeightedAverage       ( const CodingUnit& cu, const PelUnitBuf& pcYuvSrc0, const PelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const bool bdofApplied, PelUnitBuf *yuvPredTmp = NULL );
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

// The MV of one luma sub-block, as xPredAffineBlk derives it: the affine model at the sub-block centre, rounded to
// 1/16 sample and to the MV storage range. It is stored for the chroma pass, which reads the luma MVs. (A CU whose MV
// spread is over the limit, where every sub-block takes the MV at the CU centre, is predicted before the CU kernel.)
static inline Mv affineLumaSubblockMv( const InterPredInterpolation::AffineLumaCU& cu, int w, int h )
{
  const int half = AFFINE_MIN_BLOCK_SIZE >> 1;
  const int x    = half + w;
  const int y    = half + h;
  int mvHor      = cu.mvScaleHor + cu.dMvHorX * x + cu.dMvVerX * y;
  int mvVer      = cu.mvScaleVer + cu.dMvHorY * x + cu.dMvVerY * y;
  roundAffineMv( mvHor, mvVer, cu.roundShift );
  Mv mv( mvHor, mvVer );
  mv.clipToStorageBitDepth();
  cu.storedMv[( h / AFFINE_MIN_BLOCK_SIZE ) * cu.storedMvStride + w / AFFINE_MIN_BLOCK_SIZE] = mv;
  return mv;
}

// Where one luma sub-block of the CU loop reads the reference: its MV, clipped to the reference window, split into the
// integer position (ref) and the 1/16 fractions; nearest is the nearest integer position, which PROF takes its border
// from.
struct AffineLumaSubblockRef
{
  const Pel* ref;
  const Pel* nearest;
  int        xFrac, yFrac;
};

static ALWAYS_INLINE AffineLumaSubblockRef affineLumaSubblockRef( const InterPredInterpolation::AffineLumaCU& cu, int w, int h )
{
  const Mv  mv    = affineLumaSubblockMv( cu, w, h );
  const int mvHor = std::min<int>( cu.horMax, std::max<int>( cu.horMin, mv.hor ) );
  const int mvVer = std::min<int>( cu.verMax, std::max<int>( cu.verMin, mv.ver ) );
  AffineLumaSubblockRef r;
  r.xFrac   = mvHor & 15;
  r.yFrac   = mvVer & 15;
  r.ref     = cu.ref + ( h + ( mvVer >> 4 ) ) * cu.refStride + w + ( mvHor >> 4 );
  r.nearest = r.ref + ( r.yFrac >> 3 ) * cu.refStride + ( r.xFrac >> 3 );
  return r;
}

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


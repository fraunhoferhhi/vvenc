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
/** \file     IntraPrediction.h
    \brief    prediction class (header)
*/

#pragma once

#include "Unit.h"
#include "Picture.h"
#include "MatrixIntraPrediction.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if ENABLE_SIMD_OPT_INTRAPRED && defined( TARGET_SIMD_X86 )
using namespace x86_simd;
#endif

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
enum PredBuf
{
  PRED_BUF_UNFILTERED = 0,
  PRED_BUF_FILTERED   = 1,
  NUM_PRED_BUF        = 2
};

static const uint32_t MAX_INTRA_FILTER_DEPTHS=8;

class IntraPrediction
{
private:
  Pel         m_refBuffer[MAX_NUM_COMP][NUM_PRED_BUF][(MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX) * 2];
  uint32_t    m_refBufferStride[MAX_NUM_COMP];
  int         m_numIntraNeighbor;
  bool        m_neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  Area        m_lastArea;
  ChannelType m_lastCh;

  static const uint8_t m_aucIntraFilter[MAX_INTRA_FILTER_DEPTHS];

  struct IntraPredParam //parameters of Intra Prediction
  {
    bool refFilterFlag;
    bool applyPDPC;
    bool isModeVer;
    int  multiRefIndex;
    int  intraPredAngle;
    int  absInvAngle;
    bool interpolationFlag;
    int  angularScale;

    IntraPredParam()
      : refFilterFlag     (false)
      , applyPDPC         (false)
      , isModeVer         (false)
      , multiRefIndex     (-1)
      , intraPredAngle    (std::numeric_limits<int>::max())
      , absInvAngle       (std::numeric_limits<int>::max())
      , interpolationFlag (false)
      , angularScale      (-1)
    {
    }
  };

  IntraPredParam        m_ipaParam;
  Pel*                  m_pMdlmTemp; // for MDLM mode
  MatrixIntraPrediction m_matrixIntraPred;

protected:
  ChromaFormat          m_currChromaFormat;

  int                   m_topRefLength;
  int                   m_leftRefLength;
  void setReferenceArrayLengths(const CompArea& area);

private:
  static bool isIntegerSlope      ( const int absAng) { return (0 == (absAng & 0x1F)); }
  static int getWideAngle         ( int width, int height, int predMode );

  // prediction
  void xPredIntraDc               ( PelBuf& pDst, const CPelBuf& pSrc );
  void xPredIntraAng              ( PelBuf& pDst, const CPelBuf& pSrc, const ChannelType channelType, const ClpRng& clpRng);
  Pel  xGetPredValDc              ( const CPelBuf& pSrc, const Size& dstSize );
  void xPredIntraBDPCM            ( PelBuf& pDst, const CPelBuf& pSrc, const uint32_t dirMode, const ClpRng& clpRng );

  void xFillReferenceSamples      ( const CPelBuf& recoBuf,      Pel* refBufUnfiltered, const CompArea& area, const CodingUnit &cu );
  void xFilterReferenceSamples    ( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea& area, const SPS &sps, int multiRefIdx, int predStride = 0 );
  void xGetLMParameters(const CodingUnit& cu, const ComponentID compID, const CompArea& chromaArea, int& a, int& b, int& iShift);

#if ENABLE_SIMD_OPT_INTRAPRED && defined( TARGET_SIMD_X86 )
  void initIntraPredictionX86();
  template <X86_VEXT vext>
  void _initIntraPredictionX86();
#endif

#if ENABLE_SIMD_OPT_INTRAPRED && defined( TARGET_SIMD_ARM )
  void initIntraPredictionARM();
  template <ARM_VEXT vext>
  void _initIntraPredictionARM();
#endif

public:
  IntraPrediction( bool enableOpt = true );
  virtual ~IntraPrediction();

  void init                   ( ChromaFormat chromaFormatIDC, const unsigned bitDepthY);
  void reset                  ();
  void destroy                ();

  void initPredIntraParams    ( const CodingUnit& cu,  const CompArea compArea, const SPS& sps );

  // Angular Intra
  void predIntraAng           ( const ComponentID compId, PelBuf& piPred, const CodingUnit& cu);
  Pel* getPredictorPtr        ( const ComponentID compId ) { return m_refBuffer[compId][m_ipaParam.refFilterFlag ? PRED_BUF_FILTERED : PRED_BUF_UNFILTERED]; }

  // Cross-component Chroma
  void predIntraChromaLM      ( const ComponentID compID, PelBuf& piPred, const CodingUnit& cu, const CompArea& chromaArea, int intraDir);
  void loadLMLumaRecPels      ( const CodingUnit& cu, const CompArea& chromaArea );
  /// set parameters from CU data for accessing intra data
  void initIntraPatternChType ( const CodingUnit &cu, const CompArea& area, const bool forceRefFilterFlag = false); // use forceRefFilterFlag to get both filtered and unfiltered buffers
  void initIntraPatternChTypeISP( const CodingUnit& cu, const CompArea& area, PelBuf& piReco, const bool forceRefFilterFlag = false );

  // Matrix-based intra prediction
  void initIntraMip           ( const CodingUnit& cu);
  void predIntraMip           ( PelBuf &piPred, const CodingUnit& cu);

  int getNumIntraCiip         ( const CodingUnit& cu )
  {
    const Position posBL = cu.Y().bottomLeft();
    const Position posTR = cu.Y().topRight();
    const CodingUnit *neigh0 = cu.cs->getCURestricted(posBL.offset(-1, 0), cu, CH_L);
    const CodingUnit *neigh1 = cu.cs->getCURestricted(posTR.offset(0, -1), cu, CH_L);

    int numIntra = 0;
    numIntra += (neigh0 && (neigh0->predMode == MODE_INTRA))?1:0;
    numIntra += (neigh1 && (neigh1->predMode == MODE_INTRA))?1:0;

    return numIntra;
  }

  void ( *IntraPredAngleLuma )    ( Pel* pDstBuf, const ptrdiff_t dstStride, Pel* refMain, int width, int height, int deltaPos, int intraPredAngle, const TFilterCoeff* ff, const bool useCubicFilter, const ClpRng& clpRng );
  void ( *IntraPredAngleChroma )  ( Pel* pDst, const ptrdiff_t dstStride, Pel* pBorder, int width, int height, int deltaPos, int intraPredAngle );
  void ( *IntraAnglePDPC )        ( Pel* pDsty, const int dstStride, Pel* refSide, const int width, const int height, int scale, int invAngle );
  void ( *IntraHorVerPDPC )       ( Pel* pDsty, const int dstStride, Pel* refSide, const int width, const int height, int scale, const Pel* refMain, const ClpRng& clpRng );
  void ( *IntraPredSampleFilter ) ( PelBuf& piPred, const CPelBuf& pSrc );
  void ( *xPredIntraPlanar )      ( PelBuf& pDst, const CPelBuf& pSrc );
};

} // namespace vvenc

//! \}


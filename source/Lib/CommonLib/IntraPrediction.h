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

#if ISP_VVC
  int                   m_topRefLength;
  int                   m_leftRefLength;
  void setReferenceArrayLengths(const CompArea& area);
#endif

private:
  static bool isIntegerSlope      ( const int absAng) { return (0 == (absAng & 0x1F)); }
  static int getWideAngle         ( int width, int height, int predMode );

  // prediction
  void (*xPredIntraPlanar)        ( PelBuf& pDst, const CPelBuf& pSrc );
  void xPredIntraDc               ( PelBuf& pDst, const CPelBuf& pSrc );
  void xPredIntraAng              ( PelBuf& pDst, const CPelBuf& pSrc, const ChannelType channelType, const ClpRng& clpRng);
  Pel  xGetPredValDc              ( const CPelBuf& pSrc, const Size& dstSize );

  void xFillReferenceSamples      ( const CPelBuf& recoBuf,      Pel* refBufUnfiltered, const CompArea& area, const CodingUnit &cu );
  void xFilterReferenceSamples    ( const Pel* refBufUnfiltered, Pel* refBufFiltered, const CompArea& area, const SPS &sps, int multiRefIdx, int predStride = 0 );
  void xGetLMParameters(const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, int& a, int& b, int& iShift);

  void ( *IntraPredAngleLuma )    ( Pel* pDstBuf, const ptrdiff_t dstStride, Pel* refMain, int width, int height, int deltaPos, int intraPredAngle, const TFilterCoeff *ff, const bool useCubicFilter, const ClpRng& clpRng);
  void ( *IntraPredAngleChroma )  ( Pel* pDst,    const ptrdiff_t dstStride, Pel* pBorder, int width, int height, int deltaPos, int intraPredAngle);
  void ( *IntraAnglePDPC )        ( Pel* pDsty, const int dstStride, Pel* refSide, const int width, const int height, int scale, int invAngle);
  void ( *IntraHorVerPDPC )       ( Pel* pDsty, const int dstStride, Pel* refSide, const int width, const int height, int scale, const Pel* refMain, const ClpRng& clpRng);
  void ( *IntraPredSampleFilter)  ( PelBuf& piPred, const CPelBuf& pSrc );

#if ENABLE_SIMD_OPT_INTRAPRED
  void initIntraPredictionX86();
  template <X86_VEXT vext>
  void _initIntraPredictionX86();
#endif

public:
  IntraPrediction();
  virtual ~IntraPrediction();

  void init                   ( ChromaFormat chromaFormatIDC, const unsigned bitDepthY);
  void reset                  ();
  void destroy                ();

  void initPredIntraParams    ( const PredictionUnit & pu,  const CompArea compArea, const SPS& sps );

  // Angular Intra
  void predIntraAng           ( const ComponentID compId, PelBuf& piPred, const PredictionUnit &pu);
  Pel* getPredictorPtr        ( const ComponentID compId ) { return m_refBuffer[compId][m_ipaParam.refFilterFlag ? PRED_BUF_FILTERED : PRED_BUF_UNFILTERED]; }

  // Cross-component Chroma
  void predIntraChromaLM      ( const ComponentID compID, PelBuf& piPred, const PredictionUnit &pu, const CompArea& chromaArea, int intraDir);
  void loadLMLumaRecPels      ( const PredictionUnit &pu, const CompArea& chromaArea );
  /// set parameters from CU data for accessing intra data
  void initIntraPatternChType ( const CodingUnit &cu, const CompArea& area, const bool forceRefFilterFlag = false); // use forceRefFilterFlag to get both filtered and unfiltered buffers
#if ISP_VVC
  void initIntraPatternChTypeISP( const CodingUnit& cu, const CompArea& area, PelBuf& piReco, const bool forceRefFilterFlag = false );
#endif

  // Matrix-based intra prediction
  void initIntraMip           ( const PredictionUnit &pu);
  void predIntraMip           ( PelBuf &piPred, const PredictionUnit &pu);

  int getNumIntraCiip         ( const PredictionUnit& pu )
  {
    const Position posBL = pu.Y().bottomLeft();
    const Position posTR = pu.Y().topRight();
    const PredictionUnit *neigh0 = pu.cs->getPURestricted(posBL.offset(-1, 0), pu, CH_L);
    const PredictionUnit *neigh1 = pu.cs->getPURestricted(posTR.offset(0, -1), pu, CH_L);

    int numIntra = 0;
    numIntra += (neigh0 && (neigh0->cu->predMode == MODE_INTRA))?1:0;
    numIntra += (neigh1 && (neigh1->cu->predMode == MODE_INTRA))?1:0;

    return numIntra;
  }

};

} // namespace vvenc

//! \}


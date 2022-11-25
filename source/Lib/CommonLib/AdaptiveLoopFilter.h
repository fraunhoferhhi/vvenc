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
/** \file     AdaptiveLoopFilter.h
    \brief    adaptive loop filter class (header)
*/

#pragma once

#include "CommonDef.h"

#include "Unit.h"
#include "UnitTools.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

struct AlfClassifier
{
  AlfClassifier() {}
  AlfClassifier( uint8_t cIdx, uint8_t tIdx )
    : classIdx( cIdx ), transposeIdx( tIdx )
  {
  }

  uint8_t classIdx;
  uint8_t transposeIdx;
};

enum Direction
{
  HOR,
  VER,
  DIAG0,
  DIAG1,
  NUM_DIRECTIONS
};

class AdaptiveLoopFilter
{
public:
  static inline int clipALF(const int clip, const short ref, const short val0, const short val1)
  {
    return Clip3<int>(-clip, +clip, val0-ref) + Clip3<int>(-clip, +clip, val1-ref);
  }

  static inline int clipALF(const int clip, const int val0, const int val1)
  {
    return Clip3<int>(-clip, +clip, val0) + Clip3<int>(-clip, +clip, val1);
  }

  static constexpr int AlfNumClippingValues      = 4;
  static constexpr int MaxAlfNumClippingValues   = 4;
  static constexpr int m_NUM_BITS                = 8;
  static constexpr int m_CLASSIFICATION_BLK_SIZE = 128;  //non-normative, local buffer size
  static constexpr int m_ALF_UNUSED_CLASSIDX     = 255;
  static constexpr int m_ALF_UNUSED_TRANSPOSIDX  = 255;

  AdaptiveLoopFilter();
  virtual ~AdaptiveLoopFilter() {}
  void        reconstructCoeffAPSs    ( CodingStructure& cs, bool luma, bool chroma, bool isRdo);
  void        reconstructCoeff        ( AlfParam& alfParam, ChannelType channel, const bool isRdo, const bool isRedo = false);
  void        ALFProcess              ( CodingStructure& cs);
  void        create                  ( const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CH] );
  void        destroy                 ();
  static void deriveClassificationBlk ( AlfClassifier *classifier,
                                       const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, const int shift,
                                       const int vbCTUHeight, int vbPos);
  void        deriveClassification    ( AlfClassifier* classifier, const CPelBuf& srcLuma, const Area& blkDst, const Area& blk );
  template<AlfFilterType filtTypeCcAlf>
  static void filterBlkCcAlf         ( const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc,
                                       const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs,
                                       CodingStructure &cs, int vbCTUHeight, int vbPos);

  template<AlfFilterType filtType>
  static void filterBlk              ( const AlfClassifier *classifier, const PelUnitBuf& recDst, const CPelUnitBuf& recSrc,
                                       const Area& blkDst, const Area& blk, const ComponentID compId, const short *filterSet,
                                       const short *fClipSet, const ClpRng &clpRng, const CodingStructure &cs, const int vbCTUHeight,
                                       int vbPos);
  void (*m_deriveClassificationBlk) ( AlfClassifier *classifier, const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, const int shift, const int vbCTUHeight, int vbPos);
  void (*m_filterCcAlf)             ( const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc,
                                      const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs,
                                      CodingStructure &cs, int vbCTUHeight, int vbPos);
  void applyCcAlfFilterCTU          ( CodingStructure& cs, ComponentID compID, int ctuRsAddr );
  CcAlfFilterParam &getCcAlfFilterParam() { return m_ccAlfFilterParam; }
  uint8_t* getCcAlfControlIdc       ( const ComponentID compID)   { return m_ccAlfFilterControl[compID-1]; }
  void (*m_filter5x5Blk[2])         ( const AlfClassifier *classifier, const PelUnitBuf& recDst, const CPelUnitBuf& recSrc,
                                     const Area& blkDst, const Area& blk, const ComponentID compId, const short *filterSet,
                                     const short *fClipSet, const ClpRng &clpRng, const CodingStructure &cs, const int vbCTUHeight,
                                     int vbPos);
  void (*m_filter7x7Blk[2])        ( const AlfClassifier *classifier, const PelUnitBuf& recDst, const CPelUnitBuf& recSrc,
                                     const Area& blkDst, const Area& blk, const ComponentID compId, const short *filterSet,
                                     const short *fClipSet, const ClpRng &clpRng, const CodingStructure &cs, const int vbCTUHeight,
                                     int vbPos);

#ifdef TARGET_SIMD_X86
  void initAdaptiveLoopFilterX86();
  template <X86_VEXT vext>
  void _initAdaptiveLoopFilterX86();
#endif

protected:
  bool isCrossedByVirtualBoundaries( const CodingStructure& cs, const int xPos, const int yPos, const int width, const int height, bool& clipTop, bool& clipBottom, bool& clipLeft, bool& clipRight, int& numHorVirBndry, int& numVerVirBndry, int horVirBndryPos[], int verVirBndryPos[], int& rasterSliceAlfPad );
 
  static const int             m_classToFilterMapping[NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES];
  static const int             m_fixedFilterSetCoeff[ALF_FIXED_FILTER_NUM][MAX_NUM_ALF_LUMA_COEFF];
  short                        m_fixedFilterSetCoeffDec[NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short                        m_coeffApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
  short                        m_clippApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
  short                        m_clipDefault[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  bool                         m_created = false;
  short                        m_chromaCoeffFinal[VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_LUMA_COEFF];
  AlfParam*                    m_alfParamChroma;
  Pel                          m_alfClippingValues[MAX_NUM_CH][MaxAlfNumClippingValues];
  std::vector<AlfFilterShape>  m_filterShapesCcAlf[2];
  std::vector<AlfFilterShape>  m_filterShapes[MAX_NUM_CH];
  AlfClassifier*               m_classifier;
  short                        m_coeffFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short                        m_clippFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short                        m_chromaClippFinal[VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_LUMA_COEFF];
  uint8_t*                     m_ctuEnableFlag[MAX_NUM_COMP];
  uint8_t*                     m_ctuAlternative[MAX_NUM_COMP];
  PelStorage                   m_tempBuf;
  PelStorage                   m_tempBuf2;
  int                          m_inputBitDepth[MAX_NUM_CH];
  int                          m_picWidth;
  int                          m_picHeight;
  int                          m_maxCUWidth;
  int                          m_maxCUHeight;
  int                          m_maxCUDepth;
  uint32_t                     m_numCTUsInWidth;
  uint32_t                     m_numCTUsInHeight;
  uint32_t                     m_numCTUsInPic;
  int                          m_alfVBLumaPos;
  int                          m_alfVBChmaPos;
  int                          m_alfVBLumaCTUHeight;
  int                          m_alfVBChmaCTUHeight;
  ChromaFormat                 m_chromaFormat;

public :
  static constexpr int   m_scaleBits = 7; // 8-bits
  CcAlfFilterParam       m_ccAlfFilterParam;
  uint8_t*               m_ccAlfFilterControl[2];

};

} // namespace vvenc

//! \}

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
  static void deriveClassificationBlk ( AlfClassifier *classifier, int **laplacian[NUM_DIRECTIONS],
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
  void (*m_deriveClassificationBlk) ( AlfClassifier *classifier, int **laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma,
                                      const Area& blkDst, const Area& blk, const int shift, const int vbCTUHeight,
                                      int vbPos);
  void (*m_filterCcAlf)             ( const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc,
                                      const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs,
                                      CodingStructure &cs, int vbCTUHeight, int vbPos);
  void applyCcAlfFilter             ( CodingStructure &cs, ComponentID compID, const PelBuf &dstBuf, const PelUnitBuf &recYuvExt,
                                      uint8_t *   filterControl,
                                      const short filterSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF],
                                      const int   selectedFilterIdx);
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
  short                        m_chromaCoeffFinal[MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_LUMA_COEFF];
  AlfParam*                    m_alfParamChroma;
  Pel                          m_alfClippingValues[MAX_NUM_CH][MaxAlfNumClippingValues];
  std::vector<AlfFilterShape>  m_filterShapesCcAlf[2];
  std::vector<AlfFilterShape>  m_filterShapes[MAX_NUM_CH];
  AlfClassifier*               m_classifier;
  short                        m_coeffFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short                        m_clippFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short                        m_chromaClippFinal[MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_LUMA_COEFF];
  int**                        m_laplacian[NUM_DIRECTIONS];
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
  int                          m_numCTUsInWidth;
  int                          m_numCTUsInHeight;
  int                          m_numCTUsInPic;
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

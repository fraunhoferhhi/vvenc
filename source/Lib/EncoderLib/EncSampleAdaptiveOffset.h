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
/**
 \file     EncSampleAdaptiveOffset.h
 \brief    estimation part of sample adaptive offset class (header)
 */

#pragma once

#include "CABACWriter.h"
#include "CommonLib/SampleAdaptiveOffset.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

class EncCfg;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct SAOStatData //data structure for SAO statistics
{
  int64_t diff[MAX_NUM_SAO_CLASSES];
  int64_t count[MAX_NUM_SAO_CLASSES];

  SAOStatData(){}
  ~SAOStatData(){}
  void reset()
  {
    ::memset(diff, 0, sizeof(int64_t)*MAX_NUM_SAO_CLASSES);
    ::memset(count, 0, sizeof(int64_t)*MAX_NUM_SAO_CLASSES);
  }
  const SAOStatData& operator=(const SAOStatData& src)
  {
    ::memcpy(diff, src.diff, sizeof(int64_t)*MAX_NUM_SAO_CLASSES);
    ::memcpy(count, src.count, sizeof(int64_t)*MAX_NUM_SAO_CLASSES);
    return *this;
  }
  const SAOStatData& operator+= (const SAOStatData& src)
  {
    for(int i=0; i< MAX_NUM_SAO_CLASSES; i++)
    {
      diff[i] += src.diff[i];
      count[i] += src.count[i];
    }
    return *this;
  }
};

class EncSampleAdaptiveOffset : public SampleAdaptiveOffset
{
public:
  EncSampleAdaptiveOffset();
  virtual ~EncSampleAdaptiveOffset();

  //interface
  void init                  ( const EncCfg& encCfg );
  void initSlice             ( const Slice* slice );
  void setCtuEncRsrc         ( CABACWriter* cabacEstimator, CtxCache* ctxCache );

  static void disabledRate   ( CodingStructure& cs, double saoDisabledRate[ MAX_NUM_COMP ][ MAX_TLAYER ], SAOBlkParam* reconParams, const double saoEncodingRate, const double saoEncodingRateChroma, const ChromaFormat& chromaFormat );
  static void decidePicParams( const CodingStructure& cs, double saoDisabledRate[ MAX_NUM_COMP ][ MAX_TLAYER ], bool saoEnabled[ MAX_NUM_COMP ], const double saoEncodingRate, const double saoEncodingRateChroma, const ChromaFormat& chromaFormat );

  void storeCtuReco          ( CodingStructure& cs, const UnitArea& ctuArea );
  void getCtuStatistics      ( CodingStructure& cs, std::vector<SAOStatData**>& saoStatistics, const UnitArea& ctuArea, const int ctuRsAddr );
  void decideCtuParams       ( CodingStructure& cs, const std::vector<SAOStatData**>& saoStatistics, const bool saoEnabled[ MAX_NUM_COMP ], const bool allBlksDisabled, const UnitArea& ctuArea, const int ctuRsAddr, SAOBlkParam* reconParams, SAOBlkParam* codedParams );

private:
  void    getStatistics      (std::vector<SAOStatData**>& blkStats, PelUnitBuf& orgYuv, PelUnitBuf& srcYuv, CodingStructure& cs, bool isCalculatePreDeblockSamples = false);
  void    getBlkStats        (const ComponentID compIdx, const int channelBitDepth, SAOStatData* statsDataTypes, Pel* srcBlk, Pel* orgBlk, int srcStride, int orgStride, int width, int height, bool isLeftAvail,  bool isRightAvail, bool isAboveAvail, bool isBelowAvail, bool isAboveLeftAvail, bool isAboveRightAvail, bool isCalculatePreDeblockSamples = false );
  void    deriveModeNewRDO   (const BitDepths &bitDepths, int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES], const bool* sliceEnabled, const std::vector<SAOStatData**>& blkStats, SAOBlkParam& modeParam, double& modeNormCost );
  void    deriveModeMergeRDO (const BitDepths &bitDepths, int ctuRsAddr, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES], const bool* sliceEnabled, const std::vector<SAOStatData**>& blkStats, SAOBlkParam& modeParam, double& modeNormCost );
  int64_t getDistortion      (const int channelBitDepth, int typeIdc, int typeAuxInfo, int* offsetVal, SAOStatData& statData);
  void    deriveOffsets      (ComponentID compIdx, const int channelBitDepth, int typeIdc, SAOStatData& statData, int* quantOffsets, int& typeAuxInfo);
  void    deriveLoopFilterBoundaryAvailibility(CodingStructure& cs, const Position& pos, bool& isLeftAvail, bool& isAboveAvail, bool& isAboveLeftAvail) const;
  inline int64_t  estSaoDist    (int64_t count, int64_t offset, int64_t diffSum, int shift);
  inline int      estIterOffset (int typeIdx, double lambda, int offsetInput, int64_t count, int64_t diffSum, int shift, int bitIncrease, int64_t& bestDist, double& bestCost, int offsetTh );

private:

  const EncCfg* m_EncCfg;

  //for RDO
  CABACWriter*  m_CABACEstimator;
  CtxCache*     m_CtxCache;
  double        m_lambda[ MAX_NUM_COMP ];
};

} // namespace vvenc

//! \}


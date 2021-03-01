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

Copyright (c) 2019-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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

class VVEncCfg;

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
  void init                  ( const VVEncCfg& encCfg );
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

  const VVEncCfg* m_EncCfg;

  //for RDO
  CABACWriter*  m_CABACEstimator;
  CtxCache*     m_CtxCache;
  double        m_lambda[ MAX_NUM_COMP ];
};

} // namespace vvenc

//! \}


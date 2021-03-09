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
/** \file     EncCu.h
    \brief    Coding Unit (CU) encoder class (header)
*/

#pragma once

#include "vvenc/vvencCfg.h"
#include "CABACWriter.h"
#include "IntraSearch.h"
#include "InterSearch.h"
#include "EncModeCtrl.h"
#include "RateCtrl.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitPartitioner.h"
#include "CommonLib/LoopFilter.h"
#include "DecoderLib/DecCu.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

class EncModeCtrl;
struct EncTestMode;
class EncPicture;

// ====================================================================================================================
// Class definition
// ====================================================================================================================
  struct GeoMergeCombo
  {
    int    splitDir;
    int    mergeIdx0;
    int    mergeIdx1;
    double cost;
    GeoMergeCombo() : splitDir(), mergeIdx0(-1), mergeIdx1(-1), cost(0.0){};
    GeoMergeCombo(int _splitDir, int _mergeIdx0, int _mergeIdx1, double _cost)
      : splitDir(_splitDir), mergeIdx0(_mergeIdx0), mergeIdx1(_mergeIdx1), cost(_cost){};
  };

  struct SmallerThanComboCost
  {
    inline bool operator()(const GeoMergeCombo &first, const GeoMergeCombo &second)
    {
      return (first.cost < second.cost);
    }
  };

  struct GeoComboCostList
  {
    std::vector<GeoMergeCombo> list;
    void                       sortByCost() { std::stable_sort(list.begin(), list.end(), SmallerThanComboCost()); };
  };

  struct SingleGeoMergeEntry
  {
    int    mergeIdx;
    double cost;
    SingleGeoMergeEntry() : mergeIdx(0), cost(MAX_DOUBLE){};
    SingleGeoMergeEntry(int _mergeIdx, double _cost) : mergeIdx(_mergeIdx), cost(_cost){};
  };

  class FastGeoCostList
  {
  public:
    FastGeoCostList() { numGeoTemplatesInitialized = 0; };
    ~FastGeoCostList()
    {
      for (int partIdx = 0; partIdx < 2; partIdx++)
      {
        for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
        {
          delete[] singleDistList[partIdx][splitDir];
        }
        delete[] singleDistList[partIdx];
        singleDistList[partIdx] = nullptr;
      }
    };
    SingleGeoMergeEntry **singleDistList[2];
    void                  init(int numTemplates, int maxNumGeoCand)
    {
      if (numGeoTemplatesInitialized == 0 || numGeoTemplatesInitialized < numTemplates)
      {
        for (int partIdx = 0; partIdx < 2; partIdx++)
        {
          singleDistList[partIdx] = new SingleGeoMergeEntry *[numTemplates];
          for (int splitDir = 0; splitDir < numTemplates; splitDir++)
          {
            singleDistList[partIdx][splitDir] = new SingleGeoMergeEntry[maxNumGeoCand];
          }
        }
        numGeoTemplatesInitialized = numTemplates;
      }
    }
    void insert(int geoIdx, int partIdx, int mergeIdx, double cost)
    {
      assert(geoIdx < numGeoTemplatesInitialized);
      singleDistList[partIdx][geoIdx][mergeIdx] = SingleGeoMergeEntry(mergeIdx, cost);
    }
    int numGeoTemplatesInitialized;
  };


/// CU encoder class
class EncCu
  : DecCu
{
private:
  struct CtxPair
  {
    Ctx start;
    Ctx best;
  };

  std::vector<CtxPair>  m_CtxBuffer;
  CtxPair*              m_CurrCtx;
  CtxCache*             m_CtxCache;

  //  Data : encoder control

  static const int maxCuDepth = ( MAX_CU_SIZE_IDX - MIN_CU_LOG2 ) << 1;

  int                   m_cuChromaQpOffsetIdxPlus1;
  int                   m_tempQpDiff;
  std::vector<int>*     m_globalCtuQpVector;
  XUCache               m_unitCache;
  std::mutex*           m_wppMutex;
  std::mutex*           m_rcMutex;
  CodingStructure*      m_pTempCS[maxCuDepth];
  CodingStructure*      m_pBestCS[maxCuDepth];
  CodingStructure*      m_pTempCS2;
  CodingStructure*      m_pBestCS2;
  PelStorage            m_pOrgBuffer[maxCuDepth];
  PelStorage            m_pRspBuffer[maxCuDepth];

  //  Access channel
  const VVEncCfg*       m_pcEncCfg;
  IntraSearch           m_cIntraSearch;
  InterSearch           m_cInterSearch;
  RdCost                m_cRdCost;
  LoopFilter            m_cLoopFilter;

  CABACWriter*          m_CABACEstimator;
  EncModeCtrl           m_modeCtrl;
  TrQuant               m_cTrQuant;                          ///< transform & quantization
  RateCtrl*             m_pcRateCtrl;

  PelStorage            m_aTmpStorageLCU[MAX_TMP_BUFS];     ///< used with CIIP, EDO, GEO
  SortedPelUnitBufs<SORTED_BUFS> m_SortedPelUnitBufs;
  FastGeoCostList       m_GeoCostList;
  double                m_AFFBestSATDCost;
  static const uint8_t  m_GeoModeTest[GEO_MAX_NUM_CANDS][2];
  double                m_mergeBestSATDCost;

  MotionInfo            m_subPuMiBuf[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
  
  Mv                    m_refinedMvdL0[MRG_MAX_NUM_CANDS][MAX_NUM_SUBCU_DMVR];

  double                m_sbtCostSave[2];
  // thread stuff
  Ctx*                  m_syncPicCtx;                        ///< context storage for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row used for estimation
  PelStorage            m_dbBuffer;
  int                   m_ctuRcQP;
  
  Partitioner           m_partitioner;

public:
  EncCu();
  virtual ~EncCu();

  void  init                  ( const VVEncCfg& encCfg, const SPS& sps, std::vector<int>* const globalCtuQpVector, Ctx* syncPicCtx, RateCtrl* pRateCtrl );
  void  setCtuEncRsrc         ( CABACWriter* cabacEstimator, CtxCache* ctxCache, ReuseUniMv* pReuseUniMv, BlkUniMvInfoBuffer* pBlkUniMvInfoBuffer, AffineProfList* pAffineProfList );
  void  destroy               ();

  std::vector<int>* getQpPtr  () const { return m_globalCtuQpVector; }

  void  initPic               ( Picture* pic );
  void  initSlice             ( const Slice* slice );
  void  setUpLambda           ( Slice& slice, const double dLambda, const int iQP, const bool setSliceLambda, const bool saveUnadjusted, const bool useRC = false);
  void  updateLambda          ( const Slice& slice, const double ctuLambda, const int ctuQP, const int newQP, const bool saveUnadjusted);

  void encodeCtu              ( Picture* pic, int (&prevQP)[MAX_NUM_CH], uint32_t ctuXPosInCtus, uint32_t ctuYPosInCtus );

private:
  void  xCompressCtu          ( CodingStructure& cs, const UnitArea& area, const unsigned ctuRsAddr, const int prevQP[] );

  void xCalDebCost            ( CodingStructure& cs, Partitioner& partitioner );
  Distortion xGetDistortionDb ( CodingStructure& cs, CPelBuf& org, CPelBuf& reco, const CompArea& compArea, bool beforeDb );

  void xCompressCU            ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm );
  bool xCheckBestMode         ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestmode, const bool useEDO = false );

  void xCheckRDCostIntra      ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestMode );
  void xCheckRDCostInter      ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestMode );
  void xCheckRDCostInterIMV   ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestMode );
  void xCheckRDCostAffineMerge( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestMode );
  void xCheckRDCostMerge      ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm,       EncTestMode& encTestMode );
  void xCheckRDCostMergeGeo   ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestMode );
  void xCheckModeSplit        ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestMode );
  void xCheckModeSplitInternal( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestMode, const ModeType modeTypeParent, bool& skipInterPass );
  void xReuseCachedResult     ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm );

  void xSetCtuQPRC            ( CodingStructure& cs, const Slice* slice, const Picture* pic, const int ctuRsAddr );
  void xUpdateAfterCtuRC      ( CodingStructure& cs, const Slice* slice, const UnitArea& ctuArea, const double oldLambda, const int numberOfWrittenBits, const int ctuRsAddr );

  void xCheckDQP              ( CodingStructure& cs, Partitioner& partitioner, bool bKeepCtx = false);
  void xEncodeDontSplit       ( CodingStructure& cs, Partitioner& partitioner);

  void xEncodeInterResidual   ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestMode, 
                                int residualPass = 0, bool* bestHasNonResi = NULL, double* equBcwCost = NULL );

  uint64_t xCalcPuMeBits      ( const CodingUnit& cu);
  double   xCalcDistortion    ( CodingStructure *&cur_CS, ChannelType chType, int BitDepth, int imv);
  int      xCheckMMVDCand     ( uint32_t& mmvdMergeCand, int& bestDir, int tempNum, double& bestCostOffset, double& bestCostMerge, double bestCostList );
};

} // namespace vvenc

//! \}


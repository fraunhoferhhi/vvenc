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
/** \file     EncCu.h
    \brief    Coding Unit (CU) encoder class (header)
*/

#pragma once

#include "../../../include/vvenc/EncCfg.h"
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
    void                       sortByCost() { std::sort(list.begin(), list.end(), SmallerThanComboCost()); };
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
  int                   m_cuChromaQpOffsetIdxPlus1;
  int                   m_tempQpDiff;
  std::vector<int>*     m_globalCtuQpVector;
  XUCache               m_unitCache;
  std::mutex*           m_wppMutex;
  std::mutex*           m_rcMutex;
  CodingStructure***    m_pTempCS;
  CodingStructure***    m_pBestCS;
  CodingStructure***    m_pTempCS2;
  CodingStructure***    m_pBestCS2;
  PelStorage***         m_pOrgBuffer;
  PelStorage***         m_pRspBuffer;

  //  Access channel
  const EncCfg*         m_pcEncCfg;
  IntraSearch           m_cIntraSearch;
  InterSearch           m_cInterSearch;
  RdCost                m_cRdCost;
  LoopFilter*           m_pcLoopFilter;

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

  double                m_sbtCostSave[2];
  // thread stuff
  Ctx*                  m_syncPicCtx;                        ///< context storage for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row used for estimation
  PelStorage            m_dbBuffer;

public:
  EncCu();
  virtual ~EncCu();

  void  init                  ( const EncCfg& encCfg, const SPS& sps, LoopFilter* LoopFilter, std::vector<int>* const globalCtuQpVector, Ctx* syncPicCtx, RateCtrl* pRateCtrl );
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

  uint64_t xCalcPuMeBits      ( const PredictionUnit& pu);
  double   xCalcDistortion    ( CodingStructure *&cur_CS, ChannelType chType, int BitDepth, int imv);
  int      xCheckMMVDCand     ( uint32_t& mmvdMergeCand, int& bestDir, int tempNum, double& bestCostOffset, double& bestCostMerge, double bestCostList );
};

} // namespace vvenc

//! \}


/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
    int    splitDir  =  0;
    int8_t mergeIdx0 = -1;
    int8_t mergeIdx1 = -1;
    double cost      = 0.0;
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

  class FastGeoCostList
  {
    using CostArray = double[GEO_NUM_PARTITION_MODE][2];

    int        m_maxNumGeoCand{ 0 };
    CostArray *m_singleDistList{ nullptr };

  public:

    FastGeoCostList() {}
    ~FastGeoCostList()
    {
      delete[] m_singleDistList;
      m_singleDistList = nullptr;
    }

    void init( int maxNumGeoCand )
    {
      if( m_maxNumGeoCand != maxNumGeoCand )
      {
        delete[] m_singleDistList;
        m_singleDistList = nullptr;

        CHECK( maxNumGeoCand > MRG_MAX_NUM_CANDS, "Too many candidates" );
        m_singleDistList = new CostArray[maxNumGeoCand];
        m_maxNumGeoCand  = maxNumGeoCand;
      }
    }

    void insert( int splitDir, int partIdx, int mergeIdx, double cost )
    {
      CHECKD( splitDir >= GEO_NUM_PARTITION_MODE, "geoIdx is too large" );
      CHECKD( mergeIdx >= m_maxNumGeoCand, "mergeIdx is too large" );
      CHECKD( partIdx >= 2, "partIdx is too large" );

      m_singleDistList[mergeIdx][splitDir][partIdx] = cost;
    }

    double getCost( const int splitDir, const int mergeCand0, const int mergeCand1 )
    {
      return m_singleDistList[mergeCand0][splitDir][0] + m_singleDistList[mergeCand1][splitDir][1];
    }
  };



  class MergeItem
  {
  private:
    PelStorage m_pelStorage;
    std::vector<MotionInfo> m_mvStorage;

  public:

    enum class MergeItemType
    {
      REGULAR,
      SBTMVP,
      AFFINE,
      MMVD,
      CIIP,
      GPM,
      IBC,
      NUM,
    };

    double        cost;
    std::array<MvField[3], 2>
                  mvField;
    int           mergeIdx;
    uint8_t       bcwIdx;
    uint8_t       interDir;
    bool          useAltHpelIf;
    int8_t        affineType;
    int8_t        numCiipIntra;

    bool          noResidual;
    bool          noBdofRefine;

    bool          lumaPredReady;
    bool          chromaPredReady;

    MergeItemType mergeItemType;

    MergeItem();
    ~MergeItem();

    void          create          (ChromaFormat chromaFormat, const Area& area);
    void          init            ();
    void          importMergeInfo (const MergeCtx&       mergeCtx, int _mergeIdx, MergeItemType _mergeItemType, CodingUnit& cu);
    void          importMergeInfo (const AffineMergeCtx& mergeCtx, int _mergeIdx, MergeItemType _mergeItemType, CodingUnit& cu);
    bool          exportMergeInfo (CodingUnit& cu, bool forceNoResidual) const;
    PelUnitBuf    getPredBuf      (const UnitArea& unitArea) { return m_pelStorage.getCompactBuf( unitArea ); }
    CMotionBuf    getMvBuf        (const UnitArea &unitArea) const { return CMotionBuf( m_mvStorage.data(), g_miScaling.scale( unitArea.lumaSize() ) ); }
    MotionBuf     getMvBuf        (const UnitArea &unitArea)       { return  MotionBuf( m_mvStorage.data(), g_miScaling.scale( unitArea.lumaSize() ) ); }

    static int getGpmUnfiedIndex( int splitDir, const MergeIdxPair &geoMergeIdx )
    {
      return ( splitDir << 8 ) | ( geoMergeIdx[ 0 ] << 4 ) | geoMergeIdx[ 1 ];
    }

    static void updateGpmIdx( int mergeIdx, uint8_t &splitDir, MergeIdxPair &geoMergeIdx )
    {
      splitDir       = ( mergeIdx >> 8 ) & 0xFF;
      geoMergeIdx[0] = ( mergeIdx >> 4 ) & 0xF;
      geoMergeIdx[1] =   mergeIdx        & 0xF;
    }
  };

  class MergeItemList
  {
  private:
    std::vector<MergeItem*>  m_mergeItems;
    std::vector<MergeItem*>  m_list;
    size_t                   m_maxTrackingNum = 0;
    size_t                   m_maxSize;
    size_t                   m_maxExtSize;
    size_t                   m_numExt;

  public:
    MergeItemList();
    ~MergeItemList();

    void          init                  (size_t maxListSize, size_t maxExtSize, ChromaFormat chromaFormat, SizeType ctuWidth, SizeType ctuHeight);
    MergeItem*    allocateNewMergeItem  ();
    bool          insertMergeItemToList (MergeItem* p);
    void          giveBackMergeItem     (MergeItem* p);
    void          resetList             (size_t maxTrackingNum);
    void          shrinkList            (size_t reduceTo);
    MergeItem*    getMergeItemInList    (size_t index);
    size_t        size                  () const { return m_list.size(); }
    size_t        capacity              () const { return m_maxTrackingNum; }
  };

using MergeBufVector = static_vector<PelUnitBuf, MRG_MAX_NUM_CANDS>;

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

  PelStorage            m_aTmpStorageLCU  [MAX_TMP_BUFS];     ///< used with CIIP, EDO, GEO
  PelStorage            m_acMergeTmpBuffer[MRG_MAX_NUM_CANDS];

  SortedPelUnitBufs<SORTED_BUFS>
                        m_SortedPelUnitBufs;
  FastGeoCostList       m_GeoCostList;
  static const MergeIdxPair
                        m_GeoModeTest[GEO_MAX_NUM_CANDS];
  double                m_mergeBestSATDCost;

  MotionInfo            m_subPuMiBuf[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
  
  Mv                    m_refinedMvdL0[MRG_MAX_NUM_CANDS][MAX_NUM_SUBCU_DMVR];

  double                m_sbtCostSave[2];
  // thread stuff
  Ctx*                  m_syncPicCtx;                        ///< context storage for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row used for estimation
  PelStorage            m_dbBuffer;
  
  Partitioner           m_partitioner;

  int                   m_bestBcwIdx[2];
  double                m_bestBcwCost[2];

  int                   m_MergeSimpleFlag;
  int                   m_tileIdx;
  int                   m_EDO;

  GeoComboCostList      m_comboList;
  MergeItemList         m_mergeItemList;
  std::array<Mv, MAX_NUM_SUBCU_DMVR>
                        m_subPuMvOffset[MRG_MAX_NUM_CANDS];
  Distortion            m_uiSadBestForQPA;

  static const double coefSquareCUsFasterFastMedium[2][5][2][2][2];
  static const double coefSquareCUsSlowSlower[2][5][2][5][2][2][2];

public:
  EncCu();
  virtual ~EncCu();

  void  init                  ( const VVEncCfg& encCfg, const SPS& sps, std::vector<int>* const globalCtuQpVector, Ctx* syncPicCtx, RateCtrl* pRateCtrl );
  void  setCtuEncRsrc         ( CABACWriter* cabacEstimator, CtxCache* ctxCache, ReuseUniMv* pReuseUniMv, BlkUniMvInfoBuffer* pBlkUniMvInfoBuffer, AffineProfList* pAffineProfList, IbcBvCand* pCachedBvs );
  void  destroy               ();

  std::vector<int>* getQpPtr  () const { return m_globalCtuQpVector; }

  void  initPic               ( Picture* pic );
  void  initSlice             ( const Slice* slice );
  void  setUpLambda           ( Slice& slice, const double dLambda, const int iQP, const bool setSliceLambda, const bool saveUnadjusted);
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
  void xCheckRDCostUnifiedMerge
                              ( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm,       EncTestMode& encTestMode );
  void xCheckModeSplit        ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestMode );
  void xCheckModeSplitInternal( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestMode, const ModeType modeTypeParent, bool& skipInterPass );
  void xReuseCachedResult     ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm );

  void xCheckDQP              ( CodingStructure& cs, Partitioner& partitioner, bool bKeepCtx = false);
  void xEncodeDontSplit       ( CodingStructure& cs, Partitioner& partitioner);

  void xEncodeInterResidual   ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestMode, 
                                int residualPass = 0, bool* bestHasNonResi = NULL, double* equBcwCost = NULL );

  bool xIsBcwSkip( const CodingUnit& cu )
  {
    if( cu.slice->sliceType != VVENC_B_SLICE )
    {
      return true;
    }
    return( (m_pcEncCfg->m_QP > 32) && ((cu.slice->TLayer >= 4)
            || ((cu.refIdx[0] >= 0 && cu.refIdx[1] >= 0)
              && (  abs(cu.slice->poc - cu.slice->getRefPOC(REF_PIC_LIST_0, cu.refIdx[0])) == 1
                ||  abs(cu.slice->poc - cu.slice->getRefPOC(REF_PIC_LIST_1, cu.refIdx[1])) == 1
               ) )                     ));
  }

  uint64_t xCalcPuMeBits                 ( const CodingUnit& cu);
  double   xCalcDistortion               ( CodingStructure *&cur_CS, ChannelType chType, int BitDepth, int imv);
  int      xCheckMMVDCand                ( MmvdIdx& mmvdMergeCand, int& bestDir, int tempNum, double& bestCostOffset, double& bestCostMerge, double bestCostList );
  void     xCheckRDCostIBCMode           ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& pm, const EncTestMode& encTestMode );
  void     xCheckRDCostIBCModeMerge2Nx2N ( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& partitioner, const EncTestMode& encTestMode );


  CodingUnit* getCuForInterPrediction ( CodingStructure* cs, const EncTestMode& encTestMode );
  unsigned int updateRdCheckingNum    ( MergeItemList&, double threshold, unsigned int numMergeSatdCand );

  void    generateMergePrediction     ( const UnitArea& unitArea, MergeItem* mergeItem, CodingUnit& cu, bool luma, bool chroma,
                                        PelUnitBuf& dstBuf, bool finalRd, bool forceNoResidual, PelUnitBuf* predBuf1, PelUnitBuf* predBuf2 );
  double  calcLumaCost4MergePrediction( const TempCtx& ctxStart, const PelUnitBuf& predBuf, double lambda, CodingUnit& pu, DistParam& distParam );
  void    addRegularCandsToPruningList( const MergeCtx& mergeCtx, const UnitArea& localUnitArea, double sqrtLambdaForFirstPassIntra,
                                        const TempCtx& ctxStart, DistParam& distParam, CodingUnit& cu, bool* sameMv, MergeBufVector& regularPred );
  void    addCiipCandsToPruningList   ( const MergeCtx& mergeCtx, const UnitArea& localUnitArea, double sqrtLambdaForFirstPassIntra,
                                        const TempCtx& ctxStart, DistParam& distParam, CodingUnit& cu, bool* sameMv );
  void    addMmvdCandsToPruningList   ( const MergeCtx& mergeCtx, const UnitArea& localUnitArea, double sqrtLambdaForFirstPassIntra,
                                        const TempCtx& ctxStart, DistParam& distParam, CodingUnit& cu );
  void    addAffineCandsToPruningList ( AffineMergeCtx& affineMergeCtx, const UnitArea& localUnitArea, double sqrtLambdaForFirstPass,
                                        const TempCtx& ctxStart, DistParam& distParam, CodingUnit& cu );
  void    addGpmCandsToPruningList    ( const MergeCtx& mergeCtx, const UnitArea& localUnitArea, double sqrtLambdaForFirstPass,
                                        const TempCtx& ctxStart, const GeoComboCostList& comboList, MergeBufVector& geoBuffer, DistParam& distParam, CodingUnit& cu );

  bool    prepareGpmComboList         ( const MergeCtx& mergeCtx, const UnitArea& localUnitArea, double sqrtLambdaForFirstPass,
                                        GeoComboCostList& comboList, MergeBufVector& geoBuffer, CodingUnit& cu );
  // TODO: for now skip that
  //void    checkEarlySkip              ( const CodingStructure* bestCS, const Partitioner &partitioner );
};

} // namespace vvenc

//! \}


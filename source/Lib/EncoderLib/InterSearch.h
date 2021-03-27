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
/** \file     InterSearch.h
    \brief    inter search class (header)
 */

#pragma once

#include "CABACWriter.h"
#include "CommonLib/MotionInfo.h"
#include "CommonLib/InterPrediction.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/Unit.h"
#include "CommonLib/UnitPartitioner.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/AffineGradientSearch.h"
#if IBC_VTM 
#include <unordered_map>
#endif

#include <vector>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Class definition
// ====================================================================================================================

static const uint32_t MAX_NUM_REF_LIST_ADAPT_SR = 2;
static const uint32_t MAX_IDX_ADAPT_SR          = MAX_REF_PICS;
static const uint32_t NUM_MV_PREDICTORS         = 3;

struct BlkUniMvInfo
{
  Mv uniMvs[2][MAX_REF_PICS];
  int x, y, w, h;
};

#if IBC_VTM
struct BlkRecord
{
  std::unordered_map<Mv, Distortion> bvRecord;
};
#endif
struct BlkUniMvInfoBuffer
{
  BlkUniMvInfo* getBlkUniMvInfo( int i ) const { return m_uniMvList + ((m_uniMvListIdx - 1 - i + m_uniMvListMaxSize) % (m_uniMvListMaxSize)); }

  void insertUniMvCands( const Area& blkArea, const Mv* cMvTemp)
  {
    BlkUniMvInfo* curMvInfo = m_uniMvList + m_uniMvListIdx;
    int j = 0;
    for (; j < m_uniMvListSize; j++)
    {
      BlkUniMvInfo* prevMvInfo = getBlkUniMvInfo( j );
      if ((blkArea.x == prevMvInfo->x) && (blkArea.y == prevMvInfo->y) && (blkArea.width == prevMvInfo->w) && (blkArea.height == prevMvInfo->h))
      {
        break;
      }
    }

//    DTRACE( g_trace_ctx, D_TMP, "%d unimv insert %d, %d [%d %d] \n", g_trace_ctx->getChannelCounter(D_TMP), blkArea.x, blkArea.y, blkArea.width, blkArea.height );
    if (j < m_uniMvListSize)
    {
      curMvInfo = getBlkUniMvInfo( j );
    }

    ::memcpy(curMvInfo->uniMvs, cMvTemp, 2 * MAX_REF_PICS * sizeof(Mv));
    if (j == m_uniMvListSize)  // new element
    {
      curMvInfo->x = blkArea.x;
      curMvInfo->y = blkArea.y;
      curMvInfo->w = blkArea.width;
      curMvInfo->h = blkArea.height;
      m_uniMvListSize = std::min(m_uniMvListSize + 1, m_uniMvListMaxSize);
      m_uniMvListIdx = (m_uniMvListIdx + 1) % (m_uniMvListMaxSize);
    }
  }

  void savePrevUniMvInfo(const CompArea& blkArea, BlkUniMvInfo &tmpUniMvInfo, bool& isUniMvInfoSaved)
  {
    int j = 0;
    BlkUniMvInfo* curMvInfo = nullptr;
    for (; j < m_uniMvListSize; j++)
    {
      curMvInfo = getBlkUniMvInfo( j );
      if ((blkArea.x == curMvInfo->x) && (blkArea.y == curMvInfo->y) && (blkArea.width == curMvInfo->w) && (blkArea.height == curMvInfo->h))
      {
        break;
      }
    }

    if (j < m_uniMvListSize)
    {
//      DTRACE( g_trace_ctx, D_TMP, "%d unimv save %d, %d [%d %d] \n", g_trace_ctx->getChannelCounter(D_TMP), curMvInfo->x, curMvInfo->y, curMvInfo->w, curMvInfo->h );
      isUniMvInfoSaved = true;
      tmpUniMvInfo = *curMvInfo;
    }
  }

  void addUniMvInfo( const BlkUniMvInfo &tmpUniMVInfo)
  {
    int j = 0;
    BlkUniMvInfo* curMvInfo = nullptr;
    for (; j < m_uniMvListSize; j++)
    {
      curMvInfo = getBlkUniMvInfo( j );
      if ((tmpUniMVInfo.x == curMvInfo->x) && (tmpUniMVInfo.y == curMvInfo->y) && (tmpUniMVInfo.w == curMvInfo->w) && (tmpUniMVInfo.h == curMvInfo->h))
      {
        break;
      }
    }

//    DTRACE( g_trace_ctx, D_TMP, "%d unimv add %d, %d [%d %d] \n", g_trace_ctx->getChannelCounter(D_TMP), tmpUniMVInfo.x, tmpUniMVInfo.y, tmpUniMVInfo.w, tmpUniMVInfo.h );
    if (j < m_uniMvListSize)
    {
      *curMvInfo = tmpUniMVInfo;
    }
    else
    {
      m_uniMvList[m_uniMvListIdx] = tmpUniMVInfo;
      m_uniMvListIdx = (m_uniMvListIdx + 1) % m_uniMvListMaxSize;
      m_uniMvListSize = std::min(m_uniMvListSize + 1, m_uniMvListMaxSize);
    }
  }

  void resetUniMvList               () { m_uniMvListIdx = 0; m_uniMvListSize = 0; }

  BlkUniMvInfoBuffer()
  {
    m_uniMvListMaxSize = 15;
    m_uniMvList = new BlkUniMvInfo[m_uniMvListMaxSize];
    m_uniMvListIdx = 0;
    m_uniMvListSize = 0;
  }

  ~BlkUniMvInfoBuffer()
  {
    if ( m_uniMvList )
    {
      delete[] m_uniMvList;
      m_uniMvList = nullptr;
    }
    m_uniMvListIdx = 0;
    m_uniMvListSize = 0;
  }

  BlkUniMvInfo*   m_uniMvList;
  int             m_uniMvListIdx;
  int             m_uniMvListSize;
  int             m_uniMvListMaxSize;
};

class EncPicture;
class VVEncCfg;
class EncModeCtrl;
class EncReshape;
class EncCu;

struct AffineMVInfo
{
  Mv  affMVs[2][MAX_REF_PICS][3];
  int x, y, w, h;
};

typedef struct
{
  Mv acMvAffine4Para[2][3];
  Mv acMvAffine6Para[2][3];
  int16_t affine4ParaRefIdx[2];
  int16_t affine6ParaRefIdx[2];
  bool affine4ParaAvail;
  bool affine6ParaAvail;
} EncAffineMotion;

struct AffineProfList
{
  AffineProfList()
  {
    m_affMVListMaxSize = 0;
    m_affMVList = nullptr;
    m_affMVListIdx = 0;
    m_affMVListSize = 0;
  }

  void init( int intraPeriod )
  {
    m_affMVListMaxSize = (intraPeriod == -1) ? AFFINE_ME_LIST_SIZE_LD : AFFINE_ME_LIST_SIZE;
    if( !m_affMVList)
    {
      m_affMVList = new AffineMVInfo[m_affMVListMaxSize];
    }
    m_affMVListIdx = 0;
    m_affMVListSize = 0;
  }

  ~AffineProfList()
  {
    if( m_affMVList)
    {
      delete[] m_affMVList;
      m_affMVList = nullptr;
    }
    m_affMVListIdx = 0;
    m_affMVListSize = 0;
  }

  void resetAffineMVList() { m_affMVListIdx = 0; m_affMVListSize = 0; }
  bool savePrevAffMVInfo(int idx, AffineMVInfo &tmpMVInfo )
  {
    if( m_affMVListSize > idx)
    {
      tmpMVInfo = m_affMVList[(m_affMVListIdx - 1 - idx + m_affMVListMaxSize) % m_affMVListMaxSize];
      return true;
    }

    return false;
  }

  void addAffMVInfo(AffineMVInfo &tmpMVInfo)
  {
    int j = 0;
    AffineMVInfo *prevInfo = nullptr;
    for (; j < m_affMVListSize; j++)
    {
      prevInfo = m_affMVList + ((m_affMVListIdx - j - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));
      if ((tmpMVInfo.x == prevInfo->x) && (tmpMVInfo.y == prevInfo->y) && (tmpMVInfo.w == prevInfo->w) && (tmpMVInfo.h == prevInfo->h))
      {
        break;
      }
    }
    if (j < m_affMVListSize)
    {
      *prevInfo = tmpMVInfo;
    }
    else
    {
      m_affMVList[m_affMVListIdx] = tmpMVInfo;
      m_affMVListIdx = (m_affMVListIdx + 1) % m_affMVListMaxSize;
      m_affMVListSize = std::min(m_affMVListSize + 1, m_affMVListMaxSize);
    }
  }

  void insert( const AffineMVInfo& cMvTemp, const Area& area )
  {
    AffineMVInfo *affMVInfo = m_affMVList + m_affMVListIdx;

    //check;
    int j = 0;
    for (; j < m_affMVListSize; j++)
    {
      AffineMVInfo *prevMvInfo = m_affMVList + ((m_affMVListIdx - j - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));
      if ((area.x == prevMvInfo->x) && (area.y == prevMvInfo->y) && (area.width == prevMvInfo->w) && (area.height == prevMvInfo->h))
      {
        break;
      }
    }
    if (j < m_affMVListSize)
    {
      affMVInfo = m_affMVList + ((m_affMVListIdx - j - 1 + m_affMVListMaxSize) % (m_affMVListMaxSize));
    }

    ::memcpy(affMVInfo->affMVs, cMvTemp.affMVs, sizeof(cMvTemp.affMVs));

    if (j == m_affMVListSize)
    {
      affMVInfo->x = area.x;
      affMVInfo->y = area.y;
      affMVInfo->w = area.width;
      affMVInfo->h = area.height;
      m_affMVListSize = std::min(m_affMVListSize + 1, m_affMVListMaxSize);
      m_affMVListIdx = (m_affMVListIdx + 1) % (m_affMVListMaxSize);
    }
  }

  AffineMVInfo*   m_affMVList;
  int             m_affMVListIdx;
  int             m_affMVListSize;
  int             m_affMVListMaxSize;
};

struct ReuseUniMv
{
  Mv*             m_reusedUniMVs[6][6][32][32];

  ReuseUniMv();
  ~ReuseUniMv();
  void resetReusedUniMvs();
};

/// encoder search class
class InterSearch : public InterPrediction, AffineGradientSearch
{
private:
  EncModeCtrl*      m_modeCtrl;

  PelStorage        m_tmpPredStorage[NUM_REF_PIC_LIST_01];
  PelStorage        m_tmpStorageLCU;
  CodingStructure** m_pSaveCS;

  ClpRng            m_lumaClpRng;
#if IBC_VTM
  Mv                m_acBVs[2 * IBC_NUM_CANDIDATES];
  unsigned int      m_numBVs;
  PatentBvCand      m_defaultCachedBvs;
  std::unordered_map< Position, std::unordered_map< Size, BlkRecord> > m_ctuRecord;
#endif

protected:
  // interface to option
  const VVEncCfg*   m_pcEncCfg;

  // interface to classes
  TrQuant*          m_pcTrQuant;
  EncReshape*       m_pcReshape;

  // ME parameters
  int               m_iSearchRange;
  int               m_bipredSearchRange; // Search range for bi-prediction
  MESearchMethod    m_motionEstimationSearchMethod;
  int               m_aaiAdaptSR[MAX_NUM_REF_LIST_ADAPT_SR][MAX_IDX_ADAPT_SR];

  // RD computation
  CABACWriter*      m_CABACEstimator;
  CtxCache*         m_CtxCache;
  DistParam         m_cDistParam;
  RdCost*           m_pcRdCost;

  Distortion        m_hevcCost;
  EncAffineMotion   m_affineMotion;
  PelStorage        m_tmpAffiStorage;
  Pel*              m_tmpAffiError;
  int*              m_tmpAffiDeri[2];
  MotionInfo        m_subPuMiBuf[(MAX_CU_SIZE * MAX_CU_SIZE) >> (MIN_CU_LOG2 << 1)];
  // Misc.
  Pel*              m_pTempPel;

  // AMVP cost computation
  uint32_t          m_auiMVPIdxCost[AMVP_MAX_NUM_CANDS+1][AMVP_MAX_NUM_CANDS+1];
  Distortion        m_estMinDistSbt[NUMBER_SBT_MODE + 1]; // estimated minimum SSE value of the PU if using a SBT mode
  uint8_t           m_sbtRdoOrder[NUMBER_SBT_MODE];       // order of SBT mode in RDO
  bool              m_skipSbtAll;                         // to skip all SBT modes for the current PU

public:
  ReuseUniMv*         m_ReuseUniMv;
  BlkUniMvInfoBuffer* m_BlkUniMvInfoBuffer;
  AffineProfList*     m_AffineProfList;

#if IBC_VTM
  bool                m_clipMvInSubPic;
#endif

public:
  InterSearch();
  virtual ~InterSearch();

  void init                         ( const VVEncCfg& encCfg, TrQuant* pTrQuant, RdCost* pRdCost, EncModeCtrl* pModeCtrl, CodingStructure **pSaveCS );
  void setCtuEncRsrc                ( CABACWriter* cabacEstimator, CtxCache* ctxCache, ReuseUniMv* pReuseUniMv, BlkUniMvInfoBuffer* pBlkUniMvInfoBuffer, AffineProfList* pAffineProfList );

  void destroy                      ();

  /// encoder estimation - inter prediction (non-skip)
  void predInterSearch              ( CodingUnit& cu, Partitioner& partitioner );
  /// set ME search range
  void encodeResAndCalcRdInterCU    ( CodingStructure &cs, Partitioner &partitioner, const bool skipResidual );

  void setSearchRange               ( const Slice* slice, const VVEncCfg& encCfg );

  void resetSavedAffineMotion       ();
  void storeAffineMotion            ( Mv acAffineMv[2][3], int16_t affineRefIdx[2], EAffineModel affineType, int BcwIdx);
  void loadGlobalUniMvs             ( const Area& lumaArea, const PreCalcValues& pcv);

  uint8_t    skipSbtByRDCost        ( int width, int height, int mtDepth, uint8_t sbtIdx, uint8_t sbtPos, double bestCost, Distortion distSbtOff, double costSbtOff, bool rootCbfSbtOff );
  bool       getSkipSbtAll          ()                            const { return m_skipSbtAll; }
  uint8_t    getSbtRdoOrder         ( uint8_t idx )               const { assert( m_sbtRdoOrder[idx] < NUMBER_SBT_MODE ); assert( (uint32_t)( m_estMinDistSbt[m_sbtRdoOrder[idx]] >> 2 ) < ( MAX_UINT >> 1 ) ); return m_sbtRdoOrder[idx]; }
  Distortion getEstDistSbt          ( uint8_t sbtMode)            const { return m_estMinDistSbt[sbtMode]; }
  void       initSbtRdoOrder        ( uint8_t sbtMode )                 { m_sbtRdoOrder[0] = sbtMode; m_estMinDistSbt[0] = m_estMinDistSbt[sbtMode]; }

  void       getBestSbt             ( CodingStructure* tempCS, CodingUnit* cu, uint8_t& histBestSbt, Distortion& curPuSse, uint8_t sbtAllowed, bool doPreAnalyzeResi, bool mtsAllowed );
#if IBC_VTM
  void       resetCtuRecordIBC(){ m_ctuRecord.clear(); }
  bool       predIBCSearch(CodingUnit& cu, Partitioner& partitioner);
  void       resetIbcSearch()
  {
    for (int i = 0; i < IBC_NUM_CANDIDATES; i++)
    {
      m_defaultCachedBvs.m_bvCands[i].setZero();
    }
    m_defaultCachedBvs.currCnt = 0;
  }
 bool searchBvIBC(CodingUnit& pu, int xPos, int yPos, int width, int height, int picWidth, int picHeight, int xBv, int yBv, int ctuSize);

#endif
private:
  void       xCalcMinDistSbt        ( CodingStructure &cs, const CodingUnit& cu, const uint8_t sbtAllowed );
  /// sub-function for motion vector refinement used in fractional-pel accuracy
  Distortion xPatternRefinement     ( const CPelBuf* pcPatternKey, Mv baseRefMv, int iFrac, Mv& rcMvFrac, bool bAllowUseOfHadamard, Distortion& uiDistBest, int& patternId, CPelBuf* pattern, bool useAltHpelIf );

   typedef struct
   {
     int left;
     int right;
     int top;
     int bottom;
   }SearchRange;

  typedef struct
  {
    SearchRange     searchRange;
    const CPelBuf*  pcPatternKey;
    const Pel*      piRefY;
    int             iRefStride;
    int             iBestX;
    int             iBestY;
    uint32_t        uiBestRound;
    uint32_t        uiBestDistance;
    Distortion      uiBestSad;
    uint8_t         ucPointNr;
    int             subShiftMode;
    unsigned        imvShift;
    bool            useAltHpelIf;
    bool            inCtuSearch;
    bool            zeroMV;
  } TZSearchStruct;

  // sub-functions for ME
  inline void xTZSearchHelp         ( TZSearchStruct& rcStruct, const int iSearchX, const int iSearchY, const uint8_t ucPointNr, const uint32_t uiDistance );
  inline void xTZ2PointSearch       ( TZSearchStruct& rcStruct );
  inline void xTZ8PointSquareSearch ( TZSearchStruct& rcStruct, const int iStartX, const int iStartY, const int iDist );
  inline void xTZ8PointDiamondSearch( TZSearchStruct& rcStruct, const int iStartX, const int iStartY, const int iDist, const bool bCheckCornersAtDist1 );

  // -------------------------------------------------------------------------------------------------------------------
  // Inter search (AMP)
  // -------------------------------------------------------------------------------------------------------------------

  void       xEstimateMvPredAMVP  ( CodingUnit& cu, CPelUnitBuf& origBuf, RefPicList refPicList, int iRefIdx, Mv& rcMvPred, AMVPInfo& amvpInfo, Distortion& distBiP );
  void       xCheckBestMVP        ( RefPicList refPicList, const Mv& cMv, Mv& rcMvPred, int& riMVPIdx, AMVPInfo& amvpInfo, uint32_t&  ruiBits, Distortion& ruiCost, const uint8_t imv);
  Distortion xGetTemplateCost     ( const CodingUnit& cu, CPelUnitBuf& origBuf, PelUnitBuf& predBuf, Mv cMvCand, int iMVPIdx, int iMVPNum, RefPicList refPicList, int iRefIdx );

  void       xCopyAMVPInfo        ( AMVPInfo* pSrc, AMVPInfo* pDst );
  uint32_t   xGetMvpIdxBits       ( int iIdx, int iNum );
  void       xGetBlkBits          ( bool bPSlice, int iPartIdx,  uint32_t uiLastMode, uint32_t uiBlkBit[3]);


  // -------------------------------------------------------------------------------------------------------------------
  // motion estimation
  // -------------------------------------------------------------------------------------------------------------------

  void xMotionEstimation          ( CodingUnit&           cu,
                                    CPelUnitBuf&          origBuf,
                                    RefPicList            refPicList,
                                    Mv&                   rcMvPred,
                                    int                   iRefIdxPred,
                                    Mv&                   rcMv,
                                    int&                  riMVPIdx,
                                    uint32_t&             ruiBits,
                                    Distortion&           ruiCost,
                                    const AMVPInfo&       amvpInfo,
                                    bool                  bBi = false
                                  );

  void xTZSearch                  ( const CodingUnit&     cu,
                                    RefPicList            refPicList,
                                    int                   iRefIdxPred,
                                    TZSearchStruct&       cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD,
                                    const Mv* const       pIntegerMv2Nx2NPred,
                                    const bool            bExtendedSettings,
                                    const bool            bFastSettings = false
                                  );

  void xTZSearchSelective         ( const CodingUnit&     cu,
                                    RefPicList            refPicList,
                                    int                   iRefIdxPred,
                                    TZSearchStruct&       cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD,
                                    const Mv* const       pIntegerMv2Nx2NPred
                                  );

  void xSetSearchRange            ( const CodingUnit&     cu,
                                    const Mv&             cMvPred,
                                    const int             iSrchRng,
                                    SearchRange&          sr                                  
                                  );

  void xPatternSearchFast         ( const CodingUnit&     cu,
                                    RefPicList            refPicList,
                                    int                   iRefIdxPred,
                                    TZSearchStruct&       cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD,
                                    const Mv* const       pIntegerMv2Nx2NPred
                                  );

  void xPatternSearch             ( TZSearchStruct&       cStruct,
                                    Mv&                   rcMv,
                                    Distortion&           ruiSAD
                                  );

  void xPatternSearchIntRefine    ( CodingUnit&         cu,
                                    TZSearchStruct&     cStruct,
                                    Mv&                 rcMv,
                                    Mv&                 rcMvPred,
                                    int&                riMVPIdx,
                                    uint32_t&           uiBits,
                                    Distortion&         ruiCost,
                                    const AMVPInfo&     amvpInfo,
                                    double              fWeight
                                  );

  void xPatternSearchFracDIF      ( const CodingUnit&     cu,
                                    RefPicList            refPicList,
                                    int                   iRefIdx,
                                    TZSearchStruct&       cStruct,
                                    const Mv&             rcMvInt,
                                    Mv&                   rcMvHalf,
                                    Mv&                   rcMvQter,
                                    Distortion&           ruiCost
                                  );

  void xPredAffineInterSearch     ( CodingUnit&           cu,
                                    CPelUnitBuf&          origBuf,
                                    int                   puIdx,
                                    uint32_t&             lastMode,
                                    Distortion&           affineCost,
                                    Mv                    hevcMv[2][MAX_REF_PICS],
                                    Mv                    mvAffine4Para[2][MAX_REF_PICS][3],
                                    int                   refIdx4Para[2],
                                    uint8_t               BcwIdx = BCW_DEFAULT,
                                    bool                  enforceBcwPred = false,
                                    uint32_t              BcwIdxBits = 0
                                  );

  void  xAffineMotionEstimation  ( CodingUnit&           cu,
                                   CPelUnitBuf&          origBuf,
                                   RefPicList            refPicList,
                                   Mv                    acMvPred[3],
                                   int                   iRefIdxPred,
                                   Mv                    acMv[3],
                                   uint32_t&             ruiBits,
                                   Distortion&           ruiCost,
                                   int&                  mvpIdx,
                                   const AffineAMVPInfo& aamvpi,
                                   bool                  bBi = false
                                 );

  void        xEstimateAffineAMVP     ( CodingUnit& cu, AffineAMVPInfo& affineAMVPInfo, CPelUnitBuf& origBuf, RefPicList refPicList, int iRefIdx, Mv acMvPred[3], Distortion& distBiP);

  Distortion  xGetAffineTemplateCost  ( CodingUnit& cu, CPelUnitBuf& origBuf, PelUnitBuf& predBuf, Mv acMvCand[3], int iMVPIdx, int iMVPNum, RefPicList refPicList, int iRefIdx);
  void        xCopyAffineAMVPInfo     ( AffineAMVPInfo& src, AffineAMVPInfo& dst );
  void        xCheckBestAffineMVP     ( CodingUnit& cu, AffineAMVPInfo &affineAMVPInfo, RefPicList refPicList, Mv acMv[3], Mv acMvPred[3], int& riMVPIdx, uint32_t& ruiBits, Distortion& ruiCost );
  uint32_t    xCalcAffineMVBits       ( CodingUnit& cu, Mv mvCand[3], Mv mvPred[3]);

  Distortion  xGetSymCost             ( const CodingUnit& cu, CPelUnitBuf& origBuf, RefPicList eCurRefPicList, const MvField& cCurMvField, MvField& cTarMvField , int BcwIdx );
  Distortion  xSymRefineMvSearch      ( CodingUnit& cu, CPelUnitBuf& origBuf, Mv& rcMvCurPred, Mv& rcMvTarPred, RefPicList refPicList,
                                        MvField& rCurMvField, MvField& rTarMvField, Distortion uiMinCost, int searchPattern, int nSearchStepShift, uint32_t uiMaxSearchRounds, int BcwIdx );
  void        xSymMotionEstimation    ( CodingUnit& cu, CPelUnitBuf& origBuf, Mv& rcMvCurPred, Mv& rcMvTarPred, RefPicList refPicList, MvField& rCurMvField, MvField& rTarMvField, Distortion& ruiCost, int BcwIdx );
  double      xGetMEDistortionWeight  ( uint8_t BcwIdx, RefPicList refPicList);

  void xSymMvdCheckBestMvp            ( CodingUnit& cu,  CPelUnitBuf& origBuf, Mv curMv, RefPicList curRefList, AMVPInfo amvpInfo[2][MAX_REF_PICS], 
                                        int32_t BcwIdx, Mv cMvPredSym[2], int32_t mvpIdxSym[2], Distortion& bestCost, bool skip );

  void xExtDIFUpSamplingH             ( CPelBuf* pcPattern, bool useAltHpelIf);
  void xExtDIFUpSamplingQ             ( CPelBuf* pcPatternKey, Mv halfPelRef, int& patternId );

  void xEncodeInterResidualQT         ( CodingStructure &cs, Partitioner &partitioner, const ComponentID compID );
  void xEstimateInterResidualQT       ( CodingStructure &cs, Partitioner &partitioner, Distortion *puiZeroDist = NULL );
  uint64_t xGetSymbolFracBitsInter    ( CodingStructure &cs, Partitioner &partitioner );
#if IBC_VTM
  void  xSetIntraSearchRangeIBC       ( CodingUnit& pu, int iRoiWidth, int iRoiHeight, const int localSearchRangeX, const int localSearchRangeY, Mv& rcMvSrchRngLT, Mv& rcMvSrchRngRB);
  void  xIBCEstimation                ( CodingUnit& cu, PelUnitBuf& origBuf, Mv* pcMvPred, Mv& rcMv, Distortion& ruiCost, const int localSearchRangeX, const int localSearchRangeY);
  void  xIBCSearchMVCandUpdate        ( Distortion  uiSad, int x, int y, Distortion* uiSadBestCand, Mv* cMVCand);
  int   xIBCSearchMVChromaRefine      ( CodingUnit& cu, int iRoiWidth, int iRoiHeight, int cuPelX, int cuPelY, Distortion* uiSadBestCand, Mv* cMVCand);
  void  xIntraPatternSearchIBC        ( CodingUnit& pu, TZSearchStruct& cStruct, Mv& rcMv, Distortion& ruiCost, Mv* cMvSrchRngLT, Mv* cMvSrchRngRB, Mv* pcMvPred);
#endif
};// END CLASS DEFINITION EncSearch

} // namespace vvenc

//! \}


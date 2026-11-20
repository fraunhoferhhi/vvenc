/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     EncModeCtrl.h
    \brief    Encoder controller for trying out specific modes
*/

#pragma once

#include "InterSearch.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/CodingStructure.h"

#include <memory>
#include <typeinfo>
#include <vector>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

//////////////////////////////////////////////////////////////////////////
// Encoder modes to try out
//////////////////////////////////////////////////////////////////////////


enum EncTestModeType
{
  ETM_MERGE_SKIP,
  ETM_INTER_ME,
  ETM_INTER_IMV,
  ETM_INTRA,
  ETM_SPLIT_QT,
  ETM_SPLIT_BT_H,
  ETM_SPLIT_BT_V,
  ETM_SPLIT_TT_H,
  ETM_SPLIT_TT_V,
  ETM_RECO_CACHED,
  ETM_IBC,
  ETM_IBC_MERGE,
  ETM_INVALID
};

enum EncTestModeOpts
{
  ETO_STANDARD    =  0,                   // empty      (standard option)
  ETO_FORCE_MERGE =  1<<0,                // bit   0    (indicates forced merge)
  ETO_IMV_SHIFT   =  1,                   // bits  1-3  (imv parameter starts at bit 1)
  ETO_IMV         =  7<<ETO_IMV_SHIFT,    // bits  1-3  (imv parameter uses 3 bits)
  ETO_DUMMY       =  1<<5,                // bit   5    (dummy)
  ETO_INVALID     = 0xffffffff            // bits 0-31  (invalid option)
};

static inline void getAreaIdxNew(const Area& area, const PreCalcValues &pcv, unsigned& idx1, unsigned& idx2, unsigned& idx3, unsigned& idx4)
{
  idx1 = Log2( area.width  )-2;
  idx2 = Log2( area.height )-2;
  idx3 = (area.x & pcv.maxCUSizeMask) >> MIN_CU_LOG2;
  idx4 = (area.y & pcv.maxCUSizeMask) >> MIN_CU_LOG2;
}

struct EncTestMode
{
  EncTestMode()
    : type( ETM_INVALID ), opts( ETO_INVALID  ), qp( -1  ), lossless( false ) {}
  EncTestMode( EncTestModeType _type )
    : type( _type       ), opts( ETO_STANDARD ), qp( -1  ), lossless( false ) {}
  EncTestMode( EncTestModeType _type, int _qp, bool _lossless )
    : type( _type       ), opts( ETO_STANDARD ), qp( _qp ), lossless( _lossless ) {}
  EncTestMode( EncTestModeType _type, EncTestModeOpts _opts, int _qp, bool _lossless )
    : type( _type       ), opts( _opts        ), qp( _qp ), lossless( _lossless ) {}

  EncTestModeType type;
  EncTestModeOpts opts;
  int             qp;
  bool            lossless;
  double          maxCostAllowed;
};


inline bool isModeSplit( const EncTestMode& encTestmode )
{
  switch( encTestmode.type )
  {
  case ETM_SPLIT_QT     :
  case ETM_SPLIT_BT_H   :
  case ETM_SPLIT_BT_V   :
  case ETM_SPLIT_TT_H   :
  case ETM_SPLIT_TT_V   :
    return true;
  default:
    return false;
  }
}

inline bool isModeNoSplit( const EncTestMode& encTestmode )
{
  return !isModeSplit( encTestmode );
}

inline bool isModeInter( const EncTestMode& encTestmode ) // perhaps remove
{
  return (   encTestmode.type == ETM_INTER_ME
          || encTestmode.type == ETM_INTER_IMV
          || encTestmode.type == ETM_MERGE_SKIP
         );
}

inline PartSplit getPartSplit( const EncTestMode& encTestmode )
{
  switch( encTestmode.type )
  {
  case ETM_SPLIT_QT     : return CU_QUAD_SPLIT;
  case ETM_SPLIT_BT_H   : return CU_HORZ_SPLIT;
  case ETM_SPLIT_BT_V   : return CU_VERT_SPLIT;
  case ETM_SPLIT_TT_H   : return CU_TRIH_SPLIT;
  case ETM_SPLIT_TT_V   : return CU_TRIV_SPLIT;
  default:                return CU_DONT_SPLIT;
  }
}

//////////////////////////////////////////////////////////////////////////
// EncModeCtrl controls if specific modes should be tested
//////////////////////////////////////////////////////////////////////////

struct ComprCUCtx
{
  ComprCUCtx()
  {
  }

  ComprCUCtx( const CodingStructure& cs, const uint32_t _minDepth, const uint32_t _maxDepth )
    : minDepth      ( _minDepth  )
    , maxDepth      ( _maxDepth  )
    , bestCS        ( nullptr    )
    , bestCU        ( nullptr    )
    , bestTU        ( nullptr    )
    , bestMode      ()
    , bestCostBeforeSplit   (MAX_DOUBLE)
    , bestCostVertSplit     (MAX_DOUBLE)
    , bestCostHorzSplit     (MAX_DOUBLE)
    , bestCostNoImv         (MAX_DOUBLE *.5)
    , grad_horVal           (0)
    , grad_verVal           (0)
    , grad_dupVal           (0)
    , grad_dowVal           (0)
    , interHad              (MAX_DISTORTION)
    , maxQtSubDepth         (0)
    , isReusingCu           (false)
    , qtBeforeBt            (false)
    , doTriHorzSplit        (false)
    , doTriVertSplit        (false)
    , didQuadSplit          (false)
    , didHorzSplit          (false)
    , didVertSplit          (false)
    , doHorChromaSplit      (false)
    , doVerChromaSplit      (false)
    , doQtChromaSplit       (false)
    , isBestNoSplitSkip     (false)
    , intraWasTested        (false)
    , relatedCuIsValid      (false)
    , isIntra               (false)
    , nonSkipWasTested      (false)
    , bestNsPredMode        (EncTestMode())
    , mttDepthLimit         (0)
    , bestNoSplitCost       (MAX_DOUBLE)
    , mttSplitCost          { MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE }
    , refinableSplits       (0)
    , meCost                (MAX_DOUBLE)
    , meLargestMvd          (-1)
  {
  }

  unsigned          minDepth;
  unsigned          maxDepth;
  CodingStructure*  bestCS;
  CodingUnit*       bestCU;
  TransformUnit*    bestTU;
  EncTestMode       bestMode;
  double            bestCostBeforeSplit;
  double            bestCostVertSplit;
  double            bestCostHorzSplit;
  double            bestCostNoImv;
  double            grad_horVal;
  double            grad_verVal;
  double            grad_dupVal;
  double            grad_dowVal;
  Distortion        interHad;
  int               maxQtSubDepth;
  bool              isReusingCu;
  bool              qtBeforeBt;
  bool              doTriHorzSplit;
  bool              doTriVertSplit;
  int               doMoreSplits;
  bool              didQuadSplit;
  bool              didHorzSplit;
  bool              didVertSplit;
  bool              doHorChromaSplit;
  bool              doVerChromaSplit;
  bool              doQtChromaSplit;
  bool              isBestNoSplitSkip;
  bool              intraWasTested;
  bool              relatedCuIsValid;
  bool              isIntra;
  bool              nonSkipWasTested;
  EncTestMode       bestNsPredMode;
  unsigned          mttDepthLimit;     // the MTT depth limit the split rules, the reuse of stored results and the split-cost table read at this node:
                                       // the partitioner's, one lower in the first pass of FastMTTRefine (EncModeCtrl::xSearchMttDepthLimit)
  // FastMTTRefine; split sets are bit masks of 1 << PartSplit
  double            bestNoSplitCost;   // cost of the best mode without a split plus its deblocking offset
  double            mttSplitCost[4];   // first pass: cost of BT-H, BT-V, TT-H, TT-V as compared, MAX_DOUBLE where not searched
  unsigned          refinableSplits;   // first pass: the splits below which a search of the deepest MTT level was left out
  // FastAMVR: the motion search with quarter-sample MVDs and equal bi-prediction weights
  double            meCost;            // its best RD cost, MAX_DOUBLE where it has none
  int               meLargestMvd;      // the largest MVD component of its result in quarter samples, -1 where it has none
};

//////////////////////////////////////////////////////////////////////////
// some utility interfaces that expose some functionality that can be used without concerning about which particular controller is used
//////////////////////////////////////////////////////////////////////////

static const int MAX_STORED_CU_INFO_REFS = 4;

struct CodedCUInfo
{
  bool      relatedCuIsValid;
  bool      isInter;
  bool      isIntra;
  bool      isSkip;
  bool      isMMVDSkip;
  bool      isIBC;
  uint8_t   BcwIdx;
  uint8_t   numPuInfoStored;
  int       poc, ctuRsAddr;
  bool      validMv[NUM_REF_PIC_LIST_01][MAX_STORED_CU_INFO_REFS];
  Mv        saveMv [NUM_REF_PIC_LIST_01][MAX_STORED_CU_INFO_REFS];
  uint32_t  puSse  [SBT_NUM_SL];
  uint8_t   puSbt  [SBT_NUM_SL];
  double    bestCost;
  uint8_t   refineSplits;   // FastMTTRefine: the BT / TT splits (bits 1 << PartSplit) this block searches again as a child of its MTT root

  bool getMv  ( const RefPicList refPicList, const int iRefIdx,       Mv& rMv ) const;
  void setMv  ( const RefPicList refPicList, const int iRefIdx, const Mv& rMv );
};

class CacheBlkInfoCtrl
{
protected:
  // x in CTU, y in CTU, width, height
  CodedCUInfo*         m_codedCUInfo[6][6][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];
  CodedCUInfo*         m_codedCUInfoBuf;
  const PreCalcValues* m_pcv;

protected:

  void create   ( int ctuSize );
  void destroy  ();
  void init     ( const Slice &slice );

public:
  CacheBlkInfoCtrl() : m_codedCUInfoBuf( nullptr ) {}
  ~CacheBlkInfoCtrl () {}

  CodedCUInfo& getBlkInfo   ( const UnitArea& area );
  void         initBlk      ( const UnitArea& area, int poc );

  uint8_t      findBestSbt  ( const UnitArea& area, const uint32_t curPuSse );
  bool         saveBestSbt  ( const UnitArea& area, const uint32_t curPuSse, const uint8_t curPuSbt );
};

struct BestEncodingInfo
{ 
  CodingUnit      cu;
  TransformUnit   tu;
  EncTestMode     testMode;
  int             poc;
  Distortion      dist;
  double          costEDO;
  uint32_t        storeNumber;   // position of this result among the results stored for the CTU (BestEncInfoCache::m_storeCount)
};

// Hands out runs of T from chunks that are kept for reuse. Runs never move, so pointers into them
// stay valid until reset() makes all chunks available again.
template<typename T, size_t ChunkSize>
class ReusableChunkArena
{
public:
  T* take( const size_t n )
  {
    CHECK( n > ChunkSize, "Run larger than an arena chunk" );
    if( m_current < 0 || m_used + n > ChunkSize )
    {
      m_current++;
      if( m_current == ( int ) m_chunks.size() )
      {
        m_chunks.emplace_back( new T[ChunkSize] );
      }
      m_used = 0;
    }
    T* run  = m_chunks[m_current].get() + m_used;
    m_used += n;
    return run;
  }

  void reset()   { m_current = -1; m_used = 0; }
  void release() { m_chunks.clear(); reset(); }

private:
  std::vector<std::unique_ptr<T[]>> m_chunks;
  int    m_current = -1;
  size_t m_used    = 0;
};

class BestEncInfoCache
{
private:
  const PreCalcValues* m_pcv;
  // The stored result of every CU size and position inside the CTU being encoded, or null. A result
  // only matches a CU of the same picture and position, so none outlives the CTU it was stored for,
  // and init() empties the slots of the previous CTU and reuses their storage.
  BestEncodingInfo*    m_bestEncInfo[6][6][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];
  std::vector<BestEncodingInfo**>           m_filledSlots;
  ReusableChunkArena<BestEncodingInfo, 256> m_entryArena;
  ReusableChunkArena<TCoeffSig, 131072>     m_coeffArena;
  ReusableChunkArena<Mv, 4096>              m_dmvrMvArena;
  ChromaFormat         m_chFmt;
  CodingStructure      m_dummyCS;
  XUCache              m_dummyCache;

  // A stored result is reused only where the CU still has the neighbours it was found with (isTheSameNbHood), which
  // relies on every node being searched once. FastMTTRefine searches the splits of an MTT root a second time: while it
  // does (m_revisitFirstStore > 0), the results stored before are trusted only for what lies outside the root, the
  // first m_revisitSharedLevels levels of the split path.
  uint32_t             m_storeCount;            // results stored for the current CTU
  uint32_t             m_revisitFirstStore;     // m_storeCount when the second search began, 0 while none runs
  int                  m_revisitSharedLevels;

  BestEncodingInfo*    xNewEntry    ( const int wIdx, const int hIdx );
  int                  xSharedLevels( const BestEncodingInfo& encInfo ) const;

protected:
  bool                 m_reuseCuResults;
  void create   ( const bool reuseCuResults, const ChromaFormat chFmt, int ctuSize );
  void destroy  ();
  // around the second search of the splits of an MTT root whose split path has sharedLevels levels (the CTU counts as one)
  void beginRevisit( const int sharedLevels ) { m_revisitFirstStore = m_storeCount; m_revisitSharedLevels = sharedLevels; }
  void endRevisit  ()                         { m_revisitFirstStore = 0; }

public:
  BestEncInfoCache() : m_pcv( nullptr ), m_chFmt( CHROMA_420 ), m_dummyCS( m_dummyCache, nullptr ), m_storeCount( 0 ), m_revisitFirstStore( 0 ), m_revisitSharedLevels( 0 ) {}
  ~BestEncInfoCache() {}

  void init             ( const Slice &slice );
  bool setCsFrom        (       CodingStructure& cs,       EncTestMode& testMode, const Partitioner& partitioner ) const;
  bool setFromCs        ( const CodingStructure& cs, const EncTestMode& testMode, const Partitioner& partitioner );
  // mttDepthLimit: the MTT depth limit the CU's split rules read (ComprCUCtx::mttDepthLimit)
  bool isReusingCuValid ( const CodingStructure &cs, const Partitioner &partitioner, int qp, unsigned mttDepthLimit );
};

//////////////////////////////////////////////////////////////////////////
// EncModeCtrl - allows and controls modes introduced by QTBT (inkl. multi-type-tree)
//                    - only 2Nx2N, no RQT, additional binary/triary CU splits
//////////////////////////////////////////////////////////////////////////

class EncModeCtrl: public CacheBlkInfoCtrl, public BestEncInfoCache
{
protected:

  const VVEncCfg*       m_pcEncCfg;
        RdCost*         m_pcRdCost;
  static_vector<ComprCUCtx, ( MAX_CU_DEPTH << 2 )> m_ComprCUCtxList;
  unsigned              m_skipThresholdE0023FastEnc;

  // FastMTTRefine (EncModeCtrl.cpp): the passes of the BT / TT search below an MTT root
  enum MttSearchPass { MTT_SEARCH_FULL, MTT_SEARCH_CAPPED, MTT_SEARCH_REFINE };
  unsigned              m_mttRefineLimit;   // the MTT depth limit of the inter pictures FastMTTRefine acts on (the highest
                                            // per-layer MaxMTTDepth digit, m_splitCostThrParamId), 0 for none
  MttSearchPass         m_mttSearchPass;

  bool     xRefinesMtt            ( const CodingStructure& cs, const Partitioner& partitioner ) const;
  unsigned xSearchMttDepthLimit   ( const CodingStructure& cs, const Partitioner& partitioner ) const;
  bool     xSkipsDeepestMttLevel  ( const CodingStructure& cs, const Partitioner& partitioner );
  bool     xChoosesSplitsToRefine ( const Partitioner& partitioner ) const;
  unsigned xSplitsToRefine        ( const ComprCUCtx& cuECtx ) const;
  void     xStoreSplitsToRefine   ( const Partitioner& partitioner );
  void     xNoteMotionSearchCost  ( const EncTestMode& encTestmode, const CodingStructure& tempCS, const bool useEDO );
  bool     xKeepIfBest            ( const EncTestMode& encTestmode, CodingStructure* tempCS, const bool useEDO );

public:
  ComprCUCtx*           comprCUCtx;

  ~EncModeCtrl    () { destroy(); }

  void init               ( const VVEncCfg& encCfg, RdCost *pRdCost );
  void destroy            ();
  void initCTUEncoding    ( const Slice &slice );
  // cuLeft / cuAbove: the restricted left and above neighbours of the CU (DeriveCtx::determineNeighborCus)
  void initCULevel        ( Partitioner &partitioner, const CodingStructure& cs, int  MergeSimpleFlag, const CodingUnit* cuLeft, const CodingUnit* cuAbove );
  void finishCULevel      ( Partitioner &partitioner );

  bool tryMode            ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner );
  bool trySplit           ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner, const EncTestMode& lastTestmode );
  bool useModeResult      ( const EncTestMode& encTestmode, CodingStructure*& tempCS,  Partitioner& partitioner, const bool useEDO );

  void beforeSplit        ( Partitioner& partitioner );

  // FastMTTRefine, called by EncCu::xCompressCU at a node away from the picture border: before the node's BT / TT
  // splits (true: the node is an MTT root FastMTTRefine acts on, and its first pass begins), and after them for such
  // a root (ends the first pass and begins the second, returning the splits the caller searches again, bits
  // 1 << PartSplit), then once the caller has searched them
  bool     beginCappedSplitPass ( const CodingStructure& cs, const Partitioner& partitioner );
  unsigned beginSplitRefinePass ( const Partitioner& partitioner );
  void     endSplitRefinePass   ();

  // FastAMVR: noteMotionSearchMvd keeps the largest MVD of the current node's motion search with equal bi-prediction weights;
  // skipsAmvrSearch and skips4PelAmvr say where the AMVR search, or its 4-sample MVDs, cannot pay
  void     noteMotionSearchMvd  ( const CodingUnit& cu );
  bool     skipsAmvrSearch      ( const CodingStructure& bestCS ) const;
  bool     skips4PelAmvr        () const;
};

} // namespace vvenc

//! \}


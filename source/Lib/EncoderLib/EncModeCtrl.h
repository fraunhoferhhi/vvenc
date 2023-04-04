/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
  ETM_AFFINE,
  ETM_MERGE_GEO,
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
          || encTestmode.type == ETM_AFFINE
          || encTestmode.type == ETM_MERGE_GEO
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
    , bestInterCost             ( MAX_DOUBLE )
    , bestCostBeforeSplit       ( MAX_DOUBLE )
    , bestCostVertSplit     (MAX_DOUBLE)
    , bestCostHorzSplit     (MAX_DOUBLE)
    , bestCostTriVertSplit  (MAX_DOUBLE)
    , bestCostTriHorzSplit  (MAX_DOUBLE)
    , bestCostImv           (MAX_DOUBLE *.5)
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
    , skipSecondMTSPass     (false)
    , intraWasTested        (false)
    , relatedCuIsValid      (false)
    , bestIntraMode         (0)
    , isIntra               (false)
    , nonSkipWasTested      (false)
  {
  }

  unsigned          minDepth;
  unsigned          maxDepth;
  CodingStructure*  bestCS;
  CodingUnit*       bestCU;
  TransformUnit*    bestTU;
  EncTestMode       bestMode;
  double            bestInterCost;
  double            bestCostBeforeSplit;
  double            bestCostVertSplit;
  double            bestCostHorzSplit;
  double            bestCostTriVertSplit;
  double            bestCostTriHorzSplit;
  double            bestCostImv;
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
  bool              skipSecondMTSPass;
  bool              intraWasTested;
  bool              relatedCuIsValid;
  int               bestIntraMode;
  bool              isIntra;
  bool              nonSkipWasTested;
};

//////////////////////////////////////////////////////////////////////////
// some utility interfaces that expose some functionality that can be used without concerning about which particular controller is used
//////////////////////////////////////////////////////////////////////////

static const int MAX_STORED_CU_INFO_REFS = 4;

struct CodedCUInfo
{
  bool isInter;
  bool isIntra;
  bool isSkip;
  bool isMMVDSkip;
  int  isMergeSimple;
  bool isIBC;
  uint8_t BcwIdx;
  int  ctuRsAddr, poc;
  uint8_t  numPuInfoStored;
  bool validMv  [NUM_REF_PIC_LIST_01][MAX_STORED_CU_INFO_REFS];
  Mv   saveMv   [NUM_REF_PIC_LIST_01][MAX_STORED_CU_INFO_REFS];
  uint32_t puSse[SBT_NUM_SL];
  uint8_t  puSbt[SBT_NUM_SL];
  double bestCost;
  bool   relatedCuIsValid;
  int    bestIntraMode;

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

  void create   ();
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
};

class BestEncInfoCache
{
private:
  const PreCalcValues* m_pcv;
  BestEncodingInfo*    m_bestEncInfo[6][6][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];
  TCoeffSig*           m_pCoeff;
  BestEncodingInfo*    m_encInfoBuf;
  Mv*                  m_dmvrMvBuf;
  CodingStructure      m_dummyCS;
  XUCache              m_dummyCache;

protected:

  void create   ( const ChromaFormat chFmt );
  void destroy  ();
public:
  BestEncInfoCache() : m_pcv( nullptr ), m_pCoeff( nullptr ), m_encInfoBuf( nullptr ), m_dmvrMvBuf( nullptr ), m_dummyCS( m_dummyCache, nullptr ) {}
  ~BestEncInfoCache() {}

  void init             ( const Slice &slice );
  bool setCsFrom        (       CodingStructure& cs,       EncTestMode& testMode, const Partitioner& partitioner ) const;
  bool setFromCs        ( const CodingStructure& cs, const EncTestMode& testMode, const Partitioner& partitioner );
  bool isReusingCuValid ( const CodingStructure &cs, const Partitioner &partitioner, int qp );
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
  unsigned              m_tileIdx;

public:
  ComprCUCtx*           comprCUCtx;

  ~EncModeCtrl    () { destroy(); }

  void init               ( const VVEncCfg& encCfg, RdCost *pRdCost );
  void destroy            ();
  void initCTUEncoding    ( const Slice &slice, int tileIdx );
  void initCULevel        ( Partitioner &partitioner, const CodingStructure& cs, int  MergeSimpleFlag );
  void finishCULevel      ( Partitioner &partitioner );

  bool tryMode            ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner );
  bool trySplit           ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner, const EncTestMode& lastTestmode );
  bool useModeResult      ( const EncTestMode& encTestmode, CodingStructure*& tempCS,  Partitioner& partitioner, const bool useEDO );

  void beforeSplit        ( Partitioner& partitioner );
};

} // namespace vvenc

//! \}


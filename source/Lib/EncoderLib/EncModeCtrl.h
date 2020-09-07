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

inline EncTestMode getCSEncMode( const CodingStructure& cs )
{
  return EncTestMode( EncTestModeType( (unsigned)cs.features[ENC_FT_ENC_MODE_TYPE] ),
                      EncTestModeOpts( (unsigned)cs.features[ENC_FT_ENC_MODE_OPTS] ),
                      false);
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
    , bestInterCost             ( MAX_DOUBLE )
    , bestCostVertSplit     (MAX_DOUBLE)
    , bestCostHorzSplit     (MAX_DOUBLE)
    , bestCostTriVertSplit  (MAX_DOUBLE)
    , bestCostTriHorzSplit  (MAX_DOUBLE)
    , bestCostImv           (MAX_DOUBLE *.5)
    , bestCostNoImv         (MAX_DOUBLE *.5)
    , interHad              (MAX_DISTORTION)
    , maxQtSubDepth         (0)
    , earlySkip             (false)
    , isReusingCu           (false)
    , qtBeforeBt            (false)
    , doTriHorzSplit        (false)
    , doTriVertSplit        (false)
    , didQuadSplit          (false)
    , didHorzSplit          (false)
    , didVertSplit          (false)
    , isBestNoSplitSkip     (false)
    , skipSecondMTSPass     (false)
  {
  }

  unsigned          minDepth;
  unsigned          maxDepth;
  CodingStructure*  bestCS;
  CodingUnit*       bestCU;
  TransformUnit*    bestTU;
  double            bestInterCost;
  double            bestCostVertSplit;
  double            bestCostHorzSplit;
  double            bestCostTriVertSplit;
  double            bestCostTriHorzSplit;
  double            bestCostImv;
  double            bestCostNoImv;
  Distortion        interHad;
  int               maxQtSubDepth;
  bool              earlySkip;
  bool              isReusingCu;
  bool              qtBeforeBt;
  bool              doTriHorzSplit;
  bool              doTriVertSplit;
  int               doMoreSplits;
  bool              didQuadSplit;
  bool              didHorzSplit;
  bool              didVertSplit;
  bool              isBestNoSplitSkip;
  bool              skipSecondMTSPass;
};

//////////////////////////////////////////////////////////////////////////
// some utility interfaces that expose some functionality that can be used without concerning about which particular controller is used
//////////////////////////////////////////////////////////////////////////
struct SaveLoadStructSbt
{
  uint8_t  numPuInfoStored;
  uint32_t puSse[SBT_NUM_SL];
  uint8_t  puSbt[SBT_NUM_SL];
};

class SaveLoadEncInfoSbt
{
protected:
  void init( const Slice &slice );
  void create();
  void destroy();

private:
  SaveLoadStructSbt m_saveLoadSbt[6][6][32][32];
  const PreCalcValues* m_pcv;

public:
  virtual  ~SaveLoadEncInfoSbt() { }
  void     resetSaveloadSbt   ( int maxSbtSize );
  uint8_t  findBestSbt        ( const UnitArea& area, const uint32_t curPuSse );
  bool     saveBestSbt        ( const UnitArea& area, const uint32_t curPuSse, const uint8_t curPuSbt );
};

static const int MAX_STORED_CU_INFO_REFS = 4;

struct CodedCUInfo
{
  bool isInter;
  bool isIntra;
  bool isSkip;
  bool isMMVDSkip;
  bool isIBC;
  bool validMv[NUM_REF_PIC_LIST_01][MAX_STORED_CU_INFO_REFS];
  Mv   saveMv [NUM_REF_PIC_LIST_01][MAX_STORED_CU_INFO_REFS];

  uint8_t BcwIdx;
  bool getMv  ( const RefPicList refPicList, const int iRefIdx,       Mv& rMv ) const;
  void setMv  ( const RefPicList refPicList, const int iRefIdx, const Mv& rMv );
};

class CacheBlkInfoCtrl
{
protected:
  // x in CTU, y in CTU, width, height
  CodedCUInfo*         m_codedCUInfo[6][6][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];
  const PreCalcValues* m_pcv;

protected:

  void create   ();
  void destroy  ();
  void init     ( const Slice &slice );

public:
  virtual ~CacheBlkInfoCtrl() {}

  CodedCUInfo& getBlkInfo( const UnitArea& area );
};

struct BestEncodingInfo
{
  CodingUnit     cu;
  PredictionUnit pu;
  TransformUnit  tu;
  EncTestMode    testMode;
  int            poc;
  Distortion     dist;
  double         costEDO;
};

class BestEncInfoCache
{
private:
  const PreCalcValues* m_pcv;
  BestEncodingInfo*    m_bestEncInfo[6][6][MAX_CU_SIZE >> MIN_CU_LOG2][MAX_CU_SIZE >> MIN_CU_LOG2];
  TCoeff*              m_pCoeff;
  Pel*                 m_pPcmBuf;
  bool*                m_runType;
  CodingStructure      m_dummyCS;
  XUCache              m_dummyCache;

protected:

  void create   ( const ChromaFormat chFmt );
  void destroy  ();
public:
  BestEncInfoCache() : m_pcv( nullptr ), m_pCoeff( nullptr ), m_pPcmBuf( nullptr ), m_runType( nullptr ), m_dummyCS( m_dummyCache, nullptr ) {}
  virtual ~BestEncInfoCache() {}

  void init             ( const Slice &slice );
  bool setCsFrom        ( CodingStructure& cs, EncTestMode& testMode, const Partitioner& partitioner ) const;
  bool setFromCs        ( const CodingStructure& cs, const Partitioner& partitioner );
  bool isReusingCuValid ( const CodingStructure &cs, const Partitioner &partitioner, int qp );
};

//////////////////////////////////////////////////////////////////////////
// EncModeCtrl - allows and controls modes introduced by QTBT (inkl. multi-type-tree)
//                    - only 2Nx2N, no RQT, additional binary/triary CU splits
//////////////////////////////////////////////////////////////////////////

class EncModeCtrl: public CacheBlkInfoCtrl, public BestEncInfoCache, public SaveLoadEncInfoSbt
{
protected:

  const EncCfg*         m_pcEncCfg;
        RdCost*         m_pcRdCost;
  static_vector<ComprCUCtx, ( MAX_CU_DEPTH << 2 )> m_ComprCUCtxList;
  unsigned              m_skipThresholdE0023FastEnc;

public:
  ComprCUCtx*           comprCUCtx;

  virtual ~EncModeCtrl    () { destroy(); }

  void init               ( const EncCfg& encCfg, RdCost *pRdCost );
  void destroy            ();
  void initCTUEncoding    ( const Slice &slice );
  void initCULevel        ( Partitioner &partitioner, const CodingStructure& cs );
  void finishCULevel      ( Partitioner &partitioner );

  bool tryMode            ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner );
  bool trySplit           ( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner, const EncTestMode& lastTestmode );
  bool useModeResult      ( const EncTestMode& encTestmode, CodingStructure*& tempCS,  Partitioner& partitioner, const bool useEDO );

  void beforeSplit        ( Partitioner& partitioner );

private:
  void xExtractFeatures   ( const EncTestMode& encTestmode, CodingStructure& cs );

};

} // namespace vvenc

//! \}


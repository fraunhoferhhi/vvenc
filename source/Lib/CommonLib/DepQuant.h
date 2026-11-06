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
#pragma once

#include "CommonDef.h"
#include "Contexts.h"
#include "Slice.h"
#include "Unit.h"
#include "UnitPartitioner.h"
#include "QuantRDOQ2.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

namespace DQIntern
{
  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   R A T E   E S T I M A T O R                                        =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  struct NbInfoSbb
  {
    uint8_t   numInv;
    uint8_t   invInPos[5];
  };
  struct NbInfoOut
  {
    uint16_t  maxDist;
    uint16_t  num;
    uint16_t  outPos[5];
  };
  // For TU shapes with 4x4 SBBs, where the outside neighbours of an SBB's positions (NbInfoOut) lie: in its right,
  // below or diagonal SBB. start[] is the start of that SBB in scan order relative to the SBB's own start, or -1 when
  // it lies outside the non-zero-out region.
  enum NbSbb { NB_SBB_RIGHT = 0, NB_SBB_BELOW = 1, NB_SBB_DIAG = 2, NUM_NB_SBB = 3 };
  struct NbSbbStarts
  {
    int16_t   start[NUM_NB_SBB];
  };
  // The same for every TU shape with 4x4 SBBs: for neighbour k of the position p of an SBB (k as in the template:
  // (x+1,y), (x+2,y), (x+1,y+1), (x,y+1), (x,y+2)), its position inside neighbouring SBB s as idx[k][s][p], or 0x80
  // when it does not lie there. Used as pshufb indices.
  struct NbOutShuffle16
  {
    uint8_t   idx[5][NUM_NB_SBB][16];
  };
  struct CoeffFracBits
  {
    int32_t   bits[6];
  };

  enum ScanPosType : int8_t { SCAN_ISCSBB = 0, SCAN_SOCSBB = 1, SCAN_EOCSBB = 2 };

  struct ScanInfo
  {
    ScanInfo() {}
    short         numSbb;
    short         scanIdx;
    short         rasterPos;
    short         sbbPos; // byte
    short         nextSbbRight;
    short         nextSbbBelow;
    int8_t        sbbSize;
    int8_t        insidePos;
    int8_t        nextInsidePos;
    ScanPosType   spt;
    int8_t        posX;
    int8_t        posY;
    int8_t        sigCtxOffsetNext;
    int8_t        gtxCtxOffsetNext;
    NbInfoSbb     currNbInfoSbb;
  };

  class Rom;
  struct TUParameters
  {
    TUParameters ( const Rom& rom, const unsigned width, const unsigned height, const ChannelType chType );
    ~TUParameters()
    {
      delete [] m_scanInfo;
    }

    ChannelType       m_chType;
    unsigned          m_width;
    unsigned          m_height;
    unsigned          m_numCoeff;
    unsigned          m_numSbb;
    unsigned          m_log2SbbWidth;
    unsigned          m_log2SbbHeight;
    unsigned          m_log2SbbSize;
    unsigned          m_sbbSize;
    unsigned          m_sbbMask;
    unsigned          m_widthInSbb;
    unsigned          m_heightInSbb;
    const ScanElement *m_scanSbbId2SbbPos;
    const ScanElement *m_scanId2BlkPos;
    const NbInfoSbb*  m_scanId2NbInfoSbb;
    const NbInfoOut*  m_scanId2NbInfoOut;
    const NbSbbStarts* m_scanSbb2NbSbbStarts;   // per SBB in scan order; nullptr unless the SBBs are 4x4
    const NbOutShuffle16* m_nbOutShuffle16;
    ScanInfo*         m_scanInfo;
  private:
    void xSetScanInfo( ScanInfo& scanInfo, int scanIdx );
  };

  class Rom
  {
  public:
    Rom() : m_scansInitialized(false) {}
    ~Rom() { xUninitScanArrays(); }
    void                init        ()                       { xInitScanArrays(); }
    const NbInfoSbb*    getNbInfoSbb( int hd, int vd ) const { return m_scanId2NbInfoSbbArray[hd][vd]; }
    const NbInfoOut*    getNbInfoOut( int hd, int vd ) const { return m_scanId2NbInfoOutArray[hd][vd]; }
    const NbSbbStarts*  getNbSbbStarts( int hd, int vd ) const { return m_scanSbb2NbSbbStartsArray[hd][vd]; }
    const NbOutShuffle16& getNbOutShuffle16() const          { return m_nbOutShuffle16; }
    const TUParameters* getTUPars   ( const CompArea& area, const ComponentID compID ) const
    {
      return m_tuParameters[Log2(area.width)][Log2(area.height)][toChannelType(compID)];
    }
  private:
    void  xInitScanArrays   ();
    void  xUninitScanArrays ();
    void  xSetNbSbb16       ( NbSbbStarts& starts, const int cpos[5], int x, int y, int insidePos, int begSbb );
    void  xCheckNbSbb16     ( const NbSbbStarts* starts, const NbInfoOut* nbOut, unsigned numCoeff ) const;
  private:
    bool          m_scansInitialized;
    NbInfoSbb*    m_scanId2NbInfoSbbArray[ MAX_TU_SIZE_IDX ][ MAX_TU_SIZE_IDX ];
    NbInfoOut*    m_scanId2NbInfoOutArray[ MAX_TU_SIZE_IDX ][ MAX_TU_SIZE_IDX ];
    NbSbbStarts*  m_scanSbb2NbSbbStartsArray[ MAX_TU_SIZE_IDX ][ MAX_TU_SIZE_IDX ];
    NbOutShuffle16 m_nbOutShuffle16;
    TUParameters* m_tuParameters         [ MAX_TU_SIZE_IDX ][ MAX_TU_SIZE_IDX ][ MAX_NUM_CH ];
  };

  struct RateTablesMemo;

  class RateEstimator
  {
  public:
    RateEstimator () {}
    ~RateEstimator() {}
    void initCtx  ( const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID, const FracBitsAccess& fracBitsAccess,
                    RateTablesMemo& memo );

    inline const BinFracBits *sigSbbFracBits() const { return m_tables->sigSbb; }
    inline const BinFracBits *sigFlagBits(unsigned stateId) const
    {
      return m_tables->sig[std::max(((int) stateId) - 1, 0)];
    }
    inline const CoeffFracBits *gtxFracBits() const { return m_tables->gtx; }
    inline int32_t              lastOffset(unsigned scanIdx) const
    {
      return m_lastBitsX[m_scanId2Pos[scanIdx].x] + m_lastBitsY[m_scanId2Pos[scanIdx].y];
    }

  private:
    void  xSetLastCoeffOffset ( const FracBitsAccess& fracBitsAccess, const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID );

  public:
    static const unsigned sm_numCtxSetsSig    = 3;
    static const unsigned sm_numCtxSetsGtx    = 2;
    static const unsigned sm_maxNumSigSbbCtx  = 2;
    static const unsigned sm_maxNumSigCtx     = 12;
    static const unsigned sm_maxNumGtxCtx     = 21;
    // the coefficient-coding contexts the sig / gtx tables are built from, for both channel types: SigCoeffGroup[2],
    // SigFlag[6], ParFlag[2] and GtxFlag[4], defined back to back (checked in RateTablesMemo's constructor)
    static const unsigned sm_numCoeffCtx      = 4 + 60 + 32 + 64;
    static const unsigned sm_coeffCtxBytes    = sm_numCoeffCtx * sizeof( BinProbModel );

    // the tables of one channel type
    struct RateTables
    {
      BinFracBits   sigSbb[ sm_maxNumSigSbbCtx ];
      BinFracBits   sig   [ sm_numCtxSetsSig ][ sm_maxNumSigCtx ];
      CoeffFracBits gtx   [ sm_maxNumGtxCtx ];
    };

  private:
    void  xSetSigSbbFracBits  ( const FracBitsAccess& fracBitsAccess, ChannelType chType, RateTables& tables );
    void  xSetSigFlagBits     ( const FracBitsAccess& fracBitsAccess, ChannelType chType, RateTables& tables );
    void  xSetGtxFlagBits     ( const FracBitsAccess& fracBitsAccess, ChannelType chType, RateTables& tables );

    const ScanElement * m_scanId2Pos;
    int32_t             m_lastBitsX      [ MAX_TB_SIZEY ];
    int32_t             m_lastBitsY      [ MAX_TB_SIZEY ];
    const RateTables*   m_tables = nullptr;   // the tables of the current block's channel type, held in the memo
  };

  // The memo of RateEstimator::initCtx: per channel type, the sig / gtx tables last built for it and the raw bytes of
  // the coefficient-coding contexts they came from (equal bytes give equal tables). DepQuant keeps it after the trellis
  // and state arrays, so they keep their place in the object.
  struct RateTablesMemo
  {
    RateTablesMemo();
    struct Channel
    {
      uint8_t                   ctxStates[ RateEstimator::sm_coeffCtxBytes ];
      RateEstimator::RateTables tables;
    };
    Channel channel[ MAX_NUM_CH ];
  };

  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   P R E - Q U A N T I Z E R                                          =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  struct PQData
  {
    TCoeff  absLevel;
    int64_t deltaDist;
  };

  // What Quantizer::initQuantBlock works out for a block.
  struct QuantBlockParams
  {
    // quantization
    int               m_QShift;
    int64_t           m_QAdd;
    int64_t           m_QScale;
    TCoeff            m_maxQIdx;
    TCoeff            m_thresLast;
    TCoeff            m_thresSSbb;
    TCoeff            m_thresDefault;   // the threshold on absolute coefficients without scaling lists
    // distortion normalization
    int               m_DistShift;
    int64_t           m_DistAdd;
    int64_t           m_DistStepAdd;
    int64_t           m_DistOrgFact;
  };

  // The memo of Quantizer::initQuantBlock with no gValue. With the transform skip path excluded, the result depends
  // only on the channel type, the block size, QP and lambda, and on values fixed for the sequence, and those four rarely
  // change from one block of a size to the next. Log2( width ) + Log2( height ) is enough for the size: it gives the transform
  // shift and whether the sqrt(2) adjustment applies.
  struct QuantBlockMemo
  {
    struct Entry
    {
      double           lambda = 0;
      int              qpDQ   = std::numeric_limits<int>::min();
      QuantBlockParams params;
    };
    Entry entry[ MAX_NUM_CH ][ 2 * 6 + 1 ];
  };

  class Quantizer : public QuantBlockParams
  {
  public:
    Quantizer() {}
    void   init            ( int dqThrVal ) { m_DqThrVal = dqThrVal; }
    void   initQuantBlock  ( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda, QuantBlockMemo& memo );
    void   dequantBlock    ( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff, bool enableScalingLists, int* piDequantCoef ) const;
    void   initQuantBlock  ( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda, int gValue = -1 );
    bool   preQuantCoeff   ( const TCoeff absCoeff, PQData *pqData, int quanCoeff ) const;
    TCoeff getLastThreshold() const { return m_thresLast; }
    TCoeff getSSbbThreshold() const { return m_thresSSbb; }

    int64_t getQScale      () const { return m_QScale; }

    int               m_DqThrVal;
  };

#define RICEMAX 32
  extern const int32_t g_goRiceBits[4][RICEMAX];

  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q   S T A T E                                                  =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  struct StateMem;

  struct SbbCtx
  {
    uint8_t* sbbFlags;
    uint8_t* levels;
  };

  class CommonCtx
  {
  friend class DepQuant;

  public:
    CommonCtx() : m_currSbbCtx( m_allSbbCtx ), m_prevSbbCtx( m_currSbbCtx + 4 ) {}

    inline void swap() { std::swap( m_currSbbCtx, m_prevSbbCtx ); }

    inline void reset( const TUParameters& tuPars, const RateEstimator& rateEst )
    {
      m_nbInfo = tuPars.m_scanId2NbInfoOut;
      m_nbSbbStarts = tuPars.m_scanSbb2NbSbbStarts;
      m_nbOutShuffle16 = tuPars.m_nbOutShuffle16;
      ::memcpy( m_sbbFlagBits, rateEst.sigSbbFracBits(), 2 * sizeof( BinFracBits ) );
      const int numSbb = tuPars.m_numSbb;
      const int chunkSize = numSbb + tuPars.m_numCoeff;
      uint8_t* nextMem = m_memory;
      for( int k = 0; k < 8; k++, nextMem += chunkSize )
      {
        m_allSbbCtx[k].sbbFlags = nextMem;
        m_allSbbCtx[k].levels = nextMem + numSbb;
      }
    }

    // At the start of an SBB: takes over the SBB flags and the levels of the SBBs coded so far from the context prevId,
    // and sets the template sums (tplAcc, sum1st) of the new SBB's positions from their neighbours in those SBBs.
    void update( const ScanInfo& scanInfo, const int prevId, int stateId, StateMem& curr )
    {
      if( const uint8_t* absLevels = updateSbbFlags( scanInfo, prevId, stateId, curr ) )
      {
        updateTemplates( scanInfo, absLevels, stateId, curr );
      }
    }
    // The first part of update(). Returns the levels from the new SBB's start when its template sums are needed (a
    // neighbouring SBB has significant coefficients), else nullptr: the sums stay 0.
    const uint8_t* updateSbbFlags( const ScanInfo& scanInfo, const int prevId, int stateId, StateMem& curr );

    // For TUs with 4x4 SBBs, else nullptr: where the outside neighbours of the new SBB's positions lie.
    const NbSbbStarts* nbSbbStarts( const ScanInfo& scanInfo ) const
    {
      return m_nbSbbStarts ? m_nbSbbStarts + ( ( scanInfo.scanIdx - scanInfo.sbbSize ) >> 4 ) : nullptr;
    }
    const NbOutShuffle16& nbOutShuffle16() const { return *m_nbOutShuffle16; }

    void getLevelPtrs( const ScanInfo& scanInfo, uint8_t*& levels0, uint8_t*& levels1, uint8_t*& levels2, uint8_t*& levels3 )
    {
      levels0 = m_currSbbCtx[0].levels + scanInfo.scanIdx;
      levels1 = m_currSbbCtx[1].levels + scanInfo.scanIdx;
      levels2 = m_currSbbCtx[2].levels + scanInfo.scanIdx;
      levels3 = m_currSbbCtx[3].levels + scanInfo.scanIdx;
    }

  private:
    // The second part of update(): the template sums of the new SBB's 16 positions for stateId. Expects them to be 0.
    void updateTemplates( const ScanInfo& scanInfo, const uint8_t* absLevels, int stateId, StateMem& curr ) const;

    const NbInfoOut* m_nbInfo;
    const NbSbbStarts* m_nbSbbStarts;
    const NbOutShuffle16* m_nbOutShuffle16;
    BinFracBits      m_sbbFlagBits[2];
    SbbCtx           m_allSbbCtx[8];
    SbbCtx*          m_currSbbCtx;
    SbbCtx*          m_prevSbbCtx;
    uint8_t          m_memory[8 * ( MAX_TB_SIZEY * MAX_TB_SIZEY + MLS_GRP_NUM )];
  };

  static constexpr int64_t rdCostInit = std::numeric_limits<int64_t>::max() >> 1;

  struct Decisions
  {
    int64_t   rdCost[4];
    TCoeffSig absLevel[4];
    int8_t    prevId[4];
  };

  struct StateMem
  {
    int64_t  rdCost[4];
    int16_t  remRegBins[4];
    int32_t  sbbBits0[4];
    int32_t  sbbBits1[4];

    uint8_t tplAcc[16][4];
    uint8_t sum1st[16][4];
    uint8_t absVal[16][4];

    struct
    {
      uint8_t sig[4];
      uint8_t cff[4];
    } ctx;

    uint8_t  numSig[4];
    int8_t   refSbbCtxId[4];

    int32_t  cffBits1[RateEstimator::sm_maxNumGtxCtx + 3];

    int8_t   m_goRicePar[4];
    int8_t   m_goRiceZero[4];
    const BinFracBits* m_sigFracBitsArray[4];
    const CoeffFracBits* m_gtxFracBitsArray;

    int      cffBitsCtxOffset;
    bool     anyRemRegBinsLt4;
    int      initRemRegBins;
  };

  static constexpr size_t StateMemSkipCpySize = offsetof( StateMem, sbbBits1 );
}

class DepQuant : public QuantRDOQ2, DQIntern::RateEstimator
{
public:
  DepQuant( const Quant* other, bool enc, bool useScalingLists, bool enableOpt = true );
  virtual ~DepQuant();

  virtual void quant  (       TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx& ctx );
  virtual void dequant( const TransformUnit& tu, CoeffBuf& dstCoeff, const ComponentID compID, const QpParam& cQP );
  
  virtual void init   ( int rdoq = 0, bool useRDOQTS = false, int dqThrVal = 8 );

private:
  void    xQuantDQ          ( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP, const double lambda, const Ctx& ctx, TCoeff& absSum, bool enableScalingLists, int* quantCoeff );
  void    xDequantDQ        ( const TransformUnit& tu, CoeffBuf& recCoeff, const ComponentID compID, const QpParam& cQP, bool enableScalingLists, int* quantCoeff );
  void    xDecideAndUpdate  ( const TCoeff absCoeff, const DQIntern::ScanInfo& scanInfo, bool zeroOut, int quantCoeff);
  void    xDecide           ( const DQIntern::ScanInfo &scanInfo, const TCoeff absCoeff, const int lastOffset, DQIntern::Decisions &decisions, bool zeroOut, int quantCoeff );

  DQIntern::CommonCtx m_commonCtx;
  std::shared_ptr<DQIntern::Rom>
                      m_scansRom;
  DQIntern::Quantizer m_quant;

  DQIntern::Decisions m_trellis[MAX_TB_SIZEY * MAX_TB_SIZEY][2];
  DQIntern::StateMem  m_state_curr;
  DQIntern::StateMem  m_state_skip;
  DQIntern::RateTablesMemo m_rateTablesMemo;   // after the trellis and state arrays: see RateTablesMemo
  DQIntern::QuantBlockMemo m_quantBlockMemo;

  // has to be called as a first check, assumes no decision has been made yet!!!
  void( *m_checkAllRdCosts )( const DQIntern::ScanPosType spt, const DQIntern::PQData* pqData, DQIntern::Decisions& decisions, const DQIntern::StateMem& state );

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_QUANT
  void initDepQuantX86();
  template <X86_VEXT vext>
  void _initDepQuantX86();
#endif
#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_QUANT
  void initDepQuantARM();
  template <ARM_VEXT vext>
  void _initDepQuantARM();
#endif

public:
  // has to be called as a first check, assumes no decision has been made yet!!!
  void( *m_checkAllRdCostsOdd1 )( const DQIntern::ScanPosType spt, const int64_t pq_a_dist, const int64_t pq_b_dist, DQIntern::Decisions& decisions, const DQIntern::StateMem& state );
  void( *m_updateStates )( const DQIntern::ScanInfo& scanInfo, const DQIntern::Decisions& decisions, DQIntern::StateMem& curr );
  void( *m_updateStatesEOS )( const DQIntern::ScanInfo& scanInfo, const DQIntern::Decisions& decisions, const DQIntern::StateMem& skip, DQIntern::StateMem& curr, DQIntern::CommonCtx& commonCtx );
  void( *m_findFirstPos )( int& firstTestPos, const TCoeff* tCoeff, const DQIntern::TUParameters& tuPars, int defaultTh, bool zeroOutForThres, int zeroOutWidth, int zeroOutHeight );
};

} // namespace vvenc

//! \}


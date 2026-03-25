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
    const TUParameters* getTUPars   ( const CompArea& area, const ComponentID compID ) const
    {
      return m_tuParameters[Log2(area.width)][Log2(area.height)][toChannelType(compID)];
    }
  private:
    void  xInitScanArrays   ();
    void  xUninitScanArrays ();
  private:
    bool          m_scansInitialized;
    NbInfoSbb*    m_scanId2NbInfoSbbArray[ MAX_TU_SIZE_IDX ][ MAX_TU_SIZE_IDX ];
    NbInfoOut*    m_scanId2NbInfoOutArray[ MAX_TU_SIZE_IDX ][ MAX_TU_SIZE_IDX ];
    TUParameters* m_tuParameters         [ MAX_TU_SIZE_IDX ][ MAX_TU_SIZE_IDX ][ MAX_NUM_CH ];
  };

  class RateEstimator
  {
  public:
    RateEstimator () {}
    ~RateEstimator() {}
    void initCtx  ( const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID, const FracBitsAccess& fracBitsAccess );

    inline const BinFracBits *sigSbbFracBits() const { return m_sigSbbFracBits; }
    inline const BinFracBits *sigFlagBits(unsigned stateId) const
    {
      return m_sigFracBits[std::max(((int) stateId) - 1, 0)];
    }
    inline const CoeffFracBits *gtxFracBits() const { return m_gtxFracBits; }
    inline int32_t              lastOffset(unsigned scanIdx) const
    {
      return m_lastBitsX[m_scanId2Pos[scanIdx].x] + m_lastBitsY[m_scanId2Pos[scanIdx].y];
    }

  private:
    void  xSetLastCoeffOffset ( const FracBitsAccess& fracBitsAccess, const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID );
    void  xSetSigSbbFracBits  ( const FracBitsAccess& fracBitsAccess, ChannelType chType );
    void  xSetSigFlagBits     ( const FracBitsAccess& fracBitsAccess, ChannelType chType );
    void  xSetGtxFlagBits     ( const FracBitsAccess& fracBitsAccess, ChannelType chType );

  public:
    static const unsigned sm_numCtxSetsSig    = 3;
    static const unsigned sm_numCtxSetsGtx    = 2;
    static const unsigned sm_maxNumSigSbbCtx  = 2;
    static const unsigned sm_maxNumSigCtx     = 12;
    static const unsigned sm_maxNumGtxCtx     = 21;

  private:
    const ScanElement * m_scanId2Pos;
    int32_t             m_lastBitsX      [ MAX_TB_SIZEY ];
    int32_t             m_lastBitsY      [ MAX_TB_SIZEY ];
    BinFracBits         m_sigSbbFracBits [ sm_maxNumSigSbbCtx ];
    BinFracBits         m_sigFracBits    [ sm_numCtxSetsSig   ][ sm_maxNumSigCtx ];
    CoeffFracBits       m_gtxFracBits                          [ sm_maxNumGtxCtx ];
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

  class Quantizer
  {
  public:
    Quantizer() {}
    void   init            ( int dqThrVal ) { m_DqThrVal = dqThrVal; }
    void   dequantBlock    ( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff, bool enableScalingLists, int* piDequantCoef ) const;
    void   initQuantBlock  ( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda, int gValue = -1 );
    bool   preQuantCoeff   ( const TCoeff absCoeff, PQData *pqData, int quanCoeff ) const;
    TCoeff getLastThreshold() const { return m_thresLast; }
    TCoeff getSSbbThreshold() const { return m_thresSSbb; }

    int64_t getQScale      () const { return m_QScale; }

    // quantization
    int               m_DqThrVal;
    int               m_QShift;
    int64_t           m_QAdd;
    int64_t           m_QScale;
    TCoeff            m_maxQIdx;
    TCoeff            m_thresLast;
    TCoeff            m_thresSSbb;
    // distortion normalization
    int               m_DistShift;
    int64_t           m_DistAdd;
    int64_t           m_DistStepAdd;
    int64_t           m_DistOrgFact;
  };

#define RICEMAX 32
  extern const int32_t g_goRiceBits[4][RICEMAX];

  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q   S T A T E                                                  =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  class State;

  struct Decision
  {
    int64_t rdCost;
    TCoeff  absLevel;
    int     prevId;
  };

  struct SbbCtx
  {
    uint8_t* sbbFlags;
    uint8_t* levels;
  };

  class CommonCtx
  {
  public:
    CommonCtx() : m_currSbbCtx( m_allSbbCtx ), m_prevSbbCtx( m_currSbbCtx + 4 ) {}

    inline void swap() { std::swap( m_currSbbCtx, m_prevSbbCtx ); }

    inline void reset( const TUParameters& tuPars, const RateEstimator& rateEst )
    {
      m_nbInfo = tuPars.m_scanId2NbInfoOut;
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

    inline void update( const ScanInfo& scanInfo, const State* prevState, State& currState );

  private:
    const NbInfoOut* m_nbInfo;
    BinFracBits                 m_sbbFlagBits[2];
    SbbCtx                      m_allSbbCtx[8];
    SbbCtx* m_currSbbCtx;
    SbbCtx* m_prevSbbCtx;
    uint8_t                     m_memory[8 * ( MAX_TB_SIZEY * MAX_TB_SIZEY + MLS_GRP_NUM )];
  };

  class State
  {
    friend class CommonCtx;
  public:
    State( const RateEstimator& rateEst, CommonCtx& commonCtx, const int stateId );

    inline void updateState( const ScanInfo& scanInfo, const State* prevStates, const Decision& decision );
    inline void updateStateEOS( const ScanInfo& scanInfo, const State* prevStates, const State* skipStates,
                                const Decision& decision );

    inline void init()
    {
      m_rdCost = rdCostInit;
      m_numSigSbb = 0;
      m_remRegBins = 4;  // just large enough for last scan pos
      m_refSbbCtxId = -1;
      m_sigFracBits = m_sigFracBitsArray[0];
      m_coeffFracBits = m_gtxFracBitsArray[0];
      m_goRicePar = 0;
      m_goRiceZero = 0;
      VALGRIND_MEMCLEAR( m_state, sizeof( m_state ) );
    }

    void checkRdCosts( const ScanPosType spt, const PQData& pqDataA, const PQData& pqDataB, Decision& decisionA, Decision& decisionB ) const
    {
      const int32_t* goRiceTab = g_goRiceBits[m_goRicePar];
      int64_t         rdCostA = m_rdCost + pqDataA.deltaDist;
      int64_t         rdCostB = m_rdCost + pqDataB.deltaDist;
      int64_t         rdCostZ = m_rdCost;

      if( m_remRegBins >= 4 )
      {
        if( pqDataA.absLevel < 4 )
          rdCostA += m_coeffFracBits.bits[pqDataA.absLevel];
        else
        {
          const unsigned value = ( pqDataA.absLevel - 4 ) >> 1;
          rdCostA += m_coeffFracBits.bits[pqDataA.absLevel - ( value << 1 )] + goRiceTab[std::min<unsigned>( value, RICEMAX - 1 )];
        }

        if( pqDataB.absLevel < 4 )
          rdCostB += m_coeffFracBits.bits[pqDataB.absLevel];
        else
        {
          const unsigned value = ( pqDataB.absLevel - 4 ) >> 1;
          rdCostB += m_coeffFracBits.bits[pqDataB.absLevel - ( value << 1 )] + goRiceTab[std::min<unsigned>( value, RICEMAX - 1 )];
        }

        if( spt == SCAN_ISCSBB )
        {
          rdCostA += m_sigFracBits.intBits[1];
          rdCostB += m_sigFracBits.intBits[1];
          rdCostZ += m_sigFracBits.intBits[0];
        }
        else if( spt == SCAN_SOCSBB )
        {
          rdCostA += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[1];
          rdCostB += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[1];
          rdCostZ += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[0];
        }
        else if( m_numSigSbb )
        {
          rdCostA += m_sigFracBits.intBits[1];
          rdCostB += m_sigFracBits.intBits[1];
          rdCostZ += m_sigFracBits.intBits[0];
        }
        else
        {
          rdCostZ = decisionA.rdCost;
        }
      }
      else
      {
        rdCostA += ( 1 << SCALE_BITS ) + goRiceTab[pqDataA.absLevel <= m_goRiceZero ? pqDataA.absLevel - 1 : std::min<int>( pqDataA.absLevel, RICEMAX - 1 )];
        rdCostB += ( 1 << SCALE_BITS ) + goRiceTab[pqDataB.absLevel <= m_goRiceZero ? pqDataB.absLevel - 1 : std::min<int>( pqDataB.absLevel, RICEMAX - 1 )];
        rdCostZ += goRiceTab[m_goRiceZero];
      }

      if( rdCostA < rdCostZ && rdCostA < decisionA.rdCost )
      {
        decisionA.rdCost = rdCostA;
        decisionA.absLevel = pqDataA.absLevel;
        decisionA.prevId = m_stateId;
      }
      else if( rdCostZ < decisionA.rdCost )
      {
        decisionA.rdCost = rdCostZ;
        decisionA.absLevel = 0;
        decisionA.prevId = m_stateId;
      }

      if( rdCostB < decisionB.rdCost )
      {
        decisionB.rdCost = rdCostB;
        decisionB.absLevel = pqDataB.absLevel;
        decisionB.prevId = m_stateId;
      }
    }

    void checkRdCostsOdd1( const ScanPosType spt, const PQData& pqDataA, Decision& decisionA, Decision& decisionZ ) const
    {
      CHECKD( pqDataA.absLevel != 1, "" );

      const int32_t* goRiceTab = g_goRiceBits[m_goRicePar];
      int64_t         rdCostA = m_rdCost + pqDataA.deltaDist;
      int64_t         rdCostZ = m_rdCost;

      if( m_remRegBins >= 4 )
      {
        rdCostA += m_coeffFracBits.bits[1];

        if( spt == SCAN_ISCSBB )
        {
          rdCostA += m_sigFracBits.intBits[1];
          rdCostZ += m_sigFracBits.intBits[0];
        }
        else if( spt == SCAN_SOCSBB )
        {
          rdCostA += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[1];
          rdCostZ += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[0];
        }
        else if( m_numSigSbb )
        {
          rdCostA += m_sigFracBits.intBits[1];
          rdCostZ += m_sigFracBits.intBits[0];
        }
        else
        {
          rdCostZ = decisionZ.rdCost;
        }
      }
      else
      {
        rdCostA += ( 1 << SCALE_BITS ) + goRiceTab[0];
        rdCostZ += goRiceTab[m_goRiceZero];
      }

      if( rdCostA < decisionA.rdCost )
      {
        decisionA.rdCost = rdCostA;
        decisionA.absLevel = 1;
        decisionA.prevId = m_stateId;
      }

      if( rdCostZ < decisionZ.rdCost )
      {
        decisionZ.rdCost = rdCostZ;
        decisionZ.absLevel = 0;
        decisionZ.prevId = m_stateId;
      }
    }

    inline void checkRdCostStart( int32_t lastOffset, const PQData& pqData, Decision& decision ) const
    {
      int64_t rdCost = pqData.deltaDist + lastOffset;
      if( pqData.absLevel < 4 )
      {
        rdCost += m_coeffFracBits.bits[pqData.absLevel];
      }
      else
      {
        const unsigned value = ( pqData.absLevel - 4 ) >> 1;
        rdCost += m_coeffFracBits.bits[pqData.absLevel - ( value << 1 )] + g_goRiceBits[m_goRicePar][value < RICEMAX ? value : RICEMAX - 1];
      }
      if( rdCost < decision.rdCost )
      {
        decision.rdCost = rdCost;
        decision.absLevel = pqData.absLevel;
        decision.prevId = -1;
      }
    }

    inline void checkRdCostSkipSbb( Decision& decision ) const
    {
      int64_t rdCost = m_rdCost + m_sbbFracBits.intBits[0];
      if( rdCost < decision.rdCost )
      {
        decision.rdCost = rdCost;
        decision.absLevel = 0;
        decision.prevId = 4 | m_stateId;
      }
    }

    inline void checkRdCostSkipSbbZeroOut( Decision& decision ) const
    {
      int64_t rdCost = m_rdCost + m_sbbFracBits.intBits[0];
      decision.rdCost = rdCost;
      decision.absLevel = 0;
      decision.prevId = 4 | m_stateId;
    }

    inline void setRiceParam( const ScanInfo& scanInfo )
    {
      if( m_remRegBins >= 4 )
      {
        TCoeff  sumAbs = m_sbb.ctx[scanInfo.insidePos].sumAbs;
        int sumAll = std::max( std::min( 31, ( int ) sumAbs - 4 * 5 ), 0 );
        m_goRicePar = g_auiGoRiceParsCoeff[sumAll];
      }
    }

    struct CtxAcc
    {
      // tplAcc: lower 5 bits are absSum1, upper 3 bits are numPos
      uint8_t tplAcc, sumAbs;
    };

  private:

    int64_t                   m_rdCost;
    union
    {
      uint8_t                 m_state[48];
      struct
      {
        uint8_t               absLevels[16];
        CtxAcc                ctx[16];
      } m_sbb;
    };
    int8_t                    m_numSigSbb;
    int                       m_remRegBins;
    int8_t                    m_refSbbCtxId;
    BinFracBits               m_sbbFracBits;
    BinFracBits               m_sigFracBits;
    CoeffFracBits             m_coeffFracBits;
    int8_t                    m_goRicePar;
    int8_t                    m_goRiceZero;
    const int8_t              m_stateId;
    const BinFracBits* const   m_sigFracBitsArray;
    const CoeffFracBits* const m_gtxFracBitsArray;
    CommonCtx&                m_commonCtx;
  public:
    static const int64_t      rdCostInit = std::numeric_limits<int64_t>::max() >> 1;
    unsigned                  effWidth;
    unsigned                  effHeight;
  };
}

class DepQuant : public QuantRDOQ2, DQIntern::RateEstimator
{
public:
  DepQuant( const Quant* other, bool enc, bool useScalingLists );
  virtual ~DepQuant();

  virtual void quant  (       TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx& ctx );
  virtual void dequant( const TransformUnit& tu, CoeffBuf& dstCoeff, const ComponentID compID, const QpParam& cQP );
  
  virtual void init   ( int rdoq = 0, bool useRDOQTS = false, int dqThrVal = 8 );

private:
  void    xQuantDQ          ( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP, const double lambda, const Ctx& ctx, TCoeff& absSum, bool enableScalingLists, int* quantCoeff );
  void    xDequantDQ        ( const TransformUnit& tu, CoeffBuf& recCoeff, const ComponentID compID, const QpParam& cQP, bool enableScalingLists, int* quantCoeff );
  void    xDecideAndUpdate  ( const TCoeff absCoeff, const DQIntern::ScanInfo& scanInfo, bool zeroOut, int quantCoeff);
  void    xDecide           ( const DQIntern::ScanInfo& scanInfo, const TCoeff absCoeff, const int lastOffset, DQIntern::Decision* decisions, bool zeroOut, int quantCoeff );

  DQIntern::CommonCtx m_commonCtx;
  DQIntern::State     m_allStates[ 12 ];
  DQIntern::State*    m_currStates;
  DQIntern::State*    m_prevStates;
  DQIntern::State*    m_skipStates;
  DQIntern::State     m_startState;
  DQIntern::Decision  m_trellis[ MAX_TB_SIZEY * MAX_TB_SIZEY ][ 8 ];
  DQIntern::Rom       m_scansRom;
  DQIntern::Quantizer m_quant;

#if defined(TARGET_SIMD_X86)  && ENABLE_SIMD_OPT_QUANT
  void initDepQuantX86();
  template <X86_VEXT vext>
  void _initDepQuantX86();
#endif
};

} // namespace vvenc

//! \}


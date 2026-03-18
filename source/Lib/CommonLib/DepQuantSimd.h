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
/**
 * \file     DepQuantSimd.h
 * \brief    SIMD optimized dependent quantization class
 */

#pragma once

#include "CodingStructure.h"
#include "DepQuant.h"
#include "UnitTools.h"

#if defined ENABLE_SIMD_OPT_QUANT
#include "TrQuant.h"

//! \ingroup CommonLib
//! \{

namespace vvenc
{

namespace DQIntern
{
/*================================================================================*/
/*=====                                                                      =====*/
/*=====   T C Q   S T A T E                                                  =====*/
/*=====                                                                      =====*/
/*================================================================================*/

static constexpr int64_t rdCostInit = std::numeric_limits<int64_t>::max() >> 1;

struct Decisions
{
  int64_t rdCost[4];
  TCoeffSig absLevel[4];
  int8_t prevId[4];
};

struct StateMem
{
  int64_t rdCost[4];
  int16_t remRegBins[4];
  int32_t sbbBits0[4];
  int32_t sbbBits1[4];

  uint8_t tplAcc[64];
  uint8_t sum1st[64];
  uint8_t absVal[64];

  struct
  {
    uint8_t sig[4];
    uint8_t cff[4];
  } ctx;

  uint8_t numSig[4];
  int8_t refSbbCtxId[4];

  int32_t cffBits1[RateEstimator::sm_maxNumGtxCtx + 3];

  int8_t m_goRicePar[4];
  int8_t m_goRiceZero[4];
  const BinFracBits* m_sigFracBitsArray[4];
  const CoeffFracBits* m_gtxFracBitsArray;

  int cffBitsCtxOffset;
  bool anyRemRegBinsLt4;
  int initRemRegBins;
};

static constexpr size_t StateMemSkipCpySize = offsetof( StateMem, sbbBits1 );

class StateSimd
{
  friend class CommonCtx;

public:
  static inline void init( const int stateId, StateMem& state )
  {
    state.rdCost[stateId] = rdCostInit;
    state.ctx.cff[stateId] = 0;
    state.ctx.sig[stateId] = 0;
    state.numSig[stateId] = 0;
    state.refSbbCtxId[stateId] = -1;
    state.remRegBins[stateId] = 4;
    state.cffBitsCtxOffset = 0;
    state.m_goRicePar[stateId] = 0;
    state.m_goRiceZero[stateId] = 0;
    state.sbbBits0[stateId] = 0;
    state.sbbBits1[stateId] = 0;
  }

  static inline void checkRdCosts( const int stateId, const ScanPosType spt, const PQData& pqDataA,
                                   const PQData& pqDataB, Decisions& decisions, int idxAZ, int idxB,
                                   const StateMem& state )
  {
    const int32_t* goRiceTab = g_goRiceBits[state.m_goRicePar[stateId]];
    int64_t rdCostA = state.rdCost[stateId] + pqDataA.deltaDist;
    int64_t rdCostB = state.rdCost[stateId] + pqDataB.deltaDist;
    int64_t rdCostZ = state.rdCost[stateId];

    if( state.remRegBins[stateId] >= 4 )
    {
      const CoeffFracBits& cffBits = state.m_gtxFracBitsArray[state.ctx.cff[stateId]];
      const BinFracBits sigBits = state.m_sigFracBitsArray[stateId][state.ctx.sig[stateId]];

      if( pqDataA.absLevel < 4 )
        rdCostA += cffBits.bits[pqDataA.absLevel];
      else
      {
        const unsigned value = ( pqDataA.absLevel - 4 ) >> 1;
        rdCostA +=
            cffBits.bits[pqDataA.absLevel - ( value << 1 )] + goRiceTab[std::min<unsigned>( value, RICEMAX - 1 )];
      }

      if( pqDataB.absLevel < 4 )
        rdCostB += cffBits.bits[pqDataB.absLevel];
      else
      {
        const unsigned value = ( pqDataB.absLevel - 4 ) >> 1;
        rdCostB +=
            cffBits.bits[pqDataB.absLevel - ( value << 1 )] + goRiceTab[std::min<unsigned>( value, RICEMAX - 1 )];
      }

      if( spt == SCAN_ISCSBB )
      {
        rdCostA += sigBits.intBits[1];
        rdCostB += sigBits.intBits[1];
        rdCostZ += sigBits.intBits[0];
      }
      else if( spt == SCAN_SOCSBB )
      {
        rdCostA += state.sbbBits1[stateId] + sigBits.intBits[1];
        rdCostB += state.sbbBits1[stateId] + sigBits.intBits[1];
        rdCostZ += state.sbbBits1[stateId] + sigBits.intBits[0];
      }
      else if( state.numSig[stateId] )
      {
        rdCostA += sigBits.intBits[1];
        rdCostB += sigBits.intBits[1];
        rdCostZ += sigBits.intBits[0];
      }
      else
      {
        rdCostZ = rdCostInit;
      }
    }
    else
    {
      rdCostA +=
          ( 1 << SCALE_BITS ) +
          goRiceTab[pqDataA.absLevel <= state.m_goRiceZero[stateId] ? pqDataA.absLevel - 1
                                                                    : std::min<int>( pqDataA.absLevel, RICEMAX - 1 )];
      rdCostB +=
          ( 1 << SCALE_BITS ) +
          goRiceTab[pqDataB.absLevel <= state.m_goRiceZero[stateId] ? pqDataB.absLevel - 1
                                                                    : std::min<int>( pqDataB.absLevel, RICEMAX - 1 )];
      rdCostZ += goRiceTab[state.m_goRiceZero[stateId]];
    }

    if( rdCostA < rdCostZ && rdCostA < decisions.rdCost[idxAZ] )
    {
      decisions.rdCost[idxAZ] = rdCostA;
      decisions.absLevel[idxAZ] = pqDataA.absLevel;
      decisions.prevId[idxAZ] = stateId;
    }
    else if( rdCostZ < decisions.rdCost[idxAZ] )
    {
      decisions.rdCost[idxAZ] = rdCostZ;
      decisions.absLevel[idxAZ] = 0;
      decisions.prevId[idxAZ] = stateId;
    }

    if( rdCostB < decisions.rdCost[idxB] )
    {
      decisions.rdCost[idxB] = rdCostB;
      decisions.absLevel[idxB] = pqDataB.absLevel;
      decisions.prevId[idxB] = stateId;
    }
  }

  static void checkRdCostsOdd1( const int stateId, const ScanPosType spt, const int64_t deltaDist, Decisions& decisions,
                                int idxA, int idxZ, const StateMem& state )
  {
    int64_t rdCostA = state.rdCost[stateId] + deltaDist;
    int64_t rdCostZ = state.rdCost[stateId];

    if( state.remRegBins[stateId] >= 4 )
    {
      const BinFracBits sigBits = state.m_sigFracBitsArray[stateId][state.ctx.sig[stateId]];

      rdCostA += state.m_gtxFracBitsArray[state.ctx.cff[stateId]].bits[1];

      if( spt == SCAN_ISCSBB )
      {
        rdCostA += sigBits.intBits[1];
        rdCostZ += sigBits.intBits[0];
      }
      else if( spt == SCAN_SOCSBB )
      {
        rdCostA += state.sbbBits1[stateId] + sigBits.intBits[1];
        rdCostZ += state.sbbBits1[stateId] + sigBits.intBits[0];
      }
      else if( state.numSig[stateId] )
      {
        rdCostA += sigBits.intBits[1];
        rdCostZ += sigBits.intBits[0];
      }
      else
      {
        rdCostZ = rdCostInit;
      }
    }
    else
    {
      const int32_t* goRiceTab = g_goRiceBits[state.m_goRicePar[stateId]];

      rdCostA += ( 1 << SCALE_BITS ) + goRiceTab[0];
      rdCostZ += goRiceTab[state.m_goRiceZero[stateId]];
    }

    if( rdCostA < decisions.rdCost[idxA] )
    {
      decisions.rdCost[idxA] = rdCostA;
      decisions.absLevel[idxA] = 1;
      decisions.prevId[idxA] = stateId;
    }

    if( rdCostZ < decisions.rdCost[idxZ] )
    {
      decisions.rdCost[idxZ] = rdCostZ;
      decisions.absLevel[idxZ] = 0;
      decisions.prevId[idxZ] = stateId;
    }
  }

  static inline void checkRdCostStart( int32_t lastOffset, const PQData& pqData, Decisions& decisions, int idx,
                                       const StateMem& state )
  {
    const CoeffFracBits& cffBits = state.m_gtxFracBitsArray[0];

    int64_t rdCost = pqData.deltaDist + lastOffset;
    if( pqData.absLevel < 4 )
    {
      rdCost += cffBits.bits[pqData.absLevel];
    }
    else
    {
      const unsigned value = ( pqData.absLevel - 4 ) >> 1;
      rdCost += cffBits.bits[pqData.absLevel - ( value << 1 )] + g_goRiceBits[0][value < RICEMAX ? value : RICEMAX - 1];
    }

    if( rdCost < decisions.rdCost[idx] )
    {
      decisions.rdCost[idx] = rdCost;
      decisions.absLevel[idx] = pqData.absLevel;
      decisions.prevId[idx] = -1;
    }
  }

  static inline void checkRdCostSkipSbb( const int stateId, Decisions& decisions, int idx, const StateMem& state )
  {
    int64_t rdCost = state.rdCost[stateId] + state.sbbBits0[stateId];
    if( rdCost < decisions.rdCost[idx] )
    {
      decisions.rdCost[idx] = rdCost;
      decisions.absLevel[idx] = 0;
      decisions.prevId[idx] = 4 | stateId;
    }
  }

  static inline void checkRdCostSkipSbbZeroOut( const int stateId, Decisions& decisions, int idx,
                                                const StateMem& state )
  {
    int64_t rdCost = state.rdCost[stateId] + state.sbbBits0[stateId];
    decisions.rdCost[idx] = rdCost;
    decisions.absLevel[idx] = 0;
    decisions.prevId[idx] = 4 | stateId;
  }

  static inline void setRiceParam( const int stateId, const ScanInfo& scanInfo, StateMem& state, bool ge4 )
  {
    if( state.remRegBins[stateId] < 4 || ge4 )
    {
      const int addr = ( scanInfo.insidePos << 2 ) + stateId;
      TCoeff sumAbs = state.sum1st[addr];
      int sumSub = state.remRegBins[stateId] < 4 ? 0 : 4 * 5;
      int sumAll = std::max( std::min( 31, ( int )sumAbs - sumSub ), 0 );
      state.m_goRicePar[stateId] = g_auiGoRiceParsCoeff[sumAll];

      if( state.remRegBins[stateId] < 4 )
      {
        state.m_goRiceZero[stateId] = g_auiGoRicePosCoeff0( stateId, state.m_goRicePar[stateId] );
      }
    }
  }
  // End of class StateSimd.
};

class DepQuantSimd : private RateEstimator, public DepQuantImpl
{

public:
  const Decisions startDec[2] = { Decisions{
                                      { rdCostInit >> 2, rdCostInit >> 2, rdCostInit >> 2, rdCostInit >> 2 },
                                      { -1, -1, -1, -1 },
                                      { -2, -2, -2, -2 },
                                  },
                                  Decisions{
                                      { rdCostInit >> 2, rdCostInit >> 2, rdCostInit >> 2, rdCostInit >> 2 },
                                      { 0, 0, 0, 0 },
                                      { 4, 5, 6, 7 },
                                  } };

  DepQuantSimd() : RateEstimator(), m_commonCtx()
  {
    m_scansRom.init();

    for( int t = 0; t < ( MAX_TB_SIZEY * MAX_TB_SIZEY ); t++ )
    {
      memcpy( m_trellis[t], startDec, sizeof( startDec ) );
    }

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_QUANT
    initDepQuantSimdX86();
#endif
#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_QUANT
    initDepQuantSimdARM();
#endif
  }
#undef TINIT

  ~DepQuantSimd()
  {
  }

  void quant( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP,
              const double lambda, const Ctx& ctx, TCoeff& absSum, bool enableScalingLists, int* quantCoeff )
  {
    //===== reset / pre-init =====
    const TUParameters& tuPars = *m_scansRom.getTUPars( tu.blocks[compID], compID );
    m_quant.initQuantBlock( tu, compID, cQP, lambda );
    TCoeffSig* qCoeff = tu.getCoeffs( compID ).buf;
    const TCoeff* tCoeff = srcCoeff.buf;
    const int numCoeff = tu.blocks[compID].area();
    ::memset( qCoeff, 0x00, numCoeff * sizeof( TCoeffSig ) );
    absSum = 0;

    const CompArea& area = tu.blocks[compID];
    const uint32_t width = area.width;
    const uint32_t height = area.height;
    const uint32_t lfnstIdx = tu.cu->lfnstIdx;
    //===== scaling matrix ====
    // const int         qpDQ = cQP.Qp + 1;
    // const int         qpPer = qpDQ / 6;
    // const int         qpRem = qpDQ - 6 * qpPer;

    // TCoeff thresTmp = thres;
    bool zeroOut = false;
    bool zeroOutforThres = false;
    int effWidth = tuPars.m_width, effHeight = tuPars.m_height;
    if( ( tu.mtsIdx[compID] > MTS_SKIP ||
          ( tu.cs->sps->MTS && tu.cu->sbtInfo != 0 && tuPars.m_height <= 32 && tuPars.m_width <= 32 ) ) &&
        compID == COMP_Y )
    {
      effHeight = ( tuPars.m_height == 32 ) ? 16 : tuPars.m_height;
      effWidth = ( tuPars.m_width == 32 ) ? 16 : tuPars.m_width;
      zeroOut = ( effHeight < tuPars.m_height || effWidth < tuPars.m_width );
    }
    zeroOutforThres = zeroOut || ( 32 < tuPars.m_height || 32 < tuPars.m_width );
    //===== find first test position =====
    int firstTestPos = std::min<int>( tuPars.m_width, JVET_C0024_ZERO_OUT_TH ) *
                           std::min<int>( tuPars.m_height, JVET_C0024_ZERO_OUT_TH ) -
                       1;
    if( lfnstIdx > 0 && tu.mtsIdx[compID] != MTS_SKIP && width >= 4 && height >= 4 )
    {
      firstTestPos = ( ( width == 4 && height == 4 ) || ( width == 8 && height == 8 ) ) ? 7 : 15;
    }

    const TCoeff defaultQuantisationCoefficient = ( TCoeff )m_quant.getQScale();
    const TCoeff thres = m_quant.getLastThreshold();
    const int zeroOutWidth = ( tuPars.m_width == 32 && zeroOut ) ? 16 : 32;
    const int zeroOutHeight = ( tuPars.m_height == 32 && zeroOut ) ? 16 : 32;

    if( enableScalingLists )
    {
      for( ; firstTestPos >= 0; firstTestPos-- )
      {
        if( zeroOutforThres && ( tuPars.m_scanId2BlkPos[firstTestPos].x >= zeroOutWidth ||
                                 tuPars.m_scanId2BlkPos[firstTestPos].y >= zeroOutHeight ) )
          continue;

        const TCoeff thresTmp = TCoeff( thres / ( 4 * quantCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) );

        if( abs( tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) > thresTmp )
          break;
      }
    }
    else
    {
      const TCoeff defaultTh = TCoeff( thres / ( defaultQuantisationCoefficient << 2 ) );

      // if more than one 4x4 coding subblock is available, use SIMD to find first subblock with coefficient larger than
      // threshold
      if( firstTestPos >= 16 && tuPars.m_log2SbbWidth == 2 &&
          tuPars.m_log2SbbHeight == 2 && m_xFindFirstTestPos != nullptr )
      {
        firstTestPos = m_xFindFirstTestPos( tuPars, tCoeff, firstTestPos, defaultTh,
                                    zeroOutforThres, zeroOutWidth, zeroOutHeight );
      }
      for( ; firstTestPos >= 0; firstTestPos-- )
      {
        if( zeroOutforThres && ( tuPars.m_scanId2BlkPos[firstTestPos].x >= zeroOutWidth ||
                                 tuPars.m_scanId2BlkPos[firstTestPos].y >= zeroOutHeight ) )
          continue;
        if( abs( tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) > defaultTh )
          break;
      }
    }

    if( firstTestPos < 0 )
    {
      tu.lastPos[compID] = -1;
      return;
    }

    //===== real init =====
    RateEstimator::initCtx( tuPars, tu, compID, ctx.getFracBitsAcess() );
    m_commonCtx.reset( tuPars, *this );
    for( int k = 0; k < 4; k++ )
    {
      StateSimd::init( k, m_state_curr );
      StateSimd::init( k, m_state_skip );
      m_state_curr.m_sigFracBitsArray[k] = RateEstimator::sigFlagBits( k );
    }

    m_state_curr.m_gtxFracBitsArray = RateEstimator::gtxFracBits();
    // memset( m_state_curr.tplAcc, 0, sizeof( m_state_curr.tplAcc ) ); // will be set in updateStates{,EOS} before
    // first access
    memset( m_state_curr.sum1st, 0,
            sizeof( m_state_curr.sum1st ) ); // will be accessed in setRiceParam before updateState{,EOS}
    // memset( m_state_curr.absVal, 0, sizeof( m_state_curr.absVal ) ); // will be set in updateStates{,EOS} before
    // first access

    const int numCtx = isLuma( compID ) ? 21 : 11;
    const CoeffFracBits* const cffBits = gtxFracBits();
    for( int i = 0; i < numCtx; i++ )
    {
      m_state_curr.cffBits1[i] = cffBits[i].bits[1];
    }

    int effectWidth = std::min( 32, effWidth );
    int effectHeight = std::min( 32, effHeight );
    m_state_curr.initRemRegBins = ( effectWidth * effectHeight * MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT ) / 16;
    m_state_curr.anyRemRegBinsLt4 = true; // for the first coeff use scalar impl., because it check against the init
                                          // state, which prohibits some paths

    //===== populate trellis =====
    for( int scanIdx = firstTestPos; scanIdx >= 0; scanIdx-- )
    {
      const ScanInfo& scanInfo = tuPars.m_scanInfo[scanIdx];
      if( enableScalingLists )
      {
        m_quant.initQuantBlock( tu, compID, cQP, lambda, quantCoeff[scanInfo.rasterPos] );
        xDecideAndUpdate( abs( tCoeff[scanInfo.rasterPos] ), scanInfo,
                          zeroOut && ( scanInfo.posX >= effWidth || scanInfo.posY >= effHeight ),
                          quantCoeff[scanInfo.rasterPos] );
      }
      else
        xDecideAndUpdate( abs( tCoeff[scanInfo.rasterPos] ), scanInfo,
                          zeroOut && ( scanInfo.posX >= effWidth || scanInfo.posY >= effHeight ),
                          defaultQuantisationCoefficient );
    }

    //===== find best path =====
    int prevId = -1;
    int64_t minPathCost = 0;
    for( int8_t stateId = 0; stateId < 4; stateId++ )
    {
      int64_t pathCost = m_trellis[0][0].rdCost[stateId];
      if( pathCost < minPathCost )
      {
        prevId = stateId;
        minPathCost = pathCost;
      }
    }

    //===== backward scanning =====
    int scanIdx = 0;
    for( ; prevId >= 0; scanIdx++ )
    {
      TCoeffSig absLevel = m_trellis[scanIdx][prevId >> 2].absLevel[prevId & 3];
      int32_t blkpos = tuPars.m_scanId2BlkPos[scanIdx].idx;
      qCoeff[blkpos] = TCoeffSig( tCoeff[blkpos] < 0 ? -absLevel : absLevel );
      absSum += absLevel;
      prevId = m_trellis[scanIdx][prevId >> 2].prevId[prevId & 3];
    }

    tu.lastPos[compID] = scanIdx - 1;
  }

private:
  void xDecideAndUpdate( const TCoeff absCoeff, const ScanInfo& scanInfo, bool zeroOut, int quantCoeff )
  {
    Decisions* decisions = &m_trellis[scanInfo.scanIdx][0];

    xDecide( scanInfo, absCoeff, lastOffset( scanInfo.scanIdx ), *decisions, zeroOut, quantCoeff );

    if( scanInfo.scanIdx )
    {
      if( scanInfo.spt == SCAN_SOCSBB )
      {
        memcpy( &m_state_skip, &m_state_curr, StateMemSkipCpySize );
      }

      if( scanInfo.insidePos == 0 )
      {
        m_commonCtx.swap();
        m_updateStatesEOS( scanInfo, *decisions, m_state_skip, m_state_curr, m_commonCtx );
        ::memcpy( decisions + 1, decisions, sizeof( Decisions ) );
      }
      else if( !zeroOut )
      {
        m_updateStates( scanInfo, *decisions, m_state_curr );
      }
    }
  }

  void xDecide( const ScanInfo& scanInfo, const TCoeff absCoeff, const int lastOffset, Decisions& decisions,
                bool zeroOut, int quantCoeff )
  {
    ::memcpy( &decisions, startDec, sizeof( Decisions ) );

    StateMem& skip = m_state_skip;

    if( zeroOut )
    {
      if( scanInfo.spt == SCAN_EOCSBB )
      {
        StateSimd::checkRdCostSkipSbbZeroOut( 0, decisions, 0, skip );
        StateSimd::checkRdCostSkipSbbZeroOut( 1, decisions, 1, skip );
        StateSimd::checkRdCostSkipSbbZeroOut( 2, decisions, 2, skip );
        StateSimd::checkRdCostSkipSbbZeroOut( 3, decisions, 3, skip );
      }
      return;
    }

    StateMem& prev = m_state_curr;

    /// start inline prequant
    int64_t scaledOrg = int64_t( absCoeff ) * quantCoeff;
    TCoeff qIdx = TCoeff( ( scaledOrg + m_quant.m_QAdd ) >> m_quant.m_QShift );

    if( qIdx < 0 )
    {
      int64_t scaledAdd = m_quant.m_DistStepAdd - scaledOrg * m_quant.m_DistOrgFact;
      int64_t pq_a_dist = ( ( scaledAdd + 0 * m_quant.m_DistStepAdd ) * 1 + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
      int64_t pq_b_dist = ( ( scaledAdd + 1 * m_quant.m_DistStepAdd ) * 2 + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
      /// stop inline prequant

      if( prev.anyRemRegBinsLt4 )
      {
        StateSimd::setRiceParam( 0, scanInfo, prev, false );
        StateSimd::checkRdCostsOdd1( 0, scanInfo.spt, pq_b_dist, decisions, 2, 0, prev );

        StateSimd::setRiceParam( 1, scanInfo, prev, false );
        StateSimd::checkRdCostsOdd1( 1, scanInfo.spt, pq_b_dist, decisions, 0, 2, prev );

        StateSimd::setRiceParam( 2, scanInfo, prev, false );
        StateSimd::checkRdCostsOdd1( 2, scanInfo.spt, pq_a_dist, decisions, 3, 1, prev );

        StateSimd::setRiceParam( 3, scanInfo, prev, false );
        StateSimd::checkRdCostsOdd1( 3, scanInfo.spt, pq_a_dist, decisions, 1, 3, prev );
      }
      else
      {
        // has to be called as a first check, assumes no decision has been made yet
        m_checkAllRdCostsOdd1( scanInfo.spt, pq_a_dist, pq_b_dist, decisions, prev );
      }

      StateSimd::checkRdCostStart( lastOffset, PQData{ 1, pq_b_dist }, decisions, 2, prev );
    }
    else
    {
      /// start inline prequant
      qIdx = std::max<TCoeff>( 1, std::min<TCoeff>( m_quant.m_maxQIdx, qIdx ) );
      int64_t scaledAdd = qIdx * m_quant.m_DistStepAdd - scaledOrg * m_quant.m_DistOrgFact;

      PQData pqData[4];

      PQData& pq_a = pqData[( qIdx + 0 ) & 3];
      PQData& pq_b = pqData[( qIdx + 1 ) & 3];
      PQData& pq_c = pqData[( qIdx + 2 ) & 3];
      PQData& pq_d = pqData[( qIdx + 3 ) & 3];

      pq_a.deltaDist =
          ( ( scaledAdd + 0 * m_quant.m_DistStepAdd ) * ( qIdx + 0 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
      pq_a.absLevel = ( qIdx + 1 ) >> 1;

      pq_b.deltaDist =
          ( ( scaledAdd + 1 * m_quant.m_DistStepAdd ) * ( qIdx + 1 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
      pq_b.absLevel = ( qIdx + 2 ) >> 1;

      pq_c.deltaDist =
          ( ( scaledAdd + 2 * m_quant.m_DistStepAdd ) * ( qIdx + 2 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
      pq_c.absLevel = ( qIdx + 3 ) >> 1;

      pq_d.deltaDist =
          ( ( scaledAdd + 3 * m_quant.m_DistStepAdd ) * ( qIdx + 3 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
      pq_d.absLevel = ( qIdx + 4 ) >> 1;
      /// stop inline prequant

      bool cff02ge4 = pqData[0].absLevel >= 4 /* || pqData[2].absLevel >= 4 */;
      bool cff13ge4 = /* pqData[1].absLevel >= 4 || */ pqData[3].absLevel >= 4;

      if( cff02ge4 || cff13ge4 || prev.anyRemRegBinsLt4 )
      {
        if( prev.anyRemRegBinsLt4 || cff02ge4 )
        {
          StateSimd::setRiceParam( 0, scanInfo, prev, cff02ge4 );
          StateSimd::setRiceParam( 1, scanInfo, prev, cff02ge4 );
        }

        if( prev.anyRemRegBinsLt4 || cff13ge4 )
        {
          StateSimd::setRiceParam( 2, scanInfo, prev, cff13ge4 );
          StateSimd::setRiceParam( 3, scanInfo, prev, cff13ge4 );
        }

        StateSimd::checkRdCosts( 0, scanInfo.spt, pqData[0], pqData[2], decisions, 0, 2, prev );
        StateSimd::checkRdCosts( 1, scanInfo.spt, pqData[0], pqData[2], decisions, 2, 0, prev );
        StateSimd::checkRdCosts( 2, scanInfo.spt, pqData[3], pqData[1], decisions, 1, 3, prev );
        StateSimd::checkRdCosts( 3, scanInfo.spt, pqData[3], pqData[1], decisions, 3, 1, prev );
      }
      else
      {
        // has to be called as a first check, assumes no decision has been made yet
        m_checkAllRdCosts( scanInfo.spt, pqData, decisions, prev );
      }

      StateSimd::checkRdCostStart( lastOffset, pqData[0], decisions, 0, prev );
      StateSimd::checkRdCostStart( lastOffset, pqData[2], decisions, 2, prev );
    }

    if( scanInfo.spt == SCAN_EOCSBB )
    {
      StateSimd::checkRdCostSkipSbb( 0, decisions, 0, skip );
      StateSimd::checkRdCostSkipSbb( 1, decisions, 1, skip );
      StateSimd::checkRdCostSkipSbb( 2, decisions, 2, skip );
      StateSimd::checkRdCostSkipSbb( 3, decisions, 3, skip );
    }
  }

private:
  CommonCtx m_commonCtx;
  Decisions m_trellis[MAX_TB_SIZEY * MAX_TB_SIZEY][2];
  Rom m_scansRom;

  StateMem m_state_curr;
  StateMem m_state_skip;

#if defined(TARGET_SIMD_X86) && ENABLE_SIMD_OPT_QUANT
  void initDepQuantSimdX86();
  template <X86_VEXT vext>
  void _initDepQuantSimdX86();
#endif

#if defined(TARGET_SIMD_ARM) && ENABLE_SIMD_OPT_QUANT
  void initDepQuantSimdARM();
  template <ARM_VEXT vext>
  void _initDepQuantSimdARM();
#endif

public:
  void ( *m_updateStates )( const ScanInfo& scanInfo, const Decisions& decisions, StateMem& curr ) = nullptr;
  void ( *m_updateStatesEOS )( const ScanInfo& scanInfo, const Decisions& decisions, const StateMem& skip,
                               StateMem& curr, CommonCtx& commonCtx ) = nullptr;
  void ( *m_checkAllRdCosts )( const ScanPosType spt, const PQData* pqData, Decisions& decisions,
                               const StateMem& state ) = nullptr;
  void ( *m_checkAllRdCostsOdd1 )( const ScanPosType spt, const int64_t pq_a_dist, const int64_t pq_b_dist,
                                   Decisions& decisions, const StateMem& state ) = nullptr;
  int ( *m_xFindFirstTestPos )( const TUParameters& tuPars, const TCoeff* tCoeff, int firstTestPos, TCoeff defaultTh,
                                bool zeroOutforThres, int zeroOutWidth, int zeroOutHeight ) = nullptr;

  // End of class DepQuantSimd.
};

} // namespace DQIntern

//! \}

} // namespace vvenc

#endif //ENABLE_SIMD_OPT_QUANT

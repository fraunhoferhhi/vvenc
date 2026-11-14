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

// The decisions of the dependent quantization trellis for one coefficient, and the loop over a block's coefficients. Only
// DepQuant.cpp and x86/DepQuantX86.h include this: DepQuant.cpp instantiates the templates with its function pointers
// (the scalar kernels, or Arm's), every x86 SIMD translation unit with its own kernels, which then inline. All functions
// here are templates or have internal linkage, so the copies built with different instruction sets stay apart.

#include "DepQuant.h"

#include <cstring>

//! \ingroup CommonLib
//! \{

namespace vvenc {

namespace DQIntern
{
  static const Decisions startDec[2] =
  {
    Decisions
    {
      { rdCostInit >> 2, rdCostInit >> 2, rdCostInit >> 2, rdCostInit >> 2 },
      { -1, -1, -1, -1 },
      { -2, -2, -2, -2 },
    },
    Decisions
    {
      { rdCostInit >> 2, rdCostInit >> 2, rdCostInit >> 2, rdCostInit >> 2 },
      { 0, 0, 0, 0 },
      { 4, 5, 6, 7 },
    }
  };

  template<bool rrgEnsured = false>
  static inline void checkRdCosts( const int stateId, const DQIntern::ScanPosType spt, const DQIntern::PQData& pqDataA, const DQIntern::PQData& pqDataB, DQIntern::Decisions& decisions, int idxAZ, int idxB, const DQIntern::StateMem& state )
  {
    const int32_t* goRiceTab = DQIntern::g_goRiceBits[state.m_goRicePar[stateId]];
    int64_t         rdCostA = state.rdCost[stateId] + pqDataA.deltaDist;
    int64_t         rdCostB = state.rdCost[stateId] + pqDataB.deltaDist;
    int64_t         rdCostZ = state.rdCost[stateId];

    if( rrgEnsured || state.remRegBins[stateId] >= 4 )
    {
      const CoeffFracBits& cffBits = state.m_gtxFracBitsArray[state.ctx.cff[stateId]];
      const BinFracBits    sigBits = state.m_sigFracBitsArray[stateId][state.ctx.sig[stateId]];

      if( pqDataA.absLevel < 4 )
        rdCostA += cffBits.bits[pqDataA.absLevel];
      else
      {
        const unsigned value = ( pqDataA.absLevel - 4 ) >> 1;
        rdCostA += cffBits.bits[pqDataA.absLevel - ( value << 1 )] + goRiceTab[std::min<unsigned>( value, RICEMAX - 1 )];
      }

      if( pqDataB.absLevel < 4 )
        rdCostB += cffBits.bits[pqDataB.absLevel];
      else
      {
        const unsigned value = ( pqDataB.absLevel - 4 ) >> 1;
        rdCostB += cffBits.bits[pqDataB.absLevel - ( value << 1 )] + goRiceTab[std::min<unsigned>( value, RICEMAX - 1 )];
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
      rdCostA += ( 1 << SCALE_BITS ) + goRiceTab[pqDataA.absLevel <= state.m_goRiceZero[stateId] ? pqDataA.absLevel - 1 : std::min<int>( pqDataA.absLevel, RICEMAX - 1 )];
      rdCostB += ( 1 << SCALE_BITS ) + goRiceTab[pqDataB.absLevel <= state.m_goRiceZero[stateId] ? pqDataB.absLevel - 1 : std::min<int>( pqDataB.absLevel, RICEMAX - 1 )];
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

  template<bool rrgEnsured = false>
  static inline void checkRdCostsOdd1( const int stateId, const ScanPosType spt, const int64_t deltaDist, Decisions& decisions, int idxA, int idxZ, const StateMem& state )
  {
    int64_t         rdCostA = state.rdCost[stateId] + deltaDist;
    int64_t         rdCostZ = state.rdCost[stateId];

    if( rrgEnsured || state.remRegBins[stateId] >= 4 )
    {
      const BinFracBits sigBits = state.m_sigFracBitsArray[stateId][state.ctx.sig[stateId]];

      rdCostA += state.cffBits1[state.ctx.cff[stateId]];

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

  static inline void checkRdCostStart( int32_t lastOffset, const PQData& pqData, Decisions& decisions, int idx, const StateMem& state )
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
      decisions.rdCost[idx]   = rdCost;
      decisions.absLevel[idx] = pqData.absLevel;
      decisions.prevId[idx]   = -1;
    }
  }

  static inline void checkRdCostSkipSbb( const int stateId, Decisions& decisions, int idx, const StateMem& state )
  {
    int64_t rdCost = state.rdCost[stateId] + state.sbbBits0[stateId];
    if( rdCost < decisions.rdCost[idx] )
    {
      decisions.rdCost[idx]   = rdCost;
      decisions.absLevel[idx] = 0;
      decisions.prevId[idx]   = 4 | stateId;
    }
  }

  static inline void checkRdCostSkipSbbZeroOut( const int stateId, Decisions& decisions, int idx, const StateMem& state )
  {
    int64_t rdCost          = state.rdCost[stateId] + state.sbbBits0[stateId];
    decisions.rdCost[idx]   = rdCost;
    decisions.absLevel[idx] = 0;
    decisions.prevId[idx]   = 4 | stateId;
  }

  static inline void setRiceParam( const int stateId, const ScanInfo& scanInfo, StateMem& state, bool ge4 )
  {
    if( state.remRegBins[stateId] < 4 || ge4 )
    {
      TCoeff  sumAbs = state.sum1st[scanInfo.insidePos][stateId];
      int sumSub     = state.remRegBins[stateId] < 4 ? 0 : 4 * 5;
      int sumAll     = std::max( std::min( 31, ( int ) sumAbs - sumSub ), 0 );
      state.m_goRicePar[stateId]
                     = g_auiGoRiceParsCoeff[sumAll];

      if( state.remRegBins[stateId] < 4 )
      {
        state.m_goRiceZero[stateId] = g_auiGoRicePosCoeff0( stateId, state.m_goRicePar[stateId] );
      }
    }
  }
} // namespace DQIntern

template<class Kernels>
inline void DepQuant::xDecide( const DQIntern::ScanInfo& scanInfo, const TCoeff absCoeff, const int lastOffset, DQIntern::Decisions& decisions, bool zeroOut, int quantCoeff )
{
  using namespace DQIntern;

  // Kernels::checkAllRdCosts and checkAllRdCostsOdd1 set all of decisions themselves, and this runs for every coefficient:
  // only the paths that compare against it start from startDec
  StateMem& skip = m_state_skip;

  if( zeroOut )
  {
    ::memcpy( &decisions, startDec, sizeof( Decisions ) );
    if( scanInfo.spt == SCAN_EOCSBB )
    {
      checkRdCostSkipSbbZeroOut( 0, decisions, 0, skip );
      checkRdCostSkipSbbZeroOut( 1, decisions, 1, skip );
      checkRdCostSkipSbbZeroOut( 2, decisions, 2, skip );
      checkRdCostSkipSbbZeroOut( 3, decisions, 3, skip );
    }
    return;
  }

  StateMem& prev = m_state_curr;

  /// start inline prequant
  int64_t scaledOrg = int64_t( absCoeff ) * quantCoeff;
  TCoeff  qIdx      = TCoeff( ( scaledOrg + m_quant.m_QAdd ) >> m_quant.m_QShift );

  if( qIdx < 0 )
  {
    int64_t scaledAdd = m_quant.m_DistStepAdd - scaledOrg * m_quant.m_DistOrgFact;
    int64_t pq_a_dist = ( ( scaledAdd + 0 * m_quant.m_DistStepAdd ) * 1 + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
    int64_t pq_b_dist = ( ( scaledAdd + 1 * m_quant.m_DistStepAdd ) * 2 + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
    /// stop inline prequant

    if( prev.anyRemRegBinsLt4 )
    {
      ::memcpy( &decisions, startDec, sizeof( Decisions ) );

      // at a block's first position all states still hold rdCostInit, so none of them can beat startDec's cost
      if( scanInfo.scanIdx != m_firstScanIdx )
      {
        setRiceParam( 0, scanInfo, prev, false );
        checkRdCostsOdd1( 0, scanInfo.spt, pq_b_dist, decisions, 2, 0, prev );

        setRiceParam( 1, scanInfo, prev, false );
        checkRdCostsOdd1( 1, scanInfo.spt, pq_b_dist, decisions, 0, 2, prev );

        setRiceParam( 2, scanInfo, prev, false );
        checkRdCostsOdd1( 2, scanInfo.spt, pq_a_dist, decisions, 3, 1, prev );

        setRiceParam( 3, scanInfo, prev, false );
        checkRdCostsOdd1( 3, scanInfo.spt, pq_a_dist, decisions, 1, 3, prev );
      }
    }
    else
    {
      // has to be called as a first check, assumes no decision has been made yet
      Kernels::checkAllRdCostsOdd1( *this, scanInfo.spt, pq_a_dist, pq_b_dist, decisions, prev );
    }

    checkRdCostStart( lastOffset, PQData{ 1, pq_b_dist }, decisions, 2, prev );
  }
  else
  {
    /// start inline prequant
    // at least 1. qIdx is not negative here, and how often it is 0 is hard to predict: no branch for it
    qIdx  = std::min<TCoeff>( m_quant.m_maxQIdx, qIdx );
    qIdx += TCoeff( qIdx == 0 );
    int64_t scaledAdd = qIdx * m_quant.m_DistStepAdd - scaledOrg * m_quant.m_DistOrgFact;

    PQData  pqData[4];

    PQData& pq_a = pqData[( qIdx + 0 ) & 3];
    PQData& pq_b = pqData[( qIdx + 1 ) & 3];
    PQData& pq_c = pqData[( qIdx + 2 ) & 3];
    PQData& pq_d = pqData[( qIdx + 3 ) & 3];

    pq_a.deltaDist = ( ( scaledAdd + 0 * m_quant.m_DistStepAdd ) * ( qIdx + 0 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
    pq_a.absLevel = ( qIdx + 1 ) >> 1;

    pq_b.deltaDist = ( ( scaledAdd + 1 * m_quant.m_DistStepAdd ) * ( qIdx + 1 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
    pq_b.absLevel = ( qIdx + 2 ) >> 1;

    pq_c.deltaDist = ( ( scaledAdd + 2 * m_quant.m_DistStepAdd ) * ( qIdx + 2 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
    pq_c.absLevel = ( qIdx + 3 ) >> 1;

    pq_d.deltaDist = ( ( scaledAdd + 3 * m_quant.m_DistStepAdd ) * ( qIdx + 3 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
    pq_d.absLevel = ( qIdx + 4 ) >> 1;
    /// stop inline prequant

    bool cff02ge4 = pqData[0].absLevel >= 4/* || pqData[2].absLevel >= 4 */;
    bool cff13ge4 = /* pqData[1].absLevel >= 4 || */ pqData[3].absLevel >= 4;

    if( prev.anyRemRegBinsLt4 )
    {
      ::memcpy( &decisions, startDec, sizeof( Decisions ) );

      // at a block's first position all states still hold rdCostInit, so none of them can beat startDec's cost
      if( scanInfo.scanIdx != m_firstScanIdx )
      {
        setRiceParam( 0, scanInfo, prev, cff02ge4 );
        setRiceParam( 1, scanInfo, prev, cff02ge4 );
        setRiceParam( 2, scanInfo, prev, cff13ge4 );
        setRiceParam( 3, scanInfo, prev, cff13ge4 );

        checkRdCosts( 0, scanInfo.spt, pqData[0], pqData[2], decisions, 0, 2, prev );
        checkRdCosts( 1, scanInfo.spt, pqData[0], pqData[2], decisions, 2, 0, prev );
        checkRdCosts( 2, scanInfo.spt, pqData[3], pqData[1], decisions, 1, 3, prev );
        checkRdCosts( 3, scanInfo.spt, pqData[3], pqData[1], decisions, 3, 1, prev );
      }
    }
    else if( cff02ge4 || cff13ge4 )
    {
      // Every state has 4 or more regular bins left (and this is not a block's first position, which always takes the
      // branch above), so only the Rice code of a level of 4 or more differs from checkAllRdCosts. pqData[2] >= 4 only
      // with pqData[0] >= 4, and pqData[1] >= 4 only with pqData[3] >= 4: the states whose Rice parameter is not set
      // here only see levels below 4, which don't read it.
      if( cff02ge4 )
      {
        setRiceParam( 0, scanInfo, prev, true );
        setRiceParam( 1, scanInfo, prev, true );
      }

      if( cff13ge4 )
      {
        setRiceParam( 2, scanInfo, prev, true );
        setRiceParam( 3, scanInfo, prev, true );
      }

      // has to be called as a first check, assumes no decision has been made yet
      Kernels::checkAllRdCostsGe4( *this, scanInfo.spt, pqData, decisions, prev );
    }
    else
    {
      // has to be called as a first check, assumes no decision has been made yet
      Kernels::checkAllRdCosts( *this, scanInfo.spt, pqData, decisions, prev );
    }

    checkRdCostStart( lastOffset, pqData[0], decisions, 0, prev );
    checkRdCostStart( lastOffset, pqData[2], decisions, 2, prev );
  }

  if( scanInfo.spt == SCAN_EOCSBB )
  {
    checkRdCostSkipSbb( 0, decisions, 0, skip );
    checkRdCostSkipSbb( 1, decisions, 1, skip );
    checkRdCostSkipSbb( 2, decisions, 2, skip );
    checkRdCostSkipSbb( 3, decisions, 3, skip );
  }
}

template<class Kernels>
inline void DepQuant::xDecideAndUpdate( const TCoeff absCoeff, const DQIntern::ScanInfo& scanInfo, bool zeroOut, int quantCoeff )
{
  using namespace DQIntern;

  Decisions* decisions = &m_trellis[scanInfo.scanIdx][0];

  xDecide<Kernels>( scanInfo, absCoeff, lastOffset( scanInfo ), *decisions, zeroOut, quantCoeff );

  if( scanInfo.scanIdx )
  {
    if( scanInfo.spt == SCAN_SOCSBB )
    {
      memcpy( &m_state_skip, &m_state_curr, DQIntern::StateMemSkipCpySize );
    }

    if( scanInfo.insidePos == 0 )
    {
      m_commonCtx.swap();
      Kernels::updateStatesEOS( *this, scanInfo, *decisions, m_state_skip, m_state_curr, m_commonCtx );
      ::memcpy( decisions + 1, decisions, sizeof( Decisions ) );
    }
    else if( !zeroOut )
    {
      Kernels::updateStates( *this, scanInfo, *decisions, m_state_curr );
    }
  }
}

// The trellis of a block without scaling lists: the decisions and state updates of its coefficients from firstTestPos back
// to the first. Instantiated per set of kernels, so that a block costs one indirect call (m_populateTrellis) instead of up
// to three per coefficient.
template<class Kernels>
void DepQuant::xPopulateTrellis( DepQuant& dq, const DQIntern::TUParameters& tuPars, const int firstTestPos, const TCoeff* tCoeff,
                                 const bool zeroOut, const int effWidth, const int effHeight, const int quantCoeff )
{
  for( int scanIdx = firstTestPos; scanIdx >= 0; scanIdx-- )
  {
    const DQIntern::ScanInfo& scanInfo = tuPars.m_scanInfo[scanIdx];
    dq.xDecideAndUpdate<Kernels>( abs( tCoeff[scanInfo.rasterPos] ), scanInfo, zeroOut && ( scanInfo.posX >= effWidth || scanInfo.posY >= effHeight ), quantCoeff );
  }
}

} // namespace vvenc

//! \}

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
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

/** \file     DepQuant_neon.cpp
    \brief    Dependent Quantization ARM NEON implementation
*/

#include "../CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/DepQuant.h"
#include "CommonLib/CodingStructure.h"

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_QUANT

#include <arm_neon.h>
#include <algorithm>

namespace vvenc
{

namespace DQIntern
{

static constexpr int64_t rdCostInit = std::numeric_limits<int64_t>::max() >> 1;

// same layout as DQIntern::Decisions in DepQuantX86.h
struct Decisions
{
  int64_t   rdCost  [4];
  TCoeffSig absLevel[4];
  int8_t    prevId  [4];
};

struct SbbCtx
{
  uint8_t* sbbFlags;
  uint8_t* levels;
};

// same layout as DQIntern::StateMem in DepQuantX86.h
struct StateMem
{
  int64_t  rdCost[4];
  int16_t  remRegBins[4];
  int32_t  sbbBits0[4];
  int32_t  sbbBits1[4];

  uint8_t tplAcc[64];
  uint8_t sum1st[64];
  uint8_t absVal[64];

  struct
  {
    uint8_t sig[4];
    uint8_t cff[4];
  } ctx;

  uint8_t numSig[4];
  int8_t  refSbbCtxId[4];

  int32_t cffBits1[RateEstimator::sm_maxNumGtxCtx + 3];

  int8_t   m_goRicePar[4];
  int8_t   m_goRiceZero[4];
  const BinFracBits*   m_sigFracBitsArray[4];
  const CoeffFracBits* m_gtxFracBitsArray;

  int  cffBitsCtxOffset;
  bool anyRemRegBinsLt4;
  int  initRemRegBins;
};

static constexpr size_t StateMemSkipCpySize = offsetof( StateMem, sbbBits1 );

// ---------------------------------------------------------------------------
// CommonCtxNeon: context tracking, scalar (same logic as CommonCtx<vext>)
// ---------------------------------------------------------------------------
class CommonCtxNeon
{
public:
  CommonCtxNeon() : m_currSbbCtx( m_allSbbCtx ), m_prevSbbCtx( m_currSbbCtx + 4 ) {}

  inline void swap() { std::swap( m_currSbbCtx, m_prevSbbCtx ); }

  inline void reset( const TUParameters& tuPars, const RateEstimator& rateEst )
  {
    m_nbInfo = tuPars.m_scanId2NbInfoOut;
    ::memcpy( m_sbbFlagBits, rateEst.sigSbbFracBits(), 2 * sizeof( BinFracBits ) );
    const int numSbb    = tuPars.m_numSbb;
    const int chunkSize = numSbb + tuPars.m_numCoeff;
    uint8_t*  nextMem   = m_memory;
    for( int k = 0; k < 8; k++, nextMem += chunkSize )
    {
      m_allSbbCtx[k].sbbFlags = nextMem;
      m_allSbbCtx[k].levels   = nextMem + numSbb;
    }
  }

  inline void update( const ScanInfo& scanInfo, const int prevId, int stateId, StateMem& curr )
  {
    uint8_t*    sbbFlags  = m_currSbbCtx[stateId].sbbFlags;
    uint8_t*    levels    = m_currSbbCtx[stateId].levels;
    uint16_t    maxDist   = m_nbInfo[scanInfo.scanIdx - 1].maxDist;
    uint16_t    sbbSize   = scanInfo.sbbSize;
    std::size_t setCpSize = ( maxDist > sbbSize ? maxDist - sbbSize : 0 ) * sizeof( uint8_t );

    if( prevId >= 0 )
    {
      ::memcpy( sbbFlags, m_prevSbbCtx[prevId].sbbFlags, scanInfo.numSbb * sizeof( uint8_t ) );
      ::memcpy( levels + scanInfo.scanIdx + sbbSize,
                m_prevSbbCtx[prevId].levels + scanInfo.scanIdx + sbbSize,
                setCpSize );
    }
    else
    {
      ::memset( sbbFlags, 0, scanInfo.numSbb * sizeof( uint8_t ) );
      ::memset( levels + scanInfo.scanIdx + sbbSize, 0, setCpSize );
    }
    sbbFlags[scanInfo.sbbPos] = !!curr.numSig[stateId];

    const int sigNSbb =
      ( ( scanInfo.nextSbbRight ? sbbFlags[scanInfo.nextSbbRight] : false ) ||
        ( scanInfo.nextSbbBelow ? sbbFlags[scanInfo.nextSbbBelow] : false ) )
      ? 1 : 0;
    curr.refSbbCtxId[stateId] = stateId;
    const BinFracBits sbbBits  = m_sbbFlagBits[sigNSbb];
    curr.sbbBits0[stateId]     = sbbBits.intBits[0];
    curr.sbbBits1[stateId]     = sbbBits.intBits[1];

    if( sigNSbb ||
        ( ( scanInfo.nextSbbRight && scanInfo.nextSbbBelow )
          ? sbbFlags[scanInfo.nextSbbBelow + 1] : false ) )
    {
      const int        scanBeg   = scanInfo.scanIdx - scanInfo.sbbSize;
      const NbInfoOut* nbOut     = m_nbInfo + scanBeg;
      const uint8_t*   absLevels = levels   + scanBeg;

      for( int id = 0; id < scanInfo.sbbSize; id++, nbOut++ )
      {
        const int idAddr = ( id << 2 ) + stateId;
        if( nbOut->num )
        {
          TCoeff sumAbs = 0, sumAbs1 = 0, sumNum = 0;
#define UPDATE(k) { TCoeff t = absLevels[nbOut->outPos[k]]; sumAbs += t; sumAbs1 += std::min<TCoeff>( 4 + ( t & 1 ), t ); sumNum += !!t; }
          switch( nbOut->num )
          {
          default:
          case 5: UPDATE(4);
          case 4: UPDATE(3);
          case 3: UPDATE(2);
          case 2: UPDATE(1);
          case 1: UPDATE(0);
          }
#undef UPDATE
          curr.tplAcc[idAddr] = ( sumNum << 5 ) | sumAbs1;
          curr.sum1st[idAddr] = (uint8_t)std::min( 255, (int)sumAbs );
        }
      }
    }
  }

  // De-interleave absVal (4-state interleaved) into per-state level arrays
  inline void updateAllLvls( const ScanInfo& scanInfo, const StateMem& curr )
  {
    for( int j = 0; j < scanInfo.sbbSize; j++ )
    {
      m_currSbbCtx[0].levels[scanInfo.scanIdx + j] = curr.absVal[j * 4 + 0];
      m_currSbbCtx[1].levels[scanInfo.scanIdx + j] = curr.absVal[j * 4 + 1];
      m_currSbbCtx[2].levels[scanInfo.scanIdx + j] = curr.absVal[j * 4 + 2];
      m_currSbbCtx[3].levels[scanInfo.scanIdx + j] = curr.absVal[j * 4 + 3];
    }
  }

private:
  const NbInfoOut* m_nbInfo;
  BinFracBits      m_sbbFlagBits[2];
  SbbCtx           m_allSbbCtx[8];
  SbbCtx*          m_currSbbCtx;
  SbbCtx*          m_prevSbbCtx;
  uint8_t          m_memory[8 * ( MAX_TB_SIZEY * MAX_TB_SIZEY + MLS_GRP_NUM )];
};

// ---------------------------------------------------------------------------
// StateNeon: per-scan-position updates
// ---------------------------------------------------------------------------
class StateNeon
{
public:

  static inline void init( const int stateId, StateMem& state )
  {
    state.rdCost     [stateId] = rdCostInit;
    state.ctx.cff    [stateId] = 0;
    state.ctx.sig    [stateId] = 0;
    state.numSig     [stateId] = 0;
    state.refSbbCtxId[stateId] = -1;
    state.remRegBins [stateId] = 4;
    state.cffBitsCtxOffset     = 0;
    state.m_goRicePar [stateId] = 0;
    state.m_goRiceZero[stateId] = 0;
    state.sbbBits0    [stateId] = 0;
    state.sbbBits1    [stateId] = 0;
  }

  static inline void setRiceParam( const int stateId, const ScanInfo& scanInfo,
                                   StateMem& state, bool ge4 )
  {
    if( state.remRegBins[stateId] < 4 || ge4 )
    {
      const int addr  = ( scanInfo.insidePos << 2 ) + stateId;
      TCoeff    sumAbs = state.sum1st[addr];
      int       sumSub = state.remRegBins[stateId] < 4 ? 0 : 4 * 5;
      int       sumAll = std::max( std::min( 31, (int)sumAbs - sumSub ), 0 );
      state.m_goRicePar[stateId] = g_auiGoRiceParsCoeff[sumAll];
      if( state.remRegBins[stateId] < 4 )
        state.m_goRiceZero[stateId] = g_auiGoRicePosCoeff0( stateId, state.m_goRicePar[stateId] );
    }
  }

  static inline void checkRdCosts( const int stateId, const ScanPosType spt,
                                   const PQData& pqDataA, const PQData& pqDataB,
                                   Decisions& decisions, int idxAZ, int idxB,
                                   const StateMem& state )
  {
    const int32_t* goRiceTab = g_goRiceBits[state.m_goRicePar[stateId]];
    int64_t rdCostA = state.rdCost[stateId] + pqDataA.deltaDist;
    int64_t rdCostB = state.rdCost[stateId] + pqDataB.deltaDist;
    int64_t rdCostZ = state.rdCost[stateId];

    if( state.remRegBins[stateId] >= 4 )
    {
      const CoeffFracBits& cffBits = state.m_gtxFracBitsArray[state.ctx.cff[stateId]];
      const BinFracBits    sigBits = state.m_sigFracBitsArray[stateId][state.ctx.sig[stateId]];

      if( pqDataA.absLevel < 4 )
        rdCostA += cffBits.bits[pqDataA.absLevel];
      else
      {
        const unsigned value = ( pqDataA.absLevel - 4 ) >> 1;
        rdCostA += cffBits.bits[pqDataA.absLevel - ( value << 1 )] +
                   goRiceTab[std::min<unsigned>( value, RICEMAX - 1 )];
      }
      if( pqDataB.absLevel < 4 )
        rdCostB += cffBits.bits[pqDataB.absLevel];
      else
      {
        const unsigned value = ( pqDataB.absLevel - 4 ) >> 1;
        rdCostB += cffBits.bits[pqDataB.absLevel - ( value << 1 )] +
                   goRiceTab[std::min<unsigned>( value, RICEMAX - 1 )];
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
      rdCostA += ( 1 << SCALE_BITS ) +
                 goRiceTab[pqDataA.absLevel <= state.m_goRiceZero[stateId]
                           ? pqDataA.absLevel - 1
                           : std::min<int>( pqDataA.absLevel, RICEMAX - 1 )];
      rdCostB += ( 1 << SCALE_BITS ) +
                 goRiceTab[pqDataB.absLevel <= state.m_goRiceZero[stateId]
                           ? pqDataB.absLevel - 1
                           : std::min<int>( pqDataB.absLevel, RICEMAX - 1 )];
      rdCostZ += goRiceTab[state.m_goRiceZero[stateId]];
    }

    if( rdCostA < rdCostZ && rdCostA < decisions.rdCost[idxAZ] )
    {
      decisions.rdCost  [idxAZ] = rdCostA;
      decisions.absLevel[idxAZ] = pqDataA.absLevel;
      decisions.prevId  [idxAZ] = stateId;
    }
    else if( rdCostZ < decisions.rdCost[idxAZ] )
    {
      decisions.rdCost  [idxAZ] = rdCostZ;
      decisions.absLevel[idxAZ] = 0;
      decisions.prevId  [idxAZ] = stateId;
    }
    if( rdCostB < decisions.rdCost[idxB] )
    {
      decisions.rdCost  [idxB] = rdCostB;
      decisions.absLevel[idxB] = pqDataB.absLevel;
      decisions.prevId  [idxB] = stateId;
    }
  }

  static inline void checkRdCostsOdd1( const int stateId, const ScanPosType spt,
                                       const int64_t deltaDist, Decisions& decisions,
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
      decisions.rdCost  [idxA] = rdCostA;
      decisions.absLevel[idxA] = 1;
      decisions.prevId  [idxA] = stateId;
    }
    if( rdCostZ < decisions.rdCost[idxZ] )
    {
      decisions.rdCost  [idxZ] = rdCostZ;
      decisions.absLevel[idxZ] = 0;
      decisions.prevId  [idxZ] = stateId;
    }
  }

  static inline void checkRdCostStart( int32_t lastOffset, const PQData& pqData,
                                       Decisions& decisions, int idx,
                                       const StateMem& state )
  {
    const CoeffFracBits& cffBits = state.m_gtxFracBitsArray[0];
    int64_t rdCost = pqData.deltaDist + lastOffset;
    if( pqData.absLevel < 4 )
      rdCost += cffBits.bits[pqData.absLevel];
    else
    {
      const unsigned value = ( pqData.absLevel - 4 ) >> 1;
      rdCost += cffBits.bits[pqData.absLevel - ( value << 1 )] +
                g_goRiceBits[0][value < RICEMAX ? value : RICEMAX - 1];
    }
    if( rdCost < decisions.rdCost[idx] )
    {
      decisions.rdCost  [idx] = rdCost;
      decisions.absLevel[idx] = pqData.absLevel;
      decisions.prevId  [idx] = -1;
    }
  }

  static inline void checkRdCostSkipSbb( const int stateId, Decisions& decisions,
                                         int idx, const StateMem& state )
  {
    int64_t rdCost = state.rdCost[stateId] + state.sbbBits0[stateId];
    if( rdCost < decisions.rdCost[idx] )
    {
      decisions.rdCost  [idx] = rdCost;
      decisions.absLevel[idx] = 0;
      decisions.prevId  [idx] = 4 | stateId;
    }
  }

  static inline void checkRdCostSkipSbbZeroOut( const int stateId, Decisions& decisions,
                                                int idx, const StateMem& state )
  {
    int64_t rdCost          = state.rdCost[stateId] + state.sbbBits0[stateId];
    decisions.rdCost  [idx] = rdCost;
    decisions.absLevel[idx] = 0;
    decisions.prevId  [idx] = 4 | stateId;
  }

  // checkAllRdCosts: all pqData[i].absLevel < 4, remRegBins >= 4.
  // State-decision mapping:
  //   d0: Z/A from state0 (pq0/pq0), B from state1 (pq2)
  //   d1: Z/A from state2 (pq3/pq3), B from state3 (pq1)
  //   d2: Z/A from state1 (pq0/pq0), B from state0 (pq2)
  //   d3: Z/A from state3 (pq3/pq3), B from state2 (pq1)
  static void checkAllRdCosts( const ScanPosType spt, const PQData* pqData,
                               Decisions& decisions, const StateMem& state )
  {
    // Load rdCost[0..3]
    int64x2_t mrd01 = vld1q_s64( &state.rdCost[0] );          // [rc0, rc1]
    int64x2_t mrd23 = vld1q_s64( &state.rdCost[2] );          // [rc2, rc3]

    // rdCostZ01=[rc0,rc2], rdCostZ23=[rc1,rc3]
    int64x2_t rdCostZ01 = vcombine_s64( vget_low_s64(mrd01),  vget_low_s64(mrd23)  );
    int64x2_t rdCostZ23 = vcombine_s64( vget_high_s64(mrd01), vget_high_s64(mrd23) );

    // B costs: d0←state1+pq2, d1←state3+pq1 / d2←state0+pq2, d3←state2+pq1
    int64x2_t deltaDistB = vcombine_s64( vld1_s64(&pqData[2].deltaDist),
                                         vld1_s64(&pqData[1].deltaDist) );
    int64x2_t rdCostB01 = vaddq_s64( rdCostZ23, deltaDistB );
    int64x2_t rdCostB23 = vaddq_s64( rdCostZ01, deltaDistB );

    // A costs: d0←state0+pq0, d1←state2+pq3 / d2←state1+pq0, d3←state3+pq3
    int64x2_t deltaDistA = vcombine_s64( vld1_s64(&pqData[0].deltaDist),
                                         vld1_s64(&pqData[3].deltaDist) );
    int64x2_t rdCostA01 = vaddq_s64( rdCostZ01, deltaDistA );
    int64x2_t rdCostA23 = vaddq_s64( rdCostZ23, deltaDistA );

    // Load BinFracBits (intBits[0], intBits[1]) for each state
    // After de-interleave: sgbts02 = [sg0_b0, sg2_b0, sg0_b1, sg2_b1]
    //                      sgbts13 = [sg1_b0, sg3_b0, sg1_b1, sg3_b1]
    int32x4_t sgbts02 = vcombine_s32(
        vld1_s32( (const int32_t*)&state.m_sigFracBitsArray[0][state.ctx.sig[0]] ),
        vld1_s32( (const int32_t*)&state.m_sigFracBitsArray[2][state.ctx.sig[2]] ) );
    int32x4_t sgbts13 = vcombine_s32(
        vld1_s32( (const int32_t*)&state.m_sigFracBitsArray[1][state.ctx.sig[1]] ),
        vld1_s32( (const int32_t*)&state.m_sigFracBitsArray[3][state.ctx.sig[3]] ) );

    // De-interleave: [s0b0,s0b1,s2b0,s2b1] → [s0b0,s2b0,s0b1,s2b1]
    {
      int32x4x2_t uzp02 = vuzpq_s32( sgbts02, sgbts02 );
      sgbts02 = vcombine_s32( vget_low_s32(uzp02.val[0]), vget_low_s32(uzp02.val[1]) );
      int32x4x2_t uzp13 = vuzpq_s32( sgbts13, sgbts13 );
      sgbts13 = vcombine_s32( vget_low_s32(uzp13.val[0]), vget_low_s32(uzp13.val[1]) );
    }

    // Add cffBits to rdCostB (scalar table lookup; absLevel < 4 guaranteed)
    {
      const CoeffFracBits* base = state.m_gtxFracBitsArray;
      int32_t arr[4] = {
        base[state.ctx.cff[1]].bits[pqData[2].absLevel],
        base[state.ctx.cff[3]].bits[pqData[1].absLevel],
        base[state.ctx.cff[0]].bits[pqData[2].absLevel],
        base[state.ctx.cff[2]].bits[pqData[1].absLevel],
      };
      int32x4_t cffB = vld1q_s32( arr );
      rdCostB01 = vaddq_s64( rdCostB01, vmovl_s32( vget_low_s32(cffB)  ) );
      rdCostB23 = vaddq_s64( rdCostB23, vmovl_s32( vget_high_s32(cffB) ) );
    }

    // Add cffBits to rdCostA
    {
      const CoeffFracBits* base = state.m_gtxFracBitsArray;
      int32_t arr[4] = {
        base[state.ctx.cff[0]].bits[pqData[0].absLevel],
        base[state.ctx.cff[2]].bits[pqData[3].absLevel],
        base[state.ctx.cff[1]].bits[pqData[0].absLevel],
        base[state.ctx.cff[3]].bits[pqData[3].absLevel],
      };
      int32x4_t cffA = vld1q_s32( arr );
      rdCostA01 = vaddq_s64( rdCostA01, vmovl_s32( vget_low_s32(cffA)  ) );
      rdCostA23 = vaddq_s64( rdCostA23, vmovl_s32( vget_high_s32(cffA) ) );
    }

    // Add sigBits based on scan position type
    int32x2_t sg02_b0 = vget_low_s32( sgbts02 );   // [sg0_bits0, sg2_bits0]
    int32x2_t sg13_b0 = vget_low_s32( sgbts13 );
    int32x2_t sg02_b1 = vget_high_s32( sgbts02 );  // [sg0_bits1, sg2_bits1]
    int32x2_t sg13_b1 = vget_high_s32( sgbts13 );

    if( spt == SCAN_ISCSBB )
    {
      rdCostZ01 = vaddq_s64( rdCostZ01, vmovl_s32(sg02_b0) );
      rdCostZ23 = vaddq_s64( rdCostZ23, vmovl_s32(sg13_b0) );
      rdCostB01 = vaddq_s64( rdCostB01, vmovl_s32(sg13_b1) );
      rdCostB23 = vaddq_s64( rdCostB23, vmovl_s32(sg02_b1) );
      rdCostA01 = vaddq_s64( rdCostA01, vmovl_s32(sg02_b1) );
      rdCostA23 = vaddq_s64( rdCostA23, vmovl_s32(sg13_b1) );
    }
    else if( spt == SCAN_SOCSBB )
    {
      // sbbBits1 reordered: even states→lo64, odd states→hi64
      int32x4_t sbbBits1 = vld1q_s32( state.sbbBits1 );   // [sb0,sb1,sb2,sb3]
      int32x4x2_t uzp    = vuzpq_s32( sbbBits1, sbbBits1 );
      int64x2_t sbb02    = vmovl_s32( vget_low_s32(uzp.val[0]) );  // [sb0,sb2]
      int64x2_t sbb13    = vmovl_s32( vget_low_s32(uzp.val[1]) );  // [sb1,sb3]

      rdCostZ01 = vaddq_s64( rdCostZ01, vmovl_s32(sg02_b0) );
      rdCostZ23 = vaddq_s64( rdCostZ23, vmovl_s32(sg13_b0) );

      rdCostB23 = vaddq_s64( rdCostB23, sbb02 );
      rdCostA01 = vaddq_s64( rdCostA01, sbb02 );
      rdCostZ01 = vaddq_s64( rdCostZ01, sbb02 );

      rdCostB01 = vaddq_s64( rdCostB01, sbb13 );
      rdCostA23 = vaddq_s64( rdCostA23, sbb13 );
      rdCostZ23 = vaddq_s64( rdCostZ23, sbb13 );

      rdCostB01 = vaddq_s64( rdCostB01, vmovl_s32(sg13_b1) );
      rdCostB23 = vaddq_s64( rdCostB23, vmovl_s32(sg02_b1) );
      rdCostA01 = vaddq_s64( rdCostA01, vmovl_s32(sg02_b1) );
      rdCostA23 = vaddq_s64( rdCostA23, vmovl_s32(sg13_b1) );
    }
    else
    {
      // Common interior case: conditionally add sigBits; zero-out rdCostZ when numSig=0
      rdCostZ01 = vaddq_s64( rdCostZ01, vmovl_s32(sg02_b0) );
      rdCostZ23 = vaddq_s64( rdCostZ23, vmovl_s32(sg13_b0) );

      // Build 64-bit lane masks from numSig[0..3]
      uint32_t numSigU32;
      memcpy( &numSigU32, state.numSig, 4 );
      uint8x16_t numSigV = vreinterpretq_u8_u32( vdupq_n_u32(0) );
      numSigV = vreinterpretq_u8_u32( vld1q_lane_u32( &numSigU32, vreinterpretq_u32_u8(numSigV), 0 ) );
      static const uint8_t sh02_data[16] = {0,0,0,0,0,0,0,0, 2,2,2,2,2,2,2,2};
      static const uint8_t sh13_data[16] = {1,1,1,1,1,1,1,1, 3,3,3,3,3,3,3,3};
      uint8x16_t sh02 = vld1q_u8( sh02_data );
      uint8x16_t sh13 = vld1q_u8( sh13_data );
      uint64x2_t mask02 = vreinterpretq_u64_s8( vcgtq_s8( vreinterpretq_s8_u8( vqtbl1q_u8(numSigV, sh02) ), vdupq_n_s8(0) ) );
      uint64x2_t mask13 = vreinterpretq_u64_s8( vcgtq_s8( vreinterpretq_s8_u8( vqtbl1q_u8(numSigV, sh13) ), vdupq_n_s8(0) ) );

      int64x2_t sgB01 = vmovl_s32( sg13_b1 );
      int64x2_t sgB23 = vmovl_s32( sg02_b1 );

      rdCostB01 = vaddq_s64( rdCostB01,
          vreinterpretq_s64_u64( vandq_u64( mask13, vreinterpretq_u64_s64(sgB01) ) ) );
      rdCostB23 = vaddq_s64( rdCostB23,
          vreinterpretq_s64_u64( vandq_u64( mask02, vreinterpretq_u64_s64(sgB23) ) ) );
      rdCostA01 = vaddq_s64( rdCostA01,
          vreinterpretq_s64_u64( vandq_u64( mask02, vreinterpretq_u64_s64(sgB23) ) ) );
      rdCostA23 = vaddq_s64( rdCostA23,
          vreinterpretq_s64_u64( vandq_u64( mask13, vreinterpretq_u64_s64(sgB01) ) ) );

      int64x2_t rdMax = vdupq_n_s64( rdCostInit );
      rdCostZ01 = vbslq_s64( mask02, rdCostZ01, rdMax );
      rdCostZ23 = vbslq_s64( mask13, rdCostZ23, rdMax );
    }

    // pick best: Z01 vs A01, B23 vs Z23
    int64x2_t rdBest01 = rdCostZ01;
    int64x2_t rdBest23 = rdCostB23;

    int32x4_t valBest = { 0, 0, (int32_t)pqData[2].absLevel, (int32_t)pqData[1].absLevel };
    int32x4_t valCand = { (int32_t)pqData[0].absLevel, (int32_t)pqData[3].absLevel, 0, 0 };
    int32x4_t idxBest = { 0, 2, 0, 2 };
    int32x4_t idxCand = { 0, 2, 1, 3 };

    uint64x2_t chng01 = vcgtq_s64( rdBest01, rdCostA01 );
    uint64x2_t chng23 = vcgtq_s64( rdBest23, rdCostZ23 );
    uint32x4_t chng   = vcombine_u32( vmovn_u64(chng01), vmovn_u64(chng23) );

    rdBest01 = vbslq_s64( chng01, rdCostA01, rdBest01 );
    rdBest23 = vbslq_s64( chng23, rdCostZ23, rdBest23 );
    valBest  = vbslq_s32( chng,   valCand,   valBest  );
    idxBest  = vbslq_s32( chng,   idxCand,   idxBest  );

    // pick best: B01 vs current, A23 vs current
    valCand = (int32x4_t){ (int32_t)pqData[2].absLevel, (int32_t)pqData[1].absLevel,
                           (int32_t)pqData[0].absLevel, (int32_t)pqData[3].absLevel };
    idxCand = (int32x4_t){ 1, 3, 1, 3 };

    chng01 = vcgtq_s64( rdBest01, rdCostB01 );
    chng23 = vcgtq_s64( rdBest23, rdCostA23 );
    chng   = vcombine_u32( vmovn_u64(chng01), vmovn_u64(chng23) );

    rdBest01 = vbslq_s64( chng01, rdCostB01, rdBest01 );
    rdBest23 = vbslq_s64( chng23, rdCostA23, rdBest23 );
    valBest  = vbslq_s32( chng,   valCand,   valBest  );
    idxBest  = vbslq_s32( chng,   idxCand,   idxBest  );

    vst1q_s64( &decisions.rdCost[0], rdBest01 );
    vst1q_s64( &decisions.rdCost[2], rdBest23 );

    // Pack absLevel: int32x4 → int16x4 (saturating) → store 8 bytes
    vst1_s16( (int16_t*)decisions.absLevel, vqmovn_s32(valBest) );

    // Pack prevId: int32x4 → int16x4 → int8x8 → store 4 bytes
    int8x8_t idx8 = vmovn_s16( vcombine_s16( vmovn_s32(idxBest), vdup_n_s16(0) ) );
    vst1_lane_u32( (uint32_t*)decisions.prevId, vreinterpret_u32_s8(idx8), 0 );
  }

  // checkAllRdCostsOdd1: absLevel == 1 for A-path, remRegBins >= 4.
  static inline void checkAllRdCostsOdd1( const ScanPosType spt,
                                          const int64_t pq_a_dist,
                                          const int64_t pq_b_dist,
                                          Decisions& decisions,
                                          const StateMem& state )
  {
    int64x2_t mrd01 = vld1q_s64( &state.rdCost[0] );
    int64x2_t mrd23 = vld1q_s64( &state.rdCost[2] );
    int64x2_t rdCostZ01 = vcombine_s64( vget_low_s64(mrd01),  vget_low_s64(mrd23)  );
    int64x2_t rdCostZ23 = vcombine_s64( vget_high_s64(mrd01), vget_high_s64(mrd23) );

    // A costs: d0/d1 use states 1/3 (pq_b_dist), d2/d3 use states 0/2 (pq_a_dist)
    int64x2_t deltaDistB = vcombine_s64( vdup_n_s64(pq_b_dist), vdup_n_s64(pq_a_dist) );
    int64x2_t rdCostA01  = vaddq_s64( rdCostZ23, deltaDistB );
    int64x2_t rdCostA23  = vaddq_s64( rdCostZ01, deltaDistB );

    // sigFracBits de-interleave
    int32x4_t sgbts02 = vcombine_s32(
        vld1_s32( (const int32_t*)&state.m_sigFracBitsArray[0][state.ctx.sig[0]] ),
        vld1_s32( (const int32_t*)&state.m_sigFracBitsArray[2][state.ctx.sig[2]] ) );
    int32x4_t sgbts13 = vcombine_s32(
        vld1_s32( (const int32_t*)&state.m_sigFracBitsArray[1][state.ctx.sig[1]] ),
        vld1_s32( (const int32_t*)&state.m_sigFracBitsArray[3][state.ctx.sig[3]] ) );
    {
      int32x4x2_t uzp02 = vuzpq_s32( sgbts02, sgbts02 );
      sgbts02 = vcombine_s32( vget_low_s32(uzp02.val[0]), vget_low_s32(uzp02.val[1]) );
      int32x4x2_t uzp13 = vuzpq_s32( sgbts13, sgbts13 );
      sgbts13 = vcombine_s32( vget_low_s32(uzp13.val[0]), vget_low_s32(uzp13.val[1]) );
    }

    // cffBits[1] for absLevel==1; states reordered for d0←st1,d1←st3,d2←st0,d3←st2
    {
      int32_t arr[4] = {
        state.m_gtxFracBitsArray[state.ctx.cff[1]].bits[1],
        state.m_gtxFracBitsArray[state.ctx.cff[3]].bits[1],
        state.m_gtxFracBitsArray[state.ctx.cff[0]].bits[1],
        state.m_gtxFracBitsArray[state.ctx.cff[2]].bits[1],
      };
      int32x4_t cff = vld1q_s32( arr );
      rdCostA01 = vaddq_s64( rdCostA01, vmovl_s32( vget_low_s32(cff)  ) );
      rdCostA23 = vaddq_s64( rdCostA23, vmovl_s32( vget_high_s32(cff) ) );
    }

    int32x2_t sg02_b0 = vget_low_s32( sgbts02 );
    int32x2_t sg13_b0 = vget_low_s32( sgbts13 );
    int32x2_t sg02_b1 = vget_high_s32( sgbts02 );
    int32x2_t sg13_b1 = vget_high_s32( sgbts13 );

    if( spt == SCAN_ISCSBB )
    {
      rdCostZ01 = vaddq_s64( rdCostZ01, vmovl_s32(sg02_b0) );
      rdCostZ23 = vaddq_s64( rdCostZ23, vmovl_s32(sg13_b0) );
      rdCostA01 = vaddq_s64( rdCostA01, vmovl_s32(sg13_b1) );
      rdCostA23 = vaddq_s64( rdCostA23, vmovl_s32(sg02_b1) );
    }
    else if( spt == SCAN_SOCSBB )
    {
      int32x4_t sbbBits1 = vld1q_s32( state.sbbBits1 );
      int32x4x2_t uzp    = vuzpq_s32( sbbBits1, sbbBits1 );
      int64x2_t sbb02    = vmovl_s32( vget_low_s32(uzp.val[0]) );
      int64x2_t sbb13    = vmovl_s32( vget_low_s32(uzp.val[1]) );

      rdCostZ01 = vaddq_s64( rdCostZ01, vmovl_s32(sg02_b0) );
      rdCostZ23 = vaddq_s64( rdCostZ23, vmovl_s32(sg13_b0) );

      rdCostA23 = vaddq_s64( rdCostA23, sbb02 );
      rdCostZ01 = vaddq_s64( rdCostZ01, sbb02 );
      rdCostA01 = vaddq_s64( rdCostA01, sbb13 );
      rdCostZ23 = vaddq_s64( rdCostZ23, sbb13 );

      rdCostA01 = vaddq_s64( rdCostA01, vmovl_s32(sg13_b1) );
      rdCostA23 = vaddq_s64( rdCostA23, vmovl_s32(sg02_b1) );
    }
    else
    {
      rdCostZ01 = vaddq_s64( rdCostZ01, vmovl_s32(sg02_b0) );
      rdCostZ23 = vaddq_s64( rdCostZ23, vmovl_s32(sg13_b0) );

      uint32_t numSigU32;
      memcpy( &numSigU32, state.numSig, 4 );
      uint8x16_t numSigV = vreinterpretq_u8_u32( vdupq_n_u32(0) );
      numSigV = vreinterpretq_u8_u32( vld1q_lane_u32( &numSigU32, vreinterpretq_u32_u8(numSigV), 0 ) );
      static const uint8_t sh01_data[16] = {1,1,1,1,1,1,1,1, 3,3,3,3,3,3,3,3};
      static const uint8_t sh23_data[16] = {0,0,0,0,0,0,0,0, 2,2,2,2,2,2,2,2};
      uint8x16_t sh01 = vld1q_u8( sh01_data );
      uint8x16_t sh23 = vld1q_u8( sh23_data );
      uint64x2_t mask_odd  = vreinterpretq_u64_s8( vcgtq_s8( vreinterpretq_s8_u8( vqtbl1q_u8(numSigV, sh01) ), vdupq_n_s8(0) ) );
      uint64x2_t mask_even = vreinterpretq_u64_s8( vcgtq_s8( vreinterpretq_s8_u8( vqtbl1q_u8(numSigV, sh23) ), vdupq_n_s8(0) ) );

      int64x2_t sgA01 = vmovl_s32( sg13_b1 );
      int64x2_t sgA23 = vmovl_s32( sg02_b1 );

      rdCostA01 = vaddq_s64( rdCostA01,
          vreinterpretq_s64_u64( vandq_u64( mask_odd,  vreinterpretq_u64_s64(sgA01) ) ) );
      rdCostA23 = vaddq_s64( rdCostA23,
          vreinterpretq_s64_u64( vandq_u64( mask_even, vreinterpretq_u64_s64(sgA23) ) ) );

      int64x2_t rdMax = vdupq_n_s64( rdCostInit );
      rdCostZ01 = vbslq_s64( mask_even, rdCostZ01, rdMax );
      rdCostZ23 = vbslq_s64( mask_odd,  rdCostZ23, rdMax );
    }

    // d0: Z0 vs A0, d1: Z1 vs A1, d2: A2 vs Z2, d3: A3 vs Z3
    int64x2_t rdBest01 = rdCostZ01;
    int64x2_t rdBest23 = rdCostA23;

    int32x4_t valBest = { 0, 0, 1, 1 };
    int32x4_t valCand = { 1, 1, 0, 0 };
    int32x4_t idxBest = { 0, 2, 0, 2 };
    int32x4_t idxCand = { 1, 3, 1, 3 };

    uint64x2_t chng01 = vcgtq_s64( rdBest01, rdCostA01 );
    uint64x2_t chng23 = vcgtq_s64( rdBest23, rdCostZ23 );
    uint32x4_t chng   = vcombine_u32( vmovn_u64(chng01), vmovn_u64(chng23) );

    rdBest01 = vbslq_s64( chng01, rdCostA01, rdBest01 );
    rdBest23 = vbslq_s64( chng23, rdCostZ23, rdBest23 );
    valBest  = vbslq_s32( chng,   valCand,   valBest  );
    idxBest  = vbslq_s32( chng,   idxCand,   idxBest  );

    vst1q_s64( &decisions.rdCost[0], rdBest01 );
    vst1q_s64( &decisions.rdCost[2], rdBest23 );
    vst1_s16( (int16_t*)decisions.absLevel, vqmovn_s32(valBest) );
    int8x8_t idx8 = vmovn_s16( vcombine_s16( vmovn_s32(idxBest), vdup_n_s16(0) ) );
    vst1_lane_u32( (uint32_t*)decisions.prevId, vreinterpret_u32_s8(idx8), 0 );
  }

  static inline void updateStates( const ScanInfo& scanInfo, const Decisions& decisions,
                                   StateMem& curr )
  {
    memcpy( curr.rdCost, decisions.rdCost, sizeof(decisions.rdCost) );

    int8_t s[4], l[4], t[4];
    memcpy( s, decisions.prevId, 4 );

    int16x4_t al4  = vld1_s16( (const int16_t*)decisions.absLevel );
    int16x4_t a1_4 = vand_s16( al4, vdup_n_s16(1) );
    int8x8_t lv8   = vqmovn_s16( vcombine_s16(
        vmin_s16( al4, vadd_s16(vdup_n_s16(126), a1_4) ), vdup_n_s16(0) ) );
    int8x8_t tv8   = vqmovn_s16( vcombine_s16(
        vmin_s16( al4, vadd_s16(vdup_n_s16(  4), a1_4) ), vdup_n_s16(0) ) );

    // OR bit 5 (32) into tv8 where t > 0: encodes numSig-increment alongside sumAbs1
    tv8 = vorr_s8( tv8, vand_s8(vdup_n_s8(32),
                                vcgt_s8(tv8, vdup_n_s8(0))) );

    // Zero l and t where prevId < 0 (inactive / start state)
    uint8x8_t active = vcge_s8(
        vreinterpret_s8_u8( vreinterpret_u8_s32(
            vld1_lane_s32((int32_t*)s, vdup_n_s32(0), 0) ) ), vdup_n_s8(0) );
    lv8 = vreinterpret_s8_u8( vand_u8(vreinterpret_u8_s8(lv8), active) );
    tv8 = vreinterpret_s8_u8( vand_u8(vreinterpret_u8_s8(tv8), active) );
    vst1_lane_s32( (int32_t*)l, vreinterpret_s32_s8(lv8), 0 );
    vst1_lane_s32( (int32_t*)t, vreinterpret_s32_s8(tv8), 0 );

    // Layout: [s0,s1,s2,s3, s0,s1,s2,s3, s0,s1,s2,s3, s0,s1,s2,s3] + group offsets
    // Inactive (s[k] < 0) → 0xFF so vqtbl1q_u8 returns 0
    uint32_t su32;
    memcpy( &su32, s, 4 );
    uint8x16_t vshuf    = vreinterpretq_u8_u32( vdupq_n_u32(su32) );
    uint8x16_t neg_mask = vcltq_s8( vreinterpretq_s8_u8(vshuf), vdupq_n_s8(0) );
    static const uint8_t off_data[16] = { 0,0,0,0, 4,4,4,4, 8,8,8,8, 12,12,12,12 };
    vshuf = vaddq_u8( vshuf, vld1q_u8(off_data) );
    vshuf = vbslq_u8( neg_mask, vdupq_n_u8(0xFF), vshuf );

    // shuffle tplAcc/absVal/sum1st
    for( int i = 0; i < 64; i += 16 )
    {
      vst1q_u8( &curr.tplAcc[i], vqtbl1q_u8(vld1q_u8(&curr.tplAcc[i]), vshuf) );
      vst1q_u8( &curr.absVal[i], vqtbl1q_u8(vld1q_u8(&curr.absVal[i]), vshuf) );
      vst1q_u8( &curr.sum1st[i], vqtbl1q_u8(vld1q_u8(&curr.sum1st[i]), vshuf) );
    }

    // numSig: shuffle then increment where l > 0 (saturating at 127)
    // numSig -= cmpgt(l, 0)  [cmpgt gives -1, so -= -1 == += 1]
    {
      uint8x16_t nsV = vreinterpretq_u8_u32(
          vld1q_lane_u32((uint32_t*)curr.numSig, vdupq_n_u32(0), 0) );
      nsV = vqtbl1q_u8( nsV, vshuf );
      uint8x16_t lvV = vreinterpretq_u8_u32(
          vld1q_lane_u32((uint32_t*)l, vdupq_n_u32(0), 0) );
      nsV = vreinterpretq_u8_s8( vqsubq_s8(
          vreinterpretq_s8_u8(nsV),
          vreinterpretq_s8_u8( vcgtq_s8(vreinterpretq_s8_u8(lvV), vdupq_n_s8(0)) ) ) );
      vst1q_lane_u32( (uint32_t*)curr.numSig, vreinterpretq_u32_u8(nsV), 0 );
    }

    // refSbbCtxId: shuffle then set to -1 where inactive
    {
      uint8x16_t rscV = vreinterpretq_u8_u32(
          vld1q_lane_u32((uint32_t*)curr.refSbbCtxId, vdupq_n_u32(0), 0) );
      rscV = vqtbl1q_u8( rscV, vshuf );
      rscV = vbslq_u8( neg_mask, vdupq_n_u8(0xFF), rscV );
      vst1q_lane_u32( (uint32_t*)curr.refSbbCtxId, vreinterpretq_u32_u8(rscV), 0 );
    }

    {
      static const int8_t mlutb[4] = { 0, 1, 3, 0 };
      // snapshot before loop: s[2] in {0,1} would alias k=0/1 writes (x86 uses atomic SSE shuffle)
      int16_t origRrb[4];
      memcpy( origRrb, curr.remRegBins, 8 );
      bool anyLt4 = false;
      for( int k = 0; k < 4; k++ )
      {
        int16_t rrb = s[k] >= 0 ? ( origRrb[s[k]] - 1 )
                                 : (int16_t)curr.initRemRegBins;
        rrb -= mlutb[std::min<int>( l[k], 2 )];
        curr.remRegBins[k] = rrb;
        if( rrb < 4 ) anyLt4 = true;
      }
      curr.anyRemRegBinsLt4 = anyLt4;
    }

    // update sum1st and tplAcc for neighbors
    {
      uint8x8_t lvl4 = vreinterpret_u8_s32( vld1_lane_s32((int32_t*)l, vdup_n_s32(0), 0) );
      uint8x8_t tpl4 = vreinterpret_u8_s32( vld1_lane_s32((int32_t*)t, vdup_n_s32(0), 0) );

#define UPDATE_DEP(k) { \
      int addr = scanInfo.currNbInfoSbb.invInPos[k] << 2; \
      uint8x8_t ms = vreinterpret_u8_s32( vld1_lane_s32((int32_t*)&curr.sum1st[addr], vdup_n_s32(0), 0) ); \
      vst1_lane_s32( (int32_t*)&curr.sum1st[addr], vreinterpret_s32_u8(vqadd_u8(ms, lvl4)), 0 ); \
      uint8x8_t tp = vreinterpret_u8_s32( vld1_lane_s32((int32_t*)&curr.tplAcc[addr], vdup_n_s32(0), 0) ); \
      vst1_lane_s32( (int32_t*)&curr.tplAcc[addr], vreinterpret_s32_u8(vadd_u8(tp, tpl4)), 0 ); }
      switch( scanInfo.currNbInfoSbb.numInv )
      {
      default:
      case 5: UPDATE_DEP(4);
      case 4: UPDATE_DEP(3);
      case 3: UPDATE_DEP(2);
      case 2: UPDATE_DEP(1);
      case 1: UPDATE_DEP(0);
      case 0: ;
      }
#undef UPDATE_DEP
    }

    vst1_lane_s32( (int32_t*)&curr.absVal[scanInfo.insidePos << 2],
                   vreinterpret_s32_s8(lv8), 0 );

    // ctx for next position
    {
      int base = scanInfo.nextInsidePos << 2;
      uint8x8_t tplA = vreinterpret_u8_s32(
          vld1_lane_s32((int32_t*)&curr.tplAcc[base], vdup_n_s32(0), 0) );
      uint8x8_t sAbs1 = vand_u8( tplA, vdup_n_u8(31) );
      uint8x8_t sNum  = vand_u8( vshr_n_u8(tplA, 5), vdup_n_u8(7) );
      int8x8_t  sGt1  = vmin_s8(
          vsub_s8(vreinterpret_s8_u8(sAbs1), vreinterpret_s8_u8(sNum)), vdup_n_s8(4) );
      vst1_lane_s32( (int32_t*)curr.ctx.cff,
          vreinterpret_s32_s8( vadd_s8(sGt1, vdup_n_s8(scanInfo.gtxCtxOffsetNext)) ), 0 );
      uint8x8_t sSig = vmin_u8( vshr_n_u8(vadd_u8(sAbs1, vdup_n_u8(1)), 1), vdup_n_u8(3) );
      vst1_lane_s32( (int32_t*)curr.ctx.sig,
          vreinterpret_s32_u8( vadd_u8(sSig, vdup_n_u8(scanInfo.sigCtxOffsetNext)) ), 0 );
      curr.cffBitsCtxOffset = scanInfo.gtxCtxOffsetNext;
    }
  }

  static inline void updateStatesEOS( const ScanInfo& scanInfo, const Decisions& decisions,
                                      const StateMem& skip, StateMem& curr,
                                      CommonCtxNeon& commonCtx )
  {
    int8_t s[4], l[4], z[4];
    for( int i = 0; i < 4; i++ )
    {
      s[i]           = decisions.prevId[i] >= 4 ? -2 : decisions.prevId[i];
      l[i]           = s[i] > -2 ? (int8_t)std::min<int>( decisions.absLevel[i],
                                     126 + ( decisions.absLevel[i] & 1 ) ) : 0;
      z[i]           = 3 - decisions.prevId[i];
      curr.rdCost[i] = decisions.rdCost[i];
    }

    uint32_t su32;
    memcpy( &su32, s, 4 );
    uint8x16_t vshuf    = vreinterpretq_u8_u32( vdupq_n_u32(su32) );
    uint8x16_t neg_mask = vcltq_s8( vreinterpretq_s8_u8(vshuf), vdupq_n_s8(0) );
    static const uint8_t off_data[16] = { 0,0,0,0, 4,4,4,4, 8,8,8,8, 12,12,12,12 };
    vshuf = vaddq_u8( vshuf, vld1q_u8(off_data) );
    vshuf = vbslq_u8( neg_mask, vdupq_n_u8(0xFF), vshuf );

    // shuffle absVal only (tplAcc/sum1st are reset below)
    for( int i = 0; i < 64; i += 16 )
      vst1q_u8( &curr.absVal[i], vqtbl1q_u8(vld1q_u8(&curr.absVal[i]), vshuf) );

    {
      uint8x16_t nsV = vreinterpretq_u8_u32(
          vld1q_lane_u32((uint32_t*)curr.numSig, vdupq_n_u32(0), 0) );
      nsV = vqtbl1q_u8( nsV, vshuf );

      memcpy( &curr.absVal[scanInfo.insidePos << 2], l, 4 );

      uint8x16_t lvV = vreinterpretq_u8_u32(
          vld1q_lane_u32((uint32_t*)l, vdupq_n_u32(0), 0) );
      nsV = vreinterpretq_u8_s8( vqsubq_s8(
          vreinterpretq_s8_u8(nsV),
          vreinterpretq_s8_u8( vcgtq_s8(vreinterpretq_s8_u8(lvV), vdupq_n_s8(0)) ) ) );
      vst1q_lane_u32( (uint32_t*)curr.numSig, vreinterpretq_u32_u8(nsV), 0 );
    }

    {
      uint8x16_t rscV = vreinterpretq_u8_u32(
          vld1q_lane_u32((uint32_t*)curr.refSbbCtxId, vdupq_n_u32(0), 0) );
      rscV = vqtbl1q_u8( rscV, vshuf );
      rscV = vbslq_u8( neg_mask, vdupq_n_u8(0xFF), rscV );
      vst1q_lane_u32( (uint32_t*)curr.refSbbCtxId, vreinterpretq_u32_u8(rscV), 0 );
    }

    {
      static const int8_t mlutb[4] = { 0, 1, 3, 0 };
      // snapshot before loop: same aliasing hazard as updateStates
      int16_t origRrb[4];
      memcpy( origRrb, curr.remRegBins, 8 );
      bool anyLt4 = false;
      for( int k = 0; k < 4; k++ )
      {
        int16_t rrb;
        if( s[k] == -2 )       rrb = (int16_t)curr.initRemRegBins;
        else if( s[k] >= 0 )   rrb = origRrb[s[k]] - 1;
        else                   rrb = (int16_t)curr.initRemRegBins;
        if( z[k] < 0 ) rrb = skip.remRegBins[k];  // prevId>=4 means from skip state
        rrb -= mlutb[std::min<int>( l[k], 2 )];
        curr.remRegBins[k] = rrb;
        if( rrb < 4 ) anyLt4 = true;
      }
      curr.anyRemRegBinsLt4 = anyLt4;
    }

    commonCtx.updateAllLvls( scanInfo, curr );

    memset( curr.absVal, 0, sizeof(curr.absVal) );
    memset( curr.tplAcc, 0, sizeof(curr.tplAcc) );
    memset( curr.sum1st, 0, sizeof(curr.sum1st) );

    for( int i = 0; i < 4; i++ )
    {
      int prevId = decisions.prevId[i];
      if( prevId > -2 )
      {
        const int refId = prevId < 0 ? -1
                        : ( prevId < 4 ? curr.refSbbCtxId[i] : prevId - 4 );
        commonCtx.update( scanInfo, refId, i, curr );
      }
    }

    memset( curr.numSig, 0, sizeof(curr.numSig) );

    // ctx for next position
    {
      int base = scanInfo.nextInsidePos << 2;
      uint8x8_t tplA = vreinterpret_u8_s32(
          vld1_lane_s32((int32_t*)&curr.tplAcc[base], vdup_n_s32(0), 0) );
      uint8x8_t sAbs1 = vand_u8( tplA, vdup_n_u8(31) );
      uint8x8_t sNum  = vand_u8( vshr_n_u8(tplA, 5), vdup_n_u8(7) );
      int8x8_t  sGt1  = vmin_s8(
          vsub_s8(vreinterpret_s8_u8(sAbs1), vreinterpret_s8_u8(sNum)), vdup_n_s8(4) );
      vst1_lane_s32( (int32_t*)curr.ctx.cff,
          vreinterpret_s32_s8( vadd_s8(sGt1, vdup_n_s8(scanInfo.gtxCtxOffsetNext)) ), 0 );
      uint8x8_t sSig = vmin_u8( vshr_n_u8(vadd_u8(sAbs1, vdup_n_u8(1)), 1), vdup_n_u8(3) );
      vst1_lane_s32( (int32_t*)curr.ctx.sig,
          vreinterpret_s32_u8( vadd_u8(sSig, vdup_n_u8(scanInfo.sigCtxOffsetNext)) ), 0 );
      curr.cffBitsCtxOffset = scanInfo.gtxCtxOffsetNext;
    }
  }
};

// ---------------------------------------------------------------------------
// DepQuantNeon: DepQuant using NEON
// ---------------------------------------------------------------------------
class DepQuantNeon : private RateEstimator, public DepQuantImpl
{
public:
  const Decisions startDec[2] =
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

  DepQuantNeon() : RateEstimator(), m_commonCtx()
  {
    m_scansRom.init();
    for( int t = 0; t < ( MAX_TB_SIZEY * MAX_TB_SIZEY ); t++ )
      memcpy( m_trellis[t], startDec, sizeof(startDec) );
  }

  ~DepQuantNeon() {}

  void quant( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID,
              const QpParam& cQP, const double lambda, const Ctx& ctx,
              TCoeff& absSum, bool enableScalingLists, int* quantCoeff ) override
  {
    const TUParameters& tuPars = *m_scansRom.getTUPars( tu.blocks[compID], compID );
    m_quant.initQuantBlock( tu, compID, cQP, lambda );
    TCoeffSig*    qCoeff   = tu.getCoeffs( compID ).buf;
    const TCoeff* tCoeff   = srcCoeff.buf;
    const int     numCoeff = tu.blocks[compID].area();
    ::memset( qCoeff, 0x00, numCoeff * sizeof(TCoeffSig) );
    absSum = 0;

    const CompArea& area      = tu.blocks[compID];
    const uint32_t  width     = area.width;
    const uint32_t  height    = area.height;
    const uint32_t  lfnstIdx  = tu.cu->lfnstIdx;

    bool zeroOut         = false;
    bool zeroOutforThres = false;
    int  effWidth        = tuPars.m_width;
    int  effHeight       = tuPars.m_height;

    if( ( tu.mtsIdx[compID] > MTS_SKIP ||
          ( tu.cs->sps->MTS && tu.cu->sbtInfo != 0 &&
            tuPars.m_height <= 32 && tuPars.m_width <= 32 ) ) &&
        compID == COMP_Y )
    {
      effHeight = ( tuPars.m_height == 32 ) ? 16 : tuPars.m_height;
      effWidth  = ( tuPars.m_width  == 32 ) ? 16 : tuPars.m_width;
      zeroOut   = ( effHeight < (int)tuPars.m_height || effWidth < (int)tuPars.m_width );
    }
    zeroOutforThres = zeroOut || ( 32 < (int)tuPars.m_height || 32 < (int)tuPars.m_width );

    int firstTestPos =
      std::min<int>( tuPars.m_width, JVET_C0024_ZERO_OUT_TH ) *
      std::min<int>( tuPars.m_height, JVET_C0024_ZERO_OUT_TH ) - 1;
    if( lfnstIdx > 0 && tu.mtsIdx[compID] != MTS_SKIP && width >= 4 && height >= 4 )
      firstTestPos = ( ( width == 4 && height == 4 ) || ( width == 8 && height == 8 ) ) ? 7 : 15;

    const TCoeff defaultQuantisationCoefficient = (TCoeff)m_quant.getQScale();
    const TCoeff thres          = m_quant.getLastThreshold();
    const int    zeroOutWidth   = ( tuPars.m_width  == 32 && zeroOut ) ? 16 : 32;
    const int    zeroOutHeight  = ( tuPars.m_height == 32 && zeroOut ) ? 16 : 32;

    if( enableScalingLists )
    {
      for( ; firstTestPos >= 0; firstTestPos-- )
      {
        if( zeroOutforThres &&
            ( tuPars.m_scanId2BlkPos[firstTestPos].x >= (unsigned)zeroOutWidth ||
              tuPars.m_scanId2BlkPos[firstTestPos].y >= (unsigned)zeroOutHeight ) )
          continue;
        const TCoeff thresTmp =
          TCoeff( thres / ( 4 * quantCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) );
        if( abs( tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) > thresTmp ) break;
      }
    }
    else
    {
      const TCoeff defaultTh = TCoeff( thres / ( defaultQuantisationCoefficient << 2 ) );
      for( ; firstTestPos >= 0; firstTestPos-- )
      {
        if( zeroOutforThres &&
            ( tuPars.m_scanId2BlkPos[firstTestPos].x >= (unsigned)zeroOutWidth ||
              tuPars.m_scanId2BlkPos[firstTestPos].y >= (unsigned)zeroOutHeight ) )
          continue;
        if( abs( tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) > defaultTh ) break;
      }
    }

    if( firstTestPos < 0 )
    {
      tu.lastPos[compID] = -1;
      return;
    }

    RateEstimator::initCtx( tuPars, tu, compID, ctx.getFracBitsAcess() );
    m_commonCtx.reset( tuPars, *this );
    for( int k = 0; k < 4; k++ )
    {
      StateNeon::init( k, m_state_curr );
      StateNeon::init( k, m_state_skip );
      m_state_curr.m_sigFracBitsArray[k] = RateEstimator::sigFlagBits( k );
    }
    m_state_curr.m_gtxFracBitsArray = RateEstimator::gtxFracBits();
    memset( m_state_curr.sum1st, 0, sizeof(m_state_curr.sum1st) );
    memset( m_state_curr.tplAcc, 0, sizeof(m_state_curr.tplAcc) );
    memset( m_state_curr.absVal, 0, sizeof(m_state_curr.absVal) );

    const int              numCtx  = isLuma( compID ) ? 21 : 11;
    const CoeffFracBits* const cffBits = gtxFracBits();
    // cffBits1: kept for StateMem layout compatibility; not used in NEON hot paths
    for( int i = 0; i < numCtx; i++ )
      m_state_curr.cffBits1[i] = cffBits[i].bits[1];

    int effectWidth  = std::min( 32, effWidth  );
    int effectHeight = std::min( 32, effHeight );
    m_state_curr.initRemRegBins =
      ( effectWidth * effectHeight * MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT ) / 16;
    m_state_curr.anyRemRegBinsLt4 = true;

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

    // Best path selection
    int     prevId      = -1;
    int64_t minPathCost = 0;
    for( int8_t stateId = 0; stateId < 4; stateId++ )
    {
      int64_t pathCost = m_trellis[0][0].rdCost[stateId];
      if( pathCost < minPathCost )
      {
        prevId      = stateId;
        minPathCost = pathCost;
      }
    }

    int scanIdx = 0;
    for( ; prevId >= 0; scanIdx++ )
    {
      TCoeffSig absLevel = m_trellis[scanIdx][prevId >> 2].absLevel[prevId & 3];
      int32_t   blkpos   = tuPars.m_scanId2BlkPos[scanIdx].idx;
      qCoeff[blkpos]     = TCoeffSig( tCoeff[blkpos] < 0 ? -absLevel : absLevel );
      absSum            += absLevel;
      prevId             = m_trellis[scanIdx][prevId >> 2].prevId[prevId & 3];
    }
    tu.lastPos[compID] = scanIdx - 1;
  }

private:

  void xDecideAndUpdate( const TCoeff absCoeff, const ScanInfo& scanInfo,
                         bool zeroOut, int quantCoeff )
  {
    Decisions* decisions = &m_trellis[scanInfo.scanIdx][0];
    xDecide( scanInfo, absCoeff, lastOffset( scanInfo.scanIdx ), *decisions,
             zeroOut, quantCoeff );

    if( scanInfo.scanIdx )
    {
      if( scanInfo.spt == SCAN_SOCSBB )
        memcpy( &m_state_skip, &m_state_curr, StateMemSkipCpySize );

      if( scanInfo.insidePos == 0 )
      {
        m_commonCtx.swap();
        StateNeon::updateStatesEOS( scanInfo, *decisions, m_state_skip,
                                    m_state_curr, m_commonCtx );
        ::memcpy( decisions + 1, decisions, sizeof(Decisions) );
      }
      else if( !zeroOut )
      {
        StateNeon::updateStates( scanInfo, *decisions, m_state_curr );
      }
    }
  }

  void xDecide( const ScanInfo& scanInfo, const TCoeff absCoeff, const int lastOffset,
                Decisions& decisions, bool zeroOut, int quantCoeff )
  {
    ::memcpy( &decisions, startDec, sizeof(Decisions) );
    StateMem& skip = m_state_skip;

    if( zeroOut )
    {
      if( scanInfo.spt == SCAN_EOCSBB )
      {
        StateNeon::checkRdCostSkipSbbZeroOut( 0, decisions, 0, skip );
        StateNeon::checkRdCostSkipSbbZeroOut( 1, decisions, 1, skip );
        StateNeon::checkRdCostSkipSbbZeroOut( 2, decisions, 2, skip );
        StateNeon::checkRdCostSkipSbbZeroOut( 3, decisions, 3, skip );
      }
      return;
    }

    StateMem& prev = m_state_curr;

    int64_t scaledOrg = int64_t(absCoeff) * quantCoeff;
    TCoeff  qIdx      = TCoeff( ( scaledOrg + m_quant.m_QAdd ) >> m_quant.m_QShift );

    if( qIdx < 0 )
    {
      int64_t scaledAdd  = m_quant.m_DistStepAdd - scaledOrg * m_quant.m_DistOrgFact;
      int64_t pq_a_dist  = ( ( scaledAdd + 0 * m_quant.m_DistStepAdd ) * 1 +
                              m_quant.m_DistAdd ) >> m_quant.m_DistShift;
      int64_t pq_b_dist  = ( ( scaledAdd + 1 * m_quant.m_DistStepAdd ) * 2 +
                              m_quant.m_DistAdd ) >> m_quant.m_DistShift;

      if( prev.anyRemRegBinsLt4 )
      {
        StateNeon::setRiceParam( 0, scanInfo, prev, false );
        StateNeon::checkRdCostsOdd1( 0, scanInfo.spt, pq_b_dist, decisions, 2, 0, prev );
        StateNeon::setRiceParam( 1, scanInfo, prev, false );
        StateNeon::checkRdCostsOdd1( 1, scanInfo.spt, pq_b_dist, decisions, 0, 2, prev );
        StateNeon::setRiceParam( 2, scanInfo, prev, false );
        StateNeon::checkRdCostsOdd1( 2, scanInfo.spt, pq_a_dist, decisions, 3, 1, prev );
        StateNeon::setRiceParam( 3, scanInfo, prev, false );
        StateNeon::checkRdCostsOdd1( 3, scanInfo.spt, pq_a_dist, decisions, 1, 3, prev );
      }
      else
      {
        StateNeon::checkAllRdCostsOdd1( scanInfo.spt, pq_a_dist, pq_b_dist,
                                        decisions, prev );
      }
      StateNeon::checkRdCostStart( lastOffset, PQData{ 1, pq_b_dist }, decisions, 2, prev );
    }
    else
    {
      qIdx             = std::max<TCoeff>( 1, std::min<TCoeff>( m_quant.m_maxQIdx, qIdx ) );
      int64_t scaledAdd = qIdx * m_quant.m_DistStepAdd - scaledOrg * m_quant.m_DistOrgFact;

      PQData pqData[4];
      PQData& pq_a = pqData[( qIdx + 0 ) & 3];
      PQData& pq_b = pqData[( qIdx + 1 ) & 3];
      PQData& pq_c = pqData[( qIdx + 2 ) & 3];
      PQData& pq_d = pqData[( qIdx + 3 ) & 3];

      pq_a.deltaDist = ( ( scaledAdd + 0 * m_quant.m_DistStepAdd ) * ( qIdx + 0 ) +
                          m_quant.m_DistAdd ) >> m_quant.m_DistShift;
      pq_a.absLevel  = ( qIdx + 1 ) >> 1;
      pq_b.deltaDist = ( ( scaledAdd + 1 * m_quant.m_DistStepAdd ) * ( qIdx + 1 ) +
                          m_quant.m_DistAdd ) >> m_quant.m_DistShift;
      pq_b.absLevel  = ( qIdx + 2 ) >> 1;
      pq_c.deltaDist = ( ( scaledAdd + 2 * m_quant.m_DistStepAdd ) * ( qIdx + 2 ) +
                          m_quant.m_DistAdd ) >> m_quant.m_DistShift;
      pq_c.absLevel  = ( qIdx + 3 ) >> 1;
      pq_d.deltaDist = ( ( scaledAdd + 3 * m_quant.m_DistStepAdd ) * ( qIdx + 3 ) +
                          m_quant.m_DistAdd ) >> m_quant.m_DistShift;
      pq_d.absLevel  = ( qIdx + 4 ) >> 1;

      bool cff02ge4 = pqData[0].absLevel >= 4;
      bool cff13ge4 = pqData[3].absLevel >= 4;

      if( cff02ge4 || cff13ge4 || prev.anyRemRegBinsLt4 )
      {
        if( prev.anyRemRegBinsLt4 || cff02ge4 )
        {
          StateNeon::setRiceParam( 0, scanInfo, prev, cff02ge4 );
          StateNeon::setRiceParam( 1, scanInfo, prev, cff02ge4 );
        }
        if( prev.anyRemRegBinsLt4 || cff13ge4 )
        {
          StateNeon::setRiceParam( 2, scanInfo, prev, cff13ge4 );
          StateNeon::setRiceParam( 3, scanInfo, prev, cff13ge4 );
        }
        StateNeon::checkRdCosts( 0, scanInfo.spt, pqData[0], pqData[2], decisions, 0, 2, prev );
        StateNeon::checkRdCosts( 1, scanInfo.spt, pqData[0], pqData[2], decisions, 2, 0, prev );
        StateNeon::checkRdCosts( 2, scanInfo.spt, pqData[3], pqData[1], decisions, 1, 3, prev );
        StateNeon::checkRdCosts( 3, scanInfo.spt, pqData[3], pqData[1], decisions, 3, 1, prev );
      }
      else
      {
        StateNeon::checkAllRdCosts( scanInfo.spt, pqData, decisions, prev );
      }

      StateNeon::checkRdCostStart( lastOffset, pqData[0], decisions, 0, prev );
      StateNeon::checkRdCostStart( lastOffset, pqData[2], decisions, 2, prev );
    }

    if( scanInfo.spt == SCAN_EOCSBB )
    {
      StateNeon::checkRdCostSkipSbb( 0, decisions, 0, skip );
      StateNeon::checkRdCostSkipSbb( 1, decisions, 1, skip );
      StateNeon::checkRdCostSkipSbb( 2, decisions, 2, skip );
      StateNeon::checkRdCostSkipSbb( 3, decisions, 3, skip );
    }
  }

  CommonCtxNeon m_commonCtx;
  Decisions     m_trellis[MAX_TB_SIZEY * MAX_TB_SIZEY][2];
  Rom           m_scansRom;
  StateMem      m_state_curr;
  StateMem      m_state_skip;
};

} // namespace DQIntern

// Registration
template<ARM_VEXT vext>
void DepQuant::_initDepQuantARM()
{
  p = new DQIntern::DepQuantNeon();
}

template void DepQuant::_initDepQuantARM<NEON>();

} // namespace vvenc

#endif // TARGET_SIMD_ARM && ENABLE_SIMD_OPT_QUANT

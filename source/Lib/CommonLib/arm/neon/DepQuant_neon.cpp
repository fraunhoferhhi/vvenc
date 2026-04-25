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
 * \file DepQuant_neon.cpp
 * \brief Neon implementation of DepQuant for AArch64.
 */
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include <arm_neon.h>

#include "CommonDefARM.h"
#include "CommonLib/DepQuant.h"
#include "CommonLib/arm/mem_neon.h"
#include "CommonLib/arm/neon/permute_neon.h"

#include <string.h>

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_QUANT
namespace vvenc
{
namespace DQInternSimd
{

using namespace DQIntern;

static inline void buildStateShuffle_neon( const int8x8_t prevState, uint8x8_t& hasPrevMask,
                                           uint8x16_t& stateShuffle16 )
{
  hasPrevMask = vcge_s8( prevState, vdup_n_s8( 0 ) );

  // If prevState is [-2, -1], return 0xF0 such that it stays out of bounds even after adding stateOffsetsTbl.
  const uint8x8_t stateShuffleBase = vbsl_u8( hasPrevMask, vreinterpret_u8_s8( prevState ), vdup_n_u8( 0xF0 ) );
  const uint8x16_t stateShuffleDupQ =
      vreinterpretq_u8_u32( vdupq_lane_u32( vreinterpret_u32_u8( stateShuffleBase ), 0 ) );
  static constexpr uint8_t stateOffsetsTbl[16] = { 0, 0, 0, 0, 4, 4, 4, 4, 8, 8, 8, 8, 12, 12, 12, 12 };
  stateShuffle16 = vaddq_u8( stateShuffleDupQ, vld1q_u8( stateOffsetsTbl ) );
}

static inline void shuffleStateBuf_neon( uint8_t* buf, const uint8x16_t stateShuffle16 )
{
  const uint8x16x2_t vsrc = load_u8x16x2( buf );

  uint8x16x2_t vdst;
  vdst.val[0] = vvenc_vqtbl1q_u8( vsrc.val[0], stateShuffle16 );
  vdst.val[1] = vvenc_vqtbl1q_u8( vsrc.val[1], stateShuffle16 );

  store_u8x16x2( buf, vdst );
}

static inline uint8x8_t capWithParity_u8x8_neon( const uint8x8_t value, int capEven )
{
  CHECKD( capEven % 2 == 0, "Cap value must be even" );
  uint8x8_t cappedValue = vmin_u8( value, vdup_n_u8( uint8_t( capEven ) ) );
  cappedValue = vbsl_u8( vdup_n_u8( 1 ), value, cappedValue ); // Preserve parity bit.
  return cappedValue;
}

static inline int16x4_t capWithParity_s16x4_neon( const int16x4_t value, int capEven )
{
  CHECKD( capEven % 2 == 0, "Cap value must be even" );
  int16x4_t cappedValue = vmin_s16( value, vdup_n_s16( int16_t( capEven ) ) );
  cappedValue = vbsl_s16( vdup_n_u16( 1 ), value, cappedValue ); // Preserve parity bit.
  return cappedValue;
}

static inline uint8x8_t computeAbsVal_neon( const ScanInfo& scanInfo, const Decisions& decisions,
                                            const int8x8_t prevState, StateMem& curr )
{
  const int16x4_t absLevel = vld1_s16( decisions.absLevel );
  const int16x4_t absLevelCapped = capWithParity_s16x4_neon( absLevel, 126 );
  const int8x8_t absLevelCappedByte = vmovn_s16( vcombine_s16( absLevelCapped, vdup_n_s16( 0 ) ) );
  const uint8x8_t absValMask = vcgt_s8( prevState, vdup_n_s8( -2 ) );
  const uint8x8_t absVal = vand_u8( absValMask, vreinterpret_u8_s8( absLevelCappedByte ) );

  store_u8x4( curr.absVal[scanInfo.insidePos], absVal );

  return absVal;
}

static inline void updateNumSig_neon( const uint8x8_t stateShuffle4, const uint8x8_t absVal, StateMem& curr )
{
  int8x8_t numSig = vreinterpret_s8_u8( vtbl1_u8( load_u8x4( curr.numSig ), stateShuffle4 ) );
  numSig = vqsub_s8( numSig, vreinterpret_s8_u8( vcgt_u8( absVal, vdup_n_u8( 0 ) ) ) );
  store_u8x4( curr.numSig, vreinterpret_u8_s8( numSig ) );
}

static inline void updateRefSbbCtxId_neon( const uint8x8_t stateShuffle4, const uint8x8_t hasPrevMask, StateMem& curr )
{
  int8x8_t refSbbCtxId = vtbl1_s8( load_s8x4( curr.refSbbCtxId ), vreinterpret_s8_u8( stateShuffle4 ) );
  refSbbCtxId = vorn_s8( refSbbCtxId, vreinterpret_s8_u8( hasPrevMask ) );
  store_s8x4( curr.refSbbCtxId, refSbbCtxId );
}

template<bool SkipEOS>
static inline void updateRemRegBins_neon( const int8x8_t prevState, const uint8x8_t skipMask,
                                          const uint8x8_t hasPrevMask, const uint8x8_t absVal, const StateMem& skip,
                                          StateMem& curr )
{
  const int8x8_t prevState_x2 = vshl_n_s8( prevState, 1 );
  const int8x8_t rrbShuffle = vzip_s8( prevState_x2, vadd_s8( prevState_x2, vdup_n_s8( 1 ) ) ).val[0];
  const uint16x4_t hasPrevMaskHalf = vreinterpret_u16_u8( vzip_u8( hasPrevMask, hasPrevMask ).val[0] );
  int16x4_t remRegBins =
      vreinterpret_s16_s8( vtbl1_s8( vreinterpret_s8_s16( vld1_s16( curr.remRegBins ) ), rrbShuffle ) );
  remRegBins = vsub_s16( remRegBins, vdup_n_s16( 1 ) );
  remRegBins = vbsl_s16( hasPrevMaskHalf, remRegBins, vdup_n_s16( curr.initRemRegBins ) );

  if( SkipEOS )
  {
    const uint16x4_t rrbMask = vreinterpret_u16_u8( vzip_u8( skipMask, skipMask ).val[0] );
    const int16x4_t skipRegBins = vld1_s16( skip.remRegBins );
    remRegBins = vbsl_s16( rrbMask, skipRegBins, remRegBins );
  }

  static constexpr uint8_t subBinLutTbl[8] = { 0, 1, 3, 0, 0, 0, 0, 0 };
  const uint8x8_t subBinLut = vld1_u8( subBinLutTbl );
  const uint8x8_t subFac = vtbl1_u8( subBinLut, vmin_u8( absVal, vdup_n_u8( 2 ) ) );
  const uint16x4_t subFac16 = vget_low_u16( vmovl_u8( subFac ) );
  remRegBins = vsub_s16( remRegBins, vreinterpret_s16_u16( subFac16 ) );
  vst1_s16( curr.remRegBins, remRegBins );

  const uint16x4_t remRegMask = vclt_s16( remRegBins, vdup_n_s16( 4 ) );
  curr.anyRemRegBinsLt4 = vget_lane_u64( vreinterpret_u64_u16( remRegMask ), 0 ) != 0;
}

static inline void updateNextCtx_neon( const ScanInfo& scanInfo, StateMem& curr )
{
  // tplAcc: lower 5 bits are sumAbs1, upper 3 bits are sumNum.
  const uint8x8_t tplAcc = load_u8x4( curr.tplAcc[scanInfo.nextInsidePos] );

  const uint8x8_t sumAbs1 = vand_u8( tplAcc, vdup_n_u8( 31 ) );
  const uint8x8_t sumNum = vshr_n_u8( tplAcc, 5 );
  int8x8_t sumGt1 = vsub_s8( vreinterpret_s8_u8( sumAbs1 ), vreinterpret_s8_u8( sumNum ) );
  sumGt1 = vmin_s8( sumGt1, vdup_n_s8( 4 ) );
  const int8x8_t ctx_cff = vadd_s8( sumGt1, vdup_n_s8( scanInfo.gtxCtxOffsetNext ) );
  store_u8x4( curr.ctx.cff, vreinterpret_u8_s8( ctx_cff ) );

  const uint8x8_t sig = vmin_u8( vrhadd_u8( sumAbs1, vdup_n_u8( 0 ) ), vdup_n_u8( 3 ) );
  const uint8x8_t ctx_sig = vadd_u8( sig, vdup_n_u8( scanInfo.sigCtxOffsetNext ) );
  store_u8x4( curr.ctx.sig, ctx_sig );

  curr.cffBitsCtxOffset = scanInfo.gtxCtxOffsetNext;
}

void updateStates_neon( const ScanInfo& scanInfo, const Decisions& decisions, StateMem& curr )
{
  static_assert( sizeof( curr.rdCost ) == sizeof( decisions.rdCost ), "Non-matching array size" );
  ::memcpy( curr.rdCost, decisions.rdCost, sizeof( decisions.rdCost ) );

  static constexpr int regSize = 4;
  static constexpr int ctxSize = 16 * regSize;

  uint8x8_t hasPrevMask;
  uint8x16_t stateShuffle16;
  const int8x8_t prevState = load_s8x4( decisions.prevId );
  buildStateShuffle_neon( prevState, hasPrevMask, stateShuffle16 );
  uint8_t* tplAccBuf = &curr.tplAcc[0][0];
  uint8_t* absValBuf = &curr.absVal[0][0];
  uint8_t* sum1stBuf = &curr.sum1st[0][0];

  for( int i = 0; i < ctxSize; i += 32 )
  {
    shuffleStateBuf_neon( tplAccBuf + i, stateShuffle16 );
    shuffleStateBuf_neon( absValBuf + i, stateShuffle16 );
    shuffleStateBuf_neon( sum1stBuf + i, stateShuffle16 );
  }

  const uint8x8_t stateShuffle4 = vget_low_u8( stateShuffle16 );
  const uint8x8_t absVal = computeAbsVal_neon( scanInfo, decisions, prevState, curr );

  updateNumSig_neon( stateShuffle4, absVal, curr );

  updateRefSbbCtxId_neon( stateShuffle4, hasPrevMask, curr );

  updateRemRegBins_neon<false>( prevState, vdup_n_u8( 0 ), hasPrevMask, absVal, curr, curr );

  if( scanInfo.currNbInfoSbb.numInv )
  {
    const uint8x8_t absValCapped = capWithParity_u8x8_neon( absVal, 4 );
    const uint8x8_t tpl_add = vand_u8( vadd_u8( absValCapped, vdup_n_u8( 32 ) ), vcgt_u8( absVal, vdup_n_u8( 0 ) ) );

    CHECKD( scanInfo.currNbInfoSbb.numInv > 5, "numInv out of range" );
    unsigned k = scanInfo.currNbInfoSbb.numInv;
    do
    {
      const int addr = scanInfo.currNbInfoSbb.invInPos[k - 1];

      uint8x8_t vsum = load_u8x4( curr.sum1st[addr] );
      vsum = vqadd_u8( vsum, absVal );
      store_u8x4( curr.sum1st[addr], vsum );

      uint8x8_t vtpl = load_u8x4( curr.tplAcc[addr] );
      vtpl = vadd_u8( vtpl, tpl_add );
      store_u8x4( curr.tplAcc[addr], vtpl );
    } while( --k != 0 );
  }

  updateNextCtx_neon( scanInfo, curr );
}

static inline void updateAllLvls_neon( const ScanInfo& scanInfo, const uint8_t* absValBuf, CommonCtx& commonCtx )
{
  uint8_t* levels0;
  uint8_t* levels1;
  uint8_t* levels2;
  uint8_t* levels3;

  commonCtx.getLevelPtrs( scanInfo, levels0, levels1, levels2, levels3 );

  const int sbbSize = scanInfo.sbbSize;
  CHECKD( sbbSize != 4 && sbbSize != 16, "sbbSize must be 4 or 16 only" );

  if( sbbSize == 16 )
  {
    const uint8x16x4_t in = vld4q_u8( absValBuf );
    vst1q_u8( levels0, in.val[0] ); // [0 4 8 12 16 20 24 28 32 36 40 44 48 52 56 60]
    vst1q_u8( levels1, in.val[1] ); // [1 5 9 13 17 21 25 29 33 37 41 45 49 53 57 61]
    vst1q_u8( levels2, in.val[2] ); // [2 6 10 14 18 22 26 30 34 38 42 46 50 54 58 62]
    vst1q_u8( levels3, in.val[3] ); // [3 7 11 15 19 23 27 31 35 39 43 47 51 55 59 63]
  }
  else // sbbSize == 4
  {
    const uint8x16_t in = vld1q_u8( absValBuf );     // [0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15]
    const uint8x16_t t0 = vuzpq_u8( in, in ).val[0]; // [0 2 4 6 8 10 12 14 0 2 4 6 8 10 12 14]
    const uint8x16_t t1 = vuzpq_u8( in, in ).val[1]; // [1 3 5 7 9 11 13 15 1 3 5 7 9 11 13 15]
    const uint8x16_t d0 = vuzpq_u8( t0, t0 ).val[0]; // [0 4 8 12 0 4 8 12 0 4 8 12 0 4 8 12]
    const uint8x16_t d1 = vuzpq_u8( t1, t1 ).val[0]; // [1 5 9 13 1 5 9 13 1 5 9 13 1 5 9 13]
    const uint8x16_t d2 = vuzpq_u8( t0, t0 ).val[1]; // [2 6 10 14 2 6 10 14 2 6 10 14 2 6 10 14]
    const uint8x16_t d3 = vuzpq_u8( t1, t1 ).val[1]; // [3 7 11 15 3 7 11 15 3 7 11 15 3 7 11 15]

    store_u8x4( levels0, vget_low_u8( d0 ) ); // [0 4 8 12]
    store_u8x4( levels1, vget_low_u8( d1 ) ); // [1 5 9 13]
    store_u8x4( levels2, vget_low_u8( d2 ) ); // [2 6 10 14]
    store_u8x4( levels3, vget_low_u8( d3 ) ); // [3 7 11 15]
  }
}

void updateStatesEOS_neon( const ScanInfo& scanInfo, const Decisions& decisions, const StateMem& skip, StateMem& curr,
                           CommonCtx& commonCtx )
{
  static_assert( sizeof( curr.rdCost ) == sizeof( decisions.rdCost ), "Non-matching array size" );
  ::memcpy( curr.rdCost, decisions.rdCost, sizeof( decisions.rdCost ) );

  static constexpr int regSize = 4;
  static constexpr int ctxSize = 16 * regSize;

  // Set prevState to -2 if prevId >= 4.
  const int8x8_t prevId = load_s8x4( decisions.prevId );
  const uint8x8_t skipMask = vcge_s8( prevId, vdup_n_s8( 4 ) );
  const int8x8_t prevState = vbsl_s8( skipMask, vdup_n_s8( -2 ), prevId );
  uint8x8_t hasPrevMask;
  uint8x16_t stateShuffle16;
  buildStateShuffle_neon( prevState, hasPrevMask, stateShuffle16 );
  uint8_t* absValBuf = &curr.absVal[0][0];

  for( int i = 0; i < ctxSize; i += 32 )
  {
    shuffleStateBuf_neon( absValBuf + i, stateShuffle16 );
  }

  const uint8x8_t stateShuffle4 = vget_low_u8( stateShuffle16 );
  const uint8x8_t absVal = computeAbsVal_neon( scanInfo, decisions, prevState, curr );

  updateNumSig_neon( stateShuffle4, absVal, curr );

  updateRefSbbCtxId_neon( stateShuffle4, hasPrevMask, curr );

  updateRemRegBins_neon<true>( prevState, skipMask, hasPrevMask, absVal, skip, curr );

  updateAllLvls_neon( scanInfo, absValBuf, commonCtx );

  memset( curr.absVal, 0, sizeof( curr.absVal ) );
  memset( curr.tplAcc, 0, sizeof( curr.tplAcc ) );
  memset( curr.sum1st, 0, sizeof( curr.sum1st ) );

  for( int i = 0; i < regSize; i++ )
  {
    const int prevId = decisions.prevId[i];

    if( prevId > -2 )
    {
      const int refId = prevId < 0 ? -1 : prevId < 4 ? curr.refSbbCtxId[i] : prevId - 4;
      commonCtx.update( scanInfo, refId, i, curr );
    }
  }

  memset( curr.numSig, 0, sizeof( curr.numSig ) );

  updateNextCtx_neon( scanInfo, curr );
}

static inline uint64x2_t vvenc_vcgtq_s64( int64x2_t a, int64x2_t b )
{
#if REAL_TARGET_AARCH64
  return vcgtq_s64( a, b );
#else
  return vreinterpretq_u64_s64( vshrq_n_s64( vqsubq_s64( b, a ), 63 ) );
#endif
}

void checkAllRdCostsOdd1_neon( const ScanPosType spt, const int64_t pq_a_dist, const int64_t pq_b_dist,
                               Decisions& decisions, const StateMem& state )
{
  int64x2_t rdCostZ01 = vld1q_s64( &state.rdCost[0] );
  int64x2_t rdCostZ23 = vld1q_s64( &state.rdCost[2] );

  const int64x2_t delta01 = vdupq_n_s64( pq_b_dist );
  const int64x2_t delta23 = vdupq_n_s64( pq_a_dist );
  int64x2_t rdCostA01 = vaddq_s64( rdCostZ01, delta01 );
  int64x2_t rdCostA23 = vaddq_s64( rdCostZ23, delta23 );

  // cffBits1[24], but ctx.cff[4] actual range is [0..20] only.
  int32x4_t cffBits = vsetq_lane_s32( state.cffBits1[state.ctx.cff[0]], vdupq_n_s32( 0 ), 0 );
  cffBits = vsetq_lane_s32( state.cffBits1[state.ctx.cff[1]], cffBits, 1 );
  cffBits = vsetq_lane_s32( state.cffBits1[state.ctx.cff[2]], cffBits, 2 );
  cffBits = vsetq_lane_s32( state.cffBits1[state.ctx.cff[3]], cffBits, 3 );

  rdCostA01 = vaddw_s32( rdCostA01, vget_low_s32( cffBits ) );
  rdCostA23 = vaddw_s32( rdCostA23, vget_high_s32( cffBits ) );

  CHECKD( state.ctx.sig[0] > RateEstimator::sm_maxNumSigCtx - 1, "ctx.sig[0] out of range" );
  CHECKD( state.ctx.sig[1] > RateEstimator::sm_maxNumSigCtx - 1, "ctx.sig[1] out of range" );
  CHECKD( state.ctx.sig[2] > RateEstimator::sm_maxNumSigCtx - 1, "ctx.sig[2] out of range" );
  CHECKD( state.ctx.sig[3] > RateEstimator::sm_maxNumSigCtx - 1, "ctx.sig[3] out of range" );

  // BinFracBits: uint32_t intBits[2].
  const BinFracBits sigBits0 = state.m_sigFracBitsArray[0][state.ctx.sig[0]];
  const BinFracBits sigBits1 = state.m_sigFracBitsArray[1][state.ctx.sig[1]];
  const BinFracBits sigBits2 = state.m_sigFracBitsArray[2][state.ctx.sig[2]];
  const BinFracBits sigBits3 = state.m_sigFracBitsArray[3][state.ctx.sig[3]];
  const uint32x4_t sigBits02 = vcombine_u32( vld1_u32( sigBits0.intBits ), vld1_u32( sigBits2.intBits ) );
  const uint32x4_t sigBits13 = vcombine_u32( vld1_u32( sigBits1.intBits ), vld1_u32( sigBits3.intBits ) );
  const int32x4_t sigBitsZ = vreinterpretq_s32_u32( vtrnq_u32( sigBits02, sigBits13 ).val[0] );
  const int32x4_t sigBitsA = vreinterpretq_s32_u32( vtrnq_u32( sigBits02, sigBits13 ).val[1] );

  const int64x2_t rdCostZ01_sigBits = vaddw_s32( rdCostZ01, vget_low_s32( sigBitsZ ) );
  const int64x2_t rdCostZ23_sigBits = vaddw_s32( rdCostZ23, vget_high_s32( sigBitsZ ) );
  const int64x2_t rdCostA01_sigBits = vaddw_s32( rdCostA01, vget_low_s32( sigBitsA ) );
  const int64x2_t rdCostA23_sigBits = vaddw_s32( rdCostA23, vget_high_s32( sigBitsA ) );

  if( spt == DQIntern::SCAN_ISCSBB )
  {
    rdCostZ01 = rdCostZ01_sigBits;
    rdCostZ23 = rdCostZ23_sigBits;
    rdCostA01 = rdCostA01_sigBits;
    rdCostA23 = rdCostA23_sigBits;
  }
  else if( spt == DQIntern::SCAN_SOCSBB )
  {
    const int32x4_t sbbBits = vld1q_s32( state.sbbBits1 );

    rdCostZ01 = vaddw_s32( rdCostZ01_sigBits, vget_low_s32( sbbBits ) );
    rdCostZ23 = vaddw_s32( rdCostZ23_sigBits, vget_high_s32( sbbBits ) );
    rdCostA01 = vaddw_s32( rdCostA01_sigBits, vget_low_s32( sbbBits ) );
    rdCostA23 = vaddw_s32( rdCostA23_sigBits, vget_high_s32( sbbBits ) );
  }
  else // if( spt == DQIntern::SCAN_EOCSBB )
  {
    const int64x2_t rdMax = vdupq_n_s64( DQIntern::rdCostInit );
    const uint8x8_t numSig = load_u8x4( state.numSig );
    const uint8x8_t mask8 = vcgt_u8( numSig, vdup_n_u8( 0 ) );
    const uint64x2_t mask64_lo =
        vreinterpretq_u64_u8( vcombine_u8( vdup_lane_u8( mask8, 0 ), vdup_lane_u8( mask8, 1 ) ) );
    const uint64x2_t mask64_hi =
        vreinterpretq_u64_u8( vcombine_u8( vdup_lane_u8( mask8, 2 ), vdup_lane_u8( mask8, 3 ) ) );

    rdCostZ01 = vbslq_s64( mask64_lo, rdCostZ01_sigBits, rdMax );
    rdCostZ23 = vbslq_s64( mask64_hi, rdCostZ23_sigBits, rdMax );
    rdCostA01 = vbslq_s64( mask64_lo, rdCostA01_sigBits, rdCostA01 );
    rdCostA23 = vbslq_s64( mask64_hi, rdCostA23_sigBits, rdCostA23 );
  }

  const int64x2_t rdCostZ02 = vcombine_s64( vget_low_s64( rdCostZ01 ), vget_low_s64( rdCostZ23 ) );
  const int64x2_t rdCostZ13 = vcombine_s64( vget_high_s64( rdCostZ01 ), vget_high_s64( rdCostZ23 ) );
  const int64x2_t rdCostA02 = vcombine_s64( vget_low_s64( rdCostA01 ), vget_low_s64( rdCostA23 ) );
  const int64x2_t rdCostA13 = vcombine_s64( vget_high_s64( rdCostA01 ), vget_high_s64( rdCostA23 ) );

  const uint64x2_t cmp64_lo = vvenc_vcgtq_s64( rdCostZ02, rdCostA13 );
  const uint64x2_t cmp64_hi = vvenc_vcgtq_s64( rdCostA02, rdCostZ13 );

  const int64x2_t rdBest01 = vbslq_s64( cmp64_lo, rdCostA13, rdCostZ02 );
  const int64x2_t rdBest23 = vbslq_s64( cmp64_hi, rdCostZ13, rdCostA02 );
  vst1q_s64( &decisions.rdCost[0], rdBest01 );
  vst1q_s64( &decisions.rdCost[2], rdBest23 );

  static constexpr int16_t absLevelLut[8] = { 1, 1, 0, 0, 0, 0, 1, 1 };
  const uint32x4_t cmp32 = vcombine_u32( vmovn_u64( cmp64_lo ), vmovn_u64( cmp64_hi ) );
  const uint16x4_t cmp16 = vmovn_u32( cmp32 );
  const int16x4_t absLevel = vbsl_s16( cmp16, vld1_s16( absLevelLut ), vld1_s16( absLevelLut + 4 ) );
  vst1_s16( decisions.absLevel, absLevel );

  static constexpr int8_t prevIdLut[8] = { 0, 2, 0, 2, 0, 0, 0, 0 };
  const uint8x8_t cmp8 = vmovn_u16( vcombine_u16( cmp16, vdup_n_u16( 0 ) ) );
  const int8x8_t prevId = vsub_s8( vld1_s8( prevIdLut ), vreinterpret_s8_u8( cmp8 ) );
  store_s8x4( decisions.prevId, prevId );
}

} // namespace DQInternSimd

template<>
void DepQuant::_initDepQuantARM<NEON>()
{
  m_checkAllRdCostsOdd1 = DQInternSimd::checkAllRdCostsOdd1_neon;
  m_updateStates = DQInternSimd::updateStates_neon;
  m_updateStatesEOS = DQInternSimd::updateStatesEOS_neon;
}

} // namespace vvenc

#endif

//! \}

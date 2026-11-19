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
#include "CommonLib/DepQuantDecide.h"
#include "CommonLib/arm/mem_neon.h"
#include "CommonLib/arm/neon/permute_neon.h"
#include "CommonLib/arm/neon/sum_neon.h"

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
  CHECKD( capEven % 2 != 0, "Cap value must be even" );
  uint8x8_t cappedValue = vmin_u8( value, vdup_n_u8( uint8_t( capEven ) ) );
  cappedValue = vbsl_u8( vdup_n_u8( 1 ), value, cappedValue ); // Preserve parity bit.
  return cappedValue;
}

static inline int16x4_t capWithParity_s16x4_neon( const int16x4_t value, int capEven )
{
  CHECKD( capEven % 2 != 0, "Cap value must be even" );
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

// CommonCtx::updateTemplates for the 16 positions of a 4x4 SBB at once, as x86's templateSums16. absLevels points to the
// SBB's start; nbSbb and shuffle give where the outside neighbours of its positions lie: each neighbour is a table
// lookup in the 16 levels of one neighbouring SBB (index 0x80 gives 0, as for pshufb). All sums fit bytes: sum1st =
// min( 255, sumAbs ) is a saturating byte sum, sumAbs1 <= 5 * 5 and sumNum <= 5, so sumNum << 5 stays in the byte.
static inline void templateSums16_neon( const uint8_t* absLevels, const NbSbbStarts& nbSbb,
                                        const NbOutShuffle16& shuffle, uint8x16_t& tplAcc, uint8x16_t& sum1st )
{
  uint8x16_t levels[NUM_NB_SBB];
  for( int s = 0; s < NUM_NB_SBB; s++ )
  {
    levels[s] = nbSbb.start[s] >= 0 ? vld1q_u8( absLevels + nbSbb.start[s] ) : vdupq_n_u8( 0 );
  }

  // (x+1,y) and (x+2,y) lie in the right SBB, (x,y+1) and (x,y+2) in the one below, (x+1,y+1) in any of the three
  const uint8x16_t idxRight0 = vld1q_u8( shuffle.idx[0][NB_SBB_RIGHT] );
  const uint8x16_t idxRight1 = vld1q_u8( shuffle.idx[1][NB_SBB_RIGHT] );
  const uint8x16_t idxRight2 = vld1q_u8( shuffle.idx[2][NB_SBB_RIGHT] );
  const uint8x16_t idxBelow2 = vld1q_u8( shuffle.idx[2][NB_SBB_BELOW] );
  const uint8x16_t idxDiag2  = vld1q_u8( shuffle.idx[2][NB_SBB_DIAG] );
  const uint8x16_t idxBelow3 = vld1q_u8( shuffle.idx[3][NB_SBB_BELOW] );
  const uint8x16_t idxBelow4 = vld1q_u8( shuffle.idx[4][NB_SBB_BELOW] );

  const uint8x16_t diagRight = vvenc_vqtbl1q_u8( levels[NB_SBB_RIGHT], idxRight2 );
  const uint8x16_t diagBelow = vvenc_vqtbl1q_u8( levels[NB_SBB_BELOW], idxBelow2 );
  const uint8x16_t diagDiag  = vvenc_vqtbl1q_u8( levels[NB_SBB_DIAG], idxDiag2 );

  uint8x16_t nbLevel[5];
  nbLevel[0] = vvenc_vqtbl1q_u8( levels[NB_SBB_RIGHT], idxRight0 );
  nbLevel[1] = vvenc_vqtbl1q_u8( levels[NB_SBB_RIGHT], idxRight1 );
  nbLevel[2] = vorrq_u8( vorrq_u8( diagRight, diagBelow ), diagDiag );
  nbLevel[3] = vvenc_vqtbl1q_u8( levels[NB_SBB_BELOW], idxBelow3 );
  nbLevel[4] = vvenc_vqtbl1q_u8( levels[NB_SBB_BELOW], idxBelow4 );

  uint8x16_t sumAbs  = vdupq_n_u8( 0 );
  uint8x16_t sumAbs1 = vdupq_n_u8( 0 );
  uint8x16_t sumNum  = vdupq_n_u8( 0 );
  for( int k = 0; k < 5; k++ )
  {
    const uint8x16_t t = nbLevel[k];
    sumAbs  = vqaddq_u8( sumAbs, t );
    sumAbs1 = vaddq_u8( sumAbs1, vminq_u8( t, vaddq_u8( vandq_u8( t, vdupq_n_u8( 1 ) ), vdupq_n_u8( 4 ) ) ) );   // min( 4 + ( t & 1 ), t )
    sumNum  = vaddq_u8( sumNum, vminq_u8( t, vdupq_n_u8( 1 ) ) );                                              // !!t
  }
  tplAcc = vorrq_u8( vshlq_n_u8( sumNum, 5 ), sumAbs1 );
  sum1st = sumAbs;
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

  // the context each state continues from, or -2 for none (the state is not used); each state's update below writes
  // only its own refSbbCtxId, so reading all four first gives the same values
  int refId[regSize];
  for( int i = 0; i < regSize; i++ )
  {
    const int prevId = decisions.prevId[i];
    refId[i] = prevId <= -2 ? -2 : prevId < 0 ? -1 : prevId < 4 ? curr.refSbbCtxId[i] : prevId - 4;
  }

  if( const NbSbbStarts* nbSbb = commonCtx.nbSbbStarts( scanInfo ) )
  {
    // 4x4 SBBs: the template sums of the four states in registers; vst4q stores state i into column i of [16][4]
    uint8x16x4_t tplAcc, sum1st;
    for( int i = 0; i < regSize; i++ )
    {
      tplAcc.val[i] = vdupq_n_u8( 0 );
      sum1st.val[i] = vdupq_n_u8( 0 );
      if( refId[i] > -2 )
      {
        if( const uint8_t* absLevels = commonCtx.updateSbbFlags( scanInfo, refId[i], i, curr ) )
        {
          templateSums16_neon( absLevels, *nbSbb, commonCtx.nbOutShuffle16(), tplAcc.val[i], sum1st.val[i] );
        }
      }
    }
    vst4q_u8( &curr.tplAcc[0][0], tplAcc );
    vst4q_u8( &curr.sum1st[0][0], sum1st );
  }
  else
  {
    memset( curr.tplAcc, 0, sizeof( curr.tplAcc ) );
    memset( curr.sum1st, 0, sizeof( curr.sum1st ) );

    for( int i = 0; i < regSize; i++ )
    {
      if( refId[i] > -2 )
      {
        commonCtx.update( scanInfo, refId[i], i, curr );
      }
    }
  }

  memset( curr.numSig, 0, sizeof( curr.numSig ) );

  updateNextCtx_neon( scanInfo, curr );
}

// DQIntern::dequantCoeffs, the NEON version of DQInternSimd::dequantCoeffs<vext> (DepQuantX86.h) with its fallbacks:
// one 4x4 sub-block (scan group of 16) at a time in raster order, only the state chain runs in scan order.
// The chain runs 8 levels per step: entry[ state << 8 | the parities of 8 scan positions (bit k = position k, run from k = 7
// down to 0) ] = the state after them << 8 | ( the state before position k ) >> 1 in bit k.
struct DequantStateChain8
{
  uint16_t entry[4 << 8];

  DequantStateChain8()
  {
    for( int start = 0; start < 4; start++ )
    {
      for( int parities = 0; parities < 256; parities++ )
      {
        int state = start, stateBits = 0;
        for( int k = 7; k >= 0; k-- )
        {
          stateBits |= ( state >> 1 ) << k;
          state = ( 32040 >> ( ( state << 2 ) + ( ( ( parities >> k ) & 1 ) << 1 ) ) ) & 3;
        }
        entry[( start << 8 ) | parities] = uint16_t( ( state << 8 ) | stateBits );
      }
    }
  }
};

// built on first use (a function-local static: no static initialisation order to rely on)
static const uint16_t* dequantStateChain8()
{
  static const DequantStateChain8 table;
  return table.entry;
}

// the sum of the 8 lanes (here distinct bits, so their OR)
static inline unsigned addLanes_u16x8( const uint16x8_t v )
{
#if REAL_TARGET_AARCH64
  return vaddvq_u16( v );
#else
  const uint64x2_t sums = vpaddlq_u32( vpaddlq_u16( v ) );
  return unsigned( vgetq_lane_u64( sums, 0 ) + vgetq_lane_u64( sums, 1 ) );
#endif
}

static void dequantCoeffs_neon( const TCoeffSig* qCoeff, TCoeff* tCoeff, const ScanElement* scan, int lastScanIdx,
                                int width, int height, int shift, Intermediate_Int invQScale, const int* piDequantCoef,
                                TCoeff minTCoeff, TCoeff maxTCoeff )
{
  // |qIdx| = | 2 level - sign( level ) ( state >> 1 ) | <= 65536 for 16-bit levels
  const int     add      = shift < 0 ? 0 : ( 1 << shift ) >> 1;
  const int64_t maxScale = int64_t( invQScale ) << std::max( -shift, 0 );
  const int     numCoded = std::min( width, JVET_C0024_ZERO_OUT_TH ) * std::min( height, JVET_C0024_ZERO_OUT_TH );
  if( piDequantCoef || width < 4 || height < 4 || lastScanIdx >= numCoded
      || maxScale * 65536 + add > std::numeric_limits<int32_t>::max() )
  {
    DQIntern::dequantCoeffs( qCoeff, tCoeff, scan, lastScanIdx, width, height, shift, invQScale, piDequantCoef, minTCoeff,
                             maxTCoeff );
    return;
  }

  // as in the scalar code: a negative shift scales the step once, at the last position, when its level is nonzero
  const int       scale  = shift < 0 && qCoeff[scan[lastScanIdx].idx] ? invQScale << -shift : invQScale;
  const int32x4_t vScale = vdupq_n_s32( scale );
  const int32x4_t vAdd   = vdupq_n_s32( add );
  const int32x4_t vShift = vdupq_n_s32( -std::max( shift, 0 ) );
  const int32x4_t vMin   = vdupq_n_s32( minTCoeff );
  const int32x4_t vMax   = vdupq_n_s32( maxTCoeff );
  const int16x8_t vOne   = vdupq_n_s16( 1 );
  const int16x8_t vZero  = vdupq_n_s16( 0 );

  // the diagonal scan inside a 4x4 sub-block, per raster position r = 4 y + x (rows 0-1, rows 2-3): its scan index, and
  // 1 << that index (its bit in the scan-order masks)
  static const uint16_t rasterToScan[16] = { 0, 2, 5, 9, 1, 4, 8, 12, 3, 7, 11, 14, 6, 10, 13, 15 };
  static const uint16_t scanBit[16]      = { 1,  4,   32,   512,  2,    16,   256,   4096,
                                             8,  128, 2048, 16384, 64, 1024, 8192, 32768 };
  const uint16x8_t scanIdx01 = vld1q_u16( rasterToScan ), scanIdx23 = vld1q_u16( rasterToScan + 8 );
  const uint16x8_t bit01     = vld1q_u16( scanBit ), bit23 = vld1q_u16( scanBit + 8 );

  const uint16_t* stateChain8 = dequantStateChain8();
  int             chain       = 0;   // the state between groups, << 8 (as in stateChain8's entries)
  for( int group = lastScanIdx >> 4; group >= 0; group-- )
  {
    const int      firstIdx = group << 4;
    const unsigned topLeft  = scan[firstIdx].idx;   // a group's diagonal scan starts at its sub-block's top-left
    CHECKD( ( scan[firstIdx].x & 3 ) || ( scan[firstIdx].y & 3 ), "a scan group must be an aligned 4x4 sub-block" );
    const TCoeffSig* src = qCoeff + topLeft;
    TCoeff*          dst = tCoeff + topLeft;

    // the levels, rows 0-1 and rows 2-3; those after lastScanIdx (in the first group) count as 0, as in the scalar loop
    int16x8_t lvl01 = vcombine_s16( vld1_s16( src ), vld1_s16( src + width ) );
    int16x8_t lvl23 = vcombine_s16( vld1_s16( src + 2 * width ), vld1_s16( src + 3 * width ) );
    if( lastScanIdx - firstIdx < 15 )
    {
      const uint16x8_t lastInGroup = vdupq_n_u16( uint16_t( lastScanIdx - firstIdx ) );
      lvl01 = vbicq_s16( lvl01, vreinterpretq_s16_u16( vcgtq_u16( scanIdx01, lastInGroup ) ) );
      lvl23 = vbicq_s16( lvl23, vreinterpretq_s16_u16( vcgtq_u16( scanIdx23, lastInGroup ) ) );
    }

    // the parities as a 16-bit scan-order mask (each raster position adds its own bit), run through the chain 8 at a
    // time from scan index 15 down; stateBits bit i: the state before scan index i, >> 1
    const unsigned parities = addLanes_u16x8( vorrq_u16( vandq_u16( vtstq_s16( lvl01, vOne ), bit01 ),
                                                         vandq_u16( vtstq_s16( lvl23, vOne ), bit23 ) ) );
    unsigned entry     = stateChain8[chain | ( parities >> 8 )];
    unsigned stateBits = ( entry & 0xff ) << 8;
    entry              = stateChain8[( entry & 0x300 ) | ( parities & 0xff )];
    stateBits         |= entry & 0xff;
    chain              = entry & 0x300;

    // -sign( level ) * ( state >> 1 ) per raster position: all ones where the state before it is 2 or 3
    const uint16x8_t states = vdupq_n_u16( uint16_t( stateBits ) );
    auto correction = [&]( const int16x8_t lvl, const uint16x8_t bit )
    {
      const int16x8_t negSign = vreinterpretq_s16_u16( vsubq_u16( vcgtq_s16( lvl, vZero ), vcltq_s16( lvl, vZero ) ) );
      return vandq_s16( vreinterpretq_s16_u16( vtstq_u16( states, bit ) ), negSign );
    };
    const int16x8_t corr01 = correction( lvl01, bit01 );
    const int16x8_t corr23 = correction( lvl23, bit23 );

    // qIdx = 2 level - sign( level ) ( state >> 1 ) in 32 bits, then ( qIdx scale + add ) >> shift, clipped
    auto dequantRow = [&]( const int16x4_t lvl, const int16x4_t corr, TCoeff* row )
    {
      const int32x4_t qIdx = vaddw_s16( vshll_n_s16( lvl, 1 ), corr );
      const int32x4_t coef = vshlq_s32( vmlaq_s32( vAdd, qIdx, vScale ), vShift );
      vst1q_s32( row, vminq_s32( vmaxq_s32( coef, vMin ), vMax ) );
    };
    dequantRow( vget_low_s16( lvl01 ), vget_low_s16( corr01 ), dst );
    dequantRow( vget_high_s16( lvl01 ), vget_high_s16( corr01 ), dst + width );
    dequantRow( vget_low_s16( lvl23 ), vget_low_s16( corr23 ), dst + 2 * width );
    dequantRow( vget_high_s16( lvl23 ), vget_high_s16( corr23 ), dst + 3 * width );
  }
}

static inline uint64x2_t vvenc_vcgtq_s64( int64x2_t a, int64x2_t b )
{
#if REAL_TARGET_AARCH64
  return vcgtq_s64( a, b );
#else
  return vreinterpretq_u64_s64( vshrq_n_s64( vqsubq_s64( b, a ), 63 ) );
#endif
}

// findFirstPos with the control flow of the x86 kernel: whole 4x4 subblocks from the last one backwards until one holds
// a coefficient above the threshold, then coefficient by coefficient. A subblock's test is one compare of the largest
// absolute value of its four rows; vabsq_s32 wraps INT32_MIN like _mm_abs_epi32 and the signed max keeps it the
// smallest, so the test is true exactly when x86's is.
void findFirstPos_neon( int& firstTestPos, const TCoeff* tCoeff, const TUParameters& tuPars, int defaultTh,
                        bool zeroOutForThres, int zeroOutWidth, int zeroOutHeight )
{
  if( firstTestPos >= 16 && tuPars.m_log2SbbWidth == 2 && tuPars.m_log2SbbHeight == 2 )
  {
    const int       sbbSize = tuPars.m_sbbSize;
    const ptrdiff_t stride  = tuPars.m_width;

    // move to the start of the current subblock
    firstTestPos -= sbbSize - 1;

    for( ; firstTestPos >= 0; firstTestPos -= sbbSize )
    {
      const ScanElement& sbbStart = tuPars.m_scanId2BlkPos[firstTestPos];
      if( zeroOutForThres && ( sbbStart.x >= zeroOutWidth || sbbStart.y >= zeroOutHeight ) )
      {
        continue;
      }

      // the subblock is dense: 4 rows of 4 in raster order
      const TCoeff*   row  = &tCoeff[sbbStart.idx];
      const int32x4_t row0 = vld1q_s32( row );
      const int32x4_t row1 = vld1q_s32( row + stride );
      const int32x4_t row2 = vld1q_s32( row + 2 * stride );
      const int32x4_t row3 = vld1q_s32( row + 3 * stride );

      const int32x4_t max01  = vmaxq_s32( vabsq_s32( row0 ), vabsq_s32( row1 ) );
      const int32x4_t max23  = vmaxq_s32( vabsq_s32( row2 ), vabsq_s32( row3 ) );
      const int32x4_t maxAbs = vmaxq_s32( max01, max23 );
      if( horizontal_max_s32x4( maxAbs ) > defaultTh )
      {
        break;
      }
    }

    if( firstTestPos >= 0 )
    {
      // a coefficient was found: back to the end of its subblock for the coefficient-wise search
      firstTestPos += sbbSize - 1;
    }
  }

  for( ; firstTestPos >= 0; firstTestPos-- )
  {
    const ScanElement& pos = tuPars.m_scanId2BlkPos[firstTestPos];
    if( zeroOutForThres && ( pos.x >= zeroOutWidth || pos.y >= zeroOutHeight ) )
    {
      continue;
    }
    if( abs( tCoeff[pos.idx] ) > defaultTh )
    {
      break;
    }
  }
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

// The bits of a candidate level in the coefficient context cff of state stateId, as in the scalar checkRdCosts: from 4
// on, two levels share a gtx entry and the Rice code with the state's parameter tells them apart
static inline int32_t levelBits_neon( const StateMem& state, const int stateId, const CoeffFracBits& cff, const TCoeff level )
{
  if( level < 4 )
  {
    return cff.bits[level];
  }
  const unsigned value = ( level - 4 ) >> 1;
  return cff.bits[level - ( value << 1 )] + g_goRiceBits[state.m_goRicePar[stateId]][std::min<unsigned>( value, RICEMAX - 1 )];
}

// checkAllRdCosts of the four states with the data flow of the x86 DQInternSimd::checkAllRdCosts: the same costs, and per
// decision the same candidates in the same order, each replacing the best only when strictly smaller. Decisions are
// written, not read. anyGe4: a candidate level may be 4 or more. Per decision (A: pqData[0] from states 0 and 1,
// pqData[3] from 2 and 3; B: pqData[2] from 0 and 1, pqData[1] from 2 and 3):
//   decision 0: Z from 0, A from 0, B from 1    decision 1: Z from 2, A from 2, B from 3
//   decision 2: B from 0, Z from 1, A from 1    decision 3: B from 2, Z from 3, A from 3
// Forced inline: xPopulateTrellis calls it for every coefficient
template<bool anyGe4>
static ALWAYS_INLINE void checkAllRdCosts_neon( const ScanPosType spt, const PQData* pqData, Decisions& decisions, const StateMem& state )
{
  const int64x2_t rdCost01 = vld1q_s64( &state.rdCost[0] );
  const int64x2_t rdCost23 = vld1q_s64( &state.rdCost[2] );

  int64x2_t rdCostZ01 = rdCost01;
  int64x2_t rdCostZ23 = rdCost23;
  int64x2_t rdCostA01 = vaddq_s64( rdCost01, vdupq_n_s64( pqData[0].deltaDist ) );
  int64x2_t rdCostA23 = vaddq_s64( rdCost23, vdupq_n_s64( pqData[3].deltaDist ) );
  int64x2_t rdCostB01 = vaddq_s64( rdCost01, vdupq_n_s64( pqData[2].deltaDist ) );
  int64x2_t rdCostB23 = vaddq_s64( rdCost23, vdupq_n_s64( pqData[1].deltaDist ) );

  // the significance bits of each state's context: intBits[0] for Z, intBits[1] for A and B
  const uint32x2_t sigBits0  = vld1_u32( state.m_sigFracBitsArray[0][state.ctx.sig[0]].intBits );
  const uint32x2_t sigBits1  = vld1_u32( state.m_sigFracBitsArray[1][state.ctx.sig[1]].intBits );
  const uint32x2_t sigBits2  = vld1_u32( state.m_sigFracBitsArray[2][state.ctx.sig[2]].intBits );
  const uint32x2_t sigBits3  = vld1_u32( state.m_sigFracBitsArray[3][state.ctx.sig[3]].intBits );
  const uint32x4x2_t sigBits = vtrnq_u32( vcombine_u32( sigBits0, sigBits2 ), vcombine_u32( sigBits1, sigBits3 ) );
  int32x4_t addZ             = vreinterpretq_s32_u32( sigBits.val[0] );
  int32x4_t sigBitsNonZero   = vreinterpretq_s32_u32( sigBits.val[1] );

  // the level bits of each state's coefficient context: A of (lvl0, lvl0, lvl3, lvl3), B of (lvl2, lvl2, lvl1, lvl1)
  const TCoeff lvl0 = pqData[0].absLevel;
  const TCoeff lvl1 = pqData[1].absLevel;
  const TCoeff lvl2 = pqData[2].absLevel;
  const TCoeff lvl3 = pqData[3].absLevel;
  const CoeffFracBits& cff0 = state.m_gtxFracBitsArray[state.ctx.cff[0]];
  const CoeffFracBits& cff1 = state.m_gtxFracBitsArray[state.ctx.cff[1]];
  const CoeffFracBits& cff2 = state.m_gtxFracBitsArray[state.ctx.cff[2]];
  const CoeffFracBits& cff3 = state.m_gtxFracBitsArray[state.ctx.cff[3]];
  int32x4_t addA, addB;
  if( !anyGe4 )
  {
    // levels below 4: each lane straight from its table entry
    int32x2_t addA01 = vld1_dup_s32( &cff0.bits[lvl0] );
    int32x2_t addA23 = vld1_dup_s32( &cff2.bits[lvl3] );
    int32x2_t addB01 = vld1_dup_s32( &cff0.bits[lvl2] );
    int32x2_t addB23 = vld1_dup_s32( &cff2.bits[lvl1] );
    addA01 = vld1_lane_s32( &cff1.bits[lvl0], addA01, 1 );
    addA23 = vld1_lane_s32( &cff3.bits[lvl3], addA23, 1 );
    addB01 = vld1_lane_s32( &cff1.bits[lvl2], addB01, 1 );
    addB23 = vld1_lane_s32( &cff3.bits[lvl1], addB23, 1 );
    addA = vcombine_s32( addA01, addA23 );
    addB = vcombine_s32( addB01, addB23 );
  }
  else
  {
    const int32_t bitsA0 = levelBits_neon( state, 0, cff0, lvl0 );
    const int32_t bitsA1 = levelBits_neon( state, 1, cff1, lvl0 );
    const int32_t bitsA2 = levelBits_neon( state, 2, cff2, lvl3 );
    const int32_t bitsA3 = levelBits_neon( state, 3, cff3, lvl3 );
    const int32_t bitsB0 = levelBits_neon( state, 0, cff0, lvl2 );
    const int32_t bitsB1 = levelBits_neon( state, 1, cff1, lvl2 );
    const int32_t bitsB2 = levelBits_neon( state, 2, cff2, lvl1 );
    const int32_t bitsB3 = levelBits_neon( state, 3, cff3, lvl1 );
    addA = vcombine_s32( vset_lane_s32( bitsA1, vdup_n_s32( bitsA0 ), 1 ), vset_lane_s32( bitsA3, vdup_n_s32( bitsA2 ), 1 ) );
    addB = vcombine_s32( vset_lane_s32( bitsB1, vdup_n_s32( bitsB0 ), 1 ), vset_lane_s32( bitsB3, vdup_n_s32( bitsB2 ), 1 ) );
  }

  if( spt == SCAN_SOCSBB )
  {
    const int32x4_t sbbBits = vld1q_s32( state.sbbBits1 );
    addA = vaddq_s32( addA, sbbBits );
    addB = vaddq_s32( addB, sbbBits );
    addZ = vaddq_s32( addZ, sbbBits );
  }
  else if( spt == SCAN_EOCSBB )
  {
    // a state without significant coefficients: Z costs rdCostInit (plus its significance bits, as in the x86
    // kernel), A and B have no significance bits; noSig is all ones where numSig is 0, widened by sign extension
    const uint8x8_t numSig     = load_u8x4( state.numSig );
    const int8x8_t  noSig8     = vreinterpret_s8_u8( vceq_u8( numSig, vdup_n_u8( 0 ) ) );
    const int32x4_t noSig      = vmovl_s16( vget_low_s16( vmovl_s8( noSig8 ) ) );
    const uint64x2_t noSig01   = vreinterpretq_u64_s64( vmovl_s32( vget_low_s32( noSig ) ) );
    const uint64x2_t noSig23   = vreinterpretq_u64_s64( vmovl_s32( vget_high_s32( noSig ) ) );
    const int64x2_t rdCostInit = vdupq_n_s64( DQIntern::rdCostInit );
    sigBitsNonZero = vbicq_s32( sigBitsNonZero, noSig );
    rdCostZ01      = vbslq_s64( noSig01, rdCostInit, rdCostZ01 );
    rdCostZ23      = vbslq_s64( noSig23, rdCostInit, rdCostZ23 );
  }

  addA = vaddq_s32( addA, sigBitsNonZero );
  addB = vaddq_s32( addB, sigBitsNonZero );

  rdCostZ01 = vaddw_s32( rdCostZ01, vget_low_s32( addZ ) );
  rdCostZ23 = vaddw_s32( rdCostZ23, vget_high_s32( addZ ) );
  rdCostA01 = vaddw_s32( rdCostA01, vget_low_s32( addA ) );
  rdCostA23 = vaddw_s32( rdCostA23, vget_high_s32( addA ) );
  rdCostB01 = vaddw_s32( rdCostB01, vget_low_s32( addB ) );
  rdCostB23 = vaddw_s32( rdCostB23, vget_high_s32( addB ) );

  const int64x2_t rdCostZ02 = vcombine_s64( vget_low_s64( rdCostZ01 ), vget_low_s64( rdCostZ23 ) );
  const int64x2_t rdCostZ13 = vcombine_s64( vget_high_s64( rdCostZ01 ), vget_high_s64( rdCostZ23 ) );
  const int64x2_t rdCostA02 = vcombine_s64( vget_low_s64( rdCostA01 ), vget_low_s64( rdCostA23 ) );
  const int64x2_t rdCostA13 = vcombine_s64( vget_high_s64( rdCostA01 ), vget_high_s64( rdCostA23 ) );
  const int64x2_t rdCostB02 = vcombine_s64( vget_low_s64( rdCostB01 ), vget_low_s64( rdCostB23 ) );
  const int64x2_t rdCostB13 = vcombine_s64( vget_high_s64( rdCostB01 ), vget_high_s64( rdCostB23 ) );

  // per decision the prevId (lanes 0-3) and absLevel (lanes 4-7) of the best candidate: start (0, 2, 0, 2) and
  // (0, 0, lvl2, lvl1), first candidates (0, 2, 1, 3) and (lvl0, lvl3, 0, 0), second (1, 3, 1, 3) and (lvl2, lvl1, lvl0, lvl3)
  int16x4_t lvl03 = vdup_n_s16( int16_t( lvl0 ) );
  lvl03           = vset_lane_s16( int16_t( lvl3 ), lvl03, 1 );
  lvl03           = vset_lane_s16( int16_t( lvl2 ), lvl03, 2 );
  lvl03           = vset_lane_s16( int16_t( lvl1 ), lvl03, 3 );
  static constexpr int16_t candMaskLut[4] = { -1, -1, 0, 0 };
  static constexpr int16_t prevIdLut[12]  = { 0, 2, 0, 2, 0, 2, 1, 3, 1, 3, 1, 3 };
  const int16x4_t candMask = vld1_s16( candMaskLut );
  const int16x4_t prevId0  = vld1_s16( prevIdLut );
  const int16x4_t prevId1  = vld1_s16( prevIdLut + 4 );
  const int16x4_t prevId2  = vld1_s16( prevIdLut + 8 );
  const int16x8_t cand1    = vcombine_s16( prevId1, vand_s16( lvl03, candMask ) );
  const int16x8_t cand2    = vcombine_s16( prevId2, vext_s16( lvl03, lvl03, 2 ) );
  int16x8_t       best     = vcombine_s16( prevId0, vbic_s16( lvl03, candMask ) );

  // first candidates
  uint64x2_t chng01 = vvenc_vcgtq_s64( rdCostZ02, rdCostA02 );
  uint64x2_t chng23 = vvenc_vcgtq_s64( rdCostB02, rdCostZ13 );
  int64x2_t  best01 = vbslq_s64( chng01, rdCostA02, rdCostZ02 );
  int64x2_t  best23 = vbslq_s64( chng23, rdCostZ13, rdCostB02 );
  uint16x4_t chng   = vmovn_u32( vcombine_u32( vmovn_u64( chng01 ), vmovn_u64( chng23 ) ) );
  best              = vbslq_s16( vcombine_u16( chng, chng ), cand1, best );

  // second candidates
  chng01 = vvenc_vcgtq_s64( best01, rdCostB13 );
  chng23 = vvenc_vcgtq_s64( best23, rdCostA13 );
  best01 = vbslq_s64( chng01, rdCostB13, best01 );
  best23 = vbslq_s64( chng23, rdCostA13, best23 );
  chng   = vmovn_u32( vcombine_u32( vmovn_u64( chng01 ), vmovn_u64( chng23 ) ) );
  best   = vbslq_s16( vcombine_u16( chng, chng ), cand2, best );

  vst1q_s64( &decisions.rdCost[0], best01 );
  vst1q_s64( &decisions.rdCost[2], best23 );
  vst1_s16( decisions.absLevel, vget_high_s16( best ) );
  store_s8x4( decisions.prevId, vmovn_s16( best ) );
}

// The per-coefficient kernels of this file, for DepQuant::xPopulateTrellis to inline (as DQInternSimd::KernelsX86)
struct KernelsNeon
{
  static void checkAllRdCosts( const vvenc::DepQuant&, const ScanPosType spt, const PQData* pqData, Decisions& decisions, const StateMem& state )
  {
    checkAllRdCosts_neon<false>( spt, pqData, decisions, state );
  }
  static void checkAllRdCostsGe4( const vvenc::DepQuant&, const ScanPosType spt, const PQData* pqData, Decisions& decisions, const StateMem& state )
  {
    checkAllRdCosts_neon<true>( spt, pqData, decisions, state );
  }
  static void checkAllRdCostsOdd1( const vvenc::DepQuant&, const ScanPosType spt, const int64_t pq_a_dist, const int64_t pq_b_dist, Decisions& decisions, const StateMem& state )
  {
    checkAllRdCostsOdd1_neon( spt, pq_a_dist, pq_b_dist, decisions, state );
  }
  static void updateStates( const vvenc::DepQuant&, const ScanInfo& scanInfo, const Decisions& decisions, StateMem& curr )
  {
    updateStates_neon( scanInfo, decisions, curr );
  }
  static void updateStatesEOS( const vvenc::DepQuant&, const ScanInfo& scanInfo, const Decisions& decisions, const StateMem& skip, StateMem& curr, CommonCtx& commonCtx )
  {
    updateStatesEOS_neon( scanInfo, decisions, skip, curr, commonCtx );
  }
};

} // namespace DQInternSimd

template<>
void DepQuant::_initDepQuantARM<NEON>()
{
  m_findFirstPos = DQInternSimd::findFirstPos_neon;
  m_checkAllRdCostsOdd1 = DQInternSimd::checkAllRdCostsOdd1_neon;
  m_dequantCoeffs = DQInternSimd::dequantCoeffs_neon;
  m_updateStates = DQInternSimd::updateStates_neon;
  m_updateStatesEOS = DQInternSimd::updateStatesEOS_neon;
  m_checkAllRdCosts = DQInternSimd::checkAllRdCosts_neon<false>;
  m_checkAllRdCostsGe4 = DQInternSimd::checkAllRdCosts_neon<true>;
  m_populateTrellis = xPopulateTrellis<DQInternSimd::KernelsNeon>;
}

} // namespace vvenc

#endif

//! \}

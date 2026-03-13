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

static inline void updateRemRegBins_neon( const int8x8_t prevState, const uint8x8_t hasPrevMask, const uint8x8_t absVal,
                                          StateMem& curr )
{
  const int8x8_t prevState_x2 = vshl_n_s8( prevState, 1 );
  const int8x8_t rrbShuffle = vzip_s8( prevState_x2, vadd_s8( prevState_x2, vdup_n_s8( 1 ) ) ).val[0];
  const uint16x4_t hasPrevMaskHalf = vreinterpret_u16_u8( vzip_u8( hasPrevMask, hasPrevMask ).val[0] );
  int16x4_t remRegBins =
      vreinterpret_s16_s8( vtbl1_s8( vreinterpret_s8_s16( vld1_s16( curr.remRegBins ) ), rrbShuffle ) );
  remRegBins = vsub_s16( remRegBins, vdup_n_s16( 1 ) );
  remRegBins = vbsl_s16( hasPrevMaskHalf, remRegBins, vdup_n_s16( curr.initRemRegBins ) );

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

  updateRemRegBins_neon( prevState, hasPrevMask, absVal, curr );

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

} // namespace DQInternSimd

template<>
void DepQuant::_initDepQuantARM<NEON>()
{
  m_updateStates = DQInternSimd::updateStates_neon;
}

} // namespace vvenc

#endif

//! \}

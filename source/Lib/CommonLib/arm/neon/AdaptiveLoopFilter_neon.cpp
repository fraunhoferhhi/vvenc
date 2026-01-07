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
 * \file AdaptiveLoopFilter_neon.cpp
 * \brief Neon implementation of ALF for AArch64.
 */
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include <arm_neon.h>

#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "permute_neon.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_ALF
namespace vvenc
{

constexpr int ALF_7x7_VB_FOLD_MIN_DIST = -2;
constexpr int ALF_7x7_VB_FOLD_MAX_DIST = 3;

// Lookup table for ALF coefficient shuffling.
static const uint8_t g_alf_shuffle_table[4][32] = {
    // transposeIdx = 0
    {  0,  1,  2,  3,  4,  5,  6,  7,   8,   9,  10,  11,  12,  13,  14,  15,
      16, 17, 18, 19, 20, 21, 22, 23, 255, 255, 255, 255, 255, 255, 255, 255 },
    // transposeIdx = 1
    { 18, 19, 8, 9, 20, 21, 16, 17,   2,   3,  10,  11,  22,  23,  14,  15,
       6,  7, 0, 1,  4,  5, 12, 13, 255, 255, 255, 255, 255, 255, 255, 255 },
    // transposeIdx = 2
    { 0, 1,  6,  7,  4,  5,  2,  3,  16,  17,  14,  15,  12,  13,  10,  11,
      8, 9, 18, 19, 20, 21, 22, 23, 255, 255, 255, 255, 255, 255, 255, 255 },
    // transposeIdx = 3
    { 18, 19, 16, 17, 20, 21,  8,  9,  6,    7,  14,  15,  22,  23,  10,  11,
       2,  3,  0,  1,  4,  5, 12, 13, 255, 255, 255, 255, 255, 255, 255, 255 } };

static inline void load_shuffle_coeffs_neon( const uint8_t* shuffleTable0, const uint8_t* shuffleTable1,
                                             const short* coeff0, const short* coeff1, int16x4_t coeff[6] )
{
  const uint8x16_t c0_04 = vreinterpretq_u8_s16( vld1q_s16( coeff0 ) );
  const uint8x16_t c1_04 = vreinterpretq_u8_s16( vld1q_s16( coeff1 ) );
  const uint8x16_t c0_8 = vreinterpretq_u8_s16( vcombine_s16( vld1_s16( coeff0 + 8 ), vdup_n_s16( 0 ) ) );
  const uint8x16_t c1_8 = vreinterpretq_u8_s16( vcombine_s16( vld1_s16( coeff1 + 8 ), vdup_n_s16( 0 ) ) );

  const uint8x16_t shuffleIndices0_04 = vld1q_u8( shuffleTable0 + 0 );
  const uint8x16_t shuffleIndices1_04 = vld1q_u8( shuffleTable1 + 0 );
  const uint8x16_t shuffleIndices0_8 = vld1q_u8( shuffleTable0 + 16 );
  const uint8x16_t shuffleIndices1_8 = vld1q_u8( shuffleTable1 + 16 );

  const uint8x16x2_t raw_coeff0 = { c0_04, c0_8 };
  const uint8x16x2_t raw_coeff1 = { c1_04, c1_8 };

  const int16x8_t coeff02 = vreinterpretq_s16_u8( vvenc_vqtbl2q_u8( raw_coeff0, shuffleIndices0_04 ) );
  const int16x8_t coeff13 = vreinterpretq_s16_u8( vvenc_vqtbl2q_u8( raw_coeff1, shuffleIndices1_04 ) );
  const int16x8_t coeff4 = vreinterpretq_s16_u8( vvenc_vqtbl2q_u8( raw_coeff0, shuffleIndices0_8 ) );
  const int16x8_t coeff5 = vreinterpretq_s16_u8( vvenc_vqtbl2q_u8( raw_coeff1, shuffleIndices1_8 ) );

  coeff[0] = vget_low_s16( coeff02 );
  coeff[2] = vget_high_s16( coeff02 );
  coeff[1] = vget_low_s16( coeff13 );
  coeff[3] = vget_high_s16( coeff13 );
  coeff[4] = vget_low_s16( coeff4 );
  coeff[5] = vget_low_s16( coeff5 );
}

static inline int16x8_t clip3_neon( const int16x8_t val, const int16x8_t minVal, const int16x8_t maxVal )
{
  return vmaxq_s16( vminq_s16( val, maxVal ), minVal );
}

static inline void processALF_CoeffPair_neon( const Pel* ptr0, const Pel* ptr1, const Pel* ptr2, const Pel* ptr3,
                                              const int16x8_t curr, int16x8_t& accA, int16x8_t& accB )
{
  const int16x8_t v0 = vld1q_s16( ptr0 );
  const int16x8_t v1 = vld1q_s16( ptr1 );
  const int16x8_t v2 = vld1q_s16( ptr2 );
  const int16x8_t v3 = vld1q_s16( ptr3 );

  const int16x8_t diff0 = vsubq_s16( v0, curr );
  const int16x8_t diff1 = vsubq_s16( v1, curr );
  const int16x8_t diff2 = vsubq_s16( v2, curr );
  const int16x8_t diff3 = vsubq_s16( v3, curr );

  accA = vaddq_s16( diff0, diff1 );
  accB = vaddq_s16( diff2, diff3 );
}

template<bool isFoldingRequired>
static inline int16x8_t processALF7x7Row_neon( const Pel* pImg0, const int distance, const ptrdiff_t srcStride,
                                               const int16x4_t coeff[6], const int clpRngMax )
{
  constexpr int SHIFT = AdaptiveLoopFilter::m_NUM_BITS - 1;

  const Pel* pImg1 = pImg0 + srcStride; // y+1
  const Pel* pImg2 = pImg0 - srcStride; // y-1
  const Pel* pImg3 = pImg1 + srcStride; // y+2
  const Pel* pImg4 = pImg2 - srcStride; // y-2
  const Pel* pImg5 = pImg3 + srcStride; // y+3
  const Pel* pImg6 = pImg4 - srcStride; // y-3

  if( isFoldingRequired )
  {
    // When the current line is near the VB (vbPos), some of rows access could point across the CTU boundary.
    // Distance = 0 or 1, reuse current row (pImg0).
    // Distance = 2 or -1, reuse previous folded row (pImg1/pImg2).
    // Distance = 3 or -2, reuse previous folded row (pImg3/pImg4).
    if( distance > 0 && distance <= ALF_7x7_VB_FOLD_MAX_DIST ) // Above.
    {
      pImg1 = distance == 1 ? pImg0 : pImg1;
      pImg3 = distance <= 2 ? pImg1 : pImg3;
      pImg5 = distance <= 3 ? pImg3 : pImg5;

      pImg2 = distance == 1 ? pImg0 : pImg2;
      pImg4 = distance <= 2 ? pImg2 : pImg4;
      pImg6 = distance <= 3 ? pImg4 : pImg6;
    }
    else if( distance <= 0 && distance >= ALF_7x7_VB_FOLD_MIN_DIST ) // Bottom.
    {
      pImg1 = distance == 0 ? pImg0 : pImg1;
      pImg3 = distance >= -1 ? pImg1 : pImg3;
      pImg5 = distance >= -2 ? pImg3 : pImg5;

      pImg2 = distance == 0 ? pImg0 : pImg2;
      pImg4 = distance >= -1 ? pImg2 : pImg4;
      pImg6 = distance >= -2 ? pImg4 : pImg6;
    }
  }

  const int16x8_t curr = vld1q_s16( pImg0 );

  int16x8_t a0, a1, a2, a3, a4, a5;
  int16x8_t b0, b1, b2, b3, b4, b5;
  processALF_CoeffPair_neon( pImg5 + 0, pImg6 + 0, pImg3 + 1, pImg4 - 1, curr, a0, b0 );
  processALF_CoeffPair_neon( pImg3 + 0, pImg4 + 0, pImg3 - 1, pImg4 + 1, curr, a1, b1 );
  processALF_CoeffPair_neon( pImg1 + 2, pImg2 - 2, pImg1 + 1, pImg2 - 1, curr, a2, b2 );
  processALF_CoeffPair_neon( pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1, curr, a3, b3 );
  processALF_CoeffPair_neon( pImg1 - 2, pImg2 + 2, pImg0 + 3, pImg0 - 3, curr, a4, b4 );
  processALF_CoeffPair_neon( pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1, curr, a5, b5 );

  int32x4_t accLo = vmull_lane_s16( vget_low_s16( a0 ), coeff[0], 0 );
  int32x4_t accHi = vmull_lane_s16( vget_high_s16( a0 ), coeff[1], 0 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b0 ), coeff[0], 1 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b0 ), coeff[1], 1 );

  accLo = vmlal_lane_s16( accLo, vget_low_s16( a1 ), coeff[0], 2 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a1 ), coeff[1], 2 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b1 ), coeff[0], 3 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b1 ), coeff[1], 3 );

  accLo = vmlal_lane_s16( accLo, vget_low_s16( a2 ), coeff[2], 0 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a2 ), coeff[3], 0 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b2 ), coeff[2], 1 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b2 ), coeff[3], 1 );

  accLo = vmlal_lane_s16( accLo, vget_low_s16( a3 ), coeff[2], 2 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a3 ), coeff[3], 2 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b3 ), coeff[2], 3 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b3 ), coeff[3], 3 );

  accLo = vmlal_lane_s16( accLo, vget_low_s16( a4 ), coeff[4], 0 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a4 ), coeff[5], 0 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b4 ), coeff[4], 1 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b4 ), coeff[5], 1 );

  accLo = vmlal_lane_s16( accLo, vget_low_s16( a5 ), coeff[4], 2 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a5 ), coeff[5], 2 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b5 ), coeff[4], 3 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b5 ), coeff[5], 3 );

  int16x8_t acc;
  if( isFoldingRequired && distance >= 0 && distance <= 1 )
  {
    // Weaker filter, closer to VB.
    acc = vcombine_s16( vrshrn_n_s32( accLo, SHIFT + 3 ), vrshrn_n_s32( accHi, SHIFT + 3 ) );
  }
  else
  {
    // Regular filter strength.
    acc = vcombine_s16( vrshrn_n_s32( accLo, SHIFT ), vrshrn_n_s32( accHi, SHIFT ) );
  }

  acc = vqaddq_s16( acc, curr );
  return clip3_neon( acc, vdupq_n_s16( 0 ), vdupq_n_s16( clpRngMax ) );
}

void Filter7x7Blk_neon( const AlfClassifier* classifier, const PelUnitBuf& recDst, const CPelUnitBuf& recSrc,
                        const Area& blkDst, const Area& blk, const ComponentID compId, const short* filterSet,
                        const short* fClipSet, const ClpRng& clpRng, const CodingStructure& cs, const int vbCTUHeight,
                        int vbPos )
{
  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const ptrdiff_t srcStride = srcLuma.stride;
  const ptrdiff_t dstStride = dstLuma.stride;

  const int width = blk.width;
  const int height = blk.height;

  const Pel* src = srcLuma.buf + blk.y * srcStride + blk.x;
  Pel* dst = dstLuma.buf + blkDst.y * dstStride + blkDst.x;

  constexpr size_t STEP_X = 8;
  constexpr size_t STEP_Y = 4;

  CHECKD( width % STEP_X, "Width must be multiple of 8!" );
  CHECKD( height % STEP_Y, "Height must be multiple of 4!" );
  CHECKD( width <= 0, "Width must be greater than 0!" );
  CHECKD( height <= 0, "Height must be greater than 0!" );

  const int clpRngMax = clpRng.max();

  int i = 0;
  do
  {
    int yVbPos = ( blkDst.y + i ) & ( vbCTUHeight - 1 ); // Row’s position inside its CTU.

    auto calculateNextVbPosDist = [&yVbPos, vbPos, vbCTUHeight]() -> int
    {
      int distance = vbPos - yVbPos;
      if( ++yVbPos == vbCTUHeight )
      {
        yVbPos = 0;
      }
      return distance;
    };

    int VbDistance[STEP_Y];
    VbDistance[0] = calculateNextVbPosDist();
    VbDistance[1] = calculateNextVbPosDist();
    VbDistance[2] = calculateNextVbPosDist();
    VbDistance[3] = calculateNextVbPosDist();
    const bool foldingRequired =
        ( VbDistance[0] >= ALF_7x7_VB_FOLD_MIN_DIST && VbDistance[0] <= ALF_7x7_VB_FOLD_MAX_DIST ) ||
        ( VbDistance[1] >= ALF_7x7_VB_FOLD_MIN_DIST && VbDistance[1] <= ALF_7x7_VB_FOLD_MAX_DIST ) ||
        ( VbDistance[2] >= ALF_7x7_VB_FOLD_MIN_DIST && VbDistance[2] <= ALF_7x7_VB_FOLD_MAX_DIST ) ||
        ( VbDistance[3] >= ALF_7x7_VB_FOLD_MIN_DIST && VbDistance[3] <= ALF_7x7_VB_FOLD_MAX_DIST );

    int cl_index = ( i / 4 ) * ( MAX_CU_SIZE / 4 );
    const Pel* pImg0 = src;
    Pel* pDst = dst;

    int j = width;
    do
    {
      const AlfClassifier& cl0 = classifier[cl_index + 0];
      const AlfClassifier& cl1 = classifier[cl_index + 1];

      const int index0 = cl0.classIdx * MAX_NUM_ALF_LUMA_COEFF;
      const int index1 = cl1.classIdx * MAX_NUM_ALF_LUMA_COEFF;

      int16x4_t coeff[MAX_NUM_ALF_LUMA_COEFF / 2];

      // Copy all 12 coeffs.
      load_shuffle_coeffs_neon( g_alf_shuffle_table[cl0.transposeIdx], g_alf_shuffle_table[cl1.transposeIdx],
                                filterSet + index0, filterSet + index1, coeff );

      int16x8_t dst0, dst1, dst2, dst3;
      if( foldingRequired )
      {
        dst0 = processALF7x7Row_neon<true>( pImg0 + 0 * srcStride, VbDistance[0], srcStride, coeff, clpRngMax );
        dst1 = processALF7x7Row_neon<true>( pImg0 + 1 * srcStride, VbDistance[1], srcStride, coeff, clpRngMax );
        dst2 = processALF7x7Row_neon<true>( pImg0 + 2 * srcStride, VbDistance[2], srcStride, coeff, clpRngMax );
        dst3 = processALF7x7Row_neon<true>( pImg0 + 3 * srcStride, VbDistance[3], srcStride, coeff, clpRngMax );
      }
      else
      {
        dst0 = processALF7x7Row_neon<false>( pImg0 + 0 * srcStride, 0, srcStride, coeff, clpRngMax );
        dst1 = processALF7x7Row_neon<false>( pImg0 + 1 * srcStride, 0, srcStride, coeff, clpRngMax );
        dst2 = processALF7x7Row_neon<false>( pImg0 + 2 * srcStride, 0, srcStride, coeff, clpRngMax );
        dst3 = processALF7x7Row_neon<false>( pImg0 + 3 * srcStride, 0, srcStride, coeff, clpRngMax );
      }

      vst1q_s16( pDst + 0 * dstStride, dst0 );
      vst1q_s16( pDst + 1 * dstStride, dst1 );
      vst1q_s16( pDst + 2 * dstStride, dst2 );
      vst1q_s16( pDst + 3 * dstStride, dst3 );

      cl_index += 2;
      pImg0 += STEP_X;
      pDst += STEP_X;
      j -= STEP_X;
    } while( j != 0 );

    src += srcStride * STEP_Y;
    dst += dstStride * STEP_Y;
    i += STEP_Y;
  } while( i < height );
}

template<>
void AdaptiveLoopFilter::_initAdaptiveLoopFilterARM<NEON>()
{
  m_filter7x7Blk[0] = Filter7x7Blk_neon;
}

} // namespace vvenc
#endif
//! \}

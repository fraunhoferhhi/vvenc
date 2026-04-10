/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

/** \file     RdCost_sve2.cpp
    \brief    RD cost computation class, SVE version
*/

#include <arm_neon.h>
#include <math.h>

#include "CommonLib/CommonDef.h"
#include "CommonLib/RdCost.h"
#include "neon/RdCost_neon.h"
#include "neon/permute_neon.h"
#include "neon/sum_neon.h"
#include "neon_sve2_bridge.h"
#include "sve/neon_sve_bridge.h"

#include <arm_sve.h>

namespace vvenc
{

#if ENABLE_SIMD_OPT_DIST && defined( TARGET_SIMD_ARM )

static const uint16_t kZip1DTbl[] = { 0, 2, 4, 6, 1, 3, 5, 7 };

Distortion xCalcHAD16x16_fast_sve2( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                                    const ptrdiff_t iStrideCur )
{
  int16x8_t m1[8], m2[8];

  for( int i = 0; i < 8; i++ )
  {
    int16x8_t r0 = vld1q_s16( piOrg );
    int16x8_t r1 = vld1q_s16( piCur );
    int16x8_t r2 = vld1q_s16( piOrg + iStrideOrg );
    int16x8_t r3 = vld1q_s16( piCur + iStrideCur );

    r0 = vaddq_s16( r0, r2 );
    r1 = vaddq_s16( r1, r3 );

    r2 = vld1q_s16( piOrg + 8 );
    r3 = vld1q_s16( piCur + 8 );

    int16x8_t r4 = vld1q_s16( piOrg + iStrideOrg + 8 );
    int16x8_t r5 = vld1q_s16( piCur + iStrideCur + 8 );

    r2 = vaddq_s16( r2, r4 );
    r3 = vaddq_s16( r3, r5 );

    r0 = pairwise_add_s16x8( r0, r2 );
    r1 = pairwise_add_s16x8( r1, r3 );

    r0 = vrshrq_n_s16( r0, 2 );
    r1 = vrshrq_n_s16( r1, 2 );

    m1[i] = vsubq_s16( r0, r1 ); // 11-bit

    piCur += iStrideCur * 2;
    piOrg += iStrideOrg * 2;
  }

  const uint16x8_t idxs = vld1q_u16( kZip1DTbl );

  // Horizontal.
  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] ); // 12-bit

  m1[0] = vvenc_svtbl_s16( m2[0], idxs );
  m1[1] = vvenc_svtbl_s16( m2[1], idxs );
  m1[2] = vvenc_svtbl_s16( m2[2], idxs );
  m1[3] = vvenc_svtbl_s16( m2[3], idxs );
  m1[4] = vvenc_svtbl_s16( m2[4], idxs );
  m1[5] = vvenc_svtbl_s16( m2[5], idxs );
  m1[6] = vvenc_svtbl_s16( m2[6], idxs );
  m1[7] = vvenc_svtbl_s16( m2[7], idxs );

  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] ); // 13-bit

  m1[0] = vvenc_svtbl_s16( m2[0], idxs );
  m1[1] = vvenc_svtbl_s16( m2[1], idxs );
  m1[2] = vvenc_svtbl_s16( m2[2], idxs );
  m1[3] = vvenc_svtbl_s16( m2[3], idxs );
  m1[4] = vvenc_svtbl_s16( m2[4], idxs );
  m1[5] = vvenc_svtbl_s16( m2[5], idxs );
  m1[6] = vvenc_svtbl_s16( m2[6], idxs );
  m1[7] = vvenc_svtbl_s16( m2[7], idxs );

  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] ); // 14-bit

  // Vertical.
  m1[0] = vaddq_s16( m2[0], m2[4] );
  m1[1] = vaddq_s16( m2[1], m2[5] );
  m1[2] = vaddq_s16( m2[2], m2[6] );
  m1[3] = vaddq_s16( m2[3], m2[7] );
  m1[4] = vsubq_s16( m2[0], m2[4] );
  m1[5] = vsubq_s16( m2[1], m2[5] );
  m1[6] = vsubq_s16( m2[2], m2[6] );
  m1[7] = vsubq_s16( m2[3], m2[7] ); // 15-bit

  // The last butterfly uses |x+y|+|x-y| = 2*max(|x|,|y|); we delay the "*2".
  // vabsq_s16 of -2^15 returns -2^15, which gets reinterpreted to 2^15.
  int16x8_t dcVec0 = vaddq_s16( m1[0], m1[2] );
  int16x8_t dcVec1 = vaddq_s16( m1[1], m1[3] );
  uint16x8_t r0 = vreinterpretq_u16_s16( vabsq_s16( dcVec0 ) );
  uint16x8_t r1 = vreinterpretq_u16_s16( vabsq_s16( dcVec1 ) );
  uint16x8_t r2 = vreinterpretq_u16_s16( vabdq_s16( m1[0], m1[2] ) );
  uint16x8_t r3 = vreinterpretq_u16_s16( vabdq_s16( m1[1], m1[3] ) );
  uint16x8_t r4 = vreinterpretq_u16_s16( vabsq_s16( vaddq_s16( m1[4], m1[6] ) ) );
  uint16x8_t r5 = vreinterpretq_u16_s16( vabsq_s16( vaddq_s16( m1[5], m1[7] ) ) );
  uint16x8_t r6 = vreinterpretq_u16_s16( vabdq_s16( m1[4], m1[6] ) );
  uint16x8_t r7 = vreinterpretq_u16_s16( vabdq_s16( m1[5], m1[7] ) ); // 16-bit

  const int32x4_t dcVec = vabsq_s32( vaddl_s16( vget_high_s16( dcVec0 ), vget_high_s16( dcVec1 ) ) );
  const uint32_t absDC = ( uint32_t )vgetq_lane_s32( dcVec, 3 );

  const uint16x8_t max0 = vmaxq_u16( r0, r1 );
  const uint16x8_t max1 = vmaxq_u16( r2, r3 );
  const uint16x8_t max2 = vmaxq_u16( r4, r5 );
  const uint16x8_t max3 = vmaxq_u16( r6, r7 );

  const uint32x4_t sum = horizontal_add_long_4d_u16x8( max0, max1, max2, max3 );

  uint32_t sad = horizontal_add_u32x4( sum );
  sad <<= 1; // Apply the deferred doubling from the last butterfly.
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( sad + 2 ) >> 2;

  return sad << 2;
}

Distortion xCalcHAD8x8_sve2( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                             const ptrdiff_t iStrideCur )
{
  int16x8_t m1[8], m2[8];

  for( int i = 0; i < 8; i++ )
  {
    const int16x8_t org = vld1q_s16( piOrg + iStrideOrg * i );
    const int16x8_t cur = vld1q_s16( piCur + iStrideCur * i );
    m1[i] = vsubq_s16( org, cur ); // 11-bit
  }

  const uint16x8_t idxs = vld1q_u16( kZip1DTbl );

  // Horizontal.
  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] ); // 12-bit

  m1[0] = vvenc_svtbl_s16( m2[0], idxs );
  m1[1] = vvenc_svtbl_s16( m2[1], idxs );
  m1[2] = vvenc_svtbl_s16( m2[2], idxs );
  m1[3] = vvenc_svtbl_s16( m2[3], idxs );
  m1[4] = vvenc_svtbl_s16( m2[4], idxs );
  m1[5] = vvenc_svtbl_s16( m2[5], idxs );
  m1[6] = vvenc_svtbl_s16( m2[6], idxs );
  m1[7] = vvenc_svtbl_s16( m2[7], idxs );

  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] ); // 13-bit

  m1[0] = vvenc_svtbl_s16( m2[0], idxs );
  m1[1] = vvenc_svtbl_s16( m2[1], idxs );
  m1[2] = vvenc_svtbl_s16( m2[2], idxs );
  m1[3] = vvenc_svtbl_s16( m2[3], idxs );
  m1[4] = vvenc_svtbl_s16( m2[4], idxs );
  m1[5] = vvenc_svtbl_s16( m2[5], idxs );
  m1[6] = vvenc_svtbl_s16( m2[6], idxs );
  m1[7] = vvenc_svtbl_s16( m2[7], idxs );

  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] ); // 14-bit

  // Vertical.
  m1[0] = vaddq_s16( m2[0], m2[4] );
  m1[1] = vaddq_s16( m2[1], m2[5] );
  m1[2] = vaddq_s16( m2[2], m2[6] );
  m1[3] = vaddq_s16( m2[3], m2[7] );
  m1[4] = vsubq_s16( m2[0], m2[4] );
  m1[5] = vsubq_s16( m2[1], m2[5] );
  m1[6] = vsubq_s16( m2[2], m2[6] );
  m1[7] = vsubq_s16( m2[3], m2[7] ); // 15-bit

  // The last butterfly uses |x+y|+|x-y| = 2*max(|x|,|y|); we delay the "*2".
  int16x8_t dcVec0 = vaddq_s16( m1[0], m1[2] );
  int16x8_t dcVec1 = vaddq_s16( m1[1], m1[3] );

  // vabsq_s16 of -2^15 returns -2^15, which gets reinterpreted to 2^15.
  uint16x8_t r0 = vreinterpretq_u16_s16( vabsq_s16( dcVec0 ) );
  uint16x8_t r1 = vreinterpretq_u16_s16( vabsq_s16( dcVec1 ) );
  uint16x8_t r2 = vreinterpretq_u16_s16( vabdq_s16( m1[0], m1[2] ) );
  uint16x8_t r3 = vreinterpretq_u16_s16( vabdq_s16( m1[1], m1[3] ) );
  uint16x8_t r4 = vreinterpretq_u16_s16( vabsq_s16( vaddq_s16( m1[4], m1[6] ) ) );
  uint16x8_t r5 = vreinterpretq_u16_s16( vabsq_s16( vaddq_s16( m1[5], m1[7] ) ) );
  uint16x8_t r6 = vreinterpretq_u16_s16( vabdq_s16( m1[4], m1[6] ) );
  uint16x8_t r7 = vreinterpretq_u16_s16( vabdq_s16( m1[5], m1[7] ) ); // 16-bit

  const int32x4_t dcVec = vabsq_s32( vaddl_s16( vget_high_s16( dcVec0 ), vget_high_s16( dcVec1 ) ) );
  const uint32_t absDC = ( uint32_t )vgetq_lane_s32( dcVec, 3 );

  const uint16x8_t max0 = vmaxq_u16( r0, r1 );
  const uint16x8_t max1 = vmaxq_u16( r2, r3 );
  const uint16x8_t max2 = vmaxq_u16( r4, r5 );
  const uint16x8_t max3 = vmaxq_u16( r6, r7 );

  const uint32x4_t sum = horizontal_add_long_4d_u16x8( max0, max1, max2, max3 );

  uint64_t sad = horizontal_add_long_u32x4( sum );
  sad <<= 1; // Apply the deferred doubling from the last butterfly.
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( sad + 2 ) >> 2;

  return sad;
}

static inline int16x8_t vvenc_vqtbl2q_s16_u8( int16x8_t a0, int16x8_t a1, uint8x16_t idx )
{
  uint8x16x2_t tbl;
  tbl.val[0] = vreinterpretq_u8_s16( a0 );
  tbl.val[1] = vreinterpretq_u8_s16( a1 );
  return vreinterpretq_s16_u8( vvenc_vqtbl2q_u8( tbl, idx ) );
}

static const uint8_t kTrnSwapTbl0[] = { 0, 1, 4, 5, 2, 3, 6, 7, 16, 17, 20, 21, 18, 19, 22, 23 };
static const uint8_t kTrnSwapTbl1[] = { 8, 9, 12, 13, 10, 11, 14, 15, 24, 25, 28, 29, 26, 27, 30, 31 };

Distortion xCalcHAD8x16_sve2( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                              const ptrdiff_t iStrideCur )
{
  int16x8_t m1[16], m2[16];
  int32x4_t m3[32];

  for( int k = 0; k < 16; k++ )
  {
    int16x8_t org = vld1q_s16( piOrg );
    int16x8_t cur = vld1q_s16( piCur );

    m2[k] = vsubq_s16( org, cur ); // 11-bit

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  const uint8x16_t idx0 = vld1q_u8( kTrnSwapTbl0 );
  const uint8x16_t idx1 = vld1q_u8( kTrnSwapTbl1 );

  // Vertical distance 8.
  m1[0] = vaddq_s16( m2[0], m2[8] );
  m1[1] = vaddq_s16( m2[1], m2[9] );
  m1[2] = vaddq_s16( m2[2], m2[10] );
  m1[3] = vaddq_s16( m2[3], m2[11] );
  m1[4] = vaddq_s16( m2[4], m2[12] );
  m1[5] = vaddq_s16( m2[5], m2[13] );
  m1[6] = vaddq_s16( m2[6], m2[14] );
  m1[7] = vaddq_s16( m2[7], m2[15] );
  m1[8] = vsubq_s16( m2[0], m2[8] );
  m1[9] = vsubq_s16( m2[1], m2[9] );
  m1[10] = vsubq_s16( m2[2], m2[10] );
  m1[11] = vsubq_s16( m2[3], m2[11] );
  m1[12] = vsubq_s16( m2[4], m2[12] );
  m1[13] = vsubq_s16( m2[5], m2[13] );
  m1[14] = vsubq_s16( m2[6], m2[14] );
  m1[15] = vsubq_s16( m2[7], m2[15] ); // 12-bit

  // Vertical distance 4.
  m2[0] = vaddq_s16( m1[0], m1[4] );
  m2[1] = vaddq_s16( m1[1], m1[5] );
  m2[2] = vaddq_s16( m1[2], m1[6] );
  m2[3] = vaddq_s16( m1[3], m1[7] );
  m2[4] = vsubq_s16( m1[0], m1[4] );
  m2[5] = vsubq_s16( m1[1], m1[5] );
  m2[6] = vsubq_s16( m1[2], m1[6] );
  m2[7] = vsubq_s16( m1[3], m1[7] );
  m2[8] = vaddq_s16( m1[8], m1[12] );
  m2[9] = vaddq_s16( m1[9], m1[13] );
  m2[10] = vaddq_s16( m1[10], m1[14] );
  m2[11] = vaddq_s16( m1[11], m1[15] );
  m2[12] = vsubq_s16( m1[8], m1[12] );
  m2[13] = vsubq_s16( m1[9], m1[13] );
  m2[14] = vsubq_s16( m1[10], m1[14] );
  m2[15] = vsubq_s16( m1[11], m1[15] ); // 13-bit

  // Vertical distance 2.
  m1[0] = vaddq_s16( m2[0], m2[2] );
  m1[1] = vaddq_s16( m2[1], m2[3] );
  m1[2] = vsubq_s16( m2[0], m2[2] );
  m1[3] = vsubq_s16( m2[1], m2[3] );
  m1[4] = vaddq_s16( m2[4], m2[6] );
  m1[5] = vaddq_s16( m2[5], m2[7] );
  m1[6] = vsubq_s16( m2[4], m2[6] );
  m1[7] = vsubq_s16( m2[5], m2[7] );
  m1[8] = vaddq_s16( m2[8], m2[10] );
  m1[9] = vaddq_s16( m2[9], m2[11] );
  m1[10] = vsubq_s16( m2[8], m2[10] );
  m1[11] = vsubq_s16( m2[9], m2[11] );
  m1[12] = vaddq_s16( m2[12], m2[14] );
  m1[13] = vaddq_s16( m2[13], m2[15] );
  m1[14] = vsubq_s16( m2[12], m2[14] );
  m1[15] = vsubq_s16( m2[13], m2[15] ); // 14-bit

  // Horizontal distance 1.
  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] );
  m2[8] = vvenc_cadd_s16<90>( m1[8], m1[8] );
  m2[9] = vvenc_cadd_s16<90>( m1[9], m1[9] );
  m2[10] = vvenc_cadd_s16<90>( m1[10], m1[10] );
  m2[11] = vvenc_cadd_s16<90>( m1[11], m1[11] );
  m2[12] = vvenc_cadd_s16<90>( m1[12], m1[12] );
  m2[13] = vvenc_cadd_s16<90>( m1[13], m1[13] );
  m2[14] = vvenc_cadd_s16<90>( m1[14], m1[14] );
  m2[15] = vvenc_cadd_s16<90>( m1[15], m1[15] ); // 15-bit

  m1[0] = vvenc_vqtbl2q_s16_u8( m2[0], m2[1], idx0 );
  m1[1] = vvenc_vqtbl2q_s16_u8( m2[0], m2[1], idx1 );
  m1[2] = vvenc_vqtbl2q_s16_u8( m2[2], m2[3], idx0 );
  m1[3] = vvenc_vqtbl2q_s16_u8( m2[2], m2[3], idx1 );
  m1[4] = vvenc_vqtbl2q_s16_u8( m2[4], m2[5], idx0 );
  m1[5] = vvenc_vqtbl2q_s16_u8( m2[4], m2[5], idx1 );
  m1[6] = vvenc_vqtbl2q_s16_u8( m2[6], m2[7], idx0 );
  m1[7] = vvenc_vqtbl2q_s16_u8( m2[6], m2[7], idx1 );
  m1[8] = vvenc_vqtbl2q_s16_u8( m2[8], m2[9], idx0 );
  m1[9] = vvenc_vqtbl2q_s16_u8( m2[8], m2[9], idx1 );
  m1[10] = vvenc_vqtbl2q_s16_u8( m2[10], m2[11], idx0 );
  m1[11] = vvenc_vqtbl2q_s16_u8( m2[10], m2[11], idx1 );
  m1[12] = vvenc_vqtbl2q_s16_u8( m2[12], m2[13], idx0 );
  m1[13] = vvenc_vqtbl2q_s16_u8( m2[12], m2[13], idx1 );
  m1[14] = vvenc_vqtbl2q_s16_u8( m2[14], m2[15], idx0 );
  m1[15] = vvenc_vqtbl2q_s16_u8( m2[14], m2[15], idx1 );

  // Horizontal distance 2.
  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] );
  m2[8] = vvenc_cadd_s16<90>( m1[8], m1[8] );
  m2[9] = vvenc_cadd_s16<90>( m1[9], m1[9] );
  m2[10] = vvenc_cadd_s16<90>( m1[10], m1[10] );
  m2[11] = vvenc_cadd_s16<90>( m1[11], m1[11] );
  m2[12] = vvenc_cadd_s16<90>( m1[12], m1[12] );
  m2[13] = vvenc_cadd_s16<90>( m1[13], m1[13] );
  m2[14] = vvenc_cadd_s16<90>( m1[14], m1[14] );
  m2[15] = vvenc_cadd_s16<90>( m1[15], m1[15] ); // 16-bit

  // Horizontal distance 4
  auto dcVec0 = vaddl_s16( vget_low_s16( m2[0] ), vget_low_s16( m2[1] ) );
  auto dcVec1 = vaddl_s16( vget_high_s16( m2[0] ), vget_high_s16( m2[1] ) );

  // The last butterfly uses |x+y|+|x-y| = 2*max(|x|,|y|); we delay the "*2".
  m3[0] = vabsq_s32( vaddl_s16( vget_low_s16( m2[0] ), vget_low_s16( m2[1] ) ) );
  m3[1] = vabdl_s16( vget_low_s16( m2[0] ), vget_low_s16( m2[1] ) );
  m3[2] = vabsq_s32( vaddl_s16( vget_high_s16( m2[0] ), vget_high_s16( m2[1] ) ) );
  m3[3] = vabdl_s16( vget_high_s16( m2[0] ), vget_high_s16( m2[1] ) );
  m3[4] = vabsq_s32( vaddl_s16( vget_low_s16( m2[2] ), vget_low_s16( m2[3] ) ) );
  m3[5] = vabdl_s16( vget_low_s16( m2[2] ), vget_low_s16( m2[3] ) );
  m3[6] = vabsq_s32( vaddl_s16( vget_high_s16( m2[2] ), vget_high_s16( m2[3] ) ) );
  m3[7] = vabdl_s16( vget_high_s16( m2[2] ), vget_high_s16( m2[3] ) );
  m3[8] = vabsq_s32( vaddl_s16( vget_low_s16( m2[4] ), vget_low_s16( m2[5] ) ) );
  m3[9] = vabdl_s16( vget_low_s16( m2[4] ), vget_low_s16( m2[5] ) );
  m3[10] = vabsq_s32( vaddl_s16( vget_high_s16( m2[4] ), vget_high_s16( m2[5] ) ) );
  m3[11] = vabdl_s16( vget_high_s16( m2[4] ), vget_high_s16( m2[5] ) );
  m3[12] = vabsq_s32( vaddl_s16( vget_low_s16( m2[6] ), vget_low_s16( m2[7] ) ) );
  m3[13] = vabdl_s16( vget_low_s16( m2[6] ), vget_low_s16( m2[7] ) );
  m3[14] = vabsq_s32( vaddl_s16( vget_high_s16( m2[6] ), vget_high_s16( m2[7] ) ) );
  m3[15] = vabdl_s16( vget_high_s16( m2[6] ), vget_high_s16( m2[7] ) );

  int32_t absDC = vgetq_lane_s32( vabsq_s32( vaddq_s32( dcVec0, dcVec1 ) ), 3 );

  // Vertical distance 1.
  int32x4_t max0 = vmaxq_s32( m3[0], m3[2] );
  int32x4_t max1 = vmaxq_s32( m3[1], m3[3] );
  int32x4_t max2 = vmaxq_s32( m3[4], m3[6] );
  int32x4_t max3 = vmaxq_s32( m3[5], m3[7] );
  int32x4_t max4 = vmaxq_s32( m3[8], m3[10] );
  int32x4_t max5 = vmaxq_s32( m3[9], m3[11] );
  int32x4_t max6 = vmaxq_s32( m3[12], m3[14] );
  int32x4_t max7 = vmaxq_s32( m3[13], m3[15] );

  m3[16] = vabsq_s32( vaddl_s16( vget_low_s16( m2[8] ), vget_low_s16( m2[9] ) ) );
  m3[17] = vabdl_s16( vget_low_s16( m2[8] ), vget_low_s16( m2[9] ) );
  m3[18] = vabsq_s32( vaddl_s16( vget_high_s16( m2[8] ), vget_high_s16( m2[9] ) ) );
  m3[19] = vabdl_s16( vget_high_s16( m2[8] ), vget_high_s16( m2[9] ) );
  m3[20] = vabsq_s32( vaddl_s16( vget_low_s16( m2[10] ), vget_low_s16( m2[11] ) ) );
  m3[21] = vabdl_s16( vget_low_s16( m2[10] ), vget_low_s16( m2[11] ) );
  m3[22] = vabsq_s32( vaddl_s16( vget_high_s16( m2[10] ), vget_high_s16( m2[11] ) ) );
  m3[23] = vabdl_s16( vget_high_s16( m2[10] ), vget_high_s16( m2[11] ) );
  m3[24] = vabsq_s32( vaddl_s16( vget_low_s16( m2[12] ), vget_low_s16( m2[13] ) ) );
  m3[25] = vabdl_s16( vget_low_s16( m2[12] ), vget_low_s16( m2[13] ) );
  m3[26] = vabsq_s32( vaddl_s16( vget_high_s16( m2[12] ), vget_high_s16( m2[13] ) ) );
  m3[27] = vabdl_s16( vget_high_s16( m2[12] ), vget_high_s16( m2[13] ) );
  m3[28] = vabsq_s32( vaddl_s16( vget_low_s16( m2[14] ), vget_low_s16( m2[15] ) ) );
  m3[29] = vabdl_s16( vget_low_s16( m2[14] ), vget_low_s16( m2[15] ) );
  m3[30] = vabsq_s32( vaddl_s16( vget_high_s16( m2[14] ), vget_high_s16( m2[15] ) ) );
  m3[31] = vabdl_s16( vget_high_s16( m2[14] ), vget_high_s16( m2[15] ) );

  int32x4_t max8 = vmaxq_s32( m3[16], m3[18] );
  int32x4_t max9 = vmaxq_s32( m3[17], m3[19] );
  int32x4_t max10 = vmaxq_s32( m3[20], m3[22] );
  int32x4_t max11 = vmaxq_s32( m3[21], m3[23] );
  int32x4_t max12 = vmaxq_s32( m3[24], m3[26] );
  int32x4_t max13 = vmaxq_s32( m3[25], m3[27] );
  int32x4_t max14 = vmaxq_s32( m3[28], m3[30] );
  int32x4_t max15 = vmaxq_s32( m3[29], m3[31] );

  const int32x4_t sum0 = horizontal_add_4d_s32x4( max0, max1, max2, max3 );
  const int32x4_t sum1 = horizontal_add_4d_s32x4( max4, max5, max6, max7 );
  const int32x4_t sum2 = horizontal_add_4d_s32x4( max8, max9, max10, max11 );
  const int32x4_t sum3 = horizontal_add_4d_s32x4( max12, max13, max14, max15 );

  const int32x4_t sum0123 = horizontal_add_4d_s32x4( sum0, sum1, sum2, sum3 );
  uint32_t sad = ( uint32_t )horizontal_add_s32x4( sum0123 );
  sad <<= 1; // Apply the deferred doubling from the last butterfly.
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( uint32_t )( ( ( double )sad / sqrt( 16.0 * 8.0 ) ) * 2.0 );

  return sad;
}

Distortion xCalcHAD16x8_sve2( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                              const ptrdiff_t iStrideCur )
{
  int16x8_t m1[16], m2[16];
  int32x4_t m3[32];

  for( int k = 0; k < 8; k++ )
  {
    int16x8_t r0_lo = vld1q_s16( piOrg + k * iStrideOrg );
    int16x8_t r0_hi = vld1q_s16( piOrg + k * iStrideOrg + 8 );
    int16x8_t r1_lo = vld1q_s16( piCur + k * iStrideCur );
    int16x8_t r1_hi = vld1q_s16( piCur + k * iStrideCur + 8 );

    m2[2 * k + 0] = vsubq_s16( r0_lo, r1_lo );
    m2[2 * k + 1] = vsubq_s16( r0_hi, r1_hi ); // 11-bit
  }

  const uint8x16_t idx0 = vld1q_u8( kTrnSwapTbl0 );
  const uint8x16_t idx1 = vld1q_u8( kTrnSwapTbl1 );

  // Vertical distance 4.
  m1[0] = vaddq_s16( m2[0], m2[8] );
  m1[1] = vaddq_s16( m2[1], m2[9] );
  m1[2] = vaddq_s16( m2[2], m2[10] );
  m1[3] = vaddq_s16( m2[3], m2[11] );
  m1[4] = vaddq_s16( m2[4], m2[12] );
  m1[5] = vaddq_s16( m2[5], m2[13] );
  m1[6] = vaddq_s16( m2[6], m2[14] );
  m1[7] = vaddq_s16( m2[7], m2[15] );
  m1[8] = vsubq_s16( m2[0], m2[8] );
  m1[9] = vsubq_s16( m2[1], m2[9] );
  m1[10] = vsubq_s16( m2[2], m2[10] );
  m1[11] = vsubq_s16( m2[3], m2[11] );
  m1[12] = vsubq_s16( m2[4], m2[12] );
  m1[13] = vsubq_s16( m2[5], m2[13] );
  m1[14] = vsubq_s16( m2[6], m2[14] );
  m1[15] = vsubq_s16( m2[7], m2[15] ); // 12-bit

  // Vertical distance 2.
  m2[0] = vaddq_s16( m1[0], m1[4] );
  m2[1] = vaddq_s16( m1[1], m1[5] );
  m2[2] = vaddq_s16( m1[2], m1[6] );
  m2[3] = vaddq_s16( m1[3], m1[7] );
  m2[4] = vsubq_s16( m1[0], m1[4] );
  m2[5] = vsubq_s16( m1[1], m1[5] );
  m2[6] = vsubq_s16( m1[2], m1[6] );
  m2[7] = vsubq_s16( m1[3], m1[7] );
  m2[8] = vaddq_s16( m1[8], m1[12] );
  m2[9] = vaddq_s16( m1[9], m1[13] );
  m2[10] = vaddq_s16( m1[10], m1[14] );
  m2[11] = vaddq_s16( m1[11], m1[15] );
  m2[12] = vsubq_s16( m1[8], m1[12] );
  m2[13] = vsubq_s16( m1[9], m1[13] );
  m2[14] = vsubq_s16( m1[10], m1[14] );
  m2[15] = vsubq_s16( m1[11], m1[15] ); // 13-bit

  // Vertical distance 1.
  m1[0] = vaddq_s16( m2[0], m2[2] );
  m1[1] = vaddq_s16( m2[1], m2[3] );
  m1[2] = vsubq_s16( m2[0], m2[2] );
  m1[3] = vsubq_s16( m2[1], m2[3] );
  m1[4] = vaddq_s16( m2[4], m2[6] );
  m1[5] = vaddq_s16( m2[5], m2[7] );
  m1[6] = vsubq_s16( m2[4], m2[6] );
  m1[7] = vsubq_s16( m2[5], m2[7] );
  m1[8] = vaddq_s16( m2[8], m2[10] );
  m1[9] = vaddq_s16( m2[9], m2[11] );
  m1[10] = vsubq_s16( m2[8], m2[10] );
  m1[11] = vsubq_s16( m2[9], m2[11] );
  m1[12] = vaddq_s16( m2[12], m2[14] );
  m1[13] = vaddq_s16( m2[13], m2[15] );
  m1[14] = vsubq_s16( m2[12], m2[14] );
  m1[15] = vsubq_s16( m2[13], m2[15] ); // 14-bit

  // Horizontal distance 1.
  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] );
  m2[8] = vvenc_cadd_s16<90>( m1[8], m1[8] );
  m2[9] = vvenc_cadd_s16<90>( m1[9], m1[9] );
  m2[10] = vvenc_cadd_s16<90>( m1[10], m1[10] );
  m2[11] = vvenc_cadd_s16<90>( m1[11], m1[11] );
  m2[12] = vvenc_cadd_s16<90>( m1[12], m1[12] );
  m2[13] = vvenc_cadd_s16<90>( m1[13], m1[13] );
  m2[14] = vvenc_cadd_s16<90>( m1[14], m1[14] );
  m2[15] = vvenc_cadd_s16<90>( m1[15], m1[15] ); // 15-bit

  m1[0] = vvenc_vqtbl2q_s16_u8( m2[0], m2[1], idx0 );
  m1[1] = vvenc_vqtbl2q_s16_u8( m2[0], m2[1], idx1 );
  m1[2] = vvenc_vqtbl2q_s16_u8( m2[2], m2[3], idx0 );
  m1[3] = vvenc_vqtbl2q_s16_u8( m2[2], m2[3], idx1 );
  m1[4] = vvenc_vqtbl2q_s16_u8( m2[4], m2[5], idx0 );
  m1[5] = vvenc_vqtbl2q_s16_u8( m2[4], m2[5], idx1 );
  m1[6] = vvenc_vqtbl2q_s16_u8( m2[6], m2[7], idx0 );
  m1[7] = vvenc_vqtbl2q_s16_u8( m2[6], m2[7], idx1 );
  m1[8] = vvenc_vqtbl2q_s16_u8( m2[8], m2[9], idx0 );
  m1[9] = vvenc_vqtbl2q_s16_u8( m2[8], m2[9], idx1 );
  m1[10] = vvenc_vqtbl2q_s16_u8( m2[10], m2[11], idx0 );
  m1[11] = vvenc_vqtbl2q_s16_u8( m2[10], m2[11], idx1 );
  m1[12] = vvenc_vqtbl2q_s16_u8( m2[12], m2[13], idx0 );
  m1[13] = vvenc_vqtbl2q_s16_u8( m2[12], m2[13], idx1 );
  m1[14] = vvenc_vqtbl2q_s16_u8( m2[14], m2[15], idx0 );
  m1[15] = vvenc_vqtbl2q_s16_u8( m2[14], m2[15], idx1 );

  // Horizontal distance 2.
  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] );
  m2[8] = vvenc_cadd_s16<90>( m1[8], m1[8] );
  m2[9] = vvenc_cadd_s16<90>( m1[9], m1[9] );
  m2[10] = vvenc_cadd_s16<90>( m1[10], m1[10] );
  m2[11] = vvenc_cadd_s16<90>( m1[11], m1[11] );
  m2[12] = vvenc_cadd_s16<90>( m1[12], m1[12] );
  m2[13] = vvenc_cadd_s16<90>( m1[13], m1[13] );
  m2[14] = vvenc_cadd_s16<90>( m1[14], m1[14] );
  m2[15] = vvenc_cadd_s16<90>( m1[15], m1[15] ); // 16-bit

  // Horizontal distance 4
  auto dcVec0 = vaddl_s16( vget_low_s16( m2[0] ), vget_low_s16( m2[1] ) );
  auto dcVec1 = vaddl_s16( vget_high_s16( m2[0] ), vget_high_s16( m2[1] ) );

  // The last butterfly uses |x+y|+|x-y| = 2*max(|x|,|y|); we delay the "*2".
  m3[0] = vabsq_s32( vaddl_s16( vget_low_s16( m2[0] ), vget_low_s16( m2[1] ) ) );
  m3[1] = vabdl_s16( vget_low_s16( m2[0] ), vget_low_s16( m2[1] ) );
  m3[2] = vabsq_s32( vaddl_s16( vget_high_s16( m2[0] ), vget_high_s16( m2[1] ) ) );
  m3[3] = vabdl_s16( vget_high_s16( m2[0] ), vget_high_s16( m2[1] ) );
  m3[4] = vabsq_s32( vaddl_s16( vget_low_s16( m2[2] ), vget_low_s16( m2[3] ) ) );
  m3[5] = vabdl_s16( vget_low_s16( m2[2] ), vget_low_s16( m2[3] ) );
  m3[6] = vabsq_s32( vaddl_s16( vget_high_s16( m2[2] ), vget_high_s16( m2[3] ) ) );
  m3[7] = vabdl_s16( vget_high_s16( m2[2] ), vget_high_s16( m2[3] ) );
  m3[8] = vabsq_s32( vaddl_s16( vget_low_s16( m2[4] ), vget_low_s16( m2[5] ) ) );
  m3[9] = vabdl_s16( vget_low_s16( m2[4] ), vget_low_s16( m2[5] ) );
  m3[10] = vabsq_s32( vaddl_s16( vget_high_s16( m2[4] ), vget_high_s16( m2[5] ) ) );
  m3[11] = vabdl_s16( vget_high_s16( m2[4] ), vget_high_s16( m2[5] ) );
  m3[12] = vabsq_s32( vaddl_s16( vget_low_s16( m2[6] ), vget_low_s16( m2[7] ) ) );
  m3[13] = vabdl_s16( vget_low_s16( m2[6] ), vget_low_s16( m2[7] ) );
  m3[14] = vabsq_s32( vaddl_s16( vget_high_s16( m2[6] ), vget_high_s16( m2[7] ) ) );
  m3[15] = vabdl_s16( vget_high_s16( m2[6] ), vget_high_s16( m2[7] ) );

  int32_t absDC = vgetq_lane_s32( vabsq_s32( vaddq_s32( dcVec0, dcVec1 ) ), 3 );

  // Horizontal distance 8.
  int32x4_t max0 = vmaxq_s32( m3[0], m3[2] );
  int32x4_t max1 = vmaxq_s32( m3[1], m3[3] );
  int32x4_t max2 = vmaxq_s32( m3[4], m3[6] );
  int32x4_t max3 = vmaxq_s32( m3[5], m3[7] );
  int32x4_t max4 = vmaxq_s32( m3[8], m3[10] );
  int32x4_t max5 = vmaxq_s32( m3[9], m3[11] );
  int32x4_t max6 = vmaxq_s32( m3[12], m3[14] );
  int32x4_t max7 = vmaxq_s32( m3[13], m3[15] );

  m3[16] = vabsq_s32( vaddl_s16( vget_low_s16( m2[8] ), vget_low_s16( m2[9] ) ) );
  m3[17] = vabdl_s16( vget_low_s16( m2[8] ), vget_low_s16( m2[9] ) );
  m3[18] = vabsq_s32( vaddl_s16( vget_high_s16( m2[8] ), vget_high_s16( m2[9] ) ) );
  m3[19] = vabdl_s16( vget_high_s16( m2[8] ), vget_high_s16( m2[9] ) );
  m3[20] = vabsq_s32( vaddl_s16( vget_low_s16( m2[10] ), vget_low_s16( m2[11] ) ) );
  m3[21] = vabdl_s16( vget_low_s16( m2[10] ), vget_low_s16( m2[11] ) );
  m3[22] = vabsq_s32( vaddl_s16( vget_high_s16( m2[10] ), vget_high_s16( m2[11] ) ) );
  m3[23] = vabdl_s16( vget_high_s16( m2[10] ), vget_high_s16( m2[11] ) );
  m3[24] = vabsq_s32( vaddl_s16( vget_low_s16( m2[12] ), vget_low_s16( m2[13] ) ) );
  m3[25] = vabdl_s16( vget_low_s16( m2[12] ), vget_low_s16( m2[13] ) );
  m3[26] = vabsq_s32( vaddl_s16( vget_high_s16( m2[12] ), vget_high_s16( m2[13] ) ) );
  m3[27] = vabdl_s16( vget_high_s16( m2[12] ), vget_high_s16( m2[13] ) );
  m3[28] = vabsq_s32( vaddl_s16( vget_low_s16( m2[14] ), vget_low_s16( m2[15] ) ) );
  m3[29] = vabdl_s16( vget_low_s16( m2[14] ), vget_low_s16( m2[15] ) );
  m3[30] = vabsq_s32( vaddl_s16( vget_high_s16( m2[14] ), vget_high_s16( m2[15] ) ) );
  m3[31] = vabdl_s16( vget_high_s16( m2[14] ), vget_high_s16( m2[15] ) );

  int32x4_t max8 = vmaxq_s32( m3[16], m3[18] );
  int32x4_t max9 = vmaxq_s32( m3[17], m3[19] );
  int32x4_t max10 = vmaxq_s32( m3[20], m3[22] );
  int32x4_t max11 = vmaxq_s32( m3[21], m3[23] );
  int32x4_t max12 = vmaxq_s32( m3[24], m3[26] );
  int32x4_t max13 = vmaxq_s32( m3[25], m3[27] );
  int32x4_t max14 = vmaxq_s32( m3[28], m3[30] );
  int32x4_t max15 = vmaxq_s32( m3[29], m3[31] );

  const int32x4_t sum0 = horizontal_add_4d_s32x4( max0, max1, max2, max3 );
  const int32x4_t sum1 = horizontal_add_4d_s32x4( max4, max5, max6, max7 );
  const int32x4_t sum2 = horizontal_add_4d_s32x4( max8, max9, max10, max11 );
  const int32x4_t sum3 = horizontal_add_4d_s32x4( max12, max13, max14, max15 );

  const int32x4_t sum0123 = horizontal_add_4d_s32x4( sum0, sum1, sum2, sum3 );
  uint32_t sad = ( uint32_t )horizontal_add_s32x4( sum0123 );
  sad <<= 1; // Apply the deferred doubling from the last butterfly.
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( uint32_t )( ( ( double )sad / sqrt( 16.0 * 8.0 ) ) * 2.0 );

  return sad;
}

Distortion xCalcHAD8x4_sve2( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                             const ptrdiff_t iStrideCur )
{
  int16x8_t m1[4], m2[4];

  for( int k = 0; k < 4; k++ )
  {
    int16x8_t org = vld1q_s16( piOrg );
    int16x8_t cur = vld1q_s16( piCur );

    m1[k] = vsubq_s16( org, cur ); // 11-bit

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  const uint16x8_t idxs = vld1q_u16( kZip1DTbl );

  // Horizontal.
  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] ); // 12-bit

  m1[0] = vvenc_svtbl_s16( m2[0], idxs );
  m1[1] = vvenc_svtbl_s16( m2[1], idxs );
  m1[2] = vvenc_svtbl_s16( m2[2], idxs );
  m1[3] = vvenc_svtbl_s16( m2[3], idxs );

  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] ); // 13-bit

  m1[0] = vvenc_svtbl_s16( m2[0], idxs );
  m1[1] = vvenc_svtbl_s16( m2[1], idxs );
  m1[2] = vvenc_svtbl_s16( m2[2], idxs );
  m1[3] = vvenc_svtbl_s16( m2[3], idxs );

  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] ); // 14-bit

  // The last butterfly uses |x+y|+|x-y| = 2*max(|x|,|y|); we delay the "*2".
  int16x8_t dcVec0 = vaddq_s16( m2[0], m2[2] );
  int16x8_t dcVec1 = vaddq_s16( m2[1], m2[3] );

  m1[0] = vabsq_s16( vaddq_s16( m2[0], m2[2] ) );
  m1[1] = vabsq_s16( vaddq_s16( m2[1], m2[3] ) );
  m1[2] = vabdq_s16( m2[0], m2[2] );
  m1[3] = vabdq_s16( m2[1], m2[3] ); // 15-bit

  const int32x4_t dcVec = vabsq_s32( vaddl_s16( vget_high_s16( dcVec0 ), vget_high_s16( dcVec1 ) ) );
  const uint32_t absDC = ( uint32_t )vgetq_lane_s32( dcVec, 3 );

  const uint16x8_t max0 = vmaxq_u16( vreinterpretq_u16_s16( m1[0] ), vreinterpretq_u16_s16( m1[1] ) );
  const uint16x8_t max1 = vmaxq_u16( vreinterpretq_u16_s16( m1[2] ), vreinterpretq_u16_s16( m1[3] ) );

  uint32_t sad = horizontal_add_long_u16x8( vaddq_u16( max0, max1 ) );
  sad <<= 1; // Apply the deferred doubling from the last butterfly.
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( uint32_t )( ( double )sad / sqrt( 4.0 * 8.0 ) * 2.0 );

  return sad;
}

static const uint16_t kZip1DHalfTbl[] = { 0, 2, 1, 3, 4, 6, 5, 7 };

Distortion xCalcHAD4x8_sve2( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                             const ptrdiff_t iStrideCur )
{
  int16x8_t m1[4], m2[4];
  int16x4_t diff[8];
  for( int k = 0; k < 8; k++ )
  {
    int16x4_t org = vld1_s16( piOrg );
    int16x4_t cur = vld1_s16( piCur );

    diff[k] = vsub_s16( org, cur ); // 11-bit

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  const uint16x8_t idxs = vld1q_u16( kZip1DHalfTbl );

  // Vertical distance 1.
  m1[0] = vcombine_s16( vadd_s16( diff[0], diff[1] ), vsub_s16( diff[0], diff[1] ) );
  m1[1] = vcombine_s16( vadd_s16( diff[2], diff[3] ), vsub_s16( diff[2], diff[3] ) );
  m1[2] = vcombine_s16( vadd_s16( diff[4], diff[5] ), vsub_s16( diff[4], diff[5] ) );
  m1[3] = vcombine_s16( vadd_s16( diff[6], diff[7] ), vsub_s16( diff[6], diff[7] ) ); // 12-bit

  // Horizontal distance 1.
  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] ); // 12-bit

  m1[0] = vvenc_svtbl_s16( m2[0], idxs );
  m1[1] = vvenc_svtbl_s16( m2[1], idxs );
  m1[2] = vvenc_svtbl_s16( m2[2], idxs );
  m1[3] = vvenc_svtbl_s16( m2[3], idxs );

  // Horizontal distance 2.
  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] ); // 13-bit

  // The last butterfly uses |x+y|+|x-y| = 2*max(|x|,|y|); we delay the "*2".
  int16x8_t dcVec0 = vaddq_s16( m2[0], m2[2] );
  int16x8_t dcVec1 = vaddq_s16( m2[1], m2[3] );

  // Vertical distance 4.
  m1[0] = vabsq_s16( vaddq_s16( m2[0], m2[2] ) );
  m1[1] = vabsq_s16( vaddq_s16( m2[1], m2[3] ) );
  m1[2] = vabdq_s16( m2[0], m2[2] );
  m1[3] = vabdq_s16( m2[1], m2[3] ); // 15-bit

  const int32x4_t dcVec = vabsq_s32( vaddl_s16( vget_low_s16( dcVec0 ), vget_low_s16( dcVec1 ) ) );
  const uint32_t absDC = ( uint32_t )vgetq_lane_s32( dcVec, 3 );

  const uint16x8_t max0 = vmaxq_u16( vreinterpretq_u16_s16( m1[0] ), vreinterpretq_u16_s16( m1[1] ) );
  const uint16x8_t max1 = vmaxq_u16( vreinterpretq_u16_s16( m1[2] ), vreinterpretq_u16_s16( m1[3] ) );

  uint32_t sad = horizontal_add_long_u16x8( vaddq_u16( max0, max1 ) );
  sad <<= 1; // Apply the deferred doubling from the last butterfly.
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( uint32_t )( ( double )sad / sqrt( 4.0 * 8.0 ) * 2.0 );

  return sad;
}

Distortion xGetHADs_generic_sve2( const DistParam& rcDtParam )
{
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  int iRows = rcDtParam.org.height;
  const int iCols = rcDtParam.org.width;
  const ptrdiff_t iStrideCur = rcDtParam.cur.stride;
  const ptrdiff_t iStrideOrg = rcDtParam.org.stride;

  CHECKD( rcDtParam.bitDepth > 10, "Only bit-depths of up to 10 bits supported!" );

  Distortion uiSum = 0;

  if( iCols > iRows && iCols % 16 == 0 && iRows % 8 == 0 )
  {
    do
    {
      for( int x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHAD16x8_sve2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
      iRows -= 8;
    } while( iRows != 0 );
  }
  else if( iCols < iRows && iRows % 16 == 0 && iCols % 8 == 0 )
  {
    do
    {
      for( int x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHAD8x16_sve2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 16 * iStrideOrg;
      piCur += 16 * iStrideCur;
      iRows -= 16;
    } while( iRows != 0 );
  }
  else if( iCols > iRows && iCols % 8 == 0 && iRows % 4 == 0 )
  {
    for( int x = 0; x < iCols; x += 8 )
    {
      uiSum += xCalcHAD8x4_sve2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
    }
  }
  else if( iCols < iRows && iRows % 8 == 0 && iCols % 4 == 0 )
  {
    do
    {
      uiSum += xCalcHAD4x8_sve2( piOrg, piCur, iStrideOrg, iStrideCur );

      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
      iRows -= 8;
    } while( iRows != 0 );
  }
  else if( iCols % 8 == 0 && iRows == iCols )
  {
    do
    {
      for( int x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHAD8x8_sve2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
      iRows -= 8;
    } while( iRows != 0 );
  }
  else if( iRows % 4 == 0 && iCols % 4 == 0 )
  {
    do
    {
      for( int x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHAD4x4_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 4 * iStrideOrg;
      piCur += 4 * iStrideCur;
      iRows -= 4;
    } while( iRows != 0 );
  }
  else
  {
    do
    {
      for( int x = 0; x < iCols; x += 2 )
      {
        uiSum += RdCost::xCalcHADs2x2( &piOrg[x], &piCur[x], ( int )iStrideOrg, ( int )iStrideCur );
      }
      piOrg += 2 * iStrideOrg;
      piCur += 2 * iStrideCur;
      iRows -= 2;
    } while( iRows != 0 );
  }

  return uiSum;
}

template<int width>
Distortion xGetHADs_sve2( const DistParam& rcDtParam )
{
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  int iRows = rcDtParam.org.height;
  const int iCols = width;
  const ptrdiff_t iStrideCur = rcDtParam.cur.stride;
  const ptrdiff_t iStrideOrg = rcDtParam.org.stride;

  CHECKD( iRows < 2 || iRows > 128 || iCols < 2 || iCols > 128 || iRows % 2 != 0 || iCols % 2 != 0,
          "Invalid row or column count!" );
  CHECKD( rcDtParam.bitDepth > 10, "Only bit-depths of up to 10 bits supported!" );

  Distortion uiSum = 0;

  if( iCols > iRows && iCols % 16 == 0 && iRows % 8 == 0 )
  {
    do
    {
      for( int x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHAD16x8_sve2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
      iRows -= 8;
    } while( iRows != 0 );
  }
  else if( iCols < iRows && iRows % 16 == 0 && iCols % 8 == 0 )
  {
    do
    {
      for( int x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHAD8x16_sve2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 16 * iStrideOrg;
      piCur += 16 * iStrideCur;
      iRows -= 16;
    } while( iRows != 0 );
  }
  else if( iCols > iRows && iCols % 8 == 0 && iRows % 4 == 0 )
  {
    for( int x = 0; x < iCols; x += 8 )
    {
      uiSum += xCalcHAD8x4_sve2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
    }
  }
  else if( iCols < iRows && iRows % 8 == 0 && iCols % 4 == 0 )
  {
    do
    {
      uiSum += xCalcHAD4x8_sve2( piOrg, piCur, iStrideOrg, iStrideCur );

      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
      iRows -= 8;
    } while( iRows != 0 );
  }
  else if( iCols % 8 == 0 && iRows == iCols )
  {
    do
    {
      for( int x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHAD8x8_sve2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
      iRows -= 8;
    } while( iRows != 0 );
  }
  else if( iRows % 4 == 0 && iCols % 4 == 0 )
  {
    do
    {
      for( int x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHAD4x4_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 4 * iStrideOrg;
      piCur += 4 * iStrideCur;
      iRows -= 4;
    } while( iRows != 0 );
  }
  else
  {
    do
    {
      for( int x = 0; x < iCols; x += 2 )
      {
        uiSum += RdCost::xCalcHADs2x2( &piOrg[x], &piCur[x], ( int )iStrideOrg, ( int )iStrideCur );
      }
      piOrg += 2 * iStrideOrg;
      piCur += 2 * iStrideCur;
      iRows -= 2;
    } while( iRows != 0 );
  }

  return uiSum;
}

template<int width>
Distortion xGetHADs_fast_sve2( const DistParam& rcDtParam )
{
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  int iRows = rcDtParam.org.height;
  int iCols = width;
  const ptrdiff_t iStrideCur = rcDtParam.cur.stride;
  const ptrdiff_t iStrideOrg = rcDtParam.org.stride;

  CHECKD( rcDtParam.bitDepth > 10, "Only bit-depths of up to 10 bits supported!" );
  CHECKD( iRows < 2 || iRows > 128 || iCols < 2 || iCols > 128 || iRows % 2 != 0 || iCols % 2 != 0,
          "Invalid row or column count!" );

  Distortion uiSum = 0;
  if( iCols % 32 == 0 && iRows == iCols )
  {
    do
    {
      for( int x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHAD16x16_fast_sve2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 16 * iStrideOrg;
      piCur += 16 * iStrideCur;
      iRows -= 16;
    } while( iRows != 0 );
  }
  else
  {
    return xGetHADs_sve2<width>( rcDtParam );
  }

  return uiSum;
}

template<>
void RdCost::_initRdCostARM<SVE2>()
{
  m_afpDistortFunc[0][DF_HAD] = xGetHADs_generic_sve2;
  m_afpDistortFunc[0][DF_HAD2] = xGetHADs_sve2<2>;
  m_afpDistortFunc[0][DF_HAD4] = xGetHADs_sve2<4>;
  m_afpDistortFunc[0][DF_HAD8] = xGetHADs_sve2<8>;
  m_afpDistortFunc[0][DF_HAD16] = xGetHADs_sve2<16>;
  m_afpDistortFunc[0][DF_HAD32] = xGetHADs_sve2<32>;
  m_afpDistortFunc[0][DF_HAD64] = xGetHADs_sve2<64>;
  m_afpDistortFunc[0][DF_HAD128] = xGetHADs_sve2<128>;

  m_afpDistortFunc[0][DF_HAD_fast] = xGetHADs_generic_sve2;
  m_afpDistortFunc[0][DF_HAD2_fast] = xGetHADs_sve2<2>;
  m_afpDistortFunc[0][DF_HAD4_fast] = xGetHADs_sve2<4>;
  m_afpDistortFunc[0][DF_HAD8_fast] = xGetHADs_sve2<8>;
  m_afpDistortFunc[0][DF_HAD16_fast] = xGetHADs_sve2<16>;
  m_afpDistortFunc[0][DF_HAD32_fast] = xGetHADs_fast_sve2<32>;
  m_afpDistortFunc[0][DF_HAD64_fast] = xGetHADs_fast_sve2<64>;
  m_afpDistortFunc[0][DF_HAD128_fast] = xGetHADs_fast_sve2<128>;
}

#endif // defined( TARGET_SIMD_ARM )

} // namespace vvenc

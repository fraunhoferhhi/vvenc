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

/** \file     RdCost_neon.cpp
    \brief    RD cost computation class, Neon version
*/

#include <cstdlib>
#include <math.h>
#include <limits>

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/RdCost.h"
#include "permute_neon.h"
#include "reverse_neon.h"
#include "sum_neon.h"
#include "transpose_neon.h"

namespace vvenc
{

#if ENABLE_SIMD_OPT_DIST && defined( TARGET_SIMD_ARM )

Distortion xCalcHAD16x16_fast_neon( const Pel* piOrg, const Pel* piCur, const int iStrideOrg, const int iStrideCur )
{
  int16x8_t m1[8], m2[8];
  int32x4_t m3[8];

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

    m2[i] = vsubq_s16( r0, r1 ); // 11-bit

    piCur += iStrideCur * 2;
    piOrg += iStrideOrg * 2;
  }

  // Vertical.
  m1[0] = vaddq_s16( m2[0], m2[4] );
  m1[1] = vaddq_s16( m2[1], m2[5] );
  m1[2] = vaddq_s16( m2[2], m2[6] );
  m1[3] = vaddq_s16( m2[3], m2[7] );
  m1[4] = vsubq_s16( m2[0], m2[4] );
  m1[5] = vsubq_s16( m2[1], m2[5] );
  m1[6] = vsubq_s16( m2[2], m2[6] );
  m1[7] = vsubq_s16( m2[3], m2[7] ); // 12-bit

  m2[0] = vaddq_s16( m1[0], m1[2] );
  m2[1] = vaddq_s16( m1[1], m1[3] );
  m2[2] = vsubq_s16( m1[0], m1[2] );
  m2[3] = vsubq_s16( m1[1], m1[3] );
  m2[4] = vaddq_s16( m1[4], m1[6] );
  m2[5] = vaddq_s16( m1[5], m1[7] );
  m2[6] = vsubq_s16( m1[4], m1[6] );
  m2[7] = vsubq_s16( m1[5], m1[7] ); // 13-bit

  m1[0] = vaddq_s16( m2[0], m2[1] );
  m1[1] = vsubq_s16( m2[0], m2[1] );
  m1[2] = vaddq_s16( m2[2], m2[3] );
  m1[3] = vsubq_s16( m2[2], m2[3] );
  m1[4] = vaddq_s16( m2[4], m2[5] );
  m1[5] = vsubq_s16( m2[4], m2[5] );
  m1[6] = vaddq_s16( m2[6], m2[7] );
  m1[7] = vsubq_s16( m2[6], m2[7] ); // 14-bit

  const uint32_t absDC = std::abs( horizontal_add_long_s16x8( m1[0] ) );

  // Partial transpose to do the first horizontal step.
  m2[0] = vtrnq_s16( m1[0], m1[1] ).val[0];
  m2[1] = vtrnq_s16( m1[0], m1[1] ).val[1];
  m2[2] = vtrnq_s16( m1[2], m1[3] ).val[0];
  m2[3] = vtrnq_s16( m1[2], m1[3] ).val[1];
  m2[4] = vtrnq_s16( m1[4], m1[5] ).val[0];
  m2[5] = vtrnq_s16( m1[4], m1[5] ).val[1];
  m2[6] = vtrnq_s16( m1[6], m1[7] ).val[0];
  m2[7] = vtrnq_s16( m1[6], m1[7] ).val[1];

  // First horizontal butterfly.
  m3[0] = vreinterpretq_s32_s16( vaddq_s16( m2[0], m2[1] ) );
  m3[1] = vreinterpretq_s32_s16( vsubq_s16( m2[0], m2[1] ) );
  m3[2] = vreinterpretq_s32_s16( vaddq_s16( m2[2], m2[3] ) );
  m3[3] = vreinterpretq_s32_s16( vsubq_s16( m2[2], m2[3] ) );
  m3[4] = vreinterpretq_s32_s16( vaddq_s16( m2[4], m2[5] ) );
  m3[5] = vreinterpretq_s32_s16( vsubq_s16( m2[4], m2[5] ) );
  m3[6] = vreinterpretq_s32_s16( vaddq_s16( m2[6], m2[7] ) );
  m3[7] = vreinterpretq_s32_s16( vsubq_s16( m2[6], m2[7] ) ); // 15-bit

  m1[0] = vreinterpretq_s16_s32( vzipq_s32( m3[0], m3[1] ).val[0] );
  m1[1] = vreinterpretq_s16_s32( vzipq_s32( m3[0], m3[1] ).val[1] );
  m1[2] = vreinterpretq_s16_s32( vzipq_s32( m3[2], m3[3] ).val[0] );
  m1[3] = vreinterpretq_s16_s32( vzipq_s32( m3[2], m3[3] ).val[1] );
  m1[4] = vreinterpretq_s16_s32( vzipq_s32( m3[4], m3[5] ).val[0] );
  m1[5] = vreinterpretq_s16_s32( vzipq_s32( m3[4], m3[5] ).val[1] );
  m1[6] = vreinterpretq_s16_s32( vzipq_s32( m3[6], m3[7] ).val[0] );
  m1[7] = vreinterpretq_s16_s32( vzipq_s32( m3[6], m3[7] ).val[1] );

  // vabs_s16 of -2^15 returns -2^15, which later gets reinterpreted to 2^15.
  m2[0] = vabsq_s16( vaddq_s16( m1[0], m1[1] ) );
  m2[1] = vabdq_s16( m1[0], m1[1] );
  m2[2] = vabsq_s16( vaddq_s16( m1[2], m1[3] ) );
  m2[3] = vabdq_s16( m1[2], m1[3] );
  m2[4] = vabsq_s16( vaddq_s16( m1[4], m1[5] ) );
  m2[5] = vabdq_s16( m1[4], m1[5] );
  m2[6] = vabsq_s16( vaddq_s16( m1[6], m1[7] ) );
  m2[7] = vabdq_s16( m1[6], m1[7] ); // 16-bit

  const uint16x8_t r0 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[0], m2[1] ).val[0] );
  const uint16x8_t r1 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[0], m2[1] ).val[1] );
  const uint16x8_t r2 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[2], m2[3] ).val[0] );
  const uint16x8_t r3 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[2], m2[3] ).val[1] );
  const uint16x8_t r4 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[4], m2[5] ).val[0] );
  const uint16x8_t r5 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[4], m2[5] ).val[1] );
  const uint16x8_t r6 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[6], m2[7] ).val[0] );
  const uint16x8_t r7 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[6], m2[7] ).val[1] );

  // The last butterfly uses |x+y|+|x-y| = 2*max(|x|,|y|); we delay the "*2".
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

Distortion xCalcHAD8x8_neon( const Pel* piOrg, const Pel* piCur, const int iStrideOrg, const int iStrideCur )
{
  int16x8_t m1[8], m2[8];
  int32x4_t m3[8];
  for( int i = 0; i < 8; i++ )
  {
    const int16x8_t org = vld1q_s16( piOrg + iStrideOrg * i );
    const int16x8_t cur = vld1q_s16( piCur + iStrideCur * i );
    m2[i] = vsubq_s16( org, cur ); // 11-bit
  }

  // Vertical Hadamard.
  m1[0] = vaddq_s16( m2[0], m2[4] );
  m1[1] = vaddq_s16( m2[1], m2[5] );
  m1[2] = vaddq_s16( m2[2], m2[6] );
  m1[3] = vaddq_s16( m2[3], m2[7] );
  m1[4] = vsubq_s16( m2[0], m2[4] );
  m1[5] = vsubq_s16( m2[1], m2[5] );
  m1[6] = vsubq_s16( m2[2], m2[6] );
  m1[7] = vsubq_s16( m2[3], m2[7] ); // 12-bit

  m2[0] = vaddq_s16( m1[0], m1[2] );
  m2[1] = vaddq_s16( m1[1], m1[3] );
  m2[2] = vsubq_s16( m1[0], m1[2] );
  m2[3] = vsubq_s16( m1[1], m1[3] );
  m2[4] = vaddq_s16( m1[4], m1[6] );
  m2[5] = vaddq_s16( m1[5], m1[7] );
  m2[6] = vsubq_s16( m1[4], m1[6] );
  m2[7] = vsubq_s16( m1[5], m1[7] ); // 13-bit

  m1[0] = vaddq_s16( m2[0], m2[1] );
  m1[1] = vsubq_s16( m2[0], m2[1] );
  m1[2] = vaddq_s16( m2[2], m2[3] );
  m1[3] = vsubq_s16( m2[2], m2[3] );
  m1[4] = vaddq_s16( m2[4], m2[5] );
  m1[5] = vsubq_s16( m2[4], m2[5] );
  m1[6] = vaddq_s16( m2[6], m2[7] );
  m1[7] = vsubq_s16( m2[6], m2[7] ); // 14-bit

  const uint32_t absDC = std::abs( horizontal_add_long_s16x8( m1[0] ) );

  // Partial transpose to do the first horizontal step.
  m2[0] = vtrnq_s16( m1[0], m1[1] ).val[0];
  m2[1] = vtrnq_s16( m1[0], m1[1] ).val[1];
  m2[2] = vtrnq_s16( m1[2], m1[3] ).val[0];
  m2[3] = vtrnq_s16( m1[2], m1[3] ).val[1];
  m2[4] = vtrnq_s16( m1[4], m1[5] ).val[0];
  m2[5] = vtrnq_s16( m1[4], m1[5] ).val[1];
  m2[6] = vtrnq_s16( m1[6], m1[7] ).val[0];
  m2[7] = vtrnq_s16( m1[6], m1[7] ).val[1];

  // First horizontal butterfly.
  m3[0] = vreinterpretq_s32_s16( vaddq_s16( m2[0], m2[1] ) );
  m3[1] = vreinterpretq_s32_s16( vsubq_s16( m2[0], m2[1] ) );
  m3[2] = vreinterpretq_s32_s16( vaddq_s16( m2[2], m2[3] ) );
  m3[3] = vreinterpretq_s32_s16( vsubq_s16( m2[2], m2[3] ) );
  m3[4] = vreinterpretq_s32_s16( vaddq_s16( m2[4], m2[5] ) );
  m3[5] = vreinterpretq_s32_s16( vsubq_s16( m2[4], m2[5] ) );
  m3[6] = vreinterpretq_s32_s16( vaddq_s16( m2[6], m2[7] ) );
  m3[7] = vreinterpretq_s32_s16( vsubq_s16( m2[6], m2[7] ) ); // 15-bit

  m1[0] = vreinterpretq_s16_s32( vzipq_s32( m3[0], m3[1] ).val[0] );
  m1[1] = vreinterpretq_s16_s32( vzipq_s32( m3[0], m3[1] ).val[1] );
  m1[2] = vreinterpretq_s16_s32( vzipq_s32( m3[2], m3[3] ).val[0] );
  m1[3] = vreinterpretq_s16_s32( vzipq_s32( m3[2], m3[3] ).val[1] );
  m1[4] = vreinterpretq_s16_s32( vzipq_s32( m3[4], m3[5] ).val[0] );
  m1[5] = vreinterpretq_s16_s32( vzipq_s32( m3[4], m3[5] ).val[1] );
  m1[6] = vreinterpretq_s16_s32( vzipq_s32( m3[6], m3[7] ).val[0] );
  m1[7] = vreinterpretq_s16_s32( vzipq_s32( m3[6], m3[7] ).val[1] );

  // vabs_s16 of -2^15 returns -2^15, which later gets reinterpreted to 2^15.
  m2[0] = vabsq_s16( vaddq_s16( m1[0], m1[1] ) );
  m2[1] = vabdq_s16( m1[0], m1[1] );
  m2[2] = vabsq_s16( vaddq_s16( m1[2], m1[3] ) );
  m2[3] = vabdq_s16( m1[2], m1[3] );
  m2[4] = vabsq_s16( vaddq_s16( m1[4], m1[5] ) );
  m2[5] = vabdq_s16( m1[4], m1[5] );
  m2[6] = vabsq_s16( vaddq_s16( m1[6], m1[7] ) );
  m2[7] = vabdq_s16( m1[6], m1[7] ); // 16-bit

  const uint16x8_t r0 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[0], m2[1] ).val[0] );
  const uint16x8_t r1 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[0], m2[1] ).val[1] );
  const uint16x8_t r2 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[2], m2[3] ).val[0] );
  const uint16x8_t r3 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[2], m2[3] ).val[1] );
  const uint16x8_t r4 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[4], m2[5] ).val[0] );
  const uint16x8_t r5 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[4], m2[5] ).val[1] );
  const uint16x8_t r6 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[6], m2[7] ).val[0] );
  const uint16x8_t r7 = vreinterpretq_u16_s16( vvenc_vtrnq_s64_to_s16( m2[6], m2[7] ).val[1] );

  // The last butterfly uses |x+y|+|x-y| = 2*max(|x|,|y|); we delay the "*2".
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

  return sad;
}

Distortion xCalcHAD16x8_neon( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int16x8_t diff_s16[16];
  for( int k = 0; k < 8; k++ )
  {
    int16x8_t r0_lo = vld1q_s16( piOrg + k * iStrideOrg );
    int16x8_t r0_hi = vld1q_s16( piOrg + k * iStrideOrg + 8 );
    int16x8_t r1_lo = vld1q_s16( piCur + k * iStrideCur );
    int16x8_t r1_hi = vld1q_s16( piCur + k * iStrideCur + 8 );

    diff_s16[2 * k + 0] = vsubq_s16( r0_lo, r1_lo );
    diff_s16[2 * k + 1] = vsubq_s16( r0_hi, r1_hi ); // 11-bit
  }

  int16x8_t m1[16], m2[16];
  int32x4_t m3[16], m4[32];

  // Vertical.
  m2[0] = vaddq_s16( diff_s16[0], diff_s16[8] );
  m2[1] = vaddq_s16( diff_s16[1], diff_s16[9] );
  m2[2] = vaddq_s16( diff_s16[2], diff_s16[10] );
  m2[3] = vaddq_s16( diff_s16[3], diff_s16[11] );
  m2[4] = vaddq_s16( diff_s16[4], diff_s16[12] );
  m2[5] = vaddq_s16( diff_s16[5], diff_s16[13] );
  m2[6] = vaddq_s16( diff_s16[6], diff_s16[14] );
  m2[7] = vaddq_s16( diff_s16[7], diff_s16[15] );
  m2[8] = vsubq_s16( diff_s16[0], diff_s16[8] );
  m2[9] = vsubq_s16( diff_s16[1], diff_s16[9] );
  m2[10] = vsubq_s16( diff_s16[2], diff_s16[10] );
  m2[11] = vsubq_s16( diff_s16[3], diff_s16[11] );
  m2[12] = vsubq_s16( diff_s16[4], diff_s16[12] );
  m2[13] = vsubq_s16( diff_s16[5], diff_s16[13] );
  m2[14] = vsubq_s16( diff_s16[6], diff_s16[14] );
  m2[15] = vsubq_s16( diff_s16[7], diff_s16[15] ); // 12-bit

  m1[0] = vaddq_s16( m2[0], m2[4] );
  m1[1] = vaddq_s16( m2[1], m2[5] );
  m1[2] = vaddq_s16( m2[2], m2[6] );
  m1[3] = vaddq_s16( m2[3], m2[7] );
  m1[4] = vsubq_s16( m2[0], m2[4] );
  m1[5] = vsubq_s16( m2[1], m2[5] );
  m1[6] = vsubq_s16( m2[2], m2[6] );
  m1[7] = vsubq_s16( m2[3], m2[7] );
  m1[8] = vaddq_s16( m2[8], m2[12] );
  m1[9] = vaddq_s16( m2[9], m2[13] );
  m1[10] = vaddq_s16( m2[10], m2[14] );
  m1[11] = vaddq_s16( m2[11], m2[15] );
  m1[12] = vsubq_s16( m2[8], m2[12] );
  m1[13] = vsubq_s16( m2[9], m2[13] );
  m1[14] = vsubq_s16( m2[10], m2[14] );
  m1[15] = vsubq_s16( m2[11], m2[15] ); // 13-bit

  m2[0] = vaddq_s16( m1[0], m1[2] );
  m2[1] = vaddq_s16( m1[1], m1[3] );
  m2[2] = vsubq_s16( m1[0], m1[2] );
  m2[3] = vsubq_s16( m1[1], m1[3] );
  m2[4] = vaddq_s16( m1[4], m1[6] );
  m2[5] = vaddq_s16( m1[5], m1[7] );
  m2[6] = vsubq_s16( m1[4], m1[6] );
  m2[7] = vsubq_s16( m1[5], m1[7] );
  m2[8] = vaddq_s16( m1[8], m1[10] );
  m2[9] = vaddq_s16( m1[9], m1[11] );
  m2[12] = vsubq_s16( m1[8], m1[10] );
  m2[13] = vsubq_s16( m1[9], m1[11] );
  m2[10] = vaddq_s16( m1[12], m1[14] );
  m2[11] = vaddq_s16( m1[13], m1[15] );
  m2[14] = vsubq_s16( m1[12], m1[14] );
  m2[15] = vsubq_s16( m1[13], m1[15] ); // 14-bit

  // Horizontal.
  m1[0] = vaddq_s16( m2[0], m2[1] );
  m1[1] = vaddq_s16( m2[2], m2[3] );
  m1[2] = vaddq_s16( m2[4], m2[5] );
  m1[3] = vaddq_s16( m2[6], m2[7] );
  m1[4] = vaddq_s16( m2[8], m2[9] );
  m1[5] = vaddq_s16( m2[10], m2[11] );
  m1[6] = vaddq_s16( m2[12], m2[13] );
  m1[7] = vaddq_s16( m2[14], m2[15] );
  m1[8] = vsubq_s16( m2[0], m2[1] );
  m1[9] = vsubq_s16( m2[2], m2[3] );
  m1[10] = vsubq_s16( m2[4], m2[5] );
  m1[11] = vsubq_s16( m2[6], m2[7] );
  m1[12] = vsubq_s16( m2[8], m2[9] );
  m1[13] = vsubq_s16( m2[10], m2[11] );
  m1[14] = vsubq_s16( m2[12], m2[13] );
  m1[15] = vsubq_s16( m2[14], m2[15] ); // 15-bit

  const uint32_t absDC = std::abs( horizontal_add_long_s16x8( m1[0] ) );

  m2[0] = vtrnq_s16( m1[0], m1[1] ).val[0];
  m2[1] = vtrnq_s16( m1[0], m1[1] ).val[1];
  m2[2] = vtrnq_s16( m1[2], m1[3] ).val[0];
  m2[3] = vtrnq_s16( m1[2], m1[3] ).val[1];
  m2[4] = vtrnq_s16( m1[4], m1[5] ).val[0];
  m2[5] = vtrnq_s16( m1[4], m1[5] ).val[1];
  m2[6] = vtrnq_s16( m1[6], m1[7] ).val[0];
  m2[7] = vtrnq_s16( m1[6], m1[7] ).val[1];
  m2[8] = vtrnq_s16( m1[8], m1[9] ).val[0];
  m2[9] = vtrnq_s16( m1[8], m1[9] ).val[1];
  m2[10] = vtrnq_s16( m1[10], m1[11] ).val[0];
  m2[11] = vtrnq_s16( m1[10], m1[11] ).val[1];
  m2[12] = vtrnq_s16( m1[12], m1[13] ).val[0];
  m2[13] = vtrnq_s16( m1[12], m1[13] ).val[1];
  m2[14] = vtrnq_s16( m1[14], m1[15] ).val[0];
  m2[15] = vtrnq_s16( m1[14], m1[15] ).val[1];

  m3[0] = vreinterpretq_s32_s16( vaddq_s16( m2[0], m2[1] ) );
  m3[1] = vreinterpretq_s32_s16( vsubq_s16( m2[0], m2[1] ) );
  m3[2] = vreinterpretq_s32_s16( vaddq_s16( m2[2], m2[3] ) );
  m3[3] = vreinterpretq_s32_s16( vsubq_s16( m2[2], m2[3] ) );
  m3[4] = vreinterpretq_s32_s16( vaddq_s16( m2[4], m2[5] ) );
  m3[5] = vreinterpretq_s32_s16( vsubq_s16( m2[4], m2[5] ) );
  m3[6] = vreinterpretq_s32_s16( vaddq_s16( m2[6], m2[7] ) );
  m3[7] = vreinterpretq_s32_s16( vsubq_s16( m2[6], m2[7] ) );
  m3[8] = vreinterpretq_s32_s16( vaddq_s16( m2[8], m2[9] ) );
  m3[9] = vreinterpretq_s32_s16( vsubq_s16( m2[8], m2[9] ) );
  m3[10] = vreinterpretq_s32_s16( vaddq_s16( m2[10], m2[11] ) );
  m3[11] = vreinterpretq_s32_s16( vsubq_s16( m2[10], m2[11] ) );
  m3[12] = vreinterpretq_s32_s16( vaddq_s16( m2[12], m2[13] ) );
  m3[13] = vreinterpretq_s32_s16( vsubq_s16( m2[12], m2[13] ) );
  m3[14] = vreinterpretq_s32_s16( vaddq_s16( m2[14], m2[15] ) );
  m3[15] = vreinterpretq_s32_s16( vsubq_s16( m2[14], m2[15] ) ); // 16-bit

  m1[0] = vreinterpretq_s16_s32( vzipq_s32( m3[0], m3[1] ).val[0] );
  m1[1] = vreinterpretq_s16_s32( vzipq_s32( m3[0], m3[1] ).val[1] );
  m1[2] = vreinterpretq_s16_s32( vzipq_s32( m3[2], m3[3] ).val[0] );
  m1[3] = vreinterpretq_s16_s32( vzipq_s32( m3[2], m3[3] ).val[1] );
  m1[4] = vreinterpretq_s16_s32( vzipq_s32( m3[4], m3[5] ).val[0] );
  m1[5] = vreinterpretq_s16_s32( vzipq_s32( m3[4], m3[5] ).val[1] );
  m1[6] = vreinterpretq_s16_s32( vzipq_s32( m3[6], m3[7] ).val[0] );
  m1[7] = vreinterpretq_s16_s32( vzipq_s32( m3[6], m3[7] ).val[1] );
  m1[8] = vreinterpretq_s16_s32( vzipq_s32( m3[8], m3[9] ).val[0] );
  m1[9] = vreinterpretq_s16_s32( vzipq_s32( m3[8], m3[9] ).val[1] );
  m1[10] = vreinterpretq_s16_s32( vzipq_s32( m3[10], m3[11] ).val[0] );
  m1[11] = vreinterpretq_s16_s32( vzipq_s32( m3[10], m3[11] ).val[1] );
  m1[12] = vreinterpretq_s16_s32( vzipq_s32( m3[12], m3[13] ).val[0] );
  m1[13] = vreinterpretq_s16_s32( vzipq_s32( m3[12], m3[13] ).val[1] );
  m1[14] = vreinterpretq_s16_s32( vzipq_s32( m3[14], m3[15] ).val[0] );
  m1[15] = vreinterpretq_s16_s32( vzipq_s32( m3[14], m3[15] ).val[1] );

  m4[0] = vabsq_s32( vaddl_s16( vget_low_s16( m1[0] ), vget_low_s16( m1[1] ) ) );
  m4[1] = vabsq_s32( vaddl_s16( vget_high_s16( m1[0] ), vget_high_s16( m1[1] ) ) );
  m4[2] = vabdl_s16( vget_low_s16( m1[0] ), vget_low_s16( m1[1] ) );
  m4[3] = vabdl_s16( vget_high_s16( m1[0] ), vget_high_s16( m1[1] ) );
  m4[4] = vabsq_s32( vaddl_s16( vget_low_s16( m1[2] ), vget_low_s16( m1[3] ) ) );
  m4[5] = vabsq_s32( vaddl_s16( vget_high_s16( m1[2] ), vget_high_s16( m1[3] ) ) );
  m4[6] = vabdl_s16( vget_low_s16( m1[2] ), vget_low_s16( m1[3] ) );
  m4[7] = vabdl_s16( vget_high_s16( m1[2] ), vget_high_s16( m1[3] ) );
  m4[8] = vabsq_s32( vaddl_s16( vget_low_s16( m1[4] ), vget_low_s16( m1[5] ) ) );
  m4[9] = vabsq_s32( vaddl_s16( vget_high_s16( m1[4] ), vget_high_s16( m1[5] ) ) );
  m4[10] = vabdl_s16( vget_low_s16( m1[4] ), vget_low_s16( m1[5] ) );
  m4[11] = vabdl_s16( vget_high_s16( m1[4] ), vget_high_s16( m1[5] ) );
  m4[12] = vabsq_s32( vaddl_s16( vget_low_s16( m1[6] ), vget_low_s16( m1[7] ) ) );
  m4[13] = vabsq_s32( vaddl_s16( vget_high_s16( m1[6] ), vget_high_s16( m1[7] ) ) );
  m4[14] = vabdl_s16( vget_low_s16( m1[6] ), vget_low_s16( m1[7] ) );
  m4[15] = vabdl_s16( vget_high_s16( m1[6] ), vget_high_s16( m1[7] ) );
  m4[16] = vabsq_s32( vaddl_s16( vget_low_s16( m1[8] ), vget_low_s16( m1[9] ) ) );
  m4[17] = vabsq_s32( vaddl_s16( vget_high_s16( m1[8] ), vget_high_s16( m1[9] ) ) );
  m4[18] = vabdl_s16( vget_low_s16( m1[8] ), vget_low_s16( m1[9] ) );
  m4[19] = vabdl_s16( vget_high_s16( m1[8] ), vget_high_s16( m1[9] ) );
  m4[20] = vabsq_s32( vaddl_s16( vget_low_s16( m1[10] ), vget_low_s16( m1[11] ) ) );
  m4[21] = vabsq_s32( vaddl_s16( vget_high_s16( m1[10] ), vget_high_s16( m1[11] ) ) );
  m4[22] = vabdl_s16( vget_low_s16( m1[10] ), vget_low_s16( m1[11] ) );
  m4[23] = vabdl_s16( vget_high_s16( m1[10] ), vget_high_s16( m1[11] ) );
  m4[24] = vabsq_s32( vaddl_s16( vget_low_s16( m1[12] ), vget_low_s16( m1[13] ) ) );
  m4[25] = vabsq_s32( vaddl_s16( vget_high_s16( m1[12] ), vget_high_s16( m1[13] ) ) );
  m4[26] = vabdl_s16( vget_low_s16( m1[12] ), vget_low_s16( m1[13] ) );
  m4[27] = vabdl_s16( vget_high_s16( m1[12] ), vget_high_s16( m1[13] ) );
  m4[28] = vabsq_s32( vaddl_s16( vget_low_s16( m1[14] ), vget_low_s16( m1[15] ) ) );
  m4[29] = vabsq_s32( vaddl_s16( vget_high_s16( m1[14] ), vget_high_s16( m1[15] ) ) );
  m4[30] = vabdl_s16( vget_low_s16( m1[14] ), vget_low_s16( m1[15] ) );
  m4[31] = vabdl_s16( vget_high_s16( m1[14] ), vget_high_s16( m1[15] ) );

  const int32x4_t max0 = vmaxq_s32( m4[0], m4[1] );
  const int32x4_t max1 = vmaxq_s32( m4[2], m4[3] );
  const int32x4_t max2 = vmaxq_s32( m4[4], m4[5] );
  const int32x4_t max3 = vmaxq_s32( m4[6], m4[7] );
  const int32x4_t max4 = vmaxq_s32( m4[8], m4[9] );
  const int32x4_t max5 = vmaxq_s32( m4[10], m4[11] );
  const int32x4_t max6 = vmaxq_s32( m4[12], m4[13] );
  const int32x4_t max7 = vmaxq_s32( m4[14], m4[15] );
  const int32x4_t max8 = vmaxq_s32( m4[16], m4[17] );
  const int32x4_t max9 = vmaxq_s32( m4[18], m4[19] );
  const int32x4_t max10 = vmaxq_s32( m4[20], m4[21] );
  const int32x4_t max11 = vmaxq_s32( m4[22], m4[23] );
  const int32x4_t max12 = vmaxq_s32( m4[24], m4[25] );
  const int32x4_t max13 = vmaxq_s32( m4[26], m4[27] );
  const int32x4_t max14 = vmaxq_s32( m4[28], m4[29] );
  const int32x4_t max15 = vmaxq_s32( m4[30], m4[31] );

  const int32x4_t sum0 = horizontal_add_4d_s32x4( max0, max1, max2, max3 );
  const int32x4_t sum1 = horizontal_add_4d_s32x4( max4, max5, max6, max7 );
  const int32x4_t sum2 = horizontal_add_4d_s32x4( max8, max9, max10, max11 );
  const int32x4_t sum3 = horizontal_add_4d_s32x4( max12, max13, max14, max15 );

  int32x4_t sum0123 = horizontal_add_4d_s32x4( sum0, sum1, sum2, sum3 );

  uint32_t sad = ( uint32_t )horizontal_add_s32x4( sum0123 );
  sad <<= 1; // Apply the deferred doubling from the last butterfly.
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( int )( sad / sqrt( 16.0 * 8 ) * 2 );

  return sad;
}

static Distortion xCalcHAD8x16_neon( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int16x8_t diff_s16[16];
  for( int k = 0; k < 16; k++ )
  {
    int16x8_t org = vld1q_s16( piOrg );
    int16x8_t cur = vld1q_s16( piCur );

    diff_s16[k] = vsubq_s16( org, cur ); // 11-bit

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  int16x8_t m1[16], m2[16];

  // Vertical.
  m2[0] = vaddq_s16( diff_s16[0], diff_s16[8] );
  m2[1] = vaddq_s16( diff_s16[1], diff_s16[9] );
  m2[2] = vaddq_s16( diff_s16[2], diff_s16[10] );
  m2[3] = vaddq_s16( diff_s16[3], diff_s16[11] );
  m2[4] = vaddq_s16( diff_s16[4], diff_s16[12] );
  m2[5] = vaddq_s16( diff_s16[5], diff_s16[13] );
  m2[6] = vaddq_s16( diff_s16[6], diff_s16[14] );
  m2[7] = vaddq_s16( diff_s16[7], diff_s16[15] );
  m2[8] = vsubq_s16( diff_s16[0], diff_s16[8] );
  m2[9] = vsubq_s16( diff_s16[1], diff_s16[9] );
  m2[10] = vsubq_s16( diff_s16[2], diff_s16[10] );
  m2[11] = vsubq_s16( diff_s16[3], diff_s16[11] );
  m2[12] = vsubq_s16( diff_s16[4], diff_s16[12] );
  m2[13] = vsubq_s16( diff_s16[5], diff_s16[13] );
  m2[14] = vsubq_s16( diff_s16[6], diff_s16[14] );
  m2[15] = vsubq_s16( diff_s16[7], diff_s16[15] ); // 12-bit

  m1[0] = vaddq_s16( m2[0], m2[4] );
  m1[1] = vaddq_s16( m2[1], m2[5] );
  m1[2] = vaddq_s16( m2[2], m2[6] );
  m1[3] = vaddq_s16( m2[3], m2[7] );
  m1[4] = vsubq_s16( m2[0], m2[4] );
  m1[5] = vsubq_s16( m2[1], m2[5] );
  m1[6] = vsubq_s16( m2[2], m2[6] );
  m1[7] = vsubq_s16( m2[3], m2[7] );
  m1[8] = vaddq_s16( m2[8], m2[12] );
  m1[9] = vaddq_s16( m2[9], m2[13] );
  m1[10] = vaddq_s16( m2[10], m2[14] );
  m1[11] = vaddq_s16( m2[11], m2[15] );
  m1[12] = vsubq_s16( m2[8], m2[12] );
  m1[13] = vsubq_s16( m2[9], m2[13] );
  m1[14] = vsubq_s16( m2[10], m2[14] );
  m1[15] = vsubq_s16( m2[11], m2[15] ); // 13-bit

  m2[0] = vaddq_s16( m1[0], m1[2] );
  m2[1] = vaddq_s16( m1[1], m1[3] );
  m2[2] = vsubq_s16( m1[0], m1[2] );
  m2[3] = vsubq_s16( m1[1], m1[3] );
  m2[4] = vaddq_s16( m1[4], m1[6] );
  m2[5] = vaddq_s16( m1[5], m1[7] );
  m2[6] = vsubq_s16( m1[4], m1[6] );
  m2[7] = vsubq_s16( m1[5], m1[7] );
  m2[8] = vaddq_s16( m1[8], m1[10] );
  m2[9] = vaddq_s16( m1[9], m1[11] );
  m2[10] = vsubq_s16( m1[8], m1[10] );
  m2[11] = vsubq_s16( m1[9], m1[11] );
  m2[12] = vaddq_s16( m1[12], m1[14] );
  m2[13] = vaddq_s16( m1[13], m1[15] );
  m2[14] = vsubq_s16( m1[12], m1[14] );
  m2[15] = vsubq_s16( m1[13], m1[15] ); // 14-bit

  m1[0] = vaddq_s16( m2[0], m2[1] );
  m1[1] = vsubq_s16( m2[0], m2[1] );
  m1[2] = vaddq_s16( m2[2], m2[3] );
  m1[3] = vsubq_s16( m2[2], m2[3] );
  m1[4] = vaddq_s16( m2[4], m2[5] );
  m1[5] = vsubq_s16( m2[4], m2[5] );
  m1[6] = vaddq_s16( m2[6], m2[7] );
  m1[7] = vsubq_s16( m2[6], m2[7] );
  m1[8] = vaddq_s16( m2[8], m2[9] );
  m1[9] = vsubq_s16( m2[8], m2[9] );
  m1[10] = vaddq_s16( m2[10], m2[11] );
  m1[11] = vsubq_s16( m2[10], m2[11] );
  m1[12] = vaddq_s16( m2[12], m2[13] );
  m1[13] = vsubq_s16( m2[12], m2[13] );
  m1[14] = vaddq_s16( m2[14], m2[15] );
  m1[15] = vsubq_s16( m2[14], m2[15] ); // 15-bit

  // Transpose.
  m2[0] = vzipq_s16( m1[0], m1[4] ).val[0];
  m2[1] = vzipq_s16( m1[0], m1[4] ).val[1];
  m2[2] = vzipq_s16( m1[1], m1[5] ).val[0];
  m2[3] = vzipq_s16( m1[1], m1[5] ).val[1];
  m2[4] = vzipq_s16( m1[2], m1[6] ).val[0];
  m2[5] = vzipq_s16( m1[2], m1[6] ).val[1];
  m2[6] = vzipq_s16( m1[3], m1[7] ).val[0];
  m2[7] = vzipq_s16( m1[3], m1[7] ).val[1];
  m2[8] = vzipq_s16( m1[8], m1[12] ).val[0];
  m2[9] = vzipq_s16( m1[8], m1[12] ).val[1];
  m2[10] = vzipq_s16( m1[9], m1[13] ).val[0];
  m2[11] = vzipq_s16( m1[9], m1[13] ).val[1];
  m2[12] = vzipq_s16( m1[10], m1[14] ).val[0];
  m2[13] = vzipq_s16( m1[10], m1[14] ).val[1];
  m2[14] = vzipq_s16( m1[11], m1[15] ).val[0];
  m2[15] = vzipq_s16( m1[11], m1[15] ).val[1];

  m1[0] = vzipq_s16( m2[0], m2[4] ).val[0];
  m1[1] = vzipq_s16( m2[0], m2[4] ).val[1];
  m1[2] = vzipq_s16( m2[1], m2[5] ).val[0];
  m1[3] = vzipq_s16( m2[1], m2[5] ).val[1];
  m1[4] = vzipq_s16( m2[2], m2[6] ).val[0];
  m1[5] = vzipq_s16( m2[2], m2[6] ).val[1];
  m1[6] = vzipq_s16( m2[3], m2[7] ).val[0];
  m1[7] = vzipq_s16( m2[3], m2[7] ).val[1];
  m1[8] = vzipq_s16( m2[8], m2[12] ).val[0];
  m1[9] = vzipq_s16( m2[8], m2[12] ).val[1];
  m1[10] = vzipq_s16( m2[9], m2[13] ).val[0];
  m1[11] = vzipq_s16( m2[9], m2[13] ).val[1];
  m1[12] = vzipq_s16( m2[10], m2[14] ).val[0];
  m1[13] = vzipq_s16( m2[10], m2[14] ).val[1];
  m1[14] = vzipq_s16( m2[11], m2[15] ).val[0];
  m1[15] = vzipq_s16( m2[11], m2[15] ).val[1];

  m2[0] = vzipq_s16( m1[0], m1[4] ).val[0];
  m2[1] = vzipq_s16( m1[0], m1[4] ).val[1];
  m2[2] = vzipq_s16( m1[1], m1[5] ).val[0];
  m2[3] = vzipq_s16( m1[1], m1[5] ).val[1];
  m2[4] = vzipq_s16( m1[2], m1[6] ).val[0];
  m2[5] = vzipq_s16( m1[2], m1[6] ).val[1];
  m2[6] = vzipq_s16( m1[3], m1[7] ).val[0];
  m2[7] = vzipq_s16( m1[3], m1[7] ).val[1];
  m2[8] = vzipq_s16( m1[8], m1[12] ).val[0];
  m2[9] = vzipq_s16( m1[8], m1[12] ).val[1];
  m2[10] = vzipq_s16( m1[9], m1[13] ).val[0];
  m2[11] = vzipq_s16( m1[9], m1[13] ).val[1];
  m2[12] = vzipq_s16( m1[10], m1[14] ).val[0];
  m2[13] = vzipq_s16( m1[10], m1[14] ).val[1];
  m2[14] = vzipq_s16( m1[11], m1[15] ).val[0];
  m2[15] = vzipq_s16( m1[11], m1[15] ).val[1];

  int32x4_t m3[2][16], m4[2][16];

  // Horizontal.
  m3[0][0] = vaddl_s16( vget_low_s16( m2[0] ), vget_low_s16( m2[4] ) );
  m3[1][0] = vaddl_s16( vget_high_s16( m2[0] ), vget_high_s16( m2[4] ) );
  m3[0][1] = vaddl_s16( vget_low_s16( m2[1] ), vget_low_s16( m2[5] ) );
  m3[1][1] = vaddl_s16( vget_high_s16( m2[1] ), vget_high_s16( m2[5] ) );
  m3[0][2] = vaddl_s16( vget_low_s16( m2[2] ), vget_low_s16( m2[6] ) );
  m3[1][2] = vaddl_s16( vget_high_s16( m2[2] ), vget_high_s16( m2[6] ) );
  m3[0][3] = vaddl_s16( vget_low_s16( m2[3] ), vget_low_s16( m2[7] ) );
  m3[1][3] = vaddl_s16( vget_high_s16( m2[3] ), vget_high_s16( m2[7] ) );
  m3[0][4] = vsubl_s16( vget_low_s16( m2[0] ), vget_low_s16( m2[4] ) );
  m3[1][4] = vsubl_s16( vget_high_s16( m2[0] ), vget_high_s16( m2[4] ) );
  m3[0][5] = vsubl_s16( vget_low_s16( m2[1] ), vget_low_s16( m2[5] ) );
  m3[1][5] = vsubl_s16( vget_high_s16( m2[1] ), vget_high_s16( m2[5] ) );
  m3[0][6] = vsubl_s16( vget_low_s16( m2[2] ), vget_low_s16( m2[6] ) );
  m3[1][6] = vsubl_s16( vget_high_s16( m2[2] ), vget_high_s16( m2[6] ) );
  m3[0][7] = vsubl_s16( vget_low_s16( m2[3] ), vget_low_s16( m2[7] ) );
  m3[1][7] = vsubl_s16( vget_high_s16( m2[3] ), vget_high_s16( m2[7] ) );
  m3[0][8] = vaddl_s16( vget_low_s16( m2[8] ), vget_low_s16( m2[12] ) );
  m3[1][8] = vaddl_s16( vget_high_s16( m2[8] ), vget_high_s16( m2[12] ) );
  m3[0][9] = vaddl_s16( vget_low_s16( m2[9] ), vget_low_s16( m2[13] ) );
  m3[1][9] = vaddl_s16( vget_high_s16( m2[9] ), vget_high_s16( m2[13] ) );
  m3[0][10] = vaddl_s16( vget_low_s16( m2[10] ), vget_low_s16( m2[14] ) );
  m3[1][10] = vaddl_s16( vget_high_s16( m2[10] ), vget_high_s16( m2[14] ) );
  m3[0][11] = vaddl_s16( vget_low_s16( m2[11] ), vget_low_s16( m2[15] ) );
  m3[1][11] = vaddl_s16( vget_high_s16( m2[11] ), vget_high_s16( m2[15] ) );
  m3[0][12] = vsubl_s16( vget_low_s16( m2[8] ), vget_low_s16( m2[12] ) );
  m3[1][12] = vsubl_s16( vget_high_s16( m2[8] ), vget_high_s16( m2[12] ) );
  m3[0][13] = vsubl_s16( vget_low_s16( m2[9] ), vget_low_s16( m2[13] ) );
  m3[1][13] = vsubl_s16( vget_high_s16( m2[9] ), vget_high_s16( m2[13] ) );
  m3[0][14] = vsubl_s16( vget_low_s16( m2[10] ), vget_low_s16( m2[14] ) );
  m3[1][14] = vsubl_s16( vget_high_s16( m2[10] ), vget_high_s16( m2[14] ) );
  m3[0][15] = vsubl_s16( vget_low_s16( m2[11] ), vget_low_s16( m2[15] ) );
  m3[1][15] = vsubl_s16( vget_high_s16( m2[11] ), vget_high_s16( m2[15] ) );

  m4[0][0] = vaddq_s32( m3[0][0], m3[0][2] );
  m4[1][0] = vaddq_s32( m3[1][0], m3[1][2] );
  m4[0][2] = vsubq_s32( m3[0][0], m3[0][2] );
  m4[1][2] = vsubq_s32( m3[1][0], m3[1][2] );
  m4[0][1] = vaddq_s32( m3[0][1], m3[0][3] );
  m4[1][1] = vaddq_s32( m3[1][1], m3[1][3] );
  m4[0][3] = vsubq_s32( m3[0][1], m3[0][3] );
  m4[1][3] = vsubq_s32( m3[1][1], m3[1][3] );
  m4[0][4] = vaddq_s32( m3[0][4], m3[0][6] );
  m4[1][4] = vaddq_s32( m3[1][4], m3[1][6] );
  m4[0][6] = vsubq_s32( m3[0][4], m3[0][6] );
  m4[1][6] = vsubq_s32( m3[1][4], m3[1][6] );
  m4[0][5] = vaddq_s32( m3[0][5], m3[0][7] );
  m4[1][5] = vaddq_s32( m3[1][5], m3[1][7] );
  m4[0][7] = vsubq_s32( m3[0][5], m3[0][7] );
  m4[1][7] = vsubq_s32( m3[1][5], m3[1][7] );
  m4[0][8] = vaddq_s32( m3[0][8], m3[0][10] );
  m4[1][8] = vaddq_s32( m3[1][8], m3[1][10] );
  m4[0][10] = vsubq_s32( m3[0][8], m3[0][10] );
  m4[1][10] = vsubq_s32( m3[1][8], m3[1][10] );
  m4[0][9] = vaddq_s32( m3[0][9], m3[0][11] );
  m4[1][9] = vaddq_s32( m3[1][9], m3[1][11] );
  m4[0][11] = vsubq_s32( m3[0][9], m3[0][11] );
  m4[1][11] = vsubq_s32( m3[1][9], m3[1][11] );
  m4[0][12] = vaddq_s32( m3[0][12], m3[0][14] );
  m4[1][12] = vaddq_s32( m3[1][12], m3[1][14] );
  m4[0][14] = vsubq_s32( m3[0][12], m3[0][14] );
  m4[1][14] = vsubq_s32( m3[1][12], m3[1][14] );
  m4[0][13] = vaddq_s32( m3[0][13], m3[0][15] );
  m4[1][13] = vaddq_s32( m3[1][13], m3[1][15] );
  m4[0][15] = vsubq_s32( m3[0][13], m3[0][15] );
  m4[1][15] = vsubq_s32( m3[1][13], m3[1][15] );

  m3[0][0] = vabsq_s32( vaddq_s32( m4[0][0], m4[0][1] ) );
  m3[1][0] = vabsq_s32( vaddq_s32( m4[1][0], m4[1][1] ) );
  m3[0][1] = vabdq_s32( m4[0][0], m4[0][1] );
  m3[1][1] = vabdq_s32( m4[1][0], m4[1][1] );
  m3[0][2] = vabsq_s32( vaddq_s32( m4[0][2], m4[0][3] ) );
  m3[1][2] = vabsq_s32( vaddq_s32( m4[1][2], m4[1][3] ) );
  m3[0][3] = vabdq_s32( m4[0][2], m4[0][3] );
  m3[1][3] = vabdq_s32( m4[1][2], m4[1][3] );
  m3[0][4] = vabsq_s32( vaddq_s32( m4[0][4], m4[0][5] ) );
  m3[1][4] = vabsq_s32( vaddq_s32( m4[1][4], m4[1][5] ) );
  m3[0][5] = vabdq_s32( m4[0][4], m4[0][5] );
  m3[1][5] = vabdq_s32( m4[1][4], m4[1][5] );
  m3[0][6] = vabsq_s32( vaddq_s32( m4[0][6], m4[0][7] ) );
  m3[1][6] = vabsq_s32( vaddq_s32( m4[1][6], m4[1][7] ) );
  m3[0][7] = vabdq_s32( m4[0][6], m4[0][7] );
  m3[1][7] = vabdq_s32( m4[1][6], m4[1][7] );
  m3[0][8] = vabsq_s32( vaddq_s32( m4[0][8], m4[0][9] ) );
  m3[1][8] = vabsq_s32( vaddq_s32( m4[1][8], m4[1][9] ) );
  m3[0][9] = vabdq_s32( m4[0][8], m4[0][9] );
  m3[1][9] = vabdq_s32( m4[1][8], m4[1][9] );
  m3[0][10] = vabsq_s32( vaddq_s32( m4[0][10], m4[0][11] ) );
  m3[1][10] = vabsq_s32( vaddq_s32( m4[1][10], m4[1][11] ) );
  m3[0][11] = vabdq_s32( m4[0][10], m4[0][11] );
  m3[1][11] = vabdq_s32( m4[1][10], m4[1][11] );
  m3[0][12] = vabsq_s32( vaddq_s32( m4[0][12], m4[0][13] ) );
  m3[1][12] = vabsq_s32( vaddq_s32( m4[1][12], m4[1][13] ) );
  m3[0][13] = vabdq_s32( m4[0][12], m4[0][13] );
  m3[1][13] = vabdq_s32( m4[1][12], m4[1][13] );
  m3[0][14] = vabsq_s32( vaddq_s32( m4[0][14], m4[0][15] ) );
  m3[1][14] = vabsq_s32( vaddq_s32( m4[1][14], m4[1][15] ) );
  m3[0][15] = vabdq_s32( m4[0][14], m4[0][15] );
  m3[1][15] = vabdq_s32( m4[1][14], m4[1][15] );

  uint32_t absDC = vgetq_lane_s32( m3[0][0], 0 );

  m3[0][0] = vaddq_s32( m3[0][0], m3[0][1] );
  m3[0][2] = vaddq_s32( m3[0][2], m3[0][3] );
  m3[0][4] = vaddq_s32( m3[0][4], m3[0][5] );
  m3[0][6] = vaddq_s32( m3[0][6], m3[0][7] );
  m3[0][8] = vaddq_s32( m3[0][8], m3[0][9] );
  m3[0][10] = vaddq_s32( m3[0][10], m3[0][11] );
  m3[0][12] = vaddq_s32( m3[0][12], m3[0][13] );
  m3[0][14] = vaddq_s32( m3[0][14], m3[0][15] );
  m3[1][0] = vaddq_s32( m3[1][0], m3[1][1] );
  m3[1][2] = vaddq_s32( m3[1][2], m3[1][3] );
  m3[1][4] = vaddq_s32( m3[1][4], m3[1][5] );
  m3[1][6] = vaddq_s32( m3[1][6], m3[1][7] );
  m3[1][8] = vaddq_s32( m3[1][8], m3[1][9] );
  m3[1][10] = vaddq_s32( m3[1][10], m3[1][11] );
  m3[1][12] = vaddq_s32( m3[1][12], m3[1][13] );
  m3[1][14] = vaddq_s32( m3[1][14], m3[1][15] );

  m3[0][0] = vaddq_s32( m3[0][0], m3[0][2] );
  m3[0][4] = vaddq_s32( m3[0][4], m3[0][6] );
  m3[0][8] = vaddq_s32( m3[0][8], m3[0][10] );
  m3[0][12] = vaddq_s32( m3[0][12], m3[0][14] );
  m3[1][0] = vaddq_s32( m3[1][0], m3[1][2] );
  m3[1][4] = vaddq_s32( m3[1][4], m3[1][6] );
  m3[1][8] = vaddq_s32( m3[1][8], m3[1][10] );
  m3[1][12] = vaddq_s32( m3[1][12], m3[1][14] );

  m3[0][0] = vaddq_s32( m3[0][0], m3[0][4] );
  m3[0][8] = vaddq_s32( m3[0][8], m3[0][12] );
  m3[1][0] = vaddq_s32( m3[1][0], m3[1][4] );
  m3[1][8] = vaddq_s32( m3[1][8], m3[1][12] );

  m3[0][0] = vaddq_s32( m3[0][0], m3[0][8] );
  m3[1][0] = vaddq_s32( m3[1][0], m3[1][8] );

  m3[0][0] = vaddq_s32( m3[0][0], m3[1][0] );

  uint32_t sad = horizontal_add_s32x4( m3[0][0] );

  sad -= absDC;
  sad += absDC >> 2;
  sad = ( uint32_t )( ( ( double )sad / sqrt( 16.0 * 8.0 ) ) * 2.0 );

  return sad;
}

Distortion xCalcHAD8x4_neon( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int16x8_t diff[4];

  for( int k = 0; k < 4; k++ )
  {
    int16x8_t org = vld1q_s16( piOrg );
    int16x8_t cur = vld1q_s16( piCur );

    diff[k] = vsubq_s16( org, cur ); // 11-bit

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  int16x8_t m0[4], m1[4];
  int32x4_t m2[8];

  // Vertical.
  m0[0] = vaddq_s16( diff[0], diff[2] );
  m0[1] = vaddq_s16( diff[1], diff[3] );
  m0[2] = vsubq_s16( diff[0], diff[2] );
  m0[3] = vsubq_s16( diff[1], diff[3] ); // 12-bit

  m1[0] = vaddq_s16( m0[0], m0[1] );
  m1[1] = vsubq_s16( m0[0], m0[1] );
  m1[2] = vaddq_s16( m0[2], m0[3] );
  m1[3] = vsubq_s16( m0[2], m0[3] ); // 13-bit

  const uint32_t absDC = std::abs( horizontal_add_long_s16x8( m1[0] ) );

  // Partial transpose to do the first horizontal step.
  m0[0] = vtrnq_s16( m1[0], m1[1] ).val[0];
  m0[1] = vtrnq_s16( m1[0], m1[1] ).val[1];
  m0[2] = vtrnq_s16( m1[2], m1[3] ).val[0];
  m0[3] = vtrnq_s16( m1[2], m1[3] ).val[1];

  // First horizontal butterfly.
  m2[0] = vreinterpretq_s32_s16( vaddq_s16( m0[0], m0[1] ) );
  m2[1] = vreinterpretq_s32_s16( vsubq_s16( m0[0], m0[1] ) );
  m2[2] = vreinterpretq_s32_s16( vaddq_s16( m0[2], m0[3] ) );
  m2[3] = vreinterpretq_s32_s16( vsubq_s16( m0[2], m0[3] ) ); // 14-bit

  m1[0] = vreinterpretq_s16_s32( vzipq_s32( m2[0], m2[1] ).val[0] );
  m1[1] = vreinterpretq_s16_s32( vzipq_s32( m2[0], m2[1] ).val[1] );
  m1[2] = vreinterpretq_s16_s32( vzipq_s32( m2[2], m2[3] ).val[0] );
  m1[3] = vreinterpretq_s16_s32( vzipq_s32( m2[2], m2[3] ).val[1] );

  m0[0] = vabsq_s16( vaddq_s16( m1[0], m1[1] ) );
  m0[1] = vabdq_s16( m1[0], m1[1] );
  m0[2] = vabsq_s16( vaddq_s16( m1[2], m1[3] ) );
  m0[3] = vabdq_s16( m1[2], m1[3] ); // 15-bit

  // The last butterfly uses |x+y|+|x-y| = 2*max(|x|,|y|); we delay the "*2".
  m1[0] = vvenc_vtrnq_s64_to_s16( m0[0], m0[1] ).val[0];
  m1[1] = vvenc_vtrnq_s64_to_s16( m0[0], m0[1] ).val[1];
  m1[2] = vvenc_vtrnq_s64_to_s16( m0[2], m0[3] ).val[0];
  m1[3] = vvenc_vtrnq_s64_to_s16( m0[2], m0[3] ).val[1];

  const uint16x8_t max0 = vreinterpretq_u16_s16( vmaxq_s16( m1[0], m1[1] ) );
  const uint16x8_t max1 = vreinterpretq_u16_s16( vmaxq_s16( m1[2], m1[3] ) );

  uint32_t sad = ( uint32_t )horizontal_add_long_u16x8( vaddq_u16( max0, max1 ) );
  sad <<= 1; // Apply the deferred doubling from the last butterfly.
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( uint32_t )( ( double )sad / sqrt( 4.0 * 8.0 ) * 2.0 );

  return sad;
}

Distortion xCalcHAD4x8_neon( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur, int iBitDepth )
{
  int16x4_t diff[8];

  for( int k = 0; k < 8; k++ )
  {
    int16x4_t org = vld1_s16( piOrg );
    int16x4_t cur = vld1_s16( piCur );

    diff[k] = vsub_s16( org, cur ); // 11-bit

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  int16x8_t m1[8], m2[8];
  int32x4_t m3[4];

  // Vertical.
  m1[0] = vcombine_s16( vadd_s16( diff[0], diff[1] ), vsub_s16( diff[0], diff[1] ) );
  m1[1] = vcombine_s16( vadd_s16( diff[2], diff[3] ), vsub_s16( diff[2], diff[3] ) );
  m1[2] = vcombine_s16( vadd_s16( diff[4], diff[5] ), vsub_s16( diff[4], diff[5] ) );
  m1[3] = vcombine_s16( vadd_s16( diff[6], diff[7] ), vsub_s16( diff[6], diff[7] ) ); // 12-bit

  m2[0] = vaddq_s16( m1[0], m1[1] );
  m2[1] = vsubq_s16( m1[0], m1[1] );
  m2[2] = vaddq_s16( m1[2], m1[3] );
  m2[3] = vsubq_s16( m1[2], m1[3] ); // 13-bit

  const uint32_t absDC = std::abs( horizontal_add_long_s16x4( vget_low_s16( vaddq_s16( m2[0], m2[2] ) ) ) );

  m3[0] = vreinterpretq_s32_s16( vaddq_s16( m2[0], m2[2] ) );
  m3[1] = vreinterpretq_s32_s16( vaddq_s16( m2[1], m2[3] ) );
  m3[2] = vreinterpretq_s32_s16( vsubq_s16( m2[0], m2[2] ) );
  m3[3] = vreinterpretq_s32_s16( vsubq_s16( m2[1], m2[3] ) ); // 14-bit

  // Horizontal.
  m2[0] = vreinterpretq_s16_s32( vuzpq_s32( m3[0], m3[1] ).val[0] );
  m2[1] = vreinterpretq_s16_s32( vuzpq_s32( m3[0], m3[1] ).val[1] );
  m2[2] = vreinterpretq_s16_s32( vuzpq_s32( m3[2], m3[3] ).val[0] );
  m2[3] = vreinterpretq_s16_s32( vuzpq_s32( m3[2], m3[3] ).val[1] );

  m1[0] = vabsq_s16( vaddq_s16( m2[0], m2[1] ) );
  m1[1] = vabsq_s16( vaddq_s16( m2[2], m2[3] ) );
  m1[2] = vabdq_s16( m2[0], m2[1] );
  m1[3] = vabdq_s16( m2[2], m2[3] ); // 15-bit

  m2[0] = vtrnq_s16( m1[0], m1[1] ).val[0];
  m2[1] = vtrnq_s16( m1[0], m1[1] ).val[1];
  m2[2] = vtrnq_s16( m1[2], m1[3] ).val[0];
  m2[3] = vtrnq_s16( m1[2], m1[3] ).val[1];

  const uint16x8_t max0 = vreinterpretq_u16_s16( vmaxq_s16( m2[0], m2[1] ) );
  const uint16x8_t max1 = vreinterpretq_u16_s16( vmaxq_s16( m2[2], m2[3] ) );

  uint32_t sad = ( uint32_t )horizontal_add_long_u16x8( vaddq_u16( max0, max1 ) );
  sad <<= 1; // Apply the deferred doubling from the last butterfly.

  sad -= absDC;
  sad += absDC >> 2;
  sad = ( uint32_t )( ( double )sad / sqrt( 4.0 * 8.0 ) * 2.0 );

  return sad;
}

static const uint8_t k4x4PermuteTbl[] = { 6, 7, 4, 5, 22, 23, 20, 21, 14, 15, 12, 13, 30, 31, 28, 29 };

Distortion xCalcHAD4x4_neon( const Pel* piOrg, const Pel* piCur, const int iStrideOrg, const int iStrideCur )
{
  const int16x4_t r0 = vld1_s16( piOrg + 0 * iStrideOrg );
  const int16x4_t r1 = vld1_s16( piOrg + 1 * iStrideOrg );
  const int16x4_t r2 = vld1_s16( piOrg + 2 * iStrideOrg );
  const int16x4_t r3 = vld1_s16( piOrg + 3 * iStrideOrg );

  const int16x4_t c0 = vld1_s16( piCur + 0 * iStrideCur );
  const int16x4_t c1 = vld1_s16( piCur + 1 * iStrideCur );
  const int16x4_t c2 = vld1_s16( piCur + 2 * iStrideCur );
  const int16x4_t c3 = vld1_s16( piCur + 3 * iStrideCur );

  const int16x4_t diff0 = vsub_s16( r0, c0 );
  const int16x4_t diff1 = vsub_s16( r1, c1 );
  const int16x4_t diff2 = vsub_s16( r2, c2 );
  const int16x4_t diff3 = vsub_s16( r3, c3 );

  int16x8_t m1[2], m2[2];

  // Vertical.
  m2[0] = vcombine_s16( vadd_s16( diff0, diff3 ), vadd_s16( diff1, diff2 ) ); // 11-bit
  m2[1] = vcombine_s16( vsub_s16( diff0, diff3 ), vsub_s16( diff1, diff2 ) );

  m1[0] = vvenc_vtrnq_s64_to_s16( m2[0], m2[1] ).val[0];
  m1[1] = vvenc_vtrnq_s64_to_s16( m2[0], m2[1] ).val[1];

  m2[0] = vaddq_s16( m1[0], m1[1] );
  m2[1] = vsubq_s16( m1[0], m1[1] ); // 13-bit

  const int absDC = std::abs( horizontal_add_long_s16x4( vget_low_s16( m2[0] ) ) );

  const uint8x16_t idx0 = vld1q_u8( k4x4PermuteTbl );
  uint8x16x2_t tbl;
  tbl.val[0] = vreinterpretq_u8_s16( m2[0] );
  tbl.val[1] = vreinterpretq_u8_s16( m2[1] );

  m1[0] = vreinterpretq_s16_s32( vtrnq_s32( vreinterpretq_s32_s16( m2[0] ), vreinterpretq_s32_s16( m2[1] ) ).val[0] );
  m1[1] = vreinterpretq_s16_u8( vvenc_vqtbl2q_u8( tbl, idx0 ) );

  // The last butterfly uses |x+y|+|x-y| = 2*max(|x|,|y|); we delay the "*2".
  m2[0] = vabsq_s16( vaddq_s16( m1[0], m1[1] ) );
  m2[1] = vabdq_s16( m1[0], m1[1] ); // 14-bit

  m1[0] = vtrnq_s16( m2[0], m2[1] ).val[0];
  m1[1] = vtrnq_s16( m2[0], m2[1] ).val[1];

  int16x8_t max = vmaxq_s16( m1[0], m1[1] );

  int sad = horizontal_add_long_s16x8( max );
  sad <<= 1;
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( sad + 1 ) >> 1;

  return sad;
}

template<bool fastHad>
Distortion xGetHADs_neon( const DistParam& rcDtParam )
{
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int iRows = rcDtParam.org.height;
  const int iCols = rcDtParam.org.width;
  const int iStrideCur = rcDtParam.cur.stride;
  const int iStrideOrg = rcDtParam.org.stride;
  const int iBitDepth = rcDtParam.bitDepth;

  CHECKD( iBitDepth > 10, "Only bit-depths of up to 10 bits supported!" );

  int x, y;
  Distortion uiSum = 0;

  if( iCols > iRows && ( iCols & 15 ) == 0 && ( iRows & 7 ) == 0 )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHAD16x8_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
    }
  }
  else if( iCols < iRows && ( iRows & 15 ) == 0 && ( iCols & 7 ) == 0 )
  {
    for( y = 0; y < iRows; y += 16 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHAD8x16_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 16*iStrideOrg;
      piCur += 16*iStrideCur;
    }
  }
  else if( iCols > iRows && ( iCols & 7 ) == 0 && ( iRows & 3 ) == 0 )
  {
    for( y = 0; y < iRows; y += 4 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHAD8x4_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 4 * iStrideOrg;
      piCur += 4 * iStrideCur;
    }
  }
  else if( iCols < iRows && ( iRows & 7 ) == 0 && ( iCols & 3 ) == 0 )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHAD4x8_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
      }
      piOrg += 8*iStrideOrg;
      piCur += 8*iStrideCur;
    }
  }
  else if( fastHad && ( ( ( iRows | iCols ) & 31 ) == 0 ) && ( iRows == iCols ) )
  {
    for( y = 0; y < iRows; y += 16 )
    {
      for( x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHAD16x16_fast_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 16 * iStrideOrg;
      piCur += 16 * iStrideCur;
    }
  }
  else if( ( ( ( iRows | iCols ) & 7 ) == 0 ) && ( iRows == iCols ) )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHAD8x8_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
    }
  }
  else if( ( iRows % 4 == 0 ) && ( iCols % 4 == 0 ) )
  {
    for( y = 0; y < iRows; y += 4 )
    {
      for( x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHAD4x4_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 4*iStrideOrg;
      piCur += 4*iStrideCur;
    }
  }
  else if( ( iRows % 2 == 0 ) && ( iCols % 2 == 0 ) )
  {
    for( y = 0; y < iRows; y += 2 )
    {
      for( x = 0; x < iCols; x += 2 )
      {
        uiSum += RdCost::xCalcHADs2x2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 2*iStrideOrg;
      piCur += 2*iStrideCur;
    }
  }
  else
  {
    THROW( "Unsupported size" );
  }

  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

template<bool isCalCentrePos>
void xGetSADX5_16xN_neon_impl( const DistParam& rcDtParam, Distortion* cost )
{
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf - 4;
  int height = rcDtParam.org.height;
  constexpr int iSubShift = 1;
  constexpr int iSubStep = 1 << iSubShift;
  const ptrdiff_t iStrideCur = rcDtParam.cur.stride * iSubStep;
  const ptrdiff_t iStrideOrg = rcDtParam.org.stride * iSubStep;

  CHECKD( rcDtParam.subShift != 1, "Only SubShift = 1 is supported!" );
  CHECKD( rcDtParam.bitDepth > 10, "Only bit-depths of up to 10 bits supported!" );
  CHECKD( height <= 0, "Height cannot be <= 0!" );

  int16x8_t sum0 = vdupq_n_s16( 0 );
  int16x8_t sum1 = vdupq_n_s16( 0 );
  int16x8_t sum2 = vdupq_n_s16( 0 );
  int16x8_t sum3 = vdupq_n_s16( 0 );
  int16x8_t sum4 = vdupq_n_s16( 0 );

  do
  {
    int16x8_t org0 = vld1q_s16( piOrg + 0 );
    int16x8_t org1 = vld1q_s16( piOrg + 1 );
    int16x8_t org3 = vld1q_s16( piOrg + 3 );
    int16x8_t org4 = vld1q_s16( piOrg + 4 );

    int16x8_t org8 = vld1q_s16( piOrg + 8 );
    int16x8_t org9 = vld1q_s16( piOrg + 9 );
    int16x8_t org11 = vld1q_s16( piOrg + 11 );
    int16x8_t org12 = vld1q_s16( piOrg + 12 );

    int16x8_t cur0 = vld1q_s16( piCur + 0 );
    int16x8_t cur1 = vld1q_s16( piCur + 1 );
    int16x8_t cur3 = vld1q_s16( piCur + 3 );
    int16x8_t cur4 = vld1q_s16( piCur + 4 );

    int16x8_t cur8 = vld1q_s16( piCur + 8 );
    int16x8_t cur9 = vld1q_s16( piCur + 9 );
    int16x8_t cur11 = vld1q_s16( piCur + 11 );
    int16x8_t cur12 = vld1q_s16( piCur + 12 );

    sum0 = vabaq_s16( sum0, org0, cur4 );
    sum0 = vabaq_s16( sum0, org8, cur12 );

    sum1 = vabaq_s16( sum1, org1, cur3 );
    sum1 = vabaq_s16( sum1, org9, cur11 );

    if( isCalCentrePos )
    {
      int16x8_t org2 = vld1q_s16( piOrg + 2 );
      int16x8_t cur2 = vld1q_s16( piCur + 2 );

      int16x8_t org10 = vld1q_s16( piOrg + 10 );
      int16x8_t cur10 = vld1q_s16( piCur + 10 );

      sum2 = vabaq_s16( sum2, org2, cur2 );
      sum2 = vabaq_s16( sum2, org10, cur10 );
    }
    sum3 = vabaq_s16( sum3, org3, cur1 );
    sum3 = vabaq_s16( sum3, org11, cur9 );

    sum4 = vabaq_s16( sum4, org4, cur0 );
    sum4 = vabaq_s16( sum4, org12, cur8 );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
    height -= iSubStep;
  } while( height != 0 );

  int32x4_t sum = horizontal_add_long_4d_s16x8( sum0, sum1, sum3, sum4 );

  cost[0] = vgetq_lane_s32( sum, 0 );
  cost[1] = vgetq_lane_s32( sum, 1 );
  if( isCalCentrePos )
  {
    cost[2] = horizontal_add_long_s16x8( sum2 );
  }
  cost[3] = vgetq_lane_s32( sum, 2 );
  cost[4] = vgetq_lane_s32( sum, 3 );
}

void xGetSADX5_16xN_neon( const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos )
{
  if( rcDtParam.bitDepth > 10 )
  {
    RdCost::xGetSAD16X5( rcDtParam, cost, isCalCentrePos );
    return;
  }

  if( isCalCentrePos )
  {
    xGetSADX5_16xN_neon_impl<true>( rcDtParam, cost );
  }
  else
  {
    xGetSADX5_16xN_neon_impl<false>( rcDtParam, cost );
  }
}

static inline Distortion xGetSAD_generic_neon( const DistParam& rcDtParam, const int iCols )
{
  if( iCols < 4 )
  {
    return RdCost::xGetSAD( rcDtParam );
  }

  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  int iRows = rcDtParam.org.height;
  const int iSubShift = rcDtParam.subShift;
  const int iSubStep = 1 << iSubShift;
  const int iStrideCur = rcDtParam.cur.stride * iSubStep;
  const int iStrideOrg = rcDtParam.org.stride * iSubStep;

  // These checks ensure that height_limit is well-defined and non-zero.
  // The project enforces block dimensions <= 128 and power-of-two sizes,
  // which guarantees that delayed widening is safe under the supported
  // bit-depth constraints.
  CHECKD( rcDtParam.bitDepth > 10, "Only bit-depths of up to 10 bits supported!" );
  CHECKD( iRows > 128 || iCols > 128, "Exceeded MAX_CU_SIZE equal to 128!" );
  CHECKD( ( iCols & ( iCols - 1 ) ) != 0, "Width can only be power of two!" );
  CHECKD( ( iRows & ( iRows - 1 ) ) != 0, "Height can only be power of two!" );

  Distortion uiSum = 0;
  if( ( iCols & 15 ) == 0 )
  {
    uint32x4_t sum_u32_lo = vdupq_n_u32( 0 );
    uint32x4_t sum_u32_hi = vdupq_n_u32( 0 );

    int sampled_rows = iRows / iSubStep;
    // 32 guaranteed safe accumulation in u16 lanes; derived from bitDepth <= 10.
    const int height_limit = std::min( 32 * 16 / iCols, sampled_rows );
    if( height_limit > 4 )
    {
      // Partial chunk processing with height_limit for safe delayed widening.
      do
      {
        // u16 accumulators for delayed widening.
        uint16x8_t sum_u16_lo = vdupq_n_u16( 0 );
        uint16x8_t sum_u16_hi = vdupq_n_u16( 0 );

        int row = 0;
        do
        {
          int col = 0;
          do
          {
            const int16x8_t org_lo = vld1q_s16( piOrg + col + 0 );
            const int16x8_t org_hi = vld1q_s16( piOrg + col + 8 );
            const int16x8_t cur_lo = vld1q_s16( piCur + col + 0 );
            const int16x8_t cur_hi = vld1q_s16( piCur + col + 8 );

            sum_u16_lo = vvenc_vabaq_s16( sum_u16_lo, org_lo, cur_lo );
            sum_u16_hi = vvenc_vabaq_s16( sum_u16_hi, org_hi, cur_hi );

            col += 16;
          } while( col != iCols );

          piOrg += iStrideOrg;
          piCur += iStrideCur;
        } while( ++row != height_limit );

        // Delayed pairwise widen (u16 -> u32) and accumulate into 32-bit lanes.
        sum_u32_lo = vpadalq_u16( sum_u32_lo, sum_u16_lo );
        sum_u32_hi = vpadalq_u16( sum_u32_hi, sum_u16_hi );

        sampled_rows -= height_limit;
      } while( sampled_rows != 0 );
    }
    else
    {
      // Immediate widening/fallback path.
      do
      {
        int col = 0;
        do
        {
          const int16x8_t org_lo = vld1q_s16( piOrg + col + 0 );
          const int16x8_t org_hi = vld1q_s16( piOrg + col + 8 );
          const int16x8_t cur_lo = vld1q_s16( piCur + col + 0 );
          const int16x8_t cur_hi = vld1q_s16( piCur + col + 8 );

          const uint16x8_t abs_lo = vreinterpretq_u16_s16( vabdq_s16( org_lo, cur_lo ) );
          const uint16x8_t abs_hi = vreinterpretq_u16_s16( vabdq_s16( org_hi, cur_hi ) );
          sum_u32_lo = vpadalq_u16( sum_u32_lo, abs_lo );
          sum_u32_hi = vpadalq_u16( sum_u32_hi, abs_hi );

          col += 16;
        } while( col != iCols );

        piOrg += iStrideOrg;
        piCur += iStrideCur;
        iRows -= iSubStep;
      } while( iRows != 0 );
    }

    uiSum = horizontal_add_u32x4( vaddq_u32( sum_u32_lo, sum_u32_hi ) );
  }
  else if( iCols == 8 )
  {
    uint32x4_t sum_u32 = vdupq_n_u32( 0 );
    // Width is a power of two (8 here), allowing safe delayed widening
    // for (height / subStep) <= 32 without u16 overflow.
    if( iRows / iSubStep <= 32 )
    {
      uint16x8_t sum_u16 = vdupq_n_u16( 0 );
      do
      {
        const int16x8_t org = vld1q_s16( piOrg );
        const int16x8_t cur = vld1q_s16( piCur );

        const uint16x8_t abs = vreinterpretq_u16_s16( vabdq_s16( org, cur ) );
        sum_u16 = vaddq_u16( sum_u16, abs );

        piOrg += iStrideOrg;
        piCur += iStrideCur;
        iRows -= iSubStep;
      } while( iRows != 0 );

      sum_u32 = vpadalq_u16( sum_u32, sum_u16 );
    }
    else
    {
      do
      {
        const int16x8_t org = vld1q_s16( piOrg );
        const int16x8_t cur = vld1q_s16( piCur );

        const uint16x8_t abs = vreinterpretq_u16_s16( vabdq_s16( org, cur ) );
        sum_u32 = vpadalq_u16( sum_u32, abs );

        piOrg += iStrideOrg;
        piCur += iStrideCur;
        iRows -= iSubStep;
      } while( iRows != 0 );
    }

    uiSum = horizontal_add_u32x4( sum_u32 );
  }
  else
  {
    // Delayed widening is unnecessary in this path since there is no complex
    // accumulation pattern.
    CHECKD( iCols != 4, "iCols Must be equal to 4, got: " << iCols );
    uint32x4_t sum_u32 = vdupq_n_u32( 0 );
    do
    {
      const int16x4_t org = vld1_s16( piOrg );
      const int16x4_t cur = vld1_s16( piCur );

      const uint16x4_t abs = vreinterpret_u16_s16( vabd_s16( org, cur ) );
      sum_u32 = vaddw_u16( sum_u32, abs );

      piOrg += iStrideOrg;
      piCur += iStrideCur;
      iRows -= iSubStep;
    } while( iRows != 0 );

    uiSum = horizontal_add_u32x4( sum_u32 );
  }

  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth );
}

Distortion xGetHAD2SADs_neon( const DistParam& rcDtParam )
{
  Distortion distHad = xGetHADs_neon<false>( rcDtParam );
  Distortion distSad = xGetSAD_generic_neon( rcDtParam, rcDtParam.org.width );

  return std::min( distHad, 2 * distSad );
}

template<int iWidth>
Distortion xGetSAD_NxN_neon( const DistParam& rcDtParam )
{
  return xGetSAD_generic_neon( rcDtParam, iWidth );
}

Distortion xGetSAD_neon( const DistParam& rcDtParam )
{
  return xGetSAD_generic_neon( rcDtParam, rcDtParam.org.width );
}

Distortion xGetSADwMask_neon( const DistParam& rcDtParam )
{
  if (rcDtParam.org.width < 4 || rcDtParam.bitDepth > 10 || rcDtParam.applyWeight)
  {
    return RdCost::xGetSADwMask(rcDtParam);
  }

  const short *src1       = (const short *) rcDtParam.org.buf;
  const short *src2       = (const short *) rcDtParam.cur.buf;
  const short *weightMask = (const short *) rcDtParam.mask;
  int          rows       = rcDtParam.org.height;
  int          cols       = rcDtParam.org.width;
  int          subShift   = rcDtParam.subShift;
  int          subStep    = 1 << subShift;
  const int    strideSrc1 = rcDtParam.org.stride * subStep;
  const int    strideSrc2 = rcDtParam.cur.stride * subStep;
  const int    strideMask = rcDtParam.maskStride * subStep;

  int32x4_t sum0 = vdupq_n_s32( 0 );
  int32x4_t sum1 = vdupq_n_s32( 0 );

  do
  {
    int x = 0;
    do
    {
      int16x8_t vsrc1 = vld1q_s16( src1 + x );
      int16x8_t vsrc2 = vld1q_s16( src2 + x );
      int16x8_t vmask;
      if (rcDtParam.stepX == -1)
      {
        vmask = vld1q_s16( weightMask - x - 7 );
        vmask = reverse_vector_s16x8( vmask );
      }
      else
      {
        vmask = vld1q_s16( weightMask + x );
      }
      int16x8_t diff = vabdq_s16( vsrc1, vsrc2 );
      sum0 = vmlal_s16( sum0, vget_low_s16( diff ), vget_low_s16( vmask ) );
      sum1 = vmlal_s16( sum1, vget_high_s16( diff ), vget_high_s16( vmask ) );

      x += 8;
    } while( x != cols );

    src1 += strideSrc1;
    src2 += strideSrc2;
    weightMask += strideMask;
    rows -= subStep;
  } while( rows != 0 );

  Distortion sum = horizontal_add_s32x4( vaddq_s32( sum0, sum1 ) );
  sum <<= subShift;
  return sum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

static inline void calcWeightedMSE8( int16x8_t org, int16x8_t curr, int32x4_t fixedPTweight0, int32x4_t fixedPTweight1,
                                     int64x2_t* acc0, int64x2_t* acc1 )
{
  int32x4_t diff_lo = vabdl_s16( vget_low_s16( org ), vget_low_s16( curr ) );
  int32x4_t diff_hi = vabdl_s16( vget_high_s16( org ), vget_high_s16( curr ) );
  // ( a * ( b * b ) ) >> 16 rewritten as ( ( a * b ) * ( b << 15 ) ) >> 31
  int32x4_t s0 = vmulq_s32( fixedPTweight0, diff_lo );
  int32x4_t s1 = vmulq_s32( fixedPTweight1, diff_hi );
  s0 = vqrdmulhq_s32( s0, vshlq_n_s32( diff_lo, 15 ) );
  s1 = vqrdmulhq_s32( s1, vshlq_n_s32( diff_hi, 15 ) );
  *acc0 = vpadalq_s32( *acc0, s0 );
  *acc1 = vpadalq_s32( *acc1, s1 );
}

static inline void calcWeightedMSE4( int16x4_t org, int16x4_t curr, int32x4_t fixedPTweight, int64x2_t* acc )
{
  int32x4_t diff = vabdl_s16( org, curr );
  // ( a * ( b * b ) ) >> 16 rewritten as ( ( a * b ) * ( b << 15 ) ) >> 31
  int32x4_t s = vmulq_s32( fixedPTweight, diff );
  s = vqrdmulhq_s32( s, vshlq_n_s32( diff, 15 ) );
  *acc = vpadalq_s32( *acc, s );
}

template<int csx>
Distortion lumaWeightedSSE_neon( const DistParam& rcDtParam, ChromaFormat chmFmt, const uint32_t* lumaWeights )
{
  int iRows = rcDtParam.org.height;
  int iCols = rcDtParam.org.width;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int iStrideCur = rcDtParam.cur.stride;
  const int iStrideOrg = rcDtParam.org.stride;
  const Pel* piOrgLuma = rcDtParam.orgLuma->buf;
  const int iStrideOrgLuma = rcDtParam.orgLuma->stride;
  const ComponentID compId = rcDtParam.compID;

  CHECK( ( iCols & ( iCols - 1 ) ) != 0, "Width can only be power of two!" );

  const size_t cShiftY = getComponentScaleY( compId, chmFmt );
  Distortion uiSum = 0;

  if( iCols % 8 == 0 )
  {
    int64x2_t acc0 = vdupq_n_s64( 0 );
    int64x2_t acc1 = vdupq_n_s64( 0 );
    do
    {
      do
      {
        int16x8_t org = vld1q_s16( piOrg );  // 14 bit
        int16x8_t curr = vld1q_s16( piCur ); // 14 bit
        const uint32_t lweights[8] = { lumaWeights[piOrgLuma[0]],        lumaWeights[piOrgLuma[1 << csx]],
                                       lumaWeights[piOrgLuma[2 << csx]], lumaWeights[piOrgLuma[3 << csx]],
                                       lumaWeights[piOrgLuma[4 << csx]], lumaWeights[piOrgLuma[5 << csx]],
                                       lumaWeights[piOrgLuma[6 << csx]], lumaWeights[piOrgLuma[7 << csx]] };
        int32x4_t fixedPTweight0 = vreinterpretq_s32_u32( vld1q_u32( lweights + 0 ) ); // 17 bit
        int32x4_t fixedPTweight1 = vreinterpretq_s32_u32( vld1q_u32( lweights + 4 ) );

        calcWeightedMSE8( org, curr, fixedPTweight0, fixedPTweight1, &acc0, &acc1 );

        piOrg += 8;
        piCur += 8;
        piOrgLuma += 8 << csx;
        iCols -= 8;
      } while( iCols != 0 );

      iCols = rcDtParam.org.width;
      piOrg += iStrideOrg - iCols;
      piCur += iStrideCur - iCols;
      piOrgLuma += ( iStrideOrgLuma << cShiftY ) - ( iCols << csx );
    } while( --iRows != 0 );

    acc0 = vaddq_s64( acc0, acc1 );
    uiSum = ( Distortion )horizontal_add_s64x2( acc0 );
  }
  else if( iCols == 4 )
  {
    int64x2_t acc = vdupq_n_s64( 0 );
    do
    {
      int16x4_t org = vld1_s16( piOrg );  // 14 bit
      int16x4_t curr = vld1_s16( piCur ); // 14 bit
      const uint32_t lweights[4] = { lumaWeights[piOrgLuma[0]], lumaWeights[piOrgLuma[1 << csx]],
                                     lumaWeights[piOrgLuma[2 << csx]], lumaWeights[piOrgLuma[3 << csx]] };
      int32x4_t fixedPTweight = vreinterpretq_s32_u32( vld1q_u32( lweights ) ); // 17 bit

      calcWeightedMSE4( org, curr, fixedPTweight, &acc );

      piOrg += iStrideOrg;
      piCur += iStrideCur;
      piOrgLuma += iStrideOrgLuma << cShiftY;
    } while( --iRows != 0 );

    uiSum = ( Distortion )horizontal_add_s64x2( acc );
  }
  else if( iCols == 2 )
  {
    CHECK( iRows % 2, "Height must be a multiple of two" );
    int64x2_t acc = vdupq_n_s64( 0 );
    do
    {
      int16_t orgData[4] = { piOrg[0], piOrg[1], piOrg[iStrideOrg], piOrg[iStrideOrg + 1] };
      int16_t currData[4] = { piCur[0], piCur[1], piCur[iStrideCur], piCur[iStrideCur + 1] };
      int16x4_t org = vld1_s16( orgData );   // 14 bit
      int16x4_t curr = vld1_s16( currData ); // 14 bit
      const uint32_t lweights[4] = { lumaWeights[piOrgLuma[0]], lumaWeights[piOrgLuma[1 << csx]],
                                     lumaWeights[piOrgLuma[iStrideOrgLuma << cShiftY]],
                                     lumaWeights[piOrgLuma[( iStrideOrgLuma << cShiftY ) + ( 1 << csx )]] };
      int32x4_t fixedPTweight = vreinterpretq_s32_u32( vld1q_u32( lweights ) ); // 17 bit

      calcWeightedMSE4( org, curr, fixedPTweight, &acc );

      piOrg += iStrideOrg << 1;
      piCur += iStrideCur << 1;
      piOrgLuma += ( iStrideOrgLuma << ( cShiftY + 1 ) );
      iRows -= 2;
    } while( iRows != 0 );

    uiSum = ( Distortion )horizontal_add_s64x2( acc );
  }
  else if( iCols == 1 )
  {
    do
    {
      const int32_t iTemp = piOrg[0] - piCur[0];
      const uint32_t lweight = lumaWeights[piOrgLuma[0]];
      uiSum += ( ( int64_t )lweight * ( iTemp * iTemp ) + ( 1 << 15 ) ) >> 16;

      piOrg += iStrideOrg;
      piCur += iStrideCur;
      piOrgLuma += iStrideOrgLuma << cShiftY;
    } while( --iRows != 0 );
  }

  return uiSum;
}

Distortion fixWeightedSSE_neon( const DistParam& rcDtParam, uint32_t fixedPTweight )
{
  int iRows = rcDtParam.org.height;
  int iCols = rcDtParam.org.width;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int iStrideCur = rcDtParam.cur.stride;
  const int iStrideOrg = rcDtParam.org.stride;
  Distortion uiSum = 0;

  CHECK( ( iCols & ( iCols - 1 ) ) != 0, "Width can only be power of two!" );

  if( iCols % 8 == 0 )
  {
    int64x2_t acc0 = vdupq_n_s64( 0 );
    int64x2_t acc1 = vdupq_n_s64( 0 );
    int32x4_t fxpWeight = vdupq_n_s32( fixedPTweight ); // 17 bit
    do
    {
      do
      {
        int16x8_t org = vld1q_s16( piOrg );  // 14 bit
        int16x8_t curr = vld1q_s16( piCur ); // 14 bit

        calcWeightedMSE8( org, curr, fxpWeight, fxpWeight, &acc0, &acc1 );

        piOrg += 8;
        piCur += 8;
        iCols -= 8;
      } while( iCols != 0 );

      iCols = rcDtParam.org.width;
      piOrg += iStrideOrg - iCols;
      piCur += iStrideCur - iCols;
    } while( --iRows != 0 );

    acc0 = vaddq_s64( acc0, acc1 );
    uiSum = ( Distortion )horizontal_add_s64x2( acc0 );
  }
  else if( iCols == 4 )
  {
    int64x2_t acc = vdupq_n_s64( 0 );
    int32x4_t fxpWeight = vdupq_n_s32( fixedPTweight ); // 17 bit
    do
    {
      int16x4_t org = vld1_s16( piOrg );  // 14 bit
      int16x4_t curr = vld1_s16( piCur ); // 14 bit

      calcWeightedMSE4( org, curr, fxpWeight, &acc );

      piOrg += iStrideOrg;
      piCur += iStrideCur;
    } while( --iRows != 0 );

    uiSum = ( Distortion )horizontal_add_s64x2( acc );
  }
  else if( iCols == 2 )
  {
    CHECK( iRows % 2, "Height must be a multiple of two" );
    int64x2_t acc = vdupq_n_s64( 0 );
    int32x4_t fxpWeight = vdupq_n_s32( fixedPTweight ); // 17 bit
    do
    {
      int16_t orgData[4] = { piOrg[0], piOrg[1], piOrg[iStrideOrg], piOrg[iStrideOrg + 1] };
      int16_t currData[4] = { piCur[0], piCur[1], piCur[iStrideCur], piCur[iStrideCur + 1] };
      int16x4_t org = vld1_s16( orgData );   // 14 bit
      int16x4_t curr = vld1_s16( currData ); // 14 bit

      calcWeightedMSE4( org, curr, fxpWeight, &acc );

      piOrg += iStrideOrg << 1;
      piCur += iStrideCur << 1;
      iRows -= 2;
    } while( iRows != 0 );

    uiSum = ( Distortion )horizontal_add_s64x2( acc );
  }
  else if( iCols == 1 )
  {
    do
    {
      const int32_t iTemp = piOrg[0] - piCur[0];
      uiSum += ( ( int64_t )fixedPTweight * ( iTemp * iTemp ) + ( 1 << 15 ) ) >> 16;

      piOrg += iStrideOrg;
      piCur += iStrideCur;
    } while( --iRows != 0 );
  }

  return uiSum;
}

template<>
void RdCost::_initRdCostARM<NEON>()
{
  m_afpDistortFuncX5[1] = xGetSADX5_16xN_neon;

  m_afpDistortFunc[0][DF_SAD_WITH_MASK] = xGetSADwMask_neon;
  m_afpDistortFunc[0][DF_HAD_2SAD ] = xGetHAD2SADs_neon;

  m_afpDistortFunc[0][DF_HAD]     = xGetHADs_neon<false>;
  m_afpDistortFunc[0][DF_HAD2]    = xGetHADs_neon<false>;
  m_afpDistortFunc[0][DF_HAD4]    = xGetHADs_neon<false>;
  m_afpDistortFunc[0][DF_HAD8]    = xGetHADs_neon<false>;
  m_afpDistortFunc[0][DF_HAD16]   = xGetHADs_neon<false>;
  m_afpDistortFunc[0][DF_HAD32]   = xGetHADs_neon<false>;
  m_afpDistortFunc[0][DF_HAD64]   = xGetHADs_neon<false>;
  m_afpDistortFunc[0][DF_HAD128]  = xGetHADs_neon<false>;

  m_afpDistortFunc[0][DF_HAD_fast]     = xGetHADs_neon<true>;
  m_afpDistortFunc[0][DF_HAD2_fast]    = xGetHADs_neon<true>;
  m_afpDistortFunc[0][DF_HAD4_fast]    = xGetHADs_neon<true>;
  m_afpDistortFunc[0][DF_HAD8_fast]    = xGetHADs_neon<true>;
  m_afpDistortFunc[0][DF_HAD16_fast]   = xGetHADs_neon<true>;
  m_afpDistortFunc[0][DF_HAD32_fast]   = xGetHADs_neon<true>;
  m_afpDistortFunc[0][DF_HAD64_fast]   = xGetHADs_neon<true>;
  m_afpDistortFunc[0][DF_HAD128_fast]  = xGetHADs_neon<true>;

  m_afpDistortFunc[0][DF_SAD] = xGetSAD_neon;
  m_afpDistortFunc[0][DF_SAD4] = xGetSAD_NxN_neon<4>;
  m_afpDistortFunc[0][DF_SAD8] = xGetSAD_NxN_neon<8>;
  m_afpDistortFunc[0][DF_SAD16] = xGetSAD_NxN_neon<16>;
  m_afpDistortFunc[0][DF_SAD32] = xGetSAD_NxN_neon<32>;
  m_afpDistortFunc[0][DF_SAD64] = xGetSAD_NxN_neon<64>;
  m_afpDistortFunc[0][DF_SAD128] = xGetSAD_NxN_neon<128>;

  m_wtdPredPtr[0] = lumaWeightedSSE_neon<0>;
  m_wtdPredPtr[1] = lumaWeightedSSE_neon<1>;
  m_fxdWtdPredPtr = fixWeightedSSE_neon;
}

#endif  // defined( TARGET_SIMD_ARM )

}   // namespace vvenc

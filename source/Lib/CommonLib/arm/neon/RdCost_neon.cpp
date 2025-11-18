/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

#include <math.h>
#include <limits>

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/RdCost.h"
#include "reverse_neon.h"
#include "sum_neon.h"
#include "transpose_neon.h"

#if ENABLE_SIMD_OPT_DIST && defined( TARGET_SIMD_X86 )
#if SIMD_EVERYWHERE_EXTENSION_LEVEL_ID == X86_SIMD_AVX2
#define USE_AVX2
#elif SIMD_EVERYWHERE_EXTENSION_LEVEL_ID == X86_SIMD_SSE42
#define USE_SSE42
#elif SIMD_EVERYWHERE_EXTENSION_LEVEL_ID == X86_SIMD_SSE41
#define USE_SSE41
#endif
#endif

#if ENABLE_SIMD_OPT_DIST && defined( TARGET_SIMD_X86 )
# include "CommonLib/x86/RdCostX86.h"
#endif

namespace vvenc
{

#if ENABLE_SIMD_OPT_DIST && defined( TARGET_SIMD_ARM )

// The xGetHADs_neon functions depend on the SIMDe kernels being enabled
// during compilation.
#if ENABLE_SIMD_OPT_DIST && defined( TARGET_SIMD_X86 )

//working up to 12-bit
static uint32_t xCalcHAD16x16_fast_Neon( const Pel *piOrg, const Pel *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  int16x8x2_t m1[8], m2[8];
  int32x4x2_t m3[8], m4[8];

  CHECK( iBitDepth > 10, "Only bit-depths of up to 10 bits supported!" );

  //0
  int16x8_t r0, r1, r2, r3;
  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );
  r2 = vld1q_s16( ( piOrg + iStrideOrg ) );
  r3 = vld1q_s16( ( piCur + iStrideCur ) );

  r0 = vaddq_s16( r0, r2 );
  r1 = vaddq_s16( r1, r3 );

  r2 = vld1q_s16( ( piOrg + 8 ) );
  r3 = vld1q_s16( ( piCur + 8 ) );

  int16x8_t r4, r5;
  r4 = vld1q_s16( ( piOrg + iStrideOrg + 8 ) );
  r5 = vld1q_s16( ( piCur + iStrideCur + 8 ) );

  r2 = vaddq_s16( r2, r4 );
  r3 = vaddq_s16( r3, r5 );

  r0 = pairwise_add_s16x8( r0, r2 );
  r1 = pairwise_add_s16x8( r1, r3 );

  r0 = vaddq_s16( r0, vdupq_n_s16( 2 ) );
  r1 = vaddq_s16( r1, vdupq_n_s16( 2 ) );
  r0 = vshrq_n_s16( r0, 2 );
  r1 = vshrq_n_s16( r1, 2 );

  m2[0].val[0] = vsubq_s16( r0, r1 ); // 11bit
  //m2[1][k] = _mm_cvtepi16_epi32( _mm_srli_si128( m2[0][k], 8 ) );
  //m2[0][k] = _mm_cvtepi16_epi32( m2[0][k] );
  piCur += iStrideCur * 2;
  piOrg += iStrideOrg * 2;

  //1
  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );
  r2 = vld1q_s16( ( piOrg + iStrideOrg ) );
  r3 = vld1q_s16( ( piCur + iStrideCur ) );

  r0 = vaddq_s16( r0, r2 );
  r1 = vaddq_s16( r1, r3 );

  r2 = vld1q_s16( ( piOrg + 8 ) );
  r3 = vld1q_s16( ( piCur + 8 ) );

  r4 = vld1q_s16( ( piOrg + iStrideOrg + 8 ) );
  r5 = vld1q_s16( ( piCur + iStrideCur + 8 ) );

  r2 = vaddq_s16( r2, r4 );
  r3 = vaddq_s16( r3, r5 );

  r0 = pairwise_add_s16x8( r0, r2 );
  r1 = pairwise_add_s16x8( r1, r3 );

  r0 = vaddq_s16( r0, vdupq_n_s16( 2 ) );
  r1 = vaddq_s16( r1, vdupq_n_s16( 2 ) );
  r0 = vshrq_n_s16( r0, 2 );
  r1 = vshrq_n_s16( r1, 2 );

  m2[1].val[0] = vsubq_s16( r0, r1 ); // 11bit
  //m2[1][k] = _mm_cvtepi16_epi32( _mm_srli_si128( m2[0][k], 8 ) );
  //m2[0][k] = _mm_cvtepi16_epi32( m2[0][k] );
  piCur += iStrideCur * 2;
  piOrg += iStrideOrg * 2;

  //2
  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );
  r2 = vld1q_s16( ( piOrg + iStrideOrg ) );
  r3 = vld1q_s16( ( piCur + iStrideCur ) );

  r0 = vaddq_s16( r0, r2 );
  r1 = vaddq_s16( r1, r3 );

  r2 = vld1q_s16( ( piOrg + 8 ) );
  r3 = vld1q_s16( ( piCur + 8 ) );

  r4 = vld1q_s16( ( piOrg + iStrideOrg + 8 ) );
  r5 = vld1q_s16( ( piCur + iStrideCur + 8 ) );

  r2 = vaddq_s16( r2, r4 );
  r3 = vaddq_s16( r3, r5 );

  r0 = pairwise_add_s16x8( r0, r2 );
  r1 = pairwise_add_s16x8( r1, r3 );

  r0 = vaddq_s16( r0, vdupq_n_s16( 2 ) );
  r1 = vaddq_s16( r1, vdupq_n_s16( 2 ) );
  r0 = vshrq_n_s16( r0, 2 );
  r1 = vshrq_n_s16( r1, 2 );

  m2[2].val[0] = vsubq_s16( r0, r1 ); // 11bit
  //m2[1][k] = _mm_cvtepi16_epi32( _mm_srli_si128( m2[0][k], 8 ) );
  //m2[0][k] = _mm_cvtepi16_epi32( m2[0][k] );
  piCur += iStrideCur * 2;
  piOrg += iStrideOrg * 2;

  //3
  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );
  r2 = vld1q_s16( ( piOrg + iStrideOrg ) );
  r3 = vld1q_s16( ( piCur + iStrideCur ) );

  r0 = vaddq_s16( r0, r2 );
  r1 = vaddq_s16( r1, r3 );

  r2 = vld1q_s16( ( piOrg + 8 ) );
  r3 = vld1q_s16( ( piCur + 8 ) );

  r4 = vld1q_s16( ( piOrg + iStrideOrg + 8 ) );
  r5 = vld1q_s16( ( piCur + iStrideCur + 8 ) );

  r2 = vaddq_s16( r2, r4 );
  r3 = vaddq_s16( r3, r5 );

  r0 = pairwise_add_s16x8( r0, r2 );
  r1 = pairwise_add_s16x8( r1, r3 );

  r0 = vaddq_s16( r0, vdupq_n_s16( 2 ) );
  r1 = vaddq_s16( r1, vdupq_n_s16( 2 ) );
  r0 = vshrq_n_s16( r0, 2 );
  r1 = vshrq_n_s16( r1, 2 );

  m2[3].val[0] = vsubq_s16( r0, r1 ); // 11bit
  //m2[1][k] = _mm_cvtepi16_epi32( _mm_srli_si128( m2[0][k], 8 ) );
  //m2[0][k] = _mm_cvtepi16_epi32( m2[0][k] );
  piCur += iStrideCur * 2;
  piOrg += iStrideOrg * 2;

  //4
  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );
  r2 = vld1q_s16( ( piOrg + iStrideOrg ) );
  r3 = vld1q_s16( ( piCur + iStrideCur ) );

  r0 = vaddq_s16( r0, r2 );
  r1 = vaddq_s16( r1, r3 );

  r2 = vld1q_s16( ( piOrg + 8 ) );
  r3 = vld1q_s16( ( piCur + 8 ) );

  r4 = vld1q_s16( ( piOrg + iStrideOrg + 8 ) );
  r5 = vld1q_s16( ( piCur + iStrideCur + 8 ) );

  r2 = vaddq_s16( r2, r4 );
  r3 = vaddq_s16( r3, r5 );

  r0 = pairwise_add_s16x8( r0, r2 );
  r1 = pairwise_add_s16x8( r1, r3 );

  r0 = vaddq_s16( r0, vdupq_n_s16( 2 ) );
  r1 = vaddq_s16( r1, vdupq_n_s16( 2 ) );
  r0 = vshrq_n_s16( r0, 2 );
  r1 = vshrq_n_s16( r1, 2 );

  m2[4].val[0] = vsubq_s16( r0, r1 ); // 11bit
  //m2[1][k] = _mm_cvtepi16_epi32( _mm_srli_si128( m2[0][k], 8 ) );
  //m2[0][k] = _mm_cvtepi16_epi32( m2[0][k] );
  piCur += iStrideCur * 2;
  piOrg += iStrideOrg * 2;

  //5
  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );
  r2 = vld1q_s16( ( piOrg + iStrideOrg ) );
  r3 = vld1q_s16( ( piCur + iStrideCur ) );

  r0 = vaddq_s16( r0, r2 );
  r1 = vaddq_s16( r1, r3 );

  r2 = vld1q_s16( ( piOrg + 8 ) );
  r3 = vld1q_s16( ( piCur + 8 ) );

  r4 = vld1q_s16( ( piOrg + iStrideOrg + 8 ) );
  r5 = vld1q_s16( ( piCur + iStrideCur + 8 ) );

  r2 = vaddq_s16( r2, r4 );
  r3 = vaddq_s16( r3, r5 );

  r0 = pairwise_add_s16x8( r0, r2 );
  r1 = pairwise_add_s16x8( r1, r3 );

  r0 = vaddq_s16( r0, vdupq_n_s16( 2 ) );
  r1 = vaddq_s16( r1, vdupq_n_s16( 2 ) );
  r0 = vshrq_n_s16( r0, 2 );
  r1 = vshrq_n_s16( r1, 2 );

  m2[5].val[0] = vsubq_s16( r0, r1 ); // 11bit
  //m2[1][k] = _mm_cvtepi16_epi32( _mm_srli_si128( m2[0][k], 8 ) );
  //m2[0][k] = _mm_cvtepi16_epi32( m2[0][k] );
  piCur += iStrideCur * 2;
  piOrg += iStrideOrg * 2;

  //6
  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );
  r2 = vld1q_s16( ( piOrg + iStrideOrg ) );
  r3 = vld1q_s16( ( piCur + iStrideCur ) );

  r0 = vaddq_s16( r0, r2 );
  r1 = vaddq_s16( r1, r3 );

  r2 = vld1q_s16( ( piOrg + 8 ) );
  r3 = vld1q_s16( ( piCur + 8 ) );

  r4 = vld1q_s16( ( piOrg + iStrideOrg + 8 ) );
  r5 = vld1q_s16( ( piCur + iStrideCur + 8 ) );

  r2 = vaddq_s16( r2, r4 );
  r3 = vaddq_s16( r3, r5 );

  r0 = pairwise_add_s16x8( r0, r2 );
  r1 = pairwise_add_s16x8( r1, r3 );

  r0 = vaddq_s16( r0, vdupq_n_s16( 2 ) );
  r1 = vaddq_s16( r1, vdupq_n_s16( 2 ) );
  r0 = vshrq_n_s16( r0, 2 );
  r1 = vshrq_n_s16( r1, 2 );

  m2[6].val[0] = vsubq_s16( r0, r1 ); // 11bit
  //m2[1][k] = _mm_cvtepi16_epi32( _mm_srli_si128( m2[0][k], 8 ) );
  //m2[0][k] = _mm_cvtepi16_epi32( m2[0][k] );
  piCur += iStrideCur * 2;
  piOrg += iStrideOrg * 2;

  //7
  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );
  r2 = vld1q_s16( ( piOrg + iStrideOrg ) );
  r3 = vld1q_s16( ( piCur + iStrideCur ) );

  r0 = vaddq_s16( r0, r2 );
  r1 = vaddq_s16( r1, r3 );

  r2 = vld1q_s16( ( piOrg + 8 ) );
  r3 = vld1q_s16( ( piCur + 8 ) );

  r4 = vld1q_s16( ( piOrg + iStrideOrg + 8 ) );
  r5 = vld1q_s16( ( piCur + iStrideCur + 8 ) );

  r2 = vaddq_s16( r2, r4 );
  r3 = vaddq_s16( r3, r5 );

  r0 = pairwise_add_s16x8( r0, r2 );
  r1 = pairwise_add_s16x8( r1, r3 );

  r0 = vaddq_s16( r0, vdupq_n_s16( 2 ) );
  r1 = vaddq_s16( r1, vdupq_n_s16( 2 ) );
  r0 = vshrq_n_s16( r0, 2 );
  r1 = vshrq_n_s16( r1, 2 );

  m2[7].val[0] = vsubq_s16( r0, r1 ); // 11bit
  //m2[1][k] = _mm_cvtepi16_epi32( _mm_srli_si128( m2[0][k], 8 ) );
  //m2[0][k] = _mm_cvtepi16_epi32( m2[0][k] );
  piCur += iStrideCur * 2;
  piOrg += iStrideOrg * 2;

  //horizontal
  m1[0].val[0] = vaddq_s16( m2[0].val[0], m2[4].val[0] );
  m1[1].val[0] = vaddq_s16( m2[1].val[0], m2[5].val[0] );
  m1[2].val[0] = vaddq_s16( m2[2].val[0], m2[6].val[0] );
  m1[3].val[0] = vaddq_s16( m2[3].val[0], m2[7].val[0] );
  m1[4].val[0] = vsubq_s16( m2[0].val[0], m2[4].val[0] );
  m1[5].val[0] = vsubq_s16( m2[1].val[0], m2[5].val[0] );
  m1[6].val[0] = vsubq_s16( m2[2].val[0], m2[6].val[0] );
  m1[7].val[0] = vsubq_s16( m2[3].val[0], m2[7].val[0] ); // 12 bit

  m2[0].val[0] = vaddq_s16( m1[0].val[0], m1[2].val[0] );
  m2[1].val[0] = vaddq_s16( m1[1].val[0], m1[3].val[0] );
  m2[2].val[0] = vsubq_s16( m1[0].val[0], m1[2].val[0] );
  m2[3].val[0] = vsubq_s16( m1[1].val[0], m1[3].val[0] );
  m2[4].val[0] = vaddq_s16( m1[4].val[0], m1[6].val[0] );
  m2[5].val[0] = vaddq_s16( m1[5].val[0], m1[7].val[0] );
  m2[6].val[0] = vsubq_s16( m1[4].val[0], m1[6].val[0] );
  m2[7].val[0] = vsubq_s16( m1[5].val[0], m1[7].val[0] ); // 13 bit

  m1[0].val[0] = vaddq_s16( m2[0].val[0], m2[1].val[0] );
  m1[1].val[0] = vsubq_s16( m2[0].val[0], m2[1].val[0] );
  m1[2].val[0] = vaddq_s16( m2[2].val[0], m2[3].val[0] );
  m1[3].val[0] = vsubq_s16( m2[2].val[0], m2[3].val[0] );
  m1[4].val[0] = vaddq_s16( m2[4].val[0], m2[5].val[0] );
  m1[5].val[0] = vsubq_s16( m2[4].val[0], m2[5].val[0] );
  m1[6].val[0] = vaddq_s16( m2[6].val[0], m2[7].val[0] );
  m1[7].val[0] = vsubq_s16( m2[6].val[0], m2[7].val[0] ); // 14 bit

  m2[0].val[0] = vzipq_s16( m1[0].val[0], m1[1].val[0] ).val[0];
  m2[1].val[0] = vzipq_s16( m1[2].val[0], m1[3].val[0] ).val[0];
  m2[2].val[0] = vzipq_s16( m1[0].val[0], m1[1].val[0] ).val[1];
  m2[3].val[0] = vzipq_s16( m1[2].val[0], m1[3].val[0] ).val[1];
  m2[4].val[0] = vzipq_s16( m1[4].val[0], m1[5].val[0] ).val[0];
  m2[5].val[0] = vzipq_s16( m1[6].val[0], m1[7].val[0] ).val[0];
  m2[6].val[0] = vzipq_s16( m1[4].val[0], m1[5].val[0] ).val[1];
  m2[7].val[0] = vzipq_s16( m1[6].val[0], m1[7].val[0] ).val[1];

  m1[0].val[0] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16(m2[0].val[0]) , vreinterpretq_s32_s16(m2[1].val[0]) ).val[0] );
  m1[1].val[0] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16(m2[0].val[0]) , vreinterpretq_s32_s16(m2[1].val[0]) ).val[1] );
  m1[2].val[0] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16(m2[2].val[0]) , vreinterpretq_s32_s16(m2[3].val[0]) ).val[0] );
  m1[3].val[0] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16(m2[2].val[0]) , vreinterpretq_s32_s16(m2[3].val[0]) ).val[1] );
  m1[4].val[0] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16(m2[4].val[0]) , vreinterpretq_s32_s16(m2[5].val[0]) ).val[0] );
  m1[5].val[0] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16(m2[4].val[0]) , vreinterpretq_s32_s16(m2[5].val[0]) ).val[1] );
  m1[6].val[0] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16(m2[6].val[0]) , vreinterpretq_s32_s16(m2[7].val[0]) ).val[0] );
  m1[7].val[0] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16(m2[6].val[0]) , vreinterpretq_s32_s16(m2[7].val[0]) ).val[1] );

  m3[0].val[1] = vmovl_s16( vget_high_s16( m1[0].val[0] ) );
  m3[0].val[0] = vmovl_s16( vget_low_s16( m1[0].val[0] ) );
  m3[1].val[1] = vmovl_s16( vget_high_s16( m1[1].val[0] ) );
  m3[1].val[0] = vmovl_s16( vget_low_s16( m1[1].val[0] ) );
  m3[2].val[1] = vmovl_s16( vget_high_s16( m1[2].val[0] ) );
  m3[2].val[0] = vmovl_s16( vget_low_s16( m1[2].val[0] ) );
  m3[3].val[1] = vmovl_s16( vget_high_s16( m1[3].val[0] ) );
  m3[3].val[0] = vmovl_s16( vget_low_s16( m1[3].val[0] ) );
  m3[4].val[1] = vmovl_s16( vget_high_s16( m1[4].val[0] ) );
  m3[4].val[0] = vmovl_s16( vget_low_s16( m1[4].val[0] ) );
  m3[5].val[1] = vmovl_s16( vget_high_s16( m1[5].val[0] ) );
  m3[5].val[0] = vmovl_s16( vget_low_s16( m1[5].val[0] ) );
  m3[6].val[1] = vmovl_s16( vget_high_s16( m1[6].val[0] ) );
  m3[6].val[0] = vmovl_s16( vget_low_s16( m1[6].val[0] ) );
  m3[7].val[1] = vmovl_s16( vget_high_s16( m1[7].val[0] ) );
  m3[7].val[0] = vmovl_s16( vget_low_s16( m1[7].val[0] ) );

  m4[0].val[0] = m3[0].val[0];
  m4[0].val[1] = m3[4].val[0];

  m4[1].val[0] = m3[1].val[0];
  m4[1].val[1] = m3[5].val[0];

  m4[2].val[0] = m3[2].val[0];
  m4[2].val[1] = m3[6].val[0];

  m4[3].val[0] = m3[3].val[0];
  m4[3].val[1] = m3[7].val[0];

  m4[4].val[0] = m3[0].val[1];
  m4[4].val[1] = m3[4].val[1];

  m4[5].val[0] = m3[1].val[1];
  m4[5].val[1] = m3[5].val[1];

  m4[6].val[0] = m3[2].val[1];
  m4[6].val[1] = m3[6].val[1];

  m4[7].val[0] = m3[3].val[1];
  m4[7].val[1] = m3[7].val[1];

  m3[0].val[0] = vaddq_s32( m4[0].val[0], m4[4].val[0] );
  m3[1].val[0] = vaddq_s32( m4[1].val[0], m4[5].val[0] );
  m3[2].val[0] = vaddq_s32( m4[2].val[0], m4[6].val[0] );
  m3[3].val[0] = vaddq_s32( m4[3].val[0], m4[7].val[0] );
  m3[4].val[0] = vsubq_s32( m4[0].val[0], m4[4].val[0] );
  m3[5].val[0] = vsubq_s32( m4[1].val[0], m4[5].val[0] );
  m3[6].val[0] = vsubq_s32( m4[2].val[0], m4[6].val[0] );
  m3[7].val[0] = vsubq_s32( m4[3].val[0], m4[7].val[0] );

  m4[0].val[0] = vaddq_s32( m3[0].val[0], m3[2].val[0] );
  m4[1].val[0] = vaddq_s32( m3[1].val[0], m3[3].val[0] );
  m4[2].val[0] = vsubq_s32( m3[0].val[0], m3[2].val[0] );
  m4[3].val[0] = vsubq_s32( m3[1].val[0], m3[3].val[0] );
  m4[4].val[0] = vaddq_s32( m3[4].val[0], m3[6].val[0] );
  m4[5].val[0] = vaddq_s32( m3[5].val[0], m3[7].val[0] );
  m4[6].val[0] = vsubq_s32( m3[4].val[0], m3[6].val[0] );
  m4[7].val[0] = vsubq_s32( m3[5].val[0], m3[7].val[0] );

  m3[0].val[0] = vabsq_s32( vaddq_s32( m4[0].val[0], m4[1].val[0] ) );
  m3[1].val[0] = vabdq_s32(m4[0].val[0], m4[1].val[0]);
  m3[2].val[0] = vabsq_s32( vaddq_s32( m4[2].val[0], m4[3].val[0] ) );
  m3[3].val[0] = vabdq_s32(m4[2].val[0], m4[3].val[0]);
  m3[4].val[0] = vabsq_s32( vaddq_s32( m4[4].val[0], m4[5].val[0] ) );
  m3[5].val[0] = vabdq_s32(m4[4].val[0], m4[5].val[0]);
  m3[6].val[0] = vabsq_s32( vaddq_s32( m4[6].val[0], m4[7].val[0] ) );
  m3[7].val[0] = vabdq_s32(m4[6].val[0], m4[7].val[0]);

  // --------------------------------------------------------------------------------------------
  m3[0].val[1] = vaddq_s32( m4[0].val[1], m4[4].val[1] );
  m3[1].val[1] = vaddq_s32( m4[1].val[1], m4[5].val[1] );
  m3[2].val[1] = vaddq_s32( m4[2].val[1], m4[6].val[1] );
  m3[3].val[1] = vaddq_s32( m4[3].val[1], m4[7].val[1] );
  m3[4].val[1] = vsubq_s32( m4[0].val[1], m4[4].val[1] );
  m3[5].val[1] = vsubq_s32( m4[1].val[1], m4[5].val[1] );
  m3[6].val[1] = vsubq_s32( m4[2].val[1], m4[6].val[1] );
  m3[7].val[1] = vsubq_s32( m4[3].val[1], m4[7].val[1] );

  m4[0].val[1] = vaddq_s32( m3[0].val[1], m3[2].val[1] );
  m4[1].val[1] = vaddq_s32( m3[1].val[1], m3[3].val[1] );
  m4[2].val[1] = vsubq_s32( m3[0].val[1], m3[2].val[1] );
  m4[3].val[1] = vsubq_s32( m3[1].val[1], m3[3].val[1] );
  m4[4].val[1] = vaddq_s32( m3[4].val[1], m3[6].val[1] );
  m4[5].val[1] = vaddq_s32( m3[5].val[1], m3[7].val[1] );
  m4[6].val[1] = vsubq_s32( m3[4].val[1], m3[6].val[1] );
  m4[7].val[1] = vsubq_s32( m3[5].val[1], m3[7].val[1] );

  m3[0].val[1] = vabsq_s32( vaddq_s32( m4[0].val[1], m4[1].val[1] ) );
  m3[1].val[1] = vabdq_s32(m4[0].val[1], m4[1].val[1]);
  m3[2].val[1] = vabsq_s32( vaddq_s32( m4[2].val[1], m4[3].val[1] ) );
  m3[3].val[1] = vabdq_s32(m4[2].val[1], m4[3].val[1]);
  m3[4].val[1] = vabsq_s32( vaddq_s32( m4[4].val[1], m4[5].val[1] ) );
  m3[5].val[1] = vabdq_s32(m4[4].val[1], m4[5].val[1]);
  m3[6].val[1] = vabsq_s32( vaddq_s32( m4[6].val[1], m4[7].val[1] ) );
  m3[7].val[1] = vabdq_s32(m4[6].val[1], m4[7].val[1]);

  m4[0].val[0] = m3[0].val[0];

  m3[0].val[0] = vaddq_s32( m3[0].val[0], m3[0].val[1] );
  m3[1].val[0] = vaddq_s32( m3[1].val[0], m3[1].val[1] );
  m3[2].val[0] = vaddq_s32( m3[2].val[0], m3[2].val[1] );
  m3[3].val[0] = vaddq_s32( m3[3].val[0], m3[3].val[1] );
  m3[4].val[0] = vaddq_s32( m3[4].val[0], m3[4].val[1] );
  m3[5].val[0] = vaddq_s32( m3[5].val[0], m3[5].val[1] );
  m3[6].val[0] = vaddq_s32( m3[6].val[0], m3[6].val[1] );
  m3[7].val[0] = vaddq_s32( m3[7].val[0], m3[7].val[1] );

  m3[0].val[0] = vaddq_s32( m3[0].val[0], m3[1].val[0] );
  m3[2].val[0] = vaddq_s32( m3[2].val[0], m3[3].val[0] );
  m3[4].val[0] = vaddq_s32( m3[4].val[0], m3[5].val[0] );
  m3[6].val[0] = vaddq_s32( m3[6].val[0], m3[7].val[0] );

  m3[0].val[0] = vaddq_s32( m3[0].val[0], m3[2].val[0] );
  m3[4].val[0] = vaddq_s32( m3[4].val[0], m3[6].val[0] );
  int32x4_t iSum = vaddq_s32( m3[0].val[0], m3[4].val[0] );

  uint32_t sad = ( uint32_t ) horizontal_add_s32x4(iSum);
  uint32_t absDc = vgetq_lane_s32( m4[0].val[0], 0 );
  sad -= absDc;
  sad += absDc >> 2;
  sad = ( ( sad + 2 ) >> 2 );

  return ( sad << 2 );
}

static uint32_t xCalcHAD8x8_Neon( const Pel *piOrg, const Pel *piCur, const int iStrideOrg, const int iStrideCur, const int iBitDepth )
{
  CHECK( iBitDepth > 10, "Only bit-depths of up to 10 bits supported!" );

  int16x8_t m1[8], m2[8];
  int32x4x2_t m3[8], m4[8];

  for( int i = 0; i < 8; i++ )
  {
    int16x8_t r0 = vld1q_s16( piOrg + i * iStrideOrg );
    int16x8_t r1 = vld1q_s16( piCur + i * iStrideCur );

    m2[i] = vsubq_s16( r0, r1 ); // 11-bit
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

  // Partial transpose to do the first horizontal step.
  m2[0] = vtrnq_s16( m1[0], m1[1] ).val[0];
  m2[1] = vtrnq_s16( m1[0], m1[1] ).val[1];
  m2[2] = vtrnq_s16( m1[2], m1[3] ).val[0];
  m2[3] = vtrnq_s16( m1[2], m1[3] ).val[1];
  m2[4] = vtrnq_s16( m1[4], m1[5] ).val[0];
  m2[5] = vtrnq_s16( m1[4], m1[5] ).val[1];
  m2[6] = vtrnq_s16( m1[6], m1[7] ).val[0];
  m2[7] = vtrnq_s16( m1[6], m1[7] ).val[1];

  m1[0] = vaddq_s16( m2[0], m2[1] );
  m1[1] = vaddq_s16( m2[2], m2[3] );
  m1[2] = vaddq_s16( m2[4], m2[5] );
  m1[3] = vaddq_s16( m2[6], m2[7] );
  m1[4] = vsubq_s16( m2[0], m2[1] );
  m1[5] = vsubq_s16( m2[2], m2[3] );
  m1[6] = vsubq_s16( m2[4], m2[5] );
  m1[7] = vsubq_s16( m2[6], m2[7] ); // 15-bit

  // Finish the transpose.
  m2[0] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16( m1[0] ), vreinterpretq_s32_s16( m1[1] ) ).val[0] );
  m2[1] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16( m1[0] ), vreinterpretq_s32_s16( m1[1] ) ).val[1] );
  m2[2] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16( m1[2] ), vreinterpretq_s32_s16( m1[3] ) ).val[0] );
  m2[3] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16( m1[2] ), vreinterpretq_s32_s16( m1[3] ) ).val[1] );
  m2[4] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16( m1[4] ), vreinterpretq_s32_s16( m1[5] ) ).val[0] );
  m2[5] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16( m1[4] ), vreinterpretq_s32_s16( m1[5] ) ).val[1] );
  m2[6] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16( m1[6] ), vreinterpretq_s32_s16( m1[7] ) ).val[0] );
  m2[7] = vreinterpretq_s16_s32( vzipq_s32( vreinterpretq_s32_s16( m1[6] ), vreinterpretq_s32_s16( m1[7] ) ).val[1] );

  m4[0].val[0] = vaddl_s16( vget_low_s16( m2[0] ), vget_low_s16( m2[1] ) );
  m4[1].val[0] = vaddl_s16( vget_high_s16( m2[0] ), vget_high_s16( m2[1] ) );
  m4[2].val[0] = vsubl_s16( vget_low_s16( m2[0] ), vget_low_s16( m2[1] ) );
  m4[3].val[0] = vsubl_s16( vget_high_s16( m2[0] ), vget_high_s16( m2[1] ) );
  m4[4].val[0] = vaddl_s16( vget_low_s16( m2[4] ), vget_low_s16( m2[5] ) );
  m4[5].val[0] = vaddl_s16( vget_high_s16( m2[4] ), vget_high_s16( m2[5] ) );
  m4[6].val[0] = vsubl_s16( vget_low_s16( m2[4] ), vget_low_s16( m2[5] ) );
  m4[7].val[0] = vsubl_s16( vget_high_s16( m2[4] ), vget_high_s16( m2[5] ) );

  m3[0].val[0] = vabsq_s32( vaddq_s32( m4[0].val[0], m4[1].val[0] ) );
  m3[1].val[0] = vabdq_s32(m4[0].val[0], m4[1].val[0]);
  m3[2].val[0] = vabsq_s32( vaddq_s32( m4[2].val[0], m4[3].val[0] ) );
  m3[3].val[0] = vabdq_s32(m4[2].val[0], m4[3].val[0]);
  m3[4].val[0] = vabsq_s32( vaddq_s32( m4[4].val[0], m4[5].val[0] ) );
  m3[5].val[0] = vabdq_s32(m4[4].val[0], m4[5].val[0]);
  m3[6].val[0] = vabsq_s32( vaddq_s32( m4[6].val[0], m4[7].val[0] ) );
  m3[7].val[0] = vabdq_s32(m4[6].val[0], m4[7].val[0]);

  // --------------------------------------------------------------------------------------------
  m4[0].val[1] = vaddl_s16( vget_low_s16( m2[2] ), vget_low_s16( m2[3] ) );
  m4[1].val[1] = vaddl_s16( vget_high_s16( m2[2] ), vget_high_s16( m2[3] ) );
  m4[2].val[1] = vsubl_s16( vget_low_s16( m2[2] ), vget_low_s16( m2[3] ) );
  m4[3].val[1] = vsubl_s16( vget_high_s16( m2[2] ), vget_high_s16( m2[3] ) );
  m4[4].val[1] = vaddl_s16( vget_low_s16( m2[6] ), vget_low_s16( m2[7] ) );
  m4[5].val[1] = vaddl_s16( vget_high_s16( m2[6] ), vget_high_s16( m2[7] ) );
  m4[6].val[1] = vsubl_s16( vget_low_s16( m2[6] ), vget_low_s16( m2[7] ) );
  m4[7].val[1] = vsubl_s16( vget_high_s16( m2[6] ), vget_high_s16( m2[7] ) );

  m3[0].val[1] = vabsq_s32( vaddq_s32( m4[0].val[1], m4[1].val[1] ) );
  m3[1].val[1] = vabdq_s32(m4[0].val[1], m4[1].val[1]);
  m3[2].val[1] = vabsq_s32( vaddq_s32( m4[2].val[1], m4[3].val[1] ) );
  m3[3].val[1] = vabdq_s32(m4[2].val[1], m4[3].val[1]);
  m3[4].val[1] = vabsq_s32( vaddq_s32( m4[4].val[1], m4[5].val[1] ) );
  m3[5].val[1] = vabdq_s32(m4[4].val[1], m4[5].val[1]);
  m3[6].val[1] = vabsq_s32( vaddq_s32( m4[6].val[1], m4[7].val[1] ) );
  m3[7].val[1] = vabdq_s32(m4[6].val[1], m4[7].val[1]);

  m4[0].val[0] = m3[0].val[0];

  m3[0].val[0] = vaddq_s32( m3[0].val[0], m3[0].val[1] );
  m3[1].val[0] = vaddq_s32( m3[1].val[0], m3[1].val[1] );
  m3[2].val[0] = vaddq_s32( m3[2].val[0], m3[2].val[1] );
  m3[3].val[0] = vaddq_s32( m3[3].val[0], m3[3].val[1] );
  m3[4].val[0] = vaddq_s32( m3[4].val[0], m3[4].val[1] );
  m3[5].val[0] = vaddq_s32( m3[5].val[0], m3[5].val[1] );
  m3[6].val[0] = vaddq_s32( m3[6].val[0], m3[6].val[1] );
  m3[7].val[0] = vaddq_s32( m3[7].val[0], m3[7].val[1] );

  m3[0].val[0] = vaddq_s32( m3[0].val[0], m3[1].val[0] );
  m3[2].val[0] = vaddq_s32( m3[2].val[0], m3[3].val[0] );
  m3[4].val[0] = vaddq_s32( m3[4].val[0], m3[5].val[0] );
  m3[6].val[0] = vaddq_s32( m3[6].val[0], m3[7].val[0] );

  m3[0].val[0] = vaddq_s32( m3[0].val[0], m3[2].val[0] );
  m3[4].val[0] = vaddq_s32( m3[4].val[0], m3[6].val[0] );
  int32x4_t iSum = vaddq_s32( m3[0].val[0], m3[4].val[0] );

  uint32_t sad = ( uint32_t ) horizontal_add_s32x4(iSum);
  uint32_t absDC = vgetq_lane_s32( m4[0].val[0], 0 );
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( ( sad + 2 ) >> 2 );

  return sad;
}

static Distortion xCalcHAD16x8_neon( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
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

  // Transpose.
  m1[0] = vzipq_s16( m2[0], m2[8] ).val[0];
  m1[1] = vzipq_s16( m2[0], m2[8] ).val[1];
  m1[2] = vzipq_s16( m2[2], m2[10] ).val[0];
  m1[3] = vzipq_s16( m2[2], m2[10] ).val[1];
  m1[4] = vzipq_s16( m2[4], m2[12] ).val[0];
  m1[5] = vzipq_s16( m2[4], m2[12] ).val[1];
  m1[6] = vzipq_s16( m2[6], m2[14] ).val[0];
  m1[7] = vzipq_s16( m2[6], m2[14] ).val[1];
  m1[8] = vzipq_s16( m2[1], m2[9] ).val[0];
  m1[9] = vzipq_s16( m2[1], m2[9] ).val[1];
  m1[10] = vzipq_s16( m2[3], m2[11] ).val[0];
  m1[11] = vzipq_s16( m2[3], m2[11] ).val[1];
  m1[12] = vzipq_s16( m2[5], m2[13] ).val[0];
  m1[13] = vzipq_s16( m2[5], m2[13] ).val[1];
  m1[14] = vzipq_s16( m2[7], m2[15] ).val[0];
  m1[15] = vzipq_s16( m2[7], m2[15] ).val[1];

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

  // Horizontal.
  m2[0] = vaddq_s16( m1[0], m1[8] );
  m2[1] = vaddq_s16( m1[1], m1[9] );
  m2[2] = vaddq_s16( m1[2], m1[10] );
  m2[3] = vaddq_s16( m1[3], m1[11] );
  m2[4] = vaddq_s16( m1[4], m1[12] );
  m2[5] = vaddq_s16( m1[5], m1[13] );
  m2[6] = vaddq_s16( m1[6], m1[14] );
  m2[7] = vaddq_s16( m1[7], m1[15] );
  m2[8] = vsubq_s16( m1[0], m1[8] );
  m2[9] = vsubq_s16( m1[1], m1[9] );
  m2[10] = vsubq_s16( m1[2], m1[10] );
  m2[11] = vsubq_s16( m1[3], m1[11] );
  m2[12] = vsubq_s16( m1[4], m1[12] );
  m2[13] = vsubq_s16( m1[5], m1[13] );
  m2[14] = vsubq_s16( m1[6], m1[14] );
  m2[15] = vsubq_s16( m1[7], m1[15] ); // 15-bit

  int32x4_t m3[32], m4[32];

  m3[0] = vaddl_s16( vget_low_s16( m2[0] ), vget_low_s16( m2[4] ) );
  m3[1] = vaddl_s16( vget_high_s16( m2[0] ), vget_high_s16( m2[4] ) );
  m3[2] = vaddl_s16( vget_low_s16( m2[1] ), vget_low_s16( m2[5] ) );
  m3[3] = vaddl_s16( vget_high_s16( m2[1] ), vget_high_s16( m2[5] ) );
  m3[4] = vaddl_s16( vget_low_s16( m2[2] ), vget_low_s16( m2[6] ) );
  m3[5] = vaddl_s16( vget_high_s16( m2[2] ), vget_high_s16( m2[6] ) );
  m3[6] = vaddl_s16( vget_low_s16( m2[3] ), vget_low_s16( m2[7] ) );
  m3[7] = vaddl_s16( vget_high_s16( m2[3] ), vget_high_s16( m2[7] ) );
  m3[8] = vsubl_s16( vget_low_s16( m2[0] ), vget_low_s16( m2[4] ) );
  m3[9] = vsubl_s16( vget_high_s16( m2[0] ), vget_high_s16( m2[4] ) );
  m3[10] = vsubl_s16( vget_low_s16( m2[1] ), vget_low_s16( m2[5] ) );
  m3[11] = vsubl_s16( vget_high_s16( m2[1] ), vget_high_s16( m2[5] ) );
  m3[12] = vsubl_s16( vget_low_s16( m2[2] ), vget_low_s16( m2[6] ) );
  m3[13] = vsubl_s16( vget_high_s16( m2[2] ), vget_high_s16( m2[6] ) );
  m3[14] = vsubl_s16( vget_low_s16( m2[3] ), vget_low_s16( m2[7] ) );
  m3[15] = vsubl_s16( vget_high_s16( m2[3] ), vget_high_s16( m2[7] ) );
  m3[16] = vaddl_s16( vget_low_s16( m2[8] ), vget_low_s16( m2[12] ) );
  m3[17] = vaddl_s16( vget_high_s16( m2[8] ), vget_high_s16( m2[12] ) );
  m3[18] = vaddl_s16( vget_low_s16( m2[9] ), vget_low_s16( m2[13] ) );
  m3[19] = vaddl_s16( vget_high_s16( m2[9] ), vget_high_s16( m2[13] ) );
  m3[20] = vaddl_s16( vget_low_s16( m2[10] ), vget_low_s16( m2[14] ) );
  m3[21] = vaddl_s16( vget_high_s16( m2[10] ), vget_high_s16( m2[14] ) );
  m3[22] = vaddl_s16( vget_low_s16( m2[11] ), vget_low_s16( m2[15] ) );
  m3[23] = vaddl_s16( vget_high_s16( m2[11] ), vget_high_s16( m2[15] ) );
  m3[24] = vsubl_s16( vget_low_s16( m2[8] ), vget_low_s16( m2[12] ) );
  m3[25] = vsubl_s16( vget_high_s16( m2[8] ), vget_high_s16( m2[12] ) );
  m3[26] = vsubl_s16( vget_low_s16( m2[9] ), vget_low_s16( m2[13] ) );
  m3[27] = vsubl_s16( vget_high_s16( m2[9] ), vget_high_s16( m2[13] ) );
  m3[28] = vsubl_s16( vget_low_s16( m2[10] ), vget_low_s16( m2[14] ) );
  m3[29] = vsubl_s16( vget_high_s16( m2[10] ), vget_high_s16( m2[14] ) );
  m3[30] = vsubl_s16( vget_low_s16( m2[11] ), vget_low_s16( m2[15] ) );
  m3[31] = vsubl_s16( vget_high_s16( m2[11] ), vget_high_s16( m2[15] ) );

  m4[0] = vaddq_s32( m3[0], m3[4] );
  m4[1] = vaddq_s32( m3[1], m3[5] );
  m4[2] = vaddq_s32( m3[2], m3[6] );
  m4[3] = vaddq_s32( m3[3], m3[7] );
  m4[4] = vsubq_s32( m3[0], m3[4] );
  m4[5] = vsubq_s32( m3[1], m3[5] );
  m4[6] = vsubq_s32( m3[2], m3[6] );
  m4[7] = vsubq_s32( m3[3], m3[7] );
  m4[8] = vaddq_s32( m3[8], m3[12] );
  m4[9] = vaddq_s32( m3[9], m3[13] );
  m4[10] = vaddq_s32( m3[10], m3[14] );
  m4[11] = vaddq_s32( m3[11], m3[15] );
  m4[12] = vsubq_s32( m3[8], m3[12] );
  m4[13] = vsubq_s32( m3[9], m3[13] );
  m4[14] = vsubq_s32( m3[10], m3[14] );
  m4[15] = vsubq_s32( m3[11], m3[15] );
  m4[16] = vaddq_s32( m3[16], m3[20] );
  m4[17] = vaddq_s32( m3[17], m3[21] );
  m4[18] = vaddq_s32( m3[18], m3[22] );
  m4[19] = vaddq_s32( m3[19], m3[23] );
  m4[20] = vsubq_s32( m3[16], m3[20] );
  m4[21] = vsubq_s32( m3[17], m3[21] );
  m4[22] = vsubq_s32( m3[18], m3[22] );
  m4[23] = vsubq_s32( m3[19], m3[23] );
  m4[24] = vaddq_s32( m3[24], m3[28] );
  m4[25] = vaddq_s32( m3[25], m3[29] );
  m4[26] = vaddq_s32( m3[26], m3[30] );
  m4[27] = vaddq_s32( m3[27], m3[31] );
  m4[28] = vsubq_s32( m3[24], m3[28] );
  m4[29] = vsubq_s32( m3[25], m3[29] );
  m4[30] = vsubq_s32( m3[26], m3[30] );
  m4[31] = vsubq_s32( m3[27], m3[31] );

  m3[0] = vabsq_s32( vaddq_s32( m4[0], m4[2] ) );
  m3[1] = vabsq_s32( vaddq_s32( m4[1], m4[3] ) );
  m3[2] = vabdq_s32( m4[0], m4[2] );
  m3[3] = vabdq_s32( m4[1], m4[3] );
  m3[4] = vabsq_s32( vaddq_s32( m4[4], m4[6] ) );
  m3[5] = vabsq_s32( vaddq_s32( m4[5], m4[7] ) );
  m3[6] = vabdq_s32( m4[4], m4[6] );
  m3[7] = vabdq_s32( m4[5], m4[7] );
  m3[8] = vabsq_s32( vaddq_s32( m4[8], m4[10] ) );
  m3[9] = vabsq_s32( vaddq_s32( m4[9], m4[11] ) );
  m3[10] = vabdq_s32( m4[8], m4[10] );
  m3[11] = vabdq_s32( m4[9], m4[11] );
  m3[12] = vabsq_s32( vaddq_s32( m4[12], m4[14] ) );
  m3[13] = vabsq_s32( vaddq_s32( m4[13], m4[15] ) );
  m3[14] = vabdq_s32( m4[12], m4[14] );
  m3[15] = vabdq_s32( m4[13], m4[15] );
  m3[16] = vabsq_s32( vaddq_s32( m4[16], m4[18] ) );
  m3[17] = vabsq_s32( vaddq_s32( m4[17], m4[19] ) );
  m3[18] = vabdq_s32( m4[16], m4[18] );
  m3[19] = vabdq_s32( m4[17], m4[19] );
  m3[20] = vabsq_s32( vaddq_s32( m4[20], m4[22] ) );
  m3[21] = vabsq_s32( vaddq_s32( m4[21], m4[23] ) );
  m3[22] = vabdq_s32( m4[20], m4[22] );
  m3[23] = vabdq_s32( m4[21], m4[23] );
  m3[24] = vabsq_s32( vaddq_s32( m4[24], m4[26] ) );
  m3[25] = vabsq_s32( vaddq_s32( m4[25], m4[27] ) );
  m3[26] = vabdq_s32( m4[24], m4[26] );
  m3[27] = vabdq_s32( m4[25], m4[27] );
  m3[28] = vabsq_s32( vaddq_s32( m4[28], m4[30] ) );
  m3[29] = vabsq_s32( vaddq_s32( m4[29], m4[31] ) );
  m3[30] = vabdq_s32( m4[28], m4[30] );
  m3[31] = vabdq_s32( m4[29], m4[31] );

  uint32_t absDC = vgetq_lane_s32( m3[0], 0 );

  m3[0] = vaddq_s32( m3[0], m3[1] );
  m3[2] = vaddq_s32( m3[2], m3[3] );
  m3[4] = vaddq_s32( m3[4], m3[5] );
  m3[6] = vaddq_s32( m3[6], m3[7] );
  m3[8] = vaddq_s32( m3[8], m3[9] );
  m3[10] = vaddq_s32( m3[10], m3[11] );
  m3[12] = vaddq_s32( m3[12], m3[13] );
  m3[14] = vaddq_s32( m3[14], m3[15] );
  m3[16] = vaddq_s32( m3[16], m3[17] );
  m3[18] = vaddq_s32( m3[18], m3[19] );
  m3[20] = vaddq_s32( m3[20], m3[21] );
  m3[22] = vaddq_s32( m3[22], m3[23] );
  m3[24] = vaddq_s32( m3[24], m3[25] );
  m3[26] = vaddq_s32( m3[26], m3[27] );
  m3[28] = vaddq_s32( m3[28], m3[29] );
  m3[30] = vaddq_s32( m3[30], m3[31] );

  m3[0] = vaddq_s32( m3[0], m3[2] );
  m3[4] = vaddq_s32( m3[4], m3[6] );
  m3[8] = vaddq_s32( m3[8], m3[10] );
  m3[12] = vaddq_s32( m3[12], m3[14] );
  m3[16] = vaddq_s32( m3[16], m3[18] );
  m3[20] = vaddq_s32( m3[20], m3[22] );
  m3[24] = vaddq_s32( m3[24], m3[26] );
  m3[28] = vaddq_s32( m3[28], m3[30] );

  m3[0] = vaddq_s32( m3[0], m3[4] );
  m3[8] = vaddq_s32( m3[8], m3[12] );
  m3[16] = vaddq_s32( m3[16], m3[20] );
  m3[24] = vaddq_s32( m3[24], m3[28] );

  m3[0] = vaddq_s32( m3[0], m3[8] );
  m3[16] = vaddq_s32( m3[16], m3[24] );

  m3[0] = vaddq_s32( m3[0], m3[16] );

  int sad = horizontal_add_s32x4( m3[0] );

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

static Distortion xCalcHAD8x4_neon( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
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

  // Vertical.
  m0[0] = vaddq_s16( diff[0], diff[2] );
  m0[1] = vaddq_s16( diff[1], diff[3] );
  m0[2] = vsubq_s16( diff[0], diff[2] );
  m0[3] = vsubq_s16( diff[1], diff[3] ); // 12-bit

  m1[0] = vaddq_s16( m0[0], m0[1] );
  m1[1] = vsubq_s16( m0[0], m0[1] );
  m1[2] = vaddq_s16( m0[2], m0[3] );
  m1[3] = vsubq_s16( m0[2], m0[3] ); // 13-bit

  // Transpose.
  int16x8x2_t zip02 = vzipq_s16( m1[0], m1[2] );
  int16x8x2_t zip13 = vzipq_s16( m1[1], m1[3] );

  int16x8x2_t zip0123 = vzipq_s16( zip02.val[0], zip13.val[0] );
  int16x8x2_t zip4567 = vzipq_s16( zip02.val[1], zip13.val[1] );

  m0[0] = zip0123.val[0];
  m0[1] = zip0123.val[1];
  m0[2] = zip4567.val[0];
  m0[3] = zip4567.val[1];

  // Horizontal.
  m1[0] = vaddq_s16( m0[0], m0[2] );
  m1[1] = vaddq_s16( m0[1], m0[3] );
  m1[2] = vsubq_s16( m0[0], m0[2] );
  m1[3] = vsubq_s16( m0[1], m0[3] ); // 14-bit

  m0[0] = vaddq_s16( m1[0], m1[1] );
  m0[1] = vsubq_s16( m1[0], m1[1] );
  m0[2] = vaddq_s16( m1[2], m1[3] );
  m0[3] = vsubq_s16( m1[2], m1[3] ); // 15-bit

  int16x8x2_t a = vvenc_vtrnq_s64_to_s16( m0[0], m0[1] );
  int16x8x2_t b = vvenc_vtrnq_s64_to_s16( m0[2], m0[3] );

  m1[0] = a.val[0];
  m1[1] = a.val[1];
  m1[2] = b.val[0];
  m1[3] = b.val[1];

  int32x4_t sum0 = vabsq_s32( vaddl_s16( vget_low_s16( m1[0] ), vget_low_s16( m1[1] ) ) );
  uint32_t absDC = ( uint32_t )vgetq_lane_s32( sum0, 0 );
  sum0 = vabal_s16( sum0, vget_low_s16( m1[0] ), vget_low_s16( m1[1] ) );

  int32x4_t sum1 = vabsq_s32( vaddl_s16( vget_high_s16( m1[0] ), vget_high_s16( m1[1] ) ) );
  sum1 = vabal_s16( sum1, vget_high_s16( m1[0] ), vget_high_s16( m1[1] ) );

  int32x4_t sum2 = vabsq_s32( vaddl_s16( vget_low_s16( m1[2] ), vget_low_s16( m1[3] ) ) );
  sum2 = vabal_s16( sum2, vget_low_s16( m1[2] ), vget_low_s16( m1[3] ) );

  int32x4_t sum3 = vabsq_s32( vaddl_s16( vget_high_s16( m1[2] ), vget_high_s16( m1[3] ) ) );
  sum3 = vabal_s16( sum3, vget_high_s16( m1[2] ), vget_high_s16( m1[3] ) );

  int32x4_t total = horizontal_add_4d_s32x4( sum0, sum1, sum2, sum3 );

  uint32_t sad = ( uint32_t )horizontal_add_s32x4( total );
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( uint32_t )( ( double )sad / sqrt( 4.0 * 8.0 ) * 2.0 );

  return sad;
}

static Distortion xCalcHAD4x8_neon( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur, int iBitDepth )
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

  int16x4_t m1[8], m2[8];

  // Vertical.
  m1[0] = vadd_s16( diff[0], diff[4] );
  m1[1] = vadd_s16( diff[1], diff[5] );
  m1[2] = vadd_s16( diff[2], diff[6] );
  m1[3] = vadd_s16( diff[3], diff[7] );
  m1[4] = vsub_s16( diff[0], diff[4] );
  m1[5] = vsub_s16( diff[1], diff[5] );
  m1[6] = vsub_s16( diff[2], diff[6] );
  m1[7] = vsub_s16( diff[3], diff[7] ); // 12-bit

  m2[0] = vadd_s16( m1[0], m1[2] );
  m2[1] = vadd_s16( m1[1], m1[3] );
  m2[2] = vsub_s16( m1[0], m1[2] );
  m2[3] = vsub_s16( m1[1], m1[3] );
  m2[4] = vadd_s16( m1[4], m1[6] );
  m2[5] = vadd_s16( m1[5], m1[7] );
  m2[6] = vsub_s16( m1[4], m1[6] );
  m2[7] = vsub_s16( m1[5], m1[7] ); // 13-bit

  m1[0] = vadd_s16( m2[0], m2[1] );
  m1[1] = vsub_s16( m2[0], m2[1] );
  m1[2] = vadd_s16( m2[2], m2[3] );
  m1[3] = vsub_s16( m2[2], m2[3] );
  m1[4] = vadd_s16( m2[4], m2[5] );
  m1[5] = vsub_s16( m2[4], m2[5] );
  m1[6] = vadd_s16( m2[6], m2[7] );
  m1[7] = vsub_s16( m2[6], m2[7] ); // 14-bit

  // Transpose.
  int16x8_t m1q[8];
  const int16x4_t zero = vdup_n_s16( 0 );
  m1q[0] = vcombine_s16( m1[0], zero );
  m1q[1] = vcombine_s16( m1[1], zero );
  m1q[2] = vcombine_s16( m1[2], zero );
  m1q[3] = vcombine_s16( m1[3], zero );
  m1q[4] = vcombine_s16( m1[4], zero );
  m1q[5] = vcombine_s16( m1[5], zero );
  m1q[6] = vcombine_s16( m1[6], zero );
  m1q[7] = vcombine_s16( m1[7], zero );

  int16x8_t m3[4], m4[4];
  m3[0] = vzipq_s16( m1q[0], m1q[4] ).val[0];
  m3[1] = vzipq_s16( m1q[2], m1q[6] ).val[0];
  m3[2] = vzipq_s16( m1q[1], m1q[5] ).val[0];
  m3[3] = vzipq_s16( m1q[3], m1q[7] ).val[0];

  m4[0] = vzipq_s16( m3[0], m3[1] ).val[0];
  m4[1] = vzipq_s16( m3[0], m3[1] ).val[1];
  m4[2] = vzipq_s16( m3[2], m3[3] ).val[0];
  m4[3] = vzipq_s16( m3[2], m3[3] ).val[1];

  m3[0] = vzipq_s16( m4[0], m4[2] ).val[0];
  m3[1] = vzipq_s16( m4[0], m4[2] ).val[1];
  m3[2] = vzipq_s16( m4[1], m4[3] ).val[0];
  m3[3] = vzipq_s16( m4[1], m4[3] ).val[1];

  // Horizontal.
  m4[0] = vaddq_s16( m3[0], m3[2] );
  m4[1] = vaddq_s16( m3[1], m3[3] );
  m4[2] = vsubq_s16( m3[0], m3[2] );
  m4[3] = vsubq_s16( m3[1], m3[3] );

  m3[0] = vabsq_s16( vaddq_s16( m4[0], m4[1] ) );
  m3[1] = vabdq_s16( m4[0], m4[1] );
  m3[2] = vabsq_s16( vaddq_s16( m4[2], m4[3] ) );
  m3[3] = vabdq_s16( m4[2], m4[3] );

  uint32_t absDC = ( uint32_t )vgetq_lane_s16( m3[0], 0 );

  int32x4_t total = horizontal_add_long_4d_s16x8( m3[0], m3[1], m3[2], m3[3] );
  uint32_t sad = ( uint32_t )horizontal_add_s32x4( total );

  sad -= absDC;
  sad += absDC >> 2;
  sad = ( uint32_t )( ( double )sad / sqrt( 4.0 * 8.0 ) * 2.0 );

  return sad;
}

template<bool fastHad>
Distortion xGetHADs_neon( const DistParam &rcDtParam )
{
  const Pel*  piOrg = rcDtParam.org.buf;
  const Pel*  piCur = rcDtParam.cur.buf;
  const int iRows = rcDtParam.org.height;
  const int iCols = rcDtParam.org.width;
  const int iStrideCur = rcDtParam.cur.stride;
  const int iStrideOrg = rcDtParam.org.stride;
  const int iBitDepth  = rcDtParam.bitDepth;

  int  x, y;
  Distortion uiSum = 0;

  if( iCols > iRows && ( iCols & 15 ) == 0 && ( iRows & 7 ) == 0 )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHAD16x8_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8*iStrideOrg;
      piCur += 8*iStrideCur;
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
      piOrg += 4*iStrideOrg;
      piCur += 4*iStrideCur;
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
#ifdef USE_AVX2
  else if( fastHad && ( ( ( iRows | iCols ) & 31 ) == 0 ) && ( iRows == iCols ) )
  {
    for( y = 0; y < iRows; y += 32 )
    {
      for( x = 0; x < iCols; x += 32 )
      {
        uiSum += xCalcHAD32x32_fast_AVX2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
      }
      piOrg += 32 * iStrideOrg;
      piCur += 32 * iStrideCur;
    }
  }
#endif
  else if( fastHad && ( ( ( iRows | iCols ) & 31 ) == 0 ) && ( iRows == iCols ) )
  {
    for( y = 0; y < iRows; y += 16 )
    {
      for( x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHAD16x16_fast_Neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
      }
      piOrg += 16 * iStrideOrg;
      piCur += 16 * iStrideCur;
    }
  }
  else if( ( ( ( iRows | iCols ) & 7 ) == 0 ) && ( iRows == iCols ) )
  {
    for( y = 0; y<iRows; y += 8 )
    {
      {
        for( x = 0; x < iCols; x += 8 )
        {
          uiSum += xCalcHAD8x8_Neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
        }
      }
      piOrg += 8*iStrideOrg;
      piCur += 8*iStrideCur;
    }
  }
  else if( ( iRows % 4 == 0 ) && ( iCols % 4 == 0 ) )
  {
    for( y = 0; y < iRows; y += 4 )
    {
      for( x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHAD4x4_SSE( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
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
#endif  // defined( TARGET_SIMD_X86 )

template<bool isCalCentrePos>
void xGetSADX5_16xN_neon_impl( const DistParam& rcDtParam, Distortion* cost )
{
  int        i, j;
  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf - 4;
  int        height     = rcDtParam.org.height;
  int        iSubShift  = rcDtParam.subShift;
  int        iSubStep   = ( 1 << iSubShift );
  ptrdiff_t  iStrideCur = rcDtParam.cur.stride * iSubStep;
  ptrdiff_t  iStrideOrg = rcDtParam.org.stride * iSubStep;

  int16x8_t sum0 = vdupq_n_s16( 0 );
  int16x8_t sum1 = vdupq_n_s16( 0 );
  int16x8_t sum2 = vdupq_n_s16( 0 );
  int16x8_t sum3 = vdupq_n_s16( 0 );
  int16x8_t sum4 = vdupq_n_s16( 0 );

  for( i = 0; i < height; i += iSubStep )
  {
    for( j = 0; j < 16; j += 8 )
    {
      int16x8_t s0 = vld1q_s16( piOrg + j + 0 );
      int16x8_t s1 = vld1q_s16( piCur + j + 0 );
      int16x8_t s2 = vcombine_s16( vld1_s16( piOrg + j + 8 ), vdup_n_s16( 0 ) );
      int16x8_t s3 = vcombine_s16( vld1_s16( piCur + j + 8 ), vdup_n_s16( 0 ) );

      int16x8_t org0, org1, org2, org3, org4;
      org0 = s0;
      org1 = vextq_s16( s0, s2, 1 );
      if( isCalCentrePos )
        org2 = vextq_s16( s0, s2, 2 );
      org3 = vextq_s16( s0, s2, 3 );
      org4 = vextq_s16( s0, s2, 4 );

      int16x8_t cur0, cur1, cur2, cur3, cur4;
      cur4 = s1;
      cur0 = vextq_s16( s1, s3, 4 );
      cur1 = vextq_s16( s1, s3, 3 );
      if( isCalCentrePos )
        cur2 = vextq_s16( s1, s3, 2 );
      cur3 = vextq_s16( s1, s3, 1 );

      sum0 = vabaq_s16( sum0, org0, cur0 );   // komplett insane
      sum1 = vabaq_s16( sum1, org1, cur1 );
      if( isCalCentrePos )
        sum2 = vabaq_s16( sum2, org2, cur2 );
      sum3 = vabaq_s16( sum3, org3, cur3 );
      sum4 = vabaq_s16( sum4, org4, cur4 );
    }

    INCY( piOrg, iStrideOrg );
    INCY( piCur, iStrideCur );
  }

  int32x4_t sum = horizontal_add_long_4d_s16x8( sum0, sum1, sum3, sum4 );

  int32x4_t sumTwo;
  if( isCalCentrePos )
    sumTwo = vdupq_n_s32( horizontal_add_long_s16x8( sum2 ) );

  // vshlq_n_s32 doesnt work because iSubShift ist not a const.
  sum = vshlq_s32( sum, vdupq_n_s32( iSubShift ) );
  if( isCalCentrePos )
    sumTwo = vshlq_s32( sumTwo, vdupq_n_s32( iSubShift ) );

  sum = vshrq_n_s32( sum, ( 1 + ( DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth ) ) ) );
  if( isCalCentrePos )
    sumTwo = vshrq_n_s32( sumTwo, ( 1 + ( DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth ) ) ) );

  vst1q_s32( (int32_t*) &cost[0], vzipq_s32( sum, vdupq_n_s32(0) ).val[0] );
  if (isCalCentrePos) cost[2] = (vgetq_lane_s32(sumTwo,0));
  vst1q_s32( (int32_t*) &cost[3], vzipq_s32( sum, vdupq_n_s32(0) ).val[1] );
}

void xGetSADX5_16xN_neon(const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos)
{
  if( rcDtParam.bitDepth > 10 )
  {
    RdCost::xGetSAD16X5( rcDtParam, cost, isCalCentrePos );
    return;
  }

  if( isCalCentrePos )
    xGetSADX5_16xN_neon_impl<true>( rcDtParam, cost );
  else
    xGetSADX5_16xN_neon_impl<false>( rcDtParam, cost );
}

static inline Distortion xGetSAD_generic_neon( const DistParam& rcDtParam, const int width )
{
  if( width < 4 )
  {
    return RdCost::xGetSAD( rcDtParam );
  }

  const int16_t* src1 = rcDtParam.org.buf;
  const int16_t* src2 = rcDtParam.cur.buf;
  int height = rcDtParam.org.height;
  int subShift = rcDtParam.subShift;
  int subStep = 1 << subShift;
  const int strideSrc1 = rcDtParam.org.stride * subStep;
  const int strideSrc2 = rcDtParam.cur.stride * subStep;

  uint32x4_t sum_u32[2] = { vdupq_n_u32( 0 ), vdupq_n_u32( 0 ) };
  Distortion sum = 0;
  do
  {
    int w = width;

    const int16_t* src1_ptr = src1;
    const int16_t* src2_ptr = src2;

    while( w >= 16 )
    {
      const int16x8_t s1_lo = vld1q_s16( src1_ptr );
      const int16x8_t s1_hi = vld1q_s16( src1_ptr + 8 );
      const int16x8_t s2_lo = vld1q_s16( src2_ptr );
      const int16x8_t s2_hi = vld1q_s16( src2_ptr + 8 );

      const uint16x8_t abs_lo = vreinterpretq_u16_s16( vabdq_s16( s1_lo, s2_lo ) );
      const uint16x8_t abs_hi = vreinterpretq_u16_s16( vabdq_s16( s1_hi, s2_hi ) );

      sum_u32[0] = vpadalq_u16( sum_u32[0], abs_lo );
      sum_u32[1] = vpadalq_u16( sum_u32[1], abs_hi );

      src1_ptr += 16;
      src2_ptr += 16;
      w -= 16;
    }

    if( w >= 8 )
    {
      const int16x8_t s1 = vld1q_s16( src1_ptr );
      const int16x8_t s2 = vld1q_s16( src2_ptr );

      const uint16x8_t abs = vreinterpretq_u16_s16( vabdq_s16( s1, s2 ) );
      sum_u32[0] = vpadalq_u16( sum_u32[0], abs );

      src1_ptr += 8;
      src2_ptr += 8;
      w -= 8;
    }

    if( w >= 4 )
    {
      const int16x4_t s1 = vld1_s16( src1_ptr );
      const int16x4_t s2 = vld1_s16( src2_ptr );

      const uint16x4_t abs = vreinterpret_u16_s16( vabd_s16( s1, s2 ) );
      sum_u32[0] = vaddw_u16( sum_u32[0], abs );

      src1_ptr += 4;
      src2_ptr += 4;
      w -= 4;
    }

    while( w != 0 )
    {
      sum += abs( src1_ptr[w - 1] - src2_ptr[w - 1] );

      w--;
    }

    src1 += strideSrc1;
    src2 += strideSrc2;
    height -= subStep;
  } while( height != 0 );

  sum += horizontal_add_u32x4( vaddq_u32( sum_u32[0], sum_u32[1] ) );
  sum <<= subShift;
  return sum >> DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth );
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

#if defined( TARGET_SIMD_X86 )
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
#endif // defined( TARGET_SIMD_X86 )

  m_afpDistortFunc[0][DF_SAD4   ] = xGetSAD_NxN_neon<4>;
  m_afpDistortFunc[0][DF_SAD8   ] = xGetSAD_NxN_neon<8>;
  m_afpDistortFunc[0][DF_SAD16  ] = xGetSAD_NxN_neon<16>;
  m_afpDistortFunc[0][DF_SAD32  ] = xGetSAD_NxN_neon<32>;
  m_afpDistortFunc[0][DF_SAD64  ] = xGetSAD_NxN_neon<64>;
  m_afpDistortFunc[0][DF_SAD128]  = xGetSAD_NxN_neon<128>;

  m_wtdPredPtr[0] = lumaWeightedSSE_neon<0>;
  m_wtdPredPtr[1] = lumaWeightedSSE_neon<1>;
  m_fxdWtdPredPtr = fixWeightedSSE_neon;
}

#endif  // defined( TARGET_SIMD_ARM )

}   // namespace vvenc

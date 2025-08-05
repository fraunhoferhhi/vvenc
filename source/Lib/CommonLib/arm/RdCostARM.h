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

/** \file     RdCostARM.h
    \brief    RD cost computation class, SIMD version
*/

#include <math.h>
#include <limits>

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "neon/sum_neon.h"
#include "../RdCost.h"

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
# include "../x86/RdCostX86.h"
#endif

namespace vvenc
{

static inline int32x4_t neon_madd_16( int16x8_t a, int16x8_t b )
{
  int32x4_t c = vmull_s16( vget_low_s16( a ), vget_low_s16( b ) );
  int32x4_t d = vmull_s16( vget_high_s16( a ), vget_high_s16( b ) );
  return pairwise_add_s32x4( c, d );
}

#if ENABLE_SIMD_OPT_DIST && defined( TARGET_SIMD_ARM )

// The xGetHADs_ARMSIMD functions depend on the SIMDe kernels being enabled
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

  int16x8x2_t m1[8], m2[8];
  int32x4x2_t m3[8], m4[8];
  
  int16x8_t r0, r1;
  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );

  m2[0].val[0] = vsubq_s16( r0, r1 ); // 11bit

  piCur += iStrideCur;
  piOrg += iStrideOrg;

  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );

  m2[1].val[0] = vsubq_s16( r0, r1 ); // 11bit

  piCur += iStrideCur;
  piOrg += iStrideOrg;

  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );

  m2[2].val[0] = vsubq_s16( r0, r1 ); // 11bit

  piCur += iStrideCur;
  piOrg += iStrideOrg;

  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );

  m2[3].val[0] = vsubq_s16( r0, r1 ); // 11bit

  piCur += iStrideCur;
  piOrg += iStrideOrg;

  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );

  m2[4].val[0] = vsubq_s16( r0, r1 ); // 11bit

  piCur += iStrideCur;
  piOrg += iStrideOrg;

  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );

  m2[5].val[0] = vsubq_s16( r0, r1 ); // 11bit

  piCur += iStrideCur;
  piOrg += iStrideOrg;

  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );

  m2[6].val[0] = vsubq_s16( r0, r1 ); // 11bit

  piCur += iStrideCur;
  piOrg += iStrideOrg;

  r0 = vld1q_s16( piOrg );
  r1 = vld1q_s16( piCur );

  m2[7].val[0] = vsubq_s16( r0, r1 ); // 11bit

  piCur += iStrideCur;
  piOrg += iStrideOrg;

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

  return sad;
}

template<ARM_VEXT vext, bool fastHad>
Distortion RdCost::xGetHADs_ARMSIMD( const DistParam &rcDtParam )
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
#ifdef USE_AVX2
        uiSum += xCalcHAD16x8_AVX2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
#else
        uiSum += xCalcHAD16x8_SSE( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
#endif
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
#ifdef USE_AVX2
        uiSum += xCalcHAD8x16_AVX2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
#else
        uiSum += xCalcHAD8x16_SSE( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
#endif
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
        uiSum += xCalcHAD8x4_SSE( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
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
        uiSum += xCalcHAD4x8_SSE( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur, iBitDepth );
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
        uiSum += xCalcHADs2x2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
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

template<ARM_VEXT vext>
Distortion RdCost::xGetHAD2SADs_ARMSIMD( const DistParam &rcDtParam )
{ 
  Distortion distHad = xGetHADs_ARMSIMD<vext, false>( rcDtParam );
  Distortion distSad = RdCost::xGetSAD_SIMD<SIMD_EVERYWHERE_EXTENSION_LEVEL>( rcDtParam );

  return std::min( distHad, 2*distSad);
}
#endif  // defined( TARGET_SIMD_X86 )

template<ARM_VEXT vext, bool isCalCentrePos>
void xGetSADX5_16xN_SIMDImp_ARM( const DistParam& rcDtParam, Distortion* cost )
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

template <ARM_VEXT vext>
void RdCost::xGetSADX5_16xN_SIMD_ARM(const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos)
{
  if( rcDtParam.bitDepth > 10 )
  {
    RdCost::xGetSAD16X5( rcDtParam, cost, isCalCentrePos );
    return;
  }

  if( isCalCentrePos )
    xGetSADX5_16xN_SIMDImp_ARM<vext, true>( rcDtParam, cost );
  else
    xGetSADX5_16xN_SIMDImp_ARM<vext, false>( rcDtParam, cost );
}

template< int iWidth, ARM_VEXT vext >
Distortion RdCost::xGetSAD_NxN_ARMSIMD( const DistParam &rcDtParam )
{

  const short* pSrc1   = (const short*)rcDtParam.org.buf;
  const short* pSrc2   = (const short*)rcDtParam.cur.buf;
  int  iRows           = rcDtParam.org.height;
  int  iSubShift       = rcDtParam.subShift;
  int  iSubStep        = ( 1 << iSubShift );
  const int iStrideSrc1 = rcDtParam.org.stride * iSubStep;
  const int iStrideSrc2 = rcDtParam.cur.stride * iSubStep;

  uint32_t uiSum = 0;
  int16x8_t vzero_16 = vdupq_n_s16(0);

  if( iWidth == 4 )
  {
    if( iRows == 4 && iSubShift == 0 )
    {
      int16x8_t vsrc1 = vcombine_s16( vld1_s16( ( const int16_t* )pSrc1 ),  vld1_s16( ( const int16_t* )( &pSrc1[iStrideSrc1] ) ) );
      int16x8_t vsrc2 = vcombine_s16( vld1_s16( ( const int16_t* )pSrc2 ),  vld1_s16( ( const int16_t* )( &pSrc2[iStrideSrc2] ) ) );
      int32x4_t vsum =
        vmovl_s16( vget_low_s16( pairwise_add_s16x8( vabsq_s16( vsubq_s16( vsrc1, vsrc2 ) ), vzero_16 ) ) );
      vsrc1 = vcombine_s16( vld1_s16( ( const int16_t* )( &pSrc1[2 * iStrideSrc1] ) ),  vld1_s16( ( const int16_t* )( &pSrc1[3 * iStrideSrc1] ) ) );
      vsrc2 = vcombine_s16( vld1_s16( ( const int16_t* )( &pSrc2[2 * iStrideSrc2] ) ),  vld1_s16( ( const int16_t* )( &pSrc2[3 * iStrideSrc2] ) ) );
      vsum = vaddq_s32(
        vsum, vmovl_s16( vget_low_s16( pairwise_add_s16x8( vabsq_s16( vsubq_s16( vsrc1, vsrc2 ) ), vzero_16 ) ) ) );
      uiSum = horizontal_add_s32x4( vsum );
    }
    else
    {
      int32x4_t vsum32 = vdupq_n_s32(0);
      for( int iY = 0; iY < iRows; iY += iSubStep )
      {
        int32x4_t vsrc1 = vmovl_s16( vld1_s16( ( const int16_t* )pSrc1 ) );
        int32x4_t vsrc2 = vmovl_s16( vld1_s16( ( const int16_t* )pSrc2 ) );
        vsum32 = vaddq_s32( vsum32, vabsq_s32( vsubq_s32( vsrc1, vsrc2 ) ) );

        pSrc1 += iStrideSrc1;
        pSrc2 += iStrideSrc2;
      }
      uiSum = horizontal_add_s32x4( vsum32 );
    }
  }
  else
  {    
    static constexpr bool earlyExitAllowed = iWidth >= 64;
    int32x4_t vsum32 = vdupq_n_s32( 0 );
    int checkExit = 3;

    for( int iY = 0; iY < iRows; iY+=iSubStep )
    {
      int16x8_t vsrc1  = vld1q_s16( ( const int16_t* )( pSrc1 ) );
      int16x8_t vsrc2  = vld1q_s16( ( const int16_t* )( pSrc2 ) );
      int16x8_t vsum16 = vabsq_s16( vsubq_s16( vsrc1, vsrc2 ) );

      if( iWidth >= 16 )
      {
        vsrc1  = vld1q_s16( ( const int16_t* )( &pSrc1[8] ) );
        vsrc2  = vld1q_s16( ( const int16_t* )( &pSrc2[8] ) );
        vsum16 = vaddq_s16( vsum16, vabsq_s16( vsubq_s16( vsrc1, vsrc2 ) ) );

        for( int iX = 16; iX < iWidth; iX += 16 )
        {
          vsrc1  = vld1q_s16( ( const int16_t* )( &pSrc1[iX] ) );
          vsrc2  = vld1q_s16( ( const int16_t* )( &pSrc2[iX] ) );
          vsum16 = vaddq_s16( vsum16, vabsq_s16( vsubq_s16( vsrc1, vsrc2 ) ) );
          
          vsrc1  = vld1q_s16( ( const int16_t* )( &pSrc1[iX + 8] ) );
          vsrc2  = vld1q_s16( ( const int16_t* )( &pSrc2[iX + 8] ) );
          vsum16 = vaddq_s16( vsum16, vabsq_s16( vsubq_s16( vsrc1, vsrc2 ) ) );
        }
      }

      int32x4_t vsumtemp = vpaddlq_s16( vsum16);

      if( earlyExitAllowed ) vsum32 = pairwise_add_s32x4( vsum32, vsumtemp );
      else                   vsum32 = vaddq_s32 ( vsum32, vsumtemp );

      pSrc1   += iStrideSrc1;
      pSrc2   += iStrideSrc2;

      if( earlyExitAllowed && checkExit == 0 )
      {
        Distortion distTemp = vgetq_lane_s32(vsum32, 0); 
        distTemp <<= iSubShift;
        distTemp >>= DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth );
        if( distTemp > rcDtParam.maximumDistortionForEarlyExit ) return distTemp;
        checkExit = 3;
      }
      else if( earlyExitAllowed )
      {
        checkExit--;
      }
    }
    uiSum = horizontal_add_s32x4( vsum32 );
  }

  uiSum <<= iSubShift;
  return uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);
}

static inline int16x8_t reverse_vector_s16( int16x8_t x )
{
#if REAL_TARGET_AARCH64
  static const uint8_t shuffle_table[ 16 ] = { 14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1 };
  uint8x16_t shuffle_indices               = vld1q_u8( shuffle_table );
  return vreinterpretq_s16_u8( vqtbl1q_u8( vreinterpretq_u8_s16( x ), shuffle_indices ) );
#else
  int16x8_t rev_halves = vrev64q_s16( x );
  return vcombine_s16( vget_high_s16( rev_halves ), vget_low_s16( rev_halves ) );
#endif
}

template<ARM_VEXT vext>
Distortion RdCost::xGetSADwMask_ARMSIMD( const DistParam& rcDtParam )
{
  if (rcDtParam.org.width < 4 || rcDtParam.bitDepth > 10 || rcDtParam.applyWeight)
    return RdCost::xGetSADwMask(rcDtParam);

  const short *src1       = (const short *) rcDtParam.org.buf;
  const short *src2       = (const short *) rcDtParam.cur.buf;
  const short *weightMask = (const short *) rcDtParam.mask;
  int          rows       = rcDtParam.org.height;
  int          cols       = rcDtParam.org.width;
  int          subShift   = rcDtParam.subShift;
  int          subStep    = (1 << subShift);
  const int    strideSrc1 = rcDtParam.org.stride * subStep;
  const int    strideSrc2 = rcDtParam.cur.stride * subStep;
  const int    strideMask = rcDtParam.maskStride * subStep;

  Distortion sum = 0;

  int32x4_t vsum32 = vdupq_n_s32( 0 );

  for (int y = 0; y < rows; y += subStep)
  {
    for (int x = 0; x < cols; x += 8)
    {
      int16x8_t vsrc1  = vld1q_s16( ( const int16_t* )(&src1[x] ) );
      int16x8_t vsrc2  = vld1q_s16( ( const int16_t* )(&src2[x] ) );
      int16x8_t vmask;
      if (rcDtParam.stepX == -1)
      {
        vmask = vld1q_s16( ( const int16_t* )( ( &weightMask[ x ] ) - ( x << 1 ) - ( 8 - 1 ) ) );
        vmask = reverse_vector_s16( vmask );
      }
      else
      {
        vmask = vld1q_s16( ( const int16_t* ) (&weightMask[x]));
      }
      vsum32 = vaddq_s32(vsum32, neon_madd_16(vmask, vabsq_s16(vsubq_s16(vsrc1, vsrc2))));
    }
    src1 += strideSrc1;
    src2 += strideSrc2;
    weightMask += strideMask;
  }
  sum = horizontal_add_s32x4( vsum32 );
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

template<ARM_VEXT vext>
void RdCost::_initRdCostARM()
{
  m_afpDistortFuncX5[1] = xGetSADX5_16xN_SIMD_ARM<vext>;

#if defined( TARGET_SIMD_X86 )
  m_afpDistortFunc[0][DF_HAD_2SAD ] = RdCost::xGetHAD2SADs_ARMSIMD<vext>;

  m_afpDistortFunc[0][DF_HAD]     = RdCost::xGetHADs_ARMSIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD2]    = RdCost::xGetHADs_ARMSIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD4]    = RdCost::xGetHADs_ARMSIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD8]    = RdCost::xGetHADs_ARMSIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD16]   = RdCost::xGetHADs_ARMSIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD32]   = RdCost::xGetHADs_ARMSIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD64]   = RdCost::xGetHADs_ARMSIMD<vext, false>;
  m_afpDistortFunc[0][DF_HAD128]  = RdCost::xGetHADs_ARMSIMD<vext, false>;

  m_afpDistortFunc[0][DF_HAD_fast]     = RdCost::xGetHADs_ARMSIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD2_fast]    = RdCost::xGetHADs_ARMSIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD4_fast]    = RdCost::xGetHADs_ARMSIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD8_fast]    = RdCost::xGetHADs_ARMSIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD16_fast]   = RdCost::xGetHADs_ARMSIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD32_fast]   = RdCost::xGetHADs_ARMSIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD64_fast]   = RdCost::xGetHADs_ARMSIMD<vext, true>;
  m_afpDistortFunc[0][DF_HAD128_fast]  = RdCost::xGetHADs_ARMSIMD<vext, true>;

  m_afpDistortFunc[0][DF_SAD4   ] = xGetSAD_NxN_ARMSIMD<4,  vext>;
  m_afpDistortFunc[0][DF_SAD8   ] = xGetSAD_NxN_ARMSIMD<8,  vext>;
  m_afpDistortFunc[0][DF_SAD16  ] = xGetSAD_NxN_ARMSIMD<16, vext>;
  m_afpDistortFunc[0][DF_SAD32  ] = xGetSAD_NxN_ARMSIMD<32, vext>;
  m_afpDistortFunc[0][DF_SAD64  ] = xGetSAD_NxN_ARMSIMD<64, vext>;
  m_afpDistortFunc[0][DF_SAD128]  = xGetSAD_NxN_ARMSIMD<128, vext>;

  m_afpDistortFunc[0][DF_SAD_WITH_MASK] = xGetSADwMask_ARMSIMD<vext>;

#endif  // defined( TARGET_SIMD_X86 )

  m_wtdPredPtr[0] = lumaWeightedSSE_neon<0>;
  m_wtdPredPtr[1] = lumaWeightedSSE_neon<1>;
  m_fxdWtdPredPtr = fixWeightedSSE_neon;
}

template void RdCost::_initRdCostARM<SIMDARM>();

#endif  // defined( TARGET_SIMD_ARM )

}   // namespace vvenc

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
/**
 * \file AffineGradientSearch_neon.cpp
 * \brief Neon implementation of AffineGradientSearch for Arm.
 */

// ====================================================================================================================
// Includes
// ====================================================================================================================

#include <algorithm>

#include "../CommonDefARM.h"
#include "CommonLib/AffineGradientSearch.h"
#include "sum_neon.h"

//! \ingroup CommonLib
//! \{

#if ENABLE_SIMD_OPT_AFFINE_ME && defined( TARGET_SIMD_ARM )

namespace vvenc
{

static inline int64x2_t vmlal_s32_x2( int64x2_t acc, int32x4_t x, int32x4_t y )
{
  acc = vmlal_s32( acc, vget_low_s32( x ), vget_low_s32( y ) );
  acc = vmlal_s32( acc, vget_high_s32( x ), vget_high_s32( y ) );
  return acc;
}

template<bool b6Param>
void simdEqualCoeffComputer_neon( Pel* const pResidue, const int residueStride, Pel** const ppDerivate,
                                  const int derivateBufStride, const int width, const int height,
                                  int64_t ( *pEqualCoeff )[7] );

template<>
void simdEqualCoeffComputer_neon<false>( Pel* const pResidue, const int residueStride, Pel** const ppDerivate,
                                         const int derivateBufStride, const int width, const int height,
                                         int64_t ( *pEqualCoeff )[7] )
{
  CHECK( height < 1, "Height must be >= 1" );
  CHECK( width < 4, "Width must be >= 4" );
  CHECK( height > 128, "Height must be <= 128" );
  CHECK( width > 128, "Width must be <= 128" );
  CHECK( ( height & ( height - 1 ) ) != 0, "Height must be power of two" );
  CHECK( ( width & ( width - 1 ) ) != 0, "Width must be power of two" );

  int64x2_t out64_c0_r0 = vdupq_n_s64( 0 );
  int64x2_t out64_c0_r1 = vdupq_n_s64( 0 );
  int64x2_t out64_c0_r2 = vdupq_n_s64( 0 );
  int64x2_t out64_c0_r3 = vdupq_n_s64( 0 );
  int64x2_t out64_c1_r1 = vdupq_n_s64( 0 );
  int64x2_t out64_c1_r2 = vdupq_n_s64( 0 );
  int64x2_t out64_c1_r3 = vdupq_n_s64( 0 );
  int64x2_t out64_c2_r2 = vdupq_n_s64( 0 );
  int64x2_t out64_c2_r3 = vdupq_n_s64( 0 );
  int64x2_t out64_c3_r3 = vdupq_n_s64( 0 );

  int64x2_t out64_c0_r4 = vdupq_n_s64( 0 );
  int64x2_t out64_c1_r4 = vdupq_n_s64( 0 );
  int64x2_t out64_c2_r4 = vdupq_n_s64( 0 );
  int64x2_t out64_c3_r4 = vdupq_n_s64( 0 );

  int h = 0;
  do
  {
    // 4x4 row centerpoint.
    const int cy = ( h & ~3 ) + 2;

    // Initialize int32x4_t accumulators to zero.
    int32x4_t out32_c0_r0 = vdupq_n_s32( 0 );
    int32x4_t out32_c0_r2 = vdupq_n_s32( 0 );
    int32x4_t out32_c0_r4 = vdupq_n_s32( 0 );
    int32x4_t out32_c2_r2 = vdupq_n_s32( 0 );
    int32x4_t out32_c2_r4 = vdupq_n_s32( 0 );

    int w = 0;
    do
    {
      const int drvIdx = h * derivateBufStride + w;
      const int resIdx = h * residueStride + w;
      const int cx = w + 2;

      int16x4_t iC0h = vld1_s16( &ppDerivate[0][drvIdx] );
      int16x4_t iC2h = vld1_s16( &ppDerivate[1][drvIdx] );
      int16x4_t res_h = vld1_s16( &pResidue[resIdx] );
      int32x4_t res = vmovl_s16( res_h );

      // For iC0 and iC2 multiplied by each other:
      // 13 bits * 13 bits => 26 bits. Have 32 bits of storage so can
      // accumulate 1 << 6 = 64 times.
      // Max number of inner loop iterations is 128/4 = 32, so fine to
      // accumulate into int32 for these cases.

      int32x4_t iC0 = vmovl_s16( iC0h );
      int32x4_t iC1 = vmull_n_s16( iC0h, cx );
      iC1 = vmlal_n_s16( iC1, iC2h, cy );
      int32x4_t iC2 = vmovl_s16( iC2h );
      int32x4_t iC3 = vmull_n_s16( iC0h, cy );
      iC3 = vmlsl_n_s16( iC3, iC2h, cx );

      out32_c0_r0 = vmlal_s16( out32_c0_r0, iC0h, iC0h );
      out64_c0_r1 = vmlal_s32_x2( out64_c0_r1, iC1, iC0 );
      out32_c0_r2 = vmlal_s16( out32_c0_r2, iC2h, iC0h );
      out64_c0_r3 = vmlal_s32_x2( out64_c0_r3, iC3, iC0 );
      // Skip compute due to symmetry.
      // out32_c1_r0 = out32_c0_r1;
      out64_c1_r1 = vmlal_s32_x2( out64_c1_r1, iC1, iC1 );
      out64_c1_r2 = vmlal_s32_x2( out64_c1_r2, iC2, iC1 );
      out64_c1_r3 = vmlal_s32_x2( out64_c1_r3, iC3, iC1 );
      // Skip compute due to symmetry.
      // out32_c2_r0 = out32_c0_r2;
      // out32_c2_r1 = out32_c1_r2;
      out32_c2_r2 = vmlal_s16( out32_c2_r2, iC2h, iC2h );
      out64_c2_r3 = vmlal_s32_x2( out64_c2_r3, iC3, iC2 );
      // Skip compute due to symmetry.
      // out32_c3_r0 = out32_c0_r3;
      // out32_c3_r1 = out32_c1_r3;
      // out32_c3_r2 = out32_c2_r3;
      out64_c3_r3 = vmlal_s32_x2( out64_c3_r3, iC3, iC3 );

      // Final Row: iC[] x Residue
      out32_c0_r4 = vmlal_s16( out32_c0_r4, iC0h, res_h );
      out64_c1_r4 = vmlal_s32_x2( out64_c1_r4, iC1, res );
      out32_c2_r4 = vmlal_s16( out32_c2_r4, iC2h, res_h );
      out64_c3_r4 = vmlal_s32_x2( out64_c3_r4, iC3, res );

      w += 4;
    } while( w != width );

    // Promote int32x4 to int64x2 after every inner loop.
    out64_c0_r0 = vpadalq_s32( out64_c0_r0, out32_c0_r0 );
    out64_c0_r2 = vpadalq_s32( out64_c0_r2, out32_c0_r2 );
    out64_c2_r2 = vpadalq_s32( out64_c2_r2, out32_c2_r2 );
    out64_c0_r4 = vpadalq_s32( out64_c0_r4, out32_c0_r4 );
    out64_c2_r4 = vpadalq_s32( out64_c2_r4, out32_c2_r4 );
  } while( ++h != height );

  int64x2_t out64_c0_r01 = pairwise_add_s64x2( out64_c0_r0, out64_c0_r1 );
  int64x2_t out64_c0_r23 = pairwise_add_s64x2( out64_c0_r2, out64_c0_r3 );
  int64x2_t out64_c1_r23 = pairwise_add_s64x2( out64_c1_r2, out64_c1_r3 );
  int64x2_t out64_c2_r23 = pairwise_add_s64x2( out64_c2_r2, out64_c2_r3 );
  int64x2_t out64_c1r1_c3r3 = pairwise_add_s64x2( out64_c1_r1, out64_c3_r3 );

  // Store all outputs, copy the symmetric ones.
  vst1q_s64( &pEqualCoeff[1][0], out64_c0_r01 );
  vst1q_s64( &pEqualCoeff[1][2], out64_c0_r23 );
  pEqualCoeff[2][0] = pEqualCoeff[1][1];
  pEqualCoeff[2][1] = vgetq_lane_s64( out64_c1r1_c3r3, 0 );
  vst1q_s64( &pEqualCoeff[2][2], out64_c1_r23 );
  pEqualCoeff[3][0] = pEqualCoeff[1][2];
  pEqualCoeff[3][1] = pEqualCoeff[2][2];
  vst1q_s64( &pEqualCoeff[3][2], out64_c2_r23 );
  pEqualCoeff[4][0] = pEqualCoeff[1][3];
  pEqualCoeff[4][1] = pEqualCoeff[2][3];
  pEqualCoeff[4][2] = pEqualCoeff[3][3];
  pEqualCoeff[4][3] = vgetq_lane_s64( out64_c1r1_c3r3, 1 );

  // Final row: Left-shift by 3.
  pEqualCoeff[1][4] = horizontal_add_s64x2( out64_c0_r4 ) << 3;
  pEqualCoeff[2][4] = horizontal_add_s64x2( out64_c1_r4 ) << 3;
  pEqualCoeff[3][4] = horizontal_add_s64x2( out64_c2_r4 ) << 3;
  pEqualCoeff[4][4] = horizontal_add_s64x2( out64_c3_r4 ) << 3;
};

template<>
void simdEqualCoeffComputer_neon<true>( Pel* const pResidue, const int residueStride, Pel** const ppDerivate,
                                        const int derivateBufStride, const int width, const int height,
                                        int64_t ( *pEqualCoeff )[7] )
{
  CHECK( height < 1, "Height must be >= 1" );
  CHECK( width < 4, "Width must be >= 4" );
  CHECK( height > 128, "Height must be <= 128" );
  CHECK( width > 128, "Width must be <= 128" );
  CHECK( ( height & ( height - 1 ) ) != 0, "Height must be power of two" );
  CHECK( ( width & ( width - 1 ) ) != 0, "Width must be power of two" );

  int64x2_t out64_c1_r0 = vdupq_n_s64( 0 );
  int64x2_t out64_c1_r1 = vdupq_n_s64( 0 );
  int64x2_t out64_c1_r2 = vdupq_n_s64( 0 );
  int64x2_t out64_c1_r3 = vdupq_n_s64( 0 );
  int64x2_t out64_c1_r4 = vdupq_n_s64( 0 );
  int64x2_t out64_c1_r5 = vdupq_n_s64( 0 );
  int64x2_t out64_c2_r1 = vdupq_n_s64( 0 );
  int64x2_t out64_c2_r2 = vdupq_n_s64( 0 );
  int64x2_t out64_c2_r3 = vdupq_n_s64( 0 );
  int64x2_t out64_c2_r4 = vdupq_n_s64( 0 );
  int64x2_t out64_c2_r5 = vdupq_n_s64( 0 );
  int64x2_t out64_c3_r2 = vdupq_n_s64( 0 );
  int64x2_t out64_c3_r3 = vdupq_n_s64( 0 );
  int64x2_t out64_c3_r4 = vdupq_n_s64( 0 );
  int64x2_t out64_c3_r5 = vdupq_n_s64( 0 );
  int64x2_t out64_c4_r3 = vdupq_n_s64( 0 );
  int64x2_t out64_c4_r4 = vdupq_n_s64( 0 );
  int64x2_t out64_c4_r5 = vdupq_n_s64( 0 );
  int64x2_t out64_c5_r4 = vdupq_n_s64( 0 );
  int64x2_t out64_c5_r5 = vdupq_n_s64( 0 );
  int64x2_t out64_c6_r5 = vdupq_n_s64( 0 );

  int64x2_t out64_c1_r6 = vdupq_n_s64( 0 );
  int64x2_t out64_c2_r6 = vdupq_n_s64( 0 );
  int64x2_t out64_c3_r6 = vdupq_n_s64( 0 );
  int64x2_t out64_c4_r6 = vdupq_n_s64( 0 );
  int64x2_t out64_c5_r6 = vdupq_n_s64( 0 );
  int64x2_t out64_c6_r6 = vdupq_n_s64( 0 );

  int h = 0;
  do
  {
    // 4x4 row centerpoint.
    const int cy = ( h & ~3 ) + 2;

    int32x4_t out32_c1_r0 = vdupq_n_s32( 0 );
    int32x4_t out32_c1_r2 = vdupq_n_s32( 0 );
    int32x4_t out32_c3_r2 = vdupq_n_s32( 0 );

    int32x4_t out32_c1_r6 = vdupq_n_s32( 0 );
    int32x4_t out32_c3_r6 = vdupq_n_s32( 0 );

    int w = 0;
    do
    {
      const int drvIdx = h * derivateBufStride + w;
      const int resIdx = h * residueStride + w;
      const int cx = w + 2;

      int16x4_t iC0h = vld1_s16( &ppDerivate[0][drvIdx] );
      int16x4_t iC2h = vld1_s16( &ppDerivate[1][drvIdx] );
      int16x4_t res_h = vld1_s16( &pResidue[resIdx] );
      int32x4_t res = vmovl_s16( res_h );

      int32x4_t iC0 = vmovl_s16( iC0h );       // 13 bits
      int32x4_t iC1 = vmull_n_s16( iC0h, cx ); // 13+7 => 20 bits
      int32x4_t iC2 = vmovl_s16( iC2h );       // 13 bits
      int32x4_t iC3 = vmull_n_s16( iC2h, cx ); // 13+7 => 20 bits
      int32x4_t iC4 = vmull_n_s16( iC0h, cy ); // 13+7 => 20 bits
      int32x4_t iC5 = vmull_n_s16( iC2h, cy ); // 13+7 => 20 bits

      // For iC0 and iC2 multiplied by each other:
      // 13 bits * 13 bits => 26 bits. Have 32 bits of storage so can
      // accumulate 1 << 6 = 64 times.
      // Max number of inner loop iterations is 128/4 = 32, so fine to
      // accumulate into int32 for these cases.

      // Row 0-5.
      out32_c1_r0 = vmlal_s16( out32_c1_r0, iC0h, iC0h );
      out64_c1_r1 = vmlal_s32_x2( out64_c1_r1, iC1, iC0 );
      out32_c1_r2 = vmlal_s16( out32_c1_r2, iC2h, iC0h );
      out64_c1_r3 = vmlal_s32_x2( out64_c1_r3, iC3, iC0 );
      out64_c1_r4 = vmlal_s32_x2( out64_c1_r4, iC4, iC0 );
      out64_c1_r5 = vmlal_s32_x2( out64_c1_r5, iC5, iC0 );
      out64_c2_r1 = vmlal_s32_x2( out64_c2_r1, iC1, iC1 );
      out64_c2_r2 = vmlal_s32_x2( out64_c2_r2, iC2, iC1 );
      out64_c2_r3 = vmlal_s32_x2( out64_c2_r3, iC3, iC1 );
      out64_c2_r4 = vmlal_s32_x2( out64_c2_r4, iC4, iC1 );
      out64_c2_r5 = vmlal_s32_x2( out64_c2_r5, iC5, iC1 );
      out32_c3_r2 = vmlal_s16( out32_c3_r2, iC2h, iC2h );
      out64_c3_r3 = vmlal_s32_x2( out64_c3_r3, iC3, iC2 );
      out64_c3_r4 = vmlal_s32_x2( out64_c3_r4, iC4, iC2 );
      out64_c3_r5 = vmlal_s32_x2( out64_c3_r5, iC5, iC2 );
      out64_c4_r3 = vmlal_s32_x2( out64_c4_r3, iC3, iC3 );
      out64_c4_r4 = vmlal_s32_x2( out64_c4_r4, iC4, iC3 );
      out64_c4_r5 = vmlal_s32_x2( out64_c4_r5, iC5, iC3 );
      out64_c5_r4 = vmlal_s32_x2( out64_c5_r4, iC4, iC4 );
      out64_c5_r5 = vmlal_s32_x2( out64_c5_r5, iC5, iC4 );
      out64_c6_r5 = vmlal_s32_x2( out64_c6_r5, iC5, iC5 );

      // Final Row
      out32_c1_r6 = vmlal_s16( out32_c1_r6, iC0h, res_h );
      out64_c2_r6 = vmlal_s32_x2( out64_c2_r6, iC1, res );
      out32_c3_r6 = vmlal_s16( out32_c3_r6, iC2h, res_h );
      out64_c4_r6 = vmlal_s32_x2( out64_c4_r6, iC3, res );
      out64_c5_r6 = vmlal_s32_x2( out64_c5_r6, iC4, res );
      out64_c6_r6 = vmlal_s32_x2( out64_c6_r6, iC5, res );

      w += 4;
    } while( w != width );

    out64_c1_r0 = vpadalq_s32( out64_c1_r0, out32_c1_r0 );
    out64_c1_r2 = vpadalq_s32( out64_c1_r2, out32_c1_r2 );
    out64_c3_r2 = vpadalq_s32( out64_c3_r2, out32_c3_r2 );
    out64_c1_r6 = vpadalq_s32( out64_c1_r6, out32_c1_r6 );
    out64_c3_r6 = vpadalq_s32( out64_c3_r6, out32_c3_r6 );
  } while( ++h != height);

  int64x2_t out64_c1_r01 = pairwise_add_s64x2( out64_c1_r0, out64_c1_r1 );
  int64x2_t out64_c1_r23 = pairwise_add_s64x2( out64_c1_r2, out64_c1_r3 );
  int64x2_t out64_c1_r45 = pairwise_add_s64x2( out64_c1_r4, out64_c1_r5 );
  int64_t out64_c2_r01 = horizontal_add_s64x2( out64_c2_r1 );
  int64x2_t out64_c2_r23 = pairwise_add_s64x2( out64_c2_r2, out64_c2_r3 );
  int64x2_t out64_c2_r45 = pairwise_add_s64x2( out64_c2_r4, out64_c2_r5 );
  int64x2_t out64_c3_r23 = pairwise_add_s64x2( out64_c3_r2, out64_c3_r3 );
  int64x2_t out64_c3_r45 = pairwise_add_s64x2( out64_c3_r4, out64_c3_r5 );
  int64_t out64_c4_r23 = horizontal_add_s64x2( out64_c4_r3 );
  int64x2_t out64_c4_r45 = pairwise_add_s64x2( out64_c4_r4, out64_c4_r5 );
  int64x2_t out64_c5_r45 = pairwise_add_s64x2( out64_c5_r4, out64_c5_r5 );
  int64_t out64_c6_r45 = horizontal_add_s64x2( out64_c6_r5 );
  int64x2_t out64_c12_r6 = pairwise_add_s64x2( out64_c1_r6, out64_c2_r6 );
  int64x2_t out64_c34_r6 = pairwise_add_s64x2( out64_c3_r6, out64_c4_r6 );
  int64x2_t out64_c56_r6 = pairwise_add_s64x2( out64_c5_r6, out64_c6_r6 );

  // Store accumulated EqualCoeff.
  vst1q_s64( &pEqualCoeff[1][0], out64_c1_r01 );
  vst1q_s64( &pEqualCoeff[1][2], out64_c1_r23 );
  vst1q_s64( &pEqualCoeff[1][4], out64_c1_r45 );
  pEqualCoeff[2][1] = out64_c2_r01;
  vst1q_s64( &pEqualCoeff[2][2], out64_c2_r23 );
  vst1q_s64( &pEqualCoeff[2][4], out64_c2_r45 );
  vst1q_s64( &pEqualCoeff[3][2], out64_c3_r23 );
  vst1q_s64( &pEqualCoeff[3][4], out64_c3_r45 );
  pEqualCoeff[4][3] = out64_c4_r23;
  vst1q_s64( &pEqualCoeff[4][4], out64_c4_r45 );
  vst1q_s64( &pEqualCoeff[5][4], out64_c5_r45 );
  pEqualCoeff[6][5] = out64_c6_r45;

  // Copy symmetric outputs.
  pEqualCoeff[2][0] = pEqualCoeff[1][1];
  pEqualCoeff[3][0] = pEqualCoeff[1][2];
  pEqualCoeff[4][0] = pEqualCoeff[1][3];
  pEqualCoeff[5][0] = pEqualCoeff[1][4];
  pEqualCoeff[6][0] = pEqualCoeff[1][5];
  pEqualCoeff[3][1] = pEqualCoeff[2][2];
  pEqualCoeff[4][1] = pEqualCoeff[2][3];
  pEqualCoeff[5][1] = pEqualCoeff[2][4];
  pEqualCoeff[6][1] = pEqualCoeff[2][5];
  pEqualCoeff[4][2] = pEqualCoeff[3][3];
  pEqualCoeff[5][2] = pEqualCoeff[3][4];
  pEqualCoeff[6][2] = pEqualCoeff[3][5];
  pEqualCoeff[5][3] = pEqualCoeff[4][4];
  pEqualCoeff[6][3] = pEqualCoeff[4][5];
  pEqualCoeff[6][4] = pEqualCoeff[5][5];

  // Hoist << 3 outside into the final accumulate.
  out64_c12_r6 = vshlq_n_s64( out64_c12_r6, 3 );
  out64_c34_r6 = vshlq_n_s64( out64_c34_r6, 3 );
  out64_c56_r6 = vshlq_n_s64( out64_c56_r6, 3 );
  pEqualCoeff[1][6] = vgetq_lane_s64( out64_c12_r6, 0 );
  pEqualCoeff[2][6] = vgetq_lane_s64( out64_c12_r6, 1 );
  pEqualCoeff[3][6] = vgetq_lane_s64( out64_c34_r6, 0 );
  pEqualCoeff[4][6] = vgetq_lane_s64( out64_c34_r6, 1 );
  pEqualCoeff[5][6] = vgetq_lane_s64( out64_c56_r6, 0 );
  pEqualCoeff[6][6] = vgetq_lane_s64( out64_c56_r6, 1 );
}

template<>
void AffineGradientSearch::_initAffineGradientSearchARM<NEON>()
{
  m_EqualCoeffComputer[0] = simdEqualCoeffComputer_neon<false>;
  m_EqualCoeffComputer[1] = simdEqualCoeffComputer_neon<true>;
}

} // namespace vvenc

#endif // ENABLE_SIMD_OPT_AFFINE_ME && defined(TARGET_SIMD_ARM)

//! \}

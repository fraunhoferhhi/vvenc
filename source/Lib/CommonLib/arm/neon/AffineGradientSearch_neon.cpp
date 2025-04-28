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

  // Set max inner loop count for accumulating into int32x4.
  static constexpr int MAX_INT32_LOOP_CNT = 64;
  const int innerloop_max = std::min( MAX_INT32_LOOP_CNT, height * ( width >> 2 ) );

  int64x2_t out64_c0_r01 = vdupq_n_s64( 0 );
  int64x2_t out64_c0_r23 = vdupq_n_s64( 0 );
  int64x2_t out64_c1_r23 = vdupq_n_s64( 0 );
  int64x2_t out64_c2_r23 = vdupq_n_s64( 0 );
  int64x2_t out64_c1r1_c3r3 = vdupq_n_s64( 0 );

  int h = 0;

  // Right-shift by 1 to fit more bits into the int32x4 accumulator.
  int cy = ( h + 2 ) >> 1; // Max: (124+2)>>1 = 63

  const int innerloop_height = innerloop_max / ( width >> 2 );
  do
  {
    int32x4_t out_c0_r0 = vdupq_n_s32( 0 );
    int32x4_t out_c0_r1 = vdupq_n_s32( 0 );
    int32x4_t out_c0_r2 = vdupq_n_s32( 0 );
    int32x4_t out_c0_r3 = vdupq_n_s32( 0 );
    int32x4_t out_c0_r4 = vdupq_n_s32( 0 );

    int64x2_t out64_c1_r1 = vdupq_n_s64( 0 );
    int32x4_t out_c1_r2 = vdupq_n_s32( 0 );
    int64x2_t out64_c1_r3 = vdupq_n_s64( 0 );
    int32x4_t out_c1_r4 = vdupq_n_s32( 0 );

    int32x4_t out_c2_r2 = vdupq_n_s32( 0 );
    int32x4_t out_c2_r3 = vdupq_n_s32( 0 );
    int32x4_t out_c2_r4 = vdupq_n_s32( 0 );

    int64x2_t out64_c3_r3 = vdupq_n_s64( 0 );
    int32x4_t out_c3_r4 = vdupq_n_s32( 0 );

    int innerloop = innerloop_height;
    do
    {
      int w = 0;
      do
      {
        const int drvIdx = h * derivateBufStride + w;
        const int resIdx = h * residueStride + w;

        // Right-shift by 1 to fit more bits into the int32x4 accumulator.
        const int cx = ( w + 2 ) >> 1; // Max: (124+2)>>1 = 63

        // The max bits are written for each multiply and accumulate to ensure they fit within int16/32/64.
        int16x4_t iC0_s16 = vld1_s16( &ppDerivate[0][drvIdx] ); // 10bit signed
        int16x4_t iC2_s16 = vld1_s16( &ppDerivate[1][drvIdx] ); // 10bit signed
        int32x4_t iC0_s32 = vmovl_s16( iC0_s16 );               // 10bit
        int32x4_t iC2_s32 = vmovl_s16( iC2_s16 );               // 10bit
        int32x4_t iC1_s32 = vmull_n_s16( iC0_s16, cx );         // 17bit
        iC1_s32 = vmlal_n_s16( iC1_s32, iC2_s16, cy );          // 17bit
        int32x4_t iC3_s32 = vmull_n_s16( iC0_s16, cy );         // 17bit
        iC3_s32 = vmlsl_n_s16( iC3_s32, iC2_s16, cx );          // 17bit

        out_c0_r0 = vmlal_s16( out_c0_r0, iC0_s16, iC0_s16 ); // 20bit * 64
        out_c0_r1 = vmlaq_s32( out_c0_r1, iC1_s32, iC0_s32 ); // 26bit * 64
        out_c0_r2 = vmlal_s16( out_c0_r2, iC2_s16, iC0_s16 ); // 20bit * 64
        out_c0_r3 = vmlaq_s32( out_c0_r3, iC3_s32, iC0_s32 ); // 26bit * 64

        // Skip compute due to symmetry.
        // out_c1_r0 = out_c0_r1;

        out64_c1_r1 = vmlal_s32( out64_c1_r1, vget_low_s32( iC1_s32 ), vget_low_s32( iC1_s32 ) );   // 33bit * 128
        out64_c1_r1 = vmlal_s32( out64_c1_r1, vget_high_s32( iC1_s32 ), vget_high_s32( iC1_s32 ) ); // 33bit * 128
        out_c1_r2 = vmlaq_s32( out_c1_r2, iC2_s32, iC1_s32 );                                       // 26bit * 64
        out64_c1_r3 = vmlal_s32( out64_c1_r3, vget_low_s32( iC3_s32 ), vget_low_s32( iC1_s32 ) );   // 33bit * 128
        out64_c1_r3 = vmlal_s32( out64_c1_r3, vget_high_s32( iC3_s32 ), vget_high_s32( iC1_s32 ) ); // 33bit * 128

        // Skip compute due to symmetry.
        // out_c2_r0 = out_c0_r2;
        // out_c2_r1 = out_c1_r2;

        out_c2_r2 = vmlal_s16( out_c2_r2, iC2_s16, iC2_s16 ); // 20bit * 64
        out_c2_r3 = vmlaq_s32( out_c2_r3, iC3_s32, iC2_s32 ); // 26bit * 64

        // Skip compute due to symmetry.
        // out_c3_r0 = out_c0_r3;
        // out_c3_r1 = out_c1_r3;
        // out_c3_r2 = out_c2_r3;

        out64_c3_r3 = vmlal_s32( out64_c3_r3, vget_low_s32( iC3_s32 ), vget_low_s32( iC3_s32 ) );   // 33bit * 128
        out64_c3_r3 = vmlal_s32( out64_c3_r3, vget_high_s32( iC3_s32 ), vget_high_s32( iC3_s32 ) ); // 33bit * 128

        // Final Row: iC[] x Residue
        int16x4_t res16 = vld1_s16( &pResidue[resIdx] );    // 10bit signed
        int32x4_t res32 = vmovl_s16( res16 );               // 10bit
        out_c0_r4 = vmlal_s16( out_c0_r4, iC0_s16, res16 ); // 20bit * 64
        out_c1_r4 = vmlaq_s32( out_c1_r4, iC1_s32, res32 ); // 26bit * 64
        out_c2_r4 = vmlal_s16( out_c2_r4, iC2_s16, res16 ); // 20bit * 64
        out_c3_r4 = vmlaq_s32( out_c3_r4, iC3_s32, res32 ); // 26bit * 64

        w += 4;
      } while( w != width );

      h++;
      if( h % 4 == 0 )
        cy += 2;
    } while( --innerloop != 0 );

    // Promote int32x4 to int64x2 after every MAX_INT32_LOOP_CNT loops of accumulation.
    int64x2_t out64_c0_r0 = vpaddlq_s32( out_c0_r0 );
    int64x2_t out64_c0_r1 = vpaddlq_s32( out_c0_r1 );
    out64_c0_r01 = vaddq_s64( out64_c0_r01, pairwise_add_s64x2( out64_c0_r0, out64_c0_r1 ) );
    int64x2_t out64_c0_r2 = vpaddlq_s32( out_c0_r2 );
    int64x2_t out64_c0_r3 = vpaddlq_s32( out_c0_r3 );
    out64_c0_r23 = vaddq_s64( out64_c0_r23, pairwise_add_s64x2( out64_c0_r2, out64_c0_r3 ) );

    int64x2_t out64_c1_r2 = vpaddlq_s32( out_c1_r2 );
    out64_c1_r23 = vaddq_s64( out64_c1_r23, pairwise_add_s64x2( out64_c1_r2, out64_c1_r3 ) );

    int64x2_t out64_c2_r2 = vpaddlq_s32( out_c2_r2 );
    int64x2_t out64_c2_r3 = vpaddlq_s32( out_c2_r3 );
    out64_c2_r23 = vaddq_s64( out64_c2_r23, pairwise_add_s64x2( out64_c2_r2, out64_c2_r3 ) );

    out64_c1r1_c3r3 = vaddq_s64( out64_c1r1_c3r3, pairwise_add_s64x2( out64_c1_r1, out64_c3_r3 ) );

    // Promote and store final row accumulate from int32x4 to int64.
    pEqualCoeff[1][4] += horizontal_add_long_s32x4( out_c0_r4 );
    pEqualCoeff[2][4] += horizontal_add_long_s32x4( out_c1_r4 );
    pEqualCoeff[3][4] += horizontal_add_long_s32x4( out_c2_r4 );
    pEqualCoeff[4][4] += horizontal_add_long_s32x4( out_c3_r4 );

  } while( h != height );

  // Store all outputs, copy the symmetric ones.
  // Apply left-shift by 1 or 2 to cancel right-shift on cx and cy.
  const int64_t shift[3] = { 0, 1, 2 };
  const int64x2_t vshift0 = vld1q_s64( shift );
  vst1q_s64( &pEqualCoeff[1][0], vshlq_s64( out64_c0_r01, vshift0 ) );
  vst1q_s64( &pEqualCoeff[1][2], vshlq_s64( out64_c0_r23, vshift0 ) );

  pEqualCoeff[2][0] = pEqualCoeff[1][1];
  out64_c1r1_c3r3 = vshlq_n_s64( out64_c1r1_c3r3, 2 );
  pEqualCoeff[2][1] = vgetq_lane_s64( out64_c1r1_c3r3, 0 );
  const int64x2_t vshift1 = vld1q_s64( shift + 1 );
  vst1q_s64( &pEqualCoeff[2][2], vshlq_s64( out64_c1_r23, vshift1 ) );

  pEqualCoeff[3][0] = pEqualCoeff[1][2];
  pEqualCoeff[3][1] = pEqualCoeff[2][2];
  vst1q_s64( &pEqualCoeff[3][2], vshlq_s64( out64_c2_r23, vshift0 ) );

  pEqualCoeff[4][0] = pEqualCoeff[1][3];
  pEqualCoeff[4][1] = pEqualCoeff[2][3];
  pEqualCoeff[4][2] = pEqualCoeff[3][3];
  pEqualCoeff[4][3] = vgetq_lane_s64( out64_c1r1_c3r3, 1 );

  // Final row: Left-shift by 3.
  // Apply extra left-shift by 1 to cancel right-shift on cx and cy.
  pEqualCoeff[1][4] <<= 3;
  pEqualCoeff[2][4] <<= 4;
  pEqualCoeff[3][4] <<= 3;
  pEqualCoeff[4][4] <<= 4;
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

  // Set max inner loop count for accumulating into int32x4.
  static constexpr int MAX_INT32_LOOP_CNT = 128;
  const int innerloop_max = std::min( MAX_INT32_LOOP_CNT, height * ( width >> 2 ) );

  int64x2_t out64_c0_r01 = vdupq_n_s64( 0 );
  int64x2_t out64_c0_r23 = vdupq_n_s64( 0 );
  int64x2_t out64_c0_r45 = vdupq_n_s64( 0 );
  int64x2_t out64_c1_r23 = vdupq_n_s64( 0 );
  int64x2_t out64_c1_r45 = vdupq_n_s64( 0 );
  int64x2_t out64_c2_r23 = vdupq_n_s64( 0 );
  int64x2_t out64_c2_r45 = vdupq_n_s64( 0 );
  int64x2_t out64_c3_r45 = vdupq_n_s64( 0 );
  int64x2_t out64_c4_r45 = vdupq_n_s64( 0 );
  int64x2_t out64_c5_r45 = vdupq_n_s64( 0 );
  int64x2_t out64_c1r1_c3r3 = vdupq_n_s64( 0 );

  int h = 0;

  // Right-shift by 1 to fit more bits into the int32x4 accumulator.
  int cy = ( h + 2 ) >> 1; // Max: (124+2)>>1 = 63

  const int innerloop_height = innerloop_max / ( width >> 2 );
  do
  {
    int32x4_t out_c0_r0 = vdupq_n_s32( 0 );
    int32x4_t out_c0_r1 = vdupq_n_s32( 0 );
    int32x4_t out_c0_r2 = vdupq_n_s32( 0 );
    int32x4_t out_c0_r3 = vdupq_n_s32( 0 );
    int32x4_t out_c0_r4 = vdupq_n_s32( 0 );
    int32x4_t out_c0_r5 = vdupq_n_s32( 0 );
    int32x4_t out_c0_r6 = vdupq_n_s32( 0 );

    int64x2_t out64_c1_r1 = vdupq_n_s64( 0 );
    int32x4_t out_c1_r2 = vdupq_n_s32( 0 );
    int64x2_t out64_c1_r3 = vdupq_n_s64( 0 );
    int64x2_t out64_c1_r4 = vdupq_n_s64( 0 );
    int64x2_t out64_c1_r5 = vdupq_n_s64( 0 );
    int32x4_t out_c1_r6 = vdupq_n_s32( 0 );

    int32x4_t out_c2_r2 = vdupq_n_s32( 0 );
    int32x4_t out_c2_r3 = vdupq_n_s32( 0 );
    int32x4_t out_c2_r4 = vdupq_n_s32( 0 );
    int32x4_t out_c2_r5 = vdupq_n_s32( 0 );
    int32x4_t out_c2_r6 = vdupq_n_s32( 0 );

    int64x2_t out64_c3_r3 = vdupq_n_s64( 0 );
    int64x2_t out64_c3_r4 = vdupq_n_s64( 0 );
    int64x2_t out64_c3_r5 = vdupq_n_s64( 0 );
    int32x4_t out_c3_r6 = vdupq_n_s32( 0 );

    int64x2_t out64_c4_r4 = vdupq_n_s64( 0 );
    int64x2_t out64_c4_r5 = vdupq_n_s64( 0 );
    int32x4_t out_c4_r6 = vdupq_n_s32( 0 );

    int64x2_t out64_c5_r5 = vdupq_n_s64( 0 );
    int32x4_t out_c5_r6 = vdupq_n_s32( 0 );

    int innerloop = innerloop_height;
    do
    {
      int w = 0;
      do
      {
        const int drvIdx = h * derivateBufStride + w;
        const int resIdx = h * residueStride + w;

        // Right-shift by 1 to fit more bits into the int32x4 accumulator.
        const int cx = ( w + 2 ) >> 1; // Max: (124+2)>>1 = 63

        // The max bits are written for each multiply and accumulate to ensure they fit within int16/32/64.
        int16x4_t iC0 = vld1_s16( &ppDerivate[0][drvIdx] ); // 10bit signed
        int16x4_t iC2 = vld1_s16( &ppDerivate[1][drvIdx] ); // 10bit signed
        int16x4_t iC1 = vmul_n_s16( iC0, cx );              // 16bit
        int16x4_t iC3 = vmul_n_s16( iC2, cx );              // 16bit
        int16x4_t iC4 = vmul_n_s16( iC0, cy );              // 16bit
        int16x4_t iC5 = vmul_n_s16( iC2, cy );              // 16bit

        out_c0_r0 = vmlal_s16( out_c0_r0, iC0, iC0 ); // 20bit * 128
        out_c0_r1 = vmlal_s16( out_c0_r1, iC1, iC0 ); // 25bit * 128
        out_c0_r2 = vmlal_s16( out_c0_r2, iC2, iC0 ); // 20bit * 128
        out_c0_r3 = vmlal_s16( out_c0_r3, iC3, iC0 ); // 25bit * 128
        out_c0_r4 = vmlal_s16( out_c0_r4, iC4, iC0 ); // 25bit * 128
        out_c0_r5 = vmlal_s16( out_c0_r5, iC5, iC0 ); // 25bit * 128

        // Skip compute due to symmetry.
        // out_c1_r0 = out_c0_r1;
        int32x4_t out_c1_r1 = vmull_s16( iC1, iC1 );                        // 31bit
        out64_c1_r1 = vaddw_s32( out64_c1_r1, vget_low_s32( out_c1_r1 ) );  // 31bit * 256
        out64_c1_r1 = vaddw_s32( out64_c1_r1, vget_high_s32( out_c1_r1 ) ); // 31bit * 256
        out_c1_r2 = vmlal_s16( out_c1_r2, iC2, iC1 );                       // 25bit * 128
        int32x4_t out_c1_r3 = vmull_s16( iC3, iC1 );                        // 31bit
        out64_c1_r3 = vaddw_s32( out64_c1_r3, vget_low_s32( out_c1_r3 ) );  // 31bit * 256
        out64_c1_r3 = vaddw_s32( out64_c1_r3, vget_high_s32( out_c1_r3 ) ); // 31bit * 256
        int32x4_t out_c1_r4 = vmull_s16( iC4, iC1 );                        // 31bit
        out64_c1_r4 = vaddw_s32( out64_c1_r4, vget_low_s32( out_c1_r4 ) );  // 31bit * 256
        out64_c1_r4 = vaddw_s32( out64_c1_r4, vget_high_s32( out_c1_r4 ) ); // 31bit * 256
        int32x4_t out_c1_r5 = vmull_s16( iC5, iC1 );                        // 31bit
        out64_c1_r5 = vaddw_s32( out64_c1_r5, vget_low_s32( out_c1_r5 ) );  // 31bit * 256
        out64_c1_r5 = vaddw_s32( out64_c1_r5, vget_high_s32( out_c1_r5 ) ); // 31bit * 256

        // Skip compute due to symmetry.
        // out_c2_r0 = out_c0_r2;
        // out_c2_r1 = out_c1_r2;
        out_c2_r2 = vmlal_s16( out_c2_r2, iC2, iC2 ); // 20bit * 128
        out_c2_r3 = vmlal_s16( out_c2_r3, iC3, iC2 ); // 25bit * 128
        out_c2_r4 = vmlal_s16( out_c2_r4, iC4, iC2 ); // 25bit * 128
        out_c2_r5 = vmlal_s16( out_c2_r5, iC5, iC2 ); // 25bit * 128

        // Skip compute due to symmetry.
        // out_c3_r0 = out_c0_r3;
        // out_c3_r1 = out_c1_r3;
        // out_c3_r2 = out_c2_r3;
        int32x4_t out_c3_r3 = vmull_s16( iC3, iC3 );                        // 31bit
        out64_c3_r3 = vaddw_s32( out64_c3_r3, vget_low_s32( out_c3_r3 ) );  // 31bit * 256
        out64_c3_r3 = vaddw_s32( out64_c3_r3, vget_high_s32( out_c3_r3 ) ); // 31bit * 256
        int32x4_t out_c3_r4 = vmull_s16( iC4, iC3 );                        // 31bit
        out64_c3_r4 = vaddw_s32( out64_c3_r4, vget_low_s32( out_c3_r4 ) );  // 31bit * 256
        out64_c3_r4 = vaddw_s32( out64_c3_r4, vget_high_s32( out_c3_r4 ) ); // 31bit * 256
        int32x4_t out_c3_r5 = vmull_s16( iC5, iC3 );                        // 31bit
        out64_c3_r5 = vaddw_s32( out64_c3_r5, vget_low_s32( out_c3_r5 ) );  // 31bit * 256
        out64_c3_r5 = vaddw_s32( out64_c3_r5, vget_high_s32( out_c3_r5 ) ); // 31bit * 256

        // Skip compute due to symmetry.
        // out_c4_r0 = out_c0_r4;
        // out_c4_r1 = out_c1_r4;
        // out_c4_r2 = out_c2_r4;
        // out_c4_r3 = out_c3_r4;
        int32x4_t out_c4_r4 = vmull_s16( iC4, iC4 );                        // 31bit
        out64_c4_r4 = vaddw_s32( out64_c4_r4, vget_low_s32( out_c4_r4 ) );  // 31bit * 256
        out64_c4_r4 = vaddw_s32( out64_c4_r4, vget_high_s32( out_c4_r4 ) ); // 31bit * 256
        int32x4_t out_c4_r5 = vmull_s16( iC5, iC4 );                        // 31bit
        out64_c4_r5 = vaddw_s32( out64_c4_r5, vget_low_s32( out_c4_r5 ) );  // 31bit * 256
        out64_c4_r5 = vaddw_s32( out64_c4_r5, vget_high_s32( out_c4_r5 ) ); // 31bit * 256

        // Skip compute due to symmetry.
        // out_c5_r0 = out_c0_r5;
        // out_c5_r1 = out_c1_r5;
        // out_c5_r2 = out_c2_r5;
        // out_c5_r3 = out_c3_r5;
        // out_c5_r4 = out_c4_r5;
        int32x4_t out_c5_r5 = vmull_s16( iC5, iC5 );                        // 31bit
        out64_c5_r5 = vaddw_s32( out64_c5_r5, vget_low_s32( out_c5_r5 ) );  // 31bit * 256
        out64_c5_r5 = vaddw_s32( out64_c5_r5, vget_high_s32( out_c5_r5 ) ); // 31bit * 256

        // Final Row: iC[] x Residue
        int16x4_t res = vld1_s16( &pResidue[resIdx] ); // 10bit signed
        out_c0_r6 = vmlal_s16( out_c0_r6, iC0, res );  // 20bit * 128
        out_c1_r6 = vmlal_s16( out_c1_r6, iC1, res );  // 25bit * 128
        out_c2_r6 = vmlal_s16( out_c2_r6, iC2, res );  // 20bit * 128
        out_c3_r6 = vmlal_s16( out_c3_r6, iC3, res );  // 25bit * 128
        out_c4_r6 = vmlal_s16( out_c4_r6, iC4, res );  // 25bit * 128
        out_c5_r6 = vmlal_s16( out_c5_r6, iC5, res );  // 25bit * 128

        w += 4;
      } while( w != width );

      h++;
      if( h % 4 == 0 )
        cy += 2;
    } while( --innerloop != 0 );

    // Promote int32x4 to int64x2 after every MAX_INT32_LOOP_CNT loops of accumulation.
    int64x2_t out64_c0_r0 = vpaddlq_s32( out_c0_r0 );
    int64x2_t out64_c0_r1 = vpaddlq_s32( out_c0_r1 );
    out64_c0_r01 = vaddq_s64( out64_c0_r01, pairwise_add_s64x2( out64_c0_r0, out64_c0_r1 ) );
    int64x2_t out64_c0_r2 = vpaddlq_s32( out_c0_r2 );
    int64x2_t out64_c0_r3 = vpaddlq_s32( out_c0_r3 );
    out64_c0_r23 = vaddq_s64( out64_c0_r23, pairwise_add_s64x2( out64_c0_r2, out64_c0_r3 ) );
    int64x2_t out64_c0_r4 = vpaddlq_s32( out_c0_r4 );
    int64x2_t out64_c0_r5 = vpaddlq_s32( out_c0_r5 );
    out64_c0_r45 = vaddq_s64( out64_c0_r45, pairwise_add_s64x2( out64_c0_r4, out64_c0_r5 ) );

    int64x2_t out64_c1_r2 = vpaddlq_s32( out_c1_r2 );
    out64_c1_r23 = vaddq_s64( out64_c1_r23, pairwise_add_s64x2( out64_c1_r2, out64_c1_r3 ) );
    out64_c1_r45 = vaddq_s64( out64_c1_r45, pairwise_add_s64x2( out64_c1_r4, out64_c1_r5 ) );

    int64x2_t out64_c2_r2 = vpaddlq_s32( out_c2_r2 );
    int64x2_t out64_c2_r3 = vpaddlq_s32( out_c2_r3 );
    out64_c2_r23 = vaddq_s64( out64_c2_r23, pairwise_add_s64x2( out64_c2_r2, out64_c2_r3 ) );
    int64x2_t out64_c2_r4 = vpaddlq_s32( out_c2_r4 );
    int64x2_t out64_c2_r5 = vpaddlq_s32( out_c2_r5 );
    out64_c2_r45 = vaddq_s64( out64_c2_r45, pairwise_add_s64x2( out64_c2_r4, out64_c2_r5 ) );

    out64_c1r1_c3r3 = vaddq_s64( out64_c1r1_c3r3, pairwise_add_s64x2( out64_c1_r1, out64_c3_r3 ) );
    out64_c3_r45 = vaddq_s64( out64_c3_r45, pairwise_add_s64x2( out64_c3_r4, out64_c3_r5 ) );

    out64_c4_r45 = vaddq_s64( out64_c4_r45, pairwise_add_s64x2( out64_c4_r4, out64_c4_r5 ) );

    out64_c5_r45 = vaddq_s64( out64_c5_r45, pairwise_add_s64x2( out64_c4_r5, out64_c5_r5 ) );

    // Promote and store final row accumulate from int32x4 to int64.
    pEqualCoeff[1][6] += horizontal_add_long_s32x4( out_c0_r6 );
    pEqualCoeff[2][6] += horizontal_add_long_s32x4( out_c1_r6 );
    pEqualCoeff[3][6] += horizontal_add_long_s32x4( out_c2_r6 );
    pEqualCoeff[4][6] += horizontal_add_long_s32x4( out_c3_r6 );
    pEqualCoeff[5][6] += horizontal_add_long_s32x4( out_c4_r6 );
    pEqualCoeff[6][6] += horizontal_add_long_s32x4( out_c5_r6 );

  } while( h != height );

  // Store all outputs, copy the symmetric ones.
  // Apply left-shift by 1 or 2 to cancel right-shift on cx and cy.
  const int64_t shift[3] = { 0, 1, 2 };
  const int64x2_t vshift0 = vld1q_s64( shift );
  vst1q_s64( &pEqualCoeff[1][0], vshlq_s64( out64_c0_r01, vshift0 ) );
  vst1q_s64( &pEqualCoeff[1][2], vshlq_s64( out64_c0_r23, vshift0 ) );
  vst1q_s64( &pEqualCoeff[1][4], vshlq_n_s64( out64_c0_r45, 1 ) );

  pEqualCoeff[2][0] = pEqualCoeff[1][1];
  out64_c1r1_c3r3 = vshlq_n_s64( out64_c1r1_c3r3, 2 );
  pEqualCoeff[2][1] = vgetq_lane_s64( out64_c1r1_c3r3, 0 );
  const int64x2_t vshift1 = vld1q_s64( shift + 1 );
  vst1q_s64( &pEqualCoeff[2][2], vshlq_s64( out64_c1_r23, vshift1 ) );
  vst1q_s64( &pEqualCoeff[2][4], vshlq_n_s64( out64_c1_r45, 2 ) );

  pEqualCoeff[3][0] = pEqualCoeff[1][2];
  pEqualCoeff[3][1] = pEqualCoeff[2][2];
  vst1q_s64( &pEqualCoeff[3][2], vshlq_s64( out64_c2_r23, vshift0 ) );
  vst1q_s64( &pEqualCoeff[3][4], vshlq_n_s64( out64_c2_r45, 1 ) );

  pEqualCoeff[4][0] = pEqualCoeff[1][3];
  pEqualCoeff[4][1] = pEqualCoeff[2][3];
  pEqualCoeff[4][2] = pEqualCoeff[3][3];
  pEqualCoeff[4][3] = vgetq_lane_s64( out64_c1r1_c3r3, 1 );
  vst1q_s64( &pEqualCoeff[4][4], vshlq_n_s64( out64_c3_r45, 2 ) );

  pEqualCoeff[5][0] = pEqualCoeff[1][4];
  pEqualCoeff[5][1] = pEqualCoeff[2][4];
  pEqualCoeff[5][2] = pEqualCoeff[3][4];
  pEqualCoeff[5][3] = pEqualCoeff[4][4];
  vst1q_s64( &pEqualCoeff[5][4], vshlq_n_s64( out64_c4_r45, 2 ) );

  pEqualCoeff[6][0] = pEqualCoeff[1][5];
  pEqualCoeff[6][1] = pEqualCoeff[2][5];
  pEqualCoeff[6][2] = pEqualCoeff[3][5];
  pEqualCoeff[6][3] = pEqualCoeff[4][5];
  vst1q_s64( &pEqualCoeff[6][4], vshlq_n_s64( out64_c5_r45, 2 ) );

  // Final row: Left-shift by 3.
  // Apply extra left-shift by 1 to cancel right-shift on cx and cy.
  pEqualCoeff[1][6] <<= 3;
  pEqualCoeff[2][6] <<= 4;
  pEqualCoeff[3][6] <<= 3;
  pEqualCoeff[4][6] <<= 4;
  pEqualCoeff[5][6] <<= 4;
  pEqualCoeff[6][6] <<= 4;
};

template<>
void AffineGradientSearch::_initAffineGradientSearchARM<NEON>()
{
  m_EqualCoeffComputer[0] = simdEqualCoeffComputer_neon<false>;
  m_EqualCoeffComputer[1] = simdEqualCoeffComputer_neon<true>;
}

} // namespace vvenc

#endif // ENABLE_SIMD_OPT_AFFINE_ME && defined(TARGET_SIMD_ARM)

//! \}

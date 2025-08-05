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
 * \file MCTF_neon.cpp
 * \brief Neon implementation of MCTF for AArch64.
 */
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "MCTF.h"
#include "sum_neon.h"

#include <arm_neon.h>
#include <math.h>

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCTF

#include "MCTF_neon.h"

namespace vvenc
{

static int16x8_t motionErrorLumaFrac_loRes_step( const int16x8_t xf, const Pel* rowStart, const Pel maxSampleValue )
{
  int16x8_t row04 = vld1q_s16( rowStart + 0 );
  int16x8_t row15 = vld1q_s16( rowStart + 1 );
  int16x8_t row26 = vld1q_s16( rowStart + 2 );
  int16x8_t row37 = vld1q_s16( rowStart + 3 );

  int32x4_t sum0 = vmull_s16( vget_low_s16( xf ), vget_low_s16( row04 ) );
  int32x4_t sum4 = vmull_s16( vget_high_s16( xf ), vget_high_s16( row04 ) );
  int32x4_t sum1 = vmull_s16( vget_low_s16( xf ), vget_low_s16( row15 ) );
  int32x4_t sum5 = vmull_s16( vget_high_s16( xf ), vget_high_s16( row15 ) );
  int32x4_t sum2 = vmull_s16( vget_low_s16( xf ), vget_low_s16( row26 ) );
  int32x4_t sum6 = vmull_s16( vget_high_s16( xf ), vget_high_s16( row26 ) );
  int32x4_t sum3 = vmull_s16( vget_low_s16( xf ), vget_low_s16( row37 ) );
  int32x4_t sum7 = vmull_s16( vget_high_s16( xf ), vget_high_s16( row37 ) );

  int32x4_t sum0123 = horizontal_add_4d_s32x4( sum0, sum1, sum2, sum3 );
  int32x4_t sum4567 = horizontal_add_4d_s32x4( sum4, sum5, sum6, sum7 );
  uint16x8_t sum = vcombine_u16( vqrshrun_n_s32( sum0123, 6 ), vqrshrun_n_s32( sum4567, 6 ) );

  return vminq_s16( vreinterpretq_s16_u16( sum ), vdupq_n_s16( maxSampleValue ) );
}

int motionErrorLumaFrac_loRes_neon( const Pel* org, const ptrdiff_t origStride, const Pel* buf,
                                    const ptrdiff_t buffStride, const int w, const int h, const int16_t* xFilter,
                                    const int16_t* yFilter, const int bitDepth, const int besterror )
{
  const Pel maxSampleValue = ( 1 << bitDepth ) - 1;

  CHECK( w & 7, "SIMD blockSize needs to be a multiple of 8" );

  const int16x8_t xf = vreinterpretq_s16_u64( vld1q_dup_u64( ( const uint64_t* )xFilter ) );
  const int16x4_t yf = vld1_s16( yFilter );

  int error = 0;
  int x = 0;
  do
  {
    const Pel* rowStart0 = buf + -1 * buffStride + x - 1;
    int16x8_t xsum0 = motionErrorLumaFrac_loRes_step( xf, rowStart0, maxSampleValue );

    const Pel* rowStart1 = buf + 0 * buffStride + x - 1;
    int16x8_t xsum1 = motionErrorLumaFrac_loRes_step( xf, rowStart1, maxSampleValue );

    const Pel* rowStart2 = buf + 1 * buffStride + x - 1;
    int16x8_t xsum2 = motionErrorLumaFrac_loRes_step( xf, rowStart2, maxSampleValue );

    int y = 0;
    do
    {
      const Pel* rowStart = buf + ( y + 2 ) * buffStride + x - 1;
      int16x8_t xsum3 = motionErrorLumaFrac_loRes_step( xf, rowStart, maxSampleValue );

      const Pel* origRow = org + y * origStride;

      int32x4_t ysumLo = vmull_lane_s16( vget_low_s16( xsum0 ), yf, 0 );
      ysumLo = vmlal_lane_s16( ysumLo, vget_low_s16( xsum1 ), yf, 1 );
      ysumLo = vmlal_lane_s16( ysumLo, vget_low_s16( xsum2 ), yf, 2 );
      ysumLo = vmlal_lane_s16( ysumLo, vget_low_s16( xsum3 ), yf, 3 );

      int32x4_t ysumHi = vmull_lane_s16( vget_high_s16( xsum0 ), yf, 0 );
      ysumHi = vmlal_lane_s16( ysumHi, vget_high_s16( xsum1 ), yf, 1 );
      ysumHi = vmlal_lane_s16( ysumHi, vget_high_s16( xsum2 ), yf, 2 );
      ysumHi = vmlal_lane_s16( ysumHi, vget_high_s16( xsum3 ), yf, 3 );

      uint16x8_t ysum = vcombine_u16( vqrshrun_n_s32( ysumLo, 6 ), vqrshrun_n_s32( ysumHi, 6 ) );

      int16x8_t ysum16 = vreinterpretq_s16_u16( vminq_u16( ysum, vdupq_n_u16( maxSampleValue ) ) );
      int16x8_t orig = vld1q_s16( origRow + x );
      int16x8_t diff = vabdq_s16( ysum16, orig );

      int32x4_t diff2 = vmull_s16( vget_low_s16( diff ), vget_low_s16( diff ) );
      diff2 = vmlal_s16( diff2, vget_high_s16( diff ), vget_high_s16( diff ) );

      error += horizontal_add_s32x4( diff2 );
      if( error > besterror )
      {
        return error;
      }

      xsum0 = xsum1;
      xsum1 = xsum2;
      xsum2 = xsum3;
    } while( ++y != h );
    x += 8;
  } while( x != w );

  return error;
}

static int motionErrorLumaInt_neon( const Pel* org, const ptrdiff_t origStride, const Pel* buf,
                                    const ptrdiff_t buffStride, const int w, int h, const int besterror )
{
  CHECK( w % 8 != 0, "Width must be a multiple of eight" );

  int error = 0;
  do
  {
    int32x4_t acc_lo = vdupq_n_s32( 0 );
    int32x4_t acc_hi = vdupq_n_s32( 0 );

    int x1 = 0;
    do
    {
      int16x8_t o = vld1q_s16( org + x1 );
      int16x8_t b = vld1q_s16( buf + x1 );

      int16x8_t diff = vabdq_s16( o, b );
      acc_lo         = vmlal_s16( acc_lo, vget_low_s16( diff ), vget_low_s16( diff ) );
      acc_hi         = vmlal_s16( acc_hi, vget_high_s16( diff ), vget_high_s16( diff ) );

      x1 += 8;
    } while( x1 != w );

    int32x4_t diff2_sum = vaddq_s32( acc_lo, acc_hi );
    error += horizontal_add_s32x4( diff2_sum );
    if( error > besterror )
    {
      return error;
    }

    org += origStride;
    buf += buffStride;
  } while( --h != 0 );

  return error;
}

void applyPlanarCorrection_neon( const Pel* refPel, const ptrdiff_t refStride, Pel* dstPel, const ptrdiff_t dstStride,
                                 const int32_t w, const int32_t h, const ClpRng& clpRng, const uint16_t motionError )
{
  int32_t x1yzm = 0, x2yzm = 0, ySum = 0;

  CHECK( w != h, "Width must be same as height!" );
  CHECK( w < 4, "Width must be greater than or equal to 4!" );
  CHECK( ( w & ( w - 1 ) ) != 0, "Width can only be power of two!" );

  if( w % 8 == 0 )
  {
    Pel* pDst = dstPel;
    const Pel* pRef = refPel;

    const int16_t idx_off[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    int16x8_t vidx_off = vld1q_s16( idx_off );
    int32x4_t acc_x_lo = vdupq_n_s32( 0 );
    int32x4_t acc_x_hi = vdupq_n_s32( 0 );

    for( int32_t y = 0; y < h; y += 2 ) // Sum up dot-products between indices and sample diffs.
    {
      int32_t x = 0;
      int16x8_t acc_z0 = vdupq_n_s16( 0 ); // Σ(z) for one iteration of y.
      int16x8_t acc_z1 = vdupq_n_s16( 0 );
      int16x8_t xv = vidx_off;

      do
      {
        int16x8_t dst0 = vld1q_s16( pDst + x );
        int16x8_t ref0 = vld1q_s16( pRef + x );
        int16x8_t dst1 = vld1q_s16( pDst + dstStride + x );
        int16x8_t ref1 = vld1q_s16( pRef + refStride + x );

        int16x8_t diff0 = vsubq_s16( dst0, ref0 );
        int16x8_t diff1 = vsubq_s16( dst1, ref1 );

        acc_z0 = vaddq_s16( acc_z0, diff0 );
        acc_z1 = vaddq_s16( acc_z1, diff1 );

        acc_x_lo = vmlal_s16( acc_x_lo, vget_low_s16( diff0 ), vget_low_s16( xv ) );
        acc_x_hi = vmlal_s16( acc_x_hi, vget_high_s16( diff0 ), vget_high_s16( xv ) );
        acc_x_lo = vmlal_s16( acc_x_lo, vget_low_s16( diff1 ), vget_low_s16( xv ) );
        acc_x_hi = vmlal_s16( acc_x_hi, vget_high_s16( diff1 ), vget_high_s16( xv ) );

        xv = vaddq_s16( xv, vdupq_n_s16( 8 ) );
        x += 8;
      } while( x != w );

      int32_t zAcc0 = horizontal_add_long_s16x8( acc_z0 );
      int32_t zAcc1 = horizontal_add_long_s16x8( acc_z1 );

      x2yzm += y * zAcc0 + ( y + 1 ) * zAcc1; // Σ(y*z) calculated as y * Σ(z).
      ySum += zAcc0 + zAcc1;                  // Σ(z)

      pDst += dstStride << 1;
      pRef += refStride << 1;
    }
    acc_x_hi = vaddq_s32( acc_x_hi, acc_x_lo );
    x1yzm = horizontal_add_s32x4( acc_x_hi ); // Σ(x*z)
  }
  else if( w == 4 )
  {
    Pel* pDst = dstPel;
    const Pel* pRef = refPel;

    // Sum up dot-products between indices and sample diffs.
    int16x8_t dst = vcombine_s16( vld1_s16( pDst ), vld1_s16( pDst + dstStride ) ); // 10 bit
    int16x8_t ref = vcombine_s16( vld1_s16( pRef ), vld1_s16( pRef + refStride ) ); // 10 bit
    int16x8_t diff0 = vsubq_s16( dst, ref );
    int16x8_t acc_y = vextq_s16( diff0, vdupq_n_s16( 0 ), 4 );

    dst = vcombine_s16( vld1_s16( pDst + 2 * dstStride ), vld1_s16( pDst + 3 * dstStride ) );
    ref = vcombine_s16( vld1_s16( pRef + 2 * refStride ), vld1_s16( pRef + 3 * refStride ) );
    int16x8_t diff1 = vsubq_s16( dst, ref );
    int16x8_t acc_z = vaddq_s16( diff0, diff1 );

    const int16_t idx_off1[] = { 2, 2, 2, 2, 3, 3, 3, 3 };
    int16x8_t v_idx_off1 = vld1q_s16( idx_off1 );
    const int16_t idx_off2[] = { 0, 1, 2, 3, 0, 1, 2, 3 };
    int16x8_t v_idx_off2 = vld1q_s16( idx_off2 );
    acc_y = vmlaq_s16( acc_y, diff1, v_idx_off1 );
    int16x8_t acc_x = vmulq_s16( acc_z, v_idx_off2 );

    ySum = horizontal_add_long_s16x8( acc_z );  // Σ(z)
    x2yzm = horizontal_add_long_s16x8( acc_y ); // Σ(y*z)
    x1yzm = horizontal_add_long_s16x8( acc_x ); // Σ(x*z)
  }

  applyPlanarDeblockingCorrection_common( dstPel, dstStride, x1yzm, x2yzm, ySum, w, h, clpRng, motionError );
}

void applyBlock_neon( const CPelBuf& src, PelBuf& dst, const CompArea& blk, const ClpRng& clpRng,
                      const Pel** correctedPics, int numRefs, const int* verror, const double* refStrenghts,
                      double weightScaling, double sigmaSq )
{
  const int w = blk.width;
  const int h = blk.height;
  const int bx = blk.x;
  const int by = blk.y;

  CHECK( w < 4, "Width must be greater than or equal to 4!" );
  CHECK( h % 2 != 0, "Height must be multiple of 2!" );

  const ptrdiff_t srcStride = src.stride;
  const Pel* srcPel = src.bufAt( bx, by );

  int vnoise[2 * VVENC_MCTF_RANGE] = { 0 };

  int minError = INT32_MAX;

  for( int i = 0; i < numRefs; i++ )
  {
    int64_t variance = 0, diffsum = 0;
    const ptrdiff_t refStride = w;
    const Pel* refBuf = correctedPics[i];
    const Pel* srcBuf = srcPel;

    if( w % 8 == 0 )
    {
      int32x4_t variance_acc = vdupq_n_s32( 0 );
      int32x4_t diffR_acc = vdupq_n_s32( 0 );
      int32x4_t diffD_acc = vdupq_n_s32( 0 );
      int16_t ind[] = { 1, 1, 1, 1, 1, 1, 1, 0 };
      int16x8_t clear_lastlane = vld1q_s16( ind );

      int y1 = h >> 1;
      do
      {
        // One iteration for x1 done outside loop.
        int16x8_t pix0 = vld1q_s16( srcBuf );             // unsigned 10bit
        int16x8_t ref0 = vld1q_s16( refBuf );             // unsigned 10bit
        int16x8_t pix1 = vld1q_s16( srcBuf + srcStride ); // unsigned 10bit
        int16x8_t ref1 = vld1q_s16( refBuf + refStride ); // unsigned 10bit

        int16x8_t diff0 = vsubq_s16( pix0, ref0 ); // 11bit
        int16x8_t diff1 = vsubq_s16( pix1, ref1 ); // 11bit

        for( int x1 = 0; x1 < w - 8; x1 += 8 )
        {
          int16x8_t pixNext0 = vld1q_s16( srcBuf + 8 );             // unsigned 10bit
          int16x8_t refNext0 = vld1q_s16( refBuf + 8 );             // unsigned 10bit
          int16x8_t pixNext1 = vld1q_s16( srcBuf + srcStride + 8 ); // unsigned 10bit
          int16x8_t refNext1 = vld1q_s16( refBuf + refStride + 8 ); // unsigned 10bit

          int16x8_t diffNext0 = vsubq_s16( pixNext0, refNext0 );
          int16x8_t diffNext1 = vsubq_s16( pixNext1, refNext1 );
          int16x8_t diffR0 = vextq_s16( diff0, diffNext0, 1 );
          int16x8_t diffR1 = vextq_s16( diff1, diffNext1, 1 );
          int16x8_t diffD0 = vsubq_s16( diff1, diff0 ); // 11bit

          diffR0 = vsubq_s16( diffR0, diff0 ); // 11bit
          diffR1 = vsubq_s16( diffR1, diff1 ); // 11bit

          variance_acc = vmlal_s16( variance_acc, vget_low_s16( diff0 ), vget_low_s16( diff0 ) ); // 29bit
          variance_acc = vmlal_s16( variance_acc, vget_high_s16( diff0 ), vget_high_s16( diff0 ) );
          diffR_acc = vmlal_s16( diffR_acc, vget_low_s16( diffR0 ), vget_low_s16( diffR0 ) ); // 31bit
          diffR_acc = vmlal_s16( diffR_acc, vget_high_s16( diffR0 ), vget_high_s16( diffR0 ) );
          diffD_acc = vmlal_s16( diffD_acc, vget_low_s16( diffD0 ), vget_low_s16( diffD0 ) ); // 31bit
          diffD_acc = vmlal_s16( diffD_acc, vget_high_s16( diffD0 ), vget_high_s16( diffD0 ) );

          variance_acc = vmlal_s16( variance_acc, vget_low_s16( diff1 ), vget_low_s16( diff1 ) ); // 29bit
          variance_acc = vmlal_s16( variance_acc, vget_high_s16( diff1 ), vget_high_s16( diff1 ) );
          diffR_acc = vmlal_s16( diffR_acc, vget_low_s16( diffR1 ), vget_low_s16( diffR1 ) ); // 31bit
          diffR_acc = vmlal_s16( diffR_acc, vget_high_s16( diffR1 ), vget_high_s16( diffR1 ) );

          if( y1 != 1 )
          {
            int16x8_t pixD1 = vld1q_s16( srcBuf + ( srcStride << 1 ) ); // unsigned 10bit
            int16x8_t refD1 = vld1q_s16( refBuf + ( refStride << 1 ) ); // unsigned 10bit

            int16x8_t diffD1 = vsubq_s16( pixD1, refD1 );                                       // 11bit
            diffD1 = vsubq_s16( diffD1, diff1 );                                                // 11bit
            diffD_acc = vmlal_s16( diffD_acc, vget_low_s16( diffD1 ), vget_low_s16( diffD1 ) ); // 31bit
            diffD_acc = vmlal_s16( diffD_acc, vget_high_s16( diffD1 ), vget_high_s16( diffD1 ) );
          }

          diff0 = diffNext0;
          diff1 = diffNext1;
          srcBuf += 8;
          refBuf += 8;
        }

        // Last iteration of x1.
        int16x8_t diffR0 = vextq_s16( diff0, vdupq_n_s16( 0 ), 1 );
        int16x8_t diffR1 = vextq_s16( diff1, vdupq_n_s16( 0 ), 1 );
        int16x8_t diffD0 = vsubq_s16( diff1, diff0 ); // 11bit

        diffR0 = vmlsq_s16( diffR0, diff0, clear_lastlane ); // 11 bit
        diffR1 = vmlsq_s16( diffR1, diff1, clear_lastlane );

        variance_acc = vmlal_s16( variance_acc, vget_low_s16( diff0 ), vget_low_s16( diff0 ) ); // 29bit
        variance_acc = vmlal_s16( variance_acc, vget_high_s16( diff0 ), vget_high_s16( diff0 ) );
        diffR_acc = vmlal_s16( diffR_acc, vget_low_s16( diffR0 ), vget_low_s16( diffR0 ) ); // 31bit
        diffR_acc = vmlal_s16( diffR_acc, vget_high_s16( diffR0 ), vget_high_s16( diffR0 ) );
        diffD_acc = vmlal_s16( diffD_acc, vget_low_s16( diffD0 ), vget_low_s16( diffD0 ) ); // 31bit
        diffD_acc = vmlal_s16( diffD_acc, vget_high_s16( diffD0 ), vget_high_s16( diffD0 ) );

        variance_acc = vmlal_s16( variance_acc, vget_low_s16( diff1 ), vget_low_s16( diff1 ) ); // 29bit
        variance_acc = vmlal_s16( variance_acc, vget_high_s16( diff1 ), vget_high_s16( diff1 ) );
        diffR_acc = vmlal_s16( diffR_acc, vget_low_s16( diffR1 ), vget_low_s16( diffR1 ) ); // 31bit
        diffR_acc = vmlal_s16( diffR_acc, vget_high_s16( diffR1 ), vget_high_s16( diffR1 ) );

        if( y1 != 1 )
        {
          int16x8_t pixD1 = vld1q_s16( srcBuf + ( srcStride << 1 ) ); // 10bit unsigned
          int16x8_t refD1 = vld1q_s16( refBuf + ( refStride << 1 ) ); // 10bit unsigned

          int16x8_t diffD1 = vsubq_s16( pixD1, refD1 );                                       // 11bit
          diffD1 = vsubq_s16( diffD1, diff1 );                                                // 11bit
          diffD_acc = vmlal_s16( diffD_acc, vget_low_s16( diffD1 ), vget_low_s16( diffD1 ) ); // 31bit
          diffD_acc = vmlal_s16( diffD_acc, vget_high_s16( diffD1 ), vget_high_s16( diffD1 ) );
        }

        refBuf += ( refStride << 1 ) - ( w - 8 );
        srcBuf += ( srcStride << 1 ) - ( w - 8 );
      } while( --y1 != 0 );

      variance = horizontal_add_long_s32x4( variance_acc );
      uint32x4_t vDiffSum =
          vaddq_u32( vreinterpretq_u32_s32( diffR_acc ), vreinterpretq_u32_s32( diffD_acc ) ); // 32 bit
      diffsum = ( int64_t )horizontal_add_long_u32x4( vDiffSum );
    }
    else
    {
      CHECK( w != 4, "Width must be equal to 4!" );

      // Last iteration for y1 done outside loop.
      int32x4_t variance_acc = vdupq_n_s32( 0 );
      int32x4_t diffR_acc = vdupq_n_s32( 0 );
      int32x4_t diffD_acc = vdupq_n_s32( 0 );

      int16x4_t pix = vld1_s16( srcBuf );
      int16x4_t ref = vld1_s16( refBuf );
      int16x4_t diff = vsub_s16( pix, ref );

      int16_t ind[] = { 1, 1, 1, 0 };
      int16x4_t clear_lastlane = vld1_s16( ind );

      int y1 = h - 1;
      do
      {
        int16x4_t pixD = vld1_s16( srcBuf + srcStride );
        int16x4_t refD = vld1_s16( refBuf + refStride );

        int16x4_t diffNext = vsub_s16( pixD, refD );
        int16x4_t diffR = vext_s16( diff, vdup_n_s16( 0 ), 1 );
        diffR = vmls_s16( diffR, diff, clear_lastlane );

        variance_acc = vmlal_s16( variance_acc, diff, diff );
        diffR_acc = vmlal_s16( diffR_acc, diffR, diffR );

        int16x4_t diffD = vsub_s16( diffNext, diff );
        diffD_acc = vmlal_s16( diffD_acc, diffD, diffD );

        diff = diffNext;
        refBuf += refStride;
        srcBuf += srcStride;
      } while( --y1 != 0 );

      int16x4_t diffR = vext_s16( diff, vdup_n_s16( 0 ), 1 );
      diffR = vmls_s16( diffR, diff, clear_lastlane );

      variance_acc = vmlal_s16( variance_acc, diff, diff );
      diffR_acc = vmlal_s16( diffR_acc, diffR, diffR );

      variance = horizontal_add_long_s32x4( variance_acc );
      diffsum = horizontal_add_long_s32x4( vaddq_s32( diffR_acc, diffD_acc ) );
    }
    variance <<= 2 * ( 10 - clpRng.bd );
    diffsum <<= 2 * ( 10 - clpRng.bd );
    const int cntV = w * h;
    const int cntD = 2 * cntV - w - h;
    vnoise[i] = ( int )round( ( 15.0 * cntD / cntV * variance + 5.0 ) / ( diffsum + 5.0 ) );
    minError = std::min( minError, verror[i] );
  }

  applyBlock_common( src, dst, blk, clpRng, correctedPics, numRefs, verror, vnoise, refStrenghts, minError,
                     weightScaling, sigmaSq );
}

template<>
void MCTF::_initMCTF_ARM<NEON>()
{
  m_motionErrorLumaFrac8[1] = motionErrorLumaFrac_loRes_neon;
  m_motionErrorLumaInt8     = motionErrorLumaInt_neon;
  m_applyPlanarCorrection   = applyPlanarCorrection_neon;
  m_applyBlock              = applyBlock_neon;
}

} // namespace vvenc
#endif
//! \}

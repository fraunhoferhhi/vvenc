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
#include "mem_neon.h"

namespace vvenc
{

template<FilterCoeffType4 xType, FilterCoeffType4 yType>
static inline int motionErrorLumaFrac_loRes2D_neon( const Pel* org, const ptrdiff_t origStride, const Pel* buf,
                                                    const ptrdiff_t buffStride, int w, int h, const int16_t* xFilter,
                                                    const int16_t* yFilter, const int bitDepth, const int besterror )
{
  const Pel maxSampleValue = ( 1 << bitDepth ) - 1;

  CHECKD( w % 8 != 0, "Width must be multiple of 8!" );
  CHECKD( h % 4 != 0, "Height must be multiple of 4!" );

  const int16x4_t xf = vrshr_n_s16( vld1_s16( xFilter ), 1 );
  const int16x4_t yf = vrshr_n_s16( vld1_s16( yFilter ), 1 );

  constexpr int numFilterTaps = 4;
  int16x8_t h_src[numFilterTaps];
  int16x8_t v_src[numFilterTaps + 3]; // 3 extra elements are needed because the height loop is unrolled 4 times.

  int error = 0;

  do
  {
    load_s16_16x8x4( buf - 1 * buffStride - 1, 1, h_src );
    v_src[0] = motionErrorLumaFrac_loRes1D_neon<xType>( h_src, xf, maxSampleValue );

    load_s16_16x8x4( buf + 0 * buffStride - 1, 1, h_src );
    v_src[1] = motionErrorLumaFrac_loRes1D_neon<xType>( h_src, xf, maxSampleValue );

    load_s16_16x8x4( buf + 1 * buffStride - 1, 1, h_src );
    v_src[2] = motionErrorLumaFrac_loRes1D_neon<xType>( h_src, xf, maxSampleValue );

    const Pel* rowStart = buf + 2 * buffStride - 1;
    const Pel* origRow = org;

    int32x4_t diffSq0 = vdupq_n_s32( 0 );
    int32x4_t diffSq1 = vdupq_n_s32( 0 );

    int y = h;
    do
    {
      load_s16_16x8x4( rowStart + 0 * buffStride, 1, h_src );
      v_src[3] = motionErrorLumaFrac_loRes1D_neon<xType>( h_src, xf, maxSampleValue );

      load_s16_16x8x4( rowStart + 1 * buffStride, 1, h_src );
      v_src[4] = motionErrorLumaFrac_loRes1D_neon<xType>( h_src, xf, maxSampleValue );

      load_s16_16x8x4( rowStart + 2 * buffStride, 1, h_src );
      v_src[5] = motionErrorLumaFrac_loRes1D_neon<xType>( h_src, xf, maxSampleValue );

      load_s16_16x8x4( rowStart + 3 * buffStride, 1, h_src );
      v_src[6] = motionErrorLumaFrac_loRes1D_neon<xType>( h_src, xf, maxSampleValue );

      int16x8_t ysum0 = motionErrorLumaFrac_loRes1D_neon<yType>( &v_src[0], yf, maxSampleValue );
      int16x8_t ysum1 = motionErrorLumaFrac_loRes1D_neon<yType>( &v_src[1], yf, maxSampleValue );
      int16x8_t ysum2 = motionErrorLumaFrac_loRes1D_neon<yType>( &v_src[2], yf, maxSampleValue );
      int16x8_t ysum3 = motionErrorLumaFrac_loRes1D_neon<yType>( &v_src[3], yf, maxSampleValue );

      int16x8_t orig0 = vld1q_s16( origRow + 0 * origStride );
      int16x8_t orig1 = vld1q_s16( origRow + 1 * origStride );
      int16x8_t orig2 = vld1q_s16( origRow + 2 * origStride );
      int16x8_t orig3 = vld1q_s16( origRow + 3 * origStride );

      int16x8_t diff0 = vabdq_s16( ysum0, orig0 );
      int16x8_t diff1 = vabdq_s16( ysum1, orig1 );
      int16x8_t diff2 = vabdq_s16( ysum2, orig2 );
      int16x8_t diff3 = vabdq_s16( ysum3, orig3 );

      diffSq0 = vmlal_s16( diffSq0, vget_low_s16( diff0 ), vget_low_s16( diff0 ) );
      diffSq0 = vmlal_s16( diffSq0, vget_high_s16( diff0 ), vget_high_s16( diff0 ) );
      diffSq0 = vmlal_s16( diffSq0, vget_low_s16( diff1 ), vget_low_s16( diff1 ) );
      diffSq0 = vmlal_s16( diffSq0, vget_high_s16( diff1 ), vget_high_s16( diff1 ) );

      diffSq1 = vmlal_s16( diffSq1, vget_low_s16( diff2 ), vget_low_s16( diff2 ) );
      diffSq1 = vmlal_s16( diffSq1, vget_high_s16( diff2 ), vget_high_s16( diff2 ) );
      diffSq1 = vmlal_s16( diffSq1, vget_low_s16( diff3 ), vget_low_s16( diff3 ) );
      diffSq1 = vmlal_s16( diffSq1, vget_high_s16( diff3 ), vget_high_s16( diff3 ) );

      v_src[0] = v_src[4];
      v_src[1] = v_src[5];
      v_src[2] = v_src[6];

      rowStart += 4 * buffStride;
      origRow += 4 * origStride;
      y -= 4;
    } while( y != 0 );

    int32x4_t diffSq = vaddq_s32( diffSq0, diffSq1 );
    error += horizontal_add_s32x4( diffSq );
    if( error > besterror )
    {
      return error;
    }

    buf += 8;
    org += 8;
    w -= 8;
  } while( w != 0 );

  return error;
}

template<FilterCoeffType4 xType>
static inline auto get_motionErrorLumaFrac2D( FilterCoeffType4 type )
{
  switch( type )
  {
  case FilterCoeffType4::SkewLeft:
    return &motionErrorLumaFrac_loRes2D_neon<xType, FilterCoeffType4::SkewLeft>;
  case FilterCoeffType4::SkewRight:
    return &motionErrorLumaFrac_loRes2D_neon<xType, FilterCoeffType4::SkewRight>;
  case FilterCoeffType4::FullSymmetric:
    return &motionErrorLumaFrac_loRes2D_neon<xType, FilterCoeffType4::FullSymmetric>;
  case FilterCoeffType4::Generic:
  default:
    return &motionErrorLumaFrac_loRes2D_neon<xType, FilterCoeffType4::Generic>;
  }
}

int motionErrorLumaFrac_loRes_neon( const Pel* org, const ptrdiff_t origStride, const Pel* buf,
                                    const ptrdiff_t buffStride, const int w, const int h, const int16_t* xFilter,
                                    const int16_t* yFilter, const int bitDepth, const int besterror )
{
  const FilterCoeffType4 xType = selectFilterType4( xFilter );
  const FilterCoeffType4 yType = selectFilterType4( yFilter );

  using motionErrorLumaFrac_loResFunc = int ( * )( const Pel*, const ptrdiff_t, const Pel*, const ptrdiff_t, const int,
                                                   const int, const int16_t*, const int16_t*, const int, const int );
  motionErrorLumaFrac_loResFunc func;

  switch( xType )
  {
  case FilterCoeffType4::SkewLeft:
    func = get_motionErrorLumaFrac2D<FilterCoeffType4::SkewLeft>( yType );
    break;
  case FilterCoeffType4::SkewRight:
    func = get_motionErrorLumaFrac2D<FilterCoeffType4::SkewRight>( yType );
    break;
  case FilterCoeffType4::FullSymmetric:
    func = get_motionErrorLumaFrac2D<FilterCoeffType4::FullSymmetric>( yType );
    break;
  case FilterCoeffType4::Generic:
  default:
    func = get_motionErrorLumaFrac2D<FilterCoeffType4::Generic>( yType );
    break;
  }

  int error = func( org, origStride, buf, buffStride, w, h, xFilter, yFilter, bitDepth, besterror );

  return error;
}

static int motionErrorLumaInt_neon( const Pel* org, const ptrdiff_t origStride, const Pel* buf,
                                    const ptrdiff_t buffStride, const int w, int h, const int besterror )
{
  CHECKD( w % 8 != 0, "Width must be a multiple of eight" );
  CHECKD( h % 2 != 0, "Height must be a multiple of two" );

  int error = 0;
  do
  {
    int32x4_t acc_lo = vdupq_n_s32( 0 );
    int32x4_t acc_hi = vdupq_n_s32( 0 );

    int x1 = 0;
    do
    {
      int16x8_t o1 = vld1q_s16( org + x1 );
      int16x8_t b1 = vld1q_s16( buf + x1 );
      int16x8_t o2 = vld1q_s16( org + origStride + x1 );
      int16x8_t b2 = vld1q_s16( buf + buffStride + x1 );

      int16x8_t diff1 = vabdq_s16( o1, b1 );
      int16x8_t diff2 = vabdq_s16( o2, b2 );
      acc_lo = vmlal_s16( acc_lo, vget_low_s16( diff1 ), vget_low_s16( diff1 ) );
      acc_hi = vmlal_s16( acc_hi, vget_high_s16( diff1 ), vget_high_s16( diff1 ) );

      acc_lo = vmlal_s16( acc_lo, vget_low_s16( diff2 ), vget_low_s16( diff2 ) );
      acc_hi = vmlal_s16( acc_hi, vget_high_s16( diff2 ), vget_high_s16( diff2 ) );

      x1 += 8;
    } while( x1 != w );

    int32x4_t diff2_sum = vaddq_s32( acc_lo, acc_hi );
    error += horizontal_add_s32x4( diff2_sum );
    if( error > besterror )
    {
      return error;
    }

    org += 2 * origStride;
    buf += 2 * buffStride;
    h -= 2;
  } while( h != 0 );

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

enum class FilterCoeffType
{
  NoOp = 1,          // No-op filter.
  FullSymmetric,     // Fully symmetric around center: coeff[1] == coeff[6], coeff[2] == coeff[5], coeff[3] == coeff[4].
  AddLeft,           // Left side addition: coeff[1] == 1, coeff[6] == 0.
  AddRight,          // Right side addition: coeff[6] == 1, coeff[1] == 0.
  AddSymmetric,      // Symmetric addition taps: coeff[1] == coeff[6] == 1.
  ShiftLeft,         // Shift Left: coeff[1] == 2, coeff[6] != 2.
  ShiftRight,        // Shift Right: coeff[6] == 2, coeff[1] != 2.
  MultiplySymmetric, // Partially symmetric: coeff[1] == coeff[6], but coeff[2] != coeff[5] or coeff[3] != coeff[4].
};

enum class FilterDirection
{
  Horizontal = 1,
  Vertical
};

using ApplyFrac6TapFunc = void ( * )( const Pel*, ptrdiff_t, Pel*, ptrdiff_t, int, int, const int16_t*, const int16_t*,
                                      int );

static inline FilterCoeffType selectFilterType( const int16_t* coeff )
{
  if( coeff[1] == 0 && coeff[2] == 0 && coeff[4] == 0 && coeff[5] == 0 && coeff[6] == 0 )
    return FilterCoeffType::NoOp;
  if( coeff[1] == coeff[6] && coeff[2] == coeff[5] && coeff[3] == coeff[4] )
    return FilterCoeffType::FullSymmetric;
  if( coeff[1] == 1 && coeff[6] == 1 )
    return FilterCoeffType::AddSymmetric;
  if( coeff[1] == 1 && coeff[6] == 0 )
    return FilterCoeffType::AddLeft;
  if( coeff[6] == 1 && coeff[1] == 0 )
    return FilterCoeffType::AddRight;
  if( coeff[1] == coeff[6] )
    return FilterCoeffType::MultiplySymmetric;
  if( coeff[1] == 2 )
    return FilterCoeffType::ShiftLeft;
  if( coeff[6] == 2 )
    return FilterCoeffType::ShiftRight;

  CHECK( true, "Invalid filter coefficients!" );
  return FilterCoeffType::NoOp; // Fallback dummy.
};

static inline void applyFrac6Tap_8x_copy_neon( const Pel* org, const ptrdiff_t origStride, Pel* dst,
                                               const ptrdiff_t dstStride, int w, int h )
{
  CHECKD( w % 8 != 0, "Width must be multiple of 8!" );
  CHECKD( h % 4 != 0, "Height must be multiple of 4!" );

  h >>= 2;

  do
  {
    int width = w >> 3;
    do
    {
      const int16x8_t src0 = vld1q_s16( org + 0 * origStride );
      const int16x8_t src1 = vld1q_s16( org + 1 * origStride );
      const int16x8_t src2 = vld1q_s16( org + 2 * origStride );
      const int16x8_t src3 = vld1q_s16( org + 3 * origStride );
      vst1q_s16( dst + 0 * dstStride, src0 );
      vst1q_s16( dst + 1 * dstStride, src1 );
      vst1q_s16( dst + 2 * dstStride, src2 );
      vst1q_s16( dst + 3 * dstStride, src3 );

      org += 8;
      dst += 8;
    } while( --width != 0 );

    org += 4 * origStride - w;
    dst += 4 * dstStride - w;
  } while( --h != 0 );
}

template<FilterDirection direction, FilterCoeffType Type>
static inline int16x8_t applyFrac6Tap_8x_filter1D_neon( const int16x8_t* src, const int16x8_t filterVal )
{
  // Filter weight is 64.
  constexpr int filterBits = 6;

  int16x8_t acc_s16 = vdupq_n_s16( 0 );
  int32x4_t acc_s32_lo, acc_s32_hi;

  // Accumulate outer taps (1, 2, 5, 6) in 16-bit, and central taps (3, 4) in 32-bit precision.
  if( Type == FilterCoeffType::FullSymmetric )
  {
    const int16x8_t sum05 = vaddq_s16( src[0], src[5] );
    const int16x8_t sum14 = vaddq_s16( src[1], src[4] );
    const int16x8_t sum23 = vaddq_s16( src[2], src[3] );

    acc_s32_lo = vmull_lane_s16( vget_low_s16( sum05 ), vget_low_s16( filterVal ), 1 );
    acc_s32_hi = vmull_lane_s16( vget_high_s16( sum05 ), vget_low_s16( filterVal ), 1 );
    acc_s32_lo = vmlal_lane_s16( acc_s32_lo, vget_low_s16( sum14 ), vget_low_s16( filterVal ), 2 );
    acc_s32_hi = vmlal_lane_s16( acc_s32_hi, vget_high_s16( sum14 ), vget_low_s16( filterVal ), 2 );
    acc_s32_lo = vmlal_lane_s16( acc_s32_lo, vget_low_s16( sum23 ), vget_low_s16( filterVal ), 3 );
    acc_s32_hi = vmlal_lane_s16( acc_s32_hi, vget_high_s16( sum23 ), vget_low_s16( filterVal ), 3 );

    if( direction == FilterDirection::Horizontal )
    {
      return vcombine_s16( vrshrn_n_s32( acc_s32_lo, filterBits ), vrshrn_n_s32( acc_s32_hi, filterBits ) );
    }
    else // Vertical.
    {
      const uint16x8_t ysum =
          vcombine_u16( vqrshrun_n_s32( acc_s32_lo, filterBits ), vqrshrun_n_s32( acc_s32_hi, filterBits ) );
      return vreinterpretq_s16_u16( ysum );
    }
  }
  else if( Type == FilterCoeffType::AddLeft )
  {
    acc_s16 = vmlaq_lane_s16( src[0], src[1], vget_low_s16( filterVal ), 2 );
    acc_s16 = vmlaq_lane_s16( acc_s16, src[4], vget_high_s16( filterVal ), 1 );
  }
  else if( Type == FilterCoeffType::AddRight )
  {
    acc_s16 = vmlaq_lane_s16( src[5], src[1], vget_low_s16( filterVal ), 2 );
    acc_s16 = vmlaq_lane_s16( acc_s16, src[4], vget_high_s16( filterVal ), 1 );
  }
  else if( Type == FilterCoeffType::AddSymmetric )
  {
    acc_s16 = vaddq_s16( src[0], src[5] );
    acc_s16 = vmlaq_lane_s16( acc_s16, src[1], vget_low_s16( filterVal ), 2 );
    acc_s16 = vmlaq_lane_s16( acc_s16, src[4], vget_high_s16( filterVal ), 1 );
  }
  else if( Type == FilterCoeffType::ShiftLeft )
  {
    acc_s16 = vshlq_n_s16( src[0], 1 );
    acc_s16 = vmlaq_lane_s16( acc_s16, src[1], vget_low_s16( filterVal ), 2 );
    acc_s16 = vmlaq_lane_s16( acc_s16, src[4], vget_high_s16( filterVal ), 1 );
    acc_s16 = vmlaq_lane_s16( acc_s16, src[5], vget_high_s16( filterVal ), 2 );
  }
  else if( Type == FilterCoeffType::ShiftRight )
  {
    acc_s16 = vshlq_n_s16( src[5], 1 );
    acc_s16 = vmlaq_lane_s16( acc_s16, src[0], vget_low_s16( filterVal ), 1 );
    acc_s16 = vmlaq_lane_s16( acc_s16, src[1], vget_low_s16( filterVal ), 2 );
    acc_s16 = vmlaq_lane_s16( acc_s16, src[4], vget_high_s16( filterVal ), 1 );
  }
  else if( Type == FilterCoeffType::MultiplySymmetric )
  {
    int16x8_t sum05 = vaddq_s16( src[0], src[5] );
    acc_s16 = vmulq_lane_s16( sum05, vget_low_s16( filterVal ), 1 );
    acc_s16 = vmlaq_lane_s16( acc_s16, src[1], vget_low_s16( filterVal ), 2 );
    acc_s16 = vmlaq_lane_s16( acc_s16, src[4], vget_high_s16( filterVal ), 1 );
  }

  acc_s32_lo = vmovl_s16( vget_low_s16( acc_s16 ) );
  acc_s32_hi = vmovl_s16( vget_high_s16( acc_s16 ) );
  acc_s32_lo = vmlal_lane_s16( acc_s32_lo, vget_low_s16( src[2] ), vget_low_s16( filterVal ), 3 );
  acc_s32_hi = vmlal_lane_s16( acc_s32_hi, vget_high_s16( src[2] ), vget_low_s16( filterVal ), 3 );
  acc_s32_lo = vmlal_lane_s16( acc_s32_lo, vget_low_s16( src[3] ), vget_high_s16( filterVal ), 0 );
  acc_s32_hi = vmlal_lane_s16( acc_s32_hi, vget_high_s16( src[3] ), vget_high_s16( filterVal ), 0 );

  if( direction == FilterDirection::Horizontal )
  {
    return vcombine_s16( vrshrn_n_s32( acc_s32_lo, filterBits ), vrshrn_n_s32( acc_s32_hi, filterBits ) );
  }
  else // Vertical.
  {
    const uint16x8_t ysum =
        vcombine_u16( vqrshrun_n_s32( acc_s32_lo, filterBits ), vqrshrun_n_s32( acc_s32_hi, filterBits ) );
    return vreinterpretq_s16_u16( ysum );
  }
}

template<FilterCoeffType xType, FilterCoeffType yType>
static inline void applyFrac6Tap_8x_filter2D_neon( const Pel* org, const ptrdiff_t origStride, Pel* dst,
                                                   const ptrdiff_t dstStride, int w, int h, const int16_t* xFilter,
                                                   const int16_t* yFilter, const int bitDepth )
{
  constexpr int numFilterTaps = 6;
  const int maxValue = ( 1 << bitDepth ) - 1;

  CHECKD( w % 8 != 0, "Width must be multiple of 8!" );
  CHECKD( h % 4 != 0, "Height must be multiple of 4!" );

  const Pel* sourceRow = org - ( numFilterTaps / 2 - 1 ) * origStride - ( numFilterTaps / 2 - 1 );

  int16x8_t v_src[numFilterTaps + 3]; // Extra 3 elements needed because the height loop is unrolled 4 times.
  int16x8_t h_src[numFilterTaps];

  const int16x8_t xFilterVal = vld1q_s16( xFilter );
  const int16x8_t yFilterVal = vld1q_s16( yFilter );

  h >>= 2;

  do
  {
    if( xType == FilterCoeffType::NoOp )
    {
      // If yType == FilterCoeffType::NoOp, skip v_src[0] and v_src[1].
      if( yType != FilterCoeffType::NoOp )
      {
        v_src[0] = vld1q_s16( sourceRow + 0 * origStride + 2 );
        v_src[1] = vld1q_s16( sourceRow + 1 * origStride + 2 );
      }
      v_src[2] = vld1q_s16( sourceRow + 2 * origStride + 2 );
      v_src[3] = vld1q_s16( sourceRow + 3 * origStride + 2 );
      v_src[4] = vld1q_s16( sourceRow + 4 * origStride + 2 );
    }
    else
    {
      if( yType != FilterCoeffType::NoOp )
      {
        load_s16_16x8x6( sourceRow + 0 * origStride, 1, h_src );
        v_src[0] = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
        load_s16_16x8x6( sourceRow + 1 * origStride, 1, h_src );
        v_src[1] = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
      }
      load_s16_16x8x6( sourceRow + 2 * origStride, 1, h_src );
      v_src[2] = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
      load_s16_16x8x6( sourceRow + 3 * origStride, 1, h_src );
      v_src[3] = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
      load_s16_16x8x6( sourceRow + 4 * origStride, 1, h_src );
      v_src[4] = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
    }

    const Pel* sourceCol = sourceRow + 5 * origStride;
    Pel* dstCol = dst;

    int height = h;
    do
    {
      if( xType == FilterCoeffType::NoOp )
      {
        v_src[5] = vld1q_s16( sourceCol + 0 * origStride + 2 );
        v_src[6] = vld1q_s16( sourceCol + 1 * origStride + 2 );
        v_src[7] = vld1q_s16( sourceCol + 2 * origStride + 2 );
        v_src[8] = vld1q_s16( sourceCol + 3 * origStride + 2 );
      }
      else
      {
        load_s16_16x8x6( sourceCol + 0 * origStride, 1, h_src );
        v_src[5] = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
        load_s16_16x8x6( sourceCol + 1 * origStride, 1, h_src );
        v_src[6] = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
        load_s16_16x8x6( sourceCol + 2 * origStride, 1, h_src );
        v_src[7] = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
        load_s16_16x8x6( sourceCol + 3 * origStride, 1, h_src );
        v_src[8] = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
      }

      int16x8_t dstSum0, dstSum1, dstSum2, dstSum3;
      if( yType == FilterCoeffType::NoOp )
      {
        dstSum0 = vmaxq_s16( v_src[2], vdupq_n_s16( 0 ) );
        dstSum1 = vmaxq_s16( v_src[3], vdupq_n_s16( 0 ) );
        dstSum2 = vmaxq_s16( v_src[4], vdupq_n_s16( 0 ) );
        dstSum3 = vmaxq_s16( v_src[5], vdupq_n_s16( 0 ) );
      }
      else
      {
        dstSum0 = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Vertical, yType>( &v_src[0], yFilterVal );
        dstSum1 = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Vertical, yType>( &v_src[1], yFilterVal );
        dstSum2 = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Vertical, yType>( &v_src[2], yFilterVal );
        dstSum3 = applyFrac6Tap_8x_filter1D_neon<FilterDirection::Vertical, yType>( &v_src[3], yFilterVal );
      }

      dstSum0 = vminq_s16( dstSum0, vdupq_n_s16( maxValue ) );
      dstSum1 = vminq_s16( dstSum1, vdupq_n_s16( maxValue ) );
      dstSum2 = vminq_s16( dstSum2, vdupq_n_s16( maxValue ) );
      dstSum3 = vminq_s16( dstSum3, vdupq_n_s16( maxValue ) );

      vst1q_s16( dstCol + 0 * dstStride, dstSum0 );
      vst1q_s16( dstCol + 1 * dstStride, dstSum1 );
      vst1q_s16( dstCol + 2 * dstStride, dstSum2 );
      vst1q_s16( dstCol + 3 * dstStride, dstSum3 );

      if( yType != FilterCoeffType::NoOp )
      {
        v_src[0] = v_src[4];
        v_src[1] = v_src[5];
      }
      v_src[2] = v_src[6];
      v_src[3] = v_src[7];
      v_src[4] = v_src[8];

      dstCol += 4 * dstStride;
      sourceCol += 4 * origStride;
    } while( --height != 0 );

    sourceRow += 8;
    dst += 8;
    w -= 8;
  } while( w != 0 );
}

template<FilterCoeffType xType>
static inline ApplyFrac6TapFunc applyFrac6Tap_8x_getYEntry( FilterCoeffType yType )
{
  switch( yType )
  {
  case FilterCoeffType::NoOp:
    return &applyFrac6Tap_8x_filter2D_neon<xType, FilterCoeffType::NoOp>;
  case FilterCoeffType::FullSymmetric:
    return &applyFrac6Tap_8x_filter2D_neon<xType, FilterCoeffType::FullSymmetric>;
  case FilterCoeffType::AddLeft:
    return &applyFrac6Tap_8x_filter2D_neon<xType, FilterCoeffType::AddLeft>;
  case FilterCoeffType::AddRight:
    return &applyFrac6Tap_8x_filter2D_neon<xType, FilterCoeffType::AddRight>;
  case FilterCoeffType::AddSymmetric:
    return &applyFrac6Tap_8x_filter2D_neon<xType, FilterCoeffType::AddSymmetric>;
  case FilterCoeffType::ShiftLeft:
    return &applyFrac6Tap_8x_filter2D_neon<xType, FilterCoeffType::ShiftLeft>;
  case FilterCoeffType::ShiftRight:
    return &applyFrac6Tap_8x_filter2D_neon<xType, FilterCoeffType::ShiftRight>;
  case FilterCoeffType::MultiplySymmetric:
    return &applyFrac6Tap_8x_filter2D_neon<xType, FilterCoeffType::MultiplySymmetric>;
  }

  CHECK( true, "Invalid filter coefficients!" );
  return nullptr;
}

void applyFrac6Tap_8x_neon( const Pel* org, const ptrdiff_t origStride, Pel* dst, const ptrdiff_t dstStride,
                            const int w, const int h, const int16_t* xFilter, const int16_t* yFilter,
                            const int bitDepth )
{
  const FilterCoeffType xType = selectFilterType( xFilter );
  const FilterCoeffType yType = selectFilterType( yFilter );

  if( xType == FilterCoeffType::NoOp && yType == FilterCoeffType::NoOp )
  {
    applyFrac6Tap_8x_copy_neon( org, origStride, dst, dstStride, w, h );
    return;
  }

  ApplyFrac6TapFunc func{ nullptr };

  switch( xType )
  {
  case FilterCoeffType::NoOp:
    func = applyFrac6Tap_8x_getYEntry<FilterCoeffType::NoOp>( yType );
    break;
  case FilterCoeffType::FullSymmetric:
    func = applyFrac6Tap_8x_getYEntry<FilterCoeffType::FullSymmetric>( yType );
    break;
  case FilterCoeffType::AddLeft:
    func = applyFrac6Tap_8x_getYEntry<FilterCoeffType::AddLeft>( yType );
    break;
  case FilterCoeffType::AddRight:
    func = applyFrac6Tap_8x_getYEntry<FilterCoeffType::AddRight>( yType );
    break;
  case FilterCoeffType::AddSymmetric:
    func = applyFrac6Tap_8x_getYEntry<FilterCoeffType::AddSymmetric>( yType );
    break;
  case FilterCoeffType::ShiftLeft:
    func = applyFrac6Tap_8x_getYEntry<FilterCoeffType::ShiftLeft>( yType );
    break;
  case FilterCoeffType::ShiftRight:
    func = applyFrac6Tap_8x_getYEntry<FilterCoeffType::ShiftRight>( yType );
    break;
  case FilterCoeffType::MultiplySymmetric:
    func = applyFrac6Tap_8x_getYEntry<FilterCoeffType::MultiplySymmetric>( yType );
    break;
  }

  CHECKD( func == nullptr, "Invalid filter type!" );

  func( org, origStride, dst, dstStride, w, h, xFilter, yFilter, bitDepth );
}

static inline void applyFrac6Tap_4x_copy_neon( const Pel* org, const ptrdiff_t origStride, Pel* dst,
                                               const ptrdiff_t dstStride, int w, int h )
{
  CHECKD( w != 4, "w must be 4!" );
  CHECKD( h % 4 != 0, "Height must be multiple of 4!" );

  h >>= 2;

  do
  {
    const int16x4_t src0 = vld1_s16( org + 0 * origStride );
    const int16x4_t src1 = vld1_s16( org + 1 * origStride );
    const int16x4_t src2 = vld1_s16( org + 2 * origStride );
    const int16x4_t src3 = vld1_s16( org + 3 * origStride );
    vst1_s16( dst + 0 * dstStride, src0 );
    vst1_s16( dst + 1 * dstStride, src1 );
    vst1_s16( dst + 2 * dstStride, src2 );
    vst1_s16( dst + 3 * dstStride, src3 );

    org += 4 * origStride;
    dst += 4 * dstStride;
  } while( --h != 0 );
}

template<FilterDirection direction, FilterCoeffType Type>
static inline int16x4_t applyFrac6Tap_4x_filter1D_neon( const int16x4_t* src, const int16x8_t filterVal )
{
  // Filter weight is 64.
  constexpr int filterBits = 6;

  int16x4_t acc_s16 = vdup_n_s16( 0 );
  int32x4_t acc_s32;

  // Accumulate outer taps (1, 2, 5, 6) in 16-bit, and central taps (3, 4) in 32-bit precision.
  if( Type == FilterCoeffType::FullSymmetric )
  {
    const int16x4_t sum05 = vadd_s16( src[0], src[5] );
    const int16x4_t sum14 = vadd_s16( src[1], src[4] );
    const int16x4_t sum23 = vadd_s16( src[2], src[3] );
    int16x4_t acc_s16 = vmul_lane_s16( sum05, vget_low_s16( filterVal ), 1 );
    acc_s16 = vmla_lane_s16( acc_s16, sum14, vget_low_s16( filterVal ), 2 );
    acc_s32 = vmovl_s16( acc_s16 );
    acc_s32 = vmlal_lane_s16( acc_s32, sum23, vget_low_s16( filterVal ), 3 );

    if( direction == FilterDirection::Horizontal )
    {
      return vqrshrn_n_s32( acc_s32, filterBits );
    }
    else // Vertical.
    {
      return vreinterpret_s16_u16( vqrshrun_n_s32( acc_s32, filterBits ) );
    }
  }
  else if( Type == FilterCoeffType::AddLeft )
  {
    acc_s16 = vmla_lane_s16( src[0], src[1], vget_low_s16( filterVal ), 2 );
    acc_s16 = vmla_lane_s16( acc_s16, src[4], vget_high_s16( filterVal ), 1 );
  }
  else if( Type == FilterCoeffType::AddRight )
  {
    acc_s16 = vmla_lane_s16( src[5], src[1], vget_low_s16( filterVal ), 2 );
    acc_s16 = vmla_lane_s16( acc_s16, src[4], vget_high_s16( filterVal ), 1 );
  }
  else if( Type == FilterCoeffType::AddSymmetric )
  {
    acc_s16 = vadd_s16( src[0], src[5] );
    acc_s16 = vmla_lane_s16( acc_s16, src[1], vget_low_s16( filterVal ), 2 );
    acc_s16 = vmla_lane_s16( acc_s16, src[4], vget_high_s16( filterVal ), 1 );
  }
  else if( Type == FilterCoeffType::ShiftLeft )
  {
    acc_s16 = vshl_n_s16( src[0], 1 );
    acc_s16 = vmla_lane_s16( acc_s16, src[1], vget_low_s16( filterVal ), 2 );
    acc_s16 = vmla_lane_s16( acc_s16, src[4], vget_high_s16( filterVal ), 1 );
    acc_s16 = vmla_lane_s16( acc_s16, src[5], vget_high_s16( filterVal ), 2 );
  }
  else if( Type == FilterCoeffType::ShiftRight )
  {
    acc_s16 = vshl_n_s16( src[5], 1 );
    acc_s16 = vmla_lane_s16( acc_s16, src[0], vget_low_s16( filterVal ), 1 );
    acc_s16 = vmla_lane_s16( acc_s16, src[1], vget_low_s16( filterVal ), 2 );
    acc_s16 = vmla_lane_s16( acc_s16, src[4], vget_high_s16( filterVal ), 1 );
  }
  else if( Type == FilterCoeffType::MultiplySymmetric )
  {
    int16x4_t sum05 = vadd_s16( src[0], src[5] );
    acc_s16 = vmul_lane_s16( sum05, vget_low_s16( filterVal ), 1 );
    acc_s16 = vmla_lane_s16( acc_s16, src[1], vget_low_s16( filterVal ), 2 );
    acc_s16 = vmla_lane_s16( acc_s16, src[4], vget_high_s16( filterVal ), 1 );
  }

  acc_s32 = vmovl_s16( acc_s16 );
  acc_s32 = vmlal_lane_s16( acc_s32, src[2], vget_low_s16( filterVal ), 3 );
  acc_s32 = vmlal_lane_s16( acc_s32, src[3], vget_high_s16( filterVal ), 0 );

  if( direction == FilterDirection::Horizontal )
  {
    return vqrshrn_n_s32( acc_s32, filterBits );
  }
  else // Vertical.
  {
    return vreinterpret_s16_u16( vqrshrun_n_s32( acc_s32, filterBits ) );
  }
}

template<FilterCoeffType xType, FilterCoeffType yType>
static inline void applyFrac6Tap_4x_filter2D_neon( const Pel* org, const ptrdiff_t origStride, Pel* dst,
                                                   const ptrdiff_t dstStride, int w, int h, const int16_t* xFilter,
                                                   const int16_t* yFilter, const int bitDepth )
{
  CHECKD( w != 4, "w must be 4!" );
  CHECKD( h % 4 != 0, "Height must be multiple of 4!" );

  constexpr int numFilterTaps = 6;
  const int maxValue = ( 1 << bitDepth ) - 1;

  int16x4_t v_src[numFilterTaps + 3]; // Extra 3 elements needed because the height loop is unrolled 4 times.
  int16x4_t h_src[numFilterTaps];

  const Pel* sourceRow = org - ( numFilterTaps / 2 - 1 ) * origStride - ( numFilterTaps / 2 - 1 );

  const int16x8_t xFilterVal = vld1q_s16( xFilter );
  const int16x8_t yFilterVal = vld1q_s16( yFilter );

  if( xType == FilterCoeffType::NoOp )
  {
    if( yType != FilterCoeffType::NoOp )
    {
      v_src[0] = vld1_s16( sourceRow + 0 * origStride + 2 );
      v_src[1] = vld1_s16( sourceRow + 1 * origStride + 2 );
    }
    v_src[2] = vld1_s16( sourceRow + 2 * origStride + 2 );
    v_src[3] = vld1_s16( sourceRow + 3 * origStride + 2 );
    v_src[4] = vld1_s16( sourceRow + 4 * origStride + 2 );
  }
  else
  {
    if( yType != FilterCoeffType::NoOp )
    {
      load_s16x4x6( sourceRow + 0 * origStride, 1, h_src );
      v_src[0] = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
      load_s16x4x6( sourceRow + 1 * origStride, 1, h_src );
      v_src[1] = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
    }
    load_s16x4x6( sourceRow + 2 * origStride, 1, h_src );
    v_src[2] = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
    load_s16x4x6( sourceRow + 3 * origStride, 1, h_src );
    v_src[3] = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
    load_s16x4x6( sourceRow + 4 * origStride, 1, h_src );
    v_src[4] = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
  }

  const Pel* sourceCol = sourceRow + 5 * origStride;
  Pel* dstCol = dst;

  h >>= 2;

  do
  {
    if( xType == FilterCoeffType::NoOp )
    {
      v_src[5] = vld1_s16( sourceCol + 0 * origStride + 2 );
      v_src[6] = vld1_s16( sourceCol + 1 * origStride + 2 );
      v_src[7] = vld1_s16( sourceCol + 2 * origStride + 2 );
      v_src[8] = vld1_s16( sourceCol + 3 * origStride + 2 );
    }
    else
    {
      load_s16x4x6( sourceCol + 0 * origStride, 1, h_src );
      v_src[5] = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
      load_s16x4x6( sourceCol + 1 * origStride, 1, h_src );
      v_src[6] = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
      load_s16x4x6( sourceCol + 2 * origStride, 1, h_src );
      v_src[7] = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
      load_s16x4x6( sourceCol + 3 * origStride, 1, h_src );
      v_src[8] = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Horizontal, xType>( h_src, xFilterVal );
    }

    int16x4_t dstSum0, dstSum1, dstSum2, dstSum3;
    if( yType == FilterCoeffType::NoOp )
    {
      dstSum0 = vmax_s16( v_src[2], vdup_n_s16( 0 ) );
      dstSum1 = vmax_s16( v_src[3], vdup_n_s16( 0 ) );
      dstSum2 = vmax_s16( v_src[4], vdup_n_s16( 0 ) );
      dstSum3 = vmax_s16( v_src[5], vdup_n_s16( 0 ) );
    }
    else
    {
      dstSum0 = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Vertical, yType>( &v_src[0], yFilterVal );
      dstSum1 = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Vertical, yType>( &v_src[1], yFilterVal );
      dstSum2 = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Vertical, yType>( &v_src[2], yFilterVal );
      dstSum3 = applyFrac6Tap_4x_filter1D_neon<FilterDirection::Vertical, yType>( &v_src[3], yFilterVal );
    }
    dstSum0 = vmin_s16( dstSum0, vdup_n_s16( maxValue ) );
    dstSum1 = vmin_s16( dstSum1, vdup_n_s16( maxValue ) );
    dstSum2 = vmin_s16( dstSum2, vdup_n_s16( maxValue ) );
    dstSum3 = vmin_s16( dstSum3, vdup_n_s16( maxValue ) );

    vst1_s16( dstCol + 0 * dstStride, dstSum0 );
    vst1_s16( dstCol + 1 * dstStride, dstSum1 );
    vst1_s16( dstCol + 2 * dstStride, dstSum2 );
    vst1_s16( dstCol + 3 * dstStride, dstSum3 );

    if( yType != FilterCoeffType::NoOp )
    {
      v_src[0] = v_src[4];
      v_src[1] = v_src[5];
    }
    v_src[2] = v_src[6];
    v_src[3] = v_src[7];
    v_src[4] = v_src[8];

    dstCol += 4 * dstStride;
    sourceCol += 4 * origStride;
  } while( --h != 0 );
}

template<FilterCoeffType xType>
static inline ApplyFrac6TapFunc applyFrac6Tap_4x_getYEntry( FilterCoeffType yType )
{
  switch( yType )
  {
  case FilterCoeffType::NoOp:
    return &applyFrac6Tap_4x_filter2D_neon<xType, FilterCoeffType::NoOp>;
  case FilterCoeffType::FullSymmetric:
    return &applyFrac6Tap_4x_filter2D_neon<xType, FilterCoeffType::FullSymmetric>;
  case FilterCoeffType::AddLeft:
    return &applyFrac6Tap_4x_filter2D_neon<xType, FilterCoeffType::AddLeft>;
  case FilterCoeffType::AddRight:
    return &applyFrac6Tap_4x_filter2D_neon<xType, FilterCoeffType::AddRight>;
  case FilterCoeffType::AddSymmetric:
    return &applyFrac6Tap_4x_filter2D_neon<xType, FilterCoeffType::AddSymmetric>;
  case FilterCoeffType::ShiftLeft:
    return &applyFrac6Tap_4x_filter2D_neon<xType, FilterCoeffType::ShiftLeft>;
  case FilterCoeffType::ShiftRight:
    return &applyFrac6Tap_4x_filter2D_neon<xType, FilterCoeffType::ShiftRight>;
  case FilterCoeffType::MultiplySymmetric:
    return &applyFrac6Tap_4x_filter2D_neon<xType, FilterCoeffType::MultiplySymmetric>;
  }

  CHECK( true, "Invalid filter coefficients!" );
  return nullptr;
}

void applyFrac6Tap_4x_neon( const Pel* org, const ptrdiff_t origStride, Pel* dst, const ptrdiff_t dstStride,
                            const int w, const int h, const int16_t* xFilter, const int16_t* yFilter,
                            const int bitDepth )
{
  CHECKD( w % 4 != 0, "Width must be multiple of 4!" );
  CHECKD( h % 4 != 0, "Height must be multiple of 4!" );

  int width8x = ( w / 8 ) * 8;
  if( width8x > 0 )
  {
    applyFrac6Tap_8x_neon( org, origStride, dst, dstStride, width8x, h, xFilter, yFilter, bitDepth );
  }

  int tailWidth = w - width8x;
  if( tailWidth > 0 )
  {
    org += width8x;
    dst += width8x;

    const FilterCoeffType xType = selectFilterType( xFilter );
    const FilterCoeffType yType = selectFilterType( yFilter );

    if( xType == FilterCoeffType::NoOp && yType == FilterCoeffType::NoOp )
    {
      applyFrac6Tap_4x_copy_neon( org, origStride, dst, dstStride, w, h );
      return;
    }

    ApplyFrac6TapFunc func{ nullptr };

    switch( xType )
    {
    case FilterCoeffType::NoOp:
      func = applyFrac6Tap_4x_getYEntry<FilterCoeffType::NoOp>( yType );
      break;
    case FilterCoeffType::FullSymmetric:
      func = applyFrac6Tap_4x_getYEntry<FilterCoeffType::FullSymmetric>( yType );
      break;
    case FilterCoeffType::AddLeft:
      func = applyFrac6Tap_4x_getYEntry<FilterCoeffType::AddLeft>( yType );
      break;
    case FilterCoeffType::AddRight:
      func = applyFrac6Tap_4x_getYEntry<FilterCoeffType::AddRight>( yType );
      break;
    case FilterCoeffType::AddSymmetric:
      func = applyFrac6Tap_4x_getYEntry<FilterCoeffType::AddSymmetric>( yType );
      break;
    case FilterCoeffType::ShiftLeft:
      func = applyFrac6Tap_4x_getYEntry<FilterCoeffType::ShiftLeft>( yType );
      break;
    case FilterCoeffType::ShiftRight:
      func = applyFrac6Tap_4x_getYEntry<FilterCoeffType::ShiftRight>( yType );
      break;
    case FilterCoeffType::MultiplySymmetric:
      func = applyFrac6Tap_4x_getYEntry<FilterCoeffType::MultiplySymmetric>( yType );
      break;
    }

    CHECKD( func == nullptr, "Invalid filter type!" );

    func( org, origStride, dst, dstStride, w, h, xFilter, yFilter, bitDepth );
  }
}

template<>
void MCTF::_initMCTF_ARM<NEON>()
{
  m_motionErrorLumaFrac8[1] = motionErrorLumaFrac_loRes_neon;
  m_motionErrorLumaInt8     = motionErrorLumaInt_neon;
  m_applyPlanarCorrection   = applyPlanarCorrection_neon;
  m_applyBlock              = applyBlock_neon;
  m_applyFrac[0][0]         = applyFrac6Tap_8x_neon;
  m_applyFrac[1][0]         = applyFrac6Tap_4x_neon;
}

} // namespace vvenc
#endif
//! \}

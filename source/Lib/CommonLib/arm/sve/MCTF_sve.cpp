/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2025-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
 * \file MCTF_sve.cpp
 * \brief SVE implementation of MCTF for AArch64.
 */

//  ====================================================================================================================
//  Includes
//  ====================================================================================================================
#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"

#include "MCTF.h"

#include <math.h>

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCTF

#include "MCTF_neon.h"
#include "mem_neon.h"
#include "neon_sve_bridge.h"
#include <arm_neon.h>
#include <arm_sve.h>

namespace vvenc
{
int motionErrorLumaInt_sve( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride,
                            const int w, int h, const int besterror )
{
  CHECKD( w % 8 != 0, "Width must be a multiple of eight" );
  CHECKD( h % 4 != 0, "Height must be a multiple of four" );

  int error = 0;
  do
  {
    int64x2_t acc1 = vdupq_n_s64( 0 );
    int64x2_t acc2 = vdupq_n_s64( 0 );

    int x1 = 0;
    do
    {
      int16x8_t o1 = vld1q_s16( org + 0 * origStride + x1 );
      int16x8_t b1 = vld1q_s16( buf + 0 * origStride + x1 );
      int16x8_t o2 = vld1q_s16( org + 1 * origStride + x1 );
      int16x8_t b2 = vld1q_s16( buf + 1 * buffStride + x1 );
      int16x8_t o3 = vld1q_s16( org + 2 * origStride + x1 );
      int16x8_t b3 = vld1q_s16( buf + 2 * buffStride + x1 );
      int16x8_t o4 = vld1q_s16( org + 3 * origStride + x1 );
      int16x8_t b4 = vld1q_s16( buf + 3 * buffStride + x1 );

      int16x8_t diff1 = vabdq_s16( o1, b1 );
      int16x8_t diff2 = vabdq_s16( o2, b2 );
      int16x8_t diff3 = vabdq_s16( o3, b3 );
      int16x8_t diff4 = vabdq_s16( o4, b4 );

      acc1 = vvenc_sdotq_s16( acc1, diff1, diff1 );
      acc2 = vvenc_sdotq_s16( acc2, diff2, diff2 );
      acc1 = vvenc_sdotq_s16( acc1, diff3, diff3 );
      acc2 = vvenc_sdotq_s16( acc2, diff4, diff4 );

      x1 += 8;
    } while( x1 != w );

    int64x2_t diff2_sum = vaddq_s64( acc1, acc2 );
    error += ( int32_t )vaddvq_s64( diff2_sum );
    if( error > besterror )
    {
      return error;
    }

    org += 4 * origStride;
    buf += 4 * buffStride;
    h -= 4;
  } while( h != 0 );

  return error;
}

template<FilterCoeffType4 xType, FilterCoeffType4 yType>
static inline int motionErrorLumaFrac_loRes2D_sve( const Pel* org, const ptrdiff_t origStride, const Pel* buf,
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

    int64x2_t diffSq0 = vdupq_n_s64( 0 );
    int64x2_t diffSq1 = vdupq_n_s64( 0 );

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

      diffSq0 = vvenc_sdotq_s16( diffSq0, diff0, diff0 );
      diffSq0 = vvenc_sdotq_s16( diffSq0, diff1, diff1 );
      diffSq1 = vvenc_sdotq_s16( diffSq1, diff2, diff2 );
      diffSq1 = vvenc_sdotq_s16( diffSq1, diff3, diff3 );

      v_src[0] = v_src[4];
      v_src[1] = v_src[5];
      v_src[2] = v_src[6];

      rowStart += 4 * buffStride;
      origRow += 4 * origStride;
      y -= 4;
    } while( y != 0 );

    int64x2_t diffSq = vaddq_s64( diffSq0, diffSq1 );
    error += ( int32_t )vaddvq_s64( diffSq );
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
    return &motionErrorLumaFrac_loRes2D_sve<xType, FilterCoeffType4::SkewLeft>;
  case FilterCoeffType4::SkewRight:
    return &motionErrorLumaFrac_loRes2D_sve<xType, FilterCoeffType4::SkewRight>;
  case FilterCoeffType4::FullSymmetric:
    return &motionErrorLumaFrac_loRes2D_sve<xType, FilterCoeffType4::FullSymmetric>;
  case FilterCoeffType4::Generic:
  default:
    return &motionErrorLumaFrac_loRes2D_sve<xType, FilterCoeffType4::Generic>;
  }
}

int motionErrorLumaFrac_loRes_sve( const Pel* org, const ptrdiff_t origStride, const Pel* buf,
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

  return func( org, origStride, buf, buffStride, w, h, xFilter, yFilter, bitDepth, besterror );
}

void applyPlanarCorrection_sve( const Pel* refPel, const ptrdiff_t refStride, Pel* dstPel, const ptrdiff_t dstStride,
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
    int64x2_t acc_x = vdupq_n_s64( 0 );

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

        acc_x = vvenc_sdotq_s16( acc_x, diff0, xv );
        acc_x = vvenc_sdotq_s16( acc_x, diff1, xv );

        xv = vaddq_s16( xv, vdupq_n_s16( 8 ) );
        x += 8;
      } while( x != w );

      int32_t zAcc0 = vaddlvq_s16( acc_z0 );
      int32_t zAcc1 = vaddlvq_s16( acc_z1 );

      x2yzm += y * zAcc0 + ( y + 1 ) * zAcc1; // Σ(y*z) calculated as y * Σ(z).
      ySum += zAcc0 + zAcc1;                  // Σ(z)

      pDst += dstStride << 1;
      pRef += refStride << 1;
    }
    x1yzm = ( int32_t )vaddvq_s64( acc_x ); // Σ(x*z)
  }
  else // w = h = 4
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

    ySum = vaddlvq_s16( acc_z );  // Σ(z)
    x2yzm = vaddlvq_s16( acc_y ); // Σ(y*z)
    x1yzm = vaddlvq_s16( acc_x ); // Σ(x*z)
  }

  applyPlanarDeblockingCorrection_common( dstPel, dstStride, x1yzm, x2yzm, ySum, w, h, clpRng, motionError );
}

void applyBlock_sve( const CPelBuf& src, PelBuf& dst, const CompArea& blk, const ClpRng& clpRng,
                     const Pel** correctedPics, int numRefs, const int* verror, const double* refStrenghts,
                     double weightScaling, double sigmaSq )
{
  const int w = blk.width;
  const int h = blk.height;
  const int bx = blk.x;
  const int by = blk.y;

  CHECKD( w < 4, "Width must be greater than or equal to 4!" );
  CHECKD( h % 2 != 0, "Height must be multiple of 2!" );

  const ptrdiff_t srcStride = src.stride;
  const Pel* srcPel = src.bufAt( bx, by );

  int vnoise[2 * VVENC_MCTF_RANGE] = { 0 };

  int minError = INT32_MAX;

  int i = numRefs - 1;
  do
  {
    int64_t variance = 0, diffsum = 0;
    const ptrdiff_t refStride = w;
    const Pel* refBuf = correctedPics[i];
    const Pel* srcBuf = srcPel;

    if( w % 8 == 0 )
    {
      int64x2_t variance_acc = vdupq_n_s64( 0 );
      int64x2_t diffR_acc = vdupq_n_s64( 0 );
      int64x2_t diffD_acc = vdupq_n_s64( 0 );
      const int16_t ind[] = { 1, 1, 1, 1, 1, 1, 1, 0 };
      const int16x8_t clear_lastlane = vld1q_s16( ind );

      int y = h >> 1;

      do
      {
        // One iteration for x done outside loop.
        for( int x = 0; x < w - 8; x += 8 )
        {
          int16x8_t pix0 = vld1q_s16( srcBuf );         // unsigned 10bit
          int16x8_t ref0 = vld1q_s16( refBuf );         // unsigned 10bit
          int16x8_t pixNext0 = vld1q_s16( srcBuf + 8 ); // unsigned 10bit
          int16x8_t refNext0 = vld1q_s16( refBuf + 8 ); // unsigned 10bit
          int16x8_t diff0 = vsubq_s16( pix0, ref0 );    // 11bit
          int16x8_t diffNext0 = vsubq_s16( pixNext0, refNext0 );

          int16x8_t pix1 = vld1q_s16( srcBuf + srcStride );         // unsigned 10bit
          int16x8_t ref1 = vld1q_s16( refBuf + refStride );         // unsigned 10bit
          int16x8_t pixNext1 = vld1q_s16( srcBuf + srcStride + 8 ); // unsigned 10bit
          int16x8_t refNext1 = vld1q_s16( refBuf + refStride + 8 ); // unsigned 10bit
          int16x8_t diff1 = vsubq_s16( pix1, ref1 );                // 11bit
          int16x8_t diffNext1 = vsubq_s16( pixNext1, refNext1 );

          int16x8_t diffR0 = vextq_s16( diff0, diffNext0, 1 );
          int16x8_t diffR1 = vextq_s16( diff1, diffNext1, 1 );
          int16x8_t diffD0 = vsubq_s16( diff1, diff0 ); // 11bit

          diffR0 = vsubq_s16( diffR0, diff0 ); // 11bit
          diffR1 = vsubq_s16( diffR1, diff1 ); // 11bit

          variance_acc = vvenc_sdotq_s16( variance_acc, diff0, diff0 ); // 29bit
          diffR_acc = vvenc_sdotq_s16( diffR_acc, diffR0, diffR0 );     // 31bit
          diffD_acc = vvenc_sdotq_s16( diffD_acc, diffD0, diffD0 );     // 31bit

          variance_acc = vvenc_sdotq_s16( variance_acc, diff1, diff1 ); // 29bit
          diffR_acc = vvenc_sdotq_s16( diffR_acc, diffR1, diffR1 );     // 31bit

          if( y != 1 )
          {
            int16x8_t pixD1 = vld1q_s16( srcBuf + 2 * srcStride ); // unsigned 10bit
            int16x8_t refD1 = vld1q_s16( refBuf + 2 * refStride ); // unsigned 10bit

            int16x8_t diffD1 = vsubq_s16( pixD1, refD1 );             // 11bit
            diffD1 = vsubq_s16( diffD1, diff1 );                      // 11bit
            diffD_acc = vvenc_sdotq_s16( diffD_acc, diffD1, diffD1 ); // 31bit
          }

          srcBuf += 8;
          refBuf += 8;
        }

        // Last iteration of x.
        int16x8_t pix0 = vld1q_s16( srcBuf );             // unsigned 10bit
        int16x8_t ref0 = vld1q_s16( refBuf );             // unsigned 10bit
        int16x8_t pix1 = vld1q_s16( srcBuf + srcStride ); // unsigned 10bit
        int16x8_t ref1 = vld1q_s16( refBuf + refStride ); // unsigned 10bit
        int16x8_t diff0 = vsubq_s16( pix0, ref0 );        // 11bit
        int16x8_t diff1 = vsubq_s16( pix1, ref1 );        // 11bit

        int16x8_t diffR0 = vextq_s16( diff0, vdupq_n_s16( 0 ), 1 );
        int16x8_t diffR1 = vextq_s16( diff1, vdupq_n_s16( 0 ), 1 );
        int16x8_t diffD0 = vsubq_s16( diff1, diff0 ); // 11bit

        diffR0 = vmlsq_s16( diffR0, diff0, clear_lastlane ); // 11 bit
        diffR1 = vmlsq_s16( diffR1, diff1, clear_lastlane );

        variance_acc = vvenc_sdotq_s16( variance_acc, diff0, diff0 ); // 29bit
        diffR_acc = vvenc_sdotq_s16( diffR_acc, diffR0, diffR0 );     // 31bit
        diffD_acc = vvenc_sdotq_s16( diffD_acc, diffD0, diffD0 );     // 31bit

        variance_acc = vvenc_sdotq_s16( variance_acc, diff1, diff1 ); // 29bit
        diffR_acc = vvenc_sdotq_s16( diffR_acc, diffR1, diffR1 );     // 31bit

        if( y != 1 )
        {
          int16x8_t pixD1 = vld1q_s16( srcBuf + 2 * srcStride ); // 10bit unsigned
          int16x8_t refD1 = vld1q_s16( refBuf + 2 * refStride ); // 10bit unsigned

          int16x8_t diffD1 = vsubq_s16( pixD1, refD1 );             // 11bit
          diffD1 = vsubq_s16( diffD1, diff1 );                      // 11bit
          diffD_acc = vvenc_sdotq_s16( diffD_acc, diffD1, diffD1 ); // 31bit
        }

        refBuf += 2 * refStride - ( w - 8 );
        srcBuf += 2 * srcStride - ( w - 8 );
      } while( --y != 0 );

      variance = vaddvq_s64( variance_acc );
      diffsum = vaddvq_s64( vaddq_s64( diffR_acc, diffD_acc ) );
    }
    else
    {
      CHECKD( w != 4, "Width must be equal to 4!" );

      int64x2_t variance_acc = vdupq_n_s64( 0 );
      int64x2_t diffR_acc = vdupq_n_s64( 0 );
      int64x2_t diffD_acc = vdupq_n_s64( 0 );

      // -1 for selecting a lane.
      const int16_t mask[] = { -1, -1, -1, 0, -1, -1, -1, 0 };
      const int16x8_t mask_lanes = vld1q_s16( mask );

      int16x4_t pixD = vld1_s16( srcBuf );
      int16x4_t refD = vld1_s16( refBuf );

      for( int y = 0; y < h - 2; y += 2 )
      {
        int16x8_t pix = vcombine_s16( pixD, vld1_s16( srcBuf + srcStride ) );
        int16x8_t ref = vcombine_s16( refD, vld1_s16( refBuf + refStride ) );

        int16x8_t diff = vsubq_s16( pix, ref );
        int16x8_t diffR = vextq_s16( diff, vdupq_n_s16( 0 ), 1 );
        diffR = vsubq_s16( diffR, diff );
        diffR = vandq_s16( diffR, mask_lanes );

        // pixD and refD used for next iteration too.
        pixD = vld1_s16( srcBuf + 2 * srcStride );
        refD = vld1_s16( refBuf + 2 * refStride );

        variance_acc = vvenc_sdotq_s16( variance_acc, diff, diff );
        diffR_acc = vvenc_sdotq_s16( diffR_acc, diffR, diffR );

        int16x4_t tmp_diffD = vsub_s16( pixD, refD );
        int16x8_t diffD = vcombine_s16( vget_high_s16( diff ), tmp_diffD );
        diffD = vsubq_s16( diffD, diff );
        diffD_acc = vvenc_sdotq_s16( diffD_acc, diffD, diffD );

        refBuf += 2 * refStride;
        srcBuf += 2 * srcStride;
      }

      // Last iteration for y done outside loop.
      int16x8_t pix = vcombine_s16( pixD, vld1_s16( srcBuf + srcStride ) );
      int16x8_t ref = vcombine_s16( refD, vld1_s16( refBuf + refStride ) );
      int16x8_t diff = vsubq_s16( pix, ref );

      int16x8_t diffR = vextq_s16( diff, vdupq_n_s16( 0 ), 1 );
      diffR = vsubq_s16( diffR, diff );
      diffR = vandq_s16( diffR, mask_lanes );

      variance_acc = vvenc_sdotq_s16( variance_acc, diff, diff );
      diffR_acc = vvenc_sdotq_s16( diffR_acc, diffR, diffR );

      // Low four lanes of diffD will be cleared after sub.
      int16x8_t diffD = vcombine_s16( vget_high_s16( diff ), vget_high_s16( diff ) );
      diffD = vsubq_s16( diffD, diff );
      diffD_acc = vvenc_sdotq_s16( diffD_acc, diffD, diffD );

      variance = vaddvq_s64( variance_acc );
      diffsum = vaddvq_s64( vaddq_s64( diffR_acc, diffD_acc ) );
    }

    variance <<= 2 * ( 10 - clpRng.bd );
    diffsum <<= 2 * ( 10 - clpRng.bd );
    const int cntV = w * h;
    const int cntD = 2 * cntV - w - h;
    vnoise[i] = ( int )round( ( 15.0 * cntD / cntV * variance + 5.0 ) / ( diffsum + 5.0 ) );
    minError = std::min( minError, verror[i] );
  } while( i-- != 0 );

  applyBlock_common( src, dst, blk, clpRng, correctedPics, numRefs, verror, vnoise, refStrenghts, minError,
                     weightScaling, sigmaSq );
}

template<>
void MCTF::_initMCTF_ARM<SVE>()
{
  m_motionErrorLumaInt8 = motionErrorLumaInt_sve;
  m_motionErrorLumaFrac8[1] = motionErrorLumaFrac_loRes_sve;
  m_applyPlanarCorrection = applyPlanarCorrection_sve;
  m_applyBlock = applyBlock_sve;
}

} // namespace vvenc

#endif
//! \}

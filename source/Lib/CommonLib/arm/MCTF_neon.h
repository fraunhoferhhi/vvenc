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
/** \file     MCTF_neon.h
    \brief    SIMD for MCTF
*/

#pragma once

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "MCTF.h"

#include <arm_neon.h>

namespace vvenc
{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCTF

static const int32_t xSzm[6] = { 0, 1, 20, 336, 5440, 87296 };

static inline void applyPlanarDeblockingCorrection_common( Pel* dstPel, const ptrdiff_t dstStride, const int32_t x1yzm,
                                                           const int32_t x2yzm, const int32_t ySum, const int32_t w,
                                                           const int32_t h, const ClpRng& clpRng,
                                                           const uint16_t motionError )
{
  const int32_t blockSize = w * h;
  const int32_t log2Width = floorLog2( w );
  const int32_t maxPelVal = clpRng.max();
  const int32_t mWeight = std::min( 512u, ( uint32_t )motionError * motionError );
  const int32_t xSum = ( blockSize * ( w - 1 ) ) >> 1;
  int32_t b0, b1, b2;
  int64_t numer, denom;

  CHECK( w != h, "Width must be same as height!" );
  CHECK( w < 4, "Width must be greater than or equal to 4!" );
  CHECK( ( w & ( w - 1 ) ) != 0, "Width can only be power of two!" );

  denom = blockSize * xSzm[log2Width]; // Plane-fit parameters, in fixed-point arithmetic.
  numer = ( int64_t )mWeight * ( ( int64_t )x1yzm * blockSize - xSum * ySum );
  b1 = int32_t( ( numer < 0 ? numer - ( denom >> 1 ) : numer + ( denom >> 1 ) ) / denom );
  b1 = b1 < INT16_MIN ? INT16_MIN : b1 > INT16_MAX ? INT16_MAX : b1;
  numer = ( int64_t )mWeight * ( ( int64_t )x2yzm * blockSize - xSum * ySum );
  b2 = int32_t( ( numer < 0 ? numer - ( denom >> 1 ) : numer + ( denom >> 1 ) ) / denom );
  b2 = b2 > INT16_MAX ? INT16_MAX : b2 < INT16_MIN ? INT16_MIN : b2;
  b0 = ( mWeight * ySum - ( b1 + b2 ) * xSum + ( blockSize >> 1 ) ) >> ( log2Width << 1 );

  if( b0 == 0 && b1 == 0 && b2 == 0 )
  {
    return;
  }

  if( w % 8 == 0 )
  {
    Pel* pDst = dstPel;
    const int32_t idx_off[4] = { 0, 1, 2, 3 };
    int32x4_t idxVec = vld1q_s32( idx_off );
    int32x4_t b1x = vmulq_s32( idxVec, vdupq_n_s32( b1 ) ); // {0*b1,1*b1,2*b1,3*b1}

    int32x4_t step4 = vdupq_n_s32( b1 << 2 );
    int32x4_t step8 = vdupq_n_s32( b1 << 3 );

    for( int32_t y = 0; y < h; y++ )
    {
      int32x4_t pc0 = vaddq_s32( vdupq_n_s32( b0 + b2 * y ), b1x ); // Plane corrector for x…x+3.
      int32x4_t pc1 = vaddq_s32( pc0, step4 );                      // Plane corrector for x+4…x+7.
      int32_t x = w;

      do
      {
        int16x8_t dst = vld1q_s16( pDst );
        int16x4_t p_lo = vqrshrn_n_s32( pc0, 9 ); // With saturation.
        int16x4_t p_hi = vqrshrn_n_s32( pc1, 9 );
        int16x8_t p = vcombine_s16( p_lo, p_hi );
        int16x8_t z = vqsubq_s16( dst, p ); // With saturation.
        z = vmaxq_s16( vdupq_n_s16( 0 ), vminq_s16( z, vdupq_n_s16( maxPelVal ) ) );
        vst1q_s16( pDst, z );

        // Increment the plane corrector by 8 * b1.
        pc0 = vaddq_s32( pc0, step8 );
        pc1 = vaddq_s32( pc1, step8 );
        pDst += 8;
        x -= 8;
      } while( x != 0 );

      pDst += dstStride - w;
    }
  }
  else
  {
    Pel* pDst = dstPel;
    const int32_t b1_idx[] = { 0, b1, b1 << 1, b1 * 3 };
    int32x4_t b1x = vld1q_s32( b1_idx );
    int32x4_t pc = vaddq_s32( b1x, vdupq_n_s32( b0 ) ); // Plane corrector.
    int32_t y = 4;

    do
    {
      // Perform deblocking by adding fitted correction plane.
      int16x4_t dst = vld1_s16( pDst );
      int16x4_t p = vqrshrn_n_s32( pc, 9 );
      int16x4_t z = vqsub_s16( dst, p );
      z = vmax_s16( vdup_n_s16( 0 ), vmin_s16( z, vdup_n_s16( maxPelVal ) ) );
      vst1_s16( pDst, z );

      // b0 + b1 * x + b2 * y for next iteration.
      pc = vaddq_s32( pc, vdupq_n_s32( b2 ) );
      pDst += dstStride;
    } while( --y != 0 );
  }
}

static inline float32x4_t div_f32x4( const float32x4_t a, const float32x4_t b )
{
#if REAL_TARGET_AARCH64
  return vdivq_f32( a, b );
#else
  float num[4], den[4], quo[4];
  vst1q_f32( num, a );
  vst1q_f32( den, b );

  quo[0] = num[0] / den[0];
  quo[1] = num[1] / den[1];
  quo[2] = num[2] / den[2];
  quo[3] = num[3] / den[3];

  return vld1q_f32( quo );
#endif
}

static inline float32x4_t mla_f32x4( const float32x4_t acc, const float32x4_t a, const float32x4_t b )
{
#if REAL_TARGET_AARCH64
  return vfmaq_f32( acc, a, b );
#else
  return vmlaq_f32( acc, a, b );
#endif
}

static inline void fastExp( float32x4_t num1, float32x4_t num2, float32x4_t denom, float32x4_t* x1, float32x4_t* x2 )
{
  // Apply fast exp with 10 iterations.
  float32x4_t x_lo = mla_f32x4( vdupq_n_f32( 1.0f ), num1, denom );
  float32x4_t x_hi = mla_f32x4( vdupq_n_f32( 1.0f ), num2, denom );

  x_lo = vmulq_f32( x_lo, x_lo );
  x_hi = vmulq_f32( x_hi, x_hi );
  x_lo = vmulq_f32( x_lo, x_lo );
  x_hi = vmulq_f32( x_hi, x_hi );
  x_lo = vmulq_f32( x_lo, x_lo );
  x_hi = vmulq_f32( x_hi, x_hi );
  x_lo = vmulq_f32( x_lo, x_lo );
  x_hi = vmulq_f32( x_hi, x_hi );
  x_lo = vmulq_f32( x_lo, x_lo );
  x_hi = vmulq_f32( x_hi, x_hi );
  x_lo = vmulq_f32( x_lo, x_lo );
  x_hi = vmulq_f32( x_hi, x_hi );
  x_lo = vmulq_f32( x_lo, x_lo );
  x_hi = vmulq_f32( x_hi, x_hi );
  x_lo = vmulq_f32( x_lo, x_lo );
  x_hi = vmulq_f32( x_hi, x_hi );
  x_lo = vmulq_f32( x_lo, x_lo );
  x_hi = vmulq_f32( x_hi, x_hi );
  x_lo = vmulq_f32( x_lo, x_lo );
  x_hi = vmulq_f32( x_hi, x_hi );

  *x1 = x_lo;
  *x2 = x_hi;
}

static inline void applyBlock_common( const CPelBuf& src, PelBuf& dst, const CompArea& blk, const ClpRng& clpRng,
                                      const Pel** correctedPics, int numRefs, const int* verror, int* vnoise,
                                      const double* refStrenghts, int minError, double weightScaling, double sigmaSq )
{
  int h = blk.height;
  const int w = blk.width;
  const int bx = blk.x;
  const int by = blk.y;

  CHECKD( w < 4, "Width must be greater than or equal to 4!" );
  CHECKD( h % 2 != 0, "Height must be multiple of 2!" );

  const ptrdiff_t srcStride = src.stride;
  const ptrdiff_t dstStride = dst.stride;

  const Pel* srcPel = src.bufAt( bx, by );
  Pel* dstPel = dst.bufAt( bx, by );

  const Pel maxSampleValue = clpRng.max();

  float vsw[2 * VVENC_MCTF_RANGE] = { 0.0f };
  float vww[2 * VVENC_MCTF_RANGE] = { 0.0f };

  int i = numRefs - 1;
  do
  {
    const int error = verror[i];
    const int noise = vnoise[i];
    float ww = 1, sw = 1;
    ww *= noise < 25 ? 1.0 : 0.6;
    sw *= noise < 25 ? 1.0 : 0.8;
    ww *= error < 50 ? 1.2 : ( error > 100 ? 0.6 : 1.0 );
    sw *= error < 50 ? 1.0 : 0.8;
    ww *= ( minError + 1.0 ) / ( error + 1.0 );

    vww[i] = ww * weightScaling * refStrenghts[i];
    vsw[i] = sw * 2 * sigmaSq;
    vsw[i] = 1.0f / ( -vsw[i] * 1024 ); // Simplify fastExp calculation by taking reciprocal and negation.
  } while( i-- != 0 );

  if( w % 8 == 0 )
  {
    int stride = 0;

    do
    {
      int x = w;
      do
      {
        int16x8_t orgVal = vld1q_s16( srcPel );

        float32x4_t newVal_lo = vcvtq_f32_s32( vmovl_s16( vget_low_s16( orgVal ) ) );
        float32x4_t newVal_hi = vcvtq_f32_s32( vmovl_s16( vget_high_s16( orgVal ) ) );

        float32x4_t temporalWeightSum_lo = vdupq_n_f32( 1.0f );
        float32x4_t temporalWeightSum_hi = vdupq_n_f32( 1.0f );

        int n = 0;
        do
        {
          int16x8_t refVal = vld1q_s16( correctedPics[n] + stride );

          float32x4_t refVal_lo = vcvtq_f32_s32( vmovl_s16( vget_low_s16( refVal ) ) );
          float32x4_t refVal_hi = vcvtq_f32_s32( vmovl_s16( vget_high_s16( refVal ) ) );

          int16x8_t diff = vsubq_s16( refVal, orgVal );

          int32x4_t diffSq_lo = vmull_s16( vget_low_s16( diff ), vget_low_s16( diff ) );
          int32x4_t diffSq_hi = vmull_s16( vget_high_s16( diff ), vget_high_s16( diff ) );

          float32x4_t num_lo = vcvtq_f32_s32( diffSq_lo );
          float32x4_t num_hi = vcvtq_f32_s32( diffSq_hi );
          float32x4_t recip_denom = vdupq_n_f32( vsw[n] ); // Negation already applied to vsw[n].

          float32x4_t x_lo, x_hi;
          fastExp( num_lo, num_hi, recip_denom, &x_lo, &x_hi );

          float32x4_t vww_val = vdupq_n_f32( vww[n] );
          float32x4_t weight_lo = vmulq_f32( x_lo, vww_val );
          float32x4_t weight_hi = vmulq_f32( x_hi, vww_val );

          newVal_lo = mla_f32x4( newVal_lo, weight_lo, refVal_lo );
          newVal_hi = mla_f32x4( newVal_hi, weight_hi, refVal_hi );

          temporalWeightSum_lo = vaddq_f32( temporalWeightSum_lo, weight_lo );
          temporalWeightSum_hi = vaddq_f32( temporalWeightSum_hi, weight_hi );
        } while( ++n != numRefs );

        newVal_lo = div_f32x4( newVal_lo, temporalWeightSum_lo );
        newVal_hi = div_f32x4( newVal_hi, temporalWeightSum_hi );

        uint16x4_t out_lo = vqmovn_u32( vcvtq_u32_f32( vaddq_f32( newVal_lo, vdupq_n_f32( 0.5f ) ) ) );
        uint16x4_t out_hi = vqmovn_u32( vcvtq_u32_f32( vaddq_f32( newVal_hi, vdupq_n_f32( 0.5f ) ) ) );

        uint16x8_t sample = vcombine_u16( out_lo, out_hi );
        sample = vminq_u16( sample, vdupq_n_u16( maxSampleValue ) );

        vst1q_s16( dstPel, vreinterpretq_s16_u16( sample ) );

        srcPel += 8;
        dstPel += 8;
        stride += 8;
        x -= 8;
      } while( x != 0 );

      srcPel += srcStride - w;
      dstPel += dstStride - w;
    } while( --h != 0 );
  }
  else
  {
    CHECKD( w != 4, "Width must be equal to 4!" );

    int stride = 0;
    h >>= 1;
    do
    {
      int16x4_t orgVal0 = vld1_s16( srcPel );
      int16x4_t orgVal1 = vld1_s16( srcPel + srcStride );

      float32x4_t newVal0 = vcvtq_f32_s32( vmovl_s16( orgVal0 ) );
      float32x4_t newVal1 = vcvtq_f32_s32( vmovl_s16( orgVal1 ) );

      float32x4_t temporalWeightSum0 = vdupq_n_f32( 1.0f );
      float32x4_t temporalWeightSum1 = vdupq_n_f32( 1.0f );

      int n = 0;
      do
      {
        int16x4_t refVal0 = vld1_s16( correctedPics[n] + stride );
        int16x4_t refVal1 = vld1_s16( correctedPics[n] + stride + w );

        int16x4_t diff0 = vsub_s16( refVal0, orgVal0 );
        int16x4_t diff1 = vsub_s16( refVal1, orgVal1 );

        int32x4_t diffSq0 = vmull_s16( diff0, diff0 );
        int32x4_t diffSq1 = vmull_s16( diff1, diff1 );

        float32x4_t recip_denom = vdupq_n_f32( vsw[n] ); // Negation already applied to vsw[n].
        float32x4_t num_lo = vcvtq_f32_s32( diffSq0 );
        float32x4_t num_hi = vcvtq_f32_s32( diffSq1 );

        float32x4_t x0, x1;
        fastExp( num_lo, num_hi, recip_denom, &x0, &x1 );

        float32x4_t vww_val = vdupq_n_f32( vww[n] );
        float32x4_t weight0 = vmulq_f32( x0, vww_val );
        float32x4_t weight1 = vmulq_f32( x1, vww_val );

        newVal0 = mla_f32x4( newVal0, weight0, vcvtq_f32_s32( vmovl_s16( refVal0 ) ) );
        newVal1 = mla_f32x4( newVal1, weight1, vcvtq_f32_s32( vmovl_s16( refVal1 ) ) );

        temporalWeightSum0 = vaddq_f32( temporalWeightSum0, weight0 );
        temporalWeightSum1 = vaddq_f32( temporalWeightSum1, weight1 );
      } while( ++n != numRefs );

      newVal0 = div_f32x4( newVal0, temporalWeightSum0 );
      newVal1 = div_f32x4( newVal1, temporalWeightSum1 );

      uint16x4_t sample0 = vqmovn_u32( vcvtq_u32_f32( vaddq_f32( newVal0, vdupq_n_f32( 0.5f ) ) ) );
      uint16x4_t sample1 = vqmovn_u32( vcvtq_u32_f32( vaddq_f32( newVal1, vdupq_n_f32( 0.5f ) ) ) );

      sample0 = vmin_u16( sample0, vdup_n_u16( maxSampleValue ) );
      sample1 = vmin_u16( sample1, vdup_n_u16( maxSampleValue ) );

      vst1_s16( dstPel + 0, vreinterpret_s16_u16( sample0 ) );
      vst1_s16( dstPel + dstStride, vreinterpret_s16_u16( sample1 ) );

      srcPel += 2 * srcStride;
      dstPel += 2 * dstStride;
      stride += 2 * w;
    } while( --h != 0 );
  }
}
#endif
} // namespace vvenc

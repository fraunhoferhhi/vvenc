/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCTF

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

static const int32_t xSzm[6] = { 0, 1, 20, 336, 5440, 87296 };

void applyPlanarCorrection_neon( const Pel* refPel, const ptrdiff_t refStride, Pel* dstPel, const ptrdiff_t dstStride,
                                 const int32_t w, const int32_t h, const ClpRng& clpRng, const uint16_t motionError )
{
  const int32_t blockSize = w * h;
  const int32_t log2Width = floorLog2( w );
  const int32_t maxPelVal = clpRng.max();
  const int32_t mWeight = std::min( 512u, ( uint32_t )motionError * motionError );
  const int32_t xSum = ( blockSize * ( w - 1 ) ) >> 1;
  int32_t x1yzm = 0, x2yzm = 0, ySum = 0;
  int32_t b0, b1, b2;
  int64_t numer, denom;

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
  else if( w == 4 )
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

template<>
void MCTF::_initMCTF_ARM<NEON>()
{
  m_motionErrorLumaFrac8[1] = motionErrorLumaFrac_loRes_neon;
  m_motionErrorLumaInt8     = motionErrorLumaInt_neon;
  m_applyPlanarCorrection   = applyPlanarCorrection_neon;
}

} // namespace vvenc
#endif
//! \}

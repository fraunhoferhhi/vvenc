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

template<>
void MCTF::_initMCTF_ARM<NEON>()
{
  m_motionErrorLumaFrac8[1] = motionErrorLumaFrac_loRes_neon;
  m_motionErrorLumaInt8     = motionErrorLumaInt_neon;
}

} // namespace vvenc
#endif
//! \}

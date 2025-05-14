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
 * \file IntraPred_neon.cpp
 * \brief Neon implementation of IntraPrediction for Arm.
 */
//  ====================================================================================================================
//  Includes
//  ====================================================================================================================

#include "../CommonDefARM.h"
#include "CommonLib/InterpolationFilter.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/Rom.h"
#include "CommonLib/Unit.h"

#if ENABLE_SIMD_OPT_INTRAPRED && defined( TARGET_SIMD_ARM )

#include <arm_neon.h>

namespace vvenc
{

void IntraPredAngleLuma_neon( Pel* pDstBuf, const ptrdiff_t dstStride, Pel* refMain, int width, int height,
                              int deltaPos, int intraPredAngle, const TFilterCoeff* /* ff_unused */,
                              const bool useCubicFilter, const ClpRng& clpRng )
{
  CHECK( height < 1, "Invalid height. Must be >= 1" );
  CHECK( width < 4, "Invalid width. Must be >= 4" );
  CHECK( ( width & 7 ) && ( width != 4 ), "Invalid width. Must be 4 or a multiple of 8" );
  CHECK( clpRng.bd > 10, "Invalid bit-depth. Must be <= 10" );

  static constexpr int shift = 6;
  static constexpr int round = 1 << ( shift - 1 );

  if( useCubicFilter )
  {
    do
    {
      const int deltaInt = deltaPos >> 5;
      const int deltaFract = deltaPos & ( 32 - 1 );

      // Chroma Filter max value is 64.
      const TFilterCoeff* f = InterpolationFilter::getChromaFilterTable( deltaFract );
      const int16x4_t filter = vld1_s16( f );

      const Pel* pRef = refMain + deltaInt;
      for( int w = 0; w <= width - 8; w += 8 )
      {
        int16x8_t p0 = vld1q_s16( pRef + 0 );
        int16x8_t p1 = vld1q_s16( pRef + 1 );
        int16x8_t p2 = vld1q_s16( pRef + 2 );
        int16x8_t p3 = vld1q_s16( pRef + 3 );

        // Max accumulate for filter[0,3] is (-6 + -4) * 10-bit so use int16_t.
        int16x8_t sum = vmulq_lane_s16( p0, filter, 0 );
        sum = vmlaq_lane_s16( sum, p3, filter, 3 );
        // Max accumulate for filter[1,2] is (46 + 28) * 10-bit so use int32_t.
        int32x4_t sum_lo = vmull_lane_s16( vget_low_s16( p1 ), filter, 1 );
        sum_lo = vmlal_lane_s16( sum_lo, vget_low_s16( p2 ), filter, 2 );
        sum_lo = vaddw_s16( sum_lo, vget_low_s16( sum ) );
        int32x4_t sum_hi = vmull_lane_s16( vget_high_s16( p1 ), filter, 1 );
        sum_hi = vmlal_lane_s16( sum_hi, vget_high_s16( p2 ), filter, 2 );
        sum_hi = vaddw_s16( sum_hi, vget_high_s16( sum ) );

        // Only cubic filter has negative coefficients and requires clipping.
        uint16x4_t d_lo = vqrshrun_n_s32( sum_lo, shift );
        uint16x4_t d_hi = vqrshrun_n_s32( sum_hi, shift );
        uint16x8_t d = vminq_u16( vcombine_u16( d_lo, d_hi ), vdupq_n_u16( clpRng.max() ) );
        vst1q_s16( pDstBuf + w, vreinterpretq_s16_u16( d ) );

        pRef += 8;
      }
      if( width == 4 )
      {
        int16x4_t p0 = vld1_s16( pRef + 0 );
        int16x4_t p1 = vld1_s16( pRef + 1 );
        int16x4_t p2 = vld1_s16( pRef + 2 );
        int16x4_t p3 = vld1_s16( pRef + 3 );

        // Max accumulate is (46 + 28) * 10-bit which will not fit into int16_t so use int32_t.
        int32x4_t sum = vmull_lane_s16( p0, filter, 0 );
        sum = vmlal_lane_s16( sum, p1, filter, 1 );
        sum = vmlal_lane_s16( sum, p2, filter, 2 );
        sum = vmlal_lane_s16( sum, p3, filter, 3 );

        // Only cubic filter has negative coefficients and requires clipping.
        uint16x4_t d = vqrshrun_n_s32( sum, shift );
        d = vmin_u16( d, vdup_n_u16( clpRng.max() ) );
        vst1_s16( pDstBuf, vreinterpret_s16_u16( d ) );
      }

      pDstBuf += dstStride;
      deltaPos += intraPredAngle;
    } while( --height != 0 );
  }
  else // ( !useCubicFilter )
  {
    do
    {
      const int deltaInt = deltaPos >> 5;
      const int deltaFract = deltaPos & ( 32 - 1 );

      // Smoothing Filter max value is 31. Total adds up to 64.
      static const uint16_t intraSmoothingFilter[4] = { 16, 32, 16, 0 };
      const uint16x4_t vIntraSmoothingFilter = vld1_u16( intraSmoothingFilter );
      const uint16x4_t vDeltaFract = vdup_n_u16( deltaFract >> 1 );
      const uint16x4_t filter01 = vsub_u16( vIntraSmoothingFilter, vDeltaFract );
      const uint16x4_t filter23 = vadd_u16( vIntraSmoothingFilter, vDeltaFract );

      const Pel* pRef = refMain + deltaInt;
      for( int w = 0; w <= width - 8; w += 8 )
      {
        uint16x8_t p0 = vreinterpretq_u16_s16( vld1q_s16( pRef + 0 ) );
        uint16x8_t p1 = vreinterpretq_u16_s16( vld1q_s16( pRef + 1 ) );
        uint16x8_t p2 = vreinterpretq_u16_s16( vld1q_s16( pRef + 2 ) );
        uint16x8_t p3 = vreinterpretq_u16_s16( vld1q_s16( pRef + 3 ) );

        // Max accumulate is unsigned 10-bit << 6, so use uint16_t.
        uint16x8_t sum = vmlaq_lane_u16( vdupq_n_u16( round ), p0, filter01, 0 );
        sum = vmlaq_lane_u16( sum, p1, filter01, 1 );
        sum = vmlaq_lane_u16( sum, p2, filter23, 2 );
        sum = vmlaq_lane_u16( sum, p3, filter23, 3 );

        uint16x8_t d = vshrq_n_u16( sum, shift );
        vst1q_s16( pDstBuf + w, vreinterpretq_s16_u16( d ) );

        pRef += 8;
      }
      if( width == 4 )
      {
        uint16x4_t p0 = vreinterpret_u16_s16( vld1_s16( pRef + 0 ) );
        uint16x4_t p1 = vreinterpret_u16_s16( vld1_s16( pRef + 1 ) );
        uint16x4_t p2 = vreinterpret_u16_s16( vld1_s16( pRef + 2 ) );
        uint16x4_t p3 = vreinterpret_u16_s16( vld1_s16( pRef + 3 ) );

        // Max accumulate is unsigned 10-bit << 6, so use uint16_t.
        uint16x4_t sum = vmla_lane_u16( vdup_n_u16( round ), p0, filter01, 0 );
        sum = vmla_lane_u16( sum, p1, filter01, 1 );
        sum = vmla_lane_u16( sum, p2, filter23, 2 );
        sum = vmla_lane_u16( sum, p3, filter23, 3 );

        uint16x4_t d = vshr_n_u16( sum, shift );
        vst1_s16( pDstBuf, vreinterpret_s16_u16( d ) );
      }

      pDstBuf += dstStride;
      deltaPos += intraPredAngle;
    } while( --height != 0 );
  }
}

template<>
void IntraPrediction::_initIntraPredictionARM<NEON>()
{
  IntraPredAngleLuma = IntraPredAngleLuma_neon;
}

} // namespace vvenc

#endif // ENABLE_SIMD_OPT_INTRAPRED && defined( TARGET_SIMD_ARM )

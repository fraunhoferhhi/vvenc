/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     InterpolationFilter_neon.h
    \brief    SIMD for InterpolationFilter
*/

#pragma once

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/InterpolationFilter.h"
#include "neon/sum_neon.h"

#include <arm_neon.h>

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCIF

namespace vvenc
{

template<bool isLast>
static inline int16x4_t pack_sum_s32_to_s16x4( int32x4_t vsuma, int16x4_t vibdimax )
{
  if( isLast )
  {
    // Narrow to u16 with saturation (clamp negatives to 0), then clamp at vibdimax.
    uint16x4_t usum = vqmovun_s32( vsuma );
    return vmin_s16( vibdimax, vreinterpret_s16_u16( usum ) );
  }
  else
  {
    // Narrow to s16 with saturation.
    return vqmovn_s32( vsuma );
  }
}

template<bool isLast>
static inline int16x8_t pack_sum_s32_to_s16x8( int32x4_t vsuma, int32x4_t vsumb, int16x8_t vibdimax )
{
  if( isLast )
  {
    // Narrow to u16 with saturation (clamp negatives to 0), then clamp at vibdimax.
    uint16x8_t usum = vcombine_u16( vqmovun_s32( vsuma ), vqmovun_s32( vsumb ) );
    return vminq_s16( vibdimax, vreinterpretq_s16_u16( usum ) );
  }
  else
  {
    // Narrow to s16 with saturation.
    return vcombine_s16( vqmovn_s32( vsuma ), vqmovn_s32( vsumb ) );
  }
}

static inline int32x4_t filter_vert_4x1_N6_neon( int16x4_t const vsrc[6], int16x8_t cv, int32x4_t voffset2 )
{
  // For 6-tap, the 0th and 7th cv coefficients are zeros so remove them.
  int32x4_t vsum = vmlal_lane_s16( voffset2, vsrc[0], vget_low_s16( cv ), 1 );
  vsum = vmlal_lane_s16( vsum, vsrc[1], vget_low_s16( cv ), 2 );
  vsum = vmlal_lane_s16( vsum, vsrc[2], vget_low_s16( cv ), 3 );
  vsum = vmlal_lane_s16( vsum, vsrc[3], vget_high_s16( cv ), 0 );
  vsum = vmlal_lane_s16( vsum, vsrc[4], vget_high_s16( cv ), 1 );
  vsum = vmlal_lane_s16( vsum, vsrc[5], vget_high_s16( cv ), 2 );
  return vsum;
}

static inline int32x4x2_t filter_vert_8x1_N8_neon( int16x8_t const vsrc[8], int16x8_t cv, int32x4_t voffset2 )
{
  int32x4x2_t vsum;
  vsum.val[0] = vmlal_lane_s16( voffset2, vget_low_s16( vsrc[0] ), vget_low_s16( cv ), 0 );
  vsum.val[1] = vmlal_lane_s16( voffset2, vget_high_s16( vsrc[0] ), vget_low_s16( cv ), 0 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[1] ), vget_low_s16( cv ), 1 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[1] ), vget_low_s16( cv ), 1 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[2] ), vget_low_s16( cv ), 2 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[2] ), vget_low_s16( cv ), 2 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[3] ), vget_low_s16( cv ), 3 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[3] ), vget_low_s16( cv ), 3 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[4] ), vget_high_s16( cv ), 0 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[4] ), vget_high_s16( cv ), 0 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[5] ), vget_high_s16( cv ), 1 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[5] ), vget_high_s16( cv ), 1 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[6] ), vget_high_s16( cv ), 2 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[6] ), vget_high_s16( cv ), 2 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[7] ), vget_high_s16( cv ), 3 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[7] ), vget_high_s16( cv ), 3 );
  return vsum;
}

static inline int32x4x4_t filter_vert_16x1_N8_neon( int16x8x2_t const vsrc[8], int16x8_t cv, int32x4_t voffset2 )
{
  int16x8_t vsrc0[8] = { vsrc[0].val[0], vsrc[1].val[0], vsrc[2].val[0], vsrc[3].val[0],
                         vsrc[4].val[0], vsrc[5].val[0], vsrc[6].val[0], vsrc[7].val[0] };
  int16x8_t vsrc1[8] = { vsrc[0].val[1], vsrc[1].val[1], vsrc[2].val[1], vsrc[3].val[1],
                         vsrc[4].val[1], vsrc[5].val[1], vsrc[6].val[1], vsrc[7].val[1] };

  int32x4x2_t vsum0 = filter_vert_8x1_N8_neon( vsrc0, cv, voffset2 );
  int32x4x2_t vsum1 = filter_vert_8x1_N8_neon( vsrc1, cv, voffset2 );

  int32x4x4_t vsum;
  vsum.val[0] = vsum0.val[0];
  vsum.val[1] = vsum0.val[1];
  vsum.val[2] = vsum1.val[0];
  vsum.val[3] = vsum1.val[1];
  return vsum;
}

static inline int32x4_t filter_vert_4x1_N4_neon( int16x4_t const vsrc[4], int16x4_t cv, int32x4_t voffset2 )
{
  int32x4_t vsum = vmlal_lane_s16( voffset2, vsrc[0], cv, 0 );
  vsum = vmlal_lane_s16( vsum, vsrc[1], cv, 1 );
  vsum = vmlal_lane_s16( vsum, vsrc[2], cv, 2 );
  vsum = vmlal_lane_s16( vsum, vsrc[3], cv, 3 );
  return vsum;
}

static inline int32x4x2_t filter_vert_8x1_N4_neon( int16x8_t const vsrc[4], int16x4_t cv, int32x4_t voffset2 )
{
  int32x4x2_t vsum;
  vsum.val[0] = vmlal_lane_s16( voffset2, vget_low_s16( vsrc[0] ), cv, 0 );
  vsum.val[1] = vmlal_lane_s16( voffset2, vget_high_s16( vsrc[0] ), cv, 0 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[1] ), cv, 1 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[1] ), cv, 1 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[2] ), cv, 2 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[2] ), cv, 2 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[3] ), cv, 3 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[3] ), cv, 3 );
  return vsum;
}

static inline int32x4x4_t filter_vert_16x1_N4_neon( int16x8x2_t const vsrc[4], int16x4_t cv, int32x4_t voffset2 )
{
  int32x4x4_t vsum;
  vsum.val[0] = vmlal_lane_s16( voffset2, vget_low_s16( vsrc[0].val[0] ), cv, 0 );
  vsum.val[1] = vmlal_lane_s16( voffset2, vget_high_s16( vsrc[0].val[0] ), cv, 0 );
  vsum.val[2] = vmlal_lane_s16( voffset2, vget_low_s16( vsrc[0].val[1] ), cv, 0 );
  vsum.val[3] = vmlal_lane_s16( voffset2, vget_high_s16( vsrc[0].val[1] ), cv, 0 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[1].val[0] ), cv, 1 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[1].val[0] ), cv, 1 );
  vsum.val[2] = vmlal_lane_s16( vsum.val[2], vget_low_s16( vsrc[1].val[1] ), cv, 1 );
  vsum.val[3] = vmlal_lane_s16( vsum.val[3], vget_high_s16( vsrc[1].val[1] ), cv, 1 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[2].val[0] ), cv, 2 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[2].val[0] ), cv, 2 );
  vsum.val[2] = vmlal_lane_s16( vsum.val[2], vget_low_s16( vsrc[2].val[1] ), cv, 2 );
  vsum.val[3] = vmlal_lane_s16( vsum.val[3], vget_high_s16( vsrc[2].val[1] ), cv, 2 );
  vsum.val[0] = vmlal_lane_s16( vsum.val[0], vget_low_s16( vsrc[3].val[0] ), cv, 3 );
  vsum.val[1] = vmlal_lane_s16( vsum.val[1], vget_high_s16( vsrc[3].val[0] ), cv, 3 );
  vsum.val[2] = vmlal_lane_s16( vsum.val[2], vget_low_s16( vsrc[3].val[1] ), cv, 3 );
  vsum.val[3] = vmlal_lane_s16( vsum.val[3], vget_high_s16( vsrc[3].val[1] ), cv, 3 );
  return vsum;
}

// The horizontal stage of the 6-tap 4x4 filter for 4 outputs, tap by tap: offset + sum of ch[t] * src[x + t] over the
// six non-zero taps t = 1 .. 6 (taps 0 and 7 of the 6-tap table are zero), shifted and narrowed to 16 bits.
static inline int16x4_t filter_horiz_4x1_N6_neon( Pel const* src, int16x8_t ch, int32x4_t voffset1, int32x4_t invshift1st )
{
  const int16x4_t vsrc1 = vld1_s16( src + 1 );
  const int16x4_t vsrc2 = vld1_s16( src + 2 );
  const int16x4_t vsrc3 = vld1_s16( src + 3 );
  const int16x4_t vsrc4 = vld1_s16( src + 4 );
  const int16x4_t vsrc5 = vld1_s16( src + 5 );
  const int16x4_t vsrc6 = vld1_s16( src + 6 );

  int32x4_t vsum = vmlal_lane_s16( voffset1, vsrc1, vget_low_s16( ch ), 1 );
  vsum           = vmlal_lane_s16( vsum, vsrc2, vget_low_s16( ch ), 2 );
  vsum           = vmlal_lane_s16( vsum, vsrc3, vget_low_s16( ch ), 3 );
  vsum           = vmlal_lane_s16( vsum, vsrc4, vget_high_s16( ch ), 0 );
  vsum           = vmlal_lane_s16( vsum, vsrc5, vget_high_s16( ch ), 1 );
  vsum           = vmlal_lane_s16( vsum, vsrc6, vget_high_s16( ch ), 2 );

  vsum = vshlq_s32( vsum, invshift1st );
  return vqmovn_s32( vsum );
}

template<bool isLast>
static ALWAYS_INLINE void simdFilter4x4_N6_neon( const ClpRng& clpRng, Pel const* src, int srcStride, Pel* dst, int dstStride, int width,
                                                 int height, TFilterCoeff const* coeffH, TFilterCoeff const* coeffV )
{
  CHECKD( width != 4, "Width must be 4" );
  CHECKD( height != 4, "Height must be 4" );
  CHECKD( IF_INTERNAL_PREC - clpRng.bd < 2, "Bit depth headroom must be at least 2" );

  src -= 3 + 2 * srcStride;

  const int headRoom = IF_INTERNAL_PREC - clpRng.bd;
  const int shift1st = IF_FILTER_PREC - headRoom;
  const int shift2nd = IF_FILTER_PREC + headRoom;

  const int offset1st = -IF_INTERNAL_OFFS * ( 1 << shift1st );
  int offset2nd;
  if( isLast )
  {
    offset2nd = ( 1 << ( shift2nd - 1 ) ) + ( IF_INTERNAL_OFFS << IF_FILTER_PREC );
  }
  else
  {
    offset2nd = 0;
  }
  const int32x4_t voffset1 = vdupq_n_s32( offset1st );
  const int32x4_t voffset2 = vdupq_n_s32( offset2nd );
  const int16x4_t vibdimax = vdup_n_s16( clpRng.max() );

  int16x8_t ch = vld1q_s16( coeffH );
  int16x8_t cv = vld1q_s16( coeffV );

  int32x4_t invshift1st = vdupq_n_s32( -shift1st );
  int32x4_t invshift2nd = vdupq_n_s32( -shift2nd );

  int16x4_t vsrcv[9];
  vsrcv[0] = filter_horiz_4x1_N6_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[1] = filter_horiz_4x1_N6_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[2] = filter_horiz_4x1_N6_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[3] = filter_horiz_4x1_N6_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[4] = filter_horiz_4x1_N6_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[5] = filter_horiz_4x1_N6_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[6] = filter_horiz_4x1_N6_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[7] = filter_horiz_4x1_N6_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[8] = filter_horiz_4x1_N6_neon( src, ch, voffset1, invshift1st );

  int h = 0;
  do
  {
    int32x4_t vsum0 = filter_vert_4x1_N6_neon( vsrcv + h, cv, voffset2 );

    int16x4_t vsum01;
    if( isLast ) // clip
    {
      uint16x4_t usum01 = vqmovun_s32( vshlq_s32( vsum0, invshift2nd ) );
      vsum01 = vmin_s16( vibdimax, vreinterpret_s16_u16( usum01 ) );
    }
    else
    {
      vsum01 = vqshrn_n_s32( vsum0, IF_FILTER_PREC );
    }

    vst1_s16( dst, vsum01 );

    dst += dstStride;
  } while( ++h != 4 );
}

} // namespace vvenc

#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCIF

//! \}
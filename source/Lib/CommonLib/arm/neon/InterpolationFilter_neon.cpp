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
 * \file InterpolationFilter_neon.cpp
 * \brief Neon implementation of InterpolationFilter for AArch64.
 */
//  ====================================================================================================================
//  Includes
//  ====================================================================================================================

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "../InterpolationFilter.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCIF

namespace vvenc
{

static void simdInterpolateN2_2D_neon( const ClpRng& clpRng, const Pel* src, const int srcStride, Pel* dst, const int dstStride, int width, int height, TFilterCoeff const *ch, TFilterCoeff const *cv )
{
  const int shift1st  = IF_FILTER_PREC_BILINEAR - ( IF_INTERNAL_PREC_BILINEAR - clpRng.bd );
  const int offset1st = 1 << ( shift1st - 1 );

  const int shift2nd  = 4;
  const int offset2nd = 1 << ( shift2nd - 1 );

  int16x8_t mmOffset1 = vdupq_n_s16( offset1st );
  int16x8_t mmOffset2 = vdupq_n_s16( offset2nd );
  int16x8_t mmCoeffH  = vdupq_n_s16( ch[ 1 ] );
  int16x8_t mmCoeffV  = vdupq_n_s16( cv[ 1 ] );

  int16x8_t mmLastH[ 16 ];

  int16x8_t mmLast4H;

  // workaround for over-sensitive compilers
  mmLastH[ 0 ] = vdupq_n_s16( 0 );

  int16x8_t shift1inv = vdupq_n_s16( -shift1st );
  int16x8_t shift2inv = vdupq_n_s16( -shift2nd );

  for( int row = -1; row < height; row++ )
  {
    int16x8_t mmPix  = vld1q_s16( src );
    int16x8_t mmPix1 = vld1q_s16( src + 1 );

    int16x8_t mmFiltered = vmlaq_n_s16( mmOffset1, mmPix, 16 );

    mmFiltered = vmlaq_s16( mmFiltered, vsubq_s16( mmPix1, mmPix ), mmCoeffH );
    mmFiltered = vshlq_s16( mmFiltered, shift1inv );

    if( row >= 0 )
    {
      int16x8_t mmFiltered2 = vmlaq_n_s16( mmOffset2, mmLast4H, 16 );
      mmFiltered2           = vmlaq_s16( mmFiltered2, vsubq_s16( mmFiltered, mmLast4H ), mmCoeffV );
      mmFiltered2           = vshlq_s16( mmFiltered2, shift2inv );

      vst1q_lane_s64( (int64_t*) dst, (int64x2_t) mmFiltered2, 0 );
    }

    mmLast4H = mmFiltered;

    for( int x = 4; x < width; x += 8 )
    {
      int16x8_t mmPix  = vld1q_s16( src + x );
      int16x8_t mmPix1 = vld1q_s16( src + x + 1 );

      int16x8_t mmFiltered = vmlaq_n_s16( mmOffset1, mmPix, 16 );
      mmFiltered           = vmlaq_s16( mmFiltered, vsubq_s16( mmPix1, mmPix ), mmCoeffH );
      mmFiltered           = vshlq_s16( mmFiltered, shift1inv );

      int       idx   = x >> 3;
      int16x8_t mLast = mmLastH[ idx ];
      mmLastH[ idx ]  = mmFiltered;

      if( row >= 0 )
      {
        int16x8_t mmFiltered2 = vmlaq_n_s16( mmOffset2, mLast, 16 );
        mmFiltered2           = vmlaq_s16( mmFiltered2, vsubq_s16( mmFiltered, mLast ), mmCoeffV );
        mmFiltered2           = vshlq_s16( mmFiltered2, shift2inv );

        vst1q_s16( ( dst + x ), mmFiltered2 );
      }
    }
    if( row >= 0 )
      dst += dstStride;

    src += srcStride;
  }
}

static int16x8_t simdFilter16xX_N8_half( Pel const* src, int16x8_t ch, int32x4_t voffset1, int32x4_t invshift1st )
{
  int16x8_t vsrca00 = vld1q_s16( src + 0 );
  int16x8_t vsrca01 = vld1q_s16( src + 1 );
  int16x8_t vsrca10 = vld1q_s16( src + 2 );
  int16x8_t vsrca11 = vld1q_s16( src + 3 );
  int16x8_t vsrcb00 = vld1q_s16( src + 4 );
  int16x8_t vsrcb01 = vld1q_s16( src + 5 );
  int16x8_t vsrcb10 = vld1q_s16( src + 6 );
  int16x8_t vsrcb11 = vld1q_s16( src + 7 );

  int32x4_t a0 = vmull_s16( vget_low_s16( vsrca00 ), vget_low_s16( ch ) );
  int32x4_t a1 = vmull_s16( vget_low_s16( vsrca01 ), vget_low_s16( ch ) );
  int32x4_t a2 = vmull_s16( vget_low_s16( vsrca10 ), vget_low_s16( ch ) );
  int32x4_t a3 = vmull_s16( vget_low_s16( vsrca11 ), vget_low_s16( ch ) );

  int32x4_t b0 = vmull_s16( vget_low_s16( vsrcb00 ), vget_low_s16( ch ) );
  int32x4_t b1 = vmull_s16( vget_low_s16( vsrcb01 ), vget_low_s16( ch ) );
  int32x4_t b2 = vmull_s16( vget_low_s16( vsrcb10 ), vget_low_s16( ch ) );
  int32x4_t b3 = vmull_s16( vget_low_s16( vsrcb11 ), vget_low_s16( ch ) );

  a0 = vmlal_s16( a0, vget_high_s16( vsrca00 ), vget_high_s16( ch ) );
  a1 = vmlal_s16( a1, vget_high_s16( vsrca01 ), vget_high_s16( ch ) );
  a2 = vmlal_s16( a2, vget_high_s16( vsrca10 ), vget_high_s16( ch ) );
  a3 = vmlal_s16( a3, vget_high_s16( vsrca11 ), vget_high_s16( ch ) );

  b0 = vmlal_s16( b0, vget_high_s16( vsrcb00 ), vget_high_s16( ch ) );
  b1 = vmlal_s16( b1, vget_high_s16( vsrcb01 ), vget_high_s16( ch ) );
  b2 = vmlal_s16( b2, vget_high_s16( vsrcb10 ), vget_high_s16( ch ) );
  b3 = vmlal_s16( b3, vget_high_s16( vsrcb11 ), vget_high_s16( ch ) );

  int32x4_t vsuma = vpaddq_s32( vpaddq_s32( a0, a1 ), vpaddq_s32( a2, a3 ) );
  int32x4_t vsumb = vpaddq_s32( vpaddq_s32( b0, b1 ), vpaddq_s32( b2, b3 ) );

  vsuma = vaddq_s32( vsuma, voffset1 );
  vsumb = vaddq_s32( vsumb, voffset1 );

  vsuma = vshlq_s32( vsuma, invshift1st );
  vsumb = vshlq_s32( vsumb, invshift1st );

  return vcombine_s16( vqmovn_s32( vsuma ), vqmovn_s32( vsumb ) );
}

static int16x8x2_t simdFilter16xX_N8_step( Pel const* src, int16x8_t ch, int32x4_t voffset1, int32x4_t invshift1st )
{
  int16x8_t a = simdFilter16xX_N8_half( src + 0, ch, voffset1, invshift1st );
  int16x8_t b = simdFilter16xX_N8_half( src + 8, ch, voffset1, invshift1st );
  return ( int16x8x2_t ){ a, b };
}

template<bool isLast>
static void simdFilter16xX_N8_neon( const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV )
{
  OFFSET( src, srcStride, -3, -3 );

  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20
  const int headRoom = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
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

  const int16x8_t vibdimin = vdupq_n_s16( clpRng.min() );
  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );

  int16x8_t ch = vld1q_s16( coeffH );
  int16x8_t cv = vld1q_s16( coeffV );

  int32x4_t invshift1st = vdupq_n_s32( -shift1st );
  int32x4_t invshift2nd = vdupq_n_s32( -shift2nd );

  int16x8x2_t vsrcv0 = simdFilter16xX_N8_step( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8x2_t vsrcv1 = simdFilter16xX_N8_step( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8x2_t vsrcv2 = simdFilter16xX_N8_step( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8x2_t vsrcv3 = simdFilter16xX_N8_step( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8x2_t vsrcv4 = simdFilter16xX_N8_step( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8x2_t vsrcv5 = simdFilter16xX_N8_step( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8x2_t vsrcv6 = simdFilter16xX_N8_step( src, ch, voffset1, invshift1st );
  src += srcStride;

  do
  {
    int16x8x2_t vsrcv7 = simdFilter16xX_N8_step( src, ch, voffset1, invshift1st );
    src += srcStride;

    int32x4_t vsum0 = vdupq_n_s32( offset2nd );
    int32x4_t vsum1 = vdupq_n_s32( offset2nd );
    int32x4_t vsum2 = vdupq_n_s32( offset2nd );
    int32x4_t vsum3 = vdupq_n_s32( offset2nd );

    vsum0 = vmlal_laneq_s16( vsum0, vget_low_s16( vsrcv0.val[ 0 ] ), cv, 0 );
    vsum1 = vmlal_laneq_s16( vsum1, vget_high_s16( vsrcv0.val[ 0 ] ), cv, 0 );
    vsum2 = vmlal_laneq_s16( vsum2, vget_low_s16( vsrcv0.val[ 1 ] ), cv, 0 );
    vsum3 = vmlal_laneq_s16( vsum3, vget_high_s16( vsrcv0.val[ 1 ] ), cv, 0 );

    vsum0 = vmlal_laneq_s16( vsum0, vget_low_s16( vsrcv1.val[ 0 ] ), cv, 1 );
    vsum1 = vmlal_laneq_s16( vsum1, vget_high_s16( vsrcv1.val[ 0 ] ), cv, 1 );
    vsum2 = vmlal_laneq_s16( vsum2, vget_low_s16( vsrcv1.val[ 1 ] ), cv, 1 );
    vsum3 = vmlal_laneq_s16( vsum3, vget_high_s16( vsrcv1.val[ 1 ] ), cv, 1 );

    vsum0 = vmlal_laneq_s16( vsum0, vget_low_s16( vsrcv2.val[ 0 ] ), cv, 2 );
    vsum1 = vmlal_laneq_s16( vsum1, vget_high_s16( vsrcv2.val[ 0 ] ), cv, 2 );
    vsum2 = vmlal_laneq_s16( vsum2, vget_low_s16( vsrcv2.val[ 1 ] ), cv, 2 );
    vsum3 = vmlal_laneq_s16( vsum3, vget_high_s16( vsrcv2.val[ 1 ] ), cv, 2 );

    vsum0 = vmlal_laneq_s16( vsum0, vget_low_s16( vsrcv3.val[ 0 ] ), cv, 3 );
    vsum1 = vmlal_laneq_s16( vsum1, vget_high_s16( vsrcv3.val[ 0 ] ), cv, 3 );
    vsum2 = vmlal_laneq_s16( vsum2, vget_low_s16( vsrcv3.val[ 1 ] ), cv, 3 );
    vsum3 = vmlal_laneq_s16( vsum3, vget_high_s16( vsrcv3.val[ 1 ] ), cv, 3 );

    vsum0 = vmlal_laneq_s16( vsum0, vget_low_s16( vsrcv4.val[ 0 ] ), cv, 4 );
    vsum1 = vmlal_laneq_s16( vsum1, vget_high_s16( vsrcv4.val[ 0 ] ), cv, 4 );
    vsum2 = vmlal_laneq_s16( vsum2, vget_low_s16( vsrcv4.val[ 1 ] ), cv, 4 );
    vsum3 = vmlal_laneq_s16( vsum3, vget_high_s16( vsrcv4.val[ 1 ] ), cv, 4 );

    vsum0 = vmlal_laneq_s16( vsum0, vget_low_s16( vsrcv5.val[ 0 ] ), cv, 5 );
    vsum1 = vmlal_laneq_s16( vsum1, vget_high_s16( vsrcv5.val[ 0 ] ), cv, 5 );
    vsum2 = vmlal_laneq_s16( vsum2, vget_low_s16( vsrcv5.val[ 1 ] ), cv, 5 );
    vsum3 = vmlal_laneq_s16( vsum3, vget_high_s16( vsrcv5.val[ 1 ] ), cv, 5 );

    vsum0 = vmlal_laneq_s16( vsum0, vget_low_s16( vsrcv6.val[ 0 ] ), cv, 6 );
    vsum1 = vmlal_laneq_s16( vsum1, vget_high_s16( vsrcv6.val[ 0 ] ), cv, 6 );
    vsum2 = vmlal_laneq_s16( vsum2, vget_low_s16( vsrcv6.val[ 1 ] ), cv, 6 );
    vsum3 = vmlal_laneq_s16( vsum3, vget_high_s16( vsrcv6.val[ 1 ] ), cv, 6 );

    vsum0 = vmlal_laneq_s16( vsum0, vget_low_s16( vsrcv7.val[ 0 ] ), cv, 7 );
    vsum1 = vmlal_laneq_s16( vsum1, vget_high_s16( vsrcv7.val[ 0 ] ), cv, 7 );
    vsum2 = vmlal_laneq_s16( vsum2, vget_low_s16( vsrcv7.val[ 1 ] ), cv, 7 );
    vsum3 = vmlal_laneq_s16( vsum3, vget_high_s16( vsrcv7.val[ 1 ] ), cv, 7 );

    int16x8_t vsum01, vsum23;
    if( isLast ) // clip
    {
      vsum0 = vshlq_s32( vsum0, invshift2nd );
      vsum1 = vshlq_s32( vsum1, invshift2nd );
      vsum2 = vshlq_s32( vsum2, invshift2nd );
      vsum3 = vshlq_s32( vsum3, invshift2nd );

      vsum01 = vcombine_s16( vqmovn_s32( vsum0 ), vqmovn_s32( vsum1 ) );
      vsum23 = vcombine_s16( vqmovn_s32( vsum2 ), vqmovn_s32( vsum3 ) );

      vsum01 = vminq_s16( vibdimax, vmaxq_s16( vibdimin, vsum01 ) );
      vsum23 = vminq_s16( vibdimax, vmaxq_s16( vibdimin, vsum23 ) );
    }
    else
    {
      vsum01 = vcombine_s16( vqshrn_n_s32( vsum0, IF_FILTER_PREC ), vqshrn_n_s32( vsum1, IF_FILTER_PREC ) );
      vsum23 = vcombine_s16( vqshrn_n_s32( vsum2, IF_FILTER_PREC ), vqshrn_n_s32( vsum3, IF_FILTER_PREC ) );
    }

    vsrcv0 = vsrcv1;
    vsrcv1 = vsrcv2;
    vsrcv2 = vsrcv3;
    vsrcv3 = vsrcv4;
    vsrcv4 = vsrcv5;
    vsrcv5 = vsrcv6;
    vsrcv6 = vsrcv7;

    vst1q_s16( dst + 0, vsum01 );
    vst1q_s16( dst + 8, vsum23 );
    dst += dstStride;
  } while( --height != 0 );
}

template<>
void InterpolationFilter::_initInterpolationFilterARM<NEON>()
{
  m_filter16x16[ 0 ][ 0 ] = simdFilter16xX_N8_neon<false>;
  m_filter16x16[ 0 ][ 1 ] = simdFilter16xX_N8_neon<true>;

  m_filterN2_2D = simdInterpolateN2_2D_neon;
}

} // namespace vvenc
#endif
//! \}

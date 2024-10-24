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

#include "../InterpolationFilter.h"
#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "sum_neon.h"

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

static int16x4_t filter4xX_N8_neon( Pel const* src, int16x8_t ch, int32x4_t voffset1, int32x4_t invshift1st )
{
  int16x8_t vsrca0 = vld1q_s16( src + 0 );
  int16x8_t vsrca1 = vld1q_s16( src + 1 );
  int16x8_t vsrca2 = vld1q_s16( src + 2 );
  int16x8_t vsrca3 = vld1q_s16( src + 3 );

  int32x4_t a0 = vmull_s16( vget_low_s16( vsrca0 ), vget_low_s16( ch ) );
  int32x4_t a1 = vmull_s16( vget_low_s16( vsrca1 ), vget_low_s16( ch ) );
  int32x4_t a2 = vmull_s16( vget_low_s16( vsrca2 ), vget_low_s16( ch ) );
  int32x4_t a3 = vmull_s16( vget_low_s16( vsrca3 ), vget_low_s16( ch ) );

  a0 = vmlal_s16( a0, vget_high_s16( vsrca0 ), vget_high_s16( ch ) );
  a1 = vmlal_s16( a1, vget_high_s16( vsrca1 ), vget_high_s16( ch ) );
  a2 = vmlal_s16( a2, vget_high_s16( vsrca2 ), vget_high_s16( ch ) );
  a3 = vmlal_s16( a3, vget_high_s16( vsrca3 ), vget_high_s16( ch ) );

  int32x4_t vsuma = horizontal_add_4d_s32x4( a0, a1, a2, a3 );
  vsuma           = vaddq_s32( vsuma, voffset1 );
  vsuma           = vshlq_s32( vsuma, invshift1st );
  return vqmovn_s32( vsuma );
}

static int16x8_t filter8xX_N8_neon( Pel const* src, int16x8_t ch, int32x4_t voffset1, int32x4_t invshift1st )
{
  int16x4_t lo = filter4xX_N8_neon( src + 0, ch, voffset1, invshift1st );
  int16x4_t hi = filter4xX_N8_neon( src + 4, ch, voffset1, invshift1st );
  return vcombine_s16( lo, hi );
}

static int16x8x2_t filter16xX_N8_neon( Pel const* src, int16x8_t ch, int32x4_t voffset1, int32x4_t invshift1st )
{
  int16x8_t a = filter8xX_N8_neon( src + 0, ch, voffset1, invshift1st );
  int16x8_t b = filter8xX_N8_neon( src + 8, ch, voffset1, invshift1st );
  return ( int16x8x2_t ){ a, b };
}

template<bool isLast>
static void simdFilter4xX_N8_neon( const ClpRng& clpRng, Pel const* src, int srcStride, Pel* dst, int dstStride,
                                   int width, int height, TFilterCoeff const* coeffH, TFilterCoeff const* coeffV )
{
  OFFSET( src, srcStride, -3, -3 );

  // With the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20.
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

  const int16x4_t vibdimin = vdup_n_s16( clpRng.min() );
  const int16x4_t vibdimax = vdup_n_s16( clpRng.max() );

  int16x8_t ch = vld1q_s16( coeffH );
  int16x8_t cv = vld1q_s16( coeffV );

  int32x4_t invshift1st = vdupq_n_s32( -shift1st );
  int32x4_t invshift2nd = vdupq_n_s32( -shift2nd );

  int16x4_t vsrcv0 = filter4xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x4_t vsrcv1 = filter4xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x4_t vsrcv2 = filter4xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x4_t vsrcv3 = filter4xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x4_t vsrcv4 = filter4xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x4_t vsrcv5 = filter4xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x4_t vsrcv6 = filter4xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;

  do
  {
    int16x4_t vsrcv7 = filter4xX_N8_neon( src, ch, voffset1, invshift1st );
    src += srcStride;

    int32x4_t vsum0 = vdupq_n_s32( offset2nd );
    vsum0           = vmlal_lane_s16( vsum0, vsrcv0, vget_low_s16( cv ), 0 );
    vsum0           = vmlal_lane_s16( vsum0, vsrcv1, vget_low_s16( cv ), 1 );
    vsum0           = vmlal_lane_s16( vsum0, vsrcv2, vget_low_s16( cv ), 2 );
    vsum0           = vmlal_lane_s16( vsum0, vsrcv3, vget_low_s16( cv ), 3 );
    vsum0           = vmlal_lane_s16( vsum0, vsrcv4, vget_high_s16( cv ), 0 );
    vsum0           = vmlal_lane_s16( vsum0, vsrcv5, vget_high_s16( cv ), 1 );
    vsum0           = vmlal_lane_s16( vsum0, vsrcv6, vget_high_s16( cv ), 2 );
    vsum0           = vmlal_lane_s16( vsum0, vsrcv7, vget_high_s16( cv ), 3 );

    int16x4_t vsum01;
    if( isLast )  // clip
    {
      vsum01 = vqmovn_s32( vshlq_s32( vsum0, invshift2nd ) );
      vsum01 = vmin_s16( vibdimax, vmax_s16( vibdimin, vsum01 ) );
    }
    else
    {
      vsum01 = vqshrn_n_s32( vsum0, IF_FILTER_PREC );
    }

    vsrcv0 = vsrcv1;
    vsrcv1 = vsrcv2;
    vsrcv2 = vsrcv3;
    vsrcv3 = vsrcv4;
    vsrcv4 = vsrcv5;
    vsrcv5 = vsrcv6;
    vsrcv6 = vsrcv7;

    vst1_s16( dst, vsum01 );
    dst += dstStride;
  } while( --height != 0 );
}

template<bool isLast>
static void simdFilter8xX_N8_neon( const ClpRng& clpRng, Pel const* src, int srcStride, Pel* dst, int dstStride,
                                   int width, int height, TFilterCoeff const* coeffH, TFilterCoeff const* coeffV )
{
  OFFSET( src, srcStride, -3, -3 );

  // With the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20.
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

  int16x8_t vsrcv0 = filter8xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8_t vsrcv1 = filter8xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8_t vsrcv2 = filter8xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8_t vsrcv3 = filter8xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8_t vsrcv4 = filter8xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8_t vsrcv5 = filter8xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8_t vsrcv6 = filter8xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;

  do
  {
    int16x8_t vsrcv7 = filter8xX_N8_neon( src, ch, voffset1, invshift1st );
    src += srcStride;

    int32x4_t vsum0 = vdupq_n_s32( offset2nd );
    int32x4_t vsum1 = vdupq_n_s32( offset2nd );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv0 ), vget_low_s16( cv ), 0 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv0 ), vget_low_s16( cv ), 0 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv1 ), vget_low_s16( cv ), 1 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv1 ), vget_low_s16( cv ), 1 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv2 ), vget_low_s16( cv ), 2 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv2 ), vget_low_s16( cv ), 2 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv3 ), vget_low_s16( cv ), 3 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv3 ), vget_low_s16( cv ), 3 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv4 ), vget_high_s16( cv ), 0 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv4 ), vget_high_s16( cv ), 0 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv5 ), vget_high_s16( cv ), 1 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv5 ), vget_high_s16( cv ), 1 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv6 ), vget_high_s16( cv ), 2 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv6 ), vget_high_s16( cv ), 2 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv7 ), vget_high_s16( cv ), 3 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv7 ), vget_high_s16( cv ), 3 );

    int16x8_t vsum01;
    if( isLast )  // clip
    {
      vsum0 = vshlq_s32( vsum0, invshift2nd );
      vsum1 = vshlq_s32( vsum1, invshift2nd );

      vsum01 = vcombine_s16( vqmovn_s32( vsum0 ), vqmovn_s32( vsum1 ) );
      vsum01 = vminq_s16( vibdimax, vmaxq_s16( vibdimin, vsum01 ) );
    }
    else
    {
      vsum01 = vcombine_s16( vqshrn_n_s32( vsum0, IF_FILTER_PREC ), vqshrn_n_s32( vsum1, IF_FILTER_PREC ) );
    }

    vsrcv0 = vsrcv1;
    vsrcv1 = vsrcv2;
    vsrcv2 = vsrcv3;
    vsrcv3 = vsrcv4;
    vsrcv4 = vsrcv5;
    vsrcv5 = vsrcv6;
    vsrcv6 = vsrcv7;

    vst1q_s16( dst, vsum01 );
    dst += dstStride;
  } while( --height != 0 );
}

template<bool isLast>
static void simdFilter16xX_N8_neon( const ClpRng& clpRng, Pel const* src, int srcStride, Pel* dst, int dstStride,
                                    int width, int height, TFilterCoeff const* coeffH, TFilterCoeff const* coeffV )
{
  OFFSET( src, srcStride, -3, -3 );

  // With the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20.
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

  int16x8x2_t vsrcv0 = filter16xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8x2_t vsrcv1 = filter16xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8x2_t vsrcv2 = filter16xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8x2_t vsrcv3 = filter16xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8x2_t vsrcv4 = filter16xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8x2_t vsrcv5 = filter16xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;
  int16x8x2_t vsrcv6 = filter16xX_N8_neon( src, ch, voffset1, invshift1st );
  src += srcStride;

  do
  {
    int16x8x2_t vsrcv7 = filter16xX_N8_neon( src, ch, voffset1, invshift1st );
    src += srcStride;

    int32x4_t vsum0 = vdupq_n_s32( offset2nd );
    int32x4_t vsum1 = vdupq_n_s32( offset2nd );
    int32x4_t vsum2 = vdupq_n_s32( offset2nd );
    int32x4_t vsum3 = vdupq_n_s32( offset2nd );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv0.val[ 0 ] ), vget_low_s16( cv ), 0 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv0.val[ 0 ] ), vget_low_s16( cv ), 0 );
    vsum2 = vmlal_lane_s16( vsum2, vget_low_s16( vsrcv0.val[ 1 ] ), vget_low_s16( cv ), 0 );
    vsum3 = vmlal_lane_s16( vsum3, vget_high_s16( vsrcv0.val[ 1 ] ), vget_low_s16( cv ), 0 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv1.val[ 0 ] ), vget_low_s16( cv ), 1 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv1.val[ 0 ] ), vget_low_s16( cv ), 1 );
    vsum2 = vmlal_lane_s16( vsum2, vget_low_s16( vsrcv1.val[ 1 ] ), vget_low_s16( cv ), 1 );
    vsum3 = vmlal_lane_s16( vsum3, vget_high_s16( vsrcv1.val[ 1 ] ), vget_low_s16( cv ), 1 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv2.val[ 0 ] ), vget_low_s16( cv ), 2 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv2.val[ 0 ] ), vget_low_s16( cv ), 2 );
    vsum2 = vmlal_lane_s16( vsum2, vget_low_s16( vsrcv2.val[ 1 ] ), vget_low_s16( cv ), 2 );
    vsum3 = vmlal_lane_s16( vsum3, vget_high_s16( vsrcv2.val[ 1 ] ), vget_low_s16( cv ), 2 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv3.val[ 0 ] ), vget_low_s16( cv ), 3 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv3.val[ 0 ] ), vget_low_s16( cv ), 3 );
    vsum2 = vmlal_lane_s16( vsum2, vget_low_s16( vsrcv3.val[ 1 ] ), vget_low_s16( cv ), 3 );
    vsum3 = vmlal_lane_s16( vsum3, vget_high_s16( vsrcv3.val[ 1 ] ), vget_low_s16( cv ), 3 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv4.val[ 0 ] ), vget_high_s16( cv ), 0 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv4.val[ 0 ] ), vget_high_s16( cv ), 0 );
    vsum2 = vmlal_lane_s16( vsum2, vget_low_s16( vsrcv4.val[ 1 ] ), vget_high_s16( cv ), 0 );
    vsum3 = vmlal_lane_s16( vsum3, vget_high_s16( vsrcv4.val[ 1 ] ), vget_high_s16( cv ), 0 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv5.val[ 0 ] ), vget_high_s16( cv ), 1 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv5.val[ 0 ] ), vget_high_s16( cv ), 1 );
    vsum2 = vmlal_lane_s16( vsum2, vget_low_s16( vsrcv5.val[ 1 ] ), vget_high_s16( cv ), 1 );
    vsum3 = vmlal_lane_s16( vsum3, vget_high_s16( vsrcv5.val[ 1 ] ), vget_high_s16( cv ), 1 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv6.val[ 0 ] ), vget_high_s16( cv ), 2 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv6.val[ 0 ] ), vget_high_s16( cv ), 2 );
    vsum2 = vmlal_lane_s16( vsum2, vget_low_s16( vsrcv6.val[ 1 ] ), vget_high_s16( cv ), 2 );
    vsum3 = vmlal_lane_s16( vsum3, vget_high_s16( vsrcv6.val[ 1 ] ), vget_high_s16( cv ), 2 );

    vsum0 = vmlal_lane_s16( vsum0, vget_low_s16( vsrcv7.val[ 0 ] ), vget_high_s16( cv ), 3 );
    vsum1 = vmlal_lane_s16( vsum1, vget_high_s16( vsrcv7.val[ 0 ] ), vget_high_s16( cv ), 3 );
    vsum2 = vmlal_lane_s16( vsum2, vget_low_s16( vsrcv7.val[ 1 ] ), vget_high_s16( cv ), 3 );
    vsum3 = vmlal_lane_s16( vsum3, vget_high_s16( vsrcv7.val[ 1 ] ), vget_high_s16( cv ), 3 );

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
  m_filter4x4[ 0 ][ 0 ] = simdFilter4xX_N8_neon<false>;
  m_filter4x4[ 0 ][ 1 ] = simdFilter4xX_N8_neon<true>;

  m_filter8x8[ 0 ][ 0 ] = simdFilter8xX_N8_neon<false>;
  m_filter8x8[ 0 ][ 1 ] = simdFilter8xX_N8_neon<true>;

  m_filter16x16[ 0 ][ 0 ] = simdFilter16xX_N8_neon<false>;
  m_filter16x16[ 0 ][ 1 ] = simdFilter16xX_N8_neon<true>;

  m_filterN2_2D = simdInterpolateN2_2D_neon;
}

} // namespace vvenc
#endif
//! \}

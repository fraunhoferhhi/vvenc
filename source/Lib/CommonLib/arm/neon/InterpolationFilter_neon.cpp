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

#if defined( TARGET_SIMD_X86 )
#if SIMD_EVERYWHERE_EXTENSION_LEVEL_ID == X86_SIMD_AVX2
#define USE_AVX2
#elif SIMD_EVERYWHERE_EXTENSION_LEVEL_ID == X86_SIMD_SSE42
#define USE_SSE42
#elif SIMD_EVERYWHERE_EXTENSION_LEVEL_ID == X86_SIMD_SSE41
#define USE_SSE41
#endif

# include "../x86/InterpolationFilterX86.h"
#endif  // defined( TARGET_SIMD_X86 )

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
  int16x8x2_t result;
  result.val[0] = filter8xX_N8_neon( src + 0, ch, voffset1, invshift1st );
  result.val[1] = filter8xX_N8_neon( src + 8, ch, voffset1, invshift1st );
  return result; // explicit return since MSVC for arm64 does not support direct return with typecast
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

template<int N, bool shiftBack>
static void simdInterpolateHorM8_Neon( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{

  int16x8_t vibdimin   = vdupq_n_s16( clpRng.min() );
  int16x8_t vibdimax   = vdupq_n_s16( clpRng.max() );
  int32x4_t vsuma, vsumb;
  int16x8_t vsum, vsrc0, vsrc1;

  for( int row = 0; row < height; row++ )
  {
    for( int col = 0; col < width; col+=8 )
    {
      vsuma = vdupq_n_s32(offset);
      vsumb = vdupq_n_s32(offset);

      vsrc0 = vld1q_s16( ( const int16_t * )&src[col] );
      vsrc1 = vld1q_s16( ( const int16_t * )&src[col + 4] );

      vsuma = vmlal_s16( vsuma, vget_low_s16( vextq_s16( vsrc0, vsrc0, 0 ) ), vdup_n_s16( coeff[ 0 ] ) );
      vsuma = vmlal_s16( vsuma, vget_low_s16( vextq_s16( vsrc0, vsrc0, 1 ) ), vdup_n_s16( coeff[ 1 ] ) );
      vsuma = vmlal_s16( vsuma, vget_low_s16( vextq_s16( vsrc0, vsrc0, 2 ) ), vdup_n_s16( coeff[ 2 ] ) );
      vsuma = vmlal_s16( vsuma, vget_low_s16( vextq_s16( vsrc0, vsrc0, 3 ) ), vdup_n_s16( coeff[ 3 ] ) );

      vsumb = vmlal_s16( vsumb, vget_low_s16( vextq_s16( vsrc1, vsrc1, 0 ) ), vdup_n_s16( coeff[ 0 ] ) );
      vsumb = vmlal_s16( vsumb, vget_low_s16( vextq_s16( vsrc1, vsrc1, 1 ) ), vdup_n_s16( coeff[ 1 ] ) );
      vsumb = vmlal_s16( vsumb, vget_low_s16( vextq_s16( vsrc1, vsrc1, 2 ) ), vdup_n_s16( coeff[ 2 ] ) );
      vsumb = vmlal_s16( vsumb, vget_low_s16( vextq_s16( vsrc1, vsrc1, 3 ) ), vdup_n_s16( coeff[ 3 ] ) );

      if( N == 8 )
      {
        vsrc0 = vld1q_s16( ( const int16_t* )&src[ col + 8 ] );
        vsuma = vmlal_s16( vsuma, vget_low_s16( vextq_s16( vsrc1, vsrc1, 0 ) ), vdup_n_s16( coeff[ 4 ] ) );
        vsuma = vmlal_s16( vsuma, vget_low_s16( vextq_s16( vsrc1, vsrc1, 1 ) ), vdup_n_s16( coeff[ 5 ] ) );
        vsuma = vmlal_s16( vsuma, vget_low_s16( vextq_s16( vsrc1, vsrc1, 2 ) ), vdup_n_s16( coeff[ 6 ] ) );
        vsuma = vmlal_s16( vsuma, vget_low_s16( vextq_s16( vsrc1, vsrc1, 3 ) ), vdup_n_s16( coeff[ 7 ] ) );

        vsumb = vmlal_s16( vsumb, vget_low_s16( vextq_s16( vsrc0, vsrc0, 0 ) ), vdup_n_s16( coeff[ 4 ] ) );
        vsumb = vmlal_s16( vsumb, vget_low_s16( vextq_s16( vsrc0, vsrc0, 1 ) ), vdup_n_s16( coeff[ 5 ] ) );
        vsumb = vmlal_s16( vsumb, vget_low_s16( vextq_s16( vsrc0, vsrc0, 2 ) ), vdup_n_s16( coeff[ 6 ] ) );
        vsumb = vmlal_s16( vsumb, vget_low_s16( vextq_s16( vsrc0, vsrc0, 3 ) ), vdup_n_s16( coeff[ 7 ] ) );
      }
      if( N == 6 )
      {
        vsrc0 = vld1q_s16( ( const int16_t* )&src[ col + 8 ] );
        vsuma = vmlal_s16( vsuma, vget_low_s16( vextq_s16( vsrc1, vsrc1, 0 ) ), vdup_n_s16( coeff[ 4 ] ) );
        vsuma = vmlal_s16( vsuma, vget_low_s16( vextq_s16( vsrc1, vsrc1, 1 ) ), vdup_n_s16( coeff[ 5 ] ) );

        vsumb = vmlal_s16( vsumb, vget_low_s16( vextq_s16( vsrc0, vsrc0, 0 ) ), vdup_n_s16( coeff[ 4 ] ) );
        vsumb = vmlal_s16( vsumb, vget_low_s16( vextq_s16( vsrc0, vsrc0, 1 ) ), vdup_n_s16( coeff[ 5 ] ) );
      }

      vsuma = vshlq_s32( vsuma, vdupq_n_s32( -1 * shift ) );
      vsumb = vshlq_s32( vsumb, vdupq_n_s32( -1 * shift ) );
      vsum  = vcombine_s16( vqmovn_s32( vsuma ), vqmovn_s32( vsumb ) );

      if( shiftBack )
      {
        vsum = vminq_s16( vibdimax, vmaxq_s16( vibdimin, vsum ) );
      }
      vst1q_s16( ( int16_t* )&dst[ col ], vsum );
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<int N, bool shiftBack>
static void simdInterpolateVerM8_Neon( const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  const Pel* srcOrig = src;
  int16_t *dstOrig = dst;

  int16x8_t vsrc[N+1];
  int32x4_t voffset = vdupq_n_s32( offset );
  int16x8_t vibdimin = vdupq_n_s16( clpRng.min() );
  int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );
  int32x4_t vsuma, vsumb;
  int16x8_t vsum;
  vsrc[N] = vdupq_n_s16(0);
  for( int col = 0; col < width; col += 8 )
  {

    for( int i = 0; i < N - 1; i++ )
    {
      vsrc[i] = vld1q_s16( ( int16_t const * )&src[col + i * srcStride] );
    }

    for( int row = 0; row < height; row++ )
    {
      vsrc[N - 1] = vld1q_s16( ( int16_t const * )&src[col + ( N - 1 ) * srcStride] );
      vsuma = vsumb = voffset;
      if(N < 2)
      {  
        vsuma = vmlal_s16(vsuma, vget_low_s16(vsrc[ 0]), vdup_n_s16(coeff[0]));
        vsuma = vmlal_s16(vsuma, vget_low_s16(vsrc[ 1]), vdup_n_s16(coeff[1]));
        vsumb = vmlal_s16( vsumb, vget_high_s16( vsrc[ 0 ] ), vdup_n_s16( coeff[ 0 ] ) );
        vsumb = vmlal_s16( vsumb, vget_high_s16( vsrc[ 1 ] ), vdup_n_s16( coeff[ 1 ] ) );

        vsrc[0] = vsrc[1];
      }
      else
      {
        for( int i = 0; i < N; i += 2 )
        {
          vsuma = vmlal_s16(vsuma, vget_low_s16(vsrc[i + 0]), vdup_n_s16(coeff[i + 0]));
          vsuma = vmlal_s16(vsuma, vget_low_s16(vsrc[i + 1]), vdup_n_s16(coeff[i + 1]));
          vsumb       = vmlal_s16( vsumb, vget_high_s16( vsrc[ i + 0 ] ), vdup_n_s16( coeff[ i + 0 ] ) );
          vsumb       = vmlal_s16( vsumb, vget_high_s16( vsrc[ i + 1 ] ), vdup_n_s16( coeff[ i + 1 ] ) );
          vsrc[i    ] = vsrc[i + 1];
          vsrc[i + 1] = vsrc[i + 2];
        }
      }
      vsuma = vshlq_s32( vsuma, vdupq_n_s32(-1*shift) );
      vsumb = vshlq_s32( vsumb, vdupq_n_s32(-1*shift) );
      vsum = vcombine_s16(vqmovn_s32(vsuma), vqmovn_s32(vsumb));
      if( shiftBack ) 
      {
        vsum = vminq_s16( vibdimax, vmaxq_s16( vibdimin, vsum ) );
      }
      vst1q_s16((int16_t*) &dst[col], vsum);
      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
}

template<int N, bool isVertical, bool isFirst, bool isLast>
static void simdFilterARM( const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, TFilterCoeff const *coeff )
{
  int row, col;

  Pel c[8];
  c[0] = coeff[0];
  c[1] = coeff[1];
  if( N >= 4 )
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if( N >= 6 )
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
  if( N == 8 )
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }

  int cStride = ( isVertical ) ? srcStride : 1;
  src -= ( N/2 - 1 ) * cStride;

  int offset;
  int headRoom = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shift    = IF_FILTER_PREC;
  CHECK( shift < 0, "Negative shift" );
  
  if( N != 2 )
  {
    if( isLast )
    {
      shift  += ( isFirst ) ? 0 : headRoom;
      offset  = 1 << ( shift - 1 );
      offset += ( isFirst ) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
    }
    else
    {
      shift -= ( isFirst ) ? headRoom : 0;
      offset = ( isFirst ) ? -IF_INTERNAL_OFFS * (1<< shift) : 0;
    }
  }
  else
  {
    if( isFirst )
    {
      shift  = IF_FILTER_PREC_BILINEAR - (IF_INTERNAL_PREC_BILINEAR - clpRng.bd);
      offset = 1 << (shift - 1);
    }
    else
    {
      shift  = 4;
      offset = 1 << (shift - 1);
    }
  }

  CHECKD( clpRng.bd > 10, "VVenC does not support bitdepths larger than 10!" );

  if( N == 6 )
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
#if defined( TARGET_SIMD_X86 )
    int src8tOff = cStride;
#endif
      
    if( !( width & 7 ) )
    {
      if( !isVertical )
      {
        simdInterpolateHorM8_Neon<6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c + 1 );
      }
      else
      {
        simdInterpolateVerM8_Neon<6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c + 1 );
      }
    }
#if defined( TARGET_SIMD_X86 )
    else if( !( width & 3 ) )
    {
      if( !isVertical )
      {
        simdInterpolateHorM4<SIMD_EVERYWHERE_EXTENSION_LEVEL, 8, isLast>( src - src8tOff, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      else
        simdInterpolateVerM4<SIMD_EVERYWHERE_EXTENSION_LEVEL, 6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c + 1 );
    }
    else if( width == 1 && !isVertical )
    {
      simdInterpolateHorM1<SIMD_EVERYWHERE_EXTENSION_LEVEL, 8, isLast>( src - src8tOff, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
    }
    else if( width == 1 && isVertical )
#else
    else
#endif
    {
      c[0] = c[1]; c[1] = c[2]; c[2] = c[3]; c[3] = c[4]; c[4] = c[5]; c[5] = c[6];
      goto scalar_if;
    }

    return;
  }

  if( !isVertical && N != 2 )
  {
    if( ( width & 7 ) == 0 )
    {
      simdInterpolateHorM8_Neon<N, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
    }
#if defined( TARGET_SIMD_X86 )
    else if( ( width & 3 ) == 0 )
      simdInterpolateHorM4<SIMD_EVERYWHERE_EXTENSION_LEVEL, N, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
    else if( ( width & 1 ) == 0 )
      simdInterpolateHorM2<SIMD_EVERYWHERE_EXTENSION_LEVEL, N, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
    else
      simdInterpolateHorM1<SIMD_EVERYWHERE_EXTENSION_LEVEL, N, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
#else
    else goto scalar_if;
#endif
    return;
  }

  else if( N != 2 )
  {
    if( ( width & 7 ) == 0 )
    {
      simdInterpolateVerM8_Neon<N, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
    }
#if defined( TARGET_SIMD_X86 )
    else if( ( width & 3 ) == 0 )
      simdInterpolateVerM4<SIMD_EVERYWHERE_EXTENSION_LEVEL, N, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
    else if( ( width & 1 ) == 0 )
      simdInterpolateVerM2<SIMD_EVERYWHERE_EXTENSION_LEVEL, N, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
    else
      simdInterpolateVerM1<SIMD_EVERYWHERE_EXTENSION_LEVEL, N, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
#else
    else goto scalar_if;
#endif
    return;
  }
  else
  {
    THROW( "To be implemented" );
    return;
  }

scalar_if:
  for( row = 0; row < height; row++ )
  {
    for( col = 0; col < width; col++ )
    {
      int sum;

      sum  = src[col + 0 * cStride] * c[0];
      sum += src[col + 1 * cStride] * c[1];
      if( N >= 4 )
      {
        sum += src[col + 2 * cStride] * c[2];
        sum += src[col + 3 * cStride] * c[3];
      }
      if( N >= 6 )
      {
        sum += src[col + 4 * cStride] * c[4];
        sum += src[col + 5 * cStride] * c[5];
      }
      if( N == 8 )  

      {
        sum += src[col + 6 * cStride] * c[6];
        sum += src[col + 7 * cStride] * c[7];
      }

      Pel val = ( sum + offset ) >> shift;
      if( isLast )
      {
        val = ClipPel( val, clpRng );
      }
      dst[col] = val;
    }

    src += srcStride;
    dst += dstStride;
  }
}

void simdFilterCopy_DMVR_neon( const ClpRng& clpRng, const Pel* src, int srcStride, Pel* dst, int dstStride, int width,
                               int height )
{
  // Special case of FilterCopy<true, false> where biMCForDMVR = true, clpRng.bd < 10.

  CHECK( height < 1, "Height must be >= 1" );
  CHECK( width < 4 || width % 4, "Width must be >= 4 and a multiple of 4" );
  CHECK( clpRng.bd - IF_INTERNAL_PREC_BILINEAR > 0, "VVenC doesn't support bitdepth over '10'!" );

  const int16_t shift10BitOut = IF_INTERNAL_PREC_BILINEAR - clpRng.bd;

  do
  {
    int w;
    for( w = 0; w <= width - 16; w += 16 )
    {
      int16x8_t s_lo = vld1q_s16( src + w + 0 );
      int16x8_t s_hi = vld1q_s16( src + w + 8 );
      s_lo = vshlq_s16( s_lo, vdupq_n_s16( shift10BitOut ) );
      s_hi = vshlq_s16( s_hi, vdupq_n_s16( shift10BitOut ) );
      vst1q_s16( dst + w + 0, s_lo );
      vst1q_s16( dst + w + 8, s_hi );
    }
    if( width & 8 )
    {
      int16x8_t s = vld1q_s16( src + w );
      s = vshlq_s16( s, vdupq_n_s16( shift10BitOut ) );
      vst1q_s16( dst + w, s );

      w += 8;
    }
    if( width & 4 )
    {
      int16x4_t s = vld1_s16( src + w );
      s = vshl_s16( s, vdup_n_s16( shift10BitOut ) );
      vst1_s16( dst + w, s );
    }

    src += srcStride;
    dst += dstStride;
  } while( --height != 0 );
}

template<bool isFirst, bool isLast>
void simdFilterCopy_noDMVR_neon( const ClpRng& clpRng, const Pel* src, int srcStride, Pel* dst, int dstStride,
                                 int width, int height )
{
  CHECK( height < 1, "Height must be >= 1" );
  CHECK( width < 4 || width % 4, "Width must be >= 4 and a multiple of 4" );

  if( isFirst == isLast )
  {
    do
    {
      int w;
      for( w = 0; w <= width - 16; w += 16 )
      {
        int16x8_t s_lo = vld1q_s16( src + w + 0 );
        int16x8_t s_hi = vld1q_s16( src + w + 8 );
        vst1q_s16( dst + w + 0, s_lo );
        vst1q_s16( dst + w + 8, s_hi );
      }
      if( width & 8 )
      {
        int16x8_t s = vld1q_s16( src + w );
        vst1q_s16( dst + w, s );

        w += 8;
      }
      if( width & 4 )
      {
        int16x4_t s = vld1_s16( src + w );
        vst1_s16( dst + w, s );
      }

      src += srcStride;
      dst += dstStride;
    } while( --height != 0 );
  }
  else if( isFirst )
  {
    const int16_t shift = std::max<int>( 2, IF_INTERNAL_PREC - clpRng.bd );

    do
    {
      int w;
      for( w = 0; w <= width - 16; w += 16 )
      {
        int16x8_t s_lo = vld1q_s16( src + w + 0 );
        int16x8_t s_hi = vld1q_s16( src + w + 8 );
        s_lo = vshlq_s16( s_lo, vdupq_n_s16( shift ) );
        s_hi = vshlq_s16( s_hi, vdupq_n_s16( shift ) );
        s_lo = vsubq_s16( s_lo, vdupq_n_s16( IF_INTERNAL_OFFS ) );
        s_hi = vsubq_s16( s_hi, vdupq_n_s16( IF_INTERNAL_OFFS ) );
        vst1q_s16( dst + w + 0, s_lo );
        vst1q_s16( dst + w + 8, s_hi );
      }
      if( width & 8 )
      {
        int16x8_t s = vld1q_s16( src + w );
        s = vshlq_s16( s, vdupq_n_s16( shift ) );
        s = vsubq_s16( s, vdupq_n_s16( IF_INTERNAL_OFFS ) );
        vst1q_s16( dst + w, s );

        w += 8;
      }
      if( width & 4 )
      {
        int16x4_t s = vld1_s16( src + w );
        s = vshl_s16( s, vdup_n_s16( shift ) );
        s = vsub_s16( s, vdup_n_s16( IF_INTERNAL_OFFS ) );
        vst1_s16( dst + w, s );
      }

      src += srcStride;
      dst += dstStride;
    } while( --height != 0 );
  }
  else
  {
    const int16_t shift = std::max<int>( 2, IF_INTERNAL_PREC - clpRng.bd );
    const int16_t offset = ( 1 << ( shift - 1 ) ) + IF_INTERNAL_OFFS;

    do
    {
      int w;
      for( w = 0; w <= width - 16; w += 16 )
      {
        int16x8_t s_lo = vld1q_s16( src + w + 0 );
        int16x8_t s_hi = vld1q_s16( src + w + 8 );
        s_lo = vaddq_s16( s_lo, vdupq_n_s16( offset ) );
        s_hi = vaddq_s16( s_hi, vdupq_n_s16( offset ) );
        s_lo = vshlq_s16( s_lo, vdupq_n_s16( -shift ) );
        s_hi = vshlq_s16( s_hi, vdupq_n_s16( -shift ) );
        s_lo = vminq_s16( s_lo, vdupq_n_s16( clpRng.max() ) );
        s_hi = vminq_s16( s_hi, vdupq_n_s16( clpRng.max() ) );
        vst1q_s16( dst + w + 0, s_lo );
        vst1q_s16( dst + w + 8, s_hi );
      }
      if( width & 8 )
      {
        int16x8_t s = vld1q_s16( src + w );
        s = vaddq_s16( s, vdupq_n_s16( offset ) );
        s = vshlq_s16( s, vdupq_n_s16( -shift ) );
        s = vminq_s16( s, vdupq_n_s16( clpRng.max() ) );
        vst1q_s16( dst + w, s );

        w += 8;
      }
      if( width & 4 )
      {
        int16x4_t s = vld1_s16( src + w );
        s = vadd_s16( s, vdup_n_s16( offset ) );
        s = vshl_s16( s, vdup_n_s16( -shift ) );
        s = vmin_s16( s, vdup_n_s16( clpRng.max() ) );
        vst1_s16( dst + w, s );
      }

      src += srcStride;
      dst += dstStride;
    } while( --height != 0 );
  }
}

template<bool isFirst, bool isLast>
void simdFilterCopy_neon( const ClpRng& clpRng, const Pel* src, int srcStride, Pel* dst, int dstStride, int width,
                          int height, bool biMCForDMVR )
{
  if( biMCForDMVR && width % 4 == 0 )
  {
    if( isFirst && !isLast )
    {
      if( clpRng.bd == 10 )
      {
        simdFilterCopy_noDMVR_neon<true, true>( clpRng, src, srcStride, dst, dstStride, width, height );
      }
      else
      {
        simdFilterCopy_DMVR_neon( clpRng, src, srcStride, dst, dstStride, width, height );
      }
    }
    else // Invalid path: Other cases of <isFirst, isLast> for biMCForDMVR are not used.
    {
      InterpolationFilter::filterCopy<isFirst, isLast>( clpRng, src, srcStride, dst, dstStride, width, height,
                                                        biMCForDMVR );
    }
  }
  else if( width % 4 == 0 )
  {
    simdFilterCopy_noDMVR_neon<isFirst, isLast>( clpRng, src, srcStride, dst, dstStride, width, height );
  }
  else // Scalar
  {
    InterpolationFilter::filterCopy<isFirst, isLast>( clpRng, src, srcStride, dst, dstStride, width, height,
                                                      biMCForDMVR );
  }
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

  m_filterHor[0][0][0] = simdFilterARM<8, false, false, false>;
  m_filterHor[0][0][1] = simdFilterARM<8, false, false, true>;
  m_filterHor[0][1][0] = simdFilterARM<8, false, true, false>;
  m_filterHor[0][1][1] = simdFilterARM<8, false, true, true>;

  m_filterHor[1][0][0] = simdFilterARM<4, false, false, false>;
  m_filterHor[1][0][1] = simdFilterARM<4, false, false, true>;
  m_filterHor[1][1][0] = simdFilterARM<4, false, true, false>;
  m_filterHor[1][1][1] = simdFilterARM<4, false, true, true>;

  m_filterHor[3][0][0] = simdFilterARM<6, false, false, false>;
  m_filterHor[3][0][1] = simdFilterARM<6, false, false, true>;
  m_filterHor[3][1][0] = simdFilterARM<6, false, true, false>;
  m_filterHor[3][1][1] = simdFilterARM<6, false, true, true>;

  m_filterVer[0][0][0] = simdFilterARM<8, true, false, false>;
  m_filterVer[0][0][1] = simdFilterARM<8, true, false, true>;
  m_filterVer[0][1][0] = simdFilterARM<8, true, true, false>;
  m_filterVer[0][1][1] = simdFilterARM<8, true, true, true>;

  m_filterVer[1][0][0] = simdFilterARM<4, true, false, false>;
  m_filterVer[1][0][1] = simdFilterARM<4, true, false, true>;
  m_filterVer[1][1][0] = simdFilterARM<4, true, true, false>;
  m_filterVer[1][1][1] = simdFilterARM<4, true, true, true>;

  m_filterVer[3][0][0] = simdFilterARM<6, true, false, false>;
  m_filterVer[3][0][1] = simdFilterARM<6, true, false, true>;
  m_filterVer[3][1][0] = simdFilterARM<6, true, true, false>;
  m_filterVer[3][1][1] = simdFilterARM<6, true, true, true>;

  m_filterCopy[0][0] = simdFilterCopy_neon<false, false>;
  m_filterCopy[0][1] = simdFilterCopy_neon<false, true>;
  m_filterCopy[1][0] = simdFilterCopy_neon<true, false>;
  m_filterCopy[1][1] = simdFilterCopy_neon<true, true>;
}

} // namespace vvenc
#endif
//! \}

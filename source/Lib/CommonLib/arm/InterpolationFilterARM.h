/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
 * \file
 * \brief Implementation of InterpolationFilter class
 */
//  ====================================================================================================================
//  Includes
//  ====================================================================================================================

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "../InterpolationFilter.h"


namespace vvenc
{

#ifdef TARGET_SIMD_ARM
#if __ARM_ARCH >= 8

template<ARM_VEXT vext>
static void simdInterpolateN2_2D( const ClpRng& clpRng, const Pel* src, const int srcStride, Pel* dst, const int dstStride, int width, int height, TFilterCoeff const *ch, TFilterCoeff const *cv )
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

template<ARM_VEXT vext, bool isLast>
void simdFilter16xX_N8( const ClpRng& clpRng, Pel const *src, int srcStride, Pel* dst, int dstStride, int width, int height, TFilterCoeff const *coeffH, TFilterCoeff const *coeffV )
{
  OFFSET( src, srcStride, -3, -3 );

  int       offset1st, offset2nd;
  int       headRoom  = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  const int shift1st  = IF_FILTER_PREC - headRoom;
  int       shift2nd  = IF_FILTER_PREC;
  int       extHeight = height + 7;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20

  //  shift1st -= headRoom;
  offset1st = -IF_INTERNAL_OFFS * ( 1 << shift1st );

  if( isLast )
  {
    shift2nd += headRoom;
    offset2nd = 1 << ( shift2nd - 1 );
    offset2nd += IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    offset2nd = 0;
  }
  const int32x4_t voffset1 = vdupq_n_s32( offset1st );

  const int16x8_t vibdimin = vdupq_n_s16( clpRng.min() );
  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );

  int64x1x2_t vcoeff0 = vld2_s64( (int64_t*) coeffH );
  int16x8_t vsum;
  int32x4_t vsuma, vsumb;

  int32x4_t vsrcv[ 2 ][ 9 ];

  int32x4_t invshift1st = vdupq_n_s32( -shift1st );
  int32x4_t invshift2nd = vdupq_n_s32( -shift2nd );

  for( int row = 0; row < extHeight; row++ )
  {
    int32x4_t vsrc0, vsrc1;
    int16x4_t vsrca00, vsrca01, vsrca10, vsrca11;
    int16x4_t vsrcb00, vsrcb01, vsrcb10, vsrcb11;

    vsrca00 = vld1_s16( &src[ 0 ] );
    vsrca01 = vld1_s16( &src[ 1 ] );
    vsrca10 = vld1_s16( &src[ 2 ] );
    vsrca11 = vld1_s16( &src[ 3 ] );

    for( int j = 0; j < 2; j++ )
    {
      vsrcb00 = vld1_s16( &src[ ( j << 3 ) + 4 ] );
      vsrcb01 = vld1_s16( &src[ ( j << 3 ) + 5 ] );
      vsrcb10 = vld1_s16( &src[ ( j << 3 ) + 6 ] );
      vsrcb11 = vld1_s16( &src[ ( j << 3 ) + 7 ] );

      vsuma[ 0 ] = vaddvq_s32( vmull_s16( vsrca00, vreinterpret_s16_s64( vcoeff0.val[ 0 ] ) ) );
      vsuma[ 1 ] = vaddvq_s32( vmull_s16( vsrca01, vreinterpret_s16_s64( vcoeff0.val[ 0 ] ) ) );
      vsuma[ 2 ] = vaddvq_s32( vmull_s16( vsrca10, vreinterpret_s16_s64( vcoeff0.val[ 0 ] ) ) );
      vsuma[ 3 ] = vaddvq_s32( vmull_s16( vsrca11, vreinterpret_s16_s64( vcoeff0.val[ 0 ] ) ) );

      vsumb[ 0 ] = vaddvq_s32( vmull_s16( vsrcb00, vreinterpret_s16_s64( vcoeff0.val[ 0 ] ) ) );
      vsumb[ 1 ] = vaddvq_s32( vmull_s16( vsrcb01, vreinterpret_s16_s64( vcoeff0.val[ 0 ] ) ) );
      vsumb[ 2 ] = vaddvq_s32( vmull_s16( vsrcb10, vreinterpret_s16_s64( vcoeff0.val[ 0 ] ) ) );
      vsumb[ 3 ] = vaddvq_s32( vmull_s16( vsrcb11, vreinterpret_s16_s64( vcoeff0.val[ 0 ] ) ) );

      vsrc1[ 0 ] = vaddvq_s32( vmull_s16( vsrcb00, vreinterpret_s16_s64( vcoeff0.val[ 1 ] ) ) );
      vsrc1[ 1 ] = vaddvq_s32( vmull_s16( vsrcb01, vreinterpret_s16_s64( vcoeff0.val[ 1 ] ) ) );
      vsrc1[ 2 ] = vaddvq_s32( vmull_s16( vsrcb10, vreinterpret_s16_s64( vcoeff0.val[ 1 ] ) ) );
      vsrc1[ 3 ] = vaddvq_s32( vmull_s16( vsrcb11, vreinterpret_s16_s64( vcoeff0.val[ 1 ] ) ) );

      vsrca00 = vld1_s16( &src[ ( j << 3 ) + 8 ] );
      vsrca01 = vld1_s16( &src[ ( j << 3 ) + 9 ] );
      vsrca10 = vld1_s16( &src[ ( j << 3 ) + 10 ] );
      vsrca11 = vld1_s16( &src[ ( j << 3 ) + 11 ] );

      vsrc0[ 0 ] = vaddvq_s32( vmull_s16( vsrca00, vreinterpret_s16_s64( vcoeff0.val[ 1 ] ) ) );
      vsrc0[ 1 ] = vaddvq_s32( vmull_s16( vsrca01, vreinterpret_s16_s64( vcoeff0.val[ 1 ] ) ) );
      vsrc0[ 2 ] = vaddvq_s32( vmull_s16( vsrca10, vreinterpret_s16_s64( vcoeff0.val[ 1 ] ) ) );
      vsrc0[ 3 ] = vaddvq_s32( vmull_s16( vsrca11, vreinterpret_s16_s64( vcoeff0.val[ 1 ] ) ) );

      vsuma = vaddq_s32( vsuma, vsrc1 );
      vsumb = vaddq_s32( vsumb, vsrc0 );

      vsuma = vaddq_s32( vsuma, voffset1 );
      vsumb = vaddq_s32( vsumb, voffset1 );

      vsuma = vshlq_s32( vsuma, invshift1st );
      vsumb = vshlq_s32( vsumb, invshift1st );

      vsum = vqmovn_high_s32( vqmovn_s32( vsuma ), vsumb );

      if( row < 7 )
      {
        vsrcv[ j ][ row + 1 ] = (int32x4_t) vsum;
      }
      else
      {
        vsrcv[ j ][ 8 ] = (int32x4_t) vsum;
        vsuma = vsumb = vdupq_n_s32( offset2nd );

        for( int i = 0; i < 8; i += 2 )
        {
          vsrc0               = vsrcv[ j ][ i + 1 ];
          vsrc1               = vsrcv[ j ][ i + 2 ];
          int16x4_t vsrc0l    = vget_low_s16( (int16x8_t) vsrc0 );               // 0a 0b 0c 0d
          int16x4_t vsrc0h    = vget_high_s16( (int16x8_t) vsrc0 );              // 0e 0f 0g 0h
          int16x4_t vsrc1l    = vget_low_s16( (int16x8_t) vsrc1 );               // 1a 1b 1c 1d
          int16x4_t vsrc1h    = vget_high_s16( (int16x8_t) vsrc1 );              // 1e 1f 1g 1h
          vsuma               = vmlal_n_s16( vsuma, vsrc0l, coeffV[ i ] );       // 0a * c0 + offset2nd, 0b * c0 + offset2nd, ...
          vsuma               = vmlal_n_s16( vsuma, vsrc1l, coeffV[ i + 1 ] );   // 1a * c1 + 0a * c1 + offset2nd, 1b * c1 + 0b * c0 + offset2nd, ...
          vsumb               = vmlal_n_s16( vsumb, vsrc0h, coeffV[ i ] );
          vsumb               = vmlal_n_s16( vsumb, vsrc1h, coeffV[ i + 1 ] );
          vsrcv[ j ][ i ]     = vsrc0;
          vsrcv[ j ][ i + 1 ] = vsrc1;
        }
        vsuma = vshlq_s32( vsuma, invshift2nd );
        vsumb = vshlq_s32( vsumb, invshift2nd );

        vsum = vqmovn_high_s32( vqmovn_s32( vsuma ), vsumb );

        if( isLast )   // clip
        {
          vsum = vminq_s16( vibdimax, vmaxq_s16( vibdimin, vsum ) );
        }

        vst1q_s16( &dst[ j << 3 ], vsum );

        INCY( dst, j * dstStride );
      }
    }

    INCY( src, srcStride );
  }
}

template<ARM_VEXT vext>
void InterpolationFilter::_initInterpolationFilterARM()
{
  m_filter16x16[ 0 ][ 0 ] = simdFilter16xX_N8<vext, false>;
  m_filter16x16[ 0 ][ 1 ] = simdFilter16xX_N8<vext, true>;

  m_filterN2_2D = simdInterpolateN2_2D<vext>;
}

#else    //  !__ARM_ARCH >= 8

template<ARM_VEXT vext>
void InterpolationFilter::_initInterpolationFilterARM()
{}

#endif   //  !__ARM_ARCH >= 8

template void InterpolationFilter::_initInterpolationFilterARM<SIMDARM>();

#endif   // #ifdef TARGET_SIMD_ARM

}   // namespace vvdec

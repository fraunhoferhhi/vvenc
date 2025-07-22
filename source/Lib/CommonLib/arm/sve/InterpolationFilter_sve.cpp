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
 * \file InterpolationFilter_sve.cpp
 * \brief SVE implementation of InterpolationFilter for AArch64.
 */
//  ====================================================================================================================
//  Includes
//  ====================================================================================================================
#include "../InterpolationFilter_neon.h"
#include "../CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/InterpolationFilter.h"
#include "neon_sve_bridge.h"
#include "../neon/transpose_neon.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCIF

namespace vvenc
{

static inline int16x4_t filter_horiz_4x1_N8_sve( Pel const* src, int16x8_t ch, int32x4_t voffset1,
                                                 int32x4_t invshift1st )
{
  int16x8_t vsrc0 = vld1q_s16( src + 0 );
  int16x8_t vsrc1 = vld1q_s16( src + 1 );
  int16x8_t vsrc2 = vld1q_s16( src + 2 );
  int16x8_t vsrc3 = vld1q_s16( src + 3 );

  int64x2_t vsum0 = vvenc_sdotq_s16( vdupq_n_s64( 0 ), vsrc0, ch );
  int64x2_t vsum1 = vvenc_sdotq_s16( vdupq_n_s64( 0 ), vsrc1, ch );
  int64x2_t vsum2 = vvenc_sdotq_s16( vdupq_n_s64( 0 ), vsrc2, ch );
  int64x2_t vsum3 = vvenc_sdotq_s16( vdupq_n_s64( 0 ), vsrc3, ch );

  int64x2_t vsum01 = vpaddq_s64( vsum0, vsum1 );
  int64x2_t vsum23 = vpaddq_s64( vsum2, vsum3 );

  int32x4_t vsum = vcombine_s32( vmovn_s64( vsum01 ), vmovn_s64( vsum23 ) );
  vsum = vaddq_s32( vsum, voffset1 );
  vsum = vshlq_s32( vsum, invshift1st );

  return vqmovn_s32( vsum );
}

static inline int16x8_t filter_horiz_8x1_N8_sve( Pel const* src, int16x8_t ch, int32x4_t voffset1,
                                                 int32x4_t invshift1st )
{
  int16x4_t lo = filter_horiz_4x1_N8_sve( src + 0, ch, voffset1, invshift1st );
  int16x4_t hi = filter_horiz_4x1_N8_sve( src + 4, ch, voffset1, invshift1st );
  return vcombine_s16( lo, hi );
}

static inline int16x8x2_t filter_horiz_16x1_N8_sve( Pel const* src, int16x8_t ch, int32x4_t voffset1,
                                                    int32x4_t invshift1st )
{
  int16x8x2_t result;
  result.val[0] = filter_horiz_8x1_N8_sve( src + 0, ch, voffset1, invshift1st );
  result.val[1] = filter_horiz_8x1_N8_sve( src + 8, ch, voffset1, invshift1st );
  return result;
}

template<bool isLast>
void simdFilter4x4_N6_sve( const ClpRng& clpRng, Pel const* src, int srcStride, Pel* dst, int dstStride, int width,
                           int height, TFilterCoeff const* coeffH, TFilterCoeff const* coeffV )
{
  CHECKD( width != 4, "Width must be 4" );
  CHECKD( height != 4, "Height must be 4" );
  CHECKD( IF_INTERNAL_PREC - clpRng.bd < 2, "Bit depth headroom must be at least 2" );

  OFFSET( src, srcStride, -3, -2 );

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
  vsrcv[0] = filter_horiz_4x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[1] = filter_horiz_4x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[2] = filter_horiz_4x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[3] = filter_horiz_4x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[4] = filter_horiz_4x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[5] = filter_horiz_4x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[6] = filter_horiz_4x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[7] = filter_horiz_4x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[8] = filter_horiz_4x1_N8_sve( src, ch, voffset1, invshift1st );

  int32x4_t vsum0 = filter_vert_4x1_N6_neon( vsrcv + 0, cv, voffset2 );
  int32x4_t vsum1 = filter_vert_4x1_N6_neon( vsrcv + 1, cv, voffset2 );
  int32x4_t vsum2 = filter_vert_4x1_N6_neon( vsrcv + 2, cv, voffset2 );
  int32x4_t vsum3 = filter_vert_4x1_N6_neon( vsrcv + 3, cv, voffset2 );

  int16x4_t d0, d1, d2, d3;
  if( isLast ) // clip
  {
    uint16x4_t usum0 = vqmovun_s32( vshlq_s32( vsum0, invshift2nd ) );
    uint16x4_t usum1 = vqmovun_s32( vshlq_s32( vsum1, invshift2nd ) );
    uint16x4_t usum2 = vqmovun_s32( vshlq_s32( vsum2, invshift2nd ) );
    uint16x4_t usum3 = vqmovun_s32( vshlq_s32( vsum3, invshift2nd ) );

    d0 = vmin_s16( vibdimax, vreinterpret_s16_u16( usum0 ) );
    d1 = vmin_s16( vibdimax, vreinterpret_s16_u16( usum1 ) );
    d2 = vmin_s16( vibdimax, vreinterpret_s16_u16( usum2 ) );
    d3 = vmin_s16( vibdimax, vreinterpret_s16_u16( usum3 ) );
  }
  else
  {
    d0 = vqshrn_n_s32( vsum0, IF_FILTER_PREC );
    d1 = vqshrn_n_s32( vsum1, IF_FILTER_PREC );
    d2 = vqshrn_n_s32( vsum2, IF_FILTER_PREC );
    d3 = vqshrn_n_s32( vsum3, IF_FILTER_PREC );
  }

  vst1_s16( dst, d0 );
  dst += dstStride;
  vst1_s16( dst, d1 );
  dst += dstStride;
  vst1_s16( dst, d2 );
  dst += dstStride;
  vst1_s16( dst, d3 );
  dst += dstStride;
}

template<bool isLast>
void simdFilter8xH_N8_sve( const ClpRng& clpRng, Pel const* src, int srcStride, Pel* dst, int dstStride, int width,
                           int height, TFilterCoeff const* coeffH, TFilterCoeff const* coeffV )
{
  CHECKD( width != 8, "Width must be 8" );
  CHECKD( height < 4, "Height must be >= 4" );
  CHECKD( height % 4 != 0, "Height must be a multiple of 4" );
  CHECKD( IF_INTERNAL_PREC - clpRng.bd < 2, "Bit depth headroom must be at least 2" );

  OFFSET( src, srcStride, -3, -3 );

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
  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );

  int16x8_t ch = vld1q_s16( coeffH );
  int16x8_t cv = vld1q_s16( coeffV );

  int32x4_t invshift1st = vdupq_n_s32( -shift1st );
  int32x4_t invshift2nd = vdupq_n_s32( -shift2nd );

  int16x8_t vsrcv[11];
  vsrcv[0] = filter_horiz_8x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[1] = filter_horiz_8x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[2] = filter_horiz_8x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[3] = filter_horiz_8x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[4] = filter_horiz_8x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[5] = filter_horiz_8x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[6] = filter_horiz_8x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;

  do
  {
    vsrcv[7] = filter_horiz_8x1_N8_sve( src, ch, voffset1, invshift1st );
    src += srcStride;
    vsrcv[8] = filter_horiz_8x1_N8_sve( src, ch, voffset1, invshift1st );
    src += srcStride;
    vsrcv[9] = filter_horiz_8x1_N8_sve( src, ch, voffset1, invshift1st );
    src += srcStride;
    vsrcv[10] = filter_horiz_8x1_N8_sve( src, ch, voffset1, invshift1st );
    src += srcStride;

    int32x4x2_t vsum0 = filter_vert_8x1_N8_neon( vsrcv + 0, cv, voffset2 );
    int32x4x2_t vsum1 = filter_vert_8x1_N8_neon( vsrcv + 1, cv, voffset2 );
    int32x4x2_t vsum2 = filter_vert_8x1_N8_neon( vsrcv + 2, cv, voffset2 );
    int32x4x2_t vsum3 = filter_vert_8x1_N8_neon( vsrcv + 3, cv, voffset2 );

    int16x8_t d0, d1, d2, d3;
    if( isLast ) // clip
    {
      int32x4_t vsum00 = vshlq_s32( vsum0.val[0], invshift2nd );
      int32x4_t vsum01 = vshlq_s32( vsum0.val[1], invshift2nd );
      int32x4_t vsum10 = vshlq_s32( vsum1.val[0], invshift2nd );
      int32x4_t vsum11 = vshlq_s32( vsum1.val[1], invshift2nd );
      int32x4_t vsum20 = vshlq_s32( vsum2.val[0], invshift2nd );
      int32x4_t vsum21 = vshlq_s32( vsum2.val[1], invshift2nd );
      int32x4_t vsum30 = vshlq_s32( vsum3.val[0], invshift2nd );
      int32x4_t vsum31 = vshlq_s32( vsum3.val[1], invshift2nd );

      uint16x8_t usum0 = vcombine_u16( vqmovun_s32( vsum00 ), vqmovun_s32( vsum01 ) );
      uint16x8_t usum1 = vcombine_u16( vqmovun_s32( vsum10 ), vqmovun_s32( vsum11 ) );
      uint16x8_t usum2 = vcombine_u16( vqmovun_s32( vsum20 ), vqmovun_s32( vsum21 ) );
      uint16x8_t usum3 = vcombine_u16( vqmovun_s32( vsum30 ), vqmovun_s32( vsum31 ) );

      d0 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum0 ) );
      d1 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum1 ) );
      d2 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum2 ) );
      d3 = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum3 ) );
    }
    else
    {
      int16x4_t t00 = vqshrn_n_s32( vsum0.val[0], IF_FILTER_PREC );
      int16x4_t t01 = vqshrn_n_s32( vsum0.val[1], IF_FILTER_PREC );
      d0 = vcombine_s16( t00, t01 );

      int16x4_t t10 = vqshrn_n_s32( vsum1.val[0], IF_FILTER_PREC );
      int16x4_t t11 = vqshrn_n_s32( vsum1.val[1], IF_FILTER_PREC );
      d1 = vcombine_s16( t10, t11 );

      int16x4_t t20 = vqshrn_n_s32( vsum2.val[0], IF_FILTER_PREC );
      int16x4_t t21 = vqshrn_n_s32( vsum2.val[1], IF_FILTER_PREC );
      d2 = vcombine_s16( t20, t21 );

      int16x4_t t30 = vqshrn_n_s32( vsum3.val[0], IF_FILTER_PREC );
      int16x4_t t31 = vqshrn_n_s32( vsum3.val[1], IF_FILTER_PREC );
      d3 = vcombine_s16( t30, t31 );
    }

    vst1q_s16( dst, d0 );
    dst += dstStride;
    vst1q_s16( dst, d1 );
    dst += dstStride;
    vst1q_s16( dst, d2 );
    dst += dstStride;
    vst1q_s16( dst, d3 );
    dst += dstStride;

    vsrcv[0] = vsrcv[4];
    vsrcv[1] = vsrcv[5];
    vsrcv[2] = vsrcv[6];
    vsrcv[3] = vsrcv[7];
    vsrcv[4] = vsrcv[8];
    vsrcv[5] = vsrcv[9];
    vsrcv[6] = vsrcv[10];

    height -= 4;
  } while( height != 0 );
}

template<bool isLast>
void simdFilter16xH_N8_sve( const ClpRng& clpRng, Pel const* src, int srcStride, Pel* dst, int dstStride, int width,
                            int height, TFilterCoeff const* coeffH, TFilterCoeff const* coeffV )
{
  CHECKD( width != 16, "Width must be 16" );
  CHECKD( height < 4, "Height must be >= 4" );
  CHECKD( height % 4 != 0, "Height must be a multiple of 4" );
  CHECKD( IF_INTERNAL_PREC - clpRng.bd < 2, "Bit depth headroom must be at least 2" );

  OFFSET( src, srcStride, -3, -3 );

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
  const int16x8_t vibdimax = vdupq_n_s16( clpRng.max() );

  int16x8_t ch = vld1q_s16( coeffH );
  int16x8_t cv = vld1q_s16( coeffV );

  int32x4_t invshift1st = vdupq_n_s32( -shift1st );
  int32x4_t invshift2nd = vdupq_n_s32( -shift2nd );

  int16x8x2_t vsrcv[8];
  vsrcv[0] = filter_horiz_16x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[1] = filter_horiz_16x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[2] = filter_horiz_16x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[3] = filter_horiz_16x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[4] = filter_horiz_16x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[5] = filter_horiz_16x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;
  vsrcv[6] = filter_horiz_16x1_N8_sve( src, ch, voffset1, invshift1st );
  src += srcStride;

  do
  {
    vsrcv[7] = filter_horiz_16x1_N8_sve( src, ch, voffset1, invshift1st );
    src += srcStride;

    int32x4x4_t vsum0 = filter_vert_16x1_N8_neon( vsrcv, cv, voffset2 );

    int16x8_t d0_lo, d0_hi;
    if( isLast ) // clip
    {
      int32x4_t vsum00 = vshlq_s32( vsum0.val[0], invshift2nd );
      int32x4_t vsum01 = vshlq_s32( vsum0.val[1], invshift2nd );
      int32x4_t vsum02 = vshlq_s32( vsum0.val[2], invshift2nd );
      int32x4_t vsum03 = vshlq_s32( vsum0.val[3], invshift2nd );

      uint16x8_t usum0_lo = vcombine_u16( vqmovun_s32( vsum00 ), vqmovun_s32( vsum01 ) );
      uint16x8_t usum0_hi = vcombine_u16( vqmovun_s32( vsum02 ), vqmovun_s32( vsum03 ) );

      d0_lo = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum0_lo ) );
      d0_hi = vminq_s16( vibdimax, vreinterpretq_s16_u16( usum0_hi ) );
    }
    else
    {
      int16x4_t t00 = vqshrn_n_s32( vsum0.val[0], IF_FILTER_PREC );
      int16x4_t t01 = vqshrn_n_s32( vsum0.val[1], IF_FILTER_PREC );
      d0_lo = vcombine_s16( t00, t01 );

      int16x4_t t02 = vqshrn_n_s32( vsum0.val[2], IF_FILTER_PREC );
      int16x4_t t03 = vqshrn_n_s32( vsum0.val[3], IF_FILTER_PREC );
      d0_hi = vcombine_s16( t02, t03 );
    }

    vst1q_s16( dst + 0, d0_lo );
    vst1q_s16( dst + 8, d0_hi );
    dst += dstStride;

    vsrcv[0] = vsrcv[1];
    vsrcv[1] = vsrcv[2];
    vsrcv[2] = vsrcv[3];
    vsrcv[3] = vsrcv[4];
    vsrcv[4] = vsrcv[5];
    vsrcv[5] = vsrcv[6];
    vsrcv[6] = vsrcv[7];

  } while( --height != 0 );
}

template<>
void InterpolationFilter::_initInterpolationFilterARM<SVE>()
{
  m_filter4x4[0][0] = simdFilter4x4_N6_sve<false>;
  m_filter4x4[0][1] = simdFilter4x4_N6_sve<true>;

  m_filter8xH[0][0] = simdFilter8xH_N8_sve<false>;
  m_filter8xH[0][1] = simdFilter8xH_N8_sve<true>;

  m_filter16xH[0][0] = simdFilter16xH_N8_sve<false>;
  m_filter16xH[0][1] = simdFilter16xH_N8_sve<true>;
}

} // namespace vvenc
#endif
//! \}

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
/** \file     InterpolationFilter4x4X86.h
    \brief    SIMD bodies of the 4x4 6-tap luma interpolation filter, shared by InterpolationFilterX86.h (the
              m_filter4x4 kernels) and InterPredX86.h (the per-CU affine luma loop). Only inline functions live here,
              so both translation units can include it.
*/

#pragma once

#include "CommonDefX86.h"
#include "InterpolationFilter.h"
#include <type_traits>

#if defined(TARGET_SIMD_X86)

//! \ingroup CommonLib
//! \{

namespace vvenc {

// The shift of a pass of simdFilter (and of the fused 2-D kernels: first pass isFirst && !isLast, second pass
// !isFirst): a function of the internal bit depth and the pass only. The hot kernels take it as an immediate, a
// template argument, instantiated per bit depth: a vector shift by a count in a register is 2 uops on Intel cores,
// by an immediate 1 (and frees the register).
static constexpr int ifHeadRoom( int bitDepth )
{
  return std::max<int>( 2, IF_INTERNAL_PREC - bitDepth );
}

static constexpr int ifFilterShift( int N, bool isFirst, bool isLast, int bitDepth )
{
  return N == 2 ? ( isFirst ? IF_FILTER_PREC_BILINEAR - ( IF_INTERNAL_PREC_BILINEAR - bitDepth ) : 4 )
       : isLast ? IF_FILTER_PREC + ( isFirst ? 0 : ifHeadRoom( bitDepth ) )
                : IF_FILTER_PREC - ( isFirst ? ifHeadRoom( bitDepth ) : 0 );
}

static_assert( ifFilterShift( 8, true,  false, 10 ) ==  2 && ifFilterShift( 8, false, true,  10 ) == 10, "10-bit shifts" );
static_assert( ifFilterShift( 8, true,  false,  8 ) ==  0 && ifFilterShift( 8, false, true,   8 ) == 12, "8-bit shifts" );
static_assert( ifFilterShift( 8, true,  true,   8 ) ==  6 && ifFilterShift( 8, false, false,  8 ) ==  6, "bit-depth independent shifts" );

// Calls f with std::integral_constant<int, internal bit depth>: VVenC runs at 8 or 10 bit (vvencCfg checks it).
template<typename Fn>
static ALWAYS_INLINE void dispatchBitDepth( int bitDepth, Fn&& f )
{
  if( bitDepth == 10 )
  {
    f( std::integral_constant<int, 10>() );
  }
  else
  {
    CHECK( bitDepth != 8, "the x86 interpolation kernels support internal bit depths 8 and 10" );
    f( std::integral_constant<int, 8>() );
  }
}

// The rounding of the two filter stages of simdFilter4x4_N6: the horizontal stage keeps headroom bits, the vertical
// stage either removes them and clips (isLast) or leaves the 14-bit intermediate format.
struct Filter4x4N6Rounding
{
  int offset1st, shift1st, offset2nd, shift2nd;
};

#if USE_AVX2
// The pack and clip of a last pass: packus saturates the 32-bit values to [0, 65535], min_epu16 clips to the maximum
// sample. For every 32-bit input that equals packs_epi32 followed by the clip to [0, max] (packus orders the lanes as
// packs does), as the minimum of every ClpRng is 0.
static_assert( ClpRng::min() == 0, "the unsigned clip of the last passes needs a zero minimum" );

static ALWAYS_INLINE __m256i packClipLast_AVX2( const __m256i& lo, const __m256i& hi, const __m256i& vmax )
{
  return _mm256_min_epu16( vmax, _mm256_packus_epi32( lo, hi ) );
}
#endif

// A constant expression of the bit depth: the callers instantiate per internal bit depth, so the shifts are immediates.
template<bool isLast>
static constexpr Filter4x4N6Rounding filter4x4N6Rounding( int bitDepth )
{
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20
  const int headRoom = std::max<int>( 2, ( IF_INTERNAL_PREC - bitDepth ) );
  Filter4x4N6Rounding r{ 0, 0, 0, 0 };
  r.shift1st  = IF_FILTER_PREC - headRoom;
  r.offset1st = -( IF_INTERNAL_OFFS << r.shift1st );
  if( isLast )
  {
    r.shift2nd  = IF_FILTER_PREC + headRoom;
    r.offset2nd = ( 1 << ( r.shift2nd - 1 ) ) + ( IF_INTERNAL_OFFS << IF_FILTER_PREC );
  }
  else
  {
    r.shift2nd  = IF_FILTER_PREC;
    r.offset2nd = 0;
  }
  return r;
}

#if USE_AVX2
// A pair of 16-bit filter coefficients as one 32-bit value, for _mm256_madd_epi16 with a pair of neighbouring values.
static inline int xCoeffPair( TFilterCoeff first, TFilterCoeff second )
{
  return int( uint16_t( first ) | ( uint32_t( uint16_t( second ) ) << 16 ) );
}

// The 4x4 block of simdFilter4x4_N6 with AVX2. src points two rows above and two samples left of the block. The
// filters have 6 taps (coefficients 0 and 7 are zero for every 4x4 luma filter), and the horizontal results fit in
// 16 bits for bit depths up to 12 (as the previous code also assumed), so the vertical filter runs on 16-bit values
// like the horizontal one. The results are exactly those of the C version.
template<bool isLast>
static ALWAYS_INLINE void simdFilter4x4_N6_AVX2( const ClpRng& clpRng, const Pel* src, int srcStride, Pel* dst, int dstStride,
                                                 const TFilterCoeff* coeffH, const TFilterCoeff* coeffV,
                                                 int offset1st, int shift1st, int offset2nd, int shift2nd )
{
  const __m256i h12 = _mm256_set1_epi32( xCoeffPair( coeffH[1], coeffH[2] ) );
  const __m256i h34 = _mm256_set1_epi32( xCoeffPair( coeffH[3], coeffH[4] ) );
  const __m256i h56 = _mm256_set1_epi32( xCoeffPair( coeffH[5], coeffH[6] ) );
  const __m256i off = _mm256_set1_epi32( offset1st );

  // Horizontal filter of the 9 rows the 4 output rows need, two rows per register, one per 128-bit lane: rows[i]
  // holds the 4 results of rows 2i and 2i + 1 as 32-bit values (row 8 goes into both lanes of rows[4], as there is
  // no row 9 to read). Output x takes samples x to x + 5, as the pairs (x, x + 1), (x + 2, x + 3) and (x + 4, x + 5).
  __m256i rows[5];
  for( int i = 0; i < 5; i++ )
  {
    const Pel*    row  = src + 2 * i * srcStride;
    const Pel*    next = src + std::min( 2 * i + 1, 8 ) * srcStride;
    const __m256i from0 = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i* ) row ) ),
                                                   _mm_loadu_si128( ( const __m128i* ) next ), 1 );                  // samples 0..7
    const __m256i from1 = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i* ) ( row + 1 ) ) ),
                                                   _mm_loadu_si128( ( const __m128i* ) ( next + 1 ) ), 1 );          // samples 1..8
    const __m256i samplePairs0 = _mm256_unpacklo_epi16( from0, from1 );                  // (0,1) (1,2) (2,3) (3,4)
    const __m256i samplePairs4 = _mm256_unpackhi_epi16( from0, from1 );                  // (4,5) (5,6) (6,7) (7,8)
    const __m256i samplePairs2 = _mm256_alignr_epi8( samplePairs4, samplePairs0, 8 );    // (2,3) (3,4) (4,5) (5,6)
    __m256i sum = _mm256_madd_epi16( samplePairs0, h12 );
    sum = _mm256_add_epi32( sum, _mm256_madd_epi16( samplePairs2, h34 ) );
    sum = _mm256_add_epi32( sum, _mm256_madd_epi16( samplePairs4, h56 ) );
    rows[i] = _mm256_srai_epi32( _mm256_add_epi32( sum, off ), shift1st );
  }

  // Vertical filter. Output row y takes rows y to y + 5 as the pairs (y, y + 1), (y + 2, y + 3) and (y + 4, y + 5).
  // rowPairs[i] holds rows 2i and 2i + 1 interleaved in its low lane and rows 2i + 1 and 2i + 2 in its high lane, so
  // one multiply-add serves two output rows at once.
  const __m256i interleave = _mm256_setr_epi8( 0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15,
                                               0, 1, 8, 9, 2, 3, 10, 11, 4, 5, 12, 13, 6, 7, 14, 15 );
  __m256i rowPairs[4];
  for( int i = 0; i < 4; i++ )
  {
    // the pack gives the 64-bit chunks (rows 2i, 2i + 2, 2i + 1, 2i + 3); reorder them to (2i, 2i + 1, 2i + 1, 2i + 2)
    const __m256i packed = _mm256_packs_epi32( rows[i], rows[i + 1] );
    rowPairs[i] = _mm256_shuffle_epi8( _mm256_permute4x64_epi64( packed, 0 + ( 2 << 2 ) + ( 2 << 4 ) + ( 1 << 6 ) ), interleave );
  }

  const __m256i v12  = _mm256_set1_epi32( xCoeffPair( coeffV[1], coeffV[2] ) );
  const __m256i v34  = _mm256_set1_epi32( xCoeffPair( coeffV[3], coeffV[4] ) );
  const __m256i v56  = _mm256_set1_epi32( xCoeffPair( coeffV[5], coeffV[6] ) );
  const __m256i off2 = _mm256_set1_epi32( offset2nd );

  __m256i out01 = _mm256_add_epi32( off2, _mm256_madd_epi16( rowPairs[0], v12 ) );
  out01 = _mm256_add_epi32( out01, _mm256_madd_epi16( rowPairs[1], v34 ) );
  out01 = _mm256_add_epi32( out01, _mm256_madd_epi16( rowPairs[2], v56 ) );
  out01 = _mm256_srai_epi32( out01, shift2nd );

  __m256i out23 = _mm256_add_epi32( off2, _mm256_madd_epi16( rowPairs[1], v12 ) );
  out23 = _mm256_add_epi32( out23, _mm256_madd_epi16( rowPairs[2], v34 ) );
  out23 = _mm256_add_epi32( out23, _mm256_madd_epi16( rowPairs[3], v56 ) );
  out23 = _mm256_srai_epi32( out23, shift2nd );

  // the pack gives the 64-bit chunks (rows 0, 2, 1, 3)
  const __m256i out   = isLast ? packClipLast_AVX2( out01, out23, _mm256_set1_epi16( clpRng.max() ) ) : _mm256_packs_epi32( out01, out23 );
  const __m128i out02 = _mm256_castsi256_si128( out );
  const __m128i out13 = _mm256_extracti128_si256( out, 1 );
  _vv_storel_epi64( ( __m128i* ) ( dst                 ), out02 );
  _vv_storel_epi64( ( __m128i* ) ( dst +     dstStride ), out13 );
  _vv_storel_epi64( ( __m128i* ) ( dst + 2 * dstStride ), _mm_unpackhi_epi64( out02, out02 ) );
  _vv_storel_epi64( ( __m128i* ) ( dst + 3 * dstStride ), _mm_unpackhi_epi64( out13, out13 ) );
}
#endif

// The 4x4 block of simdFilter4x4_N6 without AVX2, one row at a time. src points two rows above and two samples left
// of the block. prefetch keeps the m_filter4x4 kernel's prefetch of the next rows; the per-CU affine loop goes
// without it.
template<bool isLast, bool prefetch>
static ALWAYS_INLINE void simdFilter4x4_N6_SSE( const ClpRng& clpRng, const Pel* src, int srcStride, Pel* dst, int dstStride,
                                                const TFilterCoeff* coeffH, const TFilterCoeff* _coeffV,
                                                int offset1st, int shift1st, int offset2nd, int shift2nd )
{
  __m128i _dst0x;
  __m128i _dst1x;
  __m128i _dst2x;
  __m128i _dst3x;
  __m128i zerox = _mm_setzero_si128();

  {
    ALIGN_DATA( 64, TFilterCoeff coeffV[17] ) = { 0, 0, 0, _coeffV[6], _coeffV[5], _coeffV[4], _coeffV[3], _coeffV[2], _coeffV[1], 0, 0, 0, 0, 0, 0 };
    //ALIGN_DATA( 64, TFilterCoeff coeffV[17] ) = { 0, 0, 0, _coeffV[1], _coeffV[2], _coeffV[3], _coeffV[4], _coeffV[5], _coeffV[6], 0, 0, 0, 0, 0, 0,0,0 };

    __m128i cH    = _mm_loadu_si128  ( ( const __m128i* ) ( coeffH + 1 ) );
            cH    = _mm_blend_epi16  ( cH, _mm_setzero_si128(), 0x80 );
    __m128i off   = _mm_set1_epi32   ( offset1st );
    __m128i _src1, _src2, _srcx, cV;

    _dst0x = _mm_set1_epi32( offset2nd );
    _dst1x = _mm_set1_epi32( offset2nd );
    _dst2x = _mm_set1_epi32( offset2nd );
    _dst3x = _mm_set1_epi32( offset2nd );

    for( int row = 0; row < 9; row++ )
    {
      if( prefetch )
      {
        _mm_prefetch( ( const char* ) ( src + 2 * srcStride ), _MM_HINT_T0 );
        _mm_prefetch( ( const char* ) ( src + 3 * srcStride ), _MM_HINT_T0 );
      }

      cV = _vv_loadl_epi64    ( ( const __m128i* ) ( coeffV + 9 - row - 1 ) );
      cV = _mm_cvtepu16_epi32 ( cV );

      _src1 = _mm_loadu_si128      ( ( const __m128i* ) &src[0] );
      _src1 = _mm_madd_epi16       ( _src1, cH );

      _src2 = _mm_loadu_si128      ( ( const __m128i* ) &src[1] );
      _src2 = _mm_madd_epi16       ( _src2, cH );
      _srcx = _mm_hadd_epi32       ( _src1, _src2 );

      _src1 = _mm_loadu_si128      ( ( const __m128i* ) &src[2] );
      _src1 = _mm_madd_epi16       ( _src1, cH );

      _src2 = _mm_loadu_si128      ( ( const __m128i* ) &src[3] );
      _src2 = _mm_madd_epi16       ( _src2, cH );
      _src2 = _mm_hadd_epi32       ( _src1, _src2 );

      _src2 = _mm_hadd_epi32       ( _srcx, _src2 );
      _src2 = _mm_add_epi32        ( _src2, off );
      _src2 = _mm_srai_epi32       ( _src2, shift1st );

      _src1 = _mm_shuffle_epi32    ( _src2, 0 );
      _src1 = _mm_madd_epi16       ( _src1, cV );
      _dst0x = _mm_add_epi32       ( _src1, _dst0x );

      _src1 = _mm_shuffle_epi32    ( _src2, 1 + 4 + 16 + 64 );
      _src1 = _mm_madd_epi16       ( _src1, cV );
      _dst1x = _mm_add_epi32       ( _src1, _dst1x );

      _src1 = _mm_shuffle_epi32    ( _src2, 2 + 8 + 32 + 128 );
      _src1 = _mm_madd_epi16       ( _src1, cV );
      _dst2x = _mm_add_epi32       ( _src1, _dst2x );

      _src1 = _mm_shuffle_epi32    ( _src2, 3 + 12 + 48 + 192 );
      _src1 = _mm_madd_epi16       ( _src1, cV );
      _dst3x = _mm_add_epi32       ( _src1, _dst3x );

      src += srcStride;
    }

    _dst0x = _mm_srai_epi32( _dst0x, shift2nd );
    _dst1x = _mm_srai_epi32( _dst1x, shift2nd );
    _dst2x = _mm_srai_epi32( _dst2x, shift2nd );
    _dst3x = _mm_srai_epi32( _dst3x, shift2nd );

    if( isLast )
    {
      __m128i vmin = _mm_set1_epi32( clpRng.min() );
      __m128i vmax = _mm_set1_epi32( clpRng.max() );

      _dst0x = _mm_max_epi32( _mm_min_epi32( _dst0x, vmax ), vmin );
      _dst1x = _mm_max_epi32( _mm_min_epi32( _dst1x, vmax ), vmin );
      _dst2x = _mm_max_epi32( _mm_min_epi32( _dst2x, vmax ), vmin );
      _dst3x = _mm_max_epi32( _mm_min_epi32( _dst3x, vmax ), vmin );
    }
  }

  __m128i a01b01 = _mm_unpacklo_epi32( _dst0x, _dst1x );
  __m128i a23b23 = _mm_unpackhi_epi32( _dst0x, _dst1x );
  __m128i c01d01 = _mm_unpacklo_epi32( _dst2x, _dst3x );
  __m128i c23d23 = _mm_unpackhi_epi32( _dst2x, _dst3x );

  _dst0x = _mm_unpacklo_epi64( a01b01, c01d01 );
  _dst1x = _mm_unpackhi_epi64( a01b01, c01d01 );
  _dst2x = _mm_unpacklo_epi64( a23b23, c23d23 );
  _dst3x = _mm_unpackhi_epi64( a23b23, c23d23 );

  _dst0x = _mm_packs_epi32( _dst0x, zerox );
  _dst1x = _mm_packs_epi32( _dst1x, zerox );
  _dst2x = _mm_packs_epi32( _dst2x, zerox );
  _dst3x = _mm_packs_epi32( _dst3x, zerox );

  _vv_storel_epi64( ( __m128i* ) ( dst                 ), _dst0x );
  _vv_storel_epi64( ( __m128i* ) ( dst +     dstStride ), _dst1x );
  _vv_storel_epi64( ( __m128i* ) ( dst + 2 * dstStride ), _dst2x );
  _vv_storel_epi64( ( __m128i* ) ( dst + 3 * dstStride ), _dst3x );
}

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86

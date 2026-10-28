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
/**
 * \file
 * \brief SIMD kernels of the CU-tree look-ahead (EncoderLib/CuTree.cpp)
 */

#include "CommonDefX86.h"
#include "EncoderLib/CuTree.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT

namespace vvenc {

#ifdef USE_AVX2

// The 8-point Hadamard butterflies across eight registers, one per row: transforms the columns.
static inline void hadamard8x8Columns( __m256i* v )
{
  const __m256i a0 = _mm256_add_epi32( v[0], v[4] ), a1 = _mm256_add_epi32( v[1], v[5] );
  const __m256i a2 = _mm256_add_epi32( v[2], v[6] ), a3 = _mm256_add_epi32( v[3], v[7] );
  const __m256i a4 = _mm256_sub_epi32( v[0], v[4] ), a5 = _mm256_sub_epi32( v[1], v[5] );
  const __m256i a6 = _mm256_sub_epi32( v[2], v[6] ), a7 = _mm256_sub_epi32( v[3], v[7] );
  const __m256i b0 = _mm256_add_epi32( a0, a2 ), b1 = _mm256_add_epi32( a1, a3 );
  const __m256i b2 = _mm256_sub_epi32( a0, a2 ), b3 = _mm256_sub_epi32( a1, a3 );
  const __m256i b4 = _mm256_add_epi32( a4, a6 ), b5 = _mm256_add_epi32( a5, a7 );
  const __m256i b6 = _mm256_sub_epi32( a4, a6 ), b7 = _mm256_sub_epi32( a5, a7 );
  v[0] = _mm256_add_epi32( b0, b1 ); v[1] = _mm256_sub_epi32( b0, b1 );
  v[2] = _mm256_add_epi32( b2, b3 ); v[3] = _mm256_sub_epi32( b2, b3 );
  v[4] = _mm256_add_epi32( b4, b5 ); v[5] = _mm256_sub_epi32( b4, b5 );
  v[6] = _mm256_add_epi32( b6, b7 ); v[7] = _mm256_sub_epi32( b6, b7 );
}

static inline void transpose8x8( __m256i* v )
{
  const __m256i t0 = _mm256_unpacklo_epi32( v[0], v[1] ), t1 = _mm256_unpackhi_epi32( v[0], v[1] );
  const __m256i t2 = _mm256_unpacklo_epi32( v[2], v[3] ), t3 = _mm256_unpackhi_epi32( v[2], v[3] );
  const __m256i t4 = _mm256_unpacklo_epi32( v[4], v[5] ), t5 = _mm256_unpackhi_epi32( v[4], v[5] );
  const __m256i t6 = _mm256_unpacklo_epi32( v[6], v[7] ), t7 = _mm256_unpackhi_epi32( v[6], v[7] );
  const __m256i u0 = _mm256_unpacklo_epi64( t0, t2 ), u1 = _mm256_unpackhi_epi64( t0, t2 );
  const __m256i u2 = _mm256_unpacklo_epi64( t1, t3 ), u3 = _mm256_unpackhi_epi64( t1, t3 );
  const __m256i u4 = _mm256_unpacklo_epi64( t4, t6 ), u5 = _mm256_unpackhi_epi64( t4, t6 );
  const __m256i u6 = _mm256_unpacklo_epi64( t5, t7 ), u7 = _mm256_unpackhi_epi64( t5, t7 );
  v[0] = _mm256_permute2x128_si256( u0, u4, 0x20 ); v[4] = _mm256_permute2x128_si256( u0, u4, 0x31 );
  v[1] = _mm256_permute2x128_si256( u1, u5, 0x20 ); v[5] = _mm256_permute2x128_si256( u1, u5, 0x31 );
  v[2] = _mm256_permute2x128_si256( u2, u6, 0x20 ); v[6] = _mm256_permute2x128_si256( u2, u6, 0x31 );
  v[3] = _mm256_permute2x128_si256( u3, u7, 0x20 ); v[7] = _mm256_permute2x128_si256( u3, u7, 0x31 );
}

// AVX2 version of tplCodeBlockCore<8>, identical results. The registers hold the transposed coefficients, which
// does not matter for the element-wise quantization and the sums, and is undone by the inverse transform.
template<X86_VEXT vext>
void tplCodeBlock8x8_SIMD( const Pel* org, ptrdiff_t orgStride, const Pel* pred, Pel* recon, ptrdiff_t reconStride,
                           const TplQuant& quant, int maxVal, int64_t& dist, int& bits )
{
  __m256i v[8];
  for( int y = 0; y < 8; y++ )
  {
    const __m256i o = _mm256_cvtepi16_epi32( _mm_loadu_si128( ( const __m128i* ) ( org + y * orgStride ) ) );
    const __m256i p = _mm256_cvtepi16_epi32( _mm_loadu_si128( ( const __m128i* ) ( pred + y * 8 ) ) );
    v[y] = _mm256_sub_epi32( o, p );
  }
  hadamard8x8Columns( v );
  transpose8x8( v );
  hadamard8x8Columns( v );

  const __m256i scale   = _mm256_set1_epi32( quant.scale );
  const __m256i round   = _mm256_set1_epi32( quant.round );
  const __m256i deqStep = _mm256_set1_epi32( quant.deqStep );
  const __m256i eight   = _mm256_set1_epi32( 8 );
  const __m256i three   = _mm256_set1_epi32( 3 );
  const __m256i bias    = _mm256_set1_epi32( 127 );
  __m256i distSum = _mm256_setzero_si256();
  __m256i bitSum  = _mm256_setzero_si256();

  for( int i = 0; i < 8; i++ )
  {
    const __m256i absCoeff = _mm256_abs_epi32( v[i] );
    const __m256i level    = _mm256_srai_epi32( _mm256_add_epi32( _mm256_mullo_epi32( absCoeff, scale ), round ), 16 );
    const __m256i absDeq   = _mm256_srai_epi32( _mm256_add_epi32( _mm256_mullo_epi32( level, deqStep ), eight ), 4 );
    const __m256i err      = _mm256_sub_epi32( absCoeff, absDeq );
    // squared errors as 64-bit sums: even lanes, then odd lanes shifted down
    distSum = _mm256_add_epi64( distSum, _mm256_mul_epi32( err, err ) );
    const __m256i errOdd = _mm256_srli_epi64( err, 32 );
    distSum = _mm256_add_epi64( distSum, _mm256_mul_epi32( errOdd, errOdd ) );
    // bits: 2 * floorLog2( level ) + 3 for non-zero levels, floorLog2 from the float exponent
    const __m256i log2    = _mm256_sub_epi32( _mm256_srli_epi32( _mm256_castps_si256( _mm256_cvtepi32_ps( level ) ), 23 ), bias );
    const __m256i nonZero = _mm256_cmpgt_epi32( level, _mm256_setzero_si256() );
    bitSum = _mm256_add_epi32( bitSum, _mm256_and_si256( nonZero, _mm256_add_epi32( _mm256_add_epi32( log2, log2 ), three ) ) );
    v[i] = _mm256_sign_epi32( absDeq, v[i] );
  }

  __m128i d = _mm_add_epi64( _mm256_castsi256_si128( distSum ), _mm256_extracti128_si256( distSum, 1 ) );
  d = _mm_add_epi64( d, _mm_unpackhi_epi64( d, d ) );
  dist = _mm_cvtsi128_si64( d );
  __m128i b = _mm_add_epi32( _mm256_castsi256_si128( bitSum ), _mm256_extracti128_si256( bitSum, 1 ) );
  b = _mm_hadd_epi32( b, b );
  b = _mm_hadd_epi32( b, b );
  bits = _mm_cvtsi128_si32( b );

  if( recon )
  {
    hadamard8x8Columns( v );
    transpose8x8( v );
    hadamard8x8Columns( v );
    const __m256i r32  = _mm256_set1_epi32( 32 );
    const __m128i vmax = _mm_set1_epi16( ( short ) maxVal );
    for( int y = 0; y < 8; y++ )
    {
      const __m256i p   = _mm256_cvtepi16_epi32( _mm_loadu_si128( ( const __m128i* ) ( pred + y * 8 ) ) );
      const __m256i sum = _mm256_add_epi32( p, _mm256_srai_epi32( _mm256_add_epi32( v[y], r32 ), 6 ) );
      __m128i packed    = _mm_packs_epi32( _mm256_castsi256_si128( sum ), _mm256_extracti128_si256( sum, 1 ) );
      packed            = _mm_min_epi16( _mm_max_epi16( packed, _mm_setzero_si128() ), vmax );
      _mm_storeu_si128( ( __m128i* ) ( recon + y * reconStride ), packed );
    }
  }
}

// AVX2 version of tplPredBlockCore<8>, identical results: horizontal taps in 16 bits (at most 1023 * 16), the
// vertical taps as 32-bit multiply-adds of interleaved row pairs.
template<X86_VEXT vext>
void tplPredBlock8x8_SIMD( const Pel* src, ptrdiff_t srcStride, int fx, int fy, Pel* dst )
{
  const __m128i wx0 = _mm_set1_epi16( short( 16 - fx ) );
  const __m128i wx1 = _mm_set1_epi16( short( fx ) );
  const __m256i wy  = _mm256_set1_epi32( ( fy << 16 ) | ( 16 - fy ) );
  const __m256i rnd = _mm256_set1_epi32( 128 );

  __m128i rows[9];
  for( int j = 0; j < 9; j++ )
  {
    const __m128i a = _mm_loadu_si128( ( const __m128i* ) ( src + j * srcStride ) );
    const __m128i b = _mm_loadu_si128( ( const __m128i* ) ( src + j * srcStride + 1 ) );
    rows[j] = _mm_add_epi16( _mm_mullo_epi16( a, wx0 ), _mm_mullo_epi16( b, wx1 ) );
  }
  for( int j = 0; j < 8; j++ )
  {
    // ( top, bottom ) pairs, multiplied by ( 16 - fy, fy ) and summed in 32 bits
    const __m256i pairs = _mm256_set_m128i( _mm_unpackhi_epi16( rows[j], rows[j + 1] ), _mm_unpacklo_epi16( rows[j], rows[j + 1] ) );
    const __m256i sum   = _mm256_srai_epi32( _mm256_add_epi32( _mm256_madd_epi16( pairs, wy ), rnd ), 8 );
    _mm_storeu_si128( ( __m128i* ) ( dst + j * 8 ), _mm_packs_epi32( _mm256_castsi256_si128( sum ), _mm256_extracti128_si256( sum, 1 ) ) );
  }
}

// The 16-point Hadamard butterflies of the columns of a 16x16 block held as 8x8 quadrants, eight registers each:
// one stage across the top and bottom quadrant, then the 8-point transform of each.
static inline void hadamard16Columns( __m256i* top, __m256i* bottom )
{
  for( int i = 0; i < 8; i++ )
  {
    const __m256i a = top[i], b = bottom[i];
    top[i]    = _mm256_add_epi32( a, b );
    bottom[i] = _mm256_sub_epi32( a, b );
  }
  hadamard8x8Columns( top );
  hadamard8x8Columns( bottom );
}

// The 2-D transform of a 16x16 block in quadrants q[0] top left, q[1] top right, q[2] bottom left, q[3] bottom right:
// columns, transposition (each quadrant, with the off-diagonal quadrants swapped), columns again. The result is the
// transposed coefficients; applying it twice gives 256 times the input in its original order.
static inline void hadamard16x16( __m256i q[4][8] )
{
  hadamard16Columns( q[0], q[2] );
  hadamard16Columns( q[1], q[3] );
  for( int k = 0; k < 4; k++ ) transpose8x8( q[k] );
  for( int i = 0; i < 8; i++ ) std::swap( q[1][i], q[2][i] );
  hadamard16Columns( q[0], q[2] );
  hadamard16Columns( q[1], q[3] );
}

// AVX2 version of tplCodeBlockCore<16>, identical results, with the coefficients transposed as in the 8x8 version.
template<X86_VEXT vext>
void tplCodeBlock16x16_SIMD( const Pel* org, ptrdiff_t orgStride, const Pel* pred, Pel* recon, ptrdiff_t reconStride,
                             const TplQuant& quant, int maxVal, int64_t& dist, int& bits )
{
  __m256i q[4][8];
  for( int y = 0; y < 16; y++ )
  {
    const __m256i o = _mm256_loadu_si256( ( const __m256i* ) ( org + y * orgStride ) );
    const __m256i p = _mm256_loadu_si256( ( const __m256i* ) ( pred + y * 16 ) );
    const __m256i r = _mm256_sub_epi16( o, p );   // residuals fit 16 bits
    __m256i* left   = y < 8 ? q[0] : q[2];
    __m256i* right  = y < 8 ? q[1] : q[3];
    left [y & 7] = _mm256_cvtepi16_epi32( _mm256_castsi256_si128( r ) );
    right[y & 7] = _mm256_cvtepi16_epi32( _mm256_extracti128_si256( r, 1 ) );
  }
  hadamard16x16( q );

  const __m256i scale   = _mm256_set1_epi32( quant.scale );
  const __m256i round   = _mm256_set1_epi32( quant.round );
  const __m256i deqStep = _mm256_set1_epi32( quant.deqStep );
  const __m256i eight   = _mm256_set1_epi32( 8 );
  const __m256i three   = _mm256_set1_epi32( 3 );
  const __m256i bias    = _mm256_set1_epi32( 127 );
  __m256i distSum = _mm256_setzero_si256();
  __m256i bitSum  = _mm256_setzero_si256();

  for( int k = 0; k < 4; k++ )
  {
    for( int i = 0; i < 8; i++ )
    {
      __m256i& v = q[k][i];
      const __m256i absCoeff = _mm256_abs_epi32( v );
      const __m256i level    = _mm256_srai_epi32( _mm256_add_epi32( _mm256_mullo_epi32( absCoeff, scale ), round ), 16 );
      const __m256i absDeq   = _mm256_srai_epi32( _mm256_add_epi32( _mm256_mullo_epi32( level, deqStep ), eight ), 4 );
      const __m256i err      = _mm256_sub_epi32( absCoeff, absDeq );
      // squared errors as 64-bit sums: even lanes, then odd lanes shifted down
      distSum = _mm256_add_epi64( distSum, _mm256_mul_epi32( err, err ) );
      const __m256i errOdd = _mm256_srli_epi64( err, 32 );
      distSum = _mm256_add_epi64( distSum, _mm256_mul_epi32( errOdd, errOdd ) );
      // bits: 2 * floorLog2( level ) + 3 for non-zero levels, floorLog2 from the float exponent
      const __m256i log2    = _mm256_sub_epi32( _mm256_srli_epi32( _mm256_castps_si256( _mm256_cvtepi32_ps( level ) ), 23 ), bias );
      const __m256i nonZero = _mm256_cmpgt_epi32( level, _mm256_setzero_si256() );
      bitSum = _mm256_add_epi32( bitSum, _mm256_and_si256( nonZero, _mm256_add_epi32( _mm256_add_epi32( log2, log2 ), three ) ) );
      v = _mm256_sign_epi32( absDeq, v );
    }
  }

  __m128i d = _mm_add_epi64( _mm256_castsi256_si128( distSum ), _mm256_extracti128_si256( distSum, 1 ) );
  d = _mm_add_epi64( d, _mm_unpackhi_epi64( d, d ) );
  dist = _mm_cvtsi128_si64( d );
  __m128i b = _mm_add_epi32( _mm256_castsi256_si128( bitSum ), _mm256_extracti128_si256( bitSum, 1 ) );
  b = _mm_hadd_epi32( b, b );
  b = _mm_hadd_epi32( b, b );
  bits = _mm_cvtsi128_si32( b );

  if( recon )
  {
    hadamard16x16( q );
    const __m256i r128 = _mm256_set1_epi32( 128 );
    const __m256i vmax = _mm256_set1_epi16( ( short ) maxVal );
    for( int y = 0; y < 16; y++ )
    {
      const __m256i* left  = y < 8 ? q[0] : q[2];
      const __m256i* right = y < 8 ? q[1] : q[3];
      const __m256i  p     = _mm256_loadu_si256( ( const __m256i* ) ( pred + y * 16 ) );
      const __m256i  lo    = _mm256_srai_epi32( _mm256_add_epi32( left [y & 7], r128 ), 8 );
      const __m256i  hi    = _mm256_srai_epi32( _mm256_add_epi32( right[y & 7], r128 ), 8 );
      // packs works per 128-bit lane: reorder the 64-bit quarters back to sample order. The reconstructed
      // residual is a sample difference plus at most half the unnormalised step, inside 16 bits even at QP 63,
      // so neither the packing nor the addition saturates.
      const __m256i  res   = _mm256_permute4x64_epi64( _mm256_packs_epi32( lo, hi ), 0xd8 );
      const __m256i  sum   = _mm256_min_epi16( _mm256_max_epi16( _mm256_adds_epi16( p, res ), _mm256_setzero_si256() ), vmax );
      _mm256_storeu_si256( ( __m256i* ) ( recon + y * reconStride ), sum );
    }
  }
}

// AVX2 version of tplPredBlockCore<16>, identical results: horizontal taps in 16 bits (at most 1023 * 16), one row of
// 16 samples per register, the vertical taps as 32-bit multiply-adds of interleaved row pairs. The unpacking and the
// packing both work per 128-bit lane, so the samples come out in order.
template<X86_VEXT vext>
void tplPredBlock16x16_SIMD( const Pel* src, ptrdiff_t srcStride, int fx, int fy, Pel* dst )
{
  const __m256i wx0 = _mm256_set1_epi16( short( 16 - fx ) );
  const __m256i wx1 = _mm256_set1_epi16( short( fx ) );
  const __m256i wy  = _mm256_set1_epi32( ( fy << 16 ) | ( 16 - fy ) );
  const __m256i rnd = _mm256_set1_epi32( 128 );

  __m256i prev;
  for( int j = 0; j <= 16; j++ )
  {
    const __m256i a   = _mm256_loadu_si256( ( const __m256i* ) ( src + j * srcStride ) );
    const __m256i b   = _mm256_loadu_si256( ( const __m256i* ) ( src + j * srcStride + 1 ) );
    const __m256i row = _mm256_add_epi16( _mm256_mullo_epi16( a, wx0 ), _mm256_mullo_epi16( b, wx1 ) );
    if( j > 0 )
    {
      // ( top, bottom ) pairs, multiplied by ( 16 - fy, fy ) and summed in 32 bits
      const __m256i lo = _mm256_srai_epi32( _mm256_add_epi32( _mm256_madd_epi16( _mm256_unpacklo_epi16( prev, row ), wy ), rnd ), 8 );
      const __m256i hi = _mm256_srai_epi32( _mm256_add_epi32( _mm256_madd_epi16( _mm256_unpackhi_epi16( prev, row ), wy ), rnd ), 8 );
      _mm256_storeu_si256( ( __m256i* ) ( dst + ( j - 1 ) * 16 ), _mm256_packs_epi32( lo, hi ) );
    }
    prev = row;
  }
}

// The squared error of an 8x8 block against its bilinear prediction, the prediction computed as in
// tplPredBlock8x8_SIMD and kept in registers.
template<X86_VEXT vext>
int tplPredError8x8_SIMD( const Pel* src, ptrdiff_t srcStride, int fx, int fy, const Pel* org, ptrdiff_t orgStride )
{
  const __m128i wx0 = _mm_set1_epi16( short( 16 - fx ) );
  const __m128i wx1 = _mm_set1_epi16( short( fx ) );
  const __m256i wy  = _mm256_set1_epi32( ( fy << 16 ) | ( 16 - fy ) );
  const __m256i rnd = _mm256_set1_epi32( 128 );

  __m128i rows[9];
  for( int j = 0; j < 9; j++ )
  {
    const __m128i a = _mm_loadu_si128( ( const __m128i* ) ( src + j * srcStride ) );
    const __m128i b = _mm_loadu_si128( ( const __m128i* ) ( src + j * srcStride + 1 ) );
    rows[j] = _mm_add_epi16( _mm_mullo_epi16( a, wx0 ), _mm_mullo_epi16( b, wx1 ) );
  }
  __m128i sum = _mm_setzero_si128();
  for( int j = 0; j < 8; j++ )
  {
    const __m256i pairs = _mm256_set_m128i( _mm_unpackhi_epi16( rows[j], rows[j + 1] ), _mm_unpacklo_epi16( rows[j], rows[j + 1] ) );
    const __m256i pred  = _mm256_srai_epi32( _mm256_add_epi32( _mm256_madd_epi16( pairs, wy ), rnd ), 8 );
    const __m128i diff  = _mm_sub_epi16( _mm_loadu_si128( ( const __m128i* ) ( org + j * orgStride ) ),
                                         _mm_packs_epi32( _mm256_castsi256_si128( pred ), _mm256_extracti128_si256( pred, 1 ) ) );
    sum = _mm_add_epi32( sum, _mm_madd_epi16( diff, diff ) );
  }
  sum = _mm_hadd_epi32( sum, sum );
  sum = _mm_hadd_epi32( sum, sum );
  return _mm_cvtsi128_si32( sum );
}

// The squared error of a 16x16 block against its bilinear prediction, the prediction computed as in
// tplPredBlock16x16_SIMD and kept in registers.
template<X86_VEXT vext>
int tplPredError16x16_SIMD( const Pel* src, ptrdiff_t srcStride, int fx, int fy, const Pel* org, ptrdiff_t orgStride )
{
  const __m256i wx0 = _mm256_set1_epi16( short( 16 - fx ) );
  const __m256i wx1 = _mm256_set1_epi16( short( fx ) );
  const __m256i wy  = _mm256_set1_epi32( ( fy << 16 ) | ( 16 - fy ) );
  const __m256i rnd = _mm256_set1_epi32( 128 );

  __m256i sum = _mm256_setzero_si256();
  __m256i prev;
  for( int j = 0; j <= 16; j++ )
  {
    const __m256i a   = _mm256_loadu_si256( ( const __m256i* ) ( src + j * srcStride ) );
    const __m256i b   = _mm256_loadu_si256( ( const __m256i* ) ( src + j * srcStride + 1 ) );
    const __m256i row = _mm256_add_epi16( _mm256_mullo_epi16( a, wx0 ), _mm256_mullo_epi16( b, wx1 ) );
    if( j > 0 )
    {
      const __m256i lo   = _mm256_srai_epi32( _mm256_add_epi32( _mm256_madd_epi16( _mm256_unpacklo_epi16( prev, row ), wy ), rnd ), 8 );
      const __m256i hi   = _mm256_srai_epi32( _mm256_add_epi32( _mm256_madd_epi16( _mm256_unpackhi_epi16( prev, row ), wy ), rnd ), 8 );
      const __m256i diff = _mm256_sub_epi16( _mm256_loadu_si256( ( const __m256i* ) ( org + ( j - 1 ) * orgStride ) ), _mm256_packs_epi32( lo, hi ) );
      sum = _mm256_add_epi32( sum, _mm256_madd_epi16( diff, diff ) );
    }
    prev = row;
  }
  __m128i s = _mm_add_epi32( _mm256_castsi256_si128( sum ), _mm256_extracti128_si256( sum, 1 ) );
  s = _mm_hadd_epi32( s, s );
  s = _mm_hadd_epi32( s, s );
  return _mm_cvtsi128_si32( s );
}

// AVX2 version of tplDownsampleRowCore: 16 output samples per step, the rest scalar.
template<X86_VEXT vext>
void tplDownsampleRow_SIMD( const Pel* src, ptrdiff_t srcStride, Pel* dst, int dstWidth )
{
  const Pel*    below = src + srcStride;
  const __m256i ones  = _mm256_set1_epi16( 1 );
  const __m256i two   = _mm256_set1_epi32( 2 );
  int x = 0;
  for( ; x + 16 <= dstWidth; x += 16 )
  {
    const __m256i r0  = _mm256_add_epi16( _mm256_loadu_si256( ( const __m256i* ) ( src   + 2 * x ) ),      _mm256_loadu_si256( ( const __m256i* ) ( below + 2 * x ) ) );
    const __m256i r1  = _mm256_add_epi16( _mm256_loadu_si256( ( const __m256i* ) ( src   + 2 * x + 16 ) ), _mm256_loadu_si256( ( const __m256i* ) ( below + 2 * x + 16 ) ) );
    const __m256i s0  = _mm256_srai_epi32( _mm256_add_epi32( _mm256_madd_epi16( r0, ones ), two ), 2 );
    const __m256i s1  = _mm256_srai_epi32( _mm256_add_epi32( _mm256_madd_epi16( r1, ones ), two ), 2 );
    // packs works per 128-bit lane: reorder the 64-bit quarters back to sample order
    const __m256i out = _mm256_permute4x64_epi64( _mm256_packs_epi32( s0, s1 ), 0xd8 );
    _mm256_storeu_si256( ( __m256i* ) ( dst + x ), out );
  }
  for( ; x < dstWidth; x++ )
  {
    dst[x] = Pel( ( src[2 * x] + src[2 * x + 1] + below[2 * x] + below[2 * x + 1] + 2 ) >> 2 );
  }
}

#endif // USE_AVX2

template<X86_VEXT vext>
void CuTree::_initCuTree_X86()
{
#ifdef USE_AVX2
  m_codeBlock[TPL_BLK_8]  = tplCodeBlock8x8_SIMD<vext>;
  m_codeBlock[TPL_BLK_16] = tplCodeBlock16x16_SIMD<vext>;
  m_predBlock[TPL_BLK_8]  = tplPredBlock8x8_SIMD<vext>;
  m_predBlock[TPL_BLK_16] = tplPredBlock16x16_SIMD<vext>;
  m_predError[TPL_BLK_8]  = tplPredError8x8_SIMD<vext>;
  m_predError[TPL_BLK_16] = tplPredError16x16_SIMD<vext>;
  m_downsampleRow         = tplDownsampleRow_SIMD<vext>;
#endif
}

template void CuTree::_initCuTree_X86<SIMDX86>();

} // namespace vvenc

#endif
//! \}

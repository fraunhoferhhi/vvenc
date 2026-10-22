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
/** \file     FGAX86.h
    \brief    SIMD for the film grain analysis (row and block kernels)
*/

#include "CommonDefX86.h"
#include "Rom.h"
#include "SEIFilmGrainAnalyzer.h"

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_FGA

namespace vvenc {

// The kernels compute exactly what the scalar row kernels (fgaSobelCore etc. in SEIFilmGrainAnalyzer.cpp) compute.
// The vector loops cover the samples whose left and right neighbours are inside the row; the scalar kernels do the
// first and the remaining samples.

// Edge directions of 8 samples from their gradients, as in fgaSobelCore: 0, 45, 90 or 135 degrees.
static inline __m128i fgaDirection( __m128i gx, __m128i gy, __m128i a, __m128i b )
{
  const __m128i zero  = _mm_setzero_si128();
  const __m128i neg2b = _mm_sub_epi16( zero, _mm_add_epi16( b, b ) );
  // (a + b)^2 - 2 b^2 > 0: not 0 degrees. The products are 32-bit: madd of (s, b) and (s, -2b) gives s*s - 2*b*b.
  const __m128i s     = _mm_add_epi16( a, b );
  const __m128i not0  = _mm_packs_epi32( _mm_cmpgt_epi32( _mm_madd_epi16( _mm_unpacklo_epi16( s, b ), _mm_unpacklo_epi16( s, neg2b ) ), zero ),
                                         _mm_cmpgt_epi32( _mm_madd_epi16( _mm_unpackhi_epi16( s, b ), _mm_unpackhi_epi16( s, neg2b ) ), zero ) );
  // a > b and (a - b)^2 - 2 b^2 > 0: 90 degrees
  const __m128i d     = _mm_sub_epi16( a, b );
  const __m128i is90  = _mm_and_si128( _mm_cmpgt_epi16( a, b ),
                        _mm_packs_epi32( _mm_cmpgt_epi32( _mm_madd_epi16( _mm_unpacklo_epi16( d, b ), _mm_unpacklo_epi16( d, neg2b ) ), zero ),
                                         _mm_cmpgt_epi32( _mm_madd_epi16( _mm_unpackhi_epi16( d, b ), _mm_unpackhi_epi16( d, neg2b ) ), zero ) ) );
  // 45 degrees when gx and gy have the same sign, else 135
  const __m128i diffSign = _mm_srai_epi16( _mm_xor_si128( gx, gy ), 15 );
  __m128i dir = _mm_blendv_epi8( _mm_set1_epi16( 45 ), _mm_set1_epi16( 135 ), diffSign );
  dir = _mm_blendv_epi8( dir, _mm_set1_epi16( 90 ), is90 );
  return _mm_and_si128( dir, not0 );
}

#ifdef USE_AVX2
static inline __m256i fgaDirection( __m256i gx, __m256i gy, __m256i a, __m256i b )
{
  const __m256i zero  = _mm256_setzero_si256();
  const __m256i neg2b = _mm256_sub_epi16( zero, _mm256_add_epi16( b, b ) );
  const __m256i s     = _mm256_add_epi16( a, b );
  const __m256i not0  = _mm256_packs_epi32( _mm256_cmpgt_epi32( _mm256_madd_epi16( _mm256_unpacklo_epi16( s, b ), _mm256_unpacklo_epi16( s, neg2b ) ), zero ),
                                            _mm256_cmpgt_epi32( _mm256_madd_epi16( _mm256_unpackhi_epi16( s, b ), _mm256_unpackhi_epi16( s, neg2b ) ), zero ) );
  const __m256i d     = _mm256_sub_epi16( a, b );
  const __m256i is90  = _mm256_and_si256( _mm256_cmpgt_epi16( a, b ),
                        _mm256_packs_epi32( _mm256_cmpgt_epi32( _mm256_madd_epi16( _mm256_unpacklo_epi16( d, b ), _mm256_unpacklo_epi16( d, neg2b ) ), zero ),
                                            _mm256_cmpgt_epi32( _mm256_madd_epi16( _mm256_unpackhi_epi16( d, b ), _mm256_unpackhi_epi16( d, neg2b ) ), zero ) ) );
  const __m256i diffSign = _mm256_srai_epi16( _mm256_xor_si256( gx, gy ), 15 );
  __m256i dir = _mm256_blendv_epi8( _mm256_set1_epi16( 45 ), _mm256_set1_epi16( 135 ), diffSign );
  dir = _mm256_blendv_epi8( dir, _mm256_set1_epi16( 90 ), is90 );
  return _mm256_and_si256( dir, not0 );
}
#endif

template<X86_VEXT vext>
void fgaSobelSIMD( const Pel* above, const Pel* cur, const Pel* below, int width, Pel maxVal, Pel* mag, Pel* dir )
{
  int x = 1;
#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i vmax = _mm256_set1_epi16( maxVal );
    for( ; x + 16 < width; x += 16 )
    {
      const __m256i al = _mm256_loadu_si256( (const __m256i*) ( above + x - 1 ) );
      const __m256i ac = _mm256_loadu_si256( (const __m256i*) ( above + x ) );
      const __m256i ar = _mm256_loadu_si256( (const __m256i*) ( above + x + 1 ) );
      const __m256i cl = _mm256_loadu_si256( (const __m256i*) ( cur + x - 1 ) );
      const __m256i cr = _mm256_loadu_si256( (const __m256i*) ( cur + x + 1 ) );
      const __m256i bl = _mm256_loadu_si256( (const __m256i*) ( below + x - 1 ) );
      const __m256i bc = _mm256_loadu_si256( (const __m256i*) ( below + x ) );
      const __m256i br = _mm256_loadu_si256( (const __m256i*) ( below + x + 1 ) );
      const __m256i gx = _mm256_sub_epi16( _mm256_add_epi16( _mm256_add_epi16( bl, br ), _mm256_add_epi16( bc, bc ) ),
                                           _mm256_add_epi16( _mm256_add_epi16( al, ar ), _mm256_add_epi16( ac, ac ) ) );
      const __m256i gy = _mm256_sub_epi16( _mm256_add_epi16( _mm256_add_epi16( ar, br ), _mm256_add_epi16( cr, cr ) ),
                                           _mm256_add_epi16( _mm256_add_epi16( al, bl ), _mm256_add_epi16( cl, cl ) ) );
      const __m256i a  = _mm256_abs_epi16( gx );
      const __m256i b  = _mm256_abs_epi16( gy );
      _mm256_storeu_si256( (__m256i*) ( mag + x ), _mm256_min_epi16( _mm256_srli_epi16( _mm256_add_epi16( a, b ), 1 ), vmax ) );
      _mm256_storeu_si256( (__m256i*) ( dir + x ), fgaDirection( gx, gy, a, b ) );
    }
  }
#endif
  const __m128i vmax = _mm_set1_epi16( maxVal );
  for( ; x + 8 < width; x += 8 )
  {
    const __m128i al = _mm_loadu_si128( (const __m128i*) ( above + x - 1 ) );
    const __m128i ac = _mm_loadu_si128( (const __m128i*) ( above + x ) );
    const __m128i ar = _mm_loadu_si128( (const __m128i*) ( above + x + 1 ) );
    const __m128i cl = _mm_loadu_si128( (const __m128i*) ( cur + x - 1 ) );
    const __m128i cr = _mm_loadu_si128( (const __m128i*) ( cur + x + 1 ) );
    const __m128i bl = _mm_loadu_si128( (const __m128i*) ( below + x - 1 ) );
    const __m128i bc = _mm_loadu_si128( (const __m128i*) ( below + x ) );
    const __m128i br = _mm_loadu_si128( (const __m128i*) ( below + x + 1 ) );
    const __m128i gx = _mm_sub_epi16( _mm_add_epi16( _mm_add_epi16( bl, br ), _mm_add_epi16( bc, bc ) ),
                                      _mm_add_epi16( _mm_add_epi16( al, ar ), _mm_add_epi16( ac, ac ) ) );
    const __m128i gy = _mm_sub_epi16( _mm_add_epi16( _mm_add_epi16( ar, br ), _mm_add_epi16( cr, cr ) ),
                                      _mm_add_epi16( _mm_add_epi16( al, bl ), _mm_add_epi16( cl, cl ) ) );
    const __m128i a  = _mm_abs_epi16( gx );
    const __m128i b  = _mm_abs_epi16( gy );
    _mm_storeu_si128( (__m128i*) ( mag + x ), _mm_min_epi16( _mm_srli_epi16( _mm_add_epi16( a, b ), 1 ), vmax ) );
    _mm_storeu_si128( (__m128i*) ( dir + x ), fgaDirection( gx, gy, a, b ) );
  }
  fgaSobelCore( above, cur, below, width, 0, 1, maxVal, mag, dir );
  fgaSobelCore( above, cur, below, width, x, width, maxVal, mag, dir );
}

template<X86_VEXT vext>
void fgaNonMaxSIMD( const Pel* magAbove, const Pel* mag, const Pel* magBelow, const Pel* dir, int width, Pel* dst )
{
  int x = 1;
#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i d0 = _mm256_setzero_si256(), d45 = _mm256_set1_epi16( 45 ), d90 = _mm256_set1_epi16( 90 );
    for( ; x + 16 < width; x += 16 )
    {
      const __m256i ml = _mm256_loadu_si256( (const __m256i*) ( mag + x - 1 ) );
      const __m256i mc = _mm256_loadu_si256( (const __m256i*) ( mag + x ) );
      const __m256i mr = _mm256_loadu_si256( (const __m256i*) ( mag + x + 1 ) );
      const __m256i al = _mm256_loadu_si256( (const __m256i*) ( magAbove + x - 1 ) );
      const __m256i ac = _mm256_loadu_si256( (const __m256i*) ( magAbove + x ) );
      const __m256i ar = _mm256_loadu_si256( (const __m256i*) ( magAbove + x + 1 ) );
      const __m256i bl = _mm256_loadu_si256( (const __m256i*) ( magBelow + x - 1 ) );
      const __m256i bc = _mm256_loadu_si256( (const __m256i*) ( magBelow + x ) );
      const __m256i br = _mm256_loadu_si256( (const __m256i*) ( magBelow + x + 1 ) );
      const __m256i d  = _mm256_loadu_si256( (const __m256i*) ( dir + x ) );
      const __m256i is0  = _mm256_cmpeq_epi16( d, d0 );
      const __m256i is45 = _mm256_cmpeq_epi16( d, d45 );
      const __m256i is90 = _mm256_cmpeq_epi16( d, d90 );
      // neighbours for 135 degrees, replaced for the other directions
      __m256i n1 = _mm256_blendv_epi8( _mm256_blendv_epi8( _mm256_blendv_epi8( bl, bc, is90 ), br, is45 ), mr, is0 );
      __m256i n2 = _mm256_blendv_epi8( _mm256_blendv_epi8( _mm256_blendv_epi8( ar, ac, is90 ), al, is45 ), ml, is0 );
      const __m256i suppressed = _mm256_cmpgt_epi16( _mm256_max_epi16( n1, n2 ), mc );
      _mm256_storeu_si256( (__m256i*) ( dst + x ), _mm256_andnot_si256( suppressed, mc ) );
    }
  }
#endif
  const __m128i d0 = _mm_setzero_si128(), d45 = _mm_set1_epi16( 45 ), d90 = _mm_set1_epi16( 90 );
  for( ; x + 8 < width; x += 8 )
  {
    const __m128i ml = _mm_loadu_si128( (const __m128i*) ( mag + x - 1 ) );
    const __m128i mc = _mm_loadu_si128( (const __m128i*) ( mag + x ) );
    const __m128i mr = _mm_loadu_si128( (const __m128i*) ( mag + x + 1 ) );
    const __m128i al = _mm_loadu_si128( (const __m128i*) ( magAbove + x - 1 ) );
    const __m128i ac = _mm_loadu_si128( (const __m128i*) ( magAbove + x ) );
    const __m128i ar = _mm_loadu_si128( (const __m128i*) ( magAbove + x + 1 ) );
    const __m128i bl = _mm_loadu_si128( (const __m128i*) ( magBelow + x - 1 ) );
    const __m128i bc = _mm_loadu_si128( (const __m128i*) ( magBelow + x ) );
    const __m128i br = _mm_loadu_si128( (const __m128i*) ( magBelow + x + 1 ) );
    const __m128i d  = _mm_loadu_si128( (const __m128i*) ( dir + x ) );
    const __m128i is0  = _mm_cmpeq_epi16( d, d0 );
    const __m128i is45 = _mm_cmpeq_epi16( d, d45 );
    const __m128i is90 = _mm_cmpeq_epi16( d, d90 );
    __m128i n1 = _mm_blendv_epi8( _mm_blendv_epi8( _mm_blendv_epi8( bl, bc, is90 ), br, is45 ), mr, is0 );
    __m128i n2 = _mm_blendv_epi8( _mm_blendv_epi8( _mm_blendv_epi8( ar, ac, is90 ), al, is45 ), ml, is0 );
    const __m128i suppressed = _mm_cmpgt_epi16( _mm_max_epi16( n1, n2 ), mc );
    _mm_storeu_si128( (__m128i*) ( dst + x ), _mm_andnot_si128( suppressed, mc ) );
  }
  fgaNonMaxCore( magAbove, mag, magBelow, dir, width, 0, 1, dst );
  fgaNonMaxCore( magAbove, mag, magBelow, dir, width, x, width, dst );
}

template<X86_VEXT vext>
void fgaMorphRowSIMD( const Pel* src, int width, bool dilate, Pel* dst )
{
  int x = 1;
#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    for( ; x + 16 < width; x += 16 )
    {
      const __m256i l = _mm256_loadu_si256( (const __m256i*) ( src + x - 1 ) );
      const __m256i c = _mm256_loadu_si256( (const __m256i*) ( src + x ) );
      const __m256i r = _mm256_loadu_si256( (const __m256i*) ( src + x + 1 ) );
      const __m256i v = dilate ? _mm256_max_epi16( _mm256_max_epi16( l, c ), r ) : _mm256_min_epi16( _mm256_min_epi16( l, c ), r );
      _mm256_storeu_si256( (__m256i*) ( dst + x ), v );
    }
  }
#endif
  for( ; x + 8 < width; x += 8 )
  {
    const __m128i l = _mm_loadu_si128( (const __m128i*) ( src + x - 1 ) );
    const __m128i c = _mm_loadu_si128( (const __m128i*) ( src + x ) );
    const __m128i r = _mm_loadu_si128( (const __m128i*) ( src + x + 1 ) );
    const __m128i v = dilate ? _mm_max_epi16( _mm_max_epi16( l, c ), r ) : _mm_min_epi16( _mm_min_epi16( l, c ), r );
    _mm_storeu_si128( (__m128i*) ( dst + x ), v );
  }
  fgaMorphRowCore( src, width, 0, 1, dilate, dst );
  fgaMorphRowCore( src, width, x, width, dilate, dst );
}

template<X86_VEXT vext>
void fgaMorphColSIMD( const Pel* above, const Pel* cur, const Pel* below, int width, bool dilate, Pel* dst )
{
  int x = 0;
#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    for( ; x + 16 <= width; x += 16 )
    {
      const __m256i a = _mm256_loadu_si256( (const __m256i*) ( above + x ) );
      const __m256i c = _mm256_loadu_si256( (const __m256i*) ( cur + x ) );
      const __m256i b = _mm256_loadu_si256( (const __m256i*) ( below + x ) );
      const __m256i v = dilate ? _mm256_max_epi16( _mm256_max_epi16( a, c ), b ) : _mm256_min_epi16( _mm256_min_epi16( a, c ), b );
      _mm256_storeu_si256( (__m256i*) ( dst + x ), v );
    }
  }
#endif
  for( ; x + 8 <= width; x += 8 )
  {
    const __m128i a = _mm_loadu_si128( (const __m128i*) ( above + x ) );
    const __m128i c = _mm_loadu_si128( (const __m128i*) ( cur + x ) );
    const __m128i b = _mm_loadu_si128( (const __m128i*) ( below + x ) );
    const __m128i v = dilate ? _mm_max_epi16( _mm_max_epi16( a, c ), b ) : _mm_min_epi16( _mm_min_epi16( a, c ), b );
    _mm_storeu_si128( (__m128i*) ( dst + x ), v );
  }
  fgaMorphColCore( above, cur, below, x, width, dilate, dst );
}

template<X86_VEXT vext>
Pel fgaRowMaxSIMD( const Pel* src, int n )
{
  int i = 0;
  __m128i vmax = _mm_setzero_si128();
#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    __m256i m = _mm256_setzero_si256();
    for( ; i + 16 <= n; i += 16 )
    {
      m = _mm256_max_epi16( m, _mm256_loadu_si256( (const __m256i*) ( src + i ) ) );
    }
    vmax = _mm_max_epi16( _mm256_castsi256_si128( m ), _mm256_extracti128_si256( m, 1 ) );
  }
#endif
  for( ; i + 8 <= n; i += 8 )
  {
    vmax = _mm_max_epi16( vmax, _mm_loadu_si128( (const __m128i*) ( src + i ) ) );
  }
  vmax = _mm_max_epi16( vmax, _mm_shuffle_epi32( vmax, 0x4e ) );
  vmax = _mm_max_epi16( vmax, _mm_shuffle_epi32( vmax, 0xb1 ) );
  vmax = _mm_max_epi16( vmax, _mm_shufflelo_epi16( vmax, 0xb1 ) );
  return std::max( (Pel) _mm_extract_epi16( vmax, 0 ), fgaRowMaxCore( src + i, n - i ) );
}

template<X86_VEXT vext>
void fgaClassifySIMD( Pel* row, int n, Pel low, Pel high, Pel weak, Pel strong )
{
  int i = 0;
#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i vl = _mm256_set1_epi16( low ), vh = _mm256_set1_epi16( high );
    const __m256i vw = _mm256_set1_epi16( weak ), vs = _mm256_set1_epi16( strong );
    for( ; i + 16 <= n; i += 16 )
    {
      const __m256i v = _mm256_loadu_si256( (const __m256i*) ( row + i ) );
      const __m256i r = _mm256_blendv_epi8( _mm256_and_si256( _mm256_cmpgt_epi16( v, vl ), vw ), vs, _mm256_cmpgt_epi16( v, vh ) );
      _mm256_storeu_si256( (__m256i*) ( row + i ), r );
    }
  }
#endif
  const __m128i vl = _mm_set1_epi16( low ), vh = _mm_set1_epi16( high );
  const __m128i vw = _mm_set1_epi16( weak ), vs = _mm_set1_epi16( strong );
  for( ; i + 8 <= n; i += 8 )
  {
    const __m128i v = _mm_loadu_si128( (const __m128i*) ( row + i ) );
    const __m128i r = _mm_blendv_epi8( _mm_and_si128( _mm_cmpgt_epi16( v, vl ), vw ), vs, _mm_cmpgt_epi16( v, vh ) );
    _mm_storeu_si128( (__m128i*) ( row + i ), r );
  }
  fgaClassifyCore( row + i, n - i, low, high, weak, strong );
}

// Compares 16 (AVX2) or 8 samples at a time and visits only the set bits of the comparison mask.
template<X86_VEXT vext>
int fgaFindEqualSIMD( const Pel* row, int n, Pel value, int* positions )
{
  int count = 0, i = 0;
#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i v = _mm256_set1_epi16( value );
    for( ; i + 16 <= n; i += 16 )
    {
      // two mask bits per 16-bit sample; keep the even ones
      uint32_t bits = (uint32_t) _mm256_movemask_epi8( _mm256_cmpeq_epi16( _mm256_loadu_si256( (const __m256i*) ( row + i ) ), v ) ) & 0x55555555u;
      while( bits )
      {
        unsigned long b;
#ifdef _MSC_VER
        _BitScanForward( &b, bits );
#else
        b = __builtin_ctz( bits );
#endif
        positions[ count++ ] = i + (int) ( b >> 1 );
        bits &= bits - 1;
      }
    }
  }
#endif
  const __m128i v = _mm_set1_epi16( value );
  for( ; i + 8 <= n; i += 8 )
  {
    uint32_t bits = (uint32_t) _mm_movemask_epi8( _mm_cmpeq_epi16( _mm_loadu_si128( (const __m128i*) ( row + i ) ), v ) ) & 0x5555u;
    while( bits )
    {
      unsigned long b;
#ifdef _MSC_VER
      _BitScanForward( &b, bits );
#else
      b = __builtin_ctz( bits );
#endif
      positions[ count++ ] = i + (int) ( b >> 1 );
      bits &= bits - 1;
    }
  }
  // the remaining samples; their positions are relative to row + i
  const int tail = fgaFindEqualCore( row + i, n - i, value, positions + count );
  for( int t = 0; t < tail; t++ )
  {
    positions[ count + t ] += i;
  }
  return count + tail;
}

template<X86_VEXT vext>
bool fgaIsZeroBlockSIMD( const Pel* src, int stride )
{
#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    __m256i acc = _mm256_setzero_si256();
    for( int y = 0; y < 16; y++ )
    {
      acc = _mm256_or_si256( acc, _mm256_loadu_si256( (const __m256i*) ( src + y * stride ) ) );
    }
    return _mm256_testz_si256( acc, acc ) != 0;
  }
#endif
  __m128i acc = _mm_setzero_si128();
  for( int y = 0; y < 16; y++ )
  {
    acc = _mm_or_si128( acc, _mm_or_si128( _mm_loadu_si128( (const __m128i*) ( src + y * stride ) ),
                                           _mm_loadu_si128( (const __m128i*) ( src + y * stride + 8 ) ) ) );
  }
  return _mm_testz_si128( acc, acc ) != 0;
}

// The 32-bit lanes cannot overflow: a lane sums 2 squares per row over 16 rows, at most 32 * 4095^2 at 12 bits.
template<X86_VEXT vext>
void fgaBlockSumsSIMD( const Pel* src, int stride, int64_t& sum, int64_t& sumSq )
{
  __m128i s, q;
#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i ones = _mm256_set1_epi16( 1 );
    __m256i vs = _mm256_setzero_si256(), vq = _mm256_setzero_si256();
    for( int y = 0; y < 16; y++ )
    {
      const __m256i v = _mm256_loadu_si256( (const __m256i*) ( src + y * stride ) );
      vs = _mm256_add_epi32( vs, _mm256_madd_epi16( v, ones ) );
      vq = _mm256_add_epi32( vq, _mm256_madd_epi16( v, v ) );
    }
    s = _mm_add_epi32( _mm256_castsi256_si128( vs ), _mm256_extracti128_si256( vs, 1 ) );
    q = _mm_add_epi32( _mm256_castsi256_si128( vq ), _mm256_extracti128_si256( vq, 1 ) );
  }
  else
#endif
  {
    const __m128i ones = _mm_set1_epi16( 1 );
    s = q = _mm_setzero_si128();
    for( int y = 0; y < 16; y++ )
    {
      const __m128i v0 = _mm_loadu_si128( (const __m128i*) ( src + y * stride ) );
      const __m128i v1 = _mm_loadu_si128( (const __m128i*) ( src + y * stride + 8 ) );
      s = _mm_add_epi32( s, _mm_add_epi32( _mm_madd_epi16( v0, ones ), _mm_madd_epi16( v1, ones ) ) );
      q = _mm_add_epi32( q, _mm_add_epi32( _mm_madd_epi16( v0, v0 ), _mm_madd_epi16( v1, v1 ) ) );
    }
  }
  // sum the four 32-bit lanes in 64 bits (the square sums can exceed 31 bits in total at 12 bits)
  const __m128i q64 = _mm_add_epi64( _mm_cvtepu32_epi64( q ), _mm_cvtepu32_epi64( _mm_srli_si128( q, 8 ) ) );
  const __m128i s64 = _mm_add_epi64( _mm_cvtepi32_epi64( s ), _mm_cvtepi32_epi64( _mm_srli_si128( s, 8 ) ) );
  sum   = _mm_extract_epi64( s64, 0 ) + _mm_extract_epi64( s64, 1 );
  sumSq = _mm_extract_epi64( q64, 0 ) + _mm_extract_epi64( q64, 1 );
}

// 16x16 DCT with madd: the horizontal stage multiplies a broadcast pair of samples with the basis pairs of 8 (AVX2)
// or 4 (SSE4.1) frequencies; the vertical stage multiplies interleaved pairs of rows with a broadcast basis pair.
// Rounding, shifts and clipping are those of fgaSquaredDct16Core, so the result is identical.
template<X86_VEXT vext>
void fgaSquaredDct16SIMD( const Pel* src, int stride, int maxRange, double* sum )
{
  constexpr int N = 16;
  const TMatrixCoeff* tr = g_trCoreDCT2P16[ TRANSFORM_FORWARD ][ 0 ];

  // basis pairs: pairs[ p ][ u ] = ( T[u][2p], T[u][2p+1] ) as one 32-bit value
  int32_t pairs[ N / 2 ][ N ];
  for( int p = 0; p < N / 2; p++ )
  {
    for( int u = 0; u < N; u++ )
    {
      pairs[ p ][ u ] = (int32_t) ( (uint16_t) tr[ u * N + 2 * p ] | ( (uint32_t) (uint16_t) tr[ u * N + 2 * p + 1 ] << 16 ) );
    }
  }

  alignas( 32 ) int16_t g[ N ][ N ];   // first stage: g[ row ][ horizontal frequency ]
  alignas( 32 ) int32_t c[ N ];        // second stage, one vertical frequency
#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i rnd = _mm256_set1_epi32( 128 );
    for( int y = 0; y < N; y++ )
    {
      const Pel* row = src + y * stride;
      __m256i lo = _mm256_setzero_si256(), hi = _mm256_setzero_si256();
      for( int p = 0; p < N / 2; p++ )
      {
        const __m256i x = _mm256_set1_epi32( (int32_t) ( (uint16_t) row[ 2 * p ] | ( (uint32_t) (uint16_t) row[ 2 * p + 1 ] << 16 ) ) );
        lo = _mm256_add_epi32( lo, _mm256_madd_epi16( _mm256_loadu_si256( (const __m256i*) &pairs[ p ][ 0 ] ), x ) );
        hi = _mm256_add_epi32( hi, _mm256_madd_epi16( _mm256_loadu_si256( (const __m256i*) &pairs[ p ][ 8 ] ), x ) );
      }
      lo = _mm256_srai_epi32( _mm256_add_epi32( lo, rnd ), 8 );
      hi = _mm256_srai_epi32( _mm256_add_epi32( hi, rnd ), 8 );
      _mm256_store_si256( (__m256i*) g[ y ], _mm256_permute4x64_epi64( _mm256_packs_epi32( lo, hi ), 0xd8 ) );
    }
    // the 128-bit lanes of unpacklo hold frequencies 0-3 and 8-11, those of unpackhi 4-7 and 12-15
    const __m256i vmax = _mm256_set1_epi32( maxRange ), vmin = _mm256_set1_epi32( -maxRange );
    for( int v = 0; v < N; v++ )
    {
      __m256i lo = _mm256_setzero_si256(), hi = _mm256_setzero_si256();
      for( int p = 0; p < N / 2; p++ )
      {
        const __m256i r0 = _mm256_load_si256( (const __m256i*) g[ 2 * p ] );
        const __m256i r1 = _mm256_load_si256( (const __m256i*) g[ 2 * p + 1 ] );
        const __m256i t  = _mm256_set1_epi32( pairs[ p ][ v ] );
        lo = _mm256_add_epi32( lo, _mm256_madd_epi16( _mm256_unpacklo_epi16( r0, r1 ), t ) );
        hi = _mm256_add_epi32( hi, _mm256_madd_epi16( _mm256_unpackhi_epi16( r0, r1 ), t ) );
      }
      lo = _mm256_max_epi32( vmin, _mm256_min_epi32( vmax, _mm256_srai_epi32( _mm256_add_epi32( lo, rnd ), 8 ) ) );
      hi = _mm256_max_epi32( vmin, _mm256_min_epi32( vmax, _mm256_srai_epi32( _mm256_add_epi32( hi, rnd ), 8 ) ) );
      _mm256_store_si256( (__m256i*) &c[ 0 ], _mm256_permute2x128_si256( lo, hi, 0x20 ) );   // 0-3, 4-7
      _mm256_store_si256( (__m256i*) &c[ 8 ], _mm256_permute2x128_si256( lo, hi, 0x31 ) );   // 8-11, 12-15
      // c now holds frequencies 0-15 in order: lane 0 of lo and hi, then lane 1 of lo and hi
      for( int u = 0; u < N; u++ )
      {
        sum[ u * N + v ] += (double) c[ u ] * c[ u ];
      }
    }
    return;
  }
#endif
  const __m128i rnd = _mm_set1_epi32( 128 );
  for( int y = 0; y < N; y++ )
  {
    const Pel* row = src + y * stride;
    __m128i acc[ 4 ] = { _mm_setzero_si128(), _mm_setzero_si128(), _mm_setzero_si128(), _mm_setzero_si128() };
    for( int p = 0; p < N / 2; p++ )
    {
      const __m128i x = _mm_set1_epi32( (int32_t) ( (uint16_t) row[ 2 * p ] | ( (uint32_t) (uint16_t) row[ 2 * p + 1 ] << 16 ) ) );
      for( int q = 0; q < 4; q++ )
      {
        acc[ q ] = _mm_add_epi32( acc[ q ], _mm_madd_epi16( _mm_loadu_si128( (const __m128i*) &pairs[ p ][ 4 * q ] ), x ) );
      }
    }
    for( int q = 0; q < 4; q++ )
    {
      acc[ q ] = _mm_srai_epi32( _mm_add_epi32( acc[ q ], rnd ), 8 );
    }
    _mm_store_si128( (__m128i*) &g[ y ][ 0 ], _mm_packs_epi32( acc[ 0 ], acc[ 1 ] ) );
    _mm_store_si128( (__m128i*) &g[ y ][ 8 ], _mm_packs_epi32( acc[ 2 ], acc[ 3 ] ) );
  }
  const __m128i vmax = _mm_set1_epi32( maxRange ), vmin = _mm_set1_epi32( -maxRange );
  for( int v = 0; v < N; v++ )
  {
    __m128i acc[ 4 ] = { _mm_setzero_si128(), _mm_setzero_si128(), _mm_setzero_si128(), _mm_setzero_si128() };
    for( int p = 0; p < N / 2; p++ )
    {
      const __m128i t = _mm_set1_epi32( pairs[ p ][ v ] );
      for( int half = 0; half < 2; half++ )
      {
        const __m128i r0 = _mm_load_si128( (const __m128i*) &g[ 2 * p ][ 8 * half ] );
        const __m128i r1 = _mm_load_si128( (const __m128i*) &g[ 2 * p + 1 ][ 8 * half ] );
        acc[ 2 * half ]     = _mm_add_epi32( acc[ 2 * half ],     _mm_madd_epi16( _mm_unpacklo_epi16( r0, r1 ), t ) );
        acc[ 2 * half + 1 ] = _mm_add_epi32( acc[ 2 * half + 1 ], _mm_madd_epi16( _mm_unpackhi_epi16( r0, r1 ), t ) );
      }
    }
    for( int q = 0; q < 4; q++ )
    {
      _mm_store_si128( (__m128i*) &c[ 4 * q ], _mm_max_epi32( vmin, _mm_min_epi32( vmax, _mm_srai_epi32( _mm_add_epi32( acc[ q ], rnd ), 8 ) ) ) );
    }
    for( int u = 0; u < N; u++ )
    {
      sum[ u * N + v ] += (double) c[ u ] * c[ u ];
    }
  }
}

// 4x downsampling: the four rows are added in 16 bits (at most 4 * 4095), then groups of four samples with madd
// (pairs) and hadd (pairs of pairs).
template<X86_VEXT vext>
void fgaDownsample4SIMD( const Pel* src, int stride, int n, Pel* dst )
{
  int x = 0;
#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i ones  = _mm256_set1_epi16( 1 );
    const __m256i round = _mm256_set1_epi32( 8 );
    for( ; x + 8 <= n; x += 8 )
    {
      const Pel* s = src + 4 * x;
      __m256i a = _mm256_loadu_si256( (const __m256i*) s );
      __m256i b = _mm256_loadu_si256( (const __m256i*) ( s + 16 ) );
      for( int y = 1; y < 4; y++ )
      {
        a = _mm256_add_epi16( a, _mm256_loadu_si256( (const __m256i*) ( s + y * stride ) ) );
        b = _mm256_add_epi16( b, _mm256_loadu_si256( (const __m256i*) ( s + y * stride + 16 ) ) );
      }
      // lane 0 holds the sums of outputs 0, 1, 4, 5 and lane 1 those of outputs 2, 3, 6, 7
      __m256i q = _mm256_hadd_epi32( _mm256_madd_epi16( a, ones ), _mm256_madd_epi16( b, ones ) );
      q = _mm256_permute4x64_epi64( q, 0xD8 );
      q = _mm256_srai_epi32( _mm256_add_epi32( q, round ), 4 );
      q = _mm256_permute4x64_epi64( _mm256_packs_epi32( q, q ), 0x08 );
      _mm_storeu_si128( (__m128i*) ( dst + x ), _mm256_castsi256_si128( q ) );
    }
  }
#endif
  const __m128i ones  = _mm_set1_epi16( 1 );
  const __m128i round = _mm_set1_epi32( 8 );
  for( ; x + 4 <= n; x += 4 )
  {
    const Pel* s = src + 4 * x;
    __m128i a = _mm_loadu_si128( (const __m128i*) s );
    __m128i b = _mm_loadu_si128( (const __m128i*) ( s + 8 ) );
    for( int y = 1; y < 4; y++ )
    {
      a = _mm_add_epi16( a, _mm_loadu_si128( (const __m128i*) ( s + y * stride ) ) );
      b = _mm_add_epi16( b, _mm_loadu_si128( (const __m128i*) ( s + y * stride + 8 ) ) );
    }
    __m128i q = _mm_hadd_epi32( _mm_madd_epi16( a, ones ), _mm_madd_epi16( b, ones ) );
    q = _mm_srai_epi32( _mm_add_epi32( q, round ), 4 );
    _mm_storel_epi64( (__m128i*) ( dst + x ), _mm_packs_epi32( q, q ) );
  }
  fgaDownsample4Core( src + 4 * x, stride, n - x, dst + x );
}

// SAD of 4x4 blocks: absolute differences of four rows added in 16 bits (at most 4 * 4095), then groups of four
// samples with madd and hadd.
template<X86_VEXT vext>
void fgaSad4x4RowSIMD( const Pel* cur, int curStride, const Pel* ref, int refStride, int n, int* sad )
{
  int b = 0;
#ifdef USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i ones = _mm256_set1_epi16( 1 );
    for( ; b + 8 <= n; b += 8 )
    {
      __m256i lo = _mm256_setzero_si256(), hi = _mm256_setzero_si256();
      for( int y = 0; y < 4; y++ )
      {
        const Pel* c = cur + y * curStride + 4 * b;
        const Pel* r = ref + y * refStride + 4 * b;
        lo = _mm256_add_epi16( lo, _mm256_abs_epi16( _mm256_sub_epi16( _mm256_loadu_si256( (const __m256i*) c ), _mm256_loadu_si256( (const __m256i*) r ) ) ) );
        hi = _mm256_add_epi16( hi, _mm256_abs_epi16( _mm256_sub_epi16( _mm256_loadu_si256( (const __m256i*) ( c + 16 ) ), _mm256_loadu_si256( (const __m256i*) ( r + 16 ) ) ) ) );
      }
      const __m256i q = _mm256_hadd_epi32( _mm256_madd_epi16( lo, ones ), _mm256_madd_epi16( hi, ones ) );
      _mm256_storeu_si256( (__m256i*) ( sad + b ), _mm256_permute4x64_epi64( q, 0xD8 ) );
    }
  }
#endif
  const __m128i ones = _mm_set1_epi16( 1 );
  for( ; b + 4 <= n; b += 4 )
  {
    __m128i lo = _mm_setzero_si128(), hi = _mm_setzero_si128();
    for( int y = 0; y < 4; y++ )
    {
      const Pel* c = cur + y * curStride + 4 * b;
      const Pel* r = ref + y * refStride + 4 * b;
      lo = _mm_add_epi16( lo, _mm_abs_epi16( _mm_sub_epi16( _mm_loadu_si128( (const __m128i*) c ), _mm_loadu_si128( (const __m128i*) r ) ) ) );
      hi = _mm_add_epi16( hi, _mm_abs_epi16( _mm_sub_epi16( _mm_loadu_si128( (const __m128i*) ( c + 8 ) ), _mm_loadu_si128( (const __m128i*) ( r + 8 ) ) ) ) );
    }
    _mm_storeu_si128( (__m128i*) ( sad + b ), _mm_hadd_epi32( _mm_madd_epi16( lo, ones ), _mm_madd_epi16( hi, ones ) ) );
  }
  fgaSad4x4RowCore( cur + 4 * b, curStride, ref + 4 * b, refStride, n - b, sad + b );
}

template<X86_VEXT vext>
void fgaResidualRowSIMD( const Pel* cur, const Pel* prev, const Pel* next, int n, Pel* dst )
{
  int x = 0;
  if( next )
  {
#ifdef USE_AVX2
    if( vext >= AVX2 )
    {
      for( ; x + 16 <= n; x += 16 )
      {
        const __m256i c = _mm256_loadu_si256( (const __m256i*) ( cur + x ) );
        const __m256i d = _mm256_sub_epi16( _mm256_sub_epi16( _mm256_add_epi16( c, c ), _mm256_loadu_si256( (const __m256i*) ( prev + x ) ) ),
                                            _mm256_loadu_si256( (const __m256i*) ( next + x ) ) );
        _mm256_storeu_si256( (__m256i*) ( dst + x ), d );
      }
    }
#endif
    for( ; x + 8 <= n; x += 8 )
    {
      const __m128i c = _mm_loadu_si128( (const __m128i*) ( cur + x ) );
      const __m128i d = _mm_sub_epi16( _mm_sub_epi16( _mm_add_epi16( c, c ), _mm_loadu_si128( (const __m128i*) ( prev + x ) ) ),
                                       _mm_loadu_si128( (const __m128i*) ( next + x ) ) );
      _mm_storeu_si128( (__m128i*) ( dst + x ), d );
    }
  }
  else
  {
    for( ; x + 8 <= n; x += 8 )
    {
      _mm_storeu_si128( (__m128i*) ( dst + x ), _mm_sub_epi16( _mm_loadu_si128( (const __m128i*) ( cur + x ) ), _mm_loadu_si128( (const __m128i*) ( prev + x ) ) ) );
    }
  }
  fgaResidualRowCore( cur + x, prev + x, next ? next + x : nullptr, n - x, dst + x );
}

template<X86_VEXT vext>
void FGAnalyzer::_initFGAnalyzerX86( FgaRowKernels& k )
{
  k.sobel    = fgaSobelSIMD<vext>;
  k.nonMax   = fgaNonMaxSIMD<vext>;
  k.morphRow = fgaMorphRowSIMD<vext>;
  k.morphCol = fgaMorphColSIMD<vext>;
  k.rowMax   = fgaRowMaxSIMD<vext>;
  k.classify = fgaClassifySIMD<vext>;
  k.findEqual = fgaFindEqualSIMD<vext>;
  k.isZeroBlock  = fgaIsZeroBlockSIMD<vext>;
  k.blockSums    = fgaBlockSumsSIMD<vext>;
  k.squaredDct16 = fgaSquaredDct16SIMD<vext>;
  k.downsample4  = fgaDownsample4SIMD<vext>;
  k.sad4x4Row    = fgaSad4x4RowSIMD<vext>;
  k.residualRow  = fgaResidualRowSIMD<vext>;
}

template void FGAnalyzer::_initFGAnalyzerX86<SIMDX86>( FgaRowKernels& k );

} // namespace vvenc

#endif // TARGET_SIMD_X86 && ENABLE_SIMD_OPT_FGA

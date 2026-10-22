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
/** \file     FGA_neon.cpp
    \brief    Neon SIMD for the film grain analysis (row and block kernels)
*/

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "EncoderLib/SEIFilmGrainAnalyzer.h"
#include "sum_neon.h"

#include <arm_neon.h>
#ifdef _MSC_VER
#include <intrin.h>
#endif

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_FGA

namespace vvenc
{

// The kernels compute exactly what the scalar kernels (fgaSobelCore etc. in SEIFilmGrainAnalyzer.cpp) and their x86
// versions (FGAX86.h) compute. The vector loops cover the samples whose left and right neighbours are inside the row;
// the scalar kernels do the first and the remaining samples.

// Largest of the eight lanes.
static inline int16_t maxAcrossLanes( int16x8_t v )
{
#if REAL_TARGET_AARCH64
  return vmaxvq_s16( v );
#else
  int16x4_t m = vpmax_s16( vget_low_s16( v ), vget_high_s16( v ) );
  m = vpmax_s16( m, m );
  m = vpmax_s16( m, m );
  return vget_lane_s16( m, 0 );
#endif
}

// One bit per lane, lane i at bit i, of a comparison result.
static inline unsigned laneBits( uint16x8_t mask )
{
  static const uint16_t weights[ 8 ] = { 1, 2, 4, 8, 16, 32, 64, 128 };
  const uint16x8_t bits = vandq_u16( mask, vld1q_u16( weights ) );
#if REAL_TARGET_AARCH64
  return vaddvq_u16( bits );
#else
  const uint64x2_t sums = vpaddlq_u32( vpaddlq_u16( bits ) );
  return (unsigned) ( vgetq_lane_u64( sums, 0 ) + vgetq_lane_u64( sums, 1 ) );
#endif
}

static inline int lowestSetBit( unsigned bits )
{
#ifdef _MSC_VER
  unsigned long b;
  _BitScanForward( &b, bits );
  return (int) b;
#else
  return __builtin_ctz( bits );
#endif
}

// x * x - b * b2 > 0 for every lane, with the products in 32 bits.
static inline uint16x8_t squareExceeds( int16x8_t x, int16x8_t b, int16x8_t b2 )
{
  const int32x4_t lo = vmlsl_s16( vmull_s16( vget_low_s16( x ), vget_low_s16( x ) ), vget_low_s16( b ), vget_low_s16( b2 ) );
  const int32x4_t hi = vmlsl_s16( vmull_s16( vget_high_s16( x ), vget_high_s16( x ) ), vget_high_s16( b ), vget_high_s16( b2 ) );
  return vcombine_u16( vmovn_u32( vcgtq_s32( lo, vdupq_n_s32( 0 ) ) ), vmovn_u32( vcgtq_s32( hi, vdupq_n_s32( 0 ) ) ) );
}

// Edge directions of 8 samples from their gradients, as in fgaSobelCore: 0, 45, 90 or 135 degrees.
static inline int16x8_t fgaDirection( int16x8_t gx, int16x8_t gy, int16x8_t a, int16x8_t b )
{
  const int16x8_t  b2   = vaddq_s16( b, b );
  // (a + b)^2 - 2 b^2 > 0: not 0 degrees
  const uint16x8_t not0 = squareExceeds( vaddq_s16( a, b ), b, b2 );
  // a > b and (a - b)^2 - 2 b^2 > 0: 90 degrees
  const uint16x8_t is90 = vandq_u16( vcgtq_s16( a, b ), squareExceeds( vsubq_s16( a, b ), b, b2 ) );
  // 45 degrees when gx and gy have the same sign, else 135
  const uint16x8_t diffSign = vcltq_s16( veorq_s16( gx, gy ), vdupq_n_s16( 0 ) );
  int16x8_t dir = vbslq_s16( diffSign, vdupq_n_s16( 135 ), vdupq_n_s16( 45 ) );
  dir = vbslq_s16( is90, vdupq_n_s16( 90 ), dir );
  return vandq_s16( dir, vreinterpretq_s16_u16( not0 ) );
}

static void fgaSobel_neon( const Pel* above, const Pel* cur, const Pel* below, int width, Pel maxVal, Pel* mag, Pel* dir )
{
  const int16x8_t vmax = vdupq_n_s16( maxVal );
  int x = 1;
  for( ; x + 8 < width; x += 8 )
  {
    const int16x8_t al = vld1q_s16( above + x - 1 ), ac = vld1q_s16( above + x ), ar = vld1q_s16( above + x + 1 );
    const int16x8_t cl = vld1q_s16( cur + x - 1 ),                                 cr = vld1q_s16( cur + x + 1 );
    const int16x8_t bl = vld1q_s16( below + x - 1 ), bc = vld1q_s16( below + x ), br = vld1q_s16( below + x + 1 );
    const int16x8_t gx = vsubq_s16( vaddq_s16( vaddq_s16( bl, br ), vaddq_s16( bc, bc ) ),
                                    vaddq_s16( vaddq_s16( al, ar ), vaddq_s16( ac, ac ) ) );
    const int16x8_t gy = vsubq_s16( vaddq_s16( vaddq_s16( ar, br ), vaddq_s16( cr, cr ) ),
                                    vaddq_s16( vaddq_s16( al, bl ), vaddq_s16( cl, cl ) ) );
    const int16x8_t a  = vabsq_s16( gx );
    const int16x8_t b  = vabsq_s16( gy );
    vst1q_s16( mag + x, vminq_s16( vshrq_n_s16( vaddq_s16( a, b ), 1 ), vmax ) );
    vst1q_s16( dir + x, fgaDirection( gx, gy, a, b ) );
  }
  fgaSobelCore( above, cur, below, width, 0, 1, maxVal, mag, dir );
  fgaSobelCore( above, cur, below, width, x, width, maxVal, mag, dir );
}

static void fgaNonMax_neon( const Pel* magAbove, const Pel* mag, const Pel* magBelow, const Pel* dir, int width, Pel* dst )
{
  int x = 1;
  for( ; x + 8 < width; x += 8 )
  {
    const int16x8_t ml = vld1q_s16( mag + x - 1 ), mc = vld1q_s16( mag + x ), mr = vld1q_s16( mag + x + 1 );
    const int16x8_t al = vld1q_s16( magAbove + x - 1 ), ac = vld1q_s16( magAbove + x ), ar = vld1q_s16( magAbove + x + 1 );
    const int16x8_t bl = vld1q_s16( magBelow + x - 1 ), bc = vld1q_s16( magBelow + x ), br = vld1q_s16( magBelow + x + 1 );
    const int16x8_t d  = vld1q_s16( dir + x );
    const uint16x8_t is0  = vceqq_s16( d, vdupq_n_s16( 0 ) );
    const uint16x8_t is45 = vceqq_s16( d, vdupq_n_s16( 45 ) );
    const uint16x8_t is90 = vceqq_s16( d, vdupq_n_s16( 90 ) );
    // neighbours for 135 degrees, replaced for the other directions
    const int16x8_t n1 = vbslq_s16( is0, mr, vbslq_s16( is45, br, vbslq_s16( is90, bc, bl ) ) );
    const int16x8_t n2 = vbslq_s16( is0, ml, vbslq_s16( is45, al, vbslq_s16( is90, ac, ar ) ) );
    const uint16x8_t suppressed = vcgtq_s16( vmaxq_s16( n1, n2 ), mc );
    vst1q_s16( dst + x, vbicq_s16( mc, vreinterpretq_s16_u16( suppressed ) ) );
  }
  fgaNonMaxCore( magAbove, mag, magBelow, dir, width, 0, 1, dst );
  fgaNonMaxCore( magAbove, mag, magBelow, dir, width, x, width, dst );
}

static void fgaMorphRow_neon( const Pel* src, int width, bool dilate, Pel* dst )
{
  int x = 1;
  for( ; x + 8 < width; x += 8 )
  {
    const int16x8_t l = vld1q_s16( src + x - 1 ), c = vld1q_s16( src + x ), r = vld1q_s16( src + x + 1 );
    vst1q_s16( dst + x, dilate ? vmaxq_s16( vmaxq_s16( l, c ), r ) : vminq_s16( vminq_s16( l, c ), r ) );
  }
  fgaMorphRowCore( src, width, 0, 1, dilate, dst );
  fgaMorphRowCore( src, width, x, width, dilate, dst );
}

static void fgaMorphCol_neon( const Pel* above, const Pel* cur, const Pel* below, int width, bool dilate, Pel* dst )
{
  int x = 0;
  for( ; x + 8 <= width; x += 8 )
  {
    const int16x8_t a = vld1q_s16( above + x ), c = vld1q_s16( cur + x ), b = vld1q_s16( below + x );
    vst1q_s16( dst + x, dilate ? vmaxq_s16( vmaxq_s16( a, c ), b ) : vminq_s16( vminq_s16( a, c ), b ) );
  }
  fgaMorphColCore( above, cur, below, x, width, dilate, dst );
}

static Pel fgaRowMax_neon( const Pel* src, int n )
{
  int16x8_t m = vdupq_n_s16( 0 );
  int i = 0;
  for( ; i + 8 <= n; i += 8 )
  {
    m = vmaxq_s16( m, vld1q_s16( src + i ) );
  }
  return std::max( (Pel) maxAcrossLanes( m ), fgaRowMaxCore( src + i, n - i ) );
}

static void fgaClassify_neon( Pel* row, int n, Pel low, Pel high, Pel weak, Pel strong )
{
  const int16x8_t vl = vdupq_n_s16( low ), vh = vdupq_n_s16( high );
  const int16x8_t vw = vdupq_n_s16( weak ), vs = vdupq_n_s16( strong );
  int i = 0;
  for( ; i + 8 <= n; i += 8 )
  {
    const int16x8_t v = vld1q_s16( row + i );
    const int16x8_t w = vandq_s16( vreinterpretq_s16_u16( vcgtq_s16( v, vl ) ), vw );
    vst1q_s16( row + i, vbslq_s16( vcgtq_s16( v, vh ), vs, w ) );
  }
  fgaClassifyCore( row + i, n - i, low, high, weak, strong );
}

// Compares 8 samples at a time and visits only the lanes that match; most rows have few weak edges.
static int fgaFindEqual_neon( const Pel* row, int n, Pel value, int* positions )
{
  const int16x8_t v = vdupq_n_s16( value );
  int count = 0, i = 0;
  for( ; i + 8 <= n; i += 8 )
  {
    unsigned bits = laneBits( vceqq_s16( vld1q_s16( row + i ), v ) );
    while( bits )
    {
      positions[ count++ ] = i + lowestSetBit( bits );
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

static bool fgaIsZeroBlock_neon( const Pel* src, int stride )
{
  int16x8_t acc = vdupq_n_s16( 0 );
  for( int y = 0; y < 16; y++, src += stride )
  {
    acc = vorrq_s16( acc, vorrq_s16( vld1q_s16( src ), vld1q_s16( src + 8 ) ) );
  }
  const uint64x2_t any = vreinterpretq_u64_s16( acc );
  return ( vgetq_lane_u64( any, 0 ) | vgetq_lane_u64( any, 1 ) ) == 0;
}

// Every 32-bit lane of the square sums adds two squares per row over 16 rows, at most 32 * 4095^2 at 12 bits; the
// lanes are added as unsigned, as in the x86 kernel.
static void fgaBlockSums_neon( const Pel* src, int stride, int64_t& sum, int64_t& sumSq )
{
  int32x4_t s = vdupq_n_s32( 0 ), qLo = vdupq_n_s32( 0 ), qHi = vdupq_n_s32( 0 );
  for( int y = 0; y < 16; y++, src += stride )
  {
    const int16x8_t v0 = vld1q_s16( src ), v1 = vld1q_s16( src + 8 );
    s   = vpadalq_s16( vpadalq_s16( s, v0 ), v1 );
    qLo = vmlal_s16( vmlal_s16( qLo, vget_low_s16( v0 ), vget_low_s16( v0 ) ), vget_low_s16( v1 ), vget_low_s16( v1 ) );
    qHi = vmlal_s16( vmlal_s16( qHi, vget_high_s16( v0 ), vget_high_s16( v0 ) ), vget_high_s16( v1 ), vget_high_s16( v1 ) );
  }
  sum   = horizontal_add_s32x4( s );
  sumSq = (int64_t) ( horizontal_add_long_u32x4( vreinterpretq_u32_s32( qLo ) ) + horizontal_add_long_u32x4( vreinterpretq_u32_s32( qHi ) ) );
}

// 16x16 DCT with multiply-accumulate by a broadcast scalar: the horizontal stage adds sample k times basis column k
// to 16 frequencies, the vertical stage adds row k of the first stage times one basis value. Rounding, shifts,
// saturation and clipping are those of fgaSquaredDct16Core and the x86 kernel, so the result is identical.
static void fgaSquaredDct16_neon( const Pel* src, int stride, int maxRange, double* sum )
{
  constexpr int N = 16;
  const TMatrixCoeff* tr = g_trCoreDCT2P16[ TRANSFORM_FORWARD ][ 0 ];

  int16_t basisColumns[ N ][ N ];   // basisColumns[ k ][ u ] = T[ u ][ k ]
  for( int k = 0; k < N; k++ )
  {
    for( int u = 0; u < N; u++ )
    {
      basisColumns[ k ][ u ] = tr[ u * N + k ];
    }
  }

  int16_t g[ N ][ N ];   // first stage: g[ row ][ horizontal frequency ]
  for( int y = 0; y < N; y++ )
  {
    const Pel* row = src + y * stride;
    int32x4_t acc[ 4 ] = { vdupq_n_s32( 0 ), vdupq_n_s32( 0 ), vdupq_n_s32( 0 ), vdupq_n_s32( 0 ) };
    for( int k = 0; k < N; k++ )
    {
      for( int q = 0; q < 4; q++ )
      {
        acc[ q ] = vmlal_n_s16( acc[ q ], vld1_s16( &basisColumns[ k ][ 4 * q ] ), row[ k ] );
      }
    }
    // ( acc + 128 ) >> 8, saturated to 16 bits as the x86 pack does
    vst1q_s16( &g[ y ][ 0 ], vcombine_s16( vqrshrn_n_s32( acc[ 0 ], 8 ), vqrshrn_n_s32( acc[ 1 ], 8 ) ) );
    vst1q_s16( &g[ y ][ 8 ], vcombine_s16( vqrshrn_n_s32( acc[ 2 ], 8 ), vqrshrn_n_s32( acc[ 3 ], 8 ) ) );
  }

  const int32x4_t vmax = vdupq_n_s32( maxRange ), vmin = vdupq_n_s32( -maxRange );
  int32_t c[ N ];   // second stage, one vertical frequency
  for( int v = 0; v < N; v++ )
  {
    int32x4_t acc[ 4 ] = { vdupq_n_s32( 0 ), vdupq_n_s32( 0 ), vdupq_n_s32( 0 ), vdupq_n_s32( 0 ) };
    for( int k = 0; k < N; k++ )
    {
      const int16_t t = tr[ v * N + k ];
      for( int q = 0; q < 4; q++ )
      {
        acc[ q ] = vmlal_n_s16( acc[ q ], vld1_s16( &g[ k ][ 4 * q ] ), t );
      }
    }
    for( int q = 0; q < 4; q++ )
    {
      vst1q_s32( &c[ 4 * q ], vmaxq_s32( vmin, vminq_s32( vmax, vrshrq_n_s32( acc[ q ], 8 ) ) ) );
    }
    for( int u = 0; u < N; u++ )
    {
      sum[ u * N + v ] += (double) c[ u ] * c[ u ];
    }
  }
}

template<>
void FGAnalyzer::_initFGAnalyzerARM<NEON>( FgaRowKernels& k )
{
  k.sobel        = fgaSobel_neon;
  k.nonMax       = fgaNonMax_neon;
  k.morphRow     = fgaMorphRow_neon;
  k.morphCol     = fgaMorphCol_neon;
  k.rowMax       = fgaRowMax_neon;
  k.classify     = fgaClassify_neon;
  k.findEqual    = fgaFindEqual_neon;
  k.isZeroBlock  = fgaIsZeroBlock_neon;
  k.blockSums    = fgaBlockSums_neon;
  k.squaredDct16 = fgaSquaredDct16_neon;
}

} // namespace vvenc

#endif // TARGET_SIMD_ARM && ENABLE_SIMD_OPT_FGA

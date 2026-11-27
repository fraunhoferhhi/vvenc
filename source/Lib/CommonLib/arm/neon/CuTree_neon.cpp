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
 * \file CuTree_neon.cpp
 * \brief Neon kernels of the CU-tree look-ahead (EncoderLib/CuTree.cpp)
 */

#include "EncoderLib/CuTree.h"
#include "sum_neon.h"
#include "transpose_neon.h"

#include <algorithm>
#include <arm_neon.h>

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT

namespace vvenc
{

// The kernels work on samples of at most 10 bits (VVenC's internal bit depth is 8 or 10), as the AVX2 versions do.

// ====================================================================================================================
// Bilinear prediction and down-sampling
// ====================================================================================================================

// The horizontal taps of 8 samples, src[i] * ( 16 - fx ) + src[i + 1] * fx: at most 1023 * 16, inside 16 bits.
static inline int16x8_t tplPredRow_neon( const Pel* src, int16_t wx0, int16_t wx1 )
{
  const int16x8_t a = vld1q_s16( src );
  const int16x8_t b = vld1q_s16( src + 1 );
  return vmlaq_n_s16( vmulq_n_s16( a, wx0 ), b, wx1 );
}

// The vertical taps of two rows of horizontal taps in 32 bits, ( top * ( 16 - fy ) + bottom * fy + 128 ) >> 8.
static inline int16x8_t tplPredCol_neon( int16x8_t top, int16x8_t bot, int16_t wy0, int16_t wy1 )
{
  const int32x4_t lo = vmlal_n_s16( vmull_n_s16( vget_low_s16( top ), wy0 ), vget_low_s16( bot ), wy1 );
  const int32x4_t hi = vmlal_n_s16( vmull_n_s16( vget_high_s16( top ), wy0 ), vget_high_s16( bot ), wy1 );
  return vcombine_s16( vrshrn_n_s32( lo, 8 ), vrshrn_n_s32( hi, 8 ) );
}

// Neon version of tplPredBlockCore<8>, identical results. The horizontal taps of each reference row are the bottom
// of one output row and the top of the next.
static void tplPredBlock8x8_neon( const Pel* src, ptrdiff_t srcStride, int fx, int fy, Pel* dst )
{
  const int16_t wx0 = 16 - fx, wx1 = fx, wy0 = 16 - fy, wy1 = fy;
  int16x8_t top = tplPredRow_neon( src, wx0, wx1 );
  for( int j = 0; j < 8; j++ )
  {
    const int16x8_t bot = tplPredRow_neon( src + ( j + 1 ) * srcStride, wx0, wx1 );
    vst1q_s16( dst + j * 8, tplPredCol_neon( top, bot, wy0, wy1 ) );
    top = bot;
  }
}

// Neon version of tplPredBlockCore<16>, identical results, as tplPredBlock8x8_neon with two vectors per row.
static void tplPredBlock16x16_neon( const Pel* src, ptrdiff_t srcStride, int fx, int fy, Pel* dst )
{
  const int16_t wx0 = 16 - fx, wx1 = fx, wy0 = 16 - fy, wy1 = fy;
  int16x8_t top0 = tplPredRow_neon( src, wx0, wx1 );
  int16x8_t top1 = tplPredRow_neon( src + 8, wx0, wx1 );
  for( int j = 0; j < 16; j++ )
  {
    const Pel* row = src + ( j + 1 ) * srcStride;
    const int16x8_t bot0 = tplPredRow_neon( row, wx0, wx1 );
    const int16x8_t bot1 = tplPredRow_neon( row + 8, wx0, wx1 );
    vst1q_s16( dst + j * 16, tplPredCol_neon( top0, bot0, wy0, wy1 ) );
    vst1q_s16( dst + j * 16 + 8, tplPredCol_neon( top1, bot1, wy0, wy1 ) );
    top0 = bot0;
    top1 = bot1;
  }
}

// Neon version of tplDownsampleRowCore, identical results: 8 output samples per step, the rest by the C function.
static void tplDownsampleRow_neon( const Pel* src, ptrdiff_t srcStride, Pel* dst, int dstWidth )
{
  const Pel* below = src + srcStride;
  int x = 0;
  for( ; x + 8 <= dstWidth; x += 8 )
  {
    const int16x8_t top0 = vld1q_s16( src + 2 * x );
    const int16x8_t top1 = vld1q_s16( src + 2 * x + 8 );
    const int16x8_t bot0 = vld1q_s16( below + 2 * x );
    const int16x8_t bot1 = vld1q_s16( below + 2 * x + 8 );
    // vertical pairs, then horizontal pairs: sums of four samples, at most 4092
    const int16x8_t sum = pairwise_add_s16x8( vaddq_s16( top0, bot0 ), vaddq_s16( top1, bot1 ) );
    vst1q_s16( dst + x, vrshrq_n_s16( sum, 2 ) );
  }
  tplDownsampleRowCore( src + 2 * x, srcStride, dst + x, dstWidth - x );
}

// ====================================================================================================================
// Block coder
// ====================================================================================================================

// The first two stages of the 8-point Hadamard butterflies of tplCodeBlockCore across eight registers, one per row.
static inline void hadamard8Stages12_s16( const int16x8_t* v, int16x8_t* b )
{
  const int16x8_t a0 = vaddq_s16( v[0], v[4] ), a1 = vaddq_s16( v[1], v[5] );
  const int16x8_t a2 = vaddq_s16( v[2], v[6] ), a3 = vaddq_s16( v[3], v[7] );
  const int16x8_t a4 = vsubq_s16( v[0], v[4] ), a5 = vsubq_s16( v[1], v[5] );
  const int16x8_t a6 = vsubq_s16( v[2], v[6] ), a7 = vsubq_s16( v[3], v[7] );
  b[0] = vaddq_s16( a0, a2 ); b[1] = vaddq_s16( a1, a3 );
  b[2] = vsubq_s16( a0, a2 ); b[3] = vsubq_s16( a1, a3 );
  b[4] = vaddq_s16( a4, a6 ); b[5] = vaddq_s16( a5, a7 );
  b[6] = vsubq_s16( a4, a6 ); b[7] = vsubq_s16( a5, a7 );
}

// The 8-point Hadamard butterflies across eight registers, one per row: transforms the columns.
static inline void hadamard8_s16( int16x8_t* v )
{
  int16x8_t b[8];
  hadamard8Stages12_s16( v, b );
  v[0] = vaddq_s16( b[0], b[1] ); v[1] = vsubq_s16( b[0], b[1] );
  v[2] = vaddq_s16( b[2], b[3] ); v[3] = vsubq_s16( b[2], b[3] );
  v[4] = vaddq_s16( b[4], b[5] ); v[5] = vsubq_s16( b[4], b[5] );
  v[6] = vaddq_s16( b[6], b[7] ); v[7] = vsubq_s16( b[6], b[7] );
}

// The 16-point butterflies across sixteen registers: rows i and i + 8, then 8 points in each half.
static inline void hadamard16_s16( int16x8_t* v )
{
  for( int i = 0; i < 8; i++ )
  {
    const int16x8_t a = v[i], b = v[i + 8];
    v[i]     = vaddq_s16( a, b );
    v[i + 8] = vsubq_s16( a, b );
  }
  hadamard8_s16( v );
  hadamard8_s16( v + 8 );
}

// a + b and a - b of eight 16-bit lanes, widened to 32 bits: low half in sum[0] / dif[0], high half in [1].
static inline void butterflyWiden_s16( int16x8_t a, int16x8_t b, int32x4_t* sum, int32x4_t* dif )
{
  sum[0] = vaddl_s16( vget_low_s16( a ), vget_low_s16( b ) );
  sum[1] = vaddl_s16( vget_high_s16( a ), vget_high_s16( b ) );
  dif[0] = vsubl_s16( vget_low_s16( a ), vget_low_s16( b ) );
  dif[1] = vsubl_s16( vget_high_s16( a ), vget_high_s16( b ) );
}

// hadamard8_s16 with the last stage widened to 32 bits: output row m in c[2 m] (lanes 0-3) and c[2 m + 1] (4-7).
static inline void hadamard8Widen_s16( const int16x8_t* v, int32x4_t* c )
{
  int16x8_t b[8];
  hadamard8Stages12_s16( v, b );
  butterflyWiden_s16( b[0], b[1], c + 0,  c + 2  );
  butterflyWiden_s16( b[2], b[3], c + 4,  c + 6  );
  butterflyWiden_s16( b[4], b[5], c + 8,  c + 10 );
  butterflyWiden_s16( b[6], b[7], c + 12, c + 14 );
}

// The 8-point Hadamard butterflies on v[0], v[s], ..., v[7 s], 32 bits.
static inline void hadamard8_s32( int32x4_t* v, int s )
{
  const int32x4_t a0 = vaddq_s32( v[0], v[4 * s] ), a1 = vaddq_s32( v[s], v[5 * s] );
  const int32x4_t a2 = vaddq_s32( v[2 * s], v[6 * s] ), a3 = vaddq_s32( v[3 * s], v[7 * s] );
  const int32x4_t a4 = vsubq_s32( v[0], v[4 * s] ), a5 = vsubq_s32( v[s], v[5 * s] );
  const int32x4_t a6 = vsubq_s32( v[2 * s], v[6 * s] ), a7 = vsubq_s32( v[3 * s], v[7 * s] );
  const int32x4_t b0 = vaddq_s32( a0, a2 ), b1 = vaddq_s32( a1, a3 );
  const int32x4_t b2 = vsubq_s32( a0, a2 ), b3 = vsubq_s32( a1, a3 );
  const int32x4_t b4 = vaddq_s32( a4, a6 ), b5 = vaddq_s32( a5, a7 );
  const int32x4_t b6 = vsubq_s32( a4, a6 ), b7 = vsubq_s32( a5, a7 );
  v[0]     = vaddq_s32( b0, b1 ); v[s]     = vsubq_s32( b0, b1 );
  v[2 * s] = vaddq_s32( b2, b3 ); v[3 * s] = vsubq_s32( b2, b3 );
  v[4 * s] = vaddq_s32( b4, b5 ); v[5 * s] = vsubq_s32( b4, b5 );
  v[6 * s] = vaddq_s32( b6, b7 ); v[7 * s] = vsubq_s32( b6, b7 );
}

// The 16-point Hadamard butterflies on v[0], v[s], ..., v[15 s], 32 bits.
static inline void hadamard16_s32( int32x4_t* v, int s )
{
  for( int i = 0; i < 8; i++ )
  {
    const int32x4_t a = v[i * s], b = v[( i + 8 ) * s];
    v[i * s]         = vaddq_s32( a, b );
    v[( i + 8 ) * s] = vsubq_s32( a, b );
  }
  hadamard8_s32( v, s );
  hadamard8_s32( v + 8 * s, s );
}

// The 2-D transform of an 8x8 block of 32-bit values, row y in c[2 y] and c[2 y + 1]: columns, transposition, columns
// again. Like the forward kernel it gives the transposed transform, so applied to that kernel's output it gives 64 times
// the input in its original order.
static inline void hadamard2d8x8_s32( int32x4_t* c )
{
  hadamard8_s32( c,     2 );
  hadamard8_s32( c + 1, 2 );
  transpose_8x8_s32( c );
  hadamard8_s32( c,     2 );
  hadamard8_s32( c + 1, 2 );
}

// The columns of a 16x16 block of 32-bit values, row y in c[4 y .. 4 y + 3].
static inline void hadamard16Columns_s32( int32x4_t* c )
{
  hadamard16_s32( c,     4 );
  hadamard16_s32( c + 1, 4 );
  hadamard16_s32( c + 2, 4 );
  hadamard16_s32( c + 3, 4 );
}

// The 2-D transform of a 16x16 block of 32-bit values as hadamard2d8x8_s32 does it, row y in c[4 y .. 4 y + 3]:
// 256 times the input when applied to the forward kernel's output.
static inline void hadamard2d16x16_s32( int32x4_t* c )
{
  hadamard16Columns_s32( c );
  transpose_16x16_s32( c );
  hadamard16Columns_s32( c );
}

// Quantizes four coefficients as tplCodeBlockCore does: returns the magnitudes of their dequantized values, adds the
// squared coefficient errors to distLo (lanes 0-1) and distHi (lanes 2-3) and the estimated bits to bits.
static inline int32x4_t tplQuantize_neon( const int32x4_t c, const int32_t scale, const int32x4_t roundOffset,
                                          const int32_t deqStep, int64x2_t& distLo, int64x2_t& distHi, int32x4_t& bits )
{
  // level = ( |c| * scale + round ) >> 16, wrapping in 32 bits like the C code when |c| * scale overflows
  const int32x4_t absCoeff = vabsq_s32( c );
  const int32x4_t level    = vshrq_n_s32( vmlaq_n_s32( roundOffset, absCoeff, scale ), 16 );
  // |c'| = ( level * deqStep + 8 ) >> 4: level * deqStep stays below 2^23 (about 16 |c| without the wrap above, and
  // |level| < 2^15 with deqStep <= 128 when it wraps), so the rounding shift gives the same value
  const int32x4_t absDeq   = vrshrq_n_s32( vmulq_n_s32( level, deqStep ), 4 );
  const int32x4_t err      = vsubq_s32( absCoeff, absDeq );
  distLo = vmlal_s32( distLo, vget_low_s32( err ), vget_low_s32( err ) );
  distHi = vmlal_s32( distHi, vget_high_s32( err ), vget_high_s32( err ) );
  // 2 * floorLog2( level ) + 3 = 65 - 2 * clz( level ) bits for each level > 0
  const int32x4_t  length  = vmlsq_n_s32( vdupq_n_s32( 65 ), vclzq_s32( level ), 2 );
  const uint32x4_t nonZero = vcgtq_s32( level, vdupq_n_s32( 0 ) );
  bits = vaddq_s32( bits, vandq_s32( length, vreinterpretq_s32_u32( nonZero ) ) );
  return absDeq;
}

// -absDeq where c < 0, else absDeq.
static inline int32x4_t withSignOf_s32( const int32x4_t c, const int32x4_t absDeq )
{
  return vbslq_s32( vcltq_s32( c, vdupq_n_s32( 0 ) ), vnegq_s32( absDeq ), absDeq );
}

// Quantizes the NumVec coefficient vectors c: the squared coefficient error and the bits of the block, and when deq
// is given the dequantized coefficients.
template<int NumVec>
static void tplQuantizeBlock_neon( const int32x4_t* c, const TplQuant& quant, int32x4_t* deq, int64_t& dist, int& bits )
{
  const int32x4_t roundOffset = vdupq_n_s32( quant.round );
  int64x2_t distLo0 = vdupq_n_s64( 0 ), distHi0 = vdupq_n_s64( 0 );
  int64x2_t distLo1 = vdupq_n_s64( 0 ), distHi1 = vdupq_n_s64( 0 );
  int32x4_t bitSum  = vdupq_n_s32( 0 );
  for( int i = 0; i < NumVec; i += 2 )
  {
    const int32x4_t absDeq0 = tplQuantize_neon( c[i],     quant.scale, roundOffset, quant.deqStep, distLo0, distHi0, bitSum );
    const int32x4_t absDeq1 = tplQuantize_neon( c[i + 1], quant.scale, roundOffset, quant.deqStep, distLo1, distHi1, bitSum );
    if( deq )
    {
      deq[i]     = withSignOf_s32( c[i],     absDeq0 );
      deq[i + 1] = withSignOf_s32( c[i + 1], absDeq1 );
    }
  }
  dist = horizontal_add_s64x2( vaddq_s64( vaddq_s64( distLo0, distHi0 ), vaddq_s64( distLo1, distHi1 ) ) );
  bits = horizontal_add_s32x4( bitSum );
}

// recon = pred + ( ( res + 2^( SHIFT - 1 ) ) >> SHIFT ) clipped to [0, maxVal], for the N x N block res (32 bits, row
// y in res[y * N / 4 .. ]), SHIFT = 2 log2 N: the unnormalised transform applied twice scales by N^2.
template<int N>
static void tplStoreRecon_neon( const int32x4_t* res, const Pel* pred, Pel* recon, ptrdiff_t reconStride, int maxVal )
{
  constexpr int SHIFT = N == 16 ? 8 : 6;
  const uint16x8_t maxValVec = vdupq_n_u16( ( uint16_t ) maxVal );
  for( int y = 0; y < N; y++ )
  {
    for( int x = 0; x < N; x += 8 )
    {
      const uint16x8_t p = vld1q_u16( ( const uint16_t* ) pred + y * N + x );
      const int32x4_t* r = res + y * ( N / 4 ) + x / 4;
      // the rounded residual saturated to 16 bits, then added to pred with saturation at 0: a saturated residual is
      // beyond the reach of pred (0 ... maxVal), so the clipped sum stays exact
      const int16x8_t  d   = vcombine_s16( vqrshrn_n_s32( r[0], SHIFT ), vqrshrn_n_s32( r[1], SHIFT ) );
      const uint16x8_t sum = vminq_u16( vvenc_vsqaddq_u16( p, d ), maxValVec );
      vst1q_s16( recon + y * reconStride + x, vreinterpretq_s16_u16( sum ) );
    }
  }
}

// Neon version of tplCodeBlockCore<8>, identical results, with the coefficients transposed like the AVX2 version.
// |org - pred| <= 1023, so the column transform and the first two stages of the row transform stay in 16 bits (at
// most 32 * 1023); the last stage widens to 32 bits.
static void tplCodeBlock8x8_neon( const Pel* org, ptrdiff_t orgStride, const Pel* pred, Pel* recon, ptrdiff_t reconStride,
                                  const TplQuant& quant, int maxVal, int64_t& dist, int& bits )
{
  int16x8_t r[8];
  for( int y = 0; y < 8; y++ )
  {
    const int16x8_t o = vld1q_s16( org + y * orgStride );
    const int16x8_t p = vld1q_s16( pred + y * 8 );
    r[y] = vsubq_s16( o, p );
  }
  hadamard8_s16( r );
  transpose_8x8_s16( r );
  int32x4_t c[16], deq[16];
  hadamard8Widen_s16( r, c );
  tplQuantizeBlock_neon<16>( c, quant, recon ? deq : nullptr, dist, bits );
  if( recon )
  {
    hadamard2d8x8_s32( deq );
    tplStoreRecon_neon<8>( deq, pred, recon, reconStride, maxVal );
  }
}

// Neon version of tplCodeBlockCore<16>, identical results, with the coefficients transposed like the AVX2 version.
// |org - pred| <= 1023, so the column transform stays in 16 bits (at most 16 * 1023); the row transform widens to 32
// bits in its first stage.
static void tplCodeBlock16x16_neon( const Pel* org, ptrdiff_t orgStride, const Pel* pred, Pel* recon, ptrdiff_t reconStride,
                                    const TplQuant& quant, int maxVal, int64_t& dist, int& bits )
{
  // the residual rows, columns 0-7 in res0 and 8-15 in res1
  int16x8_t res0[16], res1[16];
  for( int y = 0; y < 16; y++ )
  {
    const int16x8_t o0 = vld1q_s16( org + y * orgStride );
    const int16x8_t o1 = vld1q_s16( org + y * orgStride + 8 );
    const int16x8_t p0 = vld1q_s16( pred + y * 16 );
    const int16x8_t p1 = vld1q_s16( pred + y * 16 + 8 );
    res0[y] = vsubq_s16( o0, p0 );
    res1[y] = vsubq_s16( o1, p1 );
  }
  hadamard16_s16( res0 );
  hadamard16_s16( res1 );
  // transposition by 8x8 quadrants (top left res0[0..7], top right res1[0..7], bottom left res0[8..15]): the
  // off-diagonal quadrants trade places
  transpose_8x8_s16( res0 );
  transpose_8x8_s16( res0 + 8 );
  transpose_8x8_s16( res1 );
  transpose_8x8_s16( res1 + 8 );
  std::swap_ranges( res1, res1 + 8, res0 + 8 );

  // the row transform: rows y and y + 8 widened, then 8 points in each half; row y in c[4 y .. 4 y + 3]
  int32x4_t c[64], deq[64];
  for( int y = 0; y < 8; y++ )
  {
    butterflyWiden_s16( res0[y], res0[y + 8], c + 4 * y,     c + 4 * y + 32 );
    butterflyWiden_s16( res1[y], res1[y + 8], c + 4 * y + 2, c + 4 * y + 34 );
  }
  hadamard8_s32( c,      4 );
  hadamard8_s32( c + 1,  4 );
  hadamard8_s32( c + 2,  4 );
  hadamard8_s32( c + 3,  4 );
  hadamard8_s32( c + 32, 4 );
  hadamard8_s32( c + 33, 4 );
  hadamard8_s32( c + 34, 4 );
  hadamard8_s32( c + 35, 4 );
  tplQuantizeBlock_neon<64>( c, quant, recon ? deq : nullptr, dist, bits );
  if( recon )
  {
    hadamard2d16x16_s32( deq );
    tplStoreRecon_neon<16>( deq, pred, recon, reconStride, maxVal );
  }
}

template<>
void CuTree::_initCuTreeARM<NEON>()
{
  m_codeBlock[TPL_BLK_8]  = tplCodeBlock8x8_neon;
  m_codeBlock[TPL_BLK_16] = tplCodeBlock16x16_neon;
  m_predBlock[TPL_BLK_8]  = tplPredBlock8x8_neon;
  m_predBlock[TPL_BLK_16] = tplPredBlock16x16_neon;
  m_downsampleRow         = tplDownsampleRow_neon;
}

} // namespace vvenc

#endif
//! \}

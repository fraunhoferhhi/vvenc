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

/** \file     Buffer_neon.cpp
    \brief    SIMD averaging.
*/

//! \ingroup CommonLib
//! \{

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP 1

#include "../CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Buffer.h"
#include "sum_neon.h"

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER

#if defined( _MSC_VER )
#pragma warning( disable : 4700 ) // MSVC equivalent of GCC_WARNING_DISABLE_maybe_uninitialized
#endif

namespace vvenc
{

// The averaging kernels compute ClipPel( ( src0 + src1 + offset ) >> rshift ) like addAvgCore, for any 16-bit inputs.
// The inputs are unclipped predictions, which overshoot the sample range at edges, so the sum can be negative or need
// more than 16 bits. Saturating additions keep it in 16 bits without changing the result: a sum that saturates high is
// clipped to the maximum anyway, and one that saturates low stays negative. vqshluq_n_s16( sum, 1 ) then clips
// negative sums to 0, and the logical shift right by rshift + 1 gives at most 32767 >> rshift, the maximum sample
// value.
static inline void checkAddAvgArgs( unsigned rshift, int offset, const ClpRng& clpRng )
{
  CHECK( offset < 0 || offset > INT16_MAX, "Offset must fit in 16 bits" );
  CHECK( ( INT16_MAX >> rshift ) != clpRng.max(), "rshift must be 15 - bitDepth" );
}

static inline int16x8_t addAvg_x8( int16x8_t src0, int16x8_t src1, int16x8_t offset, int16x8_t shift )
{
  int16x8_t sum = vqaddq_s16( vqaddq_s16( src0, src1 ), offset );
  return vreinterpretq_s16_u16( vshlq_u16( vqshluq_n_s16( sum, 1 ), shift ) );
}

static inline int16x4_t addAvg_x4( int16x4_t src0, int16x4_t src1, int16x4_t offset, int16x4_t shift )
{
  int16x4_t sum = vqadd_s16( vqadd_s16( src0, src1 ), offset );
  return vreinterpret_s16_u16( vshl_u16( vqshlu_n_s16( sum, 1 ), shift ) );
}

void addAvg_neon( const Pel* src0, const Pel* src1, Pel* dest, int numSamples, unsigned rshift, int offset,
                  const ClpRng& clpRng )
{
  CHECK( numSamples < 4, "numSamples must be >= 4" );
  checkAddAvgArgs( rshift, offset, clpRng );

  const int16x8_t voffset = vdupq_n_s16( offset );
  const int16x8_t vshift  = vdupq_n_s16( -static_cast<int>( rshift ) - 1 );

  if( ( numSamples & 15 ) == 0 )
  {
    int n = 0;
    do
    {
      int16x8_t s0_lo = vld1q_s16( src0 + n + 0 );
      int16x8_t s0_hi = vld1q_s16( src0 + n + 8 );
      int16x8_t s1_lo = vld1q_s16( src1 + n + 0 );
      int16x8_t s1_hi = vld1q_s16( src1 + n + 8 );

      int16x8_t d_lo = addAvg_x8( s0_lo, s1_lo, voffset, vshift );
      int16x8_t d_hi = addAvg_x8( s0_hi, s1_hi, voffset, vshift );

      vst1q_s16( dest + n + 0, d_lo );
      vst1q_s16( dest + n + 8, d_hi );

      n += 16;
    } while( n != numSamples );
  }
  else if( numSamples == 8 )
  {
    vst1q_s16( dest, addAvg_x8( vld1q_s16( src0 ), vld1q_s16( src1 ), voffset, vshift ) );
  }
  else if( numSamples == 4 )
  {
    vst1_s16( dest, addAvg_x4( vld1_s16( src0 ), vld1_s16( src1 ), vget_low_s16( voffset ), vget_low_s16( vshift ) ) );
  }
  else
  {
    THROW( "Unsupported size. numSamples must be 4 or 8 or a multiple of 16" );
  }
}

void recoCore_neon( const Pel* src0, const Pel* src1, Pel* dest, int numSamples, const ClpRng& clpRng )
{
  CHECKD( numSamples != 4 && numSamples != 8 && ( numSamples & 15 ) != 0,
          "numSamples must be 4 or 8 or a multiple of 16" );

  if( ( numSamples & 15 ) == 0 )
  {
    const uint16x8_t vbdmax = vdupq_n_u16( clpRng.max() );

    int x = 0;
    do
    {
      uint16x8_t s0 = vreinterpretq_u16_s16( vld1q_s16( src0 + x ) );
      uint16x8_t s2 = vreinterpretq_u16_s16( vld1q_s16( src0 + x + 8 ) );
      int16x8_t s1 = vld1q_s16( src1 + x );
      int16x8_t s3 = vld1q_s16( src1 + x + 8 );

      uint16x8_t sum0 = vvenc_vsqaddq_u16( s0, s1 );
      uint16x8_t sum1 = vvenc_vsqaddq_u16( s2, s3 );
      sum0 = vminq_u16( vbdmax, sum0 );
      sum1 = vminq_u16( vbdmax, sum1 );

      vst1q_s16( dest + x, vreinterpretq_s16_u16( sum0 ) );
      vst1q_s16( dest + x + 8, vreinterpretq_s16_u16( sum1 ) );

      x += 16;
    } while( x != numSamples );
  }
  else if( numSamples == 8 )
  {
    uint16x8_t s0 = vreinterpretq_u16_s16( vld1q_s16( src0 ) );
    int16x8_t s1 = vld1q_s16( src1 );

    uint16x8_t sum = vvenc_vsqaddq_u16( s0, s1 );
    sum = vminq_u16( vdupq_n_u16( clpRng.max() ), sum );

    vst1q_s16( dest, vreinterpretq_s16_u16( sum ) );
  }
  else // if( numSamples == 4 )
  {
    uint16x4_t s0 = vreinterpret_u16_s16( vld1_s16( src0 ) );
    int16x4_t s1 = vld1_s16( src1 );

    uint16x4_t sum = vvenc_vsqadd_u16( s0, s1 );
    sum = vmin_u16( vdup_n_u16( clpRng.max() ), sum );

    vst1_s16( dest, vreinterpret_s16_u16( sum ) );
  }
}

// copyBuffer, as copyBufferCore (one memcpy per row): whole-vector rows are copied with vector loads and stores, 64
// bytes per step, because for the small blocks the encoder copies most a memcpy call per row costs more than the copy.
static void copyBuffer_neon( const char* src, int srcStride, char* dst, int dstStride, int width, int height )
{
  if( width == srcStride && width == dstStride )
  {
    memcpy( dst, src, width * height );
  }
  else if( ( width & 15 ) == 0 )
  {
    for( ; height > 0; height--, src += srcStride, dst += dstStride )
    {
      const uint8_t* s = ( const uint8_t* ) src;
      uint8_t*       d = ( uint8_t* ) dst;
      int            x = 0;
      for( ; x + 64 <= width; x += 64 )
      {
        const uint8x16_t a = vld1q_u8( s + x );
        const uint8x16_t b = vld1q_u8( s + x + 16 );
        const uint8x16_t c = vld1q_u8( s + x + 32 );
        const uint8x16_t e = vld1q_u8( s + x + 48 );
        vst1q_u8( d + x, a );
        vst1q_u8( d + x + 16, b );
        vst1q_u8( d + x + 32, c );
        vst1q_u8( d + x + 48, e );
      }
      // the 0, 16, 32 or 48 bytes left
      if( width & 32 )
      {
        const uint8x16_t a = vld1q_u8( s + x );
        const uint8x16_t b = vld1q_u8( s + x + 16 );
        vst1q_u8( d + x, a );
        vst1q_u8( d + x + 16, b );
        x += 32;
      }
      if( width & 16 )
      {
        const uint8x16_t a = vld1q_u8( s + x );
        vst1q_u8( d + x, a );
      }
    }
  }
  else if( width == 8 )
  {
    for( ; height > 0; height--, src += srcStride, dst += dstStride )
    {
      const uint8x8_t a = vld1_u8( ( const uint8_t* ) src );
      vst1_u8( ( uint8_t* ) dst, a );
    }
  }
  else
  {
    for( ; height > 0; height--, src += srcStride, dst += dstStride )
    {
      memcpy( dst, src, width );
    }
  }
}

// Block statistics. Each kernel returns exactly its C core's result (Buffer.cpp): the sums are formed in wider lanes
// than any partial sum needs, and only the order of the integer additions differs.

// adds the absolute differences of absGradientSums_neon at the 8 positions from row, those outside mask as 0
static inline void absGradientStep_neon( const Pel* row, const Pel* below, const uint16x8_t mask, uint32x4_t& hor,
                                         uint32x4_t& ver, uint32x4_t& down, uint32x4_t& up )
{
  const int16x8_t a0 = vld1q_s16( row );
  const int16x8_t a1 = vld1q_s16( row + 1 );
  const int16x8_t b0 = vld1q_s16( below );
  const int16x8_t b1 = vld1q_s16( below + 1 );
  hor  = vpadalq_u16( hor, vandq_u16( vreinterpretq_u16_s16( vabdq_s16( a1, a0 ) ), mask ) );
  ver  = vpadalq_u16( ver, vandq_u16( vreinterpretq_u16_s16( vabdq_s16( b0, a0 ) ), mask ) );
  down = vpadalq_u16( down, vandq_u16( vreinterpretq_u16_s16( vabdq_s16( a1, b0 ) ), mask ) );
  up   = vpadalq_u16( up, vandq_u16( vreinterpretq_u16_s16( vabdq_s16( b1, a0 ) ), mask ) );
}

// Sums of the absolute differences between neighbouring samples in four directions over the ( width - 1 ) x
// ( height - 1 ) top-left positions, as absGradientSumsCore: sums[0] horizontal, sums[1] vertical, sums[2] down-left,
// sums[3] down-right. For samples in 0 ... 2^15 - 1 a difference fits 16 bits unsigned; vpadal adds the pairs of 8
// differences to 32-bit lanes. Each row ends with one vector ending at position width - 2 whose lanes counted before
// are masked off. Blocks narrower than 16 take the C core.
void absGradientSums_neon( const Pel* src, const ptrdiff_t stride, const int width, const int height, int* sums )
{
  if( width < 16 )
  {
    absGradientSumsCore( src, stride, width, height, sums );
    return;
  }

  static const uint16_t laneIndex16[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
  const uint16x8_t      laneIndex      = vld1q_u16( laneIndex16 );
  const uint16x8_t      allLanes       = vdupq_n_u16( 0xFFFF );
  const int             numPos         = width - 1;
  const int             last           = numPos - 8;

  uint32x4_t hor  = vdupq_n_u32( 0 );
  uint32x4_t ver  = vdupq_n_u32( 0 );
  uint32x4_t down = vdupq_n_u32( 0 );
  uint32x4_t up   = vdupq_n_u32( 0 );

  for( int y = 0; y < height - 1; y++, src += stride )
  {
    const Pel* below = src + stride;
    int        x     = 0;
    for( ; x + 8 <= numPos; x += 8 )
    {
      absGradientStep_neon( src + x, below + x, allLanes, hor, ver, down, up );
    }
    if( x < numPos )
    {
      const uint16x8_t firstNotCounted = vdupq_n_u16( static_cast<uint16_t>( x - last ) );
      absGradientStep_neon( src + last, below + last, vcgeq_u16( laneIndex, firstNotCounted ), hor, ver, down, up );
    }
  }

  vst1q_s32( sums, vreinterpretq_s32_u32( horizontal_add_4d_u32x4( hor, ver, down, up ) ) );
}

// The sum of the samples of a block, as sumCore: per row in 32-bit lanes (at most width * 2^15), then in 64 bit.
int64_t sum_neon( const Pel* src, const ptrdiff_t stride, const int width, const int height )
{
  int64x2_t acc  = vdupq_n_s64( 0 );
  int64_t   tail = 0;
  for( int y = 0; y < height; y++, src += stride )
  {
    int32x4_t rowSum = vdupq_n_s32( 0 );
    int       x      = 0;
    for( ; x + 8 <= width; x += 8 )
    {
      const int16x8_t s = vld1q_s16( src + x );
      rowSum            = vpadalq_s16( rowSum, s );
    }
    if( x + 4 <= width )
    {
      const int16x4_t s = vld1_s16( src + x );
      rowSum            = vaddw_s16( rowSum, s );
      x += 4;
    }
    acc = vpadalq_s32( acc, rowSum );
    for( ; x < width; x++ )
    {
      tail += src[x];
    }
  }
  return tail + horizontal_add_s64x2( acc );
}

// The sum of the samples of a block and of their squares, as sumAndSumSquaresCore. The sums as in sum_neon; each
// square is at most 2^30, so the two squares that vmull / vmlal add in a lane fit it read as unsigned 32 bit, and
// vpadal adds the pairs of lanes to 64 bit.
void sumAndSumSquares_neon( const Pel* src, const ptrdiff_t stride, const int width, const int height, int64_t& sum,
                            int64_t& sumSquares )
{
  int64x2_t  acc    = vdupq_n_s64( 0 );
  uint64x2_t accSq  = vdupq_n_u64( 0 );
  int64_t    tail   = 0;
  int64_t    tailSq = 0;
  for( int y = 0; y < height; y++, src += stride )
  {
    int32x4_t rowSum = vdupq_n_s32( 0 );
    int       x      = 0;
    for( ; x + 8 <= width; x += 8 )
    {
      const int16x8_t s     = vld1q_s16( src + x );
      const int32x4_t sqLo  = vmull_s16( vget_low_s16( s ), vget_low_s16( s ) );
      const int32x4_t sqSum = vmlal_s16( sqLo, vget_high_s16( s ), vget_high_s16( s ) );
      rowSum                = vpadalq_s16( rowSum, s );
      accSq                 = vpadalq_u32( accSq, vreinterpretq_u32_s32( sqSum ) );
    }
    if( x + 4 <= width )
    {
      const int16x4_t s  = vld1_s16( src + x );
      const int32x4_t sq = vmull_s16( s, s );
      rowSum             = vaddw_s16( rowSum, s );
      accSq              = vpadalq_u32( accSq, vreinterpretq_u32_s32( sq ) );
      x += 4;
    }
    acc = vpadalq_s32( acc, rowSum );
    for( ; x < width; x++ )
    {
      tail   += src[x];
      tailSq += src[x] * src[x];
    }
  }
  sum        = tail + horizontal_add_s64x2( acc );
  sumSquares = tailSq + static_cast<int64_t>( horizontal_add_u64x2( accSq ) );
}

// Statistics of a row of 4x4 blocks, as blockStats4x4Core: per block the sample sum s and sum( | s - 16 * p | ). For
// samples of up to 10 bits, s and 16 * p are at most 16368 and the four absolute deviations of a column at most
// 65472, so all fit 16 bits; two blocks per vector. Higher bit depths and an odd last block take the C core.
void blockStats4x4_neon( const Pel* src, const ptrdiff_t stride, const int numBlocks, const int bitDepth, int* sum,
                         int* absDev )
{
  if( bitDepth > 10 )
  {
    blockStats4x4Core( src, stride, numBlocks, bitDepth, sum, absDev );
    return;
  }

  int b = 0;
  for( ; b + 2 <= numBlocks; b += 2, src += 8 )
  {
    const int16x8_t row0 = vld1q_s16( src );
    const int16x8_t row1 = vld1q_s16( src + stride );
    const int16x8_t row2 = vld1q_s16( src + 2 * stride );
    const int16x8_t row3 = vld1q_s16( src + 3 * stride );

    const int16x8_t columns    = vaddq_s16( vaddq_s16( row0, row1 ), vaddq_s16( row2, row3 ) );
    const int32x4_t columnPair = vpaddlq_s16( columns );
    const int32x2_t blockSums  = vpadd_s32( vget_low_s32( columnPair ), vget_high_s32( columnPair ) );   // s0 s1

    // each block's sum in its four lanes
    const int16x4_t sums16 = vmovn_s32( vcombine_s32( blockSums, blockSums ) );   // s0 s1 s0 s1
    const int16x8_t s      = vcombine_s16( vdup_lane_s16( sums16, 0 ), vdup_lane_s16( sums16, 1 ) );

    uint16x8_t dev = vreinterpretq_u16_s16( vabdq_s16( s, vshlq_n_s16( row0, 4 ) ) );
    dev            = vvenc_vabaq_s16( dev, s, vshlq_n_s16( row1, 4 ) );
    dev            = vvenc_vabaq_s16( dev, s, vshlq_n_s16( row2, 4 ) );
    dev            = vvenc_vabaq_s16( dev, s, vshlq_n_s16( row3, 4 ) );
    const uint32x4_t devPair   = vpaddlq_u16( dev );
    const uint32x2_t blockDevs = vpadd_u32( vget_low_u32( devPair ), vget_high_u32( devPair ) );

    vst1_s32( sum + b, blockSums );
    vst1_s32( absDev + b, vreinterpret_s32_u32( blockDevs ) );
  }
  blockStats4x4Core( src, stride, numBlocks - b, bitDepth, sum + b, absDev + b );
}

// One row of 8 columns for the 3x3 high-pass: the centre samples and the sums of their left and right neighbours.
struct HighPassRow
{
  int16x8_t centre;
  int16x8_t sides;
};

static inline HighPassRow loadHighPassRow( const Pel* p )
{
  const int16x8_t left   = vld1q_s16( p - 1 );
  const int16x8_t right  = vld1q_s16( p + 1 );
  const int16x8_t centre = vld1q_s16( p );
  return { centre, vaddq_s16( left, right ) };
}

// The 3x3 high-pass 12 * c - 2 * ( 4 direct neighbours ) - ( 4 diagonal neighbours ) of the 8 samples of the middle
// row, from the rows above, at and below them; for samples of up to 10 bits every term and the result fit 16 bits
// (| hp | <= 12 * 1023).
static inline int16x8_t highPass3x3_neon( const HighPassRow& above, const HighPassRow& row, const HighPassRow& below )
{
  const int16x8_t direct   = vaddq_s16( row.sides, vaddq_s16( above.centre, below.centre ) );
  const int16x8_t diagonal = vaddq_s16( above.sides, below.sides );
  return vsubq_s16( vmlsq_n_s16( vmulq_n_s16( row.centre, 12 ), direct, 2 ), diagonal );
}

// The sum over the block of hp( cb ) * hp( cr ), as highPassCorrelationCore. Strips of 8 columns from top to bottom,
// so each row is loaded once and serves as the row above, at and below. A product is at most 12276^2 < 2^28, so the
// two products of a lane fit 32 bits; vpadal adds them to 64 bit. Higher bit depths take the C core, the last
// width % 8 columns too.
int64_t highPassCorrelation_neon( const Pel* cb, const ptrdiff_t cbStride, const Pel* cr, const ptrdiff_t crStride,
                                  const int width, const int height, const int bitDepth )
{
  if( bitDepth > 10 )
  {
    return highPassCorrelationCore( cb, cbStride, cr, crStride, width, height, bitDepth );
  }

  int64x2_t acc = vdupq_n_s64( 0 );
  int       x   = 0;
  for( ; x + 8 <= width; x += 8 )
  {
    HighPassRow cbAbove = loadHighPassRow( cb + x - cbStride );
    HighPassRow crAbove = loadHighPassRow( cr + x - crStride );
    HighPassRow cbRow   = loadHighPassRow( cb + x );
    HighPassRow crRow   = loadHighPassRow( cr + x );
    for( int y = 0; y < height; y++ )
    {
      const HighPassRow cbBelow = loadHighPassRow( cb + x + ( y + 1 ) * cbStride );
      const HighPassRow crBelow = loadHighPassRow( cr + x + ( y + 1 ) * crStride );
      const int16x8_t   hpCb    = highPass3x3_neon( cbAbove, cbRow, cbBelow );
      const int16x8_t   hpCr    = highPass3x3_neon( crAbove, crRow, crBelow );
      const int32x4_t   prodLo  = vmull_s16( vget_low_s16( hpCb ), vget_low_s16( hpCr ) );
      const int32x4_t   prodSum = vmlal_s16( prodLo, vget_high_s16( hpCb ), vget_high_s16( hpCr ) );
      acc                       = vpadalq_s32( acc, prodSum );

      cbAbove = cbRow;
      crAbove = crRow;
      cbRow   = cbBelow;
      crRow   = crBelow;
    }
  }
  const int64_t tail = highPassCorrelationCore( cb + x, cbStride, cr + x, crStride, width - x, height, bitDepth );
  return tail + horizontal_add_s64x2( acc );
}

// The 2x2 average ( a + b + c + d + 2 ) >> 2 of each 2x2 block, as downsample2x2Core: the pairs of a row added to
// 32 bit, those of the row below added on, then the rounding shift narrowing back to 16 bits, where the average of
// four 16-bit samples fits.
void downsample2x2_neon( const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride,
                         const int dstWidth, const int dstHeight )
{
  for( int y = 0; y < dstHeight; y++, src += 2 * srcStride, dst += dstStride )
  {
    const Pel* below = src + srcStride;
    int        x     = 0;
    for( ; x + 8 <= dstWidth; x += 8 )
    {
      const int16x8_t topLow     = vld1q_s16( src + 2 * x );
      const int16x8_t topHigh    = vld1q_s16( src + 2 * x + 8 );
      const int16x8_t bottomLow  = vld1q_s16( below + 2 * x );
      const int16x8_t bottomHigh = vld1q_s16( below + 2 * x + 8 );
      const int32x4_t sumsLow    = vpadalq_s16( vpaddlq_s16( topLow ), bottomLow );
      const int32x4_t sumsHigh   = vpadalq_s16( vpaddlq_s16( topHigh ), bottomHigh );
      vst1q_s16( dst + x, vcombine_s16( vrshrn_n_s32( sumsLow, 2 ), vrshrn_n_s32( sumsHigh, 2 ) ) );
    }
    for( ; x < dstWidth; x++ )
    {
      dst[x] = Pel( ( src[2 * x] + below[2 * x] + src[2 * x + 1] + below[2 * x + 1] + 2 ) >> 2 );
    }
  }
}

// CCLM's linear model, as linTfCore: dst = ( scale * src >> shift ) + offset, clipped to [0, max] with bClip. The
// product and the arithmetic shift are in 32 bit as there. With the clip, vqmovun narrows to 0..65535 and the min to
// max completes the C code's clip (ClpRng::min() is 0). Without it, vqmovn narrows with saturation like x86's packs,
// which equals the C code whenever the value fits 16 bits.
static inline int32x4_t linTf4Wide_neon( const int16x4_t src, const int32x4_t scale, const int32x4_t negShift,
                                         const int32x4_t offset )
{
  const int32x4_t product = vmulq_s32( vmovl_s16( src ), scale );
  return vaddq_s32( vshlq_s32( product, negShift ), offset );
}

static void linTf4_neon( const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height, int scale,
                         unsigned shift, int offset, const ClpRng& clpRng, bool bClip )
{
  CHECKD( width & 3, "Width must be a multiple of 4" );

  const int32x4_t vscale    = vdupq_n_s32( scale );
  const int32x4_t vnegShift = vdupq_n_s32( -static_cast<int>( shift ) );
  const int32x4_t voffset   = vdupq_n_s32( offset );

  if( bClip )
  {
    const uint16x4_t vmax = vdup_n_u16( clpRng.max() );
    for( int y = 0; y < height; y++, src += srcStride, dst += dstStride )
    {
      for( int x = 0; x < width; x += 4 )
      {
        const int16x4_t  s = vld1_s16( src + x );
        const uint16x4_t d = vqmovun_s32( linTf4Wide_neon( s, vscale, vnegShift, voffset ) );
        vst1_s16( dst + x, vreinterpret_s16_u16( vmin_u16( d, vmax ) ) );
      }
    }
  }
  else
  {
    for( int y = 0; y < height; y++, src += srcStride, dst += dstStride )
    {
      for( int x = 0; x < width; x += 4 )
      {
        const int16x4_t s = vld1_s16( src + x );
        vst1_s16( dst + x, vqmovn_s32( linTf4Wide_neon( s, vscale, vnegShift, voffset ) ) );
      }
    }
  }
}

static void linTf8_neon( const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height, int scale,
                         unsigned shift, int offset, const ClpRng& clpRng, bool bClip )
{
  CHECKD( width & 7, "Width must be a multiple of 8" );

  const int32x4_t vscale    = vdupq_n_s32( scale );
  const int32x4_t vnegShift = vdupq_n_s32( -static_cast<int>( shift ) );
  const int32x4_t voffset   = vdupq_n_s32( offset );

  if( bClip )
  {
    const uint16x8_t vmax = vdupq_n_u16( clpRng.max() );
    for( int y = 0; y < height; y++, src += srcStride, dst += dstStride )
    {
      for( int x = 0; x < width; x += 8 )
      {
        const int16x8_t  s    = vld1q_s16( src + x );
        const uint16x4_t low  = vqmovun_s32( linTf4Wide_neon( vget_low_s16( s ), vscale, vnegShift, voffset ) );
        const uint16x4_t high = vqmovun_s32( linTf4Wide_neon( vget_high_s16( s ), vscale, vnegShift, voffset ) );
        vst1q_s16( dst + x, vreinterpretq_s16_u16( vminq_u16( vcombine_u16( low, high ), vmax ) ) );
      }
    }
  }
  else
  {
    for( int y = 0; y < height; y++, src += srcStride, dst += dstStride )
    {
      for( int x = 0; x < width; x += 8 )
      {
        const int16x8_t s    = vld1q_s16( src + x );
        const int16x4_t low  = vqmovn_s32( linTf4Wide_neon( vget_low_s16( s ), vscale, vnegShift, voffset ) );
        const int16x4_t high = vqmovn_s32( linTf4Wide_neon( vget_high_s16( s ), vscale, vnegShift, voffset ) );
        vst1q_s16( dst + x, vcombine_s16( low, high ) );
      }
    }
  }
}

template<int W>
void addAvg_strided_neon( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dest, int destStride,
                          int width, int height, unsigned rshift, int offset, const ClpRng& clpRng );

template<>
void addAvg_strided_neon<16>( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dest,
                              int destStride, int width, int height, unsigned rshift, int offset, const ClpRng& clpRng )
{
  CHECK( height < 1, "Height must be >= 1" );
  CHECK( width < 16 || width & 15, "Width must be >= 16 and a multiple of 16" );
  checkAddAvgArgs( rshift, offset, clpRng );

  const int16x8_t voffset = vdupq_n_s16( offset );
  const int16x8_t vshift  = vdupq_n_s16( -static_cast<int>( rshift ) - 1 );

  do
  {
    int w = 0;
    do
    {
      int16x8_t s0_lo = vld1q_s16( src0 + w + 0 );
      int16x8_t s0_hi = vld1q_s16( src0 + w + 8 );
      int16x8_t s1_lo = vld1q_s16( src1 + w + 0 );
      int16x8_t s1_hi = vld1q_s16( src1 + w + 8 );

      int16x8_t d_lo = addAvg_x8( s0_lo, s1_lo, voffset, vshift );
      int16x8_t d_hi = addAvg_x8( s0_hi, s1_hi, voffset, vshift );

      vst1q_s16( dest + w + 0, d_lo );
      vst1q_s16( dest + w + 8, d_hi );

      w += 16;
    } while( w != width );

    src0 += src0Stride;
    src1 += src1Stride;
    dest += destStride;
  } while( --height != 0 );
}

template<>
void addAvg_strided_neon<8>( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dest,
                             int destStride, int width, int height, unsigned rshift, int offset, const ClpRng& clpRng )
{
  CHECK( height < 1, "Height must be >= 1" );
  CHECK( width < 8 || width & 7, "Width must be >= 8 and a multiple of 8" );
  checkAddAvgArgs( rshift, offset, clpRng );

  const int16x8_t voffset = vdupq_n_s16( offset );
  const int16x8_t vshift  = vdupq_n_s16( -static_cast<int>( rshift ) - 1 );

  do
  {
    int w = 0;
    do
    {
      int16x8_t d = addAvg_x8( vld1q_s16( src0 + w ), vld1q_s16( src1 + w ), voffset, vshift );
      vst1q_s16( dest + w, d );

      w += 8;
    } while( w != width );

    src0 += src0Stride;
    src1 += src1Stride;
    dest += destStride;
  } while( --height != 0 );
}

template<>
void addAvg_strided_neon<4>( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dest,
                             int destStride, int width, int height, unsigned rshift, int offset, const ClpRng& clpRng )
{
  CHECK( height < 1, "Height must be >= 1" );
  CHECK( width < 4 || width & 3, "Width must be >= 4 and a multiple of 4" );
  checkAddAvgArgs( rshift, offset, clpRng );

  const int16x4_t voffset = vdup_n_s16( offset );
  const int16x4_t vshift  = vdup_n_s16( -static_cast<int>( rshift ) - 1 );

  do
  {
    int w = 0;
    do
    {
      int16x4_t d = addAvg_x4( vld1_s16( src0 + w ), vld1_s16( src1 + w ), voffset, vshift );
      vst1_s16( dest + w, d );

      w += 4;
    } while( w != width );

    src0 += src0Stride;
    src1 += src1Stride;
    dest += destStride;
  } while( --height != 0 );
}

template<>
void PelBufferOps::_initPelBufOpsARM<NEON>()
{
  addAvg   = addAvg_neon;
  reco     = recoCore_neon;
  copyBuffer = copyBuffer_neon;
  absGradientSums     = absGradientSums_neon;
  sum                 = sum_neon;
  sumAndSumSquares    = sumAndSumSquares_neon;
  blockStats4x4       = blockStats4x4_neon;
  highPassCorrelation = highPassCorrelation_neon;
  downsample2x2       = downsample2x2_neon;
  linTf4              = linTf4_neon;
  linTf8              = linTf8_neon;
  addAvg4  = addAvg_strided_neon<4>;
  addAvg8  = addAvg_strided_neon<8>;
  addAvg16 = addAvg_strided_neon<16>;
}

} // namespace vvenc

#if defined( _MSC_VER )
#pragma warning( default : 4700 )
#endif

#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER
//! \}

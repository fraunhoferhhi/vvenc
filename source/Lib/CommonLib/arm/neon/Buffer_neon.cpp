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

// The QPA visual activity sums (BitAllocation.cpp). Their C cores in Buffer.cpp take the blocks too narrow for a
// vector.
uint64_t AvgHighPassCore( const int width, const int height, const Pel* pSrc, const int iSrcStride );
uint64_t HDHighPassCore( const int width, const int height, const Pel* pSrc, const Pel* pSM1, const int iSrcStride,
                         const int iSM1Stride );
uint64_t HDHighPass2Core( const int width, const int height, const Pel* pSrc, const Pel* pSM1, const Pel* pSM2,
                          const int iSrcStride, const int iSM1Stride, const int iSM2Stride );
uint64_t AvgHighPassWithDownsamplingCore( const int width, const int height, const Pel* pSrc, const int iSrcStride );
uint64_t AvgHighPassWithDownsamplingDiff1stCore( const int width, const int height, const Pel* pSrc, const Pel* pSrcM1,
                                                 const int iSrcStride, const int iSrcM1Stride );
uint64_t AvgHighPassWithDownsamplingDiff2ndCore( const int width, const int height, const Pel* pSrc, const Pel* pSrcM1,
                                                 const Pel* pSrcM2, const int iSrcStride, const int iSM1Stride,
                                                 const int iSM2Stride );

// The QPA kernels below (but HDHighPass) return the sums of their C cores for picture samples, which lie in [0, 4095]
// (at most 12 bit). Each filter's positive and negative parts are sums of samples small enough for uint16 lanes
// (uint32 for the downsampled spatial filter), and the filter's absolute value is the absolute difference of the two
// parts (then ( 1 + 3 t ) >> 1 for the first-order temporal ones). Each row ends with one vector that overlaps the one
// before it so that it ends at the row's last output; the lanes the vectors before it already summed are masked off.

static const uint16_t qpaLaneIndex16[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
static const uint32_t qpaLaneIndex32[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };

// | 12 c - 2 ( l + r + u + d ) - ( the four diagonal neighbours ) | for the 8 samples at src: 12 c <= 49140 and
// 2 ( l + r + u + d ) + ( diagonals ) <= 49140
static inline uint16x8_t avgHighPassAbs_neon( const uint16_t* src, const int stride )
{
  const uint16x8_t aboveLeft  = vld1q_u16( src - stride - 1 );
  const uint16x8_t above      = vld1q_u16( src - stride );
  const uint16x8_t aboveRight = vld1q_u16( src - stride + 1 );
  const uint16x8_t left       = vld1q_u16( src - 1 );
  const uint16x8_t centre     = vld1q_u16( src );
  const uint16x8_t right      = vld1q_u16( src + 1 );
  const uint16x8_t belowLeft  = vld1q_u16( src + stride - 1 );
  const uint16x8_t below      = vld1q_u16( src + stride );
  const uint16x8_t belowRight = vld1q_u16( src + stride + 1 );

  const uint16x8_t cross = vaddq_u16( vaddq_u16( left, right ), vaddq_u16( above, below ) );
  const uint16x8_t diag  = vaddq_u16( vaddq_u16( aboveLeft, aboveRight ), vaddq_u16( belowLeft, belowRight ) );
  const uint16x8_t pos   = vmulq_n_u16( centre, 12 );
  const uint16x8_t neg   = vmlaq_n_u16( diag, cross, 2 );
  return vabdq_u16( pos, neg );
}

static uint64_t AvgHighPass_neon( const int width, const int height, const Pel* pSrc, const int iSrcStride )
{
  if( width < 10 )
  {
    return AvgHighPassCore( width, height, pSrc, iSrcStride );
  }

  const int       last = width - 9;   // the last vector takes columns last .. width - 2
  const uint16_t* src  = reinterpret_cast<const uint16_t*>( pSrc );
  uint64x2_t      sum  = vdupq_n_u64( 0 );
  const uint16x8_t laneIndex = vld1q_u16( qpaLaneIndex16 );

  for( int y = 0; y < height - 2; y++ )   // the C core's rows: centred on rows 0 .. height - 3
  {
    const uint16_t* row    = src + y * iSrcStride;
    uint32x4_t      rowSum = vdupq_n_u32( 0 );
    int             x      = 1;
    for( ; x < last; x += 8 )
    {
      rowSum = vpadalq_u16( rowSum, avgHighPassAbs_neon( row + x, iSrcStride ) );
    }
    const uint16x8_t notSummed = vcgeq_u16( laneIndex, vdupq_n_u16( static_cast<uint16_t>( x - last ) ) );
    rowSum = vpadalq_u16( rowSum, vandq_u16( avgHighPassAbs_neon( row + last, iSrcStride ), notSummed ) );
    sum    = vpadalq_u32( sum, rowSum );
  }
  return horizontal_add_u64x2( sum );
}

// HDHighPass as the x86 kernel (HDHighPass_SIMD) computes it, which the encoder ran on every SIMD target: it adds the
// | s - m1 | of each column pair 1 + 2 i, 2 + 2 i and takes ( 1 + 3 ( | t0 | + | t1 | ) ) >> 1 per pair, where the C
// core takes ( 1 + 3 | t | ) >> 1 per column. ( 1 + 3 p ) >> 1 = ( p + 2 p + 1 ) >> 1 is a rounding halving add. Equal
// to the x86 kernel for samples up to 11 bit (x86 adds four pairs with int16 saturation) and blocks at least 9 wide
// (narrower ones take the C core). The last vector starts at an odd column, so that it pairs the columns as the x86
// kernel does; its lanes past column width - 2 and those the vectors before it summed are zeroed before the pairs are
// added.
static inline uint16x8_t hdHighPassAbsDiff_neon( const uint16_t* src, const uint16_t* srcM1 )
{
  const uint16x8_t s  = vld1q_u16( src );
  const uint16x8_t m1 = vld1q_u16( srcM1 );
  return vabdq_u16( s, m1 );
}

// ( 1 + 3 ( | t0 | + | t1 | ) ) >> 1 for each lane pair of absDiff
static inline uint32x4_t hdHighPassPairs_neon( const uint16x8_t absDiff )
{
  const uint32x4_t p = vpaddlq_u16( absDiff );
  return vrhaddq_u32( p, vshlq_n_u32( p, 1 ) );
}

static uint64_t HDHighPass_neon( const int width, const int height, const Pel* pSrc, const Pel* pSM1,
                                 const int iSrcStride, const int iSM1Stride )
{
  if( width < 9 )
  {
    return HDHighPassCore( width, height, pSrc, pSM1, iSrcStride, iSM1Stride );
  }

  const int        last      = ( width - 9 ) | 1;   // odd, last + 7 <= width - 1
  const uint16x8_t laneIndex = vld1q_u16( qpaLaneIndex16 );
  const uint16x8_t inBlock   = vcleq_u16( laneIndex, vdupq_n_u16( static_cast<uint16_t>( width - 2 - last ) ) );
  uint64x2_t       sum       = vdupq_n_u64( 0 );

  for( int y = 0; y < height - 2; y++ )
  {
    const uint16_t* src    = reinterpret_cast<const uint16_t*>( pSrc ) + y * iSrcStride;
    const uint16_t* srcM1  = reinterpret_cast<const uint16_t*>( pSM1 ) + y * iSM1Stride;
    uint32x4_t      rowSum = vdupq_n_u32( 0 );
    int             x      = 1;
    for( ; x < last; x += 8 )
    {
      rowSum = vaddq_u32( rowSum, hdHighPassPairs_neon( hdHighPassAbsDiff_neon( src + x, srcM1 + x ) ) );
    }
    const uint16x8_t firstNotSummed = vdupq_n_u16( static_cast<uint16_t>( x - last ) );
    const uint16x8_t notSummed      = vandq_u16( vcgeq_u16( laneIndex, firstNotSummed ), inBlock );
    const uint16x8_t lastAbsDiff    = vandq_u16( hdHighPassAbsDiff_neon( src + last, srcM1 + last ), notSummed );
    rowSum = vaddq_u32( rowSum, hdHighPassPairs_neon( lastAbsDiff ) );
    sum    = vpadalq_u32( sum, rowSum );
  }
  return horizontal_add_u64x2( sum );
}

// | s - 2 m1 + m2 | for 8 samples, as | ( s + m2 ) - 2 m1 | with both parts <= 8190
static inline uint16x8_t hdHighPass2_neon( const uint16_t* src, const uint16_t* srcM1, const uint16_t* srcM2 )
{
  const uint16x8_t s  = vld1q_u16( src );
  const uint16x8_t m1 = vld1q_u16( srcM1 );
  const uint16x8_t m2 = vld1q_u16( srcM2 );
  return vabdq_u16( vaddq_u16( s, m2 ), vshlq_n_u16( m1, 1 ) );
}

static uint64_t HDHighPass2_neon( const int width, const int height, const Pel* pSrc, const Pel* pSM1, const Pel* pSM2,
                                  const int iSrcStride, const int iSM1Stride, const int iSM2Stride )
{
  if( width < 10 )
  {
    return HDHighPass2Core( width, height, pSrc, pSM1, pSM2, iSrcStride, iSM1Stride, iSM2Stride );
  }

  const int  last = width - 9;
  uint64x2_t sum  = vdupq_n_u64( 0 );
  const uint16x8_t laneIndex = vld1q_u16( qpaLaneIndex16 );

  for( int y = 0; y < height - 2; y++ )
  {
    const uint16_t* src    = reinterpret_cast<const uint16_t*>( pSrc ) + y * iSrcStride;
    const uint16_t* srcM1  = reinterpret_cast<const uint16_t*>( pSM1 ) + y * iSM1Stride;
    const uint16_t* srcM2  = reinterpret_cast<const uint16_t*>( pSM2 ) + y * iSM2Stride;
    uint32x4_t      rowSum = vdupq_n_u32( 0 );
    int             x      = 1;
    for( ; x < last; x += 8 )
    {
      rowSum = vpadalq_u16( rowSum, hdHighPass2_neon( src + x, srcM1 + x, srcM2 + x ) );
    }
    const uint16x8_t notSummed = vcgeq_u16( laneIndex, vdupq_n_u16( static_cast<uint16_t>( x - last ) ) );
    rowSum = vpadalq_u16( rowSum, vandq_u16( hdHighPass2_neon( src + last, srcM1 + last, srcM2 + last ), notSummed ) );
    sum    = vpadalq_u32( sum, rowSum );
  }
  return horizontal_add_u64x2( sum );
}

// The downsampled kernels (UHD) take one output per 2x2 block, at the even x from 2 and the even y from 2 (rows counted
// from pSrc - 2 rows, as in the C cores). A vector takes the 8 outputs x, x + 2, .., x + 14, from a row's samples
// x .. x + 15 and their neighbours. Each row's last vector starts at ( width - 17 ) & ~1, so that it ends at the row's
// last output; the spatial filter then reads up to column width - 1 for an even width (width for an odd one, as the C
// core).

// the sums of the 2x2 blocks of the 8 outputs at rowA (the blocks' top row) and rowB (their bottom row): the column
// sums of the two rows, then the sums of adjacent columns, <= 16380 (int16 for the pairwise add)
static inline uint16x8_t sum2x2_neon( const uint16_t* rowA, const uint16_t* rowB )
{
  const uint16x8_t topLeft     = vld1q_u16( rowA );
  const uint16x8_t topRight    = vld1q_u16( rowA + 8 );
  const uint16x8_t bottomLeft  = vld1q_u16( rowB );
  const uint16x8_t bottomRight = vld1q_u16( rowB + 8 );
  const int16x8_t  columnsLeft  = vreinterpretq_s16_u16( vaddq_u16( topLeft, bottomLeft ) );
  const int16x8_t  columnsRight = vreinterpretq_s16_u16( vaddq_u16( topRight, bottomRight ) );
  return vreinterpretq_u16_s16( pairwise_add_s16x8( columnsLeft, columnsRight ) );
}

// One row of the spatial filter's window around the 8 outputs at x: per output, the samples at x - 2 (outerLeft),
// x - 1 (left), x and x + 1 (centre: their sum), x + 2 (right) and x + 3 (outerRight): vuzpq splits the samples
// into even and odd columns, and the pairwise add sums each pair of columns x, x + 1
struct QpaWindowRow
{
  uint16x8_t outerLeft, left, centre, right, outerRight;
};

static inline QpaWindowRow qpaWindowRow_neon( const uint16_t* row )
{
  const uint16x8_t beforeLow  = vld1q_u16( row - 2 );   // samples x - 2 .. x + 5
  const uint16x8_t beforeHigh = vld1q_u16( row + 6 );   // x + 6 .. x + 13
  const uint16x8_t midLow     = vld1q_u16( row );       // x .. x + 7
  const uint16x8_t midHigh    = vld1q_u16( row + 8 );   // x + 8 .. x + 15
  const uint16x8_t afterLow   = vld1q_u16( row + 2 );   // x + 2 .. x + 9
  const uint16x8_t afterHigh  = vld1q_u16( row + 10 );  // x + 10 .. x + 17
  const uint16x8x2_t before = vuzpq_u16( beforeLow, beforeHigh );   // even: x - 2, odd: x - 1
  const uint16x8x2_t after  = vuzpq_u16( afterLow, afterHigh );     // even: x + 2, odd: x + 3
  QpaWindowRow r;
  r.outerLeft  = before.val[0];
  r.left       = before.val[1];
  r.centre     = vreinterpretq_u16_s16( pairwise_add_s16x8( vreinterpretq_s16_u16( midLow ),
                                                         vreinterpretq_s16_u16( midHigh ) ) );   // <= 8190
  r.right      = after.val[0];
  r.outerRight = after.val[1];
  return r;
}

// | f | of AvgHighPassWithDownsamplingCore for the 8 outputs at src (row y, column x), in two uint32x4 halves:
// f = 12 G12 - ( 3 G3 + 2 G2 + G1 ), where G12 sums the 2x2 block (4 samples), G3 its 8 nearest neighbours, G2 the 4
// diagonal ones and G1 the 16 samples around those. 12 G12 <= 196560 and 3 G3 + 2 G2 + G1 <= 196560 need uint32;
// 2 G2 plus the 8 samples of G1 in rows y - 2 and y + 3 are <= 65520 and fit uint16.
static inline uint32x4x2_t downsampledHighPassAbs_neon( const uint16_t* src, const int stride )
{
  const QpaWindowRow m2 = qpaWindowRow_neon( src - 2 * stride );
  const QpaWindowRow m1 = qpaWindowRow_neon( src - stride );
  const QpaWindowRow r0 = qpaWindowRow_neon( src );
  const QpaWindowRow p1 = qpaWindowRow_neon( src + stride );
  const QpaWindowRow p2 = qpaWindowRow_neon( src + 2 * stride );
  const QpaWindowRow p3 = qpaWindowRow_neon( src + 3 * stride );

  const uint16x8_t g12 = vaddq_u16( r0.centre, p1.centre );
  const uint16x8_t g3  = vaddq_u16( vaddq_u16( m1.centre, p2.centre ),
                                    vaddq_u16( vaddq_u16( r0.left, r0.right ), vaddq_u16( p1.left, p1.right ) ) );
  const uint16x8_t g2  = vaddq_u16( vaddq_u16( m1.left, m1.right ), vaddq_u16( p2.left, p2.right ) );
  const uint16x8_t g1Rows   = vaddq_u16( vaddq_u16( vaddq_u16( m2.left, m2.right ), m2.centre ),
                                         vaddq_u16( vaddq_u16( p3.left, p3.right ), p3.centre ) );
  const uint16x8_t g1Top    = vaddq_u16( vaddq_u16( m1.outerLeft, m1.outerRight ),
                                         vaddq_u16( r0.outerLeft, r0.outerRight ) );
  const uint16x8_t g1Bottom = vaddq_u16( vaddq_u16( p1.outerLeft, p1.outerRight ),
                                         vaddq_u16( p2.outerLeft, p2.outerRight ) );
  const uint16x8_t g1Cols   = vaddq_u16( g1Top, g1Bottom );
  const uint16x8_t g2g1Rows = vmlaq_n_u16( g1Rows, g2, 2 );

  const uint32x4_t posLo = vmull_n_u16( vget_low_u16( g12 ), 12 );
  const uint32x4_t posHi = vmull_n_u16( vget_high_u16( g12 ), 12 );
  const uint32x4_t g12Lo = vaddl_u16( vget_low_u16( g2g1Rows ), vget_low_u16( g1Cols ) );    // 2 G2 + G1
  const uint32x4_t g12Hi = vaddl_u16( vget_high_u16( g2g1Rows ), vget_high_u16( g1Cols ) );
  const uint32x4_t negLo = vmlal_n_u16( g12Lo, vget_low_u16( g3 ), 3 );
  const uint32x4_t negHi = vmlal_n_u16( g12Hi, vget_high_u16( g3 ), 3 );

  uint32x4x2_t absF;
  absF.val[0] = vabdq_u32( posLo, negLo );
  absF.val[1] = vabdq_u32( posHi, negHi );
  return absF;
}

static uint64_t AvgHighPassWithDownsampling_neon( const int width, const int height, const Pel* pSrc,
                                                  const int iSrcStride )
{
  if( width < 19 )
  {
    return AvgHighPassWithDownsamplingCore( width, height, pSrc, iSrcStride );
  }

  const int       last = ( width - 17 ) & ~1;
  const uint16_t* base = reinterpret_cast<const uint16_t*>( pSrc ) - 2 * iSrcStride;
  uint64x2_t      sum  = vdupq_n_u64( 0 );
  const uint32x4_t laneIndexLo = vld1q_u32( qpaLaneIndex32 );
  const uint32x4_t laneIndexHi = vld1q_u32( qpaLaneIndex32 + 4 );

  for( int y = 2; y < height - 2; y += 2 )
  {
    const uint16_t* row    = base + y * iSrcStride;
    uint32x4_t      rowSum = vdupq_n_u32( 0 );
    int             x      = 2;
    for( ; x < last; x += 16 )
    {
      const uint32x4x2_t absF = downsampledHighPassAbs_neon( row + x, iSrcStride );
      rowSum = vaddq_u32( rowSum, vaddq_u32( absF.val[0], absF.val[1] ) );
    }
    const uint32x4_t   firstNotSummed = vdupq_n_u32( static_cast<uint32_t>( ( x - last ) >> 1 ) );
    const uint32x4_t   notSummedLo    = vcgeq_u32( laneIndexLo, firstNotSummed );
    const uint32x4_t   notSummedHi    = vcgeq_u32( laneIndexHi, firstNotSummed );
    const uint32x4x2_t absF           = downsampledHighPassAbs_neon( row + last, iSrcStride );
    rowSum = vaddq_u32( rowSum, vandq_u32( absF.val[0], notSummedLo ) );
    rowSum = vaddq_u32( rowSum, vandq_u32( absF.val[1], notSummedHi ) );
    sum    = vpadalq_u32( sum, rowSum );
  }
  return horizontal_add_u64x2( sum );
}

// ( 1 + 3 | S - S1 | ) >> 1 <= 24570 for the 2x2 sums S of the picture and S1 of the previous one, as
// ( t + 2 t + 1 ) >> 1
static inline uint16x8_t downsampledDiff1st_neon( const uint16_t* src, const int srcStride, const uint16_t* srcM1,
                                                  const int srcM1Stride )
{
  const uint16x8_t s  = sum2x2_neon( src, src + srcStride );
  const uint16x8_t m1 = sum2x2_neon( srcM1, srcM1 + srcM1Stride );
  const uint16x8_t t  = vabdq_u16( s, m1 );
  return vrhaddq_u16( t, vshlq_n_u16( t, 1 ) );
}

static uint64_t AvgHighPassWithDownsamplingDiff1st_neon( const int width, const int height, const Pel* pSrc,
                                                         const Pel* pSrcM1, const int iSrcStride,
                                                         const int iSrcM1Stride )
{
  if( width < 19 )
  {
    return AvgHighPassWithDownsamplingDiff1stCore( width, height, pSrc, pSrcM1, iSrcStride, iSrcM1Stride );
  }

  const int       last   = ( width - 17 ) & ~1;
  const uint16_t* base   = reinterpret_cast<const uint16_t*>( pSrc ) - 2 * iSrcStride;
  const uint16_t* baseM1 = reinterpret_cast<const uint16_t*>( pSrcM1 ) - 2 * iSrcM1Stride;
  uint64x2_t      sum    = vdupq_n_u64( 0 );
  const uint16x8_t laneIndex = vld1q_u16( qpaLaneIndex16 );

  for( int y = 2; y < height - 2; y += 2 )
  {
    const uint16_t* src    = base + y * iSrcStride;
    const uint16_t* srcM1  = baseM1 + y * iSrcM1Stride;
    uint32x4_t      rowSum = vdupq_n_u32( 0 );
    int             x      = 2;
    for( ; x < last; x += 16 )
    {
      rowSum = vpadalq_u16( rowSum, downsampledDiff1st_neon( src + x, iSrcStride, srcM1 + x, iSrcM1Stride ) );
    }
    const uint16x8_t notSummed = vcgeq_u16( laneIndex, vdupq_n_u16( static_cast<uint16_t>( ( x - last ) >> 1 ) ) );
    const uint16x8_t lastDiff  = downsampledDiff1st_neon( src + last, iSrcStride, srcM1 + last, iSrcM1Stride );
    rowSum = vpadalq_u16( rowSum, vandq_u16( lastDiff, notSummed ) );
    sum    = vpadalq_u32( sum, rowSum );
  }
  return horizontal_add_u64x2( sum );
}

// | ( S + S2 ) - 2 S1 | for the 2x2 sums of the picture and the two before it, both parts <= 32760
static inline uint16x8_t downsampledDiff2nd_neon( const uint16_t* src, const int srcStride, const uint16_t* srcM1,
                                                  const int srcM1Stride, const uint16_t* srcM2, const int srcM2Stride )
{
  const uint16x8_t s  = sum2x2_neon( src, src + srcStride );
  const uint16x8_t m1 = sum2x2_neon( srcM1, srcM1 + srcM1Stride );
  const uint16x8_t m2 = sum2x2_neon( srcM2, srcM2 + srcM2Stride );
  return vabdq_u16( vaddq_u16( s, m2 ), vshlq_n_u16( m1, 1 ) );
}

static uint64_t AvgHighPassWithDownsamplingDiff2nd_neon( const int width, const int height, const Pel* pSrc,
                                                         const Pel* pSrcM1, const Pel* pSrcM2, const int iSrcStride,
                                                         const int iSM1Stride, const int iSM2Stride )
{
  if( width < 19 )
  {
    return AvgHighPassWithDownsamplingDiff2ndCore( width, height, pSrc, pSrcM1, pSrcM2, iSrcStride, iSM1Stride,
                                                   iSM2Stride );
  }

  const int       last   = ( width - 17 ) & ~1;
  const uint16_t* base   = reinterpret_cast<const uint16_t*>( pSrc ) - 2 * iSrcStride;
  const uint16_t* baseM1 = reinterpret_cast<const uint16_t*>( pSrcM1 ) - 2 * iSM1Stride;
  const uint16_t* baseM2 = reinterpret_cast<const uint16_t*>( pSrcM2 ) - 2 * iSM2Stride;
  uint64x2_t      sum    = vdupq_n_u64( 0 );
  const uint16x8_t laneIndex = vld1q_u16( qpaLaneIndex16 );

  for( int y = 2; y < height - 2; y += 2 )
  {
    const uint16_t* src    = base + y * iSrcStride;
    const uint16_t* srcM1  = baseM1 + y * iSM1Stride;
    const uint16_t* srcM2  = baseM2 + y * iSM2Stride;
    uint32x4_t      rowSum = vdupq_n_u32( 0 );
    int             x      = 2;
    for( ; x < last; x += 16 )
    {
      const uint16x8_t diff = downsampledDiff2nd_neon( src + x, iSrcStride, srcM1 + x, iSM1Stride, srcM2 + x,
                                                       iSM2Stride );
      rowSum = vpadalq_u16( rowSum, diff );
    }
    const uint16x8_t notSummed = vcgeq_u16( laneIndex, vdupq_n_u16( static_cast<uint16_t>( ( x - last ) >> 1 ) ) );
    const uint16x8_t lastDiff  = downsampledDiff2nd_neon( src + last, iSrcStride, srcM1 + last, iSM1Stride,
                                                          srcM2 + last, iSM2Stride );
    rowSum = vpadalq_u16( rowSum, vandq_u16( lastDiff, notSummed ) );
    sum    = vpadalq_u32( sum, rowSum );
  }
  return horizontal_add_u64x2( sum );
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
  AvgHighPass                        = AvgHighPass_neon;
  HDHighPass                         = HDHighPass_neon;
  HDHighPass2                        = HDHighPass2_neon;
  AvgHighPassWithDownsampling        = AvgHighPassWithDownsampling_neon;
  AvgHighPassWithDownsamplingDiff1st = AvgHighPassWithDownsamplingDiff1st_neon;
  AvgHighPassWithDownsamplingDiff2nd = AvgHighPassWithDownsamplingDiff2nd_neon;
  reco     = recoCore_neon;
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

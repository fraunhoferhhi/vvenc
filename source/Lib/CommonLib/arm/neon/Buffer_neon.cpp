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
#include "../mem_neon.h"
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

// adds the squared differences of eight samples: the first four to accLo, the last four to accHi
static inline void addSquaredDiffs8( int32x4_t& accLo, int32x4_t& accHi, const Pel* src0, const Pel* src1 )
{
  const int16x8_t a    = vld1q_s16( src0 );
  const int16x8_t b    = vld1q_s16( src1 );
  const int16x8_t diff = vsubq_s16( a, b );
  accLo = vmlal_s16( accLo, vget_low_s16( diff ), vget_low_s16( diff ) );
  accHi = vmlal_s16( accHi, vget_high_s16( diff ), vget_high_s16( diff ) );
}

// the four part sums of a row of parts of width 32 or 64 from a low and a high accumulator per part
static inline uint32x4_t partSumsLoHi( const int32x4_t accLo0, const int32x4_t accHi0, const int32x4_t accLo1,
                                       const int32x4_t accHi1, const int32x4_t accLo2, const int32x4_t accHi2,
                                       const int32x4_t accLo3, const int32x4_t accHi3 )
{
  const int32x4_t acc0 = vaddq_s32( accLo0, accHi0 );
  const int32x4_t acc1 = vaddq_s32( accLo1, accHi1 );
  const int32x4_t acc2 = vaddq_s32( accLo2, accHi2 );
  const int32x4_t acc3 = vaddq_s32( accLo3, accHi3 );
  return horizontal_add_4d_u32x4( vreinterpretq_u32_s32( acc0 ), vreinterpretq_u32_s32( acc1 ),
                                  vreinterpretq_u32_s32( acc2 ), vreinterpretq_u32_s32( acc3 ) );
}

// the part sums of a row of parts of width 8 or less from its column sums (columns 0-3 in accLo, 4-7 in accHi, the
// lanes past the width 0): the parts of 2 columns are the pair sums, those of 4 two of them
static inline uint32x4_t partSumsOfColumns( const int32x4_t accLo, const int32x4_t accHi, const int lengthX )
{
  int32x4_t pairs = pairwise_add_s32x4( accLo, accHi );
  if( lengthX == 4 )
  {
    pairs = pairwise_add_s32x4( pairs, pairs );
  }
  return vreinterpretq_u32_s32( pairs );
}

// width 64: four parts of 16 columns (two vectors each)
static void sumSquaredErrorParts64_neon( const Pel* src0, const ptrdiff_t stride0, const Pel* src1, const ptrdiff_t stride1,
                                         const int lengthY, const int numY, uint32_t sums[][4] )
{
  for( int j = 0; j < numY; j++, src0 += lengthY * stride0, src1 += lengthY * stride1 )
  {
    const Pel* row0 = src0;
    const Pel* row1 = src1;
    int32x4_t accLo0 = vdupq_n_s32( 0 ), accHi0 = vdupq_n_s32( 0 );
    int32x4_t accLo1 = vdupq_n_s32( 0 ), accHi1 = vdupq_n_s32( 0 );
    int32x4_t accLo2 = vdupq_n_s32( 0 ), accHi2 = vdupq_n_s32( 0 );
    int32x4_t accLo3 = vdupq_n_s32( 0 ), accHi3 = vdupq_n_s32( 0 );
    for( int y = 0; y < lengthY; y++, row0 += stride0, row1 += stride1 )
    {
      addSquaredDiffs8( accLo0, accHi0, row0, row1 );
      addSquaredDiffs8( accLo1, accHi1, row0 + 16, row1 + 16 );
      addSquaredDiffs8( accLo2, accHi2, row0 + 32, row1 + 32 );
      addSquaredDiffs8( accLo3, accHi3, row0 + 48, row1 + 48 );
      addSquaredDiffs8( accLo0, accHi0, row0 + 8, row1 + 8 );
      addSquaredDiffs8( accLo1, accHi1, row0 + 24, row1 + 24 );
      addSquaredDiffs8( accLo2, accHi2, row0 + 40, row1 + 40 );
      addSquaredDiffs8( accLo3, accHi3, row0 + 56, row1 + 56 );
    }
    vst1q_u32( sums[j], partSumsLoHi( accLo0, accHi0, accLo1, accHi1, accLo2, accHi2, accLo3, accHi3 ) );
  }
}

// width 32: four parts of 8 columns (a vector each)
static void sumSquaredErrorParts32_neon( const Pel* src0, const ptrdiff_t stride0, const Pel* src1, const ptrdiff_t stride1,
                                         const int lengthY, const int numY, uint32_t sums[][4] )
{
  for( int j = 0; j < numY; j++, src0 += lengthY * stride0, src1 += lengthY * stride1 )
  {
    const Pel* row0 = src0;
    const Pel* row1 = src1;
    int32x4_t accLo0 = vdupq_n_s32( 0 ), accHi0 = vdupq_n_s32( 0 );
    int32x4_t accLo1 = vdupq_n_s32( 0 ), accHi1 = vdupq_n_s32( 0 );
    int32x4_t accLo2 = vdupq_n_s32( 0 ), accHi2 = vdupq_n_s32( 0 );
    int32x4_t accLo3 = vdupq_n_s32( 0 ), accHi3 = vdupq_n_s32( 0 );
    for( int y = 0; y < lengthY; y++, row0 += stride0, row1 += stride1 )
    {
      addSquaredDiffs8( accLo0, accHi0, row0, row1 );
      addSquaredDiffs8( accLo1, accHi1, row0 + 8, row1 + 8 );
      addSquaredDiffs8( accLo2, accHi2, row0 + 16, row1 + 16 );
      addSquaredDiffs8( accLo3, accHi3, row0 + 24, row1 + 24 );
    }
    vst1q_u32( sums[j], partSumsLoHi( accLo0, accHi0, accLo1, accHi1, accLo2, accHi2, accLo3, accHi3 ) );
  }
}

// width 16: four parts of 4 columns, the halves of each vector of 8, one accumulator per part
static void sumSquaredErrorParts16_neon( const Pel* src0, const ptrdiff_t stride0, const Pel* src1, const ptrdiff_t stride1,
                                         const int lengthY, const int numY, uint32_t sums[][4] )
{
  for( int j = 0; j < numY; j++, src0 += lengthY * stride0, src1 += lengthY * stride1 )
  {
    const Pel* row0 = src0;
    const Pel* row1 = src1;
    int32x4_t acc0 = vdupq_n_s32( 0 );
    int32x4_t acc1 = vdupq_n_s32( 0 );
    int32x4_t acc2 = vdupq_n_s32( 0 );
    int32x4_t acc3 = vdupq_n_s32( 0 );
    for( int y = 0; y < lengthY; y++, row0 += stride0, row1 += stride1 )
    {
      addSquaredDiffs8( acc0, acc1, row0, row1 );
      addSquaredDiffs8( acc2, acc3, row0 + 8, row1 + 8 );
    }
    vst1q_u32( sums[j], horizontal_add_4d_u32x4( vreinterpretq_u32_s32( acc0 ), vreinterpretq_u32_s32( acc1 ),
                                                 vreinterpretq_u32_s32( acc2 ), vreinterpretq_u32_s32( acc3 ) ) );
  }
}

// width 8: the sum of each column first, then the columns of each part
static void sumSquaredErrorParts8_neon( const Pel* src0, const ptrdiff_t stride0, const Pel* src1, const ptrdiff_t stride1,
                                        const int lengthX, const int lengthY, const int numY, uint32_t sums[][4] )
{
  for( int j = 0; j < numY; j++, src0 += lengthY * stride0, src1 += lengthY * stride1 )
  {
    const Pel* row0 = src0;
    const Pel* row1 = src1;
    int32x4_t accLo = vdupq_n_s32( 0 );
    int32x4_t accHi = vdupq_n_s32( 0 );
    for( int y = 0; y < lengthY; y++, row0 += stride0, row1 += stride1 )
    {
      addSquaredDiffs8( accLo, accHi, row0, row1 );
    }
    vst1q_u32( sums[j], partSumsOfColumns( accLo, accHi, lengthX ) );
  }
}

// width 4: the sum of each column first, then the columns of each part
static void sumSquaredErrorParts4_neon( const Pel* src0, const ptrdiff_t stride0, const Pel* src1, const ptrdiff_t stride1,
                                        const int lengthX, const int lengthY, const int numY, uint32_t sums[][4] )
{
  for( int j = 0; j < numY; j++, src0 += lengthY * stride0, src1 += lengthY * stride1 )
  {
    const Pel* row0 = src0;
    const Pel* row1 = src1;
    int32x4_t acc = vdupq_n_s32( 0 );
    for( int y = 0; y < lengthY; y++, row0 += stride0, row1 += stride1 )
    {
      const int16x4_t a    = vld1_s16( row0 );
      const int16x4_t b    = vld1_s16( row1 );
      const int16x4_t diff = vsub_s16( a, b );
      acc = vmlal_s16( acc, diff, diff );
    }
    vst1q_u32( sums[j], partSumsOfColumns( acc, vdupq_n_s32( 0 ), lengthX ) );
  }
}

// width 2: the sum of each column first, then the columns of each part
static void sumSquaredErrorParts2_neon( const Pel* src0, const ptrdiff_t stride0, const Pel* src1, const ptrdiff_t stride1,
                                        const int lengthX, const int lengthY, const int numY, uint32_t sums[][4] )
{
  for( int j = 0; j < numY; j++, src0 += lengthY * stride0, src1 += lengthY * stride1 )
  {
    const Pel* row0 = src0;
    const Pel* row1 = src1;
    int32x4_t acc = vdupq_n_s32( 0 );
    for( int y = 0; y < lengthY; y++, row0 += stride0, row1 += stride1 )
    {
      const int16x4_t a    = load_s16x2( row0 );
      const int16x4_t b    = load_s16x2( row1 );
      const int16x4_t diff = vsub_s16( a, b );
      acc = vmlal_s16( acc, diff, diff );
    }
    vst1q_u32( sums[j], partSumsOfColumns( acc, vdupq_n_s32( 0 ), lengthX ) );
  }
}

// The sums of squared errors of the numX x numY parts of lengthX x lengthY samples (Buffer.h: parts of up to 16 x 16
// samples of up to 12 bits). A lane adds at most 64 squares of 12-bit differences (below 2^30) and a part's sum is below
// 2^32, so the lanes and the final unsigned additions are exact.
static void sumSquaredErrorParts_neon( const Pel* src0, const ptrdiff_t stride0, const Pel* src1, const ptrdiff_t stride1, const int lengthX, const int lengthY, const int numX, const int numY, uint32_t sums[][4] )
{
  const int width = lengthX * numX;
  CHECKD( lengthX > 16 || lengthY > 16, "parts of up to 16 x 16 samples" );
  CHECKD( width > 8 && numX != 4, "only four parts in a row from width 16 up" );
  CHECKD( width <= 8 && lengthX > 4, "parts of the widths up to 8 are 2 or 4 wide" );
  switch( width )
  {
  case 64:
    sumSquaredErrorParts64_neon( src0, stride0, src1, stride1, lengthY, numY, sums );
    break;
  case 32:
    sumSquaredErrorParts32_neon( src0, stride0, src1, stride1, lengthY, numY, sums );
    break;
  case 16:
    sumSquaredErrorParts16_neon( src0, stride0, src1, stride1, lengthY, numY, sums );
    break;
  case 8:
    sumSquaredErrorParts8_neon( src0, stride0, src1, stride1, lengthX, lengthY, numY, sums );
    break;
  case 4:
    sumSquaredErrorParts4_neon( src0, stride0, src1, stride1, lengthX, lengthY, numY, sums );
    break;
  default:
    CHECKD( width != 2, "the widths are 2 to 64" );
    sumSquaredErrorParts2_neon( src0, stride0, src1, stride1, lengthX, lengthY, numY, sums );
    break;
  }
}

template<>
void PelBufferOps::_initPelBufOpsARM<NEON>()
{
  sumSquaredErrorParts = sumSquaredErrorParts_neon;
  addAvg   = addAvg_neon;
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

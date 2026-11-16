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
#include "../mem_neon.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Buffer.h"
#include "sum_neon.h"
#include "transpose_neon.h"

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER

#if defined( _MSC_VER )
#pragma warning( disable : 4700 ) // MSVC equivalent of GCC_WARNING_DISABLE_maybe_uninitialized
#endif

namespace vvenc
{

// MIP matrix multiply, as mipMatrixMulCore: res[n] = Clip( 0, maxVal, ( sum_k input[k] * weight[K * n + k] + offset ) >> 6 )
// for K = 4 or 8 inputs and 16 or 64 outputs, then transposed for `transpose`. For K = 4, vld4q_u8 de-interleaves the
// weights so that each vector holds one tap of 16 outputs, and each tap adds weight * input[k] to the 16 sums. For
// K = 8, each output's 8 weights times the 8 inputs are taken with vmull_s16 / vmlal_s16, and the 4 partial sums of
// each output are reduced with horizontal_add_4d_s32x4. The weights are below 128 and |input| < 2^12, so products and
// sums are C's in 32 bits; offset holds C's rounding term. vqshrun shifts, clips negative values to 0 and saturates to
// 16 bits, so with the min to maxVal it equals C's clip of the 32-bit value.
static inline int32x4_t mipOffset_neon( const Pel* input, const int inputSize, const int inputOffset )
{
  int sum = 0;
  for( int i = 0; i < inputSize; i++ )
  {
    sum += input[i];
  }
  return vdupq_n_s32( ( 1 << ( MIP_SHIFT_MATRIX - 1 ) ) - MIP_OFFSET_MATRIX * sum + ( inputOffset << MIP_SHIFT_MATRIX ) );
}

// acc[i] lane j: sum of output 4 i + j; tap lane n: the weight of output n for this input
static inline void mipAddTap_neon( int32x4_t* acc, const uint8x16_t tap, const Pel input )
{
  const int16x8_t lo = vreinterpretq_s16_u16( vmovl_u8( vget_low_u8( tap ) ) );
  const int16x8_t hi = vreinterpretq_s16_u16( vmovl_u8( vget_high_u8( tap ) ) );
  acc[0] = vmlal_n_s16( acc[0], vget_low_s16( lo ), input );
  acc[1] = vmlal_n_s16( acc[1], vget_high_s16( lo ), input );
  acc[2] = vmlal_n_s16( acc[2], vget_low_s16( hi ), input );
  acc[3] = vmlal_n_s16( acc[3], vget_high_s16( hi ), input );
}

static inline int16x8_t mipClip_neon( const int32x4_t sum0, const int32x4_t sum1, const uint16x8_t vmax )
{
  const uint16x8_t res =
    vcombine_u16( vqshrun_n_s32( sum0, MIP_SHIFT_MATRIX ), vqshrun_n_s32( sum1, MIP_SHIFT_MATRIX ) );
  return vreinterpretq_s16_u16( vminq_u16( res, vmax ) );
}

// the dot products of the 8 inputs with the 8 weights of each of 4 outputs (32 bytes at weight): lane j is output j,
// without the offset
static inline int32x4_t mipDots4Outputs8Inputs_neon( const uint8_t* weight, const int16x8_t in )
{
  const uint8x16_t w01 = vld1q_u8( weight );
  const uint8x16_t w23 = vld1q_u8( weight + 16 );
  const int16x8_t  w0  = vreinterpretq_s16_u16( vmovl_u8( vget_low_u8( w01 ) ) );
  const int16x8_t  w1  = vreinterpretq_s16_u16( vmovl_u8( vget_high_u8( w01 ) ) );
  const int16x8_t  w2  = vreinterpretq_s16_u16( vmovl_u8( vget_low_u8( w23 ) ) );
  const int16x8_t  w3  = vreinterpretq_s16_u16( vmovl_u8( vget_high_u8( w23 ) ) );
  int32x4_t d0 = vmull_s16( vget_low_s16( w0 ), vget_low_s16( in ) );
  int32x4_t d1 = vmull_s16( vget_low_s16( w1 ), vget_low_s16( in ) );
  int32x4_t d2 = vmull_s16( vget_low_s16( w2 ), vget_low_s16( in ) );
  int32x4_t d3 = vmull_s16( vget_low_s16( w3 ), vget_low_s16( in ) );
  d0 = vmlal_s16( d0, vget_high_s16( w0 ), vget_high_s16( in ) );
  d1 = vmlal_s16( d1, vget_high_s16( w1 ), vget_high_s16( in ) );
  d2 = vmlal_s16( d2, vget_high_s16( w2 ), vget_high_s16( in ) );
  d3 = vmlal_s16( d3, vget_high_s16( w3 ), vget_high_s16( in ) );
  return horizontal_add_4d_s32x4( d0, d1, d2, d3 );
}

// 16 outputs of 8 inputs from 128 weight bytes, clipped: outputs 0..7 in rows[0], 8..15 in rows[1]
static inline void mip16Outputs8Inputs_neon( const uint8_t* weight, const Pel* input, const int32x4_t offset,
                                             const uint16x8_t vmax, int16x8_t* rows )
{
  const int16x8_t in = vld1q_s16( input );
  int32x4_t       acc[4];
  for( int i = 0; i < 4; i++ )
  {
    acc[i] = vaddq_s32( offset, mipDots4Outputs8Inputs_neon( weight + 32 * i, in ) );
  }
  rows[0] = mipClip_neon( acc[0], acc[1], vmax );
  rows[1] = mipClip_neon( acc[2], acc[3], vmax );
}

// 4 inputs, 4 x 4 outputs
static void mipMatrixMul_4_4_neon( Pel* res, const Pel* input, const uint8_t* weight, const int maxVal,
                                   const int inputOffset, bool transpose )
{
  const int32x4_t   offset = mipOffset_neon( input, 4, inputOffset );
  const uint8x16x4_t w     = vld4q_u8( weight );   // w.val[k]: tap k of outputs 0..15

  int32x4_t acc[4] = { offset, offset, offset, offset };
  for( int k = 0; k < 4; k++ )
  {
    mipAddTap_neon( acc, w.val[k], input[k] );
  }
  const uint16x8_t vmax    = vdupq_n_u16( maxVal );
  int16x8_t        rows[2] = { mipClip_neon( acc[0], acc[1], vmax ), mipClip_neon( acc[2], acc[3], vmax ) };
  if( transpose )
  {
    transpose_4x4_s16_paired( rows );
  }
  vst1q_s16( res, rows[0] );
  vst1q_s16( res + 8, rows[1] );
}

// 8 inputs, 4 x 4 outputs
static void mipMatrixMul_8_4_neon( Pel* res, const Pel* input, const uint8_t* weight, const int maxVal,
                                   const int inputOffset, bool transpose )
{
  int16x8_t rows[2];
  mip16Outputs8Inputs_neon( weight, input, mipOffset_neon( input, 8, inputOffset ), vdupq_n_u16( maxVal ), rows );
  if( transpose )
  {
    transpose_4x4_s16_paired( rows );
  }
  vst1q_s16( res, rows[0] );
  vst1q_s16( res + 8, rows[1] );
}

// 8 inputs, 8 x 8 outputs
static void mipMatrixMul_8_8_neon( Pel* res, const Pel* input, const uint8_t* weight, const int maxVal,
                                   const int inputOffset, bool transpose )
{
  const int32x4_t  offset = mipOffset_neon( input, 8, inputOffset );
  const uint16x8_t vmax   = vdupq_n_u16( maxVal );

  int16x8_t rows[8];
  for( int i = 0; i < 4; i++ )
  {
    mip16Outputs8Inputs_neon( weight + 128 * i, input, offset, vmax, rows + 2 * i );
  }
  if( transpose )
  {
    transpose_8x8_s16( rows );
  }
  for( int i = 0; i < 8; i++ )
  {
    vst1q_s16( res + 8 * i, rows[i] );
  }
}

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

// The element-wise kernels below compute in 16 bits, which equals C's int arithmetic stored to Pel (the same low 16
// bits). Widths are multiples of 8; 16 samples per step, then 8.

// dst = 2 * org - other, as removeHighFreq
static void removeHighFreq8_neon( Pel* dst, int dstStride, const Pel* org, int orgStride, const Pel* other,
                                  int otherStride, int width, int height )
{
  for( ; height > 0; height--, dst += dstStride, org += orgStride, other += otherStride )
  {
    int x = 0;
    for( ; x + 16 <= width; x += 16 )
    {
      const int16x8_t o0 = vld1q_s16( org + x );
      const int16x8_t o1 = vld1q_s16( org + x + 8 );
      const int16x8_t p0 = vld1q_s16( other + x );
      const int16x8_t p1 = vld1q_s16( other + x + 8 );
      vst1q_s16( dst + x, vsubq_s16( vshlq_n_s16( o0, 1 ), p0 ) );
      vst1q_s16( dst + x + 8, vsubq_s16( vshlq_n_s16( o1, 1 ), p1 ) );
    }
    if( x < width )
    {
      const int16x8_t o0 = vld1q_s16( org + x );
      const int16x8_t p0 = vld1q_s16( other + x );
      vst1q_s16( dst + x, vsubq_s16( vshlq_n_s16( o0, 1 ), p0 ) );
    }
  }
}

// dest = src0 - src1, as subsCore
static void sub8_neon( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dest, int destStride,
                       int width, int height )
{
  for( ; height > 0; height--, src0 += src0Stride, src1 += src1Stride, dest += destStride )
  {
    int x = 0;
    for( ; x + 16 <= width; x += 16 )
    {
      const int16x8_t a0 = vld1q_s16( src0 + x );
      const int16x8_t a1 = vld1q_s16( src0 + x + 8 );
      const int16x8_t b0 = vld1q_s16( src1 + x );
      const int16x8_t b1 = vld1q_s16( src1 + x + 8 );
      vst1q_s16( dest + x, vsubq_s16( a0, b0 ) );
      vst1q_s16( dest + x + 8, vsubq_s16( a1, b1 ) );
    }
    if( x < width )
    {
      const int16x8_t a0 = vld1q_s16( src0 + x );
      const int16x8_t b0 = vld1q_s16( src1 + x );
      vst1q_s16( dest + x, vsubq_s16( a0, b0 ) );
    }
  }
}

// dst = ClipPel( src ) of a contiguous block, as copyClipCore; numSamples is a multiple of 8 or exactly 4
static void copyClip_neon( const Pel* src, Pel* dst, int numSamples, const ClpRng& clpRng )
{
  CHECKD( ( numSamples & 7 ) != 0 && numSamples != 4, "numSamples must be a multiple of 8, or exactly 4" );

  const int16x8_t vmin = vdupq_n_s16( clpRng.min() );
  const int16x8_t vmax = vdupq_n_s16( clpRng.max() );
  int             n    = 0;
  for( ; n + 16 <= numSamples; n += 16 )
  {
    const int16x8_t a = vld1q_s16( src + n );
    const int16x8_t b = vld1q_s16( src + n + 8 );
    vst1q_s16( dst + n, vminq_s16( vmaxq_s16( a, vmin ), vmax ) );
    vst1q_s16( dst + n + 8, vminq_s16( vmaxq_s16( b, vmin ), vmax ) );
  }
  if( n + 8 <= numSamples )
  {
    const int16x8_t a = vld1q_s16( src + n );
    vst1q_s16( dst + n, vminq_s16( vmaxq_s16( a, vmin ), vmax ) );
  }
  else if( n < numSamples )
  {
    const int16x4_t a = vld1_s16( src + n );
    vst1_s16( dst + n, vmin_s16( vmax_s16( a, vget_low_s16( vmin ) ), vget_low_s16( vmax ) ) );
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

template<int W>
void addAvg_strided_neon( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dest, int destStride,
                          int width, int height, unsigned rshift, int offset, const ClpRng& clpRng );

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

// GPM's split SADs of all merge candidates (EncCu.cpp, geoPrefixSums then geoRunSads per split direction), the NEON
// version of geoPrefixSums_AVX2 / geoRunSads_AVX2 (BufferX86.h) with their layout: per row the positions 0..width, and
// at each position the running sums of the 8 lanes (candidates) side by side, so that geoRunSads takes one 8-lane load
// per run end. The two only work as a pair (the C pair keeps one lane after the other). All lanes are computed: the
// caller points unused lanes at the original and ignores their sums.

// Per group of 8 columns: each lane's 8 absolute differences, transposed to per column the differences of all lanes,
// added to the running sums of lanes 0-3 and 4-7 and stored as one position. |org - pred| <= 1023 (both are samples),
// so a row's sums stay far below 2^32.
static void geoPrefixSums_neon( const Pel* org, int orgStride, const Pel* const* pred, int predStride, int width, int height, const bool* /*all lanes are computed*/, uint32_t* prefix )
{
  CHECKD( width % 8 != 0, "GEO widths are multiples of 8" );
  static_assert( GEO_PREFIX_LANES == 8, "two vectors of 32-bit sums" );

  for( int y = 0; y < height; y++ )
  {
    uint32x4_t sumLow  = vdupq_n_u32( 0 );   // lanes 0..3
    uint32x4_t sumHigh = vdupq_n_u32( 0 );   // lanes 4..7
    vst1q_u32( prefix, sumLow );
    vst1q_u32( prefix + 4, sumHigh );
    prefix += GEO_PREFIX_LANES;

    for( int x = 0; x < width; x += 8 )
    {
      const int16x8_t orgs = vld1q_s16( org + x );
      uint16x8_t diffs[GEO_PREFIX_LANES];
      for( int lane = 0; lane < GEO_PREFIX_LANES; lane++ )
      {
        diffs[lane] = vreinterpretq_u16_s16( vabdq_s16( orgs, vld1q_s16( pred[lane] + y * predStride + x ) ) );
      }
      uint16x8_t columns[8];
      transpose_8x8_u16( diffs, columns );
      for( int k = 0; k < 8; k++ )
      {
        sumLow  = vaddw_u16( sumLow, vget_low_u16( columns[k] ) );
        sumHigh = vaddw_u16( sumHigh, vget_high_u16( columns[k] ) );
        vst1q_u32( prefix, sumLow );
        vst1q_u32( prefix + 4, sumHigh );
        prefix += GEO_PREFIX_LANES;
      }
    }
    org += orgStride;
  }
}

static void geoRunSads_neon( const uint32_t* prefix, const uint8_t* runStart, const uint8_t* runEnd, int width, int height, const bool* /*all lanes are computed*/, uint32_t* sad )
{
  uint32x4_t sumLow  = vdupq_n_u32( 0 );
  uint32x4_t sumHigh = vdupq_n_u32( 0 );
  for( int y = 0; y < height; y++, prefix += ( width + 1 ) * GEO_PREFIX_LANES )
  {
    const uint32_t* atEnd   = prefix + runEnd[y] * GEO_PREFIX_LANES;
    const uint32_t* atStart = prefix + runStart[y] * GEO_PREFIX_LANES;
    sumLow  = vaddq_u32( sumLow, vsubq_u32( vld1q_u32( atEnd ), vld1q_u32( atStart ) ) );
    sumHigh = vaddq_u32( sumHigh, vsubq_u32( vld1q_u32( atEnd + 4 ), vld1q_u32( atStart + 4 ) ) );
  }
  vst1q_u32( sad, sumLow );
  vst1q_u32( sad + 4, sumHigh );
}

// sumSquaredError with the arithmetic of sumSquaredError_SIMD at SSE4.1: the differences in 16 bits (they wrap like
// _mm_sub_epi16), their squares (at most 2^30) widened to 64 bits, unsigned. Integer addition does not depend on the
// order, so the sum is the same as x86's and the C code's whenever the differences lie in [-2^15, 2^15], as for any
// samples of up to 15 bits.

// adds the squares of the four differences a - b to acc
static inline uint64x2_t addSquares_neon( const uint64x2_t acc, const int16x4_t a, const int16x4_t b )
{
  const int16x4_t diff = vsub_s16( a, b );
  return vpadalq_u32( acc, vreinterpretq_u32_s32( vmull_s16( diff, diff ) ) );
}

// adds the squares of the eight differences a - b to acc; two squares share a 32-bit lane first, at most 2^31, so the
// lane is exact as an unsigned value
static inline uint64x2_t addSquares_neon( const uint64x2_t acc, const int16x8_t a, const int16x8_t b )
{
  const int16x8_t diff    = vsubq_s16( a, b );
  const int32x4_t squares = vmull_s16( vget_low_s16( diff ), vget_low_s16( diff ) );
  const int32x4_t sums    = vmlal_s16( squares, vget_high_s16( diff ), vget_high_s16( diff ) );
  return vpadalq_u32( acc, vreinterpretq_u32_s32( sums ) );
}

// the two samples of each of up to four rows in one vector, the lanes of missing rows zero (like _vv_loadWidth2Rows)
static inline int16x8_t loadWidth2Rows_neon( const Pel* p, const ptrdiff_t stride, const int numRows )
{
  uint32_t   row;
  uint32x4_t v = vdupq_n_u32( 0 );
  memcpy( &row, p, sizeof( row ) );
  v = vsetq_lane_u32( row, v, 0 );
  if( numRows > 1 )
  {
    memcpy( &row, &p[stride], sizeof( row ) );
    v = vsetq_lane_u32( row, v, 1 );
  }
  if( numRows > 2 )
  {
    memcpy( &row, &p[2 * stride], sizeof( row ) );
    v = vsetq_lane_u32( row, v, 2 );
  }
  if( numRows > 3 )
  {
    memcpy( &row, &p[3 * stride], sizeof( row ) );
    v = vsetq_lane_u32( row, v, 3 );
  }
  return vreinterpretq_s16_u32( v );
}

static uint64_t sumSquaredError_neon( const Pel* src0, const ptrdiff_t stride0, const Pel* src1, const ptrdiff_t stride1, const int width, const int height )
{
  // below 16 the encoder passes only the widths 2, 4 and 8 (SBT parts); the others take the C code
  if( width < 16 && width != 2 && width != 4 && width != 8 )
  {
    return sumSquaredErrorCore( src0, stride0, src1, stride1, width, height );
  }

  uint64x2_t acc0 = vdupq_n_u64( 0 );

  if( width == 2 )
  {
    int y = 0;
    for( ; y + 4 <= height; y += 4, src0 += 4 * stride0, src1 += 4 * stride1 )
    {
      const int16x8_t a = loadWidth2Rows_neon( src0, stride0, 4 );
      const int16x8_t b = loadWidth2Rows_neon( src1, stride1, 4 );
      acc0 = addSquares_neon( acc0, a, b );
    }
    if( y < height )
    {
      const int16x8_t a = loadWidth2Rows_neon( src0, stride0, height - y );
      const int16x8_t b = loadWidth2Rows_neon( src1, stride1, height - y );
      acc0 = addSquares_neon( acc0, a, b );
    }
    return horizontal_add_u64x2( acc0 );
  }

  if( width == 4 )
  {
    int y = 0;
    for( ; y + 2 <= height; y += 2, src0 += 2 * stride0, src1 += 2 * stride1 )
    {
      const int16x4_t a0 = vld1_s16( src0 );
      const int16x4_t a1 = vld1_s16( src0 + stride0 );
      const int16x4_t b0 = vld1_s16( src1 );
      const int16x4_t b1 = vld1_s16( src1 + stride1 );
      acc0 = addSquares_neon( acc0, vcombine_s16( a0, a1 ), vcombine_s16( b0, b1 ) );
    }
    if( y < height )
    {
      const int16x4_t a = vld1_s16( src0 );
      const int16x4_t b = vld1_s16( src1 );
      acc0 = addSquares_neon( acc0, a, b );
    }
    return horizontal_add_u64x2( acc0 );
  }

  // widths 8 and above: two accumulators (four from width 32), for two rows (width 8) or the two halves of 16 columns
  uint64x2_t acc1 = vdupq_n_u64( 0 );

  if( width == 8 )
  {
    int y = 0;
    for( ; y + 2 <= height; y += 2, src0 += 2 * stride0, src1 += 2 * stride1 )
    {
      const int16x8_t a0 = vld1q_s16( src0 );
      const int16x8_t b0 = vld1q_s16( src1 );
      const int16x8_t a1 = vld1q_s16( src0 + stride0 );
      const int16x8_t b1 = vld1q_s16( src1 + stride1 );
      acc0 = addSquares_neon( acc0, a0, b0 );
      acc1 = addSquares_neon( acc1, a1, b1 );
    }
    if( y < height )
    {
      const int16x8_t a = vld1q_s16( src0 );
      const int16x8_t b = vld1q_s16( src1 );
      acc0 = addSquares_neon( acc0, a, b );
    }
    return horizontal_add_u64x2( vaddq_u64( acc0, acc1 ) );
  }

  if( width == 16 )
  {
    for( int y = 0; y < height; y++, src0 += stride0, src1 += stride1 )
    {
      const int16x8_t a0 = vld1q_s16( src0 );
      const int16x8_t b0 = vld1q_s16( src1 );
      const int16x8_t a1 = vld1q_s16( src0 + 8 );
      const int16x8_t b1 = vld1q_s16( src1 + 8 );
      acc0 = addSquares_neon( acc0, a0, b0 );
      acc1 = addSquares_neon( acc1, a1, b1 );
    }
    return horizontal_add_u64x2( vaddq_u64( acc0, acc1 ) );
  }

  // wider: 32 columns per step on four accumulators, then 16 columns if that many are left; the last width % 16
  // columns as the 16 ending at the row's end, with the lanes of the columns already counted set to zero in both
  // sources
  uint64x2_t acc2 = vdupq_n_u64( 0 );
  uint64x2_t acc3 = vdupq_n_u64( 0 );

  static const int16_t laneIndex[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
  const int       tail     = width % 16;
  const int       lastX    = width - 16;
  const int16x8_t firstNew = vdupq_n_s16( 16 - tail );
  const int16x8_t index0   = vld1q_s16( laneIndex );
  const int16x8_t index1   = vld1q_s16( laneIndex + 8 );
  const int16x8_t keep0    = vreinterpretq_s16_u16( vcgeq_s16( index0, firstNew ) );
  const int16x8_t keep1    = vreinterpretq_s16_u16( vcgeq_s16( index1, firstNew ) );

  for( int y = 0; y < height; y++, src0 += stride0, src1 += stride1 )
  {
    int x = 0;
    for( ; x + 32 <= width; x += 32 )
    {
      const int16x8_t a0 = vld1q_s16( &src0[x] );
      const int16x8_t b0 = vld1q_s16( &src1[x] );
      const int16x8_t a1 = vld1q_s16( &src0[x + 8] );
      const int16x8_t b1 = vld1q_s16( &src1[x + 8] );
      const int16x8_t a2 = vld1q_s16( &src0[x + 16] );
      const int16x8_t b2 = vld1q_s16( &src1[x + 16] );
      const int16x8_t a3 = vld1q_s16( &src0[x + 24] );
      const int16x8_t b3 = vld1q_s16( &src1[x + 24] );
      acc0 = addSquares_neon( acc0, a0, b0 );
      acc1 = addSquares_neon( acc1, a1, b1 );
      acc2 = addSquares_neon( acc2, a2, b2 );
      acc3 = addSquares_neon( acc3, a3, b3 );
    }
    if( x + 16 <= width )
    {
      const int16x8_t a0 = vld1q_s16( &src0[x] );
      const int16x8_t b0 = vld1q_s16( &src1[x] );
      const int16x8_t a1 = vld1q_s16( &src0[x + 8] );
      const int16x8_t b1 = vld1q_s16( &src1[x + 8] );
      acc0 = addSquares_neon( acc0, a0, b0 );
      acc1 = addSquares_neon( acc1, a1, b1 );
    }
    if( tail != 0 )
    {
      const int16x8_t a0 = vld1q_s16( &src0[lastX] );
      const int16x8_t b0 = vld1q_s16( &src1[lastX] );
      const int16x8_t a1 = vld1q_s16( &src0[lastX + 8] );
      const int16x8_t b1 = vld1q_s16( &src1[lastX + 8] );
      acc0 = addSquares_neon( acc0, vandq_s16( a0, keep0 ), vandq_s16( b0, keep0 ) );
      acc1 = addSquares_neon( acc1, vandq_s16( a1, keep1 ), vandq_s16( b1, keep1 ) );
    }
  }
  return horizontal_add_u64x2( vaddq_u64( vaddq_u64( acc0, acc1 ), vaddq_u64( acc2, acc3 ) ) );
}

template<>
void PelBufferOps::_initPelBufOpsARM<NEON>()
{
  mipMatrixMul_4_4 = mipMatrixMul_4_4_neon;
  mipMatrixMul_8_4 = mipMatrixMul_8_4_neon;
  mipMatrixMul_8_8 = mipMatrixMul_8_8_neon;

  addAvg   = addAvg_neon;
  reco     = recoCore_neon;
  copyBuffer = copyBuffer_neon;
  addAvg4  = addAvg_strided_neon<4>;
  removeHighFreq8 = removeHighFreq8_neon;
  sub8            = sub8_neon;
  copyClip        = copyClip_neon;
  addAvg8  = addAvg_strided_neon<8>;
  sumSquaredErrorParts = sumSquaredErrorParts_neon;
  addAvg16 = addAvg_strided_neon<16>;

  // a pair: geoRunSads_neon reads geoPrefixSums_neon's layout
  geoPrefixSums = geoPrefixSums_neon;
  geoRunSads    = geoRunSads_neon;

  sumSquaredError = sumSquaredError_neon;
}

} // namespace vvenc

#if defined( _MSC_VER )
#pragma warning( default : 4700 )
#endif

#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER
//! \}

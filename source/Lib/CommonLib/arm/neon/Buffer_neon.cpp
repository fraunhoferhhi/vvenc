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

// Block statistics. Each kernel returns exactly its C core's result (Buffer.cpp): the sums are formed in wider lanes
// than any partial sum needs, and only the order of the integer additions differs.

// the absolute differences of absGradientSums_neon at the 8 positions from row: horizontal, vertical, down-left and
// down-right
static inline uint16x8x4_t absGradients_neon( const Pel* row, const Pel* below )
{
  const int16x8_t a0 = vld1q_s16( row );
  const int16x8_t a1 = vld1q_s16( row + 1 );
  const int16x8_t b0 = vld1q_s16( below );
  const int16x8_t b1 = vld1q_s16( below + 1 );

  uint16x8x4_t gradients;
  gradients.val[0] = vreinterpretq_u16_s16( vabdq_s16( a1, a0 ) );
  gradients.val[1] = vreinterpretq_u16_s16( vabdq_s16( b0, a0 ) );
  gradients.val[2] = vreinterpretq_u16_s16( vabdq_s16( a1, b0 ) );
  gradients.val[3] = vreinterpretq_u16_s16( vabdq_s16( b1, a0 ) );
  return gradients;
}

// adds the gradients of absGradients_neon pairwise to the 32-bit sums of each direction
static inline void addAbsGradients_neon( const uint16x8x4_t gradients, uint32x4_t& hor, uint32x4_t& ver,
                                         uint32x4_t& down, uint32x4_t& up )
{
  hor  = vpadalq_u16( hor, gradients.val[0] );
  ver  = vpadalq_u16( ver, gradients.val[1] );
  down = vpadalq_u16( down, gradients.val[2] );
  up   = vpadalq_u16( up, gradients.val[3] );
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
      addAbsGradients_neon( absGradients_neon( src + x, below + x ), hor, ver, down, up );
    }
    if( x < numPos )
    {
      const uint16x8_t notCounted = vcgeq_u16( laneIndex, vdupq_n_u16( static_cast<uint16_t>( x - last ) ) );
      uint16x8x4_t     gradients  = absGradients_neon( src + last, below + last );
      gradients.val[0]            = vandq_u16( gradients.val[0], notCounted );
      gradients.val[1]            = vandq_u16( gradients.val[1], notCounted );
      gradients.val[2]            = vandq_u16( gradients.val[2], notCounted );
      gradients.val[3]            = vandq_u16( gradients.val[3], notCounted );
      addAbsGradients_neon( gradients, hor, ver, down, up );
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
    const int32x4_t blockSums  = pairwise_add_s32x4( columnPair, columnPair );   // s0 s1 s0 s1

    // each block's sum in its four lanes
    const int16x4_t sums16 = vmovn_s32( blockSums );
    const int16x8_t s      = vcombine_s16( vdup_lane_s16( sums16, 0 ), vdup_lane_s16( sums16, 1 ) );

    uint16x8_t dev = vreinterpretq_u16_s16( vabdq_s16( s, vshlq_n_s16( row0, 4 ) ) );
    dev            = vvenc_vabaq_s16( dev, s, vshlq_n_s16( row1, 4 ) );
    dev            = vvenc_vabaq_s16( dev, s, vshlq_n_s16( row2, 4 ) );
    dev            = vvenc_vabaq_s16( dev, s, vshlq_n_s16( row3, 4 ) );
    const uint32x4_t devPair   = vpaddlq_u16( dev );
    const uint32x2_t blockDevs = vpadd_u32( vget_low_u32( devPair ), vget_high_u32( devPair ) );

    vst1_s32( sum + b, vget_low_s32( blockSums ) );
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
  AvgHighPass                        = AvgHighPass_neon;
  HDHighPass                         = HDHighPass_neon;
  HDHighPass2                        = HDHighPass2_neon;
  AvgHighPassWithDownsampling        = AvgHighPassWithDownsampling_neon;
  AvgHighPassWithDownsamplingDiff1st = AvgHighPassWithDownsamplingDiff1st_neon;
  AvgHighPassWithDownsamplingDiff2nd = AvgHighPassWithDownsamplingDiff2nd_neon;
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

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
#include "transpose_neon.h"

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER

#if defined( _MSC_VER )
#pragma warning( disable : 4700 ) // MSVC equivalent of GCC_WARNING_DISABLE_maybe_uninitialized
#endif

namespace vvenc
{

// MIP matrix multiply, as mipMatrixMulCore: res[n] = Clip( 0, maxVal, ( sum_k input[k] * weight[K * n + k] + offset ) >> 6 )
// for K = 4 or 8 inputs and 16 or 64 outputs, then transposed for `transpose`. The weights are de-interleaved so that
// each vector holds one tap of 16 outputs: vld4q_u8 does it for K = 4; for K = 8 it leaves taps k and k + 4 of 8
// outputs in alternating lanes, which vuzpq_u8 of two such vectors separates for 16 outputs. Each tap then adds
// weight * input[k] to the 16 sums, with no horizontal adds. The weights are below 128 and |input| < 2^12, so products
// and sums are C's in 32 bits; offset holds C's rounding term. The saturating narrow then the clip to 0..maxVal equal
// C's clip of the 32-bit value.
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

static inline int16x8_t mipClip_neon( const int32x4_t sum0, const int32x4_t sum1, const int16x8_t vmax )
{
  const int16x8_t res = vcombine_s16( vqmovn_s32( vshrq_n_s32( sum0, MIP_SHIFT_MATRIX ) ),
                                      vqmovn_s32( vshrq_n_s32( sum1, MIP_SHIFT_MATRIX ) ) );
  return vminq_s16( vmaxq_s16( res, vdupq_n_s16( 0 ) ), vmax );
}

// 16 outputs of 8 inputs from 128 weight bytes, clipped: outputs 0..7 in rows[0], 8..15 in rows[1]
static inline void mip16Outputs8Inputs_neon( const uint8_t* weight, const Pel* input, const int32x4_t offset,
                                             const int16x8_t vmax, int16x8_t* rows )
{
  // w0.val[k] lane m: tap k + 4 * ( m & 1 ) of output m >> 1; w1 the same for outputs 8..15
  const uint8x16x4_t w0 = vld4q_u8( weight );
  const uint8x16x4_t w1 = vld4q_u8( weight + 64 );

  int32x4_t acc[4] = { offset, offset, offset, offset };
  for( int k = 0; k < 4; k++ )
  {
    const uint8x16x2_t taps = vuzpq_u8( w0.val[k], w1.val[k] );   // tap k, tap k + 4 of outputs 0..15
    mipAddTap_neon( acc, taps.val[0], input[k] );
    mipAddTap_neon( acc, taps.val[1], input[k + 4] );
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
  const int16x8_t vmax    = vdupq_n_s16( maxVal );
  int16x8_t       rows[2] = { mipClip_neon( acc[0], acc[1], vmax ), mipClip_neon( acc[2], acc[3], vmax ) };
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
  mip16Outputs8Inputs_neon( weight, input, mipOffset_neon( input, 8, inputOffset ), vdupq_n_s16( maxVal ), rows );
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
  const int32x4_t offset = mipOffset_neon( input, 8, inputOffset );
  const int16x8_t vmax   = vdupq_n_s16( maxVal );

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

template<>
void PelBufferOps::_initPelBufOpsARM<NEON>()
{
  mipMatrixMul_4_4 = mipMatrixMul_4_4_neon;
  mipMatrixMul_8_4 = mipMatrixMul_8_4_neon;
  mipMatrixMul_8_8 = mipMatrixMul_8_8_neon;

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

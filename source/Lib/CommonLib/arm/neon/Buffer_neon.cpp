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

// CIIP blend, as weightCiipCore. numIntra 1: the rounded average of res and src. Otherwise
// res = ( unscale + 3 * scale + 2 ) >> 2, where numIntra picks the side weighted 3; that equals
// vrhadd( vhadd( unscale, scale ), scale ): with unscale + scale = 2k + r (r = 0 or 1) both are
// floor( ( k + scale + 1 + r / 2 ) / 2 ) = floor( ( k + scale + 1 ) / 2 ), exact for any Pel values.
static void weightCiip_neon( Pel* res, const Pel* src, const int numSamples, int numIntra )
{
  CHECKD( ( numSamples & 15 ) != 0, "numSamples must be a multiple of 16" );
  if( numIntra == 1 )
  {
    for( int n = 0; n < numSamples; n += 16 )
    {
      const int16x8_t r0 = vld1q_s16( res + n );
      const int16x8_t r1 = vld1q_s16( res + n + 8 );
      const int16x8_t s0 = vld1q_s16( src + n );
      const int16x8_t s1 = vld1q_s16( src + n + 8 );
      vst1q_s16( res + n,     vrhaddq_s16( r0, s0 ) );
      vst1q_s16( res + n + 8, vrhaddq_s16( r1, s1 ) );
    }
  }
  else
  {
    const Pel* scale   = numIntra ? src : res;
    const Pel* unscale = numIntra ? res : src;
    for( int n = 0; n < numSamples; n += 16 )
    {
      const int16x8_t s0 = vld1q_s16( scale + n );
      const int16x8_t s1 = vld1q_s16( scale + n + 8 );
      const int16x8_t u0 = vld1q_s16( unscale + n );
      const int16x8_t u1 = vld1q_s16( unscale + n + 8 );
      vst1q_s16( res + n,     vrhaddq_s16( vhaddq_s16( u0, s0 ), s0 ) );
      vst1q_s16( res + n + 8, vrhaddq_s16( vhaddq_s16( u1, s1 ), s1 ) );
    }
  }
}

// GPM rounding, as roundGeoCore: dst = ClipPel( ( src + offset ) >> rshift ). The encoder's offset is the rounding half
// plus IF_INTERNAL_OFFS, a multiple of 2^rshift, so this is a rounding shift of src (no int16 overflow) plus
// offset >> rshift. The add saturates only where the clip cuts anyway; ClpRng::min() is 0, so vqshlu by 0 clips at 0.
static void roundGeo_neon( const Pel* src, Pel* dst, const int numSamples, unsigned rshift, int offset,
                           const ClpRng& clpRng )
{
  CHECKD( ( numSamples & 15 ) != 0, "numSamples must be a multiple of 16" );
  CHECKD( ( offset & ( ( 1 << rshift ) - 1 ) ) != ( 1 << ( rshift - 1 ) ),
          "offset must be the rounding half plus a multiple of 2^rshift" );
  const int16x8_t  vshift  = vdupq_n_s16( -( int ) rshift );
  const int16x8_t  voffset = vdupq_n_s16( offset >> rshift );
  const uint16x8_t vmax    = vdupq_n_u16( clpRng.max() );
  for( int n = 0; n < numSamples; n += 16 )
  {
    const int16x8_t a0 = vld1q_s16( src + n );
    const int16x8_t a1 = vld1q_s16( src + n + 8 );
    const int16x8_t r0 = vqaddq_s16( vrshlq_s16( a0, vshift ), voffset );
    const int16x8_t r1 = vqaddq_s16( vrshlq_s16( a1, vshift ), voffset );
    vst1q_s16( dst + n,     vreinterpretq_s16_u16( vminq_u16( vqshluq_n_s16( r0, 0 ), vmax ) ) );
    vst1q_s16( dst + n + 8, vreinterpretq_s16_u16( vminq_u16( vqshluq_n_s16( r1, 0 ), vmax ) ) );
  }
}

// Affine PROF dMv rounding, as roundAffineMv and Clip3: v = clip( ( v + 2^(nShift-1) - ( v >= 0 ) ) >> nShift ). Adding
// the v > 0 mask (-1) before a rounding shift gives the same; at v == 0 both give 0.
static void roundIntVector_neon( int* v, int size, unsigned int nShift, const int dmvLimit )
{
  CHECKD( ( size & 15 ) != 0, "size must be a multiple of 16" );
  const int32x4_t vshift = vdupq_n_s32( -( int ) nShift );
  const int32x4_t vmin   = vdupq_n_s32( -dmvLimit );
  const int32x4_t vmax   = vdupq_n_s32( dmvLimit );
  const int32x4_t vzero  = vdupq_n_s32( 0 );
  for( int i = 0; i < size; i += 16 )
  {
    const int32x4_t a0 = vld1q_s32( v + i );
    const int32x4_t a1 = vld1q_s32( v + i + 4 );
    const int32x4_t a2 = vld1q_s32( v + i + 8 );
    const int32x4_t a3 = vld1q_s32( v + i + 12 );
    // the v > 0 masks are -1
    const int32x4_t r0 = vrshlq_s32( vaddq_s32( a0, vreinterpretq_s32_u32( vcgtq_s32( a0, vzero ) ) ), vshift );
    const int32x4_t r1 = vrshlq_s32( vaddq_s32( a1, vreinterpretq_s32_u32( vcgtq_s32( a1, vzero ) ) ), vshift );
    const int32x4_t r2 = vrshlq_s32( vaddq_s32( a2, vreinterpretq_s32_u32( vcgtq_s32( a2, vzero ) ) ), vshift );
    const int32x4_t r3 = vrshlq_s32( vaddq_s32( a3, vreinterpretq_s32_u32( vcgtq_s32( a3, vzero ) ) ), vshift );
    vst1q_s32( v + i,      vminq_s32( vmaxq_s32( r0, vmin ), vmax ) );
    vst1q_s32( v + i + 4,  vminq_s32( vmaxq_s32( r1, vmin ), vmax ) );
    vst1q_s32( v + i + 8,  vminq_s32( vmaxq_s32( r2, vmin ), vmax ) );
    vst1q_s32( v + i + 12, vminq_s32( vmaxq_s32( r3, vmin ), vmax ) );
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

// The block operations below serve AreaBuf::subtract, copyClip and reconstruct for blocks that are not contiguous (a
// contiguous one takes copyClip_neon or recoCore_neon), by width: 2, multiples of 4 and multiples of 8. Like C and the
// x86 kernels they hold for any Pel values: sub wraps, reco adds with saturation (which keeps the order of the sums, so
// the clip gives C's result) and then clips.

// dest = src0 - src1 of a block 2 wide, two rows per vector
static void sub2_neon( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dest, int destStride,
                       int width, int height )
{
  CHECKD( width != 2, "only for blocks 2 wide" );
  for( ; height >= 2; height -= 2, src0 += 2 * src0Stride, src1 += 2 * src1Stride, dest += 2 * destStride )
  {
    const int16x4_t a = load_s16x2x2( src0, src0Stride );
    const int16x4_t b = load_s16x2x2( src1, src1Stride );
    store_s16x2x2( dest, vsub_s16( a, b ), destStride );
  }
  if( height > 0 )
  {
    const int16x4_t a = load_s16x2( src0 );
    const int16x4_t b = load_s16x2( src1 );
    store_s16x2( dest, vsub_s16( a, b ) );
  }
}

// dest = src0 - src1 of a block whose width is a multiple of 4, 8 samples at a time and the last 4
static void sub4_neon( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dest, int destStride,
                       int width, int height )
{
  CHECKD( ( width & 3 ) != 0, "width must be a multiple of 4" );
  for( ; height > 0; height--, src0 += src0Stride, src1 += src1Stride, dest += destStride )
  {
    int x = 0;
    for( ; x + 8 <= width; x += 8 )
    {
      const int16x8_t a = vld1q_s16( src0 + x );
      const int16x8_t b = vld1q_s16( src1 + x );
      vst1q_s16( dest + x, vsubq_s16( a, b ) );
    }
    if( x < width )
    {
      const int16x4_t a = vld1_s16( src0 + x );
      const int16x4_t b = vld1_s16( src1 + x );
      vst1_s16( dest + x, vsub_s16( a, b ) );
    }
  }
}

// dst = ClipPel( src ) of a block 2 wide, two rows per vector
static void copyClip2_neon( const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height,
                            const ClpRng& clpRng )
{
  CHECKD( width != 2, "only for blocks 2 wide" );
  const int16x4_t vmin = vdup_n_s16( clpRng.min() );
  const int16x4_t vmax = vdup_n_s16( clpRng.max() );
  for( ; height >= 2; height -= 2, src += 2 * srcStride, dst += 2 * dstStride )
  {
    const int16x4_t a = load_s16x2x2( src, srcStride );
    store_s16x2x2( dst, vmin_s16( vmax_s16( a, vmin ), vmax ), dstStride );
  }
  if( height > 0 )
  {
    const int16x4_t a = load_s16x2( src );
    store_s16x2( dst, vmin_s16( vmax_s16( a, vmin ), vmax ) );
  }
}

// dst = ClipPel( src ) of a block whose width is a multiple of 4, 8 samples at a time and the last 4
static void copyClip4_neon( const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height,
                            const ClpRng& clpRng )
{
  CHECKD( ( width & 3 ) != 0, "width must be a multiple of 4" );
  const int16x8_t vmin  = vdupq_n_s16( clpRng.min() );
  const int16x8_t vmax  = vdupq_n_s16( clpRng.max() );
  const int16x4_t vmin4 = vget_low_s16( vmin );
  const int16x4_t vmax4 = vget_low_s16( vmax );
  if( width == 4 )
  {
    // a block 4 wide, two rows per vector; an odd last row is left to the loop below
    for( ; height >= 2; height -= 2, src += 2 * srcStride, dst += 2 * dstStride )
    {
      const int16x4_t a0 = vld1_s16( src );
      const int16x4_t a1 = vld1_s16( src + srcStride );
      const int16x8_t d  = vminq_s16( vmaxq_s16( vcombine_s16( a0, a1 ), vmin ), vmax );
      vst1_s16( dst,             vget_low_s16( d ) );
      vst1_s16( dst + dstStride, vget_high_s16( d ) );
    }
  }
  for( ; height > 0; height--, src += srcStride, dst += dstStride )
  {
    int x = 0;
    for( ; x + 8 <= width; x += 8 )
    {
      const int16x8_t a = vld1q_s16( src + x );
      vst1q_s16( dst + x, vminq_s16( vmaxq_s16( a, vmin ), vmax ) );
    }
    if( x < width )
    {
      const int16x4_t a = vld1_s16( src + x );
      vst1_s16( dst + x, vmin_s16( vmax_s16( a, vmin4 ), vmax4 ) );
    }
  }
}

// dst = ClipPel( src ) of a block whose width is a multiple of 8, 16 samples at a time and the last 8
static void copyClip8_neon( const Pel* src, int srcStride, Pel* dst, int dstStride, int width, int height,
                            const ClpRng& clpRng )
{
  CHECKD( ( width & 7 ) != 0, "width must be a multiple of 8" );
  const int16x8_t vmin = vdupq_n_s16( clpRng.min() );
  const int16x8_t vmax = vdupq_n_s16( clpRng.max() );
  for( ; height > 0; height--, src += srcStride, dst += dstStride )
  {
    int x = 0;
    for( ; x + 16 <= width; x += 16 )
    {
      const int16x8_t a0 = vld1q_s16( src + x );
      const int16x8_t a1 = vld1q_s16( src + x + 8 );
      vst1q_s16( dst + x,     vminq_s16( vmaxq_s16( a0, vmin ), vmax ) );
      vst1q_s16( dst + x + 8, vminq_s16( vmaxq_s16( a1, vmin ), vmax ) );
    }
    if( x < width )
    {
      const int16x8_t a = vld1q_s16( src + x );
      vst1q_s16( dst + x, vminq_s16( vmaxq_s16( a, vmin ), vmax ) );
    }
  }
}

// dest = ClipPel( src0 + src1 ) of a block 2 wide, two rows per vector
static void reco2_neon( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dest, int destStride,
                        int width, int height, const ClpRng& clpRng )
{
  CHECKD( width != 2, "only for blocks 2 wide" );
  const int16x4_t vmin = vdup_n_s16( clpRng.min() );
  const int16x4_t vmax = vdup_n_s16( clpRng.max() );
  for( ; height >= 2; height -= 2, src0 += 2 * src0Stride, src1 += 2 * src1Stride, dest += 2 * destStride )
  {
    const int16x4_t a = load_s16x2x2( src0, src0Stride );
    const int16x4_t b = load_s16x2x2( src1, src1Stride );
    store_s16x2x2( dest, vmin_s16( vmax_s16( vqadd_s16( a, b ), vmin ), vmax ), destStride );
  }
  if( height > 0 )
  {
    const int16x4_t a = load_s16x2( src0 );
    const int16x4_t b = load_s16x2( src1 );
    store_s16x2( dest, vmin_s16( vmax_s16( vqadd_s16( a, b ), vmin ), vmax ) );
  }
}

// dest = ClipPel( src0 + src1 ) of a block whose width is a multiple of 4, 8 samples at a time and the last 4
static void reco4_neon( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dest, int destStride,
                        int width, int height, const ClpRng& clpRng )
{
  CHECKD( ( width & 3 ) != 0, "width must be a multiple of 4" );
  const int16x8_t vmin  = vdupq_n_s16( clpRng.min() );
  const int16x8_t vmax  = vdupq_n_s16( clpRng.max() );
  const int16x4_t vmin4 = vget_low_s16( vmin );
  const int16x4_t vmax4 = vget_low_s16( vmax );
  if( width == 4 )
  {
    // a block 4 wide, two rows per vector; an odd last row is left to the loop below
    for( ; height >= 2; height -= 2, src0 += 2 * src0Stride, src1 += 2 * src1Stride, dest += 2 * destStride )
    {
      const int16x4_t a0  = vld1_s16( src0 );
      const int16x4_t a1  = vld1_s16( src0 + src0Stride );
      const int16x4_t b0  = vld1_s16( src1 );
      const int16x4_t b1  = vld1_s16( src1 + src1Stride );
      const int16x8_t sum = vqaddq_s16( vcombine_s16( a0, a1 ), vcombine_s16( b0, b1 ) );
      const int16x8_t d   = vminq_s16( vmaxq_s16( sum, vmin ), vmax );
      vst1_s16( dest,              vget_low_s16( d ) );
      vst1_s16( dest + destStride, vget_high_s16( d ) );
    }
  }
  for( ; height > 0; height--, src0 += src0Stride, src1 += src1Stride, dest += destStride )
  {
    int x = 0;
    for( ; x + 8 <= width; x += 8 )
    {
      const int16x8_t a = vld1q_s16( src0 + x );
      const int16x8_t b = vld1q_s16( src1 + x );
      vst1q_s16( dest + x, vminq_s16( vmaxq_s16( vqaddq_s16( a, b ), vmin ), vmax ) );
    }
    if( x < width )
    {
      const int16x4_t a = vld1_s16( src0 + x );
      const int16x4_t b = vld1_s16( src1 + x );
      vst1_s16( dest + x, vmin_s16( vmax_s16( vqadd_s16( a, b ), vmin4 ), vmax4 ) );
    }
  }
}

// dest = ClipPel( src0 + src1 ) of a block whose width is a multiple of 8, 16 samples at a time and the last 8
static void reco8_neon( const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dest, int destStride,
                        int width, int height, const ClpRng& clpRng )
{
  CHECKD( ( width & 7 ) != 0, "width must be a multiple of 8" );
  const int16x8_t vmin = vdupq_n_s16( clpRng.min() );
  const int16x8_t vmax = vdupq_n_s16( clpRng.max() );
  for( ; height > 0; height--, src0 += src0Stride, src1 += src1Stride, dest += destStride )
  {
    int x = 0;
    for( ; x + 16 <= width; x += 16 )
    {
      const int16x8_t a0 = vld1q_s16( src0 + x );
      const int16x8_t a1 = vld1q_s16( src0 + x + 8 );
      const int16x8_t b0 = vld1q_s16( src1 + x );
      const int16x8_t b1 = vld1q_s16( src1 + x + 8 );
      vst1q_s16( dest + x,     vminq_s16( vmaxq_s16( vqaddq_s16( a0, b0 ), vmin ), vmax ) );
      vst1q_s16( dest + x + 8, vminq_s16( vmaxq_s16( vqaddq_s16( a1, b1 ), vmin ), vmax ) );
    }
    if( x < width )
    {
      const int16x8_t a = vld1q_s16( src0 + x );
      const int16x8_t b = vld1q_s16( src1 + x );
      vst1q_s16( dest + x, vminq_s16( vmaxq_s16( vqaddq_s16( a, b ), vmin ), vmax ) );
    }
  }
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
  weightCiip     = weightCiip_neon;
  roundGeo       = roundGeo_neon;
  roundIntVector = roundIntVector_neon;
  addAvg4  = addAvg_strided_neon<4>;
  addAvg8  = addAvg_strided_neon<8>;
  sub2      = sub2_neon;
  sub4      = sub4_neon;
  copyClip2 = copyClip2_neon;
  copyClip4 = copyClip4_neon;
  copyClip8 = copyClip8_neon;
  reco2     = reco2_neon;
  reco4     = reco4_neon;
  reco8     = reco8_neon;
  addAvg16 = addAvg_strided_neon<16>;
}

} // namespace vvenc

#if defined( _MSC_VER )
#pragma warning( default : 4700 )
#endif

#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER
//! \}

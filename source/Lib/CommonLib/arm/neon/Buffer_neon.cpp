/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER

#if defined( _MSC_VER )
#pragma warning( disable : 4700 ) // MSVC equivalent of GCC_WARNING_DISABLE_maybe_uninitialized
#endif

namespace vvenc
{

void addAvg_neon( const Pel* src0, const Pel* src1, Pel* dest, int numSamples, unsigned rshift, int offset,
                  const ClpRng& clpRng )
{
  CHECK( numSamples < 4, "numSamples must be >= 4" );
  CHECK( offset > 16448, "Offset must be <= 16448" ); // Max: (1 << (rshift - 1)) + 2 * (1 << 13), where rshift=7.

  const int lshift = -static_cast<int>( rshift );

  if( ( numSamples & 15 ) == 0 )
  {
    int n = 0;
    do
    {
      uint16x8_t s1_lo = vreinterpretq_u16_s16( vld1q_s16( src0 + n + 0 ) );
      uint16x8_t s1_hi = vreinterpretq_u16_s16( vld1q_s16( src0 + n + 8 ) );
      uint16x8_t s2_lo = vreinterpretq_u16_s16( vld1q_s16( src1 + n + 0 ) );
      uint16x8_t s2_hi = vreinterpretq_u16_s16( vld1q_s16( src1 + n + 8 ) );

      uint16x8_t d_lo = vaddq_u16( s1_lo, s2_lo );
      d_lo = vaddq_u16( d_lo, vdupq_n_u16( offset ) );
      d_lo = vshlq_u16( d_lo, vdupq_n_s16( lshift ) );
      d_lo = vminq_u16( d_lo, vdupq_n_u16( clpRng.max() ) );
      uint16x8_t d_hi = vaddq_u16( s1_hi, s2_hi );
      d_hi = vaddq_u16( d_hi, vdupq_n_u16( offset ) );
      d_hi = vshlq_u16( d_hi, vdupq_n_s16( lshift ) );
      d_hi = vminq_u16( d_hi, vdupq_n_u16( clpRng.max() ) );

      vst1q_s16( dest + n + 0, vreinterpretq_s16_u16( d_lo ) );
      vst1q_s16( dest + n + 8, vreinterpretq_s16_u16( d_hi ) );

      n += 16;
    } while( n != numSamples );
  }
  else if( numSamples == 8 )
  {
    uint16x8_t s1 = vreinterpretq_u16_s16( vld1q_s16( src0 ) );
    uint16x8_t s2 = vreinterpretq_u16_s16( vld1q_s16( src1 ) );

    uint16x8_t d = vaddq_u16( s1, s2 );
    d = vaddq_u16( d, vdupq_n_u16( offset ) );
    d = vshlq_u16( d, vdupq_n_s16( lshift ) );
    d = vminq_u16( d, vdupq_n_u16( clpRng.max() ) );

    vst1q_s16( dest, vreinterpretq_s16_u16( d ) );
  }
  else if( numSamples == 4 )
  {
    uint16x4_t s1 = vreinterpret_u16_s16( vld1_s16( src0 ) );
    uint16x4_t s2 = vreinterpret_u16_s16( vld1_s16( src1 ) );

    uint16x4_t d = vadd_u16( s1, s2 );
    d = vadd_u16( d, vdup_n_u16( offset ) );
    d = vshl_u16( d, vdup_n_s16( lshift ) );
    d = vmin_u16( d, vdup_n_u16( clpRng.max() ) );

    vst1_s16( dest, vreinterpret_s16_u16( d ) );
  }
  else
  {
    THROW( "Unsupported size. numSamples must be 4 or 8 or a multiple of 16" );
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
  CHECK( offset > 16448, "Offset must be <= 16448" ); // Max: (1 << (rshift - 1)) + 2 * (1 << 13), where rshift=7.

  const int lshift = -static_cast<int>( rshift );

  do
  {
    int w = 0;
    do
    {
      uint16x8_t s1_lo = vreinterpretq_u16_s16( vld1q_s16( src0 + w + 0 ) );
      uint16x8_t s1_hi = vreinterpretq_u16_s16( vld1q_s16( src0 + w + 8 ) );
      uint16x8_t s2_lo = vreinterpretq_u16_s16( vld1q_s16( src1 + w + 0 ) );
      uint16x8_t s2_hi = vreinterpretq_u16_s16( vld1q_s16( src1 + w + 8 ) );

      uint16x8_t d_lo = vaddq_u16( s1_lo, s2_lo );
      d_lo = vaddq_u16( d_lo, vdupq_n_u16( offset ) );
      d_lo = vshlq_u16( d_lo, vdupq_n_s16( lshift ) );
      d_lo = vminq_u16( d_lo, vdupq_n_u16( clpRng.max() ) );
      uint16x8_t d_hi = vaddq_u16( s1_hi, s2_hi );
      d_hi = vaddq_u16( d_hi, vdupq_n_u16( offset ) );
      d_hi = vshlq_u16( d_hi, vdupq_n_s16( lshift ) );
      d_hi = vminq_u16( d_hi, vdupq_n_u16( clpRng.max() ) );

      vst1q_s16( dest + w + 0, vreinterpretq_s16_u16( d_lo ) );
      vst1q_s16( dest + w + 8, vreinterpretq_s16_u16( d_hi ) );

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
  CHECK( offset > 16448, "Offset must be <= 16448" ); // Max: (1 << (rshift - 1)) + 2 * (1 << 13), where rshift=7.

  const int lshift = -static_cast<int>( rshift );

  do
  {
    int w = 0;
    do
    {
      uint16x8_t s1 = vreinterpretq_u16_s16( vld1q_s16( src0 + w ) );
      uint16x8_t s2 = vreinterpretq_u16_s16( vld1q_s16( src1 + w ) );

      uint16x8_t d = vaddq_u16( s1, s2 );
      d = vaddq_u16( d, vdupq_n_u16( offset ) );
      d = vshlq_u16( d, vdupq_n_s16( lshift ) );
      d = vminq_u16( d, vdupq_n_u16( clpRng.max() ) );

      vst1q_s16( dest + w, vreinterpretq_s16_u16( d ) );

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
  CHECK( offset > 16448, "Offset must be <= 16448" ); // Max: (1 << (rshift - 1)) + 2 * (1 << 13), where rshift=7.

  const int lshift = -static_cast<int>( rshift );

  do
  {
    int w = 0;
    do
    {
      uint16x4_t s1 = vreinterpret_u16_s16( vld1_s16( src0 + w ) );
      uint16x4_t s2 = vreinterpret_u16_s16( vld1_s16( src1 + w ) );

      uint16x4_t d = vadd_u16( s1, s2 );
      d = vadd_u16( d, vdup_n_u16( offset ) );
      d = vshl_u16( d, vdup_n_s16( lshift ) );
      d = vmin_u16( d, vdup_n_u16( clpRng.max() ) );

      vst1_s16( dest + w, vreinterpret_s16_u16( d ) );

      w += 4;
    } while( w != width );

    src0 += src0Stride;
    src1 += src1Stride;
    dest += destStride;
  } while( --height != 0 );
}

void applyLut_neon( const Pel* src, const ptrdiff_t srcStride, Pel* dst, const ptrdiff_t dstStride, int width,
                    int height, const Pel* lut )
{

  if( ( width & 31 ) == 0 && ( height & 3 ) == 0 )
  {
    int16x8x4_t xtmp1;
    int16x8x4_t xtmp2;
    int16x8x4_t xtmp3;
    int16x8x4_t xtmp4;

    for( int y = 0; y < height; y += 4 )
    {
      for( int x = 0; x < width; x += 32 )
      {
        GCC_WARNING_DISABLE_maybe_uninitialized // when building for aarch64 without LTO gcc complains about
                                                // xtmp{1,2,3,4}.val[] not being initialized

        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 0]], xtmp1.val[0], 0 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 1]], xtmp1.val[1], 0 );
        xtmp1.val[2] = vsetq_lane_s16( lut[src[x + 2]], xtmp1.val[2], 0 );
        xtmp1.val[3] = vsetq_lane_s16( lut[src[x + 3]], xtmp1.val[3], 0 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 4]], xtmp1.val[0], 1 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 5]], xtmp1.val[1], 1 );
        xtmp1.val[2] = vsetq_lane_s16( lut[src[x + 6]], xtmp1.val[2], 1 );
        xtmp1.val[3] = vsetq_lane_s16( lut[src[x + 7]], xtmp1.val[3], 1 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 8]], xtmp1.val[0], 2 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 9]], xtmp1.val[1], 2 );
        xtmp1.val[2] = vsetq_lane_s16( lut[src[x + 10]], xtmp1.val[2], 2 );
        xtmp1.val[3] = vsetq_lane_s16( lut[src[x + 11]], xtmp1.val[3], 2 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 12]], xtmp1.val[0], 3 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 13]], xtmp1.val[1], 3 );
        xtmp1.val[2] = vsetq_lane_s16( lut[src[x + 14]], xtmp1.val[2], 3 );
        xtmp1.val[3] = vsetq_lane_s16( lut[src[x + 15]], xtmp1.val[3], 3 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 16]], xtmp1.val[0], 4 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 17]], xtmp1.val[1], 4 );
        xtmp1.val[2] = vsetq_lane_s16( lut[src[x + 18]], xtmp1.val[2], 4 );
        xtmp1.val[3] = vsetq_lane_s16( lut[src[x + 19]], xtmp1.val[3], 4 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 20]], xtmp1.val[0], 5 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 21]], xtmp1.val[1], 5 );
        xtmp1.val[2] = vsetq_lane_s16( lut[src[x + 22]], xtmp1.val[2], 5 );
        xtmp1.val[3] = vsetq_lane_s16( lut[src[x + 23]], xtmp1.val[3], 5 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 24]], xtmp1.val[0], 6 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 25]], xtmp1.val[1], 6 );
        xtmp1.val[2] = vsetq_lane_s16( lut[src[x + 26]], xtmp1.val[2], 6 );
        xtmp1.val[3] = vsetq_lane_s16( lut[src[x + 27]], xtmp1.val[3], 6 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 28]], xtmp1.val[0], 7 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 29]], xtmp1.val[1], 7 );
        xtmp1.val[2] = vsetq_lane_s16( lut[src[x + 30]], xtmp1.val[2], 7 );
        xtmp1.val[3] = vsetq_lane_s16( lut[src[x + 31]], xtmp1.val[3], 7 );

        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 0]], xtmp2.val[0], 0 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 1]], xtmp2.val[1], 0 );
        xtmp2.val[2] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 2]], xtmp2.val[2], 0 );
        xtmp2.val[3] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 3]], xtmp2.val[3], 0 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 4]], xtmp2.val[0], 1 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 5]], xtmp2.val[1], 1 );
        xtmp2.val[2] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 6]], xtmp2.val[2], 1 );
        xtmp2.val[3] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 7]], xtmp2.val[3], 1 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 8]], xtmp2.val[0], 2 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 9]], xtmp2.val[1], 2 );
        xtmp2.val[2] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 10]], xtmp2.val[2], 2 );
        xtmp2.val[3] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 11]], xtmp2.val[3], 2 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 12]], xtmp2.val[0], 3 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 13]], xtmp2.val[1], 3 );
        xtmp2.val[2] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 14]], xtmp2.val[2], 3 );
        xtmp2.val[3] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 15]], xtmp2.val[3], 3 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 16]], xtmp2.val[0], 4 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 17]], xtmp2.val[1], 4 );
        xtmp2.val[2] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 18]], xtmp2.val[2], 4 );
        xtmp2.val[3] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 19]], xtmp2.val[3], 4 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 20]], xtmp2.val[0], 5 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 21]], xtmp2.val[1], 5 );
        xtmp2.val[2] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 22]], xtmp2.val[2], 5 );
        xtmp2.val[3] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 23]], xtmp2.val[3], 5 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 24]], xtmp2.val[0], 6 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 25]], xtmp2.val[1], 6 );
        xtmp2.val[2] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 26]], xtmp2.val[2], 6 );
        xtmp2.val[3] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 27]], xtmp2.val[3], 6 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 28]], xtmp2.val[0], 7 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 29]], xtmp2.val[1], 7 );
        xtmp2.val[2] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 30]], xtmp2.val[2], 7 );
        xtmp2.val[3] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 31]], xtmp2.val[3], 7 );

        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 0]], xtmp3.val[0], 0 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 1]], xtmp3.val[1], 0 );
        xtmp3.val[2] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 2]], xtmp3.val[2], 0 );
        xtmp3.val[3] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 3]], xtmp3.val[3], 0 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 4]], xtmp3.val[0], 1 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 5]], xtmp3.val[1], 1 );
        xtmp3.val[2] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 6]], xtmp3.val[2], 1 );
        xtmp3.val[3] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 7]], xtmp3.val[3], 1 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 8]], xtmp3.val[0], 2 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 9]], xtmp3.val[1], 2 );
        xtmp3.val[2] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 10]], xtmp3.val[2], 2 );
        xtmp3.val[3] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 11]], xtmp3.val[3], 2 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 12]], xtmp3.val[0], 3 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 13]], xtmp3.val[1], 3 );
        xtmp3.val[2] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 14]], xtmp3.val[2], 3 );
        xtmp3.val[3] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 15]], xtmp3.val[3], 3 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 16]], xtmp3.val[0], 4 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 17]], xtmp3.val[1], 4 );
        xtmp3.val[2] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 18]], xtmp3.val[2], 4 );
        xtmp3.val[3] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 19]], xtmp3.val[3], 4 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 20]], xtmp3.val[0], 5 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 21]], xtmp3.val[1], 5 );
        xtmp3.val[2] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 22]], xtmp3.val[2], 5 );
        xtmp3.val[3] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 23]], xtmp3.val[3], 5 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 24]], xtmp3.val[0], 6 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 25]], xtmp3.val[1], 6 );
        xtmp3.val[2] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 26]], xtmp3.val[2], 6 );
        xtmp3.val[3] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 27]], xtmp3.val[3], 6 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 28]], xtmp3.val[0], 7 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 29]], xtmp3.val[1], 7 );
        xtmp3.val[2] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 30]], xtmp3.val[2], 7 );
        xtmp3.val[3] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 31]], xtmp3.val[3], 7 );

        // interleaved assign -> there is only interleaved storing/loading
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 0]], xtmp4.val[0], 0 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 1]], xtmp4.val[1], 0 );
        xtmp4.val[2] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 2]], xtmp4.val[2], 0 );
        xtmp4.val[3] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 3]], xtmp4.val[3], 0 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 4]], xtmp4.val[0], 1 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 5]], xtmp4.val[1], 1 );
        xtmp4.val[2] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 6]], xtmp4.val[2], 1 );
        xtmp4.val[3] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 7]], xtmp4.val[3], 1 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 8]], xtmp4.val[0], 2 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 9]], xtmp4.val[1], 2 );
        xtmp4.val[2] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 10]], xtmp4.val[2], 2 );
        xtmp4.val[3] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 11]], xtmp4.val[3], 2 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 12]], xtmp4.val[0], 3 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 13]], xtmp4.val[1], 3 );
        xtmp4.val[2] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 14]], xtmp4.val[2], 3 );
        xtmp4.val[3] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 15]], xtmp4.val[3], 3 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 16]], xtmp4.val[0], 4 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 17]], xtmp4.val[1], 4 );
        xtmp4.val[2] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 18]], xtmp4.val[2], 4 );
        xtmp4.val[3] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 19]], xtmp4.val[3], 4 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 20]], xtmp4.val[0], 5 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 21]], xtmp4.val[1], 5 );
        xtmp4.val[2] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 22]], xtmp4.val[2], 5 );
        xtmp4.val[3] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 23]], xtmp4.val[3], 5 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 24]], xtmp4.val[0], 6 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 25]], xtmp4.val[1], 6 );
        xtmp4.val[2] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 26]], xtmp4.val[2], 6 );
        xtmp4.val[3] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 27]], xtmp4.val[3], 6 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 28]], xtmp4.val[0], 7 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 29]], xtmp4.val[1], 7 );
        xtmp4.val[2] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 30]], xtmp4.val[2], 7 );
        xtmp4.val[3] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 31]], xtmp4.val[3], 7 );

        GCC_WARNING_RESET

        // deinterleaved storing
        vst4q_s16( &dst[x], xtmp1 );
        vst4q_s16( &dst[x + 1 * dstStride], xtmp2 );
        vst4q_s16( &dst[x + 2 * dstStride], xtmp3 );
        vst4q_s16( &dst[x + 3 * dstStride], xtmp4 );
      }
      src += ( srcStride << 2 );
      dst += ( dstStride << 2 );
    }
  }
  else if( ( width & 15 ) == 0 && ( height & 3 ) == 0 )
  {
    int16x8x2_t xtmp1;
    int16x8x2_t xtmp2;
    int16x8x2_t xtmp3;
    int16x8x2_t xtmp4;

    for( int y = 0; y < height; y += 4 )
    {
      for( int x = 0; x < width; x += 16 )
      {
        // vld2q_s16( &src[ x ] );

        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 0]], xtmp1.val[0], 0 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 1]], xtmp1.val[1], 0 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 2]], xtmp1.val[0], 1 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 3]], xtmp1.val[1], 1 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 4]], xtmp1.val[0], 2 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 5]], xtmp1.val[1], 2 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 6]], xtmp1.val[0], 3 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 7]], xtmp1.val[1], 3 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 8]], xtmp1.val[0], 4 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 9]], xtmp1.val[1], 4 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 10]], xtmp1.val[0], 5 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 11]], xtmp1.val[1], 5 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 12]], xtmp1.val[0], 6 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 13]], xtmp1.val[1], 6 );
        xtmp1.val[0] = vsetq_lane_s16( lut[src[x + 14]], xtmp1.val[0], 7 );
        xtmp1.val[1] = vsetq_lane_s16( lut[src[x + 15]], xtmp1.val[1], 7 );

        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 0]], xtmp2.val[0], 0 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 1]], xtmp2.val[1], 0 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 2]], xtmp2.val[0], 1 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 3]], xtmp2.val[1], 1 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 4]], xtmp2.val[0], 2 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 5]], xtmp2.val[1], 2 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 6]], xtmp2.val[0], 3 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 7]], xtmp2.val[1], 3 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 8]], xtmp2.val[0], 4 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 9]], xtmp2.val[1], 4 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 10]], xtmp2.val[0], 5 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 11]], xtmp2.val[1], 5 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 12]], xtmp2.val[0], 6 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 13]], xtmp2.val[1], 6 );
        xtmp2.val[0] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 14]], xtmp2.val[0], 7 );
        xtmp2.val[1] = vsetq_lane_s16( lut[src[x + 1 * srcStride + 15]], xtmp2.val[1], 7 );

        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 0]], xtmp3.val[0], 0 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 1]], xtmp3.val[1], 0 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 2]], xtmp3.val[0], 1 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 3]], xtmp3.val[1], 1 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 4]], xtmp3.val[0], 2 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 5]], xtmp3.val[1], 2 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 6]], xtmp3.val[0], 3 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 7]], xtmp3.val[1], 3 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 8]], xtmp3.val[0], 4 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 9]], xtmp3.val[1], 4 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 10]], xtmp3.val[0], 5 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 11]], xtmp3.val[1], 5 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 12]], xtmp3.val[0], 6 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 13]], xtmp3.val[1], 6 );
        xtmp3.val[0] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 14]], xtmp3.val[0], 7 );
        xtmp3.val[1] = vsetq_lane_s16( lut[src[x + 2 * srcStride + 15]], xtmp3.val[1], 7 );

        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 0]], xtmp4.val[0], 0 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 1]], xtmp4.val[1], 0 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 2]], xtmp4.val[0], 1 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 3]], xtmp4.val[1], 1 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 4]], xtmp4.val[0], 2 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 5]], xtmp4.val[1], 2 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 6]], xtmp4.val[0], 3 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 7]], xtmp4.val[1], 3 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 8]], xtmp4.val[0], 4 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 9]], xtmp4.val[1], 4 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 10]], xtmp4.val[0], 5 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 11]], xtmp4.val[1], 5 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 12]], xtmp4.val[0], 6 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 13]], xtmp4.val[1], 6 );
        xtmp4.val[0] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 14]], xtmp4.val[0], 7 );
        xtmp4.val[1] = vsetq_lane_s16( lut[src[x + 3 * srcStride + 15]], xtmp4.val[1], 7 );

        vst2q_s16( &dst[x], xtmp1 );
        vst2q_s16( &dst[x + 1 * dstStride], xtmp2 );
        vst2q_s16( &dst[x + 2 * dstStride], xtmp3 );
        vst2q_s16( &dst[x + 3 * dstStride], xtmp4 );
      }
      src += ( srcStride << 2 );
      dst += ( dstStride << 2 );
    }
  }
  else if( ( width & 7 ) == 0 && ( height & 3 ) == 0 )
  {
    int16x8_t xtmp1;
    int16x8_t xtmp2;
    int16x8_t xtmp3;
    int16x8_t xtmp4;

    for( int y = 0; y < height; y += 4 )
    {
      for( int x = 0; x < width; x += 8 )
      {
        GCC_WARNING_DISABLE_maybe_uninitialized

        xtmp1 = vsetq_lane_s16( lut[src[x + 0]], xtmp1, 0 );
        xtmp1 = vsetq_lane_s16( lut[src[x + 1]], xtmp1, 1 );
        xtmp1 = vsetq_lane_s16( lut[src[x + 2]], xtmp1, 2 );
        xtmp1 = vsetq_lane_s16( lut[src[x + 3]], xtmp1, 3 );
        xtmp1 = vsetq_lane_s16( lut[src[x + 4]], xtmp1, 4 );
        xtmp1 = vsetq_lane_s16( lut[src[x + 5]], xtmp1, 5 );
        xtmp1 = vsetq_lane_s16( lut[src[x + 6]], xtmp1, 6 );
        xtmp1 = vsetq_lane_s16( lut[src[x + 7]], xtmp1, 7 );

        xtmp2 = vsetq_lane_s16( lut[src[x + 1 * srcStride + 0]], xtmp2, 0 );
        xtmp2 = vsetq_lane_s16( lut[src[x + 1 * srcStride + 1]], xtmp2, 1 );
        xtmp2 = vsetq_lane_s16( lut[src[x + 1 * srcStride + 2]], xtmp2, 2 );
        xtmp2 = vsetq_lane_s16( lut[src[x + 1 * srcStride + 3]], xtmp2, 3 );
        xtmp2 = vsetq_lane_s16( lut[src[x + 1 * srcStride + 4]], xtmp2, 4 );
        xtmp2 = vsetq_lane_s16( lut[src[x + 1 * srcStride + 5]], xtmp2, 5 );
        xtmp2 = vsetq_lane_s16( lut[src[x + 1 * srcStride + 6]], xtmp2, 6 );
        xtmp2 = vsetq_lane_s16( lut[src[x + 1 * srcStride + 7]], xtmp2, 7 );

        xtmp3 = vsetq_lane_s16( lut[src[x + 2 * srcStride + 0]], xtmp3, 0 );
        xtmp3 = vsetq_lane_s16( lut[src[x + 2 * srcStride + 1]], xtmp3, 1 );
        xtmp3 = vsetq_lane_s16( lut[src[x + 2 * srcStride + 2]], xtmp3, 2 );
        xtmp3 = vsetq_lane_s16( lut[src[x + 2 * srcStride + 3]], xtmp3, 3 );
        xtmp3 = vsetq_lane_s16( lut[src[x + 2 * srcStride + 4]], xtmp3, 4 );
        xtmp3 = vsetq_lane_s16( lut[src[x + 2 * srcStride + 5]], xtmp3, 5 );
        xtmp3 = vsetq_lane_s16( lut[src[x + 2 * srcStride + 6]], xtmp3, 6 );
        xtmp3 = vsetq_lane_s16( lut[src[x + 2 * srcStride + 7]], xtmp3, 7 );

        xtmp4 = vsetq_lane_s16( lut[src[x + 3 * srcStride + 0]], xtmp4, 0 );
        xtmp4 = vsetq_lane_s16( lut[src[x + 3 * srcStride + 1]], xtmp4, 1 );
        xtmp4 = vsetq_lane_s16( lut[src[x + 3 * srcStride + 2]], xtmp4, 2 );
        xtmp4 = vsetq_lane_s16( lut[src[x + 3 * srcStride + 3]], xtmp4, 3 );
        xtmp4 = vsetq_lane_s16( lut[src[x + 3 * srcStride + 4]], xtmp4, 4 );
        xtmp4 = vsetq_lane_s16( lut[src[x + 3 * srcStride + 5]], xtmp4, 5 );
        xtmp4 = vsetq_lane_s16( lut[src[x + 3 * srcStride + 6]], xtmp4, 6 );
        xtmp4 = vsetq_lane_s16( lut[src[x + 3 * srcStride + 7]], xtmp4, 7 );

        GCC_WARNING_RESET

        vst1q_s16( &dst[x], xtmp1 );
        vst1q_s16( &dst[x + 1 * dstStride], xtmp2 );
        vst1q_s16( &dst[x + 2 * dstStride], xtmp3 );
        vst1q_s16( &dst[x + 3 * dstStride], xtmp4 );
      }

      src += ( srcStride << 2 );
      dst += ( dstStride << 2 );
    }
  }
  else
  {
#define RSP_SGNL_OP( ADDR ) dst[ADDR] = lut[src[ADDR]]
#define RSP_SGNL_INC        src += srcStride; dst += dstStride;

    SIZE_AWARE_PER_EL_OP( RSP_SGNL_OP, RSP_SGNL_INC )

#undef RSP_SGNL_OP
#undef RSP_SGNL_INC
  }
}

template<>
void PelBufferOps::_initPelBufOpsARM<NEON>()
{
  addAvg   = addAvg_neon;
  addAvg4  = addAvg_strided_neon<4>;
  addAvg8  = addAvg_strided_neon<8>;
  addAvg16 = addAvg_strided_neon<16>;

  applyLut = applyLut_neon;
}

} // namespace vvenc

#if defined( _MSC_VER )
#pragma warning( default : 4700 )
#endif

#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER
//! \}

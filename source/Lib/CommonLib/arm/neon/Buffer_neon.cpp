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

template<>
void PelBufferOps::_initPelBufOpsARM<NEON>()
{
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

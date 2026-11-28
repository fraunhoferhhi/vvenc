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
 * \file BvSearchOps_neon.cpp
 * \brief Neon helpers of the IBC block vector search.
 */

#include "EncoderLib/BvSearchOps.h"
#include "CommonLib/arm/mem_neon.h"
#include "CommonLib/arm/neon/sum_neon.h"
#include <arm_neon.h>

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT

namespace vvenc
{

// true if mv is one of the first n entries of list. A motion vector consists of two 32-bit components, so one 64-bit
// lane compares one candidate; eight candidates are tested per branch. The loads are 32-bit: an Mv is 4-byte aligned.
static bool containsMv_neon( const Mv* list, const unsigned n, const Mv& mv )
{
  static_assert( sizeof( Mv ) == 2 * sizeof( int32_t ), "an Mv has to consist of its two 32-bit components only" );

  const uint32_t*  cand = ( const uint32_t* ) list;
  const uint32x2_t mv32 = vld1_u32( ( const uint32_t* ) &mv );
  const uint64x2_t key  = vreinterpretq_u64_u32( vcombine_u32( mv32, mv32 ) );

  unsigned i = 0;
  for( ; i + 8 <= n; i += 8 )
  {
    const uint64x2_t c0 = vreinterpretq_u64_u32( vld1q_u32( cand + 2 * i ) );
    const uint64x2_t c1 = vreinterpretq_u64_u32( vld1q_u32( cand + 2 * i + 4 ) );
    const uint64x2_t c2 = vreinterpretq_u64_u32( vld1q_u32( cand + 2 * i + 8 ) );
    const uint64x2_t c3 = vreinterpretq_u64_u32( vld1q_u32( cand + 2 * i + 12 ) );
    const uint64x2_t eq01 = vorrq_u64( vvenc_vceqq_u64( c0, key ), vvenc_vceqq_u64( c1, key ) );
    const uint64x2_t eq23 = vorrq_u64( vvenc_vceqq_u64( c2, key ), vvenc_vceqq_u64( c3, key ) );
    if( any_lane_set_u64x2( vorrq_u64( eq01, eq23 ) ) )
    {
      return true;
    }
  }
  for( ; i + 2 <= n; i += 2 )
  {
    const uint64x2_t c = vreinterpretq_u64_u32( vld1q_u32( cand + 2 * i ) );
    if( any_lane_set_u64x2( vvenc_vceqq_u64( c, key ) ) )
    {
      return true;
    }
  }
  return i < n && list[i] == mv;
}

// acc += ( |o - c| >> shift ) of 8 or 4 samples; vabd is exact for samples in 0 ... 2^15 - 1, and vshl by -shift is the
// logical right shift
static inline uint32x4_t addAbsDiffShifted( uint32x4_t acc, int16x8_t o, int16x8_t c, int16x8_t negShift )
{
  return vpadalq_u16( acc, vshlq_u16( vreinterpretq_u16_s16( vabdq_s16( o, c ) ), negShift ) );
}

static inline uint32x4_t addAbsDiffShifted( uint32x4_t acc, int16x4_t o, int16x4_t c, int16x8_t negShift )
{
  return vaddw_u16( acc, vshl_u16( vreinterpret_u16_s16( vabd_s16( o, c ) ), vget_low_s16( negShift ) ) );
}

// ( |org - cur| >> shift ) of one row: vectors of 8 samples into acc, the last width % 8 samples into tail
static inline void addRowAbsDiffShifted( const Pel* org, const Pel* cur, const int width, const int shift, int16x8_t negShift, uint32x4_t& acc, Distortion& tail )
{
  int x = 0;
  for( ; x + 8 <= width; x += 8 )
  {
    const int16x8_t o = vld1q_s16( org + x );
    const int16x8_t c = vld1q_s16( cur + x );
    acc = addAbsDiffShifted( acc, o, c, negShift );
  }
  for( ; x < width; x++ )
  {
    tail += abs( cur[x] - org[x] ) >> shift;
  }
}

// sum over the block of ( |org - cur| >> shift ), for samples in 0 ... 2^15 - 1: the 32-bit lane sums cannot overflow
// for blocks of up to 2^16 samples. Blocks 2 or 4 wide put two rows in one vector.
static Distortion sumAbsDiffShifted_neon( const Pel* org, const ptrdiff_t orgStride, const Pel* cur, const ptrdiff_t curStride, const int width, const int height, const int shift )
{
  CHECKD( width * height > ( 1 << 16 ), "block too large for 32-bit lane sums" );

  const int16x8_t negShift = vdupq_n_s16( -shift );
  uint32x4_t      acc0     = vdupq_n_u32( 0 );
  uint32x4_t      acc1     = vdupq_n_u32( 0 );
  Distortion      tail     = 0;

  int y = 0;
  if( width == 2 )
  {
    for( ; y + 2 <= height; y += 2, org += 2 * orgStride, cur += 2 * curStride )
    {
      const int16x4_t o = load_s16x2x2( org, orgStride );
      const int16x4_t c = load_s16x2x2( cur, curStride );
      acc0 = addAbsDiffShifted( acc0, o, c, negShift );
    }
    if( y < height )
    {
      const int16x4_t o = load_s16x2( org );
      const int16x4_t c = load_s16x2( cur );
      acc0 = addAbsDiffShifted( acc0, o, c, negShift );
    }
  }
  else if( width == 4 )
  {
    for( ; y + 2 <= height; y += 2, org += 2 * orgStride, cur += 2 * curStride )
    {
      const int16x4_t o0 = vld1_s16( org );
      const int16x4_t o1 = vld1_s16( org + orgStride );
      const int16x4_t c0 = vld1_s16( cur );
      const int16x4_t c1 = vld1_s16( cur + curStride );
      acc0 = addAbsDiffShifted( acc0, vcombine_s16( o0, o1 ), vcombine_s16( c0, c1 ), negShift );
    }
    if( y < height )
    {
      const int16x4_t o = vld1_s16( org );
      const int16x4_t c = vld1_s16( cur );
      acc0 = addAbsDiffShifted( acc0, o, c, negShift );
    }
  }
  else
  {
    // even rows into acc0, odd rows into acc1: two independent accumulation chains
    for( ; y + 2 <= height; y += 2, org += 2 * orgStride, cur += 2 * curStride )
    {
      addRowAbsDiffShifted( org, cur, width, shift, negShift, acc0, tail );
      addRowAbsDiffShifted( org + orgStride, cur + curStride, width, shift, negShift, acc1, tail );
    }
    if( y < height )
    {
      addRowAbsDiffShifted( org, cur, width, shift, negShift, acc0, tail );
    }
  }

  return tail + horizontal_add_u32x4( vaddq_u32( acc0, acc1 ) );
}

template<>
void BvSearchOps::_initBvSearchOpsARM<NEON>()
{
  containsMv        = containsMv_neon;
  sumAbsDiffShifted = sumAbsDiffShifted_neon;
}

} // namespace vvenc

#endif

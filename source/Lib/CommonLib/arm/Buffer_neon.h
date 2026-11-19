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
/** \file     Buffer_neon.h
    \brief    SIMD for PelBufferOps: code shared by the NEON and SVE kernels
*/

#pragma once

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"

#include <arm_neon.h>
#include <cstring>

namespace vvenc
{
#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER

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

// sumSquaredError with the arithmetic of sumSquaredError_SIMD at SSE4.1: the differences in 16 bits (they wrap like
// _mm_sub_epi16) and their squares (at most 2^30), summed by SquareSum, which adds the squares of the differences of
// two vectors (add( a, b )), merges another sum (add( other )) and returns the total (sum()). Integer addition does not
// depend on the order, so the sum is the same as x86's and the C code's whenever the differences lie in
// [-2^15, 2^15], as for any samples of up to 15 bits.
template<class SquareSum>
static inline uint64_t sumSquaredError_common( const Pel* src0, const ptrdiff_t stride0, const Pel* src1,
                                               const ptrdiff_t stride1, const int width, const int height )
{
  // below 16 the encoder passes only the widths 2, 4 and 8 (SBT parts); the others take the C code
  if( width < 16 && width != 2 && width != 4 && width != 8 )
  {
    return sumSquaredErrorCore( src0, stride0, src1, stride1, width, height );
  }

  SquareSum acc0;

  if( width == 2 )
  {
    int y = 0;
    for( ; y + 4 <= height; y += 4, src0 += 4 * stride0, src1 += 4 * stride1 )
    {
      const int16x8_t a = loadWidth2Rows_neon( src0, stride0, 4 );
      const int16x8_t b = loadWidth2Rows_neon( src1, stride1, 4 );
      acc0.add( a, b );
    }
    if( y < height )
    {
      const int16x8_t a = loadWidth2Rows_neon( src0, stride0, height - y );
      const int16x8_t b = loadWidth2Rows_neon( src1, stride1, height - y );
      acc0.add( a, b );
    }
    return acc0.sum();
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
      acc0.add( vcombine_s16( a0, a1 ), vcombine_s16( b0, b1 ) );
    }
    if( y < height )
    {
      const int16x4_t a = vld1_s16( src0 );
      const int16x4_t b = vld1_s16( src1 );
      acc0.add( a, b );
    }
    return acc0.sum();
  }

  // widths 8 and above: two accumulators (four from width 32), for two rows (width 8) or the two halves of 16 columns
  SquareSum acc1;

  if( width == 8 )
  {
    int y = 0;
    for( ; y + 2 <= height; y += 2, src0 += 2 * stride0, src1 += 2 * stride1 )
    {
      const int16x8_t a0 = vld1q_s16( src0 );
      const int16x8_t b0 = vld1q_s16( src1 );
      const int16x8_t a1 = vld1q_s16( src0 + stride0 );
      const int16x8_t b1 = vld1q_s16( src1 + stride1 );
      acc0.add( a0, b0 );
      acc1.add( a1, b1 );
    }
    if( y < height )
    {
      const int16x8_t a = vld1q_s16( src0 );
      const int16x8_t b = vld1q_s16( src1 );
      acc0.add( a, b );
    }
    acc0.add( acc1 );
    return acc0.sum();
  }

  if( width == 16 )
  {
    for( int y = 0; y < height; y++, src0 += stride0, src1 += stride1 )
    {
      const int16x8_t a0 = vld1q_s16( src0 );
      const int16x8_t b0 = vld1q_s16( src1 );
      const int16x8_t a1 = vld1q_s16( src0 + 8 );
      const int16x8_t b1 = vld1q_s16( src1 + 8 );
      acc0.add( a0, b0 );
      acc1.add( a1, b1 );
    }
    acc0.add( acc1 );
    return acc0.sum();
  }

  // wider: 32 columns per step on four accumulators, then 16 columns if that many are left; the last width % 16
  // columns as the 16 ending at the row's end, with the lanes of the columns already counted set to zero in both
  // sources
  SquareSum acc2;
  SquareSum acc3;

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
      acc0.add( a0, b0 );
      acc1.add( a1, b1 );
      acc2.add( a2, b2 );
      acc3.add( a3, b3 );
    }
    if( x + 16 <= width )
    {
      const int16x8_t a0 = vld1q_s16( &src0[x] );
      const int16x8_t b0 = vld1q_s16( &src1[x] );
      const int16x8_t a1 = vld1q_s16( &src0[x + 8] );
      const int16x8_t b1 = vld1q_s16( &src1[x + 8] );
      acc0.add( a0, b0 );
      acc1.add( a1, b1 );
    }
    if( tail != 0 )
    {
      const int16x8_t a0 = vld1q_s16( &src0[lastX] );
      const int16x8_t b0 = vld1q_s16( &src1[lastX] );
      const int16x8_t a1 = vld1q_s16( &src0[lastX + 8] );
      const int16x8_t b1 = vld1q_s16( &src1[lastX + 8] );
      acc0.add( vandq_s16( a0, keep0 ), vandq_s16( b0, keep0 ) );
      acc1.add( vandq_s16( a1, keep1 ), vandq_s16( b1, keep1 ) );
    }
  }
  acc0.add( acc1 );
  acc2.add( acc3 );
  acc0.add( acc2 );
  return acc0.sum();
}

#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER
} // namespace vvenc

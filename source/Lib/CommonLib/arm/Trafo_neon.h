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
/** \file     Trafo_neon.h
    \brief    SIMD for the LFNST: code shared by the NEON and SVE kernels
*/

#pragma once

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "neon/sum_neon.h"

#include <arm_neon.h>
#include <cstring>

namespace vvenc
{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_TRAFO

// LFNST with the arithmetic of simdInvLfnstNxN / simdFwdLfnstNxN at SSE4.1: each output is the dot product of the
// input and one int8 matrix row in 32 bits, wrapping mod 2^32 like _mm_mullo_epi32 and _mm_add_epi32, then rounded
// to ( sum + 64 ) >> 7 with vrshr. When every input read fits int16, RowDots::dot4 takes the products of int16 and int8
// values; their sum is at most 48 * 2^15 * 2^7 = 1.5 * 2^27 in magnitude, so it does not wrap and equals the 32-bit
// sum. Otherwise the 32-bit products are taken 4 at a time with vmlaq_s32. vrshr does not wrap the + 64 as
// _mm_add_epi32 does, which differs only for sums within 64 of INT32_MAX, where the C code's + 64 overflows.

// true when all n (a multiple of 8) values of src fit int16
static inline bool lfnstInputFitsInt16( const int* src, const int n )
{
  uint32x4_t differs = vdupq_n_u32( 0 );
  for( int i = 0; i < n; i += 4 )
  {
    const int32x4_t v = vld1q_s32( src + i );
    // v minus its value saturated to int16: zero exactly where v fits
    differs = vorrq_u32( differs, vreinterpretq_u32_s32( vsubw_s16( v, vqmovn_s32( v ) ) ) );
  }
  return !any_lane_set_u32x4( differs );
}

// the n (a multiple of 8) values of src as int16, 8 per vector
static inline void lfnstNarrowInput( const int* src, const int n, int16x8_t* dst )
{
  for( int i = 0; i < n; i += 8 )
  {
    const int32x4_t lo = vld1q_s32( src + i );
    const int32x4_t hi = vld1q_s32( src + i + 4 );
    dst[i / 8] = vcombine_s16( vmovn_s32( lo ), vmovn_s32( hi ) );
  }
}

// dot product of the n (8, 16 or 48) int32 inputs and an int8 matrix row, as four 32-bit partial sums (wrapping)
static inline int32x4_t lfnstDot32( const int* src, const int8_t* row, const int n )
{
  int32x4_t acc = vdupq_n_s32( 0 );
  for( int i = 0; i < n; i += 8 )
  {
    const int8x8_t  m8 = vld1_s8( row + i );
    const int16x8_t m  = vmovl_s8( m8 );
    const int32x4_t s0 = vld1q_s32( src + i );
    const int32x4_t s1 = vld1q_s32( src + i + 4 );
    acc = vmlaq_s32( acc, s0, vmovl_s16( vget_low_s16( m ) ) );
    acc = vmlaq_s32( acc, s1, vmovl_s16( vget_high_s16( m ) ) );
  }
  return acc;
}

// out[j] = ( dot( src, mat row j ) + 64 ) >> 7 for numOut (a multiple of 4) rows of length n, rows rowStride apart;
// in16 holds src as int16 when inInt16; clipOutput clips to 16 bits (inverse). RowDots::dot4( in16, row, rowStride, n )
// returns the dot products of in16 and the four rows starting at row, in row order.
template<class RowDots, bool inInt16, bool clipOutput>
static inline void lfnstRows( const int* src, const int16x8_t* in16, int* out, const int8_t* mat, const int rowStride,
                              const int n, const int numOut )
{
  for( int j = 0; j < numOut; j += 4, out += 4, mat += 4 * rowStride )
  {
    int32x4_t sum;
    if( inInt16 )
    {
      sum = RowDots::dot4( in16, mat, rowStride, n );
    }
    else
    {
      const int32x4_t dot0 = lfnstDot32( src, mat, n );
      const int32x4_t dot1 = lfnstDot32( src, mat + rowStride, n );
      const int32x4_t dot2 = lfnstDot32( src, mat + 2 * rowStride, n );
      const int32x4_t dot3 = lfnstDot32( src, mat + 3 * rowStride, n );
      sum = horizontal_add_4d_s32x4( dot0, dot1, dot2, dot3 );
    }
    int32x4_t res = vrshrq_n_s32( sum, 7 );
    if( clipOutput )
    {
      res = vminq_s32( vmaxq_s32( res, vdupq_n_s32( -( 1 << 15 ) ) ), vdupq_n_s32( ( 1 << 15 ) - 1 ) );
    }
    vst1q_s32( out, res );
  }
}

template<class RowDots, bool clipOutput>
static inline void lfnst_common( const int* src, int* out, const int8_t* mat, const int rowStride, const int n,
                                 const int numOut )
{
  if( lfnstInputFitsInt16( src, n ) )
  {
    int16x8_t in16[6] = {};
    lfnstNarrowInput( src, n, in16 );
    lfnstRows<RowDots, true, clipOutput>( src, in16, out, mat, rowStride, n, numOut );
  }
  else
  {
    lfnstRows<RowDots, false, clipOutput>( src, nullptr, out, mat, rowStride, n, numOut );
  }
}

template<class RowDots>
static inline void invLfnstNxN_common( int* src, int* dst, const uint32_t mode, const uint32_t index,
                                       const uint32_t size, int zeroOutSize )
{
  CHECKD( index > 2 || ( zeroOutSize != 8 && zeroOutSize != 16 ), "Wrong parameters" );

  const int8_t* mat    = size > 4 ? g_lfnstInv8x8[mode][index][0] : g_lfnstInv4x4[mode][index][0];
  const int     trSize = size > 4 ? 48 : 16;
  lfnst_common<RowDots, true>( src, dst, mat, 16, zeroOutSize, trSize );
}

template<class RowDots>
static inline void fwdLfnstNxN_common( int* src, int* dst, const uint32_t mode, const uint32_t index,
                                       const uint32_t size, int zeroOutSize )
{
  const int8_t* mat    = size > 4 ? g_lfnstFwd8x8[mode][index][0] : g_lfnstFwd4x4[mode][index][0];
  const int     trSize = size > 4 ? 48 : 16;
  lfnst_common<RowDots, false>( src, dst, mat, trSize, trSize, zeroOutSize );
  memset( dst + zeroOutSize, 0, ( trSize - zeroOutSize ) * sizeof( int ) );
}

#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_TRAFO

} // namespace vvenc

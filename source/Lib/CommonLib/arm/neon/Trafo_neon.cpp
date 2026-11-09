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
 * \file Trafo_neon.cpp
 * \brief Neon implementation of TCoeffOps for AArch64.
 */

//  ====================================================================================================================
//  Includes
//  ====================================================================================================================
#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "TrQuant.h"
#include "TrQuant_EMT.h"
#include "sum_neon.h"
#include "transpose_neon.h"

#include <type_traits>
#include <cstdint>
#include <cstring>

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_TRAFO

namespace vvenc
{

static inline int64_t shift_and_round( int64_t x, int shift )
{
  return ( x + ( static_cast<int64_t>(1) << ( shift - 1 ) ) ) >> shift;
}

template<int W>
void cpyCoeff_neon( const Pel* src, ptrdiff_t srcStride, TCoeff* dst, unsigned width, unsigned height );

template<>
void cpyCoeff_neon<4>( const Pel* src, ptrdiff_t srcStride, TCoeff* dst, unsigned width, unsigned height )
{
  CHECK( height < 1, "Height must be >= 1" );
  CHECK( width != 4, "Width must be 4" );

  do
  {
    int16x4_t s = vld1_s16( src );
    vst1q_s32( dst, vmovl_s16( s ) );

    src += srcStride;
    dst += width;
  } while( --height != 0 );
}

template<>
void cpyCoeff_neon<8>( const Pel* src, ptrdiff_t srcStride, TCoeff* dst, unsigned width, unsigned height )
{
  CHECK( height < 1, "Height must be >= 1" );
  CHECK( width < 8 || width & 7, "Width must be >= 8 and a multiple of 8" );

  do
  {
    unsigned w = 0;
    for( ; w + 16 <= width; w += 16 )
    {
      int16x8_t s_lo = vld1q_s16( src + w + 0 );
      int16x8_t s_hi = vld1q_s16( src + w + 8 );
      vst1q_s32( dst + w + 0, vmovl_s16( vget_low_s16( s_lo ) ) );
      vst1q_s32( dst + w + 4, vmovl_s16( vget_high_s16( s_lo ) ) );
      vst1q_s32( dst + w + 8, vmovl_s16( vget_low_s16( s_hi ) ) );
      vst1q_s32( dst + w + 12, vmovl_s16( vget_high_s16( s_hi ) ) );
    }
    if( width & 8 )
    {
      int16x8_t s = vld1q_s16( src + w );
      vst1q_s32( dst + w + 0, vmovl_s16( vget_low_s16( s ) ) );
      vst1q_s32( dst + w + 4, vmovl_s16( vget_high_s16( s ) ) );
    }

    src += srcStride;
    dst += width;
  } while( --height != 0 );
}

template<int W>
void roundClip_neon( TCoeff* dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin,
                     const TCoeff outputMax, const TCoeff round, const TCoeff shift );

template<>
void roundClip_neon<4>( TCoeff* dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin,
                        const TCoeff outputMax, const TCoeff round, const TCoeff shift )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width != 4, "Width must be 4" );
  CHECKD( round != 1 << ( shift - 1 ), "Round must be 1 << ( shift - 1 )" );
  CHECKD( outputMin != INT16_MIN, "OutputMin should be -32768" );
  CHECKD( outputMax != INT16_MAX, "OutputMax should be 32767" );

  // Current call to this function guarantees a minimum rshift of 7.
  // If dst max bit-depth is 23-bit, we do not need to do clipping to int16 range.
  CHECKD( shift < 7, "Shift must be >= 7" );

  do
  {
    int32x4_t d = vld1q_s32( dst );
    d = vrshlq_s32( d, vdupq_n_s32( -shift ) );
    vst1q_s32( dst, d );

    dst += stride;
  } while( --height != 0 );
}

template<>
void roundClip_neon<8>( TCoeff* dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin,
                        const TCoeff outputMax, const TCoeff round, const TCoeff shift )
{
  CHECKD( height < 1, "Height must be >= 1" );
  CHECKD( width < 8 || width & 7, "Width must be >= 8 and a multiple of 8" );
  CHECKD( round != 1 << ( shift - 1 ), "Round must be 1 << ( shift - 1 )" );
  CHECKD( outputMin != INT16_MIN, "OutputMin should be -32768" );
  CHECKD( outputMax != INT16_MAX, "OutputMax should be 32767" );

  // Current call to this function guarantees a minimum rshift of 7.
  // If dst max bit-depth is 23-bit, we do not need to do clipping to int16 range.
  CHECKD( shift < 7, "Shift must be >= 7" );

  do
  {
    unsigned w = 0;
    do
    {
      int32x4_t d_lo = vld1q_s32( dst + w + 0 );
      int32x4_t d_hi = vld1q_s32( dst + w + 4 );
      d_lo = vrshlq_s32( d_lo, vdupq_n_s32( -shift ) );
      d_hi = vrshlq_s32( d_hi, vdupq_n_s32( -shift ) );
      vst1q_s32( dst + w + 0, d_lo );
      vst1q_s32( dst + w + 4, d_hi );

      w += 8;
    } while( w != width );

    dst += stride;
  } while( --height != 0 );
}

template<unsigned trSize>
static void fastInvCore_neon( const TMatrixCoeff* it, const TCoeff* src, TCoeff* dst, unsigned lines,
                              unsigned reducedLines, unsigned rows )
{
  static_assert( trSize % 4 == 0, "trSize should be a multiple of four" );
  CHECK( rows % 4 != 0, "rows should be a multiple of four" );
  CHECK( rows == 0, "rows should be non-zero" );

  memset( dst, 0, reducedLines * trSize * sizeof( TCoeff ) );

  unsigned i = 0;
  for( ; i != ( reducedLines & ~3U ); i += 4 )
  {
    unsigned j = 0;
    do
    {
      const TCoeff* srci      = src + i;
      const TMatrixCoeff* itj = it + j;
      TCoeff* dstij           = dst + i * trSize + j;

      int32x4_t d0 = vld1q_s32( dstij + 0 * trSize );
      int32x4_t d1 = vld1q_s32( dstij + 1 * trSize );
      int32x4_t d2 = vld1q_s32( dstij + 2 * trSize );
      int32x4_t d3 = vld1q_s32( dstij + 3 * trSize );

      unsigned k = rows;
      do
      {
        int16x4_t s = vmovn_s32( vld1q_s32( srci ) );
        int16x4_t c = vld1_s16( itj );

        d0 = vmlal_lane_s16( d0, c, s, 0 );
        d1 = vmlal_lane_s16( d1, c, s, 1 );
        d2 = vmlal_lane_s16( d2, c, s, 2 );
        d3 = vmlal_lane_s16( d3, c, s, 3 );

        srci += lines;
        itj += trSize;
      } while( --k != 0 );

      vst1q_s32( dstij + 0 * trSize, d0 );
      vst1q_s32( dstij + 1 * trSize, d1 );
      vst1q_s32( dstij + 2 * trSize, d2 );
      vst1q_s32( dstij + 3 * trSize, d3 );

      j += 4;
    } while( j != trSize );
  }

  for( ; i != reducedLines; ++i )
  {
    unsigned j = 0;
    do
    {
      const TCoeff* srci      = src + i;
      const TMatrixCoeff* itj = it + j;
      TCoeff* dstij           = dst + i * trSize + j;

      int32_t d0 = *dstij;
      unsigned k = rows;
      do
      {
        d0 += *srci * *itj;

        srci += lines;
        itj += trSize;
      } while( --k != 0 );

      *dstij = d0;
    } while( ++j != trSize );
  }
}

template<unsigned trSize>
static void fastFwdCore_neon( const TMatrixCoeff* tc, const TCoeff* src, TCoeff* dst, unsigned line,
                              unsigned reducedLine, unsigned cutoff, int shift )
{
  static_assert( trSize % 4 == 0, "trSize should be a multiple of four" );
  CHECK( cutoff % 4 != 0, "cutoff should be a multiple of four" );
  CHECK( cutoff == 0, "cutoff should be non-zero" );
  CHECK( shift == 0, "shift must be at least one" );

  unsigned i = 0;
  for( ; i < ( reducedLine & ~3U ); i += 4 )
  {
    unsigned j = 0;
    do
    {
      const TMatrixCoeff* tcj = tc + j * trSize;
      const TCoeff* srci      = src + i * trSize;

      int32x4_t sum00 = vdupq_n_s32( 0 );
      int32x4_t sum01 = vdupq_n_s32( 0 );
      int32x4_t sum02 = vdupq_n_s32( 0 );
      int32x4_t sum03 = vdupq_n_s32( 0 );
      int32x4_t sum10 = vdupq_n_s32( 0 );
      int32x4_t sum11 = vdupq_n_s32( 0 );
      int32x4_t sum12 = vdupq_n_s32( 0 );
      int32x4_t sum13 = vdupq_n_s32( 0 );
      int32x4_t sum20 = vdupq_n_s32( 0 );
      int32x4_t sum21 = vdupq_n_s32( 0 );
      int32x4_t sum22 = vdupq_n_s32( 0 );
      int32x4_t sum23 = vdupq_n_s32( 0 );
      int32x4_t sum30 = vdupq_n_s32( 0 );
      int32x4_t sum31 = vdupq_n_s32( 0 );
      int32x4_t sum32 = vdupq_n_s32( 0 );
      int32x4_t sum33 = vdupq_n_s32( 0 );
      for( unsigned k = 0; k < trSize; k += 4 )
      {
        int16x4_t s0 = vmovn_s32( vld1q_s32( srci + 0 * trSize ) );
        int16x4_t s1 = vmovn_s32( vld1q_s32( srci + 1 * trSize ) );
        int16x4_t s2 = vmovn_s32( vld1q_s32( srci + 2 * trSize ) );
        int16x4_t s3 = vmovn_s32( vld1q_s32( srci + 3 * trSize ) );
        int16x4_t c0 = vld1_s16( tcj + 0 * trSize );
        int16x4_t c1 = vld1_s16( tcj + 1 * trSize );
        int16x4_t c2 = vld1_s16( tcj + 2 * trSize );
        int16x4_t c3 = vld1_s16( tcj + 3 * trSize );

        sum00 = vmlal_s16( sum00, s0, c0 );
        sum01 = vmlal_s16( sum01, s0, c1 );
        sum02 = vmlal_s16( sum02, s0, c2 );
        sum03 = vmlal_s16( sum03, s0, c3 );
        sum10 = vmlal_s16( sum10, s1, c0 );
        sum11 = vmlal_s16( sum11, s1, c1 );
        sum12 = vmlal_s16( sum12, s1, c2 );
        sum13 = vmlal_s16( sum13, s1, c3 );
        sum20 = vmlal_s16( sum20, s2, c0 );
        sum21 = vmlal_s16( sum21, s2, c1 );
        sum22 = vmlal_s16( sum22, s2, c2 );
        sum23 = vmlal_s16( sum23, s2, c3 );
        sum30 = vmlal_s16( sum30, s3, c0 );
        sum31 = vmlal_s16( sum31, s3, c1 );
        sum32 = vmlal_s16( sum32, s3, c2 );
        sum33 = vmlal_s16( sum33, s3, c3 );

        srci += 4;
        tcj += 4;
      }
      int32x4_t sum0 = horizontal_add_4d_s32x4( sum00, sum10, sum20, sum30 );
      int32x4_t sum1 = horizontal_add_4d_s32x4( sum01, sum11, sum21, sum31 );
      int32x4_t sum2 = horizontal_add_4d_s32x4( sum02, sum12, sum22, sum32 );
      int32x4_t sum3 = horizontal_add_4d_s32x4( sum03, sum13, sum23, sum33 );

      sum0 = vrshlq_s32( sum0, vdupq_n_s32( -shift ) );
      sum1 = vrshlq_s32( sum1, vdupq_n_s32( -shift ) );
      sum2 = vrshlq_s32( sum2, vdupq_n_s32( -shift ) );
      sum3 = vrshlq_s32( sum3, vdupq_n_s32( -shift ) );

      TCoeff* dstij = dst + j * line + i;
      vst1q_s32( dstij + 0 * line, sum0 );
      vst1q_s32( dstij + 1 * line, sum1 );
      vst1q_s32( dstij + 2 * line, sum2 );
      vst1q_s32( dstij + 3 * line, sum3 );

      j += 4;
    } while( j != cutoff );
  }
  for( ; i < reducedLine; ++i )
  {
    int j = 0;
    do
    {
      const TMatrixCoeff* tcj = tc + j * trSize;
      const TCoeff* srci      = src + i * trSize;

      int32x4_t sum00 = vdupq_n_s32( 0 );
      int32x4_t sum01 = vdupq_n_s32( 0 );
      int32x4_t sum02 = vdupq_n_s32( 0 );
      int32x4_t sum03 = vdupq_n_s32( 0 );
      for( int k = 0; k < trSize; k += 4 )
      {
        int16x4_t s0 = vmovn_s32( vld1q_s32( srci ) );
        int16x4_t c0 = vld1_s16( tcj + 0 * trSize );
        int16x4_t c1 = vld1_s16( tcj + 1 * trSize );
        int16x4_t c2 = vld1_s16( tcj + 2 * trSize );
        int16x4_t c3 = vld1_s16( tcj + 3 * trSize );

        sum00 = vmlal_s16( sum00, s0, c0 );
        sum01 = vmlal_s16( sum01, s0, c1 );
        sum02 = vmlal_s16( sum02, s0, c2 );
        sum03 = vmlal_s16( sum03, s0, c3 );

        srci += 4;
        tcj += 4;
      }
      TCoeff* dstij = dst + j * line + i;

      dstij[ 0 * line ] = (TCoeff) shift_and_round( horizontal_add_s32x4( sum00 ), shift );
      dstij[ 1 * line ] = (TCoeff) shift_and_round( horizontal_add_s32x4( sum01 ), shift );
      dstij[ 2 * line ] = (TCoeff) shift_and_round( horizontal_add_s32x4( sum02 ), shift );
      dstij[ 3 * line ] = (TCoeff) shift_and_round( horizontal_add_s32x4( sum03 ), shift );

      j += 4;
    } while( j != cutoff );
  }
}

// rows (row0, row0 + 2, ..) of the forward matrix, numRows of them (2 or 4), times the 4 lines' E (even rows) or O
// (odd rows), lanes = lines: dst row j, columns i .. i + 3 of the 4 lines
template<int N, int numRows>
static inline void fwdEvenOddRows_neon( const TMatrixCoeff* tc, int row0, const int16x4_t* even, const int16x4_t* odd,
                                        TCoeff* dst, int line, const int32x4_t vshift )
{
  int32x4_t accE[numRows], accO[numRows];
  for( int q = 0; q < numRows; q++ )
  {
    accE[q] = vdupq_n_s32( 0 );
    accO[q] = vdupq_n_s32( 0 );
  }
  const TMatrixCoeff* te = tc + row0 * N;
  const TMatrixCoeff* to = tc + ( row0 + 1 ) * N;
  for( int k = 0; k < N / 2; k += 4 )
  {
    for( int q = 0; q < numRows; q++ )
    {
      const int16x4_t ce = vld1_s16( te + 2 * q * N + k );
      const int16x4_t co = vld1_s16( to + 2 * q * N + k );
      accE[q] = vmlal_lane_s16( accE[q], even[k + 0], ce, 0 );
      accO[q] = vmlal_lane_s16( accO[q], odd[k + 0], co, 0 );
      accE[q] = vmlal_lane_s16( accE[q], even[k + 1], ce, 1 );
      accO[q] = vmlal_lane_s16( accO[q], odd[k + 1], co, 1 );
      accE[q] = vmlal_lane_s16( accE[q], even[k + 2], ce, 2 );
      accO[q] = vmlal_lane_s16( accO[q], odd[k + 2], co, 2 );
      accE[q] = vmlal_lane_s16( accE[q], even[k + 3], ce, 3 );
      accO[q] = vmlal_lane_s16( accO[q], odd[k + 3], co, 3 );
    }
  }
  for( int q = 0; q < numRows; q++ )
  {
    vst1q_s32( dst + ( row0 + 2 * q ) * line, vrshlq_s32( accE[q], vshift ) );
    vst1q_s32( dst + ( row0 + 2 * q + 1 ) * line, vrshlq_s32( accO[q], vshift ) );
  }
}

// The second even/odd level for N = 32 (at N = 16 its range checks cost what it saves): the even rows are those of a
// 16-point DCT-II of E and symmetric again, T[j][N/2 - 1 - k] = (-1)^(j/2) T[j][k] for even j, so rows j = 0 mod 4 take
// EE[k] = E[k] + E[N/2 - 1 - k] and rows j = 2 mod 4 take EO[k] = E[k] - E[N/2 - 1 - k], k < N/4: 3/8 instead of 1/2 of
// the matrix multiply-adds. Taken only when all x of the 4 lines lie in [-8192, 8191] (x + 8192 has bits 14 and 15
// clear in every lane of range8192): then E lies in [-16384, 16382] and EE, EO in [-32768, 32764], so they fit int16.
// Returns false otherwise.
template<int N>
static inline bool fwdTwoLevelLines_neon( std::false_type, const TMatrixCoeff*, const int16x4_t*, const int16x4_t*, const int16x8_t, int, TCoeff*,
                                          int, const int32x4_t )
{
  return false;
}

template<int N>
static inline bool fwdTwoLevelLines_neon( std::true_type, const TMatrixCoeff* tc, const int16x4_t* even, const int16x4_t* odd,
                                          const int16x8_t range8192, int cutoff, TCoeff* dst, int line, const int32x4_t vshift )
{
  const uint16x4_t range = vreinterpret_u16_s16( vorr_s16( vget_low_s16( range8192 ), vget_high_s16( range8192 ) ) );
  if( vget_lane_u64( vreinterpret_u64_u16( vshr_n_u16( range, 14 ) ), 0 ) != 0 )
  {
    return false;
  }
  int16x4_t ee[N / 4], eo[N / 4];
  for( int k = 0; k < N / 4; k++ )
  {
    ee[k] = vadd_s16( even[k], even[N / 2 - 1 - k] );
    eo[k] = vsub_s16( even[k], even[N / 2 - 1 - k] );
  }
  // rows j (EE), j + 1 (O), j + 2 (EO), j + 3 (O); cutoff is a multiple of 4
  for( int j = 0; j < cutoff; j += 4 )
  {
    const TMatrixCoeff* t = tc + j * N;
    int32x4_t acc0 = vdupq_n_s32( 0 ), acc1 = vdupq_n_s32( 0 ), acc2 = vdupq_n_s32( 0 ), acc3 = vdupq_n_s32( 0 );
    for( int k = 0; k < N / 4; k += 4 )
    {
      const int16x4_t c0 = vld1_s16( t + k );
      const int16x4_t c2 = vld1_s16( t + 2 * N + k );
      acc0 = vmlal_lane_s16( acc0, ee[k + 0], c0, 0 );
      acc2 = vmlal_lane_s16( acc2, eo[k + 0], c2, 0 );
      acc0 = vmlal_lane_s16( acc0, ee[k + 1], c0, 1 );
      acc2 = vmlal_lane_s16( acc2, eo[k + 1], c2, 1 );
      acc0 = vmlal_lane_s16( acc0, ee[k + 2], c0, 2 );
      acc2 = vmlal_lane_s16( acc2, eo[k + 2], c2, 2 );
      acc0 = vmlal_lane_s16( acc0, ee[k + 3], c0, 3 );
      acc2 = vmlal_lane_s16( acc2, eo[k + 3], c2, 3 );
    }
    for( int k = 0; k < N / 2; k += 4 )
    {
      const int16x4_t c1 = vld1_s16( t + N + k );
      const int16x4_t c3 = vld1_s16( t + 3 * N + k );
      acc1 = vmlal_lane_s16( acc1, odd[k + 0], c1, 0 );
      acc3 = vmlal_lane_s16( acc3, odd[k + 0], c3, 0 );
      acc1 = vmlal_lane_s16( acc1, odd[k + 1], c1, 1 );
      acc3 = vmlal_lane_s16( acc3, odd[k + 1], c3, 1 );
      acc1 = vmlal_lane_s16( acc1, odd[k + 2], c1, 2 );
      acc3 = vmlal_lane_s16( acc3, odd[k + 2], c3, 2 );
      acc1 = vmlal_lane_s16( acc1, odd[k + 3], c1, 3 );
      acc3 = vmlal_lane_s16( acc3, odd[k + 3], c3, 3 );
    }
    vst1q_s32( dst + ( j + 0 ) * line, vrshlq_s32( acc0, vshift ) );
    vst1q_s32( dst + ( j + 1 ) * line, vrshlq_s32( acc1, vshift ) );
    vst1q_s32( dst + ( j + 2 ) * line, vrshlq_s32( acc2, vshift ) );
    vst1q_s32( dst + ( j + 3 ) * line, vrshlq_s32( acc3, vshift ) );
  }
  return true;
}

// Forward DCT-II, N = 8, 16, 32 or 64 (TCoeffOps::fastFwdDCT2[5], [2..4]), with the even/odd split of the AVX2
// fastForwardDCT2_EvenOdd_SIMD: the matrix rows are symmetric, T[j][N - 1 - k] = (-1)^j T[j][k], so output j is row
// j's first half times E[k] = x[k] + x[N - 1 - k] (j even) or O[k] = x[k] - x[N - 1 - k] (j odd), half the
// multiply-adds of the matrix multiply. Exact with fastFwdCore_neon, which the C default (fastForwardDCT2_MM) runs:
// the same x = the inputs truncated to int16 (vmovn), the same int32 sums (no overflow with |T| <= 91) and the same
// rounding shift. Groups of 4 lines whose x all lie in [-16384, 16383] (E and O fit int16) take the split, with the
// lines in the lanes (no horizontal adds); other groups run fastFwdCore_neon. Fewer than 4 lines and a cut-off of 4
// take fastForwardDCT2_MM.
template<int N>
static NO_INLINE void fwdEvenOddLines_neon( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2 )
{
  const int reducedLine = line - iSkipLine;
  const int cutoff      = N - iSkipLine2;
  CHECKD( cutoff % 4 != 0 || cutoff == 0, "cutoff should be a non-zero multiple of four" );

  const TMatrixCoeff* tc = N == 8  ? g_trCoreDCT2P8[TRANSFORM_FORWARD][0]
                         : N == 16 ? g_trCoreDCT2P16[TRANSFORM_FORWARD][0]
                         : N == 32 ? g_trCoreDCT2P32[TRANSFORM_FORWARD][0]
                         :           g_trCoreDCT2P64[TRANSFORM_FORWARD][0];
  const int32x4_t vshift = vdupq_n_s32( -shift );
  const int       half   = cutoff >> 1;   // output rows of each parity, even since cutoff is a multiple of 4

  int i = 0;
  for( ; i + 4 <= reducedLine; i += 4, src += 4 * N )
  {
    int16x4_t even[N / 2], odd[N / 2];   // lanes = lines i .. i + 3
    // x + 16384 wraps to a negative int16 exactly when x lies outside [-16384, 16383]
    int16x8_t range = vdupq_n_s16( 0 );
    int16x8_t range8192 = vdupq_n_s16( 0 );   // the same with x + 8192 for the second even/odd level
    for( int k = 0; k < N / 2; k += 4 )
    {
      // per line x[k .. k + 3] and x[N - 4 - k .. N - 1 - k], truncated to int16 as in fastFwdCore_neon
      int16x8_t x[4];
      for( int l = 0; l < 4; l++ )
      {
        const int32x4_t first  = vld1q_s32( src + l * N + k );
        const int32x4_t mirror = vld1q_s32( src + l * N + N - 4 - k );
        x[l]  = vcombine_s16( vmovn_s32( first ), vmovn_s32( mirror ) );
        range = vorrq_s16( range, vaddq_s16( x[l], vdupq_n_s16( 16384 ) ) );
        if( N == 32 )
        {
          range8192 = vorrq_s16( range8192, vaddq_s16( x[l], vdupq_n_s16( 8192 ) ) );
        }
      }
      // t[0]: x[k], x[k + 1] of the 4 lines; t[1]: x[k + 2], x[k + 3]; t[2]: x[N - 4 - k], x[N - 3 - k];
      // t[3]: x[N - 2 - k], x[N - 1 - k]
      int16x8_t t[4];
      transpose_concat_8x4_s16( x[0], x[1], x[2], x[3], t[0], t[1], t[2], t[3] );
      // the halves of t[3] and t[2] swapped: x[N - 1 - k], x[N - 2 - k] and x[N - 3 - k], x[N - 4 - k]
      const int16x8_t mirror01 = vextq_s16( t[3], t[3], 4 );
      const int16x8_t mirror23 = vextq_s16( t[2], t[2], 4 );
      const int16x8_t even01   = vaddq_s16( t[0], mirror01 );
      const int16x8_t odd01    = vsubq_s16( t[0], mirror01 );
      const int16x8_t even23   = vaddq_s16( t[1], mirror23 );
      const int16x8_t odd23    = vsubq_s16( t[1], mirror23 );
      even[k + 0] = vget_low_s16( even01 );
      even[k + 1] = vget_high_s16( even01 );
      even[k + 2] = vget_low_s16( even23 );
      even[k + 3] = vget_high_s16( even23 );
      odd[k + 0]  = vget_low_s16( odd01 );
      odd[k + 1]  = vget_high_s16( odd01 );
      odd[k + 2]  = vget_low_s16( odd23 );
      odd[k + 3]  = vget_high_s16( odd23 );
    }
    const int16x4_t rangeAll = vorr_s16( vget_low_s16( range ), vget_high_s16( range ) );
    if( vget_lane_s64( vreinterpret_s64_s16( vshr_n_s16( rangeAll, 15 ) ), 0 ) != 0 )
    {
      fastFwdCore_neon<N>( tc, src, dst + i, line, 4, cutoff, shift );
      continue;
    }
    if( fwdTwoLevelLines_neon<N>( std::integral_constant<bool, N == 32>(), tc, even, odd, range8192, cutoff, dst + i, line, vshift ) )
    {
      continue;
    }
    // 4 even and 4 odd rows at a time, then 2 and 2 if cutoff / 2 is not a multiple of 4
    int r = 0;
    for( ; r + 4 <= half; r += 4 )
    {
      fwdEvenOddRows_neon<N, 4>( tc, 2 * r, even, odd, dst + i, line, vshift );
    }
    if( r < half )
    {
      fwdEvenOddRows_neon<N, 2>( tc, 2 * r, even, odd, dst + i, line, vshift );
    }
  }
  if( i < reducedLine )
  {
    fastFwdCore_neon<N>( tc, src, dst + i, line, reducedLine - i, cutoff, shift );
  }

  fastForwardZeroSkipped( dst, line, reducedLine, cutoff, iSkipLine );
}

template<int N>
static void fastForwardDCT2_EvenOdd_neon( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2 )
{
  const int reducedLine = line - iSkipLine;
  const int cutoff      = N - iSkipLine2;

  if( reducedLine < 4 || cutoff <= 4 )
  {
    fastForwardDCT2_MM<N>( src, dst, shift, line, iSkipLine, iSkipLine2 );
  }
  else
  {
    fwdEvenOddLines_neon<N>( src, dst, shift, line, iSkipLine, iSkipLine2 );
  }
}

template<>
void TCoeffOps::_initTCoeffOpsARM<NEON>()
{
  cpyCoeff4  = cpyCoeff_neon<4>;
  cpyCoeff8  = cpyCoeff_neon<8>;
  roundClip4 = roundClip_neon<4>;
  roundClip8 = roundClip_neon<8>;

  fastInvCore[ 0 ]    = fastInvCore_neon<4>;
  fastInvCore[ 1 ]    = fastInvCore_neon<8>;
  fastInvCore[ 2 ]    = fastInvCore_neon<16>;
  fastInvCore[ 3 ]    = fastInvCore_neon<32>;
  fastInvCore[ 4 ]    = fastInvCore_neon<64>;
  fastFwdCore_2D[ 0 ] = fastFwdCore_neon<4>;
  fastFwdCore_2D[ 1 ] = fastFwdCore_neon<8>;
  fastFwdCore_2D[ 2 ] = fastFwdCore_neon<16>;
  fastFwdCore_2D[ 3 ] = fastFwdCore_neon<32>;
  fastFwdCore_2D[ 4 ] = fastFwdCore_neon<64>;

  fastFwdDCT2[ 2 ] = fastForwardDCT2_EvenOdd_neon<16>;
  fastFwdDCT2[ 3 ] = fastForwardDCT2_EvenOdd_neon<32>;
  fastFwdDCT2[ 4 ] = fastForwardDCT2_EvenOdd_neon<64>;
  fastFwdDCT2[ 5 ] = fastForwardDCT2_EvenOdd_neon<8>;
}

}  // namespace vvenc

#endif
//! \}

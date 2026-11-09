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
#include "reverse_neon.h"
#include "sum_neon.h"

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

// Stores one output line's columns j .. j + 3 ( E + O ) and N - 4 - j .. N - 1 - j ( E - O, in reverse order ) of the
// inverse DCT-II below, each with the rounding shift.
template<int N>
static inline void storeInvEvenOdd_neon( TCoeff* dstLine, int j, int32x4_t e, int32x4_t o, int32x4_t negShift )
{
  const int32x4_t first = vrshlq_s32( vaddq_s32( e, o ), negShift );
  const int32x4_t last  = vrshlq_s32( vsubq_s32( e, o ), negShift );
  vst1q_s32( dstLine + j, first );
  vst1q_s32( dstLine + N - 4 - j, reverse_vector_s32x4( last ) );
}

// Stores one output line's columns of the four column pairs jLow, jHigh = N / 2 - 4 - jLow of the inverse DCT-II
// below (columns jLow, jHigh, N - 4 - jHigh and N - 4 - jLow, four each): E is EE + EO at jLow and EE - EO, in reverse
// order, at jHigh.
template<int N>
static inline void storeInvEvenEvenOdd_neon( TCoeff* dstLine, int jLow, int jHigh, int32x4_t ee, int32x4_t eo,
                                             int32x4_t oLow, int32x4_t oHigh, int32x4_t negShift )
{
  const int32x4_t eHighReversed = vsubq_s32( ee, eo );   // E at columns jHigh + 3 .. jHigh
  storeInvEvenOdd_neon<N>( dstLine, jLow, vaddq_s32( ee, eo ), oLow, negShift );
  storeInvEvenOdd_neon<N>( dstLine, jHigh, reverse_vector_s32x4( eHighReversed ), oHigh, negShift );
}

// Inverse DCT-II, N = 16, 32 or 64 (TCoeffOps::fastInvDCT2[3..5]): fastInvCore_neon's matrix multiply followed by
// roundClip_neon<8>'s rounding shift, with fewer multiply-adds. The DCT-II basis is symmetric,
// T[k][N - 1 - j] = (-1)^k T[k][j], so for j < N / 2 output j is E + O and output N - 1 - j is E - O, where E sums the
// products of the even rows k and O those of the odd rows (half the multiply-adds; E splits once more below). Exact:
// the same inputs truncated to int16 (vmovn), the same int32 products and their sum in another order (int32 additions
// wrap alike), the same rounding shift. Like roundClip_neon<8> it does not clip: the output range is int16 and the
// shift at least 7 (its CHECKDs).
template<int N>
static void fastInverseDCT2_EvenOdd_neon( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine,
                                          int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
  CHECKD( outputMinimum != INT16_MIN || outputMaximum != INT16_MAX, "Output range must be int16" );
  CHECKD( shift < 7, "Shift must be >= 7" );

  const TMatrixCoeff* it = N == 16 ? g_trCoreDCT2P16[TRANSFORM_INVERSE][0]
                         : N == 32 ? g_trCoreDCT2P32[TRANSFORM_INVERSE][0]
                         :           g_trCoreDCT2P64[TRANSFORM_INVERSE][0];
  const int reducedLine  = line - iSkipLine;
  const int cutoff       = N - iSkipLine2;
  CHECKD( cutoff <= 0 || ( cutoff & 3 ), "Cut-off must be a positive multiple of four" );
  const int32x4_t negShift = vdupq_n_s32( -shift );

  // Four lines at a time, one accumulator per line, lanes = four output columns. A second level splits E: the even rows
  // restricted to the columns j < N / 2 are the DCT-II of size N / 2, so for j < N / 4, E at column j is EE + EO and E
  // at column N / 2 - 1 - j is EE - EO, where EE sums the products of the rows k = 0 mod 4 and EO those of the rows
  // k = 2 mod 4. Each step takes the four columns jLow and the four columns jHigh = N / 2 - 4 - jLow.
  int i = 0;
  for( ; i + 4 <= reducedLine; i += 4 )
  {
    TCoeff* dstLine = dst + i * N;
    for( int jLow = 0; jLow < N / 4; jLow += 4 )
    {
      const int jHigh = N / 2 - 4 - jLow;
      int32x4_t ee0 = vdupq_n_s32( 0 );
      int32x4_t ee1 = vdupq_n_s32( 0 );
      int32x4_t ee2 = vdupq_n_s32( 0 );
      int32x4_t ee3 = vdupq_n_s32( 0 );
      int32x4_t eo0 = vdupq_n_s32( 0 );
      int32x4_t eo1 = vdupq_n_s32( 0 );
      int32x4_t eo2 = vdupq_n_s32( 0 );
      int32x4_t eo3 = vdupq_n_s32( 0 );
      int32x4_t oLow0 = vdupq_n_s32( 0 );
      int32x4_t oLow1 = vdupq_n_s32( 0 );
      int32x4_t oLow2 = vdupq_n_s32( 0 );
      int32x4_t oLow3 = vdupq_n_s32( 0 );
      int32x4_t oHigh0 = vdupq_n_s32( 0 );
      int32x4_t oHigh1 = vdupq_n_s32( 0 );
      int32x4_t oHigh2 = vdupq_n_s32( 0 );
      int32x4_t oHigh3 = vdupq_n_s32( 0 );

      const TCoeff*       srck = src + i;
      const TMatrixCoeff* itk  = it;
      for( int k = 0; k < cutoff; k += 4 )
      {
        const int32x4_t src0    = vld1q_s32( srck );
        const int32x4_t src1    = vld1q_s32( srck + line );
        const int32x4_t src2    = vld1q_s32( srck + 2 * line );
        const int32x4_t src3    = vld1q_s32( srck + 3 * line );
        const int16x4_t s0      = vmovn_s32( src0 );
        const int16x4_t s1      = vmovn_s32( src1 );
        const int16x4_t s2      = vmovn_s32( src2 );
        const int16x4_t s3      = vmovn_s32( src3 );
        const int16x4_t cEE     = vld1_s16( itk + jLow );
        const int16x4_t cO1Low  = vld1_s16( itk + N + jLow );
        const int16x4_t cO1High = vld1_s16( itk + N + jHigh );
        const int16x4_t cEO     = vld1_s16( itk + 2 * N + jLow );
        const int16x4_t cO3Low  = vld1_s16( itk + 3 * N + jLow );
        const int16x4_t cO3High = vld1_s16( itk + 3 * N + jHigh );

        ee0    = vmlal_lane_s16( ee0, cEE, s0, 0 );
        eo0    = vmlal_lane_s16( eo0, cEO, s2, 0 );
        oLow0  = vmlal_lane_s16( oLow0, cO1Low, s1, 0 );
        oLow0  = vmlal_lane_s16( oLow0, cO3Low, s3, 0 );
        oHigh0 = vmlal_lane_s16( oHigh0, cO1High, s1, 0 );
        oHigh0 = vmlal_lane_s16( oHigh0, cO3High, s3, 0 );

        ee1    = vmlal_lane_s16( ee1, cEE, s0, 1 );
        eo1    = vmlal_lane_s16( eo1, cEO, s2, 1 );
        oLow1  = vmlal_lane_s16( oLow1, cO1Low, s1, 1 );
        oLow1  = vmlal_lane_s16( oLow1, cO3Low, s3, 1 );
        oHigh1 = vmlal_lane_s16( oHigh1, cO1High, s1, 1 );
        oHigh1 = vmlal_lane_s16( oHigh1, cO3High, s3, 1 );

        ee2    = vmlal_lane_s16( ee2, cEE, s0, 2 );
        eo2    = vmlal_lane_s16( eo2, cEO, s2, 2 );
        oLow2  = vmlal_lane_s16( oLow2, cO1Low, s1, 2 );
        oLow2  = vmlal_lane_s16( oLow2, cO3Low, s3, 2 );
        oHigh2 = vmlal_lane_s16( oHigh2, cO1High, s1, 2 );
        oHigh2 = vmlal_lane_s16( oHigh2, cO3High, s3, 2 );

        ee3    = vmlal_lane_s16( ee3, cEE, s0, 3 );
        eo3    = vmlal_lane_s16( eo3, cEO, s2, 3 );
        oLow3  = vmlal_lane_s16( oLow3, cO1Low, s1, 3 );
        oLow3  = vmlal_lane_s16( oLow3, cO3Low, s3, 3 );
        oHigh3 = vmlal_lane_s16( oHigh3, cO1High, s1, 3 );
        oHigh3 = vmlal_lane_s16( oHigh3, cO3High, s3, 3 );

        srck += 4 * line;
        itk  += 4 * N;
      }

      storeInvEvenEvenOdd_neon<N>( dstLine + 0 * N, jLow, jHigh, ee0, eo0, oLow0, oHigh0, negShift );
      storeInvEvenEvenOdd_neon<N>( dstLine + 1 * N, jLow, jHigh, ee1, eo1, oLow1, oHigh1, negShift );
      storeInvEvenEvenOdd_neon<N>( dstLine + 2 * N, jLow, jHigh, ee2, eo2, oLow2, oHigh2, negShift );
      storeInvEvenEvenOdd_neon<N>( dstLine + 3 * N, jLow, jHigh, ee3, eo3, oLow3, oHigh3, negShift );
    }
  }

  // the remaining lines one at a time
  for( ; i < reducedLine; i++ )
  {
    for( int j = 0; j < N / 2; j += 4 )
    {
      int32x4_t e = vdupq_n_s32( 0 );
      int32x4_t o = vdupq_n_s32( 0 );

      const TCoeff*       srck = src + i;
      const TMatrixCoeff* itk  = it + j;
      for( int k = 0; k < cutoff; k += 2 )
      {
        const int16x4_t cEven = vld1_s16( itk );
        const int16x4_t cOdd  = vld1_s16( itk + N );

        const int16_t   sEven = static_cast<int16_t>( srck[0] );
        const int16_t   sOdd  = static_cast<int16_t>( srck[line] );

        e = vmlal_n_s16( e, cEven, sEven );
        o = vmlal_n_s16( o, cOdd, sOdd );

        srck += 2 * line;
        itk  += 2 * N;
      }

      storeInvEvenOdd_neon<N>( dst + i * N, j, e, o, negShift );
    }
  }

  if( iSkipLine )
  {
    memset( dst + reducedLine * N, 0, iSkipLine * N * sizeof( TCoeff ) );
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
  fastInvDCT2[ 3 ]    = fastInverseDCT2_EvenOdd_neon<16>;
  fastInvDCT2[ 4 ]    = fastInverseDCT2_EvenOdd_neon<32>;
  fastInvDCT2[ 5 ]    = fastInverseDCT2_EvenOdd_neon<64>;
  fastFwdCore_2D[ 0 ] = fastFwdCore_neon<4>;
  fastFwdCore_2D[ 1 ] = fastFwdCore_neon<8>;
  fastFwdCore_2D[ 2 ] = fastFwdCore_neon<16>;
  fastFwdCore_2D[ 3 ] = fastFwdCore_neon<32>;
  fastFwdCore_2D[ 4 ] = fastFwdCore_neon<64>;
}

}  // namespace vvenc

#endif
//! \}

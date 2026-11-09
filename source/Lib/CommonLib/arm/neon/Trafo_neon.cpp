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
#include "../Trafo_neon.h"
#include "TrQuant.h"
#include "TrQuant_EMT.h"
#include "reverse_neon.h"
#include "sum_neon.h"
#include "transpose_neon.h"

#include <type_traits>
#include <cstdint>

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

// cpyResi: the residual (rows contiguous in src) narrowed to Pel with signed saturation, as the x86 kernels'
// _mm_packs_epi32; C's truncation gives the same in the inverse transform's 16-bit output range.
template<int W>
void cpyResi_neon( const TCoeff* src, Pel* dst, ptrdiff_t stride, unsigned width, unsigned height );

template<>
void cpyResi_neon<4>( const TCoeff* src, Pel* dst, ptrdiff_t stride, unsigned width, unsigned height )
{
  // the callers take cpyResi8 for widths that are multiples of 8, so with power-of-two widths only 4 is left
  CHECKD( width != 4, "Width must be 4" );

  for( ; height > 0; height--, src += 4, dst += stride )
  {
    const int32x4_t s = vld1q_s32( src );
    vst1_s16( dst, vqmovn_s32( s ) );
  }
}

template<>
void cpyResi_neon<8>( const TCoeff* src, Pel* dst, ptrdiff_t stride, unsigned width, unsigned height )
{
  for( ; height > 0; height--, src += width, dst += stride )
  {
    unsigned x = 0;
    for( ; x + 16 <= width; x += 16 )
    {
      const int32x4_t s0 = vld1q_s32( src + x );
      const int32x4_t s1 = vld1q_s32( src + x + 4 );
      const int32x4_t s2 = vld1q_s32( src + x + 8 );
      const int32x4_t s3 = vld1q_s32( src + x + 12 );
      vst1q_s16( dst + x, vcombine_s16( vqmovn_s32( s0 ), vqmovn_s32( s1 ) ) );
      vst1q_s16( dst + x + 8, vcombine_s16( vqmovn_s32( s2 ), vqmovn_s32( s3 ) ) );
    }
    if( width & 8 )
    {
      const int32x4_t s0 = vld1q_s32( src + x );
      const int32x4_t s1 = vld1q_s32( src + x + 4 );
      vst1q_s16( dst + x, vcombine_s16( vqmovn_s32( s0 ), vqmovn_s32( s1 ) ) );
    }
  }
}

template<int W>
void roundClip_neon( TCoeff* dst, unsigned width, unsigned height, unsigned stride, const TCoeff outputMin,
                     const TCoeff outputMax, const TCoeff round, const TCoeff shift );

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

// Inverse DCT-II of 8 points (TCoeffOps::fastInvDCT2[2]), as fastInverseDCT2_B8_Core: 4 lines at a time, one per
// lane. The sums are C's in 32 bits: the inputs are 16-bit coefficients and the matrix entries at most 89, so they stay
// below 2^25 in magnitude; the even part's entries 64 and -64 are shifts by 6. C's ( d + add ) >> shift is a rounding
// shift (it cannot overflow either), then the clip to the output range.

// lines j..j+3: s[r] = src[r * line + j..j+3], out[8 * i + r] = result r of line j + i
static inline void invDCT2_B8_4lines_neon( const TMatrixCoeff* iT, const TCoeff* src, int line, TCoeff* out,
                                           const int32x4_t shift, const int32x4_t vmin, const int32x4_t vmax )
{
  int32x4_t s[8];
  for( int r = 0; r < 8; r++ )
  {
    s[r] = vld1q_s32( src + r * line );
  }

  int32x4_t O[4];
  for( int k = 0; k < 4; k++ )
  {
    O[k] = vmulq_n_s32( s[1], iT[1 * 8 + k] );
    O[k] = vmlaq_n_s32( O[k], s[3], iT[3 * 8 + k] );
    O[k] = vmlaq_n_s32( O[k], s[5], iT[5 * 8 + k] );
    O[k] = vmlaq_n_s32( O[k], s[7], iT[7 * 8 + k] );
  }
  const int32x4_t EO0 = vmlaq_n_s32( vmulq_n_s32( s[2], iT[2 * 8 + 0] ), s[6], iT[6 * 8 + 0] );
  const int32x4_t EO1 = vmlaq_n_s32( vmulq_n_s32( s[2], iT[2 * 8 + 1] ), s[6], iT[6 * 8 + 1] );
  const int32x4_t EE0 = vshlq_n_s32( vaddq_s32( s[0], s[4] ), 6 );
  const int32x4_t EE1 = vshlq_n_s32( vsubq_s32( s[0], s[4] ), 6 );
  const int32x4_t E[4] = { vaddq_s32( EE0, EO0 ), vaddq_s32( EE1, EO1 ), vsubq_s32( EE1, EO1 ), vsubq_s32( EE0, EO0 ) };

  int32x4_t d[8];
  for( int k = 0; k < 4; k++ )
  {
    d[k]     = vaddq_s32( E[k], O[k] );
    d[k + 4] = vsubq_s32( E[3 - k], O[3 - k] );
  }
  for( int r = 0; r < 8; r++ )
  {
    d[r] = vminq_s32( vmaxq_s32( vrshlq_s32( d[r], shift ), vmin ), vmax );
  }

  // d[r] holds result r of the 4 lines: after the transposes, d[i] and d[i + 4] are line j + i's 8 results
  transpose_4x4_s32( d[0], d[1], d[2], d[3] );
  transpose_4x4_s32( d[4], d[5], d[6], d[7] );
  for( int i = 0; i < 4; i++ )
  {
    vst1q_s32( out + 8 * i, d[i] );
    vst1q_s32( out + 8 * i + 4, d[i + 4] );
  }
}

static void fastInverseDCT2_B8_neon( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2,
                                     const TCoeff outputMinimum, const TCoeff outputMaximum )
{
  const TMatrixCoeff* iT = g_trCoreDCT2P8[TRANSFORM_INVERSE][0];
  CHECKD( iT[0] != 64 || iT[4 * 8] != 64 || iT[1] != 64 || iT[4 * 8 + 1] != -64, "unexpected DCT-II coefficients" );
  const int32x4_t vshift      = vdupq_n_s32( -shift );
  const int32x4_t vmin        = vdupq_n_s32( outputMinimum );
  const int32x4_t vmax        = vdupq_n_s32( outputMaximum );
  const int       reducedLine = line - iSkipLine;

  int j = 0;
  for( ; j + 4 <= reducedLine; j += 4 )
  {
    invDCT2_B8_4lines_neon( iT, src + j, line, dst + 8 * j, vshift, vmin, vmax );
  }
  if( j < reducedLine )
  {
    // the last 1 to 3 lines through a copy padded with zero lines
    TCoeff paddedSrc[8 * 4] = { 0 };
    TCoeff paddedDst[8 * 4];
    for( int r = 0; r < 8; r++ )
    {
      memcpy( paddedSrc + 4 * r, src + r * line + j, ( reducedLine - j ) * sizeof( TCoeff ) );
    }
    invDCT2_B8_4lines_neon( iT, paddedSrc, 4, paddedDst, vshift, vmin, vmax );
    memcpy( dst + 8 * j, paddedDst, 8 * ( reducedLine - j ) * sizeof( TCoeff ) );
  }
}

template<unsigned trSize>
static void fastInvCore_neon( const TMatrixCoeff* it, const TCoeff* src, TCoeff* dst, unsigned lines,
                              unsigned reducedLines, unsigned rows, const TCoeff outputMin, const TCoeff outputMax,
                              const TCoeff round, const TCoeff shift )
{
  CHECKD( round != 1 << ( shift - 1 ), "Round must be 1 << ( shift - 1 )" );

  // the rounding shift (vrshl by -shift adds 1 << ( shift - 1 ) first) and the clip of each sum
  const int32x4_t vshift = vdupq_n_s32( -shift );
  const int32x4_t vmin   = vdupq_n_s32( outputMin );
  const int32x4_t vmax   = vdupq_n_s32( outputMax );
  auto roundClip = [&]( int32x4_t d ) { return vminq_s32( vmaxq_s32( vrshlq_s32( d, vshift ), vmin ), vmax ); };

  static_assert( trSize % 4 == 0, "trSize should be a multiple of four" );
  CHECK( rows % 4 != 0, "rows should be a multiple of four" );
  CHECK( rows == 0, "rows should be non-zero" );

  // Every element of dst[0, reducedLines * trSize) is stored below, so dst needs no clearing first.
  unsigned i = 0;
  for( ; i != ( reducedLines & ~3U ); i += 4 )
  {
    unsigned j = 0;
    do
    {
      const TCoeff* srci      = src + i;
      const TMatrixCoeff* itj = it + j;
      TCoeff* dstij           = dst + i * trSize + j;

      // The first row starts the sums.
      int16x4_t s0 = vmovn_s32( vld1q_s32( srci ) );
      int16x4_t c0 = vld1_s16( itj );

      int32x4_t d0 = vmull_lane_s16( c0, s0, 0 );
      int32x4_t d1 = vmull_lane_s16( c0, s0, 1 );
      int32x4_t d2 = vmull_lane_s16( c0, s0, 2 );
      int32x4_t d3 = vmull_lane_s16( c0, s0, 3 );

      srci += lines;
      itj += trSize;

      unsigned k = rows - 1;
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

      vst1q_s32( dstij + 0 * trSize, roundClip( d0 ) );
      vst1q_s32( dstij + 1 * trSize, roundClip( d1 ) );
      vst1q_s32( dstij + 2 * trSize, roundClip( d2 ) );
      vst1q_s32( dstij + 3 * trSize, roundClip( d3 ) );

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

      int32_t d0 = 0;
      unsigned k = rows;
      do
      {
        d0 += *srci * *itj;

        srci += lines;
        itj += trSize;
      } while( --k != 0 );

      *dstij = Clip3<TCoeff>( outputMin, outputMax, ( TCoeff ) shift_and_round( d0, shift ) );
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
    // the high bytes of the eight lanes keep their sign bits
    const uint64_t highBytes = vget_lane_u64( vreinterpret_u64_s8( vshrn_n_s16( range, 8 ) ), 0 );
    if( ( highBytes & 0x8080808080808080ull ) != 0 )
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

// the four row dots of the LFNST (Trafo_neon.h): the products of 8 int16 inputs and 8 int8 matrix values, widened to
// int16, with vmlal_s16, as four 32-bit partial sums per row
struct LfnstRowDotsNeon
{
  static inline int32x4_t dot( const int16x8_t* in, const int8_t* row, const int n )
  {
    int32x4_t acc = vdupq_n_s32( 0 );
    for( int i = 0; i < n; i += 8 )
    {
      const int8x8_t  m8 = vld1_s8( row + i );
      const int16x8_t m  = vmovl_s8( m8 );
      acc = vmlal_s16( acc, vget_low_s16( in[i / 8] ), vget_low_s16( m ) );
      acc = vmlal_s16( acc, vget_high_s16( in[i / 8] ), vget_high_s16( m ) );
    }
    return acc;
  }

  static inline int32x4_t dot4( const int16x8_t* in, const int8_t* row, const int rowStride, const int n )
  {
    const int32x4_t dot0 = dot( in, row, n );
    const int32x4_t dot1 = dot( in, row + rowStride, n );
    const int32x4_t dot2 = dot( in, row + 2 * rowStride, n );
    const int32x4_t dot3 = dot( in, row + 3 * rowStride, n );
    return horizontal_add_4d_s32x4( dot0, dot1, dot2, dot3 );
  }
};

static void invLfnstNxN_neon( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size,
                              int zeroOutSize )
{
  invLfnstNxN_common<LfnstRowDotsNeon>( src, dst, mode, index, size, zeroOutSize );
}

static void fwdLfnstNxN_neon( int* src, int* dst, const uint32_t mode, const uint32_t index, const uint32_t size,
                              int zeroOutSize )
{
  fwdLfnstNxN_common<LfnstRowDotsNeon>( src, dst, mode, index, size, zeroOutSize );
}

template<>
void TCoeffOps::_initTCoeffOpsARM<NEON>()
{
  fastInvDCT2[ 2 ] = fastInverseDCT2_B8_neon;

  cpyCoeff4  = cpyCoeff_neon<4>;
  cpyCoeff8  = cpyCoeff_neon<8>;
  roundClip8 = roundClip_neon<8>;
  cpyResi4   = cpyResi_neon<4>;
  cpyResi8   = cpyResi_neon<8>;

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

  fastFwdDCT2[ 2 ] = fastForwardDCT2_EvenOdd_neon<16>;
  fastFwdDCT2[ 3 ] = fastForwardDCT2_EvenOdd_neon<32>;
  fastFwdDCT2[ 4 ] = fastForwardDCT2_EvenOdd_neon<64>;
  fastFwdDCT2[ 5 ] = fastForwardDCT2_EvenOdd_neon<8>;
}

template<>
void TrQuant::_initTrQuantARM<NEON>()
{
  m_invLfnstNxN = invLfnstNxN_neon;
  m_fwdLfnstNxN = fwdLfnstNxN_neon;
}

}  // namespace vvenc

#endif
//! \}

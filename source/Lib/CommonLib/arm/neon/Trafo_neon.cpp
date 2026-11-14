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
#include "../mem_neon.h"
#include "CommonLib/CommonDef.h"
#include "TrQuant.h"
#include "TrQuant_EMT.h"
#include "sum_neon.h"
#include "transpose_neon.h"

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

  if( iSkipLine )
  {
    memset( dst + 8 * reducedLine, 0, 8 * iSkipLine * sizeof( TCoeff ) );
  }
}

// The 2- and 4-point transforms (TCoeffOps::fastInvDCT2[0..1], fastFwdDCT2[0..1], fastFwdDST7_4, fastFwdDCT8_4). Like
// their SSE4.1 versions, they take 4 lines at a time, one line per lane, in 32-bit arithmetic as C. The products of the
// 4-point matrices (entries at most 84) with transform inputs stay far inside int32. C's ( d + add ) >> shift is a
// rounding shift: vrshlq_s32 by -shift, which needs no add and is the identity when shift is 0, where C's add is 0 as
// well. The inverse clips to the output range. Both directions clear the skipped lines, as C. Each kernel below takes
// numLines (1..4) lines: the loop calls it with the constant 4, so the partial loads and stores fold away; the last 1
// to 3 lines of a call, if any, take one more call that loads and stores only those lines.
//
// The 2-point matrix is { 64, 64 }, { 64, -64 }. C rounds 64 * x by shift, which is x rounded by shift - 6: the 6 zero
// low bits of 64 * x never change the rounding, and for shift < 6 both are x << ( 6 - shift ). vrshlq_s32 by 6 - shift
// does both, so the 2-point kernels do not multiply by 64 (equal wherever 64 * x does not overflow, as above).

// Inverse 2-point DCT-II of lines j..j+numLines-1: src[j..] and src[line + j..], out[2 * i + r] = result r of line
// j + i; shift6 = 6 - shift
static ALWAYS_INLINE void invDCT2_B2_lines_neon( const TCoeff* src, int line, int numLines, TCoeff* out,
                                                 const int32x4_t shift6, const int32x4_t vmin, const int32x4_t vmax )
{
  const int32x4_t s0 = load_s32x4_n( src, numLines );
  const int32x4_t s1 = load_s32x4_n( src + line, numLines );

  int32x4_t e = vaddq_s32( s0, s1 );
  int32x4_t o = vsubq_s32( s0, s1 );
  e = vminq_s32( vmaxq_s32( vrshlq_s32( e, shift6 ), vmin ), vmax );
  o = vminq_s32( vmaxq_s32( vrshlq_s32( o, shift6 ), vmin ), vmax );

  // eo.val[0]: e0 o0 e1 o1, eo.val[1]: e2 o2 e3 o3
  const int32x4x2_t eo        = vzipq_s32( e, o );
  const int         numValues = 2 * numLines;
  store_s32x4_n( out, eo.val[0], numValues > 4 ? 4 : numValues );
  store_s32x4_n( out + 4, eo.val[1], numValues > 4 ? numValues - 4 : 0 );
}

static void fastInverseDCT2_B2_neon( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2,
                                     const TCoeff outputMinimum, const TCoeff outputMaximum )
{
  CHECKD( g_trCoreDCT2P2[TRANSFORM_INVERSE][0][0] != 64 || g_trCoreDCT2P2[TRANSFORM_INVERSE][0][2] != 64,
          "unexpected DCT-II coefficients" );
  const int32x4_t vshift6     = vdupq_n_s32( 6 - shift );
  const int32x4_t vmin        = vdupq_n_s32( outputMinimum );
  const int32x4_t vmax        = vdupq_n_s32( outputMaximum );
  const int       reducedLine = line - iSkipLine;

  int j = 0;
  for( ; j + 4 <= reducedLine; j += 4 )
  {
    invDCT2_B2_lines_neon( src + j, line, 4, dst + 2 * j, vshift6, vmin, vmax );
  }
  if( j < reducedLine )
  {
    invDCT2_B2_lines_neon( src + j, line, reducedLine - j, dst + 2 * j, vshift6, vmin, vmax );
  }

  if( iSkipLine )
  {
    memset( dst + 2 * reducedLine, 0, 2 * iSkipLine * sizeof( TCoeff ) );
  }
}

// Inverse 4-point DCT-II of lines j..j+numLines-1: s0..s3 = src[r * line + j..] for r = 0..3, out[4 * i + r] = result
// r of line j + i
static ALWAYS_INLINE void invDCT2_B4_lines_neon( const TMatrixCoeff* iT, const TCoeff* src, int line, int numLines,
                                                 TCoeff* out, const int32x4_t shift, const int32x4_t vmin,
                                                 const int32x4_t vmax )
{
  const int32x4_t s0 = load_s32x4_n( src, numLines );
  const int32x4_t s1 = load_s32x4_n( src + line, numLines );
  const int32x4_t s2 = load_s32x4_n( src + 2 * line, numLines );
  const int32x4_t s3 = load_s32x4_n( src + 3 * line, numLines );

  const int32x4_t O0 = vmlaq_n_s32( vmulq_n_s32( s1, iT[1 * 4 + 0] ), s3, iT[3 * 4 + 0] );
  const int32x4_t O1 = vmlaq_n_s32( vmulq_n_s32( s1, iT[1 * 4 + 1] ), s3, iT[3 * 4 + 1] );
  const int32x4_t E0 = vshlq_n_s32( vaddq_s32( s0, s2 ), 6 );
  const int32x4_t E1 = vshlq_n_s32( vsubq_s32( s0, s2 ), 6 );

  int32x4_t d0 = vaddq_s32( E0, O0 );
  int32x4_t d1 = vaddq_s32( E1, O1 );
  int32x4_t d2 = vsubq_s32( E1, O1 );
  int32x4_t d3 = vsubq_s32( E0, O0 );
  d0 = vminq_s32( vmaxq_s32( vrshlq_s32( d0, shift ), vmin ), vmax );
  d1 = vminq_s32( vmaxq_s32( vrshlq_s32( d1, shift ), vmin ), vmax );
  d2 = vminq_s32( vmaxq_s32( vrshlq_s32( d2, shift ), vmin ), vmax );
  d3 = vminq_s32( vmaxq_s32( vrshlq_s32( d3, shift ), vmin ), vmax );

  // d_r holds result r of lines j..j+3; after the transpose, d_i holds results 0..3 of line j + i
  transpose_4x4_s32( d0, d1, d2, d3 );
  vst1q_s32( out, d0 );
  if( numLines > 1 )
  {
    vst1q_s32( out + 4, d1 );
  }
  if( numLines > 2 )
  {
    vst1q_s32( out + 8, d2 );
  }
  if( numLines > 3 )
  {
    vst1q_s32( out + 12, d3 );
  }
}

static void fastInverseDCT2_B4_neon( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2,
                                     const TCoeff outputMinimum, const TCoeff outputMaximum )
{
  const TMatrixCoeff* iT = g_trCoreDCT2P4[TRANSFORM_INVERSE][0];
  CHECKD( iT[0] != 64 || iT[2 * 4] != 64 || iT[1] != 64 || iT[2 * 4 + 1] != -64, "unexpected DCT-II coefficients" );
  const int32x4_t vshift      = vdupq_n_s32( -shift );
  const int32x4_t vmin        = vdupq_n_s32( outputMinimum );
  const int32x4_t vmax        = vdupq_n_s32( outputMaximum );
  const int       reducedLine = line - iSkipLine;

  int j = 0;
  for( ; j + 4 <= reducedLine; j += 4 )
  {
    invDCT2_B4_lines_neon( iT, src + j, line, 4, dst + 4 * j, vshift, vmin, vmax );
  }
  if( j < reducedLine )
  {
    invDCT2_B4_lines_neon( iT, src + j, line, reducedLine - j, dst + 4 * j, vshift, vmin, vmax );
  }

  if( iSkipLine )
  {
    memset( dst + 4 * reducedLine, 0, 4 * iSkipLine * sizeof( TCoeff ) );
  }
}

// Forward 2-point DCT-II of lines j..j+numLines-1: src = their values line by line, dst[k * line + i] = result k of
// line j + i; shift6 = 6 - shift
static ALWAYS_INLINE void fwdDCT2_B2_lines_neon( const TCoeff* src, int numLines, TCoeff* dst, int line,
                                                 const int32x4_t shift6 )
{
  const int       numValues = 2 * numLines;
  const int32x4_t a         = load_s32x4_n( src, numValues > 4 ? 4 : numValues );
  const int32x4_t b         = load_s32x4_n( src + 4, numValues > 4 ? numValues - 4 : 0 );

  // a and b hold the pairs (s0, s1) of 2 lines each: s.val[0] is s0 of the 4 lines, s.val[1] their s1
  const int32x4x2_t s = vuzpq_s32( a, b );

  const int32x4_t E = vaddq_s32( s.val[0], s.val[1] );
  const int32x4_t O = vsubq_s32( s.val[0], s.val[1] );
  store_s32x4_n( dst, vrshlq_s32( E, shift6 ), numLines );
  store_s32x4_n( dst + line, vrshlq_s32( O, shift6 ), numLines );
}

static void fastForwardDCT2_B2_neon( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2 )
{
  CHECKD( g_trCoreDCT2P2[TRANSFORM_FORWARD][0][0] != 64 || g_trCoreDCT2P2[TRANSFORM_FORWARD][0][2] != 64,
          "unexpected DCT-II coefficients" );
  const int32x4_t vshift6     = vdupq_n_s32( 6 - shift );
  const int       reducedLine = line - iSkipLine;

  int j = 0;
  for( ; j + 4 <= reducedLine; j += 4 )
  {
    fwdDCT2_B2_lines_neon( src + 2 * j, 4, dst + j, line, vshift6 );
  }
  if( j < reducedLine )
  {
    fwdDCT2_B2_lines_neon( src + 2 * j, reducedLine - j, dst + j, line, vshift6 );
  }

  if( iSkipLine )
  {
    memset( dst + reducedLine, 0, sizeof( TCoeff ) * iSkipLine );
    memset( dst + line + reducedLine, 0, sizeof( TCoeff ) * iSkipLine );
  }
}

// The forward 4-point butterflies of TrQuant_EMT.cpp, on values s0..s3 of 4 lines: d0..d3 before the rounding shift

static ALWAYS_INLINE void fwdButterflyDCT2_4_neon( const TMatrixCoeff* iT, const int32x4_t s0, const int32x4_t s1,
                                                   const int32x4_t s2, const int32x4_t s3, int32x4_t& d0,
                                                   int32x4_t& d1, int32x4_t& d2, int32x4_t& d3 )
{
  const int32x4_t E0 = vaddq_s32( s0, s3 );
  const int32x4_t O0 = vsubq_s32( s0, s3 );
  const int32x4_t E1 = vaddq_s32( s1, s2 );
  const int32x4_t O1 = vsubq_s32( s1, s2 );

  d0 = vshlq_n_s32( vaddq_s32( E0, E1 ), 6 );
  d2 = vshlq_n_s32( vsubq_s32( E0, E1 ), 6 );
  d1 = vmlaq_n_s32( vmulq_n_s32( O0, iT[4] ), O1, iT[5] );
  d3 = vmlaq_n_s32( vmulq_n_s32( O0, iT[12] ), O1, iT[13] );
}

static ALWAYS_INLINE void fwdButterflyDST7_4_neon( const TMatrixCoeff* iT, const int32x4_t s0, const int32x4_t s1,
                                                   const int32x4_t s2, const int32x4_t s3, int32x4_t& d0,
                                                   int32x4_t& d1, int32x4_t& d2, int32x4_t& d3 )
{
  const int32x4_t c0 = vaddq_s32( s0, s3 );
  const int32x4_t c1 = vaddq_s32( s1, s3 );
  const int32x4_t c2 = vsubq_s32( s0, s1 );
  const int32x4_t c3 = vmulq_n_s32( s2, iT[2] );

  d0 = vmlaq_n_s32( vmlaq_n_s32( c3, c0, iT[0] ), c1, iT[1] );
  d1 = vmulq_n_s32( vsubq_s32( vaddq_s32( s0, s1 ), s3 ), iT[2] );
  d2 = vsubq_s32( vmlaq_n_s32( vmulq_n_s32( c2, iT[0] ), c0, iT[1] ), c3 );
  d3 = vmlsq_n_s32( vmlaq_n_s32( c3, c2, iT[1] ), c1, iT[0] );
}

static ALWAYS_INLINE void fwdButterflyDCT8_4_neon( const TMatrixCoeff* iT, const int32x4_t s0, const int32x4_t s1,
                                                   const int32x4_t s2, const int32x4_t s3, int32x4_t& d0,
                                                   int32x4_t& d1, int32x4_t& d2, int32x4_t& d3 )
{
  const int32x4_t c0 = vaddq_s32( s0, s3 );
  const int32x4_t c1 = vaddq_s32( s2, s0 );
  const int32x4_t c2 = vsubq_s32( s3, s2 );
  const int32x4_t c3 = vmulq_n_s32( s1, iT[1] );

  d0 = vmlaq_n_s32( vmlaq_n_s32( c3, c0, iT[3] ), c1, iT[2] );
  d1 = vmulq_n_s32( vsubq_s32( vsubq_s32( s0, s2 ), s3 ), iT[1] );
  d2 = vsubq_s32( vmlaq_n_s32( vmulq_n_s32( c2, iT[3] ), c0, iT[2] ), c3 );
  d3 = vsubq_s32( vmlsq_n_s32( vmulq_n_s32( c1, iT[3] ), c2, iT[2] ), c3 );
}

typedef void ( *FwdButterfly4Neon )( const TMatrixCoeff* iT, const int32x4_t s0, const int32x4_t s1,
                                    const int32x4_t s2, const int32x4_t s3, int32x4_t& d0, int32x4_t& d1, int32x4_t& d2,
                                    int32x4_t& d3 );

// Forward 4-point transform of lines j..j+numLines-1: src = their values line by line, dst[k * line + i] = result k of
// line j + i
template<FwdButterfly4Neon butterfly>
static ALWAYS_INLINE void fwd_B4_lines_neon( const TMatrixCoeff* iT, const TCoeff* src, int numLines, TCoeff* dst,
                                             int line, const int32x4_t shift )
{
  // the lines past numLines are zero
  const int32x4_t zero = vdupq_n_s32( 0 );
  int32x4_t       s0   = vld1q_s32( src );
  int32x4_t       s1   = numLines > 1 ? vld1q_s32( src + 4 ) : zero;
  int32x4_t       s2   = numLines > 2 ? vld1q_s32( src + 8 ) : zero;
  int32x4_t       s3   = numLines > 3 ? vld1q_s32( src + 12 ) : zero;

  // s0..s3 hold one line each: after the transpose, s0..s3 hold values 0..3 of the 4 lines
  transpose_4x4_s32( s0, s1, s2, s3 );

  int32x4_t d0, d1, d2, d3;
  butterfly( iT, s0, s1, s2, s3, d0, d1, d2, d3 );

  store_s32x4_n( dst, vrshlq_s32( d0, shift ), numLines );
  store_s32x4_n( dst + line, vrshlq_s32( d1, shift ), numLines );
  store_s32x4_n( dst + 2 * line, vrshlq_s32( d2, shift ), numLines );
  store_s32x4_n( dst + 3 * line, vrshlq_s32( d3, shift ), numLines );
}

template<FwdButterfly4Neon butterfly>
static void fastForward_B4_neon( const TMatrixCoeff* iT, const TCoeff* src, TCoeff* dst, int shift, int line,
                                 int iSkipLine )
{
  const int32x4_t vshift      = vdupq_n_s32( -shift );
  const int       reducedLine = line - iSkipLine;

  int j = 0;
  for( ; j + 4 <= reducedLine; j += 4 )
  {
    fwd_B4_lines_neon<butterfly>( iT, src + 4 * j, 4, dst + j, line, vshift );
  }
  if( j < reducedLine )
  {
    fwd_B4_lines_neon<butterfly>( iT, src + 4 * j, reducedLine - j, dst + j, line, vshift );
  }

  if( iSkipLine )
  {
    memset( dst + reducedLine, 0, sizeof( TCoeff ) * iSkipLine );
    memset( dst + line + reducedLine, 0, sizeof( TCoeff ) * iSkipLine );
    memset( dst + 2 * line + reducedLine, 0, sizeof( TCoeff ) * iSkipLine );
    memset( dst + 3 * line + reducedLine, 0, sizeof( TCoeff ) * iSkipLine );
  }
}

static void fastForwardDCT2_B4_neon( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2 )
{
  const TMatrixCoeff* iT = g_trCoreDCT2P4[TRANSFORM_FORWARD][0];
  CHECKD( iT[0] != 64 || iT[1] != 64 || iT[8] != 64 || iT[9] != -64, "unexpected DCT-II coefficients" );
  fastForward_B4_neon<fwdButterflyDCT2_4_neon>( iT, src, dst, shift, line, iSkipLine );
}

static void fastForwardDST7_B4_neon( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2 )
{
  fastForward_B4_neon<fwdButterflyDST7_4_neon>( g_trCoreDST7P4[TRANSFORM_FORWARD][0], src, dst, shift, line, iSkipLine );
}

static void fastForwardDCT8_B4_neon( const TCoeff* src, TCoeff* dst, int shift, int line, int iSkipLine, int iSkipLine2 )
{
  // C's DCT-VIII adds 1 << ( shift - 1 ) without checking for shift 0, so its shift is never 0 here
  CHECKD( shift == 0, "DCT-VIII with shift 0" );
  fastForward_B4_neon<fwdButterflyDCT8_4_neon>( g_trCoreDCT8P4[TRANSFORM_FORWARD][0], src, dst, shift, line, iSkipLine );
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

template<>
void TCoeffOps::_initTCoeffOpsARM<NEON>()
{
  fastInvDCT2[ 0 ] = fastInverseDCT2_B2_neon;
  fastInvDCT2[ 1 ] = fastInverseDCT2_B4_neon;
  fastInvDCT2[ 2 ] = fastInverseDCT2_B8_neon;
  fastFwdDCT2[ 0 ] = fastForwardDCT2_B2_neon;
  fastFwdDCT2[ 1 ] = fastForwardDCT2_B4_neon;
  fastFwdDST7_4    = fastForwardDST7_B4_neon;
  fastFwdDCT8_4    = fastForwardDCT8_B4_neon;

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
}

}  // namespace vvenc

#endif
//! \}

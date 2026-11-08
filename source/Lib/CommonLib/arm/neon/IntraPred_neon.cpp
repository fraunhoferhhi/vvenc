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
 * \file IntraPred_neon.cpp
 * \brief Neon implementation of IntraPrediction for Arm.
 */
//  ====================================================================================================================
//  Includes
//  ====================================================================================================================

#include "../CommonDefARM.h"
#include "CommonLib/InterpolationFilter.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/Rom.h"
#include "CommonLib/Unit.h"

#if ENABLE_SIMD_OPT_INTRAPRED && defined( TARGET_SIMD_ARM )

#include <arm_neon.h>

#include "transpose_neon.h"

namespace vvenc
{

// The 4-tap filters of IntraPredAngleLuma_neon on 8 (or 4) outputs, from the 4 vectors p0..p3 of reference samples
// starting 0..3 samples after the first output's integer position.
// Cubic (the chroma filter of the fraction, rounded, clipped to [0, max]): the accumulate of taps 0 and 3 is at most
// (-6 + -4) * 10-bit, so int16_t; that of taps 1 and 2 (46 + 28) * 10-bit, so int32_t.
static inline int16x8_t intraCubic8_neon( const int16x8_t p0, const int16x8_t p1, const int16x8_t p2, const int16x8_t p3,
                                          const int16x4_t filter, const uint16x8_t vmax )
{
  int16x8_t sum = vmulq_lane_s16( p0, filter, 0 );
  sum = vmlaq_lane_s16( sum, p3, filter, 3 );
  int32x4_t sum_lo = vmull_lane_s16( vget_low_s16( p1 ), filter, 1 );
  sum_lo = vmlal_lane_s16( sum_lo, vget_low_s16( p2 ), filter, 2 );
  sum_lo = vaddw_s16( sum_lo, vget_low_s16( sum ) );
  int32x4_t sum_hi = vmull_lane_s16( vget_high_s16( p1 ), filter, 1 );
  sum_hi = vmlal_lane_s16( sum_hi, vget_high_s16( p2 ), filter, 2 );
  sum_hi = vaddw_s16( sum_hi, vget_high_s16( sum ) );

  // Only cubic filter has negative coefficients and requires clipping.
  const uint16x4_t d_lo = vqrshrun_n_s32( sum_lo, 6 );
  const uint16x4_t d_hi = vqrshrun_n_s32( sum_hi, 6 );
  return vreinterpretq_s16_u16( vminq_u16( vcombine_u16( d_lo, d_hi ), vmax ) );
}

static inline int16x4_t intraCubic4_neon( const int16x4_t p0, const int16x4_t p1, const int16x4_t p2, const int16x4_t p3,
                                          const int16x4_t filter, const uint16x4_t vmax )
{
  // Max accumulate is (46 + 28) * 10-bit which will not fit into int16_t so use int32_t.
  int32x4_t sum = vmull_lane_s16( p0, filter, 0 );
  sum = vmlal_lane_s16( sum, p1, filter, 1 );
  sum = vmlal_lane_s16( sum, p2, filter, 2 );
  sum = vmlal_lane_s16( sum, p3, filter, 3 );

  // Only cubic filter has negative coefficients and requires clipping.
  return vreinterpret_s16_u16( vmin_u16( vqrshrun_n_s32( sum, 6 ), vmax ) );
}

// Smoothing (taps 16 - f / 2, 32 - f / 2, 16 + f / 2, f / 2, which add up to 64): the accumulate is at most unsigned
// 10-bit << 6, so uint16_t, and needs no clip.
static inline uint16x4_t intraSmoothingFilter_neon( const int deltaFract )
{
  // in registers: lanes built through a stack array would stall store forwarding on every row
  static const int16_t base[4] = { 16, 32, 16, 0 };
  static const int16_t sign[4] = { -1, -1, 1, 1 };
  return vreinterpret_u16_s16( vmla_n_s16( vld1_s16( base ), vld1_s16( sign ), int16_t( deltaFract >> 1 ) ) );
}

static inline int16x8_t intraSmooth8_neon( const int16x8_t p0, const int16x8_t p1, const int16x8_t p2, const int16x8_t p3,
                                           const uint16x4_t filter )
{
  uint16x8_t sum = vmlaq_lane_u16( vdupq_n_u16( 32 ), vreinterpretq_u16_s16( p0 ), filter, 0 );
  sum = vmlaq_lane_u16( sum, vreinterpretq_u16_s16( p1 ), filter, 1 );
  sum = vmlaq_lane_u16( sum, vreinterpretq_u16_s16( p2 ), filter, 2 );
  sum = vmlaq_lane_u16( sum, vreinterpretq_u16_s16( p3 ), filter, 3 );
  return vreinterpretq_s16_u16( vshrq_n_u16( sum, 6 ) );
}

static inline int16x4_t intraSmooth4_neon( const int16x4_t p0, const int16x4_t p1, const int16x4_t p2, const int16x4_t p3,
                                           const uint16x4_t filter )
{
  uint16x4_t sum = vmla_lane_u16( vdup_n_u16( 32 ), vreinterpret_u16_s16( p0 ), filter, 0 );
  sum = vmla_lane_u16( sum, vreinterpret_u16_s16( p1 ), filter, 1 );
  sum = vmla_lane_u16( sum, vreinterpret_u16_s16( p2 ), filter, 2 );
  sum = vmla_lane_u16( sum, vreinterpret_u16_s16( p3 ), filter, 3 );
  return vreinterpret_s16_u16( vshr_n_u16( sum, 6 ) );
}

void IntraPredAngleLuma_neon( Pel* pDstBuf, const ptrdiff_t dstStride, Pel* refMain, int width, int height,
                              int deltaPos, int intraPredAngle, const TFilterCoeff* /* ff_unused */,
                              const bool useCubicFilter, const ClpRng& clpRng )
{
  CHECK( height < 1, "Invalid height. Must be >= 1" );
  CHECK( width < 4, "Invalid width. Must be >= 4" );
  CHECK( ( width & 7 ) && ( width != 4 ), "Invalid width. Must be 4 or a multiple of 8" );
  CHECK( clpRng.bd > 10, "Invalid bit-depth. Must be <= 10" );

  if( useCubicFilter )
  {
    const uint16x8_t vmax = vdupq_n_u16( clpRng.max() );
    do
    {
      const int deltaInt = deltaPos >> 5;
      const int deltaFract = deltaPos & ( 32 - 1 );

      // Chroma Filter max value is 64.
      const int16x4_t filter = vld1_s16( InterpolationFilter::getChromaFilterTable( deltaFract ) );

      const Pel* pRef = refMain + deltaInt;
      for( int w = 0; w <= width - 8; w += 8 )
      {
        vst1q_s16( pDstBuf + w, intraCubic8_neon( vld1q_s16( pRef + 0 ), vld1q_s16( pRef + 1 ), vld1q_s16( pRef + 2 ),
                                                  vld1q_s16( pRef + 3 ), filter, vmax ) );
        pRef += 8;
      }
      if( width == 4 )
      {
        vst1_s16( pDstBuf, intraCubic4_neon( vld1_s16( pRef + 0 ), vld1_s16( pRef + 1 ), vld1_s16( pRef + 2 ),
                                             vld1_s16( pRef + 3 ), filter, vget_low_u16( vmax ) ) );
      }

      pDstBuf += dstStride;
      deltaPos += intraPredAngle;
    } while( --height != 0 );
  }
  else // ( !useCubicFilter )
  {
    do
    {
      const int deltaInt = deltaPos >> 5;
      const int deltaFract = deltaPos & ( 32 - 1 );

      const uint16x4_t filter = intraSmoothingFilter_neon( deltaFract );

      const Pel* pRef = refMain + deltaInt;
      for( int w = 0; w <= width - 8; w += 8 )
      {
        vst1q_s16( pDstBuf + w, intraSmooth8_neon( vld1q_s16( pRef + 0 ), vld1q_s16( pRef + 1 ), vld1q_s16( pRef + 2 ),
                                                   vld1q_s16( pRef + 3 ), filter ) );
        pRef += 8;
      }
      if( width == 4 )
      {
        vst1_s16( pDstBuf, intraSmooth4_neon( vld1_s16( pRef + 0 ), vld1_s16( pRef + 1 ), vld1_s16( pRef + 2 ),
                                              vld1_s16( pRef + 3 ), filter ) );
      }

      pDstBuf += dstStride;
      deltaPos += intraPredAngle;
    } while( --height != 0 );
  }
}

// Transposes 8 vectors of 8 16-bit values in place (v[i] lane j -> v[j] lane i), with vtrn and 64-bit halves only.
static inline void transpose8x8_s16( int16x8_t* v )
{
  const int16x8x2_t t01 = vtrnq_s16( v[0], v[1] );
  const int16x8x2_t t23 = vtrnq_s16( v[2], v[3] );
  const int16x8x2_t t45 = vtrnq_s16( v[4], v[5] );
  const int16x8x2_t t67 = vtrnq_s16( v[6], v[7] );

  // e.g. u02.val[0] holds lane 0 of v[0..3], then lane 4 of v[0..3]; w02 the same of v[4..7]
  const int32x4x2_t u02 = vtrnq_s32( vreinterpretq_s32_s16( t01.val[0] ), vreinterpretq_s32_s16( t23.val[0] ) );
  const int32x4x2_t u13 = vtrnq_s32( vreinterpretq_s32_s16( t01.val[1] ), vreinterpretq_s32_s16( t23.val[1] ) );
  const int32x4x2_t w02 = vtrnq_s32( vreinterpretq_s32_s16( t45.val[0] ), vreinterpretq_s32_s16( t67.val[0] ) );
  const int32x4x2_t w13 = vtrnq_s32( vreinterpretq_s32_s16( t45.val[1] ), vreinterpretq_s32_s16( t67.val[1] ) );

  const int16x8x2_t c04 = vvenc_vtrnq_s64_to_s16( vreinterpretq_s16_s32( u02.val[0] ), vreinterpretq_s16_s32( w02.val[0] ) );
  const int16x8x2_t c15 = vvenc_vtrnq_s64_to_s16( vreinterpretq_s16_s32( u13.val[0] ), vreinterpretq_s16_s32( w13.val[0] ) );
  const int16x8x2_t c26 = vvenc_vtrnq_s64_to_s16( vreinterpretq_s16_s32( u02.val[1] ), vreinterpretq_s16_s32( w02.val[1] ) );
  const int16x8x2_t c37 = vvenc_vtrnq_s64_to_s16( vreinterpretq_s16_s32( u13.val[1] ), vreinterpretq_s16_s32( w13.val[1] ) );
  v[0] = c04.val[0];
  v[1] = c15.val[0];
  v[2] = c26.val[0];
  v[3] = c37.val[0];
  v[4] = c04.val[1];
  v[5] = c15.val[1];
  v[6] = c26.val[1];
  v[7] = c37.val[1];
}

// Transposes 4 vectors of 4 16-bit values (in[i] lane j -> out[j] lane i).
static inline void transpose4x4_s16( const int16x4_t* in, int16x4_t* out )
{
  const int16x4x2_t t01 = vtrn_s16( in[0], in[1] );
  const int16x4x2_t t23 = vtrn_s16( in[2], in[3] );
  const int32x2x2_t u02 = vtrn_s32( vreinterpret_s32_s16( t01.val[0] ), vreinterpret_s32_s16( t23.val[0] ) );
  const int32x2x2_t u13 = vtrn_s32( vreinterpret_s32_s16( t01.val[1] ), vreinterpret_s32_s16( t23.val[1] ) );
  out[0] = vreinterpret_s16_s32( u02.val[0] );
  out[1] = vreinterpret_s16_s32( u13.val[0] );
  out[2] = vreinterpret_s16_s32( u02.val[1] );
  out[3] = vreinterpret_s16_s32( u13.val[1] );
}

// IntraAnglePDPC's blend of one row in the block's own orientation: pred + ( ( wL * ( left - pred ) + 32 ) >> 6 ) with
// weight = wL << 9, as vqrdmulh( d, wL << 9 ) = ( 2 * d * wL * 512 + 2^15 ) >> 16 = ( d * wL + 32 ) >> 6 exactly
// (|d| < 2^11 and wL <= 32: no saturation). Weight 0 keeps the row.
static inline int16x8_t intraPdpcRow_neon( const int16x8_t pred, const Pel* left, const int16_t weight )
{
  return vaddq_s16( pred, vqrdmulhq_n_s16( vsubq_s16( vld1q_s16( left ), pred ), weight ) );
}

// Horizontal luma modes predicted in the block's own orientation (the contract of IntraPredAngleLumaHor_SIMD,
// IntraPredX86.h): IntraPredAngleLuma_neon and IntraAnglePDPC of the transposed block, transposed, without the
// temporary block. Block column x is row x of the transposed block: its integer position i_x and fraction from
// deltaPos = angle * ( x + 1 + mri ) and its filter are constant down the column, and its outputs in rows y .. y + 7
// read the consecutive samples refMain[i_x + y + k .. i_x + y + k + 7], k = 0..3. So per tile of 8 columns and 8 rows
// (4 for blocks of height 4) each column is filtered as one vector with IntraPredAngleLuma_neon's arithmetic, and a
// transpose in registers gives the rows. With PDPC (positive angles, reference line 0) rows y < n = min( 3 << scale,
// height ) then get the blend with their left samples refSide[x + off( y )], consecutive in x. Reads: the samples the
// C code reads, no others. width a multiple of 8, height 4 or a multiple of 8, at most 10 bits.
template<bool cubic, bool pdpc>
static void intraPredAngleLumaHor_neon( Pel* dst, const ptrdiff_t dstStride, const Pel* refMain, const Pel* refSide,
                                        const int width, const int height, const int intraPredAngle,
                                        const int multiRefIdx, const int scale, const int invAngle, const ClpRng& clpRng )
{
  const uint16x8_t vmax = vdupq_n_u16( clpRng.max() );

  // PDPC: rows y < n with their left sample offset (+ 1) and weight << 9; the other rows of the tiles that have such
  // rows get weight 0 and row n - 1's offset (a valid address)
  int     n = 0;
  int     off[16];
  int16_t wgt[16];
  if( pdpc )
  {
    n = std::min( 3 << scale, height );
    for( int y = 0; y < 16; y++ )
    {
      off[y] = ( ( ( std::min( y, n - 1 ) + 1 ) * invAngle + 256 ) >> 9 ) + 1;
      wgt[y] = y < n ? int16_t( ( 32 >> ( 2 * y >> scale ) ) << 9 ) : 0;
    }
  }

  for( int x0 = 0; x0 < width; x0 += 8, dst += 8 )
  {
    const Pel* ref[8];
    int16x4_t  filter[8];
    for( int j = 0; j < 8; j++ )
    {
      const int deltaPos = intraPredAngle * ( x0 + j + 1 + multiRefIdx );
      ref[j]    = refMain + ( deltaPos >> 5 );
      filter[j] = cubic ? vld1_s16( InterpolationFilter::getChromaFilterTable( deltaPos & 31 ) )
                        : vreinterpret_s16_u16( intraSmoothingFilter_neon( deltaPos & 31 ) );
    }

    if( height == 4 )
    {
      int16x4_t cols[8], rowsLeft[4], rowsRight[4];
      for( int j = 0; j < 8; j++ )
      {
        const Pel*      r = ref[j];
        const int16x4_t f = filter[j];
        cols[j] = cubic ? intraCubic4_neon( vld1_s16( r ), vld1_s16( r + 1 ), vld1_s16( r + 2 ), vld1_s16( r + 3 ), f, vget_low_u16( vmax ) )
                        : intraSmooth4_neon( vld1_s16( r ), vld1_s16( r + 1 ), vld1_s16( r + 2 ), vld1_s16( r + 3 ), vreinterpret_u16_s16( f ) );
      }
      transpose4x4_s16( cols, rowsLeft );
      transpose4x4_s16( cols + 4, rowsRight );
      for( int y = 0; y < 4; y++ )
      {
        int16x8_t row = vcombine_s16( rowsLeft[y], rowsRight[y] );
        if( pdpc )
        {
          row = intraPdpcRow_neon( row, refSide + x0 + off[y], wgt[y] );
        }
        vst1q_s16( dst + y * dstStride, row );
      }
      continue;
    }

    for( int y0 = 0; y0 < height; y0 += 8 )
    {
      int16x8_t v[8];
      for( int j = 0; j < 8; j++ )
      {
        const Pel*      r = ref[j] + y0;
        const int16x4_t f = filter[j];
        v[j] = cubic ? intraCubic8_neon( vld1q_s16( r ), vld1q_s16( r + 1 ), vld1q_s16( r + 2 ), vld1q_s16( r + 3 ), f, vmax )
                     : intraSmooth8_neon( vld1q_s16( r ), vld1q_s16( r + 1 ), vld1q_s16( r + 2 ), vld1q_s16( r + 3 ), vreinterpret_u16_s16( f ) );
      }
      transpose8x8_s16( v );   // v[y]: row y0 + y of the 8 columns

      if( pdpc && y0 < n )
      {
        for( int y = 0; y < 8; y++ )
        {
          v[y] = intraPdpcRow_neon( v[y], refSide + x0 + off[y0 + y], wgt[y0 + y] );
        }
      }
      for( int y = 0; y < 8; y++ )
      {
        vst1q_s16( dst + ( y0 + y ) * dstStride, v[y] );
      }
    }
  }
}

static void IntraPredAngleLumaHor_neon( Pel* dst, const ptrdiff_t dstStride, const Pel* refMain, const Pel* refSide,
                                        const int width, const int height, const int intraPredAngle,
                                        const int multiRefIdx, const int scale, const int invAngle,
                                        const bool useCubicFilter, const bool applyPDPC, const ClpRng& clpRng )
{
  CHECKD( width < 8 || ( width & 7 ) != 0 || ( height != 4 && ( height & 7 ) != 0 ), "block size not supported" );
  CHECKD( clpRng.bd > 10, "Only bit-depths of up to 10 bits supported!" );

  if( useCubicFilter )
  {
    if( applyPDPC ) intraPredAngleLumaHor_neon<true, true >( dst, dstStride, refMain, refSide, width, height, intraPredAngle, multiRefIdx, scale, invAngle, clpRng );
    else            intraPredAngleLumaHor_neon<true, false>( dst, dstStride, refMain, refSide, width, height, intraPredAngle, multiRefIdx, scale, invAngle, clpRng );
  }
  else
  {
    if( applyPDPC ) intraPredAngleLumaHor_neon<false, true >( dst, dstStride, refMain, refSide, width, height, intraPredAngle, multiRefIdx, scale, invAngle, clpRng );
    else            intraPredAngleLumaHor_neon<false, false>( dst, dstStride, refMain, refSide, width, height, intraPredAngle, multiRefIdx, scale, invAngle, clpRng );
  }
}

template<>
void IntraPrediction::_initIntraPredictionARM<NEON>()
{
  IntraPredAngleLuma    = IntraPredAngleLuma_neon;
  IntraPredAngleLumaHor = IntraPredAngleLumaHor_neon;
}

} // namespace vvenc

#endif // ENABLE_SIMD_OPT_INTRAPRED && defined( TARGET_SIMD_ARM )

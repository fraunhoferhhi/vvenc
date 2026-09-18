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
 * \file Quant_neon.cpp
 * \brief Neon implementation of quantization functions for Arm.
 */
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include <arm_neon.h>

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Quant.h"
#include "CommonLib/arm/mem_neon.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_QUANT

namespace vvenc
{

// Local any-lane-set reduction over a u32 all-ones/all-zeros compare
// mask. vpmaxq_u32 is AArch64-only, so this follows the same
// #if REAL_TARGET_AARCH64 workaround pattern as the pairwise_add_*
// helpers in sum_neon.h for armv7.
static inline bool any_lane_set_u32x4( const uint32x4_t v )
{
#if REAL_TARGET_AARCH64
  return vgetq_lane_u64( vreinterpretq_u64_u32( vpmaxq_u32( v, v ) ), 0 ) != 0;
#else
  const uint32x2_t m = vorr_u32( vget_low_u32( v ), vget_high_u32( v ) );
  return vget_lane_u64( vreinterpret_u64_u32( m ), 0 ) != 0;
#endif
}

// Mirrors needRdoqCore (Quant.cpp): a reduction over the coefficient
// buffer that returns true as soon as any coefficient survives
// quantization. Real call sites always pass a numCoeff that is a
// multiple of four (efArea = width * min(height,32), both powers of
// two), so the bulk of the buffer is handled 8 coefficients at a time
// with a 4-wide remainder step.
//
// The scalar test is ( abs(iLevel)*quantCoeff + offset ) >> shift != 0.
// That sum is non-negative in the real call domain (coeff clipped to
// +-2^15, quantCoeff and offset always positive), so it's non-zero iff
// abs(iLevel)*quantCoeff >= (1<<shift) - offset =: rem, and since
// quantCoeff is always a positive constant (g_quantScales), that's
// abs(iLevel) >= ceil(rem/quantCoeff) =: levelThresh, i.e.
//   iLevel >= levelThresh || iLevel <= -levelThresh.
// levelThresh fits comfortably in int32 for the real shift range
// [13,29] (max ~34800). The ceil via (rem+quantCoeff-1)/quantCoeff is
// only proven correct here for rem>=0, which always holds in the real
// domain.
//
// This does one scalar division per call, then two vector compares per
// chunk. Comparing iLevel*quantCoeff against +-rem directly avoids the
// division at the cost of a per-element multiply instead; not used here
// because it is slower for the numCoeff values real call sites pass.
static bool needRdoqNeon( const TCoeff* pCoeff, size_t numCoeff, int quantCoeff, int64_t offset, int shift )
{
  CHECKD( numCoeff % 4 != 0, "numCoeff must be a multiple of four" );

  const int64_t  rem         = ( ( int64_t )1 << shift ) - offset;
  const int32_t  levelThresh = ( int32_t )( ( rem + quantCoeff - 1 ) / quantCoeff );

  const int32x4_t vposThresh = vdupq_n_s32( levelThresh );
  const int32x4_t vnegThresh = vdupq_n_s32( -levelThresh );

  auto survivors = [&]( int32x4_t c ) -> uint32x4_t {
    return vorrq_u32( vcgeq_s32( c, vposThresh ), vcleq_s32( c, vnegThresh ) );
  };

  size_t i = 0;
  for( ; i + 8 <= numCoeff; i += 8 )
  {
    const uint32x4_t s0 = survivors( vld1q_s32( pCoeff + i ) );
    const uint32x4_t s1 = survivors( vld1q_s32( pCoeff + i + 4 ) );

    if( any_lane_set_u32x4( vorrq_u32( s0, s1 ) ) )
      return true;
  }
  if( i < numCoeff )
  {
    const uint32x4_t s0 = survivors( vld1q_s32( pCoeff + i ) );
    if( any_lane_set_u32x4( s0 ) )
      return true;
  }
  return false;
}

// Mirrors DeQuantCore (Quant.cpp). Per coefficient:
//   q  = clip(piQCoef[x], -inputMaximum-1, inputMaximum)
//   v  = rightShift > 0 ? (q*scale + (1 << (rightShift-1))) >> rightShift
//                        :  q*scale * (1 << -rightShift)
//   piCoef[x] = clip(v, -transformMaximum-1, transformMaximum)
//
// vrshlq_s32 with a per-lane shift count of -rightShift reproduces both
// branches exactly: for rightShift>0 (negative count) SRSHL adds the same
// rounding half-unit before an arithmetic right shift; for rightShift<=0
// (non-negative count) it's a plain left shift, numerically the same as
// the scalar's unrounded multiplication by `1 << -rightShift`. TCoeffSig is
// int16, TCoeff/Intermediate_Int are int32, so a single widening multiply
// (vmull_n_s16) into 32-bit lanes replaces DeQuantCore's Intermediate_Int
// arithmetic; no 64-bit lanes are needed. Overflow check for the real
// (bitDepth, QP, shape) domain of this
// codebase: the most negative rightShift this encoder's parameter formulas
// can produce is -10, reached by an SBT-halved 2x2 chroma sub-block (e.g.
// an 8x4 -- or 4x8 -- inter CU in 4:2:0 has 4x2 -- or 2x4 -- chroma, and a
// single vertical- or horizontal-half SBT split halves that once more to
// 2x2; SBT cannot recurse into a second split of an already-SBT-split TU,
// so this is the only way to reach it) at QP_per's maximum (10 at 8-bit, 12
// at 10-bit). At that shift inputMaximum -- derived at the call site as
// (1 << (min(16, 25+rightShift) - 1)) - 1 -- is itself only 16383, not
// 32767, and clip(q, -inputMaximum-1, inputMaximum) admits the asymmetric
// minimum -16384, so |q*scale| <= 16384*102 < 2^21, and the largest left
// shift (10) keeps the shifted product under 2^31 - 1. inputMaximum only
// reaches its ordinary 32767 for rightShift >= -9, where the same argument
// (|q| <= 32768) holds a fortiori.
//
// Real call sites (Quant::dequant's !enableScalingLists path) always pass
// a width (maxX+1) that is 1 (degenerate ISP), 2 (e.g. 2x4/4x2/2x2 chroma),
// 4, or a multiple of 8 (ordinary transform sizes up to 64) -- never 3, 5,
// 6, or 7. Each width uses an exactly-sized tight load/store (no over-read
// or over-write).
//
// Each width gets its own top-level, non-overlapping loop (rather than a
// width check re-evaluated on every row inside one shared loop), and input
// clipping is applied once per load at whatever vector width that load
// already has the data in -- 4 lanes for width 1/2/4, 8 lanes for width>=8
// -- instead of being repeated separately on the low and high halves of an
// 8-lane load. width==1 additionally batches four rows (when available)
// into a single 4-lane vector, since piCoef's row stride is exactly one
// TCoeff there, so four scalar rows can share one multiply/shift/store
// instead of doing all three per row.
static void dequantNeon( const int maxX, const int maxY, const int scale, const TCoeffSig* const piQCoef,
                          const size_t piQCfStride, TCoeff* const piCoef, const int rightShift,
                          const int inputMaximum, const TCoeff transformMaximum )
{
  const int width = maxX + 1;
  CHECKD( !( width == 1 || width == 2 || width == 4 || ( width >= 8 && ( width & 7 ) == 0 ) ),
          "width must be 1, 2, 4, or a multiple of eight" );

  const int16x4_t vInputMax  = vdup_n_s16( ( int16_t )inputMaximum );
  const int16x4_t vInputMin  = vdup_n_s16( ( int16_t )( -inputMaximum - 1 ) );
  const int16x8_t vInputMaxQ = vdupq_n_s16( ( int16_t )inputMaximum );
  const int16x8_t vInputMinQ = vdupq_n_s16( ( int16_t )( -inputMaximum - 1 ) );
  const int32x4_t vShift     = vdupq_n_s32( -rightShift );
  const int32x4_t vTMax      = vdupq_n_s32( transformMaximum );
  const int32x4_t vTMin      = vdupq_n_s32( -transformMaximum - 1 );

  // Multiply/round-shift/output-clip only; callers input-clip q themselves,
  // once, before calling this.
  auto scaleShiftClip = [&]( int16x4_t q ) -> int32x4_t {
    int32x4_t v = vmull_n_s16( q, ( int16_t )scale );
    v           = vrshlq_s32( v, vShift );
    return vmaxq_s32( vminq_s32( v, vTMax ), vTMin );
  };

  if( width == 1 )
  {
    int y = 0;
    for( ; y + 4 <= maxY + 1; y += 4 )
    {
      int16x4_t q = vdup_n_s16( 0 );
      q = vld1_lane_s16( piQCoef + ( size_t )( y + 0 ) * piQCfStride, q, 0 );
      q = vld1_lane_s16( piQCoef + ( size_t )( y + 1 ) * piQCfStride, q, 1 );
      q = vld1_lane_s16( piQCoef + ( size_t )( y + 2 ) * piQCfStride, q, 2 );
      q = vld1_lane_s16( piQCoef + ( size_t )( y + 3 ) * piQCfStride, q, 3 );
      q = vmax_s16( vmin_s16( q, vInputMax ), vInputMin );
      vst1q_s32( piCoef + y, scaleShiftClip( q ) );
    }
    for( ; y <= maxY; y++ )
    {
      int16x4_t q = vld1_lane_s16( piQCoef + ( size_t )y * piQCfStride, vdup_n_s16( 0 ), 0 );
      q           = vmax_s16( vmin_s16( q, vInputMax ), vInputMin );
      vst1q_lane_s32( piCoef + y, scaleShiftClip( q ), 0 );
    }
    return;
  }

  if( width == 2 )
  {
    for( int y = 0; y <= maxY; y++ )
    {
      int16x4_t q = vmax_s16( vmin_s16( load_s16x2( piQCoef + y * piQCfStride ), vInputMax ), vInputMin );
      vst1_s32( piCoef + y * width, vget_low_s32( scaleShiftClip( q ) ) );
    }
    return;
  }

  if( width == 4 )
  {
    for( int y = 0; y <= maxY; y++ )
    {
      int16x4_t q = vmax_s16( vmin_s16( vld1_s16( piQCoef + y * piQCfStride ), vInputMax ), vInputMin );
      vst1q_s32( piCoef + y * width, scaleShiftClip( q ) );
    }
    return;
  }

  for( int y = 0; y <= maxY; y++ )
  {
    const TCoeffSig* src = piQCoef + y * piQCfStride;
    TCoeff*          dst = piCoef + y * width;

    for( int x = 0; x < width; x += 8 )
    {
      int16x8_t s = vld1q_s16( src + x );
      s           = vmaxq_s16( vminq_s16( s, vInputMaxQ ), vInputMinQ );
      vst1q_s32( dst + x, scaleShiftClip( vget_low_s16( s ) ) );
      vst1q_s32( dst + x + 4, scaleShiftClip( vget_high_s16( s ) ) );
    }
  }
}

template<>
void Quant::_initQuantARM<NEON>()
{
  xNeedRdoq = needRdoqNeon;
  xDeQuant  = dequantNeon;
}

}  // namespace vvenc

#endif  // TARGET_SIMD_ARM && ENABLE_SIMD_OPT_QUANT

//! \}

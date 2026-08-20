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
// with a 4-wide remainder step; a smaller remainder falls through to a
// plain scalar loop.
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
  if( i + 4 <= numCoeff )
  {
    const uint32x4_t s0 = survivors( vld1q_s32( pCoeff + i ) );
    if( any_lane_set_u32x4( s0 ) )
      return true;
    i += 4;
  }
  for( ; i < numCoeff; i++ )  // defensive: real call sites never leave a <4 remainder here
  {
    const TCoeff  iLevel   = pCoeff[i];
    const int64_t tmpLevel = ( int64_t ) std::abs( iLevel ) * quantCoeff;
    if( TCoeff( ( tmpLevel + offset ) >> shift ) != 0 )
      return true;
  }
  return false;
}

template<>
void Quant::_initQuantARM<NEON>()
{
  xNeedRdoq = needRdoqNeon;
}

}  // namespace vvenc

#endif  // TARGET_SIMD_ARM && ENABLE_SIMD_OPT_QUANT

//! \}

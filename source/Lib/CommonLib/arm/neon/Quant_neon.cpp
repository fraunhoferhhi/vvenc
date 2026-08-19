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
 * \brief Neon implementation of quantization functions for AArch64.
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

// Mirrors needRdoqCore (Quant.cpp): a reduction over the coefficient
// buffer that returns true as soon as any coefficient survives
// quantization. Real call sites always pass a numCoeff that is a
// multiple of four (efArea = width * min(height,32), both powers of
// two), so the bulk of the buffer is handled 8 coefficients at a time
// with a 4-wide remainder step; a smaller remainder falls through to a
// plain scalar loop.
//
// abs(coeff)*quantCoeff + offset is always non-negative in the real call
// domain (coeff is clipped to +-2^15, quantCoeff and offset are always
// positive), so vshlq_s64's arithmetic right shift and a logical shift
// would produce identical bits here.
static bool needRdoqNeon( const TCoeff* pCoeff, size_t numCoeff, int quantCoeff, int64_t offset, int shift )
{
  const int32x2_t vqnt   = vdup_n_s32( quantCoeff );
  const int64x2_t voff   = vdupq_n_s64( offset );
  const int64x2_t vshift = vdupq_n_s64( -(int64_t)shift );  // negative count -> right shift (SSHL)

  auto lane = [&]( int32x2_t c ) -> int64x2_t {
    return vshlq_s64( vaddq_s64( vmull_s32( c, vqnt ), voff ), vshift );
  };

  size_t i = 0;
  for( ; i + 8 <= numCoeff; i += 8 )
  {
    const int32x4_t c0 = vabsq_s32( vld1q_s32( pCoeff + i ) );
    const int32x4_t c1 = vabsq_s32( vld1q_s32( pCoeff + i + 4 ) );

    const int64x2_t l0 = lane( vget_low_s32( c0 ) );
    const int64x2_t l1 = lane( vget_high_s32( c0 ) );
    const int64x2_t l2 = lane( vget_low_s32( c1 ) );
    const int64x2_t l3 = lane( vget_high_s32( c1 ) );

    const uint64x2_t any = vreinterpretq_u64_s64( vorrq_s64( vorrq_s64( l0, l1 ), vorrq_s64( l2, l3 ) ) );
    if( vget_lane_u64( vorr_u64( vget_low_u64( any ), vget_high_u64( any ) ), 0 ) != 0 )
      return true;
  }
  if( i + 4 <= numCoeff )
  {
    const int32x4_t c0 = vabsq_s32( vld1q_s32( pCoeff + i ) );
    const uint64x2_t any = vreinterpretq_u64_s64( vorrq_s64( lane( vget_low_s32( c0 ) ), lane( vget_high_s32( c0 ) ) ) );
    if( vget_lane_u64( vorr_u64( vget_low_u64( any ), vget_high_u64( any ) ), 0 ) != 0 )
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

template <ARM_VEXT vext>
void Quant::_initQuantARM()
{
  xNeedRdoq = needRdoqNeon;
}

template void Quant::_initQuantARM<NEON>();

}  // namespace vvenc

#endif  // TARGET_SIMD_ARM && ENABLE_SIMD_OPT_QUANT

//! \}

/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

/** \file     sum_neon.h
    \brief    Helper functions for adding across vectors
*/

#pragma once

#include "CommonDef.h"

#if defined( TARGET_SIMD_ARM )

#include <arm_neon.h>

namespace vvenc
{

static inline int16_t horizontal_add_s16x8( const int16x8_t a )
{
#if REAL_TARGET_AARCH64
  return vaddvq_s16( a );
#else
  const int16x4_t b = vpadd_s16( vget_low_s16( a ), vget_high_s16( a ) );
  const int16x4_t c = vpadd_s16( b, b );
  const int16x4_t d = vpadd_s16( c, c );
  return vget_lane_s16( d, 0 );
#endif
}

static inline int horizontal_add_s32x4( const int32x4_t a )
{
#if REAL_TARGET_AARCH64
  return vaddvq_s32( a );
#else
  const int64x2_t b = vpaddlq_s32( a );
  const int32x2_t c = vadd_s32( vreinterpret_s32_s64( vget_low_s64( b ) ), vreinterpret_s32_s64( vget_high_s64( b ) ) );
  return vget_lane_s32( c, 0 );
#endif
}

static inline int64_t horizontal_add_s64x2( const int64x2_t a )
{
#if REAL_TARGET_AARCH64
  return vaddvq_s64( a );
#else
  return vgetq_lane_s64( a, 0 ) + vgetq_lane_s64( a, 1 );
#endif
}

static inline uint64_t horizontal_add_u64x2( const uint64x2_t a )
{
#if REAL_TARGET_AARCH64
  return vaddvq_u64( a );
#else
  return vgetq_lane_u64( a, 0 ) + vgetq_lane_u64( a, 1 );
#endif
}

static inline int horizontal_add_long_s16x8( const int16x8_t a )
{
#if REAL_TARGET_AARCH64
  return vaddlvq_s16( a );
#else
  return horizontal_add_s32x4( vpaddlq_s16( a ) );
#endif
}

static inline int64_t horizontal_add_long_s32x4( const int32x4_t a )
{
#if REAL_TARGET_AARCH64
  return vaddlvq_s32( a );
#else
  const int64x2_t b = vpaddlq_s32( a );
  return horizontal_add_s64x2( b );
#endif
}

static inline uint64_t horizontal_add_long_u32x4( const uint32x4_t a )
{
#if REAL_TARGET_AARCH64
  return vaddlvq_u32( a );
#else
  const uint64x2_t b = vpaddlq_u32( a );
  return horizontal_add_u64x2( b );
#endif
}

static inline int32x4_t horizontal_add_4d_s32x4( const int32x4_t v0, const int32x4_t v1, const int32x4_t v2,
                                                 const int32x4_t v3 )
{
#if REAL_TARGET_AARCH64
  int32x4_t v01 = vpaddq_s32( v0, v1 );
  int32x4_t v23 = vpaddq_s32( v2, v3 );
  return vpaddq_s32( v01, v23 );
#else
  int32x4_t res = vdupq_n_s32( 0 );
  res           = vsetq_lane_s32( horizontal_add_s32x4( v0 ), res, 0 );
  res           = vsetq_lane_s32( horizontal_add_s32x4( v1 ), res, 1 );
  res           = vsetq_lane_s32( horizontal_add_s32x4( v2 ), res, 2 );
  res           = vsetq_lane_s32( horizontal_add_s32x4( v3 ), res, 3 );
  return res;
#endif
}

static inline int32x4_t horizontal_add_long_4d_s16x8( const int16x8_t v0, const int16x8_t v1, const int16x8_t v2,
                                                      const int16x8_t v3 )
{
  return horizontal_add_4d_s32x4( vpaddlq_s16( v0 ), vpaddlq_s16( v1 ), vpaddlq_s16( v2 ), vpaddlq_s16( v3 ) );
}

static inline int16x8_t pairwise_add_s16x8( const int16x8_t a, const int16x8_t b )
{
#if REAL_TARGET_AARCH64
  return vpaddq_s16( a, b );
#else
  int16x4_t lo = vpadd_s16( vget_low_s16( a ), vget_high_s16( a ) );
  int16x4_t hi = vpadd_s16( vget_low_s16( b ), vget_high_s16( b ) );
  return vcombine_s16( lo, hi );
#endif
}

static inline int32x4_t pairwise_add_s32x4( const int32x4_t a, const int32x4_t b )
{
#if REAL_TARGET_AARCH64
  return vpaddq_s32( a, b );
#else
  int32x2_t lo = vpadd_s32( vget_low_s32( a ), vget_high_s32( a ) );
  int32x2_t hi = vpadd_s32( vget_low_s32( b ), vget_high_s32( b ) );
  return vcombine_s32( lo, hi );
#endif
}

static inline int64x2_t pairwise_add_s64x2( const int64x2_t a, const int64x2_t b )
{
#if REAL_TARGET_AARCH64
  return vpaddq_s64( a, b );
#else
  int64x1_t lo = vadd_s64( vget_low_s64( a ), vget_high_s64( a ) );
  int64x1_t hi = vadd_s64( vget_low_s64( b ), vget_high_s64( b ) );
  return vcombine_s64( lo, hi );
#endif
}

} // namespace vvenc

#endif

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

/** \file     mem_neon.h
    \brief    Helper functions for loading vectors
*/

#pragma once

#include "CommonDef.h"

#if defined( TARGET_SIMD_ARM )

#include <arm_neon.h>

namespace vvenc
{

// Load Helpers
static inline int16x4_t load_s16x2( const int16_t* src )
{
  int32_t tmp;
  memcpy( &tmp, src, 4 );
  return vreinterpret_s16_s32( vset_lane_s32( tmp, vdup_n_s32( 0 ), 0 ) );
}

static inline int16x4_t load_s16x2x2( const int16_t* src, ptrdiff_t stride )
{
  int32x2_t ret = vdup_n_s32( 0 );
  int32_t tmp0, tmp1;
  memcpy( &tmp0, src + 0 * stride, 4 );
  memcpy( &tmp1, src + 1 * stride, 4 );
  ret = vset_lane_s32( tmp0, ret, 0 );
  ret = vset_lane_s32( tmp1, ret, 1 );
  return vreinterpret_s16_s32( ret );
}

static inline void load_s16x4x6( const int16_t* src, const ptrdiff_t p, int16x4_t s[6] )
{
  s[0] = vld1_s16( src );
  src += p;
  s[1] = vld1_s16( src );
  src += p;
  s[2] = vld1_s16( src );
  src += p;
  s[3] = vld1_s16( src );
  src += p;
  s[4] = vld1_s16( src );
  src += p;
  s[5] = vld1_s16( src );
}

static inline void load_s16_16x8x4( const int16_t* src, const ptrdiff_t p, int16x8_t s[4] )
{
  s[0] = vld1q_s16( src );
  src += p;
  s[1] = vld1q_s16( src);
  src += p;
  s[2] = vld1q_s16( src );
  src += p;
  s[3] = vld1q_s16( src );
}

static inline void load_s16_16x8x6( const int16_t* src, const ptrdiff_t p, int16x8_t s[6] )
{
  s[0] = vld1q_s16( src );
  src += p;
  s[1] = vld1q_s16( src );
  src += p;
  s[2] = vld1q_s16( src );
  src += p;
  s[3] = vld1q_s16( src );
  src += p;
  s[4] = vld1q_s16( src );
  src += p;
  s[5] = vld1q_s16( src );
}

// Store Helpers
static inline void store_s16x2( int16_t* dst, int16x4_t src )
{
  int32_t tmp = vget_lane_s32( vreinterpret_s32_s16( src ), 0 );
  memcpy( dst, &tmp, 4 );
}

static inline void store_s16x2x2( int16_t* dst, int16x4_t src, ptrdiff_t stride )
{
  int32_t tmp0 = vget_lane_s32( vreinterpret_s32_s16( src ), 0 );
  int32_t tmp1 = vget_lane_s32( vreinterpret_s32_s16( src ), 1 );
  memcpy( dst + 0 * stride, &tmp0, 4 );
  memcpy( dst + 1 * stride, &tmp1, 4 );
}

} // namespace vvenc

#endif

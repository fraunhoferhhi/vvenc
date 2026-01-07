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

/** \file     reverse_neon.h
    \brief    Helper functions for reversing across vectors
*/

#pragma once

#include "CommonDef.h"

#if defined( TARGET_SIMD_ARM )

#include <arm_neon.h>

namespace vvenc
{

static inline int16x8_t reverse_vector_s16x8( int16x8_t x )
{
#if REAL_TARGET_AARCH64
  static const uint8_t shuffle_table[16] = { 14, 15, 12, 13, 10, 11, 8, 9, 6, 7, 4, 5, 2, 3, 0, 1 };
  const uint8x16_t shuffle_indices = vld1q_u8( shuffle_table );
  return vreinterpretq_s16_u8( vqtbl1q_u8( vreinterpretq_u8_s16( x ), shuffle_indices ) );
#else
  int16x8_t rev_halves = vrev64q_s16( x );
  return vcombine_s16( vget_high_s16( rev_halves ), vget_low_s16( rev_halves ) );
#endif
}

static inline int16x8_t load_deinterleave_reverse_s16x8( const int16_t* ptr )
{
#if REAL_TARGET_AARCH64
  uint8x16x2_t x;
  x.val[0] = vreinterpretq_u8_s16( vld1q_s16( ptr ) );
  x.val[1] = vreinterpretq_u8_s16( vld1q_s16( ptr + 8 ) );

  static const uint8_t shuffle_table[16] = { 30, 31, 26, 27, 22, 23, 18, 19, 14, 15, 10, 11, 6, 7, 2, 3 };
  const uint8x16_t shuffle_indices = vld1q_u8( shuffle_table );

  return vreinterpretq_s16_u8( vqtbl2q_u8( x, shuffle_indices ) );
#else
  return reverse_vector_s16x8( vld2q_s16( ptr ).val[1] );
#endif
}

static inline int16x4_t load_deinterleave_reverse_s16x4( const int16_t* ptr )
{
#if REAL_TARGET_AARCH64
  static const uint8_t shuffle_table[8] = { 14, 15, 10, 11, 6, 7, 2, 3 };
  const uint8x8_t shuffle_indices = vld1_u8( shuffle_table );

  int16x8_t x = vld1q_s16( ptr );
  return vreinterpret_s16_u8( vqtbl1_u8( vreinterpretq_u8_s16( x ), shuffle_indices ) );
#else
  return vrev64_s16( vld2_s16( ptr ).val[1] );
#endif
}

} // namespace vvenc

#endif

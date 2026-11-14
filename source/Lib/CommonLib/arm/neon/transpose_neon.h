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

/** \file     transpose_neon.h
    \brief    Helper functions for transposing vectors
*/

#pragma once

#include "CommonLib/CommonDef.h"

#if defined( TARGET_SIMD_ARM )

#include <arm_neon.h>

namespace vvenc
{
// Transposes the two 16-bit 4x4 blocks held in the low and high halves of a0..a3 in place: afterwards the low half of
// a[j] is column j of the low block, its high half column j of the high block.
static inline void transpose_4x4_s16_halves( int16x8_t& a0, int16x8_t& a1, int16x8_t& a2, int16x8_t& a3 )
{
  // t01.val[0]: a0[0] a1[0] a0[2] a1[2] ..., t01.val[1]: a0[1] a1[1] a0[3] a1[3] ...; t23 alike
  const int16x8x2_t t01 = vtrnq_s16( a0, a1 );
  const int16x8x2_t t23 = vtrnq_s16( a2, a3 );

  // c02.val[0]: lanes 0 and 4 of a0..a3, c02.val[1]: lanes 2 and 6; c13: lanes 1 and 5, lanes 3 and 7
  const int32x4x2_t c02 = vtrnq_s32( vreinterpretq_s32_s16( t01.val[0] ), vreinterpretq_s32_s16( t23.val[0] ) );
  const int32x4x2_t c13 = vtrnq_s32( vreinterpretq_s32_s16( t01.val[1] ), vreinterpretq_s32_s16( t23.val[1] ) );

  a0 = vreinterpretq_s16_s32( c02.val[0] );
  a1 = vreinterpretq_s16_s32( c13.val[0] );
  a2 = vreinterpretq_s16_s32( c02.val[1] );
  a3 = vreinterpretq_s16_s32( c13.val[1] );
}

static inline void transpose_concat_8x4_s16( int16x8_t a0, int16x8_t a1, int16x8_t a2, int16x8_t a3,
                                             int16x8_t& b0, int16x8_t& b1, int16x8_t& b2, int16x8_t& b3 )
{
  // Transpose 16-bit 8x4 and concatenate result as follows:
  // a0: 00 01 02 03 04 05 06 07
  // a1: 10 11 12 13 14 15 16 17
  // a2: 20 21 22 23 24 25 26 27
  // a3: 30 31 32 33 34 35 36 37

  // 00 20 01 21 02 22 03 23
  // 04 24 05 25 06 26 07 27
  int16x8x2_t a02 = vzipq_s16( a0, a2 );
  // 10 30 11 31 12 32 13 33
  // 14 34 15 35 16 36 17 37
  int16x8x2_t a13 = vzipq_s16( a1, a3 );

  // b0: 00 10 20 30 01 11 21 31
  // b1: 02 12 22 32 03 13 23 33
  // b2: 04 14 24 34 05 15 25 35
  // b3: 06 16 26 36 07 17 27 37
  int16x8x2_t b01 = vzipq_s16( a02.val[0], a13.val[0] );
  int16x8x2_t b23 = vzipq_s16( a02.val[1], a13.val[1] );

  b0 = b01.val[0];
  b1 = b01.val[1];
  b2 = b23.val[0];
  b3 = b23.val[1];
}

static inline int16x8x2_t vvenc_vtrnq_s64_to_s16( int16x8_t a0, int16x8_t a1 )
{
  int16x8x2_t b0;
#if REAL_TARGET_AARCH64
  b0.val[0] = vreinterpretq_s16_s64( vtrn1q_s64( vreinterpretq_s64_s16( a0 ), vreinterpretq_s64_s16( a1 ) ) );
  b0.val[1] = vreinterpretq_s16_s64( vtrn2q_s64( vreinterpretq_s64_s16( a0 ), vreinterpretq_s64_s16( a1 ) ) );
#else
  b0.val[0] = vcombine_s16( vget_low_s16( a0 ), vget_low_s16( a1 ) );
  b0.val[1] = vcombine_s16( vget_high_s16( a0 ), vget_high_s16( a1 ) );
#endif
  return b0;
}

} // namespace vvenc

#endif

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

static inline uint16x8x2_t vvenc_vtrnq_u64_to_u16( uint32x4_t a0, uint32x4_t a1 )
{
  uint16x8x2_t b0;
#if REAL_TARGET_AARCH64
  b0.val[0] = vreinterpretq_u16_u64( vtrn1q_u64( vreinterpretq_u64_u32( a0 ), vreinterpretq_u64_u32( a1 ) ) );
  b0.val[1] = vreinterpretq_u16_u64( vtrn2q_u64( vreinterpretq_u64_u32( a0 ), vreinterpretq_u64_u32( a1 ) ) );
#else
  b0.val[0] = vreinterpretq_u16_u32( vcombine_u32( vget_low_u32( a0 ), vget_low_u32( a1 ) ) );
  b0.val[1] = vreinterpretq_u16_u32( vcombine_u32( vget_high_u32( a0 ), vget_high_u32( a1 ) ) );
#endif
  return b0;
}

// Transposes rows[r][c] (8 x 8 16-bit values) into columns[c][r]: the 16-bit vtrn pairs rows 0-1, 2-3, ..., the
// 32-bit vtrn those pairs within rows 0-3 and 4-7, and the 64-bit halves of rows 0-3 and 4-7 then combine into one
// column each.
static inline void transpose_8x8_u16( const uint16x8_t* rows, uint16x8_t* columns )
{
  const uint16x8x2_t r01 = vtrnq_u16( rows[0], rows[1] );
  const uint16x8x2_t r23 = vtrnq_u16( rows[2], rows[3] );
  const uint16x8x2_t r45 = vtrnq_u16( rows[4], rows[5] );
  const uint16x8x2_t r67 = vtrnq_u16( rows[6], rows[7] );

  // e.g. top02.val[0] holds column 0 (rows 0-3) then column 4 (rows 0-3), top02.val[1] columns 2 and 6
  const uint32x4x2_t top02    = vtrnq_u32( vreinterpretq_u32_u16( r01.val[0] ), vreinterpretq_u32_u16( r23.val[0] ) );
  const uint32x4x2_t top13    = vtrnq_u32( vreinterpretq_u32_u16( r01.val[1] ), vreinterpretq_u32_u16( r23.val[1] ) );
  const uint32x4x2_t bottom02 = vtrnq_u32( vreinterpretq_u32_u16( r45.val[0] ), vreinterpretq_u32_u16( r67.val[0] ) );
  const uint32x4x2_t bottom13 = vtrnq_u32( vreinterpretq_u32_u16( r45.val[1] ), vreinterpretq_u32_u16( r67.val[1] ) );

  const uint16x8x2_t c04 = vvenc_vtrnq_u64_to_u16( top02.val[0], bottom02.val[0] );
  const uint16x8x2_t c15 = vvenc_vtrnq_u64_to_u16( top13.val[0], bottom13.val[0] );
  const uint16x8x2_t c26 = vvenc_vtrnq_u64_to_u16( top02.val[1], bottom02.val[1] );
  const uint16x8x2_t c37 = vvenc_vtrnq_u64_to_u16( top13.val[1], bottom13.val[1] );

  columns[0] = c04.val[0];
  columns[1] = c15.val[0];
  columns[2] = c26.val[0];
  columns[3] = c37.val[0];
  columns[4] = c04.val[1];
  columns[5] = c15.val[1];
  columns[6] = c26.val[1];
  columns[7] = c37.val[1];
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

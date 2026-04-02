/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

/** \file     neon_sve2_bridge.h
    \brief    Helper functions for SVE2
*/

#pragma once

#if defined( TARGET_SIMD_ARM )

#include <arm_neon_sve_bridge.h>

namespace vvenc
{
// We can access instructions exclusive to the SVE2 instruction set from a
// predominantly Neon context by making use of the Neon-SVE2 bridge intrinsics
// to reinterpret Neon vectors as SVE2 vectors - with the high part of the SVE2
// vector (if it's longer than 128 bits) being "don't care".

// While sub-optimal on machines that have SVE2 vector length > 128-bit - as the
// remainder of the vector is unused - this approach is still beneficial when
// compared to a Neon-only solution.

template<uint64_t Rotation>
static inline int16x8_t vvenc_cadd_s16( int16x8_t a, int16x8_t b )
{
  return svget_neonq_s16(
      svcadd_s16( svset_neonq_s16( svundef_s16(), a ), svset_neonq_s16( svundef_s16(), b ), Rotation ) );
}

static inline int16x8_t vvenc_svtbl2_s16( int16x8_t data0, int16x8_t data1, uint16x8_t indices )
{
  return svget_neonq_s16(
      svtbl2_s16( svcreate2_s16( svset_neonq_s16( svundef_s16(), data0 ), svset_neonq_s16( svundef_s16(), data1 ) ),
                  svset_neonq_u16( svundef_u16(), indices ) ) );
}

} // namespace vvenc

#endif

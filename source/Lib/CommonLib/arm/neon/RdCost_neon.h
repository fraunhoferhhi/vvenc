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

/** \file     RdCost_neon.h
    \brief    RD cost computation class, Neon version
*/

#pragma once

#include "CommonLib/CommonDef.h"
#include "CommonLib/RdCost.h"

namespace vvenc
{

#if ENABLE_SIMD_OPT_DIST && defined( TARGET_SIMD_ARM )

template<int width>
Distortion xGetHADs_neon( const DistParam& rcDtParam );

Distortion xCalcHAD8x8_neon( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                             const ptrdiff_t iStrideCur );

Distortion xCalcHAD16x8_neon( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                              const ptrdiff_t iStrideCur );

Distortion xCalcHAD8x16_neon( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                              const ptrdiff_t iStrideCur );

Distortion xCalcHAD8x4_neon( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                             const ptrdiff_t iStrideCur );

Distortion xCalcHAD4x8_neon( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                             const ptrdiff_t iStrideCur );

Distortion xCalcHAD4x4_neon( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                             const ptrdiff_t iStrideCur );

// The normalization ( uint32_t )( sad / sqrt( n ) * 2 ) of the n = 16 * 8 and n = 4 * 8 SATDs without the double
// division: ( sad * M ) >> 42 with M = ceil( 2^43 / c ), c the double sqrt( n ). The result is identical for every sad
// below 7095941 (checked exhaustively), and these SATDs stay far below: the transform is orthogonal up to sqrt( n ), so
// the sum of absolute coefficients is at most n * sqrt( n ) * 2047 < 3 million.
static constexpr uint32_t HAD_SCALE_MAX_SAD   = 7095940;
static constexpr uint64_t HAD_SCALE_16x8_MULT = 777472127994ull;    // n = 128
static constexpr uint64_t HAD_SCALE_8x4_MULT  = 1554944255988ull;   // n = 32

static inline uint32_t xScaleHAD( const uint32_t sad, const uint64_t mult )
{
  CHECKD( sad > HAD_SCALE_MAX_SAD, "SATD outside the range of the fixed-point normalization" );
  return uint32_t( ( uint64_t( sad ) * mult ) >> 42 );
}

#endif // defined( TARGET_SIMD_ARM )

} // namespace vvenc

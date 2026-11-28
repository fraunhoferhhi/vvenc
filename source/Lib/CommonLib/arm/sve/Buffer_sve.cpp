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
/** \file     Buffer_sve.cpp
    \brief    SIMD for PelBufferOps, SVE version
*/

#include "../Buffer_neon.h"
#include "../neon/sum_neon.h"
#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"
#include "neon_sve_bridge.h"

#include <arm_sve.h>

namespace vvenc
{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER

// the squares of the differences in 64-bit lanes, each vector's squares added in one 16-bit dot product (exact: at
// most 2^32 per lane and step)
struct SquareSumSve
{
  int64x2_t acc = vdupq_n_s64( 0 );

  void add( const int16x4_t a, const int16x4_t b )
  {
    const int16x8_t diff = vcombine_s16( vsub_s16( a, b ), vdup_n_s16( 0 ) );
    acc = vvenc_sdotq_s16( acc, diff, diff );
  }
  void add( const int16x8_t a, const int16x8_t b )
  {
    const int16x8_t diff = vsubq_s16( a, b );
    acc = vvenc_sdotq_s16( acc, diff, diff );
  }
  void add( const SquareSumSve& other ) { acc = vaddq_s64( acc, other.acc ); }
  uint64_t sum() const { return uint64_t( horizontal_add_s64x2( acc ) ); }
};

static uint64_t sumSquaredError_sve( const Pel* src0, const ptrdiff_t stride0, const Pel* src1, const ptrdiff_t stride1,
                                     const int width, const int height )
{
  return sumSquaredError_common<SquareSumSve>( src0, stride0, src1, stride1, width, height );
}

template<>
void PelBufferOps::_initPelBufOpsARM<SVE>()
{
  sumSquaredError = sumSquaredError_sve;
}

#endif // defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BUFFER

} // namespace vvenc

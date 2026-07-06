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
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

/** \file     FGA_neon.cpp
    \brief    film grain analysis, Neon version
*/

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "EncoderLib/SEIFilmGrainAnalyzer.h"
#include "sum_neon.h"

#include <arm_neon.h>

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_FGA

namespace vvenc
{

// Mirrors calcMeanSse: returns the integer sum of all w*h pixels (w is a
// multiple of 8). The caller divides by the block size.
static int calcMeanNeon( const Pel* org, const ptrdiff_t origStride, const int w, const int h )
{
  // Two independent accumulators allow out-of-order execution to overlap
  // the add chains. vpadalq_s16 folds the pairwise widen and accumulate
  // into a single instruction.
  int32x4_t acc0 = vdupq_n_s32( 0 );
  int32x4_t acc1 = vdupq_n_s32( 0 );
  for( int y = 0; y < h; y++, org += origStride )
  {
    int x = 0;
    for( ; x < w - 8; x += 16 )
    {
      acc0 = vpadalq_s16( acc0, vld1q_s16( org + x ) );
      acc1 = vpadalq_s16( acc1, vld1q_s16( org + x + 8 ) );
    }
    for( ; x < w; x += 8 )
      acc0 = vpadalq_s16( acc0, vld1q_s16( org + x ) );
  }
  return horizontal_add_s32x4( vaddq_s32( acc0, acc1 ) );
}

#if ENABLE_SIMD_OPT_MCTF
// Defined in MCTF_neon.cpp and reused here, mirroring how the x86 build
// shares calcVarSse between MCTF and the film grain analyzer.
double calcVar_neon( const Pel* org, const ptrdiff_t origStride, const int w, const int h );
#endif

template <ARM_VEXT vext>
void FGAnalyzer::_initFGAnalyzerARM()
{
#if ENABLE_SIMD_OPT_MCTF
  calcVar  = calcVar_neon;
#endif
  calcMean = calcMeanNeon;
}

template void FGAnalyzer::_initFGAnalyzerARM<NEON>();

}  // namespace vvenc

#endif  // TARGET_SIMD_ARM && ENABLE_SIMD_OPT_FGA

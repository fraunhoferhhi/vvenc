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
#include "CommonLib/Rom.h"
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

// Mirrors calcVarSse: integer accumulation throughout, single final divide.
static double calcVarNeon( const Pel* org, const ptrdiff_t origStride, const int w, const int h )
{
  // Pass 1: mean — same dual-accumulator + vpadalq_s16 pattern as calcMean.
  int32x4_t avgAcc0 = vdupq_n_s32( 0 );
  int32x4_t avgAcc1 = vdupq_n_s32( 0 );
  const Pel* p      = org;
  for( int y = 0; y < h; y++, p += origStride )
  {
    int x = 0;
    for( ; x < w - 8; x += 16 )
    {
      avgAcc0 = vpadalq_s16( avgAcc0, vld1q_s16( p + x ) );
      avgAcc1 = vpadalq_s16( avgAcc1, vld1q_s16( p + x + 8 ) );
    }
    for( ; x < w; x += 8 )
      avgAcc0 = vpadalq_s16( avgAcc0, vld1q_s16( p + x ) );
  }
  const int shift    = Log2( w ) + Log2( h ) - 4;
  const int avg      = horizontal_add_s32x4( vaddq_s32( avgAcc0, avgAcc1 ) ) >> shift;
  // Match _mm_packs_epi32 saturation to the int16 range.
  const int16x8_t vavg = vdupq_n_s16( (int16_t)Clip3( -32768, 32767, avg ) );

  // Pass 2: variance — dual int64 accumulators; vpadalq_s32 pairwise-adds
  // adjacent squared products and widens to int64 in a single instruction.
  int64x2_t var0 = vdupq_n_s64( 0 );
  int64x2_t var1 = vdupq_n_s64( 0 );
  p = org;
  for( int y = 0; y < h; y++, p += origStride )
  {
    for( int x = 0; x < w; x += 8 )
    {
      int16x8_t pix = vshlq_n_s16( vld1q_s16( p + x ), 4 );
      pix           = vsubq_s16( pix, vavg );
      const int32x4_t lo = vmull_s16( vget_low_s16( pix ), vget_low_s16( pix ) );
      const int32x4_t hi = vmull_s16( vget_high_s16( pix ), vget_high_s16( pix ) );
      var0 = vpadalq_s32( var0, lo );
      var1 = vpadalq_s32( var1, hi );
    }
  }

  return horizontal_add_s64x2( vaddq_s64( var0, var1 ) ) / 256.0;
}

template <ARM_VEXT vext>
void FGAnalyzer::_initFGAnalyzerARM()
{
  calcVar  = calcVarNeon;
  calcMean = calcMeanNeon;
}

template void FGAnalyzer::_initFGAnalyzerARM<NEON>();

}  // namespace vvenc

#endif  // TARGET_SIMD_ARM && ENABLE_SIMD_OPT_FGA

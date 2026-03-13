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

/** \file     SampleAdaptiveOffset_neon.cpp
    \brief    Sample adaptive offset class, Neon version of calcSaoStatisticsBo.
*/

#include "../CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include <arm_neon.h>

namespace vvenc
{

#if ENABLE_SIMD_OPT_SAO && defined( TARGET_SIMD_ARM )

void calcSaoStatisticsBo_neon( int width, int endX, int endY, Pel* srcLine, Pel* orgLine, int srcStride, int orgStride,
                               int channelBitDepth, int64_t* count, int64_t* diff )
{
  const int shift = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;

  int32_t countTmp[2][32] = { { 0 }, { 0 } };
  int32_t diffTmp[2][32] = { { 0 }, { 0 } };

  do
  {
    for( int x = 0; x < endX; ++x )
    {
      int idx = x & 1;
      int bandIdx = srcLine[x] >> shift;
      diffTmp[idx][bandIdx] += orgLine[x] - srcLine[x];
      countTmp[idx][bandIdx]++;
    }

    srcLine += srcStride;
    orgLine += orgStride;
  } while( --endY != 0 );

  for( int i = 0; i < 32; i += 4 )
  {
    int32x4_t c0 = vld1q_s32( countTmp[0] + i );
    int32x4_t c1 = vld1q_s32( countTmp[1] + i );
    int32x4_t d0 = vld1q_s32( diffTmp[0] + i );
    int32x4_t d1 = vld1q_s32( diffTmp[1] + i );

    int64x2_t cTotal_lo = vaddl_s32( vget_low_s32( c0 ), vget_low_s32( c1 ) );
    int64x2_t cTotal_hi = vaddl_s32( vget_high_s32( c0 ), vget_high_s32( c1 ) );
    int64x2_t dTotal_lo = vaddl_s32( vget_low_s32( d0 ), vget_low_s32( d1 ) );
    int64x2_t dTotal_hi = vaddl_s32( vget_high_s32( d0 ), vget_high_s32( d1 ) );

    vst1q_s64( count + i + 0, cTotal_lo );
    vst1q_s64( count + i + 2, cTotal_hi );
    vst1q_s64( diff + i + 0, dTotal_lo );
    vst1q_s64( diff + i + 2, dTotal_hi );
  }
}

template<>
void SampleAdaptiveOffset::_initSampleAdaptiveOffsetARM<NEON>()
{
  calcSaoStatisticsBo = calcSaoStatisticsBo_neon;
}

#endif // ENABLE_SIMD_OPT_SAO && defined( TARGET_SIMD_ARM )

} // namespace vvenc

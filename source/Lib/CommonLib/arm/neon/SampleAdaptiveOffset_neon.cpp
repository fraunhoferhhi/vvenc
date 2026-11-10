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

// One sample of the band-offset statistics in a packed table entry: the entry holds count * 2^32 + diffSum.
static inline void addToBand( int64_t* table, const int band, const int diff )
{
  table[band] += ( int64_t( 1 ) << 32 ) + diff;
}

// Lane j (0 to 3) of four int16 values packed in a uint64
static inline int lane16( const uint64_t lanes, const int j )
{
  return int16_t( lanes >> ( 16 * j ) );
}

void calcSaoStatisticsBo_neon( int width, int endX, int endY, Pel* srcLine, Pel* orgLine, int srcStride, int orgStride,
                               int channelBitDepth, int64_t* count, int64_t* diff )
{
  const int shift = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
  const int16x8_t negShift = vdupq_n_s16( -shift );

  // Table k takes the samples at x % 8 == k; the tables are summed at the end. Each entry packs count and diffSum: at
  // most 128 x 128 samples with |org - src| < 2^15 keep |diffSum| < 2^29, so the low 32 bits read as a signed integer
  // are diffSum and the rest is the count.
  int64_t packed[8][NUM_SAO_BO_CLASSES] = {};

  do
  {
    int x = 0;
    for( ; x + 8 <= endX; x += 8 )
    {
      const int16x8_t src = vld1q_s16( srcLine + x );
      const int16x8_t org = vld1q_s16( orgLine + x );
      const int16x8_t band = vshlq_s16( src, negShift );
      const int16x8_t d = vsubq_s16( org, src );

      // the differences are moved to general registers as two uint64 and taken from them with shifts, the bands lane
      // by lane
      const uint64_t diffLo = vgetq_lane_u64( vreinterpretq_u64_s16( d ), 0 );
      const uint64_t diffHi = vgetq_lane_u64( vreinterpretq_u64_s16( d ), 1 );

      addToBand( packed[0], vgetq_lane_s16( band, 0 ), lane16( diffLo, 0 ) );
      addToBand( packed[1], vgetq_lane_s16( band, 1 ), lane16( diffLo, 1 ) );
      addToBand( packed[2], vgetq_lane_s16( band, 2 ), lane16( diffLo, 2 ) );
      addToBand( packed[3], vgetq_lane_s16( band, 3 ), lane16( diffLo, 3 ) );
      addToBand( packed[4], vgetq_lane_s16( band, 4 ), lane16( diffHi, 0 ) );
      addToBand( packed[5], vgetq_lane_s16( band, 5 ), lane16( diffHi, 1 ) );
      addToBand( packed[6], vgetq_lane_s16( band, 6 ), lane16( diffHi, 2 ) );
      addToBand( packed[7], vgetq_lane_s16( band, 7 ), lane16( diffHi, 3 ) );
    }
    for( ; x < endX; ++x )
    {
      addToBand( packed[0], srcLine[x] >> shift, orgLine[x] - srcLine[x] );
    }

    srcLine += srcStride;
    orgLine += orgStride;
  } while( --endY != 0 );

  for( int i = 0; i < NUM_SAO_BO_CLASSES; i += 2 )
  {
    int64x2_t sum = vld1q_s64( packed[0] + i );
    for( int k = 1; k < 8; k++ )
    {
      const int64x2_t next = vld1q_s64( packed[k] + i );
      sum = vaddq_s64( sum, next );
    }

    const int64x2_t diffSum = vmovl_s32( vmovn_s64( sum ) );
    const int64x2_t bandCount = vshrq_n_s64( vsubq_s64( sum, diffSum ), 32 );

    vst1q_s64( diff + i, diffSum );
    vst1q_s64( count + i, bandCount );
  }
}

template<>
void SampleAdaptiveOffset::_initSampleAdaptiveOffsetARM<NEON>()
{
  calcSaoStatisticsBo = calcSaoStatisticsBo_neon;
}

#endif // ENABLE_SIMD_OPT_SAO && defined( TARGET_SIMD_ARM )

} // namespace vvenc

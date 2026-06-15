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
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

/**
 * \file EncAdaptiveLoopFilter_sve.cpp
 * \brief SVE helpers for encoder-side adaptive loop filter code.
 */

#include <arm_neon.h>
#include <arm_sve.h>

#include "EncoderLib/EncAdaptiveLoopFilter.h"
#include "CommonLib/arm/neon/sum_neon.h"
#include "CommonLib/arm/sve/neon_sve_bridge.h"

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_ALF

namespace vvenc
{

static inline int64x2_t dot_s16x8x2_sve( const int16x8_t v1, const int16x8_t v2, const int16x8_t v3,
                                         const int16x8_t v4 )
{
  int64x2_t sum = vvenc_sdotq_s16( vdupq_n_s64( 0 ), v1, v2 );
  sum = vvenc_sdotq_s16( sum, v3, v4 );
  return sum;
}

template<int NumBins, int NumCoeff>
void getPreBlkStatsAccumFixedSize_sve( AlfCovariance& alfCovariance, const Pel* ELocal, const Pel yLocal[4][4] )
{
  static constexpr int bstride = MAX_NUM_ALF_LUMA_COEFF << 4;
  static constexpr int kstride = 1 << 4;

  const int16x8_t y0 = vld1q_s16( &yLocal[0][0] );
  const int16x8_t y1 = vld1q_s16( &yLocal[2][0] );

  auto y = alfCovariance.y;
  auto E = alfCovariance.E;

  for( int b0 = 0; b0 < NumBins; b0++ )
  {
    // ELocal is one flat buffer, but logically it is ELocal[NumBins][MAX_NUM_ALF_LUMA_COEFF][16].
    const Pel* Elocalk = ELocal + b0 * bstride;

    for( int k = 0; k < NumCoeff; k++ )
    {
      const int16x8_t ek0 = vld1q_s16( Elocalk + 0 );
      const int16x8_t ek1 = vld1q_s16( Elocalk + 8 );

      for( int b1 = 0; b1 < NumBins; b1++ )
      {
        alf_float_t* cov = &E[b0][b1][k][k];
        const Pel* Elocall = ELocal + b1 * bstride + k * kstride;

        int l = k;

        for( ; l < NumCoeff - 3; l += 4 )
        {
          const int16x8_t el00 = vld1q_s16( Elocall + 0 * 16 + 0 );
          const int16x8_t el10 = vld1q_s16( Elocall + 1 * 16 + 0 );
          const int16x8_t el20 = vld1q_s16( Elocall + 2 * 16 + 0 );
          const int16x8_t el30 = vld1q_s16( Elocall + 3 * 16 + 0 );
          const int16x8_t el01 = vld1q_s16( Elocall + 0 * 16 + 8 );
          const int16x8_t el11 = vld1q_s16( Elocall + 1 * 16 + 8 );
          const int16x8_t el21 = vld1q_s16( Elocall + 2 * 16 + 8 );
          const int16x8_t el31 = vld1q_s16( Elocall + 3 * 16 + 8 );

          const int64x2_t sum0 = dot_s16x8x2_sve( el00, ek0, el01, ek1 );
          const int64x2_t sum1 = dot_s16x8x2_sve( el10, ek0, el11, ek1 );
          const int64x2_t sum2 = dot_s16x8x2_sve( el20, ek0, el21, ek1 );
          const int64x2_t sum3 = dot_s16x8x2_sve( el30, ek0, el31, ek1 );

          const int64x2_t sum01 = pairwise_add_s64x2( sum0, sum1 );
          const int64x2_t sum23 = pairwise_add_s64x2( sum2, sum3 );
          const int32x4_t sum = vuzp1q_s32( vreinterpretq_s32_s64( sum01 ), vreinterpretq_s32_s64( sum23 ) );

          float32x4_t vCov = vld1q_f32( cov );
          vCov = vaddq_f32( vcvtq_f32_s32( sum ), vCov );
          vst1q_f32( cov, vCov );

          cov += 4;
          Elocall += 4 * 16;
        }

        for( ; l < NumCoeff; l++ )
        {
          const int16x8_t el0 = vld1q_s16( Elocall + 0 );
          const int16x8_t el1 = vld1q_s16( Elocall + 8 );

          *cov++ += horizontal_add_s64x2( dot_s16x8x2_sve( el0, ek0, el1, ek1 ) );
          Elocall += 16;
        }
      }

      y[b0][k] += horizontal_add_s64x2( dot_s16x8x2_sve( ek0, y0, ek1, y1 ) );

      Elocalk += 16;
    }
  }

  alfCovariance.pixAcc += horizontal_add_s64x2( dot_s16x8x2_sve( y0, y0, y1, y1 ) );
}

void getPreBlkStatsAccum_sve( AlfCovariance& alfCovariance, const AlfFilterShape& shape, const Pel* ELocal,
                              const Pel yLocal[4][4], const int numBins )
{
  if( shape.numCoeff == 7 )
  {
    if( numBins == 1 )
    {
      return getPreBlkStatsAccumFixedSize_sve</*NumBins=*/1, /*NumCoeff=*/7>( alfCovariance, ELocal, yLocal );
    }
    else if( numBins == 4 )
    {
      return getPreBlkStatsAccumFixedSize_sve</*NumBins=*/4, /*NumCoeff=*/7>( alfCovariance, ELocal, yLocal );
    }
  }
  else if( shape.numCoeff == 13 )
  {
    if( numBins == 1 )
    {
      return getPreBlkStatsAccumFixedSize_sve</*NumBins=*/1, /*NumCoeff=*/13>( alfCovariance, ELocal, yLocal );
    }
    else if( numBins == 4 )
    {
      return getPreBlkStatsAccumFixedSize_sve</*NumBins=*/4, /*NumCoeff=*/13>( alfCovariance, ELocal, yLocal );
    }
  }
  CHECK( true, "Unsupported ALF filter shape or number of bins" );
}

template<>
void EncAdaptiveLoopFilter::_initEncAdaptiveLoopFilterARM<SVE>()
{
  m_getPreBlkStatsAccum = getPreBlkStatsAccum_sve;
}

} // namespace vvenc

#endif

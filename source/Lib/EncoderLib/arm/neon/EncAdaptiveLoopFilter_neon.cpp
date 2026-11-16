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

/**
 * \file EncAdaptiveLoopFilter_neon.cpp
 * \brief Neon helpers for encoder-side adaptive loop filter code.
 */

#include "EncoderLib/EncAdaptiveLoopFilter.h"
#include "EncoderLib/arm/EncAdaptiveLoopFilter_neon.h"
#include "CommonLib/arm/neon/sum_neon.h"
#include "CommonLib/arm/mem_neon.h"
#include <arm_neon.h>

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_ALF

namespace vvenc
{

static inline int32x4_t dot_s16x8x2_neon( const int16x8_t v1, const int16x8_t v2, const int16x8_t v3,
                                          const int16x8_t v4 )
{
  int32x4_t sum = vmull_s16( vget_low_s16( v1 ), vget_low_s16( v2 ) );
  sum = vmlal_s16( sum, vget_high_s16( v1 ), vget_high_s16( v2 ) );
  sum = vmlal_s16( sum, vget_low_s16( v3 ), vget_low_s16( v4 ) );
  sum = vmlal_s16( sum, vget_high_s16( v3 ), vget_high_s16( v4 ) );
  return sum;
}

// The dot products of the 16 samples ek0 | ek1 with each of the four rows starting at rows (16 samples apart).
static inline int32x4_t rowDots4_neon( const int16x8_t ek0, const int16x8_t ek1, const Pel* rows )
{
  const int16x8_t el00 = vld1q_s16( rows + 0 * 16 + 0 );
  const int16x8_t el01 = vld1q_s16( rows + 0 * 16 + 8 );
  const int16x8_t el10 = vld1q_s16( rows + 1 * 16 + 0 );
  const int16x8_t el11 = vld1q_s16( rows + 1 * 16 + 8 );
  const int16x8_t el20 = vld1q_s16( rows + 2 * 16 + 0 );
  const int16x8_t el21 = vld1q_s16( rows + 2 * 16 + 8 );
  const int16x8_t el30 = vld1q_s16( rows + 3 * 16 + 0 );
  const int16x8_t el31 = vld1q_s16( rows + 3 * 16 + 8 );

  const int32x4_t sum0 = dot_s16x8x2_neon( el00, ek0, el01, ek1 );
  const int32x4_t sum1 = dot_s16x8x2_neon( el10, ek0, el11, ek1 );
  const int32x4_t sum2 = dot_s16x8x2_neon( el20, ek0, el21, ek1 );
  const int32x4_t sum3 = dot_s16x8x2_neon( el30, ek0, el31, ek1 );

  return horizontal_add_4d_s32x4( sum0, sum1, sum2, sum3 );
}

template<int NumBins, int NumCoeff>
void getPreBlkStatsAccumFixedSize_neon( AlfCovariance& alfCovariance, const Pel* ELocal, const Pel yLocal[4][4] )
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

      y[b0][k] += horizontal_add_s32x4( dot_s16x8x2_neon( ek0, y0, ek1, y1 ) );

      for( int b1 = 0; b1 < NumBins; b1++ )
      {
        alf_float_t* cov = &E[b0][b1][k][k];
        const Pel* Elocall = ELocal + b1 * bstride + k * kstride;

        int l = k;

        for( ; l < NumCoeff - 3; l += 4 )
        {
          float32x4_t vCov = vld1q_f32( cov );
          const int32x4_t vSum = rowDots4_neon( ek0, ek1, Elocall );
          vCov = vaddq_f32( vcvtq_f32_s32( vSum ), vCov );
          vst1q_f32( cov, vCov );

          cov += 4;
          Elocall += 4 * 16;
        }

        for( ; l < NumCoeff; l++ )
        {
          const int16x8_t el0 = vld1q_s16( Elocall + 0 );
          const int16x8_t el1 = vld1q_s16( Elocall + 8 );

          *cov++ += horizontal_add_s32x4( dot_s16x8x2_neon( el0, ek0, el1, ek1 ) );
          Elocall += 16;
        }
      }

      Elocalk += 16;
    }
  }

  alfCovariance.pixAcc += horizontal_add_s32x4( dot_s16x8x2_neon( y0, y0, y1, y1 ) );
}

// The covariance sums of one 4x4 chroma block in EncAdaptiveLoopFilter::getBlkStatsCcAlf, as its C code:
// E[k][l] += ELocal[k] . ELocal[l] for k <= l (Cb's statistics only), y[k] += ELocal[k] . yLocal[c] and
// pixAcc += yLocal[c] . yLocal[c] for Cb and Cr. The samples have at most 12 bits, so a difference has at most 13 and
// the 16 products of a dot product add up to less than 2^30 in 32 bit: each integer sum equals the C code's, and each
// is added to its floating-point statistic once per block, as there.
static void getBlkCovarianceCcAlf_neon( AlfCovariance& covarianceCb, AlfCovariance& covarianceCr, const int numCoeff,
                                        const Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][16],
                                        const Pel yLocal[2][4][4] )
{
  const int numRows = numCoeff - 1;

  int16x8_t eLow[MAX_NUM_CC_ALF_CHROMA_COEFF];    // samples 0 ... 7 of each ELocal row
  int16x8_t eHigh[MAX_NUM_CC_ALF_CHROMA_COEFF];   // samples 8 ... 15
  for( int k = 0; k < numRows; k++ )
  {
    eLow[k]  = vld1q_s16( &ELocal[k][0] );
    eHigh[k] = vld1q_s16( &ELocal[k][8] );
  }
  const int16x8_t yLowCb  = vld1q_s16( &yLocal[0][0][0] );   // rows 0 and 1 of Cb's 4x4 block
  const int16x8_t yHighCb = vld1q_s16( &yLocal[0][2][0] );   // rows 2 and 3
  const int16x8_t yLowCr  = vld1q_s16( &yLocal[1][0][0] );
  const int16x8_t yHighCr = vld1q_s16( &yLocal[1][2][0] );

  for( int k = 0; k < numRows; k++ )
  {
    for( int l = k; l < numRows; l++ )
    {
      covarianceCb.E[0][0][k][l] += horizontal_add_s32x4( dot_s16x8x2_neon( eLow[k], eLow[l], eHigh[k], eHigh[l] ) );
    }
    covarianceCb.y[0][k] += horizontal_add_s32x4( dot_s16x8x2_neon( eLow[k], yLowCb, eHigh[k], yHighCb ) );
    covarianceCr.y[0][k] += horizontal_add_s32x4( dot_s16x8x2_neon( eLow[k], yLowCr, eHigh[k], yHighCr ) );
  }
  covarianceCb.pixAcc += horizontal_add_s32x4( dot_s16x8x2_neon( yLowCb, yLowCb, yHighCb, yHighCb ) );
  covarianceCr.pixAcc += horizontal_add_s32x4( dot_s16x8x2_neon( yLowCr, yLowCr, yHighCr, yHighCr ) );
}

// getPreBlkStatsAccumFixedSize_neon with all clipping bins, for a block whose equal bins form the runs of
// equalBinsMask (AlfEqualBinRuns): each dot product is computed once per pair of runs and added, with the same
// instructions as in getPreBlkStatsAccumFixedSize_neon, to every cell the pair stands for, so the result is identical.
template<int NumCoeff>
void getPreBlkStatsAccumEqualBinsFixedSize_neon( AlfCovariance& alfCovariance, const Pel* ELocal, const Pel yLocal[4][4],
                                                 const int equalBinsMask )
{
  static constexpr int bstride = MAX_NUM_ALF_LUMA_COEFF << 4;
  static constexpr int kstride = 1 << 4;

  const AlfEqualBinRuns runs( equalBinsMask );

  const int16x8_t y0 = vld1q_s16( &yLocal[0][0] );
  const int16x8_t y1 = vld1q_s16( &yLocal[2][0] );

  auto y = alfCovariance.y;
  auto E = alfCovariance.E;

  for( int r0 = 0; r0 < runs.numRuns; r0++ )
  {
    const int firstBin0 = runs.first[r0];
    const int endBin0   = firstBin0 + runs.size[r0];

    // ELocal is one flat buffer, but logically it is ELocal[NumBins][MAX_NUM_ALF_LUMA_COEFF][16].
    const Pel* Elocalk = ELocal + firstBin0 * bstride;

    for( int k = 0; k < NumCoeff; k++ )
    {
      const int16x8_t ek0 = vld1q_s16( Elocalk + 0 );
      const int16x8_t ek1 = vld1q_s16( Elocalk + 8 );

      const int sumY = horizontal_add_s32x4( dot_s16x8x2_neon( ek0, y0, ek1, y1 ) );
      for( int b0 = firstBin0; b0 < endBin0; b0++ )
      {
        y[b0][k] += sumY;
      }

      for( int r1 = 0; r1 < runs.numRuns; r1++ )
      {
        const int firstBin1 = runs.first[r1];
        const int endBin1   = firstBin1 + runs.size[r1];
        const Pel* Elocall  = ELocal + firstBin1 * bstride + k * kstride;

        int l = k;

        for( ; l < NumCoeff - 3; l += 4 )
        {
          const float32x4_t vSum = vcvtq_f32_s32( rowDots4_neon( ek0, ek1, Elocall ) );

          for( int b0 = firstBin0; b0 < endBin0; b0++ )
          {
            for( int b1 = firstBin1; b1 < endBin1; b1++ )
            {
              alf_float_t* cov       = &E[b0][b1][k][l];
              const float32x4_t vCov = vld1q_f32( cov );
              vst1q_f32( cov, vaddq_f32( vSum, vCov ) );
            }
          }

          Elocall += 4 * 16;
        }

        for( ; l < NumCoeff; l++ )
        {
          const int16x8_t el0 = vld1q_s16( Elocall + 0 );
          const int16x8_t el1 = vld1q_s16( Elocall + 8 );

          const int sum = horizontal_add_s32x4( dot_s16x8x2_neon( el0, ek0, el1, ek1 ) );
          for( int b0 = firstBin0; b0 < endBin0; b0++ )
          {
            for( int b1 = firstBin1; b1 < endBin1; b1++ )
            {
              E[b0][b1][k][l] += sum;
            }
          }

          Elocall += 16;
        }
      }

      Elocalk += 16;
    }
  }

  alfCovariance.pixAcc += horizontal_add_s32x4( dot_s16x8x2_neon( y0, y0, y1, y1 ) );
}

void getPreBlkStatsAccumEqualBins_neon( AlfCovariance& alfCovariance, const AlfFilterShape& shape, const Pel* ELocal,
                                        const Pel yLocal[4][4], const int equalBinsMask )
{
  if( shape.numCoeff == 7 )
  {
    if( !alfEqualBinsKernelTakes( shape.numCoeff, equalBinsMask ) )
    {
      return getPreBlkStatsAccumFixedSize_neon</*NumBins=*/4, /*NumCoeff=*/7>( alfCovariance, ELocal, yLocal );
    }
    return getPreBlkStatsAccumEqualBinsFixedSize_neon</*NumCoeff=*/7>( alfCovariance, ELocal, yLocal, equalBinsMask );
  }
  else if( shape.numCoeff == 13 )
  {
    return getPreBlkStatsAccumEqualBinsFixedSize_neon</*NumCoeff=*/13>( alfCovariance, ELocal, yLocal, equalBinsMask );
  }
  CHECK( true, "Unsupported ALF filter shape" );
}

void getPreBlkStatsAccum_neon( AlfCovariance& alfCovariance, const AlfFilterShape& shape, const Pel* ELocal,
                               const Pel yLocal[4][4], const int numBins )
{
  if( shape.numCoeff == 7 )
  {
    if( numBins == 1 )
    {
      return getPreBlkStatsAccumFixedSize_neon</*NumBins=*/1, /*NumCoeff=*/7>( alfCovariance, ELocal, yLocal );
    }
    else if( numBins == 4 )
    {
      return getPreBlkStatsAccumFixedSize_neon</*NumBins=*/4, /*NumCoeff=*/7>( alfCovariance, ELocal, yLocal );
    }
  }
  else if( shape.numCoeff == 13 )
  {
    if( numBins == 1 )
    {
      return getPreBlkStatsAccumFixedSize_neon</*NumBins=*/1, /*NumCoeff=*/13>( alfCovariance, ELocal, yLocal );
    }
    else if( numBins == 4 )
    {
      return getPreBlkStatsAccumFixedSize_neon</*NumBins=*/4, /*NumCoeff=*/13>( alfCovariance, ELocal, yLocal );
    }
  }
  CHECK( true, "Unsupported ALF filter shape or number of bins" );
}

// The filter taps of linear ALF's statistics in the order of calcLinCovariance4, per transposeIdx: the offset { dy, dx }
// of one sample of each symmetric pair (the other one is at { -dy, -dx }). transposeIdx 1 swaps dy and dx of
// transposeIdx 0, 2 mirrors dx of transposeIdx 0, 3 swaps dy and dx of transposeIdx 2. The sample itself comes after
// the last tap.
static const int8_t g_alfLinTaps7x7[4][12][2] = {
  { { -3, 0 }, { -2, -1 }, { -2, 0 }, { -2, 1 }, { -1, -2 }, { -1, -1 }, { -1, 0 }, { -1, 1 }, { -1, 2 }, { 0, -3 }, { 0, -2 }, { 0, -1 } },
  { { 0, -3 }, { -1, -2 }, { 0, -2 }, { 1, -2 }, { -2, -1 }, { -1, -1 }, { 0, -1 }, { 1, -1 }, { 2, -1 }, { -3, 0 }, { -2, 0 }, { -1, 0 } },
  { { -3, 0 }, { -2, 1 }, { -2, 0 }, { -2, -1 }, { -1, 2 }, { -1, 1 }, { -1, 0 }, { -1, -1 }, { -1, -2 }, { 0, -3 }, { 0, -2 }, { 0, -1 } },
  { { 0, -3 }, { 1, -2 }, { 0, -2 }, { -1, -2 }, { 2, -1 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -2, -1 }, { -3, 0 }, { -2, 0 }, { -1, 0 } },
};
static const int8_t g_alfLinTaps5x5[4][6][2] = {
  { { -2, 0 }, { -1, -1 }, { -1, 0 }, { -1, 1 }, { 0, -2 }, { 0, -1 } },
  { { 0, -2 }, { -1, -1 }, { 0, -1 }, { 1, -1 }, { -2, 0 }, { -1, 0 } },
  { { -2, 0 }, { -1, 1 }, { -1, 0 }, { -1, -1 }, { 0, -2 }, { 0, -1 } },
  { { 0, -2 }, { 1, -1 }, { 0, -1 }, { -1, -1 }, { -2, 0 }, { -1, 0 } },
};

// EncAdaptiveLoopFilter::m_getLinBlkInputs as getLinBlkInputs (calcLinCovariance4 of each of the 4 rows), for all 16
// samples of the 4x4 block at once: yLocal = org - rec, and per tap the 16 values rec[dy][dx] + rec[-dy][-dx] -
// 2 * rec[0][0], stored row after row (16 Pels per tap in ELocal), then the samples themselves. The values wrap in
// 16 bits as the C code's Pel arithmetic.
template<int NumTaps>
static void getLinBlkInputs4x4_neon( Pel* ELocal, Pel yLocal[4][4], const Pel* org, const ptrdiff_t orgStride,
                                     const Pel* rec, const ptrdiff_t recStride, const int8_t taps[NumTaps][2] )
{
  int16x8_t curRows[2];
  int16x8_t orgRows[2];
  load_s16x4x4( rec, recStride, curRows );
  load_s16x4x4( org, orgStride, orgRows );
  vst1q_s16( &yLocal[0][0], vsubq_s16( orgRows[0], curRows[0] ) );
  vst1q_s16( &yLocal[2][0], vsubq_s16( orgRows[1], curRows[1] ) );

  int16x8_t cur2Rows[2];
  cur2Rows[0] = vshlq_n_s16( curRows[0], 1 );
  cur2Rows[1] = vshlq_n_s16( curRows[1], 1 );

  for( int k = 0; k < NumTaps; k++ )
  {
    const ptrdiff_t offset = taps[k][0] * recStride + taps[k][1];
    int16x8_t posRows[2];
    int16x8_t negRows[2];
    load_s16x4x4( rec + offset, recStride, posRows );
    load_s16x4x4( rec - offset, recStride, negRows );
    vst1q_s16( ELocal + 16 * k + 0, vsubq_s16( vaddq_s16( posRows[0], negRows[0] ), cur2Rows[0] ) );
    vst1q_s16( ELocal + 16 * k + 8, vsubq_s16( vaddq_s16( posRows[1], negRows[1] ), cur2Rows[1] ) );
  }

  vst1q_s16( ELocal + 16 * NumTaps + 0, curRows[0] );
  vst1q_s16( ELocal + 16 * NumTaps + 8, curRows[1] );
}

static void getLinBlkInputs_neon( Pel* ELocal, Pel yLocal[4][4], const Pel* org, const int orgStride, const Pel* rec,
                                  const int recStride, const int halfFilterLength, const int transposeIdx )
{
  // the 7x7 luma and 5x5 chroma diamonds
  if( halfFilterLength == 3 )
  {
    getLinBlkInputs4x4_neon<12>( ELocal, yLocal, org, orgStride, rec, recStride, g_alfLinTaps7x7[transposeIdx] );
  }
  else
  {
    CHECKD( halfFilterLength != 2, "Unsupported ALF filter length" );
    getLinBlkInputs4x4_neon<6>( ELocal, yLocal, org, orgStride, rec, recStride, g_alfLinTaps5x5[transposeIdx] );
  }
}

template<>
void EncAdaptiveLoopFilter::_initEncAdaptiveLoopFilterARM<NEON>()
{
  m_getLinBlkInputs = getLinBlkInputs_neon;
  m_getPreBlkStatsAccum = getPreBlkStatsAccum_neon;
  m_getPreBlkStatsAccumEqualBins = getPreBlkStatsAccumEqualBins_neon;
  m_getBlkCovarianceCcAlf        = getBlkCovarianceCcAlf_neon;
}

} // namespace vvenc

#endif

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
 * \file AffineGradientSearch_neon.cpp
 * \brief Neon implementation of AffineGradientSearch for Arm.
 */

// ====================================================================================================================
// Includes
// ====================================================================================================================

#include <algorithm>

#include "../CommonDefARM.h"
#include "CommonLib/AffineGradientSearch.h"
#include "sum_neon.h"

//! \ingroup CommonLib
//! \{

#if ENABLE_SIMD_OPT_AFFINE_ME && defined( TARGET_SIMD_ARM )

namespace vvenc
{

// The normal equations of the affine motion search from sums per 4x4 block (the scheme is described in
// AffineGradientSearch.h, above eqCoeffMatrixFromBlockTotals). Per band of 4 rows and 16 columns (four blocks),
// vmlal_s16 gives each block's column sums of the five products, two pairwise adds give the four block sums, and those
// are weighted by 1, cx and cx^2 into 64-bit totals; after each band the running sums for the cy weights are updated.
// Ranges: the block sums are exact in 32 bits while the gradients and the residue stay within +-8192
// (16 * 8192 * 8192 = 2^30; real encodes show up to 13 bits), cx^2 is below 2^14, and the totals of a 128x128 block stay
// below 2^55.
template<bool b6Param>
void simdEqualCoeffComputer_neon( Pel* const pResidue, const int residueStride, Pel** const ppDerivate,
                                  const int derivateBufStride, const int width, const int height,
                                  int64_t ( *pEqualCoeff )[7] )
{
  // the affine motion search runs on CUs of 16 to 128 samples in each direction, powers of two
  CHECK( width < 16 || width > 128 || ( width & ( width - 1 ) ) != 0, "Width must be a power of two from 16 to 128" );
  CHECK( height < 16 || height > 128 || ( height & ( height - 1 ) ) != 0, "Height must be a power of two from 16 to 128" );

  // total[s][k]: the block sums of product s weighted by cx^k (k = 0, 1, 2), over all bands so far;
  // runSum1[s][k] (k = 0, 1): total[s][k] added after each band; runSum2[s]: runSum1[s][0] added after each band.
  int64x2_t total[NUM_EQ_COEFF_SUMS][3];
  int64x2_t runSum1[NUM_EQ_COEFF_SUMS][2];
  int64x2_t runSum2[NUM_EQ_COEFF_SUMS];
  for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
  {
    total[s][0]   = vdupq_n_s64( 0 );
    total[s][1]   = vdupq_n_s64( 0 );
    total[s][2]   = vdupq_n_s64( 0 );
    runSum1[s][0] = vdupq_n_s64( 0 );
    runSum1[s][1] = vdupq_n_s64( 0 );
    runSum2[s]    = vdupq_n_s64( 0 );
  }

  static const int32_t firstCentres[4] = { 2, 6, 10, 14 };   // cx of the four blocks of the first 16 columns

  for( int y = 0; y < height; y += 4 )
  {
    int32x4_t cx = vld1q_s32( firstCentres );
    for( int x = 0; x < width; x += 16 )
    {
      const Pel* d0  = ppDerivate[0] + y * derivateBufStride + x;
      const Pel* d1  = ppDerivate[1] + y * derivateBufStride + x;
      const Pel* res = pResidue + y * residueStride + x;

      // colSum[s][k]: the four column sums over 4 rows of product s in block k
      int32x4_t colSum[NUM_EQ_COEFF_SUMS][4];
      for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
      {
        for( int k = 0; k < 4; k++ )
        {
          colSum[s][k] = vdupq_n_s32( 0 );
        }
      }

      for( int row = 0; row < 4; row++ )
      {
        const int16x8_t a01 = vld1q_s16( d0 );
        const int16x8_t a23 = vld1q_s16( d0 + 8 );
        const int16x8_t b01 = vld1q_s16( d1 );
        const int16x8_t b23 = vld1q_s16( d1 + 8 );
        const int16x8_t r01 = vld1q_s16( res );
        const int16x8_t r23 = vld1q_s16( res + 8 );
        const int16x4_t a[4] = { vget_low_s16( a01 ), vget_high_s16( a01 ), vget_low_s16( a23 ), vget_high_s16( a23 ) };
        const int16x4_t b[4] = { vget_low_s16( b01 ), vget_high_s16( b01 ), vget_low_s16( b23 ), vget_high_s16( b23 ) };
        const int16x4_t r[4] = { vget_low_s16( r01 ), vget_high_s16( r01 ), vget_low_s16( r23 ), vget_high_s16( r23 ) };
        for( int k = 0; k < 4; k++ )
        {
          colSum[SUM_D0D0][k] = vmlal_s16( colSum[SUM_D0D0][k], a[k], a[k] );
          colSum[SUM_D0D1][k] = vmlal_s16( colSum[SUM_D0D1][k], a[k], b[k] );
          colSum[SUM_D1D1][k] = vmlal_s16( colSum[SUM_D1D1][k], b[k], b[k] );
          colSum[SUM_D0R][k]  = vmlal_s16( colSum[SUM_D0R][k], a[k], r[k] );
          colSum[SUM_D1R][k]  = vmlal_s16( colSum[SUM_D1R][k], b[k], r[k] );
        }
        d0  += derivateBufStride;
        d1  += derivateBufStride;
        res += residueStride;
      }

      const int32x4_t cx2 = vmulq_s32( cx, cx );
      for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
      {
        // lane k: the sum of block k
        const int32x4_t blockSum = pairwise_add_s32x4( pairwise_add_s32x4( colSum[s][0], colSum[s][1] ),
                                                       pairwise_add_s32x4( colSum[s][2], colSum[s][3] ) );
        total[s][0] = vpadalq_s32( total[s][0], blockSum );
        total[s][1] = vmlal_s32( total[s][1], vget_low_s32( blockSum ), vget_low_s32( cx ) );
        total[s][1] = vmlal_s32( total[s][1], vget_high_s32( blockSum ), vget_high_s32( cx ) );
        total[s][2] = vmlal_s32( total[s][2], vget_low_s32( blockSum ), vget_low_s32( cx2 ) );
        total[s][2] = vmlal_s32( total[s][2], vget_high_s32( blockSum ), vget_high_s32( cx2 ) );
      }
      cx = vaddq_s32( cx, vdupq_n_s32( 16 ) );
    }
    for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
    {
      runSum1[s][0] = vaddq_s64( runSum1[s][0], total[s][0] );
      runSum1[s][1] = vaddq_s64( runSum1[s][1], total[s][1] );
      runSum2[s]    = vaddq_s64( runSum2[s], runSum1[s][0] );
    }
  }

  EqCoeffBlockTotals totals;
  for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
  {
    totals.sum[s]    = horizontal_add_s64x2( total[s][0] );
    totals.sumCx[s]  = horizontal_add_s64x2( total[s][1] );
    totals.sumCx2[s] = horizontal_add_s64x2( total[s][2] );
    totals.run[s]    = horizontal_add_s64x2( runSum1[s][0] );
    totals.runCx[s]  = horizontal_add_s64x2( runSum1[s][1] );
    totals.run2[s]   = horizontal_add_s64x2( runSum2[s] );
  }
  eqCoeffMatrixFromBlockTotals<b6Param>( totals, height / 4, pEqualCoeff );
}

template<>
void AffineGradientSearch::_initAffineGradientSearchARM<NEON>()
{
  m_EqualCoeffComputer[0] = simdEqualCoeffComputer_neon<false>;
  m_EqualCoeffComputer[1] = simdEqualCoeffComputer_neon<true>;
}

} // namespace vvenc

#endif // ENABLE_SIMD_OPT_AFFINE_ME && defined(TARGET_SIMD_ARM)

//! \}

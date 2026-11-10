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
 * \file
 * \brief Declaration of AffineGradientSearch class
 */

#pragma once

#include "CommonDef.h"
#include "TypeDef.h"

namespace vvenc {
  //! \ingroup CommonLib
  //! \{

#if defined(TARGET_SIMD_X86) && ENABLE_SIMD_OPT_AFFINE_ME
using namespace x86_simd;
#endif

#if defined(TARGET_SIMD_ARM) && ENABLE_SIMD_OPT_AFFINE_ME
using namespace arm_simd;
#endif

  // The normal equations of the affine motion search from sums per 4x4 block (used by the AVX2 and NEON kernels).
  //
  // Per pixel the scalar version (AffineGradientSearch::xEqualCoeffComputer) adds the products iC[p] * iC[q] and
  // iC[p] * 8 * residue, where every iC[p] is a sum of terms coef * cx^i * cy^j * d with d one of the two gradients
  // and cx = 4 * ( x / 4 ) + 2, cy = 4 * ( y / 4 ) + 2: constant within each 4x4 block. So every matrix entry is a
  // weighted sum of five per-block sums, S = sum of d0 * d0, d0 * d1, d1 * d1, d0 * residue and d1 * residue, weighted
  // by cx^i * cy^j with i + j <= 2. The kernels gather, for each S over all blocks of the CU, its sums weighted by 1, cx
  // and cx^2, and running sums over the bands of 4 rows for the cy weights (EqCoeffBlockTotals); the matrix follows from
  // them here. All sums are exact integers, so the result is identical to the per-pixel sums of the scalar version.

  enum EqCoeffSum { SUM_D0D0, SUM_D0D1, SUM_D1D1, SUM_D0R, SUM_D1R, NUM_EQ_COEFF_SUMS };
  static constexpr int NUM_EQ_COEFF_POWERS = 6;   // cx^i * cy^j for (i, j) = (0,0) (1,0) (2,0) (0,1) (1,1) (0,2)

  static inline int eqCoeffPower( int i, int j )
  {
    return j == 0 ? i : ( j == 1 ? 3 + i : 5 );
  }

  // One term coef * cx^i * cy^j * d of an iC[p], with d the gradient grad (0 or 1). An iC[p] has one or two terms; a
  // missing second term has coef 0, so every product of terms runs over two by two terms with fixed bounds (which the
  // compiler unrolls and folds) and the missing ones add 0.
  struct EqCoeffTerm  { int coef, i, j, grad; };
  struct EqCoeffTerms { EqCoeffTerm term[2]; };

  // Per sum S, with B_b the sums S of band b (rows 4 b to 4 b + 3) of N bands:
  //   sum, sumCx, sumCx2  S over all blocks, weighted by 1, cx and cx^2;
  //   run, runCx          sum_b ( N - b ) B_b, unweighted and weighted by cx;
  //   run2                sum_b ( N - b )( N - b + 1 ) / 2 B_b.
  // A kernel gets run and runCx by adding its totals so far after each band, and run2 by adding run after each band.
  struct EqCoeffBlockTotals
  {
    int64_t sum[NUM_EQ_COEFF_SUMS], sumCx[NUM_EQ_COEFF_SUMS], sumCx2[NUM_EQ_COEFF_SUMS];
    int64_t run[NUM_EQ_COEFF_SUMS], runCx[NUM_EQ_COEFF_SUMS], run2[NUM_EQ_COEFF_SUMS];
  };

  template<bool b6Param>
  static inline void eqCoeffMatrixFromBlockTotals( const EqCoeffBlockTotals& t, const int64_t numBands, int64_t ( *pEqualCoeff )[7] )
  {
    // weighted[s][power]: the sum over the block of cx^i * cy^j * S. From the running sums:
    //   sum_b ( N - b ) B_b = run, sum_b ( N - b )^2 B_b = 2 run2 - run, hence
    //   sum_b b B_b = N sum - run and sum_b b^2 B_b = N^2 sum - 2 N run + 2 run2 - run,
    // and cy = 4 b + 2, cy^2 = 16 b^2 + 16 b + 4. All exact in 64 bits (below 2^55 for 128x128 and 14-bit inputs).
    int64_t weighted[NUM_EQ_COEFF_SUMS][NUM_EQ_COEFF_POWERS];
    for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
    {
      const int64_t sumB   = numBands * t.sum[s] - t.run[s];      // sum_b b B_b
      const int64_t sumBCx = numBands * t.sumCx[s] - t.runCx[s];  // sum_b b B_b, weighted by cx
      const int64_t sumB2  = numBands * numBands * t.sum[s] - 2 * numBands * t.run[s] + 2 * t.run2[s] - t.run[s];
      weighted[s][eqCoeffPower( 0, 0 )] = t.sum[s];
      weighted[s][eqCoeffPower( 1, 0 )] = t.sumCx[s];
      weighted[s][eqCoeffPower( 2, 0 )] = t.sumCx2[s];
      weighted[s][eqCoeffPower( 0, 1 )] = 4 * sumB + 2 * t.sum[s];
      weighted[s][eqCoeffPower( 1, 1 )] = 4 * sumBCx + 2 * t.sumCx[s];
      weighted[s][eqCoeffPower( 0, 2 )] = 16 * sumB2 + 16 * sumB + 4 * t.sum[s];
    }

    // the iC[p] of the scalar version as terms: 6 parameters iC = { d0, cx d0, d1, cx d1, cy d0, cy d1 },
    // 4 parameters iC = { d0, cx d0 + cy d1, d1, cy d0 - cx d1 }
    static constexpr int n = b6Param ? 6 : 4;
    static constexpr EqCoeffTerms terms6[6] = { { { { 1, 0, 0, 0 } } }, { { { 1, 1, 0, 0 } } }, { { { 1, 0, 0, 1 } } },
                                                { { { 1, 1, 0, 1 } } }, { { { 1, 0, 1, 0 } } }, { { { 1, 0, 1, 1 } } } };
    static constexpr EqCoeffTerms terms4[4] = { { { { 1, 0, 0, 0 } } }, { { { 1, 1, 0, 0 }, { 1, 0, 1, 1 } } },
                                                { { { 1, 0, 0, 1 } } }, { { { 1, 0, 1, 0 }, { -1, 1, 0, 1 } } } };
    const EqCoeffTerms* terms = b6Param ? terms6 : terms4;
    static constexpr EqCoeffSum gradProduct[2][2] = { { SUM_D0D0, SUM_D0D1 }, { SUM_D0D1, SUM_D1D1 } };

    for( int col = 0; col < n; col++ )
    {
      for( int row = col; row < n; row++ )
      {
        int64_t sum = 0;
        for( int a = 0; a < 2; a++ )
        {
          const EqCoeffTerm& tc = terms[col].term[a];
          for( int b = 0; b < 2; b++ )
          {
            const EqCoeffTerm& tr = terms[row].term[b];
            sum += tc.coef * tr.coef * weighted[gradProduct[tc.grad][tr.grad]][eqCoeffPower( tc.i + tr.i, tc.j + tr.j )];
          }
        }
        // as the previous SIMD versions: the entry and its mirror both take the sum of the upper entry
        pEqualCoeff[col + 1][row] += sum;
        pEqualCoeff[row + 1][col]  = pEqualCoeff[col + 1][row];
      }
      int64_t sum = 0;
      for( int a = 0; a < 2; a++ )
      {
        const EqCoeffTerm& tc = terms[col].term[a];
        sum += tc.coef * weighted[tc.grad == 0 ? SUM_D0R : SUM_D1R][eqCoeffPower( tc.i, tc.j )];
      }
      pEqualCoeff[col + 1][n] += sum * 8;   // the scalar version's residue * (1 << 3)
    }
  }

  class AffineGradientSearch
  {
  public:
    void  (*m_HorizontalSobelFilter)  (Pel* const pPred, const int predStride, Pel *const pDerivate,   const int derivateBufStride, const int width, const int height);
    void  (*m_VerticalSobelFilter)    (Pel* const pPred, const int predStride, Pel *const pDerivate,   const int derivateBufStride, const int width, const int height);
    void  (*m_EqualCoeffComputer[2])  (Pel* const pResi, const int resiStride, Pel **const ppDerivate, const int derivateBufStride, const int width, const int height, int64_t(*pEqualCoeff)[7]);

    static void xHorizontalSobelFilter( Pel* const pPred, const int predStride, Pel *const pDerivate,   const int derivateBufStride, const int width, const int height);
    static void xVerticalSobelFilter  ( Pel* const pPred, const int predStride, Pel *const pDerivate,   const int derivateBufStride, const int width, const int height);
    template<bool b6Param>
    static void xEqualCoeffComputer   ( Pel* const pResi, const int resiStride, Pel **const ppDerivate, const int derivateBufStride, const int width, const int height, int64_t(*pEqualCoeff)[7]);

    AffineGradientSearch( bool enableOpt = true );
    ~AffineGradientSearch() {}

#if defined(TARGET_SIMD_X86) && ENABLE_SIMD_OPT_AFFINE_ME
    void initAffineGradientSearchX86();
    template <X86_VEXT vext>
    void _initAffineGradientSearchX86();
#endif

#if defined(TARGET_SIMD_ARM) && ENABLE_SIMD_OPT_AFFINE_ME
    void initAffineGradientSearchARM();
    template <ARM_VEXT vext>
    void _initAffineGradientSearchARM();
#endif
  };

} // namespace vvenc

//! \}


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
 * \brief Implementation of AffineGradientSearch class
 */
//#define USE_AVX2
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "CommonDefX86.h"
#include "../AffineGradientSearch.h"

//! \ingroup CommonLib
//! \{

#if defined(TARGET_SIMD_X86)  && ENABLE_SIMD_OPT_AFFINE_ME

namespace vvenc {

  template<X86_VEXT vext>
  static void simdHorizontalSobelFilter(Pel* const pPred, const int predStride, Pel *const pDerivate, const int derivateBufStride, const int width, const int height)
  {
    CHECK( width % 8, "Invalid size!" );

    // pPred is 10-bit

    // -1 0 1
    // -2 0 2
    // -1 0 1
    // 
    // sum( sobel ) = 8, i.e. 4-bit extension

    for( int y = 1; y < ( height - 1 ); y++ )
    {
      int x = 1;
      for( ; x < ( width - 8 ); x += 8 )
      {
        __m128i acc = _mm_loadu_si128( ( const __m128i* ) &pPred[y * predStride + x - 1] );
        acc         = _mm_sub_epi16( _mm_loadu_si128( ( const __m128i* ) &pPred[y * predStride + x + 1] ), acc );
        acc         = _mm_slli_epi16( acc, 1 );
        acc         = _mm_sub_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y - 1 ) * predStride + x - 1] ) );
        acc         = _mm_add_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y - 1 ) * predStride + x + 1] ) );
        acc         = _mm_sub_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y + 1 ) * predStride + x - 1] ) );
        acc         = _mm_add_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y + 1 ) * predStride + x + 1] ) );

        _mm_storeu_si128( ( __m128i* ) &pDerivate[y * derivateBufStride + x], acc );
      }

      __m128i acc = _mm_loadu_si128( ( const __m128i* ) &pPred[y * predStride + x - 1] );
      acc         = _mm_sub_epi16( _mm_loadu_si128( ( const __m128i* ) &pPred[y * predStride + x + 1] ), acc );
      acc         = _mm_slli_epi16( acc, 1 );
      acc         = _mm_sub_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y - 1 ) * predStride + x - 1] ) );
      acc         = _mm_add_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y - 1 ) * predStride + x + 1] ) );
      acc         = _mm_sub_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y + 1 ) * predStride + x - 1] ) );
      acc         = _mm_add_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y + 1 ) * predStride + x + 1] ) );

      _vv_storel_epi64( ( __m128i* ) &pDerivate[y * derivateBufStride + x],                         acc );
      _mm_storeu_si32 (              &pDerivate[y * derivateBufStride + x + 4], _mm_unpackhi_epi64( acc, acc ) );

      pDerivate[y * derivateBufStride]               = pDerivate[y * derivateBufStride + 1];
      pDerivate[y * derivateBufStride + (width - 1)] = pDerivate[y * derivateBufStride + (width - 2)];
    }

    memcpy( pDerivate,                                      pDerivate + derivateBufStride,                  width * sizeof( pDerivate[ 0 ] ) );
    memcpy( pDerivate + ( height - 1 ) * derivateBufStride, pDerivate + ( height - 2 ) * derivateBufStride, width * sizeof( pDerivate[ 0 ] ) );
  }

  template<X86_VEXT vext>
  static void simdVerticalSobelFilter(Pel* const pPred, const int predStride, Pel *const pDerivate, const int derivateBufStride, const int width, const int height)
  {
    CHECK( width % 8, "Invalid size!" );

    // pPred is 10-bit

    // -1 -2 -1
    //  0  0  0
    //  1  2  1
    // 
    // sum( sobel ) = 8, i.e. 4-bit extension

    for( int y = 1; y < ( height - 1 ); y++ )
    {
      int x = 1;
      for( ; x < ( width - 8 ); x += 8 )
      {
        __m128i acc = _mm_loadu_si128( ( const __m128i* ) &pPred[( y - 1 ) * predStride + x] );
        acc         = _mm_sub_epi16( _mm_loadu_si128( ( const __m128i* ) &pPred[( y + 1 ) * predStride + x] ), acc );
        acc         = _mm_slli_epi16( acc, 1 );
        acc         = _mm_sub_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y - 1 ) * predStride + x - 1] ) );
        acc         = _mm_sub_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y - 1 ) * predStride + x + 1] ) );
        acc         = _mm_add_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y + 1 ) * predStride + x - 1] ) );
        acc         = _mm_add_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y + 1 ) * predStride + x + 1] ) );

        _mm_storeu_si128( ( __m128i* ) &pDerivate[y * derivateBufStride + x], acc );
      }
      
      __m128i acc = _mm_loadu_si128( ( const __m128i* ) &pPred[( y - 1 ) * predStride + x] );
      acc         = _mm_sub_epi16( _mm_loadu_si128( ( const __m128i* ) &pPred[( y + 1 ) * predStride + x] ), acc );
      acc         = _mm_slli_epi16( acc, 1 );
      acc         = _mm_sub_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y - 1 ) * predStride + x - 1] ) );
      acc         = _mm_sub_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y - 1 ) * predStride + x + 1] ) );
      acc         = _mm_add_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y + 1 ) * predStride + x - 1] ) );
      acc         = _mm_add_epi16( acc, _mm_loadu_si128( ( const __m128i* ) &pPred[( y + 1 ) * predStride + x + 1] ) );

      _vv_storel_epi64( ( __m128i* ) &pDerivate[y * derivateBufStride + x],                         acc );
      _mm_storeu_si32 (              &pDerivate[y * derivateBufStride + x + 4], _mm_unpackhi_epi64( acc, acc ) );

      pDerivate[y * derivateBufStride]               = pDerivate[y * derivateBufStride + 1];
      pDerivate[y * derivateBufStride + (width - 1)] = pDerivate[y * derivateBufStride + (width - 2)];
    }

    memcpy( pDerivate,                                    pDerivate + derivateBufStride,                width * sizeof( pDerivate[ 0 ] ) );
    memcpy( pDerivate + (height - 1) * derivateBufStride, pDerivate + (height - 2) * derivateBufStride, width * sizeof( pDerivate[ 0 ] ) );
  }



#define CALC_EQUAL_COEFF_8PXLS(x1,x2,y1,y2,tmp0,tmp1,tmp2,tmp3,inter0,inter1,inter2,inter3,loadLocation)       \
{                                                                                                              \
inter0 = _mm_mul_epi32(x1, y1);                                                                                \
inter1 = _mm_mul_epi32(tmp0, tmp2);                                                                            \
inter2 = _mm_mul_epi32(x2, y2);                                                                                \
inter3 = _mm_mul_epi32(tmp1, tmp3);                                                                            \
inter2 = _mm_add_epi64(inter0, inter2);                                                                        \
inter3 = _mm_add_epi64(inter1, inter3);                                                                        \
inter0 = _vv_loadl_epi64(loadLocation);                                                                        \
inter3 = _mm_add_epi64(inter2, inter3);                                                                        \
inter1 = _mm_srli_si128(inter3, 8);                                                                            \
inter3 = _mm_add_epi64(inter1, inter3);                                                                        \
inter3 = _mm_add_epi64(inter0, inter3);                                                                        \
}

  template<X86_VEXT vext, bool b6Param>
  static void simdEqualCoeffComputer(Pel* const pResidue, const int residueStride, Pel **const ppDerivate, const int derivateBufStride, const int width, const int height, int64_t(*pEqualCoeff)[7])
  {
    __m128i mmFour;
    __m128i mmTmp[4];
    __m128i mmIntermediate[4];
    __m128i mmIndxK, mmIndxJ;
    __m128i mmResidue[2];
    __m128i mmC[12];

    // Add directly to indexes to get new index
    mmFour  = _mm_set1_epi32(4);
    mmIndxJ = _mm_set1_epi32(-2);


    static constexpr int n = b6Param ? 6 : 4;
    int idx1 = -2 * derivateBufStride - 4;
    int idx2 = -    derivateBufStride - 4;
    int resIdx1 = -2 * residueStride - 4;
    int resIdx2 = -    residueStride - 4;

    for (int j = 0; j < height; j += 2)
    {
      if (!(j & 3))
        mmIndxJ = _mm_add_epi32(mmIndxJ, mmFour);
      mmIndxK = _mm_set1_epi32(-2);
      idx1 += (derivateBufStride << 1);
      idx2 += (derivateBufStride << 1);
      resIdx1 += (residueStride << 1);
      resIdx2 += (residueStride << 1);

      for (int k = 0; k < width; k += 4)
      {
        idx1 += 4;
        idx2 += 4;
        resIdx1 += 4;
        resIdx2 += 4;
        mmIndxK = _mm_add_epi32(mmIndxK, mmFour);

        if (b6Param)
        {
          // mmC[0-5] for iC[0-5] of 1st row of pixels
          mmC[0] = _mm_cvtepi16_epi32(_vv_loadl_epi64((const __m128i*)&ppDerivate[0][idx1]));
          mmC[2] = _mm_cvtepi16_epi32(_vv_loadl_epi64((const __m128i*)&ppDerivate[1][idx1]));
          mmC[1] = _mm_mullo_epi32(mmIndxK, mmC[0]);
          mmC[3] = _mm_mullo_epi32(mmIndxK, mmC[2]);
          mmC[4] = _mm_mullo_epi32(mmIndxJ, mmC[0]);
          mmC[5] = _mm_mullo_epi32(mmIndxJ, mmC[2]);

          // mmC[6-11] for iC[0-5] of 2nd row of pixels
          mmC[6] = _mm_cvtepi16_epi32(_vv_loadl_epi64((const __m128i*)&ppDerivate[0][idx2]));
          mmC[8] = _mm_cvtepi16_epi32(_vv_loadl_epi64((const __m128i*)&ppDerivate[1][idx2]));
          mmC[7] = _mm_mullo_epi32(mmIndxK, mmC[6]);
          mmC[9] = _mm_mullo_epi32(mmIndxK, mmC[8]);
          mmC[10] = _mm_mullo_epi32(mmIndxJ, mmC[6]);
          mmC[11] = _mm_mullo_epi32(mmIndxJ, mmC[8]);
        }
        else
        {
          // mmC[0-3] for iC[0-3] of 1st row of pixels
          mmC[0] = _mm_cvtepi16_epi32(_vv_loadl_epi64((const __m128i*)&ppDerivate[0][idx1]));
          mmC[2] = _mm_cvtepi16_epi32(_vv_loadl_epi64((const __m128i*)&ppDerivate[1][idx1]));
          mmC[1] = _mm_mullo_epi32(mmIndxK, mmC[0]);
          mmC[3] = _mm_mullo_epi32(mmIndxJ, mmC[0]);
          mmTmp[0] = _mm_mullo_epi32(mmIndxJ, mmC[2]);
          mmTmp[1] = _mm_mullo_epi32(mmIndxK, mmC[2]);
          mmC[1] = _mm_add_epi32(mmC[1], mmTmp[0]);
          mmC[3] = _mm_sub_epi32(mmC[3], mmTmp[1]);

          // mmC[4-7] for iC[0-3] of 1st row of pixels
          mmC[4] = _mm_cvtepi16_epi32(_vv_loadl_epi64((const __m128i*)&ppDerivate[0][idx2]));
          mmC[6] = _mm_cvtepi16_epi32(_vv_loadl_epi64((const __m128i*)&ppDerivate[1][idx2]));
          mmC[5] = _mm_mullo_epi32(mmIndxK, mmC[4]);
          mmC[7] = _mm_mullo_epi32(mmIndxJ, mmC[4]);
          mmTmp[2] = _mm_mullo_epi32(mmIndxJ, mmC[6]);
          mmTmp[3] = _mm_mullo_epi32(mmIndxK, mmC[6]);
          mmC[5] = _mm_add_epi32(mmC[5], mmTmp[2]);
          mmC[7] = _mm_sub_epi32(mmC[7], mmTmp[3]);
        }

        // Residue
        mmResidue[0] = _vv_loadl_epi64((const __m128i*)&pResidue[resIdx1]);
        mmResidue[1] = _vv_loadl_epi64((const __m128i*)&pResidue[resIdx2]);
        mmResidue[0] = _mm_cvtepi16_epi32(mmResidue[0]);
        mmResidue[1] = _mm_cvtepi16_epi32(mmResidue[1]);
        mmResidue[0] = _mm_slli_epi32(mmResidue[0], 3);
        mmResidue[1] = _mm_slli_epi32(mmResidue[1], 3);

        // Calculation of coefficient matrix
        for (int col = 0; col < n; col++)
        {
          mmTmp[0] = _mm_srli_si128(mmC[0 + col], 4);
          mmTmp[1] = _mm_srli_si128(mmC[n + col], 4);
          CALC_EQUAL_COEFF_8PXLS(mmC[0 + col], mmC[n + col], mmC[0 + col], mmC[n + col], mmTmp[0], mmTmp[1], mmTmp[0], mmTmp[1], mmIntermediate[0], mmIntermediate[1], mmIntermediate[2], mmIntermediate[3], (const __m128i*)&pEqualCoeff[col + 1][col]);
          _vv_storel_epi64((__m128i*)&pEqualCoeff[col + 1][col], mmIntermediate[3]);

          for (int row = col + 1; row < n; row++)
          {
            mmTmp[2] = _mm_srli_si128(mmC[0 + row], 4);
            mmTmp[3] = _mm_srli_si128(mmC[n + row], 4);
            CALC_EQUAL_COEFF_8PXLS(mmC[0 + col], mmC[n + col], mmC[0 + row], mmC[n + row], mmTmp[0], mmTmp[1], mmTmp[2], mmTmp[3], mmIntermediate[0], mmIntermediate[1], mmIntermediate[2], mmIntermediate[3], (const __m128i*)&pEqualCoeff[col + 1][row]);
            _vv_storel_epi64((__m128i*)&pEqualCoeff[col + 1][row], mmIntermediate[3]);
            _vv_storel_epi64((__m128i*)&pEqualCoeff[row + 1][col], mmIntermediate[3]);
          }

          mmTmp[2] = _mm_srli_si128(mmResidue[0], 4);
          mmTmp[3] = _mm_srli_si128(mmResidue[1], 4);
          CALC_EQUAL_COEFF_8PXLS(mmC[0 + col], mmC[n + col], mmResidue[0], mmResidue[1], mmTmp[0], mmTmp[1], mmTmp[2], mmTmp[3], mmIntermediate[0], mmIntermediate[1], mmIntermediate[2], mmIntermediate[3], (const __m128i*)&pEqualCoeff[col + 1][n]);
          _vv_storel_epi64((__m128i*)&pEqualCoeff[col + 1][n], mmIntermediate[3]);
        }
      }

      idx1 -= (width);
      idx2 -= (width);
      resIdx1 -= (width);
      resIdx2 -= (width);
    }
  }

#if USE_AVX2

  // The normal equations of the affine motion search, computed per 4x4 block (the scheme is described at
  // eqCoeffMatrixFromBlockTotals in AffineGradientSearch.h). The block sums come from _mm256_madd_epi16 (16 pixels per
  // instruction); the weights are applied once per block instead of once per pixel.
  //
  // Ranges: the block sums are exact in 32 bits while the gradients and the residue stay within +-8192 (16 * 8192 *
  // 8192 = 2^30); real encodes have shown values up to 13 bits (the unit test checks those). A weighted sum over a
  // 128x128 block stays below 2^55.

  // The five sums of the four 4x4 blocks of 16 pixels and 4 rows starting at the given pointers, one block per 64-bit
  // lane: blockSum[sum] holds block k's sum in both 32-bit halves of lane k.
  static inline void eqCoeffBlockSums( const Pel* d0, const Pel* d1, const Pel* res, int derivateBufStride, int residueStride,
                                       __m256i blockSum[NUM_EQ_COEFF_SUMS] )
  {
    __m256i acc[NUM_EQ_COEFF_SUMS];
    for( auto& a : acc ) a = _mm256_setzero_si256();
    for( int row = 0; row < 4; row++ )
    {
      const __m256i a = _mm256_loadu_si256( ( const __m256i* ) d0 );
      const __m256i b = _mm256_loadu_si256( ( const __m256i* ) d1 );
      const __m256i r = _mm256_loadu_si256( ( const __m256i* ) res );
      // madd_epi16: the products of two neighbouring pixels added, eight 32-bit sums of pixel pairs
      acc[SUM_D0D0] = _mm256_add_epi32( acc[SUM_D0D0], _mm256_madd_epi16( a, a ) );
      acc[SUM_D0D1] = _mm256_add_epi32( acc[SUM_D0D1], _mm256_madd_epi16( a, b ) );
      acc[SUM_D1D1] = _mm256_add_epi32( acc[SUM_D1D1], _mm256_madd_epi16( b, b ) );
      acc[SUM_D0R]  = _mm256_add_epi32( acc[SUM_D0R],  _mm256_madd_epi16( a, r ) );
      acc[SUM_D1R]  = _mm256_add_epi32( acc[SUM_D1R],  _mm256_madd_epi16( b, r ) );
      d0  += derivateBufStride;
      d1  += derivateBufStride;
      res += residueStride;
    }
    for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
    {
      // block k's 4 columns are 32-bit pair sums 2k and 2k + 1, the two halves of 64-bit lane k: swapping the halves
      // (shuffle 0xB1) and adding puts the block sum into both
      blockSum[s] = _mm256_add_epi32( acc[s], _mm256_shuffle_epi32( acc[s], 0xB1 ) );
    }
  }

  // The sum of the four 64-bit lanes
  static inline int64_t eqCoeffLaneSum( const __m256i v )
  {
    const __m128i halves = _mm_add_epi64( _mm256_castsi256_si128( v ), _mm256_extracti128_si256( v, 1 ) );
    return _mm_cvtsi128_si64( _mm_add_epi64( halves, _mm_unpackhi_epi64( halves, halves ) ) );
  }

  template<bool b6Param>
  static void simdEqualCoeffComputer_avx2(Pel* const pResidue, const int residueStride, Pel **const ppDerivate, const int derivateBufStride, const int width, const int height, int64_t(*pEqualCoeff)[7])
  {
    // the affine motion search runs on CUs wider and higher than 8: powers of two from 16 to 128
    CHECKD( ( width & 15 ) || ( height & 3 ), "EqualCoeffComputer: unsupported block size" );

    // Per 64-bit lane (one 4x4 block column of every 16-pixel step), in vectors:
    //   total[s][k]   the block sums S weighted by cx^k (k = 0, 1, 2), over all bands so far;
    //   runSum1[s][k] (k = 0, 1) total[s][k] added once after each band, runSum2[s] runSum1[s][0] added after each band.
    // So band b (of N bands) is counted N - b times in runSum1 and (N - b)(N - b + 1) / 2 times in runSum2, which gives
    // the cy weights (cy = 4 b + 2) at the end without a multiply per band. _mm256_mul_epi32 multiplies the low, signed
    // 32 bits of each 64-bit lane into a 64-bit product: exact, as the block sums are below 2^30 and cx, cx^2 below 2^14.
    // The arrays keep the 30 accumulators aligned in memory where they do not fit the 16 registers.
    alignas( 32 ) __m256i total[NUM_EQ_COEFF_SUMS][3];
    alignas( 32 ) __m256i runSum1[NUM_EQ_COEFF_SUMS][2];
    alignas( 32 ) __m256i runSum2[NUM_EQ_COEFF_SUMS];
    alignas( 32 ) __m256i blockSum[NUM_EQ_COEFF_SUMS];
    for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
    {
      total[s][0] = total[s][1] = total[s][2] = runSum1[s][0] = runSum1[s][1] = runSum2[s] = _mm256_setzero_si256();
    }
    const __m256i one = _mm256_set1_epi64x( 1 );

    for( int y = 0; y < height; y += 4 )
    {
      __m256i cx = _mm256_set_epi64x( 14, 10, 6, 2 );   // the block centres of the first step, lane 0 first
      for( int x = 0; x < width; x += 16 )
      {
        const Pel* d0  = ppDerivate[0] + y * derivateBufStride + x;
        const Pel* d1  = ppDerivate[1] + y * derivateBufStride + x;
        const Pel* res = pResidue + y * residueStride + x;
        eqCoeffBlockSums( d0, d1, res, derivateBufStride, residueStride, blockSum );
        const __m256i cx2 = _mm256_mul_epi32( cx, cx );
        for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
        {
          total[s][0] = _mm256_add_epi64( total[s][0], _mm256_mul_epi32( blockSum[s], one ) );   // sign-extends to 64 bits
          total[s][1] = _mm256_add_epi64( total[s][1], _mm256_mul_epi32( blockSum[s], cx ) );
          total[s][2] = _mm256_add_epi64( total[s][2], _mm256_mul_epi32( blockSum[s], cx2 ) );
        }
        cx = _mm256_add_epi64( cx, _mm256_set1_epi64x( 16 ) );
      }
      for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
      {
        runSum1[s][0] = _mm256_add_epi64( runSum1[s][0], total[s][0] );
        runSum1[s][1] = _mm256_add_epi64( runSum1[s][1], total[s][1] );
        runSum2[s]    = _mm256_add_epi64( runSum2[s], runSum1[s][0] );
      }
    }

    EqCoeffBlockTotals totals;
    for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
    {
      totals.sum[s]    = eqCoeffLaneSum( total[s][0] );
      totals.sumCx[s]  = eqCoeffLaneSum( total[s][1] );
      totals.sumCx2[s] = eqCoeffLaneSum( total[s][2] );
      totals.run[s]    = eqCoeffLaneSum( runSum1[s][0] );
      totals.runCx[s]  = eqCoeffLaneSum( runSum1[s][1] );
      totals.run2[s]   = eqCoeffLaneSum( runSum2[s] );
    }
    eqCoeffMatrixFromBlockTotals<b6Param>( totals, height / 4, pEqualCoeff );
  }
#endif

  template <X86_VEXT vext>
  void AffineGradientSearch::_initAffineGradientSearchX86()
  {
    m_HorizontalSobelFilter = simdHorizontalSobelFilter<vext>;
    m_VerticalSobelFilter   = simdVerticalSobelFilter<vext>;
#if USE_AVX2
    m_EqualCoeffComputer[0] = simdEqualCoeffComputer_avx2<false>;
    m_EqualCoeffComputer[1] = simdEqualCoeffComputer_avx2<true>;
#else
    m_EqualCoeffComputer[0] = simdEqualCoeffComputer<vext, false>;
    m_EqualCoeffComputer[1] = simdEqualCoeffComputer<vext, true>;
#endif
  }

  template void AffineGradientSearch::_initAffineGradientSearchX86<SIMDX86>();

}

#endif //#ifdef TARGET_SIMD_X86
//! \}

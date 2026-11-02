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

  // The normal equations of the affine motion search, computed per 4x4 block.
  //
  // Per pixel the scalar version (AffineGradientSearch::xEqualCoeffComputer) adds the products iC[p] * iC[q] and
  // iC[p] * 8 * residue, where every iC[p] is a sum of terms coef * cx^i * cy^j * d with d one of the two gradients
  // and cx = 4 * ( x / 4 ) + 2, cy = 4 * ( y / 4 ) + 2: constant within each 4x4 block. So every matrix entry is a
  // weighted sum of five per-block sums, S = sum of d0 * d0, d0 * d1, d1 * d1, d0 * residue and d1 * residue, weighted
  // by cx^i * cy^j with i + j <= 2. The block sums come from _mm256_madd_epi16 (16 pixels per instruction); the
  // weights are applied once per block instead of once per pixel. All sums are exact integers (int32 within a block,
  // int64 above), so the result is identical to the per-pixel sums of the scalar version.
  //
  // Ranges: the block sums are exact in 32 bits while the gradients and the residue stay within +-8192 (16 * 8192 *
  // 8192 = 2^30); real encodes have shown values up to 13 bits (the unit test checks those). A weighted sum over a
  // 128x128 block stays below 2^55.

  enum EqCoeffSum { SUM_D0D0, SUM_D0D1, SUM_D1D1, SUM_D0R, SUM_D1R, NUM_EQ_COEFF_SUMS };
  static constexpr int NUM_EQ_COEFF_POWERS = 6;   // cx^i * cy^j for (i, j) = (0,0) (1,0) (2,0) (0,1) (1,1) (0,2)

  static inline int eqCoeffPower( int i, int j )
  {
    return j == 0 ? i : ( j == 1 ? 3 + i : 5 );
  }

  // The five sums of the four 4x4 blocks of 16 pixels and 4 rows starting at the given pointers: returned per block in
  // blockSum[sum][block].
  static inline void eqCoeffBlockSums( const Pel* d0, const Pel* d1, const Pel* res, int derivateBufStride, int residueStride,
                                       int32_t blockSum[NUM_EQ_COEFF_SUMS][4] )
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
      // hadd adds neighbouring pairs within each 128-bit half: the low 64 bits of each half hold its two block sums
      const __m256i blocks = _mm256_hadd_epi32( acc[s], acc[s] );
      blockSum[s][0] = _mm256_extract_epi32( blocks, 0 );
      blockSum[s][1] = _mm256_extract_epi32( blocks, 1 );
      blockSum[s][2] = _mm256_extract_epi32( blocks, 4 );
      blockSum[s][3] = _mm256_extract_epi32( blocks, 5 );
    }
  }

  // One term coef * cx^i * cy^j * d of an iC[p], with d the gradient grad (0 or 1); an iC[p] has one or two terms.
  struct EqCoeffTerm  { int coef, i, j, grad; };
  struct EqCoeffTerms { int num; EqCoeffTerm term[2]; };

  template<bool b6Param>
  static void simdEqualCoeffComputer_avx2(Pel* const pResidue, const int residueStride, Pel **const ppDerivate, const int derivateBufStride, const int width, const int height, int64_t(*pEqualCoeff)[7])
  {
    // the affine motion search runs on CUs wider and higher than 8: powers of two from 16 to 128
    CHECKD( ( width & 15 ) || ( height & 3 ), "EqualCoeffComputer: unsupported block size" );

    // weighted[s][power]: the sum over the block of cx^i * cy^j * S
    int64_t weighted[NUM_EQ_COEFF_SUMS][NUM_EQ_COEFF_POWERS] = {};
    int32_t blockSum[NUM_EQ_COEFF_SUMS][4];

    for( int y = 0; y < height; y += 4 )
    {
      // the sums of this band of 4 rows, weighted by 1, cx and cx^2
      int64_t band[NUM_EQ_COEFF_SUMS][3] = {};
      for( int x = 0; x < width; x += 16 )
      {
        const Pel* d0  = ppDerivate[0] + y * derivateBufStride + x;
        const Pel* d1  = ppDerivate[1] + y * derivateBufStride + x;
        const Pel* res = pResidue + y * residueStride + x;
        eqCoeffBlockSums( d0, d1, res, derivateBufStride, residueStride, blockSum );
        for( int blk = 0; blk < 4; blk++ )
        {
          const int64_t cx = x + 4 * blk + 2;
          for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
          {
            band[s][0] += blockSum[s][blk];
            band[s][1] += cx * blockSum[s][blk];
            band[s][2] += cx * cx * blockSum[s][blk];
          }
        }
      }
      const int64_t cy = y + 2;
      for( int s = 0; s < NUM_EQ_COEFF_SUMS; s++ )
      {
        weighted[s][eqCoeffPower( 0, 0 )] += band[s][0];
        weighted[s][eqCoeffPower( 1, 0 )] += band[s][1];
        weighted[s][eqCoeffPower( 2, 0 )] += band[s][2];
        weighted[s][eqCoeffPower( 0, 1 )] += cy * band[s][0];
        weighted[s][eqCoeffPower( 1, 1 )] += cy * band[s][1];
        weighted[s][eqCoeffPower( 0, 2 )] += cy * cy * band[s][0];
      }
    }

    // the iC[p] of the scalar version as terms: 6 parameters iC = { d0, cx d0, d1, cx d1, cy d0, cy d1 },
    // 4 parameters iC = { d0, cx d0 + cy d1, d1, cy d0 - cx d1 }
    static constexpr int n = b6Param ? 6 : 4;
    static constexpr EqCoeffTerms terms6[6] = { { 1, { { 1, 0, 0, 0 } } }, { 1, { { 1, 1, 0, 0 } } }, { 1, { { 1, 0, 0, 1 } } },
                                                { 1, { { 1, 1, 0, 1 } } }, { 1, { { 1, 0, 1, 0 } } }, { 1, { { 1, 0, 1, 1 } } } };
    static constexpr EqCoeffTerms terms4[4] = { { 1, { { 1, 0, 0, 0 } } }, { 2, { { 1, 1, 0, 0 }, { 1, 0, 1, 1 } } },
                                                { 1, { { 1, 0, 0, 1 } } }, { 2, { { 1, 0, 1, 0 }, { -1, 1, 0, 1 } } } };
    const EqCoeffTerms* terms = b6Param ? terms6 : terms4;
    static constexpr EqCoeffSum gradProduct[2][2] = { { SUM_D0D0, SUM_D0D1 }, { SUM_D0D1, SUM_D1D1 } };

    for( int col = 0; col < n; col++ )
    {
      for( int row = col; row < n; row++ )
      {
        int64_t sum = 0;
        for( int a = 0; a < terms[col].num; a++ )
        {
          const EqCoeffTerm& t = terms[col].term[a];
          for( int b = 0; b < terms[row].num; b++ )
          {
            const EqCoeffTerm& u = terms[row].term[b];
            sum += t.coef * u.coef * weighted[gradProduct[t.grad][u.grad]][eqCoeffPower( t.i + u.i, t.j + u.j )];
          }
        }
        // as the previous SIMD versions: the entry and its mirror both take the sum of the upper entry
        pEqualCoeff[col + 1][row] += sum;
        pEqualCoeff[row + 1][col]  = pEqualCoeff[col + 1][row];
      }
      int64_t sum = 0;
      for( int a = 0; a < terms[col].num; a++ )
      {
        const EqCoeffTerm& t = terms[col].term[a];
        sum += t.coef * weighted[t.grad == 0 ? SUM_D0R : SUM_D1R][eqCoeffPower( t.i, t.j )];
      }
      pEqualCoeff[col + 1][n] += sum * 8;   // the scalar version's residue * (1 << 3)
    }
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

/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

#ifdef TARGET_SIMD_X86

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

    for (int j = 0; j < height; j += 2)
    {
      if (!(j & 3))
        mmIndxJ = _mm_add_epi32(mmIndxJ, mmFour);
      mmIndxK = _mm_set1_epi32(-2);
      idx1 += (derivateBufStride << 1);
      idx2 += (derivateBufStride << 1);

      for (int k = 0; k < width; k += 4)
      {
        idx1 += 4;
        idx2 += 4;
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
        mmResidue[0] = _vv_loadl_epi64((const __m128i*)&pResidue[idx1]);
        mmResidue[1] = _vv_loadl_epi64((const __m128i*)&pResidue[idx2]);
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
    }
  }

#if USE_AVX2

#define CALC_EQUAL_COEFF_8PXLS_AVX2(x1,x2,y1,y2,tmp0,tmp1,tmp2,tmp3,inter0,inter1,inter2,inter3,res,loadLocation)  \
{                                                                                                                  \
inter0 = _mm256_mul_epi32(x1, y1);                                                                                 \
inter1 = _mm256_mul_epi32(tmp0, tmp2);                                                                             \
inter2 = _mm256_mul_epi32(x2, y2);                                                                                 \
inter3 = _mm256_mul_epi32(tmp1, tmp3);                                                                             \
inter2 = _mm256_add_epi64(inter0, inter2);                                                                         \
inter3 = _mm256_add_epi64(inter1, inter3);                                                                         \
res    = _vv_loadl_epi64(loadLocation);                                                                            \
inter3 = _mm256_add_epi64(inter2, inter3);                                                                         \
inter1 = _mm256_srli_si256(inter3, 8);                                                                             \
inter3 = _mm256_add_epi64(inter1, inter3);                                                                         \
res    = _mm_add_epi64(res, _mm256_castsi256_si128(inter3));                                                       \
res    = _mm_add_epi64(res, _mm256_extracti128_si256(inter3, 1));                                                  \
}

  template<bool b6Param>
  static void simdEqualCoeffComputer_avx2(Pel* const pResidue, const int residueStride, Pel **const ppDerivate, const int derivateBufStride, const int width, const int height, int64_t(*pEqualCoeff)[7])
  {
    __m256i mmFour;
    __m256i mmTmp[4];
    __m256i mmIntermediate[4];
    __m256i mmIndxK, mmIndxJ;
    __m256i mmResidue[2];
    __m256i mmC[12];
    __m128i mmRes;

    // Add directly to indexes to get new index
    mmFour  = _mm256_set1_epi32(4);
    mmIndxJ = _mm256_set1_epi32(-2);

    static constexpr int n = b6Param ? 6 : 4;
    int idx1 = -2 * derivateBufStride - 8;
    int idx2 = -    derivateBufStride - 8;

    for (int j = 0; j < height; j += 2)
    {
      if (!(j & 3))
        mmIndxJ = _mm256_add_epi32(mmIndxJ, mmFour);
      mmIndxK = _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_set1_epi32( -6 ) ), _mm_set1_epi32( -2 ), 1 );
      idx1 += (derivateBufStride << 1);
      idx2 += (derivateBufStride << 1);

      for (int k = 0; k < width; k += 8)
      {
        idx1 += 8;
        idx2 += 8;
        mmIndxK = _mm256_add_epi32(mmIndxK, mmFour);
        mmIndxK = _mm256_add_epi32(mmIndxK, mmFour);

        if (b6Param)
        {
          // mmC[0-5] for iC[0-5] of 1st row of pixels
          mmC[0] = _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i*)&ppDerivate[0][idx1]));
          mmC[2] = _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i*)&ppDerivate[1][idx1]));
          mmC[1] = _mm256_mullo_epi32(mmIndxK, mmC[0]);
          mmC[3] = _mm256_mullo_epi32(mmIndxK, mmC[2]);
          mmC[4] = _mm256_mullo_epi32(mmIndxJ, mmC[0]);
          mmC[5] = _mm256_mullo_epi32(mmIndxJ, mmC[2]);
        
          // mmC[6-11] for iC[0-5] of 2nd row of pixels
          mmC[6] = _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i*)&ppDerivate[0][idx2]));
          mmC[8] = _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i*)&ppDerivate[1][idx2]));
          mmC[7] = _mm256_mullo_epi32(mmIndxK, mmC[6]);
          mmC[9] = _mm256_mullo_epi32(mmIndxK, mmC[8]);
          mmC[10] = _mm256_mullo_epi32(mmIndxJ, mmC[6]);
          mmC[11] = _mm256_mullo_epi32(mmIndxJ, mmC[8]);
        }
        else
        {
          // mmC[0-3] for iC[0-3] of 1st row of pixels
          mmC[0] = _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i*)&ppDerivate[0][idx1]));
          mmC[2] = _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i*)&ppDerivate[1][idx1]));
          mmC[1] = _mm256_mullo_epi32(mmIndxK, mmC[0]);
          mmC[3] = _mm256_mullo_epi32(mmIndxJ, mmC[0]);
          mmTmp[0] = _mm256_mullo_epi32(mmIndxJ, mmC[2]);
          mmTmp[1] = _mm256_mullo_epi32(mmIndxK, mmC[2]);
          mmC[1] = _mm256_add_epi32(mmC[1], mmTmp[0]);
          mmC[3] = _mm256_sub_epi32(mmC[3], mmTmp[1]);

          // mmC[4-7] for iC[0-3] of 1st row of pixels
          mmC[4] = _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i*)&ppDerivate[0][idx2]));
          mmC[6] = _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i*)&ppDerivate[1][idx2]));
          mmC[5] = _mm256_mullo_epi32(mmIndxK, mmC[4]);
          mmC[7] = _mm256_mullo_epi32(mmIndxJ, mmC[4]);
          mmTmp[2] = _mm256_mullo_epi32(mmIndxJ, mmC[6]);
          mmTmp[3] = _mm256_mullo_epi32(mmIndxK, mmC[6]);
          mmC[5] = _mm256_add_epi32(mmC[5], mmTmp[2]);
          mmC[7] = _mm256_sub_epi32(mmC[7], mmTmp[3]);
        }

        // Residue
        mmResidue[0] = _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i*)&pResidue[idx1]));
        mmResidue[1] = _mm256_cvtepi16_epi32(_mm_loadu_si128((const __m128i*)&pResidue[idx2]));
        mmResidue[0] = _mm256_slli_epi32(mmResidue[0], 3);
        mmResidue[1] = _mm256_slli_epi32(mmResidue[1], 3);

        // Calculation of coefficient matrix
        for (int col = 0; col < n; col++)
        {
          mmTmp[0] = _mm256_srli_si256(mmC[0 + col], 4);
          mmTmp[1] = _mm256_srli_si256(mmC[n + col], 4);
          CALC_EQUAL_COEFF_8PXLS_AVX2(mmC[0 + col], mmC[n + col], mmC[0 + col], mmC[n + col], mmTmp[0], mmTmp[1], mmTmp[0], mmTmp[1], mmIntermediate[0], mmIntermediate[1], mmIntermediate[2], mmIntermediate[3], mmRes, (const __m128i*)&pEqualCoeff[col + 1][col]);
          _vv_storel_epi64((__m128i*)&pEqualCoeff[col + 1][col], mmRes);

          for (int row = col + 1; row < n; row++)
          {
            mmTmp[2] = _mm256_srli_si256(mmC[0 + row], 4);
            mmTmp[3] = _mm256_srli_si256(mmC[n + row], 4);
            CALC_EQUAL_COEFF_8PXLS_AVX2(mmC[0 + col], mmC[n + col], mmC[0 + row], mmC[n + row], mmTmp[0], mmTmp[1], mmTmp[2], mmTmp[3], mmIntermediate[0], mmIntermediate[1], mmIntermediate[2], mmIntermediate[3], mmRes, (const __m128i*)&pEqualCoeff[col + 1][row]);
            _vv_storel_epi64((__m128i*)&pEqualCoeff[col + 1][row], mmRes);
            _vv_storel_epi64((__m128i*)&pEqualCoeff[row + 1][col], mmRes);
          }

          mmTmp[2] = _mm256_srli_si256(mmResidue[0], 4);
          mmTmp[3] = _mm256_srli_si256(mmResidue[1], 4);
          CALC_EQUAL_COEFF_8PXLS_AVX2(mmC[0 + col], mmC[n + col], mmResidue[0], mmResidue[1], mmTmp[0], mmTmp[1], mmTmp[2], mmTmp[3], mmIntermediate[0], mmIntermediate[1], mmIntermediate[2], mmIntermediate[3], mmRes, (const __m128i*)&pEqualCoeff[col + 1][n]);
          _vv_storel_epi64((__m128i*)&pEqualCoeff[col + 1][n], mmRes);
        }
      }

      idx1 -= (width);
      idx2 -= (width);
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

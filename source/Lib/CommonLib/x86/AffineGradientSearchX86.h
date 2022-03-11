/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD

License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

#if defined _MSC_VER
#include <tmmintrin.h>
#else
#include <immintrin.h>
#endif

namespace vvenc {

#define CALC_EQUAL_COEFF_8PXLS(x1,x2,y1,y2,tmp0,tmp1,tmp2,tmp3,inter0,inter1,inter2,inter3,loadLocation)       \
{                                                                                                              \
inter0 = _mm_mul_epi32(x1, y1);                                                                                \
inter1 = _mm_mul_epi32(tmp0, tmp2);                                                                            \
inter2 = _mm_mul_epi32(x2, y2);                                                                                \
inter3 = _mm_mul_epi32(tmp1, tmp3);                                                                            \
inter2 = _mm_add_epi64(inter0, inter2);                                                                        \
inter3 = _mm_add_epi64(inter1, inter3);                                                                        \
inter0 = _mm_loadl_epi64(loadLocation);                                                                        \
inter3 = _mm_add_epi64(inter2, inter3);                                                                        \
inter1 = _mm_srli_si128(inter3, 8);                                                                            \
inter3 = _mm_add_epi64(inter1, inter3);                                                                        \
inter3 = _mm_add_epi64(inter0, inter3);                                                                        \
}

  template<X86_VEXT vext>
  static void simdHorizontalSobelFilter(Pel* const pPred, const int predStride, int *const pDerivate, const int derivateBufStride, const int width, const int height)
  {
    __m128i mmPred[4];
    __m128i mm2xPred[2];
    __m128i mmIntermediates[4];
    __m128i mmDerivate[2];

    assert(!(height % 2));
    assert(!(width % 4));

    /* Derivates of the rows and columns at the boundary are done at the end of this function */
    /* The value of col and row indicate the columns and rows for which the derivates have already been computed */
    for (int col = 1; (col + 2) < width; col += 2)
    {
      mmPred[0] = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(&pPred[0 * predStride + col - 1]));
      mmPred[1] = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(&pPred[1 * predStride + col - 1]));

      mmPred[0] = _mm_cvtepi16_epi32(mmPred[0]);
      mmPred[1] = _mm_cvtepi16_epi32(mmPred[1]);

      for (int row = 1; row < (height - 1); row += 2)
      {
        mmPred[2] = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(&pPred[(row + 1) * predStride + col - 1]));
        mmPred[3] = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(&pPred[(row + 2) * predStride + col - 1]));

        mmPred[2] = _mm_cvtepi16_epi32(mmPred[2]);
        mmPred[3] = _mm_cvtepi16_epi32(mmPred[3]);

        mm2xPred[0] = _mm_slli_epi32(mmPred[1], 1);
        mm2xPred[1] = _mm_slli_epi32(mmPred[2], 1);

        mmIntermediates[0] = _mm_add_epi32(mm2xPred[0], mmPred[0]);
        mmIntermediates[2] = _mm_add_epi32(mm2xPred[1], mmPred[1]);

        mmIntermediates[0] = _mm_add_epi32(mmIntermediates[0], mmPred[2]);
        mmIntermediates[2] = _mm_add_epi32(mmIntermediates[2], mmPred[3]);

        mmPred[0] = mmPred[2];
        mmPred[1] = mmPred[3];

        mmIntermediates[1] = _mm_srli_si128(mmIntermediates[0], 8);
        mmIntermediates[3] = _mm_srli_si128(mmIntermediates[2], 8);

        mmDerivate[0] = _mm_sub_epi32(mmIntermediates[1], mmIntermediates[0]);
        mmDerivate[1] = _mm_sub_epi32(mmIntermediates[3], mmIntermediates[2]);

        _mm_storel_epi64(reinterpret_cast<__m128i *> (&pDerivate[col + (row + 0) * derivateBufStride]), mmDerivate[0]);
        _mm_storel_epi64(reinterpret_cast<__m128i *> (&pDerivate[col + (row + 1) * derivateBufStride]), mmDerivate[1]);
      }
    }

    for (int j = 1; j < (height - 1); j++)
    {
      pDerivate[j * derivateBufStride] = pDerivate[j * derivateBufStride + 1];
      pDerivate[j * derivateBufStride + (width - 1)] = pDerivate[j * derivateBufStride + (width - 2)];
    }

    memcpy(pDerivate, pDerivate + derivateBufStride, width * sizeof(pDerivate[0]));
    memcpy(pDerivate + (height - 1) * derivateBufStride, pDerivate + (height - 2) * derivateBufStride, width * sizeof(pDerivate[0])
    );
  }

  template<X86_VEXT vext>
  static void simdVerticalSobelFilter(Pel* const pPred, const int predStride, int *const pDerivate, const int derivateBufStride, const int width, const int height)
  {
    __m128i mmPred[4];
    __m128i mmIntermediates[6];
    __m128i mmDerivate[2];

    assert(!(height % 2));
    assert(!(width % 4));

    /* Derivates of the rows and columns at the boundary are done at the end of this function */
    /* The value of col and row indicate the columns and rows for which the derivates have already been computed */
    for (int col = 1; col < (width - 1); col += 2)
    {
      mmPred[0] = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(&pPred[0 * predStride + col - 1]));
      mmPred[1] = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(&pPred[1 * predStride + col - 1]));

      mmPred[0] = _mm_cvtepi16_epi32(mmPred[0]);
      mmPred[1] = _mm_cvtepi16_epi32(mmPred[1]);

      for (int row = 1; row < (height - 1); row += 2)
      {
        mmPred[2] = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(&pPred[(row + 1) * predStride + col - 1]));
        mmPred[3] = _mm_loadl_epi64(reinterpret_cast<const __m128i *>(&pPred[(row + 2) * predStride + col - 1]));

        mmPred[2] = _mm_cvtepi16_epi32(mmPred[2]);
        mmPred[3] = _mm_cvtepi16_epi32(mmPred[3]);

        mmIntermediates[0] = _mm_sub_epi32(mmPred[2], mmPred[0]);
        mmIntermediates[3] = _mm_sub_epi32(mmPred[3], mmPred[1]);

        mmPred[0] = mmPred[2];
        mmPred[1] = mmPred[3];

        mmIntermediates[1] = _mm_srli_si128(mmIntermediates[0], 4);
        mmIntermediates[4] = _mm_srli_si128(mmIntermediates[3], 4);
        mmIntermediates[2] = _mm_srli_si128(mmIntermediates[0], 8);
        mmIntermediates[5] = _mm_srli_si128(mmIntermediates[3], 8);

        mmIntermediates[1] = _mm_slli_epi32(mmIntermediates[1], 1);
        mmIntermediates[4] = _mm_slli_epi32(mmIntermediates[4], 1);

        mmIntermediates[0] = _mm_add_epi32(mmIntermediates[0], mmIntermediates[2]);
        mmIntermediates[3] = _mm_add_epi32(mmIntermediates[3], mmIntermediates[5]);

        mmDerivate[0] = _mm_add_epi32(mmIntermediates[0], mmIntermediates[1]);
        mmDerivate[1] = _mm_add_epi32(mmIntermediates[3], mmIntermediates[4]);

        _mm_storel_epi64(reinterpret_cast<__m128i *> (&pDerivate[col + (row + 0) * derivateBufStride]), mmDerivate[0]);
        _mm_storel_epi64(reinterpret_cast<__m128i *> (&pDerivate[col + (row + 1) * derivateBufStride]), mmDerivate[1]);
      }
    }

    for (int j = 1; j < (height - 1); j++)
    {
      pDerivate[j * derivateBufStride] = pDerivate[j * derivateBufStride + 1];
      pDerivate[j * derivateBufStride + (width - 1)] = pDerivate[j * derivateBufStride + (width - 2)];
    }

    memcpy(pDerivate, pDerivate + derivateBufStride, width * sizeof(pDerivate[0]));
    memcpy(pDerivate + (height - 1) * derivateBufStride, pDerivate + (height - 2) * derivateBufStride, width * sizeof(pDerivate[0]));
  }

  template<X86_VEXT vext>
  static void simdEqualCoeffComputer(Pel* pResidue, int residueStride, int **ppDerivate, int derivateBufStride, int64_t(*pEqualCoeff)[7], int width, int height, bool b6Param)
  {
    __m128i mmFour;
    __m128i mmTmp[4];
    __m128i mmIntermediate[4];
    __m128i mmIndxK, mmIndxJ;
    __m128i mmResidue[2];
    __m128i mmC[12];

    // Add directly to indexes to get new index
    mmFour = _mm_set1_epi32(4);
    mmIndxJ = _mm_set1_epi32(-2);


    int n = b6Param ? 6 : 4;
    int idx1 = 0, idx2 = 0;
    idx1 = -2 * derivateBufStride - 4;
    idx2 = -derivateBufStride - 4;

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
          mmC[0] = _mm_loadu_si128((const __m128i*)&ppDerivate[0][idx1]);
          mmC[2] = _mm_loadu_si128((const __m128i*)&ppDerivate[1][idx1]);
          mmC[1] = _mm_mullo_epi32(mmIndxK, mmC[0]);
          mmC[3] = _mm_mullo_epi32(mmIndxK, mmC[2]);
          mmC[4] = _mm_mullo_epi32(mmIndxJ, mmC[0]);
          mmC[5] = _mm_mullo_epi32(mmIndxJ, mmC[2]);

          // mmC[6-11] for iC[0-5] of 2nd row of pixels
          mmC[6] = _mm_loadu_si128((const __m128i*)&ppDerivate[0][idx2]);
          mmC[8] = _mm_loadu_si128((const __m128i*)&ppDerivate[1][idx2]);
          mmC[7] = _mm_mullo_epi32(mmIndxK, mmC[6]);
          mmC[9] = _mm_mullo_epi32(mmIndxK, mmC[8]);
          mmC[10] = _mm_mullo_epi32(mmIndxJ, mmC[6]);
          mmC[11] = _mm_mullo_epi32(mmIndxJ, mmC[8]);
        }
        else
        {
          // mmC[0-3] for iC[0-3] of 1st row of pixels
          mmC[0] = _mm_loadu_si128((const __m128i*)&ppDerivate[0][idx1]);
          mmC[2] = _mm_loadu_si128((const __m128i*)&ppDerivate[1][idx1]);
          mmC[1] = _mm_mullo_epi32(mmIndxK, mmC[0]);
          mmC[3] = _mm_mullo_epi32(mmIndxJ, mmC[0]);
          mmTmp[0] = _mm_mullo_epi32(mmIndxJ, mmC[2]);
          mmTmp[1] = _mm_mullo_epi32(mmIndxK, mmC[2]);
          mmC[1] = _mm_add_epi32(mmC[1], mmTmp[0]);
          mmC[3] = _mm_sub_epi32(mmC[3], mmTmp[1]);

          // mmC[4-7] for iC[0-3] of 1st row of pixels
          mmC[4] = _mm_loadu_si128((const __m128i*)&ppDerivate[0][idx2]);
          mmC[6] = _mm_loadu_si128((const __m128i*)&ppDerivate[1][idx2]);
          mmC[5] = _mm_mullo_epi32(mmIndxK, mmC[4]);
          mmC[7] = _mm_mullo_epi32(mmIndxJ, mmC[4]);
          mmTmp[2] = _mm_mullo_epi32(mmIndxJ, mmC[6]);
          mmTmp[3] = _mm_mullo_epi32(mmIndxK, mmC[6]);
          mmC[5] = _mm_add_epi32(mmC[5], mmTmp[2]);
          mmC[7] = _mm_sub_epi32(mmC[7], mmTmp[3]);
        }

        // Residue
        mmResidue[0] = _mm_loadl_epi64((const __m128i*)&pResidue[idx1]);
        mmResidue[1] = _mm_loadl_epi64((const __m128i*)&pResidue[idx2]);
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
          _mm_storel_epi64((__m128i*)&pEqualCoeff[col + 1][col], mmIntermediate[3]);

          for (int row = col + 1; row < n; row++)
          {
            mmTmp[2] = _mm_srli_si128(mmC[0 + row], 4);
            mmTmp[3] = _mm_srli_si128(mmC[n + row], 4);
            CALC_EQUAL_COEFF_8PXLS(mmC[0 + col], mmC[n + col], mmC[0 + row], mmC[n + row], mmTmp[0], mmTmp[1], mmTmp[2], mmTmp[3], mmIntermediate[0], mmIntermediate[1], mmIntermediate[2], mmIntermediate[3], (const __m128i*)&pEqualCoeff[col + 1][row]);
            _mm_storel_epi64((__m128i*)&pEqualCoeff[col + 1][row], mmIntermediate[3]);
            _mm_storel_epi64((__m128i*)&pEqualCoeff[row + 1][col], mmIntermediate[3]);
          }

          mmTmp[2] = _mm_srli_si128(mmResidue[0], 4);
          mmTmp[3] = _mm_srli_si128(mmResidue[1], 4);
          CALC_EQUAL_COEFF_8PXLS(mmC[0 + col], mmC[n + col], mmResidue[0], mmResidue[1], mmTmp[0], mmTmp[1], mmTmp[2], mmTmp[3], mmIntermediate[0], mmIntermediate[1], mmIntermediate[2], mmIntermediate[3], (const __m128i*)&pEqualCoeff[col + 1][n]);
          _mm_storel_epi64((__m128i*)&pEqualCoeff[col + 1][n], mmIntermediate[3]);
        }
      }

      idx1 -= (width);
      idx2 -= (width);
    }
  }


  template <X86_VEXT vext>
  void AffineGradientSearch::_initAffineGradientSearchX86()
  {
    m_HorizontalSobelFilter = simdHorizontalSobelFilter<vext>;
    m_VerticalSobelFilter = simdVerticalSobelFilter<vext>;
    m_EqualCoeffComputer = simdEqualCoeffComputer<vext>;
  }

  template void AffineGradientSearch::_initAffineGradientSearchX86<SIMDX86>();

}

#endif //#ifdef TARGET_SIMD_X86
//! \}

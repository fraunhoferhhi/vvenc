/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVenc

(c) Copyright (2019-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVenc (“Fraunhofer Versatile Video Encoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Encoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Encoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Encoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Encoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Encoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Encoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Encoding Library. If You use the Fraunhofer Versatile Video Encoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Encoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Encoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Encoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Encoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
----------------------------------------------------------------------------- */
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

#endif //#ifdef TARGET_SIMD_X86
}
//! \}

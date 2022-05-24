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

// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "AffineGradientSearch.h"

namespace vvenc {

  //! \ingroup CommonLib
  //! \{

  // ====================================================================================================================
  // Private member functions
  // ====================================================================================================================

  AffineGradientSearch::AffineGradientSearch()
  {
    m_HorizontalSobelFilter = xHorizontalSobelFilter;
    m_VerticalSobelFilter = xVerticalSobelFilter;
    m_EqualCoeffComputer = xEqualCoeffComputer;

#if ENABLE_SIMD_OPT_AFFINE_ME
#ifdef TARGET_SIMD_X86
    initAffineGradientSearchX86();
#endif
#endif
  }

  void AffineGradientSearch::xHorizontalSobelFilter(Pel* const pPred, const int predStride, int *const pDerivate, const int derivateBufStride, const int width, const int height)
  {
    for (int j = 1; j < height - 1; j++)
    {
      for (int k = 1; k < width - 1; k++)
      {
        int iCenter = j * predStride + k;

        pDerivate[j * derivateBufStride + k] =
            (pPred[iCenter + 1 - predStride] -
             pPred[iCenter - 1 - predStride] +
            (pPred[iCenter + 1] << 1) -
            (pPred[iCenter - 1] << 1) +
             pPred[iCenter + 1 + predStride] -
             pPred[iCenter - 1 + predStride]);
      }
      pDerivate[j * derivateBufStride] = pDerivate[j * derivateBufStride + 1];
      pDerivate[j * derivateBufStride + width - 1] = pDerivate[j * derivateBufStride + width - 2];
    }

    pDerivate[0] = pDerivate[derivateBufStride + 1];
    pDerivate[width - 1] = pDerivate[derivateBufStride + width - 2];
    pDerivate[(height - 1) * derivateBufStride] = pDerivate[(height - 2) * derivateBufStride + 1];
    pDerivate[(height - 1) * derivateBufStride + width - 1] = pDerivate[(height - 2) * derivateBufStride + (width - 2)];

    for (int j = 1; j < width - 1; j++)
    {
      pDerivate[j] = pDerivate[derivateBufStride + j];
      pDerivate[(height - 1) * derivateBufStride + j] = pDerivate[(height - 2) * derivateBufStride + j];
    }
  }

  void AffineGradientSearch::xVerticalSobelFilter(Pel* const pPred, const int predStride, int *const pDerivate, const int derivateBufStride, const int width, const int height)
  {
    for (int k = 1; k < width - 1; k++)
    {
      for (int j = 1; j < height - 1; j++)
      {
        int iCenter = j * predStride + k;

        pDerivate[j * derivateBufStride + k] =
           (pPred[iCenter + predStride - 1] -
            pPred[iCenter - predStride - 1] +
           (pPred[iCenter + predStride] << 1) -
           (pPred[iCenter - predStride] << 1) +
            pPred[iCenter + predStride + 1] -
            pPred[iCenter - predStride + 1]);
      }

      pDerivate[k] = pDerivate[derivateBufStride + k];
      pDerivate[(height - 1) * derivateBufStride + k] = pDerivate[(height - 2) * derivateBufStride + k];
    }

    pDerivate[0] = pDerivate[derivateBufStride + 1];
    pDerivate[width - 1] = pDerivate[derivateBufStride + width - 2];
    pDerivate[(height - 1) * derivateBufStride] = pDerivate[(height - 2) * derivateBufStride + 1];
    pDerivate[(height - 1) * derivateBufStride + width - 1] = pDerivate[(height - 2) * derivateBufStride + (width - 2)];

    for (int j = 1; j < height - 1; j++)
    {
      pDerivate[j * derivateBufStride] = pDerivate[j * derivateBufStride + 1];
      pDerivate[j * derivateBufStride + width - 1] = pDerivate[j * derivateBufStride + width - 2];
    }
  }

  void AffineGradientSearch::xEqualCoeffComputer(Pel* pResidue, int residueStride, int **ppDerivate, int derivateBufStride, int64_t(*pEqualCoeff)[7], int width, int height, bool b6Param)
  {
    int affineParamNum = b6Param ? 6 : 4;

    for (int j = 0; j != height; j++)
    {
      int cy = ((j >> 2) << 2) + 2;
      for (int k = 0; k != width; k++)
      {
        int iC[6];

        int idx = j * derivateBufStride + k;
        int cx = ((k >> 2) << 2) + 2;
        if (!b6Param)
        {
          iC[0] = ppDerivate[0][idx];
          iC[1] = cx * ppDerivate[0][idx] + cy * ppDerivate[1][idx];
          iC[2] = ppDerivate[1][idx];
          iC[3] = cy * ppDerivate[0][idx] - cx * ppDerivate[1][idx];
        }
        else
        {
          iC[0] = ppDerivate[0][idx];
          iC[1] = cx * ppDerivate[0][idx];
          iC[2] = ppDerivate[1][idx];
          iC[3] = cx * ppDerivate[1][idx];
          iC[4] = cy * ppDerivate[0][idx];
          iC[5] = cy * ppDerivate[1][idx];
        }
        for (int col = 0; col < affineParamNum; col++)
        {
          for (int row = 0; row < affineParamNum; row++)
          {
            pEqualCoeff[col + 1][row] += (int64_t)iC[col] * iC[row];
          }
          pEqualCoeff[col + 1][affineParamNum] += ((int64_t)iC[col] * pResidue[idx]) *(1<< 3);
        }
      }
    }
  }

}

//! \}

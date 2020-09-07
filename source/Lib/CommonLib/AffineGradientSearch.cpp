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
          pEqualCoeff[col + 1][affineParamNum] += ((int64_t)iC[col] * pResidue[idx]) << 3;
        }
      }
    }
  }

}

//! \}

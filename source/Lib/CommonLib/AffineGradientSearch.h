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


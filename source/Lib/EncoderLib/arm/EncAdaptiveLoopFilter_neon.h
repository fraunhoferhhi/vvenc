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
/** \file     EncAdaptiveLoopFilter_neon.h
    \brief    NEON ALF statistics kernels that the SVE file also uses
*/

#pragma once

#include "EncoderLib/EncAdaptiveLoopFilter.h"

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_ALF

namespace vvenc
{

// Whether getPreBlkStatsAccumEqualBins_neon computes a block with its equal-bins kernel: with 13 coefficients every
// block the encoder hands it, with 7 coefficients only blocks whose four clipping bins are all equal; it gives the
// others to the 16-pair kernel.
static inline bool alfEqualBinsKernelTakes( const int numCoeff, const int equalBinsMask )
{
  return numCoeff != 7 || equalBinsMask == ( 1 << ( AdaptiveLoopFilter::AlfNumClippingValues - 1 ) ) - 1;
}

void getPreBlkStatsAccumEqualBins_neon( AlfCovariance& alfCovariance, const AlfFilterShape& shape, const Pel* ELocal,
                                        const Pel yLocal[4][4], const int equalBinsMask );

} // namespace vvenc

#endif

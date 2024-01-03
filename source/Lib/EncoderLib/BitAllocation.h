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
/** \file     BitAllocation.h
\brief    Bit allocation class for QP adaptation and, possibly, rate control (header)
*/

#pragma once

#include "CommonLib/Slice.h"
#include "CommonLib/Unit.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

  double filterAndCalculateAverageActivity ( const Pel* pSrc,
                                             const int iSrcStride,
                                             const int height,
                                             const int width,
                                             const Pel* pSM1,
                                             const int iSM1Stride,
                                             const Pel* pSM2,
                                             const int iSM2Stride,
                                             uint32_t frameRate,
                                             const uint32_t bitDepth,
                                             const bool isUHD,
                                             unsigned* minVisAct,
                                             unsigned* spVisAct);

  // BitAllocation functions
  namespace BitAllocation
  {
    int applyQPAdaptationSlice ( const Slice* slice,
                                 const VVEncCfg* encCfg,
                                 const int sliceQP,
                                 const double sliceLambda,
                                 uint16_t* const picVisActLuma,
                                 std::vector<int>& ctuPumpRedQP,
                                 std::vector<uint8_t>* ctuRCQPMemory,
                                 int* const optChromaQPOffsets,
                                 const uint8_t* minNoiseLevels,
                                 const uint32_t ctuStartAddr,
                                 const uint32_t ctuBoundingAddr );
    int applyQPAdaptationSubCtu( const Slice* slice,
                                 const VVEncCfg* encCfg,
                                 const Area& lumaArea,
                                 const uint8_t* minNoiseLevels );
    int getCtuPumpingReducingQP( const Slice* slice,
                                 const CPelBuf& origY,
                                 const Distortion uiSadBestForQPA,
                                 std::vector<int>& ctuPumpRedQP,
                                 const uint32_t ctuRsAddr,
                                 const int baseQP,
                                 const bool isBIM );
  }

} // namespace vvenc

//! \}

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

/*
 * \ingroup CommonLib
 * \file    InitARM.cpp
 * \brief   Initialize encoder SIMD functions.
 */

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/InterpolationFilter.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/Buffer.h"
#include "CommonLib/TrQuant_EMT.h"
#include "CommonLib/IntraPrediction.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/Picture.h"
#include "CommonLib/MCTF.h"

#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonLib/SampleAdaptiveOffset.h"

namespace vvenc
{

#ifdef TARGET_SIMD_ARM

#if ENABLE_SIMD_OPT_MCIF
void InterpolationFilter::initInterpolationFilterARM()
{
  auto vext = read_arm_extension_flags();
  if( vext >= NEON )
  {
    _initInterpolationFilterARM<NEON>();
  }
}
#endif

#if ENABLE_SIMD_OPT_BUFFER
void PelBufferOps::initPelBufOpsARM()
{
  auto vext = read_arm_extension_flags();
  if( vext >= NEON )
  {
    _initPelBufOpsARM<NEON>();
  }
}
#endif

#if ENABLE_SIMD_OPT_DIST
void RdCost::initRdCostARM()
{
  auto vext = read_arm_extension_flags();
  if( vext >= NEON )
  {
    _initRdCostARM<NEON>();
  }
}
#endif

#if ENABLE_SIMD_OPT_MCTF
void MCTF::initMCTF_ARM()
{
  auto vext = read_arm_extension_flags();
  if( vext >= NEON )
  {
    _initMCTF_ARM<NEON>();
  }
}
#endif  // ENABLE_SIMD_OPT_MCTF

#if ENABLE_SIMD_TRAFO
void TCoeffOps::initTCoeffOpsARM()
{
  auto vext = read_arm_extension_flags();
  if( vext >= NEON )
  {
    _initTCoeffOpsARM<NEON>();
  }
}
#endif

#if ENABLE_SIMD_OPT_BDOF
void InterPredInterpolation::initInterPredictionARM()
{
  auto vext = read_arm_extension_flags();
  switch (vext){
    case NEON:
      _initInterPredictionARM<NEON>();
      break;
    default:
      break;
  }
}
#endif



#endif   // TARGET_SIMD_ARM

}   // namespace

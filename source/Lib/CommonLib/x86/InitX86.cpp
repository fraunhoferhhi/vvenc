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
 * \file    InitX86.cpp
 * \brief   Initialize encoder SIMD functions.
 */

#include "CommonDefX86.h"
#include "InterpolationFilter.h"
#include "TrQuant.h"
#include "RdCost.h"
#include "Unit.h"
#include "LoopFilter.h"
#include "AdaptiveLoopFilter.h"
#include "SampleAdaptiveOffset.h"
#include "InterPrediction.h"
#include "IntraPrediction.h"
#include "AffineGradientSearch.h"
#include "MCTF.h"
#include "TrQuant_EMT.h"
#include "QuantRDOQ2.h"
#include "SEIFilmGrainAnalyzer.h"

#ifdef TARGET_SIMD_X86

//! \ingroup CommonLib
//! \{

#  if defined( REAL_TARGET_X86 ) \
    || ( defined( SIMD_EVERYWHERE_EXTENSION_LEVEL_ID ) && SIMD_EVERYWHERE_EXTENSION_LEVEL_ID >= X86_SIMD_AVX2 )
#    define ENABLE_AVX2_IMPLEMENTATIONS 1
#  else
#    define ENABLE_AVX2_IMPLEMENTATIONS 0
#  endif

namespace vvenc {

#if ENABLE_SIMD_OPT_MCIF
void InterpolationFilter::initInterpolationFilterX86( /*int iBitDepthY, int iBitDepthC*/ )
{
  auto vext = read_x86_extension_flags();
  switch (vext){
  case AVX512:
  case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
    _initInterpolationFilterX86<AVX2>(/*iBitDepthY, iBitDepthC*/);
    break;
#endif
  case AVX:
    //_initInterpolationFilterX86<AVX>(/*iBitDepthY, iBitDepthC*/);
    //break;
  case SSE42:
  case SSE41:
    _initInterpolationFilterX86<SSE41>(/*iBitDepthY, iBitDepthC*/);
    break;
  default:
    break;
  }
}
#endif

#if ENABLE_SIMD_OPT_BUFFER
void PelBufferOps::initPelBufOpsX86()
{
  if( isInitX86Done )
    return;

  isInitX86Done = true;

  auto vext = read_x86_extension_flags();
  switch (vext){
    case AVX512:
    case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
      _initPelBufOpsX86<AVX2>();
      break;
#endif
    case AVX:
      //_initPelBufOpsX86<AVX>();
      //break;
    case SSE42:
    case SSE41:
      _initPelBufOpsX86<SSE41>();
      break;
    default:
      break;
  }
}
#endif


#if ENABLE_SIMD_DBLF
void LoopFilter::initLoopFilterX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext)
  {
  case AVX512:
  case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
    _initLoopFilterX86<AVX2>();
    break;
#endif
  case AVX:
    //_initLoopFilterX86<AVX>();
    //break;
  case SSE42:
  case SSE41:
    _initLoopFilterX86<SSE41>();
    break;
  default:
    break;
  }
}
#endif


#if ENABLE_SIMD_OPT_DIST
void RdCost::initRdCostX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){ 
    case AVX512:
    case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
#if defined( _MSC_VER ) && _MSC_VER >= 1938 && _MSC_VER < 1939
#else
      _initRdCostX86<AVX2>();
      break;
#endif
#endif
    case AVX:
      //_initRdCostX86<AVX>();
      //break;
    case SSE42:
    case SSE41:
      _initRdCostX86<SSE41>();
      break;
    default:
      break;
  }
}
#endif

#if ENABLE_SIMD_OPT_ALF
void AdaptiveLoopFilter::initAdaptiveLoopFilterX86()
{
  auto vext = read_x86_extension_flags();
  switch( vext )
  {
  case AVX512:
  case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
    _initAdaptiveLoopFilterX86<AVX2>();
    break;
#endif
  case AVX:
    //_initAdaptiveLoopFilterX86<AVX>();
    //break;
  case SSE42:
  case SSE41:
    _initAdaptiveLoopFilterX86<SSE41>();
    break;
  default:
    break;
  }
}
#endif

#if ENABLE_SIMD_OPT_SAO
void SampleAdaptiveOffset::initSampleAdaptiveOffsetX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){
    case AVX512:
    case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
      _initSampleAdaptiveOffsetX86<AVX2>();
      break;
#endif
    case AVX:
      //_initSampleAdaptiveOffsetX86<AVX>();
      //break;
    case SSE42:
    case SSE41:
      _initSampleAdaptiveOffsetX86<SSE41>();
      break;
    default:
      break;
  }
}

#endif

#if ENABLE_SIMD_OPT_BDOF
void InterPredInterpolation::initInterPredictionX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){
    case AVX512:
    case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
      _initInterPredictionX86<AVX2>();
      break;
#endif
    case AVX:
      //_initInterPredictionX86<AVX>();
      //break;
    case SSE42:
    case SSE41:
      _initInterPredictionX86<SSE41>();
      break;
    default:
      break;
  }
}
#endif

#if ENABLE_SIMD_OPT_AFFINE_ME
void AffineGradientSearch::initAffineGradientSearchX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext) {
  case AVX512:
  case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
    _initAffineGradientSearchX86<AVX2>();
    break;
#endif
  case AVX:
    //_initAffineGradientSearchX86<AVX>();
    //break;
  case SSE42:
  case SSE41:
    _initAffineGradientSearchX86<SSE41>();
    break;
  default:
    break;
  }
}
#endif

#if ENABLE_SIMD_OPT_INTRAPRED
void IntraPrediction::initIntraPredictionX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){
    case AVX512:
    case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
      _initIntraPredictionX86<AVX2>();
      break;
#endif
    case AVX:
      //_initIntraPredictionX86<AVX>();
      //break;
    case SSE42:
    case SSE41:
      _initIntraPredictionX86<SSE41>();
      break;
    default:
      break;
  }
}

#endif
#if ENABLE_SIMD_OPT_MCTF
void MCTF::initMCTF_X86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){
    case AVX512:
    case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
      _initMCTF_X86<AVX2 >();
      break;
#endif
    case AVX:
      //_initMCTF_X86<AVX  >();
      //break;
    case SSE42:
      //_initMCTF_X86<SSE42>();
      //break;
    case SSE41:
      _initMCTF_X86<SSE41>();
      break;
    default:
      break;
  }
}

#endif
#if ENABLE_SIMD_TRAFO
void TCoeffOps::initTCoeffOpsX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){
    case AVX512:
    case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
      _initTCoeffOpsX86<AVX2 >();
      break;
#endif
    case AVX:
      //_initTCoeffOpsX86<AVX  >();
      //break;
    case SSE42:
      //_initTCoeffOpsX86<SSE42>();
      //break;
    case SSE41:
      _initTCoeffOpsX86<SSE41>();
      break;
    default:
      break;
  }
}

void TrQuant::initTrQuantX86()
{
  auto vext = read_x86_extension_flags();
  switch( vext )
  {
  case AVX512:
  case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
    _initTrQuantX86<AVX2 >();
    break;
#endif
  case AVX:
    //_initTrQuantX86<AVX  >();
    //break;
  case SSE42:
    //_initTrQuantX86<SSE42>();
    //break;
  case SSE41:
    _initTrQuantX86<SSE41>();
    break;
  default:
  break;
  }
}

#endif

#if ENABLE_SIMD_OPT_QUANT

void Quant::initQuantX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){
    case AVX512:
    case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
      _initQuantX86<AVX2>();
      break;
#endif
    case AVX:
      //_initQuantX86<AVX>();
      //break;
    case SSE42:
    case SSE41:
      _initQuantX86<SSE41>();
      break;
    default:
      break;
  }
}

void DepQuant::initDepQuantX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){
  case AVX512:
  case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
    _initDepQuantX86<AVX2>();
    break;
#endif
  case AVX:
  case SSE42:
    _initDepQuantX86<SSE42>();
    break;
  case SSE41:
    _initDepQuantX86<SSE41>();
    break;
  default:
    break;
  }
}

#endif

#if ENABLE_SIMD_OPT_FGA
void Canny::initFGACannyX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){
    case AVX512:
    case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
      _initFGACannyX86<AVX2>();
      break;
#endif
    case SSE42:
    case SSE41:
      _initFGACannyX86<SSE41>();
      break;
    default:
      break;
  }
}
void Morph::initFGAMorphX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){
    case AVX512:
    case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
      _initFGAMorphX86<AVX2>();
      break;
#endif
    case SSE42:
    case SSE41:
      _initFGAMorphX86<SSE41>();
      break;
    default:
      break;
  }
}

void FGAnalyzer::initFGAnalyzerX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){
    case AVX512:
    case AVX2:
#if ENABLE_AVX2_IMPLEMENTATIONS
      _initFGAnalyzerX86<AVX2>();
      break;
#endif
    case SSE42:
    case SSE41:
      _initFGAnalyzerX86<SSE41>();
      break;
    default:
      break;
  }
}
#endif

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


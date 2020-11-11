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

#ifdef TARGET_SIMD_X86

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if ENABLE_SIMD_OPT_MCIF
void InterpolationFilter::initInterpolationFilterX86( /*int iBitDepthY, int iBitDepthC*/ )
{
  auto vext = read_x86_extension_flags();
  switch (vext){
  case AVX512:
  case AVX2:
    _initInterpolationFilterX86<AVX2>(/*iBitDepthY, iBitDepthC*/);
    break;
  case AVX:
    _initInterpolationFilterX86<AVX>(/*iBitDepthY, iBitDepthC*/);
    break;
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
  if ( isInitX86Done )
    return;

  auto vext = read_x86_extension_flags();
  switch (vext){
    case AVX512:
    case AVX2:
      _initPelBufOpsX86<AVX2>();
      break;
    case AVX:
      _initPelBufOpsX86<AVX>();
      break;
    case SSE42:
    case SSE41:
      _initPelBufOpsX86<SSE41>();
      break;
    default:
      break;
  }

  isInitX86Done = true;
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
    _initLoopFilterX86<AVX2>();
    break;
  case AVX:
    _initLoopFilterX86<AVX>();
    break;
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
      _initRdCostX86<AVX2>();
      break;
    case AVX:
      _initRdCostX86<AVX>();
      break;
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
    _initAdaptiveLoopFilterX86<AVX2>();
    break;
  case AVX:
    _initAdaptiveLoopFilterX86<AVX>();
    break;
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
      _initSampleAdaptiveOffsetX86<AVX2>();
      break;
    case AVX:
      _initSampleAdaptiveOffsetX86<AVX>();
      break;
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
      _initInterPredictionX86<AVX2>();
      break;
    case AVX:
      _initInterPredictionX86<AVX>();
      break;
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
    _initAffineGradientSearchX86<AVX2>();
    break;
  case AVX:
    _initAffineGradientSearchX86<AVX>();
    break;
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
      _initIntraPredictionX86<AVX2>();
      break;
    case AVX:
      _initIntraPredictionX86<AVX>();
      break;
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
      _initMCTF_X86<AVX2 >();
      break;
    case AVX:
      _initMCTF_X86<AVX  >();
      break;
    case SSE42:
      _initMCTF_X86<SSE42>();
      break;
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
      _initTCoeffOpsX86<AVX2 >();
      break;
    case AVX:
      _initTCoeffOpsX86<AVX  >();
      break;
    case SSE42:
      _initTCoeffOpsX86<SSE42>();
      break;
    case SSE41:
      _initTCoeffOpsX86<SSE41>();
      break;
    default:
      break;
  }
}

#endif

#if ENABLE_SIMD_OPT_QUANT
void QuantRDOQ2::initQuantX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){
  case AVX512:
  case AVX2:
    _initQuantX86<AVX2>();
    break;
  case AVX:
    _initQuantX86<AVX>();
    break;
  case SSE42:
  case SSE41:
    _initQuantX86<SSE41>();
    break;
  default:
    break;
  }

}
void Quant::initQuantX86()
{
  auto vext = read_x86_extension_flags();
  switch (vext){
    case AVX512:
    case AVX2:
      _initQuantX86<AVX2>();
      break;
    case AVX:
      _initQuantX86<AVX>();
      break;
    case SSE42:
    case SSE41:
      _initQuantX86<SSE41>();
      break;
    default:
      break;
  }

}


#endif

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


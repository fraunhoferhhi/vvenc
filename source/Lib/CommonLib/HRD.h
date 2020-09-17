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


#pragma once

#include "Common.h"
#include "SEI.h"

namespace vvenc {

struct OlsHrdParams
{
  bool     fixedPicRateGeneralFlag;
  bool     fixedPicRateWithinCvsFlag;
  bool     lowDelayHrdFlag;

  uint32_t elementDurationInTcMinus1;
  uint32_t bitRateValueMinus1[MAX_CPB_CNT][2];
  uint32_t cpbSizeValueMinus1[MAX_CPB_CNT][2];
  uint32_t duCpbSizeValueMinus1[MAX_CPB_CNT][2];
  uint32_t duBitRateValueMinus1[MAX_CPB_CNT][2];
  bool     cbrFlag[MAX_CPB_CNT][2];

  OlsHrdParams()
    : fixedPicRateGeneralFlag   (false)
    , fixedPicRateWithinCvsFlag (false)
    , lowDelayHrdFlag           (false)
    , elementDurationInTcMinus1 (0)
  {
    memset( bitRateValueMinus1,   0, sizeof(bitRateValueMinus1));
    memset( cpbSizeValueMinus1,   0, sizeof(cpbSizeValueMinus1));
    memset( duCpbSizeValueMinus1, 0, sizeof(duCpbSizeValueMinus1));
    memset( duBitRateValueMinus1, 0, sizeof(duBitRateValueMinus1));
    memset( cbrFlag,              0, sizeof(cbrFlag));
  }
};

struct GeneralHrdParams
{
  uint32_t numUnitsInTick;
  uint32_t timeScale;
  bool     generalNalHrdParamsPresent;
  bool     generalVclHrdParamsPresent;
  bool     generalSamePicTimingInAllOlsFlag;
  uint32_t tickDivisorMinus2;
  bool     generalDecodingUnitHrdParamsPresent;
  uint32_t bitRateScale;
  uint32_t cpbSizeScale;
  uint32_t cpbSizeDuScale;
  uint32_t hrdCpbCntMinus1;

  GeneralHrdParams()
    : generalNalHrdParamsPresent              (false)
    , generalVclHrdParamsPresent              (false)
    , generalSamePicTimingInAllOlsFlag        (true)
    , tickDivisorMinus2                       (0)
    , generalDecodingUnitHrdParamsPresent     (false)
    , bitRateScale                            (0)
    , cpbSizeScale                            (0)
    , cpbSizeDuScale                          (0)
    , hrdCpbCntMinus1                         (0)
  {}

  bool operator==(const GeneralHrdParams& other) const
  {
    return (numUnitsInTick == other.numUnitsInTick
      && timeScale == other.timeScale
      && generalNalHrdParamsPresent == other.generalNalHrdParamsPresent
      && generalVclHrdParamsPresent == other.generalVclHrdParamsPresent
      && generalSamePicTimingInAllOlsFlag == other.generalSamePicTimingInAllOlsFlag
      && generalDecodingUnitHrdParamsPresent == other.generalDecodingUnitHrdParamsPresent
      && (generalDecodingUnitHrdParamsPresent ? (tickDivisorMinus2 == other.tickDivisorMinus2): 1)
      && bitRateScale == other.bitRateScale
      && cpbSizeScale == other.cpbSizeScale
      && (generalDecodingUnitHrdParamsPresent ? (cpbSizeDuScale == other.cpbSizeDuScale) : 1)
      && hrdCpbCntMinus1 == other.hrdCpbCntMinus1
      );
  }

  GeneralHrdParams& operator=(const GeneralHrdParams& input)
  {
    numUnitsInTick = input.numUnitsInTick;
    timeScale = input.timeScale;
    generalNalHrdParamsPresent = input.generalNalHrdParamsPresent;
    generalVclHrdParamsPresent = input.generalVclHrdParamsPresent;
    generalSamePicTimingInAllOlsFlag = input.generalSamePicTimingInAllOlsFlag;
    generalDecodingUnitHrdParamsPresent = input.generalDecodingUnitHrdParamsPresent;
    if (input.generalDecodingUnitHrdParamsPresent)
    {
      tickDivisorMinus2 = input.tickDivisorMinus2;
    }
    bitRateScale = input.bitRateScale;
    cpbSizeScale = input.cpbSizeScale;
    if (input.generalDecodingUnitHrdParamsPresent)
    {
      cpbSizeDuScale = input.cpbSizeDuScale;
    }
    hrdCpbCntMinus1 = input.hrdCpbCntMinus1;
    return *this;
  }
};

struct HRD
{
  HRD()
  : bufferingPeriodInitialized  (false)
  , pictureTimingAvailable      (false)	
  {};

  GeneralHrdParams      generalHrdParams;
  OlsHrdParams          olsHrdParams[MAX_TLAYER];
  bool                  bufferingPeriodInitialized;
  bool                  pictureTimingAvailable;
  SEIBufferingPeriod    bufferingPeriodSEI;
  SEIPictureTiming      pictureTimingSEI;
};

} // namespace vvenc

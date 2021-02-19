/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software,
especially patent licenses, a separate Agreement needs to be closed. 
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2019-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */


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

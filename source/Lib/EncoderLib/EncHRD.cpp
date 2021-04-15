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

Copyright (c) 2019-2021, Fraunhofer-Gesellschaft zur F�rderung der angewandten Forschung e.V.
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

#include "EncHRD.h"
#include "CommonLib/ProfileLevelTier.h"

namespace vvenc {

// calculate scale value of bitrate and initial delay
int EncHRD::xCalcScale(int x)
{
  if (x==0)
  {
    return 0;
  }
  uint32_t mask = 0xffffffff;
  int scaleValue = 32;

  while ((x&mask) != 0)
  {
    scaleValue--;
    mask = (mask >> 1);
  }

  return scaleValue;
}

void EncHRD::initHRDParameters(const VVEncCfg& encCfg, const SPS& sps)
{
//  if (!encCfg.m_hrdParametersPresent && !encCfg.getCpbSaturationEnabled())
//  {
//    return;
//  }
  ProfileLevelTierFeatures profileLevelTierFeatures;
  profileLevelTierFeatures.extractPTLInformation( sps );

  bool useSubCpbParams = false; //encCfg.getNoPicPartitionFlag() == false;
  int bitRate = encCfg.m_RCTargetBitrate;
  int cpbSize = profileLevelTierFeatures.getCpbSizeInBits();

  CHECK(!(cpbSize != 0), "Unspecified error");  // CPB size may not be equal to zero. ToDo: have a better default and check for level constraints

  switch (encCfg.m_FrameRate)
  {
  case 24:
    generalHrdParams.numUnitsInTick = 1125000;    generalHrdParams.timeScale = 27000000;
    break;
  case 25:
    generalHrdParams.numUnitsInTick = 1080000;    generalHrdParams.timeScale = 27000000;
    break;
  case 30:
    generalHrdParams.numUnitsInTick = 900900;     generalHrdParams.timeScale = 27000000;
    break;
  case 50:
    generalHrdParams.numUnitsInTick = 540000;     generalHrdParams.timeScale = 27000000;
    break;
  case 60:
    generalHrdParams.numUnitsInTick = 450450;     generalHrdParams.timeScale = 27000000;
    break;
  default:
    generalHrdParams.numUnitsInTick = 1001;       generalHrdParams.timeScale = 60000;
    break;
  }

  if (encCfg.m_temporalSubsampleRatio > 1)
  {
    uint32_t temporalSubsampleRatio = encCfg.m_temporalSubsampleRatio;
    if (double(generalHrdParams.numUnitsInTick) * temporalSubsampleRatio > std::numeric_limits<uint32_t>::max())
    {
      generalHrdParams.timeScale = generalHrdParams.timeScale / temporalSubsampleRatio;
    }
    else
    {
      generalHrdParams.numUnitsInTick = generalHrdParams.numUnitsInTick * temporalSubsampleRatio;
    }
  }
  bool rateCnt = (bitRate > 0);

  generalHrdParams.generalNalHrdParamsPresent = rateCnt;
  generalHrdParams.generalVclHrdParamsPresent = rateCnt;

  generalHrdParams.generalSamePicTimingInAllOlsFlag = true;
  useSubCpbParams &= (generalHrdParams.generalNalHrdParamsPresent || generalHrdParams.generalVclHrdParamsPresent);
  generalHrdParams.generalDecodingUnitHrdParamsPresent = useSubCpbParams;

  if (generalHrdParams.generalDecodingUnitHrdParamsPresent)
  {
    generalHrdParams.tickDivisorMinus2 = (100 - 2);
  }

  if (xCalcScale(bitRate) <= 6)
  {
    generalHrdParams.bitRateScale = 0;
  }
  else
  {
    generalHrdParams.bitRateScale = xCalcScale(bitRate) - 6;
  }

  if (xCalcScale(cpbSize) <= 4)
  {
    generalHrdParams.cpbSizeScale = 0;
  }
  else
  {
    generalHrdParams.cpbSizeScale = xCalcScale(cpbSize) - 4;
  }

  generalHrdParams.cpbSizeDuScale = 6;                                     // in units of 2^( 4 + 6 ) = 1,024 bit
  generalHrdParams.hrdCpbCntMinus1 = 0;


  // Note: parameters for all temporal layers are initialized with the same values
  int i, j;
  uint32_t bitrateValue, cpbSizeValue;
  uint32_t duCpbSizeValue;
  uint32_t duBitRateValue = 0;

  for (i = 0; i < MAX_TLAYER; i++)
  {
    olsHrdParams[i].fixedPicRateGeneralFlag = true;
    olsHrdParams[i].fixedPicRateWithinCvsFlag = true;
    olsHrdParams[i].elementDurationInTcMinus1 = 0;
    olsHrdParams[i].lowDelayHrdFlag = false;

    //! \todo check for possible PTL violations
    // BitRate[ i ] = ( bit_rate_value_minus1[ i ] + 1 ) * 2^( 6 + bit_rate_scale )
    bitrateValue = bitRate / (1 << (6 + generalHrdParams.bitRateScale));      // bitRate is in bits, so it needs to be scaled down
                                                                              // CpbSize[ i ] = ( cpb_size_value_minus1[ i ] + 1 ) * 2^( 4 + cpb_size_scale )
    cpbSizeValue = cpbSize / (1 << (4 + generalHrdParams.cpbSizeScale));      // using bitRate results in 1 second CPB size

                                                                              // DU CPB size could be smaller (i.e. bitrateValue / number of DUs), but we don't know
                                                                              // in how many DUs the slice segment settings will result
    duCpbSizeValue = bitrateValue;
    duBitRateValue = cpbSizeValue;

    for (j = 0; j < (generalHrdParams.hrdCpbCntMinus1 + 1); j++)
    {
      olsHrdParams[i].bitRateValueMinus1[j][0] =  bitrateValue - 1;
      olsHrdParams[i].cpbSizeValueMinus1[j][0] =  cpbSizeValue - 1;
      olsHrdParams[i].duCpbSizeValueMinus1[j][0] =  duCpbSizeValue - 1;
      olsHrdParams[i].duBitRateValueMinus1[j][0] =  duBitRateValue - 1;
      olsHrdParams[i].cbrFlag[j][0] =  false;

      olsHrdParams[i].bitRateValueMinus1[j][1] =  bitrateValue - 1;
      olsHrdParams[i].cpbSizeValueMinus1[j][1] =  cpbSizeValue - 1;
      olsHrdParams[i].duCpbSizeValueMinus1[j][1] =  duCpbSizeValue - 1;
      olsHrdParams[i].duBitRateValueMinus1[j][1] =  duBitRateValue - 1;
      olsHrdParams[i].cbrFlag[j][1] =  false;
    }
  }
}

} //namespace

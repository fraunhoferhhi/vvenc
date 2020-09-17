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

void EncHRD::initHRDParameters(const EncCfg& encCfg, const SPS& sps)
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
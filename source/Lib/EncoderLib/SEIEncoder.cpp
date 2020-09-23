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


#include "SEIEncoder.h"

#include "../../../include/vvenc/EncCfg.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/SEI.h"
#include "CommonLib/PicYuvMD5.h"
#include "CommonLib/HRD.h"
#include "CommonLib/Slice.h"
#include "EncHRD.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

void SEIEncoder::init( const EncCfg& encCfg, EncHRD& encHRD)
{
  m_pcEncCfg      = &encCfg;
  m_pcEncHRD      = &encHRD;
  m_isInitialized = true;
  ::memset(m_lastBPSEI,  0, sizeof(m_lastBPSEI));
  ::memset(m_totalCoded, 0, sizeof(m_totalCoded));
}

void SEIEncoder::initBufferingPeriodSEI( SEIBufferingPeriod& bpSei, bool noLeadingPictures)
{
  CHECK(!(m_isInitialized), "bufferingPeriodSEI already initialized");

  uint32_t uiInitialCpbRemovalDelay = (90000/2);                      // 0.5 sec
  bpSei.bpNalCpbParamsPresent = true;
  bpSei.bpVclCpbParamsPresent = true;
  bpSei.bpMaxSubLayers = m_pcEncCfg->m_maxTempLayer;
  bpSei.bpCpbCnt = 1;
  for(int i=0; i < bpSei.bpMaxSubLayers; i++)
  {
    for(int j=0; j < bpSei.bpCpbCnt; j++)
    {
      bpSei.initialCpbRemovalDelay[j][i][0] = uiInitialCpbRemovalDelay;
      bpSei.initialCpbRemovalDelay[j][i][1] = uiInitialCpbRemovalDelay;
      bpSei.initialCpbRemovalOffset[j][i][0] = uiInitialCpbRemovalDelay;
      bpSei.initialCpbRemovalOffset[j][i][1] = uiInitialCpbRemovalDelay;
    }
  }
  // We don't set concatenation_flag here. max_initial_removal_delay_for_concatenation depends on the usage scenario.
  // The parameters could be added to config file, but as long as the initialisation of generic buffering parameters is
  // not controllable, it does not seem to make sense to provide settings for these.
  bpSei.concatenationFlag = false;
  bpSei.maxInitialRemovalDelayForConcatenation = uiInitialCpbRemovalDelay;
  bpSei.bpDecodingUnitHrdParamsPresent = false;//m_pcEncCfg->m_noPicPartitionFlag == false;
  bpSei.decodingUnitCpbParamsInPicTimingSeiFlag = !m_pcEncCfg->m_decodingUnitInfoSEIEnabled;
  bpSei.initialCpbRemovalDelayLength = 16;                  // assuming 0.5 sec, log2( 90,000 * 0.5 ) = 16-bit
  // Note: The following parameters require some knowledge about the GOP structure.
  //       Using getIntraPeriod() should be avoided though, because it assumes certain GOP
  //       properties, which are only valid in CTC.
  //       Still copying this setting from HM for consistency, improvements welcome
  bool isRandomAccess  = m_pcEncCfg->m_IntraPeriod > 0;
  if( isRandomAccess )
  {
    bpSei.cpbRemovalDelayLength = 6;                        // 32 = 2^5 (plus 1)
    bpSei.dpbOutputDelayLength =  6;                        // 32 + 3 = 2^6
  }
  else
  {
    bpSei.cpbRemovalDelayLength = 9;                        // max. 2^10
    bpSei.dpbOutputDelayLength =  9;                        // max. 2^10
  }
  bpSei.duCpbRemovalDelayIncrementLength = 7;               // ceil( log2( tick_divisor_minus2 + 2 ) )
  bpSei.dpbOutputDelayDuLength = bpSei.dpbOutputDelayLength + bpSei.duCpbRemovalDelayIncrementLength;
  //for the concatenation, it can be set to one during splicing.
  bpSei.concatenationFlag = 0;
  //since the temporal layer HRDParameters is not ready, we assumed it is fixed
  bpSei.auCpbRemovalDelayDelta = 1;
  bool bpDeltasGOPStructure = m_pcEncCfg->m_GOPSize == 8 || m_pcEncCfg->m_GOPSize == 16; //assume GOPs specified as in CTC
  bpSei.cpbRemovalDelayDeltasPresent = bpDeltasGOPStructure;
  if (bpSei.cpbRemovalDelayDeltasPresent)
  {
    switch (m_pcEncCfg->m_GOPSize)
    {
      case 8:
      {
        if (noLeadingPictures)
        {
          bpSei.numCpbRemovalDelayDeltas         = 5;
          bpSei.cpbRemovalDelayDelta[0]          = 1;
          bpSei.cpbRemovalDelayDelta[1]          = 2;
          bpSei.cpbRemovalDelayDelta[2]          = 3;
          bpSei.cpbRemovalDelayDelta[3]          = 6;
          bpSei.cpbRemovalDelayDelta[4]          = 7;
        }
        else
        {
          bpSei.numCpbRemovalDelayDeltas         = 3;
          bpSei.cpbRemovalDelayDelta[0]          = 1;
          bpSei.cpbRemovalDelayDelta[1]          = 2;
          bpSei.cpbRemovalDelayDelta[2]          = 3;
        }
      }
        break;
      case 16:
      {
        if (noLeadingPictures)
        {
          bpSei.numCpbRemovalDelayDeltas         = 9;
          bpSei.cpbRemovalDelayDelta[0]          = 1;
          bpSei.cpbRemovalDelayDelta[1]          = 2;
          bpSei.cpbRemovalDelayDelta[2]          = 3;
          bpSei.cpbRemovalDelayDelta[3]          = 4;
          bpSei.cpbRemovalDelayDelta[4]          = 6;
          bpSei.cpbRemovalDelayDelta[5]          = 7;
          bpSei.cpbRemovalDelayDelta[6]          = 9;
          bpSei.cpbRemovalDelayDelta[7]          = 14;
          bpSei.cpbRemovalDelayDelta[8]          = 15;
        }
        else
        {
          bpSei.numCpbRemovalDelayDeltas         = 5;
          bpSei.cpbRemovalDelayDelta[0]          = 1;
          bpSei.cpbRemovalDelayDelta[1]          = 2;
          bpSei.cpbRemovalDelayDelta[2]          = 3;
          bpSei.cpbRemovalDelayDelta[3]          = 6;
          bpSei.cpbRemovalDelayDelta[4]          = 7;
        }
      }
        break;
      default:
      {
        THROW("m_cpbRemovalDelayDelta not applicable for the GOP size");
      }
      break;
    }
  }
  bpSei.sublayerDpbOutputOffsetsPresent = true;
  for(int i = 0; i < bpSei.bpMaxSubLayers; i++)
  {
    bpSei.dpbOutputTidOffset[i] = m_pcEncCfg->m_numReorderPics[i] * (1<<(bpSei.bpMaxSubLayers-1-i));
    if(bpSei.dpbOutputTidOffset[i] >= m_pcEncCfg->m_numReorderPics[bpSei.bpMaxSubLayers-1])
    {
      bpSei.dpbOutputTidOffset[i] -= m_pcEncCfg->m_numReorderPics[bpSei.bpMaxSubLayers-1];
    }
    else
    {
      bpSei.dpbOutputTidOffset[i] = 0;
    }
  }
  // A commercial encoder should track the buffer state for all layers and sub-layers
  // to ensure CPB conformance. Such tracking is required for calculating alternative
  // CPB parameters.
  // Unfortunately VTM does not have such tracking. Thus we cannot encode alternative
  // CPB parameters here.
  bpSei.altCpbParamsPresent = false;
  bpSei.useAltCpbParamsFlag = false;
}

//! calculate hashes for entire reconstructed picture
void SEIEncoder::initDecodedPictureHashSEI( SEIDecodedPictureHash& dphSei, const CPelUnitBuf& pic, std::string &rHashString, const BitDepths &bitDepths)
{
  CHECK(!(m_isInitialized), "Unspecified error");

  dphSei.method = m_pcEncCfg->m_decodedPictureHashSEIType;
  switch (m_pcEncCfg->m_decodedPictureHashSEIType)
  {
    case HASHTYPE_MD5:
      {
        uint32_t numChar=calcMD5(pic, dphSei.pictureHash, bitDepths);
        rHashString = hashToString(dphSei.pictureHash, numChar);
      }
      break;
    case HASHTYPE_CRC:
      {
        uint32_t numChar=calcCRC(pic, dphSei.pictureHash, bitDepths);
        rHashString = hashToString(dphSei.pictureHash, numChar);
      }
      break;
    case HASHTYPE_CHECKSUM:
    default:
      {
        uint32_t numChar=calcChecksum(pic, dphSei.pictureHash, bitDepths);
        rHashString = hashToString(dphSei.pictureHash, numChar);
      }
      break;
  }
}

void SEIEncoder::initPictureTimingSEI( SEIMessages& seiMessages, SEIMessages& nestedSeiMessages, SEIMessages& duInfoSeiMessages, const Slice *slice, const uint32_t numDU, const bool bpPresentInAU)
{
  // Picture timing depends on buffering period. When either of those is not disabled,
  // initialization would fail. Needs more cleanup after DU timing is integrated.
  if (!(m_pcEncCfg->m_pictureTimingSEIEnabled && m_pcEncCfg->m_bufferingPeriodSEIEnabled))
  {
    return;
  }

  const GeneralHrdParams *hrd = &slice->sps->generalHrdParams;

  // update decoding unit parameters
  if ((m_pcEncCfg->m_pictureTimingSEIEnabled || m_pcEncCfg->m_bufferingPeriodSEIEnabled) && slice->nuhLayerId == slice->vps->layerId[0])
  {
    int picSptDpbOutputDuDelay = 0;
    SEIPictureTiming *ptSei = new SEIPictureTiming();
    const uint32_t maxNumSubLayers = slice->sps->maxTLayers;

    // DU parameters
    if( hrd->generalDecodingUnitHrdParamsPresent )
    {
      ptSei->numDecodingUnitsMinus1      = numDU - 1;
      ptSei->duCommonCpbRemovalDelayFlag = false;
      ptSei->numNalusInDuMinus1.resize( numDU );
      ptSei->duCpbRemovalDelayMinus1.resize( numDU * maxNumSubLayers );
    }
    const uint32_t cpbRemovalDelayLegth = m_pcEncHRD->bufferingPeriodSEI.cpbRemovalDelayLength;
    ptSei->auCpbRemovalDelay[maxNumSubLayers-1] = std::min<int>(std::max<int>(1, m_totalCoded[maxNumSubLayers-1] - m_lastBPSEI[maxNumSubLayers-1]), (1<<cpbRemovalDelayLegth)); // Syntax element signalled as minus, hence the .
    CHECK( (m_totalCoded[maxNumSubLayers-1] - m_lastBPSEI[maxNumSubLayers-1]) > (1<<cpbRemovalDelayLegth), " cpbRemovalDelayLegth too small for m_auCpbRemovalDelay[pt_max_sub_layers_minus1] at picture timing SEI " );
    const uint32_t temporalId = slice->TLayer;
    for( int i = temporalId ; i < maxNumSubLayers - 1 ; i ++ )
    {
      int indexWithinGOP = (m_totalCoded[maxNumSubLayers - 1] - m_lastBPSEI[maxNumSubLayers - 1]) % m_pcEncCfg->m_GOPSize;
      ptSei->ptSubLayerDelaysPresent[i] = true;
      if( ((m_rapWithLeading == true) && (indexWithinGOP == 0)) || (m_totalCoded[maxNumSubLayers - 1] == 0) || bpPresentInAU || (slice->poc + m_pcEncCfg->m_GOPSize) > m_pcEncCfg->m_framesToBeEncoded )
      {
        ptSei->cpbRemovalDelayDeltaEnabledFlag[i] = false;
      }
      else
      {
        ptSei->cpbRemovalDelayDeltaEnabledFlag[i] = m_pcEncHRD->bufferingPeriodSEI.cpbRemovalDelayDeltasPresent;
      }
      if( ptSei->cpbRemovalDelayDeltaEnabledFlag[i] )
      {
        if( m_rapWithLeading == false )
        {
          switch (m_pcEncCfg->m_GOPSize)
          {
            case 8:
            {
              if((indexWithinGOP == 1 && i == 2))
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 0;
              }
              else if((indexWithinGOP == 2 && i == 2) || (indexWithinGOP == 6 && i == 2))
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 1;
              }
              else if((indexWithinGOP == 1 && i == 1) || (indexWithinGOP == 3 && i == 2))
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 2;
              }
              else if(indexWithinGOP == 2 && i == 1)
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 3;
              }
              else if(indexWithinGOP == 1 && i == 0)
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 4;
              }
              else
              {
                THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
              }
            }
              break;
            case 16:
            {
              if((indexWithinGOP == 1 && i == 3))
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 0;
              }
              else if((indexWithinGOP == 2 && i == 3) || (indexWithinGOP == 10 && i == 3) || (indexWithinGOP == 14 && i == 3))
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 1;
              }
              else if((indexWithinGOP == 1 && i == 2) || (indexWithinGOP == 3 && i == 3) || (indexWithinGOP == 7 && i == 3) || (indexWithinGOP == 11 && i == 3))
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 2;
              }
              else if(indexWithinGOP == 4 && i == 3)
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 3;
              }
              else if((indexWithinGOP == 2 && i == 2) || (indexWithinGOP == 10 && i == 2))
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 4;
              }
              else if(indexWithinGOP == 1 && i == 1)
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 5;
              }
              else if(indexWithinGOP == 3 && i == 2)
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 6;
              }
              else if(indexWithinGOP == 2 && i == 1)
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 7;
              }
              else if(indexWithinGOP == 1 && i == 0)
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 8;
              }
              else
              {
                THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
              }
            }
              break;
            default:
            {
              THROW("m_cpbRemovalDelayDeltaIdx not supported for the current GOP size");
            }
              break;
          }
        }
        else
        {
          switch (m_pcEncCfg->m_GOPSize)
          {
            case 8:
            {
              if((indexWithinGOP == 1 && i == 2) || (indexWithinGOP == 5 && i == 2))
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 0;
              }
              else if(indexWithinGOP == 2 && i == 2)
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 1;
              }
              else if(indexWithinGOP == 1 && i == 1)
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 2;
              }
              else
              {
                THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
              }
            }
              break;
            case 16:
            {
              if((indexWithinGOP == 1 && i == 3) || (indexWithinGOP == 9 && i == 3) || (indexWithinGOP == 13 && i == 3))
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 0;
              }
              else if((indexWithinGOP == 2 && i == 3) || (indexWithinGOP == 6 && i == 3) || (indexWithinGOP == 10 && i == 3))
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 1;
              }
              else if((indexWithinGOP == 1 && i == 2) || (indexWithinGOP == 9 && i == 2) || (indexWithinGOP == 3 && i == 3))
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 2;
              }
              else if(indexWithinGOP == 2 && i == 2)
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 3;
              }
              else if(indexWithinGOP == 1 && i == 1)
              {
                ptSei->cpbRemovalDelayDeltaIdx[i] = 4;
              }
              else
              {
                THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
              }
            }
              break;
            default:
            {
              THROW("m_cpbRemovalDelayDeltaIdx not applicable for the sub-layer and GOP size");
            }
              break;
          }
        }
      }
      else
      {
        int scaledDistToBuffPeriod = (m_totalCoded[i] - m_lastBPSEI[i]) * (1<<(maxNumSubLayers - 1 - i));
        ptSei->auCpbRemovalDelay[i] = std::min<int>(std::max<int>(1, scaledDistToBuffPeriod), (1<<cpbRemovalDelayLegth)); // Syntax element signalled as minus, hence the .
        CHECK( scaledDistToBuffPeriod > (1<<cpbRemovalDelayLegth), " cpbRemovalDelayLegth too small for m_auCpbRemovalDelay[i] at picture timing SEI " );
      }
    }
    ptSei->picDpbOutputDelay = slice->sps->numReorderPics[slice->sps->maxTLayers-1] + slice->poc - m_totalCoded[maxNumSubLayers-1];
//    if(m_pcEncCfg->m_efficientFieldIRAPEnabled && IRAPGOPid > 0 && IRAPGOPid < m_iGopSize)
//    {
//      // if pictures have been swapped there is likely one more picture delay on their tid. Very rough approximation
//      ptSei->picDpbOutputDelay ++;
//    }
    int factor = hrd->tickDivisorMinus2 + 2;
    ptSei->picDpbOutputDuDelay = factor * ptSei->picDpbOutputDelay;
    if( m_pcEncCfg->m_decodingUnitInfoSEIEnabled )
    {
      picSptDpbOutputDuDelay = factor * ptSei->picDpbOutputDelay;
    }
    if( bpPresentInAU )
    {
      for( int i = temporalId ; i < maxNumSubLayers ; i ++ )
      {
        m_lastBPSEI[i] = m_totalCoded[i];
      }
      if( (slice->nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL)||(slice->nalUnitType == NAL_UNIT_CODED_SLICE_CRA) )
      {
        m_rapWithLeading = true;
      }
    }


    if( m_pcEncCfg->m_pictureTimingSEIEnabled )
    {
      seiMessages.push_back( ptSei );

//      if (m_pcEncCfg->m_scalableNestingSEIEnabled && !m_pcEncCfg->m_samePicTimingInAllOLS)
//      {
//        SEIPictureTiming *pictureTimingSEIcopy = new SEIPictureTiming();
//        *pictureTimingSEIcopy = *pictureTimingSEI;
//        nestedSeiMessages.push_back(pictureTimingSEIcopy);
//      }
    }

    if( m_pcEncCfg->m_decodingUnitInfoSEIEnabled && hrd->generalDecodingUnitHrdParamsPresent )
    {
      for( int i = 0; i < ( ptSei->numDecodingUnitsMinus1 + 1 ); i ++ )
      {
        SEIDecodingUnitInfo *duInfoSEI = new SEIDecodingUnitInfo();
        duInfoSEI->decodingUnitIdx = i;
        for( int j = temporalId; j <= maxNumSubLayers; j++ )
        {
          duInfoSEI->duSptCpbRemovalDelayIncrement[j] = ptSei->duCpbRemovalDelayMinus1[i*maxNumSubLayers+j] + 1;
        }
        duInfoSEI->dpbOutputDuDelayPresent = false;
        duInfoSEI->picSptDpbOutputDuDelay = picSptDpbOutputDuDelay;

        duInfoSeiMessages.push_back(duInfoSEI);
      }
    }

    if( !m_pcEncCfg->m_pictureTimingSEIEnabled && ptSei )
    {
      delete ptSei;
    }
  }

  // not sure if this is the final place
  for( int i = slice->TLayer; i < slice->sps->maxTLayers; i ++ )
  {
    m_totalCoded[i]++;
  }
}


} // namespace vvenc

//! \}


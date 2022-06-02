/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


#include "SEIEncoder.h"

#include "vvenc/vvencCfg.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/SEI.h"
#include "CommonLib/PicYuvMD5.h"
#include "CommonLib/HRD.h"
#include "CommonLib/Slice.h"
#include "EncHRD.h"
#include "GOPCfg.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

void SEIEncoder::init( const VVEncCfg& encCfg, const GOPCfg* gopCfg, EncHRD& encHRD)
{
  m_pcEncCfg      = &encCfg;
  m_gopCfg        = gopCfg;
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
  bpSei.bpMaxSubLayers        = m_pcEncCfg->m_maxTLayer + 1;
  bpSei.bpCpbCnt              = 1;
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
  bool isRandomAccess  = m_pcEncCfg->m_picReordering;
  if( isRandomAccess && m_pcEncCfg->m_IntraPeriod < 256)
  {
    bpSei.cpbRemovalDelayLength =                                                           // 6  // 32 = 2^5 (plus 1)
    bpSei.dpbOutputDelayLength  =  ceilLog2( m_pcEncCfg->m_IntraPeriod)+1;                      // 6  // 32 + 3 = 2^6
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
  CHECK( m_pcEncCfg->m_IntraPeriod % m_pcEncCfg->m_GOPSize != 0, "broken for aip" );
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
  const std::vector<int>& numReorderPics = m_gopCfg->getNumReorderPics();
  for(int i = 0; i < bpSei.bpMaxSubLayers; i++)
  {
    bpSei.dpbOutputTidOffset[i] = numReorderPics[i] * (1<<(bpSei.bpMaxSubLayers-1-i));
    if(bpSei.dpbOutputTidOffset[i] >= numReorderPics[bpSei.bpMaxSubLayers-1])
    {
      bpSei.dpbOutputTidOffset[i] -= numReorderPics[bpSei.bpMaxSubLayers-1];
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

  dphSei.method         = m_pcEncCfg->m_decodedPictureHashSEIType;
  dphSei.singleCompFlag = m_pcEncCfg->m_internChromaFormat == 0;

  switch (m_pcEncCfg->m_decodedPictureHashSEIType)
  {
    case VVENC_HASHTYPE_MD5:
      {
        uint32_t numChar=calcMD5(pic, dphSei.pictureHash, bitDepths);
        rHashString = hashToString(dphSei.pictureHash, numChar);
      }
      break;
    case VVENC_HASHTYPE_CRC:
      {
        uint32_t numChar=calcCRC(pic, dphSei.pictureHash, bitDepths);
        rHashString = hashToString(dphSei.pictureHash, numChar);
      }
      break;
    case VVENC_HASHTYPE_CHECKSUM:
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
      CHECK( m_pcEncCfg->m_IntraPeriod % m_pcEncCfg->m_GOPSize != 0, "broken for aip" );
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
      if( (slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL)||(slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_CRA) )
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
  for( uint32_t i = slice->TLayer; i < slice->sps->maxTLayers; i ++ )
  {
    m_totalCoded[i]++;
  }
}

void SEIEncoder::initSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics *seiAltTransCharacteristics)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiAltTransCharacteristics!=NULL), "Unspecified error");
  //  Set SEI message parameters read from command line options
  seiAltTransCharacteristics->preferredTransferCharacteristics = m_pcEncCfg->m_preferredTransferCharacteristics;
}

void SEIEncoder::initSEIMasteringDisplayColourVolume(SEIMasteringDisplayColourVolume *seiMDCV)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiMDCV != NULL), "Unspecified error");

  //  Set SEI message parameters read from command line options
  seiMDCV->values.primaries[0][0] = m_pcEncCfg->m_masteringDisplay[0];
  seiMDCV->values.primaries[0][1] = m_pcEncCfg->m_masteringDisplay[1];

  seiMDCV->values.primaries[1][0] = m_pcEncCfg->m_masteringDisplay[2];
  seiMDCV->values.primaries[1][1] = m_pcEncCfg->m_masteringDisplay[3];

  seiMDCV->values.primaries[2][0] = m_pcEncCfg->m_masteringDisplay[4];
  seiMDCV->values.primaries[2][1] = m_pcEncCfg->m_masteringDisplay[5];

  seiMDCV->values.whitePoint[0]   = m_pcEncCfg->m_masteringDisplay[6];
  seiMDCV->values.whitePoint[1]   = m_pcEncCfg->m_masteringDisplay[7];

  seiMDCV->values.maxLuminance    = m_pcEncCfg->m_masteringDisplay[8];
  seiMDCV->values.minLuminance    = m_pcEncCfg->m_masteringDisplay[9];
}

void SEIEncoder::initSEIContentLightLevel(SEIContentLightLevelInfo *seiCLL)
{
  CHECK(!(m_isInitialized), "Unspecified error");
  CHECK(!(seiCLL != NULL), "Unspecified error");

  //  Set SEI message parameters read from command line options
  seiCLL->maxContentLightLevel    = m_pcEncCfg->m_contentLightLevel[0];
  seiCLL->maxPicAverageLightLevel = m_pcEncCfg->m_contentLightLevel[1];
}


} // namespace vvenc

//! \}


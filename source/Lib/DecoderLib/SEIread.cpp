/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/**
 \file     SEIread.cpp
 \brief    reading funtionality for SEI messages
 */

#include "CommonLib/CommonDef.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/SEI.h"
#include "CommonLib/Slice.h"
#include "VLCReader.h"
#include "SEIread.h"
#include "CommonLib/Picture.h"
#include "CommonLib/dtrace_next.h"
#include "Utilities/MsgLog.h"
#include <iomanip>


//! \ingroup DecoderLib
//! \{

namespace vvenc {
void SEIReader::sei_read_scode(std::ostream *pOS, uint32_t length, int& code, const char *pSymbolName)
{
  READ_SCODE(length, code, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << code << "\n";
  }
}

void SEIReader::sei_read_code(std::ostream *pOS, uint32_t uiLength, uint32_t& ruiCode, const char *pSymbolName)
{
  READ_CODE(uiLength, ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

void SEIReader::sei_read_uvlc(std::ostream *pOS, uint32_t& ruiCode, const char *pSymbolName)
{
  READ_UVLC(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

void SEIReader::sei_read_svlc(std::ostream *pOS, int& ruiCode, const char *pSymbolName)
{
  READ_SVLC(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << ruiCode << "\n";
  }
}

void SEIReader::sei_read_flag(std::ostream *pOS, uint32_t& ruiCode, const char *pSymbolName)
{
  READ_FLAG(ruiCode, pSymbolName);
  if (pOS)
  {
    (*pOS) << "  " << std::setw(55) << pSymbolName << ": " << (ruiCode?1:0) << "\n";
  }
}

static inline void output_sei_message_header(SEI &sei, std::ostream *pDecodedMessageOutputStream, uint32_t payloadSize)
{
  if (pDecodedMessageOutputStream)
  {
    std::string seiMessageHdr(SEI::getSEIMessageString(sei.payloadType())); seiMessageHdr+=" SEI message";
    (*pDecodedMessageOutputStream) << std::setfill('-') << std::setw((int)seiMessageHdr.size()) << "-" << std::setfill(' ') << "\n" << seiMessageHdr << " (" << payloadSize << " bytes)"<< "\n";
  }
}

#undef READ_CODE
#undef READ_SCODE
#undef READ_SVLC
#undef READ_UVLC
#undef READ_FLAG


/**
 * unmarshal a single SEI message from bitstream bs
 */
 // note: for independent parsing no parameter set should not be required here
void SEIReader::parseSEImessage(InputBitstream* bs, SEIMessages& seis, const vvencNalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId, const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream)
{
  SEIMessages   seiListInCurNalu;
  setBitstream(bs);
  CHECK(m_pcBitstream->getNumBitsUntilByteAligned(), "Bitstream not aligned");

  do
  {
    xReadSEImessage(seis, nalUnitType, nuh_layer_id, temporalId, vps, sps, hrd, pDecodedMessageOutputStream);
    seiListInCurNalu.push_back(seis.back());
    /* SEI messages are an integer number of bytes, something has failed
    * in the parsing if bitstream not byte-aligned */
    CHECK(m_pcBitstream->getNumBitsUntilByteAligned(), "Bitstream not aligned");
  }
  while (m_pcBitstream->getNumBitsLeft() > 8);

  SEIMessages fillerData = getSeisByType(seiListInCurNalu, SEI::FILLER_PAYLOAD);
  CHECK(fillerData.size() > 0 && fillerData.size() != seiListInCurNalu.size(), "When an SEI NAL unit contains an SEI message with payloadType equal to filler payload, the SEI NAL unit shall not contain any other SEI message with payloadType not equal to filler payload");

  xReadRbspTrailingBits();
}

void SEIReader::xReadSEImessage(SEIMessages& seis, const vvencNalUnitType nalUnitType, const uint32_t nuh_layer_id, const uint32_t temporalId, const VPS *vps, const SPS *sps, HRD &hrd, std::ostream *pDecodedMessageOutputStream)
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== SEI message ===========\n" );

  int payloadType = 0;
  uint32_t val = 0;

  do
  {
    sei_read_code(NULL, 8, val, "payload_type");
    payloadType += val;
  } while (val==0xFF);

  uint32_t payloadSize = 0;
  do
  {
    sei_read_code(NULL, 8, val, "payload_size");
    payloadSize += val;
  } while (val==0xFF);

  DTRACE( g_trace_ctx, D_HEADER, "=========== %s SEI message ===========\n", SEI::getSEIMessageString( (SEI::PayloadType)payloadType ) );

  /* extract the payload for this single SEI message.
   * This allows greater safety in erroneous parsing of an SEI message
   * from affecting subsequent messages.
   * After parsing the payload, bs needs to be restored as the primary
   * bitstream.
   */
  InputBitstream *bs = getBitstream();
  setBitstream(bs->extractSubstream(payloadSize * 8));

  SEI *sei = NULL;
  const SEIBufferingPeriod *bp = NULL;
  const SEIPictureTiming *pt = NULL;

  if(nalUnitType == VVENC_NAL_UNIT_PREFIX_SEI)
  {
    switch (payloadType)
    {
    case SEI::USER_DATA_UNREGISTERED:
      sei = new SEIuserDataUnregistered;
      xParseSEIuserDataUnregistered((SEIuserDataUnregistered&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DECODING_UNIT_INFO:
      bp = &hrd.bufferingPeriodSEI;
      if (!bp)
      {
        msg.log( VVENC_WARNING, "Warning: Found Decoding unit information SEI message, but no active buffering period is available. Ignoring.");
      }
      else
      {
        sei = new SEIDecodingUnitInfo;
        xParseSEIDecodingUnitInfo((SEIDecodingUnitInfo&) *sei, payloadSize, *bp, temporalId, pDecodedMessageOutputStream);
      }
      break;
    case SEI::BUFFERING_PERIOD:
        sei = new SEIBufferingPeriod;
      xParseSEIBufferingPeriod((SEIBufferingPeriod&) *sei, payloadSize, pDecodedMessageOutputStream);
      hrd.bufferingPeriodSEI = *((SEIBufferingPeriod*) sei);
      break;
    case SEI::PICTURE_TIMING:
      {
        bp = &hrd.bufferingPeriodSEI;
        if (!bp)
        {
          msg.log( VVENC_WARNING, "Warning: Found Picture timing SEI message, but no active buffering period is available. Ignoring.");
        }
        else
        {
          sei = new SEIPictureTiming;
          xParseSEIPictureTiming((SEIPictureTiming&)*sei, payloadSize, temporalId, *bp, pDecodedMessageOutputStream);
          hrd.pictureTimingSEI = *( (SEIPictureTiming*) sei );
        }
      }
      break;
    case SEI::SCALABLE_NESTING:
      sei = new SEIScalableNesting;
      xParseSEIScalableNesting((SEIScalableNesting&)*sei, nalUnitType, nuh_layer_id, payloadSize, vps, sps, pDecodedMessageOutputStream);
      break;
    case SEI::FRAME_FIELD_INFO:
      sei = new SEIFrameFieldInfo;
      pt = &hrd.pictureTimingSEI;
      xParseSEIFrameFieldinfo((SEIFrameFieldInfo&) *sei, *pt, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::DEPENDENT_RAP_INDICATION:
      sei = new SEIDependentRAPIndication;
      xParseSEIDependentRAPIndication((SEIDependentRAPIndication&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::FRAME_PACKING:
      sei = new SEIFramePacking;
      xParseSEIFramePacking((SEIFramePacking&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::PARAMETER_SETS_INCLUSION_INDICATION:
      sei = new SEIParameterSetsInclusionIndication;
      xParseSEIParameterSetsInclusionIndication((SEIParameterSetsInclusionIndication&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::MASTERING_DISPLAY_COLOUR_VOLUME:
      sei = new SEIMasteringDisplayColourVolume;
      xParseSEIMasteringDisplayColourVolume((SEIMasteringDisplayColourVolume&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS:
      sei = new SEIAlternativeTransferCharacteristics;
      xParseSEIAlternativeTransferCharacteristics((SEIAlternativeTransferCharacteristics&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::EQUIRECTANGULAR_PROJECTION:
      sei = new SEIEquirectangularProjection;
      xParseSEIEquirectangularProjection((SEIEquirectangularProjection&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SPHERE_ROTATION:
      sei = new SEISphereRotation;
      xParseSEISphereRotation((SEISphereRotation&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::OMNI_VIEWPORT:
      sei = new SEIOmniViewport;
      xParseSEIOmniViewport((SEIOmniViewport&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::REGION_WISE_PACKING:
      sei = new SEIRegionWisePacking;
      xParseSEIRegionWisePacking((SEIRegionWisePacking&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::GENERALIZED_CUBEMAP_PROJECTION:
      sei = new SEIGeneralizedCubemapProjection;
      xParseSEIGeneralizedCubemapProjection((SEIGeneralizedCubemapProjection&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SUBPICTURE_LEVEL_INFO:
      sei = new SEISubpicureLevelInfo;
      xParseSEISubpictureLevelInfo((SEISubpicureLevelInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::SAMPLE_ASPECT_RATIO_INFO:
      sei = new SEISampleAspectRatioInfo;
      xParseSEISampleAspectRatioInfo((SEISampleAspectRatioInfo&) *sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::USER_DATA_REGISTERED_ITU_T_T35:
      sei = new SEIUserDataRegistered;
      xParseSEIUserDataRegistered((SEIUserDataRegistered&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::FILM_GRAIN_CHARACTERISTICS:
      sei = new SEIFilmGrainCharacteristics;
      xParseSEIFilmGrainCharacteristics((SEIFilmGrainCharacteristics&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::CONTENT_LIGHT_LEVEL_INFO:
      sei = new SEIContentLightLevelInfo;
      xParseSEIContentLightLevelInfo((SEIContentLightLevelInfo&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::AMBIENT_VIEWING_ENVIRONMENT:
      sei = new SEIAmbientViewingEnvironment;
      xParseSEIAmbientViewingEnvironment((SEIAmbientViewingEnvironment&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    case SEI::CONTENT_COLOUR_VOLUME:
      sei = new SEIContentColourVolume;
      xParseSEIContentColourVolume((SEIContentColourVolume&)*sei, payloadSize, pDecodedMessageOutputStream);
      break;
    default:
      for (uint32_t i = 0; i < payloadSize; i++)
      {
        uint32_t seiByte;
        sei_read_code (NULL, 8, seiByte, "unknown prefix SEI payload byte");
      }
      msg.log( VVENC_WARNING, "Unknown prefix SEI message (payloadType = %d) was found!\n", payloadType);
      if (pDecodedMessageOutputStream)
      {
        (*pDecodedMessageOutputStream) << "Unknown prefix SEI message (payloadType = " << payloadType << ") was found!\n";
      }
      break;
    }
  }
  else
  {
    switch (payloadType)
    {
      case SEI::USER_DATA_UNREGISTERED:
        sei = new SEIuserDataUnregistered;
        xParseSEIuserDataUnregistered((SEIuserDataUnregistered&) *sei, payloadSize, pDecodedMessageOutputStream);
        break;
      case SEI::DECODED_PICTURE_HASH:
        sei = new SEIDecodedPictureHash;
        xParseSEIDecodedPictureHash((SEIDecodedPictureHash&) *sei, payloadSize, pDecodedMessageOutputStream);
        break;
      case SEI::SCALABLE_NESTING:
        sei = new SEIScalableNesting;
        xParseSEIScalableNesting((SEIScalableNesting&)*sei, nalUnitType, nuh_layer_id, payloadSize, vps, sps, pDecodedMessageOutputStream);
        break;
      default:
        for (uint32_t i = 0; i < payloadSize; i++)
        {
          uint32_t seiByte;
          sei_read_code( NULL, 8, seiByte, "unknown suffix SEI payload byte");
        }
        msg.log( VVENC_WARNING, "Unknown suffix SEI message (payloadType = %d) was found!\n", payloadType);
        if (pDecodedMessageOutputStream)
        {
          (*pDecodedMessageOutputStream) << "Unknown suffix SEI message (payloadType = " << payloadType << ") was found!\n";
        }
        break;
    }
  }

  if (sei != NULL)
  {
    seis.push_back(sei);
  }

  /* By definition the underlying bitstream terminates in a byte-aligned manner.
   * 1. Extract all bar the last MIN(bitsremaining,nine) bits as reserved_payload_extension_data
   * 2. Examine the final 8 bits to determine the payload_bit_equal_to_one marker
   * 3. Extract the remainingreserved_payload_extension_data bits.
   *
   * If there are fewer than 9 bits available, extract them.
   */
  int payloadBitsRemaining = getBitstream()->getNumBitsLeft();
  if (payloadBitsRemaining) /* more_data_in_payload() */
  {
    for (; payloadBitsRemaining > 9; payloadBitsRemaining--)
    {
      uint32_t reservedPayloadExtensionData;
      sei_read_code ( pDecodedMessageOutputStream, 1, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    /* 2 */
    int finalBits = getBitstream()->peekBits(payloadBitsRemaining);
    int finalPayloadBits = 0;
    for (int mask = 0xff; finalBits & (mask >> finalPayloadBits); finalPayloadBits++)
    {
      continue;
    }

    /* 3 */
    for (; payloadBitsRemaining > 9 - finalPayloadBits; payloadBitsRemaining--)
    {
      uint32_t reservedPayloadExtensionData;
      sei_read_flag ( 0, reservedPayloadExtensionData, "reserved_payload_extension_data");
    }

    uint32_t dummy;
    sei_read_flag( 0, dummy, "payload_bit_equal_to_one"); payloadBitsRemaining--;
    while (payloadBitsRemaining)
    {
      sei_read_flag( 0, dummy, "payload_bit_equal_to_zero"); payloadBitsRemaining--;
    }
  }

  /* restore primary bitstream for sei_message */
  delete getBitstream();
  setBitstream(bs);
}

/**
 * parse bitstream bs and unpack a user_data_unregistered SEI message
 * of payloasSize bytes into sei.
 */

void SEIReader::xParseSEIuserDataUnregistered(SEIuserDataUnregistered &sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  CHECK(payloadSize < ISO_IEC_11578_LEN, "Payload too small");
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  for (uint32_t i = 0; i < ISO_IEC_11578_LEN; i++)
  {
    sei_read_code( pDecodedMessageOutputStream, 8, val, "uuid_iso_iec_11578");
    sei.uuid_iso_iec_11578[i] = val;
  }

  sei.userDataLength = payloadSize - ISO_IEC_11578_LEN;
  if (!sei.userDataLength)
  {
    sei.userData = 0;
    return;
  }

  sei.userData = new uint8_t[sei.userDataLength];
  for (uint32_t i = 0; i < sei.userDataLength; i++)
  {
    sei_read_code( NULL, 8, val, "user_data_payload_byte" );
    sei.userData[i] = val;
  }
  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  User data payload size: " << sei.userDataLength << "\n";
  }
}

/**
 * parse bitstream bs and unpack a decoded picture hash SEI message
 * of payloadSize bytes into sei.
 */
void SEIReader::xParseSEIDecodedPictureHash(SEIDecodedPictureHash& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t bytesRead = 0;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  uint32_t val;
  sei_read_code( pDecodedMessageOutputStream, 8, val, "dpb_sei_hash_type");
  sei.method = static_cast<vvencHashType>(val); bytesRead++;
  sei_read_code( pDecodedMessageOutputStream, 1, val, "dph_sei_single_component_flag");
  sei.singleCompFlag = val;
  sei_read_code( pDecodedMessageOutputStream, 7, val, "dph_sei_reserved_zero_7bits");
  bytesRead++;
  uint32_t expectedSize = ( sei.singleCompFlag ? 1 : 3 ) * (sei.method == 0 ? 16 : (sei.method == 1 ? 2 : 4));
  CHECK ((payloadSize - bytesRead) != expectedSize, "The size of the decoded picture hash does not match the expected size.");

  const char *traceString="\0";
  switch (sei.method)
  {
    case VVENC_HASHTYPE_MD5: traceString="picture_md5"; break;
    case VVENC_HASHTYPE_CRC: traceString="picture_crc"; break;
    case VVENC_HASHTYPE_CHECKSUM: traceString="picture_checksum"; break;
    default: THROW("Unknown hash type"); break;
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  " << std::setw(55) << traceString << ": " << std::hex << std::setfill('0');
  }

  sei.pictureHash.hash.clear();
  for(;bytesRead < payloadSize; bytesRead++)
  {
    sei_read_code( NULL, 8, val, traceString);
    sei.pictureHash.hash.push_back((uint8_t)val);
    if (pDecodedMessageOutputStream)
    {
      (*pDecodedMessageOutputStream) << std::setw(2) << val;
    }
  }

  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << std::dec << std::setfill(' ') << "\n";
  }
}

void SEIReader::xParseSEIScalableNesting(SEIScalableNesting& sei, const vvencNalUnitType nalUnitType, const uint32_t nuhLayerId, uint32_t payloadSize, const VPS *vps, const SPS *sps, std::ostream *decodedMessageOutputStream)
{
  uint32_t symbol;
  SEIMessages seis;
  output_sei_message_header(sei, decodedMessageOutputStream, payloadSize);

  sei_read_flag(decodedMessageOutputStream, symbol, "sn_ols_flag"); sei.snOlsFlag = symbol;
  sei_read_flag(decodedMessageOutputStream, symbol, "sn_subpic_flag"); sei.snSubpicFlag = symbol;
  if (sei.snOlsFlag)
  {
    sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_num_olss_minus1"); sei.snNumOlssMinus1 = symbol;
    for (uint32_t i = 0; i <= sei.snNumOlssMinus1; i++)
    {
      sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_ols_idx_delta_minus1[i]"); sei.snOlsIdxDeltaMinus1[i] = symbol;
    }
    for (uint32_t i = 0; i <= sei.snNumOlssMinus1; i++)
    {
      if (i == 0)
      {
        sei.snOlsIdx[i] = sei.snOlsIdxDeltaMinus1[i];
      }
      else
      {
        sei.snOlsIdx[i] = sei.snOlsIdxDeltaMinus1[i] + sei.snOlsIdxDeltaMinus1[i - 1] + 1;
      }
    }
    if (vps && vps->vpsId != 0)
    {
      uint32_t lowestLayerId = MAX_UINT;
      for (uint32_t olsIdxForSEI = 0; olsIdxForSEI <= sei.snNumOlssMinus1; olsIdxForSEI++)
      {
        int olsIdx = sei.snOlsIdx[olsIdxForSEI];
        for (int layerIdx = 0; layerIdx < vps->numLayersInOls[olsIdx]; layerIdx++)
        {
          if (lowestLayerId > vps->layerIdInOls[olsIdx][layerIdx])
          {
            lowestLayerId = vps->layerIdInOls[olsIdx][layerIdx];
          }
        }
      }
      CHECK(lowestLayerId!= nuhLayerId, "nuh_layer_id is not equal to the lowest layer among Olss that the scalable SEI applies");
    }
  }
  else
  {
    sei_read_flag(decodedMessageOutputStream, symbol, "sn_all_layers_flag"); sei.snAllLayersFlag = symbol;
    if (!sei.snAllLayersFlag)
    {
      sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_num_layers_minus1"); sei.snNumLayersMinus1 = symbol;
      sei.snLayerId[0] = nuhLayerId;
      for (uint32_t i = 1; i <= sei.snNumLayersMinus1; i++)
      {
        sei_read_code(decodedMessageOutputStream, 6, symbol, "sn_layer_id[i]"); sei.snLayerId[i] = symbol;
      }
    }
  }
  if (sei.snSubpicFlag)
  {
    sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_num_subpics_minus1"); sei.snNumSubpics = symbol + 1;
    sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_subpic_id_len_minus1"); sei.snSubpicIdLen = symbol + 1;
    sei.snSubpicId.resize(sei.snNumSubpics);
    for (uint32_t i = 0; i < sei.snNumSubpics; i++)
    {
      sei_read_code(decodedMessageOutputStream, sei.snSubpicIdLen, symbol, "sn_subpic_id[i]"); sei.snSubpicId[i] = symbol;
    }
  }

  sei_read_uvlc(decodedMessageOutputStream, symbol, "sn_num_seis_minus1"); sei.snNumSEIs = symbol + 1;
  CHECK (sei.snNumSEIs > 64, "The value of sn_num_seis_minus1 shall be in the range of 0 to 63");

  // byte alignment
  while (m_pcBitstream->getNumBitsRead() % 8 != 0)
  {
    sei_read_flag(decodedMessageOutputStream, symbol, "sn_zero_bit");
  }

  // read nested SEI messages
  for (int32_t i=0; i<sei.snNumSEIs; i++)
  {
    SEIMessages tmpSEIs;
    xReadSEImessage(tmpSEIs, nalUnitType, nuhLayerId, 0, vps, sps, m_nestedHrd, decodedMessageOutputStream);
    if (tmpSEIs.front()->payloadType() == SEI::BUFFERING_PERIOD)
    {
      SEIBufferingPeriod *bp = (SEIBufferingPeriod*) tmpSEIs.front();
      m_nestedHrd.bufferingPeriodSEI = *bp;
    }
    sei.nestedSEIs.push_back(tmpSEIs.front());
    tmpSEIs.clear();
  }

  xCheckScalableNestingConstraints(sei, nalUnitType, vps);

  if (decodedMessageOutputStream)
  {
    (*decodedMessageOutputStream) << "End of scalable nesting SEI message\n";
  }
}

void SEIReader::xCheckScalableNestingConstraints(const SEIScalableNesting& sei, const vvencNalUnitType nalUnitType, const VPS* vps)
{
  const std::vector<int> vclAssociatedSeiList { 3, 19, 45, 129, 137, 144, 145, 147, 148, 149, 150, 153, 154, 155, 156, 168, 204 };

  bool containBPorPTorDUIorSLI = false;
  bool containNoBPorPTorDUIorSLI = false;

  for (auto nestedsei : sei.nestedSEIs)
  {
    CHECK(nestedsei->payloadType() == SEI::FILLER_PAYLOAD || nestedsei->payloadType() == SEI::SCALABLE_NESTING, "An SEI message that has payloadType equal to filler payload or scalable nesting shall not be contained in a scalable nesting SEI message");

    CHECK(nestedsei->payloadType() != SEI::FILLER_PAYLOAD && nestedsei->payloadType() != SEI::DECODED_PICTURE_HASH && nalUnitType != VVENC_NAL_UNIT_PREFIX_SEI, "When a scalable nesting SEI message contains an SEI message that has payloadType not equal to filler payload or decoded picture hash, the SEI NAL unit containing the scalable nesting SEI message shall have nal_unit_type equal to PREFIX_SEI_NUT");

    CHECK(nestedsei->payloadType() == SEI::DECODED_PICTURE_HASH && nalUnitType != VVENC_NAL_UNIT_SUFFIX_SEI, "When a scalable nesting SEI message contains an SEI message that has payloadType equal to decoded picture hash, the SEI NAL unit containing the scalable nesting SEI message shall have nal_unit_type equal to SUFFIX_SEI_NUT");

    CHECK(nestedsei->payloadType() == SEI::DECODED_PICTURE_HASH && !sei.snSubpicFlag, "When the scalable nesting SEI message contains an SEI message that has payloadType equal to decoded picture hash, the value of sn_subpic_flag shall be equal to 1");

    CHECK(nestedsei->payloadType() == SEI::SUBPICTURE_LEVEL_INFO && sei.snSubpicFlag, "When the scalable nesting SEI message contains an SEI message that has payloadType equal to SLI, the value of sn_subpic_flag shall be equal to 0");

    CHECK(vps->generalHrdParams.generalSamePicTimingInAllOlsFlag && nestedsei->payloadType() == SEI::PICTURE_TIMING, "When general_same_pic_timing_in_all_ols_flag is equal to 1, there shall be no SEI NAL unit that contain a scalable-nested SEI message with payloadType equal to PT");

    for (int i = 0; i < vclAssociatedSeiList.size(); i++)
    {
      CHECK(nestedsei->payloadType() == vclAssociatedSeiList[i] && sei.snOlsFlag, "When the scalable nesting SEI message contains an SEI message that has payloadType equal to a value in vclAssociatedSeiList, the value of sn_ols_flag shall be equal to 0");
    }

    if (nestedsei->payloadType() == SEI::BUFFERING_PERIOD || nestedsei->payloadType() == SEI::PICTURE_TIMING || nestedsei->payloadType() == SEI::DECODING_UNIT_INFO || nestedsei->payloadType() == SEI::SUBPICTURE_LEVEL_INFO)
    {
      containBPorPTorDUIorSLI = true;
      CHECK(!sei.snOlsFlag, "When the scalable nesting SEI message contains an SEI message that has payloadType equal to BP, PT, or DUI, or SLI, the value of sn_ols_flag shall be equal to 1");
    }
    if (!(nestedsei->payloadType() == SEI::BUFFERING_PERIOD || nestedsei->payloadType() == SEI::PICTURE_TIMING || nestedsei->payloadType() == SEI::DECODING_UNIT_INFO || nestedsei->payloadType() == SEI::SUBPICTURE_LEVEL_INFO))
    {
      containNoBPorPTorDUIorSLI = true;
    }
  }
  CHECK(containBPorPTorDUIorSLI && containNoBPorPTorDUIorSLI, "When a scalable nesting SEI message contains a BP, PT, DUI, or SLI SEI message, the scalable nesting SEI message shall not contain any other SEI message with payloadType not equal to BP, PT, DUI, or SLI");
}

void SEIReader::xParseSEIDecodingUnitInfo(SEIDecodingUnitInfo& sei, uint32_t payloadSize, const SEIBufferingPeriod& bp, const uint32_t temporalId, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_uvlc( pDecodedMessageOutputStream, val, "decoding_unit_idx");
  sei.decodingUnitIdx = val;

  if(!bp.decodingUnitCpbParamsInPicTimingSeiFlag)
  {
    for (int i = temporalId; i <= bp.bpMaxSubLayers - 1; i++)
    {
      if (i < (bp.bpMaxSubLayers - 1))
      {
        sei_read_flag( pDecodedMessageOutputStream, val, "dui_sub_layer_delays_present_flag[i]" );
        sei.duiSubLayerDelaysPresent[i] = val;
      }
      else
      {
        sei.duiSubLayerDelaysPresent[i] = 1;
      }
      if( sei.duiSubLayerDelaysPresent[i] )
      {
        sei_read_code( pDecodedMessageOutputStream, bp.duCpbRemovalDelayIncrementLength, val, "du_spt_cpb_removal_delay_increment[i]");
        sei.duSptCpbRemovalDelayIncrement[i] = val;
      }
      else
      {
        sei.duSptCpbRemovalDelayIncrement[i] = 0;
      }
    }
  }
  else
  {
    for( int i = temporalId; i < bp.bpMaxSubLayers - 1; i ++ )
    {
      sei.duSptCpbRemovalDelayIncrement[i] = 0;
    }
  }
  if (bp.decodingUnitDpbDuParamsInPicTimingSeiFlag)
  {
    sei_read_flag(pDecodedMessageOutputStream, val, "dpb_output_du_delay_present_flag"); sei.dpbOutputDuDelayPresent = (val != 0);
  }
  else
  {
    sei.dpbOutputDuDelayPresent = false;
  }
  if(sei.dpbOutputDuDelayPresent)
  {
    sei_read_code( pDecodedMessageOutputStream, bp.dpbOutputDelayDuLength, val, "pic_spt_dpb_output_du_delay");
    sei.picSptDpbOutputDuDelay = val;
  }
}

void SEIReader::xParseSEIBufferingPeriod(SEIBufferingPeriod& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  int i, nalOrVcl;
  uint32_t code;


  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, code, "bp_nal_hrd_parameters_present_flag" );               sei.bpNalCpbParamsPresent = code;
  sei_read_flag( pDecodedMessageOutputStream, code, "bp_vcl_hrd_parameters_present_flag" );               sei.bpVclCpbParamsPresent = code;

  sei_read_code( pDecodedMessageOutputStream, 5, code, "initial_cpb_removal_delay_length_minus1" );     sei.initialCpbRemovalDelayLength = code + 1;
  sei_read_code( pDecodedMessageOutputStream, 5, code, "cpb_removal_delay_length_minus1" );             sei.cpbRemovalDelayLength        = code + 1;
  sei_read_code( pDecodedMessageOutputStream, 5, code, "dpb_output_delay_length_minus1" );              sei.dpbOutputDelayLength         = code + 1;
  sei_read_flag( pDecodedMessageOutputStream, code, "bp_decoding_unit_hrd_params_present_flag" );       sei.bpDecodingUnitHrdParamsPresent = code;
  if( sei.bpDecodingUnitHrdParamsPresent )
  {
    sei_read_code( pDecodedMessageOutputStream, 5, code, "du_cpb_removal_delay_increment_length_minus1" );  sei.duCpbRemovalDelayIncrementLength = code + 1;
    sei_read_code( pDecodedMessageOutputStream, 5, code, "dpb_output_delay_du_length_minus1" );             sei.dpbOutputDelayDuLength = code + 1;
    sei_read_flag( pDecodedMessageOutputStream, code, "decoding_unit_cpb_params_in_pic_timing_sei_flag" );  sei.decodingUnitCpbParamsInPicTimingSeiFlag = code;
    sei_read_flag(pDecodedMessageOutputStream, code, "decoding_unit_dpb_du_params_in_pic_timing_sei_flag");  sei.decodingUnitDpbDuParamsInPicTimingSeiFlag = code;
  }
  else
  {
    sei.duCpbRemovalDelayIncrementLength = 24;
    sei.dpbOutputDelayDuLength = 24;
    sei.decodingUnitDpbDuParamsInPicTimingSeiFlag = false;
  }

  CHECK(sei.altCpbParamsPresent && sei.bpDecodingUnitHrdParamsPresent,"When bp_alt_cpb_params_present_flag is equal to 1, the value of bp_du_hrd_params_present_flag shall be equal to 0");

  sei_read_flag( pDecodedMessageOutputStream, code, "concatenation_flag");
  sei.concatenationFlag = code;
  sei_read_flag ( pDecodedMessageOutputStream, code, "additional_concatenation_info_present_flag");
  sei.additionalConcatenationInfoPresent = code;
  if (sei.additionalConcatenationInfoPresent)
  {
    sei_read_code( pDecodedMessageOutputStream, sei.initialCpbRemovalDelayLength, code, "max_initial_removal_delay_for_concatenation" );
    sei.maxInitialRemovalDelayForConcatenation = code;
  }

  sei_read_code( pDecodedMessageOutputStream, ( sei.cpbRemovalDelayLength ), code, "au_cpb_removal_delay_delta_minus1" );
  sei.auCpbRemovalDelayDelta = code + 1;
  sei_read_code(pDecodedMessageOutputStream, 3, code, "bp_max_sub_layers_minus1");
  sei.bpMaxSubLayers = code + 1;
  if (sei.bpMaxSubLayers - 1 > 0)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "cpb_removal_delay_deltas_present_flag");
    sei.cpbRemovalDelayDeltasPresent = code;
  }
  else
  {
    sei.cpbRemovalDelayDeltasPresent = false;
  }
  if (sei.cpbRemovalDelayDeltasPresent)
  {
    sei_read_uvlc( pDecodedMessageOutputStream, code, "num_cpb_removal_delay_deltas_minus1" );
    sei.numCpbRemovalDelayDeltas = code + 1;
    for( i = 0; i < sei.numCpbRemovalDelayDeltas; i ++ )
    {
      sei_read_code( pDecodedMessageOutputStream, ( sei.cpbRemovalDelayLength ), code, "cpb_removal_delay_delta[i]" );
      sei.cpbRemovalDelayDelta[ i ] = code;
    }
  }

  sei_read_uvlc( pDecodedMessageOutputStream, code, "bp_cpb_cnt_minus1"); 
  sei.bpCpbCnt = code + 1;

  if (sei.bpMaxSubLayers - 1 > 0)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "bp_sublayer_initial_cpb_removal_delay_present_flag");
    sei.sublayerInitialCpbRemovalDelayPresent = code;
  }
  else
  {
    sei.sublayerInitialCpbRemovalDelayPresent = false;
  }
  for (i = (sei.sublayerInitialCpbRemovalDelayPresent ? 0 : sei.bpMaxSubLayers - 1); i < sei.bpMaxSubLayers; i++)
  {
    for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( sei.bpNalCpbParamsPresent ) ) ||
         ( ( nalOrVcl == 1 ) && ( sei.bpVclCpbParamsPresent ) ) )
      {
        for( int j = 0; j < ( sei.bpCpbCnt ); j ++ )
        {
          sei_read_code( pDecodedMessageOutputStream, sei.initialCpbRemovalDelayLength, code, nalOrVcl ? "vcl_initial_cpb_removal_delay[i][j]" : "nal_initial_cpb_removal_delay[i][j]" );
          sei.initialCpbRemovalDelay[i][j][nalOrVcl] = code;
          sei_read_code( pDecodedMessageOutputStream, sei.initialCpbRemovalDelayLength, code, nalOrVcl ? "vcl_initial_cpb_removal_offset[i][j]" : "nal_initial_cpb_removal_offset[i][j]" );
          sei.initialCpbRemovalDelay[i][j][nalOrVcl] = code;
        }
      }
    }
  }
  if (sei.bpMaxSubLayers-1 > 0) 
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "bp_sublayer_dpb_output_offsets_present_flag");
    sei.sublayerDpbOutputOffsetsPresent = code; 
  }
  else
  {
    sei.sublayerDpbOutputOffsetsPresent = false;
  }
  if(sei.sublayerDpbOutputOffsetsPresent)
  {
    for(int i = 0; i < sei.bpMaxSubLayers - 1; i++)
    {
      sei_read_uvlc( pDecodedMessageOutputStream, code, "dpb_output_tid_offset[i]" );
      sei.dpbOutputTidOffset[i] = code;
    }
    sei.dpbOutputTidOffset[sei.bpMaxSubLayers-1] = 0;
  }
  sei_read_flag(pDecodedMessageOutputStream, code, "bp_alt_cpb_params_present_flag");
  sei.altCpbParamsPresent = code;
  if (sei.altCpbParamsPresent)
  {
    sei_read_flag(pDecodedMessageOutputStream, code, "use_alt_cpb_params_flag"); sei.useAltCpbParamsFlag = code;
  }

}

void SEIReader::xParseSEIPictureTiming(SEIPictureTiming& sei, uint32_t payloadSize, const uint32_t temporalId, const SEIBufferingPeriod& bp, std::ostream *pDecodedMessageOutputStream)
{

  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  uint32_t symbol;
  sei_read_code( pDecodedMessageOutputStream, bp.cpbRemovalDelayLength, symbol, "pt_cpb_removal_delay_minus1[bp_max_sub_layers_minus1]" );
  sei.auCpbRemovalDelay[bp.bpMaxSubLayers - 1] = symbol + 1;
  if (bp.bpMaxSubLayers == 1)
  {
    sei.ptSubLayerDelaysPresent[0] = true;
  }
  for (int i = temporalId; i < bp.bpMaxSubLayers - 1; i++)
  {
    sei_read_flag(pDecodedMessageOutputStream, symbol, "pt_sub_layer_delays_present_flag[i]");
    sei.ptSubLayerDelaysPresent[i] = (symbol == 1);
    if (sei.ptSubLayerDelaysPresent[i])
    {
      if (bp.cpbRemovalDelayDeltasPresent)
      {
        sei_read_flag(pDecodedMessageOutputStream, symbol, "pt_cpb_removal_delay_delta_enabled_flag[i]");
        sei.cpbRemovalDelayDeltaEnabledFlag[i] = (symbol == 1);
      }
      else
      {
        sei.cpbRemovalDelayDeltaEnabledFlag[i] = false;
      }
      if (sei.cpbRemovalDelayDeltaEnabledFlag[i])
      {
        if ((bp.numCpbRemovalDelayDeltas - 1) > 0)
        {
          sei_read_code(pDecodedMessageOutputStream, ceilLog2(bp.numCpbRemovalDelayDeltas), symbol, "pt_cpb_removal_delay_delta_idx[i]");
          sei.cpbRemovalDelayDeltaIdx[i] = symbol;
        }
        else
        {
          sei.cpbRemovalDelayDeltaIdx[i] = 0;
        }
      }
      else
      {
        sei_read_code(pDecodedMessageOutputStream, bp.cpbRemovalDelayLength, symbol, "pt_cpb_removal_delay_minus1[i]");
        sei.auCpbRemovalDelay[i] = symbol + 1;
      }
    }
  }
  sei_read_code(pDecodedMessageOutputStream, bp.dpbOutputDelayLength, symbol, "pt_dpb_output_delay");
  sei.picDpbOutputDelay = symbol;

  if( bp.altCpbParamsPresent )
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol, "cpb_alt_timing_info_present_flag" ); sei.cpbAltTimingInfoPresent = symbol;
    if( sei.cpbAltTimingInfoPresent )
    {
      if (bp.bpNalCpbParamsPresent)
      {
        sei.nalCpbAltInitialRemovalDelayDelta.resize(bp.bpMaxSubLayers);
        sei.nalCpbAltInitialRemovalOffsetDelta.resize(bp.bpMaxSubLayers);
        for (int i = 0; i <= bp.bpMaxSubLayers - 1; ++i)
        {
          sei.nalCpbAltInitialRemovalDelayDelta[i].resize(bp.bpCpbCnt, 0);
          sei.nalCpbAltInitialRemovalOffsetDelta[i].resize(bp.bpCpbCnt, 0);
        }
        sei.nalCpbDelayOffset.resize(bp.bpMaxSubLayers, 0);
        sei.nalDpbDelayOffset.resize(bp.bpMaxSubLayers, 0);
        for (int i = (bp.sublayerInitialCpbRemovalDelayPresent ? 0 : bp.bpMaxSubLayers - 1);
             i <= bp.bpMaxSubLayers - 1; ++i)
        {
          for (int j = 0; j < bp.bpCpbCnt; j++)
          {
            sei_read_code(pDecodedMessageOutputStream, bp.initialCpbRemovalDelayLength, symbol,
                          "nal_cpb_alt_initial_cpb_removal_delay_delta[ i ][ j ]");
            sei.nalCpbAltInitialRemovalDelayDelta[i][j] = symbol;
            sei_read_code(pDecodedMessageOutputStream, bp.initialCpbRemovalDelayLength, symbol,
                          "nal_cpb_alt_initial_cpb_removal_offset_delta[ i ][ j ]");
            sei.nalCpbAltInitialRemovalOffsetDelta[i][j] = symbol;
          }
          sei_read_code(pDecodedMessageOutputStream, bp.cpbRemovalDelayLength, sei.nalCpbDelayOffset[i],
                        "nal_cpb_delay_offset[ i ]");
          sei_read_code(pDecodedMessageOutputStream, bp.dpbOutputDelayLength, sei.nalDpbDelayOffset[i],
                        "nal_dpb_delay_offset[ i ]");
        }
      }

      if (bp.bpVclCpbParamsPresent)
      {
        sei.vclCpbAltInitialRemovalDelayDelta.resize(bp.bpMaxSubLayers);
        sei.vclCpbAltInitialRemovalOffsetDelta.resize(bp.bpMaxSubLayers);
        for (int i = 0; i <= bp.bpMaxSubLayers - 1; ++i)
        {
          sei.vclCpbAltInitialRemovalDelayDelta[i].resize(bp.bpCpbCnt, 0);
          sei.vclCpbAltInitialRemovalOffsetDelta[i].resize(bp.bpCpbCnt, 0);
        }
        sei.vclCpbDelayOffset.resize(bp.bpMaxSubLayers, 0);
        sei.vclDpbDelayOffset.resize(bp.bpMaxSubLayers, 0);
        for (int i = (bp.sublayerInitialCpbRemovalDelayPresent ? 0 : bp.bpMaxSubLayers - 1);
             i <= bp.bpMaxSubLayers - 1; ++i)
        {
          for (int j = 0; j < bp.bpCpbCnt; j++)
          {
            sei_read_code(pDecodedMessageOutputStream, bp.initialCpbRemovalDelayLength, symbol,
                          "vcl_cpb_alt_initial_cpb_removal_delay_delta[ i ][ j ]");
            sei.vclCpbAltInitialRemovalDelayDelta[i][j] = symbol;
            sei_read_code(pDecodedMessageOutputStream, bp.initialCpbRemovalDelayLength, symbol,
                          "vcl_cpb_alt_initial_cpb_removal_offset_delta[ i ][ j ]");
            sei.vclCpbAltInitialRemovalOffsetDelta[i][j] = symbol;
          }
          sei_read_code(pDecodedMessageOutputStream, bp.cpbRemovalDelayLength, sei.vclCpbDelayOffset[i],
                        "vcl_cpb_delay_offset[ i ]");
          sei_read_code(pDecodedMessageOutputStream, bp.dpbOutputDelayLength, sei.vclDpbDelayOffset[i],
                        "vcl_dpb_delay_offset[ i ]");
        }
      }
    }
  }
  else
  {
    sei.cpbAltTimingInfoPresent = false;
    sei.nalCpbAltInitialRemovalDelayDelta.resize(bp.bpMaxSubLayers);
    sei.nalCpbAltInitialRemovalOffsetDelta.resize(bp.bpMaxSubLayers);
    sei.nalCpbDelayOffset.resize(bp.bpMaxSubLayers, 0);
    sei.nalDpbDelayOffset.resize(bp.bpMaxSubLayers, 0);
    for (int i = (bp.sublayerInitialCpbRemovalDelayPresent ? 0 : bp.bpMaxSubLayers - 1);
         i <= bp.bpMaxSubLayers - 1; ++i)
    {
      sei.nalCpbAltInitialRemovalDelayDelta[i].resize(bp.bpCpbCnt, 0);
      sei.nalCpbAltInitialRemovalOffsetDelta[i].resize(bp.bpCpbCnt, 0);
    }

    sei.vclCpbAltInitialRemovalDelayDelta.resize(bp.bpMaxSubLayers);
    sei.vclCpbAltInitialRemovalOffsetDelta.resize(bp.bpMaxSubLayers);
    sei.vclCpbDelayOffset.resize(bp.bpMaxSubLayers, 0);
    sei.vclDpbDelayOffset.resize(bp.bpMaxSubLayers, 0);
    for (int i = (bp.sublayerInitialCpbRemovalDelayPresent ? 0 : bp.bpMaxSubLayers - 1);
         i <= bp.bpMaxSubLayers - 1; ++i)
    {
      sei.vclCpbAltInitialRemovalDelayDelta[i].resize(bp.bpCpbCnt, 0);
      sei.vclCpbAltInitialRemovalOffsetDelta[i].resize(bp.bpCpbCnt, 0);
    }
  }

  if ( bp.bpDecodingUnitHrdParamsPresent && bp.decodingUnitDpbDuParamsInPicTimingSeiFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, bp.dpbOutputDelayDuLength, symbol, "pic_dpb_output_du_delay" );
    sei.picDpbOutputDuDelay = symbol;
  }
  if( bp.bpDecodingUnitHrdParamsPresent && bp.decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    sei_read_uvlc( pDecodedMessageOutputStream, symbol, "num_decoding_units_minus1" );
    sei.numDecodingUnitsMinus1 = symbol;
    sei.numNalusInDuMinus1.resize(sei.numDecodingUnitsMinus1 + 1 );
    sei.duCpbRemovalDelayMinus1.resize( (sei.numDecodingUnitsMinus1 + 1) * bp.bpMaxSubLayers );

    if (sei.numDecodingUnitsMinus1 > 0)
    {
      sei_read_flag( pDecodedMessageOutputStream, symbol, "du_common_cpb_removal_delay_flag" );
      sei.duCommonCpbRemovalDelayFlag = symbol;
      if( sei.duCommonCpbRemovalDelayFlag )
      {
        for( int i = temporalId; i < bp.bpMaxSubLayers - 1; i ++ )
        {
          if( sei.ptSubLayerDelaysPresent[i] )
          {
            sei_read_code( pDecodedMessageOutputStream, bp.duCpbRemovalDelayIncrementLength, symbol, "du_common_cpb_removal_delay_increment_minus1[i]" );
            sei.duCommonCpbRemovalDelayMinus1[i] = symbol;
          }
        }
      }
      for( int i = 0; i <= sei.numDecodingUnitsMinus1; i ++ )
      {
        sei_read_uvlc( pDecodedMessageOutputStream, symbol, "num_nalus_in_du_minus1[i]" );
        sei.numNalusInDuMinus1[i] = symbol;
        if( !sei.duCommonCpbRemovalDelayFlag && i < sei.numDecodingUnitsMinus1 )
        {
          for( int j = temporalId; j < bp.bpMaxSubLayers - 1; j ++ )
          {
            if( sei.ptSubLayerDelaysPresent[j] )
            {
              sei_read_code( pDecodedMessageOutputStream, bp.duCpbRemovalDelayIncrementLength, symbol, "du_cpb_removal_delay_increment_minus1[i][j]" );
              sei.duCpbRemovalDelayMinus1[i * bp.bpMaxSubLayers + j] = symbol;
            }
          }
        }
      }
    }
    else
    {
      sei.duCommonCpbRemovalDelayFlag = 0;
    }
  }

  if (bp.additionalConcatenationInfoPresent)
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol, "pt_delay_for_concatenation_ensured_flag" );
    sei.delayForConcatenationEnsureFlag = symbol;
  }
  sei_read_code( pDecodedMessageOutputStream, 8, symbol, "pt_display_elemental_periods_minus1" );
  sei.ptDisplayElementalPeriodsMinus1 = symbol;
}

void SEIReader::xParseSEIFrameFieldinfo(SEIFrameFieldInfo& sei, const SEIPictureTiming& pt, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  uint32_t symbol;
  sei_read_flag( pDecodedMessageOutputStream, symbol,      "ffi_field_pic_flag" );
  sei.fieldPicFlag= symbol;
  if (sei.fieldPicFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "ffi_bottom_field_flag" );
    sei.bottomFieldFlag = symbol;
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "ffi_pairing_indicated_flag" );
    sei.pairingIndicatedFlag = symbol;
    if (sei.pairingIndicatedFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, symbol,  "ffi_paired_with_next_field_flag" );
      sei.pairedWithNextFieldFlag = symbol;
    }
  }
  else
  {
    sei_read_flag( pDecodedMessageOutputStream, symbol,    "ffi_display_fields_from_frame_flag" );
    sei.displayFieldsFromFrameFlag = symbol;
    if (sei.displayFieldsFromFrameFlag)
    {
      sei_read_flag( pDecodedMessageOutputStream, symbol,  "ffi_display_fields_from_frame_flag" );
      sei.topFieldFirstFlag = symbol;
    }
    sei_read_code( pDecodedMessageOutputStream, 8, symbol,    "ffi_display_elemental_periods_minus1" );
    sei.displayElementalPeriodsMinus1 = symbol;
  }
  sei_read_code( pDecodedMessageOutputStream, 2, symbol,   "ffi_source_scan_type" );
  sei.sourceScanType = symbol;
  sei_read_flag( pDecodedMessageOutputStream, symbol,      "ffi_duplicate_flag" );
  sei.duplicateFlag = symbol;
}

void SEIReader::xParseSEIDependentRAPIndication( SEIDependentRAPIndication& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream )
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
}


void SEIReader::xParseSEIFramePacking(SEIFramePacking& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_uvlc( pDecodedMessageOutputStream, val, "fp_arrangement_id" );                 sei.arrangementId = val;
  sei_read_flag( pDecodedMessageOutputStream, val, "fp_arrangement_cancel_flag" );        sei.arrangementCancelFlag = val;

  if( !sei.arrangementCancelFlag )
  {
    sei_read_code( pDecodedMessageOutputStream, 7, val, "fp_arrangement_type" );          sei.arrangementType = val;
    CHECK( ( sei.arrangementType <= 2 ) || ( sei.arrangementType >= 6 ), "Invalid arrangement type" );

    sei_read_flag( pDecodedMessageOutputStream, val, "fp_quincunx_sampling_flag" );                     sei.quincunxSamplingFlag = val;

    sei_read_code( pDecodedMessageOutputStream, 6, val, "fp_content_interpretation_type" );             sei.contentInterpretationType = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "fp_spatial_flipping_flag" );                      sei.spatialFlippingFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "fp_frame0_flipped_flag" );                        sei.frame0FlippedFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "fp_field_views_flag" );                           sei.fieldViewsFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "fp_current_frame_is_frame0_flag" );               sei.currentFrameIsFrame0Flag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "fp_frame0_self_contained_flag" );                 sei.frame0SelfContainedFlag = val;
    sei_read_flag( pDecodedMessageOutputStream, val, "fp_frame1_self_contained_flag" );                 sei.frame1SelfContainedFlag = val;

    if ( sei.quincunxSamplingFlag == 0 && sei.arrangementType != 5)
    {
      sei_read_code( pDecodedMessageOutputStream, 4, val, "fp_frame0_grid_position_x" );                sei.frame0GridPositionX = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "fp_frame0_grid_position_y" );                sei.frame0GridPositionY = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "fp_frame1_grid_position_x" );                sei.frame1GridPositionX = val;
      sei_read_code( pDecodedMessageOutputStream, 4, val, "fp_frame1_grid_position_y" );                sei.frame1GridPositionY = val;
    }

    sei_read_code( pDecodedMessageOutputStream, 8, val, "fp_frame_packing_arrangement_reserved_byte" );   sei.arrangementReservedByte = val;
    sei_read_flag( pDecodedMessageOutputStream, val,  "fp_frame_packing_arrangement_persistence_flag" );  sei.arrangementPersistenceFlag = (val != 0);
  }
  sei_read_flag( pDecodedMessageOutputStream, val, "fp_upsampled_aspect_ratio_flag" );                  sei.upsampledAspectRatio = val;
}

void SEIReader::xParseSEIParameterSetsInclusionIndication(SEIParameterSetsInclusionIndication& sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header( sei, pDecodedMessageOutputStream, payloadSize );

  sei_read_flag( pDecodedMessageOutputStream, val, "psii_self_contained_clvs_flag" );
  sei.selfContainedClvsFlag = val;
}

void SEIReader::xParseSEIMasteringDisplayColourVolume(SEIMasteringDisplayColourVolume& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_display_primaries_x[0]" ); sei.values.primaries[0][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_display_primaries_y[0]" ); sei.values.primaries[0][1] = code;

  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_display_primaries_x[1]" ); sei.values.primaries[1][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_display_primaries_y[1]" ); sei.values.primaries[1][1] = code;

  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_display_primaries_x[2]" ); sei.values.primaries[2][0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_display_primaries_y[2]" ); sei.values.primaries[2][1] = code;


  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_white_point_x" ); sei.values.whitePoint[0] = code;
  sei_read_code( pDecodedMessageOutputStream, 16, code, "mdcv_white_point_y" ); sei.values.whitePoint[1] = code;

  sei_read_code( pDecodedMessageOutputStream, 32, code, "mdcv_max_display_mastering_luminance" ); sei.values.maxLuminance = code;
  sei_read_code( pDecodedMessageOutputStream, 32, code, "mdcv_min_display_mastering_luminance" ); sei.values.minLuminance = code;
}

void SEIReader::xParseSEIAlternativeTransferCharacteristics(SEIAlternativeTransferCharacteristics& sei, uint32_t payloadSize, std::ostream* pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 8, code, "preferred_transfer_characteristics"); sei.preferredTransferCharacteristics = code;
}

void SEIReader::xParseSEIUserDataRegistered(SEIUserDataRegistered& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  uint32_t code;
  assert(payloadSize>0);
  sei_read_code(pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code"); payloadSize--;
  if (code == 255)
  {
    assert(payloadSize>0);
    sei_read_code(pDecodedMessageOutputStream, 8, code, "itu_t_t35_country_code_extension_byte"); payloadSize--;
    code += 255;
  }
  sei.ituCountryCode = code;
  sei.userData.resize(payloadSize);
  for (uint32_t i = 0; i < sei.userData.size(); i++)
  {
    sei_read_code(NULL, 8, code, "itu_t_t35_payload_byte");
    sei.userData[i] = code;
  }
  if (pDecodedMessageOutputStream)
  {
    (*pDecodedMessageOutputStream) << "  itu_t_t35 payload size: " << sei.userData.size() << "\n";
  }
}

void SEIReader::xParseSEIFilmGrainCharacteristics(SEIFilmGrainCharacteristics& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag(pDecodedMessageOutputStream, code, "fg_characteristics_cancel_flag");     sei.filmGrainCharacteristicsCancelFlag = code != 0;
  if (!sei.filmGrainCharacteristicsCancelFlag)
  {
    sei_read_code(pDecodedMessageOutputStream, 2, code, "fg_model_id");                   sei.filmGrainModelId = code;
    sei_read_flag(pDecodedMessageOutputStream, code, "separate_colour_description_present_flag"); sei.separateColourDescriptionPresent = code != 0;
    if (sei.separateColourDescriptionPresent)
    {
      sei_read_code(pDecodedMessageOutputStream, 3, code, "fg_bit_depth_luma_minus8");    sei.filmGrainBitDepthLumaMinus8 = code;
      sei_read_code(pDecodedMessageOutputStream, 3, code, "fg_bit_depth_chroma_minus8");  sei.filmGrainBitDepthChromaMinus8 = code;
      sei_read_flag(pDecodedMessageOutputStream, code, "fg_full_range_flag");             sei.filmGrainFullRangeFlag = code != 0;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "fg_colour_primaries");         sei.filmGrainColourPrimaries = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "fg_transfer_characteristics"); sei.filmGrainTransferCharacteristics = code;
      sei_read_code(pDecodedMessageOutputStream, 8, code, "fg_matrix_coeffs");            sei.filmGrainMatrixCoeffs = code;
    }
    sei_read_code(pDecodedMessageOutputStream, 2, code, "fg_blending_mode_id");                      sei.blendingModeId = code;
    sei_read_code(pDecodedMessageOutputStream, 4, code, "fg_log2_scale_factor");                     sei.log2ScaleFactor = code;
    for (int c = 0; c<3; c++)
    {
      sei_read_flag(pDecodedMessageOutputStream, code, "fg_comp_model_present_flag[c]");             sei.compModel[c].presentFlag = code != 0;
    }
    for (int c = 0; c<3; c++)
    {
      SEIFilmGrainCharacteristics::CompModel &cm = sei.compModel[c];
      if (cm.presentFlag)
      {
        uint32_t numIntensityIntervals;
        sei_read_code(pDecodedMessageOutputStream, 8, code, "fg_num_intensity_intervals_minus1[c]"); numIntensityIntervals = code + 1;
        sei_read_code(pDecodedMessageOutputStream, 3, code, "fg_num_model_values_minus1[c]");        cm.numModelValues = code + 1;
        cm.intensityValues.resize(numIntensityIntervals);
        for (uint32_t interval = 0; interval<numIntensityIntervals; interval++)
        {
          SEIFilmGrainCharacteristics::CompModelIntensityValues &cmiv = cm.intensityValues[interval];
          sei_read_code(pDecodedMessageOutputStream, 8, code, "fg_intensity_interval_lower_bound[c][i]"); cmiv.intensityIntervalLowerBound = code;
          sei_read_code(pDecodedMessageOutputStream, 8, code, "fg_intensity_interval_upper_bound[c][i]"); cmiv.intensityIntervalUpperBound = code;
          cmiv.compModelValue.resize(cm.numModelValues);
          for (uint32_t j = 0; j<cm.numModelValues; j++)
          {
            sei_read_svlc(pDecodedMessageOutputStream, cmiv.compModelValue[j], "fg_comp_model_value[c][i]");
          }
        }
      }
    } // for c
    sei_read_flag(pDecodedMessageOutputStream, code, "fg_characteristics_persistence_flag"); sei.filmGrainCharacteristicsPersistenceFlag = code != 0;
  } // cancel flag
}

void SEIReader::xParseSEIContentLightLevelInfo(SEIContentLightLevelInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 16, code, "clli_max_content_light_level");     sei.maxContentLightLevel = code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "clli_max_pic_average_light_level"); sei.maxPicAverageLightLevel = code;
}

void SEIReader::xParseSEIAmbientViewingEnvironment(SEIAmbientViewingEnvironment& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_code(pDecodedMessageOutputStream, 32, code, "ambient_illuminance"); sei.ambientIlluminance = code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "ambient_light_x");     sei.ambientLightX = (uint16_t)code;
  sei_read_code(pDecodedMessageOutputStream, 16, code, "ambient_light_y");     sei.ambientLightY = (uint16_t)code;
}

void SEIReader::xParseSEIContentColourVolume(SEIContentColourVolume& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  int i;
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag(pDecodedMessageOutputStream, val, "ccv_cancel_flag");          sei.ccvCancelFlag = val;
  if (!sei.ccvCancelFlag)
  {
    int iVal;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_persistence_flag");   sei.ccvPersistenceFlag = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_primaries_present_flag");   sei.ccvPrimariesPresent = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_min_luminance_value_present_flag");   sei.ccvMinLuminanceValuePresent = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_max_luminance_value_present_flag");   sei.ccvMaxLuminanceValuePresent = val;
    sei_read_flag(pDecodedMessageOutputStream, val, "ccv_avg_luminance_value_present_flag");   sei.ccvAvgLuminanceValuePresent = val;

    if (sei.ccvPrimariesPresent)
    {
      for (i = 0; i < MAX_NUM_COMP; i++)
      {
        sei_read_scode(pDecodedMessageOutputStream, 32, iVal, "ccv_primaries_x[i]");          sei.ccvPrimariesX[i] = iVal;
        sei_read_scode(pDecodedMessageOutputStream, 32, iVal, "ccv_primaries_y[i]");          sei.ccvPrimariesY[i] = iVal;
      }
    }
    if (sei.ccvMinLuminanceValuePresent)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_min_luminance_value");   sei.ccvMinLuminanceValue = val;
    }
    if (sei.ccvMaxLuminanceValuePresent)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_max_luminance_value");   sei.ccvMaxLuminanceValue = val;
    }
    if (sei.ccvAvgLuminanceValuePresent)
    {
      sei_read_code(pDecodedMessageOutputStream, 32, val, "ccv_avg_luminance_value");   sei.ccvAvgLuminanceValue = val;
    }
  }
}
void SEIReader::xParseSEIEquirectangularProjection(SEIEquirectangularProjection& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream, val,       "erp_cancel_flag" );              sei.erpCancelFlag = val;
  if( !sei.erpCancelFlag )
  {
    sei_read_flag( pDecodedMessageOutputStream, val,      "erp_persistence_flag"    );     sei.erpPersistenceFlag   = val;
    sei_read_flag( pDecodedMessageOutputStream, val,      "erp_guard_band_flag"     );     sei.erpGuardBandFlag     = val;
    sei_read_code( pDecodedMessageOutputStream, 2, val,   "erp_reserved_zero_2bits" );
    if ( sei.erpGuardBandFlag == 1)
    {
      sei_read_code( pDecodedMessageOutputStream, 3, val,     "erp_guard_band_type"       );   sei.erpGuardBandType  = val;
      sei_read_code( pDecodedMessageOutputStream, 8, val,     "erp_left_guard_band_width" );   sei.erpLeftGuardBandWidth = val;
      sei_read_code( pDecodedMessageOutputStream, 8, val,     "erp_right_guard_band_width");   sei.erpRightGuardBandWidth = val;
    }
  }
}

void SEIReader::xParseSEISphereRotation(SEISphereRotation& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  int  sval;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_flag( pDecodedMessageOutputStream, val,       "sphere_rotation_cancel_flag" );              sei.sphereRotationCancelFlag = val;
  if( !sei.sphereRotationCancelFlag )
  {
    sei_read_flag ( pDecodedMessageOutputStream,      val,   "sphere_rotation_persistence_flag"    );     sei.sphereRotationPersistenceFlag = val;
    sei_read_code ( pDecodedMessageOutputStream, 6,   val,   "sphere_rotation_reserved_zero_6bits" );
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_yaw"                 );     sei.sphereRotationYaw = sval;
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_pitch"               );     sei.sphereRotationPitch = sval;
    sei_read_scode( pDecodedMessageOutputStream, 32, sval,   "sphere_rotation_roll"                );     sei.sphereRotationRoll = sval;
  }
}

void SEIReader::xParseSEIOmniViewport(SEIOmniViewport& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t code;
  int  scode;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  sei_read_code( pDecodedMessageOutputStream, 10, code, "omni_viewport_id"          ); sei.omniViewportId         = code;
  sei_read_flag( pDecodedMessageOutputStream,     code, "omni_viewport_cancel_flag" ); sei.omniViewportCancelFlag = code;
  if (!sei.omniViewportCancelFlag)
  {
    uint32_t numRegions;
    sei_read_flag( pDecodedMessageOutputStream,    code,       "omni_viewport_persistence_flag" ); sei.omniViewportPersistenceFlag = code;
    sei_read_code( pDecodedMessageOutputStream, 4, numRegions, "omni_viewport_cnt_minus1"       ); numRegions++;
    sei.omniViewportRegions.resize(numRegions);
    for(uint32_t region=0; region<numRegions; region++)
    {
      SEIOmniViewport::OmniViewport &viewport = sei.omniViewportRegions[region];
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_azimuth_centre"   );   viewport.azimuthCentre = scode;
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_elevation_centre" );   viewport.elevationCentre = scode;
      sei_read_scode( pDecodedMessageOutputStream, 32, scode, "omni_viewport_tilt_centre"      );   viewport.tiltCentre = code;
      sei_read_code( pDecodedMessageOutputStream,  32, code, "omni_viewport_hor_range"         );   viewport.horRange        = code;
      sei_read_code( pDecodedMessageOutputStream,  32, code, "omni_viewport_ver_range"         );   viewport.verRange        = code;
    }
  }
  else
  {
    sei.omniViewportRegions.clear();
    sei.omniViewportPersistenceFlag=false;
  }
}

void SEIReader::xParseSEIRegionWisePacking(SEIRegionWisePacking& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;

  sei_read_flag( pDecodedMessageOutputStream,           val,      "rwp_cancel_flag" );                      sei.rwpCancelFlag = val;
  if (!sei.rwpCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,           val,    "rwp_persistence_flag" );                 sei.rwpPersistenceFlag = val;
    sei_read_flag( pDecodedMessageOutputStream,           val,    "rwp_constituent_picture_matching_flag" );    sei.constituentPictureMatchingFlag = val;
    sei_read_code( pDecodedMessageOutputStream,       5,  val,    "rwp_reserved_zero_5bits" );
    sei_read_code( pDecodedMessageOutputStream,       8,  val,    "rwp_num_packed_regions" );                   sei.numPackedRegions = val;
    sei_read_code( pDecodedMessageOutputStream,       32, val,    "rwp_proj_picture_width" );                   sei.projPictureWidth = val;
    sei_read_code( pDecodedMessageOutputStream,       32, val,    "rwp_proj_picture_height" );                  sei.projPictureHeight = val;
    sei_read_code( pDecodedMessageOutputStream,       16, val,    "rwp_packed_picture_width" );                 sei.packedPictureWidth = val;
    sei_read_code( pDecodedMessageOutputStream,       16, val,    "rwp_packed_picture_height" );                sei.packedPictureHeight = val;

    sei.rwpTransformType.resize(sei.numPackedRegions);
    sei.rwpGuardBandFlag.resize(sei.numPackedRegions);
    sei.projRegionWidth.resize(sei.numPackedRegions);
    sei.projRegionHeight.resize(sei.numPackedRegions);
    sei.rwpProjRegionTop.resize(sei.numPackedRegions);
    sei.projRegionLeft.resize(sei.numPackedRegions);
    sei.packedRegionWidth.resize(sei.numPackedRegions);
    sei.packedRegionHeight.resize(sei.numPackedRegions);
    sei.packedRegionTop.resize(sei.numPackedRegions);
    sei.packedRegionLeft.resize(sei.numPackedRegions);
    sei.rwpLeftGuardBandWidth.resize(sei.numPackedRegions);
    sei.rwpRightGuardBandWidth.resize(sei.numPackedRegions);
    sei.rwpTopGuardBandHeight.resize(sei.numPackedRegions);
    sei.rwpBottomGuardBandHeight.resize(sei.numPackedRegions);
    sei.rwpGuardBandNotUsedForPredFlag.resize(sei.numPackedRegions);
    sei.rwpGuardBandType.resize(4*sei.numPackedRegions);

    for( int i=0; i < sei.numPackedRegions; i++ )
    {
      sei_read_code( pDecodedMessageOutputStream,     4,  val,    "rwp_reserved_zero_4bits" );
      sei_read_code( pDecodedMessageOutputStream,     3,  val,    "rwp_tTransform_type" );                  sei.rwpTransformType[i] = val;
      sei_read_flag( pDecodedMessageOutputStream,         val,    "rwp_guard_band_flag" );                  sei.rwpGuardBandFlag[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "rwp_proj_region_width" );                    sei.projRegionWidth[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "rwp_proj_region_height" );                   sei.projRegionHeight[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "rwp_proj_region_top" );                   sei.rwpProjRegionTop[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     32, val,    "rwp_proj_region_left" );                     sei.projRegionLeft[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "rwp_packed_region_width" );                  sei.packedRegionWidth[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "rwp_packed_region_height" );                 sei.packedRegionHeight[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "rwp_packed_region_top" );                    sei.packedRegionTop[i] = val;
      sei_read_code( pDecodedMessageOutputStream,     16, val,    "rwp_packed_region_left" );                   sei.packedRegionLeft[i] = val;
      if( sei.rwpGuardBandFlag[i] )
      {
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_left_guard_band_width" );            sei.rwpLeftGuardBandWidth[i] = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_right_guard_band_width" );           sei.rwpRightGuardBandWidth[i] = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_top_guard_band_height" );            sei.rwpTopGuardBandHeight[i]  = val;
        sei_read_code( pDecodedMessageOutputStream,   8,  val,    "rwp_bottom_guard_band_height" );         sei.rwpBottomGuardBandHeight[i]  = val;
        sei_read_flag( pDecodedMessageOutputStream,       val,    "rwp_guard_band_not_used_forPred_flag" ); sei.rwpGuardBandNotUsedForPredFlag[i] = val;
        for( int j=0; j < 4; j++ )
        {
          sei_read_code( pDecodedMessageOutputStream, 3,  val,     "rwp_guard_band_type" ); sei.rwpGuardBandType[i*4 + j] = val;
        }
        sei_read_code( pDecodedMessageOutputStream,   3,  val,    "rwp_guard_band_reserved_zero_3bits" );
      }
    }
  }
}

void SEIReader::xParseSEIGeneralizedCubemapProjection(SEIGeneralizedCubemapProjection& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  uint32_t val;
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);

  sei_read_flag( pDecodedMessageOutputStream,          val,    "gcmp_cancel_flag" );                      sei.gcmpCancelFlag = val;
  if (!sei.gcmpCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,        val,    "gcmp_persistence_flag"    );              sei.gcmpPersistenceFlag = val;
    sei_read_code( pDecodedMessageOutputStream,     3, val,    "gcmp_packing_type" );                     sei.gcmpPackingType = val;
    sei_read_code( pDecodedMessageOutputStream,     2, val,    "gcmp_mapping_function_type"     );        sei.gcmpMappingFunctionType = val;

    int numFace = sei.gcmpPackingType == 4 || sei.gcmpPackingType == 5 ? 5 : 6;
    sei.gcmpFaceIndex.resize(numFace);
    sei.gcmpFaceRotation.resize(numFace);
    if (sei.gcmpMappingFunctionType == 2)
    {
      sei.gcmpFunctionCoeffU.resize(numFace);
      sei.gcmpFunctionUAffectedByVFlag.resize(numFace);
      sei.gcmpFunctionCoeffV.resize(numFace);
      sei.gcmpFunctionVAffectedByUFlag.resize(numFace);
    }

    for (int i = 0; i < numFace; i++)
    {
      sei_read_code( pDecodedMessageOutputStream,   3, val,    "gcmp_face_index" );                       sei.gcmpFaceIndex[i] = val;
      sei_read_code( pDecodedMessageOutputStream,   2, val,    "gcmp_face_rotation" );                    sei.gcmpFaceRotation[i] = val;
      if (sei.gcmpMappingFunctionType == 2)
      {
        sei_read_code( pDecodedMessageOutputStream, 7, val,    "gcmp_function_coeff_u" );                 sei.gcmpFunctionCoeffU[i] = val;
        sei_read_flag( pDecodedMessageOutputStream,    val,    "gcmp_function_u_affected_by_v_flag"    ); sei.gcmpFunctionUAffectedByVFlag[i] = val;
        sei_read_code( pDecodedMessageOutputStream, 7, val,    "gcmp_function_coeff_v" );                 sei.gcmpFunctionCoeffV[i] = val;
        sei_read_flag( pDecodedMessageOutputStream,    val,    "gcmp_function_v_affected_by_u_flag"    ); sei.gcmpFunctionVAffectedByUFlag[i] = val;
      }
    }
    sei_read_flag( pDecodedMessageOutputStream,        val,    "gcmp_guard_band_flag" );                  sei.gcmpGuardBandFlag = val;
    if (sei.gcmpGuardBandFlag)
    {
      sei_read_code( pDecodedMessageOutputStream,   3, val,    "gcmp_guard_band_type" );                   sei.gcmpGuardBandType = val;
      sei_read_flag( pDecodedMessageOutputStream,      val,    "gcmp_guard_band_boundary_exterior_flag" ); sei.gcmpGuardBandBoundaryExteriorFlag = val;
      sei_read_code( pDecodedMessageOutputStream,   4, val,    "gcmp_guard_band_samples_minus1" );         sei.gcmpGuardBandSamplesMinus1 = val;
    }
  }
}

void SEIReader::xParseSEISubpictureLevelInfo(SEISubpicureLevelInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;
  sei_read_code( pDecodedMessageOutputStream,   3,  val,    "sli_num_ref_levels_minus1" );            sei.numRefLevels  = val + 1;
  sei_read_flag( pDecodedMessageOutputStream,       val,    "sli_cbr_constraint_flag" );              sei.cbrConstraintFlag = val;
  sei_read_flag( pDecodedMessageOutputStream,       val,    "sli_explicit_fraction_present_flag" );   sei.explicitFractionPresent = val;
  if (sei.explicitFractionPresent)
  {
    sei_read_uvlc(pDecodedMessageOutputStream,      val,    "sli_num_subpics_minus1");             sei.numSubpics = val + 1;
    sei_read_code(pDecodedMessageOutputStream,  3,  val,    "sli_max_sublayers_minus1"  );            sei.sliMaxSublayers = val + 1;
    sei_read_flag(pDecodedMessageOutputStream,      val,    "sli_sublayer_info_present_flag");        sei.sliSublayerInfoPresent = val;
    while (!isByteAligned())
    {
      sei_read_flag( pDecodedMessageOutputStream,   val,    "sli_alignment_zero_bit" );           CHECK (val != 0, "sli_alignment_zero_bit not equal to zero" );
    }
  }

  sei.refLevelIdc.resize(sei.numRefLevels);
  sei.nonSubpicLayersFraction.resize(sei.numRefLevels);
  // sei parameters initialization
  for (int i = 0; i < sei.numRefLevels; i++)
  {
    sei.nonSubpicLayersFraction[i].resize(sei.sliMaxSublayers);
    sei.refLevelIdc[i].resize(sei.sliMaxSublayers);
    for (int k = 0; k < sei.sliMaxSublayers; k++)
      {
      sei.refLevelIdc[i][k] = vvencLevel::VVENC_LEVEL15_5;
    }
  }
  if (sei.explicitFractionPresent)
  {
    sei.refLevelFraction.resize(sei.numRefLevels);
    for (int i = 0; i < sei.numRefLevels; i++)
    {
      sei.refLevelFraction[i].resize(sei.numSubpics);
      for (int j = 0; j < sei.numSubpics; j++)
        {
        sei.refLevelFraction[i][j].resize(sei.sliMaxSublayers);
        for (int k = 0; k < sei.sliMaxSublayers; k++)
        {
          sei.refLevelFraction[i][j][k] = 0;
        }
      }
    }
  }

  // parsing
  for (int k = sei.sliSublayerInfoPresent ? 0 : sei.sliMaxSublayers - 1; k < sei.sliMaxSublayers; k++)
  {
    for (int i = 0; i < sei.numRefLevels; i++)
    {
      sei_read_code(pDecodedMessageOutputStream, 8, val, "sli_non_subpic_layers_fraction[i][k]");    sei.nonSubpicLayersFraction[i][k] = (vvencLevel) val;
      sei_read_code(pDecodedMessageOutputStream, 8, val, "sli_ref_level_idc[i][k]");                 sei.refLevelIdc[i][k] = (vvencLevel) val;

      if (sei.explicitFractionPresent)
      {
        for (int j = 0; j < sei.numSubpics; j++)
        {
          sei_read_code(pDecodedMessageOutputStream, 8, val, "sli_ref_level_fraction_minus1[i][j][k]");  sei.refLevelFraction[i][j][k] = val;
        }
      }
    }
  }

  // update the inference of m_refLevelIdc[][] and m_refLevelFraction[][][]
  if (!sei.sliSublayerInfoPresent)
  {
    for (int k = sei.sliMaxSublayers - 2; k >= 0; k--)
    {
      for (int i = 0; i < sei.numRefLevels; i++)
      {
        sei.nonSubpicLayersFraction[i][k] = sei.nonSubpicLayersFraction[i][sei.sliMaxSublayers - 1];
        sei.refLevelIdc[i][k] = sei.refLevelIdc[i][sei.sliMaxSublayers - 1];
        if (sei.explicitFractionPresent)
        {
          for (int j = 0; j < sei.numSubpics; j++)
          {
            sei.refLevelFraction[i][j][k] = sei.refLevelFraction[i][j][sei.sliMaxSublayers - 1];
          }
        }
      }
    }
  }
}

void SEIReader::xParseSEISampleAspectRatioInfo(SEISampleAspectRatioInfo& sei, uint32_t payloadSize, std::ostream *pDecodedMessageOutputStream)
{
  output_sei_message_header(sei, pDecodedMessageOutputStream, payloadSize);
  uint32_t val;

  sei_read_flag( pDecodedMessageOutputStream,           val,    "sari_cancel_flag" );                      sei.sariCancelFlag = val;
  if (!sei.sariCancelFlag)
  {
    sei_read_flag( pDecodedMessageOutputStream,         val,    "sari_persistence_flag" );                 sei.sariPersistenceFlag = val;
    sei_read_code( pDecodedMessageOutputStream,     8,  val,    "sari_aspect_ratio_idc" );                 sei.sariAspectRatioIdc = val;
    if (sei.sariAspectRatioIdc == 255)
    {
      sei_read_code( pDecodedMessageOutputStream,  16,  val,    "sari_sar_width" );                        sei.sariSarWidth = val;
      sei_read_code( pDecodedMessageOutputStream,  16,  val,    "sari_sar_height" );                       sei.sariSarHeight = val;
    }
  }
}


}// namespace

//! \}


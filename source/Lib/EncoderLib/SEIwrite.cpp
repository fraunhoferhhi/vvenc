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

Copyright (c) 2019-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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


#include "SEIwrite.h"
#include "CommonLib/BitStream.h"
#include "CommonLib/SEI.h"
#include "CommonLib/Slice.h"
#include "CommonLib/Picture.h"
#include "CommonLib/dtrace_next.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

void SEIWriter::xWriteSEIpayloadData(OutputBitstream &bs, const SEI& sei, HRD &hrd, const uint32_t temporalId)
{
  const SEIBufferingPeriod *bp = NULL;
  switch (sei.payloadType())
  {
  case SEI::USER_DATA_UNREGISTERED:
    xWriteSEIuserDataUnregistered(*static_cast<const SEIuserDataUnregistered*>(&sei));
    break;
  case SEI::DECODING_UNIT_INFO:
    bp = &hrd.bufferingPeriodSEI;
    CHECK (bp == nullptr, "Buffering Period need to be initialized in HRD to allow writing of Decoding Unit Information SEI");
    xWriteSEIDecodingUnitInfo(*static_cast<const SEIDecodingUnitInfo*>(& sei), *bp, temporalId);
    break;
  case SEI::SCALABLE_NESTING:
    xWriteSEIScalableNesting(bs, *static_cast<const SEIScalableNesting*>(&sei));
    break;
  case SEI::DECODED_PICTURE_HASH:
    xWriteSEIDecodedPictureHash(*static_cast<const SEIDecodedPictureHash*>(&sei));
    break;
  case SEI::BUFFERING_PERIOD:
    xWriteSEIBufferingPeriod(*static_cast<const SEIBufferingPeriod*>(&sei));
    hrd.bufferingPeriodSEI = *(static_cast<const SEIBufferingPeriod*>(&sei));
    break;
  case SEI::PICTURE_TIMING:
    {
      bp = &hrd.bufferingPeriodSEI;
      CHECK (bp == nullptr, "Buffering Period need to be initialized in HRD to allow writing of Picture Timing SEI");
      xWriteSEIPictureTiming(*static_cast<const SEIPictureTiming*>(&sei), *bp, temporalId);
    }
    break;
  case SEI::FRAME_FIELD_INFO:
    xWriteSEIFrameFieldInfo(*static_cast<const SEIFrameFieldInfo*>(&sei));
    break;
  case SEI::DEPENDENT_RAP_INDICATION:
    xWriteSEIDependentRAPIndication(*static_cast<const SEIDependentRAPIndication*>(&sei));
    break;
  case SEI::FRAME_PACKING:
    xWriteSEIFramePacking(*static_cast<const SEIFramePacking*>(&sei));
    break;
  case SEI::PARAMETER_SETS_INCLUSION_INDICATION:
    xWriteSEIParameterSetsInclusionIndication(*static_cast<const SEIParameterSetsInclusionIndication*>(&sei));
    break;
  case SEI::MASTERING_DISPLAY_COLOUR_VOLUME:
    xWriteSEIMasteringDisplayColourVolume(*static_cast<const SEIMasteringDisplayColourVolume*>(&sei));
    break;
  case SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS:
    xWriteSEIAlternativeTransferCharacteristics(*static_cast<const SEIAlternativeTransferCharacteristics*>(&sei));
    break;
  case SEI::EQUIRECTANGULAR_PROJECTION:
    xWriteSEIEquirectangularProjection(*static_cast<const SEIEquirectangularProjection*>(&sei));
    break;
  case SEI::SPHERE_ROTATION:
    xWriteSEISphereRotation(*static_cast<const SEISphereRotation*>(&sei));
    break;
  case SEI::OMNI_VIEWPORT:
    xWriteSEIOmniViewport(*static_cast<const SEIOmniViewport*>(&sei));
    break;
  case SEI::REGION_WISE_PACKING:
    xWriteSEIRegionWisePacking(*static_cast<const SEIRegionWisePacking*>(&sei));
    break;
  case SEI::GENERALIZED_CUBEMAP_PROJECTION:
    xWriteSEIGeneralizedCubemapProjection(*static_cast<const SEIGeneralizedCubemapProjection*>(&sei));
    break;
  case SEI::USER_DATA_REGISTERED_ITU_T_T35:
    xWriteSEIUserDataRegistered(*static_cast<const SEIUserDataRegistered*>(&sei));
    break;
  case SEI::FILM_GRAIN_CHARACTERISTICS:
    xWriteSEIFilmGrainCharacteristics(*static_cast<const SEIFilmGrainCharacteristics*>(&sei));
    break;
  case SEI::CONTENT_LIGHT_LEVEL_INFO:
    xWriteSEIContentLightLevelInfo(*static_cast<const SEIContentLightLevelInfo*>(&sei));
    break;
  case SEI::AMBIENT_VIEWING_ENVIRONMENT:
    xWriteSEIAmbientViewingEnvironment(*static_cast<const SEIAmbientViewingEnvironment*>(&sei));
    break;
  case SEI::CONTENT_COLOUR_VOLUME:
    xWriteSEIContentColourVolume(*static_cast<const SEIContentColourVolume*>(&sei));
    break;
  case SEI::SUBPICTURE_LEVEL_INFO:
    xWriteSEISubpictureLevelInfo(*static_cast<const SEISubpicureLevelInfo*>(&sei));
    break;
  case SEI::SAMPLE_ASPECT_RATIO_INFO:
    xWriteSEISampleAspectRatioInfo(*static_cast<const SEISampleAspectRatioInfo*>(&sei));
    break;
  default:
    THROW("Trying to write unhandled SEI message");
    break;
  }
  xWriteByteAlign();
}

/**
 * marshal all SEI messages in provided list into one bitstream bs
 */
void SEIWriter::writeSEImessages(OutputBitstream& bs, const SEIMessages &seiList, HRD &hrd, bool isNested, const uint32_t temporalId)
{
#if ENABLE_TRACING
  if (g_HLSTraceEnable)
      DTRACE( g_trace_ctx, D_HEADER, "=========== SEI message ===========\n" );
#endif
  OutputBitstream bs_count;

  for (SEIMessages::const_iterator sei=seiList.begin(); sei!=seiList.end(); sei++)
  {
    // calculate how large the payload data is
    // TODO: this would be far nicer if it used vectored buffers
    bs_count.clear();
    setBitstream(&bs_count);

#if ENABLE_TRACING
    bool traceEnable = g_HLSTraceEnable;
    g_HLSTraceEnable = false;
#endif
    xWriteSEIpayloadData(bs_count, **sei, hrd, temporalId);
#if ENABLE_TRACING
    g_HLSTraceEnable = traceEnable;
#endif
    uint32_t payload_data_num_bits = bs_count.getNumberOfWrittenBits();
    CHECK(0 != payload_data_num_bits % 8, "Invalid number of payload data bits");

    setBitstream(&bs);
    uint32_t payloadType = (*sei)->payloadType();
    for (; payloadType >= 0xff; payloadType -= 0xff)
    {
      WRITE_CODE(0xff, 8, "payload_type");
    }
    WRITE_CODE(payloadType, 8, "payload_type");

    uint32_t payloadSize = payload_data_num_bits/8;
    for (; payloadSize >= 0xff; payloadSize -= 0xff)
    {
      WRITE_CODE(0xff, 8, "payload_size");
    }
    WRITE_CODE(payloadSize, 8, "payload_size");

    /* payloadData */
#if ENABLE_TRACING
    if (g_HLSTraceEnable)
      DTRACE( g_trace_ctx, D_HEADER, "=========== %s SEI message ===========\n", SEI::getSEIMessageString( (SEI::PayloadType)payloadType ) );
#endif

    xWriteSEIpayloadData(bs, **sei, hrd, temporalId);
  }
  if (!isNested)
  {
    xWriteRbspTrailingBits();
  }
}

/**
 * marshal a user_data_unregistered SEI message sei, storing the marshalled
 * representation in bitstream bs.
 */
void SEIWriter::xWriteSEIuserDataUnregistered(const SEIuserDataUnregistered &sei)
{
  for (uint32_t i = 0; i < ISO_IEC_11578_LEN; i++)
  {
    WRITE_CODE(sei.uuid_iso_iec_11578[i], 8 , "sei.uuid_iso_iec_11578[i]");
  }

  for (uint32_t i = 0; i < sei.userDataLength; i++)
  {
    WRITE_CODE(sei.userData[i], 8 , "user_data_payload_byte");
  }
}

/**
 * marshal a decoded picture hash SEI message, storing the marshalled
 * representation in bitstream bs.
 */
void SEIWriter::xWriteSEIDecodedPictureHash(const SEIDecodedPictureHash& sei)
{
  const char *traceString="\0";
  switch (sei.method)
  {
    case HASHTYPE_MD5: traceString="picture_md5"; break;
    case HASHTYPE_CRC: traceString="picture_crc"; break;
    case HASHTYPE_CHECKSUM: traceString="picture_checksum"; break;
    default: THROW("Unknown hash type"); break;
  }

  if (traceString != 0) //use of this variable is needed to avoid a compiler error with G++ 4.6.1
  {
    WRITE_CODE(sei.method, 8, "dph_sei_hash_type");
    WRITE_CODE(sei.singleCompFlag, 1, "dph_sei_single_component_flag");
    WRITE_CODE(0, 7, "dph_sei_reserved_zero_7bits");
    for(uint32_t i=0; i<uint32_t(sei.pictureHash.hash.size()); i++)
    {
      WRITE_CODE(sei.pictureHash.hash[i], 8, traceString);
    }
  }
}


void SEIWriter::xWriteSEIDecodingUnitInfo(const SEIDecodingUnitInfo& sei, const SEIBufferingPeriod& bp, const uint32_t temporalId)
{
  WRITE_UVLC(sei.decodingUnitIdx, "decoding_unit_idx");
  if( !bp.decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    for (int i = temporalId; i <= bp.bpMaxSubLayers - 1; i++)
    {
      if (i < bp.bpMaxSubLayers - 1)
      {
      WRITE_FLAG(sei.duiSubLayerDelaysPresent[i], "dui_sub_layer_delays_present_flag[i]");
      }
      if( sei.duiSubLayerDelaysPresent[i] )
        WRITE_CODE( sei.duSptCpbRemovalDelayIncrement[i], bp.duCpbRemovalDelayIncrementLength, "du_spt_cpb_removal_delay_increment[i]");
    }
  }
  if (bp.decodingUnitDpbDuParamsInPicTimingSeiFlag)
  {
    WRITE_FLAG(sei.dpbOutputDuDelayPresent, "dpb_output_du_delay_present_flag");
  }
  if(sei.dpbOutputDuDelayPresent)
  {
    WRITE_CODE(sei.picSptDpbOutputDuDelay, bp.dpbOutputDelayDuLength, "pic_spt_dpb_output_du_delay");
  }
}

void SEIWriter::xWriteSEIBufferingPeriod(const SEIBufferingPeriod& sei)
{
  WRITE_FLAG( sei.bpNalCpbParamsPresent, "bp_nal_hrd_parameters_present_flag");
  WRITE_FLAG( sei.bpVclCpbParamsPresent, "bp_vcl_hrd_parameters_present_flag");
  CHECK(!sei.bpNalCpbParamsPresent && !sei.bpVclCpbParamsPresent, "bp_nal_hrd_parameters_present_flag and/or bp_vcl_hrd_parameters_present_flag must be true");
  CHECK (sei.initialCpbRemovalDelayLength < 1, "sei.initialCpbRemovalDelayLength must be > 0");
  WRITE_CODE( sei.initialCpbRemovalDelayLength - 1, 5, "initial_cpb_removal_delay_length_minus1" );
  CHECK (sei.cpbRemovalDelayLength < 1, "sei.cpbRemovalDelayLength must be > 0");
  WRITE_CODE( sei.cpbRemovalDelayLength - 1,        5, "cpb_removal_delay_length_minus1" );
  CHECK (sei.dpbOutputDelayLength < 1, "sei.dpbOutputDelayLength must be > 0");
  WRITE_CODE( sei.dpbOutputDelayLength - 1,         5, "dpb_output_delay_length_minus1" );
  WRITE_FLAG( sei.bpDecodingUnitHrdParamsPresent, "bp_decoding_unit_hrd_params_present_flag"  );
  if( sei.bpDecodingUnitHrdParamsPresent )
  {
    CHECK (sei.duCpbRemovalDelayIncrementLength < 1, "sei.duCpbRemovalDelayIncrementLength must be > 0");
    WRITE_CODE( sei.duCpbRemovalDelayIncrementLength - 1, 5, "du_cpb_removal_delay_increment_length_minus1" );
    CHECK (sei.dpbOutputDelayDuLength < 1, "sei.dpbOutputDelayDuLength must be > 0");
    WRITE_CODE( sei.dpbOutputDelayDuLength - 1, 5, "dpb_output_delay_du_length_minus1" );
    WRITE_FLAG( sei.decodingUnitCpbParamsInPicTimingSeiFlag, "decoding_unit_cpb_params_in_pic_timing_sei_flag" );
    WRITE_FLAG(sei.decodingUnitDpbDuParamsInPicTimingSeiFlag, "decoding_unit_dpb_du_params_in_pic_timing_sei_flag");
  }

  WRITE_FLAG( sei.concatenationFlag, "concatenation_flag");
  WRITE_FLAG( sei.additionalConcatenationInfoPresent, "additional_concatenation_info_present_flag");
  if (sei.additionalConcatenationInfoPresent)
  {
    WRITE_CODE( sei.maxInitialRemovalDelayForConcatenation, sei.initialCpbRemovalDelayLength, "max_initial_removal_delay_for_concatenation" );
  }

  CHECK (sei.auCpbRemovalDelayDelta < 1, "sei.auCpbRemovalDelayDelta must be > 0");
  WRITE_CODE( sei.auCpbRemovalDelayDelta - 1, sei.cpbRemovalDelayLength, "au_cpb_removal_delay_delta_minus1" );

  CHECK(sei.bpMaxSubLayers < 1, "bp_max_sub_layers_minus1 must be > 0");
  WRITE_CODE(sei.bpMaxSubLayers - 1, 3, "bp_max_sub_layers_minus1");
  if (sei.bpMaxSubLayers - 1 > 0)
  {
    WRITE_FLAG(sei.cpbRemovalDelayDeltasPresent, "cpb_removal_delay_deltas_present_flag");
  }

  if (sei.cpbRemovalDelayDeltasPresent)
  {
    CHECK (sei.numCpbRemovalDelayDeltas < 1, "m_numCpbRemovalDelayDeltas must be > 0");
    WRITE_UVLC( sei.numCpbRemovalDelayDeltas - 1, "num_cpb_removal_delay_deltas_minus1" );
    for( int i = 0; i < sei.numCpbRemovalDelayDeltas; i ++ )
    {
      WRITE_CODE( sei.cpbRemovalDelayDelta[i],        sei.cpbRemovalDelayLength, "cpb_removal_delay_delta[i]" );
    }
  }
  CHECK (sei.bpCpbCnt < 1, "sei.bpCpbCnt must be > 0");
  WRITE_UVLC( sei.bpCpbCnt - 1, "bp_cpb_cnt_minus1");
  if (sei.bpMaxSubLayers - 1 > 0)
  {
    WRITE_FLAG(sei.sublayerInitialCpbRemovalDelayPresent, "bp_sublayer_initial_cpb_removal_delay_present_flag");
  }
  for (int i = (sei.sublayerInitialCpbRemovalDelayPresent ? 0 : sei.bpMaxSubLayers - 1); i < sei.bpMaxSubLayers; i++)
  {
    for( int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( sei.bpNalCpbParamsPresent ) ) ||
         ( ( nalOrVcl == 1 ) && ( sei.bpVclCpbParamsPresent ) ) )
      {
        for( int j = 0; j < sei.bpCpbCnt; j ++ )
        {
          WRITE_CODE( sei.initialCpbRemovalDelay[j][i][nalOrVcl],  sei.initialCpbRemovalDelayLength,           "initial_cpb_removal_delay[j][i][nalOrVcl]" );
          WRITE_CODE( sei.initialCpbRemovalOffset[j][i][nalOrVcl], sei.initialCpbRemovalDelayLength,           "initial_cpb_removal_delay_offset[j][i][nalOrVcl]" );
        }
      }
    }
  }
  if (sei.bpMaxSubLayers-1 > 0) 
  {
    WRITE_FLAG(sei.sublayerDpbOutputOffsetsPresent, "bp_sublayer_dpb_output_offsets_present_flag");
  }

  if(sei.sublayerDpbOutputOffsetsPresent)
  {
    for(int i = 0; i < sei.bpMaxSubLayers - 1; i++)
    {
      WRITE_UVLC( sei.dpbOutputTidOffset[i], "dpb_output_tid_offset[i]" );
    }
  }
  WRITE_FLAG(sei.altCpbParamsPresent, "bp_alt_cpb_params_present_flag");
  if (sei.altCpbParamsPresent)
  {
    WRITE_FLAG(sei.useAltCpbParamsFlag, "use_alt_cpb_params_flag");
  }

}

void SEIWriter::xWriteSEIPictureTiming(const SEIPictureTiming& sei, const SEIBufferingPeriod &bp, const uint32_t temporalId)
{
  WRITE_CODE( sei.auCpbRemovalDelay[bp.bpMaxSubLayers - 1] - 1, bp.cpbRemovalDelayLength,               "pt_cpb_removal_delay_minus1[bp_max_sub_layers_minus1]" );
  for (int i = temporalId; i < bp.bpMaxSubLayers - 1; i++)
  {
    WRITE_FLAG(sei.ptSubLayerDelaysPresent[i], "pt_sub_layer_delays_present_flag[i]");
    if (sei.ptSubLayerDelaysPresent[i])
    {
      if (bp.cpbRemovalDelayDeltasPresent)
      {
        WRITE_FLAG(sei.cpbRemovalDelayDeltaEnabledFlag[i], "pt_cpb_removal_delay_delta_enabled_flag[i]");
      }
      if (sei.cpbRemovalDelayDeltaEnabledFlag[i])
      {
        if ((bp.numCpbRemovalDelayDeltas - 1) > 0)
        {
          WRITE_CODE(sei.cpbRemovalDelayDeltaIdx[i], ceilLog2(bp.numCpbRemovalDelayDeltas), "pt_cpb_removal_delay_delta_idx[i]");
        }
      }
      else
      {
        WRITE_CODE(sei.auCpbRemovalDelay[i] - 1, bp.cpbRemovalDelayLength, "pt_cpb_removal_delay_minus1[i]");
      }
    }
  }
  WRITE_CODE(sei.picDpbOutputDelay, bp.dpbOutputDelayLength, "pt_dpb_output_delay");
  if( bp.altCpbParamsPresent )
  {
    WRITE_FLAG( sei.cpbAltTimingInfoPresent, "cpb_alt_timing_info_present_flag" );
    if( sei.cpbAltTimingInfoPresent )
    {
      if (bp.bpNalCpbParamsPresent)
      {
        for (int i = (bp.sublayerInitialCpbRemovalDelayPresent ? 0 : bp.bpMaxSubLayers - 1); i <= bp.bpMaxSubLayers - 1; ++i)
        {
          for (int j = 0; j < bp.bpCpbCnt; j++)
          {
            WRITE_CODE(sei.nalCpbAltInitialRemovalDelayDelta[i][j], bp.initialCpbRemovalDelayLength,
                       "nal_cpb_alt_initial_cpb_removal_delay_delta[ i ][ j ]");
            WRITE_CODE(sei.nalCpbAltInitialRemovalOffsetDelta[i][j], bp.initialCpbRemovalDelayLength,
                       "nal_cpb_alt_initial_cpb_removal_offset_delta[ i ][ j ]");
          }
          WRITE_CODE(sei.nalCpbDelayOffset[i], bp.cpbRemovalDelayLength, "nal_cpb_delay_offset[ i ]");
          WRITE_CODE(sei.nalDpbDelayOffset[i], bp.dpbOutputDelayLength, "nal_dpb_delay_offset[ i ]");
        }
      }

      if (bp.bpVclCpbParamsPresent)
      {
        for (int i = (bp.sublayerInitialCpbRemovalDelayPresent ? 0 : bp.bpMaxSubLayers - 1); i <= bp.bpMaxSubLayers - 1; ++i)
        {
          for (int j = 0; j < bp.bpCpbCnt; j++)
          {
            WRITE_CODE(sei.vclCpbAltInitialRemovalDelayDelta[i][j], bp.initialCpbRemovalDelayLength,
                       "vcl_cpb_alt_initial_cpb_removal_delay_delta[ i ][ j ]");
            WRITE_CODE(sei.vclCpbAltInitialRemovalOffsetDelta[i][j], bp.initialCpbRemovalDelayLength,
                       "vcl_cpb_alt_initial_cpb_removal_offset_delta[ i ][ j ]");
          }
          WRITE_CODE(sei.vclCpbDelayOffset[i], bp.cpbRemovalDelayLength, "vcl_cpb_delay_offset[ i ]");
          WRITE_CODE(sei.vclDpbDelayOffset[i], bp.dpbOutputDelayLength,  "vcl_dpb_delay_offset[ i ]");
        }
      }
    }
  }
  if (bp.bpDecodingUnitHrdParamsPresent && bp.decodingUnitDpbDuParamsInPicTimingSeiFlag)
  {
    WRITE_CODE( sei.picDpbOutputDuDelay, bp.dpbOutputDelayDuLength, "pic_dpb_output_du_delay" );
  }
  if( bp.bpDecodingUnitHrdParamsPresent && bp.decodingUnitCpbParamsInPicTimingSeiFlag )
  {
    WRITE_UVLC( sei.numDecodingUnitsMinus1, "num_decoding_units_minus1" );
    if (sei.numDecodingUnitsMinus1 > 0)
    {
      WRITE_FLAG( sei.duCommonCpbRemovalDelayFlag, "du_commmon_cpb_removal_delay_flag" );
      if( sei.duCommonCpbRemovalDelayFlag )
      {
        for( int i = temporalId; i < bp.bpMaxSubLayers - 1; i ++ )
        {
          if( sei.ptSubLayerDelaysPresent[i] )
            WRITE_CODE( sei.duCommonCpbRemovalDelayMinus1[i], bp.duCpbRemovalDelayIncrementLength, "du_common_cpb_removal_delay_increment_minus1[i]" );
        }
      }
      for( int i = 0; i <= sei.numDecodingUnitsMinus1; i ++ )
      {
        WRITE_UVLC( sei.numNalusInDuMinus1[i], "num_nalus_in_du_minus1[i]" );
        if( !sei.duCommonCpbRemovalDelayFlag && i < sei.numDecodingUnitsMinus1 )
        {
          for( int j = temporalId; j < bp.bpMaxSubLayers - 1; j ++ )
          {
            if( sei.ptSubLayerDelaysPresent[j] )
              WRITE_CODE( sei.duCpbRemovalDelayMinus1[i * bp.bpMaxSubLayers + j], bp.duCpbRemovalDelayIncrementLength, "du_cpb_removal_delay_increment_minus1[i][j]" );
          }
        }
      }
    }
  }

  if (bp.additionalConcatenationInfoPresent)
  {
    WRITE_FLAG( sei.delayForConcatenationEnsureFlag, "pt_delay_for_concatenation_ensured_flag" );
  }
  WRITE_CODE( sei.ptDisplayElementalPeriodsMinus1, 8, "pt_display_elemental_periods_minus1" );
}

void SEIWriter::xWriteSEIFrameFieldInfo(const SEIFrameFieldInfo& sei)
{
  WRITE_FLAG( sei.fieldPicFlag ? 1 : 0,                    "ffi_field_pic_flag" );
  if (sei.fieldPicFlag)
  {
    WRITE_FLAG( sei.bottomFieldFlag ? 1 : 0,               "ffi_bottom_field_flag" );
    WRITE_FLAG( sei.pairingIndicatedFlag ? 1 : 0,          "ffi_pairing_indicated_flag" );
    if (sei.pairingIndicatedFlag)
    {
      WRITE_FLAG( sei.pairedWithNextFieldFlag ? 1 : 0,     "ffi_paired_with_next_field_flag" );
    }
  }
  else
  {
    WRITE_FLAG( sei.displayFieldsFromFrameFlag ? 1 : 0,     "ffi_display_fields_from_frame_flag" );
    if (sei.displayFieldsFromFrameFlag)
    {
      WRITE_FLAG( sei.topFieldFirstFlag ? 1 : 0,            "ffi_display_fields_from_frame_flag" );
    }
    WRITE_CODE( sei.displayElementalPeriodsMinus1, 8,       "ffi_display_elemental_periods_minus1" );
  }
  WRITE_CODE( sei.sourceScanType, 2,                        "ffi_source_scan_type" );
  WRITE_FLAG( sei.duplicateFlag ? 1 : 0,                    "ffi_duplicate_flag" );
}

void SEIWriter::xWriteSEIDependentRAPIndication(const SEIDependentRAPIndication& /*sei*/)
{
  // intentionally empty
}

void SEIWriter::xWriteSEIScalableNesting(OutputBitstream& bs, const SEIScalableNesting& sei)
{
  CHECK (sei.nestedSEIs.size()<1, "There must be at lease one SEI message nested in the scalable nesting SEI.")

  WRITE_FLAG(sei.snOlsFlag, "sn_ols_flag");
  WRITE_FLAG(sei.snSubpicFlag, "sn_subpic_flag");
  if (sei.snOlsFlag)
  {
    WRITE_UVLC(sei.snNumOlssMinus1, "sn_num_olss_minus1");
    for (uint32_t i = 0; i <= sei.snNumOlssMinus1; i++)
    {
      WRITE_UVLC(sei.snOlsIdxDeltaMinus1[i], "sn_ols_idx_delta_minus1[i]");
    }
  }
  else
  {
    WRITE_FLAG(sei.snAllLayersFlag, "sn_all_layers_flag");
    if (!sei.snAllLayersFlag)
    {
      WRITE_UVLC(sei.snNumLayersMinus1, "sn_num_layers");
      for (uint32_t i = 1; i <= sei.snNumLayersMinus1; i++)
      {
        WRITE_CODE(sei.snLayerId[i], 6, "sn_layer_id");
      }
    }
  }
  if (sei.snSubpicFlag)
  {
    WRITE_UVLC( sei.snNumSubpics - 1, "sn_num_subpics_minus1");
    CHECK(sei.snSubpicIdLen < 1, "sn_subpic_id_len_minus1 must be >= 0");
    WRITE_UVLC( sei.snSubpicIdLen - 1, "sn_subpic_id_len_minus1");
    for (uint32_t i = 0; i < sei.snNumSubpics; i++)
    {
      WRITE_CODE(sei.snSubpicId[i], sei.snSubpicIdLen, "sn_subpic_id[i]");
    }
  }

  WRITE_UVLC( (uint32_t)sei.nestedSEIs.size() - 1, "sn_num_seis_minus1");

  // byte alignment
  while (m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
  {
    WRITE_FLAG(0, "sn_zero_bit");
  }

  SEIMessages bufferingPeriod = getSeisByType(sei.nestedSEIs, SEI::BUFFERING_PERIOD);
  if (!bufferingPeriod.empty())
  {
    SEIBufferingPeriod *bp = (SEIBufferingPeriod*)bufferingPeriod.front();
    m_nestingHrd.bufferingPeriodSEI = *(bp);
  }

  // write nested SEI messages
  writeSEImessages(bs, sei.nestedSEIs, m_nestingHrd, true, 0);
}

void SEIWriter::xWriteSEIFramePacking(const SEIFramePacking& sei)
{
  WRITE_UVLC( sei.arrangementId,                  "fp_arrangement_id" );
  WRITE_FLAG( sei.arrangementCancelFlag,          "fp_arrangement_cancel_flag" );

  if( sei.arrangementCancelFlag == 0 )
  {
    WRITE_CODE( sei.arrangementType, 7,           "fp_arrangement_type" );

    WRITE_FLAG( sei.quincunxSamplingFlag,         "fp_quincunx_sampling_flag" );
    WRITE_CODE( sei.contentInterpretationType, 6, "fp_content_interpretation_type" );
    WRITE_FLAG( sei.spatialFlippingFlag,          "fp_spatial_flipping_flag" );
    WRITE_FLAG( sei.frame0FlippedFlag,            "fp_frame0_flipped_flag" );
    WRITE_FLAG( sei.fieldViewsFlag,               "fp_field_views_flag" );
    WRITE_FLAG( sei.currentFrameIsFrame0Flag,     "fp_current_frame_is_frame0_flag" );

    WRITE_FLAG( sei.frame0SelfContainedFlag,      "fp_frame0_self_contained_flag" );
    WRITE_FLAG( sei.frame1SelfContainedFlag,      "fp_frame1_self_contained_flag" );

    if(sei.quincunxSamplingFlag == 0 && sei.arrangementType != 5)
    {
      WRITE_CODE( sei.frame0GridPositionX, 4,     "fp_frame0_grid_position_x" );
      WRITE_CODE( sei.frame0GridPositionY, 4,     "fp_frame0_grid_position_y" );
      WRITE_CODE( sei.frame1GridPositionX, 4,     "fp_frame1_grid_position_x" );
      WRITE_CODE( sei.frame1GridPositionY, 4,     "fp_frame1_grid_position_y" );
    }

    WRITE_CODE( sei.arrangementReservedByte, 8,   "fp_frame_packing_arrangement_reserved_byte" );
    WRITE_FLAG( sei.arrangementPersistenceFlag,   "fp_frame_packing_arrangement_persistence_flag" );
  }

  WRITE_FLAG( sei.upsampledAspectRatio,           "fp_upsampled_aspect_ratio" );
}


void SEIWriter::xWriteSEIParameterSetsInclusionIndication(const SEIParameterSetsInclusionIndication& sei)
{
  WRITE_FLAG(sei.selfContainedClvsFlag, "psii_self_contained_clvs_flag");
}

void SEIWriter::xWriteSEIMasteringDisplayColourVolume(const SEIMasteringDisplayColourVolume& sei)
{
  WRITE_CODE( sei.values.primaries[0][0],  16,  "mdcv_display_primaries_x[0]" );
  WRITE_CODE( sei.values.primaries[0][1],  16,  "mdcv_display_primaries_y[0]" );

  WRITE_CODE( sei.values.primaries[1][0],  16,  "mdcv_display_primaries_x[1]" );
  WRITE_CODE( sei.values.primaries[1][1],  16,  "mdcv_display_primaries_y[1]" );

  WRITE_CODE( sei.values.primaries[2][0],  16,  "mdcv_display_primaries_x[2]" );
  WRITE_CODE( sei.values.primaries[2][1],  16,  "mdcv_display_primaries_y[2]" );

  WRITE_CODE( sei.values.whitePoint[0],    16,  "mdcv_white_point_x" );
  WRITE_CODE( sei.values.whitePoint[1],    16,  "mdcv_white_point_y" );

  WRITE_CODE( sei.values.maxLuminance,     32,  "mdcv_max_display_mastering_luminance" );
  WRITE_CODE( sei.values.minLuminance,     32,  "mdcv_min_display_mastering_luminance" );
}

void SEIWriter::xWriteByteAlign()
{
  if( m_pcBitIf->getNumberOfWrittenBits() % 8 != 0)
  {
    WRITE_FLAG( 1, "payload_bit_equal_to_one" );
    while( m_pcBitIf->getNumberOfWrittenBits() % 8 != 0 )
    {
      WRITE_FLAG( 0, "payload_bit_equal_to_zero" );
    }
  }
}

void SEIWriter::xWriteSEIAlternativeTransferCharacteristics(const SEIAlternativeTransferCharacteristics& sei)
{
  WRITE_CODE(sei.preferredTransferCharacteristics, 8, "preferred_transfer_characteristics");
}

void SEIWriter::xWriteSEIEquirectangularProjection(const SEIEquirectangularProjection &sei)
{
  WRITE_FLAG( sei.erpCancelFlag, "erp_cancel_flag" );
  if( !sei.erpCancelFlag )
  {
    WRITE_FLAG( sei.erpPersistenceFlag, "erp_persistence_flag" );
    WRITE_FLAG( sei.erpGuardBandFlag,   "erp_guard_band_flag" );
    WRITE_CODE( 0, 2, "erp_reserved_zero_2bits" );
    if ( sei.erpGuardBandFlag == 1)
    {
      WRITE_CODE( sei.erpGuardBandType,       3, "erp_guard_band_type" );
      WRITE_CODE( sei.erpLeftGuardBandWidth,  8, "erp_left_guard_band_width" );
      WRITE_CODE( sei.erpRightGuardBandWidth, 8, "erp_right_guard_band_width" );
    }
  }
}

void SEIWriter::xWriteSEISphereRotation(const SEISphereRotation &sei)
{
  WRITE_FLAG( sei.sphereRotationCancelFlag, "sphere_rotation_cancel_flag" );
  if( !sei.sphereRotationCancelFlag )
  {
    WRITE_FLAG( sei.sphereRotationPersistenceFlag,    "sphere_rotation_persistence_flag" );
    WRITE_CODE( 0,                                   6, "sphere_rotation_reserved_zero_6bits" );
    WRITE_SCODE(sei.sphereRotationYaw,            32, "sphere_rotation_yaw" );
    WRITE_SCODE(sei.sphereRotationPitch,          32, "sphere_rotation_pitch" );
    WRITE_SCODE(sei.sphereRotationRoll,           32, "sphere_rotation_roll" );
  }
}

void SEIWriter::xWriteSEIOmniViewport(const SEIOmniViewport &sei)
{
  WRITE_CODE( sei.omniViewportId,     10,    "omni_viewport_id" );
  WRITE_FLAG( sei.omniViewportCancelFlag, "omni_viewport_cancel_flag" );
  if ( !sei.omniViewportCancelFlag )
  {
    WRITE_FLAG( sei.omniViewportPersistenceFlag, "omni_viewport_persistence_flag" );
    const uint32_t numRegions = (uint32_t) sei.omniViewportRegions.size();
    WRITE_CODE( numRegions - 1, 4, "omni_viewport_cnt_minus1" );
    for(uint32_t region=0; region<numRegions; region++)
    {
      const SEIOmniViewport::OmniViewport &viewport=sei.omniViewportRegions[region];
      WRITE_SCODE( viewport.azimuthCentre,     32,  "omni_viewport_azimuth_centre"   );
      WRITE_SCODE( viewport.elevationCentre,   32,  "omni_viewport_elevation_centre" );
      WRITE_SCODE( viewport.tiltCentre,        32,  "omni_viewport_tilt_center" );
      WRITE_CODE( viewport.horRange,           32, "omni_viewport_hor_range[i]" );
      WRITE_CODE( viewport.verRange,           32, "omni_viewport_ver_range[i]" );
    }
  }
}

void SEIWriter::xWriteSEIRegionWisePacking(const SEIRegionWisePacking &sei)
{
  WRITE_FLAG( sei.rwpCancelFlag,                                           "rwp_cancel_flag" );
  if(!sei.rwpCancelFlag)
  {
    WRITE_FLAG( sei.rwpPersistenceFlag,                                    "rwp_persistence_flag" );
    WRITE_FLAG( sei.constituentPictureMatchingFlag,                        "rwp_constituent_picture_matching_flag" );
    WRITE_CODE( 0, 5,                                                        "rwp_reserved_zero_5bits" );
    WRITE_CODE( (uint32_t)sei.numPackedRegions,                 8,             "rwp_num_packed_regions" );
    WRITE_CODE( (uint32_t)sei.projPictureWidth,                 32,            "rwp_proj_picture_width" );
    WRITE_CODE( (uint32_t)sei.projPictureHeight,                32,            "rwp_proj_picture_height" );
    WRITE_CODE( (uint32_t)sei.packedPictureWidth,               16,            "rwp_packed_picture_width" );
    WRITE_CODE( (uint32_t)sei.packedPictureHeight,              16,            "rwp_packed_picture_height" );
    for( int i=0; i < sei.numPackedRegions; i++ )
    {
      WRITE_CODE( 0, 4,                                                      "rwp_reserved_zero_4bits" );
      WRITE_CODE( (uint32_t)sei.rwpTransformType[i],            3,             "rwp_tTransform_type" );
      WRITE_FLAG( sei.rwpGuardBandFlag[i],                                 "rwp_guard_band_flag" );
      WRITE_CODE( (uint32_t)sei.projRegionWidth[i],             32,            "rwp_proj_region_width" );
      WRITE_CODE( (uint32_t)sei.projRegionHeight[i],            32,            "rwp_proj_region_height" );
      WRITE_CODE( (uint32_t)sei.rwpProjRegionTop[i],            32,            "rwp_proj_regionTop" );
      WRITE_CODE( (uint32_t)sei.projRegionLeft[i],              32,            "rwp_proj_region_left" );
      WRITE_CODE( (uint32_t)sei.packedRegionWidth[i],           16,            "rwp_packed_region_width" );
      WRITE_CODE( (uint32_t)sei.packedRegionHeight[i],          16,            "rwp_packed_region_height" );
      WRITE_CODE( (uint32_t)sei.packedRegionTop[i],             16,            "rwp_packed_region_top" );
      WRITE_CODE( (uint32_t)sei.packedRegionLeft[i],            16,            "rwp_packed_region_left" );
      if( sei.rwpGuardBandFlag[i] )
      {
        WRITE_CODE( (uint32_t)sei.rwpLeftGuardBandWidth[i],     8,             "rwp_left_guard_band_width");
        WRITE_CODE( (uint32_t)sei.rwpRightGuardBandWidth[i],    8,             "rwp_right_guard_band_width");
        WRITE_CODE( (uint32_t)sei.rwpTopGuardBandHeight[i],     8,             "rwp_top_guard_band_height");
        WRITE_CODE( (uint32_t)sei.rwpBottomGuardBandHeight[i], 8,             "rwp_bottom_guard_band_height");
        WRITE_FLAG( sei.rwpGuardBandNotUsedForPredFlag[i],                 "rwp_guard_band_not_used_forPred_flag" );
        for( int j=0; j < 4; j++ )
        {
          WRITE_CODE( (uint32_t)sei.rwpGuardBandType[i*4 + j],  3,             "rwp_guard_band_type");
          }
        WRITE_CODE( 0, 3,                                                    "rwp_guard_band_reserved_zero_3bits" );
        }
      }
    }
}

void SEIWriter::xWriteSEIGeneralizedCubemapProjection(const SEIGeneralizedCubemapProjection &sei)
{
  WRITE_FLAG( sei.gcmpCancelFlag,                           "gcmp_cancel_flag" );
  if (!sei.gcmpCancelFlag)
  {
    WRITE_FLAG( sei.gcmpPersistenceFlag,                    "gcmp_persistence_flag" );
    WRITE_CODE( sei.gcmpPackingType,                     3, "gcmp_packing_type" );
    WRITE_CODE( sei.gcmpMappingFunctionType,             2, "gcmp_mapping_function_type" );
    int numFace = sei.gcmpPackingType == 4 || sei.gcmpPackingType == 5 ? 5 : 6;
    for (int i = 0; i < numFace; i++)
    {
      WRITE_CODE( sei.gcmpFaceIndex[i],                  3, "gcmp_face_index" );
      WRITE_CODE( sei.gcmpFaceRotation[i],               2, "gcmp_face_rotation" );
      if (sei.gcmpMappingFunctionType == 2)
      {
        WRITE_CODE( sei.gcmpFunctionCoeffU[i],           7, "gcmp_function_coeff_u" );
        WRITE_FLAG( sei.gcmpFunctionUAffectedByVFlag[i],    "gcmp_function_u_affected_by_v_flag" );
        WRITE_CODE( sei.gcmpFunctionCoeffV[i],           7, "gcmp_function_coeff_v" );
        WRITE_FLAG( sei.gcmpFunctionVAffectedByUFlag[i],    "gcmp_function_v_affected_by_u_flag" );
      }
    }
    WRITE_FLAG( sei.gcmpGuardBandFlag,                      "gcmp_guard_band_flag" );
    if (sei.gcmpGuardBandFlag)
    {
      WRITE_CODE( sei.gcmpGuardBandType,                 3, "gcmp_guard_band_type" );
      WRITE_FLAG( sei.gcmpGuardBandBoundaryExteriorFlag,    "gcmp_guard_band_boundary_exterior_flag" );
      WRITE_CODE( sei.gcmpGuardBandSamplesMinus1,        4, "gcmp_guard_band_samples_minus1" );
    }
  }
}

void SEIWriter::xWriteSEISubpictureLevelInfo(const SEISubpicureLevelInfo &sei)
{
  CHECK(sei.numRefLevels < 1, "SEISubpicureLevelInfo: numRefLevels must be greater than zero");
  CHECK(sei.numRefLevels != (int)sei.refLevelIdc.size(), "SEISubpicureLevelInfo: numRefLevels must be equal to the number of levels");
  if (sei.explicitFractionPresent)
  {
    CHECK(sei.numRefLevels != (int)sei.refLevelFraction.size(), "SEISubpicureLevelInfo: numRefLevels must be equal to the number of fractions");
  }
  WRITE_CODE( (uint32_t)sei.numRefLevels - 1, 3,                            "sli_num_ref_levels_minus1");
  WRITE_FLAG(           sei.cbrConstraintFlag,                              "sli_cbr_constraint_flag");
  WRITE_FLAG(           sei.explicitFractionPresent,                    "sli_explicit_fraction_present_flag");
  if (sei.explicitFractionPresent)
  {
    WRITE_UVLC(         sei.numSubpics -1 ,                                 "sli_num_subpics_minus1");
    WRITE_CODE( (uint32_t)sei.sliMaxSublayers - 1, 3,                       "sli_max_sublayers_minus1");
    WRITE_FLAG(           sei.sliSublayerInfoPresent,                   "sli_sublayer_info_present_flag");
    while (!isByteAligned())
    {
      WRITE_FLAG(       0,                                                    "sli_alignment_zero_bit");
    }
  }

  for (int k = sei.sliSublayerInfoPresent ? 0 : sei.sliMaxSublayers - 1; k < sei.sliMaxSublayers; k++)
  {
    for (int i = 0; i < sei.numRefLevels; i++)
    {
      WRITE_CODE((uint32_t)sei.nonSubpicLayersFraction[i][k], 8, "sli_non_subpic_layers_fraction[i][k]");
      WRITE_CODE((uint32_t)sei.refLevelIdc[i][k], 8, "sli_ref_level_idc[i][k]");
      if (sei.explicitFractionPresent)
      {
        CHECK(sei.numSubpics != (int)sei.refLevelFraction[i].size(), "SEISubpicureLevelInfo: number of fractions differs from number of subpictures");
        for (int j = 0; j < sei.numSubpics; j++)
        {
          WRITE_CODE((uint32_t)sei.refLevelFraction[i][j][k], 8, "sli_ref_level_fraction_minus1[i][j][k]");
        }
      }
    }
  }
}

void SEIWriter::xWriteSEISampleAspectRatioInfo(const SEISampleAspectRatioInfo &sei)
{
  WRITE_FLAG( sei.sariCancelFlag,                                           "sari_cancel_flag" );
  if(!sei.sariCancelFlag)
  {
    WRITE_FLAG( sei.sariPersistenceFlag,                                    "sari_persistence_flag" );
    WRITE_CODE( (uint32_t)sei.sariAspectRatioIdc, 8,                        "sari_aspect_ratio_idc");
    if (sei.sariAspectRatioIdc == 255)
    {
      WRITE_CODE( (uint32_t)sei.sariSarWidth, 16,                           "sari_sar_width");
      WRITE_CODE( (uint32_t)sei.sariSarHeight, 16,                           "sari_sar_height");
    }
  }
}

void SEIWriter::xWriteSEIUserDataRegistered(const SEIUserDataRegistered &sei)
{
  WRITE_CODE((sei.ituCountryCode>255) ? 0xff : sei.ituCountryCode, 8, "itu_t_t35_country_code");
  if (sei.ituCountryCode >= 255)
  {
    assert(sei.ituCountryCode < 255 + 256);
    WRITE_CODE(sei.ituCountryCode - 255, 8, "itu_t_t35_country_code_extension_byte");
  }
  for (uint32_t i = 0; i<sei.userData.size(); i++)
  {
    WRITE_CODE(sei.userData[i], 8, "itu_t_t35_payload_byte");
  }
}

void SEIWriter::xWriteSEIFilmGrainCharacteristics(const SEIFilmGrainCharacteristics &sei)
{
  WRITE_FLAG(sei.filmGrainCharacteristicsCancelFlag, "fg_characteristics_cancel_flag");
  if (!sei.filmGrainCharacteristicsCancelFlag)
  {
    WRITE_CODE(sei.filmGrainModelId, 2, "fg_model_id");
    WRITE_FLAG(sei.separateColourDescriptionPresent, "separate_colour_description_present_flag");
    if (sei.separateColourDescriptionPresent)
    {
      WRITE_CODE(sei.filmGrainBitDepthLumaMinus8, 3, "fg_bit_depth_luma_minus8");
      WRITE_CODE(sei.filmGrainBitDepthChromaMinus8, 3, "fg_bit_depth_chroma_minus8");
      WRITE_FLAG(sei.filmGrainFullRangeFlag, "fg_full_range_flag");
      WRITE_CODE(sei.filmGrainColourPrimaries, 8, "fg_colour_primaries");
      WRITE_CODE(sei.filmGrainTransferCharacteristics, 8, "fg_transfer_characteristics");
      WRITE_CODE(sei.filmGrainMatrixCoeffs, 8, "fg_matrix_coeffs");
    }
    WRITE_CODE(sei.blendingModeId, 2, "fg_blending_mode_id");
    WRITE_CODE(sei.log2ScaleFactor, 4, "fg_log2_scale_factor");
    for (int c = 0; c<3; c++)
    {
      const SEIFilmGrainCharacteristics::CompModel &cm = sei.compModel[c];
      const uint32_t numIntensityIntervals = (uint32_t)cm.intensityValues.size();
      const uint32_t numModelValues = cm.numModelValues;
      WRITE_FLAG(sei.compModel[c].presentFlag && numIntensityIntervals>0 && numModelValues>0, "fg_comp_model_present_flag[c]");
    }
    for (uint32_t c = 0; c<3; c++)
    {
      const SEIFilmGrainCharacteristics::CompModel &cm = sei.compModel[c];
      const uint32_t numIntensityIntervals = (uint32_t)cm.intensityValues.size();
      const uint32_t numModelValues = cm.numModelValues;
      if (cm.presentFlag && numIntensityIntervals>0 && numModelValues>0)
      {
        assert(numIntensityIntervals <= 256);
        assert(numModelValues <= 256);
        WRITE_CODE(numIntensityIntervals - 1, 8, "fg_num_intensity_intervals_minus1[c]");
        WRITE_CODE(numModelValues - 1, 3, "fg_num_model_values_minus1[c]");
        for (uint32_t interval = 0; interval<numIntensityIntervals; interval++)
        {
          const SEIFilmGrainCharacteristics::CompModelIntensityValues &cmiv = cm.intensityValues[interval];
          WRITE_CODE(cmiv.intensityIntervalLowerBound, 8, "fg_intensity_interval_lower_bound[c][i]");
          WRITE_CODE(cmiv.intensityIntervalUpperBound, 8, "fg_intensity_interval_upper_bound[c][i]");
          assert(cmiv.compModelValue.size() == numModelValues);
          for (uint32_t j = 0; j<cm.numModelValues; j++)
          {
            WRITE_SVLC(cmiv.compModelValue[j], "fg_comp_model_value[c][i]");
          }
        }
      }
    } // for c
    WRITE_FLAG(sei.filmGrainCharacteristicsPersistenceFlag, "fg_characteristics_persistence_flag");
  } // cancel flag
}

void SEIWriter::xWriteSEIContentLightLevelInfo(const SEIContentLightLevelInfo& sei)
{
  WRITE_CODE( sei.maxContentLightLevel,    16, "clli_max_content_light_level"     );
  WRITE_CODE( sei.maxPicAverageLightLevel, 16, "clli_max_pic_average_light_level" );
}

void SEIWriter::xWriteSEIAmbientViewingEnvironment(const SEIAmbientViewingEnvironment& sei)
{
  WRITE_CODE(sei.ambientIlluminance, 32, "ambient_illuminance" );
  WRITE_CODE(sei.ambientLightX,      16, "ambient_light_x" );
  WRITE_CODE(sei.ambientLightY,      16, "ambient_light_y" );
}

void SEIWriter::xWriteSEIContentColourVolume(const SEIContentColourVolume &sei)
{
  WRITE_FLAG(sei.ccvCancelFlag, "ccv_cancel_flag");
  if (!sei.ccvCancelFlag)
  {
    WRITE_FLAG(sei.ccvPersistenceFlag, "ccv_persistence_flag");
    WRITE_FLAG(sei.ccvPrimariesPresent, "ccv_primaries_present_flag");
    WRITE_FLAG(sei.ccvMinLuminanceValuePresent, "ccv_min_luminance_value_present_flag");
    WRITE_FLAG(sei.ccvMaxLuminanceValuePresent, "ccv_max_luminance_value_present_flag");
    WRITE_FLAG(sei.ccvAvgLuminanceValuePresent, "ccv_avg_luminance_value_present_flag");

    if (sei.ccvPrimariesPresent == true)
    {
      for (int i = 0; i < MAX_NUM_COMP; i++)
      {
        WRITE_SCODE((int32_t)sei.ccvPrimariesX[i], 32, "ccv_primaries_x[i]");
        WRITE_SCODE((int32_t)sei.ccvPrimariesY[i], 32, "ccv_primaries_y[i]");
      }
    }

    if (sei.ccvMinLuminanceValuePresent == true)
    {
      WRITE_CODE((uint32_t)sei.ccvMinLuminanceValue, 32, "ccv_min_luminance_value");
    }
    if (sei.ccvMinLuminanceValuePresent == true)
    {
      WRITE_CODE((uint32_t)sei.ccvMaxLuminanceValue, 32, "ccv_max_luminance_value");
    }
    if (sei.ccvMinLuminanceValuePresent == true)
    {
      WRITE_CODE((uint32_t)sei.ccvAvgLuminanceValue, 32, "ccv_avg_luminance_value");
    }
  }
}

} // namespace vvenc

//! \}


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


/** \file     VLCWriter.cpp
 *  \brief    Writer for high level syntax
 */

#include "VLCWriter.h"
#include "SEIwrite.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Picture.h" // th remove this
#include "CommonLib/dtrace_next.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

#if ENABLE_TRACING

void  VLCWriter::xWriteSCodeTr (int value, uint32_t  length, const char *pSymbolName)
{
  xWriteSCode (value,length);
  if( g_HLSTraceEnable )
  {
    if( length<10 )
    {
      DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d)  : %d\n", pSymbolName, length, value );
    }
    else
    {
      DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d) : %d\n", pSymbolName, length, value );
    }
  }
}

void  VLCWriter::xWriteCodeTr (uint32_t value, uint32_t  length, const char *pSymbolName)
{
  xWriteCode (value,length);

  if( g_HLSTraceEnable )
  {
    if( length < 10 )
    {
      DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d)  : %d\n", pSymbolName, length, value );
    }
    else
    {
      DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d) : %d\n", pSymbolName, length, value );
    }
  }
}

void  VLCWriter::xWriteUvlcTr (uint32_t value, const char *pSymbolName)
{
  xWriteUvlc (value);
  if( g_HLSTraceEnable )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s ue(v) : %d\n", pSymbolName, value );
  }
}

void  VLCWriter::xWriteSvlcTr (int value, const char *pSymbolName)
{
  xWriteSvlc(value);
  if( g_HLSTraceEnable )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s se(v) : %d\n", pSymbolName, value );
  }
}

void  VLCWriter::xWriteFlagTr(bool flag, const char *pSymbolName)
{
  xWriteFlag(flag);
  if( g_HLSTraceEnable )
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(1)  : %d\n", pSymbolName, flag?1:0 );
  }
}

bool g_HLSTraceEnable = true;

#endif

void VLCWriter::xWriteSCode    ( int code, uint32_t length )
{
  assert ( length > 0 && length<=32 );
  assert( length==32 || (code>=-(1<<(length-1)) && code<(1<<(length-1))) );
  m_pcBitIf->write( length==32 ? uint32_t(code) : ( uint32_t(code)&((1<<length)-1) ), length );
}

void VLCWriter::xWriteCode     ( uint32_t uiCode, uint32_t uiLength )
{
  CHECK( uiLength == 0, "Code of length '0' not supported" );
  m_pcBitIf->write( uiCode, uiLength );
}

void VLCWriter::xWriteUvlc     ( uint32_t uiCode )
{
  uint32_t uiLength = 1;
  uint32_t uiTemp = ++uiCode;

  CHECK( !uiTemp, "Integer overflow" );

  while( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  // Take care of cases where uiLength > 32
  m_pcBitIf->write( 0, uiLength >> 1);
  m_pcBitIf->write( uiCode, (uiLength+1) >> 1);
}

void VLCWriter::xWriteSvlc     ( int iCode )
{
  uint32_t uiCode = uint32_t( iCode <= 0 ? (-iCode)<<1 : (iCode<<1)-1);
  xWriteUvlc( uiCode );
}

void VLCWriter::xWriteFlag( bool flag )
{
  m_pcBitIf->write( flag?1:0, 1 );
}

void VLCWriter::xWriteRbspTrailingBits()
{
  WRITE_FLAG( 1, "rbsp_stop_one_bit");
  int cnt = 0;
  while (m_pcBitIf->getNumBitsUntilByteAligned())
  {
    WRITE_FLAG( 0, "rbsp_alignment_zero_bit");
    cnt++;
  }
  CHECK(cnt>=8, "More than '8' alignment bytes read");
}

void HLSWriter::codeAUD(const int audIrapOrGdrAuFlag, const int pictureType)
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Access Unit Delimiter ===========\n" );

  CHECK(pictureType >= 3, "Invalid picture type");
  WRITE_FLAG(audIrapOrGdrAuFlag, "aud_irap_or_gdr_au_flag");
  WRITE_CODE(pictureType, 3, "pic_type");
  xWriteRbspTrailingBits();
}

void HLSWriter::xCodeRefPicList( const ReferencePictureList* rpl, bool isLongTermPresent, uint32_t ltLsbBitsCount, const bool isForbiddenZeroDeltaPoc, int rplIdx )
{
  uint32_t numRefPic = rpl->numberOfShorttermPictures + rpl->numberOfLongtermPictures + rpl->numberOfInterLayerPictures;
  WRITE_UVLC(numRefPic, "num_ref_entries[ listIdx ][ rplsIdx ]");

  if (isLongTermPresent && numRefPic > 0 && rplIdx != -1)
  {
    WRITE_FLAG(rpl->ltrpInSliceHeader, "ltrp_in_slice_header_flag[ listIdx ][ rplsIdx ]");
  }
  int prevDelta = MAX_INT;
  int deltaValue = 0;
  bool firstSTRP = true;
  for (int ii = 0; ii < numRefPic; ii++)
  {
    if( rpl->interLayerPresent )
    {
      WRITE_FLAG( rpl->isInterLayerRefPic[ii], "inter_layer_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]" );

      if( rpl->isInterLayerRefPic[ii] )
      {
        CHECK( rpl->interLayerRefPicIdx[ii] < 0, "Wrong inter-layer reference index" );
        WRITE_UVLC( rpl->interLayerRefPicIdx[ii], "ilrp_idx[ listIdx ][ rplsIdx ][ i ]" );
      }
    }

    if( !rpl->isInterLayerRefPic[ii] )
    {
      if (rpl->numberOfLongtermPictures)
      {
        WRITE_FLAG(!rpl->isLongtermRefPic[ii], "st_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]");
      }
      if (!rpl->isLongtermRefPic[ii])
      {
        if (firstSTRP)
        {
          firstSTRP = false;
          deltaValue = prevDelta = rpl->refPicIdentifier[ii];
        }
        else
        {
          deltaValue = rpl->refPicIdentifier[ii] - prevDelta;
          prevDelta = rpl->refPicIdentifier[ii];
        }
        unsigned int absDeltaValue = (deltaValue < 0) ? 0 - deltaValue : deltaValue;
        if( isForbiddenZeroDeltaPoc || ii == 0 )
        {
          CHECK( !absDeltaValue, "Zero delta POC is not used without WP" );
          WRITE_UVLC( absDeltaValue - 1, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]" );
        }
        else
        WRITE_UVLC(absDeltaValue, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]");
        if (absDeltaValue > 0)
        {
          WRITE_FLAG((deltaValue < 0), "strp_entry_sign_flag[ listIdx ][ rplsIdx ][ i ]");  //0  means negative delta POC : 1 means positive
        }
      }
      else if (!rpl->ltrpInSliceHeader)
      {
        WRITE_CODE(rpl->refPicIdentifier[ii], ltLsbBitsCount, "poc_lsb_lt[listIdx][rplsIdx][i]");
      }
    }
  }
}

void HLSWriter::codePPS( const PPS* pcPPS, const SPS* pcSPS )
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Picture Parameter Set  ===========\n" );

  WRITE_CODE( pcPPS->ppsId, 6,                        "pps_pic_parameter_set_id" );
  WRITE_CODE( pcPPS->spsId, 4,                        "pps_seq_parameter_set_id" );

  WRITE_FLAG( pcPPS->mixedNaluTypesInPic,             "pps_mixed_nalu_types_in_pic_flag" );

  WRITE_UVLC( pcPPS->picWidthInLumaSamples,           "pic_width_in_luma_samples" );
  WRITE_UVLC( pcPPS->picHeightInLumaSamples,          "pic_height_in_luma_samples" );

  if( pcPPS->picWidthInLumaSamples == pcSPS->maxPicWidthInLumaSamples && pcPPS->picHeightInLumaSamples == pcSPS->maxPicHeightInLumaSamples )
  {
    WRITE_FLAG( 0,                                    "pps_conformance_window_flag" );
  }
  else
  {
    const Window& conf = pcPPS->conformanceWindow;
    WRITE_FLAG( conf.enabledFlag,                     "pps_conformance_window_flag" );
    if( conf.enabledFlag )
    {
      WRITE_UVLC( conf.winLeftOffset   / SPS::getWinUnitX(pcSPS->chromaFormatIdc ), "conf_win_left_offset" );
      WRITE_UVLC( conf.winRightOffset  / SPS::getWinUnitX(pcSPS->chromaFormatIdc ), "conf_win_right_offset" );
      WRITE_UVLC( conf.winTopOffset    / SPS::getWinUnitY(pcSPS->chromaFormatIdc ), "conf_win_top_offset" );
      WRITE_UVLC( conf.winBottomOffset / SPS::getWinUnitY(pcSPS->chromaFormatIdc ), "conf_win_bottom_offset" );
    }
  }

  const Window& scWnd = pcPPS->scalingWindow;
  WRITE_FLAG( scWnd.enabledFlag,                      "pps_scaling_window_flag" );
  if( scWnd.enabledFlag )
  {
    WRITE_UVLC( scWnd.winLeftOffset   / SPS::getWinUnitX(pcSPS->chromaFormatIdc ), "pps_scaling_win_left_offset" );
    WRITE_UVLC( scWnd.winRightOffset  / SPS::getWinUnitX(pcSPS->chromaFormatIdc ), "pps_scaling_win_right_offset" );
    WRITE_UVLC( scWnd.winTopOffset    / SPS::getWinUnitY(pcSPS->chromaFormatIdc ), "pps_scaling_win_top_offset" );
    WRITE_UVLC( scWnd.winBottomOffset / SPS::getWinUnitY(pcSPS->chromaFormatIdc ), "pps_scaling_win_bottom_offset" );
  }

  WRITE_FLAG( pcPPS->outputFlagPresent,               "pps_output_flag_present_flag" );
  WRITE_FLAG( pcPPS->noPicPartition,                  "pps_no_pic_partition_flag" );
  WRITE_FLAG( pcPPS->subPicIdMappingInPps,            "pps_subpic_id_mapping_in_pps_flag" );
  if( pcPPS->subPicIdMappingInPps )
  {
    if( pcPPS->noPicPartition )
    {
      WRITE_UVLC( pcPPS->numSubPics - 1,              "pps_num_subpics_minus1" );
    }
    WRITE_UVLC( pcPPS->subPicIdLen - 1,               "pps_subpic_id_len_minus1" );

    CHECK((1 << pcPPS->subPicIdLen) < pcPPS->numSubPics, "pps_subpic_id_len exceeds valid range");
    for( int picIdx = 0; picIdx < pcPPS->numSubPics; picIdx++ )
    {
      WRITE_CODE( pcPPS->subPicId[picIdx], pcPPS->subPicIdLen, "pps_subpic_id[i]" );
    }
  }

  if( !pcPPS->noPicPartition )
  {
    THROW("no suppport");
  }

  WRITE_FLAG( pcPPS->cabacInitPresent,                "pps_cabac_init_present_flag" );
  WRITE_UVLC( pcPPS->numRefIdxL0DefaultActive-1,      "pps_num_ref_idx_l0_default_active_minus1");
  WRITE_UVLC( pcPPS->numRefIdxL1DefaultActive-1,      "pps_num_ref_idx_l1_default_active_minus1");
  WRITE_FLAG( pcPPS->rpl1IdxPresent,                  "pps_rpl1_idx_present_flag");

  WRITE_FLAG( pcPPS->weightPred,                      "pps_weighted_pred_flag" );   // Use of Weighting Prediction (P_SLICE)
  WRITE_FLAG( pcPPS->weightedBiPred,                  "pps_weighted_bipred_flag" );  // Use of Weighting Bi-Prediction (B_SLICE)
  WRITE_FLAG( pcPPS->wrapAroundEnabled,               "pps_ref_wraparound_enabled_flag" );
  if( pcPPS->wrapAroundEnabled )
  {
    WRITE_UVLC(pcPPS->picWidthMinusWrapAroundOffset,  "pps_pic_width_minus_wraparound_offset");
  }

  WRITE_SVLC( pcPPS->picInitQPMinus26,                "pps_init_qp_minus26");
  WRITE_FLAG( pcPPS->useDQP,                          "pps_cu_qp_delta_enabled_flag" );
  WRITE_FLAG (pcPPS->usePPSChromaTool,                "pps_chroma_tool_offsets_present_flag");
  if (pcPPS->usePPSChromaTool)
  {
    WRITE_SVLC( pcPPS->chromaQpOffset[COMP_Cb],       "pps_cb_qp_offset" );
    WRITE_SVLC( pcPPS->chromaQpOffset[COMP_Cr],       "pps_cr_qp_offset" );
    WRITE_FLAG( pcPPS->jointCbCrQpOffsetPresent,      "pps_joint_cbcr_qp_offset_present_flag");
    if (pcPPS->jointCbCrQpOffsetPresent)
    {
      WRITE_SVLC(pcPPS->chromaQpOffset[COMP_JOINT_CbCr],"pps_joint_cbcr_qp_offset_value");
    }

    WRITE_FLAG( pcPPS->sliceChromaQpFlag,               "pps_slice_chroma_qp_offsets_present_flag" );

    bool cuChromaQpOffsetEnabled = pcPPS->chromaQpOffsetListLen>0;
    WRITE_FLAG(cuChromaQpOffsetEnabled,                 "pps_cu_chroma_qp_offset_list_enabled_flag" );
    if( cuChromaQpOffsetEnabled )
    {
      WRITE_UVLC(pcPPS->chromaQpOffsetListLen - 1,      "pps_chroma_qp_offset_list_len_minus1");
      /* skip zero index */
      for (int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx < pcPPS->chromaQpOffsetListLen; cuChromaQpOffsetIdx++)
      {
        WRITE_SVLC(pcPPS->getChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1).u.comp.CbOffset,     "pps_cb_qp_offset_list[i]");
        WRITE_SVLC(pcPPS->getChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1).u.comp.CrOffset,     "pps_cr_qp_offset_list[i]");
        if (pcPPS->jointCbCrQpOffsetPresent)
        {
          WRITE_SVLC(pcPPS->getChromaQpOffsetListEntry(cuChromaQpOffsetIdx + 1).u.comp.JointCbCrOffset, "pps_joint_cbcr_qp_offset_list[i]");
        }
      }
    }
  }
  WRITE_FLAG( pcPPS->deblockingFilterControlPresent,    "debpps_locking_filter_control_present_flag");
  if(pcPPS->deblockingFilterControlPresent)
  {
    WRITE_FLAG( pcPPS->deblockingFilterOverrideEnabled, "pps_deblocking_filter_override_enabled_flag" );
    WRITE_FLAG( pcPPS->deblockingFilterDisabled,        "pps_deblocking_filter_disabled_flag" );
    if (!pcPPS->noPicPartition && pcPPS->deblockingFilterOverrideEnabled)
    {
      WRITE_FLAG(pcPPS->dbfInfoInPh,                    "pps_dbf_info_in_ph_flag");
    }

    if(!pcPPS->deblockingFilterDisabled )
    {
      WRITE_SVLC( pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Y],            "pps_beta_offset_div2" );
      WRITE_SVLC( pcPPS->deblockingFilterTcOffsetDiv2[COMP_Y],              "pps_tc_offset_div2" );
      if( pcPPS->usePPSChromaTool )
      {
        WRITE_SVLC( pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Cb],         "pps_cb_beta_offset_div2" );
        WRITE_SVLC( pcPPS->deblockingFilterTcOffsetDiv2[COMP_Cb],           "pps_cb_tc_offset_div2" );
        WRITE_SVLC( pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Cr],         "pps_cr_beta_offset_div2" );
        WRITE_SVLC( pcPPS->deblockingFilterTcOffsetDiv2[COMP_Cr],           "pps_cr_tc_offset_div2" );
      }
    }
  }
  if ( !pcPPS->noPicPartition )
  {
    WRITE_FLAG(pcPPS->rplInfoInPh,                        "pps_rpl_info_in_ph_flag");
    WRITE_FLAG(pcPPS->saoInfoInPh,                        "pps_sao_info_in_ph_flag");
    WRITE_FLAG(pcPPS->alfInfoInPh,                        "pps_alf_info_in_ph_flag");
    if( (pcPPS->weightPred || pcPPS->weightedBiPred) && pcPPS->rplInfoInPh)
    {
      WRITE_FLAG(pcPPS->wpInfoInPh,                       "pps_wp_info_in_ph_flag");
    }
    WRITE_FLAG(pcPPS->qpDeltaInfoInPh,                    "pps_qp_delta_info_in_ph_flag");
  }

  WRITE_FLAG( pcPPS->pictureHeaderExtensionPresent,       "pps_picture_header_extension_present_flag");
  WRITE_FLAG( pcPPS->sliceHeaderExtensionPresent,         "pps_slice_header_extension_present_flag");

  WRITE_FLAG( false,                                      "pps_extension_present_flag" );

  xWriteRbspTrailingBits();
}

void HLSWriter::codeAPS( const APS* pcAPS )
{
  DTRACE(g_trace_ctx, D_HEADER, "=========== Adaptation Parameter Set  ===========\n");

  WRITE_CODE(pcAPS->apsType, 3,        "aps_params_type");
  WRITE_CODE(pcAPS->apsId, 5,          "adaptation_parameter_set_id");
  WRITE_FLAG(pcAPS->chromaPresent, "aps_chroma_present_flag");

  if (pcAPS->apsType == ALF_APS)
  {
    codeAlfAps(pcAPS);
  }
  else if (pcAPS->apsType == LMCS_APS)
  {
    codeLmcsAps (pcAPS );
  }
  else if( pcAPS->apsType == SCALING_LIST_APS )
  {
    THROW("no support");
  }
  else
  {
    THROW("invalid APS Type");
  }
  WRITE_FLAG(0, "aps_extension_flag");
  xWriteRbspTrailingBits();
}

void HLSWriter::codeAlfAps( const APS* pcAPS )
{
  const AlfParam& param = pcAPS->alfParam;

  WRITE_FLAG(param.newFilterFlag[CH_L],                 "alf_luma_new_filter");
  const CcAlfFilterParam& paramCcAlf = pcAPS->ccAlfParam;
  if (pcAPS->chromaPresent)
  {
    WRITE_FLAG(param.newFilterFlag[CH_C],               "alf_chroma_new_filter");
    WRITE_FLAG(paramCcAlf.newCcAlfFilter[COMP_Cb - 1],  "alf_cc_cb_filter_signal_flag");
    WRITE_FLAG(paramCcAlf.newCcAlfFilter[COMP_Cr - 1],  "alf_cc_cr_filter_signal_flag");
  }

  if (param.newFilterFlag[CH_L])
  {
    WRITE_FLAG( param.nonLinearFlag[CH_L],              "alf_luma_clip" );

    WRITE_UVLC(param.numLumaFilters - 1,                "alf_luma_num_filters_signalled_minus1");
    if (param.numLumaFilters > 1)
    {
      const int len = ceilLog2( param.numLumaFilters);
      for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
      {
        WRITE_CODE(param.filterCoeffDeltaIdx[i], len,   "alf_luma_coeff_delta_idx" );
      }
    }
    alfFilter(param, false, 0);
  }

  if (param.newFilterFlag[CH_C])
  {
    WRITE_FLAG(param.nonLinearFlag[CH_C],               "alf_nonlinear_enable_flag_chroma");
    if( MAX_NUM_ALF_ALTERNATIVES_CHROMA > 1 )
    {
      WRITE_UVLC( param.numAlternativesChroma - 1,      "alf_chroma_num_alts_minus1" );
    }
    for( int altIdx=0; altIdx < param.numAlternativesChroma; ++altIdx )
    {
      alfFilter(param, true, altIdx);
    }
  }

  for (int ccIdx = 0; ccIdx < 2; ccIdx++)
  {
    if (paramCcAlf.newCcAlfFilter[ccIdx])
    {
      const int filterCount = paramCcAlf.ccAlfFilterCount[ccIdx];
      CHECK(filterCount > MAX_NUM_CC_ALF_FILTERS, "CC ALF Filter count is too large");
      CHECK(filterCount == 0,                     "CC ALF Filter count is too small");

      if (MAX_NUM_CC_ALF_FILTERS > 1)
      {
        WRITE_UVLC(filterCount - 1, ccIdx == 0 ? "alf_cc_cb_filters_signalled_minus1" : "alf_cc_cr_filters_signalled_minus1");
      }

      for (int filterIdx = 0; filterIdx < filterCount; filterIdx++)
      {
        AlfFilterShape alfShape(size_CC_ALF);

        const short *coeff = paramCcAlf.ccAlfCoeff[ccIdx][filterIdx];
        // Filter coefficients
        for (int i = 0; i < alfShape.numCoeff - 1; i++)
        {
          if (coeff[i] == 0)
          {
            WRITE_CODE(0, CCALF_BITS_PER_COEFF_LEVEL, ccIdx == 0 ? "alf_cc_cb_mapped_coeff_abs" : "alf_cc_cr_mapped_coeff_abs");
          }
          else
          {
            WRITE_CODE(1 + floorLog2(abs(coeff[i])), CCALF_BITS_PER_COEFF_LEVEL, ccIdx == 0 ? "alf_cc_cb_mapped_coeff_abs" : "alf_cc_cr_mapped_coeff_abs");
            WRITE_FLAG(coeff[i] < 0 ? 1 : 0, ccIdx == 0 ? "alf_cc_cb_coeff_sign" : "alf_cc_cr_coeff_sign");
          }
        }

        DTRACE(g_trace_ctx, D_SYNTAX, "%s coeff filterIdx %d: ", ccIdx == 0 ? "Cb" : "Cr", filterIdx);
        for (int i = 0; i < alfShape.numCoeff; i++)
        {
          DTRACE(g_trace_ctx, D_SYNTAX, "%d ", coeff[i]);
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "\n");
      }
    }
  }
}

void HLSWriter::codeLmcsAps( const APS* aps )
{
  const LmcsParam& param = aps->lmcsParam;
  WRITE_UVLC(param.reshaperModelMinBinIdx,                          "lmcs_min_bin_idx");
  WRITE_UVLC(PIC_CODE_CW_BINS - 1 - param.reshaperModelMaxBinIdx,   "lmcs_delta_max_bin_idx");
  assert(param.maxNbitsNeededDeltaCW > 0);
  WRITE_UVLC(param.maxNbitsNeededDeltaCW - 1,                       "lmcs_delta_cw_prec_minus1");

  for (int i = param.reshaperModelMinBinIdx; i <= param.reshaperModelMaxBinIdx; i++)
  {
    int deltaCW = param.reshaperModelBinCWDelta[i];
    int signCW = (deltaCW < 0) ? 1 : 0;
    int absCW = (deltaCW < 0) ? (-deltaCW) : deltaCW;
    WRITE_CODE(absCW, param.maxNbitsNeededDeltaCW,                  "lmcs_delta_abs_cw[ i ]");
    if (absCW > 0)
    {
      WRITE_FLAG(signCW,                                            "lmcs_delta_sign_cw_flag[ i ]");
    }
  }
  int deltaCRS = aps->chromaPresent ? param.chrResScalingOffset : 0;
  int signCRS = (deltaCRS < 0) ? 1 : 0;
  int absCRS = (deltaCRS < 0) ? (-deltaCRS) : deltaCRS;
  if (aps->chromaPresent)
  {
    WRITE_CODE(absCRS, 3,                                           "lmcs_delta_abs_crs");
  }
  if (absCRS > 0)
  {
    WRITE_FLAG(signCRS,                                             "lmcs_delta_sign_crs_val_flag");
  }
}

void HLSWriter::codeVUI( const VUI *pcVUI, const SPS* pcSPS )
{
#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
#endif

  WRITE_FLAG(pcVUI->progressiveSourceFlag,                "vui_general_progressive_source_flag"         );
  WRITE_FLAG(pcVUI->interlacedSourceFlag,                 "vui_general_interlaced_source_flag"          );
  WRITE_FLAG(pcVUI->nonPackedFlag,                        "vui_non_packed_constraint_flag");
  WRITE_FLAG(pcVUI->nonProjectedFlag,                     "vui_non_projected_constraint_flag");
  WRITE_FLAG(pcVUI->aspectRatioInfoPresent,               "aspect_ratio_info_present_flag");
  if (pcVUI->aspectRatioInfoPresent)
  {
    WRITE_CODE(pcVUI->aspectRatioIdc, 8,                  "aspect_ratio_idc" );
    if (pcVUI->aspectRatioIdc == 255)
    {
      WRITE_CODE(pcVUI->sarWidth, 16,                     "sar_width");
      WRITE_CODE(pcVUI->sarHeight, 16,                    "sar_height");
    }
  }
  WRITE_FLAG(pcVUI->overscanInfoPresent,                  "vui_overscan_info_present_flag");
  if (pcVUI->overscanInfoPresent)
  {
    WRITE_FLAG(pcVUI->overscanAppropriateFlag,            "vui_overscan_appropriate_flag");
  }
  WRITE_FLAG(pcVUI->colourDescriptionPresent,             "colour_description_present_flag");
  if (pcVUI->colourDescriptionPresent)
  {
    WRITE_CODE(pcVUI->colourPrimaries, 8,                 "colour_primaries");
    WRITE_CODE(pcVUI->transferCharacteristics, 8,         "transfer_characteristics");
    WRITE_CODE(pcVUI->matrixCoefficients, 8,              "matrix_coeffs");
    WRITE_FLAG(pcVUI->videoFullRangeFlag,                 "vui_video_full_range_flag");
  }
  WRITE_FLAG(pcVUI->chromaLocInfoPresent,                 "chroma_loc_info_present_flag");
  if (pcVUI->chromaLocInfoPresent)
  {
    if(pcVUI->progressiveSourceFlag && !pcVUI->interlacedSourceFlag)
    {
      WRITE_UVLC(pcVUI->chromaSampleLocType,              "chroma_sample_loc_type");
    }
    else
    {
      WRITE_UVLC(pcVUI->chromaSampleLocTypeTopField,      "chroma_sample_loc_type_top_field");
      WRITE_UVLC(pcVUI->chromaSampleLocTypeBottomField,   "chroma_sample_loc_type_bottom_field");
    }
  }

  if(!isByteAligned())
  {
    WRITE_FLAG(1,   "vui_payload_bit_equal_to_one");
    while(!isByteAligned())
    {
      WRITE_FLAG(0, "vui_payload_bit_equal_to_zero");
    }
  }
}

void HLSWriter::codeGeneralHrdparameters(const GeneralHrdParams * hrd)
{
  WRITE_CODE(hrd->numUnitsInTick, 32,                   "num_units_in_tick");
  WRITE_CODE(hrd->timeScale, 32,                        "time_scale");
  WRITE_FLAG(hrd->generalNalHrdParamsPresent,           "general_nal_hrd_parameters_present_flag");
  WRITE_FLAG(hrd->generalVclHrdParamsPresent,           "general_vcl_hrd_parameters_present_flag");
  WRITE_FLAG(hrd->generalSamePicTimingInAllOlsFlag,     "general_same_pic_timing_in_all_ols_flag");
  WRITE_FLAG(hrd->generalDecodingUnitHrdParamsPresent,  "general_decoding_unit_hrd_params_present_flag");
  if (hrd->generalDecodingUnitHrdParamsPresent)
  {
    WRITE_CODE(hrd->tickDivisorMinus2, 8,               "tick_divisor_minus2");
  }
  WRITE_CODE(hrd->bitRateScale, 4,                      "bit_rate_scale");
  WRITE_CODE(hrd->cpbSizeScale, 4,                      "cpb_size_scale");
  if (hrd->generalDecodingUnitHrdParamsPresent)
  {
    WRITE_CODE(hrd->cpbSizeDuScale, 4,                  "cpb_size_du_scale");
  }
  WRITE_UVLC(hrd->hrdCpbCntMinus1,                      "hrd_cpb_cnt_minus1");
}

void HLSWriter::codeOlsHrdParameters(const GeneralHrdParams * generalHrd, const OlsHrdParams *olsHrd, const uint32_t firstSubLayer, const uint32_t maxNumSubLayersMinus1)
{
  for( int i = firstSubLayer; i <= maxNumSubLayersMinus1; i ++ )
  {
    const OlsHrdParams *hrd = &(olsHrd[i]);
    WRITE_FLAG(hrd->fixedPicRateGeneralFlag,      "fixed_pic_rate_general_flag");

    if (!hrd->fixedPicRateGeneralFlag)
    {
      WRITE_FLAG(hrd->fixedPicRateWithinCvsFlag,  "fixed_pic_rate_within_cvs_flag");
    }
    if (hrd->fixedPicRateWithinCvsFlag)
    {
      WRITE_UVLC(hrd->elementDurationInTcMinus1,  "elemental_duration_in_tc_minus1");
    }
    else if (generalHrd->hrdCpbCntMinus1 == 0)
    {
      WRITE_FLAG(hrd->lowDelayHrdFlag,            "low_delay_hrd_flag");
    }

    for( int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if (((nalOrVcl == 0) && (generalHrd->generalNalHrdParamsPresent)) || ((nalOrVcl == 1) && (generalHrd->generalVclHrdParamsPresent)))
      {
        for (int j = 0; j <= (generalHrd->hrdCpbCntMinus1); j++)
        {
          WRITE_UVLC(hrd->bitRateValueMinus1[j][nalOrVcl], "bit_rate_value_minus1");
          WRITE_UVLC(hrd->cpbSizeValueMinus1[j][nalOrVcl], "cpb_size_value_minus1");
          if (generalHrd->generalDecodingUnitHrdParamsPresent)
          {
            WRITE_UVLC(hrd->duCpbSizeValueMinus1[j][nalOrVcl], "cpb_size_du_value_minus1");
            WRITE_UVLC(hrd->duBitRateValueMinus1[j][nalOrVcl], "bit_rate_du_value_minus1");
          }
          WRITE_FLAG(hrd->cbrFlag[j][nalOrVcl], "cbr_flag");
        }
      }
    }
  }
}

void HLSWriter::dpb_parameters(int maxSubLayersMinus1, bool subLayerInfoFlag, const SPS *pcSPS)
{
  for (uint32_t i = (subLayerInfoFlag ? 0 : maxSubLayersMinus1); i <= maxSubLayersMinus1; i++)
  {
    WRITE_UVLC(pcSPS->maxDecPicBuffering[i] - 1,          "dpb_max_dec_pic_buffering_minus1[i]");
    WRITE_UVLC(pcSPS->numReorderPics[i],                  "dpb_max_num_reorder_pics[i]");
    WRITE_UVLC(pcSPS->maxLatencyIncreasePlus1[i],         "dpb_max_latency_increase_plus1[i]");
  }
}

void HLSWriter::codeSPS( const SPS* pcSPS )
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Sequence Parameter Set  ===========\n" );

  WRITE_CODE( pcSPS->spsId, 4,                            "sps_seq_parameter_set_id" );
  WRITE_CODE( pcSPS->vpsId, 4,                            "sps_video_parameter_set_id" );
  CHECK(pcSPS->maxTLayers == 0, "Maximum number of temporal sub-layers is '0'");

  WRITE_CODE(pcSPS->maxTLayers - 1, 3,                    "sps_max_sub_layers_minus1");
  WRITE_CODE( int(pcSPS->chromaFormatIdc), 2,             "sps_chroma_format_idc" );
  WRITE_CODE(floorLog2(pcSPS->CTUSize) - 5, 2,            "sps_log2_ctu_size_minus5");
  WRITE_FLAG(pcSPS->ptlDpbHrdParamsPresent,               "sps_ptl_dpb_hrd_params_present_flag");

  if (pcSPS->ptlDpbHrdParamsPresent)
  {
    codeProfileTierLevel( &pcSPS->profileTierLevel, true, pcSPS->maxTLayers - 1 );
  }

  WRITE_FLAG(pcSPS->GDR,                                  "sps_gdr_enabled_flag");
  WRITE_FLAG( pcSPS->rprEnabled,                          "sps_ref_pic_resampling_enabled_flag" );
  if( pcSPS->rprEnabled )
  {
    WRITE_FLAG(pcSPS->resChangeInClvsEnabled, "sps_res_change_in_clvs_allowed_flag");
  }
  WRITE_UVLC( pcSPS->maxPicWidthInLumaSamples,            "sps_pic_width_max_in_luma_samples" );
  WRITE_UVLC( pcSPS->maxPicHeightInLumaSamples,           "sps_pic_height_max_in_luma_samples" );

  const Window& conf = pcSPS->conformanceWindow;
  WRITE_FLAG( conf.enabledFlag,                           "sps_conformance_window_flag" );
  if (conf.enabledFlag)
  {
    WRITE_UVLC( conf.winLeftOffset   / SPS::getWinUnitX(pcSPS->chromaFormatIdc ), "sps_conf_win_left_offset" );
    WRITE_UVLC( conf.winRightOffset  / SPS::getWinUnitX(pcSPS->chromaFormatIdc ), "sps_conf_win_right_offset" );
    WRITE_UVLC( conf.winTopOffset    / SPS::getWinUnitY(pcSPS->chromaFormatIdc ), "sps_conf_win_top_offset" );
    WRITE_UVLC( conf.winBottomOffset / SPS::getWinUnitY(pcSPS->chromaFormatIdc ), "sps_conf_win_bottom_offset" );
  }

  WRITE_FLAG(pcSPS->subPicInfoPresent,                    "sps_subpic_info_present_flag");

  if (pcSPS->subPicInfoPresent)
  {
    THROW("no suppport");
  }

  WRITE_UVLC( pcSPS->bitDepths[ CH_L ] - 8,               "sps_bitdepth_minus8" );
  WRITE_FLAG( pcSPS->entropyCodingSyncEnabled,            "sps_entropy_coding_sync_enabled_flag" );
  WRITE_FLAG( pcSPS->entryPointsPresent,                  "sps_entry_point_offsets_present_flag" );
  WRITE_CODE( pcSPS->bitsForPOC-4, 4,                     "sps_log2_max_pic_order_cnt_lsb_minus4" );
  WRITE_FLAG( pcSPS->pocMsbFlag,                          "sps_poc_msb_flag");

  if (pcSPS->pocMsbFlag)
  {
    WRITE_UVLC(pcSPS->pocMsbLen - 1,                      "sps_poc_msb_len_minus1");
  }

  WRITE_CODE(0, 2,                                        "sps_num_extra_ph_bits_bytes");
  WRITE_CODE(0, 2,                                        "sps_num_extra_sh_bits_bytes");

  if (pcSPS->ptlDpbHrdParamsPresent)
  {
    if (pcSPS->maxTLayers > 1)
    {
      WRITE_FLAG(pcSPS->subLayerDpbParams,                "sps_sublayer_dpb_params_flag");
    }
    dpb_parameters(pcSPS->maxTLayers - 1, pcSPS->subLayerDpbParams, pcSPS);
  }

  WRITE_UVLC(pcSPS->log2MinCodingBlockSize - 2,                           "log2_min_luma_coding_block_size_minus2");
  WRITE_FLAG(pcSPS->partitionOverrideEnabled,                             "sps_partition_constraints_override_enabled_flag");
  WRITE_UVLC(Log2(pcSPS->minQTSize[0]) - pcSPS->log2MinCodingBlockSize,   "sps_log2_diff_min_qt_min_cb_intra_slice_luma");
  WRITE_UVLC(pcSPS->maxMTTDepth[0],                                       "sps_max_mtt_hierarchy_depth_intra_slice_luma");
  if (pcSPS->maxMTTDepth[0] != 0)
  {
    WRITE_UVLC(Log2(pcSPS->maxBTSize[0]) - Log2(pcSPS->minQTSize[0]),     "sps_log2_diff_max_bt_min_qt_intra_slice_luma");
    WRITE_UVLC(Log2(pcSPS->maxTTSize[0]) - Log2(pcSPS->minQTSize[0]),     "sps_log2_diff_max_tt_min_qt_intra_slice_luma");
  }
  if( pcSPS->chromaFormatIdc != CHROMA_400 )
  {
    WRITE_FLAG(pcSPS->dualITree,                                          "sps_qtbtt_dual_tree_intra_flag");
  }
  if (pcSPS->dualITree)
  {
    WRITE_UVLC(Log2(pcSPS->minQTSize[2]) - pcSPS->log2MinCodingBlockSize, "sps_log2_diff_min_qt_min_cb_intra_slice_chroma");
    WRITE_UVLC(pcSPS->maxMTTDepth[2],                                     "sps_max_mtt_hierarchy_depth_intra_slice_chroma");
    if (pcSPS->maxMTTDepth[2] != 0)
    {
      WRITE_UVLC(Log2(pcSPS->maxBTSize[2]) - Log2(pcSPS->minQTSize[2]),   "sps_log2_diff_max_bt_min_qt_intra_slice_chroma");
      WRITE_UVLC(Log2(pcSPS->maxTTSize[2]) - Log2(pcSPS->minQTSize[2]),   "sps_log2_diff_max_tt_min_qt_intra_slice_chroma");
    }
  }

  WRITE_UVLC(Log2(pcSPS->minQTSize[1]) - pcSPS->log2MinCodingBlockSize,   "sps_log2_diff_min_qt_min_cb_inter_slice");
  WRITE_UVLC(pcSPS->maxMTTDepth[1],                                       "sps_max_mtt_hierarchy_depth_inter_slice");
  if (pcSPS->maxMTTDepth[1] != 0)
  {
    WRITE_UVLC(Log2(pcSPS->maxBTSize[1]) - Log2(pcSPS->minQTSize[1]),     "sps_log2_diff_max_bt_min_qt_inter_slice");
    WRITE_UVLC(Log2(pcSPS->maxTTSize[1]) - Log2(pcSPS->minQTSize[1]),     "sps_log2_diff_max_tt_min_qt_inter_slice");
  }

  if (pcSPS->CTUSize > 32)
  {
    WRITE_FLAG( (pcSPS->log2MaxTbSize - 5) != 0,                          "sps_max_luma_transform_size_64_flag" );
  }
  WRITE_FLAG(pcSPS->transformSkip,                                        "sps_transform_skip_enabled_flag");
  if (pcSPS->transformSkip)
  {
    WRITE_UVLC(pcSPS->log2MaxTransformSkipBlockSize - 2,                  "sps_log2_transform_skip_max_size_minus2");
    WRITE_FLAG(pcSPS->BDPCM,                                              "sps_bdpcm_enabled_flag");
  }
  WRITE_FLAG( pcSPS->MTS,                                                 "sps_mts_enabled_flag" );
  if ( pcSPS->MTS )
  {
    WRITE_FLAG( pcSPS->MTSIntra,                                          "sps_explicit_mts_intra_enabled_flag" );
    WRITE_FLAG( pcSPS->MTSInter,                                          "sps_explicit_mts_inter_enabled_flag" );
  }
  WRITE_FLAG( pcSPS->LFNST,                                               "sps_lfnst_enabled_flag");

  if (pcSPS->chromaFormatIdc != CHROMA_400)
  {
    WRITE_FLAG(pcSPS->jointCbCr,                                          "sps_joint_cbcr_enabled_flag");

    const ChromaQpMappingTable& chromaQpMappingTable = pcSPS->chromaQpMappingTable;
    WRITE_FLAG(chromaQpMappingTable.m_sameCQPTableForAllChromaFlag,       "same_qp_table_for_chroma");
    int numQpTables = chromaQpMappingTable.m_sameCQPTableForAllChromaFlag ? 1 : (pcSPS->jointCbCr ? 3 : 2);
    CHECK(numQpTables != chromaQpMappingTable.m_numQpTables, " numQpTables does not match at encoder side ");
    for (int i = 0; i < numQpTables; i++)
    {
      WRITE_SVLC(chromaQpMappingTable.m_qpTableStartMinus26[i],           "sps_qp_table_starts_minus26");
      WRITE_UVLC(chromaQpMappingTable.m_numPtsInCQPTableMinus1[i],        "sps_num_points_in_qp_table_minus1");

      for (int j = 0; j <= chromaQpMappingTable.m_numPtsInCQPTableMinus1[i]; j++)
      {
        WRITE_UVLC(chromaQpMappingTable.m_deltaQpInValMinus1[i][j],       "sps_delta_qp_in_val_minus1");
        WRITE_UVLC(chromaQpMappingTable.m_deltaQpOutVal[i][j] ^ chromaQpMappingTable.m_deltaQpInValMinus1[i][j], "sps_delta_qp_diff_val");
      }
    }
  }

  WRITE_FLAG( pcSPS->saoEnabled,                          "sps_sao_enabled_flag");
  WRITE_FLAG( pcSPS->alfEnabled,                          "sps_alf_enabled_flag" );
  if (pcSPS->alfEnabled && pcSPS->chromaFormatIdc != CHROMA_400)
  {
    WRITE_FLAG( pcSPS->ccalfEnabled,                      "sps_ccalf_enabled_flag" );
  }
  WRITE_FLAG(pcSPS->lumaReshapeEnable,                    "sps_lmcs_enable_flag");
  WRITE_FLAG( pcSPS->weightPred,                          "sps_weighted_pred_flag" );   // Use of Weighting Prediction (P_SLICE)
  WRITE_FLAG( pcSPS->weightedBiPred,                      "sps_weighted_bipred_flag" );  // Use of Weighting Bi-Prediction (B_SLICE)
  WRITE_FLAG( pcSPS->longTermRefsPresent,                 "sps_long_term_ref_pics_flag" );
  if( pcSPS->vpsId > 0 )
  {
    WRITE_FLAG( pcSPS->interLayerPresent,                 "sps_inter_layer_ref_pics_present_flag" );
  }
  WRITE_FLAG( pcSPS->idrRefParamList,                     "sps_idr_rpl_present_flag" );
  WRITE_FLAG( pcSPS->rpl1CopyFromRpl0,                    "sps_rpl1_copy_from_rpl0_flag");

  //Write candidate for List0
  uint32_t numberOfRPL = (uint32_t)pcSPS->getNumRPL(0);
  WRITE_UVLC(numberOfRPL,                                 "sps_num_ref_pic_lists_in_sps[0]");
  for (int ii = 0; ii < numberOfRPL; ii++)
  {
    xCodeRefPicList( &pcSPS->rplList[0][ii], pcSPS->longTermRefsPresent, pcSPS->bitsForPOC, !pcSPS->weightPred && !pcSPS->weightedBiPred, ii );
  }

  //Write candidate for List1
  if (!pcSPS->rpl1CopyFromRpl0)
  {
    numberOfRPL = (uint32_t)pcSPS->getNumRPL(1);
    WRITE_UVLC(numberOfRPL,                               "sps_num_ref_pic_lists_in_sps[1]");
    for (int ii = 0; ii < numberOfRPL; ii++)
    {
      xCodeRefPicList( &pcSPS->rplList[1][ii], pcSPS->longTermRefsPresent, pcSPS->bitsForPOC, !pcSPS->weightPred && !pcSPS->weightedBiPred, ii );
    }
  }

  WRITE_FLAG( pcSPS->wrapAroundEnabled,                   "sps_ref_wraparound_enabled_flag" );

  WRITE_FLAG( pcSPS->temporalMVPEnabled,                  "sps_temporal_mvp_enabled_flag" );

  if ( pcSPS->temporalMVPEnabled )
  {
    WRITE_FLAG( pcSPS->SbtMvp,                            "sps_sbtmvp_enabled_flag");
  }

  WRITE_FLAG( pcSPS->AMVR,                                "sps_amvr_enabled_flag" );

  WRITE_FLAG( pcSPS->BDOF,                                "sps_bdof_enabled_flag" );
  if (pcSPS->BDOF)
  {
    WRITE_FLAG(pcSPS->BdofPresent,                        "sps_bdof_pic_present_flag");
  }
  WRITE_FLAG( pcSPS->SMVD,                                "sps_smvd_enabled_flag" );
  WRITE_FLAG( pcSPS->DMVR,                                "sps_dmvr_enabled_flag" );
  if (pcSPS->DMVR)
  {
    WRITE_FLAG(pcSPS->DmvrPresent,                        "sps_dmvr_pic_present_flag");
  }
  WRITE_FLAG(pcSPS->MMVD,                                 "sps_mmvd_enabled_flag");
  if ( pcSPS->MMVD )
  {
    WRITE_FLAG( pcSPS->fpelMmvd,                          "sps_fpel_mmvd_enabled_flag" );
  }
  WRITE_UVLC(MRG_MAX_NUM_CANDS - pcSPS->maxNumMergeCand,  "sps_six_minus_max_num_merge_cand");
  WRITE_FLAG( pcSPS->SBT,                                 "sps_sbt_enabled_flag" );
  WRITE_FLAG( pcSPS->Affine,                              "sps_affine_enabled_flag" );
  if ( pcSPS->Affine )
  {
    WRITE_UVLC(AFFINE_MRG_MAX_NUM_CANDS - pcSPS->maxNumAffineMergeCand, "five_minus_max_num_subblock_merge_cand");
    WRITE_FLAG( pcSPS->AffineType,                        "sps_affine_type_flag" );
    if (pcSPS->AMVR )
    {
      WRITE_FLAG( pcSPS->AffineAmvr,                      "sps_affine_amvr_enabled_flag" );
    }

    WRITE_FLAG( pcSPS->PROF,                              "sps_affine_prof_enabled_flag" );
    if (pcSPS->PROF)
    {
      WRITE_FLAG(pcSPS->ProfPresent,                      "sps_prof_pic_present_flag" );
    }
  }

  WRITE_FLAG(pcSPS->BCW,                                  "sps_bcw_enabled_flag");

  WRITE_FLAG( pcSPS->CIIP,                                "sps_ciip_enabled_flag" );

  if (pcSPS->maxNumMergeCand >= 2)
  {
    WRITE_FLAG(pcSPS->GEO,                                "sps_gpm_enabled_flag");
    if (pcSPS->GEO && pcSPS->maxNumMergeCand >= 3)
    {
      WRITE_UVLC(pcSPS->maxNumMergeCand - pcSPS->maxNumGeoCand,   "sps_max_num_merge_cand_minus_max_num_gpm_cand");
    }
  }

  WRITE_UVLC(pcSPS->log2ParallelMergeLevelMinus2,         "sps_log2_parallel_merge_level_minus2");
  WRITE_FLAG( pcSPS->ISP,                                 "sps_isp_enabled_flag");
  WRITE_FLAG( pcSPS->MRL,                                 "sps_mrl_enabled_flag");
  WRITE_FLAG( pcSPS->MIP,                                 "sps_mip_enabled_flag");
  if( pcSPS->chromaFormatIdc != CHROMA_400)
  {
    WRITE_FLAG( pcSPS->LMChroma,                          "sps_cclm_enabled_flag" );
  }
  if ( pcSPS->chromaFormatIdc == CHROMA_420 )
  {
    WRITE_FLAG( pcSPS->horCollocatedChroma,               "sps_chroma_horizontal_collocated_flag" );
    WRITE_FLAG( pcSPS->verCollocatedChroma,               "sps_chroma_vertical_collocated_flag" );
  }

  WRITE_FLAG(pcSPS->PLT,                                  "sps_palette_enabled_flag" );

  if (pcSPS->chromaFormatIdc == CHROMA_444)
  {
    WRITE_FLAG(pcSPS->PLT,                                "sps_plt_enabled_flag" );
  }
  if (pcSPS->chromaFormatIdc == CHROMA_444 && pcSPS->log2MaxTbSize != 6)
  {
    WRITE_FLAG(pcSPS->useColorTrans,                      "sps_act_enabled_flag");
  }
  if (pcSPS->transformSkip || pcSPS->PLT)
  {
    WRITE_UVLC(pcSPS->internalMinusInputBitDepth[CH_L],   "sps_internal_bit_depth_minus_input_bit_depth");
  }

  WRITE_FLAG(pcSPS->IBC,                                  "sps_ibc_enabled_flag");
  if( pcSPS->IBC )
  {
    WRITE_UVLC(IBC_MRG_MAX_NUM_CANDS - pcSPS->maxNumIBCMergeCand, "six_minus_max_num_ibc_merge_cand");
  }

  WRITE_FLAG( pcSPS->LADF,                                "sps_ladf_enabled_flag" );
  if ( pcSPS->LADF )
  {
    THROW("no support");
  }

  WRITE_FLAG( pcSPS->scalingListEnabled,                  "sps_explicit_scaling_list_enabled_flag" );
  if (pcSPS->LFNST && pcSPS->scalingListEnabled )
  {
    WRITE_FLAG(pcSPS->disableScalingMatrixForLfnstBlks,   "sps_scaling_matrix_for_lfnst_disabled_flag");
  }
  if (pcSPS->useColorTrans && pcSPS->scalingListEnabled)
  {
    WRITE_FLAG(pcSPS->scalingMatrixAlternativeColourSpaceDisabled, "sps_scaling_matrix_for_alternative_colour_space_disabled_flag");
  }
  if (pcSPS->scalingMatrixAlternativeColourSpaceDisabled)
  {
    WRITE_FLAG(pcSPS->scalingMatrixDesignatedColourSpace, "sps_scaling_matrix_designated_colour_space_flag");
  }
  WRITE_FLAG(pcSPS->depQuantEnabled,                      "sps_dep_quant_enabled_flag");
  WRITE_FLAG(pcSPS->signDataHidingEnabled,                "sps_sign_data_hiding_enabled_flag");

  WRITE_FLAG( pcSPS->virtualBoundariesEnabled,            "sps_virtual_boundaries_enabled_flag" );
  if( pcSPS->virtualBoundariesEnabled )
  {
    WRITE_CODE( pcSPS->numVerVirtualBoundaries, 2,        "sps_num_ver_virtual_boundaries");
    for( unsigned i = 0; i < pcSPS->numVerVirtualBoundaries; i++ )
    {
      WRITE_UVLC((pcSPS->virtualBoundariesPosX[i]>>3),    "sps_virtual_boundaries_pos_x");
    }
    WRITE_CODE(pcSPS->numHorVirtualBoundaries, 2,         "sps_num_hor_virtual_boundaries");
    for( unsigned i = 0; i < pcSPS->numHorVirtualBoundaries; i++ )
    {
      WRITE_UVLC((pcSPS->virtualBoundariesPosY[i]>>3),    "sps_virtual_boundaries_pos_y");
    }
  }

  if (pcSPS->ptlDpbHrdParamsPresent)
  {
    WRITE_FLAG(pcSPS->hrdParametersPresent,               "sps_general_hrd_params_present_flag");

    if( pcSPS->hrdParametersPresent )
    {
    codeGeneralHrdparameters(&pcSPS->generalHrdParams);
    if ((pcSPS->maxTLayers - 1) > 0)
    {
      WRITE_FLAG(pcSPS->subLayerParametersPresent, "sps_sublayer_cpb_params_present_flag");
    }
    uint32_t firstSubLayer = pcSPS->subLayerParametersPresent ? 0 : (pcSPS->maxTLayers - 1);
    codeOlsHrdParameters(&pcSPS->generalHrdParams, pcSPS->olsHrdParams, firstSubLayer, pcSPS->maxTLayers - 1);
    }
  }

  WRITE_FLAG(pcSPS->fieldSeqFlag,                         "sps_field_seq_flag");

  WRITE_FLAG( pcSPS->vuiParametersPresent,                "sps_vui_parameters_present_flag" );
  if (pcSPS->vuiParametersPresent)
  {
    OutputBitstream *bs = m_pcBitIf; // save the original ono
    OutputBitstream bs_count;
    setBitstream(&bs_count);
#if ENABLE_TRACING
    bool traceEnable = g_HLSTraceEnable;
    g_HLSTraceEnable = false;
#endif
    codeVUI(&pcSPS->vuiParameters, pcSPS);
#if ENABLE_TRACING
    g_HLSTraceEnable = traceEnable;
#endif
    unsigned vui_payload_data_num_bits = bs_count.getNumberOfWrittenBits();
    CHECK( vui_payload_data_num_bits % 8 != 0, "Invalid number of VUI payload data bits" );
    setBitstream(bs);
    WRITE_UVLC((vui_payload_data_num_bits >> 3) - 1,        "sps_vui_payload_size_minus1");
    while (!isByteAligned())
    {
      WRITE_FLAG(0,                                         "sps_vui_alignment_zero_bit");
    }
    codeVUI(&pcSPS->vuiParameters, pcSPS);
  }

  bool sps_extension_present_flag=false;
  bool sps_extension_flags[NUM_SPS_EXTENSION_FLAGS]={false};

  sps_extension_flags[SPS_EXT__REXT] = pcSPS->spsRExt.settingsDifferFromDefaults();

  for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
  {
    sps_extension_present_flag|=sps_extension_flags[i];
  }

  WRITE_FLAG( sps_extension_present_flag,                   "sps_extension_present_flag" );

  if (sps_extension_present_flag)
  {
#if ENABLE_TRACING
    static const char *syntaxStrings[]={ "sps_range_extension_flag",
      "sps_multilayer_extension_flag",
      "sps_extension_6bits[0]",
      "sps_extension_6bits[1]",
      "sps_extension_6bits[2]",
      "sps_extension_6bits[3]",
      "sps_extension_6bits[4]",
      "sps_extension_6bits[5]" };
#endif

    for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
    {
      WRITE_FLAG( sps_extension_flags[i], syntaxStrings[i] );
    }

    for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (sps_extension_flags[i])
      {
        switch (SPSExtensionFlagIndex(i))
        {
        case SPS_EXT__REXT:
        {
          const SPSRExt &spsRangeExtension=pcSPS->spsRExt;
          WRITE_FLAG( spsRangeExtension.transformSkipRotationEnabled,        "transform_skip_rotation_enabled_flag");
          WRITE_FLAG( spsRangeExtension.transformSkipContextEnabled,         "transform_skip_context_enabled_flag");
          WRITE_FLAG( spsRangeExtension.extendedPrecisionProcessing,         "extended_precision_processing_flag" );
          WRITE_FLAG( spsRangeExtension.intraSmoothingDisabled,              "intra_smoothing_disabled_flag" );
          WRITE_FLAG( spsRangeExtension.highPrecisionOffsetsEnabled,         "high_precision_offsets_enabled_flag" );
          WRITE_FLAG( spsRangeExtension.persistentRiceAdaptationEnabled,     "persistent_rice_adaptation_enabled_flag" );
          WRITE_FLAG( spsRangeExtension.cabacBypassAlignmentEnabled,         "cabac_bypass_alignment_enabled_flag" );
          break;
        }
        default:
          CHECK(sps_extension_flags[i]!=false, "Unknown PPS extension signalled"); // Should never get here with an active SPS extension flag.
          break;
        }
      }
    }
  }
  xWriteRbspTrailingBits();
}

void HLSWriter::codeDCI( const DCI* dci )
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Decoding Parameter Set     ===========\n" );

  WRITE_CODE( 0,                                    4,        "dci_reserved_zero_5bits" );
  uint32_t numPTLs = (uint32_t) dci->profileTierLevel.size();
  CHECK (numPTLs<1, "At least one PTL must be available in DPS");

  WRITE_CODE( numPTLs - 1,                          4,        "dci_num_ptls_minus1" );

  for (int i=0; i< numPTLs; i++)
  {
    codeProfileTierLevel( &dci->profileTierLevel[i], true, 0 );
  }
  WRITE_FLAG( 0,                                              "dci_extension_flag" );
  xWriteRbspTrailingBits();
}

void HLSWriter::codeVPS(const VPS* pcVPS)
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Video Parameter Set     ===========\n" );

  WRITE_CODE(pcVPS->vpsId,              4,              "vps_video_parameter_set_id");
  WRITE_CODE(pcVPS->maxLayers - 1,      6,              "vps_max_layers_minus1");
  WRITE_CODE(pcVPS->maxSubLayers - 1,   3,              "vps_max_sublayers_minus1");
  if (pcVPS->maxLayers > 1 && pcVPS->maxSubLayers > 1)
  {
    WRITE_FLAG(pcVPS->defaultPtlDpbHrdMaxTidFlag,        "vps_default_ptl_dpb_hrd_max_tid_flag");
  }
  if (pcVPS->maxLayers > 1)
  {
    WRITE_FLAG(pcVPS->allIndependentLayers,             "vps_all_independent_layers_flag");
  }
  for (uint32_t i = 0; i < pcVPS->maxLayers; i++)
  {
    WRITE_CODE(pcVPS->layerId[i], 6,                    "vps_layer_id");
    if (i > 0 && !pcVPS->allIndependentLayers)
    {
      WRITE_FLAG(pcVPS->independentLayer[i],            "vps_independent_layer_flag");
      if (!pcVPS->independentLayer[i])
      {
        for (int j = 0; j < i; j++)
        {
          WRITE_FLAG(pcVPS->directRefLayer[i][j],       "vps_direct_dependency_flag");
        }
        bool presentFlag = ( pcVPS->maxTidIlRefPicsPlus1[i] != 7 );
        WRITE_FLAG(presentFlag,                         "vps_max_tid_ref_present_flag[ i ]");
        if (presentFlag)
        {
          WRITE_CODE(pcVPS->maxTidIlRefPicsPlus1[i], 3, "vps_max_tid_il_ref_pics_plus1[ i ]");
        }
      }
    }
  }
  if( pcVPS->maxLayers > 1 )
  {
    if (pcVPS->allIndependentLayers)
    {
      WRITE_FLAG(pcVPS->eachLayerIsAnOls,               "vps_each_layer_is_an_ols_flag");
    }
    if (!pcVPS->eachLayerIsAnOls)
    {
      if (!pcVPS->allIndependentLayers)
      {
        WRITE_CODE(pcVPS->olsModeIdc, 2,                "vps_ols_mode_idc");
      }
      if (pcVPS->olsModeIdc == 2)
      {
        WRITE_CODE(pcVPS->numOutputLayerSets - 2, 8,    "vps_num_output_layer_sets_minus2");
        for (uint32_t i = 1; i < pcVPS->numOutputLayerSets; i++)
        {
          for (uint32_t j = 0; j < pcVPS->maxLayers; j++)
          {
            WRITE_FLAG(pcVPS->olsOutputLayer[i][j],     "vps_ols_output_layer_flag");
          }
        }
      }
    }
    CHECK(pcVPS->numPtls - 1 >= pcVPS->totalNumOLSs, "vps_num_ptls_minus1 shall be less than TotalNumOlss");
    WRITE_CODE(pcVPS->numPtls - 1, 8,                   "vps_num_ptls_minus1");
  }

  int totalNumOlss = pcVPS->totalNumOLSs;
  for (int i = 0; i < pcVPS->numPtls; i++)
  {
    if(i > 0)
    {
      WRITE_FLAG(pcVPS->ptPresent[i],                   "vps_ptl_present_flag");
    }
    if(!pcVPS->allLayersSameNumSubLayers)
    {
      WRITE_CODE(pcVPS->ptlMaxTemporalId[i] ,3,         "vps_ptl_max_temporal_id");
    }
  }
  int cnt = 0;
  while (m_pcBitIf->getNumBitsUntilByteAligned())
  {
    WRITE_FLAG( 0,                                      "vps_ptl_reserved_zero_bit");
    cnt++;
  }
  CHECK(cnt>=8, "More than '8' alignment bytes written");
  for (int i = 0; i < pcVPS->numPtls; i++)
  {
    codeProfileTierLevel(&pcVPS->profileTierLevel[i], pcVPS->ptPresent[i], pcVPS->ptlMaxTemporalId[i] - 1);
  }
  for (int i = 0; i < totalNumOlss; i++)
  {
    if(pcVPS->numPtls > 1 && pcVPS->numPtls != pcVPS->totalNumOLSs)
      WRITE_CODE(pcVPS->olsPtlIdx[i], 8,                "vps_ols_ptl_idx");
  }
  if( !pcVPS->allIndependentLayers )
  {
    WRITE_UVLC( pcVPS->numDpbParams,                    "vps_num_dpb_params" );
  }

  if( pcVPS->numDpbParams > 0 && pcVPS->maxSubLayers > 1 )
  {
    WRITE_FLAG( pcVPS->sublayerDpbParamsPresent,        "vps_sublayer_dpb_params_present_flag" );
  }

  for( int i = 0; i < pcVPS->numDpbParams; i++ )
  {
    if( !pcVPS->allLayersSameNumSubLayers )
    {
      WRITE_CODE( pcVPS->dpbMaxTemporalId[i], 3,      "vps_dpb_max_temporal_id[i]" );
    }
    if( pcVPS->maxSubLayers == 1 )
    {
      CHECK( pcVPS->dpbMaxTemporalId[i] != 0, "When vps_max_sublayers_minus1 is equal to 0, the value of dpb_max_temporal_id[ i ] is inferred to be equal to 0" );
    }
    else
    {
      if( pcVPS->defaultPtlDpbHrdMaxTidFlag )
      {
        CHECK( pcVPS->dpbMaxTemporalId[i] != pcVPS->maxSubLayers - 1, "When vps_max_sublayers_minus1 is greater than 0 and vps_all_layers_same_num_sublayers_flag is equal to 1, the value of dpb_max_temporal_id[ i ] is inferred to be equal to vps_max_sublayers_minus1" );
      }
      else
      {
        WRITE_CODE( pcVPS->dpbMaxTemporalId[i], 3,      "vps_dpb_max_temporal_id[i]" );
      }
    }

    for( int j = ( pcVPS->sublayerDpbParamsPresent ? 0 : pcVPS->dpbMaxTemporalId[i] ); j <= pcVPS->dpbMaxTemporalId[i]; j++ )
    {
      WRITE_UVLC( pcVPS->dpbParameters[i].maxDecPicBuffering[j],      "max_dec_pic_buffering_minus1[i]" );
      WRITE_UVLC( pcVPS->dpbParameters[i].numReorderPics[j],          "max_num_reorder_pics[i]" );
      WRITE_UVLC( pcVPS->dpbParameters[i].maxLatencyIncreasePlus1[j], "max_latency_increase_plus1[i]" );
    }
  }

  for( int i = 0; i < pcVPS->totalNumOLSs; i++ )
  {
    if( pcVPS->numLayersInOls[i] > 1 )
    {
      WRITE_UVLC( pcVPS->olsDpbPicSize[i].width,          "vps_ols_dpb_pic_width[i]" );
      WRITE_UVLC( pcVPS->olsDpbPicSize[i].height,         "vps_ols_dpb_pic_height[i]" );
      WRITE_CODE( pcVPS->olsDpbChromaFormatIdc[i], 2,     "vps_ols_dpb_chroma_format[i]");
      WRITE_UVLC( pcVPS->olsDpbBitDepthMinus8[i],         "vps_ols_dpb_bitdepth_minus8[i]");
      if( pcVPS->numDpbParams > 1 && (pcVPS->numDpbParams != pcVPS->numMultiLayeredOlss) )
      {
        WRITE_UVLC( pcVPS->olsDpbParamsIdx[i],            "vps_ols_dpb_params_idx[i]" );
      }
    }
  }


  if (!pcVPS->eachLayerIsAnOls)
  {
    WRITE_FLAG(pcVPS->generalHrdParamsPresent, "vps_general_hrd_params_present_flag");
  }
  if (pcVPS->generalHrdParamsPresent)
  {
    codeGeneralHrdparameters(&pcVPS->generalHrdParams);
    if ((pcVPS->maxSubLayers-1) > 0)
    {
      WRITE_FLAG(pcVPS->sublayerCpbParamsPresent, "vps_sublayer_cpb_params_present_flag");
    }
    WRITE_UVLC(pcVPS->numOlsHrdParamsMinus1, "vps_num_ols_hrd_params_minus1");
    for (int i = 0; i <= pcVPS->numOlsHrdParamsMinus1; i++)
    {
      if (!pcVPS->defaultPtlDpbHrdMaxTidFlag)
      {
        WRITE_CODE(pcVPS->hrdMaxTid[i], 3, "vps_hrd_vps_max_tid[i]");
      }
      uint32_t firstSublayer = pcVPS->sublayerCpbParamsPresent ? 0 : pcVPS->hrdMaxTid[i];
      codeOlsHrdParameters(&pcVPS->generalHrdParams, &pcVPS->olsHrdParams[i], firstSublayer, pcVPS->hrdMaxTid[i]);
    }
    if ((pcVPS->numOlsHrdParamsMinus1 > 0) && ((pcVPS->numOlsHrdParamsMinus1 + 1) != pcVPS->numMultiLayeredOlss))
    {
      for (int i = 0; i < pcVPS->numMultiLayeredOlss; i++)
      {
        WRITE_UVLC(pcVPS->olsHrdIdx[i], "vps_ols_hrd_idx[i]");
      }
    }
  }
  WRITE_FLAG(0,                                           "vps_extension_flag");

  //future extensions here..
  xWriteRbspTrailingBits();
}

void HLSWriter::codePictureHeader( const PicHeader* picHeader, bool writeRbspTrailingBits )
{
  const PPS*  pps = NULL;
  const SPS*  sps = NULL;

  DTRACE( g_trace_ctx, D_HEADER, "=========== Picture Header ===========\n" );

  CodingStructure& cs = *picHeader->pic->cs;
  WRITE_FLAG(picHeader->gdrOrIrapPic, "ph_gdr_or_irap_pic_flag");
  WRITE_FLAG(picHeader->nonRefPic,    "ph_non_ref_pic_flag");
  if (picHeader->gdrOrIrapPic)
  {
    WRITE_FLAG(picHeader->gdrPic,     "ph_gdr_pic_flag");
  }
  // Q0781, two-flags
  WRITE_FLAG(picHeader->picInterSliceAllowed,   "ph_inter_slice_allowed_flag");
  if (picHeader->picInterSliceAllowed)
  {
    WRITE_FLAG(picHeader->picIntraSliceAllowed, "ph_intra_slice_allowed_flag");
  }
  // parameter sets
  WRITE_UVLC(picHeader->ppsId,                  "ph_pic_parameter_set_id");
  pps = cs.slice->pps;
  CHECK(pps == 0, "Invalid PPS");
  sps = cs.slice->sps;
  CHECK(sps == 0, "Invalid SPS");
  int pocBits = cs.slice->sps->bitsForPOC;
  int pocMask = (1 << pocBits) - 1;
  WRITE_CODE(cs.slice->poc & pocMask, pocBits,  "ph_pic_order_cnt_lsb");
  if( picHeader->gdrPic )
  {
    WRITE_UVLC(picHeader->recoveryPocCnt,       "ph_recovery_poc_cnt");
  }

  // PH extra bits are not written in the reference encoder
  // as these bits are reserved for future extensions
  // for( i = 0; i < NumExtraPhBits; i++ )
  //    ph_extra_bit[ i ]

  if (sps->pocMsbFlag)
  {
    WRITE_FLAG(picHeader->pocMsbPresent,        "ph_poc_msb_present_flag");
    if (picHeader->pocMsbPresent)
    {
      WRITE_CODE(picHeader->pocMsbVal, sps->pocMsbLen, "ph_poc_msb_val");
    }
  }

   // alf enable flags and aps IDs
  if( sps->alfEnabled)
  {
    if (pps->alfInfoInPh)
    {
      WRITE_FLAG(picHeader->alfEnabled[COMP_Y],         "ph_alf_enabled_flag");
      if (picHeader->alfEnabled[COMP_Y])
      {
        WRITE_CODE(picHeader->numAlfAps, 3,             "ph_num_alf_aps_ids_luma");
        for (int i = 0; i < picHeader->numAlfAps; i++)
        {
          WRITE_CODE(picHeader->alfApsId[i], 3,         "ph_alf_aps_id_luma");
        }

        const int alfChromaIdc = picHeader->alfEnabled[COMP_Cb] + picHeader->alfEnabled[COMP_Cr] * 2 ;
        if (sps->chromaFormatIdc != CHROMA_400)
        {
          WRITE_CODE(picHeader->alfEnabled[COMP_Cb], 1, "ph_alf_cb_enabled_flag");
          WRITE_CODE(picHeader->alfEnabled[COMP_Cr], 1, "ph_alf_cr_enabled_flag");
        }
        if (alfChromaIdc)
        {
          WRITE_CODE(picHeader->alfChromaApsId, 3,      "ph_alf_aps_id_chroma");
        }
        if (sps->ccalfEnabled)
        {
          WRITE_FLAG(picHeader->ccalfEnabled[COMP_Cb],  "ph_cc_alf_cb_enabled_flag");
          if (picHeader->ccalfEnabled[COMP_Cb])
          {
            WRITE_CODE(picHeader->ccalfCbApsId, 3,      "ph_cc_alf_cb_aps_id");
          }
          WRITE_FLAG(picHeader->ccalfEnabled[COMP_Cr],  "ph_cc_alf_cr_enabled_flag");
          if (picHeader->ccalfEnabled[COMP_Cr])
          {
            WRITE_CODE(picHeader->ccalfCrApsId, 3,      "ph_cc_alf_cr_aps_id");
          }
        }
      }
    }
  }

  // luma mapping / chroma scaling controls
  if (sps->lumaReshapeEnable)
  {
    WRITE_FLAG(picHeader->lmcsEnabled,                  "ph_lmcs_enabled_flag");
    if (picHeader->lmcsEnabled)
    {
      WRITE_CODE(picHeader->lmcsApsId, 2,               "ph_lmcs_aps_id");
      if (sps->chromaFormatIdc != CHROMA_400)
      {
        WRITE_FLAG(picHeader->lmcsChromaResidualScale,  "ph_chroma_residual_scale_flag");
      }
    }
  }

  // quantization scaling lists
  if( sps->scalingListEnabled )
  {
    WRITE_FLAG( picHeader->explicitScalingListEnabled,  "ph_scaling_list_present_flag" );
    if( picHeader->explicitScalingListEnabled )
    {
      WRITE_CODE( picHeader->scalingListApsId, 3,       "ph_scaling_list_aps_id" );
    }
  }

  // virtual boundaries
  if( sps->virtualBoundariesEnabled && !sps->virtualBoundariesPresent )
  {
    WRITE_FLAG( picHeader->virtualBoundariesEnabled,    "ph_loop_filter_across_virtual_boundaries_disabled_present_flag" );
    if( picHeader->virtualBoundariesEnabled )
    {
      WRITE_CODE(picHeader->numVerVirtualBoundaries, 2, "ph_num_ver_virtual_boundaries");
      for( unsigned i = 0; i < picHeader->numVerVirtualBoundaries; i++ )
      {
        WRITE_UVLC(picHeader->virtualBoundariesPosX[i] >> 3, "ph_virtual_boundaries_pos_x");
      }
      WRITE_CODE(picHeader->numHorVirtualBoundaries, 2, "ph_num_hor_virtual_boundaries");
      for( unsigned i = 0; i < picHeader->numHorVirtualBoundaries; i++ )
      {
        WRITE_UVLC(picHeader->virtualBoundariesPosY[i]>>3, "ph_virtual_boundaries_pos_y");
      }
    }
  }

  // picture output flag
  if( pps->outputFlagPresent && !picHeader->nonRefPic)
  {
    WRITE_FLAG( picHeader->picOutputFlag, "ph_pic_output_flag" );
  }

  // reference picture lists
  if (pps->rplInfoInPh)
  {
    // List0 and List1
    for(int listIdx = 0; listIdx < 2; listIdx++)
    {
      if(sps->getNumRPL(listIdx) > 0 &&
          (listIdx == 0 || (listIdx == 1 && pps->rpl1IdxPresent)))
      {
        WRITE_FLAG(picHeader->rplIdx[listIdx] != -1 ? 1 : 0, "pic_rpl_sps_flag[i]");
      }
      else if(sps->getNumRPL(listIdx) == 0)
      {
        CHECK(picHeader->rplIdx[listIdx] != -1, "rpl_sps_flag[1] will be infer to 0 and this is not what was expected");
      }
      else if(listIdx == 1)
      {
        auto rplsSpsFlag0 = picHeader->rplIdx[0] != -1 ? 1 : 0;
        auto rplsSpsFlag1 = picHeader->rplIdx[1] != -1 ? 1 : 0;
        CHECK(rplsSpsFlag1 != rplsSpsFlag0, "rpl_sps_flag[1] will be infer to 0 and this is not what was expected");
      }

      if(picHeader->rplIdx[listIdx] != -1)
      {
        if(sps->getNumRPL(listIdx) > 1 &&
            (listIdx == 0 || (listIdx == 1 && pps->rpl1IdxPresent)))
        {
          int numBits = ceilLog2(sps->getNumRPL( listIdx ));
          WRITE_CODE(picHeader->rplIdx[listIdx], numBits, "pic_rpl_idx[i]");
        }
        else if(sps->getNumRPL(listIdx) == 1)
        {
          CHECK(picHeader->rplIdx[listIdx] != 0, "RPL1Idx is not signalled but it is not equal to 0");
        }
        else
        {
          CHECK(picHeader->rplIdx[1] != picHeader->rplIdx[0], "RPL1Idx is not signalled but it is not the same as RPL0Idx");
        }
      }
      // explicit RPL in picture header
      else
      {
        xCodeRefPicList( picHeader->pRPL[listIdx], sps->longTermRefsPresent, sps->bitsForPOC, !(sps->weightPred||sps->weightedBiPred), -1 );
      }

      // POC MSB cycle signalling for LTRP
      if (picHeader->pRPL[listIdx]->numberOfLongtermPictures)
      {
        for (int i = 0; i < picHeader->pRPL[listIdx]->numberOfLongtermPictures + picHeader->pRPL[listIdx]->numberOfShorttermPictures; i++)
        {
          if (picHeader->pRPL[listIdx]->isLongtermRefPic[i])
          {
            if (picHeader->pRPL[listIdx]->ltrpInSliceHeader)
            {
              WRITE_CODE(picHeader->pRPL[listIdx]->refPicIdentifier[i], sps->bitsForPOC,
                         "pic_poc_lsb_lt[listIdx][rplsIdx][j]");
            }
            WRITE_FLAG(picHeader->pRPL[listIdx]->deltaPocMSBPresent[i], "pic_delta_poc_msb_present_flag[i][j]");
            if (picHeader->pRPL[listIdx]->deltaPocMSBPresent[i])
            {
              WRITE_UVLC(picHeader->pRPL[listIdx]->deltaPocMSBCycleLT[i], "pic_delta_poc_msb_cycle_lt[i][j]");
            }
          }
        }
      }
    }
  }

  // partitioning constraint overrides
  if (sps->partitionOverrideEnabled )
  {
    WRITE_FLAG(picHeader->splitConsOverride, "partition_constraints_override_flag");
  }

  // Q0781, two-flags
  if (picHeader->picIntraSliceAllowed)
  {
    if (picHeader->splitConsOverride)
    {
      WRITE_UVLC(floorLog2(picHeader->minQTSize[0]) - sps->log2MinCodingBlockSize, "pic_log2_diff_min_qt_min_cb_intra_slice_luma");
      WRITE_UVLC(picHeader->maxMTTDepth[0], "ph_max_mtt_hierarchy_depth_intra_slice_luma");
      if (picHeader->maxMTTDepth[0] != 0)
      {
        WRITE_UVLC(floorLog2(picHeader->maxBTSize[0]) - floorLog2(picHeader->minQTSize[0]), "ph_log2_diff_max_bt_min_qt_intra_slice_luma");
        WRITE_UVLC(floorLog2(picHeader->maxTTSize[0]) - floorLog2(picHeader->minQTSize[0]), "ph_log2_diff_max_tt_min_qt_intra_slice_luma");
      }

      if (sps->dualITree)
      {
        WRITE_UVLC(floorLog2(picHeader->minQTSize[2]) - sps->log2MinCodingBlockSize, "ph_log2_diff_min_qt_min_cb_intra_slice_chroma");
        WRITE_UVLC(picHeader->maxMTTDepth[2], "ph_max_mtt_hierarchy_depth_intra_slice_chroma");
        if (picHeader->maxMTTDepth[2] != 0)
        {
          WRITE_UVLC(floorLog2(picHeader->maxBTSize[2]) - floorLog2(picHeader->minQTSize[2]), "ph_log2_diff_max_bt_min_qt_intra_slice_chroma");
          WRITE_UVLC(floorLog2(picHeader->maxTTSize[2]) - floorLog2(picHeader->minQTSize[2]), "ph_log2_diff_max_tt_min_qt_intra_slice_chroma");
        }
      }
    }
  }
  if (picHeader->picIntraSliceAllowed )
  {
  // delta quantization and chrom and chroma offset
    if (pps->useDQP)
    {
      WRITE_UVLC( picHeader->cuQpDeltaSubdivIntra, "ph_cu_qp_delta_subdiv_intra_slice" );
    }
    if (pps->chromaQpOffsetListLen )
    {
      WRITE_UVLC( picHeader->cuChromaQpOffsetSubdivIntra, "ph_cu_chroma_qp_offset_subdiv_intra_slice" );
    }
  }

  if (picHeader->picInterSliceAllowed )
  {
    if (picHeader->splitConsOverride )
    {
      WRITE_UVLC(floorLog2(picHeader->minQTSize[1]) - sps->log2MinCodingBlockSize, "ph_log2_diff_min_qt_min_cb_inter_slice");
      WRITE_UVLC(picHeader->maxMTTDepth[1], "ph_max_mtt_hierarchy_depth_inter_slice");
      if (picHeader->maxMTTDepth[1] != 0)
      {
        WRITE_UVLC(floorLog2(picHeader->maxBTSize[1]) - floorLog2(picHeader->minQTSize[1]), "ph_log2_diff_max_bt_min_qt_inter_slice");
        WRITE_UVLC(floorLog2(picHeader->maxTTSize[1]) - floorLog2(picHeader->minQTSize[1]), "ph_log2_diff_max_tt_min_qt_inter_slice");
      }
    }

    // delta quantization and chrom and chroma offset
    if (pps->useDQP)
    {
      WRITE_UVLC(picHeader->cuQpDeltaSubdivInter, "ph_cu_qp_delta_subdiv_inter_slice");
    }

    if (pps->chromaQpOffsetListLen )
    {
      WRITE_UVLC(picHeader->cuChromaQpOffsetSubdivInter, "ph_cu_chroma_qp_offset_subdiv_inter_slice");
    }

    // temporal motion vector prediction
    if (sps->temporalMVPEnabled)
    {
      WRITE_FLAG( picHeader->enableTMVP, "ph_temporal_mvp_enabled_flag" );
      if (picHeader->enableTMVP && pps->rplInfoInPh)
      {
        if (picHeader->pRPL[1]->getNumRefEntries() > 0)
        {
          WRITE_CODE(picHeader->picColFromL0, 1, "ph_collocated_from_l0_flag");
        }
        if ((picHeader->picColFromL0 && picHeader->pRPL[0]->getNumRefEntries() > 1) ||
          (!picHeader->picColFromL0 && picHeader->pRPL[1]->getNumRefEntries() > 1))
        {
          WRITE_UVLC(picHeader->colRefIdx, "ph_collocated_ref_idx");
        }
      }
    }

  // full-pel MMVD flag
    if (sps->fpelMmvd )
    {
      WRITE_FLAG( picHeader->disFracMMVD, "ph_fpel_mmvd_enabled_flag" );
    }
  // mvd L1 zero flag
    if (!pps->rplInfoInPh || picHeader->pRPL[1]->getNumRefEntries() > 0)
    {
      WRITE_FLAG(picHeader->mvdL1Zero, "ph_mvd_l1_zero_flag");
    }

  // picture level BDOF disable flags
    if (sps->BdofPresent && (!pps->rplInfoInPh || picHeader->pRPL[1]->getNumRefEntries() > 0))
    {
      WRITE_FLAG(picHeader->disBdofFlag, "ph_disable_bdof_flag");
    }

  // picture level DMVR disable flags
    if (sps->DmvrPresent && (!pps->rplInfoInPh || picHeader->pRPL[1]->getNumRefEntries() > 0))
    {
      WRITE_FLAG(picHeader->disDmvrFlag, "ph_disable_dmvr_flag");
    }

  // picture level PROF disable flags
    if (sps->ProfPresent)
    {
      WRITE_FLAG(picHeader->disProfFlag, "ph_disable_prof_flag");
    }

    if ((pps->weightPred || pps->weightedBiPred) && pps->wpInfoInPh )
    {
      xCodePredWeightTable(picHeader, sps);
    }
   }

  if (pps->qpDeltaInfoInPh)
  {
    WRITE_SVLC(picHeader->qpDelta, "ph_qp_delta");
  }

  // joint Cb/Cr sign flag
  if (sps->jointCbCr )
  {
    WRITE_FLAG( picHeader->jointCbCrSign, "ph_joint_cbcr_sign_flag" );
  }

  // sao enable flags
  if(sps->saoEnabled)
  {
    if (pps->saoInfoInPh)
    {
      WRITE_FLAG(picHeader->saoEnabled[CH_L], "ph_sao_luma_enabled_flag");
      if (sps->chromaFormatIdc != CHROMA_400)
      {
        WRITE_FLAG(picHeader->saoEnabled[CH_C], "ph_sao_chroma_enabled_flag");
      }
    }
  }

  // deblocking filter controls
  if (pps->deblockingFilterControlPresent )
  {
    if(pps->deblockingFilterOverrideEnabled)
    {
      if (pps->dbfInfoInPh)
      {
        WRITE_FLAG ( picHeader->deblockingFilterOverride, "ph_deblocking_filter_override_flag" );
      }
    }

    if(picHeader->deblockingFilterOverride)
    {
      WRITE_FLAG( picHeader->deblockingFilterDisable, "ph_deblocking_filter_disabled_flag" );
      if( !picHeader->deblockingFilterDisable )
      {
        WRITE_SVLC( picHeader->deblockingFilterBetaOffsetDiv2[COMP_Y], "ph_beta_offset_div2" );
        WRITE_SVLC( picHeader->deblockingFilterTcOffsetDiv2[COMP_Y], "ph_tc_offset_div2" );
        if( pps->usePPSChromaTool )
        {
          WRITE_SVLC( picHeader->deblockingFilterBetaOffsetDiv2[COMP_Cb], "ph_cb_beta_offset_div2" );
          WRITE_SVLC( picHeader->deblockingFilterTcOffsetDiv2[COMP_Cb], "ph_cb_tc_offset_div2" );
          WRITE_SVLC( picHeader->deblockingFilterBetaOffsetDiv2[COMP_Cr], "ph_cr_beta_offset_div2" );
          WRITE_SVLC( picHeader->deblockingFilterTcOffsetDiv2[COMP_Cr], "ph_cr_tc_offset_div2" );
        }
      }
    }
  }

  // picture header extension
  if(pps->pictureHeaderExtensionPresent)
  {
    WRITE_UVLC(0,"ph_extension_length");
  }

  if ( writeRbspTrailingBits )
  {
    xWriteRbspTrailingBits();
  }
}


void HLSWriter::codeSliceHeader( const Slice* slice )
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Slice ===========\n" );

  CodingStructure& cs        = *slice->pic->cs;
  const PicHeader *picHeader = cs.picHeader;
  const ChromaFormat format  = slice->sps->chromaFormatIdc;
  const uint32_t numberValidComponents = getNumberValidComponents(format);
  const bool chromaEnabled = isChromaEnabled(format);

  WRITE_FLAG(slice->pictureHeaderInSliceHeader, "sh_picture_header_in_slice_header_flag");
  if(slice->pictureHeaderInSliceHeader)
  {
    codePictureHeader(picHeader, false);
  }

  if (slice->sps->subPicInfoPresent)
  {
    uint32_t bitsSubPicId;
    if (slice->sps->subPicIdMappingExplicitlySignalled)
    {
      bitsSubPicId = slice->sps->subPicIdLen;
    }
    else if (slice->pps->subPicIdMappingInPps)
    {
      bitsSubPicId = slice->pps->subPicIdLen;
    }
    else
    {
      bitsSubPicId = ceilLog2(slice->sps->numSubPics);
    }
    WRITE_CODE(slice->sliceSubPicId, bitsSubPicId, "sh_subpic_id");
  }

  if (!slice->pps->rectSlice)
  {
    THROW("no suppport");
  }
  else
  {
    // slice address is the index of the slice within the current sub-picture
    uint32_t currSubPicIdx = slice->pps->getSubPicIdxFromSubPicId( slice->sliceSubPicId );
    SubPic currSubPic = slice->pps->subPics[currSubPicIdx];
    if( currSubPic.numSlicesInSubPic > 1 )
    {
      int numSlicesInPreviousSubPics = 0;
      for(int sp = 0; sp < currSubPicIdx; sp++)
      {
        numSlicesInPreviousSubPics += slice->pps->subPics[sp].numSlicesInSubPic;
      }
      int bitsSliceAddress = ceilLog2(currSubPic.numSlicesInSubPic);
      WRITE_CODE( slice->sliceSubPicId - numSlicesInPreviousSubPics, bitsSliceAddress, "sh_slice_address");
    }
  }

  if (picHeader->picInterSliceAllowed)
  {
    WRITE_UVLC(slice->sliceType, "sh_slice_type");
  }
  if (picHeader->gdrOrIrapPic) //th check this
  {
    WRITE_FLAG(picHeader->noOutputOfPriorPics, "sh_no_output_of_prior_pics_flag");
  }

  if (!picHeader->picIntraSliceAllowed )
  {
    CHECK(slice->sliceType == I_SLICE, "when pic_intra_slice_allowed_flag = 0, no I_Slice is allowed");
  }

  if (slice->sps->alfEnabled && !slice->pps->alfInfoInPh)
  {
    const int alfEnabled = slice->tileGroupAlfEnabled[COMP_Y];
    WRITE_FLAG(alfEnabled, "sh_alf_enabled_flag");

    if (alfEnabled)
    {
      WRITE_CODE(slice->tileGroupNumAps, 3, "sh_num_alf_aps_ids_luma");
      for (int i = 0; i < slice->tileGroupNumAps; i++)
      {
        WRITE_CODE(slice->tileGroupLumaApsId[i], 3, "sh_alf_aps_id_luma");
      }

      const int alfChromaIdc = slice->tileGroupAlfEnabled[COMP_Cb] + slice->tileGroupAlfEnabled[COMP_Cr] * 2;
      if (chromaEnabled)
      {
        WRITE_FLAG(slice->tileGroupAlfEnabled[COMP_Cb], "sh_alf_cb_enabled_flag");
        WRITE_FLAG(slice->tileGroupAlfEnabled[COMP_Cr], "sh_alf_cr_enabled_flag");
      }
      if (alfChromaIdc)
      {
        WRITE_CODE(slice->tileGroupChromaApsId, 3,      "sh_alf_aps_id_chroma");
      }

      if (slice->sps->ccalfEnabled)
      {
        WRITE_FLAG(slice->tileGroupCcAlfCbEnabled,      "sh_cc_alf_cb_enabled_flag");
        if( slice->tileGroupCcAlfCbEnabled )
        {
          // write CC ALF Cb APS ID
          WRITE_CODE(slice->tileGroupCcAlfCbApsId, 3,   "sh_cc_alf_cb_aps_id");
        }
        // Cr
        WRITE_FLAG(slice->tileGroupCcAlfCrEnabled,      "sh_cc_alf_cr_enabled_flag");
        if( slice->tileGroupCcAlfCrEnabled )
        {
          // write CC ALF Cr APS ID
          WRITE_CODE(slice->tileGroupCcAlfCrApsId, 3,   "sh_cc_alf_cr_aps_id");
        }
      }
    }
  }

  if (picHeader->lmcsEnabled && !slice->pictureHeaderInSliceHeader)
  {
    WRITE_FLAG( slice->lmcsEnabled,             "sh_lmcs_enabled_flag");
  }
  if (picHeader->explicitScalingListEnabled && !slice->pictureHeaderInSliceHeader)
  {
    WRITE_FLAG(slice->explicitScalingListUsed,  "sh_explicit_scaling_list_used_flag");
  }

  if(  !slice->pps->rplInfoInPh && (!slice->getIdrPicFlag() || slice->sps->idrRefParamList))
  {
    int numRPL0 = slice->sps->getNumRPL(0);
    //Write L0 related syntax elements
    if (numRPL0 > 0)
    {
      WRITE_FLAG(slice->rplIdx[0] != -1, "ref_pic_list_sps_flag[0]");
    }
    if (slice->rplIdx[0] != -1)
    {
      if (numRPL0 > 1)
      {
        int numBits = 0;
        while ((1 << numBits) < numRPL0)
        {
          numBits++;
        }
        WRITE_CODE(slice->rplIdx[0], numBits, "ref_pic_list_idx[0]");
      }
    }
    else
    {  //write local RPL0
      xCodeRefPicList( slice->rpl[0], slice->sps->longTermRefsPresent, slice->sps->bitsForPOC, !slice->sps->weightPred && !slice->sps->weightedBiPred, -1 );
    }
    //Deal POC Msb cycle signalling for LTRP
    if (slice->rpl[0]->numberOfLongtermPictures)
    {
      for (int i = 0; i < slice->rpl[0]->numberOfLongtermPictures + slice->rpl[0]->numberOfShorttermPictures; i++)
      {
        if (slice->rpl[0]->isLongtermRefPic[i])
        {
          if (slice->rpl[0]->ltrpInSliceHeader)
          {
            WRITE_CODE(slice->rpl[0]->refPicIdentifier[i], slice->sps->bitsForPOC, "slice_poc_lsb_lt[listIdx][rplsIdx][j]");
          }
          WRITE_FLAG(slice->rpl[0]->deltaPocMSBPresent[i], "delta_poc_msb_present_flag[i][j]");
          if (slice->rpl[0]->deltaPocMSBPresent[i])
          {
            WRITE_UVLC(slice->rpl[0]->deltaPocMSBCycleLT[i], "delta_poc_msb_cycle_lt[i][j]");
          }
        }
      }
    }

    //Write L1 related syntax elements
      if (!slice->pps->rpl1IdxPresent && slice->pps->rpl1IdxPresent)
      {
        WRITE_FLAG(slice->rplIdx[1] != -1 ? 1 : 0, "ref_pic_list_sps_flag[1]");
      }
      else if (slice->sps->getNumRPL(1) == 0)
      {
        CHECK(slice->rplIdx[1] != -1, "rpl_sps_flag[1] will be infer to 0 and this is not what was expected");
      }
      else
      {
        auto rplsSpsFlag0 = slice->rplIdx[0] != -1 ? 1 : 0;
        auto rplsSpsFlag1 = slice->rplIdx[1] != -1 ? 1 : 0;
        CHECK(rplsSpsFlag1 != rplsSpsFlag0, "rpl_sps_flag[1] will be infer to 0 and this is not what was expected");
      }

      if (slice->rplIdx[1] != -1)
      {
        if (slice->sps->getNumRPL(1) > 1 && slice->pps->rpl1IdxPresent)
        {
          int numBits = 0;
          while ((1 << numBits) < slice->sps->getNumRPL(1))
          {
            numBits++;
          }
          WRITE_CODE(slice->rplIdx[1], numBits, "ref_pic_list_idx[1]");
        }
        else if (slice->sps->getNumRPL(1) == 1)
        {
          CHECK(slice->rplIdx[1] != 0, "RPL1Idx is not signalled but it is not equal to 0");
        }
        else
        {
          CHECK(slice->rplIdx[1] != slice->rplIdx[0], "RPL1Idx is not signalled but it is not the same as RPL0Idx");
        }
      }
      else
      {  //write local RPL1
        xCodeRefPicList( slice->rpl[1], slice->sps->longTermRefsPresent, slice->sps->bitsForPOC, !(slice->sps->weightPred || slice->sps->weightedBiPred), -1 );
      }
      //Deal POC Msb cycle signalling for LTRP
      if (slice->rpl[1]->numberOfLongtermPictures)
      {
        for (int i = 0; i < slice->rpl[1]->numberOfLongtermPictures + slice->rpl[1]->numberOfShorttermPictures; i++)
        {
          if (slice->rpl[1]->isLongtermRefPic[i])
          {
            if (slice->rpl[1]->ltrpInSliceHeader)
            {
              WRITE_CODE(slice->rpl[1]->refPicIdentifier[i], slice->sps->bitsForPOC,
                         "slice_poc_lsb_lt[listIdx][rplsIdx][j]");
            }
            WRITE_FLAG(slice->rpl[1]->deltaPocMSBPresent[i], "delta_poc_msb_present_flag[i][j]");
            if (slice->rpl[1]->deltaPocMSBPresent[i])
            {
              WRITE_UVLC(slice->rpl[1]->deltaPocMSBCycleLT[i], "delta_poc_msb_cycle_lt[i][j]");
            }
          }
        }
      }
    }

    //check if numrefidxes match the defaults. If not, override

    if ((!slice->isIntra() && slice->rpl[0]->getNumRefEntries() > 1) ||
        (slice->isInterB() && slice->rpl[1]->getNumRefEntries() > 1) )
    {
      int defaultL0 = std::min<int>(slice->rpl[0]->getNumRefEntries(), slice->pps->numRefIdxL0DefaultActive);
      int defaultL1 = slice->isInterB() ? std::min<int>(slice->rpl[1]->getNumRefEntries(), slice->pps->numRefIdxL1DefaultActive) : 0;
      bool overrideFlag = ( slice->numRefIdx[ REF_PIC_LIST_0 ] != defaultL0 || ( slice->isInterB() && slice->numRefIdx[ REF_PIC_LIST_1 ] != defaultL1 ) );
      WRITE_FLAG( overrideFlag ? 1 : 0, "num_ref_idx_active_override_flag" );
      if( overrideFlag )
      {
        if(slice->rpl[0]->getNumRefEntries() > 1)
        {
          WRITE_UVLC( slice->numRefIdx[ REF_PIC_LIST_0 ] - 1, "num_ref_idx_l0_active_minus1" );
        }

        if( slice->isInterB() && slice->rpl[1]->getNumRefEntries() > 1)
        {
          WRITE_UVLC( slice->numRefIdx[ REF_PIC_LIST_1 ] - 1, "num_ref_idx_l1_active_minus1" );
        }
      }
    }

    if( !slice->isIntra() )
    {
      if( !slice->isIntra() && slice->pps->cabacInitPresent )
      {
        const SliceType encCABACTableIdx = slice->encCABACTableIdx;
        bool encCabacInitFlag = ( slice->sliceType != encCABACTableIdx && encCABACTableIdx != I_SLICE ) ? true : false;
        WRITE_FLAG( encCabacInitFlag ? 1 : 0, "sh_cabac_init_flag" );
      }
    }

    if( slice->picHeader->enableTMVP  && !slice->pps->rplInfoInPh)
    {
      if(!slice->pps->rplInfoInPh)
      {
        if (slice->sliceType == B_SLICE)
        {
          WRITE_FLAG(slice->colFromL0Flag, "sh_collocated_from_l0_flag");
        }
      }

    if( slice->sliceType != I_SLICE &&
      ( ( slice->colFromL0Flag == 1 && slice->numRefIdx[ REF_PIC_LIST_0 ] > 1 ) ||
        ( slice->colFromL0Flag == 0 && slice->numRefIdx[ REF_PIC_LIST_1 ] > 1 ) ) )
    {
      WRITE_UVLC( slice->colRefIdx, "sh_collocated_ref_idx" );
    }
  }

  if( ( slice->pps->weightPred && slice->sliceType == P_SLICE ) || ( slice->pps->weightedBiPred && slice->sliceType == B_SLICE ) )
  {
    if( !slice->pps->wpInfoInPh )
    {
      xCodePredWeightTable( slice );
    }
  }

  if (!slice->pps->qpDeltaInfoInPh)
  {
    WRITE_SVLC(slice->sliceQp - (slice->pps->picInitQPMinus26 + 26), "slice_qp_delta");
  }
  if (slice->pps->sliceChromaQpFlag)
  {
    if (numberValidComponents > COMP_Cb)
    {
      WRITE_SVLC( slice->sliceChromaQpDelta[COMP_Cb], "sh_cb_qp_offset" );
    }
    if (numberValidComponents > COMP_Cr)
    {
      WRITE_SVLC( slice->sliceChromaQpDelta[COMP_Cr], "sh_cr_qp_offset" );
      if (slice->sps->jointCbCr)
      {
        WRITE_SVLC( slice->sliceChromaQpDelta[COMP_JOINT_CbCr], "sh_joint_cbcr_qp_offset");
      }
    }
    CHECK(numberValidComponents < COMP_Cr+1, "Too many valid components");
  }

  if (slice->pps->chromaQpOffsetListLen>0)
  {
    WRITE_FLAG(slice->chromaQpAdjEnabled, "sh_cu_chroma_qp_offset_enabled_flag");
  }

  if( slice->sps->saoEnabled && !slice->pps->saoInfoInPh )
  {
    WRITE_FLAG( slice->saoEnabled[CH_L], "sh_sao_luma_flag" );
    if( chromaEnabled )
    {
      WRITE_FLAG( slice->saoEnabled[CH_C], "sh_sao_chroma_flag" );
    }
  }


  if (slice->pps->deblockingFilterControlPresent && !slice->pps->dbfInfoInPh)
  {
    if (slice->pps->deblockingFilterOverrideEnabled )
    {
      WRITE_FLAG(slice->deblockingFilterOverrideFlag, "sh_deblocking_params_present_flag");
    }
    if (slice->deblockingFilterOverrideFlag)
    {
      if (!slice->pps->deblockingFilterDisabled)
      {
       WRITE_FLAG(slice->deblockingFilterDisable, "sh_deblocking_filter_disabled_flag");
      }
      if(!slice->deblockingFilterDisable)
      {
        WRITE_SVLC (slice->deblockingFilterBetaOffsetDiv2[COMP_Y],   "slice_beta_offset_div2");
        WRITE_SVLC (slice->deblockingFilterTcOffsetDiv2[COMP_Y],     "slice_tc_offset_div2");
        if( slice->pps->usePPSChromaTool )
        {
          WRITE_SVLC (slice->deblockingFilterBetaOffsetDiv2[COMP_Cb], "slice_cb_beta_offset_div2");
          WRITE_SVLC (slice->deblockingFilterTcOffsetDiv2[COMP_Cb],   "slice_cb_tc_offset_div2");
          WRITE_SVLC (slice->deblockingFilterBetaOffsetDiv2[COMP_Cr], "slice_cr_beta_offset_div2");
          WRITE_SVLC (slice->deblockingFilterTcOffsetDiv2[COMP_Cr],   "slice_cr_tc_offset_div2");
        }
      }
    }
  }

  // dependent quantization
  if( slice->sps->depQuantEnabled )
  {
    WRITE_FLAG(slice->depQuantEnabled, "sh_dep_quant_used_flag");
  }

  // sign data hiding
  if( slice->sps->signDataHidingEnabled && !slice->depQuantEnabled )
  {
    WRITE_FLAG(slice->signDataHidingEnabled, "sh_sign_data_hiding_used_flag" );
  }
  if( slice->sps->transformSkip && !slice->depQuantEnabled && !slice->signDataHidingEnabled )
  {
    WRITE_FLAG(slice->tsResidualCodingDisabled, "sh_ts_residual_coding_disabled_flag");
  }

  if(slice->pps->sliceHeaderExtensionPresent)
  {
    WRITE_UVLC(0,"slice_header_extension_length");
  }
}

void  HLSWriter::codeConstraintInfo  ( const ConstraintInfo* cinfo )
{
  WRITE_FLAG(cinfo->gciPresent,                         "gci_present_flag");
  if (cinfo->gciPresent)
  {
    WRITE_FLAG(cinfo->intraOnlyConstraintFlag,              "gci_intra_only_constraint_flag");
    WRITE_FLAG(cinfo->allLayersIndependentConstraintFlag,   "gci_all_layers_independent_constraint_flag");
    WRITE_FLAG(cinfo->onePictureOnlyConstraintFlag,         "gci_one_au_only_constraint_flag");

    /* picture format */
    WRITE_CODE(16 - cinfo->maxBitDepthConstraintIdc, 4,     "gci_sixteen_minus_max_bitdepth_constraint_idc");
    WRITE_CODE(3 - cinfo->maxChromaFormatConstraintIdc, 2,  "gci_three_minus_max_chroma_format_constraint_idc");

    /* NAL unit type related */
    WRITE_FLAG(cinfo->noMixedNaluTypesInPicConstraintFlag,  "gci_no_mixed_nalu_types_in_pic_constraint_flag");
    WRITE_FLAG(cinfo->noTrailConstraintFlag,                "gci_no_trail_constraint_flag");
    WRITE_FLAG(cinfo->noStsaConstraintFlag,                 "gci_no_stsa_constraint_flag");
    WRITE_FLAG(cinfo->noRaslConstraintFlag,                 "gci_no_rasl_constraint_flag");
    WRITE_FLAG(cinfo->noRadlConstraintFlag,                 "gci_no_radl_constraint_flag");
    WRITE_FLAG(cinfo->noIdrConstraintFlag,                  "gci_no_idr_constraint_flag");
    WRITE_FLAG(cinfo->noCraConstraintFlag,                  "gci_no_cra_constraint_flag");
    WRITE_FLAG(cinfo->noGdrConstraintFlag,                  "gci_no_gdr_constraint_flag");
    WRITE_FLAG(cinfo->noApsConstraintFlag,                  "gci_no_aps_constraint_flag");
    WRITE_FLAG(cinfo->noIdrRplConstraintFlag,               "gci_no_idr_rpl_constraint_flag");

    /* tile, slice, subpicture partitioning */
    WRITE_FLAG(cinfo->oneTilePerPicConstraintFlag,          "gci_one_tile_per_pic_constraint_flag");
    WRITE_FLAG(cinfo->picHeaderInSliceHeaderConstraintFlag, "gci_pic_header_in_slice_header_constraint_flag");
    WRITE_FLAG(cinfo->oneSlicePerPicConstraintFlag,         "gci_one_slice_per_pic_constraint_flag");
    WRITE_FLAG(cinfo->noRectSliceConstraintFlag,            "gci_no_rectangular_slice_constraint_flag");
    WRITE_FLAG(cinfo->oneSlicePerSubpicConstraintFlag,      "gci_one_slice_per_subpic_constraint_flag");
    WRITE_FLAG(cinfo->noSubpicInfoConstraintFlag,           "gci_no_subpic_info_constraint_flag");


    /* CTU and block partitioning */
    WRITE_CODE(3 - (cinfo->maxLog2CtuSizeConstraintIdc - 5), 2, "gci_three_minus_max_log2_ctu_size_constraint_idc");
    WRITE_FLAG(cinfo->noPartitionConstraintsOverrideConstraintFlag, "gci_no_partition_constraints_override_constraint_flag");
    WRITE_FLAG(cinfo->noMttConstraintFlag,                  "gci_no_mtt_constraint_flag");
    WRITE_FLAG(cinfo->noQtbttDualTreeIntraConstraintFlag,   "gci_no_qtbtt_dual_tree_intra_constraint_flag");

    /* intra */
    WRITE_FLAG(cinfo->noPaletteConstraintFlag,              "gci_no_palette_constraint_flag");
    WRITE_FLAG(cinfo->noIbcConstraintFlag,                  "gci_no_ibc_constraint_flag");
    WRITE_FLAG(cinfo->noIspConstraintFlag,                  "gci_no_isp_constraint_flag");
    WRITE_FLAG(cinfo->noMrlConstraintFlag,                  "gci_no_mrl_constraint_flag");
    WRITE_FLAG(cinfo->noMipConstraintFlag,                  "gci_no_mip_constraint_flag");
    WRITE_FLAG(cinfo->noCclmConstraintFlag,                 "gci_no_cclm_constraint_flag");

    /* inter */
    WRITE_FLAG(cinfo->noRprConstraintFlag,                  "gci_no_ref_pic_resampling_constraint_flag");
    WRITE_FLAG(cinfo->noResChangeInClvsConstraintFlag,      "gci_no_res_change_in_clvs_constraint_flag");
    WRITE_FLAG(cinfo->noWeightedPredictionConstraintFlag,   "gci_no_weighted_prediction_constraint_flag");
    WRITE_FLAG(cinfo->noRefWraparoundConstraintFlag,        "gci_no_ref_wraparound_constraint_flag");
    WRITE_FLAG(cinfo->noTemporalMvpConstraintFlag,          "gci_no_temporal_mvp_constraint_flag");
    WRITE_FLAG(cinfo->noSbtmvpConstraintFlag,               "gci_no_sbtmvp_constraint_flag");
    WRITE_FLAG(cinfo->noAmvrConstraintFlag,                 "gci_no_amvr_constraint_flag");
    WRITE_FLAG(cinfo->noBdofConstraintFlag,                 "gci_no_bdof_constraint_flag");
    WRITE_FLAG(cinfo->noSmvdConstraintFlag,                 "gci_no_smvd_constraint_flag");
    WRITE_FLAG(cinfo->noDmvrConstraintFlag,                 "gci_no_dmvr_constraint_flag");
    WRITE_FLAG(cinfo->noMmvdConstraintFlag,                 "gci_no_mmvd_constraint_flag");
    WRITE_FLAG(cinfo->noAffineMotionConstraintFlag,         "gci_no_affine_motion_constraint_flag");
    WRITE_FLAG(cinfo->noProfConstraintFlag,                 "gci_no_prof_constraint_flag");
    WRITE_FLAG(cinfo->noBcwConstraintFlag,                  "gci_no_bcw_constraint_flag");
    WRITE_FLAG(cinfo->noCiipConstraintFlag,                 "gci_no_ciip_constraint_flag");
    WRITE_FLAG(cinfo->noGeoConstraintFlag,                  "gci_no_gpm_constraint_flag");

    /* transform, quantization, residual */
    WRITE_FLAG(cinfo->noLumaTransformSize64ConstraintFlag,  "gci_no_luma_transform_size_64_constraint_flag");
    WRITE_FLAG(cinfo->noTransformSkipConstraintFlag,        "gci_no_transform_skip_constraint_flag");
    WRITE_FLAG(cinfo->noBDPCMConstraintFlag,                "gci_no_bdpcm_constraint_flag");
    WRITE_FLAG(cinfo->noMtsConstraintFlag,                  "gci_no_mts_constraint_flag");
    WRITE_FLAG(cinfo->noLfnstConstraintFlag,                "gci_no_lfnst_constraint_flag");
    WRITE_FLAG(cinfo->noJointCbCrConstraintFlag,            "gci_no_joint_cbcr_constraint_flag");
    WRITE_FLAG(cinfo->noSbtConstraintFlag,                  "gci_no_sbt_constraint_flag");
    WRITE_FLAG(cinfo->noActConstraintFlag,                  "gci_no_act_constraint_flag");
    WRITE_FLAG(cinfo->noExplicitScaleListConstraintFlag,    "gci_no_explicit_scaling_list_constraint_flag");
    WRITE_FLAG(cinfo->noDepQuantConstraintFlag,             "gci_no_dep_quant_constraint_flag");
    WRITE_FLAG(cinfo->noSignDataHidingConstraintFlag,       "gci_no_sign_data_hiding_constraint_flag");
    WRITE_FLAG(cinfo->noQpDeltaConstraintFlag,              "gci_no_qp_delta_constraint_flag");
    WRITE_FLAG(cinfo->noChromaQpOffsetConstraintFlag,       "gci_no_chroma_qp_offset_constraint_flag");

    /* loop filter */
    WRITE_FLAG(cinfo->noSaoConstraintFlag,                  "gci_no_sao_constraint_flag");
    WRITE_FLAG(cinfo->noAlfConstraintFlag,                  "gci_no_alf_constraint_flag");
    WRITE_FLAG(cinfo->noCCAlfConstraintFlag,                "gci_no_ccalf_constraint_flag");
    WRITE_FLAG(cinfo->noLmcsConstraintFlag,                 "gci_no_lmcs_constraint_flag");
    WRITE_FLAG(cinfo->noLadfConstraintFlag,                 "gci_no_ladf_constraint_flag");
    WRITE_FLAG(cinfo->noVirtualBoundaryConstraintFlag,      "gci_no_virtual_boundaries_constraint_flag");

    //The value of gci_num_reserved_bits shall be equal to 0 in bitstreams conforming to this version of this Specification.
    //Other values of gci_num_reserved_bits are reserved for future use by ITU-T | ISO/IEC.
    WRITE_CODE(0, 8,                                        "gci_num_reserved_bits");
  }

  while (!isByteAligned())
  {
    WRITE_FLAG(0, "gci_alignment_zero_bit");
  }
}


void  HLSWriter::codeProfileTierLevel    ( const ProfileTierLevel* ptl, bool profileTierPresent, int maxNumSubLayersMinus1 )
{
  if(profileTierPresent)
  {
    WRITE_CODE( (uint32_t)ptl->profileIdc, 7 ,        "general_profile_idc"                     );
    WRITE_FLAG( ptl->tierFlag==Level::HIGH,           "general_tier_flag"                       );
  }

  WRITE_CODE( (uint32_t)ptl->levelIdc, 8 ,            "general_level_idc");
  WRITE_FLAG( ptl->frameOnlyConstraintFlag,           "ptl_frame_only_constraint_flag" );
  WRITE_FLAG( ptl->multiLayerEnabledFlag,             "ptl_multilayer_enabled_flag"    );
  if(profileTierPresent)
  {
    codeConstraintInfo( &ptl->constraintInfo );
  }

  for (int i = maxNumSubLayersMinus1 - 1; i >= 0; i--)
  {
    WRITE_FLAG( ptl->subLayerLevelPresent[i],         "sub_layer_level_present_flag[i]" );
  }

  while (!isByteAligned())
  {
    WRITE_FLAG(0,                                     "ptl_reserved_zero_bit");
  }

  for (int i = maxNumSubLayersMinus1 - 1; i >= 0; i--)
  {
    if( ptl->subLayerLevelPresent[i] )
    {
      WRITE_CODE( (uint32_t)ptl->subLayerLevelIdc[i], 8, "sub_layer_level_idc[i]" );
    }
  }

  if (profileTierPresent)
  {
    WRITE_CODE(ptl->numSubProfile, 8, "ptl_num_sub_profiles");
    for (int i = 0; i < ptl->numSubProfile; i++)
    {
      WRITE_CODE(ptl->subProfileIdc[i], 32, "general_sub_profile_idc[i]");
    }
  }
}


/**
* Write tiles and wavefront substreams sizes for the slice header (entry points).
*
* \param pSlice Slice structure that contains the substream size information.
*/
void  HLSWriter::codeTilesWPPEntryPoint( Slice* pSlice )
{
  int numEntryPoints = pSlice->getNumEntryPoints( *pSlice->sps, *pSlice->pps );
  if( numEntryPoints == 0 )
  {
    return;
  }

  uint32_t maxOffset = 0;
  for(int idx=0; idx<pSlice->getNumberOfSubstreamSizes(); idx++)
  {
    uint32_t offset=pSlice->getSubstreamSize(idx);
    if ( offset > maxOffset )
    {
      maxOffset = offset;
    }
  }

  // Determine number of bits "offsetLenMinus1+1" required for entry point information
  uint32_t offsetLenMinus1 = 0;
  while (maxOffset >= (1u << (offsetLenMinus1 + 1)))
  {
    offsetLenMinus1++;
    CHECK(offsetLenMinus1 + 1 >= 32, "Invalid offset length minus 1");
  }

  if (pSlice->getNumberOfSubstreamSizes()>0)
  {
    WRITE_UVLC(offsetLenMinus1, "sh_entry_offset_len_minus1");

    for (uint32_t idx=0; idx<pSlice->getNumberOfSubstreamSizes(); idx++)
    {
      WRITE_CODE(pSlice->getSubstreamSize(idx)-1, offsetLenMinus1+1, "sh_entry_point_offset_minus1");
    }
  }
}


// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! Code weighted prediction tables
void HLSWriter::xCodePredWeightTable( const Slice* slice )
{
  WPScalingParam      *wp;
  const ChromaFormat  format                = slice->sps->chromaFormatIdc;
  const uint32_t      numberValidComponents = getNumberValidComponents(format);
  const bool          bChroma               = isChromaEnabled(format);
  const int           iNbRef                = (slice->sliceType == B_SLICE ) ? (2) : (1);
  bool                bDenomCoded           = false;
  uint32_t            uiTotalSignalledWeightFlags = 0;

  if ( (slice->sliceType==P_SLICE && slice->pps->weightPred) || (slice->sliceType==B_SLICE && slice->pps->weightedBiPred) )
  {
    for ( int iNumRef=0 ; iNumRef<iNbRef ; iNumRef++ ) // loop over l0 and l1 syntax elements
    {
      RefPicList  refPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

      // NOTE: wp[].log2WeightDenom and wp[].presentFlag are actually per-channel-type settings.

      for ( int iRefIdx=0 ; iRefIdx<slice->numRefIdx[ refPicList ] ; iRefIdx++ )
      {
        slice->getWpScaling(refPicList, iRefIdx, wp);
        if ( !bDenomCoded )
        {
          int iDeltaDenom;
          WRITE_UVLC( wp[COMP_Y].log2WeightDenom, "luma_log2_weight_denom" );

          if( bChroma )
          {
            CHECK( wp[COMP_Cb].log2WeightDenom != wp[COMP_Cr].log2WeightDenom, "Chroma blocks of different size not supported" );
            iDeltaDenom = (wp[COMP_Cb].log2WeightDenom - wp[COMP_Y].log2WeightDenom);
            WRITE_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );
          }
          bDenomCoded = true;
        }
        WRITE_FLAG( wp[COMP_Y].presentFlag, iNumRef==0?"luma_weight_l0_flag[i]":"luma_weight_l1_flag[i]" );
        uiTotalSignalledWeightFlags += wp[COMP_Y].presentFlag;
      }
      if (bChroma)
      {
        for ( int iRefIdx=0 ; iRefIdx<slice->numRefIdx[ refPicList ] ; iRefIdx++ )
        {
          slice->getWpScaling( refPicList, iRefIdx, wp );
          CHECK( wp[COMP_Cb].presentFlag != wp[COMP_Cr].presentFlag, "Inconsistent settings for chroma channels" );
          WRITE_FLAG( wp[COMP_Cb].presentFlag, iNumRef==0?"chroma_weight_l0_flag[i]":"chroma_weight_l1_flag[i]" );
          uiTotalSignalledWeightFlags += 2*wp[COMP_Cb].presentFlag;
        }
      }

      for ( int iRefIdx=0 ; iRefIdx<slice->numRefIdx[ refPicList ] ; iRefIdx++ )
      {
        slice->getWpScaling(refPicList, iRefIdx, wp);
        if ( wp[COMP_Y].presentFlag )
        {
          int iDeltaWeight = (wp[COMP_Y].iWeight - (1<<wp[COMP_Y].log2WeightDenom));
          WRITE_SVLC( iDeltaWeight, iNumRef==0?"delta_luma_weight_l0[i]":"delta_luma_weight_l1[i]" );
          WRITE_SVLC( wp[COMP_Y].iOffset, iNumRef==0?"luma_offset_l0[i]":"luma_offset_l1[i]" );
        }

        if ( bChroma )
        {
          if ( wp[COMP_Cb].presentFlag )
          {
            for ( int j = COMP_Cb ; j < numberValidComponents ; j++ )
            {
              CHECK(wp[COMP_Cb].log2WeightDenom != wp[COMP_Cr].log2WeightDenom, "Chroma blocks of different size not supported");
              int iDeltaWeight = (wp[j].iWeight - (1<<wp[COMP_Cb].log2WeightDenom));
              WRITE_SVLC( iDeltaWeight, iNumRef==0?"delta_chroma_weight_l0[i]":"delta_chroma_weight_l1[i]" );

              int range=slice->sps->spsRExt.highPrecisionOffsetsEnabled ? (1<<slice->sps->bitDepths[ CH_C ])/2 : 128;
              int pred = ( range - ( ( range*wp[j].iWeight)>>(wp[j].log2WeightDenom) ) );
              int iDeltaChroma = (wp[j].iOffset - pred);
              WRITE_SVLC( iDeltaChroma, iNumRef==0?"delta_chroma_offset_l0[i]":"delta_chroma_offset_l1[i]" );
            }
          }
        }
      }
    }
    CHECK(uiTotalSignalledWeightFlags>24, "Too many signalled weight flags");
  }
}


void HLSWriter::xCodePredWeightTable( const PicHeader *picHeader, const SPS *sps )
{
  WPScalingParam  *wp;
  const ChromaFormat  format                = sps->chromaFormatIdc;
  const uint32_t      numberValidComponents = getNumberValidComponents(format);
  const bool          bChroma               = isChromaEnabled(format);
  bool                bDenomCoded           = false;
  uint32_t            uiTotalSignalledWeightFlags = 0;
  uint32_t            numLxWeights                = picHeader->numL0Weights;
  bool                moreSyntaxToBeParsed        = true;
  for (int iNumRef = 0; iNumRef < NUM_REF_PIC_LIST_01 && moreSyntaxToBeParsed; iNumRef++)   // loop over l0 and l1 syntax elements
  {
    RefPicList  refPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );

    // NOTE: wp[].log2WeightDenom and wp[].presentFlag are actually per-channel-type settings.

    for ( int iRefIdx=0 ; iRefIdx<numLxWeights ; iRefIdx++ )
    {
      picHeader->getWpScaling(refPicList, iRefIdx, wp);
      if ( !bDenomCoded )
      {
        int iDeltaDenom;
        WRITE_UVLC( wp[COMP_Y].log2WeightDenom, "luma_log2_weight_denom" );

        if( bChroma )
        {
          CHECK( wp[COMP_Cb].log2WeightDenom != wp[COMP_Cr].log2WeightDenom, "Chroma blocks of different size not supported" );
          iDeltaDenom = (wp[COMP_Cb].log2WeightDenom - wp[COMP_Y].log2WeightDenom);
          WRITE_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );
        }
        bDenomCoded = true;
      }
      WRITE_FLAG( wp[COMP_Y].presentFlag, iNumRef==0?"luma_weight_l0_flag[i]":"luma_weight_l1_flag[i]" );
      uiTotalSignalledWeightFlags += wp[COMP_Y].presentFlag;
    }
    if (bChroma)
    {
      for ( int iRefIdx=0 ; iRefIdx<numLxWeights; iRefIdx++ )
      {
        picHeader->getWpScaling( refPicList, iRefIdx, wp );
        CHECK( wp[COMP_Cb].presentFlag != wp[COMP_Cr].presentFlag, "Inconsistent settings for chroma channels" );
        WRITE_FLAG( wp[COMP_Cb].presentFlag, iNumRef==0?"chroma_weight_l0_flag[i]":"chroma_weight_l1_flag[i]" );
        uiTotalSignalledWeightFlags += 2*wp[COMP_Cb].presentFlag;
      }
    }

    for ( int iRefIdx=0 ; iRefIdx<numLxWeights; iRefIdx++ )
    {
      picHeader->getWpScaling(refPicList, iRefIdx, wp);
      if ( wp[COMP_Y].presentFlag )
      {
        int iDeltaWeight = (wp[COMP_Y].iWeight - (1<<wp[COMP_Y].log2WeightDenom));
        WRITE_SVLC( iDeltaWeight, iNumRef==0?"delta_luma_weight_l0[i]":"delta_luma_weight_l1[i]" );
        WRITE_SVLC( wp[COMP_Y].iOffset, iNumRef==0?"luma_offset_l0[i]":"luma_offset_l1[i]" );
      }

      if ( bChroma )
      {
        if ( wp[COMP_Cb].presentFlag )
        {
          for ( int j = COMP_Cb ; j < numberValidComponents ; j++ )
          {
            CHECK(wp[COMP_Cb].log2WeightDenom != wp[COMP_Cr].log2WeightDenom, "Chroma blocks of different size not supported");
            int iDeltaWeight = (wp[j].iWeight - (1<<wp[COMP_Cb].log2WeightDenom));
            WRITE_SVLC( iDeltaWeight, iNumRef==0?"delta_chroma_weight_l0[i]":"delta_chroma_weight_l1[i]" );

            int range=sps->spsRExt.highPrecisionOffsetsEnabled ? (1<<sps->bitDepths[ CH_C ])/2 : 128;
            int pred = ( range - ( ( range*wp[j].iWeight)>>(wp[j].log2WeightDenom) ) );
            int iDeltaChroma = (wp[j].iOffset - pred);
            WRITE_SVLC( iDeltaChroma, iNumRef==0?"delta_chroma_offset_l0[i]":"delta_chroma_offset_l1[i]" );
          }
        }
      }
    }
    if (iNumRef == 0)
    {
      numLxWeights         = picHeader->numL1Weights;
      if (picHeader->pRPL[1]->getNumRefEntries() > 0)
      {
        WRITE_UVLC(numLxWeights, "num_l1_weights");
      }
      moreSyntaxToBeParsed = (numLxWeights == 0) ? false : true;
    }
  }
  CHECK(uiTotalSignalledWeightFlags>24, "Too many signalled weight flags");
}

void HLSWriter::alfFilter( const AlfParam& alfParam, const bool isChroma, const int altIdx )
{
  AlfFilterShape alfShape(isChroma ? 5 : 7);
  const short* coeff = isChroma ? alfParam.chromaCoeff[altIdx] : alfParam.lumaCoeff;
  const short* clipp = isChroma ? alfParam.chromaClipp[altIdx] : alfParam.lumaClipp;
  const int numFilters = isChroma ? 1 : alfParam.numLumaFilters;

  // vlc for all

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      WRITE_UVLC( abs(coeff[ ind* MAX_NUM_ALF_LUMA_COEFF + i ]), isChroma ? "alf_chroma_coeff_abs" : "alf_luma_coeff_abs" ); //alf_coeff_chroma[i], alf_coeff_luma_delta[i][j]
      if( abs( coeff[ ind* MAX_NUM_ALF_LUMA_COEFF + i ] ) != 0 )
      {
        WRITE_FLAG( ( coeff[ ind* MAX_NUM_ALF_LUMA_COEFF + i ] < 0 ) ? 1 : 0, isChroma ? "alf_chroma_coeff_sign" : "alf_luma_coeff_sign" );
      }
    }
  }

  // Clipping values coding
  if( alfParam.nonLinearFlag[isChroma] )
  {
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        WRITE_CODE(clipp[ind* MAX_NUM_ALF_LUMA_COEFF + i], 2, isChroma ? "alf_chroma_clip_idx" : "alf_luma_clip_idx");
      }
    }
  }
}

} // namespace vvenc

//! \}


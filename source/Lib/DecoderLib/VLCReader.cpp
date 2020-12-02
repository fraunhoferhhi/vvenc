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


/** \file     VLCWReader.cpp
 *  \brief    Reader for high level syntax
 */

#include "VLCReader.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/dtrace_next.h"

//! \ingroup DecoderLib
//! \{

namespace vvenc {

#if ENABLE_TRACING

void  VLCReader::xReadCodeTr(uint32_t length, uint32_t& rValue, const char *pSymbolName)
{
  xReadCode (length, rValue);
  if (length < 10)
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d)  : %u\n", pSymbolName, length, rValue );
  }
  else
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s u(%d) : %u\n", pSymbolName, length, rValue );
  }
}

void  VLCReader::xReadUvlcTr(uint32_t& rValue, const char *pSymbolName)
{
  xReadUvlc (rValue);
  DTRACE( g_trace_ctx, D_HEADER, "%-50s ue(v) : %u\n", pSymbolName, rValue );
}

void  VLCReader::xReadSvlcTr(int& rValue, const char *pSymbolName)
{
  xReadSvlc (rValue);
  DTRACE( g_trace_ctx, D_HEADER, "%-50s se(v) : %d\n", pSymbolName, rValue );
}

void  VLCReader::xReadFlagTr(uint32_t& rValue, const char *pSymbolName)
{
  xReadFlag (rValue);
  DTRACE( g_trace_ctx, D_HEADER, "%-50s u(1)  : %d\n", pSymbolName, rValue );
}

void  VLCReader::xReadFlagTr(bool& rValue, const char *pSymbolName)
{
  xReadFlag (rValue);
  DTRACE( g_trace_ctx, D_HEADER, "%-50s u(1)  : %d\n", pSymbolName, rValue?1:0 );
}

void xTraceFillerData ()
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Filler Data ===========\n");
}

void VLCReader::xReadSCode (uint32_t length, int& value, const char *pSymbolName)
{
  xReadSCode( length, value);

  if (length < 10)
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s i(%d)  : %d\n", pSymbolName, length, value );
  }
  else
  {
    DTRACE( g_trace_ctx, D_HEADER, "%-50s i(%d) : %d\n", pSymbolName, length, value );
  }
}

#endif

void VLCReader::xReadSCode (uint32_t length, int& value)
{
  uint32_t val;
  assert ( length > 0 && length<=32);
  m_pcBitstream->read (length, val);
  value= length>=32 ? int(val) : ( (-int( val & (uint32_t(1)<<(length-1)))) | int(val) );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================
void VLCReader::xReadCode (uint32_t uiLength, uint32_t& ruiCode)
{
  CHECK( uiLength == 0, "Reading a code of lenght '0'" );
  m_pcBitstream->read (uiLength, ruiCode);
}

void VLCReader::xReadUvlc( uint32_t& ruiVal)
{
  uint32_t uiVal = 0;
  uint32_t uiCode = 0;
  uint32_t uiLength;
  m_pcBitstream->read( 1, uiCode );

  if( 0 == uiCode )
  {
    uiLength = 0;

    while( ! ( uiCode & 1 ))
    {
      m_pcBitstream->read( 1, uiCode );
      uiLength++;
    }

    m_pcBitstream->read( uiLength, uiVal );

    uiVal += (1 << uiLength)-1;
  }

  ruiVal = uiVal;
}

void VLCReader::xReadSvlc( int& riVal)
{
  uint32_t uiBits = 0;
  m_pcBitstream->read( 1, uiBits );
  if( 0 == uiBits )
  {
    uint32_t uiLength = 0;

    while( ! ( uiBits & 1 ))
    {
      m_pcBitstream->read( 1, uiBits );
      uiLength++;
    }

    m_pcBitstream->read( uiLength, uiBits );

    uiBits += (1 << uiLength);
    riVal = ( uiBits & 1) ? -(int)(uiBits>>1) : (int)(uiBits>>1);
  }
  else
  {
    riVal = 0;
  }
}

void VLCReader::xReadFlag (uint32_t& ruiCode)
{
  m_pcBitstream->read( 1, ruiCode );
}

void VLCReader::xReadFlag (bool& ruiCode)
{
  uint32_t uiCode;
  m_pcBitstream->read( 1, uiCode );
  ruiCode = uiCode!=0;
}

void VLCReader::xReadRbspTrailingBits()
{
  uint32_t bit;
  READ_FLAG( bit, "rbsp_stop_one_bit");
  CHECK(bit!=1, "Trailing bit not '1'");
  int cnt = 0;
  while (m_pcBitstream->getNumBitsUntilByteAligned())
  {
    READ_FLAG( bit, "rbsp_alignment_zero_bit");
    CHECK(bit!=0, "Alignment bit is not '0'");
    cnt++;
  }
  CHECK(cnt >= 8, "Read more than '8' trailing bits");
}

void AUDReader::parseAccessUnitDelimiter(InputBitstream* bs, uint32_t &audIrapOrGdrAuFlag, uint32_t &picType)
{
  setBitstream(bs);

  DTRACE( g_trace_ctx, D_HEADER, "=========== Access Unit Delimiter ===========\n" );

  READ_FLAG (audIrapOrGdrAuFlag, "aud_irap_or_gdr_au_flag");
  READ_CODE (3, picType, "pic_type");
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseFillerData(InputBitstream* bs, uint32_t &fdSize)
{
  setBitstream(bs);
#if ENABLE_TRACING
  xTraceFillerData();
#endif
  uint32_t ffByte;
  fdSize = 0;
  while( m_pcBitstream->getNumBitsLeft() >8 )
  {
    READ_CODE (8, ffByte, "ff_byte");
    CHECK(ffByte!=0xff, "Invalid filler data : not '0xff'");
    fdSize++;
  }
  xReadRbspTrailingBits();
}

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

HLSyntaxReader::HLSyntaxReader()
{
}

HLSyntaxReader::~HLSyntaxReader()
{

}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void HLSyntaxReader::copyRefPicList(SPS* sps, ReferencePictureList* source_rpl, ReferencePictureList* dest_rp)
{
  dest_rp->numberOfShorttermPictures = source_rpl->numberOfShorttermPictures;

  if (sps->longTermRefsPresent)
    dest_rp->numberOfLongtermPictures = dest_rp->numberOfLongtermPictures;
  else
    dest_rp->numberOfLongtermPictures = 0;

  uint32_t numRefPic = dest_rp->numberOfShorttermPictures + dest_rp->numberOfLongtermPictures;
  for (int ii = 0; ii < numRefPic; ii++)
    dest_rp->setRefPicIdentifier(ii, source_rpl->refPicIdentifier[ii], source_rpl->isLongtermRefPic[ii], source_rpl->isInterLayerRefPic[ii], source_rpl->interLayerRefPicIdx[ii]);
}

void HLSyntaxReader::parseRefPicList(SPS* sps, ReferencePictureList* rpl, int rplIdx)
{
  *rpl = ReferencePictureList();
  
  uint32_t code;
  READ_UVLC(code, "num_ref_entries[ listIdx ][ rplsIdx ]");
  uint32_t numRefPic = code;
  uint32_t numStrp = 0;
  uint32_t numLtrp = 0;
  uint32_t numIlrp = 0;

  if (sps->longTermRefsPresent && rplIdx != -1)
  {
    READ_FLAG(rpl->ltrpInSliceHeader, "ltrp_in_slice_header_flag[ listIdx ][ rplsIdx ]");
  }
  else if(sps->longTermRefsPresent)
  {
    rpl->ltrpInSliceHeader = true;
  }

  bool isLongTerm;
  int prevDelta = MAX_INT;
  int deltaValue = 0;
  bool firstSTRP = true;

  rpl->interLayerPresent =  sps->interLayerPresent;

  for (int ii = 0; ii < numRefPic; ii++)
  {
    uint32_t isInterLayerRefPic = 0;

    if( rpl->interLayerPresent )
    {
      READ_FLAG( isInterLayerRefPic, "inter_layer_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]" );

      if( isInterLayerRefPic )
      {
        READ_UVLC( code, "ilrp_idx[ listIdx ][ rplsIdx ][ i ]" );
        rpl->setRefPicIdentifier( ii, 0, true, true, code );
        numIlrp++;
      }
    }

    if( !isInterLayerRefPic )
    {
    isLongTerm = false;
    if (sps->longTermRefsPresent)
    {
      READ_FLAG(code, "st_ref_pic_flag[ listIdx ][ rplsIdx ][ i ]");
      isLongTerm = (code == 1) ? false : true;
    }
    else
      isLongTerm = false;

    if (!isLongTerm)
    {
      READ_UVLC(code, "abs_delta_poc_st[ listIdx ][ rplsIdx ][ i ]");
      if(( !sps->weightPred && !sps->weightedBiPred ) || (ii==0))
      {
        code++;
      }
      int readValue = code;
      if (readValue > 0)
      {
        READ_FLAG(code, "strp_entry_sign_flag[ listIdx ][ rplsIdx ][ i ]");
        if (code)
        {
          readValue = -readValue;
        }
      }
      if (firstSTRP)
      {
        firstSTRP = false;
        prevDelta = deltaValue = readValue;
      }
      else
      {
        deltaValue = prevDelta + readValue;
        prevDelta = deltaValue;
      }
      rpl->setRefPicIdentifier(ii, deltaValue, isLongTerm, false, 0);
      numStrp++;
    } 
    else
    {
      if (!rpl->ltrpInSliceHeader)
        READ_CODE(sps->bitsForPOC, code, "poc_lsb_lt[listIdx][rplsIdx][i]");
      rpl->setRefPicIdentifier(ii, code, isLongTerm, false, 0);
      numLtrp++;
    }
  }
}
  rpl->numberOfShorttermPictures = numStrp;
  rpl->numberOfLongtermPictures = numLtrp;
  rpl->numberOfInterLayerPictures = numIlrp;
}

void HLSyntaxReader::parsePPS( PPS* pcPPS, ParameterSetManager *parameterSetManager )
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Picture Parameter Set  ===========\n" );
  uint32_t  uiCode;

  int   iCode;

  READ_CODE( 6, uiCode, "pps_pic_parameter_set_id");
  CHECK(uiCode > 63, "PPS id exceeds boundary (63)");
  pcPPS->ppsId = (uiCode);

  READ_CODE( 4, uiCode, "pps_seq_parameter_set_id");
  pcPPS->spsId = (uiCode);

  READ_FLAG( pcPPS->mixedNaluTypesInPic, "pps_mixed_nalu_types_in_pic_flag" );

  READ_UVLC( pcPPS->picWidthInLumaSamples, "pps_pic_width_in_luma_samples" );
  READ_UVLC( pcPPS->picHeightInLumaSamples, "pps_pic_height_in_luma_samples" );

  READ_FLAG( uiCode, "pps_conformance_window_flag" );

  const SPS* pcSPS = parameterSetManager->getSPS(pcPPS->spsId);
  if( uiCode != 0 )
  {
    //th assume that the sps is there and its the final one  
    Window &conf = pcPPS->conformanceWindow;
    READ_UVLC(   uiCode, "pps_conf_win_left_offset" );               conf.winLeftOffset  = ( uiCode * SPS::getWinUnitX( pcSPS->chromaFormatIdc ) );
    READ_UVLC(   uiCode, "pps_conf_win_right_offset" );              conf.winRightOffset = ( uiCode * SPS::getWinUnitX( pcSPS->chromaFormatIdc ) );
    READ_UVLC(   uiCode, "pps_conf_win_top_offset" );                conf.winTopOffset   = ( uiCode * SPS::getWinUnitY( pcSPS->chromaFormatIdc ) );
    READ_UVLC(   uiCode, "pps_conf_win_bottom_offset" );             conf.winBottomOffset= ( uiCode * SPS::getWinUnitY( pcSPS->chromaFormatIdc ) );
    conf.enabledFlag = true;
  }

  if( pcPPS->picWidthInLumaSamples == pcSPS->maxPicWidthInLumaSamples && pcPPS->picHeightInLumaSamples == pcSPS->maxPicHeightInLumaSamples )
  {
    CHECK( pcPPS->conformanceWindow.enabledFlag, "When pic_width_in_luma_samples is equal to pic_width_max_in_luma_samples and pic_height_in_luma_samples is equal to pic_height_max_in_luma_samples, the value of pps_conformance_window_flag shall be equal to 0");
    pcPPS->conformanceWindow = pcSPS->conformanceWindow;
  }

  READ_FLAG( uiCode, "spps_caling_window_flag" );
  if( uiCode != 0 )
  {
    //th assume that the sps is there and its the final one  
    Window &conf = pcPPS->scalingWindow;
    int iCode;
    READ_SVLC(   iCode, "pps_scaling_win_left_offset" );               conf.winLeftOffset  = ( iCode * SPS::getWinUnitX( pcSPS->chromaFormatIdc ) );
    READ_SVLC(   iCode, "pps_scaling_win_right_offset" );              conf.winRightOffset = ( iCode * SPS::getWinUnitX( pcSPS->chromaFormatIdc ) );
    READ_SVLC(   iCode, "pps_scaling_win_top_offset" );                conf.winTopOffset   = ( iCode * SPS::getWinUnitY( pcSPS->chromaFormatIdc ) );
    READ_SVLC(   iCode, "pps_scaling_win_bottom_offset" );             conf.winBottomOffset= ( iCode * SPS::getWinUnitY( pcSPS->chromaFormatIdc ) );
    conf.enabledFlag = true;
  }
  else
  {
    pcPPS->scalingWindow = pcPPS->conformanceWindow;
  }

  READ_FLAG( pcPPS->outputFlagPresent, "pps_output_flag_present_flag" );

  READ_FLAG( pcPPS->noPicPartition, "pps_no_pic_partition_flag");    
  READ_FLAG( pcPPS->subPicIdMappingInPps, "pps_subpic_id_mapping_in_pps_flag" );
  if( pcPPS->subPicIdMappingInPps )
  {
    if( !pcPPS->noPicPartition )
    {
      READ_UVLC( uiCode, "pps_num_subpics_minus1" ); pcPPS->numSubPics = uiCode + 1;
      CHECK( uiCode + 1 > MAX_NUM_SUB_PICS - 1, "Number of sub-pictures exceeds limit" );
    }
    else
    {
      pcPPS->numSubPics = (1);
    }

    READ_UVLC( pcPPS->subPicIdLen, "pps_subpic_id_len_minus1" );
    CHECK( pcPPS->subPicIdLen > 15, "Invalid pps_subpic_id_len_minus1 signalled");

    CHECK((1 << pcPPS->subPicIdLen) < pcPPS->numSubPics, "pps_subpic_id_len exceeds valid range");
    for( int picIdx = 0; picIdx < pcPPS->numSubPics; picIdx++ )
    {
      READ_CODE( pcPPS->subPicId[picIdx], uiCode, "pps_subpic_id[i]" );
    }
  }
  else 
  {
    for( int picIdx = 0; picIdx < MAX_NUM_SUB_PICS; picIdx++ )
    {
      pcPPS->subPicId[picIdx] = picIdx;
    }
  }

  if(!pcPPS->noPicPartition)
  {
    THROW("no support");
  }

  READ_FLAG( pcPPS->cabacInitPresent,   "pps_cabac_init_present_flag" );

  READ_UVLC(uiCode, "pps_num_ref_idx_l0_default_active_minus1");
  CHECK(uiCode > 14, "Invalid code read");
  pcPPS->numRefIdxL0DefaultActive = (uiCode+1);

  READ_UVLC(uiCode, "pps_num_ref_idx_l1_default_active_minus1");
  CHECK(uiCode > 14, "Invalid code read");
  pcPPS->numRefIdxL1DefaultActive = (uiCode+1);

  READ_FLAG(pcPPS->rpl1IdxPresent, "pps_rpl1_idx_present_flag");
  READ_FLAG( pcPPS->weightPred, "pps_weighted_pred_flag" );          // Use of Weighting Prediction (P_SLICE)
  READ_FLAG( pcPPS->weightedBiPred, "pps_weighted_bipred_flag" );         // Use of Bi-Directional Weighting Prediction (B_SLICE)
  READ_FLAG(pcPPS->wrapAroundEnabled, "pps_ref_wraparound_enabled_flag");
  if (pcPPS->wrapAroundEnabled)
  {
    READ_UVLC(uiCode, "pps_ref_wraparound_offset");               
    pcPPS->wrapAroundOffset = (uiCode);
  }

  READ_SVLC(pcPPS->picInitQPMinus26, "pps_init_qp_minus26" );                            
  
  READ_FLAG( pcPPS->useDQP, "pps_cu_qp_delta_enabled_flag" );
  READ_FLAG(pcPPS->usePPSChromaTool, "pps_chroma_tool_offsets_present_flag");
  if (pcPPS->usePPSChromaTool)
  {
    READ_SVLC( iCode, "pps_cb_qp_offset");
    CHECK( iCode < -12, "Invalid Cb QP offset" );
    CHECK( iCode >  12, "Invalid Cb QP offset" );
    pcPPS->chromaQpOffset[COMP_Cb] = iCode;

    READ_SVLC( iCode, "pps_cr_qp_offset");
    CHECK( iCode < -12, "Invalid Cr QP offset" );
    CHECK( iCode >  12, "Invalid Cr QP offset" );
    pcPPS->chromaQpOffset[COMP_Cr] = iCode;

    READ_FLAG(pcPPS->jointCbCrQpOffsetPresent, "pps_joint_cbcr_qp_offset_present_flag");

    if(pcPPS->jointCbCrQpOffsetPresent)
    {
      READ_SVLC(iCode, "pps_joint_cbcr_qp_offset_value");
    }
    else
    {
      iCode = 0;
    }
    CHECK( iCode < -12, "Invalid CbCr QP offset" );
    CHECK( iCode >  12, "Invalid CbCr QP offset" );
    pcPPS->chromaQpOffset[COMP_JOINT_CbCr] = iCode;

    CHECK(MAX_NUM_COMP>3, "Invalid maximal number of components");

    READ_FLAG( pcPPS->sliceChromaQpFlag, "pps_slice_chroma_qp_offsets_present_flag" );

    READ_FLAG( uiCode, "pps_cu_chroma_qp_offset_list_enabled_flag");
    if (uiCode != 0)
    {
      uint32_t tableSizeMinus1 = 0;
      READ_UVLC(tableSizeMinus1, "pps_chroma_qp_offset_list_len_minus1");
      CHECK(tableSizeMinus1 >= MAX_QP_OFFSET_LIST_SIZE, "Table size exceeds maximum");

      for (int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx <= (tableSizeMinus1); cuChromaQpOffsetIdx++)
      {
        int cbOffset;
        int crOffset;
        int jointCbCrOffset = 0;
        READ_SVLC(cbOffset, "pps_cb_qp_offset_list[i]");
        CHECK(cbOffset < -12 || cbOffset > 12, "Invalid chroma QP offset");
        READ_SVLC(crOffset, "pps_cr_qp_offset_list[i]");
        CHECK(crOffset < -12 || crOffset > 12, "Invalid chroma QP offset");
        if (pcPPS->jointCbCrQpOffsetPresent )
        {
          READ_SVLC(jointCbCrOffset, "pps_joint_cbcr_qp_offset_list[i]");
        }
        CHECK(jointCbCrOffset < -12 || jointCbCrOffset > 12, "Invalid chroma QP offset");
        // table uses +1 for index (see comment inside the function)
        pcPPS->setChromaQpOffsetListEntry(cuChromaQpOffsetIdx + 1, cbOffset, crOffset, jointCbCrOffset);
      }
      CHECK(pcPPS->chromaQpOffsetListLen != tableSizeMinus1 + 1, "Invalid chroma QP offset list length");
    }
  }
  READ_FLAG( pcPPS->deblockingFilterControlPresent, "pps_deblocking_filter_control_present_flag" );
  if(pcPPS->deblockingFilterControlPresent)
  {
    READ_FLAG( pcPPS->deblockingFilterOverrideEnabled, "deblocking_filter_override_enabled_flag" );
    READ_FLAG( pcPPS->deblockingFilterDisabled, "pps_deblocking_filter_disabled_flag" );
    if (!pcPPS->noPicPartition && pcPPS->deblockingFilterOverrideEnabled)
    {
      READ_FLAG(pcPPS->dbfInfoInPh, "pps_dbf_info_in_ph_flag");
    }
    if( ! pcPPS->deblockingFilterDisabled)
    {
      READ_SVLC( pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Y], "pps_beta_offset_div2" );
      CHECK(  pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Y] < -12 ||
              pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Y] > 12, "Invalid deblocking filter configuration" );

      READ_SVLC( pcPPS->deblockingFilterTcOffsetDiv2[COMP_Y], "pps_tc_offset_div2");
      CHECK(  pcPPS->deblockingFilterTcOffsetDiv2[COMP_Y] < -12 ||
              pcPPS->deblockingFilterTcOffsetDiv2[COMP_Y] > 12, "Invalid deblocking filter configuration" );
      if( pcPPS->usePPSChromaTool )
      {
      READ_SVLC( pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Cb], "pps_cb_beta_offset_div2");
      CHECK(  pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Cb] < -12 ||
              pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Cb] > 12, "Invalid deblocking filter configuration" );

      READ_SVLC( pcPPS->deblockingFilterTcOffsetDiv2[COMP_Cb], "pps_cb_tc_offset_div2");
      CHECK(  pcPPS->deblockingFilterTcOffsetDiv2[COMP_Cb] < -12 ||
              pcPPS->deblockingFilterTcOffsetDiv2[COMP_Cb] > 12, "Invalid deblocking filter configuration" );

      READ_SVLC( pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Cr], "pps_cr_beta_offset_div2");
      CHECK(  pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Cr] < -12 ||
              pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Cr] > 12, "Invalid deblocking filter configuration" );

      READ_SVLC( pcPPS->deblockingFilterTcOffsetDiv2[COMP_Cr], "pps_cr_tc_offset_div2");
      CHECK(  pcPPS->deblockingFilterTcOffsetDiv2[COMP_Cr] < -12 ||
              pcPPS->deblockingFilterTcOffsetDiv2[COMP_Cr] > 12, "Invalid deblocking filter configuration" );
    }
    else
      {
        pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Cb]= pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Y];
        pcPPS->deblockingFilterTcOffsetDiv2[COMP_Cb]  = pcPPS->deblockingFilterTcOffsetDiv2[COMP_Y];
        pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Cr]= pcPPS->deblockingFilterBetaOffsetDiv2[COMP_Y];
        pcPPS->deblockingFilterTcOffsetDiv2[COMP_Cr]  = pcPPS->deblockingFilterTcOffsetDiv2[COMP_Y];
      }
    }
  }
  if (!pcPPS->noPicPartition)
  {
    READ_FLAG(pcPPS->rplInfoInPh, "pps_rpl_info_in_ph_flag");
    READ_FLAG(pcPPS->saoInfoInPh, "pps_sao_info_in_ph_flag");
    READ_FLAG(pcPPS->alfInfoInPh, "pps_alf_info_in_ph_flag");
    if ((pcPPS->weightPred || pcPPS->weightedBiPred) && pcPPS->rplInfoInPh)
    {
      READ_FLAG(pcPPS->wpInfoInPh, "pps_wp_info_in_ph_flag");
    }
    READ_FLAG(pcPPS->qpDeltaInfoInPh, "pps_qp_delta_info_in_ph_flag");
  }

  READ_FLAG( pcPPS->pictureHeaderExtensionPresent, "pps_picture_header_extension_present_flag");

  READ_FLAG( pcPPS->sliceHeaderExtensionPresent, "pps_slice_header_extension_present_flag");
  
  READ_FLAG( uiCode, "pps_extension_present_flag");
  if (uiCode)
  {
    while ( xMoreRbspData() )
    {
      READ_FLAG( uiCode, "pps_extension_data_flag");
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseAPS( APS* aps )
{
  DTRACE(g_trace_ctx, D_HEADER, "=========== Adaptation Parameter Set  ===========\n");

  bool flag;

  READ_CODE(3, aps->apsType, "aps_params_type");

  READ_CODE(5, aps->apsId, "adaptation_parameter_set_id");

  uint32_t codeApsChromaPresent;
  READ_FLAG(codeApsChromaPresent, "aps_chroma_present_flag");
  aps->chromaPresent = codeApsChromaPresent;


  if (aps->apsType == ALF_APS)
  {
    parseAlfAps( aps );
  }
  else if (aps->apsType == LMCS_APS)
  {
    parseLmcsAps( aps );
  }
  READ_FLAG(flag, "aps_extension_flag");
  if (flag)
  {
    while (xMoreRbspData())
    {
      READ_FLAG(flag, "aps_extension_data_flag");
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseAlfAps( APS* aps )
{
  AlfParam& alfp = aps->alfParam;
  uint32_t  code;
  alfp.reset();
  alfp.alfEnabled[COMP_Y] = alfp.alfEnabled[COMP_Cb] = alfp.alfEnabled[COMP_Cr] = true;
  READ_FLAG(code, "alf_luma_new_filter");
  alfp.newFilterFlag[CH_L] = code;
  CcAlfFilterParam& ccAlfParam = aps->ccAlfParam;
  if (aps->chromaPresent)
  {
    READ_FLAG(code, "alf_chroma_new_filter");
    alfp.newFilterFlag[CH_C] = code;

    READ_FLAG(code, "alf_cc_cb_filter_signal_flag");
    ccAlfParam.newCcAlfFilter[COMP_Cb - 1] = code;
    READ_FLAG(code, "alf_cc_cr_filter_signal_flag");
    ccAlfParam.newCcAlfFilter[COMP_Cr - 1] = code;
  }
  CHECK(alfp.newFilterFlag[CH_L] == 0 && alfp.newFilterFlag[CH_C] == 0
          && ccAlfParam.newCcAlfFilter[COMP_Cb - 1] == 0 && ccAlfParam.newCcAlfFilter[COMP_Cr - 1] == 0,
        "bitstream conformance error: one of alf_luma_filter_signal_flag, alf_chroma_filter_signal_flag, "
        "alf_cross_component_cb_filter_signal_flag, and alf_cross_component_cr_filter_signal_flag shall be nonzero");

  if (alfp.newFilterFlag[CH_L])
  {
    READ_FLAG(code, "alf_luma_clip");
    alfp.nonLinearFlag[CH_L] = code ? true : false;
    READ_UVLC(code, "alf_luma_num_filters_signalled_minus1");
    alfp.numLumaFilters = code + 1;
    if (alfp.numLumaFilters > 1)
    {
      const int length = ceilLog2(alfp.numLumaFilters);
      for (int i = 0; i < MAX_NUM_ALF_CLASSES; i++)
      {
        READ_CODE(length, code, "alf_luma_coeff_delta_idx");
        alfp.filterCoeffDeltaIdx[i] = code;
      }
    }
    else
    {
      memset(alfp.filterCoeffDeltaIdx, 0, sizeof(alfp.filterCoeffDeltaIdx));
    }
    alfFilter( alfp, false, 0 );
  }
  if (alfp.newFilterFlag[CH_C])
  {
    READ_FLAG(alfp.nonLinearFlag[CH_C], "alf_nonlinear_enable_flag_chroma");

    if( MAX_NUM_ALF_ALTERNATIVES_CHROMA > 1 )
      READ_UVLC( code, "alf_chroma_num_alts_minus1" );
    else
      code = 0;

    alfp.numAlternativesChroma = code + 1;

    for( int altIdx=0; altIdx < alfp.numAlternativesChroma; ++altIdx )
    {
      alfFilter( alfp, true, altIdx );
    }
  }
  for (int ccIdx = 0; ccIdx < 2; ccIdx++)
  {
    if (ccAlfParam.newCcAlfFilter[ccIdx])
    {
      if (MAX_NUM_CC_ALF_FILTERS > 1)
      {
        READ_UVLC(code, ccIdx == 0 ? "alf_cc_cb_filters_signalled_minus1" : "alf_cc_cr_filters_signalled_minus1");
      }
      else
      {
        code = 0;
      }
      ccAlfParam.ccAlfFilterCount[ccIdx] = code + 1;

      for (int filterIdx = 0; filterIdx < ccAlfParam.ccAlfFilterCount[ccIdx]; filterIdx++)
      {
        ccAlfParam.ccAlfFilterIdxEnabled[ccIdx][filterIdx] = true;
        AlfFilterShape alfShape(size_CC_ALF);

        short *coeff = ccAlfParam.ccAlfCoeff[ccIdx][filterIdx];
        // Filter coefficients
        for (int i = 0; i < alfShape.numCoeff - 1; i++)
        {
          READ_CODE(CCALF_BITS_PER_COEFF_LEVEL, code,
                    ccIdx == 0 ? "alf_cc_cb_mapped_coeff_abs" : "alf_cc_cr_mapped_coeff_abs");
          if (code == 0)
          {
            coeff[i] = 0;
          }
          else
          {
            coeff[i] = 1 << (code - 1);
            READ_FLAG(code, ccIdx == 0 ? "alf_cc_cb_coeff_sign" : "alf_cc_cr_coeff_sign");
            coeff[i] *= 1 - 2 * code;
          }
        }

        DTRACE(g_trace_ctx, D_SYNTAX, "%s coeff filterIdx %d: ", ccIdx == 0 ? "Cb" : "Cr", filterIdx);
        for (int i = 0; i < alfShape.numCoeff; i++)
        {
          DTRACE(g_trace_ctx, D_SYNTAX, "%d ", coeff[i]);
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "\n");
      }

      for (int filterIdx = ccAlfParam.ccAlfFilterCount[ccIdx]; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        ccAlfParam.ccAlfFilterIdxEnabled[ccIdx][filterIdx] = false;
      }
    }
  }
}

void HLSyntaxReader::parseLmcsAps( APS* aps )
{
  LmcsParam& info = aps->lmcsParam;
  uint32_t  code;

  memset(info.reshaperModelBinCWDelta, 0, PIC_CODE_CW_BINS * sizeof(int));
  READ_UVLC(code, "lmcs_min_bin_idx");                             info.reshaperModelMinBinIdx = code;
  READ_UVLC(code, "lmcs_delta_max_bin_idx");                       info.reshaperModelMaxBinIdx = PIC_CODE_CW_BINS - 1 - code;
  READ_UVLC(code, "lmcs_delta_cw_prec_minus1");                    info.maxNbitsNeededDeltaCW = code + 1;
  assert(info.maxNbitsNeededDeltaCW > 0);
  for (uint32_t i = info.reshaperModelMinBinIdx; i <= info.reshaperModelMaxBinIdx; i++)
  {
    READ_CODE(info.maxNbitsNeededDeltaCW, code, "lmcs_delta_abs_cw[ i ]");
    int absCW = code;
    if (absCW > 0)
    {
      READ_CODE(1, code, "lmcs_delta_sign_cw_flag[ i ]");
    }
    int signCW = code;
    info.reshaperModelBinCWDelta[i] = (1 - 2 * signCW) * absCW;
  }

  if (aps->chromaPresent)
  {
    READ_CODE(3, code, "lmcs_delta_abs_crs");
  }
  int absCW = aps->chromaPresent ? code : 0;
  if (absCW > 0)
  {
    READ_CODE(1, code, "lmcs_delta_sign_crs_flag");
  }
  int signCW = code;
  info.chrResScalingOffset = (1 - 2 * signCW) * absCW;
}

void  HLSyntaxReader::parseVUI(VUI* pcVUI, SPS *pcSPS)
{
  assert(0); //to be checked
#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_HEADER, "----------- vui_parameters -----------\n");
#endif
  READ_FLAG(pcVUI->progressiveSourceFlag,  "vui_progressive_source_flag" ); 
  READ_FLAG(pcVUI->interlacedSourceFlag,  "vui_interlaced_source_flag" ); 
  READ_FLAG(pcVUI->nonPackedFlag, "vui_non_packed_constraint_flag");
  READ_FLAG(pcVUI->nonProjectedFlag, "vui_non_projected_constraint_flag");
  READ_FLAG( pcVUI->aspectRatioInfoPresent, "vui_aspect_ratio_info_present_flag");
  if (pcVUI->aspectRatioInfoPresent)
  {
    READ_CODE(8, pcVUI->aspectRatioIdc, "vui_aspect_ratio_idc");
    if (pcVUI->aspectRatioIdc == 255)
    {
      READ_CODE(16, pcVUI->sarWidth, "vui_sar_width");
      READ_CODE(16, pcVUI->sarHeight, "vui_sar_height");
    }
  }

  READ_FLAG(pcVUI->overscanInfoPresent, "vui_overscan_info_present_flag");
  if (pcVUI->overscanInfoPresent)
  {
    READ_FLAG(pcVUI->overscanAppropriateFlag, "vui_overscan_appropriate_flag");
  }
  READ_FLAG(pcVUI->colourDescriptionPresent, "vui_colour_description_present_flag");
  if (pcVUI->colourDescriptionPresent)
  {
    READ_CODE(8, pcVUI->colourPrimaries, "vui_colour_primaries");
    READ_CODE(8, pcVUI->transferCharacteristics, "vui_transfer_characteristics");
    READ_CODE(8, pcVUI->matrixCoefficients, "vui_matrix_coeffs");
    READ_FLAG(   pcVUI->videoFullRangeFlag, "vui_full_range_flag");
  }

  READ_FLAG( pcVUI->chromaLocInfoPresent, "vui_chroma_loc_info_present_flag");
  if (pcVUI->chromaLocInfoPresent)
  {
    if(pcVUI->progressiveSourceFlag && !pcVUI->interlacedSourceFlag)
    {
      READ_UVLC( pcVUI->chromaSampleLocType, "vui_chroma_sample_loc_type" );
    }
    else
    {
      READ_UVLC( pcVUI->chromaSampleLocTypeTopField, "vui_chroma_sample_loc_type_top_field" );
      READ_UVLC( pcVUI->chromaSampleLocTypeBottomField, "vui_chroma_sample_loc_type_bottom_field" );
    }
  }
}

void HLSyntaxReader::parseGeneralHrdParameters(GeneralHrdParams *hrd)
{
  uint32_t  symbol;
  READ_CODE( 32, symbol, "num_units_in_tick");                          hrd->numUnitsInTick = symbol;
  READ_CODE( 32, symbol, "time_scale");                                 hrd->timeScale = symbol;
  READ_FLAG( symbol, "general_nal_hrd_parameters_present_flag");        hrd->generalNalHrdParamsPresent = (symbol == 1);
  READ_FLAG( symbol, "general_vcl_hrd_parameters_present_flag");        hrd->generalVclHrdParamsPresent = (symbol == 1);
  if(  hrd->generalNalHrdParamsPresent || hrd->generalVclHrdParamsPresent )
  {
    READ_FLAG( symbol, "general_same_pic_timing_in_all_ols_flag");        hrd->generalSamePicTimingInAllOlsFlag = (symbol == 1);
    READ_FLAG( symbol, "general_decoding_unit_hrd_params_present_flag");  hrd->generalDecodingUnitHrdParamsPresent = (symbol == 1);
    if (hrd->generalDecodingUnitHrdParamsPresent)
    {
      READ_CODE( 8, symbol, "tick_divisor_minus2");                       hrd->tickDivisorMinus2 = symbol;
    }
    READ_CODE( 4, symbol, "bit_rate_scale");                              hrd->bitRateScale = symbol;
    READ_CODE( 4, symbol, "cpb_size_scale");                              hrd->cpbSizeScale = symbol;
    if (hrd->generalDecodingUnitHrdParamsPresent)
    {
      READ_CODE( 4, symbol, "cpb_size_du_scale");                         hrd->cpbSizeDuScale = symbol;
    }
    READ_UVLC( symbol, "hrd_cpb_cnt_minus1");                             hrd->hrdCpbCntMinus1 = symbol;
    CHECK(symbol > 31,"The value of hrd_cpb_cnt_minus1 shall be in the range of 0 to 31, inclusive");
}
}
void HLSyntaxReader::parseOlsHrdParameters(GeneralHrdParams * generalHrd, OlsHrdParams *olsHrd, uint32_t firstSubLayer, uint32_t maxNumSubLayersMinus1)
{
  uint32_t  symbol;

  for( int i = firstSubLayer; i <= maxNumSubLayersMinus1; i ++ )
  {
    OlsHrdParams *hrd = &(olsHrd[i]);
    READ_FLAG( symbol, "fixed_pic_rate_general_flag");         hrd->fixedPicRateGeneralFlag = (symbol == 1);
    if (!hrd->fixedPicRateGeneralFlag)
    {
      READ_FLAG( symbol, "fixed_pic_rate_within_cvs_flag");    hrd->fixedPicRateWithinCvsFlag = (symbol == 1);
    }
    else
    {
      hrd->fixedPicRateWithinCvsFlag = (true);
    }

    hrd->lowDelayHrdFlag = (false); // Inferred to be 0 when not present

    if (hrd->fixedPicRateWithinCvsFlag)
    {
      READ_UVLC( symbol, "elemental_duration_in_tc_minus1");   hrd->elementDurationInTcMinus1 = symbol;
    }
    else if((generalHrd->generalNalHrdParamsPresent || generalHrd->generalVclHrdParamsPresent) &&generalHrd->hrdCpbCntMinus1 == 0)
    {
      READ_FLAG( symbol, "low_delay_hrd_flag");                hrd->lowDelayHrdFlag = (symbol == 1);
    }

    for( int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if (((nalOrVcl == 0) && (generalHrd->generalNalHrdParamsPresent)) || ((nalOrVcl == 1) && (generalHrd->generalVclHrdParamsPresent)))
      {
        for (int j = 0; j <= (generalHrd->hrdCpbCntMinus1); j++)
        {
          READ_UVLC( symbol, "bit_rate_value_minus1");         hrd->bitRateValueMinus1[j][nalOrVcl] = symbol;
          READ_UVLC( symbol, "cpb_size_value_minus1");         hrd->cpbSizeValueMinus1[j][nalOrVcl] = symbol;
          if (generalHrd->generalDecodingUnitHrdParamsPresent)
          {
            READ_UVLC( symbol, "cpb_size_du_value_minus1");    hrd->duCpbSizeValueMinus1[j][nalOrVcl] = symbol;
            READ_UVLC( symbol, "bit_rate_du_value_minus1");    hrd->duBitRateValueMinus1[j][nalOrVcl] = symbol;
          }
          READ_FLAG( symbol, "cbr_flag");                      hrd->cbrFlag[j][nalOrVcl] = (symbol == 1);
        }
      }
    }
  }
  for (int i = 0; i < firstSubLayer; i++)
  {
    OlsHrdParams* hrdHighestTLayer = &(olsHrd[maxNumSubLayersMinus1]);
    OlsHrdParams* hrdTemp = &(olsHrd[i]);
    hrdTemp->fixedPicRateGeneralFlag   = hrdHighestTLayer->fixedPicRateGeneralFlag;
    hrdTemp->fixedPicRateWithinCvsFlag = hrdHighestTLayer->fixedPicRateWithinCvsFlag;
    hrdTemp->elementDurationInTcMinus1 = hrdHighestTLayer->elementDurationInTcMinus1;
    for (int nalOrVcl = 0; nalOrVcl < 2; nalOrVcl++)
    {
      if (((nalOrVcl == 0) && (generalHrd->generalNalHrdParamsPresent)) || ((nalOrVcl == 1) && (generalHrd->generalVclHrdParamsPresent)))
      {
        for (int j = 0; j <= (generalHrd->hrdCpbCntMinus1); j++)
        {
          hrdTemp->bitRateValueMinus1[j][nalOrVcl] = hrdHighestTLayer->bitRateValueMinus1[j][nalOrVcl];
          hrdTemp->cpbSizeValueMinus1[j][nalOrVcl] = hrdHighestTLayer->cpbSizeValueMinus1[j][nalOrVcl];
          if (generalHrd->generalDecodingUnitHrdParamsPresent)
          {
            hrdTemp->duBitRateValueMinus1[j][nalOrVcl] = hrdHighestTLayer->duBitRateValueMinus1[j][nalOrVcl]; 
            hrdTemp->duCpbSizeValueMinus1[j][nalOrVcl] = hrdHighestTLayer->duCpbSizeValueMinus1[j][nalOrVcl]; 
          }
          hrdTemp->cbrFlag[j][nalOrVcl] = hrdHighestTLayer->cbrFlag[j][nalOrVcl]; 
        }
      }
    }
  }
}

void HLSyntaxReader::dpb_parameters(int maxSubLayersMinus1, bool subLayerInfoFlag, SPS *pcSPS)
{
  uint32_t code;
  for (int i = (subLayerInfoFlag ? 0 : maxSubLayersMinus1); i <= maxSubLayersMinus1; i++)
  {
    READ_UVLC(pcSPS->maxDecPicBuffering[i], "dpb_max_dec_pic_buffering_minus1[i]");
    READ_UVLC(code,      "dpb_max_num_reorder_pics[i]"); pcSPS->numReorderPics[i] = code;
    READ_UVLC(pcSPS->maxLatencyIncreasePlus1[i], "dpb_max_latency_increase_plus1[i]");
  }
}
 
void HLSyntaxReader::parseExtraPHBitsStruct( SPS *sps, int numBytes )
{
  uint32_t symbol;
  std::vector<bool> presentFlags;
  presentFlags.resize ( 8 * numBytes );
  
  for (int i=0; i < 8*numBytes; i++)
  {
    READ_FLAG(symbol, "sps_extra_ph_bit_present_flag[ i ]");
    presentFlags[i] = symbol;
  }
}

void HLSyntaxReader::parseExtraSHBitsStruct( SPS *sps, int numBytes )
{
  uint32_t symbol;
  std::vector<bool> presentFlags;
  presentFlags.resize ( 8 * numBytes );
  
  for (int i=0; i < 8*numBytes; i++)
  {
    READ_FLAG(symbol, "sps_extra_sh_bit_present_flag[ i ]");
    presentFlags[i] = symbol;
  }
}


void HLSyntaxReader::parseSPS(SPS* pcSPS)
{
  uint32_t  uiCode;
  DTRACE( g_trace_ctx, D_HEADER, "=========== Sequence Parameter Set  ===========\n" );
  READ_CODE( 4,  uiCode, "sps_seq_parameter_set_id");       pcSPS->dciId = ( uiCode );
  READ_CODE( 4,  uiCode, "sps_video_parameter_set_id" );      pcSPS->vpsId = ( uiCode );
  READ_CODE(3, uiCode, "sps_max_sub_layers_minus1");          pcSPS->maxTLayers  = (uiCode + 1);
  CHECK(uiCode > 6, "Invalid maximum number of T-layer signalled");
  READ_CODE(2,  uiCode, "sps_chroma_format_idc" );                  pcSPS->chromaFormatIdc = ( ChromaFormat(uiCode) );
  READ_CODE(2, uiCode, "sps_log2_ctu_size_minus5");                pcSPS->CTUSize = (1 << (uiCode + 5));
  CHECK(uiCode > 2, "sps_log2_ctu_size_minus5 must be less than or equal to 2");
  unsigned ctbLog2SizeY = uiCode + 5;

  READ_FLAG(pcSPS->ptlDpbHrdParamsPresent, "sps_ptl_dpb_hrd_params_present_flag");

  if( !pcSPS->vpsId )
  {
    CHECK( !pcSPS->ptlDpbHrdParamsPresent, "When sps_video_parameter_set_id is equal to 0, the value of sps_ptl_dpb_hrd_params_present_flag shall be equal to 1" );
  }


  if (pcSPS->ptlDpbHrdParamsPresent)
  {
    parseProfileTierLevel(&pcSPS->profileTierLevel, true, pcSPS->maxTLayers - 1);
  }
  READ_FLAG(pcSPS->GDR, "gdr_enabled_flag");

  READ_FLAG(pcSPS->rprEnabled, "sps_ref_pic_resampling_enabled_flag");
  if (pcSPS->rprEnabled)
  {
    READ_FLAG(pcSPS->resChangeInClvsEnabled, "sps_res_change_in_clvs_allowed_flag");
  }

  if (pcSPS->profileTierLevel.constraintInfo.noResChangeInClvsConstraintFlag)
  {
    CHECK(uiCode != 0, "When no_res_change_in_clvs_constraint_flag is equal to 1, res_change_in_clvs_allowed_flag shall be equal to 0");
  }

  READ_UVLC (    uiCode, "sps_pic_width_max_in_luma_samples" );          pcSPS->maxPicWidthInLumaSamples = ( uiCode    );
  READ_UVLC (    uiCode, "sps_pic_height_max_in_luma_samples" );         pcSPS->maxPicHeightInLumaSamples = ( uiCode    );

  READ_FLAG(uiCode, "sps_conformance_window_flag");
  if (uiCode != 0)
  {
    Window &conf = pcSPS->conformanceWindow;
    READ_UVLC(   uiCode, "sps_conf_win_left_offset" );               conf.winLeftOffset  = ( uiCode * SPS::getWinUnitX( pcSPS->chromaFormatIdc ) );
    READ_UVLC(   uiCode, "sps_conf_win_right_offset" );              conf.winRightOffset = ( uiCode * SPS::getWinUnitX( pcSPS->chromaFormatIdc ) );
    READ_UVLC(   uiCode, "sps_conf_win_top_offset" );                conf.winTopOffset   = ( uiCode * SPS::getWinUnitY( pcSPS->chromaFormatIdc ) );
    READ_UVLC(   uiCode, "sps_conf_win_bottom_offset" );             conf.winBottomOffset= ( uiCode * SPS::getWinUnitY( pcSPS->chromaFormatIdc ) );
    conf.enabledFlag = true;
  }

  READ_FLAG( pcSPS->subPicInfoPresent, "sps_subpic_info_present_flag" );

  if (pcSPS->subPicInfoPresent)
  {
    THROW("no support");
  }

  READ_UVLC(     uiCode, "sps_bitdepth_minus8" );
  CHECK(uiCode > 8, "Invalid bit depth signalled");
  pcSPS->bitDepths.recon[CH_L] = 8 + uiCode;
  pcSPS->bitDepths.recon[CH_C] = 8 + uiCode;
  pcSPS->qpBDOffset[CH_L] = 6*uiCode;
  pcSPS->qpBDOffset[CH_C] = 6*uiCode;

  READ_FLAG( pcSPS->entropyCodingSyncEnabled, "sps_entropy_coding_sync_enabled_flag" );
  READ_FLAG( pcSPS->entryPointsPresent, "sps_entry_point_offsets_present_flag");


  READ_CODE( 4, uiCode,    "sps_log2_max_pic_order_cnt_lsb_minus4" );   pcSPS->bitsForPOC = ( 4 + uiCode );
  CHECK(uiCode > 12, "Invalid code");

  READ_FLAG(pcSPS->pocMsbFlag, "sps_poc_msb_flag");
  if( pcSPS->pocMsbFlag)
  {
    READ_UVLC(uiCode, "sps_poc_msb_len_minus1");                  pcSPS->pocMsbLen = (1 + uiCode);
    CHECK(uiCode > (32 - ( pcSPS->bitsForPOC - 4 )- 5), "The value of poc_msb_len_minus1 shall be in the range of 0 to 32 - log2_max_pic_order_cnt_lsb_minus4 - 5, inclusive");
  }

  // extra bits are for future extensions, we will read, but ignore them,
  // unless a meaning is specified in the spec
  READ_CODE(2, uiCode, "sps_num_extra_ph_bits_bytes");  pcSPS->numExtraPHBitsBytes = (uiCode);
  parseExtraPHBitsStruct( pcSPS, uiCode );
  READ_CODE(2, uiCode, "sps_num_extra_sh_bits_bytes");  pcSPS->numExtraSHBitsBytes = (uiCode);
  parseExtraSHBitsStruct( pcSPS, uiCode );

  if (pcSPS->ptlDpbHrdParamsPresent)
  {
    if (pcSPS->maxTLayers - 1 > 0)
    {
      READ_FLAG(pcSPS->subLayerDpbParams, "sps_sublayer_dpb_params_flag");  
    }    
    dpb_parameters(pcSPS->maxTLayers - 1, pcSPS->subLayerDpbParams, pcSPS);
  }

  READ_UVLC(uiCode, "sps_log2_min_luma_coding_block_size_minus2");
  int log2MinCUSize = uiCode + 2;
  pcSPS->log2MinCodingBlockSize = (log2MinCUSize);

  CHECK(log2MinCUSize > std::min(6, (int)(ctbLog2SizeY)), "log2_min_luma_coding_block_size_minus2 shall be in the range of 0 to min (4, log2_ctu_size - 2)");

  const int minCuSize = 1 << pcSPS->log2MinCodingBlockSize;
  CHECK( ( pcSPS->maxPicWidthInLumaSamples % ( std::max( 8, minCuSize ) ) ) != 0, "Coded frame width must be a multiple of Max(8, the minimum unit size)" );
  CHECK( ( pcSPS->maxPicHeightInLumaSamples % ( std::max( 8, minCuSize ) ) ) != 0, "Coded frame height must be a multiple of Max(8, the minimum unit size)" );

  READ_FLAG(pcSPS->partitionOverrideEnabled, "sps_partition_constraints_override_enabled_flag");
  READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_intra_slice_luma");
  unsigned minQtLog2SizeIntraY = uiCode + log2MinCUSize;
  pcSPS->minQTSize[0] = 1 << minQtLog2SizeIntraY;

  READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_intra_slice_luma");     pcSPS->maxMTTDepth[0] = uiCode;
  CHECK(uiCode > 2 * (ctbLog2SizeY - log2MinCUSize), "sps_max_mtt_hierarchy_depth_intra_slice_luma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");

  pcSPS->maxTTSize[0] = pcSPS->maxBTSize[0] = pcSPS->minQTSize[0];
  if (pcSPS->maxMTTDepth[0] != 0)
  {
    READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_intra_slice_luma"); pcSPS->maxBTSize[0] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY, "Invalid code");
    READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_intra_slice_luma"); pcSPS->maxTTSize[0] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY, "Invalid code");
  }
  if( pcSPS->chromaFormatIdc != CHROMA_400 ) 
  {
    READ_FLAG(pcSPS->dualITree, "sps_qtbtt_dual_tree_intra_flag");
  }
  if (pcSPS->dualITree)
  {
    READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_intra_slice_chroma"); pcSPS->minQTSize[2] = 1 << (uiCode + log2MinCUSize);
    READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_intra_slice_chroma"); pcSPS->maxMTTDepth[2] = uiCode;
    CHECK(uiCode > 2 * (ctbLog2SizeY - log2MinCUSize), "sps_max_mtt_hierarchy_depth_intra_slice_chroma shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");
    pcSPS->maxTTSize[2] = pcSPS->maxBTSize[2] = pcSPS->minQTSize[2];
    if (pcSPS->maxMTTDepth[2] != 0)
    {
      READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_intra_slice_chroma"); pcSPS->maxBTSize[2] <<= uiCode;
      READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_intra_slice_chroma"); pcSPS->maxTTSize[2] <<= uiCode;
    }
  }

  READ_UVLC(uiCode, "sps_log2_diff_min_qt_min_cb_inter_slice");
  unsigned minQtLog2SizeInterY = uiCode + log2MinCUSize;
  pcSPS->minQTSize[1] = 1 << minQtLog2SizeInterY;

  READ_UVLC(uiCode, "sps_max_mtt_hierarchy_depth_inter_slice");     pcSPS->maxMTTDepth[1] = uiCode;
  CHECK(uiCode > 2*(ctbLog2SizeY - log2MinCUSize), "sps_max_mtt_hierarchy_depth_inter_slice shall be in the range 0 to 2*(ctbLog2SizeY - log2MinCUSize)");

  pcSPS->maxTTSize[1] = pcSPS->maxBTSize[1] = pcSPS->minQTSize[1];
  if (pcSPS->maxMTTDepth[1] != 0)
  {
    READ_UVLC(uiCode, "sps_log2_diff_max_bt_min_qt_inter_slice");     pcSPS->maxBTSize[1] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY, "Invalid code");
    READ_UVLC(uiCode, "sps_log2_diff_max_tt_min_qt_inter_slice");     pcSPS->maxTTSize[1] <<= uiCode;
    CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY, "Invalid code");
  }

  READ_FLAG( uiCode, "sps_max_luma_transform_size_64_flag");        pcSPS->log2MaxTbSize = ( (uiCode ? 1 : 0) + 5 );

  READ_FLAG(pcSPS->transformSkip, "sps_transform_skip_enabled_flag");
  if (pcSPS->transformSkip)
  {
    READ_UVLC(uiCode, "sps_log2_transform_skip_max_size_minus2");
    pcSPS->log2MaxTransformSkipBlockSize = (uiCode + 2);

    READ_FLAG( pcSPS->BDPCM, "sps_bdpcm_enabled_flag");
  }  
  READ_FLAG( pcSPS->MTS,    "sps_mts_enabled_flag" );
  if ( pcSPS->MTS )
  {
    READ_FLAG( pcSPS->MTSIntra,    "sps_explicit_mts_intra_enabled_flag" );
    READ_FLAG( pcSPS->MTSInter,    "sps_explicit_mts_inter_enabled_flag" );
  }
  READ_FLAG( pcSPS->LFNST, "sps_lfnst_enabled_flag" );


  if (pcSPS->chromaFormatIdc != CHROMA_400)
  {
    READ_FLAG(pcSPS->jointCbCr, "sps_joint_cbcr_enabled_flag");
    ChromaQpMappingTableParams chromaQpMappingTableParams;
    READ_FLAG(chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag, "sps_same_qp_table_for_chroma");
    int numQpTables = chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag ? 1 : (pcSPS->jointCbCr ? 3 : 2);
    chromaQpMappingTableParams.m_numQpTables = numQpTables;
    for (int i = 0; i < numQpTables; i++)
    {
      int32_t qpTableStart = 0;
      READ_SVLC(qpTableStart, "sps_qp_table_starts_minus26"); chromaQpMappingTableParams.m_qpTableStartMinus26[i] = qpTableStart;
      READ_UVLC(uiCode, "sps_num_points_in_qp_table_minus1"); chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[i] = uiCode;
      std::vector<int> deltaQpInValMinus1(chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[i] + 1);
      std::vector<int> deltaQpOutVal(chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[i] + 1);
      for (int j = 0; j <= chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[i]; j++)
      {
        READ_UVLC(uiCode, "sps_delta_qp_in_val_minus1");  deltaQpInValMinus1[j] = uiCode;
        READ_UVLC(uiCode, "sps_delta_qp_diff_val");
        deltaQpOutVal[j] = uiCode ^ deltaQpInValMinus1[j];
      }
      chromaQpMappingTableParams.m_deltaQpInValMinus1[i] = deltaQpInValMinus1;
      chromaQpMappingTableParams.m_deltaQpOutVal[i] = deltaQpOutVal;
    }
    pcSPS->chromaQpMappingTable.m_numQpTables = chromaQpMappingTableParams.m_numQpTables;
    pcSPS->chromaQpMappingTable.setParams(chromaQpMappingTableParams, pcSPS->qpBDOffset[ CH_C ]);
    pcSPS->chromaQpMappingTable.derivedChromaQPMappingTables();
  }

  READ_FLAG( pcSPS->saoEnabled, "sps_sao_enabled_flag" );
  READ_FLAG( pcSPS->alfEnabled, "sps_alf_enabled_flag" );
  if (pcSPS->alfEnabled && pcSPS->chromaFormatIdc != CHROMA_400)
  {
    READ_FLAG( pcSPS->ccalfEnabled, "sps_ccalf_enabled_flag" );
  }
  READ_FLAG( pcSPS->lumaReshapeEnable, "sps_lmcs_enable_flag");
  READ_FLAG( pcSPS->weightPred, "sps_weighted_pred_flag" );
  READ_FLAG( pcSPS->weightedBiPred, "sps_weighted_bipred_flag" );
  READ_FLAG( pcSPS->longTermRefsPresent, "long_term_ref_pics_flag");
  if (pcSPS->vpsId > 0)
  {
    READ_FLAG(pcSPS->interLayerPresent, "sps_inter_layer_ref_pics_present_flag");
  }
  READ_FLAG(pcSPS->idrRefParamList, "sps_idr_rpl_present_flag");
  READ_FLAG(pcSPS->rpl1CopyFromRpl0, "rpl1_copy_from_rpl0_flag");

  //Read candidate for List0
  READ_UVLC(uiCode, "sps_num_ref_pic_lists_in_sps[0]");
  uint32_t numberOfRPL = uiCode;
  pcSPS->rplList[0].resize(numberOfRPL+1);
  for (uint32_t ii = 0; ii < numberOfRPL; ii++)
  {
    parseRefPicList(pcSPS, &pcSPS->rplList[0][ii], ii);
  }

  //Read candidate for List1
  if (!pcSPS->rpl1CopyFromRpl0)
  {
    READ_UVLC(uiCode, "sps_num_ref_pic_lists_in_sps[1]");
    numberOfRPL = uiCode;
    pcSPS->rplList[1].resize(numberOfRPL+1);
    for (uint32_t ii = 0; ii < numberOfRPL; ii++)
    {
      parseRefPicList(pcSPS, &pcSPS->rplList[1][ii], ii);
    }
  }
  else
  {
    numberOfRPL = (uint32_t)pcSPS->getNumRPL(0);
    pcSPS->rplList[1].resize(numberOfRPL+1);
    for (uint32_t ii = 0; ii < numberOfRPL; ii++)
      copyRefPicList(pcSPS, &pcSPS->rplList[0][ii], &pcSPS->rplList[1][ii]);
  }
  pcSPS->rpl1IdxPresent = (pcSPS->rplList[0].size() != pcSPS->rplList[1].size());

  READ_FLAG( pcSPS->wrapAroundEnabled, "sps_ref_wraparound_enabled_flag");
  READ_FLAG( pcSPS->temporalMVPEnabled, "sps_temporal_mvp_enabled_flag" );

  if ( pcSPS->temporalMVPEnabled )
  {
    READ_FLAG( pcSPS->SbtMvp,    "sps_sbtmvp_enabled_flag" );
  }

  READ_FLAG( pcSPS->AMVR,  "sps_amvr_enabled_flag" );

  READ_FLAG( pcSPS->BDOF, "sps_bdof_enabled_flag" );
  if (pcSPS->BDOF)
  {
    READ_FLAG(pcSPS->BdofPresent, "sps_bdof_pic_present_flag");
  }

  READ_FLAG(pcSPS->SMVD, "sps_smvd_enabled_flag");
  READ_FLAG(pcSPS->DMVR, "sps_dmvr_enabled_flag");
  if (pcSPS->DMVR)
  {
    READ_FLAG(pcSPS->DmvrPresent, "sps_dmvr_pic_present_flag");
  }

  READ_FLAG(pcSPS->MMVD, "sps_mmvd_enabled_flag");  
  if (pcSPS->MMVD)
  {
    READ_FLAG(pcSPS->fpelMmvd, "sps_mmvd_fullpel_only_flag");
  }
  READ_UVLC(uiCode, "sps_six_minus_max_num_merge_cand");
  CHECK(MRG_MAX_NUM_CANDS <= uiCode, "Incorrrect max number of merge candidates!");
  pcSPS->maxNumMergeCand = (MRG_MAX_NUM_CANDS - uiCode);
  READ_FLAG(pcSPS->SBT, "sps_sbt_enabled_flag");
  READ_FLAG( pcSPS->Affine, "sps_affine_enabled_flag" );
  if ( pcSPS->Affine )
  {
    READ_UVLC(uiCode, "sps_five_minus_max_num_subblock_merge_cand");
    pcSPS->maxNumAffineMergeCand = (AFFINE_MRG_MAX_NUM_CANDS - uiCode);
    READ_FLAG( pcSPS->AffineType,  "sps_affine_type_flag" );
    if( pcSPS->AMVR)
    {
      READ_FLAG( pcSPS->AffineAmvr, "sps_affine_amvr_enabled_flag" );
    }
    READ_FLAG( pcSPS->PROF, "sps_affine_prof_enabled_flag" );
    if (pcSPS->PROF)
    {
      READ_FLAG(pcSPS->ProfPresent, "sps_prof_pic_present_flag");
    }
  }

  READ_FLAG( pcSPS->BCW,    "sps_bcw_enabled_flag" );
  READ_FLAG( pcSPS->CIIP,     "sps_ciip_enabled_flag" );
  if (pcSPS->maxNumMergeCand >= 2)
  {
    READ_FLAG(pcSPS->GEO, "sps_gpm_enabled_flag");
    if (pcSPS->GEO )
    {
      if( pcSPS->maxNumMergeCand >= 3)
      {
        READ_UVLC(uiCode, "sps_max_num_merge_cand_minus_max_num_gpm_cand");
        CHECK(pcSPS->maxNumMergeCand < uiCode, "Incorrrect max number of GEO candidates!");
        pcSPS->maxNumGeoCand = ((uint32_t)(pcSPS->maxNumMergeCand - uiCode));
      }
      else 
      {
        pcSPS->maxNumGeoCand = 2;
      }
    }
  }
  READ_UVLC(uiCode, "sps_log2_parallel_merge_level_minus2");
  CHECK(uiCode + 2 > ctbLog2SizeY, "The value of log2_parallel_merge_level_minus2 shall be in the range of 0 to ctbLog2SizeY - 2");
  pcSPS->log2ParallelMergeLevelMinus2 = (uiCode);


  READ_FLAG(pcSPS->ISP, "sps_isp_enabled_flag");
  READ_FLAG(pcSPS->MRL, "sps_mrl_enabled_flag");
  READ_FLAG(pcSPS->MIP, "sps_mip_enabled_flag");
  if( pcSPS->chromaFormatIdc != CHROMA_400) 
  {
    READ_FLAG( pcSPS->LMChroma, "sps_cclm_enabled_flag" );
  }
  if( pcSPS->chromaFormatIdc == CHROMA_420 )
  {
    READ_FLAG( pcSPS->horCollocatedChroma, "sps_chroma_horizontal_collocated_flag" );
    READ_FLAG( pcSPS->verCollocatedChroma, "sps_chroma_vertical_collocated_flag" );
  }

  READ_FLAG( pcSPS->PLT,  "sps_palette_enabled_flag");
  if( pcSPS->chromaFormatIdc  == CHROMA_444 && pcSPS->log2MaxTbSize != 6)
  {
   READ_FLAG(pcSPS->useColorTrans, "sps_act_enabled_flag");
  }
  if (pcSPS->transformSkip || pcSPS->PLT)
  {
    READ_UVLC(uiCode, "sps_internal_bit_depth_minus_input_bit_depth");
    pcSPS->internalMinusInputBitDepth[CH_L] = pcSPS->internalMinusInputBitDepth[CH_C] = uiCode;
  }
  READ_FLAG(pcSPS->IBC,     "sps_ibc_enabled_flag");
  if (pcSPS->IBC)
  {
    READ_UVLC(uiCode, "sps_six_minus_max_num_ibc_merge_cand");
    CHECK(IBC_MRG_MAX_NUM_CANDS <= uiCode, "Incorrrect max number of IBC merge candidates!");
    pcSPS->maxNumIBCMergeCand = (IBC_MRG_MAX_NUM_CANDS - uiCode);
  }
  READ_FLAG( pcSPS->LADF, "sps_ladf_enabled_flag" );
  if ( pcSPS->LADF )
  {
    THROW("no support");
  }

   READ_FLAG(pcSPS->scalingListEnabled, "sps_explicit_scaling_list_enabled_flag");

  if (pcSPS->LFNST && pcSPS->scalingListEnabled)
  {
    READ_FLAG(pcSPS->disableScalingMatrixForLfnstBlks, "sps_scaling_matrix_for_lfnst_disabled_flag"); 
  }

  if (pcSPS->useColorTrans && pcSPS->scalingListEnabled)
  {
    READ_FLAG(pcSPS->scalingMatrixAlternativeColourSpaceDisabled, "sps_scaling_matrix_for_alternative_colour_space_disabled_flag");
  }
  if (pcSPS->scalingMatrixAlternativeColourSpaceDisabled)
  {
    READ_FLAG(pcSPS->scalingMatrixDesignatedColourSpace, "sps_scaling_matrix_designated_colour_space_flag");
  }
  READ_FLAG(pcSPS->depQuantEnabled, "sps_dep_quant_enabled_flag"); 
  READ_FLAG(pcSPS->signDataHidingEnabled, "sps_sign_data_hiding_enabled_flag"); 

  READ_FLAG( pcSPS->virtualBoundariesEnabled, "sps_virtual_boundaries_enabled_flag" );
  if( pcSPS->virtualBoundariesEnabled )
  {
    READ_FLAG( pcSPS->virtualBoundariesPresent, "sps_loop_filter_across_virtual_boundaries_present_flag" ); 
    if( pcSPS->virtualBoundariesPresent )
    {
      READ_CODE( 2, uiCode, "sps_num_ver_virtual_boundaries");        pcSPS->numVerVirtualBoundaries = ( uiCode );
      for( unsigned i = 0; i < pcSPS->numVerVirtualBoundaries; i++ )
      {
        READ_UVLC(uiCode, "sps_virtual_boundaries_pos_x");        pcSPS->virtualBoundariesPosX[i] = (uiCode << 3);
      }
      READ_CODE( 2, uiCode, "sps_num_hor_virtual_boundaries");        pcSPS->numHorVirtualBoundaries = ( uiCode );
      for( unsigned i = 0; i < pcSPS->numHorVirtualBoundaries; i++ )
      {
        READ_UVLC(uiCode, "sps_virtual_boundaries_pos_y");        pcSPS->virtualBoundariesPosY[i] = (uiCode << 3);
      }
    }
  }

  if(pcSPS->ptlDpbHrdParamsPresent)
  {

    READ_FLAG( pcSPS->hrdParametersPresent, "sps_general_hrd_params_present_flag");
    if( pcSPS->hrdParametersPresent )
    {
      parseGeneralHrdParameters(&pcSPS->generalHrdParams);
      if ((pcSPS->maxTLayers-1) > 0)
      {
        READ_FLAG(uiCode, "sps_sublayer_cpb_params_present_flag");  pcSPS->subLayerParametersPresent = uiCode;
      }
      else if((pcSPS->maxTLayers-1) == 0)
      {
        pcSPS->subLayerParametersPresent = 0;
      }

      uint32_t firstSubLayer = pcSPS->subLayerParametersPresent ? 0 : (pcSPS->maxTLayers - 1);
      parseOlsHrdParameters(&pcSPS->generalHrdParams, pcSPS->olsHrdParams, firstSubLayer, pcSPS->maxTLayers - 1);
    }
  }

  READ_FLAG(     pcSPS->fieldSeqFlag, "sps_field_seq_flag");

  READ_FLAG( pcSPS->vuiParametersPresent, "sps_vui_parameters_present_flag" );

  if (pcSPS->vuiParametersPresent)
  {
    READ_UVLC(uiCode, "sps_vui_payload_size_minus1");
    while (!isByteAligned())
    {
      READ_FLAG(uiCode, "sps_vui_alignment_zero_bit");
      CHECK(uiCode != 0, "sps_vui_alignment_zero_bit not equal to 0");
    }
    parseVUI(&pcSPS->vuiParameters, pcSPS);
  }

  // KJS: no SPS extensions defined yet

  READ_FLAG( uiCode, "sps_extension_present_flag");
  if (uiCode)
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
    bool sps_extension_flags[NUM_SPS_EXTENSION_FLAGS];

    for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
    {
      READ_FLAG( uiCode, syntaxStrings[i] );
      sps_extension_flags[i] = uiCode!=0;
    }

    bool bSkipTrailingExtensionBits=false;
    for(int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (sps_extension_flags[i])
      {
        switch (SPSExtensionFlagIndex(i))
        {
        case SPS_EXT__REXT:
          CHECK(bSkipTrailingExtensionBits, "Skipping trailing extension bits not supported");
          {
            SPSRExt &spsRExt = pcSPS->spsRExt;
            READ_FLAG( spsRExt.transformSkipRotationEnabled,    "transform_skip_rotation_enabled_flag");
            READ_FLAG( spsRExt.transformSkipContextEnabled,     "transform_skip_context_enabled_flag");
            READ_FLAG( spsRExt.extendedPrecisionProcessing,     "extended_precision_processing_flag");
            READ_FLAG( spsRExt.intraSmoothingDisabled,          "intra_smoothing_disabled_flag");
            READ_FLAG( spsRExt.highPrecisionOffsetsEnabled,     "high_precision_offsets_enabled_flag");
            READ_FLAG( spsRExt.persistentRiceAdaptationEnabled, "persistent_rice_adaptation_enabled_flag");
            READ_FLAG( spsRExt.cabacBypassAlignmentEnabled,     "cabac_bypass_alignment_enabled_flag");
          }
          break;
        default:
          bSkipTrailingExtensionBits=true;
          break;
        }
      }
    }
    if (bSkipTrailingExtensionBits)
    {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "sps_extension_data_flag");
      }
    }
  }
  xReadRbspTrailingBits();
}


void HLSyntaxReader::parseDCI(DCI* dci)
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Decoding Parameter Set     ===========\n" );

  uint32_t  symbol;

  READ_CODE( 4,  symbol,  "dci_decoding_parameter_set_id" );
  
  uint32_t numPTLs;
  READ_CODE( 4, numPTLs,       "dci_num_ptls_minus1" );
  numPTLs += 1;

  std::vector<ProfileTierLevel> ptls;
  ptls.resize(numPTLs);
  for (int i=0; i<numPTLs; i++)
  {
    parseProfileTierLevel(&dci->profileTierLevel[i], true, 0);
  }
  READ_FLAG( symbol,      "dci_extension_flag" );
  if (symbol)
  {
    while ( xMoreRbspData() )
    {
      READ_FLAG( symbol, "dci_extension_data_flag");
    }
  }
  xReadRbspTrailingBits();
}

void HLSyntaxReader::parseVPS(VPS* pcVPS)
{
  DTRACE( g_trace_ctx, D_HEADER, "=========== Video Parameter Set     ===========\n" );

  uint32_t  uiCode;

  READ_CODE(4,  uiCode, "vps_video_parameter_set_id");
  pcVPS->vpsId = uiCode;
  READ_CODE(6, uiCode, "vps_max_layers_minus1");
  pcVPS->maxLayers = uiCode;

  if (pcVPS->maxLayers - 1 == 0)
  {
    pcVPS->eachLayerIsAnOls = true;
  }
  READ_CODE(3, uiCode, "vps_max_sublayers_minus1");           pcVPS->maxSubLayers = (uiCode + 1); CHECK(uiCode + 1 > MAX_VPS_SUBLAYERS, "Invalid code");
  if( pcVPS->maxLayers > 1 && pcVPS->maxSubLayers > 1)
  {
    READ_FLAG(pcVPS->defaultPtlDpbHrdMaxTidFlag, "vps_default_ptl_dpb_hrd_max_tid_flag");
  }
  else
  {
    pcVPS->defaultPtlDpbHrdMaxTidFlag = true;
  }
  if( pcVPS->maxLayers > 1 )
  {
    READ_FLAG(pcVPS->allIndependentLayers, "vps_all_independent_layers_flag");
    if (pcVPS->allIndependentLayers == 0)
    {
      pcVPS->eachLayerIsAnOls = false;
    }
  }

  for (uint32_t i = 0; i < pcVPS->maxLayers; i++)
  {
    READ_CODE(6, uiCode, "vps_layer_id");                     pcVPS->layerId[i] = uiCode;
    pcVPS->generalLayerIdx[i] = uiCode;

    if (i > 0 && !pcVPS->allIndependentLayers)
    {
      READ_FLAG(uiCode, "vps_independent_layer_flag");     pcVPS->independentLayer[i] = uiCode;
      if (!pcVPS->independentLayer[i])
      {
        READ_FLAG(uiCode, "max_tid_ref_present_flag[ i ]");
        bool presentFlag = uiCode;
        uint16_t sumUiCode = 0;
        for (int j = 0, k = 0; j < i; j++)
        {
          READ_FLAG(uiCode, "vps_direct_ref_layer_flag"); pcVPS->directRefLayer[i][j] = uiCode;
          if (uiCode)
          {
            pcVPS->interLayerRefIdx[i][j] = k;
            pcVPS->directRefLayerIdx[i][k++] = j;
            sumUiCode++;
          }
          if (presentFlag && pcVPS->directRefLayer[i][j])
          {
            READ_CODE(3, uiCode, "max_tid_il_ref_pics_plus1[ i ][ j ]");
            pcVPS->maxTidIlRefPicsPlus1[i][j] = uiCode;
          }
          else
          {
            pcVPS->maxTidIlRefPicsPlus1[i][j]= 7;
          }
        }
        CHECK(sumUiCode == 0, "There has to be at least one value of j such that the value of vps_direct_dependency_flag[ i ][ j ] is equal to 1,when vps_independent_layer_flag[ i ] is equal to 0 ");
      }
    }
  }

  if (pcVPS->maxLayers > 1)
  {
    if (pcVPS->allIndependentLayers)
    {
      READ_FLAG(pcVPS->eachLayerIsAnOls, "vps_each_layer_is_an_ols_flag");
      if (pcVPS->eachLayerIsAnOls == 0)
      {
        pcVPS->olsModeIdc = (2);
      }
    }
    if (!pcVPS->eachLayerIsAnOls)
    {
      if (!pcVPS->allIndependentLayers)
      {
        READ_CODE(2, uiCode, "vps_ols_mode_idc");             pcVPS->olsModeIdc = (uiCode); CHECK(uiCode > MAX_VPS_OLS_MODE_IDC, "Invalid code");
      }
      if (pcVPS->olsModeIdc == 2)
      {
        READ_CODE(8, uiCode, "vps_num_output_layer_sets_minus2");   pcVPS->numOutputLayerSets = (uiCode + 2);
        for (uint32_t i = 1; i <= pcVPS->numOutputLayerSets - 1; i++)
        {
          for (uint32_t j = 0; j < pcVPS->maxLayers; j++)
          {
            READ_FLAG(pcVPS->olsOutputLayer[i][j], "vps_ols_output_layer_flag");
          }
        }
      }
    }
    READ_CODE(8, uiCode, "vps_num_ptls_minus1");      pcVPS->numPtls = uiCode + 1;
  }

  pcVPS->deriveOutputLayerSets();
  for (int i = 0; i < pcVPS->numPtls; i++)
  {
    if(i > 0)
    {
      READ_FLAG(pcVPS->ptPresent[i], "vps_pt_present_flag");           
    }
    else
      pcVPS->ptPresent[0] = true;
    if(!pcVPS->defaultPtlDpbHrdMaxTidFlag)
    {
      READ_CODE(3, uiCode, "vps_ptl_max_temporal_id");    
      pcVPS->ptlMaxTemporalId[i] = uiCode;
    }
    else if(pcVPS->maxSubLayers > 1)
      pcVPS->ptlMaxTemporalId[i] = pcVPS->maxSubLayers - 1;
    else
      pcVPS->ptlMaxTemporalId[i] = 0;
  }
  int cnt = 0;
  while (m_pcBitstream->getNumBitsUntilByteAligned())
  {
    READ_FLAG( uiCode, "vps_ptl_reserved_zero_bit");
    CHECK(uiCode!=0, "Alignment bit is not '0'");
    cnt++;
  }
  CHECK(cnt >= 8, "Read more than '8' alignment bits");
  pcVPS->profileTierLevel.resize(pcVPS->numPtls);
  for (int i = 0; i < pcVPS->numPtls; i++)
  {
    parseProfileTierLevel(&pcVPS->profileTierLevel[i], pcVPS->ptPresent[i], pcVPS->ptlMaxTemporalId[i] - 1);
  }

  for (int i = 0; i < pcVPS->totalNumOLSs; i++)
  {
    if(pcVPS->numPtls > 1 && pcVPS->numPtls != pcVPS->totalNumOLSs)
    {
      READ_CODE(8, uiCode, "ols_ptl_idx");
      pcVPS->olsPtlIdx[i] = uiCode;
    }
    else if (pcVPS->numPtls == pcVPS->totalNumOLSs)
    {
      pcVPS->olsPtlIdx[i] = i;
    }
    else
      pcVPS->olsPtlIdx[i] = 0;
  }

  if( !pcVPS->eachLayerIsAnOls )
  {
    READ_UVLC( uiCode, "vps_num_dpb_params" ); pcVPS->numDpbParams = uiCode;
  }

  if( pcVPS->numDpbParams > 0 && pcVPS->maxSubLayers > 1 )
  {
    READ_FLAG( uiCode, "vps_sublayer_dpb_params_present_flag" ); pcVPS->sublayerDpbParamsPresent = uiCode;
  }

  pcVPS->dpbParameters.resize( pcVPS->numDpbParams );

  for( int i = 0; i < pcVPS->numDpbParams; i++ )
  {
    if (!pcVPS->allLayersSameNumSubLayers)
    {
      READ_CODE(3, uiCode, "vps_dpb_max_temporal_id[i]");
      pcVPS->dpbMaxTemporalId.push_back(uiCode);
    }
    else
    {
      pcVPS->dpbMaxTemporalId.push_back(pcVPS->maxSubLayers - 1);
    }

    for( int j = ( pcVPS->sublayerDpbParamsPresent ? 0 : pcVPS->dpbMaxTemporalId[i] ); j <= pcVPS->dpbMaxTemporalId[i]; j++ )
    {
      READ_UVLC( uiCode, "vps_max_dec_pic_buffering_minus1[i]" );  pcVPS->dpbParameters[i].maxDecPicBuffering[j] = uiCode+1;
      READ_UVLC( uiCode, "vps_max_num_reorder_pics[i]" );          pcVPS->dpbParameters[i].numReorderPics[j] = uiCode;
      READ_UVLC( uiCode, "vps_max_latency_increase_plus1[i]" );    pcVPS->dpbParameters[i].maxLatencyIncreasePlus1[j] = uiCode;
    }

    for( int j = ( pcVPS->sublayerDpbParamsPresent ? pcVPS->dpbMaxTemporalId[i] : 0 ); j < pcVPS->dpbMaxTemporalId[i]; j++ )
    {
      // When max_dec_pic_buffering_minus1[ i ] is not present for i in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to max_dec_pic_buffering_minus1[ maxSubLayersMinus1 ].
      pcVPS->dpbParameters[i].maxDecPicBuffering[j] = pcVPS->dpbParameters[i].maxDecPicBuffering[pcVPS->dpbMaxTemporalId[i]];

      // When max_num_reorder_pics[ i ] is not present for i in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to max_num_reorder_pics[ maxSubLayersMinus1 ].
      pcVPS->dpbParameters[i].numReorderPics[j] = pcVPS->dpbParameters[i].numReorderPics[pcVPS->dpbMaxTemporalId[i]];

      // When max_latency_increase_plus1[ i ] is not present for i in the range of 0 to maxSubLayersMinus1 - 1, inclusive, due to subLayerInfoFlag being equal to 0, it is inferred to be equal to max_latency_increase_plus1[ maxSubLayersMinus1 ].
      pcVPS->dpbParameters[i].maxLatencyIncreasePlus1[j] = pcVPS->dpbParameters[i].maxLatencyIncreasePlus1[pcVPS->dpbMaxTemporalId[i]];
    }
  }

  for( int i = 0, j = 0; i < pcVPS->totalNumOLSs; i++ )
  {
    if( pcVPS->numLayersInOls[i] > 1 )
    {
      READ_UVLC( uiCode, "vps_ols_dpb_pic_width[i]" ); pcVPS->olsDpbPicSize[i].width = uiCode;
      READ_UVLC( uiCode, "vps_ols_dpb_pic_height[i]" ); pcVPS->olsDpbPicSize[i].height = uiCode;        
      READ_CODE( 2, uiCode, "vps_ols_dpb_chroma_format[i]"); pcVPS->olsDpbChromaFormatIdc[i] = uiCode;
      READ_UVLC( uiCode, "vps_ols_dpb_bitdepth_minus8[i]"); pcVPS->olsDpbBitDepthMinus8[i] = uiCode;
      if ((pcVPS->numDpbParams > 1) && (pcVPS->numDpbParams != pcVPS->numMultiLayeredOlss))
      {
        READ_UVLC( uiCode, "vps_ols_dpb_params_idx[i]" ); pcVPS->olsDpbParamsIdx[i] = uiCode;
      }
      else if (pcVPS->numDpbParams == 1)
      {
        pcVPS->olsDpbParamsIdx[i] = 0;
      }
      else
      {
        pcVPS->olsDpbParamsIdx[i] = j;
      }
       j += 1;
    }
  }

  READ_FLAG(uiCode, "vps_extension_flag");
  if (uiCode)
  {
    while (xMoreRbspData())
    {
      READ_FLAG(uiCode, "vps_extension_data_flag");
    }
  }

  xReadRbspTrailingBits();
}

void HLSyntaxReader::parsePictureHeader( PicHeader* picHeader, ParameterSetManager *parameterSetManager, bool readRbspTrailingBits )
{
  uint32_t  uiCode; 
  PPS*      pps = NULL;
  SPS*      sps = NULL;
  
  DTRACE( g_trace_ctx, D_HEADER, "=========== Picture Header ===========\n" );

  READ_FLAG(picHeader->gdrOrIrapPic, "ph_gdr_or_irap_pic_flag");
  READ_FLAG(picHeader->nonRefPic, "ph_non_ref_pic_flag");
  if (picHeader->gdrOrIrapPic)
  {
    READ_FLAG(picHeader->gdrPic, "ph_gdr_pic_flag");
  }
  READ_FLAG(picHeader->picInterSliceAllowed, "ph_inter_slice_allowed_flag");
  if (picHeader->picInterSliceAllowed)
  {
    READ_FLAG(picHeader->picIntraSliceAllowed, "ph_intra_slice_allowed_flag");
  }
  else
  {
    picHeader->picIntraSliceAllowed = true;
  }
  CHECK(!picHeader->picInterSliceAllowed && !picHeader->picIntraSliceAllowed, "Invalid picture without intra or inter slice");

  // parameter sets
  READ_UVLC( uiCode, "ph_pic_parameter_set_id");
  picHeader->ppsId = uiCode;
  pps = parameterSetManager->getPPS(picHeader->ppsId);
  CHECK(pps == 0, "Invalid PPS");
  picHeader->spsId = pps->spsId;
  sps = parameterSetManager->getSPS(picHeader->spsId);
  CHECK(sps == 0, "Invalid SPS");
  READ_CODE(sps->bitsForPOC, uiCode, "ph_pic_order_cnt_lsb");
  picHeader->pocLsb = (uiCode);

  if( picHeader->gdrPic ) 
  {
    READ_UVLC(picHeader->recoveryPocCnt, "ph_recovery_poc_cnt");
  }
  
  std::vector<bool> phExtraBitsPresent = sps->extraPHBitPresent;
  for (int i=0; i< sps->numExtraPHBitsBytes * 8; i++)
  {
    // extra bits are ignored (when present)
    if (phExtraBitsPresent[i])
    {
      READ_FLAG(uiCode, "ph_extra_bit[ i ]");
    }
  }
  
  if (sps->pocMsbFlag)
  {
    READ_FLAG(picHeader->pocMsbPresent, "ph_poc_msb_present_flag");
    if (picHeader->pocMsbPresent)
    {
      READ_CODE(sps->pocMsbLen, uiCode, "ph_poc_msb_val");
      picHeader->pocMsbVal = (uiCode);
    }
  }


  // alf enable flags and aps IDs
  if (sps->alfEnabled)
  {
    if (pps->alfInfoInPh)
    {
      READ_FLAG(picHeader->alfEnabled[COMP_Y], "ph_alf_enabled_flag");

      bool alfCbEnabledFlag = false;
      bool alfCrEnabledFlag = false;
      if (uiCode)
      {
        READ_CODE(3, uiCode, "ph_num_alf_aps_ids_luma");
        int numAps = uiCode;
        picHeader->numAlfAps = (numAps);

        picHeader->alfApsId.resize( numAps );
        for (int i = 0; i < numAps; i++)
        {
          READ_CODE(3, uiCode, "ph_alf_aps_id_luma");
          picHeader->alfApsId[i] = uiCode;
        }

        if (sps->chromaFormatIdc != CHROMA_400)
        {
          READ_FLAG(alfCbEnabledFlag, "ph_alf_cb_enabled_flag");
          READ_FLAG(alfCrEnabledFlag, "ph_alf_cr_enabled_flag");
        }

        if (alfCbEnabledFlag || alfCrEnabledFlag)
        {
          READ_CODE(3, uiCode, "ph_alf_aps_id_chroma");
          picHeader->alfChromaApsId = uiCode;
        }
        if (sps->ccalfEnabled )
        {
          READ_FLAG(picHeader->ccalfEnabled[COMP_Cb], "ph_cc_alf_cb_enabled_flag");
          picHeader->ccalfCbApsId = (-1);
          if (picHeader->ccalfEnabled[COMP_Cb])
          {
            // parse APS ID
            READ_CODE(3, uiCode, "ph_cc_alf_cb_aps_id");
            picHeader->ccalfCbApsId = uiCode;
          }
          // Cr
          READ_FLAG(picHeader->ccalfEnabled[COMP_Cr], "ph_cc_alf_cr_enabled_flag");
          picHeader->ccalfCrApsId = (-1);
          if (picHeader->ccalfEnabled[COMP_Cr])
          {
            // parse APS ID
            READ_CODE(3, uiCode, "ph_cc_alf_cr_aps_id");
            picHeader->ccalfCrApsId = uiCode;
          }
        }
      }
      picHeader->alfEnabled[COMP_Cb]= alfCbEnabledFlag;
      picHeader->alfEnabled[COMP_Cr]= alfCrEnabledFlag;
    }
    else
    {
      picHeader->alfEnabled[COMP_Y] = true;
      picHeader->alfEnabled[COMP_Cb]= true;
      picHeader->alfEnabled[COMP_Cr]= true;
    }
  }

  // luma mapping / chroma scaling controls
  if (sps->lumaReshapeEnable)
  {
    READ_FLAG(picHeader->lmcsEnabled, "ph_lmcs_enabled_flag");

    if (picHeader->lmcsEnabled)
    {
      READ_CODE(2, uiCode, "ph_lmcs_aps_id");
      picHeader->lmcsApsId = uiCode;
      if (sps->chromaFormatIdc != CHROMA_400)
      {
        READ_FLAG(picHeader->lmcsChromaResidualScale, "ph_chroma_residual_scale_flag");
      }
    }
  }

  // quantization scaling lists
  if (sps->scalingListEnabled)
  {
    READ_FLAG(picHeader->explicitScalingListEnabled, "ph_explicit_scaling_list_enabled_flag");
    
    if (picHeader->explicitScalingListEnabled)
    {
      READ_CODE(3, uiCode, "ph_scaling_list_aps_id");
      picHeader->scalingListApsId = uiCode;
    }
  }
  if (pps->picWidthInLumaSamples == sps->maxPicWidthInLumaSamples && pps->picHeightInLumaSamples == sps->maxPicHeightInLumaSamples)
  {
    pps->conformanceWindow = sps->conformanceWindow;
  }
 
  // initialize tile/slice info for no partitioning case
  if( pps->noPicPartition )
  {
    pps->picWidthInCtu  = (pps->picWidthInLumaSamples + (sps->CTUSize-1)) / sps->CTUSize;
    pps->picHeightInCtu = (pps->picHeightInLumaSamples + (sps->CTUSize-1)) / sps->CTUSize;
    pps->log2CtuSize = ( ceilLog2(sps->CTUSize) );
    pps->tileColWidth.push_back(pps->picWidthInCtu );
    pps->tileRowHeight.push_back( pps->picHeightInCtu );
    pps->subPics.clear();
    pps->subPics.resize(1);
    pps->subPics[0].init( pps->picWidthInCtu, pps->picHeightInCtu, pps->picWidthInLumaSamples, pps->picHeightInLumaSamples);
    pps->sliceMap.clear();
    pps->sliceMap.resize(1);
    pps->sliceMap[0].addCtusToSlice(0, pps->picWidthInCtu, 0, pps->picHeightInCtu, pps->picWidthInCtu);
    pps->ctuToTileCol.resize(pps->picWidthInCtu, 0);
    pps->ctuToTileRow.resize(pps->picHeightInCtu, 0);
 
    // when no Pic partition, number of sub picture shall be less than 2
    CHECK(pps->numSubPics>=2, "error, no picture partitions, but have equal to or more than 2 sub pictures");
  }
  else 
  {
    CHECK(pps->ctuSize != sps->CTUSize, "PPS CTU size does not match CTU size in SPS");
  }

  if( pps->wrapAroundEnabled )
  {
    int minCbSizeY = (1 << sps->log2MinCodingBlockSize);
    pps->wrapAroundOffset = (minCbSizeY * (pps->picWidthInLumaSamples/minCbSizeY- pps->picWidthMinusWrapAroundOffset));
  }

  // virtual boundaries
  if( sps->virtualBoundariesEnabled && !sps->virtualBoundariesPresent)
  {
    READ_FLAG( picHeader->virtualBoundariesEnabled, "ph_loop_filter_across_virtual_boundaries_disabled_present_flag" );
    if( picHeader->virtualBoundariesEnabled )
    {
      READ_CODE( 2, picHeader->numVerVirtualBoundaries, "ph_num_ver_virtual_boundaries");
      for( unsigned i = 0; i < picHeader->numVerVirtualBoundaries; i++ )
      {
        READ_UVLC( uiCode, "ph_virtual_boundaries_pos_x");        picHeader->virtualBoundariesPosX[i] = (uiCode << 3);
      }
      READ_CODE( 2, picHeader->numHorVirtualBoundaries, "ph_num_hor_virtual_boundaries");
      for( unsigned i = 0; i < picHeader->numHorVirtualBoundaries; i++ )
      {
        READ_UVLC(uiCode, "ph_virtual_boundaries_pos_y");        picHeader->virtualBoundariesPosY[i] = (uiCode << 3);
      }
    }
  }
  else
  {
    picHeader->virtualBoundariesPresent = sps->virtualBoundariesPresent;
    if( picHeader->virtualBoundariesPresent )
    {
    picHeader->virtualBoundariesEnabled = ( sps->virtualBoundariesEnabled );
    picHeader->numVerVirtualBoundaries = ( sps->numVerVirtualBoundaries );
    picHeader->numHorVirtualBoundaries = ( sps->numHorVirtualBoundaries );
    for( unsigned i = 0; i < 3; i++ ) 
    {
      picHeader->virtualBoundariesPosX[i] = sps->virtualBoundariesPosX[i];
      picHeader->virtualBoundariesPosY[i] = sps->virtualBoundariesPosY[i];
    }
    }
  }
  
  // picture output flag
  if( pps->outputFlagPresent  && !picHeader->nonRefPic)
  {
    READ_FLAG( picHeader->picOutputFlag, "ph_pic_output_flag" );
  }

  // reference picture lists
  if (pps->rplInfoInPh)
  {
    bool rplSpsFlag0 = 0;

    // List0 and List1
    for(int listIdx = 0; listIdx < 2; listIdx++) 
    {                 
      if (sps->getNumRPL(listIdx) > 0 &&
          (listIdx == 0 || (listIdx == 1 && pps->rpl1IdxPresent)))
      {
        READ_FLAG(uiCode, "rpl_sps_flag[i]");
      }
      else if (sps->getNumRPL(listIdx) == 0)
      {
        uiCode = 0;
      }
      else
      {
        uiCode = rplSpsFlag0;
      }

      if (listIdx == 0)
      {
        rplSpsFlag0 = uiCode;
      }

      // explicit RPL in picture header
      if (!uiCode)
      {
        ReferencePictureList* rpl = &picHeader->localRPL[ listIdx ];
        parseRefPicList(sps, rpl, -1);
        picHeader->rplIdx[listIdx] = -1;
        picHeader->pRPL  [listIdx] = rpl;
      }
      // use list from SPS
      else
      {
        if (sps->getNumRPL(listIdx) > 1 &&
            (listIdx == 0 || (listIdx == 1 && pps->rpl1IdxPresent)))
        {
          int numBits = ceilLog2(sps->getNumRPL(listIdx));
          READ_CODE(numBits, uiCode, "rpl_idx[i]");
          picHeader->rplIdx[listIdx] = uiCode;
          picHeader->pRPL[listIdx] =  &sps->rplList[listIdx][uiCode];
        }
        else if( sps->getNumRPL(listIdx) == 1)
        {
          picHeader->rplIdx[listIdx] = 0;
          picHeader->pRPL[listIdx] = &sps->rplList[listIdx][0];
        }
        else
        {
          assert(picHeader->rplIdx[0] != -1);
          picHeader->rplIdx[listIdx] = picHeader->rplIdx[0];
          picHeader->pRPL[ listIdx] = &sps->rplList[ listIdx ][picHeader->rplIdx[listIdx]];
        }
      }

      // POC MSB cycle signalling for LTRP
      for (int i = 0; i < picHeader->pRPL[listIdx]->numberOfLongtermPictures + picHeader->pRPL[listIdx]->numberOfShorttermPictures; i++)
      {
        picHeader->localRPL[listIdx].deltaPocMSBPresent[i] = false;
        picHeader->localRPL[listIdx].deltaPocMSBCycleLT[i] = 0;
      }
      if (picHeader->pRPL[listIdx]->numberOfLongtermPictures)
      {
        for (int i = 0; i < picHeader->pRPL[listIdx]->numberOfLongtermPictures + picHeader->pRPL[listIdx]->numberOfShorttermPictures; i++)
        {
          if (picHeader->pRPL[listIdx]->isLongtermRefPic[i])
          {
            if (picHeader->pRPL[listIdx]->ltrpInSliceHeader)
            {
              READ_CODE(sps->bitsForPOC, uiCode, "poc_lsb_lt[i][j]");
              picHeader->localRPL[listIdx].setRefPicIdentifier( i, uiCode, true, false, 0 );
            }
            READ_FLAG(picHeader->localRPL[listIdx].deltaPocMSBPresent[i], "pic_delta_poc_msb_present_flag[i][j]");
            if (picHeader->localRPL[listIdx].deltaPocMSBPresent[i])
            {
              READ_UVLC(uiCode, "delta_poc_msb_cycle_lt[i][j]");
              picHeader->localRPL[listIdx].deltaPocMSBCycleLT[i] = uiCode;
            }
          }
        }
      }
    }
  }

  // partitioning constraint overrides
  if (sps->partitionOverrideEnabled)
  {
    READ_FLAG(picHeader->splitConsOverride, "partition_constraints_override_flag");
  }

  // Q0781, two-flags
  unsigned  ctbLog2SizeY = floorLog2(sps->CTUSize);

  if (picHeader->picIntraSliceAllowed)
  {
    if (picHeader->splitConsOverride)
    {
      READ_UVLC(uiCode, "ph_log2_diff_min_qt_min_cb_intra_slice_luma");
      unsigned minQtLog2SizeIntraY = uiCode + sps->log2MinCodingBlockSize;
      picHeader->minQTSize[0] = 1 << minQtLog2SizeIntraY;
      READ_UVLC(uiCode, "ph_max_mtt_hierarchy_depth_intra_slice_luma");         picHeader->maxMTTDepth[0] = uiCode;

      picHeader->maxTTSize[0] = picHeader->maxBTSize[0] = picHeader->minQTSize[0];
      if (picHeader->maxMTTDepth[0] != 0)
      {
        READ_UVLC(uiCode, "ph_log2_diff_max_bt_min_qt_intra_slice_luma");       picHeader->maxBTSize[0] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY, "Invalid code");
        READ_UVLC(uiCode, "ph_log2_diff_max_tt_min_qt_intra_slice_luma");       picHeader->maxTTSize[0] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeIntraY, "Invalid code");
      }

      if (sps->dualITree )
      {
        READ_UVLC(uiCode, "ph_log2_diff_min_qt_min_cb_intra_slice_chroma");     picHeader->minQTSize[2] = 1 << (uiCode + sps->log2MinCodingBlockSize);
        READ_UVLC(uiCode, "ph_max_mtt_hierarchy_depth_intra_slice_chroma");     picHeader->maxMTTDepth[2] = uiCode;
        picHeader->maxTTSize[2] = picHeader->maxBTSize[2] = picHeader->minQTSize[2];
        if (picHeader->maxMTTDepth[2] != 0)
        {
          READ_UVLC(uiCode, "ph_log2_diff_max_bt_min_qt_intra_slice_chroma");   picHeader->maxBTSize[2] <<= uiCode;
          READ_UVLC(uiCode, "ph_log2_diff_max_tt_min_qt_intra_slice_chroma");   picHeader->maxTTSize[2] <<= uiCode;
        }
      }
    }
  }

  if (picHeader->picIntraSliceAllowed)
  {
  // delta quantization and chrom and chroma offset
    if (pps->useDQP)
    {
      READ_UVLC( uiCode, "ph_cu_qp_delta_subdiv_intra_slice" );   picHeader->cuQpDeltaSubdivIntra = ( uiCode );
    }
    if (pps->chromaQpOffsetListLen)
    {
      READ_UVLC( uiCode, "ph_cu_chroma_qp_offset_subdiv_intra_slice" );   picHeader->cuChromaQpOffsetSubdivIntra = ( uiCode );
    }
  }

  if (picHeader->picInterSliceAllowed)
  {
    if (picHeader->splitConsOverride)
    {
      READ_UVLC(uiCode, "ph_log2_diff_min_qt_min_cb_inter_slice");
      unsigned minQtLog2SizeInterY = uiCode + sps->log2MinCodingBlockSize;
      picHeader->minQTSize[1] = 1 << minQtLog2SizeInterY;
      READ_UVLC(uiCode, "ph_max_mtt_hierarchy_depth_inter_slice");              picHeader->maxMTTDepth[1] = uiCode;

      picHeader->maxTTSize[1] = picHeader->maxBTSize[1] = picHeader->minQTSize[1];
      if (picHeader->maxMTTDepth[1] != 0)
      {
        READ_UVLC(uiCode, "ph_log2_diff_max_bt_min_qt_inter_slice");            picHeader->maxBTSize[1] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY, "Invalid code");
        READ_UVLC(uiCode, "ph_log2_diff_max_tt_min_qt_inter_slice");            picHeader->maxTTSize[1] <<= uiCode;
        CHECK(uiCode > ctbLog2SizeY - minQtLog2SizeInterY, "Invalid code");
      }
    }
    // delta quantization and chrom and chroma offset
    if (pps->useDQP)
    {
      READ_UVLC(uiCode, "ph_cu_qp_delta_subdiv_inter_slice");   picHeader->cuQpDeltaSubdivInter = (uiCode);
    }
    if (pps->chromaQpOffsetListLen)
    {
      READ_UVLC(uiCode, "ph_cu_chroma_qp_offset_subdiv_inter_slice");   picHeader->cuChromaQpOffsetSubdivInter = (uiCode);
    }

  // temporal motion vector prediction
    if (sps->temporalMVPEnabled)
    {
      READ_FLAG( picHeader->enableTMVP, "ph_temporal_mvp_enabled_flag" );
    }

    if (picHeader->enableTMVP && pps->rplInfoInPh)
    {
      if (picHeader->pRPL[1]->getNumRefEntries() > 0)
      {
      READ_FLAG( picHeader->picColFromL0, "ph_collocated_from_l0_flag");
      }
      else
      {
        picHeader->picColFromL0 = true;
      }
      if ((picHeader->picColFromL0 && picHeader->pRPL[0]->getNumRefEntries() > 1) ||
        (!picHeader->picColFromL0 && picHeader->pRPL[1]->getNumRefEntries() > 1))
      {
        READ_UVLC(uiCode, "ph_collocated_ref_idx");
        picHeader->colRefIdx = (uiCode);
      }

    }

  // subblock merge candidate list size
    if ( sps->Affine )
    {
      picHeader->maxNumAffineMergeCand = ( sps->maxNumAffineMergeCand );
    }
    else
    {
      picHeader->maxNumAffineMergeCand = ( sps->SbtMvp && picHeader->enableTMVP );
    }

  // full-pel MMVD flag
    if (sps->fpelMmvd)
    {
      READ_FLAG( picHeader->disFracMMVD, "ph_fpel_mmvd_enabled_flag" );
    }

  // mvd L1 zero flag
    if (!pps->rplInfoInPh || picHeader->pRPL[1]->getNumRefEntries() > 0)
    {
      READ_FLAG(picHeader->mvdL1Zero, "ph_mvd_l1_zero_flag");
    }
    else
    { 
      picHeader->mvdL1Zero = true;
    }
  
  // picture level BDOF disable flags
    if (sps->BdofPresent && (!pps->rplInfoInPh || picHeader->pRPL[1]->getNumRefEntries() > 0))
    {
      READ_FLAG(picHeader->disBdofFlag, "ph_disable_bdof_flag");
    }
    else if (sps->BdofPresent == 0)
    {
      picHeader->disBdofFlag = !sps->BDOF;
    }
    else
    {
      picHeader->disBdofFlag = true;
    }

  // picture level DMVR disable flags
    if (sps->DmvrPresent && (!pps->rplInfoInPh || picHeader->pRPL[1]->getNumRefEntries() > 0))
    {
      READ_FLAG(picHeader->disDmvrFlag, "ph_disable_dmvr_flag");
    }
    else if (!sps->DmvrPresent)
    {
      picHeader->disDmvrFlag = !sps->DMVR;
    }
    else
    {
      picHeader->disDmvrFlag = true;
    }


  // picture level PROF disable flags
    if (sps->ProfPresent)
    {
      READ_FLAG(picHeader->disProfFlag, "ph_disable_prof_flag");
    }

    if( (pps->weightPred || pps->weightedBiPred) && pps->wpInfoInPh )
    {
      parsePredWeightTable(picHeader, pps, sps);
    }
  }
  // inherit constraint values from SPS
  if (!sps->partitionOverrideEnabled || !picHeader->splitConsOverride)
  {
    memcpy( picHeader->minQTSize, sps->minQTSize, sizeof(sps->minQTSize));
    memcpy( picHeader->maxMTTDepth, sps->maxMTTDepth, sizeof(sps->maxMTTDepth));
    memcpy( picHeader->maxBTSize, sps->maxBTSize, sizeof(sps->maxBTSize));
    memcpy( picHeader->maxTTSize, sps->maxTTSize, sizeof(sps->maxTTSize));
  }

  if (pps->qpDeltaInfoInPh)
  {
    READ_SVLC(picHeader->qpDelta, "ph_qp_delta");
  }

  // joint Cb/Cr sign flag
  if (sps->jointCbCr)
  {
    READ_FLAG( picHeader->jointCbCrSign, "ph_joint_cbcr_sign_flag" ); 
  }

  // sao enable flags
  if(sps->saoEnabled)
  {
    if (pps->saoInfoInPh)
    {    
      READ_FLAG(picHeader->saoEnabled[CH_L], "ph_sao_luma_enabled_flag");

      if (sps->chromaFormatIdc != CHROMA_400)
      {
        READ_FLAG(picHeader->saoEnabled[CH_C], "ph_sao_chroma_enabled_flag");
      }
    }
    else 
    {
      picHeader->saoEnabled[CH_L] = true;
      picHeader->saoEnabled[CH_C] = sps->chromaFormatIdc != CHROMA_400;
    }
  }

  // deblocking filter controls
  if (pps->deblockingFilterControlPresent)
  {
    if(pps->deblockingFilterOverrideEnabled)
    {
      if (pps->dbfInfoInPh)
      {
        READ_FLAG ( picHeader->deblockingFilterOverride, "ph_deblocking_filter_override_flag" );
      }
    }

    if(picHeader->deblockingFilterOverride)
    {
      if (!pps->deblockingFilterDisabled)
      {
        READ_FLAG( picHeader->deblockingFilterDisable, "ph_deblocking_filter_disabled_flag" );
      }
      if (!picHeader->deblockingFilterDisable)
      {
        READ_SVLC( picHeader->deblockingFilterBetaOffsetDiv2[COMP_Y], "ph_beta_offset_div2" );
        CHECK(  picHeader->deblockingFilterBetaOffsetDiv2[COMP_Y] < -12 ||
                picHeader->deblockingFilterBetaOffsetDiv2[COMP_Y] > 12, "Invalid deblocking filter configuration");

        READ_SVLC( picHeader->deblockingFilterTcOffsetDiv2[COMP_Y], "ph_tc_offset_div2" );
        CHECK(  picHeader->deblockingFilterTcOffsetDiv2[COMP_Y] < -12 ||
                picHeader->deblockingFilterTcOffsetDiv2[COMP_Y] > 12, "Invalid deblocking filter configuration");
        if( pps->usePPSChromaTool )
        {
          READ_SVLC( picHeader->deblockingFilterBetaOffsetDiv2[COMP_Cb], "ph_cb_beta_offset_div2" );
          CHECK(  picHeader->deblockingFilterBetaOffsetDiv2[COMP_Cb] < -12 ||
                  picHeader->deblockingFilterBetaOffsetDiv2[COMP_Cb] > 12, "Invalid deblocking filter configuration");

          READ_SVLC( picHeader->deblockingFilterTcOffsetDiv2[COMP_Cb], "ph_cb_tc_offset_div2" );
          CHECK(  picHeader->deblockingFilterTcOffsetDiv2[COMP_Cb] < -12 ||
                  picHeader->deblockingFilterTcOffsetDiv2[COMP_Cb] > 12, "Invalid deblocking filter configuration");

          READ_SVLC( picHeader->deblockingFilterBetaOffsetDiv2[COMP_Cr], "ph_cr_beta_offset_div2" );
          CHECK(  picHeader->deblockingFilterBetaOffsetDiv2[COMP_Cr] < -12 ||
                  picHeader->deblockingFilterBetaOffsetDiv2[COMP_Cr] > 12, "Invalid deblocking filter configuration");

          READ_SVLC( picHeader->deblockingFilterTcOffsetDiv2[COMP_Cr], "ph_cr_tc_offset_div2" );
          CHECK(  picHeader->deblockingFilterTcOffsetDiv2[COMP_Cr] < -12 ||
                  picHeader->deblockingFilterTcOffsetDiv2[COMP_Cr] > 12, "Invalid deblocking filter configuration");
        }
        else
        {
          for( int comp = 1; comp < MAX_NUM_COMP; comp++)
          {
            picHeader->deblockingFilterBetaOffsetDiv2[comp] = picHeader->deblockingFilterBetaOffsetDiv2[COMP_Y];
            picHeader->deblockingFilterTcOffsetDiv2[comp]   = picHeader->deblockingFilterTcOffsetDiv2[COMP_Y];
          } 
        }
      }
    }
    else
    {
      picHeader->deblockingFilterDisable = pps->deblockingFilterDisabled;
      for( int comp = 0; comp < MAX_NUM_COMP; comp++)
      {
        picHeader->deblockingFilterBetaOffsetDiv2[comp] = pps->deblockingFilterBetaOffsetDiv2[comp];
        picHeader->deblockingFilterTcOffsetDiv2[comp]   = pps->deblockingFilterTcOffsetDiv2[comp];
      } 
    }
  }


  // picture header extension
  if(pps->pictureHeaderExtensionPresent)
  {
    READ_UVLC(uiCode,"ph_extension_length");
    for(int i=0; i<uiCode; i++)
    {
      uint32_t ignore_;
      READ_CODE(8,ignore_,"ph_extension_data_byte");
    }
  }

  if( readRbspTrailingBits )
  {
    xReadRbspTrailingBits();
  }
}

void HLSyntaxReader::parseSliceHeader (Slice* pcSlice, PicHeader* picHeader, ParameterSetManager *parameterSetManager, const int prevTid0POC)
{
  Slice* slice=pcSlice;
  uint32_t  uiCode;
  int   iCode;

  DTRACE( g_trace_ctx, D_HEADER, "=========== Slice ===========\n" );

  PPS* pps = NULL;
  SPS* sps = NULL;
  READ_FLAG(uiCode, "picture_header_in_slice_header_flag");
  if (uiCode)
  {
    parsePictureHeader(picHeader, parameterSetManager, false);
  }

  CHECK(picHeader==0, "Invalid Picture Header");
  pps = parameterSetManager->getPPS( picHeader->ppsId );
  //!KS: need to add error handling code here, if PPS is not available
  CHECK(pps==0, "Invalid PPS");
  sps = parameterSetManager->getSPS(pps->spsId);
  //!KS: need to add error handling code here, if SPS is not available
  CHECK(sps==0, "Invalid SPS");

  const ChromaFormat chFmt = sps->chromaFormatIdc;
  const uint32_t numValidComp=getNumberValidComponents(chFmt);
  const bool bChroma=(chFmt!=CHROMA_400);

  // picture order count
  uiCode = picHeader->pocLsb;
  int iPOClsb = uiCode;
  int iMaxPOClsb = 1 << sps->bitsForPOC;
  int iPOCmsb;
  if (pcSlice->getIdrPicFlag())
  {
    if (picHeader->pocMsbPresent)
    {
      iPOCmsb = picHeader->pocMsbVal*iMaxPOClsb;
    }
    else
    {
      iPOCmsb = 0;
    }
    pcSlice->poc = (iPOCmsb + iPOClsb);
  }
  else
  {
    int iPrevPOC = prevTid0POC;
    int iPrevPOClsb = iPrevPOC & (iMaxPOClsb - 1);
    int iPrevPOCmsb = iPrevPOC - iPrevPOClsb;
    if (picHeader->pocMsbPresent)
    {
      iPOCmsb = picHeader->pocMsbVal*iMaxPOClsb;
    }
    else
    {
      if ((iPOClsb < iPrevPOClsb) && ((iPrevPOClsb - iPOClsb) >= (iMaxPOClsb / 2)))
      {
        iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
      }
      else if ((iPOClsb > iPrevPOClsb) && ((iPOClsb - iPrevPOClsb) > (iMaxPOClsb / 2)))
      {
        iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
      }
      else
      {
        iPOCmsb = iPrevPOCmsb;
      }
    }
    pcSlice->poc = (iPOCmsb + iPOClsb);
  }

  if (sps->subPicInfoPresent)
  {
    uint32_t bitsSubPicId;
    if (sps->subPicIdMappingExplicitlySignalled)
    {
      bitsSubPicId = sps->subPicIdLen;
    }
    else if (pps->subPicIdMappingInPps)
    {
      bitsSubPicId = pps->subPicIdLen;
    }
    else
    {
      bitsSubPicId = ceilLog2(sps->numSubPics);
    }
    READ_CODE(bitsSubPicId, uiCode, "slice_subpic_id");    pcSlice->sliceSubPicId = (uiCode);
  }

  // raster scan slices
  if(!pps->rectSlice) 
  {
  }
  // rectangular slices
  else 
  {
    uint32_t sliceAddr;

    // slice address is the index of the slice within the current sub-picture
    uint32_t currSubPicIdx = pps->getSubPicIdxFromSubPicId( pcSlice->sliceSubPicId );
    SubPic currSubPic = pps->subPics[currSubPicIdx];
    if( currSubPic.numSlicesInSubPic > 1 )
    {
      int bitsSliceAddress = ceilLog2(currSubPic.numSlicesInSubPic);  
      READ_CODE(bitsSliceAddress, uiCode, "slice_address");  sliceAddr = uiCode;
      CHECK(sliceAddr >= pps->numSlicesInPic, "Invalid slice address");
    }
    else
    {
      sliceAddr = 0;
    }
    uint32_t picLevelSliceIdx = sliceAddr;
    for(int subpic = 0; subpic < currSubPicIdx; subpic++)
    {
      picLevelSliceIdx += pps->subPics[subpic].numSlicesInSubPic;
    }
    pcSlice->sliceMap = ( pps->sliceMap[picLevelSliceIdx] );
    pcSlice->sliceSubPicId = (picLevelSliceIdx);
  }

  std::vector<bool> shExtraBitsPresent = sps->extraSHBitPresent;
  for (int i=0; i< sps->numExtraSHBitsBytes * 8; i++)
  {
    // extra bits are ignored (when present)
    if (shExtraBitsPresent[i])
    {
      READ_FLAG(uiCode, "sh_extra_bit[ i ]");
    }
  }

  if (picHeader->picInterSliceAllowed)
  {
    READ_UVLC (    uiCode, "slice_type" );            pcSlice->sliceType = ((SliceType)uiCode);
  }
  else
  {
    pcSlice->sliceType = (I_SLICE);
  }
  if (!picHeader->picIntraSliceAllowed)
  {
    CHECK(pcSlice->sliceType == I_SLICE, "when ph_intra_slice_allowed_flag = 0, no I_Slice is allowed");
  }

  //   set default values in case slice overrides are disabled
  if (pps->rplInfoInPh)
  {
    for( int i = 0; i < 2; i++)
    {
      pcSlice->rplIdx[i]   = picHeader->rplIdx[i];
      pcSlice->rplLocal[i] = picHeader->localRPL[i];
      if(pcSlice->rplIdx[i] != -1)
      {
        pcSlice->rpl[i] = &sps->rplList[i][pcSlice->rplIdx[i]];
      }
      else
      {
        pcSlice->rpl[1] = &pcSlice->rplLocal[1];
      }
    }
  }

  for( int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
    pcSlice->deblockingFilterBetaOffsetDiv2[comp] = picHeader->deblockingFilterBetaOffsetDiv2[comp];
    pcSlice->deblockingFilterTcOffsetDiv2[comp]   = picHeader->deblockingFilterTcOffsetDiv2[comp];
  } 
  pcSlice->deblockingFilterDisable          = picHeader->deblockingFilterDisable;
  pcSlice->saoEnabled[CH_L]                 = picHeader->saoEnabled[CH_L];
  pcSlice->saoEnabled[CH_C]                 = picHeader->saoEnabled[CH_C];

  pcSlice->tileGroupAlfEnabled[COMP_Y]      = picHeader->alfEnabled[COMP_Y];
  pcSlice->tileGroupAlfEnabled[COMP_Cb]     = picHeader->alfEnabled[COMP_Cb];
  pcSlice->tileGroupAlfEnabled[COMP_Cr]     = picHeader->alfEnabled[COMP_Cr];
  pcSlice->tileGroupNumAps                  = picHeader->numAlfAps;
  pcSlice->tileGroupChromaApsId             = picHeader->alfChromaApsId;
  pcSlice->tileGroupCcAlfCbEnabled          = picHeader->ccalfEnabled[COMP_Cb];
  pcSlice->tileGroupCcAlfCrEnabled          = picHeader->ccalfEnabled[COMP_Cr];
  pcSlice->tileGroupCcAlfCbApsId            = picHeader->ccalfCbApsId;
  pcSlice->tileGroupCcAlfCrApsId            = picHeader->ccalfCrApsId;
  pcSlice->ccAlfFilterParam.ccAlfFilterEnabled[COMP_Cb - 1] = picHeader->ccalfEnabled[COMP_Cb];
  pcSlice->ccAlfFilterParam.ccAlfFilterEnabled[COMP_Cr - 1] = picHeader->ccalfEnabled[COMP_Cr];

  if (pcSlice->nalUnitType == NAL_UNIT_CODED_SLICE_CRA || pcSlice->nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP || pcSlice->nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL || pcSlice->nalUnitType == NAL_UNIT_CODED_SLICE_GDR)
  {
    READ_FLAG(picHeader->noOutputOfPriorPics, "sh_no_output_of_prior_pics_flag");
  }

  if (sps->alfEnabled && !pps->alfInfoInPh)
  {
    READ_FLAG(uiCode, "sh_alf_enabled_flag");
    pcSlice->tileGroupAlfEnabled[COMP_Y] = uiCode;
    bool alfCbEnabledFlag = false;
    bool alfCrEnabledFlag = false;
    if (uiCode)
    {
      READ_CODE(3, uiCode, "sh_num_alf_aps_ids_luma");
      int numAps = uiCode;
      pcSlice->tileGroupNumAps = (numAps);
      std::vector<int> apsId(numAps, -1);
      for (int i = 0; i < numAps; i++)
      {
        READ_CODE(3, uiCode, "sh_alf_aps_id_luma");
        apsId[i] = uiCode;
        APS* APStoCheckLuma = parameterSetManager->getAPS(apsId[i], ALF_APS);
        CHECK(APStoCheckLuma == nullptr, "referenced APS not found");
        CHECK(APStoCheckLuma->alfParam.newFilterFlag[CH_L] != 1, "bitstream conformance error, alf_luma_filter_signal_flag shall be equal to 1");
      }


      pcSlice->setAlfApsIds(apsId);
      if (bChroma)
      {
        READ_FLAG( alfCbEnabledFlag, "sh_alf_cb_enabled_flag");
        READ_FLAG( alfCrEnabledFlag, "sh_alf_cr_enabled_flag");
      }
      if (alfCbEnabledFlag || alfCrEnabledFlag)
      {
        READ_CODE(3, uiCode, "sh_alf_aps_id_chroma");
        pcSlice->tileGroupChromaApsId = uiCode;
        APS* APStoCheckChroma = parameterSetManager->getAPS(uiCode, ALF_APS);
        CHECK(APStoCheckChroma->alfParam.newFilterFlag[CH_C] != 1, "bitstream conformance error, alf_chroma_filter_signal_flag shall be equal to 1");
        pcSlice->ccAlfFilterParam = APStoCheckChroma->ccAlfParam; 
      }
    }
    else
    {
      pcSlice->tileGroupNumAps = (0);
    }
    pcSlice->tileGroupAlfEnabled[COMP_Cb] = alfCbEnabledFlag;
    pcSlice->tileGroupAlfEnabled[COMP_Cr] = alfCrEnabledFlag;

    if (sps->ccalfEnabled && pcSlice->tileGroupAlfEnabled[COMP_Y])
    {
      READ_FLAG(pcSlice->tileGroupCcAlfCbEnabled, "sh_cc_alf_cb_enabled_flag");

      if (pcSlice->tileGroupCcAlfCbEnabled)
      {
        // parse APS ID
        READ_CODE(3, uiCode, "sh_cc_alf_cb_aps_id");
        pcSlice->tileGroupCcAlfCbApsId = (uiCode);
        pcSlice->ccAlfFilterParam = parameterSetManager->getAPS( uiCode, ALF_APS )->ccAlfParam;
      }
      // Cr
      READ_FLAG(pcSlice->tileGroupCcAlfCrEnabled, "sh_cc_alf_cr_enabled_flag");
      if (pcSlice->tileGroupCcAlfCrEnabled)
      {
        // parse APS ID
        READ_CODE(3, uiCode, "sh_cc_alf_cr_aps_id");
        pcSlice->tileGroupCcAlfCrApsId = (uiCode);
        pcSlice->ccAlfFilterParam = parameterSetManager->getAPS( uiCode, ALF_APS )->ccAlfParam;
      }
    }
  }

  if (picHeader->lmcsEnabled && !pcSlice->pictureHeaderInSliceHeader)
  {
    READ_FLAG(pcSlice->lmcsEnabled, "sh_lmcs_enabled_flag");
  }
  else
  {
    pcSlice->lmcsEnabled = (pcSlice->pictureHeaderInSliceHeader ? picHeader->lmcsEnabled : false);
  }
  if (picHeader->explicitScalingListEnabled && !pcSlice->pictureHeaderInSliceHeader)
  {
    READ_FLAG(pcSlice->explicitScalingListUsed, "sh_explicit_scaling_list_used_flag");
  }
  else
  {
    pcSlice->explicitScalingListUsed = pcSlice->pictureHeaderInSliceHeader ? picHeader->explicitScalingListEnabled : false;
  }

  if( pps->rplInfoInPh )
  {
    pcSlice->rpl[0] = (picHeader->pRPL[0]);
    pcSlice->rpl[1] = (picHeader->pRPL[1]);
    pcSlice->rplLocal[0] = picHeader->localRPL[0];
    pcSlice->rplLocal[1] = picHeader->localRPL[1];
  }
  else if( slice->getIdrPicFlag() && !sps->idrRefParamList)
  {
    ReferencePictureList* rpl0 = &slice->rplLocal[0];
    (*rpl0) = ReferencePictureList();
    slice->rpl[0] = rpl0;
    ReferencePictureList* rpl1 = &slice->rplLocal[1];
    (*rpl1) = ReferencePictureList();
    slice->rpl[1] = rpl1;
  }
  else 
  {
    //Read L0 related syntax elements
    bool rplSpsFlag0 = 0;

    if (sps->getNumRPL(0) > 0)
    {
      READ_FLAG(uiCode, "ref_pic_list_sps_flag[0]");
    }
    else
    {
      uiCode = 0;
    }

    rplSpsFlag0 = uiCode;

    if (!uiCode) //explicitly carried in this SH
    {
      ReferencePictureList* rpl0 = &slice->rplLocal[0];
      parseRefPicList(sps, rpl0, -1);
      slice->rplIdx[0] = -1;
      slice->rpl[0] = rpl0;
    }
    else    //Refer to list in SPS
    {
      if (sps->getNumRPL(0) > 1)
      {
        int numBits = ceilLog2(sps->getNumRPL(0));
        READ_CODE(numBits, uiCode, "ref_pic_list_idx[0]");
        slice->rplIdx[0] = uiCode;
        slice->rpl[0] = &sps->rplList[0][uiCode];
      }
      else
      {
        slice->rplIdx[0] = 0;
        slice->rpl[0] = &sps->rplList[0][0];
      }
    }
    //Deal POC Msb cycle signalling for LTRP
    for (int i = 0; i < slice->rpl[0]->numberOfLongtermPictures + slice->rpl[0]->numberOfShorttermPictures; i++)
    {
      slice->rplLocal[0].deltaPocMSBPresent[i] = false;
      slice->rplLocal[0].deltaPocMSBCycleLT[i] = 0;
    }
    if (slice->rpl[0]->numberOfLongtermPictures)
    {
      for (int i = 0; i < slice->rpl[0]->numberOfLongtermPictures + slice->rpl[0]->numberOfShorttermPictures; i++)
      {
        if (slice->rpl[0]->isLongtermRefPic[i])
        {
          if (slice->rpl[0]->ltrpInSliceHeader)
          {
            READ_CODE(sps->bitsForPOC, uiCode, "slice_poc_lsb_lt[i][j]");
            slice->rplLocal[0].setRefPicIdentifier(i, uiCode, true, false, 0);
          }
          READ_FLAG(slice->rplLocal[0].deltaPocMSBPresent[i], "delta_poc_msb_present_flag[i][j]");
          
          if (slice->rplLocal[0].deltaPocMSBPresent[i])
          {
            READ_UVLC(slice->rplLocal[0].deltaPocMSBCycleLT[i], "delta_poc_msb_cycle_lt[i][j]");
          }
        }
      }
     // th check bunch of else cond
    }

    //Read L1 related syntax elements
    if (sps->getNumRPL(1) > 0 && pps->rpl1IdxPresent)
    {
      READ_FLAG(uiCode, "ref_pic_list_sps_flag[1]");
    }
    else if (sps->getNumRPL(1) == 0)
    {
      uiCode = 0;
    }
    else
    {
      uiCode = rplSpsFlag0;
    }

    if (uiCode == 1)
    {
      if (sps->getNumRPL(1) > 1 && pps->rpl1IdxPresent)
      {
        int numBits = ceilLog2(sps->getNumRPL(1));
        READ_CODE(numBits, uiCode, "ref_pic_list_idx[1]");
        slice->rplIdx[1] = uiCode;
        slice->rpl[1] = &sps->rplList[1][uiCode];
      }
      else if (sps->getNumRPL(1) == 1)
      {
        slice->rplIdx[1] = 0;
        slice->rpl[1] = &sps->rplList[1][0];
      }
      else
      {
        assert(pcSlice->rplIdx[0] != -1);
        pcSlice->rplIdx[1] = pcSlice->rplIdx[0];
        pcSlice->rpl[1] = &sps->rplList[1][pcSlice->rplIdx[0]];
      }
    }
    else
    {
      ReferencePictureList* rpl1 = &slice->rplLocal[1];
      parseRefPicList(sps, rpl1, -1);
      slice->rplIdx[1] = -1;
      slice->rpl[1] = rpl1;
    }

    //Deal POC Msb cycle signalling for LTRP
    for (int i = 0; i < slice->rpl[1]->numberOfLongtermPictures + slice->rpl[1]->numberOfShorttermPictures; i++)
    {
      slice->rplLocal[1].deltaPocMSBPresent[i] = false;
      slice->rplLocal[1].deltaPocMSBCycleLT[i] = 0;
    }
    if (slice->rpl[1]->numberOfLongtermPictures)
    {
      for (int i = 0; i < slice->rpl[1]->numberOfLongtermPictures + slice->rpl[1]->numberOfShorttermPictures; i++)
      {
        if (slice->rpl[1]->isLongtermRefPic[i])
        {
          if (slice->rpl[1]->ltrpInSliceHeader)
          {
            READ_CODE(sps->bitsForPOC, uiCode, "slice_poc_lsb_lt[i][j]");
            slice->rplLocal[1].setRefPicIdentifier(i, uiCode, true,false,0);
          }
          READ_FLAG(slice->rplLocal[1].deltaPocMSBPresent[i], "delta_poc_msb_present_flag[i][j]");
          if (slice->rplLocal[1].deltaPocMSBPresent[i])
          {
            READ_UVLC(slice->rplLocal[1].deltaPocMSBCycleLT[i], "slice_delta_poc_msb_cycle_lt[i][j]");
          }
        }
      }
    }
    // th check bunch of else cond
  }
  if( !pps->rplInfoInPh && pcSlice->getIdrPicFlag() && !(sps->idrRefParamList) )
  {
    pcSlice->numRefIdx[REF_PIC_LIST_0] =  0;
    pcSlice->numRefIdx[REF_PIC_LIST_1] =  0;
  }

  if ((!pcSlice->isIntra() && pcSlice->rpl[0]->getNumRefEntries() > 1) ||
      (pcSlice->isInterB() && pcSlice->rpl[1]->getNumRefEntries() > 1) )
  {
    READ_FLAG( uiCode, "sh_num_ref_idx_active_override_flag");
    if (uiCode)
    {
      if(slice->rpl[0]->getNumRefEntries() > 1)
      {
        READ_UVLC (uiCode, "sh_num_ref_idx_l0_active_minus1" );
      }
      else
      {
        uiCode = 0;
      }
      slice->numRefIdx[ REF_PIC_LIST_0] = ( uiCode + 1 );
      if (slice->isInterB())
      {
        if(slice->rpl[1]->getNumRefEntries() > 1)
        {
          READ_UVLC (uiCode, "sh_num_ref_idx_l1_active_minus1" );
        }
        else
        {
          uiCode = 0;
        }
        slice->numRefIdx[ REF_PIC_LIST_1 ] = ( uiCode + 1 );
      }
      else
      {
        slice->numRefIdx[REF_PIC_LIST_1] = 0;
      }
    }
    else
    {
      if(slice->rpl[0]->getNumRefEntries() >= pps->numRefIdxL0DefaultActive)
      {
        slice->numRefIdx[REF_PIC_LIST_0] = pps->numRefIdxL0DefaultActive;
      }
      else
      {
        slice->numRefIdx[REF_PIC_LIST_0] = slice->rpl[0]->getNumRefEntries();
      }

      if (slice->isInterB())
      {
        if(slice->rpl[1]->getNumRefEntries() >= pps->numRefIdxL1DefaultActive)
        {
          slice->numRefIdx[REF_PIC_LIST_1] = pps->numRefIdxL1DefaultActive;
        }
        else
        {
          slice->numRefIdx[REF_PIC_LIST_1] = slice->rpl[1]->getNumRefEntries();
        }
      }
      else
      {
        slice->numRefIdx[REF_PIC_LIST_1] = 0;
      }
    }
  }
  else
  {
    slice->numRefIdx[REF_PIC_LIST_0] = slice->isIntra() ? 0 : slice->rpl[0]->getNumRefEntries();
    slice->numRefIdx[REF_PIC_LIST_1] = slice->isInterB() ? slice->rpl[1]->getNumRefEntries() : 0;
  }

  if (slice->isInterP() || slice->isInterB())
  {
    CHECK(slice->numRefIdx[REF_PIC_LIST_0] == 0, "Number of active entries in RPL0 of P or B picture shall be greater than 0");
    if (slice->isInterB())
      CHECK(slice->numRefIdx[REF_PIC_LIST_1] == 0, "Number of active entries in RPL1 of B picture shall be greater than 0");
  }

 
  slice->cabacInitFlag = false; // default
  if(pps->cabacInitPresent && !slice->isIntra())
  {
    READ_FLAG(slice->cabacInitFlag, "sh_cabac_init_flag");
    slice->encCABACTableIdx = ( slice->sliceType == B_SLICE ? ( slice->cabacInitFlag ? P_SLICE : B_SLICE ) : ( slice->cabacInitFlag ? B_SLICE : P_SLICE ) );
  }

  if ( picHeader->enableTMVP )
  {
    if( pcSlice->sliceType == P_SLICE )
    {
      pcSlice->colFromL0Flag = true;
    }
    else if( !pps->rplInfoInPh && pcSlice->sliceType == B_SLICE )
    {
      READ_FLAG( pcSlice->colFromL0Flag, "sh_collocated_from_l0_flag" );
    }
    else
    {
      pcSlice->colFromL0Flag = picHeader->picColFromL0;
    }

    if (!pps->rplInfoInPh)
    {
      if ( slice->sliceType != I_SLICE &&
            ((slice->colFromL0Flag == 1 && slice->numRefIdx[ REF_PIC_LIST_0 ] > 1)||
            (slice->colFromL0Flag == 0 && slice->numRefIdx[ REF_PIC_LIST_1 ] > 1)))
      {
        READ_UVLC( uiCode, "collocated_ref_idx" );
        slice->colRefIdx = uiCode;
      }
      else
      {
        slice->colRefIdx = 0;
      }
    }
  }
  if ( (pps->weightPred && slice->sliceType==P_SLICE) || (pps->weightedBiPred && slice->sliceType==B_SLICE) )
  {
    if (pps->wpInfoInPh)
    {
      THROW("no support");
    }
    else
    {
      parsePredWeightTable(pcSlice, sps);
    }
  }
 
  int qpDelta = 0;
  if (pps->qpDeltaInfoInPh)
  {
    qpDelta = picHeader->qpDelta;
  }
  else
  {
    READ_SVLC(iCode, "sh_slice_qp_delta");
    qpDelta = iCode;
  }
  pcSlice->sliceQp = (26 + pps->picInitQPMinus26 + qpDelta);

  CHECK( slice->sliceQp < -sps->qpBDOffset[ CH_L ], "Invalid slice QP delta" );
  CHECK( slice->sliceQp > MAX_QP, "Invalid slice QP" );

  if (pps->sliceChromaQpFlag)
  {
    if (numValidComp>COMP_Cb)
    {
      READ_SVLC( iCode, "sh_cb_qp_offset" );
      slice->sliceChromaQpDelta[COMP_Cb] = iCode;
      CHECK( slice->sliceChromaQpDelta[COMP_Cb] < -12, "Invalid chroma QP offset" );
      CHECK( slice->sliceChromaQpDelta[COMP_Cb] >  12, "Invalid chroma QP offset" );
      CHECK( (pps->chromaQpOffset[COMP_Cb] + slice->sliceChromaQpDelta[COMP_Cb]) < -12, "Invalid chroma QP offset" );
      CHECK( (pps->chromaQpOffset[COMP_Cb] + slice->sliceChromaQpDelta[COMP_Cb]) >  12, "Invalid chroma QP offset" );
    }

    if (numValidComp>COMP_Cr)
    {
      READ_SVLC( iCode, "sh_cr_qp_offset" );
      slice->sliceChromaQpDelta[COMP_Cr] = iCode;
      CHECK( slice->sliceChromaQpDelta[COMP_Cr] < -12, "Invalid chroma QP offset" );
      CHECK( slice->sliceChromaQpDelta[COMP_Cr] >  12, "Invalid chroma QP offset" );
      CHECK( (pps->chromaQpOffset[COMP_Cr] + slice->sliceChromaQpDelta[COMP_Cr]) < -12, "Invalid chroma QP offset" );
      CHECK( (pps->chromaQpOffset[COMP_Cr] + slice->sliceChromaQpDelta[COMP_Cr]) >  12, "Invalid chroma QP offset" );
      if (sps->jointCbCr)
      {
        READ_SVLC(iCode, "sh_joint_cbcr_qp_offset" );
        slice->sliceChromaQpDelta[COMP_JOINT_CbCr] =  iCode;
        CHECK( slice->sliceChromaQpDelta[COMP_JOINT_CbCr] < -12, "Invalid chroma QP offset");
        CHECK( slice->sliceChromaQpDelta[COMP_JOINT_CbCr] >  12, "Invalid chroma QP offset");
        CHECK( (pps->chromaQpOffset[COMP_JOINT_CbCr] + slice->sliceChromaQpDelta[COMP_JOINT_CbCr]) < -12, "Invalid chroma QP offset");
        CHECK( (pps->chromaQpOffset[COMP_JOINT_CbCr] + slice->sliceChromaQpDelta[COMP_JOINT_CbCr]) >  12, "Invalid chroma QP offset");
      }
    }
  }

  if (pps->chromaQpOffsetListLen>0)
  {
    READ_FLAG(slice->chromaQpAdjEnabled, "sh_cu_chroma_qp_offset_enabled_flag");
  }
  else
  {
    slice->chromaQpAdjEnabled = false;
  }


  if(sps->saoEnabled )
  {
    READ_FLAG(slice->saoEnabled[CH_L], "sh_slice_sao_luma_flag");
    if (bChroma)
    {
      READ_FLAG(slice->saoEnabled[CH_C], "sh_slice_sao_chroma_flag");
    }
  }

  if (pps->deblockingFilterControlPresent)
  {
    if(pps->deblockingFilterOverrideEnabled&& !pps->dbfInfoInPh)
    {
      READ_FLAG ( slice->deblockingFilterOverrideFlag, "sh_deblocking_filter_params_present_flag" );
    }

    if(slice->deblockingFilterOverrideFlag)
    {
      if (!pps->deblockingFilterDisabled )
      {
        READ_FLAG ( slice->deblockingFilterDisable, "sh_deblocking_filter_disabled_flag" );
      }
      if(!slice->deblockingFilterDisable)
      {
        READ_SVLC( slice->deblockingFilterBetaOffsetDiv2[COMP_Y], "sh_beta_offset_div2" );
        CHECK(  slice->deblockingFilterBetaOffsetDiv2[COMP_Y] < -12 &&
                slice->deblockingFilterBetaOffsetDiv2[COMP_Y] >  12, "Invalid deblocking filter configuration");
        READ_SVLC( slice->deblockingFilterTcOffsetDiv2[COMP_Y], "sh_tc_offset_div2" );
        CHECK  (slice->deblockingFilterTcOffsetDiv2[COMP_Y] < -12 &&
                slice->deblockingFilterTcOffsetDiv2[COMP_Y] >  12, "Invalid deblocking filter configuration");
        if( pps->usePPSChromaTool )         
        {
          READ_SVLC( iCode, "sh_cb_beta_offset_div2" );                  pcSlice->deblockingFilterBetaOffsetDiv2[COMP_Cb] = ( iCode );
          CHECK(  pcSlice->deblockingFilterBetaOffsetDiv2[COMP_Cb] < -12 ||
                  pcSlice->deblockingFilterBetaOffsetDiv2[COMP_Cb] > 12, "Invalid deblocking filter configuration");
          READ_SVLC( iCode, "sh_cb_tc_offset_div2" );                    pcSlice->deblockingFilterTcOffsetDiv2[COMP_Cb] = ( iCode );
          CHECK(  pcSlice->deblockingFilterTcOffsetDiv2[COMP_Cb] < -12 ||
                  pcSlice->deblockingFilterTcOffsetDiv2[COMP_Cb] > 12, "Invalid deblocking filter configuration");

          READ_SVLC( iCode, "sh_cr_beta_offset_div2" );                  pcSlice->deblockingFilterBetaOffsetDiv2[COMP_Cr] = ( iCode );
          CHECK(  pcSlice->deblockingFilterBetaOffsetDiv2[COMP_Cr] < -12 ||
                  pcSlice->deblockingFilterBetaOffsetDiv2[COMP_Cr] > 12, "Invalid deblocking filter configuration");
          READ_SVLC( iCode, "sh_cr_tc_offset_div2" );                    pcSlice->deblockingFilterTcOffsetDiv2[COMP_Cr] = ( iCode );
          CHECK(  pcSlice->deblockingFilterTcOffsetDiv2[COMP_Cr] < -12 ||
                  pcSlice->deblockingFilterTcOffsetDiv2[COMP_Cr] > 12, "Invalid deblocking filter configuration");
         }
         else
         {
            for( int comp = 1; comp < MAX_NUM_COMP; comp++)
            {
              slice->deblockingFilterBetaOffsetDiv2[comp] = slice->deblockingFilterBetaOffsetDiv2[COMP_Y];
              slice->deblockingFilterTcOffsetDiv2[comp]   = slice->deblockingFilterTcOffsetDiv2[COMP_Y];
            }
         }
      }
      else
      {
        slice->deblockingFilterDisable  = pps->deblockingFilterDisabled;
        for( int comp = 0; comp < MAX_NUM_COMP; comp++)
        {
          slice->deblockingFilterBetaOffsetDiv2[comp] = pps->deblockingFilterBetaOffsetDiv2[comp];
          slice->deblockingFilterTcOffsetDiv2[comp]   = pps->deblockingFilterTcOffsetDiv2[comp];
        } 
      }
    }
  }
    // dependent quantization
  if( sps->depQuantEnabled )
  {
    READ_FLAG(pcSlice->depQuantEnabled, "sh_dep_quant_enabled_flag");
  }

  // sign data hiding
  if( sps->signDataHidingEnabled && !pcSlice->depQuantEnabled )
  {
    READ_FLAG( pcSlice->signDataHidingEnabled, "sh_sign_data_hiding_enabled_flag" );
  }

  // signal TS residual coding disabled flag
  if (sps->transformSkip && !pcSlice->depQuantEnabled && !pcSlice->signDataHidingEnabled)
  {
    READ_FLAG(pcSlice->tsResidualCodingDisabled, "sh_ts_residual_coding_disabled_flag");
  }

  pcSlice->setDefaultClpRng( *sps );

  if(pps->sliceHeaderExtensionPresent)
  {
    READ_UVLC(uiCode,"sh_slice_header_extension_length");
    for(int i=0; i<uiCode; i++)
    {
      uint32_t ignore_;
      READ_CODE(8,ignore_,"sh_slice_header_extension_data_byte");
    }
  }

  int numEntryPoints = slice->getNumEntryPoints( *sps, *pps );
  std::vector<uint32_t> entryPointOffset;
  if( numEntryPoints > 0 )
  {
    uint32_t offsetLenMinus1;
    READ_UVLC( offsetLenMinus1, "sh_entry_offset_len_minus1" );
    entryPointOffset.resize( numEntryPoints );
    for( int idx = 0; idx < numEntryPoints; idx++ )
    {
      READ_CODE( offsetLenMinus1 + 1, uiCode, "sh_entry_point_offset_minus1" );
      entryPointOffset[idx] = uiCode + 1;
    }
  }

  m_pcBitstream->readByteAlignment();

  slice->clearSubstreamSizes();

  if( numEntryPoints > 0 )
  {
    int endOfSliceHeaderLocation = m_pcBitstream->getByteLocation();

    // Adjust endOfSliceHeaderLocation to account for emulation prevention bytes in the slice segment header
    for ( uint32_t curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
    {
      if ( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) < endOfSliceHeaderLocation )
      {
        endOfSliceHeaderLocation++;
      }
    }

    int  curEntryPointOffset     = 0;
    int  prevEntryPointOffset    = 0;
    for (uint32_t idx=0; idx<entryPointOffset.size(); idx++)
    {
      curEntryPointOffset += entryPointOffset[ idx ];

      int emulationPreventionByteCount = 0;
      for ( uint32_t curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
      {
        if ( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) >= ( prevEntryPointOffset + endOfSliceHeaderLocation ) &&
             m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) <  ( curEntryPointOffset  + endOfSliceHeaderLocation ) )
        {
          emulationPreventionByteCount++;
        }
      }

      entryPointOffset[ idx ] -= emulationPreventionByteCount;
      prevEntryPointOffset = curEntryPointOffset;
      slice->addSubstreamSize(entryPointOffset [ idx ] );
    }
  }
  return;
}

void HLSyntaxReader::getSlicePoc(Slice* pcSlice, PicHeader* picHeader, ParameterSetManager *parameterSetManager, const int prevTid0POC)
{
  uint32_t  uiCode;
  uint32_t  pocLsb;
  PPS* pps = NULL;
  SPS* sps = NULL;

  CHECK(picHeader==0, "Invalid Picture Header");
  pps = parameterSetManager->getPPS( picHeader->ppsId );
  //!KS: need to add error handling code here, if PPS is not available
  CHECK(pps==0, "Invalid PPS");
  sps = parameterSetManager->getSPS(pps->spsId);
  //!KS: need to add error handling code here, if SPS is not available
  CHECK(sps==0, "Invalid SPS");

  READ_FLAG(uiCode, "sh_picture_header_in_slice_header_flag");
  if (uiCode == 0)
  {
    pocLsb = picHeader->pocLsb;
  }
  else
  {
    READ_FLAG(uiCode, "ph_gdr_or_irap_pic_flag");
    if (uiCode)
    {
      READ_FLAG(uiCode, "ph_gdr_pic_flag");
    }
    READ_FLAG(uiCode, "ph_inter_slice_allowed_flag");
    if (uiCode)
    {
      READ_FLAG(uiCode, "ph_intra_slice_allowed_flag");
    }
    READ_FLAG(uiCode, "ph_non_reference_picture_flag");
    // parameter sets
    READ_UVLC(uiCode, "ph_pic_parameter_set_id");
    // picture order count
    READ_CODE(sps->bitsForPOC, pocLsb, "ph_pic_order_cnt_lsb");
  }
  int maxPocLsb = 1 << sps->bitsForPOC;
  int pocMsb;
  if (pcSlice->getIdrPicFlag())
  {
    if (picHeader->pocMsbPresent)
    {
      pocMsb = picHeader->pocMsbVal*maxPocLsb;
    }
    else
    {
      pocMsb = 0;
    }
  }
  else
  {
    int prevPoc = prevTid0POC;
    int prevPocLsb = prevPoc & (maxPocLsb - 1);
    int prevPocMsb = prevPoc - prevPocLsb;
    if (picHeader->pocMsbPresent)
    {
      pocMsb = picHeader->pocMsbVal*maxPocLsb;
    }
    else
    {
      if ((pocLsb < prevPocLsb) && ((prevPocLsb - pocLsb) >= (maxPocLsb / 2)))
      {
        pocMsb = prevPocMsb + maxPocLsb;
      }
      else if ((pocLsb > prevPocLsb) && ((pocLsb - prevPocLsb) > (maxPocLsb / 2)))
      {
        pocMsb = prevPocMsb - maxPocLsb;
      }
      else
      {
        pocMsb = prevPocMsb;
      }
    }
  }
  pcSlice->poc = (pocMsb + pocLsb);
}

void HLSyntaxReader::parseConstraintInfo(ConstraintInfo *cinfo)
{
  uint32_t symbol;
  READ_FLAG(cinfo->gciPresent, "gci_present_flag");
  if (cinfo->gciPresent)
  {
    READ_FLAG(cinfo->intraOnlyConstraintFlag, "gci_intra_only_constraint_flag");
    READ_FLAG(cinfo->allLayersIndependentConstraintFlag, "gci_all_layers_independent_constraint_flag");
    READ_FLAG(cinfo->onePictureOnlyConstraintFlag, "gci_one_au_only_constraint_flag");

    /* picture format */
    READ_CODE(4, symbol, "gci_sixteen_minus_max_bitdepth_constraint_idc"); cinfo->maxBitDepthConstraintIdc = (symbol>8 ? 16 : (16 - symbol));
    CHECK(symbol>8, "gci_sixteen_minus_max_bitdepth_constraint_idc shall be in the range 0 to 8, inclusive");
    READ_CODE(2, symbol, "gci_three_minus_max_chroma_format_constraint_idc"); cinfo->maxChromaFormatConstraintIdc = ((ChromaFormat)(3 - symbol));

    /* NAL unit type related */
    READ_FLAG(cinfo->noMixedNaluTypesInPicConstraintFlag, "gci_no_mixed_nalu_types_in_pic_constraint_flag");
    READ_FLAG(cinfo->noTrailConstraintFlag, "gci_no_trail_constraint_flag");
    READ_FLAG(cinfo->noStsaConstraintFlag, "gci_no_stsa_constraint_flag");
    READ_FLAG(cinfo->noRaslConstraintFlag, "gci_no_rasl_constraint_flag");
    READ_FLAG(cinfo->noRadlConstraintFlag, "gci_no_radl_constraint_flag");
    READ_FLAG(cinfo->noIdrConstraintFlag, "gci_no_idr_constraint_flag");
    READ_FLAG(cinfo->noCraConstraintFlag, "gci_no_cra_constraint_flag");
    READ_FLAG(cinfo->noGdrConstraintFlag, "gci_no_gdr_constraint_flag");
    READ_FLAG(cinfo->noApsConstraintFlag, "gci_no_aps_constraint_flag");
    READ_FLAG(cinfo->noIdrRplConstraintFlag, "gci_no_idr_rpl_constraint_flag");

    /* tile, slice, subpicture partitioning */
    READ_FLAG(cinfo->oneTilePerPicConstraintFlag, "gci_one_tile_per_pic_constraint_flag");
    READ_FLAG(cinfo->picHeaderInSliceHeaderConstraintFlag, "gci_pic_header_in_slice_header_constraint_flag");
    READ_FLAG(cinfo->oneSlicePerPicConstraintFlag, "gci_one_slice_per_pic_constraint_flag");
    READ_FLAG(cinfo->noRectSliceConstraintFlag, "gci_no_rectangular_slice_constraint_flag");
    READ_FLAG(cinfo->oneSlicePerSubpicConstraintFlag, "gci_one_slice_per_subpic_constraint_flag");
    READ_FLAG(cinfo->noSubpicInfoConstraintFlag, "gci_no_subpic_info_constraint_flag");


    /* CTU and block partitioning */
    READ_CODE(2, symbol, "gci_three_minus_max_log2_ctu_size_constraint_idc");   cinfo->maxLog2CtuSizeConstraintIdc = (((3 - symbol) + 5));
    READ_FLAG(cinfo->noPartitionConstraintsOverrideConstraintFlag, "gci_no_partition_constraints_override_constraint_flag");
    READ_FLAG(cinfo->noMttConstraintFlag, "gci_no_mtt_constraint_flag");
    READ_FLAG(cinfo->noQtbttDualTreeIntraConstraintFlag, "gci_no_qtbtt_dual_tree_intra_constraint_flag");

    /* intra */
    READ_FLAG(cinfo->noPaletteConstraintFlag, "gci_no_palette_constraint_flag");
    READ_FLAG(cinfo->noIbcConstraintFlag, "gci_no_ibc_constraint_flag");
    READ_FLAG(cinfo->noIspConstraintFlag, "gci_no_isp_constraint_flag");
    READ_FLAG(cinfo->noMrlConstraintFlag, "gci_no_mrl_constraint_flag");
    READ_FLAG(cinfo->noMipConstraintFlag, "gci_no_mip_constraint_flag");
    READ_FLAG(cinfo->noCclmConstraintFlag, "gci_no_cclm_constraint_flag");

    /* inter */
    READ_FLAG(cinfo->noRprConstraintFlag, "gci_no_ref_pic_resampling_constraint_flag");
    READ_FLAG(cinfo->noResChangeInClvsConstraintFlag, "gci_no_res_change_in_clvs_constraint_flag");
    READ_FLAG(cinfo->noWeightedPredictionConstraintFlag, "gci_no_weighted_prediction_constraint_flag");
    READ_FLAG(cinfo->noRefWraparoundConstraintFlag, "gci_no_ref_wraparound_constraint_flag");
    READ_FLAG(cinfo->noTemporalMvpConstraintFlag, "gci_no_temporal_mvp_constraint_flag");
    READ_FLAG(cinfo->noSbtmvpConstraintFlag, "gci_no_sbtmvp_constraint_flag");
    READ_FLAG(cinfo->noAmvrConstraintFlag, "gci_no_amvr_constraint_flag");
    READ_FLAG(cinfo->noBdofConstraintFlag, "gci_no_bdof_constraint_flag");
    READ_FLAG(cinfo->noSmvdConstraintFlag, "gci_no_smvd_constraint_flag");
    READ_FLAG(cinfo->noDmvrConstraintFlag, "gci_no_dmvr_constraint_flag");
    READ_FLAG(cinfo->noMmvdConstraintFlag, "gci_no_mmvd_constraint_flag");
    READ_FLAG(cinfo->noAffineMotionConstraintFlag, "gci_no_affine_motion_constraint_flag");
    READ_FLAG(cinfo->noProfConstraintFlag, "gci_no_prof_constraint_flag");
    READ_FLAG(cinfo->noBcwConstraintFlag, "gci_no_bcw_constraint_flag");
    READ_FLAG(cinfo->noCiipConstraintFlag, "gci_no_ciip_constraint_flag");
    READ_FLAG(cinfo->noGeoConstraintFlag, "gci_no_gpm_constraint_flag");

    /* transform, quantization, residual */
    READ_FLAG(cinfo->noLumaTransformSize64ConstraintFlag, "gci_no_luma_transform_size_64_constraint_flag");
    READ_FLAG(cinfo->noTransformSkipConstraintFlag, "gci_no_transform_skip_constraint_flag");
    READ_FLAG(cinfo->noBDPCMConstraintFlag, "gci_no_bdpcm_constraint_flag");
    READ_FLAG(cinfo->noMtsConstraintFlag, "gci_no_mts_constraint_flag");
    READ_FLAG(cinfo->noLfnstConstraintFlag, "gci_no_lfnst_constraint_flag");
    READ_FLAG(cinfo->noJointCbCrConstraintFlag, "gci_no_joint_cbcr_constraint_flag");
    READ_FLAG(cinfo->noSbtConstraintFlag, "gci_no_sbt_constraint_flag");
    READ_FLAG(cinfo->noActConstraintFlag, "gci_no_act_constraint_flag");
    READ_FLAG(cinfo->noExplicitScaleListConstraintFlag, "gci_no_explicit_scaling_list_constraint_flag");
    READ_FLAG(cinfo->noDepQuantConstraintFlag, "gci_no_dep_quant_constraint_flag");
    READ_FLAG(cinfo->noSignDataHidingConstraintFlag, "gci_no_sign_data_hiding_constraint_flag");
    READ_FLAG(cinfo->noQpDeltaConstraintFlag, "gci_no_qp_delta_constraint_flag");
    READ_FLAG(cinfo->noChromaQpOffsetConstraintFlag, "gci_no_chroma_qp_offset_constraint_flag");

    /* loop filter */
    READ_FLAG(cinfo->noSaoConstraintFlag, "gci_no_sao_constraint_flag");
    READ_FLAG(cinfo->noAlfConstraintFlag, "gci_no_alf_constraint_flag");
    READ_FLAG(cinfo->noCCAlfConstraintFlag, "gci_no_ccalf_constraint_flag");
    READ_FLAG(cinfo->noLmcsConstraintFlag, "gci_no_lmcs_constraint_flag");
    READ_FLAG(cinfo->noLadfConstraintFlag, "gci_no_ladf_constraint_flag");
    READ_FLAG(cinfo->noVirtualBoundaryConstraintFlag, "gci_no_virtual_boundaries_constraint_flag");

    READ_CODE(8, symbol, "gci_num_reserved_bits");
    uint32_t const numReservedBits = symbol;
    for (int i = 0; i < numReservedBits; i++)
    {
      READ_FLAG(symbol, "gci_reserved_zero_bit");                    CHECK(symbol != 0, "gci_reserved_zero_bit not equal to zero");
    }
  }
  while (!isByteAligned())
  {
    READ_FLAG(symbol, "gci_alignment_zero_bit");                     CHECK(symbol != 0, "gci_alignment_zero_bit not equal to zero");
  }
}


void HLSyntaxReader::parseProfileTierLevel(ProfileTierLevel *ptl, bool profileTierPresent, int maxNumSubLayersMinus1)
{
  bool flag;
  uint32_t symbol;
  if(profileTierPresent)
  {
    READ_CODE(7 , symbol,   "general_profile_idc"              );
    ptl->profileIdc  = Profile::Name(symbol);

    READ_FLAG(    flag,   "general_tier_flag"                );
    ptl->tierFlag =  flag ? Level::HIGH : Level::MAIN;
  }

  READ_CODE(8 , symbol,   "general_level_idc"                );
  ptl->levelIdc   = Level::Name(symbol);
  READ_FLAG( ptl->frameOnlyConstraintFlag,   "ptl_frame_only_constraint_flag"   );
  READ_FLAG( ptl->multiLayerEnabledFlag,     "ptl_multilayer_enabled_flag" );

  if(profileTierPresent)
  {
    parseConstraintInfo( &ptl->constraintInfo );
  }

  for (int i = maxNumSubLayersMinus1 - 1; i >= 0; i--)
  {
    READ_FLAG( ptl->subLayerLevelPresent[i], "sub_layer_level_present_flag[i]"   );
  }

  while (!isByteAligned())
  {
    READ_FLAG(    flag,   "ptl_reserved_zero_bit"        ); CHECK (flag, "ptl_reserved_zero_bit not equal to zero");
  }

  for (int i = maxNumSubLayersMinus1 - 1; i >= 0; i--)
  {
    if (ptl->subLayerLevelPresent[i])
    {
      READ_CODE(8 , symbol,   "sub_layer_level_idc"                );
      ptl->subLayerLevelIdc[i] = Level::Name(symbol);
    }
  }

  if (profileTierPresent)
  {
    READ_CODE(8, symbol, "ptl_num_sub_profiles");
    ptl->numSubProfile = symbol;
    ptl->subProfileIdc.resize( ptl->numSubProfile );
    for (int i = 0; i < ptl->numSubProfile; i++)
    {
      READ_CODE(32, symbol, "general_sub_profile_idc[i]");
      ptl->subProfileIdc[i] = symbol;
    }
  }
}

void HLSyntaxReader::parseTerminatingBit( uint32_t& ruiBit )
{
  ruiBit = false;
  int iBitsLeft = m_pcBitstream->getNumBitsLeft();
  if(iBitsLeft <= 8)
  {
    uint32_t uiPeekValue = m_pcBitstream->peekBits(iBitsLeft);
    if (uiPeekValue == (1<<(iBitsLeft-1)))
    {
      ruiBit = true;
    }
  }
}

void HLSyntaxReader::parseRemainingBytes( bool noTrailingBytesExpected )
{
  if (noTrailingBytesExpected)
  {
    CHECK( 0 != m_pcBitstream->getNumBitsLeft(), "Bits left although no bits expected" );
  }
  else
  {
    while (m_pcBitstream->getNumBitsLeft())
    {
      uint32_t trailingNullByte=m_pcBitstream->readByte();
      if (trailingNullByte!=0)
      {
        msg( ERROR, "Trailing byte should be 0, but has value %02x\n", trailingNullByte);
        THROW("Invalid trailing '0' byte");
      }
    }
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! parse explicit wp tables
void HLSyntaxReader::parsePredWeightTable( Slice* slice, const SPS *sps )
{
  WPScalingParam *wp;
  const ChromaFormat    chFmt        = sps->chromaFormatIdc;
  const int             numValidComp = int(getNumberValidComponents(chFmt));
  const bool            bChroma      = (chFmt!=CHROMA_400);
  const SliceType       eSliceType   = slice->sliceType;
  const int             iNbRef       = (eSliceType == B_SLICE ) ? (2) : (1);
  uint32_t            log2WeightDenomLuma=0, log2WeightDenomChroma=0;
  uint32_t            uiTotalSignalledWeightFlags = 0;

  int iDeltaDenom;
  // decode delta_luma_log2_weight_denom :
  READ_UVLC( log2WeightDenomLuma, "luma_log2_weight_denom" );
  CHECK( log2WeightDenomLuma > 7, "Invalid code" );
  if( bChroma )
  {
    READ_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );
    CHECK((iDeltaDenom + (int)log2WeightDenomLuma)<0, "Invalid code");
    CHECK((iDeltaDenom + (int)log2WeightDenomLuma)>7, "Invalid code");
    log2WeightDenomChroma = (uint32_t)(iDeltaDenom + log2WeightDenomLuma);
  }

  for ( int iNumRef=0 ; iNumRef<iNbRef ; iNumRef++ ) // loop over l0 and l1 syntax elements
  {
    RefPicList  refPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for ( int iRefIdx=0 ; iRefIdx<slice->numRefIdx[ refPicList ] ; iRefIdx++ )
    {
      slice->getWpScaling(refPicList, iRefIdx, wp);

      wp[COMP_Y].log2WeightDenom = log2WeightDenomLuma;
      for(int j=1; j<numValidComp; j++)
      {
        wp[j].log2WeightDenom = log2WeightDenomChroma;
      }

      uint32_t  uiCode;
      READ_FLAG( uiCode, iNumRef==0?"luma_weight_l0_flag[i]":"luma_weight_l1_flag[i]" );
      wp[COMP_Y].presentFlag = ( uiCode == 1 );
      uiTotalSignalledWeightFlags += wp[COMP_Y].presentFlag;
    }
    if ( bChroma )
    {
      uint32_t  uiCode;
      for ( int iRefIdx=0 ; iRefIdx<slice->numRefIdx[ refPicList ] ; iRefIdx++ )
      {
        slice->getWpScaling(refPicList, iRefIdx, wp);
        READ_FLAG( uiCode, iNumRef==0?"chroma_weight_l0_flag[i]":"chroma_weight_l1_flag[i]" );
        for(int j=1; j<numValidComp; j++)
        {
          wp[j].presentFlag = ( uiCode == 1 );
        }
        uiTotalSignalledWeightFlags += 2*wp[COMP_Cb].presentFlag;
      }
    }
    for ( int iRefIdx=0 ; iRefIdx<slice->numRefIdx[ refPicList ] ; iRefIdx++ )
    {
      slice->getWpScaling(refPicList, iRefIdx, wp);
      if ( wp[COMP_Y].presentFlag )
      {
        int iDeltaWeight;
        READ_SVLC( iDeltaWeight, iNumRef==0?"delta_luma_weight_l0[i]":"delta_luma_weight_l1[i]" );
        CHECK( iDeltaWeight < -128, "Invalid code" );
        CHECK( iDeltaWeight >  127, "Invalid code" );
        wp[COMP_Y].iWeight = (iDeltaWeight + (1<<wp[COMP_Y].log2WeightDenom));
        READ_SVLC( wp[COMP_Y].iOffset, iNumRef==0?"luma_offset_l0[i]":"luma_offset_l1[i]" );
        const int range=sps->spsRExt.highPrecisionOffsetsEnabled ? (1<<sps->bitDepths[ CH_L ])/2 : 128;
        CHECK ( wp[0].iOffset < -range , "luma_offset_lx shall be in the rage of -128 to 127");
        CHECK ( wp[0].iOffset >= range , "luma_offset_lx shall be in the rage of -128 to 127");
      }
      else
      {
        wp[COMP_Y].iWeight = (1 << wp[COMP_Y].log2WeightDenom);
        wp[COMP_Y].iOffset = 0;
      }
      if ( bChroma )
      {
        if ( wp[COMP_Cb].presentFlag )
        {
          int range=sps->spsRExt.highPrecisionOffsetsEnabled ? (1<<sps->bitDepths[ CH_C ])/2 : 128;
          for ( int j=1 ; j<numValidComp ; j++ )
          {
            int iDeltaWeight;
            READ_SVLC( iDeltaWeight, iNumRef==0?"delta_chroma_weight_l0[i]":"delta_chroma_weight_l1[i]" );
            CHECK( iDeltaWeight < -128, "Invalid code" );
            CHECK( iDeltaWeight >  127, "Invalid code" );
            wp[j].iWeight = (iDeltaWeight + (1<<wp[j].log2WeightDenom));

            int iDeltaChroma;
            READ_SVLC( iDeltaChroma, iNumRef==0?"delta_chroma_offset_l0[i]":"delta_chroma_offset_l1[i]" );
            CHECK( iDeltaChroma <  -4*range, "Invalid code" );
            CHECK( iDeltaChroma >  (4*range-1), "Invalid code" );
            int pred = ( range - ( ( range*wp[j].iWeight)>>(wp[j].log2WeightDenom) ) );
            wp[j].iOffset = Clip3(-range, range-1, (iDeltaChroma + pred) );
          }
        }
        else
        {
          for ( int j=1 ; j<numValidComp ; j++ )
          {
            wp[j].iWeight = (1 << wp[j].log2WeightDenom);
            wp[j].iOffset = 0;
          }
        }
      }
    }

    for ( int iRefIdx=slice->numRefIdx[ refPicList ] ; iRefIdx<MAX_NUM_REF ; iRefIdx++ )
    {
      slice->getWpScaling(refPicList, iRefIdx, wp);

      wp[0].presentFlag = false;
      wp[1].presentFlag = false;
      wp[2].presentFlag = false;
    }
  }
  CHECK(uiTotalSignalledWeightFlags>24, "Too many weight flag signalled");
}

void HLSyntaxReader::parsePredWeightTable( PicHeader* picHeader, const PPS *pps, const SPS *sps )
{
  WPScalingParam *wp;
  const ChromaFormat    chFmt        = sps->chromaFormatIdc;
  const int             numValidComp = int(getNumberValidComponents(chFmt));
  const bool         chroma                    = (chFmt != CHROMA_400);
  uint32_t           log2WeightDenomLuma       = 0;
  uint32_t           log2WeightDenomChroma     = 0;
  uint32_t           totalSignalledWeightFlags = 0;

  int deltaDenom;
  READ_UVLC(log2WeightDenomLuma, "luma_log2_weight_denom");
  CHECK(log2WeightDenomLuma > 7, "The value of luma_log2_weight_denom shall be in the range of 0 to 7");
  if (chroma)
  {
    READ_SVLC(deltaDenom, "delta_chroma_log2_weight_denom");
    CHECK((deltaDenom + (int) log2WeightDenomLuma) < 0, "luma_log2_weight_denom + delta_chroma_log2_weight_denom shall be in the range of 0 to 7");
    CHECK((deltaDenom + (int) log2WeightDenomLuma) > 7, "luma_log2_weight_denom + delta_chroma_log2_weight_denom shall be in the range of 0 to 7");
    log2WeightDenomChroma = (uint32_t)(deltaDenom + log2WeightDenomLuma);
  }

  uint32_t numLxWeights;
  READ_UVLC(numLxWeights, "num_l0_weights");
  picHeader->numL0Weights = (numLxWeights);
  picHeader->numL1Weights = (0);

  bool moreSyntaxToBeParsed = true;
  for ( int numRef=0 ; numRef<NUM_REF_PIC_LIST_01 && moreSyntaxToBeParsed ; numRef++ ) // loop over l0 and l1 syntax elements
  {
    RefPicList  refPicList = ( numRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for ( int iRefIdx=0 ; iRefIdx<numLxWeights ; iRefIdx++ )
    {
      picHeader->getWpScaling(refPicList, iRefIdx, wp);

      wp[COMP_Y].log2WeightDenom = log2WeightDenomLuma;
      for(int j=1; j<numValidComp; j++)
      {
        wp[j].log2WeightDenom = log2WeightDenomChroma;
      }

      uint32_t  uiCode;
      READ_FLAG( uiCode, numRef==0?"luma_weight_l0_flag[i]":"luma_weight_l1_flag[i]" );
      wp[COMP_Y].presentFlag = ( uiCode == 1 );
      totalSignalledWeightFlags += wp[COMP_Y].presentFlag;
    }
    if ( chroma )
    {
      uint32_t  uiCode;
      for ( int iRefIdx=0 ; iRefIdx<numLxWeights ; iRefIdx++ )
      {
        picHeader->getWpScaling(refPicList, iRefIdx, wp);
        READ_FLAG( uiCode, numRef==0?"chroma_weight_l0_flag[i]":"chroma_weight_l1_flag[i]" );
        for(int j=1; j<numValidComp; j++)
        {
          wp[j].presentFlag = ( uiCode == 1 );
        }
        totalSignalledWeightFlags += 2*wp[COMP_Cb].presentFlag;
      }
    }
    else
    {
      for ( int refIdx=0; refIdx<MAX_NUM_REF; refIdx++ )
      {
        picHeader->getWpScaling(refPicList, refIdx, wp);
        wp[1].presentFlag = false;
        wp[2].presentFlag = false;
      }
    }
    for ( int refIdx=0 ; refIdx<numLxWeights; refIdx++ )
    {
      picHeader->getWpScaling(refPicList, refIdx, wp);
      if ( wp[COMP_Y].presentFlag )
      {
        int deltaWeight;
        READ_SVLC(deltaWeight, numRef == 0 ? "delta_luma_weight_l0[i]" : "delta_luma_weight_l1[i]");
        CHECK(deltaWeight < -128, "delta_luma_weight_lx shall be in the rage of -128 to 127");
        CHECK(deltaWeight > 127, "delta_luma_weight_lx shall be in the rage of -128 to 127");
        wp[COMP_Y].iWeight = (deltaWeight + (1 << wp[COMP_Y].log2WeightDenom));
        READ_SVLC(wp[COMP_Y].iOffset, numRef == 0 ? "luma_offset_l0[i]" : "luma_offset_l1[i]");
        const int range = sps->spsRExt.highPrecisionOffsetsEnabled ? (1 << sps->bitDepths[CH_L]) / 2 : 128;
        CHECK ( wp[0].iOffset < -range , "luma_offset_lx shall be in the rage of -128 to 127");
        CHECK ( wp[0].iOffset >= range , "luma_offset_lx shall be in the rage of -128 to 127");
      }
      else
      {
        wp[COMP_Y].iWeight = (1 << wp[COMP_Y].log2WeightDenom);
        wp[COMP_Y].iOffset = 0;
      }
      if ( chroma )
      {
        if ( wp[COMP_Cb].presentFlag )
        {
          int range=sps->spsRExt.highPrecisionOffsetsEnabled ? (1<<sps->bitDepths[ CH_C ])/2 : 128;
          for ( int j=1 ; j<numValidComp ; j++ )
          {
            int deltaWeight;
            READ_SVLC(deltaWeight, numRef == 0 ? "delta_chroma_weight_l0[i]" : "delta_chroma_weight_l1[i]");
            CHECK( deltaWeight < -128, "delta_chroma_weight_lx shall be in the rage of -128 to 127" );
            CHECK( deltaWeight >  127, "delta_chroma_weight_lx shall be in the rage of -128 to 127" );
            wp[j].iWeight = (deltaWeight + (1 << wp[j].log2WeightDenom));

            int deltaChroma;
            READ_SVLC(deltaChroma, numRef == 0 ? "delta_chroma_offset_l0[i]" : "delta_chroma_offset_l1[i]");
            CHECK( deltaChroma <  -4*range, "delta_chroma_offset_lx shall be in the range of -4 * 128 to 4 * 127" );
            CHECK( deltaChroma >=  4*range, "delta_chroma_offset_lx shall be in the range of -4 * 128 to 4 * 127" );
            int pred      = (range - ((range * wp[j].iWeight) >> (wp[j].log2WeightDenom)));
            wp[j].iOffset = Clip3(-range, range - 1, (deltaChroma + pred));
          }
        }
        else
        {
          for ( int j=1 ; j<numValidComp ; j++ )
          {
            wp[j].iWeight = (1 << wp[j].log2WeightDenom);
            wp[j].iOffset = 0;
          }
        }
      }
    }

    for ( int iRefIdx=numLxWeights ; iRefIdx<MAX_NUM_REF ; iRefIdx++ )
    {
      picHeader->getWpScaling(refPicList, iRefIdx, wp);

      wp[0].presentFlag = false;
      wp[1].presentFlag = false;
      wp[2].presentFlag = false;
    }

    if (numRef == 0)
    {
      if (pps->weightedBiPred && picHeader->pRPL[1]->getNumRefEntries() > 0)
      {
        READ_UVLC(numLxWeights, "num_l1_weights");
      }
      moreSyntaxToBeParsed = numLxWeights != 0;
      picHeader->numL1Weights = (numLxWeights);
    }
  }
  CHECK(totalSignalledWeightFlags>24, "Too many weight flag signalled");
}

bool HLSyntaxReader::xMoreRbspData()
{
  int bitsLeft = m_pcBitstream->getNumBitsLeft();

  // if there are more than 8 bits, it cannot be rbsp_trailing_bits
  if (bitsLeft > 8)
  {
    return true;
  }

  uint8_t lastByte = m_pcBitstream->peekBits(bitsLeft);
  int cnt = bitsLeft;

  // remove trailing bits equal to zero
  while ((cnt>0) && ((lastByte & 1) == 0))
  {
    lastByte >>= 1;
    cnt--;
  }
  // remove bit equal to one
  cnt--;

  // we should not have a negative number of bits
  CHECK (cnt<0, "Negative number of bits");

  // we have more data, if cnt is not zero
  return (cnt>0);
}

void HLSyntaxReader::alfFilter( AlfParam& alfParam, const bool isChroma, const int altIdx )
{
  uint32_t code;

  // derive maxGolombIdx
  AlfFilterShape alfShape( isChroma ? 5 : 7 );
  const int numFilters = isChroma ? 1 : alfParam.numLumaFilters;
  short* coeff = isChroma ? alfParam.chromaCoeff[altIdx] : alfParam.lumaCoeff;
  short* clipp = isChroma ? alfParam.chromaClipp[altIdx] : alfParam.lumaClipp;

  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      READ_UVLC( code, isChroma ? "alf_chroma_coeff_abs" : "alf_luma_coeff_abs" );
      coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ] = code;
      if( coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ] != 0 )
      {
        READ_FLAG( code, isChroma ? "alf_chroma_coeff_sign" : "alf_luma_coeff_sign" );
        coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ] = ( code ) ? -coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ] : coeff[ ind * MAX_NUM_ALF_LUMA_COEFF + i ];
      }
      CHECK( isChroma &&
             ( coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] > 127 || coeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] < -127 )
             , "AlfCoeffC shall be in the range of -127 to 127, inclusive" );
    }
  }

  // Clipping values coding
  if ( alfParam.nonLinearFlag[isChroma] )
  {

    // Filter coefficients
    for( int ind = 0; ind < numFilters; ++ind )
    {

      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        READ_CODE(2, code, isChroma ? "alf_chroma_clip_idx" : "alf_luma_clip_idx");
        clipp[ind * MAX_NUM_ALF_LUMA_COEFF + i] = code;
      }
    }
  }
  else
  {
    for( int ind = 0; ind < numFilters; ++ind )
    {
      std::fill_n( clipp + ind * MAX_NUM_ALF_LUMA_COEFF, alfShape.numCoeff, 0 );
    }
  }
}

} // namespace vvenc

//! \}


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


/** \file     VVEncCfg.cpp
    \brief    encoder configuration class
*/

#include "vvenc/vvencCfg.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Slice.h"

#include <math.h>

//! \ingroup Interface
//! \{

namespace vvenc {

bool VVEncCfg::confirmParameter( bool bflag, const char* message )
{
  if ( ! bflag )
    return false;
  msg( ERROR, "Parameter Check Error: %s\n", message );
  m_confirmFailed = true;
  return true;
}

bool VVEncCfg::checkExperimental( bool bflag, const char* message )
{
  if( !bflag )
    return false;

  msg( WARNING, "Warning: Setting experimental: %s\n\n", message );
  return true;
}

bool VVEncCfg::initCfgParameter()
{
#define CONFIRM_PARAMETER_OR_RETURN( _f, _m ) { if ( confirmParameter( _f, _m ) ) return true; }


  m_confirmFailed = false;


  //
  // set a lot of dependent parameters
  //
  if( m_profile == Profile::PROFILE_AUTO )
  {
    const int maxBitDepth= std::max(m_internalBitDepth[CH_L], m_internalBitDepth[m_internChromaFormat==ChromaFormat::CHROMA_400 ? CH_L : CH_C]);
    m_profile=Profile::PROFILE_NONE;

    if (m_internChromaFormat==ChromaFormat::CHROMA_400 || m_internChromaFormat==ChromaFormat::CHROMA_420)
    {
      if (maxBitDepth<=10)
      {
        m_profile=Profile::MAIN_10;
      }
    }
    else if (m_internChromaFormat==ChromaFormat::CHROMA_422 || m_internChromaFormat==ChromaFormat::CHROMA_444)
    {
      if (maxBitDepth<=10)
      {
        m_profile=Profile::MAIN_10_444;
      }
    }

    CONFIRM_PARAMETER_OR_RETURN(  m_profile == Profile::PROFILE_NONE, "can not determin auto profile");
  }

  if ( m_InputQueueSize <= 0 )
  {
    m_InputQueueSize = m_maxParallelFrames ? 2*m_GOPSize + 1: m_GOPSize;
  }
  if ( m_MCTF )
  {
    m_InputQueueSize += MCTF_ADD_QUEUE_DELAY;
  }

  m_framesToBeEncoded = ( m_framesToBeEncoded + m_temporalSubsampleRatio - 1 ) / m_temporalSubsampleRatio;

  m_MCTFNumLeadFrames  = std::min( m_MCTFNumLeadFrames,  MCTF_RANGE );
  m_MCTFNumTrailFrames = std::min( m_MCTFNumTrailFrames, MCTF_RANGE );
  /* rules for input, output and internal bitdepths as per help text */
  if (m_MSBExtendedBitDepth[CH_L  ] == 0)
  {
    m_MSBExtendedBitDepth[CH_L  ] = m_inputBitDepth      [CH_L  ];
  }
  if (m_MSBExtendedBitDepth[CH_C] == 0)
  {
    m_MSBExtendedBitDepth[CH_C] = m_MSBExtendedBitDepth[CH_L  ];
  }
  if (m_internalBitDepth   [CH_L  ] == 0)
  {
    m_internalBitDepth   [CH_L  ] = m_MSBExtendedBitDepth[CH_L  ];
  }
  if (m_internalBitDepth   [CH_C] == 0)
  {
    m_internalBitDepth   [CH_C] = m_internalBitDepth   [CH_L  ];
  }
  if (m_inputBitDepth      [CH_C] == 0)
  {
    m_inputBitDepth      [CH_C] = m_inputBitDepth      [CH_L  ];
  }
  if (m_outputBitDepth     [CH_L  ] == 0)
  {
    m_outputBitDepth     [CH_L  ] = m_internalBitDepth   [CH_L  ];
  }
  if (m_outputBitDepth     [CH_C] == 0)
  {
    m_outputBitDepth     [CH_C] = m_outputBitDepth     [CH_L  ];
  }

  CONFIRM_PARAMETER_OR_RETURN( m_fastInterSearchMode<0 || m_fastInterSearchMode>FASTINTERSEARCH_MODE3, "Error: FastInterSearchMode parameter out of range" );

  CONFIRM_PARAMETER_OR_RETURN( m_motionEstimationSearchMethod < 0 || m_motionEstimationSearchMethod >= MESEARCH_NUMBER_OF_METHODS, "Error: FastSearch parameter out of range" );

  switch (m_conformanceWindowMode)
  {
  case 0:
    {
      // no conformance or padding
      m_confWinLeft = m_confWinRight = m_confWinTop = m_confWinBottom = 0;
      m_aiPad[1] = m_aiPad[0] = 0;
      break;
    }
  case 1:
    {
      // automatic padding to minimum CU size
      const int minCuSize = 1 << ( MIN_CU_LOG2 + 1 );
      if (m_SourceWidth % minCuSize)
      {
        m_aiPad[0] = m_confWinRight  = ((m_SourceWidth / minCuSize) + 1) * minCuSize - m_SourceWidth;
      }
      if (m_SourceHeight % minCuSize)
      {
        m_aiPad[1] = m_confWinBottom = ((m_SourceHeight / minCuSize) + 1) * minCuSize - m_SourceHeight;
      }
      CONFIRM_PARAMETER_OR_RETURN( m_aiPad[0] % SPS::getWinUnitX(m_internChromaFormat) != 0, "Error: picture width is not an integer multiple of the specified chroma subsampling" );
      CONFIRM_PARAMETER_OR_RETURN( m_aiPad[1] % SPS::getWinUnitY(m_internChromaFormat) != 0, "Error: picture height is not an integer multiple of the specified chroma subsampling" );
      break;
    }
  case 2:
    {
      //padding
      m_confWinRight  = m_aiPad[0];
      m_confWinBottom = m_aiPad[1];
      break;
    }
  case 3:
    {
      // conformance
      if ((m_confWinLeft == 0) && (m_confWinRight == 0) && (m_confWinTop == 0) && (m_confWinBottom == 0))
      {
        msg( ERROR, "Warning: Conformance window enabled, but all conformance window parameters set to zero\n");
      }
      if ((m_aiPad[1] != 0) || (m_aiPad[0]!=0))
      {
        msg( ERROR, "Warning: Conformance window enabled, padding parameters will be ignored\n");
      }
      m_aiPad[1] = m_aiPad[0] = 0;
      break;
    }
  }
    m_PadSourceWidth  = m_SourceWidth + m_aiPad[0];
    m_PadSourceHeight = m_SourceHeight + m_aiPad[1];

  m_sliceId.resize(1,0);

  for(uint32_t ch=0; ch < MAX_NUM_CH; ch++ )
  {
    if (m_saoOffsetBitShift[ch]<0)
    {
      if (m_internalBitDepth[ch]>10)
      {
        m_log2SaoOffsetScale[ch]=uint32_t(Clip3<int>(0, m_internalBitDepth[ch]-10, int(m_internalBitDepth[ch]-10 + 0.165*m_QP - 3.22 + 0.5) ) );
      }
      else
      {
        m_log2SaoOffsetScale[ch]=0;
      }
    }
    else
    {
      m_log2SaoOffsetScale[ch]=uint32_t(m_saoOffsetBitShift[ch]);
    }
  }

  CONFIRM_PARAMETER_OR_RETURN(m_qpInValsCb.size() != m_qpOutValsCb.size(), "Chroma QP table for Cb is incomplete.");
  CONFIRM_PARAMETER_OR_RETURN(m_qpInValsCr.size() != m_qpOutValsCr.size(), "Chroma QP table for Cr is incomplete.");
  CONFIRM_PARAMETER_OR_RETURN(m_qpInValsCbCr.size() != m_qpOutValsCbCr.size(), "Chroma QP table for CbCr is incomplete.");
  if (m_useIdentityTableForNon420Chroma && m_internChromaFormat != CHROMA_420)
  {
    m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag = true;
    m_qpInValsCb = { 0 };
    m_qpInValsCr = { 0 };
    m_qpInValsCbCr = { 0 };
    m_qpOutValsCb = { 0 };
    m_qpOutValsCr = { 0 };
    m_qpOutValsCbCr = { 0 };
  }
  int qpBdOffsetC = 6 * (m_internalBitDepth[CH_C] - 8);

  m_chromaQpMappingTableParams.m_numQpTables = m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag? 1 : (m_JointCbCrMode ? 3 : 2);
  m_chromaQpMappingTableParams.m_deltaQpInValMinus1[0].resize(m_qpInValsCb.size());
  m_chromaQpMappingTableParams.m_deltaQpOutVal[0].resize(m_qpOutValsCb.size());
  m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[0] = (m_qpOutValsCb.size() > 1) ? (int)m_qpOutValsCb.size() - 2 : 0;
  m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] = (m_qpOutValsCb.size() > 1) ? -26 + m_qpInValsCb[0] : 0;
  CONFIRM_PARAMETER_OR_RETURN(m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] < -26 - qpBdOffsetC || m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] > 36, "qpTableStartMinus26[0] is out of valid range of -26 -qpBdOffsetC to 36, inclusive.")
  CONFIRM_PARAMETER_OR_RETURN(m_qpInValsCb[0] != m_qpOutValsCb[0], "First qpInValCb value should be equal to first qpOutValCb value");
  for (int i = 0; i < m_qpInValsCb.size() - 1; i++)
  {
    CONFIRM_PARAMETER_OR_RETURN(m_qpInValsCb[i] < -qpBdOffsetC || m_qpInValsCb[i] > MAX_QP, "Some entries cfg_qpInValCb are out of valid range of -qpBdOffsetC to 63, inclusive.");
    CONFIRM_PARAMETER_OR_RETURN(m_qpOutValsCb[i] < -qpBdOffsetC || m_qpOutValsCb[i] > MAX_QP, "Some entries cfg_qpOutValCb are out of valid range of -qpBdOffsetC to 63, inclusive.");
    m_chromaQpMappingTableParams.m_deltaQpInValMinus1[0][i] = m_qpInValsCb[i + 1] - m_qpInValsCb[i] - 1;
    m_chromaQpMappingTableParams.m_deltaQpOutVal[0][i] = m_qpOutValsCb[i + 1] - m_qpOutValsCb[i];
  }
  if (!m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag)
  {
    m_chromaQpMappingTableParams.m_deltaQpInValMinus1[1].resize(m_qpInValsCr.size());
    m_chromaQpMappingTableParams.m_deltaQpOutVal[1].resize(m_qpOutValsCr.size());
    m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[1] = (m_qpOutValsCr.size() > 1) ? (int)m_qpOutValsCr.size() - 2 : 0;
    m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] = (m_qpOutValsCr.size() > 1) ? -26 + m_qpInValsCr[0] : 0;
    CONFIRM_PARAMETER_OR_RETURN(m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] < -26 - qpBdOffsetC || m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] > 36, "qpTableStartMinus26[1] is out of valid range of -26 -qpBdOffsetC to 36, inclusive.")
    CONFIRM_PARAMETER_OR_RETURN(m_qpInValsCr[0] != m_qpOutValsCr[0], "First qpInValCr value should be equal to first qpOutValCr value");
    for (int i = 0; i < m_qpInValsCr.size() - 1; i++)
    {
      CONFIRM_PARAMETER_OR_RETURN(m_qpInValsCr[i] < -qpBdOffsetC || m_qpInValsCr[i] > MAX_QP, "Some entries cfg_qpInValCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      CONFIRM_PARAMETER_OR_RETURN(m_qpOutValsCr[i] < -qpBdOffsetC || m_qpOutValsCr[i] > MAX_QP, "Some entries cfg_qpOutValCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      m_chromaQpMappingTableParams.m_deltaQpInValMinus1[1][i] = m_qpInValsCr[i + 1] - m_qpInValsCr[i] - 1;
      m_chromaQpMappingTableParams.m_deltaQpOutVal[1][i] = m_qpOutValsCr[i + 1] - m_qpOutValsCr[i];
    }
    m_chromaQpMappingTableParams.m_deltaQpInValMinus1[2].resize(m_qpInValsCbCr.size());
    m_chromaQpMappingTableParams.m_deltaQpOutVal[2].resize(m_qpOutValsCbCr.size());
    m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[2] = (m_qpOutValsCbCr.size() > 1) ? (int)m_qpOutValsCbCr.size() - 2 : 0;
    m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] = (m_qpOutValsCbCr.size() > 1) ? -26 + m_qpInValsCbCr[0] : 0;
    CONFIRM_PARAMETER_OR_RETURN(m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] < -26 - qpBdOffsetC || m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] > 36, "qpTableStartMinus26[2] is out of valid range of -26 -qpBdOffsetC to 36, inclusive.")
    CONFIRM_PARAMETER_OR_RETURN(m_qpInValsCbCr[0] != m_qpInValsCbCr[0], "First qpInValCbCr value should be equal to first qpOutValCbCr value");
    for (int i = 0; i < m_qpInValsCbCr.size() - 1; i++)
    {
      CONFIRM_PARAMETER_OR_RETURN(m_qpInValsCbCr[i] < -qpBdOffsetC || m_qpInValsCbCr[i] > MAX_QP, "Some entries cfg_qpInValCbCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      CONFIRM_PARAMETER_OR_RETURN(m_qpOutValsCbCr[i] < -qpBdOffsetC || m_qpOutValsCbCr[i] > MAX_QP, "Some entries cfg_qpOutValCbCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      m_chromaQpMappingTableParams.m_deltaQpInValMinus1[2][i] = m_qpInValsCbCr[i + 1] - m_qpInValsCbCr[i] - 1;
      m_chromaQpMappingTableParams.m_deltaQpOutVal[2][i] = m_qpInValsCbCr[i + 1] - m_qpInValsCbCr[i];
    }
  }

  const int minCuSize = 1 << MIN_CU_LOG2;
  m_MaxCodingDepth = 0;
  while( ( m_CTUSize >> m_MaxCodingDepth ) > minCuSize )
  {
    m_MaxCodingDepth++;
  }
  m_log2DiffMaxMinCodingBlockSize = m_MaxCodingDepth;

  m_reshapeCW.binCW.resize(3);
  m_reshapeCW.rspFps     = m_FrameRate;
  m_reshapeCW.rspPicSize = m_PadSourceWidth*m_PadSourceHeight;
  m_reshapeCW.rspFpsToIp = std::max(16, 16 * (int)(round((double)m_FrameRate /16.0)));
  m_reshapeCW.rspBaseQP  = m_QP;
  m_reshapeCW.updateCtrl = m_updateCtrl;
  m_reshapeCW.adpOption  = m_adpOption;
  m_reshapeCW.initialCW  = m_initialCW;


  if( m_IntraPeriod == 0 &&  m_IntraPeriodSec > 0 )
  {
    if ( m_FrameRate % m_GOPSize == 0 )
    {
      m_IntraPeriod = m_FrameRate * m_IntraPeriodSec;
    }
    else
    {
      int iIDRPeriod  = (m_FrameRate * m_IntraPeriodSec);
      if( iIDRPeriod < m_GOPSize )
      {
        iIDRPeriod = m_GOPSize;
      }

      int iDiff = iIDRPeriod % m_GOPSize;
      if( iDiff < m_GOPSize >> 1 )
      {
        m_IntraPeriod = iIDRPeriod - iDiff;
      }
      else
      {
        m_IntraPeriod = iIDRPeriod + m_GOPSize - iDiff;
      }
    }
  }


  if( m_ensureWppBitEqual < 0 )
  {
    m_ensureWppBitEqual = m_numThreads > 0 ? 1 : 0;
  }


  //
  // do some check and set of parameters next
  //


  msg( NOTICE, "\n" );
  if ( m_decodedPictureHashSEIType == HASHTYPE_NONE )
  {
    msg( DETAILS, "******************************************************************\n");
    msg( DETAILS, "** WARNING: --SEIDecodedPictureHash is now disabled by default. **\n");
    msg( DETAILS, "**          Automatic verification of decoded pictures by a     **\n");
    msg( DETAILS, "**          decoder requires this option to be enabled.         **\n");
    msg( DETAILS, "******************************************************************\n");
  }
  if( m_profile == Profile::PROFILE_NONE )
  {
    msg( DETAILS, "***************************************************************************\n");
    msg( DETAILS, "** WARNING: For conforming bitstreams a valid Profile value must be set! **\n");
    msg( DETAILS, "***************************************************************************\n");
  }
  if( m_level == Level::LEVEL_NONE )
  {
    msg( DETAILS, "***************************************************************************\n");
    msg( DETAILS, "** WARNING: For conforming bitstreams a valid Level value must be set!   **\n");
    msg( DETAILS, "***************************************************************************\n");
  }

  if( m_DepQuantEnabled )
  {
    confirmParameter( !m_RDOQ || !m_useRDOQTS, "RDOQ and RDOQTS must be greater 0 if dependent quantization is enabled" );
    confirmParameter( m_SignDataHidingEnabled, "SignHideFlag must be equal to 0 if dependent quantization is enabled" );
  }

  confirmParameter( (m_MSBExtendedBitDepth[CH_L  ] < m_inputBitDepth[CH_L  ]), "MSB-extended bit depth for luma channel (--MSBExtendedBitDepth) must be greater than or equal to input bit depth for luma channel (--InputBitDepth)" );
  confirmParameter( (m_MSBExtendedBitDepth[CH_C] < m_inputBitDepth[CH_C]), "MSB-extended bit depth for chroma channel (--MSBExtendedBitDepthC) must be greater than or equal to input bit depth for chroma channel (--InputBitDepthC)" );

  const uint32_t maxBitDepth=(m_internChromaFormat==CHROMA_400) ? m_internalBitDepth[CH_L] : std::max(m_internalBitDepth[CH_L], m_internalBitDepth[CH_C]);
  confirmParameter(m_bitDepthConstraintValue<maxBitDepth, "The internalBitDepth must not be greater than the bitDepthConstraint value");

  confirmParameter(m_bitDepthConstraintValue!=10, "BitDepthConstraint must be 8 for MAIN profile and 10 for MAIN10 profile.");
  confirmParameter(m_intraOnlyConstraintFlag==true, "IntraOnlyConstraintFlag must be false for non main_RExt profiles.");

  // check range of parameters
  confirmParameter( m_inputBitDepth[CH_L  ] < 8,                                 "InputBitDepth must be at least 8" );
  confirmParameter( m_inputBitDepth[CH_C] < 8,                                   "InputBitDepthC must be at least 8" );

#if !RExt__HIGH_BIT_DEPTH_SUPPORT
  for (uint32_t channelType = 0; channelType < MAX_NUM_CH; channelType++)
  {
    confirmParameter((m_internalBitDepth[channelType] > 12) , "Model is not configured to support high enough internal accuracies - enable RExt__HIGH_BIT_DEPTH_SUPPORT to use increased precision internal data types etc...");
  }
#endif

  confirmParameter( m_log2SaoOffsetScale[CH_L]   > (m_internalBitDepth[CH_L  ]<10?0:(m_internalBitDepth[CH_L  ]-10)), "SaoLumaOffsetBitShift must be in the range of 0 to InternalBitDepth-10, inclusive");
  confirmParameter( m_log2SaoOffsetScale[CH_C] > (m_internalBitDepth[CH_C]<10?0:(m_internalBitDepth[CH_C]-10)), "SaoChromaOffsetBitShift must be in the range of 0 to InternalBitDepthC-10, inclusive");

  confirmParameter( m_internChromaFormat >= NUM_CHROMA_FORMAT,                                  "Intern chroma format must be either 400, 420, 422 or 444" );
  confirmParameter( m_FrameRate <= 0,                                                           "Frame rate must be more than 1" );
  confirmParameter( m_temporalSubsampleRatio < 1,                                               "Temporal subsample rate must be no less than 1" );
  confirmParameter( m_framesToBeEncoded < m_switchPOC,                                          "debug POC out of range" );

  confirmParameter( m_GOPSize < 1 ,                                                             "GOP Size must be greater or equal to 1" );
  confirmParameter( m_GOPSize > 1 &&  m_GOPSize % 2,                                            "GOP Size must be a multiple of 2, if GOP Size is greater than 1" );
  confirmParameter( (m_IntraPeriod > 0 && m_IntraPeriod < m_GOPSize) || m_IntraPeriod == 0,     "Intra period must be more than GOP size, or -1 , not 0" );
  confirmParameter( m_InputQueueSize < m_GOPSize ,                                              "Input queue size must be greater or equal to gop size" );
  confirmParameter( m_MCTF && m_InputQueueSize < m_GOPSize + MCTF_ADD_QUEUE_DELAY ,             "Input queue size must be greater or equal to gop size + N frames for MCTF" );
  confirmParameter( m_DecodingRefreshType < 0 || m_DecodingRefreshType > 3,                     "Decoding Refresh Type must be comprised between 0 and 3 included" );
#if IDR_FIX
  confirmParameter( m_IntraPeriod > 0 && !(m_DecodingRefreshType==1 || m_DecodingRefreshType==2), "Only Decoding Refresh Type CRA for non low delay supported" );                  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#else
  confirmParameter( m_IntraPeriod > 0 && m_DecodingRefreshType !=1,                             "Only Decoding Refresh Type CRA for non low delay supported" );                  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#endif
  confirmParameter( m_IntraPeriod < 0 && m_DecodingRefreshType !=0,                             "Only Decoding Refresh Type 0 for low delay supported" );
  confirmParameter( m_QP < -6 * (m_internalBitDepth[CH_L] - 8) || m_QP > MAX_QP,                "QP exceeds supported range (-QpBDOffsety to 63)" );
  for( int comp = 0; comp < 3; comp++)
  {
    confirmParameter( m_loopFilterBetaOffsetDiv2[comp] < -12 || m_loopFilterBetaOffsetDiv2[comp] > 12,          "Loop Filter Beta Offset div. 2 exceeds supported range (-12 to 12)" );
    confirmParameter( m_loopFilterTcOffsetDiv2[comp] < -12 || m_loopFilterTcOffsetDiv2[comp] > 12,              "Loop Filter Tc Offset div. 2 exceeds supported range (-12 to 12)" );
  }
  confirmParameter( m_SearchRange < 0 ,                                                         "Search Range must be more than 0" );
  confirmParameter( m_bipredSearchRange < 0 ,                                                   "Bi-prediction refinement search range must be more than 0" );
  confirmParameter( m_minSearchWindow < 0,                                                      "Minimum motion search window size for the adaptive window ME must be greater than or equal to 0" );

  confirmParameter( m_MCTFFrames.size() != m_MCTFStrengths.size(),        "MCTF parameter list sizes differ");
  confirmParameter( m_MCTFNumLeadFrames  < 0,                             "MCTF number of lead frames must be greater than or equal to 0" );
  confirmParameter( m_MCTFNumTrailFrames < 0,                             "MCTF number of trailing frames must be greater than or equal to 0" );
  confirmParameter( m_MCTFNumLeadFrames  > 0 && ! m_MCTF,                 "MCTF disabled but number of MCTF lead frames is given" );
  confirmParameter( m_MCTFNumTrailFrames > 0 && ! m_MCTF,                 "MCTF disabled but number of MCTF trailing frames is given" );
  confirmParameter( m_MCTFNumTrailFrames > 0 && m_framesToBeEncoded <= 0, "If number of MCTF trailing frames is given, the total number of frames to be encoded has to be set" );

  if (m_lumaReshapeEnable)
  {
    confirmParameter( m_reshapeSignalType < RESHAPE_SIGNAL_SDR || m_reshapeSignalType > RESHAPE_SIGNAL_HLG, "LMCSSignalType out of range" );
    confirmParameter(m_updateCtrl < 0,    "Min. LMCS Update Control is 0");
    confirmParameter(m_updateCtrl > 2,    "Max. LMCS Update Control is 2");
    confirmParameter(m_adpOption < 0,     "Min. LMCS Adaptation Option is 0");
    confirmParameter(m_adpOption > 4,     "Max. LMCS Adaptation Option is 4");
    confirmParameter(m_initialCW < 0,     "Min. Initial Total Codeword is 0");
    confirmParameter(m_initialCW > 1023,  "Max. Initial Total Codeword is 1023");
    confirmParameter(m_LMCSOffset < -7,   "Min. LMCS Offset value is -7");
    confirmParameter(m_LMCSOffset > 7,    "Max. LMCS Offset value is 7");
    if (m_updateCtrl > 0 && m_adpOption > 2) { m_adpOption -= 2; }
  }
  confirmParameter( m_EDO && m_bLoopFilterDisable,          "no EDO support with LoopFilter disabled" );
  confirmParameter( m_EDO < 0 || m_EDO > 2,                 "EDO out of range [0..2]" );
  confirmParameter( m_TMVPModeId < 0 || m_TMVPModeId > 2,   "TMVPMode out of range [0..2]" );
  confirmParameter( m_AMVRspeed < 0 || m_AMVRspeed > 7,     "AMVR/IMV out of range [0..7]" );
  confirmParameter( m_Affine < 0 || m_Affine > 2,           "Affine out of range [0..2]" );
  confirmParameter( m_MMVD < 0 || m_MMVD > 4,               "MMVD out of range [0..4]" );
  confirmParameter( m_SMVD < 0 || m_SMVD > 3,               "SMVD out of range [0..3]" );
  confirmParameter( m_Geo  < 0 || m_Geo  > 3,               "Geo out of range [0..3]" );
  confirmParameter( m_CIIP < 0 || m_CIIP > 3,               "CIIP out of range [0..3]" );
  confirmParameter( m_SBT  < 0 || m_SBT  > 3,               "SBT out of range [0..3]" );
  confirmParameter( m_LFNST< 0 || m_LFNST> 3,               "LFNST out of range [0..3]" );
  confirmParameter( m_MCTF < 0 || m_MCTF > 2,               "MCTF out of range [0..2]" );
  confirmParameter( m_ISP < 0 || m_ISP > 3,                 "ISP out of range [0..3]" );
  confirmParameter(m_TS < 0 || m_TS > 2,                    "TS out of range [0..2]" );
  confirmParameter(m_TSsize < 2 || m_TSsize > 5,            "TSsize out of range [2..5]" );
  confirmParameter(m_useBDPCM < 0 || m_useBDPCM > 2,        "BDPCM out of range [0..2]");
  confirmParameter(m_useBDPCM  && m_TS==0,                  "BDPCM cannot be used when transform skip is disabled" );
  confirmParameter(m_useBDPCM==1  && m_TS==2,               "BDPCM cannot be permanently used when transform skip is auto" );

  if( m_alf )
  {
    confirmParameter( m_maxNumAlfAlternativesChroma < 1 || m_maxNumAlfAlternativesChroma > MAX_NUM_ALF_ALTERNATIVES_CHROMA, std::string( std::string( "The maximum number of ALF Chroma filter alternatives must be in the range (1-" ) + std::to_string( MAX_NUM_ALF_ALTERNATIVES_CHROMA ) + std::string( ", inclusive)" ) ).c_str() );
  }

  confirmParameter( m_useFastMrg < 0 || m_useFastMrg > 2,   "FastMrg out of range [0..2]" );
  confirmParameter( m_useFastMIP < 0 || m_useFastMIP > 4,   "FastMIP out of range [0..4]" );
  confirmParameter( m_fastSubPel < 0 || m_fastSubPel > 1,   "FastSubPel out of range [0..1]" );


  confirmParameter( m_RCRateControlMode != 0 && m_RCRateControlMode != 2, "Invalid rate control mode. Only the frame-level rate control is currently supported" );
  confirmParameter( m_RCRateControlMode == 1 && m_usePerceptQPA > 0, "CTU-level rate control cannot be combined with QPA" );
  confirmParameter( m_RCRateControlMode == 0 && m_RCNumPasses != 1, "Only single pass encoding supported, when rate control is disabled" );
  confirmParameter( m_RCNumPasses < 1 || m_RCNumPasses > 2, "Only one pass or two pass encoding supported" );
  confirmParameter( m_verbosity < SILENT || m_verbosity > DETAILS, "verbosity is out of range[0..6]" );
  confirmParameter(!((m_level==Level::LEVEL1) 
    || (m_level==Level::LEVEL2) || (m_level==Level::LEVEL2_1)
    || (m_level==Level::LEVEL3) || (m_level==Level::LEVEL3_1)
    || (m_level==Level::LEVEL4) || (m_level==Level::LEVEL4_1)
    || (m_level==Level::LEVEL5) || (m_level==Level::LEVEL5_1) || (m_level==Level::LEVEL5_2)
    || (m_level==Level::LEVEL6) || (m_level==Level::LEVEL6_1) || (m_level==Level::LEVEL6_2) || (m_level==Level::LEVEL6_3)
    || (m_level==Level::LEVEL15_5)), "invalid level selected");
  confirmParameter(!((m_levelTier==Tier::TIER_MAIN) || (m_levelTier==Tier::TIER_HIGH)), "invalid tier selected");


  confirmParameter( m_chromaCbQpOffset < -12,           "Min. Chroma Cb QP Offset is -12" );
  confirmParameter( m_chromaCbQpOffset >  12,           "Max. Chroma Cb QP Offset is  12" );
  confirmParameter( m_chromaCrQpOffset < -12,           "Min. Chroma Cr QP Offset is -12" );
  confirmParameter( m_chromaCrQpOffset >  12,           "Max. Chroma Cr QP Offset is  12" );
  confirmParameter( m_chromaCbQpOffsetDualTree < -12,   "Min. Chroma Cb QP Offset for dual tree is -12" );
  confirmParameter( m_chromaCbQpOffsetDualTree >  12,   "Max. Chroma Cb QP Offset for dual tree is  12" );
  confirmParameter( m_chromaCrQpOffsetDualTree < -12,   "Min. Chroma Cr QP Offset for dual tree is -12" );
  confirmParameter( m_chromaCrQpOffsetDualTree >  12,   "Max. Chroma Cr QP Offset for dual tree is  12" );
  if ( m_JointCbCrMode && (m_internChromaFormat == CHROMA_400) )
  {
    msg( WARNING, "****************************************************************************\n");
    msg( WARNING, "** WARNING: --JointCbCr has been disabled because the chromaFormat is 400 **\n");
    msg( WARNING, "****************************************************************************\n");
    m_JointCbCrMode = false;
  }
  if ( m_JointCbCrMode )
  {
    confirmParameter( m_chromaCbCrQpOffset < -12, "Min. Joint Cb-Cr QP Offset is -12");
    confirmParameter( m_chromaCbCrQpOffset >  12, "Max. Joint Cb-Cr QP Offset is  12");
    confirmParameter( m_chromaCbCrQpOffsetDualTree < -12, "Min. Joint Cb-Cr QP Offset for dual tree is -12");
    confirmParameter( m_chromaCbCrQpOffsetDualTree >  12, "Max. Joint Cb-Cr QP Offset for dual tree is  12");
  }

  if (m_lumaLevelToDeltaQPEnabled)
  {
    CONFIRM_PARAMETER_OR_RETURN(m_usePerceptQPA != 0 && m_usePerceptQPA != 5, "LumaLevelToDeltaQP and PerceptQPA conflict");
    msg( WARNING, "\n using deprecated LumaLevelToDeltaQP to force PerceptQPA mode 5" );
    m_usePerceptQPA = 5; // force QPA mode
  }

  if (m_usePerceptQPA && m_dualITree && (m_internChromaFormat != CHROMA_400) && (m_chromaCbQpOffsetDualTree != 0 || m_chromaCrQpOffsetDualTree != 0 || m_chromaCbCrQpOffsetDualTree != 0))
  {
    msg(WARNING, "***************************************************************************\n");
    msg(WARNING, "** WARNING: chroma QPA on, ignoring nonzero dual-tree chroma QP offsets! **\n");
    msg(WARNING, "***************************************************************************\n");
  }

  if (m_usePerceptQPATempFiltISlice < 0 )
  {
    m_usePerceptQPATempFiltISlice = 0;
    if ( (m_usePerceptQPA == 2 || m_usePerceptQPA == 4 )
        && m_RCRateControlMode > 0
        && m_RCNumPasses == 2
        && m_QP <= MAX_QP_PERCEPT_QPA
        && m_GOPSize > 8
        && m_IntraPeriod >= 2 * m_GOPSize )
    {
      m_usePerceptQPATempFiltISlice = 1;
    }
  }
  if (m_cuQpDeltaSubdiv < 0)
  {
    m_cuQpDeltaSubdiv = 0;
    if ( m_usePerceptQPA > 0
        && m_QP <= MAX_QP_PERCEPT_QPA
        && m_CTUSize == 128
        && m_PadSourceWidth <= 2048
        && m_PadSourceHeight <= 1280 )
    {
      m_cuQpDeltaSubdiv = 2;
    }
    if ( m_usePerceptQPATempFiltISlice
        && m_RCNumPasses == 2
        && m_CTUSize == 128)
    {
      m_cuQpDeltaSubdiv = 2; // use subdiv. 2 even for UHD with 2-pass rate control
    }
  }
  if (m_sliceChromaQpOffsetPeriodicity < 0)
  {
    m_sliceChromaQpOffsetPeriodicity = 0;
    if ( m_usePerceptQPA > 0
        && m_internChromaFormat != CHROMA_400 )
    {
      m_sliceChromaQpOffsetPeriodicity = 1;
    }
  }

  confirmParameter( m_usePerceptQPATempFiltISlice && (m_IntraPeriod <= 16 || m_GOPSize <= 8),             "invalid combination of PerceptQPATempFiltIPic, IntraPeriod, and GOPSize" );

  confirmParameter( (m_usePerceptQPA > 0) && (m_cuQpDeltaSubdiv > 2),                                     "MaxCuDQPSubdiv must be 2 or smaller when PerceptQPA is on" );
  if ( m_DecodingRefreshType == 2 )
  {
    confirmParameter( m_IntraPeriod > 0 && m_IntraPeriod <= m_GOPSize ,                                   "Intra period must be larger than GOP size for periodic IDR pictures");
  }
  confirmParameter( m_MaxCodingDepth > MAX_CU_DEPTH,                                                      "MaxPartitionDepth exceeds predefined MAX_CU_DEPTH limit");
  confirmParameter( m_MinQT[0] < 1<<MIN_CU_LOG2,                                                          "Minimum QT size should be larger than or equal to 4");
  confirmParameter( m_MinQT[1] < 1<<MIN_CU_LOG2,                                                          "Minimum QT size should be larger than or equal to 4");
  confirmParameter( m_CTUSize < 32,                                                                       "CTUSize must be greater than or equal to 32");
  confirmParameter( m_CTUSize > 128,                                                                      "CTUSize must be less than or equal to 128");
  confirmParameter( m_CTUSize != 32 && m_CTUSize != 64 && m_CTUSize != 128,                               "CTUSize must be a power of 2 (32, 64, or 128)");
  confirmParameter( m_MaxCodingDepth < 1,                                                                 "MaxPartitionDepth must be greater than zero");
  confirmParameter( (m_CTUSize  >> ( m_MaxCodingDepth - 1 ) ) < 8,                                        "Minimum partition width size should be larger than or equal to 8");
  confirmParameter( (m_PadSourceWidth  % std::max( 8, int(m_CTUSize  >> ( m_MaxCodingDepth - 1 )))) != 0, "Resulting coded frame width must be a multiple of Max(8, the minimum CU size)");
  confirmParameter( m_log2MaxTbSize > 6,                                                                  "Log2MaxTbSize must be 6 or smaller." );
  confirmParameter( m_log2MaxTbSize < 5,                                                                  "Log2MaxTbSize must be 5 or greater." );

  confirmParameter( m_PadSourceWidth  % SPS::getWinUnitX(m_internChromaFormat) != 0, "Picture width must be an integer multiple of the specified chroma subsampling");
  confirmParameter( m_PadSourceHeight % SPS::getWinUnitY(m_internChromaFormat) != 0, "Picture height must be an integer multiple of the specified chroma subsampling");

  confirmParameter( m_aiPad[0] % SPS::getWinUnitX(m_internChromaFormat) != 0, "Horizontal padding must be an integer multiple of the specified chroma subsampling");
  confirmParameter( m_aiPad[1] % SPS::getWinUnitY(m_internChromaFormat) != 0, "Vertical padding must be an integer multiple of the specified chroma subsampling");

  confirmParameter( m_confWinLeft   % SPS::getWinUnitX(m_internChromaFormat) != 0, "Left conformance window offset must be an integer multiple of the specified chroma subsampling");
  confirmParameter( m_confWinRight  % SPS::getWinUnitX(m_internChromaFormat) != 0, "Right conformance window offset must be an integer multiple of the specified chroma subsampling");
  confirmParameter( m_confWinTop    % SPS::getWinUnitY(m_internChromaFormat) != 0, "Top conformance window offset must be an integer multiple of the specified chroma subsampling");
  confirmParameter( m_confWinBottom % SPS::getWinUnitY(m_internChromaFormat) != 0, "Bottom conformance window offset must be an integer multiple of the specified chroma subsampling");

  confirmParameter( m_numThreads < -1 || m_numThreads > 256,              "Number of threads out of range");
  confirmParameter( m_ensureWppBitEqual < 0 || m_ensureWppBitEqual > 1,   "WppBitEqual out of range");
  confirmParameter( m_numThreads > 0 && m_ensureWppBitEqual == 0,         "NumThreads > 0 requires WppBitEqual > 0");
  confirmParameter( m_maxParallelFrames < -1,                             "Max parallel frames out of range" );
  confirmParameter( m_maxParallelFrames > 0 && m_numThreads == 0,         "For frame parallel processing NumThreads > 0 is required" );
  confirmParameter( m_maxParallelFrames > m_InputQueueSize,               "Max parallel frames should be less than size of input queue" );

  if( m_maxParallelFrames )
  {
    confirmParameter( m_useAMaxBT,             "Frame parallel processing: AMaxBT is not supported (must be disabled)" );
    confirmParameter( m_cabacInitPresent,      "Frame parallel processing: CabacInitPresent is not supported (must be disabled)" );
    confirmParameter( m_saoEncodingRate > 0.0, "Frame parallel processing: SaoEncodingRate is not supported (must be disabled)" );
    confirmParameter( m_alfTempPred,           "Frame parallel processing: ALFTempPred is not supported (must be disabled)" );
#if ENABLE_TRACING
    confirmParameter( !m_traceFile.empty() && m_maxParallelFrames > 1, "Tracing and frame parallel encoding not supported" );
#endif
  }

  confirmParameter(((m_PadSourceWidth) & 7) != 0, "internal picture width must be a multiple of 8 - check cropping options");
  confirmParameter(((m_PadSourceHeight) & 7) != 0, "internal picture height must be a multiple of 8 - check cropping options");


  confirmParameter( m_maxNumMergeCand < 1,                              "MaxNumMergeCand must be 1 or greater.");
  confirmParameter( m_maxNumMergeCand > MRG_MAX_NUM_CANDS,              "MaxNumMergeCand must be no more than MRG_MAX_NUM_CANDS." );
  confirmParameter( m_maxNumGeoCand > GEO_MAX_NUM_UNI_CANDS,            "MaxNumGeoCand must be no more than GEO_MAX_NUM_UNI_CANDS." );
  confirmParameter( m_maxNumGeoCand > m_maxNumMergeCand,                "MaxNumGeoCand must be no more than MaxNumMergeCand." );
  confirmParameter( 0 < m_maxNumGeoCand && m_maxNumGeoCand < 2,         "MaxNumGeoCand must be no less than 2 unless MaxNumGeoCand is 0." );
  confirmParameter( m_maxNumAffineMergeCand < (m_SbTMVP ? 1 : 0),       "MaxNumAffineMergeCand must be greater than 0 when SbTMVP is enabled");
  confirmParameter( m_maxNumAffineMergeCand > AFFINE_MRG_MAX_NUM_CANDS, "MaxNumAffineMergeCand must be no more than AFFINE_MRG_MAX_NUM_CANDS." );


  confirmParameter( m_hrdParametersPresent && (0 == m_RCRateControlMode),   "HrdParametersPresent requires RateControl enabled");
  confirmParameter( m_bufferingPeriodSEIEnabled && !m_hrdParametersPresent, "BufferingPeriodSEI requires HrdParametersPresent enabled");
  confirmParameter( m_pictureTimingSEIEnabled && !m_hrdParametersPresent,   "PictureTimingSEI requires HrdParametersPresent enabled");

  // max CU width and height should be power of 2
  uint32_t ui = m_CTUSize;
  while(ui)
  {
    ui >>= 1;
    if( (ui & 1) == 1)
    {
      confirmParameter( ui != 1 , "CTU Size should be 2^n");
    }
  }

  /* if this is an intra-only sequence, ie IntraPeriod=1, don't verify the GOP structure
   * This permits the ability to omit a GOP structure specification */
  if ( m_IntraPeriod == 1 && m_GOPList[0].m_POC == -1 )
  {
    m_GOPList[0] = GOPEntry();
    m_GOPList[0].m_QPFactor = 1;
    m_GOPList[0].m_betaOffsetDiv2 = 0;
    m_GOPList[0].m_tcOffsetDiv2 = 0;
    m_GOPList[0].m_POC = 1;
    m_RPLList0[0] = RPLEntry();
    m_RPLList1[0] = RPLEntry();
    m_RPLList0[0].m_POC = m_RPLList1[0].m_POC = 1;
    m_RPLList0[0].m_numRefPicsActive = 4;
    m_GOPList[0].m_numRefPicsActive[0] = 4;
  }
  else
  {
    // set default RA config
    if( m_GOPSize == 16 && m_GOPList[0].m_POC == -1 && m_GOPList[1].m_POC == -1 )
    {
      for( int i = 0; i < 16; i++ )
      {
        m_GOPList[i] = GOPEntry();
        m_GOPList[i].m_sliceType = 'B';
        m_GOPList[i].m_QPFactor = 1;

        m_GOPList[i].m_numRefPicsActive[0] = 2;
        m_GOPList[i].m_numRefPicsActive[1] = 2;
        m_GOPList[i].m_numRefPics[0] = 2;
        m_GOPList[i].m_numRefPics[1] = 2;
      }
      m_GOPList[0].m_POC  = 16;  m_GOPList[0].m_temporalId  = 0;
      m_GOPList[1].m_POC  =  8;  m_GOPList[1].m_temporalId  = 1;
      m_GOPList[2].m_POC  =  4;  m_GOPList[2].m_temporalId  = 2;
      m_GOPList[3].m_POC  =  2;  m_GOPList[3].m_temporalId  = 3;
      m_GOPList[4].m_POC  =  1;  m_GOPList[4].m_temporalId  = 4;
      m_GOPList[5].m_POC  =  3;  m_GOPList[5].m_temporalId  = 4;
      m_GOPList[6].m_POC  =  6;  m_GOPList[6].m_temporalId  = 3;
      m_GOPList[7].m_POC  =  5;  m_GOPList[7].m_temporalId  = 4;
      m_GOPList[8].m_POC  =  7;  m_GOPList[8].m_temporalId  = 4;
      m_GOPList[9].m_POC  = 12;  m_GOPList[9].m_temporalId  = 2;
      m_GOPList[10].m_POC = 10;  m_GOPList[10].m_temporalId = 3;
      m_GOPList[11].m_POC =  9;  m_GOPList[11].m_temporalId = 4;
      m_GOPList[12].m_POC = 11;  m_GOPList[12].m_temporalId = 4;
      m_GOPList[13].m_POC = 14;  m_GOPList[13].m_temporalId = 3;
      m_GOPList[14].m_POC = 13;  m_GOPList[14].m_temporalId = 4;
      m_GOPList[15].m_POC = 15;  m_GOPList[15].m_temporalId = 4;

      m_GOPList[0].m_numRefPics[0]  = 3;
      m_GOPList[8].m_numRefPics[0]  = 3;
      m_GOPList[12].m_numRefPics[0] = 3;
      m_GOPList[13].m_numRefPics[0] = 3;
      m_GOPList[14].m_numRefPics[0] = 3;
      m_GOPList[15].m_numRefPics[0] = 4;

      m_GOPList[0].m_deltaRefPics[0][0]  = 16; m_GOPList[0].m_deltaRefPics[0][1]  = 32; m_GOPList[0].m_deltaRefPics[0][2]  = 24;
      m_GOPList[1].m_deltaRefPics[0][0]  =  8; m_GOPList[1].m_deltaRefPics[0][1]  = 16;
      m_GOPList[2].m_deltaRefPics[0][0]  =  4; m_GOPList[2].m_deltaRefPics[0][1]  = 12;
      m_GOPList[3].m_deltaRefPics[0][0]  =  2; m_GOPList[3].m_deltaRefPics[0][1]  = 10;
      m_GOPList[4].m_deltaRefPics[0][0]  =  1; m_GOPList[4].m_deltaRefPics[0][1]  = -1;
      m_GOPList[5].m_deltaRefPics[0][0]  =  1; m_GOPList[5].m_deltaRefPics[0][1]  = 3;
      m_GOPList[6].m_deltaRefPics[0][0]  =  2; m_GOPList[6].m_deltaRefPics[0][1]  = 6;
      m_GOPList[7].m_deltaRefPics[0][0]  =  1; m_GOPList[7].m_deltaRefPics[0][1]  = 5;
      m_GOPList[8].m_deltaRefPics[0][0]  =  1; m_GOPList[8].m_deltaRefPics[0][1]  = 3; m_GOPList[8].m_deltaRefPics[0][2]  = 7;
      m_GOPList[9].m_deltaRefPics[0][0]  =  4; m_GOPList[9].m_deltaRefPics[0][1]  = 12;
      m_GOPList[10].m_deltaRefPics[0][0] =  2; m_GOPList[10].m_deltaRefPics[0][1] = 10;
      m_GOPList[11].m_deltaRefPics[0][0] =  1; m_GOPList[11].m_deltaRefPics[0][1] = 9;
      m_GOPList[12].m_deltaRefPics[0][0] =  1; m_GOPList[12].m_deltaRefPics[0][1] = 3; m_GOPList[12].m_deltaRefPics[0][2]  = 11;
      m_GOPList[13].m_deltaRefPics[0][0] =  2; m_GOPList[13].m_deltaRefPics[0][1] = 6; m_GOPList[13].m_deltaRefPics[0][2]  = 14;
      m_GOPList[14].m_deltaRefPics[0][0] =  1; m_GOPList[14].m_deltaRefPics[0][1] = 5; m_GOPList[14].m_deltaRefPics[0][2]  = 13;
      m_GOPList[15].m_deltaRefPics[0][0] =  1; m_GOPList[15].m_deltaRefPics[0][1] = 3; m_GOPList[15].m_deltaRefPics[0][2]  = 7; m_GOPList[15].m_deltaRefPics[0][3]  = 15;

      m_GOPList[3].m_numRefPics[1]  = 3;
      m_GOPList[4].m_numRefPics[1]  = 4;
      m_GOPList[5].m_numRefPics[1]  = 3;
      m_GOPList[7].m_numRefPics[1]  = 3;
      m_GOPList[11].m_numRefPics[1] = 3;

      m_GOPList[0].m_deltaRefPics[1][0]  = 16; m_GOPList[0].m_deltaRefPics[1][1]  =  32;
      m_GOPList[1].m_deltaRefPics[1][0]  = -8; m_GOPList[1].m_deltaRefPics[1][1]  =   8;
      m_GOPList[2].m_deltaRefPics[1][0]  = -4; m_GOPList[2].m_deltaRefPics[1][1]  = -12;
      m_GOPList[3].m_deltaRefPics[1][0]  = -2; m_GOPList[3].m_deltaRefPics[1][1]  =  -6; m_GOPList[3].m_deltaRefPics[1][2]  = -14;
      m_GOPList[4].m_deltaRefPics[1][0]  = -1; m_GOPList[4].m_deltaRefPics[1][1]  =  -3; m_GOPList[4].m_deltaRefPics[1][2]  =  -7;  m_GOPList[4].m_deltaRefPics[1][3]  = -15;
      m_GOPList[5].m_deltaRefPics[1][0]  = -1; m_GOPList[5].m_deltaRefPics[1][1]  =  -5; m_GOPList[5].m_deltaRefPics[1][2]  = -13;
      m_GOPList[6].m_deltaRefPics[1][0]  = -2; m_GOPList[6].m_deltaRefPics[1][1]  =  -10;
      m_GOPList[7].m_deltaRefPics[1][0]  = -1; m_GOPList[7].m_deltaRefPics[1][1]  =  -3; m_GOPList[7].m_deltaRefPics[1][2]  = -11;
      m_GOPList[8].m_deltaRefPics[1][0]  = -1; m_GOPList[8].m_deltaRefPics[1][1]  =  -9;
      m_GOPList[9].m_deltaRefPics[1][0]  = -4; m_GOPList[9].m_deltaRefPics[1][1]  =   4;
      m_GOPList[10].m_deltaRefPics[1][0] = -2; m_GOPList[10].m_deltaRefPics[1][1] =  -6;
      m_GOPList[11].m_deltaRefPics[1][0] = -1; m_GOPList[11].m_deltaRefPics[1][1] =  -3; m_GOPList[11].m_deltaRefPics[1][2]  = -7;
      m_GOPList[12].m_deltaRefPics[1][0] = -1; m_GOPList[12].m_deltaRefPics[1][1] =  -5;
      m_GOPList[13].m_deltaRefPics[1][0] = -2; m_GOPList[13].m_deltaRefPics[1][1] =   2;
      m_GOPList[14].m_deltaRefPics[1][0] = -1; m_GOPList[14].m_deltaRefPics[1][1] =  -3;
      m_GOPList[15].m_deltaRefPics[1][0] = -1; m_GOPList[15].m_deltaRefPics[1][1] =   1;

      for( int i = 0; i < 16; i++ )
      {
        switch( m_GOPList[i].m_temporalId )
        {
        case 0: m_GOPList[i].m_QPOffset   = 1;
                m_GOPList[i].m_QPOffsetModelOffset = 0.0;
                m_GOPList[i].m_QPOffsetModelScale  = 0.0;
        break;
        case 1: m_GOPList[i].m_QPOffset   = 1;
                m_GOPList[i].m_QPOffsetModelOffset = -4.8848;
                m_GOPList[i].m_QPOffsetModelScale  = 0.2061;
        break;
        case 2: m_GOPList[i].m_QPOffset   = 4;
                m_GOPList[i].m_QPOffsetModelOffset = -5.7476;
                m_GOPList[i].m_QPOffsetModelScale  = 0.2286;
        break;
        case 3: m_GOPList[i].m_QPOffset   = 5;
                m_GOPList[i].m_QPOffsetModelOffset = -5.90;
                m_GOPList[i].m_QPOffsetModelScale  = 0.2333;
        break;
        case 4: m_GOPList[i].m_QPOffset   = 6;
                m_GOPList[i].m_QPOffsetModelOffset = -7.1444;
                m_GOPList[i].m_QPOffsetModelScale  = 0.3;
        break;
        default: break;
        }
      }
    }
    else if( m_GOPSize == 32 &&
            ( (m_GOPList[0].m_POC == -1 && m_GOPList[1].m_POC == -1) ||
              (m_GOPList[16].m_POC == -1 && m_GOPList[17].m_POC == -1)
              ) )
    {
      for( int i = 0; i < 32; i++ )
      {
        m_GOPList[i] = GOPEntry();
        m_GOPList[i].m_sliceType = 'B';
        m_GOPList[i].m_QPFactor = 1;

        m_GOPList[i].m_numRefPicsActive[0] = 2;
        m_GOPList[i].m_numRefPicsActive[1] = 2;
        m_GOPList[i].m_numRefPics[0] = 2;
        m_GOPList[i].m_numRefPics[1] = 2;
      }
      m_GOPList[ 0].m_POC = 32;   m_GOPList[0].m_temporalId  = 0;
      m_GOPList[ 1].m_POC = 16;   m_GOPList[1].m_temporalId  = 1;
      m_GOPList[ 2].m_POC =  8;   m_GOPList[2].m_temporalId  = 2;
      m_GOPList[ 3].m_POC =  4;   m_GOPList[3].m_temporalId  = 3;
      m_GOPList[ 4].m_POC =  2;   m_GOPList[4].m_temporalId  = 4;
      m_GOPList[ 5].m_POC =  1;   m_GOPList[5].m_temporalId  = 5;
      m_GOPList[ 6].m_POC =  3;   m_GOPList[6].m_temporalId  = 5;
      m_GOPList[ 7].m_POC =  6;   m_GOPList[7].m_temporalId  = 4;
      m_GOPList[ 8].m_POC =  5;   m_GOPList[8].m_temporalId  = 5;
      m_GOPList[ 9].m_POC =  7;   m_GOPList[9].m_temporalId  = 5;
      m_GOPList[10].m_POC = 12;   m_GOPList[10].m_temporalId = 3;
      m_GOPList[11].m_POC = 10;   m_GOPList[11].m_temporalId = 4;
      m_GOPList[12].m_POC =  9;   m_GOPList[12].m_temporalId = 5;
      m_GOPList[13].m_POC = 11;   m_GOPList[13].m_temporalId = 5;
      m_GOPList[14].m_POC = 14;   m_GOPList[14].m_temporalId = 4;
      m_GOPList[15].m_POC = 13;   m_GOPList[15].m_temporalId = 5;

      m_GOPList[16].m_POC = 15;   m_GOPList[16].m_temporalId = 5;
      m_GOPList[17].m_POC = 24;   m_GOPList[17].m_temporalId = 2;
      m_GOPList[18].m_POC = 20;   m_GOPList[18].m_temporalId = 3;
      m_GOPList[19].m_POC = 18;   m_GOPList[19].m_temporalId = 4;
      m_GOPList[20].m_POC = 17;   m_GOPList[20].m_temporalId = 5;
      m_GOPList[21].m_POC = 19;   m_GOPList[21].m_temporalId = 5;
      m_GOPList[22].m_POC = 22;   m_GOPList[22].m_temporalId = 4;
      m_GOPList[23].m_POC = 21;   m_GOPList[23].m_temporalId = 5;
      m_GOPList[24].m_POC = 23;   m_GOPList[24].m_temporalId = 5;
      m_GOPList[25].m_POC = 28;   m_GOPList[25].m_temporalId = 3;
      m_GOPList[26].m_POC = 26;   m_GOPList[26].m_temporalId = 4;
      m_GOPList[27].m_POC = 25;   m_GOPList[27].m_temporalId = 5;
      m_GOPList[28].m_POC = 27;   m_GOPList[28].m_temporalId = 5;
      m_GOPList[29].m_POC = 30;   m_GOPList[29].m_temporalId = 4;
      m_GOPList[30].m_POC = 29;   m_GOPList[30].m_temporalId = 5;
      m_GOPList[31].m_POC = 31;   m_GOPList[31].m_temporalId = 5;

      m_GOPList[ 0].m_numRefPics[0] = 3;
      m_GOPList[ 1].m_numRefPics[0] = 2;
      m_GOPList[ 2].m_numRefPics[0] = 2;
      m_GOPList[ 3].m_numRefPics[0] = 2;
      m_GOPList[ 4].m_numRefPics[0] = 2;
      m_GOPList[ 5].m_numRefPics[0] = 2;
      m_GOPList[ 6].m_numRefPics[0] = 2;
      m_GOPList[ 7].m_numRefPics[0] = 2;
      m_GOPList[ 8].m_numRefPics[0] = 2;
      m_GOPList[ 9].m_numRefPics[0] = 3;
      m_GOPList[10].m_numRefPics[0] = 2;
      m_GOPList[11].m_numRefPics[0] = 2;
      m_GOPList[12].m_numRefPics[0] = 2;
      m_GOPList[13].m_numRefPics[0] = 3;
      m_GOPList[14].m_numRefPics[0] = 3;
      m_GOPList[15].m_numRefPics[0] = 3;

      m_GOPList[16].m_numRefPics[0] = 4;
      m_GOPList[17].m_numRefPics[0] = 2;
      m_GOPList[18].m_numRefPics[0] = 2;
      m_GOPList[19].m_numRefPics[0] = 2;
      m_GOPList[20].m_numRefPics[0] = 2;
      m_GOPList[21].m_numRefPics[0] = 3;
      m_GOPList[22].m_numRefPics[0] = 3;
      m_GOPList[23].m_numRefPics[0] = 3;
      m_GOPList[24].m_numRefPics[0] = 4;
      m_GOPList[25].m_numRefPics[0] = 3;
      m_GOPList[26].m_numRefPics[0] = 3;
      m_GOPList[27].m_numRefPics[0] = 3;
      m_GOPList[28].m_numRefPics[0] = 4;
      m_GOPList[29].m_numRefPics[0] = 3;
      m_GOPList[30].m_numRefPics[0] = 3;
      m_GOPList[31].m_numRefPics[0] = 4;

      m_GOPList[ 0].m_deltaRefPics[0][0] = 32; m_GOPList[ 0].m_deltaRefPics[0][1] = 64; m_GOPList[ 0].m_deltaRefPics[0][2] = 48; //th swapped order of ref-pic 1 and 2
      m_GOPList[ 1].m_deltaRefPics[0][0] = 16; m_GOPList[ 1].m_deltaRefPics[0][1] = 32;
      m_GOPList[ 2].m_deltaRefPics[0][0] =  8; m_GOPList[ 2].m_deltaRefPics[0][1] = 24;
      m_GOPList[ 3].m_deltaRefPics[0][0] =  4; m_GOPList[ 3].m_deltaRefPics[0][1] = 20;

      m_GOPList[ 4].m_deltaRefPics[0][0] =  2; m_GOPList[ 4].m_deltaRefPics[0][1] = 18;
      m_GOPList[ 5].m_deltaRefPics[0][0] =  1; m_GOPList[ 5].m_deltaRefPics[0][1] = -1;
      m_GOPList[ 6].m_deltaRefPics[0][0] =  1; m_GOPList[ 6].m_deltaRefPics[0][1] =  3;
      m_GOPList[ 7].m_deltaRefPics[0][0] =  2; m_GOPList[ 7].m_deltaRefPics[0][1] =  6;

      m_GOPList[ 8].m_deltaRefPics[0][0] =  1; m_GOPList[ 8].m_deltaRefPics[0][1] =  5;
      m_GOPList[ 9].m_deltaRefPics[0][0] =  1; m_GOPList[ 9].m_deltaRefPics[0][1] =  3; m_GOPList[ 9].m_deltaRefPics[0][2] =  7;
      m_GOPList[10].m_deltaRefPics[0][0] =  4; m_GOPList[10].m_deltaRefPics[0][1] = 12;
      m_GOPList[11].m_deltaRefPics[0][0] =  2; m_GOPList[11].m_deltaRefPics[0][1] = 10;

      m_GOPList[12].m_deltaRefPics[0][0] =  1; m_GOPList[12].m_deltaRefPics[0][1] =  9;
      m_GOPList[13].m_deltaRefPics[0][0] =  1; m_GOPList[13].m_deltaRefPics[0][1] =  3; m_GOPList[13].m_deltaRefPics[0][2] = 11;
      m_GOPList[14].m_deltaRefPics[0][0] =  2; m_GOPList[14].m_deltaRefPics[0][1] =  6; m_GOPList[14].m_deltaRefPics[0][2] = 14;
      m_GOPList[15].m_deltaRefPics[0][0] =  1; m_GOPList[15].m_deltaRefPics[0][1] =  5; m_GOPList[15].m_deltaRefPics[0][2] = 13;

      m_GOPList[16].m_deltaRefPics[0][0] =  1; m_GOPList[16].m_deltaRefPics[0][1] =  3; m_GOPList[16].m_deltaRefPics[0][2] =  7; m_GOPList[16].m_deltaRefPics[0][3] = 15;
      m_GOPList[17].m_deltaRefPics[0][0] =  8; m_GOPList[17].m_deltaRefPics[0][1] = 24;
      m_GOPList[18].m_deltaRefPics[0][0] =  4; m_GOPList[18].m_deltaRefPics[0][1] = 20;
      m_GOPList[19].m_deltaRefPics[0][0] =  2; m_GOPList[19].m_deltaRefPics[0][1] = 18;

      m_GOPList[20].m_deltaRefPics[0][0] =  1; m_GOPList[20].m_deltaRefPics[0][1] = 17;
      m_GOPList[21].m_deltaRefPics[0][0] =  1; m_GOPList[21].m_deltaRefPics[0][1] =  3; m_GOPList[21].m_deltaRefPics[0][2] = 19;
      m_GOPList[22].m_deltaRefPics[0][0] =  2; m_GOPList[22].m_deltaRefPics[0][1] =  6; m_GOPList[22].m_deltaRefPics[0][2] = 22;
      m_GOPList[23].m_deltaRefPics[0][0] =  1; m_GOPList[23].m_deltaRefPics[0][1] =  5; m_GOPList[23].m_deltaRefPics[0][2] = 21;

      m_GOPList[24].m_deltaRefPics[0][0] =  1; m_GOPList[24].m_deltaRefPics[0][1] =  3; m_GOPList[24].m_deltaRefPics[0][2] =  7; m_GOPList[24].m_deltaRefPics[0][3] = 23;
      m_GOPList[25].m_deltaRefPics[0][0] =  4; m_GOPList[25].m_deltaRefPics[0][1] = 12; m_GOPList[25].m_deltaRefPics[0][2] = 28;
      m_GOPList[26].m_deltaRefPics[0][0] =  2; m_GOPList[26].m_deltaRefPics[0][1] = 10; m_GOPList[26].m_deltaRefPics[0][2] = 26;
      m_GOPList[27].m_deltaRefPics[0][0] =  1; m_GOPList[27].m_deltaRefPics[0][1] =  9; m_GOPList[27].m_deltaRefPics[0][2] = 25;

      m_GOPList[28].m_deltaRefPics[0][0] =  1; m_GOPList[28].m_deltaRefPics[0][1] =  3; m_GOPList[28].m_deltaRefPics[0][2] = 11; m_GOPList[28].m_deltaRefPics[0][3] = 27;
      m_GOPList[29].m_deltaRefPics[0][0] =  2; m_GOPList[29].m_deltaRefPics[0][1] = 14; m_GOPList[29].m_deltaRefPics[0][2] = 30;
      m_GOPList[30].m_deltaRefPics[0][0] =  1; m_GOPList[30].m_deltaRefPics[0][1] = 13; m_GOPList[30].m_deltaRefPics[0][2] = 29;
      m_GOPList[31].m_deltaRefPics[0][0] =  1; m_GOPList[31].m_deltaRefPics[0][1] =  3; m_GOPList[31].m_deltaRefPics[0][2] = 15; m_GOPList[31].m_deltaRefPics[0][3] = 31;

      m_GOPList[ 0].m_numRefPics[1] = 2;
      m_GOPList[ 1].m_numRefPics[1] = 2;
      m_GOPList[ 2].m_numRefPics[1] = 2;
      m_GOPList[ 3].m_numRefPics[1] = 3;
      m_GOPList[ 4].m_numRefPics[1] = 4;
      m_GOPList[ 5].m_numRefPics[1] = 5;
      m_GOPList[ 6].m_numRefPics[1] = 4;
      m_GOPList[ 7].m_numRefPics[1] = 3;
      m_GOPList[ 8].m_numRefPics[1] = 4;
      m_GOPList[ 9].m_numRefPics[1] = 3;
      m_GOPList[10].m_numRefPics[1] = 2;
      m_GOPList[11].m_numRefPics[1] = 3;
      m_GOPList[12].m_numRefPics[1] = 4;
      m_GOPList[13].m_numRefPics[1] = 3;
      m_GOPList[14].m_numRefPics[1] = 2;
      m_GOPList[15].m_numRefPics[1] = 3;

      m_GOPList[16].m_numRefPics[1] = 2;
      m_GOPList[17].m_numRefPics[1] = 2;
      m_GOPList[18].m_numRefPics[1] = 2;
      m_GOPList[19].m_numRefPics[1] = 3;
      m_GOPList[20].m_numRefPics[1] = 4;
      m_GOPList[21].m_numRefPics[1] = 3;
      m_GOPList[22].m_numRefPics[1] = 2;
      m_GOPList[23].m_numRefPics[1] = 3;
      m_GOPList[24].m_numRefPics[1] = 2;
      m_GOPList[25].m_numRefPics[1] = 2;
      m_GOPList[26].m_numRefPics[1] = 2;
      m_GOPList[27].m_numRefPics[1] = 3;
      m_GOPList[28].m_numRefPics[1] = 2;
      m_GOPList[29].m_numRefPics[1] = 2;
      m_GOPList[30].m_numRefPics[1] = 2;
      m_GOPList[31].m_numRefPics[1] = 2;

      m_GOPList[ 0].m_deltaRefPics[1][0] =  32; m_GOPList[ 0].m_deltaRefPics[1][1] =  64; //th48
      m_GOPList[ 1].m_deltaRefPics[1][0] = -16; m_GOPList[ 1].m_deltaRefPics[1][1] =  16;
      m_GOPList[ 2].m_deltaRefPics[1][0] =  -8; m_GOPList[ 2].m_deltaRefPics[1][1] = -24;
      m_GOPList[ 3].m_deltaRefPics[1][0] =  -4; m_GOPList[ 3].m_deltaRefPics[1][1] = -12; m_GOPList[ 3].m_deltaRefPics[1][2] = -28;

      m_GOPList[ 4].m_deltaRefPics[1][0] =  -2; m_GOPList[ 4].m_deltaRefPics[1][1] =  -6; m_GOPList[ 4].m_deltaRefPics[1][2] = -14; m_GOPList[ 4].m_deltaRefPics[1][3] = -30;
      m_GOPList[ 5].m_deltaRefPics[1][0] =  -1; m_GOPList[ 5].m_deltaRefPics[1][1] =  -3; m_GOPList[ 5].m_deltaRefPics[1][2] =  -7; m_GOPList[ 5].m_deltaRefPics[1][3] = -15; m_GOPList[5].m_deltaRefPics[1][4] = -31;
      m_GOPList[ 6].m_deltaRefPics[1][0] =  -1; m_GOPList[ 6].m_deltaRefPics[1][1] =  -5; m_GOPList[ 6].m_deltaRefPics[1][2] = -13; m_GOPList[ 6].m_deltaRefPics[1][3] = -29;
      m_GOPList[ 7].m_deltaRefPics[1][0] =  -2; m_GOPList[ 7].m_deltaRefPics[1][1] = -10; m_GOPList[ 7].m_deltaRefPics[1][2] = -26;

      m_GOPList[ 8].m_deltaRefPics[1][0] =  -1; m_GOPList[ 8].m_deltaRefPics[1][1] =  -3; m_GOPList[ 8].m_deltaRefPics[1][2] = -11; m_GOPList[ 8].m_deltaRefPics[1][3] = -27;
      m_GOPList[ 9].m_deltaRefPics[1][0] =  -1; m_GOPList[ 9].m_deltaRefPics[1][1] =  -9; m_GOPList[ 9].m_deltaRefPics[1][2] = -25;
      m_GOPList[10].m_deltaRefPics[1][0] =  -4; m_GOPList[10].m_deltaRefPics[1][1] = -20;
      m_GOPList[11].m_deltaRefPics[1][0] =  -2; m_GOPList[11].m_deltaRefPics[1][1] =  -6; m_GOPList[11].m_deltaRefPics[1][2] = -22;

      m_GOPList[12].m_deltaRefPics[1][0] =  -1; m_GOPList[12].m_deltaRefPics[1][1] =  -3; m_GOPList[12].m_deltaRefPics[1][2] =  -7; m_GOPList[12].m_deltaRefPics[1][3] = -23;
      m_GOPList[13].m_deltaRefPics[1][0] =  -1; m_GOPList[13].m_deltaRefPics[1][1] =  -5; m_GOPList[13].m_deltaRefPics[1][2] = -21;
      m_GOPList[14].m_deltaRefPics[1][0] =  -2; m_GOPList[14].m_deltaRefPics[1][1] = -18;
      m_GOPList[15].m_deltaRefPics[1][0] =  -1; m_GOPList[15].m_deltaRefPics[1][1] =  -3; m_GOPList[15].m_deltaRefPics[1][2] = -19;

      m_GOPList[16].m_deltaRefPics[1][0] =  -1; m_GOPList[16].m_deltaRefPics[1][1] = -17;
      m_GOPList[17].m_deltaRefPics[1][0] =  -8; m_GOPList[17].m_deltaRefPics[1][1] =   8;
      m_GOPList[18].m_deltaRefPics[1][0] =  -4; m_GOPList[18].m_deltaRefPics[1][1] = -12;
      m_GOPList[19].m_deltaRefPics[1][0] =  -2; m_GOPList[19].m_deltaRefPics[1][1] =  -6; m_GOPList[19].m_deltaRefPics[1][2] = -14;

      m_GOPList[20].m_deltaRefPics[1][0] =  -1; m_GOPList[20].m_deltaRefPics[1][1] =  -3; m_GOPList[20].m_deltaRefPics[1][2] =  -7; m_GOPList[20].m_deltaRefPics[1][3] = -15;
      m_GOPList[21].m_deltaRefPics[1][0] =  -1; m_GOPList[21].m_deltaRefPics[1][1] =  -5; m_GOPList[21].m_deltaRefPics[1][2] = -13;
      m_GOPList[22].m_deltaRefPics[1][0] =  -2; m_GOPList[22].m_deltaRefPics[1][1] = -10;
      m_GOPList[23].m_deltaRefPics[1][0] =  -1; m_GOPList[23].m_deltaRefPics[1][1] =  -3; m_GOPList[23].m_deltaRefPics[1][2] = -11;

      m_GOPList[24].m_deltaRefPics[1][0] =  -1; m_GOPList[24].m_deltaRefPics[1][1] =  -9;
      m_GOPList[25].m_deltaRefPics[1][0] =  -4; m_GOPList[25].m_deltaRefPics[1][1] =   4;
      m_GOPList[26].m_deltaRefPics[1][0] =  -2; m_GOPList[26].m_deltaRefPics[1][1] =  -6;
      m_GOPList[27].m_deltaRefPics[1][0] =  -1; m_GOPList[27].m_deltaRefPics[1][1] =  -3; m_GOPList[27].m_deltaRefPics[1][2] = -7;

      m_GOPList[28].m_deltaRefPics[1][0] =  -1; m_GOPList[28].m_deltaRefPics[1][1] =  -5;
      m_GOPList[29].m_deltaRefPics[1][0] =  -2; m_GOPList[29].m_deltaRefPics[1][1] =   2;
      m_GOPList[30].m_deltaRefPics[1][0] =  -1; m_GOPList[30].m_deltaRefPics[1][1] =  -3;
      m_GOPList[31].m_deltaRefPics[1][0] =  -1; m_GOPList[31].m_deltaRefPics[1][1] =   1;

      for( int i = 0; i < 32; i++ )
      {
        switch( m_GOPList[i].m_temporalId )
        {
        case 0: m_GOPList[i].m_QPOffset   = -1;
                m_GOPList[i].m_QPOffsetModelOffset = 0.0;
                m_GOPList[i].m_QPOffsetModelScale  = 0.0;
                break;
        case 1: m_GOPList[i].m_QPOffset   = 0;
                m_GOPList[i].m_QPOffsetModelOffset = -4.9309;
                m_GOPList[i].m_QPOffsetModelScale  =  0.2265;
                break;
        case 2: m_GOPList[i].m_QPOffset   = 0;
                m_GOPList[i].m_QPOffsetModelOffset = -4.5000;
                m_GOPList[i].m_QPOffsetModelScale  =  0.2353;
                break;
        case 3: m_GOPList[i].m_QPOffset   = 3;
                m_GOPList[i].m_QPOffsetModelOffset = -5.4095;
                m_GOPList[i].m_QPOffsetModelScale  =  0.2571;
                break;
        case 4: m_GOPList[i].m_QPOffset   = 5;
                m_GOPList[i].m_QPOffsetModelOffset = -4.4895;
                m_GOPList[i].m_QPOffsetModelScale  =  0.1947;
                break;
        case 5: m_GOPList[i].m_QPOffset   = 6;
                m_GOPList[i].m_QPOffsetModelOffset = -5.4429;
                m_GOPList[i].m_QPOffsetModelScale  =  0.2429;
                break;
        default: break;
        }
      }
    }

    confirmParameter( m_intraOnlyConstraintFlag, "IntraOnlyConstraintFlag cannot be 1 for inter sequences");
  }

  for (int i = 0; m_GOPList[i].m_POC != -1 && i < MAX_GOP + 1; i++)
  {
    m_RPLList0[i].m_POC = m_RPLList1[i].m_POC = m_GOPList[i].m_POC;
    m_RPLList0[i].m_temporalId = m_RPLList1[i].m_temporalId = m_GOPList[i].m_temporalId;
    m_RPLList0[i].m_refPic = m_RPLList1[i].m_refPic = m_GOPList[i].m_refPic;
    m_RPLList0[i].m_sliceType = m_RPLList1[i].m_sliceType = m_GOPList[i].m_sliceType;

    m_RPLList0[i].m_numRefPicsActive = m_GOPList[i].m_numRefPicsActive[0];
    m_RPLList1[i].m_numRefPicsActive = m_GOPList[i].m_numRefPicsActive[1];
    m_RPLList0[i].m_numRefPics = m_GOPList[i].m_numRefPics[0];
    m_RPLList1[i].m_numRefPics = m_GOPList[i].m_numRefPics[1];
    m_RPLList0[i].m_ltrp_in_slice_header_flag = m_GOPList[i].m_ltrp_in_slice_header_flag;
    m_RPLList1[i].m_ltrp_in_slice_header_flag = m_GOPList[i].m_ltrp_in_slice_header_flag;

    for (int j = 0; j < m_GOPList[i].m_numRefPics[0]; j++)
      m_RPLList0[i].m_deltaRefPics[j] = m_GOPList[i].m_deltaRefPics[0][j];
    for (int j = 0; j < m_GOPList[i].m_numRefPics[1]; j++)
      m_RPLList1[i].m_deltaRefPics[j] = m_GOPList[i].m_deltaRefPics[1][j];
  }

  int multipleFactor = /*m_compositeRefEnabled ? 2 :*/ 1;
  bool verifiedGOP=false;
  bool errorGOP=false;
  int checkGOP=1;
  int refList[MAX_NUM_REF_PICS+1] = {0};
  bool isOK[MAX_GOP];
  for(int i=0; i<MAX_GOP; i++)
  {
    isOK[i]=false;
  }
  int numOK=0;
  confirmParameter( m_IntraPeriod >=0&&(m_IntraPeriod%m_GOPSize!=0), "Intra period must be a multiple of GOPSize, or -1" );
  confirmParameter( m_temporalSubsampleRatio < 1, "TemporalSubsampleRatio must be greater than 0");

  for(int i=0; i<m_GOPSize; i++)
  {
    if (m_GOPList[i].m_POC == m_GOPSize * multipleFactor)
    {
      confirmParameter( m_GOPList[i].m_temporalId!=0 , "The last frame in each GOP must have temporal ID = 0 " );
    }
  }

  if ( (m_IntraPeriod != 1) && !m_loopFilterOffsetInPPS && (!m_bLoopFilterDisable) )
  {
    for(int i=0; i<m_GOPSize; i++)
    {
      for( int comp = 0; comp < 3; comp++ )
      {
        confirmParameter( (m_GOPList[i].m_betaOffsetDiv2 + m_loopFilterBetaOffsetDiv2[comp]) < -12 || (m_GOPList[i].m_betaOffsetDiv2 + m_loopFilterBetaOffsetDiv2[comp]) > 12, "Loop Filter Beta Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)" );
        confirmParameter( (m_GOPList[i].m_tcOffsetDiv2 + m_loopFilterTcOffsetDiv2[comp]) < -12 || (m_GOPList[i].m_tcOffsetDiv2 + m_loopFilterTcOffsetDiv2[comp]) > 12, "Loop Filter Tc Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)" );
      }
    }
  }

  for(int i=0; i<m_GOPSize; i++)
  {
    confirmParameter( abs(m_GOPList[i].m_CbQPoffset               ) > 12, "Cb QP Offset for one of the GOP entries exceeds supported range (-12 to 12)" );
    confirmParameter( abs(m_GOPList[i].m_CbQPoffset + m_chromaCbQpOffset) > 12, "Cb QP Offset for one of the GOP entries, when combined with the PPS Cb offset, exceeds supported range (-12 to 12)" );
    confirmParameter( abs(m_GOPList[i].m_CrQPoffset               ) > 12, "Cr QP Offset for one of the GOP entries exceeds supported range (-12 to 12)" );
    confirmParameter( abs(m_GOPList[i].m_CrQPoffset + m_chromaCrQpOffset) > 12, "Cr QP Offset for one of the GOP entries, when combined with the PPS Cr offset, exceeds supported range (-12 to 12)" );
  }
  confirmParameter( abs(m_sliceChromaQpOffsetIntraOrPeriodic[0]                 ) > 12, "Intra/periodic Cb QP Offset exceeds supported range (-12 to 12)" );
  confirmParameter( abs(m_sliceChromaQpOffsetIntraOrPeriodic[0]  + m_chromaCbQpOffset ) > 12, "Intra/periodic Cb QP Offset, when combined with the PPS Cb offset, exceeds supported range (-12 to 12)" );
  confirmParameter( abs(m_sliceChromaQpOffsetIntraOrPeriodic[1]                 ) > 12, "Intra/periodic Cr QP Offset exceeds supported range (-12 to 12)" );
  confirmParameter( abs(m_sliceChromaQpOffsetIntraOrPeriodic[1]  + m_chromaCrQpOffset ) > 12, "Intra/periodic Cr QP Offset, when combined with the PPS Cr offset, exceeds supported range (-12 to 12)" );

  confirmParameter( m_fastLocalDualTreeMode < 0 || m_fastLocalDualTreeMode > 2, "FastLocalDualTreeMode must be in range [0..2]" );

  int extraRPLs = 0;
  int numRefs   = 1;
  //start looping through frames in coding order until we can verify that the GOP structure is correct.
  while (!verifiedGOP && !errorGOP)
  {
    int curGOP = (checkGOP - 1) % m_GOPSize;
    int curPOC = ((checkGOP - 1) / m_GOPSize)*m_GOPSize * multipleFactor + m_RPLList0[curGOP].m_POC;
    if (m_RPLList0[curGOP].m_POC < 0 || m_RPLList1[curGOP].m_POC < 0)
    {
      msg(WARNING, "\nError: found fewer Reference Picture Sets than GOPSize\n");
      errorGOP = true;
    }
    else
    {
      //check that all reference pictures are available, or have a POC < 0 meaning they might be available in the next GOP.
      bool beforeI = false;
      for (int i = 0; i< m_RPLList0[curGOP].m_numRefPics; i++)
      {
        int absPOC = curPOC - m_RPLList0[curGOP].m_deltaRefPics[i];
        if (absPOC < 0)
        {
          beforeI = true;
        }
        else
        {
          bool found = false;
          for (int j = 0; j<numRefs; j++)
          {
            if (refList[j] == absPOC)
            {
              found = true;
              for (int k = 0; k<m_GOPSize; k++)
              {
                if (absPOC % (m_GOPSize * multipleFactor) == m_RPLList0[k].m_POC % (m_GOPSize * multipleFactor))
                {
                  if (m_RPLList0[k].m_temporalId == m_RPLList0[curGOP].m_temporalId)
                  {
                    m_RPLList0[k].m_refPic = true;
                  }
                }
              }
            }
          }
          if (!found)
          {
            msg(WARNING, "\nError: ref pic %d is not available for GOP frame %d\n", m_RPLList0[curGOP].m_deltaRefPics[i], curGOP + 1);
            errorGOP = true;
          }
        }
      }
      if (!beforeI && !errorGOP)
      {
        //all ref frames were present
        if (!isOK[curGOP])
        {
          numOK++;
          isOK[curGOP] = true;
          if (numOK == m_GOPSize)
          {
            verifiedGOP = true;
          }
        }
      }
      else
      {
        //create a new RPLEntry for this frame containing all the reference pictures that were available (POC > 0)
        m_RPLList0[m_GOPSize + extraRPLs] = m_RPLList0[curGOP];
        m_RPLList1[m_GOPSize + extraRPLs] = m_RPLList1[curGOP];
        int newRefs0 = 0;
        for (int i = 0; i< m_RPLList0[curGOP].m_numRefPics; i++)
        {
          int absPOC = curPOC - m_RPLList0[curGOP].m_deltaRefPics[i];
          if (absPOC >= 0)
          {
            m_RPLList0[m_GOPSize + extraRPLs].m_deltaRefPics[newRefs0] = m_RPLList0[curGOP].m_deltaRefPics[i];
            newRefs0++;
          }
        }
        int numPrefRefs0 = m_RPLList0[curGOP].m_numRefPicsActive;

        int newRefs1 = 0;
        for (int i = 0; i< m_RPLList1[curGOP].m_numRefPics; i++)
        {
          int absPOC = curPOC - m_RPLList1[curGOP].m_deltaRefPics[i];
          if (absPOC >= 0)
          {
            m_RPLList1[m_GOPSize + extraRPLs].m_deltaRefPics[newRefs1] = m_RPLList1[curGOP].m_deltaRefPics[i];
            newRefs1++;
          }
        }
        int numPrefRefs1 = m_RPLList1[curGOP].m_numRefPicsActive;

        for (int offset = -1; offset>-checkGOP; offset--)
        {
          //step backwards in coding order and include any extra available pictures we might find useful to replace the ones with POC < 0.
          int offGOP = (checkGOP - 1 + offset) % m_GOPSize;
          int offPOC = ((checkGOP - 1 + offset) / m_GOPSize)*(m_GOPSize * multipleFactor) + m_RPLList0[offGOP].m_POC;
          if (offPOC >= 0 && m_RPLList0[offGOP].m_temporalId <= m_RPLList0[curGOP].m_temporalId)
          {
            bool newRef = false;
            for (int i = 0; i<(newRefs0 + newRefs1); i++)
            {
              if (refList[i] == offPOC)
              {
                newRef = true;
              }
            }
            for (int i = 0; i<newRefs0; i++)
            {
              if (m_RPLList0[m_GOPSize + extraRPLs].m_deltaRefPics[i] == curPOC - offPOC)
              {
                newRef = false;
              }
            }
            if (newRef)
            {
              int insertPoint = newRefs0;
              //this picture can be added, find appropriate place in list and insert it.
              if (m_RPLList0[offGOP].m_temporalId == m_RPLList0[curGOP].m_temporalId)
              {
                m_RPLList0[offGOP].m_refPic = true;
              }
              for (int j = 0; j<newRefs0; j++)
              {
                if (m_RPLList0[m_GOPSize + extraRPLs].m_deltaRefPics[j] > curPOC - offPOC && curPOC - offPOC > 0)
                {
                  insertPoint = j;
                  break;
                }
              }
              int prev = curPOC - offPOC;
              for (int j = insertPoint; j<newRefs0 + 1; j++)
              {
                int newPrev = m_RPLList0[m_GOPSize + extraRPLs].m_deltaRefPics[j];
                m_RPLList0[m_GOPSize + extraRPLs].m_deltaRefPics[j] = prev;
                prev = newPrev;
              }
              newRefs0++;
            }
          }
          if (newRefs0 >= numPrefRefs0)
          {
            break;
          }
        }

        for (int offset = -1; offset>-checkGOP; offset--)
        {
          //step backwards in coding order and include any extra available pictures we might find useful to replace the ones with POC < 0.
          int offGOP = (checkGOP - 1 + offset) % m_GOPSize;
          int offPOC = ((checkGOP - 1 + offset) / m_GOPSize)*(m_GOPSize * multipleFactor) + m_RPLList1[offGOP].m_POC;
          if (offPOC >= 0 && m_RPLList1[offGOP].m_temporalId <= m_RPLList1[curGOP].m_temporalId)
          {
            bool newRef = false;
            for (int i = 0; i<(newRefs0 + newRefs1); i++)
            {
              if (refList[i] == offPOC)
              {
                newRef = true;
              }
            }
            for (int i = 0; i<newRefs1; i++)
            {
              if (m_RPLList1[m_GOPSize + extraRPLs].m_deltaRefPics[i] == curPOC - offPOC)
              {
                newRef = false;
              }
            }
            if (newRef)
            {
              int insertPoint = newRefs1;
              //this picture can be added, find appropriate place in list and insert it.
              if (m_RPLList1[offGOP].m_temporalId == m_RPLList1[curGOP].m_temporalId)
              {
                m_RPLList1[offGOP].m_refPic = true;
              }
              for (int j = 0; j<newRefs1; j++)
              {
                if (m_RPLList1[m_GOPSize + extraRPLs].m_deltaRefPics[j] > curPOC - offPOC && curPOC - offPOC > 0)
                {
                  insertPoint = j;
                  break;
                }
              }
              int prev = curPOC - offPOC;
              for (int j = insertPoint; j<newRefs1 + 1; j++)
              {
                int newPrev = m_RPLList1[m_GOPSize + extraRPLs].m_deltaRefPics[j];
                m_RPLList1[m_GOPSize + extraRPLs].m_deltaRefPics[j] = prev;
                prev = newPrev;
              }
              newRefs1++;
            }
          }
          if (newRefs1 >= numPrefRefs1)
          {
            break;
          }
        }

        m_RPLList0[m_GOPSize + extraRPLs].m_numRefPics = newRefs0;
        m_RPLList0[m_GOPSize + extraRPLs].m_numRefPicsActive = std::min(m_RPLList0[m_GOPSize + extraRPLs].m_numRefPics, m_RPLList0[m_GOPSize + extraRPLs].m_numRefPicsActive);
        m_RPLList1[m_GOPSize + extraRPLs].m_numRefPics = newRefs1;
        m_RPLList1[m_GOPSize + extraRPLs].m_numRefPicsActive = std::min(m_RPLList1[m_GOPSize + extraRPLs].m_numRefPics, m_RPLList1[m_GOPSize + extraRPLs].m_numRefPicsActive);
        curGOP = m_GOPSize + extraRPLs;
        extraRPLs++;
      }
      numRefs = 0;
      for (int i = 0; i< m_RPLList0[curGOP].m_numRefPics; i++)
      {
        int absPOC = curPOC - m_RPLList0[curGOP].m_deltaRefPics[i];
        if (absPOC >= 0)
        {
          refList[numRefs] = absPOC;
          numRefs++;
        }
      }
      for (int i = 0; i< m_RPLList1[curGOP].m_numRefPics; i++)
      {
        int absPOC = curPOC - m_RPLList1[curGOP].m_deltaRefPics[i];
        if (absPOC >= 0)
        {
          bool alreadyExist = false;
          for (int j = 0; !alreadyExist && j < numRefs; j++)
          {
            if (refList[j] == absPOC)
            {
              alreadyExist = true;
            }
          }
          if (!alreadyExist)
          {
            refList[numRefs] = absPOC;
            numRefs++;
          }
        }
      }
      refList[numRefs] = curPOC;
      numRefs++;
    }
    checkGOP++;
  }
  confirmParameter(errorGOP, "Invalid GOP structure given");

  m_maxTempLayer = 1;

  for(int i=0; i<m_GOPSize; i++)
  {
    if(m_GOPList[i].m_temporalId >= m_maxTempLayer)
    {
      m_maxTempLayer = m_GOPList[i].m_temporalId+1;
    }
    confirmParameter(m_GOPList[i].m_sliceType!='B' && m_GOPList[i].m_sliceType!='P' && m_GOPList[i].m_sliceType!='I', "Slice type must be equal to B or P or I");
  }
  for(int i=0; i<MAX_TLAYER; i++)
  {
    m_maxNumReorderPics[i] = 0;
    m_maxDecPicBuffering[i] = 1;
  }
  for(int i=0; i<m_GOPSize; i++)
  {
    int numRefPic = m_RPLList0[i].m_numRefPics;
    for (int tmp = 0; tmp < m_RPLList1[i].m_numRefPics; tmp++)
    {
      bool notSame = true;
      for (int jj = 0; notSame && jj < m_RPLList0[i].m_numRefPics; jj++)
      {
        if (m_RPLList1[i].m_deltaRefPics[tmp] == m_RPLList0[i].m_deltaRefPics[jj]) notSame = false;
      }
      if (notSame) numRefPic++;
    }
    if (numRefPic + 1 > m_maxDecPicBuffering[m_GOPList[i].m_temporalId])
    {
      m_maxDecPicBuffering[m_GOPList[i].m_temporalId] = numRefPic + 1;
    }
    int highestDecodingNumberWithLowerPOC = 0;
    for(int j=0; j<m_GOPSize; j++)
    {
      if(m_GOPList[j].m_POC <= m_GOPList[i].m_POC)
      {
        highestDecodingNumberWithLowerPOC = j;
      }
    }
    int numReorder = 0;
    for(int j=0; j<highestDecodingNumberWithLowerPOC; j++)
    {
      if(m_GOPList[j].m_temporalId <= m_GOPList[i].m_temporalId &&
          m_GOPList[j].m_POC > m_GOPList[i].m_POC)
      {
        numReorder++;
      }
    }
    if(numReorder > m_maxNumReorderPics[m_GOPList[i].m_temporalId])
    {
      m_maxNumReorderPics[m_GOPList[i].m_temporalId] = numReorder;
    }
  }

  for(int i=0; i<MAX_TLAYER-1; i++)
  {
    // a lower layer can not have higher value of m_numReorderPics than a higher layer
    if(m_maxNumReorderPics[i+1] < m_maxNumReorderPics[i])
    {
      m_maxNumReorderPics[i+1] = m_maxNumReorderPics[i];
    }
    // the value of num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] - 1, inclusive
    if(m_maxNumReorderPics[i] > m_maxDecPicBuffering[i] - 1)
    {
      m_maxDecPicBuffering[i] = m_maxNumReorderPics[i] + 1;
    }
    // a lower layer can not have higher value of m_uiMaxDecPicBuffering than a higher layer
    if(m_maxDecPicBuffering[i+1] < m_maxDecPicBuffering[i])
    {
      m_maxDecPicBuffering[i+1] = m_maxDecPicBuffering[i];
    }
  }

  // the value of num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] -  1, inclusive
  if(m_maxNumReorderPics[MAX_TLAYER-1] > m_maxDecPicBuffering[MAX_TLAYER-1] - 1)
  {
    m_maxDecPicBuffering[MAX_TLAYER-1] = m_maxNumReorderPics[MAX_TLAYER-1] + 1;
  }

  confirmParameter( m_MCTF > 2 || m_MCTF < 0, "MCTF out of range" );

  if( m_MCTF && m_QP < 17 )
  {
    msg( WARNING, "disable MCTF for QP < 17\n");
    m_MCTF = 0;
  }

  if( m_MCTF )
  {
    if( m_MCTFFrames.empty() )
    {
      if( m_GOPSize == 32 )
      {
        m_MCTFStrengths.push_back(0.28125); //  9/32
        m_MCTFFrames.push_back(8);
        m_MCTFStrengths.push_back(0.5625);  // 18/32
        m_MCTFFrames.push_back(16);
        m_MCTFStrengths.push_back(0.84375); // 27/32
        m_MCTFFrames.push_back(32);
      }
      else if( m_GOPSize == 16 )
      {
        m_MCTFStrengths.push_back(0.4);
        m_MCTFFrames.push_back(8);
        m_MCTFStrengths.push_back(0.8);
        m_MCTFFrames.push_back(16);
      }
      else if( m_GOPSize == 8 )
      {
        m_MCTFStrengths.push_back(0.65625); // 21/32
        m_MCTFFrames.push_back(8);
      }
      else
      {
        msg( WARNING, "no MCTF frames selected, MCTF will be inactive!\n");
      }
    }

    confirmParameter( m_MCTFFrames.size() != m_MCTFStrengths.size(), "MCTFFrames and MCTFStrengths do not match");
  }

  if ( ! m_MMVD && m_allowDisFracMMVD )
  {
    msg( WARNING, "MMVD disabled, thus disable AllowDisFracMMVD too\n" );
    m_allowDisFracMMVD = false;
  }

  if( m_fastForwardToPOC != -1 )
  {
    if( m_cabacInitPresent )  { msg( WARNING, "WARNING usage of FastForwardToPOC and CabacInitPresent might cause different behaviour\n\n" ); }
    if( m_alf )               { msg( WARNING, "WARNING usage of FastForwardToPOC and ALF might cause different behaviour\n\n" ); }
  }

  //
  // finalize initialization
  //


  // coding structure
  m_numRPLList0 = 0;
  for ( int i = 0; i < MAX_GOP; i++ )
  {
    if ( m_RPLList0[ i ].m_POC != -1 )
      m_numRPLList0++;
  }
  m_numRPLList1 = 0;
  for ( int i = 0; i < MAX_GOP; i++ )
  {
    if ( m_RPLList1[ i ].m_POC != -1 )
      m_numRPLList1++;
  }

  m_PROF &= bool(m_Affine);
  if (m_Affine > 1)
  {
    m_PROF = bool(m_Affine);
    m_AffineType = (m_Affine == 2) ? true : false;
  }

  /// Experimental settings
  // checkExperimental( experimental combination of parameters, "Description!" );

  return( m_confirmFailed );
}

int VVEncCfg::initDefault( int width, int height, int framerate, int targetbitrate, int qp, PresetMode preset )
{
  int iRet = 0;
  m_QP                  = qp;                       // quantization parameter 0-63
  m_SourceWidth         = width;                    // luminance width of input picture
  m_SourceHeight        = height;                   // luminance height of input picture
  m_GOPSize             = 32;                       //  gop size (1: intra only, 16, 32: hierarchical b frames)
  m_DecodingRefreshType = vvenc::DRT_CRA;           // intra period refresh type
  m_IntraPeriodSec      = 1;                        // intra period in seconds for IDR/CDR intra refresh/RAP flag (should be > 0)
  m_IntraPeriod         = 0;                        // intra period in frames for IDR/CDR intra refresh/RAP flag (should be a factor of GopSize)
  m_verbosity           = vvenc::VERBOSE;           // log level > 4 (VERBOSE) enables psnr/rate output
  m_FrameRate           = framerate;                // temporal rate (fps)
  m_TicksPerSecond      = 90000;                    // ticks per second e.g. 90000 for dts generation
  m_framesToBeEncoded   = 0;                        // max number of frames to be encoded
  m_FrameSkip           = 0;                        // number of frames to skip before start encoding
  m_numThreads          = -1;                       // number of worker threads (should not exceed the number of physical cpu's)
  m_usePerceptQPA       = 2;                        // percepual qpa adaptation, 0 off, 1 on for sdr(wpsnr), 2 on for sdr(xpsnr), 3 on for hdr(wpsrn), 4 on for hdr(xpsnr), on for hdr(MeanLuma)
  m_inputBitDepth[0]    = 8;                        // input bitdepth
  m_internalBitDepth[0] = 10;                       // internal bitdepth
  m_profile             = vvenc::Profile::MAIN_10;  // profile: use main_10 or main_10_still_picture
  m_level               = vvenc::Level::LEVEL4_1;   // level
  m_levelTier           = vvenc::Tier::TIER_MAIN;   // tier
  m_SegmentMode         = vvenc::SEG_OFF;           // segment mode

// th this has to go into initcfg
  if( targetbitrate )
  {
    m_RCTargetBitrate       = targetbitrate;        // target bitrate
    m_RCRateControlMode     = RateControlMode::RCM_PICTURE_LEVEL;
    m_RCKeepHierarchicalBit = 2;
    m_RCUseLCUSeparateModel = 1;
    m_RCInitialQP           = 0;
    m_RCForceIntraQP        = 0;
  }
  else
  {
    m_RCTargetBitrate       = 0;
    m_RCRateControlMode     = RateControlMode::RCM_OFF;
  }

  iRet = initPreset( preset );

  return iRet;
}

int VVEncCfg::initPreset( PresetMode preset )
{
  m_qpInValsCb.clear();
  m_qpInValsCb.push_back( 17 );
  m_qpInValsCb.push_back( 22 );
  m_qpInValsCb.push_back( 34 );
  m_qpInValsCb.push_back( 42 );
  m_qpOutValsCb.clear();
  m_qpOutValsCb.push_back( 17 );
  m_qpOutValsCb.push_back( 23 );
  m_qpOutValsCb.push_back( 35 );
  m_qpOutValsCb.push_back( 39 );

  // basic settings
  m_intraQPOffset                 = -3;
  m_lambdaFromQPEnable            = true;
  m_MaxCodingDepth                = 5;
  m_log2DiffMaxMinCodingBlockSize = 5;
  m_bUseASR                       = true;
  m_bUseHADME                     = true;
  m_useRDOQTS                     = true;
  m_useSelectiveRDOQ              = false;
  m_cabacInitPresent              = true;
  m_fastQtBtEnc                   = true;
  m_fastInterSearchMode           = FASTINTERSEARCH_MODE1;
  m_motionEstimationSearchMethod  = MESEARCH_DIAMOND_FAST;
  m_SearchRange                   = 384;
  m_minSearchWindow               = 96;
  m_maxNumMergeCand               = 6;
  m_TSsize                        = 3;
  m_reshapeSignalType             = 0;
  m_updateCtrl                    = 0;
  m_LMCSOffset                    = 6;
  m_RDOQ                          = 1;
  m_SignDataHidingEnabled         = 0;
  m_useFastLCTU                   = 1;

  // partitioning
  m_dualITree                     = 1;
  m_MinQT[ 0 ]                    = 8;
  m_MinQT[ 1 ]                    = 8;
  m_MinQT[ 2 ]                    = 4;
  m_maxMTTDepth                   = 1;
  m_maxMTTDepthI                  = 2;
  m_maxMTTDepthIChroma            = 2;

  // disable tools
  m_Affine                        = 0;
  m_alf                           = 0;
  m_allowDisFracMMVD              = 0;
  m_useBDPCM                      = 0;
  m_BDOF                          = 0;
  m_ccalf                         = 0;
  m_useChromaTS                   = 0;
  m_CIIP                          = 0;
  m_DepQuantEnabled               = 0;
  m_DMVR                          = 0;
  m_EDO                           = 0;
  m_Geo                           = 0;
  m_AMVRspeed                     = 0;
  m_ISP                           = 0;
  m_JointCbCrMode                 = 0;
  m_LFNST                         = 0;
  m_LMChroma                      = 0;
  m_lumaReshapeEnable             = 0;
  m_MCTF                          = 0;
  m_MIP                           = 0;
  m_MMVD                          = 0;
  m_MRL                           = 0;
  m_MTS                           = 0;
  m_MTSImplicit                   = 0;
  m_PROF                          = 0;
  m_bUseSAO                       = 0;
  m_SbTMVP                        = 0;
  m_SBT                           = 0;
  m_SMVD                          = 0;
  m_TMVPModeId                    = 0;
  m_TS                            = 0;
  m_useNonLinearAlfChroma         = 0;
  m_useNonLinearAlfLuma           = 0;

  // enable speedups
  m_qtbttSpeedUp                  = 2;
  m_contentBasedFastQtbt          = 1;
  m_usePbIntraFast                = 1;
  m_useFastMrg                    = 2;
  m_useAMaxBT                     = 1;
  m_useFastMIP                    = 4;
  m_fastLocalDualTreeMode         = 1;
  m_fastSubPel                    = 1;

  switch( preset )
  {
    case PresetMode::FIRSTPASS:
      // Q44B11
      m_MinQT[ 0 ]                = 8;
      m_MinQT[ 1 ]                = 32;
      m_MinQT[ 2 ]                = 4;
      m_maxMTTDepth               = 1;
      m_maxMTTDepthI              = 1;
      m_maxMTTDepthIChroma        = 1;

      m_RDOQ                      = 2;
      m_SignDataHidingEnabled     = 1;

      m_useBDPCM                  = 2;
      m_DMVR                      = 1;
      m_LMChroma                  = 1;
      m_MTSImplicit               = 1;
      m_bUseSAO                   = 1;
      m_TMVPModeId                = 1;
      m_TS                        = 2;
      break;

    case PresetMode::FASTER:
      // Q44B11
      m_MinQT[ 0 ]                = 8;
      m_MinQT[ 1 ]                = 32;
      m_MinQT[ 2 ]                = 4;
      m_maxMTTDepth               = 1;
      m_maxMTTDepthI              = 1;
      m_maxMTTDepthIChroma        = 1;

      m_RDOQ                      = 2;
      m_SignDataHidingEnabled     = 1;

      m_useBDPCM                  = 2;
      m_DMVR                      = 1;
      m_LMChroma                  = 1;
      m_MTSImplicit               = 1;
      m_bUseSAO                   = 1;
      m_TMVPModeId                = 1;
      m_TS                        = 2;
      break;

    case PresetMode::FAST:
      // Q43B11
      m_MinQT[ 0 ]                = 8;
      m_MinQT[ 1 ]                = 16;
      m_MinQT[ 2 ]                = 4;
      m_maxMTTDepth               = 1;
      m_maxMTTDepthI              = 1;
      m_maxMTTDepthIChroma        = 1;

      m_RDOQ                      = 2;
      m_SignDataHidingEnabled     = 1;

      m_alf                       = 1;
      m_ccalf                     = 1;
      m_useBDPCM                  = 2;
      m_DMVR                      = 1;
      m_LMChroma                  = 1;
      m_MCTF                      = 2;
      m_MTSImplicit               = 1;
      m_bUseSAO                   = 1;
      m_TMVPModeId                = 1;
      m_TS                        = 2;
      break;

    case PresetMode::MEDIUM:
      // Q44B21
      m_MinQT[ 0 ]                = 8;
      m_MinQT[ 1 ]                = 8;
      m_MinQT[ 2 ]                = 4;
      m_maxMTTDepth               = 1;
      m_maxMTTDepthI              = 2;
      m_maxMTTDepthIChroma        = 2;

      m_Affine                    = 2;
      m_alf                       = 1;
      m_allowDisFracMMVD          = 1;
      m_useBDPCM                  = 2;
      m_BDOF                      = 1;
      m_ccalf                     = 1;
      m_DepQuantEnabled           = 1;
      m_DMVR                      = 1;
      m_EDO                       = 2;
      m_Geo                       = 3;
      m_AMVRspeed                 = 5;
      m_JointCbCrMode             = 1;
      m_LFNST                     = 1;
      m_LMChroma                  = 1;
      m_lumaReshapeEnable         = 1;
      m_MCTF                      = 2;
      m_MIP                       = 1;
      m_MMVD                      = 3;
      m_MRL                       = 1;
      m_MTSImplicit               = 1;
      m_PROF                      = 1;
      m_bUseSAO                   = 1;
      m_SbTMVP                    = 1;
      m_SMVD                      = 3;
      m_TMVPModeId                = 1;
      m_TS                        = 2;
      break;

    case PresetMode::SLOW:
      // Q44B32
      m_MinQT[ 0 ]                = 8;
      m_MinQT[ 1 ]                = 8;
      m_MinQT[ 2 ]                = 4;
      m_maxMTTDepth               = 2;
      m_maxMTTDepthI              = 3;
      m_maxMTTDepthIChroma        = 3;

      m_Affine                    = 2;
      m_alf                       = 1;
      m_allowDisFracMMVD          = 1;
      m_useBDPCM                  = 2;
      m_BDOF                      = 1;
      m_ccalf                     = 1;
      m_DepQuantEnabled           = 1;
      m_CIIP                      = 1;
      m_DMVR                      = 1;
      m_EDO                       = 2;
      m_Geo                       = 1;
      m_AMVRspeed                 = 1;
      m_ISP                       = 3;
      m_JointCbCrMode             = 1;
      m_LFNST                     = 1;
      m_LMChroma                  = 1;
      m_lumaReshapeEnable         = 1;
      m_MCTF                      = 2;
      m_MIP                       = 1;
      m_MMVD                      = 3;
      m_MRL                       = 1;
      m_MTSImplicit               = 1;
      m_PROF                      = 1;
      m_bUseSAO                   = 1;
      m_SbTMVP                    = 1;
      m_SBT                       = 1;
      m_SMVD                      = 3;
      m_TMVPModeId                = 1;
      m_TS                        = 2;

      m_contentBasedFastQtbt      = 0;
      break;

    case PresetMode::SLOWER:

      m_motionEstimationSearchMethod = MESEARCH_DIAMOND;

      // Q44B33
      m_MinQT[ 0 ]                = 8;
      m_MinQT[ 1 ]                = 8;
      m_MinQT[ 2 ]                = 4;
      m_maxMTTDepth               = 3;
      m_maxMTTDepthI              = 3;
      m_maxMTTDepthIChroma        = 3;

      m_Affine                    = 1;
      m_alf                       = 1;
      m_allowDisFracMMVD          = 1;
      m_useBDPCM                  = 2;
      m_BDOF                      = 1;
      m_ccalf                     = 1;
      m_DepQuantEnabled           = 1;
      m_CIIP                      = 1;
      m_DMVR                      = 1;
      m_EDO                       = 2;
      m_Geo                       = 1;
      m_AMVRspeed                 = 1;
      m_ISP                       = 1;
      m_JointCbCrMode             = 1;
      m_LFNST                     = 1;
      m_LMChroma                  = 1;
      m_lumaReshapeEnable         = 1;
      m_MCTF                      = 2;
      m_MIP                       = 1;
      m_MMVD                      = 1;
      m_MRL                       = 1;
      m_MTS                       = 1;
      m_MTSImplicit               = 0;
      m_PROF                      = 1;
      m_bUseSAO                   = 1;
      m_SbTMVP                    = 1;
      m_SBT                       = 1;
      m_SMVD                      = 1;
      m_TMVPModeId                = 1;
      m_TS                        = 2;
      m_useNonLinearAlfChroma     = 1;
      m_useNonLinearAlfLuma       = 1;

      m_qtbttSpeedUp              = 1;
      m_contentBasedFastQtbt      = 0;
      m_useFastMrg                = 1;
      m_useFastMIP                = 0;
      m_fastSubPel                = 0;
      break;

    case PresetMode::TOOLTEST:
      // Q44B21
      m_MinQT[ 0 ]                = 8;
      m_MinQT[ 1 ]                = 8;
      m_MinQT[ 2 ]                = 4;
      m_maxMTTDepth               = 1;
      m_maxMTTDepthI              = 2;
      m_maxMTTDepthIChroma        = 2;

      m_Affine                    = 2;
      m_alf                       = 1;
      m_allowDisFracMMVD          = 1;
      m_useBDPCM                  = 1;
      m_BDOF                      = 1;
      m_ccalf                     = 1;
      m_DepQuantEnabled           = 1;
      m_CIIP                      = 3;
      m_DMVR                      = 1;
      m_EDO                       = 1;
      m_Geo                       = 2;
      m_AMVRspeed                 = 3;
      m_ISP                       = 2;
      m_JointCbCrMode             = 1;
      m_LFNST                     = 1;
      m_LMChroma                  = 1;
      m_lumaReshapeEnable         = 1;
      m_MCTF                      = 2;
      m_MIP                       = 1;
      m_MMVD                      = 2;
      m_MRL                       = 1;
      m_MTS                       = 1;
      m_PROF                      = 1;
      m_bUseSAO                   = 1;
      m_SbTMVP                    = 1;
      m_SBT                       = 2;
      m_SMVD                      = 3;
      m_TMVPModeId                = 1;
      m_TS                        = 1;
      m_useNonLinearAlfChroma     = 1;
      m_useNonLinearAlfLuma       = 1;
      break;

    default:
      return -1;
  }

  return 0;
}

static inline std::string getProfileStr( int profile )
{
  std::string cT;
  switch( profile )
  {
    case Profile::PROFILE_NONE                         : cT = "none"; break;
    case Profile::MAIN_10                              : cT = "main_10"; break;
    case Profile::MAIN_10_STILL_PICTURE                : cT = "main_10_still_picture"; break;
    case Profile::MAIN_10_444                          : cT = "main_10_444"; break;
    case Profile::MAIN_10_444_STILL_PICTURE            : cT = "main_10_444_still_picture"; break;
    case Profile::MULTILAYER_MAIN_10                   : cT = "multilayer_main_10"; break;
    case Profile::MULTILAYER_MAIN_10_STILL_PICTURE     : cT = "multilayer_main_10_still_picture"; break;
    case Profile::MULTILAYER_MAIN_10_444               : cT = "multilayer_main_10_444"; break;
    case Profile::MULTILAYER_MAIN_10_444_STILL_PICTURE : cT = "multilayer_main_10_444_still_picture"; break;
    case Profile::PROFILE_AUTO                         : cT = "auto"; break;
    default                                            : cT = "unknown"; break;
  }
  return cT;
}

static inline std::string getLevelStr( int level )
{
  std::string cT;
  switch( level )
  {
    case Level::LEVEL_NONE: cT = "none";    break;
    case Level::LEVEL1    : cT = "1";       break;
    case Level::LEVEL2    : cT = "2";       break;
    case Level::LEVEL2_1  : cT = "2.1";     break;
    case Level::LEVEL3    : cT = "3";       break;
    case Level::LEVEL3_1  : cT = "3.1";     break;
    case Level::LEVEL4    : cT = "4";       break;
    case Level::LEVEL4_1  : cT = "4.1";     break;
    case Level::LEVEL5    : cT = "5";       break;
    case Level::LEVEL5_1  : cT = "5.1";     break;
    case Level::LEVEL5_2  : cT = "5.2";     break;
    case Level::LEVEL6    : cT = "6";       break;
    case Level::LEVEL6_1  : cT = "6.1";     break;
    case Level::LEVEL6_2  : cT = "6.2";     break;
    case Level::LEVEL6_3  : cT = "6.3";     break;
    case Level::LEVEL15_5 : cT = "15.5";    break;
    default               : cT = "unknown"; break;
  }
  return cT;
}

static inline std::string getCostFunctionStr( int cost )
{
  std::string cT;
  switch( cost )
  {
    case COST_STANDARD_LOSSY               : cT = "Lossy coding"; break;
    case COST_SEQUENCE_LEVEL_LOSSLESS      : cT = "Sequence level lossless coding"; break;
    case COST_LOSSLESS_CODING              : cT = "Lossless coding"; break;
    case COST_MIXED_LOSSLESS_LOSSY_CODING  : cT = "Mixed lossless lossy coding"; break;
    default                                : cT = "Unknown"; break;
  }
  return cT;
}

std::string VVEncCfg::getConfigAsString( MsgLevel eMsgLevel ) const
{
  std::stringstream css;

  if( eMsgLevel >= DETAILS )
  {
  css << "Real     Format                        : " << m_PadSourceWidth - m_confWinLeft - m_confWinRight << "x" << m_PadSourceHeight - m_confWinTop - m_confWinBottom << " " << (double)m_FrameRate / m_temporalSubsampleRatio << "Hz\n";
  css << "Internal Format                        : " << m_PadSourceWidth << "x" << m_PadSourceHeight << " " <<  (double)m_FrameRate / m_temporalSubsampleRatio << "Hz\n";
  css << "Sequence PSNR output                   : " << (m_printMSEBasedSequencePSNR ? "Linear average, MSE-based" : "Linear average only") << "\n";
  css << "Hexadecimal PSNR output                : " << (m_printHexPsnr ? "Enabled" : "Disabled") << "\n";
  css << "Sequence MSE output                    : " << (m_printSequenceMSE ? "Enabled" : "Disabled") << "\n";
  css << "Frame MSE output                       : " << (m_printFrameMSE ? "Enabled" : "Disabled") << "\n";
  css << "Cabac-zero-word-padding                : " << (m_cabacZeroWordPaddingEnabled ? "Enabled" : "Disabled") << "\n";
  css << "Frame/Field                            : Frame based coding\n";
  if ( m_framesToBeEncoded > 0 )
    css << "Frame index                            : " << m_framesToBeEncoded << " frames\n";
  else
    css << "Frame index                            : all frames\n";

  css << "Profile                                : " << getProfileStr( m_profile ) << "\n";
  css << "Level                                  : " << getLevelStr( m_level ) << "\n";
  css << "CU size / total-depth                  : " << m_CTUSize << " / " << m_MaxCodingDepth << "\n";
  css << "Max TB size                            : " << (1 << m_log2MaxTbSize) << "\n";
  css << "Motion search range                    : " << m_SearchRange << "\n";
  css << "Intra period                           : " << m_IntraPeriod << "\n";
  css << "Decoding refresh type                  : " << m_DecodingRefreshType << "\n";
  css << "QP                                     : " << m_QP << "\n";
  css << "Percept QPA                            : " << m_usePerceptQPA << "\n";
  css << "Max dQP signaling subdiv               : " << m_cuQpDeltaSubdiv << "\n";
  css << "Cb QP Offset (dual tree)               : " << m_chromaCbQpOffset << " (" << m_chromaCbQpOffsetDualTree << ")\n";
  css << "Cr QP Offset (dual tree)               : " << m_chromaCrQpOffset << " (" << m_chromaCrQpOffsetDualTree << ")\n";
  css << "GOP size                               : " << m_GOPSize << "\n";
  css << "Input queue size                       : " << m_InputQueueSize << "\n";
  css << "Input bit depth                        : (Y:" << m_inputBitDepth[ CH_L ] << ", C:" << m_inputBitDepth[ CH_C ] << ")\n";
  css << "MSB-extended bit depth                 : (Y:" << m_MSBExtendedBitDepth[ CH_L ] << ", C:" << m_MSBExtendedBitDepth[ CH_C ] << ")\n";
  css << "Internal bit depth                     : (Y:" << m_internalBitDepth[ CH_L ] << ", C:" << m_internalBitDepth[ CH_C ] << ")\n";
  css << "cu_chroma_qp_offset_subdiv             : " << m_cuChromaQpOffsetSubdiv << "\n";
  if (m_bUseSAO)
  {
    css << "log2_sao_offset_scale_luma             : " << m_log2SaoOffsetScale[ CH_L ] << "\n";
    css << "log2_sao_offset_scale_chroma           : " << m_log2SaoOffsetScale[ CH_C ] << "\n";
  }
  css << "Cost function:                         : " << getCostFunctionStr( m_costMode ) << "\n";
  css << "\n";
  }

  if( eMsgLevel >= VERBOSE )
  {
  // verbose output
  css << "CODING TOOL CFG: ";
  css << "IBD:" << ((m_internalBitDepth[ CH_L ] > m_MSBExtendedBitDepth[ CH_L ]) || (m_internalBitDepth[ CH_C ] > m_MSBExtendedBitDepth[ CH_C ])) << " ";
  css << "CIP:" << m_bUseConstrainedIntraPred << " ";
  css << "SAO:" << (m_bUseSAO ? 1 : 0) << " ";
  css << "ALF:" << (m_alf ? 1 : 0) << " ";
  if( m_alf )
  {
    css << "(NonLinLuma:" << m_useNonLinearAlfLuma << " ";
    css << "NonLinChr:" << m_useNonLinearAlfChroma << ") ";
  }
  css << "CCALF:" << (m_ccalf ? 1 : 0) << " ";

  const int iWaveFrontSubstreams = m_entropyCodingSyncEnabled ? ( m_PadSourceHeight + m_CTUSize - 1 ) / m_CTUSize : 1;
  css << "WPP:" << (m_entropyCodingSyncEnabled ? 1 : 0) << " ";
  css << "WPP-Substreams:" << iWaveFrontSubstreams << " ";
  css << "TMVP:" << m_TMVPModeId << " ";

  css << "DQ:" << m_DepQuantEnabled << " ";
  if( m_DepQuantEnabled )
  {
    if( m_dqThresholdVal & 1 )
      css << "(Thr: " << (m_dqThresholdVal >> 1) << ".5) ";
    else
      css << "(Thr: " << (m_dqThresholdVal >> 1) << ") ";
  }
  css << "SDH:" << m_SignDataHidingEnabled << " ";
  css << "CST:" << m_dualITree << " ";
  css << "BDOF:" << m_BDOF << " ";
  css << "DMVR:" << m_DMVR << " ";
  css << "MTSImplicit:" << m_MTSImplicit << " ";
  css << "SBT:" << m_SBT << " ";
  css << "JCbCr:" << m_JointCbCrMode << " ";
  css << "CabacInitPresent:" << m_cabacInitPresent << " ";
  css << "AMVR:" << m_AMVRspeed << " ";
  css << "SMVD:" << m_SMVD << " ";

  css << "LMCS:" << m_lumaReshapeEnable << " ";
  if( m_lumaReshapeEnable )
  {
    css << "(Signal:" << (m_reshapeSignalType == 0 ? "SDR" : (m_reshapeSignalType == 2 ? "HDR-HLG" : "HDR-PQ")) << " ";
    css << "Opt:" << m_adpOption << "";
    if( m_adpOption > 0 )
    {
      css << " CW:" << m_initialCW << "";
    }
    css << ") ";
  }
  css << "CIIP:" << m_CIIP << " ";
  css << "MIP:" << m_MIP << " ";
  css << "AFFINE:" << m_Affine << " ";
  if( m_Affine )
  {
    css << "(PROF:" << m_PROF << ", ";
    css << "Type:" << m_AffineType << ")";
  }
  css << "MMVD:" << m_MMVD << " ";
  if( m_MMVD )
    css << "DisFracMMVD:" << m_allowDisFracMMVD << " ";
  css << "SbTMVP:" << m_SbTMVP << " ";
  css << "GPM:" << m_Geo << " ";
  css << "LFNST:" << m_LFNST << " ";
  css << "MTS:" << m_MTS << " ";
  if( m_MTS )
  {
    css << "(IntraCand:" << m_MTSIntraMaxCand << ")";
  }
  css << "ISP:" << m_ISP << " ";
  css << "TS:" << m_TS << " ";
  if( m_TS )
  {
    css << "TSLog2MaxSize:" << m_TSsize << " ";
    css << "useChromaTS:" << m_useChromaTS << " ";
  }
  css << "BDPCM:" << m_useBDPCM << " ";

  css << "\nENC. ALG. CFG: ";
  css << "QPA:" << m_usePerceptQPA << " ";
  css << "HAD:" << m_bUseHADME << " ";
  css << "RDQ:" << m_RDOQ << " ";
  css << "RDQTS:" << m_useRDOQTS << " ";
  css << "ASR:" << m_bUseASR << " ";
  css << "MinSearchWindow:" << m_minSearchWindow << " ";
  css << "RestrictMESampling:" << m_bRestrictMESampling << " ";
  css << "EDO:" << m_EDO << " ";
  css << "MCTF:" << m_MCTF << " ";
  if( m_MCTF )
  {
    css << "[L:" << m_MCTFNumLeadFrames << ", T:" << m_MCTFNumTrailFrames << "] ";
  }

  css << "\nFAST TOOL CFG: ";
  css << "ECU:" << m_bUseEarlyCU << " ";
  css << "FEN:" << m_fastInterSearchMode << " ";
  css << "FDM:" << m_useFastDecisionForMerge << " ";
  css << "ESD:" << m_useEarlySkipDetection << " ";
  css << "FastSearch:" << m_motionEstimationSearchMethod << " ";
  css << "LCTUFast:" << m_useFastLCTU << " ";
  css << "FastMrg:" << m_useFastMrg << " ";
  css << "PBIntraFast:" << m_usePbIntraFast << " ";
  css << "AMaxBT:" << m_useAMaxBT << " ";
  css << "FastQtBtEnc:" << m_fastQtBtEnc << " ";
  css << "ContentBasedFastQtbt:" << m_contentBasedFastQtbt << " ";
  if( m_MIP )
  {
    css << "FastMIP:" << m_useFastMIP << " ";
  }
  css << "FastLocalDualTree:" << m_fastLocalDualTreeMode << " ";
  css << "FastSubPel:" << m_fastSubPel << " ";
  css << "QtbttExtraFast:" << m_qtbttSpeedUp << " ";

  css << "\nRATE CONTROL CFG: ";
  css << "RateControl:" << m_RCRateControlMode << " ";
  if ( m_RCRateControlMode )
  {
    css << "Passes:" << m_RCNumPasses << " ";
    css << "TargetBitrate:" << m_RCTargetBitrate << " ";
    css << "KeepHierarchicalBit:" << m_RCKeepHierarchicalBit << " ";
    css << "RCLCUSeparateModel:" << m_RCUseLCUSeparateModel << " ";
    css << "InitialQP:" << m_RCInitialQP << " ";
    css << "RCForceIntraQP:" << m_RCForceIntraQP << " ";
  }

  css << "\nPARALLEL PROCESSING CFG: ";
  css << "NumThreads:" << m_numThreads << " ";
  css << "MaxParallelFrames:" << m_maxParallelFrames << " ";
  css << "WppBitEqual:" << m_ensureWppBitEqual << " ";
  css << "WF:" << m_entropyCodingSyncEnabled << "";
  css << "\n";
  }

  return css.str();
}


} // namespace vvenc

//! \}


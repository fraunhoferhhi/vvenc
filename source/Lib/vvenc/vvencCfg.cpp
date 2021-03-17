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


/** \file     VVEncCfg.cpp
    \brief    encoder configuration class
*/

#include "vvenc/vvencCfg.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Slice.h"
#include "CommonLib/ProfileLevelTier.h"

#include <math.h>
#include <thread>

//! \ingroup Interface
//! \{

namespace vvenc {

VVENC_DECL void vvenc_cfg_default(VVEncCfg *cfg )
{
 cfg->m_confirmFailed        = false;         ///< state variable

 cfg->m_verbosity            = VVENC_VERBOSE;       ///< encoder verbosity
 cfg->m_framesToBeEncoded    = 0;             ///< number of encoded frames

 cfg->m_FrameRate            = 0;             ///< source frame-rates (Hz)
 cfg->m_FrameSkip            = 0;             ///< number of skipped frames from the beginning
 cfg->m_SourceWidth          = 0;             ///< source width in pixel
 cfg->m_SourceHeight         = 0;             ///< source height in pixel (when interlaced = field height)
 cfg->m_TicksPerSecond       = 90000;         ///< ticks per second e.g. 90000 for dts generation (1..27000000)

 cfg->m_profile              = vvencProfile::VVENC_PROFILE_AUTO;
 cfg->m_levelTier            = vvencTier::VVENC_TIER_MAIN ;
 cfg->m_level                = vvencLevel::VVENC_LEVEL_AUTO;

 cfg->m_IntraPeriod          = 0;             ///< period of I-slice (random access period)
 cfg->m_IntraPeriodSec       = 1;             ///< period of I-slice in seconds (random access period)
 cfg->m_DecodingRefreshType  = VVENC_DRT_CRA;       ///< random access type
 cfg->m_GOPSize              = 32;            ///< GOP size of hierarchical structure

 cfg->m_QP                   = 32;            ///< QP value of key-picture (integer)
 cfg->m_usePerceptQPA        = false;         ///< Mode of perceptually motivated input-adaptive QP modification, abbrev. perceptual QP adaptation (QPA).

 cfg->m_RCTargetBitrate      = 0;
 cfg->m_RCNumPasses          = -1;

 cfg->m_SegmentMode          = VVENC_SEG_OFF;

 cfg->m_numThreads           = 0;             ///< number of worker threads

 cfg->m_inputBitDepth[0]=8;     cfg->m_inputBitDepth[1]=0; ///< bit-depth of input file
 cfg->m_internalBitDepth[0]=10; cfg->m_internalBitDepth[1]=0; ///< bit-depth codec operates at (input/output files will be converted)

 cfg->m_HdrMode              = VVENC_HDR_OFF;

  vvenc_initPreset( cfg, vvencPresetMode::VVENC_MEDIUM );
}

VVENC_DECL bool vvenc_confirmParameter ( VVEncCfg *c, bool bflag, const char* message );
{
  if ( ! bflag )
    return false;
  msg( VVENC_ERROR, "Parameter Check Error: %s\n", message );
  c->m_confirmFailed = true;
  return true;
}

VVENC_DECL bool vvenc_initCfgParameter( VVEncCfg *c )
{
  c->m_confirmFailed = false;

  // check for valid base parameter
  confirmParameter(  (c->m_SourceWidth <= 0 || c->m_SourceHeight <= 0), "Error: input resolution not set");

  confirmParameter( c->m_inputBitDepth[CH_L] < 8 || c->m_inputBitDepth[CH_L] > 16,                    "InputBitDepth must be at least 8" );
  confirmParameter( c->m_inputBitDepth[CH_L] != 8 && c->m_inputBitDepth[CH_L] != 10,                  "Input bitdepth must be 8 or 10 bit" );
  confirmParameter( c->m_internalBitDepth[0] != 8 && c->m_internalBitDepth[0] != 10,                  "Internal bitdepth must be 8 or 10 bit" );

  confirmParameter( c->m_FrameRate <= 0,                                                           "Frame rate must be more than 1" );
  confirmParameter( c->m_TicksPerSecond <= 0 || c->m_TicksPerSecond > 27000000,                       "TicksPerSecond must be in range from 1 to 27000000" );

  int temporalRate   = c->m_FrameRate;
  int temporalScale  = 1;

  switch( c->m_FrameRate )
  {
  case 23: temporalRate = 24000; temporalScale = 1001; break;
  case 29: temporalRate = 30000; temporalScale = 1001; break;
  case 59: temporalRate = 60000; temporalScale = 1001; break;
  default: break;
  }

  confirmParameter( (c->m_TicksPerSecond < 90000) && (c->m_TicksPerSecond*temporalScale)%temporalRate, "TicksPerSecond should be a multiple of FrameRate/Framscale" );

  confirmParameter( c->m_numThreads < -1 || c->m_numThreads > 256,              "Number of threads out of range (-1 <= t <= 256)");

  confirmParameter( c->m_IntraPeriod < 0,                 "IDR period (in frames) must be >= 0");
  confirmParameter( c->m_IntraPeriodSec < 0,              "IDR period (in seconds) must be >= 0");

  confirmParameter( c->m_GOPSize < 1 ,                                                             "GOP Size must be greater or equal to 1" );
  confirmParameter( c->m_GOPSize > 1 &&  c->m_GOPSize % 2,                                            "GOP Size must be a multiple of 2, if GOP Size is greater than 1" );
  confirmParameter( c->m_GOPSize > 1 &&  c->m_GOPSize % 2,                                            "GOP Size must be a multiple of 2, if GOP Size is greater than 1" );
  confirmParameter( c->m_GOPSize > 64,                                                             "GOP size must be <= 64" );
  confirmParameter( c->m_GOPSize != 1 && c->m_GOPSize != 16 && c->m_GOPSize != 32,                       "GOP size only supporting: 1, 16, 32" );

  confirmParameter( c->m_QP < 0 || c->m_QP > MAX_QP,                                                  "QP exceeds supported range (0 to 63)" );

  confirmParameter( c->m_RCTargetBitrate < 0 || c->m_RCTargetBitrate > 800000000,                     "TargetBitrate must be between 0 - 800000000" );

  if( 0 == c->m_RCTargetBitrate )
   {
     confirmParameter( c->m_hrdParametersPresent > 0,          "hrdParameters present requires rate control" );
     confirmParameter( c->m_bufferingPeriodSEIEnabled,         "bufferingPeriod SEI enabled requires rate control" );
     confirmParameter( c->m_pictureTimingSEIEnabled,           "pictureTiming SEI enabled requires rate control" );
   }

  confirmParameter( c->m_HdrMode < VVENC_HDR_OFF || c->m_HdrMode > VVENC_HDR_USER_DEFINED,  "HdrMode must be in the range 0 - 5" );

  confirmParameter( c->m_verbosity < VVENC_SILENT || c->m_verbosity > VVENC_DETAILS, "verbosity is out of range[0..6]" );

  if ( c->m_confirmFailed )
  {
    return c->m_confirmFailed;
  }

  //
  // set a lot of dependent parameters
  //

  if ( c->m_internChromaFormat < 0 || c->m_internChromaFormat >= VVENC_NUM_CHROMA_FORMAT )
  {
    c->m_internChromaFormat = VVENC_CHROMA_420;
  }

  if( c->m_profile == vvencProfile::VVENC_PROFILE_AUTO )
  {
    const int maxBitDepth= std::max(c->m_internalBitDepth[CH_L], c->m_internalBitDepth[c->m_internChromaFormat==vvencChromaFormat::VVENC_CHROMA_400 ? CH_L : CH_C]);

    if (c->m_internChromaFormat==vvencChromaFormat::VVENC_CHROMA_400 || m_internChromaFormat==vvencChromaFormat::VVENC_CHROMA_420)
    {
      if (maxBitDepth<=10)
      {
        c->m_profile=vvencProfile::VVENC_MAIN_10;
      }
    }
    else if (c->m_internChromaFormat==vvencChromaFormat::VVENC_CHROMA_422 || c->m_internChromaFormat==vvencChromaFormat::VVENC_CHROMA_444)
    {
      if (maxBitDepth<=10)
      {
        c->m_profile=vvencProfile::VVENC_MAIN_10_444;
      }
    }
  }

  if( c->m_level == vvencLevel::VVENC_LEVEL_AUTO )
  {
    c->m_level = LevelTierFeatures::getLevelForInput( c->m_SourceWidth, c->m_SourceHeight );
  }

  if ( c->m_InputQueueSize <= 0 )
  {
    c->m_InputQueueSize = c->m_GOPSize;

    if ( c->m_MCTF )
    {
      c->m_InputQueueSize += MCTF_ADD_QUEUE_DELAY;
    }
  }

  if( c->m_temporalSubsampleRatio )
  {
    int framesSubsampled = (c->m_framesToBeEncoded + c->m_temporalSubsampleRatio - 1) / c->m_temporalSubsampleRatio;
    if( c->m_framesToBeEncoded != framesSubsampled )
    {
      c->m_framesToBeEncoded = framesSubsampled;
    }
  }

  m_maxBT[0] = std::min( m_CTUSize, m_maxBT[0] );
  m_maxBT[1] = std::min( m_CTUSize, m_maxBT[1] );
  m_maxBT[2] = std::min( m_CTUSize, m_maxBT[2] );

  m_maxTT[0] = std::min( m_CTUSize, m_maxTT[0] );
  m_maxTT[1] = std::min( m_CTUSize, m_maxTT[1] );
  m_maxTT[2] = std::min( m_CTUSize, m_maxTT[2] );

  // set MCTF Lead/Trail frames
  if( c->m_SegmentMode != VVENC_SEG_OFF )
  {
    if( c->m_MCTF )
    {
      switch( c->m_SegmentMode )
      {
        case VVENC_SEG_FIRST:
          c->m_MCTFNumLeadFrames  = 0;
          c->m_MCTFNumTrailFrames = c->m_MCTFNumTrailFrames == 0 ? VVENC_MCTF_RANGE : c->m_MCTFNumTrailFrames;
          break;
        case VVENC_SEG_MID:
          c->m_MCTFNumLeadFrames  = VVENC_MCTF_RANGE;
          c->m_MCTFNumTrailFrames = c->m_MCTFNumTrailFrames == 0 ? VVENC_MCTF_RANGE : c->m_MCTFNumTrailFrames;
          break;
        case VVENC_SEG_LAST:
          c->m_MCTFNumLeadFrames  = c->m_MCTFNumLeadFrames == 0 ? VVENC_MCTF_RANGE : c->m_MCTFNumTrailFrames;
          c->m_MCTFNumTrailFrames = 0;
          break;
        default:
          break;
      }
    }
  }

  // rate control
  if( c->m_RCNumPasses < 0 )                                m_RCNumPasses           = m_RCTargetBitrate > 0 ? 2 : 1;

  // threading
  if( c->m_numThreads < 0 )
  {
    const int numCores = std::thread::hardware_concurrency();
    c->m_numThreads = c->m_SourceWidth > 832 && c->m_SourceHeight > 480 ? 8 : 4;
    c->m_numThreads = std::min( c->m_numThreads, numCores );
  }
  if( m_ensureWppBitEqual < 0 )       m_ensureWppBitEqual     = m_numThreads ? 1   : 0   ;
  if( m_useAMaxBT < 0 )               m_useAMaxBT             = m_numThreads ? 0   : 1   ;
  if( m_cabacInitPresent < 0 )        m_cabacInitPresent      = m_numThreads ? 0   : 1   ;
  if( m_alfTempPred < 0 )             m_alfTempPred           = m_numThreads ? 0   : 1   ;
  if( m_saoEncodingRate < 0.0 )       m_saoEncodingRate       = m_numThreads ? 0.0 : 0.75;
  if( m_saoEncodingRateChroma < 0.0 ) m_saoEncodingRateChroma = m_numThreads ? 0.0 : 0.5 ;
  if( m_maxParallelFrames < 0 )
  {
    m_maxParallelFrames = std::min( m_numThreads, 4 );
    if( m_RCTargetBitrate > 0
        && m_RCNumPasses == 1
        && m_maxParallelFrames > 2 )
    {
      m_maxParallelFrames = 2;
    }
  }

  // MCTF
  m_MCTFNumLeadFrames  = std::min( m_MCTFNumLeadFrames,  VVENC_MCTF_RANGE );
  m_MCTFNumTrailFrames = std::min( m_MCTFNumTrailFrames, VVENC_MCTF_RANGE );

  /* rules for input, output and internal bitdepths as per help text */
  if (m_MSBExtendedBitDepth[CH_L  ] == 0)
    m_MSBExtendedBitDepth[CH_L  ] = c->m_inputBitDepth      [CH_L  ];
  if (m_MSBExtendedBitDepth[CH_C] == 0)
    m_MSBExtendedBitDepth[CH_C] = m_MSBExtendedBitDepth[CH_L  ];
  if (m_internalBitDepth   [CH_L  ] == 0)
    m_internalBitDepth   [CH_L  ] = m_MSBExtendedBitDepth[CH_L  ];
  if (m_internalBitDepth   [CH_C] == 0)
    m_internalBitDepth   [CH_C] = m_internalBitDepth   [CH_L  ];
  if (m_inputBitDepth      [CH_C] == 0)
    m_inputBitDepth      [CH_C] = m_inputBitDepth      [CH_L  ];
  if (m_outputBitDepth     [CH_L  ] == 0)
    m_outputBitDepth     [CH_L  ] = m_internalBitDepth   [CH_L  ];
  if (m_outputBitDepth     [CH_C] == 0)
    m_outputBitDepth     [CH_C] = m_outputBitDepth     [CH_L  ];

  if( m_fastInterSearchMode  == VVENC_FASTINTERSEARCH_AUTO )
  {
    m_fastInterSearchMode = VVENC_FASTINTERSEARCH_MODE1;
  }

  if( c->m_HdrMode == VVENC_HDR_OFF &&
     (( c->m_masteringDisplay[0] != 0 && c->m_masteringDisplay[1] != 0 && c->m_masteringDisplay[8] != 0 && c->m_masteringDisplay[9] != 0 ) ||
     ( c->m_contentLightLevel[0] != 0 && c->m_contentLightLevel[1] != 0 ) ) )
  {
    // enable hdr pq bt2020/bt709 mode (depending on set colour primaries)
    c->m_HdrMode = c->m_colourPrimaries==9 ? VVENC_HDR_PQ_BT2020 : VVENC_HDR_PQ;
  }

  if( c->m_HdrMode == VVENC_HDR_PQ || c->m_HdrMode == VVENC_HDR_PQ_BT2020 )
  {
    m_reshapeSignalType = RESHAPE_SIGNAL_PQ;
    m_LMCSOffset              = 1;
    m_useSameChromaQPTables   = false;
    m_verCollocatedChromaFlag = true;

    VVEncCfg cBaseCfg;
    vvenc_cfg_default(&cBaseCfg);
    if( m_qpInValsCb == cBaseCfg.m_qpInValsCb )
    {
      memset(&m_qpInValsCb,0, sizeof(m_qpInValsCb));
      std::vector<int>  qpInVals = { 13,20,36,38,43,54 };
      std::copy(qpInVals.begin(), qpInVals.end(), m_qpInValsCb);
    }
    if( m_qpOutValsCb == cBaseCfg.m_qpOutValsCb )
    {
      memset(&m_qpOutValsCb,0, sizeof(m_qpOutValsCb));
      std::vector<int>  qpInVals = { 13,21,29,29,32,37 };
      std::copy(qpInVals.begin(), qpInVals.end(), m_qpOutValsCb);
    }
    if( m_qpInValsCr == cBaseCfg.m_qpInValsCr )
    {
      memset(&m_qpInValsCr,0, sizeof(m_qpInValsCr));
      std::vector<int>  qpInVals = { 13,20,37,41,44,54 };
      std::copy(qpInVals.begin(), qpInVals.end(), m_qpInValsCr);
    }
    if( m_qpOutValsCr == cBaseCfg.m_qpOutValsCr )
    {
      memset(&m_qpOutValsCr,0, sizeof(m_qpOutValsCr));
      std::vector<int>  qpInVals = { 13,21,27,29,32,37 };
      std::copy(qpInVals.begin(), qpInVals.end(), m_qpOutValsCr);
    }
    if( m_qpInValsCbCr == cBaseCfg.m_qpInValsCbCr )
    {
      memset(&m_qpInValsCbCr,0, sizeof(m_qpInValsCbCr));
      std::vector<int>  qpInVals = { 12,21,41,43,54 };
      std::copy(qpInVals.begin(), qpInVals.end(), m_qpInValsCbCr);
    }
    if( m_qpOutValsCbCr == cBaseCfg.m_qpOutValsCbCr )
    {
      memset(&m_qpOutValsCbCr,0, sizeof(m_qpOutValsCbCr));
      std::vector<int>  qpInVals = { 12,22,30,32,37 };
      std::copy(qpInVals.begin(), qpInVals.end(), m_qpOutValsCbCr);
    }

    // VUI and SEI options
    m_vuiParametersPresent     = m_vuiParametersPresent != 0 ? 1:0; // enable vui only if not explicitly disabled
    m_colourDescriptionPresent = true;                                // enable colour_primaries, transfer_characteristics and matrix_coefficients in vui

    m_transferCharacteristics = 16; // smpte2084 - HDR10
    if( m_colourPrimaries == 2 )
    {
      m_colourPrimaries = m_HdrMode == VVENC_HDR_PQ_BT2020 ? 9 : 1; //  bt2020(9) : bt709 (1)
    }
    if( m_matrixCoefficients == 2 )
    {
      m_matrixCoefficients = m_HdrMode == VVENC_HDR_PQ_BT2020 ? 9 : 1; // bt2020nc : bt709
    }
  }
  else if( m_HdrMode == VVENC_HDR_HLG || m_HdrMode == VVENC_HDR_HLG_BT2020 )
  {
    m_reshapeSignalType       = RESHAPE_SIGNAL_HLG;
    m_LMCSOffset              = 0;
    m_useSameChromaQPTables   = true;
    m_verCollocatedChromaFlag = true;

    VVEncCfg cBaseCfg;
    vvenc_cfg_default(&cBaseCfg);
    if( m_qpInValsCb == cBaseCfg.m_qpInValsCb )
    {
      std::vector<int>  qpVals = { 9, 23, 33, 42 };
      std::copy(qpVals.begin(), qpVals.end(), m_qpInValsCb);
    }
    if( m_qpOutValsCb == cBaseCfg.m_qpOutValsCb )
    {
      std::vector<int>  qpVals = { 9, 24, 33, 37 };
      std::copy(qpVals.begin(), qpVals.end(), m_qpOutValsCb);
    }

    // VUI and SEI options
    m_vuiParametersPresent = m_vuiParametersPresent != 0 ? 1:0; // enable vui only if not explicitly disabled
    m_colourDescriptionPresent = true;                            // enable colour_primaries, transfer_characteristics and matrix_coefficients in vui

    if( m_colourPrimaries == 2 )
    {
      m_colourPrimaries = m_HdrMode == VVENC_HDR_HLG_BT2020 ? 9 : 1; //  bt2020(9) : bt709 (1)
    }

    if( m_matrixCoefficients == 2 )
    {
      m_matrixCoefficients = m_HdrMode == VVENC_HDR_HLG_BT2020 ? 9 : 1; // bt2020nc : bt709
    }

    if( m_transferCharacteristics == 2 )
    {
      m_transferCharacteristics = m_HdrMode == VVENC_HDR_HLG_BT2020 ? 14 : 1; // bt2020-10 : bt709
    }

    if( m_preferredTransferCharacteristics < 0 )
    {
      m_preferredTransferCharacteristics = 18; // ARIB STD-B67 (HLG)
    }
  }

  if( m_preferredTransferCharacteristics < 0 )
  {
    m_preferredTransferCharacteristics = 0;
  }

  if( m_AccessUnitDelimiter < 0 )
  {
    m_AccessUnitDelimiter = 0;
  }

  if ( m_vuiParametersPresent < 0 )
  {
    m_vuiParametersPresent = 0;
  }

  if ( m_hrdParametersPresent < 0 )
  {
    m_hrdParametersPresent = 0;
  }

  switch ( m_conformanceWindowMode)
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
        msg( VVENC_ERROR, "Warning: Conformance window enabled, but all conformance window parameters set to zero\n");
      }
      if ((m_aiPad[1] != 0) || (m_aiPad[0]!=0))
      {
        msg( VVENC_ERROR, "Warning: Conformance window enabled, padding parameters will be ignored\n");
      }
      m_aiPad[1] = m_aiPad[0] = 0;
      break;
    }
  }
    m_PadSourceWidth  = m_SourceWidth  + m_aiPad[0];
    m_PadSourceHeight = m_SourceHeight + m_aiPad[1];

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

  m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag = m_useSameChromaQPTables;

  if (m_useIdentityTableForNon420Chroma && m_internChromaFormat != VVENC_CHROMA_420)
  {
    m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag = true;
    memset(&m_qpInValsCb   ,0, sizeof(m_qpInValsCb));
    memset(&m_qpInValsCr   ,0, sizeof(m_qpInValsCr));
    memset(&m_qpInValsCbCr ,0, sizeof(m_qpInValsCbCr));
    memset(&m_qpOutValsCb  ,0, sizeof(m_qpOutValsCb));
    memset(&m_qpOutValsCr  ,0, sizeof(m_qpOutValsCr));
    memset(&m_qpOutValsCbCr,0, sizeof(m_qpOutValsCbCr));
  }

  m_chromaQpMappingTableParams.m_numQpTables = m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag? 1 : (m_JointCbCrMode ? 3 : 2);
  m_chromaQpMappingTableParams.m_deltaQpInValMinus1[0].resize(m_qpInValsCb.size());
  m_chromaQpMappingTableParams.m_deltaQpOutVal[0].resize(m_qpOutValsCb.size());
  m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[0] = (m_qpOutValsCb.size() > 1) ? (int)m_qpOutValsCb.size() - 2 : 0;
  m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] = (m_qpOutValsCb.size() > 1) ? -26 + m_qpInValsCb[0] : 0;
  for (int i = 0; i < m_qpInValsCb.size() - 1; i++)
  {
    m_chromaQpMappingTableParams.m_deltaQpInValMinus1[0][i] = m_qpInValsCb[i + 1] - m_qpInValsCb[i] - 1;
    m_chromaQpMappingTableParams.m_deltaQpOutVal[0][i] = m_qpOutValsCb[i + 1] - m_qpOutValsCb[i];
  }
  if (!m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag)
  {
    m_chromaQpMappingTableParams.m_deltaQpInValMinus1[1].resize(m_qpInValsCr.size());
    m_chromaQpMappingTableParams.m_deltaQpOutVal[1].resize(m_qpOutValsCr.size());
    m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[1] = (m_qpOutValsCr.size() > 1) ? (int)m_qpOutValsCr.size() - 2 : 0;
    m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] = (m_qpOutValsCr.size() > 1) ? -26 + m_qpInValsCr[0] : 0;
    for (int i = 0; i < m_qpInValsCr.size() - 1; i++)
    {
      m_chromaQpMappingTableParams.m_deltaQpInValMinus1[1][i] = m_qpInValsCr[i + 1] - m_qpInValsCr[i] - 1;
      m_chromaQpMappingTableParams.m_deltaQpOutVal[1][i] = m_qpOutValsCr[i + 1] - m_qpOutValsCr[i];
    }
    m_chromaQpMappingTableParams.m_deltaQpInValMinus1[2].resize(m_qpInValsCbCr.size());
    m_chromaQpMappingTableParams.m_deltaQpOutVal[2].resize(m_qpOutValsCbCr.size());
    m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[2] = (m_qpOutValsCbCr.size() > 1) ? (int)m_qpOutValsCbCr.size() - 2 : 0;
    m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] = (m_qpOutValsCbCr.size() > 1) ? -26 + m_qpInValsCbCr[0] : 0;
    for (int i = 0; i < m_qpInValsCbCr.size() - 1; i++)
    {
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

  m_reshapeCW.rspFps     = m_FrameRate;
  m_reshapeCW.rspPicSize = m_PadSourceWidth*m_PadSourceHeight;
  m_reshapeCW.rspFpsToIp = std::max(16, 16 * (int)(round((double)m_FrameRate /16.0)));
  m_reshapeCW.rspBaseQP  = m_QP;
  m_reshapeCW.updateCtrl = m_updateCtrl;
  m_reshapeCW.adpOption  = m_adpOption;
  m_reshapeCW.initialCW  = m_initialCW;

  confirmParameter( m_rprEnabledFlag < 0 || m_rprEnabledFlag > 2, "RPR must be either 0, 1 or 2" );

  if( m_rprEnabledFlag == 2 )
  {
    m_resChangeInClvsEnabled = true;
    m_craAPSreset            = true;
    m_rprRASLtoolSwitch      = true;
  }  
    
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

  //
  // do some check and set of parameters next
  //

  if (m_lumaReshapeEnable)
  {
    if (m_updateCtrl > 0 && m_adpOption > 2) { m_adpOption -= 2; }
  }

  if ( m_JointCbCrMode && (m_internChromaFormat == VVENC_CHROMA_400) )
  {
    m_JointCbCrMode = false;
  }

  if ( m_MCTF && m_QP < 17 )
  {
    msg( VVENC_WARNING, "disable MCTF for QP < 17\n");
    m_MCTF = 0;
  }
  if( m_MCTF )
  {
    if( m_MCTFFrames[0] == 0 && m_MCTFFrames[1] == 0 )
    {
      if( m_GOPSize == 32 )
      {
        m_MCTFFrames[0] = 8;
        m_MCTFFrames[1] = 16;
        m_MCTFFrames[2] = 32;

        m_MCTFStrengths[0] = 0.28125;     //  9/32
        m_MCTFStrengths[1] = 0.5625;      // 18/32
        m_MCTFStrengths[2] = 0.84375;     // 27/32
      }
      else if( m_GOPSize == 16 )
      {
        m_MCTFFrames[0] = 8;
        m_MCTFFrames[1] = 16;

        m_MCTFStrengths[0] = 0.4;     // ~12.75/32
        m_MCTFStrengths[1] = 0.8;     // ~25.50/32
      }
      else if( m_GOPSize == 8 )
      {
        m_MCTFFrames[0] = 8;
        m_MCTFStrengths[0] = 0.65625;     // 21/32
      }
    }
  }

  if ( m_usePerceptQPATempFiltISlice < 0 )
  {
    m_usePerceptQPATempFiltISlice = 0;
    if ( m_usePerceptQPA ) // auto mode for temp.filt.
    {
      m_usePerceptQPATempFiltISlice = ( m_RCTargetBitrate > 0 && m_RCNumPasses == 2 ? 2 : 1 );
    }
  }
  if ( m_usePerceptQPATempFiltISlice == 2
      && (m_QP <= 27 || m_QP > MAX_QP_PERCEPT_QPA || m_GOPSize <= 8 || m_IntraPeriod < 2 * m_GOPSize) )
  {
    m_usePerceptQPATempFiltISlice = 1; // disable temporal pumping reduction aspect
  }
  if ( m_usePerceptQPATempFiltISlice > 0
      && (m_MCTF == 0 || !m_usePerceptQPA) )
  {
    m_usePerceptQPATempFiltISlice = 0; // fully disable temporal filtering features
  }

  if (m_cuQpDeltaSubdiv < 0)
  {
    m_cuQpDeltaSubdiv = 0;
    if ( m_usePerceptQPA
        && m_QP <= MAX_QP_PERCEPT_QPA
        && m_CTUSize == 128
        && m_PadSourceWidth <= 2048
        && m_PadSourceHeight <= 1280 )
    {
      m_cuQpDeltaSubdiv = 2;
    }
  }
  if (m_sliceChromaQpOffsetPeriodicity < 0)
  {
    m_sliceChromaQpOffsetPeriodicity = 0;
    if ( m_usePerceptQPA && m_internChromaFormat != CHROMA_400 )
    {
      m_sliceChromaQpOffsetPeriodicity = 1;
    }
  }

  /* if this is an intra-only sequence, ie IntraPeriod=1, don't verify the GOP structure
   * This permits the ability to omit a GOP structure specification */
  if ( m_IntraPeriod == 1 && m_GOPList[0].m_POC == -1 )
  {
    m_GOPList[0] = vvencGOPEntry();
    m_GOPList[0].m_QPFactor = 1;
    m_GOPList[0].m_betaOffsetDiv2 = 0;
    m_GOPList[0].m_tcOffsetDiv2 = 0;
    m_GOPList[0].m_POC = 1;
    m_RPLList0[0] = vvencRPLEntry();
    m_RPLList1[0] = vvencRPLEntry();
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
        m_GOPList[i] = vvencGOPEntry();
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
        vvenc_GOPEntry_default(&m_GOPList[i]);
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
  }

  for (int i = 0; m_GOPList[i].m_POC != -1 && i < VVENC_MAX_GOP + 1; i++)
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
  int refList[VVENC_MAX_NUM_REF_PICS+1] = {0};
  bool isOK[VVENC_MAX_GOP];
  for(int i=0; i<VVENC_MAX_GOP; i++)
  {
    isOK[i]=false;
  }
  int numOK=0;

  int extraRPLs = 0;
  int numRefs   = 1;
  //start looping through frames in coding order until we can verify that the GOP structure is correct.
  while (!verifiedGOP && !errorGOP)
  {
    int curGOP = (checkGOP - 1) % m_GOPSize;
    int curPOC = ((checkGOP - 1) / m_GOPSize)*m_GOPSize * multipleFactor + m_RPLList0[curGOP].m_POC;
    if (m_RPLList0[curGOP].m_POC < 0 || m_RPLList1[curGOP].m_POC < 0)
    {
      msg(VVENC_WARNING, "\nError: found fewer Reference Picture Sets than GOPSize\n");
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

  m_maxTempLayer = 1;

  for(int i=0; i<m_GOPSize; i++)
  {
    if(m_GOPList[i].m_temporalId >= m_maxTempLayer)
    {
      m_maxTempLayer = m_GOPList[i].m_temporalId+1;
    }
  }
  for(int i=0; i< VVENC_MAX_TLAYER; i++)
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

  for(int i=0; i < VVENC_MAX_TLAYER-1; i++)
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
  if(m_maxNumReorderPics[VVENC_MAX_TLAYER-1] > m_maxDecPicBuffering[VVENC_MAX_TLAYER-1] - 1)
  {
    m_maxDecPicBuffering[VVENC_MAX_TLAYER-1] = m_maxNumReorderPics[VVENC_MAX_TLAYER-1] + 1;
  }

  if ( ! m_MMVD && m_allowDisFracMMVD )
  {
    msg( VVENC_WARNING, "MMVD disabled, thus disable AllowDisFracMMVD too\n" );
    m_allowDisFracMMVD = false;
  }

  //
  // finalize initialization
  //


  // coding structure
  m_numRPLList0 = 0;
  for ( int i = 0; i < VVENC_MAX_GOP; i++ )
  {
    if ( m_RPLList0[ i ].m_POC != -1 )
      m_numRPLList0++;
  }
  m_numRPLList1 = 0;
  for ( int i = 0; i < VVENC_MAX_GOP; i++ )
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

  m_confirmFailed = checkCfgParameter();
  return( m_confirmFailed );
}

bool VVEncCfg::checkCfgParameter( )
{
  // run base check first
  confirmParameter( m_profile == Profile::PROFILE_AUTO, "can not determin auto profile");
  confirmParameter( (m_profile != Profile::MAIN_10 && m_profile !=MAIN_10_STILL_PICTURE ), "unsupported profile. currently only supporting auto,main10,main10stillpicture");

  confirmParameter( m_level   == Level::LEVEL_AUTO, "can not determin level");

  confirmParameter( m_fastInterSearchMode<VVENC_FASTINTERSEARCH_AUTO || m_fastInterSearchMode>VVENC_FASTINTERSEARCH_MODE3, "Error: FastInterSearchMode parameter out of range" );
  confirmParameter( m_motionEstimationSearchMethod < 0 || m_motionEstimationSearchMethod >= VVENC_MESEARCH_NUMBER_OF_METHODS, "Error: FastSearch parameter out of range" );
  confirmParameter( m_internChromaFormat >= VVENC_NUM_CHROMA_FORMAT,                                                "Intern chroma format must be either 400, 420, 422 or 444" );

  switch ( m_conformanceWindowMode)
  {
  case 0:
      break;
  case 1:
      // automatic padding to minimum CU size
      confirmParameter( m_aiPad[0] % SPS::getWinUnitX(m_internChromaFormat) != 0, "Error: picture width is not an integer multiple of the specified chroma subsampling" );
      confirmParameter( m_aiPad[1] % SPS::getWinUnitY(m_internChromaFormat) != 0, "Error: picture height is not an integer multiple of the specified chroma subsampling" );
      break;
  case 2:
      break;
  case 3:
      // conformance
      if ((m_confWinLeft == 0) && (m_confWinRight == 0) && (m_confWinTop == 0) && (m_confWinBottom == 0))
      {
        msg( VVENC_ERROR, "Warning: Conformance window enabled, but all conformance window parameters set to zero\n");
      }
      if ((m_aiPad[1] != 0) || (m_aiPad[0]!=0))
      {
        msg( VVENC_ERROR, "Warning: Conformance window enabled, padding parameters will be ignored\n");
      }
      break;
  }

  confirmParameter(  m_colourPrimaries < 0 || m_colourPrimaries > 12,                 "colourPrimaries must be in range 0 <= x <= 12" );
  confirmParameter(  m_transferCharacteristics < 0 || m_transferCharacteristics > 18, "transferCharacteristics must be in range 0 <= x <= 18" );
  confirmParameter(  m_matrixCoefficients < 0 || m_matrixCoefficients > 14,           "matrixCoefficients must be in range 0 <= x <= 14" );


  confirmParameter(m_qpInValsCb.size() != m_qpOutValsCb.size(), "Chroma QP table for Cb is incomplete.");
  confirmParameter(m_qpInValsCr.size() != m_qpOutValsCr.size(), "Chroma QP table for Cr is incomplete.");
  confirmParameter(m_qpInValsCbCr.size() != m_qpOutValsCbCr.size(), "Chroma QP table for CbCr is incomplete.");

  if ( m_confirmFailed )
  {
    return m_confirmFailed;
  }

  int qpBdOffsetC = 6 * (m_internalBitDepth[CH_C] - 8);

  confirmParameter(m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] < -26 - qpBdOffsetC || m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] > 36, "qpTableStartMinus26[0] is out of valid range of -26 -qpBdOffsetC to 36, inclusive.");
  confirmParameter(m_qpInValsCb[0] != m_qpOutValsCb[0], "First qpInValCb value should be equal to first qpOutValCb value");
  for (size_t i = 0; i < m_qpInValsCb.size() - 1; i++)
  {
    confirmParameter(m_qpInValsCb[i] < -qpBdOffsetC || m_qpInValsCb[i] > MAX_QP, "Some entries cfg_qpInValCb are out of valid range of -qpBdOffsetC to 63, inclusive.");
    confirmParameter(m_qpOutValsCb[i] < -qpBdOffsetC || m_qpOutValsCb[i] > MAX_QP, "Some entries cfg_qpOutValCb are out of valid range of -qpBdOffsetC to 63, inclusive.");
  }
  if (!m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag)
  {
    confirmParameter(m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] < -26 - qpBdOffsetC || m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] > 36, "qpTableStartMinus26[1] is out of valid range of -26 -qpBdOffsetC to 36, inclusive.");
    confirmParameter(m_qpInValsCr[0] != m_qpOutValsCr[0], "First qpInValCr value should be equal to first qpOutValCr value");
    for (size_t i = 0; i < m_qpInValsCr.size() - 1; i++)
    {
      confirmParameter(m_qpInValsCr[i] < -qpBdOffsetC || m_qpInValsCr[i] > MAX_QP, "Some entries cfg_qpInValCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      confirmParameter(m_qpOutValsCr[i] < -qpBdOffsetC || m_qpOutValsCr[i] > MAX_QP, "Some entries cfg_qpOutValCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
    }
    confirmParameter(m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] < -26 - qpBdOffsetC || m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] > 36, "qpTableStartMinus26[2] is out of valid range of -26 -qpBdOffsetC to 36, inclusive.");
    confirmParameter(m_qpInValsCbCr[0] != m_qpInValsCbCr[0], "First qpInValCbCr value should be equal to first qpOutValCbCr value");
    for (size_t i = 0; i < m_qpInValsCbCr.size() - 1; i++)
    {
      confirmParameter(m_qpInValsCbCr[i] < -qpBdOffsetC || m_qpInValsCbCr[i] > MAX_QP, "Some entries cfg_qpInValCbCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      confirmParameter(m_qpOutValsCbCr[i] < -qpBdOffsetC || m_qpOutValsCbCr[i] > MAX_QP, "Some entries cfg_qpOutValCbCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
    }
  }

  //
  // do some check and set of parameters next
  //

  confirmParameter( m_AccessUnitDelimiter < 0,   "AccessUnitDelimiter must be >= 0" );
  confirmParameter( m_vuiParametersPresent < 0,  "vuiParametersPresent must be >= 0" );
  confirmParameter( m_hrdParametersPresent < 0,  "hrdParametersPresent must be >= 0" );

  if( m_DepQuantEnabled )
  {
    confirmParameter( !m_RDOQ || !m_useRDOQTS, "RDOQ and RDOQTS must be greater 0 if dependent quantization is enabled" );
    confirmParameter( m_SignDataHidingEnabled, "SignHideFlag must be equal to 0 if dependent quantization is enabled" );
  }

  confirmParameter( (m_MSBExtendedBitDepth[CH_L] < m_inputBitDepth[CH_L]), "MSB-extended bit depth for luma channel (--MSBExtendedBitDepth) must be greater than or equal to input bit depth for luma channel (--InputBitDepth)" );
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


  confirmParameter( (m_HdrMode != VVENC_HDR_OFF && m_internalBitDepth[CH_L] < 10 )     ,       "internalBitDepth must be at least 10 bit for HDR");
  confirmParameter( (m_HdrMode != VVENC_HDR_OFF && m_internChromaFormat != VVENC_CHROMA_420 ) ,"internChromaFormat must YCbCr 4:2:0 for HDR");
  confirmParameter( (m_contentLightLevel[0] < 0 || m_contentLightLevel[0] > 10000),  "max content light level must 0 <= cll <= 10000 ");
  confirmParameter( (m_contentLightLevel[1] < 0 || m_contentLightLevel[1] > 10000),  "max average content light level must 0 <= cll <= 10000 ");

  {
    bool outOfRGBRange = false;
    for( size_t i = 0; i < sizeof(m_masteringDisplay); i++ )
    {
      if( i < 8 && m_masteringDisplay[i] > 50000 )
      {
        outOfRGBRange = true; break;
      }
    }
    confirmParameter( outOfRGBRange,  "mastering display colour volume RGB values must be in range 0 <= RGB <= 50000");
  }

  confirmParameter( m_log2SaoOffsetScale[CH_L]   > (m_internalBitDepth[CH_L  ]<10?0:(m_internalBitDepth[CH_L  ]-10)), "SaoLumaOffsetBitShift must be in the range of 0 to InternalBitDepth-10, inclusive");
  confirmParameter( m_log2SaoOffsetScale[CH_C] > (m_internalBitDepth[CH_C]<10?0:(m_internalBitDepth[CH_C]-10)), "SaoChromaOffsetBitShift must be in the range of 0 to InternalBitDepthC-10, inclusive");

  confirmParameter( m_temporalSubsampleRatio < 1,                                               "Temporal subsample rate must be no less than 1" );
  confirmParameter( m_framesToBeEncoded < m_switchPOC,                                          "debug POC out of range" );

  confirmParameter( (m_IntraPeriod > 0 && m_IntraPeriod < m_GOPSize) || m_IntraPeriod == 0,     "Intra period must be more than GOP size, or -1 , not 0" );
  confirmParameter( m_InputQueueSize < m_GOPSize ,                                              "Input queue size must be greater or equal to gop size" );
  confirmParameter( m_MCTF && m_InputQueueSize < m_GOPSize + MCTF_ADD_QUEUE_DELAY ,             "Input queue size must be greater or equal to gop size + N frames for MCTF" );

  confirmParameter( m_DecodingRefreshType < 0 || m_DecodingRefreshType > 3,                     "Decoding Refresh Type must be comprised between 0 and 3 included" );
  confirmParameter( m_IntraPeriod > 0 && !(m_DecodingRefreshType==1 || m_DecodingRefreshType==2), "Only Decoding Refresh Type CRA for non low delay supported" );                  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
  confirmParameter( m_SegmentMode != VVENC_SEG_OFF && m_framesToBeEncoded < VVENC_MCTF_RANGE,  "When using segment parallel encoding more then 2 frames have to be encoded" );

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
  }
  confirmParameter( m_EDO && m_bLoopFilterDisable,             "no EDO support with LoopFilter disabled" );
  confirmParameter( m_EDO < 0 || m_EDO > 2,                    "EDO out of range [0..2]" );
  confirmParameter( m_TMVPModeId < 0 || m_TMVPModeId > 2,      "TMVPMode out of range [0..2]" );
  confirmParameter( m_AMVRspeed < 0 || m_AMVRspeed > 7,        "AMVR/IMV out of range [0..7]" );
  confirmParameter( m_Affine < 0 || m_Affine > 2,              "Affine out of range [0..2]" );
  confirmParameter( m_MMVD < 0 || m_MMVD > 4,                  "MMVD out of range [0..4]" );
  confirmParameter( m_SMVD < 0 || m_SMVD > 3,                  "SMVD out of range [0..3]" );
  confirmParameter( m_Geo  < 0 || m_Geo  > 3,                  "Geo out of range [0..3]" );
  confirmParameter( m_CIIP < 0 || m_CIIP > 3,                  "CIIP out of range [0..3]" );
  confirmParameter( m_SBT  < 0 || m_SBT  > 3,                  "SBT out of range [0..3]" );
  confirmParameter( m_LFNST< 0 || m_LFNST> 3,                  "LFNST out of range [0..3]" );
  confirmParameter( m_MCTF < 0 || m_MCTF > 2,                  "MCTF out of range [0..2]" );
  confirmParameter( m_ISP < 0 || m_ISP > 3,                    "ISP out of range [0..3]" );
  confirmParameter(m_TS < 0 || m_TS > 2,                       "TS out of range [0..2]" );
  confirmParameter(m_TSsize < 2 || m_TSsize > 5,               "TSsize out of range [2..5]" );
  confirmParameter(m_useBDPCM < 0 || m_useBDPCM > 2,           "BDPCM out of range [0..2]");
  confirmParameter(m_useBDPCM  && m_TS==0,                     "BDPCM cannot be used when transform skip is disabled" );
  confirmParameter(m_useBDPCM==1  && m_TS==2,                  "BDPCM cannot be permanently used when transform skip is auto" );
  confirmParameter(m_FastIntraTools <0 || m_FastIntraTools >2, "SpeedIntraTools out of range [0..2]");

  if( m_alf )
  {
    confirmParameter( m_maxNumAlfAlternativesChroma < 1 || m_maxNumAlfAlternativesChroma > VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA, std::string( std::string( "The maximum number of ALF Chroma filter alternatives must be in the range (1-" ) + std::to_string( MAX_NUM_ALF_ALTERNATIVES_CHROMA ) + std::string( ", inclusive)" ) ).c_str() );
  }

  confirmParameter( m_useFastMrg < 0 || m_useFastMrg > 2,   "FastMrg out of range [0..2]" );
  confirmParameter( m_useFastMIP < 0 || m_useFastMIP > 4,   "FastMIP out of range [0..4]" );
  confirmParameter( m_fastSubPel < 0 || m_fastSubPel > 1,   "FastSubPel out of range [0..1]" );

  confirmParameter( m_RCTargetBitrate == 0 && m_RCNumPasses != 1, "Only single pass encoding supported, when rate control is disabled" );
  confirmParameter( m_RCNumPasses < 1 || m_RCNumPasses > 2,       "Only one pass or two pass encoding supported" );

  confirmParameter(!((m_level==VVENC_LEVEL1)
    || (m_level==VVENC_LEVEL2) || (m_level==VVENC_LEVEL2_1)
    || (m_level==VVENC_LEVEL3) || (m_level==VVENC_LEVEL3_1)
    || (m_level==VVENC_LEVEL4) || (m_level==VVENC_LEVEL4_1)
    || (m_level==VVENC_LEVEL5) || (m_level==VVENC_LEVEL5_1) || (m_level==VVENC_LEVEL5_2)
    || (m_level==VVENC_LEVEL6) || (m_level==VVENC_LEVEL6_1) || (m_level==VVENC_LEVEL6_2) || (m_level==VVENC_LEVEL6_3)
    || (m_level==VVENC_LEVEL15_5)), "invalid level selected");
  confirmParameter(!((m_levelTier==VVENC_TIER_MAIN) || (m_levelTier==VVENC_TIER_HIGH)), "invalid tier selected");


  confirmParameter( m_chromaCbQpOffset < -12,           "Min. Chroma Cb QP Offset is -12" );
  confirmParameter( m_chromaCbQpOffset >  12,           "Max. Chroma Cb QP Offset is  12" );
  confirmParameter( m_chromaCrQpOffset < -12,           "Min. Chroma Cr QP Offset is -12" );
  confirmParameter( m_chromaCrQpOffset >  12,           "Max. Chroma Cr QP Offset is  12" );
  confirmParameter( m_chromaCbQpOffsetDualTree < -12,   "Min. Chroma Cb QP Offset for dual tree is -12" );
  confirmParameter( m_chromaCbQpOffsetDualTree >  12,   "Max. Chroma Cb QP Offset for dual tree is  12" );
  confirmParameter( m_chromaCrQpOffsetDualTree < -12,   "Min. Chroma Cr QP Offset for dual tree is -12" );
  confirmParameter( m_chromaCrQpOffsetDualTree >  12,   "Max. Chroma Cr QP Offset for dual tree is  12" );

  if ( m_JointCbCrMode )
  {
    confirmParameter( m_chromaCbCrQpOffset < -12, "Min. Joint Cb-Cr QP Offset is -12");
    confirmParameter( m_chromaCbCrQpOffset >  12, "Max. Joint Cb-Cr QP Offset is  12");
    confirmParameter( m_chromaCbCrQpOffsetDualTree < -12, "Min. Joint Cb-Cr QP Offset for dual tree is -12");
    confirmParameter( m_chromaCbCrQpOffsetDualTree >  12, "Max. Joint Cb-Cr QP Offset for dual tree is  12");
  }

  if (m_usePerceptQPA && m_dualITree && (m_internChromaFormat != CHROMA_400) && (m_chromaCbQpOffsetDualTree != 0 || m_chromaCrQpOffsetDualTree != 0 || m_chromaCbCrQpOffsetDualTree != 0))
  {
    msg(VVENC_WARNING, "***************************************************************************\n");
    msg(VVENC_WARNING, "** WARNING: chroma QPA on, ignoring nonzero dual-tree chroma QP offsets! **\n");
    msg(VVENC_WARNING, "***************************************************************************\n");
  }

  confirmParameter( m_usePerceptQPATempFiltISlice > 2,                                                    "PerceptQPATempFiltIPic out of range, must be 2 or less" );
  confirmParameter( m_usePerceptQPATempFiltISlice > 0 && m_MCTF == 0,                                     "PerceptQPATempFiltIPic must be turned off when MCTF is off" );

  confirmParameter( m_usePerceptQPA && (m_cuQpDeltaSubdiv > 2),                                     "MaxCuDQPSubdiv must be 2 or smaller when PerceptQPA is on" );
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

  confirmParameter( m_numThreads < 0,                                               "NumThreads out of range" );
  confirmParameter( m_ensureWppBitEqual < 0       || m_ensureWppBitEqual > 1,       "WppBitEqual out of range (0,1)");
  confirmParameter( m_useAMaxBT < 0               || m_useAMaxBT > 1,               "AMaxBT out of range (0,1)");
  confirmParameter( m_cabacInitPresent < 0        || m_cabacInitPresent > 1,        "CabacInitPresent out of range (0,1)");
  confirmParameter( m_alfTempPred < 0             || m_alfTempPred > 1,             "ALFTempPred out of range (0,1)");
  confirmParameter( m_saoEncodingRate < 0.0       || m_saoEncodingRate > 1.0,       "SaoEncodingRate out of range [0.0 .. 1.0]");
  confirmParameter( m_saoEncodingRateChroma < 0.0 || m_saoEncodingRateChroma > 1.0, "SaoEncodingRateChroma out of range [0.0 .. 1.0]");
  confirmParameter( m_maxParallelFrames < 0,                                        "MaxParallelFrames out of range" );

  confirmParameter( m_numThreads > 0 && m_ensureWppBitEqual == 0, "NumThreads > 0 requires WppBitEqual > 0");

  if( m_maxParallelFrames )
  {
    confirmParameter( m_numThreads == 0,       "For frame parallel processing NumThreads > 0 is required" );
    confirmParameter( m_useAMaxBT,             "Frame parallel processing: AMaxBT is not supported (must be disabled)" );
    confirmParameter( m_cabacInitPresent,      "Frame parallel processing: CabacInitPresent is not supported (must be disabled)" );
    confirmParameter( m_saoEncodingRate > 0.0, "Frame parallel processing: SaoEncodingRate is not supported (must be disabled)" );
    confirmParameter( m_alfTempPred,           "Frame parallel processing: ALFTempPred is not supported (must be disabled)" );
#if ENABLE_TRACING
    confirmParameter( !m_traceFile.empty() && m_maxParallelFrames > 1, "Tracing and frame parallel encoding not supported" );
#endif
    confirmParameter( m_maxParallelFrames > m_InputQueueSize, "Max parallel frames should be less than size of input queue" );
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


  confirmParameter( (m_hrdParametersPresent>0) && (0 == m_RCTargetBitrate),  "HrdParametersPresent requires RateControl enabled");
  confirmParameter( m_bufferingPeriodSEIEnabled && (m_hrdParametersPresent<1), "BufferingPeriodSEI requires HrdParametersPresent enabled");
  confirmParameter( m_pictureTimingSEIEnabled && (m_hrdParametersPresent<1),   "PictureTimingSEI requires HrdParametersPresent enabled");

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

  if ( m_IntraPeriod == 1 && m_GOPList[0].m_POC == -1 )
  {
  }
  else
  {
    confirmParameter( m_intraOnlyConstraintFlag, "IntraOnlyConstraintFlag cannot be 1 for inter sequences");
  }

  int multipleFactor = /*m_compositeRefEnabled ? 2 :*/ 1;
  bool verifiedGOP=false;
  bool errorGOP=false;
  int checkGOP=1;
  int refList[VVENC_MAX_NUM_REF_PICS+1] = {0};
  bool isOK[VVENC_MAX_GOP];
  for(int i=0; i<VVENC_MAX_GOP; i++)
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
      msg(VVENC_WARNING, "\nError: found fewer Reference Picture Sets than GOPSize\n");
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
            }
          }
          if (!found)
          {
            msg(VVENC_WARNING, "\nError: ref pic %d is not available for GOP frame %d\n", m_RPLList0[curGOP].m_deltaRefPics[i], curGOP + 1);
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
        int newRefs0 = 0;
        for (int i = 0; i< m_RPLList0[curGOP].m_numRefPics; i++)
        {
          int absPOC = curPOC - m_RPLList0[curGOP].m_deltaRefPics[i];
          if (absPOC >= 0)
          {
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
              newRefs1++;
            }
          }
          if (newRefs1 >= numPrefRefs1)
          {
            break;
          }
        }

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

  for(int i=0; i<m_GOPSize; i++)
  {
    confirmParameter(m_GOPList[i].m_sliceType!='B' && m_GOPList[i].m_sliceType!='P' && m_GOPList[i].m_sliceType!='I', "Slice type must be equal to B or P or I");
  }

  confirmParameter( m_MCTF > 2 || m_MCTF < 0, "MCTF out of range" );

  if( m_MCTF )
  {
    if( m_MCTFFrames[0] == 0 )
    {
      msg( VVENC_WARNING, "no MCTF frames selected, MCTF will be inactive!\n");
    }

    confirmParameter( m_MCTFFrames.size() != m_MCTFStrengths.size(), "MCTFFrames and MCTFStrengths do not match");
  }

  if( m_fastForwardToPOC != -1 )
  {
    if( m_cabacInitPresent )  { msg( VVENC_WARNING, "WARNING usage of FastForwardToPOC and CabacInitPresent might cause different behaviour\n\n" ); }
    if( m_alf )               { msg( VVENC_WARNING, "WARNING usage of FastForwardToPOC and ALF might cause different behaviour\n\n" ); }
  }


  return( m_confirmFailed );
}

VVENC_DECL int vvenc_initDefault( VVEncCfg *c, int width, int height, int framerate, int targetbitrate = 0, int qp = 32, vvencPresetMode preset = vvencPresetMode::VVENC_MEDIUM )
{
  int iRet = 0;
  c->m_SourceWidth         = width;                    // luminance width of input picture
  c->m_SourceHeight        = height;                   // luminance height of input picture

  c->m_FrameRate           = framerate;                // temporal rate (fps)
  c->m_TicksPerSecond      = 90000;                    // ticks per second e.g. 90000 for dts generation

  c->m_inputBitDepth[0]    = 8;                        // input bitdepth
  c->m_internalBitDepth[0] = 10;                       // internal bitdepth

  c->m_QP                  = qp;                       // quantization parameter 0-63
  c->m_usePerceptQPA       = true;                     // perceptual QP adaptation (false: off, true: on)

  c->m_RCTargetBitrate     = targetbitrate;            // target bitrate in bps

  c->m_numThreads          = -1;                       // number of worker threads (-1: auto, 0: off, else set worker threads)

  iRet = vvenc_initPreset(c, preset );
  return iRet;
}

VVENC_DECL int vvenc_initPreset( VVEncCfg *c, vvencPresetMode preset )
{
  memset(&c->m_qpInValsCb ,0, sizeof(c->m_qpInValsCb));
  memset(&c->m_qpOutValsCb,0, sizeof(c->m_qpOutValsCb));

  std::vector<int>  qpVals = { 17, 22, 34, 42 };
  std::copy(qpVals.begin(), qpVals.end(), c->m_qpInValsCb);

  qpVals = { 17, 23, 35, 39 };
  std::copy(qpVals.begin(), qpVals.end(), c->m_qpOutValsCb);

  // basic settings
  c->m_intraQPOffset                 = -3;
  c->m_lambdaFromQPEnable            = true;
  c->m_MaxCodingDepth                = 5;
  c->m_log2DiffMaxMinCodingBlockSize = 5;
  c->m_bUseASR                       = true;
  c->m_bUseHADME                     = true;
  c->m_useRDOQTS                     = true;
  c->m_useSelectiveRDOQ              = false;
  c->m_fastQtBtEnc                   = true;
  c->m_fastInterSearchMode           = VVENC_FASTINTERSEARCH_MODE1;
  c->m_motionEstimationSearchMethod  = VVENC_MESEARCH_DIAMOND_FAST;
  c->m_SearchRange                   = 384;
  c->m_minSearchWindow               = 96;
  c->m_maxNumMergeCand               = 6;
  c->m_TSsize                        = 3;
  c->m_reshapeSignalType             = 0;
  c->m_updateCtrl                    = 0;
  c->m_LMCSOffset                    = 6;
  c->m_RDOQ                          = 1;
  c->m_SignDataHidingEnabled         = 0;
  c->m_useFastLCTU                   = 1;

  // partitioning
  c->m_CTUSize                       = 128;
  c->m_dualITree                     = 1;
  c->m_MinQT[ 0 ]                    = 8;
  c->m_MinQT[ 1 ]                    = 8;
  c->m_MinQT[ 2 ]                    = 4;
  c->m_maxMTTDepth                   = 3;
  c->m_maxMTTDepthI                  = 3;
  c->m_maxMTTDepthIChroma            = 3;

  // disable tools
  c->m_Affine                        = 0;
  c->m_alf                           = 0;
  c->m_allowDisFracMMVD              = 0;
  c->m_useBDPCM                      = 0;
  c->m_BDOF                          = 0;
  c->m_ccalf                         = 0;
  c->m_useChromaTS                   = 0;
  c->m_CIIP                          = 0;
  c->m_DepQuantEnabled               = 0;
  c->m_DMVR                          = 0;
  c->m_EDO                           = 0;
  c->m_Geo                           = 0;
  c->m_AMVRspeed                     = 0;
  c->m_ISP                           = 0;
  c->m_JointCbCrMode                 = 0;
  c->m_LFNST                         = 0;
  c->m_LMChroma                      = 0;
  c->m_lumaReshapeEnable             = 0;
  c->m_MCTF                          = 0;
  c->m_MIP                           = 0;
  c->m_MMVD                          = 0;
  c->m_MRL                           = 0;
  c->m_MTS                           = 0;
  c->m_MTSImplicit                   = 0;
  c->m_PROF                          = 0;
  c->m_bUseSAO                       = 0;
  c->m_SbTMVP                        = 0;
  c->m_SBT                           = 0;
  c->m_SMVD                          = 0;
  c->m_TMVPModeId                    = 0;
  c->m_TS                            = 0;
  c->m_useNonLinearAlfChroma         = 0;
  c->m_useNonLinearAlfLuma           = 0;

  // enable speedups
  c->m_qtbttSpeedUp                  = 2;
  c->m_contentBasedFastQtbt          = 1;
  c->m_usePbIntraFast                = 1;
  c->m_useFastMrg                    = 2;
  c->m_useFastMIP                    = 4;
  c->m_fastLocalDualTreeMode         = 1;
  c->m_fastSubPel                    = 1;
  c->m_FastIntraTools                = 0;

  switch( preset )
  {
    case vvencPresetMode::VVENC_FIRSTPASS:
    case vvencPresetMode::VVENC_FASTER:
      // CTUSize64 QT44MTT00
      c->m_CTUSize                   = 64;
      c->m_MinQT[ 0 ]                = 4;
      c->m_MinQT[ 1 ]                = 4;
      c->m_MinQT[ 2 ]                = 2;
      c->m_maxMTTDepth               = 0;
      c->m_maxMTTDepthI              = 0;
      c->m_maxMTTDepthIChroma        = 0;

      c->m_RDOQ                      = 2;
      c->m_SignDataHidingEnabled     = 1;

      c->m_useBDPCM                  = 2;
      c->m_DMVR                      = 1;
      c->m_LMChroma                  = 1;
      c->m_MTSImplicit               = 1;
      c->m_bUseSAO                   = 1;
      c->m_TMVPModeId                = 1;
      c->m_TS                        = 2;
      break;

    case vvencPresetMode::VVENC_FAST:
      // CTUSize64 QT44MTT10
      c->m_CTUSize                   = 64;
      c->m_MinQT[ 0 ]                = 4;
      c->m_MinQT[ 1 ]                = 4;
      c->m_MinQT[ 2 ]                = 2;
      c->m_maxMTTDepth               = 0;
      c->m_maxMTTDepthI              = 1;
      c->m_maxMTTDepthIChroma        = 1;

      c->m_RDOQ                      = 2;
      c->m_SignDataHidingEnabled     = 1;

      c->m_Affine                    = 2;
      c->m_alf                       = 1;
      c->m_ccalf                     = 1;
      c->m_useBDPCM                  = 2;
      c->m_BDOF                      = 1;
      c->m_DMVR                      = 1;
      c->m_AMVRspeed                 = 5;
      c->m_LMChroma                  = 1;
      c->m_MCTF                      = 2;
      c->m_MTSImplicit               = 1;
      c->m_PROF                      = 1;
      c->m_bUseSAO                   = 1;
      c->m_TMVPModeId                = 1;
      c->m_TS                        = 2;
      break;

    case vvencPresetMode::VVENC_MEDIUM:
      // CTUSize128 QT44MTT21
      c->m_CTUSize                   = 128;
      c->m_MinQT[ 0 ]                = 8;
      c->m_MinQT[ 1 ]                = 8;
      c->m_MinQT[ 2 ]                = 4;
      c->m_maxMTTDepth               = 1;
      c->m_maxMTTDepthI              = 2;
      c->m_maxMTTDepthIChroma        = 2;

      c->m_Affine                    = 2;
      c->m_alf                       = 1;
      c->m_allowDisFracMMVD          = 1;
      c->m_useBDPCM                  = 2;
      c->m_BDOF                      = 1;
      c->m_ccalf                     = 1;
      c->m_DepQuantEnabled           = 1;
      c->m_DMVR                      = 1;
      c->m_EDO                       = 2;
      c->m_Geo                       = 3;
      c->m_AMVRspeed                 = 5;
      c->m_ISP                       = 3;
      c->m_JointCbCrMode             = 1;
      c->m_LFNST                     = 1;
      c->m_LMChroma                  = 1;
      c->m_lumaReshapeEnable         = 1;
      c->m_MCTF                      = 2;
      c->m_MIP                       = 1;
      c->m_MMVD                      = 3;
      c->m_MRL                       = 1;
      c->m_MTSImplicit               = 1;
      c->m_PROF                      = 1;
      c->m_bUseSAO                   = 1;
      c->m_SbTMVP                    = 1;
      c->m_SMVD                      = 3;
      c->m_TMVPModeId                = 1;
      c->m_TS                        = 2;

      c->m_FastIntraTools            = 1;
      break;

    case vvencPresetMode::VVENC_SLOW:
      // CTUSize128 QT44MTT32
      c->m_CTUSize                   = 128;
      c->m_MinQT[ 0 ]                = 8;
      c->m_MinQT[ 1 ]                = 8;
      c->m_MinQT[ 2 ]                = 4;
      c->m_maxMTTDepth               = 2;
      c->m_maxMTTDepthI              = 3;
      c->m_maxMTTDepthIChroma        = 3;

      c->m_Affine                    = 2;
      c->m_alf                       = 1;
      c->m_allowDisFracMMVD          = 1;
      c->m_useBDPCM                  = 2;
      c->m_BDOF                      = 1;
      c->m_ccalf                     = 1;
      c->m_DepQuantEnabled           = 1;
      c->m_CIIP                      = 1;
      c->m_DMVR                      = 1;
      c->m_EDO                       = 2;
      c->m_Geo                       = 1;
      c->m_AMVRspeed                 = 1;
      c->m_ISP                       = 3;
      c->m_JointCbCrMode             = 1;
      c->m_LFNST                     = 1;
      c->m_LMChroma                  = 1;
      c->m_lumaReshapeEnable         = 1;
      c->m_MCTF                      = 2;
      c->m_MIP                       = 1;
      c->m_MMVD                      = 3;
      c->m_MRL                       = 1;
      c->m_MTSImplicit               = 1;
      c->m_PROF                      = 1;
      c->m_bUseSAO                   = 1;
      c->m_SbTMVP                    = 1;
      c->m_SBT                       = 1;
      c->m_SMVD                      = 3;
      c->m_TMVPModeId                = 1;
      c->m_TS                        = 2;

      c->m_contentBasedFastQtbt      = 0;
      break;

    case vvencPresetMode::VVENC_SLOWER:

      c->m_motionEstimationSearchMethod = VVENC_MESEARCH_DIAMOND;

      // CTUSize128 QT44MTT33
      c->m_CTUSize                   = 128;
      c->m_MinQT[ 0 ]                = 8;
      c->m_MinQT[ 1 ]                = 8;
      c->m_MinQT[ 2 ]                = 4;
      c->m_maxMTTDepth               = 3;
      c->m_maxMTTDepthI              = 3;
      c->m_maxMTTDepthIChroma        = 3;

      c->m_Affine                    = 1;
      c->m_alf                       = 1;
      c->m_allowDisFracMMVD          = 1;
      c->m_useBDPCM                  = 2;
      c->m_BDOF                      = 1;
      c->m_ccalf                     = 1;
      c->m_DepQuantEnabled           = 1;
      c->m_CIIP                      = 1;
      c->m_DMVR                      = 1;
      c->m_EDO                       = 2;
      c->m_Geo                       = 1;
      c->m_AMVRspeed                 = 1;
      c->m_ISP                       = 1;
      c->m_JointCbCrMode             = 1;
      c->m_LFNST                     = 1;
      c->m_LMChroma                  = 1;
      c->m_lumaReshapeEnable         = 1;
      c->m_MCTF                      = 2;
      c->m_MIP                       = 1;
      c->m_MMVD                      = 1;
      c->m_MRL                       = 1;
      c->m_MTS                       = 1;
      c->m_MTSImplicit               = 0;
      c->m_PROF                      = 1;
      c->m_bUseSAO                   = 1;
      c->m_SbTMVP                    = 1;
      c->m_SBT                       = 1;
      c->m_SMVD                      = 1;
      c->m_TMVPModeId                = 1;
      c->m_TS                        = 2;
      c->m_useNonLinearAlfChroma     = 1;
      c->m_useNonLinearAlfLuma       = 1;

      c->m_qtbttSpeedUp              = 1;
      c->m_contentBasedFastQtbt      = 0;
      c->m_useFastMrg                = 1;
      c->m_useFastMIP                = 0;
      c->m_fastSubPel                = 0;
      break;

    case vvencPresetMode::VVENC_TOOLTEST:
      // CTUSize128 QT44MTT21
      c->m_CTUSize                   = 128;
      c->m_MinQT[ 0 ]                = 8;
      c->m_MinQT[ 1 ]                = 8;
      c->m_MinQT[ 2 ]                = 4;
      c->m_maxMTTDepth               = 1;
      c->m_maxMTTDepthI              = 2;
      c->m_maxMTTDepthIChroma        = 2;

      c->m_Affine                    = 2;
      c->m_alf                       = 1;
      c->m_allowDisFracMMVD          = 1;
      c->m_useBDPCM                  = 1;
      c->m_BDOF                      = 1;
      c->m_ccalf                     = 1;
      c->m_DepQuantEnabled           = 1;
      c->m_CIIP                      = 3;
      c->m_DMVR                      = 1;
      c->m_EDO                       = 1;
      c->m_Geo                       = 2;
      c->m_AMVRspeed                 = 3;
      c->m_ISP                       = 2;
      c->m_JointCbCrMode             = 1;
      c->m_LFNST                     = 1;
      c->m_LMChroma                  = 1;
      c->m_lumaReshapeEnable         = 1;
      c->m_MCTF                      = 2;
      c->m_MIP                       = 1;
      c->m_MMVD                      = 2;
      c->m_MRL                       = 1;
      c->m_MTS                       = 1;
      c->m_PROF                      = 1;
      c->m_bUseSAO                   = 1;
      c->m_SbTMVP                    = 1;
      c->m_SBT                       = 2;
      c->m_SMVD                      = 3;
      c->m_TMVPModeId                = 1;
      c->m_TS                        = 1;
      c->m_useNonLinearAlfChroma     = 1;
      c->m_useNonLinearAlfLuma       = 1;
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
    case VVENC_MAIN_10                              : cT = "main_10"; break;
    case VVENC_MAIN_10_STILL_PICTURE                : cT = "main_10_still_picture"; break;
    case VVENC_MAIN_10_444                          : cT = "main_10_444"; break;
    case VVENC_MAIN_10_444_STILL_PICTURE            : cT = "main_10_444_still_picture"; break;
    case VVENC_MULTILAYER_MAIN_10                   : cT = "multilayer_main_10"; break;
    case VVENC_MULTILAYER_MAIN_10_STILL_PICTURE     : cT = "multilayer_main_10_still_picture"; break;
    case VVENC_MULTILAYER_MAIN_10_444               : cT = "multilayer_main_10_444"; break;
    case VVENC_MULTILAYER_MAIN_10_444_STILL_PICTURE : cT = "multilayer_main_10_444_still_picture"; break;
    case VVENC_PROFILE_AUTO                         : cT = "auto"; break;
    default                                            : cT = "unknown"; break;
  }
  return cT;
}

static inline std::string getLevelStr( int level )
{
  std::string cT;
  switch( level )
  {
    case VVENC_LEVEL_AUTO: cT = "auto";    break;
    case VVENC_LEVEL1    : cT = "1";       break;
    case VVENC_LEVEL2    : cT = "2";       break;
    case VVENC_LEVEL2_1  : cT = "2.1";     break;
    case VVENC_LEVEL3    : cT = "3";       break;
    case VVENC_LEVEL3_1  : cT = "3.1";     break;
    case VVENC_LEVEL4    : cT = "4";       break;
    case VVENC_LEVEL4_1  : cT = "4.1";     break;
    case VVENC_LEVEL5    : cT = "5";       break;
    case VVENC_LEVEL5_1  : cT = "5.1";     break;
    case VVENC_LEVEL5_2  : cT = "5.2";     break;
    case VVENC_LEVEL6    : cT = "6";       break;
    case VVENC_LEVEL6_1  : cT = "6.1";     break;
    case VVENC_LEVEL6_2  : cT = "6.2";     break;
    case VVENC_LEVEL6_3  : cT = "6.3";     break;
    case VVENC_LEVEL15_5 : cT = "15.5";    break;
    default               : cT = "unknown"; break;
  }
  return cT;
}

static inline std::string getCostFunctionStr( int cost )
{
  std::string cT;
  switch( cost )
  {
    case VVENC_COST_STANDARD_LOSSY               : cT = "Lossy coding"; break;
    case VVENC_COST_SEQUENCE_LEVEL_LOSSLESS      : cT = "Sequence level lossless coding"; break;
    case VVENC_COST_LOSSLESS_CODING              : cT = "Lossless coding"; break;
    case VVENC_COST_MIXED_LOSSLESS_LOSSY_CODING  : cT = "Mixed lossless lossy coding"; break;
    default                                : cT = "Unknown"; break;
  }
  return cT;
}

static inline std::string getDynamicRangeStr( int dynamicRange )
{
  std::string cT;
  switch( dynamicRange )
  {
    case VVENC_HDR_OFF            : cT = "SDR"; break;
    case VVENC_HDR_PQ             : cT = "HDR10/PQ"; break;
    case VVENC_HDR_HLG            : cT = "HDR HLG"; break;
    case VVENC_HDR_PQ_BT2020      : cT = "HDR10/PQ BT.2020"; break;
    case VVENC_HDR_HLG_BT2020     : cT = "HDR HLG BT.2020"; break;
    case VVENC_HDR_USER_DEFINED   : cT = "HDR user defined"; break;
    default                          : cT = "unknown"; break;
  }
  return cT;
}

static inline std::string getMasteringDisplayStr(  std::vector<uint32_t> md )
{
  std::stringstream css;

  if(  md.size() != 10 )
  {
    return "unspecified";
  }

  css << "G(" << md[0] << "," << md[1] << ")";
  css << "B(" << md[2] << "," << md[3] << ")";
  css << "R(" << md[4] << "," << md[5] << ")";
  css << "WP("<< md[6] << "," << md[7] << ")";
  css << "L(" << md[8] << "," << md[9] << ")";

  css << " (= nits: ";
  css << "G(" << md[0]/50000.0 << "," << md[1]/50000.0 << ")";
  css << "B(" << md[2]/50000.0 << "," << md[3]/50000.0 << ")";
  css << "R(" << md[4]/50000.0 << "," << md[5]/50000.0 << ")";
  css << "WP("<< md[6]/50000.0 << "," << md[7]/50000.0 << ")";
  css << "L(" << md[8]/10000.0 << "," << md[9]/10000.0 << ")";
  css << ")";
  return css.str();
}

static inline std::string getContentLightLevel(  std::vector<uint32_t> cll )
{
  std::stringstream css;

  if(  cll.size() != 2 )
  {
    return "unspecified";
  }

  css << cll[0] << "," << cll[1] << " (cll,fall)";
  return css.str();
}

std::string VVEncCfg::getConfigAsString( MsgLevel eMsgLevel ) const
{
  std::stringstream css;

  if( eMsgLevel >= VVENC_DETAILS )
  {
  css << "Real     Format                        : " << m_PadSourceWidth - m_confWinLeft - m_confWinRight << "x" << m_PadSourceHeight - m_confWinTop - m_confWinBottom << " " <<
                                                        (double)m_FrameRate / m_temporalSubsampleRatio << "Hz " << getDynamicRangeStr(m_HdrMode) << "\n";
  css << "Internal Format                        : " << m_PadSourceWidth << "x" << m_PadSourceHeight << " " <<  (double)m_FrameRate / m_temporalSubsampleRatio << "Hz "  << getDynamicRangeStr(m_HdrMode) << "\n";
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

  if( !m_masteringDisplay.empty() )
  {
    css << "Mastering display color volume         : " << getMasteringDisplayStr( m_masteringDisplay ) << "\n";
  }
  if( !m_contentLightLevel.empty() )
  {
    css << "Content light level                    : " << getContentLightLevel( m_contentLightLevel ) << "\n";
  }
  css << "\n";
  }

  if( eMsgLevel >= VERBOSE )
  {
  // verbose output
  css << "CODING TOOL CFG: ";
  css << "CTU" << m_CTUSize << " QT" << Log2( m_CTUSize / m_MinQT[0] ) << Log2( m_CTUSize / m_MinQT[1] ) << "BTT" << m_maxMTTDepthI << m_maxMTTDepth << " ";
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
  css << "FastIntraTools:" << m_FastIntraTools << " ";
  css << "FastLocalDualTree:" << m_fastLocalDualTreeMode << " ";
  css << "FastSubPel:" << m_fastSubPel << " ";
  css << "QtbttExtraFast:" << m_qtbttSpeedUp << " ";

  css << "\nRATE CONTROL CFG: ";
  css << "RateControl:" << ( m_RCTargetBitrate > 0 ) << " ";
  if ( m_RCTargetBitrate > 0 )
  {
    css << "Passes:" << m_RCNumPasses << " ";
    css << "TargetBitrate:" << m_RCTargetBitrate << " ";
    css << "RCInitialQP:" << m_RCInitialQP << " ";
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


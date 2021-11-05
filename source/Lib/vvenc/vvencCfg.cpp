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


/** \file     vvencCfg.cpp
    \brief    encoder configuration class
*/

#include "vvenc/vvencCfg.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Slice.h"
#include "CommonLib/ProfileLevelTier.h"

#include <math.h>
#include <thread>

VVENC_NAMESPACE_BEGIN

static bool checkCfgParameter( vvenc_config *cfg );
static void checkCfgPicPartitioningParameter( vvenc_config *c );
static std::string vvenc_cfgString;

static int vvenc_getQpValsSize( int QpVals[] )
{
  int size=0;
  for ( int i = 0; i < VVENC_MAX_QP_VALS_CHROMA; i++ )
  {
    if( QpVals[i] > 0) size++;
    else break;
  }

  if( size == 0)
  {
    size = 1;  // at least first value must be set to 0
  }

  return size;
}

static std::vector<int> vvenc_getQpValsAsVec( int QpVals[] )
{
  std::vector<int> QpValsVec;
  for ( int i = 0; i < VVENC_MAX_QP_VALS_CHROMA; i++ )
  {
    if( QpVals[i] > 0) QpValsVec.push_back( QpVals[i] );
    else break;
  }
  if( QpValsVec.empty() )
  {
    QpValsVec = { 0 };
  }
  return QpValsVec;
}


static void vvenc_checkCharArrayStr( char array[], int size )
{
  bool resetArray = false;
  if ( 0 == strcmp(array, "empty") )
  {
    resetArray = true;
  }
  else if ( 0 == strcmp(array, "undef") )
  {
    resetArray = true;
  }
  else if ( 0 == strcmp(array, "''") )
  {
    resetArray = true;
  }
  else if ( 0 == strcmp(array, "\"\"") )
  {
    resetArray = true;
  }
  else if ( 0 == strcmp(array, "[]") )
  {
    resetArray = true;
  }

  if( resetArray )
  {
    for( int i = 0; i < size; i++ ) memset(&array[i],0, sizeof(char));
    array[0] = '\0';
  }
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

static inline std::string vvenc_getMasteringDisplayStr( unsigned int md[10]  )
{
  std::stringstream css;

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

static inline std::string vvenc_getContentLightLevelStr( unsigned int cll[2] )
{
  std::stringstream css;

  css << cll[0] << "," << cll[1] << " (cll,fall)";
  return css.str();
}

VVENC_DECL void vvenc_GOPEntry_default(vvencGOPEntry *GOPEntry )
{
  GOPEntry->m_POC                       = -1;
  GOPEntry->m_QPOffset                  = 0;
  GOPEntry->m_QPOffsetModelOffset       = 0.0;
  GOPEntry->m_QPOffsetModelScale        = 0.0;
  GOPEntry->m_CbQPoffset                = 0;
  GOPEntry->m_CrQPoffset                = 0;
  GOPEntry->m_QPFactor                  = 0.0;
  GOPEntry->m_tcOffsetDiv2              = 0;
  GOPEntry->m_betaOffsetDiv2            = 0;
  GOPEntry->m_CbTcOffsetDiv2            = 0;
  GOPEntry->m_CbBetaOffsetDiv2          = 0;
  GOPEntry->m_CrTcOffsetDiv2            = 0;
  GOPEntry->m_CrBetaOffsetDiv2          = 0;
  GOPEntry->m_temporalId                = 0;
  GOPEntry->m_refPic                    = false;
  GOPEntry->m_sliceType                 = 'P';
  memset( GOPEntry->m_numRefPicsActive, 0, sizeof( GOPEntry->m_numRefPicsActive ) );
  memset( GOPEntry->m_numRefPics, 0, sizeof( GOPEntry->m_numRefPics ) );
  memset( GOPEntry->m_deltaRefPics, 0, sizeof( GOPEntry->m_deltaRefPics ) );
  GOPEntry->m_isEncoded                 = false;
  GOPEntry->m_ltrp_in_slice_header_flag = false;
}

VVENC_DECL void vvenc_WCGChromaQPControl_default(vvencWCGChromaQPControl *WCGChromaQPControl )
{
  WCGChromaQPControl->enabled         = false;  ///< Enabled flag (0:default)
  WCGChromaQPControl->chromaCbQpScale = 1.0;    ///< Chroma Cb QP Scale (1.0:default)
  WCGChromaQPControl->chromaCrQpScale = 1.0;    ///< Chroma Cr QP Scale (1.0:default)
  WCGChromaQPControl->chromaQpScale   = 0.0;    ///< Chroma QP Scale (0.0:default)
  WCGChromaQPControl->chromaQpOffset  = 0-0;    ///< Chroma QP Offset (0.0:default)
}

VVENC_DECL void vvenc_ChromaQpMappingTableParams_default(vvencChromaQpMappingTableParams *p )
{
  p->m_numQpTables                                          = 0;
  p->m_qpBdOffset                                           = 12;
  p->m_sameCQPTableForAllChromaFlag                         = true;

  memset( p->m_qpTableStartMinus26, 0, sizeof( p->m_qpTableStartMinus26 ) );
  memset( p->m_numPtsInCQPTableMinus1, 0, sizeof( p->m_numPtsInCQPTableMinus1 ) );
  memset( p->m_deltaQpInValMinus1, 0, sizeof( p->m_deltaQpInValMinus1 ) );
  memset( p->m_deltaQpOutVal, 0, sizeof( p->m_deltaQpOutVal ) );
}

VVENC_DECL void vvenc_RPLEntry_default(vvencRPLEntry *RPLEntry )
{
  RPLEntry->m_POC                              = -1;
  RPLEntry->m_temporalId                       = 0;
  RPLEntry->m_refPic                           = false;
  RPLEntry->m_ltrp_in_slice_header_flag        = false;
  RPLEntry->m_numRefPicsActive                 = 0;
  RPLEntry->m_sliceType                        ='P';
  RPLEntry->m_numRefPics                       = 0;

  memset(&RPLEntry->m_deltaRefPics,0, sizeof(RPLEntry->m_deltaRefPics));
}

VVENC_DECL void vvenc_ReshapeCW_default(vvencReshapeCW *reshapeCW )
{
  memset( reshapeCW->binCW, 0, sizeof( reshapeCW->binCW ) );
  reshapeCW->updateCtrl = 0;
  reshapeCW->adpOption  = 0;
  reshapeCW->initialCW  = 0;
  reshapeCW->rspPicSize = 0;
  reshapeCW->rspFps     = 0;
  reshapeCW->rspBaseQP  = 0;
  reshapeCW->rspTid     = 0;
  reshapeCW->rspFpsToIp = 0;
}

VVENC_DECL void vvenc_vvencMCTF_default(vvencMCTF *vvencMCTF )
{
  vvencMCTF->MCTF = 0;
  vvencMCTF->MCTFSpeed = 0;
  vvencMCTF->MCTFFutureReference = true;
  vvencMCTF->MCTFNumLeadFrames = 0;
  vvencMCTF->MCTFNumTrailFrames = 0;
  vvencMCTF->numFrames = 0;
  vvencMCTF->numStrength = 0;
  memset( vvencMCTF->MCTFFrames, 0, sizeof( vvencMCTF->MCTFFrames ) );
  memset( vvencMCTF->MCTFStrengths, 0, sizeof( vvencMCTF->MCTFStrengths ) );
}

VVENC_DECL void vvenc_config_default(vvenc_config *c )
{
  int i = 0;

  //basic params
  c->m_configDone                              = false;
  c->m_confirmFailed                           = false;         ///< state variable

  c->m_verbosity                               = VVENC_VERBOSE;       ///< encoder verbosity
  c->m_framesToBeEncoded                       = 0;             ///< number of encoded frames

  c->m_FrameRate                               = 0;             ///< source frame-rates (Hz)
  c->m_FrameSkip                               = 0;             ///< number of skipped frames from the beginning
  c->m_SourceWidth                             = 0;             ///< source width in pixel
  c->m_SourceHeight                            = 0;             ///< source height in pixel (when interlaced = field height)
  c->m_TicksPerSecond                          = 90000;         ///< ticks per second e.g. 90000 for dts generation (1..27000000)

  c->m_profile                                 = vvencProfile::VVENC_PROFILE_AUTO;
  c->m_levelTier                               = vvencTier::VVENC_TIER_MAIN ;
  c->m_level                                   = vvencLevel::VVENC_LEVEL_AUTO;

  c->m_IntraPeriod                             = 0;             ///< period of I-slice (random access period)
  c->m_IntraPeriodSec                          = 1;             ///< period of I-slice in seconds (random access period)
  c->m_DecodingRefreshType                     = VVENC_DRT_CRA;       ///< random access type
  c->m_GOPSize                                 = 32;            ///< GOP size of hierarchical structure

  c->m_QP                                      = 32;            ///< QP value of key-picture (integer)
  c->m_usePerceptQPA                           = false;         ///< Mode of perceptually motivated input-adaptive QP modification, abbrev. perceptual QP adaptation (QPA).

  c->m_RCTargetBitrate                         = 0;
  c->m_RCNumPasses                             = -1;
  c->m_RCPass                                  = -1;

  c->m_SegmentMode                             = VVENC_SEG_OFF;

  c->m_numThreads                              = 0;             ///< number of worker threads

  c->m_inputBitDepth[0]                        =8;             ///< bit-depth of input file
  c->m_inputBitDepth[1]                        =0;
  c->m_internalBitDepth[0]                     =10;                                 ///< bit-depth codec operates at (input/output files will be converted)
  c->m_internalBitDepth[1]                     =0;

  c->m_HdrMode                                 = VVENC_HDR_OFF;

  // expert options
  c->m_conformanceWindowMode                   = 1;
  c->m_confWinLeft                             = 0;
  c->m_confWinRight                            = 0;
  c->m_confWinTop                              = 0;
  c->m_confWinBottom                           = 0;

  c->m_temporalSubsampleRatio                  = 1;                                     ///< temporal subsample ratio, 2 means code every two frames

  c->m_PadSourceWidth                          = 0;                                     ///< source width in pixel
  c->m_PadSourceHeight                         = 0;                                     ///< source height in pixel (when interlaced = field height)

  memset(&c->m_aiPad,0, sizeof(c->m_aiPad));                                    ///< number of padded pixels for width and height
  c->m_enablePictureHeaderInSliceHeader        = true;
  c->m_AccessUnitDelimiter                     = -1;                                    ///< add Access Unit Delimiter NAL units, default: auto (only enable if needed by dependent options)

  c->m_printMSEBasedSequencePSNR               = false;
  c->m_printHexPsnr                            = false;
  c->m_printFrameMSE                           = false;
  c->m_printSequenceMSE                        = false;
  c->m_cabacZeroWordPaddingEnabled             = true;

  c->m_subProfile                              = 0;
  c->m_bitDepthConstraintValue                 = 10;
  c->m_intraOnlyConstraintFlag                 = false;

  c->m_InputQueueSize                          = 0;                                     ///< Size of frame input queue
  c->m_rewriteParamSets                        = true;                                 ///< Flag to enable rewriting of parameter sets at random access points
  c->m_idrRefParamList                         = false;                                 ///< indicates if reference picture list syntax elements are present in slice headers of IDR pictures
  for( i = 0; i < VVENC_MAX_GOP; i++ )
  {
    vvenc_RPLEntry_default( &c->m_RPLList0[i]);                                         ///< the RPL entries from the config file
    vvenc_RPLEntry_default( &c->m_RPLList1[i]);                                         ///< the RPL entries from the config file
    vvenc_GOPEntry_default( &c->m_GOPList[i]);                                          ///< the coding structure entries from the config file
  }
  memset(&c->m_maxDecPicBuffering,0, sizeof(c->m_maxDecPicBuffering));                ///< total number of pictures in the decoded picture buffer
  memset(&c->m_maxNumReorderPics,0, sizeof(c->m_maxNumReorderPics));                  ///< total number of reorder pictures
  c->m_maxTempLayer                            = 0;                                     ///< Max temporal layer
  c->m_numRPLList0                             = 0;
  c->m_numRPLList1                             = 0;

  c->m_useSameChromaQPTables                   = true;

  vvenc_ChromaQpMappingTableParams_default( &c->m_chromaQpMappingTableParams );
  c->m_intraQPOffset                           = 0;                                     ///< QP offset for intra slice (integer)
  c->m_lambdaFromQPEnable                      = false;                                 ///< enable flag for QP:lambda fix

  for( i = 0; i < VVENC_MAX_TLAYER; i++ )
  {
    c->m_adLambdaModifier[i] = 1.0;                                                     ///< Lambda modifier array for each temporal layer
    c->m_adIntraLambdaModifier[i] = 0.0;                                                ///< Lambda modifier for Intra pictures, one for each temporal layer. If size>temporalLayer, then use [temporalLayer], else if size>0, use [size()-1], else use m_adLambdaModifier.
  }

  c->m_dIntraQpFactor                          = -1.0;                                  ///< Intra Q Factor. If negative, use a default equation: 0.57*(1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? (GopSize-1)/2 : GopSize-1) ))

  memset(&c->m_qpInValsCb    ,0, sizeof(c->m_qpInValsCb));                      ///< qp input values used to derive the chroma QP mapping table
  memset(&c->m_qpInValsCr    ,0, sizeof(c->m_qpInValsCr));                      ///< qp input values used to derive the chroma QP mapping table
  memset(&c->m_qpInValsCbCr  ,0, sizeof(c->m_qpInValsCbCr));                      ///< qp input values used to derive the chroma QP mapping table
  memset(&c->m_qpOutValsCb   ,0, sizeof(c->m_qpOutValsCb));                      ///< qp output values used to derive the chroma QP mapping table
  memset(&c->m_qpOutValsCr   ,0, sizeof(c->m_qpOutValsCr));                      ///< qp output values used to derive the chroma QP mapping table
  memset(&c->m_qpOutValsCbCr ,0, sizeof(c->m_qpOutValsCbCr));                      ///< qp output values used to derive the chroma QP mapping table

  c->m_qpInValsCb[0] = 17;  c->m_qpInValsCb[1] = 22;  c->m_qpInValsCb[2] = 34;  c->m_qpInValsCb[3] = 42;
  c->m_qpOutValsCb[0] = 17; c->m_qpOutValsCb[1] = 23; c->m_qpOutValsCb[2] = 35; c->m_qpOutValsCb[3] = 39;

  c->m_cuQpDeltaSubdiv                         = -1;                                    ///< Maximum subdiv for CU luma Qp adjustment (0:default)
  c->m_cuChromaQpOffsetSubdiv                  = -1;                                    ///< If negative, then do not apply chroma qp offsets.
  c->m_chromaCbQpOffset                        = 0;                                     ///< Chroma Cb QP Offset (0:default)
  c->m_chromaCrQpOffset                        = 0;                                     ///< Chroma Cr QP Offset (0:default)
  c->m_chromaCbQpOffsetDualTree                = 0;                                     ///< Chroma Cb QP Offset for dual tree (overwrite m_chromaCbQpOffset for dual tree)
  c->m_chromaCrQpOffsetDualTree                = 0;                                     ///< Chroma Cr QP Offset for dual tree (overwrite m_chromaCrQpOffset for dual tree)
  c->m_chromaCbCrQpOffset                      = -1;                                    ///< QP Offset for joint Cb-Cr mode
  c->m_chromaCbCrQpOffsetDualTree              = 0;                                     ///< QP Offset for joint Cb-Cr mode (overwrite m_chromaCbCrQpOffset for dual tree)
  c->m_sliceChromaQpOffsetPeriodicity          = -1;                                    ///< Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.

  memset(&c->m_sliceChromaQpOffsetIntraOrPeriodic,0, sizeof(c->m_sliceChromaQpOffsetIntraOrPeriodic));            ///< Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.

  c->m_usePerceptQPATempFiltISlice             = -1;                                    ///< Flag indicating if temporal high-pass filtering in visual activity calculation in QPA should (true) or shouldn't (false) be applied for I-slices

  c->m_lumaLevelToDeltaQPEnabled               = false;
  vvenc_WCGChromaQPControl_default(&c->m_wcgChromaQpControl );

  c->m_internChromaFormat                      = VVENC_NUM_CHROMA_FORMAT;
  c->m_useIdentityTableForNon420Chroma         = true;
  c->m_outputBitDepth[0]      = c->m_outputBitDepth[1]      = 0;                              ///< bit-depth of output file
  c->m_MSBExtendedBitDepth[0] = c->m_MSBExtendedBitDepth[1] = 0;                    ///< bit-depth of input samples after MSB extension
  c->m_costMode                                = VVENC_COST_STANDARD_LOSSY;                   ///< Cost mode to use

  c->m_decodedPictureHashSEIType               = VVENC_HASHTYPE_NONE;                         ///< Checksum mode for decoded picture hash SEI message
  c->m_bufferingPeriodSEIEnabled               = false;
  c->m_pictureTimingSEIEnabled                 = false;
  c->m_decodingUnitInfoSEIEnabled              = false;

  c->m_entropyCodingSyncEnabled                = false;
  c->m_entryPointsPresent                      = true;

  c->m_CTUSize                                 = 128;
  c->m_MinQT[0] = c->m_MinQT[1] = 8;                                            ///< 0: I slice luma; 1: P/B slice; 2: I slice chroma
  c->m_MinQT[2] = 4;
  c->m_maxMTTDepth                             = 3;
  c->m_maxMTTDepthI                            = 3;
  c->m_maxMTTDepthIChroma                      = 3;

  c->m_maxBT[0]=32;  c->m_maxBT[1]=128;  c->m_maxBT[2]=64;
  c->m_maxTT[0]=32;  c->m_maxTT[1]=64;  c->m_maxTT[2]=32;
  c->m_dualITree                               = true;
  c->m_MaxCodingDepth                          = 0;                                     ///< max. total CU depth - includes depth of transform-block structure
  c->m_log2DiffMaxMinCodingBlockSize           = 0;                                     ///< difference between largest and smallest CU depth
  c->m_log2MaxTbSize                           = 6;
  c->m_log2MinCodingBlockSize                  = 2;

  c->m_bUseASR                                 = false;                                 ///< flag for using adaptive motion search range
  c->m_bUseHADME                               = true;                                  ///< flag for using HAD in sub-pel ME
  c->m_RDOQ                                    = 1;                                     ///< flag for using RD optimized quantization
  c->m_useRDOQTS                               = true;                                  ///< flag for using RD optimized quantization for transform skip
  c->m_useSelectiveRDOQ                        = false;                                 ///< flag for using selective RDOQ

  c->m_JointCbCrMode                           = false;
  c->m_cabacInitPresent                        = -1;
  c->m_useFastLCTU                             = false;
  c->m_usePbIntraFast                          = 0;
  c->m_useFastMrg                              = 0;
  c->m_useAMaxBT                               = -1;
  c->m_fastQtBtEnc                             = true;
  c->m_contentBasedFastQtbt                    = false;
  c->m_fastInterSearchMode                     = VVENC_FASTINTERSEARCH_AUTO;
  c->m_useEarlyCU                              = 0;
  c->m_useFastDecisionForMerge                 = true;

  c->m_bDisableIntraCUsInInterSlices           = false;
  c->m_bUseConstrainedIntraPred                = false;
  c->m_bFastUDIUseMPMEnabled                   = true;
  c->m_bFastMEForGenBLowDelayEnabled           = true;

  c->m_MTSImplicit                             = false;
  c->m_TMVPModeId                              = 1;
  c->m_DepQuantEnabled                         = true;
  c->m_SignDataHidingEnabled                   = false;

  c->m_MIP                                     = false;
  c->m_useFastMIP                              = 0;

  c->m_maxNumMergeCand                         = 5;
  c->m_maxNumAffineMergeCand                   = 5;
  c->m_Geo                                     = 0;
  c->m_maxNumGeoCand                           = 5;
  c->m_FastIntraTools                          = 0;
  c->m_IntraEstDecBit                          = 1;

  c->m_RCInitialQP                             = 0;
  c->m_RCForceIntraQP                          = false;

  c->m_motionEstimationSearchMethod            = VVENC_MESEARCH_DIAMOND;
  c->m_motionEstimationSearchMethodSCC         = 0;
  c->m_bRestrictMESampling                     = false;
  c->m_SearchRange                             = 96;
  c->m_bipredSearchRange                       = 4;
  c->m_minSearchWindow                         = 8;
  c->m_bClipForBiPredMeEnabled                 = false;
  c->m_bFastMEAssumingSmootherMVEnabled        = true;
  c->m_bIntegerET                              = false;
  c->m_fastSubPel                              = 0;
  c->m_SMVD                                    = 0;
  c->m_AMVRspeed                               = 0;
  c->m_LMChroma                                = false;
  c->m_horCollocatedChromaFlag                 = true;
  c->m_verCollocatedChromaFlag                 = false;
  c->m_MRL                                     = true;
  c->m_BDOF                                    = false;
  c->m_DMVR                                    = false;
  c->m_EDO                                     = 0;
  c->m_lumaReshapeEnable                       = 0;
  c->m_reshapeSignalType                       = 0;
  c->m_updateCtrl                              = 0;
  c->m_adpOption                               = 0;
  c->m_initialCW                               = 0;
  c->m_LMCSOffset                              = 0;
  vvenc_ReshapeCW_default( &c->m_reshapeCW );
  c->m_Affine                                  = 0;
  c->m_PROF                                    = false;
  c->m_AffineType                              = true;
  c->m_MMVD                                    = 0;
  c->m_MmvdDisNum                              = 6;
  c->m_allowDisFracMMVD                        = false;
  c->m_CIIP                                    = 0;
  c->m_SbTMVP                                  = false;
  c->m_SBT                                     = 0;
  c->m_LFNST                                   = 0;
  c->m_MTS                                     = 0;
  c->m_MTSIntraMaxCand                         = 3;
  c->m_ISP                                     = 0;
  c->m_TS                                      = 0;
  c->m_TSsize                                  = 3;
  c->m_useChromaTS                             = 0;
  c->m_useBDPCM                                = 0;

  c->m_rprEnabledFlag                          = -1;
  c->m_resChangeInClvsEnabled                  = false;
  c->m_craAPSreset                             = false;
  c->m_rprRASLtoolSwitch                       = false;

  c->m_IBCMode                                 = 0;
  c->m_IBCFastMethod                           = 1;

  c->m_BCW                                     = 0;

  c->m_FIMMode                                 = 0;
  c->m_FastInferMerge                          = 0;

  c->m_bLoopFilterDisable                      = false;
  c->m_loopFilterOffsetInPPS                   = true;

  memset(&c->m_loopFilterBetaOffsetDiv2,0, sizeof(c->m_loopFilterBetaOffsetDiv2));
  memset(&c->m_loopFilterTcOffsetDiv2,0, sizeof(c->m_loopFilterTcOffsetDiv2));

  c->m_deblockingFilterMetric                  = 0;

  c->m_bDisableLFCrossTileBoundaryFlag         = false;
  c->m_bDisableLFCrossSliceBoundaryFlag        = false;

  c->m_bUseSAO                                 = true;
  c->m_saoEncodingRate                         = -1.0;
  c->m_saoEncodingRateChroma                   = -1.0;
  c->m_log2SaoOffsetScale[0]=c->m_log2SaoOffsetScale[1] = 0;
  c->m_saoOffsetBitShift[0]=c->m_saoOffsetBitShift[1] = 0;

  c->m_decodingParameterSetEnabled             = false;
  c->m_vuiParametersPresent                    = -1;
  c->m_hrdParametersPresent                    = -1;
  c->m_aspectRatioInfoPresent                  = false;
  c->m_aspectRatioIdc                          = 0;
  c->m_sarWidth                                = 0;
  c->m_sarHeight                               = 0;
  c->m_colourDescriptionPresent                = false;
  c->m_colourPrimaries                         = 2;
  c->m_transferCharacteristics                 = 2;
  c->m_matrixCoefficients                      = 2;
  c->m_chromaLocInfoPresent                    = false;
  c->m_chromaSampleLocTypeTopField             = 0;
  c->m_chromaSampleLocTypeBottomField          = 0;
  c->m_chromaSampleLocType                     = 0;
  c->m_overscanInfoPresent                     = false;
  c->m_overscanAppropriateFlag                 = false;
  c->m_videoSignalTypePresent                  = false;
  c->m_videoFullRangeFlag                      = false;

  memset(&c->m_masteringDisplay,0, sizeof(c->m_masteringDisplay));
  memset(&c->m_contentLightLevel,0, sizeof(c->m_contentLightLevel));
  c->m_preferredTransferCharacteristics        = -1;

  c->m_alf                                     = false;
  c->m_alfSpeed                                = 0;
  c->m_useNonLinearAlfLuma                     = true;
  c->m_useNonLinearAlfChroma                   = true;
  c->m_maxNumAlfAlternativesChroma             = VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA;
  c->m_ccalf                                   = false;
  c->m_ccalfQpThreshold                        = 37;
  c->m_alfTempPred                             = -1;

  vvenc_vvencMCTF_default( &c->m_vvencMCTF );

  c->m_quantThresholdVal                       = -1;
  c->m_qtbttSpeedUp                            = 1;
#if 1//QTBTT_SPEED3
  c->m_qtbttSpeedUpMode                        = 0;
#endif 
#if FASTTT_TH
  c->m_fastTTSplit                             = 0;
#endif

  c->m_fastLocalDualTreeMode                   = 0;

  c->m_maxParallelFrames                       = -1;
  c->m_ensureWppBitEqual                       = -1;

  c->m_picPartitionFlag                        = false;
  memset( c->m_tileColumnWidth, 0, sizeof(c->m_tileColumnWidth) );
  memset( c->m_tileRowHeight,   0, sizeof(c->m_tileRowHeight) );
  c->m_numExpTileCols                          = 1;
  c->m_numExpTileRows                          = 1;
  c->m_numTileCols                             = 1;
  c->m_numTileRows                             = 1;
  c->m_numSlicesInPic                          = 1;
  
  memset( c->m_summaryOutFilename    , '\0', sizeof(c->m_summaryOutFilename) );
  memset( c->m_summaryPicFilenameBase, '\0', sizeof(c->m_summaryPicFilenameBase) );
  c->m_summaryVerboseness                      = 0;

  memset( c->m_decodeBitstreams[0], '\0', sizeof(c->m_decodeBitstreams[0]) );
  memset( c->m_decodeBitstreams[1], '\0', sizeof(c->m_decodeBitstreams[1]) );

  c->m_switchPOC                               = -1;
  c->m_switchDQP                               = 0;
  c->m_fastForwardToPOC                        = -1;
  c->m_stopAfterFFtoPOC                        = false;
  c->m_bs2ModPOCAndType                        = false;
  c->m_forceDecodeBitstream1                   = false;

  c->m_listTracingChannels                     = false;
  memset( c->m_traceRule, '\0', sizeof(c->m_traceRule) );
  memset( c->m_traceFile, '\0', sizeof(c->m_traceFile) );

  c->m_numIntraModesFullRD = -1;
  c->m_reduceIntraChromaModesFullRD = false;


  // init default preset
  vvenc_init_preset( c, vvencPresetMode::VVENC_MEDIUM );
}

static bool vvenc_confirmParameter ( vvenc_config *c, bool bflag, const char* message )
{
  if ( ! bflag )
    return false;
  vvenc::msg( VVENC_ERROR, "Parameter Check Error: %s\n", message );
  c->m_confirmFailed = true;
  return true;
}

VVENC_DECL bool vvenc_init_config_parameter( vvenc_config *c )
{
  c->m_confirmFailed = false;

  // check for valid base parameter
  vvenc_confirmParameter( c,  (c->m_SourceWidth <= 0 || c->m_SourceHeight <= 0), "Error: input resolution not set");

  vvenc_confirmParameter( c, c->m_inputBitDepth[0] < 8 || c->m_inputBitDepth[0] > 16,                    "InputBitDepth must be at least 8" );
  vvenc_confirmParameter( c, c->m_inputBitDepth[0] != 8 && c->m_inputBitDepth[0] != 10,                  "Input bitdepth must be 8 or 10 bit" );
  vvenc_confirmParameter( c, c->m_internalBitDepth[0] != 8 && c->m_internalBitDepth[0] != 10,                  "Internal bitdepth must be 8 or 10 bit" );

  vvenc_confirmParameter( c, c->m_FrameRate <= 0,                                                        "Frame rate must be greater than 0" );
  vvenc_confirmParameter( c, c->m_TicksPerSecond <= 0 || c->m_TicksPerSecond > 27000000,                 "TicksPerSecond must be in range from 1 to 27000000" );

  int temporalRate   = c->m_FrameRate;
  int temporalScale  = 1;

  switch( c->m_FrameRate )
  {
  case 23: temporalRate = 24000; temporalScale = 1001; break;
  case 29: temporalRate = 30000; temporalScale = 1001; break;
  case 59: temporalRate = 60000; temporalScale = 1001; break;
  default: break;
  }

  vvenc_confirmParameter( c, (c->m_TicksPerSecond < 90000) && (c->m_TicksPerSecond*temporalScale)%temporalRate, "TicksPerSecond should be a multiple of FrameRate/Framscale" );

  vvenc_confirmParameter( c, c->m_numThreads < -1 || c->m_numThreads > 256,              "Number of threads out of range (-1 <= t <= 256)");

  vvenc_confirmParameter( c, c->m_IntraPeriod < -1,                                            "IDR period (in frames) must be >= -1");
  vvenc_confirmParameter( c, c->m_IntraPeriodSec < 0,                                          "IDR period (in seconds) must be >= 0");

  vvenc_confirmParameter( c, c->m_GOPSize < 1 || c->m_GOPSize > 64,                                                        "GOP Size must be between 1 and 64" );
  vvenc_confirmParameter( c, c->m_GOPSize > 1 &&  c->m_GOPSize % 2,                                                        "GOP Size must be a multiple of 2" );
  vvenc_confirmParameter( c, c->m_GOPList[0].m_POC == -1 && c->m_GOPSize != 1 && c->m_GOPSize != 16 && c->m_GOPSize != 32, "GOP list auto config only supported GOP sizes: 1, 16, 32" );

  vvenc_confirmParameter( c, c->m_QP < 0 || c->m_QP > vvenc::MAX_QP,                                                 "QP exceeds supported range (0 to 63)" );

  vvenc_confirmParameter( c, c->m_RCTargetBitrate < 0 || c->m_RCTargetBitrate > 800000000,                           "TargetBitrate must be between 0 - 800000000" );

  if( 0 == c->m_RCTargetBitrate )
   {
     vvenc_confirmParameter( c, c->m_hrdParametersPresent > 0,          "hrdParameters present requires rate control" );
     vvenc_confirmParameter( c, c->m_bufferingPeriodSEIEnabled,         "bufferingPeriod SEI enabled requires rate control" );
     vvenc_confirmParameter( c, c->m_pictureTimingSEIEnabled,           "pictureTiming SEI enabled requires rate control" );
   }

  vvenc_confirmParameter( c, c->m_HdrMode < VVENC_HDR_OFF || c->m_HdrMode > VVENC_HDR_USER_DEFINED,  "HdrMode must be in the range 0 - 5" );

  vvenc_confirmParameter( c, c->m_verbosity < VVENC_SILENT || c->m_verbosity > VVENC_DETAILS, "verbosity is out of range[0..6]" );

  vvenc_confirmParameter( c,  (c->m_numIntraModesFullRD < -1 || c->m_numIntraModesFullRD == 0 || c->m_numIntraModesFullRD > 3), "Error: NumIntraModesFullRD must be -1 or between 1 and 3");

  
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
    const int maxBitDepth= std::max(c->m_internalBitDepth[0], c->m_internalBitDepth[c->m_internChromaFormat==vvencChromaFormat::VVENC_CHROMA_400 ? 0 : 1]);

    if (c->m_internChromaFormat==vvencChromaFormat::VVENC_CHROMA_400 || c->m_internChromaFormat==vvencChromaFormat::VVENC_CHROMA_420)
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
    c->m_level = vvenc::LevelTierFeatures::getLevelForInput( c->m_SourceWidth, c->m_SourceHeight, c->m_levelTier, temporalRate, temporalScale, c->m_RCTargetBitrate );
  }

  if ( c->m_InputQueueSize <= 0 )
  {
    c->m_InputQueueSize = c->m_GOPSize;

    if ( c->m_vvencMCTF.MCTF )
    {
      c->m_InputQueueSize += vvenc::MCTF_ADD_QUEUE_DELAY;
    }
  }

  if( !c->m_configDone )
  {
    if( c->m_temporalSubsampleRatio )
    {
      int framesSubsampled = ( c->m_framesToBeEncoded + c->m_temporalSubsampleRatio - 1 ) / c->m_temporalSubsampleRatio;
      if( c->m_framesToBeEncoded != framesSubsampled )
      {
        c->m_framesToBeEncoded = framesSubsampled;
      }
    }
  }

  c->m_maxBT[0] = std::min( c->m_CTUSize, c->m_maxBT[0] );
  c->m_maxBT[1] = std::min( c->m_CTUSize, c->m_maxBT[1] );
  c->m_maxBT[2] = std::min( c->m_CTUSize, c->m_maxBT[2] );

  c->m_maxTT[0] = std::min( c->m_CTUSize, c->m_maxTT[0] );
  c->m_maxTT[1] = std::min( c->m_CTUSize, c->m_maxTT[1] );
  c->m_maxTT[2] = std::min( c->m_CTUSize, c->m_maxTT[2] );

  // set MCTF Lead/Trail frames
  if( c->m_SegmentMode != VVENC_SEG_OFF )
  {
    if( c->m_vvencMCTF.MCTF )
    {
      switch( c->m_SegmentMode )
      {
        case VVENC_SEG_FIRST:
          c->m_vvencMCTF.MCTFNumLeadFrames  = 0;
          c->m_vvencMCTF.MCTFNumTrailFrames = c->m_vvencMCTF.MCTFNumTrailFrames == 0 ? VVENC_MCTF_RANGE : c->m_vvencMCTF.MCTFNumTrailFrames;
          break;
        case VVENC_SEG_MID:
          c->m_vvencMCTF.MCTFNumLeadFrames  = VVENC_MCTF_RANGE;
          c->m_vvencMCTF.MCTFNumTrailFrames = c->m_vvencMCTF.MCTFNumTrailFrames == 0 ? VVENC_MCTF_RANGE : c->m_vvencMCTF.MCTFNumTrailFrames;
          break;
        case VVENC_SEG_LAST:
          c->m_vvencMCTF.MCTFNumLeadFrames  = c->m_vvencMCTF.MCTFNumLeadFrames == 0 ? VVENC_MCTF_RANGE : c->m_vvencMCTF.MCTFNumTrailFrames;
          c->m_vvencMCTF.MCTFNumTrailFrames = 0;
          break;
        default:
          break;
      }
    }
  }

  // rate control
  if( c->m_RCNumPasses < 0 )
  {
    if( c->m_RCPass > 0 )
      c->m_RCNumPasses = 2;
    else
      c->m_RCNumPasses = c->m_RCTargetBitrate > 0 ? 2 : 1;
  }

  // threading
  if( c->m_numThreads < 0 )
  {
    const int numCores = std::thread::hardware_concurrency();
    c->m_numThreads = c->m_SourceWidth >= 1280 && c->m_SourceHeight >= 720 ? 8 : 4;
    c->m_numThreads = std::min( c->m_numThreads, numCores );
  }
  if( c->m_ensureWppBitEqual < 0 )       c->m_ensureWppBitEqual     = c->m_numThreads ? 1   : 0   ;
  if( c->m_useAMaxBT < 0 )               c->m_useAMaxBT             = c->m_numThreads ? 0   : 1   ;
  if( c->m_cabacInitPresent < 0 )        c->m_cabacInitPresent      = c->m_numThreads ? 0   : 1   ;
  if( c->m_alfTempPred < 0 )             c->m_alfTempPred           = c->m_numThreads ? 0   : 1   ;
  if( c->m_saoEncodingRate < 0.0 )       c->m_saoEncodingRate       = c->m_numThreads ? 0.0 : 0.75;
  if( c->m_saoEncodingRateChroma < 0.0 ) c->m_saoEncodingRateChroma = c->m_numThreads ? 0.0 : 0.5 ;
  if( c->m_maxParallelFrames < 0 )
  {
    c->m_maxParallelFrames = std::min( c->m_numThreads, 4 );
    if( c->m_RCTargetBitrate > 0
        && c->m_RCNumPasses == 1
        && c->m_maxParallelFrames > 2 )
    {
      c->m_maxParallelFrames = 2;
    }
  }

  // quantization threshold
  if( c->m_quantThresholdVal < 0 )
  {
    c->m_quantThresholdVal = 8;
  }

  // MCTF
  c->m_vvencMCTF.MCTFNumLeadFrames  = std::min( c->m_vvencMCTF.MCTFNumLeadFrames,  VVENC_MCTF_RANGE );
  c->m_vvencMCTF.MCTFNumTrailFrames = std::min( c->m_vvencMCTF.MCTFNumTrailFrames, VVENC_MCTF_RANGE );

  /* rules for input, output and internal bitdepths as per help text */
  if (c->m_MSBExtendedBitDepth[0  ] == 0)
    c->m_MSBExtendedBitDepth[0  ] = c->m_inputBitDepth      [0  ];
  if (c->m_MSBExtendedBitDepth[1] == 0)
    c->m_MSBExtendedBitDepth[1] = c->m_MSBExtendedBitDepth[0  ];
  if (c->m_internalBitDepth   [0  ] == 0)
    c->m_internalBitDepth   [0  ] = c->m_MSBExtendedBitDepth[0  ];
  if (c->m_internalBitDepth   [1] == 0)
    c->m_internalBitDepth   [1] = c->m_internalBitDepth   [0  ];
  if (c->m_inputBitDepth      [1] == 0)
    c->m_inputBitDepth      [1] = c->m_inputBitDepth      [0  ];
  if (c->m_outputBitDepth     [0  ] == 0)
    c->m_outputBitDepth     [0  ] = c->m_internalBitDepth   [0  ];
  if (c->m_outputBitDepth     [1] == 0)
    c->m_outputBitDepth     [1] = c->m_outputBitDepth     [0  ];

  if( c->m_fastInterSearchMode  == VVENC_FASTINTERSEARCH_AUTO )
  {
    c->m_fastInterSearchMode = VVENC_FASTINTERSEARCH_MODE1;
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
    c->m_reshapeSignalType       = vvenc::RESHAPE_SIGNAL_PQ;
    c->m_LMCSOffset              = 1;
    c->m_useSameChromaQPTables   = false;
    c->m_verCollocatedChromaFlag = true;

    vvenc_config cBaseCfg;
    vvenc_config_default(&cBaseCfg);
    // if qpInVal/qpOutVal are set to default value and not overwritten by user defined values, overwrite them with PQ/HLG specifc qp values
    if( memcmp( c->m_qpInValsCb, cBaseCfg.m_qpInValsCb, sizeof( c->m_qpInValsCb ) ) == 0 )
    {
      memset(&c->m_qpInValsCb,0, sizeof(c->m_qpInValsCb));
      std::vector<int>  qpInVals = { 13,20,36,38,43,54 };
      std::copy(qpInVals.begin(), qpInVals.end(), c->m_qpInValsCb);
    }
    if( memcmp( c->m_qpOutValsCb, cBaseCfg.m_qpOutValsCb, sizeof( c->m_qpOutValsCb ) ) == 0 )
    {
      memset(&c->m_qpOutValsCb,0, sizeof(c->m_qpOutValsCb));
      std::vector<int>  qpInVals = { 13,21,29,29,32,37 };
      std::copy(qpInVals.begin(), qpInVals.end(), c->m_qpOutValsCb);
    }
    if( memcmp( c->m_qpInValsCr, cBaseCfg.m_qpInValsCr, sizeof( c->m_qpInValsCr ) ) == 0 )
    {
      memset(&c->m_qpInValsCr,0, sizeof(c->m_qpInValsCr));
      std::vector<int>  qpInVals = { 13,20,37,41,44,54 };
      std::copy(qpInVals.begin(), qpInVals.end(), c->m_qpInValsCr);
    }
    if( memcmp( c->m_qpOutValsCr, cBaseCfg.m_qpOutValsCr, sizeof( c->m_qpOutValsCr ) ) == 0 )
    {
      memset(&c->m_qpOutValsCr,0, sizeof(c->m_qpOutValsCr));
      std::vector<int>  qpInVals = { 13,21,27,29,32,37 };
      std::copy(qpInVals.begin(), qpInVals.end(), c->m_qpOutValsCr);
    }
    if( memcmp( c->m_qpInValsCbCr, cBaseCfg.m_qpInValsCbCr, sizeof( c->m_qpInValsCbCr ) ) == 0 )
    {
      memset(&c->m_qpInValsCbCr,0, sizeof(c->m_qpInValsCbCr));
      std::vector<int>  qpInVals = { 12,21,41,43,54 };
      std::copy(qpInVals.begin(), qpInVals.end(), c->m_qpInValsCbCr);
    }
    if( memcmp( c->m_qpOutValsCbCr, cBaseCfg.m_qpOutValsCbCr, sizeof( c->m_qpOutValsCbCr ) ) == 0 )
    {
      memset(&c->m_qpOutValsCbCr,0, sizeof(c->m_qpOutValsCbCr));
      std::vector<int>  qpInVals = { 12,22,30,32,37 };
      std::copy(qpInVals.begin(), qpInVals.end(), c->m_qpOutValsCbCr);
    }

    // VUI and SEI options
    c->m_vuiParametersPresent     = c->m_vuiParametersPresent != 0 ? 1:0; // enable vui only if not explicitly disabled
    c->m_colourDescriptionPresent = true;                                // enable colour_primaries, transfer_characteristics and matrix_coefficients in vui

    c->m_transferCharacteristics = 16; // smpte2084 - HDR10
    if( c->m_colourPrimaries == 2 )
    {
      c->m_colourPrimaries = c->m_HdrMode == VVENC_HDR_PQ_BT2020 ? 9 : 1; //  bt2020(9) : bt709 (1)
    }
    if( c->m_matrixCoefficients == 2 )
    {
      c->m_matrixCoefficients = c->m_HdrMode == VVENC_HDR_PQ_BT2020 ? 9 : 1; // bt2020nc : bt709
    }
  }
  else if( c->m_HdrMode == VVENC_HDR_HLG || c->m_HdrMode == VVENC_HDR_HLG_BT2020 )
  {
    c->m_reshapeSignalType       = vvenc::RESHAPE_SIGNAL_HLG;
    c->m_LMCSOffset              = 0;
    c->m_useSameChromaQPTables   = true;
    c->m_verCollocatedChromaFlag = true;

    vvenc_config cBaseCfg;
    vvenc_config_default(&cBaseCfg);
    // if qpInVal/qpOutVal are set to default value and not overwritten by user defined values, overwrite them with PQ/HLG specifc qp values
    if( memcmp( c->m_qpInValsCb, cBaseCfg.m_qpInValsCb, sizeof( c->m_qpInValsCb ) ) == 0 )
    {
      std::vector<int>  qpVals = { 9, 23, 33, 42 };
      std::copy(qpVals.begin(), qpVals.end(), c->m_qpInValsCb);
    }
    if( memcmp( c->m_qpOutValsCb, cBaseCfg.m_qpOutValsCb, sizeof( c->m_qpOutValsCb ) ) == 0 )
    {
      std::vector<int>  qpVals = { 9, 24, 33, 37 };
      std::copy(qpVals.begin(), qpVals.end(), c->m_qpOutValsCb);
    }

    // VUI and SEI options
    c->m_vuiParametersPresent = c->m_vuiParametersPresent != 0 ? 1:0; // enable vui only if not explicitly disabled
    c->m_colourDescriptionPresent = true;                            // enable colour_primaries, transfer_characteristics and matrix_coefficients in vui

    if( c->m_colourPrimaries == 2 )
    {
      c->m_colourPrimaries = c->m_HdrMode == VVENC_HDR_HLG_BT2020 ? 9 : 1; //  bt2020(9) : bt709 (1)
    }

    if( c->m_matrixCoefficients == 2 )
    {
      c->m_matrixCoefficients = c->m_HdrMode == VVENC_HDR_HLG_BT2020 ? 9 : 1; // bt2020nc : bt709
    }

    if( c->m_transferCharacteristics == 2 )
    {
      c->m_transferCharacteristics = c->m_HdrMode == VVENC_HDR_HLG_BT2020 ? 14 : 1; // bt2020-10 : bt709
    }

    if( c->m_preferredTransferCharacteristics < 0 )
    {
      c->m_preferredTransferCharacteristics = 18; // ARIB STD-B67 (HLG)
    }
  }

  if( c->m_preferredTransferCharacteristics < 0 )
  {
    c->m_preferredTransferCharacteristics = 0;
  }

  if( c->m_AccessUnitDelimiter < 0 )
  {
    c->m_AccessUnitDelimiter = 0;
  }

  if ( c->m_vuiParametersPresent < 0 )
  {
    c->m_vuiParametersPresent = 0;
  }

  if ( c->m_hrdParametersPresent < 0 )
  {
    c->m_hrdParametersPresent = 0;
  }

  switch ( c->m_conformanceWindowMode)
  {
  case 0:
    {
      // no conformance or padding
      c->m_confWinLeft = c->m_confWinRight = c->m_confWinTop = c->m_confWinBottom = 0;
      c->m_aiPad[1] = c->m_aiPad[0] = 0;
      break;
    }
  case 1:
    {
      // automatic padding to minimum CU size
      const int minCuSize = 1 << ( vvenc::MIN_CU_LOG2 + 1 );
      if (c->m_SourceWidth % minCuSize)
      {
        c->m_aiPad[0] = c->m_confWinRight  = ((c->m_SourceWidth / minCuSize) + 1) * minCuSize - c->m_SourceWidth;
      }
      if (c->m_SourceHeight % minCuSize)
      {
        c->m_aiPad[1] =c->m_confWinBottom = ((c->m_SourceHeight / minCuSize) + 1) * minCuSize - c->m_SourceHeight;
      }
      break;
    }
  case 2:
    {
      //padding
      c->m_confWinRight  = c->m_aiPad[0];
      c->m_confWinBottom = c->m_aiPad[1];
      break;
    }
  case 3:
    {
      // conformance
      if ((c->m_confWinLeft == 0) && (c->m_confWinRight == 0) && (c->m_confWinTop == 0) && (c->m_confWinBottom == 0))
      {
        vvenc::msg( VVENC_ERROR, "Warning: Conformance window enabled, but all conformance window parameters set to zero\n");
      }
      if ((c->m_aiPad[1] != 0) || (c->m_aiPad[0]!=0))
      {
        vvenc::msg( VVENC_ERROR, "Warning: Conformance window enabled, padding parameters will be ignored\n");
      }
      c->m_aiPad[1] = c->m_aiPad[0] = 0;
      break;
    }
  }
    c->m_PadSourceWidth  = c->m_SourceWidth  + c->m_aiPad[0];
    c->m_PadSourceHeight = c->m_SourceHeight + c->m_aiPad[1];

  for(uint32_t ch=0; ch < 2; ch++ )
  {
    if (c->m_saoOffsetBitShift[ch]<0)
    {
      if (c->m_internalBitDepth[ch]>10)
      {
        c->m_log2SaoOffsetScale[ch]=uint32_t(vvenc::Clip3<int>(0, c->m_internalBitDepth[ch]-10, int(c->m_internalBitDepth[ch]-10 + 0.165*c->m_QP - 3.22 + 0.5) ) );
      }
      else
      {
        c->m_log2SaoOffsetScale[ch]=0;
      }
    }
    else
    {
      c->m_log2SaoOffsetScale[ch]=uint32_t(c->m_saoOffsetBitShift[ch]);
    }
  }

  c->m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag = c->m_useSameChromaQPTables;

  if (c->m_useIdentityTableForNon420Chroma && c->m_internChromaFormat != VVENC_CHROMA_420)
  {
    c->m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag = true;
    memset(&c->m_qpInValsCb   ,0, sizeof(c->m_qpInValsCb));
    memset(&c->m_qpInValsCr   ,0, sizeof(c->m_qpInValsCr));
    memset(&c->m_qpInValsCbCr ,0, sizeof(c->m_qpInValsCbCr));
    memset(&c->m_qpOutValsCb  ,0, sizeof(c->m_qpOutValsCb));
    memset(&c->m_qpOutValsCr  ,0, sizeof(c->m_qpOutValsCr));
    memset(&c->m_qpOutValsCbCr,0, sizeof(c->m_qpOutValsCbCr));
  }

  std::vector<int> qpInValsCb  = vvenc_getQpValsAsVec( c->m_qpInValsCb );
  std::vector<int> qpInValsCr = vvenc_getQpValsAsVec( c->m_qpInValsCr );
  std::vector<int> qpInValsCbCr= vvenc_getQpValsAsVec( c->m_qpInValsCbCr );
  std::vector<int> qpOutValsCb= vvenc_getQpValsAsVec( c->m_qpOutValsCb );
  std::vector<int> qpOutValsCr= vvenc_getQpValsAsVec( c->m_qpOutValsCr );
  std::vector<int> qpOutValsCbCr= vvenc_getQpValsAsVec( c->m_qpOutValsCbCr );

  c->m_chromaQpMappingTableParams.m_numQpTables = c->m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag? 1 : (c->m_JointCbCrMode ? 3 : 2);
  c->m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[0] = (qpOutValsCb.size() > 1) ? (int)qpOutValsCb.size() - 2 : 0;
  c->m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] = (qpOutValsCb.size() > 1) ? -26 + qpInValsCb[0] : 0;
  for ( size_t i = 0; i < qpInValsCb.size() - 1; i++)
  {
    c->m_chromaQpMappingTableParams.m_deltaQpInValMinus1[0][i] = qpInValsCb[i + 1] - qpInValsCb[i] - 1;
    c->m_chromaQpMappingTableParams.m_deltaQpOutVal[0][i] = qpOutValsCb[i + 1] - qpOutValsCb[i];
  }
  if (!c->m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag)
  {
    c->m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[1] = (qpOutValsCr.size() > 1) ? (int)qpOutValsCr.size() - 2 : 0;
    c->m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] = (qpOutValsCr.size() > 1) ? -26 + qpInValsCr[0] : 0;
    for (size_t i = 0; i < qpInValsCr.size() - 1; i++)
    {
      c->m_chromaQpMappingTableParams.m_deltaQpInValMinus1[1][i] = qpInValsCr[i + 1] - qpInValsCr[i] - 1;
      c->m_chromaQpMappingTableParams.m_deltaQpOutVal[1][i] = qpOutValsCr[i + 1] - qpOutValsCr[i];
    }
    c->m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[2] = (qpOutValsCbCr.size() > 1) ? (int)qpOutValsCbCr.size() - 2 : 0;
    c->m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] = (qpOutValsCbCr.size() > 1) ? -26 + qpInValsCbCr[0] : 0;
    for (size_t i = 0; i < qpInValsCbCr.size() - 1; i++)
    {
      c->m_chromaQpMappingTableParams.m_deltaQpInValMinus1[2][i] = qpInValsCbCr[i + 1] - qpInValsCbCr[i] - 1;
      c->m_chromaQpMappingTableParams.m_deltaQpOutVal[2][i] = qpInValsCbCr[i + 1] - qpInValsCbCr[i];
    }
  }

  const int minCuSize = 1 << vvenc::MIN_CU_LOG2;
  c->m_MaxCodingDepth = 0;
  while( ( c->m_CTUSize >> c->m_MaxCodingDepth ) > minCuSize )
  {
    c->m_MaxCodingDepth++;
  }
  c->m_log2DiffMaxMinCodingBlockSize = c->m_MaxCodingDepth;

  c->m_reshapeCW.rspFps     = c->m_FrameRate;
  c->m_reshapeCW.rspPicSize = c->m_PadSourceWidth*c->m_PadSourceHeight;
  c->m_reshapeCW.rspFpsToIp = std::max(16, 16 * (int)(round((double)c->m_FrameRate /16.0)));
  c->m_reshapeCW.rspBaseQP  = c->m_QP;
  c->m_reshapeCW.updateCtrl = c->m_updateCtrl;
  c->m_reshapeCW.adpOption  = c->m_adpOption;
  c->m_reshapeCW.initialCW  = c->m_initialCW;

  if( c->m_rprEnabledFlag == -1 )
  {
    c->m_rprEnabledFlag = c->m_DecodingRefreshType == VVENC_DRT_CRA_CRE ? 2 : 0;
  }
  
  vvenc_confirmParameter( c, c->m_rprEnabledFlag < -1 || c->m_rprEnabledFlag > 2, "RPR must be either -1, 0, 1 or 2" );
  vvenc_confirmParameter( c, c->m_rprEnabledFlag == 2 && c->m_DecodingRefreshType != VVENC_DRT_CRA_CRE, "for using RPR=2 costrained rasl encoding, DecodingRefreshType has to be set to VVENC_DRT_CRA_CRE" );

  if( c->m_rprEnabledFlag == 2 )
  {
    c->m_resChangeInClvsEnabled = true;
    c->m_craAPSreset            = true;
    c->m_rprRASLtoolSwitch      = true;
  }  
    
  if( c->m_IntraPeriod == 0 &&  c->m_IntraPeriodSec > 0 )
  {
    if ( c->m_FrameRate % c->m_GOPSize == 0 )
    {
      c->m_IntraPeriod = c->m_FrameRate * c->m_IntraPeriodSec;
    }
    else
    {
      int iIDRPeriod  = (c->m_FrameRate * c->m_IntraPeriodSec);
      if( iIDRPeriod < c->m_GOPSize )
      {
        iIDRPeriod = c->m_GOPSize;
      }

      int iDiff = iIDRPeriod % c->m_GOPSize;
      if( iDiff < c->m_GOPSize >> 1 )
      {
        c->m_IntraPeriod = iIDRPeriod - iDiff;
      }
      else
      {
        c->m_IntraPeriod = iIDRPeriod + c->m_GOPSize - iDiff;
      }
    }
  }

  //
  // do some check and set of parameters next
  //

  if ( c->m_lumaReshapeEnable )
  {
    if ( c->m_updateCtrl > 0 && c->m_adpOption > 2 ) { c->m_adpOption -= 2; }
  }

  if ( c->m_JointCbCrMode && ( c->m_internChromaFormat == VVENC_CHROMA_400) )
  {
    c->m_JointCbCrMode = false;
  }

  if ( c->m_vvencMCTF.MCTF && c->m_QP < 17 )
  {
    vvenc::msg( VVENC_WARNING, "disable MCTF for QP < 17\n");
    c->m_vvencMCTF.MCTF = 0;
  }
  if( c->m_vvencMCTF.MCTF )
  {
    if( c->m_vvencMCTF.numFrames == 0 && c->m_vvencMCTF.numStrength == 0 )
    {
      if( c->m_GOPSize == 32 )
      {
#if JVET_V0056_MCTF
        c->m_vvencMCTF.MCTFFrames[0] = 8;
        c->m_vvencMCTF.MCTFFrames[1] = 16;
        c->m_vvencMCTF.MCTFFrames[2] = 32;

        c->m_vvencMCTF.MCTFStrengths[0] = 0.95;
        c->m_vvencMCTF.MCTFStrengths[1] = 1.5;
        c->m_vvencMCTF.MCTFStrengths[2] = 1.5;
        c->m_vvencMCTF.numFrames = c->m_vvencMCTF.numStrength = 3;
#else
        c->m_vvencMCTF.MCTFFrames[0] = 8;
        c->m_vvencMCTF.MCTFFrames[1] = 16;
        c->m_vvencMCTF.MCTFFrames[2] = 32;

        c->m_vvencMCTF.MCTFStrengths[0] = 0.28125;     //  9/32
        c->m_vvencMCTF.MCTFStrengths[1] = 0.5625;      // 18/32
        c->m_vvencMCTF.MCTFStrengths[2] = 0.84375;     // 27/32
        c->m_vvencMCTF.numFrames = c->m_vvencMCTF.numStrength = 3;
#endif
      }
      else if( c->m_GOPSize == 16 )
      {
#if JVET_V0056_MCTF
        c->m_vvencMCTF.MCTFFrames[0] = 8;
        c->m_vvencMCTF.MCTFFrames[1] = 16;

        c->m_vvencMCTF.MCTFStrengths[0] = 0.95;
        c->m_vvencMCTF.MCTFStrengths[1] = 1.5;
        c->m_vvencMCTF.numFrames = c->m_vvencMCTF.numStrength = 2;
#else
        c->m_vvencMCTF.MCTFFrames[0] = 8;
        c->m_vvencMCTF.MCTFFrames[1] = 16;

        c->m_vvencMCTF.MCTFStrengths[0] = 0.4;     // ~12.75/32
        c->m_vvencMCTF.MCTFStrengths[1] = 0.8;     // ~25.50/32
        c->m_vvencMCTF.numFrames = c->m_vvencMCTF.numStrength = 2;
#endif
      }
      else if( c->m_GOPSize == 8 )
      {
        c->m_vvencMCTF.MCTFFrames[0]    = 8;
        c->m_vvencMCTF.MCTFStrengths[0] = 0.65625;     // 21/32
        c->m_vvencMCTF.numFrames = c->m_vvencMCTF.numStrength = 1;
      }
    }
  }

  if ( c->m_usePerceptQPATempFiltISlice < 0 )
  {
    c->m_usePerceptQPATempFiltISlice = 0;
    if ( c->m_usePerceptQPA ) // automatic mode for temporal filtering depending on RC
    {
      c->m_usePerceptQPATempFiltISlice = ( c->m_RCTargetBitrate > 0 && c->m_RCNumPasses == 2 ? 2 : 1 );
    }
  }
  if ( c->m_usePerceptQPATempFiltISlice == 2
      && ( c->m_QP <= 27 || c->m_QP > vvenc::MAX_QP_PERCEPT_QPA || c->m_GOPSize <= 8 || c->m_IntraPeriod < 2 * c->m_GOPSize) )
  {
    c->m_usePerceptQPATempFiltISlice = 1; // disable temporal pumping reduction aspect
  }
  if ( c->m_usePerceptQPATempFiltISlice > 0
      && ( c->m_vvencMCTF.MCTF == 0 || ! c->m_usePerceptQPA) )
  {
    c->m_usePerceptQPATempFiltISlice = 0; // fully disable temporal filtering features
  }

  if ( c->m_cuQpDeltaSubdiv < 0)
  {
    c->m_cuQpDeltaSubdiv = 0;
    if ( c->m_usePerceptQPA
        && c->m_QP <= vvenc::MAX_QP_PERCEPT_QPA
        && ( c->m_CTUSize == 128 || ( c->m_CTUSize == 64 && c->m_PadSourceWidth <= 1024 && c->m_PadSourceHeight <= 640 ) )
        && c->m_PadSourceWidth <= 2048
        && c->m_PadSourceHeight <= 1280 )
    {
      c->m_cuQpDeltaSubdiv = 2;
    }
  }
  if ( c->m_sliceChromaQpOffsetPeriodicity < 0)
  {
    c->m_sliceChromaQpOffsetPeriodicity = 0;
    if ( c->m_usePerceptQPA && c->m_internChromaFormat != VVENC_CHROMA_400 )
    {
      c->m_sliceChromaQpOffsetPeriodicity = 1;
    }
  }

  if ( c->m_usePerceptQPA ) c->m_ccalfQpThreshold = vvenc::MAX_QP_PERCEPT_QPA;

  /* if this is an intra-only sequence, ie IntraPeriod=1, don't verify the GOP structure
   * This permits the ability to omit a GOP structure specification */
  if ( c->m_IntraPeriod == 1 && c->m_GOPList[0].m_POC == -1 )
  {
    vvenc_GOPEntry_default( &c->m_GOPList[0] );
    c->m_GOPList[0].m_QPFactor = 1;
    c->m_GOPList[0].m_betaOffsetDiv2 = 0;
    c->m_GOPList[0].m_tcOffsetDiv2 = 0;
    c->m_GOPList[0].m_POC = 1;
    vvenc_RPLEntry_default( &c->m_RPLList0[0] );
    vvenc_RPLEntry_default( &c->m_RPLList1[0] );
    c->m_RPLList0[0].m_POC = c->m_RPLList1[0].m_POC = 1;
    c->m_RPLList0[0].m_numRefPicsActive = 4;
    c->m_GOPList[0].m_numRefPicsActive[0] = 4;
  }
  else
  {
    // set default RA config
    if( c->m_GOPSize == 16 && c->m_GOPList[0].m_POC == -1 && c->m_GOPList[1].m_POC == -1 )
    {
      for( int i = 0; i < 16; i++ )
      {
        vvenc_GOPEntry_default( &c->m_GOPList[i] );
        c->m_GOPList[i].m_sliceType = 'B';
        c->m_GOPList[i].m_QPFactor = 1;

        c->m_GOPList[i].m_numRefPicsActive[0] = 2;
        c->m_GOPList[i].m_numRefPicsActive[1] = 2;
        c->m_GOPList[i].m_numRefPics[0] = 2;
        c->m_GOPList[i].m_numRefPics[1] = 2;
      }
      c->m_GOPList[0].m_POC  = 16;  c->m_GOPList[0].m_temporalId  = 0;
      c->m_GOPList[1].m_POC  =  8;  c->m_GOPList[1].m_temporalId  = 1;
      c->m_GOPList[2].m_POC  =  4;  c->m_GOPList[2].m_temporalId  = 2;
      c->m_GOPList[3].m_POC  =  2;  c->m_GOPList[3].m_temporalId  = 3;
      c->m_GOPList[4].m_POC  =  1;  c->m_GOPList[4].m_temporalId  = 4;
      c->m_GOPList[5].m_POC  =  3;  c->m_GOPList[5].m_temporalId  = 4;
      c->m_GOPList[6].m_POC  =  6;  c->m_GOPList[6].m_temporalId  = 3;
      c->m_GOPList[7].m_POC  =  5;  c->m_GOPList[7].m_temporalId  = 4;
      c->m_GOPList[8].m_POC  =  7;  c->m_GOPList[8].m_temporalId  = 4;
      c->m_GOPList[9].m_POC  = 12;  c->m_GOPList[9].m_temporalId  = 2;
      c->m_GOPList[10].m_POC = 10;  c->m_GOPList[10].m_temporalId = 3;
      c->m_GOPList[11].m_POC =  9;  c->m_GOPList[11].m_temporalId = 4;
      c->m_GOPList[12].m_POC = 11;  c->m_GOPList[12].m_temporalId = 4;
      c->m_GOPList[13].m_POC = 14;  c->m_GOPList[13].m_temporalId = 3;
      c->m_GOPList[14].m_POC = 13;  c->m_GOPList[14].m_temporalId = 4;
      c->m_GOPList[15].m_POC = 15;  c->m_GOPList[15].m_temporalId = 4;

      c->m_GOPList[0].m_numRefPics[0]  = 3;
      c->m_GOPList[8].m_numRefPics[0]  = 3;
      c->m_GOPList[12].m_numRefPics[0] = 3;
      c->m_GOPList[13].m_numRefPics[0] = 3;
      c->m_GOPList[14].m_numRefPics[0] = 3;
      c->m_GOPList[15].m_numRefPics[0] = 4;

      c->m_GOPList[0].m_deltaRefPics[0][0]  = 16; c->m_GOPList[0].m_deltaRefPics[0][1]  = 32; c->m_GOPList[0].m_deltaRefPics[0][2]  = 24;
      c->m_GOPList[1].m_deltaRefPics[0][0]  =  8; c->m_GOPList[1].m_deltaRefPics[0][1]  = 16;
      c->m_GOPList[2].m_deltaRefPics[0][0]  =  4; c->m_GOPList[2].m_deltaRefPics[0][1]  = 12;
      c->m_GOPList[3].m_deltaRefPics[0][0]  =  2; c->m_GOPList[3].m_deltaRefPics[0][1]  = 10;
      c->m_GOPList[4].m_deltaRefPics[0][0]  =  1; c->m_GOPList[4].m_deltaRefPics[0][1]  = -1;
      c->m_GOPList[5].m_deltaRefPics[0][0]  =  1; c->m_GOPList[5].m_deltaRefPics[0][1]  = 3;
      c->m_GOPList[6].m_deltaRefPics[0][0]  =  2; c->m_GOPList[6].m_deltaRefPics[0][1]  = 6;
      c->m_GOPList[7].m_deltaRefPics[0][0]  =  1; c->m_GOPList[7].m_deltaRefPics[0][1]  = 5;
      c->m_GOPList[8].m_deltaRefPics[0][0]  =  1; c->m_GOPList[8].m_deltaRefPics[0][1]  = 3; c->m_GOPList[8].m_deltaRefPics[0][2]  = 7;
      c->m_GOPList[9].m_deltaRefPics[0][0]  =  4; c->m_GOPList[9].m_deltaRefPics[0][1]  = 12;
      c->m_GOPList[10].m_deltaRefPics[0][0] =  2; c->m_GOPList[10].m_deltaRefPics[0][1] = 10;
      c->m_GOPList[11].m_deltaRefPics[0][0] =  1; c->m_GOPList[11].m_deltaRefPics[0][1] = 9;
      c->m_GOPList[12].m_deltaRefPics[0][0] =  1; c->m_GOPList[12].m_deltaRefPics[0][1] = 3; c->m_GOPList[12].m_deltaRefPics[0][2]  = 11;
      c->m_GOPList[13].m_deltaRefPics[0][0] =  2; c->m_GOPList[13].m_deltaRefPics[0][1] = 6; c->m_GOPList[13].m_deltaRefPics[0][2]  = 14;
      c->m_GOPList[14].m_deltaRefPics[0][0] =  1; c->m_GOPList[14].m_deltaRefPics[0][1] = 5; c->m_GOPList[14].m_deltaRefPics[0][2]  = 13;
      c->m_GOPList[15].m_deltaRefPics[0][0] =  1; c->m_GOPList[15].m_deltaRefPics[0][1] = 3; c->m_GOPList[15].m_deltaRefPics[0][2]  = 7; c->m_GOPList[15].m_deltaRefPics[0][3]  = 15;

      c->m_GOPList[3].m_numRefPics[1]  = 3;
      c->m_GOPList[4].m_numRefPics[1]  = 4;
      c->m_GOPList[5].m_numRefPics[1]  = 3;
      c->m_GOPList[7].m_numRefPics[1]  = 3;
      c->m_GOPList[11].m_numRefPics[1] = 3;

      c->m_GOPList[0].m_deltaRefPics[1][0]  = 16; c->m_GOPList[0].m_deltaRefPics[1][1]  =  32;
      c->m_GOPList[1].m_deltaRefPics[1][0]  = -8; c->m_GOPList[1].m_deltaRefPics[1][1]  =   8;
      c->m_GOPList[2].m_deltaRefPics[1][0]  = -4; c->m_GOPList[2].m_deltaRefPics[1][1]  = -12;
      c->m_GOPList[3].m_deltaRefPics[1][0]  = -2; c->m_GOPList[3].m_deltaRefPics[1][1]  =  -6; c->m_GOPList[3].m_deltaRefPics[1][2]  = -14;
      c->m_GOPList[4].m_deltaRefPics[1][0]  = -1; c->m_GOPList[4].m_deltaRefPics[1][1]  =  -3; c->m_GOPList[4].m_deltaRefPics[1][2]  =  -7;  c->m_GOPList[4].m_deltaRefPics[1][3]  = -15;
      c->m_GOPList[5].m_deltaRefPics[1][0]  = -1; c->m_GOPList[5].m_deltaRefPics[1][1]  =  -5; c->m_GOPList[5].m_deltaRefPics[1][2]  = -13;
      c->m_GOPList[6].m_deltaRefPics[1][0]  = -2; c->m_GOPList[6].m_deltaRefPics[1][1]  =  -10;
      c->m_GOPList[7].m_deltaRefPics[1][0]  = -1; c->m_GOPList[7].m_deltaRefPics[1][1]  =  -3; c->m_GOPList[7].m_deltaRefPics[1][2]  = -11;
      c->m_GOPList[8].m_deltaRefPics[1][0]  = -1; c->m_GOPList[8].m_deltaRefPics[1][1]  =  -9;
      c->m_GOPList[9].m_deltaRefPics[1][0]  = -4; c->m_GOPList[9].m_deltaRefPics[1][1]  =   4;
      c->m_GOPList[10].m_deltaRefPics[1][0] = -2; c->m_GOPList[10].m_deltaRefPics[1][1] =  -6;
      c->m_GOPList[11].m_deltaRefPics[1][0] = -1; c->m_GOPList[11].m_deltaRefPics[1][1] =  -3; c->m_GOPList[11].m_deltaRefPics[1][2]  = -7;
      c->m_GOPList[12].m_deltaRefPics[1][0] = -1; c->m_GOPList[12].m_deltaRefPics[1][1] =  -5;
      c->m_GOPList[13].m_deltaRefPics[1][0] = -2; c->m_GOPList[13].m_deltaRefPics[1][1] =   2;
      c->m_GOPList[14].m_deltaRefPics[1][0] = -1; c->m_GOPList[14].m_deltaRefPics[1][1] =  -3;
      c->m_GOPList[15].m_deltaRefPics[1][0] = -1; c->m_GOPList[15].m_deltaRefPics[1][1] =   1;

      for( int i = 0; i < 16; i++ )
      {
        switch( c->m_GOPList[i].m_temporalId )
        {
        case 0: c->m_GOPList[i].m_QPOffset   = 1;
                c->m_GOPList[i].m_QPOffsetModelOffset = 0.0;
                c->m_GOPList[i].m_QPOffsetModelScale  = 0.0;
        break;
        case 1: c->m_GOPList[i].m_QPOffset   = 1;
                c->m_GOPList[i].m_QPOffsetModelOffset = -4.8848;
                c->m_GOPList[i].m_QPOffsetModelScale  = 0.2061;
        break;
        case 2: c->m_GOPList[i].m_QPOffset   = 4;
                c->m_GOPList[i].m_QPOffsetModelOffset = -5.7476;
                c->m_GOPList[i].m_QPOffsetModelScale  = 0.2286;
        break;
        case 3: c->m_GOPList[i].m_QPOffset   = 5;
                c->m_GOPList[i].m_QPOffsetModelOffset = -5.90;
                c->m_GOPList[i].m_QPOffsetModelScale  = 0.2333;
        break;
        case 4: c->m_GOPList[i].m_QPOffset   = 6;
                c->m_GOPList[i].m_QPOffsetModelOffset = -7.1444;
                c->m_GOPList[i].m_QPOffsetModelScale  = 0.3;
        break;
        default: break;
        }
      }
    }
    else if( c->m_GOPSize == 32 &&
            ( (c->m_GOPList[0].m_POC == -1 && c->m_GOPList[1].m_POC == -1) ||
              (c->m_GOPList[16].m_POC == -1 && c->m_GOPList[17].m_POC == -1)
              ) )
    {
      for( int i = 0; i < 32; i++ )
      {
        vvenc_GOPEntry_default(&c->m_GOPList[i]);
        c->m_GOPList[i].m_sliceType = 'B';
        c->m_GOPList[i].m_QPFactor = 1;

        c->m_GOPList[i].m_numRefPicsActive[0] = 2;
        c->m_GOPList[i].m_numRefPicsActive[1] = 2;
        c->m_GOPList[i].m_numRefPics[0] = 2;
        c->m_GOPList[i].m_numRefPics[1] = 2;
      }
      c->m_GOPList[ 0].m_POC = 32;   c->m_GOPList[0].m_temporalId  = 0;
      c->m_GOPList[ 1].m_POC = 16;   c->m_GOPList[1].m_temporalId  = 1;
      c->m_GOPList[ 2].m_POC =  8;   c->m_GOPList[2].m_temporalId  = 2;
      c->m_GOPList[ 3].m_POC =  4;   c->m_GOPList[3].m_temporalId  = 3;
      c->m_GOPList[ 4].m_POC =  2;   c->m_GOPList[4].m_temporalId  = 4;
      c->m_GOPList[ 5].m_POC =  1;   c->m_GOPList[5].m_temporalId  = 5;
      c->m_GOPList[ 6].m_POC =  3;   c->m_GOPList[6].m_temporalId  = 5;
      c->m_GOPList[ 7].m_POC =  6;   c->m_GOPList[7].m_temporalId  = 4;
      c->m_GOPList[ 8].m_POC =  5;   c->m_GOPList[8].m_temporalId  = 5;
      c->m_GOPList[ 9].m_POC =  7;   c->m_GOPList[9].m_temporalId  = 5;
      c->m_GOPList[10].m_POC = 12;   c->m_GOPList[10].m_temporalId = 3;
      c->m_GOPList[11].m_POC = 10;   c->m_GOPList[11].m_temporalId = 4;
      c->m_GOPList[12].m_POC =  9;   c->m_GOPList[12].m_temporalId = 5;
      c->m_GOPList[13].m_POC = 11;   c->m_GOPList[13].m_temporalId = 5;
      c->m_GOPList[14].m_POC = 14;   c->m_GOPList[14].m_temporalId = 4;
      c->m_GOPList[15].m_POC = 13;   c->m_GOPList[15].m_temporalId = 5;

      c->m_GOPList[16].m_POC = 15;   c->m_GOPList[16].m_temporalId = 5;
      c->m_GOPList[17].m_POC = 24;   c->m_GOPList[17].m_temporalId = 2;
      c->m_GOPList[18].m_POC = 20;   c->m_GOPList[18].m_temporalId = 3;
      c->m_GOPList[19].m_POC = 18;   c->m_GOPList[19].m_temporalId = 4;
      c->m_GOPList[20].m_POC = 17;   c->m_GOPList[20].m_temporalId = 5;
      c->m_GOPList[21].m_POC = 19;   c->m_GOPList[21].m_temporalId = 5;
      c->m_GOPList[22].m_POC = 22;   c->m_GOPList[22].m_temporalId = 4;
      c->m_GOPList[23].m_POC = 21;   c->m_GOPList[23].m_temporalId = 5;
      c->m_GOPList[24].m_POC = 23;   c->m_GOPList[24].m_temporalId = 5;
      c->m_GOPList[25].m_POC = 28;   c->m_GOPList[25].m_temporalId = 3;
      c->m_GOPList[26].m_POC = 26;   c->m_GOPList[26].m_temporalId = 4;
      c->m_GOPList[27].m_POC = 25;   c->m_GOPList[27].m_temporalId = 5;
      c->m_GOPList[28].m_POC = 27;   c->m_GOPList[28].m_temporalId = 5;
      c->m_GOPList[29].m_POC = 30;   c->m_GOPList[29].m_temporalId = 4;
      c->m_GOPList[30].m_POC = 29;   c->m_GOPList[30].m_temporalId = 5;
      c->m_GOPList[31].m_POC = 31;   c->m_GOPList[31].m_temporalId = 5;

      c->m_GOPList[ 0].m_numRefPics[0] = 3;
      c->m_GOPList[ 1].m_numRefPics[0] = 2;
      c->m_GOPList[ 2].m_numRefPics[0] = 2;
      c->m_GOPList[ 3].m_numRefPics[0] = 2;
      c->m_GOPList[ 4].m_numRefPics[0] = 2;
      c->m_GOPList[ 5].m_numRefPics[0] = 2;
      c->m_GOPList[ 6].m_numRefPics[0] = 2;
      c->m_GOPList[ 7].m_numRefPics[0] = 2;
      c->m_GOPList[ 8].m_numRefPics[0] = 2;
      c->m_GOPList[ 9].m_numRefPics[0] = 3;
      c->m_GOPList[10].m_numRefPics[0] = 2;
      c->m_GOPList[11].m_numRefPics[0] = 2;
      c->m_GOPList[12].m_numRefPics[0] = 2;
      c->m_GOPList[13].m_numRefPics[0] = 3;
      c->m_GOPList[14].m_numRefPics[0] = 3;
      c->m_GOPList[15].m_numRefPics[0] = 3;

      c->m_GOPList[16].m_numRefPics[0] = 4;
      c->m_GOPList[17].m_numRefPics[0] = 2;
      c->m_GOPList[18].m_numRefPics[0] = 2;
      c->m_GOPList[19].m_numRefPics[0] = 2;
      c->m_GOPList[20].m_numRefPics[0] = 2;
      c->m_GOPList[21].m_numRefPics[0] = 3;
      c->m_GOPList[22].m_numRefPics[0] = 3;
      c->m_GOPList[23].m_numRefPics[0] = 3;
      c->m_GOPList[24].m_numRefPics[0] = 4;
      c->m_GOPList[25].m_numRefPics[0] = 3;
      c->m_GOPList[26].m_numRefPics[0] = 3;
      c->m_GOPList[27].m_numRefPics[0] = 3;
      c->m_GOPList[28].m_numRefPics[0] = 4;
      c->m_GOPList[29].m_numRefPics[0] = 3;
      c->m_GOPList[30].m_numRefPics[0] = 3;
      c->m_GOPList[31].m_numRefPics[0] = 4;

      c->m_GOPList[ 0].m_deltaRefPics[0][0] = 32; c->m_GOPList[ 0].m_deltaRefPics[0][1] = 64; c->m_GOPList[ 0].m_deltaRefPics[0][2] = 48; //th swapped order of ref-pic 1 and 2
      c->m_GOPList[ 1].m_deltaRefPics[0][0] = 16; c->m_GOPList[ 1].m_deltaRefPics[0][1] = 32;
      c->m_GOPList[ 2].m_deltaRefPics[0][0] =  8; c->m_GOPList[ 2].m_deltaRefPics[0][1] = 24;
      c->m_GOPList[ 3].m_deltaRefPics[0][0] =  4; c->m_GOPList[ 3].m_deltaRefPics[0][1] = 20;

      c->m_GOPList[ 4].m_deltaRefPics[0][0] =  2; c->m_GOPList[ 4].m_deltaRefPics[0][1] = 18;
      c->m_GOPList[ 5].m_deltaRefPics[0][0] =  1; c->m_GOPList[ 5].m_deltaRefPics[0][1] = -1;
      c->m_GOPList[ 6].m_deltaRefPics[0][0] =  1; c->m_GOPList[ 6].m_deltaRefPics[0][1] =  3;
      c->m_GOPList[ 7].m_deltaRefPics[0][0] =  2; c->m_GOPList[ 7].m_deltaRefPics[0][1] =  6;

      c->m_GOPList[ 8].m_deltaRefPics[0][0] =  1; c->m_GOPList[ 8].m_deltaRefPics[0][1] =  5;
      c->m_GOPList[ 9].m_deltaRefPics[0][0] =  1; c->m_GOPList[ 9].m_deltaRefPics[0][1] =  3; c->m_GOPList[ 9].m_deltaRefPics[0][2] =  7;
      c->m_GOPList[10].m_deltaRefPics[0][0] =  4; c->m_GOPList[10].m_deltaRefPics[0][1] = 12;
      c->m_GOPList[11].m_deltaRefPics[0][0] =  2; c->m_GOPList[11].m_deltaRefPics[0][1] = 10;

      c->m_GOPList[12].m_deltaRefPics[0][0] =  1; c->m_GOPList[12].m_deltaRefPics[0][1] =  9;
      c->m_GOPList[13].m_deltaRefPics[0][0] =  1; c->m_GOPList[13].m_deltaRefPics[0][1] =  3; c->m_GOPList[13].m_deltaRefPics[0][2] = 11;
      c->m_GOPList[14].m_deltaRefPics[0][0] =  2; c->m_GOPList[14].m_deltaRefPics[0][1] =  6; c->m_GOPList[14].m_deltaRefPics[0][2] = 14;
      c->m_GOPList[15].m_deltaRefPics[0][0] =  1; c->m_GOPList[15].m_deltaRefPics[0][1] =  5; c->m_GOPList[15].m_deltaRefPics[0][2] = 13;

      c->m_GOPList[16].m_deltaRefPics[0][0] =  1; c->m_GOPList[16].m_deltaRefPics[0][1] =  3; c->m_GOPList[16].m_deltaRefPics[0][2] =  7; c->m_GOPList[16].m_deltaRefPics[0][3] = 15;
      c->m_GOPList[17].m_deltaRefPics[0][0] =  8; c->m_GOPList[17].m_deltaRefPics[0][1] = 24;
      c->m_GOPList[18].m_deltaRefPics[0][0] =  4; c->m_GOPList[18].m_deltaRefPics[0][1] = 20;
      c->m_GOPList[19].m_deltaRefPics[0][0] =  2; c->m_GOPList[19].m_deltaRefPics[0][1] = 18;

      c->m_GOPList[20].m_deltaRefPics[0][0] =  1; c->m_GOPList[20].m_deltaRefPics[0][1] = 17;
      c->m_GOPList[21].m_deltaRefPics[0][0] =  1; c->m_GOPList[21].m_deltaRefPics[0][1] =  3; c->m_GOPList[21].m_deltaRefPics[0][2] = 19;
      c->m_GOPList[22].m_deltaRefPics[0][0] =  2; c->m_GOPList[22].m_deltaRefPics[0][1] =  6; c->m_GOPList[22].m_deltaRefPics[0][2] = 22;
      c->m_GOPList[23].m_deltaRefPics[0][0] =  1; c->m_GOPList[23].m_deltaRefPics[0][1] =  5; c->m_GOPList[23].m_deltaRefPics[0][2] = 21;

      c->m_GOPList[24].m_deltaRefPics[0][0] =  1; c->m_GOPList[24].m_deltaRefPics[0][1] =  3; c->m_GOPList[24].m_deltaRefPics[0][2] =  7; c->m_GOPList[24].m_deltaRefPics[0][3] = 23;
      c->m_GOPList[25].m_deltaRefPics[0][0] =  4; c->m_GOPList[25].m_deltaRefPics[0][1] = 12; c->m_GOPList[25].m_deltaRefPics[0][2] = 28;
      c->m_GOPList[26].m_deltaRefPics[0][0] =  2; c->m_GOPList[26].m_deltaRefPics[0][1] = 10; c->m_GOPList[26].m_deltaRefPics[0][2] = 26;
      c->m_GOPList[27].m_deltaRefPics[0][0] =  1; c->m_GOPList[27].m_deltaRefPics[0][1] =  9; c->m_GOPList[27].m_deltaRefPics[0][2] = 25;

      c->m_GOPList[28].m_deltaRefPics[0][0] =  1; c->m_GOPList[28].m_deltaRefPics[0][1] =  3; c->m_GOPList[28].m_deltaRefPics[0][2] = 11; c->m_GOPList[28].m_deltaRefPics[0][3] = 27;
      c->m_GOPList[29].m_deltaRefPics[0][0] =  2; c->m_GOPList[29].m_deltaRefPics[0][1] = 14; c->m_GOPList[29].m_deltaRefPics[0][2] = 30;
      c->m_GOPList[30].m_deltaRefPics[0][0] =  1; c->m_GOPList[30].m_deltaRefPics[0][1] = 13; c->m_GOPList[30].m_deltaRefPics[0][2] = 29;
      c->m_GOPList[31].m_deltaRefPics[0][0] =  1; c->m_GOPList[31].m_deltaRefPics[0][1] =  3; c->m_GOPList[31].m_deltaRefPics[0][2] = 15; c->m_GOPList[31].m_deltaRefPics[0][3] = 31;

      c->m_GOPList[ 0].m_numRefPics[1] = 2;
      c->m_GOPList[ 1].m_numRefPics[1] = 2;
      c->m_GOPList[ 2].m_numRefPics[1] = 2;
      c->m_GOPList[ 3].m_numRefPics[1] = 3;
      c->m_GOPList[ 4].m_numRefPics[1] = 4;
      c->m_GOPList[ 5].m_numRefPics[1] = 5;
      c->m_GOPList[ 6].m_numRefPics[1] = 4;
      c->m_GOPList[ 7].m_numRefPics[1] = 3;
      c->m_GOPList[ 8].m_numRefPics[1] = 4;
      c->m_GOPList[ 9].m_numRefPics[1] = 3;
      c->m_GOPList[10].m_numRefPics[1] = 2;
      c->m_GOPList[11].m_numRefPics[1] = 3;
      c->m_GOPList[12].m_numRefPics[1] = 4;
      c->m_GOPList[13].m_numRefPics[1] = 3;
      c->m_GOPList[14].m_numRefPics[1] = 2;
      c->m_GOPList[15].m_numRefPics[1] = 3;

      c->m_GOPList[16].m_numRefPics[1] = 2;
      c->m_GOPList[17].m_numRefPics[1] = 2;
      c->m_GOPList[18].m_numRefPics[1] = 2;
      c->m_GOPList[19].m_numRefPics[1] = 3;
      c->m_GOPList[20].m_numRefPics[1] = 4;
      c->m_GOPList[21].m_numRefPics[1] = 3;
      c->m_GOPList[22].m_numRefPics[1] = 2;
      c->m_GOPList[23].m_numRefPics[1] = 3;
      c->m_GOPList[24].m_numRefPics[1] = 2;
      c->m_GOPList[25].m_numRefPics[1] = 2;
      c->m_GOPList[26].m_numRefPics[1] = 2;
      c->m_GOPList[27].m_numRefPics[1] = 3;
      c->m_GOPList[28].m_numRefPics[1] = 2;
      c->m_GOPList[29].m_numRefPics[1] = 2;
      c->m_GOPList[30].m_numRefPics[1] = 2;
      c->m_GOPList[31].m_numRefPics[1] = 2;

      c->m_GOPList[ 0].m_deltaRefPics[1][0] =  32; c->m_GOPList[ 0].m_deltaRefPics[1][1] =  64; //th48
      c->m_GOPList[ 1].m_deltaRefPics[1][0] = -16; c->m_GOPList[ 1].m_deltaRefPics[1][1] =  16;
      c->m_GOPList[ 2].m_deltaRefPics[1][0] =  -8; c->m_GOPList[ 2].m_deltaRefPics[1][1] = -24;
      c->m_GOPList[ 3].m_deltaRefPics[1][0] =  -4; c->m_GOPList[ 3].m_deltaRefPics[1][1] = -12; c->m_GOPList[ 3].m_deltaRefPics[1][2] = -28;

      c->m_GOPList[ 4].m_deltaRefPics[1][0] =  -2; c->m_GOPList[ 4].m_deltaRefPics[1][1] =  -6; c->m_GOPList[ 4].m_deltaRefPics[1][2] = -14; c->m_GOPList[ 4].m_deltaRefPics[1][3] = -30;
      c->m_GOPList[ 5].m_deltaRefPics[1][0] =  -1; c->m_GOPList[ 5].m_deltaRefPics[1][1] =  -3; c->m_GOPList[ 5].m_deltaRefPics[1][2] =  -7; c->m_GOPList[ 5].m_deltaRefPics[1][3] = -15; c->m_GOPList[5].m_deltaRefPics[1][4] = -31;
      c->m_GOPList[ 6].m_deltaRefPics[1][0] =  -1; c->m_GOPList[ 6].m_deltaRefPics[1][1] =  -5; c->m_GOPList[ 6].m_deltaRefPics[1][2] = -13; c->m_GOPList[ 6].m_deltaRefPics[1][3] = -29;
      c->m_GOPList[ 7].m_deltaRefPics[1][0] =  -2; c->m_GOPList[ 7].m_deltaRefPics[1][1] = -10; c->m_GOPList[ 7].m_deltaRefPics[1][2] = -26;

      c->m_GOPList[ 8].m_deltaRefPics[1][0] =  -1; c->m_GOPList[ 8].m_deltaRefPics[1][1] =  -3; c->m_GOPList[ 8].m_deltaRefPics[1][2] = -11; c->m_GOPList[ 8].m_deltaRefPics[1][3] = -27;
      c->m_GOPList[ 9].m_deltaRefPics[1][0] =  -1; c->m_GOPList[ 9].m_deltaRefPics[1][1] =  -9; c->m_GOPList[ 9].m_deltaRefPics[1][2] = -25;
      c->m_GOPList[10].m_deltaRefPics[1][0] =  -4; c->m_GOPList[10].m_deltaRefPics[1][1] = -20;
      c->m_GOPList[11].m_deltaRefPics[1][0] =  -2; c->m_GOPList[11].m_deltaRefPics[1][1] =  -6; c->m_GOPList[11].m_deltaRefPics[1][2] = -22;

      c->m_GOPList[12].m_deltaRefPics[1][0] =  -1; c->m_GOPList[12].m_deltaRefPics[1][1] =  -3; c->m_GOPList[12].m_deltaRefPics[1][2] =  -7; c->m_GOPList[12].m_deltaRefPics[1][3] = -23;
      c->m_GOPList[13].m_deltaRefPics[1][0] =  -1; c->m_GOPList[13].m_deltaRefPics[1][1] =  -5; c->m_GOPList[13].m_deltaRefPics[1][2] = -21;
      c->m_GOPList[14].m_deltaRefPics[1][0] =  -2; c->m_GOPList[14].m_deltaRefPics[1][1] = -18;
      c->m_GOPList[15].m_deltaRefPics[1][0] =  -1; c->m_GOPList[15].m_deltaRefPics[1][1] =  -3; c->m_GOPList[15].m_deltaRefPics[1][2] = -19;

      c->m_GOPList[16].m_deltaRefPics[1][0] =  -1; c->m_GOPList[16].m_deltaRefPics[1][1] = -17;
      c->m_GOPList[17].m_deltaRefPics[1][0] =  -8; c->m_GOPList[17].m_deltaRefPics[1][1] =   8;
      c->m_GOPList[18].m_deltaRefPics[1][0] =  -4; c->m_GOPList[18].m_deltaRefPics[1][1] = -12;
      c->m_GOPList[19].m_deltaRefPics[1][0] =  -2; c->m_GOPList[19].m_deltaRefPics[1][1] =  -6; c->m_GOPList[19].m_deltaRefPics[1][2] = -14;

      c->m_GOPList[20].m_deltaRefPics[1][0] =  -1; c->m_GOPList[20].m_deltaRefPics[1][1] =  -3; c->m_GOPList[20].m_deltaRefPics[1][2] =  -7; c->m_GOPList[20].m_deltaRefPics[1][3] = -15;
      c->m_GOPList[21].m_deltaRefPics[1][0] =  -1; c->m_GOPList[21].m_deltaRefPics[1][1] =  -5; c->m_GOPList[21].m_deltaRefPics[1][2] = -13;
      c->m_GOPList[22].m_deltaRefPics[1][0] =  -2; c->m_GOPList[22].m_deltaRefPics[1][1] = -10;
      c->m_GOPList[23].m_deltaRefPics[1][0] =  -1; c->m_GOPList[23].m_deltaRefPics[1][1] =  -3; c->m_GOPList[23].m_deltaRefPics[1][2] = -11;

      c->m_GOPList[24].m_deltaRefPics[1][0] =  -1; c->m_GOPList[24].m_deltaRefPics[1][1] =  -9;
      c->m_GOPList[25].m_deltaRefPics[1][0] =  -4; c->m_GOPList[25].m_deltaRefPics[1][1] =   4;
      c->m_GOPList[26].m_deltaRefPics[1][0] =  -2; c->m_GOPList[26].m_deltaRefPics[1][1] =  -6;
      c->m_GOPList[27].m_deltaRefPics[1][0] =  -1; c->m_GOPList[27].m_deltaRefPics[1][1] =  -3; c->m_GOPList[27].m_deltaRefPics[1][2] = -7;

      c->m_GOPList[28].m_deltaRefPics[1][0] =  -1; c->m_GOPList[28].m_deltaRefPics[1][1] =  -5;
      c->m_GOPList[29].m_deltaRefPics[1][0] =  -2; c->m_GOPList[29].m_deltaRefPics[1][1] =   2;
      c->m_GOPList[30].m_deltaRefPics[1][0] =  -1; c->m_GOPList[30].m_deltaRefPics[1][1] =  -3;
      c->m_GOPList[31].m_deltaRefPics[1][0] =  -1; c->m_GOPList[31].m_deltaRefPics[1][1] =   1;

      for( int i = 0; i < 32; i++ )
      {
        switch( c->m_GOPList[i].m_temporalId )
        {
        case 0: c->m_GOPList[i].m_QPOffset   = -1;
                c->m_GOPList[i].m_QPOffsetModelOffset = 0.0;
                c->m_GOPList[i].m_QPOffsetModelScale  = 0.0;
                break;
        case 1: c->m_GOPList[i].m_QPOffset   = 0;
                c->m_GOPList[i].m_QPOffsetModelOffset = -4.9309;
                c->m_GOPList[i].m_QPOffsetModelScale  =  0.2265;
                break;
        case 2: c->m_GOPList[i].m_QPOffset   = 0;
                c->m_GOPList[i].m_QPOffsetModelOffset = -4.5000;
                c->m_GOPList[i].m_QPOffsetModelScale  =  0.2353;
                break;
        case 3: c->m_GOPList[i].m_QPOffset   = 3;
                c->m_GOPList[i].m_QPOffsetModelOffset = -5.4095;
                c->m_GOPList[i].m_QPOffsetModelScale  =  0.2571;
                break;
        case 4: c->m_GOPList[i].m_QPOffset   = 5;
                c->m_GOPList[i].m_QPOffsetModelOffset = -4.4895;
                c->m_GOPList[i].m_QPOffsetModelScale  =  0.1947;
                break;
        case 5: c->m_GOPList[i].m_QPOffset   = 6;
                c->m_GOPList[i].m_QPOffsetModelOffset = -5.4429;
                c->m_GOPList[i].m_QPOffsetModelScale  =  0.2429;
                break;
        default: break;
        }
      }
    }
  }

  for (int i = 0; c->m_GOPList[i].m_POC != -1 && i < VVENC_MAX_GOP + 1; i++)
  {
    c->m_RPLList0[i].m_POC = c->m_RPLList1[i].m_POC = c->m_GOPList[i].m_POC;
    c->m_RPLList0[i].m_temporalId = c->m_RPLList1[i].m_temporalId = c->m_GOPList[i].m_temporalId;
    c->m_RPLList0[i].m_refPic = c->m_RPLList1[i].m_refPic = c->m_GOPList[i].m_refPic;
    c->m_RPLList0[i].m_sliceType = c->m_RPLList1[i].m_sliceType = c->m_GOPList[i].m_sliceType;

    c->m_RPLList0[i].m_numRefPicsActive = c->m_GOPList[i].m_numRefPicsActive[0];
    c->m_RPLList1[i].m_numRefPicsActive = c->m_GOPList[i].m_numRefPicsActive[1];
    c->m_RPLList0[i].m_numRefPics = c->m_GOPList[i].m_numRefPics[0];
    c->m_RPLList1[i].m_numRefPics = c->m_GOPList[i].m_numRefPics[1];
    c->m_RPLList0[i].m_ltrp_in_slice_header_flag = c->m_GOPList[i].m_ltrp_in_slice_header_flag;
    c->m_RPLList1[i].m_ltrp_in_slice_header_flag = c->m_GOPList[i].m_ltrp_in_slice_header_flag;

    for (int j = 0; j < c->m_GOPList[i].m_numRefPics[0]; j++)
      c->m_RPLList0[i].m_deltaRefPics[j] = c->m_GOPList[i].m_deltaRefPics[0][j];
    for (int j = 0; j < c->m_GOPList[i].m_numRefPics[1]; j++)
      c->m_RPLList1[i].m_deltaRefPics[j] = c->m_GOPList[i].m_deltaRefPics[1][j];
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
    int curGOP = (checkGOP - 1) % c->m_GOPSize;
    int curPOC = ((checkGOP - 1) / c->m_GOPSize)*c->m_GOPSize * multipleFactor + c->m_RPLList0[curGOP].m_POC;
    if (c->m_RPLList0[curGOP].m_POC < 0 || c->m_RPLList1[curGOP].m_POC < 0)
    {
      vvenc::msg(VVENC_WARNING, "\nError: found fewer Reference Picture Sets than GOPSize\n");
      errorGOP = true;
    }
    else
    {
      //check that all reference pictures are available, or have a POC < 0 meaning they might be available in the next GOP.
      bool beforeI = false;
      for (int i = 0; i< c->m_RPLList0[curGOP].m_numRefPics; i++)
      {
        int absPOC = curPOC - c->m_RPLList0[curGOP].m_deltaRefPics[i];
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
              for (int k = 0; k < c->m_GOPSize; k++)
              {
                if (absPOC % (c->m_GOPSize * multipleFactor) == c->m_RPLList0[k].m_POC % (c->m_GOPSize * multipleFactor))
                {
                  if (c->m_RPLList0[k].m_temporalId == c->m_RPLList0[curGOP].m_temporalId)
                  {
                    c->m_RPLList0[k].m_refPic = true;
                  }
                }
              }
            }
          }
          if (!found)
          {
            vvenc::msg(VVENC_WARNING, "\nError: ref pic %d is not available for GOP frame %d\n", c->m_RPLList0[curGOP].m_deltaRefPics[i], curGOP + 1);
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
          if (numOK == c->m_GOPSize)
          {
            verifiedGOP = true;
          }
        }
      }
      else
      {
        //create a new RPLEntry for this frame containing all the reference pictures that were available (POC > 0)
        c->m_RPLList0[c->m_GOPSize + extraRPLs] = c->m_RPLList0[curGOP];
        c->m_RPLList1[c->m_GOPSize + extraRPLs] = c->m_RPLList1[curGOP];
        int newRefs0 = 0;
        for (int i = 0; i< c->m_RPLList0[curGOP].m_numRefPics; i++)
        {
          int absPOC = curPOC - c->m_RPLList0[curGOP].m_deltaRefPics[i];
          if (absPOC >= 0)
          {
            c->m_RPLList0[c->m_GOPSize + extraRPLs].m_deltaRefPics[newRefs0] = c->m_RPLList0[curGOP].m_deltaRefPics[i];
            newRefs0++;
          }
        }
        int numPrefRefs0 = c->m_RPLList0[curGOP].m_numRefPicsActive;

        int newRefs1 = 0;
        for (int i = 0; i< c->m_RPLList1[curGOP].m_numRefPics; i++)
        {
          int absPOC = curPOC - c->m_RPLList1[curGOP].m_deltaRefPics[i];
          if (absPOC >= 0)
          {
            c->m_RPLList1[c->m_GOPSize + extraRPLs].m_deltaRefPics[newRefs1] = c->m_RPLList1[curGOP].m_deltaRefPics[i];
            newRefs1++;
          }
        }
        int numPrefRefs1 = c->m_RPLList1[curGOP].m_numRefPicsActive;

        for (int offset = -1; offset>-checkGOP; offset--)
        {
          //step backwards in coding order and include any extra available pictures we might find useful to replace the ones with POC < 0.
          int offGOP = (checkGOP - 1 + offset) % c->m_GOPSize;
          int offPOC = ((checkGOP - 1 + offset) / c->m_GOPSize)*(c->m_GOPSize * multipleFactor) + c->m_RPLList0[offGOP].m_POC;
          if (offPOC >= 0 && c->m_RPLList0[offGOP].m_temporalId <= c->m_RPLList0[curGOP].m_temporalId)
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
              if (c->m_RPLList0[c->m_GOPSize + extraRPLs].m_deltaRefPics[i] == curPOC - offPOC)
              {
                newRef = false;
              }
            }
            if (newRef)
            {
              int insertPoint = newRefs0;
              //this picture can be added, find appropriate place in list and insert it.
              if (c->m_RPLList0[offGOP].m_temporalId == c->m_RPLList0[curGOP].m_temporalId)
              {
                c->m_RPLList0[offGOP].m_refPic = true;
              }
              for (int j = 0; j<newRefs0; j++)
              {
                if (c->m_RPLList0[c->m_GOPSize + extraRPLs].m_deltaRefPics[j] > curPOC - offPOC && curPOC - offPOC > 0)
                {
                  insertPoint = j;
                  break;
                }
              }
              int prev = curPOC - offPOC;
              for (int j = insertPoint; j<newRefs0 + 1; j++)
              {
                int newPrev = c->m_RPLList0[c->m_GOPSize + extraRPLs].m_deltaRefPics[j];
                c->m_RPLList0[c->m_GOPSize + extraRPLs].m_deltaRefPics[j] = prev;
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
          int offGOP = (checkGOP - 1 + offset) % c->m_GOPSize;
          int offPOC = ((checkGOP - 1 + offset) / c->m_GOPSize)*(c->m_GOPSize * multipleFactor) + c->m_RPLList1[offGOP].m_POC;
          if (offPOC >= 0 && c->m_RPLList1[offGOP].m_temporalId <= c->m_RPLList1[curGOP].m_temporalId)
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
              if (c->m_RPLList1[c->m_GOPSize + extraRPLs].m_deltaRefPics[i] == curPOC - offPOC)
              {
                newRef = false;
              }
            }
            if (newRef)
            {
              int insertPoint = newRefs1;
              //this picture can be added, find appropriate place in list and insert it.
              if (c->m_RPLList1[offGOP].m_temporalId == c->m_RPLList1[curGOP].m_temporalId)
              {
                c->m_RPLList1[offGOP].m_refPic = true;
              }
              for (int j = 0; j<newRefs1; j++)
              {
                if (c->m_RPLList1[c->m_GOPSize + extraRPLs].m_deltaRefPics[j] > curPOC - offPOC && curPOC - offPOC > 0)
                {
                  insertPoint = j;
                  break;
                }
              }
              int prev = curPOC - offPOC;
              for (int j = insertPoint; j<newRefs1 + 1; j++)
              {
                int newPrev = c->m_RPLList1[c->m_GOPSize + extraRPLs].m_deltaRefPics[j];
                c->m_RPLList1[c->m_GOPSize + extraRPLs].m_deltaRefPics[j] = prev;
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

        c->m_RPLList0[c->m_GOPSize + extraRPLs].m_numRefPics = newRefs0;
        c->m_RPLList0[c->m_GOPSize + extraRPLs].m_numRefPicsActive = std::min(c->m_RPLList0[c->m_GOPSize + extraRPLs].m_numRefPics, c->m_RPLList0[c->m_GOPSize + extraRPLs].m_numRefPicsActive);
        c->m_RPLList1[c->m_GOPSize + extraRPLs].m_numRefPics = newRefs1;
        c->m_RPLList1[c->m_GOPSize + extraRPLs].m_numRefPicsActive = std::min(c->m_RPLList1[c->m_GOPSize + extraRPLs].m_numRefPics, c->m_RPLList1[c->m_GOPSize + extraRPLs].m_numRefPicsActive);
        curGOP = c->m_GOPSize + extraRPLs;
        extraRPLs++;
      }
      numRefs = 0;
      for (int i = 0; i< c->m_RPLList0[curGOP].m_numRefPics; i++)
      {
        int absPOC = curPOC - c->m_RPLList0[curGOP].m_deltaRefPics[i];
        if (absPOC >= 0)
        {
          refList[numRefs] = absPOC;
          numRefs++;
        }
      }
      for (int i = 0; i< c->m_RPLList1[curGOP].m_numRefPics; i++)
      {
        int absPOC = curPOC - c->m_RPLList1[curGOP].m_deltaRefPics[i];
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

  c->m_maxTempLayer = 1;

  for(int i=0; i < c->m_GOPSize; i++)
  {
    if(c->m_GOPList[i].m_temporalId >= c->m_maxTempLayer)
    {
      c->m_maxTempLayer = c->m_GOPList[i].m_temporalId+1;
    }
  }
  for(int i=0; i< VVENC_MAX_TLAYER; i++)
  {
    c->m_maxNumReorderPics[i] = 0;
    c->m_maxDecPicBuffering[i] = 1;
  }
  for(int i=0; i < c->m_GOPSize; i++)
  {
    int numRefPic = c->m_RPLList0[i].m_numRefPics;
    for (int tmp = 0; tmp < c->m_RPLList1[i].m_numRefPics; tmp++)
    {
      bool notSame = true;
      for (int jj = 0; notSame && jj < c->m_RPLList0[i].m_numRefPics; jj++)
      {
        if (c->m_RPLList1[i].m_deltaRefPics[tmp] == c->m_RPLList0[i].m_deltaRefPics[jj]) notSame = false;
      }
      if (notSame) numRefPic++;
    }
    if (numRefPic + 1 > c->m_maxDecPicBuffering[c->m_GOPList[i].m_temporalId])
    {
      c->m_maxDecPicBuffering[c->m_GOPList[i].m_temporalId] = numRefPic + 1;
    }
    int highestDecodingNumberWithLowerPOC = 0;
    for(int j=0; j < c->m_GOPSize; j++)
    {
      if(c->m_GOPList[j].m_POC <= c->m_GOPList[i].m_POC)
      {
        highestDecodingNumberWithLowerPOC = j;
      }
    }
    int numReorder = 0;
    for(int j=0; j<highestDecodingNumberWithLowerPOC; j++)
    {
      if(c->m_GOPList[j].m_temporalId <= c->m_GOPList[i].m_temporalId &&
          c->m_GOPList[j].m_POC > c->m_GOPList[i].m_POC)
      {
        numReorder++;
      }
    }
    if(numReorder > c->m_maxNumReorderPics[c->m_GOPList[i].m_temporalId])
    {
      c->m_maxNumReorderPics[c->m_GOPList[i].m_temporalId] = numReorder;
    }
  }

  for(int i=0; i < VVENC_MAX_TLAYER-1; i++)
  {
    // a lower layer can not have higher value of m_numReorderPics than a higher layer
    if( c->m_maxNumReorderPics[i+1] < c->m_maxNumReorderPics[i])
    {
      c->m_maxNumReorderPics[i+1] = c->m_maxNumReorderPics[i];
    }
    // the value of num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] - 1, inclusive
    if( c->m_maxNumReorderPics[i] > c->m_maxDecPicBuffering[i] - 1)
    {
      c->m_maxDecPicBuffering[i] = c->m_maxNumReorderPics[i] + 1;
    }
    // a lower layer can not have higher value of c->m_uiMaxDecPicBuffering than a higher layer
    if( c->m_maxDecPicBuffering[i+1] < c->m_maxDecPicBuffering[i])
    {
      c->m_maxDecPicBuffering[i+1] = c->m_maxDecPicBuffering[i];
    }
  }

  // the value of num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] -  1, inclusive
  if( c->m_maxNumReorderPics[VVENC_MAX_TLAYER-1] > c->m_maxDecPicBuffering[VVENC_MAX_TLAYER-1] - 1)
  {
    c->m_maxDecPicBuffering[VVENC_MAX_TLAYER-1] = c->m_maxNumReorderPics[VVENC_MAX_TLAYER-1] + 1;
  }

  if ( ! c->m_MMVD && c->m_allowDisFracMMVD )
  {
    vvenc::msg( VVENC_WARNING, "MMVD disabled, thus disable AllowDisFracMMVD too\n" );
    c->m_allowDisFracMMVD = false;
  }

  //
  // finalize initialization
  //


  // coding structure
  c->m_numRPLList0 = 0;
  for ( int i = 0; i < VVENC_MAX_GOP; i++ )
  {
    if ( c->m_RPLList0[ i ].m_POC != -1 )
      c->m_numRPLList0++;
  }
  c->m_numRPLList1 = 0;
  for ( int i = 0; i < VVENC_MAX_GOP; i++ )
  {
    if ( c->m_RPLList1[ i ].m_POC != -1 )
      c->m_numRPLList1++;
  }

  c->m_PROF &= bool(c->m_Affine);
  if (c->m_Affine > 1)
  {
    c->m_PROF = bool(c->m_Affine);
    c->m_AffineType = (c->m_Affine == 2) ? true : false;
  }

  // check char array and reset them, if they seems to be unset
  vvenc_checkCharArrayStr( c->m_decodeBitstreams[0], VVENC_MAX_STRING_LEN);
  vvenc_checkCharArrayStr( c->m_decodeBitstreams[1], VVENC_MAX_STRING_LEN);
  vvenc_checkCharArrayStr( c->m_traceRule, VVENC_MAX_STRING_LEN);
  vvenc_checkCharArrayStr( c->m_traceFile, VVENC_MAX_STRING_LEN);
  vvenc_checkCharArrayStr( c->m_summaryOutFilename, VVENC_MAX_STRING_LEN);
  vvenc_checkCharArrayStr( c->m_summaryPicFilenameBase, VVENC_MAX_STRING_LEN);

  c->m_configDone = true;

  c->m_confirmFailed = checkCfgParameter(c);

  return( c->m_confirmFailed );
}

static bool checkCfgParameter( vvenc_config *c )
{
  // run base check first
  vvenc_confirmParameter( c, c->m_profile == vvencProfile::VVENC_PROFILE_AUTO, "can not determin auto profile");
  vvenc_confirmParameter( c, (c->m_profile != vvencProfile::VVENC_MAIN_10 
                           && c->m_profile != vvencProfile::VVENC_MAIN_10_STILL_PICTURE
                           && c->m_profile != vvencProfile::VVENC_MAIN_10_444
                           && c->m_profile != vvencProfile::VVENC_MAIN_10_444_STILL_PICTURE
                           && c->m_profile != vvencProfile::VVENC_MULTILAYER_MAIN_10
                           && c->m_profile != vvencProfile::VVENC_MULTILAYER_MAIN_10_STILL_PICTURE
                           && c->m_profile != vvencProfile::  VVENC_MULTILAYER_MAIN_10_444
                           && c->m_profile != vvencProfile::VVENC_MULTILAYER_MAIN_10_444_STILL_PICTURE),
                              "unsupported profile. currently only supporting auto,main10,main10stillpicture");

  vvenc_confirmParameter( c, c->m_level   == vvencLevel::VVENC_LEVEL_AUTO, "can not determin level");

  vvenc_confirmParameter( c, c->m_fastInterSearchMode<VVENC_FASTINTERSEARCH_AUTO || c->m_fastInterSearchMode>VVENC_FASTINTERSEARCH_MODE3,    "Error: FastInterSearchMode parameter out of range" );
  vvenc_confirmParameter( c, c->m_motionEstimationSearchMethod < 0 || c->m_motionEstimationSearchMethod >= VVENC_MESEARCH_NUMBER_OF_METHODS, "Error: FastSearch parameter out of range" );
  vvenc_confirmParameter( c, c->m_motionEstimationSearchMethodSCC < 0 || c->m_motionEstimationSearchMethodSCC > 3,                           "Error: FastSearchSCC parameter out of range" );
  vvenc_confirmParameter( c, c->m_internChromaFormat > VVENC_CHROMA_420,                                                                     "Intern chroma format must be either 400, 420" );

  switch ( c->m_conformanceWindowMode)
  {
  case 0:
      break;
  case 1:
      // automatic padding to minimum CU size
      vvenc_confirmParameter( c, c->m_aiPad[0] % vvenc::SPS::getWinUnitX(c->m_internChromaFormat) != 0, "Error: picture width is not an integer multiple of the specified chroma subsampling" );
      vvenc_confirmParameter( c, c->m_aiPad[1] % vvenc::SPS::getWinUnitY(c->m_internChromaFormat) != 0, "Error: picture height is not an integer multiple of the specified chroma subsampling" );
      break;
  case 2:
      break;
  case 3:
      // conformance
      if ((c->m_confWinLeft == 0) && (c->m_confWinRight == 0) && (c->m_confWinTop == 0) && (c->m_confWinBottom == 0))
      {
        vvenc::msg( VVENC_ERROR, "Warning: Conformance window enabled, but all conformance window parameters set to zero\n");
      }
      if ((c->m_aiPad[1] != 0) || (c->m_aiPad[0]!=0))
      {
        vvenc::msg( VVENC_ERROR, "Warning: Conformance window enabled, padding parameters will be ignored\n");
      }
      break;
  }

  vvenc_confirmParameter( c, c->m_colourPrimaries < 0 || c->m_colourPrimaries > 12,                 "colourPrimaries must be in range 0 <= x <= 12" );
  vvenc_confirmParameter( c, c->m_transferCharacteristics < 0 || c->m_transferCharacteristics > 18, "transferCharacteristics must be in range 0 <= x <= 18" );
  vvenc_confirmParameter( c, c->m_matrixCoefficients < 0 || c->m_matrixCoefficients > 14,           "matrixCoefficients must be in range 0 <= x <= 14" );

  vvenc_confirmParameter( c, vvenc_getQpValsSize(c->m_qpInValsCb )  != vvenc_getQpValsSize(c->m_qpOutValsCb), "Chroma QP table for Cb is incomplete.");
  vvenc_confirmParameter( c, vvenc_getQpValsSize(c->m_qpInValsCr)   != vvenc_getQpValsSize(c->m_qpOutValsCr), "Chroma QP table for Cr is incomplete.");
  vvenc_confirmParameter( c, vvenc_getQpValsSize(c->m_qpInValsCbCr) != vvenc_getQpValsSize(c->m_qpOutValsCbCr), "Chroma QP table for CbCr is incomplete.");

  if ( c->m_confirmFailed )
  {
    return c->m_confirmFailed;
  }

  int qpBdOffsetC = 6 * (c->m_internalBitDepth[1] - 8);

  vvenc_confirmParameter( c,c->m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] < -26 - qpBdOffsetC || c->m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] > 36, "qpTableStartMinus26[0] is out of valid range of -26 -qpBdOffsetC to 36, inclusive.");
  vvenc_confirmParameter( c,c->m_qpInValsCb[0] != c->m_qpOutValsCb[0], "First qpInValCb value should be equal to first qpOutValCb value");
  for (int i = 0; i < vvenc_getQpValsSize(c->m_qpInValsCb) - 1; i++)
  {
    vvenc_confirmParameter( c,c->m_qpInValsCb[i]  < -qpBdOffsetC || c->m_qpInValsCb[i] > vvenc::MAX_QP, "Some entries cfg_qpInValCb are out of valid range of -qpBdOffsetC to 63, inclusive.");
    vvenc_confirmParameter( c,c->m_qpOutValsCb[i] < -qpBdOffsetC || c->m_qpOutValsCb[i] > vvenc::MAX_QP, "Some entries cfg_qpOutValCb are out of valid range of -qpBdOffsetC to 63, inclusive.");
  }
  if (!c->m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag)
  {
    vvenc_confirmParameter( c,c->m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] < -26 - qpBdOffsetC || c->m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] > 36, "qpTableStartMinus26[1] is out of valid range of -26 -qpBdOffsetC to 36, inclusive.");
    vvenc_confirmParameter( c,c->m_qpInValsCr[0] != c->m_qpOutValsCr[0], "First qpInValCr value should be equal to first qpOutValCr value");
    for (int i = 0; i < vvenc_getQpValsSize(c->m_qpInValsCr) - 1; i++)
    {
      vvenc_confirmParameter( c,c->m_qpInValsCr[i] < -qpBdOffsetC || c->m_qpInValsCr[i] > vvenc::MAX_QP, "Some entries cfg_qpInValCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      vvenc_confirmParameter( c,c->m_qpOutValsCr[i] < -qpBdOffsetC || c->m_qpOutValsCr[i] > vvenc::MAX_QP, "Some entries cfg_qpOutValCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
    }
    vvenc_confirmParameter( c,c->m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] < -26 - qpBdOffsetC || c->m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] > 36, "qpTableStartMinus26[2] is out of valid range of -26 -qpBdOffsetC to 36, inclusive.");
    vvenc_confirmParameter( c,c->m_qpInValsCbCr[0] != c->m_qpOutValsCbCr[0], "First qpInValCbCr value should be equal to first qpOutValCbCr value");
    for (int i = 0; i < vvenc_getQpValsSize(c->m_qpInValsCbCr) - 1; i++)
    {
      vvenc_confirmParameter( c,c->m_qpInValsCbCr[i]  < -qpBdOffsetC || c->m_qpInValsCbCr[i] > vvenc::MAX_QP, "Some entries cfg_qpInValCbCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
      vvenc_confirmParameter( c,c->m_qpOutValsCbCr[i] < -qpBdOffsetC || c->m_qpOutValsCbCr[i] > vvenc::MAX_QP, "Some entries cfg_qpOutValCbCr are out of valid range of -qpBdOffsetC to 63, inclusive.");
    }
  }

  //
  // do some check and set of parameters next
  //

  vvenc_confirmParameter( c, c->m_AccessUnitDelimiter < 0,   "AccessUnitDelimiter must be >= 0" );
  vvenc_confirmParameter( c, c->m_vuiParametersPresent < 0,  "vuiParametersPresent must be >= 0" );
  vvenc_confirmParameter( c, c->m_hrdParametersPresent < 0,  "hrdParametersPresent must be >= 0" );

  if( c->m_DepQuantEnabled )
  {
    vvenc_confirmParameter( c, !c->m_RDOQ || !c->m_useRDOQTS, "RDOQ and RDOQTS must be greater 0 if dependent quantization is enabled" );
    vvenc_confirmParameter( c, c->m_SignDataHidingEnabled, "SignHideFlag must be equal to 0 if dependent quantization is enabled" );
  }

  vvenc_confirmParameter( c, (c->m_MSBExtendedBitDepth[0] < c->m_inputBitDepth[0]), "MSB-extended bit depth for luma channel (--MSBExtendedBitDepth) must be greater than or equal to input bit depth for luma channel (--InputBitDepth)" );
  vvenc_confirmParameter( c, (c->m_MSBExtendedBitDepth[1] < c->m_inputBitDepth[1]), "MSB-extended bit depth for chroma channel (--MSBExtendedBitDepthC) must be greater than or equal to input bit depth for chroma channel (--InputBitDepthC)" );

  const uint32_t maxBitDepth=(c->m_internChromaFormat==VVENC_CHROMA_400) ? c->m_internalBitDepth[0] : std::max(c->m_internalBitDepth[0], c->m_internalBitDepth[1]);
  vvenc_confirmParameter( c,c->m_bitDepthConstraintValue<maxBitDepth, "The internalBitDepth must not be greater than the bitDepthConstraint value");

  vvenc_confirmParameter( c,c->m_bitDepthConstraintValue!=10, "BitDepthConstraint must be 8 for MAIN profile and 10 for MAIN10 profile.");
  vvenc_confirmParameter( c,c->m_intraOnlyConstraintFlag==true, "IntraOnlyConstraintFlag must be false for non main_RExt profiles.");

  // check range of parameters
  vvenc_confirmParameter( c, c->m_inputBitDepth[0  ] < 8,                                 "InputBitDepth must be at least 8" );
  vvenc_confirmParameter( c, c->m_inputBitDepth[1] < 8,                                   "InputBitDepthC must be at least 8" );

#if !RExt__HIGH_BIT_DEPTH_SUPPORT
  for (uint32_t channelType = 0; channelType < 2; channelType++)
  {
    vvenc_confirmParameter( c,(c->m_internalBitDepth[channelType] > 12) , "Model is not configured to support high enough internal accuracies - enable RExt__HIGH_BIT_DEPTH_SUPPORT to use increased precision internal data types etc...");
  }
#endif


  vvenc_confirmParameter( c, (c->m_HdrMode != VVENC_HDR_OFF && c->m_internalBitDepth[0] < 10 )     ,       "InternalBitDepth must be at least 10 bit for HDR");
  vvenc_confirmParameter( c, (c->m_HdrMode != VVENC_HDR_OFF && c->m_internChromaFormat != VVENC_CHROMA_420 ) ,"ChromaFormatIDC must be YCbCr 4:2:0 for HDR");
  vvenc_confirmParameter( c, (c->m_contentLightLevel[0] < 0 || c->m_contentLightLevel[0] > 10000),  "max content light level must 0 <= cll <= 10000 ");
  vvenc_confirmParameter( c, (c->m_contentLightLevel[1] < 0 || c->m_contentLightLevel[1] > 10000),  "max average content light level must 0 <= cll <= 10000 ");

  {
    bool outOfRGBRange = false;
    for( size_t i = 0; i < sizeof(c->m_masteringDisplay); i++ )
    {
      if( i < 8 && c->m_masteringDisplay[i] > 50000 )
      {
        outOfRGBRange = true; break;
      }
    }
    vvenc_confirmParameter( c, outOfRGBRange,  "mastering display colour volume RGB values must be in range 0 <= RGB <= 50000");
  }

  vvenc_confirmParameter( c, c->m_log2SaoOffsetScale[0]   > (c->m_internalBitDepth[0  ]<10?0:(c->m_internalBitDepth[0  ]-10)), "SaoLumaOffsetBitShift must be in the range of 0 to InternalBitDepth-10, inclusive");
  vvenc_confirmParameter( c, c->m_log2SaoOffsetScale[1] > (c->m_internalBitDepth[1]<10?0:(c->m_internalBitDepth[1]-10)), "SaoChromaOffsetBitShift must be in the range of 0 to InternalBitDepthC-10, inclusive");

  vvenc_confirmParameter( c, c->m_temporalSubsampleRatio < 1,                                               "Temporal subsample rate must be no less than 1" );
  vvenc_confirmParameter( c, c->m_framesToBeEncoded < c->m_switchPOC,                                          "debug POC out of range" );

  vvenc_confirmParameter( c, (c->m_IntraPeriod > 0 && c->m_IntraPeriod < c->m_GOPSize) || c->m_IntraPeriod == 0,     "Intra period must be more than GOP size, or -1 , not 0" );
  vvenc_confirmParameter( c, c->m_InputQueueSize < c->m_GOPSize ,                                              "Input queue size must be greater or equal to gop size" );
  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTF && c->m_InputQueueSize < c->m_GOPSize + vvenc::MCTF_ADD_QUEUE_DELAY , "Input queue size must be greater or equal to gop size + N frames for MCTF" );

  vvenc_confirmParameter( c, c->m_DecodingRefreshType < 0 || c->m_DecodingRefreshType > 5,                     "Decoding Refresh Type must be comprised between 0 and 5 included" );
  vvenc_confirmParameter( c, c->m_IntraPeriod > 0 && !(c->m_DecodingRefreshType==1 || c->m_DecodingRefreshType==2 || c->m_DecodingRefreshType==4 || c->m_DecodingRefreshType==5), "Only Decoding Refresh Type CRA for non low delay supported" );                  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  vvenc_confirmParameter( c, c->m_IntraPeriod < 0 && c->m_DecodingRefreshType !=0,                             "Only Decoding Refresh Type 0 for low delay supported" );

  vvenc_confirmParameter( c, c->m_QP < -6 * (c->m_internalBitDepth[0] - 8) || c->m_QP > vvenc::MAX_QP,                "QP exceeds supported range (-QpBDOffsety to 63)" );
  for( int comp = 0; comp < 3; comp++)
  {
    vvenc_confirmParameter( c, c->m_loopFilterBetaOffsetDiv2[comp] < -12 || c->m_loopFilterBetaOffsetDiv2[comp] > 12,          "Loop Filter Beta Offset div. 2 exceeds supported range (-12 to 12)" );
    vvenc_confirmParameter( c, c->m_loopFilterTcOffsetDiv2[comp] < -12 || c->m_loopFilterTcOffsetDiv2[comp] > 12,              "Loop Filter Tc Offset div. 2 exceeds supported range (-12 to 12)" );
  }
  vvenc_confirmParameter( c, c->m_SearchRange < 0 ,                                                         "Search Range must be more than 0" );
  vvenc_confirmParameter( c, c->m_bipredSearchRange < 0 ,                                                   "Bi-prediction refinement search range must be more than 0" );
  vvenc_confirmParameter( c, c->m_minSearchWindow < 0,                                                      "Minimum motion search window size for the adaptive window ME must be greater than or equal to 0" );

  vvenc_confirmParameter( c, c->m_vvencMCTF.numFrames != c->m_vvencMCTF.numStrength,            "MCTF parameter list sizes differ");
  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTFNumLeadFrames  < 0,                             "MCTF number of lead frames must be greater than or equal to 0" );
  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTFNumTrailFrames < 0,                             "MCTF number of trailing frames must be greater than or equal to 0" );
  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTFNumLeadFrames  > 0 && ! c->m_vvencMCTF.MCTF,                 "MCTF disabled but number of MCTF lead frames is given" );
  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTFNumTrailFrames > 0 && ! c->m_vvencMCTF.MCTF,                 "MCTF disabled but number of MCTF trailing frames is given" );
  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTFNumTrailFrames > 0 && c->m_framesToBeEncoded <= 0, "If number of MCTF trailing frames is given, the total number of frames to be encoded has to be set" );
  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTFSpeed < 0 || c->m_vvencMCTF.MCTFSpeed > 4 ,        "MCTFSpeed exceeds supported range (0..4)" );
  static const std::string errorSegLessRng = std::string( "When using segment parallel encoding more then " ) + static_cast< char >( VVENC_MCTF_RANGE + '0' ) + " frames have to be encoded";
  vvenc_confirmParameter( c, c->m_SegmentMode != VVENC_SEG_OFF && c->m_framesToBeEncoded < VVENC_MCTF_RANGE, errorSegLessRng.c_str() );

  if (c->m_lumaReshapeEnable)
  {
    vvenc_confirmParameter( c, c->m_reshapeSignalType < vvenc::RESHAPE_SIGNAL_SDR || c->m_reshapeSignalType > vvenc::RESHAPE_SIGNAL_HLG, "LMCSSignalType out of range" );
    vvenc_confirmParameter( c, c->m_updateCtrl < 0,    "Min. LMCS Update Control is 0");
    vvenc_confirmParameter( c, c->m_updateCtrl > 2,    "Max. LMCS Update Control is 2");
    vvenc_confirmParameter( c, c->m_adpOption < 0,     "Min. LMCS Adaptation Option is 0");
    vvenc_confirmParameter( c, c->m_adpOption > 4,     "Max. LMCS Adaptation Option is 4");
    vvenc_confirmParameter( c, c->m_initialCW < 0,     "Min. Initial Total Codeword is 0");
    vvenc_confirmParameter( c, c->m_initialCW > 1023,  "Max. Initial Total Codeword is 1023");
    vvenc_confirmParameter( c, c->m_LMCSOffset < -7,   "Min. LMCS Offset value is -7");
    vvenc_confirmParameter( c, c->m_LMCSOffset > 7,    "Max. LMCS Offset value is 7");
  }
  vvenc_confirmParameter( c, c->m_EDO && c->m_bLoopFilterDisable,             "no EDO support with LoopFilter disabled" );
  vvenc_confirmParameter( c, c->m_EDO < 0 || c->m_EDO > 2,                    "EDO out of range [0..2]" );
  vvenc_confirmParameter( c, c->m_TMVPModeId < 0 || c->m_TMVPModeId > 2,      "TMVPMode out of range [0..2]" );
  vvenc_confirmParameter( c, c->m_AMVRspeed < 0 || c->m_AMVRspeed > 7,        "AMVR/IMV out of range [0..7]" );
  vvenc_confirmParameter( c, c->m_Affine < 0 || c->m_Affine > 2,              "Affine out of range [0..2]" );
  vvenc_confirmParameter( c, c->m_MMVD < 0 || c->m_MMVD > 4,                  "MMVD out of range [0..4]" );
  vvenc_confirmParameter( c, c->m_SMVD < 0 || c->m_SMVD > 3,                  "SMVD out of range [0..3]" );
  vvenc_confirmParameter( c, c->m_Geo  < 0 || c->m_Geo  > 3,                  "Geo out of range [0..3]" );
  vvenc_confirmParameter( c, c->m_CIIP < 0 || c->m_CIIP > 3,                  "CIIP out of range [0..3]" );
  vvenc_confirmParameter( c, c->m_SBT  < 0 || c->m_SBT  > 3,                  "SBT out of range [0..3]" );
  vvenc_confirmParameter( c, c->m_LFNST< 0 || c->m_LFNST> 3,                  "LFNST out of range [0..3]" );
  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTF < 0 || c->m_vvencMCTF.MCTF > 2,  "MCTF out of range [0..2]" );
  vvenc_confirmParameter( c, c->m_ISP  < 0 || c->m_ISP > 3,                    "ISP out of range [0..3]" );
  vvenc_confirmParameter( c, c->m_TS   < 0 || c->m_TS > 2,                     "TS out of range [0..2]" );
  vvenc_confirmParameter( c, c->m_TSsize < 2 || c->m_TSsize > 5,               "TSsize out of range [2..5]" );
  vvenc_confirmParameter( c, c->m_useBDPCM < 0 || c->m_useBDPCM > 2,           "BDPCM out of range [0..2]");
  vvenc_confirmParameter( c, c->m_useBDPCM  && c->m_TS==0,                     "BDPCM cannot be used when transform skip is disabled" );
  vvenc_confirmParameter( c, c->m_useBDPCM==1  && c->m_TS==2,                  "BDPCM cannot be permanently used when transform skip is auto" );
  vvenc_confirmParameter( c, c->m_FastIntraTools <0 || c->m_FastIntraTools >2, "SpeedIntraTools out of range [0..2]");
  vvenc_confirmParameter( c, c->m_IBCMode < 0 ||  c->m_IBCMode > 2,            "IBC out of range [0..2]");
  vvenc_confirmParameter( c, c->m_IBCFastMethod < 0 ||  c->m_IBCFastMethod > 6,"IBCFastMethod out of range [0..6]");
  vvenc_confirmParameter( c, c->m_BCW < 0 || c->m_BCW > 2,                     "BCW out of range [0..2]");
  vvenc_confirmParameter( c, c->m_FIMMode < 0 || c->m_FIMMode > 4,             "FastInferMerge out of range [0..4]");
#if QTBTT_SPEED3
  vvenc_confirmParameter( c, c->m_qtbttSpeedUp < 0 || c->m_qtbttSpeedUp > 7,   "QtbttExtraFast out of range [0..7]");
#else
  vvenc_confirmParameter( c, c->m_qtbttSpeedUp < 0 || c->m_qtbttSpeedUp > 3,   "QtbttExtraFast out of range [0..3]");
#endif
#if FASTTT_TH
  vvenc_confirmParameter( c, c->m_fastTTSplit < 0 || c->m_fastTTSplit > 7,     "FastTTSplit out of range [0..7]");
#endif

  const int fimModeMap[] = { 0, 3, 19, 27, 29 };
  c->m_FastInferMerge = fimModeMap[ c->m_FIMMode ];
  if( 1 << ( c->m_FastInferMerge & 7 ) > c->m_GOPSize )
  {
    const int hbm = c->m_FastInferMerge >> 3;
    const int lbm = std::max<int>( 7, log2( c->m_GOPSize ) );
    c->m_FastInferMerge = ( hbm << 3 ) | lbm;
  }

#if QTBTT_SPEED3
  c->m_qtbttSpeedUpMode = (c->m_qtbttSpeedUp > 2) ? (c->m_qtbttSpeedUp - 2) : 0;
  const int QTBTSMModeMap[] = { 0, 1, 3, 4, 5, 7 };
  c->m_qtbttSpeedUpMode = QTBTSMModeMap[c->m_qtbttSpeedUpMode];
#endif
#if FASTTT_TH
  static const float TT_THRESHOLDS[7] = { 1.1f, 1.075f, 1.05f, 1.025f, 1.0f,  0.975f, 0.95f };
  c->m_fastTT_th = c->m_fastTTSplit ? TT_THRESHOLDS[c->m_fastTTSplit - 1] : 0;
#endif

  if( c->m_alf )
  {
    vvenc_confirmParameter( c, c->m_maxNumAlfAlternativesChroma < 1 || c->m_maxNumAlfAlternativesChroma > VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA, std::string( std::string( "The maximum number of ALF Chroma filter alternatives must be in the range (1-" ) + std::to_string( VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA ) + std::string( ", inclusive)" ) ).c_str() );
  }

  vvenc_confirmParameter( c, c->m_useFastMrg < 0 || c->m_useFastMrg > 2,   "FastMrg out of range [0..2]" );
  vvenc_confirmParameter( c, c->m_useFastMIP < 0 || c->m_useFastMIP > 4,   "FastMIP out of range [0..4]" );
  vvenc_confirmParameter( c, c->m_fastSubPel < 0 || c->m_fastSubPel > 2,   "FastSubPel out of range [0..2]" );
  vvenc_confirmParameter( c, c->m_useEarlyCU < 0 || c->m_useEarlyCU > 2,   "ECU out of range [0..2]" );

  vvenc_confirmParameter( c, c->m_RCTargetBitrate == 0 && c->m_RCNumPasses != 1, "Only single pass encoding supported, when rate control is disabled" );
  vvenc_confirmParameter( c, c->m_RCNumPasses < 1 || c->m_RCNumPasses > 2,       "Only one pass or two pass encoding supported" );
  vvenc_confirmParameter( c, c->m_RCNumPasses < 2 && c->m_RCPass > 1,            "Only one pass supported in single pass encoding" );
  vvenc_confirmParameter( c, c->m_RCPass != -1 && ( c->m_RCPass < 1 || c->m_RCPass > 2 ), "Invalid pass parameter, only -1, 1 or 2 supported" );
  vvenc_confirmParameter( c, c->m_RCTargetBitrate > 0 && c->m_maxParallelFrames > 4, "Up to 4 parallel frames supported with rate control" );

  vvenc_confirmParameter(c, !((c->m_level==VVENC_LEVEL1)
    || (c->m_level==VVENC_LEVEL2) || (c->m_level==VVENC_LEVEL2_1)
    || (c->m_level==VVENC_LEVEL3) || (c->m_level==VVENC_LEVEL3_1)
    || (c->m_level==VVENC_LEVEL4) || (c->m_level==VVENC_LEVEL4_1)
    || (c->m_level==VVENC_LEVEL5) || (c->m_level==VVENC_LEVEL5_1) || (c->m_level==VVENC_LEVEL5_2)
    || (c->m_level==VVENC_LEVEL6) || (c->m_level==VVENC_LEVEL6_1) || (c->m_level==VVENC_LEVEL6_2) || (c->m_level==VVENC_LEVEL6_3)
    || (c->m_level==VVENC_LEVEL15_5)), "invalid level selected");
  vvenc_confirmParameter(c, !((c->m_levelTier==VVENC_TIER_MAIN) || (c->m_levelTier==VVENC_TIER_HIGH)), "invalid tier selected");


  vvenc_confirmParameter( c, c->m_chromaCbQpOffset < -12,           "Min. Chroma Cb QP Offset is -12" );
  vvenc_confirmParameter( c, c->m_chromaCbQpOffset >  12,           "Max. Chroma Cb QP Offset is  12" );
  vvenc_confirmParameter( c, c->m_chromaCrQpOffset < -12,           "Min. Chroma Cr QP Offset is -12" );
  vvenc_confirmParameter( c, c->m_chromaCrQpOffset >  12,           "Max. Chroma Cr QP Offset is  12" );
  vvenc_confirmParameter( c, c->m_chromaCbQpOffsetDualTree < -12,   "Min. Chroma Cb QP Offset for dual tree is -12" );
  vvenc_confirmParameter( c, c->m_chromaCbQpOffsetDualTree >  12,   "Max. Chroma Cb QP Offset for dual tree is  12" );
  vvenc_confirmParameter( c, c->m_chromaCrQpOffsetDualTree < -12,   "Min. Chroma Cr QP Offset for dual tree is -12" );
  vvenc_confirmParameter( c, c->m_chromaCrQpOffsetDualTree >  12,   "Max. Chroma Cr QP Offset for dual tree is  12" );

  if ( c->m_JointCbCrMode )
  {
    vvenc_confirmParameter( c, c->m_chromaCbCrQpOffset < -12, "Min. Joint Cb-Cr QP Offset is -12");
    vvenc_confirmParameter( c, c->m_chromaCbCrQpOffset >  12, "Max. Joint Cb-Cr QP Offset is  12");
    vvenc_confirmParameter( c, c->m_chromaCbCrQpOffsetDualTree < -12, "Min. Joint Cb-Cr QP Offset for dual tree is -12");
    vvenc_confirmParameter( c, c->m_chromaCbCrQpOffsetDualTree >  12, "Max. Joint Cb-Cr QP Offset for dual tree is  12");
  }

  if (c->m_usePerceptQPA && c->m_dualITree && (c->m_internChromaFormat != VVENC_CHROMA_400) && (c->m_chromaCbQpOffsetDualTree != 0 || c->m_chromaCrQpOffsetDualTree != 0 || c->m_chromaCbCrQpOffsetDualTree != 0))
  {
    vvenc::msg(VVENC_WARNING, "***************************************************************************\n");
    vvenc::msg(VVENC_WARNING, "** WARNING: chroma QPA on, ignoring nonzero dual-tree chroma QP offsets! **\n");
    vvenc::msg(VVENC_WARNING, "***************************************************************************\n");
  }

  vvenc_confirmParameter(c, c->m_usePerceptQPATempFiltISlice > 2,                                                    "PerceptQPATempFiltIPic out of range, must be 2 or less" );
  vvenc_confirmParameter(c, c->m_usePerceptQPATempFiltISlice > 0 && c->m_vvencMCTF.MCTF == 0,                        "PerceptQPATempFiltIPic must be turned off when MCTF is off" );

  vvenc_confirmParameter(c, c->m_usePerceptQPA && (c->m_cuQpDeltaSubdiv > 2),                                     "MaxCuDQPSubdiv must be 2 or smaller when PerceptQPA is on" );

  vvenc_confirmParameter(c, c->m_MaxCodingDepth > vvenc::MAX_CU_DEPTH,                                                      "MaxPartitionDepth exceeds predefined MAX_CU_DEPTH limit");
  vvenc_confirmParameter(c, c->m_MinQT[0] < 1<<vvenc::MIN_CU_LOG2,                                                          "Minimum QT size should be larger than or equal to 4");
  vvenc_confirmParameter(c, c->m_MinQT[1] < 1<<vvenc::MIN_CU_LOG2,                                                          "Minimum QT size should be larger than or equal to 4");
  vvenc_confirmParameter(c, c->m_CTUSize < 32,                                                                       "CTUSize must be greater than or equal to 32");
  vvenc_confirmParameter(c, c->m_CTUSize > 128,                                                                      "CTUSize must be less than or equal to 128");
  vvenc_confirmParameter(c, c->m_CTUSize != 32 && c->m_CTUSize != 64 && c->m_CTUSize != 128,                               "CTUSize must be a power of 2 (32, 64, or 128)");
  vvenc_confirmParameter(c, c->m_MaxCodingDepth < 1,                                                                 "MaxPartitionDepth must be greater than zero");
  vvenc_confirmParameter(c, (c->m_CTUSize  >> ( c->m_MaxCodingDepth - 1 ) ) < 8,                                        "Minimum partition width size should be larger than or equal to 8");
  vvenc_confirmParameter(c, (c->m_PadSourceWidth  % std::max( 8, int(c->m_CTUSize  >> ( c->m_MaxCodingDepth - 1 )))) != 0, "Resulting coded frame width must be a multiple of Max(8, the minimum CU size)");
  vvenc_confirmParameter(c, c->m_log2MaxTbSize > 6,                                                                  "Log2MaxTbSize must be 6 or smaller." );
  vvenc_confirmParameter(c, c->m_log2MaxTbSize < 5,                                                                  "Log2MaxTbSize must be 5 or greater." );

  vvenc_confirmParameter( c, c->m_log2MinCodingBlockSize < 2,                                                         "Log2MinCodingBlockSize must be 2 or greater." );
  vvenc_confirmParameter( c, c->m_CTUSize < ( 1 << c->m_log2MinCodingBlockSize ),                                        "Log2MinCodingBlockSize must be smaller than max CTU size." );
  vvenc_confirmParameter( c, c->m_MinQT[ 0 ] < ( 1 << c->m_log2MinCodingBlockSize ),                                     "Log2MinCodingBlockSize must be greater than min QT size for I slices" );
  vvenc_confirmParameter( c, c->m_MinQT[ 1 ] < ( 1 << c->m_log2MinCodingBlockSize ),                                     "Log2MinCodingBlockSize must be greater than min QT size for non I slices" );

  const int chromaScaleX = ( (c->m_internChromaFormat==VVENC_CHROMA_444) ) ? 0 : 1;
  vvenc_confirmParameter( c, ( c->m_MinQT[ 2 ] << chromaScaleX ) < ( 1 << c->m_log2MinCodingBlockSize ), "Log2MinCodingBlockSize must be greater than min chroma QT size for I slices" );

  vvenc_confirmParameter(c, c->m_PadSourceWidth  % vvenc::SPS::getWinUnitX(c->m_internChromaFormat) != 0, "Picture width must be an integer multiple of the specified chroma subsampling");
  vvenc_confirmParameter(c, c->m_PadSourceHeight % vvenc::SPS::getWinUnitY(c->m_internChromaFormat) != 0, "Picture height must be an integer multiple of the specified chroma subsampling");

  vvenc_confirmParameter(c, c->m_aiPad[0] % vvenc::SPS::getWinUnitX(c->m_internChromaFormat) != 0, "Horizontal padding must be an integer multiple of the specified chroma subsampling");
  vvenc_confirmParameter(c, c->m_aiPad[1] % vvenc::SPS::getWinUnitY(c->m_internChromaFormat) != 0, "Vertical padding must be an integer multiple of the specified chroma subsampling");

  vvenc_confirmParameter(c, c->m_confWinLeft   % vvenc::SPS::getWinUnitX(c->m_internChromaFormat) != 0, "Left conformance window offset must be an integer multiple of the specified chroma subsampling");
  vvenc_confirmParameter(c, c->m_confWinRight  % vvenc::SPS::getWinUnitX(c->m_internChromaFormat) != 0, "Right conformance window offset must be an integer multiple of the specified chroma subsampling");
  vvenc_confirmParameter(c, c->m_confWinTop    % vvenc::SPS::getWinUnitY(c->m_internChromaFormat) != 0, "Top conformance window offset must be an integer multiple of the specified chroma subsampling");
  vvenc_confirmParameter(c, c->m_confWinBottom % vvenc::SPS::getWinUnitY(c->m_internChromaFormat) != 0, "Bottom conformance window offset must be an integer multiple of the specified chroma subsampling");

  vvenc_confirmParameter(c, c->m_numThreads < 0,                                                  "NumThreads out of range" );
  vvenc_confirmParameter(c, c->m_ensureWppBitEqual < 0       || c->m_ensureWppBitEqual > 1,       "WppBitEqual out of range (0,1)");
  vvenc_confirmParameter(c, c->m_useAMaxBT < 0               || c->m_useAMaxBT > 1,               "AMaxBT out of range (0,1)");
  vvenc_confirmParameter(c, c->m_cabacInitPresent < 0        || c->m_cabacInitPresent > 1,        "CabacInitPresent out of range (0,1)");
  vvenc_confirmParameter(c, c->m_alfTempPred < 0             || c->m_alfTempPred > 1,             "ALFTempPred out of range (0,1)");
  vvenc_confirmParameter(c, c->m_alfSpeed < 0                || c->m_alfSpeed > 1,                "ALFSpeed out of range (0,1)");
  vvenc_confirmParameter(c, c->m_maxTempLayer > 1 && c->m_maxTempLayer - c->m_alfSpeed <= 0,      "ALFSpeed disables ALF for this temporal configuration. Disable ALF if intended, or turn off ALFSpeed!");
  vvenc_confirmParameter(c, c->m_saoEncodingRate < 0.0       || c->m_saoEncodingRate > 1.0,       "SaoEncodingRate out of range [0.0 .. 1.0]");
  vvenc_confirmParameter(c, c->m_saoEncodingRateChroma < 0.0 || c->m_saoEncodingRateChroma > 1.0, "SaoEncodingRateChroma out of range [0.0 .. 1.0]");
  vvenc_confirmParameter(c, c->m_maxParallelFrames < 0,                                           "MaxParallelFrames out of range" );

  vvenc_confirmParameter(c, c->m_numThreads > 0 && c->m_ensureWppBitEqual == 0, "NumThreads > 0 requires WppBitEqual > 0");

  if( c->m_maxParallelFrames )
  {
    vvenc_confirmParameter(c, c->m_numThreads == 0,       "For frame parallel processing NumThreads > 0 is required" );
    vvenc_confirmParameter(c, c->m_useAMaxBT,             "Frame parallel processing: AMaxBT is not supported (must be disabled)" );
    vvenc_confirmParameter(c, c->m_cabacInitPresent,      "Frame parallel processing: CabacInitPresent is not supported (must be disabled)" );
    vvenc_confirmParameter(c, c->m_saoEncodingRate > 0.0, "Frame parallel processing: SaoEncodingRate is not supported (must be disabled)" );
    vvenc_confirmParameter(c, c->m_alfTempPred,           "Frame parallel processing: ALFTempPred is not supported (must be disabled)" );
#if ENABLE_TRACING
    vvenc_confirmParameter(c, c->m_traceFile[0] != '\0' && c->m_maxParallelFrames > 1, "Tracing and frame parallel encoding not supported" );
#endif
    vvenc_confirmParameter(c, c->m_maxParallelFrames > c->m_InputQueueSize, "Max parallel frames should be less than size of input queue" );
  }

  vvenc_confirmParameter(c,((c->m_PadSourceWidth) & 7) != 0, "internal picture width must be a multiple of 8 - check cropping options");
  vvenc_confirmParameter(c,((c->m_PadSourceHeight) & 7) != 0, "internal picture height must be a multiple of 8 - check cropping options");


  vvenc_confirmParameter(c, c->m_maxNumMergeCand < 1,                              "MaxNumMergeCand must be 1 or greater.");
  vvenc_confirmParameter(c, c->m_maxNumMergeCand > vvenc::MRG_MAX_NUM_CANDS,              "MaxNumMergeCand must be no more than MRG_MAX_NUM_CANDS." );
  vvenc_confirmParameter(c, c->m_maxNumGeoCand > vvenc::GEO_MAX_NUM_UNI_CANDS,            "MaxNumGeoCand must be no more than GEO_MAX_NUM_UNI_CANDS." );
  vvenc_confirmParameter(c, c->m_maxNumGeoCand > c->m_maxNumMergeCand,                "MaxNumGeoCand must be no more than MaxNumMergeCand." );
  vvenc_confirmParameter(c, 0 < c->m_maxNumGeoCand && c->m_maxNumGeoCand < 2,         "MaxNumGeoCand must be no less than 2 unless MaxNumGeoCand is 0." );
  vvenc_confirmParameter(c, c->m_maxNumAffineMergeCand < (c->m_SbTMVP ? 1 : 0),       "MaxNumAffineMergeCand must be greater than 0 when SbTMVP is enabled");
  vvenc_confirmParameter(c, c->m_maxNumAffineMergeCand > vvenc::AFFINE_MRG_MAX_NUM_CANDS, "MaxNumAffineMergeCand must be no more than AFFINE_MRG_MAX_NUM_CANDS." );


  vvenc_confirmParameter(c, (c->m_hrdParametersPresent>0) && (0 == c->m_RCTargetBitrate),  "HrdParametersPresent requires RateControl enabled");
  vvenc_confirmParameter(c, c->m_bufferingPeriodSEIEnabled && (c->m_hrdParametersPresent<1), "BufferingPeriodSEI requires HrdParametersPresent enabled");
  vvenc_confirmParameter(c, c->m_pictureTimingSEIEnabled && (c->m_hrdParametersPresent<1),   "PictureTimingSEI requires HrdParametersPresent enabled");

  // max CU width and height should be power of 2
  uint32_t ui = c->m_CTUSize;
  while(ui)
  {
    ui >>= 1;
    if( (ui & 1) == 1)
    {
      vvenc_confirmParameter(c, ui != 1 , "CTU Size should be 2^n");
    }
  }

  if ( c->m_IntraPeriod == 1 && c->m_GOPList[0].m_POC == -1 )
  {
  }
  else
  {
    vvenc_confirmParameter(c,  c->m_intraOnlyConstraintFlag, "IntraOnlyConstraintFlag cannot be 1 for inter sequences");
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
  vvenc_confirmParameter(c,  c->m_IntraPeriod >=0&&(c->m_IntraPeriod%c->m_GOPSize!=0), "Intra period must be a multiple of GOPSize, or -1" );
  vvenc_confirmParameter(c,  c->m_temporalSubsampleRatio < 1, "TemporalSubsampleRatio must be greater than 0");

  for(int i=0; i<c->m_GOPSize; i++)
  {
    if (c->m_GOPList[i].m_POC == c->m_GOPSize * multipleFactor)
    {
      vvenc_confirmParameter(c,  c->m_GOPList[i].m_temporalId!=0 , "The last frame in each GOP must have temporal ID = 0 " );
    }
  }

  if ( (c->m_IntraPeriod != 1) && !c->m_loopFilterOffsetInPPS && (!c->m_bLoopFilterDisable) )
  {
    for(int i=0; i<c->m_GOPSize; i++)
    {
      for( int comp = 0; comp < 3; comp++ )
      {
        vvenc_confirmParameter(c,  (c->m_GOPList[i].m_betaOffsetDiv2 + c->m_loopFilterBetaOffsetDiv2[comp]) < -12 || (c->m_GOPList[i].m_betaOffsetDiv2 + c->m_loopFilterBetaOffsetDiv2[comp]) > 12, "Loop Filter Beta Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)" );
        vvenc_confirmParameter(c,  (c->m_GOPList[i].m_tcOffsetDiv2 + c->m_loopFilterTcOffsetDiv2[comp]) < -12 || (c->m_GOPList[i].m_tcOffsetDiv2 + c->m_loopFilterTcOffsetDiv2[comp]) > 12, "Loop Filter Tc Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)" );
      }
    }
  }

  for(int i=0; i<c->m_GOPSize; i++)
  {
    vvenc_confirmParameter(c, abs(c->m_GOPList[i].m_CbQPoffset               ) > 12, "Cb QP Offset for one of the GOP entries exceeds supported range (-12 to 12)" );
    vvenc_confirmParameter(c, abs(c->m_GOPList[i].m_CbQPoffset + c->m_chromaCbQpOffset) > 12, "Cb QP Offset for one of the GOP entries, when combined with the PPS Cb offset, exceeds supported range (-12 to 12)" );
    vvenc_confirmParameter(c, abs(c->m_GOPList[i].m_CrQPoffset               ) > 12, "Cr QP Offset for one of the GOP entries exceeds supported range (-12 to 12)" );
    vvenc_confirmParameter(c, abs(c->m_GOPList[i].m_CrQPoffset + c->m_chromaCrQpOffset) > 12, "Cr QP Offset for one of the GOP entries, when combined with the PPS Cr offset, exceeds supported range (-12 to 12)" );
  }
  vvenc_confirmParameter(c, abs(c->m_sliceChromaQpOffsetIntraOrPeriodic[0]                 ) > 12, "Intra/periodic Cb QP Offset exceeds supported range (-12 to 12)" );
  vvenc_confirmParameter(c, abs(c->m_sliceChromaQpOffsetIntraOrPeriodic[0]  + c->m_chromaCbQpOffset ) > 12, "Intra/periodic Cb QP Offset, when combined with the PPS Cb offset, exceeds supported range (-12 to 12)" );
  vvenc_confirmParameter(c, abs(c->m_sliceChromaQpOffsetIntraOrPeriodic[1]                 ) > 12, "Intra/periodic Cr QP Offset exceeds supported range (-12 to 12)" );
  vvenc_confirmParameter(c, abs(c->m_sliceChromaQpOffsetIntraOrPeriodic[1]  + c->m_chromaCrQpOffset ) > 12, "Intra/periodic Cr QP Offset, when combined with the PPS Cr offset, exceeds supported range (-12 to 12)" );

  vvenc_confirmParameter(c, c->m_fastLocalDualTreeMode < 0 || c->m_fastLocalDualTreeMode > 2, "FastLocalDualTreeMode must be in range [0..2]" );

  int extraRPLs = 0;
  int numRefs   = 1;
  //start looping through frames in coding order until we can verify that the GOP structure is correct.
  while (!verifiedGOP && !errorGOP)
  {
    int curGOP = (checkGOP - 1) % c->m_GOPSize;
    int curPOC = ((checkGOP - 1) / c->m_GOPSize)*c->m_GOPSize * multipleFactor + c->m_RPLList0[curGOP].m_POC;
    if (c->m_RPLList0[curGOP].m_POC < 0 || c->m_RPLList1[curGOP].m_POC < 0)
    {
      vvenc::msg(VVENC_WARNING, "\nError: found fewer Reference Picture Sets than GOPSize\n");
      errorGOP = true;
    }
    else
    {
      //check that all reference pictures are available, or have a POC < 0 meaning they might be available in the next GOP.
      bool beforeI = false;
      for (int i = 0; i< c->m_RPLList0[curGOP].m_numRefPics; i++)
      {
        int absPOC = curPOC - c->m_RPLList0[curGOP].m_deltaRefPics[i];
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
            vvenc::msg(VVENC_WARNING, "\nError: ref pic %d is not available for GOP frame %d\n", c->m_RPLList0[curGOP].m_deltaRefPics[i], curGOP + 1);
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
          if (numOK == c->m_GOPSize)
          {
            verifiedGOP = true;
          }
        }
      }
      else
      {
        //create a new RPLEntry for this frame containing all the reference pictures that were available (POC > 0)
        int newRefs0 = 0;
        for (int i = 0; i< c->m_RPLList0[curGOP].m_numRefPics; i++)
        {
          int absPOC = curPOC - c->m_RPLList0[curGOP].m_deltaRefPics[i];
          if (absPOC >= 0)
          {
            newRefs0++;
          }
        }
        int numPrefRefs0 = c->m_RPLList0[curGOP].m_numRefPicsActive;

        int newRefs1 = 0;
        for (int i = 0; i< c->m_RPLList1[curGOP].m_numRefPics; i++)
        {
          int absPOC = curPOC - c->m_RPLList1[curGOP].m_deltaRefPics[i];
          if (absPOC >= 0)
          {
            newRefs1++;
          }
        }
        int numPrefRefs1 = c->m_RPLList1[curGOP].m_numRefPicsActive;

        for (int offset = -1; offset>-checkGOP; offset--)
        {
          //step backwards in coding order and include any extra available pictures we might find useful to replace the ones with POC < 0.
          int offGOP = (checkGOP - 1 + offset) % c->m_GOPSize;
          int offPOC = ((checkGOP - 1 + offset) / c->m_GOPSize)*(c->m_GOPSize * multipleFactor) + c->m_RPLList0[offGOP].m_POC;
          if (offPOC >= 0 && c->m_RPLList0[offGOP].m_temporalId <= c->m_RPLList0[curGOP].m_temporalId)
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
              if (c->m_RPLList0[c->m_GOPSize + extraRPLs].m_deltaRefPics[i] == curPOC - offPOC)
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
          int offGOP = (checkGOP - 1 + offset) % c->m_GOPSize;
          int offPOC = ((checkGOP - 1 + offset) / c->m_GOPSize)*(c->m_GOPSize * multipleFactor) + c->m_RPLList1[offGOP].m_POC;
          if (offPOC >= 0 && c->m_RPLList1[offGOP].m_temporalId <= c->m_RPLList1[curGOP].m_temporalId)
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
              if (c->m_RPLList1[c->m_GOPSize + extraRPLs].m_deltaRefPics[i] == curPOC - offPOC)
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

        curGOP = c->m_GOPSize + extraRPLs;
        extraRPLs++;
      }
      numRefs = 0;
      for (int i = 0; i< c->m_RPLList0[curGOP].m_numRefPics; i++)
      {
        int absPOC = curPOC - c->m_RPLList0[curGOP].m_deltaRefPics[i];
        if (absPOC >= 0)
        {
          refList[numRefs] = absPOC;
          numRefs++;
        }
      }
      for (int i = 0; i< c->m_RPLList1[curGOP].m_numRefPics; i++)
      {
        int absPOC = curPOC - c->m_RPLList1[curGOP].m_deltaRefPics[i];
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
  vvenc_confirmParameter(c, errorGOP, "Invalid GOP structure given");

  for(int i=0; i<c->m_GOPSize; i++)
  {
    vvenc_confirmParameter(c, c->m_GOPList[i].m_sliceType!='B' && c->m_GOPList[i].m_sliceType!='P' && c->m_GOPList[i].m_sliceType!='I', "Slice type must be equal to B or P or I");
  }

  vvenc_confirmParameter(c,  c->m_vvencMCTF.MCTF > 2 || c->m_vvencMCTF.MCTF < 0, "MCTF out of range" );

  if( c->m_vvencMCTF.MCTF )
  {
    if( c->m_vvencMCTF.MCTFFrames[0] == 0 )
    {
      vvenc::msg( VVENC_WARNING, "no MCTF frames selected, MCTF will be inactive!\n");
    }

    vvenc_confirmParameter(c, c->m_vvencMCTF.numFrames != c->m_vvencMCTF.numStrength, "MCTFFrames and MCTFStrengths do not match");
  }

  if( c->m_fastForwardToPOC != -1 )
  {
    if( c->m_cabacInitPresent )  { vvenc::msg( VVENC_WARNING, "WARNING usage of FastForwardToPOC and CabacInitPresent might cause different behaviour\n\n" ); }
    if( c->m_alf )               { vvenc::msg( VVENC_WARNING, "WARNING usage of FastForwardToPOC and ALF might cause different behaviour\n\n" ); }
  }

  if( c->m_picPartitionFlag )
  {
    checkCfgPicPartitioningParameter( c );
  }

  return( c->m_confirmFailed );
}

static void checkCfgPicPartitioningParameter( vvenc_config *c )
{
  vvenc::PPS pps;

  pps.picWidthInLumaSamples  = c->m_SourceWidth;
  pps.picHeightInLumaSamples = c->m_SourceHeight;
  pps.log2CtuSize            = vvenc::ceilLog2( c->m_CTUSize );
  pps.picWidthInCtu          = ( pps.picWidthInLumaSamples + c->m_CTUSize - 1 ) / c->m_CTUSize;
  pps.picHeightInCtu         = ( pps.picHeightInLumaSamples + c->m_CTUSize - 1 ) / c->m_CTUSize;

  int i, numTileColumnWidths, numTileRowHeights;

  // set default tile column if not provided
  if( c->m_tileColumnWidth[0] == 0 )
  {
    c->m_tileColumnWidth[0] = pps.picWidthInCtu;
  }
  // set default tile row if not provided
  if( c->m_tileRowHeight[0] == 0 )
  {
    c->m_tileRowHeight[0] = pps.picHeightInCtu;
  }

  // remove any tile columns that can be specified implicitly
  if( c->m_tileColumnWidth[1] > 0 )
  {
    i = 9;
    while( i > 0 && c->m_tileColumnWidth[i-1] == c->m_tileColumnWidth[i] )
    {
      c->m_tileColumnWidth[i] = 0;
      i--;
    }
    numTileColumnWidths = i;
  }
  else
  {
    numTileColumnWidths = 1;
  }

  // remove any tile rows that can be specified implicitly
  if( c->m_tileRowHeight[1] > 0 )
  {
    i = 9;
    while( i > 0 && c->m_tileRowHeight[i-1] == c->m_tileRowHeight[i] )
    {
      c->m_tileRowHeight[i] = 0;
      i--;
    }
    numTileRowHeights = i;
  }
  else
  {
    numTileRowHeights = 1;
  }

  // setup tiles in temporary PPS structure
  uint32_t remSize = pps.picWidthInCtu;
  int colIdx;
  for( colIdx=0; remSize > 0 && colIdx < numTileColumnWidths; colIdx++ )
  {
    vvenc_confirmParameter( c, c->m_tileColumnWidth[ colIdx ] == 0, "Tile column widths cannot be equal to 0" );
    c->m_tileColumnWidth[ colIdx ] = std::min( remSize, c->m_tileColumnWidth[ colIdx ]);
    pps.tileColWidth.push_back( c->m_tileColumnWidth[ colIdx ] );
    remSize -= c->m_tileColumnWidth[ colIdx ];
  }
  if( colIdx < numTileColumnWidths && remSize == 0 )
  {
    vvenc_confirmParameter( c, true, "Explicitly given tile column widths exceed picture width" );
    return;
  }
  pps.numExpTileCols  = numTileColumnWidths;
  c->m_numExpTileCols = numTileColumnWidths;
  remSize = pps.picHeightInCtu;
  int rowIdx;
  for( rowIdx=0; remSize > 0 && rowIdx < numTileRowHeights; rowIdx++ )
  {
    vvenc_confirmParameter( c, c->m_tileRowHeight[ rowIdx ] == 0, "Tile row heights cannot be equal to 0" );
    c->m_tileRowHeight[ rowIdx ] = std::min( remSize, c->m_tileRowHeight[ rowIdx ]);
    pps.tileRowHeight.push_back( c->m_tileRowHeight[ rowIdx ] );
    remSize -= c->m_tileRowHeight[ rowIdx ];
  }
  if( rowIdx < numTileRowHeights && remSize == 0 )
  {
    vvenc_confirmParameter( c, true, "Explicitly given tile row heights exceed picture width" );
    return;
  }
  pps.numExpTileRows  = numTileRowHeights;
  c->m_numExpTileRows = numTileRowHeights;
  pps.initTiles();

  uint32_t maxTileCols, maxTileRows;
  vvenc::LevelTierFeatures::getMaxTileColsRowsPerLevel( c->m_level, maxTileCols, maxTileRows );
  vvenc_confirmParameter( c, pps.numTileCols > maxTileCols, "Number of tile columns exceeds maximum number allowed according to specified level" );
  vvenc_confirmParameter( c, pps.numTileRows > maxTileRows, "Number of tile rows exceeds maximum number allowed according to specified level" );
  c->m_numTileCols = pps.numTileCols;
  c->m_numTileRows = pps.numTileRows;

  vvenc_confirmParameter( c, c->m_numThreads > 0 && c->m_bDisableLFCrossTileBoundaryFlag, "Multiple tiles and disabling loppfilter across boundaries doesn't work mulit-threaded yet" );
}

VVENC_DECL int vvenc_init_default( vvenc_config *c, int width, int height, int framerate, int targetbitrate, int qp, vvencPresetMode preset )
{
  int iRet = 0;
  vvenc_config_default(c);
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

  iRet = vvenc_init_preset(c, preset );
  return iRet;
}

VVENC_DECL int vvenc_init_preset( vvenc_config *c, vvencPresetMode preset )
{
  memset(&c->m_qpInValsCb ,0, sizeof(c->m_qpInValsCb));
  memset(&c->m_qpOutValsCb,0, sizeof(c->m_qpOutValsCb));

  std::vector<int>  qpVals = { 17, 22, 34, 42 };
  std::copy(qpVals.begin(), qpVals.end(), c->m_qpInValsCb);

  qpVals = { 17, 23, 35, 39 };
  std::copy(qpVals.begin(), qpVals.end(), c->m_qpOutValsCb);

  // basic settings
  c->m_intraQPOffset                   = -3;
  c->m_lambdaFromQPEnable              = true;
  c->m_MaxCodingDepth                  = 5;
  c->m_log2DiffMaxMinCodingBlockSize   = 5;
  c->m_bUseASR                         = true;
  c->m_bUseHADME                       = true;
  c->m_useRDOQTS                       = true;
  c->m_useSelectiveRDOQ                = false;
  c->m_fastQtBtEnc                     = true;
  c->m_maxNumMergeCand                 = 6;
  c->m_reshapeSignalType               = 0;
  c->m_updateCtrl                      = 0;
  c->m_LMCSOffset                      = 6;
  c->m_RDOQ                            = 1;
  c->m_SignDataHidingEnabled           = 0;
  c->m_useFastLCTU                     = 1;

  // tools
  c->m_Affine                          = 0;
  c->m_alf                             = 0;
  c->m_alfSpeed                        = 0;
  c->m_allowDisFracMMVD                = 0;
  c->m_BCW                             = 0;
  c->m_BDOF                            = 0;
  c->m_ccalf                           = 0;
  c->m_CIIP                            = 0;
  c->m_DepQuantEnabled                 = 0;
  c->m_DMVR                            = 0;
  c->m_EDO                             = 0;
  c->m_Geo                             = 0;
  c->m_AMVRspeed                       = 0;
  c->m_ISP                             = 0;
  c->m_JointCbCrMode                   = 0;
  c->m_LFNST                           = 0;
  c->m_LMChroma                        = 0;
  c->m_lumaReshapeEnable               = 0;
  c->m_vvencMCTF.MCTF                  = 0;
  c->m_vvencMCTF.MCTFSpeed             = 0;
  c->m_MIP                             = 0;
  c->m_useFastMIP                      = 0;
  c->m_MMVD                            = 0;
  c->m_MRL                             = 0;
  c->m_MTS                             = 0;
  c->m_MTSImplicit                     = 0;
  c->m_PROF                            = 0;
  c->m_bUseSAO                         = 1;
  c->m_SbTMVP                          = 0;
  c->m_SBT                             = 0;
  c->m_SMVD                            = 0;
  c->m_TMVPModeId                      = 1;
  c->m_useNonLinearAlfChroma           = 0;
  c->m_useNonLinearAlfLuma             = 0;

  // ssc tools
  c->m_motionEstimationSearchMethodSCC = 2;
  c->m_useBDPCM                        = 2;
  c->m_IBCMode                         = 2;
  c->m_IBCFastMethod                   = 6;
  c->m_TS                              = 2;
  c->m_useChromaTS                     = 0;
  c->m_TSsize                          = 3;

  switch( preset )
  {
    case vvencPresetMode::VVENC_FIRSTPASS:
      c->m_DMVR                            = 0;

      // motion estimation
      c->m_SearchRange                     = 128;
      c->m_bipredSearchRange               = 1;
      c->m_minSearchWindow                 = 96;
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE1;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND_FAST;

      // partitioning: CTUSize64 QT44MTT00
      c->m_CTUSize                         = 64;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 32;
      c->m_MinQT[ 1 ]                      = 32;
      c->m_MinQT[ 2 ]                      = 16;
      c->m_maxMTTDepth                     = 0;
      c->m_maxMTTDepthI                    = 0;
      c->m_maxMTTDepthIChroma              = 0;
      c->m_log2MinCodingBlockSize          = 5;

      // speedups
      c->m_qtbttSpeedUp                    = 7;
#if FASTTT_TH
      c->m_fastTTSplit                     = 0;
#endif
      c->m_contentBasedFastQtbt            = 0;
      c->m_usePbIntraFast                  = 1;
      c->m_useFastMrg                      = 2;
      c->m_fastLocalDualTreeMode           = 1;
      c->m_fastSubPel                      = 2;
      c->m_FastIntraTools                  = 0;
      c->m_FIMMode                         = 4;
      c->m_useEarlyCU                      = 2;
      c->m_bIntegerET                      = 1;
      c->m_IntraEstDecBit                  = 3;
      c->m_numIntraModesFullRD             = 1;
      c->m_reduceIntraChromaModesFullRD    = true;

      // tools
      c->m_RDOQ                            = 2;
      c->m_SignDataHidingEnabled           = 1;
      c->m_LMChroma                        = 1;
      c->m_vvencMCTF.MCTF                  = 2;
      c->m_vvencMCTF.MCTFSpeed             = 4;
      c->m_MTSImplicit                     = 1;
      // scc
      c->m_IBCFastMethod                   = 6;
      c->m_TSsize                          = 3;

      break;

    case vvencPresetMode::VVENC_FASTER:
      c->m_DMVR                            = 1;

      // motion estimation
      c->m_SearchRange                     = 128;
      c->m_bipredSearchRange               = 1;
      c->m_minSearchWindow                 = 96;
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE1;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND_FAST;

      // partitioning: CTUSize64 QT44MTT00
      c->m_CTUSize                         = 64;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 4;
      c->m_MinQT[ 1 ]                      = 4;
      c->m_MinQT[ 2 ]                      = 2;
      c->m_maxMTTDepth                     = 0;
      c->m_maxMTTDepthI                    = 0;
      c->m_maxMTTDepthIChroma              = 0;
      c->m_log2MinCodingBlockSize          = 2;

      // speedups
      c->m_qtbttSpeedUp                    = 7;
#if FASTTT_TH
      c->m_fastTTSplit                     = 0;
#endif
      c->m_contentBasedFastQtbt            = 1;
      c->m_usePbIntraFast                  = 2;
      c->m_useFastMrg                      = 2;
      c->m_fastLocalDualTreeMode           = 1;
      c->m_fastSubPel                      = 1;
      c->m_FastIntraTools                  = 0;
      c->m_FIMMode                         = 4;
      c->m_useEarlyCU                      = 1;
      c->m_bIntegerET                      = 1;
      c->m_IntraEstDecBit                  = 3;
      c->m_numIntraModesFullRD             = 1;
      c->m_reduceIntraChromaModesFullRD    = true;

      // tools
      c->m_alf                             = 1;
      c->m_alfSpeed                        = 1;
      c->m_ccalf                           = 1;
      c->m_RDOQ                            = 2;
      c->m_SignDataHidingEnabled           = 1;
      c->m_LMChroma                        = 1;
      c->m_vvencMCTF.MCTF                  = 2;
      c->m_vvencMCTF.MCTFSpeed             = 4;
      c->m_MTSImplicit                     = 1;
      // scc
      c->m_IBCFastMethod                   = 6;
      c->m_TSsize                          = 3;

      break;

    case vvencPresetMode::VVENC_FAST:

      // motion estimation
      c->m_SearchRange                     = 128;
      c->m_bipredSearchRange               = 1;
      c->m_minSearchWindow                 = 96;
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE1;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND_FAST;

      // partitioning: CTUSize64 QT44MTT10
      c->m_CTUSize                         = 64;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 4;
      c->m_MinQT[ 1 ]                      = 4;
      c->m_MinQT[ 2 ]                      = 2;
      c->m_maxMTTDepth                     = 0;
      c->m_maxMTTDepthI                    = 1;
      c->m_maxMTTDepthIChroma              = 1;
      c->m_log2MinCodingBlockSize          = 2;

      // speedups                          
      c->m_qtbttSpeedUp                    = 3;
#if FASTTT_TH
      c->m_fastTTSplit                     = 0;
#endif
      c->m_contentBasedFastQtbt            = 1;
      c->m_usePbIntraFast                  = 2;
      c->m_useFastMrg                      = 2;
      c->m_fastLocalDualTreeMode           = 1;
      c->m_fastSubPel                      = 1;
      c->m_FastIntraTools                  = 0;
      c->m_FIMMode                         = 2;
      c->m_useEarlyCU                      = 1;
      c->m_bIntegerET                      = 0;
      c->m_IntraEstDecBit                  = 2;
      c->m_numIntraModesFullRD             = -1;
      c->m_reduceIntraChromaModesFullRD    = true;

      // tools                             
      c->m_RDOQ                            = 2;
      c->m_SignDataHidingEnabled           = 1;
      c->m_Affine                          = 2;
      c->m_alf                             = 1;
      c->m_alfSpeed                        = 1;
      c->m_allowDisFracMMVD                = 1;
      c->m_BDOF                            = 1;
      c->m_ccalf                           = 1;
      c->m_DMVR                            = 1;
      c->m_AMVRspeed                       = 5;
      c->m_JointCbCrMode                   = 1;
      c->m_LFNST                           = 1;
      c->m_LMChroma                        = 1;
      c->m_lumaReshapeEnable               = 2;
      c->m_vvencMCTF.MCTF                  = 2;
      c->m_vvencMCTF.MCTFSpeed             = 2;
      c->m_MMVD                            = 3;
      c->m_MRL                             = 1;
      c->m_MTSImplicit                     = 1;
      c->m_PROF                            = 1;
      c->m_SbTMVP                          = 1;
      // scc
      c->m_IBCFastMethod                   = 4;
      c->m_TSsize                          = 4;

      break;

    case vvencPresetMode::VVENC_MEDIUM:

      // motion estimation
      c->m_SearchRange                     = 384;
      c->m_bipredSearchRange               = 4;
      c->m_minSearchWindow                 = 96;
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE1;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND_FAST;

      // partitioning: CTUSize128 QT44MTT21
      c->m_CTUSize                         = 128;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 8;
      c->m_MinQT[ 1 ]                      = 8;
      c->m_MinQT[ 2 ]                      = 4;
      c->m_maxMTTDepth                     = 1;
      c->m_maxMTTDepthI                    = 2;
      c->m_maxMTTDepthIChroma              = 2;
      c->m_log2MinCodingBlockSize          = 2;

      // speedups                          
      c->m_qtbttSpeedUp                    = 3;
#if FASTTT_TH
      c->m_fastTTSplit                     = 0;
#endif
      c->m_contentBasedFastQtbt            = 0;
      c->m_usePbIntraFast                  = 1;
      c->m_useFastMrg                      = 2;
      c->m_fastLocalDualTreeMode           = 1;
      c->m_fastSubPel                      = 1;
      c->m_FastIntraTools                  = 1;
      c->m_FIMMode                         = 0;
      c->m_useEarlyCU                      = 0;
      c->m_bIntegerET                      = 0;
      c->m_IntraEstDecBit                  = 2;
      c->m_numIntraModesFullRD             = -1;
      c->m_reduceIntraChromaModesFullRD    = false;

      // tools
      c->m_Affine                          = 2;
      c->m_alf                             = 1;
      c->m_alfSpeed                        = 0;
      c->m_allowDisFracMMVD                = 1;
      c->m_BDOF                            = 1;
      c->m_ccalf                           = 1;
      c->m_DepQuantEnabled                 = 1;
      c->m_DMVR                            = 1;
      c->m_EDO                             = 2;
      c->m_Geo                             = 3;
      c->m_AMVRspeed                       = 5;
      c->m_ISP                             = 3;
      c->m_JointCbCrMode                   = 1;
      c->m_LFNST                           = 1;
      c->m_LMChroma                        = 1;
      c->m_lumaReshapeEnable               = 2;
      c->m_vvencMCTF.MCTF                  = 2;
      c->m_vvencMCTF.MCTFSpeed             = 0;
      c->m_MIP                             = 1;
      c->m_useFastMIP                      = 4;
      c->m_MMVD                            = 3;
      c->m_MRL                             = 1;
      c->m_MTSImplicit                     = 1;
      c->m_PROF                            = 1;
      c->m_SbTMVP                          = 1;
      c->m_SMVD                            = 3;
      // scc
      c->m_IBCFastMethod                   = 3;
      c->m_TSsize                          = 4;

      break;

    case vvencPresetMode::VVENC_SLOW:

      // motion estimation
      c->m_SearchRange                     = 384;
      c->m_bipredSearchRange               = 4;
      c->m_minSearchWindow                 = 96;
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE1;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND_FAST;

      // partitioning: CTUSize128 QT44MTT32
      c->m_CTUSize                         = 128;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 8;
      c->m_MinQT[ 1 ]                      = 8;
      c->m_MinQT[ 2 ]                      = 4;
      c->m_maxMTTDepth                     = 2;
      c->m_maxMTTDepthI                    = 3;
      c->m_maxMTTDepthIChroma              = 3;
      c->m_log2MinCodingBlockSize          = 2;

      // speedups                          
      c->m_qtbttSpeedUp                    = 2;
#if FASTTT_TH
      c->m_fastTTSplit                     = 5;
#endif
      c->m_contentBasedFastQtbt            = 0;
      c->m_usePbIntraFast                  = 1;
      c->m_useFastMrg                      = 2;
      c->m_fastLocalDualTreeMode           = 1;
      c->m_fastSubPel                      = 1;
      c->m_FastIntraTools                  = 0;
      c->m_FIMMode                         = 0;
      c->m_useEarlyCU                      = 0;
      c->m_bIntegerET                      = 0;
      c->m_IntraEstDecBit                  = 1;
      c->m_numIntraModesFullRD             = -1;
      c->m_reduceIntraChromaModesFullRD    = false;

      // tools
      c->m_Affine                          = 2;
      c->m_alf                             = 1;
      c->m_alfSpeed                        = 0;
      c->m_allowDisFracMMVD                = 1;
      c->m_BCW                             = 2;
      c->m_BDOF                            = 1;
      c->m_ccalf                           = 1;
      c->m_CIIP                            = 1;
      c->m_DepQuantEnabled                 = 1;
      c->m_DMVR                            = 1;
      c->m_EDO                             = 2;
      c->m_Geo                             = 1;
      c->m_AMVRspeed                       = 1;
      c->m_ISP                             = 1;
      c->m_JointCbCrMode                   = 1;
      c->m_LFNST                           = 1;
      c->m_LMChroma                        = 1;
      c->m_lumaReshapeEnable               = 2;
      c->m_vvencMCTF.MCTF                  = 2;
      c->m_vvencMCTF.MCTFSpeed             = 0;
      c->m_MIP                             = 1;
      c->m_useFastMIP                      = 0;
      c->m_MMVD                            = 3;
      c->m_MRL                             = 1;
      c->m_MTSImplicit                     = 1;
      c->m_PROF                            = 1;
      c->m_SBT                             = 1;
      c->m_SbTMVP                          = 1;
      c->m_SMVD                            = 3;
      // scc
      c->m_IBCFastMethod                   = 1;
      c->m_TSsize                          = 5;

      break;

    case vvencPresetMode::VVENC_SLOWER:

      // motion estimation
      c->m_SearchRange                     = 384;
      c->m_bipredSearchRange               = 4;
      c->m_minSearchWindow                 = 96;
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE1;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND;

      // partitioning: CTUSize128 QT44MTT33
      c->m_CTUSize                         = 128;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 8;
      c->m_MinQT[ 1 ]                      = 8;
      c->m_MinQT[ 2 ]                      = 4;
      c->m_maxMTTDepth                     = 3;
      c->m_maxMTTDepthI                    = 3;
      c->m_maxMTTDepthIChroma              = 3;
      c->m_log2MinCodingBlockSize          = 2;

      // speedups                          
      c->m_qtbttSpeedUp                    = 1;
#if FASTTT_TH
      c->m_fastTTSplit                     = 1;
#endif
      c->m_contentBasedFastQtbt            = 0;
      c->m_usePbIntraFast                  = 1;
      c->m_useFastMrg                      = 1;
      c->m_fastLocalDualTreeMode           = 1;
      c->m_fastSubPel                      = 0;
      c->m_FastIntraTools                  = 0;
      c->m_FIMMode                         = 0;
      c->m_useEarlyCU                      = 0;
      c->m_bIntegerET                      = 0;
      c->m_IntraEstDecBit                  = 1;
      c->m_numIntraModesFullRD             = -1;
      c->m_reduceIntraChromaModesFullRD    = false;

      // tools
      c->m_Affine                          = 1;
      c->m_alf                             = 1;
      c->m_alfSpeed                        = 0;
      c->m_allowDisFracMMVD                = 1;
      c->m_BCW                             = 2;
      c->m_BDOF                            = 1;
      c->m_ccalf                           = 1;
      c->m_CIIP                            = 1;
      c->m_DepQuantEnabled                 = 1;
      c->m_DMVR                            = 1;
      c->m_EDO                             = 2;
      c->m_Geo                             = 1;
      c->m_AMVRspeed                       = 1;
      c->m_ISP                             = 1;
      c->m_JointCbCrMode                   = 1;
      c->m_LFNST                           = 1;
      c->m_LMChroma                        = 1;
      c->m_lumaReshapeEnable               = 2;
      c->m_vvencMCTF.MCTF                  = 2;
      c->m_vvencMCTF.MCTFSpeed             = 0;
      c->m_MIP                             = 1;
      c->m_useFastMIP                      = 0;
      c->m_MMVD                            = 1;
      c->m_MRL                             = 1;
      c->m_MTS                             = 1;
      c->m_PROF                            = 1;
      c->m_SBT                             = 1;
      c->m_SbTMVP                          = 1;
      c->m_SMVD                            = 1;
      c->m_useNonLinearAlfChroma           = 1;
      c->m_useNonLinearAlfLuma             = 1;
      // scc
      c->m_IBCFastMethod                   = 1;
      c->m_TSsize                          = 5;

      break;

    case vvencPresetMode::VVENC_TOOLTEST:

      // motion estimation
      c->m_SearchRange                     = 384;
      c->m_bipredSearchRange               = 4;
      c->m_minSearchWindow                 = 96;
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE1;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND_FAST;

      // partitioning: CTUSize128 QT44MTT21
      c->m_CTUSize                         = 128;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 8;
      c->m_MinQT[ 1 ]                      = 8;
      c->m_MinQT[ 2 ]                      = 4;
      c->m_maxMTTDepth                     = 1;
      c->m_maxMTTDepthI                    = 2;
      c->m_maxMTTDepthIChroma              = 2;
      c->m_log2MinCodingBlockSize          = 2;

      // speedups                          
      c->m_qtbttSpeedUp                    = 2;
#if FASTTT_TH
      c->m_fastTTSplit                     = 0;
#endif
      c->m_contentBasedFastQtbt            = 1;
      c->m_usePbIntraFast                  = 1;
      c->m_useFastMrg                      = 2;
      c->m_fastLocalDualTreeMode           = 1;
      c->m_fastSubPel                      = 1;
      c->m_FastIntraTools                  = 1;
      c->m_FIMMode                         = 3;
      c->m_useEarlyCU                      = 1;
      c->m_bIntegerET                      = 1;
      c->m_IntraEstDecBit                  = 3;
      c->m_numIntraModesFullRD             = -1;
      c->m_reduceIntraChromaModesFullRD    = false;

      // tools
      c->m_Affine                          = 2;
      c->m_alf                             = 1;
      c->m_alfSpeed                        = 0;
      c->m_allowDisFracMMVD                = 1;
      c->m_BCW                             = 2;
      c->m_BDOF                            = 1;
      c->m_ccalf                           = 1;
      c->m_CIIP                            = 3;
      c->m_DepQuantEnabled                 = 1;
      c->m_DMVR                            = 1;
      c->m_EDO                             = 1;
      c->m_Geo                             = 2;
      c->m_AMVRspeed                       = 3;
      c->m_ISP                             = 2;
      c->m_JointCbCrMode                   = 1;
      c->m_LFNST                           = 1;
      c->m_LMChroma                        = 1;
      c->m_lumaReshapeEnable               = 2;
      c->m_vvencMCTF.MCTF                  = 2;
      c->m_vvencMCTF.MCTFSpeed             = 2;
      c->m_MIP                             = 1;
      c->m_useFastMIP                      = 4;
      c->m_MMVD                            = 2;
      c->m_MRL                             = 1;
      c->m_MTS                             = 1;
      c->m_PROF                            = 1;
      c->m_SBT                             = 2;
      c->m_SbTMVP                          = 1;
      c->m_SMVD                            = 3;
      c->m_useNonLinearAlfChroma           = 1;
      c->m_useNonLinearAlfLuma             = 1;
      // scc
      c->m_motionEstimationSearchMethodSCC = 3;
      c->m_useBDPCM                        = 1;
      c->m_IBCFastMethod                   = 5;
      c->m_TS                              = 1;
      c->m_useChromaTS                     = 1;
      c->m_TSsize                          = 3;

      break;

    default:
      return -1;
  }

  return 0;
}

VVENC_DECL const char* vvenc_get_config_as_string( vvenc_config *c, vvencMsgLevel eMsgLevel )
{
  std::stringstream css;

  if( eMsgLevel >= VVENC_DETAILS )
  {
  css << "Real     Format                        : " << c->m_PadSourceWidth - c->m_confWinLeft - c->m_confWinRight << "x" << c->m_PadSourceHeight - c->m_confWinTop - c->m_confWinBottom << " " <<
                                                        (double)c->m_FrameRate / c->m_temporalSubsampleRatio << "Hz " << getDynamicRangeStr(c->m_HdrMode) << "\n";
  css << "Internal Format                        : " << c->m_PadSourceWidth << "x" << c->m_PadSourceHeight << " " <<  (double)c->m_FrameRate / c->m_temporalSubsampleRatio << "Hz "  << getDynamicRangeStr(c->m_HdrMode) << "\n";
  css << "Sequence PSNR output                   : " << (c->m_printMSEBasedSequencePSNR ? "Linear average, MSE-based" : "Linear average only") << "\n";
  css << "Hexadecimal PSNR output                : " << (c->m_printHexPsnr ? "Enabled" : "Disabled") << "\n";
  css << "Sequence MSE output                    : " << (c->m_printSequenceMSE ? "Enabled" : "Disabled") << "\n";
  css << "Frame MSE output                       : " << (c->m_printFrameMSE ? "Enabled" : "Disabled") << "\n";
  css << "Cabac-zero-word-padding                : " << (c->m_cabacZeroWordPaddingEnabled ? "Enabled" : "Disabled") << "\n";
  css << "Frame/Field                            : Frame based coding\n";
  if ( c->m_framesToBeEncoded > 0 )
    css << "Frame index                            : " << c->m_framesToBeEncoded << " frames\n";
  else
    css << "Frame index                            : all frames\n";

  css << "Profile                                : " << getProfileStr( c->m_profile ) << "\n";
  css << "Level                                  : " << getLevelStr( c->m_level ) << "\n";
  css << "CU size / total-depth                  : " << c->m_CTUSize << " / " << c->m_MaxCodingDepth << "\n";
  css << "Max TB size                            : " << (1 << c->m_log2MaxTbSize) << "\n";
  css << "Min CB size                            : " << (1 << c->m_log2MinCodingBlockSize) << "\n";
  css << "Motion search range                    : " << c->m_SearchRange << "\n";
  css << "Intra period                           : " << c->m_IntraPeriod << "\n";
  css << "Decoding refresh type                  : " << c->m_DecodingRefreshType << "\n";
  css << "QP                                     : " << c->m_QP << "\n";
  css << "Percept QPA                            : " << c->m_usePerceptQPA << "\n";
  css << "Max dQP signaling subdiv               : " << c->m_cuQpDeltaSubdiv << "\n";
  css << "Cb QP Offset (dual tree)               : " << c->m_chromaCbQpOffset << " (" << c->m_chromaCbQpOffsetDualTree << ")\n";
  css << "Cr QP Offset (dual tree)               : " << c->m_chromaCrQpOffset << " (" << c->m_chromaCrQpOffsetDualTree << ")\n";
  css << "GOP size                               : " << c->m_GOPSize << "\n";
  css << "Input queue size                       : " << c->m_InputQueueSize << "\n";
  css << "Input bit depth                        : (Y:" << c->m_inputBitDepth[ 0 ] << ", C:" << c->m_inputBitDepth[ 1 ] << ")\n";
  css << "MSB-extended bit depth                 : (Y:" << c->m_MSBExtendedBitDepth[ 0 ] << ", C:" << c->m_MSBExtendedBitDepth[ 1 ] << ")\n";
  css << "Internal bit depth                     : (Y:" << c->m_internalBitDepth[ 0 ] << ", C:" << c->m_internalBitDepth[ 1 ] << ")\n";
  css << "cu_chroma_qp_offset_subdiv             : " << c->m_cuChromaQpOffsetSubdiv << "\n";
  if (c->m_bUseSAO)
  {
    css << "log2_sao_offset_scale_luma             : " << c->m_log2SaoOffsetScale[ 0 ] << "\n";
    css << "log2_sao_offset_scale_chroma           : " << c->m_log2SaoOffsetScale[ 1 ] << "\n";
  }
  css << "Cost function:                         : " << getCostFunctionStr( c->m_costMode ) << "\n";

  if( c->m_masteringDisplay[0] != 0 || c->m_masteringDisplay[1] != 0 || c->m_masteringDisplay[8] != 0  )
  {
    css << "Mastering display color volume         : " << vvenc_getMasteringDisplayStr( c->m_masteringDisplay ) << "\n";
  }
  if( c->m_contentLightLevel[0] != 0 || c->m_contentLightLevel[1] != 0 )
  {
    css << "Content light level                    : " << vvenc_getContentLightLevelStr( c->m_contentLightLevel ) << "\n";
  }
  css << "\n";
  }

  if( eMsgLevel >= VVENC_VERBOSE )
  {
  // verbose output
  css << "CODING TOOL CFG: ";
  css << "CTU" << c->m_CTUSize << " QT" << vvenc::Log2( c->m_CTUSize / c->m_MinQT[0] ) << vvenc::Log2( c->m_CTUSize / c->m_MinQT[1] ) << "BTT" << c->m_maxMTTDepthI << c->m_maxMTTDepth << " ";
  css << "IBD:" << ((c->m_internalBitDepth[ 0 ] > c->m_MSBExtendedBitDepth[ 0 ]) || (c->m_internalBitDepth[ 1 ] > c->m_MSBExtendedBitDepth[ 1 ])) << " ";
  css << "CIP:" << c->m_bUseConstrainedIntraPred << " ";
  css << "SAO:" << (c->m_bUseSAO ? 1 : 0) << " ";
  css << "ALF:" << (c->m_alf ? 1 : 0) << " ";
  if( c->m_alf )
  {
    css << "(NonLinLuma:" << c->m_useNonLinearAlfLuma << " ";
    css << "NonLinChr:" << c->m_useNonLinearAlfChroma << ") ";
  }
  css << "CCALF:" << (c->m_ccalf ? 1 : 0) << " ";

  css << "Tiles:" << c->m_numTileCols << "x" << c->m_numTileRows << " ";
  css << "Slices:"<< c->m_numSlicesInPic << " ";

  const int iWaveFrontSubstreams = c->m_entropyCodingSyncEnabled ? ( c->m_PadSourceHeight + c->m_CTUSize - 1 ) / c->m_CTUSize : 1;
  css << "WPP:" << (c->m_entropyCodingSyncEnabled ? 1 : 0) << " ";
  css << "WPP-Substreams:" << iWaveFrontSubstreams << " ";
  css << "TMVP:" << c->m_TMVPModeId << " ";

  css << "DQ:" << c->m_DepQuantEnabled << " ";
  css << "SDH:" << c->m_SignDataHidingEnabled << " ";
  css << "CST:" << c->m_dualITree << " ";
  css << "BDOF:" << c->m_BDOF << " ";
  css << "DMVR:" << c->m_DMVR << " ";
  css << "MTSImplicit:" << c->m_MTSImplicit << " ";
  css << "SBT:" << c->m_SBT << " ";
  css << "JCbCr:" << c->m_JointCbCrMode << " ";
  css << "CabacInitPresent:" << c->m_cabacInitPresent << " ";
  css << "AMVR:" << c->m_AMVRspeed << " ";
  css << "SMVD:" << c->m_SMVD << " ";

  css << "LMCS:" << c->m_lumaReshapeEnable << " ";
  if( c->m_lumaReshapeEnable )
  {
    css << "(Signal:" << (c->m_reshapeSignalType == 0 ? "SDR" : (c->m_reshapeSignalType == 2 ? "HDR-HLG" : "HDR-PQ")) << " ";
    css << "Opt:" << c->m_adpOption << "";
    if( c->m_adpOption > 0 )
    {
      css << " CW:" << c->m_initialCW << "";
    }
    css << ") ";
  }
  css << "CIIP:" << c->m_CIIP << " ";
  css << "MIP:" << c->m_MIP << " ";
  css << "AFFINE:" << c->m_Affine << " ";
  if( c->m_Affine )
  {
    css << "(PROF:" << c->m_PROF << ", ";
    css << "Type:" << c->m_AffineType << ") ";
  }
  css << "MMVD:" << c->m_MMVD << " ";
  if( c->m_MMVD )
    css << "DisFracMMVD:" << c->m_allowDisFracMMVD << " ";
  css << "SbTMVP:" << c->m_SbTMVP << " ";
  css << "GPM:" << c->m_Geo << " ";
  css << "LFNST:" << c->m_LFNST << " ";
  css << "MTS:" << c->m_MTS << " ";
  if( c->m_MTS )
  {
    css << "(IntraCand:" << c->m_MTSIntraMaxCand << ") ";
  }
  css << "ISP:" << c->m_ISP << " ";
  css << "TS:" << c->m_TS << " ";
  if( c->m_TS )
  {
    css << "TSLog2MaxSize:" << c->m_TSsize << " ";
    css << "useChromaTS:" << c->m_useChromaTS << " ";
  }
  css << "BDPCM:" << c->m_useBDPCM << " ";
  css << "IBC:" << c->m_IBCMode << " ";
  css << "BCW:" << c->m_BCW << " ";

  css << "\nENC. ALG. CFG: ";
  css << "QPA:" << c->m_usePerceptQPA << " ";
  css << "HAD:" << c->m_bUseHADME << " ";
  css << "RDQ:" << c->m_RDOQ << " ";
  css << "RDQTS:" << c->m_useRDOQTS << " ";
  css << "ASR:" << c->m_bUseASR << " ";
  css << "MinSearchWindow:" << c->m_minSearchWindow << " ";
  css << "RestrictMESampling:" << c->m_bRestrictMESampling << " ";
  css << "EDO:" << c->m_EDO << " ";
  css << "MCTF:" << c->m_vvencMCTF.MCTF << " ";
  if( c->m_vvencMCTF.MCTF )
  {
    css << "[L:" << c->m_vvencMCTF.MCTFNumLeadFrames << ", T:" << c->m_vvencMCTF.MCTFNumTrailFrames << "] ";
  }

  css << "\nFAST TOOL CFG: ";
  css << "ECU:" << c->m_useEarlyCU << " ";
  css << "FEN:" << c->m_fastInterSearchMode << " ";
  css << "FDM:" << c->m_useFastDecisionForMerge << " ";
  css << "FastSearch:" << c->m_motionEstimationSearchMethod << " ";
  if( c->m_motionEstimationSearchMethodSCC )
  {
    css << "(SCC:" << c->m_motionEstimationSearchMethodSCC << ") ";
  }
  css << "LCTUFast:" << c->m_useFastLCTU << " ";
  css << "FastMrg:" << c->m_useFastMrg << " ";
  css << "PBIntraFast:" << c->m_usePbIntraFast << " ";
  css << "AMaxBT:" << c->m_useAMaxBT << " ";
  css << "FastQtBtEnc:" << c->m_fastQtBtEnc << " ";
  css << "ContentBasedFastQtbt:" << c->m_contentBasedFastQtbt << " ";
  if( c->m_MIP )
  {
    css << "FastMIP:" << c->m_useFastMIP << " ";
  }
  css << "FastIntraTools:" << c->m_FastIntraTools << " ";
  css << "IntraEstDecBit:" << c->m_IntraEstDecBit << " ";
  css << "FastLocalDualTree:" << c->m_fastLocalDualTreeMode << " ";
  css << "IntegerET:" << c->m_bIntegerET << " ";
  css << "FastSubPel:" << c->m_fastSubPel << " ";
  css << "QtbttExtraFast:" << c->m_qtbttSpeedUp << " ";
#if FASTTT_TH
  css << "FastTTSplit:" << c->m_fastTTSplit << " ";
#endif
  if( c->m_IBCMode )
  {
    css << "IBCFastMethod:" << c->m_IBCFastMethod << " ";
  }
  css << "FIM:" << c->m_FIMMode << " ";
  if( c->m_FastInferMerge )
  {
    css << "(" << c->m_FastInferMerge << ") ";
  }
  if( c->m_alf )
  {
    css << "ALFSpeed:" << c->m_alfSpeed << " ";
  }
  if( c->m_quantThresholdVal & 1 )
    css << "QuantThr: " << (c->m_quantThresholdVal >> 1) << ".5 ";
  else
    css << "QuantThr: " << (c->m_quantThresholdVal >> 1) << " ";

  css << "\nRATE CONTROL CFG: ";
  css << "RateControl:" << ( c->m_RCTargetBitrate > 0 ) << " ";
  if ( c->m_RCTargetBitrate > 0 )
  {
    css << "Passes:" << c->m_RCNumPasses << " ";
    css << "Pass:" << c->m_RCPass << " ";
    css << "TargetBitrate:" << c->m_RCTargetBitrate << " ";
    css << "RCInitialQP:" << c->m_RCInitialQP << " ";
    css << "RCForceIntraQP:" << c->m_RCForceIntraQP << " ";
  }

  css << "\nPARALLEL PROCESSING CFG: ";
  css << "NumThreads:" << c->m_numThreads << " ";
  css << "MaxParallelFrames:" << c->m_maxParallelFrames << " ";
  css << "WppBitEqual:" << c->m_ensureWppBitEqual << " ";
  css << "WF:" << c->m_entropyCodingSyncEnabled << "";
  css << "\n";
  }

  vvenc_cfgString = css.str();
  return vvenc_cfgString.c_str();
}

VVENC_NAMESPACE_END



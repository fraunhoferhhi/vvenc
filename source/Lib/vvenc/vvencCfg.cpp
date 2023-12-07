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


/** \file     vvencCfg.cpp
    \brief    encoder configuration class
*/

#include "vvenc/vvencCfg.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Slice.h"
#include "CommonLib/ProfileLevelTier.h"
#include "Utilities/MsgLog.h"
#include "EncoderLib/GOPCfg.h"

#include <math.h>
#include <thread>
#include <iomanip>

#include "apputils/VVEncAppCfg.h"


VVENC_NAMESPACE_BEGIN

static bool checkCfgParameter( vvenc_config *cfg );
static void checkCfgPicPartitioningParameter( vvenc_config *c );
static void checkCfgInputArrays( vvenc_config *c, int &lastNonZeroCol, int &lastNonZeroRow, bool &cfgIsValid );
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
    case VVENC_SDR_BT709          : cT = "SDR BT.709"; break;
    case VVENC_SDR_BT2020         : cT = "SDR BT.2020"; break;
    case VVENC_SDR_BT470BG        : cT = "SDR BT.470 B/G"; break;
    default                       : cT = "unknown"; break;
  }
  return cT;
}

static inline bool isHDRMode( vvencHDRMode hdrMode )
{
  return ( hdrMode == VVENC_HDR_PQ || hdrMode == VVENC_HDR_PQ_BT2020 ||
           hdrMode == VVENC_HDR_HLG || hdrMode == VVENC_HDR_HLG_BT2020 ) ? true : false;
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

static inline std::string vvenc_getDecodingRefreshTypeStr(  int type, bool poc0idr )
{
  std::string cType( "CRA");
  switch( type )
  {
    case 0: cType = "none"; break;
    case 1: cType = "CRA"; break;
    case 2: cType = "IDR"; break;
    case 3: cType = "RecPointSEI"; break;
    case 4: cType = "IDR2 (deprecated)"; break; //deprecated
    case 5: cType = "CRA_CRE (CRA with constrained encoding for RASL pictures)"; break;
    default: cType = "unknown"; break;
  }
  if( poc0idr ) cType += " with POC 0 IDR";
  return cType;
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
  GOPEntry->m_cfgUnused1                = 0;
  GOPEntry->m_cfgUnused2                = 0;
  GOPEntry->m_cfgUnused3                = 0;
  GOPEntry->m_cfgUnused4                = 0;
  GOPEntry->m_temporalId                = 0;
  GOPEntry->m_sliceType                 = 'P';
  memset( GOPEntry->m_numRefPicsActive, 0, sizeof( GOPEntry->m_numRefPicsActive ) );
  memset( GOPEntry->m_numRefPics, 0, sizeof( GOPEntry->m_numRefPics ) );
  memset( GOPEntry->m_deltaRefPics, 0, sizeof( GOPEntry->m_deltaRefPics ) );
}

VVENC_DECL void vvenc_WCGChromaQPControl_default(vvencWCGChromaQPControl *WCGChromaQPControl )
{
  memset( WCGChromaQPControl, 0, sizeof( vvencWCGChromaQPControl ) );
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
  reshapeCW->rspTid     = 0;
  reshapeCW->rspFpsToIp = 0;
}

VVENC_DECL void vvenc_vvencMCTF_default(vvencMCTF *vvencMCTF )
{
  vvencMCTF->MCTF = 0;
  vvencMCTF->MCTFSpeed = 0;
  vvencMCTF->MCTFFutureReference = true;
  vvencMCTF->numFrames = 0;
  vvencMCTF->numStrength = 0;
  vvencMCTF->MCTFUnitSize = -1;
  memset( vvencMCTF->MCTFFrames, 0, sizeof( vvencMCTF->MCTFFrames ) );
  memset( vvencMCTF->MCTFStrengths, 0, sizeof( vvencMCTF->MCTFStrengths ) );
}

VVENC_DECL void vvenc_config_default(vvenc_config *c )
{
  int i = 0;

  // internal params
  c->m_configDone                              = false;         ///< state variable, Private context used for internal data ( do not change )
  c->m_confirmFailed                           = false;         ///< state variable, Private context used for internal data ( do not change )
  c->m_msgFnc                                  = nullptr;
  c->m_msgCtx                                  = nullptr;

  // core params
  c->m_SourceWidth                             = 0;             ///< source width in pixel
  c->m_SourceHeight                            = 0;             ///< source height in pixel (when interlaced = field height)
  c->m_FrameRate                               = 0;             ///< source frame-rates (Hz) Numerator
  c->m_FrameScale                              = 1;             ///< source frame-rates (Hz) Denominator
  c->m_TicksPerSecond                          = VVENC_TICKS_PER_SEC_DEF; ///< ticks per second for dts generation (default: 27000000, 1..27000000, -1: ticks per frame=1)

  c->m_framesToBeEncoded                       = 0;             ///< number of encoded frames

  c->m_inputBitDepth[0]                        = 8;             ///< bit-depth of input
  c->m_inputBitDepth[1]                        = 0;

  c->m_numThreads                              = 0;             ///< number of worker threads

  c->m_QP                                      = VVENC_DEFAULT_QP;
  c->m_RCTargetBitrate                         = VVENC_RC_OFF;
  c->m_RCMaxBitrate                            = 0;
  c->m_RCInitialQP                             = 0;

  c->m_verbosity                               = VVENC_INFO;    ///< encoder verbosity

  // basic params
  c->m_profile                                 = vvencProfile::VVENC_PROFILE_AUTO;
  c->m_levelTier                               = vvencTier::VVENC_TIER_MAIN ;
  c->m_level                                   = vvencLevel::VVENC_LEVEL_AUTO;

  c->m_IntraPeriod                             = 0;             ///< period of I-slice (random access period)
  c->m_IntraPeriodSec                          = 1;             ///< period of I-slice in seconds (random access period)
  c->m_DecodingRefreshType                     = VVENC_DRT_CRA;       ///< random access type
  c->m_GOPSize                                 = 32;            ///< GOP size
  c->m_poc0idr                                 = false;
  c->m_picReordering                           = 1;

  c->m_usePerceptQPA                           = false;         ///< perceptually motivated input-adaptive QP modification, abbrev. perceptual QP adaptation (QPA)
  c->m_sliceTypeAdapt                          = -1;            ///< perceptually and objectively motivated slice type adaptation (STA)

  c->m_RCNumPasses                             = -1;
  c->m_RCPass                                  = -1;
  c->m_LookAhead                               = -1;

  c->m_SegmentMode                             = VVENC_SEG_OFF;

  c->m_internalBitDepth[0]                     =10;                                 ///< bit-depth codec operates at (input/output files will be converted)
  c->m_internalBitDepth[1]                     =0;

  c->m_HdrMode                                 = VVENC_HDR_OFF;

  // expert options
  c->m_conformanceWindowMode                   = 1;
  c->m_confWinLeft                             = 0;
  c->m_confWinRight                            = 0;
  c->m_confWinTop                              = 0;
  c->m_confWinBottom                           = 0;

  c->m_PadSourceWidth                          = 0;                                     ///< source width in pixel
  c->m_PadSourceHeight                         = 0;                                     ///< source height in pixel (when interlaced = field height)

  c->m_maxPicWidth                             = 0;
  c->m_maxPicHeight                            = 0;

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

  c->m_rewriteParamSets                        = true;                                 ///< Flag to enable rewriting of parameter sets at random access points
  c->m_idrRefParamList                         = false;                                 ///< indicates if reference picture list syntax elements are present in slice headers of IDR pictures
  for( i = 0; i < VVENC_MAX_GOP; i++ )
  {
    vvenc_GOPEntry_default( &c->m_GOPList[i]);                                          ///< the coding structure entries from the config file
  }

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
  vvenc_WCGChromaQPControl_default( &c->m_cfgUnused24 );

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
  c->m_maxMTTDepthIChroma                      = -1;

  c->m_maxBT[0]=32;  c->m_maxBT[1]=128;  c->m_maxBT[2]=64;
  c->m_maxTT[0]=32;  c->m_maxTT[1]=64;  c->m_maxTT[2]=32;
  c->m_dualITree                               = true;
  c->m_log2MaxTbSize                           = 6;
  c->m_log2MinCodingBlockSize                  = 2;

  c->m_bUseASR                                 = false;                                 ///< flag for using adaptive motion search range
  c->m_bUseHADME                               = true;                                  ///< flag for using HAD in sub-pel ME
  c->m_fastHad                                 = false;
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
  c->m_fastInterSearchMode                     = VVENC_FASTINTERSEARCH_MODE3;
  c->m_useEarlyCU                              = 0;
  c->m_useFastDecisionForMerge                 = true;

  c->m_bDisableIntraCUsInInterSlices           = false;
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

  c->m_motionEstimationSearchMethod            = VVENC_MESEARCH_DIAMOND;
  c->m_motionEstimationSearchMethodSCC         = 0;
  c->m_SearchRange                             = 96;
  c->m_bipredSearchRange                       = 4;
  c->m_minSearchWindow                         = 8;
  c->m_bClipForBiPredMeEnabled                 = false;
  c->m_bFastMEAssumingSmootherMVEnabled        = true;
  c->m_bIntegerET                              = false;
  c->m_fastSubPel                              = 0;
  c->m_meReduceTap                             = 0;
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

  c->m_bDisableLFCrossTileBoundaryFlag         = false;
  c->m_bDisableLFCrossSliceBoundaryFlag        = false;

  c->m_bUseSAO                                 = true;
  c->m_saoScc                                  = false;
  c->m_saoEncodingRate                         = -1.0;
  c->m_saoEncodingRateChroma                   = -1.0;
  c->m_log2SaoOffsetScale[0]=c->m_log2SaoOffsetScale[1] = 0;
  c->m_saoOffsetBitShift[0]=c->m_saoOffsetBitShift[1] = 0;

  c->m_decodingParameterSetEnabled             = false;
  c->m_vuiParametersPresent                    = -1;
  c->m_hrdParametersPresent                    = true;
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
  c->m_alfTempPred                             = -1;
  c->m_alfUnitSize                             = -1;

  vvenc_vvencMCTF_default( &c->m_vvencMCTF );

  c->m_blockImportanceMapping                  = true;

  c->m_quantThresholdVal                       = -1;
  c->m_qtbttSpeedUp                            = 1;
  c->m_qtbttSpeedUpMode                        = 0;
  c->m_fastTTSplit                             = 0;

  c->m_fastLocalDualTreeMode                   = 0;

  c->m_maxParallelFrames                       = -1;
  c->m_ensureWppBitEqual                       = -1;
  c->m_tileParallelCtuEnc                      = true;
  c->m_fppLinesSynchro                         = 0;

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

  c->m_listTracingChannels                     = false;
  memset( c->m_traceRule, '\0', sizeof(c->m_traceRule) );
  memset( c->m_traceFile, '\0', sizeof(c->m_traceFile) );

  c->m_numIntraModesFullRD                     = -1;
  c->m_reduceIntraChromaModesFullRD            = false;

  c->m_treatAsSubPic                           = false;
  c->m_explicitAPSid                           = 0;

  c->m_leadFrames                              = 0;
  c->m_trailFrames                             = 0;

  c-> m_deblockLastTLayers                     = 0;
  c->m_addGOP32refPics                         = false;
  c->m_numRefPics                              = 0;
  c->m_numRefPicsSCC                           = -1;

  c->m_FirstPassMode                           = 0;

  c->m_reservedFlag                            = false;
  c->m_reservedInt                             = 0;
  memset( c->m_reservedDouble, 0, sizeof(c->m_reservedDouble) );

  // init default preset
  vvenc_init_preset( c, vvencPresetMode::VVENC_MEDIUM );
}

static bool vvenc_confirmParameter ( vvenc_config *c, bool bflag, const char* message )
{
  if ( ! bflag )
    return false;

  vvenc::MsgLog msg(c->m_msgCtx,c->m_msgFnc);
  msg.log( VVENC_ERROR, "Parameter Check Error: %s\n", message );

  c->m_confirmFailed = true;
  return true;
}

VVENC_DECL bool vvenc_init_config_parameter( vvenc_config *c )
{
  c->m_confirmFailed = false;

  // check for valid base parameter
  vvenc_confirmParameter( c,  (c->m_SourceWidth <= 0 || c->m_SourceHeight <= 0), "Input resolution not set");

  vvenc_confirmParameter( c, c->m_inputBitDepth[0] < 8 || c->m_inputBitDepth[0] > 16,                    "InputBitDepth must be at least 8" );
  vvenc_confirmParameter( c, c->m_inputBitDepth[0] != 8 && c->m_inputBitDepth[0] != 10,                  "Input bitdepth must be 8 or 10 bit" );
  vvenc_confirmParameter( c, c->m_internalBitDepth[0] != 8 && c->m_internalBitDepth[0] != 10,            "Internal bitdepth must be 8 or 10 bit" );

  vvenc_confirmParameter( c, c->m_FrameRate <= 0,                                                        "Frame rate must be greater than 0" );
  vvenc_confirmParameter( c, c->m_FrameScale <= 0,                                                       "Frame scale must be greater than 0" );
  vvenc_confirmParameter( c, c->m_TicksPerSecond < -1 || c->m_TicksPerSecond == 0 || c->m_TicksPerSecond > 27000000, "TicksPerSecond must be in range from 1 to 27000000, or -1 for ticks per frame=1" );

  std::stringstream css;
  if ( c->m_FrameScale != 1  && c->m_FrameScale != 1001 && c->m_TicksPerSecond == VVENC_TICKS_PER_SEC_DEF )
  {
    double dFrameRate = c->m_FrameRate/(double)c->m_FrameScale;
    css << "Detected non-standard Frame Rate " << c->m_FrameRate << "/" << c->m_FrameScale;
    css << " (" << std::fixed << std:: setprecision(2) << dFrameRate <<  std::setprecision(-1) << " Hz).";
    css << " Default TicksPerSecond (" << VVENC_TICKS_PER_SEC_DEF << ") can not be used.";

    if ( c->m_FrameRate * c->m_FrameScale <= VVENC_TICKS_PER_SEC_DEF )
    {
      css << " possible TicksPerSecond: " << (c->m_FrameRate * c->m_FrameScale) << std::endl;
    }
    else
    {
      css << " cannot find a proper value for TicksPerSecond in the range (1-" << VVENC_TICKS_PER_SEC_DEF << ")" << std::endl;
    }
  }
  else
  {
    css << "TicksPerSecond must be a multiple of FrameRate/Framescale (" << c->m_FrameRate << "/" << c->m_FrameScale << "). Use 27000000 for NTSC content"  << std::endl;
  }
  vvenc_confirmParameter( c, ( c->m_TicksPerSecond > 0 && c->m_FrameRate > 0) && ((int64_t)c->m_TicksPerSecond*(int64_t)c->m_FrameScale)%c->m_FrameRate, css.str().c_str() );


  vvenc_confirmParameter( c, c->m_numThreads < -1 || c->m_numThreads > 256,                              "Number of threads out of range (-1 <= t <= 256)");

  vvenc_confirmParameter( c, c->m_IntraPeriod < -1,                                                      "IDR period (in frames) must be >= -1");
  vvenc_confirmParameter( c, c->m_IntraPeriodSec < 0,                                                    "IDR period (in seconds) must be >= 0");

  vvenc_confirmParameter( c, c->m_GOPSize < 1 || c->m_GOPSize > VVENC_MAX_GOP,                           "GOP Size must be between 1 and 64" );
  vvenc_confirmParameter( c, c->m_leadFrames < 0 || c->m_leadFrames > VVENC_MAX_GOP,                     "Lead frames exceeds supported range (0 to 64)" );
  vvenc_confirmParameter( c, c->m_trailFrames < 0 || c->m_trailFrames > VVENC_MCTF_RANGE,                "Trail frames exceeds supported range (0 to 4)" );
  vvenc_confirmParameter( c, c->m_sliceTypeAdapt < -1 || c->m_sliceTypeAdapt > 2,                        "Slice type adaptation (STA) invalid parameter given, range is (-1 .. 2)" );

  if( VVENC_RC_OFF == c->m_RCTargetBitrate )
  {
    vvenc_confirmParameter( c, c->m_bufferingPeriodSEIEnabled, "Enabling bufferingPeriod SEI requires rate control" );
    vvenc_confirmParameter( c, c->m_pictureTimingSEIEnabled,   "Enabling pictureTiming SEI requires rate control" );
    vvenc_confirmParameter( c, c->m_RCMaxBitrate > 0,          "Specifying a maximum bitrate requires rate control" );
  }
  else if ( c->m_RCMaxBitrate <= 0 )
  {
    c->m_RCMaxBitrate = INT32_MAX;
  }

  vvenc_confirmParameter( c, c->m_HdrMode < VVENC_HDR_OFF || c->m_HdrMode > VVENC_SDR_BT470BG,  "Sdr/Hdr Mode must be in the range 0 - 8" );

  vvenc_confirmParameter( c, c->m_verbosity < VVENC_SILENT || c->m_verbosity > VVENC_DETAILS, "verbosity is out of range[0..6]" );

  vvenc_confirmParameter( c,  (c->m_numIntraModesFullRD < -1 || c->m_numIntraModesFullRD == 0 || c->m_numIntraModesFullRD > 3), "NumIntraModesFullRD must be -1 or between 1 and 3");

#if ! ENABLE_TRACING
  vvenc_confirmParameter( c, c->m_traceFile[0] != '\0', "trace file option '--tracefile' set, but encoder lib not compiled with tracing support, use make ... enable-tracing=1 or set ENABLE_TRACING" );
  vvenc_confirmParameter( c, c->m_traceRule[0] != '\0', "trace rule option '--tracerule' set, but encoder lib not compiled with tracing support, use make ... enable-tracing=1 or set ENABLE_TRACING" );
  vvenc_confirmParameter( c, c->m_listTracingChannels, "list trace channels option '--tracechannellist' set, but encoder lib not compiled with tracing support, use make ... enable-tracing=1 or set ENABLE_TRACING" );
#endif

  if ( c->m_confirmFailed )
  {
    return c->m_confirmFailed;
  }

  //
  // set a lot of dependent parameters
  //

  vvenc::MsgLog msg(c->m_msgCtx, c->m_msgFnc);

  if( c->m_FirstPassMode > 2 && c->m_RCTargetBitrate != 0 )
  {
    // lookahead will only be set to 0 later
    if( c->m_LookAhead > 0 || c->m_RCNumPasses == 1 || std::min( c->m_SourceWidth, c->m_SourceHeight ) < 720 ) 
    {
      c->m_FirstPassMode -= 2;  
      msg.log( VVENC_NOTICE, "First pass spatial downsampling (FirstPassMode=3/4) cannot be used for single pass encoding and videos with resolution lower than 720p, changing FirstPassMode to %d!\n", c->m_FirstPassMode );
    }
  }

  const double d = (3840.0 * 2160.0) / double (c->m_SourceWidth * c->m_SourceHeight);
  const int rcQP = (c->m_RCInitialQP > 0 ? std::min (vvenc::MAX_QP, c->m_RCInitialQP) : std::max (0, vvenc::MAX_QP_PERCEPT_QPA - (c->m_FirstPassMode > 2 ? 4 : 2) - int (0.5 + sqrt ((d * std::max (0, c->m_RCTargetBitrate)) / 500000.0))));

  // TODO 2.0: make this an error
  //vvenc_confirmParameter( c, c->m_RCTargetBitrate != VVENC_RC_OFF && c->m_QP != VVENC_AUTO_QP && c->m_QP != VVENC_DEFAULT_QP, "Rate-control and QP based encoding are mutually exclusive!" );

  if( c->m_RCTargetBitrate != VVENC_RC_OFF && c->m_QP != VVENC_AUTO_QP && c->m_QP != rcQP )
  {
    if( c->m_QP != VVENC_DEFAULT_QP )
      msg.log( VVENC_WARNING, "Configuration warning: Rate control is enabled since a target bitrate is specified, ignoring QP value\n\n" );
    c->m_QP = VVENC_AUTO_QP;
  }

  if( c->m_QP == VVENC_AUTO_QP ) c->m_QP = ( c->m_RCTargetBitrate != VVENC_RC_OFF ? rcQP : VVENC_DEFAULT_QP );

  vvenc_confirmParameter( c, c->m_RCTargetBitrate == VVENC_RC_OFF && ( c->m_QP < 0 || c->m_QP > vvenc::MAX_QP ), "QP exceeds supported range (0 to 63)" );

  vvenc_confirmParameter( c, c->m_RCTargetBitrate != VVENC_RC_OFF && ( c->m_RCTargetBitrate < 0 || c->m_RCTargetBitrate > 800000000 ),  "TargetBitrate must be between 0 and 800000000" );
  vvenc_confirmParameter( c, c->m_RCTargetBitrate != VVENC_RC_OFF && (int64_t) c->m_RCMaxBitrate * 2 < (int64_t) c->m_RCTargetBitrate * 3, "MaxBitrate must be at least 1.5*TargetBitrate" );
  vvenc_confirmParameter( c, c->m_RCTargetBitrate != VVENC_RC_OFF && ( c->m_FirstPassMode < 0 || c->m_FirstPassMode > 4 ), "FirstPassMode must be 0, 1, 2, 3, or 4" );

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

    if( c->m_framesToBeEncoded == 1 )
    {
      if( c->m_profile == vvencProfile::VVENC_MAIN_10     ) c->m_profile = vvencProfile::VVENC_MAIN_10_STILL_PICTURE;
      if( c->m_profile == vvencProfile::VVENC_MAIN_10_444 ) c->m_profile = vvencProfile::VVENC_MAIN_10_444_STILL_PICTURE;
    }

    vvenc_confirmParameter( c, c->m_profile == vvencProfile::VVENC_PROFILE_AUTO, "Unable to infer profile from input!" );
  }

  if( c->m_level == vvencLevel::VVENC_LEVEL_AUTO )
  {
    c->m_level = vvenc::LevelTierFeatures::getLevelForInput( c->m_SourceWidth, c->m_SourceHeight, c->m_levelTier, c->m_FrameRate, c->m_FrameScale, c->m_RCTargetBitrate );
    vvenc_confirmParameter( c, c->m_level == vvencLevel::VVENC_LEVEL_AUTO || c->m_level == vvencLevel::VVENC_NUMBER_OF_LEVELS, "Unable to infer level from input!" );
  }
  else
  {
    const vvencLevel inferedLevel = vvenc::LevelTierFeatures::getLevelForInput( c->m_SourceWidth, c->m_SourceHeight, c->m_levelTier, c->m_FrameRate, c->m_FrameScale, c->m_RCTargetBitrate );
    vvenc_confirmParameter( c, c->m_level < inferedLevel, "The level set is too low given the input dimensions (size/rate)!" );
  }

  {
    const vvenc::ProfileFeatures *profileFeatures = vvenc::ProfileFeatures::getProfileFeatures( c->m_profile );
    vvenc_confirmParameter( c, !profileFeatures, "Invalid profile!" );
    vvenc_confirmParameter( c, c->m_level == vvencLevel::VVENC_LEVEL15_5 && !profileFeatures->canUseLevel15p5, "The video dimensions (size/rate) exceed the allowed maximum throughput for the level/profile combination!" );
  }

  c->m_maxBT[0] = std::min( c->m_CTUSize, c->m_maxBT[0] );
  c->m_maxBT[1] = std::min( c->m_CTUSize, c->m_maxBT[1] );
  c->m_maxBT[2] = std::min( c->m_CTUSize, c->m_maxBT[2] );

  c->m_maxTT[0] = std::min( c->m_CTUSize, c->m_maxTT[0] );
  c->m_maxTT[1] = std::min( c->m_CTUSize, c->m_maxTT[1] );
  c->m_maxTT[2] = std::min( c->m_CTUSize, c->m_maxTT[2] );

  if( c->m_maxMTTDepthIChroma < 0 )
  {
    c->m_maxMTTDepthIChroma = c->m_maxMTTDepthI;
  }

  // rate control
  if( c->m_RCNumPasses < 0 )
  {
    c->m_RCNumPasses = ( c->m_RCPass > 0 ? 2 : 1 ); // single pass by default (SDK usage)
  }
  if ( c->m_LookAhead < 0 )
  {
    c->m_LookAhead = c->m_RCTargetBitrate > 0 && c->m_RCNumPasses == 1 ? 1 : 0;
  }

  // threading
  if( c->m_numThreads < 0 )
  {
    const int numCores = std::thread::hardware_concurrency();
    c->m_numThreads = std::min( c->m_SourceWidth, c->m_SourceHeight ) < 720 ? 4 : 8;
    c->m_numThreads = std::min( c->m_numThreads, numCores );
  }
  if( c->m_ensureWppBitEqual < 0 )       c->m_ensureWppBitEqual     = c->m_numThreads ?      1   : 0   ;
  if( c->m_useAMaxBT < 0 )               c->m_useAMaxBT             = c->m_numThreads ?      0   : 1   ;
  if( c->m_cabacInitPresent < 0 )        c->m_cabacInitPresent      = c->m_numThreads ?      0   : 1   ;
  if( c->m_alfTempPred < 0 )             c->m_alfTempPred           = c->m_fppLinesSynchro ? 0   : 1   ;
  if( c->m_saoEncodingRate < 0.0 )       c->m_saoEncodingRate       = c->m_numThreads ?      0.0 : 0.75;
  if( c->m_saoEncodingRateChroma < 0.0 ) c->m_saoEncodingRateChroma = c->m_numThreads ?      0.0 : 0.5 ;
  if( c->m_maxParallelFrames < 0 )
  {
    c->m_maxParallelFrames = std::min( c->m_numThreads, 4 );
  }

  if( c->m_alfUnitSize < 0 )
    c->m_alfUnitSize = c->m_CTUSize;

  // quantization threshold
  if( c->m_quantThresholdVal < 0 )
  {
    c->m_quantThresholdVal = 8;
  }

  /* rules for input, output and internal bitdepths as per help text */
  if (c->m_MSBExtendedBitDepth[0] == 0)
    c->m_MSBExtendedBitDepth  [0] = c->m_inputBitDepth      [0];
  if (c->m_MSBExtendedBitDepth[1] == 0)
    c->m_MSBExtendedBitDepth  [1] = c->m_MSBExtendedBitDepth[0];
  if (c->m_internalBitDepth   [0] == 0)
    c->m_internalBitDepth     [0] = c->m_MSBExtendedBitDepth[0];
  if (c->m_internalBitDepth   [1] == 0)
    c->m_internalBitDepth     [1] = c->m_internalBitDepth   [0];
  if (c->m_inputBitDepth      [1] == 0)
    c->m_inputBitDepth        [1] = c->m_inputBitDepth      [0];
  if (c->m_outputBitDepth     [0] == 0)
    c->m_outputBitDepth       [0] = c->m_internalBitDepth   [0];
  if (c->m_outputBitDepth     [1] == 0)
    c->m_outputBitDepth       [1] = c->m_outputBitDepth     [0];

  if( c->m_HdrMode == VVENC_HDR_OFF &&
     (( c->m_masteringDisplay[0] != 0 && c->m_masteringDisplay[1] != 0 && c->m_masteringDisplay[8] != 0 && c->m_masteringDisplay[9] != 0 ) ||
     ( c->m_contentLightLevel[0] != 0 && c->m_contentLightLevel[1] != 0 ) ) )
  {
    // enable hdr pq bt2020/bt709 mode (depending on set colour primaries)
    c->m_HdrMode = c->m_colourPrimaries==9 ? VVENC_HDR_PQ_BT2020 : VVENC_HDR_PQ;
  }

  if ( c->m_HdrMode != VVENC_HDR_OFF && c->m_HdrMode != VVENC_HDR_USER_DEFINED )
  {
    // VUI and SEI options
    c->m_colourDescriptionPresent = true;                                // enable colour_primaries, transfer_characteristics and matrix_coefficients in vui
    if( c->m_vuiParametersPresent < 0 )
    {
      c->m_vuiParametersPresent  = 1;                                    // enable vui only if not explicitly disabled
    }
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
  else if( c->m_HdrMode == VVENC_SDR_BT709 )
  {
    c->m_transferCharacteristics = 1;  // bt709
    c->m_colourPrimaries         = 1;  // bt709
    c->m_matrixCoefficients      = 1;  // bt709
  }
  else if( c->m_HdrMode == VVENC_SDR_BT2020 )
  {
    c->m_transferCharacteristics = 14; // bt2020-10
    c->m_colourPrimaries         = 9;  // bt2020nc
    c->m_matrixCoefficients      = 9;  // bt2020nc
  }
  else if( c->m_HdrMode == VVENC_SDR_BT470BG )
  {
    c->m_transferCharacteristics = 5;  // bt470bg
    c->m_colourPrimaries         = 5;  // bt470bg
    c->m_matrixCoefficients      = 5;  // bt470bg
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
      const int minCuSize = std::max( 1 << ( vvenc::MIN_CU_LOG2 + 1 ), 1 << c->m_log2MinCodingBlockSize );
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
      if( ( c->m_confWinLeft == 0 ) && ( c->m_confWinRight == 0 ) && ( c->m_confWinTop == 0 ) && ( c->m_confWinBottom == 0 ) )
      {
        msg.log( VVENC_WARNING, "Configuration warning: conformance window enabled, but all conformance window parameters set to zero\n\n" );
      }
      if( ( c->m_aiPad[1] != 0 ) || ( c->m_aiPad[0] != 0 ) )
      {
        msg.log( VVENC_WARNING, "Configuration warning: conformance window enabled, padding parameters will be ignored\n" );
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

  int fps = c->m_FrameRate/c->m_FrameScale;

  c->m_reshapeCW.rspFps     = fps;
  c->m_reshapeCW.rspPicSize = c->m_PadSourceWidth*c->m_PadSourceHeight;
  c->m_reshapeCW.rspFpsToIp = std::max(16, 16 * (int)(round((double)c->m_reshapeCW.rspFps/16.0)));
  c->m_reshapeCW.updateCtrl = c->m_updateCtrl;
  c->m_reshapeCW.adpOption  = c->m_adpOption;
  c->m_reshapeCW.initialCW  = c->m_initialCW;
  
  if( c->m_DecodingRefreshType == VVENC_DRT_IDR2 )
  {
    msg.log( VVENC_WARNING, "Configuration warning: DecodingRefreshType IDR2 is deprecated\n\n" );
    vvenc_confirmParameter( c, c->m_poc0idr, "for using deprecated IDR2, POC0IDR has to be disabled" );
    c->m_DecodingRefreshType = VVENC_DRT_IDR;
  }

  if( c->m_rprEnabledFlag == -1 )
  {
    c->m_rprEnabledFlag = c->m_DecodingRefreshType == VVENC_DRT_CRA_CRE ? 2 : 0;
  }

  vvenc_confirmParameter( c, c->m_rprEnabledFlag < -1 || c->m_rprEnabledFlag > 2, "RPR must be either -1, 0, 1 or 2" );
  vvenc_confirmParameter( c, c->m_rprEnabledFlag == 2 && c->m_DecodingRefreshType != VVENC_DRT_CRA_CRE, "for using RPR=2 constrained rasl encoding, DecodingRefreshType has to be set to VVENC_DRT_CRA_CRE" );

  if( c->m_rprEnabledFlag == 2 )
  {
    c->m_resChangeInClvsEnabled = true;
    c->m_craAPSreset            = true;
    c->m_rprRASLtoolSwitch      = true;
  }
  
  if( c->m_maxPicWidth > 0 && c->m_maxPicHeight > 0 )
  {
    vvenc_confirmParameter( c, !c->m_rprEnabledFlag || !c->m_resChangeInClvsEnabled, "if max picture size is set, both RPR and resChangeInClvsEnabled have to be enabled" );
  }

  if( c->m_IntraPeriod == 0 && c->m_IntraPeriodSec > 0 )
  {  
    int idrPeriod = fps * c->m_IntraPeriodSec;
    if( idrPeriod % c->m_GOPSize != 0 )
    {
      const int minGopSize = std::min( (fps * c->m_IntraPeriodSec), std::min( c->m_GOPSize, 8 ));   
      if( idrPeriod < c->m_GOPSize )
      {
        if( (idrPeriod % minGopSize) != 0)
        {
          idrPeriod = (fps > minGopSize ) ? (idrPeriod - (minGopSize>>1)) : minGopSize;
          while ( idrPeriod % minGopSize != 0 )
          {
            idrPeriod++;
          }
        }
      }
      else
      {
        int diff = idrPeriod % minGopSize;
        if( diff < minGopSize >> 1 )
        {
          idrPeriod -= diff;
        }
        else
        {
          idrPeriod += (minGopSize - diff);
        }
      }
    }
    c->m_IntraPeriod = idrPeriod;
  }
  
  if( c->m_IntraPeriod == 1 && !c->m_poc0idr )
  {
    c->m_poc0idr = true;
  }

  if( c->m_IntraPeriod == 1 && c->m_GOPSize != 1 )
  {
    // TODO 2.0: make this an error
    msg.log( VVENC_WARNING, "Configuration warning: IntraPeriod is 1, thus GOPSize is set to 1 too and given gop structures are resetted\n\n" );
    c->m_GOPSize = 1;
    for( int i = 0; i < VVENC_MAX_GOP; i++ )
    {
      vvenc_GOPEntry_default( &c->m_GOPList[i] );
    }
  }
  vvenc_confirmParameter( c, c->m_IntraPeriod == 0, "intra period must not be equal 0" );
  
  vvenc_confirmParameter( c, !c->m_poc0idr && ( c->m_IntraPeriod == 1 || !c->m_picReordering ), "when POC 0 is not an IDR frame it is only possible for random access, for all intra and low delay encoding POC0IDR must be set!" );

  if( c->m_IntraPeriod >= 16 && c->m_GOPSize >= 16 && c->m_IntraPeriod % c->m_GOPSize >= 1 && c->m_IntraPeriod % c->m_GOPSize <= 4 )
  {
    msg.log( VVENC_WARNING, "Configuration warning: setting IntraPeriod in the range of ( N * GOPSize + 1 ) .. ( N * GOPSize + 4 ), i.e. only a small distance above a multiple of the GOPSize, will lead to degraded results.\n" );
    msg.log( VVENC_WARNING, "                       consider changing the IntraPeriod for better results. For optimal results, set the IntraPeriod to a multiple of GOPSize.\n\n" );
  }

  if( c->m_GOPSize > 1 && c->m_GOPList[ 0 ].m_POC != -1  )
  {
    bool bPicReordering = false;
    for( int i = 1; i < c->m_GOPSize; i++ )
    {
      if( c->m_GOPList[ i - 1 ].m_POC > c->m_GOPList[ i ].m_POC )
      {
        bPicReordering = true;
        break;
      }
    }
    vvenc_confirmParameter( c, ! c->m_picReordering && bPicReordering, "PicReordering disabled, but given GOP configuration uses picture reordering" );
    if( c->m_picReordering && ! bPicReordering )
    {
      msg.log( VVENC_WARNING, "Configuration warning: PicReordering enabled, but not used in given GOP configuration, disabling PicReordering\n\n" );
      c->m_picReordering = false;
    }
  }

  // slice type adaptation (STA)
  if( c->m_sliceTypeAdapt < 0 )
  {
    c->m_sliceTypeAdapt = c->m_GOPSize > 8 ? 1 : 0;
  }
  vvenc_confirmParameter( c, c->m_GOPSize <= 8 && c->m_sliceTypeAdapt > 0, "Slice type adaptation for GOPSize <= 8 not supported" );

  // set number of lead / trail frames in segment mode
  const int staFrames  = c->m_sliceTypeAdapt                       ? c->m_GOPSize     : 0;
  const int mctfFrames = c->m_vvencMCTF.MCTF || c->m_usePerceptQPA ? VVENC_MCTF_RANGE : 0;
  switch( c->m_SegmentMode )
  {
    case VVENC_SEG_FIRST:
      c->m_leadFrames  = 0;
      c->m_trailFrames = mctfFrames;
      break;
    case VVENC_SEG_MID:
      c->m_leadFrames  = std::max( staFrames, mctfFrames );
      c->m_trailFrames = mctfFrames;
      c->m_poc0idr     = true;
      break;
    case VVENC_SEG_LAST:
      c->m_leadFrames  = std::max( staFrames, mctfFrames );
      c->m_trailFrames = 0;
      c->m_poc0idr     = true;
      break;
    default:
      // do nothing
      break;
  }

  vvenc_confirmParameter( c, c->m_trailFrames > 0 && c->m_framesToBeEncoded <= 0, "If number of trailing frames is given, the total number of frames to be encoded has to be set" );

  //
  // do some check and set of parameters next
  //

  if ( c->m_lumaReshapeEnable )
  {
    if ( c->m_updateCtrl > 0 && c->m_adpOption > 2 ) { c->m_adpOption -= 2; }
  }

  if ( c->m_JointCbCrMode && ( c->m_internChromaFormat == VVENC_CHROMA_400 ) )
  {
    c->m_JointCbCrMode = false;
  }

  if( c->m_vvencMCTF.MCTFUnitSize == -1 )
  {
    c->m_vvencMCTF.MCTFUnitSize = std::min( c->m_SourceWidth, c->m_SourceHeight ) < 720 ? 8 : 16;
  }

  if ( c->m_vvencMCTF.MCTF && c->m_vvencMCTF.numFrames == 0 && c->m_vvencMCTF.numStrength == 0 )
  {
    const int log2GopSize = std::min<int>( 6, vvenc::floorLog2( c->m_GOPSize ) );

    c->m_vvencMCTF.numFrames = c->m_vvencMCTF.numStrength = std::max( 1, log2GopSize - ( ( c->m_QP - ( c->m_RCTargetBitrate > 0 ? 1 : 0 ) ) >> 4 ) );

    for ( int i = 0; i < c->m_vvencMCTF.numFrames; i++ )
    {
      c->m_vvencMCTF.MCTFFrames[i] = c->m_GOPSize >> ( c->m_vvencMCTF.numFrames - i - 1 );
      c->m_vvencMCTF.MCTFStrengths[i] = vvenc::Clip3( 0.0, 2.0, ( c->m_QP - 4.0 ) / 8.0 ) / double ( c->m_vvencMCTF.numFrames - i );
    }
    c->m_vvencMCTF.MCTFStrengths[c->m_vvencMCTF.numFrames - 1] = vvenc::Clip3( 0.0, 1.5, ( c->m_QP - 4.0 ) * 3.0 / 32.0 );
  }

  vvenc_confirmParameter( c, c->m_blockImportanceMapping && !c->m_vvencMCTF.MCTF, "BIM (block importance mapping) cannot be enabled when MCTF is disabled!" );
  vvenc_confirmParameter( c, c->m_blockImportanceMapping && c->m_vvencMCTF.MCTFUnitSize > c->m_CTUSize, "MCTFUnitSize cannot exceed CTUSize if BIM is enabled!" );

  if ( c->m_usePerceptQPATempFiltISlice < 0 )
  {
    c->m_usePerceptQPATempFiltISlice = 0;
    if ( c->m_usePerceptQPA ) // automatic mode for temporal filtering depending on RC
    {
      c->m_usePerceptQPATempFiltISlice = ( c->m_RCTargetBitrate > 0 && c->m_RCNumPasses == 2 ? 2 : 1 );
    }
  }
  if ( c->m_usePerceptQPATempFiltISlice == 2
      && ( c->m_QP <= 27 || c->m_QP > vvenc::MAX_QP_PERCEPT_QPA || c->m_GOPSize <= 8 || c->m_IntraPeriod < 2 * c->m_GOPSize ) )
  {
    c->m_usePerceptQPATempFiltISlice = 1; // disable temporal pumping reduction aspect
  }
  if ( c->m_usePerceptQPATempFiltISlice > 0
      && ( c->m_vvencMCTF.MCTF == 0 || ! c->m_usePerceptQPA ) )
  {
    c->m_usePerceptQPATempFiltISlice = 0; // fully disable temporal filtering features
  }

  if ( c->m_cuQpDeltaSubdiv < 0)
  {
    c->m_cuQpDeltaSubdiv = 0;
    if ( c->m_usePerceptQPA
        && c->m_QP <= vvenc::MAX_QP_PERCEPT_QPA
        && ( c->m_CTUSize == 128 || ( c->m_CTUSize == 64 && std::min( c->m_SourceWidth, c->m_SourceHeight ) < 720 ) )
        && std::min( c->m_SourceWidth, c->m_SourceHeight ) <= 1280 )
    {
      c->m_cuQpDeltaSubdiv = 2;
    }
  }
  vvenc_confirmParameter( c, c->m_sliceChromaQpOffsetPeriodicity < -1 || c->m_sliceChromaQpOffsetPeriodicity > 1, "Only values {-1, 0, 1} supported for SliceChromaQPOffsetPeriodicity" );
  if ( c->m_sliceChromaQpOffsetPeriodicity < 0)
  {
    c->m_sliceChromaQpOffsetPeriodicity = 0;
    if ( c->m_usePerceptQPA && c->m_internChromaFormat != VVENC_CHROMA_400 )
    {
      c->m_sliceChromaQpOffsetPeriodicity = 1;
    }
  }

  if( c->m_treatAsSubPic )
  {
    if( c->m_sliceTypeAdapt )    msg.log( VVENC_WARNING, "Configuration warning: combination of TreatAsSubPic and STA may not work with VTM subPicMerge tool, consider disabling STA\n\n" );
    if( c->m_alfTempPred )       msg.log( VVENC_WARNING, "Configuration warning: disable ALF temporal prediction, when generation of subpicture streams is enabled (TreatAsSubPic)\n\n" );
    if( c->m_JointCbCrMode )     msg.log( VVENC_WARNING, "Configuration warning: disable joint coding of chroma residuals, when generation of subpicture streams is enabled (TreatAsSubPic)\n\n" );
    if( c->m_lumaReshapeEnable ) msg.log( VVENC_WARNING, "Configuration warning: disable LMCS luma mapping with chroma scaling, when generation of subpicture streams is enabled (TreatAsSubPic)\n\n" );
    c->m_alfTempPred       = 0;
    c->m_JointCbCrMode     = false;
    c->m_lumaReshapeEnable = 0;
    c->m_reshapeSignalType = 0;
    c->m_updateCtrl        = 0;
    c->m_adpOption         = 0;
    c->m_initialCW         = 0;
    c->m_LMCSOffset        = 0;
    c->m_useAMaxBT         = 0;
    vvenc_ReshapeCW_default( &c->m_reshapeCW );
  }

  const bool autoGop = c->m_GOPList[0].m_POC;

  if( c->m_numRefPicsSCC < 0 )
  {
    c->m_numRefPicsSCC = c->m_numRefPics;
  }

  if( c->m_GOPList[ 0 ].m_POC == -1 || ( c->m_addGOP32refPics && c->m_GOPSize == 32 ) )
  {
    if( c->m_IntraPeriod == 1 || c->m_GOPSize == 1 )
    {
      vvenc_confirmParameter( c, c->m_GOPSize != 1,     "gop auto configuration for all intra supports only gop size 1" );
      vvenc_confirmParameter( c, c->m_IntraPeriod != 1, "gop auto configuration for gop size 1 supports only all intra" );
      //                                    m_sliceType                m_QPOffsetModelOffset       m_temporalId   m_numRefPicsActive[ 0 ]            m_numRefPicsActive[ 1 ]
      //                                     |      m_POC               |      m_QPOffsetModelScale |              |   m_deltaRefPics[ 0 ]            |   m_deltaRefPics[ 1 ]
      //                                     |       |    m_QPOffset    |        |    m_QPFactor    |              |    |                             |    |
      c->m_GOPList[  0 ] = vvenc::GOPEntry( 'I',     1,    0,           0.0,     0.0,  0.0,         0,             0,   { },                          0,   { } );
    }
    else if( c->m_GOPSize == 8 )
    {
      vvenc_confirmParameter( c, c->m_picReordering, "gop auto configuration for gop size 8 only without picture reordering supported" );
      //                                    m_sliceType                m_QPOffsetModelOffset       m_temporalId   m_numRefPicsActive[ 0 ]            m_numRefPicsActive[ 1 ]
      //                                     |      m_POC               |      m_QPOffsetModelScale |              |   m_deltaRefPics[ 0 ]            |   m_deltaRefPics[ 1 ]
      //                                     |       |    m_QPOffset    |        |    m_QPFactor    |              |    |                             |    |
      c->m_GOPList[  0 ] = vvenc::GOPEntry( 'B',     1,    5,         -6.5,   0.2590,  1.0,         0,             4,   { 1, 9, 17, 25 },             4,   { 1, 9, 17, 25 } );
      c->m_GOPList[  1 ] = vvenc::GOPEntry( 'B',     2,    4,         -6.5,   0.2590,  1.0,         0,             4,   { 1, 2, 10, 18 },             4,   { 1, 2, 10, 18 } );
      c->m_GOPList[  2 ] = vvenc::GOPEntry( 'B',     3,    5,         -6.5,   0.2590,  1.0,         0,             4,   { 1, 3, 11, 19 },             4,   { 1, 3, 11, 19 } );
      c->m_GOPList[  3 ] = vvenc::GOPEntry( 'B',     4,    4,         -6.5,   0.2590,  1.0,         0,             4,   { 1, 4, 12, 20 },             4,   { 1, 4, 12, 20 } );
      c->m_GOPList[  4 ] = vvenc::GOPEntry( 'B',     5,    5,         -6.5,   0.2590,  1.0,         0,             4,   { 1, 5, 13, 21 },             4,   { 1, 5, 13, 21 } );
      c->m_GOPList[  5 ] = vvenc::GOPEntry( 'B',     6,    4,         -6.5,   0.2590,  1.0,         0,             4,   { 1, 6, 14, 22 },             4,   { 1, 6, 14, 22 } );
      c->m_GOPList[  6 ] = vvenc::GOPEntry( 'B',     7,    5,         -6.5,   0.2590,  1.0,         0,             4,   { 1, 7, 15, 23 },             4,   { 1, 7, 15, 23 } );
      c->m_GOPList[  7 ] = vvenc::GOPEntry( 'B',     8,    1,          0.0,      0.0,  1.0,         0,             4,   { 1, 8, 16, 24 },             4,   { 1, 8, 16, 24 } );
    }
    else if( c->m_GOPSize == 16 )
    {
      //                                    m_sliceType                m_QPOffsetModelOffset       m_temporalId   m_numRefPicsActive[ 0 ]            m_numRefPicsActive[ 1 ]
      //                                     |      m_POC               |      m_QPOffsetModelScale |              |   m_deltaRefPics[ 0 ]            |   m_deltaRefPics[ 1 ]
      //                                     |       |    m_QPOffset    |        |    m_QPFactor    |              |    |                             |    |
      c->m_GOPList[  0 ] = vvenc::GOPEntry( 'B',    16,    1,           0.0,     0.0,  1.0,         0,             2,   { 16, 32, 24     },           2,   { 16,  32           } );
      c->m_GOPList[  1 ] = vvenc::GOPEntry( 'B',     8,    1,       -4.8848,  0.2061,  1.0,         1,             2,   {  8, 16         },           2,   { -8,   8           } );
      c->m_GOPList[  2 ] = vvenc::GOPEntry( 'B',     4,    4,       -5.7476,  0.2286,  1.0,         2,             2,   {  4, 12         },           2,   { -4, -12           } );
      c->m_GOPList[  3 ] = vvenc::GOPEntry( 'B',     2,    5,         -5.90,  0.2333,  1.0,         3,             2,   {  2, 10         },           2,   { -2,  -6, -14      } );
      c->m_GOPList[  4 ] = vvenc::GOPEntry( 'B',     1,    6,       -7.1444,     0.3,  1.0,         4,             2,   {  1, -1         },           2,   { -1,  -3,  -7, -15 } );
      c->m_GOPList[  5 ] = vvenc::GOPEntry( 'B',     3,    6,       -7.1444,     0.3,  1.0,         4,             2,   {  1,  3         },           2,   { -1,  -5, -13      } );
      c->m_GOPList[  6 ] = vvenc::GOPEntry( 'B',     6,    5,         -5.90,  0.2333,  1.0,         3,             2,   {  2,  6         },           2,   { -2, -10           } );
      c->m_GOPList[  7 ] = vvenc::GOPEntry( 'B',     5,    6,       -7.1444,     0.3,  1.0,         4,             2,   {  1,  5         },           2,   { -1,  -3, -11      } );
      c->m_GOPList[  8 ] = vvenc::GOPEntry( 'B',     7,    6,       -7.1444,     0.3,  1.0,         4,             2,   {  1,  3,  7     },           2,   { -1,  -9           } );
      c->m_GOPList[  9 ] = vvenc::GOPEntry( 'B',    12,    4,       -5.7476,  0.2286,  1.0,         2,             2,   {  4, 12         },           2,   { -4,   4           } );
      c->m_GOPList[ 10 ] = vvenc::GOPEntry( 'B',    10,    5,         -5.90,  0.2333,  1.0,         3,             2,   {  2, 10         },           2,   { -2,  -6           } );
      c->m_GOPList[ 11 ] = vvenc::GOPEntry( 'B',     9,    6,       -7.1444,     0.3,  1.0,         4,             2,   {  1,  9         },           2,   { -1,  -3,  -7      } );
      c->m_GOPList[ 12 ] = vvenc::GOPEntry( 'B',    11,    6,       -7.1444,     0.3,  1.0,         4,             2,   {  1,  3, 11     },           2,   { -1,  -5           } );
      c->m_GOPList[ 13 ] = vvenc::GOPEntry( 'B',    14,    5,         -5.90,  0.2333,  1.0,         3,             2,   {  2,  6, 14     },           2,   { -2,   2           } );
      c->m_GOPList[ 14 ] = vvenc::GOPEntry( 'B',    13,    6,       -7.1444,     0.3,  1.0,         4,             2,   {  1,  5, 13     },           2,   { -1,  -3           } );
      c->m_GOPList[ 15 ] = vvenc::GOPEntry( 'B',    15,    6,       -7.1444,     0.3,  1.0,         4,             2,   {  1,  3,  7, 15 },           2,   { -1,   1           } );
    }
    else if( c->m_GOPSize == 32 )
    {
      if( !c->m_addGOP32refPics )
      {
        //                                    m_sliceType                m_QPOffsetModelOffset       m_temporalId   m_numRefPicsActive[ 0 ]            m_numRefPicsActive[ 1 ]
        //                                     |      m_POC               |      m_QPOffsetModelScale |              |   m_deltaRefPics[ 0 ]            |   m_deltaRefPics[ 1 ]
        //                                     |       |    m_QPOffset    |        |    m_QPFactor    |              |    |                             |    |
        c->m_GOPList[  0 ] = vvenc::GOPEntry( 'B',    32,   -1,           0.0,     0.0,  1.0,         0,             2,   { 32, 64, 48     },           2,   {  32,  64                } );
        c->m_GOPList[  1 ] = vvenc::GOPEntry( 'B',    16,    0,       -4.9309,  0.2265,  1.0,         1,             2,   { 16, 32         },           2,   { -16,  16                } );
        c->m_GOPList[  2 ] = vvenc::GOPEntry( 'B',     8,    0,       -3.0625,  0.1875,  1.0,         2,             2,   {  8, 24         },           2,   {  -8, -24                } );
        c->m_GOPList[  3 ] = vvenc::GOPEntry( 'B',     4,    3,       -5.4095,  0.2571,  1.0,         3,             2,   {  4, 20         },           2,   {  -4, -12, -28           } );
        c->m_GOPList[  4 ] = vvenc::GOPEntry( 'B',     2,    5,       -4.4895,  0.1947,  1.0,         4,             2,   {  2, 18         },           2,   {  -2,  -6, -14, -30      } );
        c->m_GOPList[  5 ] = vvenc::GOPEntry( 'B',     1,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1, -1         },           2,   {  -1,  -3,  -7, -15, -31 } );
        c->m_GOPList[  6 ] = vvenc::GOPEntry( 'B',     3,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3         },           2,   {  -1,  -5, -13, -29      } );
        c->m_GOPList[  7 ] = vvenc::GOPEntry( 'B',     6,    5,       -4.4895,  0.1947,  1.0,         4,             2,   {  2,  6         },           2,   {  -2, -10, -26           } );
        c->m_GOPList[  8 ] = vvenc::GOPEntry( 'B',     5,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  5         },           2,   {  -1,  -3, -11, -27      } );
        c->m_GOPList[  9 ] = vvenc::GOPEntry( 'B',     7,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3, 7      },           2,   {  -1,  -9, -25           } );
        c->m_GOPList[ 10 ] = vvenc::GOPEntry( 'B',    12,    3,       -5.4095,  0.2571,  1.0,         3,             2,   {  4, 12         },           2,   {  -4, -20                } );
        c->m_GOPList[ 11 ] = vvenc::GOPEntry( 'B',    10,    5,       -4.4895,  0.1947,  1.0,         4,             2,   {  2, 10         },           2,   {  -2,  -6, -22           } );
        c->m_GOPList[ 12 ] = vvenc::GOPEntry( 'B',     9,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  9         },           2,   {  -1,  -3,  -7, -23      } );
        c->m_GOPList[ 13 ] = vvenc::GOPEntry( 'B',    11,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3, 11     },           2,   {  -1,  -5, -21           } );
        c->m_GOPList[ 14 ] = vvenc::GOPEntry( 'B',    14,    5,       -4.4895,  0.1947,  1.0,         4,             2,   {  2,  6, 14     },           2,   {  -2, -18                } );
        c->m_GOPList[ 15 ] = vvenc::GOPEntry( 'B',    13,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  5, 13     },           2,   {  -1,  -3, -19           } );
        c->m_GOPList[ 16 ] = vvenc::GOPEntry( 'B',    15,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3, 15     },           2,   {  -1, -17                } );
        c->m_GOPList[ 17 ] = vvenc::GOPEntry( 'B',    24,    0,       -3.0625,  0.1875,  1.0,         2,             2,   {  8, 24         },           2,   {  -8,   8                } );
        c->m_GOPList[ 18 ] = vvenc::GOPEntry( 'B',    20,    3,       -5.4095,  0.2571,  1.0,         3,             2,   {  4, 20         },           2,   {  -4, -12                } );
        c->m_GOPList[ 19 ] = vvenc::GOPEntry( 'B',    18,    5,       -4.4895,  0.1947,  1.0,         4,             2,   {  2, 18         },           2,   {  -2,  -6, -14           } );
        c->m_GOPList[ 20 ] = vvenc::GOPEntry( 'B',    17,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1, 17         },           2,   {  -1,  -3,  -7, -15      } );
        c->m_GOPList[ 21 ] = vvenc::GOPEntry( 'B',    19,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3, 19     },           2,   {  -1,  -5, -13           } );
        c->m_GOPList[ 22 ] = vvenc::GOPEntry( 'B',    22,    5,       -4.4895,  0.1947,  1.0,         4,             2,   {  2,  6, 22     },           2,   {  -2, -10                } );
        c->m_GOPList[ 23 ] = vvenc::GOPEntry( 'B',    21,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  5, 21     },           2,   {  -1,  -3, -11           } );
        c->m_GOPList[ 24 ] = vvenc::GOPEntry( 'B',    23,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3,  7, 23 },           2,   {  -1,  -9                } );
        c->m_GOPList[ 25 ] = vvenc::GOPEntry( 'B',    28,    3,       -5.4095,  0.2571,  1.0,         3,             2,   {  4, 12, 28     },           2,   {  -4,   4                } );
        c->m_GOPList[ 26 ] = vvenc::GOPEntry( 'B',    26,    5,       -4.4895,  0.1947,  1.0,         4,             2,   {  2, 10, 26     },           2,   {  -2,  -6                } );
        c->m_GOPList[ 27 ] = vvenc::GOPEntry( 'B',    25,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  9, 25     },           2,   {  -1,  -3, -7            } );
        c->m_GOPList[ 28 ] = vvenc::GOPEntry( 'B',    27,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3, 11, 27 },           2,   {  -1,  -5                } );
        c->m_GOPList[ 29 ] = vvenc::GOPEntry( 'B',    30,    5,       -4.4895,  0.1947,  1.0,         4,             2,   {  2, 14, 30     },           2,   {  -2,   2                } );
        c->m_GOPList[ 30 ] = vvenc::GOPEntry( 'B',    29,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1, 13, 29     },           2,   {  -1,  -3                } );
        c->m_GOPList[ 31 ] = vvenc::GOPEntry( 'B',    31,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3, 15, 31 },           2,   {  -1,   1                } );
      }
      else
      {
        if( c->m_GOPList[ 0 ].m_POC != -1 )
        {
          msg.log( VVENC_WARNING, "Configuration warning: custom gop configuartion and option AddGOP32refPics detected, given gop configuration will be overwritten!\n\n" );
        }
        //overwrite GOPEntries
        c->m_GOPList[  0 ] = vvenc::GOPEntry(  'B',   32,   -1,           0.0,     0.0,  1.0,         0,             2,   { 32, 64, 48, 40, 36 },       1,   {  32, 48                 } );
        c->m_GOPList[  1 ] = vvenc::GOPEntry(  'B',   16,    0,       -4.9309,  0.2265,  1.0,         1,             3,   { 16, 32, 48, 24, 20 },       1,   { -16                     } );
        c->m_GOPList[  2 ] = vvenc::GOPEntry(  'B',    8,    1,       -4.5000,  0.1900,  1.0,         2,             4,   {  8, 24, 16, 40, 12 },       2,   {  -8, -24                } );
        c->m_GOPList[  3 ] = vvenc::GOPEntry(  'B',    4,    3,       -5.4095,  0.2571,  1.0,         3,             3,   {  4,  8, 20         },       3,   {  -4, -12, -28,          } );
        c->m_GOPList[  4 ] = vvenc::GOPEntry(  'B',    2,    5,       -4.4895,  0.1947,  1.0,         4,             3,   {  2,  6, 18         },       4,   {  -2,  -6, -14, -30      } );
        c->m_GOPList[  5 ] = vvenc::GOPEntry(  'B',    1,    6,       -5.4429,  0.2429,  1.0,         5,             1,   {  1                 },       2,   {  -1,  -3,  -7, -15, -31 } );
        c->m_GOPList[  6 ] = vvenc::GOPEntry(  'B',    3,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3             },       2,   {  -1,  -5, -13, -29      } );
        c->m_GOPList[  7 ] = vvenc::GOPEntry(  'B',    6,    5,       -4.4895,  0.1947,  1.0,         4,             3,   {  2,  4,  6         },       3,   {  -2, -10, -26           } );
        c->m_GOPList[  8 ] = vvenc::GOPEntry(  'B',    5,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  5             },       2,   {  -1,  -3, -11, -27      } );
        c->m_GOPList[  9 ] = vvenc::GOPEntry(  'B',    7,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3,  7         },       2,   {  -1,  -9, -25           } );
        c->m_GOPList[ 10 ] = vvenc::GOPEntry(  'B',   12,    3,       -5.4095,  0.2571,  1.0,         3,             3,   {  4,  8, 12,  6     },       2,   {  -4, -20                } );
        c->m_GOPList[ 11 ] = vvenc::GOPEntry(  'B',   10,    5,       -4.4895,  0.1947,  1.0,         4,             4,   {  2,  4,  6, 10     },       3,   {  -2,  -6, -22           } );
        c->m_GOPList[ 12 ] = vvenc::GOPEntry(  'B',    9,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  5,  9         },       2,   {  -1,  -3,  -7, -23      } );
        c->m_GOPList[ 13 ] = vvenc::GOPEntry(  'B',   11,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3, 11         },       2,   {  -1,  -5, -21           } );
        c->m_GOPList[ 14 ] = vvenc::GOPEntry(  'B',   14,    5,       -4.4895,  0.1947,  1.0,         4,             4,   {  2,  4,  6, 14     },       2,   {  -2, -18                } );
        c->m_GOPList[ 15 ] = vvenc::GOPEntry(  'B',   13,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  5, 13         },       2,   {  -1,  -3, -19           } );
        c->m_GOPList[ 16 ] = vvenc::GOPEntry(  'B',   15,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3,  7, 15     },       2,   {  -1, -17                } );
        c->m_GOPList[ 17 ] = vvenc::GOPEntry(  'B',   24,    1,       -4.5000,  0.1900,  1.0,         2,             3,   {  8, 16, 24         },       1,   {  -8                     } );
        c->m_GOPList[ 18 ] = vvenc::GOPEntry(  'B',   20,    3,       -5.4095,  0.2571,  1.0,         3,             3,   {  4, 12, 20         },       2,   {  -4, -12                } );
        c->m_GOPList[ 19 ] = vvenc::GOPEntry(  'B',   18,    5,       -4.4895,  0.1947,  1.0,         4,             3,   {  2, 10, 18         },       3,   {  -2,  -6, -14           } );
        c->m_GOPList[ 20 ] = vvenc::GOPEntry(  'B',   17,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  9, 17         },       2,   {  -1,  -3,  -7, -15      } );
        c->m_GOPList[ 21 ] = vvenc::GOPEntry(  'B',   19,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3, 19         },       2,   {  -1,  -5, -13           } );
        c->m_GOPList[ 22 ] = vvenc::GOPEntry(  'B',   22,    5,       -4.4895,  0.1947,  1.0,         4,             3,   {  2,  6, 22         },       3,   {  -2, -10, 4             } );
        c->m_GOPList[ 23 ] = vvenc::GOPEntry(  'B',   21,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  5, 21         },       2,   {  -1,  -3, -11           } );
        c->m_GOPList[ 24 ] = vvenc::GOPEntry(  'B',   23,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3,  7, 23     },       2,   {  -1, -9                 } );
        c->m_GOPList[ 25 ] = vvenc::GOPEntry(  'B',   28,    3,       -5.4095,  0.2571,  1.0,         3,             4,   {  4,  8, 12, 28     },       1,   {  -4                     } );
        c->m_GOPList[ 26 ] = vvenc::GOPEntry(  'B',   26,    5,       -4.4895,  0.1947,  1.0,         4,             4,   {  2,  6, 10, 26     },       2,   {  -2, -6                 } );
        c->m_GOPList[ 27 ] = vvenc::GOPEntry(  'B',   25,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  5,  9, 25     },       2,   {  -1, -3, -7             } );
        c->m_GOPList[ 28 ] = vvenc::GOPEntry(  'B',   27,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3, 11, 27     },       2,   {  -1, -5                 } );
        c->m_GOPList[ 29 ] = vvenc::GOPEntry(  'B',   30,    5,       -4.4895,  0.1947,  1.0,         4,             4,   {  2,  6, 14, 30     },       1,   {  -2                     } );
        c->m_GOPList[ 30 ] = vvenc::GOPEntry(  'B',   29,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  5, 13, 29     },       2,   {  -1, -3                 } );
        c->m_GOPList[ 31 ] = vvenc::GOPEntry(  'B',   31,    6,       -5.4429,  0.2429,  1.0,         5,             2,   {  1,  3,  7, 15, 31 },       1,   {  -1                     } );
      }
    }
    else
    {
      vvenc_confirmParameter( c, true, "GOP auto configuration only supported for GOP size (1,8,16,32)" );
    }

    if( autoGop && c->m_numRefPics != 0 )
    {
      const int maxTLayer  = c->m_picReordering && c->m_GOPSize > 1 ? vvenc::ceilLog2( c->m_GOPSize ) : 0;
      const int numRefCode = c->m_numRefPics;

      for( int i = 0; i < 64; i++ )
      {
        if( c->m_GOPList[i].m_POC == -1 ) break;

        int tLayer  = c->m_GOPList[i].m_temporalId;
        int numRefs = numRefCode < 10 ? numRefCode : ( int( numRefCode / pow( 10, maxTLayer - tLayer ) ) % 10 );

        if( c->m_GOPList[i].m_sliceType != 'I' )
          vvenc_confirmParameter( c, numRefs > c->m_GOPList[i].m_numRefPics[0], "Invalid number of references set in NumRefPics!" );
        if( c->m_GOPList[i].m_sliceType == 'B' )
          vvenc_confirmParameter( c, numRefs > c->m_GOPList[i].m_numRefPics[1], "Invalid number of references set in NumRefPics!" );
        if( c->m_GOPList[i].m_sliceType != 'I' )
          vvenc_confirmParameter( c, numRefs == 0, "Invalid number of references set in NumRefPics!" );
      }
    }

    if( autoGop && c->m_numRefPicsSCC != 0 )
    {
      const int maxTLayer  = c->m_picReordering && c->m_GOPSize > 1 ? vvenc::ceilLog2( c->m_GOPSize ) : 0;
      const int numRefCode = c->m_numRefPicsSCC;

      for( int i = 0; i < 64; i++ )
      {
        if( c->m_GOPList[i].m_POC == -1 ) break;

        int tLayer  = c->m_GOPList[i].m_temporalId;
        int numRefs = numRefCode < 10 ? numRefCode : ( int( numRefCode / pow( 10, maxTLayer - tLayer ) ) % 10 );

        if( c->m_GOPList[i].m_sliceType != 'I' )
          vvenc_confirmParameter( c, numRefs > c->m_GOPList[i].m_numRefPics[0], "Invalid number of references set in NumRefPics!" );
        if( c->m_GOPList[i].m_sliceType == 'B' )
          vvenc_confirmParameter( c, numRefs > c->m_GOPList[i].m_numRefPics[1], "Invalid number of references set in NumRefPics!" );
        if( c->m_GOPList[i].m_sliceType != 'I' )
          vvenc_confirmParameter( c, numRefs == 0, "Invalid number of references set in NumRefPics!" );
      }
    }
  }

  vvenc_confirmParameter( c, !autoGop && c->m_numRefPics    != 0,                         "NumRefPics cannot be used if explicit GOP configuration is used!" );
  vvenc_confirmParameter( c, !autoGop && c->m_numRefPicsSCC != 0,                         "NumRefPicsSCC cannot be used if explicit GOP configuration is used!" );
  vvenc_confirmParameter( c, !autoGop && c->m_numRefPics    != 0 && c->m_addGOP32refPics, "NumRefPics and AddGOP32refPics options are mutually exclusive!" );
  vvenc_confirmParameter( c, !autoGop && c->m_numRefPicsSCC != 0 && c->m_addGOP32refPics, "NumRefPicsSCC and AddGOP32refPics options are mutually exclusive!" );

  if ( ! c->m_MMVD && c->m_allowDisFracMMVD )
  {
    msg.log( VVENC_WARNING, "Configuration warning: MMVD disabled, thus disable AllowDisFracMMVD too\n\n" );
    c->m_allowDisFracMMVD = false;
  }

  //
  // finalize initialization 
  //


  c->m_PROF &= bool(c->m_Affine);
  if (c->m_Affine > 1)
  {
    c->m_PROF = bool(c->m_Affine);
    c->m_AffineType = (c->m_Affine > 1) ? true : false;
  }

  // check char array and reset them, if they seems to be unset
  vvenc_checkCharArrayStr( c->m_traceRule, VVENC_MAX_STRING_LEN);
  vvenc_checkCharArrayStr( c->m_traceFile, VVENC_MAX_STRING_LEN);
  vvenc_checkCharArrayStr( c->m_summaryOutFilename, VVENC_MAX_STRING_LEN);
  vvenc_checkCharArrayStr( c->m_summaryPicFilenameBase, VVENC_MAX_STRING_LEN);

  const int maxTLayer = c->m_picReordering && c->m_GOPSize > 1 ? vvenc::ceilLog2( c->m_GOPSize ) : 0;
  
  if( c->m_deblockLastTLayers > 0 )
  {
    if( maxTLayer > 0 )
    {
      vvenc_confirmParameter( c, c->m_bLoopFilterDisable, "DeblockLastTLayers can only be applied when deblocking filter is not disabled (LoopFilterDisable=0)" );
      vvenc_confirmParameter( c, maxTLayer - c->m_deblockLastTLayers <= 0, "DeblockLastTLayers exceeds the range of possible deblockable temporal layers" );
    }
    c->m_loopFilterOffsetInPPS = false;
  }

  if( c->m_alf )
  {
    if( c->m_alfSpeed > maxTLayer )
    {
      msg.log( VVENC_WARNING, "Configuration warning: ALFSpeed would disable ALF for the given GOP configuration, disabling ALFSpeed!\n\n" );

      c->m_alfSpeed = 0;
    }
  }

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
                              "unsupported profile. currently only supporting auto,main_10,main_10_still_picture");

  vvenc_confirmParameter( c, c->m_level   == vvencLevel::VVENC_LEVEL_AUTO, "can not determin level");

  vvenc_confirmParameter( c, c->m_fastInterSearchMode<VVENC_FASTINTERSEARCH_OFF || c->m_fastInterSearchMode>VVENC_FASTINTERSEARCH_MODE3,     "FastInterSearchMode parameter out of range [0...3]" );
  vvenc_confirmParameter( c, c->m_motionEstimationSearchMethod < 0
                          || c->m_motionEstimationSearchMethod >= VVENC_MESEARCH_NUMBER_OF_METHODS
                          || c->m_motionEstimationSearchMethod == VVENC_MESEARCH_DEPRECATED,                                                 "FastSearch parameter out of range [0,1,3,4]");
  vvenc_confirmParameter( c, c->m_motionEstimationSearchMethodSCC < 0
                          || c->m_motionEstimationSearchMethodSCC == 1
                          || c->m_motionEstimationSearchMethodSCC > 3,                                                                       "FastSearchSCC parameter out of range [0,2,3]" );
  vvenc_confirmParameter( c, c->m_internChromaFormat > VVENC_CHROMA_420,                                                                     "Intern chroma format must be either 400, 420" );

  vvenc::MsgLog msg(c->m_msgCtx,c->m_msgFnc);

  switch ( c->m_conformanceWindowMode)
  {
  case 0:
      break;
  case 1:
      // automatic padding to minimum CU size
      vvenc_confirmParameter( c, c->m_aiPad[0] % vvenc::SPS::getWinUnitX(c->m_internChromaFormat) != 0, "picture width is not an integer multiple of the specified chroma subsampling" );
      vvenc_confirmParameter( c, c->m_aiPad[1] % vvenc::SPS::getWinUnitY(c->m_internChromaFormat) != 0, "picture height is not an integer multiple of the specified chroma subsampling" );
      break;
  case 2:
      break;
  case 3:
      // conformance
      if ((c->m_confWinLeft == 0) && (c->m_confWinRight == 0) && (c->m_confWinTop == 0) && (c->m_confWinBottom == 0))
      {
        msg.log( VVENC_WARNING, "Configuration warning: Conformance window enabled, but all conformance window parameters set to zero\n\n" );
      }
      if ((c->m_aiPad[1] != 0) || (c->m_aiPad[0]!=0))
      {
        msg.log( VVENC_WARNING, "Configuration warning: Conformance window enabled, padding parameters will be ignored\n\n" );
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

  for (uint32_t channelType = 0; channelType < 2; channelType++)
  {
    vvenc_confirmParameter( c,(c->m_internalBitDepth[channelType] > 10) , "VVenC does not support internal bitdepth larger than 10!");
  }


  vvenc_confirmParameter( c, (isHDRMode(c->m_HdrMode) && c->m_internalBitDepth[0] < 10 )     ,       "InternalBitDepth must be at least 10 bit for HDR");
  vvenc_confirmParameter( c, (isHDRMode(c->m_HdrMode) && c->m_internChromaFormat != VVENC_CHROMA_420 ) ,"ChromaFormatIDC must be YCbCr 4:2:0 for HDR");
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

  vvenc_confirmParameter( c, c->m_DecodingRefreshType < 0 || c->m_DecodingRefreshType > 5,                "Decoding refresh type must be comprised between 0 and 5 included" );
  vvenc_confirmParameter( c,   c->m_picReordering && (c->m_DecodingRefreshType == VVENC_DRT_NONE || c->m_DecodingRefreshType == VVENC_DRT_RECOVERY_POINT_SEI), "Decoding refresh type Recovery Point SEI for non low delay not supported" );
  vvenc_confirmParameter( c, ! c->m_picReordering &&  c->m_DecodingRefreshType != VVENC_DRT_NONE,                                                              "Only decoding refresh type none for low delay supported" );

  vvenc_confirmParameter( c, c->m_QP < -6 * (c->m_internalBitDepth[0] - 8) || c->m_QP > vvenc::MAX_QP,                "QP exceeds supported range (-QpBDOffsety to 63)" );
  for( int comp = 0; comp < 3; comp++)
  {
    vvenc_confirmParameter( c, c->m_loopFilterBetaOffsetDiv2[comp] < -12 || c->m_loopFilterBetaOffsetDiv2[comp] > 12,          "Loop Filter Beta Offset div. 2 exceeds supported range (-12 to 12)" );
    vvenc_confirmParameter( c, c->m_loopFilterTcOffsetDiv2[comp] < -12 || c->m_loopFilterTcOffsetDiv2[comp] > 12,              "Loop Filter Tc Offset div. 2 exceeds supported range (-12 to 12)" );
  }
  vvenc_confirmParameter( c, c->m_SearchRange < 0 ,                                                         "Search Range must be more than 0" );
  vvenc_confirmParameter( c, c->m_bipredSearchRange < 0 ,                                                   "Bi-prediction refinement search range must be more than 0" );
  vvenc_confirmParameter( c, c->m_minSearchWindow < 0,                                                      "Minimum motion search window size for the adaptive window ME must be greater than or equal to 0" );

  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTF > 2 || c->m_vvencMCTF.MCTF < 0,                    "MCTF out of range" );
  vvenc_confirmParameter( c, c->m_vvencMCTF.numFrames != c->m_vvencMCTF.numStrength,                "MCTF parameter list sizes differ" );
  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTFSpeed < 0 || c->m_vvencMCTF.MCTFSpeed > 4,          "MCTFSpeed exceeds supported range (0..4)" );
  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTFUnitSize < 8,                                       "MCTFUnitSize is smaller than 8" );
  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTFUnitSize > 32,                                      "MCTFUnitSize is larger than 32" );
  vvenc_confirmParameter( c, c->m_vvencMCTF.MCTFUnitSize & ( c->m_vvencMCTF.MCTFUnitSize - 1 ),     "MCTFUnitSize is not a power of 2" );
  static const std::string errorSegLessRng = std::string( "When using segment parallel encoding more then " ) + static_cast< char >( VVENC_MCTF_RANGE + '0' ) + " frames have to be encoded";
  vvenc_confirmParameter( c, c->m_SegmentMode != VVENC_SEG_OFF && c->m_framesToBeEncoded < VVENC_MCTF_RANGE, errorSegLessRng.c_str() );
  for( int i = 0; i < c->m_vvencMCTF.numFrames; i++ )
  {
    vvenc_confirmParameter( c, c->m_vvencMCTF.MCTFFrames[ i ] <= 0, "MCTFFrame has to be greater then zero" );
  }

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
  vvenc_confirmParameter( c, c->m_Affine < 0 || c->m_Affine > 5,              "Affine out of range [0..5]" );
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
  vvenc_confirmParameter( c, c->m_qtbttSpeedUp < 0 || c->m_qtbttSpeedUp > 7,   "QtbttExtraFast out of range [0..7]");
  vvenc_confirmParameter( c, c->m_fastTTSplit < 0 || c->m_fastTTSplit > 7,     "FastTTSplit out of range [0..7]");
  vvenc_confirmParameter( c, c->m_MTSIntraMaxCand < 0 || c->m_MTSIntraMaxCand > 4, "MTSIntraMaxCand out of range [0..4]");

  const int fimModeMap[] = { 0, 3, 19, 27, 29 };
  const int maxTLayer = c->m_picReordering && c->m_GOPSize > 1 ? vvenc::ceilLog2( c->m_GOPSize ) : 0;
  c->m_FastInferMerge = fimModeMap[ c->m_FIMMode ];
  if( ( c->m_FastInferMerge & 7 ) > maxTLayer )
  {
    const int hbm = c->m_FastInferMerge >> 3;
    const int lbm = std::min<int>( 7, maxTLayer );
    c->m_FastInferMerge = ( hbm << 3 ) | lbm;
  }

  c->m_qtbttSpeedUpMode = (c->m_qtbttSpeedUp > 2) ? (c->m_qtbttSpeedUp - 2) : 0;
  const int QTBTSMModeMap[] = { 0, 1, 3, 4, 5, 7 };
  c->m_qtbttSpeedUpMode = QTBTSMModeMap[c->m_qtbttSpeedUpMode];
  static const float TT_THRESHOLDS[7] = { 1.1f, 1.075f, 1.05f, 1.025f, 1.0f,  0.975f, 0.95f };
  c->m_fastTT_th = c->m_fastTTSplit ? TT_THRESHOLDS[c->m_fastTTSplit - 1] : 0;

  if( c->m_alf )
  {
    vvenc_confirmParameter( c, c->m_maxNumAlfAlternativesChroma < 1 || c->m_maxNumAlfAlternativesChroma > VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA, std::string( std::string( "The maximum number of ALF Chroma filter alternatives must be in the range (1-" ) + std::to_string( VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA ) + std::string( ", inclusive)" ) ).c_str() );
  }

  vvenc_confirmParameter( c, c->m_useFastMrg < 0 || c->m_useFastMrg > 3,   "FastMrg out of range [0..3]" );
  vvenc_confirmParameter( c, c->m_useFastMIP < 0 || c->m_useFastMIP > 3,   "FastMIP out of range [0..3]" );
  vvenc_confirmParameter( c, c->m_fastSubPel < 0 || c->m_fastSubPel > 2,   "FastSubPel out of range [0..2]" );
  vvenc_confirmParameter( c, c->m_useEarlyCU < 0 || c->m_useEarlyCU > 2,   "ECU out of range [0..2]" );
  vvenc_confirmParameter( c, c->m_meReduceTap < 0 || c->m_meReduceTap > 2, "ReduceFilterME out of range [0..2]" );

  vvenc_confirmParameter( c, c->m_RCTargetBitrate == 0 && c->m_RCNumPasses != 1, "Only single pass encoding supported, when rate control is disabled" );
  vvenc_confirmParameter( c, c->m_RCNumPasses < 1 || c->m_RCNumPasses > 2,       "Only one pass or two pass encoding supported" );
  vvenc_confirmParameter( c, c->m_RCNumPasses < 2 && c->m_RCPass > 1,            "Only one pass supported in single pass encoding" );
  vvenc_confirmParameter( c, c->m_RCPass != -1 && ( c->m_RCPass < 1 || c->m_RCPass > 2 ), "Invalid pass parameter, only -1, 1 or 2 supported" );
  vvenc_confirmParameter( c, c->m_RCTargetBitrate > 0 && c->m_maxParallelFrames > 4, "Up to 4 parallel frames supported with rate control" );
  vvenc_confirmParameter( c, c->m_LookAhead < -1 || c->m_LookAhead > 1,          "Look-ahead out of range [-1..1]" );
  vvenc_confirmParameter( c, c->m_LookAhead && c->m_RCNumPasses != 1,       "Look-ahead encoding is not supported for two-pass rate control" );
  vvenc_confirmParameter( c, !c->m_LookAhead && c->m_RCNumPasses == 1 && c->m_RCTargetBitrate > 0, "Look-ahead encoding must be used with one-pass rate control" );
  vvenc_confirmParameter( c, c->m_LookAhead && c->m_RCTargetBitrate == 0,   "Look-ahead encoding is not supported when rate control is disabled" );

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
    msg.log( VVENC_WARNING, "Configuration warning: chroma QPA on, ignoring nonzero dual-tree chroma QP offsets!\n\n");
  }

  vvenc_confirmParameter(c, c->m_usePerceptQPATempFiltISlice > 2,                                                       "PerceptQPATempFiltIPic out of range, must be 2 or less" );
  vvenc_confirmParameter(c, c->m_usePerceptQPATempFiltISlice > 0 && c->m_vvencMCTF.MCTF == 0,                           "PerceptQPATempFiltIPic must be turned off when MCTF is off" );

  vvenc_confirmParameter(c, c->m_usePerceptQPA && (c->m_cuQpDeltaSubdiv > 2),                                           "MaxCuDQPSubdiv must be 2 or smaller when PerceptQPA is on" );

  vvenc_confirmParameter(c, c->m_MinQT[0] < 1<<vvenc::MIN_CU_LOG2,                                                      "Minimum QT size should be larger than or equal to 4");
  vvenc_confirmParameter(c, c->m_MinQT[1] < 1<<vvenc::MIN_CU_LOG2,                                                      "Minimum QT size should be larger than or equal to 4");
  vvenc_confirmParameter(c, c->m_CTUSize < 32,                                                                          "CTUSize must be greater than or equal to 32");
  vvenc_confirmParameter(c, c->m_CTUSize > 128,                                                                         "CTUSize must be less than or equal to 128");
  vvenc_confirmParameter(c, c->m_CTUSize != 32 && c->m_CTUSize != 64 && c->m_CTUSize != 128,                            "CTUSize must be a power of 2 (32, 64, or 128)");
  vvenc_confirmParameter(c, (c->m_PadSourceWidth  % std::max( 8, 1 << c->m_log2MinCodingBlockSize )) != 0,              "Resulting coded frame width must be a multiple of Max(8, the minimum CU size)");
  vvenc_confirmParameter(c, (c->m_PadSourceHeight % std::max( 8, 1 << c->m_log2MinCodingBlockSize )) != 0,              "Resulting coded frame width must be a multiple of Max(8, the minimum CU size)");
  vvenc_confirmParameter(c, c->m_log2MaxTbSize > 6,                                                                     "Log2MaxTbSize must be 6 or smaller." );
  vvenc_confirmParameter(c, c->m_log2MaxTbSize < 5,                                                                     "Log2MaxTbSize must be 5 or greater." );

  vvenc_confirmParameter( c, c->m_log2MinCodingBlockSize < 2,                                                           "Log2MinCodingBlockSize must be 2 or greater." );
  vvenc_confirmParameter( c, c->m_CTUSize < ( 1 << c->m_log2MinCodingBlockSize ),                                       "Log2MinCodingBlockSize must be smaller than max CTU size." );
  vvenc_confirmParameter( c, c->m_MinQT[ 0 ] < ( 1 << c->m_log2MinCodingBlockSize ),                                    "Log2MinCodingBlockSize must be greater than min QT size for I slices" );
  vvenc_confirmParameter( c, c->m_MinQT[ 1 ] < ( 1 << c->m_log2MinCodingBlockSize ),                                    "Log2MinCodingBlockSize must be greater than min QT size for non I slices" );
  const int chromaScaleX = ( (c->m_internChromaFormat==VVENC_CHROMA_444) ) ? 0 : 1;
  vvenc_confirmParameter( c, ( c->m_MinQT[ 2 ] << chromaScaleX ) < ( 1 << c->m_log2MinCodingBlockSize ),                "Log2MinCodingBlockSize must be greater than min chroma QT size for I slices" );

  if( c->m_maxMTTDepth >= 10 && c->m_maxMTTDepth >= pow( 10, ( maxTLayer + 1 ) ) )
  {
    msg.log( VVENC_WARNING, "Configuration warning: MaxMTTHierarchyDepth>=10 & larger than maxTLayer\n\n" );
  }
  vvenc_confirmParameter(c, c->m_maxMTTDepth >= 10 && c->m_maxMTTDepth < pow(10, maxTLayer ), "MaxMTTHierarchyDepth>=10 & not set for all TLs");

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

  vvenc_confirmParameter(c, c->m_alfUnitSize < c->m_CTUSize,                                      "ALF Unit Size must be greater than or equal to CTUSize");
  vvenc_confirmParameter(c, c->m_alfUnitSize % c->m_CTUSize != 0,                                 "ALF Unit Size must be a multiple of CTUSize");

  vvenc_confirmParameter(c, c->m_alfSpeed < 0 || ( maxTLayer > 0 && c->m_alfSpeed > maxTLayer ),  "ALFSpeed out of range (0,log2(GopSize))" );
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
#if ENABLE_TRACING
    vvenc_confirmParameter(c, c->m_traceFile[0] != '\0' && c->m_maxParallelFrames > 1 && c->m_numThreads > 1, "Tracing and frame parallel encoding not supported" );
#endif
    vvenc_confirmParameter(c, c->m_maxParallelFrames > c->m_GOPSize && c->m_GOPSize != 1, "Max parallel frames should be less then GOP size" );
    vvenc_confirmParameter(c, c->m_fppLinesSynchro && c->m_alfTempPred != 0, "FPP CTU-lines synchro: ALFTempPred is not supported (must be disabled)" );
    vvenc_confirmParameter(c, c->m_fppLinesSynchro && c->m_numTileRows > 1,  "FPP CTU-lines synchro: Only single tile row is supported" );
    vvenc_confirmParameter(c, c->m_fppLinesSynchro < 0, "fppLinesSynchro must be >= 0" );
  }

  vvenc_confirmParameter(c, c->m_explicitAPSid < 0 || c->m_explicitAPSid > 7, "ExplicitAPDid out of range [0 .. 7]" );

  vvenc_confirmParameter(c, c->m_maxNumMergeCand < 1,                              "MaxNumMergeCand must be 1 or greater.");
  vvenc_confirmParameter(c, c->m_maxNumMergeCand > vvenc::MRG_MAX_NUM_CANDS,              "MaxNumMergeCand must be no more than MRG_MAX_NUM_CANDS." );
  vvenc_confirmParameter(c, c->m_maxNumGeoCand > vvenc::GEO_MAX_NUM_UNI_CANDS,            "MaxNumGeoCand must be no more than GEO_MAX_NUM_UNI_CANDS." );
  vvenc_confirmParameter(c, c->m_maxNumGeoCand > c->m_maxNumMergeCand,                "MaxNumGeoCand must be no more than MaxNumMergeCand." );
  vvenc_confirmParameter(c, 0 < c->m_maxNumGeoCand && c->m_maxNumGeoCand < 2,         "MaxNumGeoCand must be no less than 2 unless MaxNumGeoCand is 0." );
  vvenc_confirmParameter(c, c->m_maxNumAffineMergeCand < (c->m_SbTMVP ? 1 : 0),       "MaxNumAffineMergeCand must be greater than 0 when SbTMVP is enabled");
  vvenc_confirmParameter(c, c->m_maxNumAffineMergeCand > vvenc::AFFINE_MRG_MAX_NUM_CANDS, "MaxNumAffineMergeCand must be no more than AFFINE_MRG_MAX_NUM_CANDS." );


  vvenc_confirmParameter(c, c->m_bufferingPeriodSEIEnabled && (!c->m_hrdParametersPresent), "BufferingPeriodSEI requires HrdParametersPresent enabled");
  vvenc_confirmParameter(c, c->m_pictureTimingSEIEnabled && (!c->m_hrdParametersPresent),   "PictureTimingSEI requires HrdParametersPresent enabled");

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

  vvenc_confirmParameter(c,  c->m_IntraPeriod != 1 && c->m_intraOnlyConstraintFlag, "IntraOnlyConstraintFlag cannot be 1 for inter sequences");

  if( c->m_GOPList[ 0 ].m_POC != -1 )
  {
    int multipleFactor = /*m_compositeRefEnabled ? 2 :*/ 1;
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
          //TODO: c->m_GOPList[i].m_tcOffsetDiv2 and c->m_GOPList[i].m_betaOffsetDiv2 are checked with the luma value also for the chroma components (currently not used or all values are equal)
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

    for(int i=0; i<c->m_GOPSize; i++)
    {
      vvenc_confirmParameter(c, c->m_GOPList[i].m_sliceType!='B' && c->m_GOPList[i].m_sliceType!='P' && c->m_GOPList[i].m_sliceType!='I', "Slice type must be equal to B or P or I");
    }
  }

  vvenc_confirmParameter(c, abs(c->m_sliceChromaQpOffsetIntraOrPeriodic[0]                 ) > 12, "Intra/periodic Cb QP Offset exceeds supported range (-12 to 12)" );
  vvenc_confirmParameter(c, abs(c->m_sliceChromaQpOffsetIntraOrPeriodic[0]  + c->m_chromaCbQpOffset ) > 12, "Intra/periodic Cb QP Offset, when combined with the PPS Cb offset, exceeds supported range (-12 to 12)" );
  vvenc_confirmParameter(c, abs(c->m_sliceChromaQpOffsetIntraOrPeriodic[1]                 ) > 12, "Intra/periodic Cr QP Offset exceeds supported range (-12 to 12)" );
  vvenc_confirmParameter(c, abs(c->m_sliceChromaQpOffsetIntraOrPeriodic[1]  + c->m_chromaCrQpOffset ) > 12, "Intra/periodic Cr QP Offset, when combined with the PPS Cr offset, exceeds supported range (-12 to 12)" );

  vvenc_confirmParameter(c, c->m_fastLocalDualTreeMode < 0 || c->m_fastLocalDualTreeMode > 2, "FastLocalDualTreeMode must be in range [0..2]" );

  if( c->m_picPartitionFlag || c->m_numTileCols > 1 || c->m_numTileRows > 1 )
  {
    if( !c->m_picPartitionFlag ) c->m_picPartitionFlag = true;

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

  int lastNonZeroColumn = -1, lastNonZeroRow = -1;
  bool validCfg = true;
  checkCfgInputArrays( c, lastNonZeroColumn, lastNonZeroRow, validCfg );
  if( !validCfg ) return;

  int numTileColumnWidths, numTileRowHeights;

  bool colWidth_all_zero  = lastNonZeroColumn == -1;
  bool rowHeight_all_zero = lastNonZeroRow == -1;

  //number of tiles is set explicitly, e.g. Tiles=2x2
  //TileColumnWidthArray and TileRowHeightArray have to be not set
  if( c->m_numTileCols > 1 || c->m_numTileRows > 1 )
  {
    vvenc_confirmParameter( c, !colWidth_all_zero  && ( lastNonZeroColumn + 1 ) != c->m_numTileCols, "Explicit number of tile columns and column widths are given, but not consistent!" );
    vvenc_confirmParameter( c, !rowHeight_all_zero && ( lastNonZeroRow    + 1 ) != c->m_numTileRows, "Explicit number of tile rows and column heights are given, but not consistent!" );

    if( !colWidth_all_zero || !rowHeight_all_zero ) return;

    if( c->m_numTileCols > 1 )
    {
      unsigned int tileWidth = pps.picWidthInCtu / c->m_numTileCols;
      if( tileWidth * c->m_numTileCols < pps.picWidthInCtu ) tileWidth++;
      c->m_tileColumnWidth[0] = tileWidth;
    }
    else
    {
      c->m_tileColumnWidth[0] = pps.picWidthInCtu;
    }
    if( c->m_numTileRows > 1 )
    {
      unsigned int tileHeight = pps.picHeightInCtu / c->m_numTileRows;
      if( tileHeight * c->m_numTileRows < pps.picHeightInCtu ) tileHeight++;
      c->m_tileRowHeight[0] = tileHeight;
    }
    else
    {
      c->m_tileRowHeight[0] = pps.picHeightInCtu;
    }

    numTileColumnWidths = 1;
    numTileRowHeights   = 1;
  }
  else
  {
    // set default tile column if not provided
    if( colWidth_all_zero )
    {
      c->m_tileColumnWidth[0] = pps.picWidthInCtu;
    }
    // set default tile row if not provided
    if( rowHeight_all_zero )
    {
      c->m_tileRowHeight[0] = pps.picHeightInCtu;
    }

    // remove any tile columns that can be specified implicitly
    if( c->m_tileColumnWidth[1] > 0 )
    {
      while( lastNonZeroColumn > 0 && c->m_tileColumnWidth[lastNonZeroColumn-1] == c->m_tileColumnWidth[lastNonZeroColumn] )
      {
        c->m_tileColumnWidth[lastNonZeroColumn] = 0;
        lastNonZeroColumn--;
      }
      numTileColumnWidths = lastNonZeroColumn+1;
    }
    else
    {
      numTileColumnWidths = 1;
    }

    // remove any tile rows that can be specified implicitly
    if( c->m_tileRowHeight[1] > 0 )
    {
      while( lastNonZeroRow > 0 && c->m_tileRowHeight[lastNonZeroRow-1] == c->m_tileRowHeight[lastNonZeroRow] )
      {
        c->m_tileRowHeight[lastNonZeroRow] = 0;
        lastNonZeroRow--;
      }
      numTileRowHeights = lastNonZeroRow+1;
    }
    else
    {
      numTileRowHeights = 1;
    }
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

  for( int col = 0; col < pps.numTileCols; col++ ) c->m_tileColumnWidth[col] = pps.tileColWidth [col];
  for( int row = 0; row < pps.numTileRows; row++ ) c->m_tileRowHeight  [row] = pps.tileRowHeight[row];

  vvenc_confirmParameter( c, c->m_treatAsSubPic && ( c->m_numTileCols > 1 || c->m_numTileRows > 1 ), "TreatAsSubPic and Tiles not supported yet");
}

static void checkCfgInputArrays( vvenc_config *c, int &lastNonZeroCol, int &lastNonZeroRow, bool &cfgIsValid )
{
  int lastNonZeroIdx = -1;
  for( int i = 9; i >= 0; i-- )
  {
    if( c->m_tileColumnWidth[i] != 0 )
    {
      lastNonZeroIdx = i;
      break;
    }
  }
  lastNonZeroCol = lastNonZeroIdx;

  lastNonZeroIdx = -1;

  for( int i = 9; i >= 0; i-- )
  {
    if( c->m_tileRowHeight[i] != 0 )
    {
      lastNonZeroIdx = i;
      break;
    }
  }
  lastNonZeroRow = lastNonZeroIdx;

  if( lastNonZeroCol > 0 )
  {
    for( int i = 0; i < lastNonZeroCol; i++ )
    {
      vvenc_confirmParameter( c, c->m_tileColumnWidth[i] == 0, "Tile column width cannot be 0! Check your TileColumnWidthArray" );
      cfgIsValid = c->m_tileColumnWidth[i] != 0;
    }
  }
  if( lastNonZeroRow > 0 )
  {
    for( int i = 0; i < lastNonZeroRow; i++ )
    {
      vvenc_confirmParameter( c, c->m_tileRowHeight[i] == 0, "Tile row height cannot be 0! Check your TileRowHeightArray" );
      cfgIsValid = c->m_tileRowHeight[i] != 0;
    }
  }

}

VVENC_DECL int vvenc_init_default( vvenc_config *c, int width, int height, int framerate, int targetbitrate, int qp, vvencPresetMode preset )
{
  int iRet = VVENC_OK;
  vvenc_config_default( c );
  c->m_SourceWidth         = width;                    // luminance width of input picture
  c->m_SourceHeight        = height;                   // luminance height of input picture

  c->m_FrameRate           = framerate;                // temporal rate (fps num)
  c->m_FrameScale          = 1;                        // temporal scale (fps denum)

  switch( framerate )
  {
    case 23:  c->m_FrameRate = 24000;  c->m_FrameScale = 1001; break;
    case 29:  c->m_FrameRate = 30000;  c->m_FrameScale = 1001; break;
    case 59:  c->m_FrameRate = 60000;  c->m_FrameScale = 1001; break;
    case 119: c->m_FrameRate = 120000; c->m_FrameScale = 1001; break;
    default: break;
  }

  c->m_TicksPerSecond      = VVENC_TICKS_PER_SEC_DEF;  // ticks per second for dts generation

  c->m_inputBitDepth[0]    = 8;                        // input bitdepth
  c->m_internalBitDepth[0] = 10;                       // internal bitdepth

  c->m_QP                  = qp;                       // quantization parameter 0-63
  c->m_usePerceptQPA       = true;                     // perceptual QP adaptation (false: off, true: on)

  c->m_RCTargetBitrate     = targetbitrate;            // target bitrate for rate ctrl. in bps
  c->m_RCMaxBitrate        = 0;                        // maximum instantaneous bitrate in bps

  c->m_numThreads          = -1;                       // number of worker threads (-1: auto, 0: off, else set worker threads)
  
  iRet = vvenc_init_preset( c, preset );
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
  c->m_log2MinCodingBlockSize          = 2;
  c->m_intraQPOffset                   = -3;
  c->m_lambdaFromQPEnable              = true;
  c->m_bUseASR                         = true;
  c->m_bUseHADME                       = true;
  c->m_fastHad                         = false;
  c->m_useRDOQTS                       = true;
  c->m_useSelectiveRDOQ                = 0;
  c->m_fastQtBtEnc                     = true;
  c->m_maxNumMergeCand                 = 6;
  c->m_reshapeSignalType               = 0;
  c->m_updateCtrl                      = 0;
  c->m_LMCSOffset                      = 6;
  c->m_RDOQ                            = 1;
  c->m_SignDataHidingEnabled           = 0;
  c->m_useFastLCTU                     = 1;
  c->m_numRefPics                      = 0;
  c->m_numRefPicsSCC                   = 0;

  // tools
  c->m_Affine                          = 0;
  c->m_alf                             = 0;
  c->m_alfSpeed                        = 0;
  c->m_allowDisFracMMVD                = 0;
  c->m_BCW                             = 0;
  c->m_blockImportanceMapping          = 0;
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

      // motion estimation
      c->m_SearchRange                     = 128;
      c->m_bipredSearchRange               = 1;
      c->m_minSearchWindow                 = 96;
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE3;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND_FAST;

      // partitioning: CTUSize64 QT44MTT00
      c->m_CTUSize                         = 64;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 32;
      c->m_MinQT[ 1 ]                      = 32;
      c->m_MinQT[ 2 ]                      = 16;
      c->m_maxMTTDepth                     = 0;
      c->m_maxMTTDepthI                    = 0;

      // speedups
      c->m_qtbttSpeedUp                    = 7;
      c->m_fastTTSplit                     = 0;
      c->m_contentBasedFastQtbt            = true;
      c->m_fastHad                         = true;
      c->m_usePbIntraFast                  = 2;
      c->m_useFastMrg                      = 3;
      c->m_fastLocalDualTreeMode           = 1;
      c->m_fastSubPel                      = 2;
      c->m_FastIntraTools                  = 0;
      c->m_FIMMode                         = 4;
      c->m_useEarlyCU                      = 2;
      c->m_bIntegerET                      = 1;
      c->m_IntraEstDecBit                  = 3;
      c->m_numIntraModesFullRD             = 1;
      c->m_reduceIntraChromaModesFullRD    = true;
      c->m_meReduceTap                     = 2;
      c->m_numRefPics                      = 1;
      c->m_numRefPicsSCC                   = 0;

      // tools
      c->m_blockImportanceMapping          = 1;
      c->m_RDOQ                            = 2;
      c->m_useSelectiveRDOQ                = 2;
      c->m_SignDataHidingEnabled           = 1;
      c->m_LMChroma                        = 1;
      c->m_vvencMCTF.MCTF                  = 2;
      c->m_vvencMCTF.MCTFSpeed             = 4;
      c->m_MTSImplicit                     = 1;
      // scc
      c->m_IBCFastMethod                   = 6;
      c->m_TSsize                          = 3;
      c->m_saoScc                          = true;

      break;

    case vvencPresetMode::VVENC_FASTER:

      c->m_FirstPassMode                   = 4;

      // motion estimation
      c->m_SearchRange                     = 128;
      c->m_bipredSearchRange               = 1;
      c->m_minSearchWindow                 = 96;
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE3;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND_FAST;

      // partitioning: CTUSize64 QT44MTT00
      c->m_CTUSize                         = 64;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 4;
      c->m_MinQT[ 1 ]                      = 4;
      c->m_MinQT[ 2 ]                      = 2;
      c->m_maxMTTDepth                     = 0;
      c->m_maxMTTDepthI                    = 0;

      // speedups
      c->m_qtbttSpeedUp                    = 7;
      c->m_fastTTSplit                     = 0;
      c->m_contentBasedFastQtbt            = true;
      c->m_fastHad                         = true;
      c->m_usePbIntraFast                  = 2;
      c->m_useFastMrg                      = 3;
      c->m_fastLocalDualTreeMode           = 1;
      c->m_fastSubPel                      = 1;
      c->m_FastIntraTools                  = 0;
      c->m_FIMMode                         = 4;
      c->m_useEarlyCU                      = 2;
      c->m_bIntegerET                      = 1;
      c->m_IntraEstDecBit                  = 3;
      c->m_numIntraModesFullRD             = 1;
      c->m_reduceIntraChromaModesFullRD    = true;
      c->m_meReduceTap                     = 2;
      c->m_numRefPics                      = 1;
      c->m_numRefPicsSCC                   = 0;

      // tools
      c->m_alf                             = 1;
      c->m_alfSpeed                        = 2;
      c->m_alfUnitSize                     = 128;
      c->m_blockImportanceMapping          = 1;
      c->m_ccalf                           = 1;
      c->m_DMVR                            = 1;
      c->m_RDOQ                            = 2;
      c->m_useSelectiveRDOQ                = 2;
      c->m_SignDataHidingEnabled           = 1;
      c->m_LMChroma                        = 1;
      c->m_vvencMCTF.MCTF                  = 2;
      c->m_vvencMCTF.MCTFSpeed             = 4;
      c->m_MTSImplicit                     = 1;
      // scc
      c->m_IBCFastMethod                   = 6;
      c->m_TSsize                          = 3;
      c->m_saoScc                          = true;

      break;

    case vvencPresetMode::VVENC_FAST:

      c->m_FirstPassMode                   = 2;

      // motion estimation
      c->m_SearchRange                     = 128;
      c->m_bipredSearchRange               = 1;
      c->m_minSearchWindow                 = 96;
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE3;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND_FAST;

      // partitioning: CTUSize64 QT44MTT10
      c->m_CTUSize                         = 64;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 4;
      c->m_MinQT[ 1 ]                      = 4;
      c->m_MinQT[ 2 ]                      = 2;
      c->m_maxMTTDepth                     = 0;
      c->m_maxMTTDepthI                    = 1;

      // speedups
      c->m_qtbttSpeedUp                    = 3;
      c->m_fastTTSplit                     = 0;
      c->m_contentBasedFastQtbt            = true;
      c->m_fastHad                         = false;
      c->m_usePbIntraFast                  = 1;
      c->m_useFastMrg                      = 3;
      c->m_fastLocalDualTreeMode           = 1;
      c->m_fastSubPel                      = 1;
      c->m_FastIntraTools                  = 0;
      c->m_FIMMode                         = 2;
      c->m_useEarlyCU                      = 2;
      c->m_bIntegerET                      = 0;
      c->m_IntraEstDecBit                  = 2;
      c->m_numIntraModesFullRD             = 1;
      c->m_reduceIntraChromaModesFullRD    = true;
      c->m_meReduceTap                     = 2;
      c->m_numRefPics                      = 222111;
      c->m_numRefPicsSCC                   = 0;

      // tools
      c->m_Affine                          = 5;
      c->m_alf                             = 1;
      c->m_alfSpeed                        = 1;
      c->m_alfUnitSize                     = 128;
      c->m_allowDisFracMMVD                = 1;
      c->m_blockImportanceMapping          = 1;
      c->m_BDOF                            = 1;
      c->m_ccalf                           = 1;
      c->m_DepQuantEnabled                 = 1;
      c->m_DMVR                            = 1;
      c->m_AMVRspeed                       = 5;
      c->m_JointCbCrMode                   = 1;
      c->m_LFNST                           = 1;
      c->m_LMChroma                        = 1;
      c->m_lumaReshapeEnable               = 2;
      c->m_vvencMCTF.MCTF                  = 2;
      c->m_vvencMCTF.MCTFSpeed             = 3;
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
    case vvencPresetMode::VVENC_MEDIUM_LOWDECNRG:

      // motion estimation
      c->m_SearchRange                     = 384;
      c->m_bipredSearchRange               = 4;
      c->m_minSearchWindow                 = 96;
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE3;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND_FAST;

      // partitioning: CTUSize128 QT44MTT21
      c->m_CTUSize                         = 128;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 8;
      c->m_MinQT[ 1 ]                      = 8;
      c->m_MinQT[ 2 ]                      = 4;
      c->m_maxMTTDepth                     = 221111;
      c->m_maxMTTDepthI                    = 2;

      // speedups
      c->m_qtbttSpeedUp                    = 3;
      c->m_fastTTSplit                     = 5;
      c->m_contentBasedFastQtbt            = true;
      c->m_fastHad                         = false;
      c->m_usePbIntraFast                  = 1;
      c->m_useFastMrg                      = 3;
      c->m_fastLocalDualTreeMode           = 1;
      c->m_fastSubPel                      = 1;
      c->m_FastIntraTools                  = 1;
      c->m_FIMMode                         = 0;
      c->m_useEarlyCU                      = 0;
      c->m_bIntegerET                      = 0;
      c->m_IntraEstDecBit                  = 2;
      c->m_numIntraModesFullRD             = -1;
      c->m_reduceIntraChromaModesFullRD    = true;
      c->m_meReduceTap                     = 2;
      c->m_numRefPics                      = 222111;
      c->m_numRefPicsSCC                   = 0;

      // tools
      c->m_Affine                          = 4;
      c->m_alf                             = 1;
      c->m_alfSpeed                        = 0;
      c->m_allowDisFracMMVD                = 1;
      c->m_blockImportanceMapping          = 1;
      c->m_BDOF                            = 1;
      c->m_ccalf                           = 1;
      c->m_CIIP                            = 0;
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
      c->m_vvencMCTF.MCTFSpeed             = 2;
      c->m_MIP                             = 1;
      c->m_useFastMIP                      = 3;
      c->m_MMVD                            = 3;
      c->m_MRL                             = 1;
      c->m_MTSImplicit                     = 1;
      c->m_PROF                            = 1;
      c->m_SbTMVP                          = 1;
      c->m_SMVD                            = 3;
      // scc
      c->m_IBCFastMethod                   = 3;
      c->m_TSsize                          = 4;

      if( preset == vvencPresetMode::VVENC_MEDIUM_LOWDECNRG )
      {
        c->m_numRefPics                    = 2;
        c->m_deblockLastTLayers            = 1;
        c->m_maxMTTDepth                   = 332222;
        c->m_maxMTTDepthI                  = 3;
        c->m_Affine                        = 3;
        c->m_alfSpeed                      = 1;
        c->m_BCW                           = 2;
        c->m_BDOF                          = 0;
        c->m_DMVR                          = 0;
        c->m_ISP                           = 0;
        c->m_LFNST                         = 0;
        c->m_lumaReshapeEnable             = 0;
        c->m_MIP                           = 0;
        c->m_useFastMIP                    = 0;
        c->m_bUseSAO                       = true;
        c->m_saoScc                        = true;
        c->m_SbTMVP                        = 0;
        c->m_useFastMrg                    = 2;
      }

      break;

    case vvencPresetMode::VVENC_SLOW:

      // motion estimation
      c->m_SearchRange                     = 384;
      c->m_bipredSearchRange               = 4;
      c->m_minSearchWindow                 = 96;
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE3;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND_FAST;

      // partitioning: CTUSize128 QT44MTT32
      c->m_CTUSize                         = 128;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 8;
      c->m_MinQT[ 1 ]                      = 8;
      c->m_MinQT[ 2 ]                      = 4;
      c->m_maxMTTDepth                     = 2;
      c->m_maxMTTDepthI                    = 3;

      // speedups
      c->m_qtbttSpeedUp                    = 2;
      c->m_fastTTSplit                     = 5;
      c->m_contentBasedFastQtbt            = true;
      c->m_fastHad                         = false;
      c->m_usePbIntraFast                  = 1;
      c->m_useFastMrg                      = 3;
      c->m_fastLocalDualTreeMode           = 1;
      c->m_fastSubPel                      = 1;
      c->m_FastIntraTools                  = 0;
      c->m_FIMMode                         = 0;
      c->m_useEarlyCU                      = 0;
      c->m_bIntegerET                      = 0;
      c->m_IntraEstDecBit                  = 1;
      c->m_numIntraModesFullRD             = -1;
      c->m_reduceIntraChromaModesFullRD    = false;
      c->m_meReduceTap                     = 2;
      c->m_numRefPics                      = 2;
      c->m_numRefPicsSCC                   = 0;

      // tools
      c->m_Affine                          = 3;
      c->m_alf                             = 1;
      c->m_alfSpeed                        = 0;
      c->m_allowDisFracMMVD                = 1;
      c->m_BCW                             = 2;
      c->m_blockImportanceMapping          = 1;
      c->m_BDOF                            = 1;
      c->m_ccalf                           = 1;
      c->m_CIIP                            = 1;
      c->m_DepQuantEnabled                 = 1;
      c->m_DMVR                            = 1;
      c->m_EDO                             = 2;
      c->m_Geo                             = 1;
      c->m_AMVRspeed                       = 1;
      c->m_ISP                             = 3;
      c->m_JointCbCrMode                   = 1;
      c->m_LFNST                           = 1;
      c->m_LMChroma                        = 1;
      c->m_lumaReshapeEnable               = 2;
      c->m_vvencMCTF.MCTF                  = 2;
      c->m_vvencMCTF.MCTFSpeed             = 2;
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
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE3;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND;

      // partitioning: CTUSize128 QT44MTT33
      c->m_CTUSize                         = 128;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 8;
      c->m_MinQT[ 1 ]                      = 8;
      c->m_MinQT[ 2 ]                      = 4;
      c->m_maxMTTDepth                     = 333332;
      c->m_maxMTTDepthI                    = 3;

      // speedups
      c->m_qtbttSpeedUp                    = 1;
      c->m_fastTTSplit                     = 1;
      c->m_contentBasedFastQtbt            = false;
      c->m_fastHad                         = false;
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
      c->m_meReduceTap                     = 2;
      c->m_numRefPics                      = 2;
      c->m_numRefPicsSCC                   = 0;

      // tools
      c->m_Affine                          = 1;
      c->m_alf                             = 1;
      c->m_alfSpeed                        = 0;
      c->m_allowDisFracMMVD                = 1;
      c->m_BCW                             = 2;
      c->m_blockImportanceMapping          = 1;
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
      c->m_vvencMCTF.MCTFSpeed             = 2;
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
      c->m_fastInterSearchMode             = VVENC_FASTINTERSEARCH_MODE3;
      c->m_motionEstimationSearchMethod    = VVENC_MESEARCH_DIAMOND_FAST;

      // partitioning: CTUSize128 QT44MTT21
      c->m_CTUSize                         = 128;
      c->m_dualITree                       = 1;
      c->m_MinQT[ 0 ]                      = 8;
      c->m_MinQT[ 1 ]                      = 8;
      c->m_MinQT[ 2 ]                      = 4;
      c->m_maxMTTDepth                     = 1;
      c->m_maxMTTDepthI                    = 2;

      // speedups
      c->m_qtbttSpeedUp                    = 2;
      c->m_fastTTSplit                     = 0;
      c->m_contentBasedFastQtbt            = true;
      c->m_fastHad                         = true;
      c->m_usePbIntraFast                  = 2;
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
      c->m_meReduceTap                     = 2;
      c->m_numRefPics                      = 222221;
      c->m_numRefPicsSCC                   = 0;

      // tools
      c->m_Affine                          = 5;
      c->m_alf                             = 1;
      c->m_alfSpeed                        = 0;
      c->m_allowDisFracMMVD                = 1;
      c->m_BCW                             = 2;
      c->m_blockImportanceMapping          = 0;
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
      c->m_useFastMIP                      = 3;
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
  std::string loglvl("vvenc ");
  switch ( eMsgLevel )
  {
    case VVENC_SILENT : loglvl.append("[silent]: ");  break;
    case VVENC_ERROR  : loglvl.append("[error]: ");   break;
    case VVENC_WARNING: loglvl.append("[warning]: "); break;
    case VVENC_INFO   : loglvl.append("[info]: ");    break;
    case VVENC_NOTICE : loglvl.append("[notice]: ");  break;
    case VVENC_VERBOSE: loglvl.append("[verbose]: "); break;
    case VVENC_DETAILS: loglvl.append("[details]: "); break;
    default: break;
  }

  if( eMsgLevel >= VVENC_INFO )
  {
    css << loglvl << "Internal format                        : " << c->m_PadSourceWidth << "x" << c->m_PadSourceHeight << "  " <<  (double)c->m_FrameRate/c->m_FrameScale << " Hz  "  << getDynamicRangeStr(c->m_HdrMode) << "\n";
    css << loglvl << "Threads                                : " << c->m_numThreads << "  (parallel frames: " << c->m_maxParallelFrames <<")\n";

    css << loglvl << "Rate control                           : ";

    if ( c->m_RCTargetBitrate > 0 )
    {
      if( c->m_RCTargetBitrate < 1000000 )
        css << "VBR  " <<  (double)c->m_RCTargetBitrate/1000.0 << " kbps  ";
      else
        css << "VBR  " <<  (double)c->m_RCTargetBitrate/1000000.0 << " Mbps  ";
      if( c->m_RCNumPasses == 2 )
      {
        css << "two-pass";
        if ( c->m_RCPass >= 0 )
          css << "  pass " << c->m_RCPass << "/2";
      }
      else
        css << "single-pass";
      if( c->m_RCMaxBitrate > 0 && c->m_RCMaxBitrate != INT32_MAX )
      {
        if( c->m_RCMaxBitrate < 1000000 )
          css << "  (max. rate " <<  (double)c->m_RCMaxBitrate/1000.0 << " kbps)";
        else
          css << "  (max. rate " <<  (double)c->m_RCMaxBitrate/1000000.0 << " Mbps)";
      }
      css << "\n";
    }
    else
      css << "QP " <<  c->m_QP << "\n";

    css << loglvl << "Perceptual optimization                : " << (c->m_usePerceptQPA ? "Enabled" : "Disabled") << "\n";
    css << loglvl << "Intra period (keyframe)                : " << c->m_IntraPeriod << "\n";
    css << loglvl << "Decoding refresh type                  : " << vvenc_getDecodingRefreshTypeStr(c->m_DecodingRefreshType,c->m_poc0idr) << "\n";

    if( c->m_masteringDisplay[0] != 0 || c->m_masteringDisplay[1] != 0 || c->m_masteringDisplay[8] != 0  )
    {
      css << loglvl << "Mastering display color volume         : " << vvenc_getMasteringDisplayStr( c->m_masteringDisplay ) << "\n";
  }
    if( c->m_contentLightLevel[0] != 0 || c->m_contentLightLevel[1] != 0 )
    {
      css << loglvl << "Content light level                    : " << vvenc_getContentLightLevelStr( c->m_contentLightLevel ) << "\n";
    }
  }

  if( eMsgLevel >= VVENC_NOTICE )
  {
    css << loglvl << "Sequence PSNR output                   : " << (c->m_printMSEBasedSequencePSNR ? "Linear average, MSE-based" : "Linear average only") << "\n";
    css << loglvl << "Hexadecimal PSNR output                : " << (c->m_printHexPsnr ? "Enabled" : "Disabled") << "\n";
    css << loglvl << "Sequence MSE output                    : " << (c->m_printSequenceMSE ? "Enabled" : "Disabled") << "\n";
    css << loglvl << "Frame MSE output                       : " << (c->m_printFrameMSE ? "Enabled" : "Disabled") << "\n";
    css << loglvl << "Cabac-zero-word-padding                : " << (c->m_cabacZeroWordPaddingEnabled ? "Enabled" : "Disabled") << "\n";
    //css << loglvl << "Frame/Field                            : Frame based coding\n";
    if ( c->m_framesToBeEncoded > 0 )
      css << loglvl << "Frame index                            : " << c->m_framesToBeEncoded << " frames\n";
    else
      css << loglvl << "Frame index                            : all frames\n";

    css << loglvl << "Profile                                : " << getProfileStr( c->m_profile ) << "\n";
    css << loglvl << "Level                                  : " << getLevelStr( c->m_level ) << "\n";
    css << loglvl << "CU size                                : " << c->m_CTUSize << "\n";
    css << loglvl << "Max TB size                            : " << (1 << c->m_log2MaxTbSize) << "\n";
    css << loglvl << "Min CB size                            : " << (1 << c->m_log2MinCodingBlockSize) << "\n";
    css << loglvl << "Motion search range                    : " << c->m_SearchRange << "\n";
    css << loglvl << "QP                                     : " << c->m_QP << "\n";
    css << loglvl << "Max dQP signaling subdiv               : " << c->m_cuQpDeltaSubdiv << "\n";
    css << loglvl << "Cb QP Offset (dual tree)               : " << c->m_chromaCbQpOffset << " (" << c->m_chromaCbQpOffsetDualTree << ")\n";
    css << loglvl << "Cr QP Offset (dual tree)               : " << c->m_chromaCrQpOffset << " (" << c->m_chromaCrQpOffsetDualTree << ")\n";
    css << loglvl << "GOP size                               : " << c->m_GOPSize << "\n";
    css << loglvl << "PicReordering                          : " << c->m_picReordering << "\n";
    css << loglvl << "Input bit depth                        : (Y:" << c->m_inputBitDepth[ 0 ] << ", C:" << c->m_inputBitDepth[ 1 ] << ")\n";
    css << loglvl << "MSB-extended bit depth                 : (Y:" << c->m_MSBExtendedBitDepth[ 0 ] << ", C:" << c->m_MSBExtendedBitDepth[ 1 ] << ")\n";
    css << loglvl << "Internal bit depth                     : (Y:" << c->m_internalBitDepth[ 0 ] << ", C:" << c->m_internalBitDepth[ 1 ] << ")\n";
    css << loglvl << "cu_chroma_qp_offset_subdiv             : " << c->m_cuChromaQpOffsetSubdiv << "\n";
    if (c->m_bUseSAO)
    {
      css << loglvl << "log2_sao_offset_scale_luma             : " << c->m_log2SaoOffsetScale[ 0 ] << "\n";
      css << loglvl << "log2_sao_offset_scale_chroma           : " << c->m_log2SaoOffsetScale[ 1 ] << "\n";
    }
    css << loglvl << "Cost function:                         : " << getCostFunctionStr( c->m_costMode ) << "\n";
    }

  if( eMsgLevel >= VVENC_VERBOSE )
  {    
    // verbose output
    css << "\n";
    css << loglvl << "CODING TOOL CFG: ";
    css << "CTU" << c->m_CTUSize << " QTMin" << vvenc::Log2( c->m_CTUSize / c->m_MinQT[0] ) << vvenc::Log2( c->m_CTUSize / c->m_MinQT[1] ) << "BTT" << c->m_maxMTTDepthI << c->m_maxMTTDepth << " ";
    css << "IBD:" << ((c->m_internalBitDepth[ 0 ] > c->m_MSBExtendedBitDepth[ 0 ]) || (c->m_internalBitDepth[ 1 ] > c->m_MSBExtendedBitDepth[ 1 ])) << " ";
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

    css << "\n" << loglvl << "ENC. ALG. CFG: ";
    css << "QPA:" << c->m_usePerceptQPA << " ";
    css << "HAD:" << c->m_bUseHADME << " ";
    if( c->m_fastHad ) css << "(fast) ";
    css << "RDQ:" << c->m_RDOQ << " ";
    css << "RDQTS:" << c->m_useRDOQTS << " ";
    css << "ASR:" << c->m_bUseASR << " ";
    css << "MinSearchWindow:" << c->m_minSearchWindow << " ";
    css << "EDO:" << c->m_EDO << " ";
    css << "MCTF:" << c->m_vvencMCTF.MCTF << " ";
    css << "BIM:" << c->m_blockImportanceMapping << " ";

    css << "\n" << loglvl << "PRE-ANALYSIS CFG: ";
    css << "STA:" << (int)c->m_sliceTypeAdapt << " ";
    css << "LeadFrames:" << c->m_leadFrames << " ";
    css << "TrailFrames:" << c->m_trailFrames << " ";

    css << "\n" << loglvl << "FAST TOOL CFG: ";
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
    css << "ReduceFilterME:" << c->m_meReduceTap << " ";
    css << "QtbttExtraFast:" << c->m_qtbttSpeedUp << " ";
    css << "FastTTSplit:" << c->m_fastTTSplit << " ";
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
      css << "QuantThr:" << (c->m_quantThresholdVal >> 1) << ".5 ";
    else
      css << "QuantThr:" << (c->m_quantThresholdVal >> 1) << " ";
    css << "SelectiveRDQO:" << ( int ) c->m_useSelectiveRDOQ << " ";

    css << "\n" << loglvl << "RATE CONTROL CFG: ";
    css << "RateControl:" << ( c->m_RCTargetBitrate > 0 ) << " ";
    if ( c->m_RCTargetBitrate > 0 )
    {
      css << "Passes:" << c->m_RCNumPasses << " ";
      css << "Pass:" << c->m_RCPass << " ";
      css << "TargetBitrate:" << c->m_RCTargetBitrate << " ";
      if ( c->m_RCMaxBitrate > 0 && c->m_RCMaxBitrate != INT32_MAX )
      {
        css << "MaxBitrate:" << c->m_RCMaxBitrate << " ";
      }
      css << "RCInitialQP:" << c->m_RCInitialQP << " ";
    }
    else
    {
      css << "QP:" << c->m_QP << " ";
    }

    css << "LookAhead:" << c->m_LookAhead << " ";
    css << "FirstPassMode:" << c->m_FirstPassMode << " ";

    css << "\n" << loglvl << "PARALLEL PROCESSING CFG: ";
    css << "NumThreads:" << c->m_numThreads << " ";
    css << "MaxParallelFrames:" << c->m_maxParallelFrames << " ";
    css << "FppLinesSynchro:" << ( int ) c->m_fppLinesSynchro << " ";
    if( c->m_picPartitionFlag )
    {
      css << "TileParallelCtuEnc:" << c->m_tileParallelCtuEnc << " ";
    }
    css << "WppBitEqual:" << c->m_ensureWppBitEqual << " ";
    css << "WF:" << c->m_entropyCodingSyncEnabled << " ";
    css << "\n";
  }

  vvenc_cfgString = css.str();
  return vvenc_cfgString.c_str();
}

VVENC_DECL void vvenc_set_msg_callback( vvenc_config *cfg, void *msgCtx, vvencLoggingCallback msgFnc )
{
  if( cfg )
  {
    cfg->m_msgCtx = msgCtx;
    cfg->m_msgFnc = msgFnc;
  }
}


VVENC_DECL int vvenc_set_param(vvenc_config *c, const char *name, const char *value)
{
  if ( !name )
  {
    return VVENC_PARAM_BAD_NAME;
  }

  bool bError = false;

  std::string n(name);
  std::string v(value);
  std::transform( n.begin(), n.end(), n.begin(), ::tolower );

  if ( name[0] == '-'  || name[1] == '-' ) // name prefix given - not supported
  {
    return VVENC_PARAM_BAD_NAME;
  }
  else
  {
    std::string namePrefix="--";  // add long option name prefix
    n = namePrefix;
    n.append(name);
  }

  if (!value)
  {
    v = "true";
  }
  else if (value[0] == '=')
  {
    value += 1;
    v = value;
  }

  char *argv[2];
  argv[0]=(char*)n.c_str();
  argv[1]=(char*)v.c_str();

  int ret = vvenc_set_param_list ( c, 2, argv);
  if( ret != 0 )
  {
    return ret;
  }

  return bError ? VVENC_PARAM_BAD_VALUE : 0;
}

VVENC_DECL int vvenc_set_param_list( vvenc_config *c, int argc, char* argv[] )
{
  if ( !argc || !c )
  {
    return -1;
  }

  apputils::VVEncAppCfg cVVEncAppCfg;
  std::stringstream cssO;
  int ret =  cVVEncAppCfg.parse( argc, argv, c, cssO );

  if( !cssO.str().empty() )
  {    
    vvenc::MsgLog msg(c->m_msgCtx,c->m_msgFnc);
    vvencMsgLevel msgLvl = VVENC_INFO;
    if( ret < 0 ) msgLvl = VVENC_ERROR;
    else if( ret == 2 ) msgLvl = VVENC_WARNING;

    msg.log( msgLvl , "%s\n", cssO.str().c_str());
  }

  return ret;
}


VVENC_NAMESPACE_END



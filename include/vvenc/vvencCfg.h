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
/** \file     vvencCfg.h
    \brief    encoder configuration class (header)
*/

#pragma once

#include "vvenc/vvencDecl.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define VVENC_NAMESPACE_BEGIN
#define VVENC_NAMESPACE_END

#ifdef __cplusplus
extern "C" {
#endif

VVENC_NAMESPACE_BEGIN

/* vvdecLoggingCallback:
   callback function to receive messages of the encoder library
  \param[in]  void* caller contex
  \param[in]  int log level
  \param[in]  const char* fmt
  \param[in]  va_list args
  \retval     none
*/
typedef void (*vvencLoggingCallback)(void*, int, const char*, va_list);



// ====================================================================================================================

#define VVENC_MAX_GOP                         64     // max. value of hierarchical GOP size
#define VVENC_MAX_NUM_REF_PICS                29     // max. number of pictures used for reference
#define VVENC_MAX_TLAYER                      7      // Explicit temporal layer QP offset - max number of temporal layer
#define VVENC_MAX_NUM_CQP_MAPPING_TABLES      3      // Maximum number of chroma QP mapping tables (Cb, Cr and joint Cb-Cr)
#define VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA 8
#define VVENC_MCTF_RANGE                      4      // max number of frames used for MCTF filtering in forward / backward direction
#define VVENC_MAX_NUM_COMP                    3      // max number of components
#define VVENC_MAX_QP_VALS_CHROMA              8      // max number qp vals in array
#define VVENC_MAX_MCTF_FRAMES                 16
#define VVENC_MAX_STRING_LEN                  1024   // max length of string/filename

// ====================================================================================================================



typedef enum
{
  VVENC_SILENT  = 0,
  VVENC_ERROR   = 1,
  VVENC_WARNING = 2,
  VVENC_INFO    = 3,
  VVENC_NOTICE  = 4,
  VVENC_VERBOSE = 5,
  VVENC_DETAILS = 6
}vvencMsgLevel;


typedef enum
{
 VVENC_NONE      = -1,
 VVENC_FASTER    = 0,
 VVENC_FAST      = 1,
 VVENC_MEDIUM    = 2,
 VVENC_SLOW      = 3,
 VVENC_SLOWER    = 4,
 VVENC_FIRSTPASS = 254,
 VVENC_TOOLTEST  = 255,
}vvencPresetMode;

/**
  \ingroup VVEnc
  The class SliceType enumerates several supported slice types.
*/
typedef enum
{
  VVENC_B_SLICE               = 0,
  VVENC_P_SLICE               = 1,
  VVENC_I_SLICE               = 2,
  VVENC_NUMBER_OF_SLICE_TYPES = 3
}vvencSliceType;

/**
  \ingroup VVEncExternalInterfaces
  \enum Profile
  The enum Profile enumerates supported profiles
*/
typedef enum
{
  VVENC_PROFILE_AUTO                         =  0,
  VVENC_MAIN_10                              =  1,
  VVENC_MAIN_10_STILL_PICTURE                =  1 + 64,
  VVENC_MAIN_10_444                          = 33,
  VVENC_MAIN_10_444_STILL_PICTURE            = 33 + 64,
  VVENC_MULTILAYER_MAIN_10                   = 17,
  VVENC_MULTILAYER_MAIN_10_STILL_PICTURE     = 17 + 64,
  VVENC_MULTILAYER_MAIN_10_444               = 49,
  VVENC_MULTILAYER_MAIN_10_444_STILL_PICTURE = 49 + 64,
}vvencProfile;


/**
  \ingroup VVEncExternalInterfaces
  \enum Tier
  The enum Tier enumerates supported tier
*/
typedef enum
{
  VVENC_TIER_MAIN = 0,
  VVENC_TIER_HIGH = 1,
  VVENC_NUMBER_OF_TIERS
}vvencTier;

/**
  \ingroup VVEncExternalInterfaces
  \enum Name
  The enum Name enumerates supported level names
*/
typedef enum
{
  VVENC_LEVEL_AUTO = 0,
  VVENC_LEVEL1   = 16,
  VVENC_LEVEL2   = 32,
  VVENC_LEVEL2_1 = 35,
  VVENC_LEVEL3   = 48,
  VVENC_LEVEL3_1 = 51,
  VVENC_LEVEL4   = 64,
  VVENC_LEVEL4_1 = 67,
  VVENC_LEVEL5   = 80,
  VVENC_LEVEL5_1 = 83,
  VVENC_LEVEL5_2 = 86,
  VVENC_LEVEL6   = 96,
  VVENC_LEVEL6_1 = 99,
  VVENC_LEVEL6_2 = 102,
  VVENC_LEVEL6_3 = 105,
  VVENC_LEVEL15_5 = 255,
  VVENC_NUMBER_OF_LEVELS
}vvencLevel;


/// supported IDR types
typedef enum
{
  VVENC_DRT_NONE               = 0,
  VVENC_DRT_CRA                = 1,
  VVENC_DRT_IDR                = 2,
  VVENC_DRT_RECOVERY_POINT_SEI = 3,
  VVENC_DRT_IDR2               = 4,
  VVENC_DRT_CRA_CRE            = 5,             //constrained RASL encoding
}vvencDecodingRefreshType;

typedef enum
{
  VVENC_SEG_OFF,
  VVENC_SEG_FIRST,
  VVENC_SEG_MID,
  VVENC_SEG_LAST
}vvencSegmentMode;


typedef enum
{
  VVENC_HDR_OFF     = 0, // SDR
  VVENC_HDR_PQ,          // HDR10, Dolby
  VVENC_HDR_HLG,         // Hybrid Log Gamma
  VVENC_HDR_PQ_BT2020,   // HDR10, Dolby + BT.2020
  VVENC_HDR_HLG_BT2020,  // Hybrid Log Gamma + BT.2020
  VVENC_HDR_USER_DEFINED // user defined HDR mode (to provide old HDR modes, HDR is set individually)
}vvencHDRMode;

typedef enum
{
  VVENC_NAL_UNIT_CODED_SLICE_TRAIL = 0,   // 0
  VVENC_NAL_UNIT_CODED_SLICE_STSA,        // 1
  VVENC_NAL_UNIT_CODED_SLICE_RADL,        // 2
  VVENC_NAL_UNIT_CODED_SLICE_RASL,        // 3

  VVENC_NAL_UNIT_RESERVED_VCL_4,
  VVENC_NAL_UNIT_RESERVED_VCL_5,
  VVENC_NAL_UNIT_RESERVED_VCL_6,

  VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL,  // 7
  VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP,    // 8
  VVENC_NAL_UNIT_CODED_SLICE_CRA,         // 9
  VVENC_NAL_UNIT_CODED_SLICE_GDR,         // 10

  VVENC_NAL_UNIT_RESERVED_IRAP_VCL_11,
  VVENC_NAL_UNIT_RESERVED_IRAP_VCL_12,
  VVENC_NAL_UNIT_DCI,                     // 13
  VVENC_NAL_UNIT_VPS,                     // 14
  VVENC_NAL_UNIT_SPS,                     // 15
  VVENC_NAL_UNIT_PPS,                     // 16
  VVENC_NAL_UNIT_PREFIX_APS,              // 17
  VVENC_NAL_UNIT_SUFFIX_APS,              // 18
  VVENC_NAL_UNIT_PH,                      // 19
  VVENC_NAL_UNIT_ACCESS_UNIT_DELIMITER,   // 20
  VVENC_NAL_UNIT_EOS,                     // 21
  VVENC_NAL_UNIT_EOB,                     // 22
  VVENC_NAL_UNIT_PREFIX_SEI,              // 23
  VVENC_NAL_UNIT_SUFFIX_SEI,              // 24
  VVENC_NAL_UNIT_FD,                      // 25

  VVENC_NAL_UNIT_RESERVED_NVCL_26,
  VVENC_NAL_UNIT_RESERVED_NVCL_27,

  VVENC_NAL_UNIT_UNSPECIFIED_28,
  VVENC_NAL_UNIT_UNSPECIFIED_29,
  VVENC_NAL_UNIT_UNSPECIFIED_30,
  VVENC_NAL_UNIT_UNSPECIFIED_31,
  VVENC_NAL_UNIT_INVALID
}vvencNalUnitType;

typedef enum
{
  VVENC_CHROMA_400        = 0,
  VVENC_CHROMA_420        = 1,
  VVENC_CHROMA_422        = 2,
  VVENC_CHROMA_444        = 3,
  VVENC_NUM_CHROMA_FORMAT = 4
}vvencChromaFormat;

typedef enum
{
  VVENC_RCM_AUTO          = -1,
  VVENC_RCM_OFF           = 0,
  VVENC_RCM_CTU_LEVEL     = 1,
  VVENC_RCM_PICTURE_LEVEL = 2,
  VVENC_RCM_GOP_LEVEL     = 3
}vvencRateControlMode;

typedef enum
{
  VVENC_FASTINTERSEARCH_AUTO     = 0,
  VVENC_FASTINTERSEARCH_MODE1    = 1,
  VVENC_FASTINTERSEARCH_MODE2    = 2,
  VVENC_FASTINTERSEARCH_MODE3    = 3
}vvencFastInterSearchMode;

/// supported ME search methods
typedef enum
{
  VVENC_MESEARCH_FULL              = 0,
  VVENC_MESEARCH_DIAMOND           = 1,
  VVENC_MESEARCH_SELECTIVE         = 2,
  VVENC_MESEARCH_DIAMOND_ENHANCED  = 3,
  VVENC_MESEARCH_DIAMOND_FAST      = 4,
  VVENC_MESEARCH_NUMBER_OF_METHODS = 5
}vvencMESearchMethod;

typedef enum
{
  VVENC_COST_STANDARD_LOSSY              = 0,
  VVENC_COST_SEQUENCE_LEVEL_LOSSLESS     = 1,
  VVENC_COST_LOSSLESS_CODING             = 2,
  VVENC_COST_MIXED_LOSSLESS_LOSSY_CODING = 3
}vvencCostMode;

typedef enum
{
  VVENC_HASHTYPE_MD5        = 0,
  VVENC_HASHTYPE_CRC        = 1,
  VVENC_HASHTYPE_CHECKSUM   = 2,
  VVENC_HASHTYPE_NONE       = 3,
  VVENC_NUMBER_OF_HASHTYPES = 4
}vvencHashType;

// ====================================================================================================================

typedef struct vvencGOPEntry
{
  int    m_POC;
  int    m_QPOffset;
  double m_QPOffsetModelOffset;
  double m_QPOffsetModelScale;
  int    m_CbQPoffset;
  int    m_CrQPoffset;
  double m_QPFactor;
  int    m_tcOffsetDiv2;
  int    m_betaOffsetDiv2;
  int    m_CbTcOffsetDiv2;
  int    m_CbBetaOffsetDiv2;
  int    m_CrTcOffsetDiv2;
  int    m_CrBetaOffsetDiv2;
  int    m_temporalId;
  bool   m_refPic;
  char   m_sliceType;
  int    m_numRefPicsActive[ 2 ];
  int    m_numRefPics[ 2 ];
  int    m_deltaRefPics[ 2 ][ VVENC_MAX_NUM_REF_PICS ];
  bool   m_isEncoded;
  bool   m_ltrp_in_slice_header_flag;
}vvencGOPEntry;

VVENC_DECL void vvenc_GOPEntry_default(vvencGOPEntry *GOPEntry );


// ====================================================================================================================

typedef struct vvencRPLEntry
{
  int    m_POC;
  int    m_temporalId;
  bool   m_refPic;
  bool   m_ltrp_in_slice_header_flag;
  int    m_numRefPicsActive;
  char   m_sliceType;
  int    m_numRefPics;
  int    m_deltaRefPics[ VVENC_MAX_NUM_REF_PICS ];
}vvencRPLEntry;

VVENC_DECL void vvenc_RPLEntry_default(vvencRPLEntry *RPLEntry );


typedef struct vvencWCGChromaQPControl
{
  bool   enabled        ;    // Enabled flag (0:default)
  double chromaCbQpScale;    // Chroma Cb QP Scale (1.0:default)
  double chromaCrQpScale;    // Chroma Cr QP Scale (1.0:default)
  double chromaQpScale  ;    // Chroma QP Scale (0.0:default)
  double chromaQpOffset ;    // Chroma QP Offset (0.0:default)
}vvencWCGChromaQPControl;

VVENC_DECL void vvenc_WCGChromaQPControl_default(vvencWCGChromaQPControl *WCGChromaQPControl );


typedef struct vvencChromaQpMappingTableParams
{
  int               m_numQpTables;
  int               m_qpBdOffset;
  bool              m_sameCQPTableForAllChromaFlag;
  int               m_qpTableStartMinus26   [ VVENC_MAX_NUM_CQP_MAPPING_TABLES ];
  int               m_numPtsInCQPTableMinus1[ VVENC_MAX_NUM_CQP_MAPPING_TABLES ];
  int               m_deltaQpInValMinus1    [ VVENC_MAX_NUM_CQP_MAPPING_TABLES ][16];
  int               m_deltaQpOutVal         [ VVENC_MAX_NUM_CQP_MAPPING_TABLES ][16];
}vvencChromaQpMappingTableParams;

VVENC_DECL void vvenc_ChromaQpMappingTableParams_default(vvencChromaQpMappingTableParams *ChromaQpMappingTableParams );


typedef struct vvencReshapeCW
{
  unsigned int binCW[3];
  int          updateCtrl;
  int          adpOption;
  unsigned int initialCW;
  int          rspPicSize;
  int          rspFps;
  int          rspBaseQP;
  int          rspTid;
  int          rspFpsToIp;
}vvencReshapeCW;

VVENC_DECL void vvenc_ReshapeCW_default(vvencReshapeCW *ReshapeCW );


typedef struct vvencMCTF
{
  int                 MCTF;
  int                 MCTFSpeed;
  bool                MCTFFutureReference;
  int                 mctfUnused1;                                                       // TODO: remove unused memory from configuration
  int                 mctfUnused2;                                                       // TODO: remove unused memory from configuration

  int                 numFrames;
  int                 MCTFFrames[VVENC_MAX_MCTF_FRAMES];
  int                 numStrength;
  double              MCTFStrengths[VVENC_MAX_MCTF_FRAMES];
}vvencMCTF;

VVENC_DECL void vvenc_vvencMCTF_default(vvencMCTF *vvencMCTF );

// ====================================================================================================================

typedef struct vvenc_config
{
  // core config params
  int                 m_SourceWidth;                                                     // source width in pixel
  int                 m_SourceHeight;                                                    // source height in pixel (when interlaced = field height)
  int                 m_FrameRate;                                                       // source frame-rates (Hz) Numerator
  int                 m_FrameScale;                                                      // source frame-rates (Hz) Denominator
  int                 m_TicksPerSecond;                                                  // ticks per second e.g. 90000 for dts generation (1..27000000)
  int                 m_framesToBeEncoded;                                               // number of encoded frames (default: 0, all)
  int                 m_inputBitDepth[ 2 ];                                              // bit-depth of input pictures (2d array for luma,chroma)

  int                 m_numThreads;                                                      // number of worker threads ( if <0: <720p 4threads, else 8threads (limited to available cores))

  int                 m_QP;                                                              // QP value of key-picture (0-63, default: 32)
  int                 m_RCTargetBitrate;                                                 // target bitrate in bps, (default. 0 (rc disabled))

  vvencMsgLevel       m_verbosity;                                                       // encoder verbosity level

  // basic config params

  vvencProfile        m_profile;                                                         // profile 
  vvencTier           m_levelTier;                                                       // Tier to use for interpretation of level (main or high)
  vvencLevel          m_level;                                                           // level limit

  int                 m_IntraPeriod;                                                     // period of I-slice in frames  (random access period)
  int                 m_IntraPeriodSec;                                                  // period of I-slice in seconds (random access period)
  vvencDecodingRefreshType m_DecodingRefreshType;                                        // random access type
  int                 m_GOPSize;                                                         // GOP size of hierarchical structure

  int                 m_RCNumPasses;                                                     // number of rc passes (default: -1, if not set and bitrate > 0 2-pass rc will be used)
  int                 m_RCPass;                                                          // current pass (0,1) for rc (only needed for 2-pass rc) 
  bool                m_cfgUnused1;                                                      // TODO: remove unused memory from configuration

  int                 m_internalBitDepth[ 2 ];                                           // bit-depth codec operates at (input/output files will be converted) (2d array for luma,chroma)

  vvencHDRMode        m_HdrMode;                                                         // High Dynamic Range mode (default: off)
  vvencSegmentMode    m_SegmentMode;                                                     // segment position for segment concatenation (only needed, when multiple separate segments are used)

  bool                m_usePerceptQPA;                                                   // usage of perceptually motivated input-adaptive QP modification, abbrev. perceptual QP adaptation (QPA).

  uint32_t            m_numTileCols;                                                     // number of tile columns
  uint32_t            m_numTileRows;                                                     // number of tile rows

  // expert config params
  int                 m_conformanceWindowMode;
  int                 m_confWinLeft;
  int                 m_confWinRight;
  int                 m_confWinTop;
  int                 m_confWinBottom;

  unsigned            m_temporalSubsampleRatio;                                          // temporal subsample ratio, 2 means code every two frames

  int                 m_PadSourceWidth;                                                  // source width in pixel
  int                 m_PadSourceHeight;                                                 // source height in pixel (when interlaced = field height)

  int                 m_aiPad[ 2 ];                                                      // number of padded pixels for width and height
  bool                m_enablePictureHeaderInSliceHeader;
  int                 m_AccessUnitDelimiter;                                             // add Access Unit Delimiter NAL units, default: auto (only enable if needed by dependent options)

  bool                m_printMSEBasedSequencePSNR;
  bool                m_printHexPsnr;
  bool                m_printFrameMSE;
  bool                m_printSequenceMSE;
  bool                m_cabacZeroWordPaddingEnabled;

  unsigned            m_subProfile;
  unsigned            m_bitDepthConstraintValue;
  bool                m_intraOnlyConstraintFlag;

  bool                m_rewriteParamSets;                                                // Flag to enable rewriting of parameter sets at random access points
  bool                m_idrRefParamList;                                                 // indicates if reference picture list syntax elements are present in slice headers of IDR pictures

  vvencRPLEntry       m_cfgUnused2[ 64 ];                                                // TODO: remove unused memory from configuration
  vvencRPLEntry       m_cfgUnused3[ 64 ];                                                // TODO: remove unused memory from configuration

  vvencGOPEntry       m_GOPList[ VVENC_MAX_GOP ];                                        // the coding structure entries from the config file

  int                 m_cfgUnused4[ 7 ];                                                 // TODO: remove unused memory from configuration
  int                 m_cfgUnused5[ 7 ];                                                 // TODO: remove unused memory from configuration
  int                 m_cfgUnused6;                                                      // TODO: remove unused memory from configuration
  int                 m_cfgUnused7;                                                      // TODO: remove unused memory from configuration
  int                 m_cfgUnused8;                                                      // TODO: remove unused memory from configuration

  bool                m_useSameChromaQPTables;
  vvencChromaQpMappingTableParams m_chromaQpMappingTableParams;
  int                 m_intraQPOffset;                                                   // QP offset for intra slice (integer)
  bool                m_lambdaFromQPEnable;                                              // enable flag for QP:lambda fix
  double              m_adLambdaModifier[ VVENC_MAX_TLAYER ];                            // Lambda modifier array for each temporal layer
  double              m_adIntraLambdaModifier[ VVENC_MAX_TLAYER ];                       // Lambda modifier for Intra pictures, one for each temporal layer. If size>temporalLayer, then use [temporalLayer], else if size>0, use [size()-1], else use m_adLambdaModifier.
  double              m_dIntraQpFactor;                                                  // Intra Q Factor. If negative, use a default equation: 0.57*(1.0 - Clip3( 0.0, 0.5, 0.05*(double)(isField ? (GopSize-1)/2 : GopSize-1) ))
  int                 m_qpInValsCb[VVENC_MAX_QP_VALS_CHROMA];                            // qp input values used to derive the chroma QP mapping table
  int                 m_qpInValsCr[VVENC_MAX_QP_VALS_CHROMA];                            // qp input values used to derive the chroma QP mapping table
  int                 m_qpInValsCbCr[VVENC_MAX_QP_VALS_CHROMA];                          // qp input values used to derive the chroma QP mapping table
  int                 m_qpOutValsCb[VVENC_MAX_QP_VALS_CHROMA];                           // qp output values used to derive the chroma QP mapping table
  int                 m_qpOutValsCr[VVENC_MAX_QP_VALS_CHROMA];                           // qp output values used to derive the chroma QP mapping table
  int                 m_qpOutValsCbCr[VVENC_MAX_QP_VALS_CHROMA];                         // qp output values used to derive the chroma QP mapping table
  int                 m_cuQpDeltaSubdiv;                                                 // Maximum subdiv for CU luma Qp adjustment (0:default)
  int                 m_cuChromaQpOffsetSubdiv;                                          // If negative, then do not apply chroma qp offsets.
  int                 m_chromaCbQpOffset;                                                // Chroma Cb QP Offset (0:default)
  int                 m_chromaCrQpOffset;                                                // Chroma Cr QP Offset (0:default)
  int                 m_chromaCbQpOffsetDualTree;                                        // Chroma Cb QP Offset for dual tree (overwrite m_chromaCbQpOffset for dual tree)
  int                 m_chromaCrQpOffsetDualTree;                                        // Chroma Cr QP Offset for dual tree (overwrite m_chromaCrQpOffset for dual tree)
  int                 m_chromaCbCrQpOffset;                                              // QP Offset for joint Cb-Cr mode
  int                 m_chromaCbCrQpOffsetDualTree;                                      // QP Offset for joint Cb-Cr mode (overwrite m_chromaCbCrQpOffset for dual tree)
  int                 m_sliceChromaQpOffsetPeriodicity;                                  // Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.
  int                 m_sliceChromaQpOffsetIntraOrPeriodic[ 2 ];                         // Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.

  int                 m_usePerceptQPATempFiltISlice;                                     // Flag indicating if temporal high-pass filtering in visual activity calculation in QPA should (true) or shouldn't (false) be applied for I-slices

  bool                m_lumaLevelToDeltaQPEnabled;
  vvencWCGChromaQPControl  m_wcgChromaQpControl;

  vvencChromaFormat   m_internChromaFormat;
  bool                m_useIdentityTableForNon420Chroma;
  int                 m_outputBitDepth[ 2 ];                                             // bit-depth of output file
  int                 m_MSBExtendedBitDepth[ 2 ];                                        // bit-depth of input samples after MSB extension
  vvencCostMode       m_costMode;                                                        // Cost mode to use

  vvencHashType       m_decodedPictureHashSEIType;                                       // Checksum mode for decoded picture hash SEI message
  bool                m_bufferingPeriodSEIEnabled;
  bool                m_pictureTimingSEIEnabled;
  bool                m_decodingUnitInfoSEIEnabled;

  bool                m_entropyCodingSyncEnabled;
  bool                m_entryPointsPresent;

  unsigned            m_CTUSize;
  unsigned            m_MinQT[ 3 ];                                                      // 0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned            m_maxMTTDepth;
  unsigned            m_maxMTTDepthI;
  unsigned            m_maxMTTDepthIChroma;
  unsigned            m_maxBT[3];
  unsigned            m_maxTT[3];
  bool                m_dualITree;
  unsigned            m_MaxCodingDepth;                                                  // max. total CU depth - includes depth of transform-block structure
  unsigned            m_log2DiffMaxMinCodingBlockSize;                                   // difference between largest and smallest CU depth
  int                 m_log2MaxTbSize;
  int                 m_log2MinCodingBlockSize;

  bool                m_bUseASR;                                                         // flag for using adaptive motion search range
  bool                m_bUseHADME;                                                       // flag for using HAD in sub-pel ME
  int                 m_RDOQ;                                                            // flag for using RD optimized quantization
  bool                m_useRDOQTS;                                                       // flag for using RD optimized quantization for transform skip
  bool                m_useSelectiveRDOQ;                                                // flag for using selective RDOQ

  bool                m_JointCbCrMode;
  int                 m_cabacInitPresent;
  bool                m_useFastLCTU;
  int                 m_usePbIntraFast;
  int                 m_useFastMrg;
  int                 m_useAMaxBT;
  bool                m_fastQtBtEnc;
  bool                m_contentBasedFastQtbt;
  int                 m_fastInterSearchMode;                                             // Parameter that controls fast encoder settings
  int                 m_useEarlyCU;                                                      // flag for using Early CU setting
  bool                m_useFastDecisionForMerge;                                         // flag for using Fast Decision Merge RD-Cost

  bool                m_bDisableIntraCUsInInterSlices;                                   // Flag for disabling intra predicted CUs in inter slices.
  bool                m_cfgUnused9;                                                      // TODO: remove unused memory from configuration
  bool                m_bFastUDIUseMPMEnabled;
  bool                m_bFastMEForGenBLowDelayEnabled;

  bool                m_MTSImplicit;
  int                 m_TMVPModeId;
  bool                m_DepQuantEnabled;
  bool                m_SignDataHidingEnabled;

  bool                m_MIP;
  int                 m_useFastMIP;

  unsigned            m_maxNumMergeCand;                                                 // Max number of merge candidates
  unsigned            m_maxNumAffineMergeCand;                                           // Max number of affine merge candidates
  int                 m_Geo;
  unsigned            m_maxNumGeoCand;
  int                 m_FastIntraTools;
  int                 m_IntraEstDecBit;                                                  // Intra estimation decimation factor.

  int                 m_RCInitialQP;
  bool                m_RCForceIntraQP;

  int                 m_motionEstimationSearchMethod;
  int                 m_motionEstimationSearchMethodSCC;
  bool                m_bRestrictMESampling;                                             // Restrict sampling for the Selective ME
  int                 m_SearchRange;                                                     // ME search range
  int                 m_bipredSearchRange;                                               // ME search range for bipred refinement
  int                 m_minSearchWindow;                                                 // ME minimum search window size for the Adaptive Window ME
  bool                m_bClipForBiPredMeEnabled;                                         // Enables clipping for Bi-Pred ME.
  bool                m_bFastMEAssumingSmootherMVEnabled;                                // Enables fast ME assuming a smoother MV.
  bool                m_bIntegerET;                                                      // Enables early termination for integer motion search.
  int                 m_fastSubPel;
  int                 m_SMVD;
  int                 m_AMVRspeed;
  bool                m_LMChroma;
  bool                m_horCollocatedChromaFlag;
  bool                m_verCollocatedChromaFlag;
  bool                m_MRL;
  bool                m_BDOF;
  bool                m_DMVR;
  int                 m_EDO;
  int                 m_lumaReshapeEnable;
  int                 m_reshapeSignalType;
  int                 m_updateCtrl;
  int                 m_adpOption;
  int                 m_initialCW;
  int                 m_LMCSOffset;
  vvencReshapeCW      m_reshapeCW;
  int                 m_Affine;
  bool                m_PROF;
  bool                m_AffineType;
  int                 m_MMVD;
  int                 m_MmvdDisNum;
  bool                m_allowDisFracMMVD;
  int                 m_CIIP;
  bool                m_SbTMVP;
  int                 m_SBT;                                                             // Sub-Block Transform for inter blocks
  int                 m_LFNST;
  int                 m_MTS;
  int                 m_MTSIntraMaxCand;
  int                 m_ISP;
  int                 m_TS;
  int                 m_TSsize;
  int                 m_useChromaTS;
  int                 m_useBDPCM;

  int                 m_rprEnabledFlag;
  bool                m_resChangeInClvsEnabled;
  bool                m_craAPSreset;
  bool                m_rprRASLtoolSwitch;

  int                 m_IBCMode;
  int                 m_IBCFastMethod;

  int                 m_BCW;

  int                 m_FIMMode;
  int                 m_FastInferMerge;

  bool                m_bLoopFilterDisable;                                              // flag for using deblocking filter
  bool                m_loopFilterOffsetInPPS;                                           // offset for deblocking filter in 0 = slice header, 1 = PPS
  int                 m_loopFilterBetaOffsetDiv2[3];                                     // beta offset for deblocking filter
  int                 m_loopFilterTcOffsetDiv2[3];                                       // tc offset for deblocking filter
  int                 m_cfgUnused10;                                                     // TODO: remove unused memory from configuration

  bool                m_bDisableLFCrossTileBoundaryFlag;                                 // 0: filter across tile boundaries 1: do not filter across tile boundaries
  bool                m_bDisableLFCrossSliceBoundaryFlag;                                // 0: filter across slice boundaries 1: do not filter across slice boundaries

  bool                m_bUseSAO;
  double              m_saoEncodingRate;                                                 // When >0 SAO early picture termination is enabled for luma and chroma
  double              m_saoEncodingRateChroma;                                           // The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma.
  unsigned            m_log2SaoOffsetScale[ 2 ];                                         // n umber of bits for the upward bit shift operation on the decoded SAO offsets
  int                 m_saoOffsetBitShift[ 2 ];

  bool                m_decodingParameterSetEnabled;                                     // enable decoding parameter set
  int                 m_vuiParametersPresent;                                            // enable generation of VUI parameters; -1 auto enable, 0: off 1: enable
  bool                m_hrdParametersPresent;                                            // enable generation or HRD parameters; 0: off 1: enable
  bool                m_aspectRatioInfoPresent;                                          // Signals whether aspect_ratio_idc is present
  int                 m_aspectRatioIdc;                                                  // aspect_ratio_idc
  int                 m_sarWidth;                                                        // horizontal size of the sample aspect ratio
  int                 m_sarHeight;                                                       // vertical size of the sample aspect ratio
  bool                m_colourDescriptionPresent;                                        // Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present
  int                 m_colourPrimaries;                                                 // Indicates chromaticity coordinates of the source primaries
  int                 m_transferCharacteristics;                                         // Indicates the opto-electronic transfer characteristics of the source
  int                 m_matrixCoefficients;                                              // Describes the matrix coefficients used in deriving luma and chroma from RGB primaries
  bool                m_chromaLocInfoPresent;                                            // Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present
  int                 m_chromaSampleLocTypeTopField;                                     // Specifies the location of chroma samples for top field
  int                 m_chromaSampleLocTypeBottomField;                                  // Specifies the location of chroma samples for bottom field
  int                 m_chromaSampleLocType;                                             // Specifies the location of chroma samples for progressive content
  bool                m_overscanInfoPresent;                                             // Signals whether overscan_appropriate_flag is present
  bool                m_overscanAppropriateFlag;                                         // Indicates whether conformant decoded pictures are suitable for display using overscan
  bool                m_cfgUnused11;                                                     // TODO: remove unused memory from configuration
  bool                m_videoFullRangeFlag;                                              // Indicates the black level and range of luma and chroma signals

  unsigned int        m_masteringDisplay[10];                                            // mastering display colour volume, vector of size 10, format: G(x,y)B(x,y)R(x,y)WP(x,y)L(max,min), 0 <= GBR,WP <= 50000, 0 <= L <= uint (SEI)
                                                                                         // GBR xy coordinates in increments of 1/50000 (in the ranges 0 to 50000) (e.g. 0.333 = 16667)
                                                                                         // min/max luminance value in units of 1/10000 candela per square metre
  unsigned int        m_contentLightLevel[2];                                            // upper bound on the max light level and max avg light level among all individual samples in a 4:4:4 representation. in units of candelas per square metre (SEI)
  int                 m_preferredTransferCharacteristics;                                // Alternative transfer characteristics SEI which will override the corresponding entry in the VUI, if < 0 SEI is not written")

  bool                m_alf;                                                             // Adaptive Loop Filter
  bool                m_useNonLinearAlfLuma;
  bool                m_useNonLinearAlfChroma;
  unsigned            m_maxNumAlfAlternativesChroma;
  bool                m_ccalf;
  int                 m_ccalfQpThreshold;
  int                 m_alfTempPred;                                                     // Indicates using of temporal filter data prediction through APS
  int                 m_alfSpeed;

  vvencMCTF           m_vvencMCTF;

  int                 m_quantThresholdVal;
  int                 m_qtbttSpeedUp;
  int                 m_qtbttSpeedUpMode;
  int                 m_fastTTSplit;
  float               m_fastTT_th;

  int                 m_fastLocalDualTreeMode;

  int                 m_maxParallelFrames;
  int                 m_ensureWppBitEqual;                                               // Flag indicating bit equalitiy for single thread runs respecting multithread restrictions
  bool                m_tileParallelCtuEnc;

  bool                m_picPartitionFlag;
  unsigned int        m_tileColumnWidth[10];
  unsigned int        m_tileRowHeight[10];
  uint32_t            m_numExpTileCols;                                                  // number of explicitly specified tile columns
  uint32_t            m_numExpTileRows;                                                  // number of explicitly specified tile rows
  uint32_t            m_numSlicesInPic;                                                  // derived number of rectangular slices in the picture (raster-scan slice specified at slice level)

  // decode bitstream options
  int                 m_switchPOC;                                                       // dbg poc.
  int                 m_switchDQP;                                                       // switch DQP.
  int                 m_fastForwardToPOC;                                                // get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC
  bool                m_stopAfterFFtoPOC;
  bool                m_bs2ModPOCAndType;
  bool                m_forceDecodeBitstream1;
  char                m_decodeBitstreams[2][VVENC_MAX_STRING_LEN];                       // filename for decode bitstreams.

  // trace rules
  bool                m_listTracingChannels;
  char                m_traceRule[VVENC_MAX_STRING_LEN];
  char                m_traceFile[VVENC_MAX_STRING_LEN];

  char                m_summaryOutFilename[VVENC_MAX_STRING_LEN];                        // filename to use for producing summary output file.
  char                m_summaryPicFilenameBase[VVENC_MAX_STRING_LEN];                    // Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended.
  unsigned            m_summaryVerboseness;                                              // Specifies the level of the verboseness of the text output.
  int                 m_numIntraModesFullRD;                                             // Number Modes for Full RD Intra Search
  bool                m_reduceIntraChromaModesFullRD;                                    // Reduce Number Modes for Full RD Intra Chroma Search

  // reserved parameters for internal use
  int                 m_reservedInt[5];
  int                 m_deblockLastTLayers;
  int                 m_leadFrames;                                                      // number of leading frames to to be given before starting with POC 0
  int                 m_trailFrames;                                                     // number of trailing frames to to be given after last frame to be encoded
  int                 m_LookAhead;                                                       // enable pre-analysis pass with picture look-ahead
  int                 m_explicitAPSid;

  bool                m_picReordering;
  bool                m_reservedFlag[6];
  bool                m_fastHad;
  bool                m_sliceTypeAdapt;                                                  // enable slice type (for now B-to-I frame) adaptation (STA)
  bool                m_treatAsSubPic;

  double              m_reservedDouble[10];

  // internal state variables
  bool                m_configDone;                                                      // state variable, Private context used for internal data ( do not change )
  bool                m_confirmFailed;                                                   // state variable, Private context used for internal data ( do not change )
  vvencLoggingCallback m_msgFnc;                                                         // logger callback function (internal info/errors will use this function to return log messages)
  void                *m_msgCtx;                                                         // caller context ( if not needed null )

}vvenc_config;

/* vvenc_config_default
  This method initializes the vvenc_config parameters to default values (constructor).
 \param[in]  vvenc_config* pointer to vvenc_config struct that contains encoder parameters
 \retval     none
 \pre        None
*/
VVENC_DECL void vvenc_config_default( vvenc_config *cfg );

/* vvenc_init_default
  This method initializes the vvenc_config parameters to default values by using all required
  parameters size, framerate, bitrate, qp, preset.
 \param[in]  vvenc_config* pointer to vvenc_config struct that contains encoder parameters
 \param[in]  width  source width in pixel
 \param[in]  height source height in pixel
 \param[in]  framerate source frame-rates (Hz)
 \param[in]  targetbitrate bitrate in bps (0: use fix qp, >0: enable rate control)
 \param[in]  qp QP value of key-picture (integer in range 0 to 63, default: 32)
 \param[in]  preset enum of used preset (default: VVENC_MEDIUM)
 \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVENC_OK indicates success.
 \pre        None
*/
VVENC_DECL int vvenc_init_default( vvenc_config *cfg, int width, int height, int framerate, int targetbitrate, int qp, vvencPresetMode preset );

/* vvenc_init_preset
  This method overwrites encoder parameter by using a preset.
 \param[in]  vvenc_config* pointer to vvenc_config struct that contains encoder parameters
 \param[in]  preset enum of used preset (default: VVENC_MEDIUM)
 \retval     int if non-zero an error occurred (see ErrorCodes), otherwise VVENC_OK indicates success.
 \pre        vvenc_config_default() or vvenc_init_default() must be called first 
*/
VVENC_DECL int vvenc_init_preset( vvenc_config *cfg, vvencPresetMode preset );


/* vvenc_set_logging_callback
 This method registers a log message callback function.
 This callback is automatically used when calling vvenc_encoder_open().
 If no such function has been registered, the library will omit all messages.
 \param[in]  vvenc_config* pointer to vvenc_config struct that contains encoder parameters
 \param[in]  msgCtx pointer of the caller, if not needed set it to null
 \param[in]  msgFnc Log message callback function.
 \retval     none
*/
VVENC_DECL void vvenc_set_msg_callback( vvenc_config *cfg, void * msgCtx, vvencLoggingCallback msgFnc );

/* vvenc_init_config_parameter (optional)
  This method initialize the encoder parameter and sets all parameter the are not initialized yet.
  All not initialized parameters are set to valid values.
  Is automatically called in vvenc_encoder_open().
 \param[in]  vvenc_config* pointer to vvenc_config struct that contains encoder parameters
  \retval    bool if true an error occurred, otherwise false.
 \pre        vvenc_config_default() or vvenc_init_default() must be called first 
*/
VVENC_DECL bool vvenc_init_config_parameter( vvenc_config *cfg );


/* following defines should be used when using the string api interface function
 * venc_set_param (config, optionname, value)
*/

//       macro                         mapped option name     vvenc_config parameter name
#define VVENC_OPT_SIZE                 "size"                 // m_SourceWidthxm_SourceHeight e.g. 1920x1080
#define VVENC_OPT_WIDTH                "sourcewidth"          // m_SourceWidth  (in pixel)
#define VVENC_OPT_HEIGHT               "sourceheight"         // m_SourceHeight (in pixel)
#define VVENC_OPT_FRAMERATE            "framerate"            // m_FrameRate    (fps numerator in Hz)
#define VVENC_OPT_FRAMESCALE           "framescale"           // m_FrameScale   (fps denominator in Hz)
#define VVENC_OPT_FPS                  "fps"                  // Framerate as fraction (num/denom)
#define VVENC_OPT_TICKSPERSEC          "tickspersec"          // m_TicksPerSecond
#define VVENC_OPT_INPUTBITDEPTH        "inputbitdepth"        // m_inputBitDepth
#define VVENC_OPT_FRAMES               "framestobeencoded"    // m_framesToBeEncoded
#define VVENC_OPT_PRESET               "preset"               // set preset like "faster,fast,medium,slow,slower
#define VVENC_OPT_THREADS              "threads"              // m_numThreads
#define VVENC_OPT_BITRATE              "bitrate"              // m_RCTargetBitrate
#define VVENC_OPT_QP                   "qp"                   // m_QP
#define VVENC_OPT_TILES                "tiles"                // number of tile column x rows in m_numTileColsxm_numTileRows, e.g. 2x2
#define VVENC_OPT_VERBOSITY            "verbosity"            // m_verbosity

#define VVENC_OPT_PROFILE               "profile"             // m_profile
#define VVENC_OPT_LEVEL                 "level"               // m_level
#define VVENC_OPT_TIER                  "tier"                // m_levelTier

#define VVENC_OPT_REFRESHDSEC           "refreshsec"          // m_IntraPeriodSec
#define VVENC_OPT_INTRAPERIOD           "intraperiod"         // m_IntraPeriod
#define VVENC_OPT_DECODINGREFRESHTYPE   "decodingrefreshtype" // m_DecodingRefreshType
#define VVENC_OPT_GOPSIZE               "gopsize"             // m_GOPSize

#define VVENC_OPT_QPA                   "qpa"                 // m_usePerceptQPA
#define VVENC_OPT_RCPASSES              "passes"              // m_RCNumPasses
#define VVENC_OPT_RCPASS                "pass"                // m_RCPass
#define VVENC_OPT_INTERNALBITDEPTH      "internalbitdepth"    // m_internalBitDepth
#define VVENC_OPT_HDR                   "hdr"                 // m_HdrMode
#define VVENC_OPT_SEGMENT               "segment"             // m_SegmentMode

/* vvenc_set_param (optional)
  This method sets one parameter by name.
  numerical range is not checked until vvenc_init_config_parameter()
  \param[in]  vvenc_config* pointer to vvenc_config struct that contains encoder parameters
  \param[in]  name option name as string
              use "help", "longhelp", "fullhelp" to print available options
  \param[in]  value option value as string
              value=NULL means "true" for boolean options, but is a BAD_VALUE for non-booleans.
  \retval    returns 0 on success, or returns one of the following values:
             VVENC_BAD_VALUE occurs only if it can't even parse the value,
             VVENC_PARAM_INFO occurs when a information should be printed (e.g. help, version)
  \pre       vvenc_config_default() or vvenc_init_default() must be called first 
*/
#define VVENC_PARAM_BAD_NAME  (-1)
#define VVENC_PARAM_BAD_VALUE (-2)
#define VVENC_PARAM_INFO      (1)
VVENC_DECL int vvenc_set_param(vvenc_config *cfg, const char *name, const char *value);

/* vvenc_set_param_list (optional)
  This method sets a list of parameters by name.
  arguments must be separated as would be used in command line. e.g.: --bitrate 500000 --framerate 50
  numerical range is not checked until vvenc_init_config_parameter()
  \param[in]  vvenc_config* pointer to vvenc_config struct that contains encoder parameters
  \param[in]  argc number or arguments in argv string list
  \param[in]  argv list of char* (argv[]); option name must be defined by prefix -- or -
  \retval     returns 0 on success, > 0 if an information was printed (help), -1 on failure.
  \pre        vvenc_config_default() or vvenc_init_default() must be called first 
*/
VVENC_DECL int vvenc_set_param_list(vvenc_config *c, int argc, char* argv[] );

/* vvenc_get_config_as_string (optional)
  This method returns the encoder configuration as string.
  \param[in]  vvenc_config* pointer to vvenc_config struct that contains encoder parameters
  \param[in]  eMsgLevel verbosity level
  \retval     const char* encoder configuration as string
  \pre        vvenc_config_default() or vvenc_init_default() must be called first 
*/
VVENC_DECL const char* vvenc_get_config_as_string( vvenc_config *cfg, vvencMsgLevel eMsgLevel );

#ifdef __cplusplus
}
#endif /*__cplusplus */

VVENC_NAMESPACE_END



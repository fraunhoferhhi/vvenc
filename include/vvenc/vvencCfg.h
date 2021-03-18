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
/** \file     vvencCfg.h
    \brief    encoder configuration class (header)
*/

#pragma once

#include "vvenc/vvencDecl.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "vvenc/vvencCfgExpert.h"

#include <string>

#define VVENC_NAMESPACE_BEGIN
#define VVENC_NAMESPACE_END

#ifdef __cplusplus
extern "C" {
#endif

VVENC_NAMESPACE_BEGIN

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
  VVENC_PROFILE_AUTO                         = 0,
  VVENC_MAIN_10                              = 1,
  VVENC_MAIN_10_STILL_PICTURE                = 2,
  VVENC_MAIN_10_444                          = 3,
  VVENC_MAIN_10_444_STILL_PICTURE            = 4,
  VVENC_MULTILAYER_MAIN_10                   = 5,
  VVENC_MULTILAYER_MAIN_10_STILL_PICTURE     = 6,
  VVENC_MULTILAYER_MAIN_10_444               = 7,
  VVENC_MULTILAYER_MAIN_10_444_STILL_PICTURE = 8,
  VVENC_NUMBER_OF_PROFILES
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
  VVENC_DRT_RECOVERY_POINT_SEI = 3
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

typedef struct VVEncCfg
{
  VVEncCfgExpert      e;
  bool                m_confirmFailed;           ///< state variable

  vvencMsgLevel       m_verbosity;                     ///< encoder verbosity
  int                 m_framesToBeEncoded;       ///< number of encoded frames

  int                 m_FrameRate;               ///< source frame-rates (Hz)
  int                 m_FrameSkip;               ///< number of skipped frames from the beginning
  int                 m_SourceWidth;             ///< source width in pixel
  int                 m_SourceHeight;            ///< source height in pixel (when interlaced = field height)
  int                 m_TicksPerSecond;          ///< ticks per second e.g. 90000 for dts generation (1..27000000)

  vvencProfile        m_profile;
  vvencTier           m_levelTier;
  vvencLevel          m_level;

  int                 m_IntraPeriod;             ///< period of I-slice (random access period)
  int                 m_IntraPeriodSec;          ///< period of I-slice in seconds (random access period)
  vvencDecodingRefreshType m_DecodingRefreshType; ///< random access type
  int                 m_GOPSize;                 ///< GOP size of hierarchical structure

  int                 m_QP;                      ///< QP value of key-picture (integer)
  bool                m_usePerceptQPA;           ///< Mode of perceptually motivated input-adaptive QP modification, abbrev. perceptual QP adaptation (QPA).

  int                 m_RCTargetBitrate;
  int                 m_RCNumPasses;

  vvencSegmentMode    m_SegmentMode;

  int                 m_numThreads;              ///< number of worker threads

  int                 m_inputBitDepth   [ 2 ];   ///< bit-depth of input file
  int                 m_internalBitDepth[ 2 ];   ///< bit-depth codec operates at (input/output files will be converted)

  vvencHDRMode        m_HdrMode;
}VVEncCfg;

VVENC_DECL void vvenc_cfg_default(VVEncCfg *cfg );

VVENC_DECL int vvenc_initPreset( VVEncCfg *cfg, vvencPresetMode preset );

VVENC_DECL int vvenc_initDefault( VVEncCfg *cfg, int width, int height, int framerate, int targetbitrate = 0, int qp = 32, vvencPresetMode preset = vvencPresetMode::VVENC_MEDIUM );


/**
  This method initializes the configuration depending on set default parameter
  \retval     bool true: error, false: ok
  \pre        none.
*/
VVENC_DECL bool vvenc_initCfgParameter( VVEncCfg *cfg );

VVENC_DECL std::string vvenc_getConfigAsString( VVEncCfg *cfg, vvencMsgLevel eMsgLevel );

#ifdef __cplusplus
}
#endif /*__cplusplus */

VVENC_NAMESPACE_END



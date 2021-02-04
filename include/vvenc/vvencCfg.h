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
/** \file     vvencCfg.h
    \brief    encoder configuration class (header)
*/

#pragma once

#include <cstring>
#include <vector>
#include <string>
#include <sstream>
#include "vvenc/vvencDecl.h"
#include "vvenc/vvencCfgExpert.h"

//! \ingroup Interface
//! \{

namespace vvenc {


/**
  \ingroup VVEncExternalInterfaces
  \enum MsgLevel
  The enum MsgLevel enumerates supported log levels/verbosity.
*/
enum MsgLevel
{
  SILENT  = 0,
  ERROR   = 1,
  WARNING = 2,
  INFO    = 3,
  NOTICE  = 4,
  VERBOSE = 5,
  DETAILS = 6
};


enum PresetMode
{
 NONE      = -1,
 FASTER    = 0,
 FAST      = 1,
 MEDIUM    = 2,
 SLOW      = 3,
 SLOWER    = 4,
 FIRSTPASS = 254,
 TOOLTEST  = 255,
};

/**
  \ingroup VVEnc
  The class SliceType enumerates several supported slice types.
*/
enum SliceType
{
  B_SLICE               = 0,
  P_SLICE               = 1,
  I_SLICE               = 2,
  NUMBER_OF_SLICE_TYPES = 3
};

/**
  \ingroup VVEncExternalInterfaces
  \enum Profile
  The enum Profile enumerates supported profiles
*/
enum Profile
{
  PROFILE_AUTO                         = 0,
  MAIN_10                              = 1,
  MAIN_10_STILL_PICTURE                = 2,
  MAIN_10_444                          = 3,
  MAIN_10_444_STILL_PICTURE            = 4,
  MULTILAYER_MAIN_10                   = 5,
  MULTILAYER_MAIN_10_STILL_PICTURE     = 6,
  MULTILAYER_MAIN_10_444               = 7,
  MULTILAYER_MAIN_10_444_STILL_PICTURE = 8,
  NUMBER_OF_PROFILES
};


/**
  \ingroup VVEncExternalInterfaces
  \enum Tier
  The enum Tier enumerates supported tier
*/
enum Tier
{
  TIER_MAIN = 0,
  TIER_HIGH = 1,
  NUMBER_OF_TIERS
};

/**
  \ingroup VVEncExternalInterfaces
  \enum Name
  The enum Name enumerates supported level names
*/
enum Level
{
  LEVEL_AUTO = 0,
  LEVEL1   = 16,
  LEVEL2   = 32,
  LEVEL2_1 = 35,
  LEVEL3   = 48,
  LEVEL3_1 = 51,
  LEVEL4   = 64,
  LEVEL4_1 = 67,
  LEVEL5   = 80,
  LEVEL5_1 = 83,
  LEVEL5_2 = 86,
  LEVEL6   = 96,
  LEVEL6_1 = 99,
  LEVEL6_2 = 102,
  LEVEL6_3 = 105,
  LEVEL15_5 = 255,
  NUMBER_OF_LEVELS
};


/// supported IDR types
enum DecodingRefreshType
{
  DRT_NONE               = 0,
  DRT_CRA                = 1,
  DRT_IDR                = 2,
  DRT_RECOVERY_POINT_SEI = 3
};

enum SegmentMode
{
  SEG_OFF,
  SEG_FIRST,
  SEG_MID,
  SEG_LAST
};


enum HDRMode
{
  HDR_OFF      = 0,
  HDR_HDR10_PQ,
  HDR_HLG
};

enum NalUnitType
{
  NAL_UNIT_CODED_SLICE_TRAIL = 0,   // 0
  NAL_UNIT_CODED_SLICE_STSA,        // 1
  NAL_UNIT_CODED_SLICE_RADL,        // 2
  NAL_UNIT_CODED_SLICE_RASL,        // 3

  NAL_UNIT_RESERVED_VCL_4,
  NAL_UNIT_RESERVED_VCL_5,
  NAL_UNIT_RESERVED_VCL_6,

  NAL_UNIT_CODED_SLICE_IDR_W_RADL,  // 7
  NAL_UNIT_CODED_SLICE_IDR_N_LP,    // 8
  NAL_UNIT_CODED_SLICE_CRA,         // 9
  NAL_UNIT_CODED_SLICE_GDR,         // 10

  NAL_UNIT_RESERVED_IRAP_VCL_11,
  NAL_UNIT_RESERVED_IRAP_VCL_12,
  NAL_UNIT_DCI,                     // 13
  NAL_UNIT_VPS,                     // 14
  NAL_UNIT_SPS,                     // 15
  NAL_UNIT_PPS,                     // 16
  NAL_UNIT_PREFIX_APS,              // 17
  NAL_UNIT_SUFFIX_APS,              // 18
  NAL_UNIT_PH,                      // 19
  NAL_UNIT_ACCESS_UNIT_DELIMITER,   // 20
  NAL_UNIT_EOS,                     // 21
  NAL_UNIT_EOB,                     // 22
  NAL_UNIT_PREFIX_SEI,              // 23
  NAL_UNIT_SUFFIX_SEI,              // 24
  NAL_UNIT_FD,                      // 25

  NAL_UNIT_RESERVED_NVCL_26,
  NAL_UNIT_RESERVED_NVCL_27,

  NAL_UNIT_UNSPECIFIED_28,
  NAL_UNIT_UNSPECIFIED_29,
  NAL_UNIT_UNSPECIFIED_30,
  NAL_UNIT_UNSPECIFIED_31,
  NAL_UNIT_INVALID
};

class VVENC_DECL VVEncCfg : public VVEncCfgExpert
{
public:
  bool                m_confirmFailed                  = false;         ///< state variable

  MsgLevel            m_verbosity                      = VERBOSE;       ///< encoder verbosity
  int                 m_framesToBeEncoded              = 0;             ///< number of encoded frames

  int                 m_FrameRate                      = 0;             ///< source frame-rates (Hz)
  int                 m_FrameSkip                      = 0;             ///< number of skipped frames from the beginning
  int                 m_SourceWidth                    = 0;             ///< source width in pixel
  int                 m_SourceHeight                   = 0;             ///< source height in pixel (when interlaced = field height)
  int                 m_TicksPerSecond                 = 90000;         ///< ticks per second e.g. 90000 for dts generation (1..27000000)
  bool                m_AccessUnitDelimiter            = false;         ///< add Access Unit Delimiter NAL units

  Profile             m_profile                        = Profile::PROFILE_AUTO;
  Tier                m_levelTier                      = Tier::TIER_MAIN ;
  Level               m_level                          = Level::LEVEL_AUTO;

  int                 m_IntraPeriod                    = 0;             ///< period of I-slice (random access period)
  int                 m_IntraPeriodSec                 = 1;             ///< period of I-slice in seconds (random access period)
  DecodingRefreshType m_DecodingRefreshType            = DRT_CRA;       ///< random access type
  int                 m_GOPSize                        = 32;            ///< GOP size of hierarchical structure

  int                 m_QP                             = 32;            ///< QP value of key-picture (integer)
  unsigned            m_usePerceptQPA                  = 0;             ///< Mode of perceptually motivated input-adaptive QP modification, abbrev. perceptual QP adaptation (QPA). (0 = off, 1 = on for SDR, 2 = on for HDR)

  int                 m_RCTargetBitrate                = 0;
  int                 m_RCNumPasses                    = 1;

  SegmentMode         m_SegmentMode                    = SEG_OFF;

  int                 m_numThreads                     = 0;             ///< number of worker threads

  int                 m_inputBitDepth   [ MAX_NUM_CH ] = { 8, 0};       ///< bit-depth of input file
  int                 m_internalBitDepth[ MAX_NUM_CH ] = { 10, 0};      ///< bit-depth codec operates at (input/output files will be converted)

  HDRMode             m_HdrMode                        = HDR_OFF;
public:

  VVEncCfg()
  {
    initPreset( PresetMode::MEDIUM );
  }

  virtual ~VVEncCfg()
  {
  }

  /**
    This method initializes the configuration depending on set default parameter
    \retval     bool true: error, false: ok
    \pre        none.
  */
  bool initCfgParameter();

  int initDefault( int width, int height, int framerate, int targetbitrate = 0, int qp = 32, PresetMode preset = PresetMode::MEDIUM );
  int initPreset( PresetMode preset );

  virtual std::string getConfigAsString( MsgLevel eMsgLevel ) const;

private:
  bool checkExperimental( bool bflag, const char* message );
  bool confirmParameter ( bool bflag, const char* message );

  /**
    This method checks if the current configuration is valid.
    The method checks all configuration parameter (base and derived/dependent)
    \param[in]  none
    \retval     bool true: error, false: ok
    \pre        The initCfgParameter must be called first.
  */
  bool checkCfgParameter( );
};


} // namespace vvenc

//! \}


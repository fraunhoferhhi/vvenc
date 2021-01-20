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
/** \file     VVEncAppCfg.h
    \brief    Handle encoder configuration parameters (header)
*/

#pragma once
#include "apputils/apputilsDecl.h"

#include <vector>
#include <string>

#include "vvenc/vvencCfg.h"

#include "apputils/IStreamIO.h"


using namespace vvenc;


namespace apputils {

//! \ingroup apputils
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================



// ====================================================================================================================
// string <-> enum fixed mappings
// ====================================================================================================================

const std::vector<SVPair<MsgLevel>> MsgLevelToEnumMap =
{
  { "silent",  MsgLevel::SILENT  },
  { "error",   MsgLevel::ERROR   },
  { "warning", MsgLevel::WARNING },
  { "info",    MsgLevel::INFO    },
  { "notice",  MsgLevel::NOTICE  },
  { "verbose", MsgLevel::VERBOSE },
  { "details", MsgLevel::DETAILS },
  { "0",       MsgLevel::SILENT  },
  { "1",       MsgLevel::ERROR   },
  { "2",       MsgLevel::WARNING },
  { "3",       MsgLevel::INFO    },
  { "4",       MsgLevel::NOTICE  },
  { "5",       MsgLevel::VERBOSE },
  { "6",       MsgLevel::DETAILS },
};

const std::vector<SVPair<PresetMode>> PresetToEnumMap =
{
  { "none",      PresetMode::NONE },
  { "faster",    PresetMode::FASTER },
  { "fast",      PresetMode::FAST },
  { "medium",    PresetMode::MEDIUM },
  { "slow",      PresetMode::SLOW },
  { "slower",    PresetMode::SLOWER },
  { "firstpass", PresetMode::FIRSTPASS },
  { "tooltest",  PresetMode::TOOLTEST },
};

const std::vector<SVPair<SegmentMode>> SegmentToEnumMap =
{
  { "off",      SegmentMode::SEG_OFF },
  { "first",    SegmentMode::SEG_FIRST },
  { "mid",      SegmentMode::SEG_MID },
  { "last",     SegmentMode::SEG_LAST },
};


const std::vector<SVPair<Profile>> ProfileToEnumMap =
{
  { "none",                                  Profile::PROFILE_NONE },
  { "main_10",                               Profile::MAIN_10 },
  { "main_10_444",                           Profile::MAIN_10_444 },
  { "main_10_still_picture",                 Profile::MAIN_10_STILL_PICTURE },
  { "main_10_444_still_picture",             Profile::MAIN_10_444_STILL_PICTURE },
  { "multilayer_main_10",                    Profile::MULTILAYER_MAIN_10 },
  { "multilayer_main_10_444",                Profile::MULTILAYER_MAIN_10_444 },
  { "multilayer_main_10_still_picture",      Profile::MULTILAYER_MAIN_10_STILL_PICTURE },
  { "multilayer_main_10_444_still_picture",  Profile::MULTILAYER_MAIN_10_444_STILL_PICTURE },
  { "auto",                                  Profile::PROFILE_AUTO }
};

const std::vector<SVPair<Level>> LevelToEnumMap =
{
  { "none",                    Level::LEVEL_NONE},
  { "1",                       Level::LEVEL1   },
  { "1.0",                     Level::LEVEL1   },
  { "2",                       Level::LEVEL2   },
  { "2.0",                     Level::LEVEL2   },
  { "2.1",                     Level::LEVEL2_1 },
  { "3",                       Level::LEVEL3   },
  { "3.0",                     Level::LEVEL3   },
  { "3.1",                     Level::LEVEL3_1 },
  { "4",                       Level::LEVEL4   },
  { "4.1",                     Level::LEVEL4_1 },
  { "5",                       Level::LEVEL5   },
  { "5.0",                     Level::LEVEL5   },
  { "5.1",                     Level::LEVEL5_1 },
  { "5.2",                     Level::LEVEL5_2 },
  { "6",                       Level::LEVEL6   },
  { "6.0",                     Level::LEVEL6   },
  { "6.1",                     Level::LEVEL6_1 },
  { "6.2",                     Level::LEVEL6_2 },
  { "6.3",                     Level::LEVEL6_3 },
  { "15.5",                    Level::LEVEL15_5 },
};

const std::vector<SVPair<Tier>> TierToEnumMap =
{
  { "main",                    Tier::TIER_MAIN },
  { "high",                    Tier::TIER_HIGH },
};

const std::vector<SVPair<CostMode>> CostModeToEnumMap =
{
  { "lossy",                   COST_STANDARD_LOSSY              },
  { "sequence_level_lossless", COST_SEQUENCE_LEVEL_LOSSLESS     },
  { "lossless",                COST_LOSSLESS_CODING             },
  { "mixed_lossless_lossy",    COST_MIXED_LOSSLESS_LOSSY_CODING }
};

const std::vector<SVPair<ChromaFormat>> ChromaFormatToEnumMap =
{
  { "400",                     CHROMA_400 },
  { "420",                     CHROMA_420 },
  { "422",                     CHROMA_422 },
  { "444",                     CHROMA_444 },
  { "0",                       NUM_CHROMA_FORMAT }
};

const std::vector<SVPair<HashType>> HashTypeToEnumMap =
{
  { "md5",                     HASHTYPE_MD5      },
  { "crc",                     HASHTYPE_CRC      },
  { "checksum",                HASHTYPE_CHECKSUM },
  { "off",                     HASHTYPE_NONE     },
  // for backward compatibility support values as well
  { "1",                       HASHTYPE_MD5      },
  { "2",                       HASHTYPE_CRC      },
  { "3",                       HASHTYPE_CHECKSUM },
  { "0",                       HASHTYPE_NONE     }
};

const std::vector<SVPair<DecodingRefreshType>> DecodingRefreshTypeToEnumMap =
{
  { "none",                  DRT_NONE },
  { "cra",                   DRT_CRA },
  { "idr",                   DRT_IDR },
  { "rpsei",                 DRT_RECOVERY_POINT_SEI },
  { "0",                     DRT_NONE },
  { "1",                     DRT_CRA },
  { "2",                     DRT_IDR },
  { "3",                     DRT_RECOVERY_POINT_SEI },
};

const std::vector<SVPair<RateControlMode>> RateControlModeToEnumMap =
{
  { "0",                     RCM_OFF },
  { "1",                     RCM_CTU_LEVEL },
  { "2",                     RCM_PICTURE_LEVEL },
  { "3",                     RCM_GOP_LEVEL },
};

enum BitDepthAndColorSpace
{
  YUV420_8,
  YUV420_10,
  YUV422_8,
  YUV422_10,
  YUV444_8,
  YUV444_10,
  YUV400_8,
  YUV400_10,
};

const std::vector<SVPair<BitDepthAndColorSpace>> InputBitColorSpaceToIntMap =
{
  { "yuv420",                    YUV420_8 },
  { "yuv420_10",                 YUV420_10 },
};

/// encoder configuration class
class APPUTILS_DECL VVEncAppCfg : public vvenc::VVEncCfg
{
public:
  std::string  m_inputFileName;                                ///< source file name
  std::string  m_bitstreamFileName;                            ///< output bitstream file
  std::string  m_reconFileName;                                ///< output reconstruction file
  vvenc::ChromaFormat m_inputFileChromaFormat  = vvenc::CHROMA_420;
  bool         m_bClipInputVideoToRec709Range  = false;
  bool         m_bClipOutputVideoToRec709Range = false;
  bool         m_packedYUVMode                 = false;        ///< If true, output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data
  bool         m_decode                        = false;

public:

  VVEncAppCfg()
  {
  }

  virtual ~VVEncAppCfg();

public:
  bool parseCfg( int argc, char* argv[] );                     ///< parse configuration fill member variables (simple app)
  bool parseCfgFF( int argc, char* argv[] );                   ///< parse configuration fill member variables for FullFeature set (expert app)

  virtual std::string getConfigAsString( vvenc::MsgLevel eMsgLevel ) const;

private:

  static void setPresets( VVEncCfg* cfg, int preset );
  static void setInputBitDepthAndColorSpace( VVEncCfg* cfg, int dbcs );
};

} // namespace

//! \}


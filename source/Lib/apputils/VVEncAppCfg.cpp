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


/** \file     VVEncAppCfg.cpp
    \brief    Handle encoder configuration parameters
*/

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <string>
#include <fstream>
#include <algorithm>
#include <cstdarg>

#include "apputils/IStreamIO.h"
#include "apputils/ParseArg.h"
#include "apputils/VVEncAppCfg.h"
#include "vvenc/vvenc.h"

#define MACRO_TO_STRING_HELPER(val) #val
#define MACRO_TO_STRING(val) MACRO_TO_STRING_HELPER(val)

using namespace std;
namespace po = apputils::df::program_options_lite;

namespace apputils {

//! \ingroup EncoderApp
//! \{
//!
//!

// ====================================================================================================================
// enums
// ====================================================================================================================
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
  YUV420_10_PACKED
};

// ====================================================================================================================
// string <-> enum fixed mappings
// ====================================================================================================================
const std::vector<SVPair<vvencMsgLevel>> MsgLevelToEnumMap =
{
  { "silent",  vvencMsgLevel::VVENC_SILENT  },
  { "error",   vvencMsgLevel::VVENC_ERROR   },
  { "warning", vvencMsgLevel::VVENC_WARNING },
  { "info",    vvencMsgLevel::VVENC_INFO    },
  { "notice",  vvencMsgLevel::VVENC_NOTICE  },
  { "verbose", vvencMsgLevel::VVENC_VERBOSE },
  { "details", vvencMsgLevel::VVENC_DETAILS },
  { "0",       vvencMsgLevel::VVENC_SILENT  },
  { "1",       vvencMsgLevel::VVENC_ERROR   },
  { "2",       vvencMsgLevel::VVENC_WARNING },
  { "3",       vvencMsgLevel::VVENC_INFO    },
  { "4",       vvencMsgLevel::VVENC_NOTICE  },
  { "5",       vvencMsgLevel::VVENC_VERBOSE },
  { "6",       vvencMsgLevel::VVENC_DETAILS },
};


const std::vector<SVPair<vvencPresetMode>> PresetToEnumMap =
{
  { "none",      vvencPresetMode::VVENC_NONE },
  { "faster",    vvencPresetMode::VVENC_FASTER },
  { "fast",      vvencPresetMode::VVENC_FAST },
  { "medium",    vvencPresetMode::VVENC_MEDIUM },
  { "slow",      vvencPresetMode::VVENC_SLOW },
  { "slower",    vvencPresetMode::VVENC_SLOWER },
  { "firstpass", vvencPresetMode::VVENC_FIRSTPASS },
  { "tooltest",  vvencPresetMode::VVENC_TOOLTEST },
};

const std::vector<SVPair<vvencSegmentMode>> SegmentToEnumMap =
{
  { "off",      vvencSegmentMode::VVENC_SEG_OFF },
  { "first",    vvencSegmentMode::VVENC_SEG_FIRST },
  { "mid",      vvencSegmentMode::VVENC_SEG_MID },
  { "last",     vvencSegmentMode::VVENC_SEG_LAST },
};


const std::vector<SVPair<vvencProfile>> ProfileToEnumMap =
{
  { "main_10",                               vvencProfile::VVENC_MAIN_10 },
  { "main_10_444",                           vvencProfile::VVENC_MAIN_10_444 },
  { "main_10_still_picture",                 vvencProfile::VVENC_MAIN_10_STILL_PICTURE },
  { "main_10_444_still_picture",             vvencProfile::VVENC_MAIN_10_444_STILL_PICTURE },
  { "multilayer_main_10",                    vvencProfile::VVENC_MULTILAYER_MAIN_10 },
  { "multilayer_main_10_444",                vvencProfile::VVENC_MULTILAYER_MAIN_10_444 },
  { "multilayer_main_10_still_picture",      vvencProfile::VVENC_MULTILAYER_MAIN_10_STILL_PICTURE },
  { "multilayer_main_10_444_still_picture",  vvencProfile::VVENC_MULTILAYER_MAIN_10_444_STILL_PICTURE },
  { "auto",                                  vvencProfile::VVENC_PROFILE_AUTO },
};

const std::vector<SVPair<vvencLevel>> LevelToEnumMap =
{
  { "auto",                    VVENC_LEVEL_AUTO},
  { "1",                       VVENC_LEVEL1    },
  { "1.0",                     VVENC_LEVEL1    },
  { "2",                       VVENC_LEVEL2    },
  { "2.0",                     VVENC_LEVEL2    },
  { "2.1",                     VVENC_LEVEL2_1  },
  { "3",                       VVENC_LEVEL3    },
  { "3.0",                     VVENC_LEVEL3    },
  { "3.1",                     VVENC_LEVEL3_1  },
  { "4",                       VVENC_LEVEL4    },
  { "4.0",                     VVENC_LEVEL4    },
  { "4.1",                     VVENC_LEVEL4_1  },
  { "5",                       VVENC_LEVEL5    },
  { "5.0",                     VVENC_LEVEL5    },
  { "5.1",                     VVENC_LEVEL5_1  },
  { "5.2",                     VVENC_LEVEL5_2  },
  { "6",                       VVENC_LEVEL6    },
  { "6.0",                     VVENC_LEVEL6    },
  { "6.1",                     VVENC_LEVEL6_1  },
  { "6.2",                     VVENC_LEVEL6_2  },
  { "6.3",                     VVENC_LEVEL6_3  },
  { "15.5",                    VVENC_LEVEL15_5 },
};

const std::vector<SVPair<vvencTier>> TierToEnumMap =
{
  { "main",                    vvencTier::VVENC_TIER_MAIN },
  { "high",                    vvencTier::VVENC_TIER_HIGH },
};

const std::vector<SVPair<vvencCostMode>> CostModeToEnumMap =
{
  { "lossy",                   VVENC_COST_STANDARD_LOSSY              },
  { "sequence_level_lossless", VVENC_COST_SEQUENCE_LEVEL_LOSSLESS     },
  { "lossless",                VVENC_COST_LOSSLESS_CODING             },
  { "mixed_lossless_lossy",    VVENC_COST_MIXED_LOSSLESS_LOSSY_CODING }
};

const std::vector<SVPair<vvencChromaFormat>> ChromaFormatToEnumMap =
{
  { "400",                     VVENC_CHROMA_400 },
  { "420",                     VVENC_CHROMA_420 },
  { "422",                     VVENC_CHROMA_422 },
  { "444",                     VVENC_CHROMA_444 },
  { "0",                       VVENC_NUM_CHROMA_FORMAT }
};

const std::vector<SVPair<vvencHashType>> HashTypeToEnumMap =
{
  { "md5",                     VVENC_HASHTYPE_MD5      },
  { "crc",                     VVENC_HASHTYPE_CRC      },
  { "checksum",                VVENC_HASHTYPE_CHECKSUM },
  { "off",                     VVENC_HASHTYPE_NONE     },
  // for backward compatibility support values as well
  { "1",                       VVENC_HASHTYPE_MD5      },
  { "2",                       VVENC_HASHTYPE_CRC      },
  { "3",                       VVENC_HASHTYPE_CHECKSUM },
  { "0",                       VVENC_HASHTYPE_NONE     }
};

const std::vector<SVPair<vvencDecodingRefreshType>> DecodingRefreshTypeToEnumMap =
{
  { "none",                  VVENC_DRT_NONE },
  { "cra",                   VVENC_DRT_CRA },
  { "idr",                   VVENC_DRT_IDR },
  { "rpsei",                 VVENC_DRT_RECOVERY_POINT_SEI },
  { "idr2",                  VVENC_DRT_IDR2 },
  { "cra_cre",               VVENC_DRT_CRA_CRE },
  { "0",                     VVENC_DRT_NONE },
  { "1",                     VVENC_DRT_CRA },
  { "2",                     VVENC_DRT_IDR },
  { "3",                     VVENC_DRT_RECOVERY_POINT_SEI },
  { "4",                     VVENC_DRT_IDR2 },
  { "5",                     VVENC_DRT_CRA_CRE },
};

const std::vector<SVPair<BitDepthAndColorSpace>> BitColorSpaceToIntMap =
{
  { "yuv420",                    YUV420_8 },
  { "yuv420_10",                 YUV420_10 },
  { "yuv420_10_packed",          YUV420_10_PACKED },
};


const std::vector<SVPair<vvencHDRMode>> HdrModeToIntMap =
{
  { "off",                 VVENC_HDR_OFF },
  { "pq",                  VVENC_HDR_PQ},
  { "hdr10",               VVENC_HDR_PQ},
  { "pq_2020",             VVENC_HDR_PQ_BT2020},
  { "hdr10_2020",          VVENC_HDR_PQ_BT2020},
  { "hlg",                 VVENC_HDR_HLG},
  { "hlg_2020",            VVENC_HDR_HLG_BT2020},
};


const std::vector<SVPair<int>> ColorPrimariesToIntMap =
{
  { "reserved",            0 },
  { "bt709",               1 },
  { "unknown",             2 },
  { "empty",               3 },
  { "bt470m",              4 },
  { "bt470bg",             5 },
  { "smpte170m",           6 },
  { "smpte240m",           7 },
  { "film",                8 },
  { "bt2020",              9 },
  { "smpte428",           10 },
  { "smpte431",           11 },
  { "smpte432",           12 },
  { "0", 0 }, { "1", 1 }, { "2", 2 }, { "3", 3 }, { "4", 4 }, { "5", 5 },
  { "6", 6 }, { "7", 7 }, { "8", 8 }, { "9", 9 }, { "10",10 }, { "11",11 }, { "12",12 }
};

const std::vector<SVPair<int>> TransferCharacteristicsToIntMap =
{
  { "auto",               -1 },
  { "reserved",            0 },
  { "bt709",               1 },
  { "unknown",             2 },
  { "empty",               3 },
  { "bt470m",              4 },
  { "bt470bg",             5 },
  { "smpte170m",           6 },
  { "smpte240m",           7 },
  { "linear",              8 },
  { "log100",              9 },
  { "log316",             10 },
  { "iec61966",           11 },
  { "bt1361e",            12 },
  { "iec61966-2-1",       13 },
  { "bt2020-10",          14 },
  { "bt2020-12",          15 },
  { "smpte2084",          16 },
  { "smpte428",           17 },
  { "arib-std-b67",       18 },
  { "0", 0 }, { "1", 1 }, { "2", 2 }, { "3", 3 }, { "4", 4 }, { "5", 5 },
  { "6", 6 }, { "7", 7 }, { "8", 8 }, { "9", 9 }, { "10",10 }, { "11",11 },
  { "12",12 },{ "13",13 },{ "14",14 },{ "15",15 },{ "16",16 },{ "17",17 },{ "18",18 }
};

const std::vector<SVPair<int>> ColorMatrixToIntMap =
{
  { "gbr",              0 },
  { "bt709",            1 },
  { "unknown",          2 },
  { "empty",            3 },
  { "fcc",              4 },
  { "bt470bg",          5 },
  { "smpte170m",        6 },
  { "smpte240m",        7 },
  { "ycgco",            8 },
  { "bt2020nc",         9 },
  { "bt2020c",          10 },
  { "smpte2085",        11 },
  { "chroma-derived-nc",12 },
  { "chroma-derived-c", 13 },
  { "ictcp",            14 },
  { "0", 0 }, { "1", 1 }, { "2", 2 }, { "3", 3 }, { "4", 4 }, { "5", 5 },
  { "6", 6 }, { "7", 7 }, { "8", 8 }, { "9", 9 }, { "10",10 }, { "11",11 },
  { "12",12 },{ "13",13 },{ "14",14 }
};


const std::vector<SVPair<int>> FlagToIntMap =
{
  { "auto",        -1 },
  { "-1",          -1 },

  { "off",          0 },
  { "disable",      0 },
  { "0",            0 },

  { "on",           1 },
  { "enable",       1 },
  { "1",            1 },
};

// this is only needed for backward compatibility and will be removed in the next release
const std::vector<SVPair<bool>> QPAToIntMap =
{
  { "off",          0 },
  { "disable",      0 },
  { "0",            0 },

  { "on",           1 },
  { "enable",       1 },
  { "1",            1 },
  { "2",            1 }, // map deprecated modes 2-5 to qpa enabled
  { "3",            1 },
  { "4",            1 },
  { "5",            1 },
};

//// ====================================================================================================================
//// string <-> enum
//// ====================================================================================================================


void setPresets( VVEncAppCfg* appcfg, vvenc_config* cfg,  int preset )
{
  VVEncAppCfg::presetChangeCallback callback = appcfg->getPresetChangeCallback();
  if( callback )
  {
    callback( cfg, (vvencPresetMode)preset );
  }
}

void setInputBitDepthAndColorSpace( VVEncAppCfg* appcfg, vvenc_config* cfg, int dbcs )
{
  switch( dbcs )
  {
  case YUV420_8 :         appcfg->m_inputFileChromaFormat = VVENC_CHROMA_420; cfg->m_inputBitDepth[0] = 8;  break;
  case YUV420_10 :        appcfg->m_inputFileChromaFormat = VVENC_CHROMA_420; cfg->m_inputBitDepth[0] = 10; break;
  case YUV420_10_PACKED : appcfg->m_inputFileChromaFormat = VVENC_CHROMA_420; cfg->m_inputBitDepth[0] = 10; appcfg->m_packedYUVInput=true; break;
  case YUV422_8 :         appcfg->m_inputFileChromaFormat = VVENC_CHROMA_422; cfg->m_inputBitDepth[0] = 8;  break;
  case YUV422_10 :        appcfg->m_inputFileChromaFormat = VVENC_CHROMA_422; cfg->m_inputBitDepth[0] = 10; break;
  case YUV444_8 :         appcfg->m_inputFileChromaFormat = VVENC_CHROMA_444; cfg->m_inputBitDepth[0] = 8;  break;
  case YUV444_10 :        appcfg->m_inputFileChromaFormat = VVENC_CHROMA_444; cfg->m_inputBitDepth[0] = 10; break;
  case YUV400_8 :         appcfg->m_inputFileChromaFormat = VVENC_CHROMA_400; cfg->m_inputBitDepth[0] = 8;  break;
  case YUV400_10 :        appcfg->m_inputFileChromaFormat = VVENC_CHROMA_400; cfg->m_inputBitDepth[0] = 10; break;
  default: break;
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

int VVEncAppCfg::parse( int argc, char* argv[], vvenc_config* c, std::ostream& rcOstr )
{
  int ret = 0;

  //
  // setup configuration parameters
  //
  bool do_help                = false;
  bool do_full_help           = false;

  int  warnUnknowParameter    = 0;
  std::string ignoreParams;

  std::string writeCfg = "";

  IStreamToEnum<vvencMsgLevel>      toMsgLevel                   ( &c->m_verbosity,   &MsgLevelToEnumMap );
  IStreamToFunc<vvencPresetMode>    toPreset                     ( setPresets, this, c, &PresetToEnumMap,vvencPresetMode::VVENC_MEDIUM);
  IStreamToRefVec<int>              toSourceSize                 ( { &c->m_SourceWidth, &c->m_SourceHeight }, true, 'x' );
  IStreamToRefVec<int>              toFps                        ( { &c->m_FrameRate, &c->m_FrameScale }, false, '/' );

  IStreamToEnum<vvencProfile>       toProfile                    ( &c->m_profile,                     &ProfileToEnumMap      );
  IStreamToEnum<vvencTier>          toLevelTier                  ( &c->m_levelTier,                   &TierToEnumMap         );
  IStreamToEnum<vvencLevel>         toLevel                      ( &c->m_level,                       &LevelToEnumMap        );
  IStreamToEnum<vvencSegmentMode>   toSegment                    ( &c->m_SegmentMode,                 &SegmentToEnumMap      );
  IStreamToEnum<vvencHDRMode>       toHDRMode                    ( &c->m_HdrMode,                     &HdrModeToIntMap       );
  IStreamToRefVec<uint32_t>         toNumTiles                   ( { &c->m_numTileCols, &c->m_numTileRows }, true, 'x'       );

  IStreamToFunc<BitDepthAndColorSpace>    toInputFormatBitdepth  ( setInputBitDepthAndColorSpace, this, c, &BitColorSpaceToIntMap, YUV420_8);
  IStreamToEnum<vvencDecodingRefreshType> toDecRefreshType       ( &c->m_DecodingRefreshType,         &DecodingRefreshTypeToEnumMap );

  IStreamToEnum<int>                toAud                        ( &c->m_AccessUnitDelimiter,         &FlagToIntMap );
  IStreamToEnum<int>                toVui                        ( &c->m_vuiParametersPresent,        &FlagToIntMap );
  IStreamToEnum<bool>               toQPA                        ( &c->m_usePerceptQPA,               &QPAToIntMap );

  IStreamToRefVec<double>           toLambdaModifier             ( { &c->m_adLambdaModifier[0], &c->m_adLambdaModifier[1], &c->m_adLambdaModifier[2], &c->m_adLambdaModifier[3], &c->m_adLambdaModifier[4], &c->m_adLambdaModifier[5], &c->m_adLambdaModifier[6] }, false );
  IStreamToEnum<vvencCostMode>      toCostMode                   ( &c->m_costMode,                    &CostModeToEnumMap     );
  IStreamToEnum<vvencChromaFormat>  toInputFileChromaFormat      ( &m_inputFileChromaFormat,       &ChromaFormatToEnumMap  );
  IStreamToEnum<vvencChromaFormat>  toInternChromaFormat         ( &c->m_internChromaFormat,          &ChromaFormatToEnumMap  );
  IStreamToEnum<vvencHashType>      toHashType                   ( &c->m_decodedPictureHashSEIType,   &HashTypeToEnumMap     );
  IStreamToArr<int>                 toQpInCb                     ( &c->m_qpInValsCb[0], VVENC_MAX_QP_VALS_CHROMA            );
  IStreamToArr<int>                 toQpOutCb                    ( &c->m_qpOutValsCb[0], VVENC_MAX_QP_VALS_CHROMA           );
  IStreamToArr<int>                 toQpInCr                     ( &c->m_qpInValsCr[0], VVENC_MAX_QP_VALS_CHROMA            );
  IStreamToArr<int>                 toQpOutCr                    ( &c->m_qpOutValsCr[0], VVENC_MAX_QP_VALS_CHROMA           );
  IStreamToArr<int>                 toQpInCbCr                   ( &c->m_qpInValsCbCr[0], VVENC_MAX_QP_VALS_CHROMA          );
  IStreamToArr<int>                 toQpOutCbCr                  ( &c->m_qpOutValsCbCr[0], VVENC_MAX_QP_VALS_CHROMA         );
  IStreamToArr<double>              toIntraLambdaModifier        ( &c->m_adIntraLambdaModifier[0], VVENC_MAX_TLAYER );

  IStreamToArr<unsigned int>        toTileColumnWidth            ( &c->m_tileColumnWidth[0], 10 );
  IStreamToArr<unsigned int>        toTileRowHeight              ( &c->m_tileRowHeight[0], 10 );

  IStreamToArr<int>                 toMCTFFrames                 ( &c->m_vvencMCTF.MCTFFrames[0], VVENC_MAX_MCTF_FRAMES   );
  IStreamToArr<double>              toMCTFStrengths              ( &c->m_vvencMCTF.MCTFStrengths[0], VVENC_MAX_MCTF_FRAMES);
  IStreamToEnum<int>                toColorPrimaries             ( &c->m_colourPrimaries,        &ColorPrimariesToIntMap );
  IStreamToEnum<int>                toTransferCharacteristics    ( &c->m_transferCharacteristics,&TransferCharacteristicsToIntMap );
  IStreamToEnum<int>                toColorMatrix                ( &c->m_matrixCoefficients,     &ColorMatrixToIntMap );
  IStreamToEnum<int>                toPrefTransferCharacteristics( &c->m_preferredTransferCharacteristics, &TransferCharacteristicsToIntMap );

  IStreamToArr<unsigned int>        toMasteringDisplay            ( &c->m_masteringDisplay[0], 10  );
  IStreamToArr<unsigned int>        toContentLightLevel           ( &c->m_contentLightLevel[0], 2 );

  IStreamToArr<char>                toTraceRule                   ( &c->m_traceRule[0], VVENC_MAX_STRING_LEN  );
  IStreamToArr<char>                toTraceFile                   ( &c->m_traceFile[0], VVENC_MAX_STRING_LEN  );
  IStreamToArr<char>                toDecodeBitstreams0           ( &c->m_decodeBitstreams[0][0], VVENC_MAX_STRING_LEN  );
  IStreamToArr<char>                toDecodeBitstreams1           ( &c->m_decodeBitstreams[1][0], VVENC_MAX_STRING_LEN  );
  IStreamToArr<char>                toSummaryOutFilename          ( &c->m_summaryOutFilename[0], VVENC_MAX_STRING_LEN  );
  IStreamToArr<char>                toSummaryPicFilenameBase      ( &c->m_summaryPicFilenameBase[0], VVENC_MAX_STRING_LEN  );

  po::Options opts;
  if( m_easyMode )
  {
    opts.setLowerCaseOnly(); // easy app uses lower case option names only
  }

  opts.setSubSection("General Options");
  opts.addOptions()
  ("help",                                            do_help,                                             "show default help")
  ("fullhelp",                                        do_full_help,                                        "show full help")
  ("Verbosity,v",                                     toMsgLevel,                                          "Specifies the level of the verboseness (0: silent, 1: error, 2: warning, 3: info, 4: notice, 5: verbose, 6: debug)")
  ("version",                                         m_showVersion,                                       "show version ")
  ;

  opts.setSubSection("Input Options");
  if( m_easyMode)
  {
    opts.addOptions()
    ("input,i",                                         m_inputFileName,                                     "original YUV input file name or '-' for reading from stdin")
    ("size,s",                                          toSourceSize,                                        "specify input resolution (WidthxHeight)")
    ("format,c",                                        toInputFormatBitdepth,                               "set input format (yuv420, yuv420_10, yuv420_10_packed)")
    ("framerate,r",                                     c->m_FrameRate,                                      "temporal rate (framerate numerator) e.g. 25,30, 30000, 50,60, 60000 ")
    ("framescale",                                      c->m_FrameScale,                                     "temporal scale (framerate denominator) e.g. 1, 1001 ")
    ("fps",                                             toFps,                                               "Framerate as int or fraction (num/denom) ")

    ("tickspersec",                                     c->m_TicksPerSecond,                                 "Ticks Per Second for dts generation, (1..27000000)")
    ("frames,f",                                        c->m_framesToBeEncoded,                              "max. frames to encode [all]")
    ;
  }
  else
  {
    opts.addOptions()
    ("InputFile,i",                                     m_inputFileName,                                     "original YUV input file name or '-' for reading from stdin")
    ("Size,s",                                          toSourceSize,                                        "input resolution (WidthxHeight)")
    ("InputBitDepth",                                   c->m_inputBitDepth[ 0 ],                             "Bit-depth of input file")
    ("FramesToBeEncoded,f",                             c->m_framesToBeEncoded,                              "Number of frames to be encoded (default=all)")
    ("FrameRate,-fr",                                   c->m_FrameRate,                                      "Temporal rate (framerate numerator) e.g. 25,30, 30000, 50,60, 60000")
    ("FrameScale",                                      c->m_FrameScale,                                     "Temporal scale (framerate denominator) e.g. 1, 1001")
    ("fps",                                             toFps,                                               "Framerate as int or fraction (num/denom) ")
    ("TicksPerSecond",                                  c->m_TicksPerSecond,                                 "Ticks Per Second for dts generation, (1..27000000)")
    ;
  }

  opts.addOptions()
  ("FrameSkip,-fs",                                     m_FrameSkip,                                         "Number of frames to skip at start of input YUV [off]")
  ("segment",                                           toSegment,                                           "when encoding multiple separate segments, specify segment position to enable segment concatenation (first, mid, last) [off]\n"
                                                                                                             "first: first segment           \n"
                                                                                                             "mid  : all segments between first and last segment\n"
                                                                                                             "last : last segment")
  ;

  if( m_easyMode )
  {
    opts.setSubSection("Output Options");
    opts.addOptions()
    ("output,o",          m_bitstreamFileName,      "Bitstream output file name")
    ;
  }
  else
  {
    opts.setSubSection("Output options");
    opts.addOptions()
    ("BitstreamFile,b",                                 m_bitstreamFileName,                                 "Bitstream output file name")
    ("ReconFile,o",                                     m_reconFileName,                                     "Reconstructed YUV output file name")
    ("OutputBitDepth",                                  c->m_outputBitDepth[ 0 ],                            "Bit-depth of output file")
    ;
  }

  if( m_easyMode )
  {
    opts.setSubSection("Encoder Options");
    opts.addOptions()
    ("preset",                                          toPreset,                                            "select preset for specific encoding setting (faster, fast, medium, slow, slower)")
    ("bitrate,b",                                       c->m_RCTargetBitrate,                                "bitrate for rate control (0: constant-QP encoding without rate control, otherwise bits/second)" )
    ("passes,p",                                        c->m_RCNumPasses,                                    "number of rate control passes (1,2)" )
    ("pass",                                            c->m_RCPass,                                         "rate control pass for two-pass rate control (-1,1,2)" )
    ("rcstatsfile",                                     m_RCStatsFileName,                                   "rate control statistics file" )
    ("qp,q",                                            c->m_QP,                                             "quantization parameter, QP (0-63)")
    ("qpa",                                             toQPA,                                               "Enable perceptually motivated QP adaptation, XPSNR based (0:off, 1:on)", true)
    ("threads,t",                                       c->m_numThreads,                                     "Number of threads default: [size < 720p: 4, >= 720p: 8]")
    ("gopsize,g",                                       c->m_GOPSize,                                        "GOP size of temporal structure (16,32)")
    ("refreshtype,-rt",                                 toDecRefreshType,                                    "intra refresh type (idr,cra,idr2,cra_cre - CRA with constrained encoding for RASL pictures)")
    ("refreshsec,-rs",                                  c->m_IntraPeriodSec,                                 "Intra period/refresh in seconds")
    ("intraperiod,-ip",                                 c->m_IntraPeriod,                                    "Intra period in frames (0: use intra period in seconds (refreshsec), else: n*gopsize)")
    ("tiles",                                           toNumTiles,                                          "Set number of tile columns and rows")
    ;
  }
  else
  {
    opts.setSubSection("Threading, performance");
    opts.addOptions()
    ("Threads,t",                                       c->m_numThreads,                                     "Number of threads")
    ("preset",                                          toPreset,                                            "select preset for specific encoding setting (faster, fast, medium, slow, slower)")
    ("Tiles",                                           toNumTiles,                                          "Set number of tile columns and rows")
    ;

    opts.setSubSection("Slice decision options");
    opts.addOptions()
    ("IntraPeriod,-ip",                                c->m_IntraPeriod,                                     "Intra period in frames (0: use intra period in seconds (refreshsec), else: n*gopsize)")
    ("RefreshSec,-rs",                                 c->m_IntraPeriodSec,                                  "Intra period/refresh in seconds")
    ("DecodingRefreshType,-dr",                        toDecRefreshType,                                     "Intra refresh type (0:none, 1:CRA, 2:IDR, 3:RecPointSEI, 4:IDR2, 5:CRA_CRE - CRA with constrained encoding for RASL pictures)")
    ("GOPSize,g",                                      c->m_GOPSize,                                         "GOP size of temporal structure (16,32)")
    ;

    opts.setSubSection("Rate control, Perceptual Quantization");
    opts.addOptions()
    ("NumPasses",                                       c->m_RCNumPasses,                                    "number of rate control passes (1,2)" )
    ("Passes",                                          c->m_RCNumPasses,                                    "number of rate control passes (1,2)" )
    ("Pass",                                            c->m_RCPass,                                         "rate control pass for two-pass rate control (-1,1,2)" )
    ("LookAhead",                                       c->m_RCLookAhead,                                    "enable pre-analysis in single pass rate control" )
    ("RCStatsFile",                                     m_RCStatsFileName,                                   "rate control statistics file" )
    ("TargetBitrate",                                   c->m_RCTargetBitrate,                                "Rate control: target bit-rate [bps]" )
    ("PerceptQPA,-qpa",                                 c->m_usePerceptQPA,                                  "Enable perceptually motivated QP adaptation, XPSNR based (0:off, 1:on)", true)
    ;

    opts.setSubSection("Quantization paramters");
    opts.addOptions()
    ("QP,q",                                            c->m_QP,                                             "Qp value (0-63)")
    ;
  }

  opts.setSubSection("Profile, Level, Tier");
  opts.addOptions()
  ("Profile",                                           toProfile,                                           "select profile (main10, main10_stillpic)")
  ("Level",                                             toLevel,                                             "Level limit (1.0, 2.0,2.1, 3.0,3.1, 4.0,4.1, 5.0,5.1,5.2, 6.0,6.1,6.2,6.3, 15.5)")
  ("Tier",                                              toLevelTier,                                         "Tier to use for interpretation of level (main or high)")
  ;

  if( m_easyMode )
  {
    opts.setSubSection("HDR and Color Options");
    opts.addOptions()
    ("hdr",                                             toHDRMode,                                           "set HDR mode (+SEI messages) + BT.709 or BT.2020 color space. "
                                                                                                             "use: off, pq|hdr10, pq_2020|hdr10_2020, hlg, hlg_2020")
    ;
  }
  else
  {
    opts.setSubSection("VUI and SEI options");
    opts.addOptions()
    ("Hdr",                                             toHDRMode,                                           "set HDR mode (+SEI messages) + BT.709 or BT.2020 color space. "
                                                                                                             "If maxcll or masteringdisplay is set, HDR10/PQ is enabled. use: off, pq|hdr10, pq_2020|hdr10_2020, hlg, hlg_2020")
    ;
  }


  std::ostringstream easyOpts;
  po::doHelp( easyOpts, opts );
  // ---------------------------------------------------

  if( m_easyMode )
  {
    opts.setSubSection("Encoder Options");
    opts.addOptions()
    ("internal-bitdepth",                               c->m_internalBitDepth[0],                           "internal bitdepth (8,10)")
    ("accessunitdelimiter,-aud",                        toAud,                                              "Emit Access Unit Delimiter NALUs  (auto(-1),off(0),on(1); default: auto - only if needed by dependent options)", true)
    ("vuiparameterspresent,-vui",                       toVui,                                              "Emit VUI information (auto(-1),off(0),on(1); default: auto - only if needed by dependent options)", true)
    ("hrdparameterspresent,-hrd",                       c->m_hrdParametersPresent,                          "Emit VUI HRD information (0: off, 1: on; default: 1)")
    ("decodedpicturehash,-dph",                         toHashType,                                         "Control generation of decode picture hash SEI messages, (0:off, 1:md5, 2:crc, 3:checksum)")
    ;
  }

  // ---------------------------------------------------

  opts.setSubSection("General Options");
  opts.addOptions()
  ("additional",                                        m_additionalSettings,                               "additional options as string (e.g: \"bitrate=1000000 passes=1\")")
  ;

  if( !m_easyMode )
  {
    opts.setSubSection("General Options");
    opts.addOptions()
    ("c",                                               po::parseConfigFile,                                "configuration file name")
    ("WriteConfig",                                     writeCfg,                                           "write the encoder config into configuration file")
    ("WarnUnknowParameter,w",                           warnUnknowParameter,                                "warn for unknown configuration parameters instead of failing")
    ("SIMD",                                            ignoreParams,                                       "SIMD extension to use (SCALAR, SSE41, SSE42, AVX, AVX2, AVX512), default: the highest supported extension")
    ;

    opts.setSubSection("Input Options");
    opts.addOptions()
    ("SourceWidth",                                     c->m_SourceWidth,                                    "Source picture width")
    ("SourceHeight",                                    c->m_SourceHeight,                                   "Source picture height")
    ("ConformanceWindowMode",                           c->m_conformanceWindowMode,                          "Window conformance mode (0:off, 1:automatic padding, 2:padding, 3:conformance")
    ("ConfWinLeft",                                     c->m_confWinLeft,                                    "Left offset for window conformance mode 3")
    ("ConfWinRight",                                    c->m_confWinRight,                                   "Right offset for window conformance mode 3")
    ("ConfWinTop",                                      c->m_confWinTop,                                     "Top offset for window conformance mode 3")
    ("ConfWinBottom",                                   c->m_confWinBottom,                                  "Bottom offset for window conformance mode 3")
    ("TemporalSubsampleRatio",                          c->m_temporalSubsampleRatio,                         "Temporal sub-sample ratio when reading input YUV")
    ("HorizontalPadding",                               c->m_aiPad[0],                                       "Horizontal source padding for conformance window mode 2")
    ("VerticalPadding",                                 c->m_aiPad[1],                                       "Vertical source padding for conformance window mode 2")
    ("InputChromaFormat",                               toInputFileChromaFormat,                             "input file chroma format (400, 420, 422, 444)")
    ("PackedInput",                                     m_packedYUVInput,                                    "Enable 10-bit packed YUV input data ( pack 4 samples( 8-byte) into 5-bytes consecutively.")
    ;

    opts.setSubSection("Profile, Level, Tier");
    opts.addOptions()
    ("SubProfile",                                      c->m_subProfile,                                     "Sub-profile idc")
    ("MaxBitDepthConstraint",                           c->m_bitDepthConstraintValue,                        "Bit depth to use for profile-constraint for RExt profiles. (0: automatically choose based upon other parameters)")
    ("IntraConstraintFlag",                             c->m_intraOnlyConstraintFlag,                        "Value of general_intra_constraint_flag to use for RExt profiles (not used if an explicit RExt sub-profile is specified)")
    ;
    ;

    opts.setSubSection("Quality reporting metrics");
    opts.addOptions()
    ("MSEBasedSequencePSNR",                            c->m_printMSEBasedSequencePSNR,                      "Emit sequence PSNR (0: only as a linear average of the frame PSNRs, 1: also based on an average of the frame MSEs")
    ("PrintHexPSNR",                                    c->m_printHexPsnr,                                   "Emit hexadecimal PSNR for each frame (0: off , 1:on")
    ("PrintFrameMSE",                                   c->m_printFrameMSE,                                  "Emit MSE values for each frame (0: off , 1:on")
    ("PrintSequenceMSE",                                c->m_printSequenceMSE,                               "Emit MSE values for the whole sequence (0: off , 1:on)")
    ;

    opts.setSubSection("Bitstream options");
    opts.addOptions()
    ("CabacZeroWordPaddingEnabled",                     c->m_cabacZeroWordPaddingEnabled,                    "Add conforming cabac-zero-words to bit streams (0: do not add, 1: add as required)")
    ;

    opts.setSubSection("Rate control, Perceptual Quantization");
    opts.addOptions()
    ("RCInitialQP",                                     c->m_RCInitialQP,                                    "Rate control: initial QP. With two-pass encoding, this specifies the first-pass base QP (instead of using a default QP). Activated if value is greater than zero" )
    ("RCForceIntraQP",                                  c->m_RCForceIntraQP,                                 "Rate control: force intra QP to be equal to initial QP" )
    ("PerceptQPATempFiltIPic",                          c->m_usePerceptQPATempFiltISlice,                    "Temporal high-pass filter in QPA activity calculation for key pictures (0:off, 1:on, 2:on incl. temporal pumping reduction, -1:auto)")
    ;

    // Coding structure paramters
    opts.setSubSection("Coding structure paramters");
    opts.addOptions()
    ("ReWriteParamSets",                                c->m_rewriteParamSets,                               "Enable rewriting of Parameter sets before every (intra) random access point")
    ("IDRRefParamList",                                 c->m_idrRefParamList,                                "Enable indication of reference picture list syntax elements in slice headers of IDR pictures")
    ;

    /* Quantization parameters */
    opts.setSubSection("Quantization paramters");
    opts.addOptions()
    ("SameCQPTablesForAllChroma",                       c->m_useSameChromaQPTables,                          "0: Different tables for Cb, Cr and joint Cb-Cr components, 1 (default): Same tables for all three chroma components")
    ("IntraQPOffset",                                   c->m_intraQPOffset,                                  "Qp offset value for intra slice, typically determined based on GOP size")
    ("LambdaFromQpEnable",                              c->m_lambdaFromQPEnable,                             "Enable flag for derivation of lambda from QP")
    ("LambdaModifier",                                  toLambdaModifier,                                    "Lambda modifier list for temporal layers. If LambdaModifierI is used, this will not affect intra pictures")
    ("LambdaModifierI",                                 toIntraLambdaModifier,                               "Lambda modifiers for Intra pictures, comma separated, up to one the number of temporal layer. If entry for temporalLayer exists, then use it, else if some are specified, use the last, else use the standard LambdaModifiers.")
    ("IQPFactor",                                       c->m_dIntraQpFactor,                                 "Intra QP Factor for Lambda Computation. If negative, the default will scale lambda based on GOP size (unless LambdaFromQpEnable then IntraQPOffset is used instead)")
    ("QpInValCb",                                       toQpInCb,                                            "Input coordinates for the QP table for Cb component")
    ("QpInValCr",                                       toQpInCr,                                            "Input coordinates for the QP table for Cr component")
    ("QpInValCbCr",                                     toQpInCbCr,                                          "Input coordinates for the QP table for joint Cb-Cr component")
    ("QpOutValCb",                                      toQpOutCb,                                           "Output coordinates for the QP table for Cb component")
    ("QpOutValCr",                                      toQpOutCr,                                           "Output coordinates for the QP table for Cr component")
    ("QpOutValCbCr",                                    toQpOutCbCr,                                         "Output coordinates for the QP table for joint Cb-Cr component")
    ("MaxCuDQPSubdiv",                                  c->m_cuQpDeltaSubdiv,                                "Maximum subdiv for CU luma Qp adjustment")
    ("MaxCuChromaQpOffsetSubdiv",                       c->m_cuChromaQpOffsetSubdiv,                         "Maximum subdiv for CU chroma Qp adjustment - set less than 0 to disable")
    ("CbQpOffset",                                      c->m_chromaCbQpOffset,                               "Chroma Cb QP Offset")
    ("CrQpOffset",                                      c->m_chromaCrQpOffset,                               "Chroma Cr QP Offset")
    ("CbQpOffsetDualTree",                              c->m_chromaCbQpOffsetDualTree,                       "Chroma Cb QP Offset for dual tree")
    ("CrQpOffsetDualTree",                              c->m_chromaCrQpOffsetDualTree,                       "Chroma Cr QP Offset for dual tree")
    ("CbCrQpOffset",                                    c->m_chromaCbCrQpOffset,                             "QP Offset for joint Cb-Cr mode")
    ("CbCrQpOffsetDualTree",                            c->m_chromaCbCrQpOffsetDualTree,                     "QP Offset for joint Cb-Cr mode in dual tree")
    ("SliceChromaQPOffsetPeriodicity",                  c->m_sliceChromaQpOffsetPeriodicity,                 "Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.")
    ("SliceCbQpOffsetIntraOrPeriodic",                  c->m_sliceChromaQpOffsetIntraOrPeriodic[0],          "Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")
    ("SliceCrQpOffsetIntraOrPeriodic",                  c->m_sliceChromaQpOffsetIntraOrPeriodic[1],          "Chroma Cr QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")

    ("LumaLevelToDeltaQPMode",                          c->m_lumaLevelToDeltaQPEnabled,                      "Luma based Delta QP 0(default): not used. 1: Based on CTU average")
    ("WCGPPSEnable",                                    c->m_wcgChromaQpControl.enabled,                     "1: Enable the WCG PPS chroma modulation scheme. 0 (default) disabled")
    ("WCGPPSCbQpScale",                                 c->m_wcgChromaQpControl.chromaCbQpScale,             "WCG PPS Chroma Cb QP Scale")
    ("WCGPPSCrQpScale",                                 c->m_wcgChromaQpControl.chromaCrQpScale,             "WCG PPS Chroma Cr QP Scale")
    ("WCGPPSChromaQpScale",                             c->m_wcgChromaQpControl.chromaQpScale,               "WCG PPS Chroma QP Scale")
    ("WCGPPSChromaQpOffset",                            c->m_wcgChromaQpControl.chromaQpOffset,              "WCG PPS Chroma QP Offset")
    ;

    opts.setSubSection("Misc. options");
    opts.addOptions()
    ("ChromaFormatIDC,-cf",                             toInternChromaFormat,                                "intern chroma format (400, 420, 422, 444) or set to 0 (default), same as InputChromaFormat")
    ("UseIdentityTableForNon420Chroma",                 c->m_useIdentityTableForNon420Chroma,                "True: Indicates that 422/444 chroma uses identity chroma QP mapping tables; False: explicit Qp table may be specified in config")
    ("InputBitDepthC",                                  c->m_inputBitDepth[ 1 ],                             "As per InputBitDepth but for chroma component. (default:InputBitDepth)")
    ("InternalBitDepth",                                c->m_internalBitDepth[ 0 ],                          "Bit-depth the codec operates at. (default: MSBExtendedBitDepth). If different to MSBExtendedBitDepth, source data will be converted")
    ("OutputBitDepthC",                                 c->m_outputBitDepth[ 1 ],                            "As per OutputBitDepth but for chroma component. (default: use luma output bit-depth)")
    ("MSBExtendedBitDepth",                             c->m_MSBExtendedBitDepth[ 0 ],                       "bit depth of luma component after addition of MSBs of value 0 (used for synthesising High Dynamic Range source material). (default:InputBitDepth)")
    ("MSBExtendedBitDepthC",                            c->m_MSBExtendedBitDepth[ 1 ],                       "As per MSBExtendedBitDepth but for chroma component. (default:MSBExtendedBitDepth)")

    ("WaveFrontSynchro",                                c->m_entropyCodingSyncEnabled,                       "Enable entropy coding sync")
    ("EntryPointsPresent",                              c->m_entryPointsPresent,                             "Enable entry points in slice header")
    
    ("TreatAsSubPic",                                   c->m_treatAsSubPic,                                  "Allow generation of subpicture streams. Disable LMCS, AlfTempPred and JCCR")
    ("ExplicitAPSid",                                   c->m_explicitAPSid,                                  "Set ALF APS id")
    ;

    opts.setSubSection("Quad-Tree size and depth");
    opts.addOptions()
    ("CTUSize",                                         c->m_CTUSize,                                        "CTUSize")
    ("MinQTISlice",                                     c->m_MinQT[0],                                       "MinQTISlice")
    ("MinQTLumaISlice",                                 c->m_MinQT[0],                                       "MinQTLumaISlice")
    ("MinQTNonISlice",                                  c->m_MinQT[1],                                       "MinQTNonISlice")
    ("MinQTChromaISliceInChromaSamples",                c->m_MinQT[2],                                       "MinQTChromaISlice")
    ("MaxMTTDepth",                                     c->m_maxMTTDepth,                                    "maxMTTDepth")
    ("MaxMTTDepthI",                                    c->m_maxMTTDepthI,                                   "maxMTTDepthI")
    ("MaxMTTDepthISliceL",                              c->m_maxMTTDepthI,                                   "maxMTTDepthISliceL")
    ("MaxMTTDepthISliceC",                              c->m_maxMTTDepthIChroma,                             "maxMTTDepthISliceC")
    // --> deprecated
    ("MaxMTTHierarchyDepth",                            c->m_maxMTTDepth,                                    "maxMTTDepth")
    ("MaxMTTHierarchyDepthI",                           c->m_maxMTTDepthI,                                   "maxMTTDepthI")
    ("MaxMTTHierarchyDepthISliceL",                     c->m_maxMTTDepthI,                                   "maxMTTDepthISliceL")
    ("MaxMTTHierarchyDepthISliceC",                     c->m_maxMTTDepthIChroma,                             "maxMTTDepthISliceC")
    // <-- deprecated
    ("MaxBTLumaISlice",                                 c->m_maxBT[0],                                       "MaxBTLumaISlice")
    ("MaxBTChromaISlice",                               c->m_maxBT[2],                                       "MaxBTChromaISlice")
    ("MaxBTNonISlice",                                  c->m_maxBT[1],                                       "MaxBTNonISlice")
    ("MaxTTLumaISlice",                                 c->m_maxTT[0],                                       "MaxTTLumaISlice")
    ("MaxTTChromaISlice",                               c->m_maxTT[2],                                       "MaxTTChromaISlice")
    ("MaxTTNonISlice",                                  c->m_maxTT[1],                                       "MaxTTNonISlice")
    ("DualITree",                                       c->m_dualITree,                                      "Use separate luma and chroma QTBT trees for intra slice")
    ("Log2MaxTbSize",                                   c->m_log2MaxTbSize,                                  "Maximum transform block size in logarithm base 2")
    ("Log2MinCodingBlockSize",                          c->m_log2MinCodingBlockSize,                         "Minimum coding block size in logarithm base 2")
    ;

    opts.setSubSection("Coding tools");
    opts.addOptions()
    ("CostMode",                                        toCostMode,                                          "Use alternative cost functions: choose between 'lossy', 'sequence_level_lossless', 'lossless' (which forces QP to " MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP) ") and 'mixed_lossless_lossy' (which used QP'=" MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME) " for pre-estimates of transquant-bypass blocks).")
    ("ASR",                                             c->m_bUseASR,                                        "Adaptive motion search range")
    ("HadamardME",                                      c->m_bUseHADME,                                      "Hadamard ME for fractional-pel")
    ("RDOQ",                                            c->m_RDOQ,                                           "Rate-Distortion Optimized Quantization mode")
    ("RDOQTS",                                          c->m_useRDOQTS,                                      "Rate-Distortion Optimized Quantization mode for TransformSkip")
    ("SelectiveRDOQ",                                   c->m_useSelectiveRDOQ,                               "Enable selective RDOQ")

    ("JointCbCr",                                       c->m_JointCbCrMode,                                  "Enable joint coding of chroma residuals (0:off, 1:on)")
    ("CabacInitPresent",                                c->m_cabacInitPresent,                               "Enable cabac table index selection based on previous frame")
    ("LCTUFast",                                        c->m_useFastLCTU,                                    "Fast methods for large CTU")
    ("PBIntraFast",                                     c->m_usePbIntraFast,                                 "Intra mode pre-check dependent on best Inter mode, skip intra if it is not probable (0:off, 1: VTM, 2: relaxed, giving intra more chance)")
    ("FastMrg",                                         c->m_useFastMrg,                                     "Fast methods for inter merge")
    ("AMaxBT",                                          c->m_useAMaxBT,                                      "Adaptive maximal BT-size")
    ("FastQtBtEnc",                                     c->m_fastQtBtEnc,                                    "Fast encoding setting for QTBT")
    ("ContentBasedFastQtbt",                            c->m_contentBasedFastQtbt,                           "Signal based QTBT speed-up")
    ("FEN",                                             c->m_fastInterSearchMode,                            "fast encoder setting")
    ("ECU",                                             c->m_useEarlyCU,                                     "Early CU setting (1: ECU limited to specific block size and TL, 2: unconstrained ECU)")
    ("FDM",                                             c->m_useFastDecisionForMerge,                        "Fast decision for Merge RD Cost")

    ("DisableIntraInInter",                             c->m_bDisableIntraCUsInInterSlices,                  "Flag to disable intra CUs in inter slices")
    ("ConstrainedIntraPred",                            c->m_bUseConstrainedIntraPred,                       "Constrained Intra Prediction")
    ("FastUDIUseMPMEnabled",                            c->m_bFastUDIUseMPMEnabled,                          "If enabled, adapt intra direction search, accounting for MPM")
    ("FastMEForGenBLowDelayEnabled",                    c->m_bFastMEForGenBLowDelayEnabled,                  "If enabled use a fast ME for generalised B Low Delay slices")

    ("MTSImplicit",                                     c->m_MTSImplicit,                                    "Enable implicit MTS when explicit MTS is off\n")
    ("TMVPMode",                                        c->m_TMVPModeId,                                     "TMVP mode enable(0: off 1: for all slices 2: for certain slices only)")
    ("DepQuant",                                        c->m_DepQuantEnabled,                                "Enable dependent quantization" )
    ("QuantThrVal",                                     c->m_quantThresholdVal,                              "Quantization threshold value for DQ last coefficient search" )
    ("SignHideFlag",                                    c->m_SignDataHidingEnabled,                          "Enable sign data hiding" )
    ("MIP",                                             c->m_MIP,                                            "Enable MIP (matrix-based intra prediction)")
    ("FastMIP",                                         c->m_useFastMIP,                                     "Fast encoder search for MIP (matrix-based intra prediction)")
    ("MaxNumMergeCand",                                 c->m_maxNumMergeCand,                                "Maximum number of merge candidates")
    ("MaxNumAffineMergeCand",                           c->m_maxNumAffineMergeCand,                          "Maximum number of affine merge candidates")
    ("Geo",                                             c->m_Geo,                                            "Enable geometric partitioning mode (0:off, 1:on)")
    ("MaxNumGeoCand",                                   c->m_maxNumGeoCand,                                  "Maximum number of geometric partitioning mode candidates")
    ("FastIntraTools",                                  c->m_FastIntraTools,                                 "SpeedUPIntraTools:LFNST,ISP,MTS. (0:off, 1:speed1, 2:speed2)")
    ("IntraEstDecBit",                                  c->m_IntraEstDecBit,                                 "Intra estimation decimation binary exponent for first pass directional modes screening (only test each (2^N)-th mode in the first estimation pass)")
    ;

    // motion search options
    opts.setSubSection("Motion search options");
    opts.addOptions()
    ("FastSearch",                                      c->m_motionEstimationSearchMethod,                   "Search mode (0:Full search 1:Diamond 2:Selective 3:Enhanced Diamond 4: FastDiamond)")
    ("FastSearchSCC",                                   c->m_motionEstimationSearchMethodSCC,                "Search mode for SCC (0:use non SCC-search 1:Selective 2:DiamondSCC 3:FastDiamondSCC)")
    ("RestrictMESampling",                              c->m_bRestrictMESampling,                            "Enable restrict ME Sampling for selective inter motion search")
    ("SearchRange,-sr",                                 c->m_SearchRange,                                    "Motion search range")
    ("BipredSearchRange",                               c->m_bipredSearchRange,                              "Motion search range for bipred refinement")
    ("MinSearchWindow",                                 c->m_minSearchWindow,                                "Minimum motion search window size for the adaptive window ME")
    ("ClipForBiPredMEEnabled",                          c->m_bClipForBiPredMeEnabled,                        "Enable clipping in the Bi-Pred ME.")
    ("FastMEAssumingSmootherMVEnabled",                 c->m_bFastMEAssumingSmootherMVEnabled,               "Enable fast ME assuming a smoother MV.")
    ("IntegerET",                                       c->m_bIntegerET,                                     "Enable early termination for integer motion search")
    ("FastSubPel",                                      c->m_fastSubPel,                                     "Enable fast sub-pel ME (1: enable fast sub-pel ME, 2: completely disable sub-pel ME)")
    ;

    // Deblocking filter parameters
    opts.setSubSection("Loop filters (deblock and SAO)");
    opts.addOptions()
    ("LoopFilterDisable",                               c->m_bLoopFilterDisable,                             "")
    ("LoopFilterOffsetInPPS",                           c->m_loopFilterOffsetInPPS,                          "")
    ("LoopFilterBetaOffset_div2",                       c->m_loopFilterBetaOffsetDiv2[0],                    "")
    ("LoopFilterTcOffset_div2",                         c->m_loopFilterTcOffsetDiv2[0],                      "")
    ("LoopFilterCbBetaOffset_div2",                     c->m_loopFilterBetaOffsetDiv2[1],                    "")
    ("LoopFilterCbTcOffset_div2",                       c->m_loopFilterTcOffsetDiv2[1],                      "")
    ("LoopFilterCrBetaOffset_div2",                     c->m_loopFilterBetaOffsetDiv2[2],                    "")
    ("LoopFilterCrTcOffset_div2",                       c->m_loopFilterTcOffsetDiv2[2],                      "")
    ("DeblockingFilterMetric",                          c->m_deblockingFilterMetric,                         "")

    ("DisableLoopFilterAcrossTiles",                    c->m_bDisableLFCrossTileBoundaryFlag,                "Loop filtering applied across tile boundaries or not (0: filter across tile boundaries  1: do not filter across tile boundaries)")
    ("DisableLoopFilterAcrossSlices",                   c->m_bDisableLFCrossSliceBoundaryFlag,               "Loop filtering applied across tile boundaries or not (0: filter across slice boundaries  1: do not filter across slice boundaries)")

    ("SAO",                                             c->m_bUseSAO,                                        "Enable Sample Adaptive Offset")
    ("SaoEncodingRate",                                 c->m_saoEncodingRate,                                "When >0 SAO early picture termination is enabled for luma and chroma")
    ("SaoEncodingRateChroma",                           c->m_saoEncodingRateChroma,                          "The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma")
    ("SaoLumaOffsetBitShift",                           c->m_saoOffsetBitShift[ 0 ],                         "Specify the luma SAO bit-shift. If negative, automatically calculate a suitable value based upon bit depth and initial QP")
    ("SaoChromaOffsetBitShift",                         c->m_saoOffsetBitShift[ 1 ],                         "Specify the chroma SAO bit-shift. If negative, automatically calculate a suitable value based upon bit depth and initial QP")
    ;

    opts.setSubSection("VUI and SEI options");
    opts.addOptions()
    ("SEIDecodedPictureHash,-dph",                      toHashType,                                          "Control generation of decode picture hash SEI messages, (0:off, 1:md5, 2:crc, 3:checksum)" )
    ("SEIBufferingPeriod",                              c->m_bufferingPeriodSEIEnabled,                      "Control generation of buffering period SEI messages")
    ("SEIPictureTiming",                                c->m_pictureTimingSEIEnabled,                        "Control generation of picture timing SEI messages")
    ("SEIDecodingUnitInfo",                             c->m_decodingUnitInfoSEIEnabled,                     "Control generation of decoding unit information SEI message.")
    ("EnableDecodingParameterSet",                      c->m_decodingParameterSetEnabled,                    "Enable writing of Decoding Parameter Set")
    ("AccessUnitDelimiter,-aud",                        toAud,                                               "Enable Access Unit Delimiter NALUs, (default: auto - enable only if needed by dependent options)" , true)
    ("VuiParametersPresent,-vui",                       toVui,                                               "Enable generation of vui_parameters(), (default: auto - enable only if needed by dependent options)", true)
    ("HrdParametersPresent,-hrd",                       c->m_hrdParametersPresent,                           "Enable generation of hrd_parameters(), (0: off, 1: on; default: 1)")
    ("AspectRatioInfoPresent",                          c->m_aspectRatioInfoPresent,                         "Signals whether aspect_ratio_idc is present")
    ("AspectRatioIdc",                                  c->m_aspectRatioIdc,                                 "aspect_ratio_idc")
    ("SarWidth",                                        c->m_sarWidth,                                       "horizontal size of the sample aspect ratio")
    ("SarHeight",                                       c->m_sarHeight,                                      "vertical size of the sample aspect ratio")
    ("ColourDescriptionPresent",                        c->m_colourDescriptionPresent,                       "Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present")
    ("ColourPrimaries",                                 toColorPrimaries,                                    "Specify color primaries (0-13): reserved, bt709, unknown, empty, bt470m, bt470bg, smpte170m, "
                                                                                                             "smpte240m, film, bt2020, smpte428, smpte431, smpte432")

    ("TransferCharacteristics",                         toTransferCharacteristics,                           "Specify opto-electroni transfer characteristics (0-18): reserved, bt709, unknown, empty, bt470m, bt470bg, smpte170m, "
                                                                                                             "smpte240m, linear, log100, log316, iec61966-2-4, bt1361e, iec61966-2-1, "
                                                                                                             "bt2020-10, bt2020-12, smpte2084, smpte428, arib-std-b67")
    ("MatrixCoefficients",                              toColorMatrix,                                       "Specify color matrix setting to derive luma/chroma from RGB primaries (0-14): gbr, bt709, unknown, empty, fcc, bt470bg, smpte170m, "
                                                                                                             "smpte240m, ycgco, bt2020nc, bt2020c, smpte2085, chroma-derived-nc, chroma-derived-c, ictcp")

    ("ChromaLocInfoPresent",                            c->m_chromaLocInfoPresent,                           "Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present")
    ("ChromaSampleLocTypeTopField",                     c->m_chromaSampleLocTypeTopField,                    "Specifies the location of chroma samples for top field")
    ("ChromaSampleLocTypeBottomField",                  c->m_chromaSampleLocTypeBottomField,                 "Specifies the location of chroma samples for bottom field")
    ("ChromaSampleLocType",                             c->m_chromaSampleLocType,                            "Specifies the location of chroma samples for progressive content")
    ("OverscanInfoPresent",                             c->m_overscanInfoPresent,                            "Indicates whether conformant decoded pictures are suitable for display using overscan")
    ("OverscanAppropriate",                             c->m_overscanAppropriateFlag,                        "Indicates whether conformant decoded pictures are suitable for display using overscan")
    ("VideoSignalTypePresent",                          c->m_videoSignalTypePresent,                         "Signals whether video_format, video_full_range_flag, and colour_description_present_flag are present")
    ("VideoFullRange",                                  c->m_videoFullRangeFlag,                             "Indicates the black level and range of luma and chroma signals")


    ("MasteringDisplayColourVolume",                    toMasteringDisplay,                               "SMPTE ST 2086 mastering display colour volume info SEI (HDR), "
                                                                                                          "vec(uint) size 10, x,y,x,y,x,y,x,y,max,min where: \"G(x,y)B(x,y)R(x,y)WP(x,y)L(max,min)\""
                                                                                                          "range: 0 <= GBR,WP <= 50000, 0 <= L <= uint; GBR xy coordinates in increment of 1/50000, min/max luminance in units of 1/10000 cd/m2" )
    ("MaxContentLightLevel",                            toContentLightLevel,                              "Specify content light level info SEI as \"cll,fall\" (HDR) max. content light level, "
                                                                                                          "max. frame average light level, range: 1 <= cll,fall <= 65535'")
    ("PreferredTransferCharacteristics",                toPrefTransferCharacteristics,                    "Specify preferred transfer characteristics SEI and overwrite transfer entry in VUI (0-18): reserved, bt709, unknown, empty, bt470m, bt470bg, smpte170m, "
                                                                                                          "smpte240m, linear, log100, log316, iec61966-2-4, bt1361e, iec61966-2-1, "
                                                                                                          "bt2020-10, bt2020-12, smpte2084, smpte428, arib-std-b67")
    ;

    opts.setSubSection("Summary options (debugging)");
    opts.addOptions()
    ("SummaryOutFilename",                              toSummaryOutFilename,                             "Filename to use for producing summary output file. If empty, do not produce a file.")
    ("SummaryPicFilenameBase",                          toSummaryPicFilenameBase,                         "Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended. If empty, do not produce a file.")
    ("SummaryVerboseness",                              c->m_summaryVerboseness,                             "Specifies the level of the verboseness of the text output")
    ;

    opts.setSubSection("Decoding options (debugging)");
    opts.addOptions()
    ("DebugBitstream",                                  toDecodeBitstreams0,                                 "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
    ("DecodeBitstream1",                                toDecodeBitstreams0,                                 "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
    ("DecodeBitstream2",                                toDecodeBitstreams1,                                 "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
    ("DebugPOC",                                        c->m_switchPOC,                                      "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
    ("SwitchPOC",                                       c->m_switchPOC,                                      "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
    ("SwitchDQP",                                       c->m_switchDQP,                                      "delta QP applied to picture with switchPOC and subsequent pictures." )
    ("FastForwardToPOC",                                c->m_fastForwardToPOC,                               "Get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC." )
    ("StopAfterFFtoPOC",                                c->m_stopAfterFFtoPOC,                               "If using fast forward to POC, after the POC of interest has been hit, stop further encoding.")
    ("DecodeBitstream2ModPOCAndType",                   c->m_bs2ModPOCAndType,                               "Modify POC and NALU-type of second input bitstream, to use second BS as closing I-slice")
    ("ForceDecodeBitstream1",                           c->m_forceDecodeBitstream1,                          "force decoding of bitstream 1 - use this only if you are really sure about what you are doing ")
    ;

    opts.setSubSection("Coding tools");
    opts.addOptions()
    ("SMVD",                                            c->m_SMVD,                                           "Enable Symmetric MVD (0:off 1:vtm 2:fast 3:faster\n")
    ("AMVR",                                            c->m_AMVRspeed,                                      "Enable Adaptive MV precision Mode (IMV)")
    // added for backward compatibility with VTM
    ("IMV",                                             c->m_AMVRspeed,                                      "Enable Adaptive MV precision Mode (IMV)")
    ("LMChroma",                                        c->m_LMChroma,                                       "Enable LMChroma prediction")
    ("MRL",                                             c->m_MRL,                                            "MultiRefernceLines")
    ("BDOF",                                            c->m_BDOF,                                           "Enable bi-directional optical flow")
    // added for backward compatibility with VTM
    ("BIO",                                             c->m_BDOF,                                           "Enable bi-directional optical flow")
    ("DMVR",                                            c->m_DMVR,                                           "Decoder-side Motion Vector Refinement")
    ("EncDbOpt",                                        c->m_EDO,                                            "Encoder optimization with deblocking filter 0:off 1:vtm 2:fast")
    ("EDO",                                             c->m_EDO,                                            "Encoder optimization with deblocking filter 0:off 1:vtm 2:fast")
    ("LMCSEnable",                                      c->m_lumaReshapeEnable,                              "Enable LMCS luma mapping with chroma scaling (0:off 1:on 2:use SCC detection to disable for screen coded content)")
    ("LMCS",                                            c->m_lumaReshapeEnable,                              "Enable LMCS luma mapping with chroma scaling (0:off 1:on 2:use SCC detection to disable for screen coded content)")
    ("LMCSSignalType",                                  c->m_reshapeSignalType,                              "Input signal type (0:SDR, 1:HDR-PQ, 2:HDR-HLG)")
    ("LMCSUpdateCtrl",                                  c->m_updateCtrl,                                     "LMCS model update control (0:RA, 1:AI, 2:LDB/LDP)")
    ("LMCSAdpOption",                                   c->m_adpOption,                                      "LMCS adaptation options: 0:automatic,"
                                                                                                               "1: rsp both (CW66 for QP<=22), 2: rsp TID0 (for all QP),"
                                                                                                               "3: rsp inter(CW66 for QP<=22), 4: rsp inter(for all QP).")
    ("LMCSInitialCW",                                   c->m_initialCW,                                      "LMCS initial total codeword (0~1023) when LMCSAdpOption > 0")
    ("LMCSOffset",                                      c->m_LMCSOffset,                                     "LMCS chroma residual scaling offset")
    ("ALF",                                             c->m_alf,                                            "Adpative Loop Filter" )
    ("ALFSpeed",                                        c->m_alfSpeed,                                       "ALF speed (skip filtering of non-referenced frames) [0-1]" )
    ("CCALF",                                           c->m_ccalf,                                          "Cross-component Adaptive Loop Filter" )
    ("CCALFQpTh",                                       c->m_ccalfQpThreshold,                               "QP threshold above which encoder reduces CCALF usage. Ignored in case of PerceptQPA.")
    ("UseNonLinearAlfLuma",                             c->m_useNonLinearAlfLuma,                            "Non-linear adaptive loop filters for Luma Channel")
    ("UseNonLinearAlfChroma",                           c->m_useNonLinearAlfChroma,                          "Non-linear adaptive loop filters for Chroma Channels")
    ("MaxNumAlfAlternativesChroma",                     c->m_maxNumAlfAlternativesChroma,                    std::string("Maximum number of alternative Chroma filters (1-") + std::to_string(VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA) + std::string (", inclusive)") )
    ("ALFTempPred",                                     c->m_alfTempPred,                                    "Enable usage of ALF temporal prediction for filter data\n" )
    ("PROF",                                            c->m_PROF,                                           "Enable prediction refinement with optical flow for affine mode")
    ("Affine",                                          c->m_Affine,                                         "Enable affine prediction")
    ("AffineType",                                      c->m_AffineType,                                     "Enable affine type prediction")
    ("MMVD",                                            c->m_MMVD,                                           "Enable Merge mode with Motion Vector Difference")
    ("MmvdDisNum",                                      c->m_MmvdDisNum,                                     "Number of MMVD Distance Entries")
    ("AllowDisFracMMVD",                                c->m_allowDisFracMMVD,                               "Disable fractional MVD in MMVD mode adaptively")
    ("MCTF",                                            c->m_vvencMCTF.MCTF,                                 "Enable GOP based temporal filter. (0:off, 1:filter all frames, 2:use SCC detection to disable for screen coded content)")
    ("MCTFSpeed",                                       c->m_vvencMCTF.MCTFSpeed,                            "MCTF Fast Mode (0:best quality .. 4:fast)")
    ("MCTFFutureReference",                             c->m_vvencMCTF.MCTFFutureReference,                  "Enable referencing of future frames in the GOP based temporal filter. This is typically disabled for Low Delay configurations.")
    ("MCTFNumLeadFrames",                               c->m_vvencMCTF.MCTFNumLeadFrames,                    "Number of additional MCTF lead frames, which will not be encoded, but can used for MCTF filtering")
    ("MCTFNumTrailFrames",                              c->m_vvencMCTF.MCTFNumTrailFrames,                   "Number of additional MCTF trail frames, which will not be encoded, but can used for MCTF filtering")
    ("MCTFFrame",                                       toMCTFFrames,                                        "Frame to filter Strength for frame in GOP based temporal filter")
    ("MCTFStrength",                                    toMCTFStrengths,                                     "Strength for  frame in GOP based temporal filter.")

    ("FastLocalDualTreeMode",                           c->m_fastLocalDualTreeMode,                          "Fast intra pass coding for local dual-tree in intra coding region (0:off, 1:use threshold, 2:one intra mode only)")
    ("QtbttExtraFast",                                  c->m_qtbttSpeedUp,                                   "Non-VTM compatible QTBTT speed-ups" )
    ("FastTTSplit",                                     c->m_fastTTSplit,                                    "Fast method for TT split" )
    ;

    opts.setSubSection("Threading, performance");
    opts.addOptions()
    ("MaxParallelFrames",                               c->m_maxParallelFrames,                              "Maximum number of frames to be processed in parallel(0:off, >=2: enable parallel frames)")
    ("WppBitEqual",                                     c->m_ensureWppBitEqual,                              "Ensure bit equality with WPP case (0:off (sequencial mode), 1:copy from wpp line above, 2:line wise reset)")
    ("EnablePicPartitioning",                           c->m_picPartitionFlag,                               "Enable picture partitioning (0: single tile, single slice, 1: multiple tiles/slices)")
    ("TileColumnWidthArray",                            toTileColumnWidth,                                   "Tile column widths in units of CTUs. Last column width in list will be repeated uniformly to cover any remaining picture width")
    ("TileRowHeightArray",                              toTileRowHeight,                                     "Tile row heights in units of CTUs. Last row height in list will be repeated uniformly to cover any remaining picture height")
    ("TileParallelCtuEnc",                              c->m_tileParallelCtuEnc,                             "Allow parallel CTU block search in different tiles")
    ;

    opts.setSubSection("Coding tools");
    opts.addOptions()
    ("SbTMVP",                                          c->m_SbTMVP,                                         "Enable Subblock Temporal Motion Vector Prediction (0: off, 1: on)")
    ("CIIP",                                            c->m_CIIP,                                           "Enable CIIP mode, 0: off, 1: vtm, 2: fast, 3: faster ")
    ("SBT",                                             c->m_SBT,                                            "Enable Sub-Block Transform for inter blocks (0: off 1: vtm, 2: fast, 3: faster)" )
    ("LFNST",                                           c->m_LFNST,                                          "Enable LFNST (0: off, 1: on)" )
    ("MTS",                                             c->m_MTS,                                            "Multiple Transform Set (MTS)" )
    ("MTSIntraMaxCand",                                 c->m_MTSIntraMaxCand,                                "Number of additional candidates to test for MTS in intra slices")
    ("ISP",                                             c->m_ISP,                                            "Intra Sub-Partitions Mode (0: off, 1: vtm, 2: fast, 3: faster)")
    ("TransformSkip",                                   c->m_TS,                                             "Intra transform skipping, 0: off, 1: TS, 2: TS with SCC detection ")
    ("TransformSkipLog2MaxSize",                        c->m_TSsize,                                         "Specify transform-skip maximum size. Minimum 2, Maximum 5")
    ("ChromaTS",                                        c->m_useChromaTS,                                    "Enable encoder search of chromaTS")
    ("BDPCM",                                           c->m_useBDPCM,                                       "BDPCM (0:off, 1:luma and chroma, 2: BDPCM with SCC detection)")
    ("RPR",                                             c->m_rprEnabledFlag,                                 "Reference Sample Resolution (0: disable, 1: eneabled, 2: RPR ready")
    ("IBC",                                             c->m_IBCMode,                                        "IBC (0:off, 1:IBC, 2: IBC with SCC detection)")
    ("IBCFastMethod",                                   c->m_IBCFastMethod,                                  "Fast methods for IBC. 1:default, [2..6] speedups")
    ("BCW",                                             c->m_BCW,                                            "Enable Generalized Bi-prediction(Bcw) 0: disabled, 1: enabled, 2: fast")
    ("FastInferMerge",                                  c->m_FIMMode,                                        "Fast method to skip Inter/Intra modes. 0: off, [1..4] speedups")
    ("NumIntraModesFullRD",                             c->m_numIntraModesFullRD,                            "Number modes for full RD intra search [-1, 1..3] (default: -1 auto)")
    ("ReduceIntraChromaModesFullRD",                    c->m_reduceIntraChromaModesFullRD,                   "Reduce modes for chroma full RD intra search")
    ;

    opts.setSubSection("Input Options");
    opts.addOptions()
    ("HorCollocatedChroma",                             c->m_horCollocatedChromaFlag,                        "Specifies location of a chroma sample relatively to the luma sample in horizontal direction in the reference picture resampling"
                                                                                                             "(0: horizontally shifted by 0.5 units of luma samples, 1: collocated)")
    ("VerCollocatedChroma",                             c->m_verCollocatedChromaFlag,                        "Specifies location of a chroma sample relatively to the luma sample in vertical direction in the cross-component linear model intra prediction and the reference picture resampling"
                                                                                                             "(0: horizontally co-sited, vertically shifted by 0.5 units of luma samples, 1: collocated)")
    ("ClipInputVideoToRec709Range",                     m_bClipInputVideoToRec709Range,                      "Enable clipping input video to the Rec. 709 Range on loading when InternalBitDepth is less than MSBExtendedBitDepth")
    ;

    opts.setSubSection("Reconstructed video options");
    opts.addOptions()
    ("ClipOutputVideoToRec709Range",                    m_bClipOutputVideoToRec709Range,                     "Enable clipping output video to the Rec. 709 Range on saving when OutputBitDepth is less than InternalBitDepth")
    ("PYUV",                                            m_packedYUVOutput,                                   "Enable output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data. Ignored for interlaced output.")
    ;
  }

#if ENABLE_TRACING
  {
    opts.setSubSection( "Tracing" );
    opts.addOptions()
    ("tracechannellist",              c->m_listTracingChannels,  "List all available tracing channels")
    ("tracerule",                     toTraceRule,            "Tracing rule (ex: \"D_CABAC:poc==8\" or \"D_REC_CB_LUMA:poc==8\")")
    ("tracefile",                     toTraceFile,            "Tracing file")
    ;
  }
#endif

  std::ostringstream fullOpts;
  po::doHelp( fullOpts, opts );

  for ( int i = 0; i < VVENC_MAX_GOP; i++ )
  {
    std::ostringstream cOSS;
    cOSS << "Frame" << i+1;
    opts.addOptions()(cOSS.str(), c->m_GOPList[i] );
  }
  opts.addOptions()("decode",                           m_decode,                                           "decode only");

  if( !m_easyMode )
  {
    // enable access to easy app param names as hidden option (usable for e.g. vvenc_set_param(c,"bitrate", 500000) )
    opts.setSubSection("extra");
    opts.addOptions()
    ("tickspersec",                                     c->m_TicksPerSecond,                                 "Ticks Per Second for dts generation, (1..27000000)")
    ("framerate,r",                                     c->m_FrameRate,                                      "temporal rate (framerate) e.g. 25,29,30,50,59,60 ")
    ("frames",                                          c->m_framesToBeEncoded,                              "max. frames to encode [all]")
    ("bitrate",                                         c->m_RCTargetBitrate,                                "bitrate for rate control (0: constant-QP encoding without rate control, otherwise bits/second)" )
    ("qpa",                                             toQPA,                                               "Enable perceptually motivated QP adaptation, XPSNR based (0:off, 1:on)", true)
    ;
  }

  //
  // parse command line parameters and read configuration files
  //
  try
  {
    po::ErrorReporter err;
    const std::list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**) argv, err );

    if ( do_help || argc == 0 )
    {
      m_showHelp = true;
      rcOstr << easyOpts.str();
      return 1;
    }
    else if ( do_full_help )
    {
      m_showHelp = true;
      rcOstr << fullOpts.str();
      return 1;
    }

    if( !m_easyMode && !writeCfg.empty() )
    {
      std::ofstream cfgFile;
      cfgFile.open( writeCfg.c_str(), std::ios::out | std::ios::trunc);
      if( !cfgFile.is_open() )
      {
        rcOstr << " [error]: failed to open output config file " << writeCfg << std::endl;
        return -1;
      }
      else
      {
        std::list<std::string> ignoreParamsLst;
        ignoreParamsLst.push_back( "help" );
        ignoreParamsLst.push_back( "longhelp" );
        ignoreParamsLst.push_back( "fullhelp" );
        ignoreParamsLst.push_back( "WriteConfig" );
        ignoreParamsLst.push_back( "configfile" );
        ignoreParamsLst.push_back( "c" );

        ignoreParamsLst.push_back( "decode" );
        for ( int i = 0; i < VVENC_MAX_GOP; i++ )
        {
          std::ostringstream cOSS;
          cOSS << "Frame" << i+1;
          ignoreParamsLst.push_back( cOSS.str() );
        }

        std::ostringstream cfgStream;
        po::saveConfig( cfgStream, opts, ignoreParamsLst );
        cfgFile << cfgStream.str() << std::endl;
        cfgFile.close();
      }
    }

    if ( m_showVersion )
    {
      return 1;
    }

    for( auto& a : argv_unhandled )
    {
      rcOstr << "Unknown argument: '" << a << "'\n";
      ret = -1;
    }

    if( err.is_errored )
    {
      rcOstr << err.outstr.str();
      if( argc == 2 ) return VVENC_PARAM_BAD_NAME;
      else            return -1;
    }
    else if( err.is_warning )
    {
      rcOstr << err.outstr.str();
      return 2;
    }
  }
  catch( df::program_options_lite::ParseFailure &e )
  {
    rcOstr << "Error parsing option \"" << e.arg << "\" with argument \"" << e.val << "\".\n";
    ret = -1;
  }

  return ret;
}

std::string VVEncAppCfg::getAppConfigAsString( vvencMsgLevel eMsgLevel ) const
{
  std::stringstream css;
  if( eMsgLevel >= VVENC_DETAILS )
  {
    css << "Input          File                    : " << m_inputFileName << "\n";
    css << "Bitstream      File                    : " << m_bitstreamFileName << "\n";
    css << "Reconstruction File                    : " << m_reconFileName << "\n";
    css << "RC Statistics  File                    : " << m_RCStatsFileName << "\n";
  }

  return css.str();
}

bool VVEncAppCfg::checkCfg( vvenc_config* c, std::ostream& rcOstr )
{
  bool ret = false;

  // check bitstream file name in encode and decode mode
  if( m_bitstreamFileName.empty() )
  {
    // if rc statsfile is defined and in 1st pass, bitstream file is not needed
    if ( !(c->m_RCPass == 1 && !m_RCStatsFileName.empty()) )
    {
      rcOstr << "error: bitstream file name must be specified (--output=bit.266)" << std::endl;
      ret = true;
    }
  }

  // check remaining parameters in encode mode only
  if( m_decode )
  {
    return ret;
  }

  // check remaining parameter set
  if( !xCheckCfg( c, rcOstr ) )
  {
    ret = true;
  }
  return ret;
}

int get_width_of_component( const vvencChromaFormat chFmt, const int frameWidth, const int compId )
{
  int w = frameWidth;
  if ( compId > 0 )
  {
    switch ( chFmt )
    {
      case VVENC_CHROMA_400: w = 0;      break;
      case VVENC_CHROMA_420:
      case VVENC_CHROMA_422: w = w >> 1; break;
      default: break;
    }
  }
  return w;
}

int get_height_of_component( const vvencChromaFormat chFmt, const int frameHeight, const int compId )
{
  int h = frameHeight;
  if ( compId > 0 )
  {
    switch ( chFmt )
    {
      case VVENC_CHROMA_400: h = 0;      break;
      case VVENC_CHROMA_420: h = h >> 1; break;
      case VVENC_CHROMA_422:
      default: break;
    }
  }
  return h;
}

bool VVEncAppCfg::xCheckCfg( vvenc_config* c, std::ostream& rcOstr )
{
  bool ret = true;

  if( m_inputFileName.empty() )
  {
    rcOstr << "error: input yuv file name must be specified (--input=video.yuv)" << std::endl;
    ret = false;
  }
  if( ! strcmp( m_inputFileName.c_str(), "-" )
      && c->m_RCNumPasses > 1
      && c->m_RCPass < 0 )
  {
    rcOstr << "error: two pass rate control within single application call and reading from stdin not supported" << std::endl;
    ret = false;
  }

  if( ! m_bitstreamFileName.empty() )
  {
    if( c->m_decodeBitstreams[0][0] != '\0' && c->m_decodeBitstreams[0] == m_bitstreamFileName )
    {
      rcOstr << "error: debug bitstream and the output bitstream cannot be equal" << std::endl;
      ret = false;
    }
    if( c->m_decodeBitstreams[1][0] != '\0' && c->m_decodeBitstreams[1] == m_bitstreamFileName )
    {
      rcOstr << "error: decode2 bitstream and the output bitstream cannot be equal" << std::endl;
      ret = false;
    }
  }

#ifndef VVENC_ENABLE_THIRDPARTY_JSON
  if( c->m_RCPass > 0 )
  {
    rcOstr << "error: reading/writing rate control statistics file not supported, please disable pass parameter or compile with json enabled" << std::endl;
    ret = false;
  }
  if( ! m_RCStatsFileName.empty() )
  {
    rcOstr << "error: reading/writing rate control statistics file not supported, please disable rcstatsfile parameter or compile with json enabled" << std::endl;
    ret = false;
  }
#endif

  if( c->m_RCPass > 0 && m_RCStatsFileName.empty() )
  {
    rcOstr << "error: rate control statistics file name must be specify, when pass parameter is set (--rcstatsfile=stats.json)" << std::endl;
    ret = false;
  }
  if( c->m_RCNumPasses == 1 && ! m_RCStatsFileName.empty() )
  {
    rcOstr << "error: rate control statistics file not supported in single pass encoding" << std::endl;
    ret = false;
  }

  if( m_inputFileChromaFormat != VVENC_CHROMA_400 && m_inputFileChromaFormat != VVENC_CHROMA_420 )
  {
    rcOstr << "error: input chroma format must be either 400, 420" << std::endl;
    ret = false;
  }

  if( ( c->m_decodeBitstreams[0][0] != '\0' || c->m_decodeBitstreams[1][0] != '\0' ) && ( c->m_RCTargetBitrate > 0 || c->m_RCLookAhead ) )
  {
    rcOstr << "Debug-bitstream for the rate-control mode is not supported yet" << std::endl;
    ret = false;
  }

  if( m_packedYUVInput )
  {
    if( get_width_of_component( c->m_internChromaFormat, c->m_SourceWidth, 0 ) % 4 != 0 )
    {
      rcOstr <<  "error: unsupported input width for packed input (width must be a multiple of 4)" << std::endl;
      ret = false;
    }

    if( (c->m_internChromaFormat == VVENC_CHROMA_420 || c->m_internChromaFormat == VVENC_CHROMA_422) &&
         get_width_of_component( c->m_internChromaFormat, c->m_SourceWidth, 1 ) % 4 != 0 )
    {
      rcOstr <<  "error: unsupported input width for packed input (chroma width must be a multiple of 4)" << std::endl;
      ret = false;
    }
  }

  if( m_packedYUVOutput && ! m_reconFileName.empty() )
  {
    // if output bitdepth not defined, use internal bitdepth as default value
    if (c->m_outputBitDepth[0] == 0) c->m_outputBitDepth[0] = c->m_internalBitDepth[0];
    if (c->m_outputBitDepth[1] == 0) c->m_outputBitDepth[1] = c->m_outputBitDepth[0];

    if( ( c->m_outputBitDepth[ 0 ] != 10 && c->m_outputBitDepth[ 0 ] != 12 )
        || ( ( ( c->m_SourceWidth ) & ( 1 + ( c->m_outputBitDepth[ 0 ] & 3 ) ) ) != 0 ) )
    {
      rcOstr << "error: invalid output bit-depth or image width for packed YUV output" << std::endl;
      ret = false;
    }
    if( ( c->m_internChromaFormat != VVENC_CHROMA_400 ) && ( ( c->m_outputBitDepth[ 1 ] != 10 && c->m_outputBitDepth[ 1 ] != 12 )
          || ( ( get_width_of_component( c->m_internChromaFormat, c->m_SourceWidth, 1 ) & ( 1 + ( c->m_outputBitDepth[ 1 ] & 3 ) ) ) != 0 ) ) )
    {
      rcOstr << "error: invalid chroma output bit-depth or image width for packed YUV output" << std::endl;
      ret = false;
    }
  }

  return ret;
}


std::vector<std::string> VVEncAppCfg::tokenize(std::string str, char delimiter )
{
  std::vector<string> tokenvec;
  bool tupleRequired=false;
  std::string cDefDel = " ";
  if ( delimiter == '=' )
  {
    // when searching for "=" we always want to return a tuple of "key value"
    replace_if( str.begin(), str.end(), []( int c ){ return isspace( c ) || c == '='; }, ' ' );
    tupleRequired=true;
  }
  else if ( delimiter == ':' )
    replace_if( str.begin(), str.end(), []( int c ){ return isspace( c ) || c == ':'; }, ' ' );
  else
    replace_if( str.begin(), str.end(), []( int c ){ return isspace( c ); }, ' ' );

  std::string cDelList = "\"";  // lists must be defined within " "

  size_t start = str.find_first_not_of(cDefDel);
  size_t end=std::string::npos;

  while (start != std::string::npos)
  {
    size_t startL = str.find(cDelList, start);  // Find occurence of a list
    size_t endL = str.find(cDelList, startL+1); // Find occurence of list end

    end = str.find(cDefDel, start); // Find next occurence of delimiter

    if( startL != std::string::npos && endL != std::string::npos && startL <= end)
    {
      // extract a value as list, when " " was found ( only when both " was found a next space is in current statement)
      std::string listItem = str.substr(start, endL-start );
      std::string::iterator end_pos = std::remove(listItem.begin(), listItem.end(), '\"');
      listItem.erase(end_pos, listItem.end());
      tokenvec.push_back( listItem );

      end = str.find(cDefDel, endL);
      start = str.find_first_not_of(cDefDel, end);
    }
    else if( startL != std::string::npos && endL == std::string::npos)
    {
      // error case, found start ", but without end
      tokenvec.clear();
      return tokenvec;
    }
    else
    {
      // Push back the token found into vector
      if( (start==0 && tokenvec.empty()) || start>0 )
      {
        if( tupleRequired && !tokenvec.empty() )
        {
          // when in tuple mode, we extract everything after = as a value
          end=std::string::npos;
          tokenvec.push_back( str.substr(start, end) );
        }
        else
        {
          tokenvec.push_back( str.substr(start, end-start) );
        }
      }
      // Skip all occurences of the delimiter to find new start
      start = str.find_first_not_of(cDefDel, end);
    }
  }

  return tokenvec;
}

std::vector <std::tuple<std::string, std::string>> VVEncAppCfg::getAdditionalSettingList()
{
  std::vector <std::tuple<std::string, std::string>> settingsDictionary;
  if ( !m_additionalSettings.empty())
  {
    char delimiter =':'; // delimiter to mark entry beside space
    std::vector<string> tokenvec = tokenize( m_additionalSettings, delimiter );

    for( auto &t : tokenvec )
    {
      char delimiterEntryValue = '='; // delimiter to mark entry from value
      std::vector<string> entry = tokenize( t, delimiterEntryValue );

      if( !entry.empty() )
      {
        std::string key   = entry.front();
        std::string value = entry.size() > 1 ? entry[1] : "";

        settingsDictionary.push_back(std::make_tuple( key, value) );
      }
    }
  }

  return settingsDictionary;
}


} // namespace

//! \}


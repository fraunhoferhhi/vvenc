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
  { "0",                     VVENC_DRT_NONE },
  { "1",                     VVENC_DRT_CRA },
  { "2",                     VVENC_DRT_IDR },
  { "3",                     VVENC_DRT_RECOVERY_POINT_SEI },
  { "4",                     VVENC_DRT_IDR2 },
};

const std::vector<SVPair<BitDepthAndColorSpace>> BitColorSpaceToIntMap =
{
  { "yuv420",                    YUV420_8 },
  { "yuv420_10",                 YUV420_10 },
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


void setPresets( VVEncAppCfg* cfg, int preset )
{
  vvenc_init_preset( cfg, (vvencPresetMode)preset );
}

void setInputBitDepthAndColorSpace( VVEncAppCfg* cfg, int dbcs )
{
  switch( dbcs )
  {
  case YUV420_8 :  cfg->m_inputFileChromaFormat = VVENC_CHROMA_420; cfg->m_inputBitDepth[0] = 8;  break;
  case YUV420_10 : cfg->m_inputFileChromaFormat = VVENC_CHROMA_420; cfg->m_inputBitDepth[0] = 10; break;
  case YUV422_8 :  cfg->m_inputFileChromaFormat = VVENC_CHROMA_422; cfg->m_inputBitDepth[0] = 8;  break;
  case YUV422_10 : cfg->m_inputFileChromaFormat = VVENC_CHROMA_422; cfg->m_inputBitDepth[0] = 10; break;
  case YUV444_8 :  cfg->m_inputFileChromaFormat = VVENC_CHROMA_444; cfg->m_inputBitDepth[0] = 8;  break;
  case YUV444_10 : cfg->m_inputFileChromaFormat = VVENC_CHROMA_444; cfg->m_inputBitDepth[0] = 10; break;
  case YUV400_8 :  cfg->m_inputFileChromaFormat = VVENC_CHROMA_400; cfg->m_inputBitDepth[0] = 8;  break;
  case YUV400_10 : cfg->m_inputFileChromaFormat = VVENC_CHROMA_400; cfg->m_inputBitDepth[0] = 10; break;
  default: break;
  }
}


// ====================================================================================================================
// Public member functions
// ====================================================================================================================  virtual ~VVEncAppCfg()
VVEncAppCfg::~VVEncAppCfg()
{
}

/** \param  argc        number of arguments
    \param  argv        array of arguments
    \retval             true when success
 */
bool VVEncAppCfg::parseCfg( int argc, char* argv[] )
{
  bool do_help                = false;
  bool do_full_help           = false;
  int  warnUnknowParameter    = 0;

  //
  // link custom formated configuration parameters with istream reader
  //
  IStreamToEnum<vvencMsgLevel>      toMsgLevel                   ( &m_verbosity,   &MsgLevelToEnumMap );
  IStreamToFunc<vvencPresetMode>    toPreset                     ( setPresets, this, &PresetToEnumMap,vvencPresetMode::VVENC_MEDIUM);
  IStreamToRefVec<int>              toSourceSize                 ( { &m_SourceWidth, &m_SourceHeight }, true, 'x' );

  IStreamToEnum<vvencProfile>       toProfile                    ( &m_profile,                     &ProfileToEnumMap      );
  IStreamToEnum<vvencTier>          toLevelTier                  ( &m_levelTier,                   &TierToEnumMap         );
  IStreamToEnum<vvencLevel>         toLevel                      ( &m_level,                       &LevelToEnumMap        );
  IStreamToEnum<vvencSegmentMode>   toSegment                    ( &m_SegmentMode,                 &SegmentToEnumMap      );
  IStreamToEnum<vvencHDRMode>       toHDRMode                    ( &m_HdrMode,                     &HdrModeToIntMap       );

  IStreamToFunc<BitDepthAndColorSpace> toInputFormatBitdepth( setInputBitDepthAndColorSpace, this, &BitColorSpaceToIntMap, YUV420_8);
  IStreamToEnum<vvencDecodingRefreshType>   toDecRefreshType     ( &m_DecodingRefreshType,         &DecodingRefreshTypeToEnumMap );

  IStreamToEnum<int>                toAud                        ( &m_AccessUnitDelimiter,             &FlagToIntMap );
  IStreamToEnum<int>                toHrd                        ( &m_hrdParametersPresent,            &FlagToIntMap );
  IStreamToEnum<int>                toVui                        ( &m_vuiParametersPresent,            &FlagToIntMap );
  IStreamToEnum<bool>               toQPA                        ( &m_usePerceptQPA,                   &QPAToIntMap );
  IStreamToArr<char>                toTraceRule                  ( &m_traceRule[0], VVENC_MAX_STRING_LEN  );
  IStreamToArr<char>                toTraceFile                  ( &m_traceFile[0], VVENC_MAX_STRING_LEN  );
  IStreamToEnum<vvencHashType>      toHashType                   ( &m_decodedPictureHashSEIType,   &HashTypeToEnumMap     );
  //
  // setup configuration parameters
  //

  std::string ignoreParams;
  po::Options opts;
  opts.setSubSection("General Options");
  opts.addOptions()
  ("help",              do_help,                  "this help text")
  ("fullhelp",          do_full_help,             "show full text")
  ("verbosity,v",       toMsgLevel,               "Specifies the level of the verboseness (0: silent, 1: error, 2: warning, 3: info, 4: notice, 5: verbose, 6: debug) ")
  ("version",           m_showVersion,            "show version ")
  ;

  opts.setSubSection("Input Options");
  opts.addOptions()
  ("input,i",           m_inputFileName,          "original YUV input file name or '-' for reading from stdin")
  ("size,s",            toSourceSize,             "specify input resolution (WidthxHeight)")
  ("format,c",          toInputFormatBitdepth,    "set input format (yuv420, yuv420_10)")

  ("framerate,r",       m_FrameRate,              "temporal rate (framerate) e.g. 25,29,30,50,59,60 ")
  ("tickspersec",       m_TicksPerSecond,         "Ticks Per Second for dts generation, (1..27000000)")

  ("frames,f",          m_framesToBeEncoded,      "max. frames to encode [all]")
  ("frameskip",         m_FrameSkip,              "Number of frames to skip at start of input YUV [off]")
  ("segment",           toSegment,                "when encoding multiple separate segments, specify segment position to enable segment concatenation (first, mid, last) [off]\n"
                                                  "first: first segment           \n"
                                                  "mid  : all segments between first and last segment\n"
                                                  "last : last segment")
  ;
  opts.setSubSection("Output Options");
  opts.addOptions()
  ("output,o",          m_bitstreamFileName,      "Bitstream output file name")
  ;
  opts.setSubSection("Encoder Options");
  opts.addOptions()
  ("preset",            toPreset,                 "select preset for specific encoding setting (faster, fast, medium, slow, slower)")

  ("bitrate,b",         m_RCTargetBitrate,        "bitrate for rate control (0: constant-QP encoding without rate control, otherwise bits/second)" )
  ("passes,p",          m_RCNumPasses,            "number of rate control passes (1,2)" )
  ("pass",              m_RCPass,                 "rate control pass for two-pass rate control (-1,1,2)" )
  ("rcstatsfile",       m_RCStatsFileName,        "rate control statistics file" )
  ("qp,q",              m_QP,                     "quantization parameter, QP (0-63)")
  ("qpa",               toQPA,                    "Enable perceptually motivated QP adaptation, XPSNR based (0:off, 1:on)", true)

  ("threads,-t",        m_numThreads,             "Number of threads default: [size < 720p: 4, >= 720p: 8]")

  ("gopsize,g",         m_GOPSize,                "GOP size of temporal structure (16,32)")
  ("refreshtype,-rt",   toDecRefreshType,         "intra refresh type (idr,cra,idr2)")
  ("refreshsec,-rs",    m_IntraPeriodSec,         "Intra period/refresh in seconds")
  ("intraperiod,-ip",   m_IntraPeriod,            "Intra period in frames (0: use intra period in seconds (refreshsec), else: n*gopsize)")
  ;

  opts.setSubSection("Profile, Level, Tier");
  opts.addOptions()
  ("profile",           toProfile,                "select profile (main10, main10_stillpic)")
  ("level",             toLevel,                  "Level limit (1.0, 2.0,2.1, 3.0,3.1, 4.0,4.1, 5.0,5.1,5.2, 6.0,6.1,6.2,6.3, 15.5)")
  ("tier",              toLevelTier,              "Tier to use for interpretation of level (main or high)")
  ;

  opts.setSubSection("HDR and Color Options");
  opts.addOptions()
  ("hdr",               toHDRMode,                "set HDR mode (+SEI messages) + BT.709 or BT.2020 color space. "
                                                  "use: off, pq|hdr10, pq_2020|hdr10_2020, hlg, hlg_2020")
   ;

  po::setDefaults( opts );
  std::ostringstream easyOpts;
  po::doHelp( easyOpts, opts );

  opts.setSubSection("Encoder Options");
  opts.addOptions()
  ("internal-bitdepth",         m_internalBitDepth[0], "internal bitdepth (8,10)")
  ("accessunitdelimiter,-aud",  toAud,                 "Emit Access Unit Delimiter NALUs  (auto(-1),off(0),on(1); default: auto - only if needed by dependent options)", true)
  ("vuiparameterspresent,-vui", toVui,                 "Emit VUI information (auto(-1),off(0),on(1); default: auto - only if needed by dependent options)", true)
  ("hrdparameterspresent,-hrd", toHrd,                 "Emit VUI HRD information (auto(-1),off(0),on(1); default: auto - only if needed by dependent options)",  true)
  ("decodedpicturehash,-dph",   toHashType,            "Control generation of decode picture hash SEI messages, (0:off, 1:md5, 2:crc, 3:checksum)")
  ;

  if ( vvenc_is_tracing_enabled() )
  {
    opts.setSubSection( "Tracing" );
    opts.addOptions()
    ("tracechannellist",              m_listTracingChannels,  "List all available tracing channels")
    ("tracerule",                     toTraceRule,            "Tracing rule (ex: \"D_CABAC:poc==8\" or \"D_REC_CB_LUMA:poc==8\")")
    ("tracefile",                     toTraceFile,            "Tracing file")
    ;
  }

  po::setDefaults( opts );
  std::ostringstream fullOpts;
  po::doHelp( fullOpts, opts );

  //
  // parse command line parameters and read configuration files
  //

  po::setDefaults( opts );
  po::ErrorReporter err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**) argv, err );
  for( auto& a : argv_unhandled )
  {
    cout << "Unhandled argument ignored: `" << a << "'\n";
  }

  if ( argc == 1 || do_help )
  {
    cout <<  easyOpts.str();
    return false;
  }
  else if ( do_full_help )
  {
    cout << fullOpts.str();
    return false;
  }

  if ( err.is_errored && ! warnUnknowParameter )
  {
    // error report has already been printed
    return false;
  }

  if( m_showVersion )
  {
    return true;
  }

  // has to be set outside
  if ( m_internChromaFormat < 0 || m_internChromaFormat >= VVENC_NUM_CHROMA_FORMAT )
  {
    m_internChromaFormat = m_inputFileChromaFormat;
  }

  return checkCfg();
}


/** \param  argc        number of arguments
    \param  argv        array of arguments
    \retval             true when success
 */
bool VVEncAppCfg::parseCfgFF( int argc, char* argv[] )
{
  bool do_help                = false;
  bool do_expert_help         = false;
  int  warnUnknowParameter    = 0;

  std::string writeCfg = "";
  //
  // link custom formated configuration parameters with istream reader
  //
  IStreamToEnum<vvencMsgLevel>      toMsgLevel                   ( &m_verbosity,                   &MsgLevelToEnumMap      );
  IStreamToFunc<vvencPresetMode>    toPreset                     ( setPresets, this, &PresetToEnumMap,vvencPresetMode::VVENC_MEDIUM);
  IStreamToRefVec<int>              toSourceSize                 ( { &m_SourceWidth, &m_SourceHeight }, true, 'x' );
  IStreamToRefVec<double>           toLambdaModifier             ( { &m_adLambdaModifier[0], &m_adLambdaModifier[1], &m_adLambdaModifier[2], &m_adLambdaModifier[3], &m_adLambdaModifier[4], &m_adLambdaModifier[5], &m_adLambdaModifier[6] }, false );

  IStreamToEnum<vvencProfile>       toProfile                    ( &m_profile,                     &ProfileToEnumMap      );
  IStreamToEnum<vvencTier>          toLevelTier                  ( &m_levelTier,                   &TierToEnumMap         );
  IStreamToEnum<vvencLevel>         toLevel                      ( &m_level,                       &LevelToEnumMap        );
  IStreamToEnum<vvencCostMode>      toCostMode                   ( &m_costMode,                    &CostModeToEnumMap     );
  IStreamToEnum<vvencChromaFormat>  toInputFileCoFormat          ( &m_inputFileChromaFormat,       &ChromaFormatToEnumMap  );
  IStreamToEnum<vvencChromaFormat>  toInternCoFormat             ( &m_internChromaFormat,          &ChromaFormatToEnumMap  );
  IStreamToEnum<vvencHashType>      toHashType                   ( &m_decodedPictureHashSEIType,   &HashTypeToEnumMap     );
  IStreamToArr<int>                 toQpInCb                     ( &m_qpInValsCb[0], VVENC_MAX_QP_VALS_CHROMA            );
  IStreamToArr<int>                 toQpOutCb                    ( &m_qpOutValsCb[0], VVENC_MAX_QP_VALS_CHROMA           );
  IStreamToArr<int>                 toQpInCr                     ( &m_qpInValsCr[0], VVENC_MAX_QP_VALS_CHROMA            );
  IStreamToArr<int>                 toQpOutCr                    ( &m_qpOutValsCr[0], VVENC_MAX_QP_VALS_CHROMA           );
  IStreamToArr<int>                 toQpInCbCr                   ( &m_qpInValsCbCr[0], VVENC_MAX_QP_VALS_CHROMA          );
  IStreamToArr<int>                 toQpOutCbCr                  ( &m_qpOutValsCbCr[0], VVENC_MAX_QP_VALS_CHROMA         );
  IStreamToArr<double>              toIntraLambdaModifier        ( &m_adIntraLambdaModifier[0], VVENC_MAX_TLAYER );

  IStreamToArr<int>                 toMCTFFrames                 ( &m_vvencMCTF.MCTFFrames[0], VVENC_MAX_MCTF_FRAMES   );
  IStreamToArr<double>              toMCTFStrengths              ( &m_vvencMCTF.MCTFStrengths[0], VVENC_MAX_MCTF_FRAMES);
  IStreamToEnum<vvencSegmentMode>   toSegment                    ( &m_SegmentMode,            &SegmentToEnumMap );
  IStreamToEnum<vvencHDRMode>       toHDRMode                    ( &m_HdrMode,                &HdrModeToIntMap       );
  IStreamToEnum<int>                toColorPrimaries             ( &m_colourPrimaries,        &ColorPrimariesToIntMap );
  IStreamToEnum<int>                toTransferCharacteristics    ( &m_transferCharacteristics,&TransferCharacteristicsToIntMap );
  IStreamToEnum<int>                toColorMatrix                ( &m_matrixCoefficients,     &ColorMatrixToIntMap );
  IStreamToEnum<int>                toPrefTransferCharacteristics( &m_preferredTransferCharacteristics, &TransferCharacteristicsToIntMap );

  IStreamToArr<unsigned int>        toMasteringDisplay           ( &m_masteringDisplay[0], 10  );
  IStreamToArr<unsigned int>        toContentLightLevel          ( &m_contentLightLevel[0], 2 );

  IStreamToEnum<vvencDecodingRefreshType> toDecRefreshType       ( &m_DecodingRefreshType, &DecodingRefreshTypeToEnumMap );

  IStreamToEnum<int>                toAud                        ( &m_AccessUnitDelimiter,             &FlagToIntMap );
  IStreamToEnum<int>                toHrd                        ( &m_hrdParametersPresent,            &FlagToIntMap );
  IStreamToEnum<int>                toVui                        ( &m_vuiParametersPresent,            &FlagToIntMap );

  IStreamToArr<char>                toTraceRule                   ( &m_traceRule[0], VVENC_MAX_STRING_LEN  );
  IStreamToArr<char>                toTraceFile                   ( &m_traceFile[0], VVENC_MAX_STRING_LEN  );
  IStreamToArr<char>                toDecodeBitstreams0           ( &m_decodeBitstreams[0][0], VVENC_MAX_STRING_LEN  );
  IStreamToArr<char>                toDecodeBitstreams1           ( &m_decodeBitstreams[1][0], VVENC_MAX_STRING_LEN  );
  IStreamToArr<char>                toSummaryOutFilename          ( &m_summaryOutFilename[0], VVENC_MAX_STRING_LEN  );
  IStreamToArr<char>                toSummaryPicFilenameBase      ( &m_summaryPicFilenameBase[0], VVENC_MAX_STRING_LEN  );

  //
  // setup configuration parameters
  //
  std::string ignoreParams;
  po::Options opts;

  opts.setSubSection("General options");
  opts.addOptions()
  ("help",                                            do_help,                                          "this help text")
  ("fullhelp",                                        do_expert_help,                                   "expert help text")
  ("Verbosity,v",                                     toMsgLevel,                                       "Specifies the level of the verboseness (0: silent, 1: error, 2: warning, 3: info, 4: notice, 5: verbose, 6: debug)")
  ("version",                                         m_showVersion,                                    "show version ")
  ;

  opts.setSubSection("Input options");
  opts.addOptions()
  ("InputFile,i",                                     m_inputFileName,                                  "Original YUV input file name or '-' for reading from stdin")
  ("Size,s",                                          toSourceSize,                                     "Input resolution (WidthxHeight)")
  ("InputBitDepth",                                   m_inputBitDepth[ 0 ],                             "Bit-depth of input file")
  ("FramesToBeEncoded,f",                             m_framesToBeEncoded,                              "Number of frames to be encoded (default=all)")
  ("FrameRate,-fr",                                   m_FrameRate,                                      "Frame rate")
  ("FrameSkip,-fs",                                   m_FrameSkip,                                      "Number of frames to skip at start of input YUV")
  ("TicksPerSecond",                                  m_TicksPerSecond,                                 "Ticks Per Second for dts generation, ( 1..27000000)")

  ("segment",                                         toSegment,                                        "when encoding multiple separate segments, specify segment position to enable segment concatenation (first, mid, last) [off]\n"
                                                                                                        "first: first segment           \n"
                                                                                                        "mid  : all segments between first and last segment\n"
                                                                                                        "last : last segment")
  ;

  opts.setSubSection("Output options");
  opts.addOptions()
  ("BitstreamFile,b",                                 m_bitstreamFileName,                              "Bitstream output file name")
  ("ReconFile,o",                                     m_reconFileName,                                  "Reconstructed YUV output file name")
  ("OutputBitDepth",                                  m_outputBitDepth[ 0 ],                            "Bit-depth of output file")
  ;

  opts.setSubSection("Profile, Level, Tier");
  opts.addOptions()
  ("Profile",                                         toProfile,                                        "Profile name to use for encoding. Use [multilayer_]main_10[_444][_still_picture], auto, or none")
  ("Tier",                                            toLevelTier,                                      "Tier to use for interpretation of level (main or high)")
  ("Level",                                           toLevel,                                          "Level limit to be used, eg 5.1, or none")
  ;

  opts.setSubSection("Threading, performance");
  opts.addOptions()
  ("Threads,t",                                       m_numThreads,                                     "Number of threads")
  ("preset",                                          toPreset,                                         "select preset for specific encoding setting (faster, fast, medium, slow, slower)")
  ;

  opts.setSubSection("Slice decision options");
  opts.addOptions()
  ("IntraPeriod,-ip",                                 m_IntraPeriod,                                    "Intra period in frames, (-1: only first frame)")
  ("RefreshSec,-rs",                                  m_IntraPeriodSec,                                 "Intra period in seconds")
  ("DecodingRefreshType,-dr",                         toDecRefreshType,                                 "Intra refresh type (0:none, 1:CRA, 2:IDR, 3:IDR2, 4:RecPointSEI)")
  ("GOPSize,g",                                       m_GOPSize,                                        "GOP size of temporal structure")
  ;

  opts.setSubSection("Rate control, Perceptual Quantization");
  opts.addOptions()
  ("NumPasses",                                       m_RCNumPasses,                                    "number of rate control passes (1,2)" )
  ("Passes",                                          m_RCNumPasses,                                    "number of rate control passes (1,2)" )
  ("Pass",                                            m_RCPass,                                         "rate control pass for two-pass rate control (-1,1,2)" )
  ("RCStatsFile",                                     m_RCStatsFileName,                                "rate control statistics file" )
  ("TargetBitrate",                                   m_RCTargetBitrate,                                "Rate control: target bit-rate [bps]" )

  ("PerceptQPA,-qpa",                                 m_usePerceptQPA,                                  "Enable perceptually motivated QP adaptation, XPSNR based (0:off, 1:on)", true)
  ;


  opts.setSubSection("VUI and SEI options");
  opts.addOptions()
  ("Hdr",                                             toHDRMode,                                        "set HDR mode (+SEI messages) + BT.709 or BT.2020 color space. "
                                                                                                        "If maxcll or masteringdisplay is set, HDR10/PQ is enabled. use: off, pq|hdr10, pq_2020|hdr10_2020, hlg, hlg_2020")
   ;

  po::setDefaults( opts );
  std::ostringstream easyOpts;
  po::doHelp( easyOpts, opts );

  opts.setSubSection("General options");
  opts.addOptions()
  ("c",                                               po::parseConfigFile,                              "configuration file name")
  ("WriteConfig",                                     writeCfg,                                         "write the encoder config into configuration file")
  ("WarnUnknowParameter,w",                           warnUnknowParameter,                              "warn for unknown configuration parameters instead of failing")
  ("SIMD",                                            ignoreParams,                                     "SIMD extension to use (SCALAR, SSE41, SSE42, AVX, AVX2, AVX512), default: the highest supported extension")
  ;

    if ( vvenc_is_tracing_enabled() )
  {
    opts.setSubSection("Tracing");
    opts.addOptions()
    ("TraceChannelsList",                             m_listTracingChannels,                            "List all available tracing channels")
    ("TraceRule",                                     toTraceRule,                                      "Tracing rule (ex: \"D_CABAC:poc==8\" or \"D_REC_CB_LUMA:poc==8\")")
    ("TraceFile",                                     toTraceFile,                                      "Tracing file")
    ;
  }

  // file, i/o and source parameters
  opts.setSubSection("Input options");
  opts.addOptions()
  ("SourceWidth",                                     m_SourceWidth,                                    "Source picture width")
  ("SourceHeight",                                    m_SourceHeight,                                   "Source picture height")

  ("ConformanceWindowMode",                           m_conformanceWindowMode,                          "Window conformance mode (0:off, 1:automatic padding, 2:padding, 3:conformance")
  ("ConfWinLeft",                                     m_confWinLeft,                                    "Left offset for window conformance mode 3")
  ("ConfWinRight",                                    m_confWinRight,                                   "Right offset for window conformance mode 3")
  ("ConfWinTop",                                      m_confWinTop,                                     "Top offset for window conformance mode 3")
  ("ConfWinBottom",                                   m_confWinBottom,                                  "Bottom offset for window conformance mode 3")

  ("TemporalSubsampleRatio",                          m_temporalSubsampleRatio,                         "Temporal sub-sample ratio when reading input YUV")

  ("HorizontalPadding",                               m_aiPad[0],                                       "Horizontal source padding for conformance window mode 2")
  ("VerticalPadding",                                 m_aiPad[1],                                       "Vertical source padding for conformance window mode 2")

  ("InputChromaFormat",                               toInputFileCoFormat,                              "input file chroma format (400, 420, 422, 444)")
  ;

  opts.setSubSection("Quality reporting metrics");
  opts.addOptions()
  ("MSEBasedSequencePSNR",                            m_printMSEBasedSequencePSNR,                      "Emit sequence PSNR (0: only as a linear average of the frame PSNRs, 1: also based on an average of the frame MSEs")
  ("PrintHexPSNR",                                    m_printHexPsnr,                                   "Emit hexadecimal PSNR for each frame (0: off , 1:on")
  ("PrintFrameMSE",                                   m_printFrameMSE,                                  "Emit MSE values for each frame (0: off , 1:on")
  ("PrintSequenceMSE",                                m_printSequenceMSE,                               "Emit MSE values for the whole sequence (0: off , 1:on)")
  ;

  opts.setSubSection("Bitstream options");
  opts.addOptions()
  ("CabacZeroWordPaddingEnabled",                     m_cabacZeroWordPaddingEnabled,                    "Add conforming cabac-zero-words to bit streams (0: do not add, 1: add as required)")
  ;

  // Profile and level
  opts.setSubSection("Profile, Level, Tier");
  opts.addOptions()
  ("SubProfile",                                      m_subProfile,                                     "Sub-profile idc")
  ("MaxBitDepthConstraint",                           m_bitDepthConstraintValue,                        "Bit depth to use for profile-constraint for RExt profiles. (0: automatically choose based upon other parameters)")
  ("IntraConstraintFlag",                             m_intraOnlyConstraintFlag,                        "Value of general_intra_constraint_flag to use for RExt profiles (not used if an explicit RExt sub-profile is specified)")
  ;

  opts.setSubSection("Rate control, Perceptual Quantization");
  opts.addOptions()
  ("RCInitialQP",                                     m_RCInitialQP,                                    "Rate control: initial QP. With two-pass encoding, this specifies the first-pass base QP (instead of using a default QP). Activated if value is greater than zero" )
  ("RCForceIntraQP",                                  m_RCForceIntraQP,                                 "Rate control: force intra QP to be equal to initial QP" )

  ("PerceptQPATempFiltIPic",                          m_usePerceptQPATempFiltISlice,                    "Temporal high-pass filter in QPA activity calculation for key pictures (0:off, 1:on, 2:on incl. temporal pumping reduction, -1:auto)")
  ;

  // Coding structure paramters
  opts.setSubSection("Coding structure paramters");
  opts.addOptions()
  ("InputQueueSize",                                  m_InputQueueSize,                                 "Size of input frames queue (use gop size)")
  ("ReWriteParamSets",                                m_rewriteParamSets,                               "Enable rewriting of Parameter sets before every (intra) random access point")
  ("IDRRefParamList",                                 m_idrRefParamList,                                "Enable indication of reference picture list syntax elements in slice headers of IDR pictures")
  ;

  /* Quantization parameters */
  opts.setSubSection("Quantization paramters");
  opts.addOptions()
  ("QP,q",                                            m_QP,                                             "Qp value")
  ("SameCQPTablesForAllChroma",                       m_useSameChromaQPTables,                          "0: Different tables for Cb, Cr and joint Cb-Cr components, 1 (default): Same tables for all three chroma components")
  ("IntraQPOffset",                                   m_intraQPOffset,                                  "Qp offset value for intra slice, typically determined based on GOP size")
  ("LambdaFromQpEnable",                              m_lambdaFromQPEnable,                             "Enable flag for derivation of lambda from QP")
  ("LambdaModifier",                                  toLambdaModifier,                                 "Lambda modifier list for temporal layers. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifierI",                                 toIntraLambdaModifier,                            "Lambda modifiers for Intra pictures, comma separated, up to one the number of temporal layer. If entry for temporalLayer exists, then use it, else if some are specified, use the last, else use the standard LambdaModifiers.")
  ("IQPFactor",                                       m_dIntraQpFactor,                                 "Intra QP Factor for Lambda Computation. If negative, the default will scale lambda based on GOP size (unless LambdaFromQpEnable then IntraQPOffset is used instead)")
  ("QpInValCb",                                       toQpInCb,                                         "Input coordinates for the QP table for Cb component")
  ("QpInValCr",                                       toQpInCr,                                         "Input coordinates for the QP table for Cr component")
  ("QpInValCbCr",                                     toQpInCbCr,                                       "Input coordinates for the QP table for joint Cb-Cr component")
  ("QpOutValCb",                                      toQpOutCb,                                        "Output coordinates for the QP table for Cb component")
  ("QpOutValCr",                                      toQpOutCr,                                        "Output coordinates for the QP table for Cr component")
  ("QpOutValCbCr",                                    toQpOutCbCr,                                      "Output coordinates for the QP table for joint Cb-Cr component")
  ("MaxCuDQPSubdiv",                                  m_cuQpDeltaSubdiv,                                "Maximum subdiv for CU luma Qp adjustment")
  ("MaxCuChromaQpOffsetSubdiv",                       m_cuChromaQpOffsetSubdiv,                         "Maximum subdiv for CU chroma Qp adjustment - set less than 0 to disable")
  ("CbQpOffset",                                      m_chromaCbQpOffset,                               "Chroma Cb QP Offset")
  ("CrQpOffset",                                      m_chromaCrQpOffset,                               "Chroma Cr QP Offset")
  ("CbQpOffsetDualTree",                              m_chromaCbQpOffsetDualTree,                       "Chroma Cb QP Offset for dual tree")
  ("CrQpOffsetDualTree",                              m_chromaCrQpOffsetDualTree,                       "Chroma Cr QP Offset for dual tree")
  ("CbCrQpOffset",                                    m_chromaCbCrQpOffset,                             "QP Offset for joint Cb-Cr mode")
  ("CbCrQpOffsetDualTree",                            m_chromaCbCrQpOffsetDualTree,                     "QP Offset for joint Cb-Cr mode in dual tree")
  ("SliceChromaQPOffsetPeriodicity",                  m_sliceChromaQpOffsetPeriodicity,                 "Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.")
  ("SliceCbQpOffsetIntraOrPeriodic",                  m_sliceChromaQpOffsetIntraOrPeriodic[0],          "Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")
  ("SliceCrQpOffsetIntraOrPeriodic",                  m_sliceChromaQpOffsetIntraOrPeriodic[1],          "Chroma Cr QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")

  ("LumaLevelToDeltaQPMode",                          m_lumaLevelToDeltaQPEnabled,                      "Luma based Delta QP 0(default): not used. 1: Based on CTU average")
  ("WCGPPSEnable",                                    m_wcgChromaQpControl.enabled,                     "1: Enable the WCG PPS chroma modulation scheme. 0 (default) disabled")
  ("WCGPPSCbQpScale",                                 m_wcgChromaQpControl.chromaCbQpScale,             "WCG PPS Chroma Cb QP Scale")
  ("WCGPPSCrQpScale",                                 m_wcgChromaQpControl.chromaCrQpScale,             "WCG PPS Chroma Cr QP Scale")
  ("WCGPPSChromaQpScale",                             m_wcgChromaQpControl.chromaQpScale,               "WCG PPS Chroma QP Scale")
  ("WCGPPSChromaQpOffset",                            m_wcgChromaQpControl.chromaQpOffset,              "WCG PPS Chroma QP Offset")
  ;

  opts.setSubSection("Misc. options");
  opts.addOptions()
  ("ChromaFormatIDC,-cf",                             toInternCoFormat,                                 "intern chroma format (400, 420, 422, 444) or set to 0 (default), same as InputChromaFormat")
  ("UseIdentityTableForNon420Chroma",                 m_useIdentityTableForNon420Chroma,                "True: Indicates that 422/444 chroma uses identity chroma QP mapping tables; False: explicit Qp table may be specified in config")
  ("InputBitDepthC",                                  m_inputBitDepth[ 1 ],                             "As per InputBitDepth but for chroma component. (default:InputBitDepth)")
  ("InternalBitDepth",                                m_internalBitDepth[ 0 ],                          "Bit-depth the codec operates at. (default: MSBExtendedBitDepth). If different to MSBExtendedBitDepth, source data will be converted")
  ("OutputBitDepthC",                                 m_outputBitDepth[ 1 ],                            "As per OutputBitDepth but for chroma component. (default: use luma output bit-depth)")
  ("MSBExtendedBitDepth",                             m_MSBExtendedBitDepth[ 0 ],                       "bit depth of luma component after addition of MSBs of value 0 (used for synthesising High Dynamic Range source material). (default:InputBitDepth)")
  ("MSBExtendedBitDepthC",                            m_MSBExtendedBitDepth[ 1 ],                       "As per MSBExtendedBitDepth but for chroma component. (default:MSBExtendedBitDepth)")

  ("WaveFrontSynchro",                                m_entropyCodingSyncEnabled,                       "Enable entropy coding sync")
  ("EntryPointsPresent",                              m_entryPointsPresent,                             "Enable entry points in slice header")

  ;

  opts.setSubSection("Quad-Tree size and depth");
  opts.addOptions()
  ("CTUSize",                                         m_CTUSize,                                        "CTUSize")
  ("MinQTISlice",                                     m_MinQT[0],                                       "MinQTISlice")
  ("MinQTLumaISlice",                                 m_MinQT[0],                                       "MinQTLumaISlice")
  ("MinQTNonISlice",                                  m_MinQT[1],                                       "MinQTNonISlice")
  ("MinQTChromaISliceInChromaSamples",                m_MinQT[2],                                       "MinQTChromaISlice")
  ("MaxMTTDepth",                                     m_maxMTTDepth,                                    "maxMTTDepth")
  ("MaxMTTDepthI",                                    m_maxMTTDepthI,                                   "maxMTTDepthI")
  ("MaxMTTDepthISliceL",                              m_maxMTTDepthI,                                   "maxMTTDepthISliceL")
  ("MaxMTTDepthISliceC",                              m_maxMTTDepthIChroma,                             "maxMTTDepthISliceC")
  // --> deprecated
  ("MaxMTTHierarchyDepth",                            m_maxMTTDepth,                                    "maxMTTDepth")
  ("MaxMTTHierarchyDepthI",                           m_maxMTTDepthI,                                   "maxMTTDepthI")
  ("MaxMTTHierarchyDepthISliceL",                     m_maxMTTDepthI,                                   "maxMTTDepthISliceL")
  ("MaxMTTHierarchyDepthISliceC",                     m_maxMTTDepthIChroma,                             "maxMTTDepthISliceC")
  // <-- deprecated
  ("MaxBTLumaISlice",                                 m_maxBT[0],                                       "MaxBTLumaISlice")
  ("MaxBTChromaISlice",                               m_maxBT[2],                                       "MaxBTChromaISlice")
  ("MaxBTNonISlice",                                  m_maxBT[1],                                       "MaxBTNonISlice")
  ("MaxTTLumaISlice",                                 m_maxTT[0],                                       "MaxTTLumaISlice")
  ("MaxTTChromaISlice",                               m_maxTT[2],                                       "MaxTTChromaISlice")
  ("MaxTTNonISlice",                                  m_maxTT[1],                                       "MaxTTNonISlice")
  ("DualITree",                                       m_dualITree,                                      "Use separate luma and chroma QTBT trees for intra slice")
  ("Log2MaxTbSize",                                   m_log2MaxTbSize,                                  "Maximum transform block size in logarithm base 2")
  ("Log2MinCodingBlockSize",                          m_log2MinCodingBlockSize,                         "Minimum coding block size in logarithm base 2")
  ;

  opts.setSubSection("Coding tools");
  opts.addOptions()
  ("CostMode",                                        toCostMode,                                       "Use alternative cost functions: choose between 'lossy', 'sequence_level_lossless', 'lossless' (which forces QP to " MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP) ") and 'mixed_lossless_lossy' (which used QP'=" MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME) " for pre-estimates of transquant-bypass blocks).")
  ("ASR",                                             m_bUseASR,                                        "Adaptive motion search range")
  ("HadamardME",                                      m_bUseHADME,                                      "Hadamard ME for fractional-pel")
  ("RDOQ",                                            m_RDOQ,                                           "Rate-Distortion Optimized Quantization mode")
  ("RDOQTS",                                          m_useRDOQTS,                                      "Rate-Distortion Optimized Quantization mode for TransformSkip")
  ("SelectiveRDOQ",                                   m_useSelectiveRDOQ,                               "Enable selective RDOQ")

  ("JointCbCr",                                       m_JointCbCrMode,                                  "Enable joint coding of chroma residuals (0:off, 1:on)")
  ("CabacInitPresent",                                m_cabacInitPresent,                               "Enable cabac table index selection based on previous frame")
  ("LCTUFast",                                        m_useFastLCTU,                                    "Fast methods for large CTU")
  ("PBIntraFast",                                     m_usePbIntraFast,                                 "Fast assertion if the intra mode is probable")
  ("FastMrg",                                         m_useFastMrg,                                     "Fast methods for inter merge")
  ("AMaxBT",                                          m_useAMaxBT,                                      "Adaptive maximal BT-size")
  ("FastQtBtEnc",                                     m_fastQtBtEnc,                                    "Fast encoding setting for QTBT")
  ("ContentBasedFastQtbt",                            m_contentBasedFastQtbt,                           "Signal based QTBT speed-up")
  ("FEN",                                             m_fastInterSearchMode,                            "fast encoder setting")
  ("ECU",                                             m_useEarlyCU,                                     "Early CU setting (1: ECU limited to specific block size and TL, 2: unconstrained ECU)")
  ("FDM",                                             m_useFastDecisionForMerge,                        "Fast decision for Merge RD Cost")

  ("DisableIntraInInter",                             m_bDisableIntraCUsInInterSlices,                  "Flag to disable intra CUs in inter slices")
  ("ConstrainedIntraPred",                            m_bUseConstrainedIntraPred,                       "Constrained Intra Prediction")
  ("FastUDIUseMPMEnabled",                            m_bFastUDIUseMPMEnabled,                          "If enabled, adapt intra direction search, accounting for MPM")
  ("FastMEForGenBLowDelayEnabled",                    m_bFastMEForGenBLowDelayEnabled,                  "If enabled use a fast ME for generalised B Low Delay slices")

  ("MTSImplicit",                                     m_MTSImplicit,                                    "Enable implicit MTS when explicit MTS is off\n")
  ("TMVPMode",                                        m_TMVPModeId,                                     "TMVP mode enable(0: off 1: for all slices 2: for certain slices only)")
  ("DepQuant",                                        m_DepQuantEnabled,                                "Enable dependent quantization" )
  ("QuantThrVal",                                     m_quantThresholdVal,                              "Quantization threshold value for DQ last coefficient search" )
  ("SignHideFlag",                                    m_SignDataHidingEnabled,                          "Enable sign data hiding" )
  ("MIP",                                             m_MIP,                                            "Enable MIP (matrix-based intra prediction)")
  ("FastMIP",                                         m_useFastMIP,                                     "Fast encoder search for MIP (matrix-based intra prediction)")
  ("MaxNumMergeCand",                                 m_maxNumMergeCand,                                "Maximum number of merge candidates")
  ("MaxNumAffineMergeCand",                           m_maxNumAffineMergeCand,                          "Maximum number of affine merge candidates")
  ("Geo",                                             m_Geo,                                            "Enable geometric partitioning mode (0:off, 1:on)")
  ("MaxNumGeoCand",                                   m_maxNumGeoCand,                                  "Maximum number of geometric partitioning mode candidates")
  ("FastIntraTools",                                  m_FastIntraTools,                                 "SpeedUPIntraTools:LFNST,ISP,MTS. (0:off, 1:speed1, 2:speed2)")
  ("IntraEstDecBit",                                  m_IntraEstDecBit,                                 "Intra estimation decimation binary exponent for first pass directional modes screening (only test each (2^N)-th mode in the first estimation pass)")
  ;

  // motion search options
  opts.setSubSection("Motion search options");
  opts.addOptions()
  ("FastSearch",                                      m_motionEstimationSearchMethod,                   "Search mode (0:Full search 1:Diamond 2:Selective 3:Enhanced Diamond 4: FastDiamond)")
  ("FastSearchSCC",                                   m_motionEstimationSearchMethodSCC,                "Search mode for SCC (0:use non SCC-search 1:Selective 2:DiamondSCC 3:FastDiamondSCC)")
  ("RestrictMESampling",                              m_bRestrictMESampling,                            "Enable restrict ME Sampling for selective inter motion search")
  ("SearchRange,-sr",                                 m_SearchRange,                                    "Motion search range")
  ("BipredSearchRange",                               m_bipredSearchRange,                              "Motion search range for bipred refinement")
  ("MinSearchWindow",                                 m_minSearchWindow,                                "Minimum motion search window size for the adaptive window ME")
  ("ClipForBiPredMEEnabled",                          m_bClipForBiPredMeEnabled,                        "Enable clipping in the Bi-Pred ME.")
  ("FastMEAssumingSmootherMVEnabled",                 m_bFastMEAssumingSmootherMVEnabled,               "Enable fast ME assuming a smoother MV.")
  ("IntegerET",                                       m_bIntegerET,                                     "Enable early termination for integer motion search")
  ("FastSubPel",                                      m_fastSubPel,                                     "Enable fast sub-pel ME (1: enable fast sub-pel ME, 2: completely disable sub-pel ME)")
  ;

  // Deblocking filter parameters
  opts.setSubSection("Loop filters (deblock and SAO)");
  opts.addOptions()
  ("LoopFilterDisable",                               m_bLoopFilterDisable,                             "")
  ("LoopFilterOffsetInPPS",                           m_loopFilterOffsetInPPS,                          "")
  ("LoopFilterBetaOffset_div2",                       m_loopFilterBetaOffsetDiv2[0],                    "")
  ("LoopFilterTcOffset_div2",                         m_loopFilterTcOffsetDiv2[0],                      "")
  ("LoopFilterCbBetaOffset_div2",                     m_loopFilterBetaOffsetDiv2[1],                    "")
  ("LoopFilterCbTcOffset_div2",                       m_loopFilterTcOffsetDiv2[1],                      "")
  ("LoopFilterCrBetaOffset_div2",                     m_loopFilterBetaOffsetDiv2[2],                    "")
  ("LoopFilterCrTcOffset_div2",                       m_loopFilterTcOffsetDiv2[2],                      "")
  ("DeblockingFilterMetric",                          m_deblockingFilterMetric,                         "")

  ("LFCrossTileBoundaryFlag",                         m_bLFCrossTileBoundaryFlag,                       "Enable cross-tile-boundary loop filtering")
  ("LFCrossSliceBoundaryFlag",                        m_bLFCrossSliceBoundaryFlag,                      "Enable cross-slice-boundary loop filtering")
  ("LoopFilterAcrossTileGroupsEnabled",               m_loopFilterAcrossSlicesEnabled,                  "Enable loop filtering across tile groups")

  ("SAO",                                             m_bUseSAO,                                        "Enable Sample Adaptive Offset")
  ("SaoEncodingRate",                                 m_saoEncodingRate,                                "When >0 SAO early picture termination is enabled for luma and chroma")
  ("SaoEncodingRateChroma",                           m_saoEncodingRateChroma,                          "The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma")
  ("SaoLumaOffsetBitShift",                           m_saoOffsetBitShift[ 0 ],                         "Specify the luma SAO bit-shift. If negative, automatically calculate a suitable value based upon bit depth and initial QP")
  ("SaoChromaOffsetBitShift",                         m_saoOffsetBitShift[ 1 ],                         "Specify the chroma SAO bit-shift. If negative, automatically calculate a suitable value based upon bit depth and initial QP")
  ;

  opts.setSubSection("VUI and SEI options");
  opts.addOptions()

  ("SEIDecodedPictureHash,-dph",                      toHashType,                                       "Control generation of decode picture hash SEI messages, (0:off, 1:md5, 2:crc, 3:checksum)" )
  ("SEIBufferingPeriod",                              m_bufferingPeriodSEIEnabled,                      "Control generation of buffering period SEI messages")
  ("SEIPictureTiming",                                m_pictureTimingSEIEnabled,                        "Control generation of picture timing SEI messages")
  ("SEIDecodingUnitInfo",                             m_decodingUnitInfoSEIEnabled,                     "Control generation of decoding unit information SEI message.")
  ("EnableDecodingParameterSet",                      m_decodingParameterSetEnabled,                    "Enable writing of Decoding Parameter Set")
  ("AccessUnitDelimiter,-aud",                        toAud,                                            "Enable Access Unit Delimiter NALUs, (default: auto - enable only if needed by dependent options)" , true)
  ("VuiParametersPresent,-vui",                       toVui,                                            "Enable generation of vui_parameters(), (default: auto - enable only if needed by dependent options)", true)
  ("HrdParametersPresent,-hrd",                       toHrd,                                            "Enable generation of hrd_parameters(), (default: auto - enable only if needed by dependent options)", true)
  ("AspectRatioInfoPresent",                          m_aspectRatioInfoPresent,                         "Signals whether aspect_ratio_idc is present")
  ("AspectRatioIdc",                                  m_aspectRatioIdc,                                 "aspect_ratio_idc")
  ("SarWidth",                                        m_sarWidth,                                       "horizontal size of the sample aspect ratio")
  ("SarHeight",                                       m_sarHeight,                                      "vertical size of the sample aspect ratio")
  ("ColourDescriptionPresent",                        m_colourDescriptionPresent,                       "Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present")
  ("ColourPrimaries",                                 toColorPrimaries,                                 "Specify color primaries (0-13): reserved, bt709, unknown, empty, bt470m, bt470bg, smpte170m, "
                                                                                                        "smpte240m, film, bt2020, smpte428, smpte431, smpte432")

  ("TransferCharacteristics",                         toTransferCharacteristics,                        "Specify opto-electroni transfer characteristics (0-18): reserved, bt709, unknown, empty, bt470m, bt470bg, smpte170m, "
                                                                                                        "smpte240m, linear, log100, log316, iec61966-2-4, bt1361e, iec61966-2-1, "
                                                                                                        "bt2020-10, bt2020-12, smpte2084, smpte428, arib-std-b67")
  ("MatrixCoefficients",                              toColorMatrix,                                    "Specify color matrix setting to derive luma/chroma from RGB primaries (0-14): gbr, bt709, unknown, empty, fcc, bt470bg, smpte170m, "
                                                                                                        "smpte240m, ycgco, bt2020nc, bt2020c, smpte2085, chroma-derived-nc, chroma-derived-c, ictcp")

  ("ChromaLocInfoPresent",                            m_chromaLocInfoPresent,                           "Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present")
  ("ChromaSampleLocTypeTopField",                     m_chromaSampleLocTypeTopField,                    "Specifies the location of chroma samples for top field")
  ("ChromaSampleLocTypeBottomField",                  m_chromaSampleLocTypeBottomField,                 "Specifies the location of chroma samples for bottom field")
  ("ChromaSampleLocType",                             m_chromaSampleLocType,                            "Specifies the location of chroma samples for progressive content")
  ("OverscanInfoPresent",                             m_overscanInfoPresent,                            "Indicates whether conformant decoded pictures are suitable for display using overscan")
  ("OverscanAppropriate",                             m_overscanAppropriateFlag,                        "Indicates whether conformant decoded pictures are suitable for display using overscan")
  ("VideoSignalTypePresent",                          m_videoSignalTypePresent,                         "Signals whether video_format, video_full_range_flag, and colour_description_present_flag are present")
  ("VideoFullRange",                                  m_videoFullRangeFlag,                             "Indicates the black level and range of luma and chroma signals")


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
  ("SummaryVerboseness",                              m_summaryVerboseness,                             "Specifies the level of the verboseness of the text output")
  ;

  opts.setSubSection("Decoding options (debugging)");
  opts.addOptions()
  ("DebugBitstream",                                  toDecodeBitstreams0,                              "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DecodeBitstream1",                                toDecodeBitstreams0,                              "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DecodeBitstream2",                                toDecodeBitstreams1,                              "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DebugPOC",                                        m_switchPOC,                                      "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
  ("SwitchPOC",                                       m_switchPOC,                                      "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
  ("SwitchDQP",                                       m_switchDQP,                                      "delta QP applied to picture with switchPOC and subsequent pictures." )
  ("FastForwardToPOC",                                m_fastForwardToPOC,                               "Get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC." )
  ("StopAfterFFtoPOC",                                m_stopAfterFFtoPOC,                               "If using fast forward to POC, after the POC of interest has been hit, stop further encoding.")
  ("DecodeBitstream2ModPOCAndType",                   m_bs2ModPOCAndType,                               "Modify POC and NALU-type of second input bitstream, to use second BS as closing I-slice")
  ("ForceDecodeBitstream1",                           m_forceDecodeBitstream1,                          "force decoding of bitstream 1 - use this only if you are really sure about what you are doing ")
  ;

  opts.setSubSection("Coding tools");
  opts.addOptions()
  ("SMVD",                                            m_SMVD,                                           "Enable Symmetric MVD (0:off 1:vtm 2:fast 3:faster\n")
  ("AMVR",                                            m_AMVRspeed,                                      "Enable Adaptive MV precision Mode (IMV)")
  // added for backward compatibility with VTM
  ("IMV",                                             m_AMVRspeed,                                      "Enable Adaptive MV precision Mode (IMV)")
  ("LMChroma",                                        m_LMChroma,                                       "Enable LMChroma prediction")
  ("MRL",                                             m_MRL,                                            "MultiRefernceLines")
  ("BDOF",                                            m_BDOF,                                           "Enable bi-directional optical flow")
  // added for backward compatibility with VTM
  ("BIO",                                             m_BDOF,                                           "Enable bi-directional optical flow")
  ("DMVR",                                            m_DMVR,                                           "Decoder-side Motion Vector Refinement")
  ("EncDbOpt",                                        m_EDO,                                            "Encoder optimization with deblocking filter 0:off 1:vtm 2:fast")
  ("EDO",                                             m_EDO,                                            "Encoder optimization with deblocking filter 0:off 1:vtm 2:fast")
  ("LMCSEnable",                                      m_lumaReshapeEnable,                              "Enable LMCS luma mapping with chroma scaling (0:off 1:on 2:use SCC detection to disable for screen coded content)")
  ("LMCS",                                            m_lumaReshapeEnable,                              "Enable LMCS luma mapping with chroma scaling (0:off 1:on 2:use SCC detection to disable for screen coded content)")
  ("LMCSSignalType",                                  m_reshapeSignalType,                              "Input signal type (0:SDR, 1:HDR-PQ, 2:HDR-HLG)")
  ("LMCSUpdateCtrl",                                  m_updateCtrl,                                     "LMCS model update control (0:RA, 1:AI, 2:LDB/LDP)")
  ("LMCSAdpOption",                                   m_adpOption,                                      "LMCS adaptation options: 0:automatic,"
                                                                                                             "1: rsp both (CW66 for QP<=22), 2: rsp TID0 (for all QP),"
                                                                                                             "3: rsp inter(CW66 for QP<=22), 4: rsp inter(for all QP).")
  ("LMCSInitialCW",                                   m_initialCW,                                      "LMCS initial total codeword (0~1023) when LMCSAdpOption > 0")
  ("LMCSOffset",                                      m_LMCSOffset,                                     "LMCS chroma residual scaling offset")
  ("ALF",                                             m_alf,                                            "Adpative Loop Filter" )
  ("ALFSpeed",                                        m_alfSpeed,                                       "ALF speed (skip filtering of non-referenced frames) [0-1]" )
  ("CCALF",                                           m_ccalf,                                          "Cross-component Adaptive Loop Filter" )
  ("CCALFQpTh",                                       m_ccalfQpThreshold,                               "QP threshold above which encoder reduces CCALF usage. Ignored in case of PerceptQPA.")
  ("UseNonLinearAlfLuma",                             m_useNonLinearAlfLuma,                            "Non-linear adaptive loop filters for Luma Channel")
  ("UseNonLinearAlfChroma",                           m_useNonLinearAlfChroma,                          "Non-linear adaptive loop filters for Chroma Channels")
  ("MaxNumAlfAlternativesChroma",                     m_maxNumAlfAlternativesChroma,                    std::string("Maximum number of alternative Chroma filters (1-") + std::to_string(VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA) + std::string (", inclusive)") )
  ("ALFTempPred",                                     m_alfTempPred,                                    "Enable usage of ALF temporal prediction for filter data\n" )
  ("PROF",                                            m_PROF,                                           "Enable prediction refinement with optical flow for affine mode")
  ("Affine",                                          m_Affine,                                         "Enable affine prediction")
  ("AffineType",                                      m_AffineType,                                     "Enable affine type prediction")
  ("MMVD",                                            m_MMVD,                                           "Enable Merge mode with Motion Vector Difference")
  ("MmvdDisNum",                                      m_MmvdDisNum,                                     "Number of MMVD Distance Entries")
  ("AllowDisFracMMVD",                                m_allowDisFracMMVD,                               "Disable fractional MVD in MMVD mode adaptively")
  ("MCTF",                                            m_vvencMCTF.MCTF,                                 "Enable GOP based temporal filter. (0:off, 1:filter all frames, 2:use SCC detection to disable for screen coded content)")
  ("MCTFSpeed",                                       m_vvencMCTF.MCTFSpeed,                            "MCTF Fast Mode (0:best quality .. 4:fast)")
  ("MCTFFutureReference",                             m_vvencMCTF.MCTFFutureReference,                  "Enable referencing of future frames in the GOP based temporal filter. This is typically disabled for Low Delay configurations.")
  ("MCTFNumLeadFrames",                               m_vvencMCTF.MCTFNumLeadFrames,                    "Number of additional MCTF lead frames, which will not be encoded, but can used for MCTF filtering")
  ("MCTFNumTrailFrames",                              m_vvencMCTF.MCTFNumTrailFrames,                   "Number of additional MCTF trail frames, which will not be encoded, but can used for MCTF filtering")
  ("MCTFFrame",                                       toMCTFFrames,                                          "Frame to filter Strength for frame in GOP based temporal filter")
  ("MCTFStrength",                                    toMCTFStrengths,                                       "Strength for  frame in GOP based temporal filter.")

  ("FastLocalDualTreeMode",                           m_fastLocalDualTreeMode,                          "Fast intra pass coding for local dual-tree in intra coding region (0:off, 1:use threshold, 2:one intra mode only)")
  ("QtbttExtraFast",                                  m_qtbttSpeedUp,                                   "Non-VTM compatible QTBTT speed-ups" )
  ;

  opts.setSubSection("Threading, performance");
  opts.addOptions()
  ("MaxParallelFrames",                               m_maxParallelFrames,                              "Maximum number of frames to be processed in parallel(0:off, >=2: enable parallel frames)")
  ("WppBitEqual",                                     m_ensureWppBitEqual,                              "Ensure bit equality with WPP case (0:off (sequencial mode), 1:copy from wpp line above, 2:line wise reset)")
  ("EnablePicPartitioning",                           m_picPartitionFlag,                               "Enable picture partitioning (0: single tile, single slice, 1: multiple tiles/slices)")
  ;

  opts.setSubSection("Coding tools");
  opts.addOptions()
  ("SbTMVP",                                          m_SbTMVP,                                         "Enable Subblock Temporal Motion Vector Prediction (0: off, 1: on)")
  ("CIIP",                                            m_CIIP,                                           "Enable CIIP mode, 0: off, 1: vtm, 2: fast, 3: faster ")
  ("SBT",                                             m_SBT,                                            "Enable Sub-Block Transform for inter blocks (0: off 1: vtm, 2: fast, 3: faster)" )
  ("LFNST",                                           m_LFNST,                                          "Enable LFNST (0: off, 1: on)" )
  ("MTS",                                             m_MTS,                                            "Multiple Transform Set (MTS)" )
  ("MTSIntraMaxCand",                                 m_MTSIntraMaxCand,                                "Number of additional candidates to test for MTS in intra slices")
  ("ISP",                                             m_ISP,                                            "Intra Sub-Partitions Mode (0: off, 1: vtm, 2: fast, 3: faster)")
  ("TransformSkip",                                   m_TS,                                             "Intra transform skipping, 0: off, 1: TS, 2: TS with SCC detection ")
  ("TransformSkipLog2MaxSize",                        m_TSsize,                                         "Specify transform-skip maximum size. Minimum 2, Maximum 5")
  ("ChromaTS",                                        m_useChromaTS,                                    "Enable encoder search of chromaTS")
  ("BDPCM",                                           m_useBDPCM,                                       "BDPCM (0:off, 1:luma and chroma, 2: BDPCM with SCC detection)")
  ("RPR",                                             m_rprEnabledFlag,                                 "Reference Sample Resolution (0: disable, 1: eneabled, 2: RPR ready")
  ("IBC",                                             m_IBCMode,                                        "IBC (0:off, 1:IBC, 2: IBC with SCC detection)")
  ("IBCFastMethod",                                   m_IBCFastMethod,                                  "Fast methods for IBC. 1:default, [2..6] speedups")
  ("BCW",                                             m_BCW,                                            "Enable Generalized Bi-prediction(Bcw) 0: disabled, 1: enabled, 2: fast")
  ("FastInferMerge",                                  m_FIMMode,                                        "Fast method to skip Inter/Intra modes. 0: off, [1..4] speedups")
  ("NumIntraModesFullRD",                             m_numIntraModesFullRD,                            "Number Modes for Full RD Intra Search. Default: -1=Auto. If set to 1: 5% speedup 1% Loss for fast and faster")
  ("ReduceIntraChromaModesFullRD",                    m_reduceIntraChromaModesFullRD,                   "Reduce Modes for Chroma Full RD Intra Search. ")
  ;

  opts.setSubSection("Input options");
  opts.addOptions()
  ("HorCollocatedChroma",                             m_horCollocatedChromaFlag,                        "Specifies location of a chroma sample relatively to the luma sample in horizontal direction in the reference picture resampling"
                                                                                                        "(0: horizontally shifted by 0.5 units of luma samples, 1: collocated)")
  ("VerCollocatedChroma",                             m_verCollocatedChromaFlag,                        "Specifies location of a chroma sample relatively to the luma sample in vertical direction in the cross-component linear model intra prediction and the reference picture resampling"
                                                                                                        "(0: horizontally co-sited, vertically shifted by 0.5 units of luma samples, 1: collocated)")
  ("ClipInputVideoToRec709Range",                     m_bClipInputVideoToRec709Range,                   "Enable clipping input video to the Rec. 709 Range on loading when InternalBitDepth is less than MSBExtendedBitDepth")
  ;

  opts.setSubSection("Reconstructed video options");
  opts.addOptions()
  ("ClipOutputVideoToRec709Range",                    m_bClipOutputVideoToRec709Range,                  "Enable clipping output video to the Rec. 709 Range on saving when OutputBitDepth is less than InternalBitDepth")
  ("PYUV",                                            m_packedYUVMode,                                  "Enable output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data. Ignored for interlaced output.")
    ;


  // following options are unused, only for compatiblity
  int  cropOffsetLeft               = 0;
  int  cropOffsetTop                = 0;
  int  cropOffsetRight              = 0;
  int  cropOffsetBottom             = 0;
  bool calculateHdrMetrics          = false;
  bool signalledSliceIdFlag         = false;
  int  signalledSliceIdLengthMinus1 = 0;
  std::vector<int> rectSliceBoundary;
  std::vector<int> signalledSliceId;
  IStreamToVec<int> toRectSliceBoundary( &rectSliceBoundary );
  IStreamToVec<int> toSignalledSliceId ( &signalledSliceId  );
  bool sdr = false;

  opts.setSubSection("Unused options (only for compatiblity to VTM)");
  opts.addOptions()
  ("EnablePictureHeaderInSliceHeader",                m_enablePictureHeaderInSliceHeader,               "Enable Picture Header in Slice Header")

  ("CropOffsetLeft",                                  cropOffsetLeft,                                   "Crop Offset Left position")
  ("CropOffsetTop",                                   cropOffsetTop,                                    "Crop Offset Top position")
  ("CropOffsetRight",                                 cropOffsetRight,                                  "Crop Offset Right position")
  ("CropOffsetBottom",                                cropOffsetBottom,                                 "Crop Offset Bottom position")
  ("CalculateHdrMetrics",                             calculateHdrMetrics,                              "Enable HDR metric calculation")

  ("SignalledIdFlag",                                 signalledSliceIdFlag,                             "Signalled Slice ID Flag")
  ("SignalledSliceIdLengthMinus1",                    signalledSliceIdLengthMinus1,                     "Signalled Tile Group Length minus 1")
  ("RectSlicesBoundaryArray",                         toRectSliceBoundary,                              "Rectangular slices boundaries in Pic")
  ("SignalledSliceId",                                toSignalledSliceId,                               "Signalled rectangular slice ID")

  ("isSDR",                                           sdr,                                              "compatibility")
  ;


  po::setDefaults( opts );
  std::ostringstream fullOpts;
  po::doHelp( fullOpts, opts );

  for ( int i = 0; i < VVENC_MAX_GOP; i++ )
  {
    std::ostringstream cOSS;
    cOSS << "Frame" << i+1;
    opts.addOptions()(cOSS.str(), m_GOPList[i] );
  }
  opts.addOptions()("decode",                          m_decode,                                        "decode only");

  //
  // parse command line parameters and read configuration files
  //

  po::setDefaults( opts );
  po::ErrorReporter err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**) argv, err );
  for( auto& a : argv_unhandled )
  {
    cout << "Unhandled argument ignored: `" << a << "'\n";
  }
  if ( argc == 1 || do_help )
  {
    /* argc == 1: no options have been specified */
    cout <<  easyOpts.str();
    return false;
  }
  if ( argc == 1 || do_expert_help )
  {
    /* argc == 1: no options have been specified */
    cout << fullOpts.str();
    return false;
  }
  if ( err.is_errored && ! warnUnknowParameter )
  {
    /* error report has already been printed on stderr */
    return false;
  }

  if( !writeCfg.empty() )
  {
    std::ofstream cfgFile;
    cfgFile.open( writeCfg.c_str(), std::ios::out | std::ios::trunc);
    if( !cfgFile.is_open() )
    {
      std::cout << " [error]: failed to open output config file " << writeCfg << std::endl;
      return false;
    }
    else
    {
      std::list<std::string> ignoreParamsLst;
      ignoreParamsLst.push_back( "help" );
      ignoreParamsLst.push_back( "fullhelp" );

      ignoreParamsLst.push_back( "WriteConfig" );
      ignoreParamsLst.push_back( "SIMD" );
      ignoreParamsLst.push_back( "c" );
      //ignoreParamsLst.push_back( "WarnUnknowParameter,w" );

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

  if( m_showVersion )
  {
    return true;
  }

  // has to be set outside
  if ( m_internChromaFormat < 0 || m_internChromaFormat >= VVENC_NUM_CHROMA_FORMAT )
  {
    m_internChromaFormat = m_inputFileChromaFormat;
  }

  return checkCfg();
}

std::string VVEncAppCfg::getConfigAsString( vvencMsgLevel eMsgLevel ) const
{
  std::stringstream css;
  if( eMsgLevel >= VVENC_DETAILS )
  {
    css << "Input          File                    : " << m_inputFileName << "\n";
    css << "Bitstream      File                    : " << m_bitstreamFileName << "\n";
    css << "Reconstruction File                    : " << m_reconFileName << "\n";
    css << "RC Statistics  File                    : " << m_RCStatsFileName << "\n";
  }

  const VVEncAppCfg* configPtr = this;
  std::string config ( vvenc_get_config_as_string( (vvenc_config*)configPtr, eMsgLevel) );
  css << config;
  css << "\n";

  return css.str();
}

bool VVEncAppCfg::checkCfg()
{
  bool ret = true;

  // check bitstream file name in encode and decode mode
  if( m_bitstreamFileName.empty() )
  {
    cout << "error: bitstream file name must be specified (--output=bit.266)" << std::endl;
    ret = false;
  }

  // check remaining parameters in encode mode only
  if( m_decode )
  {
    return ret;
  }

  // check internal config parameters and derive adapted parameter set
  VVEncAppCfg appCfg = *this;
  if( vvenc_init_config_parameter( &appCfg ) )
  {
    ret = false;
  }

  // check remaining parameter set
  return appCfg.xCheckCfg();
}

bool VVEncAppCfg::xCheckCfg()
{
  bool ret = true;

  if( m_inputFileName.empty() )
  {
    cout << "error: input yuv file name must be specified (--input=video.yuv)" << std::endl;
    ret = false;
  }
  if( ! strcmp( m_inputFileName.c_str(), "-" )
      && m_RCNumPasses > 1
      && m_RCPass < 0 )
  {
    cout << "error: two pass rate control within single application call and reading from stdin not supported" << std::endl;
    ret = false;
  }

  if( ! m_bitstreamFileName.empty() )
  {
    if( m_decodeBitstreams[0][0] != '\0' && m_decodeBitstreams[0] == m_bitstreamFileName )
    {
      cout << "error: debug bitstream and the output bitstream cannot be equal" << std::endl;
      ret = false;
    }
    if( m_decodeBitstreams[1][0] != '\0' && m_decodeBitstreams[1] == m_bitstreamFileName )
    {
      cout << "error: decode2 bitstream and the output bitstream cannot be equal" << std::endl;
      ret = false;
    }
  }

#ifndef VVENC_ENABLE_THIRDPARTY_JSON
  if( m_RCPass > 0 )
  {
    cout << "error: reading/writing rate control statistics file not supported, please disable pass parameter or compile with json enabled" << std::endl;
    ret = false;
  }
  if( ! m_RCStatsFileName.empty() )
  {
    cout << "error: reading/writing rate control statistics file not supported, please disable rcstatsfile parameter or compile with json enabled" << std::endl;
    ret = false;
  }
#endif

  if( m_RCPass > 0 && m_RCStatsFileName.empty() )
  {
    cout << "error: rate control statistics file name must be specify, when pass parameter is set (--rcstatsfile=stats.json)" << std::endl;
    ret = false;
  }
  if( m_RCNumPasses == 1 && ! m_RCStatsFileName.empty() )
  {
    cout << "error: rate control statistics file not supported in single pass encoding" << std::endl;
    ret = false;
  }

  if( m_inputFileChromaFormat != VVENC_CHROMA_400 && m_inputFileChromaFormat != VVENC_CHROMA_420 )
  {
    cout << "error: input chroma format must be either 400, 420" << std::endl;
    ret = false;
  }

  if( ! m_reconFileName.empty() && m_packedYUVMode )
  {
    if( ( m_outputBitDepth[ 0 ] != 10 && m_outputBitDepth[ 0 ] != 12 )
        || ( ( ( m_SourceWidth ) & ( 1 + ( m_outputBitDepth[ 0 ] & 3 ) ) ) != 0 ) )
    {
      cout << "error: invalid output bit-depth or image width for packed YUV output" << std::endl;
      ret = false;
    }
    if( ( m_internChromaFormat != VVENC_CHROMA_400 ) && ( ( m_outputBitDepth[ 1 ] != 10 && m_outputBitDepth[ 1 ] != 12 )
          || ( ( vvenc_get_width_of_component( m_internChromaFormat, m_SourceWidth, 1 ) & ( 1 + ( m_outputBitDepth[ 1 ] & 3 ) ) ) != 0 ) ) )
    {
      cout << "error: invalid chroma output bit-depth or image width for packed YUV output" << std::endl;
      ret = false;
    }
  }

  return ret;
}


} // namespace

//! \}


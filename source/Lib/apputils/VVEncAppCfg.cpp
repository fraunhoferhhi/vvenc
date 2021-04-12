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
  { "auto",                                  vvencProfile::VVENC_PROFILE_AUTO }
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
  { "0",                     VVENC_DRT_NONE },
  { "1",                     VVENC_DRT_CRA },
  { "2",                     VVENC_DRT_IDR },
  { "3",                     VVENC_DRT_RECOVERY_POINT_SEI },
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
  vvenc_init_preset( &cfg->conf, (vvencPresetMode)preset );
}

void setInputBitDepthAndColorSpace( VVEncAppCfg* cfg, int dbcs )
{
  switch( dbcs )
  {
  case YUV420_8 :  cfg->m_inputFileChromaFormat = VVENC_CHROMA_420; cfg->conf.m_inputBitDepth[0] = 8;  break;
  case YUV420_10 : cfg->m_inputFileChromaFormat = VVENC_CHROMA_420; cfg->conf.m_inputBitDepth[0] = 10; break;
  case YUV422_8 :  cfg->m_inputFileChromaFormat = VVENC_CHROMA_422; cfg->conf.m_inputBitDepth[0] = 8;  break;
  case YUV422_10 : cfg->m_inputFileChromaFormat = VVENC_CHROMA_422; cfg->conf.m_inputBitDepth[0] = 10; break;
  case YUV444_8 :  cfg->m_inputFileChromaFormat = VVENC_CHROMA_444; cfg->conf.m_inputBitDepth[0] = 8;  break;
  case YUV444_10 : cfg->m_inputFileChromaFormat = VVENC_CHROMA_444; cfg->conf.m_inputBitDepth[0] = 10; break;
  case YUV400_8 :  cfg->m_inputFileChromaFormat = VVENC_CHROMA_400; cfg->conf.m_inputBitDepth[0] = 8;  break;
  case YUV400_10 : cfg->m_inputFileChromaFormat = VVENC_CHROMA_400; cfg->conf.m_inputBitDepth[0] = 10; break;
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
  IStreamToEnum<vvencMsgLevel>      toMsgLevel                   ( &this->conf.m_verbosity,   &MsgLevelToEnumMap );
  IStreamToFunc<vvencPresetMode>    toPreset                     ( setPresets, this, &PresetToEnumMap,vvencPresetMode::VVENC_MEDIUM);
  IStreamToRefVec<int>              toSourceSize                 ( { &this->conf.m_SourceWidth, &this->conf.m_SourceHeight }, true, 'x' );

  IStreamToEnum<vvencProfile>       toProfile                    ( &this->conf.m_profile,                     &ProfileToEnumMap      );
  IStreamToEnum<vvencTier>          toLevelTier                  ( &this->conf.m_levelTier,                   &TierToEnumMap         );
  IStreamToEnum<vvencLevel>         toLevel                      ( &this->conf.m_level,                       &LevelToEnumMap        );
  IStreamToEnum<vvencSegmentMode>   toSegment                    ( &this->conf.m_SegmentMode,                 &SegmentToEnumMap      );
  IStreamToEnum<vvencHDRMode>       toHDRMode                    ( &this->conf.m_HdrMode,                     &HdrModeToIntMap       );

  IStreamToFunc<BitDepthAndColorSpace> toInputFormatBitdepth( setInputBitDepthAndColorSpace, this, &BitColorSpaceToIntMap, YUV420_8);
  IStreamToEnum<vvencDecodingRefreshType>   toDecRefreshType     ( &this->conf.m_DecodingRefreshType,         &DecodingRefreshTypeToEnumMap );

  IStreamToEnum<int>           toAud                        ( &this->conf.m_AccessUnitDelimiter,             &FlagToIntMap );
  IStreamToEnum<int>           toHrd                        ( &this->conf.m_hrdParametersPresent,            &FlagToIntMap );
  IStreamToEnum<int>           toVui                        ( &this->conf.m_vuiParametersPresent,            &FlagToIntMap );
  IStreamToEnum<bool>          toQPA                        ( &this->conf.m_usePerceptQPA,                   &QPAToIntMap );


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
  ;
  opts.setSubSection("Input Options");
  opts.addOptions()
  ("input,i",           m_inputFileName,          "original YUV input file name")
  ("size,s",            toSourceSize,             "specify input resolution (WidthxHeight)")
  ("format,c",          toInputFormatBitdepth,    "set input format (yuv420, yuv420_10)")

  ("framerate,r",       conf.m_FrameRate,         "temporal rate (framerate) e.g. 25,29,30,50,59,60 ")
  ("tickspersec",       conf.m_TicksPerSecond,    "Ticks Per Second for dts generation, ( 1..27000000)")

  ("frames,f",          conf.m_framesToBeEncoded, "max. frames to encode [all]")
  ("frameskip",         conf.m_FrameSkip,         "Number of frames to skip at start of input YUV [off]")
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

  ("bitrate,b",         conf.m_RCTargetBitrate,   "bitrate for rate control (0: constant-QP encoding without rate control, otherwise bits/second)" )
  ("passes,p",          conf.m_RCNumPasses,       "number of rate control passes (1,2) " )
  ("qp,q",              conf.m_QP,                "quantization parameter, QP (0-63)")
  ("qpa",               toQPA,                    "Enable perceptually motivated QP adaptation, XPSNR based (0:off, 1:on)", true)

  ("threads,-t",        conf.m_numThreads,        "Number of threads default: [size <= HD: 4, UHD: 6]")

  ("gopsize,g",         conf.m_GOPSize,           "GOP size of temporal structure (16,32)")
  ("refreshtype,-rt",   toDecRefreshType,         "intra refresh type (idr,cra)")
  ("refreshsec,-rs",    conf.m_IntraPeriodSec,    "Intra period/refresh in seconds")
  ("intraperiod,-ip",   conf.m_IntraPeriod,       "Intra period in frames (0: use intra period in seconds (refreshsec), else: n*gopsize)")
  ;

  opts.setSubSection("Profile, Level, Tier");
  opts.addOptions()
  ("profile",           toProfile,                "select profile (main10, main10_stillpic)")
  ("level",             toLevel,                  "Level limit (1.0, 2.0,2.1, 3.0,3.1, 4.0,4.1, 5.0,5.1,5.2, 6.0,6.1,6.2,6.3 15.5)")
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
  ("internal-bitdepth",         conf.m_internalBitDepth[0], "internal bitdepth (8,10)")
  ("accessunitdelimiter,-aud",  toAud,                 "Emit Access Unit Delimiter NALUs  (auto(-1),off(0),on(1); default: auto - only if needed by dependent options)", true)
  ("vuiparameterspresent,-vui", toVui,                 "Emit VUI information (auto(-1),off(0),on(1); default: auto - only if needed by dependent options)", true)
  ("hrdParameterspresent,-hrd", toHrd,                 "Emit VUI HRD information (auto(-1),off(0),on(1); default: auto - only if needed by dependent options)",  true)
  ;

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
    /* error report has already been printed on stderr */
    return false;
  }

  //
  // check own parameters
  //

  if( m_bitstreamFileName.empty() )
  {
    cout <<  "error: A bitstream file name must be specified (--output=bit.266)" << std::endl;
    return false;
  }

  // this has to be set outside
  if ( conf.m_internChromaFormat < 0 || conf.m_internChromaFormat >= VVENC_NUM_CHROMA_FORMAT )
  {
    conf.m_internChromaFormat = m_inputFileChromaFormat;
  }

  if( conf.m_RCNumPasses < 0 )
  {
    conf.m_RCNumPasses = conf.m_RCTargetBitrate > 0 ? 2 : 1;
  }

  return true;
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
  IStreamToEnum<vvencMsgLevel>      toMsgLevel                   ( &this->conf.m_verbosity,                   &MsgLevelToEnumMap      );
  IStreamToFunc<vvencPresetMode>    toPreset                     ( setPresets, this, &PresetToEnumMap,vvencPresetMode::VVENC_MEDIUM);
  IStreamToRefVec<int>              toSourceSize                 ( { &this->conf.m_SourceWidth, &this->conf.m_SourceHeight }, true, 'x' );
  IStreamToRefVec<double>           toLambdaModifier             ( { &this->conf.m_adLambdaModifier[0], &this->conf.m_adLambdaModifier[1], &this->conf.m_adLambdaModifier[2], &this->conf.m_adLambdaModifier[3], &this->conf.m_adLambdaModifier[4], &this->conf.m_adLambdaModifier[5], &this->conf.m_adLambdaModifier[6] }, false );

  IStreamToEnum<vvencProfile>       toProfile                    ( &this->conf.m_profile,                     &ProfileToEnumMap      );
  IStreamToEnum<vvencTier>          toLevelTier                  ( &this->conf.m_levelTier,                   &TierToEnumMap         );
  IStreamToEnum<vvencLevel>         toLevel                      ( &this->conf.m_level,                       &LevelToEnumMap        );
  IStreamToEnum<vvencCostMode>      toCostMode                   ( &this->conf.m_costMode,                    &CostModeToEnumMap     );
  IStreamToEnum<vvencChromaFormat>  toInputFileCoFormat          ( &m_inputFileChromaFormat,       &ChromaFormatToEnumMap  );
  IStreamToEnum<vvencChromaFormat>  toInternCoFormat             ( &this->conf.m_internChromaFormat,          &ChromaFormatToEnumMap  );
  IStreamToEnum<vvencHashType>      toHashType                   ( &this->conf.m_decodedPictureHashSEIType,   &HashTypeToEnumMap     );
  IStreamToArr<int>                 toQpInCb                     ( &this->conf.m_qpInValsCb[0], VVENC_MAX_QP_VALS_CHROMA            );
  IStreamToArr<int>                 toQpOutCb                    ( &this->conf.m_qpOutValsCb[0], VVENC_MAX_QP_VALS_CHROMA           );
  IStreamToArr<int>                 toQpInCr                     ( &this->conf.m_qpInValsCr[0], VVENC_MAX_QP_VALS_CHROMA            );
  IStreamToArr<int>                 toQpOutCr                    ( &this->conf.m_qpOutValsCr[0], VVENC_MAX_QP_VALS_CHROMA           );
  IStreamToArr<int>                 toQpInCbCr                   ( &this->conf.m_qpInValsCbCr[0], VVENC_MAX_QP_VALS_CHROMA          );
  IStreamToArr<int>                 toQpOutCbCr                  ( &this->conf.m_qpOutValsCbCr[0], VVENC_MAX_QP_VALS_CHROMA         );
  IStreamToArr<double>              toIntraLambdaModifier        ( &this->conf.m_adIntraLambdaModifier[0], VVENC_MAX_TLAYER );

  IStreamToArr<int>                 toMCTFFrames                 ( &this->conf.m_vvencMCTF.MCTFFrames[0], VVENC_MAX_MCTF_FRAMES   );
  IStreamToArr<double>              toMCTFStrengths              ( &this->conf.m_vvencMCTF.MCTFStrengths[0], VVENC_MAX_MCTF_FRAMES);
  IStreamToEnum<vvencSegmentMode>   toSegment                    ( &this->conf.m_SegmentMode,            &SegmentToEnumMap );
  IStreamToEnum<vvencHDRMode>       toHDRMode                    ( &this->conf.m_HdrMode,                &HdrModeToIntMap       );
  IStreamToEnum<int>                toColorPrimaries             ( &this->conf.m_colourPrimaries,        &ColorPrimariesToIntMap );
  IStreamToEnum<int>                toTransferCharacteristics    ( &this->conf.m_transferCharacteristics,&TransferCharacteristicsToIntMap );
  IStreamToEnum<int>                toColorMatrix                ( &this->conf.m_matrixCoefficients,     &ColorMatrixToIntMap );
  IStreamToEnum<int>                toPrefTransferCharacteristics( &this->conf.m_preferredTransferCharacteristics, &TransferCharacteristicsToIntMap );

  IStreamToArr<unsigned int>        toMasteringDisplay           ( &this->conf.m_masteringDisplay[0], 10  );
  IStreamToArr<unsigned int>        toContentLightLevel          ( &this->conf.m_contentLightLevel[0], 2 );

  IStreamToEnum<vvencDecodingRefreshType> toDecRefreshType       ( &this->conf.m_DecodingRefreshType, &DecodingRefreshTypeToEnumMap );

  IStreamToEnum<int>                toAud                        ( &this->conf.m_AccessUnitDelimiter,             &FlagToIntMap );
  IStreamToEnum<int>                toHrd                        ( &this->conf.m_hrdParametersPresent,            &FlagToIntMap );
  IStreamToEnum<int>                toVui                        ( &this->conf.m_vuiParametersPresent,            &FlagToIntMap );

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
  ;

  opts.setSubSection("Input options");
  opts.addOptions()
  ("InputFile,i",                                     m_inputFileName,                                  "Original YUV input file name")
  ("Size,s",                                          toSourceSize,                                     "Input resolution (WidthxHeight)")
  ("InputBitDepth",                                   conf.m_inputBitDepth[ 0 ],                        "Bit-depth of input file")
  ("FramesToBeEncoded,f",                             conf.m_framesToBeEncoded,                         "Number of frames to be encoded (default=all)")
  ("FrameRate,-fr",                                   conf.m_FrameRate,                                 "Frame rate")
  ("FrameSkip,-fs",                                   conf.m_FrameSkip,                                 "Number of frames to skip at start of input YUV")
  ("TicksPerSecond",                                  conf.m_TicksPerSecond,                            "Ticks Per Second for dts generation, ( 1..27000000)")

  ("segment",                                         toSegment,                                        "when encoding multiple separate segments, specify segment position to enable segment concatenation (first, mid, last) [off]\n"
                                                                                                        "first: first segment           \n"
                                                                                                        "mid  : all segments between first and last segment\n"
                                                                                                        "last : last segment")
  ;

  opts.setSubSection("Output options");
  opts.addOptions()
  ("BitstreamFile,b",                                 m_bitstreamFileName,                              "Bitstream output file name")
  ("ReconFile,o",                                     m_reconFileName,                                  "Reconstructed YUV output file name")
  ("OutputBitDepth",                                  conf.m_outputBitDepth[ 0 ],                       "Bit-depth of output file")
  ;

  opts.setSubSection("Profile, Level, Tier");
  opts.addOptions()
  ("Profile",                                         toProfile,                                        "Profile name to use for encoding. Use [multilayer_]main_10[_444][_still_picture], auto, or none")
  ("Tier",                                            toLevelTier,                                      "Tier to use for interpretation of level (main or high)")
  ("Level",                                           toLevel,                                          "Level limit to be used, eg 5.1, or none")
  ;

  opts.setSubSection("Threading, performance");
  opts.addOptions()
  ("Threads,t",                                       conf.m_numThreads,                                "Number of threads")
  ("preset",                                          toPreset,                                         "select preset for specific encoding setting (faster, fast, medium, slow, slower)")
  ;

  opts.setSubSection("Slice decision options");
  opts.addOptions()
  ("IntraPeriod,-ip",                                 conf.m_IntraPeriod,                               "Intra period in frames, (-1: only first frame)")
  ("RefreshSec,-rs",                                  conf.m_IntraPeriodSec,                            "Intra period in seconds")
  ("DecodingRefreshType,-dr",                         toDecRefreshType,                                 "Intra refresh type (0:none, 1:CRA, 2:IDR, 3:RecPointSEI)")
  ("GOPSize,g",                                       conf.m_GOPSize,                                   "GOP size of temporal structure")
  ;

  opts.setSubSection("Rate control, Perceptual Quantization");
  opts.addOptions()
  ("NumPasses",                                       conf.m_RCNumPasses,                               "number of passes; 1: one-pass rate control; 2: two-pass rate control" )
  ("TargetBitrate",                                   conf.m_RCTargetBitrate,                           "Rate control: target bit-rate [bps]" )

  ("PerceptQPA,-qpa",                                 conf.m_usePerceptQPA,                             "Enable perceptually motivated QP adaptation, XPSNR based (0:off, 1:on)", true)
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
    ("TraceChannelsList",                             conf.m_listTracingChannels,                       "List all available tracing channels")
    ("TraceRule",                                     conf.m_traceRule,                                 "Tracing rule (ex: \"D_CABAC:poc==8\" or \"D_REC_CB_LUMA:poc==8\")")
    ("TraceFile",                                     conf.m_traceFile,                                 "Tracing file")
    ;
  }

  // file, i/o and source parameters
  opts.setSubSection("Input options");
  opts.addOptions()
  ("SourceWidth",                                     conf.m_SourceWidth,                               "Source picture width")
  ("SourceHeight",                                    conf.m_SourceHeight,                              "Source picture height")

  ("ConformanceWindowMode",                           conf.m_conformanceWindowMode,                     "Window conformance mode (0:off, 1:automatic padding, 2:padding, 3:conformance")
  ("ConfWinLeft",                                     conf.m_confWinLeft,                               "Left offset for window conformance mode 3")
  ("ConfWinRight",                                    conf.m_confWinRight,                              "Right offset for window conformance mode 3")
  ("ConfWinTop",                                      conf.m_confWinTop,                                "Top offset for window conformance mode 3")
  ("ConfWinBottom",                                   conf.m_confWinBottom,                             "Bottom offset for window conformance mode 3")

  ("TemporalSubsampleRatio",                          conf.m_temporalSubsampleRatio,                    "Temporal sub-sample ratio when reading input YUV")

  ("HorizontalPadding",                               conf.m_aiPad[0],                                  "Horizontal source padding for conformance window mode 2")
  ("VerticalPadding",                                 conf.m_aiPad[1],                                  "Vertical source padding for conformance window mode 2")

  ("InputChromaFormat",                               toInputFileCoFormat,                              "input file chroma format (400, 420, 422, 444)")
  ;

  opts.setSubSection("Quality reporting metrics");
  opts.addOptions()
  ("MSEBasedSequencePSNR",                            conf.m_printMSEBasedSequencePSNR,                 "Emit sequence PSNR (0: only as a linear average of the frame PSNRs, 1: also based on an average of the frame MSEs")
  ("PrintHexPSNR",                                    conf.m_printHexPsnr,                              "Emit hexadecimal PSNR for each frame (0: off , 1:on")
  ("PrintFrameMSE",                                   conf.m_printFrameMSE,                             "Emit MSE values for each frame (0: off , 1:on")
  ("PrintSequenceMSE",                                conf.m_printSequenceMSE,                          "Emit MSE values for the whole sequence (0: off , 1:on)")
  ;

  opts.setSubSection("Bitstream options");
  opts.addOptions()
  ("CabacZeroWordPaddingEnabled",                     conf.m_cabacZeroWordPaddingEnabled,               "Add conforming cabac-zero-words to bit streams (0: do not add, 1: add as required)")
  ;

  // Profile and level
  opts.setSubSection("Profile, Level, Tier");
  opts.addOptions()
  ("SubProfile",                                      conf.m_subProfile,                                "Sub-profile idc")
  ("MaxBitDepthConstraint",                           conf.m_bitDepthConstraintValue,                   "Bit depth to use for profile-constraint for RExt profiles. (0: automatically choose based upon other parameters)")
  ("IntraConstraintFlag",                             conf.m_intraOnlyConstraintFlag,                   "Value of general_intra_constraint_flag to use for RExt profiles (not used if an explicit RExt sub-profile is specified)")
  ;

  opts.setSubSection("Rate control, Perceptual Quantization");
  opts.addOptions()
  ("RCInitialQP",                                     conf.m_RCInitialQP,                               "Rate control: initial QP" )
  ("RCForceIntraQP",                                  conf.m_RCForceIntraQP,                            "Rate control: force intra QP to be equal to initial QP" )

  ("PerceptQPATempFiltIPic",                          conf.m_usePerceptQPATempFiltISlice,               "Temporal high-pass filter in QPA activity calculation for key pictures (0:off, 1:on, 2:on incl. temporal pumping reduction, -1:auto)")
  ;

  // Coding structure paramters
  opts.setSubSection("Coding structure paramters");
  opts.addOptions()
  ("InputQueueSize",                                  conf.m_InputQueueSize,                            "Size of input frames queue (use gop size)")
  ("ReWriteParamSets",                                conf.m_rewriteParamSets,                          "Enable rewriting of Parameter sets before every (intra) random access point")
  ("IDRRefParamList",                                 conf.m_idrRefParamList,                           "Enable indication of reference picture list syntax elements in slice headers of IDR pictures")
  ;

  /* Quantization parameters */
  opts.setSubSection("Quantization paramters");
  opts.addOptions()
  ("QP,q",                                            conf.m_QP,                                        "Qp value")
  ("SameCQPTablesForAllChroma",                       conf.m_useSameChromaQPTables,                     "0: Different tables for Cb, Cr and joint Cb-Cr components, 1 (default): Same tables for all three chroma components")
  ("IntraQPOffset",                                   conf.m_intraQPOffset,                             "Qp offset value for intra slice, typically determined based on GOP size")
  ("LambdaFromQpEnable",                              conf.m_lambdaFromQPEnable,                        "Enable flag for derivation of lambda from QP")
  ("LambdaModifier",                                  toLambdaModifier,                                 "Lambda modifier list for temporal layers. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifierI",                                 toIntraLambdaModifier,                            "Lambda modifiers for Intra pictures, comma separated, up to one the number of temporal layer. If entry for temporalLayer exists, then use it, else if some are specified, use the last, else use the standard LambdaModifiers.")
  ("IQPFactor",                                       conf.m_dIntraQpFactor,                            "Intra QP Factor for Lambda Computation. If negative, the default will scale lambda based on GOP size (unless LambdaFromQpEnable then IntraQPOffset is used instead)")
  ("QpInValCb",                                       toQpInCb,                                         "Input coordinates for the QP table for Cb component")
  ("QpInValCr",                                       toQpInCr,                                         "Input coordinates for the QP table for Cr component")
  ("QpInValCbCr",                                     toQpInCbCr,                                       "Input coordinates for the QP table for joint Cb-Cr component")
  ("QpOutValCb",                                      toQpOutCb,                                        "Output coordinates for the QP table for Cb component")
  ("QpOutValCr",                                      toQpOutCr,                                        "Output coordinates for the QP table for Cr component")
  ("QpOutValCbCr",                                    toQpOutCbCr,                                      "Output coordinates for the QP table for joint Cb-Cr component")
  ("MaxCuDQPSubdiv",                                  conf.m_cuQpDeltaSubdiv,                           "Maximum subdiv for CU luma Qp adjustment")
  ("MaxCuChromaQpOffsetSubdiv",                       conf.m_cuChromaQpOffsetSubdiv,                    "Maximum subdiv for CU chroma Qp adjustment - set less than 0 to disable")
  ("CbQpOffset",                                      conf.m_chromaCbQpOffset,                          "Chroma Cb QP Offset")
  ("CrQpOffset",                                      conf.m_chromaCrQpOffset,                          "Chroma Cr QP Offset")
  ("CbQpOffsetDualTree",                              conf.m_chromaCbQpOffsetDualTree,                  "Chroma Cb QP Offset for dual tree")
  ("CrQpOffsetDualTree",                              conf.m_chromaCrQpOffsetDualTree,                  "Chroma Cr QP Offset for dual tree")
  ("CbCrQpOffset",                                    conf.m_chromaCbCrQpOffset,                        "QP Offset for joint Cb-Cr mode")
  ("CbCrQpOffsetDualTree",                            conf.m_chromaCbCrQpOffsetDualTree,                "QP Offset for joint Cb-Cr mode in dual tree")
  ("SliceChromaQPOffsetPeriodicity",                  conf.m_sliceChromaQpOffsetPeriodicity,            "Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.")
  ("SliceCbQpOffsetIntraOrPeriodic",                  conf.m_sliceChromaQpOffsetIntraOrPeriodic[0],     "Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")
  ("SliceCrQpOffsetIntraOrPeriodic",                  conf.m_sliceChromaQpOffsetIntraOrPeriodic[1],     "Chroma Cr QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")

  ("LumaLevelToDeltaQPMode",                          conf.m_lumaLevelToDeltaQPEnabled,                 "Luma based Delta QP 0(default): not used. 1: Based on CTU average")
  ("WCGPPSEnable",                                    conf.m_wcgChromaQpControl.enabled,                "1: Enable the WCG PPS chroma modulation scheme. 0 (default) disabled")
  ("WCGPPSCbQpScale",                                 conf.m_wcgChromaQpControl.chromaCbQpScale,        "WCG PPS Chroma Cb QP Scale")
  ("WCGPPSCrQpScale",                                 conf.m_wcgChromaQpControl.chromaCrQpScale,        "WCG PPS Chroma Cr QP Scale")
  ("WCGPPSChromaQpScale",                             conf.m_wcgChromaQpControl.chromaQpScale,          "WCG PPS Chroma QP Scale")
  ("WCGPPSChromaQpOffset",                            conf.m_wcgChromaQpControl.chromaQpOffset,         "WCG PPS Chroma QP Offset")
  ;

  opts.setSubSection("Misc. options");
  opts.addOptions()
  ("ChromaFormatIDC,-cf",                             toInternCoFormat,                                 "intern chroma format (400, 420, 422, 444) or set to 0 (default), same as InputChromaFormat")
  ("UseIdentityTableForNon420Chroma",                 conf.m_useIdentityTableForNon420Chroma,           "True: Indicates that 422/444 chroma uses identity chroma QP mapping tables; False: explicit Qp table may be specified in config")
  ("InputBitDepthC",                                  conf.m_inputBitDepth[ 1 ],                        "As per InputBitDepth but for chroma component. (default:InputBitDepth)")
  ("InternalBitDepth",                                conf.m_internalBitDepth[ 0 ],                     "Bit-depth the codec operates at. (default: MSBExtendedBitDepth). If different to MSBExtendedBitDepth, source data will be converted")
  ("OutputBitDepthC",                                 conf.m_outputBitDepth[ 1 ],                       "As per OutputBitDepth but for chroma component. (default: use luma output bit-depth)")
  ("MSBExtendedBitDepth",                             conf.m_MSBExtendedBitDepth[ 0 ],                  "bit depth of luma component after addition of MSBs of value 0 (used for synthesising High Dynamic Range source material). (default:InputBitDepth)")
  ("MSBExtendedBitDepthC",                            conf.m_MSBExtendedBitDepth[ 1 ],                  "As per MSBExtendedBitDepth but for chroma component. (default:MSBExtendedBitDepth)")

  ("WaveFrontSynchro",                                conf.m_entropyCodingSyncEnabled,                  "Enable entropy coding sync")
  ("EntryPointsPresent",                              conf.m_entryPointsPresent,                        "Enable entry points in slice header")
  ;

  opts.setSubSection("Quad-Tree size and depth");
  opts.addOptions()
  ("CTUSize",                                         conf.m_CTUSize,                                   "CTUSize")
  ("MinQTISlice",                                     conf.m_MinQT[0],                                  "MinQTISlice")
  ("MinQTLumaISlice",                                 conf.m_MinQT[0],                                  "MinQTLumaISlice")
  ("MinQTNonISlice",                                  conf.m_MinQT[1],                                  "MinQTNonISlice")
  ("MinQTChromaISliceInChromaSamples",                conf.m_MinQT[2],                                  "MinQTChromaISlice")
  ("MaxMTTDepth",                                     conf.m_maxMTTDepth,                               "maxMTTDepth")
  ("MaxMTTDepthI",                                    conf.m_maxMTTDepthI,                              "maxMTTDepthI")
  ("MaxMTTDepthISliceL",                              conf.m_maxMTTDepthI,                              "maxMTTDepthISliceL")
  ("MaxMTTDepthISliceC",                              conf.m_maxMTTDepthIChroma,                        "maxMTTDepthISliceC")
  // --> deprecated
  ("MaxMTTHierarchyDepth",                            conf.m_maxMTTDepth,                               "maxMTTDepth")
  ("MaxMTTHierarchyDepthI",                           conf.m_maxMTTDepthI,                              "maxMTTDepthI")
  ("MaxMTTHierarchyDepthISliceL",                     conf.m_maxMTTDepthI,                              "maxMTTDepthISliceL")
  ("MaxMTTHierarchyDepthISliceC",                     conf.m_maxMTTDepthIChroma,                        "maxMTTDepthISliceC")
  // <-- deprecated
  ("MaxBTLumaISlice",                                 conf.m_maxBT[0],                                  "MaxBTLumaISlice")
  ("MaxBTChromaISlice",                               conf.m_maxBT[2],                                  "MaxBTChromaISlice")
  ("MaxBTNonISlice",                                  conf.m_maxBT[1],                                  "MaxBTNonISlice")
  ("MaxTTLumaISlice",                                 conf.m_maxTT[0],                                  "MaxTTLumaISlice")
  ("MaxTTChromaISlice",                               conf.m_maxTT[2],                                  "MaxTTChromaISlice")
  ("MaxTTNonISlice",                                  conf.m_maxTT[1],                                  "MaxTTNonISlice")
  ("DualITree",                                       conf.m_dualITree,                                 "Use separate luma and chroma QTBT trees for intra slice")
  ("Log2MaxTbSize",                                   conf.m_log2MaxTbSize,                             "Maximum transform block size in logarithm base 2")
  ("Log2MinCodingBlockSize",                          conf.m_log2MinCodingBlockSize,                    "Minimum coding block size in logarithm base 2")
  ;

  opts.setSubSection("Coding tools");
  opts.addOptions()
  ("CostMode",                                        toCostMode,                                       "Use alternative cost functions: choose between 'lossy', 'sequence_level_lossless', 'lossless' (which forces QP to " MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP) ") and 'mixed_lossless_lossy' (which used QP'=" MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME) " for pre-estimates of transquant-bypass blocks).")
  ("ASR",                                             conf.m_bUseASR,                                   "Adaptive motion search range")
  ("HadamardME",                                      conf.m_bUseHADME,                                 "Hadamard ME for fractional-pel")
  ("RDOQ",                                            conf.m_RDOQ,                                      "Rate-Distortion Optimized Quantization mode")
  ("RDOQTS",                                          conf.m_useRDOQTS,                                 "Rate-Distortion Optimized Quantization mode for TransformSkip")
  ("SelectiveRDOQ",                                   conf.m_useSelectiveRDOQ,                          "Enable selective RDOQ")

  ("JointCbCr",                                       conf.m_JointCbCrMode,                             "Enable joint coding of chroma residuals (0:off, 1:on)")
  ("CabacInitPresent",                                conf.m_cabacInitPresent,                          "Enable cabac table index selection based on previous frame")
  ("LCTUFast",                                        conf.m_useFastLCTU,                               "Fast methods for large CTU")
  ("PBIntraFast",                                     conf.m_usePbIntraFast,                            "Fast assertion if the intra mode is probable")
  ("FastMrg",                                         conf.m_useFastMrg,                                "Fast methods for inter merge")
  ("AMaxBT",                                          conf.m_useAMaxBT,                                 "Adaptive maximal BT-size")
  ("FastQtBtEnc",                                     conf.m_fastQtBtEnc,                               "Fast encoding setting for QTBT")
  ("ContentBasedFastQtbt",                            conf.m_contentBasedFastQtbt,                      "Signal based QTBT speed-up")
  ("FEN",                                             conf.m_fastInterSearchMode,                       "fast encoder setting")
  ("ECU",                                             conf.m_bUseEarlyCU,                               "Early CU setting")
  ("FDM",                                             conf.m_useFastDecisionForMerge,                   "Fast decision for Merge RD Cost")
  ("ESD",                                             conf.m_useEarlySkipDetection,                     "Early SKIP detection setting")

  ("DisableIntraInInter",                             conf.m_bDisableIntraCUsInInterSlices,             "Flag to disable intra CUs in inter slices")
  ("ConstrainedIntraPred",                            conf.m_bUseConstrainedIntraPred,                  "Constrained Intra Prediction")
  ("FastUDIUseMPMEnabled",                            conf.m_bFastUDIUseMPMEnabled,                     "If enabled, adapt intra direction search, accounting for MPM")
  ("FastMEForGenBLowDelayEnabled",                    conf.m_bFastMEForGenBLowDelayEnabled,             "If enabled use a fast ME for generalised B Low Delay slices")

  ("MTSImplicit",                                     conf.m_MTSImplicit,                               "Enable implicit MTS when explicit MTS is off\n")
  ("TMVPMode",                                        conf.m_TMVPModeId,                                "TMVP mode enable(0: off 1: for all slices 2: for certain slices only)")
  ("DepQuant",                                        conf.m_DepQuantEnabled,                           "Enable dependent quantization" )
  ("DQThrVal",                                        conf.m_dqThresholdVal,                            "Quantization threshold value for DQ last coefficient search" )
  ("SignHideFlag",                                    conf.m_SignDataHidingEnabled,                     "Enable sign data hiding" )
  ("MIP",                                             conf.m_MIP,                                       "Enable MIP (matrix-based intra prediction)")
  ("FastMIP",                                         conf.m_useFastMIP,                                "Fast encoder search for MIP (matrix-based intra prediction)")
  ("MaxNumMergeCand",                                 conf.m_maxNumMergeCand,                           "Maximum number of merge candidates")
  ("MaxNumAffineMergeCand",                           conf.m_maxNumAffineMergeCand,                     "Maximum number of affine merge candidates")
  ("Geo",                                             conf.m_Geo,                                       "Enable geometric partitioning mode (0:off, 1:on)")
  ("MaxNumGeoCand",                                   conf.m_maxNumGeoCand,                             "Maximum number of geometric partitioning mode candidates")
  ("FastIntraTools",                                  conf.m_FastIntraTools,                            "SpeedUPIntraTools:LFNST,ISP,MTS. (0:off, 1:speed1, 2:speed2)")
  ;

  // motion search options
  opts.setSubSection("Motion search options");
  opts.addOptions()
  ("FastSearch",                                      conf.m_motionEstimationSearchMethod,              "Serach mode (0:Full search 1:Diamond 2:Selective 3:Enhanced Diamond)")
  ("RestrictMESampling",                              conf.m_bRestrictMESampling,                       "Enable restrict ME Sampling for selective inter motion search")
  ("SearchRange,-sr",                                 conf.m_SearchRange,                               "Motion search range")
  ("BipredSearchRange",                               conf.m_bipredSearchRange,                         "Motion search range for bipred refinement")
  ("MinSearchWindow",                                 conf.m_minSearchWindow,                           "Minimum motion search window size for the adaptive window ME")
  ("ClipForBiPredMEEnabled",                          conf.m_bClipForBiPredMeEnabled,                   "Enable clipping in the Bi-Pred ME.")
  ("FastMEAssumingSmootherMVEnabled",                 conf.m_bFastMEAssumingSmootherMVEnabled,          "Enable fast ME assuming a smoother MV.")
  ("FastSubPel",                                      conf.m_fastSubPel,                                "Enable fast sub-pel ME")
  ;

  // Deblocking filter parameters
  opts.setSubSection("Loop filters (deblock and SAO)");
  opts.addOptions()
  ("LoopFilterDisable",                               conf.m_bLoopFilterDisable,                        "")
  ("LoopFilterOffsetInPPS",                           conf.m_loopFilterOffsetInPPS,                     "")
  ("LoopFilterBetaOffset_div2",                       conf.m_loopFilterBetaOffsetDiv2[0],               "")
  ("LoopFilterTcOffset_div2",                         conf.m_loopFilterTcOffsetDiv2[0],                 "")
  ("LoopFilterCbBetaOffset_div2",                     conf.m_loopFilterBetaOffsetDiv2[1],               "")
  ("LoopFilterCbTcOffset_div2",                       conf.m_loopFilterTcOffsetDiv2[1],                 "")
  ("LoopFilterCrBetaOffset_div2",                     conf.m_loopFilterBetaOffsetDiv2[2],               "")
  ("LoopFilterCrTcOffset_div2",                       conf.m_loopFilterTcOffsetDiv2[2],                 "")
  ("DeblockingFilterMetric",                          conf.m_deblockingFilterMetric,                    "")

  ("LFCrossTileBoundaryFlag",                         conf.m_bLFCrossTileBoundaryFlag,                  "Enable cross-tile-boundary loop filtering")
  ("LFCrossSliceBoundaryFlag",                        conf.m_bLFCrossSliceBoundaryFlag,                 "Enable cross-slice-boundary loop filtering")
  ("LoopFilterAcrossTileGroupsEnabled",               conf.m_loopFilterAcrossSlicesEnabled,             "Enable loop filtering across tile groups")

  ("SAO",                                             conf.m_bUseSAO,                                   "Enable Sample Adaptive Offset")
  ("SaoEncodingRate",                                 conf.m_saoEncodingRate,                           "When >0 SAO early picture termination is enabled for luma and chroma")
  ("SaoEncodingRateChroma",                           conf.m_saoEncodingRateChroma,                     "The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma")
  ("SaoLumaOffsetBitShift",                           conf.m_saoOffsetBitShift[ 0 ],                    "Specify the luma SAO bit-shift. If negative, automatically calculate a suitable value based upon bit depth and initial QP")
  ("SaoChromaOffsetBitShift",                         conf.m_saoOffsetBitShift[ 1 ],                    "Specify the chroma SAO bit-shift. If negative, automatically calculate a suitable value based upon bit depth and initial QP")
  ;

  opts.setSubSection("VUI and SEI options");
  opts.addOptions()
  ("SEIDecodedPictureHash,-dph",                      toHashType,                                       "Control generation of decode picture hash SEI messages, (0:off, 1:md5, 2:crc, 3:checksum)" )
  ("SEIBufferingPeriod",                              conf.m_bufferingPeriodSEIEnabled,                 "Control generation of buffering period SEI messages")
  ("SEIPictureTiming",                                conf.m_pictureTimingSEIEnabled,                   "Control generation of picture timing SEI messages")
  ("SEIDecodingUnitInfo",                             conf.m_decodingUnitInfoSEIEnabled,                "Control generation of decoding unit information SEI message.")
  ("EnableDecodingParameterSet",                      conf.m_decodingParameterSetEnabled,               "Enable writing of Decoding Parameter Set")
  ("AccessUnitDelimiter,-aud",                        toAud,                                            "Enable Access Unit Delimiter NALUs, (default: auto - enable only if needed by dependent options)" , true)
  ("VuiParametersPresent,-vui",                       toVui,                                            "Enable generation of vui_parameters(), (default: auto - enable only if needed by dependent options)", true)
  ("HrdParametersPresent,-hrd",                       toHrd,                                            "Enable generation of hrd_parameters(), (default: auto - enable only if needed by dependent options)", true)
  ("AspectRatioInfoPresent",                          conf.m_aspectRatioInfoPresent,                    "Signals whether aspect_ratio_idc is present")
  ("AspectRatioIdc",                                  conf.m_aspectRatioIdc,                            "aspect_ratio_idc")
  ("SarWidth",                                        conf.m_sarWidth,                                  "horizontal size of the sample aspect ratio")
  ("SarHeight",                                       conf.m_sarHeight,                                 "vertical size of the sample aspect ratio")
  ("ColourDescriptionPresent",                        conf.m_colourDescriptionPresent,                  "Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present")
  ("ColourPrimaries",                                 toColorPrimaries,                                 "Specify color primaries (0-13): reserved, bt709, unknown, empty, bt470m, bt470bg, smpte170m, "
                                                                                                        "smpte240m, film, bt2020, smpte428, smpte431, smpte432")

  ("TransferCharacteristics",                         toTransferCharacteristics,                        "Specify opto-electroni transfer characteristics (0-18): reserved, bt709, unknown, empty, bt470m, bt470bg, smpte170m, "
                                                                                                        "smpte240m, linear, log100, log316, iec61966-2-4, bt1361e, iec61966-2-1, "
                                                                                                        "bt2020-10, bt2020-12, smpte2084, smpte428, arib-std-b67")
  ("MatrixCoefficients",                              toColorMatrix,                                    "Specify color matrix setting to derive luma/chroma from RGB primaries (0-14): gbr, bt709, unknown, empty, fcc, bt470bg, smpte170m, "
                                                                                                        "smpte240m, ycgco, bt2020nc, bt2020c, smpte2085, chroma-derived-nc, chroma-derived-c, ictcp")

  ("ChromaLocInfoPresent",                            conf.m_chromaLocInfoPresent,                      "Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present")
  ("ChromaSampleLocTypeTopField",                     conf.m_chromaSampleLocTypeTopField,               "Specifies the location of chroma samples for top field")
  ("ChromaSampleLocTypeBottomField",                  conf.m_chromaSampleLocTypeBottomField,            "Specifies the location of chroma samples for bottom field")
  ("ChromaSampleLocType",                             conf.m_chromaSampleLocType,                       "Specifies the location of chroma samples for progressive content")
  ("OverscanInfoPresent",                             conf.m_overscanInfoPresent,                       "Indicates whether conformant decoded pictures are suitable for display using overscan")
  ("OverscanAppropriate",                             conf.m_overscanAppropriateFlag,                   "Indicates whether conformant decoded pictures are suitable for display using overscan")
  ("VideoSignalTypePresent",                          conf.m_videoSignalTypePresent,                    "Signals whether video_format, video_full_range_flag, and colour_description_present_flag are present")
  ("VideoFullRange",                                  conf.m_videoFullRangeFlag,                        "Indicates the black level and range of luma and chroma signals")


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
  ("SummaryOutFilename",                              conf.m_summaryOutFilename,                  "Filename to use for producing summary output file. If empty, do not produce a file.")
  ("SummaryPicFilenameBase",                          conf.m_summaryPicFilenameBase,              "Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended. If empty, do not produce a file.")
  ("SummaryVerboseness",                              conf.m_summaryVerboseness,                  "Specifies the level of the verboseness of the text output")
  ;

  opts.setSubSection("Decoding options (debugging)");
  opts.addOptions()
  ("DebugBitstream",                                  conf.m_decodeBitstreams[0],                 "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DecodeBitstream1",                                conf.m_decodeBitstreams[0],                 "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DecodeBitstream2",                                conf.m_decodeBitstreams[1],                 "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DebugPOC",                                        conf.m_switchPOC,                           "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
  ("SwitchPOC",                                       conf.m_switchPOC,                           "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
  ("SwitchDQP",                                       conf.m_switchDQP,                           "delta QP applied to picture with switchPOC and subsequent pictures." )
  ("FastForwardToPOC",                                conf.m_fastForwardToPOC,                    "Get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC." )
  ("StopAfterFFtoPOC",                                conf.m_stopAfterFFtoPOC,                    "If using fast forward to POC, after the POC of interest has been hit, stop further encoding.")
  ("DecodeBitstream2ModPOCAndType",                   conf.m_bs2ModPOCAndType,                    "Modify POC and NALU-type of second input bitstream, to use second BS as closing I-slice")
  ("ForceDecodeBitstream1",                           conf.m_forceDecodeBitstream1,               "force decoding of bitstream 1 - use this only if you are realy sure about what you are doing ")
  ;

  opts.setSubSection("Coding tools");
  opts.addOptions()
  ("SMVD",                                            conf.m_SMVD,                                "Enable Symmetric MVD (0:off 1:vtm 2:fast 3:faster\n")
  ("AMVR",                                            conf.m_AMVRspeed,                           "Enable Adaptive MV precision Mode (IMV)")
  // added for backward compatibility with VTM
  ("IMV",                                             conf.m_AMVRspeed,                           "Enable Adaptive MV precision Mode (IMV)")
  ("LMChroma",                                        conf.m_LMChroma,                            "Enable LMChroma prediction")
  ("MRL",                                             conf.m_MRL,                                 "MultiRefernceLines")
  ("BDOF",                                            conf.m_BDOF,                                "Enable bi-directional optical flow")
  // added for backward compatibility with VTM
  ("BIO",                                             conf.m_BDOF,                                "Enable bi-directional optical flow")
  ("DMVR",                                            conf.m_DMVR,                                "Decoder-side Motion Vector Refinement")
  ("EncDbOpt",                                        conf.m_EDO,                                 "Encoder optimization with deblocking filter 0:off 1:vtm 2:fast")
  ("EDO",                                             conf.m_EDO,                                 "Encoder optimization with deblocking filter 0:off 1:vtm 2:fast")
  ("LMCSEnable",                                      conf.m_lumaReshapeEnable,                   "Enable LMCS (luma mapping with chroma scaling")
  ("LMCS",                                            conf.m_lumaReshapeEnable,                   "Enable LMCS (luma mapping with chroma scaling")
  ("LMCSSignalType",                                  conf.m_reshapeSignalType,                   "Input signal type (0:SDR, 1:HDR-PQ, 2:HDR-HLG)")
  ("LMCSUpdateCtrl",                                  conf.m_updateCtrl,                          "LMCS model update control (0:RA, 1:AI, 2:LDB/LDP)")
  ("LMCSAdpOption",                                   conf.m_adpOption,                           "LMCS adaptation options: 0:automatic,"
                                                                                                  "1: rsp both (CW66 for QP<=22), 2: rsp TID0 (for all QP),"
                                                                                                  "3: rsp inter(CW66 for QP<=22), 4: rsp inter(for all QP).")
  ("LMCSInitialCW",                                   conf.m_initialCW,                           "LMCS initial total codeword (0~1023) when LMCSAdpOption > 0")
  ("LMCSOffset",                                      conf.m_LMCSOffset,                          "LMCS chroma residual scaling offset")
  ("ALF",                                             conf.m_alf,                                 "Adpative Loop Filter\n" )
  ("CCALF",                                           conf.m_ccalf,                               "Cross-component Adaptive Loop Filter" )
  ("CCALFQpTh",                                       conf.m_ccalfQpThreshold,                    "QP threshold above which encoder reduces CCALF usage")
  ("UseNonLinearAlfLuma",                             conf.m_useNonLinearAlfLuma,                 "Non-linear adaptive loop filters for Luma Channel")
  ("UseNonLinearAlfChroma",                           conf.m_useNonLinearAlfChroma,               "Non-linear adaptive loop filters for Chroma Channels")
  ("MaxNumAlfAlternativesChroma",                     conf.m_maxNumAlfAlternativesChroma,         std::string("Maximum number of alternative Chroma filters (1-") + std::to_string(VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA) + std::string (", inclusive)") )
  ("ALFTempPred",                                     conf.m_alfTempPred,                         "Enable usage of ALF temporal prediction for filter data\n" )
  ("PROF",                                            conf.m_PROF,                                "Enable prediction refinement with optical flow for affine mode")
  ("Affine",                                          conf.m_Affine,                              "Enable affine prediction")
  ("AffineType",                                      conf.m_AffineType,                          "Enable affine type prediction")
  ("MMVD",                                            conf.m_MMVD,                                "Enable Merge mode with Motion Vector Difference")
  ("MmvdDisNum",                                      conf.m_MmvdDisNum,                          "Number of MMVD Distance Entries")
  ("AllowDisFracMMVD",                                conf.m_allowDisFracMMVD,                    "Disable fractional MVD in MMVD mode adaptively")
  ("MCTF",                                            conf.m_vvencMCTF.MCTF,                      "Enable GOP based temporal filter. (0:off, 1:filter all frames, 2:use SCC detection to disable for screen coded content)")
  ("MCTFFutureReference",                             conf.m_vvencMCTF.MCTFFutureReference,       "Enable referencing of future frames in the GOP based temporal filter. This is typically disabled for Low Delay configurations.")
  ("MCTFNumLeadFrames",                               conf.m_vvencMCTF.MCTFNumLeadFrames,         "Number of additional MCTF lead frames, which will not be encoded, but can used for MCTF filtering")
  ("MCTFNumTrailFrames",                              conf.m_vvencMCTF.MCTFNumTrailFrames,        "Number of additional MCTF trail frames, which will not be encoded, but can used for MCTF filtering")
  ("MCTFFrame",                                       toMCTFFrames,                               "Frame to filter Strength for frame in GOP based temporal filter")
  ("MCTFStrength",                                    toMCTFStrengths,                            "Strength for  frame in GOP based temporal filter.")

  ("FastLocalDualTreeMode",                           conf.m_fastLocalDualTreeMode,               "Fast intra pass coding for local dual-tree in intra coding region (0:off, 1:use threshold, 2:one intra mode only)")
  ("QtbttExtraFast",                                  conf.m_qtbttSpeedUp,                        "Non-VTM compatible QTBTT speed-ups" )
  ;

  opts.setSubSection("Threading, performance");
  opts.addOptions()
  ("MaxParallelFrames",                               conf.m_maxParallelFrames,                   "Maximum number of frames to be processed in parallel(0:off, >=2: enable parallel frames)")
  ("WppBitEqual",                                     conf.m_ensureWppBitEqual,                   "Ensure bit equality with WPP case (0:off (sequencial mode), 1:copy from wpp line above, 2:line wise reset)")
  ("EnablePicPartitioning",                           conf.m_picPartitionFlag,                    "Enable picture partitioning (0: single tile, single slice, 1: multiple tiles/slices)")
  ;

  opts.setSubSection("Coding tools");
  opts.addOptions()
  ("SbTMVP",                                          conf.m_SbTMVP,                              "Enable Subblock Temporal Motion Vector Prediction (0: off, 1: on)")
  ("CIIP",                                            conf.m_CIIP,                                "Enable CIIP mode, 0: off, 1: vtm, 2: fast, 3: faster ")
  ("SBT",                                             conf.m_SBT,                                 "Enable Sub-Block Transform for inter blocks (0: off 1: vtm, 2: fast, 3: faster)" )
  ("LFNST",                                           conf.m_LFNST,                               "Enable LFNST (0: off, 1: on)" )
  ("MTS",                                             conf.m_MTS,                                 "Multiple Transform Set (MTS)" )
  ("MTSIntraMaxCand",                                 conf.m_MTSIntraMaxCand,                     "Number of additional candidates to test for MTS in intra slices")
  ("ISP",                                             conf.m_ISP,                                 "Intra Sub-Partitions Mode (0: off, 1: vtm, 2: fast, 3: faster)")
  ("TransformSkip",                                   conf.m_TS,                                  "Intra transform skipping, 0: off, 1: TS, 2: TS with SCC detection ")
  ("TransformSkipLog2MaxSize",                        conf.m_TSsize,                              "Specify transform-skip maximum size. Minimum 2, Maximum 5")
  ("ChromaTS",                                        conf.m_useChromaTS,                         "Enable encoder search of chromaTS")
  ("BDPCM",                                           conf.m_useBDPCM,                            "BDPCM (0:off, 1:luma and chroma, 2: BDPCM with SCC detection)")
  ("RPR",                                             conf.m_rprEnabledFlag,                      "Reference Sample Resolution (0: disable, 1: eneabled, 2: RPR ready")
#if 1 // IBC_VTM
  ( "IBC",                                            conf.m_IBCMode,                             "IBCMode (0:off, 1:IBC, 2: IBC with SCC detection)")
  ( "IBCFastMethod",                                  conf.m_IBCFastMethod,                       "Fast methods for IBC . 1:default, [2..5]speedups")
#endif
  ;

  opts.setSubSection("Input options");
  opts.addOptions()
  ("HorCollocatedChroma",                             conf.m_horCollocatedChromaFlag,             "Specifies location of a chroma sample relatively to the luma sample in horizontal direction in the reference picture resampling"
                                                                                                  "(0: horizontally shifted by 0.5 units of luma samples, 1: collocated)")
  ("VerCollocatedChroma",                             conf.m_verCollocatedChromaFlag,             "Specifies location of a chroma sample relatively to the luma sample in vertical direction in the cross-component linear model intra prediction and the reference picture resampling"
                                                                                                  "(0: horizontally co-sited, vertically shifted by 0.5 units of luma samples, 1: collocated)")
  ("ClipInputVideoToRec709Range",                     m_bClipInputVideoToRec709Range,             "Enable clipping input video to the Rec. 709 Range on loading when InternalBitDepth is less than MSBExtendedBitDepth")
  ;

  opts.setSubSection("Reconstructed video options");
  opts.addOptions()
  ("ClipOutputVideoToRec709Range",                    m_bClipOutputVideoToRec709Range,            "Enable clipping output video to the Rec. 709 Range on saving when OutputBitDepth is less than InternalBitDepth")
  ("PYUV",                                            m_packedYUVMode,                            "Enable output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data. Ignored for interlaced output.")
    ;


  int  cropOffsetLeft;
  int  cropOffsetTop;
  int  cropOffsetRight;
  int  cropOffsetBottom;
  bool calculateHdrMetrics;
  bool signalledSliceIdFlag;
  int  signalledSliceIdLengthMinus1;
  std::vector<int>    rectSliceBoundary;                                       // unused, only for conmpatiblity
  std::vector<int>    signalledSliceId;                                        // unused, only for conmpatiblity

  IStreamToVec<int>            toRectSliceBoundary          ( &rectSliceBoundary     );
  IStreamToVec<int>            toSignalledSliceId           ( &signalledSliceId      );

  bool sdr; // unused
  opts.setSubSection("Unused options (only for compatiblity to VTM)");
  opts.addOptions()
  ("EnablePictureHeaderInSliceHeader",                this->conf.m_enablePictureHeaderInSliceHeader,    "Enable Picture Header in Slice Header")

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
    opts.addOptions()(cOSS.str(), conf.m_GOPList[i], vvencGOPEntry() );
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

  //
  // check own parameters
  //
  bool error = false;
  if( m_bitstreamFileName.empty() )
  {
    cout <<  "error: A bitstream file name must be specified (BitstreamFile)" << std::endl;
    error = true;
  }
  else
  {
    if( conf.m_decodeBitstreams[0] != nullptr && conf.m_decodeBitstreams[0] == m_bitstreamFileName )
    {
      cout <<  "error: Debug bitstream and the output bitstream cannot be equal" << std::endl;
      error = true;
    }
    if( conf.m_decodeBitstreams[1] != nullptr && conf.m_decodeBitstreams[1] == m_bitstreamFileName )
    {
      cout <<  "error: Decode2 bitstream and the output bitstream cannot be equal" << std::endl;
      error = true;
    }
  }

  if( m_inputFileChromaFormat < 0 || m_inputFileChromaFormat >= VVENC_NUM_CHROMA_FORMAT )
  {
    cout <<  "error: Input chroma format must be either 400, 420, 422 or 444" << std::endl;
    error = true;
  }

  if ( error )
  {
    return false;
  }

  if( m_decode )
  {
    return true;
  }

  //
  // set intern derived parameters (for convenience purposes only)
  //

  if ( conf.m_internChromaFormat < 0 || conf.m_internChromaFormat >= VVENC_NUM_CHROMA_FORMAT )
  {
    conf.m_internChromaFormat = m_inputFileChromaFormat;
  }

  if( conf.m_RCNumPasses < 0 )
  {
    conf.m_RCNumPasses = conf.m_RCTargetBitrate > 0 ? 2 : 1;
  }

  if( m_packedYUVMode && ! m_reconFileName.empty() )  
  {
    if( ( conf.m_outputBitDepth[ 0 ] != 10 && conf.m_outputBitDepth[ 0 ] != 12 )
        || ( ( ( conf.m_SourceWidth ) & ( 1 + ( conf.m_outputBitDepth[ 0 ] & 3 ) ) ) != 0 ) )
    {
      cout <<  "error: Invalid output bit-depth or image width for packed YUV output, aborting" << std::endl;
      error = true;
    }
    if( ( conf.m_internChromaFormat != VVENC_CHROMA_400 ) && ( ( conf.m_outputBitDepth[ 1 ] != 10 && conf.m_outputBitDepth[ 1 ] != 12 )
          || ( ( vvenc_get_width_of_component( conf.m_internChromaFormat, conf.m_SourceWidth, 1 ) & ( 1 + ( conf.m_outputBitDepth[ 1 ] & 3 ) ) ) != 0 ) ) )
    {
      cout <<  "error: Invalid chroma output bit-depth or image width for packed YUV output, aborting" << std::endl;
      error = true;
    }

    if ( error )
    {
      return false;
    }
  }

//  if ( vvenc_initCfgParameter() )
//  {
//    return false;
//  }

  return true;
}

std::string VVEncAppCfg::getConfigAsString( vvencMsgLevel eMsgLevel ) const
{
  std::stringstream css;
  if( eMsgLevel >= VVENC_DETAILS )
  {
    css << "Input          File                    : " << m_inputFileName << "\n";
    css << "Bitstream      File                    : " << m_bitstreamFileName << "\n";
    css << "Reconstruction File                    : " << m_reconFileName << "\n";
  }

  std::string config ( vvenc_get_config_as_string( (VVEncCfg*)&conf, eMsgLevel) );
  css << config;
  css << "\n";

  return css.str();
}

} // namespace

//! \}


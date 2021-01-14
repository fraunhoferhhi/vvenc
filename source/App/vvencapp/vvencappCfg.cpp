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


/** \file     vvencappCfg.cpp
    \brief    Handle encoder configuration parameters
*/

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <string>
#include <fstream>
#include <algorithm>
#include "apputils/ParseArg.h"
#include "apputils/IStreamIO.h"
#include "vvenc/vvenc.h"
#include "vvencappCfg.h"

#define MACRO_TO_STRING_HELPER(val) #val
#define MACRO_TO_STRING(val) MACRO_TO_STRING_HELPER(val)

using namespace std;
namespace po = apputils::df::program_options_lite;

//! \ingroup EncoderApp
//! \{
//!
//!
//// ====================================================================================================================
//// string <-> enum
//// ====================================================================================================================


void setPresets( EncCfg* cfg, int preset )
{
  cfg->initPreset( (PresetMode)preset );
}

// ====================================================================================================================
// string <-> enum fixed mappings
// ====================================================================================================================
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
  { "2",                       Level::LEVEL2   },
  { "2.1",                     Level::LEVEL2_1 },
  { "3",                       Level::LEVEL3   },
  { "3.1",                     Level::LEVEL3_1 },
  { "4",                       Level::LEVEL4   },
  { "4.1",                     Level::LEVEL4_1 },
  { "5",                       Level::LEVEL5   },
  { "5.1",                     Level::LEVEL5_1 },
  { "5.2",                     Level::LEVEL5_2 },
  { "6",                       Level::LEVEL6   },
  { "6.1",                     Level::LEVEL6_1 },
  { "6.2",                     Level::LEVEL6_2 },
  { "6.3",                     Level::LEVEL6_3 },
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


enum vvencappCfgBitDepth
{
  BITDEPTH_8            = 8,
  BITDEPTH_10           = 10
};

const std::vector<SVPair<vvencappCfgBitDepth>> InputBitColorSpaceToIntMap =
{
  { "yuv420",                    BITDEPTH_8 },
  { "yuv420_10",                 BITDEPTH_10 },
};


const std::vector<SVPair<DecodingRefreshType>> DecodingRefreshTypeToEnumMap =
{
  { "cra",                     DRT_CRA },
  { "idr",                     DRT_IDR },
 // { "rpsei",                     DRT_RECOVERY_POINT_SEI },
};


void setBitDepth( EncCfg* cfg, int bitdepth )
{
  cfg->m_inputBitDepth[0] = bitdepth;
}


// ====================================================================================================================
// Public member functions
// ====================================================================================================================

vvencappCfg::~vvencappCfg()
{
}


/** \param  argc        number of arguments
    \param  argv        array of arguments
    \retval             true when success
 */
bool vvencappCfg::parseCfg( int argc, char* argv[] )
{
  bool do_help                = false;
  bool do_expert_help         = false;
  int  warnUnknowParameter    = 0;

  //
  // link custom formated configuration parameters with istream reader
  //
  IStreamToFunc<PresetMode>    toPreset                     ( setPresets, this, &PresetToEnumMap,PresetMode::MEDIUM);
  IStreamToRefVec<int>         toSourceSize                 ( { &m_SourceWidth, &m_SourceHeight }, true, 'x' );

  IStreamToEnum<Profile>       toProfile                    ( &m_profile,                     &ProfileToEnumMap      );
  IStreamToEnum<Tier>          toLevelTier                  ( &m_levelTier,                   &TierToEnumMap         );
  IStreamToEnum<Level>         toLevel                      ( &m_level,                       &LevelToEnumMap        );
  IStreamToEnum<CostMode>      toCostMode                   ( &m_costMode,                    &CostModeToEnumMap     );
  IStreamToEnum<ChromaFormat>  toInputFileCoFormat          ( &m_inputFileChromaFormat,       &ChromaFormatToEnumMap  );
  IStreamToEnum<ChromaFormat>  toInternCoFormat             ( &m_internChromaFormat,          &ChromaFormatToEnumMap  );
  IStreamToEnum<HashType>      toHashType                   ( &m_decodedPictureHashSEIType,   &HashTypeToEnumMap     );
  IStreamToEnum<SegmentMode>   toSegment                    ( &m_SegmentMode, &SegmentToEnumMap );

  IStreamToFunc<vvencappCfgBitDepth> toInputBitdepth        ( setBitDepth, this, &InputBitColorSpaceToIntMap, BITDEPTH_8);
  IStreamToEnum<DecodingRefreshType> toDecRefreshType       ( &m_DecodingRefreshType, &DecodingRefreshTypeToEnumMap );

  //
  // setup configuration parameters
  //

  std::string ignoreParams;
  po::Options opts;
  opts.addOptions()
  ("help",                                            do_help,                                          "this help text")
  ("fullhelp",                                        do_expert_help,                                   "expert help text")

  ("input,i",                                         m_inputFileName,                                  "original YUV input file name")
  ("size,s",                                          toSourceSize,                                     "specify input resolution (WidthxHeight)")
  ("format,c",                                        toInputBitdepth,                                  "set input format (yuv420, yuv420_10)")

  ("framerate,r",                                     m_FrameRate,                                      "temporal rate (framerate) e.g. 25,29,30,50,59,60 ")

  ("frames,f",                                        m_framesToBeEncoded,                              "max. frames to encode (default=all)")
  ("frameskip",                                       m_FrameSkip,                                      "Number of frames to skip at start of input YUV")
  ("segment",                                         toSegment,                                        "when encoding multiple separate segments, specify segment position to enable segment concatenation (first, mid, last) [off]")

  ("tickspersec",                                     m_TicksPerSecond,                                 "Ticks Per Second for dts generation, ( 1..27000000)")

  ("output,o",                                        m_bitstreamFileName,                              "Bitstream output file name")

  ("preset",                                          toPreset,                                         "select preset for specific encoding setting (faster, fast, medium, slow, slower)")

  ("bitrate,b",                                       m_RCTargetBitrate,                                "bitrate for rate control (0: constant-QP encoding without rate control, otherwise bits/second)" )
  ("passes,p",                                        m_RCNumPasses,                                    "number of rate control passes (1,2) " )
  ("qp,q",                                            m_QP,                                             "quantization parameter, QP (0-63)")
  ("qpa",                                             m_usePerceptQPA,                                  "Mode of perceptually motivated QP adaptation (0:off, 1:SDR-WPSNR, 2:SDR-XPSNR, 3:HDR-WPSNR, 4:HDR-XPSNR 5:HDR-MeanLuma)")

  ("threads,-t",                                      m_numWppThreads,                                  "Number of threads")

  ("gopsize,g",                                       m_GOPSize,                                        "GOP size of temporal structure (16,32)")
  ("refreshtype,-rt",                                 toDecRefreshType,                                 "intra refresh type (idr,cra)")
  ("refreshsec,-rs",                                  m_IntraPeriodSec,                                 "Intra period in seconds")
  ("intraperiod,-ip",                                 m_IntraPeriod,                                    "Intra period in frames, (-1: only first frame)")


  ("profile",                                         toProfile,                                        "select profile (main10, main10_stillpic)")
  ("level",                                           toLevel,                                          "Level limit to be used, eg 5.1, or none")
  ("tier",                                            toLevelTier,                                      "Tier to use for interpretation of level (main or high)")


  ("Verbosity,v",                                     m_verbosity,                                      "Specifies the level of the verboseness")
   ;

  po::setDefaults( opts );
  std::ostringstream easyOpts;
  po::doHelp( easyOpts, opts );

  opts.addOptions()

  ("SIMD",                                            ignoreParams,                                           "SIMD extension to use (SCALAR, SSE41, SSE42, AVX, AVX2, AVX512), default: the highest supported extension")
  ("internal-bitdepth",                               m_internalBitDepth[0],                            "internal bitdepth (8,10)")
  ;

  if ( vvenc::VVEnc::isTracingEnabled() )
  {
     opts.addOptions()
    ("TraceChannelsList",                             m_listTracingChannels,                            "List all available tracing channels")
    ("TraceRule",                                     m_traceRule,                                      "Tracing rule (ex: \"D_CABAC:poc==8\" or \"D_REC_CB_LUMA:poc==8\")")
    ("TraceFile",                                     m_traceFile,                                      "Tracing file");
  }
  opts.addOptions()
  ("AccessUnitDelimiter",                             m_AccessUnitDelimiter,                            "Enable Access Unit Delimiter NALUs")
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
  for ( list<const char*>::const_iterator it = argv_unhandled.begin(); it != argv_unhandled.end(); it++ )
  {
    msgApp( vvenc::ERROR, "Unhandled argument ignored: `%s'\n", *it);
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

  //
  // check own parameters
  //

  m_confirmFailed = false;
  confirmParameter( m_bitstreamFileName.empty(),                  "A bitstream file name must be specified (--output=bit.266)" );

  if ( m_confirmFailed )
  {
    return false;
  }


  //
  // set intern derived parameters (for convenience purposes only)
  //

  if( m_numWppThreads < 0 )
  {
    if( m_SourceWidth > 1920 || m_SourceHeight > 1080)
    {
      m_numWppThreads = 6;
    }
    else
    {
      m_numWppThreads = 4;
    }
    m_ensureWppBitEqual = 1;
  }

  if(  m_RCTargetBitrate )
  {
    m_RCRateControlMode     = RateControlMode::RCM_PICTURE_LEVEL;
    m_RCKeepHierarchicalBit = 2;
    m_RCUseLCUSeparateModel = 1;
    m_RCInitialQP           = 0;
    m_RCForceIntraQP        = 0;
  }

  if ( m_internChromaFormat < 0 || m_internChromaFormat >= NUM_CHROMA_FORMAT )
  {
    m_internChromaFormat = m_inputFileChromaFormat;
  }

  //
  // setup encoder configuration
  //

  if ( EncCfg::initCfgParameter() )
  {
    return false;
  }


  return true;
}

void vvencappCfg::msgFnc( int level, const char* fmt, va_list args ) const
{
  if ( m_verbosity >= level )
  {
    vfprintf( level == 1 ? stderr : stdout, fmt, args );
  }
}

void vvencappCfg::msgApp( int level, const char* fmt, ... ) const
{
    va_list args;
    va_start( args, fmt );
    msgFnc( level, fmt, args );
    va_end( args );
}

void vvencappCfg::printCfg() const
{
  msgApp( DETAILS, "Input          File                    : %s\n", m_inputFileName.c_str() );
  msgApp( DETAILS, "Bitstream      File                    : %s\n", m_bitstreamFileName.c_str() );

  vvenc::EncCfg::printCfg();
  msgApp( NOTICE, "\n");

  fflush( stdout );
}

void vvencappCfg::printAppCfgOnly() const
{
  msgApp( DETAILS, "Input          File                    : %s\n", m_inputFileName.c_str() );
  msgApp( DETAILS, "Bitstream      File                    : %s\n", m_bitstreamFileName.c_str() );

  fflush( stdout );
}

//! \}


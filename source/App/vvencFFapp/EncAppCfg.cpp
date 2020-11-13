/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVenc

(c) Copyright (2019-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVenc (“Fraunhofer Versatile Video Encoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Encoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Encoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Encoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Encoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Encoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Encoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Encoding Library. If You use the Fraunhofer Versatile Video Encoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Encoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Encoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Encoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Encoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
----------------------------------------------------------------------------- */


/** \file     EncAppCfg.cpp
    \brief    Handle encoder configuration parameters
*/

#include "../vvencFFapp/EncAppCfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <string>
#include <fstream>
#include <algorithm>
#include "../vvencFFapp/EncApp.h"
#include "../vvencFFapp/ParseArg.h"

#define MACRO_TO_STRING_HELPER(val) #val
#define MACRO_TO_STRING(val) MACRO_TO_STRING_HELPER(val)

using namespace std;
namespace po = VVCEncoderFFApp::df::program_options_lite;

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// string <-> enum
// ====================================================================================================================


template<typename E>
struct SVPair
{
  const char* str;
  E           value;
};

template<typename T>
class IStreamToRefVec
{
  public:
    IStreamToRefVec( std::vector<T*> v, bool _allRequired, char _sep = 'x' )
      : valVec( v )
      , sep( _sep)
      , allRequired( _allRequired)
    {
    }

    ~IStreamToRefVec()
    {
    }

    template<typename F>
    friend std::istream& operator >> ( std::istream& in, IStreamToRefVec<F>& toVec );

    template<typename F>
    friend std::ostream& operator << ( std::ostream& os, const IStreamToRefVec<F>& toVec );

  private:
    std::vector<T*> valVec;
    char sep;
    bool allRequired;
};

template<typename T>
inline std::istream& operator >> ( std::istream& in, IStreamToRefVec<T>& toVec )
{
  const size_t maxSize = toVec.valVec.size();
  size_t idx = 0;
  bool fail = false;
  // split into multiple lines if any
  while ( ! in.eof() )
  {
    string line;
    std::getline( in, line );
    // treat all whitespaces and commas as valid separators
    if( toVec.sep == 'x')
      std::replace_if( line.begin(), line.end(), []( int c ){ return isspace( c ) || c == 'x'; }, ' ' );
    else
      std::replace_if( line.begin(), line.end(), []( int c ){ return isspace( c ) || c == ','; }, ' ' );
    std::stringstream tokenStream( line );
    std::string token;
    // split into multiple tokens if any
    while( std::getline( tokenStream, token, ' ' ) )
    {
      if ( ! token.length() )
        continue;
      // convert to value
      std::stringstream convStream( token );
      T val;
      convStream >> val;
      fail |= convStream.fail();
      if( idx >= maxSize )
      {
        fail = true;//try to write behind buffer
      }
      else
      {
        *toVec.valVec[idx++] =  val;
      }
    }
  }

  if ( fail || (toVec.allRequired && idx != maxSize) )
  {
    in.setstate( ios::failbit );
  }

  return in;
}

template<typename T>
inline std::ostream& operator << ( std::ostream& os, const IStreamToRefVec<T>& toVec )
{
  bool bfirst = true;
  for( auto& e: toVec.valVec )
  {
    if( bfirst )
    {
      bfirst = false;
    }
    else
    {
      os << toVec.sep;
    }
    os << *e;
  }
  return os;
}


template<typename E>
class IStreamToEnum
{
  public:
    IStreamToEnum( E* d, const std::vector<SVPair<E>>* m )
      : dstVal ( d )
        , toMap( m )
    {
    }

    ~IStreamToEnum()
    {
    }

    template<typename F>
    friend std::ostream& operator << ( std::ostream& os, const IStreamToEnum<F>& toEnum );

    template<typename F>
    friend std::istream& operator >> ( std::istream& in, IStreamToEnum<F>& toEnum );

    const char* to_string() const
    {
      for ( const auto& map : *toMap )
      {
        if ( *dstVal == map.value )
        {
          return map.str;
        }
      }
      msgApp( ERROR, "Unknown enum \"%s\" in to_string", *dstVal );
      return "";
    }

  private:
    E*                            dstVal;
    const std::vector<SVPair<E>>* toMap;
};

template<typename E>
inline std::istream& operator >> ( std::istream& in, IStreamToEnum<E>& toEnum )
{
  std::string str;
  in >> str;

  for ( const auto& map : *toEnum.toMap )
  {
    if ( str == map.str )
    {
      *toEnum.dstVal = map.value;
      return in;
    }
  }

  /* not found */
  in.setstate( ios::failbit );
  return in;
}

template<typename E>
inline std::ostream& operator << ( std::ostream& os, const IStreamToEnum<E>& toEnum )
{
  for ( const auto& map : *toEnum.toMap )
  {
    if ( *toEnum.dstVal == map.value )
    {
      os << map.str;
      return os;
    }
  }

  /* not found */
  os.setstate( ios::failbit );
  return os;
}

typedef void (*setParamFunc) (EncCfg*, int);

template<typename E>
class IStreamToFunc
{
  public:
    IStreamToFunc( setParamFunc func, EncCfg* encCfg, const std::vector<SVPair<E>>* m, const E _default )
      : mfunc( func )
      , mencCfg( encCfg )
      , toMap( m )
      , dstVal( _default )
    {
    }

    ~IStreamToFunc()
    {
    }

    template<typename F>
    friend std::istream& operator >> ( std::istream& in, IStreamToFunc<F>& toEnum );

    template<typename F>
    friend std::ostream& operator << ( std::ostream& in, const IStreamToFunc<F>& toEnum );

    const char* to_string() const
    {
      return "";
    }

  private:
    setParamFunc                  mfunc;
    EncCfg*                       mencCfg;
    const std::vector<SVPair<E>>* toMap;
    E                             dstVal;
};

template<typename E>
inline std::istream& operator >> ( std::istream& in, IStreamToFunc<E>& toEnum )
{
  std::string str;
  in >> str;

  for ( const auto& map : *toEnum.toMap )
  {
    if ( str == map.str )
    {
      toEnum.dstVal = map.value;
      toEnum.mfunc(toEnum.mencCfg, map.value);
      return in;
    }
  }

  /* not found */
  in.setstate( ios::failbit );
  return in;
}

template<typename F>
inline std::ostream& operator << ( std::ostream& os, const IStreamToFunc<F>& toEnum )
{
  for ( const auto& map : *toEnum.toMap )
  {
    if ( toEnum.dstVal == map.value )
    {
      os << map.str;
      return os;
    }
  }

  /* not found */
  os.setstate( ios::failbit );
  return os;
}

enum  MyPreset
{
 NONE      = -1,
 FASTER    = 0,
 FAST      = 1,
 MEDIUM    = 2,
 SLOW      = 3,
 TOOLTEST  = 255,
};

void setPresets( EncCfg* cfg, int preset)
{
  cfg->initPreset( preset );
}

// ====================================================================================================================
// string <-> enum fixed mappings
// ====================================================================================================================
const std::vector<SVPair<MyPreset>> PresetToEnumMap =
{
  { "none",                        MyPreset::NONE },
  { "faster",                      MyPreset::FASTER },
  { "fast",                        MyPreset::FAST },
  { "medium",                      MyPreset::MEDIUM },
  { "slow",                        MyPreset::SLOW },
  { "tooltest",                    MyPreset::TOOLTEST },
};


const std::vector<SVPair<Profile::Name>> ProfileToEnumMap =
{
  { "none",                                  Profile::NONE },
  { "main_10",                               Profile::MAIN_10 },
  { "main_10_444",                           Profile::MAIN_10_444 },
  { "main_10_still_picture",                 Profile::MAIN_10_STILL_PICTURE },
  { "main_10_444_still_picture",             Profile::MAIN_10_444_STILL_PICTURE },
  { "multilayer_main_10",                    Profile::MULTILAYER_MAIN_10 },
  { "multilayer_main_10_444",                Profile::MULTILAYER_MAIN_10_444 },
  { "multilayer_main_10_still_picture",      Profile::MULTILAYER_MAIN_10_STILL_PICTURE },
  { "multilayer_main_10_444_still_picture",  Profile::MULTILAYER_MAIN_10_444_STILL_PICTURE },
  { "auto",                                  Profile::AUTO }
};

const std::vector<SVPair<Level::Name>> LevelToEnumMap =
{
  { "none",                    Level::NONE     },
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
};

const std::vector<SVPair<Level::Tier>> TierToEnumMap =
{
  { "main",                    Level::MAIN },
  { "high",                    Level::HIGH },
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

// ====================================================================================================================
// string -> list
// ====================================================================================================================


template<typename T>
class IStreamToVec
{
  public:
    IStreamToVec( std::vector<T>* v )
      : valVec( v )
    {
    }

    ~IStreamToVec()
    {
    }

    template<typename F>
    friend std::istream& operator >> ( std::istream& in, IStreamToVec<F>& toVec );

    template<typename F>
    friend std::ostream& operator << ( std::ostream& in, const IStreamToVec<F>& toVec );

  private:
    std::vector<T>* valVec;
};

template<typename T>
inline std::istream& operator >> ( std::istream& in, IStreamToVec<T>& toVec )
{
  std::vector<T>* valVec = toVec.valVec;
  valVec->clear();

  bool fail = false;
  // split into multiple lines if any
  while ( ! in.eof() )
  {
    string line;
    std::getline( in, line );
    // treat all whitespaces and commas as valid separators
    std::replace_if( line.begin(), line.end(), []( int c ){ return isspace( c ) || c == ','; }, ' ' );
    std::stringstream tokenStream( line );
    std::string token;
    // split into multiple tokens if any
    while( std::getline( tokenStream, token, ' ' ) )
    {
      if ( ! token.length() )
        continue;
      // convert to value
      std::stringstream convStream( token );
      T val;
      convStream >> val;
      fail |= convStream.fail();
      valVec->push_back( val );
    }
  }

  if ( fail || ! valVec->size() )
  {
    in.setstate( ios::failbit );
  }

  return in;
}

template<typename T>
inline std::ostream& operator << ( std::ostream& os, const IStreamToVec<T>& toVec )
{
  bool bfirst = true;
  for( auto& e : (*(toVec.valVec)))
  {
    if( bfirst )
    {
      bfirst = false;
    }
    else
    {
      os << ",";
    }
    os << e;
  }

  return os;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================  virtual ~EncAppCfg()
EncAppCfg::~EncAppCfg()
{
}


/** \param  argc        number of arguments
    \param  argv        array of arguments
    \retval             true when success
 */
bool EncAppCfg::parseCfg( int argc, char* argv[] )
{
  bool do_help                = false;
  bool do_expert_help         = false;
  int  warnUnknowParameter    = 0;

  //
  // link custom formated configuration parameters with istream reader
  //
  IStreamToFunc<MyPreset>      toPreset                     ( setPresets, this, &PresetToEnumMap,MyPreset::MEDIUM);
  IStreamToRefVec<int>         toSourceSize                 ( { &m_SourceWidth, &m_SourceHeight }, true, 'x' );
  IStreamToRefVec<double>      toLambdaModifier             ( { &m_adLambdaModifier[0], &m_adLambdaModifier[1], &m_adLambdaModifier[2], &m_adLambdaModifier[3], &m_adLambdaModifier[4], &m_adLambdaModifier[5], &m_adLambdaModifier[6] }, false );

  IStreamToEnum<Profile::Name> toProfile                    ( &m_profile,                     &ProfileToEnumMap      );
  IStreamToEnum<Level::Tier>   toLevelTier                  ( &m_levelTier,                   &TierToEnumMap         );
  IStreamToEnum<Level::Name>   toLevel                      ( &m_level,                       &LevelToEnumMap        );
  IStreamToEnum<CostMode>      toCostMode                   ( &m_costMode,                    &CostModeToEnumMap     );
  IStreamToEnum<ChromaFormat>  toInputFileCoFormat          ( &m_inputFileChromaFormat,       &ChromaFormatToEnumMap  );
  IStreamToEnum<ChromaFormat>  toInternCoFormat             ( &m_internChromaFormat,          &ChromaFormatToEnumMap  );
  IStreamToEnum<HashType>      toHashType                   ( &m_decodedPictureHashSEIType,   &HashTypeToEnumMap     );
  IStreamToVec<int>            toQpInCb                     ( &m_qpInValsCb            );
  IStreamToVec<int>            toQpOutCb                    ( &m_qpOutValsCb           );
  IStreamToVec<int>            toQpInCr                     ( &m_qpInValsCr            );
  IStreamToVec<int>            toQpOutCr                    ( &m_qpOutValsCr           );
  IStreamToVec<int>            toQpInCbCr                   ( &m_qpInValsCbCr          );
  IStreamToVec<int>            toQpOutCbCr                  ( &m_qpOutValsCbCr         );
  IStreamToVec<double>         toIntraLambdaModifier        ( &m_adIntraLambdaModifier );
#if NOT_USED
  IStreamToVec<int>            toTileColumnWidth            ( &m_tileColumnWidth       );
  IStreamToVec<int>            toTileRowHeight              ( &m_tileRowHeight         );
#endif
  IStreamToVec<int>            toRectSliceBoundary          ( &m_rectSliceBoundary     );
  IStreamToVec<int>            toSignalledSliceId           ( &m_signalledSliceId      );

  IStreamToVec<int>            toMCTFFrames                 ( &m_MCTFFrames   );
  IStreamToVec<double>         toMCTFStrengths              ( &m_MCTFStrengths );

  //
  // setup configuration parameters
  //

  std::string ignore;
  po::Options opts;
  
  opts.addOptions()

  ("help",                                            do_help,                                          "this help text")
  ("fullhelp",                                        do_expert_help,                                   "expert help text")

  ("InputFile,i",                                     m_inputFileName,                                  "Original YUV input file name")
  ("BitstreamFile,b",                                 m_bitstreamFileName,                              "Bitstream output file name")
  ("ReconFile,o",                                     m_reconFileName,                                  "Reconstructed YUV output file name")

  ("FramesToBeEncoded,f",                             m_framesToBeEncoded,                              "Number of frames to be encoded (default=all)")
  ("FrameRate,-fr",                                   m_FrameRate,                                      "Frame rate")
  ("FrameSkip,-fs",                                   m_FrameSkip,                                      "Number of frames to skip at start of input YUV")
  ("TicksPerSecond",                                  m_TicksPerSecond,                                 "Ticks Per Second for dts generation, ( 1..27000000)")

  ("Profile",                                         toProfile,                                        "Profile name to use for encoding. Use [multilayer_]main_10[_444][_still_picture], auto, or none")
  ("Tier",                                            toLevelTier,                                      "Tier to use for interpretation of level (main or high)")
  ("Level",                                           toLevel,                                          "Level limit to be used, eg 5.1, or none")

  ("IntraPeriod,-ip",                                 m_IntraPeriod,                                    "Intra period in frames, (-1: only first frame)")
  ("DecodingRefreshType,-dr",                         m_DecodingRefreshType,                            "Intra refresh type (0:none, 1:CRA, 2:IDR, 3:RecPointSEI)")
  ("GOPSize,g",                                       m_GOPSize,                                        "GOP size of temporal structure")

  ("InputBitDepth",                                   m_inputBitDepth[ CH_L ],                          "Bit-depth of input file")
  ("OutputBitDepth",                                  m_outputBitDepth[ CH_L ],                         "Bit-depth of output file")

  ("PerceptQPA,-qpa",                                 m_usePerceptQPA,                                  "Mode of perceptually motivated QP adaptation (0:off, 1:SDR-WPSNR, 2:SDR-XPSNR, 3:HDR-WPSNR, 4:HDR-XPSNR 5:HDR-MeanLuma)")
  ("PerceptQPATempFiltIPic",                          m_usePerceptQPATempFiltISlice,                    "Temporal high-pass filter in QPA activity calculation for I Pictures (0:off, 1:on)")
  
  ("Verbosity,v",                                     m_verbosity,                                      "Specifies the level of the verboseness")
  ("Size,-s",                                         toSourceSize,                                     "Input resolution (WidthxHeight)")
  ("Threads,-t",                                      m_numWppThreads,                                  "Number of threads")
  ("preset",                                          toPreset,                                         "select preset for specific encoding setting (faster, fast, medium, slow)")
    ;

  po::setDefaults( opts );
  std::ostringstream easyOpts;
  po::doHelp( easyOpts, opts );

  opts.addOptions()

  ("c",                                               po::parseConfigFile,                              "configuration file name")
  ("WarnUnknowParameter,w",                           warnUnknowParameter,                              "warn for unknown configuration parameters instead of failing")
  ("SIMD",                                            ignore,                                           "SIMD extension to use (SCALAR, SSE41, SSE42, AVX, AVX2, AVX512), default: the highest supported extension")

  // file, i/o and source parameters

  ("SourceWidth,-wdt",                                m_SourceWidth,                                    "Source picture width")
  ("SourceHeight,-hgt",                               m_SourceHeight,                                   "Source picture height");

  if ( vvenc::isTracingEnabled() )
  {
     opts.addOptions()
    ("TraceChannelsList",                             m_listTracingChannels,                            "List all available tracing channels")
    ("TraceRule",                                     m_traceRule,                                      "Tracing rule (ex: \"D_CABAC:poc==8\" or \"D_REC_CB_LUMA:poc==8\")")
    ("TraceFile",                                     m_traceFile,                                      "Tracing file");
  }
  opts.addOptions()

  ("ConformanceWindowMode",                           m_conformanceWindowMode,                          "Window conformance mode (0:off, 1:automatic padding, 2:padding, 3:conformance")
  ("ConfWinLeft",                                     m_confWinLeft,                                    "Left offset for window conformance mode 3")
  ("ConfWinRight",                                    m_confWinRight,                                   "Right offset for window conformance mode 3")
  ("ConfWinTop",                                      m_confWinTop,                                     "Top offset for window conformance mode 3")
  ("ConfWinBottom",                                   m_confWinBottom,                                  "Bottom offset for window conformance mode 3")

  ("TemporalSubsampleRatio,-ts",                      m_temporalSubsampleRatio,                         "Temporal sub-sample ratio when reading input YUV")

  ("HorizontalPadding,-pdx",                          m_aiPad[0],                                       "Horizontal source padding for conformance window mode 2")
  ("VerticalPadding,-pdy",                            m_aiPad[1],                                       "Vertical source padding for conformance window mode 2")
  ("EnablePictureHeaderInSliceHeader",                m_enablePictureHeaderInSliceHeader,               "Enable Picture Header in Slice Header")
  ("AccessUnitDelimiter",                             m_AccessUnitDelimiter,                            "Enable Access Unit Delimiter NALUs")

  ("MSEBasedSequencePSNR",                            m_printMSEBasedSequencePSNR,                      "Emit sequence PSNR (0: only as a linear average of the frame PSNRs, 1: also based on an average of the frame MSEs")
  ("PrintHexPSNR",                                    m_printHexPsnr,                                   "Emit hexadecimal PSNR for each frame (0: off , 1:on")
  ("PrintFrameMSE",                                   m_printFrameMSE,                                  "Emit MSE values for each frame (0: off , 1:on")
  ("PrintSequenceMSE",                                m_printSequenceMSE,                               "Emit MSE values for the whole sequence (0: off , 1:on)")
  ("CabacZeroWordPaddingEnabled",                     m_cabacZeroWordPaddingEnabled,                    "Add conforming cabac-zero-words to bit streams (0: do not add, 1: add as required)")

  // Profile and level
  ("SubProfile",                                      m_subProfile,                                     "Sub-profile idc")
  ("MaxBitDepthConstraint",                           m_bitDepthConstraintValue,                        "Bit depth to use for profile-constraint for RExt profiles. (0: automatically choose based upon other parameters)")
  ("IntraConstraintFlag",                             m_intraOnlyConstraintFlag,                        "Value of general_intra_constraint_flag to use for RExt profiles (not used if an explicit RExt sub-profile is specified)")

  // Coding structure paramters
  ("InputQueueSize",                                  m_InputQueueSize,                                 "Size of input frames queue (use gop size)")
  ("ReWriteParamSets",                                m_rewriteParamSets,                               "Enable rewriting of Parameter sets before every (intra) random access point")
  ("IDRRefParamList",                                 m_idrRefParamList,                                "Enable indication of reference picture list syntax elements in slice headers of IDR pictures")

  /* Quantization parameters */
  ("QP,q",                                            m_QP,                                             "Qp value")
  ("SameCQPTablesForAllChroma",                       m_useSameChromaQPTables,                          "0: Different tables for Cb, Cr and joint Cb-Cr components, 1 (default): Same tables for all three chroma components")
  ("IntraQPOffset",                                   m_intraQPOffset,                                  "Qp offset value for intra slice, typically determined based on GOP size")
  ("LambdaFromQpEnable",                              m_lambdaFromQPEnable,                             "Enable flag for derivation of lambda from QP")
  ("LambdaModifier,-LM",                              toLambdaModifier,                                 "Lambda modifier list for temporal layers. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifierI,-LMI",                            toIntraLambdaModifier,                            "Lambda modifiers for Intra pictures, comma separated, up to one the number of temporal layer. If entry for temporalLayer exists, then use it, else if some are specified, use the last, else use the standard LambdaModifiers.")
  ("IQPFactor,-IQF",                                  m_dIntraQpFactor,                                 "Intra QP Factor for Lambda Computation. If negative, the default will scale lambda based on GOP size (unless LambdaFromQpEnable then IntraQPOffset is used instead)")
  ("QpInValCb",                                       toQpInCb,                                         "Input coordinates for the QP table for Cb component")
  ("QpInValCr",                                       toQpInCr,                                         "Input coordinates for the QP table for Cr component")
  ("QpInValCbCr",                                     toQpInCbCr,                                       "Input coordinates for the QP table for joint Cb-Cr component")
  ("QpOutValCb",                                      toQpOutCb,                                        "Output coordinates for the QP table for Cb component")
  ("QpOutValCr",                                      toQpOutCr,                                        "Output coordinates for the QP table for Cr component")
  ("QpOutValCbCr",                                    toQpOutCbCr,                                      "Output coordinates for the QP table for joint Cb-Cr component")
  ("MaxCuDQPSubdiv,-dqd",                             m_cuQpDeltaSubdiv,                                "Maximum subdiv for CU luma Qp adjustment")
  ("MaxCuChromaQpOffsetSubdiv",                       m_cuChromaQpOffsetSubdiv,                         "Maximum subdiv for CU chroma Qp adjustment - set less than 0 to disable")
  ("CbQpOffset,-cbqpofs",                             m_chromaCbQpOffset,                               "Chroma Cb QP Offset")
  ("CrQpOffset,-crqpofs",                             m_chromaCrQpOffset,                               "Chroma Cr QP Offset")
  ("CbQpOffsetDualTree",                              m_chromaCbQpOffsetDualTree,                       "Chroma Cb QP Offset for dual tree")
  ("CrQpOffsetDualTree",                              m_chromaCrQpOffsetDualTree,                       "Chroma Cr QP Offset for dual tree")
  ("CbCrQpOffset,-cbcrqpofs",                         m_chromaCbCrQpOffset,                             "QP Offset for joint Cb-Cr mode")
  ("CbCrQpOffsetDualTree",                            m_chromaCbCrQpOffsetDualTree,                     "QP Offset for joint Cb-Cr mode in dual tree")
  ("SliceChromaQPOffsetPeriodicity",                  m_sliceChromaQpOffsetPeriodicity,                 "Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.")
  ("SliceCbQpOffsetIntraOrPeriodic",                  m_sliceChromaQpOffsetIntraOrPeriodic[0],          "Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")
  ("SliceCrQpOffsetIntraOrPeriodic",                  m_sliceChromaQpOffsetIntraOrPeriodic[1],          "Chroma Cr QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")

  ("LumaLevelToDeltaQPMode",                          m_lumaLevelToDeltaQPEnabled,                      "Luma based Delta QP 0(default): not used. 1: Based on CTU average")
  ("isSDR",                                           m_sdr,                                            "compatibility")
  ("WCGPPSEnable",                                    m_wcgChromaQpControl.enabled,                     "1: Enable the WCG PPS chroma modulation scheme. 0 (default) disabled")
  ("WCGPPSCbQpScale",                                 m_wcgChromaQpControl.chromaCbQpScale,             "WCG PPS Chroma Cb QP Scale")
  ("WCGPPSCrQpScale",                                 m_wcgChromaQpControl.chromaCrQpScale,             "WCG PPS Chroma Cr QP Scale")
  ("WCGPPSChromaQpScale",                             m_wcgChromaQpControl.chromaQpScale,               "WCG PPS Chroma QP Scale")
  ("WCGPPSChromaQpOffset",                            m_wcgChromaQpControl.chromaQpOffset,              "WCG PPS Chroma QP Offset")
  ("CropOffsetLeft",                                  m_cropOffsetLeft,                                 "Crop Offset Left position")
  ("CropOffsetTop",                                   m_cropOffsetTop,                                  "Crop Offset Top position")
  ("CropOffsetRight",                                 m_cropOffsetRight,                                "Crop Offset Right position")
  ("CropOffsetBottom",                                m_cropOffsetBottom,                               "Crop Offset Bottom position")
  ("CalculateHdrMetrics",                             m_calculateHdrMetrics,                            "Enable HDR metric calculation")
  ;

  opts.addOptions()

  ("InputChromaFormat",                               toInputFileCoFormat,                              "input file chroma format (400, 420, 422, 444)")
  ("ChromaFormatIDC,-cf",                             toInternCoFormat,                                 "intern chroma format (400, 420, 422, 444) or set to 0 (default), same as InputChromaFormat")
  ("UseIdentityTableForNon420Chroma",                 m_useIdentityTableForNon420Chroma,                "True: Indicates that 422/444 chroma uses identity chroma QP mapping tables; False: explicit Qp table may be specified in config")
  ("InputBitDepthC",                                  m_inputBitDepth[ CH_C ],                          "As per InputBitDepth but for chroma component. (default:InputBitDepth)")
  ("InternalBitDepth",                                m_internalBitDepth[ CH_L ],                       "Bit-depth the codec operates at. (default: MSBExtendedBitDepth). If different to MSBExtendedBitDepth, source data will be converted")
  ("OutputBitDepthC",                                 m_outputBitDepth[ CH_C ],                         "As per OutputBitDepth but for chroma component. (default: use luma output bit-depth)")
  ("MSBExtendedBitDepth",                             m_MSBExtendedBitDepth[ CH_L ],                    "bit depth of luma component after addition of MSBs of value 0 (used for synthesising High Dynamic Range source material). (default:InputBitDepth)")
  ("MSBExtendedBitDepthC",                            m_MSBExtendedBitDepth[ CH_C ],                    "As per MSBExtendedBitDepth but for chroma component. (default:MSBExtendedBitDepth)")
  ("CostMode",                                        toCostMode,                                       "Use alternative cost functions: choose between 'lossy', 'sequence_level_lossless', 'lossless' (which forces QP to " MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP) ") and 'mixed_lossless_lossy' (which used QP'=" MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME) " for pre-estimates of transquant-bypass blocks).")
  ("SEIDecodedPictureHash,-dph",                      toHashType,                                       "Control generation of decode picture hash SEI messages, (0:off, 1:md5, 2:crc, 3:checksum)" )
  ("SEIBufferingPeriod",                              m_bufferingPeriodSEIEnabled,                      "Control generation of buffering period SEI messages")
  ("SEIPictureTiming",                                m_pictureTimingSEIEnabled,                        "Control generation of picture timing SEI messages")
  ("SEIDecodingUnitInfo",                             m_decodingUnitInfoSEIEnabled,                     "Control generation of decoding unit information SEI message.")
#if NOT_USED
  ("TileUniformSpacing",                              m_tileUniformSpacingFlag,                         "Indicates that tile columns and rows are distributed uniformly")
  ("NumTileColumnsMinus1",                            m_numTileColumnsMinus1,                           "Number of tile columns in a picture minus 1")
  ("NumTileRowsMinus1",                               m_numTileRowsMinus1,                              "Number of rows in a picture minus 1")
  ("TileColumnWidthArray",                            toTileColumnWidth,                                "Array containing tile column width values in units of CTU")
  ("TileRowHeightArray",                              toTileRowHeight,                                  "Array containing tile row height values in units of CTU")
#endif
  ("WaveFrontSynchro",                                m_entropyCodingSyncEnabled,                       "Enable entropy coding sync")
  ("EntryPointsPresent",                              m_entryPointsPresent,                             "Enable entry points in slice header")
#if NOT_USED
  ("RectSliceFlag",                                   m_rectSliceFlag,                                  "Rectangular slice flag")
//  ("NumRectSlicesInPicMinus1",                        m_numSlicesInPicMinus1,                           "Number slices in pic minus 1")
#endif
  ("SignalledIdFlag",                                 m_signalledSliceIdFlag,                           "Signalled Slice ID Flag")
  ("SignalledSliceIdLengthMinus1",                    m_signalledSliceIdLengthMinus1,                   "Signalled Tile Group Length minus 1")
  ("RectSlicesBoundaryArray",                         toRectSliceBoundary,                              "Rectangular slices boundaries in Pic")
  ("SignalledSliceId",                                toSignalledSliceId,                               "Signalled rectangular slice ID")

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
  ("ECU",                                             m_bUseEarlyCU,                                    "Early CU setting")
  ("FDM",                                             m_useFastDecisionForMerge,                        "Fast decision for Merge RD Cost")
  ("ESD",                                             m_useEarlySkipDetection,                          "Early SKIP detection setting")

  ("DisableIntraInInter",                             m_bDisableIntraCUsInInterSlices,                  "Flag to disable intra CUs in inter slices")
  ("ConstrainedIntraPred",                            m_bUseConstrainedIntraPred,                       "Constrained Intra Prediction")
  ("FastUDIUseMPMEnabled",                            m_bFastUDIUseMPMEnabled,                          "If enabled, adapt intra direction search, accounting for MPM")
  ("FastMEForGenBLowDelayEnabled",                    m_bFastMEForGenBLowDelayEnabled,                  "If enabled use a fast ME for generalised B Low Delay slices")

  ("MTSImplicit",                                     m_MTSImplicit,                                    "Enable implicit MTS when explicit MTS is off\n")
  ("TMVPMode",                                        m_TMVPModeId,                                     "TMVP mode enable(0: off 1: for all slices 2: for certain slices only)")
  ("DepQuant",                                        m_DepQuantEnabled,                                "Enable dependent quantization" )
  ("DQThrVal",                                        m_dqThresholdVal,                                 "Quantization threshold value for DQ last coefficient search" )
  ("SignHideFlag,-SBH",                               m_SignDataHidingEnabled,                          "Enable sign data hiding" )
  ("MIP",                                             m_MIP,                                            "Enable MIP (matrix-based intra prediction)")
  ("FastMIP",                                         m_useFastMIP,                                     "Fast encoder search for MIP (matrix-based intra prediction)")
  ("MaxNumMergeCand",                                 m_maxNumMergeCand,                                "Maximum number of merge candidates")
  ("MaxNumAffineMergeCand",                           m_maxNumAffineMergeCand,                          "Maximum number of affine merge candidates")
//  ("MaxNumIBCMergeCand",                              m_maxNumIBCMergeCand,                             "Maximum number of IBC merge candidates")
  ("Geo",                                             m_Geo,                                            "Enable geometric partitioning mode (0:off, 1:on)")
  ("MaxNumGeoCand",                                   m_maxNumGeoCand,                                  "Maximum number of geometric partitioning mode candidates")
  ("RateControl",                                     m_RCRateControlMode,                              "Rate control: enable rate control (0:off 1:CTU-level RC; 2:picture-level RC; 3:GOP-level RC)" )
  ("TargetBitrate",                                   m_RCTargetBitrate,                                "Rate control: target bit-rate [bps]" )
  ("KeepHierarchicalBit",                             m_RCKeepHierarchicalBit,                          "Rate control: (0:equal bit allocation, 1:fixed ratio bit allocation, 2:adaptive ratio bit allocation" )
  ("RCLCUSeparateModel",                              m_RCUseLCUSeparateModel,                          "Rate control: use CTU level separate R-lambda model" )
  ("InitialQP",                                       m_RCInitialQP,                                    "Rate control: initial QP" )
  ("RCForceIntraQP",                                  m_RCForceIntraQP,                                 "Rate control: force intra QP to be equal to initial QP" )

  // motion search options
  ("FastSearch",                                      m_motionEstimationSearchMethod,                   "Serach mode (0:Full search 1:Diamond 2:Selective 3:Enhanced Diamond)")
  ("RestrictMESampling",                              m_bRestrictMESampling,                            "Enable restrict ME Sampling for selective inter motion search")
  ("SearchRange,-sr",                                 m_SearchRange,                                    "Motion search range")
  ("BipredSearchRange",                               m_bipredSearchRange,                              "Motion search range for bipred refinement")
  ("MinSearchWindow",                                 m_minSearchWindow,                                "Minimum motion search window size for the adaptive window ME")
  ("ClipForBiPredMEEnabled",                          m_bClipForBiPredMeEnabled,                        "Enable clipping in the Bi-Pred ME.")
  ("FastMEAssumingSmootherMVEnabled",                 m_bFastMEAssumingSmootherMVEnabled,               "Enable fast ME assuming a smoother MV.")
  ("FastSubPel",                                      m_fastSubPel,                                     "Enable fast sub-pel ME");

  opts.addOptions()

  // Deblocking filter parameters
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
  ("SaoLumaOffsetBitShift",                           m_saoOffsetBitShift[ CH_L ],                      "Specify the luma SAO bit-shift. If negative, automatically calculate a suitable value based upon bit depth and initial QP")
  ("SaoChromaOffsetBitShift",                         m_saoOffsetBitShift[ CH_C ],                      "Specify the chroma SAO bit-shift. If negative, automatically calculate a suitable value based upon bit depth and initial QP")

  ("EnableDecodingParameterSet",                      m_decodingParameterSetEnabled,                    "Enable writing of Decoding Parameter Set")
  ("VuiParametersPresent,-vui",                       m_vuiParametersPresent,                           "Enable generation of vui_parameters()")
  ("HrdParametersPresent,-hrd",                       m_hrdParametersPresent,                           "Enable generation of hrd_parameters()")
  ("AspectRatioInfoPresent",                          m_aspectRatioInfoPresent,                         "Signals whether aspect_ratio_idc is present")
  ("AspectRatioIdc",                                  m_aspectRatioIdc,                                 "aspect_ratio_idc")
  ("SarWidth",                                        m_sarWidth,                                       "horizontal size of the sample aspect ratio")
  ("SarHeight",                                       m_sarHeight,                                      "vertical size of the sample aspect ratio")
  ("ColourDescriptionPresent",                        m_colourDescriptionPresent,                       "Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present")
  ("ColourPrimaries",                                 m_colourPrimaries,                                "Indicates chromaticity coordinates of the source primaries")
  ("TransferCharacteristics",                         m_transferCharacteristics,                        "Indicates the opto-electronic transfer characteristics of the source")
  ("MatrixCoefficients",                              m_matrixCoefficients,                             "Describes the matrix coefficients used in deriving luma and chroma from RGB primaries")
  ("ChromaLocInfoPresent",                            m_chromaLocInfoPresent,                           "Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present")
  ("ChromaSampleLocTypeTopField",                     m_chromaSampleLocTypeTopField,                    "Specifies the location of chroma samples for top field")
  ("ChromaSampleLocTypeBottomField",                  m_chromaSampleLocTypeBottomField,                 "Specifies the location of chroma samples for bottom field")
  ("ChromaSampleLocType",                             m_chromaSampleLocType,                            "Specifies the location of chroma samples for progressive content")
  ("OverscanInfoPresent",                             m_overscanInfoPresent,                            "Indicates whether conformant decoded pictures are suitable for display using overscan")
  ("OverscanAppropriate",                             m_overscanAppropriateFlag,                        "Indicates whether conformant decoded pictures are suitable for display using overscan")
  ("VideoSignalTypePresent",                          m_videoSignalTypePresent,                         "Signals whether video_format, video_full_range_flag, and colour_description_present_flag are present")
  ("VideoFullRange",                                  m_videoFullRangeFlag,                             "Indicates the black level and range of luma and chroma signals")

  ("SummaryOutFilename",                              m_summaryOutFilename,                             "Filename to use for producing summary output file. If empty, do not produce a file.")
  ("SummaryPicFilenameBase",                          m_summaryPicFilenameBase,                         "Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended. If empty, do not produce a file.")
  ("SummaryVerboseness",                              m_summaryVerboseness,                             "Specifies the level of the verboseness of the text output")

  ("DebugBitstream",                                  m_decodeBitstreams[0],                            "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DecodeBitstream1",                                m_decodeBitstreams[0],                            "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DecodeBitstream2",                                m_decodeBitstreams[1],                            "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DebugPOC",                                        m_switchPOC,                                      "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
  ("SwitchPOC",                                       m_switchPOC,                                      "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
  ("SwitchDQP",                                       m_switchDQP,                                      "delta QP applied to picture with switchPOC and subsequent pictures." )
  ("FastForwardToPOC",                                m_fastForwardToPOC,                               "Get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC." )
  ("StopAfterFFtoPOC",                                m_stopAfterFFtoPOC,                               "If using fast forward to POC, after the POC of interest has been hit, stop further encoding.")
  ("DecodeBitstream2ModPOCAndType",                   m_bs2ModPOCAndType,                               "Modify POC and NALU-type of second input bitstream, to use second BS as closing I-slice")
  ("ForceDecodeBitstream1",                           m_forceDecodeBitstream1,                          "force decoding of bitstream 1 - use this only if you are realy sure about what you are doing ")
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
  ("LMCSEnable",                                      m_lumaReshapeEnable,                              "Enable LMCS (luma mapping with chroma scaling")
  ("LMCS",                                            m_lumaReshapeEnable,                              "Enable LMCS (luma mapping with chroma scaling")
  ("LMCSSignalType",                                  m_reshapeSignalType,                              "Input signal type (0:SDR, 1:HDR-PQ, 2:HDR-HLG)")
  ("LMCSUpdateCtrl",                                  m_updateCtrl,                                     "LMCS model update control (0:RA, 1:AI, 2:LDB/LDP)")
  ("LMCSAdpOption",                                   m_adpOption,                                      "LMCS adaptation options: 0:automatic,"
                                                                                                        "1: rsp both (CW66 for QP<=22), 2: rsp TID0 (for all QP),"
                                                                                                        "3: rsp inter(CW66 for QP<=22), 4: rsp inter(for all QP).")
  ("LMCSInitialCW",                                   m_initialCW,                                      "LMCS initial total codeword (0~1023) when LMCSAdpOption > 0")
  ("LMCSOffset",                                      m_LMCSOffset,                                     "LMCS chroma residual scaling offset")
  ("ALF",                                             m_alf,                                            "Adpative Loop Filter\n" )
  ("CCALF",                                           m_ccalf,                                          "Cross-component Adaptive Loop Filter" )
  ("CCALFQpTh",                                       m_ccalfQpThreshold,                               "QP threshold above which encoder reduces CCALF usage")
  ("UseNonLinearAlfLuma",                             m_useNonLinearAlfLuma,                            "Non-linear adaptive loop filters for Luma Channel")
  ("UseNonLinearAlfChroma",                           m_useNonLinearAlfChroma,                          "Non-linear adaptive loop filters for Chroma Channels")
  ("MaxNumAlfAlternativesChroma",                     m_maxNumAlfAlternativesChroma,                    std::string("Maximum number of alternative Chroma filters (1-") + std::to_string(MAX_NUM_ALF_ALTERNATIVES_CHROMA) + std::string (", inclusive)") )
  ("PROF",                                            m_PROF,                                           "Enable prediction refinement with optical flow for affine mode")
  ("Affine",                                          m_Affine,                                         "Enable affine prediction")
  ("AffineType",                                      m_AffineType,                                     "Enable affine type prediction")
  ("MMVD",                                            m_MMVD,                                           "Enable Merge mode with Motion Vector Difference")
  ("MmvdDisNum",                                      m_MmvdDisNum,                                     "Number of MMVD Distance Entries")
  ("AllowDisFracMMVD",                                m_allowDisFracMMVD,                               "Disable fractional MVD in MMVD mode adaptively")
  ("MCTF",                                            m_MCTF,                                           "Enable GOP based temporal filter. (0:off, 1:filter all but the first and last frame, 2:filter all frames")
  ("MCTFFutureReference",                             m_MCTFFutureReference,                            "Enable referencing of future frames in the GOP based temporal filter. This is typically disabled for Low Delay configurations.")
  ("MCTFNumLeadFrames",                               m_MCTFNumLeadFrames,                              "Number of additional MCTF lead frames, which will not be encoded, but can used for MCTF filtering")
  ("MCTFNumTrailFrames",                              m_MCTFNumTrailFrames,                             "Number of additional MCTF trail frames, which will not be encoded, but can used for MCTF filtering")
  ("MCTFFrame",                                       toMCTFFrames,                                     "Frame to filter Strength for frame in GOP based temporal filter")
  ("MCTFStrength",                                    toMCTFStrengths,                                  "Strength for  frame in GOP based temporal filter.")

  ("FastLocalDualTreeMode",                           m_fastLocalDualTreeMode,                          "Fast intra pass coding for local dual-tree in intra coding region (0:off, 1:use threshold, 2:one intra mode only)")
  ("QtbttExtraFast",                                  m_qtbttSpeedUp,                                   "Non-VTM compatible QTBTT speed-ups" )

  ("NumWppThreads",                                   m_numWppThreads,                                  "Number of parallel wpp threads")
  ("WppBitEqual",                                     m_ensureWppBitEqual,                              "Ensure bit equality with WPP case (0:off (sequencial mode), 1:copy from wpp line above, 2:line wise reset)")
  ("FrameParallel",                                   m_frameParallel,                                  "Encode multiple frames in parallel (if permitted by GOP structure)")
  ("NumFppThreads",                                   m_numFppThreads,                                  "Number of frame parallel processing threads")
  ("FppBitEqual",                                     m_ensureFppBitEqual,                              "Ensure bit equality with frame parallel processing case")
  ("EnablePicPartitioning",                           m_picPartitionFlag,                               "Enable picture partitioning (0: single tile, single slice, 1: multiple tiles/slices)")
  ("SbTMVP",                                          m_SbTMVP,                                         "Enable Subblock Temporal Motion Vector Prediction (0: off, 1: on)")

  ("CIIP",                                            m_CIIP,                                           "Enable CIIP mode, 0: off, 1: vtm, 2: fast, 3: faster ")
  ("SBT",                                             m_SBT,                                            "Enable Sub-Block Transform for inter blocks (0: off 1: vtm, 2: fast, 3: faster)" )
  ("LFNST",                                           m_LFNST,                                          "Enable LFNST (0: off, 1: on)" )
  ("MTS",                                             m_MTS,                                            "Multiple Transform Set (MTS)" )
  ("MTSIntraMaxCand",                                 m_MTSIntraMaxCand,                                "Number of additional candidates to test for MTS in intra slices")
#if 1//ISP_VVC
  ("ISP",                                             m_ISP,                                            "Intra Sub-Partitions Mode (0: off, 1: vtm, 2: fast, 3: faster)")
#endif
#if 1//TS_VVC
  ("TransformSkip",                                   m_TS,                                             "Intra transform skipping, 0: off, 1: TS, 2: TS with SC detection ")
  ("TransformSkipLog2MaxSize",                        m_TSsize,                                         "Specify transform-skip maximum size. Minimum 2, Maximum 5")
  ("ChromaTS",                                        m_useChromaTS,                                    "Enable encoder search of chromaTS")
#endif

  ("HorCollocatedChroma",                             m_horCollocatedChromaFlag,                        "Specifies location of a chroma sample relatively to the luma sample in horizontal direction in the reference picture resampling"
                                                                                                        "(0: horizontally shifted by 0.5 units of luma samples, 1: collocated)")
  ("VerCollocatedChroma",                             m_verCollocatedChromaFlag,                        "Specifies location of a chroma sample relatively to the luma sample in vertical direction in the cross-component linear model intra prediction and the reference picture resampling"
                                                                                                        "(0: horizontally co-sited, vertically shifted by 0.5 units of luma samples, 1: collocated)")
  ("ClipInputVideoToRec709Range",                     m_bClipInputVideoToRec709Range,                   "Enable clipping input video to the Rec. 709 Range on loading when InternalBitDepth is less than MSBExtendedBitDepth")
  ("ClipOutputVideoToRec709Range",                    m_bClipOutputVideoToRec709Range,                  "Enable clipping output video to the Rec. 709 Range on saving when OutputBitDepth is less than InternalBitDepth")
  ("PYUV",                                            m_packedYUVMode,                                  "Enable output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data. Ignored for interlaced output.")

    ;

  po::setDefaults( opts );
  std::ostringstream fullOpts;
  po::doHelp( fullOpts, opts );

  for ( int i = 0; i < MAX_GOP; i++ )
  {
    std::ostringstream cOSS;
    cOSS << "Frame" << i+1;
    opts.addOptions()(cOSS.str(), m_GOPList[i], GOPEntry());
  }
  opts.addOptions()("decode",                          m_decode,                                         "decode only");

  //
  // parse command line parameters and read configuration files
  //
    
  po::setDefaults( opts );
  po::ErrorReporter err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**) argv, err );
  for ( list<const char*>::const_iterator it = argv_unhandled.begin(); it != argv_unhandled.end(); it++ )
  {
    msgApp( ERROR, "Unhandled argument ignored: `%s'\n", *it);
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

  if( m_decode )
  {
    m_confirmFailed = false;
    confirmParameter( m_bitstreamFileName.empty(),                  "A bitstream file name must be specified (BitstreamFile)" );
    if ( m_confirmFailed )
    {
      return false;
    }
    return true;
  }

  //
  // check own parameters
  //

  m_confirmFailed = false;
  confirmParameter( m_bitstreamFileName.empty(),                  "A bitstream file name must be specified (BitstreamFile)" );
  confirmParameter( m_decodeBitstreams[0] == m_bitstreamFileName, "Debug bitstream and the output bitstream cannot be equal" );
  confirmParameter( m_decodeBitstreams[1] == m_bitstreamFileName, "Decode2 bitstream and the output bitstream cannot be equal" );
  confirmParameter( m_inputFileChromaFormat < 0 || m_inputFileChromaFormat >= NUM_CHROMA_FORMAT,   "Intern chroma format must be either 400, 420, 422 or 444" );
  if ( m_confirmFailed )
  {
    return false;
  }

  //
  // set intern derived parameters (for convenience purposes only)
  //

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

void EncAppCfg::printCfg()
{
  //msgApp( DETAILS, "\n" );
  msgApp( DETAILS, "Input          File                    : %s\n", m_inputFileName.c_str() );
  msgApp( DETAILS, "Bitstream      File                    : %s\n", m_bitstreamFileName.c_str() );
  msgApp( DETAILS, "Reconstruction File                    : %s\n", m_reconFileName.c_str() );
  msgApp( DETAILS, "Real     Format                        : %dx%d %gHz\n", m_SourceWidth - m_confWinLeft - m_confWinRight, m_SourceHeight - m_confWinTop - m_confWinBottom, (double)m_FrameRate / m_temporalSubsampleRatio );
  msgApp( DETAILS, "Internal Format                        : %dx%d %gHz\n", m_SourceWidth, m_SourceHeight, (double)m_FrameRate / m_temporalSubsampleRatio );
  msgApp( DETAILS, "Sequence PSNR output                   : %s\n", ( m_printMSEBasedSequencePSNR ? "Linear average, MSE-based" : "Linear average only" ) );
  msgApp( DETAILS, "Hexadecimal PSNR output                : %s\n", ( m_printHexPsnr ? "Enabled" : "Disabled" ) );
  msgApp( DETAILS, "Sequence MSE output                    : %s\n", ( m_printSequenceMSE ? "Enabled" : "Disabled" ) );
  msgApp( DETAILS, "Frame MSE output                       : %s\n", ( m_printFrameMSE ? "Enabled" : "Disabled" ) );
  msgApp( DETAILS, "Cabac-zero-word-padding                : %s\n", ( m_cabacZeroWordPaddingEnabled ? "Enabled" : "Disabled" ) );
  msgApp( DETAILS, "Frame/Field                            : Frame based coding\n" );
  if ( m_framesToBeEncoded > 0 )
    msgApp( DETAILS, "Frame index                            : %d frames\n", m_framesToBeEncoded );
  else
    msgApp( DETAILS, "Frame index                            : all frames\n" );
  const IStreamToEnum<Profile::Name> toProfile( &m_profile, &ProfileToEnumMap );
  const IStreamToEnum<Level::Name>   toLevel( &m_level, &LevelToEnumMap );
  msgApp( DETAILS, "Profile                                : %s\n", toProfile.to_string() );
  msgApp( DETAILS, "Level                                  : %s\n", toLevel.to_string() );
  msgApp( DETAILS, "CU size / total-depth                  : %d / %d\n", m_CTUSize, m_MaxCodingDepth );
  msgApp( DETAILS, "Max TB size                            : %d\n", 1 << m_log2MaxTbSize );
  msgApp( DETAILS, "Motion search range                    : %d\n", m_SearchRange );
  msgApp( DETAILS, "Intra period                           : %d\n", m_IntraPeriod );
  msgApp( DETAILS, "Decoding refresh type                  : %d\n", m_DecodingRefreshType );
  msgApp( DETAILS, "QP                                     : %d\n", m_QP);
  msgApp( DETAILS, "Percept QPA                            : %d\n", m_usePerceptQPA );
  msgApp( DETAILS, "Max dQP signaling subdiv               : %d\n", m_cuQpDeltaSubdiv);

  msgApp( DETAILS, "Cb QP Offset (dual tree)               : %d (%d)\n", m_chromaCbQpOffset, m_chromaCbQpOffsetDualTree);
  msgApp( DETAILS, "Cr QP Offset (dual tree)               : %d (%d)\n", m_chromaCrQpOffset, m_chromaCrQpOffsetDualTree);
  msgApp( DETAILS, "GOP size                               : %d\n", m_GOPSize );
  msgApp( DETAILS, "Input queue size                       : %d\n", m_InputQueueSize );
  msgApp( DETAILS, "Input bit depth                        : (Y:%d, C:%d)\n", m_inputBitDepth[ CH_L ], m_inputBitDepth[ CH_C ] );
  msgApp( DETAILS, "MSB-extended bit depth                 : (Y:%d, C:%d)\n", m_MSBExtendedBitDepth[ CH_L ], m_MSBExtendedBitDepth[ CH_C ] );
  msgApp( DETAILS, "Internal bit depth                     : (Y:%d, C:%d)\n", m_internalBitDepth[ CH_L ], m_internalBitDepth[ CH_C ] );
  msgApp( DETAILS, "cu_chroma_qp_offset_subdiv             : %d\n", m_cuChromaQpOffsetSubdiv);
  if (m_bUseSAO)
  {
    msgApp( DETAILS, "log2_sao_offset_scale_luma             : %d\n", m_log2SaoOffsetScale[ CH_L ] );
    msgApp( DETAILS, "log2_sao_offset_scale_chroma           : %d\n", m_log2SaoOffsetScale[ CH_C ] );
  }

  switch (m_costMode)
  {
    case COST_STANDARD_LOSSY:               msgApp( DETAILS, "Cost function:                         : Lossy coding (default)\n"); break;
    case COST_SEQUENCE_LEVEL_LOSSLESS:      msgApp( DETAILS, "Cost function:                         : Sequence_level_lossless coding\n"); break;
    case COST_LOSSLESS_CODING:              msgApp( DETAILS, "Cost function:                         : Lossless coding with fixed QP\n"); break;
    case COST_MIXED_LOSSLESS_LOSSY_CODING:  msgApp( DETAILS, "Cost function:                         : Mixed_lossless_lossy coding for lossless evaluation\n"); break;
    default:                                msgApp( DETAILS, "Cost function:                         : Unknown\n"); break;
  }

  msgApp( DETAILS, "\n");

  msgApp( VERBOSE, "TOOL CFG: ");
  msgApp( VERBOSE, "IBD:%d ", ((m_internalBitDepth[ CH_L ] > m_MSBExtendedBitDepth[ CH_L ]) || (m_internalBitDepth[ CH_C ] > m_MSBExtendedBitDepth[ CH_C ])));
  msgApp( VERBOSE, "HAD:%d ", m_bUseHADME                          );
  msgApp( VERBOSE, "RDQ:%d ", m_RDOQ                               );
  msgApp( VERBOSE, "RDQTS:%d ", m_useRDOQTS                        );
  msgApp( VERBOSE, "ASR:%d ", m_bUseASR                            );
  msgApp( VERBOSE, "MinSearchWindow:%d ", m_minSearchWindow        );
  msgApp( VERBOSE, "RestrictMESampling:%d ", m_bRestrictMESampling );
  msgApp( VERBOSE, "FEN:%d ", m_fastInterSearchMode                );
  msgApp( VERBOSE, "ECU:%d ", m_bUseEarlyCU                        );
  msgApp( VERBOSE, "FDM:%d ", m_useFastDecisionForMerge            );
  msgApp( VERBOSE, "ESD:%d ", m_useEarlySkipDetection              );
#if NOT_USED
  msgApp( VERBOSE, "Tiles:%dx%d ", m_numTileColumnsMinus1 + 1, m_numTileRowsMinus1 + 1 );
#endif
  msgApp( VERBOSE, "CIP:%d ", m_bUseConstrainedIntraPred);
  msgApp( VERBOSE, "SAO:%d ", (m_bUseSAO)?(1):(0));
  msgApp( VERBOSE, "ALF:%d ", m_alf ? 1 : 0 );
  if( m_alf )
  {
    msgApp(VERBOSE, "(NonLinLuma:%d ", m_useNonLinearAlfLuma );
    msgApp(VERBOSE, "NonLinChr:%d) ", m_useNonLinearAlfChroma );
  }
  msgApp( VERBOSE, "CCALF:%d ", m_ccalf ? 1 : 0 );
  const int iWaveFrontSubstreams = m_entropyCodingSyncEnabled ? (m_SourceHeight + m_CTUSize - 1) / m_CTUSize : 1;
  msgApp( VERBOSE, "WaveFrontSynchro:%d WaveFrontSubstreams:%d ", m_entropyCodingSyncEnabled?1:0, iWaveFrontSubstreams);
  msgApp( VERBOSE, "TMVPMode:%d ", m_TMVPModeId     );

  msgApp( VERBOSE, "DQ:%d ", m_DepQuantEnabled);
  if( m_DepQuantEnabled ) { if( m_dqThresholdVal & 1 ) msgApp( VERBOSE, "(Thr: %d.5) ", m_dqThresholdVal >> 1 ); else msgApp( VERBOSE, "(Thr: %d) ", m_dqThresholdVal >> 1 ); }
  msgApp( VERBOSE, "SignBitHidingFlag:%d ", m_SignDataHidingEnabled);
  msgApp( VERBOSE, "Perceptual QPA:%d ", m_usePerceptQPA );

//  if( m_profile == Profile::AUTO )
  {
    msgApp( VERBOSE, "\nNEXT TOOL CFG: " );
    msgApp( VERBOSE, "DualITree:%d ",          m_dualITree );
    msgApp( VERBOSE, "BIO:%d ",                m_BDOF );
    msgApp( VERBOSE, "DMVR:%d ",               m_DMVR );
    msgApp( VERBOSE, "MTSImplicit:%d ",        m_MTSImplicit );
    msgApp( VERBOSE, "SBT:%d ",                m_SBT );
    msgApp( VERBOSE, "JointCbCr:%d ",          m_JointCbCrMode );
    msgApp( VERBOSE, "CabacInitPresent:%d ",   m_cabacInitPresent );
    msgApp( VERBOSE, "AMVRspeed:%d ",          m_AMVRspeed );
    msgApp( VERBOSE, "SMVD:%d ",               m_SMVD );

    msgApp(VERBOSE, "Reshape:%d ", m_lumaReshapeEnable);
    if (m_lumaReshapeEnable)
    {
      msgApp(VERBOSE, "(Signal:%s ", m_reshapeSignalType == 0 ? "SDR" : (m_reshapeSignalType == 2 ? "HDR-HLG" : "HDR-PQ"));
      msgApp(VERBOSE, "Opt:%d", m_adpOption);
      if (m_adpOption > 0) { msgApp(VERBOSE, " CW:%d", m_initialCW); }
      msgApp(VERBOSE, ") ");
    }
    msgApp(VERBOSE, "CIIP:%d ", m_CIIP);
    msgApp(VERBOSE, "MIP:%d ", m_MIP);
    msgApp(VERBOSE, "EncDbOpt:%d ", m_EDO);
    msgApp(VERBOSE, "MCTF:%d ", m_MCTF);
    if ( m_MCTF )
    {
      msgApp(VERBOSE, "[L:%d, T:%d] ", m_MCTFNumLeadFrames, m_MCTFNumTrailFrames);
    }
    msgApp( VERBOSE, "Affine:%d ",             m_Affine);
    msgApp( VERBOSE, "Affine_Prof:%d ",        m_PROF);
    msgApp( VERBOSE, "Affine_Type:%d ",        m_AffineType);
    msgApp( VERBOSE, "MMVD:%d ",               m_MMVD);
    msgApp( VERBOSE, "DisFracMMVD:%d ",        m_allowDisFracMMVD);
    msgApp( VERBOSE, "FastSearch:%d ",         m_motionEstimationSearchMethod);
    msgApp( VERBOSE, "SbTMVP:%d ",             m_SbTMVP);
    msgApp( VERBOSE, "Geo:%d ",                m_Geo);
    msgApp( VERBOSE, "LFNST:%d ",              m_LFNST);
    msgApp(VERBOSE, "MTS:%d ",                 m_MTS);
    msgApp(VERBOSE, "MTSIntraCand:%d ",        m_MTSIntraMaxCand);
#if 1// ISP_VVC
    msgApp(VERBOSE, "ISP:%d ",                 m_ISP);
#endif
#if 1//TS_VVC
    msgApp(VERBOSE, "TransformSkip:%d ",       m_TS);
    msgApp(VERBOSE, "TransformSkipLog2MaxSize:%d ", m_TSsize);
    msgApp(VERBOSE, "useChromaTS:%d ",         m_useChromaTS);
#endif
  }

  msgApp( VERBOSE, "\nFAST TOOL CFG: " );
  msgApp( VERBOSE, "LCTUFast:%d ",             m_useFastLCTU );
  msgApp( VERBOSE, "FastMrg:%d ",              m_useFastMrg );
  msgApp( VERBOSE, "PBIntraFast:%d ",          m_usePbIntraFast );
  msgApp( VERBOSE, "AMaxBT:%d ",               m_useAMaxBT );
  msgApp( VERBOSE, "FastQtBtEnc:%d ",          m_fastQtBtEnc );
  msgApp( VERBOSE, "ContentBasedFastQtbt:%d ", m_contentBasedFastQtbt );
  if( m_MIP ) msgApp(VERBOSE, "FastMIP:%d ",   m_useFastMIP);
  msgApp( VERBOSE, "FastLocalDualTree:%d ",    m_fastLocalDualTreeMode );
  msgApp( VERBOSE, "FastSubPel:%d ",           m_fastSubPel );
  msgApp( VERBOSE, "QtbttExtraFast:%d ",       m_qtbttSpeedUp );

  msgApp( VERBOSE, "RateControl:%d ",          m_RCRateControlMode );
  if ( m_RCRateControlMode )
  {
    msgApp( VERBOSE, "TargetBitrate:%d ",           m_RCTargetBitrate );
    msgApp( VERBOSE, "KeepHierarchicalBit:%d ",     m_RCKeepHierarchicalBit );
    msgApp( VERBOSE, "RCLCUSeparateModel:%d ",      m_RCUseLCUSeparateModel );
    msgApp( VERBOSE, "InitialQP:%d ",               m_RCInitialQP );
    msgApp( VERBOSE, "RCForceIntraQP:%d ",          m_RCForceIntraQP );
  }

  msgApp( VERBOSE, "\nPARALLEL PROCESSING CFG: " );
  msgApp( VERBOSE, "FPP:%d ",                  m_frameParallel );
  msgApp( VERBOSE, "NumFppThreads:%d ",        m_numFppThreads );
  msgApp( VERBOSE, "FppBitEqual:%d ",          m_ensureFppBitEqual );
  msgApp( VERBOSE, "WPP:%d ",                  m_numWppThreads );
  msgApp( VERBOSE, "WppBitEqual:%d ",          m_ensureWppBitEqual );
  msgApp( VERBOSE, "WF:%d",                    m_entropyCodingSyncEnabled );
  msgApp( VERBOSE, "\n");

  msgApp( NOTICE, "\n");

  fflush( stdout );
}

//! \}


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
/**
  \ingroup VVEncoderApp
  \file    CmdLineParser.h
  \brief   This file contains the interface for class YuvFileReader.
  \author  christian.lehmann@hhi.fraunhofer.de
  \date    08/08/2020
*/

#pragma once

#include <string>
#include <iostream>
#include <stdio.h>
#include <string.h>

#include "vvenc/vvenc.h"

namespace vvcutilities {


class CmdLineParser
{
public:
  /// Constructor
  CmdLineParser(){}

  /// Destructor
  virtual ~CmdLineParser() {}

  static void print_usage( std::string cApp, vvenc::VVEncParameter& rcParams, std::string &rcPreset,
                           std::string &rcProfile, std::string &rcLevel, std::string &rcTier, bool bFullHelp )
  {
      printf( "\n Usage:  %s  [param1] [pararm2] [...] \n", cApp.c_str() );
      std::cout <<
          "\n"
          " File input Options\n"
          "\n"
          "\t [--input,-i <str>        ] : raw yuv input file\n"
          "\t [--size,-s  <intxint>    ] : specify input resolution (width x height) ["<< rcParams.m_iWidth << "x" << rcParams.m_iHeight <<"]\n"
          "\t [--format,-c  <str>      ] : set input format (yuv420, yuv420_10) [yuv420]\n"
          "\t [--framerate,-r  <int>   ] : temporal rate (framerate) e.g. 25,29,30,50,59,60 ["<< rcParams.m_iTemporalRate/rcParams.m_iTemporalScale <<"]\n"
          "\t [--tickspersec   <int>   ] : ticks per second e.g. 90000 for dts generation [1..27000000]\n";
      if ( bFullHelp )
      {
        std::cout <<
          "\t                              29.97 fps = 30000/1001 Hz = 29\n"
          "\t                              59.94 fps = 60000/1001 Hz = 59\n";
      }
      std::cout <<
          "\t [--frames,-f <int>       ] : max. frames to encode (default: -1 all frames)\n"
          "\t [--frameskip <int>       ] : number of frames to encode skip (default: 0 off)\n";
      if ( bFullHelp )
      {
        std::cout <<
          "\t [--segment <str>         ] : when encoding multiple separate segments, specify segment position to enable segment concatenation (first, mid, last) [off]\n"
          "\t              first         : first segment\n"
          "\t              mid           : all segments between first and last segment\n"
          "\t              last          : last segment\n";
      }
      std::cout <<
          "\n"
          " Bitstream output options\n"
          "\n"
          "\t [--output,-o  <str>      ] : bitstream output file (default: not set)\n"
          "\n"
          " Encoder Options\n"
          "\n"
          "\t [--preset      <str>     ] : select preset for specific encoding setting ( faster, fast, medium, slow ) default: [" << rcPreset << "]\n";
      if ( bFullHelp )
      {
        std::cout <<
          "\t              faster        : best speed      : quality=0 :\n"
          "\t                              " << vvenc::VVEnc::getPresetParamsAsStr(0) << "\n\n"
          "\t              fast          : fast mode       : quality=1 :\n"
          "\t                              " << vvenc::VVEnc::getPresetParamsAsStr(1) << "\n\n"
          "\t              medium        : default quality : quality=2 :\n"
          "\t                              " << vvenc::VVEnc::getPresetParamsAsStr(2) << "\n\n"
          "\t              slow          : best quality    : quality=3 :\n"
          "\t                              " << vvenc::VVEnc::getPresetParamsAsStr(3) << "\n\n";
      }
      std::cout <<
          "\t [--bitrate,-b  <int>     ] : bitrate for rate control (0: constant-QP encoding without rate control, otherwise bits/second) default: [" << rcParams.m_iTargetBitRate << "]\n"
          "\t [--qp,-q <int>           ] : quantization parameter, QP (0-51) default: [" << rcParams.m_iQp << "]\n"
          "\t [--qpa   <int>           ] : perceptual QP adaptation to improve visual coding quality (0: off, on for 1: SDR (WPSNR), 2: SDR (XPSNR),\n\t\t\t\t      3: HDR (WPSNR), 4: HDR (XPSNR), 5: HDR (mean luma based)) default: [" << rcParams.m_iPerceptualQPA << "]\n"
          "\t [--threads,-t  <int>     ] : number of threads (1-n) default: [size <= HD: " << rcParams.m_iThreadCount << ", UHD: 6]\n"
          "\n"
          "\t [--gopsize,-g  <int>     ] : GOP size of temporal structure (16) [" <<  rcParams.m_iGopSize << "]\n"
          "\t [--refreshtype,-rt <str> ] : intra refresh type (idr,cra)\n"
          "\t [--refreshsec,-rs <int>  ] : Intra period/refresh in seconds [" <<  rcParams.m_iIDRPeriodSec << "]\n"
          "\t [--intraperiod,-ip <int> ] : Intra period in frames (0: use intra period in seconds (intraperiods), else: n*gopsize [" <<  rcParams.m_iIDRPeriod << "]\n"

          "\n"
          "\t [--profile      <str>    ] : select profile ( main10, main10_stillpic) default: [" << rcProfile << "]\n"
          "\t [--level        <str>    ] : select level ( 1.0, 2.0,2.1, 3.0,3.1, 4.0,4.1, 5.0,5.1,5.2, 6.0,6.1,6.2, 15.5 ) default: [" << rcLevel << "]\n"
          "\t [--tier         <str>    ] : select tier ( main, high ) default: [" << rcTier << "]\n"
          "\n"
          " General Options\n"
          "\n"
          "\t [--verbosity,-v  <int>   ] : verbosity level (0: silent, 1: error, 2: warning, 3: info, 4: notice, 5: verbose, 6: debug) default: [" << (int)rcParams.m_eLogLevel << "]\n"
          "\t [--help,-h               ] : show basic help\n"
          "\t [--fullhelp              ] : show full help\n"
          "\n" ;
      std::cout << std::endl;
  }

  static int parse_command_line( int argc, char* argv[] , vvenc::VVEncParameter& rcParams, std::string& rcInputFile, std::string& rcBitstreamFile, int& riInputBitdepth )
  {
    int iRet = 0;
    /* Check command line parameters */
    int32_t  i_arg = 1;

    /* Check general options firs*/
    while( i_arg < argc )
    {
      if( (!strcmp( (const char*)argv[i_arg], "-v" )) || !strcmp( (const char*)argv[i_arg], "--verbosity" ) )
      {
        i_arg++;
        int iLogLevel = atoi( argv[i_arg++] );
        if( iLogLevel < 0 ) iLogLevel = 0;
        if( iLogLevel > (int)vvenc::LogLevel::LL_DETAILS ) iLogLevel = (int)vvenc::LogLevel::LL_DETAILS ;
        rcParams.m_eLogLevel = (vvenc::LogLevel)iLogLevel;

        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
        {
          std::string cll;
          switch (rcParams.m_eLogLevel)
          {
            case vvenc::LL_SILENT : cll = "SILENT"; break;
            case vvenc::LL_ERROR  : cll = "ERROR"; break;
            case vvenc::LL_WARNING: cll = "WARNING"; break;
            case vvenc::LL_INFO   : cll = "INFO"; break;
            case vvenc::LL_NOTICE : cll = "NOTICE"; break;
            case vvenc::LL_VERBOSE: cll = "VERBOSE"; break;
            case vvenc::LL_DETAILS: cll = "DETAILS"; break;
            case vvenc::LL_DEBUG_PLUS_INTERNAL_LOGS: cll = "DEBUG_PLUS_INTERNAL_LOGS"; break;
            default: cll = "UNKNOWN"; break;
          };
          fprintf( stdout, "[verbosity]            : %d - %s\n", (int)rcParams.m_eLogLevel, cll.c_str() );
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-h" )) || !strcmp( (const char*)argv[i_arg], "--help" ) )
      {
        i_arg++;
        iRet = 2;
        return iRet;
      }
      else if( !strcmp( (const char*)argv[i_arg], "--fullhelp" ) )
      {
        i_arg++;
        iRet = 3;
        return iRet;
      }
      else
      {
        i_arg++;
      }
    }


    i_arg = 1;
    while( i_arg < argc )
    {
      if( (!strcmp( (const char*)argv[i_arg], "-i" )) || !strcmp( (const char*)argv[i_arg], "--input" ) ) /* In: input-file */
      {
        i_arg++;
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[input] input-file     : %s\n", argv[i_arg] );
        rcInputFile = argv[i_arg++];
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-o" )) || !strcmp( (const char*)argv[i_arg], "--output" ) ) /* Out: bitstream-file */
      {
        i_arg++;
        if( i_arg < argc && strlen( argv[i_arg] ) > 0 )
        {
          if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
            fprintf( stdout, "[output] bitstream-file: %s\n", argv[i_arg] );
          rcBitstreamFile = argv[i_arg++];
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-s" )) || !strcmp( (const char*)argv[i_arg], "--size" ) )
      {
        i_arg++;
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[Size ]                : %s\n", argv[i_arg] );
        std::string cSize = argv[i_arg++];
        std::transform( cSize.begin(), cSize.end(), cSize.begin(), ::tolower );
        std::regex pattern("([0-9]+)x([0-9]+)");

        std::smatch match;
        if (std::regex_search(cSize, match, pattern))
        {
          std::string cW = match[1].str();
          std::string cH = match[2].str();

          rcParams.m_iWidth  = std::stoi( cW ) ;
          rcParams.m_iHeight = std::stoi( cH ) ;
        }
        else
        {
          std::cerr << "wrong resolution syntax. usage: intxint e.g. 3840x2160" << std::endl;
          return -1;
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-r" )) || !strcmp( (const char*)argv[i_arg], "--framerate" ) )
      {
        i_arg++;
        int iFramerate = atoi( argv[i_arg++] );
        rcParams.m_iTemporalRate   = iFramerate;
        rcParams.m_iTemporalScale  = 1;

        switch( iFramerate )
        {
        case 23: rcParams.m_iTemporalRate = 24000; rcParams.m_iTemporalScale = 1001; break;
        case 29: rcParams.m_iTemporalRate = 30000; rcParams.m_iTemporalScale = 1001; break;
        case 59: rcParams.m_iTemporalRate = 60000; rcParams.m_iTemporalScale = 1001; break;
        default: break;
        }

        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[Temp.Rate/Scale]      : %d/%d\n", rcParams.m_iTemporalRate, rcParams.m_iTemporalScale );
      }
      else if( !strcmp( (const char*)argv[i_arg], "--tickspersec" ) )
      {
        i_arg++;
        rcParams.m_iTicksPerSecond = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[tickspersec]          : %d\n", rcParams.m_iTicksPerSecond );
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-f" )) || !strcmp( (const char*)argv[i_arg], "--frames" ) )
      {
        i_arg++;
        rcParams.m_iMaxFrames = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[frames]               : %d\n", rcParams.m_iMaxFrames );
      }
      else if( !strcmp( (const char*)argv[i_arg], "--frameskip" ) )
      {
        i_arg++;
        rcParams.m_iFrameSkip= atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[frameskip]           : %d\n", rcParams.m_iFrameSkip );
      }
      else if( !strcmp( (const char*)argv[i_arg], "--segment" ) )
      {
        i_arg++;
        if( i_arg < argc && strlen( argv[i_arg] ) > 0 )
        {
          if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
            fprintf( stdout, "[segment]            : %s\n", argv[i_arg] );
          std::string cSegMode = argv[i_arg++];
          std::transform( cSegMode.begin(), cSegMode.end(), cSegMode.begin(), ::tolower );

          if( "off" == cSegMode)
          {
            rcParams.m_eSegMode = vvenc::VVC_SEG_OFF;
          }
          else if( "first" == cSegMode )
          {
            rcParams.m_eSegMode = vvenc::VVC_SEG_FIRST;
          }
          else if( "mid" == cSegMode )
          {
            rcParams.m_eSegMode = vvenc::VVC_SEG_MID;
          }
          else if( "last" == cSegMode )
          {
            rcParams.m_eSegMode = vvenc::VVC_SEG_LAST;
          }
          else
          {
            std::cerr << "wrong segment mode!   use: --segment off, first, mid, last" << std::endl;
            return -1;
          }
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-c" )) || !strcmp( (const char*)argv[i_arg], "--format" ) )
      {
        i_arg++;
        if( i_arg < argc && strlen( argv[i_arg] ) > 0 )
        {
          if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
            fprintf( stdout, "[format] input format  : %s\n", argv[i_arg] );
          std::string cColorSpace = argv[i_arg++];
          std::transform( cColorSpace.begin(), cColorSpace.end(), cColorSpace.begin(), ::tolower );

          if( "yuv420" == cColorSpace || "yuv420p" == cColorSpace )
          {
            riInputBitdepth = 8;
          }
          else if( "yuv420_10" == cColorSpace )
          {
            riInputBitdepth = 10;
          }
          else
          {
            std::cerr << "wrong inputformat!   use: -c yuv420, yuv420_10" << std::endl;
            return -1;
          }
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-t" )) || !strcmp( (const char*)argv[i_arg], "--threads" ) )
      {
        i_arg++;
        int iThreads = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[threads]              : %d\n", iThreads );
        rcParams.m_iThreadCount = iThreads;
      }
      else if( !strcmp( (const char*)argv[i_arg], "--preset") )
      {
        i_arg++;
        if( i_arg < argc && strlen( argv[i_arg] ) > 0 )
        {
          if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
            fprintf( stdout, "[preset]               : %s\n", argv[i_arg] );
          std::string cPreset = argv[i_arg++];
          std::transform( cPreset.begin(), cPreset.end(), cPreset.begin(), ::tolower );

          if( "tooltest" == cPreset)
          {
            rcParams.m_iQuality = 255;
          }
          else if( "slow" == cPreset)
          {
            rcParams.m_iQuality = 3;
          }
          else if( "medium" == cPreset )
          {
            rcParams.m_iQuality = 2;
          }
          else if( "fast" == cPreset )
          {
            rcParams.m_iQuality = 1;
          }
          else if( "faster" == cPreset )
          {
            rcParams.m_iQuality = 0;
          }
          else
          {
            std::cerr << "wrong preset!   use: --preset faster, fast, medium, slow" << std::endl;
            return -1;
          }
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-b" )) || !strcmp( (const char*)argv[i_arg], "--bitrate" ) )
      {
        i_arg++;
        rcParams.m_iTargetBitRate = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[bitrate]              : %d bits per second\n", rcParams.m_iTargetBitRate );
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-q" )) || !strcmp( (const char*)argv[i_arg], "--qp" ) )
      {
        i_arg++;
        rcParams.m_iQp = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[qp]                   : %d\n", rcParams.m_iQp );
      }
      else if( !strcmp( (const char*)argv[i_arg], "--qpa" ) )
      {
        i_arg++;
        rcParams.m_iPerceptualQPA = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[qpa]                  : %d\n", rcParams.m_iPerceptualQPA );
      }
      else if( (!strcmp( (const char*)argv[i_arg], "--gopsize" )) || !strcmp( (const char*)argv[i_arg], "-g" ) )
      {
        i_arg++;
        rcParams.m_iGopSize = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[gopsize]              : %d\n", rcParams.m_iGopSize );

        if( rcParams.m_iGopSize != 16 )
        {
          std::cerr << "unsupported gopsize " <<  rcParams.m_iGopSize << ". use: --gopsize 16" << std::endl;
          return -1;
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-rs" )) || !strcmp( (const char*)argv[i_arg], "--refreshsec" ) )
      {
        i_arg++;
        rcParams.m_iIDRPeriodSec = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[refreshsec]          : %d\n", rcParams.m_iIDRPeriodSec );
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-ip" )) || !strcmp( (const char*)argv[i_arg], "--intraperiod" ) )
      {
        i_arg++;
        rcParams.m_iIDRPeriod = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[intraperiod]          : %d\n", rcParams.m_iIDRPeriod );
      }
      else if( (!strcmp( (const char*)argv[i_arg], "--refreshtype") ) || (!strcmp( (const char*)argv[i_arg], "-rt" )) )
      {
        i_arg++;
        if( i_arg < argc && strlen( argv[i_arg] ) > 0 )
        {
          if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
            fprintf( stdout, "[decrefreshtype]       : %s\n", argv[i_arg] );
          std::string cDecRefreshType = argv[i_arg++];
          std::transform( cDecRefreshType.begin(), cDecRefreshType.end(), cDecRefreshType.begin(), ::tolower );

          if( "idr" == cDecRefreshType)
          {
            rcParams.m_eDecodingRefreshType = vvenc::VvcDecodingRefreshType::VVC_DRT_IDR;
          }
          else if( "cra" == cDecRefreshType )
          {
            rcParams.m_eDecodingRefreshType = vvenc::VvcDecodingRefreshType::VVC_DRT_CRA;
          }
          else
          {
            std::cerr << "wrong preset!   use: --preset fast, superfast" << std::endl;
            return -1;
          }
        }
      }
      else if( !strcmp( (const char*)argv[i_arg], "--profile") )
      {
        i_arg++;
        if( i_arg < argc && strlen( argv[i_arg] ) > 0 )
        {
          if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
            fprintf( stdout, "[profile]              : %s\n", argv[i_arg] );
          std::string cProfile = argv[i_arg++];
          std::transform( cProfile.begin(), cProfile.end(), cProfile.begin(), ::tolower );

          if( "main10" == cProfile)
          {
            rcParams.m_eProfile = vvenc::VVC_PROFILE_MAIN_10;
          }
          else if( "main10_stillpic" == cProfile )
          {
            rcParams.m_eProfile = vvenc::VVC_PROFILE_MAIN_10_STILL_PICTURE;
          }
          else
          {
            std::cerr << "wrong profile!   use: --profile main10, main10_stillpic" << std::endl;
            return -1;
          }
        }
      }
      else if( !strcmp( (const char*)argv[i_arg], "--level") )
      {
        i_arg++;
        if( i_arg < argc && strlen( argv[i_arg] ) > 0 )
        {
          if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
            fprintf( stdout, "[level]                : %s\n", argv[i_arg] );
          std::string cLevel = argv[i_arg++];
          std::transform( cLevel.begin(), cLevel.end(), cLevel.begin(), ::tolower );

          if( "1.0" == cLevel)
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_1;
          }
          else if( "2.0" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_2;
          }
          else if( "2.1" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_2_1;
          }
          else if( "3.0" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_3;
          }
          else if( "3.1" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_3_1;
          }
          else if( "4.0" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_4;
          }
          else if( "4.1" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_4_1;
          }
          else if( "5.0" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_5;
          }
          else if( "5.1" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_5_1;
          }
          else if( "5.2" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_5_2;
          }
          else if( "6.0" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_6;
          }
          else if( "6.1" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_6_1;
          }
          else if( "6.2" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_6_2;
          }
          else if( "15.5" == cLevel )
          {
            rcParams.m_eLevel = vvenc::VVC_LEVEL_15_5;
          }
          else
          {
            std::cerr << "wrong profile!   use: --level 1.0, 2.0,2.1, 3.0,3.1, 4.0,4.1, 5.0,5.1,5.2, 6.0,6.1,6.2, 15.5" << std::endl;
            return -1;
          }
        }
      }
      else if( !strcmp( (const char*)argv[i_arg], "--tier") )
      {
        i_arg++;
        if( i_arg < argc && strlen( argv[i_arg] ) > 0 )
        {
          if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
            fprintf( stdout, "[tier]                 : %s\n", argv[i_arg] );
          std::string cTier = argv[i_arg++];
          std::transform( cTier.begin(), cTier.end(), cTier.begin(), ::tolower );

          if( "main" == cTier)
          {
            rcParams.m_eTier = vvenc::VVC_TIER_MAIN;
          }
          else if( "high" == cTier )
          {
            rcParams.m_eTier = vvenc::VVC_TIER_HIGH;
          }
          else
          {
            std::cerr << "wrong tier!   use: --tier main,high" << std::endl;
            return -1;
          }
        }
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-v" )) || !strcmp( (const char*)argv[i_arg], "--verbosity" ) )
      {
        // already processed
        i_arg++;
        i_arg++;
      }
      else if( (!strcmp( (const char*)argv[i_arg], "-h" )) || !strcmp( (const char*)argv[i_arg], "--help" ) )
      {
        // already processed
        i_arg++;
      }
      else if( !strcmp( (const char*)argv[i_arg], "--fullhelp" ) )
      {
      // already processed
        i_arg++;
      }
      else
      {
        fprintf( stderr, " - IGNORED: %s \n", argv[i_arg++] );
      }
    }

    return iRet;
  }

private:
  std::ofstream m_cOS;
};



} // namespace


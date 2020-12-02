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
          "\t [--passes,-p   <int>     ] : number of rate control passes (1 or 2) default: [" << rcParams.m_iNumPasses << "]\n"
          "\t [--qp,-q <int>           ] : quantization parameter, QP (0-51) default: [" << rcParams.m_iQp << "]\n"
          "\t [--qpa   <int>           ] : perceptual QP adaptation to improve visual coding quality (0: off, on for 1: SDR (WPSNR), 2: SDR (XPSNR),\n\t\t\t\t      3: HDR (WPSNR), 4: HDR (XPSNR), 5: HDR (mean luma based)) default: [" << rcParams.m_iPerceptualQPA << "]\n"
          "\t [--threads,-t  <int>     ] : number of threads (1-n) default: [size <= HD: " << rcParams.m_iThreadCount << ", UHD: 6]\n"
          "\n"
          "\t [--gopsize,-g  <int>     ] : GOP size of temporal structure (16) [" <<  rcParams.m_iGopSize << "]\n"
          "\t [--refreshtype,-rt <str> ] : intra refresh type (idr,cra)\n"
          "\t [--refreshsec,-rs <int>  ] : Intra period/refresh in seconds [" <<  rcParams.m_iIDRPeriodSec << "]\n"
          "\t [--intraperiod,-ip <int> ] : Intra period in frames (0: use intra period in seconds (intraperiods), else: n*gopsize [" <<  rcParams.m_iIDRPeriod << "]\n";
      if ( bFullHelp )
      {
        std::cout <<
          "\t [--internal-bitdepth <int> ] : internal bitdepth [" <<  rcParams.m_iInternalBitDepth << "]\n";
      }

      std::cout <<
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

  static int parse_command_line( int argc, char* argv[] , vvenc::VVEncParameter& rcParams, std::string& rcInputFile, std::string& rcBitstreamFile )
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
        if( iLogLevel > (int)vvenc::LL_DETAILS ) iLogLevel = (int)vvenc::LL_DETAILS;
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
            rcParams.m_iInputBitDepth = 8;
          }
          else if( "yuv420_10" == cColorSpace )
          {
            rcParams.m_iInputBitDepth = 10;
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
          else if( "slower" == cPreset)
          {
            rcParams.m_iQuality = 4;
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
      else if( (!strcmp( (const char*)argv[i_arg], "-p" )) || !strcmp( (const char*)argv[i_arg], "--passes" ) )
      {
        i_arg++;
        rcParams.m_iNumPasses = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[passes]               : %d\n", rcParams.m_iNumPasses );
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
      else if( !strcmp( (const char*)argv[i_arg], "--internal-bitdepth" ) )
      {
        i_arg++;
        rcParams.m_iInternalBitDepth = atoi( argv[i_arg++] );
        if( rcParams.m_eLogLevel > vvenc::LL_VERBOSE )
          fprintf( stdout, "[internal-bitdepth]      : %d\n", rcParams.m_iInternalBitDepth );
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


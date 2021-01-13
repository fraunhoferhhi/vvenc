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
  \file    VVEncoderApp.cpp
  \brief   This VVEncoderApp.cpp file contains the main entry point of the application.
  \author  lehmann
  \date    2019-10-10
*/

#include <iostream>

#include <stdio.h>
#include <string>
#include <fstream>
#include <cstring>
#include <ctime>
#include <chrono>

#include "vvenc/version.h"
#include "vvenc/vvenc.h"

#include "CmdLineParser.h"
#include "YuvFileReader.h"

#include "apputils/EncAppCfg.h"
#include "apputils/ParseArg.h"

#define USE_FFPARAMS 0


int g_verbosity = vvenc::VERBOSE;

void msgFnc( int level, const char* fmt, va_list args )
{
  if ( g_verbosity >= level )
  {
    vfprintf( level == 1 ? stderr : stdout, fmt, args );
  }
}

void msgApp( int level, const char* fmt, ... )
{
    va_list args;
    va_start( args, fmt );
    msgFnc( level, fmt, args );
    va_end( args );
}

void printVVEncErrorMsg( const std::string cAppname, const std::string cMessage, int code, const std::string cErr )
{
  std::cout << cAppname  << " [error]: " << cMessage << ", ";
  switch( code )
  {
    case vvenc::VVENC_ERR_CPU :           std::cout << "SSE 4.1 cpu support required."; break;
    case vvenc::VVENC_ERR_PARAMETER :     std::cout << "invalid parameter."; break;
    case vvenc::VVENC_ERR_NOT_SUPPORTED : std::cout << "unsupported request."; break;
    default :                             std::cout << "error " << code; break;
  };
  if( !cErr.empty() )
  {
    std::cout << " - " << cErr;
  }
  std::cout << std::endl;
}



#if USE_FFPARAMS
bool parseCfg( int argc, char* argv[], EncAppCfg& rcEncAppCfg )
{
  try
  {
    if( ! rcEncAppCfg.parseCfg( argc, argv ) )
    {
      return false;
    }
  }
  catch( VVCEncoderFFApp::df::program_options_lite::ParseFailure &e )
  {
    msgApp( ERROR, "Error parsing option \"%s\" with argument \"%s\".\n", e.arg.c_str(), e.val.c_str() );
    return false;
  }

  if( ! rcEncAppCfg.m_decode )
  {
    rcEncAppCfg.printAppCfgOnly();
  }

  return true;
}

#endif

int main( int argc, char* argv[] )
{
  std::string cAppname = argv[0];
  std::size_t iPos = (int)cAppname.find_last_of("/");
  if( std::string::npos != iPos )
  {
    cAppname = cAppname.substr(iPos+1 );
  }

  int iRet = 0;

  vvenc::VVEnc::registerMsgCbf( msgFnc );

  std::string cInputFile;
  std::string cOutputfile = "";

  vvenc::VVEncParameter cVVEncParameter;

#if USE_FFPARAMS
  std::string simdOpt;
  VVCEncoderFFApp::df::program_options_lite::Options opts;
  opts.addOptions()
    ( "c",           VVCEncoderFFApp::df::program_options_lite::parseConfigFile, "" )
    ( "Verbosity,v", g_verbosity,                               "" )
    ( "SIMD",        simdOpt,                                   "" );
  VVCEncoderFFApp::df::program_options_lite::SilentReporter err;
  VVCEncoderFFApp::df::program_options_lite::scanArgv( opts, argc, ( const char** ) argv, err );

  simdOpt = vvenc::VVEnc::setSIMDExtension( simdOpt );

  EncAppCfg    cEncAppCfg;                      ///< encoder configuration

  cEncAppCfg.m_QP                  = 32;                       // quantization parameter 0-51
  cEncAppCfg.m_SourceWidth         = 1920;                     // luminance width of input picture
  cEncAppCfg.m_SourceHeight        = 1080;                     // luminance height of input picture
  cEncAppCfg.m_GOPSize             = 32;                       //  gop size (1: intra only, 16, 32: hierarchical b frames)
  cEncAppCfg.m_DecodingRefreshType = vvenc::DRT_CRA;           // intra period refresh type
  cEncAppCfg.m_IntraPeriod         = 1;                        // intra period in seconds for IDR/CDR intra refresh/RAP flag (should be > 0)
  cEncAppCfg.m_IntraPeriod         = 0;                        // intra period in frames for IDR/CDR intra refresh/RAP flag (should be a factor of GopSize)
  cEncAppCfg.m_verbosity           = (int)vvenc::VERBOSE;      // log level > 4 (VERBOSE) enables psnr/rate output
  cEncAppCfg.m_FrameRate           = 60;                       // temporal rate (fps)
  cEncAppCfg.m_TicksPerSecond      = 90000;                    // ticks per second e.g. 90000 for dts generation
  cEncAppCfg.m_framesToBeEncoded   = 0;                        // max number of frames to be encoded
  cEncAppCfg.m_FrameSkip           = 0;                        // number of frames to skip before start encoding
  cEncAppCfg.m_numWppThreads       = -1;                       // number of worker threads (should not exceed the number of physical cpu's)
  cEncAppCfg.m_usePerceptQPA       = 2;                        // percepual qpa adaptation, 0 off, 1 on for sdr(wpsnr), 2 on for sdr(xpsnr), 3 on for hdr(wpsrn), 4 on for hdr(xpsnr), on for hdr(MeanLuma)
  cEncAppCfg.m_inputBitDepth[0]    = 8;                        // input bitdepth
  cEncAppCfg.m_internalBitDepth[0] = 10;                       // internal bitdepth
  cEncAppCfg.m_profile             = vvenc::Profile::MAIN_10;  // profile: use main_10 or main_10_still_picture
  cEncAppCfg.m_level               = vvenc::Level::LEVEL4_1;   // level
  cEncAppCfg.m_levelTier           = vvenc::Tier::TIER_MAIN;   // tier
  cEncAppCfg.m_SegmentMode         = vvenc::SEG_OFF;           // segment mode

  if( 0 != cEncAppCfg.initPreset( vvenc::PresetMode::MEDIUM ))
  {
    std::cerr << cAppname  << " [error]: undefined preset " << std::endl;
    return -1;
  }

  // parse configuration
  if ( ! parseCfg( argc, argv, cEncAppCfg ) )
  {
    return 1;
  }

  if( cEncAppCfg.m_inputFileName.empty() )
  {
    std::cerr << cAppname  << " [error]: no input file given. run VVEncoderApp --help to see available options" << std::endl;
    return -1;
  }

  if( cEncAppCfg.m_bitstreamFileName.empty() )
  {
    std::cout << cAppname  << " [error]: no output bitstream file given." << std::endl;
    return -1;
  }

  cInputFile  = cEncAppCfg.m_inputFileName;
  cOutputfile = cEncAppCfg.m_bitstreamFileName;

  if( cEncAppCfg.m_verbosity > vvenc::SILENT && cEncAppCfg.m_verbosity < vvenc::NOTICE )
  {
    std::cout << "-------------------" << std::endl;
    std::cout << cAppname  << " version " << vvenc::VVEnc::getVersionNumber() << std::endl;
  }

  if( cEncAppCfg.m_numWppThreads < 0 )
  {
    if( cEncAppCfg.m_SourceWidth > 1920 || cEncAppCfg.m_SourceHeight > 1080)
    {
      cEncAppCfg.m_numWppThreads = 6;
    }
    else
    {
      cEncAppCfg.m_numWppThreads = 4;
    }
    cEncAppCfg.m_ensureWppBitEqual = 1;
  }

  cVVEncParameter.msgLevel = (vvenc::MsgLevel)cEncAppCfg.m_verbosity;
  cVVEncParameter.width    = cEncAppCfg.m_SourceWidth;
  cVVEncParameter.height   = cEncAppCfg.m_SourceHeight;

  cVVEncParameter.inputBitDepth    = cEncAppCfg.m_inputBitDepth[0];
  cVVEncParameter.internalBitDepth = cEncAppCfg.m_internalBitDepth[0];

  cVVEncParameter.maxFrames = cEncAppCfg.m_framesToBeEncoded;
  cVVEncParameter.frameSkip = cEncAppCfg.m_FrameSkip;

  cVVEncParameter.ticksPerSecond = cEncAppCfg.m_TicksPerSecond;
  cVVEncParameter.temporalRate   = cEncAppCfg.m_FrameRate;
  cVVEncParameter.temporalScale  = 1;

  switch( cEncAppCfg.m_FrameRate )
  {
  case 23: cVVEncParameter.temporalRate = 24000; cVVEncParameter.temporalScale = 1001; break;
  case 29: cVVEncParameter.temporalRate = 30000; cVVEncParameter.temporalScale = 1001; break;
  case 59: cVVEncParameter.temporalRate = 60000; cVVEncParameter.temporalScale = 1001; break;
  default: break;
  }

  cVVEncParameter.numPasses = cEncAppCfg.m_RCNumPasses;

#else
  // set desired encoding options
  cVVEncParameter.qp               = 32;                         // quantization parameter 0-51
  cVVEncParameter.width            = 1920;                       // luminance width of input picture
  cVVEncParameter.height           = 1080;                       // luminance height of input picture
  cVVEncParameter.gopSize          = 32;                         //  gop size (1: intra only, 16, 32: hierarchical b frames)
  cVVEncParameter.decodingRefreshType = vvenc::DRT_CRA;          // intra period refresh type
  cVVEncParameter.idrPeriodSec     = 1;                          // intra period in seconds for IDR/CDR intra refresh/RAP flag (should be > 0)
  cVVEncParameter.idrPeriod        = 0;                          // intra period in frames for IDR/CDR intra refresh/RAP flag (should be a factor of GopSize)
  cVVEncParameter.msgLevel         = vvenc::VERBOSE;             // log level > 4 (VERBOSE) enables psnr/rate output
  cVVEncParameter.temporalRate     = 60;                         // temporal rate (fps)
  cVVEncParameter.temporalScale    = 1;                          // temporal scale (fps)
  cVVEncParameter.ticksPerSecond   = 90000;                      // ticks per second e.g. 90000 for dts generation
  cVVEncParameter.maxFrames        = 0;                          // max number of frames to be encoded
  cVVEncParameter.frameSkip        = 0;                          // number of frames to skip before start encoding
  cVVEncParameter.threadCount      = -1;                         // number of worker threads (should not exceed the number of physical cpu's)
  cVVEncParameter.quality          = 2;                          // encoding quality (vs speed) 0: faster, 1: fast, 2: medium, 3: slow, 4: slower
  cVVEncParameter.perceptualQPA    = 2;                          // percepual qpa adaptation, 0 off, 1 on for sdr(wpsnr), 2 on for sdr(xpsnr), 3 on for hdr(wpsrn), 4 on for hdr(xpsnr), on for hdr(MeanLuma)
  cVVEncParameter.inputBitDepth    = 8;                          // input bitdepth
  cVVEncParameter.internalBitDepth = 10;                         // internal bitdepth
  cVVEncParameter.profile          = vvenc::Profile::MAIN_10;    // profile: use main_10 or main_10_still_picture
  cVVEncParameter.level            = vvenc::Level::LEVEL4_1;     // level
  cVVEncParameter.tier             = vvenc::Tier::TIER_MAIN;     // tier
  cVVEncParameter.segmentMode      = vvenc::SEG_OFF;             // segment mode

  std::string cPreset  = "medium";
  std::string cProfile = "main10";
  std::string cLevel   = "4.1";
  std::string cTier    = "main";

  if(  argc > 1 && (!strcmp( (const char*) argv[1], "--help" ) || !strcmp( (const char*) argv[1], "-h" )) )
  {
    std::cout << cAppname  << " version: " << VVENC_VERSION << std::endl;
    vvcutilities::CmdLineParser::print_usage( cAppname, cVVEncParameter, cPreset, cProfile, cLevel, cTier, false );
    return 0;
  }
  else if(  argc > 1 && (!strcmp( (const char*) argv[1], "--fullhelp" ) ) )
  {
    std::cout << cAppname  << " version: " << VVENC_VERSION << std::endl;
    vvcutilities::CmdLineParser::print_usage( cAppname, cVVEncParameter, cPreset, cProfile, cLevel, cTier, true );
    return 0;
  }

  iRet = vvcutilities::CmdLineParser::parse_command_line(  argc, argv, cVVEncParameter, cInputFile, cOutputfile );
  if( iRet != 0 )
  {
    if( iRet == 2 || iRet == 3 )
    {
      bool bFullHelp = ( iRet == 3) ? true : false;
      std::cout << cAppname  << " version: " << VVENC_VERSION << std::endl;
      vvcutilities::CmdLineParser::print_usage( cAppname, cVVEncParameter, cPreset, cProfile, cLevel, cTier, bFullHelp);
      return 0;
    }

    std::cerr << cAppname  <<  " [error]: cannot parse command line. run VVEncoderApp --help to see available options" << std::endl;
    return -1;
  }

  g_verbosity = cVVEncParameter.msgLevel;

  if( cInputFile.empty() )
  {
    std::cerr << cAppname  << " [error]: no input file given. run VVEncoderApp --help to see available options" << std::endl;
    return -1;
  }

  if( cOutputfile.empty() )
  {
    std::cout << cAppname  << " [error]: no output bitstream file given." << std::endl;
    return -1;
  }

  if( cVVEncParameter.msgLevel > vvenc::SILENT && cVVEncParameter.msgLevel < vvenc::NOTICE )
  {
    std::cout << "-------------------" << std::endl;
    std::cout << cAppname  << " version " << vvenc::VVEnc::getVersionNumber() << std::endl;
  }

  if( cVVEncParameter.threadCount < 0 )
  {
    if( cVVEncParameter.width > 1920 || cVVEncParameter.height > 1080)
    {
      cVVEncParameter.threadCount = 6;
    }
    else
    {
      cVVEncParameter.threadCount = 4;
    }
  }
#endif

  vvenc::VVEnc cVVEnc;

  // initialize the encoder
#if USE_FFPARAMS
  iRet = cVVEnc.init( cEncAppCfg );
#else
  iRet = cVVEnc.init( cVVEncParameter );
#endif
  if( 0 != iRet )
  {
    printVVEncErrorMsg( cAppname, "cannot create encoder", iRet, cVVEnc.getLastError() );
    return iRet;
  }

#if USE_FFPARAMS
  cVVEnc.printConfig();
#endif

  if( cVVEncParameter.msgLevel > vvenc::WARNING )
  {
    std::cout << "VVEnc info: " << cVVEnc.getEncoderInfo() << std::endl;
  }

  // open output file
  std::ofstream cOutBitstream;
  if( !cOutputfile.empty() )
  {
    cOutBitstream.open( cOutputfile.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
    if( !cOutBitstream.is_open() )
    {
      std::cout << cAppname  << " [error]: failed to open output file " << cOutputfile << std::endl;
      return -1;
    }
  }

  // --- allocate memory for output packets
  vvenc::AccessUnit cAccessUnit;

  // --- start timer
  std::chrono::steady_clock::time_point cTPStartRun;
  std::chrono::steady_clock::time_point cTPEndRun;

  std::time_t startTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  if( cVVEncParameter.msgLevel > vvenc::WARNING )
  {
    std::cout  << "started @ " << std::ctime(&startTime2)  << std::endl;
  }

  vvenc::YuvPicture cYuvPicture;
  unsigned char* pucDeletePicBuffer = nullptr;

  unsigned int uiFrames = 0;
  for( int pass = 0; pass < cVVEncParameter.numPasses; pass++ )
  {
    // initialize the encoder pass
    iRet = cVVEnc.initPass( pass );
    if( 0 != iRet )
    {
      printVVEncErrorMsg( cAppname, "cannot init encoder", iRet, cVVEnc.getLastError() );
      return iRet;
    }

    // open the input file
    vvcutilities::YuvFileReader cYuvFileReader;
    if( 0 != cYuvFileReader.open( cInputFile.c_str(), cVVEncParameter.inputBitDepth, cVVEncParameter.internalBitDepth, cVVEncParameter.width, cVVEncParameter.height ) )
    {
      std::cout << cAppname  << " [error]: failed to open input file " << cInputFile << std::endl;
      return -1;
    }

    // allocate input picture buffer
    if( cYuvPicture.width == 0 )
    {
      iRet = cYuvFileReader.allocBuffer( cYuvPicture );
      if( 0 != iRet )
      {
        std::cout << cAppname  << " [error]: failed to allocate picture buffer " << std::endl;
        return iRet;
      }
      pucDeletePicBuffer = cYuvPicture.deletePicBuffer;
      cYuvPicture.deletePicBuffer = NULL;
    }

    const int64_t iFrameSkip  = std::max<int64_t>( cVVEncParameter.frameSkip - cVVEnc.getNumLeadFrames(), 0 );
    const int64_t iMaxFrames  = cVVEncParameter.maxFrames + cVVEnc.getNumLeadFrames() + cVVEnc.getNumTrailFrames();
    int64_t       iSeqNumber  = 0;
    bool          bEof        = false;
    bool          bEncodeDone = false;
    vvenc::YuvPicture* pcInputPicture = &cYuvPicture;

    uiFrames    = 0;

    if( iFrameSkip )
    {
      cYuvFileReader.skipFrames(iFrameSkip);
      iSeqNumber=iFrameSkip;
    }

    while( !bEof || !bEncodeDone )
    {
      if( !bEof )
      {
        iRet = cYuvFileReader.readPicture( cYuvPicture );
        if( iRet )
        {
          if( cVVEncParameter.msgLevel > vvenc::ERROR && cVVEncParameter.msgLevel < vvenc::NOTICE )
          {
            std::cout << "EOF reached" << std::endl;
          }
          bEof = true;
          pcInputPicture = nullptr;
        }
        else
        {
          // set sequence number and cts
          cYuvPicture.sequenceNumber = iSeqNumber;
          cYuvPicture.cts            = iSeqNumber * cVVEncParameter.ticksPerSecond * cVVEncParameter.temporalScale / cVVEncParameter.temporalRate;
          cYuvPicture.ctsValid        = true;
          iSeqNumber++;
          //std::cout << "process picture " << cYuvPicture.m_uiSequenceNumber << " cts " << cYuvPicture.m_uiCts << std::endl;
        }
      }

      // call encode
      iRet = cVVEnc.encode( pcInputPicture, cAccessUnit, bEncodeDone );
      if( 0 != iRet )
      {
        printVVEncErrorMsg( cAppname, "encoding failed", iRet, cVVEnc.getLastError() );
        return iRet;
      }

      if( ! cAccessUnit.payload.empty()  )
      {
        if( cOutBitstream.is_open() )
        {
          // write output
          cOutBitstream.write( (const char*)cAccessUnit.payload.data(), cAccessUnit.payload.size() );
        }
        uiFrames++;
      }

      if( iMaxFrames > 0 && iSeqNumber >= ( iFrameSkip + iMaxFrames ) )
      {
        bEof = true;
        pcInputPicture = nullptr;
      }
    }

    cYuvFileReader.close();
  }

  cTPEndRun = std::chrono::steady_clock::now();
  double dTimeSec = (double)std::chrono::duration_cast<std::chrono::milliseconds>((cTPEndRun)-(cTPStartRun)).count() / 1000;

  delete[] pucDeletePicBuffer;

  if( cOutBitstream.is_open() )
  {
    cOutBitstream.close();
  }

  cVVEnc.printSummary();

  // un-initialize the encoder
  iRet = cVVEnc.uninit();
  if( 0 != iRet )
  {
    printVVEncErrorMsg( cAppname, "destroyencoder failed", iRet, cVVEnc.getLastError() );
    return iRet;
  }

  if( 0 == uiFrames )
  {
    std::cout << "no frames encoded" << std::endl;
  }

  if( uiFrames && cVVEncParameter.msgLevel > vvenc::SILENT )
  {
    std::time_t endTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    double dFps = (double)uiFrames / dTimeSec;

    if( cVVEncParameter.msgLevel > vvenc::WARNING )
    {
      std::cout  << "finished @ " << std::ctime(&endTime2)  << std::endl;
    }
    std::cout << "Total Time: " << dTimeSec << " sec. Fps(avg): " << dFps << " encoded Frames " << uiFrames << std::endl;
  }

  return 0;
}


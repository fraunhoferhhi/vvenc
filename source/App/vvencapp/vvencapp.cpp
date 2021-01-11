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
#include <regex>

#include "vvenc/version.h"
#include "vvenc/vvenc.h"

#include "BinFileWriter.h"
#include "CmdLineParser.h"
#include "YuvFileReader.h"

#include "apputils/EncAppCfg.h"
#include "apputils/ParseArg.h"
#include "vvenc/EncoderIf.h"


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


#define USE_FFPARAMS 1

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

  simdOpt = vvenc::setSIMDExtension( simdOpt );

  EncAppCfg    cEncAppCfg;                      ///< encoder configuration

  cEncAppCfg.m_QP               = 32;                             // quantization parameter 0-51
  cEncAppCfg.m_SourceWidth            = 1920;                    // luminance width of input picture
  cEncAppCfg.m_SourceHeight           = 1080;                    // luminance height of input picture
  cEncAppCfg.m_GOPSize           = 32;                           //  gop size (1: intra only, 16, 32: hierarchical b frames)
  cEncAppCfg.m_DecodingRefreshType = vvenc::DRT_CRA;             // intra period refresh type
  cEncAppCfg.m_IntraPeriodSec     = 1;                           // intra period in seconds for IDR/CDR intra refresh/RAP flag (should be > 0)
  cEncAppCfg.m_IntraPeriod        = 0;                           // intra period in frames for IDR/CDR intra refresh/RAP flag (should be a factor of m_iGopSize)
  cEncAppCfg.m_verbosity         = (int)vvenc::VERBOSE;          // log level > 4 (VERBOSE) enables psnr/rate output
  cEncAppCfg.m_FrameRate         = 60;                           // temporal rate (fps)
  cEncAppCfg.m_TicksPerSecond   = 90000;                         // ticks per second e.g. 90000 for dts generation
  cEncAppCfg.m_framesToBeEncoded = 0;                            // max number of frames to be encoded
  cEncAppCfg.m_FrameSkip         = 0;                            // number of frames to skip before start encoding
  cEncAppCfg.m_numWppThreads     = 0;                            // number of worker threads (should not exceed the number of physical cpu's)
  //cEncAppCfg.m_iQuality          = 2;                          // encoding quality (vs speed) 0: faster, 1: fast, 2: medium, 3: slow, 4: slower
  cEncAppCfg.m_usePerceptQPA     = 2;                            // percepual qpa adaptation, 0 off, 1 on for sdr(wpsnr), 2 on for sdr(xpsnr), 3 on for hdr(wpsrn), 4 on for hdr(xpsnr), on for hdr(MeanLuma)
  cEncAppCfg.m_inputBitDepth[0]    = 8;                          // input bitdepth
  cEncAppCfg.m_internalBitDepth[0] = 10;                         // internal bitdepth
  cEncAppCfg.m_profile          = vvenc::Profile::Name::MAIN_10; // profile: use main_10 or main_10_still_picture
  cEncAppCfg.m_level            = vvenc::Level::Name::LEVEL4_1;  // level
  cEncAppCfg.m_levelTier        = vvenc::Level::Tier::MAIN;      // tier
  cEncAppCfg.m_SegmentMode      = vvenc::SEG_OFF;                // segment mode

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

  if( cEncAppCfg.m_numWppThreads <= 0 )
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

  cVVEncParameter.m_eMsgLevel = (vvenc::MsgLevel)cEncAppCfg.m_verbosity;
  cVVEncParameter.m_iWidth    = cEncAppCfg.m_SourceWidth;
  cVVEncParameter.m_iHeight    = cEncAppCfg.m_SourceHeight;

#else
  // set desired encoding options
  cVVEncParameter.m_iQp               = 32;                         // quantization parameter 0-51
  cVVEncParameter.m_iWidth            = 1920;                       // luminance width of input picture
  cVVEncParameter.m_iHeight           = 1080;                       // luminance height of input picture
  cVVEncParameter.m_iGopSize          = 32;                         //  gop size (1: intra only, 16, 32: hierarchical b frames)
  cVVEncParameter.m_eDecodingRefreshType = vvenc::DRT_CRA;          // intra period refresh type
  cVVEncParameter.m_iIDRPeriodSec     = 1;                          // intra period in seconds for IDR/CDR intra refresh/RAP flag (should be > 0)
  cVVEncParameter.m_iIDRPeriod        = 0;                          // intra period in frames for IDR/CDR intra refresh/RAP flag (should be a factor of m_iGopSize)
  cVVEncParameter.m_eMsgLevel         = vvenc::VERBOSE;             // log level > 4 (VERBOSE) enables psnr/rate output
  cVVEncParameter.m_iTemporalRate     = 60;                         // temporal rate (fps)
  cVVEncParameter.m_iTemporalScale    = 1;                          // temporal scale (fps)
  cVVEncParameter.m_iTicksPerSecond   = 90000;                      // ticks per second e.g. 90000 for dts generation
  cVVEncParameter.m_iMaxFrames        = 0;                          // max number of frames to be encoded
  cVVEncParameter.m_iFrameSkip        = 0;                          // number of frames to skip before start encoding
  cVVEncParameter.m_iThreadCount      = -1;                         // number of worker threads (should not exceed the number of physical cpu's)
  cVVEncParameter.m_iQuality          = 2;                          // encoding quality (vs speed) 0: faster, 1: fast, 2: medium, 3: slow, 4: slower
  cVVEncParameter.m_iPerceptualQPA    = 2;                          // percepual qpa adaptation, 0 off, 1 on for sdr(wpsnr), 2 on for sdr(xpsnr), 3 on for hdr(wpsrn), 4 on for hdr(xpsnr), on for hdr(MeanLuma)
  cVVEncParameter.m_iInputBitDepth    = 8;                          // input bitdepth
  cVVEncParameter.m_iInternalBitDepth = 10;                         // internal bitdepth
  cVVEncParameter.m_eProfile          = vvenc::Profile::Name::MAIN_10; // profile: use main_10 or main_10_still_picture
  cVVEncParameter.m_eLevel            = vvenc::Level::Name::LEVEL5_1;  // level
  cVVEncParameter.m_eTier             = vvenc::Level::Tier::MAIN;      // tier
  cVVEncParameter.m_eSegMode          = vvenc::SEG_OFF;                // segment mode

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

  if( cVVEncParameter.m_eMsgLevel > vvenc::SILENT && cVVEncParameter.m_eMsgLevel < vvenc::NOTICE )
  {
    std::cout << "-------------------" << std::endl;
    std::cout << cAppname  << " version " << vvenc::VVEnc::getVersionNumber() << std::endl;
  }

  if( cVVEncParameter.m_iThreadCount < 0 )
  {
    if( cVVEncParameter.m_iWidth > 1920 || cVVEncParameter.m_iHeight > 1080)
    {
      cVVEncParameter.m_iThreadCount = 6;
    }
    else
    {
      cVVEncParameter.m_iThreadCount = 4;
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

  if( cVVEncParameter.m_eMsgLevel > vvenc::WARNING )
  {
    std::cout << "VVEnc info: " << cVVEnc.getEncoderInfo() << std::endl;
  }

  // open output file
  vvcutilities::BinFileWriter cBinFileWriter;
  if( !cOutputfile.empty() )
  {
    if( 0 != cBinFileWriter.open( cOutputfile.c_str() ) )
    {
      std::cout << cAppname  << " [error]: failed to open output file " << cOutputfile << std::endl;
      return -1;
    }
  }

  // --- allocate memory for output packets
  vvenc::VvcAccessUnit cAccessUnit;
  cAccessUnit.m_iBufSize  = cVVEncParameter.m_iWidth * cVVEncParameter.m_iHeight;
  cAccessUnit.m_pucBuffer = new unsigned char [ cAccessUnit.m_iBufSize ];

  // --- start timer
  std::chrono::steady_clock::time_point cTPStartRun;
  std::chrono::steady_clock::time_point cTPEndRun;

  std::time_t startTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  if( cVVEncParameter.m_eMsgLevel > vvenc::WARNING )
  {
    std::cout  << "started @ " << std::ctime(&startTime2)  << std::endl;
  }

  vvenc::InputPicture cInputPicture;
  unsigned char* pucDeletePicBuffer = nullptr;

  unsigned int uiFrames = 0;
  for( int pass = 0; pass < cVVEncParameter.m_iNumPasses; pass++ )
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
    if( 0 != cYuvFileReader.open( cInputFile.c_str(), cVVEncParameter.m_iInputBitDepth, cVVEncParameter.m_iInternalBitDepth, cVVEncParameter.m_iWidth, cVVEncParameter.m_iHeight ) )
    {
      std::cout << cAppname  << " [error]: failed to open input file " << cInputFile << std::endl;
      return -1;
    }

    // allocate input picture buffer
    if( cInputPicture.m_cPicBuffer.m_iWidth == 0 )
    {
      iRet = cYuvFileReader.allocBuffer( cInputPicture.m_cPicBuffer );
      if( 0 != iRet )
      {
        std::cout << cAppname  << " [error]: failed to allocate picture buffer " << std::endl;
        return iRet;
      }
      pucDeletePicBuffer = cInputPicture.m_cPicBuffer.m_pucDeletePicBuffer;
      cInputPicture.m_cPicBuffer.m_pucDeletePicBuffer = NULL;
    }

    const int64_t iFrameSkip  = std::max<int64_t>( cVVEncParameter.m_iFrameSkip - cVVEnc.getNumLeadFrames(), 0 );
    const int64_t iMaxFrames  = cVVEncParameter.m_iMaxFrames + cVVEnc.getNumLeadFrames() + cVVEnc.getNumTrailFrames();
    int64_t       iSeqNumber  = 0;
    bool          bEof        = false;
    uiFrames    = 0;

    while( !bEof )
    {
      iRet = cYuvFileReader.readPicture( cInputPicture.m_cPicBuffer );
      if( iRet )
      {
        if( cVVEncParameter.m_eMsgLevel > vvenc::ERROR && cVVEncParameter.m_eMsgLevel < vvenc::NOTICE )
        {
          std::cout << "EOF reached" << std::endl;
        }
        bEof = true;
      }

      if( !bEof && iSeqNumber >= iFrameSkip )
      {
        // set sequence number and cts
        cInputPicture.m_cPicBuffer.m_uiSequenceNumber = iSeqNumber;
        cInputPicture.m_cPicBuffer.m_uiCts            = iSeqNumber * cVVEncParameter.m_iTicksPerSecond * cVVEncParameter.m_iTemporalScale / cVVEncParameter.m_iTemporalRate;
        cInputPicture.m_cPicBuffer.m_bCtsValid        = true;

        //std::cout << "process picture " << cInputPicture.m_cPicBuffer.m_uiSequenceNumber << " cts " << cInputPicture.m_cPicBuffer.m_uiCts << std::endl;
        // call encode
        iRet = cVVEnc.encode( &cInputPicture, cAccessUnit );
        if( 0 != iRet )
        {
          printVVEncErrorMsg( cAppname, "encoding failed", iRet, cVVEnc.getLastError() );
          return iRet;
        }

        if( 0 != cAccessUnit.m_iUsedSize  )
        {
          if( cBinFileWriter.isOpen())
          {
            // write output
            cBinFileWriter.writeAU( cAccessUnit );
          }

          uiFrames++;
        }
      }
      iSeqNumber++;

      if( iMaxFrames > 0 && iSeqNumber >= ( iFrameSkip + iMaxFrames ) ){ break; }
    }

    // flush the encoder
    while( true )
    {
      iRet = cVVEnc.flush( cAccessUnit );
      if( 0 != iRet )
      {
        printVVEncErrorMsg( cAppname, "flush encoder failed", iRet, cVVEnc.getLastError() );
        return iRet;
      }

      if( 0 == cAccessUnit.m_iUsedSize  )
      {
        break;
      }

      uiFrames++;

      if( cBinFileWriter.isOpen() )
      {
        // write output
        cBinFileWriter.writeAU( cAccessUnit );
      }
    }

    cYuvFileReader.close();
  }

  cTPEndRun = std::chrono::steady_clock::now();
  double dTimeSec = (double)std::chrono::duration_cast<std::chrono::milliseconds>((cTPEndRun)-(cTPStartRun)).count() / 1000;

  delete[] cAccessUnit.m_pucBuffer;
  delete[] pucDeletePicBuffer;

  if( cBinFileWriter.isOpen())
  {
    cBinFileWriter.close();
  }

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

  if( uiFrames && cVVEncParameter.m_eMsgLevel > vvenc::SILENT )
  {
    std::time_t endTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    double dFps = (double)uiFrames / dTimeSec;

    if( cVVEncParameter.m_eMsgLevel > vvenc::WARNING )
    {
      std::cout  << "finished @ " << std::ctime(&endTime2)  << std::endl;
    }
    std::cout << "Total Time: " << dTimeSec << " sec. Fps(avg): " << dFps << " encoded Frames " << uiFrames << std::endl;
  }

  return 0;
}


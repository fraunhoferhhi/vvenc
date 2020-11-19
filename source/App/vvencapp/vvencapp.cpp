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
#include "vvenc/EncoderIf.h"

#include "BinFileWriter.h"
#include "CmdLineParser.h"
#include "YuvFileReader.h"

int g_verbosity = vvenc::LL_VERBOSE;

void msgFnc( int level, const char* fmt, va_list args )
{
  if ( g_verbosity >= level )
  {
    vfprintf( level == 1 ? stderr : stdout, fmt, args );
  }
}

void printVVEncErrorMsg( const std::string cAppname, const std::string cMessage, int code, const std::string cErr )
{
  std::cout << cAppname  << " [error]: " << cMessage << ", ";
  switch( code )
  {
    case vvenc::VVENC_ERR_CPU :       std::cout << "SSE 4.1 cpu support required."; break;
    case vvenc::VVENC_ERR_PARAMETER : std::cout << "invalid parameter."; break;
    default :                         std::cout << "error " << code; break;
  };
  if( !cErr.empty() )
  {
    std::cout << " - " << cErr;
  }
  std::cout << std::endl;
}

int main( int argc, char* argv[] )
{
  std::string cAppname = argv[0];
  std::size_t iPos = (int)cAppname.find_last_of("/");
  if( std::string::npos != iPos )
  {
    cAppname = cAppname.substr(iPos+1 );
  }

  std::string cInputFile;
  std::string cOutputfile = "";

  vvenc::VVEncParameter cVVEncParameter;
  // set desired encoding options
  cVVEncParameter.m_iQp               = 32;                         // quantization parameter 0-51
  cVVEncParameter.m_iWidth            = 1920;                       // luminance width of input picture
  cVVEncParameter.m_iHeight           = 1080;                       // luminance height of input picture
  cVVEncParameter.m_iGopSize          = 32;                         //  gop size (1: intra only, 16, 32: hierarchical b frames)
  cVVEncParameter.m_eDecodingRefreshType = vvenc::VVC_DRT_CRA;      // intra period refresh type
  cVVEncParameter.m_iIDRPeriodSec     = 1;                          // intra period in seconds for IDR/CDR intra refresh/RAP flag (should be > 0)
  cVVEncParameter.m_iIDRPeriod        = 0;                          // intra period in frames for IDR/CDR intra refresh/RAP flag (should be a factor of m_iGopSize)
  cVVEncParameter.m_eLogLevel         = vvenc::LL_VERBOSE;          // log level > 4 (VERBOSE) enables psnr/rate output
  cVVEncParameter.m_iTemporalRate     = 60;                         // temporal rate (fps)
  cVVEncParameter.m_iTemporalScale    = 1;                          // temporal scale (fps)
  cVVEncParameter.m_iTicksPerSecond   = 90000;                      // ticks per second e.g. 90000 for dts generation
  cVVEncParameter.m_iMaxFrames        = 0;                          // max number of frames to be encoded
  cVVEncParameter.m_iFrameSkip        = 0;                          // number of frames to skip before start encoding
  cVVEncParameter.m_iThreadCount      = -1;                         // number of worker threads (should not exceed the number of physical cpu's)
  cVVEncParameter.m_iQuality          = 2;                          // encoding quality (vs speed) 0: faster, 1: fast, 2: medium, 3: slow
  cVVEncParameter.m_iPerceptualQPA    = 2;                          // percepual qpa adaptation, 0 off, 1 on for sdr(wpsnr), 2 on for sdr(xpsnr), 3 on for hdr(wpsrn), 4 on for hdr(xpsnr), on for hdr(MeanLuma)
  cVVEncParameter.m_iInputBitDepth    = 8;                          // input bitdepth
  cVVEncParameter.m_iInternalBitDepth = 10;                         // internal bitdepth
  cVVEncParameter.m_eProfile          = vvenc::VVC_PROFILE_MAIN_10; // profile: use main_10 or main_10_still_picture
  cVVEncParameter.m_eLevel            = vvenc::VVC_LEVEL_4_1;       // level
  cVVEncParameter.m_eTier             = vvenc::VVC_TIER_MAIN;       // tier
  cVVEncParameter.m_eSegMode          = vvenc::VVC_SEG_OFF;         // segment mode

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

  int iRet = vvcutilities::CmdLineParser::parse_command_line(  argc, argv, cVVEncParameter, cInputFile, cOutputfile );

  vvenc::setMsgFnc( &msgFnc );
  g_verbosity = cVVEncParameter.m_eLogLevel;

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

  if( cVVEncParameter.m_eLogLevel > vvenc::LL_SILENT && cVVEncParameter.m_eLogLevel < vvenc::LL_NOTICE )
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

  vvenc::VVEnc cVVEnc;

  // initialize the encoder
  iRet = cVVEnc.init( cVVEncParameter );
  if( 0 != iRet )
  {
    printVVEncErrorMsg( cAppname, "cannot create encoder", iRet, cVVEnc.getLastError() );
    return iRet;
  }

  if( cVVEncParameter.m_eLogLevel > vvenc::LL_WARNING )
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

  vvenc::InputPicture cInputPicture;
  iRet = cVVEnc.getPreferredBuffer( cInputPicture.m_cPicBuffer );
  if( 0 != iRet )
  {
    printVVEncErrorMsg( cAppname, "failed to get preferred buffer", iRet, cVVEnc.getLastError() );
    return iRet;
  }
  const unsigned char* pucDeletePicBuffer = cInputPicture.m_cPicBuffer.m_pucDeletePicBuffer;
  cInputPicture.m_cPicBuffer.m_pucDeletePicBuffer = NULL;

  // --- start timer
  std::chrono::steady_clock::time_point cTPStart;
  std::chrono::steady_clock::time_point cTPEnd;
  cVVEnc.clockStartTime();
  cTPStart = std::chrono::steady_clock::now();
  std::time_t startTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  if( cVVEncParameter.m_eLogLevel > vvenc::LL_WARNING )
  {
    std::cout  << "started @ " << std::ctime(&startTime2)  << std::endl;
  }

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

    const int64_t iFrameSkip  = std::max<int64_t>( cVVEncParameter.m_iFrameSkip - cVVEnc.getNumLeadFrames(), 0 );
    const int64_t iMaxFrames  = cVVEncParameter.m_iMaxFrames + cVVEnc.getNumLeadFrames() + cVVEnc.getNumTrailFrames();
    int64_t       iSeqNumber  = 0;
    bool          bEof        = false;
    unsigned int  uiFramesTmp = 0;
    uiFrames    = 0;

    while( !bEof )
    {
      iRet = cYuvFileReader.readPicture( cInputPicture.m_cPicBuffer );
      if( iRet )
      {
        if( cVVEncParameter.m_eLogLevel > vvenc::LL_ERROR && cVVEncParameter.m_eLogLevel < vvenc::LL_NOTICE )
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
          uiFramesTmp++;

          if( uiFrames && cVVEncParameter.m_eLogLevel > vvenc::LL_WARNING && cVVEncParameter.m_eLogLevel < vvenc::LL_NOTICE)
          {
            cTPEnd = std::chrono::steady_clock::now();
            double dTimeMs = (double)std::chrono::duration_cast<std::chrono::milliseconds>((cTPEnd)-(cTPStart)).count();
            if( dTimeMs > 1000.0 )
            {
              if( cVVEncParameter.m_eLogLevel > vvenc::LL_INFO ){ std::cout << std::endl;}
              std::cout <<  "encoded Frames: " << uiFrames << " Fps: " << uiFramesTmp << std::endl;
              cTPStart = std::chrono::steady_clock::now();
              uiFramesTmp = 0;
            }
          }
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

      if( uiFrames && cVVEncParameter.m_eLogLevel > vvenc::LL_WARNING && cVVEncParameter.m_eLogLevel < vvenc::LL_NOTICE )
      {
        cTPEnd = std::chrono::steady_clock::now();
        double dTimeMs = (double)std::chrono::duration_cast<std::chrono::milliseconds>((cTPEnd)-(cTPStart)).count();
        if( dTimeMs > 1000.0 )
        {
          if( cVVEncParameter.m_eLogLevel > vvenc::LL_INFO ){ std::cout << std::endl;}
          std::cout << "encoded Frames: " << uiFrames << " Fps: " << uiFramesTmp << std::endl;
          cTPStart = std::chrono::steady_clock::now();
          uiFramesTmp = 0;
        }
      }

      if( cBinFileWriter.isOpen() )
      {
        // write output
        cBinFileWriter.writeAU( cAccessUnit );
      }
    }

    cYuvFileReader.close();
  }

  cVVEnc.clockEndTime();
  double dTimeSec = cVVEnc.clockGetTimeDiffMs() / 1000;

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

  if( uiFrames && cVVEncParameter.m_eLogLevel > vvenc::LL_SILENT )
  {
    std::time_t endTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    double dFps = (double)uiFrames / dTimeSec;

    if( cVVEncParameter.m_eLogLevel > vvenc::LL_WARNING )
    {
      std::cout  << "finished @ " << std::ctime(&endTime2)  << std::endl;
    }
    std::cout << "Total Time: " << dTimeSec << " sec. Fps(avg): " << dFps << " encoded Frames " << uiFrames << std::endl;
  }

  return 0;
}


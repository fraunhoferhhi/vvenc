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

/**
  \file    vvencapp.cpp
  \brief   This vvencapp.cpp file contains the main entry point of the application.
*/

#include <iostream>

#include <stdio.h>
#include <string>
#include <fstream>
#include <cstring>
#include <ctime>
#include <chrono>
#include <algorithm>
#include <cstdarg>

#include "vvenc/version.h"
#include "vvenc/vvenc.h"

#include "apputils/ParseArg.h"
#include "apputils/YuvFileIO.h"
#include "apputils/VVEncAppCfg.h"

vvenc::MsgLevel g_verbosity = vvenc::VERBOSE;

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


bool parseCfg( int argc, char* argv[], apputils::VVEncAppCfg& rcVVEncAppCfg )
{
  try
  {
    if( ! rcVVEncAppCfg.parseCfg( argc, argv ) )
    {
      return false;
    }
  }
  catch( apputils::df::program_options_lite::ParseFailure &e )
  {
    msgApp( vvenc::ERROR, "Error parsing option \"%s\" with argument \"%s\".\n", e.arg.c_str(), e.val.c_str() );
    return false;
  }
  g_verbosity = rcVVEncAppCfg.m_verbosity;

  return true;
}

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

  apputils::VVEncAppCfg vvencappCfg;                           ///< encoder configuration
  vvencappCfg.initDefault( 1920, 1080, 60 );

  // parse configuration
  if ( ! parseCfg( argc, argv, vvencappCfg ) )
  {
    return 1;
  }
  // assign verbosity used for encoder output
  g_verbosity = vvencappCfg.m_verbosity; 

  if( vvencappCfg.m_inputFileName.empty() )
  {
    std::cerr << cAppname  << " [error]: no input file given. run VVEncoderApp --help to see available options" << std::endl;
    return -1;
  }

  if( vvencappCfg.m_bitstreamFileName.empty() )
  {
    std::cout << cAppname  << " [error]: no output bitstream file given." << std::endl;
    return -1;
  }

  cInputFile  = vvencappCfg.m_inputFileName;
  cOutputfile = vvencappCfg.m_bitstreamFileName;

  if( vvencappCfg.m_verbosity > vvenc::SILENT && vvencappCfg.m_verbosity < vvenc::NOTICE )
  {
    std::cout << "-------------------" << std::endl;
    std::cout << cAppname  << " version " << vvenc::VVEnc::getVersionNumber() << std::endl;
  }

  vvenc::VVEnc cVVEnc;

  // initialize the encoder
  iRet = cVVEnc.init( vvencappCfg );
  if( 0 != iRet )
  {
    printVVEncErrorMsg( cAppname, "cannot create encoder", iRet, cVVEnc.getLastError() );
    return iRet;
  }

  if( vvencappCfg.m_verbosity > vvenc::WARNING )
  {
    std::cout << cAppname << ": " << cVVEnc.getEncoderInfo() << std::endl;
  }

  cVVEnc.getConfig( vvencappCfg ); // get the adapted config, because changes are needed for the yuv reader (m_MSBExtendedBitDepth)

  if( vvencappCfg.m_verbosity >= vvenc::INFO )
  {
    std::cout << vvencappCfg.getConfigAsString( vvencappCfg.m_verbosity ) << std::endl;
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
  std::chrono::steady_clock::time_point cTPStartRun = std::chrono::steady_clock::now();
  if( vvencappCfg.m_verbosity > vvenc::WARNING )
  {
    std::time_t startTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::cout  << "started @ " << std::ctime(&startTime2)  << std::endl;
  }

  // calc temp. rate/scale
  int temporalRate   = vvencappCfg.m_FrameRate;
  int temporalScale  = 1;

  switch( vvencappCfg.m_FrameRate )
  {
  case 23: temporalRate = 24000; temporalScale = 1001; break;
  case 29: temporalRate = 30000; temporalScale = 1001; break;
  case 59: temporalRate = 60000; temporalScale = 1001; break;
  default: break;
  }

  unsigned int uiFrames = 0;
  for( int pass = 0; pass < vvencappCfg.m_RCNumPasses; pass++ )
  {
    // initialize the encoder pass
    iRet = cVVEnc.initPass( pass );
    if( 0 != iRet )
    {
      printVVEncErrorMsg( cAppname, "cannot init encoder", iRet, cVVEnc.getLastError() );
      return iRet;
    }

    // open the input file
    apputils::YuvFileIO cYuvFileInput;
    if( 0 != cYuvFileInput.open( cInputFile, false, vvencappCfg.m_inputBitDepth[0], vvencappCfg.m_MSBExtendedBitDepth[0], vvencappCfg.m_internalBitDepth[0],
                                 vvencappCfg.m_inputFileChromaFormat, vvencappCfg.m_internChromaFormat, vvencappCfg.m_bClipOutputVideoToRec709Range, false ) )
    {
      std::cout << cAppname  << " [error]: failed to open input file " << cInputFile << std::endl;
      return -1;
    }

    vvenc::YUVBufferStorage cYUVInputBuffer( vvencappCfg.m_internChromaFormat, vvencappCfg.m_SourceWidth, vvencappCfg.m_SourceHeight );

    const int iFrameSkip  = std::max( vvencappCfg.m_FrameSkip - cVVEnc.getNumLeadFrames(), 0 );
    const int64_t iMaxFrames  = vvencappCfg.m_framesToBeEncoded + cVVEnc.getNumLeadFrames() + cVVEnc.getNumTrailFrames();
    int64_t       iSeqNumber  = 0;
    bool          bEof        = false;
    bool          bEncodeDone = false;

    uiFrames    = 0;

    if( iFrameSkip )
    {
      cYuvFileInput.skipYuvFrames(iFrameSkip, vvencappCfg.m_SourceWidth, vvencappCfg.m_SourceHeight);
      iSeqNumber=iFrameSkip;
    }

    while( !bEof || !bEncodeDone )
    {
      vvenc::YUVBuffer* ptrYUVInputBuffer = nullptr;
      if( !bEof )
      {
        if( cYuvFileInput.readYuvBuf( cYUVInputBuffer ) )
        {
          // set sequence number and cts
          cYUVInputBuffer.sequenceNumber  = iSeqNumber;
          cYUVInputBuffer.cts             = iSeqNumber * vvencappCfg.m_TicksPerSecond * temporalScale / temporalRate;
          cYUVInputBuffer.ctsValid        = true;
          ptrYUVInputBuffer               = &cYUVInputBuffer;
          iSeqNumber++;
          //std::cout << "process picture " << cYUVInputBuffer.m_uiSequenceNumber << " cts " << cYUVInputBuffer.m_uiCts << std::endl;
        }
        else
        {
          if( vvencappCfg.m_verbosity > vvenc::ERROR && vvencappCfg.m_verbosity < vvenc::NOTICE )
          {
            std::cout << "EOF reached" << std::endl;
          }
          bEof = true;
        }
      }

      // call encode
      iRet = cVVEnc.encode( ptrYUVInputBuffer, cAccessUnit, bEncodeDone );
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
      }
    }

    cYuvFileInput.close();
  }

  std::chrono::steady_clock::time_point cTPEndRun = std::chrono::steady_clock::now();
  double dTimeSec = (double)std::chrono::duration_cast<std::chrono::milliseconds>((cTPEndRun)-(cTPStartRun)).count() / 1000;

  if( cOutBitstream.is_open() )
  {
    cOutBitstream.close();
  }

  cVVEnc.printSummary();

  // un-initialize the encoder
  iRet = cVVEnc.uninit();
  if( 0 != iRet )
  {
    printVVEncErrorMsg( cAppname, "destroy encoder failed", iRet, cVVEnc.getLastError() );
    return iRet;
  }

  if( 0 == uiFrames )
  {
    std::cout << "no frames encoded" << std::endl;
  }

  if( uiFrames && vvencappCfg.m_verbosity > vvenc::SILENT )
  {
    if( vvencappCfg.m_verbosity > vvenc::WARNING )
    {
      std::time_t endTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
      std::cout  << "finished @ " << std::ctime(&endTime2)  << std::endl;
    }

    double dFps = (double)uiFrames / dTimeSec;
    std::cout << "Total Time: " << dTimeSec << " sec. Fps(avg): " << dFps << " encoded Frames " << uiFrames << std::endl;
  }

  return 0;
}


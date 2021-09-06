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

vvencMsgLevel g_verbosity = VVENC_VERBOSE;

void msgFnc( void*, int level, const char* fmt, va_list args )
{
  if ( g_verbosity >= level )
  {
    vfprintf( level == 1 ? stderr : stdout, fmt, args );
  }
}

void msgApp( void* ctx, int level, const char* fmt, ... )
{
    va_list args;
    va_start( args, fmt );
    msgFnc( ctx, level, fmt, args );
    va_end( args );
}

void printVVEncErrorMsg( const std::string cAppname, const std::string cMessage, int code, const std::string cErr )
{
  std::cout << cAppname  << " [error]: " << cMessage << ", ";
  switch( code )
  {
    case VVENC_ERR_CPU :           std::cout << "SSE 4.1 cpu support required."; break;
    case VVENC_ERR_PARAMETER :     std::cout << "invalid parameter."; break;
    case VVENC_ERR_NOT_SUPPORTED : std::cout << "unsupported request."; break;
    default :                      std::cout << "error " << code; break;
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
    msgApp( nullptr, VVENC_ERROR, "Error parsing option \"%s\" with argument \"%s\".\n", e.arg.c_str(), e.val.c_str() );
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

  vvenc_set_logging_callback( nullptr, msgFnc );

  std::string cInputFile;
  std::string cOutputfile = "";

  apputils::VVEncAppCfg vvencappCfg;                           ///< encoder configuration
  vvenc_init_default( &vvencappCfg, 1920, 1080, 60, 0, 32, vvencPresetMode::VVENC_MEDIUM );

  // parse configuration
  if ( ! parseCfg( argc, argv, vvencappCfg ) )
  {
    return 1;
  }
  // assign verbosity used for encoder output
  g_verbosity = vvencappCfg.m_verbosity;

  if( vvencappCfg.m_showVersion )
  {
    std::cout << cAppname  << " version " << vvenc_get_version()<< std::endl;
    return 0;
  }
  
  if( !strcmp( vvencappCfg.m_inputFileName.c_str(), "-" )  )
  {
    if( vvencappCfg.m_RCNumPasses > 1 )
    {
      std::cout << cAppname << " [error]: 2 pass rate control and reading from stdin is not supported yet" << std::endl;
      return -1;
    }
    else
    {
      std::cout << cAppname << " trying to read from stdin" << std::endl;
    }
  }

  if( vvencappCfg.m_bitstreamFileName.empty() )
  {
    std::cout << cAppname  << " [error]: no output bitstream file given." << std::endl;
    return -1;
  }

  cInputFile  = vvencappCfg.m_inputFileName;
  cOutputfile = vvencappCfg.m_bitstreamFileName;

  if( vvencappCfg.m_verbosity > VVENC_SILENT && vvencappCfg.m_verbosity < VVENC_NOTICE )
  {
    std::cout << "-------------------" << std::endl;
    std::cout << cAppname  << " version " << vvenc_get_version() << std::endl;
  }

  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  // initialize the encoder
  iRet = vvenc_encoder_open( enc, &vvencappCfg );
  if( 0 != iRet )
  {
    printVVEncErrorMsg( cAppname, "cannot create encoder", iRet, vvenc_get_last_error( enc ) );
    vvenc_encoder_close( enc );
    return iRet;
  }

  if( vvencappCfg.m_verbosity > VVENC_WARNING )
  {
    std::cout << cAppname << ": " << vvenc_get_enc_information( enc ) << std::endl;
  }

  vvenc_get_config( enc, &vvencappCfg ); // get the adapted config, because changes are needed for the yuv reader (m_MSBExtendedBitDepth)

  if( vvencappCfg.m_verbosity >= VVENC_INFO )
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
  vvencAccessUnit AU;
  vvenc_accessUnit_default( &AU );
  vvenc_accessUnit_alloc_payload( &AU, 3 * vvencappCfg.m_SourceWidth * vvencappCfg.m_SourceHeight + 1024 );

  // --- allocate memory for YUV input picture
  vvencYUVBuffer cYUVInputBuffer;
  vvenc_YUVBuffer_default( &cYUVInputBuffer );
  vvenc_YUVBuffer_alloc_buffer( &cYUVInputBuffer, vvencappCfg.m_internChromaFormat, vvencappCfg.m_SourceWidth, vvencappCfg.m_SourceHeight );

  // --- start timer
  std::chrono::steady_clock::time_point cTPStartRun = std::chrono::steady_clock::now();
  if( vvencappCfg.m_verbosity > VVENC_WARNING )
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
    iRet = vvenc_init_pass( enc, pass );
    if( 0 != iRet )
    {
      printVVEncErrorMsg( cAppname, "cannot init encoder", iRet, vvenc_get_last_error( enc ) );
      return iRet;
    }

    // open the input file
    apputils::YuvFileIO cYuvFileInput;
    if( 0 != cYuvFileInput.open( cInputFile, false, vvencappCfg.m_inputBitDepth[0], vvencappCfg.m_MSBExtendedBitDepth[0], vvencappCfg.m_internalBitDepth[0],
                                 vvencappCfg.m_inputFileChromaFormat, vvencappCfg.m_internChromaFormat, vvencappCfg.m_bClipOutputVideoToRec709Range, false ) )
    {
      std::cout << cAppname  << " [error]: failed to open input file " << cInputFile << std::endl;
      vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
      vvenc_accessUnit_free_payload( &AU );
      vvenc_encoder_close( enc );
      return -1;
    }

    const int iFrameSkip  = std::max( vvencappCfg.m_FrameSkip - vvenc_get_num_lead_frames(enc), 0 );
    const int64_t iMaxFrames  = vvencappCfg.m_framesToBeEncoded + vvenc_get_num_lead_frames(enc) + vvenc_get_num_trail_frames(enc);
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
      vvencYUVBuffer* ptrYUVInputBuffer = nullptr;
      if( !bEof )
      {
        if( 0 != cYuvFileInput.readYuvBuf( cYUVInputBuffer, bEof ) )
        {
          std::cout << " [error]: read file failed: " << cYuvFileInput.getLastError() << std::endl;
          vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
          vvenc_accessUnit_free_payload( &AU );
          vvenc_encoder_close( enc );
          return -1;
        }
        if( ! bEof )
        {
          // set sequence number and cts
          cYUVInputBuffer.sequenceNumber  = iSeqNumber;
          cYUVInputBuffer.cts             = iSeqNumber * vvencappCfg.m_TicksPerSecond * temporalScale / temporalRate;
          cYUVInputBuffer.ctsValid        = true;
          ptrYUVInputBuffer               = &cYUVInputBuffer;
          iSeqNumber++;
          //std::cout << "process picture " << cYUVInputBuffer.m_uiSequenceNumber << " cts " << cYUVInputBuffer.m_uiCts << std::endl;
        }
        else if( vvencappCfg.m_verbosity > VVENC_ERROR && vvencappCfg.m_verbosity < VVENC_NOTICE )
        {
          std::cout << "EOF reached" << std::endl;
        }
      }

      // call encode
      iRet = vvenc_encode( enc, ptrYUVInputBuffer, &AU, &bEncodeDone );
      if( 0 != iRet )
      {
        printVVEncErrorMsg( cAppname, "encoding failed", iRet, vvenc_get_last_error( enc ) );
        vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
        vvenc_accessUnit_free_payload( &AU );
        vvenc_encoder_close( enc );
        return iRet;
      }

      if( AU.payloadUsedSize > 0 )
      {
        if( cOutBitstream.is_open() )
        {
          // write output
          cOutBitstream.write( (const char*)AU.payload, AU.payloadUsedSize );
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

  vvenc_print_summary(enc);

  // un-initialize the encoder
  iRet = vvenc_encoder_close( enc );
  if( 0 != iRet )
  {
    printVVEncErrorMsg( cAppname, "destroy encoder failed", iRet, vvenc_get_last_error( enc ) );
    return iRet;
  }

  vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
  vvenc_accessUnit_free_payload( &AU );

  if( 0 == uiFrames )
  {
    std::cout << "no frames encoded" << std::endl;
  }

  if( uiFrames && vvencappCfg.m_verbosity > VVENC_SILENT )
  {
    if( vvencappCfg.m_verbosity > VVENC_WARNING )
    {
      std::time_t endTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
      std::cout  << "finished @ " << std::ctime(&endTime2)  << std::endl;
    }

    double dFps = (double)uiFrames / dTimeSec;
    std::cout << "Total Time: " << dTimeSec << " sec. Fps(avg): " << dFps << " encoded Frames " << uiFrames << std::endl;
  }

  return 0;
}


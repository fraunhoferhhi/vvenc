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

void changePreset( vvenc_config* c, vvencPresetMode preset )
{
  if( c ) vvenc_init_preset( c, (vvencPresetMode)preset );
}

void printVVEncErrorMsg( const std::string cAppname, const std::string cMessage, int code, const std::string cErr )
{
  std::cout << cAppname  << " [error]: " << cMessage << ", ";
  switch( code )
  {
    case VVENC_ERR_CPU :           std::cout << "SSE 4.1 cpu support required"; break;
    case VVENC_ERR_PARAMETER :     std::cout << "invalid parameter"; break;
    case VVENC_ERR_NOT_SUPPORTED : std::cout << "unsupported request"; break;
    default :                      std::cout << "code " << code; break;
  };
  if( !cErr.empty() )
  {
    std::cout  << ": " << cErr;
  }
  std::cout << std::endl;
}


bool parseCfg( int argc, char* argv[], apputils::VVEncAppCfg& rcVVEncAppCfg, vvenc_config &vvenccfg )
{
  try
  {
    if( argc )
    {
      // remove application name
      argc--;
      argv++;
    }
    if( 0 != rcVVEncAppCfg.parse( argc, argv, &vvenccfg ) )
    {
      return false;
    }
  }
  catch( apputils::df::program_options_lite::ParseFailure &e )
  {
    msgApp( nullptr, VVENC_ERROR, "Error parsing option \"%s\" with argument \"%s\".\n", e.arg.c_str(), e.val.c_str() );
    return false;
  }
  g_verbosity = vvenccfg.m_verbosity;

  bool ret = true;

  if( !rcVVEncAppCfg.m_additionalSettings.empty() )
  {
    std::vector <std::tuple<std::string, std::string>> dict = rcVVEncAppCfg.getAdditionalSettingList();

    if( dict.empty() )
    {
      msgApp( nullptr, VVENC_ERROR, "Error parsing additional option string \"%s\"\n",rcVVEncAppCfg.m_additionalSettings.c_str() );
      return false;
    }

    for( auto & d : dict )
    {
      std::string key = std::get<0>(d);
      std::string value = std::get<1>(d);
      int parse_ret = vvenc_set_param( &vvenccfg, key.c_str(), value.c_str() );
      switch (parse_ret)
      {
        case VVENC_PARAM_BAD_NAME:
            msgApp( nullptr, VVENC_ERROR, "additional params: unknown option\"%s\" \n", key.c_str() );
            ret = false;
            break;
        case VVENC_PARAM_BAD_VALUE:
          msgApp( nullptr, VVENC_ERROR, "additional params: invalid value for key \"%s\": \"%s\" \n", key.c_str(), value.c_str() );
            ret = false;
            break;
        default:
            break;
      }
    }
  }

  if ( vvenccfg.m_internChromaFormat < 0 || vvenccfg.m_internChromaFormat >= VVENC_NUM_CHROMA_FORMAT )
  {
    vvenccfg.m_internChromaFormat = rcVVEncAppCfg.m_inputFileChromaFormat;
  }

  if( vvenccfg.m_RCNumPasses < 0 && ( vvenccfg.m_RCPass > 0 || vvenccfg.m_RCTargetBitrate > 0 ) )
  {
    vvenccfg.m_RCNumPasses = 2;
  }

  if( vvenc_init_config_parameter( &vvenccfg ) )
  {
    ret = false;
  }

  if( rcVVEncAppCfg.checkCfg( &vvenccfg ))
  {
    ret = false;
  }

  return ret;
}

int main( int argc, char* argv[] )
{
  std::string cAppname = argv[0];
  std::size_t iPos = (int)cAppname.find_last_of("/");
  if( std::string::npos != iPos )
  {
    cAppname = cAppname.substr(iPos+1 );
  }

  // default encoder configuration
  apputils::VVEncAppCfg vvencappCfg = apputils::VVEncAppCfg(true); // init config in easy mode
  vvencappCfg.setPresetChangeCallback(changePreset);
  vvenc_config vvenccfg;
  vvenc_init_default( &vvenccfg, 1920, 1080, 60, 0, 32, vvencPresetMode::VVENC_MEDIUM );

  // parse configuration
  if ( ! parseCfg( argc, argv, vvencappCfg, vvenccfg ) )
  {
    return 1;
  }

  // assign verbosity used for encoder output
  g_verbosity = vvenccfg.m_verbosity;

  // show version
  if( vvencappCfg.m_showVersion
      || ( vvenccfg.m_verbosity > VVENC_SILENT && vvenccfg.m_verbosity < VVENC_NOTICE ) )
  {
    std::cout << cAppname  << " version " << vvenc_get_version()<< std::endl;
    if( vvencappCfg.m_showVersion )
      return 0;
  }

  // initialize the encoder
  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  vvenc_set_logging_callback( enc, nullptr, &::msgFnc );

  int iRet = vvenc_encoder_open( enc, &vvenccfg );
  if( 0 != iRet )
  {
    printVVEncErrorMsg( cAppname, "cannot create encoder", iRet, vvenc_get_last_error( enc ) );
    vvenc_encoder_close( enc );
    return iRet;
  }

  // get the adapted config
  vvenc_get_config( enc, &vvenccfg );
  if( vvenccfg.m_verbosity >= VVENC_INFO )
  {
    std::cout << cAppname << ": " << vvenc_get_enc_information( enc ) << std::endl;
  }
  if( vvenccfg.m_verbosity >= VVENC_INFO )
  {
    std::stringstream css;
    css << vvencappCfg.getAppConfigAsString( vvenccfg.m_verbosity );
    css << vvenc_get_config_as_string( &vvenccfg, vvenccfg.m_verbosity);
    std::cout << css.str() << std::endl;
  }
  if( vvenccfg.m_verbosity >= VVENC_INFO && ! strcmp( vvencappCfg.m_inputFileName.c_str(), "-" )  )
  {
    std::cout << cAppname << " trying to read from stdin" << std::endl;
  }

  // open output file
  std::ofstream cOutBitstream;
  if ( !vvencappCfg.m_bitstreamFileName.empty() )
  {
    cOutBitstream.open( vvencappCfg.m_bitstreamFileName, std::ios::out | std::ios::binary | std::ios::trunc );
    if( ! cOutBitstream.is_open() )
    {
      std::cout << cAppname  << " [error]: failed to open output file " << vvencappCfg.m_bitstreamFileName << std::endl;
      return -1;
    }
  }

  // --- allocate memory for output packets
  vvencAccessUnit AU;
  vvenc_accessUnit_default( &AU );
  const int auSizeScale = vvenccfg.m_internChromaFormat <= VVENC_CHROMA_420 ? 2 : 3;
  vvenc_accessUnit_alloc_payload( &AU, auSizeScale * vvenccfg.m_SourceWidth * vvenccfg.m_SourceHeight + 1024 );

  // --- allocate memory for YUV input picture
  vvencYUVBuffer cYUVInputBuffer;
  vvenc_YUVBuffer_default( &cYUVInputBuffer );
  vvenc_YUVBuffer_alloc_buffer( &cYUVInputBuffer, vvenccfg.m_internChromaFormat, vvenccfg.m_SourceWidth, vvenccfg.m_SourceHeight );

  // --- start timer
  std::chrono::steady_clock::time_point cTPStartRun = std::chrono::steady_clock::now();
  if( vvenccfg.m_verbosity > VVENC_WARNING )
  {
    std::time_t startTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::cout  << "started @ " << std::ctime(&startTime2)  << std::endl;
  }

  unsigned int uiFrames = 0;
  const int start       = vvenccfg.m_RCPass > 0 ? vvenccfg.m_RCPass - 1 : 0;
  const int end         = vvenccfg.m_RCPass > 0 ? vvenccfg.m_RCPass     : vvenccfg.m_RCNumPasses;
  for( int pass = start; pass < end; pass++ )
  {
    // initialize the encoder pass
    iRet = vvenc_init_pass( enc, pass, vvencappCfg.m_RCStatsFileName.c_str() );
    if( 0 != iRet )
    {
      printVVEncErrorMsg( cAppname, "init pass failed", iRet, vvenc_get_last_error( enc ) );
      vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
      vvenc_accessUnit_free_payload( &AU );
      vvenc_encoder_close( enc );
      return iRet;
    }

    // open the input file
    apputils::YuvFileIO cYuvFileInput;
    if( 0 != cYuvFileInput.open( vvencappCfg.m_inputFileName, false, vvenccfg.m_inputBitDepth[0], vvenccfg.m_MSBExtendedBitDepth[0], vvenccfg.m_internalBitDepth[0],
                                 vvencappCfg.m_inputFileChromaFormat, vvenccfg.m_internChromaFormat, vvencappCfg.m_bClipOutputVideoToRec709Range, vvencappCfg.m_packedYUVInput ) )
    {
      std::cout << cAppname  << " [error]: failed to open input file " << vvencappCfg.m_inputFileName << std::endl;
      vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
      vvenc_accessUnit_free_payload( &AU );
      vvenc_encoder_close( enc );
      return -1;
    }

    const int iFrameSkip  = std::max( vvencappCfg.m_FrameSkip - vvenc_get_num_lead_frames(enc), 0 );
    const int64_t iMaxFrames  = vvenccfg.m_framesToBeEncoded + vvenc_get_num_lead_frames(enc) + vvenc_get_num_trail_frames(enc);
    int64_t       iSeqNumber  = 0;
    bool          bEof        = false;
    bool          bEncodeDone = false;

    uiFrames    = 0;

    if( iFrameSkip )
    {
      cYuvFileInput.skipYuvFrames(iFrameSkip, vvenccfg.m_SourceWidth, vvenccfg.m_SourceHeight);
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
          cYUVInputBuffer.cts             = iSeqNumber * vvenccfg.m_TicksPerSecond * vvenccfg.m_FrameScale / vvenccfg.m_FrameRate;
          cYUVInputBuffer.ctsValid        = true;
          ptrYUVInputBuffer               = &cYUVInputBuffer;
          iSeqNumber++;
          //std::cout << "process picture " << cYUVInputBuffer.m_uiSequenceNumber << " cts " << cYUVInputBuffer.m_uiCts << std::endl;
        }
        else if( vvenccfg.m_verbosity > VVENC_ERROR && vvenccfg.m_verbosity < VVENC_NOTICE )
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

  if( uiFrames && vvenccfg.m_verbosity > VVENC_SILENT )
  {
    if( vvenccfg.m_verbosity > VVENC_WARNING )
    {
      std::time_t endTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
      std::cout  << "finished @ " << std::ctime(&endTime2)  << std::endl;
    }

    double dFps = (double)uiFrames / dTimeSec;
    std::cout << "Total Time: " << dTimeSec << " sec. Fps(avg): " << dFps << " encoded Frames " << uiFrames << std::endl;
  }

  return 0;
}

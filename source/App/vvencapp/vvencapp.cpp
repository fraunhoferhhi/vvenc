/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

     * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

     * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

/**
  \file    vvencapp.cpp
  \brief   This vvencapp.cpp file contains the main entry point of the application.
*/

#include <iostream>

#include <stdio.h>
#include <string>
#include <sstream>
#include <fstream>
#include <cstring>
#include <ctime>
#include <chrono>
#include <algorithm>
#include <cstdarg>

#include "vvenc/version.h"
#include "vvenc/vvenc.h"

#include "apputils/YuvFileIO.h"
#include "apputils/VVEncAppCfg.h"
#include "apputils/Stats.h"

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

void printVVEncErrorMsg( const std::string cMessage, int code, const std::string cErr )
{
  std::stringstream errstr;
  errstr << "vvencapp [error]: " << cMessage << ", ";
  switch( code )
  {
    case VVENC_ERR_CPU :           errstr << "SSE 4.1 cpu support required"; break;
    case VVENC_ERR_PARAMETER :     errstr << "invalid parameter"; break;
    case VVENC_ERR_NOT_SUPPORTED : errstr << "unsupported request"; break;
    default :                      errstr << "code " << code; break;
  };
  if( !cErr.empty() )
  {
    errstr  << ": " << cErr;
  }
  msgApp( nullptr, VVENC_ERROR, "%s\n", errstr.str().c_str() );
}


bool parseCfg( int argc, char* argv[], apputils::VVEncAppCfg& rcVVEncAppCfg, vvenc_config &vvenccfg )
{
  std::stringstream cParserStr;
  bool ret = true;

  if( argc )
  {
    // remove application name
    argc--;
    argv++;
  }
  
  int parserRes =  rcVVEncAppCfg.parse( argc, argv, &vvenccfg, cParserStr );
  if( parserRes != 0 )
  {
    if( rcVVEncAppCfg.m_showHelp )
    {
      msgApp( nullptr, VVENC_INFO, "vvencapp: %s\n", vvenc_get_enc_information( nullptr ));
      if( !cParserStr.str().empty() )
        msgApp( nullptr, VVENC_INFO, "%s", cParserStr.str().c_str() );
      return true;
    }
    else if( rcVVEncAppCfg.m_showVersion)
    {
      msgApp( nullptr, VVENC_INFO,"vvencapp version %s\n", vvenc_get_version());
      if( !cParserStr.str().empty() )
        msgApp( nullptr, VVENC_INFO, "%s", cParserStr.str().c_str() );
      return true;
    }
  };

  if(  parserRes >= 0 )  g_verbosity = vvenccfg.m_verbosity;
  else ret = false;

  msgApp( nullptr, VVENC_INFO, "vvencapp: %s\n", vvenc_get_enc_information( nullptr ));

  if( !cParserStr.str().empty() )
    msgApp( nullptr, (parserRes < 0 ) ? VVENC_ERROR : ((parserRes > 0) ? VVENC_WARNING : VVENC_INFO), "%s", cParserStr.str().c_str() );


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
      msgApp( nullptr, VVENC_DETAILS, "additional params: set option key:'%s' value:'%s'\n", key.c_str(), value.c_str() );
      int parse_ret = vvenc_set_param( &vvenccfg, key.c_str(), value.c_str() );
      switch (parse_ret)
      {
        case VVENC_PARAM_BAD_NAME:
            msgApp( nullptr, VVENC_ERROR, "additional params: unknown option \"%s\" \n", key.c_str() );
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

  cParserStr.clear();
  if( rcVVEncAppCfg.checkCfg( &vvenccfg, cParserStr ))
  {
    msgApp( nullptr, VVENC_ERROR, "%s", cParserStr.str().c_str() );      
    ret = false;
  }

  return ret;
}

int main( int argc, char* argv[] )
{
  vvenc_set_logging_callback( nullptr, msgFnc ); // register global log callback ( deprecated, will be removed)

  // default encoder configuration
  apputils::VVEncAppCfg vvencappCfg = apputils::VVEncAppCfg(true); // init config in easy mode
  vvencappCfg.setPresetChangeCallback(changePreset);
  vvenc_config vvenccfg;
  vvenc_init_default( &vvenccfg, 1920, 1080, 60, VVENC_RC_OFF, VVENC_AUTO_QP, vvencPresetMode::VVENC_MEDIUM );

  vvenc_set_msg_callback( &vvenccfg, nullptr, &::msgFnc );  // register local (thread safe) logger (global logger is overwritten )

  // parse configuration
  if ( ! parseCfg( argc, argv, vvencappCfg, vvenccfg ) )
  {
    return 1;
  }

  // show version or help
  if( vvencappCfg.m_showVersion || vvencappCfg.m_showHelp )
  {
    return 0;
  }

  // initialize the encoder
  vvencEncoder *enc = vvenc_encoder_create();
  if( nullptr == enc )
  {
    return -1;
  }

  int iRet = vvenc_encoder_open( enc, &vvenccfg );
  if( 0 != iRet )
  {
    printVVEncErrorMsg( "vvencapp cannot create encoder", iRet, vvenc_get_last_error( enc ) );
    vvenc_encoder_close( enc );
    return iRet;
  }

  // get the adapted config
  vvenc_get_config( enc, &vvenccfg );

  if( vvenccfg.m_verbosity >= VVENC_INFO )
  {
    std::stringstream css;
    css << vvencappCfg.getAppConfigAsString( &vvenccfg, vvenccfg.m_verbosity );
    css << vvenc_get_config_as_string( &vvenccfg, vvenccfg.m_verbosity);
    msgApp( nullptr, VVENC_INFO,"%s\n", css.str().c_str() );
  }
  if( !strcmp( vvencappCfg.m_inputFileName.c_str(), "-" )  )
  {
    msgApp( nullptr, VVENC_INFO,"vvencapp trying to read from stdin\n" );
  }

  // open output file
  std::ofstream cOutBitstream;
  if ( !vvencappCfg.m_bitstreamFileName.empty() )
  {
    cOutBitstream.open( vvencappCfg.m_bitstreamFileName, std::ios::out | std::ios::binary | std::ios::trunc );
    if( ! cOutBitstream.is_open() )
    {
      msgApp( nullptr, VVENC_ERROR, "vvencapp [error]: failed to open output file %s\n", vvencappCfg.m_bitstreamFileName.c_str() );
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
    msgApp( nullptr, VVENC_NOTICE, "vvencapp [notice]: started @ %s", std::ctime(&startTime2) );
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
      printVVEncErrorMsg( "init pass failed", iRet, vvenc_get_last_error( enc ) );
      vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
      vvenc_accessUnit_free_payload( &AU );
      vvenc_encoder_close( enc );
      return iRet;
    }

    // open the input file
    apputils::YuvFileIO cYuvFileInput;
    if( 0 != cYuvFileInput.open( vvencappCfg.m_inputFileName, false, vvenccfg.m_inputBitDepth[0], vvenccfg.m_MSBExtendedBitDepth[0], vvenccfg.m_internalBitDepth[0],
                                 vvencappCfg.m_inputFileChromaFormat, vvenccfg.m_internChromaFormat, vvencappCfg.m_bClipOutputVideoToRec709Range, vvencappCfg.m_packedYUVInput,
                                 vvencappCfg.m_forceY4mInput, vvencappCfg.m_logoFileName ) )
    {
      msgApp( nullptr, VVENC_ERROR, "vvencapp [error]: open input file failed: %s\n", cYuvFileInput.getLastError().c_str() );
      vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
      vvenc_accessUnit_free_payload( &AU );
      vvenc_encoder_close( enc );
      return -1;
    }

    const int iRemSkipFrames = vvencappCfg.m_FrameSkip - vvenccfg.m_leadFrames;
    if( iRemSkipFrames < 0 )
    {
      msgApp( nullptr, VVENC_ERROR, "vvencapp [error]: skip frames (%d) less than number of lead frames required (%d)\n", vvencappCfg.m_FrameSkip, vvenccfg.m_leadFrames );
      vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
      vvenc_accessUnit_free_payload( &AU );
      vvenc_encoder_close( enc );
      return -1;
    }

    const int64_t iMaxFrames  = vvenccfg.m_framesToBeEncoded + vvenccfg.m_leadFrames + vvenccfg.m_trailFrames;
    int64_t       iSeqNumber  = 0;
    bool          bEof        = false;
    bool          bEncodeDone = false;

    uiFrames    = 0;

    if( iRemSkipFrames > 0 )
    {
      if( 0 != cYuvFileInput.skipYuvFrames(iRemSkipFrames, vvenccfg.m_SourceWidth, vvenccfg.m_SourceHeight) )
      {
        if( !strcmp( vvencappCfg.m_inputFileName.c_str(), "-" )  )
          msgApp( nullptr, VVENC_ERROR, "vvencapp [error]: skip %d frames from stdin failed.\n", iRemSkipFrames );
        else
          msgApp( nullptr, VVENC_ERROR, "vvencapp [error]: skip %d frames failed. file contains %d frames only.\n", iRemSkipFrames, cYuvFileInput.countYuvFrames( vvenccfg.m_SourceWidth, vvenccfg.m_SourceHeight ) );

        vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
        vvenc_accessUnit_free_payload( &AU );
        vvenc_encoder_close( enc );
        return -1;
      }

      iSeqNumber=iRemSkipFrames;
    }

    int64_t frameCount =  apputils::VVEncAppCfg::getFrameCount( vvencappCfg.m_inputFileName, vvenccfg.m_SourceWidth, vvenccfg.m_SourceHeight, vvenccfg.m_internChromaFormat, vvenccfg.m_inputBitDepth[0], vvencappCfg.m_packedYUVInput );
    frameCount = std::max<int64_t>( 0, frameCount-vvencappCfg.m_FrameSkip );
    int64_t framesToEncode = (vvenccfg.m_framesToBeEncoded == 0 || vvenccfg.m_framesToBeEncoded >= frameCount) ? frameCount : vvenccfg.m_framesToBeEncoded;

    apputils::Stats cStats;
    cStats.init( vvenccfg.m_FrameRate, vvenccfg.m_FrameScale, (int)framesToEncode, vvenccfg.m_verbosity, "vvenc [info]: " );
    bool statsInfoReady = false;

    while( !bEof || !bEncodeDone )
    {
      vvencYUVBuffer* ptrYUVInputBuffer = nullptr;
      if( !bEof )
      {
        if( 0 != cYuvFileInput.readYuvBuf( cYUVInputBuffer, bEof ) )
        {
          msgApp( nullptr, VVENC_ERROR, "vvencapp [error]: read file failed: %s\n",cYuvFileInput.getLastError().c_str() );
          vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
          vvenc_accessUnit_free_payload( &AU );
          vvenc_encoder_close( enc );
          return -1;
        }
        if( ! bEof )
        {
          // set sequence number and cts
          cYUVInputBuffer.sequenceNumber  = iSeqNumber;
          cYUVInputBuffer.cts             = (vvenccfg.m_TicksPerSecond > 0) ? iSeqNumber * (int64_t)vvenccfg.m_TicksPerSecond * (int64_t)vvenccfg.m_FrameScale / (int64_t)vvenccfg.m_FrameRate : iSeqNumber;
          cYUVInputBuffer.ctsValid        = true;
          ptrYUVInputBuffer               = &cYUVInputBuffer;
          iSeqNumber++;
          //std::cout << "process picture " << cYUVInputBuffer.sequenceNumber << " cts " << cYUVInputBuffer.cts << std::endl;
        }
      }

      // call encode
      iRet = vvenc_encode( enc, ptrYUVInputBuffer, &AU, &bEncodeDone );
      if( 0 != iRet )
      {
        printVVEncErrorMsg( "encoding failed", iRet, vvenc_get_last_error( enc ) );
        vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
        vvenc_accessUnit_free_payload( &AU );
        vvenc_encoder_close( enc );
        return iRet;
      }

      if( AU.payloadUsedSize > 0 )
      {
        if( vvencappCfg.m_printStats )
        {
          cStats.addAU( &AU, &statsInfoReady );
          if( statsInfoReady )
          {
            msgApp( nullptr, VVENC_INFO, cStats.getInfoString().c_str() );
            fflush( stdout );
          }
        }

        if( cOutBitstream.is_open() )
        {
          // write output
          cOutBitstream.write( (const char*)AU.payload, AU.payloadUsedSize );
          if( cOutBitstream.fail() )
          {
            msgApp( nullptr, VVENC_ERROR, "\nvvencapp [error]: write bitstream file failed (disk full?)\n");
            vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
            vvenc_accessUnit_free_payload( &AU );
            vvenc_encoder_close( enc );
            return VVENC_ERR_UNSPECIFIED;
          }
        }
        uiFrames++;
      }

      if( iMaxFrames > 0 && iSeqNumber >= ( iRemSkipFrames + iMaxFrames ) )
      {
        bEof = true;
      }
    }

    cYuvFileInput.close();
    
    if( vvencappCfg.m_printStats )
    {
      msgApp( nullptr, VVENC_INFO, cStats.getFinalStats().c_str() );
      fflush( stdout );
    }
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
    printVVEncErrorMsg( "destroy encoder failed", iRet, vvenc_get_last_error( enc ) );
    return iRet;
  }

  vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
  vvenc_accessUnit_free_payload( &AU );

  if( 0 == uiFrames )
  {
    msgApp( nullptr, VVENC_INFO, "vvencapp [info]: no frames encoded" );
  }

  if( uiFrames && vvenccfg.m_verbosity > VVENC_SILENT )
  {
    if( vvenccfg.m_verbosity > VVENC_WARNING )
    {
      std::time_t endTime2 = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
      msgApp( nullptr, VVENC_NOTICE, "vvencapp [notice]: finished @ %s", std::ctime(&endTime2) );
    }

    double dFps = (double)uiFrames / dTimeSec;
    msgApp( nullptr, VVENC_INFO, "vvencapp [info]: Total Time: %.3f sec. Fps(avg): %.3f encoded Frames %d\n", dTimeSec, dFps, uiFrames );
  }

  return 0;
}

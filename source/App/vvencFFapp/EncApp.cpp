/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/** \file     EncApp.cpp
    \brief    Encoder application class
*/

#include "../vvencFFapp/EncApp.h"

#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <iomanip>

#include "vvenc/vvenc.h"
#include "apputils/Stats.h"


using namespace std;

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================

vvencMsgLevel g_verbosity = VVENC_VERBOSE;

void msgFnc( void* , int level, const char* fmt, va_list args )
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
    msgFnc( nullptr, level, fmt, args );
    va_end( args );
}

// ====================================================================================================================

bool EncApp::parseCfg( int argc, char* argv[])
{
  vvenc_set_msg_callback( &m_vvenc_config, this, &::msgFnc ); // register local (thread safe) logger (global logger is overwritten )
  std::stringstream cParserStr;

  bool ret = true;

  if( argc )
  {
    // remove application name
    argc--;
    argv++;
  }

  int parserRes = m_cEncAppCfg.parse( argc, argv, &m_vvenc_config, cParserStr);
  if( parserRes != 0 )
  {
    if( m_cEncAppCfg.m_showHelp )
    {
      msgApp( VVENC_INFO, "vvencFFapp: %s\n", vvenc_get_enc_information( nullptr ));
      if( !cParserStr.str().empty() )
        msgApp( VVENC_INFO, "%s", cParserStr.str().c_str() );
      return true;
    }
    else if( m_cEncAppCfg.m_showVersion)
    {
      msgApp( VVENC_INFO,"vvencFFapp version %s\n", vvenc_get_version());
      if( !cParserStr.str().empty() )
        msgApp( VVENC_INFO, "%s", cParserStr.str().c_str() );
      return true;
    }
  }

  if(  parserRes >= 0 )  g_verbosity = m_vvenc_config.m_verbosity;
  else ret = false;

  msgApp( VVENC_INFO, "vvencFFapp: %s\n", vvenc_get_enc_information( nullptr ));

  if( !cParserStr.str().empty() )
    msgApp( (parserRes < 0 ) ? VVENC_ERROR : ((parserRes > 0) ? VVENC_WARNING : VVENC_INFO), "%s", cParserStr.str().c_str() );

  if( !m_cEncAppCfg.m_additionalSettings.empty() )
  {
    std::vector <std::tuple<std::string, std::string>> dict = m_cEncAppCfg.getAdditionalSettingList();

    if( dict.empty() )
    {
      msgApp( VVENC_ERROR, "Error parsing additional option string \"%s\"\n",m_cEncAppCfg.m_additionalSettings.c_str() );
      return false;
    }

    for( auto & d : dict )
    {
      std::string key = std::get<0>(d);
      std::string value = std::get<1>(d);
      msgApp( VVENC_DETAILS, "additional params: set option key:'%s' value:'%s'\n", key.c_str(), value.c_str() );
      int parse_ret = vvenc_set_param( &m_vvenc_config, key.c_str(), value.c_str() );
      switch (parse_ret)
      {
        case VVENC_PARAM_BAD_NAME:
            msgApp( VVENC_ERROR, "additional params: unknown option \"%s\" \n", key.c_str() );
            ret = false;
            break;
        case VVENC_PARAM_BAD_VALUE:
          msgApp( VVENC_ERROR, "additional params: invalid value for key \"%s\": \"%s\" \n", key.c_str(), value.c_str() );
            ret = false;
            break;
        default:
            break;
      }
    }
  }

  if ( m_vvenc_config.m_internChromaFormat < 0 || m_vvenc_config.m_internChromaFormat >= VVENC_NUM_CHROMA_FORMAT )
  {
    m_vvenc_config.m_internChromaFormat = m_cEncAppCfg.m_inputFileChromaFormat;
  }

  if( m_vvenc_config.m_RCNumPasses < 0 && ( m_vvenc_config.m_RCPass > 0 || m_vvenc_config.m_RCTargetBitrate > 0 ) )
  {
    m_vvenc_config.m_RCNumPasses = 2;
  }

  if( m_cEncAppCfg.m_decode )
  {
    return ret;
  }

  if( vvenc_init_config_parameter( &m_vvenc_config ) )
  {
    ret = false;
  }

  cParserStr.str("");
  cParserStr.clear();
  if(  m_cEncAppCfg.checkCfg( &m_vvenc_config, cParserStr ))
  {
    msgApp( VVENC_ERROR, "%s", cParserStr.str().c_str() );      
    ret = false;
  }

  return ret;
}

/**
 * main encode function
 */
int EncApp::encode()
{
  apputils::VVEncAppCfg& appCfg   = m_cEncAppCfg;
  vvenc_config&          vvencCfg = m_vvenc_config;

  if( appCfg.m_decode )
  {
    return vvenc_decode_bitstream( appCfg.m_bitstreamFileName.c_str(), vvencCfg.m_traceFile, vvencCfg.m_traceRule );
  }

  // initialize encoder lib
  m_encCtx = vvenc_encoder_create();
  if( nullptr == m_encCtx )
  {
    return -1;
  }

  int iRet = vvenc_encoder_open( m_encCtx, &vvencCfg);
  if( 0 != iRet )
  {
    msgApp( VVENC_ERROR, "open encoder failed: %s\n", vvenc_get_last_error( m_encCtx ) );
    vvenc_encoder_close( m_encCtx );
    return iRet;
  }

  // get the adapted config
  vvenc_get_config( m_encCtx, &vvencCfg);

  std::stringstream css;
  css << appCfg.getAppConfigAsString( &vvencCfg, vvencCfg.m_verbosity );
  css << vvenc_get_config_as_string( &vvencCfg, vvencCfg.m_verbosity);
  css << std::endl;

  msgApp( VVENC_INFO, "%s", css.str().c_str() );
  printChromaFormat();
  if( ! strcmp( appCfg.m_inputFileName.c_str(), "-" )  )
  {
    msgApp( VVENC_INFO, " trying to read from stdin" );
  }

  if( ! openFileIO() )
  {
    vvenc_encoder_close( m_encCtx );
    return -1;
  }

  if( ! appCfg.m_reconFileName.empty() )
  {
    vvenc_encoder_set_RecYUVBufferCallback( m_encCtx, &this->m_yuvReconFile, outputYuv );
  }

  // create buffer for input YUV pic
  vvencYUVBuffer yuvInBuf;
  vvenc_YUVBuffer_default( &yuvInBuf );
  vvenc_YUVBuffer_alloc_buffer( &yuvInBuf, vvencCfg.m_internChromaFormat, vvencCfg.m_SourceWidth, vvencCfg.m_SourceHeight );

  // create sufficient memory for output data
  vvencAccessUnit au;
  vvenc_accessUnit_default( &au );
  const int auSizeScale = vvencCfg.m_internChromaFormat <= VVENC_CHROMA_420 ? 2 : 3;
  vvenc_accessUnit_alloc_payload( &au, auSizeScale * vvencCfg.m_SourceWidth * vvencCfg.m_SourceHeight + 1024 );

  // main loop
  int64_t framesRcvd  = 0;
  const int start = vvencCfg.m_RCPass > 0 ? vvencCfg.m_RCPass - 1 : 0;
  const int end   = vvencCfg.m_RCPass > 0 ? vvencCfg.m_RCPass     : vvencCfg.m_RCNumPasses;
  for( int pass = start; pass < end; pass++ )
  {
    // open input YUV

    if( m_yuvInputFile.open( appCfg.m_inputFileName, false, vvencCfg.m_inputBitDepth[0], vvencCfg.m_MSBExtendedBitDepth[0], vvencCfg.m_internalBitDepth[0],
                             appCfg.m_inputFileChromaFormat, vvencCfg.m_internChromaFormat, appCfg.m_bClipInputVideoToRec709Range, appCfg.m_packedYUVInput,
                             appCfg.m_forceY4mInput, appCfg.m_logoFileName ))
    {
      msgApp( VVENC_ERROR, "open input file failed: %s\n", m_yuvInputFile.getLastError().c_str() );
      vvenc_encoder_close( m_encCtx );
      vvenc_YUVBuffer_free_buffer( &yuvInBuf );
      vvenc_accessUnit_free_payload( &au );
      closeFileIO();
      return -1;
    }

    const int remSkipFrames = appCfg.m_FrameSkip - vvencCfg.m_leadFrames;
    if( remSkipFrames < 0 )
    {
      msgApp( VVENC_ERROR, "skip frames (%d) less than number of lead frames required (%d)\n", appCfg.m_FrameSkip, vvencCfg.m_leadFrames );
      vvenc_encoder_close( m_encCtx );
      vvenc_YUVBuffer_free_buffer( &yuvInBuf );
      vvenc_accessUnit_free_payload( &au );
      closeFileIO();
      return -1;
    }
    if( remSkipFrames > 0 )
    {
      if( 0 != m_yuvInputFile.skipYuvFrames(remSkipFrames, vvencCfg.m_SourceWidth, vvencCfg.m_SourceHeight) )
      {
        if( ! strcmp( appCfg.m_inputFileName.c_str(), "-" )  )
          msgApp( VVENC_ERROR, "skip %d frames from stdin failed\n", remSkipFrames );
        else
          msgApp( VVENC_ERROR, "skip %d frames failed. file contains %d frames only.\n", remSkipFrames, m_yuvInputFile.countYuvFrames( vvencCfg.m_SourceWidth, vvencCfg.m_SourceHeight) );

        vvenc_encoder_close( m_encCtx );
        vvenc_YUVBuffer_free_buffer( &yuvInBuf );
        vvenc_accessUnit_free_payload( &au );
        closeFileIO();
        return -1;
      }
    }

    // initialize encoder pass
    iRet = vvenc_init_pass( m_encCtx, pass, appCfg.m_RCStatsFileName.c_str() );
    if( iRet != 0 )
    {
      msgApp( VVENC_ERROR, "init pass failed: %s\n", vvenc_get_last_error(m_encCtx) );
      vvenc_encoder_close( m_encCtx );
      vvenc_YUVBuffer_free_buffer( &yuvInBuf );
      vvenc_accessUnit_free_payload( &au );
      closeFileIO();
      return -1;
    }

    apputils::Stats cStats;
    int64_t frameCount =  apputils::VVEncAppCfg::getFrameCount( appCfg.m_inputFileName, vvencCfg.m_SourceWidth, vvencCfg.m_SourceHeight, vvencCfg.m_inputBitDepth[0], appCfg.m_packedYUVInput );
    int64_t framesToEncode = (vvencCfg.m_framesToBeEncoded == 0 || vvencCfg.m_framesToBeEncoded >= frameCount) ? frameCount : vvencCfg.m_framesToBeEncoded;
    cStats.init( vvencCfg.m_FrameRate, vvencCfg.m_FrameScale, (int)framesToEncode );
    bool printStats = false;

    // loop over input YUV data
    bool inputDone  = false;
    bool encDone    = false;
         framesRcvd = 0;
    while( ! inputDone || ! encDone )
    {
      // check for more input pictures
      inputDone = ( vvencCfg.m_framesToBeEncoded > 0
          && framesRcvd >= ( vvencCfg.m_framesToBeEncoded + vvencCfg.m_leadFrames + vvencCfg.m_trailFrames ) )
        || m_yuvInputFile.isEof();

      // read input YUV
      if( ! inputDone )
      {
        if( 0 != m_yuvInputFile.readYuvBuf( yuvInBuf, inputDone ) )
        {
          msgApp( VVENC_ERROR, "read input file failed: %s\n", m_yuvInputFile.getLastError().c_str() );
          vvenc_encoder_close( m_encCtx );
          vvenc_YUVBuffer_free_buffer( &yuvInBuf );
          vvenc_accessUnit_free_payload( &au );
          closeFileIO();
          return -1;
        }
        if( !inputDone )
        {
          if( vvencCfg.m_FrameRate > 0 )
          {
            yuvInBuf.cts      = (vvencCfg.m_TicksPerSecond > 0) ? framesRcvd * (int64_t)vvencCfg.m_TicksPerSecond * (int64_t)vvencCfg.m_FrameScale / (int64_t)vvencCfg.m_FrameRate : framesRcvd;
            yuvInBuf.ctsValid = true;
          }
          framesRcvd += 1;
        }
      }

      // encode picture
      vvencYUVBuffer* inputPacket = nullptr;
      if( !inputDone )
      {
        inputPacket = &yuvInBuf;
      }

      iRet = vvenc_encode( m_encCtx, inputPacket, &au, &encDone );
      if( 0 != iRet )
      {
        msgApp( VVENC_ERROR, "encoding failed: err code %d: %s\n", iRet, vvenc_get_last_error(m_encCtx) );
        encDone = true;
        inputDone = true;
      }

      // write out encoded access units
      if( au.payloadUsedSize )
      {
        outputAU( au );

        cStats.addAU( &au, &printStats );
        if( printStats )
        {
          msgApp( VVENC_INFO, cStats.getInfoString().c_str() );
        }
      }
    }

    // close input YUV
    m_yuvInputFile.close();

    msgApp( VVENC_INFO, cStats.getFinalStats().c_str() );
  }

  printRateSummary( framesRcvd - ( vvencCfg.m_leadFrames + vvencCfg.m_trailFrames ) );

  // cleanup encoder lib
  vvenc_encoder_close( m_encCtx );

  vvenc_YUVBuffer_free_buffer( &yuvInBuf );
  vvenc_accessUnit_free_payload( &au );

  closeFileIO();
  return iRet;
}

void EncApp::outputAU( const vvencAccessUnit& au )
{
  m_bitstream.write(reinterpret_cast<const char*>(au.payload), au.payloadUsedSize);

  m_totalBytes     += au.payloadUsedSize;
  m_essentialBytes += au.essentialBytes;

  m_bitstream.flush();
}

void EncApp::outputYuv( void* ctx, vvencYUVBuffer* yuvOutBuf )
{
  apputils::YuvFileIO* yuvReconFile = ctx ? (apputils::YuvFileIO*)ctx : nullptr;
  if( yuvReconFile )
  {
    if ( yuvReconFile->isOpen() && nullptr != yuvOutBuf )
    {
      yuvReconFile->writeYuvBuf( *yuvOutBuf );
    }
  }
}

// ====================================================================================================================
// protected member functions
// ====================================================================================================================

bool EncApp::openFileIO()
{
  // output YUV
  if( ! m_cEncAppCfg.m_reconFileName.empty() )
  {
    if( m_yuvReconFile.open( m_cEncAppCfg.m_reconFileName, true, m_vvenc_config.m_outputBitDepth[0], m_vvenc_config.m_outputBitDepth[0], m_vvenc_config.m_internalBitDepth[0],
                             m_vvenc_config.m_internChromaFormat, m_vvenc_config.m_internChromaFormat, m_cEncAppCfg.m_bClipOutputVideoToRec709Range, m_cEncAppCfg.m_packedYUVOutput, false ))
    {
      msgApp( VVENC_ERROR, "open reconstruction file failed: %s\n", m_yuvReconFile.getLastError().c_str() );
      return false;
    }
  }

  // output bitstream
  if ( !m_cEncAppCfg.m_bitstreamFileName.empty() )
  {
    m_bitstream.open( m_cEncAppCfg.m_bitstreamFileName.c_str(), fstream::binary | fstream::out );
    if( ! m_bitstream )
    {
      msgApp( VVENC_ERROR, "open bitstream file failed\n" );
      return false;
    }
  }

  return true;
}

void EncApp::closeFileIO()
{
  m_yuvInputFile.close();
  if ( ! m_cEncAppCfg.m_reconFileName.empty() )
    m_yuvReconFile.close();
  m_bitstream.close();
}

void EncApp::printRateSummary( int64_t framesRcvd )
{
  vvenc_print_summary( m_encCtx );

  int fps = m_vvenc_config.m_FrameRate/m_vvenc_config.m_FrameScale;
  double time = (double) framesRcvd / fps;
  msgApp( VVENC_DETAILS,"Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time );
  if( m_vvenc_config.m_summaryVerboseness > 0 )
  {
    msgApp( VVENC_DETAILS, "Bytes for SPS/PPS/APS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time );
  }
}

void EncApp::printChromaFormat()
{
  if( m_vvenc_config.m_verbosity >= VVENC_DETAILS )
  {
    std::stringstream ssOut;
    ssOut << std::setw(43) << "Input ChromaFormat = ";
    switch( m_cEncAppCfg.m_inputFileChromaFormat )
    {
      case VVENC_CHROMA_400:  ssOut << "  YUV 400"; break;
      case VVENC_CHROMA_420:  ssOut << "  420"; break;
      case VVENC_CHROMA_422:  ssOut << "  422"; break;
      case VVENC_CHROMA_444:  ssOut << "  444"; break;
      default:          msgApp( VVENC_ERROR, "invalid chroma format\n" );
                        return;
    }
    ssOut << std::endl;

    ssOut << std::setw(43) << "Output (intern) ChromaFormat = ";
    switch( m_vvenc_config.m_internChromaFormat )
    {
      case VVENC_CHROMA_400:  ssOut << "  400"; break;
      case VVENC_CHROMA_420:  ssOut << "  420"; break;
      case VVENC_CHROMA_422:  ssOut << "  422"; break;
      case VVENC_CHROMA_444:  ssOut << "  444"; break;
      default:          msgApp( VVENC_ERROR, "invalid chroma format\n" );
                        return;
    }
    msgApp( VVENC_DETAILS, "%s\n", ssOut.str().c_str() );
  }
}

//! \}

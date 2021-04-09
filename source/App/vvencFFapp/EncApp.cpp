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
#include "apputils/ParseArg.h"

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
  try
  {
    if( ! m_cEncAppCfg.parseCfgFF( argc, argv ) )
    {
      return false;
    }
  }
  catch( apputils::df::program_options_lite::ParseFailure &e )
  {
    msgApp( VVENC_ERROR, "Error parsing option \"%s\" with argument \"%s\".\n", e.arg.c_str(), e.val.c_str() );
    return false;
  }
  g_verbosity = m_cEncAppCfg.conf.m_verbosity;
  return true;
}

/**
 * main encode function
 */
void EncApp::encode()
{
  VVEncCfg& vvencCfg = m_cEncAppCfg.conf;
  if( m_cEncAppCfg.m_decode )
  {
    vvenc_decode_bitstream( m_cEncAppCfg.m_bitstreamFileName.c_str() );
    return;
  }

  // initialize encoder lib
  m_encCtx = vvenc_encoder_create();
  if( nullptr == m_encCtx )
  {
    return;
  }

  if( 0 != vvenc_encoder_open( m_encCtx, &vvencCfg, outputYuv ) )
  {
    msgApp( VVENC_ERROR, vvenc_get_last_error( m_encCtx ) );
    vvenc_encoder_close( m_encCtx );
    return;
  }

  vvenc_get_config( m_encCtx, &vvencCfg ); // get the adapted config, because changes are needed for the yuv reader (m_MSBExtendedBitDepth)

  msgApp( VVENC_INFO, "%s",m_cEncAppCfg.getConfigAsString( vvencCfg.m_verbosity ));

  if( ! openFileIO() )
  {
    return;
  }

  printChromaFormat();

  // create buffer for input YUV pic
  vvencYUVBuffer yuvInBuf;
  vvenc_YUVBuffer_default( &yuvInBuf );
  vvenc_YUVBuffer_alloc_buffer( &yuvInBuf, vvencCfg.m_internChromaFormat, vvencCfg.m_SourceWidth, vvencCfg.m_SourceHeight );

  // main loop
  int tempRate   = vvencCfg.m_FrameRate;
  int tempScale  = 1;
  switch( vvencCfg.m_FrameRate )
  {
    case 23: tempRate = 24000; tempScale = 1001; break;
    case 29: tempRate = 30000; tempScale = 1001; break;
    case 59: tempRate = 60000; tempScale = 1001; break;
    default: break;
  }

  vvencAccessUnit *au;

  int framesRcvd = 0;
  for( int pass = 0; pass < vvencCfg.m_RCNumPasses; pass++ )
  {
    // open input YUV
    if( m_yuvInputFile.open( m_cEncAppCfg.m_inputFileName, false, vvencCfg.m_inputBitDepth[0], vvencCfg.m_MSBExtendedBitDepth[0], vvencCfg.m_internalBitDepth[0],
                             m_cEncAppCfg.m_inputFileChromaFormat, vvencCfg.m_internChromaFormat, m_cEncAppCfg.m_bClipInputVideoToRec709Range, false ))
    { 
      msgApp( VVENC_ERROR, "%s", m_yuvInputFile.getLastError().c_str() );
      break;
    }
    const int skipFrames = vvencCfg.m_FrameSkip - vvencCfg.m_vvencMCTF.MCTFNumLeadFrames;
    if( skipFrames > 0 )
    {
      m_yuvInputFile.skipYuvFrames( skipFrames, vvencCfg.m_SourceWidth, vvencCfg.m_SourceHeight );
    }

    // initialize encoder pass
    vvenc_init_pass( m_encCtx, pass );

    // loop over input YUV data
    bool inputDone  = false;
    bool encDone    = false;
         framesRcvd = 0;
    while( ! inputDone || ! encDone )
    {
      // check for more input pictures
      inputDone = ( vvencCfg.m_framesToBeEncoded > 0
          && framesRcvd >= ( vvencCfg.m_framesToBeEncoded + vvencCfg.m_vvencMCTF.MCTFNumLeadFrames + vvencCfg.m_vvencMCTF.MCTFNumTrailFrames ) )
        || m_yuvInputFile.isEof();

      // read input YUV
      if( ! inputDone )
      {
        inputDone = ! m_yuvInputFile.readYuvBuf( yuvInBuf );
        if( ! inputDone )
        {
          if( vvencCfg.m_FrameRate > 0 )
          {
            yuvInBuf.cts      = framesRcvd * vvencCfg.m_TicksPerSecond * tempScale / tempRate;
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

      int iRet = vvenc_encode( m_encCtx, inputPacket, &au, &encDone );
      if( 0 != iRet )
      {
        msgApp( VVENC_ERROR, "encoding failed: err code %d - %s\n", iRet, vvenc_get_last_error(m_encCtx) );
        encDone = true;
        inputDone = true;
      }

      // write out encoded access units
      outputAU( *au );

      // temporally skip frames
      if( ! inputDone && vvencCfg.m_temporalSubsampleRatio > 1 )
      {
        m_yuvInputFile.skipYuvFrames( vvencCfg.m_temporalSubsampleRatio - 1, vvencCfg.m_SourceWidth, vvencCfg.m_SourceHeight );
      }
    }

    // close input YUV
    m_yuvInputFile.close();
  }

  printRateSummary( framesRcvd - ( vvencCfg.m_vvencMCTF.MCTFNumLeadFrames + vvencCfg.m_vvencMCTF.MCTFNumTrailFrames ) );

  // cleanup encoder lib
  vvenc_encoder_close( m_encCtx );

  vvenc_YUVBuffer_free_buffer( &yuvInBuf );
  vvenc_accessUnit_free( au, true );

  closeFileIO();
}

void EncApp::outputAU( const vvencAccessUnit& au )
{
 if( au.payloadUsedSize )
 {
   return;
 } 

  m_bitstream.write(reinterpret_cast<const char*>(au.payload), au.payloadUsedSize);
  rateStatsAccum( au );
  m_bitstream.flush();
}

void EncApp::outputYuv( void* encapp, vvencYUVBuffer* yuvOutBuf )
{
  auto d = (EncApp*)encapp;

  if ( ! d->m_cEncAppCfg.m_reconFileName.empty() && nullptr != yuvOutBuf )
  {
    d->m_yuvReconFile.writeYuvBuf( *yuvOutBuf );
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
    if( m_yuvReconFile.open( m_cEncAppCfg.m_reconFileName, true, m_cEncAppCfg.conf.m_outputBitDepth[0], m_cEncAppCfg.conf.m_outputBitDepth[0], m_cEncAppCfg.conf.m_internalBitDepth[0],
                             m_cEncAppCfg.conf.m_internChromaFormat, m_cEncAppCfg.conf.m_internChromaFormat, m_cEncAppCfg.m_bClipOutputVideoToRec709Range, m_cEncAppCfg.m_packedYUVMode ))
    {
      msgApp( VVENC_ERROR, "%s", m_yuvReconFile.getLastError().c_str() );
      return false;
    }
  }

  // output bitstream
  m_bitstream.open( m_cEncAppCfg.m_bitstreamFileName.c_str(), fstream::binary | fstream::out );
  if( ! m_bitstream )
  {
    msgApp( VVENC_ERROR, "Failed to open bitstream file %s for writing\n", m_cEncAppCfg.m_bitstreamFileName.c_str() );
    return false;
  }

  return true;
}

void EncApp::closeFileIO()
{
  m_yuvInputFile.close();
  m_yuvReconFile.close();
  m_bitstream.close();
}

void EncApp::rateStatsAccum(const vvencAccessUnit& au )
{
//  std::vector<vvencNalUnitType>::const_iterator it_nal = au.nalUnitTypeVec.begin();
//  std::vector<uint32_t>::const_iterator it_nalsize = au.annexBsizeVec.begin();

//  for( ; it_nal != au.nalUnitTypeVec.end(); it_nal++, it_nalsize++ )
//  {
//    switch( (*it_nal) )
//    {
//      case VVENC_NAL_UNIT_CODED_SLICE_TRAIL:
//      case VVENC_NAL_UNIT_CODED_SLICE_STSA:
//      case VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL:
//      case VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP:
//      case VVENC_NAL_UNIT_CODED_SLICE_CRA:
//      case VVENC_NAL_UNIT_CODED_SLICE_GDR:
//      case VVENC_NAL_UNIT_CODED_SLICE_RADL:
//      case VVENC_NAL_UNIT_CODED_SLICE_RASL:
//      case VVENC_NAL_UNIT_DCI:
//      case VVENC_NAL_UNIT_VPS:
//      case VVENC_NAL_UNIT_SPS:
//      case VVENC_NAL_UNIT_PPS:
//      case VVENC_NAL_UNIT_PREFIX_APS:
//      case VVENC_NAL_UNIT_SUFFIX_APS:
//        m_essentialBytes += *it_nalsize;
//        break;
//      default:
//        break;
//    }

//    m_totalBytes += *it_nalsize;
//  }
}

void EncApp::printRateSummary( int framesRcvd )
{
  vvenc_print_summary( m_encCtx );

  double time = (double) framesRcvd / m_cEncAppCfg.conf.m_FrameRate * m_cEncAppCfg.conf.m_temporalSubsampleRatio;
  msgApp( VVENC_DETAILS,"Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time );
  if( m_cEncAppCfg.conf.m_summaryVerboseness > 0 )
  {
    msgApp( VVENC_DETAILS, "Bytes for SPS/PPS/APS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time );
  }
}

void EncApp::printChromaFormat()
{
  if( m_cEncAppCfg.conf.m_verbosity >= VVENC_DETAILS )
  {
    std::stringstream ssOut;
    ssOut << std::setw(43) << "Input ChromaFormat = ";
    switch( m_cEncAppCfg.m_inputFileChromaFormat )
    {
      case VVENC_CHROMA_400:  ssOut << "  YUV 400"; break;
      case VVENC_CHROMA_420:  ssOut << "  420"; break;
      case VVENC_CHROMA_422:  ssOut << "  422"; break;
      case VVENC_CHROMA_444:  ssOut << "  444"; break;
      default:          msgApp( VVENC_ERROR, "invalid chroma format" );
                        return;
    }
    ssOut << std::endl;

    ssOut << std::setw(43) << "Output (intern) ChromaFormat = ";
    switch( m_cEncAppCfg.conf.m_internChromaFormat )
    {
      case VVENC_CHROMA_400:  ssOut << "  400"; break;
      case VVENC_CHROMA_420:  ssOut << "  420"; break;
      case VVENC_CHROMA_422:  ssOut << "  422"; break;
      case VVENC_CHROMA_444:  ssOut << "  444"; break;
      default:          msgApp( VVENC_ERROR, "invalid chroma format" );
                        return;
    }
    msgApp( VVENC_DETAILS, "%s\n", ssOut.str().c_str() );
  }
}

//! \}


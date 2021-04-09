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
using namespace vvenc;

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================

vvenc::MsgLevel g_verbosity = VERBOSE;

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
    msgApp( vvenc::ERROR, "Error parsing option \"%s\" with argument \"%s\".\n", e.arg.c_str(), e.val.c_str() );
    return false;
  }
  g_verbosity = m_cEncAppCfg.m_verbosity;
  return true;
}

/**
 * main encode function
 */
void EncApp::encode()
{
  if( m_cEncAppCfg.m_decode )
  {
    vvenc::decodeBitstream( m_cEncAppCfg.m_bitstreamFileName );
    return;
  }

  // initialize encoder lib
  if( 0 != m_cVVEnc.init( m_cEncAppCfg, this ) )
  {
    msgApp( ERROR, m_cVVEnc.getLastError().c_str() );
    return;
  }

  m_cVVEnc.getConfig( m_cEncAppCfg ); // get the adapted config, because changes are needed for the yuv reader (m_MSBExtendedBitDepth)

  msgApp( vvenc::INFO, "%s",m_cEncAppCfg.getConfigAsString( m_cEncAppCfg.m_verbosity).c_str() );

  if( !strcmp( m_cEncAppCfg.m_inputFileName.c_str(), "-" ) )
  {
    if( m_cEncAppCfg.m_RCNumPasses > 1 )
    {
      msgApp( ERROR, " 2 pass rate control and reading from stdin is not supported yet\n" );
      return;
    }
    else
    {
      msgApp( INFO, " trying to read from stdin\n" );
    }
  }

  if( ! openFileIO() )
  {
    return;
  }

  printChromaFormat();

  // create buffer for input YUV pic
  YUVBufferStorage yuvInBuf( m_cEncAppCfg.m_internChromaFormat, m_cEncAppCfg.m_SourceWidth, m_cEncAppCfg.m_SourceHeight );

  // main loop
  int tempRate   = m_cEncAppCfg.m_FrameRate;
  int tempScale  = 1;
  switch( m_cEncAppCfg.m_FrameRate )
  {
    case 23: tempRate = 24000; tempScale = 1001; break;
    case 29: tempRate = 30000; tempScale = 1001; break;
    case 59: tempRate = 60000; tempScale = 1001; break;
    default: break;
  }
  
  vvenc::AccessUnit au;

  int framesRcvd = 0;
  for( int pass = 0; pass < m_cEncAppCfg.m_RCNumPasses; pass++ )
  {
    // open input YUV
    if( m_yuvInputFile.open( m_cEncAppCfg.m_inputFileName, false, m_cEncAppCfg.m_inputBitDepth[0], m_cEncAppCfg.m_MSBExtendedBitDepth[0], m_cEncAppCfg.m_internalBitDepth[0], 
                             m_cEncAppCfg.m_inputFileChromaFormat, m_cEncAppCfg.m_internChromaFormat, m_cEncAppCfg.m_bClipInputVideoToRec709Range, false ))
    { 
      msgApp( ERROR, "%s", m_yuvInputFile.getLastError().c_str() );
      break;
    }
    const int skipFrames = m_cEncAppCfg.m_FrameSkip - m_cEncAppCfg.m_MCTFNumLeadFrames;
    if( skipFrames > 0 )
    {
      m_yuvInputFile.skipYuvFrames( skipFrames, m_cEncAppCfg.m_SourceWidth, m_cEncAppCfg.m_SourceHeight );
    }

    // initialize encoder pass
    m_cVVEnc.initPass( pass );

    // loop over input YUV data
    bool inputDone  = false;
    bool encDone    = false;
         framesRcvd = 0;
    while( ! inputDone || ! encDone )
    {
      // check for more input pictures
      inputDone = ( m_cEncAppCfg.m_framesToBeEncoded > 0
          && framesRcvd >= ( m_cEncAppCfg.m_framesToBeEncoded + m_cEncAppCfg.m_MCTFNumLeadFrames + m_cEncAppCfg.m_MCTFNumTrailFrames ) )
        || m_yuvInputFile.isEof();

      // read input YUV
      if( ! inputDone )
      {
        inputDone = ! m_yuvInputFile.readYuvBuf( yuvInBuf );
        if( ! inputDone )
        {
          if( m_cEncAppCfg.m_FrameRate > 0 )
          {
            yuvInBuf.cts      = framesRcvd * m_cEncAppCfg.m_TicksPerSecond * tempScale / tempRate;
            yuvInBuf.ctsValid = true;
          }

          framesRcvd += 1;
        }
      }

      // encode picture
      YUVBuffer* inputPacket = nullptr;
      if( !inputDone )
      {
        inputPacket = &yuvInBuf;
      }

      int iRet = m_cVVEnc.encode( inputPacket, au, encDone );
      if( 0 != iRet )
      {
        msgApp( ERROR, "encoding failed: err code %d - %s\n", iRet, m_cVVEnc.getLastError().c_str() );
        encDone = true;
        inputDone = true;
      }

      // write out encoded access units
      outputAU( au );

      // temporally skip frames
      if( ! inputDone && m_cEncAppCfg.m_temporalSubsampleRatio > 1 )
      {
        m_yuvInputFile.skipYuvFrames( m_cEncAppCfg.m_temporalSubsampleRatio - 1, m_cEncAppCfg.m_SourceWidth, m_cEncAppCfg.m_SourceHeight );
      }
    }

    // close input YUV
    m_yuvInputFile.close();
  }

  printRateSummary( framesRcvd - ( m_cEncAppCfg.m_MCTFNumLeadFrames + m_cEncAppCfg.m_MCTFNumTrailFrames ) );

  // cleanup encoder lib
  m_cVVEnc.uninit();

  closeFileIO();
}

void EncApp::outputAU( const AccessUnit& au )
{
 if( au.payload.empty() )
 {
   return;
 } 

  m_bitstream.write(reinterpret_cast<const char*>(au.payload.data()), au.payload.size());
  rateStatsAccum( au );
  m_bitstream.flush();
}

void EncApp::outputYuv( const YUVBuffer& yuvOutBuf )
{
  if ( ! m_cEncAppCfg.m_reconFileName.empty() )
  {
    m_yuvReconFile.writeYuvBuf( yuvOutBuf );
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
    if( m_yuvReconFile.open( m_cEncAppCfg.m_reconFileName, true, m_cEncAppCfg.m_outputBitDepth[0], m_cEncAppCfg.m_outputBitDepth[0], m_cEncAppCfg.m_internalBitDepth[0], 
                             m_cEncAppCfg.m_internChromaFormat, m_cEncAppCfg.m_internChromaFormat, m_cEncAppCfg.m_bClipOutputVideoToRec709Range, m_cEncAppCfg.m_packedYUVMode ))
    {
      msgApp( ERROR, "%s", m_yuvReconFile.getLastError().c_str() );
      return false;
    }
  }

  // output bitstream
  m_bitstream.open( m_cEncAppCfg.m_bitstreamFileName.c_str(), fstream::binary | fstream::out );
  if( ! m_bitstream )
  {
    msgApp( ERROR, "Failed to open bitstream file %s for writing\n", m_cEncAppCfg.m_bitstreamFileName.c_str() );
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

void EncApp::rateStatsAccum(const AccessUnit& au )
{
  std::vector<NalUnitType>::const_iterator it_nal = au.nalUnitTypeVec.begin();
  std::vector<uint32_t>::const_iterator it_nalsize = au.annexBsizeVec.begin();

  for( ; it_nal != au.nalUnitTypeVec.end(); it_nal++, it_nalsize++ )
  {
    switch( (*it_nal) )
    {
      case NAL_UNIT_CODED_SLICE_TRAIL:
      case NAL_UNIT_CODED_SLICE_STSA:
      case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
      case NAL_UNIT_CODED_SLICE_IDR_N_LP:
      case NAL_UNIT_CODED_SLICE_CRA:
      case NAL_UNIT_CODED_SLICE_GDR:
      case NAL_UNIT_CODED_SLICE_RADL:
      case NAL_UNIT_CODED_SLICE_RASL:
      case NAL_UNIT_DCI:
      case NAL_UNIT_VPS:
      case NAL_UNIT_SPS:
      case NAL_UNIT_PPS:
      case NAL_UNIT_PREFIX_APS:
      case NAL_UNIT_SUFFIX_APS:
        m_essentialBytes += *it_nalsize;
        break;
      default:
        break;
    }

    m_totalBytes += *it_nalsize;
  }
}

void EncApp::printRateSummary( int framesRcvd )
{
  m_cVVEnc.printSummary();

  double time = (double) framesRcvd / m_cEncAppCfg.m_FrameRate * m_cEncAppCfg.m_temporalSubsampleRatio;
  msgApp( DETAILS,"Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time );
  if( m_cEncAppCfg.m_summaryVerboseness > 0 )
  {
    msgApp( DETAILS, "Bytes for SPS/PPS/APS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time );
  }
}

void EncApp::printChromaFormat()
{
  if( m_cEncAppCfg.m_verbosity >= DETAILS )
  {
    std::stringstream ssOut;
    ssOut << std::setw(43) << "Input ChromaFormat = ";
    switch( m_cEncAppCfg.m_inputFileChromaFormat )
    {
      case CHROMA_400:  ssOut << "  YUV 400"; break;
      case CHROMA_420:  ssOut << "  420"; break;
      case CHROMA_422:  ssOut << "  422"; break;
      case CHROMA_444:  ssOut << "  444"; break;
      default:          msgApp( ERROR, "invalid chroma format" );
                        return;
    }
    ssOut << std::endl;

    ssOut << std::setw(43) << "Output (intern) ChromaFormat = ";
    switch( m_cEncAppCfg.m_internChromaFormat )
    {
      case CHROMA_400:  ssOut << "  400"; break;
      case CHROMA_420:  ssOut << "  420"; break;
      case CHROMA_422:  ssOut << "  422"; break;
      case CHROMA_444:  ssOut << "  444"; break;
      default:          msgApp( ERROR, "invalid chroma format" );
                        return;
    }
    msgApp( DETAILS, "%s\n", ssOut.str().c_str() );
  }
}

//! \}


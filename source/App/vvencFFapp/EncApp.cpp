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

#include "vvenc/Nal.h"
#include "../vvencFFapp/ParseArg.h"

using namespace std;

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================

int g_verbosity = VERBOSE;

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

bool EncApp::parseCfg( int argc, char* argv[] )
{
  try
  {
    if( ! m_cEncAppCfg.parseCfg( argc, argv ) )
    {
      return false;
    }
  }
  catch ( VVCEncoderFFApp::df::program_options_lite::ParseFailure &e )
  {
    msgApp( ERROR, "Error parsing option \"%s\" with argument \"%s\".\n", e.arg.c_str(), e.val.c_str() );
    return false;
  }

  if( ! m_cEncAppCfg.m_decode )
  {
    m_cEncAppCfg.printCfg();
  }

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

  if ( ! openFileIO() )
    return;

  // create encoder lib
  m_cEncoderIf.createEncoderLib( m_cEncAppCfg, this );

  printChromaFormat();

  // create buffer for input YUV pic
  YUVBufferStorage yuvInBuf( m_cEncAppCfg.m_internChromaFormat, m_cEncAppCfg.m_SourceWidth, m_cEncAppCfg.m_SourceHeight );

  // main loop
  int  framesRcvd = 0;
  int iTempRate = m_cEncAppCfg.m_FrameRate;
  int iTempScale = 1;
  switch( m_cEncAppCfg.m_FrameRate )
  {
  case 23: iTempRate = 24000; iTempScale = 1001; break;
  case 29: iTempRate = 30000; iTempScale = 1001; break;
  case 59: iTempRate = 60000; iTempScale = 1001; break;
  default: break;
  }

  bool inputDone  = false;
  bool encDone    = false;
  while ( ! inputDone || ! encDone )
  {
    // check for more input pictures
    inputDone = ( m_cEncAppCfg.m_framesToBeEncoded > 0 && framesRcvd >= ( m_cEncAppCfg.m_framesToBeEncoded + m_cEncAppCfg.m_MCTFNumLeadFrames + m_cEncAppCfg.m_MCTFNumTrailFrames ) ) || m_yuvInputFile.isEof();

    // read input YUV
    if ( ! inputDone )
    {
      inputDone = ! m_yuvInputFile.readYuvBuf( yuvInBuf, m_cEncAppCfg.m_inputFileChromaFormat, m_cEncAppCfg.m_internChromaFormat, m_cEncAppCfg.m_aiPad, m_cEncAppCfg.m_bClipInputVideoToRec709Range );
      if ( ! inputDone )
      {
        if( m_cEncAppCfg.m_FrameRate > 0 )
        {
          yuvInBuf.cts = framesRcvd * m_cEncAppCfg.m_TicksPerSecond * iTempScale / iTempRate;
          yuvInBuf.ctsValid = true;
        }

        framesRcvd += 1;
      }
    }

    // encode picture
    AccessUnit au;
    m_cEncoderIf.encodePicture( inputDone, yuvInBuf, au, encDone );

    // write out encoded access units
    if ( au.size() )
    {
      outputAU( au );
    }

    // temporally skip frames
    if ( ! inputDone && m_cEncAppCfg.m_temporalSubsampleRatio > 1 )
    {
      m_yuvInputFile.skipYuvFrames( m_cEncAppCfg.m_temporalSubsampleRatio - 1, m_cEncAppCfg.m_inputFileChromaFormat, m_cEncAppCfg.m_SourceWidth - m_cEncAppCfg.m_aiPad[ 0 ], m_cEncAppCfg.m_SourceHeight - m_cEncAppCfg.m_aiPad[ 1 ] );
    }
  }

  printRateSummary( framesRcvd - ( m_cEncAppCfg.m_MCTFNumLeadFrames + m_cEncAppCfg.m_MCTFNumTrailFrames ) );

  // destroy encoder lib
  m_cEncoderIf.destroyEncoderLib();

  closeFileIO();
}

void EncApp::outputAU( const AccessUnit& au )
{
  const vector<uint32_t>& stats = writeAnnexB( m_bitstream, au );
  rateStatsAccum( au, stats );
  m_bitstream.flush();
}

void EncApp::outputYuv( const YUVBuffer& yuvOutBuf )
{
  if ( ! m_cEncAppCfg.m_reconFileName.empty() )
  {
    m_yuvReconFile.writeYuvBuf( yuvOutBuf, m_cEncAppCfg.m_internChromaFormat, m_cEncAppCfg.m_internChromaFormat, m_cEncAppCfg.m_packedYUVMode, m_cEncAppCfg.m_bClipOutputVideoToRec709Range );
  }
}

// ====================================================================================================================
// protected member functions
// ====================================================================================================================

bool EncApp::openFileIO()
{
  // input YUV
  m_yuvInputFile.open( m_cEncAppCfg.m_inputFileName, false, m_cEncAppCfg.m_inputBitDepth, m_cEncAppCfg.m_MSBExtendedBitDepth, m_cEncAppCfg.m_internalBitDepth );
  const int skipFrames = m_cEncAppCfg.m_FrameSkip - m_cEncAppCfg.m_MCTFNumLeadFrames;
  if ( skipFrames > 0 )
  {
    m_yuvInputFile.skipYuvFrames( skipFrames, m_cEncAppCfg.m_inputFileChromaFormat, m_cEncAppCfg.m_SourceWidth - m_cEncAppCfg.m_aiPad[ 0 ], m_cEncAppCfg.m_SourceHeight - m_cEncAppCfg.m_aiPad[ 1 ] );
  }

  // output YUV
  if ( ! m_cEncAppCfg.m_reconFileName.empty() )
  {
    if ( m_cEncAppCfg.m_packedYUVMode && ( ( m_cEncAppCfg.m_outputBitDepth[ CH_L ] != 10 && m_cEncAppCfg.m_outputBitDepth[ CH_L ] != 12 )
        || ( ( ( m_cEncAppCfg.m_SourceWidth - m_cEncAppCfg.m_aiPad[ 0 ] ) & ( 1 + ( m_cEncAppCfg.m_outputBitDepth[ CH_L ] & 3 ) ) ) != 0 ) ) )
    {
      msgApp( ERROR, "Invalid output bit-depth or image width for packed YUV output, aborting\n" );
      return false;
    }
    if ( m_cEncAppCfg.m_packedYUVMode && ( m_cEncAppCfg.m_internChromaFormat != CHROMA_400 ) && ( ( m_cEncAppCfg.m_outputBitDepth[ CH_C ] != 10 && m_cEncAppCfg.m_outputBitDepth[ CH_C ] != 12 )
        || ( ( getWidthOfComponent( m_cEncAppCfg.m_internChromaFormat, m_cEncAppCfg.m_SourceWidth - m_cEncAppCfg.m_aiPad[ 0 ], 1 ) & ( 1 + ( m_cEncAppCfg.m_outputBitDepth[ CH_C ] & 3 ) ) ) != 0 ) ) )
    {
      msgApp( ERROR, "Invalid chroma output bit-depth or image width for packed YUV output, aborting\n" );
      return false;
    }

    m_yuvReconFile.open( m_cEncAppCfg.m_reconFileName, true, m_cEncAppCfg.m_outputBitDepth, m_cEncAppCfg.m_outputBitDepth, m_cEncAppCfg.m_internalBitDepth );
  }

  // output bitstream
  m_bitstream.open( m_cEncAppCfg.m_bitstreamFileName.c_str(), fstream::binary | fstream::out );
  if ( ! m_bitstream )
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

void EncApp::rateStatsAccum(const AccessUnit& au, const std::vector<uint32_t>& annexBsizes)
{
  AccessUnit::const_iterator it_au = au.begin();
  vector<uint32_t>::const_iterator it_stats = annexBsizes.begin();

  for (; it_au != au.end(); it_au++, it_stats++)
  {
    switch ((*it_au)->m_nalUnitType)
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
      m_essentialBytes += *it_stats;
      break;
    default:
      break;
    }

    m_totalBytes += *it_stats;
  }
}

void EncApp::printRateSummary( int framesRcvd )
{
  m_cEncoderIf.printSummary();

  double time = (double) framesRcvd / m_cEncAppCfg.m_FrameRate * m_cEncAppCfg.m_temporalSubsampleRatio;
  msgApp( DETAILS,"Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time );
  if (m_cEncAppCfg.m_summaryVerboseness > 0)
  {
    msgApp( DETAILS, "Bytes for SPS/PPS/APS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes, 0.008 * m_essentialBytes / time );
  }
}

void EncApp::printChromaFormat()
{
  if ( m_cEncAppCfg.m_verbosity >= DETAILS )
  {
    std::stringstream ssOut;
    ssOut << std::setw(43) << "Input ChromaFormat = ";
    switch ( m_cEncAppCfg.m_inputFileChromaFormat )
    {
    case CHROMA_400:  ssOut << "  YUV 400"; break;
    case CHROMA_420:  ssOut << "  420"; break;
    case CHROMA_422:  ssOut << "  422"; break;
    case CHROMA_444:  ssOut << "  444"; break;
    default:
      msgApp( ERROR, "invalid chroma format" );
      return;
    }
    ssOut << std::endl;

    ssOut << std::setw(43) << "Output (intern) ChromaFormat = ";
    switch ( m_cEncAppCfg.m_internChromaFormat )
    {
    case CHROMA_400:  ssOut << "  400"; break;
    case CHROMA_420:  ssOut << "  420"; break;
    case CHROMA_422:  ssOut << "  422"; break;
    case CHROMA_444:  ssOut << "  444"; break;
    default:
      msgApp( ERROR, "invalid chroma format" );
      return;
    }
    msgApp( DETAILS, "%s\n", ssOut.str().c_str() );
  }
}

//! \}


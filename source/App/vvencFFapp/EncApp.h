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
/** \file     EncApp.h
    \brief    Encoder application class (header)
*/

#pragma once

#include <ostream>
#include <cstdarg>

#include "apputils/EncAppCfg.h"
#include "vvenc/vvenc.h"
#include "vvenc/FileIO.h"

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================

extern int g_verbosity;
void msgFnc( int level, const char* fmt, va_list args );
void msgApp( int level, const char* fmt, ... );

// ====================================================================================================================

static inline int getWidthOfComponent( const ChromaFormat& chFmt, const int frameWidth, const int compId )
{
  int w = frameWidth;
  if ( compId > 0 )
  {
    switch ( chFmt )
    {
      case CHROMA_400: w = 0;      break;
      case CHROMA_420:
      case CHROMA_422: w = w >> 1; break;
      default: break;
    }
  }
  return w;
}

static inline int getHeightOfComponent( const ChromaFormat& chFmt, const int frameHeight, const int compId )
{
  int h = frameHeight;
  if ( compId > 0 )
  {
    switch ( chFmt )
    {
      case CHROMA_400: h = 0;      break;
      case CHROMA_420: h = h >> 1; break;
      case CHROMA_422:
      default: break;
    }
  }
  return h;
}

struct YUVBufferStorage : public YUVBuffer
{
  YUVBufferStorage( const ChromaFormat& chFmt, const int frameWidth, const int frameHeight )
    : YUVBuffer()
  {
    for ( int i = 0; i < MAX_NUM_COMP; i++ )
    {
      YUVPlane& yuvPlane = yuvPlanes[ i ];
      yuvPlane.width     = getWidthOfComponent ( chFmt, frameWidth,  i );
      yuvPlane.height    = getHeightOfComponent( chFmt, frameHeight, i );
      yuvPlane.stride    = yuvPlane.width;
      const int size     = yuvPlane.stride * yuvPlane.height;
      yuvPlane.planeBuf  = ( size > 0 ) ? new int16_t[ size ] : nullptr;
    }
  }

  ~YUVBufferStorage()
  {
    for ( int i = 0; i < MAX_NUM_COMP; i++ )
    {
      YUVPlane& yuvPlane = yuvPlanes[ i ];
      if ( yuvPlane.planeBuf )
        delete [] yuvPlane.planeBuf;
    }
  }
};

// ====================================================================================================================

class EncApp : public vvenc::YUVWriterIf
{
private:
  EncAppCfg    m_cEncAppCfg;                      ///< encoder configuration
  VVEnc        m_cVVEnc;                          ///< encoder library class
  YuvIO        m_yuvInputFile;                    ///< input YUV file
  YuvIO        m_yuvReconFile;                    ///< output YUV reconstruction file
  std::fstream m_bitstream;                       ///< output bitstream file
  unsigned     m_essentialBytes;
  unsigned     m_totalBytes;

public:
  EncApp()
    : m_essentialBytes( 0 )
    , m_totalBytes    ( 0 )
  {
  }

  virtual ~EncApp()
  {
  }

  bool  parseCfg( int argc, char* argv[] );           ///< parse configuration file to fill member variables
  void  encode();                                     ///< main encoding function
  void  outputAU ( const AccessUnit& au );         ///< write encoded access units to bitstream
  void  outputYuv( const YUVBuffer& yuvOutBuf );      ///< write reconstructed yuv output

private:
  // file I/O
  bool openFileIO();
  void closeFileIO();

  // statistics
  void rateStatsAccum  ( const AccessUnit& au );
  void printRateSummary( int framesRcvd );
  void printChromaFormat();
};

//! \}


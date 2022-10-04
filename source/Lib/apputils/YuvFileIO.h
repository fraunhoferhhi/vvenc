/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     YuvFileIO.h
    \brief    yuv file I/O class (header)
*/

#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <regex>

#include "vvenc/vvencCfg.h"
#include "vvenc/vvenc.h"

#include "FileIOHelper.h"
#include "LogoRenderer.h"

#if defined (_WIN32) || defined (WIN32) || defined (_WIN64) || defined (WIN64)
#include <io.h>
#include <fcntl.h>
#endif

//! \ingroup Interface
//! \{

struct vvencYUVBuffer;

namespace apputils {

typedef int16_t LPel;

// ====================================================================================================================

class YuvFileIO
{
private:
  std::string         m_lastError;                              ///< temporal storage for last occured error 
  std::fstream        m_cHandle;                                ///< file handle
  int                 m_fileBitdepth        = 0;                ///< bitdepth of input/output video file
  int                 m_MSBExtendedBitDepth = 0;                ///< bitdepth after addition of MSBs (with value 0)
  int                 m_bitdepthShift       = 0;                ///< number of bits to increase or decrease image by before/after write/read
  vvencChromaFormat   m_fileChrFmt          = VVENC_CHROMA_420; ///< chroma format of the file
  vvencChromaFormat   m_bufferChrFmt        = VVENC_CHROMA_420; ///< chroma format of the buffer
  bool                m_clipToRec709        = false;            ///< clip data according to Recom.709
  bool                m_packedYUVMode       = false;            ///< used packed buffer file format
  bool                m_readStdin           = false;            ///< read input from stdin
  bool                m_y4mMode             = false;            ///< use/force y4m file format
  size_t              m_packetCount         = 0;
  LogoRenderer        m_cLogoRenderer;

public:

  int open( const std::string &fileName, bool bWriteMode, int fileBitDepth, int MSBExtendedBitDepth, int internalBitDepth, 
            vvencChromaFormat fileChrFmt, vvencChromaFormat bufferChrFmt, bool clipToRec709, bool packedYUVMode, bool y4mMode,
            std::string cLogoFilename = "" )
  {
    //NOTE: files cannot have bit depth greater than 16
    m_fileBitdepth        = std::min<unsigned>( fileBitDepth, 16 );
    m_MSBExtendedBitDepth = MSBExtendedBitDepth;
    m_bitdepthShift       = internalBitDepth - m_MSBExtendedBitDepth;
    m_fileChrFmt          = fileChrFmt;
    m_bufferChrFmt        = bufferChrFmt;
    m_clipToRec709        = clipToRec709;
    m_packedYUVMode       = packedYUVMode;
    m_readStdin           = false;
    m_y4mMode             = y4mMode;
    m_packetCount         = 0;

    if( m_packedYUVMode && !bWriteMode && m_fileBitdepth != 10 )
    {
      m_lastError = "\nERROR: file bitdepth for packed yuv input must be 10";
      return -1;
    }

    if ( m_fileBitdepth > 16 )
    {
      m_lastError =  "\nERROR: Cannot handle a yuv file of bit depth greater than 16";
      return -1;
    }

    if ( m_y4mMode && bWriteMode )
    {
      m_lastError =  "\nERROR: Cannot handle y4m yuv output (only support for y4m input)";
      return -1;
    }

    if( bWriteMode )
    {
      m_cHandle.open( fileName.c_str(), std::ios::binary | std::ios::out );

      if( m_cHandle.fail() )
      {
        m_lastError =  "\nFailed to open output YUV file:  " + fileName;
        return -1;
      }
    }
    else
    {
      if( !strcmp( fileName.c_str(), "-" ) )
      {
        m_readStdin = true;
  #if defined (_WIN32) || defined (WIN32) || defined (_WIN64) || defined (WIN64)
        if( _setmode( _fileno( stdin ), _O_BINARY ) == -1 )
        {
          m_lastError =  "\nError: Failed to set stdin to binary mode";
          return -1;
        }
  #endif
        return 0;
      }

      m_cHandle.open( fileName.c_str(), std::ios::binary | std::ios::in );

      if( m_cHandle.fail() )
      {
        m_lastError =  "\nFailed to open input YUV file:  " + fileName;
        return -1;
      }

      if ( m_y4mMode || FileIOHelper::isY4mInputFilename( fileName ) )
      {
        std::istream& inStream = m_cHandle;
        std::string headerline;
        getline(inStream, headerline);  // jump over y4m header
        m_y4mMode   = true;
      }

      if( !cLogoFilename.empty() )
      {
        std::stringstream strstr;
        if ( 0 != m_cLogoRenderer.init( cLogoFilename, m_bufferChrFmt, internalBitDepth, strstr ) )
        {
          if( !strstr.str().empty() )
            m_lastError = strstr.str();
          else
            m_lastError = "failed to open Logo overlay renderer";
          return -1;
        }
      }
    }
    return 0;
  }

  void close()
  {
    if( !m_readStdin )
      m_cHandle.close();

    if( m_cLogoRenderer.isInitialized() )
    {
      m_cLogoRenderer.uninit();
    }
  }

  bool  isOpen()  { return m_cHandle.is_open(); }
  bool  isEof()   { return m_cHandle.eof();     }
  bool  isFail()  { return m_cHandle.fail();    }
  std::string getLastError() const { return m_lastError; }   

  int   skipYuvFrames ( int numFrames, int width, int height )
  {
    if ( numFrames <= 0 )
    {
      return -1;
    }

    //set the frame size according to the chroma format
    std::streamoff frameSize      = 0;
    const int numComp = (m_fileChrFmt==VVENC_CHROMA_400) ? 1 : 3;

    if( m_packedYUVMode)
    {
      for ( int i = 0; i < numComp; i++ )
      {
        const int csx_file = ( (i == 0) || (m_fileChrFmt==VVENC_CHROMA_444) ) ? 0 : 1;
        const int csy_file = ( (i == 0) || (m_fileChrFmt!=VVENC_CHROMA_420) ) ? 0 : 1;
        frameSize += (( ( width * 5 / 4 ) >> csx_file) * (height >> csy_file));
      }
    }
    else
    {
      unsigned wordsize             = ( m_fileBitdepth > 8 ) ? 2 : 1;
      for ( int i = 0; i < numComp; i++ )
      {
        const int csx_file = ( (i == 0) || (m_fileChrFmt==VVENC_CHROMA_444) ) ? 0 : 1;
        const int csy_file = ( (i == 0) || (m_fileChrFmt!=VVENC_CHROMA_420) ) ? 0 : 1;
        frameSize += ( width >> csx_file ) * ( height >> csy_file );
      }
      frameSize *= wordsize;
    }

    if( m_y4mMode )
    {
      const char Y4MHeader[] = {'F','R','A','M','E'};
      frameSize += (sizeof(Y4MHeader) + 1);  /* assume basic FRAME\n headers */;
    }

    const std::streamoff offset = frameSize * numFrames;
    
    std::istream& inStream = m_readStdin ? std::cin : m_cHandle;
    
    // check for file size
    if( !m_readStdin )
    {
      std::streamoff fsize = m_cHandle.tellg();
      m_cHandle.seekg( 0, std::ios::end );
      std::streamoff filelength = m_cHandle.tellg() - fsize;
      m_cHandle.seekg( fsize, std::ios::beg );
      if( offset >= filelength )
      {
        return -1;
      }
    } 

    // attempt to seek
    if ( !! inStream.seekg( offset, std::ios::cur ) )
    {
      return 0; /* success */
    }

    inStream.clear();

    // fall back to consuming the input
    char buf[ 512 ];
    const std::streamoff offset_mod_bufsize = offset % sizeof( buf );
    for ( std::streamoff i = 0; i < offset - offset_mod_bufsize; i += sizeof( buf ) )
    {
      inStream.read( buf, sizeof( buf ) );
    }
    inStream.read( buf, offset_mod_bufsize );
    
    return 0;
  }


  int readYuvBuf ( vvencYUVBuffer& yuvInBuf, bool& eof )
  {
    eof = false;
    // check end-of-file
    if ( isEof() )
    {
      m_lastError = "end of file";
      eof = true;
      return 0;
    }

    if ( m_packedYUVMode &&  ( 0 != (yuvInBuf.planes[0].width >> 1) % 4 ) )
    {
      m_lastError = "unsupported file width for packed input";
      return -1;
    }

    const bool monochromFix    = ( m_bufferChrFmt==VVENC_CHROMA_400 && m_fileChrFmt!=VVENC_CHROMA_400 ); 
    const bool is16bit         = m_fileBitdepth > 8;
    const int desired_bitdepth = m_MSBExtendedBitDepth + m_bitdepthShift;
    const bool b709Compliance  = ( m_clipToRec709 ) && ( m_bitdepthShift < 0 && desired_bitdepth >= 8 );     /* ITU-R BT.709 compliant clipping for converting say 10b to 8b */
    const LPel minVal           = b709Compliance ? ( (    1 << ( desired_bitdepth - 8 ) )    ) : 0;
    const LPel maxVal           = b709Compliance ? ( ( 0xff << ( desired_bitdepth - 8 ) ) -1 ) : ( 1 << desired_bitdepth ) - 1;
    const int numComp                = (m_fileChrFmt==VVENC_CHROMA_400) ? 1 : 3;

    for( int comp = 0; comp < numComp; comp++ )
    {
      vvencYUVPlane yuvPlane = yuvInBuf.planes[ comp ];

      if( monochromFix && comp )
      {
        yuvPlane.width  = yuvInBuf.planes[0].width  >> (m_fileChrFmt == VVENC_CHROMA_444 ? 0 : 1);
        yuvPlane.height = 2*yuvInBuf.planes[0].height >> (m_fileChrFmt != VVENC_CHROMA_420 ? 0 : 1);
        yuvPlane.stride = yuvPlane.width;
      }

      std::istream& inStream = m_readStdin ? std::cin : m_cHandle;

      if( m_y4mMode && comp == 0 )
      {
        std::string y4mPrefix;
        getline(inStream, y4mPrefix);   /* assume basic FRAME\n headers */
        if( y4mPrefix != "FRAME")
        {
          m_lastError = "Source image does not contain valid y4m header (FRAME) - end of stream";
          eof = true;
          return ( m_packetCount ? 0 : -1); // return error if no frames has been proceeded, otherwise expect eof
        }
      }

      if ( ! FileIOHelper::readYuvPlane( inStream, yuvPlane, is16bit, m_fileBitdepth, m_packedYUVMode, comp, m_fileChrFmt, m_bufferChrFmt ) )
      {
        eof = true;
        return 0;
      }

      if ( m_bufferChrFmt == VVENC_CHROMA_400 && comp)
        continue;

      if ( ! FileIOHelper::verifyYuvPlane( yuvPlane, m_fileBitdepth ) )
      {
        eof = true;
        m_lastError = "Source image contains values outside the specified bit range!";
        return -1;
      }

      FileIOHelper::scaleYuvPlane( yuvPlane, yuvPlane, m_bitdepthShift, minVal, maxVal );
    }
    
    if( m_cLogoRenderer.isInitialized() )
    {
      LogoInputOptions cLogo = m_cLogoRenderer.getLogoInputOptions();
      if ( cLogo.sourceWidth > yuvInBuf.planes[0].width || cLogo.sourceHeight > yuvInBuf.planes[0].height )
      {
        std::stringstream css;
        css << "input picture size (" << yuvInBuf.planes[0].width << "x" << yuvInBuf.planes[0].height << ") < logo size (" << 
                  cLogo.sourceWidth  << "x" << cLogo.sourceHeight << ") cannot render logo" << std::endl;     
        m_lastError = css.str();   
        return -1;
      }

      if( 0 != m_cLogoRenderer.renderLogo( yuvInBuf ) )
      {
        m_lastError = "failed to render Logo";
        return -1;
      }
    }
    
    m_packetCount++;

    return 0;
  }


  bool writeYuvBuf ( const vvencYUVBuffer& yuvOutBuf )
  {
    // compute actual YUV frame size excluding padding size
    bool is16bit              = m_fileBitdepth > 8;
    bool nonZeroBitDepthShift = m_bitdepthShift != 0;

    vvencYUVBuffer yuvScaled;
    vvenc_YUVBuffer_default( &yuvScaled );
    vvenc_YUVBuffer_alloc_buffer( &yuvScaled, m_bufferChrFmt, yuvOutBuf.planes[ 0 ].width, yuvOutBuf.planes[ 0 ].height );

    if ( nonZeroBitDepthShift )
    {
      const bool b709Compliance = m_clipToRec709 && ( -m_bitdepthShift < 0 && m_MSBExtendedBitDepth >= 8 );     /* ITU-R BT.709 compliant clipping for converting say 10b to 8b */
      const LPel minVal          = b709Compliance? ( (    1 << ( m_MSBExtendedBitDepth - 8 ) )    ) : 0;
      const LPel maxVal          = b709Compliance? ( ( 0xff << ( m_MSBExtendedBitDepth - 8 ) ) -1 ) : ( 1 << m_MSBExtendedBitDepth ) - 1;
      const int numComp          = (m_bufferChrFmt==VVENC_CHROMA_400) ? 1 : 3;

      for( int comp = 0; comp < numComp; comp++ )
      {
        FileIOHelper::scaleYuvPlane( yuvScaled.planes[ comp ], yuvOutBuf.planes[ comp ],-m_bitdepthShift, minVal, maxVal );
      }
    }

    const vvencYUVBuffer& yuvWriteBuf = nonZeroBitDepthShift ? yuvScaled : yuvOutBuf;

    const int numComp = (m_fileChrFmt==VVENC_CHROMA_400) ? 1 : 3;
    for( int comp = 0; comp < numComp; comp++ )
    {
      if ( ! FileIOHelper::writeYuvPlane( m_cHandle, yuvWriteBuf.planes[ comp ], is16bit, m_fileBitdepth, m_packedYUVMode, comp, m_bufferChrFmt, m_fileChrFmt ) )
        return false;
    }
    
    m_packetCount++;
    vvenc_YUVBuffer_free_buffer( &yuvScaled );

    return true;
  }


  int countYuvFrames( int width, int height, bool countFromStart = true )
  {
    if( m_readStdin ) return -1;
    
    //set the frame size according to the chroma format
    std::streamoff frameSize      = 0;
    const int numComp = (m_fileChrFmt==VVENC_CHROMA_400) ? 1 : 3;

    if( m_packedYUVMode)
    {
      for ( int i = 0; i < numComp; i++ )
      {
        const int csx_file = ( (i == 0) || (m_fileChrFmt==VVENC_CHROMA_444) ) ? 0 : 1;
        const int csy_file = ( (i == 0) || (m_fileChrFmt!=VVENC_CHROMA_420) ) ? 0 : 1;
        frameSize += (( ( width * 5 / 4 ) >> csx_file) * (height >> csy_file));
      }
    }
    else
    {
      unsigned wordsize             = ( m_fileBitdepth > 8 ) ? 2 : 1;
      for ( int i = 0; i < numComp; i++ )
      {
        const int csx_file = ( (i == 0) || (m_fileChrFmt==VVENC_CHROMA_444) ) ? 0 : 1;
        const int csy_file = ( (i == 0) || (m_fileChrFmt!=VVENC_CHROMA_420) ) ? 0 : 1;
        frameSize += ( width >> csx_file ) * ( height >> csy_file );
      }
      frameSize *= wordsize;
    }

    if( m_y4mMode )
    {
      const char Y4MHeader[] = {'F','R','A','M','E'};
      frameSize += (sizeof(Y4MHeader) + 1);  /* assume basic FRAME\n headers */;
    }
    
    std::streamoff lastPos = m_cHandle.tellg();  // backup last position
    
    if( countFromStart )
    {
      m_cHandle.seekg( 0, std::ios::beg );
    }
    std::streamoff curPos = m_cHandle.tellg();
      
    m_cHandle.seekg( 0, std::ios::end );
    std::streamoff filelength = m_cHandle.tellg() - curPos;
    
    m_cHandle.seekg( lastPos, std::ios::beg ); // rewind to last pos
    
    return filelength / frameSize;
  }
};

} // namespace apputils

//! \}


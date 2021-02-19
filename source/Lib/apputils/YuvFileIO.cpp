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


/** \file     YuvFileIO.cpp
    \brief    yuv file I/O class (header)
*/

#include "apputils/YuvFileIO.h"
#include "vvenc/vvenc.h"

#include <algorithm>
#include <iostream>

//! \ingroup Interface
//! \{
using namespace vvenc;

namespace apputils {

// ====================================================================================================================
typedef int16_t LPel;

bool readYuvPlane( std::istream&       fd,
                   YUVBuffer::Plane&   yuvPlane,
                   bool                is16bit,
                   int                 fileBitDepth,
                   const ComponentID&  compID,
                   const ChromaFormat& inputChFmt,
                   const ChromaFormat& internChFmt
                 )
{
  const int csx_file = getComponentScaleX( compID, inputChFmt );
  const int csy_file = getComponentScaleY( compID, inputChFmt );
  const int csx_dest = getComponentScaleX( compID, internChFmt );
  const int csy_dest = getComponentScaleY( compID, internChFmt );

  const int stride  = yuvPlane.stride;
  const int width   = yuvPlane.width;
  const int height  = yuvPlane.height;
  LPel* dst         = yuvPlane.ptr;

  const int fileStride = ( ( width  << csx_dest ) * ( is16bit ? 2 : 1 ) ) >> csx_file;
  const int fileHeight = ( ( height << csy_dest )                       ) >> csy_file;

  std::vector<uint8_t> bufVec( fileStride );
  uint8_t *buf = &( bufVec[0] );

  if ( compID != COMP_Y && ( inputChFmt == CHROMA_400 || internChFmt == CHROMA_400 ) )
  {
    if ( internChFmt != CHROMA_400 )
    {
      // set chrominance data to mid-range: (1<<(fileBitDepth-1))
      const LPel val = 1 << ( fileBitDepth - 1 );
      for (int y = 0; y < height; y++, dst+= stride)
      {
        for (int x = 0; x < width; x++)
        {
          dst[x] = val;
        }
      }
    }

    if ( inputChFmt != CHROMA_400 )
    {
      fd.seekg( fileHeight * fileStride, std::ios::cur );
      if ( fd.eof() || fd.fail() )
      {
        return false;
      }
    }
  }
  else
  {
    const unsigned mask_y_file = ( 1 << csy_file ) - 1;
    const unsigned mask_y_dest = ( 1 << csy_dest ) - 1;
    for( int y444 = 0; y444 < ( height << csy_dest ); y444++ )
    {
      if ( ( y444 & mask_y_file ) == 0 )
      {
        // read a new line
        fd.read( reinterpret_cast<char*>( buf ), fileStride );
        if ( fd.eof() || fd.fail() )
        {
          return false;
        }
      }

      if ( ( y444 & mask_y_dest ) == 0 )
      {
        // process current destination line
        if ( csx_file < csx_dest )
        {
          // eg file is 444, dest is 422.
          const int sx = csx_dest - csx_file;
          if ( ! is16bit )
          {
            for ( int x = 0; x < width; x++ )
            {
              dst[ x ] = buf[ x << sx ];
            }
          }
          else
          {
            for ( int x = 0; x < width; x++ )
            {
              dst[ x ] = LPel( buf[ ( x << sx ) * 2 + 0 ] ) | ( LPel ( buf[ ( x << sx ) * 2 + 1 ] ) << 8 );
            }
          }
        }
        else
        {
          // eg file is 422, dest is 444.
          const int sx = csx_file - csx_dest;
          if ( ! is16bit )
          {
            for ( int x = 0; x < width; x++ )
            {
              dst[ x ] = buf[ x >> sx ];
            }
          }
          else
          {
            for ( int x = 0; x < width; x++ )
            {
              dst[ x ] = LPel( buf[ ( x >> sx ) * 2 + 0 ] ) | ( LPel( buf[ ( x >> sx ) * 2 + 1 ] ) << 8 );
            }
          }
        }

        dst += stride;
      }
    }

  }
  return true;
}

bool writeYuvPlane( std::ostream&           fd,
                    const YUVBuffer::Plane& yuvPlane,
                    bool                    is16bit,
                    int                     fileBitDepth,
                    int                     packedYUVOutputMode,
                    const ComponentID&      compID,
                    const ChromaFormat&     internChFmt,
                    const ChromaFormat&     outputChFmt
                  )
{
  const int stride = yuvPlane.stride;
  const int width  = yuvPlane.width;
  const int height = yuvPlane.height;
  const LPel* src  = yuvPlane.ptr;

  const uint32_t csx_file = getComponentScaleX( compID, outputChFmt );
  const uint32_t csy_file = getComponentScaleY( compID, outputChFmt );
  const uint32_t csx_src  = getComponentScaleX( compID, internChFmt  );
  const uint32_t csy_src  = getComponentScaleY( compID, internChFmt  );

  const int  fileWidth  = ( width  << csx_src ) >> csx_file;
  const int  fileHeight = ( height << csy_src ) >> csy_file;
  const bool writePYUV  = ( packedYUVOutputMode > 0 ) && ( fileBitDepth == 10 || fileBitDepth == 12 ) && ( ( fileWidth & ( 1 + ( fileBitDepth & 3 ) ) ) == 0 );
  const int  fileStride = writePYUV ? ( ( width << csx_src ) * fileBitDepth ) >> ( csx_file + 3 ) : ( ( width << csx_src ) * ( is16bit ? 2 : 1 ) ) >> csx_file;

  std::vector<uint8_t> bufVec( fileStride );
  uint8_t *buf = &( bufVec[ 0 ] );

  if ( writePYUV )
  {
    const unsigned mask_y_file = (1 << csy_file) - 1;
    const unsigned mask_y_src  = (1 << csy_src ) - 1;
    const int      widthS_file = fileWidth >> (fileBitDepth == 12 ? 1 : 2);

    for ( int y444 = 0; y444 < ( height << csy_src ); y444++)
    {
      if ( ( y444 & mask_y_file ) == 0 )  // write a new line to file
      {
        if ( csx_file < csx_src )
        {
          // eg file is 444, source is 422.
          const int sx = csx_src - csx_file;

          if (fileBitDepth == 10)  // write 4 values into 5 bytes
          {
            for ( int x = 0; x < widthS_file; x++)
            {
              const unsigned src0 = src[(4*x  ) >> sx];
              const unsigned src1 = src[(4*x+1) >> sx];
              const unsigned src2 = src[(4*x+2) >> sx];
              const unsigned src3 = src[(4*x+3) >> sx];

              buf[5*x  ] = ((src0     ) & 0xff); // src0:76543210
              buf[5*x+1] = ((src1 << 2) & 0xfc) + ((src0 >> 8) & 0x03);
              buf[5*x+2] = ((src2 << 4) & 0xf0) + ((src1 >> 6) & 0x0f);
              buf[5*x+3] = ((src3 << 6) & 0xc0) + ((src2 >> 4) & 0x3f);
              buf[5*x+4] = ((src3 >> 2) & 0xff); // src3:98765432
            }
          }
          else if ( fileBitDepth == 12 ) //...2 values into 3 bytes
          {
            for ( int x = 0; x < widthS_file; x++ )
            {
              const unsigned src0 = src[(2*x  ) >> sx];
              const unsigned src1 = src[(2*x+1) >> sx];

              buf[3*x  ] = ((src0     ) & 0xff); // src0:76543210
              buf[3*x+1] = ((src1 << 4) & 0xf0) + ((src0 >> 8) & 0x0f);
              buf[3*x+2] = ((src1 >> 4) & 0xff); // src1:BA987654
            }
          }
        }
        else
        {
          // eg file is 422, source is 444.
          const int sx = csx_file - csx_src;

          if (fileBitDepth == 10)  // write 4 values into 5 bytes
          {
            for ( int x = 0; x < widthS_file; x++ )
            {
              const unsigned src0 = src[(4*x  ) << sx];
              const unsigned src1 = src[(4*x+1) << sx];
              const unsigned src2 = src[(4*x+2) << sx];
              const unsigned src3 = src[(4*x+3) << sx];

              buf[5*x  ] = ((src0     ) & 0xff); // src0:76543210
              buf[5*x+1] = ((src1 << 2) & 0xfc) + ((src0 >> 8) & 0x03);
              buf[5*x+2] = ((src2 << 4) & 0xf0) + ((src1 >> 6) & 0x0f);
              buf[5*x+3] = ((src3 << 6) & 0xc0) + ((src2 >> 4) & 0x3f);
              buf[5*x+4] = ((src3 >> 2) & 0xff); // src3:98765432
            }
          }
          else if ( fileBitDepth == 12 ) //...2 values into 3 bytes
          {
            for ( int x = 0; x < widthS_file; x++ )
            {
              const unsigned src0 = src[(2*x  ) << sx];
              const unsigned src1 = src[(2*x+1) << sx];

              buf[3*x  ] = ((src0     ) & 0xff); // src0:76543210
              buf[3*x+1] = ((src1 << 4) & 0xf0) + ((src0 >> 8) & 0x0f);
              buf[3*x+2] = ((src1 >> 4) & 0xff); // src1:BA987654
            }
          }
        }

        fd.write( reinterpret_cast<const char*>( buf ), fileStride );
        if ( fd.eof() || fd.fail() )
        {
          return false;
        }
      }

      if ( ( y444 & mask_y_src ) == 0)
      {
        src += stride;
      }
    }
  }
  // !writePYUV
  else if ( compID != COMP_Y && ( outputChFmt == CHROMA_400 || internChFmt == CHROMA_400 ) )
  {
    if ( outputChFmt != CHROMA_400 )
    {
      const LPel value = 1 << ( fileBitDepth - 1 );

      for ( int y = 0; y < fileHeight; y++ )
      {
        if ( ! is16bit )
        {
          uint8_t val( value );
          for ( int x = 0; x < fileWidth; x++ )
          {
            buf[x]=val;
          }
        }
        else
        {
          uint16_t val( value );
          for ( int x = 0; x < fileWidth; x++ )
          {
            buf[2*x  ]= (val>>0) & 0xff;
            buf[2*x+1]= (val>>8) & 0xff;
          }
        }

        fd.write( reinterpret_cast<const char*>( buf ), fileStride );
        if ( fd.eof() || fd.fail() )
        {
          return false;
        }
      }
    }
  }
  else
  {
    const unsigned mask_y_file = (1 << csy_file) - 1;
    const unsigned mask_y_src  = (1 << csy_src ) - 1;

    for ( int y444 = 0; y444 < ( height << csy_src ); y444++ )
    {
      if ( ( y444 & mask_y_file ) == 0 )
      {
        // write a new line
        if ( csx_file < csx_src )
        {
          // eg file is 444, source is 422.
          const int sx = csx_src - csx_file;
          if ( ! is16bit )
          {
            for ( int x = 0; x < fileWidth; x++ )
            {
              buf[x] = (uint8_t)(src[x>>sx]);
            }
          }
          else
          {
            for ( int x = 0; x < fileWidth; x++ )
            {
              buf[2*x  ] = (src[x>>sx]>>0) & 0xff;
              buf[2*x+1] = (src[x>>sx]>>8) & 0xff;
            }
          }
        }
        else
        {
          // eg file is 422, source is 444.
          const int sx = csx_file - csx_src;
          if ( ! is16bit )
          {
            for ( int x = 0; x < fileWidth; x++ )
            {
              buf[x] = (uint8_t)(src[x<<sx]);
            }
          }
          else
          {
            for ( int x = 0; x < fileWidth; x++ )
            {
              buf[2*x  ] = (src[x<<sx]>>0) & 0xff;
              buf[2*x+1] = (src[x<<sx]>>8) & 0xff;
            }
          }
        }

        fd.write( reinterpret_cast<const char*>( buf ), fileStride );
        if ( fd.eof() || fd.fail() )
        {
          return false;
        }
      }

      if ( ( y444 & mask_y_src ) == 0 )
      {
        src += stride;
      }
    }
  }
  return true;
}

bool verifyYuvPlane( YUVBuffer::Plane& yuvPlane, const int bitDepth )
{
  const int stride = yuvPlane.stride;
  const int width  = yuvPlane.width;
  const int height = yuvPlane.height;
  LPel* dst        = yuvPlane.ptr;

  const LPel mask = ~( ( 1 << bitDepth ) - 1 );

  for ( int y = 0; y < height; y++, dst += stride )
  {
    for ( int x = 0; x < width; x++ )
    {
      if ( ( dst[ x ] & mask ) != 0 )
      {
        return false;
      }
    }
  }

  return true;
}

void scaleYuvPlane( YUVBuffer::Plane& yuvPlaneOut, const YUVBuffer::Plane& yuvPlaneIn, const int shiftBits, const LPel minVal, const LPel maxVal )
{
  const int stride = yuvPlaneOut.stride;
  const int width  = yuvPlaneOut.width;
  const int height = yuvPlaneOut.height;
  LPel* dst        = yuvPlaneOut.ptr;
  const LPel* src    = yuvPlaneIn.ptr;
  const int instride = yuvPlaneIn.stride;

  if( 0 == shiftBits )
  {
    return;
  }

  if( shiftBits > 0 )
  {
    for( int y = 0; y < height; y++, dst += stride, src += instride )
    {
      for( int x = 0; x < width; x++)
      {
        dst[x] = src[x] << shiftBits;
      }
    }
  }
  else if ( shiftBits < 0 )
  {
    const int shiftbitsr =- shiftBits;
    const LPel rounding = 1 << ( shiftbitsr-1 );

    for( int y = 0; y < height; y++, dst += stride, src += instride )
    {
      for( int x = 0; x < width; x++)
      {
        LPel val = ( src[ x ] + rounding ) >> shiftbitsr;
        dst[ x ] = std::min<LPel>( minVal, std::max<LPel>( maxVal, val ) );
      }
    }
  }
}

// ====================================================================================================================

int YuvFileIO::open( const std::string &fileName, bool bWriteMode, const int fileBitDepth, const int MSBExtendedBitDepth,
                     const int internalBitDepth, ChromaFormat fileChrFmt, ChromaFormat bufferChrFmt,
                     bool clipToRec709, bool packedYUVMode )
{
  //NOTE: files cannot have bit depth greater than 16
  m_fileBitdepth        = std::min<unsigned>( fileBitDepth, 16 );
  m_MSBExtendedBitDepth = MSBExtendedBitDepth;
  m_bitdepthShift       = internalBitDepth - m_MSBExtendedBitDepth;
  m_fileChrFmt          = fileChrFmt; 
  m_bufferChrFmt        = bufferChrFmt; 
  m_clipToRec709        = clipToRec709;
  m_packedYUVMode       = packedYUVMode;

  if( m_packedYUVMode && (m_bufferChrFmt == CHROMA_400) )
  {  
    m_lastError = "\nERROR: write packed yuv for chroma 400 not supported";
    return -1;
  }

  if( m_packedYUVMode &&  !bWriteMode )
  {
    m_lastError = "\nERROR: yuv file input - no packed mode support";
    return -1;
  }

  if ( m_fileBitdepth > 16 )
  {
    m_lastError =  "\nERROR: Cannot handle a yuv file of bit depth greater than 16";
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
    m_cHandle.open( fileName.c_str(), std::ios::binary | std::ios::in );

    if( m_cHandle.fail() )
    {
      m_lastError =  "\nFailed to open input YUV file:  " + fileName;
      return -1;
    }
  }
  return 0;
}

void YuvFileIO::close()
{
  m_cHandle.close();
}

bool YuvFileIO::isEof()
{
  return m_cHandle.eof();
}

bool YuvFileIO::isFail()
{
  return m_cHandle.fail();
}

void YuvFileIO::skipYuvFrames( int numFrames, int width, int height  )
{
  if ( numFrames <= 0 )
  {
    return;
  }

  //set the frame size according to the chroma format
  std::streamoff frameSize      = 0;
  unsigned wordsize             = ( m_fileBitdepth > 8 ) ? 2 : 1;
  for ( int i = 0; i < (int)getNumberValidComponents( m_fileChrFmt ); i++ )
  {
    const ComponentID compID = ComponentID( i );
    frameSize += ( width >> getComponentScaleX( compID, m_fileChrFmt ) ) * ( height >> getComponentScaleY( compID, m_fileChrFmt ));
  }
  frameSize *= wordsize;

  const std::streamoff offset = frameSize * numFrames;

  // attempt to seek
  if ( !! m_cHandle.seekg( offset, std::ios::cur ) )
  {
    return; /* success */
  }

  m_cHandle.clear();

  // fall back to consuming the input
  char buf[ 512 ];
  const std::streamoff offset_mod_bufsize = offset % sizeof( buf );
  for ( std::streamoff i = 0; i < offset - offset_mod_bufsize; i += sizeof( buf ) )
  {
    m_cHandle.read( buf, sizeof( buf ) );
  }
  m_cHandle.read( buf, offset_mod_bufsize );
}

bool YuvFileIO::readYuvBuf( YUVBuffer& yuvInBuf )
{
  // check end-of-file
  if ( isEof() )
  {
    return false;
  }

  const bool is16bit         = m_fileBitdepth > 8;
  const int desired_bitdepth = m_MSBExtendedBitDepth + m_bitdepthShift;
  const bool b709Compliance  = ( m_clipToRec709 ) && ( m_bitdepthShift < 0 && desired_bitdepth >= 8 );     /* ITU-R BT.709 compliant clipping for converting say 10b to 8b */
  const LPel minVal           = b709Compliance ? ( (    1 << ( desired_bitdepth - 8 ) )    ) : 0;
  const LPel maxVal           = b709Compliance ? ( ( 0xff << ( desired_bitdepth - 8 ) ) -1 ) : ( 1 << desired_bitdepth ) - 1;

  const int numComp          = std::max( getNumberValidComponents( m_fileChrFmt ), getNumberValidComponents( m_fileChrFmt ) );
  for( int comp = 0; comp < numComp; comp++ )
  {
    YUVBuffer::Plane& yuvPlane = yuvInBuf.planes[ comp ];

    if ( ! readYuvPlane( m_cHandle, yuvPlane, is16bit, m_fileBitdepth, ComponentID( comp ), m_fileChrFmt, m_bufferChrFmt ) )
      return false;

    if ( m_bufferChrFmt == CHROMA_400 )
      continue;

    if ( ! verifyYuvPlane( yuvPlane, m_fileBitdepth ) )
    {
      m_lastError = "Source image contains values outside the specified bit range!";
      return false;
    }

    scaleYuvPlane( yuvPlane, yuvPlane, m_bitdepthShift, minVal, maxVal );
  }

  return true;
}

bool YuvFileIO::writeYuvBuf( const YUVBuffer& yuvOutBuf )
{
  // compute actual YUV frame size excluding padding size
  bool is16bit              = m_fileBitdepth > 8;
  bool nonZeroBitDepthShift = m_bitdepthShift != 0;

  YUVBufferStorage yuvScaled( m_bufferChrFmt, yuvOutBuf.planes[ COMP_Y ].width, yuvOutBuf.planes[ COMP_Y ].height );

  if ( nonZeroBitDepthShift )
  {
    const bool b709Compliance = m_clipToRec709 && ( -m_bitdepthShift < 0 && m_MSBExtendedBitDepth >= 8 );     /* ITU-R BT.709 compliant clipping for converting say 10b to 8b */
    const LPel minVal          = b709Compliance? ( (    1 << ( m_MSBExtendedBitDepth - 8 ) )    ) : 0;
    const LPel maxVal          = b709Compliance? ( ( 0xff << ( m_MSBExtendedBitDepth - 8 ) ) -1 ) : ( 1 << m_MSBExtendedBitDepth ) - 1;
    for( int comp = 0; comp < (int)getNumberValidComponents( m_bufferChrFmt ); comp++ )
    {
      scaleYuvPlane( yuvScaled.planes[ comp ], yuvOutBuf.planes[ comp ],-m_bitdepthShift, minVal, maxVal );
    }
  }

  const YUVBuffer& yuvWriteBuf = nonZeroBitDepthShift ? yuvScaled : yuvOutBuf;

  for( int comp = 0; comp < (int)getNumberValidComponents( m_fileChrFmt ); comp++ )
  {
    if ( ! writeYuvPlane( m_cHandle, yuvWriteBuf.planes[ comp ], is16bit, m_fileBitdepth, m_packedYUVMode, ComponentID( comp ), m_bufferChrFmt, m_fileChrFmt ) )
      return false;
  }

  return true;
}


} // namespace apputils

//! \}


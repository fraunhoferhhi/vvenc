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


/** \file     FileIO.cpp
    \brief    file I/O class (header)
*/

#include "vvenc/FileIO.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"
#include "CommonLib/Slice.h"
#include "EncoderLib/NALwrite.h"

#include <iostream>

//! \ingroup Interface
//! \{

namespace vvenc {

// ====================================================================================================================

static_assert( sizeof(Pel)  == sizeof(*(YUVPlane::planeBuf)),   "internal bits per pel differ from interface definition" );

// ====================================================================================================================

bool readYuvPlane( std::istream&       fd,
                   YUVPlane&           yuvPlane,
                   bool                is16bit,
                   int                 fileBitDepth,
                   const int           pad[ 2 ],
                   const ComponentID&  compID,
                   const ChromaFormat& inputChFmt,
                   const ChromaFormat& internChFmt
                 )
{
  const int csx_file = getComponentScaleX( compID, inputChFmt );
  const int csy_file = getComponentScaleY( compID, inputChFmt );
  const int csx_dest = getComponentScaleX( compID, internChFmt );
  const int csy_dest = getComponentScaleY( compID, internChFmt );

  const int stride      = yuvPlane.stride;
  const int fullWidth   = yuvPlane.width;
  const int fullHeight  = yuvPlane.height;
  const int inputWidth  = ( ( fullWidth  << csx_dest ) - pad[ 0 ] ) >> csx_dest;
  const int inputHeight = ( ( fullHeight << csy_dest ) - pad[ 1 ] ) >> csy_dest;
  Pel* dst              = yuvPlane.planeBuf;

  const int fileStride = ( ( inputWidth  << csx_dest ) * ( is16bit ? 2 : 1 ) ) >> csx_file;
  const int fileHeight = ( ( inputHeight << csy_dest )                       ) >> csy_file;

  std::vector<uint8_t> bufVec( fileStride );
  uint8_t *buf = &( bufVec[0] );

  if ( compID != COMP_Y && ( inputChFmt == CHROMA_400 || internChFmt == CHROMA_400 ) )
  {
    if ( internChFmt != CHROMA_400 )
    {
      // set chrominance data to mid-range: (1<<(fileBitDepth-1))
      const Pel val = 1 << ( fileBitDepth - 1 );
      for (int y = 0; y < fullHeight; y++, dst+= stride)
      {
        for (int x = 0; x < fullWidth; x++)
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
    for( int y444 = 0; y444 < ( inputHeight << csy_dest ); y444++ )
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
            for ( int x = 0; x < inputWidth; x++ )
            {
              dst[ x ] = buf[ x << sx ];
            }
          }
          else
          {
            for ( int x = 0; x < inputWidth; x++ )
            {
              dst[ x ] = Pel( buf[ ( x << sx ) * 2 + 0 ] ) | ( Pel ( buf[ ( x << sx ) * 2 + 1 ] ) << 8 );
            }
          }
        }
        else
        {
          // eg file is 422, dest is 444.
          const int sx = csx_file - csx_dest;
          if ( ! is16bit )
          {
            for ( int x = 0; x < inputWidth; x++ )
            {
              dst[ x ] = buf[ x >> sx ];
            }
          }
          else
          {
            for ( int x = 0; x < inputWidth; x++ )
            {
              dst[ x ] = Pel( buf[ ( x >> sx ) * 2 + 0 ] ) | ( Pel( buf[ ( x >> sx ) * 2 + 1 ] ) << 8 );
            }
          }
        }

        // process right hand side padding
        const Pel val = dst[ inputWidth - 1 ];
        for ( int x = inputWidth; x < fullWidth; x++ )
        {
          dst[ x ] = val;
        }

        dst += stride;
      }
    }

    // process bottom padding
    for ( int y = inputHeight; y < fullHeight; y++, dst += stride )
    {
      for ( int x = 0; x < fullWidth; x++ )
      {
        dst[ x ] = (dst - stride)[ x ];
      }
    }
  }
  return true;
}

bool writeYuvPlane( std::ostream&       fd,
                    const YUVPlane&     yuvPlane,
                    bool                is16bit,
                    int                 fileBitDepth,
                    int                 packedYUVOutputMode,
                    const ComponentID&  compID,
                    const ChromaFormat& internChFmt,
                    const ChromaFormat& outputChFmt
                  )
{
  const int stride = yuvPlane.stride;
  const int width  = yuvPlane.width;
  const int height = yuvPlane.height;
  const Pel* src   = yuvPlane.planeBuf;

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
    CHECK( internChFmt == CHROMA_400, "write packed yuv for chroma 400 not supported" );

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
      const Pel value = 1 << ( fileBitDepth - 1 );

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

bool verifyYuvPlane( YUVPlane& yuvPlane, const int bitDepth )
{
  const int stride = yuvPlane.stride;
  const int width  = yuvPlane.width;
  const int height = yuvPlane.height;
  Pel* dst         = yuvPlane.planeBuf;

  const Pel mask = ~( ( 1 << bitDepth ) - 1 );

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

void scaleYuvPlane( YUVPlane& yuvPlane, const int shiftBits, const Pel minVal, const Pel maxVal )
{
  const int stride = yuvPlane.stride;
  const int width  = yuvPlane.width;
  const int height = yuvPlane.height;
  Pel* dst         = yuvPlane.planeBuf;

  if( 0 == shiftBits )
  {
    return;
  }

  if( shiftBits > 0 )
  {
    for( int y = 0; y < height; y++, dst += stride )
    {
      for( int x = 0; x < width; x++)
      {
        dst[x] <<= shiftBits;
      }
    }
  }
  else if ( shiftBits < 0 )
  {
    const int shiftbitsr =- shiftBits;
    const Pel rounding = 1 << ( shiftbitsr-1 );

    for( int y = 0; y < height; y++, dst += stride )
    {
      for( int x = 0; x < width; x++)
      {
        Pel val  = ( dst[ x ] + rounding ) >> shiftbitsr;
        dst[ x ] = std::min<Pel>( minVal, std::max<Pel>( maxVal, val ) );
      }
    }
  }
}

// ====================================================================================================================

void YuvIO::open( const std::string &fileName, bool bWriteMode, const int fileBitDepth[ MAX_NUM_CH ], const int MSBExtendedBitDepth[ MAX_NUM_CH ], const int internalBitDepth[ MAX_NUM_CH ] )
{
  //NOTE: files cannot have bit depth greater than 16
  for( int ch = 0; ch < MAX_NUM_CH; ch++ )
  {
    m_fileBitdepth       [ ch ] = std::min<unsigned>( fileBitDepth[ ch ], 16 );
    m_MSBExtendedBitDepth[ ch ] = MSBExtendedBitDepth[ ch ];
    m_bitdepthShift      [ ch ] = internalBitDepth[ ch ] - m_MSBExtendedBitDepth[ ch ];

    if ( m_fileBitdepth[ ch ] > 16 )
    {
      if ( bWriteMode )
      {
        msg( WARNING, "\nWARNING: Cannot write a yuv file of bit depth greater than 16 - output will be right-shifted down to 16-bit precision\n" );
      }
      else
      {
        EXIT( "ERROR: Cannot read a yuv file of bit depth greater than 16" );
      }
    }
  }

  if ( bWriteMode )
  {
    m_cHandle.open( fileName.c_str(), std::ios::binary | std::ios::out );

    if( m_cHandle.fail() )
    {
      EXIT( "Failed to open output YUV file: " << fileName.c_str() );
    }
  }
  else
  {
    m_cHandle.open( fileName.c_str(), std::ios::binary | std::ios::in );

    if( m_cHandle.fail() )
    {
      EXIT( "Failed to open input YUV file: " << fileName.c_str() );
    }
  }
}

void YuvIO::close()
{
  m_cHandle.close();
}

bool YuvIO::isEof()
{
  return m_cHandle.eof();
}

bool YuvIO::isFail()
{
  return m_cHandle.fail();
}

void YuvIO::skipYuvFrames( int numFrames, const ChromaFormat& inputChFmt, int width, int height  )
{
  if ( numFrames <= 0 )
  {
    return;
  }

  //set the frame size according to the chroma format
  std::streamoff frameSize      = 0;
  unsigned wordsize             = 1;
  for ( int i = 0; i < (int)getNumberValidComponents( inputChFmt ); i++ )
  {
    const ComponentID compID = ComponentID( i );
    frameSize += ( width >> getComponentScaleX( compID, inputChFmt ) ) * ( height >> getComponentScaleY( compID, inputChFmt ));
    if ( m_fileBitdepth[ toChannelType( compID ) ] > 8 )
    {
      wordsize = 2;
    }
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

bool YuvIO::readYuvBuf( YUVBuffer& yuvInBuf, const ChromaFormat& inputChFmt, const ChromaFormat& internChFmt, const int pad[ 2 ], bool bClipToRec709 )
{
  // check end-of-file
  if ( isEof() )
  {
    return false;
  }

  bool is16bit = false;
  for( int ch = 0; ch < MAX_NUM_CH; ch++ )
  {
    if ( m_fileBitdepth[ ch ] > 8 )
    {
      is16bit = true;
      break;
    }
  }

  const int numComp = std::max( getNumberValidComponents( inputChFmt ), getNumberValidComponents( internChFmt ) );

  for( int comp = 0; comp < numComp; comp++ )
  {
    const ComponentID compID   = ComponentID( comp );
    const ChannelType chType   = toChannelType( compID );
    const int desired_bitdepth = m_MSBExtendedBitDepth[ chType ] + m_bitdepthShift[ chType ];
    const bool b709Compliance  = ( bClipToRec709 ) && ( m_bitdepthShift[ chType ] < 0 && desired_bitdepth >= 8 );     /* ITU-R BT.709 compliant clipping for converting say 10b to 8b */
    const Pel minVal           = b709Compliance ? ( (    1 << ( desired_bitdepth - 8 ) )    ) : 0;
    const Pel maxVal           = b709Compliance ? ( ( 0xff << ( desired_bitdepth - 8 ) ) -1 ) : ( 1 << desired_bitdepth ) - 1;
    YUVPlane& yuvPlane         = yuvInBuf.yuvPlanes[ comp ];

    if ( ! readYuvPlane( m_cHandle, yuvPlane, is16bit, m_fileBitdepth[ chType ], pad, compID, inputChFmt, internChFmt ) )
      return false;

    if ( internChFmt == CHROMA_400 )
      continue;

    CHECK( yuvPlane.planeBuf == nullptr, "allocated yuv buffer does not fit to intern chroma format" );

    if ( ! verifyYuvPlane( yuvPlane, m_fileBitdepth[ chType ] ) )
    {
      EXIT("Source image contains values outside the specified bit range!");
    }

    scaleYuvPlane( yuvPlane, m_bitdepthShift[ chType ], minVal, maxVal );
  }

  return true;
}

bool YuvIO::writeYuvBuf( const YUVBuffer& yuvOutBuf, const ChromaFormat& internChFmt, const ChromaFormat& outputChFmt, bool bPackedYUVOutputMode, bool bClipToRec709 )
{
  // compute actual YUV frame size excluding padding size
  bool is16bit              = false;
  bool nonZeroBitDepthShift = false;

  for( int ch = 0; ch < MAX_NUM_CH; ch++ )
  {
    if ( m_fileBitdepth[ ch ] > 8 )
    {
      is16bit = true;
    }
    if ( m_bitdepthShift[ ch ] != 0 )
    {
      nonZeroBitDepthShift = true;
    }
  }

  PelStorage picScaled;
  YUVBuffer yuvScaled;
  if ( nonZeroBitDepthShift )
  {
    picScaled.create( internChFmt, Area( Position( 0, 0 ), Size( yuvOutBuf.yuvPlanes[ COMP_Y ].width, yuvOutBuf.yuvPlanes[ COMP_Y ].height ) ) );
    Window zeroWindow;
    setupYuvBuffer( picScaled, yuvScaled, &zeroWindow );
    PelUnitBuf pic;
    setupPelUnitBuf( yuvOutBuf, pic, internChFmt );
    picScaled.copyFrom( pic );
    for( int comp = 0; comp < (int)getNumberValidComponents( internChFmt ); comp++ )
    {
      const ComponentID compID  = ComponentID( comp );
      const ChannelType chType  = toChannelType( compID );
      const bool b709Compliance = bClipToRec709 && ( -m_bitdepthShift[ chType ] < 0 && m_MSBExtendedBitDepth[ chType ] >= 8 );     /* ITU-R BT.709 compliant clipping for converting say 10b to 8b */
      const Pel minVal          = b709Compliance? ( (    1 << ( m_MSBExtendedBitDepth[ chType ] - 8 ) )    ) : 0;
      const Pel maxVal          = b709Compliance? ( ( 0xff << ( m_MSBExtendedBitDepth[ chType ] - 8 ) ) -1 ) : ( 1 << m_MSBExtendedBitDepth[ chType ] ) - 1;
      YUVPlane& yuvPlane        = yuvScaled.yuvPlanes[ comp ];
      scaleYuvPlane( yuvPlane, -m_bitdepthShift[ chType ], minVal, maxVal );
    }
  }

  const YUVBuffer& yuvWriteBuf = nonZeroBitDepthShift ? yuvScaled : yuvOutBuf;

  for( int comp = 0; comp < (int)getNumberValidComponents( outputChFmt ); comp++ )
  {
    const ComponentID compID = ComponentID( comp );
    const ChannelType chType = toChannelType( compID );
    const YUVPlane& yuvPlane = yuvWriteBuf.yuvPlanes[ comp ];
    if ( ! writeYuvPlane( m_cHandle, yuvPlane, is16bit, m_fileBitdepth[ chType ], bPackedYUVOutputMode, compID, internChFmt, outputChFmt ) )
      return false;
  }

  return true;
}

// ====================================================================================================================

/**
 * write all NALunits in au to bytestream out in a manner satisfying
 * AnnexB of AVC.  NALunits are written in the order they are found in au.
 * the zero_byte word is appended to:
 *  - the initial startcode in the access unit,
 *  - any SPS/PPS nal units
 */
std::vector<uint32_t> writeAnnexB( std::ostream& out, const VvcAccessUnit& au )
{
  out.write(reinterpret_cast<const char*>(au.payload.data()), au.payload.size());
  return au.annexBsizeVec;
}

} // namespace vvenc

//! \}


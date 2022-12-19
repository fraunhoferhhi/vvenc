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
/** \file     FileIOHelper.h
    \brief    file I/O helper class (header)
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

class FileIOHelper
{
public:

  static std::string getFileExtension( std::string fileName )
  {
    std::string ext;
    if(fileName.find_last_of(".") != std::string::npos)
    {
      ext = fileName.substr(fileName.find_last_of(".")+1);
      std::transform( ext.begin(), ext.end(), ext.begin(), ::tolower );
    }
    return ext;
  }


  static bool isY4mInputFilename( std::string fileName ) {  return ( "y4m" == getFileExtension(fileName) ); }
  static bool isY4mHeaderAvailable( std::string fileName )
  {
    if( fileName == "-" ) return false;
    std::fstream cfHandle;
    cfHandle.open( fileName, std::ios::binary | std::ios::in );
    if( cfHandle.fail() ) return false;

    char line[10] = {0};
    std::streamsize headerlinesize = cfHandle.readsome( line, 10 );
    cfHandle.close();

    if( headerlinesize && memcmp( line, "YUV4MPEG2", 9 ) == 0 ) return true;

    return false;
  }

  static int parseY4mHeader( const std::string &fileName, vvenc_config& cfg, vvencChromaFormat& inputFileChromaFormat )
  {
    std::fstream cfHandle;

    if( fileName == "-" )
    {
  #if defined (_WIN32) || defined (WIN32) || defined (_WIN64) || defined (WIN64)
      if( _setmode( _fileno( stdin ), _O_BINARY ) == -1 )
      {
        return -1;
      }
  #endif
    }
    else
    {
      cfHandle.open( fileName, std::ios::binary | std::ios::in );
      if( cfHandle.fail() )
      {
        return -1;
      }
    }

    std::istream& inStream = ( fileName == "-" ) ? std::cin : cfHandle;

    // y4m header syntax example
    // YUV4MPEG2 W1920 H1080 F50:1 Ip A128:117 C420p10

    // init y4m defaults (if not given in header)
    cfg.m_inputBitDepth[0]  = 8;
    inputFileChromaFormat = VVENC_CHROMA_420;

    std::string headerline;
    getline(inStream, headerline);
    if( headerline.empty() ){ return -1; }
    std::transform( headerline.begin(), headerline.end(), headerline.begin(), ::toupper );

    std::regex reg("\\s+"); // tokenize at spaces
    std::sregex_token_iterator iter(headerline.begin(), headerline.end(), reg, -1);
    std::sregex_token_iterator end;
    std::vector<std::string> vec(iter, end);

    bool valid=false;
    int  infosFound = 0;
    for (auto &p : vec)
    {
      if( p == "YUV4MPEG2" ) // read file signature
      { valid = true; }
      else if( p[0] == 'W' ) // width
      {
        cfg.m_SourceWidth = atoi( p.substr( 1 ).c_str());
        infosFound++;
      }
      else if( p[0] == 'H' ) // height
      {
        cfg.m_SourceHeight = atoi( p.substr( 1 ).c_str());
        infosFound++;
      }
      else if( p[0] == 'F' )  // framerate,scale
      {
        size_t sep = p.find(":");
        if( sep == std::string::npos ) return -1;
        cfg.m_FrameRate  = atoi( p.substr( 1, sep-1 ).c_str());
        cfg.m_FrameScale = atoi( p.substr( sep+1 ).c_str());
        infosFound++;
      }
      else if( p[0] == 'A' ) // aspect ratio
      {
        size_t sep = p.find(":");
        if( sep == std::string::npos ) return -1;
        cfg.m_sarWidth  = atoi( p.substr( 1, sep-1 ).c_str());
        cfg.m_sarHeight = atoi( p.substr( sep+1 ).c_str());
      }
      else if( p[0] == 'C' ) // colorspace ( e.g. C420p10)
      {
        std::vector<std::string> ignores = {"JPEG", "MPEG2", "PALVD" }; // ignore some special cases
        for( auto &i : ignores )
        {
          auto n = p.find( i );
          if (n != std::string::npos) p.erase(n, i.length()); // remove from param string (e.g. 420PALVD)
        }

        size_t sep = p.find("P");
        std::string chromatype;
        if( sep != std::string::npos )
        {
          chromatype = ( p.substr( 1, sep-1 ).c_str());
          cfg.m_inputBitDepth[0] = atoi( p.substr( sep+1 ).c_str());
        }
        else
        {
          sep = p.find("MONO");
          if( sep != std::string::npos )
          {
            chromatype = "400";
            if( p == "MONO") cfg.m_inputBitDepth[0] = 8;
            else cfg.m_inputBitDepth[0] = atoi( p.substr( sep+5 ).c_str()); // e.g. mono10
          }
          else
          {
            chromatype = ( p.substr( 1 ).c_str());
            cfg.m_inputBitDepth[0] = 8;
          }
        }

        if( chromatype == "400" )      { inputFileChromaFormat =  VVENC_CHROMA_400; }
        else if( chromatype == "420" ) { inputFileChromaFormat =  VVENC_CHROMA_420; }
        else if( chromatype == "422" ) { inputFileChromaFormat =  VVENC_CHROMA_422; }
        else if( chromatype == "444" ) { inputFileChromaFormat =  VVENC_CHROMA_444; }
        else { return -1; } // unsupported chroma foramt}
      }
      else if( p[0] == 'I' ) // interlaced format (ignore it, because we cannot set it in any params
      {}
      else if( p[0] == 'X' ) // ignore comments
      {}
    }

    if( fileName != "-" )
    {
      cfHandle.close();
    }
    
    if ( infosFound < 3 )
    {
      valid = false;  
    }

    if( !valid ) return -1;

    return (int)headerline.length()+1;
  }


  static bool checkInputFile( std::string fileName, std::string& rcErrText )
  {
    if( fileName == "-" ) return true;

    std::stringstream css;
    std::fstream cfHandle;
    cfHandle.open( fileName, std::ios::binary | std::ios::in );
    if( cfHandle.fail() )
    {
      css <<"cannot open input file " << fileName;
      rcErrText = css.str();
      return false;
    }

    cfHandle.seekg( 0, std::ios::end );
    if ( 0 >= cfHandle.tellg() )
    {
      css <<"cannot determine valid input file size of file" << fileName;
      rcErrText = css.str();
      return false;
    }
    cfHandle.close();

    std::string ext = getFileExtension(fileName);
    std::vector<std::string> unsupportedList { "mp4", "mkv", "ts", "mpeg", "avi" } ;
    for (auto &e : unsupportedList)
    {
      if( e == ext )
      {
        css <<"unsupported input file format " << e << " detected. support only for RAW/YUV/Y4M input files.\n";
        rcErrText = css.str();
        return false;
      }
    }
    return true;
  }

  static bool checkBitstreamFile( std::string fileName, std::string& rcErrText )
  {
    if( fileName == "-" ) return true;
    std::stringstream css;
    std::string ext = getFileExtension(fileName);
    std::vector<std::string> unsupportedList { "mp4", "mkv", "ts", "mpeg", "avi" } ;
    for (auto &e : unsupportedList)
    {
      if( e == ext )
      {
        css <<"unsupported output file format detected. no support for " << e << " container. RAW ES (e.g. *.vvc, *.266) support only.\n" << fileName;
        rcErrText = css.str();
        return false;
      }
    }

    return true;
  }
  
  static bool verifyYuvPlane( vvencYUVPlane& yuvPlane, const int bitDepth )
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
  
  static bool readYuvPlane( std::istream&     fd,
                     vvencYUVPlane&           yuvPlane,
                     bool                     is16bit,
                     int                      fileBitDepth,
                     int                      packedYUVInput,
                     const int&               compID,
                     const vvencChromaFormat& inputChFmt,
                     const vvencChromaFormat& internChFmt
                   )
  {
    const int csx_file = ( (compID == 0) || (inputChFmt==VVENC_CHROMA_444) ) ? 0 : 1;
    const int csy_file = ( (compID == 0) || (inputChFmt!=VVENC_CHROMA_420) ) ? 0 : 1;
    const int csx_dest = ( (compID == 0) || (internChFmt==VVENC_CHROMA_444) ) ? 0 : 1;
    const int csy_dest = ( (compID == 0) || (internChFmt!=VVENC_CHROMA_420) ) ? 0 : 1;
  
    const int stride  = yuvPlane.stride;
    const int width   = yuvPlane.width;
    const int height  = yuvPlane.height;
    LPel* dst         = yuvPlane.ptr;
  
    const int fileStride = ( ( width  << csx_dest ) * ( is16bit ? 2 : 1 ) ) >> csx_file;
    const int fileHeight = ( ( height << csy_dest )                       ) >> csy_file;
  
    if ( compID != 0 && ( inputChFmt == VVENC_CHROMA_400 || internChFmt == VVENC_CHROMA_400 ) )
    {
      if ( internChFmt != VVENC_CHROMA_400 )
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
  
      if ( inputChFmt != VVENC_CHROMA_400 )
      {
        fd.seekg( fileHeight * fileStride, std::ios::cur );
        if ( fd.eof() || fd.fail() )
        {
          return false;
        }
      }
    }
    else if ( packedYUVInput )
    {
      const int fileStride_packed = ( width * 5 / 4 );
      std::vector<uint8_t> bufVec( fileStride_packed );
  
      for( int y = 0; y < height; y++ )
      {
        uint8_t *buf = &( bufVec[0] );
  
        // read a new line
        fd.read( reinterpret_cast<char*>( buf ), fileStride_packed );
        if ( fd.eof() || fd.fail() )
        {
          return false;
        }
  
        for ( int x = 0; x < width; x += 4 )
        {
          int64_t iTemp = 0;
          unsigned char* pucTemp = reinterpret_cast< unsigned char* >( &iTemp );
          pucTemp[0] = buf[0];
          pucTemp[1] = buf[1];
          pucTemp[2] = buf[2];
          pucTemp[3] = buf[3];
          pucTemp[4] = buf[4];
  
          dst[x+0] = 0x03ff & (iTemp>>0);
          dst[x+1] = 0x03ff & (iTemp>>10);
          dst[x+2] = 0x03ff & (iTemp>>20);
          dst[x+3] = 0x03ff & (iTemp>>30);
          buf += 5;
        }
        dst += stride;
      }
    }
    else
    {
      std::vector<uint8_t> bufVec( fileStride );
      uint8_t *buf = &( bufVec[0] );
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
  
  static bool writeYuvPlane( std::ostream&     fd,
                      const vvencYUVPlane&     yuvPlane,
                      bool                     is16bit,
                      int                      fileBitDepth,
                      int                      packedYUVOutputMode,
                      const int&               compID,
                      const vvencChromaFormat& internChFmt,
                      const vvencChromaFormat& outputChFmt
                    )
  {
    const int stride = yuvPlane.stride;
    const int width  = yuvPlane.width;
    const int height = yuvPlane.height;
    const LPel* src  = yuvPlane.ptr;
  
    const int csx_file = ( (compID == 0) || (outputChFmt==VVENC_CHROMA_444) ) ? 0 : 1;
    const int csy_file = ( (compID == 0) || (outputChFmt!=VVENC_CHROMA_420) ) ? 0 : 1;
    const int csx_src  = ( (compID == 0) || (internChFmt==VVENC_CHROMA_444) ) ? 0 : 1;
    const int csy_src  = ( (compID == 0) || (internChFmt!=VVENC_CHROMA_420) ) ? 0 : 1;
  
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
    else if ( compID != 0 && ( outputChFmt == VVENC_CHROMA_400 || internChFmt == VVENC_CHROMA_400 ) )
    {
      if ( outputChFmt != VVENC_CHROMA_400 )
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

  static void scaleYuvPlane( vvencYUVPlane& yuvPlaneOut, const vvencYUVPlane& yuvPlaneIn, 
                             const int shiftBits, const LPel minVal, const LPel maxVal )
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
          dst[ x ] = std::max<LPel>( minVal, std::min<LPel>( maxVal, val ) );
        }
      }
    }
  }
};

} // namespace apputils

//! \}


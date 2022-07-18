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

#include "apputils/apputilsDecl.h"
#include <fstream>
#include <string>

#include "vvenc/vvencCfg.h"

//! \ingroup Interface
//! \{

struct vvencYUVBuffer;

namespace apputils {

class VVEncAppCfg;

// ====================================================================================================================

class APPUTILS_DECL YuvFileIO
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

public:
  int   open( const std::string &fileName, bool bWriteMode, int fileBitDepth, int MSBExtendedBitDepth, int internalBitDepth, 
              vvencChromaFormat fileChrFmt, vvencChromaFormat bufferChrFmt, bool clipToRec709, bool packedYUVMode, bool y4mMode );
  void  close();
  bool  isOpen();
  bool  isEof();
  bool  isFail();
  int   skipYuvFrames ( int numFrames, int width, int height );
  int   readYuvBuf    ( vvencYUVBuffer& yuvInBuf, bool& eof );
  bool  writeYuvBuf   ( const vvencYUVBuffer& yuvOutBuf );
  int   countYuvFrames( int width, int height, bool countFromStart = true ); 
  
  std::string getLastError() const { return m_lastError; }   

  static int parseY4mHeader( const std::string &fileName, vvenc_config& config, VVEncAppCfg& appconfig );
  static bool isY4mInputFilename( std::string fileName );
  static bool isY4mHeaderAvailable( std::string fileName );

  static bool checkInputFile( std::string fileName, std::string& rcErrText );
  static bool checkBitstreamFile( std::string fileName, std::string& rcErrText );

  static std::string getFileExtension( std::string fileName );

};

} // namespace apputils

//! \}


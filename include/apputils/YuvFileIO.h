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
/** \file     YuvFileIO.h
    \brief    yuv file I/O class (header)
*/

#pragma once

#include <fstream>
#include <vector>
#include "apputils/apputilsDecl.h"
#include "vvenc/vvencDecl.h"
#include "vvenc/EncCfgExpert.h"

//! \ingroup Interface
//! \{

namespace vvenc {
struct YUVBuffer;
}

namespace apputils {

// ====================================================================================================================

class APPUTILS_DECL YuvFileIO
{
private:
  std::fstream        m_cHandle;              ///< file handle
  int                 m_fileBitdepth;         ///< bitdepth of input/output video file
  int                 m_MSBExtendedBitDepth;  ///< bitdepth after addition of MSBs (with value 0)
  int                 m_bitdepthShift;        ///< number of bits to increase or decrease image by before/after write/read
  vvenc::ChromaFormat m_fileChrFmt;           ///< chroma format of the file
  vvenc::ChromaFormat m_bufferChrFmt;         ///< chroma format of the buffer
  bool                m_clipToRec709;         ///< clip data according to Recom.709
  bool                m_packedYUVMode;        ///< used packed buffer file format
  std::string         m_lastError;            ///< temporal storage for last occured error 

public:
  int   open( const std::string &fileName, bool bWriteMode, int fileBitDepth, int MSBExtendedBitDepth, int internalBitDepth, 
              vvenc::ChromaFormat fileChrFmt, vvenc::ChromaFormat bufferChrFmt, bool clipToRec709, bool packedYUVMode );
  void  close();
  bool  isEof();
  bool  isFail();
  void  skipYuvFrames( int numFrames, int width, int height );
  bool  readYuvBuf   ( vvenc::YUVBuffer& yuvInBuf );
  bool  writeYuvBuf  ( const vvenc::YUVBuffer& yuvOutBuf );
  std::string getLastError() const { return m_lastError; }   
};

} // namespace apputils

//! \}


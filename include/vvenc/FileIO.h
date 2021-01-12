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
/** \file     FileIO.h
    \brief    file I/O class (header)
*/

#pragma once

#include <fstream>
#include <vector>
#include "vvenc/vvencDecl.h"
#include "vvenc/vvencConfig.h"
#include "vvenc/vvenc.h"

//! \ingroup Interface
//! \{

namespace vvenc {

// ====================================================================================================================

class VVENC_DECL YuvIO
{
private:
  std::fstream  m_cHandle;                            ///< file handle
  int           m_fileBitdepth[ MAX_NUM_CH ];         ///< bitdepth of input/output video file
  int           m_MSBExtendedBitDepth[ MAX_NUM_CH ];  ///< bitdepth after addition of MSBs (with value 0)
  int           m_bitdepthShift[ MAX_NUM_CH ];        ///< number of bits to increase or decrease image by before/after write/read

public:
  void  open( const std::string &fileName, bool bWriteMode, const int fileBitDepth[ MAX_NUM_CH ], const int MSBExtendedBitDepth[ MAX_NUM_CH ], const int internalBitDepth[ MAX_NUM_CH ] );
  void  close();
  bool  isEof();
  bool  isFail();
  void  skipYuvFrames( int numFrames, const ChromaFormat& inputChFmt, int width, int height );
  bool  readYuvBuf   ( YUVBuffer& yuvInBuf,        const ChromaFormat& inputChFmt,  const ChromaFormat& internChFmt, const int pad[ 2 ], bool bClipToRec709 );
  bool  writeYuvBuf  ( const YUVBuffer& yuvOutBuf, const ChromaFormat& internChFmt, const ChromaFormat& outputChFmt, bool bPackedYUVOutputMode, bool bClipToRec709 );
};

// ====================================================================================================================

class AccessUnit;
std::vector<uint32_t> VVENC_DECL writeAnnexB( std::ostream& out, const VvcAccessUnit& au );

} // namespace vvenc

//! \}


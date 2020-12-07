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
/** \file     EncoderIf.h
    \brief    encoder lib interface
*/

#pragma once

#include <string>
#include <functional>
#include "vvenc/vvencDecl.h"
#include "vvenc/Basics.h"

//! \ingroup Interface
//! \{

namespace vvenc {

class  EncLib;
class  EncCfg;
class  AccessUnit;
struct YUVBuffer;

// ====================================================================================================================

class VVENC_DECL YUVWriterIf
{
protected:
  YUVWriterIf() {}
  virtual ~YUVWriterIf() {}

public:
  virtual void outputYuv( const YUVBuffer& /*yuvOutBuf*/ )
  {
  }
};

// ====================================================================================================================

class VVENC_DECL EncoderIf
{
  private:
    EncLib* m_pEncLib;

  public:
    EncoderIf();

    ~EncoderIf();

    void  initEncoderLib  ( const EncCfg& encCfg, YUVWriterIf* yuvWriterIf = nullptr );
    void  initPass        ( int pass = 0 );
    void  encodePicture   ( bool flush, const YUVBuffer& yuvInBuf, AccessUnit& au, bool& isQueueEmpty );
    void  uninitEncoderLib();
    void  printSummary    ();
};

// ====================================================================================================================

void        VVENC_DECL registerMsgCbf( std::function<void( int, const char*, va_list )> msgFnc );   ///< set message output function for encoder lib. if not set, no messages will be printed.
std::string VVENC_DECL setSIMDExtension( const std::string& simdId );                               ///< tries to set given simd extensions used. if not supported by cpu, highest possible extension level will be set and returned.
bool        VVENC_DECL isTracingEnabled();                                                          ///< checks if library has tracing supported enabled (see ENABLE_TRACING).
std::string VVENC_DECL getCompileInfoString();                                                      ///< creates compile info string containing OS, Compiler and Bit-depth (e.g. 32 or 64 bit).
void        VVENC_DECL decodeBitstream( const std::string& FileName);                               ///< decode bitstream with limited build in decoder

} // namespace vvenc

//! \}


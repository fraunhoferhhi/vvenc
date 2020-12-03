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


/** \file     EncoderIf.cpp
    \brief    encoder lib interface
*/

#include "vvenc/EncoderIf.h"

#include "EncoderLib/EncLib.h"
#include "CommonLib/CommonDef.h"
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_TRAFO
#include "CommonLib/TrQuant_EMT.h"
#endif

//! \ingroup Interface
//! \{

namespace vvenc {

bool tryDecodePicture( Picture* pic, const int expectedPoc, const std::string& bitstreamFileName, FFwdDecoder& ffwdDecoder, ParameterSetMap<APS>* apsMap, bool bDecodeUntilPocFound = false, int debugPOC = -1, bool copyToEnc = true );

// ====================================================================================================================

EncoderIf::EncoderIf()
  : m_pEncLib( nullptr )
{
}

EncoderIf::~EncoderIf()
{
  uninitEncoderLib();
}

void EncoderIf::initEncoderLib( const EncCfg& encCfg, YUVWriterIf* yuvWriterIf )
{
  CHECK( m_pEncLib != nullptr, "encoder library already initialized" );
  m_pEncLib = new EncLib;
  m_pEncLib->initEncoderLib( encCfg, yuvWriterIf );
}

void EncoderIf::uninitEncoderLib()
{
  if ( m_pEncLib )
  {
    m_pEncLib->uninitEncoderLib();
    delete m_pEncLib;
    m_pEncLib = nullptr;
  }
}

void EncoderIf::initPass( int pass )
{
  CHECK( m_pEncLib == nullptr, "encoder library not created" );
  m_pEncLib->initPass( pass );
}

void EncoderIf::encodePicture( bool flush, const YUVBuffer& yuvInBuf, AccessUnit& au, bool& isQueueEmpty )
{
  CHECK( m_pEncLib == nullptr, "encoder library not initialized" );
  m_pEncLib->encodePicture( flush, yuvInBuf, au, isQueueEmpty );
}

void EncoderIf::printSummary()
{
  CHECK( m_pEncLib == nullptr, "encoder library not initialized" );
  m_pEncLib->printSummary();
}

// ====================================================================================================================

void setMsgFnc( MsgFnc msgFnc )
{
  g_msgFnc = msgFnc;
}

std::string setSIMDExtension( const std::string& simdId )
{
  std::string ret = "NA";
#if ENABLE_SIMD_OPT
#ifdef TARGET_SIMD_X86
  const char* simdSet = read_x86_extension( simdId );
  ret = simdSet;
#endif
  g_pelBufOP.initPelBufOpsX86();
#endif
#if ENABLE_SIMD_TRAFO
  g_tCoeffOps.initTCoeffOpsX86();
#endif
  return ret;
}

bool isTracingEnabled()
{
#if ENABLE_TRACING
  return true;
#else
  return false;
#endif
}

std::string getCompileInfoString()
{
  char convBuf[ 256 ];
  std::string compileInfo;
  snprintf( convBuf, sizeof( convBuf ), NVM_ONOS );      compileInfo += convBuf;
  snprintf( convBuf, sizeof( convBuf ), NVM_COMPILEDBY); compileInfo += convBuf;
  snprintf( convBuf, sizeof( convBuf ), NVM_BITS );      compileInfo += convBuf;
  return compileInfo;
}

void decodeBitstream( const std::string& FileName)
{
  FFwdDecoder ffwdDecoder; 
  Picture cPicture; cPicture.poc=-8000;

  if( tryDecodePicture( &cPicture, -1, FileName, ffwdDecoder, nullptr, false, cPicture.poc, false ))
  {
    msg( ERROR, "decoding failed");
    THROW("error decoding");
  }
}

} // namespace vvenc

//! \}


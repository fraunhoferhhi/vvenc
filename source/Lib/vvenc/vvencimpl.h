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
/**
  \file    hhivvcencimpl.h
  \brief   This file contains the internal interface of the vvenc SDK.
*/

#pragma once

#include "vvenc/vvencCfg.h"
#include "vvenc/vvenc.h"

namespace vvenc {

class EncLib;
class AccessUnitList;
/**
  \ingroup VVEncExternalInterfaces
  The class HhiVvcDec provides the decoder user interface. The simplest way to use the decoder is to call init() to initialize an decoder instance with the
  the given VVCDecoderParameters. After initialization the decoding of the video is performed by using the decoder() method to hand over compressed packets (bitstream chunks) in decoding order
  and retrieve uncompressed pictures. The decoding can be end by calling flush() that causes the decoder to finish decoding of all pending packets.
  Finally calling uninit() releases all allocated resources held by the decoder internally.
*/
class VVEncImpl
{
public:

  enum VVEncInternalState
  {
    INTERNAL_STATE_UNINITIALIZED = 0,
    INTERNAL_STATE_INITIALIZED   = 1,
    INTERNAL_STATE_ENCODING      = 2,
    INTERNAL_STATE_FLUSHING      = 3,
    INTERNAL_STATE_FINALIZED     = 4
  };

  VVEncImpl();
  virtual ~VVEncImpl();

  int init( const VVEncCfg& rcVVEncCfg, YUVWriterIf* pcYUVWriterIf );

  int initPass( int pass );
  int uninit();

  bool isInitialized() const;

  int encode( YUVBuffer* pcYUVBuffer, AccessUnit& rcAccessUnit, bool& rEncodeDone);

  int getConfig( VVEncCfg& rcVVEncCfg ) const;
  int checkConfig( const VVEncCfg& rcVVEncCfg );
  int reconfig( const VVEncCfg& rcVVEncCfg );

  int setAndRetErrorMsg( int Ret );

  int getNumLeadFrames() const;
  int getNumTrailFrames() const;

  int printSummary() const;

  std::string getEncoderInfo() const;

  std::string getLastError() const;

  static std::string getErrorMsg( int nRet );
  static std::string getVersionNumber();
  
  static void        registerMsgCbf( std::function<void( int, const char*, va_list )> msgFnc );   ///< set message output function for encoder lib. if not set, no messages will be printed.
  static std::string setSIMDExtension( const std::string& simdId );                               ///< tries to set given simd extensions used. if not supported by cpu, highest possible extension level will be set and returned.

private:

  int xCopyAu( AccessUnit& rcAccessUnit, const AccessUnitList& rcAu );

private:
  VVEncInternalState     m_eState               = INTERNAL_STATE_UNINITIALIZED;
  bool                   m_bInitialized         = false;

  VVEncCfg               m_cVVEncCfgExt;      // external (user) config ( not usd currently)
  VVEncCfg               m_cVVEncCfg;         // internal (adapted) config

  std::string            m_cErrorString;
  std::string            m_sEncoderCapabilities;

  EncLib*                m_pEncLib = nullptr;
};


} // namespace


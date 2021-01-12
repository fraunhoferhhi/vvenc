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
/**
  \ingroup VVEncExternalInterfaces
  \file    hhivvcencimpl.h
  \brief   This file contains the internal interface of the hhivvcenc SDK.
  \author  christian.lehmann@hhi.fraunhofer.de
  \date    08/10/2019
*/

#pragma once

#include "vvenc/EncCfg.h"
#include "vvenc/vvenc.h"

namespace vvenc {

class EncLib;
class AccessUnit;
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

  VVEncImpl();
  virtual ~VVEncImpl();

  int init( const VVEncParameter& rcVVEncParameter );
  int init( const EncCfg& rcEncCfg, YUVWriterIf* pcYUVWriterIf );

  int initPass( int pass );
  int uninit();

  bool isInitialized() const;

  int encode( YuvPicture* pcYuvPicture, VvcAccessUnit& rcVvcAccessUnit, bool& rEncodeDone );
  int encode( YUVBuffer* pcYUVBuffer, VvcAccessUnit& rcVvcAccessUnit, bool& rEncodeDone);

  int getConfig( VVEncParameter& rcVVEncParameter ) const;
  int checkConfig( const vvenc::VVEncParameter& rcVVEncParameter );
  int reconfig( const VVEncParameter& rcVVEncParameter );

  int setAndRetErrorMsg( int Ret );

  int getNumLeadFrames() const;
  int getNumTrailFrames() const;

  int printConfig() const;
  int printSummary() const;

  std::string getEncoderInfo() const;

  std::string getLastError() const;

  static std::string getErrorMsg( int nRet );
  static std::string getVersionNumber();
  static std::string getPresetParamsAsStr( int iQuality );


  static void        registerMsgCbf( std::function<void( int, const char*, va_list )> msgFnc );   ///< set message output function for encoder lib. if not set, no messages will be printed.
  static std::string setSIMDExtension( const std::string& simdId );                               ///< tries to set given simd extensions used. if not supported by cpu, highest possible extension level will be set and returned.
  static bool        isTracingEnabled();                                                          ///< checks if library has tracing supported enabled (see ENABLE_TRACING).
  static std::string getCompileInfoString();                                                      ///< creates compile info string containing OS, Compiler and Bit-depth (e.g. 32 or 64 bit).
  static void        decodeBitstream( const std::string& FileName);                               ///< decode bitstream with limited build in decoder

private:

  int xCheckParameter ( const VVEncParameter& rcSrc, std::string& rcErrorString ) const;
  int xCheckParameter( const EncCfg& rcSrc, std::string& rcErrorString ) const;

  int xInitLibCfg( const VVEncParameter& rcVVEncParameter, EncCfg& rcEncCfg );

  int xCopyAndPadInputPlane( int16_t* pDes, const int iDesStride, const int iDesWidth, const int iDesHeight,
                       const int16_t* pSrc, const int iSrcStride, const int iSrcWidth, const int iSrcHeight );
  int xCopyAu( VvcAccessUnit& rcVvcAccessUnit, const AccessUnit& rcAu );

private:
  bool                   m_bInitialized         = false;
  bool                   m_bFlushed             = false;

  VVEncParameter         m_cVVEncParameter;
  EncCfg                 m_cEncCfg;

  std::string            m_cErrorString;
  std::string            m_sEncoderCapabilities;

  EncLib*                m_pEncLib = nullptr;
};


} // namespace


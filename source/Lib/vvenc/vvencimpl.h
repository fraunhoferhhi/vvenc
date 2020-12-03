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

#include <chrono>
#include "vvenc/EncCfg.h"
#include "vvenc/EncoderIf.h"
#include "vvenc/vvenc.h"

namespace vvenc {

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
  int initPass( int pass );
  int uninit();

  int encode( InputPicture* pcInputPicture, VvcAccessUnit& rcVvcAccessUnit);
  int flush( VvcAccessUnit& rcVvcAccessUnit );

  int getPreferredBuffer( PicBuffer &rcPicBuffer );
  int getConfig( VVEncParameter& rcVVEncParameter );
  int checkConfig( const vvenc::VVEncParameter& rcVVEncParameter );

  void clockStartTime();
  void clockEndTime();
  double clockGetTimeDiffMs();

  int setAndRetErrorMsg( int Ret );

  int getNumLeadFrames();
  int getNumTrailFrames();

  const char* getEncoderInfo();

  static const char* getErrorMsg( int nRet );
  static const char* getVersionNumber();

  static const char* getPresetParamsAsStr( int iQuality );

private:

  int xCheckParameter ( const VVEncParameter& rcSrc, std::string& rcErrorString );

  int xInitLibCfg( const VVEncParameter& rcVVEncParameter, vvenc::EncCfg& rcEncCfg );

  int xCopyAndPadInputPlane( int16_t* pDes, const int iDesStride, const int iDesWidth, const int iDesHeight,
                       const int16_t* pSrc, const int iSrcStride, const int iSrcWidth, const int iSrcHeight, const int iMargin );
  int xCopyAu( VvcAccessUnit& rcVvcAccessUnit, const vvenc::AccessUnit& rcAu );

public:
  bool                                                        m_bInitialized         = false;
  bool                                                        m_bFlushed             = false;

  vvenc::EncoderIf                                            m_cEncoderIf;                      ///< encoder library class

  VVEncParameter                                              m_cVVEncParameter;
  vvenc::EncCfg                                               m_cEncCfg;

  std::string                                                 m_sEncoderInfo;
  std::string                                                 m_cErrorString;
  std::string                                                 m_sEncoderCapabilities;
  static std::string                                          m_sPresetAsStr;
  static std::string                                          m_cTmpErrorString;

  std::chrono::steady_clock::time_point                       m_cTPStart;
  std::chrono::steady_clock::time_point                       m_cTPEnd;
};


} // namespace


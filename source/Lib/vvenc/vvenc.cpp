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
  \ingroup vvencExternalInterfaces
  \file    vvenc.cpp
  \brief   This file contains the external interface of the hhivvcdec SDK.
  \author  christian.lehmann@hhi.fraunhofer.de
  \date    08/08/2020
*/

#include "vvenc/vvenc.h"
#include "vvencimpl.h"

namespace vvenc {

VVEnc::VVEnc()
{
  m_pcVVEncImpl = new VVEncImpl;
}

/// Destructor
VVEnc::~VVEnc()
{
  if( NULL != m_pcVVEncImpl )
  {
    if( m_pcVVEncImpl->isInitialized() )
    {
      uninit();
    }
    delete m_pcVVEncImpl;
    m_pcVVEncImpl = NULL;
  }
}

int VVEnc::checkConfig( const VVEncCfg& rcVVEncCfg )
{
  if( rcVVEncCfg.m_numWppThreads > 64 ){ return m_pcVVEncImpl->setAndRetErrorMsg( VVENC_ERR_NOT_SUPPORTED ); }

  return m_pcVVEncImpl->checkConfig( rcVVEncCfg );
}

int VVEnc::init( const VVEncCfg& rcVVEncCfg, YUVWriterIf* pcYUVWriterIf  )
{
  if( m_pcVVEncImpl->isInitialized() )      { return m_pcVVEncImpl->setAndRetErrorMsg( VVENC_ERR_INITIALIZE ); }

  return m_pcVVEncImpl->init( rcVVEncCfg, pcYUVWriterIf );
}

int VVEnc::initPass( int pass )
{
  if( !m_pcVVEncImpl->isInitialized() ){ return m_pcVVEncImpl->setAndRetErrorMsg( VVENC_ERR_INITIALIZE ); }

  return m_pcVVEncImpl->initPass( pass );
}

int VVEnc::uninit()
{
  if( !m_pcVVEncImpl->isInitialized() ){ return m_pcVVEncImpl->setAndRetErrorMsg( VVENC_ERR_INITIALIZE ); }

  return m_pcVVEncImpl->uninit( );
}

bool VVEnc::isInitialized()
{
  return m_pcVVEncImpl->isInitialized();
}

int VVEnc::encode( YUVBuffer* pcYUVBuffer, AccessUnit& rcAccessUnit, bool& rbEncodeDone)
{
  if( !m_pcVVEncImpl->isInitialized() ){ return m_pcVVEncImpl->setAndRetErrorMsg(VVENC_ERR_INITIALIZE); }

  return m_pcVVEncImpl->encode( pcYUVBuffer, rcAccessUnit, rbEncodeDone );
}

int VVEnc::getConfig( VVEncCfg& rcVVEncCfg )
{
  if( !m_pcVVEncImpl->isInitialized() )
  {  return m_pcVVEncImpl->setAndRetErrorMsg(VVENC_ERR_INITIALIZE); }

  return m_pcVVEncImpl->setAndRetErrorMsg( m_pcVVEncImpl->getConfig( rcVVEncCfg ) );
}

int VVEnc::reconfig( const VVEncCfg& rcVVEncCfg )
{
  if( !m_pcVVEncImpl->isInitialized() )
  {  return m_pcVVEncImpl->setAndRetErrorMsg(VVENC_ERR_INITIALIZE); }

  return m_pcVVEncImpl->setAndRetErrorMsg( m_pcVVEncImpl->reconfig( rcVVEncCfg ) );
}

std::string VVEnc::getEncoderInfo() const
{
  return m_pcVVEncImpl->getEncoderInfo();
}

std::string VVEnc::getLastError() const
{
  return m_pcVVEncImpl->getLastError();
}

int VVEnc::getNumLeadFrames() const
{
  return m_pcVVEncImpl->getNumLeadFrames();
}

int VVEnc::getNumTrailFrames() const
{
  return m_pcVVEncImpl->getNumTrailFrames();
}

int VVEnc::printSummary() const
{
  return m_pcVVEncImpl->printSummary();
}

int VVEnc::printConfig() const
{
  return m_pcVVEncImpl->printConfig();
}

std::string VVEnc::getVersionNumber()
{
  return VVEncImpl::getVersionNumber();
}

std::string VVEnc::getErrorMsg( int nRet )
{
  return VVEncImpl::getErrorMsg(nRet);
}

std::string VVEnc::getPresetParamsAsStr( int iQuality )
{
  return VVEncImpl::getPresetParamsAsStr(iQuality);
}

void VVEnc::registerMsgCbf( std::function<void( int, const char*, va_list )> msgFnc )
{
  VVEncImpl::registerMsgCbf( msgFnc );
}

std::string VVEnc::setSIMDExtension( const std::string& simdId )  ///< tries to set given simd extensions used. if not supported by cpu, highest possible extension level will be set and returned.
{
  return VVEncImpl::setSIMDExtension( simdId );
}

bool VVEnc::isTracingEnabled()
{
  return VVEncImpl::isTracingEnabled();
}

std::string VVEnc::getCompileInfoString()
{
  return VVEncImpl::getCompileInfoString();
}

void VVEnc::decodeBitstream( const std::string& FileName)
{
  VVEncImpl::decodeBitstream( FileName );
}

YUVBufferStorage::YUVBufferStorage( const ChromaFormat& chFmt, const int frameWidth, const int frameHeight )
  : YUVBuffer()
{
  for ( int i = 0; i < MAX_NUM_COMP; i++ )
  {
    YUVBuffer::Plane& yuvPlane = planes[ i ];
    yuvPlane.width  = getWidthOfComponent ( chFmt, frameWidth,  i );
    yuvPlane.height = getHeightOfComponent( chFmt, frameHeight, i );
    yuvPlane.stride = yuvPlane.width;
    const int size  = yuvPlane.stride * yuvPlane.height;
    yuvPlane.ptr    = ( size > 0 ) ? new int16_t[ size ] : nullptr;
  }
}

YUVBufferStorage::~YUVBufferStorage()
{
  for ( int i = 0; i < MAX_NUM_COMP; i++ )
  {
    delete [] planes[ i ].ptr;
  }
}

} // namespace


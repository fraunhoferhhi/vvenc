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
  \ingroup hhivvcdeclibExternalInterfaces
  \file    vvencimpl.cpp
  \brief   This file contains the internal interface of the hhivvcdec SDK.
  \author  christian.lehmann@hhi.fraunhofer.de
  \date    08/10/2019
*/

#include "vvencimpl.h"

#include <iostream>
#include <stdio.h>
#include <algorithm>

#include "vvenc/version.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/Nal.h"


#include "EncoderLib/EncLib.h"
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_TRAFO
#include "CommonLib/TrQuant_EMT.h"
#endif

namespace vvenc {

#define ROTPARAMS(x, message) if(x) { rcErrorString = message; return VVENC_ERR_PARAMETER;}

bool tryDecodePicture( Picture* pic, const int expectedPoc, const std::string& bitstreamFileName, FFwdDecoder& ffwdDecoder, ParameterSetMap<APS>* apsMap, bool bDecodeUntilPocFound = false, int debugPOC = -1, bool copyToEnc = true );

VVEncImpl::VVEncImpl()
{

}

VVEncImpl::~VVEncImpl()
{

}

int VVEncImpl::checkConfig( const vvenc::VVEncParameter& rcVVEncParameter )
{
  int iRet = xCheckParameter( rcVVEncParameter, m_cErrorString );
  if( 0 != iRet ) { return iRet; }

  vvenc::EncCfg cEncCfg;
  if( 0 != xInitLibCfg( rcVVEncParameter, cEncCfg ) )
  {
    return VVENC_ERR_INITIALIZE;
  }

  return VVENC_OK;
}

int VVEncImpl::init( const vvenc::VVEncParameter& rcVVEncParameter )
{
  if( m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  // Set SIMD extension in case if it hasn't been done before, otherwise it simply reuses the current state
  std::string simdOpt;
  std::string curSimd = setSIMDExtension( simdOpt );

  int iRet = xCheckParameter( rcVVEncParameter, m_cErrorString );
  if( 0 != iRet ) { return iRet; }

  std::stringstream cssCap;
  cssCap << getCompileInfoString() << "[SIMD=" << curSimd <<"]";
  m_sEncoderCapabilities = cssCap.str();

  m_cVVEncParameter = rcVVEncParameter;

  if( 0 != xInitLibCfg( rcVVEncParameter, m_cEncCfg ) )
  {
    return VVENC_ERR_INITIALIZE;
  }

  // initialize the encoder
  m_pEncLib = new EncLib;
  YUVWriterIf* pcYUVWriterIf = nullptr;

  try
  {
    m_pEncLib->initEncoderLib( m_cEncCfg, pcYUVWriterIf );
  }
  catch( std::exception& e )
  {
    m_cErrorString = e.what();
    return VVENC_ERR_UNSPECIFIED;
  }

  m_bInitialized = true;
  m_bFlushed     = false;
  return VVENC_OK;
}

int VVEncImpl::init( const EncCfg& rcEncCfg, YUVWriterIf* pcYUVWriterIf )
{
  if( m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  // Set SIMD extension in case if it hasn't been done before, otherwise it simply reuses the current state
  std::string simdOpt;
  std::string curSimd = setSIMDExtension( simdOpt );

  int iRet = xCheckParameter( rcEncCfg, m_cErrorString );
  if( 0 != iRet ) { return iRet; }

  std::stringstream cssCap;
  cssCap << getCompileInfoString() << "[SIMD=" << curSimd <<"]";
  m_sEncoderCapabilities = cssCap.str();

  m_cEncCfg = rcEncCfg;

  // initialize the encoder
  //m_cEncoderIf.initEncoderLib( m_cEncCfg );
  m_pEncLib = new EncLib;

  try
  {
    m_pEncLib->initEncoderLib( m_cEncCfg, pcYUVWriterIf );
  }
  catch( std::exception& e )
  {
    m_cErrorString = e.what();
    return VVENC_ERR_UNSPECIFIED;
  }

  m_bInitialized = true;
  m_bFlushed     = false;
  return VVENC_OK;
}

int VVEncImpl::initPass( int pass )
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }
  if( pass > 1 )
  {
    std::stringstream css;
    css << "initPass(" << pass << ") no support for pass " << pass << ". use 0 (first pass) and 1 (second pass)";
    m_cErrorString = css.str();
    return VVENC_ERR_NOT_SUPPORTED;
  }

  if ( m_pEncLib )
  {
    try
    {
      m_pEncLib->initPass( pass );
    }
    catch( std::exception& e )
    {
      m_cErrorString = e.what();
      return VVENC_ERR_UNSPECIFIED;
    }
  }

  m_bFlushed = false;
  return VVENC_OK;
}

int VVEncImpl::uninit()
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  if ( m_pEncLib )
  {
    try
    {
      m_pEncLib->uninitEncoderLib();
      delete m_pEncLib;
      m_pEncLib = nullptr;
    }
    catch( std::exception& e )
    {
      m_cErrorString = e.what();
      return VVENC_ERR_UNSPECIFIED;
    }
  }

  m_bInitialized = false;
  m_bFlushed     = false;
  return VVENC_OK;
}

bool VVEncImpl::isInitialized() const
{
  return m_bInitialized;
}


int VVEncImpl::encode( YuvPicture* pcYuvPicture, AccessUnit& rcAccessUnit, bool& rbEncodeDone )
{
  if( !m_bInitialized )                 { return VVENC_ERR_INITIALIZE; }
  if( m_bFlushed )                      { m_cErrorString = "encoder already flushed"; return VVENC_ERR_RESTART_REQUIRED; }

  int iRet= VVENC_OK;

  YUVBuffer cYUVBuffer;
  bool bFlush = false;
  if( pcYuvPicture )
  {
    if( pcYuvPicture->y == nullptr || pcYuvPicture->u == nullptr || pcYuvPicture->v == nullptr )
    {
      m_cErrorString = "InputPicture: invalid input buffers";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( pcYuvPicture->width != m_cEncCfg.m_SourceWidth )
    {
      m_cErrorString = "InputPicture: unsupported width";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( pcYuvPicture->height != m_cEncCfg.m_SourceHeight )
    {
      m_cErrorString = "InputPicture: unsupported height";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( pcYuvPicture->width > pcYuvPicture->stride )
    {
      m_cErrorString = "InputPicture: unsupported width stride combination";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( pcYuvPicture->cStride && pcYuvPicture->width/2 > pcYuvPicture->cStride )
    {
      m_cErrorString = "InputPicture: unsupported width cstride combination";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( pcYuvPicture->bitDepth < 10 || pcYuvPicture->bitDepth > 16 )
    {
      std::stringstream css;
      css << "InputPicture: unsupported input BitDepth " <<  pcYuvPicture->bitDepth  << ". must be 10 <= BitDepth <= 16";
      m_cErrorString = css.str();
      return VVENC_ERR_UNSPECIFIED;
    }

    // we know that the internal buffer requires to be a multiple of 8 in each direction
    int internalLumaWidth = ((pcYuvPicture->width + 7)/8)*8;
    int internalLumaHeight = ((pcYuvPicture->height + 7)/8)*8;
    int internalLumaStride = (internalLumaWidth > pcYuvPicture->stride) ? internalLumaWidth : pcYuvPicture->stride;

    int iChromaInStride = internalLumaStride >> 1;
    if( pcYuvPicture->cStride && pcYuvPicture->cStride > (internalLumaWidth >> 1) )
    {
      iChromaInStride =  pcYuvPicture->cStride;
    }

    for ( int i = 0; i < 3; i++ )
    {
      YUVPlane& yuvPlane = cYUVBuffer.yuvPlanes[ i ];
      if ( i > 0 )
      {
        yuvPlane.width     = internalLumaWidth >> 1;
        yuvPlane.height    = internalLumaHeight >> 1;
        yuvPlane.stride    = iChromaInStride;
      }
      else
      {
        yuvPlane.width     = internalLumaWidth;
        yuvPlane.height    = internalLumaHeight;
        yuvPlane.stride    = internalLumaStride;
      }
      const int size     = yuvPlane.stride * yuvPlane.height;
      yuvPlane.planeBuf  = ( size > 0 ) ? new int16_t[ size ] : nullptr;
    }

    xCopyAndPadInputPlane( cYUVBuffer.yuvPlanes[0].planeBuf, cYUVBuffer.yuvPlanes[0].stride, cYUVBuffer.yuvPlanes[0].width, cYUVBuffer.yuvPlanes[0].height,
                           (int16_t*)pcYuvPicture->y, pcYuvPicture->stride, pcYuvPicture->width, pcYuvPicture->height );
    xCopyAndPadInputPlane( cYUVBuffer.yuvPlanes[1].planeBuf, iChromaInStride, cYUVBuffer.yuvPlanes[1].width, cYUVBuffer.yuvPlanes[1].height,
                           (int16_t*)pcYuvPicture->u, iChromaInStride, pcYuvPicture->width>>1, pcYuvPicture->height>>1 );
    xCopyAndPadInputPlane( cYUVBuffer.yuvPlanes[2].planeBuf, iChromaInStride, cYUVBuffer.yuvPlanes[2].width, cYUVBuffer.yuvPlanes[2].height,
                           (int16_t*)pcYuvPicture->v, iChromaInStride, pcYuvPicture->width>>1, pcYuvPicture->height>>1 );

    cYUVBuffer.sequenceNumber = pcYuvPicture->sequenceNumber;
    if( pcYuvPicture->ctsValid )
    {
      cYUVBuffer.cts = pcYuvPicture->cts;
      cYUVBuffer.ctsValid = true;
    }
  }
  else
  {
    bFlush = true;
  }


  // reset AU data
  rcAccessUnit.payload.clear();
  rcAccessUnit.cts              = 0;
  rcAccessUnit.dts              = 0;
  rcAccessUnit.ctsValid         = false;
  rcAccessUnit.dtsValid         = false;
  rcAccessUnit.rap              = false;
  rcAccessUnit.sliceType        = NUMBER_OF_SLICE_TYPES;
  rcAccessUnit.refPic           = false;
  rcAccessUnit.temporalLayer    = 0;
  rcAccessUnit.poc              = 0;
  rcAccessUnit.status           = 0;
  rcAccessUnit.infoString.clear();
  rcAccessUnit.nalUnitTypeVec.clear();
  rcAccessUnit.annexBsizeVec.clear();

  rbEncodeDone = false;

  AccessUnitList cAu;
  try
  {
    m_pEncLib->encodePicture( bFlush, cYUVBuffer, cAu, rbEncodeDone );
  }
  catch( std::exception& e )
  {
    m_cErrorString = e.what();
    return VVENC_ERR_UNSPECIFIED;
  }

  if( rbEncodeDone ){ m_bFlushed = true; }

  /* copy output AU */
  if ( !cAu.empty() )
  {
    iRet = xCopyAu( rcAccessUnit, cAu  );
  }

  if( cYUVBuffer.yuvPlanes[0].planeBuf )
  {
    /* free memory of input image */
    for ( int i = 0; i < 3; i++ )
    {
      vvenc::YUVPlane& yuvPlane = cYUVBuffer.yuvPlanes[ i ];
      if ( yuvPlane.planeBuf )
        delete [] yuvPlane.planeBuf;
    }
  }

  return iRet;
}

int VVEncImpl::encode( YUVBuffer* pcYUVBuffer, AccessUnit& rcAccessUnit, bool& rbEncodeDone )
{
  if( !m_bInitialized )                 { return VVENC_ERR_INITIALIZE; }
  if( m_bFlushed )                      { m_cErrorString = "encoder already flushed"; return VVENC_ERR_RESTART_REQUIRED; }

  int iRet= VVENC_OK;

  YUVBuffer cYUVBuffer; // internal pic
  bool bFlush = false;
  if( pcYUVBuffer )
  {
    if( pcYUVBuffer->yuvPlanes[0].planeBuf == nullptr )
    {
      m_cErrorString = "InputPicture: invalid input buffers";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( m_cEncCfg.m_internChromaFormat != CHROMA_400 )
    {
      if( pcYUVBuffer->yuvPlanes[1].planeBuf == nullptr ||
          pcYUVBuffer->yuvPlanes[2].planeBuf == nullptr )
      {
        m_cErrorString = "InputPicture: invalid input buffers for chroma";
        return VVENC_ERR_UNSPECIFIED;
      }
    }

    if( pcYUVBuffer->yuvPlanes[0].width != m_cEncCfg.m_SourceWidth )
    {
      m_cErrorString = "InputPicture: unsupported width";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( pcYUVBuffer->yuvPlanes[0].height != m_cEncCfg.m_SourceHeight )
    {
      m_cErrorString = "InputPicture: unsupported height";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( pcYUVBuffer->yuvPlanes[0].width > pcYUVBuffer->yuvPlanes[0].stride )
    {
      m_cErrorString = "InputPicture: unsupported width stride combination";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( m_cEncCfg.m_internChromaFormat != CHROMA_400 )
    {
      if( m_cEncCfg.m_internChromaFormat == CHROMA_444 )
      {
        if( pcYUVBuffer->yuvPlanes[1].stride && pcYUVBuffer->yuvPlanes[0].width > pcYUVBuffer->yuvPlanes[1].stride )
        {
          m_cErrorString = "InputPicture: unsupported width cstride combination for 2nd plane";
          return VVENC_ERR_UNSPECIFIED;
        }

        if( pcYUVBuffer->yuvPlanes[2].stride && pcYUVBuffer->yuvPlanes[0].width > pcYUVBuffer->yuvPlanes[2].stride )
        {
          m_cErrorString = "InputPicture: unsupported width cstride combination for 3rd plane";
          return VVENC_ERR_UNSPECIFIED;
        }
      }
      else
      {
        if( pcYUVBuffer->yuvPlanes[1].stride && pcYUVBuffer->yuvPlanes[0].width/2 > pcYUVBuffer->yuvPlanes[1].stride )
        {
          m_cErrorString = "InputPicture: unsupported width cstride combination for 2nd plane";
          return VVENC_ERR_UNSPECIFIED;
        }

        if( pcYUVBuffer->yuvPlanes[2].stride && pcYUVBuffer->yuvPlanes[0].width/2 > pcYUVBuffer->yuvPlanes[2].stride )
        {
          m_cErrorString = "InputPicture: unsupported width cstride combination for 3rd plane";
          return VVENC_ERR_UNSPECIFIED;
        }
      }
    }

    // we know that the internal buffer requires to be a multiple of 8 in each direction
    int internalLumaWidth = ((pcYUVBuffer->yuvPlanes[0].width + 7)/8)*8;
    int internalLumaHeight = ((pcYUVBuffer->yuvPlanes[0].height + 7)/8)*8;
    int internalLumaStride = (internalLumaWidth > pcYUVBuffer->yuvPlanes[0].stride) ? internalLumaWidth : pcYUVBuffer->yuvPlanes[0].width;

    int iChromaInStride = (m_cEncCfg.m_internChromaFormat == CHROMA_444) ? internalLumaStride  : internalLumaStride >> 1;
    if( pcYUVBuffer->yuvPlanes[1].stride && pcYUVBuffer->yuvPlanes[1].stride > (internalLumaWidth >> 1) )
    {
      iChromaInStride =  pcYUVBuffer->yuvPlanes[1].stride;
    }
    if( pcYUVBuffer->yuvPlanes[2].stride && pcYUVBuffer->yuvPlanes[2].stride > (internalLumaWidth >> 1) && pcYUVBuffer->yuvPlanes[2].stride > iChromaInStride )
    {
      iChromaInStride =  pcYUVBuffer->yuvPlanes[2].stride;
    }

    for ( int i = 0; i < 3; i++ )
    {
      YUVPlane& yuvPlane = cYUVBuffer.yuvPlanes[ i ];
      yuvPlane.width     = pcYUVBuffer->yuvPlanes[i].width;
      yuvPlane.height    = pcYUVBuffer->yuvPlanes[i].height;
      yuvPlane.stride    = pcYUVBuffer->yuvPlanes[i].stride;

      if ( i > 0 )
      {
        yuvPlane.width     = (m_cEncCfg.m_internChromaFormat == CHROMA_444 ) ? internalLumaWidth  : internalLumaWidth >> 1;
        yuvPlane.height    = (m_cEncCfg.m_internChromaFormat == CHROMA_444 || m_cEncCfg.m_internChromaFormat == CHROMA_422 ) ? internalLumaHeight  : internalLumaHeight >> 1;
        yuvPlane.stride    = iChromaInStride;
      }
      else
      {
        yuvPlane.width     = internalLumaWidth;
        yuvPlane.height    = internalLumaHeight;
        yuvPlane.stride    = internalLumaStride;
      }
      const int size     = yuvPlane.stride * yuvPlane.height;
      yuvPlane.planeBuf  = ( size > 0 ) ? new int16_t[ size ] : nullptr;
    }

    xCopyAndPadInputPlane( cYUVBuffer.yuvPlanes[0].planeBuf, cYUVBuffer.yuvPlanes[0].stride, cYUVBuffer.yuvPlanes[0].width, cYUVBuffer.yuvPlanes[0].height,
                           pcYUVBuffer->yuvPlanes[0].planeBuf, pcYUVBuffer->yuvPlanes[0].stride, pcYUVBuffer->yuvPlanes[0].width, pcYUVBuffer->yuvPlanes[0].height );

    if( m_cEncCfg.m_internChromaFormat != CHROMA_400 )
    {
      xCopyAndPadInputPlane( cYUVBuffer.yuvPlanes[1].planeBuf, iChromaInStride, cYUVBuffer.yuvPlanes[1].width, cYUVBuffer.yuvPlanes[1].height,
                             pcYUVBuffer->yuvPlanes[1].planeBuf, pcYUVBuffer->yuvPlanes[1].stride, pcYUVBuffer->yuvPlanes[1].width, pcYUVBuffer->yuvPlanes[1].height );
      xCopyAndPadInputPlane( cYUVBuffer.yuvPlanes[2].planeBuf, iChromaInStride, cYUVBuffer.yuvPlanes[2].width, cYUVBuffer.yuvPlanes[2].height,
                             pcYUVBuffer->yuvPlanes[2].planeBuf, pcYUVBuffer->yuvPlanes[2].stride, pcYUVBuffer->yuvPlanes[2].width, pcYUVBuffer->yuvPlanes[2].height );
    }

    cYUVBuffer.sequenceNumber = pcYUVBuffer->sequenceNumber;
    cYUVBuffer.cts            = pcYUVBuffer->cts;
    cYUVBuffer.ctsValid       = pcYUVBuffer->ctsValid;
  }
  else
  {
    bFlush = true;
  }

  // reset AU data
  rcAccessUnit.payload.clear();
  rcAccessUnit.cts      = 0;
  rcAccessUnit.dts      = 0;
  rcAccessUnit.ctsValid  = false;
  rcAccessUnit.dtsValid  = false;
  rcAccessUnit.rap       = false;
  rcAccessUnit.sliceType = NUMBER_OF_SLICE_TYPES;
  rcAccessUnit.refPic    = false;
  rcAccessUnit.temporalLayer = 0;
  rcAccessUnit.poc   = 0;
  rcAccessUnit.status = 0;
  rcAccessUnit.infoString.clear();
  rcAccessUnit.nalUnitTypeVec.clear();
  rcAccessUnit.annexBsizeVec.clear();

  rbEncodeDone = false;

  AccessUnitList cAu;
  try
  {
    m_pEncLib->encodePicture( bFlush, cYUVBuffer, cAu, rbEncodeDone );
  }
  catch( std::exception& e )
  {
    m_cErrorString = e.what();
    return VVENC_ERR_UNSPECIFIED;
  }

  if( rbEncodeDone ){ m_bFlushed = true; }

  /* copy output AU */
  if ( !cAu.empty() )
  {
    iRet = xCopyAu( rcAccessUnit, cAu  );
  }

  /* free memory of input image */
  for ( int i = 0; i < 3; i++ )
  {
    vvenc::YUVPlane& yuvPlane = cYUVBuffer.yuvPlanes[ i ];
    if ( yuvPlane.planeBuf )
      delete [] yuvPlane.planeBuf;
  }

  return iRet;
}

int VVEncImpl::getConfig( vvenc::VVEncParameter& rcVVEncParameter ) const
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  rcVVEncParameter = m_cVVEncParameter;
  return 0;
}

int VVEncImpl::reconfig( const VVEncParameter& rcVVEncParameter )
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }
  return VVENC_ERR_NOT_SUPPORTED;
}


std::string VVEncImpl::getVersionNumber()
{
  std::string cVersion = VVENC_VERSION;
  return cVersion;
}

std::string VVEncImpl::getEncoderInfo() const
{
  std::string cEncoderInfo  = "Fraunhofer VVC Encoder ver. " VVENC_VERSION;
  cEncoderInfo += " ";
  cEncoderInfo += m_sEncoderCapabilities;
  return cEncoderInfo;
}

std::string VVEncImpl::getLastError() const
{
  return m_cErrorString;
}

std::string VVEncImpl::getErrorMsg( int nRet )
{
  std::string cErr;
  switch( nRet )
  {
  case VVENC_OK :                  cErr = "expected behavior"; break;
  case VVENC_ERR_UNSPECIFIED:      cErr = "unspecified malfunction"; break;
  case VVENC_ERR_INITIALIZE:       cErr = "encoder not initialized or tried to initialize multiple times"; break;
  case VVENC_ERR_ALLOCATE:         cErr = "internal allocation error"; break;
  case VVENC_NOT_ENOUGH_MEM:       cErr = "allocated memory to small to receive encoded data"; break;
  case VVENC_ERR_PARAMETER:        cErr = "inconsistent or invalid parameters"; break;
  case VVENC_ERR_NOT_SUPPORTED:    cErr = "unsupported request"; break;
  case VVENC_ERR_RESTART_REQUIRED: cErr = "encoder requires restart"; break;
  case VVENC_ERR_CPU:              cErr = "unsupported CPU - SSE 4.1 needed!"; break;
  default:                         cErr = "unknown ret code"; break;
  }

  return cErr;
}

int VVEncImpl::setAndRetErrorMsg( int iRet )
{
  if( m_cErrorString.empty() )
  {
    m_cErrorString = getErrorMsg(iRet);
  }

  return iRet;
}

int VVEncImpl::getNumLeadFrames() const
{
  return m_cEncCfg.m_MCTFNumLeadFrames;
}

int VVEncImpl::getNumTrailFrames() const
{
  return m_cEncCfg.m_MCTFNumTrailFrames;
}

int VVEncImpl::printConfig() const
{
  if( !m_bInitialized )       { return -1; }
  if( nullptr == m_pEncLib )  { return -1; }

  m_cEncCfg.printCfg();
  return 0;
}

int VVEncImpl::printSummary() const
{
  if( !m_bInitialized ){ return -1; }
  if( nullptr == m_pEncLib )  { return -1; }

  m_pEncLib->printSummary();
  return 0;
}

std::string VVEncImpl::getPresetParamsAsStr( int iQuality )
{
  std::stringstream css;
  vvenc::EncCfg cEncCfg;
  if( 0 != cEncCfg.initPreset( (PresetMode)iQuality ))
  {
    css << "undefined preset " << iQuality;
    return css.str();
  }

// tools
  if( cEncCfg.m_RDOQ )           { css << "RDOQ " << cEncCfg.m_RDOQ << " ";}
  if( cEncCfg.m_DepQuantEnabled ){ css << "DQ ";}
  if( cEncCfg.m_SignDataHidingEnabled ){ css << "SignBitHidingFlag ";}
  if( cEncCfg.m_alf )
  {
    css << "ALF ";
    if( cEncCfg.m_useNonLinearAlfLuma )   css << "NonLinLuma ";
    if( cEncCfg.m_useNonLinearAlfChroma ) css << "NonLinChr ";
  }
  if( cEncCfg.m_ccalf ){ css << "CCALF ";}

// vvc tools
  if( cEncCfg.m_BDOF )             { css << "BIO ";}
  if( cEncCfg.m_DMVR )             { css << "DMVR ";}
  if( cEncCfg.m_JointCbCrMode )    { css << "JointCbCr ";}
  if( cEncCfg.m_AMVRspeed )        { css << "AMVRspeed " << cEncCfg.m_AMVRspeed << " ";}
  if( cEncCfg.m_lumaReshapeEnable ){ css << "Reshape ";}
  if( cEncCfg.m_EDO )              { css << "EncDbOpt ";}
  if( cEncCfg.m_MRL )              { css << "MRL ";}
  if( cEncCfg.m_MCTF )             { css << "MCTF "; }
  if( cEncCfg.m_SMVD )             { css << "SMVD " << cEncCfg.m_SMVD << " ";}
  if( cEncCfg.m_Affine )
  {
    css << "Affine " << cEncCfg.m_Affine << " ";
    if( cEncCfg.m_PROF )           { css << "(Prof " << cEncCfg.m_PROF << " ";}
    if( cEncCfg.m_AffineType )     { css << "Type " << cEncCfg.m_AffineType << ") ";}
  }

  if( cEncCfg.m_MMVD )             { css << "MMVD " << cEncCfg.m_MMVD << " ";}
  if( cEncCfg.m_allowDisFracMMVD ) { css << "DisFracMMVD ";}

  if( cEncCfg.m_MIP )          { css << "MIP ";}
  if( cEncCfg.m_useFastMIP )   { css << "FastMIP " << cEncCfg.m_useFastMIP << " ";}
  if( cEncCfg.m_SbTMVP )       { css << "SbTMVP ";}
  if( cEncCfg.m_Geo )          { css << "Geo " << cEncCfg.m_Geo << " ";}
  if( cEncCfg.m_LFNST )        { css << "LFNST ";}

  if( cEncCfg.m_SBT )          { css << "SBT " ;}
  if( cEncCfg.m_CIIP )         { css << "CIIP ";}
  if (cEncCfg.m_ISP)           { css << "ISP "; }
  if (cEncCfg.m_TS)            { css << "TS "; }
  if (cEncCfg.m_useBDPCM)      { css << "BDPCM "; }

  // fast tools
  if( cEncCfg.m_contentBasedFastQtbt ) { css << "ContentBasedFastQtbt ";}

  return css.str();
}


/* converting sdk params to internal (wrapper) params*/
int VVEncImpl::xCheckParameter( const vvenc::VVEncParameter& rcSrc, std::string& rcErrorString ) const
{
  // check src params
  ROTPARAMS( rcSrc.qp < 0 || rcSrc.qp > 51,                                           "qp must be between 0 - 51."  );

  ROTPARAMS( ( rcSrc.width == 0 )   || ( rcSrc.height == 0 ),                         "specify input picture dimension"  );

  ROTPARAMS( rcSrc.temporalScale != 1 && rcSrc.temporalScale != 2 && rcSrc.temporalScale != 4 && rcSrc.temporalScale != 1001,   "TemporalScale has to be 1, 2, 4 or 1001" );

  double dFPS = (double)rcSrc.temporalRate / (double)rcSrc.temporalScale;
  ROTPARAMS( dFPS < 1.0 || dFPS > 120,                                                      "fps specified by temporal rate and scale must result in 1Hz < fps < 120Hz" );

  ROTPARAMS( rcSrc.ticksPerSecond <= 0 || rcSrc.ticksPerSecond > 27000000,            "TicksPerSecond must be in range from 1 to 27000000" );
  ROTPARAMS( (rcSrc.ticksPerSecond < 90000) && (rcSrc.ticksPerSecond*rcSrc.temporalScale)%rcSrc.temporalRate,        "TicksPerSecond should be a multiple of FrameRate/Framscale" );

  ROTPARAMS( rcSrc.threadCount < 0,                                                      "ThreadCount must be >= 0" );

  ROTPARAMS( rcSrc.idrPeriod < 0,                                                        "IDR period (in frames) must be >= 0" );
  ROTPARAMS( rcSrc.idrPeriodSec < 0,                                                     "IDR period (in seconds) must be > 0" );

  ROTPARAMS( rcSrc.temporalRate  <= 0,                                                   "TemporalRate must be > 0" );
  ROTPARAMS( rcSrc.temporalScale <= 0,                                                   "TemporalScale must be > 0" );

  ROTPARAMS( rcSrc.gopSize != 1 && rcSrc.gopSize != 16 && rcSrc.gopSize != 32,          "GOP size 1, 16, 32 supported" );

  if( 1 != rcSrc.gopSize && ( rcSrc.idrPeriod > 0  ))
  {
    ROTPARAMS( (rcSrc.decodingRefreshType == DRT_IDR || rcSrc.decodingRefreshType == DRT_CRA )&& (0 != rcSrc.idrPeriod % rcSrc.gopSize),          "IDR period must be multiple of GOPSize" );
  }

  ROTPARAMS( rcSrc.perceptualQPA < 0 || rcSrc.perceptualQPA > 5,                      "Perceptual QPA must be in the range 0 - 5" );

  ROTPARAMS( rcSrc.profile != Profile::MAIN_10 && rcSrc.profile != Profile::MAIN_10_STILL_PICTURE && rcSrc.profile != Profile::PROFILE_AUTO, "unsupported profile, use main_10, main_10_still_picture or auto" );

  ROTPARAMS( (rcSrc.quality < 0 || rcSrc.quality > 4) && rcSrc.quality != 255,        "quality must be between 0 - 4  (0: faster, 1: fast, 2: medium, 3: slow, 4: slower)" );
  ROTPARAMS( rcSrc.targetBitRate < 0 || rcSrc.targetBitRate > 100000000,              "TargetBitrate must be between 0 - 100000000" );
  ROTPARAMS( rcSrc.targetBitRate == 0 && rcSrc.numPasses != 1,                        "Only single pass encoding supported, when rate control is disabled" );
  ROTPARAMS( rcSrc.numPasses < 1 || rcSrc.numPasses > 2,                              "Only one pass or two pass encoding supported"  );

  ROTPARAMS( rcSrc.msgLevel < 0 || rcSrc.msgLevel > DETAILS,                          "log message level range 0 - 6" );

  ROTPARAMS( rcSrc.segmentMode != SEG_OFF && rcSrc.maxFrames < MCTF_RANGE,            "When using segment parallel encoding more then 2 frames have to be encoded" );

  if( 0 == rcSrc.targetBitRate )
  {
    ROTPARAMS( rcSrc.useHrdParametersPresent,              "hrdParameters present requires rate control" );
    ROTPARAMS( rcSrc.useBufferingPeriodSEIEnabled,         "bufferingPeriod SEI enabled requires rate control" );
    ROTPARAMS( rcSrc.usePictureTimingSEIEnabled,           "pictureTiming SEI enabled requires rate control" );
  }

  ROTPARAMS( rcSrc.inputBitDepth != 8 && rcSrc.inputBitDepth != 10,                   "Input bitdepth must be 8 or 10 bit" );
  ROTPARAMS( rcSrc.internalBitDepth != 8 && rcSrc.internalBitDepth != 10,             "Internal bitdepth must be 8 or 10 bit" );

  return 0;
}


/* converting sdk params to internal (wrapper) params*/
int VVEncImpl::xCheckParameter( const EncCfg& rcSrc, std::string& rcErrorString ) const
{
  // check src params
  ROTPARAMS( rcSrc.m_QP < 0 || rcSrc.m_QP > 51,                                             "qp must be between 0 - 51."  );

  ROTPARAMS( ( rcSrc.m_SourceWidth == 0 )   || ( rcSrc.m_SourceHeight == 0 ),               "specify input picture dimension"  );

  ROTPARAMS( rcSrc.m_FrameRate < 1 || rcSrc.m_FrameRate > 120,                              "fps specified by temporal rate and scale must result in 1Hz < fps < 120Hz" );

  ROTPARAMS( rcSrc.m_TicksPerSecond <= 0 || rcSrc.m_TicksPerSecond > 27000000,              "TicksPerSecond must be in range from 1 to 27000000" );

  int temporalRate   = rcSrc.m_FrameRate;
  int temporalScale  = 1;

  switch( rcSrc.m_FrameRate )
  {
  case 23: temporalRate = 24000; temporalScale = 1001; break;
  case 29: temporalRate = 30000; temporalScale = 1001; break;
  case 59: temporalRate = 60000; temporalScale = 1001; break;
  default: break;
  }

  ROTPARAMS( (rcSrc.m_TicksPerSecond < 90000) && (rcSrc.m_TicksPerSecond*temporalScale)%temporalRate, "TicksPerSecond should be a multiple of FrameRate/Framscale" );

  ROTPARAMS( rcSrc.m_numWppThreads < 0,                                                      "numWppThreads must be >= 0" );

  ROTPARAMS( rcSrc.m_IntraPeriod < 0,                                                        "IDR period (in frames) must be >= 0" );
  ROTPARAMS( rcSrc.m_IntraPeriodSec < 0,                                                     "IDR period (in seconds) must be > 0" );

  ROTPARAMS( rcSrc.m_GOPSize != 1 && rcSrc.m_GOPSize != 16 && rcSrc.m_GOPSize != 32,         "GOP size 1, 16, 32 supported" );

  if( 1 != rcSrc.m_GOPSize && ( rcSrc.m_IntraPeriod > 0  ))
  {
    ROTPARAMS( (rcSrc.m_DecodingRefreshType == DRT_IDR || rcSrc.m_DecodingRefreshType == DRT_CRA )&& (0 != rcSrc.m_IntraPeriod % rcSrc.m_GOPSize),          "IDR period must be multiple of GOPSize" );
  }

  ROTPARAMS( rcSrc.m_usePerceptQPA < 0 || rcSrc.m_usePerceptQPA > 5,                        "Perceptual QPA must be in the range 0 - 5" );

  ROTPARAMS( rcSrc.m_profile != Profile::MAIN_10 && rcSrc.m_profile != Profile::MAIN_10_STILL_PICTURE && rcSrc.m_profile != Profile::PROFILE_AUTO, "unsupported profile, use main_10, main_10_still_picture or auto" );

  ROTPARAMS( rcSrc.m_RCTargetBitrate < 0 || rcSrc.m_RCTargetBitrate > 100000000,           "TargetBitrate must be between 0 - 100000000" );
  ROTPARAMS( rcSrc.m_RCTargetBitrate == 0 && rcSrc.m_RCNumPasses != 1,                     "Only single pass encoding supported, when rate control is disabled" );
  ROTPARAMS( rcSrc.m_RCNumPasses < 1 || rcSrc.m_RCNumPasses > 2,                           "Only one pass or two pass encoding supported"  );

  ROTPARAMS( rcSrc.m_verbosity < 0 || rcSrc.m_verbosity > DETAILS,                         "log message level range 0 - 6" );

  ROTPARAMS( rcSrc.m_SegmentMode != SEG_OFF && rcSrc.m_framesToBeEncoded < MCTF_RANGE,     "When using segment parallel encoding more then 2 frames have to be encoded" );

  if( 0 == rcSrc.m_RCTargetBitrate )
  {
    ROTPARAMS( rcSrc.m_hrdParametersPresent,              "hrdParameters present requires rate control" );
    ROTPARAMS( rcSrc.m_bufferingPeriodSEIEnabled,         "bufferingPeriod SEI enabled requires rate control" );
    ROTPARAMS( rcSrc.m_pictureTimingSEIEnabled,           "pictureTiming SEI enabled requires rate control" );
  }

  ROTPARAMS( rcSrc.m_inputBitDepth[0] != 8 && rcSrc.m_inputBitDepth[0] != 10,                   "Input bitdepth must be 8 or 10 bit" );
  ROTPARAMS( rcSrc.m_internalBitDepth[0] != 8 && rcSrc.m_internalBitDepth[0] != 10,             "Internal bitdepth must be 8 or 10 bit" );

  return 0;
}

int VVEncImpl::xInitLibCfg( const VVEncParameter& rcVVEncParameter, EncCfg& rcEncCfg )
{
  rcEncCfg.m_verbosity = std::min( (int)rcVVEncParameter.msgLevel, (int)vvenc::DETAILS);

  rcEncCfg.m_SourceWidth               = rcVVEncParameter.width;
  rcEncCfg.m_SourceHeight              = rcVVEncParameter.height;
  rcEncCfg.m_QP                        = rcVVEncParameter.qp;

  rcEncCfg.m_usePerceptQPA             = rcVVEncParameter.perceptualQPA;

  rcEncCfg.m_AccessUnitDelimiter       = rcVVEncParameter.useAccessUnitDelimiter;
  rcEncCfg.m_hrdParametersPresent      = rcVVEncParameter.useHrdParametersPresent;
  rcEncCfg.m_bufferingPeriodSEIEnabled = rcVVEncParameter.useBufferingPeriodSEIEnabled;
  rcEncCfg.m_pictureTimingSEIEnabled   = rcVVEncParameter.usePictureTimingSEIEnabled;

  if(  rcVVEncParameter.targetBitRate )
  {
    rcEncCfg.m_RCRateControlMode     = 2;
    rcEncCfg.m_RCNumPasses           = rcVVEncParameter.numPasses;
    rcEncCfg.m_RCTargetBitrate       = rcVVEncParameter.targetBitRate;
    rcEncCfg.m_RCKeepHierarchicalBit = 2;
    rcEncCfg.m_RCUseLCUSeparateModel = 1;
    rcEncCfg.m_RCInitialQP           = 0;
    rcEncCfg.m_RCForceIntraQP        = 0;
  }
  else
  {
    rcEncCfg.m_RCKeepHierarchicalBit = 0;
    rcEncCfg.m_RCRateControlMode     = 0;
    rcEncCfg.m_RCTargetBitrate       = 0;
  }

  rcEncCfg.m_inputBitDepth[0]    = rcVVEncParameter.inputBitDepth;
  rcEncCfg.m_inputBitDepth[1]    = rcVVEncParameter.inputBitDepth;
  rcEncCfg.m_internalBitDepth[0] = rcVVEncParameter.internalBitDepth;
  rcEncCfg.m_internalBitDepth[1] = rcVVEncParameter.internalBitDepth;;

  rcEncCfg.m_numWppThreads = rcVVEncParameter.threadCount;
  if( rcVVEncParameter.threadCount > 0 )
  {
      rcEncCfg.m_ensureWppBitEqual = 1;
  }

  rcEncCfg.m_FrameRate                           = rcVVEncParameter.temporalRate / rcVVEncParameter.temporalScale;
  rcEncCfg.m_framesToBeEncoded                   = rcVVEncParameter.maxFrames;

  //======== Coding Structure =============
  rcEncCfg.m_GOPSize                             = rcVVEncParameter.gopSize;
  rcEncCfg.m_InputQueueSize                      = 0;

  if( rcVVEncParameter.idrPeriod >= rcVVEncParameter.gopSize  )
  {
    rcEncCfg.m_IntraPeriod                       = rcVVEncParameter.idrPeriod;
  }
  else // use idrPeriodSec
  {
    if ( rcEncCfg.m_FrameRate % rcVVEncParameter.gopSize == 0 )
    {
      rcEncCfg.m_IntraPeriod = rcEncCfg.m_FrameRate * rcVVEncParameter.idrPeriodSec;
    }
    else
    {
      int iIDRPeriod  = (rcEncCfg.m_FrameRate * rcVVEncParameter.idrPeriodSec);
      if( iIDRPeriod < rcVVEncParameter.gopSize )
      {
        iIDRPeriod = rcVVEncParameter.gopSize;
      }

      int iDiff = iIDRPeriod % rcVVEncParameter.gopSize;
      if( iDiff < rcVVEncParameter.gopSize >> 1 )
      {
        rcEncCfg.m_IntraPeriod = iIDRPeriod - iDiff;
      }
      else
      {
        rcEncCfg.m_IntraPeriod = iIDRPeriod + rcVVEncParameter.gopSize - iDiff;
      }
    }
  }

  if( rcVVEncParameter.decodingRefreshType == DRT_IDR )
  {
    rcEncCfg.m_DecodingRefreshType                 = 2;  // Random Accesss 0:none, 1:CRA, 2:IDR, 3:Recovery Point SEI
  }
  else if( rcVVEncParameter.decodingRefreshType == DRT_CRA )
  {
    rcEncCfg.m_DecodingRefreshType                 = 1;  // Random Accesss 0:none, 1:CRA, 2:IDR, 3:Recovery Point SEI
  }
  else if( rcVVEncParameter.decodingRefreshType == DRT_RECOVERY_POINT_SEI )
  {
    rcEncCfg.m_DecodingRefreshType                 = 3;  // Random Accesss 0:none, 1:CRA, 2:IDR, 3:Recovery Point SEI
  }
  else
  {
    rcEncCfg.m_DecodingRefreshType                 = 0;  // Random Accesss 0:none, 1:CRA, 2:IDR, 3:Recovery Point SEI
  }

  //======== Profile ================
  rcEncCfg.m_profile   = (vvenc::Profile)rcVVEncParameter.profile;
  rcEncCfg.m_levelTier = (vvenc::Tier)rcVVEncParameter.tier;
  rcEncCfg.m_level     = (vvenc::Level)rcVVEncParameter.level;

  rcEncCfg.m_bitDepthConstraintValue = 10;
  rcEncCfg.m_rewriteParamSets        = true;
  rcEncCfg.m_internChromaFormat      = vvenc::CHROMA_420;

  if( 0 != rcEncCfg.initPreset( (PresetMode)rcVVEncParameter.quality  ) )
  {
    std::stringstream css;
    css << "undefined quality preset " << rcVVEncParameter.quality << " quality must be between 0 - 4.";
    m_cErrorString  = css.str();
    return VVENC_ERR_PARAMETER;
  }

  if( rcVVEncParameter.segmentMode != SEG_OFF )
  {
    if( rcEncCfg.m_MCTF )
    {
      switch( rcVVEncParameter.segmentMode )
      {
        case SEG_FIRST:
          rcEncCfg.m_MCTFNumLeadFrames  = 0;
          rcEncCfg.m_MCTFNumTrailFrames = MCTF_RANGE;
          break;
        case SEG_MID:
          rcEncCfg.m_MCTFNumLeadFrames  = MCTF_RANGE;
          rcEncCfg.m_MCTFNumTrailFrames = MCTF_RANGE;
          break;
        case SEG_LAST:
          rcEncCfg.m_MCTFNumLeadFrames  = MCTF_RANGE;
          rcEncCfg.m_MCTFNumTrailFrames = 0;
          break;
        default:
          rcEncCfg.m_MCTFNumLeadFrames  = 0;
          rcEncCfg.m_MCTFNumTrailFrames = 0;
          break;
      }
    }
  }

  if ( rcEncCfg.initCfgParameter() )
  {
    m_cErrorString = "cannot init internal config parameter";
    return VVENC_ERR_PARAMETER;
  }

  rcEncCfg.printCfg();

  return 0;
}

int VVEncImpl::xCopyAndPadInputPlane( int16_t* pDes, const int iDesStride, const int iDesWidth, const int iDesHeight,
                                      const int16_t* pSrc, const int iSrcStride, const int iSrcWidth, const int iSrcHeight )
{
  if( iSrcStride == iDesStride )
  {
    ::memcpy( pDes, pSrc, iSrcStride * iSrcHeight * sizeof(int16_t) );
  }
  else
  {
    for( int y = 0; y < iSrcHeight; y++ )
    {
      ::memcpy( pDes + y*iDesStride, pSrc + y*iSrcStride, iSrcWidth * sizeof(int16_t) );
    }
  }

  if( iSrcWidth < iDesWidth )
  {
    for( int y = 0; y < iSrcHeight; y++ )
    {
      for( int x = iSrcWidth; x < iDesWidth; x++ )
      {
        pDes[ x + y*iDesStride] = pDes[ iSrcWidth - 1 + y*iDesStride];
      }
    }
  }

  if( iSrcHeight < iDesHeight )
  {
    for( int y = iSrcHeight; y < iDesHeight; y++ )
    {
      ::memcpy( pDes + y*iDesStride, pDes + (iSrcHeight-1)*iDesStride, iDesWidth * sizeof(int16_t) );
    }
  }

  return 0;
}


int VVEncImpl::xCopyAu( AccessUnit& rcAccessUnit, const vvenc::AccessUnitList& rcAuList )
{
  rcAccessUnit.rap = false;

  std::vector<uint32_t> annexBsizes;

  /* copy output AU */
  if ( ! rcAuList.empty() )
  {
    uint32_t sizeSum = 0;
    for (vvenc::AccessUnitList::const_iterator it = rcAuList.begin(); it != rcAuList.end(); it++)
    {
      const vvenc::NALUnitEBSP& nalu = **it;
      uint32_t size = 0; /* size of annexB unit in bytes */

      if (it == rcAuList.begin() ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_DCI ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_SPS ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_VPS ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_PPS ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_PREFIX_APS ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_SUFFIX_APS )
      {
        size += 4;
      }
      else
      {
        size += 3;
      }
      size += uint32_t(nalu.m_nalUnitData.str().size());
      sizeSum += size;
      annexBsizes.push_back( size );
    }

    rcAccessUnit.payload.resize( sizeSum );
    uint32_t iUsedSize = 0;
    for (vvenc::AccessUnitList::const_iterator it = rcAuList.begin(); it != rcAuList.end(); it++)
    {
      const vvenc::NALUnitEBSP& nalu = **it;
      static const uint8_t start_code_prefix[] = {0,0,0,1};
      if (it == rcAuList.begin() ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_DCI ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_SPS ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_VPS ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_PPS ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_PREFIX_APS ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_SUFFIX_APS )
      {
        /* From AVC, When any of the following conditions are fulfilled, the
         * zero_byte syntax element shall be present:
         *  - the nal_unit_type within the nal_unit() is equal to 7 (sequence
         *    parameter set) or 8 (picture parameter set),
         *  - the byte stream NAL unit syntax structure contains the first NAL
         *    unit of an access unit in decoding order, as specified by subclause
         *    7.4.1.2.3.
         */
        ::memcpy( rcAccessUnit.payload.data() + iUsedSize, reinterpret_cast<const char*>(start_code_prefix), 4 );
        iUsedSize += 4;
      }
      else
      {
        ::memcpy( rcAccessUnit.payload.data() + iUsedSize, reinterpret_cast<const char*>(start_code_prefix+1), 3 );
        iUsedSize += 3;
      }
      uint32_t nalDataSize = uint32_t(nalu.m_nalUnitData.str().size()) ;
      ::memcpy( rcAccessUnit.payload.data() + iUsedSize, nalu.m_nalUnitData.str().c_str() , nalDataSize );
      iUsedSize += nalDataSize;

      if( nalu.m_nalUnitType == vvenc::NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_CODED_SLICE_IDR_N_LP ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_CODED_SLICE_CRA ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_CODED_SLICE_GDR )
      {
        rcAccessUnit.rap = true;
      }

      rcAccessUnit.nalUnitTypeVec.push_back( nalu.m_nalUnitType );
    }

    if( iUsedSize != sizeSum  )
    {
      return VVENC_NOT_ENOUGH_MEM;
    }

    rcAccessUnit.annexBsizeVec   = annexBsizes;
    rcAccessUnit.ctsValid        = rcAuList.ctsValid;
    rcAccessUnit.dtsValid        = rcAuList.dtsValid;
    rcAccessUnit.cts             = rcAuList.cts;
    rcAccessUnit.dts             = rcAuList.dts;
    rcAccessUnit.sliceType       = (SliceType)rcAuList.sliceType;
    rcAccessUnit.refPic          = rcAuList.refPic;
    rcAccessUnit.temporalLayer   = rcAuList.temporalLayer;
    rcAccessUnit.poc             = rcAuList.poc;
    rcAccessUnit.infoString      = rcAuList.InfoString;
    rcAccessUnit.status          = rcAuList.status;
  }

  return 0;
}


///< set message output function for encoder lib. if not set, no messages will be printed.
void VVEncImpl::registerMsgCbf( std::function<void( int, const char*, va_list )> msgFnc )
{
  g_msgFnc = msgFnc;
}

///< tries to set given simd extensions used. if not supported by cpu, highest possible extension level will be set and returned.
std::string VVEncImpl::setSIMDExtension( const std::string& simdId )
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

///< checks if library has tracing supported enabled (see ENABLE_TRACING).
bool VVEncImpl::isTracingEnabled()
{
#if ENABLE_TRACING
  return true;
#else
  return false;
#endif
}

///< creates compile info string containing OS, Compiler and Bit-depth (e.g. 32 or 64 bit).
std::string VVEncImpl::getCompileInfoString()
{
  char convBuf[ 256 ];
  std::string compileInfo;
  snprintf( convBuf, sizeof( convBuf ), NVM_ONOS );      compileInfo += convBuf;
  snprintf( convBuf, sizeof( convBuf ), NVM_COMPILEDBY); compileInfo += convBuf;
  snprintf( convBuf, sizeof( convBuf ), NVM_BITS );      compileInfo += convBuf;
  return compileInfo;
}

///< decode bitstream with limited build in decoder
void VVEncImpl::decodeBitstream( const std::string& FileName)
{
  FFwdDecoder ffwdDecoder;
  Picture cPicture; cPicture.poc=-8000;

  if( tryDecodePicture( &cPicture, -1, FileName, ffwdDecoder, nullptr, false, cPicture.poc, false ))
  {
    msg( ERROR, "decoding failed");
    THROW("error decoding");
  }
}

} // namespace

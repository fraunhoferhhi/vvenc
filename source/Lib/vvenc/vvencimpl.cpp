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

#include "vvenc/Nal.h"
#include "vvenc/version.h"
#include "CommonLib/CommonDef.h"


namespace vvenc {

#define ROTPARAMS(x, message) if(x) { rcErrorString = message; return VVENC_ERR_PARAMETER;}

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
  std::string curSimd = vvenc::setSIMDExtension( simdOpt );

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
  m_cEncoderIf.initEncoderLib( m_cEncCfg );

  m_bInitialized = true;
  m_bFlushed     = false;
  return VVENC_OK;
}

int VVEncImpl::init( const EncCfg& rcEncCfg )
{
  if( m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  // Set SIMD extension in case if it hasn't been done before, otherwise it simply reuses the current state
  std::string simdOpt;
  std::string curSimd = vvenc::setSIMDExtension( simdOpt );

  int iRet = xCheckParameter( rcEncCfg, m_cErrorString );
  if( 0 != iRet ) { return iRet; }

  std::stringstream cssCap;
  cssCap << getCompileInfoString() << "[SIMD=" << curSimd <<"]";
  m_sEncoderCapabilities = cssCap.str();

  m_cEncCfg = rcEncCfg;

  m_cEncCfg.printCfg();

  // initialize the encoder
  m_cEncoderIf.initEncoderLib( m_cEncCfg );

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

  m_cEncoderIf.initPass( pass );

  m_bFlushed = false;
  return VVENC_OK;
}

int VVEncImpl::uninit()
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  m_cEncoderIf.printSummary();
  m_cEncoderIf.uninitEncoderLib();

  m_bInitialized = false;
  m_bFlushed     = false;
  return VVENC_OK;
}

bool VVEncImpl::isInitialized() const
{
  return m_bInitialized;
}


int VVEncImpl::encode( InputPicture* pcInputPicture, VvcAccessUnit& rcVvcAccessUnit )
{
  if( !m_bInitialized )                { return VVENC_ERR_INITIALIZE; }
  if( 0 == rcVvcAccessUnit.m_iBufSize ){ m_cErrorString = "AccessUnit BufferSize is 0"; return VVENC_NOT_ENOUGH_MEM; }
  if( m_bFlushed )                     { m_cErrorString = "encoder already flushed"; return VVENC_ERR_RESTART_REQUIRED; }

  int iRet= VVENC_OK;

  if( !pcInputPicture )
  {
    m_cErrorString = "InputPicture is null";
    return VVENC_ERR_UNSPECIFIED;
  }

  if( pcInputPicture->m_cPicBuffer.m_pvY == nullptr || pcInputPicture->m_cPicBuffer.m_pvU == nullptr || pcInputPicture->m_cPicBuffer.m_pvV == nullptr )
  {
    m_cErrorString = "InputPicture: invalid input buffers";
    return VVENC_ERR_UNSPECIFIED;
  }

  if( pcInputPicture->m_cPicBuffer.m_iWidth != this->m_cEncCfg.m_SourceWidth )
  {
    m_cErrorString = "InputPicture: unsupported width";
    return VVENC_ERR_UNSPECIFIED;
  }

  if( pcInputPicture->m_cPicBuffer.m_iHeight != this->m_cEncCfg.m_SourceHeight )
  {
    m_cErrorString = "InputPicture: unsupported height";
    return VVENC_ERR_UNSPECIFIED;
  }

  if( pcInputPicture->m_cPicBuffer.m_iWidth > pcInputPicture->m_cPicBuffer.m_iStride )
  {
    m_cErrorString = "InputPicture: unsupported width stride combination";
    return VVENC_ERR_UNSPECIFIED;
  }

  if( pcInputPicture->m_cPicBuffer.m_iCStride && pcInputPicture->m_cPicBuffer.m_iWidth/2 > pcInputPicture->m_cPicBuffer.m_iCStride )
  {
    m_cErrorString = "InputPicture: unsupported width cstride combination";
    return VVENC_ERR_UNSPECIFIED;
  }

  if( pcInputPicture->m_cPicBuffer.m_iBitDepth < 10 || pcInputPicture->m_cPicBuffer.m_iBitDepth > 16 )
  {
    std::stringstream css;
    css << "InputPicture: unsupported input BitDepth " <<  pcInputPicture->m_cPicBuffer.m_iBitDepth  << ". must be 10 <= BitDepth <= 16";
    m_cErrorString = css.str();
    return VVENC_ERR_UNSPECIFIED;
  }

  // we know that the internal buffer requires to be a multiple of 8 in each direction
  int internalLumaWidth = ((pcInputPicture->m_cPicBuffer.m_iWidth + 7)/8)*8;
  int internalLumaHeight = ((pcInputPicture->m_cPicBuffer.m_iHeight + 7)/8)*8;
  int internalLumaStride = (internalLumaWidth > pcInputPicture->m_cPicBuffer.m_iStride) ? internalLumaWidth : pcInputPicture->m_cPicBuffer.m_iStride;

  int iChromaInStride = internalLumaStride >> 1;
  if( pcInputPicture->m_cPicBuffer.m_iCStride && pcInputPicture->m_cPicBuffer.m_iCStride > (internalLumaWidth >> 1) )
  {
    iChromaInStride =  pcInputPicture->m_cPicBuffer.m_iCStride;
  }

  YUVBuffer cYUVBuffer;
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
                         (int16_t*)pcInputPicture->m_cPicBuffer.m_pvY, pcInputPicture->m_cPicBuffer.m_iStride, pcInputPicture->m_cPicBuffer.m_iWidth, pcInputPicture->m_cPicBuffer.m_iHeight, 0 );
  xCopyAndPadInputPlane( cYUVBuffer.yuvPlanes[1].planeBuf, iChromaInStride, cYUVBuffer.yuvPlanes[1].width, cYUVBuffer.yuvPlanes[1].height,
                         (int16_t*)pcInputPicture->m_cPicBuffer.m_pvU, iChromaInStride, pcInputPicture->m_cPicBuffer.m_iWidth>>1, pcInputPicture->m_cPicBuffer.m_iHeight>>1, 0 );
  xCopyAndPadInputPlane( cYUVBuffer.yuvPlanes[2].planeBuf, iChromaInStride, cYUVBuffer.yuvPlanes[2].width, cYUVBuffer.yuvPlanes[2].height,
                         (int16_t*)pcInputPicture->m_cPicBuffer.m_pvV, iChromaInStride, pcInputPicture->m_cPicBuffer.m_iWidth>>1, pcInputPicture->m_cPicBuffer.m_iHeight>>1, 0 );


  cYUVBuffer.sequenceNumber = pcInputPicture->m_cPicBuffer.m_uiSequenceNumber;
  if( pcInputPicture->m_cPicBuffer.m_bCtsValid )
  {
    cYUVBuffer.cts = pcInputPicture->m_cPicBuffer.m_uiCts;
    cYUVBuffer.ctsValid = true;
  }

  AccessUnit cAu;
  bool encDone = false;

  m_cEncoderIf.encodePicture( false, cYUVBuffer, cAu, encDone );

  /* copy output AU */
  rcVvcAccessUnit.m_iUsedSize = 0;
  if ( !cAu.empty() )
  {
    iRet = xCopyAu( rcVvcAccessUnit, cAu  );
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

int VVEncImpl::flush( VvcAccessUnit& rcVvcAccessUnit )
{
  if( !m_bInitialized )                { return VVENC_ERR_INITIALIZE; }
  if( 0 == rcVvcAccessUnit.m_iBufSize ){ m_cErrorString = "AccessUnit BufferSize is 0"; return VVENC_NOT_ENOUGH_MEM; }

  YUVBuffer cYUVBuffer;
  AccessUnit cAu;

  /* encode till next output AU done */
  while( !m_bFlushed && cAu.empty() )
  {
    m_cEncoderIf.encodePicture( true, cYUVBuffer, cAu, m_bFlushed );
  }

  /* copy next output AU */
  rcVvcAccessUnit.m_iUsedSize = 0;
  int iRet                    = VVENC_OK;
  if( !cAu.empty() )
  {
    iRet = xCopyAu( rcVvcAccessUnit, cAu  );
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
  ROTPARAMS( rcSrc.m_iQp < 0 || rcSrc.m_iQp > 51,                                           "qp must be between 0 - 51."  );

  ROTPARAMS( ( rcSrc.m_iWidth == 0 )   || ( rcSrc.m_iHeight == 0 ),                         "specify input picture dimension"  );

  ROTPARAMS( rcSrc.m_iTemporalScale != 1 && rcSrc.m_iTemporalScale != 2 && rcSrc.m_iTemporalScale != 4 && rcSrc.m_iTemporalScale != 1001,   "TemporalScale has to be 1, 2, 4 or 1001" );

  double dFPS = (double)rcSrc.m_iTemporalRate / (double)rcSrc.m_iTemporalScale;
  ROTPARAMS( dFPS < 1.0 || dFPS > 120,                                                      "fps specified by temporal rate and scale must result in 1Hz < fps < 120Hz" );

  ROTPARAMS( rcSrc.m_iTicksPerSecond <= 0 || rcSrc.m_iTicksPerSecond > 27000000,            "TicksPerSecond must be in range from 1 to 27000000" );
  ROTPARAMS( (rcSrc.m_iTicksPerSecond < 90000) && (rcSrc.m_iTicksPerSecond*rcSrc.m_iTemporalScale)%rcSrc.m_iTemporalRate,        "TicksPerSecond should be a multiple of FrameRate/Framscale" );

  ROTPARAMS( rcSrc.m_iThreadCount < 0,                                                      "ThreadCount must be >= 0" );

  ROTPARAMS( rcSrc.m_iIDRPeriod < 0,                                                        "IDR period (in frames) must be >= 0" );
  ROTPARAMS( rcSrc.m_iIDRPeriodSec < 0,                                                     "IDR period (in seconds) must be > 0" );

  ROTPARAMS( rcSrc.m_iTemporalRate  <= 0,                                                   "TemporalRate must be > 0" );
  ROTPARAMS( rcSrc.m_iTemporalScale <= 0,                                                   "TemporalScale must be > 0" );

  ROTPARAMS( rcSrc.m_iGopSize != 1 && rcSrc.m_iGopSize != 16 && rcSrc.m_iGopSize != 32,     "GOP size 1, 16, 32 supported" );

  if( 1 != rcSrc.m_iGopSize && ( rcSrc.m_iIDRPeriod > 0  ))
  {
    ROTPARAMS( (rcSrc.m_eDecodingRefreshType == DRT_IDR || rcSrc.m_eDecodingRefreshType == DRT_CRA )&& (0 != rcSrc.m_iIDRPeriod % rcSrc.m_iGopSize),          "IDR period must be multiple of GOPSize" );
  }

  ROTPARAMS( rcSrc.m_iPerceptualQPA < 0 || rcSrc.m_iPerceptualQPA > 5,                      "Perceptual QPA must be in the range 0 - 5" );

  ROTPARAMS( rcSrc.m_eProfile != Profile::Name::MAIN_10 && rcSrc.m_eProfile != Profile::Name::MAIN_10_STILL_PICTURE && rcSrc.m_eProfile != Profile::Name::AUTO, "unsupported profile, use main_10, main_10_still_picture or auto" );

  ROTPARAMS( (rcSrc.m_iQuality < 0 || rcSrc.m_iQuality > 4) && rcSrc.m_iQuality != 255,     "quality must be between 0 - 4  (0: faster, 1: fast, 2: medium, 3: slow, 4: slower)" );
  ROTPARAMS( rcSrc.m_iTargetBitRate < 0 || rcSrc.m_iTargetBitRate > 100000000,              "TargetBitrate must be between 0 - 100000000" );
  ROTPARAMS( rcSrc.m_iTargetBitRate == 0 && rcSrc.m_iNumPasses != 1,                        "Only single pass encoding supported, when rate control is disabled" );
  ROTPARAMS( rcSrc.m_iNumPasses < 1 || rcSrc.m_iNumPasses > 2,                              "Only one pass or two pass encoding supported"  );

  ROTPARAMS( rcSrc.m_eMsgLevel < 0 || rcSrc.m_eMsgLevel > DETAILS,                          "log message level range 0 - 6" );

  ROTPARAMS( rcSrc.m_eSegMode != SEG_OFF && rcSrc.m_iMaxFrames < MCTF_RANGE,            "When using segment parallel encoding more then 2 frames have to be encoded" );

  if( 0 == rcSrc.m_iTargetBitRate )
  {
    ROTPARAMS( rcSrc.m_bHrdParametersPresent,              "hrdParameters present requires rate control" );
    ROTPARAMS( rcSrc.m_bBufferingPeriodSEIEnabled,         "bufferingPeriod SEI enabled requires rate control" );
    ROTPARAMS( rcSrc.m_bPictureTimingSEIEnabled,           "pictureTiming SEI enabled requires rate control" );
  }

  ROTPARAMS( rcSrc.m_iInputBitDepth != 8 && rcSrc.m_iInputBitDepth != 10,                   "Input bitdepth must be 8 or 10 bit" );
  ROTPARAMS( rcSrc.m_iInternalBitDepth != 8 && rcSrc.m_iInternalBitDepth != 10,             "Internal bitdepth must be 8 or 10 bit" );

  return 0;
}


/* converting sdk params to internal (wrapper) params*/
int VVEncImpl::xCheckParameter( const EncCfg& rcSrc, std::string& rcErrorString ) const
{
  // check src params
  ROTPARAMS( rcSrc.m_QP < 0 || rcSrc.m_QP > 51,                                             "qp must be between 0 - 51."  );

  ROTPARAMS( ( rcSrc.m_SourceWidth == 0 )   || ( rcSrc.m_SourceHeight == 0 ),                         "specify input picture dimension"  );

  ROTPARAMS( rcSrc.m_FrameRate < 1 || rcSrc.m_FrameRate > 120,                                                      "fps specified by temporal rate and scale must result in 1Hz < fps < 120Hz" );

  ROTPARAMS( rcSrc.m_TicksPerSecond <= 0 || rcSrc.m_TicksPerSecond > 27000000,            "TicksPerSecond must be in range from 1 to 27000000" );

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

  ROTPARAMS( rcSrc.m_profile != Profile::MAIN_10 && rcSrc.m_profile != Profile::MAIN_10_STILL_PICTURE && rcSrc.m_profile != Profile::AUTO, "unsupported profile, use main_10, main_10_still_picture or auto" );

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
  rcEncCfg.m_verbosity = std::min( (int)rcVVEncParameter.m_eMsgLevel, (int)vvenc::DETAILS);

  rcEncCfg.m_SourceWidth               = rcVVEncParameter.m_iWidth;
  rcEncCfg.m_SourceHeight              = rcVVEncParameter.m_iHeight;
  rcEncCfg.m_QP                        = rcVVEncParameter.m_iQp;

  rcEncCfg.m_usePerceptQPA             = rcVVEncParameter.m_iPerceptualQPA;

  rcEncCfg.m_AccessUnitDelimiter       = rcVVEncParameter.m_bAccessUnitDelimiter;
  rcEncCfg.m_hrdParametersPresent      = rcVVEncParameter.m_bHrdParametersPresent;
  rcEncCfg.m_bufferingPeriodSEIEnabled = rcVVEncParameter.m_bBufferingPeriodSEIEnabled;
  rcEncCfg.m_pictureTimingSEIEnabled   = rcVVEncParameter.m_bPictureTimingSEIEnabled;

  if(  rcVVEncParameter.m_iTargetBitRate )
  {
    rcEncCfg.m_RCRateControlMode     = 2;
    rcEncCfg.m_RCNumPasses           = rcVVEncParameter.m_iNumPasses;
    rcEncCfg.m_RCTargetBitrate       = rcVVEncParameter.m_iTargetBitRate;
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

  rcEncCfg.m_inputBitDepth[0]    = rcVVEncParameter.m_iInputBitDepth;
  rcEncCfg.m_inputBitDepth[1]    = rcVVEncParameter.m_iInputBitDepth;
  rcEncCfg.m_internalBitDepth[0] = rcVVEncParameter.m_iInternalBitDepth;
  rcEncCfg.m_internalBitDepth[1] = rcVVEncParameter.m_iInternalBitDepth;;

  rcEncCfg.m_numWppThreads = rcVVEncParameter.m_iThreadCount;
  if( rcVVEncParameter.m_iThreadCount > 0 )
  {
      rcEncCfg.m_ensureWppBitEqual = 1;
  }

  rcEncCfg.m_FrameRate                           = rcVVEncParameter.m_iTemporalRate / rcVVEncParameter.m_iTemporalScale;
  rcEncCfg.m_framesToBeEncoded                   = rcVVEncParameter.m_iMaxFrames;

  //======== Coding Structure =============
  rcEncCfg.m_GOPSize                             = rcVVEncParameter.m_iGopSize;
  rcEncCfg.m_InputQueueSize                      = 0;

  if( rcVVEncParameter.m_iIDRPeriod >= rcVVEncParameter.m_iGopSize  )
  {
    rcEncCfg.m_IntraPeriod                       = rcVVEncParameter.m_iIDRPeriod;
  }
  else // use m_iIDRPeriodSec
  {
    if ( rcEncCfg.m_FrameRate % rcVVEncParameter.m_iGopSize == 0 )
    {
      rcEncCfg.m_IntraPeriod = rcEncCfg.m_FrameRate * rcVVEncParameter.m_iIDRPeriodSec;
    }
    else
    {
      int iIDRPeriod  = (rcEncCfg.m_FrameRate * rcVVEncParameter.m_iIDRPeriodSec);
      if( iIDRPeriod < rcVVEncParameter.m_iGopSize )
      {
        iIDRPeriod = rcVVEncParameter.m_iGopSize;
      }

      int iDiff = iIDRPeriod % rcVVEncParameter.m_iGopSize;
      if( iDiff < rcVVEncParameter.m_iGopSize >> 1 )
      {
        rcEncCfg.m_IntraPeriod = iIDRPeriod - iDiff;
      }
      else
      {
        rcEncCfg.m_IntraPeriod = iIDRPeriod + rcVVEncParameter.m_iGopSize - iDiff;
      }
    }
  }

  if( rcVVEncParameter.m_eDecodingRefreshType == DRT_IDR )
  {
    rcEncCfg.m_DecodingRefreshType                 = 2;  // Random Accesss 0:none, 1:CRA, 2:IDR, 3:Recovery Point SEI
  }
  else if( rcVVEncParameter.m_eDecodingRefreshType == DRT_CRA )
  {
    rcEncCfg.m_DecodingRefreshType                 = 1;  // Random Accesss 0:none, 1:CRA, 2:IDR, 3:Recovery Point SEI
  }
  else if( rcVVEncParameter.m_eDecodingRefreshType == DRT_RECOVERY_POINT_SEI )
  {
    rcEncCfg.m_DecodingRefreshType                 = 3;  // Random Accesss 0:none, 1:CRA, 2:IDR, 3:Recovery Point SEI
  }
  else
  {
    rcEncCfg.m_DecodingRefreshType                 = 0;  // Random Accesss 0:none, 1:CRA, 2:IDR, 3:Recovery Point SEI
  }

  //======== Profile ================
  rcEncCfg.m_profile   = (vvenc::Profile::Name)rcVVEncParameter.m_eProfile;
  rcEncCfg.m_levelTier = (vvenc::Level::Tier)rcVVEncParameter.m_eTier;
  rcEncCfg.m_level     = (vvenc::Level::Name)rcVVEncParameter.m_eLevel;

  rcEncCfg.m_bitDepthConstraintValue = 10;
  rcEncCfg.m_rewriteParamSets        = true;
  rcEncCfg.m_internChromaFormat      = vvenc::CHROMA_420;

  if( 0 != rcEncCfg.initPreset( (PresetMode)rcVVEncParameter.m_iQuality  ) )
  {
    std::stringstream css;
    css << "undefined quality preset " << rcVVEncParameter.m_iQuality << " quality must be between 0 - 4.";
    m_cErrorString  = css.str();
    return VVENC_ERR_PARAMETER;
  }

  if( rcVVEncParameter.m_eSegMode != SEG_OFF )
  {
    if( rcEncCfg.m_MCTF )
    {
      switch( rcVVEncParameter.m_eSegMode )
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

int VVEncImpl::xCopyAndPadInputPlane( int16_t* pDes, const int iDesStride, const int iDesWidth, const int iDesHeight, const int16_t* pSrc, const int iSrcStride, const int iSrcWidth, const int iSrcHeight, const int iMargin )
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


int VVEncImpl::xCopyAu( VvcAccessUnit& rcVvcAccessUnit, const vvenc::AccessUnit& rcAu )
{
  rcVvcAccessUnit.m_bRAP = false;

  /* copy output AU */
  if ( ! rcAu.empty() )
  {
    uint32_t size = 0;  /* size of annexB unit in bytes */
    for (vvenc::AccessUnit::const_iterator it = rcAu.begin(); it != rcAu.end(); it++)
    {
      const vvenc::NALUnitEBSP& nalu = **it;

      if (it == rcAu.begin() ||
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
    }

    if( rcVvcAccessUnit.m_iBufSize < (int)size || rcVvcAccessUnit.m_pucBuffer == NULL )
    {
      return VVENC_NOT_ENOUGH_MEM;
    }

    uint32_t iUsedSize = 0;
    for (vvenc::AccessUnit::const_iterator it = rcAu.begin(); it != rcAu.end(); it++)
    {
      const vvenc::NALUnitEBSP& nalu = **it;
      static const uint8_t start_code_prefix[] = {0,0,0,1};
      if (it == rcAu.begin() ||
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
        ::memcpy( rcVvcAccessUnit.m_pucBuffer + iUsedSize, reinterpret_cast<const char*>(start_code_prefix), 4 );
        iUsedSize += 4;
      }
      else
      {
        ::memcpy( rcVvcAccessUnit.m_pucBuffer + iUsedSize, reinterpret_cast<const char*>(start_code_prefix+1), 3 );
        iUsedSize += 3;
      }
      uint32_t nalDataSize = uint32_t(nalu.m_nalUnitData.str().size()) ;
      ::memcpy( rcVvcAccessUnit.m_pucBuffer + iUsedSize, nalu.m_nalUnitData.str().c_str() , nalDataSize );
      iUsedSize += nalDataSize;

      if( nalu.m_nalUnitType == vvenc::NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_CODED_SLICE_IDR_N_LP ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_CODED_SLICE_CRA ||
          nalu.m_nalUnitType == vvenc::NAL_UNIT_CODED_SLICE_GDR )
      {
        rcVvcAccessUnit.m_bRAP = true;
      }
    }

    if( iUsedSize != size  )
    {
      return VVENC_NOT_ENOUGH_MEM;
    }

    rcVvcAccessUnit.m_iUsedSize = size;
    rcVvcAccessUnit.m_bCtsValid = rcAu.m_bCtsValid;
    rcVvcAccessUnit.m_bDtsValid = rcAu.m_bDtsValid;
    rcVvcAccessUnit.m_uiCts     = rcAu.m_uiCts;
    rcVvcAccessUnit.m_uiDts     = rcAu.m_uiDts;

    rcVvcAccessUnit.m_eSliceType     = (SliceType)rcAu.m_eSliceType;
    rcVvcAccessUnit.m_bRefPic        = rcAu.m_bRefPic;
    rcVvcAccessUnit.m_iTemporalLayer = rcAu.m_iTemporalLayer;
    rcVvcAccessUnit.m_uiPOC          = rcAu.m_uiPOC;
    rcVvcAccessUnit.m_cInfo          = rcAu.m_cInfo;
    rcVvcAccessUnit.m_iStatus        = rcAu.m_iStatus;
  }

  return 0;
}

} // namespace

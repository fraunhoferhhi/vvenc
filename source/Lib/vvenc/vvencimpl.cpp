/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVenc

(c) Copyright (2019-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.

1.    INTRODUCTION

The Fraunhofer Software Library VVenc (“Fraunhofer Versatile Video Encoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology.
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Encoding Library. It is in your responsibility to obtain those if necessary.

The Fraunhofer Versatile Video Encoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license.
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Encoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research.

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Encoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Encoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Encoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Encoding Library. If You use the Fraunhofer Versatile Video Encoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Encoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Encoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Encoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Encoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
----------------------------------------------------------------------------- */

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


namespace vvenc {

std::string VVEncImpl::m_cTmpErrorString;
std::string VVEncImpl::m_sPresetAsStr;
int g_LogLevel = LL_ERROR;


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

  g_LogLevel = (int)rcVVEncParameter.m_eLogLevel;
  setMsgFnc( &msgFnc );

  std::stringstream cssCap;

  vvenc::EncCfg cEncCfg;
  if( 0 != xInitLibCfg( rcVVEncParameter, cEncCfg ) )
  {
    return VVENC_ERR_INITIALIZE;
  }

  if( rcVVEncParameter.m_eLogLevel != LL_DEBUG_PLUS_INTERNAL_LOGS )
  {
    setMsgFnc( &msgFncDummy );
  }

  return VVENC_OK;
}

int VVEncImpl::init( const vvenc::VVEncParameter& rcVVEncParameter )
{
  if( m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  int iRet = xCheckParameter( rcVVEncParameter, m_cErrorString );
  if( 0 != iRet ) { return iRet; }

  g_LogLevel = (int)rcVVEncParameter.m_eLogLevel;
  setMsgFnc( &msgFnc );

  std::stringstream cssCap;
//  cssCap << NVM_ONOS;
//
//  char *tmp = nullptr;
//  int iRet = asprintf(&tmp, NVM_COMPILEDBY);
//  if( 0 == iRet ) cssCap << tmp;
//  free(tmp);
//
//  iRet = asprintf(&tmp, NVM_BITS);
//  if( 0 == iRet ) cssCap << tmp;
//  free(tmp);
//
//#if ENABLE_SIMD_OPT
//  std::string cSIMD;
//  cssCap << "SIMD=" << read_x86_extension( cSIMD ) << " ";
//#else
//  cssCap << "SIMD=NONE ";
//#endif
//
//#if ENABLE_TRACING
//  cssCap << "[ENABLE_TRACING] ";
//#endif

  m_cVVEncParameter = rcVVEncParameter;
  m_sEncoderCapabilities = cssCap.str();

  if( 0 != xInitLibCfg( rcVVEncParameter, m_cEncCfg ) )
  {
    return VVENC_ERR_INITIALIZE;
  }

  if( rcVVEncParameter.m_eLogLevel != LL_DEBUG_PLUS_INTERNAL_LOGS )
  {
    setMsgFnc( &msgFncDummy );
  }

  // create the encoder
  m_cEncoderIf.createEncoderLib( m_cEncCfg );

  m_bFlushed     = false;
  m_bInitialized = true;
  return VVENC_OK;
}

int VVEncImpl::uninit()
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  setMsgFnc( &msgFnc );
  m_cEncoderIf.printSummary();
  m_cEncoderIf.destroyEncoderLib();

  m_bInitialized = false;
  return VVENC_OK;
}


int VVEncImpl::encode( InputPicture* pcInputPicture, VvcAccessUnit& rcVvcAccessUnit )
{
  if( !m_bInitialized )             { return VVENC_ERR_INITIALIZE; }
  if( 0 == rcVvcAccessUnit.m_iBufSize ){ m_cErrorString = "AccessUnit BufferSize is 0"; return VVENC_NOT_ENOUGH_MEM; }
  if ( m_bFlushed )                 { m_cErrorString = "encoder already flushed"; return VVENC_ERR_RESTART_REQUIRED; }

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

  if( pcInputPicture->m_cPicBuffer.m_iWidth != this->m_cVVEncParameter.m_iWidth )
  {
    m_cErrorString = "InputPicture: unsuported width";
    return VVENC_ERR_UNSPECIFIED;
  }

  if( pcInputPicture->m_cPicBuffer.m_iHeight != this->m_cVVEncParameter.m_iHeight )
  {
    m_cErrorString = "InputPicture: unsuported height";
    return VVENC_ERR_UNSPECIFIED;
  }

  if( pcInputPicture->m_cPicBuffer.m_iWidth > pcInputPicture->m_cPicBuffer.m_iStride )
  {
    m_cErrorString = "InputPicture: unsuported width stride combination";
    return VVENC_ERR_UNSPECIFIED;
  }

  if( pcInputPicture->m_cPicBuffer.m_iCStride && pcInputPicture->m_cPicBuffer.m_iWidth/2 > pcInputPicture->m_cPicBuffer.m_iCStride )
  {
    m_cErrorString = "InputPicture: unsuported width cstride combination";
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
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }
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

struct BufferDimensions
{
  BufferDimensions(int iWidth, int iHeight, int iBitDepth, int iMaxCUSizeLog2, int iAddMargin = 16 )
    : iMaxCuSize      (1<<iMaxCUSizeLog2)
    , iSizeFactor     ((iBitDepth>8) ? (2) : (1))
    , iLumaMarginX    (iMaxCuSize + iAddMargin)
    , iLumaMarginY    (iLumaMarginX)
    , iStride         ((((iWidth + iMaxCuSize-1)>>iMaxCUSizeLog2)<<iMaxCUSizeLog2) + 2 * iLumaMarginX)
    , iLumaSize       ( iStride * ((((iHeight + iMaxCuSize-1)>>iMaxCUSizeLog2)<<iMaxCUSizeLog2) + 2 * iLumaMarginY))
    , iLumaOffset     ( iLumaMarginY * iStride + iLumaMarginX)
    , iChromaOffset   ((iLumaMarginY * iStride + iLumaMarginX * 2)/4)
    , iAlignmentGuard (16)
  {}

  int iMaxCuSize;
  int iSizeFactor;
  int iLumaMarginX;
  int iLumaMarginY;
  int iStride;
  int iLumaSize;
  int iLumaOffset;
  int iChromaOffset;
  int iAlignmentGuard;
};

int VVEncImpl::getPreferredBuffer( PicBuffer &rcPicBuffer )
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }
  int iRet= VVENC_OK;

  bool bMarginReq          = false;
  int iAddMargin           = bMarginReq ? 16: -1;
  const int iMaxCUSizeLog2 = 7;
  const int iBitDepth      = 10;

  int iMaxCUSizeLog2Buffer = bMarginReq ? iMaxCUSizeLog2 : 0;
  const BufferDimensions bd( m_cVVEncParameter.m_iWidth, m_cVVEncParameter.m_iHeight, iBitDepth, iMaxCUSizeLog2Buffer, iAddMargin);
  rcPicBuffer.m_iBitDepth = iBitDepth;
  rcPicBuffer.m_iWidth    = m_cVVEncParameter.m_iWidth;
  rcPicBuffer.m_iHeight   = m_cVVEncParameter.m_iHeight;
  rcPicBuffer.m_iStride   = bd.iStride;
  const int iBufSize      = bd.iSizeFactor * bd.iLumaSize * 3 / 2 + 3*bd.iAlignmentGuard;

  rcPicBuffer.m_pucDeletePicBuffer = new (std::nothrow) unsigned char[ iBufSize ];
  if( NULL == rcPicBuffer.m_pucDeletePicBuffer )
  {
    return VVENC_NOT_ENOUGH_MEM;
  }

  unsigned char* pY = rcPicBuffer.m_pucDeletePicBuffer + bd.iSizeFactor * ( bd.iLumaOffset );
  unsigned char* pU = rcPicBuffer.m_pucDeletePicBuffer + bd.iSizeFactor * ( bd.iChromaOffset +   bd.iLumaSize);
  unsigned char* pV = rcPicBuffer.m_pucDeletePicBuffer + bd.iSizeFactor * ( bd.iChromaOffset + 5*bd.iLumaSize/4);

  rcPicBuffer.m_pvY = (pY +   bd.iAlignmentGuard) - (((size_t)pY) & (bd.iAlignmentGuard-1));
  rcPicBuffer.m_pvU = (pU + 2*bd.iAlignmentGuard) - (((size_t)pU) & (bd.iAlignmentGuard-1));
  rcPicBuffer.m_pvV = (pV + 3*bd.iAlignmentGuard) - (((size_t)pV) & (bd.iAlignmentGuard-1));

  rcPicBuffer.m_eColorFormat     = VVC_CF_YUV420_PLANAR;
  rcPicBuffer.m_uiSequenceNumber = 0;

  return iRet;
}

int VVEncImpl::getConfig( vvenc::VVEncParameter& rcVVEncParameter )
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  rcVVEncParameter = m_cVVEncParameter;
  return 0;
}


const char* VVEncImpl::getVersionNumber()
{
  return VVENC_VERSION;
}

const char* VVEncImpl::getEncoderInfo()
{
    m_sEncoderInfo  = "Fraunhofer VVC Encoder ver. " VVENC_VERSION;
    m_sEncoderInfo += " ";
    m_sEncoderInfo += m_sEncoderCapabilities;
    return m_sEncoderInfo.c_str();
}

const char* VVEncImpl::getErrorMsg( int nRet )
{
  switch( nRet )
  {
  case VVENC_OK :                  m_cTmpErrorString = "expected behavior"; break;
  case VVENC_ERR_UNSPECIFIED:      m_cTmpErrorString = "unspecified malfunction"; break;
  case VVENC_ERR_INITIALIZE:       m_cTmpErrorString = "decoder not initialized or tried to initialize multiple times"; break;
  case VVENC_ERR_ALLOCATE:         m_cTmpErrorString = "internal allocation error"; break;
  case VVENC_NOT_ENOUGH_MEM:       m_cTmpErrorString = "allocated memory to small to receive encoded data"; break;
  case VVENC_ERR_PARAMETER:        m_cTmpErrorString = "inconsistent or invalid parameters"; break;
  case VVENC_ERR_NOT_SUPPORTED:    m_cTmpErrorString = "unsupported request"; break;
  case VVENC_ERR_RESTART_REQUIRED: m_cTmpErrorString = "decoder requires restart"; break;
  case VVENC_ERR_CPU:              m_cTmpErrorString = "unsupported CPU - SSE 4.1 needed!"; break;
  default:                         m_cTmpErrorString = "unknown ret code"; break;
  }
  return m_cTmpErrorString.c_str();
}

int VVEncImpl::setAndRetErrorMsg( int iRet )
{
  if( m_cErrorString.empty() )
  {
    m_cErrorString = getErrorMsg(iRet);
  }

  return iRet;
}

int VVEncImpl::getNumLeadFrames()
{
  return m_cEncCfg.m_MCTFNumLeadFrames;
}

int VVEncImpl::getNumTrailFrames()
{
  return m_cEncCfg.m_MCTFNumTrailFrames;
}

void VVEncImpl::clockStartTime()
{
  m_cTPStart = std::chrono::steady_clock::now();
}

void VVEncImpl::clockEndTime()
{
  m_cTPEnd = std::chrono::steady_clock::now();
}

double VVEncImpl::clockGetTimeDiffMs()
{
  return (double)(std::chrono::duration_cast<std::chrono::milliseconds>((m_cTPEnd)-(m_cTPStart)).count());
}

const char* VVEncImpl::getPresetParamsAsStr( int iQuality )
{
  m_sPresetAsStr.clear();

  std::stringstream css;
  vvenc::EncCfg cEncCfg;
  if( 0 != cEncCfg.initPreset( iQuality ))
  {
    css << "undefined preset " << iQuality;
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


  m_sPresetAsStr = css.str();
  return m_sPresetAsStr.c_str();
}


/* converting sdk params to internal (wrapper) params*/
int VVEncImpl::xCheckParameter( const vvenc::VVEncParameter& rcSrc, std::string& rcErrorString )
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
    ROTPARAMS( (rcSrc.m_eDecodingRefreshType == VVC_DRT_IDR || rcSrc.m_eDecodingRefreshType == VVC_DRT_CRA )&& (0 != rcSrc.m_iIDRPeriod % rcSrc.m_iGopSize),          "IDR period must be multiple of GOPSize" );
  }

  ROTPARAMS( rcSrc.m_iPerceptualQPA < 0 || rcSrc.m_iPerceptualQPA > 5,                      "Perceptual QPA must be in the range 0 - 5" );

  ROTPARAMS( rcSrc.m_eProfile != VVC_PROFILE_MAIN_10 && rcSrc.m_eProfile != VVC_PROFILE_MAIN_10_STILL_PICTURE && rcSrc.m_eProfile != VVC_PROFILE_AUTO, "unsupported profile, use main_10, main_10_still_picture or auto" );

  ROTPARAMS( (rcSrc.m_iQuality < 0 || rcSrc.m_iQuality > 3) && rcSrc.m_iQuality != 255,     "quality must be between 0 - 3  (0: faster, 1: fast, 2: medium, 3: slow)" );
  ROTPARAMS( rcSrc.m_iTargetBitRate < 0 || rcSrc.m_iTargetBitRate > 100000000,              "TargetBitrate must be between 0 - 100000000" );

  ROTPARAMS( rcSrc.m_eLogLevel < 0 || rcSrc.m_eLogLevel > LL_DEBUG_PLUS_INTERNAL_LOGS,      "^log level range 0 - 7" );

  ROTPARAMS( rcSrc.m_eSegMode != VVC_SEG_OFF && rcSrc.m_iMaxFrames < MCTF_RANGE,            "When using segment parallel encoding more then 2 frames have to be encoded" );

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

int VVEncImpl::xInitLibCfg( const VVEncParameter& rcVVEncParameter, vvenc::EncCfg& rcEncCfg )
{
  rcEncCfg.m_verbosity = std::min( (int)rcVVEncParameter.m_eLogLevel, (int)vvenc::DETAILS);

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

  if( rcVVEncParameter.m_eDecodingRefreshType == VVC_DRT_IDR )
  {
    rcEncCfg.m_DecodingRefreshType                 = 2;  // Random Accesss 0:none, 1:CRA, 2:IDR, 3:Recovery Point SEI
  }
  else if( rcVVEncParameter.m_eDecodingRefreshType == VVC_DRT_CRA )
  {
    rcEncCfg.m_DecodingRefreshType                 = 1;  // Random Accesss 0:none, 1:CRA, 2:IDR, 3:Recovery Point SEI
  }
  else if( rcVVEncParameter.m_eDecodingRefreshType == VVC_DRT_RECOVERY_POINT_SEI )
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

  if( 0 != rcEncCfg.initPreset( rcVVEncParameter.m_iQuality  ) )
  {
    std::stringstream css;
    css << "undefined quality preset " << rcVVEncParameter.m_iQuality << " quality must be between 0 - 3.";
    m_cErrorString  = css.str();
    return VVENC_ERR_PARAMETER;
  }

  if( rcVVEncParameter.m_eSegMode != VVC_SEG_OFF )
  {
    if( rcEncCfg.m_MCTF )
    {
      switch( rcVVEncParameter.m_eSegMode )
      {
        case VVC_SEG_FIRST:
          rcEncCfg.m_MCTFNumLeadFrames  = 0;
          rcEncCfg.m_MCTFNumTrailFrames = MCTF_RANGE;
          break;
        case VVC_SEG_MID:
          rcEncCfg.m_MCTFNumLeadFrames  = MCTF_RANGE;
          rcEncCfg.m_MCTFNumTrailFrames = MCTF_RANGE;
          break;
        case VVC_SEG_LAST:
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

    rcVvcAccessUnit.m_eSliceType     = (VvcSliceType)rcAu.m_eSliceType;
    rcVvcAccessUnit.m_bRefPic        = rcAu.m_bRefPic;
    rcVvcAccessUnit.m_iTemporalLayer = rcAu.m_iTemporalLayer;
    rcVvcAccessUnit.m_uiPOC          = rcAu.m_uiPOC;
    rcVvcAccessUnit.m_cInfo          = rcAu.m_cInfo;
    rcVvcAccessUnit.m_iStatus        = rcAu.m_iStatus;
  }

  return 0;
}

void VVEncImpl::msgApp( int level, const char* fmt, ... )
{
    va_list args;
    va_start( args, fmt );
    msgFnc( level, fmt, args );
    va_end( args );
}

void VVEncImpl::msgFnc( int level, const char* fmt, va_list args )
{
  if ( g_LogLevel >= level )
  {
    vfprintf( level == 1 ? stderr : stdout, fmt, args );
  }
}

void VVEncImpl::msgFncDummy( int level, const char* fmt, va_list args )
{
  // just a dummy to prevent the lib to print stuff to stdout
}

} // namespace

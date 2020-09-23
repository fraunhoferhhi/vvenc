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

#include "../../../include/vvenc/Nal.h"
#include "../../../include/vvenc/version.h"


namespace vvenc {
	
std::string VVEncImpl::m_cTmpErrorString;
int g_LogLevel = LL_ERROR;


#define ROTPARAMS(x, message) if(x) { rcErrorString = message; return VVENC_ERR_PARAMETER;}


VVEncImpl::VVEncImpl()
{

}

VVEncImpl::~VVEncImpl()
{

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

  int iRet= VVENC_OK;

  if( !pcInputPicture )
  {
    m_cErrorString = "InputPicture is null";
    return VVENC_ERR_UNSPECIFIED;
  }

  if( pcInputPicture->m_cPicBuffer.m_iBitDepth < 10 )
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

  int iRet= VVENC_OK;

  YUVBuffer cYUVBuffer;
  AccessUnit cAu;
  bool encDone    = false;

  while( !encDone &&  cAu.empty() )
  {
    m_cEncoderIf.encodePicture( true, cYUVBuffer, cAu, encDone );

    /* copy output AU */
    rcVvcAccessUnit.m_iUsedSize = 0;
    if ( !cAu.empty() )
    {
      iRet = xCopyAu( rcVvcAccessUnit, cAu  );
    }
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

  bool bMarginReq = false;
  int iAddMargin           = bMarginReq ? 16: -1;
  const int iMaxCUSizeLog2 = 7;
  const int iBitDepth = 10;

  int iMaxCUSizeLog2Buffer = bMarginReq ? iMaxCUSizeLog2 : 0;
  const BufferDimensions bd( m_cVVEncParameter.m_iWidth, m_cVVEncParameter.m_iHeight, iBitDepth, iMaxCUSizeLog2Buffer, iAddMargin);
  rcPicBuffer.m_iBitDepth = iBitDepth;
  rcPicBuffer.m_iWidth    = m_cVVEncParameter.m_iWidth;
  rcPicBuffer.m_iHeight   = m_cVVEncParameter.m_iHeight;
  rcPicBuffer.m_iStride   = bd.iStride;
  const int iBufSize = bd.iSizeFactor * bd.iLumaSize * 3 / 2 + 3*bd.iAlignmentGuard;

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
  rcPicBuffer.m_uiSequenceNumber = -1;

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

  ROTPARAMS( rcSrc.m_iThreadCount <= 0,                                                     "ThreadCount must be > 0" );

  ROTPARAMS( rcSrc.m_iIDRPeriod < 0,                                                        "IDR period must be GEZ" );
  ROTPARAMS( rcSrc.m_iGopSize != 1 && rcSrc.m_iGopSize != 16 && rcSrc.m_iGopSize != 32,     "GOP size 1, 16, 32 supported" );

  if( 1 != rcSrc.m_iGopSize )
  {
    ROTPARAMS( (rcSrc.m_eDecodingRefreshType == VVC_DRT_IDR || rcSrc.m_eDecodingRefreshType == VVC_DRT_CRA )&& (0 != rcSrc.m_iIDRPeriod % rcSrc.m_iGopSize),          "IDR period must be multiple of GOPSize" );
  }

  ROTPARAMS( rcSrc.m_iPerceptualQPA < 0 || rcSrc.m_iPerceptualQPA > 5,                      "Perceptual QPA must be in the range 0 - 5" );

  ROTPARAMS( rcSrc.m_eProfile != VVC_PROFILE_MAIN_10 && rcSrc.m_eProfile != VVC_PROFILE_MAIN_10_STILL_PICTURE && rcSrc.m_eProfile != VVC_PROFILE_AUTO, "unsupported profile, use main_10, main_10_still_picture or auto" );

  ROTPARAMS( rcSrc.m_iQuality < 0 || rcSrc.m_iQuality > 3,                                  "quality must be between 0 - 3  (0: faster, 1: fast, 2: medium, 3: slow)" );
  ROTPARAMS( rcSrc.m_iTargetBitRate < 0 || rcSrc.m_iTargetBitRate > 100000000,              "TargetBitrate must be between 0 - 100000000" );

  return 0;
}

int VVEncImpl::xInitLibCfg( const VVEncParameter& rcVVEncParameter, vvenc::EncCfg& rcEncCfg )
{
  rcEncCfg.m_verbosity = (int)rcVVEncParameter.m_eLogLevel;

  rcEncCfg.m_SourceWidth                                       = rcVVEncParameter.m_iWidth;
  rcEncCfg.m_SourceHeight                                      = rcVVEncParameter.m_iHeight;
  rcEncCfg.m_QP                                                = rcVVEncParameter.m_iQp;

  rcEncCfg.m_usePerceptQPA                                     = rcVVEncParameter.m_iPerceptualQPA;

  rcEncCfg.m_AccessUnitDelimiter = true;   // enable ACCessUnitDelimiter per default

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


  rcEncCfg.m_internalBitDepth[0] = 10;

  if( rcVVEncParameter.m_iThreadCount > 1 )
  {
      rcEncCfg.m_numWppThreads     = rcVVEncParameter.m_iThreadCount;
      rcEncCfg.m_ensureWppBitEqual = 1;
  }

  rcEncCfg.m_intraQPOffset = -3;
  rcEncCfg.m_lambdaFromQPEnable = true;

  rcEncCfg.m_FrameRate                           = rcVVEncParameter.m_iTemporalRate / rcVVEncParameter.m_iTemporalScale;

  rcEncCfg.m_framesToBeEncoded = 0;

  //======== Coding Structure =============
  rcEncCfg.m_GOPSize                             = rcVVEncParameter.m_iGopSize;
  rcEncCfg.m_InputQueueSize                      = rcVVEncParameter.m_iGopSize;

  rcEncCfg.m_IntraPeriod                         = rcVVEncParameter.m_iIDRPeriod;
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

  rcEncCfg.m_maxTempLayer = 5;
  rcEncCfg.m_numRPLList0  = 20;
  rcEncCfg.m_numRPLList1  = 20;

  //======== Profile ================
  rcEncCfg.m_profile   = (vvenc::Profile::Name)rcVVEncParameter.m_eProfile;
  rcEncCfg.m_levelTier = (vvenc::Level::Tier)rcVVEncParameter.m_eTier;
  rcEncCfg.m_level     = (vvenc::Level::Name)rcVVEncParameter.m_eLevel;

  rcEncCfg.m_bitDepthConstraintValue = 10;

  rcEncCfg.m_rewriteParamSets = true;

  rcEncCfg.m_internChromaFormat = vvenc::CHROMA_420;

//  rcEncCfg.m_sliceArgument = 1500;
//  rcEncCfg.m_MaxBTDepth = 2;

  rcEncCfg.m_MaxCodingDepth = 5;
  rcEncCfg.m_log2DiffMaxMinCodingBlockSize = 5;

  rcEncCfg.m_bUseASR   = true;
  rcEncCfg.m_bUseHADME = true;
  rcEncCfg.m_RDOQ      = 1;
  rcEncCfg.m_useRDOQTS = true;
  rcEncCfg.m_useSelectiveRDOQ = false;
  rcEncCfg.m_JointCbCrMode = true;
  rcEncCfg.m_cabacInitPresent = true;
  rcEncCfg.m_useFastLCTU = true;
  rcEncCfg.m_usePbIntraFast = true;
  rcEncCfg.m_useFastMrg = true;
  rcEncCfg.m_useAMaxBT = true;
  rcEncCfg.m_fastQtBtEnc = true;
  rcEncCfg.m_contentBasedFastQtbt = true;
  rcEncCfg.m_fastInterSearchMode = 1;

  rcEncCfg.m_MTSImplicit = true;
  rcEncCfg.m_SearchRange = 384;
  rcEncCfg.m_minSearchWindow = 96;

  rcEncCfg.m_AMVRspeed = 1;
  rcEncCfg.m_LMChroma = true;

  rcEncCfg.m_BDOF = true;
  rcEncCfg.m_DMVR = true;
  rcEncCfg.m_EDO = 1;
  rcEncCfg.m_lumaReshapeEnable = true;

  rcEncCfg.m_alf                   = true;

  // adapt to RA config files
  rcEncCfg.m_qpInValsCb.clear();
  rcEncCfg.m_qpInValsCb.push_back(17);
  rcEncCfg.m_qpInValsCb.push_back(22);
  rcEncCfg.m_qpInValsCb.push_back(34);
  rcEncCfg.m_qpInValsCb.push_back(42);

  rcEncCfg.m_qpOutValsCb.clear();
  rcEncCfg.m_qpOutValsCb.push_back(17);
  rcEncCfg.m_qpOutValsCb.push_back(23);
  rcEncCfg.m_qpOutValsCb.push_back(35);
  rcEncCfg.m_qpOutValsCb.push_back(39);

  if( 0 != xInitPreset( rcEncCfg, rcVVEncParameter.m_iQuality  ) )
  {
    std::stringstream css;
    css << "undefined quality preset " << rcVVEncParameter.m_iQuality << " quality must be between 0 - 3.";
    m_cErrorString  = css.str();
    return VVENC_ERR_PARAMETER;
  }

  if ( rcEncCfg.initCfgParameter() )
  {
    m_cErrorString = "cannot init internal config parameter";
    return VVENC_ERR_PARAMETER;
  }

  xPrintCfg();

  return 0;
}

int VVEncImpl::xInitPreset( vvenc::EncCfg& rcEncCfg, int iQuality )
{

  switch( iQuality )
  {
  case 0: // faster
          rcEncCfg.m_RDOQ                  = 2;
          rcEncCfg.m_DepQuantEnabled       = false;
          rcEncCfg.m_SignDataHidingEnabled = true;
          rcEncCfg.m_BDOF                  = false;
          rcEncCfg.m_alf                   = false;
          rcEncCfg.m_DMVR                  = false;
          rcEncCfg.m_JointCbCrMode         = false;
          rcEncCfg.m_AMVRspeed             = 0;
          rcEncCfg.m_lumaReshapeEnable     = false;
          rcEncCfg.m_EDO                   = 0;
          rcEncCfg.m_motionEstimationSearchMethod = 4;

          rcEncCfg.m_useFastMrg            = 2;
          rcEncCfg.m_fastLocalDualTreeMode = 1;
          rcEncCfg.m_fastSubPel            = 1;
          rcEncCfg.m_qtbttSpeedUp          = 1;

          rcEncCfg.m_LMCSOffset      = 6;
          rcEncCfg.m_MRL             = false;

          rcEncCfg.m_maxMTTDepth        = 1;
          rcEncCfg.m_maxMTTDepthI       = 2;
          rcEncCfg.m_maxMTTDepthIChroma = 2;
          rcEncCfg.m_maxNumMergeCand    = 6;

          rcEncCfg.m_useNonLinearAlfLuma   = false;
          rcEncCfg.m_useNonLinearAlfChroma = false;

          break;
  case 1:

          rcEncCfg.m_InputQueueSize += 5;

          rcEncCfg.m_RDOQ                  = 2;
          rcEncCfg.m_DepQuantEnabled       = false;
          rcEncCfg.m_SignDataHidingEnabled = true;
          rcEncCfg.m_BDOF                  = true;
          rcEncCfg.m_alf                   = true;
          rcEncCfg.m_ccalf                 = true;
          rcEncCfg.m_DMVR                  = true;
          rcEncCfg.m_JointCbCrMode         = false;
          rcEncCfg.m_AMVRspeed             = 0;
          rcEncCfg.m_lumaReshapeEnable     = false;
          rcEncCfg.m_EDO = 0;
          rcEncCfg.m_motionEstimationSearchMethod = 4;

          rcEncCfg.m_MCTF               = 2;
          rcEncCfg.m_MCTFNumLeadFrames  = 0;
          rcEncCfg.m_MCTFNumTrailFrames = 0;

          rcEncCfg.m_useFastMrg            = 2;
          rcEncCfg.m_fastLocalDualTreeMode = 1;
          rcEncCfg.m_fastSubPel            = 1;
          rcEncCfg.m_qtbttSpeedUp          = 1;

          rcEncCfg.m_LMCSOffset         = 6;
          rcEncCfg.m_MRL                = false;

          rcEncCfg.m_maxMTTDepth        = 1;
          rcEncCfg.m_maxMTTDepthI       = 2;
          rcEncCfg.m_maxMTTDepthIChroma = 2;
          rcEncCfg.m_maxNumMergeCand    = 6;

          rcEncCfg.m_useNonLinearAlfLuma   = false;
          rcEncCfg.m_useNonLinearAlfChroma = false;

    break;
  case 2:  // medium ( = ftc )

          rcEncCfg.m_useNonLinearAlfLuma   = false;
          rcEncCfg.m_useNonLinearAlfChroma = false;

          rcEncCfg.m_AMVRspeed = 5;

          rcEncCfg.m_SMVD = 3;

          rcEncCfg.m_MCTF = 2;
          rcEncCfg.m_MCTFNumLeadFrames = 0;
          rcEncCfg.m_MCTFNumTrailFrames = 0;

          rcEncCfg.m_useFastMrg             = 2;
          rcEncCfg. m_fastLocalDualTreeMode = 1;
          rcEncCfg.m_fastSubPel             = 1;

          rcEncCfg.m_Affine = 2;
          rcEncCfg.m_PROF   = 1;

          rcEncCfg.m_MMVD             = 3;
          rcEncCfg.m_allowDisFracMMVD = 1;

          rcEncCfg.m_motionEstimationSearchMethod = 4;

          rcEncCfg.m_maxMTTDepth        = 1;
          rcEncCfg.m_maxMTTDepthI       = 2;
          rcEncCfg.m_maxMTTDepthIChroma = 2;

          rcEncCfg.m_MaxCodingDepth     = 5;

          rcEncCfg.m_MIP             = 1;
          rcEncCfg.m_useFastMIP      = 4;

          rcEncCfg.m_ccalf           = true;
          rcEncCfg.m_qtbttSpeedUp    = 1;
          rcEncCfg.m_SbTMVP          = 1;
          rcEncCfg.m_Geo             = 3;
          rcEncCfg.m_LFNST           = 1;
          rcEncCfg.m_maxNumMergeCand = 6;

          rcEncCfg.m_LMCSOffset      = 6;

          rcEncCfg.m_RCKeepHierarchicalBit = 2;

    break;

  case 3: // slower

          rcEncCfg.m_useNonLinearAlfLuma   = true;
          rcEncCfg.m_useNonLinearAlfChroma = true;

          rcEncCfg.m_AMVRspeed = 1;

          rcEncCfg.m_SMVD = 3;

          rcEncCfg.m_MCTF = 2;
          rcEncCfg.m_MCTFNumLeadFrames  = 0;
          rcEncCfg.m_MCTFNumTrailFrames = 0;

          rcEncCfg.m_useFastMrg = 2;
          rcEncCfg. m_fastLocalDualTreeMode = 1;
          rcEncCfg.m_fastSubPel = 1;

          rcEncCfg.m_Affine = 2;
          rcEncCfg.m_PROF = 1;

          rcEncCfg.m_MMVD = 3;
          rcEncCfg.m_allowDisFracMMVD = 1;

          rcEncCfg.m_motionEstimationSearchMethod = 4;

          rcEncCfg.m_maxMTTDepth        = 2;
          rcEncCfg.m_maxMTTDepthI       = 3;
          rcEncCfg.m_maxMTTDepthIChroma = 3;

          rcEncCfg.m_MaxCodingDepth      = 5;

          rcEncCfg.m_MIP             = 1;
          rcEncCfg.m_useFastMIP      = 4;

          rcEncCfg.m_ccalf           = true;
          rcEncCfg.m_qtbttSpeedUp    = 1;
          rcEncCfg.m_SbTMVP          = 1;
          rcEncCfg.m_Geo             = 1;
          rcEncCfg.m_LFNST           = 1;
          rcEncCfg.m_maxNumMergeCand = 6;

          rcEncCfg.m_LMCSOffset      = 6;

          rcEncCfg.m_RCKeepHierarchicalBit = 2;

          m_cEncCfg.m_SBT  = 1;
          m_cEncCfg.m_CIIP = 1;

          rcEncCfg.m_contentBasedFastQtbt = false;

    break;

  default:
    return -1;
  }

  return 0;
}

void VVEncImpl::xPrintCfg()
{
  //msgFnc( (int)LL_DETAILS, "\n" );
  msgApp( (int)LL_DETAILS, "Real     Format                        : %dx%d %gHz\n", m_cEncCfg.m_SourceWidth - m_cEncCfg.m_confWinLeft - m_cEncCfg.m_confWinRight, m_cEncCfg.m_SourceHeight - m_cEncCfg.m_confWinTop - m_cEncCfg.m_confWinBottom, (double)m_cEncCfg.m_FrameRate / m_cEncCfg.m_temporalSubsampleRatio );
  msgApp( (int)LL_DETAILS, "Internal Format                        : %dx%d %gHz\n", m_cEncCfg.m_SourceWidth, m_cEncCfg.m_SourceHeight, (double)m_cEncCfg.m_FrameRate / m_cEncCfg.m_temporalSubsampleRatio );
  msgApp( (int)LL_DETAILS, "Sequence PSNR output                   : %s\n", ( m_cEncCfg.m_printMSEBasedSequencePSNR ? "Linear average, MSE-based" : "Linear average only" ) );
  msgApp( (int)LL_DETAILS, "Hexadecimal PSNR output                : %s\n", ( m_cEncCfg.m_printHexPsnr ? "Enabled" : "Disabled" ) );
  msgApp( (int)LL_DETAILS, "Sequence MSE output                    : %s\n", ( m_cEncCfg.m_printSequenceMSE ? "Enabled" : "Disabled" ) );
  msgApp( (int)LL_DETAILS, "Frame MSE output                       : %s\n", ( m_cEncCfg.m_printFrameMSE ? "Enabled" : "Disabled" ) );
  msgApp( (int)LL_DETAILS, "Cabac-zero-word-padding                : %s\n", ( m_cEncCfg.m_cabacZeroWordPaddingEnabled ? "Enabled" : "Disabled" ) );
  msgApp( (int)LL_DETAILS, "Frame/Field                            : Frame based coding\n" );
  if ( m_cEncCfg.m_framesToBeEncoded > 0 )
    msgApp( (int)LL_DETAILS, "Frame index                          : %u - %d (%d frames)\n", m_cEncCfg.m_FrameSkip, m_cEncCfg.m_FrameSkip + m_cEncCfg.m_framesToBeEncoded - 1, m_cEncCfg.m_framesToBeEncoded );
  else
    msgApp( (int)LL_DETAILS, "Frame index                            : %u - .. (all frames)\n", m_cEncCfg.m_FrameSkip );
  msgApp( (int)LL_DETAILS, "Profile                                : %s\n", getProfileStr( m_cEncCfg.m_profile).c_str() );
  msgApp( (int)LL_DETAILS, "Level                                  : %s\n", getLevelStr( m_cEncCfg.m_level).c_str() );
  msgApp( (int)LL_DETAILS, "CU size / total-depth                  : %d / %d\n", m_cEncCfg.m_CTUSize, m_cEncCfg.m_MaxCodingDepth );
  msgApp( (int)LL_DETAILS, "Max TB size                            : %d \n", 1 << m_cEncCfg.m_log2MaxTbSize );
  msgApp( (int)LL_DETAILS, "Motion search range                    : %d\n", m_cEncCfg.m_SearchRange );
  msgApp( (int)LL_DETAILS, "Intra period                           : %d\n", m_cEncCfg.m_IntraPeriod );
  msgApp( (int)LL_DETAILS, "Decoding refresh type                  : %d\n", m_cEncCfg.m_DecodingRefreshType );
  msgApp( (int)LL_DETAILS, "QP                                     : %d\n", m_cEncCfg.m_QP);
  msgApp( (int)LL_DETAILS, "Percept QPA                            : %d \n", m_cEncCfg.m_usePerceptQPA );
  msgApp( (int)LL_DETAILS, "Max dQP signaling subdiv               : %d\n", m_cEncCfg.m_cuQpDeltaSubdiv);

  msgApp( (int)LL_DETAILS, "Cb QP Offset (dual tree)               : %d (%d)\n", m_cEncCfg.m_chromaCbQpOffset, m_cEncCfg.m_chromaCbQpOffsetDualTree);
  msgApp( (int)LL_DETAILS, "Cr QP Offset (dual tree)               : %d (%d)\n", m_cEncCfg.m_chromaCrQpOffset, m_cEncCfg.m_chromaCrQpOffsetDualTree);
  msgApp( (int)LL_DETAILS, "GOP size                               : %d\n", m_cEncCfg.m_GOPSize );
  msgApp( (int)LL_DETAILS, "Input queue size                       : %d\n", m_cEncCfg.m_InputQueueSize );
  msgApp( (int)LL_DETAILS, "Input bit depth                        : (Y:%d, C:%d)\n", m_cEncCfg.m_inputBitDepth[ 0 ], m_cEncCfg.m_inputBitDepth[ 1 ] );
  msgApp( (int)LL_DETAILS, "MSB-extended bit depth                 : (Y:%d, C:%d)\n", m_cEncCfg.m_MSBExtendedBitDepth[ 0 ], m_cEncCfg.m_MSBExtendedBitDepth[ 1 ] );
  msgApp( (int)LL_DETAILS, "Internal bit depth                     : (Y:%d, C:%d)\n", m_cEncCfg.m_internalBitDepth[ 0 ], m_cEncCfg.m_internalBitDepth[ 1 ] );
  msgApp( (int)LL_DETAILS, "cu_chroma_qp_offset_subdiv             : %d\n", m_cEncCfg.m_cuChromaQpOffsetSubdiv);
  if (m_cEncCfg.m_bUseSAO)
  {
    msgApp( (int)LL_DETAILS, "log2_sao_offset_scale_luma             : %d\n", m_cEncCfg.m_log2SaoOffsetScale[ 0 ] );
    msgApp( (int)LL_DETAILS, "log2_sao_offset_scale_chroma           : %d\n", m_cEncCfg.m_log2SaoOffsetScale[ 1 ] );
  }

  switch (m_cEncCfg.m_costMode)
  {
    case vvenc::COST_STANDARD_LOSSY:               msgApp( (int)LL_DETAILS, "Cost function:                         : Lossy coding (default)\n"); break;
    case vvenc::COST_SEQUENCE_LEVEL_LOSSLESS:      msgApp( (int)LL_DETAILS, "Cost function:                         : Sequence_level_lossless coding\n"); break;
    case vvenc::COST_LOSSLESS_CODING:              msgApp( (int)LL_DETAILS, "Cost function:                         : Lossless coding with fixed QP\n"); break;
    case vvenc::COST_MIXED_LOSSLESS_LOSSY_CODING:  msgApp( (int)LL_DETAILS, "Cost function:                         : Mixed_lossless_lossy coding for lossless evaluation\n"); break;
    default:                                msgApp( (int)LL_DETAILS, "Cost function:                         : Unknown\n"); break;
  }

  msgApp( (int)LL_DETAILS, "\n");

  msgApp( LL_VERBOSE, "TOOL CFG: ");
  msgApp( LL_VERBOSE, "IBD:%d ", ((m_cEncCfg.m_internalBitDepth[ 0 ] > m_cEncCfg.m_MSBExtendedBitDepth[ 0 ]) || (m_cEncCfg.m_internalBitDepth[ 1 ] > m_cEncCfg.m_MSBExtendedBitDepth[ 1 ])));
  msgApp( LL_VERBOSE, "HAD:%d ", m_cEncCfg.m_bUseHADME                          );

  msgApp( LL_VERBOSE, "RDQ:%d ", m_cEncCfg.m_RDOQ                            );
  msgApp( LL_VERBOSE, "RDQTS:%d ", m_cEncCfg.m_useRDOQTS                        );
  msgApp( LL_VERBOSE, "ASR:%d ", m_cEncCfg.m_bUseASR                            );
  msgApp( LL_VERBOSE, "MinSearchWindow:%d ", m_cEncCfg.m_minSearchWindow        );
  msgApp( LL_VERBOSE, "RestrictMESampling:%d ", m_cEncCfg.m_bRestrictMESampling );
  msgApp( LL_VERBOSE, "FEN:%d ", m_cEncCfg.m_fastInterSearchMode                );
  msgApp( LL_VERBOSE, "ECU:%d ", m_cEncCfg.m_bUseEarlyCU                        );
  msgApp( LL_VERBOSE, "FDM:%d ", m_cEncCfg.m_useFastDecisionForMerge            );
  msgApp( LL_VERBOSE, "ESD:%d ", m_cEncCfg.m_useEarlySkipDetection              );
  msgApp( LL_VERBOSE, "Tiles:%dx%d ", m_cEncCfg.m_numTileColumnsMinus1 + 1, m_cEncCfg.m_numTileRowsMinus1 + 1 );
  msgApp( LL_VERBOSE, "CIP:%d ", m_cEncCfg.m_bUseConstrainedIntraPred);
  msgApp( LL_VERBOSE, "SAO:%d ", (m_cEncCfg.m_bUseSAO)?(1):(0));
  msgApp( LL_VERBOSE, "ALF:%d ", m_cEncCfg.m_alf ? 1 : 0 );
  if( m_cEncCfg.m_alf )
  {
      msgApp(LL_VERBOSE, "(NonLinLuma:%d ", m_cEncCfg.m_useNonLinearAlfLuma );
      msgApp(LL_VERBOSE, "NonLinChr:%d) ", m_cEncCfg.m_useNonLinearAlfChroma );
  }
  msgApp( LL_VERBOSE, "CCALF:%d ", m_cEncCfg.m_ccalf ? 1 : 0 );
  const int iWaveFrontSubstreams = m_cEncCfg.m_entropyCodingSyncEnabled ? (m_cEncCfg.m_SourceHeight + m_cEncCfg.m_CTUSize - 1) / m_cEncCfg.m_CTUSize : 1;
  msgApp( LL_VERBOSE, "WaveFrontSynchro:%d WaveFrontSubstreams:%d ", m_cEncCfg.m_entropyCodingSyncEnabled?1:0, iWaveFrontSubstreams);
  msgApp( LL_VERBOSE, "TMVPMode:%d ", m_cEncCfg.m_TMVPModeId     );

  msgApp( LL_VERBOSE, "DQ:%d "               , m_cEncCfg.m_DepQuantEnabled );
  if( m_cEncCfg.m_DepQuantEnabled ) { if( m_cEncCfg.m_dqThresholdVal & 1 ) msgApp( LL_VERBOSE, "(Thr: %d.5) ", m_cEncCfg.m_dqThresholdVal >> 1 ); else msgApp( LL_VERBOSE, "(Thr: %d) ", m_cEncCfg.m_dqThresholdVal >> 1 ); }
  msgApp( LL_VERBOSE, "SignBitHidingFlag:%d ", m_cEncCfg.m_SignDataHidingEnabled );
  msgApp( LL_VERBOSE, "Perceptual QPA:%d "   , m_cEncCfg.m_usePerceptQPA );

  {
    msgApp( LL_VERBOSE, "\nNEXT TOOL CFG: " );
    msgApp( LL_VERBOSE, "DualITree:%d ",          m_cEncCfg.m_dualITree );
    msgApp( LL_VERBOSE, "BIO:%d ",                m_cEncCfg.m_BDOF );
    msgApp( LL_VERBOSE, "DMVR:%d ",               m_cEncCfg.m_DMVR );
    msgApp( LL_VERBOSE, "MTSImplicit:%d ",        m_cEncCfg.m_MTSImplicit );
    msgApp( LL_VERBOSE, "SBT:%d ",                m_cEncCfg.m_SBT );
    msgApp( LL_VERBOSE, "JointCbCr:%d ",          m_cEncCfg.m_JointCbCrMode );
    msgApp( LL_VERBOSE, "CabacInitPresent:%d ",   m_cEncCfg.m_cabacInitPresent );
    msgApp( LL_VERBOSE, "AMVRspeed:%d ",          m_cEncCfg.m_AMVRspeed );
    msgApp( LL_VERBOSE, "SMVD:%d ",               m_cEncCfg.m_SMVD );

    msgApp(LL_VERBOSE, "Reshape:%d ", m_cEncCfg.m_lumaReshapeEnable);
    if (m_cEncCfg.m_lumaReshapeEnable)
    {
      msgApp(LL_VERBOSE, "(Signal:%s ", m_cEncCfg.m_reshapeSignalType == 0 ? "SDR" : (m_cEncCfg.m_reshapeSignalType == 2 ? "HDR-HLG" : "HDR-PQ"));
      msgApp(LL_VERBOSE, "Opt:%d", m_cEncCfg.m_adpOption);
      if (m_cEncCfg.m_adpOption > 0) { msgApp(LL_VERBOSE, " CW:%d", m_cEncCfg.m_initialCW); }
      msgApp(LL_VERBOSE, ") ");
    }
    msgApp(LL_VERBOSE, "CIIP:%d "    , m_cEncCfg.m_CIIP);
    msgApp(LL_VERBOSE, "MIP:%d "     , m_cEncCfg.m_MIP);
    msgApp(LL_VERBOSE, "EncDbOpt:%d ", m_cEncCfg.m_EDO);
    msgApp(LL_VERBOSE, "MCTF:%d "    , m_cEncCfg.m_MCTF);
    if ( m_cEncCfg.m_MCTF )
    {
      msgApp(LL_VERBOSE, "[L:%d, T:%d] ", m_cEncCfg.m_MCTFNumLeadFrames, m_cEncCfg.m_MCTFNumTrailFrames);
    }
    msgApp( LL_VERBOSE, "Affine:%d ",             m_cEncCfg.m_Affine);
    msgApp( LL_VERBOSE, "Affine_Prof:%d ",        m_cEncCfg.m_PROF);
    msgApp( LL_VERBOSE, "Affine_Type:%d ",        m_cEncCfg.m_AffineType);
    msgApp( LL_VERBOSE, "MMVD:%d ",               m_cEncCfg.m_MMVD);
    msgApp( LL_VERBOSE, "DisFracMMVD:%d ",        m_cEncCfg.m_allowDisFracMMVD);
    msgApp( LL_VERBOSE, "FastSearch:%d ",         m_cEncCfg.m_motionEstimationSearchMethod);
    msgApp( LL_VERBOSE, "SbTMVP:%d ",             m_cEncCfg.m_SbTMVP);
    msgApp( LL_VERBOSE, "Geo:%d ",                m_cEncCfg.m_Geo);
    msgApp( LL_VERBOSE, "LFNST:%d ",              m_cEncCfg.m_LFNST);
    msgApp( LL_VERBOSE, "MTS:%d ",                m_cEncCfg.m_MTS);
    msgApp( LL_VERBOSE, "MTSIntraCand:%d ",       m_cEncCfg.m_MTSIntraMaxCand);
  }

  msgApp( LL_VERBOSE, "\nFAST TOOL CFG: " );
  msgApp( LL_VERBOSE, "LCTUFast:%d ",             m_cEncCfg.m_useFastLCTU );
  msgApp( LL_VERBOSE, "FastMrg:%d ",              m_cEncCfg.m_useFastMrg );
  msgApp( LL_VERBOSE, "PBIntraFast:%d ",          m_cEncCfg.m_usePbIntraFast );
  msgApp( LL_VERBOSE, "AMaxBT:%d ",               m_cEncCfg.m_useAMaxBT );
  msgApp( LL_VERBOSE, "FastQtBtEnc:%d ",          m_cEncCfg.m_fastQtBtEnc );
  msgApp( LL_VERBOSE, "ContentBasedFastQtbt:%d ", m_cEncCfg.m_contentBasedFastQtbt );
  if(m_cEncCfg. m_MIP ) msgApp(LL_VERBOSE, "FastMIP:%d ", m_cEncCfg.m_useFastMIP);
  msgApp( LL_VERBOSE, "FastLocalDualTree:%d ",m_cEncCfg. m_fastLocalDualTreeMode );
  msgApp( LL_VERBOSE, "FastSubPel:%d ",           m_cEncCfg.m_fastSubPel );
  msgApp( LL_VERBOSE, "QtbttExtraFast:%d ",       m_cEncCfg.m_qtbttSpeedUp );
  msgApp( LL_VERBOSE, "RateControl:%d ",          m_cEncCfg.m_RCRateControlMode );

  if ( m_cEncCfg.m_RCRateControlMode )
  {
    msgApp( LL_VERBOSE, "TargetBitrate:%d ",           m_cEncCfg.m_RCTargetBitrate );
    msgApp( LL_VERBOSE, "KeepHierarchicalBit:%d ",     m_cEncCfg.m_RCKeepHierarchicalBit );
    msgApp( LL_VERBOSE, "RCLCUSeparateModel:%d ",      m_cEncCfg.m_RCUseLCUSeparateModel );
    msgApp( LL_VERBOSE, "InitialQP:%d ",               m_cEncCfg.m_RCInitialQP );
    msgApp( LL_VERBOSE, "RCForceIntraQP:%d ",          m_cEncCfg.m_RCForceIntraQP );
  }

  msgApp( LL_VERBOSE, "\nPARALLEL PROCESSING CFG: " );
  msgApp( LL_VERBOSE, "FPP:%d ",                  m_cEncCfg.m_frameParallel );
  msgApp( LL_VERBOSE, "NumFppThreads:%d ",        m_cEncCfg.m_numFppThreads );
  msgApp( LL_VERBOSE, "FppBitEqual:%d ",          m_cEncCfg.m_ensureFppBitEqual );
  msgApp( LL_VERBOSE, "WPP:%d ",                  m_cEncCfg.m_numWppThreads );
  msgApp( LL_VERBOSE, "WppBitEqual:%d ",          m_cEncCfg.m_ensureWppBitEqual );
  msgApp( LL_VERBOSE, "WF:%d ",                   m_cEncCfg.m_entropyCodingSyncEnabled );

  msgApp( LL_VERBOSE, "\n\n");

  msgApp( LL_NOTICE, "\n");

  fflush( stdout );
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


std::string VVEncImpl::getProfileStr( int iProfile )
{
  std::string cT;

  switch( iProfile )
  {
    case 0: cT = "none"; break;
    case 1: cT = "main_10"; break;
    case 2: cT = "main_10_444"; break;
    case 3: cT = "main_10_still_picture"; break;
    case 4: cT = "main_10_444_still_picture"; break;
    case 5: cT = "multilayer_main_10"; break;
    case 6: cT = "multilayer_main_10_444"; break;
    case 7: cT = "multilayer_main_10_still_picture"; break;
    case 8: cT = "multilayer_main_10_444_still_picture"; break;
    case 9: cT = "auto"; break;

    default: cT="unknown"; break;
  }

  return cT;
}

std::string VVEncImpl::getLevelStr( int iLevel )
{
  std::string cT;

  switch( iLevel )
  {
    case 0:  cT = "none"; break;
    case 16: cT = "1"; break;
    case 32: cT = "2"; break;
    case 35: cT = "2.1"; break;
    case 48: cT = "3"; break;
    case 51: cT = "3.1"; break;
    case 64: cT = "4"; break;
    case 67: cT = "4.1"; break;
    case 80: cT = "5"; break;
    case 83: cT = "5.1"; break;
    case 86: cT = "5.2"; break;
    case 96: cT = "6"; break;
    case 99: cT = "6.1"; break;
    case 102: cT = "6.2"; break;
    case 255: cT = "15.5"; break;

    default: cT="unknown"; break;
  }

  return cT;
}


} // namespace

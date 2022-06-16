/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

     * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

     * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

     * Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

/**
  \ingroup hhivvcdeclibExternalInterfaces
  \file    vvencimpl.cpp
  \brief   This file contains the internal interface of the vvenc SDK.
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

#if defined( __linux__ )
#include <malloc.h>
#endif


#if _DEBUG
#define HANDLE_EXCEPTION 0
#else
#define HANDLE_EXCEPTION 1
#endif


namespace vvenc {

#define ROTPARAMS(x, message) if(x) { rcErrorString = message; return VVENC_ERR_PARAMETER;}

// ====================================================================================================================

static_assert( sizeof(Pel)  == sizeof(*(vvencYUVPlane::ptr)),   "internal bits per pel differ from interface definition" );

// ====================================================================================================================

bool tryDecodePicture( Picture* pic, const int expectedPoc, const std::string& bitstreamFileName, FFwdDecoder& ffwdDecoder, ParameterSetMap<APS>* apsMap, MsgLog& logger, bool bDecodeUntilPocFound = false, int debugPOC = -1, bool copyToEnc = true );

VVEncImpl::VVEncImpl()
{
  m_cEncoderInfo = createEncoderInfoStr();
}

VVEncImpl::~VVEncImpl()
{

}

int VVEncImpl::getConfig( vvenc_config& config ) const
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  config = m_cVVEncCfg;
  return VVENC_OK;
}

int VVEncImpl::reconfig( const vvenc_config&  )
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }
  return VVENC_ERR_NOT_SUPPORTED;
}

int VVEncImpl::checkConfig( const vvenc_config& config )
{
  vvenc_config configCpy = config;
  if ( vvenc_init_config_parameter(&configCpy) )
  {
    return VVENC_ERR_INITIALIZE;
  }

  return VVENC_OK;
}

int VVEncImpl::init( vvenc_config* config )
{
  if( m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  if (nullptr == config)
  {
    msg.log( VVENC_ERROR, "vvenc_config is null\n" );
    return VVENC_ERR_PARAMETER;
  }

  m_cVVEncCfgExt = *config;
  m_cVVEncCfg    = *config;

  if ( vvenc_init_config_parameter(&m_cVVEncCfg) ) // init auto/dependent options
  {
    return VVENC_ERR_INITIALIZE;
  }

  if( config->m_msgFnc )
  { 
    msg.setCallback( config->m_msgCtx, config->m_msgFnc );
  }
  
  // initialize the encoder
  m_pEncLib = new EncLib ( msg );

#if HANDLE_EXCEPTION
  try
#endif
  {
    m_pEncLib->initEncoderLib( m_cVVEncCfg );
  }
#if HANDLE_EXCEPTION
  catch( std::exception& e )
  {
    msg.log( VVENC_ERROR, "\n%s\n", e.what() );
    m_cErrorString = e.what();
    return VVENC_ERR_UNSPECIFIED;
  }
#endif

  m_bInitialized = true;
  m_eState       = INTERNAL_STATE_INITIALIZED;
  return VVENC_OK;
}

int VVEncImpl::initPass( int pass, const char* statsFName )
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
#if HANDLE_EXCEPTION
    try
#endif
    {
      m_pEncLib->initPass( pass, statsFName );
    }
#if HANDLE_EXCEPTION
    catch( std::exception& e )
    {
      msg.log( VVENC_ERROR, "\n%s\n", e.what() );
      m_cErrorString = e.what();
      return VVENC_ERR_UNSPECIFIED;
    }
#endif
  }

  m_eState = INTERNAL_STATE_INITIALIZED;
  return VVENC_OK;
}

int VVEncImpl::uninit()
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  if ( m_pEncLib )
  {
#if HANDLE_EXCEPTION
    try
#endif
    {
      m_pEncLib->uninitEncoderLib();
      delete m_pEncLib;
      m_pEncLib = nullptr;
    }
#if HANDLE_EXCEPTION
    catch( std::exception& e )
    {
      msg.log( VVENC_ERROR, "\n%s\n", e.what() );
      m_cErrorString = e.what();
      return VVENC_ERR_UNSPECIFIED;
    }
#endif
  }

#if defined( __linux__ )
  malloc_trim(0);   // free unused heap memory
#endif

  m_bInitialized = false;
  m_eState       = INTERNAL_STATE_UNINITIALIZED;
  return VVENC_OK;
}

bool VVEncImpl::isInitialized() const
{
  return m_bInitialized;
}

int VVEncImpl::setRecYUVBufferCallback( void * ctx, vvencRecYUVBufferCallback callback )
{
  if( !m_bInitialized || !m_pEncLib ){ return VVENC_ERR_INITIALIZE; }

   m_pEncLib->setRecYUVBufferCallback( ctx, callback );
  return VVENC_OK;
}

int VVEncImpl::encode( vvencYUVBuffer* pcYUVBuffer, vvencAccessUnit* pcAccessUnit, bool* pbEncodeDone )
{
  if( !m_bInitialized )                      { return VVENC_ERR_INITIALIZE; }
  if( m_eState == INTERNAL_STATE_FINALIZED ) { m_cErrorString = "encoder already flushed, please reinit."; return VVENC_ERR_RESTART_REQUIRED; }

  if( !pcAccessUnit )
  {
    m_cErrorString = "vvencAccessUnit is null. AU memory must be allocated before encode call.";
    return VVENC_NOT_ENOUGH_MEM;
  }
  if( pcAccessUnit->payloadSize <= 0 )
  {
    m_cErrorString = "vvencAccessUnit has no payload size. AU payload must have a sufficient size to store encoded data.";
    return VVENC_NOT_ENOUGH_MEM;
  }

  int iRet= VVENC_OK;

  bool bFlush = false;
  if( pcYUVBuffer )
  {
    if( m_eState == INTERNAL_STATE_FLUSHING ) { m_cErrorString = "encoder already received flush indication, please reinit."; return VVENC_ERR_RESTART_REQUIRED; }

    if( pcYUVBuffer->planes[0].ptr == nullptr )
    {
      m_cErrorString = "InputPicture: invalid input buffers";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( m_cVVEncCfg.m_internChromaFormat != VVENC_CHROMA_400 )
    {
      if( pcYUVBuffer->planes[1].ptr == nullptr ||
          pcYUVBuffer->planes[2].ptr == nullptr )
      {
        m_cErrorString = "InputPicture: invalid input buffers for chroma";
        return VVENC_ERR_UNSPECIFIED;
      }
    }

    if( pcYUVBuffer->planes[0].width != m_cVVEncCfg.m_SourceWidth )
    {
      m_cErrorString = "InputPicture: unsupported width";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( pcYUVBuffer->planes[0].height != m_cVVEncCfg.m_SourceHeight )
    {
      m_cErrorString = "InputPicture: unsupported height";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( pcYUVBuffer->planes[0].width > pcYUVBuffer->planes[0].stride )
    {
      m_cErrorString = "InputPicture: unsupported width stride combination";
      return VVENC_ERR_UNSPECIFIED;
    }

    if( m_cVVEncCfg.m_internChromaFormat != VVENC_CHROMA_400 )
    {
      if( m_cVVEncCfg.m_internChromaFormat == VVENC_CHROMA_444 )
      {
        if( pcYUVBuffer->planes[1].stride && pcYUVBuffer->planes[0].width > pcYUVBuffer->planes[1].stride )
        {
          m_cErrorString = "InputPicture: unsupported width cstride combination for 2nd plane";
          return VVENC_ERR_UNSPECIFIED;
        }

        if( pcYUVBuffer->planes[2].stride && pcYUVBuffer->planes[0].width > pcYUVBuffer->planes[2].stride )
        {
          m_cErrorString = "InputPicture: unsupported width cstride combination for 3rd plane";
          return VVENC_ERR_UNSPECIFIED;
        }
      }
      else
      {
        if( pcYUVBuffer->planes[1].stride && pcYUVBuffer->planes[0].width/2 > pcYUVBuffer->planes[1].stride )
        {
          m_cErrorString = "InputPicture: unsupported width cstride combination for 2nd plane";
          return VVENC_ERR_UNSPECIFIED;
        }

        if( pcYUVBuffer->planes[2].stride && pcYUVBuffer->planes[0].width/2 > pcYUVBuffer->planes[2].stride )
        {
          m_cErrorString = "InputPicture: unsupported width cstride combination for 3rd plane";
          return VVENC_ERR_UNSPECIFIED;
        }
      }
    }

    if( m_eState == INTERNAL_STATE_INITIALIZED ){ m_eState = INTERNAL_STATE_ENCODING; }
  }
  else
  {
    if( m_eState == INTERNAL_STATE_ENCODING ){ m_eState = INTERNAL_STATE_FLUSHING; }
    bFlush = true;
  }

  // reset AU data
  if( pcAccessUnit )
  {
    vvenc_accessUnit_reset(pcAccessUnit);
  }
  *pbEncodeDone  = false;

  AccessUnitList cAu;
#if HANDLE_EXCEPTION
  try
#endif
  {
    m_pEncLib->encodePicture( bFlush, pcYUVBuffer, cAu, *pbEncodeDone );
  }
#if HANDLE_EXCEPTION
  catch( std::exception& e )
  {
    msg.log( VVENC_ERROR, "\n%s\n", e.what() );
    m_cErrorString = e.what();
    return VVENC_ERR_UNSPECIFIED;
  }
#endif

  if( *pbEncodeDone )
  {
    if( m_eState == INTERNAL_STATE_FLUSHING )
    {
      m_eState = INTERNAL_STATE_FINALIZED;
    }
    else if( m_eState == INTERNAL_STATE_INITIALIZED )
    {
      m_eState = INTERNAL_STATE_INITIALIZED; // keep initialized state, when flushing without having encoded anything, we still can start to encode
    }
    else
    {
      *pbEncodeDone = false;
    }
  }

  /* copy output AU */
  if ( !cAu.empty() )
  {
    int sizeAu = xGetAccessUnitsSize( cAu );
    if( pcAccessUnit->payloadSize < sizeAu )
    {
      std::stringstream css;
      css << "vvencAccessUnit payload size is too small to store data. (payload size: " << pcAccessUnit->payloadSize << ", needed " << sizeAu << ")";
      m_cErrorString =css.str();
      return VVENC_NOT_ENOUGH_MEM;
    }

    iRet = xCopyAu( *pcAccessUnit, cAu  );
  }

#if defined( __linux__ )
  malloc_trim(0);   // free unused heap memory
#endif

  return iRet;
}

int VVEncImpl::getParameterSets( vvencAccessUnit *pcAccessUnit )
{
  if( !m_bInitialized )                      { return VVENC_ERR_INITIALIZE; }
  if( m_eState == INTERNAL_STATE_FINALIZED ) { m_cErrorString = "encoder already flushed, please reinit."; return VVENC_ERR_RESTART_REQUIRED; }

  if( !pcAccessUnit )
  {
    m_cErrorString = "vvencAccessUnit is null. AU memory must be allocated before encode call.";
    return VVENC_NOT_ENOUGH_MEM;
  }
  if( pcAccessUnit->payloadSize <= 0 )
  {
    m_cErrorString = "vvencAccessUnit has no payload size. AU payload must have a sufficient size to store encoded data.";
    return VVENC_NOT_ENOUGH_MEM;
  }

  int iRet= VVENC_OK;

  // reset AU data
  vvenc_accessUnit_reset(pcAccessUnit);

  AccessUnitList cAu;
#if HANDLE_EXCEPTION
  try
#endif
  {
    m_pEncLib->getParameterSets( cAu );
  }
#if HANDLE_EXCEPTION
  catch( std::exception& e )
  {
    msg.log( VVENC_ERROR, "\n%s\n", e.what() );
    m_cErrorString = e.what();
    return VVENC_ERR_UNSPECIFIED;
  }
#endif

  /* copy output AU */
  if ( !cAu.empty() )
  {
    int sizeAu = xGetAccessUnitsSize( cAu );
    if( pcAccessUnit->payloadSize < sizeAu )
    {
      std::stringstream css;
      css << "vvencAccessUnit payload size is too small to store data. (payload size: " << pcAccessUnit->payloadSize << ", needed " << sizeAu << ")";
      m_cErrorString =css.str();
      return VVENC_NOT_ENOUGH_MEM;
    }

    iRet = xCopyAu( *pcAccessUnit, cAu  );
  }

  return iRet;
}



const char* VVEncImpl::getVersionNumber()
{
  return VVENC_VERSION;
}

const char* VVEncImpl::getEncoderInfo() const
{
  return m_cEncoderInfo.c_str();
}

const char* VVEncImpl::getLastError() const
{
  return m_cErrorString.c_str();
}

const char* VVEncImpl::getErrorMsg( int nRet )
{
  switch( nRet )
  {
  case VVENC_OK :                  return vvencErrorMsg[0]; break;
  case VVENC_ERR_UNSPECIFIED:      return vvencErrorMsg[1]; break;
  case VVENC_ERR_INITIALIZE:       return vvencErrorMsg[2]; break;
  case VVENC_ERR_ALLOCATE:         return vvencErrorMsg[3]; break;
  case VVENC_NOT_ENOUGH_MEM:       return vvencErrorMsg[4]; break;
  case VVENC_ERR_PARAMETER:        return vvencErrorMsg[5]; break;
  case VVENC_ERR_NOT_SUPPORTED:    return vvencErrorMsg[6]; break;
  case VVENC_ERR_RESTART_REQUIRED: return vvencErrorMsg[7]; break;
  case VVENC_ERR_CPU:              return vvencErrorMsg[8]; break;
  default:                         return vvencErrorMsg[9]; break;
  }
  return vvencErrorMsg[9];
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
  return m_cVVEncCfg.m_leadFrames;
}

int VVEncImpl::getNumTrailFrames() const
{
  return m_cVVEncCfg.m_trailFrames;
}

int VVEncImpl::printSummary() const
{
  if( !m_bInitialized ){ return -1; }
  if( nullptr == m_pEncLib )  { return -1; }

  m_pEncLib->printSummary();
  return 0;
}

int VVEncImpl::xGetAccessUnitsSize( const vvenc::AccessUnitList& rcAuList )
{
  uint32_t sizeSum = 0;
  if ( ! rcAuList.empty() )
  {
    for (vvenc::AccessUnitList::const_iterator it = rcAuList.begin(); it != rcAuList.end(); it++)
    {
      const vvenc::NALUnitEBSP& nalu = **it;
      if (it == rcAuList.begin() ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_DCI ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_SPS ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_VPS ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_PPS ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_PREFIX_APS ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_SUFFIX_APS )
      {
        sizeSum += 4;
      }
      else
      {
        sizeSum += 3;
      }
      sizeSum += uint32_t(nalu.m_nalUnitData.str().size());
    }
  }

  return sizeSum;
}


int VVEncImpl::xCopyAu( vvencAccessUnit& rcAccessUnit, const vvenc::AccessUnitList& rcAuList )
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
          nalu.m_nalUnitType == VVENC_NAL_UNIT_DCI ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_SPS ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_VPS ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_PPS ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_PREFIX_APS ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_SUFFIX_APS )
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

      switch( nalu.m_nalUnitType )
      {
        case VVENC_NAL_UNIT_CODED_SLICE_TRAIL:
        case VVENC_NAL_UNIT_CODED_SLICE_STSA:
        case VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL:
        case VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP:
        case VVENC_NAL_UNIT_CODED_SLICE_CRA:
        case VVENC_NAL_UNIT_CODED_SLICE_GDR:
        case VVENC_NAL_UNIT_CODED_SLICE_RADL:
        case VVENC_NAL_UNIT_CODED_SLICE_RASL:
        case VVENC_NAL_UNIT_DCI:
        case VVENC_NAL_UNIT_VPS:
        case VVENC_NAL_UNIT_SPS:
        case VVENC_NAL_UNIT_PPS:
        case VVENC_NAL_UNIT_PREFIX_APS:
        case VVENC_NAL_UNIT_SUFFIX_APS:
          rcAccessUnit.essentialBytes += size;
          break;
        default:
          break;
      }
    }

    if( rcAccessUnit.payloadSize < (int)sizeSum )
    {
      return -1;
    }

    uint32_t iUsedSize = 0;
    for (vvenc::AccessUnitList::const_iterator it = rcAuList.begin(); it != rcAuList.end(); it++)
    {
      const vvenc::NALUnitEBSP& nalu = **it;
      static const uint8_t start_code_prefix[] = {0,0,0,1};
      if (it == rcAuList.begin() ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_DCI ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_SPS ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_VPS ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_PPS ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_PREFIX_APS ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_SUFFIX_APS )
      {
        /* From AVC, When any of the following conditions are fulfilled, the
         * zero_byte syntax element shall be present:
         *  - the nal_unit_type within the nal_unit() is equal to 7 (sequence
         *    parameter set) or 8 (picture parameter set),
         *  - the byte stream NAL unit syntax structure contains the first NAL
         *    unit of an access unit in decoding order, as specified by subclause
         *    7.4.1.2.3.
         */
        ::memcpy( rcAccessUnit.payload + iUsedSize, reinterpret_cast<const char*>(start_code_prefix), 4 );
        iUsedSize += 4;
      }
      else
      {
        ::memcpy( rcAccessUnit.payload + iUsedSize, reinterpret_cast<const char*>(start_code_prefix+1), 3 );
        iUsedSize += 3;
      }
      uint32_t nalDataSize = uint32_t(nalu.m_nalUnitData.str().size()) ;
      ::memcpy( rcAccessUnit.payload + iUsedSize, nalu.m_nalUnitData.str().c_str() , nalDataSize );
      iUsedSize += nalDataSize;

      if( nalu.m_nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_CRA ||
          nalu.m_nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_GDR )
      {
        rcAccessUnit.rap = true;
      }
      //rcAccessUnit.nalUnitTypeVec.push_back( nalu.m_nalUnitType );
    }

    rcAccessUnit.payloadUsedSize = iUsedSize;
    if( iUsedSize != sizeSum  )
    {
      return VVENC_NOT_ENOUGH_MEM;
    }

    //rcAccessUnit.annexBsizeVec   = annexBsizes;
    rcAccessUnit.ctsValid        = rcAuList.ctsValid;
    rcAccessUnit.dtsValid        = rcAuList.dtsValid;
    rcAccessUnit.cts             = rcAuList.cts;
    rcAccessUnit.dts             = rcAuList.dts;
    rcAccessUnit.sliceType       = (vvencSliceType)rcAuList.sliceType;
    rcAccessUnit.refPic          = rcAuList.refPic;
    rcAccessUnit.temporalLayer   = rcAuList.temporalLayer;
    rcAccessUnit.poc             = rcAuList.poc;
    rcAccessUnit.status          = rcAuList.status;

    if ( !rcAuList.InfoString.empty() )
    {
      if( rcAuList.InfoString.size() >= VVENC_MAX_STRING_LEN )
      {
        rcAuList.InfoString.copy( rcAccessUnit.infoString , VVENC_MAX_STRING_LEN-1 );
        rcAccessUnit.infoString[VVENC_MAX_STRING_LEN-1] = '\0';
      }
      else
      {
        rcAuList.InfoString.copy( rcAccessUnit.infoString , rcAuList.InfoString.size()+1 );
        rcAccessUnit.infoString[rcAuList.InfoString.size()] = '\0';
      }
    }
    else
    {
      rcAccessUnit.infoString[0] = '\0';
    }
  }

  return 0;
}


///< set message output function for encoder lib. if not set, no messages will be printed.
void VVEncImpl::registerMsgCbf( void * ctx, vvencLoggingCallback msgFnc )
{
  // DEPRECATED
  g_msgFnc    = msgFnc;
  g_msgFncCtx = ctx;
}

///< tries to set given simd extensions used. if not supported by cpu, highest possible extension level will be set and returned.
const char* VVEncImpl::setSIMDExtension( const char* simdId )
{
  const char* simdSet = NULL;
#if defined( TARGET_SIMD_X86 )
  std::string cSimdId ( simdId );
  simdSet = read_x86_extension( cSimdId );
#if ENABLE_SIMD_OPT_BUFFER
  g_pelBufOP.initPelBufOpsX86();
#endif
#if ENABLE_SIMD_TRAFO
  g_tCoeffOps.initTCoeffOpsX86();
#endif
#endif
  return simdSet;
}

///< creates compile info string containing OS, Compiler and Bit-depth (e.g. 32 or 64 bit).
std::string VVEncImpl::getCompileInfoString()
{
  std::string info;
  char convBuf[ 256 ];
  snprintf( convBuf, sizeof( convBuf ), NVM_ONOS );      info += convBuf;
  snprintf( convBuf, sizeof( convBuf ), NVM_COMPILEDBY); info += convBuf;
  snprintf( convBuf, sizeof( convBuf ), NVM_BITS );      info += convBuf;
  return info;
}

std::string VVEncImpl::createEncoderInfoStr()
{
  std::string cInfoStr;

    // Set SIMD extension in case if it hasn't been done before, otherwise it simply reuses the current state
  std::string curSimd;
  const char* pSimd = vvenc_set_SIMD_extension( curSimd.c_str() );
  pSimd == nullptr ? curSimd = "NA" : curSimd = pSimd;

  std::stringstream cssCap;
  cssCap << getCompileInfoString() << "[SIMD=" << curSimd <<"]";

  cInfoStr  = "Fraunhofer VVC Encoder ver. " VVENC_VERSION;
  cInfoStr += " ";
  cInfoStr += cssCap.str();

  return cInfoStr;
}

///< decode bitstream with limited build in decoder
int VVEncImpl::decodeBitstream( const char* FileName, const char* trcFile, const char* trcRule)
{
  int ret = 0;
  FFwdDecoder ffwdDecoder;
  Picture cPicture; cPicture.poc=-8000;
  MsgLog msg;

#if ENABLE_TRACING
  g_trace_ctx = tracing_init( trcFile, trcRule, msg );
#endif

  std::string filename(FileName );
#if HANDLE_EXCEPTION
  try
#endif
  {
    ret = tryDecodePicture( &cPicture, -1, filename, ffwdDecoder, nullptr, msg, false, cPicture.poc, false );
    if( ret )  
    { 
      msg.log( VVENC_ERROR, "decoding failed\n");
      return VVENC_ERR_UNSPECIFIED; 
    }
  }
#if HANDLE_EXCEPTION
  catch( std::exception& e )
  {
    msg.log( VVENC_ERROR, "decoding failed: %s\n", e.what() );
    return VVENC_ERR_UNSPECIFIED;
  }
#endif

#if ENABLE_TRACING
  tracing_uninit( g_trace_ctx );
#endif
  return ret;
}


} // namespace

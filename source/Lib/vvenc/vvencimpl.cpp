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

namespace vvenc {

#define ROTPARAMS(x, message) if(x) { rcErrorString = message; return VVENC_ERR_PARAMETER;}

// ====================================================================================================================

static_assert( sizeof(Pel)  == sizeof(*(YUVBuffer::Plane::ptr)),   "internal bits per pel differ from interface definition" );

// ====================================================================================================================

bool tryDecodePicture( Picture* pic, const int expectedPoc, const std::string& bitstreamFileName, FFwdDecoder& ffwdDecoder, ParameterSetMap<APS>* apsMap, bool bDecodeUntilPocFound = false, int debugPOC = -1, bool copyToEnc = true );

VVEncImpl::VVEncImpl()
{

}

VVEncImpl::~VVEncImpl()
{

}

int VVEncImpl::getConfig( VVEncCfg& rcVVEncCfg ) const
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  rcVVEncCfg = m_cVVEncCfg;
  return VVENC_OK;
}

int VVEncImpl::reconfig( const VVEncCfg& rcVVEncCfg )
{
  if( !m_bInitialized ){ return VVENC_ERR_INITIALIZE; }
  return VVENC_ERR_NOT_SUPPORTED;
}

int VVEncImpl::checkConfig( const VVEncCfg& rcVVEncCfg )
{
  VVEncCfg cVVEncCfgCopy = rcVVEncCfg;

  if ( cVVEncCfgCopy.initCfgParameter() )
  {
    return VVENC_ERR_INITIALIZE;
  }

  return VVENC_OK;
}

int VVEncImpl::init( const VVEncCfg& rcVVEncCfg, YUVWriterIf* pcYUVWriterIf )
{
  if( m_bInitialized ){ return VVENC_ERR_INITIALIZE; }

  // Set SIMD extension in case if it hasn't been done before, otherwise it simply reuses the current state
  std::string simdOpt;
  std::string curSimd = setSIMDExtension( simdOpt );

  m_cVVEncCfgExt = rcVVEncCfg;
  m_cVVEncCfg    = rcVVEncCfg;
  if ( m_cVVEncCfg.initCfgParameter() ) // init auto/dependent options
  {
    return VVENC_ERR_INITIALIZE;
  }

  std::stringstream cssCap;
  cssCap << getCompileInfoString() << "[SIMD=" << curSimd <<"]";
  m_sEncoderCapabilities = cssCap.str();

  // initialize the encoder
  m_pEncLib = new EncLib;

  try
  {
    m_pEncLib->initEncoderLib( m_cVVEncCfg, pcYUVWriterIf );
  }
  catch( std::exception& e )
  {
    m_cErrorString = e.what();
    return VVENC_ERR_UNSPECIFIED;
  }

  m_bInitialized = true;
  m_eState       = INTERNAL_STATE_INITIALIZED;
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

  if( pass > 1 && m_eState != INTERNAL_STATE_FINALIZED )
  {
    std::stringstream css;
    css << "initPass(" << pass << ") cannot initPass " << pass << " without having flushed the last pass. flush encoder till all frames are processed";
    m_cErrorString = css.str();
    return VVENC_ERR_INITIALIZE;
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

  m_eState = INTERNAL_STATE_INITIALIZED;
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
  m_eState       = INTERNAL_STATE_UNINITIALIZED;
  return VVENC_OK;
}

bool VVEncImpl::isInitialized() const
{
  return m_bInitialized;
}


int VVEncImpl::encode( YUVBuffer* pcYUVBuffer, AccessUnit& rcAccessUnit, bool& rbEncodeDone )
{
  if( !m_bInitialized )                      { return VVENC_ERR_INITIALIZE; }
  if( m_eState == INTERNAL_STATE_FINALIZED ) { m_cErrorString = "encoder already flushed, please reinit."; return VVENC_ERR_RESTART_REQUIRED; }

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

    if( m_cVVEncCfg.m_internChromaFormat != CHROMA_400 )
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

    if( m_cVVEncCfg.m_internChromaFormat != CHROMA_400 )
    {
      if( m_cVVEncCfg.m_internChromaFormat == CHROMA_444 )
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
  rcAccessUnit = AccessUnit();

  rbEncodeDone = false;

  AccessUnitList cAu;
  try
  {
    m_pEncLib->encodePicture( bFlush, pcYUVBuffer, cAu, rbEncodeDone );
  }
  catch( std::exception& e )
  {
    m_cErrorString = e.what();
    return VVENC_ERR_UNSPECIFIED;
  }

  if( rbEncodeDone )
  {
    if( m_eState == INTERNAL_STATE_FLUSHING )
    {
      m_eState = INTERNAL_STATE_FINALIZED;
    }
    else
    {
      rbEncodeDone = false;
    }
  }

  /* copy output AU */
  if ( !cAu.empty() )
  {
    iRet = xCopyAu( rcAccessUnit, cAu  );
  }

  return iRet;
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
  return m_cVVEncCfg.m_MCTFNumLeadFrames;
}

int VVEncImpl::getNumTrailFrames() const
{
  return m_cVVEncCfg.m_MCTFNumTrailFrames;
}

int VVEncImpl::printSummary() const
{
  if( !m_bInitialized ){ return -1; }
  if( nullptr == m_pEncLib )  { return -1; }

  m_pEncLib->printSummary();
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

///< creates compile info string containing OS, Compiler and Bit-depth (e.g. 32 or 64 bit).
std::string getCompileInfoString()
{
  char convBuf[ 256 ];
  std::string compileInfo;
  snprintf( convBuf, sizeof( convBuf ), NVM_ONOS );      compileInfo += convBuf;
  snprintf( convBuf, sizeof( convBuf ), NVM_COMPILEDBY); compileInfo += convBuf;
  snprintf( convBuf, sizeof( convBuf ), NVM_BITS );      compileInfo += convBuf;
  return compileInfo;
}

///< decode bitstream with limited build in decoder
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

int getWidthOfComponent( const ChromaFormat& chFmt, const int frameWidth, const int compId )
{
  int w = frameWidth;
  if ( compId > 0 )
  {
    switch ( chFmt )
    {
      case CHROMA_400: w = 0;      break;
      case CHROMA_420:
      case CHROMA_422: w = w >> 1; break;
      default: break;
    }
  }
  return w;
}

int getHeightOfComponent( const ChromaFormat& chFmt, const int frameHeight, const int compId )
{
  int h = frameHeight;
  if ( compId > 0 )
  {
    switch ( chFmt )
    {
      case CHROMA_400: h = 0;      break;
      case CHROMA_420: h = h >> 1; break;
      case CHROMA_422:
      default: break;
    }
  }
  return h;
}

} // namespace

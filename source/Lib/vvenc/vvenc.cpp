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

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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
  \file    vvenc.cpp
  \brief   This file contains the external interface of the vvenc SDK.
*/

#include "vvenc/vvenc.h"
#include "vvenc/vvencCfg.h"
#include "vvenc/version.h"

#include "vvencimpl.h"

VVENC_NAMESPACE_BEGIN

VVENC_DECL vvencYUVBuffer* vvenc_YUVBuffer_alloc()
{
  vvencYUVBuffer* yuvBuffer = (vvencYUVBuffer*)malloc(sizeof(vvencYUVBuffer));
  vvenc_YUVBuffer_default( yuvBuffer );
  return yuvBuffer;
}

VVENC_DECL void vvenc_YUVBuffer_free(vvencYUVBuffer *yuvBuffer, bool freePicBuffer )
{
  if( yuvBuffer )
  {
    if( freePicBuffer && yuvBuffer->planes[0].ptr )
    {
      vvenc_YUVBuffer_free_buffer ( yuvBuffer );
    }
    free(yuvBuffer);
  }
}

VVENC_DECL void vvenc_YUVBuffer_default(vvencYUVBuffer *yuvBuffer )
{
  for( int i = 0; i < 3; i ++ )
  {
    yuvBuffer->planes[i].ptr     = NULL;
    yuvBuffer->planes[i].width   = 0;
    yuvBuffer->planes[i].height  = 0;
    yuvBuffer->planes[i].stride  = 0;
  }
  yuvBuffer->sequenceNumber  = 0;
  yuvBuffer->cts             = 0;
  yuvBuffer->ctsValid        = false;
}


VVENC_DECL void vvenc_YUVBuffer_alloc_buffer( vvencYUVBuffer *yuvBuffer, const vvencChromaFormat chFmt, const int frameWidth, const int frameHeight )
{
  for ( int i = 0; i < 3; i++ )
  {
    vvencYUVPlane&    yuvPlane = yuvBuffer->planes[ i ];
    yuvPlane.width  = vvenc_get_width_of_component ( chFmt, frameWidth,  i );
    yuvPlane.height = vvenc_get_height_of_component( chFmt, frameHeight, i );
    yuvPlane.stride = yuvPlane.width;
    const int size  = yuvPlane.stride * yuvPlane.height;
    yuvPlane.ptr    = ( size > 0 ) ? new int16_t[ size ] : nullptr;
  }
}

VVENC_DECL void vvenc_YUVBuffer_free_buffer( vvencYUVBuffer *yuvBuffer )
{
  for ( int i = 0; i < 3; i++ )
  {
    if( yuvBuffer->planes[ i ].ptr )
      delete [] yuvBuffer->planes[ i ].ptr;
  }
}


VVENC_DECL vvencAccessUnit* vvenc_accessUnit_alloc()
{
  vvencAccessUnit* accessUnit = (vvencAccessUnit*)malloc(sizeof(vvencAccessUnit));
  vvenc_accessUnit_default( accessUnit );
  return accessUnit;
}

VVENC_DECL void vvenc_accessUnit_free(vvencAccessUnit *accessUnit, bool freePayload  )
{
  if( accessUnit )
  {
    if( freePayload && accessUnit->payload )
    {
      vvenc_accessUnit_free_payload ( accessUnit );
    }
    free(accessUnit);
  }
}

VVENC_DECL void vvenc_accessUnit_alloc_payload(vvencAccessUnit *accessUnit, int payload_size )
{
  accessUnit->payload = (unsigned char*)malloc(sizeof(unsigned char) * payload_size );
  accessUnit->payloadSize = payload_size;
  accessUnit->payloadUsedSize = 0;
}

VVENC_DECL void vvenc_accessUnit_free_payload(vvencAccessUnit *accessUnit )
{
  if( accessUnit->payload )
  {
    free(accessUnit->payload);
    accessUnit->payloadSize = 0;
    accessUnit->payloadUsedSize = 0;
  }
}

VVENC_DECL void vvenc_accessUnit_reset(vvencAccessUnit *accessUnit )
{
  accessUnit->payloadUsedSize = 0;
  accessUnit->cts             = 0;
  accessUnit->dts             = 0;
  accessUnit->ctsValid        = false;
  accessUnit->dtsValid        = false;
  accessUnit->rap             = false;
  accessUnit->sliceType       = VVENC_NUMBER_OF_SLICE_TYPES;
  accessUnit->refPic          = false;
  accessUnit->temporalLayer   = 0;
  accessUnit->poc             = 0;
  accessUnit->status          = 0;
  accessUnit->essentialBytes  = 0;

  memset( accessUnit->infoString, 0, sizeof( accessUnit->infoString ) );
  accessUnit->infoString[0]   ='\0';
}

VVENC_DECL void vvenc_accessUnit_default(vvencAccessUnit *accessUnit )
{
  accessUnit->payload         = NULL;
  accessUnit->payloadSize     = 0;
  vvenc_accessUnit_reset( accessUnit );
}

VVENC_DECL vvencEncoder* vvenc_encoder_create()
{
  vvenc::VVEncImpl* encCtx = new vvenc::VVEncImpl();
  if (!encCtx)
  {
    return nullptr;
  }

  return (vvencEncoder*)encCtx;
}


VVENC_DECL int vvenc_encoder_open( vvencEncoder *enc, vvenc_config* config )
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return VVENC_ERR_INITIALIZE;
  }

  int ret = e->init( config );
  if (ret != 0)
  {
    // Error initializing the decoder
    return VVENC_ERR_INITIALIZE;
  }

  return VVENC_OK;
}

VVENC_DECL int vvenc_encoder_close(vvencEncoder *enc)
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return VVENC_ERR_INITIALIZE;
  }

  int ret = e->uninit();
  delete e;

  return ret;
}

VVENC_DECL int vvenc_encoder_set_RecYUVBufferCallback(vvencEncoder *enc, void * ctx, vvencRecYUVBufferCallback callback )
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return VVENC_ERR_INITIALIZE;
  }

  e->setRecYUVBufferCallback( ctx, callback );
  return VVENC_OK;
}

VVENC_DECL int vvenc_init_pass( vvencEncoder *enc, int pass, const char * statsFName )
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return VVENC_ERR_INITIALIZE;
  }

  return e->initPass( pass, statsFName );
}


VVENC_DECL int vvenc_encode( vvencEncoder *enc, vvencYUVBuffer* YUVBuffer, vvencAccessUnit* accessUnit, bool* encodeDone )
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return VVENC_ERR_INITIALIZE;
  }

  return e->encode( YUVBuffer, accessUnit, encodeDone );
}

VVENC_DECL int vvenc_get_config( vvencEncoder *enc, vvenc_config* cfg )
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  return e->getConfig( *cfg );
}


VVENC_DECL int vvenc_reconfig( vvencEncoder *enc, const vvenc_config *cfg )
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  return e->reconfig( *cfg );
}

VVENC_DECL int vvenc_check_config( vvencEncoder *enc, const vvenc_config *cfg )
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  return e->checkConfig( *cfg );
}

VVENC_DECL int vvenc_get_headers(vvencEncoder *enc, vvencAccessUnit *accessUnit)
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  return e->getParameterSets( accessUnit );
}

VVENC_DECL const char* vvenc_get_last_error( vvencEncoder *enc )
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return NULL;
  }

  return e->getLastError();
}

VVENC_DECL const char* vvenc_get_enc_information( vvencEncoder *enc )
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    std::string info = vvenc::VVEncImpl::createEncoderInfoStr();
    char* defaultInfo = (char*)malloc(info.size()+1);
    strcpy(defaultInfo,info.c_str());
    return defaultInfo;
  }

  return e->getEncoderInfo();
}

VVENC_DECL int vvenc_get_num_lead_frames( vvencEncoder *enc )
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  return e->getNumLeadFrames();
}

VVENC_DECL int vvenc_get_num_trail_frames( vvencEncoder *enc )
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  return e->getNumTrailFrames();
}

VVENC_DECL int vvenc_print_summary( vvencEncoder *enc )
{
  auto e = (vvenc::VVEncImpl*)enc;
  if (!e)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  return e->printSummary();
}


VVENC_DECL const char* vvenc_get_version()
{
  return VVENC_VERSION;
}

VVENC_DECL const char* vvenc_get_error_msg( int nRet )
{
  return vvenc::VVEncImpl::getErrorMsg( nRet );
}


VVENC_DECL int vvenc_set_logging_callback( void * ctx, vvencLoggingCallback callback )
{
 // DEPRECATED
  vvenc::VVEncImpl::registerMsgCbf ( ctx, callback );
  return VVENC_OK;
}


VVENC_DECL const char* vvenc_set_SIMD_extension( const char* simdId )
{
  return vvenc::VVEncImpl::setSIMDExtension( simdId );
}


///< checks if library has tracing supported enabled (see ENABLE_TRACING).
VVENC_DECL bool vvenc_is_tracing_enabled()
{
#if ENABLE_TRACING
  return true;
#else
  return false;
#endif
}

// creates compile info string containing OS, Compiler and Bit-depth (e.g. 32 or 64 bit).
VVENC_DECL const char* vvenc_get_compile_info_string()
{
  return vvenc::VVEncImpl::getCompileInfoString();
}
VVENC_DECL int vvenc_decode_bitstream( const char* FileName, const char* trcFile, const char* trcRule)
{
  return vvenc::VVEncImpl::decodeBitstream( FileName, trcFile, trcRule );
}

VVENC_DECL int vvenc_get_width_of_component( const vvencChromaFormat chFmt, const int frameWidth, const int compId )
{
  int w = frameWidth;
  if ( compId > 0 )
  {
    switch ( chFmt )
    {
      case VVENC_CHROMA_400: w = 0;      break;
      case VVENC_CHROMA_420:
      case VVENC_CHROMA_422: w = w >> 1; break;
      default: break;
    }
  }
  return w;
}

VVENC_DECL int vvenc_get_height_of_component( const vvencChromaFormat chFmt, const int frameHeight, const int compId )
{
  int h = frameHeight;
  if ( compId > 0 )
  {
    switch ( chFmt )
    {
      case VVENC_CHROMA_400: h = 0;      break;
      case VVENC_CHROMA_420: h = h >> 1; break;
      case VVENC_CHROMA_422:
      default: break;
    }
  }
  return h;
}

VVENC_NAMESPACE_END

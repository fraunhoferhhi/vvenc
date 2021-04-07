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
  \file    vvenc.cpp
  \brief   This file contains the external interface of the vvenc SDK.
*/

#include "vvenc/vvenc.h"
#include "vvenc/vvencCfg.h"
#include "vvencimpl.h"

#ifdef __cplusplus
extern "C" {
#endif

VVENC_NAMESPACE_BEGIN


//YUVBufferStorage::YUVBufferStorage( const vvencChromaFormat& chFmt, const int frameWidth, const int frameHeight )
//  : YUVBuffer()
//{
//  for ( int i = 0; i < VVENC_MAX_NUM_COMP; i++ )
//  {
//    YUVBuffer::Plane& yuvPlane = planes[ i ];
//    yuvPlane.width  = vvenc_getWidthOfComponent ( chFmt, frameWidth,  i );
//    yuvPlane.height = vvenc_getHeightOfComponent( chFmt, frameHeight, i );
//    yuvPlane.stride = yuvPlane.width;
//    const int size  = yuvPlane.stride * yuvPlane.height;
//    yuvPlane.ptr    = ( size > 0 ) ? new int16_t[ size ] : nullptr;
//  }
//}

//YUVBufferStorage::~YUVBufferStorage()
//{
//  for ( int i = 0; i < VVENC_MAX_NUM_COMP; i++ )
//  {
//    delete [] planes[ i ].ptr;
//  }
//}

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
    yuvBuffer->planes[i].ptr     = NULL;          // pointer to plane buffer
    yuvBuffer->planes[i].width   = 0;             // width of the plane
    yuvBuffer->planes[i].height  = 0;             // height of the plane
    yuvBuffer->planes[i].stride  = 0;             // stride (width + left margin + right margins) of plane in samples
  }
  yuvBuffer->sequenceNumber  = 0;                 // sequence number of the picture
  yuvBuffer->cts             = 0;                 // composition time stamp in TicksPerSecond (see HEVCEncoderParameter)
  yuvBuffer->ctsValid        = false;             // composition time stamp valid flag (true: valid, false: CTS not set)
}


VVENC_DECL void vvenc_YUVBuffer_alloc_buffer( vvencYUVBuffer *yuvBuffer, const vvencChromaFormat& chFmt, const int frameWidth, const int frameHeight )
{
  for ( int i = 0; i < 3; i++ )
  {
    vvencYUVPlane&    yuvPlane = yuvBuffer->planes[ i ];
    yuvPlane.width  = vvenc_getWidthOfComponent ( chFmt, frameWidth,  i );
    yuvPlane.height = vvenc_getHeightOfComponent( chFmt, frameHeight, i );
    yuvPlane.stride = yuvPlane.width;
    const int size  = yuvPlane.stride * yuvPlane.height;
    yuvPlane.ptr    = ( size > 0 ) ? new int16_t[ size ] : nullptr;
  }
}

VVENC_DECL void vvenc_YUVBuffer_free_buffer( vvencYUVBuffer *yuvBuffer )
{
  for ( int i = 0; i < 3; i++ )
  {
    delete [] yuvBuffer->planes[ i ].ptr;
  }
}


VVENC_DECL vvencAccessUnit* vvenc_accessUnit_alloc()
{
  vvencAccessUnit* accessUnit = (vvencAccessUnit*)malloc(sizeof(vvencAccessUnit));
  vvenc_accessUnit_default( accessUnit );
  return accessUnit;
}

VVENC_DECL void vvenc_accessUnit_free(vvencAccessUnit *accessUnit )
{
  if( accessUnit )
  {
    if( accessUnit->payload )
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

VVENC_DECL void vvenc_accessUnit_default(vvencAccessUnit *accessUnit )
{
  accessUnit->payload         = NULL;         ///< pointer to buffer that retrieves the coded data,
  accessUnit->payloadSize     = 0;            ///< size of the allocated buffer in bytes
  accessUnit->payloadUsedSize = 0;            ///< length of the coded data in bytes
  accessUnit->cts             = 0;            ///< composition time stamp in TicksPerSecond (see VVCDecoderParameter)
  accessUnit->dts             = 0;            ///< decoding time stamp in TicksPerSecond (see VVCDecoderParameter)
  accessUnit->ctsValid        = false;        ///< composition time stamp valid flag (true: valid, false: CTS not set)
  accessUnit->dtsValid        = false;        ///< decoding time stamp valid flag (true: valid, false: DTS not set)
  accessUnit->rap             = false;        ///< random access point flag (true: AU is random access point, false: sequential access)
  accessUnit->sliceType     = VVENC_NUMBER_OF_SLICE_TYPES; ///< slice type (I/P/B) */
  accessUnit->refPic        = false;         ///< reference picture
  accessUnit->temporalLayer = 0;             ///< temporal layer
  accessUnit->poc           = 0;             ///< picture order count

  accessUnit->status        = 0;        ///< additional info (see Status)
  //accessUnit->infoString;                    ///< debug info from inside the encoder

}


VVENC_DECL vvencEncoder* vvenc_encoder_open( VVEncCfg* config )
{
  if (nullptr == config)
  {
    vvenc::msg( VVENC_ERROR, "vvdec_Params_t is null\n" );
    return nullptr;
  }

  vvenc::VVEncImpl* encCtx = new vvenc::VVEncImpl();
  if (!encCtx)
  {
    vvenc::msg( VVENC_ERROR, "cannot allocate memory for VVdeC decoder\n" );
    return nullptr;
  }

  int ret = encCtx->init(*config, NULL );
  if (ret != 0)
  {
    // Error initializing the decoder
    delete encCtx;

    vvenc::msg( VVENC_ERROR, "cannot init the VVdeC decoder\n" );
    return nullptr;
  }

  return (vvencEncoder*)encCtx;
}

VVENC_DECL int vvenc_encoder_close(vvencEncoder *enc)
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return VVENC_ERR_INITIALIZE;
  }

  d->uninit();

  delete d;

  return VVENC_OK;
}

VVENC_DECL int vvenc_encoder_set_YUVWriterCallback(vvencEncoder *enc, vvencYUVWriterCallback callback )
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return VVENC_ERR_INITIALIZE;
  }

  //d->setYUVWriterCallback( callback );
  return VVENC_OK;
}

VVENC_DECL int vvenc_init_pass( vvencEncoder *enc, int pass )
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return VVENC_ERR_INITIALIZE;
  }

  d->initPass( pass );
  return VVENC_OK;
}


VVENC_DECL int vvenc_encode( vvencEncoder *enc, vvencYUVBuffer* YUVBuffer, vvencAccessUnit** accessUnit, bool* encodeDone )
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return VVENC_ERR_INITIALIZE;
  }

  d->encode( YUVBuffer, accessUnit, encodeDone );
  return VVENC_OK;
}

VVENC_DECL int vvenc_getConfig( vvencEncoder *enc, VVEncCfg* cfg )
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  d->getConfig( *cfg );
  return VVENC_OK;
}


VVENC_DECL int vvenc_reconfig( vvencEncoder *enc, const VVEncCfg *cfg )
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  d->reconfig( *cfg );
  return VVENC_OK;
}

VVENC_DECL int vvenc_checkConfig( vvencEncoder *enc, const VVEncCfg *cfg )
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  d->checkConfig( *cfg );
  return VVENC_OK;
}

VVENC_DECL const char* vvenc_getLastError( vvencEncoder *enc )
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return NULL;
  }

  return d->getLastError().c_str();
}

VVENC_DECL const char* vvenc_getEncoderInfo( vvencEncoder *enc )
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return NULL;
  }

  return d->getEncoderInfo();
}

VVENC_DECL int vvenc_getNumLeadFrames( vvencEncoder *enc )
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  return d->getNumLeadFrames();
}

VVENC_DECL int vvenc_getNumTrailFrames( vvencEncoder *enc )
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  return d->getNumTrailFrames();
}

VVENC_DECL int vvenc_printSummary( vvencEncoder *enc )
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  d->printSummary();
  return VVENC_OK;
}


VVENC_DECL const char* vvenc_getVersionNumber()
{

}

VVENC_DECL const char* vvenc_getErrorMsg( int nRet )
{
  return vvenc::VVEncImpl::getErrorMsg( nRet );
}


VVENC_DECL int vvenc_set_logging_callback(vvencEncoder *enc, vvencLoggingCallback callback )
{
  auto d = (vvenc::VVEncImpl*)enc;
  if (!d)
  {
    return VVENC_ERR_UNSPECIFIED;
  }

  d->setLoggingCallback(callback );
  return VVENC_OK;
}


VVENC_DECL std::string vvenc_set_SIMD_extension( const char* simdId );


///< checks if library has tracing supported enabled (see ENABLE_TRACING).
VVENC_DECL bool vvenc_isTracingEnabled()
{
#if ENABLE_TRACING
  return true;
#else
  return false;
#endif
}

VVENC_DECL const char* vvenc_getCompileInfoString(); // creates compile info string containing OS, Compiler and Bit-depth (e.g. 32 or 64 bit).
VVENC_DECL void   vvenc_decodeBitstream( const char* FileName);

VVENC_DECL int  vvenc_getWidthOfComponent( const vvencChromaFormat& chFmt, const int frameWidth, const int compId );
VVENC_DECL int  vvenc_getHeightOfComponent( const vvencChromaFormat& chFmt, const int frameHeight, const int compId );



int vvenc_getWidthOfComponent( const vvencChromaFormat& chFmt, const int frameWidth, const int compId )
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

int vvenc_getHeightOfComponent( const vvencChromaFormat& chFmt, const int frameHeight, const int compId )
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

#ifdef __cplusplus
};
#endif

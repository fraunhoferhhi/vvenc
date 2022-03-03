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
  \file    vvencinterfacetest.c
  \brief   This vvencinterfacetest.c file contains a simple sample that shows basic encoder interface functionality.
*/

#include <stdio.h>
#include <stdarg.h>

#include "vvenc/vvenc.h"
#include "vvenc/vvencCfg.h"
#include "vvenc/version.h"

void msgFnc( void* ctx, int level, const char* fmt, va_list args )
{
  vfprintf( level == 1 ? stderr : stdout, fmt, args );
}


void msgApp( int level, const char* fmt, ... )
{
    va_list args;
    va_start( args, fmt );
    msgFnc( NULL, level, fmt, args );
    va_end( args );
}


int run( vvenc_config* vvencCfg, int maxFrames )
{
  int iRet = 0;
  vvencEncoder *enc = NULL;        // encoder handler

  vvencYUVBuffer  cYUVInputBuffer;  // input picture storage
  vvencYUVBuffer* ptrYUVInputBuffer = NULL;

  vvencAccessUnit AU;  // access unint storage

  // create the encoder
  enc = vvenc_encoder_create();
  if( NULL == enc )
  {
    printf("cannot create encoder\n");
    return -1;
  }

  // initialize the encoder
  iRet = vvenc_encoder_open( enc, vvencCfg );
  if( 0 != iRet )
  {
    printf("cannot open encoder. ret: %d, %s\n", iRet, vvenc_get_last_error( enc ) );
    return iRet;
  }

  // get the adapted config, because changes are needed for the yuv reader
  // (uninitialized parameter may be set during vvenc_encoder_open)
  vvenc_get_config( enc, vvencCfg );

  msgApp( VVENC_INFO,"%s\n", vvenc_get_config_as_string( vvencCfg, vvencCfg->m_verbosity) );

  // --- allocate memory for YUV input picture
  vvenc_YUVBuffer_default( &cYUVInputBuffer );
  vvenc_YUVBuffer_alloc_buffer( &cYUVInputBuffer, vvencCfg->m_internChromaFormat, vvencCfg->m_SourceWidth, vvencCfg->m_SourceHeight );

  // inititialize yuv input buffer
  for( int comp = 0; comp < 3; comp++ )
  {
    memset( cYUVInputBuffer.planes[ comp ].ptr, 512, cYUVInputBuffer.planes[comp].width*cYUVInputBuffer.planes[comp].height*sizeof(int16_t));
  }

  // --- allocate memory for output packets
  vvenc_accessUnit_default( &AU );
  vvenc_accessUnit_alloc_payload( &AU, vvencCfg->m_SourceWidth * vvencCfg->m_SourceHeight );

  // run encoder loop
  int AuCount = 0;
  bool encodeDone = false;
  for( int frame = 0; frame < maxFrames; frame++ )
  {
    int iWhere = frame % vvencCfg->m_SourceHeight;
    for( int comp = 0; comp < 3; comp++ )
    {
      memset( cYUVInputBuffer.planes[ comp ].ptr+iWhere, frame, cYUVInputBuffer.planes[comp].width*sizeof(int16_t));
    }
    ptrYUVInputBuffer = &cYUVInputBuffer; // assign dummy input buffer
    iRet = vvenc_encode( enc, ptrYUVInputBuffer, &AU, &encodeDone );
    if( 0 != iRet )
    {
      printf("encoding failed. ret: %d, %s\n", iRet, vvenc_get_last_error( enc ) );
      goto cleanup;
    }

    if( AU.payloadUsedSize > 0 && !encodeDone )
    {
      // write bitstream output: AU.payload, AU.payloadUsedSize
      AuCount++;
    }
    else if ( AuCount > 0 )
    {
      printf("expecting Au, but receive empty payload (frame %d/%d, payloads rcv %d)\n", frame, maxFrames, AuCount);
      goto cleanup;
    }
  }

  // flushing encoder
  while( !encodeDone )
  {
    ptrYUVInputBuffer = NULL;
    iRet = vvenc_encode( enc, ptrYUVInputBuffer, &AU, &encodeDone );
    if( 0 != iRet )
    {
      printf("encoding failed. ret: %d, %s\n", iRet, vvenc_get_last_error( enc ) );
      goto cleanup;
    }

    if( AU.payloadUsedSize > 0 )
    {
      // write bitstream output: AU.payload, AU.payloadUsedSize
      AuCount++;
    }
    else if ( AuCount > 0 && !encodeDone )
    {
      printf("expecting Au on flush, but receive empty payload (frame %d/%d, payloads rcv %d)\n",  maxFrames, maxFrames, AuCount);
      goto cleanup;
    }
  }

  vvenc_print_summary(enc);

cleanup:

  // free allocated memory
  vvenc_YUVBuffer_free_buffer( &cYUVInputBuffer );
  vvenc_accessUnit_free_payload( &AU );

  // un-initialize the encoder
  if( 0 != vvenc_encoder_close( enc ) )
  {
    printf("close encoder failed. ret: %d, %s", iRet, vvenc_get_last_error( enc ) );
    return -1;
  }

  return iRet;
}

int main( int argc, char* argv[] )
{
  vvenc_config vvencCfg;

  int width  = 320;
  int height = 240;
  int fps    = 60;
  int qp     = 32;
  int bitrate = 0;
  vvencPresetMode preset  = VVENC_FASTER;
  vvencMsgLevel verbosity = VVENC_WARNING;
  int maxFrames = 16;  // max frames to encode

  // init test without multi threading
  vvenc_init_default( &vvencCfg, width, height, fps, bitrate, qp, preset );
  vvencCfg.m_verbosity = verbosity;
  vvenc_set_msg_callback( &vvencCfg, NULL, &msgFnc );

  vvencCfg.m_numThreads = 0;

  if( 0 != run( &vvencCfg, maxFrames ))
  {
    return -1;
  }

  // init test multi threading
  vvenc_init_default( &vvencCfg, width, height, fps, bitrate, qp, preset );
  vvencCfg.m_verbosity = verbosity;
  vvenc_set_msg_callback( &vvencCfg, NULL, &msgFnc );

  if( 0 != run( &vvencCfg, maxFrames ))
  {
    return -1;
  }



}


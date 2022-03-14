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

int run( vvenc_config* vvencCfg, int maxFrames, bool runTillFlushed )
{
  int iRet = 0;
  vvencEncoder *enc = NULL;        // encoder handler

  vvencYUVBuffer  cYUVInputBuffer;  // input picture storage
  vvencYUVBuffer* ptrYUVInputBuffer = NULL;

  vvencAccessUnit AU;  // access unint storage

  bool encodeDone = false;
  int auCount = 0;
  int auCountExpected = maxFrames;

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

  if( vvencCfg->m_verbosity >= VVENC_INFO )
  {
    msgApp( VVENC_INFO,"%s\n", vvenc_get_config_as_string( vvencCfg, vvencCfg->m_verbosity) );
  }

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

  for( int frame = 0; frame < maxFrames; frame++ )
  {
    int iWhere = frame % vvencCfg->m_SourceHeight;
    for( int comp = 0; comp < 3; comp++ )
    {
      memset( cYUVInputBuffer.planes[ comp ].ptr+iWhere, frame, cYUVInputBuffer.planes[comp].width*sizeof(int16_t));
    }
    ptrYUVInputBuffer = &cYUVInputBuffer; // assign dummy input buffer
    ptrYUVInputBuffer->sequenceNumber = frame;

    //printf("vvenc_encode seq: %ld\n", ptrYUVInputBuffer->sequenceNumber );
    iRet = vvenc_encode( enc, ptrYUVInputBuffer, &AU, &encodeDone );
    if( 0 != iRet )
    {
      printf("encoding failed. ret: %d, %s\n", iRet, vvenc_get_last_error( enc ) );
      goto cleanup;
    }

    if( AU.payloadUsedSize > 0 && !encodeDone )
    {
      // write bitstream output: AU.payload, AU.payloadUsedSize
      auCount++;
    }
    else if ( auCount > 0 )
    {
      printf("expecting Au, but receive empty payload (frame %d/%d, payloads rcv %d)\n", frame, maxFrames, auCount);
      iRet=-1;
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
      auCount++;
    }
    else if ( auCount > 0 && !encodeDone )
    {
      printf("expecting Au on flush, but receive empty payload (frame %d/%d, payloads rcv %d)\n",  maxFrames, maxFrames, auCount);
      iRet=-1;
      goto cleanup;
    }

    if( !runTillFlushed && auCount > auCountExpected )
    {
      //printf("cancel after retrieving %d AU´s for test purpose\n", auCount );
      break;
    }
  }

  if( vvencCfg->m_verbosity >= VVENC_INFO )
  {
    vvenc_print_summary(enc);
  }

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

  if( runTillFlushed && auCount != auCountExpected )
  {
    printf("expecting %d Au´s but only retrieve %d", auCountExpected, auCount);
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
  int maxFrames = 16;

  // ---------------------------------------- 1 ------------------------------
  // init test run without multi threading
  vvenc_init_default( &vvencCfg, width, height, fps, bitrate, qp, preset );
  vvencCfg.m_verbosity = verbosity;
  vvenc_set_msg_callback( &vvencCfg, NULL, &msgFnc );

  vvencCfg.m_numThreads = 0;

  if( 0 != run( &vvencCfg, maxFrames, true ))
  {
    return -1;
  }

  // ---------------------------------------- 2 ------------------------------
  // init test run with multi threading
  vvenc_init_default( &vvencCfg, width, height, fps, bitrate, qp, preset );
  vvencCfg.m_verbosity = verbosity;
  vvenc_set_msg_callback( &vvencCfg, NULL, &msgFnc );

  if( 0 != run( &vvencCfg, maxFrames, true ))
  {
    return -1;
  }

  // ---------------------------------------- 3 ------------------------------
  // init test run with multi threading, (2*GOPSize)+8 franes
  vvenc_init_default( &vvencCfg, width, height, fps, bitrate, qp, preset );
  vvencCfg.m_verbosity = verbosity;
  vvenc_set_msg_callback( &vvencCfg, NULL, &msgFnc );

  maxFrames = (2 * vvencCfg.m_GOPSize)+8;
  if( 0 != run( &vvencCfg, maxFrames, true ))
  {
    return -1;
  }


  // ---------------------------------------- 4 ------------------------------
  // init test run with 1pass RC, lookahead, multi threading
  bitrate = 500000;
  vvenc_init_default( &vvencCfg, width, height, fps, bitrate, qp, preset );
  vvencCfg.m_verbosity = verbosity;
  vvenc_set_msg_callback( &vvencCfg, NULL, &msgFnc );

  vvencCfg.m_RCNumPasses = 1;

  maxFrames = (2 * vvencCfg.m_GOPSize)+8;

  if( 0 != run( &vvencCfg, maxFrames, false ))
  {
    return -1;
  }


#if 0 // only for development purposes
  // ---------------------------------------- 5 ------------------------------
  // init test run with 1pass RC, lookahead, multi threading + different frame count
  bitrate = 500000;
  vvenc_init_default( &vvencCfg, width, height, fps, bitrate, qp, preset );
  vvencCfg.m_verbosity = verbosity;
  vvenc_set_msg_callback( &vvencCfg, NULL, &msgFnc );

  vvencCfg.m_RCNumPasses = 1;

  maxFrames = vvencCfg.m_GOPSize;
  while ( maxFrames < 128 )
  {
    printf("encoding %d frames\n", maxFrames);
    if( 0 != run( &vvencCfg, maxFrames, false ))
    {
      printf("error when encoding %d frames\n", maxFrames);
      return -1;
    }
    maxFrames++;
  }
#endif
}
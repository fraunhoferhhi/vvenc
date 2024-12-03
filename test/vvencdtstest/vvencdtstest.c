/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
  \file    vvencdtstest.c
  \brief   This vvencdtstest.c file checks the DTS that the encoder outputs are
           monotonic increasing.
*/

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "vvenc/vvenc.h"
#include "vvenc/vvencCfg.h"

#define WIDTH 64
#define HEIGHT 64
#define FRAMES 20

#ifdef NDEBUG
#error "Please build this test with NDEBUG undefined"
#endif

static void init_in_buf( vvencYUVBuffer *in_buf, uint64_t seq );
static void init_out_buf( vvencAccessUnit *out_buf );
static bool check_dts( const vvencAccessUnit *out_buf, int64_t last_dts );

int main( int argc, char* argv[] )
{
  vvencAccessUnit out_buf;
  vvencYUVBuffer in_buf;
  bool success = true;
  vvencEncoder *enc;
  uint64_t last_dts;
  vvenc_config cfg;
  bool enc_done;
  int ret;

  // Config
  vvenc_init_default( &cfg, WIDTH, HEIGHT, 30, 0, VVENC_DEFAULT_QP, VVENC_FAST );
  cfg.m_TicksPerSecond = -1;
  enc = vvenc_encoder_create();
  assert( enc );
  ret = vvenc_encoder_open( enc, &cfg );
  assert( ret == 0 );

  // Encode
  init_out_buf( &out_buf );
  last_dts = -10;
  for ( uint64_t i = 0, seq = 0; i < FRAMES; i++, seq++ ) {
    init_in_buf( &in_buf, seq );
    ret = vvenc_encode( enc, &in_buf, &out_buf, &enc_done );
    assert( ret == 0 );

    if ( out_buf.payloadUsedSize > 0 ) {
      success = check_dts( &out_buf, last_dts ) && success;
      last_dts = out_buf.dts;
    }

    if ( i == FRAMES / 2 )
      seq += 10; // Jump cts forward
  }

  // Drain
  do {
    ret = vvenc_encode( enc, NULL, &out_buf, &enc_done );
    assert( ret == 0 );

    success = check_dts( &out_buf, last_dts ) && success;
    last_dts = out_buf.dts;
  } while ( !enc_done && (out_buf.payloadUsedSize > 0) );

  vvenc_encoder_close( enc );
  return success? EXIT_SUCCESS : EXIT_FAILURE;
}


static void init_in_buf( vvencYUVBuffer *in_buf, uint64_t seq )
{
  static int16_t plane[WIDTH * HEIGHT];

  in_buf->planes[0].ptr = plane;
  in_buf->planes[0].width = WIDTH;
  in_buf->planes[0].height = HEIGHT;
  in_buf->planes[0].stride = WIDTH;

  in_buf->planes[1].ptr = plane;
  in_buf->planes[1].width = WIDTH / 2;
  in_buf->planes[1].height = HEIGHT / 2;
  in_buf->planes[1].stride = WIDTH / 2;

  in_buf->planes[2].ptr = plane;
  in_buf->planes[2].width = WIDTH / 2;
  in_buf->planes[2].height = HEIGHT / 2;
  in_buf->planes[2].stride = WIDTH / 2;

  in_buf->sequenceNumber = seq;
  in_buf->cts = seq;
  in_buf->ctsValid = true;
}

static void init_out_buf( vvencAccessUnit *out_buf )
{
  static unsigned char payload[WIDTH * HEIGHT];

  out_buf->payload = payload;
  out_buf->payloadSize = WIDTH * HEIGHT;
}

static bool check_dts( const vvencAccessUnit *out_buf, int64_t last_dts )
{
  printf( "% 3ld", out_buf->dts );

  if ( (int64_t) out_buf->dts < last_dts ) {
    printf( "\t[jump backwards]\n" );
    return false;
  }

  printf( "\n" );
  return true;
}

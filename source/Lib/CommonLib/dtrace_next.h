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
/** \file     dtrace_next.h
 *  \brief    DTrace support for next software
 */

#pragma once

#include "dtrace.h"
#include "CommonDef.h"
#include "Rom.h"

#include "Utilities/MsgLog.h"

#include <cmath>

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if ENABLE_TRACING

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// DTRACE SHORT MANUAL
//
// 1. General info
//
// DTrace is a simple tracer that can be controlled at command line of the executable.
//
// Please have a look into command line parameter options to find the correspondent DTrace parameters.
// There are only two command-line parameters: tracing file and tracing rule.
// At initialization, DTrace-module parses the rule and set up the tracing context.
// The tracing context is stored in a global variable. All trace-outputs should use this global context.
//
// 2. Parameters
// 2.1 Tracing file (--TraceFile)
//
// Just a filename for a text-file.
// E.g.: --TraceFile="tracefile_rec.txt"
//
// 2.2 Tracing rule (--TraceRule)
//
// Tracing rule describes when during a runtime a particular output should be activated.
// Tracing rule consists of tracing channel(s) and tracing condition(s).
// The construction of the rule is: "channel_1,channel_2,...:condition_1,condition_2,..."
//
// Example for a tracing rule: --TraceRule="D_CABAC:poc==0"
// Here, tracing channel is D_CABAC and condition is poc==0, which means CABAC tracing output is activated at POC 0 only.
// You can also use poc>=2 or set the range like poc>=0,poc<=3 for example.
//
//
// 2.2.1 Tracing channel
//
// Channels are defined in dtrace_next.h. Users can add their own channels.
// Just put the channel definition into enum-list AND add it to the next_channels-table in the function tracing_init().
//
// 2.2.2 Tracing condition
//
// Currently supported: poc (the currently processed poc) and final ('0' if currently in RD-search, '1' if the current state is final)
// NOTE: Conditions are added and updated during runtime through DTRACE_UPDATE(...).
// It updates the DTrace internal state, so channels can be activated at the right moment.
// If it's not updated properly (at right place in the code, e.g. too late), the trace output can become wrong.
// For example, "poc"-condition should be updated at the start of the picture(AccesUnit).
// Please look into source code for how the "poc"-condition is used.
//
// 3. Using of DTrace macros
//
// The most used macro is DTRACE. It's like a printf-function with some additional parameters at the beginning.
// Format:
// DTRACE( tracing context, tracing channel, "..." [,params] );
// Example:
// DTRACE( g_trace_ctx, D_CABAC, "EP=%d \n", bin );
// There are also macros for output of buffers, picture components or conditional-outputs available. Please have a look into dtrace_next.h.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// DTrace channels
// Use already defined channels or add your own tracing channels

enum DTRACE_CHANNEL
{
  D_COMMON,
  D_BITSTREAM,
  D_HEADER,               // header file infos
  D_NALUNITHEADER,        // NAL unit header infos
  D_RPSINFO,              // bits used to send the RPS
  D_REC_CB_LUMA,          // reconstructed coding block luma pixel
  D_REC_CB_CHROMA,        // reconstructed coding block chroma pixel
  D_REC_CB_LUMA_LF,       // reconstructed coding block luma pixel after deblocking filter
  D_REC_CB_CHROMA_LF,     // reconstructed coding block chroma pixel after deblocking filter
  D_REC_CB_LUMA_SAO,      // reconstructed coding block luma pixel after SAO filter
  D_REC_CB_CHROMA_SAO,    // reconstructed coding block chroma pixel after SAO filter
  D_REC_CB_LUMA_ALF,      // reconstructed coding block luma pixel after ALF filter
  D_REC_CB_CHROMA_ALF,    // reconstructed coding block chroma pixel after ALF filter
  D_ME,                   // Motion Estimation
  D_CABAC,                // CABAC engine
  D_SYNTAX,               // syntax
  D_SYNTAX_RESI,          // syntax of the residual coding
  D_BEST_MODE,            // Cost for coding mode (encoder only)
  D_MODE_COST,            // Cost for coding mode (encoder only)
  D_QP_PRED,              // QP Prediction for DQP process
  D_DQP,                  // Delta QP read/write
  D_QP,                   // final CU QP at reading/writing stage
  D_QP_PER_CTU,           // final QP per CTU at reading
  D_MISC,                 // Miscellaneous
  D_TU_ABS_SUM,
  D_EST_FRAC_BITS,
  D_INTRA_COST,           //intra cost
  D_PRED,
  D_RESIDUALS,
  D_TCOEFF,
  D_RDOQ,
  D_RDOQ_MORE,
  D_RDOQ_COST,
  D_TMP,
  D_MOT_FIELD,
  D_MOT_COMP,             // Motion compensation
  D_ALF,
  D_CRC
};
#define _CNL_DEF(_s) {_s,(std::string(#_s))}

inline void tracing_uninit( CDTrace *pDtrace )
{
  if( pDtrace )
    delete pDtrace;
}


template< typename Tsrc >
void dtrace_block( CDTrace *trace_ctx, DTRACE_CHANNEL channel, Tsrc *buf, unsigned stride, unsigned block_w, unsigned block_h )
{
  unsigned i, j;
  for( j = 0; j < block_h; j++ )
  {
    for( i = 0; i < block_w; i++ )
    {
//      trace_ctx->dtrace<false>( channel, "%04x ", buf[j*stride + i] );
      trace_ctx->dtrace<false>( channel, "%4d ", buf[j*stride + i] );
    }
    trace_ctx->dtrace<false>( channel, "\n" );
  }
  trace_ctx->dtrace<false>( channel, "\n" );
}

template< typename Tsrc >
void dtrace_frame_blockwise( CDTrace *trace_ctx, DTRACE_CHANNEL channel, Tsrc *buf, unsigned stride, unsigned frm_w, unsigned frm_h, unsigned block_w, unsigned block_h )
{
  unsigned i, j, block;
  for( j = 0, block = 0; j < frm_h; j += block_h )
  {
    unsigned blockhf = std::min( block_h, frm_h - j);
    Tsrc *p_buf = buf + j*stride;
    for( i = 0; i < frm_w; i += block_w, block++ )
    {
      unsigned blockwf = std::min( block_w, frm_w - i);

      trace_ctx->dtrace<false>( channel, "Frame BLOCK=%d (x,y) = (%d, %d)\n", block, i, j );
      dtrace_block( trace_ctx, channel, p_buf, stride, blockwf, blockhf );
      p_buf += block_w;
    }
  }
}

#define DTRACE(ctx,channel,...)              ctx->dtrace<true>( channel, __VA_ARGS__ )
#define DTRACE_WITHOUT_COUNT(ctx,channel,...) ctx->dtrace<false>( channel, __VA_ARGS__ )
#define DTRACE_DECR_COUNTER(ctx,channel)     ctx->decrementChannelCounter( channel )
#define DTRACE_UPDATE(ctx,s)                 if((ctx)){(ctx)->update((s));}
#define DTRACE_REPEAT(ctx,channel,times,...) ctx->dtrace_repeat( channel, times,__VA_ARGS__ )
#define DTRACE_COND(cond,ctx,channel,...)    { if( cond ) ctx->dtrace<true>( channel, __VA_ARGS__ ); }
#define DTRACE_BLOCK(...)                    dtrace_block(__VA_ARGS__)
#define DTRACE_FRAME_BLOCKWISE(...)          dtrace_frame_blockwise(__VA_ARGS__)
#define DTRACE_GET_COUNTER(ctx,channel)      ctx->getChannelCounter(channel)

inline CDTrace* tracing_init( const std::string& sTracingFile, const std::string& sTracingRule, MsgLog& msg )
{
  dtrace_channel next_channels[] =
  {
    _CNL_DEF( D_COMMON ),
    _CNL_DEF( D_BITSTREAM ),
    _CNL_DEF( D_HEADER ),
    _CNL_DEF( D_NALUNITHEADER ),
    _CNL_DEF( D_RPSINFO ),
    _CNL_DEF( D_REC_CB_LUMA ),
    _CNL_DEF( D_REC_CB_CHROMA ),
    _CNL_DEF( D_REC_CB_LUMA_LF ),
    _CNL_DEF( D_REC_CB_CHROMA_LF ),
    _CNL_DEF( D_REC_CB_LUMA_SAO ),
    _CNL_DEF( D_REC_CB_CHROMA_SAO ),
    _CNL_DEF( D_REC_CB_LUMA_ALF ),
    _CNL_DEF( D_REC_CB_CHROMA_ALF ),
    _CNL_DEF( D_ME ),
    _CNL_DEF( D_CABAC ),
    _CNL_DEF( D_SYNTAX ),
    _CNL_DEF( D_SYNTAX_RESI ),
    _CNL_DEF( D_BEST_MODE ),
    _CNL_DEF( D_MODE_COST ),
    _CNL_DEF( D_QP_PRED ),
    _CNL_DEF( D_DQP ),
    _CNL_DEF( D_QP ),
    _CNL_DEF( D_QP_PER_CTU ),
    _CNL_DEF( D_MISC ),
    _CNL_DEF( D_TU_ABS_SUM ),
    _CNL_DEF( D_EST_FRAC_BITS ),
    _CNL_DEF( D_INTRA_COST ),
    _CNL_DEF( D_PRED ),
    _CNL_DEF( D_RESIDUALS ),
    _CNL_DEF( D_TCOEFF ),
    _CNL_DEF( D_RDOQ ),
    _CNL_DEF( D_RDOQ_MORE ),
    _CNL_DEF( D_RDOQ_COST ),
    _CNL_DEF( D_TMP ),
    _CNL_DEF( D_MOT_FIELD ),
    _CNL_DEF( D_MOT_COMP ),
    _CNL_DEF( D_ALF ),
    _CNL_DEF( D_CRC )
  };
  dtrace_channels_t channels( next_channels, &next_channels[sizeof( next_channels ) / sizeof( next_channels[0] )] );

  if( !sTracingFile.empty() || !sTracingRule.empty() )
  {
    msg.log( VVENC_VERBOSE, "\nTracing is enabled: %s : %s\n", sTracingFile.c_str(), sTracingRule.c_str() );
  }

  CDTrace *pDtrace = new CDTrace( sTracingFile, sTracingRule, channels );
  if( pDtrace->getLastError() )
  {
   msg.log( VVENC_WARNING, "%s\n", pDtrace->getErrMessage().c_str() );
    //return NULL;
  }

  return pDtrace;
}

#else

#define DTRACE(ctx,channel,...)
#define DTRACE_WITHOUT_COUNT(ctx,channel,...)
#define DTRACE_DECR_COUNTER(ctx,channel)
#define DTRACE_UPDATE(ctx,s)
#define DTRACE_COND(cond,level,...)
#define DTRACE_REPEAT(ctx,channel,times,...)
#define DTRACE_SET(_dst,_src)  (_dst)=(_src)
#define DTRACE_BLOCK(...)
#define DTRACE_FRAME_BLOCKWISE(...)
#define DTRACE_GET_COUNTER(ctx,channel)

#endif

} // namespace vvenc

//! \}


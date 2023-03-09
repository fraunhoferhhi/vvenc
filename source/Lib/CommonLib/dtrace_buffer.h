/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     dtrace_buffer.h
 *  \brief    Easy to use dtrace calls concerning buffers
 */

#pragma once

#include "dtrace.h"
#include "dtrace_next.h"
#include "CommonDef.h"
#include "Unit.h"
#include "UnitTools.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

inline unsigned calcCheckSum( const int iWidth, const int iHeight,  const Pel* p,  const uint32_t stride,  const int bitdepth )
{
  unsigned checksum = 0;
  for( unsigned y = 0; y < iHeight; y++)
  {
    for( unsigned  x = 0; x < iWidth; x++)
    {
      uint8_t xor_mask = (x & 0xff) ^ (y & 0xff) ^ (x >> 8) ^ (y >> 8);
      checksum = (checksum + ((p[y*stride+x] & 0xff) ^ xor_mask)) & 0xffffffff;

      if(bitdepth > 8)
      {
        checksum = (checksum + ((p[y*stride+x]>>8) ^ xor_mask)) & 0xffffffff;
      }
    }
  }
  return checksum;
}

inline unsigned calcCheckSum( const CPelBuf& buf, int bitdepth )
{
  return calcCheckSum( buf.width, buf.height, buf.buf, buf.stride, bitdepth );
}

#if ENABLE_TRACING

//////////////////////////////////////////////////////////////////////////
//
// Specialized helper functions
//
//////////////////////////////////////////////////////////////////////////
template<typename T>
inline void dtraceCoeffBuf( DTRACE_CHANNEL channnel, const AreaBuf<T>& coefBuf, const UnitArea& ua, PredMode predMode, const ComponentID compId, uint32_t zIdx = 0 )
{
  int x0 = ua.blocks[compId].x;
  int y0 = ua.blocks[compId].y;
  const uint32_t    uiStride = coefBuf.stride;
  const T*          piReco   = coefBuf.buf;
  const uint32_t    uiWidth  = ua.blocks[compId].width;
  const uint32_t    uiHeight = ua.blocks[compId].height;
  DTRACE(g_trace_ctx, channnel, "@(%4d,%4d) [%2dx%2d] comp=%d predmode=%d \n", x0, y0, uiWidth, uiHeight, compId, predMode);
  DTRACE_BLOCK(g_trace_ctx, channnel, piReco, uiStride, uiWidth, uiHeight);
}

inline void dtracePelBuf( DTRACE_CHANNEL channnel, const CPelBuf& pelBuf, const UnitArea& ua, PredMode predMode, const ComponentID compId )
{
  int x0 = ua.block(compId).x;
  int y0 = ua.block(compId).y;
  const uint32_t    uiStride     = pelBuf.stride;
  const Pel*    piReco       = pelBuf.buf;
  const uint32_t    uiWidth      = ua.block(compId).width;
  const uint32_t    uiHeight     = ua.block(compId).height;
  DTRACE      ( g_trace_ctx, channnel,   "@(%4d,%4d) [%2dx%2d] comp=%d predmode=%d \n", x0, y0, uiWidth, uiHeight, compId, predMode );
  DTRACE_BLOCK( g_trace_ctx, channnel,   piReco, uiStride, uiWidth, uiHeight );
}

inline void dtraceBlockRec( const CPelUnitBuf& pelUnitBuf, const UnitArea& ua, PredMode predMode, uint32_t zIdx = 0 )
{
  if( ua.blocks[COMP_Y].valid() )
  {
    const int     x0           = ua.lumaPos().x;
    const int     y0           = ua.lumaPos().y;
    const uint32_t    uiStride     = pelUnitBuf.Y().stride;
    const Pel*    piReco       = pelUnitBuf.Y().buf;
    const uint32_t    uiWidth      = ua.lumaSize().width;
    const uint32_t    uiHeight     = ua.lumaSize().height;
    DTRACE      ( g_trace_ctx, D_REC_CB_LUMA,   "%d, x=%d, y=%d, size=%dx%d, predmode=%d \n", zIdx, x0, y0, uiWidth, uiHeight, predMode );
    DTRACE_BLOCK( g_trace_ctx, D_REC_CB_LUMA,   piReco, uiStride, uiWidth, uiHeight );
  }
  if( ua.blocks.size() > 2 && ua.blocks[COMP_Cb].valid() )
  {
    const int     x0           = ua.blocks[1].x;
    const int     y0           = ua.blocks[1].y;
    const uint32_t    uiWidth      = ua.blocks[1].width;
    const uint32_t    uiHeight     = ua.blocks[1].height;
    const uint32_t    uiCStride    = pelUnitBuf.Cb().stride;
    const Pel*    piRecoU      = pelUnitBuf.Cb().buf;
    const Pel*    piRecoV      = pelUnitBuf.Cr().buf;
    DTRACE      ( g_trace_ctx, D_REC_CB_CHROMA, "%d, x=%d, y=%d, size=%dx%d, predmode=%d \n", zIdx, x0, y0, uiWidth, uiHeight, predMode );
    DTRACE_BLOCK( g_trace_ctx, D_REC_CB_CHROMA, piRecoU, uiCStride, uiWidth, uiHeight );
    DTRACE_BLOCK( g_trace_ctx, D_REC_CB_CHROMA, piRecoV, uiCStride, uiWidth, uiHeight );
  }
}

inline void dtraceUnitComp( DTRACE_CHANNEL channel, CPelUnitBuf& pelUnitBuf, const UnitArea& ua, ComponentID compId, PredMode predMode, uint32_t zIdx = 0 )
{
  if( !g_trace_ctx ) return;
  if( pelUnitBuf.chromaFormat == CHROMA_400 && compId != COMP_Y )  return;
  const Pel* piReco   = pelUnitBuf.bufs[compId].buf;
  uint32_t       uiStride = pelUnitBuf.bufs[compId].stride;
  uint32_t       uiWidth  = ua.blocks[compId].width;
  uint32_t       uiHeight = ua.blocks[compId].height;
  int x0              = ua.lumaPos().x;
  int y0              = ua.lumaPos().y;

  DTRACE      ( g_trace_ctx, channel, "%s: %d, x=%d, y=%d, size=%dx%d, predmode=%d \n", g_trace_ctx->getChannelName(channel), zIdx, x0, y0, uiWidth, uiHeight, predMode );
  DTRACE_BLOCK( g_trace_ctx, channel, piReco, uiStride, uiWidth, uiHeight );
}

inline void dtraceMotField( CDTrace *trace_ctx, const CodingUnit& cu )
{
  DTRACE( trace_ctx, D_MOT_FIELD, "CU %d,%d @ %d,%d\n", cu.lwidth(), cu.lheight(), cu.lx(), cu.ly() );
  const CMotionBuf mb = cu.getMotionBuf();
  for( uint32_t listIdx = 0; listIdx < 2; listIdx++ )
  {
    RefPicList eListIdx = RefPicList( listIdx );
    for( int y = 0, i = 0; y < cu.lheight(); y += 4 )
    {
      for( int x = 0; x < cu.lwidth(); x += 4, i++ )
      {
        const MotionInfo &mi = mb.at( x >> 2, y >> 2 );
        DTRACE( trace_ctx, D_MOT_FIELD, "%d,%d:%d  ", mi.mv[eListIdx].hor, mi.mv[eListIdx].ver, mi.refIdx[eListIdx] );
      }
      DTRACE( trace_ctx, D_MOT_FIELD, "\n" );
    }
    DTRACE( trace_ctx, D_MOT_FIELD, "\n" );
  }
}

inline void dtraceCRC( CDTrace *trace_ctx, DTRACE_CHANNEL channel, const CodingStructure& cs, const CPelUnitBuf& pelUnitBuf, const Area* parea = NULL )
{
  const Area& area = parea ? *parea : cs.area.Y();
  if( cs.pcv->chrFormat == VVENC_CHROMA_400)
  {
    DTRACE( trace_ctx, channel, " CRC: %6lld %3d @(%4d,%4d) [%2dx%2d] ,Checksum(%x)\n",
        DTRACE_GET_COUNTER( g_trace_ctx, channel ),
        cs.slice->poc,
        area.x, area.y, area.width, area.height,
        calcCheckSum( pelUnitBuf.bufs[COMP_Y],  cs.sps->bitDepths[CH_L]));
  }
  else
  {
    DTRACE( trace_ctx, channel, " CRC: %6lld %3d @(%4d,%4d) [%2dx%2d] ,Checksum(%x %x %x)\n",
        DTRACE_GET_COUNTER( g_trace_ctx, channel ),
        cs.slice->poc,
        area.x, area.y, area.width, area.height,
        calcCheckSum( pelUnitBuf.bufs[COMP_Y],  cs.sps->bitDepths[CH_L]),
        calcCheckSum( pelUnitBuf.bufs[COMP_Cb], cs.sps->bitDepths[CH_C]),
        calcCheckSum( pelUnitBuf.bufs[COMP_Cr], cs.sps->bitDepths[CH_C]));
  }
}

inline void dtraceCCRC( CDTrace *trace_ctx, DTRACE_CHANNEL channel, const CodingStructure& cs, const CPelBuf& pelBuf, ComponentID compId, const Area* parea = NULL )
{
  const Area& area = parea ? *parea : cs.area.Y();
  DTRACE( trace_ctx, channel, "CCRC: %6lld %3d @(%4d,%4d) [%2dx%2d] ,comp %d Checksum(%x)\n",
      DTRACE_GET_COUNTER( g_trace_ctx, channel ),
      cs.slice->poc,
      area.x, area.y, area.width, area.height, compId,
      calcCheckSum( pelBuf, cs.sps->bitDepths[ toChannelType(compId) ]));
}


inline void dtraceAreaCRC( CDTrace *trace_ctx, DTRACE_CHANNEL channel, const CodingStructure& cs, const UnitArea& ctuArea )
{
  dtraceCRC( g_trace_ctx, channel, cs, cs.picture->getRecoBuf( clipArea( ctuArea, *cs.picture ) ), &ctuArea.Y() );

  for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_L, TREE_D ), CH_L ) )
  {
    if( currCU.Y().valid() )
    {
      dtraceCCRC(g_trace_ctx, channel, *currCU.cs, currCU.cs->picture->getRecoBuf(currCU.Y()), COMP_Y, &currCU.Y());
    }
  }

  if( cs.pcv->chrFormat != VVENC_CHROMA_400 )
  {
    for( auto &currCU : cs.traverseCUs( CS::getArea( cs, ctuArea, CH_C, TREE_D ), CH_C ) )
    {
      if( currCU.Cb().valid() )
      {
        dtraceCCRC(g_trace_ctx, channel, *currCU.cs, currCU.cs->picture->getRecoBuf(currCU.Cb()), COMP_Cb, &currCU.Cb());
        dtraceCCRC(g_trace_ctx, channel, *currCU.cs, currCU.cs->picture->getRecoBuf(currCU.Cr()), COMP_Cr, &currCU.Cb());
      }
    }
  }
}


#define DTRACE_PEL_BUF(...)              dtracePelBuf( __VA_ARGS__ )
#define DTRACE_COEFF_BUF(...)            dtraceCoeffBuf( __VA_ARGS__ )
#define DTRACE_BLOCK_REC(...)            dtraceBlockRec( __VA_ARGS__ )
#define DTRACE_PEL_BUF_COND(_cond,...)   { if((_cond)) dtracePelBuf( __VA_ARGS__ ); }
#define DTRACE_COEFF_BUF_COND(_cond,...) { if((_cond)) dtraceCoeffBuf( __VA_ARGS__ ); }
#define DTRACE_BLOCK_REC_COND(_cond,...) { if((_cond)) dtraceBlockRec( __VA_ARGS__ ); }
#define DTRACE_UNIT_COMP(...)            dtraceUnitComp( __VA_ARGS__ )
#define DTRACE_MOT_FIELD(...)            dtraceMotField( __VA_ARGS__ )
#define DTRACE_CRC(...)                  dtraceCRC( __VA_ARGS__ )
#define DTRACE_CCRC(...)                 dtraceCCRC( __VA_ARGS__ )
#define DTRACE_AREA_CRC(...)             dtraceAreaCRC( __VA_ARGS__ )

#else

#define DTRACE_PEL_BUF(...)
#define DTRACE_COEFF_BUF(...)
#define DTRACE_BLOCK_REC(...)
#define DTRACE_PEL_BUF_COND(...)
#define DTRACE_COEFF_BUF_COND(...)
#define DTRACE_BLOCK_REC_COND(...)
#define DTRACE_UNIT_COMP(...)
#define DTRACE_MOT_FIELD(...)
#define DTRACE_CRC(...)
#define DTRACE_CCRC(...)
#define DTRACE_AREA_CRC(...)
#endif

} // namespace vvenc

//! \}


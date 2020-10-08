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
/** \file     dtrace_buffer.h
 *  \brief    Easy to use dtrace calls concerning buffers
 */

#pragma once

#include "dtrace.h"
#include "dtrace_next.h"
#include "CommonDef.h"
#include "Unit.h"

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
inline void dtraceCoeffBuf( DTRACE_CHANNEL channnel, const CCoeffBuf& coefBuf, const UnitArea& ua, PredMode predMode, const ComponentID compId, uint32_t zIdx = 0 )
{
  int x0 = ua.blocks[compId].x;
  int y0 = ua.blocks[compId].y;
  const uint32_t    uiStride = coefBuf.stride;
  const TCoeff* piReco   = coefBuf.buf;
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
  if( ua.blocks[COMP_Cb].valid() )
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

inline void dtraceCRC( CDTrace *trace_ctx, DTRACE_CHANNEL channel, const CodingStructure& cs, const CPelUnitBuf& pelUnitBuf, const Area* parea = NULL )
{
  const Area& area = parea ? *parea : cs.area.Y();
  DTRACE( trace_ctx, channel, " CRC: %6lld %3d @(%4d,%4d) [%2dx%2d] ,Checksum(%x %x %x)\n",
      DTRACE_GET_COUNTER( g_trace_ctx, channel ),
      cs.slice->poc,
      area.x, area.y, area.width, area.height,
      calcCheckSum( pelUnitBuf.bufs[COMP_Y],  cs.sps->bitDepths[CH_L]),
      calcCheckSum( pelUnitBuf.bufs[COMP_Cb], cs.sps->bitDepths[CH_C]),
      calcCheckSum( pelUnitBuf.bufs[COMP_Cr], cs.sps->bitDepths[CH_C]));
}

inline void dtraceCCRC( CDTrace *trace_ctx, DTRACE_CHANNEL channel, const CodingStructure& cs, const CPelBuf& pelBuf, ComponentID compId, const Area* parea = NULL )
{
  const Area& area = parea ? *parea : cs.area.Y();
  DTRACE( trace_ctx, channel, "CRC: %6lld %3d @(%4d,%4d) [%2dx%2d] ,comp %d Checksum(%x)\n",
      DTRACE_GET_COUNTER( g_trace_ctx, channel ),
      cs.slice->poc,
      area.x, area.y, area.width, area.height, compId,
      calcCheckSum( pelBuf, cs.sps->bitDepths[ toChannelType(compId) ]));
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

#define DTRACE_PEL_BUF(...)              dtracePelBuf( __VA_ARGS__ )
#define DTRACE_COEFF_BUF(...)            dtraceCoeffBuf( __VA_ARGS__ )
#define DTRACE_BLOCK_REC(...)            dtraceBlockRec( __VA_ARGS__ )
#define DTRACE_PEL_BUF_COND(_cond,...)   { if((_cond)) dtracePelBuf( __VA_ARGS__ ); }
#define DTRACE_COEFF_BUF_COND(_cond,...) { if((_cond)) dtraceCoeffBuf( __VA_ARGS__ ); }
#define DTRACE_BLOCK_REC_COND(_cond,...) { if((_cond)) dtraceBlockRec( __VA_ARGS__ ); }
#define DTRACE_UNIT_COMP(...)            dtraceUnitComp( __VA_ARGS__ )
#define DTRACE_CRC(...)                  dtraceCRC( __VA_ARGS__ )
#define DTRACE_CCRC(...)                 dtraceCCRC( __VA_ARGS__ )
#define DTRACE_MOT_FIELD(...)            dtraceMotField( __VA_ARGS__ )

#else

#define DTRACE_PEL_BUF(...)
#define DTRACE_COEFF_BUF(...)
#define DTRACE_BLOCK_REC(...)
#define DTRACE_PEL_BUF_COND(...)
#define DTRACE_COEFF_BUF_COND(...)
#define DTRACE_BLOCK_REC_COND(...)
#define DTRACE_UNIT_COMP(...)
#define DTRACE_CRC(...)
#define DTRACE_CCRC(...)
#define DTRACE_MOT_FIELD(...)

#endif

} // namespace vvenc

//! \}


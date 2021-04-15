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
/** \file     dtrace_codingstruct.h
 *  \brief    Easy to use dtrace calls concerning coding structures
 */

#pragma once

#include "dtrace.h"
#include "dtrace_next.h"
#include "CommonDef.h"
#include "CodingStructure.h"
#include "Slice.h"
#include "Mv.h"
#include "Unit.h"
#include "UnitTools.h"

#include <cmath>

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if ENABLE_TRACING

inline void dtracePicComp( DTRACE_CHANNEL channel, CodingStructure& cs, const CPelUnitBuf& pelUnitBuf, ComponentID compId )
{
  if( !g_trace_ctx ) return;
  if( pelUnitBuf.chromaFormat == CHROMA_400 && compId != COMP_Y )  return;
  const Pel* piSrc    = pelUnitBuf.bufs[compId].buf;
  uint32_t       uiStride = pelUnitBuf.bufs[compId].stride;
  uint32_t       uiWidth  = pelUnitBuf.bufs[compId].width;
  uint32_t       uiHeight = pelUnitBuf.bufs[compId].height;
  uint32_t       uiChromaScaleX = getComponentScaleX( compId, pelUnitBuf.chromaFormat );
  uint32_t       uiChromaScaleY = getComponentScaleY( compId, pelUnitBuf.chromaFormat );

  DTRACE                ( g_trace_ctx, channel, "\n%s: poc = %d, size=%dx%d\n\n", g_trace_ctx->getChannelName(channel), cs.slice->poc, uiWidth, uiHeight );
  DTRACE_FRAME_BLOCKWISE( g_trace_ctx, channel, piSrc, uiStride, uiWidth, uiHeight, cs.sps->CTUSize >> uiChromaScaleX, cs.sps->CTUSize >> uiChromaScaleY);
}

#define OLD_RDCOST 1

inline void dtraceModeCost(CodingStructure &cs, double lambda)
{
  CHECK( cs.cus.size() != 1, "Only the cost for a single CU can be show with dtraceModeCost!" );

  Distortion tempDist = cs.dist;

#if OLD_RDCOST
  uint64_t tempBits = cs.fracBits >> SCALE_BITS;
  uint64_t tempCost = (uint64_t)(cs.dist + (double)tempBits * lambda);
#else
  uint64_t tempBits = cs.fracBits;
  uint64_t tempCost = (uint64_t)cs.cost;
#endif

  if( cs.cost == MAX_DOUBLE )
  {
    tempCost = 0;
    tempBits = 0;
    tempDist = 0;
  }

  bool isIntra = CU::isIntra( *cs.cus.front() );
  int intraModeL = isIntra ? cs.cus.front()->intraDir[0] : 0;
  int intraModeC = isIntra ? cs.cus.front()->intraDir[1] : 0;
  if (isIntra && intraModeC == DM_CHROMA_IDX)
    intraModeC = 68;
  int imvVal = 0;
  imvVal = cs.cus[0]->imv;
  DTRACE( g_trace_ctx, D_MODE_COST, "ModeCost: %6lld %3d @(%4d,%4d) [%2dx%2d] %d (qp%d,pm%d,skip%d,mrg%d,fruc%d,obmc%d,ic%d,imv%d,affn%d,%d,%d) tempCS = %lld (%d,%d)\n",
    DTRACE_GET_COUNTER( g_trace_ctx, D_MODE_COST ),
    cs.slice->poc,
    cs.area.Y().x, cs.area.Y().y,
    cs.area.Y().width, cs.area.Y().height,
    cs.cus[0]->qtDepth,
    cs.cus[0]->qp,
    cs.cus[0]->predMode,
    cs.cus[0]->skip,
    cs.cus[0]->mergeFlag,
    0, 0,
    imvVal,
    0, 0,
          intraModeL, intraModeC,
          tempCost, tempBits, tempDist );
}

inline void dtraceBestMode(CodingStructure *&tempCS, CodingStructure *&bestCS, double lambda, bool useEDO)
{
  bool bSplitCS = tempCS->cus.size() > 1 || bestCS->cus.size() > 1;
  ChannelType chType = tempCS->cus.back()->chType;

  // if the last CU does not align with the CS, we probably are at the edge
  bSplitCS |= tempCS->cus.back()->blocks[chType].bottomRight() != tempCS->area.blocks[chType].bottomRight();

  Distortion tempDist = tempCS->dist;

#if OLD_RDCOST
  uint64_t tempBits = tempCS->fracBits >> SCALE_BITS;
  uint64_t bestBits = bestCS->fracBits >> SCALE_BITS;
//  uint64_t tempCost = (uint64_t)(tempCS->dist + (double)tempBits * lambda + ( useEDO ? tempCS->costDbOffset : 0 ));
//  uint64_t bestCost = (uint64_t)(bestCS->dist + (double)bestBits * lambda + ( useEDO ? bestCS->costDbOffset : 0 ));
  uint64_t tempCost = (uint64_t)(tempCS->dist + (double)tempBits * lambda);
  uint64_t bestCost = (uint64_t)(bestCS->dist + (double)bestBits * lambda);
#else
  uint64_t tempBits = tempCS->fracBits;
  uint64_t bestBits = bestCS->fracBits;
  uint64_t tempCost = (uint64_t)tempCS->cost;
  uint64_t bestCost = (uint64_t)bestCS->cost;
#endif

  if( tempCS->cost == MAX_DOUBLE )
  {
    tempCost = 0;
    tempBits = 0;
    tempDist = 0;
  }

  bool isIntra = CU::isIntra( *tempCS->cus[0] );
  int intraModeL = isIntra ? tempCS->cus[0]->intraDir[0] : 0;
  int intraModeC = isIntra ? tempCS->cus[0]->intraDir[1] : 0;

  if(!bSplitCS)
  {
    DTRACE( g_trace_ctx, D_BEST_MODE, "CheckModeCost: %6lld %3d @(%4d,%4d) [%2dx%2d] %d (%d,%d,%2d,%d,%d) tempCS = %lld (%d,%d), bestCS = %lld (%d,%d): --> choose %s\n",
            DTRACE_GET_COUNTER( g_trace_ctx, D_BEST_MODE ),
            tempCS->slice->poc,
            tempCS->area.Y().x, tempCS->area.Y().y,
            tempCS->area.Y().width, tempCS->area.Y().height,
            tempCS->cus[0]->qtDepth,
            tempCS->cus[0]->qp,
            tempCS->cus[0]->predMode,
            tempCS->cus[0]->mergeFlag,
            intraModeL, intraModeC,
            tempCost, tempBits, tempDist,
            bestCost, bestBits, bestCS->dist,
            tempCS->cost < bestCS->cost ? "TEMP" : "BEST" );
  }
  else
  {
    DTRACE( g_trace_ctx, D_BEST_MODE, "CheckModeSplitCost: %6lld %3d @(%4d,%4d) [%2dx%2d] -------------------------- tempCS = %lld (%d,%d), bestCS = %lld (%d,%d): --> choose %s\n",
            DTRACE_GET_COUNTER( g_trace_ctx, D_BEST_MODE ),
            tempCS->slice->poc,
            tempCS->area.Y().x, tempCS->area.Y().y,
            tempCS->area.Y().width, tempCS->area.Y().height,
            tempCost, tempBits, tempDist,
            bestCost, bestBits, bestCS->dist,
            tempCS->cost < bestCS->cost ? "TEMP STRUCTURE" : "BEST STRUCTURE");
  }
}


#define DTRACE_PIC_COMP(...)             dtracePicComp( __VA_ARGS__ )
#define DTRACE_PIC_COMP_COND(_cond,...)  { if((_cond)) dtracePicComp( __VA_ARGS__ ); }
#define DTRACE_BEST_MODE(...)            dtraceBestMode(__VA_ARGS__)
#define DTRACE_MODE_COST(...)            dtraceModeCost(__VA_ARGS__)
#define DTRACE_STAT(...)                 dtraceComprPicStat(__VA_ARGS__)

#else

#define DTRACE_PIC_COMP(...)
#define DTRACE_PIC_COMP_COND(_cond,...)
#define DTRACE_BEST_MODE(...)
#define DTRACE_MODE_COST(...)
#define DTRACE_STAT(...)

#endif

} // namespace vvenc

//! \}


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


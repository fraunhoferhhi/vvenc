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

Copyright (c) 2019-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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


/** \file     DecSlice.cpp
    \brief    slice decoder class
*/

#include "DecSlice.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_next.h"

#include <vector>

//! \ingroup DecoderLib
//! \{

namespace vvenc {

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

DecSlice::DecSlice()
{
}

DecSlice::~DecSlice()
{
}

void DecSlice::create()
{
}

void DecSlice::destroy()
{
}

void DecSlice::init( CABACDecoder* cabacDecoder, DecCu* pcCuDecoder )
{
  m_CABACDecoder    = cabacDecoder;
  m_pcCuDecoder     = pcCuDecoder;
}

void DecSlice::decompressSlice( Slice* slice, InputBitstream* bitstream )
{
  const SPS*     sps          = slice->sps;
  Picture*       pic          = slice->pic;
  CABACReader&   cabacReader  = *m_CABACDecoder->getCABACReader();

  // setup coding structure
  CodingStructure& cs = *pic->cs;
  cs.slice            = slice;
  cs.sps              = sps;
  cs.pps              = slice->pps;
  memcpy(cs.alfAps, slice->alfAps, sizeof(cs.alfAps));

  cs.lmcsAps = slice->picHeader->lmcsAps;

  cs.pcv              = slice->pps->pcv;
  cs.chromaQpAdj      = 0;

  cs.picture->resizeSAO(cs.pcv->sizeInCtus, 0);

  if (slice->sliceMap.ctuAddrInSlice[0] == 0)
  {
    cs.picture->resizeAlfCtuEnabled( cs.pcv->sizeInCtus );
    cs.picture->resizeAlfCtbFilterIndex(cs.pcv->sizeInCtus);
    cs.picture->resizeAlfCtuAlternative( cs.pcv->sizeInCtus );
  }

  const unsigned numSubstreams = slice->getNumberOfSubstreamSizes() + 1;

  // init each couple {EntropyDecoder, Substream}
  // Table of extracted substreams.
  std::vector<InputBitstream*> ppcSubstreams( numSubstreams );
  for( unsigned idx = 0; idx < numSubstreams; idx++ )
  {
    ppcSubstreams[idx] = bitstream->extractSubstream( idx+1 < numSubstreams ? ( slice->getSubstreamSize(idx) << 3 ) : bitstream->getNumBitsLeft() );
  }

  const unsigned  widthInCtus             = cs.pcv->widthInCtus;
  const bool      wavefrontsEnabled       = cs.sps->entropyCodingSyncEnabled;
  const bool     entryPointPresent        = cs.sps->entryPointsPresent;

  cabacReader.initBitstream( ppcSubstreams[0] );
  cabacReader.initCtxModels( *slice );

  DTRACE( g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", slice->poc );

  // for every CTU in the slice segment...
  unsigned subStrmId = 0;
  for( unsigned ctuIdx = 0; ctuIdx < slice->sliceMap.numCtuInSlice; ctuIdx++ )
  {
    const unsigned  ctuRsAddr       = slice->sliceMap.ctuAddrInSlice[ctuIdx];
    const unsigned  ctuXPosInCtus   = ctuRsAddr % widthInCtus;
    const unsigned  ctuYPosInCtus   = ctuRsAddr / widthInCtus;    
    const unsigned  tileXPosInCtus  = 0;
    const unsigned  tileYPosInCtus  = 0;
    const unsigned  tileColWidth    = widthInCtus;
    const unsigned  tileRowHeight   = cs.pcv->heightInCtus;
    const unsigned  tileIdx         = slice->pps->getTileIdx( Position(ctuXPosInCtus, ctuYPosInCtus));
    const unsigned  maxCUSize       = sps->CTUSize;
    Position pos( ctuXPosInCtus*maxCUSize, ctuYPosInCtus*maxCUSize) ;
    UnitArea ctuArea(cs.area.chromaFormat, Area( pos.x, pos.y, maxCUSize, maxCUSize ) );
    const SubPic& curSubPic = slice->pps->getSubPicFromPos(pos);
    // padding/restore at slice level
    if (slice->pps->numSubPics>=2 && curSubPic.treatedAsPic && ctuIdx==0)
    {
      for (int rlist = REF_PIC_LIST_0; rlist < NUM_REF_PIC_LIST_01; rlist++) 
      {
        int n = slice->numRefIdx[((RefPicList)rlist)];
        for (int idx = 0; idx < n; idx++) 
        {
          Picture *refPic = slice->refPicList[rlist][idx];
          if (!refPic->isSubPicBorderSaved) 
          {
            THROW("no support");
            refPic->isSubPicBorderSaved = (true);
          }
        }
      }
    }

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    cabacReader.initBitstream( ppcSubstreams[subStrmId] );

    // set up CABAC contexts' state for this CTU
    if( ctuXPosInCtus == tileXPosInCtus && ctuYPosInCtus == tileYPosInCtus )
    {
      if( ctuIdx != 0 ) // if it is the first CTU, then the entropy coder has already been reset
      {
        cabacReader.initCtxModels( *slice );
      }
      pic->cs->prevQP[0] = pic->cs->prevQP[1] = slice->sliceQp;
    }
    else if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      // Synchronize cabac probabilities with top CTU if it's available and at the start of a line.
      if( ctuIdx != 0 ) // if it is the first CTU, then the entropy coder has already been reset
      {
        cabacReader.initCtxModels( *slice );
      }
      if( cs.getCURestricted( pos.offset(0, -1), pos, slice->independentSliceIdx, tileIdx, CH_L, TREE_D ) )
      {
        // Top is available, so use it.
        cabacReader.getCtx() = m_entropyCodingSyncContextState;
      }
      pic->cs->prevQP[0] = pic->cs->prevQP[1] = slice->sliceQp;
    }

    if ((cs.slice->sliceType != I_SLICE || cs.sps->IBC) && ctuXPosInCtus == tileXPosInCtus)
    {
      cs.motionLut.lut.resize(0);
    }

    cabacReader.coding_tree_unit( cs, ctuArea, pic->cs->prevQP, ctuRsAddr );

    m_pcCuDecoder->decompressCtu( cs, ctuArea );

    if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState = cabacReader.getCtx();
    }


    if( ctuIdx == slice->sliceMap.numCtuInSlice-1 )
    {
      unsigned binVal = cabacReader.terminating_bit();
      CHECK( !binVal, "Expecting a terminating bit" );
      cabacReader.remaining_bytes( false );
    }
    else if( ( ctuXPosInCtus + 1 == tileXPosInCtus + tileColWidth ) &&
             ( ctuYPosInCtus + 1 == tileYPosInCtus + tileRowHeight || wavefrontsEnabled ) )
    {
      // The sub-stream/stream should be terminated after this CTU.
      // (end of slice-segment, end of tile, end of wavefront-CTU-row)
      unsigned binVal = cabacReader.terminating_bit();
      CHECK( !binVal, "Expecting a terminating bit" );
      if( entryPointPresent )
      {
        cabacReader.remaining_bytes( true );
        subStrmId++;
      }
    }
    if (slice->pps->numSubPics >= 2 && curSubPic.treatedAsPic && ctuIdx == (slice->sliceMap.numCtuInSlice - 1))
    // for last Ctu in the slice
    {
      for (int rlist = REF_PIC_LIST_0; rlist < NUM_REF_PIC_LIST_01; rlist++) 
      {
        int n = slice->numRefIdx[((RefPicList)rlist)];
        for (int idx = 0; idx < n; idx++) 
        {
          Picture *refPic = slice->refPicList[rlist][idx];
          if (refPic->isSubPicBorderSaved) 
          {
            THROW("no support");
            refPic->isSubPicBorderSaved = (false);
          }
        }
      }
    }
  }

  // deallocate all created substreams, including internal buffers.
  for( auto substr: ppcSubstreams )
  {
    delete substr;
  }
}

} // namespace vvenc

//! \}


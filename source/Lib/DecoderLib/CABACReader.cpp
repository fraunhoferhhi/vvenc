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


/** \file     CABACReader.cpp
 *  \brief    Reader for low level syntax
 */

#include "CABACReader.h"

#include "CommonLib/CodingStructure.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/Picture.h"

//! \ingroup DecoderLib
//! \{

namespace vvenc {


void CABACReader::initCtxModels( Slice& slice )
{
  SliceType sliceType  = slice.sliceType;
  int       qp         = slice.sliceQp;
  if( slice.pps->cabacInitPresent && slice.cabacInitFlag )
  {
    switch( sliceType )
    {
    case VVENC_P_SLICE:           // change initialization table to B_SLICE initialization
      sliceType = VVENC_B_SLICE;
      break;
    case VVENC_B_SLICE:           // change initialization table to P_SLICE initialization
      sliceType = VVENC_P_SLICE;
      break;
    default     :           // should not occur
      THROW( "Invalid slice type" );
      break;
    }
  }
  m_BinDecoder.reset( qp, (int)sliceType );
}


//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    bool  terminating_bit()
//    void  remaining_bytes( noTrailingBytesExpected )
//================================================================================

bool CABACReader::terminating_bit()
{
  if( m_BinDecoder.decodeBinTrm() )
  {
    m_BinDecoder.finish();
    m_Bitstream->readOutTrailingBits();
    return true;
  }
  return false;
}

void CABACReader::remaining_bytes( bool noTrailingBytesExpected )
{
  if( noTrailingBytesExpected )
  {
    CHECK( 0 != m_Bitstream->getNumBitsLeft(), "Bits left when not supposed" );
  }
  else
  {
    while( m_Bitstream->getNumBitsLeft() )
    {
      unsigned trailingNullByte = m_Bitstream->readByte();
      if( trailingNullByte != 0 )
      {
        THROW( "Trailing byte should be '0', but has a value of " << std::hex << trailingNullByte << std::dec << "\n" );
      }
    }
  }
}

//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit( cs, area, qpL, qpC, ctuRsAddr )
//================================================================================

void CABACReader::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr )
{
  CUCtx cuCtx( qps[CH_L] );
  Partitioner* partitioner = &m_partitioner[0];

  partitioner->initCtu( area, CH_L, *cs.slice );
  partitioner->treeType = TREE_D;
  partitioner->modeType = MODE_TYPE_ALL;


  sao( cs, ctuRsAddr );
  if (cs.sps->alfEnabled && (cs.slice->tileGroupAlfEnabled[COMP_Y]))
  {
    const PreCalcValues& pcv = *cs.pcv;
    int                 frame_width_in_ctus = pcv.widthInCtus;
    int                 ry = ctuRsAddr / frame_width_in_ctus;
    int                 rx = ctuRsAddr - ry * frame_width_in_ctus;
    const Position      pos( rx * cs.pcv->maxCUSize, ry * cs.pcv->maxCUSize );
    const uint32_t      curSliceIdx = cs.slice->independentSliceIdx;
    const uint32_t      curTileIdx = cs.pps->getTileIdx( pos );
    bool                leftAvail = cs.getCURestricted( pos.offset( -(int)pcv.maxCUSize, 0 ), pos, curSliceIdx, curTileIdx, CH_L, TREE_D ) ? true : false;
    bool                aboveAvail = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUSize ), pos, curSliceIdx, curTileIdx, CH_L, TREE_D ) ? true : false;

    int leftCTUAddr = leftAvail ? ctuRsAddr - 1 : -1;
    int aboveCTUAddr = aboveAvail ? ctuRsAddr - frame_width_in_ctus : -1;

    for( int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++ )
    {
      if (cs.slice->tileGroupAlfEnabled[compIdx])
      {
        uint8_t* ctbAlfFlag = cs.slice->pic->m_alfCtuEnabled[ compIdx ].data();
        int ctx = 0;
        ctx += leftCTUAddr > -1 ? ( ctbAlfFlag[leftCTUAddr] ? 1 : 0 ) : 0;
        ctx += aboveCTUAddr > -1 ? ( ctbAlfFlag[aboveCTUAddr] ? 1 : 0 ) : 0;

        ctbAlfFlag[ctuRsAddr] = m_BinDecoder.decodeBin( Ctx::ctbAlfFlag( compIdx * 3 + ctx ) );

        if (isLuma((ComponentID)compIdx) && ctbAlfFlag[ctuRsAddr])
        {
          readAlfCtuFilterIndex(cs, ctuRsAddr);
        }
        if( isChroma( (ComponentID)compIdx ) )
        {
          int apsIdx = cs.slice->tileGroupChromaApsId;
          CHECK(cs.slice->alfAps[apsIdx] == nullptr, "APS not initialized");
          const AlfParam& alfParam = cs.slice->alfAps[apsIdx]->alfParam;
          const int numAlts = alfParam.numAlternativesChroma;
          uint8_t* ctbAlfAlternative = cs.slice->pic->m_alfCtuAlternative[compIdx].data();
          ctbAlfAlternative[ctuRsAddr] = 0;
          if( ctbAlfFlag[ctuRsAddr] )
          {
            uint8_t decoded = 0;
            while( decoded < numAlts-1 && m_BinDecoder.decodeBin( Ctx::ctbAlfAlternative( compIdx-1 ) ) )
              ++ decoded;
            ctbAlfAlternative[ctuRsAddr] = decoded;
          }
        }
      }
    }
  }

  if (cs.sps->ccalfEnabled)
  {
    for ( int compIdx = 1; compIdx < getNumberValidComponents( cs.pcv->chrFormat ); compIdx++ )
    {
      if (cs.slice->ccAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
      {
        const int filterCount   = cs.slice->ccAlfFilterParam.ccAlfFilterCount[compIdx - 1];

        const int      ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int      rx = ctuRsAddr % cs.pcv->widthInCtus;
        const Position lumaPos(rx * cs.pcv->maxCUSize, ry * cs.pcv->maxCUSize);

        ccAlfFilterControlIdc(cs, ComponentID(compIdx), ctuRsAddr, cs.slice->ccAlfFilterControl[compIdx - 1], lumaPos,
                              filterCount);
      }
    }
  }

  if ( CS::isDualITree(cs) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUSize > 64 )
  {
    Partitioner* chromaPartitioner = &m_partitioner[1];
    chromaPartitioner->initCtu(area, CH_C, *cs.slice);
    CUCtx cuCtxChroma(qps[CH_C]);
    coding_tree(cs, *partitioner, cuCtx, chromaPartitioner, &cuCtxChroma);
    qps[CH_L] = cuCtx.qp;
    qps[CH_C] = cuCtxChroma.qp;
  }
  else
  {
    coding_tree(cs, *partitioner, cuCtx);
    qps[CH_L] = cuCtx.qp;
    if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
    {
      CUCtx cuCtxChroma( qps[CH_C] );
      partitioner->initCtu( area, CH_C, *cs.slice );
      coding_tree( cs, *partitioner, cuCtxChroma );
      qps[CH_C] = cuCtxChroma.qp;
    }
  }

  DTRACE_COND( ctuRsAddr == 0, g_trace_ctx, D_QP_PER_CTU, "\n%4d %2d", cs.picture->poc, cs.slice->sliceQp );
  DTRACE     (                 g_trace_ctx, D_QP_PER_CTU, " %3d",           qps[CH_L] - cs.slice->sliceQp );
}

void CABACReader::readAlfCtuFilterIndex(CodingStructure& cs, unsigned ctuRsAddr)
{
  short* alfCtbFilterSetIndex = cs.slice->pic->m_alfCtbFilterIndex.data();
  unsigned numAps = cs.slice->tileGroupNumAps;
  unsigned numAvailableFiltSets = numAps + NUM_FIXED_FILTER_SETS;
  uint32_t filtIndex = 0;
  if (numAvailableFiltSets > NUM_FIXED_FILTER_SETS)
  {
    unsigned usePrevFilt = m_BinDecoder.decodeBin(Ctx::AlfUseTemporalFilt());
    if (usePrevFilt)
    {
      if (numAps > 1)
      {
        xReadTruncBinCode(filtIndex, numAvailableFiltSets - NUM_FIXED_FILTER_SETS);
      }
      filtIndex += (unsigned)(NUM_FIXED_FILTER_SETS);
    }
    else
    {
      xReadTruncBinCode(filtIndex, NUM_FIXED_FILTER_SETS);
    }
  }
  else
  {
    xReadTruncBinCode(filtIndex, NUM_FIXED_FILTER_SETS);
  }
  alfCtbFilterSetIndex[ctuRsAddr] = filtIndex;
}

void CABACReader::ccAlfFilterControlIdc(CodingStructure &cs, const ComponentID compID, const int curIdx,
                                        uint8_t *filterControlIdc, Position lumaPos, int filterCount)
{
  Position       leftLumaPos    = lumaPos.offset(-(int)cs.pcv->maxCUSize, 0);
  Position       aboveLumaPos   = lumaPos.offset(0, -(int)cs.pcv->maxCUSize);
  const uint32_t curSliceIdx    = cs.slice->independentSliceIdx;
  const uint32_t curTileIdx     = cs.pps->getTileIdx( lumaPos );
  bool           leftAvail      = cs.getCURestricted( leftLumaPos,  lumaPos, curSliceIdx, curTileIdx, CH_L, TREE_D ) ? true : false;
  bool           aboveAvail     = cs.getCURestricted( aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, CH_L, TREE_D ) ? true : false;
  int            ctxt           = 0;

  if (leftAvail)
  {
    ctxt += ( filterControlIdc[curIdx - 1] ) ? 1 : 0;
  }
  if (aboveAvail)
  {
    ctxt += ( filterControlIdc[curIdx - cs.pcv->widthInCtus] ) ? 1 : 0;
  }
  ctxt += ( compID == COMP_Cr ) ? 3 : 0;

  int idcVal  = m_BinDecoder.decodeBin( Ctx::CcAlfFilterControlFlag( ctxt ) );
  if ( idcVal )
  {
    while ( ( idcVal != filterCount ) && m_BinDecoder.decodeBinEP() )
    {
      idcVal++;
    }
  }
  filterControlIdc[curIdx] = idcVal;

  DTRACE(g_trace_ctx, D_SYNTAX, "ccAlfFilterControlIdc() compID=%d pos=(%d,%d) ctxt=%d, filterCount=%d, idcVal=%d\n",
         compID, lumaPos.x, lumaPos.y, ctxt, filterCount, idcVal);
}

//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao( slice, ctuRsAddr )
//================================================================================

void CABACReader::sao( CodingStructure& cs, unsigned ctuRsAddr )
{
  const SPS&   sps   = *cs.sps;

  if( !sps.saoEnabled )
  {
    return;
  }

  const Slice& slice                        = *cs.slice;
  SAOBlkParam&      sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
  bool              slice_sao_luma_flag     = ( slice.saoEnabled[ CH_L ] );
  bool              slice_sao_chroma_flag   = ( slice.saoEnabled[ CH_C ] && sps.chromaFormatIdc != CHROMA_400 );
  sao_ctu_pars[ COMP_Y  ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMP_Cb ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMP_Cr ].modeIdc      = SAO_MODE_OFF;
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  // merge
  int             frame_width_in_ctus     = cs.pcv->widthInCtus;
  int             ry                      = ctuRsAddr      / frame_width_in_ctus;
  int             rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  int             sao_merge_type          = -1;
  const Position  pos( rx * cs.pcv->maxCUSize, ry * cs.pcv->maxCUSize );
  const unsigned  curSliceIdx = cs.slice->independentSliceIdx;

  const unsigned  curTileIdx  = cs.pps->getTileIdx( pos );
  if( cs.getCURestricted( pos.offset(-(int)cs.pcv->maxCUSize, 0), pos, curSliceIdx, curTileIdx, CH_L, TREE_D ) )
  {
    // sao_merge_left_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) );
  }

  if( sao_merge_type < 0 && cs.getCURestricted( pos.offset(0, -(int)cs.pcv->maxCUSize), pos, curSliceIdx, curTileIdx, CH_L, TREE_D ) )
  {
    // sao_merge_above_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) ) << 1;
  }
  if( sao_merge_type >= 0 )
  {
    if( slice_sao_luma_flag || slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMP_Y  ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMP_Y  ].typeIdc  = sao_merge_type;
    }
    if( slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMP_Cb ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMP_Cr ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMP_Cb ].typeIdc  = sao_merge_type;
      sao_ctu_pars[ COMP_Cr ].typeIdc  = sao_merge_type;
    }
    return;
  }

  // explicit parameters
  ComponentID firstComp = ( slice_sao_luma_flag   ? COMP_Y  : COMP_Cb );
  ComponentID lastComp  = ( slice_sao_chroma_flag ? COMP_Cr : COMP_Y  );
  for( ComponentID compID = firstComp; compID <= lastComp; compID = ComponentID( compID + 1 ) )
  {
    SAOOffset& sao_pars = sao_ctu_pars[ compID ];

    // sao_type_idx_luma / sao_type_idx_chroma
    if( compID != COMP_Cr )
    {
      if( m_BinDecoder.decodeBin( Ctx::SaoTypeIdx() ) )
      {
        if( m_BinDecoder.decodeBinEP( ) )
        {
          // edge offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_EO;
        }
        else
        {
          // band offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_BO;
        }
      }
    }
    else //Cr, follow Cb SAO type
    {
      sao_pars.modeIdc = sao_ctu_pars[ COMP_Cb ].modeIdc;
      sao_pars.typeIdc = sao_ctu_pars[ COMP_Cb ].typeIdc;
    }
    if( sao_pars.modeIdc == SAO_MODE_OFF )
    {
      continue;
    }

    // sao_offset_abs
    int       offset[4];
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( sps.bitDepths[ toChannelType(compID) ] );
    offset    [0]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [1]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [2]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [3]           = (int)unary_max_eqprob( maxOffsetQVal );

    // band offset mode
    if( sao_pars.typeIdc == SAO_TYPE_START_BO )
    {
      // sao_offset_sign
      for( int k = 0; k < 4; k++ )
      {
        if( offset[k] && m_BinDecoder.decodeBinEP( ) )
        {
          offset[k] = -offset[k];
        }
      }
      // sao_band_position
      sao_pars.typeAuxInfo = m_BinDecoder.decodeBinsEP( NUM_SAO_BO_CLASSES_LOG2 );
      for( int k = 0; k < 4; k++ )
      {
        sao_pars.offset[ ( sao_pars.typeAuxInfo + k ) % MAX_NUM_SAO_CLASSES ] = offset[k];
      }
      continue;
    }

    // edge offset mode
    sao_pars.typeAuxInfo = 0;
    if( compID != COMP_Cr )
    {
      // sao_eo_class_luma / sao_eo_class_chroma
      sao_pars.typeIdc += m_BinDecoder.decodeBinsEP( NUM_SAO_EO_TYPES_LOG2 );
    }
    else
    {
      sao_pars.typeIdc  = sao_ctu_pars[ COMP_Cb ].typeIdc;
    }
    sao_pars.offset[ SAO_CLASS_EO_FULL_VALLEY ] =  offset[0];
    sao_pars.offset[ SAO_CLASS_EO_HALF_VALLEY ] =  offset[1];
    sao_pars.offset[ SAO_CLASS_EO_PLAIN       ] =  0;
    sao_pars.offset[ SAO_CLASS_EO_HALF_PEAK   ] = -offset[2];
    sao_pars.offset[ SAO_CLASS_EO_FULL_PEAK   ] = -offset[3];
  }
}

//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    void  coding_tree       ( cs, partitioner, cuCtx )
//    bool  split_cu_flag     ( cs, partitioner )
//    split split_cu_mode_mt  ( cs, partitioner )
//================================================================================

void CABACReader::coding_tree( CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, Partitioner* pPartitionerChroma, CUCtx* pCuCtxChroma)
{
  const PPS      &pps         = *cs.pps;
  const UnitArea& currArea    = partitioner.currArea();

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  //Note: do not reset qg at chroma CU
  if( pps.useDQP && partitioner.currQgEnable() && !isChroma(partitioner.chType) )
  {
    cuCtx.qgStart    = true;
    cuCtx.isDQPCoded = false;
  }
  if( cs.slice->chromaQpAdjEnabled && partitioner.currQgChromaEnable() )
  {
    cuCtx.isChromaQpAdjCoded  = false;
    cs.chromaQpAdj = 0;
  }

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if (CS::isDualITree(cs) && pPartitionerChroma != nullptr)
  {
    if (pps.useDQP && pPartitionerChroma->currQgEnable())
    {
      pCuCtxChroma->qgStart    = true;
      pCuCtxChroma->isDQPCoded = false;
    }
    if (cs.slice->chromaQpAdjEnabled && pPartitionerChroma->currQgChromaEnable())
    {
      pCuCtxChroma->isChromaQpAdjCoded = false;
      cs.chromaQpAdj = 0;
    }
  }

  determineNeighborCus( cs, partitioner.currArea(), partitioner.chType, partitioner.treeType );

  const PartSplit splitMode = split_cu_mode( cs, partitioner );

  CHECK( !partitioner.canSplit( splitMode, cs ), "Got an invalid split!" );

  if( splitMode != CU_DONT_SPLIT )
  {
      if (CS::isDualITree(cs) && pPartitionerChroma != nullptr && (partitioner.currArea().lwidth() >= 64 || partitioner.currArea().lheight() >= 64))
      {
        partitioner.splitCurrArea(CU_QUAD_SPLIT, cs);
        pPartitionerChroma->splitCurrArea(CU_QUAD_SPLIT, cs);
        bool beContinue = true;
        bool lumaContinue = true;
        bool chromaContinue = true;

        while (beContinue)
        {
          if (partitioner.currArea().lwidth() > 64 || partitioner.currArea().lheight() > 64)
          {
            if (cs.area.blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              coding_tree(cs, partitioner, cuCtx, pPartitionerChroma, pCuCtxChroma);
            }
            lumaContinue = partitioner.nextPart(cs);
            chromaContinue = pPartitionerChroma->nextPart(cs);
            CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            beContinue = lumaContinue;
          }
          else
          {
            //dual tree coding under 64x64 block
            if (cs.area.blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              coding_tree(cs, partitioner, cuCtx);
            }
            lumaContinue = partitioner.nextPart(cs);
            if (cs.area.blocks[pPartitionerChroma->chType].contains(pPartitionerChroma->currArea().blocks[pPartitionerChroma->chType].pos()))
            {
              coding_tree(cs, *pPartitionerChroma, *pCuCtxChroma);
            }
            chromaContinue = pPartitionerChroma->nextPart(cs);
            CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            beContinue = lumaContinue;
          }
        }
        partitioner.exitCurrSplit();
        pPartitionerChroma->exitCurrSplit();

        //cat the chroma CUs together
        CodingUnit* currentCu = cs.getCU(partitioner.currArea().lumaPos(), CH_L, TREE_L);
        CodingUnit* nextCu = nullptr;
        CodingUnit* tempLastLumaCu = nullptr;
        CodingUnit* tempLastChromaCu = nullptr;
        ChannelType currentChType = currentCu->chType;
        while (currentCu->next != nullptr)
        {
          nextCu = currentCu->next;
          if (currentChType != nextCu->chType && currentChType == CH_L)
          {
            tempLastLumaCu = currentCu;
            if (tempLastChromaCu != nullptr) //swap
            {
              tempLastChromaCu->next = nextCu;
            }
          }
          else if (currentChType != nextCu->chType && currentChType == CH_C)
          {
            tempLastChromaCu = currentCu;
            if (tempLastLumaCu != nullptr) //swap
            {
              tempLastLumaCu->next = nextCu;
            }
          }
          currentCu = nextCu;
          currentChType = currentCu->chType;
        }

        CodingUnit* chromaFirstCu = cs.getCU(pPartitionerChroma->currArea().chromaPos(), CH_C, pPartitionerChroma->treeType);
        tempLastLumaCu->next = chromaFirstCu;
      }
      else
      {
        const ModeType modeTypeParent = partitioner.modeType;
        partitioner.modeType = mode_constraint( cs, partitioner, splitMode ); //change for child nodes
        //decide chroma split or not
        bool chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && partitioner.modeType == MODE_TYPE_INTRA;
        CHECK( chromaNotSplit && partitioner.chType != CH_L, "chType must be luma" );
        if( partitioner.treeType == TREE_D )
        {
          partitioner.treeType = chromaNotSplit ? TREE_L : TREE_D;
        }
      partitioner.splitCurrArea( splitMode, cs );
      do
      {
        if( cs.area.blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
      if( chromaNotSplit )
      {
        CHECK( partitioner.chType != CH_L, "must be luma status" );
        partitioner.chType = CH_C;
        partitioner.treeType = TREE_C;

        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }

        //recover treeType
        partitioner.chType = CH_L;
        partitioner.treeType = TREE_D;
      }

      //recover ModeType
      partitioner.modeType = modeTypeParent;
      }
      return;
  }

  CodingUnit& cu = cs.addCU( CS::getArea( cs, currArea, partitioner.chType, partitioner.treeType ), partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice   = cs.slice;
  cu.tileIdx = cs.pps->getTileIdx( currArea.lumaPos() );
  int lumaQPinLocalDualTree = -1;

  // Predict QP on start of quantization group
  if( cuCtx.qgStart )
  {
    cuCtx.qgStart = false;
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }

  if (pps.useDQP && partitioner.isSepTree(cs) && isChroma(cu.chType))
  {
    const Position chromaCentral(cu.chromaPos().offset(cu.chromaSize().width >> 1, cu.chromaSize().height >> 1));
    const Position lumaRefPos(chromaCentral.x << getComponentScaleX(COMP_Cb, cu.chromaFormat), chromaCentral.y << getComponentScaleY(COMP_Cb, cu.chromaFormat));
    //derive chroma qp, but the chroma qp is saved in cuCtx.qp which is used for luma qp
    //therefore, after decoding the chroma CU, the cuCtx.qp shall be recovered to luma qp in order to decode next luma cu qp
    const CodingUnit* colLumaCu = cs.getLumaCU( lumaRefPos );
    CHECK( colLumaCu == nullptr, "colLumaCU shall exist" );
    lumaQPinLocalDualTree = cuCtx.qp;

    if (colLumaCu) cuCtx.qp = colLumaCu->qp;
  }

  cu.qp = cuCtx.qp;                 //NOTE: CU QP can be changed by deltaQP signaling at TU level
  cu.chromaQpAdj = cs.chromaQpAdj;  //NOTE: CU chroma QP adjustment can be changed by adjustment signaling at TU level

  // coding unit
  coding_unit( cu, partitioner, cuCtx );
  //recover cuCtx.qp to luma qp after decoding the chroma CU
  if( pps.useDQP && partitioner.isSepTree( cs ) && isChroma( cu.chType ) )
  {
    cuCtx.qp = lumaQPinLocalDualTree;
  }

  if (CU::isPLT(cu))
  {
    THROW("no PLT support");
  }
  if( cu.chType == CH_C )
  {
    DTRACE( g_trace_ctx, D_QP, "[chroma CU]x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Cb().x, cu.Cb().y, cu.Cb().width, cu.Cb().height, cu.qp );
  }
  else
  {
  DTRACE( g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, cu.qp );
  }
}

ModeType CABACReader::mode_constraint( CodingStructure& cs, Partitioner &partitioner, PartSplit splitMode )
{
  int val = cs.signalModeCons( splitMode, partitioner, partitioner.modeType );
  if( val == LDT_MODE_TYPE_SIGNAL )
  {
    int ctxIdx = DeriveCtx::CtxModeConsFlag();
    bool flag = m_BinDecoder.decodeBin( Ctx::ModeConsFlag( ctxIdx ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "mode_cons_flag() flag=%d\n", flag );
    return flag ? MODE_TYPE_INTRA : MODE_TYPE_INTER;
  }
  else if( val == LDT_MODE_TYPE_INFER )
  {
    return MODE_TYPE_INTRA;
  }
  else
  {
    return partitioner.modeType;
  }
}

PartSplit CABACReader::split_cu_mode( CodingStructure& cs, Partitioner &partitioner )
{
  PartSplit mode = CU_DONT_SPLIT;

  bool canNo, canQt, canBh, canBv, canTh, canTv;
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv );

  const bool canSpl[6] = { canNo, canQt, canBh, canBv, canTh, canTv };

  unsigned ctxSplit = 0, ctxQtSplit = 0, ctxBttHV = 0, ctxBttH12 = 0, ctxBttV12;
  DeriveCtx::CtxSplit( partitioner, ctxSplit, ctxQtSplit, ctxBttHV, ctxBttH12, ctxBttV12, canSpl );

  bool isSplit = canBh || canBv || canTh || canTv || canQt;

  if( canNo && isSplit )
  {
    isSplit = m_BinDecoder.decodeBin( Ctx::SplitFlag( ctxSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d split=%d\n", ctxSplit, isSplit );
  //DTRACE( g_trace_ctx, D_SYNTAX, "%d split_cu_mode() ctx=%d split=%d\n", DTRACE_GET_COUNTER(g_trace_ctx,D_SYNTAX), ctxSplit, isSplit );

  if( !isSplit )
  {
    return CU_DONT_SPLIT;
  }

  const bool canBtt = canBh || canBv || canTh || canTv;
  bool       isQt   = canQt;

  if( isQt && canBtt )
  {
    isQt = m_BinDecoder.decodeBin( Ctx::SplitQtFlag( ctxQtSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d qt=%d\n", ctxQtSplit, isQt );

  if( isQt )
  {
    return CU_QUAD_SPLIT;
  }

  const bool canHor = canBh || canTh;
  bool        isVer = canBv || canTv;

  if( isVer && canHor )
  {
    isVer = m_BinDecoder.decodeBin( Ctx::SplitHvFlag( ctxBttHV ) );
  }

  const bool can14 = isVer ? canTv : canTh;
  bool        is12 = isVer ? canBv : canBh;

  if( is12 && can14 )
  {
    is12 = m_BinDecoder.decodeBin( Ctx::Split12Flag( isVer ? ctxBttV12 : ctxBttH12 ) );
  }

  if     ( isVer && is12 )  mode = CU_VERT_SPLIT;
  else if( isVer && !is12 ) mode = CU_TRIV_SPLIT;
  else if( !isVer && is12 ) mode = CU_HORZ_SPLIT;
  else                      mode = CU_TRIH_SPLIT;

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctxHv=%d ctx12=%d mode=%d\n", ctxBttHV, isVer ? ctxBttV12 : ctxBttH12, mode );

  return mode;
}

//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    void  coding_unit               ( cu, partitioner, cuCtx )
//    void  cu_skip_flag              ( cu )
//    void  pred_mode                 ( cu )
//    void  part_mode                 ( cu )
//    void  cu_pred_data              ( pus )
//    void  cu_lic_flag               ( cu )
//    void  intra_luma_pred_modes     ( pus )
//    void  intra_chroma_pred_mode    ( cu )
//    void  cu_residual               ( cu, partitioner, cuCtx )
//    void  rqt_root_cbf              ( cu )
//    bool  end_of_ctu                ( cu, cuCtx )
//================================================================================

void CABACReader::coding_unit( CodingUnit &cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  CodingStructure& cs = *cu.cs;
  CHECK( cu.treeType != partitioner.treeType || cu.modeType != partitioner.modeType, "treeType or modeType mismatch" );
  DTRACE( g_trace_ctx, D_SYNTAX, "coding_unit() treeType=%d modeType=%d\n", cu.treeType, cu.modeType );
  cu.initPuData();

  // skip flag
  if ((!cs.slice->isIntra() || cs.slice->sps->IBC) && cu.Y().valid())
  {
    cu_skip_flag( cu );
  }

  // skip data
  if( cu.skip )
  {
    cu.colorTransform = false;
    cs.addEmptyTUs( partitioner, &cu );
    MergeCtx           mrgCtx;
    prediction_unit  ( cu, mrgCtx );
    end_of_ctu( cu, cuCtx );
    return;
  }

  // prediction mode and partitioning data
  pred_mode ( cu );
  if (CU::isIntra(cu))
  {
    adaptive_color_transform(cu);
  }
  if (CU::isPLT(cu))
  {
    THROW("no support");
    return;
  }

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  // check end of cu
  end_of_ctu( cu, cuCtx );
}

void CABACReader::cu_skip_flag( CodingUnit& cu )
{
  if ((cu.slice->isIntra() || CU::isConsIntra(cu)) && cu.cs->slice->sps->IBC)
  {
    cu.skip = false;
    cu.rootCbf = false;
    cu.predMode = MODE_INTRA;
    cu.mmvdSkip = false;
    if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
    {
      unsigned ctxId = DeriveCtx::CtxSkipFlag();
      unsigned skip = m_BinDecoder.decodeBin(Ctx::SkipFlag(ctxId));
      DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0 );
      if (skip)
      {
        cu.skip = true;
        cu.rootCbf = false;
        cu.predMode = MODE_IBC;
        cu.mmvdSkip = false;
      }
    }
    return;
  }
  if ( !cu.cs->slice->sps->IBC && cu.lwidth() == 4 && cu.lheight() == 4 )
  {
    return;
  }
  if( !cu.cs->slice->sps->IBC && CU::isConsIntra(cu) )
  {
    return;
  }
  unsigned ctxId  = DeriveCtx::CtxSkipFlag();
  unsigned skip   = m_BinDecoder.decodeBin( Ctx::SkipFlag(ctxId) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0 );

  if (skip && cu.cs->slice->sps->IBC)
  {
    if (cu.lwidth() < 128 && cu.lheight() < 128 && !CU::isConsInter(cu)) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
    {
      if ( cu.lwidth() == 4 && cu.lheight() == 4 )
      {
        cu.skip     = true;
        cu.rootCbf  = false;
        cu.predMode = MODE_IBC;
        cu.mmvdSkip = false;
        return;
      }
      unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
      if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
      {
        cu.skip = true;
        cu.rootCbf = false;
        cu.predMode = MODE_IBC;
        cu.mmvdSkip = false;
        cu.regularMergeFlag = false;
      }
      else
      {
        cu.predMode = MODE_INTER;
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode);
    }
    else
    {
      cu.predMode = MODE_INTER;
    }
  }
  if ((skip && CU::isInter(cu) && cu.cs->slice->sps->IBC) ||
    (skip && !cu.cs->slice->sps->IBC))
  {
    cu.skip     = true;
    cu.rootCbf  = false;
    cu.predMode = MODE_INTER;
  }
}

void CABACReader::imv_mode( CodingUnit& cu, MergeCtx& mrgCtx )
{
  if( !cu.cs->sps->AMVR )
  {
    return;
  }

  bool bNonZeroMvd = CU::hasSubCUNonZeroMVd( cu );
  if( !bNonZeroMvd )
  {
    return;
  }

  if ( cu.affine )
  {
    return;
  }

  const SPS *sps = cu.cs->sps;

  unsigned value = 0;
  if (CU::isIBC(cu))
  {
    value = 1;
  }
  else
  {
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 0 ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 0 );

    cu.imv = value;
  if( sps->AMVR && value )
  {
    if (!CU::isIBC(cu))
    {
      value = m_BinDecoder.decodeBin(Ctx::ImvFlag(4));
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 4);
      cu.imv = value ? 1 : IMV_HPEL;
    }
    if (value)
    {
      value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 1 ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 1 );
      value++;
      cu.imv = value;
    }
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv );
}

void CABACReader::affine_amvr_mode( CodingUnit& cu, MergeCtx& mrgCtx )
{
  const SPS* sps = cu.slice->sps;

  if( !sps->AffineAmvr || !cu.affine )
  {
    return;
  }

  if ( !CU::hasSubCUNonZeroAffineMVd( cu ) )
  {
    return;
  }

  unsigned value = 0;
  value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 2 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 2 );

  if( value )
  {
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 3 );
    value++;
  }

  cu.imv = value;
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv );
}

void CABACReader::pred_mode( CodingUnit& cu )
{
  if (cu.cs->slice->sps->IBC && cu.chType != CH_C)
  {
    if( CU::isConsInter(cu) )
    {
      cu.predMode = MODE_INTER;
      return;
    }

    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || CU::isConsIntra(cu) )
    {
      cu.predMode = MODE_INTRA;
      if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
      {
      unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
      if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
      {
        cu.predMode = MODE_IBC;
      }
      }
      if (!CU::isIBC(cu) && cu.cs->slice->sps->PLT && cu.lwidth() <= 64 && cu.lheight() <= 64 && cu.lwidth() * cu.lheight() > 16)
      {
        if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
    else
    {
      if (m_BinDecoder.decodeBin(Ctx::PredMode(DeriveCtx::CtxPredModeFlag())))
      {
        cu.predMode = MODE_INTRA;
        if (cu.cs->slice->sps->PLT && cu.lwidth() <= 64 && cu.lheight() <= 64 && cu.lwidth() * cu.lheight() > 16)
        {
          if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
          {
            cu.predMode = MODE_PLT;
          }
        }
      }
      else
      {
        cu.predMode = MODE_INTER;
        if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
        {
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
        {
          cu.predMode = MODE_IBC;
        }
        }
      }
    }
  }
  else
  {
    if( CU::isConsInter(cu) )
    {
      cu.predMode = MODE_INTER;
      return;
    }

    if ( cu.cs->slice->isIntra() || (cu.lwidth() == 4 && cu.lheight() == 4) || CU::isConsIntra(cu) )
    {
      cu.predMode = MODE_INTRA;
      if (cu.cs->slice->sps->PLT && cu.lwidth() <= 64 && cu.lheight() <= 64 && (((isLuma(cu.chType) && cu.lwidth() * cu.lheight() > 16) || (!isLuma(cu.chType) && cu.chromaSize().area() > 16))) && (!CU::isLocalSepTree(cu) || isLuma(cu.chType)  )  )
      {
        if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
    else
    {
      cu.predMode = m_BinDecoder.decodeBin(Ctx::PredMode(DeriveCtx::CtxPredModeFlag())) ? MODE_INTRA : MODE_INTER;
      if (cu.cs->slice->sps->PLT && cu.lwidth() <= 64 && cu.lheight() <= 64 && (((isLuma(cu.chType) && cu.lwidth() * cu.lheight() > 16) || (!isLuma(cu.chType) && cu.chromaSize().area() > 16)))&& (!CU::isLocalSepTree(cu) || isLuma(cu.chType)  )  )
      {
        if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
  }
}

void CABACReader::bdpcm_mode( CodingUnit& cu, const ComponentID compID )
{
  if (!CU::bdpcmAllowed(cu, compID))
  {
     if (isLuma(compID))
     {
       cu.bdpcmM[CH_L] = 0;
       if (!CS::isDualITree(*cu.cs))
       {
         cu.bdpcmM[CH_C] = 0;
       }
     }
     else
     {
       cu.bdpcmM[CH_C] = 0;
     }
     return;
  }

  unsigned ctxId = isLuma( compID ) ? 0 : 2;
  int bdpcmMode = m_BinDecoder.decodeBin( Ctx::BDPCMMode(ctxId) );
  if (bdpcmMode)
  {
    bdpcmMode += m_BinDecoder.decodeBin( Ctx::BDPCMMode(ctxId+1) );
  }

  cu.bdpcmM[toChannelType(compID)] = bdpcmMode;

  if (isLuma(compID))
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CH_L, cu.lumaPos().x, cu.lumaPos().y, cu.lwidth(), cu.lheight(), cu.bdpcmM[CH_L]);
  }
  else
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CH_C, cu.chromaPos().x, cu.chromaPos().y, cu.chromaSize().width, cu.chromaSize().height, cu.bdpcmM[CH_C]);
  }
}

void CABACReader::cu_pred_data( CodingUnit &cu )
{
  if( CU::isIntra( cu ) )
  {
    if( cu.Y().valid() )
    {
      bdpcm_mode(cu, COMP_Y );
    }
    intra_luma_pred_modes( cu );
    if( ( !cu.Y().valid() || (!CU::isSepTree(cu) && cu.Y().valid() ) ) && isChromaEnabled(cu.chromaFormat) )
    {
      bdpcm_mode(cu, ComponentID(CH_C));
    } 
    intra_chroma_pred_modes( cu );
    return;
  }
  if (!cu.Y().valid()) // dual tree chroma CU
  {
    cu.predMode = MODE_IBC;
    return;
  }
  MergeCtx mrgCtx;

  prediction_unit( cu, mrgCtx );

  imv_mode   ( cu, mrgCtx );
  affine_amvr_mode( cu, mrgCtx );
  cu_bcw_flag( cu );
}

void CABACReader::cu_bcw_flag(CodingUnit& cu)
{
  if(!CU::isBcwIdxCoded(cu))
  {
    return;
  }

  CHECK(!(BCW_NUM > 1 && (BCW_NUM == 2 || (BCW_NUM & 0x01) == 1)), " !( BCW_NUM > 1 && ( BCW_NUM == 2 || ( BCW_NUM & 0x01 ) == 1 ) ) ");

  uint32_t idx = 0;

  uint32_t symbol = m_BinDecoder.decodeBin(Ctx::BcwIdx(0));

  int32_t numBcw = (cu.slice->checkLDC) ? 5 : 3;
  if(symbol == 1)
  {
    uint32_t prefixNumBits = numBcw - 2;
    uint32_t step = 1;

    idx = 1;

    for(int ui = 0; ui < prefixNumBits; ++ui)
    {
      symbol = m_BinDecoder.decodeBinEP();
      if (symbol == 0)
      {
        break;
      }
      idx += step;
    }
  }

  uint8_t BcwIdx = (uint8_t)g_BcwParsingOrder[idx];
  CU::setBcwIdx( cu, BcwIdx );

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_BCW_flag() BCW_idx=%d\n", cu.BcwIdx ? 1 : 0);
}

void CABACReader::xReadTruncBinCode(uint32_t& symbol, uint32_t maxSymbol)
{
  int thresh;
  if (maxSymbol > 256)
  {
    int threshVal = 1 << 8;
    thresh = 8;
    while (threshVal <= maxSymbol)
    {
      thresh++;
      threshVal <<= 1;
    }
    thresh--;
  }
  else
  {
    thresh = g_tbMax[maxSymbol];
  }

  int val = 1 << thresh;
  int b = maxSymbol - val;
  symbol = m_BinDecoder.decodeBinsEP(thresh);
  if (symbol >= val - b)
  {
    uint32_t altSymbol;
    altSymbol = m_BinDecoder.decodeBinEP();
    symbol <<= 1;
    symbol += altSymbol;
    symbol -= (val - b);
  }
}

void CABACReader::extend_ref_line(CodingUnit& cu)
{
  if( !cu.cs->sps->MRL || !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.bdpcmM[CH_L] )
  {
    cu.multiRefIdx = 0;
    return;
  }

  bool isFirstLineOfCtu = (((cu.block(COMP_Y).y)&((cu.cs->sps)->CTUSize - 1)) == 0);
  if (isFirstLineOfCtu)
  {
    cu.multiRefIdx = 0;
  }
  else
  {
    int multiRefIdx = 0;

    if (MRL_NUM_REF_LINES > 1)
    {
      multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(0)) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];
      if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
      {
        multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(1)) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
      }

    }
    cu.multiRefIdx = multiRefIdx;
//    DTRACE( g_trace_ctx, D_SYNTAX, "extend_ref_line() idx=%d pos=(%d,%d) mode=%d\n", 0, cu.lumaPos().x, cu.lumaPos().y, multiRefIdx );
  }
}

void CABACReader::intra_luma_pred_modes( CodingUnit &cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }

  if( cu.bdpcmM[CH_L] )
  {
    cu.intraDir[0] = cu.bdpcmM[CH_L] == 2? VER_IDX : HOR_IDX;
    return;
  }

  mip_flag(cu);
  if (cu.mipFlag)
  {
    mip_pred_modes(cu);
    return;
  }
  extend_ref_line( cu );
  isp_mode( cu );

  // prev_intra_luma_pred_flag
  int mpmFlag;
  if ( cu.multiRefIdx )
  {
    mpmFlag = true;
  }
  else
  {
    mpmFlag = m_BinDecoder.decodeBin(Ctx::IntraLumaMpmFlag());
  }

  unsigned mpm_pred[NUM_MOST_PROBABLE_MODES];  // mpm_idx / rem_intra_luma_pred_mode
  {
    CU::getIntraMPMs( cu, mpm_pred );

    if( mpmFlag )
    {
      uint32_t ipred_idx = 0;
      {
        unsigned ctx = (cu.ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0);
        if (cu.multiRefIdx == 0)
        {
          ipred_idx = m_BinDecoder.decodeBin(Ctx::IntraLumaPlanarFlag(ctx));
        }
        else
        {
          ipred_idx = 1;
        }
        if( ipred_idx )
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
        if (ipred_idx > 1)
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
        if (ipred_idx > 2)
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
        if (ipred_idx > 3)
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
      }
      cu.intraDir[0] = mpm_pred[ipred_idx];
    }
    else
    {
      unsigned ipred_mode = 0;
      xReadTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);
      //postponed sorting of MPMs (only in remaining branch)
      std::sort( mpm_pred, mpm_pred + NUM_MOST_PROBABLE_MODES );

      for( uint32_t i = 0; i < NUM_MOST_PROBABLE_MODES; i++ )
      {
        ipred_mode += (ipred_mode >= mpm_pred[i]);
      }

      cu.intraDir[0] = ipred_mode;
    }

    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", 0, cu.lumaPos().x, cu.lumaPos().y, cu.intraDir[0] );
  }
}

void CABACReader::intra_chroma_pred_modes( CodingUnit& cu )
{
  if( cu.chromaFormat == CHROMA_400 || ( CU::isSepTree(cu) && cu.chType == CH_L ) )
  {
    return;
  }

  if( cu.bdpcmM[CH_C] )
  {
    cu.intraDir[1] = cu.bdpcmM[CH_C] == 2 ? VER_IDX : HOR_IDX;
    return;
  }

  intra_chroma_pred_mode( cu );
}

bool CABACReader::intra_chroma_lmc_mode(CodingUnit& cu)
{
  int lmModeList[10];
  CU::getLMSymbolList(cu, lmModeList);

  int symbol = m_BinDecoder.decodeBin(Ctx::CclmModeIdx(0));

  if (symbol == 0)
  {
    cu.intraDir[1] = lmModeList[symbol];
    CHECK(cu.intraDir[1] != LM_CHROMA_IDX, "should be LM_CHROMA");
  }
  else
  {
    symbol += m_BinDecoder.decodeBinEP();
    cu.intraDir[1] = lmModeList[symbol];
  }
  return true; //it will only enter this function for LMC modes, so always return true ;
}

void CABACReader::intra_chroma_pred_mode(CodingUnit& cu)
{
  if (cu.colorTransform)
  {
    cu.intraDir[CH_C] = DM_CHROMA_IDX;
    return;
  }

  // LM chroma mode
  if (cu.cs->sps->LMChroma && CU::checkCCLMAllowed(cu))
  {
    bool isLMCMode = m_BinDecoder.decodeBin(Ctx::CclmModeFlag(0)) ? true : false;
    if (isLMCMode)
    {
      intra_chroma_lmc_mode(cu);
      return;
    }
  }

  if (m_BinDecoder.decodeBin(Ctx::IntraChromaPredMode(0)) == 0)
  {
    cu.intraDir[1] = DM_CHROMA_IDX;
    return;
  }

  unsigned candId = m_BinDecoder.decodeBinsEP(2);

  unsigned chromaCandModes[NUM_CHROMA_MODE];
  CU::getIntraChromaCandModes(cu, chromaCandModes);

  CHECK(candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds");
  CHECK(CU::isLMCMode(chromaCandModes[candId]), "The intra dir cannot be LM_CHROMA for this path");
  CHECK(chromaCandModes[candId] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path");

  cu.intraDir[1] = chromaCandModes[candId];
}

void CABACReader::cu_residual( CodingUnit& cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  if (!CU::isIntra(cu))
  {
    if( !cu.mergeFlag )
    {
      rqt_root_cbf( cu );
    }
    else
    {
      cu.rootCbf = true;
    }
    if( cu.rootCbf )
    {
      sbt_mode( cu );
    }
    if( !cu.rootCbf )
    {
      cu.colorTransform = false;
      cu.cs->addEmptyTUs( partitioner, &cu);
      return;
    }
  }

  if (CU::isInter(cu) || CU::isIBC(cu))
  {
    adaptive_color_transform(cu);
  }

  cuCtx.violatesLfnstConstrained[CH_L]   = false;
  cuCtx.violatesLfnstConstrained[CH_C] = false;
  cuCtx.lfnstLastScanPos = false;
  cuCtx.violatesMtsCoeffConstraint                    = false;
  cuCtx.mtsLastScanPos                                = false;


  ChromaCbfs chromaCbfs;
  if( cu.ispMode && isLuma( partitioner.chType ) )
  {
    Partitioner subTuPartitioner = partitioner;
    transform_tree(*cu.cs, subTuPartitioner, cuCtx, cu, CU::getISPType(cu, getFirstComponentOfChannel(partitioner.chType)), 0);
  }
  else
  {
    transform_tree(*cu.cs, partitioner, cuCtx, cu);
  }
  residual_lfnst_mode( cu, cuCtx );
  mts_idx            ( cu, cuCtx );
}

void CABACReader::rqt_root_cbf( CodingUnit& cu )
{
  cu.rootCbf = ( m_BinDecoder.decodeBin( Ctx::QtRootCbf() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}

void CABACReader::adaptive_color_transform(CodingUnit& cu)
{
  if (!cu.slice->sps->useColorTrans)
  {
    return;
  }

  if (CU::isSepTree(cu))
  {
    return;
  }

  if (CU::isInter(cu) || CU::isIBC(cu) || CU::isIntra(cu))
  {
    cu.colorTransform = (m_BinDecoder.decodeBin(Ctx::ACTFlag()));
  }
}

void CABACReader::sbt_mode( CodingUnit& cu )
{
  const uint8_t sbtAllowed = CU::checkAllowedSbt(cu);
  if( !sbtAllowed )
  {
    return;
  }

  SizeType cuWidth = cu.lwidth();
  SizeType cuHeight = cu.lheight();

  //bin - flag
  uint8_t ctxIdx = ( cuWidth * cuHeight <= 256 ) ? 1 : 0;
  bool sbtFlag = m_BinDecoder.decodeBin( Ctx::SbtFlag( ctxIdx ) );
  if( !sbtFlag )
  {
    return;
  }

  uint8_t sbtVerHalfAllow = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed );
  uint8_t sbtHorHalfAllow = CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  uint8_t sbtVerQuadAllow = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed );
  uint8_t sbtHorQuadAllow = CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );

  //bin - type
  bool sbtQuadFlag = false;
  if( ( sbtHorHalfAllow || sbtVerHalfAllow ) && ( sbtHorQuadAllow || sbtVerQuadAllow ) )
  {
    sbtQuadFlag = m_BinDecoder.decodeBin( Ctx::SbtQuadFlag( 0 ) );
  }
  else
  {
    sbtQuadFlag = 0;
  }

  //bin - dir
  bool sbtHorFlag = false;
  if( ( sbtQuadFlag && sbtVerQuadAllow && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtVerHalfAllow && sbtHorHalfAllow ) ) //both direction allowed
  {
    uint8_t ctxIdx = ( cuWidth == cuHeight ) ? 0 : ( cuWidth < cuHeight ? 1 : 2 );
    sbtHorFlag = m_BinDecoder.decodeBin( Ctx::SbtHorFlag( ctxIdx ) );
  }
  else
  {
    sbtHorFlag = ( sbtQuadFlag && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtHorHalfAllow );
  }
  const uint8_t sbtIdx = sbtHorFlag ? ( sbtQuadFlag ? SBT_HOR_QUAD : SBT_HOR_HALF ) : ( sbtQuadFlag ? SBT_VER_QUAD : SBT_VER_HALF );

  //bin - pos
  bool sbtPosFlag = m_BinDecoder.decodeBin( Ctx::SbtPosFlag( 0 ) );

  const uint8_t sbtPos =  sbtPosFlag ? SBT_POS1 : SBT_POS0;
  cu.sbtInfo = (sbtPos << 4) + sbtIdx;

  DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo );
}


void CABACReader::end_of_ctu( CodingUnit& cu, CUCtx& cuCtx )
{
  const Position rbPos = recalcPosition( cu.chromaFormat, cu.chType, CH_L, cu.blocks[cu.chType].bottomRight().offset( 1, 1 ) );

  if ( ( ( rbPos.x & cu.cs->pcv->maxCUSizeMask  ) == 0 || rbPos.x == cu.cs->pcv->lumaWidth )
    && ( ( rbPos.y & cu.cs->pcv->maxCUSizeMask ) == 0 || rbPos.y == cu.cs->pcv->lumaHeight )
    && ( !CU::isSepTree(cu) || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType ) )
      )
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->useDQP && !cuCtx.isDQPCoded );
  }
}

void CABACReader::cu_palette_info(CodingUnit& cu, ComponentID compBegin, uint32_t numComp, CUCtx& cuCtx)
{
  THROW("no support");
}


//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( cu, mrgCtx );
//    void  merge_flag      ( cu );
//    void  merge_data      ( cu, mrgCtx );
//    void  merge_idx       ( cu );
//    void  inter_pred_idc  ( cu );
//    void  ref_idx         ( cu, refList );
//    void  mvp_flag        ( cu, refList );
//================================================================================

void CABACReader::prediction_unit( CodingUnit& cu, MergeCtx& mrgCtx )
{
  if( cu.skip )
  {
    cu.mergeFlag = true;
  }
  else
  {
    merge_flag( cu );
  }
  if( cu.mergeFlag )
  {
    merge_data(cu);
  }
  else if (CU::isIBC(cu))
  {
    cu.interDir = 1;
    cu.affine = false;
    cu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;
    mvd_coding( cu.mvd[REF_PIC_LIST_0][0] );
    if ( cu.cs->sps->maxNumIBCMergeCand == 1 )
    {
      cu.mvpIdx[REF_PIC_LIST_0] = 0;
    }
    else
    {
      mvp_flag(cu, REF_PIC_LIST_0);
    }
  }
  else
  {
    inter_pred_idc( cu );
    affine_flag   ( cu );
    smvd_mode( cu );

    if( cu.interDir != 2 /* PRED_L1 */ )
    {
      ref_idx     ( cu, REF_PIC_LIST_0 );
      if( cu.affine )
      {
        mvd_coding( cu.mvd[REF_PIC_LIST_0][0] );
        mvd_coding( cu.mvd[REF_PIC_LIST_0][1] );
        if ( cu.affineType == AFFINEMODEL_6PARAM )
        {
          mvd_coding( cu.mvd[REF_PIC_LIST_0][2] );
        }
      }
      else
      {
        mvd_coding( cu.mvd[REF_PIC_LIST_0][0] );
      }
      mvp_flag    ( cu, REF_PIC_LIST_0 );
    }

    if( cu.interDir != 1 /* PRED_L0 */ )
    {
      if ( cu.smvdMode != 1 )
      {
        ref_idx     ( cu, REF_PIC_LIST_1 );
        if( cu.cs->slice->picHeader->mvdL1Zero && cu.interDir == 3 /* PRED_BI */ )
        {
          cu.mvd[REF_PIC_LIST_1][0] = Mv();
          cu.mvd[REF_PIC_LIST_1][1] = Mv();
          cu.mvd[REF_PIC_LIST_1][2] = Mv();
        }
        else if( cu.affine )
        {
          mvd_coding( cu.mvd[REF_PIC_LIST_1][0] );
          mvd_coding( cu.mvd[REF_PIC_LIST_1][1] );
          if ( cu.affineType == AFFINEMODEL_6PARAM )
          {
            mvd_coding( cu.mvd[REF_PIC_LIST_1][2] );
          }
        }
        else
        {
          mvd_coding( cu.mvd[REF_PIC_LIST_1][0] );
        }
      }
      mvp_flag    ( cu, REF_PIC_LIST_1 );
    }
  }
  if( cu.interDir == 3 /* PRED_BI */ && CU::isBipredRestriction(cu) )
  {
    cu.mv [REF_PIC_LIST_1][0] = Mv(0, 0);
    cu.refIdx[REF_PIC_LIST_1] = -1;
    cu.interDir               =  1;
    cu.BcwIdx = BCW_DEFAULT;
  }

  if ( cu.smvdMode )
  {
    RefPicList eCurRefList = (RefPicList)(cu.smvdMode - 1);
    cu.mvd[1 - eCurRefList][0].set( -cu.mvd[eCurRefList][0].hor, -cu.mvd[eCurRefList][0].ver );
    CHECK(!((cu.mvd[1 - eCurRefList][0].hor >= MVD_MIN) && (cu.mvd[1 - eCurRefList][0].hor <= MVD_MAX)) || !((cu.mvd[1 - eCurRefList][0].ver >= MVD_MIN) && (cu.mvd[1 - eCurRefList][0].ver <= MVD_MAX)), "Illegal MVD value");
    cu.refIdx[1 - eCurRefList] = cu.cs->slice->symRefIdx[ 1 - eCurRefList ];
  }

  CU::spanMotionInfo( cu, mrgCtx );
}

void CABACReader::smvd_mode( CodingUnit& cu )
{
  cu.smvdMode = 0;
  if ( cu.interDir != 3 || cu.affine )
  {
    return;
  }

  if ( cu.cs->slice->biDirPred == false )
  {
    return;
  }

  cu.smvdMode = m_BinDecoder.decodeBin( Ctx::SmvdFlag() ) ? 1 : 0;

  DTRACE( g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", cu.smvdMode ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height );
}

void CABACReader::subblock_merge_flag( CodingUnit& cu )
{
  cu.affine = false;

  if ( !cu.cs->slice->isIntra() && (cu.slice->picHeader->maxNumAffineMergeCand > 0) && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8 )
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag();
    cu.affine = m_BinDecoder.decodeBin( Ctx::SubblockMergeFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
  }
}

void CABACReader::affine_flag( CodingUnit& cu )
{
  if ( !cu.cs->slice->isIntra() && cu.cs->sps->Affine && cu.lumaSize().width > 8 && cu.lumaSize().height > 8 )
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag();
    cu.affine = m_BinDecoder.decodeBin( Ctx::AffineFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );

    if ( cu.affine && cu.cs->sps->AffineType )
    {
      ctxId = 0;
      cu.affineType = m_BinDecoder.decodeBin( Ctx::AffineType( ctxId ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n", cu.affineType ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
    }
    else
    {
      cu.affineType = AFFINEMODEL_4PARAM;
    }
  }
}

void CABACReader::merge_flag( CodingUnit& cu )
{
  cu.mergeFlag = ( m_BinDecoder.decodeBin( Ctx::MergeFlag() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", cu.mergeFlag ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height );

  if (cu.mergeFlag && CU::isIBC(cu))
  {
    cu.mmvdMergeFlag = false;
    cu.regularMergeFlag = false;
    return;
  }
}


void CABACReader::merge_data( CodingUnit& cu )
{
  if (CU::isIBC(cu))
  {
    merge_idx(cu);
    return;
  }
  else
  {
    subblock_merge_flag(cu);
    if (cu.affine)
    {
      merge_idx(cu);
      cu.regularMergeFlag = false;
      return;
    }

    const bool ciipAvailable = cu.cs->sps->CIIP && !cu.skip && cu.lwidth() < MAX_CU_SIZE && cu.lheight() < MAX_CU_SIZE && cu.lwidth() * cu.lheight() >= 64;
    const bool geoAvailable = cu.cs->slice->sps->GEO && cu.cs->slice->isInterB() && cu.cs->sps->maxNumGeoCand > 1
                                                                      && cu.lwidth() >= GEO_MIN_CU_SIZE && cu.lheight() >= GEO_MIN_CU_SIZE
                                                                      && cu.lwidth() <= GEO_MAX_CU_SIZE && cu.lheight() <= GEO_MAX_CU_SIZE
                                                                      && cu.lwidth() < 8 * cu.lheight() && cu.lheight() < 8 * cu.lwidth();
    if (geoAvailable || ciipAvailable)
    {
      cu.regularMergeFlag = m_BinDecoder.decodeBin(Ctx::RegularMergeFlag(cu.skip ? 0 : 1));
    }
    else
    {
      cu.regularMergeFlag = true;
    }
    if (cu.regularMergeFlag)
    {
      if (cu.cs->slice->sps->MMVD)
      {
        cu.mmvdMergeFlag = m_BinDecoder.decodeBin(Ctx::MmvdFlag(0));
        DTRACE( g_trace_ctx, D_SYNTAX, "mmvd_merge_flag() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", cu.mmvdMergeFlag ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height );
      }
      else
      {
        cu.mmvdMergeFlag = false;
      }
      if (cu.skip)
      {
        cu.mmvdSkip = cu.mmvdMergeFlag;
      }
    }
    else
    {
      cu.mmvdMergeFlag = false;
      cu.mmvdSkip = false;
      if (geoAvailable && ciipAvailable)
      {
        Ciip_flag(cu);
      }
      else if (ciipAvailable)
      {
        cu.ciip = true;
      }
      else
      {
        cu.ciip = false;
      }
      if (cu.ciip)
      {
        cu.intraDir[0] = PLANAR_IDX;
        cu.intraDir[1] = DM_CHROMA_IDX;
      }
      else
      {
        cu.geo = true;
      }
    }
  }
  if (cu.mmvdMergeFlag || cu.mmvdSkip)
  {
    mmvd_merge_idx(cu);
  }
  else
  {
    merge_idx(cu);
  }
}


void CABACReader::merge_idx( CodingUnit& cu )
{
  if ( cu.affine )
  {
    int numCandminus1 = int( cu.cs->picHeader->maxNumAffineMergeCand ) - 1;
    cu.mergeIdx = 0;
    if ( numCandminus1 > 0 )
    {
      if ( m_BinDecoder.decodeBin( Ctx::AffMergeIdx() ) )
      {
        cu.mergeIdx++;
        for ( ; cu.mergeIdx < numCandminus1; cu.mergeIdx++ )
        {
            if ( !m_BinDecoder.decodeBinEP() )
            {
              break;
            }
        }
      }
    }
    DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", cu.mergeIdx );
  }
  else
  {
  int numCandminus1 = int( cu.cs->sps->maxNumMergeCand ) - 1;
  cu.mergeIdx       = 0;

  if( cu.geo )
  {
    uint32_t splitDir  = 0;
    xReadTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
    cu.geoSplitDir = splitDir;
    const int maxNumGeoCand = cu.cs->sps->maxNumGeoCand;
    CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
    CHECK(cu.lheight() > 64 || cu.lwidth() > 64, "Incorrect block size of geo flag");
    int numCandminus2 = maxNumGeoCand - 2;
    cu.mergeIdx = 0;
    int mergeCand0 = 0;
    int mergeCand1 = 0;
    if( m_BinDecoder.decodeBin( Ctx::MergeIdx() ) )
    {
      mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
    }
    if (numCandminus2 > 0)
    {
      if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
      {
        mergeCand1 += unary_max_eqprob(numCandminus2 - 1) + 1;
      }
    }
    mergeCand1 += mergeCand1 >= mergeCand0 ? 1 : 0;
    cu.geoMergeIdx0 = mergeCand0;
    cu.geoMergeIdx1 = mergeCand1;
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%d\n", splitDir );
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx0=%d\n", mergeCand0 );
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx1=%d\n", mergeCand1 );
    return;
  }

  if (cu.predMode == MODE_IBC)
  {
    numCandminus1 = int(cu.cs->sps->maxNumIBCMergeCand) - 1;
  }
  if( numCandminus1 > 0 )
  {
    if( m_BinDecoder.decodeBin( Ctx::MergeIdx() ) )
    {
      cu.mergeIdx++;
      for( ; cu.mergeIdx < numCandminus1; cu.mergeIdx++ )
      {
          if( !m_BinDecoder.decodeBinEP() )
          {
            break;
          }
      }
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", cu.mergeIdx );
  }
}

void CABACReader::mmvd_merge_idx(CodingUnit& cu)
{
  int var0 = 0;
  if (cu.cs->sps->maxNumMergeCand > 1)
  {
    static_assert(MMVD_BASE_MV_NUM == 2, "");
    var0 = m_BinDecoder.decodeBin(Ctx::MmvdMergeIdx());
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", var0);
  int numCandminus1_step = MMVD_REFINE_STEP - 1;
  int var1 = 0;
  if (m_BinDecoder.decodeBin(Ctx::MmvdStepMvpIdx()))
  {
    var1++;
    for (; var1 < numCandminus1_step; var1++)
    {
      if (!m_BinDecoder.decodeBinEP())
      {
        break;
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "MmvdStepMvpIdx() MmvdStepMvpIdx=%d\n", var1);
  int var2 = 0;
  if (m_BinDecoder.decodeBinEP())
  {
    var2 += 2;
    if (m_BinDecoder.decodeBinEP())
    {
      var2 += 1;
    }
  }
  else
  {
    var2 += 0;
    if (m_BinDecoder.decodeBinEP())
    {
      var2 += 1;
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", var2);
  int mvpIdx = (var0 * MMVD_MAX_REFINE_NUM + var1 * 4 + var2);
  cu.mmvdMergeIdx = mvpIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", cu.mmvdMergeIdx);
}

void CABACReader::inter_pred_idc( CodingUnit& cu )
{
  if( cu.cs->slice->isInterP() )
  {
    cu.interDir = 1;
    return;
  }
  if( !(CU::isBipredRestriction(cu)) )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(cu);
    if( m_BinDecoder.decodeBin( Ctx::InterDir(ctxId) ) )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, 3, cu.lumaPos().x, cu.lumaPos().y );
      cu.interDir = 3;
      return;
    }
  }
  if( m_BinDecoder.decodeBin( Ctx::InterDir(5) ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=5 value=%d pos=(%d,%d)\n", 2, cu.lumaPos().x, cu.lumaPos().y );
    cu.interDir = 2;
    return;
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=5 value=%d pos=(%d,%d)\n", 1, cu.lumaPos().x, cu.lumaPos().y );
  cu.interDir = 1;
  return;
}

void CABACReader::ref_idx( CodingUnit& cu, RefPicList eRefList )
{
  if ( cu.smvdMode )
  {
    cu.refIdx[eRefList] = cu.cs->slice->symRefIdx[ eRefList ];
    return;
  }

  int numRef  = cu.cs->slice->numRefIdx[eRefList];

  if( numRef <= 1 || !m_BinDecoder.decodeBin( Ctx::RefPic() ) )
  {
    if( numRef > 1 )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 0, cu.lumaPos().x, cu.lumaPos().y );
    }
    cu.refIdx[eRefList] = 0;
    return;
  }
  if( numRef <= 2 || !m_BinDecoder.decodeBin( Ctx::RefPic(1) ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 1, cu.lumaPos().x, cu.lumaPos().y );
    cu.refIdx[eRefList] = 1;
    return;
  }
  for( int idx = 3; ; idx++ )
  {
    if( numRef <= idx || !m_BinDecoder.decodeBinEP() )
    {
      cu.refIdx[eRefList] = (signed char)( idx - 1 );
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", idx-1, cu.lumaPos().x, cu.lumaPos().y );
      return;
    }
  }
}

void CABACReader::mvp_flag( CodingUnit& cu, RefPicList eRefList )
{
  unsigned mvp_idx = m_BinDecoder.decodeBin( Ctx::MVPIdx() );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", mvp_idx, cu.lumaPos().x, cu.lumaPos().y );
  cu.mvpIdx [eRefList] = mvp_idx;
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, mvp_idx );
}


void CABACReader::Ciip_flag(CodingUnit& cu)
{
  if (!cu.cs->sps->CIIP)
  {
    cu.ciip = false;
    return;
  }
  if (cu.skip)
  {
    cu.ciip = false;
    return;
  }

  cu.ciip = (m_BinDecoder.decodeBin(Ctx::CiipFlag()));
  DTRACE(g_trace_ctx, D_SYNTAX, "Ciip_flag() Ciip=%d pos=(%d,%d) size=%dx%d\n", cu.ciip ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height);
}


//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( depth )
//    bool  cbf_comp            ( area, depth )
//================================================================================
void CABACReader::transform_tree(CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, CodingUnit& cuTop, const PartSplit ispType, const int subTuIdx)
{
  const UnitArea&   area = partitioner.currArea();
  CodingUnit&         cu = *cs.getCU(area.blocks[partitioner.chType], partitioner.chType, partitioner.treeType );
  int       subTuCounter = subTuIdx;

  // split_transform_flag
  bool split = partitioner.canSplit(TU_MAX_TR_SPLIT, cs);
  const unsigned  trDepth = partitioner.currTrDepth;

  if( cu.sbtInfo && partitioner.canSplit( CU::getSbtTuSplit( cu.sbtInfo ), cs ) )
  {
    split = true;
  }

  if( !split && cu.ispMode )
  {
    split = partitioner.canSplitISP(ispType, cs, cuTop);
  }

  if( split )
  {
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
#if ENABLE_TRACING
      const CompArea& tuArea = partitioner.currArea().blocks[partitioner.chType];
      DTRACE( g_trace_ctx, D_SYNTAX, "transform_tree() maxTrSplit chType=%d pos=(%d,%d) size=%dx%d\n", partitioner.chType, tuArea.x, tuArea.y, tuArea.width, tuArea.height );

#endif
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else if( cu.ispMode )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else if( cu.sbtInfo && partitioner.canSplit( CU::getSbtTuSplit( cu.sbtInfo ), cs ) )
    {
      partitioner.splitCurrArea( CU::getSbtTuSplit( cu.sbtInfo ), cs );
    }
    else
    {
        THROW( "Implicit TU split not available!" );
    }

    do
    {
      transform_tree(cs, partitioner, cuCtx, cuTop, ispType, subTuCounter);
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    TransformUnit& tu = cs.addTU( CS::getArea( cs, area, partitioner.chType, partitioner.treeType ), partitioner.chType, &cu );
    unsigned numBlocks = getNumberValidTBlocks( *cs.pcv );
    tu.checkTuNoResidual( partitioner.currPartIdx() );

    for( unsigned compID = COMP_Y; compID < numBlocks; compID++ )
    {
      if( tu.blocks[compID].valid() )
      {
        tu.getCoeffs( ComponentID( compID ) ).fill( 0 );
      }
    }
    tu.depth = trDepth;
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.blocks[tu.chType].width, tu.blocks[tu.chType].height, cu.depth, partitioner.currTrDepth );

    transform_unit(tu, cuCtx, partitioner, subTuCounter);
  }
}
bool CABACReader::cbf_comp(CodingUnit& cu, const CompArea& area, unsigned depth, const bool prevCbf, const bool useISP)
{
  unsigned  ctxId = DeriveCtx::CtxQtCbf(area.compID, prevCbf, useISP && isLuma(area.compID));
  const CtxSet& ctxSet = Ctx::QtCbf[area.compID];

  unsigned  cbf = 0;
  if (cu.bdpcmM[toChannelType(area.compID)])
  {
    ctxId = (area.compID != COMP_Cr) ? 1 : 2;
    cbf = m_BinDecoder.decodeBin(ctxSet(ctxId));
  }
  else
  {
    cbf = m_BinDecoder.decodeBin( ctxSet( ctxId ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
  return cbf;
}

//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( cu, refList )
//================================================================================

void CABACReader::mvd_coding( Mv &rMvd )
{
  // abs_mvd_greater0_flag[ 0 | 1 ]
  int horAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());
  int verAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if (horAbs)
  {
    horAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }
  if (verAbs)
  {
    verAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if (horAbs)
  {
    if (horAbs > 1)
    {
      horAbs += m_BinDecoder.decodeRemAbsEP(1, 0, MV_BITS - 1);
    }
    if (m_BinDecoder.decodeBinEP())
    {
      horAbs = -horAbs;
    }
  }
  if (verAbs)
  {
    if (verAbs > 1)
    {
      verAbs += m_BinDecoder.decodeRemAbsEP(1, 0, MV_BITS - 1);
    }
    if (m_BinDecoder.decodeBinEP())
    {
      verAbs = -verAbs;
    }
  }
  rMvd = Mv(horAbs, verAbs);
  CHECK(!((horAbs >= MVD_MIN) && (horAbs <= MVD_MAX)) || !((verAbs >= MVD_MIN) && (verAbs <= MVD_MAX)), "Illegal MVD value");
  DTRACE( g_trace_ctx, D_SYNTAX, "mvd_coding() hor=%d ver=%d\n", rMvd.hor, rMvd.ver );
}


//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================
void CABACReader::transform_unit( TransformUnit& tu, CUCtx& cuCtx, Partitioner& partitioner, const int subTuCounter)
{
  const UnitArea&         area = partitioner.currArea();
  const unsigned          trDepth = partitioner.currTrDepth;
  CodingUnit&       cu = *tu.cu;
  ChromaCbfs        chromaCbfs;
  chromaCbfs.Cb = chromaCbfs.Cr = false;

  const bool chromaCbfISP = area.chromaFormat != CHROMA_400 && area.blocks[COMP_Cb].valid() && cu.ispMode;

  // cbf_cb & cbf_cr
  if (area.chromaFormat != CHROMA_400 && area.blocks[COMP_Cb].valid() && (!CU::isSepTree(cu) || partitioner.chType == CH_C) && (!cu.ispMode || chromaCbfISP))
  {
    const int cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;
    if (!(cu.sbtInfo && tu.noResidual))
    {
      chromaCbfs.Cb = cbf_comp(cu, area.blocks[COMP_Cb], cbfDepth);
    }

    if (!(cu.sbtInfo && tu.noResidual))
    {
      chromaCbfs.Cr = cbf_comp(cu, area.blocks[COMP_Cr], cbfDepth, chromaCbfs.Cb);
    }
  }
  else if (CU::isSepTree(cu))
  {
    chromaCbfs = ChromaCbfs(false);
  }

  if (!isChroma(partitioner.chType))
  {
    if (!CU::isIntra(cu) && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      TU::setCbfAtDepth(tu, COMP_Y, trDepth, 1);
    }
    else if (cu.sbtInfo && tu.noResidual)
    {
      TU::setCbfAtDepth(tu, COMP_Y, trDepth, 0);
    }
    else if (cu.sbtInfo && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      assert(!tu.noResidual);
      TU::setCbfAtDepth(tu, COMP_Y, trDepth, 1);
    }
    else
    {
      bool lumaCbfIsInferredACT = (cu.colorTransform && cu.predMode == MODE_INTRA && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat));
      bool lastCbfIsInferred    = lumaCbfIsInferredACT; // ISP and ACT are mutually exclusive
      bool previousCbf = false;
      bool rootCbfSoFar = false;
      if (cu.ispMode)
      {
        uint32_t nTus = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> Log2(tu.lheight()) : cu.lwidth() >> Log2(tu.lwidth());
        if (subTuCounter == nTus - 1)
        {
          TransformUnit* tuPointer = cu.firstTU;
          for (int tuIdx = 0; tuIdx < nTus - 1; tuIdx++)
          {
            rootCbfSoFar |= TU::getCbfAtDepth(*tuPointer, COMP_Y, trDepth);
            tuPointer = tuPointer->next;
          }
          if (!rootCbfSoFar)
          {
            lastCbfIsInferred = true;
          }
        }
        if (!lastCbfIsInferred)
        {
          previousCbf = TU::getPrevTuCbfAtDepth(tu, COMP_Y, trDepth);
        }
      }
      bool cbfY = lastCbfIsInferred ? true : cbf_comp(cu, tu.Y(), trDepth, previousCbf, cu.ispMode);
      TU::setCbfAtDepth(tu, COMP_Y, trDepth, (cbfY ? 1 : 0));
    }
  }
  if (area.chromaFormat != CHROMA_400 && (!cu.ispMode || chromaCbfISP))
  {
    TU::setCbfAtDepth(tu, COMP_Cb, trDepth, (chromaCbfs.Cb ? 1 : 0));
    TU::setCbfAtDepth(tu, COMP_Cr, trDepth, (chromaCbfs.Cr ? 1 : 0));
  }
  bool        lumaOnly   = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMP_Cb].valid() );
  bool        cbfLuma    = ( tu.cbf[ COMP_Y ] != 0 );
  bool        cbfChroma  = ( lumaOnly ? false : ( chromaCbfs.Cb || chromaCbfs.Cr ) );

  if( ( cu.lwidth() > 64 || cu.lheight() > 64 || cbfLuma || cbfChroma ) &&
    (!CU::isSepTree(*tu.cu) || isLuma(tu.chType)) )
  {
    if( cu.cs->pps->useDQP && !cuCtx.isDQPCoded )
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
  if (!CU::isSepTree(cu) || isChroma(tu.chType))   // !DUAL_TREE_LUMA
  {
    SizeType channelWidth = !CU::isSepTree(cu) ? cu.lwidth() : cu.chromaSize().width;
    SizeType channelHeight = !CU::isSepTree(cu) ? cu.lheight() : cu.chromaSize().height;

    if (cu.cs->slice->chromaQpAdjEnabled && (channelWidth > 64 || channelHeight > 64 || cbfChroma) && !cuCtx.isChromaQpAdjCoded)
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }

  if( !lumaOnly )
  {
    joint_cb_cr( tu, ( tu.cbf[COMP_Cb] ? 2 : 0 ) + ( tu.cbf[COMP_Cr] ? 1 : 0 ) );
  }

    if( cbfLuma )
    {
      residual_coding( tu, COMP_Y, cuCtx );
    }
    if( !lumaOnly )
    {
      for( ComponentID compID = COMP_Cb; compID <= COMP_Cr; compID = ComponentID( compID + 1 ) )
      {
        if( tu.cbf[ compID ] )
        {
          residual_coding( tu, compID, cuCtx );
      }
    }
  }
}

void CABACReader::cu_qp_delta( CodingUnit& cu, int predQP, int8_t& qp )
{
  CHECK( predQP == std::numeric_limits<int>::max(), "Invalid predicted QP" );
  int qpY = predQP;
  int DQp = unary_max_symbol( Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( DQp >= CU_DQP_TU_CMAX )
  {
    DQp += exp_golomb_eqprob( CU_DQP_EG_k  );
  }
  if( DQp > 0 )
  {
    if( m_BinDecoder.decodeBinEP( ) )
    {
      DQp = -DQp;
    }
    int     qpBdOffsetY = cu.cs->sps->qpBDOffset[ CH_L ];
    qpY = ( (predQP + DQp + (MAX_QP + 1) + 2 * qpBdOffsetY) % ((MAX_QP + 1) + qpBdOffsetY)) - qpBdOffsetY;
  }
  qp = (int8_t)qpY;

  DTRACE( g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACReader::cu_chroma_qp_offset( CodingUnit& cu )
{
  // cu_chroma_qp_offset_flag
  int       length  = cu.cs->pps->chromaQpOffsetListLen;
  unsigned  qpAdj   = m_BinDecoder.decodeBin( Ctx::ChromaQpAdjFlag() );
  if( qpAdj && length > 1 )
  {
    // cu_chroma_qp_offset_idx
    qpAdj += unary_max_symbol( Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
  }
  /* NB, symbol = 0 if outer flag is not set,
   *              1 if outer flag is set and there is no inner flag
   *              1+ otherwise */
  cu.chromaQpAdj = cu.cs->chromaQpAdj = qpAdj;
}

//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    bool        transform_skip_flag     ( tu, compID )
//    int         last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACReader::joint_cb_cr( TransformUnit& tu, const int cbfMask )
{
  if ( !tu.cu->slice->sps->jointCbCr )
  {
    return;
  }

  if( ( CU::isIntra( *tu.cu ) && cbfMask ) || ( cbfMask == 3 ) )
  {
    tu.jointCbCr = ( m_BinDecoder.decodeBin( Ctx::JointCbCrFlag( cbfMask-1 ) ) ? cbfMask : 0 );
  }
}

void CABACReader::residual_coding( TransformUnit& tu, ComponentID compID, CUCtx& cuCtx )
{
  const CodingUnit& cu = *tu.cu;
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );

  if( compID == COMP_Cr && tu.jointCbCr == 3 )
  {
    return;
  }

  // parse transform skip
  ts_flag            ( tu, compID );

  if ( tu.mtsIdx[compID] == MTS_SKIP)
  {
    residual_codingTS( tu, compID );
    return;
  }

  // determine sign hiding
  bool signHiding  = cu.cs->slice->signDataHidingEnabled;

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, signHiding );
  TCoeffSig*          coeff   = tu.getCoeffs( compID ).buf;

  // parse last coeff position
  cctx.setScanPosLast( last_sig_coeff( cctx, tu, compID ) );
  tu.lastPos[compID] = cctx.scanPosLast();
  if( tu.mtsIdx[compID] != MTS_SKIP && tu.blocks[ compID ].height >= 4 && tu.blocks[ compID ].width >= 4 )
  {
    const int maxLfnstPos = ((tu.blocks[compID].height == 4 && tu.blocks[compID].width == 4) || (tu.blocks[compID].height == 8 && tu.blocks[compID].width == 8)) ? 7 : 15;
    cuCtx.violatesLfnstConstrained[ toChannelType(compID) ] |= cctx.scanPosLast() > maxLfnstPos;
  }
  if( tu.mtsIdx[compID] != MTS_SKIP && tu.blocks[ compID ].height >= 4 && tu.blocks[ compID ].width >= 4 )
  {
    const int lfnstLastScanPosTh = isLuma( compID ) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_CHROMA;
    cuCtx.lfnstLastScanPos |= cctx.scanPosLast() >= lfnstLastScanPosTh;
  }
  if (isLuma(compID) && tu.mtsIdx[compID] != MTS_SKIP)
  {
    cuCtx.mtsLastScanPos |= cctx.scanPosLast() >= 1;
  }

  // parse subblocks
  const int stateTransTab = ( tu.cs->slice->depQuantEnabled ? 32040 : 0 );
  int       state         = 0;

  int ctxBinSampleRatio = MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT;
  cctx.regBinLimit = (tu.getTbAreaAfterCoefZeroOut(compID) * ctxBinSampleRatio) >> 4;

    for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
    {
      cctx.initSubblock       ( subSetId );
      if( tu.cs->sps->MTS && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32  && compID == COMP_Y )
      {
        if( ( tu.blocks[ compID ].height == 32 && cctx.cgPosY() >= ( 16 >> cctx.log2CGHeight() ) ) || ( tu.blocks[ compID ].width == 32 && cctx.cgPosX() >= ( 16 >> cctx.log2CGWidth() ) ) )
        {
          continue;
        }
      }
      residual_coding_subblock( cctx, coeff, stateTransTab, state );
      if ( isLuma(compID) && cctx.isSigGroup() && ( cctx.cgPosY() > 3 || cctx.cgPosX() > 3 ) )
      {
        cuCtx.violatesMtsCoeffConstraint = true;
      }
    }
}

void CABACReader::ts_flag( TransformUnit& tu, ComponentID compID )
{
  int tsFlag = (tu.cu->bdpcmM[toChannelType(compID)] || tu.mtsIdx[compID] == MTS_SKIP) ? 1 : 0;
  int ctxIdx = isLuma(compID) ? 0 : 1;

  if( TU::isTSAllowed ( tu, compID ) )
  {
    tsFlag = m_BinDecoder.decodeBin( Ctx::TransformSkipFlag( ctxIdx ) );
  }
  
  tu.mtsIdx[compID] = tsFlag ? MTS_SKIP : MTS_DCT2_DCT2;
  
  DTRACE(g_trace_ctx, D_SYNTAX, "ts_flag() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMP_Y, tu.cu->lx(), tu.cu->ly(), tsFlag);
}

void CABACReader::mts_idx( CodingUnit& cu, CUCtx& cuCtx )
{
  TransformUnit &tu = *cu.firstTU;
  int        mtsIdx = tu.mtsIdx[COMP_Y]; // Transform skip flag has already been decoded

  if( CU::isMTSAllowed( cu, COMP_Y ) && !cuCtx.violatesMtsCoeffConstraint &&
      cuCtx.mtsLastScanPos && cu.lfnstIdx == 0 && mtsIdx != MTS_SKIP)
  {
    int ctxIdx = 0;
    int symbol = m_BinDecoder.decodeBin( Ctx::MTSIdx( ctxIdx ) );

    if( symbol )
    {
      ctxIdx = 1;
      mtsIdx = MTS_DST7_DST7; // mtsIdx = 2 -- 4
      for( int i = 0; i < 3; i++, ctxIdx++ )
      {
        symbol  = m_BinDecoder.decodeBin( Ctx::MTSIdx( ctxIdx ) );
        mtsIdx += symbol;

        if( !symbol )
        {
          break;
        }
      }
    }
  }

  tu.mtsIdx[COMP_Y] = mtsIdx;

  DTRACE(g_trace_ctx, D_SYNTAX, "mts_idx() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMP_Y, tu.cu->lx(), tu.cu->ly(), mtsIdx);
}

void CABACReader::isp_mode( CodingUnit& cu )
{
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.multiRefIdx || !cu.cs->sps->ISP || cu.bdpcmM[CH_L] || !CU::canUseISP( cu, getFirstComponentOfChannel( cu.chType ) ) || cu.colorTransform )
  {
    cu.ispMode = NOT_INTRA_SUBPARTITIONS;
    return;
  }

  int symbol = m_BinDecoder.decodeBin(Ctx::ISPMode(0));

  if( symbol )
  {
    cu.ispMode = 1 + m_BinDecoder.decodeBin( Ctx::ISPMode( 1 ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_subPartitions() etype=%d pos=(%d,%d) ispIdx=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, (int)cu.ispMode );
}

void CABACReader::residual_lfnst_mode( CodingUnit& cu,  CUCtx& cuCtx  )
{
  int chIdx = CS::isDualITree( *cu.cs ) && cu.chType == CH_C ? 1 : 0;
  if ( (cu.ispMode && !CU::canUseLfnstWithISP( cu, cu.chType ) ) ||
      (cu.cs->sps->LFNST && CU::isIntra(cu) && cu.mipFlag && !allowLfnstWithMip(cu.lumaSize())) ||
    ( CU::isSepTree(cu) && cu.chType == CH_C && std::min( cu.blocks[ 1 ].width, cu.blocks[ 1 ].height ) < 4 )
    || ( cu.blocks[ chIdx ].lumaSize().width > cu.cs->sps->getMaxTbSize() || cu.blocks[ chIdx ].lumaSize().height > cu.cs->sps->getMaxTbSize() )
    )
  {
    return;
  }

  if( cu.cs->sps->LFNST && CU::isIntra( cu ) )
  {
    const bool lumaFlag              = CU::isSepTree(cu) ? (   isLuma( cu.chType ) ? true : false ) : true;
    const bool chromaFlag            = CU::isSepTree(cu) ? ( isChroma( cu.chType ) ? true : false ) : true;
    bool nonZeroCoeffNonTsCorner8x8 = ( lumaFlag && cuCtx.violatesLfnstConstrained[CH_L] ) || (chromaFlag && cuCtx.violatesLfnstConstrained[CH_C] );
    bool isTrSkip = false;
    for (auto &currTU : CU::traverseTUs(cu))
    {
      const uint32_t numValidComp = getNumberValidComponents(cu.chromaFormat);
      for (uint32_t compID = COMP_Y; compID < numValidComp; compID++)
      {
        if (currTU.blocks[compID].valid() && TU::getCbf(currTU, (ComponentID)compID) && currTU.mtsIdx[compID] == MTS_SKIP)
        {
          isTrSkip = true;
          break;
        }
      }
    }
    if ((!cuCtx.lfnstLastScanPos && !cu.ispMode) || nonZeroCoeffNonTsCorner8x8 || isTrSkip)
    {
      cu.lfnstIdx = 0;
      return;
    }
  }
  else
  {
    cu.lfnstIdx = 0;
    return;
  }

  unsigned cctx = 0;
  if ( CU::isSepTree(cu) ) cctx++;

  uint32_t idxLFNST = m_BinDecoder.decodeBin( Ctx::LFNSTIdx( cctx ) );
  if( idxLFNST )
  {
    idxLFNST += m_BinDecoder.decodeBin(Ctx::LFNSTIdx(2));
  }
  cu.lfnstIdx = idxLFNST;

  DTRACE( g_trace_ctx, D_SYNTAX, "residual_lfnst_mode() etype=%d pos=(%d,%d) mode=%d\n", COMP_Y, cu.lx(), cu.ly(), ( int ) cu.lfnstIdx );
}

int CABACReader::last_sig_coeff( CoeffCodingContext& cctx, TransformUnit& tu, ComponentID compID )
{
  unsigned PosLastX = 0, PosLastY = 0;
  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();

  if( ( tu.mtsIdx[compID] > MTS_SKIP || ( tu.cs->sps->MTS && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 ) )  && compID == COMP_Y )
  {
    maxLastPosX = ( tu.blocks[ compID ].width  == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosX;
    maxLastPosY = ( tu.blocks[ compID ].height == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosY;
  }

  for( ; PosLastX < maxLastPosX; PosLastX++ )
  {
    if( !m_BinDecoder.decodeBin( cctx.lastXCtxId( PosLastX ) ) )
    {
      break;
    }
  }
  for( ; PosLastY < maxLastPosY; PosLastY++ )
  {
    if( !m_BinDecoder.decodeBin( cctx.lastYCtxId( PosLastY ) ) )
    {
      break;
    }
  }
  if( PosLastX > 3 )
  {
    uint32_t uiTemp  = 0;
    uint32_t uiCount = ( PosLastX - 2 ) >> 1;
    for ( int i = uiCount - 1; i >= 0; i-- )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
    }
    PosLastX = g_uiMinInGroup[ PosLastX ] + uiTemp;
  }
  if( PosLastY > 3 )
  {
    uint32_t uiTemp  = 0;
    uint32_t uiCount = ( PosLastY - 2 ) >> 1;
    for ( int i = uiCount - 1; i >= 0; i-- )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
    }
    PosLastY = g_uiMinInGroup[ PosLastY ] + uiTemp;
  }

  int blkPos = PosLastX + ( PosLastY * cctx.width() );

  int scanPos = 0;
  for( ; scanPos < cctx.maxNumCoeff() - 1; scanPos++ )
  {
    if( blkPos == cctx.blockPos( scanPos ) )
    {
      break;
    }
  }
  return scanPos;
}



void CABACReader::residual_coding_subblock( CoeffCodingContext& cctx, TCoeffSig* coeff, const int stateTransTable, int& state )
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function

  //===== init =====
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
  int         firstSigPos = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );
  int         nextSigPos  = firstSigPos;

  //===== decode significant_coeffgroup_flag =====
  bool sigGroup = ( isLast || !minSubPos );
  if( !sigGroup )
  {
    sigGroup = m_BinDecoder.decodeBin( cctx.sigGroupCtxId() );
  }
  if( sigGroup )
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  uint8_t   ctxOffset[16];

  //===== decode absolute values =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
  int       firstNZPos    = nextSigPos;
  int       lastNZPos     = -1;
  int       numNonZero    =  0;
  int       remRegBins    = cctx.regBinLimit;
  int       firstPosMode2 = minSubPos - 1;
  int       sigBlkPos[ 1 << MLS_CG_SIZE ];

  for( ; nextSigPos >= minSubPos && remRegBins >= 4; nextSigPos-- )
  {
    int      blkPos     = cctx.blockPos( nextSigPos );
    unsigned sigFlag    = ( !numNonZero && nextSigPos == inferSigPos );
    if( !sigFlag )
    {
      const unsigned sigCtxId = cctx.sigCtxIdAbs( nextSigPos, coeff, state );
      sigFlag = m_BinDecoder.decodeBin( sigCtxId );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      remRegBins--;
    }
    else if( nextSigPos != cctx.scanPosLast() )
    {
      cctx.sigCtxIdAbs( nextSigPos, coeff, state ); // required for setting variables that are needed for gtx/par context selection
    }

    if( sigFlag )
    {
      uint8_t&  ctxOff = ctxOffset[ nextSigPos - minSubPos ];
      ctxOff           = cctx.ctxOffsetAbs();
      sigBlkPos[ numNonZero++ ] = blkPos;
      firstNZPos = nextSigPos;
      lastNZPos  = std::max<int>( lastNZPos, nextSigPos );

      unsigned gt1Flag = m_BinDecoder.decodeBin( cctx.greater1CtxIdAbs(ctxOff) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", gt1Flag, cctx.greater1CtxIdAbs(ctxOff) );
      remRegBins--;

      unsigned parFlag = 0;
      unsigned gt2Flag = 0;
      if( gt1Flag )
      {
        parFlag = m_BinDecoder.decodeBin( cctx.parityCtxIdAbs( ctxOff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbs( ctxOff ) );

        remRegBins--;
        gt2Flag = m_BinDecoder.decodeBin( cctx.greater2CtxIdAbs( ctxOff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", gt2Flag, cctx.greater2CtxIdAbs( ctxOff ) );
        remRegBins--;
      }
      coeff[ blkPos ] += 1 + parFlag + gt1Flag + (gt2Flag << 1);
    }

    state = ( stateTransTable >> ((state<<2)+((coeff[blkPos]&1)<<1)) ) & 3;
  }
  firstPosMode2 = nextSigPos;
  cctx.regBinLimit = remRegBins;


  //===== 2nd PASS: Go-rice codes =====
  unsigned ricePar = 0;
  for( int scanPos = firstSigPos; scanPos > firstPosMode2; scanPos-- )
  {
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 4);
    ricePar = g_auiGoRiceParsCoeff[sumAll];
    TCoeffSig& tcoeff = coeff[ cctx.blockPos( scanPos ) ];
    if( tcoeff >= 4 )
    {
      int       rem     = m_BinDecoder.decodeRemAbsEP( ricePar, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar );
      tcoeff += (rem<<1);
    }
  }

  //===== coeff bypass ====
  for( int scanPos = firstPosMode2; scanPos >= minSubPos; scanPos-- )
  {
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 0);
    int       rice      = g_auiGoRiceParsCoeff                        [sumAll];
    int       pos0      = g_auiGoRicePosCoeff0(state, rice);
    int       rem       = m_BinDecoder.decodeRemAbsEP( rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice );
    TCoeff    tcoeff  = ( rem == pos0 ? 0 : rem < pos0 ? rem+1 : rem );
    state = ( stateTransTable >> ((state<<2)+((tcoeff&1)<<1)) ) & 3;
    if( tcoeff )
    {
      int        blkPos         = cctx.blockPos( scanPos );
      sigBlkPos[ numNonZero++ ] = blkPos;
      firstNZPos = scanPos;
      lastNZPos  = std::max<int>( lastNZPos, scanPos );
      coeff[blkPos] = tcoeff;
    }
  }

  //===== decode sign's =====
  const unsigned  numSigns    = ( cctx.hideSign( firstNZPos, lastNZPos ) ? numNonZero - 1 : numNonZero );
  unsigned        signPattern = m_BinDecoder.decodeBinsEP( numSigns ) << ( 32 - numSigns );

  //===== set final coefficents =====
  int sumAbs = 0;
  for( unsigned k = 0; k < numSigns; k++ )
  {
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
    sumAbs               += AbsCoeff;
    coeff[ sigBlkPos[k] ] = ( signPattern & ( 1u << 31 ) ? -AbsCoeff : AbsCoeff );
    signPattern         <<= 1;
  }
  if( numNonZero > numSigns )
  {
    int k                 = numSigns;
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
    sumAbs               += AbsCoeff;
    coeff[ sigBlkPos[k] ] = ( sumAbs & 1 ? -AbsCoeff : AbsCoeff );
  }
}

void CABACReader::residual_codingTS( TransformUnit& tu, ComponentID compID )
{
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_codingTS() etype=%d pos=(%d,%d) size=%dx%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height );

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, false, tu.cu->bdpcmM[toChannelType(compID)]);
  TCoeffSig*          coeff   = tu.getCoeffs( compID ).buf;

  int maxCtxBins = (cctx.maxNumCoeff() * 7) >> 2;
  cctx.setNumCtxBins(maxCtxBins);

  for( int subSetId = 0; subSetId <= ( cctx.maxNumCoeff() - 1 ) >> cctx.log2CGSize(); subSetId++ )
  {
    cctx.initSubblock         ( subSetId );
    residual_coding_subblockTS( cctx, coeff );
  }
}

void CABACReader::residual_coding_subblockTS( CoeffCodingContext& cctx, TCoeffSig* coeff )
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function

  //===== init =====
  const int   minSubPos   = cctx.maxSubPos();
  int         firstSigPos = cctx.minSubPos();
  int         nextSigPos  = firstSigPos;
  unsigned    signPattern = 0;

  //===== decode significant_coeffgroup_flag =====
  bool sigGroup = cctx.isLastSubSet() && cctx.noneSigGroup();
  if( !sigGroup )
  {
      sigGroup = m_BinDecoder.decodeBin( cctx.sigGroupCtxId( true ) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", sigGroup, cctx.sigGroupCtxId() );
  }
  if( sigGroup )
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  //===== decode absolute values =====
  const int inferSigPos   = minSubPos;
  int       numNonZero    =  0;
  int       sigBlkPos[ 1 << MLS_CG_SIZE ];

  int lastScanPosPass1 = -1;
  int lastScanPosPass2 = -1;
  for (; nextSigPos <= minSubPos && cctx.numCtxBins() >= 4; nextSigPos++)
  {
    int      blkPos     = cctx.blockPos( nextSigPos );
    unsigned sigFlag    = ( !numNonZero && nextSigPos == inferSigPos );
    if( !sigFlag )
    {
      const unsigned sigCtxId = cctx.sigCtxIdAbsTS( nextSigPos, coeff );
      sigFlag = m_BinDecoder.decodeBin( sigCtxId );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      cctx.decimateNumCtxBins(1);
    }

    if( sigFlag )
    {
      //===== decode sign's =====
      int sign;
      const unsigned signCtxId = cctx.signCtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      sign = m_BinDecoder.decodeBin(signCtxId);
      cctx.decimateNumCtxBins(1);

      signPattern += ( sign << numNonZero );

      sigBlkPos[numNonZero++] = blkPos;

      unsigned gt1Flag;
      const unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      gt1Flag = m_BinDecoder.decodeBin(gt1CtxId);
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1Flag, gt1CtxId );
      cctx.decimateNumCtxBins(1);

      unsigned parFlag = 0;
      if( gt1Flag )
      {
        parFlag = m_BinDecoder.decodeBin( cctx.parityCtxIdAbsTS() );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbsTS() );
        cctx.decimateNumCtxBins(1);
      }
      coeff[ blkPos ] = (sign ? -1 : 1 ) * (1 + parFlag + gt1Flag);
    }
    lastScanPosPass1 = nextSigPos;
  }

  int cutoffVal = 2;
  const int numGtBins = 4;

  //===== 2nd PASS: gt2 =====
  for (int scanPos = firstSigPos; scanPos <= minSubPos && cctx.numCtxBins() >= 4; scanPos++)
  {
    TCoeffSig& tcoeff = coeff[cctx.blockPos(scanPos)];
    cutoffVal = 2;
    for (int i = 0; i < numGtBins; i++)
    {
      if( tcoeff < 0)
      {
        tcoeff = -tcoeff;
      }
       if (tcoeff >= cutoffVal)
       {
          unsigned gt2Flag;
          gt2Flag = m_BinDecoder.decodeBin(cctx.greaterXCtxIdAbsTS(cutoffVal >> 1));
          tcoeff += (gt2Flag << 1);
          DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i, gt2Flag, cctx.greaterXCtxIdAbsTS(cutoffVal >> 1), scanPos, tcoeff);
          cctx.decimateNumCtxBins(1);
       }
       cutoffVal += 2;
    }
    lastScanPosPass2 = scanPos;
  }
  //===== 3rd PASS: Go-rice codes =====
  for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
  {
    TCoeffSig& tcoeff = coeff[ cctx.blockPos( scanPos ) ];
    cutoffVal = (scanPos <= lastScanPosPass2 ? 10 : (scanPos <= lastScanPosPass1 ? 2 : 0));
    if (tcoeff < 0)
    {
      tcoeff = -tcoeff;
    }
    if( tcoeff >= cutoffVal )
    {
      int       rice = cctx.templateAbsSumTS( scanPos, coeff );
      int       rem  = m_BinDecoder.decodeRemAbsEP( rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_rem_val() bin=%d ctx=%d sp=%d\n", rem, rice, scanPos );
      tcoeff += (scanPos <= lastScanPosPass1) ? (rem << 1) : rem;
      if (tcoeff && scanPos > lastScanPosPass1)
      {
        int      blkPos = cctx.blockPos(scanPos);
        int sign = m_BinDecoder.decodeBinEP();
        signPattern += (sign << numNonZero);
        sigBlkPos[numNonZero++] = blkPos;
      }
    }
    if (!cctx.bdpcm() && cutoffVal)
    {
      if (tcoeff > 0)
      {
        int rightPixel, belowPixel;
        cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
        tcoeff = cctx.decDeriveModCoeff(rightPixel, belowPixel, tcoeff);
      }
    }
  }

  //===== set final coefficents =====
  for( unsigned k = 0; k < numNonZero; k++ )
  {
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
    coeff[ sigBlkPos[k] ] = ( signPattern & 1 ? -AbsCoeff : AbsCoeff );
    signPattern         >>= 1;
  }
}



//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    unsigned  unary_max_symbol ( ctxId0, ctxId1, maxSymbol )
//    unsigned  unary_max_eqprob (                 maxSymbol )
//    unsigned  exp_golomb_eqprob( count )
//================================================================================

unsigned CABACReader::unary_max_symbol( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol  )
{
  unsigned onesRead = 0;
  while( onesRead < maxSymbol && m_BinDecoder.decodeBin( onesRead == 0 ? ctxId0 : ctxIdN ) == 1 )
  {
    ++onesRead;
  }
  return onesRead;
}


unsigned CABACReader::unary_max_eqprob( unsigned maxSymbol )
{
  for( unsigned k = 0; k < maxSymbol; k++ )
  {
    if( !m_BinDecoder.decodeBinEP() )
    {
      return k;
    }
  }
  return maxSymbol;
}


unsigned CABACReader::exp_golomb_eqprob( unsigned count )
{
  unsigned symbol = 0;
  unsigned bit    = 1;
  while( bit )
  {
    bit     = m_BinDecoder.decodeBinEP( );
    symbol += bit << count++;
  }
  if( --count )
  {
    symbol += m_BinDecoder.decodeBinsEP( count );
  }
  return symbol;
}

void CABACReader::mip_flag( CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }
  if( !cu.cs->sps->MIP )
  {
    cu.mipFlag = false;
    return;
  }

  unsigned ctxId = DeriveCtx::CtxMipFlag( cu );
  cu.mipFlag = m_BinDecoder.decodeBin( Ctx::MipFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.mipFlag ? 1 : 0 );
}

void CABACReader::mip_pred_modes( CodingUnit &cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }

  mip_pred_mode( cu );
}

void CABACReader::mip_pred_mode( CodingUnit& cu )
{
  cu.mipTransposedFlag = bool(m_BinDecoder.decodeBinEP());

  uint32_t mipMode;
  const int numModes = getNumModesMip( cu.Y() );
  xReadTruncBinCode( mipMode, numModes );
  cu.intraDir[CH_L] = mipMode;
  CHECKD( cu.intraDir[CH_L] < 0 || cu.intraDir[CH_L] >= numModes, "Invalid MIP mode" );

  DTRACE( g_trace_ctx, D_SYNTAX, "mip_pred_mode() pos=(%d,%d) mode=%d transposed=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.intraDir[CH_L], cu.mipTransposedFlag ? 1 : 0 );
}

} // namespace vvenc

//! \}


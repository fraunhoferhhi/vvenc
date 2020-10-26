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


/** \file     CABACWriter.cpp
 *  \brief    Writer for low level syntax
 */

#include "CABACWriter.h"
#include "EncLib.h"
#include "CommonLib/Contexts.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_buffer.h"

#include <map>
#include <algorithm>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

void CABACWriter::initCtxModels( const Slice& slice )
{
  int       qp                = slice.sliceQp;
  SliceType sliceType         = slice.sliceType;
  SliceType encCABACTableIdx  = slice.encCABACTableIdx;
  if( !slice.isIntra() && (encCABACTableIdx==B_SLICE || encCABACTableIdx==P_SLICE) && slice.pps->cabacInitPresent )
  {
    sliceType = encCABACTableIdx;
  }
  m_BinEncoder.reset( qp, (int)sliceType );
}



SliceType xGetCtxInitId( const Slice& slice, const BinEncIf& binEncoder, Ctx& ctxTest )
{
  const CtxStore& ctxStoreTest = static_cast<const CtxStore&>( ctxTest );
  const CtxStore& ctxStoreRef  = static_cast<const CtxStore&>( binEncoder.getCtx() );
  int qp = slice.sliceQp;
  if( !slice.isIntra() )
  {
    SliceType aSliceTypeChoices[] = { B_SLICE, P_SLICE };
    uint64_t  bestCost            = std::numeric_limits<uint64_t>::max();
    SliceType bestSliceType       = aSliceTypeChoices[0];
    for (uint32_t idx=0; idx<2; idx++)
    {
      uint64_t  curCost           = 0;
      SliceType curSliceType      = aSliceTypeChoices[idx];
      ctxTest.init( qp, (int)curSliceType );
      for( int k = 0; k < Ctx::NumberOfContexts; k++ )
      {
        if( binEncoder.getNumBins(k) > 0 )
        {
          curCost += uint64_t( binEncoder.getNumBins(k) ) * ctxStoreRef[k].estFracExcessBits( ctxStoreTest[k] );
        }
      }
      if (curCost < bestCost)
      {
        bestSliceType = curSliceType;
        bestCost      = curCost;
      }
    }
    return bestSliceType;
  }
  else
  {
    return I_SLICE;
  }
}


SliceType CABACWriter::getCtxInitId( const Slice& slice )
{
  return  xGetCtxInitId( slice, m_BinEncoder, m_TestCtx );
}


unsigned estBits( BinEncIf& binEnc, const std::vector<bool>& bins, const Ctx& ctx, const int ctxId, const uint8_t winSize )
{
  binEnc.initCtxAndWinSize( ctxId, ctx, winSize );
  binEnc.start();
  const std::size_t numBins   = bins.size();
  unsigned          startBits = binEnc.getNumWrittenBits();
  for( std::size_t binId = 0; binId < numBins; binId++ )
  {
    unsigned  bin = ( bins[binId] ? 1 : 0 );
    binEnc.encodeBin( bin, ctxId );
  }
  unsigned endBits    = binEnc.getNumWrittenBits();
  unsigned codedBits  = endBits - startBits;
  return   codedBits;
}


//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    void  end_of_slice()
//================================================================================

void CABACWriter::end_of_slice()
{
  m_BinEncoder.encodeBinTrm ( 1 );
  m_BinEncoder.finish       ();
}


//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit( cs, area, qp, ctuRsAddr, skipSao, skipAlf )
//================================================================================

void CABACWriter::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr, bool skipSao /* = false */, bool skipAlf /* = false */ )
{
  CUCtx cuCtx( qps[CH_L] );
  Partitioner *partitioner = PartitionerFactory::get( *cs.slice );

  partitioner->initCtu( area, CH_L, *cs.slice );

  if( !skipSao )
  {
    sao( *cs.slice, ctuRsAddr );
  }

  if (!skipAlf)
  {
    for (int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
    {
      codeAlfCtuEnabledFlag(cs, ctuRsAddr, compIdx, NULL);
      if (isLuma(ComponentID(compIdx)))
      {
        codeAlfCtuFilterIndex(cs, ctuRsAddr, cs.slice->tileGroupAlfEnabled[COMP_Y]);
      }
      if (isChroma(ComponentID(compIdx)))
      {
        uint8_t* ctbAlfFlag = cs.slice->tileGroupAlfEnabled[compIdx] ? cs.slice->pic->getAlfCtuEnabled( compIdx ) : nullptr;
        if( ctbAlfFlag && ctbAlfFlag[ctuRsAddr] )
        {
          codeAlfCtuAlternative( cs, ctuRsAddr, compIdx );
        }
      }
    }
  }

  if ( !skipAlf )
  {
    for ( int compIdx = 1; compIdx < getNumberValidComponents( cs.pcv->chrFormat ); compIdx++ )
    {
      if (cs.slice->ccAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
      {
        const int filterCount   = cs.slice->ccAlfFilterParam.ccAlfFilterCount[compIdx - 1];

        const int      ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int      rx = ctuRsAddr % cs.pcv->widthInCtus;
        const Position lumaPos(rx * cs.pcv->maxCUSize, ry * cs.pcv->maxCUSize);

        codeCcAlfFilterControlIdc(cs.slice->ccAlfFilterControl[compIdx - 1][ctuRsAddr], cs, ComponentID(compIdx),
                                  ctuRsAddr, cs.slice->ccAlfFilterControl[compIdx - 1], lumaPos, filterCount);
      }
    }
  }

  if ( CS::isDualITree(cs) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUSize > 64 )
  {
    CUCtx chromaCuCtx(qps[CH_C]);
    Partitioner *chromaPartitioner = PartitionerFactory::get(*cs.slice);
    chromaPartitioner->initCtu(area, CH_C, *cs.slice);
    coding_tree(cs, *partitioner, cuCtx, chromaPartitioner, &chromaCuCtx);
    qps[CH_L] = cuCtx.qp;
    qps[CH_C] = chromaCuCtx.qp;

    delete chromaPartitioner;
  }
  else
  {
    coding_tree( cs, *partitioner, cuCtx );
    qps[CH_L] = cuCtx.qp;
    if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
    {
      CUCtx cuCtxChroma( qps[CH_C] );
      partitioner->initCtu( area, CH_C, *cs.slice );
      coding_tree( cs, *partitioner, cuCtxChroma );
      qps[CH_C] = cuCtxChroma.qp;
    }
  }

  delete partitioner;
}


//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao             ( slice, ctuRsAddr )
//    void  sao_block_pars  ( saoPars, bitDepths, sliceEnabled, leftMergeAvail, aboveMergeAvail, onlyEstMergeInfo )
//    void  sao_offset_pars ( ctbPars, compID, sliceEnabled, bitDepth )
//================================================================================

void CABACWriter::sao( const Slice& slice, unsigned ctuRsAddr )
{
  const SPS& sps = *slice.sps;
  if( !sps.saoEnabled )
  {
    return;
  }

  CodingStructure&     cs                     = *slice.pic->cs;
  const PreCalcValues& pcv                    = *cs.pcv;
  const SAOBlkParam&  sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
  bool                slice_sao_luma_flag     = ( slice.saoEnabled[ CH_L ] );
  bool                slice_sao_chroma_flag   = ( slice.saoEnabled[ CH_C ] && sps.chromaFormatIdc != CHROMA_400 );
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  bool                sliceEnabled[3]         = { slice_sao_luma_flag, slice_sao_chroma_flag, slice_sao_chroma_flag };
  int                 frame_width_in_ctus     = pcv.widthInCtus;
  int                 ry                      = ctuRsAddr      / frame_width_in_ctus;
  int                 rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  const Position      pos                     ( rx * cs.pcv->maxCUSize, ry * cs.pcv->maxCUSize );
  const unsigned      curSliceIdx             = slice.independentSliceIdx;
  const unsigned      curTileIdx              = 0;//cs.picture->brickMap->getBrickIdxRsMap( pos );
  bool                leftMergeAvail          = cs.getCURestricted( pos.offset( -(int)pcv.maxCUSize, 0  ), pos, curSliceIdx, curTileIdx, CH_L, TREE_D ) ? true : false;
  bool                aboveMergeAvail         = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUSize ), pos, curSliceIdx, curTileIdx, CH_L, TREE_D ) ? true : false;
  sao_block_pars( sao_ctu_pars, sps.bitDepths, sliceEnabled, leftMergeAvail, aboveMergeAvail, false );
}


void CABACWriter::sao_block_pars( const SAOBlkParam& saoPars, const BitDepths& bitDepths, const bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo )
{
  bool isLeftMerge  = false;
  bool isAboveMerge = false;
  if( leftMergeAvail )
  {
    // sao_merge_left_flag
    isLeftMerge   = ( saoPars[COMP_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMP_Y].typeIdc == SAO_MERGE_LEFT );
    m_BinEncoder.encodeBin( (isLeftMerge), Ctx::SaoMergeFlag() );
  }
  if( aboveMergeAvail && !isLeftMerge )
  {
    // sao_merge_above_flag
    isAboveMerge  = ( saoPars[COMP_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMP_Y].typeIdc == SAO_MERGE_ABOVE );
    m_BinEncoder.encodeBin( (isAboveMerge), Ctx::SaoMergeFlag() );
  }
  if( onlyEstMergeInfo )
  {
    return; //only for RDO
  }
  if( !isLeftMerge && !isAboveMerge )
  {
    // explicit parameters
    for( int compIdx=0; compIdx < MAX_NUM_COMP; compIdx++ )
    {
      sao_offset_pars( saoPars[compIdx], ComponentID(compIdx), sliceEnabled[compIdx], bitDepths[ toChannelType(ComponentID(compIdx)) ] );
    }
  }
}


void CABACWriter::sao_offset_pars( const SAOOffset& ctbPars, ComponentID compID, bool sliceEnabled, int bitDepth )
{
  if( !sliceEnabled )
  {
    CHECK( ctbPars.modeIdc != SAO_MODE_OFF, "Sao must be off, if it is disabled on slice level" );
    return;
  }
  const bool isFirstCompOfChType = ( getFirstComponentOfChannel( toChannelType(compID) ) == compID );

  if( isFirstCompOfChType )
  {
    // sao_type_idx_luma / sao_type_idx_chroma
    if( ctbPars.modeIdc == SAO_MODE_OFF )
    {
      m_BinEncoder.encodeBin  ( 0, Ctx::SaoTypeIdx() );
    }
    else if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 0 );
    }
    else
    {
      CHECK(!( ctbPars.typeIdc < SAO_TYPE_START_BO ), "Unspecified error");
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 1 );
    }
  }

  if( ctbPars.modeIdc == SAO_MODE_NEW )
  {
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( bitDepth );
    int       numClasses    = ( ctbPars.typeIdc == SAO_TYPE_BO ? 4 : NUM_SAO_EO_CLASSES );
    int       k             = 0;
    int       offset[4];
    for( int i = 0; i < numClasses; i++ )
    {
      if( ctbPars.typeIdc != SAO_TYPE_BO && i == SAO_CLASS_EO_PLAIN )
      {
        continue;
      }
      int classIdx = ( ctbPars.typeIdc == SAO_TYPE_BO ? ( ctbPars.typeAuxInfo + i ) % NUM_SAO_BO_CLASSES : i );
      offset[k++]  = ctbPars.offset[classIdx];
    }

    // sao_offset_abs
    for( int i = 0; i < 4; i++ )
    {
      unsigned absOffset = ( offset[i] < 0 ? -offset[i] : offset[i] );
      unary_max_eqprob( absOffset, maxOffsetQVal );
    }

    // band offset mode
    if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      // sao_offset_sign
      for( int i = 0; i < 4; i++ )
      {
        if( offset[i] )
        {
          m_BinEncoder.encodeBinEP( (offset[i] < 0) );
        }
      }
      // sao_band_position
      m_BinEncoder.encodeBinsEP( ctbPars.typeAuxInfo, NUM_SAO_BO_CLASSES_LOG2 );
    }
    // edge offset mode
    else
    {
      if( isFirstCompOfChType )
      {
        // sao_eo_class_luma / sao_eo_class_chroma
        CHECK( ctbPars.typeIdc - SAO_TYPE_START_EO < 0, "sao edge offset class is outside valid range" );
        m_BinEncoder.encodeBinsEP( ctbPars.typeIdc - SAO_TYPE_START_EO, NUM_SAO_EO_TYPES_LOG2 );
      }
    }
  }
}


//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    void  coding_tree       ( cs, partitioner, cuCtx )
//    void  split_cu_flag     ( split, cs, partitioner )
//    void  split_cu_mode_mt  ( split, cs, partitioner )
//================================================================================

void CABACWriter::coding_tree(const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, Partitioner* pPartitionerChroma, CUCtx* pCuCtxChroma)
{
  const PPS      &pps         = *cs.pps;
  const UnitArea& currArea    = partitioner.currArea();
  const CodingUnit &cu        = *cs.getCU( currArea.blocks[partitioner.chType], partitioner.chType, partitioner.treeType );

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  //Note: do not reset qg at chroma CU
  if( pps.useDQP && partitioner.currQgEnable() && !isChroma( partitioner.chType ) )
  {
    cuCtx.qgStart    = true;
    cuCtx.isDQPCoded          = false;
  }
  if( cs.slice->chromaQpAdjEnabled && partitioner.currQgChromaEnable() )
  {
    cuCtx.isChromaQpAdjCoded  = false;
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
    }
  }

  determineNeighborCus( cs, partitioner.currArea(), partitioner.chType, partitioner.treeType );

  const PartSplit splitMode = CU::getSplitAtDepth( cu, partitioner.currDepth );

  split_cu_mode( splitMode, cs, partitioner );

  CHECK( !partitioner.canSplit( splitMode, cs ), "The chosen split mode is invalid!" );

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
          if (cs.picture->blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
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
          if (cs.picture->blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
          {
            coding_tree(cs, partitioner, cuCtx);
          }
          lumaContinue = partitioner.nextPart(cs);
          if (cs.picture->blocks[pPartitionerChroma->chType].contains(pPartitionerChroma->currArea().blocks[pPartitionerChroma->chType].pos()))
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

    }
    else
    {
      const ModeType modeTypeParent = partitioner.modeType;
      const ModeType modeTypeChild = CU::getModeTypeAtDepth( cu, partitioner.currDepth );
      mode_constraint( splitMode, cs, partitioner, modeTypeChild );
      partitioner.modeType = modeTypeChild;

      bool chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && modeTypeChild == MODE_TYPE_INTRA ? true : false;
      CHECK( chromaNotSplit && partitioner.chType != CH_L, "chType must be luma" );
      if( partitioner.treeType == TREE_D )
      {
        partitioner.treeType = chromaNotSplit ? TREE_L : TREE_D;
      }
    partitioner.splitCurrArea( splitMode, cs );

    do
    {
      if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
      {
        coding_tree( cs, partitioner, cuCtx );
      }
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
    if( chromaNotSplit )
    {
      if (isChromaEnabled(cs.pcv->chrFormat))
      {
        CHECK( partitioner.chType != CH_L, "must be luma status" );
        partitioner.chType = CH_C;
        partitioner.treeType = TREE_C;

        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }
      }
      //recover
      partitioner.chType = CH_L;
      partitioner.treeType = TREE_D;
    }
    partitioner.modeType = modeTypeParent;
    }
    return;
  }

  // Predict QP on start of quantization group
  if( cuCtx.qgStart )
  {
    cuCtx.qgStart = false;
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }
  CHECK( cu.treeType != partitioner.treeType, "treeType mismatch" );


  // coding unit
  coding_unit( cu, partitioner, cuCtx );

  if( cu.chType == CH_C )
  {
    DTRACE_COND( (isEncoding()), g_trace_ctx, D_QP, "[chroma CU]x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Cb().x, cu.Cb().y, cu.Cb().width, cu.Cb().height, cu.qp );
  }
  else
  {
  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, cu.qp );
  }
  DTRACE_BLOCK_REC_COND( ( !isEncoding() ), cs.picture->getRecoBuf( cu ), cu, cu.predMode );
}


void CABACWriter::mode_constraint( const PartSplit split, const CodingStructure& cs, Partitioner& partitioner, const ModeType modeType )
{
  CHECK( split == CU_DONT_SPLIT, "splitMode shall not be no split" );
  int val = cs.signalModeCons( split, partitioner, partitioner.modeType );
  if( val == LDT_MODE_TYPE_SIGNAL )
  {
    CHECK( modeType == MODE_TYPE_ALL, "shall not be no constraint case" );
    bool flag = modeType == MODE_TYPE_INTRA;
    int ctxIdx = DeriveCtx::CtxModeConsFlag();
    m_BinEncoder.encodeBin( flag, Ctx::ModeConsFlag( ctxIdx ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "mode_cons_flag() flag=%d\n", flag );
  }
  else if( val == LDT_MODE_TYPE_INFER )
  {
    assert( modeType == MODE_TYPE_INTRA );
  }
  else
  {
    assert( modeType == partitioner.modeType );
  }
}


void CABACWriter::split_cu_mode( const PartSplit split, const CodingStructure& cs, Partitioner& partitioner )
{
  bool canNo, canQt, canBh, canBv, canTh, canTv;
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv );

  const bool canSpl[6] = { canNo, canQt, canBh, canBv, canTh, canTv };

  unsigned ctxSplit = 0, ctxQtSplit = 0, ctxBttHV = 0, ctxBttH12 = 0, ctxBttV12;
  DeriveCtx::CtxSplit( partitioner, ctxSplit, ctxQtSplit, ctxBttHV, ctxBttH12, ctxBttV12, canSpl );

  const bool canSplit = canBh || canBv || canTh || canTv || canQt;
  const bool isNo     = split == CU_DONT_SPLIT;

  if( canNo && canSplit )
  {
    m_BinEncoder.encodeBin( !isNo, Ctx::SplitFlag( ctxSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d split=%d\n", ctxSplit, !isNo );

  if( isNo )
  {
    return;
  }

  const bool canBtt = canBh || canBv || canTh || canTv;
  const bool isQt   = split == CU_QUAD_SPLIT;

  if( canQt && canBtt )
  {
    m_BinEncoder.encodeBin( isQt, Ctx::SplitQtFlag( ctxQtSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d qt=%d\n", ctxQtSplit, isQt );

  if( isQt )
  {
    return;
  }

  const bool canHor = canBh || canTh;
  const bool canVer = canBv || canTv;
  const bool  isVer = split == CU_VERT_SPLIT || split == CU_TRIV_SPLIT;

  if( canVer && canHor )
  {
    m_BinEncoder.encodeBin( isVer, Ctx::SplitHvFlag( ctxBttHV ) );
  }

  const bool can14 = isVer ? canTv : canTh;
  const bool can12 = isVer ? canBv : canBh;
  const bool  is12 = isVer ? ( split == CU_VERT_SPLIT ) : ( split == CU_HORZ_SPLIT );

  if( can12 && can14 )
  {
    m_BinEncoder.encodeBin( is12, Ctx::Split12Flag( isVer ? ctxBttV12 : ctxBttH12 ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctxHv=%d ctx12=%d mode=%d\n", ctxBttHV, isVer ? ctxBttV12 : ctxBttH12, split );
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
//    void  end_of_ctu                ( cu, cuCtx )
//================================================================================

void CABACWriter::coding_unit( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
  DTRACE( g_trace_ctx, D_SYNTAX, "coding_unit() treeType=%d modeType=%d\n", cu.treeType, cu.modeType );
  STAT_COUNT_CU_MODES( isEncoding() && partitioner.chType == CH_L, g_cuCounters1D[CU_CODED_FINALLY][0][!cu.slice->isIntra() + cu.slice->depth] );
  STAT_COUNT_CU_MODES( isEncoding() && partitioner.chType == CH_L && !cu.slice->isIntra(), g_cuCounters2D[CU_CODED_FINALLY][Log2( cu.lheight() )][Log2( cu.lwidth() )] );

  CodingStructure& cs = *cu.cs;

  // skip flag
  if ((!cs.slice->isIntra() || cs.slice->sps->IBC) && cu.Y().valid())
  {
    cu_skip_flag( cu );
  }
  
  // skip data
  if( cu.skip )
  {
    CHECK( !cu.mergeFlag, "Merge flag has to be on!" );
    prediction_unit ( cu );
    CHECK(cu.colorTransform, "ACT should not be enabled for skip mode");
    end_of_ctu      ( cu, cuCtx );
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
  if (!CS::isDualITree(cs) && isLuma(partitioner.chType) && isChromaEnabled(cu.chromaFormat))
      bdpcm_mode(cu, ComponentID(CH_C));

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  // end of cu
  end_of_ctu( cu, cuCtx );
}


void CABACWriter::cu_skip_flag( const CodingUnit& cu )
{
  unsigned ctxId = DeriveCtx::CtxSkipFlag();

  if ((cu.slice->isIntra() || CU::isConsIntra(cu)) && cu.cs->slice->sps->IBC)
  {
    if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
    {
      m_BinEncoder.encodeBin((cu.skip), Ctx::SkipFlag(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0);
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
  m_BinEncoder.encodeBin( ( cu.skip ), Ctx::SkipFlag( ctxId ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0 );
  if (cu.skip && cu.cs->slice->sps->IBC)
  {
    if (cu.lwidth() < 128 && cu.lheight() < 128 && !CU::isConsInter(cu)) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
    {
      if ( cu.lwidth() == 4 && cu.lheight() == 4 )
      {
        return;
      }
      unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
      m_BinEncoder.encodeBin(CU::isIBC(cu) ? 1 : 0, Ctx::IBCFlag(ctxidx));
      DTRACE(g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode);
    }
  }
}


void CABACWriter::pred_mode( const CodingUnit& cu )
{
  if (cu.cs->slice->sps->IBC && cu.chType != CH_C)
  {
    if( CU::isConsInter(cu) )
    {
      assert( CU::isInter( cu ) );
      return;
    }

    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || CU::isConsIntra(cu) )
    {
      if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
      {
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        m_BinEncoder.encodeBin(CU::isIBC(cu), Ctx::IBCFlag(ctxidx));
      }
      if (!CU::isIBC(cu) && cu.cs->slice->sps->PLT && cu.lwidth() <= 64 && cu.lheight() <= 64 && (cu.lumaSize().width * cu.lumaSize().height > 16))
      {
        m_BinEncoder.encodeBin(CU::isPLT(cu), Ctx::PLTFlag(0));
      }
    }
    else
    {
      if( CU::isConsInter(cu) )
      {
        return;
      }
      m_BinEncoder.encodeBin((CU::isIntra(cu) || CU::isPLT(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag()));
      if (CU::isIntra(cu) || CU::isPLT(cu))
      {
        if (cu.cs->slice->sps->PLT && cu.lwidth() <= 64 && cu.lheight() <= 64 && (cu.lumaSize().width * cu.lumaSize().height > 16))
        {
          m_BinEncoder.encodeBin(CU::isPLT(cu), Ctx::PLTFlag(0));
        }
      }
      else
      {
        if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
        {
          unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
          m_BinEncoder.encodeBin(CU::isIBC(cu), Ctx::IBCFlag(ctxidx));
        }
      }
    }
  }
  else
  {
    if( CU::isConsInter(cu) )
    {
      assert( CU::isInter( cu ) );
      return;
    }

    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || CU::isConsIntra(cu) )
    {
      if (cu.cs->slice->sps->PLT && cu.lwidth() <= 64 && cu.lheight() <= 64 && ( ( (!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16) ) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16 ) ) ) && (!CU::isLocalSepTree(cu) || isLuma(cu.chType) ) )
      {
        m_BinEncoder.encodeBin((CU::isPLT(cu)), Ctx::PLTFlag(0));
      }
      return;
    }
    m_BinEncoder.encodeBin((CU::isIntra(cu) || CU::isPLT(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag()));
    if ((CU::isIntra(cu) || CU::isPLT(cu)) && cu.cs->slice->sps->PLT && cu.lwidth() <= 64 && cu.lheight() <= 64&& ( ( (!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16) ) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16 ) ) ) && (!CU::isLocalSepTree(cu) || isLuma(cu.chType)  ) )
    {
      m_BinEncoder.encodeBin((CU::isPLT(cu)), Ctx::PLTFlag(0));
    }
  }
}


void CABACWriter::bdpcm_mode( const CodingUnit& cu, const ComponentID compID )
{
  if( !cu.cs->sps->BDPCM) return;
  if( !CU::bdpcmAllowed( cu, compID ) ) return;

  int bdpcmMode = isLuma(compID) ? cu.bdpcmMode : cu.bdpcmModeChroma;

  unsigned ctxId = isLuma(compID) ? 0 : 2; 
  m_BinEncoder.encodeBin(bdpcmMode > 0 ? 1 : 0, Ctx::BDPCMMode(ctxId));
  if (bdpcmMode)
  {
    m_BinEncoder.encodeBin(bdpcmMode > 1 ? 1 : 0, Ctx::BDPCMMode(ctxId+1));
  }
  if (isLuma(compID))
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CH_L, cu.lumaPos().x, cu.lumaPos().y, cu.lwidth(), cu.lheight(), cu.bdpcmMode);
  }
  else
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CH_C, cu.chromaPos().x, cu.chromaPos().y, cu.chromaSize().width, cu.chromaSize().height, cu.bdpcmModeChroma);
  }
}


void CABACWriter::cu_pred_data( const CodingUnit& cu )
{
  if( CU::isIntra( cu ) )
  {
    if( cu.Y().valid() )
    {
      bdpcm_mode( cu, COMP_Y );
    }
    intra_luma_pred_modes  ( cu );
    if( ( !cu.Y().valid() || ( !CU::isSepTree(cu) && cu.Y().valid() ) ) && isChromaEnabled(cu.chromaFormat) )
    {
      bdpcm_mode( cu, ComponentID(CH_C) );
    } 
    intra_chroma_pred_modes( cu );
    return;
  }
  if (!cu.Y().valid()) // dual tree chroma CU
  {
    return;
  }

  prediction_unit ( cu );
  imv_mode        ( cu );
  affine_amvr_mode( cu );
  cu_bcw_flag     ( cu );
}


void CABACWriter::cu_bcw_flag(const CodingUnit& cu)
{
  if(!CU::isBcwIdxCoded(cu))
  {
    return;
  }

  CHECK(!(BCW_NUM > 1 && (BCW_NUM == 2 || (BCW_NUM & 0x01) == 1)), " !( BCW_NUM > 1 && ( BCW_NUM == 2 || ( BCW_NUM & 0x01 ) == 1 ) ) ");
  const uint8_t bcwCodingIdx = 0 /*(uint8_t)g_BCWCodingOrder[CU::getValidBcwIdx(cu)]*/;

  THROW("no support");

  const int32_t numBcw = (cu.slice->checkLDC) ? 5 : 3;
  m_BinEncoder.encodeBin((bcwCodingIdx == 0 ? 0 : 1), Ctx::BcwIdx(0));
  if(numBcw > 2 && bcwCodingIdx != 0)
  {
    const uint32_t prefixNumBits = numBcw - 2;
    const uint32_t step = 1;

    uint8_t idx = 1;
    for(int ui = 0; ui < prefixNumBits; ++ui)
    {
      if (bcwCodingIdx == idx)
      {
        m_BinEncoder.encodeBinEP(0);
        break;
      }
      else
      {
        m_BinEncoder.encodeBinEP(1);
        idx += step;
      }
    }
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_bcw_flag() bcw_idx=%d\n", cu.BcwIdx ? 1 : 0);
}


void CABACWriter::xWriteTruncBinCode(uint32_t symbol, uint32_t maxSymbol)
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
  assert(val <= maxSymbol);
  assert((val << 1) > maxSymbol);
  assert(symbol < maxSymbol);
  int b = maxSymbol - val;
  assert(b < val);
  if (symbol < val - b)
  {
    m_BinEncoder.encodeBinsEP(symbol, thresh);
  }
  else
  {
    symbol += val - b;
    assert(symbol < (val << 1));
    assert((symbol >> 1) >= val - b);
    m_BinEncoder.encodeBinsEP(symbol, thresh + 1);
  }
}


void CABACWriter::extend_ref_line(const CodingUnit& cu)
{
  if ( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.bdpcmMode )
  {
    return;
  }
  if( !cu.cs->sps->MRL )
  {
    return;
  }

  bool isFirstLineOfCtu = (((cu.block(COMP_Y).y)&((cu.cs->sps)->CTUSize - 1)) == 0);
  if (isFirstLineOfCtu)
  {
    return;
  }
  int multiRefIdx = cu.multiRefIdx;
  if (MRL_NUM_REF_LINES > 1)
  {
    m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], Ctx::MultiRefLineIdx(0));
    if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
    {
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], Ctx::MultiRefLineIdx(1));
    }
  }
}


void CABACWriter::intra_luma_pred_modes( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }

  if( cu.bdpcmMode )
  {
    //th ?
    //cu.intraDir[0] = cu.bdpcmMode == 2? VER_IDX : HOR_IDX;
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

  const int numMPMs   = NUM_MOST_PROBABLE_MODES;
  unsigned  mpm_pred   [numMPMs];
  unsigned  mpm_idx;
  unsigned  ipred_mode ;

  // prev_intra_luma_pred_flag
  {
    CU::getIntraMPMs( cu, mpm_pred );

    ipred_mode = cu.intraDir[0];
    mpm_idx    = numMPMs;
    for( unsigned idx = 0; idx < numMPMs; idx++ )
    {
      if( ipred_mode == mpm_pred[idx] )
      {
        mpm_idx = idx;
        break;
      }
    }
    if ( cu.multiRefIdx )
    {
      CHECK(mpm_idx >= numMPMs, "use of non-MPM");
    }
    else
    {
      m_BinEncoder.encodeBin(mpm_idx < numMPMs, Ctx::IntraLumaMpmFlag());
    }
  }

  // mpm_idx / rem_intra_luma_pred_mode
  {
    if( mpm_idx < numMPMs )
    {
      {
        unsigned ctx = (cu.ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0);
        if (cu.multiRefIdx == 0)
          m_BinEncoder.encodeBin(mpm_idx > 0, Ctx::IntraLumaPlanarFlag(ctx));
        if( mpm_idx )
        {
          m_BinEncoder.encodeBinEP( mpm_idx > 1 );
        }
        if (mpm_idx > 1)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 2);
        }
        if (mpm_idx > 2)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 3);
        }
        if (mpm_idx > 3)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 4);
        }
      }
    }
    else
    {
      // sorting of MPMs
      std::sort( mpm_pred, mpm_pred + numMPMs );

      {
        for (int idx = numMPMs - 1; idx >= 0; idx--)
        {
          if (ipred_mode > mpm_pred[idx])
          {
            ipred_mode--;
          }
        }
        CHECK(ipred_mode >= 64, "Incorrect mode");
        xWriteTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);  // Remaining mode is truncated binary coded
      }
    }

    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", 0, cu.lumaPos().x, cu.lumaPos().y, cu.intraDir[0] );
  }
}


void CABACWriter::intra_luma_pred_mode( const CodingUnit& cu )
{

  if( cu.bdpcmMode ) return;
  mip_flag(cu);
  if (cu.mipFlag)
  {
    mip_pred_mode(cu);
    return;
  }
  extend_ref_line( cu );

  isp_mode( cu );

  // prev_intra_luma_pred_flag
  const int numMPMs  = NUM_MOST_PROBABLE_MODES;
  unsigned  mpm_pred[numMPMs];

  CU::getIntraMPMs( cu, mpm_pred );

  unsigned ipred_mode = cu.intraDir[0];
  unsigned mpm_idx = numMPMs;

  for( int idx = 0; idx < numMPMs; idx++ )
  {
    if( ipred_mode == mpm_pred[idx] )
    {
      mpm_idx = idx;
      break;
    }
  }
  if ( cu.multiRefIdx )
  {
    CHECK(mpm_idx >= numMPMs, "use of non-MPM");
  }
  else
  {
    m_BinEncoder.encodeBin(mpm_idx < numMPMs, Ctx::IntraLumaMpmFlag());
  }

  // mpm_idx / rem_intra_luma_pred_mode
  if( mpm_idx < numMPMs )
  {
    {
      unsigned ctx = (cu.ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0);
      if (cu.multiRefIdx == 0)
        m_BinEncoder.encodeBin( mpm_idx > 0, Ctx::IntraLumaPlanarFlag(ctx) );
      if( mpm_idx )
      {
        m_BinEncoder.encodeBinEP( mpm_idx > 1 );
      }
      if (mpm_idx > 1)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 2);
      }
      if (mpm_idx > 2)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 3);
      }
      if (mpm_idx > 3)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 4);
      }
    }
  }
  else
  {
    std::sort( mpm_pred, mpm_pred + numMPMs );
    {
      for (int idx = numMPMs - 1; idx >= 0; idx--)
      {
        if (ipred_mode > mpm_pred[idx])
        {
          ipred_mode--;
        }
      }
      xWriteTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);  // Remaining mode is truncated binary coded
    }
  }
}


void CABACWriter::intra_chroma_pred_modes( const CodingUnit& cu )
{
  if( cu.chromaFormat == CHROMA_400 || ( CU::isSepTree(cu) && cu.chType == CH_L ) )
  {
    return;
  }
  if( cu.bdpcmModeChroma )
  {
    //th ?
//    cu.intraDir[1] = cu.bdpcmModeChroma == 2 ? VER_IDX : HOR_IDX;
    return;
  }

  intra_chroma_pred_mode( cu );
}


void CABACWriter::intra_chroma_lmc_mode(const CodingUnit& cu)
{
  const unsigned intraDir = cu.intraDir[1];
  int lmModeList[10];
  CU::getLMSymbolList(cu, lmModeList);
  int symbol = -1;
  for (int k = 0; k < LM_SYMBOL_NUM; k++)
  {
    if (lmModeList[k] == intraDir)
    {
      symbol = k;
      break;
    }
  }
  CHECK(symbol < 0, "invalid symbol found");

  m_BinEncoder.encodeBin(symbol == 0 ? 0 : 1, Ctx::CclmModeIdx(0));

  if (symbol > 0)
  {
    CHECK(symbol > 2, "invalid symbol for MMLM");
    unsigned int symbol_minus_1 = symbol - 1;
    m_BinEncoder.encodeBinEP(symbol_minus_1);
  }
}


void CABACWriter::intra_chroma_pred_mode(const CodingUnit& cu)
{
  if (cu.colorTransform)
  {
    CHECK(cu.intraDir[CH_C] != DM_CHROMA_IDX, "chroma should use DM for adaptive color transform");
    return;
  }

  const unsigned intraDir = cu.intraDir[1];
  if (cu.cs->sps->LMChroma && CU::checkCCLMAllowed(cu))
  {
    m_BinEncoder.encodeBin(CU::isLMCMode(intraDir) ? 1 : 0, Ctx::CclmModeFlag(0));
    if (CU::isLMCMode(intraDir))
    {
      intra_chroma_lmc_mode(cu);
      return;
    }
  }

  const bool     isDerivedMode = intraDir == DM_CHROMA_IDX;
  m_BinEncoder.encodeBin(isDerivedMode ? 0 : 1, Ctx::IntraChromaPredMode(0));
  if (isDerivedMode)
  {
    return;
  }

  // chroma candidate index
  unsigned chromaCandModes[NUM_CHROMA_MODE];
  CU::getIntraChromaCandModes(cu, chromaCandModes);

  int candId = 0;
  for (; candId < NUM_CHROMA_MODE; candId++)
  {
    if (intraDir == chromaCandModes[candId])
    {
      break;
    }
  }

  CHECK(candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds");
  CHECK(chromaCandModes[candId] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path");
  {
    m_BinEncoder.encodeBinsEP(candId, 2);
  }
}


void CABACWriter::cu_residual( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
  if (!CU::isIntra(cu))
  {
    if( !cu.mergeFlag )
    {
      rqt_root_cbf( cu );
    }
    if( cu.rootCbf )
    {
      sbt_mode( cu );
    }

    if( !cu.rootCbf )
    {
      CHECK(cu.colorTransform, "ACT should not be enabled for root_cbf = 0");
      return;
    }
  }

  if( CU::isInter( cu ) || CU::isIBC( cu ) )
  {
    adaptive_color_transform(cu);
  }

  cuCtx.violatesLfnstConstrained[CH_L] = false;
  cuCtx.violatesLfnstConstrained[CH_C] = false;
  cuCtx.lfnstLastScanPos               = false;
  cuCtx.violatesMtsCoeffConstraint     = false;
  cuCtx.mtsLastScanPos                                = false;

  if( cu.ispMode && isLuma( partitioner.chType ) )
  {
    transform_tree( *cu.cs, partitioner, cuCtx, CU::getISPType( cu, getFirstComponentOfChannel( partitioner.chType ) ), 0 );
  }
  else
  {
    transform_tree( *cu.cs, partitioner, cuCtx );
  }

  residual_lfnst_mode( cu, cuCtx );
  mts_idx            ( cu, &cuCtx );
}


void CABACWriter::rqt_root_cbf( const CodingUnit& cu )
{
  m_BinEncoder.encodeBin( cu.rootCbf, Ctx::QtRootCbf() );

  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}


void CABACWriter::adaptive_color_transform(const CodingUnit& cu)
{
  if (!cu.slice->sps->useColorTrans )
  {
    return;
  }

  if (CU::isSepTree(cu))
  {
    CHECK(cu.colorTransform, "adaptive color transform should be disabled when dualtree and localtree are enabled");
    return;
  }

  if (CU::isInter(cu) || CU::isIBC(cu) || CU::isIntra(cu))
  {
    m_BinEncoder.encodeBin(cu.colorTransform, Ctx::ACTFlag());
  }
}


void CABACWriter::sbt_mode( const CodingUnit& cu )
{
  uint8_t sbtAllowed = CU::checkAllowedSbt(cu);
  if( !sbtAllowed )
  {
    return;
  }

  SizeType cuWidth = cu.lwidth();
  SizeType cuHeight = cu.lheight();
  uint8_t sbtIdx = CU::getSbtIdx( cu.sbtInfo );
  uint8_t sbtPos = CU::getSbtPos( cu.sbtInfo );

  //bin - flag
  bool sbtFlag = cu.sbtInfo != 0;
  uint8_t ctxIdx = ( cuWidth * cuHeight <= 256 ) ? 1 : 0;
  m_BinEncoder.encodeBin( sbtFlag, Ctx::SbtFlag( ctxIdx ) );
  if( !sbtFlag )
  {
    return;
  }

  bool sbtQuadFlag = sbtIdx == SBT_HOR_QUAD || sbtIdx == SBT_VER_QUAD;
  bool sbtHorFlag = sbtIdx == SBT_HOR_HALF || sbtIdx == SBT_HOR_QUAD;
  bool sbtPosFlag = sbtPos == SBT_POS1;

  uint8_t sbtVerHalfAllow = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed );
  uint8_t sbtHorHalfAllow = CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  uint8_t sbtVerQuadAllow = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed );
  uint8_t sbtHorQuadAllow = CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );
  //bin - type
  if( ( sbtHorHalfAllow || sbtVerHalfAllow ) && ( sbtHorQuadAllow || sbtVerQuadAllow ) )
  {
    m_BinEncoder.encodeBin( sbtQuadFlag, Ctx::SbtQuadFlag( 0 ) );
  }
  else
  {
    assert( sbtQuadFlag == 0 );
  }

  //bin - dir
  if( ( sbtQuadFlag && sbtVerQuadAllow && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtVerHalfAllow && sbtHorHalfAllow ) ) //both direction allowed
  {
    uint8_t ctxIdx = ( cuWidth == cuHeight ) ? 0 : ( cuWidth < cuHeight ? 1 : 2 );
    m_BinEncoder.encodeBin( sbtHorFlag, Ctx::SbtHorFlag( ctxIdx ) );
  }
  else
  {
    assert( sbtHorFlag == ( ( sbtQuadFlag && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtHorHalfAllow ) ) );
  }

  //bin - pos
  m_BinEncoder.encodeBin( sbtPosFlag, Ctx::SbtPosFlag( 0 ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo );
}


void CABACWriter::end_of_ctu( const CodingUnit& cu, CUCtx& cuCtx )
{
  const bool    isLastSubCUOfCtu  = CU::isLastSubCUOfCtu( cu );

  if ( isLastSubCUOfCtu
    && ( !CU::isSepTree(cu) || cu.chromaFormat == CHROMA_400 || isChroma( cu.chType ) )
      )
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->useDQP && !cuCtx.isDQPCoded );

  }
}


void CABACWriter::cu_palette_info(const CodingUnit& cu, ComponentID compBegin, uint32_t numComp, CUCtx& cuCtx)
{
  THROW("no support");
}


//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( cu );
//    void  merge_flag      ( cu );
//    void  merge_idx       ( cu );
//    void  inter_pred_idc  ( cu );
//    void  ref_idx         ( cu, refList );
//    void  mvp_flag        ( cu, refList );
//================================================================================

void CABACWriter::prediction_unit( const CodingUnit& cu )
{
  CHECK( cu.treeType == TREE_C, "cannot be chroma CU" );
  if( cu.skip )
  {
    CHECK( !cu.mergeFlag, "merge_flag must be true for skipped CUs" );
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
    ref_idx(cu, REF_PIC_LIST_0);
    Mv mvd = cu.mvd[REF_PIC_LIST_0];
    mvd.changeIbcPrecInternal2Amvr(cu.imv);
    mvd_coding(mvd, 0); // already changed to signaling precision
    if ( cu.slice->sps->maxNumIBCMergeCand == 1 )
    {
      CHECK( cu.mvpIdx[REF_PIC_LIST_0], "mvpIdx for IBC mode should be 0" );
    }
    else
    mvp_flag(cu, REF_PIC_LIST_0);
  }
  else
  {
    inter_pred_idc( cu );
    affine_flag   ( cu );
    smvd_mode( cu );
    if( cu.interDir != 2 /* PRED_L1 */ )
    {
      ref_idx     ( cu, REF_PIC_LIST_0 );
      if ( cu.affine )
      {
        Mv mvd = cu.mvdAffi[REF_PIC_LIST_0][0];
        mvd.changeAffinePrecInternal2Amvr(cu.imv);
        mvd_coding(mvd, 0); // already changed to signaling precision
        mvd = cu.mvdAffi[REF_PIC_LIST_0][1];
        mvd.changeAffinePrecInternal2Amvr(cu.imv);
        mvd_coding(mvd, 0); // already changed to signaling precision
        if ( cu.affineType == AFFINEMODEL_6PARAM )
        {
          mvd = cu.mvdAffi[REF_PIC_LIST_0][2];
          mvd.changeAffinePrecInternal2Amvr(cu.imv);
          mvd_coding(mvd, 0); // already changed to signaling precision
        }
      }
      else
      {
        Mv mvd = cu.mvd[REF_PIC_LIST_0];
        mvd.changeTransPrecInternal2Amvr(cu.imv);
        mvd_coding(mvd, 0); // already changed to signaling precision
      }
      mvp_flag    ( cu, REF_PIC_LIST_0 );
    }
    if( cu.interDir != 1 /* PRED_L0 */ )
    {
      if ( cu.smvdMode != 1 )
      {
      ref_idx     ( cu, REF_PIC_LIST_1 );
      if( !cu.cs->picHeader->mvdL1Zero || cu.interDir != 3 /* PRED_BI */ )
      {
        if ( cu.affine )
        {
          Mv mvd = cu.mvdAffi[REF_PIC_LIST_1][0];
          mvd.changeAffinePrecInternal2Amvr(cu.imv);
          mvd_coding(mvd, 0); // already changed to signaling precision
          mvd = cu.mvdAffi[REF_PIC_LIST_1][1];
          mvd.changeAffinePrecInternal2Amvr(cu.imv);
          mvd_coding(mvd, 0); // already changed to signaling precision
          if ( cu.affineType == AFFINEMODEL_6PARAM )
          {
            mvd = cu.mvdAffi[REF_PIC_LIST_1][2];
            mvd.changeAffinePrecInternal2Amvr(cu.imv);
            mvd_coding(mvd, 0); // already changed to signaling precision
          }
        }
        else
        {
          Mv mvd = cu.mvd[REF_PIC_LIST_1];
          mvd.changeTransPrecInternal2Amvr(cu.imv);
          mvd_coding(mvd, 0); // already changed to signaling precision
        }
      }
      }
      mvp_flag    ( cu, REF_PIC_LIST_1 );
    }
  }
}


void CABACWriter::smvd_mode( const CodingUnit& cu )
{
  if ( cu.interDir != 3 || cu.affine )
  {
    return;
  }

  if ( cu.cs->slice->biDirPred == false )
  {
    return;
  }

  m_BinEncoder.encodeBin( cu.smvdMode ? 1 : 0, Ctx::SmvdFlag() );

  DTRACE( g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", cu.smvdMode ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height );
}


void CABACWriter::subblock_merge_flag( const CodingUnit& cu )
{

  if ( !cu.cs->slice->isIntra() && (cu.slice->picHeader->maxNumAffineMergeCand > 0) && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8 )
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag();
    m_BinEncoder.encodeBin( cu.affine, Ctx::SubblockMergeFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
  }
}


void CABACWriter::affine_flag( const CodingUnit& cu )
{
  if ( !cu.cs->slice->isIntra() && cu.cs->sps->Affine && cu.lumaSize().width > 8 && cu.lumaSize().height > 8 )
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag();
    m_BinEncoder.encodeBin( cu.affine, Ctx::AffineFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );

    if ( cu.affine && cu.cs->sps->AffineType )
    {
      unsigned ctxId = 0;
      m_BinEncoder.encodeBin( cu.affineType, Ctx::AffineType( ctxId ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n", cu.affineType ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
    }
  }
}


void CABACWriter::merge_flag( const CodingUnit& cu )
{
  m_BinEncoder.encodeBin( cu.mergeFlag, Ctx::MergeFlag() );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", cu.mergeFlag ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height );

}


void CABACWriter::merge_data(const CodingUnit& cu)
{
  if (CU::isIBC(cu))
  {
    merge_idx(cu);
    return;
  }
  subblock_merge_flag(cu);
  if (cu.affine)
  {
    merge_idx(cu);
    return;
  }
  const bool ciipAvailable = cu.cs->sps->CIIP && !cu.skip && cu.lwidth() < MAX_CU_SIZE && cu.lheight() < MAX_CU_SIZE && cu.lwidth() * cu.lheight() >= 64;
  const bool geoAvailable = cu.cs->slice->sps->GEO && cu.cs->slice->isInterB() && cu.cs->sps->maxNumGeoCand > 1
                                                   && cu.lwidth() >= GEO_MIN_CU_SIZE && cu.lheight() >= GEO_MIN_CU_SIZE
                                                   && cu.lwidth() <= GEO_MAX_CU_SIZE && cu.lheight() <= GEO_MAX_CU_SIZE
                                                   && cu.lwidth() < 8 * cu.lheight() && cu.lheight() < 8 * cu.lwidth();
  if (geoAvailable || ciipAvailable)
  {
    m_BinEncoder.encodeBin(cu.regularMergeFlag, Ctx::RegularMergeFlag(cu.skip ? 0 : 1));
  }
  if (cu.regularMergeFlag)
  {
    if (cu.cs->sps->MMVD)
    {
      m_BinEncoder.encodeBin(cu.mmvdMergeFlag, Ctx::MmvdFlag(0));
      DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_flag() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", cu.mmvdMergeFlag ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height);
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
  else
  {
    if (geoAvailable && ciipAvailable)
    {
      ciip_flag(cu);
    }
    merge_idx(cu);
  }
}


void CABACWriter::imv_mode( const CodingUnit& cu )
{
  const SPS *sps = cu.cs->sps;

  if( !sps->AMVR )
  {
    return;
  }
  if ( cu.affine )
  {
    return;
  }

  bool bNonZeroMvd = CU::hasSubCUNonZeroMVd( cu );
  if( !bNonZeroMvd )
  {
    return;
  }

  if (CU::isIBC(cu) == false)
    m_BinEncoder.encodeBin( (cu.imv > 0), Ctx::ImvFlag( 0 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", (cu.imv > 0), 0 );

  if( sps->AMVR && cu.imv > 0 )
  {
    if (!CU::isIBC(cu))
    {
      m_BinEncoder.encodeBin(cu.imv < IMV_HPEL, Ctx::ImvFlag(4));
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", cu.imv < 3, 4);
    }
    if (cu.imv < IMV_HPEL)
    {
    m_BinEncoder.encodeBin( (cu.imv > 1), Ctx::ImvFlag( 1 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", (cu.imv > 1), 1 );
    }
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv );
}


void CABACWriter::affine_amvr_mode( const CodingUnit& cu )
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

  m_BinEncoder.encodeBin( (cu.imv > 0), Ctx::ImvFlag( 2 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", (cu.imv > 0), 2 );

  if( cu.imv > 0 )
  {
    m_BinEncoder.encodeBin( (cu.imv > 1), Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", (cu.imv > 1), 3 );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv );
}


void CABACWriter::merge_idx( const CodingUnit& cu )
{
  if ( cu.affine )
  {
    int numCandminus1 = int( cu.cs->picHeader->maxNumAffineMergeCand ) - 1;
    if ( numCandminus1 > 0 )
    {
      if ( cu.mergeIdx == 0 )
      {
        m_BinEncoder.encodeBin( 0, Ctx::AffMergeIdx() );
        DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", cu.mergeIdx );
        return;
      }
      else
      {
        m_BinEncoder.encodeBin( 1, Ctx::AffMergeIdx() );
        for ( unsigned idx = 1; idx < numCandminus1; idx++ )
        {
            m_BinEncoder.encodeBinEP( cu.mergeIdx == idx ? 0 : 1 );
          if ( cu.mergeIdx == idx )
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
    if( cu.geo )
    {
      uint8_t splitDir = cu.geoSplitDir;
      uint8_t candIdx0 = cu.geoMergeIdx0;
      uint8_t candIdx1 = cu.geoMergeIdx1;
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%d\n", splitDir );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx0=%d\n", candIdx0 );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx1=%d\n", candIdx1 );
      xWriteTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
      candIdx1 -= candIdx1 < candIdx0 ? 0 : 1;
      const int maxNumGeoCand = cu.cs->sps->maxNumGeoCand;
      CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
      CHECK(candIdx0 >= maxNumGeoCand, "Incorrect candIdx0");
      CHECK(candIdx1 >= maxNumGeoCand, "Incorrect candIdx1");
      int numCandminus2 = maxNumGeoCand - 2;
      m_BinEncoder.encodeBin( candIdx0 == 0 ? 0 : 1, Ctx::MergeIdx() );
      if( candIdx0 > 0 )
      {
        unary_max_eqprob(candIdx0 - 1, numCandminus2);
      }
      if (numCandminus2 > 0)
      {
        m_BinEncoder.encodeBin(candIdx1 == 0 ? 0 : 1, Ctx::MergeIdx());
        if (candIdx1 > 0)
        {
          unary_max_eqprob(candIdx1 - 1, numCandminus2 - 1);
        }
      }
      return;
    }
    int numCandminus1 = (cu.predMode == MODE_IBC) ? (int(cu.cs->sps->maxNumIBCMergeCand) - 1) : (int(cu.cs->sps->maxNumMergeCand) - 1);
    if( numCandminus1 > 0 )
    {
      if( cu.mergeIdx == 0 )
      {
        m_BinEncoder.encodeBin( 0, Ctx::MergeIdx() );
        DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", cu.mergeIdx );
        return;
      }
      else
      {
        m_BinEncoder.encodeBin( 1, Ctx::MergeIdx() );
        for( unsigned idx = 1; idx < numCandminus1; idx++ )
        {
          m_BinEncoder.encodeBinEP( cu.mergeIdx == idx ? 0 : 1 );
          if( cu.mergeIdx == idx )
          {
            break;
          }
        }
      }
    }
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", cu.mergeIdx );
  }
}


void CABACWriter::mmvd_merge_idx(const CodingUnit& cu)
{
  int var0, var1, var2;
  int mvpIdx = cu.mmvdMergeIdx;
  var0 = mvpIdx / MMVD_MAX_REFINE_NUM;
  var1 = (mvpIdx - (var0 * MMVD_MAX_REFINE_NUM)) / 4;
  var2 = mvpIdx - (var0 * MMVD_MAX_REFINE_NUM) - var1 * 4;

  if (cu.cs->sps->maxNumMergeCand > 1)
  {
    static_assert(MMVD_BASE_MV_NUM == 2, "");
    assert(var0 < 2);
    m_BinEncoder.encodeBin(var0, Ctx::MmvdMergeIdx());
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", var0);

  int numCandminus1_step = MMVD_REFINE_STEP - 1;
  if (numCandminus1_step > 0)
  {
    if (var1 == 0)
    {
      m_BinEncoder.encodeBin(0, Ctx::MmvdStepMvpIdx());
    }
    else
    {
      m_BinEncoder.encodeBin(1, Ctx::MmvdStepMvpIdx());
      for (unsigned idx = 1; idx < numCandminus1_step; idx++)
      {
        m_BinEncoder.encodeBinEP(var1 == idx ? 0 : 1);
        if (var1 == idx)
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "MmvdStepMvpIdx() MmvdStepMvpIdx=%d\n", var1);

  m_BinEncoder.encodeBinsEP(var2, 2);

  DTRACE(g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", var2);
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", cu.mmvdMergeIdx);
}


void CABACWriter::inter_pred_idc( const CodingUnit& cu )
{
  if( !cu.cs->slice->isInterB() )
  {
    return;
  }
  if( !(CU::isBipredRestriction(cu)) )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(cu);
    if( cu.interDir == 3 )
    {
      m_BinEncoder.encodeBin( 1, Ctx::InterDir(ctxId) );
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, cu.interDir, cu.lumaPos().x, cu.lumaPos().y );
      return;
    }
    else
    {
      m_BinEncoder.encodeBin( 0, Ctx::InterDir(ctxId) );
    }
  }
  m_BinEncoder.encodeBin( ( cu.interDir == 2 ), Ctx::InterDir( 5 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=5 value=%d pos=(%d,%d)\n", cu.interDir, cu.lumaPos().x, cu.lumaPos().y );
}


void CABACWriter::ref_idx( const CodingUnit& cu, RefPicList eRefList )
{
  if ( cu.smvdMode )
  {
    CHECK( cu.refIdx[eRefList] != cu.cs->slice->symRefIdx[ eRefList ], "Invalid reference index!\n" );
    return;
  }

  int numRef  = cu.cs->slice->numRefIdx[eRefList];

  if (eRefList == REF_PIC_LIST_0 && cu.cs->sps->IBC)
  {
    if (CU::isIBC(cu))
      return;
  }

  if( numRef <= 1 )
  {
    return;
  }
  int refIdx  = cu.refIdx[eRefList];
  m_BinEncoder.encodeBin( (refIdx > 0), Ctx::RefPic() );
  if( numRef <= 2 || refIdx == 0 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, cu.lumaPos().x, cu.lumaPos().y );
    return;
  }
  m_BinEncoder.encodeBin( (refIdx > 1), Ctx::RefPic(1) );
  if( numRef <= 3 || refIdx == 1 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, cu.lumaPos().x, cu.lumaPos().y );
    return;
  }
  for( int idx = 3; idx < numRef; idx++ )
  {
    if( refIdx > idx - 1 )
    {
      m_BinEncoder.encodeBinEP( 1 );
    }
    else
    {
      m_BinEncoder.encodeBinEP( 0 );
      break;
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, cu.lumaPos().x, cu.lumaPos().y );
}


void CABACWriter::mvp_flag( const CodingUnit& cu, RefPicList eRefList )
{
  m_BinEncoder.encodeBin( cu.mvpIdx[eRefList], Ctx::MVPIdx() );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", cu.mvpIdx[eRefList], cu.lumaPos().x, cu.lumaPos().y );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, cu.mvpIdx[eRefList] );
}


void CABACWriter::ciip_flag(const CodingUnit& cu)
{
  if (!cu.cs->sps->CIIP)
  {
    CHECK(cu.ciip == true, "invalid Ciip SPS");
    return;
  }
  if (cu.skip)
  {
    CHECK(cu.ciip == true, "invalid Ciip and skip");
    return;
  }
  m_BinEncoder.encodeBin(cu.ciip, Ctx::CiipFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "Ciip_flag() Ciip=%d pos=(%d,%d) size=%dx%d\n", cu.ciip ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y, cu.lumaSize().width, cu.lumaSize().height);
}


//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( split, depth )
//    bool  cbf_comp            ( cbf, area, depth )
//================================================================================

void CABACWriter::transform_tree( const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, const PartSplit ispType, const int subTuIdx )
{
  const UnitArea&       area = partitioner.currArea();
  int             subTuCounter = subTuIdx;
  const TransformUnit&  tu = *cs.getTU(area.blocks[partitioner.chType].pos(), partitioner.chType, subTuIdx);
  const CodingUnit&     cu = *tu.cu;
  const unsigned        trDepth = partitioner.currTrDepth;
  const bool            split = (tu.depth > trDepth);
  if( split )
  {
    PartSplit partSplit;

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
    else if( cu.sbtInfo && partitioner.canSplit( partSplit = CU::getSbtTuSplit( cu.sbtInfo ), cs ) )
    {
      partitioner.splitCurrArea( partSplit, cs );
    }
    else
      THROW( "Implicit TU split not available" );

    do
    {
      transform_tree( cs, partitioner, cuCtx,                ispType, subTuCounter );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    // split_transform_flag
    CHECK( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) || (cu.sbtInfo && partitioner.canSplit( CU::getSbtTuSplit( cu.sbtInfo ), cs)),  "transform split implied" );

    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.blocks[tu.chType].width, tu.blocks[tu.chType].height, cu.depth, partitioner.currTrDepth );

    transform_unit( tu, cuCtx, partitioner, subTuCounter);
  }
}


void CABACWriter::cbf_comp( const CodingUnit& cu, bool cbf, const CompArea& area, unsigned depth, const bool prevCbf, const bool useISP )
{
  const CtxSet&   ctxSet  = Ctx::QtCbf[ area.compID ];
  unsigned  ctxId;
  if( ( area.compID == COMP_Y && cu.bdpcmMode ) || ( area.compID != COMP_Y && cu.bdpcmModeChroma ) )
  {
    ctxId = (area.compID != COMP_Cr) ? 1 : 2;
    m_BinEncoder.encodeBin(cbf, ctxSet(ctxId));
  }
  else
  {
    ctxId = DeriveCtx::CtxQtCbf(area.compID, prevCbf, useISP && isLuma(area.compID));
    m_BinEncoder.encodeBin( cbf, ctxSet( ctxId ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
}



//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( cu, refList )
//================================================================================
void CABACWriter::mvd_coding( const Mv &rMvd, int8_t imv )
{
  int       horMvd = rMvd.hor;
  int       verMvd = rMvd.ver;
  if ( imv > 0 )
  {
    int shift = 1;
    if (imv < IMV_HPEL)
    {
      shift = 2;
      if (imv == IMV_4PEL)//IMV_4PEL
      {
        shift = 4;
      }
    }

    CHECK((horMvd % (1<<shift)) != 0 && (verMvd % (4<<shift)) != 0, "IMV: MVD is not a multiple of 2^N ");
    horMvd >>= shift;
    verMvd >>= shift;
  }
  unsigned  horAbs  = unsigned( horMvd < 0 ? -horMvd : horMvd );
  unsigned  verAbs  = unsigned( verMvd < 0 ? -verMvd : verMvd );


  // abs_mvd_greater0_flag[ 0 | 1 ]
  m_BinEncoder.encodeBin( (horAbs > 0), Ctx::Mvd() );
  m_BinEncoder.encodeBin( (verAbs > 0), Ctx::Mvd() );

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    m_BinEncoder.encodeBin( (horAbs > 1), Ctx::Mvd(1) );
  }
  if( verAbs > 0 )
  {
    m_BinEncoder.encodeBin( (verAbs > 1), Ctx::Mvd(1) );
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    if( horAbs > 1 )
    {
      m_BinEncoder.encodeRemAbsEP(horAbs - 2, 1, 0, MV_BITS - 1);
    }
    m_BinEncoder.encodeBinEP( (horMvd < 0) );
  }
  if( verAbs > 0 )
  {
    if( verAbs > 1 )
    {
      m_BinEncoder.encodeRemAbsEP(verAbs - 2, 1, 0, MV_BITS - 1);
    }
    m_BinEncoder.encodeBinEP( (verMvd < 0) );
  }
}


//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================
void CABACWriter::transform_unit( const TransformUnit& tu, CUCtx& cuCtx, Partitioner& partitioner, const int subTuCounter)
{
  const CodingUnit&       cu = *tu.cu;
  const UnitArea&         area = partitioner.currArea();
  const unsigned          trDepth = partitioner.currTrDepth;
  ChromaCbfs              chromaCbfs;
  CHECK(tu.depth != trDepth, " transform unit should be not be futher partitioned");

  // cbf_cb & cbf_cr
  if (area.chromaFormat != CHROMA_400)
  {
    const bool              chromaCbfISP = area.blocks[COMP_Cb].valid() && cu.ispMode;
    if (area.blocks[COMP_Cb].valid() && (!CU::isSepTree(cu) || partitioner.chType == CH_C) && (!cu.ispMode || chromaCbfISP))
  {
    {
      unsigned cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;
      {
        chromaCbfs.Cb = TU::getCbfAtDepth(tu, COMP_Cb, trDepth);
        //if (!(cu.sbtInfo && trDepth == 1))
        if (!(cu.sbtInfo && tu.noResidual))
          cbf_comp(*tu.cu, chromaCbfs.Cb, area.blocks[COMP_Cb], cbfDepth);
      }

      {
        chromaCbfs.Cr = TU::getCbfAtDepth(tu, COMP_Cr, trDepth);
        //if (!(cu.sbtInfo && trDepth == 1))
        if (!(cu.sbtInfo && tu.noResidual))
          cbf_comp(*tu.cu, chromaCbfs.Cr, area.blocks[COMP_Cr], cbfDepth, chromaCbfs.Cb);
      }
    }
  }
  else if (CU::isSepTree(cu))
  {
    chromaCbfs = ChromaCbfs(false);
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
      CHECK(!TU::getCbfAtDepth(tu, COMP_Y, trDepth), "Luma cbf must be true for inter units with no chroma coeffs");
    }
    else if (cu.sbtInfo && tu.noResidual)
    {
      CHECK(TU::getCbfAtDepth(tu, COMP_Y, trDepth), "Luma cbf must be false for inter sbt no-residual tu");
    }
    else if (cu.sbtInfo && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      assert(!tu.noResidual);
      CHECK(!TU::getCbfAtDepth(tu, COMP_Y, trDepth), "Luma cbf must be true for inter sbt residual tu");
    }
    else
    {
      bool previousCbf = false;
      bool rootCbfSoFar = false;
      bool lumaCbfIsInferredACT = (cu.colorTransform && cu.predMode == MODE_INTRA && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat));
      CHECK(lumaCbfIsInferredACT && !TU::getCbfAtDepth(tu, COMP_Y, trDepth), "adaptive color transform cannot have all zero coefficients");
      bool lastCbfIsInferred    = lumaCbfIsInferredACT; // ISP and ACT are mutually exclusive
      if (cu.ispMode)
      {
        uint32_t nTus = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> Log2(tu.lheight()) : cu.lwidth() >> Log2(tu.lwidth());
        if (subTuCounter == nTus - 1)
        {
          TransformUnit* tuPointer = cu.firstTU;
          for (int tuIdx = 0; tuIdx < subTuCounter; tuIdx++)
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
          previousCbf = TU::getPrevTuCbfAtDepth(tu, COMP_Y, partitioner.currTrDepth);
        }
      }
      if (!lastCbfIsInferred)
      {
        cbf_comp(*tu.cu, TU::getCbfAtDepth(tu, COMP_Y, trDepth), tu.Y(), trDepth, previousCbf, cu.ispMode);
      }
    }
  }
  bool        lumaOnly  = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMP_Cb].valid() );
  bool        cbf[3]    = { TU::getCbf( tu, COMP_Y ), chromaCbfs.Cb, chromaCbfs.Cr };
  bool        cbfLuma   = ( cbf[ COMP_Y ] != 0 );
  bool        cbfChroma = false;

  if( !lumaOnly )
  {
    if( tu.blocks[COMP_Cb].valid() )
    {
      cbf   [ COMP_Cb  ] = TU::getCbf( tu, COMP_Cb );
      cbf   [ COMP_Cr  ] = TU::getCbf( tu, COMP_Cr );
    }
    cbfChroma = ( cbf[ COMP_Cb ] || cbf[ COMP_Cr ] );
  }

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
  if (cu.cs->slice->chromaQpAdjEnabled && cbfChroma && !cuCtx.isChromaQpAdjCoded)
  {
    cu_chroma_qp_offset( cu );
    cuCtx.isChromaQpAdjCoded = true;
  }

  if( !lumaOnly )
  {
    joint_cb_cr( tu, ( cbf[COMP_Cb] ? 2 : 0 ) + ( cbf[COMP_Cr] ? 1 : 0 ) );
  }

  if( cbfLuma )
  {
    residual_coding( tu, COMP_Y, &cuCtx );
  }
  if( !lumaOnly )
  {
    for( ComponentID compID = COMP_Cb; compID <= COMP_Cr; compID = ComponentID( compID + 1 ) )
    {
      if( cbf[ compID ] )
      {
        residual_coding( tu, compID, &cuCtx );
      }
    }
  }
}


void CABACWriter::cu_qp_delta( const CodingUnit& cu, int predQP, const int8_t qp )
{
  CHECK(!( predQP != std::numeric_limits<int>::max()), "Unspecified error");
  int       DQp         = qp - predQP;
  int       qpBdOffsetY = cu.cs->sps->qpBDOffset[ CH_L ];
  DQp                   = ( DQp + (MAX_QP + 1) + (MAX_QP + 1) / 2 + qpBdOffsetY + (qpBdOffsetY / 2)) % ((MAX_QP + 1) + qpBdOffsetY) - (MAX_QP + 1) / 2 - (qpBdOffsetY / 2);
  unsigned  absDQP      = unsigned( DQp < 0 ? -DQp : DQp );
  unsigned  unaryDQP    = std::min<unsigned>( absDQP, CU_DQP_TU_CMAX );

  unary_max_symbol( unaryDQP, Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( absDQP >= CU_DQP_TU_CMAX )
  {
    exp_golomb_eqprob( absDQP - CU_DQP_TU_CMAX, CU_DQP_EG_k );
  }
  if( absDQP > 0 )
  {
    m_BinEncoder.encodeBinEP( DQp < 0 );
  }

  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACWriter::cu_chroma_qp_offset( const CodingUnit& cu )
{
  // cu_chroma_qp_offset_flag
  unsigned qpAdj = cu.chromaQpAdj;
  if( qpAdj == 0 )
  {
    m_BinEncoder.encodeBin( 0, Ctx::ChromaQpAdjFlag() );
  }
  else
  {
    m_BinEncoder.encodeBin( 1, Ctx::ChromaQpAdjFlag() );
    int length = cu.cs->pps->chromaQpOffsetListLen;
    if( length > 1 )
    {
      unary_max_symbol( qpAdj-1, Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
    }
  }
}


//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    void        transform_skip_flag     ( tu, compID )
//    void        last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACWriter::joint_cb_cr( const TransformUnit& tu, const int cbfMask )
{
  if ( !tu.cu->slice->sps->jointCbCr )
  {
    return;
  }

  CHECK( tu.jointCbCr && tu.jointCbCr != cbfMask, "wrong value of jointCbCr (" << (int)tu.jointCbCr << " vs " << (int)cbfMask << ")" );
  if( ( CU::isIntra( *tu.cu ) && cbfMask ) || ( cbfMask == 3 ) )
  {
    m_BinEncoder.encodeBin( tu.jointCbCr ? 1 : 0, Ctx::JointCbCrFlag( cbfMask - 1 ) );
  }
}


void CABACWriter::residual_coding( const TransformUnit& tu, ComponentID compID, CUCtx* cuCtx )
{
  const CodingUnit& cu = *tu.cu;
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );

  if( compID == COMP_Cr && tu.jointCbCr == 3 )
  {
    return;
  }

  ts_flag            ( tu, compID );

  if( tu.mtsIdx[compID] == MTS_SKIP && !tu.cs->slice->tsResidualCodingDisabled )
  {
    residual_codingTS( tu, compID );
    return;
  }

  // determine sign hiding
  bool signHiding  = cu.cs->slice->signDataHidingEnabled;

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, signHiding );
  const TCoeff*       coeff   = tu.getCoeffs( compID ).buf;

  // determine and set last coeff position and sig group flags
  int                      scanPosLast = tu.lastPos[compID];
  std::bitset<MLS_GRP_NUM> sigGroupFlags;

  for( int subSetId = 0; subSetId <= ( scanPosLast >> cctx.log2CGSize() ); subSetId++ )
  {
    const int scanPosStart = subSetId << cctx.log2CGSize();
    const int scanPosEnd   = scanPosStart + ( 1 << cctx.log2CGSize() ) - 1;

    for( int scanPos = scanPosEnd; scanPos >= scanPosStart; scanPos-- )
    {
      unsigned blkPos = cctx.blockPos( scanPos );

      if( coeff[blkPos] )
      {
        sigGroupFlags.set( subSetId );
        break;
      }
    }
  }
  CHECK( scanPosLast < 0, "Coefficient coding called for empty TU" );
  cctx.setScanPosLast(scanPosLast);

  if( cuCtx && tu.mtsIdx[compID] != MTS_SKIP && tu.blocks[ compID ].height >= 4 && tu.blocks[ compID ].width >= 4 )
  {
    const int maxLfnstPos = ((tu.blocks[compID].height == 4 && tu.blocks[compID].width == 4) || (tu.blocks[compID].height == 8 && tu.blocks[compID].width == 8)) ? 7 : 15;
    cuCtx->violatesLfnstConstrained[ toChannelType(compID) ] |= cctx.scanPosLast() > maxLfnstPos;
  }
  if( cuCtx && tu.mtsIdx[compID] != MTS_SKIP && tu.blocks[ compID ].height >= 4 && tu.blocks[ compID ].width >= 4 )
  {
    const int lfnstLastScanPosTh = isLuma( compID ) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_CHROMA;
    cuCtx->lfnstLastScanPos |= cctx.scanPosLast() >= lfnstLastScanPosTh;
  }
  if (cuCtx && isLuma(compID) && tu.mtsIdx[compID] != MTS_SKIP)
  {
    cuCtx->mtsLastScanPos |= cctx.scanPosLast() >= 1;
  }
  
  // code last coeff position
  last_sig_coeff( cctx, tu, compID );

  // code subblocks
  const int stateTab  = ( tu.cs->slice->depQuantEnabled ? 32040 : 0 );
  int       state     = 0;

  int ctxBinSampleRatio = MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT;
  cctx.regBinLimit = (tu.getTbAreaAfterCoefZeroOut(compID) * ctxBinSampleRatio) >> 4;

  const bool zeroOutCheck  = isLuma( compID ) && tu.cs->sps->MTS && tu.cu->sbtInfo != 0 && tu.blocks[compID].height <= 32 && tu.blocks[compID].width <= 32;
  const bool zeroOutWidth  = tu.blocks[compID].width;
  const bool zeroOutHeight = tu.blocks[compID].height;

  for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
  {
    cctx.initSubblock( subSetId, sigGroupFlags[subSetId] );

    if( zeroOutCheck )
    {
      if( ( zeroOutHeight && cctx.cgPosY() >= ( 16 >> cctx.log2CGHeight() ) ) || ( zeroOutWidth  && cctx.cgPosX() >= ( 16 >> cctx.log2CGWidth() ) ) )
      {
        continue;
      }
    }

    residual_coding_subblock( cctx, coeff, stateTab, state );
    if ( cuCtx && isLuma(compID) && cctx.isSigGroup() && ( cctx.cgPosY() > 3 || cctx.cgPosX() > 3 ) )
    {
      cuCtx->violatesMtsCoeffConstraint = true;
    }
  }
}


void CABACWriter::ts_flag( const TransformUnit& tu, ComponentID compID )
{
  int tsFlag = tu.mtsIdx[compID] == MTS_SKIP ? 1 : 0;
  int ctxIdx = isLuma(compID) ? 0 : 1;
  
  if( TU::isTSAllowed ( tu, compID ) )
  {
    m_BinEncoder.encodeBin( tsFlag, Ctx::TransformSkipFlag(ctxIdx));
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "ts_flag() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMP_Y, tu.cu->lx(), tu.cu->ly(), tsFlag );
}


void CABACWriter::mts_idx( const CodingUnit& cu, CUCtx* cuCtx )
{
  TransformUnit &tu = *cu.firstTU;
  int        mtsIdx = tu.mtsIdx[COMP_Y];
  
  if( CU::isMTSAllowed( cu, COMP_Y ) && cuCtx && !cuCtx->violatesMtsCoeffConstraint &&
      cuCtx->mtsLastScanPos && cu.lfnstIdx == 0 && mtsIdx != MTS_SKIP)
  {
    int symbol = mtsIdx != MTS_DCT2_DCT2 ? 1 : 0;
    int ctxIdx = 0;
    
    m_BinEncoder.encodeBin( symbol, Ctx::MTSIdx(ctxIdx));
    
    if( symbol )
    {
      ctxIdx = 1;
      for( int i = 0; i < 3; i++, ctxIdx++ )
      {
        symbol = mtsIdx > i + MTS_DST7_DST7 ? 1 : 0;
        m_BinEncoder.encodeBin( symbol, Ctx::MTSIdx(ctxIdx));
        
        if( !symbol )
        {
          break;
        }
      }
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "mts_idx() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMP_Y, tu.cu->lx(), tu.cu->ly(), mtsIdx);
}


void CABACWriter::isp_mode( const CodingUnit& cu )
{
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.multiRefIdx || !cu.cs->sps->ISP || cu.bdpcmMode || !CU::canUseISP( cu, getFirstComponentOfChannel( cu.chType ) )  || cu.colorTransform)
  {
    CHECK( cu.ispMode != NOT_INTRA_SUBPARTITIONS, "cu.ispMode != 0" );
    return;
  }
  if ( cu.ispMode == NOT_INTRA_SUBPARTITIONS )
  {
    m_BinEncoder.encodeBin( 0, Ctx::ISPMode( 0 ) );
  }
  else
  {
    m_BinEncoder.encodeBin( 1, Ctx::ISPMode( 0 ) );
    m_BinEncoder.encodeBin( cu.ispMode - 1, Ctx::ISPMode( 1 ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_subPartitions() etype=%d pos=(%d,%d) ispIdx=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, (int)cu.ispMode );
}


void CABACWriter::residual_lfnst_mode( const CodingUnit& cu, CUCtx& cuCtx )
{
  int chIdx = CS::isDualITree( *cu.cs ) && cu.chType == CH_C ? 1 : 0;
  if( ( cu.ispMode && !CU::canUseLfnstWithISP( cu, cu.chType ) ) ||
      (cu.cs->sps->LFNST && CU::isIntra(cu) && cu.mipFlag && !allowLfnstWithMip(cu.lumaSize())) ||
    ( CU::isSepTree(cu) && cu.chType == CH_C && std::min( cu.blocks[ 1 ].width, cu.blocks[ 1 ].height ) < 4 )
    || ( cu.blocks[ chIdx ].lumaSize().width > cu.cs->sps->getMaxTbSize() || cu.blocks[ chIdx ].lumaSize().height > cu.cs->sps->getMaxTbSize() )
    )
  {
    return;
  }

  if( cu.cs->sps->LFNST && CU::isIntra( cu )  )
  {
    const bool lumaFlag                   = CU::isSepTree(cu) ? (   isLuma( cu.chType ) ? true : false ) : true;
    const bool chromaFlag                 = CU::isSepTree(cu) ? ( isChroma( cu.chType ) ? true : false ) : true;
          bool nonZeroCoeffNonTsCorner8x8 = ( lumaFlag && cuCtx.violatesLfnstConstrained[CH_L] ) || (chromaFlag && cuCtx.violatesLfnstConstrained[CH_C] );

    bool isTrSkip = false;

    cTUSecureTraverser trv( cu.firstTU, cu.lastTU );
    const auto* currTU = trv.begin;
    do
    {
      const uint32_t numValidComp = getNumberValidComponents(cu.chromaFormat);
      for (uint32_t compID = COMP_Y; compID < numValidComp; compID++)
      {
        if (currTU->blocks[compID].valid() && TU::getCbf(*currTU, (ComponentID)compID) && currTU->mtsIdx[compID] == MTS_SKIP)
        {
          isTrSkip = true;
          break;
        }
      }
    }
    while( currTU != trv.last && (0!=(currTU = currTU->next)));

    if( (!cuCtx.lfnstLastScanPos && !cu.ispMode) || nonZeroCoeffNonTsCorner8x8 || isTrSkip )
    {
      return;
    }
  }
  else
  {
    return;
  }
  
  unsigned cctx = 0;
  if ( CU::isSepTree(cu) ) cctx++;

  const uint32_t idxLFNST = cu.lfnstIdx;
  assert( idxLFNST < 3 );
  m_BinEncoder.encodeBin( idxLFNST ? 1 : 0, Ctx::LFNSTIdx( cctx ) );

  if( idxLFNST )
  {
    m_BinEncoder.encodeBin( (idxLFNST - 1) ? 1 : 0, Ctx::LFNSTIdx(2));
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "residual_lfnst_mode() etype=%d pos=(%d,%d) mode=%d\n", COMP_Y, cu.lx(), cu.ly(), ( int ) cu.lfnstIdx );
}


void CABACWriter::last_sig_coeff( CoeffCodingContext& cctx, const TransformUnit& tu, ComponentID compID )
{
  unsigned blkPos = cctx.blockPos( cctx.scanPosLast() );
  unsigned posX, posY;
  {
    posY  = blkPos / cctx.width();
    posX  = blkPos - ( posY * cctx.width() );
  }

  unsigned CtxLast;
  unsigned GroupIdxX = g_uiGroupIdx[ posX ];
  unsigned GroupIdxY = g_uiGroupIdx[ posY ];

  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();

  if( tu.cs->sps->MTS && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 && compID == COMP_Y )
  {
    maxLastPosX = ( tu.blocks[compID].width  == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosX;
    maxLastPosY = ( tu.blocks[compID].height == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosY;
  }

  for( CtxLast = 0; CtxLast < GroupIdxX; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastXCtxId( CtxLast ) );
  }
  if( GroupIdxX < maxLastPosX )
  {
    m_BinEncoder.encodeBin( 0, cctx.lastXCtxId( CtxLast ) );
  }
  for( CtxLast = 0; CtxLast < GroupIdxY; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastYCtxId( CtxLast ) );
  }
  if( GroupIdxY < maxLastPosY )
  {
    m_BinEncoder.encodeBin( 0, cctx.lastYCtxId( CtxLast ) );
  }
  if( GroupIdxX > 3 )
  {
    posX -= g_uiMinInGroup[ GroupIdxX ];
    for (int i = ( ( GroupIdxX - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posX >> i ) & 1 );
    }
  }
  if( GroupIdxY > 3 )
  {
    posY -= g_uiMinInGroup[ GroupIdxY ];
    for ( int i = ( ( GroupIdxY - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posY >> i ) & 1 );
    }
  }
}


void CABACWriter::residual_coding_subblock( CoeffCodingContext& cctx, const TCoeff* coeff, const int stateTransTable, int& state )
{
  //===== init =====
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
  int         firstSigPos = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );
  int         nextSigPos  = firstSigPos;

  //===== encode significant_coeffgroup_flag =====
  if( !isLast && cctx.isNotFirst() )
  {
    if( cctx.isSigGroup() )
    {
      m_BinEncoder.encodeBin( 1, cctx.sigGroupCtxId() );
    }
    else
    {
      m_BinEncoder.encodeBin( 0, cctx.sigGroupCtxId() );
      return;
    }
  }

  uint8_t   ctxOffset[16];

  //===== encode absolute values =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
  int       firstNZPos    = nextSigPos;
  int       lastNZPos     = -1;
  int       remAbsLevel   = -1;
  int       numNonZero    =  0;
  unsigned  signPattern   =  0;
  int       remRegBins    = cctx.regBinLimit;
  int       firstPosMode2 = minSubPos - 1;

  for( ; nextSigPos >= minSubPos && remRegBins >= 4; nextSigPos-- )
  {
    TCoeff    Coeff      = coeff[ cctx.blockPos( nextSigPos ) ];
    unsigned  sigFlag    = ( Coeff != 0 );
    if( numNonZero || nextSigPos != inferSigPos )
    {
      const unsigned sigCtxId = cctx.sigCtxIdAbs( nextSigPos, coeff, state );
      m_BinEncoder.encodeBin( sigFlag, sigCtxId );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      remRegBins--;
    }
    else if( nextSigPos != cctx.scanPosLast() )
    {
      cctx.sigCtxIdAbs( nextSigPos, coeff, state ); // required for setting variables that are needed for gtx/par context selection
    }

    if( sigFlag )
    {
      uint8_t&  ctxOff  = ctxOffset[ nextSigPos - minSubPos ];
      ctxOff            = cctx.ctxOffsetAbs();
      numNonZero++;
      firstNZPos  = nextSigPos;
      lastNZPos   = std::max<int>( lastNZPos, nextSigPos );
      remAbsLevel = abs( Coeff ) - 1;

      if( nextSigPos != cctx.scanPosLast() ) signPattern <<= 1;
      if( Coeff < 0 )                        signPattern++;

      unsigned gt1 = !!remAbsLevel;
      m_BinEncoder.encodeBin( gt1, cctx.greater1CtxIdAbs(ctxOff) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", gt1, cctx.greater1CtxIdAbs(ctxOff) );
      remRegBins--;

      if( gt1 )
      {
        remAbsLevel  -= 1;
        m_BinEncoder.encodeBin( remAbsLevel&1, cctx.parityCtxIdAbs( ctxOff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d ctx=%d\n", remAbsLevel&1, cctx.parityCtxIdAbs( ctxOff ) );
        remAbsLevel >>= 1;

        remRegBins--;
        unsigned gt2 = !!remAbsLevel;
        m_BinEncoder.encodeBin(gt2, cctx.greater2CtxIdAbs(ctxOff));
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", gt2, cctx.greater2CtxIdAbs(ctxOff));
        remRegBins--;
      }
    }

    state = ( stateTransTable >> ((state<<2)+((Coeff&1)<<1)) ) & 3;
  }
  firstPosMode2 = nextSigPos;
  cctx.regBinLimit = remRegBins;


  //===== 2nd PASS: Go-rice codes =====
  for( int scanPos = firstSigPos; scanPos > firstPosMode2; scanPos-- )
  {
    unsigned absLevel = abs( coeff[cctx.blockPos( scanPos )] );

    if( absLevel >= 4 )
    {
      int      sumAll   = cctx.templateAbsSum( scanPos, coeff, 4 );
      unsigned ricePar  = g_auiGoRiceParsCoeff[sumAll];
      unsigned rem      = ( absLevel - 4 ) >> 1;
      m_BinEncoder.encodeRemAbsEP( rem, ricePar, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar );
    }
  }

  //===== coeff bypass ====
  for( int scanPos = firstPosMode2; scanPos >= minSubPos; scanPos-- )
  {
    TCoeff    Coeff     = coeff[ cctx.blockPos( scanPos ) ];
    unsigned  absLevel  = abs( Coeff );
    int       sumAll    = cctx.templateAbsSum(scanPos, coeff, 0);
    int       rice      = g_auiGoRiceParsCoeff[sumAll];
    int       pos0      = g_auiGoRicePosCoeff0(state, rice);
    unsigned  rem       = ( absLevel == 0 ? pos0 : absLevel <= pos0 ? absLevel-1 : absLevel );
    m_BinEncoder.encodeRemAbsEP( rem, rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice );
    state = ( stateTransTable >> ((state<<2)+((absLevel&1)<<1)) ) & 3;
    if( absLevel )
    {
      numNonZero++;
      firstNZPos = scanPos;
      lastNZPos   = std::max<int>( lastNZPos, scanPos );
      signPattern <<= 1;
      if( Coeff < 0 ) signPattern++;
    }
  }

  //===== encode sign's =====
  unsigned numSigns = numNonZero;
  if( cctx.hideSign( firstNZPos, lastNZPos ) )
  {
    numSigns    --;
    signPattern >>= 1;
  }
  m_BinEncoder.encodeBinsEP( signPattern, numSigns );
}


void CABACWriter::residual_codingTS( const TransformUnit& tu, ComponentID compID )
{
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_codingTS() etype=%d pos=(%d,%d) size=%dx%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height );

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, false, isLuma(compID) ? tu.cu->bdpcmMode : tu.cu->bdpcmModeChroma);
  const TCoeff*       coeff   = tu.getCoeffs( compID ).buf;
  int maxCtxBins = (cctx.maxNumCoeff() * 7) >> 2;
  cctx.setNumCtxBins(maxCtxBins);

  // determine and set last coeff position and sig group flags
  std::bitset<MLS_GRP_NUM> sigGroupFlags;
  for( int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
  {
    unsigned blkPos = cctx.blockPos( scanPos );
    if( coeff[blkPos] )
    {
      sigGroupFlags.set( scanPos >> cctx.log2CGSize() );
    }
  }

  // code subblocks
  for( int subSetId = 0; subSetId <= ( cctx.maxNumCoeff() - 1 ) >> cctx.log2CGSize(); subSetId++ )
  {
    cctx.initSubblock         ( subSetId, sigGroupFlags[subSetId] );
    residual_coding_subblockTS( cctx, coeff );
  }
}


void CABACWriter::residual_coding_subblockTS( CoeffCodingContext& cctx, const TCoeff* coeff )
{
  //===== init =====
  const int   minSubPos   = cctx.maxSubPos();
  int         firstSigPos = cctx.minSubPos();
  int         nextSigPos  = firstSigPos;

  //===== encode significant_coeffgroup_flag =====
  if( !cctx.isLastSubSet() || !cctx.only1stSigGroup() )
  {
    if( cctx.isSigGroup() )
    {
        m_BinEncoder.encodeBin( 1, cctx.sigGroupCtxId( true ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", 1, cctx.sigGroupCtxId() );
    }
    else
    {
        m_BinEncoder.encodeBin( 0, cctx.sigGroupCtxId( true ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", 0, cctx.sigGroupCtxId() );
      return;
    }
  }

  //===== encode absolute values =====
  const int inferSigPos   = minSubPos;
  int       remAbsLevel   = -1;
  int       numNonZero    =  0;

  int rightPixel, belowPixel, modAbsCoeff;

  int lastScanPosPass1 = -1;
  int lastScanPosPass2 = -1;
  for (; nextSigPos <= minSubPos && cctx.numCtxBins() >= 4; nextSigPos++)
  {
    TCoeff    Coeff      = coeff[ cctx.blockPos( nextSigPos ) ];
    unsigned  sigFlag    = ( Coeff != 0 );
    if( numNonZero || nextSigPos != inferSigPos )
    {
      const unsigned sigCtxId = cctx.sigCtxIdAbsTS( nextSigPos, coeff );
      m_BinEncoder.encodeBin( sigFlag, sigCtxId );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      cctx.decimateNumCtxBins(1);
    }

    if( sigFlag )
    {
      //===== encode sign's =====
      int sign = Coeff < 0;
      const unsigned signCtxId = cctx.signCtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      m_BinEncoder.encodeBin(sign, signCtxId);
      cctx.decimateNumCtxBins(1);
      numNonZero++;
      cctx.neighTS(rightPixel, belowPixel, nextSigPos, coeff);
      modAbsCoeff = cctx.deriveModCoeff(rightPixel, belowPixel, abs(Coeff), cctx.bdpcm());
      remAbsLevel = modAbsCoeff - 1;

      unsigned gt1 = !!remAbsLevel;
      const unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      m_BinEncoder.encodeBin(gt1, gt1CtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1, gt1CtxId);
      cctx.decimateNumCtxBins(1);

      if( gt1 )
      {
        remAbsLevel  -= 1;
        m_BinEncoder.encodeBin( remAbsLevel&1, cctx.parityCtxIdAbsTS() );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() bin=%d ctx=%d\n", remAbsLevel&1, cctx.parityCtxIdAbsTS() );
        cctx.decimateNumCtxBins(1);
      }
    }
    lastScanPosPass1 = nextSigPos;
  }

  int cutoffVal = 2;
  int numGtBins = 4;

  for (int scanPos = firstSigPos; scanPos <= minSubPos && cctx.numCtxBins() >= 4; scanPos++)
  {
    unsigned absLevel;
    cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
    absLevel = cctx.deriveModCoeff(rightPixel, belowPixel, abs(coeff[cctx.blockPos(scanPos)]), cctx.bdpcm());
    cutoffVal = 2;
    for (int i = 0; i < numGtBins; i++)
    {
      if (absLevel >= cutoffVal)
      {
        unsigned gt2 = (absLevel >= (cutoffVal + 2));
        m_BinEncoder.encodeBin(gt2, cctx.greaterXCtxIdAbsTS(cutoffVal >> 1));
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i, gt2, cctx.greaterXCtxIdAbsTS(cutoffVal >> 1), scanPos, std::min<int>(absLevel, cutoffVal + 2));
        cctx.decimateNumCtxBins(1);
      }
      cutoffVal += 2;
    }
    lastScanPosPass2 = scanPos;
  }

  //===== coeff bypass ====
  for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
  {
    unsigned absLevel;
    cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
    cutoffVal = (scanPos <= lastScanPosPass2 ? 10 : (scanPos <= lastScanPosPass1 ? 2 : 0));
    absLevel = cctx.deriveModCoeff(rightPixel, belowPixel, abs(coeff[cctx.blockPos(scanPos)]), cctx.bdpcm()||!cutoffVal);
    if( absLevel >= cutoffVal )
    {
      int       rice = cctx.templateAbsSumTS( scanPos, coeff );
      unsigned  rem = scanPos <= lastScanPosPass1 ? (absLevel - cutoffVal) >> 1 : absLevel;
      m_BinEncoder.encodeRemAbsEP( rem, rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_rem_val() bin=%d ctx=%d sp=%d\n", rem, rice, scanPos );

      if (absLevel && scanPos > lastScanPosPass1)
      {
        int sign = coeff[cctx.blockPos(scanPos)] < 0;
        m_BinEncoder.encodeBinEP(sign);
      }
    }
  }
}


//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    void  unary_max_symbol  ( symbol, ctxId0, ctxIdN, maxSymbol )
//    void  unary_max_eqprob  ( symbol,                 maxSymbol )
//    void  exp_golomb_eqprob ( symbol, count )
//================================================================================

void CABACWriter::unary_max_symbol( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol )
{
  CHECK( symbol > maxSymbol, "symbol > maxSymbol" );
  const unsigned totalBinsToWrite = std::min( symbol + 1, maxSymbol );
  for( unsigned binsWritten = 0; binsWritten < totalBinsToWrite; ++binsWritten )
  {
    const unsigned nextBin = symbol > binsWritten;
    m_BinEncoder.encodeBin( nextBin, binsWritten == 0 ? ctxId0 : ctxIdN );
  }
}


void CABACWriter::unary_max_eqprob( unsigned symbol, unsigned maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return;
  }
  bool     codeLast = ( maxSymbol > symbol );
  unsigned bins     = 0;
  unsigned numBins  = 0;
  while( symbol-- )
  {
    bins   <<= 1;
    bins   ++;
    numBins++;
  }
  if( codeLast )
  {
    bins  <<= 1;
    numBins++;
  }
  CHECK(!( numBins <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP( bins, numBins );
}


void CABACWriter::exp_golomb_eqprob( unsigned symbol, unsigned count )
{
  unsigned bins    = 0;
  unsigned numBins = 0;
  while( symbol >= (unsigned)(1<<count) )
  {
    bins <<= 1;
    bins++;
    numBins++;
    symbol -= 1 << count;
    count++;
  }
  bins <<= 1;
  numBins++;
  bins = (bins << count) | symbol;
  numBins += count;
  CHECK(!( numBins <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP( bins, numBins );
}


void CABACWriter::codeAlfCtuEnabled( CodingStructure& cs, ChannelType channel, AlfParam* alfParam)
{
  if( isLuma( channel ) )
  {
    if (alfParam->alfEnabled[COMP_Y])
      codeAlfCtuEnabled( cs, COMP_Y, alfParam );
  }
  else
  {
    if (alfParam->alfEnabled[COMP_Cb])
      codeAlfCtuEnabled( cs, COMP_Cb, alfParam );
    if (alfParam->alfEnabled[COMP_Cr])
      codeAlfCtuEnabled( cs, COMP_Cr, alfParam );
  }
}


void CABACWriter::codeAlfCtuEnabled( CodingStructure& cs, ComponentID compID, AlfParam* alfParam)
{
  uint32_t numCTUs = cs.pcv->sizeInCtus;

  for( int ctuIdx = 0; ctuIdx < numCTUs; ctuIdx++ )
  {
    codeAlfCtuEnabledFlag( cs, ctuIdx, compID, alfParam );
  }
}


void CABACWriter::codeAlfCtuEnabledFlag( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, AlfParam* alfParam)
{
  const bool alfComponentEnabled = (alfParam != NULL) ? alfParam->alfEnabled[compIdx] : cs.slice->tileGroupAlfEnabled[compIdx];

  if( cs.sps->alfEnabled && alfComponentEnabled )
  {
    const PreCalcValues& pcv = *cs.pcv;
    int                 frame_width_in_ctus = pcv.widthInCtus;
    int                 ry = ctuRsAddr / frame_width_in_ctus;
    int                 rx = ctuRsAddr - ry * frame_width_in_ctus;
    const Position      pos( rx * cs.pcv->maxCUSize, ry * cs.pcv->maxCUSize );
    const uint32_t          curSliceIdx = cs.slice->independentSliceIdx;
    const uint32_t      curTileIdx = 0;//cs.picture->brickMap->getBrickIdxRsMap( pos );
    bool                leftAvail = cs.getCURestricted( pos.offset( -(int)pcv.maxCUSize, 0 ), pos, curSliceIdx, curTileIdx, CH_L, TREE_D ) ? true : false;
    bool                aboveAvail = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUSize ), pos, curSliceIdx, curTileIdx, CH_L, TREE_D ) ? true : false;

    int leftCTUAddr = leftAvail ? ctuRsAddr - 1 : -1;
    int aboveCTUAddr = aboveAvail ? ctuRsAddr - frame_width_in_ctus : -1;

    uint8_t* ctbAlfFlag = cs.slice->pic->getAlfCtuEnabled( compIdx );
    int ctx = 0;
    ctx += leftCTUAddr > -1 ? ( ctbAlfFlag[leftCTUAddr] ? 1 : 0 ) : 0;
    ctx += aboveCTUAddr > -1 ? ( ctbAlfFlag[aboveCTUAddr] ? 1 : 0 ) : 0;
    m_BinEncoder.encodeBin( ctbAlfFlag[ctuRsAddr], Ctx::ctbAlfFlag( compIdx * 3 + ctx ) );
  }
}


void CABACWriter::codeCcAlfFilterControlIdc(uint8_t idcVal, CodingStructure &cs, const ComponentID compID,
                                            const int curIdx, const uint8_t *filterControlIdc, Position lumaPos,
                                            const int filterCount)
{
  CHECK(idcVal > filterCount, "Filter index is too large");

  const uint32_t curSliceIdx    = cs.slice->independentSliceIdx;
  const uint32_t curTileIdx     = cs.pps->getTileIdx( lumaPos );
  Position       leftLumaPos    = lumaPos.offset(-(int)cs.pcv->maxCUSize, 0);
  Position       aboveLumaPos   = lumaPos.offset(0, -(int)cs.pcv->maxCUSize);
  bool           leftAvail      = cs.getCURestricted( leftLumaPos,  lumaPos, curSliceIdx, curTileIdx, CH_L, TREE_D ) ? true : false;
  bool           aboveAvail     = cs.getCURestricted( aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, CH_L, TREE_D ) ? true : false;
  int            ctxt           = 0;

  if (leftAvail)
  {
    ctxt += ( filterControlIdc[curIdx - 1]) ? 1 : 0;
  }
  if (aboveAvail)
  {
    ctxt += (filterControlIdc[curIdx - cs.pcv->widthInCtus]) ? 1 : 0;
  }
  ctxt += ( compID == COMP_Cr ) ? 3 : 0;

  m_BinEncoder.encodeBin( ( idcVal == 0 ) ? 0 : 1, Ctx::CcAlfFilterControlFlag( ctxt ) ); // ON/OFF flag is context coded
  if ( idcVal > 0 )
  {
    int val = (idcVal - 1);
    while ( val )
    {
      m_BinEncoder.encodeBinEP( 1 );
      val--;
    }
    if ( idcVal < filterCount )
    {
      m_BinEncoder.encodeBinEP( 0 );
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "ccAlfFilterControlIdc() compID=%d pos=(%d,%d) ctxt=%d, filterCount=%d, idcVal=%d\n", compID, lumaPos.x, lumaPos.y, ctxt, filterCount, idcVal );
}


void CABACWriter::mip_flag( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }
  if( !cu.cs->sps->MIP )
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxMipFlag( cu );
  m_BinEncoder.encodeBin( cu.mipFlag, Ctx::MipFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.mipFlag ? 1 : 0 );
}


void CABACWriter::mip_pred_modes( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }

  mip_pred_mode( cu );
}


void CABACWriter::mip_pred_mode( const CodingUnit& cu )
{
  m_BinEncoder.encodeBinEP( (cu.mipTransposedFlag ? 1 : 0) );

  const int numModes = getNumModesMip( cu.Y() );
  CHECKD( cu.intraDir[CH_L] < 0 || cu.intraDir[CH_L] >= numModes, "Invalid MIP mode" );
  xWriteTruncBinCode( cu.intraDir[CH_L], numModes );

  DTRACE( g_trace_ctx, D_SYNTAX, "mip_pred_mode() pos=(%d,%d) mode=%d transposed=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.intraDir[CH_L], cu.mipTransposedFlag ? 1 : 0 );
}


void CABACWriter::codeAlfCtuFilterIndex(CodingStructure& cs, uint32_t ctuRsAddr, bool alfEnableLuma)
{
  if ( (!cs.sps->alfEnabled) || (!alfEnableLuma))
  {
    return;
  }

  uint8_t* ctbAlfFlag = cs.slice->pic->getAlfCtuEnabled(COMP_Y);
  if (!ctbAlfFlag[ctuRsAddr])
  {
    return;
  }

  short* alfCtbFilterIndex = cs.slice->pic->getAlfCtbFilterIndex();
  const unsigned filterSetIdx = alfCtbFilterIndex[ctuRsAddr];
  unsigned numAps = cs.slice->tileGroupNumAps;
  unsigned numAvailableFiltSets = numAps + NUM_FIXED_FILTER_SETS;
  if (numAvailableFiltSets > NUM_FIXED_FILTER_SETS)
  {
    int useTemporalFilt = (filterSetIdx >= NUM_FIXED_FILTER_SETS) ? 1 : 0;
    m_BinEncoder.encodeBin(useTemporalFilt, Ctx::AlfUseTemporalFilt());
    if (useTemporalFilt)
    {
      CHECK((filterSetIdx - NUM_FIXED_FILTER_SETS) >= (numAvailableFiltSets - NUM_FIXED_FILTER_SETS), "temporal non-latest set");
      if (numAps > 1)
      {
        xWriteTruncBinCode(filterSetIdx - NUM_FIXED_FILTER_SETS, numAvailableFiltSets - NUM_FIXED_FILTER_SETS);
      }
    }
    else
    {
      CHECK(filterSetIdx >= NUM_FIXED_FILTER_SETS, "fixed set larger than temporal");
      xWriteTruncBinCode(filterSetIdx, NUM_FIXED_FILTER_SETS);
    }
  }
  else
  {
    CHECK(filterSetIdx >= NUM_FIXED_FILTER_SETS, "fixed set numavail < num_fixed");
    xWriteTruncBinCode(filterSetIdx, NUM_FIXED_FILTER_SETS);
  }
}


void CABACWriter::codeAlfCtuAlternatives( CodingStructure& cs, ChannelType channel, AlfParam* alfParam)
{
  if( isChroma( channel ) )
  {
    if (alfParam->alfEnabled[COMP_Cb])
      codeAlfCtuAlternatives( cs, COMP_Cb, alfParam );
    if (alfParam->alfEnabled[COMP_Cr])
      codeAlfCtuAlternatives( cs, COMP_Cr, alfParam );
  }
}


void CABACWriter::codeAlfCtuAlternatives( CodingStructure& cs, ComponentID compID, AlfParam* alfParam)
{
  if( compID == COMP_Y )
    return;
  uint32_t numCTUs = cs.pcv->sizeInCtus;
  uint8_t* ctbAlfFlag = cs.slice->pic->getAlfCtuEnabled( compID );

  for( int ctuIdx = 0; ctuIdx < numCTUs; ctuIdx++ )
  {
    if( ctbAlfFlag[ctuIdx] )
    {
      codeAlfCtuAlternative( cs, ctuIdx, compID, alfParam );
    }
  }
}


void CABACWriter::codeAlfCtuAlternative( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, const AlfParam* alfParam)
{
  if( compIdx == COMP_Y )
    return;
  int apsIdx = alfParam ? 0 : cs.slice->tileGroupChromaApsId;
  const AlfParam& alfParamRef = alfParam ? (*alfParam) : cs.slice->alfAps[apsIdx]->alfParam;

  if( alfParam || (cs.sps->alfEnabled && cs.slice->tileGroupAlfEnabled[compIdx]) )
  {
    uint8_t* ctbAlfFlag = cs.slice->pic->getAlfCtuEnabled( compIdx );

    if( ctbAlfFlag[ctuRsAddr] )
    {
      const int numAlts = alfParamRef.numAlternativesChroma;
      uint8_t* ctbAlfAlternative = cs.slice->pic->getAlfCtuAlternativeData( compIdx );
      unsigned numOnes = ctbAlfAlternative[ctuRsAddr];
      assert( ctbAlfAlternative[ctuRsAddr] < numAlts );
      for( int i = 0; i < numOnes; ++i )
        m_BinEncoder.encodeBin( 1, Ctx::ctbAlfAlternative( compIdx-1 ) );
      if( numOnes < numAlts-1 )
        m_BinEncoder.encodeBin( 0, Ctx::ctbAlfAlternative( compIdx-1 ) );
    }
  }
}

} // namespace vvenc

//! \}


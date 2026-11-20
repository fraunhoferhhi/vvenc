/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/** \file     EncModeCtrl.cpp
    \brief    Encoder controller for trying out specific modes
*/

#include "EncCu.h"

#include "EncModeCtrl.h"
#include "CommonLib/RdCost.h"
#include "CommonLib/CodingStructure.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_next.h"

#include <algorithm>
#include <climits>
#include <cmath>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

void CacheBlkInfoCtrl::create(int ctuSize)
{
  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;
  const int maxSizeIdx  = MAX_CU_SIZE_IDX - MIN_CU_LOG2;

  //static constexpr size_t numCu = 7921;

  size_t numCu = 0;
  
  for( int wIdx = 0; wIdx < maxSizeIdx; wIdx++ )
  {
    for( int hIdx = 0; hIdx < maxSizeIdx; hIdx++ )
    {
      for( unsigned y = 0; y < numPos; y++ )
      {
        for( unsigned x = 0; x < numPos; x++ )
        {
          // a block of width W might be offset of N * W + 1/2 W (bcs of TT), same for H
          // W = 1 << ( wIdx + 2 )
          // 1/2 W = 1 << ( wIdx + 1 )
          // remainder of (N+1/2)*W -> x & ( ( 1 << ( wIdx + 1 ) ) - 1 )
  
          if( (x + (1 << (wIdx)) <= (ctuSize >> MIN_CU_LOG2))
              && (y + (1 << (hIdx)) <= (ctuSize >> MIN_CU_LOG2))
              && (((x << MIN_CU_LOG2) & ((1 << (wIdx + MIN_CU_LOG2 - 1)) - 1)) == 0)
              && (((y << MIN_CU_LOG2) & ((1 << (hIdx + MIN_CU_LOG2 - 1)) - 1)) == 0) )
          {
            numCu++;
          }
        }
      }
    }
  }

  m_codedCUInfoBuf = new CodedCUInfo[numCu];
  CodedCUInfo* cuInfo = m_codedCUInfoBuf;

  for( int wIdx = 0; wIdx < maxSizeIdx; wIdx++ )
  {
    for( int hIdx = 0; hIdx < maxSizeIdx; hIdx++ )
    {
      for( unsigned y = 0; y < numPos; y++ )
      {
        for( unsigned x = 0; x < numPos; x++ )
        {
          // a block of width W might be offset of N * W + 1/2 W (bcs of TT), same for H
          // W = 1 << ( wIdx + 2 )
          // 1/2 W = 1 << ( wIdx + 1 )
          // remainder of (N+1/2)*W -> x & ( ( 1 << ( wIdx + 1 ) ) - 1 )

          if(( x + (1<<(wIdx)) <= ( ctuSize >> MIN_CU_LOG2 ) )
            && ( y + (1<<(hIdx)) <= ( ctuSize >> MIN_CU_LOG2 ) )
            && ( ( ( x << MIN_CU_LOG2 ) & ((1 << (wIdx + MIN_CU_LOG2 - 1)) - 1) ) == 0 )
            && ( ( ( y << MIN_CU_LOG2 ) & ((1 << (hIdx + MIN_CU_LOG2 - 1)) - 1) ) == 0 ) )
          {
            m_codedCUInfo[wIdx][hIdx][x][y] = cuInfo++;
            m_codedCUInfo[wIdx][hIdx][x][y]->poc       = -1;
            m_codedCUInfo[wIdx][hIdx][x][y]->ctuRsAddr = -1;
          }
          else
          {
            m_codedCUInfo[wIdx][hIdx][x][y] = nullptr;
          }
        }
      }
    }
  }
}

void CacheBlkInfoCtrl::destroy()
{
  delete[] m_codedCUInfoBuf;
  m_codedCUInfoBuf = nullptr;
}

void CacheBlkInfoCtrl::init( const Slice &slice )
{
  m_pcv = slice.pps->pcv;
}

void CacheBlkInfoCtrl::initBlk( const UnitArea& area, int poc )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew( area.Y(), *m_pcv, idx1, idx2, idx3, idx4 );

  const int ctuRsAddr = getCtuAddr( area.lumaPos(), *m_pcv );
  CodedCUInfo* cuInfo = m_codedCUInfo[idx1][idx2][idx3][idx4];

  if( cuInfo->poc != poc || cuInfo->ctuRsAddr != ctuRsAddr )
  {
    GCC_WARNING_DISABLE_class_memaccess
    memset( cuInfo, 0, sizeof( CodedCUInfo ) );
    GCC_WARNING_RESET

    cuInfo->poc       = poc;
    cuInfo->ctuRsAddr = ctuRsAddr;
  }
}

CodedCUInfo& CacheBlkInfoCtrl::getBlkInfo( const UnitArea& area )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew( area.Y(), *m_pcv, idx1, idx2, idx3, idx4 );
//  DTRACE( g_trace_ctx, D_TMP, "%d loc %d %d %d %d\n", g_trace_ctx->getChannelCounter(D_TMP), idx1, idx2, idx3, idx4);
  return *m_codedCUInfo[idx1][idx2][idx3][idx4];
}

void CodedCUInfo::setMv( const RefPicList refPicList, const int iRefIdx, const Mv& rMv )
{
  if( iRefIdx >= MAX_STORED_CU_INFO_REFS ) return;

  saveMv [refPicList][iRefIdx] = rMv;
  validMv[refPicList][iRefIdx] = true;
}

bool CodedCUInfo::getMv( const RefPicList refPicList, const int iRefIdx, Mv& rMv ) const
{
  if( iRefIdx >= MAX_STORED_CU_INFO_REFS )
  {
    rMv = saveMv[refPicList][0];
    return false;
  }

  rMv = saveMv[refPicList][iRefIdx];
  return validMv[refPicList][iRefIdx];
}

uint8_t CacheBlkInfoCtrl::findBestSbt( const UnitArea& area, const uint32_t curPuSse )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew( area.Y(), *m_pcv, idx1, idx2, idx3, idx4 );
  CodedCUInfo* pSbtSave = m_codedCUInfo[idx1][idx2][idx3][idx4];

  for( int i = 0; i < pSbtSave->numPuInfoStored; i++ )
  {
    if( curPuSse == pSbtSave->puSse[i] )
    {
      return pSbtSave->puSbt[i];
    }
  }

  return MAX_UCHAR;
}

bool CacheBlkInfoCtrl::saveBestSbt( const UnitArea& area, const uint32_t curPuSse, const uint8_t curPuSbt )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew( area.Y(), *m_pcv, idx1, idx2, idx3, idx4 );
  CodedCUInfo* pSbtSave = m_codedCUInfo[idx1][idx2][idx3][idx4];

  if( pSbtSave->numPuInfoStored == SBT_NUM_SL )
  {
    return false;
  }

  pSbtSave->puSse[pSbtSave->numPuInfoStored] = curPuSse;
  pSbtSave->puSbt[pSbtSave->numPuInfoStored] = curPuSbt;
  pSbtSave->numPuInfoStored++;

  return true;
}

// sharedLevels: the number of leading levels of the split path (the CTU counts as one) whose nodes have been searched
// only once since the result was stored; INT_MAX where that holds for the whole path (BestEncInfoCache::xSharedLevels)
static bool isTheSameNbHood( const CodingUnit &cu, const CodingStructure& cs, const Partitioner &partitioner, int picW, int picH, const int sharedLevels )
{
  if( cu.chType != partitioner.chType )
  {
    return false;
  }

  const PartitioningStack &ps = partitioner.getPartStack();

  int i = 1;

  for( ; i < ps.size(); i++ )
  {
    if( ps[i].split != CU::getSplitAtDepth( cu, i - 1 ) )
    {
      break;
    }
  }

  // A node searched a second time may by now have other blocks coded before it, also where the split path to the
  // stored result is the same: only the neighbours outside the deepest node searched once are still the same.
  i = std::min( i, sharedLevels );

  const UnitArea& cmnAnc = ps[i - 1].parts[ps[i - 1].idx];
  const UnitArea cuArea  = CS::getArea( cs, cu, partitioner.chType, partitioner.treeType );

  for( int i = 0; i < cmnAnc.blocks.size(); i++ )
  {
    if( i < cuArea.blocks.size() && cuArea.blocks[i].valid() && cuArea.blocks[i].pos() != cmnAnc.blocks[i].pos() )
    {
      return false;
    }
  }

  return true;
}

void BestEncInfoCache::create( const bool reuseCuResults, const ChromaFormat chFmt, const int ctuSize )
{
  m_reuseCuResults = reuseCuResults;

  if( !m_reuseCuResults ) return;

  m_chFmt = chFmt;
  std::fill_n( &m_bestEncInfo[0][0][0][0], sizeof( m_bestEncInfo ) / sizeof( m_bestEncInfo[0][0][0][0] ), nullptr );
}

void BestEncInfoCache::destroy()
{
  if( !m_reuseCuResults ) return;

  std::fill_n( &m_bestEncInfo[0][0][0][0], sizeof( m_bestEncInfo ) / sizeof( m_bestEncInfo[0][0][0][0] ), nullptr );
  m_filledSlots.clear();
  m_entryArena.release();
  m_coeffArena.release();
  m_dmvrMvArena.release();

  m_pcv = nullptr;
}

void BestEncInfoCache::init( const Slice &slice )
{
  if( !m_reuseCuResults ) return;

  // A new CTU starts, and no result of the previous one can match its CUs.
  for( BestEncodingInfo** slot : m_filledSlots )
  {
    *slot = nullptr;
  }
  m_filledSlots.clear();
  m_entryArena.reset();
  m_coeffArena.reset();
  m_dmvrMvArena.reset();
  m_storeCount        = 0;
  m_revisitFirstStore = 0;

  bool isInitialized = m_pcv;

  m_pcv = slice.pps->pcv;

  if( isInitialized ) return;

  m_dummyCS.pcv = m_pcv;
}

// Takes storage for the result of a CU with size indices wIdx and hIdx, set up the way the earlier
// fixed table of results was set up when it was created.
BestEncodingInfo* BestEncInfoCache::xNewEntry( const int wIdx, const int hIdx )
{
  BestEncodingInfo* encInfo = m_entryArena.take( 1 );
  encInfo->~BestEncodingInfo();
  new( encInfo ) BestEncodingInfo();

  const UnitArea area( m_chFmt, Area( 0, 0, 1 << ( wIdx + MIN_CU_LOG2 ), 1 << ( hIdx + MIN_CU_LOG2 ) ) );

  encInfo->cu.UnitArea::operator=( area );
  encInfo->tu.UnitArea::operator=( area );

  encInfo->cu.chType   = CH_L;
  encInfo->cu.treeType = TREE_D;
  encInfo->cu.modeType = MODE_TYPE_ALL;
  encInfo->cu.qp       = MAX_SCHAR;
  encInfo->tu.chType   = CH_L;

  if( hIdx >= 1 && wIdx >= 1 && ( wIdx + hIdx ) >= 3 )
  {
    const int dmvrSize = ( 1 << std::max( 0, ( wIdx + MIN_CU_LOG2 - DMVR_SUBCU_SIZE_LOG2 ) ) ) * ( 1 << std::max( 0, ( hIdx + MIN_CU_LOG2 - DMVR_SUBCU_SIZE_LOG2 ) ) );
    encInfo->cu.mvdL0SubPu = m_dmvrMvArena.take( dmvrSize );
  }

  TCoeffSig* coeff[MAX_NUM_TBLOCKS] = { 0, };
  for( int i = 0; i < area.blocks.size(); i++ )
  {
    coeff[i] = m_coeffArena.take( area.blocks[i].area() );
  }

  encInfo->tu.cs = &m_dummyCS;
  encInfo->tu.init( coeff );

  encInfo->poc      = -1;
  encInfo->testMode = EncTestMode();
  return encInfo;
}

bool BestEncInfoCache::setFromCs( const CodingStructure& cs, const EncTestMode& testMode, const Partitioner& partitioner )
{
  if( !m_reuseCuResults ) return false;

  if( cs.cus.size() != 1 || cs.tus.size() != 1 || partitioner.maxBTD <= 1 )
  {
    return false;
  }

  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew( cs.area.Y(), *m_pcv, idx1, idx2, idx3, idx4 );

  BestEncodingInfo*& slot = m_bestEncInfo[idx1][idx2][idx3][idx4];
  if( !slot )
  {
    slot = xNewEntry( idx1, idx2 );
    m_filledSlots.push_back( &slot );
  }
  BestEncodingInfo& encInfo = *slot;

  encInfo.poc            =  cs.picture->poc;
  encInfo.cu.UnitArea::operator= ( *cs.cus.front() );
  encInfo.tu.UnitArea::operator= ( *cs.tus.front() );
  encInfo.cu             = *cs.cus.front();
  for( auto &blk : cs.tus.front()->blocks )
  {
    if( blk.valid() ) encInfo.tu.copyComponentFrom( *cs.tus.front(), blk.compID );
  }
  encInfo.testMode       = testMode;
  encInfo.dist           = cs.dist;
  encInfo.costEDO        = cs.costDbOffset;
  encInfo.storeNumber    = ++m_storeCount;

  return true;
}

// How far down the split path a stored result can share its neighbourhood with the CU coded now (isTheSameNbHood)
int BestEncInfoCache::xSharedLevels( const BestEncodingInfo& encInfo ) const
{
  // never true while m_revisitFirstStore is 0: store numbers start at 1
  const bool storedBeforeRevisit = encInfo.storeNumber <= m_revisitFirstStore;
  return storedBeforeRevisit ? m_revisitSharedLevels : INT_MAX;
}

// FastMTTRefine searches some CUs as a picture with a lower MTT depth limit would (mttDepthLimit), and such a picture
// reuses nothing where that limit is 1. Results are still stored with the partitioner's limit (setFromCs): that changes
// nothing for a CU that reuses nothing, and the second pass, which searches with the partitioner's limit, can then
// reuse what the first pass found.
bool BestEncInfoCache::isReusingCuValid( const CodingStructure& cs, const Partitioner& partitioner, int qp, const unsigned mttDepthLimit )
{
  if( !m_reuseCuResults || partitioner.treeType == TREE_C || mttDepthLimit <= 1 || partitioner.currQgEnable() )
  {
    return false; //if save & load is allowed for chroma CUs, we should check whether luma info (pred, recon, etc) is the same, which is quite complex
  }

  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew( cs.area.Y(), *m_pcv, idx1, idx2, idx3, idx4 );

  const BestEncodingInfo* stored = m_bestEncInfo[idx1][idx2][idx3][idx4];
  if( !stored )
  {
    return false;
  }
  const BestEncodingInfo& encInfo = *stored;

  if( encInfo.cu.treeType != partitioner.treeType || encInfo.cu.modeType != partitioner.modeType )
  {
    return false;
  }
  if( encInfo.cu.qp != qp )
    return false;
  // the cheap tests first, the area and neighbourhood comparisons last
  if( cs.picture->poc != encInfo.poc
    || cs.currQP[partitioner.chType] != encInfo.cu.qp
    || CU::isIBC(encInfo.cu)
    || CS::getArea( cs, cs.area, partitioner.chType, partitioner.treeType ) != CS::getArea( cs, encInfo.cu, partitioner.chType, partitioner.treeType )
    || !isTheSameNbHood( encInfo.cu, cs, partitioner, (cs.picture->Y().width), (cs.picture->Y().height), xSharedLevels( encInfo ) )
    )
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool BestEncInfoCache::setCsFrom( CodingStructure& cs, EncTestMode& testMode, const Partitioner& partitioner ) const
{
  if( !m_reuseCuResults ) return false;

  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew( cs.area.Y(), *m_pcv, idx1, idx2, idx3, idx4 );

  const BestEncodingInfo* stored = m_bestEncInfo[idx1][idx2][idx3][idx4];
  if( !stored )
  {
    return false;
  }
  const BestEncodingInfo& encInfo = *stored;

  if( cs.picture->poc != encInfo.poc 
    || CS::getArea( cs, cs.area, partitioner.chType, partitioner.treeType ) != CS::getArea( cs, encInfo.cu, partitioner.chType, partitioner.treeType ) 
    || !isTheSameNbHood( encInfo.cu, cs, partitioner, (cs.picture->Y().width), (cs.picture->Y().height), xSharedLevels( encInfo ) )
    || partitioner.currQgEnable() || cs.currQP[partitioner.chType] != encInfo.cu.qp
    )
  {
    return false;
  }

  const UnitArea ua = CS::getArea( cs, cs.area, partitioner.chType, partitioner.treeType );
  CodingUnit     &cu = cs.addCU( ua, partitioner.chType );
  cu.treeType = partitioner.treeType;
  cu.modeType = partitioner.modeType;
  cu.initPuData();
  TransformUnit  &tu = cs.addTU( ua, partitioner.chType, &cu );

  cu          .repositionTo( encInfo.cu );
  cu          .repositionTo( encInfo.cu );
  tu          .repositionTo( encInfo.tu );

  cu          = encInfo.cu;
  cu          = encInfo.cu;
  for( auto &blk : tu.blocks )
  {
    if( blk.valid() ) tu.copyComponentFrom( encInfo.tu, blk.compID );
  }

  testMode    = encInfo.testMode;
  cs.dist     = encInfo.dist;
  cs.costDbOffset = encInfo.costEDO;
  return true;
}


//////////////////////////////////////////////////////////////////////////
// EncModeCtrl
//////////////////////////////////////////////////////////////////////////

// FastMTTRefine acts where the MTT depth limit is in this range. Below 2 an MTT root would itself be the node whose
// splits the first pass leaves out; above 3 the first pass has levels of nodes the second pass has no rule for.
static constexpr unsigned MIN_REFINED_MTT_DEPTH_LIMIT = 2;
static constexpr unsigned MAX_REFINED_MTT_DEPTH_LIMIT = 3;

// FastAMVR: where the motion search with quarter-sample MVDs and equal weights shows that AMVR cannot pay
static constexpr double   AMVR_ME_BEHIND_RATIO  = 1.1;   // it costs more than this times the best mode: a merge mode or intra is far ahead of any searched motion
static constexpr int      AMVR_MIN_MVD          = 2;     // its largest MVD component (quarter samples) is below this: every MVD is 0 or 1/4, no MVD bits to save
static constexpr int      AMVR_4PEL_MIN_MVD     = 16;    // ... below this (4 samples): no 4-sample MVDs, which round every MVD to 4 samples

// MTT depth of a node below the splits the picture border forces, which are not counted against the depth limit
static int freeMttDepth( const Partitioner& partitioner )
{
  return ( int ) partitioner.currMtDepth - ( int ) partitioner.currImplicitBtDepth;
}

static bool isBtOrTt( const PartSplit split )
{
  return split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT;
}

void EncModeCtrl::init( const VVEncCfg& encCfg, RdCost* pRdCost )
{
  m_pcEncCfg = &encCfg;
  m_pcRdCost = pRdCost;
  comprCUCtx = nullptr;

  CacheBlkInfoCtrl::create( encCfg.m_CTUSize );
  BestEncInfoCache::create( encCfg.m_reuseCuResults, encCfg.m_internChromaFormat, encCfg.m_CTUSize );

  // FastMTTRefine acts on the inter pictures with the highest MTT depth limit of the encode's temporal layers, which
  // is also the id of its split-cost table (3 at preset slower, whose top layer with limit 2 is searched as before)
  const unsigned highestMttDepthLimit = ( unsigned ) std::max( 0, encCfg.m_splitCostThrParamId );
  const bool     limitCanBeRefined    = highestMttDepthLimit >= MIN_REFINED_MTT_DEPTH_LIMIT && highestMttDepthLimit <= MAX_REFINED_MTT_DEPTH_LIMIT;
  m_mttRefineLimit = encCfg.m_fastMTTRefine && limitCanBeRefined ? highestMttDepthLimit : 0;
  m_mttSearchPass  = MTT_SEARCH_FULL;
}

void EncModeCtrl::destroy()
{
  CacheBlkInfoCtrl::destroy();
  BestEncInfoCache::destroy();
}

void EncModeCtrl::initCTUEncoding( const Slice &slice )
{
  CacheBlkInfoCtrl::init( slice );
  BestEncInfoCache::init( slice );

  CHECK( !m_ComprCUCtxList.empty(), "Mode list is not empty at the beginning of a CTU" );

  if( m_pcEncCfg->m_fastQtBtEnc )
  {
    m_skipThresholdE0023FastEnc = ((slice.getMinPictureDistance() <= PICTURE_DISTANCE_TH) ? FAST_SKIP_DEPTH : SKIP_DEPTH);
  }
  else
  {
    m_skipThresholdE0023FastEnc = SKIP_DEPTH;
  }
}

void EncModeCtrl::initCULevel( Partitioner &partitioner, const CodingStructure& cs, int  MergeSimpleFlag, const CodingUnit* cuLeft, const CodingUnit* cuAbove )
{
  // Min/max depth
  unsigned minDepth = 0;
  unsigned maxDepth = cs.pcv->getMaxDepth( cs.slice->sliceType, partitioner.chType );
  if( m_pcEncCfg->m_useFastLCTU )
  {
    partitioner.setMaxMinDepth(minDepth, maxDepth, cs, cs.picture->useQtbttSpeedUpMode, MergeSimpleFlag, cuLeft, cuAbove);
  }

  minDepth = std::max<unsigned>( minDepth, cs.pcv->getMinDepth( cs.slice->sliceType, partitioner.chType ) );
  maxDepth = std::min<unsigned>( maxDepth, cs.pcv->getMaxDepth( cs.slice->sliceType, partitioner.chType ) );

  m_ComprCUCtxList.push_back( ComprCUCtx( cs, minDepth, maxDepth ) );
  comprCUCtx = &m_ComprCUCtxList.back();

  const unsigned mttDepthLimit = xSearchMttDepthLimit( cs, partitioner );

  const bool qtBeforeBt = ( (  cuLeft  &&  cuAbove  && cuLeft ->qtDepth > partitioner.currQtDepth && cuAbove->qtDepth > partitioner.currQtDepth )
                         || (  cuLeft  && !cuAbove  && cuLeft ->qtDepth > partitioner.currQtDepth )
                         || ( !cuLeft  &&  cuAbove  && cuAbove->qtDepth > partitioner.currQtDepth )
                         || ( !cuAbove && !cuLeft   && cs.area.lwidth() >= ( 32 << cs.slice->TLayer ) )
                         || ( m_pcEncCfg->m_qtbttSpeedUp > 1 && mttDepthLimit < ( ( cs.slice->isIntra() && !cs.sps->IBC ) ? 3u : 2u ) ) )
                         && ( cs.area.lwidth() > ( cs.pcv->getMinQtSize( *cs.slice, partitioner.chType ) << 1 ) );

  // set features
  ComprCUCtx &cuECtx    = *comprCUCtx;
  cuECtx.mttDepthLimit  = mttDepthLimit;
  cuECtx.qtBeforeBt     = qtBeforeBt;
  cuECtx.doTriHorzSplit = true;
  cuECtx.doTriVertSplit = true;
  cuECtx.doMoreSplits   = 3;
  cuECtx.isReusingCu    = isReusingCuValid( cs, partitioner, cs.baseQP, mttDepthLimit );
  cuECtx.didHorzSplit   = partitioner.canSplit( CU_HORZ_SPLIT, cs );
  cuECtx.didVertSplit   = partitioner.canSplit( CU_VERT_SPLIT, cs );
  cuECtx.doHorChromaSplit = true;
  cuECtx.doVerChromaSplit = true;
  cuECtx.doQtChromaSplit  = true;

  if( m_pcEncCfg->m_contentBasedFastQtbt && cs.pcv->getMaxMTTDepth(*cs.slice, partitioner.chType))
  {
    const CompArea& currArea = partitioner.currArea().Y();
    int cuHeight  = currArea.height;
    int cuWidth   = currArea.width;

    const bool condIntraInter = m_pcEncCfg->m_IntraPeriod == 1 ? ( partitioner.currBtDepth == 0 ) : ( cuHeight > 32 && cuWidth > 32 );

    if( cuWidth == cuHeight && condIntraInter )
    {
      const CPelBuf bufCurrArea = cs.getOrgBuf( partitioner.currArea().block( COMP_Y ) );

      int grad[4];
      g_pelBufOP.absGradientSums( bufCurrArea.buf, bufCurrArea.stride, cuWidth, cuHeight, grad );

      cuECtx.grad_horVal = (double)grad[0];
      cuECtx.grad_verVal = (double)grad[1];
      cuECtx.grad_dowVal = (double)grad[2];
      cuECtx.grad_dupVal = (double)grad[3];
    }
  }
}

void EncModeCtrl::finishCULevel( Partitioner &partitioner )
{
  if( m_mttSearchPass == MTT_SEARCH_CAPPED && xChoosesSplitsToRefine( partitioner ) )
  {
    xStoreSplitsToRefine( partitioner );
  }

  m_ComprCUCtxList.pop_back();
  comprCUCtx = m_ComprCUCtxList.size() ? &m_ComprCUCtxList.back() : nullptr;
}

bool EncModeCtrl::trySplit( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner, const EncTestMode& lastTestmode )
{
  ComprCUCtx& cuECtx = *comprCUCtx;

  const PartSplit implicitSplit = partitioner.getImplicitSplit( cs );
  const bool isBoundary         = implicitSplit != CU_DONT_SPLIT;

  if( ( m_pcEncCfg->m_IntraPeriod == 1 ) && ( partitioner.chType == CH_C ) && ( !cuECtx.doQtChromaSplit ) )
  {
    cuECtx.maxDepth         = partitioner.currDepth;
  }

  if( isBoundary )
  {
    if( encTestmode.type != ETM_SPLIT_QT )
    {
      return getPartSplit( encTestmode ) == implicitSplit;
    }
    else
    {
      return partitioner.canSplit( CU_QUAD_SPLIT, cs );
    }
  }

  const Slice&           slice       = *cs.slice;
  const uint32_t         width       = partitioner.currArea().lumaSize().width;
  const CodingStructure *bestCS      = cuECtx.bestCS;
  const CodingUnit      *bestCU      = cuECtx.bestCU;

  if( cuECtx.minDepth > partitioner.currQtDepth && partitioner.canSplit( CU_QUAD_SPLIT, cs ) )
  {
    // enforce QT
    return encTestmode.type == ETM_SPLIT_QT;
  }
  else if( encTestmode.type == ETM_SPLIT_QT && cuECtx.maxDepth <= partitioner.currQtDepth )
  {
    // don't check this QT depth
    return false;
  }

  CHECK( ! isModeSplit( encTestmode ), "wrong method" );

  // FastMTTRefine: in the second pass a child of the MTT root searches again exactly the splits noted for it at the end
  // of the first pass. The rules below are not asked again: they let these splits through in the first pass, and now
  // they would read the results of sibling splits that are not searched again. (With an MTT depth limit of 2 nothing
  // was noted: the child's splits are the ones left out, and the rules below decide on them now for the first time.)
  if( m_mttSearchPass == MTT_SEARCH_REFINE && xChoosesSplitsToRefine( partitioner ) )
  {
    return ( getBlkInfo( partitioner.currArea() ).refineSplits >> getPartSplit( encTestmode ) ) & 1;
  }

  //////////////////////////////////////////////////////////////////////////
  // skip-history rule - don't split further if at least for three past levels
  //                     in the split tree it was found that skip is the best mode
  //////////////////////////////////////////////////////////////////////////
  int skipScore = 0;

  if ((!slice.isIntra() || slice.sps->IBC) && cuECtx.isBestNoSplitSkip )
  {
    for( int i = 2; i <= m_ComprCUCtxList.size(); i++ )
    {
      if( ( m_ComprCUCtxList.end() - i )->isBestNoSplitSkip )
      {
        skipScore += 1;
      }
      else
      {
        break;
      }
    }
  }

  const PartSplit split = getPartSplit( encTestmode );
  if( !partitioner.canSplit( split, cs ) || skipScore >= 2 || ( skipScore == 1 && m_ComprCUCtxList.size() == 2 ) )
  {
    if( split == CU_HORZ_SPLIT ) cuECtx.didHorzSplit = false;
    if( split == CU_VERT_SPLIT ) cuECtx.didVertSplit = false;
    if( split == CU_QUAD_SPLIT ) cuECtx.didQuadSplit = false;
    return false;
  }

  if( isChroma( partitioner.chType ) && !cuECtx.doHorChromaSplit && ( split == CU_HORZ_SPLIT || split == CU_TRIH_SPLIT ) )
  {
    if( split == CU_HORZ_SPLIT )
    {
      cuECtx.didHorzSplit = false;
    }

    return false;
  }

  if( isChroma( partitioner.chType ) && !cuECtx.doVerChromaSplit && ( split == CU_VERT_SPLIT || split == CU_TRIV_SPLIT ) )
  {
    if( split == CU_VERT_SPLIT )
    {
      cuECtx.didVertSplit = false;
    }

    return false;
  }

  if( m_pcEncCfg->m_contentBasedFastQtbt )
  {
    const CompArea& currArea = partitioner.currArea().Y();
    int cuHeight  = currArea.height;
    int cuWidth   = currArea.width;

    const bool condIntraInter = m_pcEncCfg->m_IntraPeriod == 1 ? ( partitioner.currBtDepth == 0 ) : ( cuHeight > 32 && cuWidth > 32 );

    if( cuWidth == cuHeight && condIntraInter && split != CU_QUAD_SPLIT )
    {
      const double th1 = m_pcEncCfg->m_IntraPeriod == 1 ?  1.2              :  1.0;
      const double th2 = m_pcEncCfg->m_IntraPeriod == 1 ? (1.2 / sqrt( 2 )) : (1.0 / sqrt( 2 ));

      if( cuECtx.grad_horVal > th1 * cuECtx.grad_verVal && cuECtx.grad_horVal > th2 * cuECtx.grad_dowVal && cuECtx.grad_horVal > th2 * cuECtx.grad_dupVal && ( split == CU_HORZ_SPLIT || split == CU_TRIH_SPLIT ) )
      {
        return false;
      }
      if( th2 * cuECtx.grad_dupVal < cuECtx.grad_verVal && th2 * cuECtx.grad_dowVal < cuECtx.grad_verVal && th1 * cuECtx.grad_horVal < cuECtx.grad_verVal && ( split == CU_VERT_SPLIT || split == CU_TRIV_SPLIT ) )
      {
        return false;
      }
    }

    if( m_pcEncCfg->m_IntraPeriod == 1 && cuWidth <= 32 && cuHeight <= 32 && bestCS && bestCS->tus.size() == 1 && bestCU && bestCU->depth == partitioner.currDepth && partitioner.currBtDepth > 1 && isLuma( partitioner.chType ) )
    {
      if( !bestCU->rootCbf )
      {
        return false;
      }
    }
  }

  if( m_pcEncCfg->m_qtbttSpeedUp > 1 )
  {
    const int availDepth = cuECtx.mttDepthLimit - partitioner.currMtDepth;
    if( bestCU && bestCU->skip && availDepth <= ( 3 - m_skipThresholdE0023FastEnc ) && !isModeSplit( lastTestmode ) && split != CU_QUAD_SPLIT )
    {
      return false;
    }
  }
  if( bestCU && bestCU->skip && bestCU->mtDepth >= m_skipThresholdE0023FastEnc && !isModeSplit( lastTestmode ) )
  {
    return false;
  }

  bool resetFeature = false;

  switch( split )
  {
    case CU_QUAD_SPLIT:
      {
        if( !cuECtx.qtBeforeBt && bestCU )
        {
          unsigned maxBTD        = cuECtx.mttDepthLimit;
          const CodingUnit *cuBR = bestCS->cus.back();
          unsigned height        = partitioner.currArea().lumaSize().height;

          if(((bestCU->btDepth == 0 && maxBTD >= ((slice.isIntra() && !slice.sps->IBC) ? 3 : 2))
            || (bestCU->btDepth == 1 && cuBR && cuBR->btDepth == 1 && maxBTD >= ((slice.isIntra() && !slice.sps->IBC) ? 4 : 3)))
            && (width <= MAX_TB_SIZEY && height <= MAX_TB_SIZEY)
            && cuECtx.didHorzSplit && cuECtx.didVertSplit )
          {
            return false;
          }
        }
        if( bestCS )
        {
          if( m_pcEncCfg->m_useEarlyCU == 2 && bestCS->cost != MAX_DOUBLE && bestCU && bestCU->skip && cuECtx.nonSkipWasTested && bestCS->cus.size() == 1 )
          {
            return false;
          }

          int stopSplit = (m_pcEncCfg->m_FastInferMerge >> 4) && (bestCS->slice->TLayer > 4);
          int limitBLsize = stopSplit ? 2048 : 1024;
          if((m_pcEncCfg->m_useEarlyCU == 1 || stopSplit) && bestCS->cost != MAX_DOUBLE && bestCU && bestCU->skip && partitioner.currArea().lumaSize().area() < limitBLsize )
          {
            return false;
          }
        }
      }
      break;
    case CU_HORZ_SPLIT:
    case CU_VERT_SPLIT:
      resetFeature = true;
      break;
    case CU_TRIH_SPLIT:
      if( cuECtx.didHorzSplit && bestCU && bestCU->btDepth == partitioner.currBtDepth && !bestCU->rootCbf )
      {
        return false;
      }

      if( m_pcEncCfg->m_qtbttSpeedUp > 1 )
      {
        //unsigned maxBTD = cs.pcv->getMaxMTTDepth( slice, partitioner.chType );
        if( /*maxBTD == 1 && */cuECtx.didHorzSplit && cuECtx.didVertSplit && cuECtx.bestCostHorzSplit > cuECtx.bestCostVertSplit )
        {
          return false;
        }
      }

      if( !cuECtx.doTriHorzSplit )
      {
        return false;
      }

      if (m_pcEncCfg->m_fastTTSplit && cuECtx.bestCostHorzSplit < MAX_DOUBLE)
      {
        if (cuECtx.bestCostBeforeSplit < MAX_DOUBLE && (cuECtx.bestCostHorzSplit > m_pcEncCfg->m_fastTT_th * cuECtx.bestCostBeforeSplit))
        {
          return false;
        }
        if (cuECtx.bestCostVertSplit < MAX_DOUBLE && (cuECtx.bestCostHorzSplit > m_pcEncCfg->m_fastTT_th * cuECtx.bestCostVertSplit))
        {
          return false;
        }
      }
      break;
    case CU_TRIV_SPLIT:
      if( cuECtx.didVertSplit && bestCU && bestCU->btDepth == partitioner.currBtDepth && !bestCU->rootCbf )
      {
        return false;
      }

      if( m_pcEncCfg->m_qtbttSpeedUp > 1 )
      {
        //unsigned maxBTD = cs.pcv->getMaxMTTDepth( slice, partitioner.chType );
        if( /*maxBTD == 1 && */cuECtx.didHorzSplit && cuECtx.didVertSplit && cuECtx.bestCostHorzSplit < cuECtx.bestCostVertSplit )
        {
          return false;
        }
      }

      if( !cuECtx.doTriVertSplit )
      {
        return false;
      }

      if (m_pcEncCfg->m_fastTTSplit && cuECtx.bestCostVertSplit < MAX_DOUBLE)
      {
        if (cuECtx.bestCostBeforeSplit < MAX_DOUBLE && (cuECtx.bestCostVertSplit > m_pcEncCfg->m_fastTT_th * cuECtx.bestCostBeforeSplit))
        {
          return false;
        }
        if (cuECtx.bestCostHorzSplit < MAX_DOUBLE && (cuECtx.bestCostVertSplit > m_pcEncCfg->m_fastTT_th * cuECtx.bestCostHorzSplit))
        {
          return false;
        }
      }
      break;
    default:
      THROW( "Only CU split modes are governed by the EncModeCtrl" );
      return false;
      break;
  }

  switch( split )
  {
    case CU_HORZ_SPLIT:
    case CU_TRIH_SPLIT:
      if( cuECtx.qtBeforeBt && cuECtx.didQuadSplit )
      {
        if( cuECtx.maxQtSubDepth > partitioner.currQtDepth + 1 )
        {
          if( resetFeature ) cuECtx.didHorzSplit = false;
          return false;
        }
      }
      break;
    case CU_VERT_SPLIT:
    case CU_TRIV_SPLIT:
      if( cuECtx.qtBeforeBt && cuECtx.didQuadSplit )
      {
        if( cuECtx.maxQtSubDepth > partitioner.currQtDepth + 1 )
        {
          if( resetFeature ) cuECtx.didVertSplit = false;
          return false;
        }
      }
      break;
    default:
      break;
  }

  const bool passedAllRules = m_pcEncCfg->m_qtbttSpeedUp <= 1 || !!cuECtx.doMoreSplits;

  if( split == CU_QUAD_SPLIT )
  {
    cuECtx.didQuadSplit = passedAllRules;
  }
  else if( passedAllRules && xSkipsDeepestMttLevel( cs, partitioner ) )
  {
    // asked last, so that FastMTTRefine notes exactly the splits the rules above would search
    return false;
  }

  return passedAllRules;
}

bool EncModeCtrl::tryMode( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner )
{
  CHECK( isModeSplit( encTestmode ), "wrong method");

  ComprCUCtx& cuECtx = m_ComprCUCtxList.back();

  if( cuECtx.minDepth > partitioner.currQtDepth && partitioner.canSplit( CU_QUAD_SPLIT, cs ) )
  {
    // enforce QT
    return false;
  }

  const Slice&           slice        = *cs.slice;
  const uint32_t         numComp      = getNumberValidComponents( slice.sps->chromaFormatIdc );
  const CodedCUInfo      &relatedCU   = getBlkInfo( partitioner.currArea() );
  const Area             &lumaArea    = partitioner.currArea().Y();
  const CodingStructure* bestCS       = cuECtx.bestCS;
  const EncTestMode      bestMode     = bestCS ?cuECtx.bestMode : EncTestMode();

  if( encTestmode.type == ETM_INTRA )
  {
    // if this is removed, the IntraSearch::xIntraCodingLumaQT needs to be adapted to support Intra TU split
    // also isXXAvailable in IntraPrediction.cpp need to be fixed to check availability within the same CU without isDecomp
    if (m_pcEncCfg->m_FastInferMerge && !slice.isIntra() && !slice.isIRAP() && !(cs.area.lwidth() == 4 && cs.area.lheight() == 4) && !partitioner.isConsIntra())
    {
      if (bestCS && (bestCS->slice->TLayer > (m_pcEncCfg->m_maxTLayer - (m_pcEncCfg->m_FastInferMerge & 7)))
        && (bestCS->bestParent != nullptr) && bestCS->bestParent->cus.size() && (bestCS->bestParent->cus[0]->skip))
      {
        return false;
      }
    }

    if( lumaArea.width > cs.sps->getMaxTbSize() || lumaArea.height > cs.sps->getMaxTbSize() )
    {
      return false;
    }

    if (m_pcEncCfg->m_usePbIntraFast && (!cs.slice->isIntra() || cs.slice->sps->IBC) && cuECtx.interHad == 0 && cuECtx.bestCU && !CU::isIntra(*cuECtx.bestCU))
    {
      return false;
    }

    // INTRA MODES
    if (cs.sps->IBC && !cuECtx.bestTU)
    {
     // return true;
    }
    else
    if( partitioner.isConsIntra() && !cuECtx.bestTU )
    {
      //return true;
    }
    else
    if ( lumaArea.width == 4 && lumaArea.height == 4 && !slice.isIntra() && !cuECtx.bestTU )
    {
      //return true;
    }
    else
    if (!(slice.isIRAP() || bestMode.type == ETM_INTRA || !cuECtx.bestTU ||
      ((!m_pcEncCfg->m_bDisableIntraCUsInInterSlices) && (!relatedCU.isInter || !relatedCU.isIBC) && (
                                    ( cuECtx.bestTU->cbf[0] != 0 ) ||
           ( ( numComp > COMP_Cb ) && cuECtx.bestTU->cbf[1] != 0 ) ||
           ( ( numComp > COMP_Cr ) && cuECtx.bestTU->cbf[2] != 0 )  // avoid very complex intra if it is unlikely
         ) ) ) )
    {
      return false;
    }
    else
    if( cuECtx.bestCS && cuECtx.bestCU && cuECtx.interHad )
    {
      // Get SATD threshold from best Inter-CU
      if( !cs.slice->isIRAP() && m_pcEncCfg->m_usePbIntraFast )
      {
        const DFunc dfunc = DF_HAD;
        DistParam distParam = m_pcRdCost->setDistParam( cs.getOrgBuf( COMP_Y ), cuECtx.bestCS->getPredBuf( COMP_Y ), cs.sps->bitDepths[ CH_L ], dfunc );
        cuECtx.interHad = distParam.distFunc( distParam );
      }
    }
    if ((m_pcEncCfg->m_IBCFastMethod > 1) && !cs.slice->isIntra()//IBC_FAST_METHOD_NOINTRA_IBCCBF0
      && (bestMode.type == ETM_IBC || bestMode.type == ETM_IBC_MERGE)
      && (!cuECtx.bestCU->Y().valid() || cuECtx.bestTU->cbf[0] == 0)
      && (!cuECtx.bestCU->Cb().valid() || cuECtx.bestTU->cbf[1] == 0)
      && (!cuECtx.bestCU->Cr().valid() || cuECtx.bestTU->cbf[2] == 0))
    {
      return false;
    }
    if (m_pcEncCfg->m_FastIntraTools)
    {
      if (relatedCU.relatedCuIsValid)
      {
        cuECtx.relatedCuIsValid = relatedCU.relatedCuIsValid;
      }
      if (relatedCU.isIntra)
      {
        cuECtx.isIntra = relatedCU.isIntra;
      }
    }
  }
  else if( isModeInter( encTestmode ) )
  {
    // INTER MODES (ME + MERGE/SKIP)
    CHECK( slice.isIntra(), "Inter-mode should not be in the I-Slice mode list!" );

    // FastMEBelowSkip: no motion search in the split parts of a block whose best mode without a split is skip. The
    // parts of a block that one merge candidate predicts without a residual rarely need a searched motion vector.
    if( m_pcEncCfg->m_fastMEBelowSkip && ( encTestmode.type == ETM_INTER_ME || encTestmode.type == ETM_INTER_IMV )
        && m_ComprCUCtxList.size() > 1 && ( m_ComprCUCtxList.end() - 2 )->isBestNoSplitSkip )
    {
      return false;
    }

    // --- Check if we can quit current mode using SAVE/LOAD coding history

    if( encTestmode.type == ETM_INTER_ME )
    {
      if( encTestmode.opts == ETO_STANDARD )
      {
        if (m_pcEncCfg->m_FastInferMerge)
        {
          if (bestCS && (bestCS->slice->TLayer > (m_pcEncCfg->m_maxTLayer - (m_pcEncCfg->m_FastInferMerge & 7)))
            && (bestCS->bestParent != nullptr) && bestCS->bestParent->cus.size() && (bestCS->bestParent->cus[0]->skip))
          {
            return false;
          }
        }
        if( relatedCU.isSkip || relatedCU.isIntra )
        {
          return false;
        }
      }
    }
  }
  else if (encTestmode.type == ETM_IBC || encTestmode.type == ETM_IBC_MERGE)
  {
    if ((m_pcEncCfg->m_IBCFastMethod > 1) && !(slice.isIRAP() || bestMode.type == ETM_INTRA || !cuECtx.bestTU ||
      ((!m_pcEncCfg->m_bDisableIntraCUsInInterSlices) && (!relatedCU.isInter || !relatedCU.isIBC) && (
        (cuECtx.bestTU->cbf[0] != 0) ||
        ((numComp > COMP_Cb) && cuECtx.bestTU->cbf[1] != 0) ||
        ((numComp > COMP_Cr) && cuECtx.bestTU->cbf[2] != 0)  // avoid very complex intra if it is unlikely
        ))))
    {
      return false;
    }
    if ((m_pcEncCfg->m_IBCFastMethod > 3) &&(lumaArea.width == 4 && lumaArea.height == 4 && !slice.isIntra()))
    {
      return false;
    }
    // IBC MODES
    return slice.sps->IBC && (partitioner.currArea().lumaSize().width < 128 && partitioner.currArea().lumaSize().height < 128);
  }
  else
  {
    THROW("problem");
    return false;
  }

  STAT_COUNT_CU_MODES( partitioner.chType == CH_L, g_cuCounters1D[CU_MODES_TRIED][0][!cs.slice->isIntra() + cs.slice->depth] );
  STAT_COUNT_CU_MODES( partitioner.chType == CH_L && !cs.slice->isIntra(), g_cuCounters2D[CU_MODES_TRIED][Log2( cs.area.lheight() )][Log2( cs.area.lwidth() )] );
  return true;
}


void EncModeCtrl::beforeSplit( Partitioner& partitioner )
{
  ComprCUCtx&           cuECtx  = m_ComprCUCtxList.back();

  if( ! cuECtx.bestCS )
    return;

  CodedCUInfo    &relatedCU   = getBlkInfo( partitioner.currArea() );
  const CodingUnit&  bestCU   = *cuECtx.bestCU;

  cuECtx.bestNsPredMode       = cuECtx.bestMode;
  cuECtx.bestCostBeforeSplit  = cuECtx.bestCS->cost;
  cuECtx.bestNoSplitCost      = cuECtx.bestCS->cost + cuECtx.bestCS->costDbOffset;   // read by FastMTTRefine only; the
                                                                                          // EDO offset is 0 where EDO did not run

  setFromCs( *cuECtx.bestCS, cuECtx.bestMode, partitioner );

  if( bestCU.skip )
  {
    cuECtx.doMoreSplits--;
  }

  if( partitioner.modeType == MODE_TYPE_INTRA && partitioner.chType == CH_L )
  {
    return; //not set best coding mode for intra coding pass
  }

  // assume the non-split modes are done and set the marks for the best found mode
  if( CU::isInter( bestCU ) )
  {
    relatedCU.isInter     = true;
    relatedCU.isSkip     |= bestCU.skip;
    relatedCU.isMMVDSkip |= bestCU.mmvdSkip;
    relatedCU.BcwIdx      = bestCU.BcwIdx;
  }
  else if (CU::isIBC(bestCU))
  {
    relatedCU.isIBC = true;
    relatedCU.isSkip |= bestCU.skip;
  }
  else if( CU::isIntra( bestCU ) )
  {
    relatedCU.isIntra     = true;
    if (m_pcEncCfg->m_FastIntraTools)
    {
      if ( cuECtx.intraWasTested && (!relatedCU.relatedCuIsValid || cuECtx.bestCS->cost < relatedCU.bestCost))
      {
        relatedCU.bestCost = cuECtx.bestCS->cost;
        relatedCU.relatedCuIsValid = true;
      }
    }
  }
  cuECtx.isBestNoSplitSkip = bestCU.skip;
}


bool EncModeCtrl::useModeResult( const EncTestMode& encTestmode, CodingStructure*& tempCS, Partitioner& partitioner, const bool useEDO )
{
  ComprCUCtx& cuECtx = m_ComprCUCtxList.back();

  if( m_pcEncCfg->m_fastAMVR )
  {
    xNoteMotionSearchCost( encTestmode, *tempCS, useEDO );
  }

  const bool isLumaBtOrTt = isLuma( partitioner.chType ) && isBtOrTt( getPartSplit( encTestmode ) );
  if( m_mttSearchPass == MTT_SEARCH_REFINE && isLumaBtOrTt && freeMttDepth( partitioner ) == 0 )
  {
    // FastMTTRefine: a split of the MTT root, searched a second time, only competes with the results so far. What the
    // rules know about the root's splits stays as the first pass left it; the root's QT decision reads it.
    if( !xKeepIfBest( encTestmode, tempCS, useEDO ) )
    {
      return false;
    }
    // the root's best result now comes from a search with the partitioner's limit, so the QT decision reads that one
    cuECtx.mttDepthLimit = partitioner.maxBTD;
    return true;
  }
  if( m_mttSearchPass == MTT_SEARCH_CAPPED && isLumaBtOrTt && tempCS->cost != MAX_DOUBLE )
  {
    // the lower of the two where a split is searched once with inter and once with intra sub-blocks
    double& splitCost = cuECtx.mttSplitCost[getPartSplit( encTestmode ) - CU_HORZ_SPLIT];
    splitCost         = std::min( splitCost, tempCS->cost + ( useEDO ? tempCS->costDbOffset : 0 ) );
  }

  if(      encTestmode.type == ETM_SPLIT_BT_H )
  {
    cuECtx.bestCostHorzSplit = tempCS->cost;
  }
  else if( encTestmode.type == ETM_SPLIT_BT_V )
  {
    cuECtx.bestCostVertSplit = tempCS->cost;
  }
  else if( !isModeSplit( encTestmode ) && isModeInter( encTestmode ) && tempCS->cus.size() == 1 )
  {
    cuECtx.nonSkipWasTested |= !tempCS->cus.front()->skip;
  }
  if (m_pcEncCfg->m_AMVRspeed && encTestmode.type == ETM_INTER_ME)
  {
    int imvMode = (encTestmode.opts & ETO_IMV) >> ETO_IMV_SHIFT;

    if (imvMode == 0)
    {
      if (tempCS->cost < cuECtx.bestCostNoImv )
      {
        cuECtx.bestCostNoImv = tempCS->cost;
      }
    }
  }

  if( encTestmode.type == ETM_SPLIT_QT )
  {
    int maxQtD = 0;
    for( const auto& cu : tempCS->cus )
    {
      maxQtD = std::max<int>( maxQtD, cu->qtDepth );
    }
    cuECtx.maxQtSubDepth = maxQtD;
  }

  int maxMtD = cuECtx.mttDepthLimit + partitioner.currImplicitBtDepth;

  if( encTestmode.type == ETM_SPLIT_BT_H )
  {
    if( tempCS->cus.size() > 2 )
    {
      int h_2   = tempCS->area.blocks[partitioner.chType].height / 2;
      int cu1_h = tempCS->cus.front()->blocks[partitioner.chType].height;
      int cu2_h = tempCS->cus.back() ->blocks[partitioner.chType].height;

      cuECtx.doTriHorzSplit = cu1_h < h_2 || cu2_h < h_2 || partitioner.currMtDepth + 1 == maxMtD;
    }
  }
  else if( encTestmode.type == ETM_SPLIT_BT_V )
  {
    if( tempCS->cus.size() > 2 )
    {
      int w_2   = tempCS->area.blocks[partitioner.chType].width / 2;
      int cu1_w = tempCS->cus.front()->blocks[partitioner.chType].width;
      int cu2_w = tempCS->cus.back() ->blocks[partitioner.chType].width;

      cuECtx.doTriVertSplit = cu1_w < w_2 || cu2_w < w_2 || partitioner.currMtDepth + 1 == maxMtD;
    }
  }

  if( encTestmode.type == ETM_SPLIT_BT_V || encTestmode.type == ETM_SPLIT_BT_H || encTestmode.type == ETM_SPLIT_QT )
  {
    bool isAllSkip = true;

    for( const auto& cu : tempCS->cus )
    {
      isAllSkip &= cu->skip;
    }

    if( isAllSkip ) cuECtx.doMoreSplits--;
    if( isAllSkip && encTestmode.type == ETM_SPLIT_QT )
                    cuECtx.doMoreSplits--;
  }

  return xKeepIfBest( encTestmode, tempCS, useEDO );
}

// Makes tempCS the best result of the current CU if it is better than the best so far; returns whether it did
bool EncModeCtrl::xKeepIfBest( const EncTestMode& encTestmode, CodingStructure* tempCS, const bool useEDO )
{
  ComprCUCtx& cuECtx = m_ComprCUCtxList.back();

  // for now just a simple decision based on RD-cost or choose tempCS if bestCS is not yet coded
  if( tempCS->cost != MAX_DOUBLE && ( !cuECtx.bestCS || ( ( tempCS->cost + ( useEDO ? tempCS->costDbOffset : 0 ) ) < ( cuECtx.bestCS->cost + ( useEDO ? cuECtx.bestCS->costDbOffset : 0 ) ) ) ) )
  {
    cuECtx.bestCS   = tempCS;
    cuECtx.bestCU   = tempCS->cus[0];
    cuECtx.bestTU   = cuECtx.bestCU->firstTU;
    cuECtx.bestMode = encTestmode;

    return true;
  }
  else
  {
    return false;
  }
}

//////////////////////////////////////////////////////////////////////////
// FastMTTRefine: two-pass BT / TT search of the inter pictures with the encode's highest MTT depth limit L (2 or 3)
//
// Every MTT root (a luma node that no BT / TT split lies above) searches its BT / TT splits twice. The first, capped
// pass leaves out the deepest MTT level, the splits that make CUs of MTT depth L. The second pass then searches again
// the root's splits whose cost in the first pass is at most the root's best cost without QT, and below which a search
// was left out, now with the deepest level. With L = 3 it searches, inside them, only the children's splits that are
// at most the child's best cost (xStoreSplitsToRefine). With L = 2 the root's children are the nodes whose splits
// were left out, so there are no costs to choose by: they search their splits as always.
// The first pass is searched as a picture with MTT depth limit L - 1 is: its split rules, its reuse of stored CU
// results and its split-cost table read that limit (ComprCUCtx::mttDepthLimit). Only the split flags are coded and
// priced with the picture's limit L.
//////////////////////////////////////////////////////////////////////////

// True for the nodes FastMTTRefine acts on: luma, in an inter picture with the MTT depth limit it refines
bool EncModeCtrl::xRefinesMtt( const CodingStructure& cs, const Partitioner& partitioner ) const
{
  return m_mttRefineLimit && isLuma( partitioner.chType ) && !cs.slice->isIntra() && partitioner.maxBTD == m_mttRefineLimit;
}

// The MTT depth limit that a node's split rules, its reuse of stored results (BestEncInfoCache::isReusingCuValid) and
// its split-cost table (EncCu::xCheckModeSplitInternal) read; called when the node begins. It is the partitioner's,
// except where FastMTTRefine acts: an MTT root and the nodes of its first pass are searched as in a picture whose
// limit is one lower, so they read that limit too (with the picture's, the rules would take the splits left out for
// searched and found useless). From 3 to 2 that changes the rules that compare depths and the row of the split-cost
// table; from 2 to 1 also QT is searched before BT / TT (QtbttExtraFast above 1), no stored result is reused, and the
// split-cost table is medium's. The nodes of the second pass read the picture's limit; the MTT root itself keeps
// the lower one until a split of the second pass is kept (useModeResult), so until then its own later splits read
// the lower limit's split-cost table, as in the research build this was measured with.
unsigned EncModeCtrl::xSearchMttDepthLimit( const CodingStructure& cs, const Partitioner& partitioner ) const
{
  if( !xRefinesMtt( cs, partitioner ) )
  {
    return partitioner.maxBTD;
  }
  // outside the two passes the only nodes FastMTTRefine acts on are the MTT roots
  const bool searchedWithLowerLimit = m_mttSearchPass != MTT_SEARCH_REFINE;
  return partitioner.maxBTD - ( searchedWithLowerLimit ? 1 : 0 );
}

// Asked by trySplit for a BT / TT split that would be searched now: true where the first pass leaves it out, as it
// makes CUs of the deepest MTT level. The node's parent then notes that its split that made this node has a search
// left out below it, which the second pass can make.
bool EncModeCtrl::xSkipsDeepestMttLevel( const CodingStructure& cs, const Partitioner& partitioner )
{
  if( m_mttSearchPass != MTT_SEARCH_CAPPED )
  {
    return false;
  }
  const bool makesDeepestLevel = partitioner.currMtDepth + 1 == partitioner.maxBTD + partitioner.currImplicitBtDepth;
  if( !makesDeepestLevel || !xRefinesMtt( cs, partitioner ) )
  {
    return false;
  }
  ComprCUCtx& parentCtx      = *( m_ComprCUCtxList.end() - 2 );
  parentCtx.refinableSplits |= 1 << partitioner.currPartLevel().split;
  return true;
}

// True below an MTT root for a child of the root that has a level of nodes between it and the deepest MTT level
// (limit 3). Such a child had its own splits searched in the first pass, so the second pass searches again only the
// ones chosen by their cost (xStoreSplitsToRefine).
bool EncModeCtrl::xChoosesSplitsToRefine( const Partitioner& partitioner ) const
{
  const int depth = freeMttDepth( partitioner );
  return isLuma( partitioner.chType ) && depth == 1 && depth + 1 < ( int ) partitioner.maxBTD;
}

// The BT / TT splits of a node (bits 1 << PartSplit) that the second pass searches again: those whose cost in the
// first pass is at most the node's best cost without QT, and that have a search left out below them
unsigned EncModeCtrl::xSplitsToRefine( const ComprCUCtx& cuECtx ) const
{
  const double bestCost = std::min( cuECtx.bestNoSplitCost, *std::min_element( std::begin( cuECtx.mttSplitCost ), std::end( cuECtx.mttSplitCost ) ) );
  unsigned     splits   = 0;
  for( int split = CU_HORZ_SPLIT; split <= CU_TRIV_SPLIT; split++ )
  {
    const double cost      = cuECtx.mttSplitCost[split - CU_HORZ_SPLIT];
    const bool   refinable = ( cuECtx.refinableSplits >> split ) & 1;
    if( refinable && cost != MAX_DOUBLE && cost <= bestCost )
    {
      splits |= 1 << split;
    }
  }
  return splits;
}

// End of the first pass at a child of the MTT root that had splits of its own searched (xChoosesSplitsToRefine):
// notes the splits the child searches again in the second pass, in the block info of its area (within one MTT root,
// each of its children has an area of its own), and tells the root that the split that made this child is worth
// searching again.
void EncModeCtrl::xStoreSplitsToRefine( const Partitioner& partitioner )
{
  const ComprCUCtx& cuECtx   = *comprCUCtx;
  const unsigned    toRefine = xSplitsToRefine( cuECtx );
  if( toRefine )
  {
    ComprCUCtx& rootCtx      = *( m_ComprCUCtxList.end() - 2 );
    rootCtx.refinableSplits |= 1 << partitioner.currPartLevel().split;
  }

  // The child's best split is searched again too, with or without a search left out below it: if a sibling makes the
  // root search this child again, the child must be able to find the result it had in the first pass.
  const PartSplit bestSplit   = cuECtx.bestCS ? getPartSplit( cuECtx.bestMode ) : CU_DONT_SPLIT;
  const unsigned  searchAgain = toRefine | ( isBtOrTt( bestSplit ) ? 1 << bestSplit : 0 );
  getBlkInfo( partitioner.currArea() ).refineSplits = ( uint8_t ) searchAgain;
}

bool EncModeCtrl::beginCappedSplitPass( const CodingStructure& cs, const Partitioner& partitioner )
{
  if( !xRefinesMtt( cs, partitioner ) || freeMttDepth( partitioner ) != 0 )
  {
    return false;
  }
  CHECKD( m_mttSearchPass != MTT_SEARCH_FULL, "MTT roots do not nest" );
  m_mttSearchPass = MTT_SEARCH_CAPPED;
  return true;
}

unsigned EncModeCtrl::beginSplitRefinePass( const Partitioner& partitioner )
{
  m_mttSearchPass = MTT_SEARCH_REFINE;
  // From here on nodes below the root are searched a second time: see BestEncInfoCache
  beginRevisit( ( int ) partitioner.getPartStack().size() );
  return xSplitsToRefine( *comprCUCtx );
}

void EncModeCtrl::endSplitRefinePass()
{
  m_mttSearchPass = MTT_SEARCH_FULL;
  endRevisit();
}

//////////////////////////////////////////////////////////////////////////
// FastAMVR: AMVR (MVDs in whole, 4 or half samples) pays where coding the MVD coarsely saves bits, or where the
// half-sample filter predicts better, and only where a searched motion vector is near the node's best mode at all.
// The motion search with quarter-sample MVDs and equal bi-prediction weights, made before it, shows where it cannot.
//////////////////////////////////////////////////////////////////////////

// The largest MVD component in quarter samples that a CU found by motion search codes: per list in use and per control
// point (one unless affine). The MVDs of the second list of an SMVD CU are not coded.
static int largestMvdInQuarterSamples( const CodingUnit& cu )
{
  static constexpr int QUARTER_SHIFT = MV_FRACTIONAL_BITS_INTERNAL - 2;   // MVDs are kept in 1/16 samples
  const int controlPoints = !cu.affine ? 1 : cu.affineType == AFFINEMODEL_6PARAM ? 3 : 2;
  int largest = 0;
  for( int list = 0; list < NUM_REF_PIC_LIST_01; list++ )
  {
    if( !( cu.interDir >> list & 1 ) || ( list == REF_PIC_LIST_1 && cu.smvdMode ) )
    {
      continue;
    }
    for( int point = 0; point < controlPoints; point++ )
    {
      largest = std::max( { largest, abs( cu.mvd[list][point].hor ) >> QUARTER_SHIFT, abs( cu.mvd[list][point].ver ) >> QUARTER_SHIFT } );
    }
  }
  return largest;
}

// Keeps the best cost of the motion search with quarter-sample MVDs and equal weights (called by useModeResult)
void EncModeCtrl::xNoteMotionSearchCost( const EncTestMode& encTestmode, const CodingStructure& tempCS, const bool useEDO )
{
  if( encTestmode.type != ETM_INTER_ME || tempCS.cost == MAX_DOUBLE || tempCS.cus.empty() || !comprCUCtx )
  {
    return;
  }
  const CodingUnit& cu   = *tempCS.cus.front();
  const double      cost = tempCS.cost + ( useEDO ? tempCS.costDbOffset : 0 );   // what useModeResult compares
  if( cu.imv == IMV_OFF && cu.BcwIdx == BCW_DEFAULT && cost < comprCUCtx->meCost )
  {
    comprCUCtx->meCost = cost;
  }
}

void EncModeCtrl::noteMotionSearchMvd( const CodingUnit& cu )
{
  if( m_pcEncCfg->m_fastAMVR && comprCUCtx )
  {
    comprCUCtx->meLargestMvd = largestMvdInQuarterSamples( cu );
  }
}

// True where the AMVR search of the current node is not made: its best mode bestCS is skip (as AMVR speeds 3 and 4
// do), or the motion search is far behind it or found MVDs too small to save bits on. The last two need a result of
// the motion search.
bool EncModeCtrl::skipsAmvrSearch( const CodingStructure& bestCS ) const
{
  if( !m_pcEncCfg->m_fastAMVR || !comprCUCtx || bestCS.cus.empty() )
  {
    return false;
  }
  const ComprCUCtx& cuECtx      = *comprCUCtx;
  const bool        hasMeResult = cuECtx.meLargestMvd >= 0 && cuECtx.meCost != MAX_DOUBLE;
  return bestCS.cus.front()->skip
      || ( hasMeResult && ( cuECtx.meCost > AMVR_ME_BEHIND_RATIO * bestCS.cost || cuECtx.meLargestMvd < AMVR_MIN_MVD ) );
}

// True where the AMVR search of the current node leaves out 4-sample MVDs
bool EncModeCtrl::skips4PelAmvr() const
{
  return m_pcEncCfg->m_fastAMVR && comprCUCtx && comprCUCtx->meLargestMvd >= 0 && comprCUCtx->meLargestMvd < AMVR_4PEL_MIN_MVD;
}

} // namespace vvenc

//! \}


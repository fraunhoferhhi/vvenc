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


/** \file     CodingStructure.h
 *  \brief    A class managing the coding information for a specific image part
 */

#include "CodingStructure.h"
#include "Unit.h"
#include "Slice.h"
#include "Picture.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

const UnitScale UnitScaleArray[NUM_CHROMA_FORMAT][MAX_NUM_COMP] =
{
  { {2,2}, {0,0}, {0,0} },  // 4:0:0
  { {2,2}, {1,1}, {1,1} },  // 4:2:0
  { {2,2}, {1,2}, {1,2} },  // 4:2:2
  { {2,2}, {2,2}, {2,2} }   // 4:4:4
};

// ---------------------------------------------------------------------------
// coding structure method definitions
// ---------------------------------------------------------------------------

CodingStructure::CodingStructure( XUCache& unitCache, std::mutex* mutex )
  : area            ()
  , picture         ( nullptr )
  , parent          ( nullptr )
  , refCS           ( nullptr )
  , bestCS          ( nullptr )
  , picHeader       ( nullptr )
  , m_isTuEnc       ( false )
  , m_cuCache       ( unitCache.cuCache )
  , m_tuCache       ( unitCache.tuCache )
  , m_unitCacheMutex( mutex )
  , bestParent      ( nullptr )
#if IBC_VTM
  , resetIBCBuffer  ( false )
#endif
{
  for( uint32_t i = 0; i < MAX_NUM_COMP; i++ )
  {
    m_coeffs[ i ] = nullptr;
    m_offsets[ i ] = 0;
  }

  for( uint32_t i = 0; i < MAX_NUM_CH; i++ )
  {
    m_cuPtr   [ i ] = nullptr;
    m_tuPtr   [ i ] = nullptr;
  }

  for( int i = 0; i < NUM_EDGE_DIR; i++ )
  {
    m_lfParam [ i ] = nullptr;
  }

  m_motionBuf = nullptr;
}

void CodingStructure::destroy()
{
  picture   = nullptr;
  parent    = nullptr;
  refCS     = nullptr;

  m_pred.destroy();
  m_resi.destroy();
  m_reco.destroy();
  m_rspreco.destroy();
  m_org = nullptr;
  m_rsporg = nullptr;

  destroyCoeffs();
  delete[] m_motionBuf;
  m_motionBuf = nullptr;

  destroyTempBuffers();

  if ( m_unitCacheMutex ) m_unitCacheMutex->lock();

  m_tuCache.cache( tus );
  m_cuCache.cache( cus );

  if ( m_unitCacheMutex ) m_unitCacheMutex->unlock();
}

void CodingStructure::releaseIntermediateData()
{
  clearTUs();
  clearCUs();
}

const int CodingStructure::signalModeCons( const PartSplit split, Partitioner &partitioner, const ModeType modeTypeParent ) const
{
  if (CS::isDualITree(*this) || modeTypeParent != MODE_TYPE_ALL || partitioner.currArea().chromaFormat == CHROMA_444 || partitioner.currArea().chromaFormat == CHROMA_400 )
    return LDT_MODE_TYPE_INHERIT;
  int minLumaArea = partitioner.currArea().lumaSize().area();
  if (split == CU_QUAD_SPLIT || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT) // the area is split into 3 or 4 parts
  {
    minLumaArea = minLumaArea >> 2;
  }
  else if (split == CU_VERT_SPLIT || split == CU_HORZ_SPLIT) // the area is split into 2 parts
  {
    minLumaArea = minLumaArea >> 1;
  }
  int minChromaBlock = minLumaArea >> (getChannelTypeScaleX(CH_C, partitioner.currArea().chromaFormat) + getChannelTypeScaleY(CH_C, partitioner.currArea().chromaFormat));
  bool is2xNChroma = (partitioner.currArea().chromaSize().width == 4 && split == CU_VERT_SPLIT) || (partitioner.currArea().chromaSize().width == 8 && split == CU_TRIV_SPLIT);
  return minChromaBlock >= 16 && !is2xNChroma ? LDT_MODE_TYPE_INHERIT : ((minLumaArea < 32) || slice->isIntra()) ? LDT_MODE_TYPE_INFER : LDT_MODE_TYPE_SIGNAL;
}

CodingUnit* CodingStructure::getLumaCU( const Position& pos )
{
  const ChannelType effChType = CH_L;
  const CompArea& _blk = area.blocks[effChType];
  CHECK( !_blk.contains( pos ), "must contain the pos" );

  return m_cuPtr[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];
}

CodingUnit* CodingStructure::getCU( const Position& pos, const ChannelType effChType, const TreeType _treeType )
{
  CHECKD(_treeType == TREE_C && effChType == CH_L && parent == nullptr && _treeType == TREE_C && effChType == CH_L, "parent shall be valid; consider using function getLumaCU()");

  CodingStructure* cs = _treeType == TREE_C && effChType == CH_L ? parent : this;
  while (cs && !cs->area.blocks[effChType].contains(pos)) cs = cs->parent;

  if (!cs)
  {
    return nullptr;
  }
  else
  {
    const Area& _blk = cs->area.blocks[effChType];
    return cs->m_cuPtr[effChType][rsAddr(pos, _blk.pos(), _blk.width, unitScale[effChType])];
  }
}

const CodingUnit* CodingStructure::getCU( const Position& pos, const ChannelType effChType, const TreeType _treeType ) const
{
  CHECKD(_treeType == TREE_C && effChType == CH_L && parent == nullptr && _treeType == TREE_C && effChType == CH_L, "parent shall be valid; consider using function getLumaCU()");

  const CodingStructure* cs = _treeType == TREE_C && effChType == CH_L ? parent : this;
  while (cs && !cs->area.blocks[effChType].contains(pos)) cs = cs->parent;

  if (!cs)
  {
    return nullptr;
  }
  else
  {
    const Area& _blk = cs->area.blocks[effChType];
    return cs->m_cuPtr[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];
  }
}

TransformUnit* CodingStructure::getTU( const Position& pos, const ChannelType effChType, const int subTuIdx )
{
  const CompArea& _blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if( parent ) return parent->getTU( pos, effChType );
    else         return nullptr;
  }
  else
  {
    TransformUnit* ptu = m_tuPtr[effChType][rsAddr(pos, _blk.pos(), _blk.width, unitScale[effChType])];
    unsigned idx = ptu->idx;
    if (!ptu && m_isTuEnc)
      return parent->getTU(pos, effChType);
    else
    {
      if (isLuma(effChType))
      {
        const TransformUnit& tu = *tus[idx - 1];
        if (tu.cu->ispMode)
        {
          if (subTuIdx == -1)
          {
            unsigned extraIdx = 0;
            while (!tus[idx - 1 + extraIdx]->blocks[getFirstComponentOfChannel(effChType)].contains(pos))
            {
              extraIdx++;
              CHECK(tus[idx - 1 + extraIdx]->cu->treeType == TREE_C,
                "tu searched by position points to a chroma tree CU");
              CHECK(extraIdx > 3, "extraIdx > 3");
            }
            return tus[idx - 1 + extraIdx];
          }
          return tus[idx - 1 + subTuIdx];
        }
      }
      return ptu;
    }
  }
}

const TransformUnit * CodingStructure::getTU( const Position& pos, const ChannelType effChType, const int subTuIdx ) const
{
  const CompArea& _blk = area.blocks[effChType];

  if( !_blk.contains( pos ) )
  {
    if( parent ) return parent->getTU( pos, effChType );
    else         return nullptr;
  }
  else
  {
    TransformUnit* ptu = m_tuPtr[effChType][rsAddr(pos, _blk.pos(), _blk.width, unitScale[effChType])];
    unsigned idx = ptu->idx;
    if (!ptu && m_isTuEnc)
      return parent->getTU(pos, effChType);
    else
    {
      if (isLuma(effChType))
      {
        const TransformUnit& tu = *tus[idx - 1];
        if (tu.cu->ispMode)
        {
          if (subTuIdx == -1)
          {
            unsigned extraIdx = 0;
            while (!tus[idx - 1 + extraIdx]->blocks[getFirstComponentOfChannel(effChType)].contains(pos))
            {
              extraIdx++;
              CHECK(tus[idx - 1 + extraIdx]->cu->treeType == TREE_C,
                "tu searched by position points to a chroma tree CU");
              CHECK(extraIdx > 3, "extraIdx > 3");
            }
            return tus[idx - 1 + extraIdx];
          }
          return tus[idx - 1 + subTuIdx];
        }
      }
      return ptu;
    }
  }
}

CodingUnit& CodingStructure::addCU( const UnitArea& unit, const ChannelType chType )
{
  if ( m_unitCacheMutex ) m_unitCacheMutex->lock();

  CodingUnit *cu = m_cuCache.get();

  if ( m_unitCacheMutex ) m_unitCacheMutex->unlock();

  cu->UnitArea::operator=( unit );
  cu->initData();
  cu->cs        = this;
  cu->slice     = nullptr;
  cu->next      = nullptr;
  cu->firstTU   = nullptr;
  cu->lastTU    = nullptr;
  cu->chType    = chType;

  CodingUnit *prevCU = m_numCUs > 0 ? cus.back() : nullptr;

  if( prevCU )
  {
    const int prevCuCtuRsAddr = getCtuAddr( recalcPosition( area.chromaFormat, prevCU->chType, CH_L, prevCU->blocks[prevCU->chType] ), *pcv );
    const int currCuCtuRsAddr = getCtuAddr( recalcPosition( area.chromaFormat,         chType, CH_L,     cu->blocks[        chType] ), *pcv );

    if( prevCuCtuRsAddr == currCuCtuRsAddr )
    {
      prevCU->next = cu;
    }
  }

  cus.push_back( cu );

  uint32_t idx = ++m_numCUs;
  cu->idx  = idx;
  cu->mvdL0SubPu = nullptr;

  if( isLuma( chType ) && unit.lheight() >= 8 && unit.lwidth() >= 8 && unit.Y().area() >= 128 )
  {
    CHECKD( m_dmvrMvCacheOffset >= m_dmvrMvCache.size(), "dmvr cache offset out of bounds" );

    cu->mvdL0SubPu       = &m_dmvrMvCache[m_dmvrMvCacheOffset];
    m_dmvrMvCacheOffset += std::max<int>( 1, unit.lwidth() >> DMVR_SUBCU_SIZE_LOG2 ) * std::max<int>( 1, unit.lheight() >> DMVR_SUBCU_SIZE_LOG2 );
  }

  uint32_t numCh = getNumberValidChannels( area.chromaFormat );

  for( uint32_t i = 0; i < numCh; i++ )
  {
    if( !cu->blocks[i].valid() )
    {
      continue;
    }

    const CompArea& _selfBlk = area.blocks[i];
    const CompArea     &_blk = cu-> blocks[i];

    const UnitScale& scale = unitScale[_blk.compID];
    const Area scaledSelf  = scale.scale( _selfBlk );
    const Area scaledBlk   = scale.scale(     _blk );
    CodingUnit **cuPtr     = m_cuPtr[i] + rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );

    CHECK( *cuPtr, "Overwriting a pre-existing value, should be '0'!" );

    g_pelBufOP.fillPtrMap( ( void** ) cuPtr, scaledSelf.width, scaledBlk.width, scaledBlk.height, ( void* ) cu );
  }

  return *cu;
}

TransformUnit& CodingStructure::addTU( const UnitArea& unit, const ChannelType chType, CodingUnit* cu )
{
  if ( m_unitCacheMutex ) m_unitCacheMutex->lock();

  TransformUnit *tu = m_tuCache.get();

  if ( m_unitCacheMutex ) m_unitCacheMutex->unlock();

  tu->UnitArea::operator=( unit );
  tu->initData();
  tu->next   = nullptr;
  tu->prev   = nullptr;
  tu->cs     = this;
  tu->cu     = cu;
  tu->chType = chType;

  TransformUnit *prevTU = m_numTUs > 0 ? tus.back() : nullptr;

  if( prevTU && prevTU->cu == tu->cu )
  {
    prevTU->next = tu;
    tu->prev     = prevTU;
  }

  tus.push_back( tu );

  if( tu->cu )
  {
    if( tu->cu->firstTU == nullptr )
    {
      tu->cu->firstTU = tu;
    }
    tu->cu->lastTU = tu;
  }

  uint32_t idx = ++m_numTUs;
  tu->idx = idx;

  TCoeff *coeffs[3] = { nullptr, nullptr, nullptr };

  uint32_t numCh = getNumberValidComponents( area.chromaFormat );

  for( uint32_t i = 0; i < numCh; i++ )
  {
    if( !tu->blocks[i].valid() )
    {
      continue;
    }

    if (i < getNumberValidChannels(area.chromaFormat))
    {
      const CompArea& _selfBlk = area.blocks[i];
      const CompArea     &_blk = tu-> blocks[i];

      bool isIspTu = tu->cu != nullptr && tu->cu->ispMode && isLuma( _blk.compID );

      bool isFirstIspTu = false;
      if( isIspTu )
      {
        isFirstIspTu = CU::isISPFirst( *tu->cu, _blk, getFirstComponentOfChannel( ChannelType( i ) ) );
      }
      if( !isIspTu || isFirstIspTu )
      {
        const UnitScale& scale = unitScale[_blk.compID];

        const Area scaledSelf  = scale.scale( _selfBlk );
        const Area scaledBlk   = isIspTu ? scale.scale( tu->cu->blocks[i] ) : scale.scale( _blk );
        TransformUnit **tuPtr  = m_tuPtr[i] + rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );

        CHECK( *tuPtr, "Overwriting a pre-existing value, should be '0'!" );

        g_pelBufOP.fillPtrMap( ( void** ) tuPtr, scaledSelf.width, scaledBlk.width, scaledBlk.height, ( void* ) tu );
      }
    }

    coeffs[i] = m_coeffs[i] + m_offsets[i];

    unsigned areaSize = tu->blocks[i].area();
    m_offsets[i] += areaSize;
  }

  tu->init( coeffs );

  return *tu;
}

void CodingStructure::addEmptyTUs( Partitioner &partitioner, CodingUnit* cu )
{
  const UnitArea& area    = partitioner.currArea();
  bool            split   = partitioner.canSplit(TU_MAX_TR_SPLIT, *this);
  const unsigned  trDepth = partitioner.currTrDepth;

  if( split )
  {
    partitioner.splitCurrArea( TU_MAX_TR_SPLIT, *this );
    do
    {
      addEmptyTUs( partitioner, cu );
    } while( partitioner.nextPart( *this ) );

    partitioner.exitCurrSplit();
  }
  else
  {
#if IBC_VTM
    TransformUnit& tu = addTU(CS::getArea(*this, area, partitioner.chType, TreeType(partitioner.treeType)), partitioner.chType, cu);
#else
    TransformUnit &tu = addTU( CS::getArea( *this, area, partitioner.chType, TREE_D ), partitioner.chType, cu );
#endif
    tu.depth = trDepth;
  }
}

CUTraverser CodingStructure::traverseCUs( const UnitArea& unit, const ChannelType effChType )
{
  //  CHECK( _treeType != treeType, "not good");
  CodingUnit* firstCU = getCU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType, TREE_D );
  CodingUnit* lastCU = firstCU;
  if( !CS::isDualITree( *this ) ) //for a more generalized separate tree
  {
    bool bContinue = true;
    CodingUnit* currCU = firstCU;
    while( bContinue )
    {
      if( currCU == nullptr )
      {
        bContinue = false;
        lastCU = currCU;
      }
      else if( currCU->chType != effChType )
      {
        lastCU = currCU;
        currCU = currCU->next;
      }
      else
      {
        if( unit.contains( *currCU ) )
        {
          lastCU = currCU;
          currCU = currCU->next;
        }
        else
        {
          bContinue = false;
          lastCU = currCU;
        }
      }
    }
  }
  else
  {
  do { } while( lastCU && (0 != ( lastCU = lastCU->next )) && unit.contains( *lastCU ) );
  }

  return CUTraverser( firstCU, lastCU );
}

TUTraverser CodingStructure::traverseTUs( const UnitArea& unit, const ChannelType effChType )
{
  TransformUnit* firstTU = getTU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  TransformUnit* lastTU  = firstTU;

  do { } while( lastTU && (0 != ( lastTU = lastTU->next )) && unit.contains( *lastTU ) );

  return TUTraverser( firstTU, lastTU );
}

cCUTraverser CodingStructure::traverseCUs( const UnitArea& unit, const ChannelType effChType ) const
{
  const CodingUnit* firstCU = getCU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType, TREE_D );
  const CodingUnit* lastCU  = firstCU;

  do { } while( lastCU && (0 != ( lastCU = lastCU->next )) && unit.contains( *lastCU ) );

  return cCUTraverser( firstCU, lastCU );
}

cTUTraverser CodingStructure::traverseTUs( const UnitArea& unit, const ChannelType effChType ) const
{
  const TransformUnit* firstTU = getTU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType );
  const TransformUnit* lastTU  = firstTU;

  do { } while( lastTU && (0 != ( lastTU = lastTU->next )) && unit.contains( *lastTU ) );

  return cTUTraverser( firstTU, lastTU );
}



LFPBuf CodingStructure::getLoopFilterParamBuf(const DeblockEdgeDir& edgeDir)
{
  return LFPBuf(m_lfParam[edgeDir], m_mapSize[0]);
}

const CLFPBuf CodingStructure::getLoopFilterParamBuf(const DeblockEdgeDir& edgeDir) const
{
  return CLFPBuf(m_lfParam[edgeDir], m_mapSize[0]);
}


// coding utilities

void CodingStructure::allocateVectorsAtPicLevel()
{
  const int  twice = ( !pcv->ISingleTree && slice->isIRAP() && pcv->chrFormat != CHROMA_400 ) ? 2 : 1;
  size_t allocSize = twice * unitScale[0].scale( area.blocks[0].size() ).area();

  cus.reserve( allocSize );
  tus.reserve( allocSize );
}



void CodingStructure::create(const ChromaFormat _chromaFormat, const Area& _area, const bool isTopLayer)
{
  createInternals( UnitArea( _chromaFormat, _area ), isTopLayer );

  if( isTopLayer ) return;

  m_reco.create( area );
  m_pred.create( area );
  m_resi.create( area );
  m_rspreco.create( CHROMA_400, area.Y() );
}

void CodingStructure::create(const UnitArea& _unit, const bool isTopLayer, const PreCalcValues* _pcv)
{
  pcv = _pcv;

  createInternals( _unit, isTopLayer );

  if( isTopLayer ) return;

  m_reco.create( area );
  m_pred.create( area );
  m_resi.create( area );
  m_rspreco.create( CHROMA_400, area.Y() );
}

void CodingStructure::createInternals( const UnitArea& _unit, const bool isTopLayer )
{
  area     = _unit;
  _maxArea = _unit;

  memcpy( unitScale, UnitScaleArray[area.chromaFormat], sizeof( unitScale ) );

  picture = nullptr;
  parent  = nullptr;
  refCS   = nullptr;

  unsigned _lumaAreaScaled = g_miScaling.scale( area.lumaSize() ).area();
  m_motionBuf = new MotionInfo[_lumaAreaScaled];

  if( isTopLayer )
  {
    motionLutBuf.resize( pcv->heightInCtus );
  }
  else
  {
    createCoeffs();
    createTempBuffers( false );
    initStructData();
  }
}

void CodingStructure::createTempBuffers( const bool isTopLayer )
{
  unsigned numCh = getNumberValidChannels( area.chromaFormat );

  for( unsigned i = 0; i < numCh; i++ )
  {
    Size allocArea  = area.blocks[i].size();
    m_mapSize[i]    = unitScale[i].scale(allocArea);

    unsigned _area  = unitScale[i].scale( area.blocks[i].size() ).area();

    m_cuPtr[i]      = _area > 0 ? new CodingUnit*    [_area] : nullptr;
    m_tuPtr[i]      = _area > 0 ? new TransformUnit* [_area] : nullptr;
  }

  for( unsigned i = 0; i < NUM_EDGE_DIR; i++ )
  {
    m_lfParam[i] = ( isTopLayer && m_mapSize[0].area() > 0 ) ? ( LoopFilterParam* ) xMalloc( LoopFilterParam, m_mapSize[0].area() ) : nullptr;
  }

  unsigned _maxNumDmvrMvs = ( area.lwidth() >> 3 ) * ( area.lheight() >> 3 );
  m_dmvrMvCache.resize( _maxNumDmvrMvs );
}

void CodingStructure::destroyTempBuffers()
{
  for( uint32_t i = 0; i < MAX_NUM_CH; i++ )
  {
    delete[] m_cuPtr[i];
    m_cuPtr[i] = nullptr;

    delete[] m_tuPtr[i];
    m_tuPtr[i] = nullptr;
  }

  for( int i = 0; i < NUM_EDGE_DIR; i++ )
  {
    xFree( m_lfParam[i] );
    m_lfParam[i] = nullptr;
  }

  // swap the contents of the vector so that memory released
  std::vector<Mv>().swap( m_dmvrMvCache );
}

void CodingStructure::addMiToLut(static_vector<HPMVInfo, MAX_NUM_HMVP_CANDS> &lut, const HPMVInfo &mi)
{
  size_t currCnt = lut.size();

  bool pruned      = false;
  int  sameCandIdx = 0;

  for (int idx = 0; idx < currCnt; idx++)
  {
    if (lut[idx] == mi)
    {
      sameCandIdx = idx;
      pruned      = true;
      break;
    }
  }

  if (pruned || currCnt == lut.capacity())
  {
    lut.erase(lut.begin() + sameCandIdx);
  }

  lut.push_back(mi);
}

void CodingStructure::rebindPicBufs()
{
  CHECK( parent, "rebindPicBufs can only be used for the top level CodingStructure" );

  if( !picture->m_bufs[ PIC_RECONSTRUCTION ].bufs.empty() ) m_reco.createFromBuf( picture->m_bufs[ PIC_RECONSTRUCTION ] );
  else                                                         m_reco.destroy();
  if( !picture->m_bufs[ PIC_PREDICTION     ].bufs.empty() ) m_pred.createFromBuf( picture->m_bufs[ PIC_PREDICTION ] );
  else                                                         m_pred.destroy();
  if( !picture->m_bufs[ PIC_RESIDUAL       ].bufs.empty() ) m_resi.createFromBuf( picture->m_bufs[ PIC_RESIDUAL ] );
  else                                                         m_resi.destroy();
}

void CodingStructure::createCoeffs()
{
  const unsigned numComp = getNumberValidComponents( area.chromaFormat );
  for( unsigned i = 0; i < numComp; i++ )
  {
    unsigned _area = area.blocks[i].area();

    m_coeffs[i] = _area > 0 ? ( TCoeff* ) xMalloc( TCoeff, _area ) : nullptr;
  }

  for( unsigned i = 0; i < numComp; i++ )
  {
    m_offsets[i] = 0;
  }
}

void CodingStructure::destroyCoeffs()
{
  for( uint32_t i = 0; i < MAX_NUM_COMP; i++ )
  {
    if( m_coeffs[i] ) { xFree( m_coeffs[i] ); m_coeffs[i] = nullptr; }
  }
}

void CodingStructure::initSubStructure( CodingStructure& subStruct, const ChannelType _chType, const UnitArea& subArea, const bool isTuEnc, PelStorage* pOrgBuffer, PelStorage* pRspBuffer )
{
  CHECK( this == &subStruct, "Trying to init self as sub-structure" );

  subStruct.parent = this;

  if( pOrgBuffer ) pOrgBuffer->compactResize( subArea );
  UnitArea subAreaLuma = subArea.singleChan( CH_L );
  subAreaLuma.blocks.resize( 1 );
  if( pRspBuffer ) pRspBuffer->compactResize( subAreaLuma );

  subStruct.m_org    = (pOrgBuffer) ? pOrgBuffer : m_org;
  subStruct.m_rsporg = (pRspBuffer) ? pRspBuffer : m_rsporg;

  subStruct.compactResize( subArea );

  subStruct.costDbOffset = 0;

  if( parent )
  {
    // allow this to be false at the top level (need for edge CTU's)
    CHECKD( !area.contains( subArea ), "Trying to init sub-structure not contained in the parent" );
  }

  subStruct.parent    = this;
  subStruct.picture   = picture;
  subStruct.refCS     = picture->cs;

  subStruct.sps       = sps;
  subStruct.vps       = vps;
  subStruct.pps       = pps;
  subStruct.picHeader = picHeader;

  memcpy(subStruct.alfAps, alfAps, sizeof(alfAps));

  subStruct.lmcsAps   = lmcsAps;

  subStruct.slice     = slice;
  subStruct.baseQP    = baseQP;
  subStruct.prevQP[_chType]
                      = prevQP[_chType];
  subStruct.pcv       = pcv;

  subStruct.m_isTuEnc = isTuEnc;

  if( nullptr == parent )
  {
    int ctuPosY = subArea.ly() >> pcv->maxCUSizeLog2;
    subStruct.motionLut = motionLutBuf[ctuPosY];
  }
  else
  {
    subStruct.motionLut = motionLut;
  }

  subStruct.initStructData( currQP[_chType] );

  if( isTuEnc )
  {
    CHECKD( area != subStruct.area, "Trying to init sub-structure for TU-encoding of incompatible size" );

    for( const auto &pcu : cus )
    {
      CodingUnit &cu = subStruct.addCU( *pcu, _chType );

      cu = *pcu;
    }
  }
}

void CodingStructure::useSubStructure( const CodingStructure& subStruct, const ChannelType chType, const TreeType _treeType, const UnitArea& subArea, const bool cpyReco )
{
  UnitArea clippedArea = clipArea( subArea, *picture );


  if( cpyReco )
  {
    CPelUnitBuf subRecoBuf = subStruct.getRecoBuf( clippedArea );

    if( parent )
    {
      // copy data to picture
      getRecoBuf( clippedArea ).copyFrom( subRecoBuf );
    }

    picture->getRecoBuf( clippedArea ).copyFrom( subRecoBuf );
  }

  if (!subStruct.m_isTuEnc && ((!slice->isIntra() || slice->sps->IBC) && chType != CH_C))
  {
    // copy motion buffer
    MotionBuf ownMB  = getMotionBuf          ( clippedArea );
    CMotionBuf subMB = subStruct.getMotionBuf( clippedArea );

    ownMB.copyFrom( subMB );

    if( nullptr == parent )
    {
      int ctuPosY = subStruct.area.ly() >> pcv->maxCUSizeLog2;
      motionLutBuf[ctuPosY] = subStruct.motionLut;
    }
    else
    {
      motionLut = subStruct.motionLut;
    }
  }

  fracBits += subStruct.fracBits;
  dist     += subStruct.dist;
  cost     += subStruct.cost;
  costDbOffset += subStruct.costDbOffset;

  if( parent )
  {
    // allow this to be false at the top level
    CHECKD( !area.contains( subArea ), "Trying to use a sub-structure not contained in self" );
  }

  // copy the CUs over
  if( subStruct.m_isTuEnc )
  {
    // don't copy if the substruct was created for encoding of the TUs
  }
  else
  {
    for( const auto &pcu : subStruct.cus )
    {
      // add an analogue CU into own CU store
      const UnitArea& cuPatch = *pcu;
      CodingUnit &cu = addCU( cuPatch, pcu->chType );

      // copy the CU info from subPatch
      cu = *pcu;
    }
  }

  // copy the TUs over
  for( const auto &ptu : subStruct.tus )
  {
    // add an analogue TU into own TU store
    const UnitArea& tuPatch = *ptu;
    TransformUnit& tu = addTU( tuPatch, ptu->chType, getCU( tuPatch.blocks[ptu->chType].pos(), ptu->chType, _treeType ) );

    // copy the TU info from subPatch
    tu = *ptu;
  }
}

void CodingStructure::copyStructure( const CodingStructure& other, const ChannelType chType, const TreeType _treeType, const bool copyTUs, const bool copyRecoBuf )
{
  fracBits      = other.fracBits;
  dist          = other.dist;
  cost          = other.cost;
  costDbOffset  = other.costDbOffset;
  CHECKD( area != other.area, "Incompatible sizes" );

  const UnitArea dualITreeArea = CS::getArea( *this, area, chType, _treeType );

  // copy the CUs over
  for (const auto &pcu : other.cus)
  {
    if( !dualITreeArea.contains( *pcu ) )
    {
      continue;
    }
    // add an analogue CU into own CU store
    const UnitArea& cuPatch = *pcu;

    CodingUnit &cu = addCU(cuPatch, pcu->chType);

    // copy the CU info from subPatch
    cu = *pcu;
  }

  if (!other.slice->isIntra() || other.slice->sps->IBC)
  {
    // copy motion buffer
    MotionBuf  ownMB = getMotionBuf();
    CMotionBuf subMB = other.getMotionBuf();

    ownMB.copyFrom( subMB );

    motionLut = other.motionLut;
  }

  if( copyTUs )
  {
    // copy the TUs over
    for( const auto &ptu : other.tus )
    {
      if( !dualITreeArea.contains( *ptu ) )
      {
        continue;
      }
      // add an analogue TU into own TU store
      const UnitArea& tuPatch = *ptu;

      TransformUnit& tu = addTU( tuPatch, ptu->chType, getCU( tuPatch.blocks[ptu->chType], ptu->chType, _treeType) );

      // copy the TU info from subPatch
      tu = *ptu;
    }
  }

  if( copyRecoBuf )
  {
    CPelUnitBuf recoBuf = other.getRecoBuf( area );

    if( parent )
    {
      // copy data to self for neighbors
      getRecoBuf( area ).copyFrom( recoBuf );
    }

    // copy data to picture
    picture->getRecoBuf( area ).copyFrom( recoBuf );
  }
}

void CodingStructure::compactResize( const UnitArea& _area )
{
  UnitArea areaLuma = _area.singleChan( CH_L );
  areaLuma.blocks.resize( 1 );

  m_pred   .compactResize( _area );
  m_reco   .compactResize( _area );
  m_resi   .compactResize( _area );
  m_rspreco.compactResize( areaLuma );

  for( uint32_t i = 0; i < _area.blocks.size(); i++ )
  {
    CHECK( _maxArea.blocks[i].area() < _area.blocks[i].area(), "Trying to init sub-structure of incompatible size" );
  }

  area = _area;
}

void CodingStructure::initStructData( const int QP, const bool skipMotBuf, const UnitArea* _area )
{
  clearTUs();
  clearCUs();

  if( _area ) compactResize( *_area );

  if( QP < MAX_INT )
  {
    currQP[0] = currQP[1] = QP;
  }

  if (!skipMotBuf && (!parent || ((!slice->isIntra() || slice->sps->IBC) && !m_isTuEnc)))
  {
    getMotionBuf().memset( 0 );
  }

  m_dmvrMvCacheOffset = 0;

  fracBits      = 0;
  dist          = 0;
  cost          = MAX_DOUBLE;
  lumaCost      = MAX_DOUBLE;
  costDbOffset  = 0;
  interHad      = MAX_DISTORTION;
}


void CodingStructure::clearTUs()
{
  int numCh = getNumberValidChannels( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    size_t _area = ( area.blocks[i].area() >> unitScale[i].area );

    memset( m_tuPtr[i], 0, sizeof( *m_tuPtr[0] ) * _area );

  }

  numCh = getNumberValidComponents( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    m_offsets[i] = 0;
  }

  for( auto &pcu : cus )
  {
    pcu->firstTU = pcu->lastTU = nullptr;
  }

  if ( m_unitCacheMutex ) m_unitCacheMutex->lock();
  m_tuCache.cache( tus );
  if ( m_unitCacheMutex ) m_unitCacheMutex->unlock();

  m_numTUs = 0;
}

void CodingStructure::clearCUs()
{
  int numCh = getNumberValidChannels( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    memset( m_cuPtr[i], 0, sizeof( *m_cuPtr[0] ) * unitScale[i].scaleArea( area.blocks[i].area() ) );
  }

  if ( m_unitCacheMutex ) m_unitCacheMutex->lock();
  m_cuCache.cache( cus );
  if ( m_unitCacheMutex ) m_unitCacheMutex->unlock();

  m_numCUs = 0;
}

MotionBuf CodingStructure::getMotionBuf( const Area& _area )
{
  const CompArea& _luma = area.Y();

  CHECKD( !_luma.contains( _area ), "Trying to access motion information outside of this coding structure" );

  const Area miArea   = g_miScaling.scale( _area );
  const Area selfArea = g_miScaling.scale( _luma );

  return MotionBuf( m_motionBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
}

const CMotionBuf CodingStructure::getMotionBuf( const Area& _area ) const
{
  const CompArea& _luma = area.Y();

  CHECKD( !_luma.contains( _area ), "Trying to access motion information outside of this coding structure" );

  const Area miArea   = g_miScaling.scale( _area );
  const Area selfArea = g_miScaling.scale( _luma );

  return MotionBuf( m_motionBuf + rsAddr( miArea.pos(), selfArea.pos(), selfArea.width ), selfArea.width, miArea.size() );
}

MotionInfo& CodingStructure::getMotionInfo( const Position& pos )
{
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );

  // bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *( m_motionBuf + miPos.y * stride + miPos.x );
}

const MotionInfo& CodingStructure::getMotionInfo( const Position& pos ) const
{
  CHECKD( !area.Y().contains( pos ), "Trying to access motion information outside of this coding structure" );

  // bypass the motion buf calling and get the value directly
  const unsigned stride = g_miScaling.scaleHor( area.lumaSize().width );
  const Position miPos  = g_miScaling.scale( pos - area.lumaPos() );

  return *( m_motionBuf + miPos.y * stride + miPos.x );
}

PelBuf CodingStructure::getBuf( const CompArea& blk, const PictureType type )
{
  if (!blk.valid())
  {
    return PelBuf();
  }

  const ComponentID compID = blk.compID;

  PelStorage* buf = type == PIC_PREDICTION ? &m_pred : ( type == PIC_RESIDUAL ? &m_resi : ( type == PIC_RECONSTRUCTION ? &m_reco : nullptr ) );
  if (type == PIC_ORIGINAL)
  {
    buf = m_org;
  }
  else if( type == PIC_ORIGINAL_RSP)
  {
    buf = m_rsporg;
  }
  else if (type == PIC_ORIGINAL_RSP_REC)
  {
    buf = &m_rspreco;
  }

  CHECK( !buf, "Unknown buffer requested" );

  CHECKD( !area.blocks[compID].contains( blk ), "Buffer not contained in self requested" );

  CompArea cFinal = blk;
  cFinal.relativeTo( area.blocks[compID] );

  if( !parent && ( type == PIC_RESIDUAL || type == PIC_PREDICTION ) )
  {
    cFinal.x &= ( pcv->maxCUSizeMask >> getComponentScaleX( blk.compID, blk.chromaFormat ) );
    cFinal.y &= ( pcv->maxCUSizeMask >> getComponentScaleY( blk.compID, blk.chromaFormat ) );
  }

  return buf->getBuf( cFinal );
}

const CPelBuf CodingStructure::getBuf( const CompArea& blk, const PictureType type ) const
{
  if (!blk.valid())
  {
    return PelBuf();
  }

  const ComponentID compID = blk.compID;

  const PelStorage* buf = type == PIC_PREDICTION ? &m_pred : ( type == PIC_RESIDUAL ? &m_resi : ( type == PIC_RECONSTRUCTION ? &m_reco : nullptr ) );
  if (type == PIC_ORIGINAL)
  {
    buf = m_org;
  }
  else if( type == PIC_ORIGINAL_RSP)
  {
    buf = m_rsporg;
  }
  else if (type == PIC_ORIGINAL_RSP_REC)
  {
    buf = &m_rspreco;
  }

  CHECK( !buf, "Unknown buffer requested" );

  CHECKD( !area.blocks[compID].contains( blk ), "Buffer not contained in self requested" );

  CompArea cFinal = blk;
  cFinal.relativeTo( area.blocks[compID] );

  if( !parent && ( type == PIC_RESIDUAL || type == PIC_PREDICTION ) )
  {
    cFinal.x &= ( pcv->maxCUSizeMask >> getComponentScaleX( blk.compID, blk.chromaFormat ) );
    cFinal.y &= ( pcv->maxCUSizeMask >> getComponentScaleY( blk.compID, blk.chromaFormat ) );
  }

  return buf->getBuf( cFinal );
}

PelUnitBuf CodingStructure::getBuf( const UnitArea& unit, const PictureType type )
{
  // no parent fetching for buffers
  if( area.chromaFormat == CHROMA_400 )
  {
    return PelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return PelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

const CPelUnitBuf CodingStructure::getBuf( const UnitArea& unit, const PictureType type ) const
{
  // no parent fetching for buffers
  if( area.chromaFormat == CHROMA_400 )
  {
    return CPelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return CPelUnitBuf( area.chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

const CodingUnit* CodingStructure::getCURestricted( const Position& pos, const CodingUnit& curCu, const ChannelType _chType ) const
{
  if( sps->entropyCodingSyncEnabled )
  {
    const int xshift = pcv->maxCUSizeLog2 - getChannelTypeScaleX( _chType, curCu.chromaFormat );
    const int yshift = pcv->maxCUSizeLog2 - getChannelTypeScaleY( _chType, curCu.chromaFormat );
    if( (pos.x >> xshift) > (curCu.blocks[_chType].x >> xshift) || (pos.y >> yshift) > (curCu.blocks[_chType].y >> yshift) )
      return nullptr;
  }
  const CodingUnit* cu = getCU( pos, _chType, curCu.treeType );
  return ( cu && CU::isSameSliceAndTile( *cu, curCu ) && ( cu->cs != curCu.cs || cu->idx <= curCu.idx ) ) ? cu : nullptr;
}

const CodingUnit* CodingStructure::getCURestricted( const Position& pos, const Position curPos, const unsigned curSliceIdx, const unsigned curTileIdx, const ChannelType _chType, const TreeType _treeType ) const
{
  if( sps->entropyCodingSyncEnabled )
  {
    const int xshift = pcv->maxCUSizeLog2 - getChannelTypeScaleX( _chType, area.chromaFormat );
    const int yshift = pcv->maxCUSizeLog2 - getChannelTypeScaleY( _chType, area.chromaFormat );
    if( (pos.x >> xshift) > (curPos.x >> xshift) || (pos.y >> yshift) > (curPos.y >> yshift) )
      return nullptr;
  }
  const CodingUnit* cu = getCU( pos, _chType, _treeType );

  return ( cu && cu->slice->independentSliceIdx == curSliceIdx && cu->tileIdx == curTileIdx ) ? cu : nullptr;
}

const TransformUnit* CodingStructure::getTURestricted( const Position& pos, const TransformUnit& curTu, const ChannelType _chType ) const
{
  if( sps->entropyCodingSyncEnabled )
  {
    const int xshift = pcv->maxCUSizeLog2 - getChannelTypeScaleX( _chType, curTu.chromaFormat );
    const int yshift = pcv->maxCUSizeLog2 - getChannelTypeScaleY( _chType, curTu.chromaFormat );
    if( (pos.x >> xshift) > (curTu.blocks[_chType].x >> xshift) || (pos.y >> yshift) > (curTu.blocks[_chType].y >> yshift) )
      return nullptr;
  }
  const TransformUnit* tu = getTU( pos, _chType );
  return ( tu && CU::isSameSliceAndTile( *tu->cu, *curTu.cu ) && ( tu->cs != curTu.cs || tu->idx <= curTu.idx ) ) ? tu : nullptr;
}

} // namespace vvenc

//! \}


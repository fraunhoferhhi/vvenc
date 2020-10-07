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
//  , m_puCache       ( unitCache.puCache )
  , m_tuCache       ( unitCache.tuCache )
  , m_unitCacheMutex( mutex )
  , bestParent      ( nullptr )
{
  for( uint32_t i = 0; i < MAX_NUM_COMP; i++ )
  {
    m_coeffs[ i ] = nullptr;
    m_pcmbuf[ i ] = nullptr;
    m_offsets[ i ] = 0;
  }

  for( uint32_t i = 0; i < MAX_NUM_CH; i++ )
  {
    m_runType [ i ] = nullptr;
    m_cuPtr   [ i ] = nullptr;
//    m_puPtr   [ i ] = nullptr;
    m_tuPtr   [ i ] = nullptr;
    m_isDecomp[ i ] = nullptr;
  }

  for( int i = 0; i < NUM_EDGE_DIR; i++ )
  {
    m_lfParam [ i ] = nullptr;
  }

  m_motionBuf = nullptr;
  features.resize( NUM_ENC_FEATURES );
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

  for( uint32_t i = 0; i < MAX_NUM_CH; i++ )
  {
    delete[] m_isDecomp[ i ];
    m_isDecomp[ i ] = nullptr;

    delete[] m_cuPtr[ i ];
    m_cuPtr[ i ] = nullptr;

//    delete[] m_puPtr[ i ];
//    m_puPtr[ i ] = nullptr;

    delete[] m_tuPtr[ i ];
    m_tuPtr[ i ] = nullptr;

  }

  for( int i = 0; i < NUM_EDGE_DIR; i++ )
  {
    xFree( m_lfParam[ i ] );
    m_lfParam[ i ] = nullptr;
  }

  delete[] m_motionBuf;
  m_motionBuf = nullptr;


  if ( m_unitCacheMutex ) m_unitCacheMutex->lock();

  m_tuCache.cache( tus );
//  m_puCache.cache( pus );
  m_cuCache.cache( cus );

  if ( m_unitCacheMutex ) m_unitCacheMutex->unlock();
}

void CodingStructure::releaseIntermediateData()
{
  clearTUs();
//  clearPUs();
  clearCUs();
}

bool CodingStructure::isDecomp( const Position& pos, const ChannelType effChType )
{
  if( area.blocks[effChType].contains( pos ) )
  {
    return m_isDecomp[effChType][rsAddr( pos, area.blocks[effChType], area.blocks[effChType].width, unitScale[effChType] )];
  }
  else if( parent )
  {
    return parent->isDecomp( pos, effChType );
  }
  else
  {
    return false;
  }
}

bool CodingStructure::isDecomp( const Position& pos, const ChannelType effChType ) const
{
  if( area.blocks[effChType].contains( pos ) )
  {
    return m_isDecomp[effChType][rsAddr( pos, area.blocks[effChType], area.blocks[effChType].width, unitScale[effChType] )];
  }
  else if( parent )
  {
    return parent->isDecomp( pos, effChType );
  }
  else
  {
    return false;
  }
}

void CodingStructure::setDecomp(const CompArea& _area, const bool _isCoded /*= true*/)
{
  const UnitScale& scale = unitScale[_area.compID];

  AreaBuf<bool> isCodedBlk( m_isDecomp[toChannelType( _area.compID )] + rsAddr( _area, area.blocks[_area.compID].pos(), area.blocks[_area.compID].width, scale ),
                            area.blocks[_area.compID].width >> scale.posx,
                            _area.width                     >> scale.posx,
                            _area.height                    >> scale.posy);
  isCodedBlk.fill( _isCoded );
}

void CodingStructure::setDecomp(const UnitArea& _area, const bool _isCoded /*= true*/)
{
  for( uint32_t i = 0; i < _area.blocks.size(); i++ )
  {
    if( _area.blocks[i].valid() ) setDecomp( _area.blocks[i], _isCoded );
  }
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
  const CompArea& _blk = area.blocks[effChType];

  if( !_blk.contains( pos ) || (_treeType == TREE_C && effChType == CH_L) )
  {
    //keep this check, which is helpful to identify bugs
    if( _treeType == TREE_C && effChType == CH_L )
    {
      CHECK( parent == nullptr, "parent shall be valid; consider using function getLumaCU()" );
    }
    if( parent ) return parent->getCU( pos, effChType, TREE_D );
    else         return nullptr;
  }
  else
  {
    return m_cuPtr[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];
  }
}

const CodingUnit* CodingStructure::getCU( const Position& pos, const ChannelType effChType, const TreeType _treeType ) const
{
  const CompArea& _blk = area.blocks[effChType];

  if( !_blk.contains( pos ) || (_treeType == TREE_C && effChType == CH_L) )
  {
    if( _treeType == TREE_C && effChType == CH_L )
    {
      CHECK( parent == nullptr, "parent shall be valid; consider using function getLumaCU()" );
    }
    if( parent ) return parent->getCU( pos, effChType, TREE_D );
    else         return nullptr;
  }
  else
  {
    return m_cuPtr[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];
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
#if 1 && !ISP_VVC 
    TransformUnit *ptu = m_tuPtr[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if( !ptu && m_isTuEnc ) return parent->getTU( pos, effChType );
    else                    return ptu;
#else
#if ISP_VVC
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
#else
    // TODO: fix when implementing ISP, see in the decoder impl
    const unsigned idx = m_tuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if( idx != 0 )
    {
      unsigned extraIdx = 0;
      if( isLuma( effChType ) )
      {
        const TransformUnit& tu = *tus[idx - 1];

        if( tu.cu->ispMode ) // Intra SubPartitions mode
        {
          //we obtain the offset to index the corresponding sub-partition
          if( subTuIdx != -1 )
          {
            extraIdx = subTuIdx;
          }
          else
          {
            while( !tus[idx - 1 + extraIdx]->blocks[getFirstComponentOfChannel( effChType )].contains( pos ) )
            {
              extraIdx++;
              CHECK( tus[idx - 1 + extraIdx]->treeType == TREE_C, "tu searched by position points to a chroma tree CU" );
              CHECK( extraIdx > 3, "extraIdx > 3" );
            }
          }
        }
      }
      return tus[idx - 1 + extraIdx];
    }
    else if( m_isTuEnc ) return parent->getTU( pos, effChType );
    else                 return nullptr;
#endif
#endif
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
#if 1 && !ISP_VVC
    TransformUnit *ptu = m_tuPtr[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];

    if( !ptu && m_isTuEnc ) return parent->getTU( pos, effChType );
    else                    return ptu;
#else
#if ISP_VVC
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
#else
    // TODO: fix when implementing ISP, see in the decoder impl
    const unsigned idx = m_tuIdx[effChType][rsAddr( pos, _blk.pos(), _blk.width, unitScale[effChType] )];
    if( idx != 0 )
    {
      unsigned extraIdx = 0;
      if( isLuma( effChType ) )
      {
        const TransformUnit& tu = *tus[idx - 1];
        if( tu.cu->ispMode ) // Intra SubPartitions mode
        {
          //we obtain the offset to index the corresponding sub-partition
          if( subTuIdx != -1 )
          {
            extraIdx = subTuIdx;
          }
          else
          {
            while ( !tus[idx - 1 + extraIdx]->blocks[getFirstComponentOfChannel( effChType )].contains(pos) )
            {
              extraIdx++;
              CHECK( tus[idx - 1 + extraIdx]->treeType == TREE_C, "tu searched by position points to a chroma tree CU" );
              CHECK( extraIdx > 3, "extraIdx > 3" );
            }
          }
        }
      }
      return tus[idx - 1 + extraIdx];
    }
    else if( m_isTuEnc ) return parent->getTU( pos, effChType );
    else                 return nullptr;
#endif
#endif
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
//  cu->cu        = nullptr;
  cu->firstTU   = nullptr;
  cu->lastTU    = nullptr;
  cu->chType    = chType;

  CodingUnit *prevCU = m_numCUs > 0 ? cus.back() : nullptr;

  if( prevCU )
  {
    prevCU->next = cu;
  }

  cus.push_back( cu );

  uint32_t idx = ++m_numCUs;
  cu->idx  = idx;

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
    CodingUnit **cuPtr    = m_cuPtr[i] + rsAddr( scaledBlk.pos(), scaledSelf.pos(), scaledSelf.width );
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
  tu->cu     = m_isTuEnc ? cus[0] : cu;
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
  tu->idx  = idx;

  TCoeff *coeffs[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
  Pel    *pcmbuf[5] = { nullptr, nullptr, nullptr, nullptr, nullptr };
  bool   *runType[5]   = { nullptr, nullptr, nullptr, nullptr, nullptr };

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
    pcmbuf[i] = m_pcmbuf[i] + m_offsets[i];
    if (i < MAX_NUM_CH)
    {
      if (m_runType[i] != nullptr) runType[i] = m_runType[i] + m_offsets[i];
    }

    unsigned areaSize = tu->blocks[i].area();
    m_offsets[i] += areaSize;
  }

  tu->init( coeffs, pcmbuf, runType);

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
    TransformUnit &tu = addTU( CS::getArea( *this, area, partitioner.chType, TREE_D ), partitioner.chType, cu );
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

cCUSecureTraverser CodingStructure::secureTraverseCUs( const UnitArea& unit, const ChannelType effChType ) const
{
//  CHECK( _treeType != treeType, "not good");
  const CodingUnit* firstCU = getCU( isLuma( effChType ) ? unit.lumaPos() : unit.chromaPos(), effChType, TREE_D );
  const CodingUnit* lastCU = firstCU;
  if( !CS::isDualITree( *this ) ) //for a more generalized separate tree
  {
    bool bContinue = true;
    const CodingUnit* currCU = firstCU;
    while( bContinue )
    {
      if( currCU == nullptr )
      {
        bContinue = false;
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
        }
      }
    }
  }
  else
  {
    do { lastCU = lastCU->next; } while(  lastCU->next && unit.contains( *lastCU->next ) );
  }

  return cCUSecureTraverser( firstCU, lastCU );
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
  area = _unit;

  memcpy( unitScale, UnitScaleArray[area.chromaFormat], sizeof( unitScale ) );

  picture = nullptr;
  parent  = nullptr;
  refCS   = nullptr;

  unsigned numCh = getNumberValidChannels(area.chromaFormat);

  for (unsigned i = 0; i < numCh; i++)
  {
    Size allocArea = area.blocks[i].size();
    m_mapSize[i] = unitScale[i].scale(allocArea);

    unsigned _area = unitScale[i].scale( area.blocks[i].size() ).area();

    m_cuPtr[i]    = _area > 0 ? new CodingUnit*    [_area] : nullptr;
    m_tuPtr[i]    = _area > 0 ? new TransformUnit* [_area] : nullptr;
    m_isDecomp[i] = _area > 0 ? new bool           [_area] : nullptr;
  }

    for( unsigned i = 0; i < NUM_EDGE_DIR; i++ )
    {
    m_lfParam[i] = ( isTopLayer && m_mapSize[0].area() > 0 ) ? ( LoopFilterParam* ) xMalloc( LoopFilterParam, m_mapSize[0].area() ) : nullptr;
    }

  numCh = getNumberValidComponents(area.chromaFormat);

  for (unsigned i = 0; i < numCh; i++)
  {
    m_offsets[i] = 0;
  }

  if( isTopLayer )
  {
    motionLutBuf.resize( pcv->heightInCtus );
  }
  else
  {
    createCoeffs();
  }

  unsigned _lumaAreaScaled = g_miScaling.scale( area.lumaSize() ).area();
  m_motionBuf       = new MotionInfo[_lumaAreaScaled];
  initStructData();
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
    m_pcmbuf[i] = _area > 0 ? ( Pel*    ) xMalloc( Pel,    _area ) : nullptr;
  }

  const unsigned numCh = getNumberValidChannels( area.chromaFormat );
  for( unsigned i = 0; i < numCh; i++ )
  {
    unsigned _area = area.blocks[i].area();
    m_runType[i]   = _area > 0 ? ( bool*  ) xMalloc( bool, _area ) : nullptr;
  }
}

void CodingStructure::destroyCoeffs()
{
  for( uint32_t i = 0; i < MAX_NUM_COMP; i++ )
  {
    if( m_coeffs[i] ) { xFree( m_coeffs[i] ); m_coeffs[i] = nullptr; }
    if( m_pcmbuf[i] ) { xFree( m_pcmbuf[i] ); m_pcmbuf[i] = nullptr; }
}
  for (uint32_t i = 0; i < MAX_NUM_CH; i++)
  {
    if (m_runType[i])   { xFree(m_runType[i]);   m_runType[i]   = nullptr; }
  }
}

void CodingStructure::initSubStructure( CodingStructure& subStruct, const ChannelType _chType, const UnitArea& subArea, const bool isTuEnc, PelStorage* pOrgBuffer, PelStorage* pRspBuffer )
{
  CHECK( this == &subStruct, "Trying to init self as sub-structure" );

  subStruct.m_org = ( pOrgBuffer ) ? pOrgBuffer : m_org;
  subStruct.m_rsporg = ( pRspBuffer ) ? pRspBuffer : m_rsporg;

  subStruct.costDbOffset = 0;

  for( uint32_t i = 0; i < subStruct.area.blocks.size(); i++ )
  {
    CHECKD( subStruct.area.blocks[i].size() != subArea.blocks[i].size(), "Trying to init sub-structure of incompatible size" );

    subStruct.area.blocks[i].pos() = subArea.blocks[i].pos();
  }

  if( parent )
  {
    // allow this to be false at the top level (need for edge CTU's)
    CHECKD( !area.contains( subStruct.area ), "Trying to init sub-structure not contained in the parent" );
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
    int ctuPosY = subStruct.area.ly() >> pcv->maxCUSizeLog2;
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

    unsigned numComp = getNumberValidChannels( area.chromaFormat );
    for( unsigned i = 0; i < numComp; i++)
    {
      ::memcpy( subStruct.m_isDecomp[i], m_isDecomp[i], (unitScale[i].scale( area.blocks[i].size() ).area() * sizeof( bool ) ) );
    }
  }
}

void CodingStructure::useSubStructure( const CodingStructure& subStruct, const ChannelType chType, const TreeType _treeType, const UnitArea& subArea, const bool cpyReco )
{
  UnitArea clippedArea = clipArea( subArea, *picture );

  setDecomp( clippedArea );

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

void CodingStructure::initStructData( const int QP, const bool skipMotBuf )
{
//  clearPUs();
  clearTUs();
  clearCUs();

  if( QP < MAX_INT )
  {
    currQP[0] = currQP[1] = QP;
  }

  if (!skipMotBuf && (!parent || ((!slice->isIntra() || slice->sps->IBC) && !m_isTuEnc)))
  {
    getMotionBuf()      .memset( 0 );
  }

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

    memset( m_isDecomp[i], false, sizeof( *m_isDecomp[0] ) * _area );
    memset( m_tuPtr   [i],     0, sizeof( *m_tuPtr   [0] ) * _area );
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

/*
void CodingStructure::clearPUs()
{
  int numCh = getNumberValidChannels( area.chromaFormat );
  for( int i = 0; i < numCh; i++ )
  {
    memset( m_puPtr[i], 0, sizeof( *m_puPtr[0] ) * unitScale[i].scaleArea( area.blocks[i].area() ) );
  }

  if ( m_unitCacheMutex ) m_unitCacheMutex->lock();
  m_puCache.cache( pus );
  if ( m_unitCacheMutex ) m_unitCacheMutex->unlock();

  m_numPUs = 0;

  for( auto &pcu : cus )
  {
    pcu->cu = nullptr;
  }
}
*/
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
#if ISP_VVC
  else if (type == PIC_ORIGINAL_RSP_REC)
  {
    buf = &m_rspreco;
  }
#endif

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
#if ISP_VVC
  else if (type == PIC_ORIGINAL_RSP_REC)
  {
    buf = &m_rspreco;
  }
#endif

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


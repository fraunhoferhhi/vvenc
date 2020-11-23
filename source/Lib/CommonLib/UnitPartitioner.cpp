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


/** \file     UnitPartitioner.h
 *  \brief    Provides a class for partitioning management
 */

#include "UnitPartitioner.h"
#include "CodingStructure.h"
#include "Unit.h"
#include "Slice.h"
#include "UnitTools.h"
#include "Picture.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

PartLevel::PartLevel()
: split               ( CU_DONT_SPLIT )
, parts               (               )
, idx                 ( 0u            )
, checkdIfImplicit    ( false         )
, isImplicit          ( false         )
, implicitSplit       ( CU_DONT_SPLIT )
, firstSubPartSplit   ( CU_DONT_SPLIT )
, canQtSplit          ( true          )
, qgEnable            ( true          )
, qgChromaEnable      ( true          )
, modeType            ( MODE_TYPE_ALL )
{
}

PartLevel::PartLevel( const PartSplit _split, const Partitioning& _parts )
: split               ( _split        )
, parts               ( _parts        )
, idx                 ( 0u            )
, checkdIfImplicit    ( false         )
, isImplicit          ( false         )
, implicitSplit       ( CU_DONT_SPLIT )
, firstSubPartSplit   ( CU_DONT_SPLIT )
, canQtSplit          ( true          )
, qgEnable            ( true          )
, qgChromaEnable      ( true          )
, modeType            ( MODE_TYPE_ALL )
{
}

PartLevel::PartLevel( const PartSplit _split, Partitioning&& _parts )
: split               ( _split                               )
, parts               ( std::forward<Partitioning>( _parts ) )
, idx                 ( 0u                                   )
, checkdIfImplicit    ( false                                )
, isImplicit          ( false                                )
, implicitSplit       ( CU_DONT_SPLIT                        )
, firstSubPartSplit   ( CU_DONT_SPLIT                        )
, canQtSplit          ( true                                 )
, qgEnable            ( true                                 )
, qgChromaEnable      ( true                                 )
, modeType            ( MODE_TYPE_ALL )
{
}

void PartLevel::init()
{
  split               = CU_DONT_SPLIT;
  idx                 = 0u;
  checkdIfImplicit    = false;
  isImplicit          = false;
  implicitSplit       = CU_DONT_SPLIT;
  firstSubPartSplit   = CU_DONT_SPLIT;
  canQtSplit          = true;
  qgEnable            = true;
  qgChromaEnable      = true;
  modeType            = MODE_TYPE_ALL;

  parts.clear();
}

//////////////////////////////////////////////////////////////////////////
// Partitioner class
//////////////////////////////////////////////////////////////////////////

SplitSeries Partitioner::getSplitSeries() const
{
  SplitSeries splitSeries = 0;
  SplitSeries depth = 0;

  for( const auto &level : m_partStack )
  {
    if( level.split == CTU_LEVEL ) continue;
    else splitSeries += static_cast< SplitSeries >( level.split ) << ( depth * SPLIT_DMULT );

    depth++;
  }

  return splitSeries;
}

ModeTypeSeries Partitioner::getModeTypeSeries() const
{
  ModeTypeSeries modeTypeSeries = 0;
  int depth = 0;

  for( const auto &level : m_partStack )
  {
    if( level.split == CTU_LEVEL ) continue;
    else modeTypeSeries += static_cast<int>(level.modeType) << (depth * 3);

    depth++;
  }

  return modeTypeSeries;
}

bool Partitioner::isSepTree( const CodingStructure &cs )
{
  return treeType != TREE_D || CS::isDualITree( cs );
}

void Partitioner::setCUData( CodingUnit& cu )
{
  cu.depth       = currDepth;
  cu.btDepth     = currBtDepth;
  cu.mtDepth     = currMtDepth;
  cu.qtDepth     = currQtDepth;
  cu.splitSeries = getSplitSeries();
  cu.modeTypeSeries = getModeTypeSeries();
  cu.treeType    = treeType; 
  cu.modeType    = modeType; 

}

void Partitioner::copyState( const Partitioner& other )
{
  m_partStack = other.m_partStack;
  currBtDepth = other.currBtDepth;
  currQtDepth = other.currQtDepth;
  currDepth   = other.currDepth;
  currMtDepth = other.currMtDepth;
  currTrDepth = other.currTrDepth;
  currSubdiv  = other.currSubdiv;
  currQgPos   = other.currQgPos;
  currQgChromaPos = other.currQgChromaPos;
  currImplicitBtDepth
              = other.currImplicitBtDepth;
  chType      = other.chType;
#ifdef _DEBUG
  m_currArea  = other.m_currArea;
#endif
}

void Partitioner::setMaxMinDepth( unsigned& minDepth, unsigned& maxDepth, const CodingStructure& cs ) const
{
  unsigned          stdMinDepth = 0;
  unsigned          stdMaxDepth = cs.pcv->getMaxDepth( cs.slice->sliceType, chType );
  const Position    pos         = currArea().blocks[chType].pos();
  const unsigned    curSliceIdx = cs.slice->independentSliceIdx;
  const unsigned    curTileIdx  = cs.pps->getTileIdx( currArea().lumaPos() );

  const CodingUnit* cuLeft        = cs.getCURestricted( pos.offset( -1,                               0 ), pos, curSliceIdx, curTileIdx, chType, treeType );
  const CodingUnit* cuBelowLeft   = cs.getCURestricted( pos.offset( -1, currArea().blocks[chType].height), pos, curSliceIdx, curTileIdx, chType, treeType );
  const CodingUnit* cuAbove       = cs.getCURestricted( pos.offset(  0,                              -1 ), pos, curSliceIdx, curTileIdx, chType, treeType );
  const CodingUnit* cuAboveRight  = cs.getCURestricted( pos.offset( currArea().blocks[chType].width, -1 ), pos, curSliceIdx, curTileIdx, chType, treeType );

  minDepth = stdMaxDepth;
  maxDepth = stdMinDepth;

  if( cuLeft )
  {
    minDepth = std::min<unsigned>( minDepth, cuLeft->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuLeft->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  if( cuBelowLeft )
  {
    minDepth = std::min<unsigned>( minDepth, cuBelowLeft->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuBelowLeft->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  if( cuAbove )
  {
    minDepth = std::min<unsigned>( minDepth, cuAbove->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuAbove->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  if( cuAboveRight )
  {
    minDepth = std::min<unsigned>( minDepth, cuAboveRight->qtDepth );
    maxDepth = std::max<unsigned>( maxDepth, cuAboveRight->qtDepth );
  }
  else
  {
    minDepth = stdMinDepth;
    maxDepth = stdMaxDepth;
  }

  minDepth = ( minDepth >= 1 ? minDepth - 1 : 0 );
  maxDepth = std::min<unsigned>( stdMaxDepth, maxDepth + 1 );
}

void Partitioner::initCtu( const UnitArea& ctuArea, const ChannelType _chType, const Slice& slice )
{
#if _DEBUG
  m_currArea = ctuArea;
#endif
  currDepth   = 0;
  currTrDepth = 0;
  currBtDepth = 0;
  currMtDepth = 0;
  currQtDepth = 0;
  currSubdiv  = 0;
  currQgPos   = ctuArea.lumaPos();
  currQgChromaPos = ctuArea.chromaFormat != CHROMA_400 ? ctuArea.chromaPos() : Position();
  currImplicitBtDepth = 0;
  chType      = _chType;

  const PreCalcValues& pcv = *slice.pps->pcv;
  
  maxBTD      = pcv.getMaxMTTDepth( slice, chType );
  maxBtSize   = pcv.getMaxBtSize  ( slice, chType );
  minTSize    = pcv.getMinTSize   ( slice, chType );
  maxTtSize   = pcv.getMaxTtSize  ( slice, chType );
  minQtSize   = pcv.getMinQtSize  ( slice, chType );

  m_partStack.clear();
  m_partStack.resize_noinit( 1 );

  m_partStack.back().init();
  m_partStack.back().split = CTU_LEVEL;
  m_partStack.back().parts = Partitioning{ ctuArea };

  treeType = TREE_D;
  modeType = MODE_TYPE_ALL;
}

void Partitioner::splitCurrArea( const PartSplit split, const CodingStructure& cs )
{
  if ((split != TU_1D_HORZ_SPLIT) && (split != TU_1D_VERT_SPLIT))
  {
    CHECKD(!canSplit(split, cs), "Trying to apply a prohibited split!");
  }

  bool isImplicit = isSplitImplicit( split, cs );
  bool canQtSplit = canSplit( CU_QUAD_SPLIT, cs );
  bool qgEnable = currQgEnable();
  bool qgChromaEnable = currQgChromaEnable();

  const UnitArea& area = currArea();
  m_partStack.resize_noinit( m_partStack.size() + 1 );
  PartLevel& back = m_partStack.back();
  back.init();
  back.split = split;

  switch( split )
  {
  case CU_QUAD_SPLIT:
    PartitionerImpl::getCUSubPartitions( back.parts, area, cs, split );
    back.modeType = modeType;
    break;
  case CU_HORZ_SPLIT:
  case CU_VERT_SPLIT:
    PartitionerImpl::getCUSubPartitions( back.parts, area, cs, split );
    back.modeType = modeType;
    break;
  case CU_TRIH_SPLIT:
  case CU_TRIV_SPLIT:
    PartitionerImpl::getCUSubPartitions( back.parts, area, cs, split );
    back.modeType = modeType;
    break;
  case TU_MAX_TR_SPLIT:
    PartitionerImpl::getMaxTuTiling( back.parts, area, cs );
    break;
  case SBT_VER_HALF_POS0_SPLIT:
  case SBT_VER_HALF_POS1_SPLIT:
  case SBT_HOR_HALF_POS0_SPLIT:
  case SBT_HOR_HALF_POS1_SPLIT:
  case SBT_VER_QUAD_POS0_SPLIT:
  case SBT_VER_QUAD_POS1_SPLIT:
  case SBT_HOR_QUAD_POS0_SPLIT:
  case SBT_HOR_QUAD_POS1_SPLIT:
    PartitionerImpl::getSbtTuTiling( back.parts, area, cs, split );
    break;
  case TU_1D_HORZ_SPLIT:
  case TU_1D_VERT_SPLIT:
  {
    PartitionerImpl::getTUIntraSubPartitions(back.parts, area, cs, split, TREE_D);
    break;
  }
  default:
    THROW( "Unknown split mode" );
    break;
  }

  currDepth++;
  currSubdiv++;
#if _DEBUG
  m_currArea = m_partStack.back().parts.front();
#endif

  if ((split == TU_MAX_TR_SPLIT) || (split == TU_1D_HORZ_SPLIT) || (split == TU_1D_VERT_SPLIT))
  {
    currTrDepth++;
  }
  else if( split >= SBT_VER_HALF_POS0_SPLIT && split <= SBT_HOR_QUAD_POS1_SPLIT )
  {
    currTrDepth++;
  }
  else
  {
    currTrDepth = 0;
  }

  if( split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT )
  {
    currBtDepth++;
    if( isImplicit ) currImplicitBtDepth++;
    currMtDepth++;

    if( split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT )
    {
      // first and last part of triple split are equivalent to double bt split
      currBtDepth++;
      currSubdiv++;
    }
    m_partStack.back().canQtSplit = canQtSplit;
  }
  else if( split == CU_QUAD_SPLIT )
  {
    CHECK( currBtDepth > 0, "Cannot split a non-square area other than with a binary split" );
    CHECK( currMtDepth > 0, "Cannot split a non-square area other than with a binary split" );
    currMtDepth = 0;
    currBtDepth = 0;
    currQtDepth++;
    currSubdiv++;
  }

  qgEnable       &= (currSubdiv <= (cs.slice->isIntra() ? cs.slice->picHeader->cuQpDeltaSubdivIntra : cs.slice->picHeader->cuQpDeltaSubdivInter ));
  qgChromaEnable &= (currSubdiv <= (cs.slice->isIntra() ? cs.slice->picHeader->cuChromaQpOffsetSubdivIntra : cs.slice->picHeader->cuChromaQpOffsetSubdivInter ));
  m_partStack.back().qgEnable       = qgEnable;
  m_partStack.back().qgChromaEnable = qgChromaEnable;
  if (qgEnable)
    currQgPos = currArea().lumaPos();
  if (qgChromaEnable)
    currQgChromaPos = currArea().chromaPos();
}

void Partitioner::canSplit( const CodingStructure &cs, bool& canNo, bool& canQt, bool& canBh, bool& canBv, bool& canTh, bool& canTv )
{
  const PartSplit implicitSplit = m_partStack.back().checkdIfImplicit ? m_partStack.back().implicitSplit : getImplicitSplit( cs );

  canNo = canQt = canBh = canTh = canBv = canTv = true;
  bool canBtt = currMtDepth < (maxBTD + currImplicitBtDepth);

  // the minimal and maximal sizes are given in luma samples
  const CompArea&  area  = currArea().Y();
  const CompArea  *areaC = (chType == CH_C) ? &(currArea().Cb()) : nullptr;
        PartLevel& level = m_partStack.back();

  const PartSplit lastSplit = level.split;
  const PartSplit parlSplit = lastSplit == CU_TRIH_SPLIT ? CU_HORZ_SPLIT : CU_VERT_SPLIT;

  // don't allow QT-splitting below a BT split
  if( lastSplit != CTU_LEVEL && lastSplit != CU_QUAD_SPLIT ) canQt = false;
  // minQtSize is in luma samples unit
  const unsigned minQTThreshold = minQtSize >> ((area.chromaFormat == CHROMA_400) ? 0 : ((int) getChannelTypeScaleX(CH_C, area.chromaFormat) - (int) getChannelTypeScaleY(CH_C, area.chromaFormat)));
  if( area.width <= minQTThreshold )                         canQt = false;
  if( areaC && areaC->width <= MIN_DUALTREE_CHROMA_WIDTH ) canQt = false;
  if( treeType == TREE_C )
  {
    canQt = canBh = canTh = canBv = canTv = false;
    return;
  }
  if( implicitSplit != CU_DONT_SPLIT )
  {
    canNo = canTh = canTv = false;

    canBh = implicitSplit == CU_HORZ_SPLIT;
    canBv = implicitSplit == CU_VERT_SPLIT;
    if (areaC && areaC->width == 4) canBv = false;
    if( !canBh && !canBv && !canQt ) canQt = true;
    return;
  }

  if( ( lastSplit == CU_TRIH_SPLIT || lastSplit == CU_TRIV_SPLIT ) && currPartIdx() == 1 )
  {
    canBh = parlSplit != CU_HORZ_SPLIT;
    canBv = parlSplit != CU_VERT_SPLIT;
  }

  if( canBtt && ( area.width <= minTSize && area.height <= minTSize ) )
  {
    canBtt = false;
  }
  if( canBtt && ( area.width > maxBtSize || area.height > maxBtSize )
      && ( ( area.width > maxTtSize || area.height > maxTtSize ) ) )
  {
    canBtt = false;
  }

  if( !canBtt )
  {
    canBh = canTh = canBv = canTv = false;

    return;
  }

  if( area.width > maxBtSize || area.height > maxBtSize )
  {
    canBh = canBv = false;
  }

  // specific check for BT splits
  if( area.height <= minTSize )                            canBh = false;
  if( area.width > MAX_TB_SIZEY && area.height <= MAX_TB_SIZEY ) canBh = false;
  if( areaC && areaC->width * areaC->height <= MIN_DUALTREE_CHROMA_SIZE )     canBh = false;
  if( area.width <= minTSize )                              canBv = false;
  if( area.width <= MAX_TB_SIZEY && area.height > MAX_TB_SIZEY ) canBv = false;
  if (areaC && (areaC->width * areaC->height <= MIN_DUALTREE_CHROMA_SIZE || areaC->width == 4))     canBv = false;
  if( modeType == MODE_TYPE_INTER && area.width * area.height == 32 )  canBv = canBh = false;
  if( area.height <= 2 * minTSize || area.height > maxTtSize || area.width > maxTtSize )
                                                                                       canTh = false;
  if( area.width > MAX_TB_SIZEY || area.height > MAX_TB_SIZEY )  canTh = false;
  if( areaC && areaC->width * areaC->height <= MIN_DUALTREE_CHROMA_SIZE*2 )     canTh = false;
  if( area.width <= 2 * minTSize || area.width > maxTtSize || area.height > maxTtSize )
                                                                                       canTv = false;
  if( area.width > MAX_TB_SIZEY || area.height > MAX_TB_SIZEY )  canTv = false;
  if (areaC && (areaC->width * areaC->height <= MIN_DUALTREE_CHROMA_SIZE * 2 || areaC->width == 8))     canTv = false;
  if( modeType == MODE_TYPE_INTER && area.width * area.height == 64 )  canTv = canTh = false;
}

bool Partitioner::canSplit( const PartSplit split, const CodingStructure &cs )
{
  const CompArea area       = currArea().Y();
  const unsigned maxTrSize  = cs.sps->getMaxTbSize();

  bool canNo, canQt, canBh, canTh, canBv, canTv;

  canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv );
  switch( split )
  {
  case CTU_LEVEL:
    THROW( "Checking if top level split is possible" );
    return true;
    break;
  case TU_MAX_TR_SPLIT:
    return area.width > maxTrSize || area.height > maxTrSize;
    break;
  case SBT_VER_HALF_POS0_SPLIT:
  case SBT_VER_HALF_POS1_SPLIT:
  case SBT_HOR_HALF_POS0_SPLIT:
  case SBT_HOR_HALF_POS1_SPLIT:
  case SBT_VER_QUAD_POS0_SPLIT:
  case SBT_VER_QUAD_POS1_SPLIT:
  case SBT_HOR_QUAD_POS0_SPLIT:
  case SBT_HOR_QUAD_POS1_SPLIT:
    return currTrDepth == 0;
    break;
  case CU_QUAD_SPLIT:
    return canQt;
  case CU_DONT_SPLIT:
    return canNo;
  case CU_HORZ_SPLIT:
    return canBh;
  case CU_VERT_SPLIT:
    return canBv;
  case CU_TRIH_SPLIT:
    return canTh;
  case CU_TRIV_SPLIT:
    return canTv;
  case CU_MT_SPLIT:
    return ( canBh || canTh || canBv || canTv );
  case CU_BT_SPLIT:
    return ( canBh || canBv );
  break;
  default:
    THROW( "Unknown split mode" );
    return false;
    break;
  }

  return true;
}

bool Partitioner::canSplitISP(const PartSplit split, const CodingStructure& cs, CodingUnit& cu)
{
  // const PartSplit implicitSplit = getImplicitSplit(cs);
  const UnitArea& area = currArea();

  switch (split)
  {
  case TU_1D_HORZ_SPLIT:
  {
    return area.lheight() == cu.lheight();
  }
  case TU_1D_VERT_SPLIT:
  {
    return area.lwidth() == cu.lwidth();
  }
  case TU_MAX_TR_SPLIT:
  {
    // this split is performed implicitly with the other splits
    return false;
  }
  default: THROW("Unknown 1-D split mode"); break;
  }
}

bool Partitioner::isSplitImplicit( const PartSplit split, const CodingStructure &cs )
{
  return split == getImplicitSplit( cs );
}

PartSplit Partitioner::getImplicitSplit( const CodingStructure &cs )
{
  if( m_partStack.back().checkdIfImplicit )
  {
    return m_partStack.back().implicitSplit;
  }

  PartSplit split = CU_DONT_SPLIT;

  if( split == CU_DONT_SPLIT )
  {
    const bool isBlInPic = cs.picture->Y().contains( currArea().Y().bottomLeft() );
    const bool isTrInPic = cs.picture->Y().contains( currArea().Y().topRight() );

    const CompArea& area      = currArea().Y();
    const bool isBtAllowed    = area.width <= maxBtSize && area.height <= maxBtSize && currMtDepth < (maxBTD + currImplicitBtDepth);
    // minQtSize is in luma samples unit
    const unsigned minQTThreshold = minQtSize >> ((area.chromaFormat == CHROMA_400) ? 0 : ((int) getChannelTypeScaleX(CH_C, area.chromaFormat) - (int) getChannelTypeScaleY(CH_C, area.chromaFormat)));
    const bool isQtAllowed    = area.width > minQTThreshold && currBtDepth == 0;

    if( !isBlInPic && !isTrInPic && isQtAllowed )
    {
      split = CU_QUAD_SPLIT;
    }
    else if( !isBlInPic && isBtAllowed && area.width <= MAX_TB_SIZEY )
    {
      split = CU_HORZ_SPLIT;
    }
    else if( !isTrInPic && isBtAllowed && area.height <= MAX_TB_SIZEY )
    {
      split = CU_VERT_SPLIT;
    }
    else if( !isBlInPic || !isTrInPic )
    {
      split = CU_QUAD_SPLIT;
    }
    if (CS::isDualITree(cs) && (currArea().Y().width > 64 || currArea().Y().height > 64))
    {
      split = CU_QUAD_SPLIT;
    }
    if( (!isBlInPic || !isTrInPic) && split == CU_DONT_SPLIT )
    {
      split = CU_QUAD_SPLIT;
    }
  }

  m_partStack.back().checkdIfImplicit = true;
  m_partStack.back().isImplicit = split != CU_DONT_SPLIT;
  m_partStack.back().implicitSplit = split;

  return split;
}

void Partitioner::exitCurrSplit()
{
  PartSplit currSplit = m_partStack.back().split;
  unsigned  currIdx = m_partStack.back().idx;

  m_partStack.pop_back();

  CHECK( currDepth == 0, "depth is '0', although a split was performed" );
  currDepth--;
  currSubdiv--;
  if( currQgEnable() )
    currQgPos = currArea().lumaPos();
  if( currArea().chromaFormat != CHROMA_400 && currQgChromaEnable() )
    currQgChromaPos = currArea().chromaPos();
#if _DEBUG
  m_currArea = m_partStack.back().parts[m_partStack.back().idx];
#endif

  if( currSplit == CU_HORZ_SPLIT || currSplit == CU_VERT_SPLIT || currSplit == CU_TRIH_SPLIT || currSplit == CU_TRIV_SPLIT )
  {
    CHECK( !m_partStack.back().checkdIfImplicit, "Didn't check if the current split is implicit" );
    CHECK( currBtDepth == 0, "BT depth is '0', athough a BT split was performed" );
    CHECK( currMtDepth == 0, "MT depth is '0', athough a BT split was performed" );
    currMtDepth--;
    if( m_partStack.back().isImplicit ) currImplicitBtDepth--;
    currBtDepth--;
    if( ( currSplit == CU_TRIH_SPLIT || currSplit == CU_TRIV_SPLIT ) && currIdx != 1 )
    {
      CHECK( currBtDepth == 0, "BT depth is '0', athough a TT split was performed" );
      currBtDepth--;
      currSubdiv--;
    }
  }
  else if( currSplit == TU_MAX_TR_SPLIT )
  {
    CHECK( currTrDepth == 0, "TR depth is '0', although a TU split was performed" );
    currTrDepth--;
  }
  else if( currSplit >= SBT_VER_HALF_POS0_SPLIT && currSplit <= SBT_HOR_QUAD_POS1_SPLIT )
  {
    CHECK( currTrDepth == 0, "TR depth is '0', although a TU split was performed" );
    currTrDepth--;
  }
  else if ((currSplit == TU_1D_HORZ_SPLIT) || (currSplit == TU_1D_VERT_SPLIT))
  {
    CHECK(currTrDepth == 0, "TR depth is '0', although a TU split was performed");
    currTrDepth--;
  }
  else
  {
    CHECK( currTrDepth > 0, "RQT found with QTBT partitioner" );

    CHECK( currQtDepth == 0, "QT depth is '0', although a QT split was performed" );
    currQtDepth--;
    currSubdiv--;
  }
}

bool Partitioner::nextPart( const CodingStructure &cs, bool autoPop /*= false*/ )
{
  const Position& prevPos = currArea().blocks[chType].pos();

  unsigned currIdx = ++m_partStack.back().idx;

  m_partStack.back().checkdIfImplicit = false;
  m_partStack.back().isImplicit = false;

  if( currIdx == 1 )
  {
    const CodingUnit* prevCU = cs.getCU( prevPos, chType, treeType );
    m_partStack.back().firstSubPartSplit = prevCU ? CU::getSplitAtDepth( *prevCU, currDepth ) : CU_DONT_SPLIT;
  }

  if( currIdx < m_partStack.back().parts.size() )
  {
    if( m_partStack.back().split == CU_TRIH_SPLIT || m_partStack.back().split == CU_TRIV_SPLIT )
    {
      // adapt the current bt depth
      if( currIdx == 1 ) currBtDepth--;
      else               currBtDepth++;
      if( currIdx == 1 ) currSubdiv--;
      else               currSubdiv++;
    }
  if( currQgEnable() )
    currQgPos = currArea().lumaPos();
  if( currQgChromaEnable() )
    currQgChromaPos = currArea().chromaPos();
#if _DEBUG
    m_currArea = m_partStack.back().parts[currIdx];
#endif
    return true;
  }
  else
  {
    if( autoPop ) exitCurrSplit();
    return false;
  }
}

bool Partitioner::hasNextPart()
{
  return ( ( m_partStack.back().idx + 1 ) < m_partStack.back().parts.size() );
}

//////////////////////////////////////////////////////////////////////////
// PartitionerFactory
//////////////////////////////////////////////////////////////////////////

Partitioner* PartitionerFactory::get( const Slice& slice )
{
  return new Partitioner;
}

//////////////////////////////////////////////////////////////////////////
// Partitioner methods describing the actual partitioning logic
//////////////////////////////////////////////////////////////////////////

void PartitionerImpl::getCUSubPartitions( Partitioning& sub, const UnitArea& cuArea, const CodingStructure &cs, const PartSplit _splitType /*= CU_QUAD_SPLIT*/ )
{
  const PartSplit splitType = _splitType;

  if( splitType == CU_QUAD_SPLIT )
  {
    if( !cs.pcv->noChroma2x2 )
    {
      sub.resize( 4, cuArea );

      for( uint32_t i = 0; i < 4; i++ )
      {
        for( auto &blk : sub[i].blocks )
        {
          blk.height >>= 1;
          blk.width  >>= 1;
          if( i >= 2 ) blk.y += blk.height;
          if( i &  1 ) blk.x += blk.width;
        }

        CHECK( sub[i].lumaSize().height < MIN_TB_SIZEY, "the split causes the block to be smaller than the minimal TU size" );
      }
    }
    else
    {
      const uint32_t minCUSize = 1 << cs.sps->log2MinCodingBlockSize;

      bool canSplit = cuArea.lumaSize().width > minCUSize && cuArea.lumaSize().height > minCUSize;

      Partitioning& ret = sub;

      if( canSplit )
      {
        ret.resize( 4 );

        if( cuArea.chromaFormat == CHROMA_400 )
        {
          CompArea  blkY = cuArea.Y();
          blkY.width >>= 1;
          blkY.height >>= 1;
          ret[0]  = UnitArea( cuArea.chromaFormat, blkY );
          blkY.x += blkY.width;
          ret[1]  = UnitArea( cuArea.chromaFormat, blkY );
          blkY.x -= blkY.width;
          blkY.y += blkY.height;
          ret[2]  = UnitArea( cuArea.chromaFormat, blkY );
          blkY.x += blkY.width;
          ret[3]  = UnitArea( cuArea.chromaFormat, blkY );
        }
        else
        {
          for( uint32_t i = 0; i < 4; i++ )
          {
            ret[i] = cuArea;

            CompArea& blkY  = ret[i].Y();
            CompArea& blkCb = ret[i].Cb();
            CompArea& blkCr = ret[i].Cr();

            blkY.width  /= 2;
            blkY.height /= 2;

            // TODO: get those params from SPS
            if( blkCb.width > 4 )
            {
              blkCb.width  /= 2;
              blkCb.height /= 2;
              blkCr.width  /= 2;
              blkCr.height /= 2;
            }
            else if( i > 0 )
            {
              blkCb = CompArea();
              blkCr = CompArea();
            }

            if( ( i & 1 ) == 1 )
            {
              blkY.x  += blkY .width;
              blkCb.x += blkCb.width;
              blkCr.x += blkCr.width;
            }

            if( i > 1 )
            {
              blkY.y  += blkY .height;
              blkCb.y += blkCb.height;
              blkCr.y += blkCr.height;
            }
          }
        }
      }
    }
  }
  else if( splitType == CU_HORZ_SPLIT )
  {
    sub.resize(2, cuArea);

    for (uint32_t i = 0; i < 2; i++)
    {
      for (auto &blk : sub[i].blocks)
      {
        blk.height >>= 1;
        if (i == 1) blk.y += blk.height;
      }

      CHECK(sub[i].lumaSize().height < MIN_TB_SIZEY, "the cs split causes the block to be smaller than the minimal TU size");
    }
  }
  else if( splitType == CU_VERT_SPLIT )
  {
    sub.resize( 2, cuArea );

    for( uint32_t i = 0; i < 2; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 1;
        if( i == 1 ) blk.x += blk.width;
      }

      CHECK( sub[i].lumaSize().width < MIN_TB_SIZEY, "the split causes the block to be smaller than the minimal TU size" );
    }
  }
  else if( splitType == CU_TRIH_SPLIT )
  {
    sub.resize( 3, cuArea );

    for( int i = 0; i < 3; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.height >>= 1;
        if( ( i + 1 ) & 1 ) blk.height >>= 1;
        if( i == 1 )        blk.y       +=     blk.height / 2;
        if( i == 2 )        blk.y       += 3 * blk.height;
      }

      CHECK( sub[i].lumaSize().height < MIN_TB_SIZEY, "the cs split causes the block to be smaller than the minimal TU size" );
    }
  }
  else if( splitType == CU_TRIV_SPLIT )
  {
    sub.resize( 3, cuArea );

    for( int i = 0; i < 3; i++ )
    {
      for( auto &blk : sub[i].blocks )
      {
        blk.width >>= 1;

        if( ( i + 1 ) & 1 ) blk.width >>= 1;
        if( i == 1 )        blk.x      +=     blk.width / 2;
        if( i == 2 )        blk.x      += 3 * blk.width;
      }

      CHECK( sub[i].lumaSize().width < MIN_TB_SIZEY, "the cs split causes the block to be smaller than the minimal TU size" );
    }
  }
  else
  {
    THROW( "Unknown CU sub-partitioning" );
  }
}

void PartitionerImpl::getTUIntraSubPartitions( Partitioning &sub, const UnitArea& tuArea, const CodingStructure &cs, const PartSplit splitType, const TreeType treeType )
{
  uint32_t nPartitions;
  uint32_t splitDimensionSize = CU::getISPSplitDim( tuArea.lumaSize().width, tuArea.lumaSize().height, splitType );

  bool isDualTree = CS::isDualITree( cs ) || treeType != TREE_D;

  if( splitType == TU_1D_HORZ_SPLIT )
  {
    nPartitions = tuArea.lumaSize().height >> Log2(splitDimensionSize);

    sub.resize( nPartitions );

    for( uint32_t i = 0; i < nPartitions; i++ )
    {
      sub[i] = tuArea;
      CompArea& blkY = sub[i].blocks[COMP_Y];

      blkY.height = splitDimensionSize;
      blkY.y = i > 0 ? sub[i - 1].blocks[COMP_Y].y + splitDimensionSize : blkY.y;

      CHECK( sub[i].lumaSize().height < 1, "the cs split causes the block to be smaller than the minimal TU size" );
    }
  }
  else if( splitType == TU_1D_VERT_SPLIT )
  {
    nPartitions = tuArea.lumaSize().width >> Log2(splitDimensionSize);

    sub.resize( nPartitions );

    for( uint32_t i = 0; i < nPartitions; i++ )
    {
      sub[i] = tuArea;
      CompArea& blkY = sub[i].blocks[COMP_Y];

      blkY.width = splitDimensionSize;
      blkY.x = i > 0 ? sub[i - 1].blocks[COMP_Y].x + splitDimensionSize : blkY.x;
      CHECK( sub[i].lumaSize().width < 1, "the split causes the block to be smaller than the minimal TU size" );
    }
  }
  else
  {
    THROW( "Unknown TU sub-partitioning" );
  }
  //we only partition luma, so there is going to be only one chroma tu at the end (unless it is dual tree, in which case there won't be any chroma components)
  uint32_t partitionsWithoutChroma = (cs.area.chromaFormat == CHROMA_400) ? 0 : (isDualTree ? nPartitions : nPartitions - 1);
  for( uint32_t i = 0; i < partitionsWithoutChroma; i++ )
  {
    CompArea& blkCb = sub[i].blocks[COMP_Cb];
    CompArea& blkCr = sub[i].blocks[COMP_Cr];
    blkCb = CompArea();
    blkCr = CompArea();
  }
}

static const int g_maxRtGridSize = 3;

static const int g_zScanToX[1 << ( g_maxRtGridSize << 1 )] =
{
   0,  1,  0,  1,  2,  3,  2,  3,
   0,  1,  0,  1,  2,  3,  2,  3,
   4,  5,  4,  5,  6,  7,  6,  7,
   4,  5,  4,  5,  6,  7,  6,  7,
   0,  1,  0,  1,  2,  3,  2,  3,
   0,  1,  0,  1,  2,  3,  2,  3,
   4,  5,  4,  5,  6,  7,  6,  7,
   4,  5,  4,  5,  6,  7,  6,  7,
};
static const int g_zScanToY[1 << ( g_maxRtGridSize << 1 )] =
{
   0,  0,  1,  1,  0,  0,  1,  1,
   2,  2,  3,  3,  2,  2,  3,  3,
   0,  0,  1,  1,  0,  0,  1,  1,
   2,  2,  3,  3,  2,  2,  3,  3,
   4,  4,  5,  5,  4,  4,  5,  5,
   6,  6,  7,  7,  6,  6,  7,  7,
   4,  4,  5,  5,  4,  4,  5,  5,
   6,  6,  7,  7,  6,  6,  7,  7,
};
static const int g_rsScanToZ[1 << ( g_maxRtGridSize << 1 )] =
{
   0,  1,  4,  5, 16, 17, 20, 21,
   2,  3,  6,  7, 18, 19, 22, 23,
   8,  9, 12, 13, 24, 25, 28, 29,
  10, 11, 14, 15, 26, 27, 30, 31,
  32, 33, 36, 37, 48, 49, 52, 53,
  34, 35, 38, 39, 50, 51, 54, 55,
  40, 41, 44, 45, 56, 57, 60, 61,
  42, 43, 46, 47, 58, 59, 62, 63,
};

void PartitionerImpl::getMaxTuTiling( Partitioning& sub, const UnitArea& cuArea, const CodingStructure &cs )
{
  static_assert( MAX_LOG2_DIFF_CU_TR_SIZE <= g_maxRtGridSize, "Z-scan tables are only provided for MAX_LOG2_DIFF_CU_TR_SIZE for up to 3 (8x8 tiling)!" );

  const Size area     = cuArea.lumaSize();
  const int maxTrSize = (area.width>64 || area.height>64) ? 64 : cs.sps->getMaxTbSize();
  const int numTilesH = std::max<int>( 1, area.width  / maxTrSize );
  const int numTilesV = std::max<int>( 1, area.height / maxTrSize );
  const int numTiles  = numTilesH * numTilesV;

  CHECK( numTiles > MAX_CU_TILING_PARTITIONS, "CU partitioning requires more partitions than available" );

  Partitioning &ret = sub;
  ret.resize( numTiles, cuArea );

  for( int i = 0; i < numTiles; i++ )
  {
    const int rsy = i / numTilesH;
    const int rsx = i % numTilesH;

    const int x = g_zScanToX[g_rsScanToZ[( rsy << g_maxRtGridSize ) + rsx]];
    const int y = g_zScanToY[g_rsScanToZ[( rsy << g_maxRtGridSize ) + rsx]];

    UnitArea& tile = ret[i];

    for( CompArea& comp : tile.blocks )
    {
      if( !comp.valid() ) continue;

      comp.width  /= numTilesH;
      comp.height /= numTilesV;

      comp.x += comp.width  * x;
      comp.y += comp.height * y;
    }
  }
}

void PartitionerImpl::getSbtTuTiling( Partitioning& ret, const UnitArea& cuArea, const CodingStructure &cs, const PartSplit splitType )
{
  int numTiles = 2;
  int widthFactor, heightFactor, xOffsetFactor, yOffsetFactor; // y = (x * factor) >> 2;
  assert( splitType >= SBT_VER_HALF_POS0_SPLIT && splitType <= SBT_HOR_QUAD_POS1_SPLIT );

  ret.resize( numTiles, cuArea );
  for( int i = 0; i < numTiles; i++ )
  {
    if( splitType >= SBT_VER_QUAD_POS0_SPLIT )
    {
      if( splitType == SBT_HOR_QUAD_POS0_SPLIT || splitType == SBT_HOR_QUAD_POS1_SPLIT )
      {
        widthFactor = 4;
        xOffsetFactor = 0;
        heightFactor = ( ( i == 0 && splitType == SBT_HOR_QUAD_POS0_SPLIT ) || ( i == 1 && splitType == SBT_HOR_QUAD_POS1_SPLIT ) ) ? 1 : 3;
        yOffsetFactor = ( i == 0 ) ? 0 : ( splitType == SBT_HOR_QUAD_POS0_SPLIT ? 1 : 3 );
      }
      else
      {
        widthFactor = ( ( i == 0 && splitType == SBT_VER_QUAD_POS0_SPLIT ) || ( i == 1 && splitType == SBT_VER_QUAD_POS1_SPLIT ) ) ? 1 : 3;
        xOffsetFactor = ( i == 0 ) ? 0 : ( splitType == SBT_VER_QUAD_POS0_SPLIT ? 1 : 3 );
        heightFactor = 4;
        yOffsetFactor = 0;
      }
    }
    else
    {
      if( splitType == SBT_HOR_HALF_POS0_SPLIT || splitType == SBT_HOR_HALF_POS1_SPLIT )
      {
        widthFactor = 4;
        xOffsetFactor = 0;
        heightFactor = 2;
        yOffsetFactor = ( i == 0 ) ? 0 : 2;
      }
      else
      {
        widthFactor = 2;
        xOffsetFactor = ( i == 0 ) ? 0 : 2;
        heightFactor = 4;
        yOffsetFactor = 0;
      }
    }

    UnitArea& tile = ret[i];
    for( CompArea& comp : tile.blocks )
    {
      if( !comp.valid() ) continue;
      comp.x += ( comp.width  * xOffsetFactor ) >> 2;
      comp.y += ( comp.height * yOffsetFactor ) >> 2;
      comp.width = ( comp.width  * widthFactor ) >> 2;
      comp.height = ( comp.height * heightFactor ) >> 2;
    }
  }
}

} // namespace vvenc

//! \}


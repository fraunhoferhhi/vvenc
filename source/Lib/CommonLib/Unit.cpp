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


/** \file     Unit.cpp
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#include "Unit.h"
#include "Picture.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

 // ---------------------------------------------------------------------------
 // block method definitions
 // ---------------------------------------------------------------------------

void CompArea::xRecalcLumaToChroma()
{
  const uint32_t csx = getComponentScaleX(compID, chromaFormat);
  const uint32_t csy = getComponentScaleY(compID, chromaFormat);

  x      >>= csx;
  y      >>= csy;
  width  >>= csx;
  height >>= csy;
}

Position CompArea::chromaPos() const
{
  if (isLuma(compID))
  {
    uint32_t scaleX = getComponentScaleX(compID, chromaFormat);
    uint32_t scaleY = getComponentScaleY(compID, chromaFormat);

    return Position(x >> scaleX, y >> scaleY);
  }
  else
  {
    return *this;
  }
}

Size CompArea::lumaSize() const
{
  if( isChroma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Size( width << scaleX, height << scaleY );
  }
  else
  {
    return *this;
  }
}

Size CompArea::chromaSize() const
{
  if( isLuma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Size( width >> scaleX, height >> scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::lumaPos() const
{
  if( isChroma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Position( x << scaleX, y << scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::compPos( const ComponentID compID ) const
{
  return isLuma( compID ) ? lumaPos() : chromaPos();
}

Position CompArea::chanPos( const ChannelType chType ) const
{
  return isLuma( chType ) ? lumaPos() : chromaPos();
}

// ---------------------------------------------------------------------------
// unit method definitions
// ---------------------------------------------------------------------------

UnitArea::UnitArea(const ChromaFormat _chromaFormat) : chromaFormat(_chromaFormat) { }

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const Area& _area) : chromaFormat(_chromaFormat), blocks(getNumberValidComponents(_chromaFormat))
{
  const uint32_t numCh = getNumberValidComponents(chromaFormat);

  for (uint32_t i = 0; i < numCh; i++)
  {
    blocks[i] = CompArea(ComponentID(i), chromaFormat, _area, true);
  }
}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea&  blkY) : chromaFormat(_chromaFormat), blocks { blkY } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat,       CompArea&& blkY) : chromaFormat(_chromaFormat), blocks { std::forward<CompArea>(blkY) } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea&  blkY, const CompArea& blkCb, const CompArea& blkCr)  : chromaFormat(_chromaFormat), blocks { blkY, blkCb, blkCr } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat,       CompArea&& blkY,      CompArea&& blkCb,      CompArea&& blkCr) : chromaFormat(_chromaFormat), blocks { std::forward<CompArea>(blkY), std::forward<CompArea>(blkCb), std::forward<CompArea>(blkCr) } {}

bool UnitArea::contains(const UnitArea& other) const
{
  bool ret = true;
  bool any = false;

  for( const auto &blk : other.blocks )
  {
    if( blk.valid() && blocks[blk.compID].valid() )
    {
      ret &= blocks[blk.compID].contains( blk );
      any = true;
    }
  }

  return any && ret;
}

bool UnitArea::contains( const UnitArea& other, const ChannelType chType ) const
{
  bool ret = true;
  bool any = false;

  for( const auto &blk : other.blocks )
  {
    if( toChannelType( blk.compID ) == chType && blk.valid() && blocks[blk.compID].valid() )
    {
      ret &= blocks[blk.compID].contains( blk );
      any = true;
    }
  }

  return any && ret;
}

void UnitArea::repositionTo(const UnitArea& unitArea)
{
  for(uint32_t i = 0; i < blocks.size(); i++)
  {
    blocks[i].repositionTo(unitArea.blocks[i]);
  }
}

const UnitArea UnitArea::singleComp(const ComponentID compID) const
{
  UnitArea ret(chromaFormat);

  for (const auto &blk : blocks)
  {
    if (blk.compID == compID)
    {
      ret.blocks.push_back(blk);
    }
    else
    {
      ret.blocks.push_back(CompArea());
    }
  }

  return ret;
}

const UnitArea UnitArea::singleChan(const ChannelType chType) const
{
  UnitArea ret(chromaFormat);

  for (const auto &blk : blocks)
  {
    if (toChannelType(blk.compID) == chType)
    {
      ret.blocks.push_back(blk);
    }
    else
    {
      ret.blocks.push_back(CompArea());
    }
  }

  return ret;
}

// ---------------------------------------------------------------------------
// coding unit method definitions
// ---------------------------------------------------------------------------

CodingUnit::CodingUnit(const UnitArea& unit)                                : UnitArea(unit),                 cs(nullptr), slice(nullptr), chType( CH_L ), next(nullptr), firstTU(nullptr), lastTU(nullptr) { initData(); initPuData(); }
CodingUnit::CodingUnit(const ChromaFormat _chromaFormat, const Area& _area) : UnitArea(_chromaFormat, _area), cs(nullptr), slice(nullptr), chType( CH_L ), next(nullptr), firstTU(nullptr), lastTU(nullptr) { initData(); initPuData(); }

CodingUnit& CodingUnit::operator=( const CodingUnit& other )
{
  slice             = other.slice;
  predMode          = other.predMode;
  qtDepth           = other.qtDepth;
  depth             = other.depth;
  btDepth           = other.btDepth;
  mtDepth           = other.mtDepth;
  splitSeries       = other.splitSeries;
  skip              = other.skip;
  mmvdSkip          = other.mmvdSkip;
  affine            = other.affine;
  affineType        = other.affineType;
  colorTransform    = other.colorTransform;
  geo               = other.geo;
  geo               = other.geo;
  bdpcmM[CH_L]      = other.bdpcmM[CH_L];
  bdpcmM[CH_C]      = other.bdpcmM[CH_C];
  qp                = other.qp;
  chromaQpAdj       = other.chromaQpAdj;
  rootCbf           = other.rootCbf;
  sbtInfo           = other.sbtInfo;
  mtsFlag           = other.mtsFlag;
  lfnstIdx          = other.lfnstIdx;
  tileIdx           = other.tileIdx;
  imv               = other.imv;
  imvNumCand        = other.imvNumCand;
  BcwIdx            = other.BcwIdx;

  smvdMode          = other.smvdMode;
  ispMode           = other.ispMode;
  mipFlag           = other.mipFlag;

  treeType          = other.treeType;
  modeType          = other.modeType;
  modeTypeSeries    = other.modeTypeSeries;

  const IntraPredictionData& ipd = other;
  *this = ipd;

  const InterPredictionData& tpd = other;
  *this = tpd;
  return *this;
}

void CodingUnit::initData()
{
  predMode          = NUMBER_OF_PREDICTION_MODES;
  qtDepth           = 0;
  depth             = 0;
  btDepth           = 0;
  mtDepth           = 0;
  splitSeries       = 0;
  skip              = false;
  mmvdSkip          = false;
  affine            = false;
  affineType        = 0;
  colorTransform    = false;
  geo               = false;
  bdpcmM[CH_L]      = 0;
  bdpcmM[CH_C]      = 0;
  qp                = 0;
  chromaQpAdj       = 0;
  rootCbf           = true;
  sbtInfo           = 0;
  mtsFlag           = 0;
  lfnstIdx          = 0;
  tileIdx           = 0;
  imv               = 0;
  imvNumCand        = 0;
  BcwIdx            = BCW_DEFAULT;
  smvdMode          = 0;
  ispMode           = 0;
  mipFlag           = false;

  treeType          = TREE_D;
  modeType          = MODE_TYPE_ALL;
  modeTypeSeries    = 0;
  mcControl         = 0;
}




// ---------------------------------------------------------------------------
// prediction unit method definitions
// ---------------------------------------------------------------------------

void CodingUnit::initPuData()
{
  // intra data - need this default initialization for PCM
  intraDir[0]       = DC_IDX;
  intraDir[1]       = PLANAR_IDX;
  multiRefIdx       = 0;
  mipTransposedFlag = false;

  // inter data
  mergeFlag         = false;
  regularMergeFlag  = false;
  ciip              = false;
  mvRefine          = false;
  mmvdMergeFlag     = false;
  mergeIdx          = MAX_UCHAR;
  geoSplitDir       = MAX_UCHAR;
  geoMergeIdx0      = MAX_UCHAR;
  geoMergeIdx1      = MAX_UCHAR;
  bv                . setZero();

  mcControl         = 0;

  interDir          = MAX_UCHAR;
  mmvdMergeIdx      = MAX_UINT;
  mergeType         = MRG_TYPE_DEFAULT_N;

  if( mvdL0SubPu )
  {
    int maxDmvrMvds = std::max<int>( 1, lwidth() >> DMVR_SUBCU_SIZE_LOG2 ) * std::max<int>( 1, lheight() >> DMVR_SUBCU_SIZE_LOG2 );
    for (uint32_t i = 0; i < maxDmvrMvds; i++)
    {
      mvdL0SubPu[i].setZero();
    }
  }

  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    mvpIdx[i] = MAX_UCHAR;
    mvpNum[i] = MAX_UCHAR;
    refIdx[i] = -1;
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvd[i][j].setZero();
      mv [i][j].setZero();
    }
  }
}

CodingUnit& CodingUnit::operator=( const IntraPredictionData& other )
{
  for( uint32_t i = 0; i < MAX_NUM_CH; i++ )
  {
    intraDir[ i ] = other.intraDir[ i ];
  }
  mipTransposedFlag = other.mipTransposedFlag;
  multiRefIdx       = other.multiRefIdx;
  return *this;
}

CodingUnit& CodingUnit::operator=( const InterPredictionData& other )
{
  mergeFlag         = other.mergeFlag;
  regularMergeFlag  = other.regularMergeFlag;
  mergeIdx          = other.mergeIdx;
  geoSplitDir       = other.geoSplitDir;
  geoMergeIdx0      = other.geoMergeIdx0;
  geoMergeIdx1      = other.geoMergeIdx1;
  mmvdMergeFlag     = other.mmvdMergeFlag;
  mmvdMergeIdx      = other.mmvdMergeIdx;
  interDir          = other.interDir;
  mergeType         = other.mergeType;
  mvRefine          = other.mvRefine;
  bv                = other.bv;

  if( other.mergeFlag && mvdL0SubPu )
  {
    const int maxDmvrMvds = std::max<int>( 1, lwidth() >> DMVR_SUBCU_SIZE_LOG2 ) * std::max<int>( 1, lheight() >> DMVR_SUBCU_SIZE_LOG2 );

    memcpy( mvdL0SubPu, other.mvdL0SubPu, sizeof( Mv ) * maxDmvrMvds );
  }

  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    mvpIdx[i]   = other.mvpIdx[i];
    mvpNum[i]   = other.mvpNum[i];
    refIdx[i]   = other.refIdx[i];
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvd[i][j] = other.mvd[i][j];
      mv [i][j] = other.mv [i][j];
    }
  }
  ciip = other.ciip;
  return *this;
}

CodingUnit& CodingUnit::operator=( const MotionInfo& mi )
{
  interDir = mi.interDir;

  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    refIdx[i] = mi.refIdx[i];
    mv [i][0] = mi.mv[i];
  }

  return *this;
}

const MotionInfo& CodingUnit::getMotionInfo() const
{
  return cs->getMotionInfo( lumaPos() );
}

const MotionInfo& CodingUnit::getMotionInfo( const Position& pos ) const
{
  CHECKD( !Y().contains( pos ), "Trying to access motion info outsied of PU" );
  return cs->getMotionInfo( pos );
}

MotionBuf CodingUnit::getMotionBuf()
{
  return cs->getMotionBuf( *this );
}

CMotionBuf CodingUnit::getMotionBuf() const
{
  return cs->getMotionBuf( *this );
}


// ---------------------------------------------------------------------------
// transform unit method definitions
// ---------------------------------------------------------------------------

TransformUnit::TransformUnit(const UnitArea& unit) : UnitArea(unit), cu(nullptr), cs(nullptr), chType( CH_L ), next( nullptr )
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    m_coeffs[i] = nullptr;
  }

  initData();
}

TransformUnit::TransformUnit(const ChromaFormat _chromaFormat, const Area& _area) : UnitArea(_chromaFormat, _area), cu(nullptr), cs(nullptr), chType( CH_L ), next( nullptr )
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    m_coeffs[i] = nullptr;
  }

  initData();
}

void TransformUnit::initData()
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    cbf[i]      = 0;
    mtsIdx[i]   = MTS_DCT2_DCT2;
    lastPos[i]  = 0;
  }
  depth       = 0;
  noResidual  = false;
  jointCbCr   = 0;
  chromaAdj   = 0;
}

void TransformUnit::init(TCoeff** coeffs)
{
  uint32_t numBlocks = getNumberValidComponents( chromaFormat );

  for (uint32_t i = 0; i < numBlocks; i++)
  {
    m_coeffs[i] = coeffs[i];
  }
}

TransformUnit& TransformUnit::operator=(const TransformUnit& other)
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  unsigned numBlocks = getNumberValidTBlocks(*cs->pcv);
  for( unsigned i = 0; i < numBlocks; i++ )
  {
    CHECKD( blocks[i].area() != other.blocks[i].area(), "Transformation units cover different areas" );

    uint32_t area = blocks[i].area();

    if (m_coeffs[i] && other.m_coeffs[i] && m_coeffs[i] != other.m_coeffs[i])
    {
      memcpy(m_coeffs[i], other.m_coeffs[i], sizeof(TCoeff) * area);
    }
    cbf[i]      = other.cbf[i];
    mtsIdx[i]   = other.mtsIdx[i];
    lastPos[i]  = other.lastPos[i];
  }
  depth         = other.depth;
  noResidual    = other.noResidual;
  jointCbCr     = other.jointCbCr;
  return *this;
}

void TransformUnit::copyComponentFrom(const TransformUnit& other, const ComponentID i)
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  CHECKD( blocks[i].area() != other.blocks[i].area(), "Transformation units cover different areas" );

  uint32_t area = blocks[i].area();

  if (m_coeffs[i] && other.m_coeffs[i] && m_coeffs[i] != other.m_coeffs[i])
  {
    memcpy(m_coeffs[i], other.m_coeffs[i], sizeof(TCoeff) * area);
  }

  cbf[i]      = other.cbf[i];

  depth       = other.depth;
  mtsIdx[i]   = other.mtsIdx[i];
  noResidual  = other.noResidual;
  jointCbCr   = isChroma( i ) ? other.jointCbCr : jointCbCr;
  lastPos[i]  = other.lastPos[i];
}

void TransformUnit::checkTuNoResidual( unsigned idx )
{
  if( CU::getSbtIdx( cu->sbtInfo ) == SBT_OFF_DCT )
  {
    return;
  }

  if( ( CU::getSbtPos( cu->sbtInfo ) == SBT_POS0 && idx == 1 ) || ( CU::getSbtPos( cu->sbtInfo ) == SBT_POS1 && idx == 0 ) )
  {
    noResidual = true;
  }
}

int TransformUnit::getTbAreaAfterCoefZeroOut(ComponentID compID) const
{
  int tbArea = blocks[compID].width * blocks[compID].height;
  int tbZeroOutWidth = blocks[compID].width;
  int tbZeroOutHeight = blocks[compID].height;

  if (cs->sps->MTS && cu->sbtInfo != 0 && blocks[compID].width <= 32 && blocks[compID].height <= 32 && compID == COMP_Y)
  {
    tbZeroOutWidth = (blocks[compID].width == 32) ? 16 : tbZeroOutWidth;
    tbZeroOutHeight = (blocks[compID].height == 32) ? 16 : tbZeroOutHeight;
  }
  tbZeroOutWidth = std::min<int>(JVET_C0024_ZERO_OUT_TH, tbZeroOutWidth);
  tbZeroOutHeight = std::min<int>(JVET_C0024_ZERO_OUT_TH, tbZeroOutHeight);
  tbArea = tbZeroOutWidth * tbZeroOutHeight;
  return tbArea;
}

} // namespace vvenc

//! \}


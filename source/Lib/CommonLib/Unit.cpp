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

CodingUnit::CodingUnit(const UnitArea& unit)                                : UnitArea(unit),                 cs(nullptr), slice(nullptr), chType( CH_L ), next(nullptr), pu(nullptr), firstTU(nullptr), lastTU(nullptr) { initData(); initPuData(); pu = cu = this; }
CodingUnit::CodingUnit(const ChromaFormat _chromaFormat, const Area& _area) : UnitArea(_chromaFormat, _area), cs(nullptr), slice(nullptr), chType( CH_L ), next(nullptr), pu(nullptr), firstTU(nullptr), lastTU(nullptr) { initData(); initPuData(); pu = cu = this; }

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
  bdpcmMode         = other.bdpcmMode;
  bdpcmModeChroma   = other.bdpcmModeChroma;
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
  refIdxBi[0]       = other.refIdxBi[0];
  refIdxBi[1]       = other.refIdxBi[1];

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
  bdpcmMode         = 0;
  bdpcmModeChroma   = 0;
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
  refIdxBi[0]       = -1;
  refIdxBi[1]       = -1;
  smvdMode          = 0;
  ispMode           = 0;
  mipFlag           = false;

  treeType          = TREE_D;
  modeType          = MODE_TYPE_ALL;
  modeTypeSeries    = 0;
}


bool CodingUnit::isSepTree() const
{
  return treeType != TREE_D || CS::isDualITree( *cs );
}

bool CodingUnit::isLocalSepTree() const
{
  return treeType != TREE_D && !CS::isDualITree(*cs);
}

bool CodingUnit::checkCCLMAllowed() const
{
  bool allowCCLM = false;

  if( !CS::isDualITree( *cs ) ) //single tree I slice or non-I slice (Note: judging chType is no longer equivalent to checking dual-tree I slice since the local dual-tree is introduced)
  {
    allowCCLM = true;
  }
  else if( slice->sps->CTUSize <= 32 ) //dual tree, CTUsize < 64
  {
    allowCCLM = true;
  }
  else //dual tree, CTU size 64 or 128
  {
    int depthFor64x64Node = slice->sps->CTUSize == 128 ? 1 : 0;
    const PartSplit cuSplitTypeDepth1 = CU::getSplitAtDepth( *this, depthFor64x64Node );
    const PartSplit cuSplitTypeDepth2 = CU::getSplitAtDepth( *this, depthFor64x64Node + 1 );

    //allow CCLM if 64x64 chroma tree node uses QT split or HBT+VBT split combination
    if( cuSplitTypeDepth1 == CU_QUAD_SPLIT || (cuSplitTypeDepth1 == CU_HORZ_SPLIT && cuSplitTypeDepth2 == CU_VERT_SPLIT) )
    {
      if( chromaFormat == CHROMA_420 )
      {
        CHECK( !(blocks[COMP_Cb].width <= 16 && blocks[COMP_Cb].height <= 16), "chroma cu size shall be <= 16x16 for YUV420 format" );
      }
      allowCCLM = true;
    }
    //allow CCLM if 64x64 chroma tree node uses NS (No Split) and becomes a chroma CU containing 32x32 chroma blocks
    else if( cuSplitTypeDepth1 == CU_DONT_SPLIT )
    {
      if( chromaFormat == CHROMA_420 )
      {
        CHECK( !(blocks[COMP_Cb].width == 32 && blocks[COMP_Cb].height == 32), "chroma cu size shall be 32x32 for YUV420 format" );
      }
      allowCCLM = true;
    }
    //allow CCLM if 64x32 chroma tree node uses NS and becomes a chroma CU containing 32x16 chroma blocks
    else if( cuSplitTypeDepth1 == CU_HORZ_SPLIT && cuSplitTypeDepth2 == CU_DONT_SPLIT )
    {
      if( chromaFormat == CHROMA_420 )
      {
        CHECK( !(blocks[COMP_Cb].width == 32 && blocks[COMP_Cb].height == 16), "chroma cu size shall be 32x16 for YUV420 format" );
      }
      allowCCLM = true;
    }

    //further check luma conditions
    if( allowCCLM )
    {
      //disallow CCLM if luma 64x64 block uses BT or TT or NS with ISP
      const Position lumaRefPos( chromaPos().x << getComponentScaleX( COMP_Cb, chromaFormat ), chromaPos().y << getComponentScaleY( COMP_Cb, chromaFormat ) );
      const CodingUnit* colLumaCu = cs->refCS->getCU( lumaRefPos, CH_L, TREE_D );

      if( colLumaCu->lwidth() < 64 || colLumaCu->lheight() < 64 ) //further split at 64x64 luma node
      {
        const PartSplit cuSplitTypeDepth1Luma = CU::getSplitAtDepth( *colLumaCu, depthFor64x64Node );
        CHECK( !(cuSplitTypeDepth1Luma >= CU_QUAD_SPLIT && cuSplitTypeDepth1Luma <= CU_TRIV_SPLIT), "split mode shall be BT, TT or QT" );
        if( cuSplitTypeDepth1Luma != CU_QUAD_SPLIT )
        {
          allowCCLM = false;
        }
      }
      else if( colLumaCu->lwidth() == 64 && colLumaCu->lheight() == 64 && colLumaCu->ispMode ) //not split at 64x64 luma node and use ISP mode
      {
        allowCCLM = false;
      }
    }
  }

  return allowCCLM;
}

uint8_t CodingUnit::checkAllowedSbt() const
{
  if( !slice->sps->SBT || predMode != MODE_INTER || pu->ciip)
  {
    return 0;
  }

  const int cuWidth  = lwidth();
  const int cuHeight = lheight();

  //parameter
  const int maxSbtCUSize = cs->sps->getMaxTbSize();

  //check on size
  if( cuWidth > maxSbtCUSize || cuHeight > maxSbtCUSize )
  {
    return 0;
  }

  const int minSbtCUSize  = 1 << ( MIN_CU_LOG2 + 1 );
  const int minQuadCUSize = 1 << ( MIN_CU_LOG2 + 2 );

  uint8_t sbtAllowed = 0;
  if( cuWidth  >= minSbtCUSize )  sbtAllowed += 1 << SBT_VER_HALF;
  if( cuHeight >= minSbtCUSize )  sbtAllowed += 1 << SBT_HOR_HALF;
  if( cuWidth  >= minQuadCUSize ) sbtAllowed += 1 << SBT_VER_QUAD;
  if( cuHeight >= minQuadCUSize ) sbtAllowed += 1 << SBT_HOR_QUAD;

  return sbtAllowed;
}


// ---------------------------------------------------------------------------
// prediction unit method definitions
// ---------------------------------------------------------------------------

//PredictionUnit::PredictionUnit(const UnitArea& unit)                                : UnitArea(unit)                , cu(nullptr), cs(nullptr), chType( CH_L ) { initData(); }
//PredictionUnit::PredictionUnit(const ChromaFormat _chromaFormat, const Area& _area) : UnitArea(_chromaFormat, _area), cu(nullptr), cs(nullptr), chType( CH_L ) { initData(); }

void CodingUnit::initPuData()
{
  // intra data - need this default initialization for PCM
  intraDir[0]       = DC_IDX;
  intraDir[1]       = PLANAR_IDX;
  mipTransposedFlag = false;
  multiRefIdx       = 0;

  // inter data
  mergeFlag         = false;
  regularMergeFlag  = false;
  mergeIdx          = MAX_UCHAR;
  geoSplitDir       = MAX_UCHAR;
  geoMergeIdx0      = MAX_UCHAR;
  geoMergeIdx1      = MAX_UCHAR;
  mmvdMergeFlag     = false;
  mmvdMergeIdx      = MAX_UINT;
  interDir          = MAX_UCHAR;
  mergeType         = MRG_TYPE_DEFAULT_N;
  mvRefine          = false;

  for (uint32_t i = 0; i < MAX_NUM_SUBCU_DMVR; i++)
  {
    mvdL0SubPu[i].setZero();
  }

  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    mvpIdx[i] = MAX_UCHAR;
    mvpNum[i] = MAX_UCHAR;
    refIdx[i] = -1;
    mv[i]     .setZero();
    mvd[i]    .setZero();
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvdAffi[i][j].setZero();
    }
    for ( uint32_t j = 0; j < 3; j++ )
    {
      mvAffi[i][j].setZero();
    }
  }

  mcControl = 0;
  ciip      = false;
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

  if( other.mergeFlag )
  {
    for (uint32_t i = 0; i < MAX_NUM_SUBCU_DMVR; i++)
    {
      mvdL0SubPu[i] = other.mvdL0SubPu[i];
    }
  }

  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    mvpIdx[i]   = other.mvpIdx[i];
    mvpNum[i]   = other.mvpNum[i];
    mv[i]       = other.mv[i];
    mvd[i]      = other.mvd[i];
    refIdx[i]   = other.refIdx[i];
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvdAffi[i][j] = other.mvdAffi[i][j];
    }
    for ( uint32_t j = 0; j < 3; j++ )
    {
      mvAffi[i][j] = other.mvAffi[i][j];
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
    mv    [i] = mi.mv[i];
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

void TransformUnit::init(TCoeff** coeffs, Pel** pcmbuf, bool** runType)
{
  uint32_t numBlocks = getNumberValidTBlocks(*cs->pcv);

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


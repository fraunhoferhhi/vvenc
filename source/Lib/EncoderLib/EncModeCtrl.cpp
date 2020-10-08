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

#include <cmath>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

void CacheBlkInfoCtrl::create()
{
  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;
  const int maxSizeIdx  = MAX_CU_SIZE_IDX-2;

  for( int wIdx = 0; wIdx < maxSizeIdx; wIdx++ )
  {
    for( int hIdx = 0; hIdx < maxSizeIdx; hIdx++ )
    {
      for( unsigned y = 0; y < numPos; y++ )
      {
        for( unsigned x = 0; x < numPos; x++ )
        {
          if(( x + (1<<(wIdx)) <= ( MAX_CU_SIZE >> MIN_CU_LOG2 ) )
          && ( y + (1<<(hIdx)) <= ( MAX_CU_SIZE >> MIN_CU_LOG2 ) ) )
          {
            m_codedCUInfo[wIdx][hIdx][x][y] = new CodedCUInfo;
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
  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;
  const int maxSizeIdx  = MAX_CU_SIZE_IDX-2;

  for( int wIdx = 0; wIdx < maxSizeIdx; wIdx++ )
  {
    for( int hIdx = 0; hIdx < maxSizeIdx; hIdx++ )
    {
      for( unsigned x = 0; x < numPos; x++ )
      {
        for( unsigned y = 0; y < numPos; y++ )
        {
          if( m_codedCUInfo[wIdx][hIdx][x][y] )
          {
            delete m_codedCUInfo[wIdx][hIdx][x][y];
          }
        }
      }
    }
  }
}

void CacheBlkInfoCtrl::init( const Slice &slice )
{
  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;
  const int maxSizeIdx  = MAX_CU_SIZE_IDX-2;

  for( int wIdx = 0; wIdx < maxSizeIdx; wIdx++ )
  {
    for( int hIdx = 0; hIdx < maxSizeIdx; hIdx++ )
    {
      for( unsigned x = 0; x < numPos; x++ )
      {
        for( unsigned y = 0; y < numPos; y++ )
        {
          if( m_codedCUInfo[wIdx][hIdx][x][y] )
          {
            GCC_WARNING_DISABLE_class_memaccess
            memset( m_codedCUInfo[wIdx][hIdx][x][y], 0, sizeof( CodedCUInfo ) );
            GCC_WARNING_RESET
          }
        }
      }
    }
  }

  m_pcv = slice.pps->pcv;
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

void SaveLoadEncInfoSbt::init( const Slice &slice )
{
  m_pcv = slice.pps->pcv;
}

void SaveLoadEncInfoSbt::create()
{
}

void SaveLoadEncInfoSbt::destroy()
{
}

uint8_t SaveLoadEncInfoSbt::findBestSbt( const UnitArea& area, const uint32_t curPuSse )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew( area.Y(), *m_pcv, idx1, idx2, idx3, idx4 );
  SaveLoadStructSbt* pSbtSave = &m_saveLoadSbt[idx1][idx2][idx3][idx4];

  for( int i = 0; i < pSbtSave->numPuInfoStored; i++ )
  {
    if( curPuSse == pSbtSave->puSse[i] )
    {
      return pSbtSave->puSbt[i];
    }
  }

  return MAX_UCHAR;
}

bool SaveLoadEncInfoSbt::saveBestSbt( const UnitArea& area, const uint32_t curPuSse, const uint8_t curPuSbt )
{
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew( area.Y(), *m_pcv, idx1, idx2, idx3, idx4 );
  SaveLoadStructSbt* pSbtSave = &m_saveLoadSbt[idx1][idx2][idx3][idx4];

  if( pSbtSave->numPuInfoStored == SBT_NUM_SL )
  {
    return false;
  }

  pSbtSave->puSse[pSbtSave->numPuInfoStored] = curPuSse;
  pSbtSave->puSbt[pSbtSave->numPuInfoStored] = curPuSbt;
  pSbtSave->numPuInfoStored++;
  return true;
}

void SaveLoadEncInfoSbt::resetSaveloadSbt( int maxSbtSize )
{
  int numSizeIdx = Log2(maxSbtSize) - MIN_CU_LOG2 + 1;
  int numPosIdx = MAX_CU_SIZE >> MIN_CU_LOG2;
  for( int wIdx = 0; wIdx < numSizeIdx; wIdx++ )
  {
    for( int hIdx = 0; hIdx < numSizeIdx; hIdx++ )
    {
      for( int xIdx = 0; xIdx < numPosIdx; xIdx++ )
      {
        memset( m_saveLoadSbt[wIdx][hIdx][xIdx], 0, numPosIdx * sizeof( SaveLoadStructSbt ) );
      }
    }
  }
}

static bool isTheSameNbHood( const CodingUnit &cu, const CodingStructure& cs, const Partitioner &partitioner, int picW, int picH )
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

void BestEncInfoCache::create( const ChromaFormat chFmt )
{
  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;
  const int maxSizeIdx  = MAX_CU_SIZE_IDX-2;

  for( int wIdx = 0; wIdx < maxSizeIdx; wIdx++ )
  {
    for( int hIdx = 0; hIdx < maxSizeIdx; hIdx++ )
    {
      for( unsigned x = 0; x < numPos; x++ )
      {
        for( unsigned y = 0; y < numPos; y++ )
        {
          if(( x + (1<<(wIdx) ) <= ( MAX_CU_SIZE >> MIN_CU_LOG2 ) )
           &&( y + (1<<(hIdx) ) <= ( MAX_CU_SIZE >> MIN_CU_LOG2 ) ))
          {
            m_bestEncInfo[wIdx][hIdx][x][y] = new BestEncodingInfo;

            const UnitArea area( chFmt, Area( 0, 0, 1<<(wIdx+2), 1<<(hIdx+2) ) );

            new ( &m_bestEncInfo[wIdx][hIdx][x][y]->cu ) CodingUnit    ( area );
            new ( &m_bestEncInfo[wIdx][hIdx][x][y]->tu ) TransformUnit( area );

            m_bestEncInfo[wIdx][hIdx][x][y]->poc      = -1;
            m_bestEncInfo[wIdx][hIdx][x][y]->testMode = EncTestMode();
          }
          else
          {
            m_bestEncInfo[wIdx][hIdx][x][y] = nullptr;
          }
        }
      }
    }
  }
}

void BestEncInfoCache::destroy()
{
  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;
  const int maxSizeIdx  = MAX_CU_SIZE_IDX-2;

  for( int wIdx = 0; wIdx < maxSizeIdx; wIdx++ )
  {
    for( int hIdx = 0; hIdx < maxSizeIdx; hIdx++ )
    {
      for( unsigned x = 0; x < numPos; x++ )
      {
        for( unsigned y = 0; y < numPos; y++ )
        {
          if( m_bestEncInfo[wIdx][hIdx][x][y] )
          {
            delete m_bestEncInfo[wIdx][hIdx][x][y];
            m_bestEncInfo[wIdx][hIdx][x][y] = nullptr;
          }
        }
      }
    }
  }

  delete[] m_pCoeff;
  m_pCoeff = nullptr;

  delete[] m_pPcmBuf;
  m_pPcmBuf = nullptr;

  if (m_runType != nullptr)
  {
    delete[] m_runType;
    m_runType = nullptr;
  }

  m_pcv = nullptr;
}

void BestEncInfoCache::init( const Slice &slice )
{
  bool isInitialized = m_pcv;

  m_pcv = slice.pps->pcv;

  if( isInitialized ) return;

  const unsigned numPos = MAX_CU_SIZE >> MIN_CU_LOG2;
  const int maxSizeIdx  = MAX_CU_SIZE_IDX-2;

  size_t numCoeff = 0;
  for( int wIdx = 0; wIdx < maxSizeIdx; wIdx++ )
  {
    for( int hIdx = 0; hIdx < maxSizeIdx; hIdx++ )
    {
      for( unsigned x = 0; x < numPos; x++ )
      {
        for( unsigned y = 0; y < numPos; y++ )
        {
          if( m_bestEncInfo[wIdx][hIdx][x][y] )
          {
            for( const CompArea& blk : m_bestEncInfo[wIdx][hIdx][x][y]->cu.blocks )
            {
              numCoeff += blk.area();
            }
          }
        }
      }
    }
  }

  m_pCoeff  = new TCoeff[numCoeff];
  m_pPcmBuf = new Pel   [numCoeff];
  m_runType   = new bool[numCoeff];

  TCoeff *coeffPtr = m_pCoeff;
  Pel    *pcmPtr   = m_pPcmBuf;
  bool   *runTypePtr   = m_runType;

  m_dummyCS.pcv = m_pcv;

  for( int wIdx = 0; wIdx < maxSizeIdx; wIdx++ )
  {
    for( int hIdx = 0; hIdx < maxSizeIdx; hIdx++ )
    {
      for( unsigned x = 0; x < numPos; x++ )
      {
        for( unsigned y = 0; y < numPos; y++ )
        {
          if( m_bestEncInfo[wIdx][hIdx][x][y] )
          {
            TCoeff *coeff[MAX_NUM_TBLOCKS] = { 0, };
            Pel    *pcmbf[MAX_NUM_TBLOCKS] = { 0, };
            bool   *runType[MAX_NUM_TBLOCKS]   = { 0, };

            const UnitArea& area = m_bestEncInfo[wIdx][hIdx][x][y]->tu;

            for( int i = 0; i < area.blocks.size(); i++ )
            {
              coeff[i] = coeffPtr; coeffPtr += area.blocks[i].area();
              pcmbf[i] =   pcmPtr;   pcmPtr += area.blocks[i].area();
              runType[i] = runTypePtr;     runTypePtr += area.blocks[i].area();
            }

            m_bestEncInfo[wIdx][hIdx][x][y]->tu.cs = &m_dummyCS;
            m_bestEncInfo[wIdx][hIdx][x][y]->tu.init(coeff, pcmbf, runType);
          }
        }
      }
    }
  }
}

bool BestEncInfoCache::setFromCs( const CodingStructure& cs, const Partitioner& partitioner )
{
  if( cs.cus.size() != 1 || cs.tus.size() != 1 )
  {
    return false;
  }

  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew( cs.area.Y(), *m_pcv, idx1, idx2, idx3, idx4 );

  BestEncodingInfo& encInfo = *m_bestEncInfo[idx1][idx2][idx3][idx4];

  encInfo.poc            =  cs.picture->poc;
  encInfo.cu.repositionTo( *cs.cus.front() );
  encInfo.tu.repositionTo( *cs.tus.front() );
  encInfo.cu             = *cs.cus.front();
  for( auto &blk : cs.tus.front()->blocks )
  {
    if( blk.valid() ) encInfo.tu.copyComponentFrom( *cs.tus.front(), blk.compID );
  }
  encInfo.testMode       = getCSEncMode( cs );
  encInfo.dist           = cs.dist;
  encInfo.costEDO        = cs.costDbOffset;

  return true;
}

bool BestEncInfoCache::isReusingCuValid( const CodingStructure& cs, const Partitioner& partitioner, int qp )
{
  if( partitioner.treeType == TREE_C )
  {
    return false; //if save & load is allowed for chroma CUs, we should check whether luma info (pred, recon, etc) is the same, which is quite complex
  }
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew( cs.area.Y(), *m_pcv, idx1, idx2, idx3, idx4 );

  BestEncodingInfo& encInfo = *m_bestEncInfo[idx1][idx2][idx3][idx4];

  if( encInfo.cu.treeType != partitioner.treeType || encInfo.cu.modeType != partitioner.modeType )
  {
    return false;
  }
  if( encInfo.cu.qp != qp )
    return false;
  if( cs.picture->poc != encInfo.poc 
    || CS::getArea( cs, cs.area, partitioner.chType, partitioner.treeType ) != CS::getArea( cs, encInfo.cu, partitioner.chType, partitioner.treeType ) 
    || !isTheSameNbHood( encInfo.cu, cs, partitioner, (cs.picture->Y().width), (cs.picture->Y().height))
    || CU::isIBC(encInfo.cu)
    || partitioner.currQgEnable() || cs.currQP[partitioner.chType] != encInfo.cu.qp
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
  unsigned idx1, idx2, idx3, idx4;
  getAreaIdxNew( cs.area.Y(), *m_pcv, idx1, idx2, idx3, idx4 );

  BestEncodingInfo& encInfo = *m_bestEncInfo[idx1][idx2][idx3][idx4];

  if( cs.picture->poc != encInfo.poc 
    || CS::getArea( cs, cs.area, partitioner.chType, partitioner.treeType ) != CS::getArea( cs, encInfo.cu, partitioner.chType, partitioner.treeType ) 
    || !isTheSameNbHood( encInfo.cu, cs, partitioner, (cs.picture->Y().width), (cs.picture->Y().height))
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
void EncModeCtrl::init( const EncCfg& encCfg, RdCost* pRdCost )
{
  m_pcEncCfg  = &encCfg;
  m_pcRdCost  = pRdCost;
  comprCUCtx  = nullptr;

  CacheBlkInfoCtrl::create();
  BestEncInfoCache::create( encCfg.m_internChromaFormat );
  SaveLoadEncInfoSbt::create();
}

void EncModeCtrl::destroy()
{
  CacheBlkInfoCtrl::destroy();
  BestEncInfoCache::destroy();
  SaveLoadEncInfoSbt::destroy();
}

void EncModeCtrl::xExtractFeatures( const EncTestMode& encTestmode, CodingStructure& cs )
{
  CHECK( cs.features.size() < NUM_ENC_FEATURES, "Features vector is not initialized" );

  cs.features[ENC_FT_RD_COST        ] = double( cs.cost              );
  cs.features[ENC_FT_ENC_MODE_TYPE  ] = double( encTestmode.type     );
  cs.features[ENC_FT_ENC_MODE_OPTS  ] = double( encTestmode.opts     );
}

void EncModeCtrl::initCTUEncoding( const Slice &slice )
{
  CacheBlkInfoCtrl::init( slice );
  BestEncInfoCache::init( slice );
  SaveLoadEncInfoSbt::init( slice );

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

void EncModeCtrl::initCULevel( Partitioner &partitioner, const CodingStructure& cs )
{
  // Min/max depth
  unsigned minDepth = 0;
  unsigned maxDepth = cs.pcv->getMaxDepth( cs.slice->sliceType, partitioner.chType );
  if( m_pcEncCfg->m_useFastLCTU )
  {
    partitioner.setMaxMinDepth( minDepth, maxDepth, cs );
  }

  m_ComprCUCtxList.push_back( ComprCUCtx( cs, minDepth, maxDepth ) );
  comprCUCtx = &m_ComprCUCtxList.back();

  const CodingUnit* cuLeft  = cs.getCU( cs.area.blocks[partitioner.chType].pos().offset( -1, 0 ), partitioner.chType, partitioner.treeType );
  const CodingUnit* cuAbove = cs.getCU( cs.area.blocks[partitioner.chType].pos().offset( 0, -1 ), partitioner.chType, partitioner.treeType );

  const bool qtBeforeBt = ( (  cuLeft  &&  cuAbove  && cuLeft ->qtDepth > partitioner.currQtDepth && cuAbove->qtDepth > partitioner.currQtDepth )
                         || (  cuLeft  && !cuAbove  && cuLeft ->qtDepth > partitioner.currQtDepth )
                         || ( !cuLeft  &&  cuAbove  && cuAbove->qtDepth > partitioner.currQtDepth )
                         || ( !cuAbove && !cuLeft   && cs.area.lwidth() >= ( 32 << cs.slice->depth ) ) )
                         && ( cs.area.lwidth() > ( cs.pcv->getMinQtSize( *cs.slice, partitioner.chType ) << 1 ) );

  // set features
  ComprCUCtx &cuECtx    = *comprCUCtx;
  cuECtx.qtBeforeBt     = qtBeforeBt;
  cuECtx.doTriHorzSplit = true;
  cuECtx.doTriVertSplit = true;
  cuECtx.doMoreSplits   = 3;
  cuECtx.isReusingCu    = isReusingCuValid( cs, partitioner, cs.baseQP );
  cuECtx.didHorzSplit   = partitioner.canSplit( CU_HORZ_SPLIT, cs );
  cuECtx.didVertSplit   = partitioner.canSplit( CU_VERT_SPLIT, cs );
}

void EncModeCtrl::finishCULevel( Partitioner &partitioner )
{
  m_ComprCUCtxList.pop_back();
  comprCUCtx = m_ComprCUCtxList.size() ? &m_ComprCUCtxList.back() : nullptr;
}

bool EncModeCtrl::trySplit( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner, const EncTestMode& lastTestmode )
{
  ComprCUCtx& cuECtx = *comprCUCtx;

  const PartSplit implicitSplit = partitioner.getImplicitSplit( cs );
  const bool isBoundary         = implicitSplit != CU_DONT_SPLIT;

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

  //////////////////////////////////////////////////////////////////////////
  // skip-history rule - don't split further if at least for three past levels
  //                     in the split tree it was found that skip is the best mode
  //////////////////////////////////////////////////////////////////////////
  int skipScore = 0;

  if ((!slice.isIntra() || slice.sps->IBC) && cuECtx.isBestNoSplitSkip )
  {
    for( int i = 2; i < m_ComprCUCtxList.size(); i++ )
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
  if( !partitioner.canSplit( split, cs ) || skipScore >= 2 )
  {
    if( split == CU_HORZ_SPLIT ) cuECtx.didHorzSplit = false;
    if( split == CU_VERT_SPLIT ) cuECtx.didVertSplit = false;
    if( split == CU_QUAD_SPLIT ) cuECtx.didQuadSplit = false;
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
      const CPelBuf bufCurrArea = cs.getOrgBuf( partitioner.currArea().block( COMP_Y ) );

      double horVal = 0;
      double verVal = 0;
      double dupVal = 0;
      double dowVal = 0;

      const double th = m_pcEncCfg->m_IntraPeriod == 1 ? 1.2 : 1.0;

      unsigned j, k;

      for( j = 0; j < cuWidth - 1; j++ )
      {
        for( k = 0; k < cuHeight - 1; k++ )
        {
          horVal += abs( bufCurrArea.at( j + 1, k     ) - bufCurrArea.at( j, k ) );
          verVal += abs( bufCurrArea.at( j    , k + 1 ) - bufCurrArea.at( j, k ) );
          dowVal += abs( bufCurrArea.at( j + 1, k )     - bufCurrArea.at( j, k + 1 ) );
          dupVal += abs( bufCurrArea.at( j + 1, k + 1 ) - bufCurrArea.at( j, k ) );
        }
      }
      if( horVal > th * verVal && sqrt( 2 ) * horVal > th * dowVal && sqrt( 2 ) * horVal > th * dupVal && ( split == CU_HORZ_SPLIT || split == CU_TRIH_SPLIT ) )
      {
        return false;
      }
      if( th * dupVal < sqrt( 2 ) * verVal && th * dowVal < sqrt( 2 ) * verVal && th * horVal < verVal && ( split == CU_VERT_SPLIT || split == CU_TRIV_SPLIT ) )
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

  if( m_pcEncCfg->m_qtbttSpeedUp )
  {
    const int availDepth = cs.pcv->getMaxMTTDepth( slice, partitioner.chType ) - partitioner.currMtDepth;
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
          unsigned maxBTD        = cs.pcv->getMaxMTTDepth( slice, partitioner.chType );
          const CodingUnit *cuBR = bestCS->cus.back();
          unsigned height        = partitioner.currArea().lumaSize().height;

          if (bestCU && ((bestCU->btDepth == 0 && maxBTD >= ((slice.isIntra() && !slice.sps->IBC) ? 3 : 2))
            || (bestCU->btDepth == 1 && cuBR && cuBR->btDepth == 1 && maxBTD >= ((slice.isIntra() && !slice.sps->IBC) ? 4 : 3)))
            && (width <= MAX_TB_SIZEY && height <= MAX_TB_SIZEY)
            && cuECtx.didHorzSplit && cuECtx.didVertSplit )
          {
            return false;
          }
        }
        if( m_pcEncCfg->m_bUseEarlyCU && bestCS->cost != MAX_DOUBLE && bestCU && bestCU->skip )
        {
          return false;
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

      if( m_pcEncCfg->m_qtbttSpeedUp )
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
      break;
    case CU_TRIV_SPLIT:
      if( cuECtx.didVertSplit && bestCU && bestCU->btDepth == partitioner.currBtDepth && !bestCU->rootCbf )
      {
        return false;
      }

      if( m_pcEncCfg->m_qtbttSpeedUp )
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

  if( split == CU_QUAD_SPLIT )
  {
    cuECtx.didQuadSplit = !m_pcEncCfg->m_qtbttSpeedUp || !!cuECtx.doMoreSplits;
  }

  return !m_pcEncCfg->m_qtbttSpeedUp || !!cuECtx.doMoreSplits;
}

bool EncModeCtrl::tryMode( const EncTestMode& encTestmode, const CodingStructure &cs, Partitioner& partitioner )
{
  CHECK( isModeSplit( encTestmode ), "wrong method");

  ComprCUCtx& cuECtx = m_ComprCUCtxList.back();

  // if early skip detected, skip all modes checking but the splits
  if( cuECtx.earlySkip && m_pcEncCfg->m_useEarlySkipDetection  && !( isModeInter( encTestmode ) ) )
  {
    return false;
  }

  if( cuECtx.minDepth > partitioner.currQtDepth && partitioner.canSplit( CU_QUAD_SPLIT, cs ) )
  {
    // enforce QT
    return false;
  }

  const Slice&           slice        = *cs.slice;
  const uint32_t         numComp      = getNumberValidComponents( slice.sps->chromaFormatIdc );
  const CodedCUInfo      &relatedCU   = getBlkInfo( partitioner.currArea() );
  const Area             &lumaArea    = partitioner.currArea().Y();

  if( encTestmode.type == ETM_INTRA )
  {
    if( lumaArea.width > 64 || lumaArea.height > 64)
    {
      return false;
    }

    if (m_pcEncCfg->m_usePbIntraFast && (!cs.slice->isIntra() || cs.slice->sps->IBC) && cuECtx.interHad == 0 && cuECtx.bestCU && !CU::isIntra(*cuECtx.bestCU))
    {
      return false;
    }

    // INTRA MODES
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
    if( !( slice.isIRAP() || /*bestModeType == ETM_INTRA || */!cuECtx.bestTU ||
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
      if (!cs.slice->isIRAP() && m_pcEncCfg->m_usePbIntraFast && !cs.slice->disableSATDForRd)
      {
        const DFunc dfunc = DF_HAD;
        DistParam distParam = m_pcRdCost->setDistParam( cs.getOrgBuf( COMP_Y ), cuECtx.bestCS->getPredBuf( COMP_Y ), cs.sps->bitDepths[ CH_L ], dfunc );
        cuECtx.interHad = distParam.distFunc( distParam );
      }
    }
  }
  else if( isModeInter( encTestmode ) )
  {
    // INTER MODES (ME + MERGE/SKIP)
    CHECK( slice.isIntra(), "Inter-mode should not be in the I-Slice mode list!" );

    // --- Check if we can quit current mode using SAVE/LOAD coding history

    if( encTestmode.type == ETM_INTER_ME )
    {
      if( encTestmode.opts == ETO_STANDARD )
      {
        // NOTE: ETO_STANDARD is always done when early SKIP mode detection is enabled
        if( !m_pcEncCfg->m_useEarlySkipDetection )
        {
          if( relatedCU.isSkip || relatedCU.isIntra )
          {
            return false;
          }
        }
      }
    }
    if (encTestmode.type == ETM_AFFINE && relatedCU.isIntra)
    {
      return false;
    }
    if (encTestmode.type == ETM_MERGE_GEO
        && (partitioner.currArea().lwidth() < GEO_MIN_CU_SIZE || partitioner.currArea().lheight() < GEO_MIN_CU_SIZE
            || partitioner.currArea().lwidth() > GEO_MAX_CU_SIZE || partitioner.currArea().lheight() > GEO_MAX_CU_SIZE
            || partitioner.currArea().lwidth() >= 8 * partitioner.currArea().lheight()
            || partitioner.currArea().lheight() >= 8 * partitioner.currArea().lwidth()))
    {
      return false;
    }
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

  setFromCs( *cuECtx.bestCS, partitioner );

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
  else if( CU::isIntra( bestCU ) )
  {
    relatedCU.isIntra     = true;
  }
  cuECtx.isBestNoSplitSkip = bestCU.skip;
}


bool EncModeCtrl::useModeResult( const EncTestMode& encTestmode, CodingStructure*& tempCS, Partitioner& partitioner, const bool useEDO )
{
  xExtractFeatures( encTestmode, *tempCS );

  ComprCUCtx& cuECtx = m_ComprCUCtxList.back();


  if(      encTestmode.type == ETM_SPLIT_BT_H )
  {
    cuECtx.bestCostHorzSplit = tempCS->cost;
  }
  else if( encTestmode.type == ETM_SPLIT_BT_V )
  {
    cuECtx.bestCostVertSplit = tempCS->cost;
  }
  else if( encTestmode.type == ETM_SPLIT_TT_H )
  {
    cuECtx.bestCostTriHorzSplit = tempCS->cost;
  }
  else if( encTestmode.type == ETM_SPLIT_TT_V )
  {
    cuECtx.bestCostTriVertSplit = tempCS->cost;
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

  int maxMtD = tempCS->pcv->getMaxMTTDepth( *tempCS->slice, partitioner.chType ) + partitioner.currImplicitBtDepth;

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

  // for now just a simple decision based on RD-cost or choose tempCS if bestCS is not yet coded
  if( tempCS->features[ENC_FT_RD_COST] != MAX_DOUBLE && ( !cuECtx.bestCS || ( ( tempCS->features[ENC_FT_RD_COST] + ( useEDO ? tempCS->costDbOffset : 0 ) ) < ( cuECtx.bestCS->features[ENC_FT_RD_COST] + ( useEDO ? cuECtx.bestCS->costDbOffset : 0 ) ) ) ) )
  {
    cuECtx.bestCS = tempCS;
    cuECtx.bestCU = tempCS->cus[0];
    cuECtx.bestTU = cuECtx.bestCU->firstTU;

    if( isModeInter( encTestmode ) )
    {
      //Here we take the best cost of both inter modes. We are assuming only the inter modes (and all of them) have come before the intra modes!!!
      cuECtx.bestInterCost = cuECtx.bestCS->cost;
    }

    return true;
  }
  else
  {
    return false;
  }
}

} // namespace vvenc

//! \}


/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/** \file     EncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include "EncCu.h"
#include "EncLib.h"
#include "Analyze.h"
#include "EncPicture.h"
#include "EncModeCtrl.h"
#include "BitAllocation.h"
#include "EncStage.h"

#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/TimeProfiler.h"
#include "CommonLib/SearchSpaceCounter.h"

#include <mutex>
#include <cmath>
#include <algorithm>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

const MergeIdxPair EncCu::m_GeoModeTest[GEO_MAX_NUM_CANDS] = { MergeIdxPair{0, 1}, MergeIdxPair{1, 0}, MergeIdxPair{0, 2}, MergeIdxPair{1, 2}, MergeIdxPair{2, 0},
                                                               MergeIdxPair{2, 1}, MergeIdxPair{0, 3}, MergeIdxPair{1, 3}, MergeIdxPair{2, 3}, MergeIdxPair{3, 0},
                                                               MergeIdxPair{3, 1}, MergeIdxPair{3, 2}, MergeIdxPair{0, 4}, MergeIdxPair{1, 4}, MergeIdxPair{2, 4},
                                                               MergeIdxPair{3, 4}, MergeIdxPair{4, 0}, MergeIdxPair{4, 1}, MergeIdxPair{4, 2}, MergeIdxPair{4, 3},
                                                               MergeIdxPair{0, 5}, MergeIdxPair{1, 5}, MergeIdxPair{2, 5}, MergeIdxPair{3, 5}, MergeIdxPair{4, 5},
                                                               MergeIdxPair{5, 0}, MergeIdxPair{5, 1}, MergeIdxPair{5, 2}, MergeIdxPair{5, 3}, MergeIdxPair{5, 4} };


// Shape coefSquareCUs (2 x 5 x 2 x 2 x 2): preset (faster and fast + medium) x cusize x nspred x sptype x numcoef

const double EncCu::coefSquareCUs[2][5][2][2][2] = {
{{{{-1.00000000, -1.00000000, }, {-1.00000000, -1.00000000, }, },  {{-1.00000000, -1.00000000, }, {-1.00000000, -1.00000000, }, },  },
{{{-1.00000000, -1.00000000, }, {-1.00000000, -1.00000000, }, },  {{-1.00000000, -1.00000000, }, {-1.00000000, -1.00000000, }, },  },
{{{-1.00000000, -1.00000000, }, {0.07848505, 0.00225808, }, },  {{-1.00000000, -1.00000000, }, {0.07509575, 0.00204789, }, },  },
{{{-1.00000000, -1.00000000, }, {0.10833051, 0.00053144, }, },  {{-1.00000000, -1.00000000, }, {0.08304352, 0.00142876, }, },  },
{{{-1.00000000, -1.00000000, }, {-1.00000000, -1.00000000, }, },  {{-1.00000000, -1.00000000, }, {-1.00000000, -1.00000000, }, },  },
},
{{{{-1.00000000, -1.00000000, }, {-1.00000000, -1.00000000, }, },  {{-1.00000000, -1.00000000, }, {-1.00000000, -1.00000000, }, },  },
{{{-1.00000000, -1.00000000, }, {-1.00000000, -1.00000000, }, },  {{-1.00000000, -1.00000000, }, {-1.00000000, -1.00000000, }, },  },
{{{0.06852235, 0.00388054, }, {0.09236045, 0.00084528, }, },  {{0.06955832, 0.00289679, }, {0.09598522, 0.00096187, }, },  },
{{{0.07268085, 0.00302796, }, {0.09323753, 0.00050996, }, },  {{0.06123618, 0.00471601, }, {0.09253389, 0.00046826, }, },  },
{{{-1.00000000, -1.00000000, }, {-1.00000000, -1.00000000, }, },  {{-1.00000000, -1.00000000, }, {-1.00000000, -1.00000000, }, },  },
},
};


// ====================================================================================================================
EncCu::EncCu()
  : m_CtxCache          ( nullptr )
  , m_globalCtuQpVector ( nullptr )
  , m_wppMutex          ( nullptr )
  , m_CABACEstimator    ( nullptr )
{
}

void EncCu::initPic( Picture* pic )
{
  const ReshapeData& reshapeData = pic->reshapeData;
  m_cRdCost.setReshapeParams( reshapeData.getReshapeLumaLevelToWeightPLUT(), reshapeData.getChromaWeight() );
  m_cInterSearch.setSearchRange( pic->cs->slice, *m_pcEncCfg );

  m_wppMutex = (m_pcEncCfg->m_numThreads > 0 ) ? &pic->wppMutex : nullptr;
}

void EncCu::initSlice( const Slice* slice )
{
  m_cTrQuant.setLambdas( slice->getLambdas() );
  m_cRdCost.setLambda( slice->getLambdas()[0], slice->sps->bitDepths );
}

void EncCu::setCtuEncRsrc( CABACWriter* cabacEstimator, CtxCache* ctxCache, ReuseUniMv* pReuseUniMv, BlkUniMvInfoBuffer* pBlkUniMvInfoBuffer, AffineProfList* pAffineProfList, IbcBvCand* pCachedBvs )
{
  m_CABACEstimator = cabacEstimator;
  m_CtxCache       = ctxCache;
  m_cIntraSearch.setCtuEncRsrc( cabacEstimator, ctxCache );
  m_cInterSearch.setCtuEncRsrc( cabacEstimator, ctxCache, pReuseUniMv, pBlkUniMvInfoBuffer, pAffineProfList, pCachedBvs );
}

void EncCu::setUpLambda (Slice& slice, const double dLambda, const int iQP, const bool setSliceLambda, const bool saveUnadjusted)
{
  // store lambda
  m_cRdCost.setLambda( dLambda, slice.sps->bitDepths );

  // for RDO
  // in RdCost there is only one lambda because the luma and chroma bits are not separated, instead we weight the distortion of chroma.
  double dLambdas[MAX_NUM_COMP] = { dLambda };
  for( uint32_t compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++ )
  {
    const ComponentID compID = ComponentID( compIdx );
    int chromaQPOffset       = slice.pps->chromaQpOffset[compID] + slice.sliceChromaQpDelta[ compID ];
    int qpc = slice.sps->chromaQpMappingTable.getMappedChromaQpValue(compID, iQP) + chromaQPOffset;
    double tmpWeight         = pow( 2.0, ( iQP - qpc ) / 3.0 );  // takes into account of the chroma qp mapping and chroma qp Offset
    if( m_pcEncCfg->m_DepQuantEnabled/* && !( m_pcEncCfg->getLFNST() ) */)
    {
      tmpWeight *= ( m_pcEncCfg->m_GOPSize >= 8 ? pow( 2.0, 0.1/3.0 ) : pow( 2.0, 0.2/3.0 ) );  // increase chroma weight for dependent quantization (in order to reduce bit rate shift from chroma to luma)
    }
    m_cRdCost.setDistortionWeight( compID, tmpWeight );
    dLambdas[compIdx] = dLambda / tmpWeight;
  }

  // for RDOQ
  m_cTrQuant.setLambdas( dLambdas );

  // for SAO, ALF
  if (setSliceLambda)
  {
    slice.setLambdas( dLambdas );
  }
  if( saveUnadjusted )
  {
    m_cRdCost.saveUnadjustedLambda();
  }
}

void EncCu::updateLambda(const Slice& slice, const double ctuLambda, const int ctuQP, const int newQP, const bool saveUnadjusted)
{
  const double  corrFactor = pow (2.0, double (newQP - ctuQP) / 3.0);
  const double  newLambda  = ctuLambda * corrFactor;
  const double* oldLambdas = slice.getLambdas(); // assumes prior setUpLambda (slice, ctuLambda) call!
  const double  newLambdas[MAX_NUM_COMP] = { oldLambdas[COMP_Y] * corrFactor, oldLambdas[COMP_Cb] * corrFactor, oldLambdas[COMP_Cr] * corrFactor };

  m_cTrQuant.setLambdas ( newLambdas);
  m_cRdCost.setLambda   ( newLambda, slice.sps->bitDepths);

  if (saveUnadjusted)
  {
    m_cRdCost.saveUnadjustedLambda(); // TODO hlm: check if this actually improves the overall quality
  }
}

void EncCu::init( const VVEncCfg& encCfg, const SPS& sps, std::vector<int>* const globalCtuQpVector, Ctx* syncPicCtx, RateCtrl* pRateCtrl )
{
  DecCu::init( &m_cTrQuant, &m_cIntraSearch, &m_cInterSearch, encCfg.m_internChromaFormat );
  m_cRdCost.create     ();
  m_cRdCost.setCostMode( encCfg.m_costMode );
  if ( encCfg.m_lumaReshapeEnable || encCfg.m_lumaLevelToDeltaQPEnabled )
  {
    m_cRdCost.setReshapeInfo( encCfg.m_lumaReshapeEnable ? encCfg.m_reshapeSignalType : RESHAPE_SIGNAL_PQ, encCfg.m_internalBitDepth[ CH_L ], encCfg.m_internChromaFormat );
  }

  m_modeCtrl.init     ( encCfg, &m_cRdCost );
  m_cIntraSearch.init ( encCfg, &m_cTrQuant, &m_cRdCost, &m_SortedPelUnitBufs, m_unitCache );
  m_cInterSearch.init ( encCfg, &m_cTrQuant, &m_cRdCost, &m_modeCtrl, m_cIntraSearch.getSaveCSBuf() );
  m_cTrQuant.init     ( nullptr, encCfg.m_RDOQ, encCfg.m_useRDOQTS, false, true, encCfg.m_quantThresholdVal );

  m_syncPicCtx = syncPicCtx;                         ///< context storage for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row used for estimation
  m_pcRateCtrl = pRateCtrl;

  // Initialise scaling lists: The encoder will only use the SPS scaling lists. The PPS will never be marked present.
  const int maxLog2TrDynamicRange[ MAX_NUM_CH ] = { sps.getMaxLog2TrDynamicRange( CH_L ), sps.getMaxLog2TrDynamicRange( CH_C ) };
  m_cTrQuant.getQuant()->setFlatScalingList( maxLog2TrDynamicRange, sps.bitDepths );

  m_pcEncCfg       = &encCfg;

  m_GeoCostList.init( encCfg.m_maxNumGeoCand );

  unsigned      uiMaxSize    = encCfg.m_CTUSize;
  ChromaFormat  chromaFormat = encCfg.m_internChromaFormat;

  Area ctuArea = Area( 0, 0, uiMaxSize, uiMaxSize );

  m_mergeItemList.init( encCfg.m_maxMergeRdCandNumTotal, m_pcEncCfg->m_Geo > 1 ? 3 : 1, chromaFormat, uiMaxSize, uiMaxSize );

  for( int i = 0; i < maxCuDepth; i++ )
  {
    Area area = Area( 0, 0, uiMaxSize >> ( i >> 1 ), uiMaxSize >> ( ( i + 1 ) >> 1 ) );

    if( area.width < (1 << MIN_CU_LOG2) || area.height < (1 << MIN_CU_LOG2) )
    {
      m_pTempCS[i] = m_pBestCS[i] = nullptr;
      continue;
    }

    m_pTempCS[i] = new CodingStructure( m_unitCache, nullptr );
    m_pBestCS[i] = new CodingStructure( m_unitCache, nullptr );

    m_pTempCS[i]->createForSearch( chromaFormat, area );
    m_pBestCS[i]->createForSearch( chromaFormat, area );

    m_pOrgBuffer[i].create( chromaFormat, area );
    m_pRspBuffer[i].create( CHROMA_400, area );
  }

  m_pTempCS2 = new CodingStructure( m_unitCache, nullptr );
  m_pBestCS2 = new CodingStructure( m_unitCache, nullptr );

  m_pTempCS2->createForSearch( chromaFormat, ctuArea );
  m_pBestCS2->createForSearch( chromaFormat, ctuArea );

  m_cuChromaQpOffsetIdxPlus1 = 0;
  m_tempQpDiff = 0;
  m_globalCtuQpVector = globalCtuQpVector;

  m_SortedPelUnitBufs.create( chromaFormat, uiMaxSize, uiMaxSize );

  for( uint8_t i = 0; i < MAX_TMP_BUFS; i++)
  {
    m_aTmpStorageLCU[i].create(chromaFormat, Area(0, 0, uiMaxSize, uiMaxSize));
  }
  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    m_acMergeTmpBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxSize, uiMaxSize));
  }

  const unsigned maxDepth = 2 * MAX_CU_SIZE_IDX;
  m_CtxBuffer.resize( maxDepth );
  m_CurrCtx = 0;
  if( encCfg.m_EDO )
    m_dbBuffer.create( chromaFormat, Area( 0, 0, uiMaxSize, uiMaxSize ), 0, 8 );

  m_MergeSimpleFlag = 0;
  m_tileIdx = 0;
}


void EncCu::destroy()
{
  for( int i = 0; i < maxCuDepth; i++ )
  {
    if( m_pTempCS[i] )
    {
      m_pTempCS[i]->destroy();
      delete m_pTempCS[i]; m_pTempCS[i] = nullptr;
    }

    if( m_pBestCS[i] )
    {
      m_pBestCS[i]->destroy();
      delete m_pBestCS[i]; m_pBestCS[i] = nullptr;
    }

    m_pOrgBuffer[i].destroy();
    m_pRspBuffer[i].destroy();
  }

  m_pTempCS2->destroy();
  m_pBestCS2->destroy();

  delete m_pTempCS2; m_pTempCS2 = nullptr;
  delete m_pBestCS2; m_pBestCS2 = nullptr;

  m_SortedPelUnitBufs.destroy();

  for( uint8_t i = 0; i < MAX_TMP_BUFS; i++)
  {
    m_aTmpStorageLCU[i].destroy();
  }
  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    m_acMergeTmpBuffer[ui].destroy();
  }


  m_dbBuffer.destroy();
}


EncCu::~EncCu()
{
  destroy();
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncCu::encodeCtu( Picture* pic, int (&prevQP)[MAX_NUM_CH], uint32_t ctuXPosInCtus, uint32_t ctuYPosInCtus )
{
  CodingStructure&     cs          = *pic->cs;
  Slice*               slice       = cs.slice;
  const PreCalcValues& pcv         = *cs.pcv;

#if ENABLE_MEASURE_SEARCH_SPACE
  if( ctuXPosInCtus == 0 && ctuYPosInCtus == 0 )
  {
    g_searchSpaceAcc.picW = pic->lwidth();
    g_searchSpaceAcc.picH = pic->lheight();
    g_searchSpaceAcc.addSlice( slice->isIntra(), slice->depth );
  }

#endif
  const int ctuRsAddr                 = ctuYPosInCtus * pcv.widthInCtus + ctuXPosInCtus;

  const Position pos (ctuXPosInCtus * pcv.maxCUSize, ctuYPosInCtus * pcv.maxCUSize);
  const UnitArea ctuArea( cs.area.chromaFormat, Area( pos.x, pos.y, pcv.maxCUSize, pcv.maxCUSize ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

  const int tileXPosInCtus = cs.pps->tileColBd[cs.pps->ctuToTileCol[ctuXPosInCtus]];
  const int tileYPosInCtus = cs.pps->tileRowBd[cs.pps->ctuToTileRow[ctuYPosInCtus]];

  if( ( cs.slice->sliceType != VVENC_I_SLICE || cs.sps->IBC ) && ctuXPosInCtus == tileXPosInCtus )
  {
    const int tileRowId = cs.pps->getTileLineId( ctuXPosInCtus, ctuYPosInCtus );
    cs.motionLutBuf[tileRowId].lut.resize( 0 );
    cs.motionLutBuf[tileRowId].lutIbc.resize( 0 );
  }

  if( ( m_pcEncCfg->m_ensureWppBitEqual || m_pcEncCfg->m_entropyCodingSyncEnabled ) && ctuXPosInCtus == tileXPosInCtus )
  {
    m_CABACEstimator->initCtxModels( *slice );

    if( m_pcEncCfg->m_entropyCodingSyncEnabled && ( ctuYPosInCtus > tileYPosInCtus ) )
    {
      m_CABACEstimator->getCtx() = m_syncPicCtx[slice->pps->getTileLineId( ctuXPosInCtus, ctuYPosInCtus - 1 )];
    }

    prevQP[CH_L] = prevQP[CH_C] = slice->sliceQp; // hlm: call CU::predictQP() here!
  }
  else if( ctuXPosInCtus == tileXPosInCtus && ctuYPosInCtus == tileYPosInCtus )
  {
    m_CABACEstimator->initCtxModels( *slice );
    prevQP[CH_L] = prevQP[CH_C] = slice->sliceQp; // hlm: call CU::predictQP() here!
  }

  xCompressCtu( cs, ctuArea, ctuRsAddr, prevQP );

  m_CABACEstimator->resetBits();
  m_CABACEstimator->coding_tree_unit( cs, ctuArea, prevQP, ctuRsAddr, true, true );

  // Store probabilities of second CTU in line into buffer - used only if wavefront-parallel-processing is enabled.
  if( ctuXPosInCtus == tileXPosInCtus && m_pcEncCfg->m_entropyCodingSyncEnabled )
  {
    m_syncPicCtx[slice->pps->getTileLineId( ctuXPosInCtus, ctuYPosInCtus )] = m_CABACEstimator->getCtx();
  }

  DTRACE_AREA_CRC( g_trace_ctx, D_CRC, cs, ctuArea );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

void EncCu::xCompressCtu( CodingStructure& cs, const UnitArea& area, const unsigned ctuRsAddr, const int prevQP[] )
{
  m_tileIdx = cs.pps->getTileIdx( area.lumaPos() );

  m_modeCtrl.initCTUEncoding( *cs.slice, m_tileIdx );

  // init the partitioning manager
  Partitioner *partitioner = &m_partitioner;
  partitioner->initCtu( area, CH_L, *cs.slice );
  
  const Position& lumaPos = area.lumaPos();
  const bool leftSameTile  = lumaPos.x == 0 || m_tileIdx == cs.pps->getTileIdx( lumaPos.offset(-1, 0) );
  const bool aboveSameTile = lumaPos.y == 0 || m_tileIdx == cs.pps->getTileIdx( lumaPos.offset( 0,-1) );
  m_EDO = (!m_pcEncCfg->m_tileParallelCtuEnc || (leftSameTile && aboveSameTile)) ? m_pcEncCfg->m_EDO : 0;
  
  if( m_pcEncCfg->m_IBCMode )
  {
    m_cInterSearch.resetCtuRecordIBC();
  }

  // init current context pointer
  m_CurrCtx = m_CtxBuffer.data();

  PelStorage* orgBuffer = &m_pOrgBuffer[0];
  PelStorage* rspBuffer = &m_pRspBuffer[0];
  CodingStructure *tempCS =  m_pTempCS [0];
  CodingStructure *bestCS =  m_pBestCS [0];
  cs.initSubStructure( *tempCS, partitioner->chType, partitioner->currArea(), false, orgBuffer, rspBuffer );
  cs.initSubStructure( *bestCS, partitioner->chType, partitioner->currArea(), false, orgBuffer, rspBuffer );
  m_CABACEstimator->determineNeighborCus( *tempCS, partitioner->currArea(), partitioner->chType, partitioner->treeType );
  PROFILER_SCOPE_AND_STAGE_EXT( 1, _TPROF, P_COMPRESS_CU, tempCS, CH_L );

  // copy the relevant area
  UnitArea clippedArea = clipArea( partitioner->currArea(), cs.area );
  CPelUnitBuf org = cs.picture->getFilteredOrigBuffer().valid() ? cs.picture->getRspOrigBuf( clippedArea ) : cs.picture->getOrigBuf( clippedArea );
  tempCS->getOrgBuf( clippedArea ).copyFrom( org );
  const ReshapeData& reshapeData = cs.picture->reshapeData;
  if( cs.slice->lmcsEnabled && reshapeData.getCTUFlag() )
  {
    tempCS->getRspOrgBuf( clippedArea.Y() ).rspSignal( org.get( COMP_Y) , reshapeData.getFwdLUT() );
  }

  tempCS->currQP[CH_L] = bestCS->currQP[CH_L] =
  tempCS->baseQP       = bestCS->baseQP       = cs.slice->sliceQp;
  tempCS->prevQP[CH_L] = bestCS->prevQP[CH_L] = prevQP[CH_L];

  xCompressCU( tempCS, bestCS, *partitioner );
  // all signals were already copied during compression if the CTU was split - at this point only the structures are copied to the top level CS
  
  // Ensure that a coding was found
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );

  if ( m_wppMutex ) m_wppMutex->lock();

  cs.useSubStructure( *bestCS, partitioner->chType, TREE_D, CS::getArea( *bestCS, area, partitioner->chType, partitioner->treeType ) );

  if ( m_wppMutex ) m_wppMutex->unlock();

  if( CS::isDualITree( cs ) && isChromaEnabled( cs.pcv->chrFormat ) )
  {
    m_CABACEstimator->getCtx() = m_CurrCtx->start;

    partitioner->initCtu( area, CH_C, *cs.slice );

    cs.initSubStructure( *tempCS, partitioner->chType, partitioner->currArea(), false, orgBuffer, rspBuffer );
    cs.initSubStructure( *bestCS, partitioner->chType, partitioner->currArea(), false, orgBuffer, rspBuffer );
    m_CABACEstimator->determineNeighborCus( *tempCS, partitioner->currArea(), partitioner->chType, partitioner->treeType );
    tempCS->currQP[CH_C] = bestCS->currQP[CH_C] =
    tempCS->baseQP       = bestCS->baseQP       = cs.slice->sliceQp;
    tempCS->prevQP[CH_C] = bestCS->prevQP[CH_C] = prevQP[CH_C];

    xCompressCU( tempCS, bestCS, *partitioner );
    
    // Ensure that a coding was found
    // Selected mode's RD-cost must be not MAX_DOUBLE.
    CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
    CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
    CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );

    if ( m_wppMutex ) m_wppMutex->lock();

    cs.useSubStructure( *bestCS, partitioner->chType, TREE_D, CS::getArea( *bestCS, area, partitioner->chType, partitioner->treeType ) );

    if ( m_wppMutex ) m_wppMutex->unlock();
  }

  // reset context states and uninit context pointer
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CurrCtx                  = 0;
}



bool EncCu::xCheckBestMode( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, const bool useEDO )
{
  bool bestCSUpdated = false;

  if( !tempCS->cus.empty() )
  {
    if( tempCS->cus.size() == 1 )
    {
      const CodingUnit& cu = *tempCS->cus.front();
      CHECK( cu.skip && !cu.mergeFlag, "Skip flag without a merge flag is not allowed!" );
    }

    DTRACE_BEST_MODE( tempCS, bestCS, m_cRdCost.getLambda(true), useEDO );

    if( m_modeCtrl.useModeResult( encTestMode, tempCS, partitioner, useEDO ) )
    {
      std::swap( tempCS, bestCS );
      // store temp best CI for next CU coding
      m_CurrCtx->best = m_CABACEstimator->getCtx();
      bestCSUpdated = true;
    }
  }

  // reset context states
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  return bestCSUpdated;

}

void xCheckFastCuChromaSplitting( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner&  partitioner, ComprCUCtx& cuECtx )
{
  const uint32_t uiLPelX = tempCS->area.Cb().lumaPos().x;
  const uint32_t uiTPelY = tempCS->area.Cb().lumaPos().y;

  int lumaw = 0, lumah = 0;
  bool splitver      = true;
  bool splithor      = true;
  bool qtSplitChroma = true;

  if( partitioner.isSepTree( *tempCS ) && isChroma( partitioner.chType ) )
  {
    Position lumaRefPos( uiLPelX, uiTPelY );
    CodingUnit* colLumaCu = bestCS->lumaCS->getCU( lumaRefPos, CH_L, TREE_D );

    if( colLumaCu )
    {
      lumah = colLumaCu->Y().height;
      lumaw = colLumaCu->Y().width;
    }
  }
  else
  {
    return;
  }

  if( partitioner.getImplicitSplit( *tempCS ) != CU_DONT_SPLIT ) return;

  const CPelBuf orgCb = tempCS->getOrgBuf( COMP_Cb );
  const CPelBuf orgCr = tempCS->getOrgBuf( COMP_Cr );

  int th1 = FCBP_TH1;

  if( ( lumaw >> getChannelTypeScaleX( CH_C, tempCS->area.chromaFormat ) ) == orgCb.width )
  {
    if( ( bestCS->cost < ( th1*orgCb.width*orgCb.height ) ) )
    {
      splitver      = false;
      qtSplitChroma = false;
    }
  }

  if( ( lumah >> getChannelTypeScaleY( CH_C, tempCS->area.chromaFormat ) ) == orgCb.height )
  {
    if( ( bestCS->cost < ( th1*orgCb.width*orgCb.height ) ) )
    {
      splithor      = false;
      qtSplitChroma = false;
    }
  }

  cuECtx.doHorChromaSplit = splithor;
  cuECtx.doVerChromaSplit = splitver;
  cuECtx.doQtChromaSplit  = qtSplitChroma;

  if( orgCb.width == orgCb.height )
  {
    int varh_cb, varv_cb;
    int varh_cr, varv_cr;

    orgCb.calcVarianceSplit( orgCb, orgCb.width, varh_cb, varv_cb );
    orgCr.calcVarianceSplit( orgCr, orgCr.width, varh_cr, varv_cr );

    if( ( varh_cr*FCBP_TH2 < varv_cr * 100 ) && ( varh_cb*FCBP_TH2 < varv_cb * 100 ) )
    {
      cuECtx.doVerChromaSplit = false;
    }
    else if( ( varv_cr*FCBP_TH2 < varh_cr * 100 ) && ( varv_cb*FCBP_TH2 < varh_cb * 100 ) )
    {
      cuECtx.doHorChromaSplit = false;
    }
  }
}

void EncCu::xCompressCU( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& partitioner )
{
  const Area& lumaArea = tempCS->area.Y();

  Slice&   slice      = *tempCS->slice;
  const PPS &pps      = *tempCS->pps;
  const SPS &sps      = *tempCS->sps;
  const uint32_t uiLPelX  = tempCS->area.Y().lumaPos().x;
  const uint32_t uiTPelY  = tempCS->area.Y().lumaPos().y;
  const bool isBimEnabled = (m_pcEncCfg->m_blockImportanceMapping && !bestCS->picture->m_picShared->m_ctuBimQpOffset.empty());

  m_modeCtrl.initBlk( tempCS->area, slice.pic->poc );

  if ((m_pcEncCfg->m_usePerceptQPA || isBimEnabled) && pps.useDQP && isLuma (partitioner.chType) && partitioner.currQgEnable())
  {
    const PreCalcValues &pcv = *pps.pcv;
    Picture* const pic = bestCS->picture;
    const uint32_t ctuRsAddr = getCtuAddr (partitioner.currQgPos, pcv);

    if (partitioner.currSubdiv == 0) // CTU-level QP adaptation
    {
      if (m_pcEncCfg->m_usePerceptQPA)
      {
        if (m_pcEncCfg->m_internalUsePerceptQPATempFiltISlice == 2)
        {
          m_tempQpDiff = pic->ctuAdaptedQP[ctuRsAddr] - BitAllocation::applyQPAdaptationSubCtu (&slice, m_pcEncCfg, lumaArea, m_pcRateCtrl->getMinNoiseLevels());
        }

        if ((!slice.isIntra()) && (pcv.maxCUSize > 64) && // sub-CTU QPA behavior - Museum fix
            (uiLPelX + (pcv.maxCUSize >> 1) < (m_pcEncCfg->m_PadSourceWidth)) &&
            (uiTPelY + (pcv.maxCUSize >> 1) < (m_pcEncCfg->m_PadSourceHeight)))
        {
          const uint32_t h = lumaArea.height >> 1;
          const uint32_t w = lumaArea.width  >> 1;
          const int adQPTL = BitAllocation::applyQPAdaptationSubCtu (&slice, m_pcEncCfg, Area (uiLPelX + 0, uiTPelY + 0, w, h), m_pcRateCtrl->getMinNoiseLevels());
          const int adQPTR = BitAllocation::applyQPAdaptationSubCtu (&slice, m_pcEncCfg, Area (uiLPelX + w, uiTPelY + 0, w, h), m_pcRateCtrl->getMinNoiseLevels());
          const int adQPBL = BitAllocation::applyQPAdaptationSubCtu (&slice, m_pcEncCfg, Area (uiLPelX + 0, uiTPelY + h, w, h), m_pcRateCtrl->getMinNoiseLevels());
          const int adQPBR = BitAllocation::applyQPAdaptationSubCtu (&slice, m_pcEncCfg, Area (uiLPelX + w, uiTPelY + h, w, h), m_pcRateCtrl->getMinNoiseLevels());

          tempCS->currQP[partitioner.chType] = tempCS->baseQP =
          bestCS->currQP[partitioner.chType] = bestCS->baseQP = std::min (std::min (adQPTL, adQPTR), std::min (adQPBL, adQPBR));

          if (m_pcEncCfg->m_internalUsePerceptQPATempFiltISlice == 2)
          {
            if ((m_globalCtuQpVector->size() > ctuRsAddr) && (slice.TLayer == 0) && // last CTU row of non-Intra key-frame
                (m_pcEncCfg->m_IntraPeriod == 2 * m_pcEncCfg->m_GOPSize) && (ctuRsAddr >= pcv.widthInCtus) && (uiTPelY + pcv.maxCUSize > m_pcEncCfg->m_PadSourceHeight))
            {
              m_globalCtuQpVector->at (ctuRsAddr) = m_globalCtuQpVector->at (ctuRsAddr - pcv.widthInCtus); // copy the pumping reducing QP offset from the top CTU neighbor
              tempCS->currQP[partitioner.chType] = tempCS->baseQP =
              bestCS->currQP[partitioner.chType] = bestCS->baseQP = tempCS->baseQP - m_globalCtuQpVector->at (ctuRsAddr);
            }
            tempCS->currQP[partitioner.chType] = tempCS->baseQP =
            bestCS->currQP[partitioner.chType] = bestCS->baseQP = Clip3 (0, MAX_QP, tempCS->baseQP + m_tempQpDiff);
          }
        }
        else
        {
          tempCS->currQP[partitioner.chType] = tempCS->baseQP =
          bestCS->currQP[partitioner.chType] = bestCS->baseQP = pic->ctuAdaptedQP[ctuRsAddr];
        }

        setUpLambda (slice, pic->ctuQpaLambda[ctuRsAddr], pic->ctuAdaptedQP[ctuRsAddr], false, true);
      }
      else // isBimEnabled without QPA
      {
        const int baseQp         = tempCS->baseQP;
        const unsigned bimQpSize = (unsigned) bestCS->picture->m_picShared->m_ctuBimQpOffset.size();
        uint32_t ctuAddr         = ctuRsAddr;

        if (bimQpSize != pcv.sizeInCtus) // re-calculate correct address of BIM CTU QP offset
        {
          const unsigned bimCtuSize  = m_pcEncCfg->m_bimCtuSize;
          const unsigned bimCtuWidth = (pcv.lumaWidth + bimCtuSize - 1) / bimCtuSize;

          ctuAddr = getCtuAddrFromCtuSize (partitioner.currQgPos, Log2 (bimCtuSize), bimCtuWidth);
          CHECK (ctuAddr >= bimQpSize, "ctuAddr exceeds size of m_ctuBimQpOffset");
        }
        tempCS->currQP[partitioner.chType] = tempCS->baseQP =
        bestCS->currQP[partitioner.chType] = bestCS->baseQP = Clip3 (-sps.qpBDOffset[CH_L], MAX_QP, tempCS->baseQP + pic->m_picShared->m_ctuBimQpOffset[ctuAddr]);

        updateLambda (slice, slice.getLambdas()[0], baseQp, tempCS->baseQP, true);
      }
    }
    else if (m_pcEncCfg->m_usePerceptQPA && slice.isIntra()) // currSubdiv 2 - use sub-CTU QPA
    {
      CHECK ((partitioner.currArea().lwidth() >= pcv.maxCUSize) || (partitioner.currArea().lheight() >= pcv.maxCUSize), "sub-CTU delta-QP error");
      tempCS->currQP[partitioner.chType] = tempCS->baseQP = BitAllocation::applyQPAdaptationSubCtu (&slice, m_pcEncCfg, lumaArea, m_pcRateCtrl->getMinNoiseLevels());

      if (m_pcEncCfg->m_internalUsePerceptQPATempFiltISlice == 2)
      {
        tempCS->currQP[partitioner.chType] = tempCS->baseQP = Clip3 (0, MAX_QP, tempCS->baseQP + m_tempQpDiff);
      }

      updateLambda (slice, pic->ctuQpaLambda[ctuRsAddr], pic->ctuAdaptedQP[ctuRsAddr], tempCS->baseQP, true);
    }
  }

  if (partitioner.currQtDepth == 0)
  {
    m_MergeSimpleFlag = 0;
  }
  m_modeCtrl.initCULevel( partitioner, *tempCS, m_MergeSimpleFlag );
  m_sbtCostSave[0] = m_sbtCostSave[1] = MAX_DOUBLE;

  m_CurrCtx->start = m_CABACEstimator->getCtx();

  m_cuChromaQpOffsetIdxPlus1 = 0;

  if( slice.chromaQpAdjEnabled && partitioner.currQgChromaEnable() )
  {
    // TODO M0133 : double check encoder decisions with respect to chroma QG detection and actual encode
    int cuChromaQpOffsetSubdiv = slice.isIntra() ? slice.picHeader->cuChromaQpOffsetSubdivIntra : slice.picHeader->cuChromaQpOffsetSubdivInter;
    int lgMinCuSize = sps.log2MinCodingBlockSize +
      std::max<int>(0, floorLog2(sps.CTUSize) - sps.log2MinCodingBlockSize - int((cuChromaQpOffsetSubdiv + 1) / 2));
    m_cuChromaQpOffsetIdxPlus1 = ( ( uiLPelX >> lgMinCuSize ) + ( uiTPelY >> lgMinCuSize ) ) % ( pps.chromaQpOffsetListLen + 1 );
  }

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cux", uiLPelX ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuy", uiTPelY ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuw", tempCS->area.lwidth() ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuh", tempCS->area.lheight() ) );
  DTRACE( g_trace_ctx, D_COMMON, "@(%4d,%4d) [%2dx%2d]\n", tempCS->area.lx(), tempCS->area.ly(), tempCS->area.lwidth(), tempCS->area.lheight() );

  if( tempCS->slice->checkLDC )
  {
    m_bestBcwCost[0] = m_bestBcwCost[1] = std::numeric_limits<double>::max();
    m_bestBcwIdx[0] = m_bestBcwIdx[1] = -1;
  }

  m_cInterSearch.resetSavedAffineMotion();
  {
    const ComprCUCtx &cuECtx      = *m_modeCtrl.comprCUCtx;
    const CodingStructure& cs     = *tempCS;
    const PartSplit implicitSplit = partitioner.getImplicitSplit( cs );
    const bool isBoundary         = implicitSplit != CU_DONT_SPLIT;
    const bool lossless           = false;
    int qp                        = cs.baseQP;

#if ENABLE_MEASURE_SEARCH_SPACE
    if( !isBoundary )
    {
      g_searchSpaceAcc.addPartition( partitioner.currArea(), partitioner.isSepTree( *tempCS ) ? partitioner.chType : MAX_NUM_CH );
    }

#endif
    if( ! isBoundary )
    {
      if (pps.useDQP && partitioner.isSepTree (*tempCS) && isChroma (partitioner.chType))
      {
        const ChromaFormat chromaFm = tempCS->area.chromaFormat;
        const Position chromaCentral (tempCS->area.Cb().chromaPos().offset (tempCS->area.Cb().chromaSize().width >> 1, tempCS->area.Cb().chromaSize().height >> 1));
        const Position lumaRefPos (chromaCentral.x << getChannelTypeScaleX (CH_C, chromaFm), chromaCentral.y << getChannelTypeScaleY (CH_C, chromaFm));
        const CodingUnit* colLumaCu = bestCS->lumaCS->getCU (lumaRefPos, CH_L, TREE_D);
        // update qp
        qp = colLumaCu->qp;
      }

      m_cIntraSearch.reset();

      bool isReuseCU = m_modeCtrl.isReusingCuValid( cs, partitioner, qp );

      bool checkIbc = m_pcEncCfg->m_IBCMode && bestCS->picture->useIBC && (partitioner.chType == CH_L);
      if ((m_pcEncCfg->m_IBCFastMethod>3) && (cs.area.lwidth() * cs.area.lheight()) > (16 * 16))
      {
        checkIbc = false;
      }
      if( isReuseCU )
      {
        xReuseCachedResult( tempCS, bestCS, partitioner );
      }
      else
      {
        // add first pass modes
        if ( !slice.isIntra() && !slice.isIRAP() && !( cs.area.lwidth() == 4 && cs.area.lheight() == 4 ) && !partitioner.isConsIntra() )
        {
          // add inter modes
          EncTestMode encTestModeSkip = { ETM_MERGE_SKIP, ETO_STANDARD, qp, lossless };
          if (m_modeCtrl.tryMode(encTestModeSkip, cs, partitioner))
          {
            xCheckRDCostUnifiedMerge(tempCS, bestCS, partitioner, encTestModeSkip);

            CodingUnit* cu = bestCS->getCU(partitioner.chType, partitioner.treeType);
            if (cu)
              cu->mmvdSkip = cu->skip == false ? false : cu->mmvdSkip;
          }
          EncTestMode encTestMode = { ETM_INTER_ME, ETO_STANDARD, qp, lossless };
          if (m_modeCtrl.tryMode(encTestMode, cs, partitioner))
          {
            xCheckRDCostInter(tempCS, bestCS, partitioner, encTestMode);
          }

          if (m_pcEncCfg->m_AMVRspeed)
          {
            double bestIntPelCost = MAX_DOUBLE;

            EncTestMode encTestMode = {ETM_INTER_IMV, ETO_STANDARD, qp, lossless};
            if( m_modeCtrl.tryMode( encTestMode, cs, partitioner ) )
            {
              const bool skipAltHpelIF = ( int( ( encTestMode.opts & ETO_IMV ) >> ETO_IMV_SHIFT ) == 4 ) && ( bestIntPelCost > 1.25 * bestCS->cost );
              if (!skipAltHpelIF)
              {
                xCheckRDCostInterIMV(tempCS, bestCS, partitioner, encTestMode );
              }
            }
          }
        }

        if (checkIbc && !partitioner.isConsInter())
        {
          EncTestMode encTestModeIBCMerge = { ETM_IBC_MERGE, ETO_STANDARD, qp, lossless };
          if ((m_pcEncCfg->m_IBCFastMethod < 4) && (partitioner.chType == CH_L) && m_modeCtrl.tryMode(encTestModeIBCMerge, cs, partitioner))
          {
            xCheckRDCostIBCModeMerge2Nx2N(tempCS, bestCS, partitioner, encTestModeIBCMerge);
          }

          EncTestMode encTestModeIBC = { ETM_IBC, ETO_STANDARD, qp, lossless };
          if (m_modeCtrl.tryMode(encTestModeIBC, cs, partitioner))
          {
            xCheckRDCostIBCMode(tempCS, bestCS, partitioner, encTestModeIBC);
          }
        }
        if( m_EDO && bestCS->cost != MAX_DOUBLE )
        {
          xCalDebCost(*bestCS, partitioner);
        }

        // add intra modes
        EncTestMode encTestMode( {ETM_INTRA, ETO_STANDARD, qp, lossless} );
        if( !partitioner.isConsInter() && m_modeCtrl.tryMode( encTestMode, cs, partitioner ) )
        {
          xCheckRDCostIntra( tempCS, bestCS, partitioner, encTestMode );
        }
      } // reusing cu

      m_modeCtrl.beforeSplit( partitioner );

      if (cuECtx.bestCS && ((cuECtx.bestCostNoImv == (MAX_DOUBLE * .5) || cuECtx.isReusingCu) && !slice.isIntra()) )
      {
        m_cInterSearch.loadGlobalUniMvs( lumaArea, *pps.pcv );
      }

      if (!cs.slice->isIntra() && (partitioner.chType == CH_L) && ( m_pcEncCfg->m_qtbttSpeedUpMode & 2) && (partitioner.currQtDepth < 3) && bestCS->cus.size())
      {
        int flagDbefore = (bestCS->cus[0]->mergeFlag && !bestCS->cus[0]->mmvdMergeFlag && !bestCS->cus[0]->ispMode && !bestCS->cus[0]->geo) ? 1 : 0;
        if (partitioner.currQtDepth == 0)
        {
          m_MergeSimpleFlag = flagDbefore;
        }
        else
        {
          int markFlag = (partitioner.currQtDepth == 1) ? 1 : 3;
          m_MergeSimpleFlag = (flagDbefore << partitioner.currQtDepth) | (m_MergeSimpleFlag & markFlag);
        }
      }
    } //boundary

    if( ( m_pcEncCfg->m_IntraPeriod == 1 ) && ( partitioner.chType == CH_C ) )
    {
      xCheckFastCuChromaSplitting( tempCS, bestCS, partitioner, *m_modeCtrl.comprCUCtx );
    }
    //////////////////////////////////////////////////////////////////////////
    // split modes
    EncTestMode lastTestMode;

    if( cuECtx.qtBeforeBt )
    {
      EncTestMode encTestMode( { ETM_SPLIT_QT, ETO_STANDARD, qp, false } );
      if( m_modeCtrl.trySplit( encTestMode, cs, partitioner, lastTestMode ) )
      {
        lastTestMode = encTestMode;
        xCheckModeSplit( tempCS, bestCS, partitioner, encTestMode );
      }
    }

    if( partitioner.canSplit( CU_HORZ_SPLIT, cs ) )
    {
      // add split modes
      EncTestMode encTestMode( { ETM_SPLIT_BT_H, ETO_STANDARD, qp, false } );
      if( m_modeCtrl.trySplit( encTestMode, cs, partitioner, lastTestMode ) )
      {
        lastTestMode = encTestMode;
        xCheckModeSplit( tempCS, bestCS, partitioner, encTestMode );
      }
    }

    if( partitioner.canSplit( CU_VERT_SPLIT, cs ) )
    {
      // add split modes
      EncTestMode encTestMode( { ETM_SPLIT_BT_V, ETO_STANDARD, qp, false } );
      if( m_modeCtrl.trySplit( encTestMode, cs, partitioner, lastTestMode ) )
      {
        lastTestMode = encTestMode;
        xCheckModeSplit( tempCS, bestCS, partitioner, encTestMode );
      }
    }

    if( partitioner.canSplit( CU_TRIH_SPLIT, cs ) )
    {
      // add split modes
      EncTestMode encTestMode( { ETM_SPLIT_TT_H, ETO_STANDARD, qp, false } );
      if( m_modeCtrl.trySplit( encTestMode, cs, partitioner, lastTestMode ) )
      {
        lastTestMode = encTestMode;
        xCheckModeSplit( tempCS, bestCS, partitioner, encTestMode );
      }
    }

    if( partitioner.canSplit( CU_TRIV_SPLIT, cs ) )
    {
      // add split modes
      EncTestMode encTestMode( { ETM_SPLIT_TT_V, ETO_STANDARD, qp, false } );
      if( m_modeCtrl.trySplit( encTestMode, cs, partitioner, lastTestMode ) )
      {
        lastTestMode = encTestMode;
        xCheckModeSplit( tempCS, bestCS, partitioner, encTestMode );
      }
    }

    if( !cuECtx.qtBeforeBt )
    {
      EncTestMode encTestMode( { ETM_SPLIT_QT, ETO_STANDARD, qp, false } );
      if( m_modeCtrl.trySplit( encTestMode, cs, partitioner, lastTestMode ) )
      {
        lastTestMode = encTestMode;
        xCheckModeSplit( tempCS, bestCS, partitioner, encTestMode );
      }
    }
  }

  if( bestCS->cus.empty() )
  {
    m_modeCtrl.finishCULevel( partitioner );
    return;
  }

  //////////////////////////////////////////////////////////////////////////
  // Finishing CU
  // set context states
  m_CABACEstimator->getCtx() = m_CurrCtx->best;

  // QP from last processed CU for further processing
  //copy the qp of the last non-chroma CU
  int numCUInThisNode = (int)bestCS->cus.size();
  if( numCUInThisNode > 1 && bestCS->cus.back()->chType == CH_C && !CS::isDualITree( *bestCS ) )
  {
    CHECK( bestCS->cus[numCUInThisNode-2]->chType != CH_L, "wrong chType" );
    bestCS->prevQP[partitioner.chType] = bestCS->cus[numCUInThisNode-2]->qp;
  }
  else
  {
    bestCS->prevQP[partitioner.chType] = bestCS->cus.back()->qp;
  }
  if( ( !slice.isIntra() || slice.sps->IBC )
    && partitioner.chType == CH_L
    && bestCS->cus.size() == 1 && ( bestCS->cus.back()->predMode == MODE_INTER || bestCS->cus.back()->predMode == MODE_IBC )
    && bestCS->area.Y() == (*bestCS->cus.back()).Y() )
  {
    const CodingUnit& cu = *bestCS->cus.front();
    bool isIbcSmallBlk = CU::isIBC(cu) && (cu.lwidth() * cu.lheight() <= 16);
    if (!cu.affine && !cu.geo && !isIbcSmallBlk)
    {
      const MotionInfo &mi = cu.getMotionInfo();
      HPMVInfo hMi( mi, ( mi.interDir() == 3 ) ? cu.BcwIdx : BCW_DEFAULT, cu.imv == IMV_HPEL, CU::isIBC( cu ) );
      cu.cs->addMiToLut( CU::isIBC( cu ) ? cu.cs->motionLut.lutIbc : cu.cs->motionLut.lut, hMi );
    }
  }

  m_modeCtrl.finishCULevel( partitioner );
  if( m_cIntraSearch.getSaveCuCostInSCIPU() && bestCS->cus.size() == 1 )
  {
    m_cIntraSearch.saveCuAreaCostInSCIPU( Area( partitioner.currArea().lumaPos(), partitioner.currArea().lumaSize() ), bestCS->cost );
  }

  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );
}


void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  const ModeType modeTypeParent  = partitioner.modeType;
  const TreeType treeTypeParent  = partitioner.treeType;
  const ChannelType chTypeParent = partitioner.chType;

  int signalModeConsVal = CS::signalModeCons( *tempCS, partitioner.currArea(), getPartSplit(encTestMode), modeTypeParent);
  int numRoundRdo = signalModeConsVal == LDT_MODE_TYPE_SIGNAL ? 2 : 1;
  bool skipInterPass = false;
  for( int i = 0; i < numRoundRdo; i++ )
  {
    //change cons modes
    if( signalModeConsVal == LDT_MODE_TYPE_SIGNAL )
    {
      CHECK( numRoundRdo != 2, "numRoundRdo shall be 2 - [LDT_MODE_TYPE_SIGNAL]" );
      partitioner.modeType = (i == 0) ? MODE_TYPE_INTER : MODE_TYPE_INTRA;
    }
    else if( signalModeConsVal == LDT_MODE_TYPE_INFER )
    {
      CHECK( numRoundRdo != 1, "numRoundRdo shall be 1 - [LDT_MODE_TYPE_INFER]" );
      partitioner.modeType = MODE_TYPE_INTRA;
    }
    else if( signalModeConsVal == LDT_MODE_TYPE_INHERIT )
    {
      CHECK( numRoundRdo != 1, "numRoundRdo shall be 1 - [LDT_MODE_TYPE_INHERIT]" );
      partitioner.modeType = modeTypeParent;
    }

    //for lite intra encoding fast algorithm, set the status to save inter coding info
    if( modeTypeParent == MODE_TYPE_ALL && partitioner.modeType == MODE_TYPE_INTER )
    {
      m_cIntraSearch.setSaveCuCostInSCIPU( true );
      m_cIntraSearch.setNumCuInSCIPU( 0 );
    }
    else if( modeTypeParent == MODE_TYPE_ALL && partitioner.modeType != MODE_TYPE_INTER )
    {
      m_cIntraSearch.setSaveCuCostInSCIPU( false );
      if( partitioner.modeType == MODE_TYPE_ALL )
      {
        m_cIntraSearch.setNumCuInSCIPU( 0 );
      }
    }

    xCheckModeSplitInternal( tempCS, bestCS, partitioner, encTestMode, modeTypeParent, skipInterPass );
    //recover cons modes
    partitioner.modeType = modeTypeParent;
    partitioner.treeType = treeTypeParent;
    partitioner.chType = chTypeParent;
    if( modeTypeParent == MODE_TYPE_ALL )
    {
      m_cIntraSearch.setSaveCuCostInSCIPU( false );
      if( numRoundRdo == 2 && partitioner.modeType == MODE_TYPE_INTRA )
      {
        m_cIntraSearch.initCuAreaCostInSCIPU();
      }
    }
    if( skipInterPass )
    {
      break;
    }
  }
}

void EncCu::xCheckModeSplitInternal(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, const ModeType modeTypeParent, bool& skipInterPass )
{
  const int qp                     = encTestMode.qp;
  const int oldPrevQp              = tempCS->prevQP[partitioner.chType];
  const auto oldMotionLut          = tempCS->motionLut;
  const ReshapeData& reshapeData   = tempCS->picture->reshapeData;
                                   
  const PartSplit split            = getPartSplit( encTestMode );
  const ModeType  modeTypeChild    = partitioner.modeType;

  CHECK( !( split == CU_QUAD_SPLIT || split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT
         || split == CU_TRIH_SPLIT || split == CU_TRIV_SPLIT ), "invalid split type" );

  tempCS->initStructData( qp );

  m_CABACEstimator->getCtx()       = m_CurrCtx->start;

  const uint16_t split_ctx_size    = Ctx::SplitFlag.size() + Ctx::SplitQtFlag.size() + Ctx::SplitHvFlag.size() + Ctx::Split12Flag.size() + Ctx::ModeConsFlag.size();
  const TempCtx  ctxSplitFlags     ( m_CtxCache, SubCtx( CtxSet( Ctx::SplitFlag(), split_ctx_size ), m_CABACEstimator->getCtx() ) );

  m_CABACEstimator->resetBits();
  m_CABACEstimator->split_cu_mode  ( split, *tempCS, partitioner );
  partitioner     . modeType       = modeTypeParent;
  m_CABACEstimator->mode_constraint( split, *tempCS, partitioner, modeTypeChild );
  partitioner     . modeType       = modeTypeChild;

  const int64_t splitBits   = m_CABACEstimator->getEstFracBits();

  const bool chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && modeTypeChild == MODE_TYPE_INTRA;
  const bool isChromaTooBig = isChromaEnabled( tempCS->pps->pcv->chrFormat ) && tempCS->area.Y().maxDim() > tempCS->sps->getMaxTbSize();
  bool       skipSplitTest  = chromaNotSplit && isChromaTooBig;

  if( !skipSplitTest )
  {
    double         a = -1, b = -1;
    const unsigned w       = partitioner.currArea().lwidth();
    const unsigned h       = partitioner.currArea().lheight();
    const bool contextCond = w == h && tempCS->slice->sliceType == VVENC_B_SLICE && isLuma( partitioner.chType ) && m_pcEncCfg->m_splitCostThrParamId >= 0 && m_pcEncCfg->m_splitCostThrParamId <= 1;

    if( contextCond )
    {
      uint8_t nsPredInd = m_modeCtrl.comprCUCtx->bestNsPredMode.type == ETM_INTRA;
      uint8_t szInd     = getLog2( w ) - 3;
      uint8_t splitInd  = split == CU_QUAD_SPLIT ? 1 : 0;
      a = coefSquareCUs[m_pcEncCfg->m_splitCostThrParamId][szInd][nsPredInd][splitInd][0];
      b = coefSquareCUs[m_pcEncCfg->m_splitCostThrParamId][szInd][nsPredInd][splitInd][1];
    }

    if( a > -1 && b > -1 )
    {
      const double bestNsCost    = m_modeCtrl.comprCUCtx->bestCostBeforeSplit == MAX_DOUBLE ? -1 : m_modeCtrl.comprCUCtx->bestCostBeforeSplit;
      const double factor        = 1.0 + b * exp( a * qp );
      const double predSplitCost = bestNsCost / factor + splitBits;
      skipSplitTest              = bestNsCost >= 0 && predSplitCost >= bestNsCost;
    }
    else
    {
      int numChild = 3;
      if( split == CU_VERT_SPLIT || split == CU_HORZ_SPLIT ) numChild--;
      else if( split == CU_QUAD_SPLIT ) numChild++;

      int64_t approxBits = m_pcEncCfg->m_qtbttSpeedUp > 0 ? numChild << SCALE_BITS : 0;

      const double factor     = ( tempCS->currQP[partitioner.chType] > 30                              ? 1.1  : 1.075 ) +
                                (   m_pcEncCfg->m_qtbttSpeedUp > 0                                     ? 0.01 : 0.0   ) +
                                ( ( m_pcEncCfg->m_qtbttSpeedUp > 0 && isChroma( partitioner.chType ) ) ? 0.2  : 0.0   );
       
      const double baseCost   = bestCS->cost + bestCS->costDbOffset;
      const double predCost   = baseCost / factor + splitBits + approxBits;
      skipSplitTest           = predCost >= baseCost;
    }
  }

  if( skipSplitTest )
  {
    m_CABACEstimator->getCtx() = SubCtx( CtxSet( Ctx::SplitFlag(), split_ctx_size ), ctxSplitFlags );
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
    return;
  }

  if( partitioner.treeType == TREE_D )
  {
    if( chromaNotSplit )
    {
      CHECK( partitioner.chType != CH_L, "chType must be luma" );
      partitioner.treeType = TREE_L;
    }
    else
    {
      partitioner.treeType = TREE_D;
    }
  }

  partitioner.splitCurrArea( split, *tempCS );
  bool qgEnableChildren = partitioner.currQgEnable(); // QG possible at children level

  m_CurrCtx++;

  AffineMVInfo tmpMVInfo;
  bool isAffMVInfoSaved = m_cInterSearch.m_AffineProfList->savePrevAffMVInfo( 0, tmpMVInfo );

  BlkUniMvInfo tmpUniMvInfo;
  bool         isUniMvInfoSaved = false;
  if( !tempCS->slice->isIntra() )
  {
    m_cInterSearch.m_BlkUniMvInfoBuffer->savePrevUniMvInfo( tempCS->area.Y(), tmpUniMvInfo, isUniMvInfoSaved );
  }

  DeriveCtx deriveCtx = m_CABACEstimator->getDeriveCtx();

  do
  {
    const auto &subCUArea  = partitioner.currArea();

    if( tempCS->picture->Y().contains( subCUArea.lumaPos() ) )
    {
      PelStorage* orgBuffer =  &m_pOrgBuffer[partitioner.currDepth];
      PelStorage* rspBuffer =  &m_pRspBuffer[partitioner.currDepth];
      CodingStructure *tempSubCS = m_pTempCS[partitioner.currDepth];
      CodingStructure *bestSubCS = m_pBestCS[partitioner.currDepth];

      tempCS->initSubStructure( *tempSubCS, partitioner.chType, subCUArea, false, orgBuffer, rspBuffer );
      tempCS->initSubStructure( *bestSubCS, partitioner.chType, subCUArea, false, orgBuffer, rspBuffer );

      // copy org buffer, need to be done after initSubStructure because of reshaping!
      orgBuffer->copyFrom( tempCS->getOrgBuf( subCUArea ) );
      if( tempCS->slice->lmcsEnabled && reshapeData.getCTUFlag() )
      {
        rspBuffer->Y().copyFrom( tempCS->getRspOrgBuf( subCUArea.Y() ) );
      }
      m_CABACEstimator->determineNeighborCus( *tempSubCS, partitioner.currArea(), partitioner.chType, partitioner.treeType );

      tempSubCS->bestParent = bestSubCS->bestParent = bestCS;

      xCompressCU(tempSubCS, bestSubCS, partitioner );

      tempSubCS->bestParent = bestSubCS->bestParent = nullptr;

      if( bestSubCS->cost == MAX_DOUBLE )
      {
        CHECK( split == CU_QUAD_SPLIT, "Split decision reusing cannot skip quad split" );
        tempCS->cost = MAX_DOUBLE;
        tempCS->costDbOffset = 0;
        m_CurrCtx--;
        partitioner.exitCurrSplit();
        xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
        if( partitioner.chType == CH_L )
        {
          tempCS->motionLut = oldMotionLut;
        }

        m_CABACEstimator->getDeriveCtx() = deriveCtx;
        return;
      }

      tempCS->useSubStructure( *bestSubCS, partitioner.chType, TREE_D, CS::getArea( *tempCS, subCUArea, partitioner.chType, partitioner.treeType ), partitioner.hasNextPart() || chromaNotSplit );

      if( partitioner.currQgEnable() )
      {
        tempCS->prevQP[partitioner.chType] = bestSubCS->prevQP[partitioner.chType];
      }
      if( partitioner.isConsInter() )
      {
        for( int i = 0; i < bestSubCS->cus.size(); i++ )
        {
          CHECK( bestSubCS->cus[i]->predMode != MODE_INTER, "all CUs must be inter mode in an Inter coding region (SCIPU)" );
        }
      }
      else if( partitioner.isConsIntra() )
      {
        for( int i = 0; i < bestSubCS->cus.size(); i++ )
        {
          CHECK( bestSubCS->cus[i]->predMode == MODE_INTER, "all CUs must not be inter mode in an Intra coding region (SCIPU)" );
        }
      }

      tempSubCS->releaseIntermediateData();
      bestSubCS->releaseIntermediateData();
      if( !tempCS->slice->isIntra() && partitioner.isConsIntra() )
      {
        tempCS->cost = m_cRdCost.calcRdCost( tempCS->fracBits, tempCS->dist );
        if( tempCS->cost > bestCS->cost )
        {
          tempCS->cost = MAX_DOUBLE;
          tempCS->costDbOffset = 0;
          m_CurrCtx--;
          partitioner.exitCurrSplit();
          if( partitioner.chType == CH_L )
          {
            tempCS->motionLut = oldMotionLut;
          }

          m_CABACEstimator->getDeriveCtx() = deriveCtx;
          return;
        }
      }
    }
  } while( partitioner.nextPart( *tempCS ) );

  partitioner.exitCurrSplit();

  m_CurrCtx--;

  m_CABACEstimator->getDeriveCtx() = deriveCtx;

  if( chromaNotSplit )
  {
    //Note: In local dual tree region, the chroma CU refers to the central luma CU's QP.
    //If the luma CU QP shall be predQP (no residual in it and before it in the QG), it must be revised to predQP before encoding the chroma CU
    //Otherwise, the chroma CU uses predQP+deltaQP in encoding but is decoded as using predQP, thus causing encoder-decoded mismatch on chroma qp.
    if( tempCS->pps->useDQP )
    {
      //find parent CS that including all coded CUs in the QG before this node
      CodingStructure* qgCS = tempCS;
      bool deltaQpCodedBeforeThisNode = false;
      if( partitioner.currArea().lumaPos() != partitioner.currQgPos )
      {
        int numParentNodeToQgCS = 0;
        while( qgCS->area.lumaPos() != partitioner.currQgPos )
        {
          CHECK( qgCS->parent == nullptr, "parent of qgCS shall exsit" );
          qgCS = qgCS->parent;
          numParentNodeToQgCS++;
        }

        //check whether deltaQP has been coded (in luma CU or luma&chroma CU) before this node
        CodingStructure* parentCS = tempCS->parent;
        for( int i = 0; i < numParentNodeToQgCS; i++ )
        {
          //checking each parent
          CHECK( parentCS == nullptr, "parentCS shall exsit" );
          for( const auto &cu : parentCS->cus )
          {
            if( cu->rootCbf && !isChroma( cu->chType ) )
            {
              deltaQpCodedBeforeThisNode = true;
              break;
            }
          }
          parentCS = parentCS->parent;
        }
      }

      //revise luma CU qp before the first luma CU with residual in the SCIPU to predQP
      if( !deltaQpCodedBeforeThisNode )
      {
        //get pred QP of the QG
        const CodingUnit* cuFirst = qgCS->getCU( CH_L, TREE_D );
        CHECK( cuFirst->lumaPos() != partitioner.currQgPos, "First cu of the Qg is wrong" );
        int predQp = CU::predictQP( *cuFirst, qgCS->prevQP[CH_L] );

        //revise to predQP
        int firstCuHasResidual = (int)tempCS->cus.size();
        for( int i = 0; i < tempCS->cus.size(); i++ )
        {
          if( tempCS->cus[i]->rootCbf )
          {
            firstCuHasResidual = i;
            break;
          }
        }

        for( int i = 0; i < firstCuHasResidual; i++ )
        {
          tempCS->cus[i]->qp = predQp;
        }
      }
    }
    partitioner.chType   = CH_C;
    partitioner.treeType = TREE_C;

    m_CurrCtx++;

    CodingStructure *tempCSChroma = m_pTempCS2;
    CodingStructure *bestCSChroma = m_pBestCS2;

    tempCS->initSubStructure( *tempCSChroma, partitioner.chType, partitioner.currArea(), false );
    tempCS->initSubStructure( *bestCSChroma, partitioner.chType, partitioner.currArea(), false );
    m_CABACEstimator->determineNeighborCus( *bestCSChroma, partitioner.currArea(), partitioner.chType, partitioner.treeType );
    tempCSChroma->lumaCS = tempCS;
    bestCSChroma->lumaCS = tempCS;
    xCompressCU( tempCSChroma, bestCSChroma, partitioner );

    //attach chromaCS to luma CS and update cost
    tempCS->useSubStructure( *bestCSChroma, partitioner.chType, TREE_D, CS::getArea( *bestCSChroma, partitioner.currArea(), partitioner.chType, partitioner.treeType ), false );

    //release tmp resource
    tempCSChroma->releaseIntermediateData();
    bestCSChroma->releaseIntermediateData();

    m_CurrCtx--;
    //recover luma tree status
    partitioner.chType = CH_L;
    partitioner.treeType = TREE_D;
    partitioner.modeType = MODE_TYPE_ALL;
  }

  // Finally, add split-signaling bits for RD-cost check
  tempCS->fracBits += splitBits; // split bits
  tempCS->cost      = m_cRdCost.calcRdCost( tempCS->fracBits, tempCS->dist );
  partitioner.modeType = modeTypeParent;

  // Check Delta QP bits for splitted structure
  if( !qgEnableChildren ) // check at deepest QG level only
    xCheckDQP( *tempCS, partitioner, true );

  // If the configuration being tested exceeds the maximum number of bytes for a slice / slice-segment, then
  // a proper RD evaluation cannot be performed. Therefore, termination of the
  // slice/slice-segment must be made prior to this CTU.
  // This can be achieved by forcing the decision to be that of the rpcTempCU.
  // The exception is each slice / slice-segment must have at least one CTU.
  if( bestCS->cost == MAX_DOUBLE )
  {
    bestCS->costDbOffset = 0;
  }

  if( tempCS->cus.size() > 0 && modeTypeParent == MODE_TYPE_ALL && modeTypeChild == MODE_TYPE_INTER )
  {
    int areaSizeNoResiCu = 0;
    for( int k = 0; k < tempCS->cus.size(); k++ )
    {
      areaSizeNoResiCu += (tempCS->cus[k]->rootCbf == false) ? tempCS->cus[k]->lumaSize().area() : 0;
    }
    if( areaSizeNoResiCu >= (tempCS->area.lumaSize().area() >> 1) )
    {
      skipInterPass = true;
    }
  }

  // RD check for sub partitioned coding structure.
  xCheckBestMode( tempCS, bestCS, partitioner, encTestMode, m_EDO );

  if( isAffMVInfoSaved )
  {
    m_cInterSearch.m_AffineProfList->addAffMVInfo(tmpMVInfo);
  }

  if( !tempCS->slice->isIntra() && isUniMvInfoSaved )
  {
    m_cInterSearch.m_BlkUniMvInfoBuffer->addUniMvInfo(tmpUniMvInfo);
  }

  tempCS->motionLut = oldMotionLut;
  tempCS->releaseIntermediateData();
  tempCS->prevQP[partitioner.chType] = oldPrevQp;
}


void EncCu::xCheckRDCostIntra( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, _TPROF, P_INTRA, tempCS, partitioner.chType );

  tempCS->initStructData( encTestMode.qp, false ); // clear motion buffer

  CodingUnit &cu      = tempCS->addCU( CS::getArea( *tempCS, tempCS->area, partitioner.chType, partitioner.treeType ), partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice            = tempCS->slice;
  cu.tileIdx          = m_tileIdx;
  cu.skip             = false;
  cu.mmvdSkip         = false;
  cu.predMode         = MODE_INTRA;
  cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
  cu.qp               = encTestMode.qp;
  cu.ispMode          = NOT_INTRA_SUBPARTITIONS;
  cu.initPuData();

  m_cIntraSearch.m_ispTestedModes[0].init(0, 0, 1);
  if (m_pcEncCfg->m_FastIntraTools)
  {
    m_modeCtrl.comprCUCtx->intraWasTested = false;
    m_cIntraSearch.m_ispTestedModes[0].relatedCuIsValid = m_modeCtrl.comprCUCtx->relatedCuIsValid;
    if (!bestCS->cus.empty())
    {
      if ((bestCS->cus[0]->mergeFlag || bestCS->cus[0]->imv || bestCS->cus[0]->affine) && (!bestCS->cus[0]->ciip))
      {
        m_cIntraSearch.m_ispTestedModes[0].bestBefore[0] = -1;
      }
    }
    if (!bestCS->slice->isIntra())
    {
      const Position posBL = cu.Y().bottomLeft();
      const Position posTR = cu.Y().topRight();
      for (int i = 0; i < 2; i++)
      {
        const CodingUnit* neigh = i ? cu.cs->getCURestricted(posTR.offset(0, -1), cu, CH_L) :cu.cs->getCURestricted(posBL.offset(-1, 0), cu, CH_L);
        m_cIntraSearch.m_ispTestedModes[0].bestBefore[i+1] = -1;
        if (neigh != nullptr)
        {
          int bestMode = neigh->firstTU->mtsIdx[0] ? 4 : 0;
          bestMode |= neigh->lfnstIdx ? 2 : 0;
          bestMode |= neigh->ispMode ? 1 : 0;
          m_cIntraSearch.m_ispTestedModes[0].bestBefore[i+1] = bestMode;
        }
      }
    }
  }

  tempCS->interHad    = m_modeCtrl.comprCUCtx->interHad;
  double maxCostAllowedForChroma = MAX_DOUBLE;
  if( isLuma( partitioner.chType ) )
  {
    if (!tempCS->slice->isIntra() && bestCS)
    {
      m_cIntraSearch.estIntraPredLumaQT(cu, partitioner, bestCS->cost);
    }
    else
    {
      m_cIntraSearch.estIntraPredLumaQT(cu, partitioner);
    }
    if (m_pcEncCfg->m_FastIntraTools)
    {
      if (m_cIntraSearch.m_ispTestedModes[0].intraWasTested)
      {
        m_modeCtrl.comprCUCtx->intraWasTested = m_cIntraSearch.m_ispTestedModes[0].intraWasTested;
      }
    }

    if( !partitioner.isSepTree( *tempCS ) )
    {
      tempCS->lumaCost = m_cRdCost.calcRdCost( tempCS->fracBits, tempCS->dist );
    }
    if (m_pcEncCfg->m_usePbIntraFast && tempCS->dist == MAX_DISTORTION && tempCS->interHad == 0)
    {
      // JEM assumes only perfect reconstructions can from now on beat the inter mode
      m_modeCtrl.comprCUCtx->interHad = 0;
      return;
    }
  }

  if( tempCS->area.chromaFormat != CHROMA_400 && ( partitioner.chType == CH_C || !CU::isSepTree(cu) ) )
  {
    bool useIntraSubPartitions = cu.ispMode != NOT_INTRA_SUBPARTITIONS;
    Partitioner subTuPartitioner = partitioner;
    if ((m_pcEncCfg->m_ISP >= 3) && (!partitioner.isSepTree(*tempCS) && useIntraSubPartitions))
    {
      maxCostAllowedForChroma = bestCS->cost < MAX_DOUBLE ? bestCS->cost - tempCS->lumaCost : MAX_DOUBLE;
    }
    m_cIntraSearch.estIntraPredChromaQT(
      cu, (!useIntraSubPartitions || (CU::isSepTree(cu) && !isLuma(CH_C))) ? partitioner : subTuPartitioner,
      maxCostAllowedForChroma);
    if ((m_pcEncCfg->m_ISP >= 3) && useIntraSubPartitions && !cu.ispMode)
    {
      return;
    }
  }

  cu.rootCbf = false;

  for (uint32_t t = 0; t < getNumberValidTBlocks(*cu.cs->pcv); t++)
  {
    cu.rootCbf |= cu.firstTU->cbf[t] != 0;
  }

  // Get total bits for current mode: encode CU
  m_CABACEstimator->resetBits();

  if ((!cu.cs->slice->isIntra() || cu.cs->slice->sps->IBC) && cu.Y().valid())
  {
    m_CABACEstimator->cu_skip_flag(cu);
  }
  m_CABACEstimator->pred_mode(cu);
  m_CABACEstimator->cu_pred_data(cu);

  // Encode Coefficients
  CUCtx cuCtx;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;
  m_CABACEstimator->cu_residual(cu, partitioner, cuCtx);

  tempCS->fracBits = m_CABACEstimator->getEstFracBits();
  tempCS->cost = m_cRdCost.calcRdCost(tempCS->fracBits, tempCS->dist);

  xEncodeDontSplit(*tempCS, partitioner);

  xCheckDQP(*tempCS, partitioner);

  if( m_EDO )
  {
    xCalDebCost(*tempCS, partitioner);
  }

  DTRACE_MODE_COST(*tempCS, m_cRdCost.getLambda(true));
  xCheckBestMode(tempCS, bestCS, partitioner, encTestMode, m_EDO);

  STAT_COUNT_CU_MODES( partitioner.chType == CH_L, g_cuCounters1D[CU_MODES_TESTED][0][!tempCS->slice->isIntra() + tempCS->slice->depth] );
  STAT_COUNT_CU_MODES( partitioner.chType == CH_L && !tempCS->slice->isIntra(), g_cuCounters2D[CU_MODES_TESTED][Log2( tempCS->area.lheight() )][Log2( tempCS->area.lwidth() )] );
}

void EncCu::xCheckDQP( CodingStructure& cs, Partitioner& partitioner, bool bKeepCtx )
{
  if( !cs.pps->useDQP )
  {
    return;
  }

  if (partitioner.isSepTree(cs) && isChroma(partitioner.chType))
  {
    return;
  }

  if( !partitioner.currQgEnable() ) // do not consider split or leaf/not leaf QG condition (checked by caller)
  {
    return;
  }

  CodingUnit* cuFirst = cs.getCU( partitioner.chType, partitioner.treeType );

  CHECK( bKeepCtx && cs.cus.size() <= 1 && partitioner.getImplicitSplit( cs ) == CU_DONT_SPLIT, "bKeepCtx should only be set in split case" );
  CHECK( !bKeepCtx && cs.cus.size() > 1, "bKeepCtx should never be set for non-split case" );
  CHECK( !cuFirst, "No CU available" );

  bool hasResidual = false;
  for( const auto &cu : cs.cus )
  {
    //not include the chroma CU because chroma CU is decided based on corresponding luma QP and deltaQP is not signaled at chroma CU
    if( cu->rootCbf && !isChroma( cu->chType ))
    {
      hasResidual = true;
      break;
    }
  }

  int predQP = CU::predictQP( *cuFirst, cs.prevQP[partitioner.chType] );

  if( hasResidual )
  {
    TempCtx ctxTemp( m_CtxCache );
    if( !bKeepCtx ) ctxTemp = SubCtx( Ctx::DeltaQP, m_CABACEstimator->getCtx() );

    m_CABACEstimator->resetBits();
    m_CABACEstimator->cu_qp_delta( *cuFirst, predQP, cuFirst->qp );

    cs.fracBits += m_CABACEstimator->getEstFracBits(); // dQP bits
    cs.cost      = m_cRdCost.calcRdCost(cs.fracBits, cs.dist);


    if( !bKeepCtx ) m_CABACEstimator->getCtx() = SubCtx( Ctx::DeltaQP, ctxTemp );

    // NOTE: reset QPs for CUs without residuals up to first coded CU
    for( const auto &cu : cs.cus )
    {
      //not include the chroma CU because chroma CU is decided based on corresponding luma QP and deltaQP is not signaled at chroma CU
      if( cu->rootCbf && !isChroma( cu->chType ))
      {
        break;
      }
      cu->qp = predQP;
    }
  }
  else
  {
    // No residuals: reset CU QP to predicted value
    for( const auto &cu : cs.cus )
    {
      cu->qp = predQP;
    }
  }
}

CodingUnit *EncCu::getCuForInterPrediction( CodingStructure *cs, const EncTestMode& encTestMode )
{
  CodingUnit *cu = cs->getCU( CH_L, TREE_D );

  if( cu == nullptr )
  {
    CHECK( cs->getCU( CH_L, TREE_D ) != nullptr, "Wrong CU/PU setting in CS" );
    cu = &cs->addCU( cs->area, CH_L );
  }

  cu->slice       = cs->slice;
  cu->tileIdx     = m_tileIdx;
  cu->skip        = false;
  cu->mmvdSkip    = false;
  cu->mmvdMergeFlag
                  = false;
  cu->geo         = false;
  cu->predMode    = MODE_INTER;
  cu->chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
  cu->qp          = encTestMode.qp;
  cu->affine      = false;
  cu->multiRefIdx = 0;
  cu->mipFlag     = false;
  cu->ciip        = false;

  return cu;
}

int getDmvrMvdNum( const CodingUnit &cu )
{
  const int dx = std::max<int>( cu.lwidth()  >> DMVR_SUBCU_SIZE_LOG2, 1 );
  const int dy = std::max<int>( cu.lheight() >> DMVR_SUBCU_SIZE_LOG2, 1 );
  return dx * dy;
}

void EncCu::xCheckRDCostUnifiedMerge( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, EncTestMode &encTestMode )
{
  const Slice &slice = *tempCS->slice;

  CHECK( slice.sliceType == VVENC_I_SLICE, "Merge modes not available for I-slices" );

  tempCS->initStructData( encTestMode.qp );

  MergeCtx          mergeCtx, gpmMergeCtx;
  AffineMergeCtx    affineMergeCtx;
  GeoComboCostList &comboList = m_comboList;
  const SPS        &sps       = *tempCS->sps;

  if( sps.SbtMvp )
  {
    const Size bufSize           = g_miScaling.scale( tempCS->area.lumaSize() );
    affineMergeCtx.subPuMvpMiBuf = MotionBuf        ( m_subPuMiBuf, bufSize );
  }

  m_mergeBestSATDCost = MAX_DOUBLE;

  CodingUnit *cu = getCuForInterPrediction( tempCS, encTestMode );
  partitioner.setCUData            ( *cu );
  CU::getInterMergeCandidates      ( *cu, mergeCtx, 0 );
  if( sps.MMVD )
    CU::getInterMMVDMergeCandidates( *cu, mergeCtx );

  bool sameMV[MRG_MAX_NUM_CANDS] = { false, };
  if( m_pcEncCfg->m_useFastMrg >= 2 )
  {
    for( int m = 0; m < mergeCtx.numValidMergeCand - 1; m++ )
    {
      if( !sameMV[m] )
      {
        for( int n = m + 1; n < mergeCtx.numValidMergeCand; n++ )
        {
          sameMV[n] |= mergeCtx.mvFieldNeighbours[m][0] == mergeCtx.mvFieldNeighbours[n][0]
                    && mergeCtx.mvFieldNeighbours[m][1] == mergeCtx.mvFieldNeighbours[n][1];
        }
      }
    }
  }

  MergeBufVector mrgPredBufNoCiip;
  MergeBufVector geoBuffer;
  const double  sqrtLambdaForFirstPass = m_cRdCost.getMotionLambda() * FRAC_BITS_SCALE;

  const UnitArea localUnitArea( cu->chromaFormat, Area( 0, 0, cu->Y().width, cu->Y().height ) );
  for( int i = 0; i < mergeCtx.numValidMergeCand; i++ )
  {
    mrgPredBufNoCiip.push_back( m_acMergeTmpBuffer[i].getCompactBuf( localUnitArea ) );
  }

  int numMergeSatdCand = std::min( bestCS->area.lumaSize().area() >= 64 ? m_pcEncCfg->m_mergeRdCandQuotaRegular : m_pcEncCfg->m_mergeRdCandQuotaRegularSmallBlk, mergeCtx.numValidMergeCand );

  bool isCiipEnabled  = sps.CIIP && bestCS->area.lumaSize().area() >= 64 && bestCS->area.lumaSize().maxDim() < MAX_CU_SIZE;
       isCiipEnabled &= m_pcEncCfg->m_CIIP <= 1 || !m_modeCtrl.getBlkInfo( tempCS->area ).isSkip; //5

  if( isCiipEnabled )
  {
    numMergeSatdCand += std::min( m_pcEncCfg->m_mergeRdCandQuotaCiip, mergeCtx.numValidMergeCand );
  }

  const bool affineMrgAvail = ( m_pcEncCfg->m_Affine <= 2 || slice.TLayer <= 3 || m_pcEncCfg->m_SbTMVP )
                           && ( m_pcEncCfg->m_Affine || sps.SbtMvp ) && m_pcEncCfg->m_maxNumAffineMergeCand && bestCS->area.Y().minDim() >= 8;

  if( affineMrgAvail )
  {
    CU::getAffineMergeCand( *cu, affineMergeCtx );
    numMergeSatdCand += std::min( m_pcEncCfg->m_mergeRdCandQuotaSubBlk, affineMergeCtx.numValidMergeCand );
  }

  int numSatdCandPreGeo = std::min( numMergeSatdCand, m_pcEncCfg->m_maxMergeRdCandNumTotal );
  bool toAddGpmCand     = false;
  if( sps.GEO && slice.isInterB() // base checks
      && cu->lumaSize().minDim() >= GEO_MIN_CU_SIZE  && cu->lumaSize().maxDim() <= GEO_MAX_CU_SIZE && cu->lumaSize().maxDim() < 8 * cu->lumaSize().minDim() // size checks
      && !( m_pcEncCfg->m_Geo > 2 && slice.TLayer <= 1 ) ) // speedups
  {
    cu->mergeFlag            = true;
    cu->geo                  = true;
    CU::getGeoMergeCandidates( *cu, gpmMergeCtx );
    toAddGpmCand             = prepareGpmComboList( gpmMergeCtx, localUnitArea, sqrtLambdaForFirstPass, comboList, geoBuffer, *cu );
    numMergeSatdCand        += toAddGpmCand ? std::min( m_pcEncCfg->m_mergeRdCandQuotaGpm, ( int ) comboList.list.size() ) : 0;
  }

  numMergeSatdCand  = std::min( numMergeSatdCand, m_pcEncCfg->m_maxMergeRdCandNumTotal );

  // 1. Pass: get SATD-cost for selected candidates and reduce their count
  m_mergeItemList.resetList( numMergeSatdCand );
  const TempCtx ctxStart   ( m_CtxCache, m_CABACEstimator->getCtx() );
  const DFunc   dfunc      = encTestMode.lossless ? DF_SAD : ( m_pcEncCfg->m_fastHad ? DF_HAD_fast : DF_HAD );
  DistParam     distParam  = m_cRdCost.setDistParam( tempCS->getOrgBuf().Y(), tempCS->getOrgBuf().Y(), sps.bitDepths[CH_L], dfunc );
  m_uiSadBestForQPA        = MAX_DISTORTION;

  addRegularCandsToPruningList( mergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart, distParam, *cu, sameMV, mrgPredBufNoCiip );

  // add CIIP candidates directly after adding regular cands
  if( isCiipEnabled )
  {
    addCiipCandsToPruningList( mergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart, distParam, *cu, sameMV );
  }

  if( sps.MMVD && !( m_pcEncCfg->m_useFastMrg >= 2 && m_mergeItemList.size() <= 1 ) )
  {
    addMmvdCandsToPruningList( mergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart, distParam, *cu );
  }

  if( affineMergeCtx.numValidMergeCand > 0 )
  {
    addAffineCandsToPruningList( affineMergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart, distParam, *cu );
  }

  if( m_pcEncCfg->m_useFastMrg > 0 && m_mergeItemList.size() > 0 )
  {
    m_mergeBestSATDCost    = m_mergeItemList.getMergeItemInList( 0 )->cost;
    const double threshold = m_mergeBestSATDCost * MRG_FAST_RATIO[tempCS->picture->useFastMrg];
    const   int shrinkSize = std::min( numSatdCandPreGeo, ( int ) updateRdCheckingNum( m_mergeItemList, threshold, numMergeSatdCand ) );
    m_mergeItemList        . shrinkList( shrinkSize );
  }
  else
  {
    m_mergeItemList        . shrinkList( numSatdCandPreGeo );
  }

  if( toAddGpmCand )
  {
    addGpmCandsToPruningList( gpmMergeCtx, localUnitArea, sqrtLambdaForFirstPass, ctxStart, comboList, geoBuffer, distParam, *cu );
  }

  if(    m_pcEncCfg->m_internalUsePerceptQPATempFiltISlice == 2 && m_uiSadBestForQPA < MAX_DISTORTION && slice.TLayer == 0 // non-Intra key-frame
      && m_pcEncCfg->m_salienceBasedOpt
      && m_pcEncCfg->m_usePerceptQPA && partitioner.currQgEnable() && partitioner.currSubdiv == 0 ) // CTU-level luma quantization group
  {
    CHECK( bestCS->cost < MAX_DOUBLE, "This has to be the first test performed!" );

    const Picture *pic         = slice.pic;
    const bool     isBIM       = m_pcEncCfg->m_RCNumPasses != 2 && m_pcEncCfg->m_blockImportanceMapping && !pic->m_picShared->m_ctuBimQpOffset.empty();
    const uint32_t rsAddr      = getCtuAddr( partitioner.currQgPos, *pic->cs->pcv );
    const int      pumpReducQP = BitAllocation::getCtuPumpingReducingQP( &slice, tempCS->getOrgBuf( COMP_Y ), m_uiSadBestForQPA, *m_globalCtuQpVector, rsAddr,
                                                                         m_pcEncCfg->m_QP, isBIM );

    if( pumpReducQP != 0 ) // subtract QP offset, reduces Intra-period pumping or overcoding
    {
      encTestMode.qp = Clip3( 0, MAX_QP, encTestMode.qp - pumpReducQP );
      tempCS->currQP[partitioner.chType] = tempCS->baseQP =
      bestCS->currQP[partitioner.chType] = bestCS->baseQP = Clip3( 0, MAX_QP, tempCS->baseQP - pumpReducQP );

      updateLambda( slice, pic->ctuQpaLambda[rsAddr], pic->ctuAdaptedQP[rsAddr], tempCS->baseQP, true );
    }
  }

  // Try to limit number of candidates using SATD-costs
  if( m_pcEncCfg->m_useFastMrg > 0 && m_mergeItemList.size() > 0 )
  {
    // shrink GEO list as well
    const double threshold = m_mergeItemList.getMergeItemInList( 0 )->cost * MRG_FAST_RATIO[0];
    numMergeSatdCand       = updateRdCheckingNum( m_mergeItemList, threshold, numMergeSatdCand );
    m_mergeBestSATDCost    = m_mergeItemList.size() != 0 ? m_mergeItemList.getMergeItemInList( 0 )->cost : MAX_DOUBLE;
  }
  else
  {
    numMergeSatdCand       = std::min<int>( numMergeSatdCand, ( int ) m_mergeItemList.size() );
  }

  // 2. Pass: RD checking 
  tempCS->initStructData( encTestMode.qp );
  m_CABACEstimator->getCtx() = ctxStart;

  double bestEndCost                            =   MAX_DOUBLE;
  bool bestIsSkip                               =   false;
  PelUnitBuf ciipBuf                            =   m_aTmpStorageLCU[1].getCompactBuf( *cu );
  bool ciipChromaDone                           =   false;
  bool isRegularTestedAsSkip[MRG_MAX_NUM_CANDS] = { false, };
  bool geoWasTested                             =   false;
  int  stopCand                                 =   numMergeSatdCand;

  CHECK( numMergeSatdCand > 0 && m_mergeItemList.size() == 0, "Empty merge item list is not expected" );

  for( uint32_t noResidualPass = 0; noResidualPass < 2; noResidualPass++ )
  {
    const bool forceNoResidual = noResidualPass == 1;
    for( uint32_t mrgHadIdx = 0; mrgHadIdx < stopCand; mrgHadIdx++ )
    {
      auto mergeItem = m_mergeItemList.getMergeItemInList( mrgHadIdx );
      CHECK( mergeItem == nullptr, "Wrong merge item" );

      const bool isCiip = mergeItem->mergeItemType == MergeItem::MergeItemType::CIIP;
      const bool isGeo  = mergeItem->mergeItemType == MergeItem::MergeItemType::GPM;
      const bool isRglr = mergeItem->mergeItemType == MergeItem::MergeItemType::REGULAR;
      const bool isMmvd = mergeItem->mergeItemType == MergeItem::MergeItemType::MMVD;

      if( noResidualPass != 0 && isCiip && isRegularTestedAsSkip[mergeItem->mergeIdx] )
      {
        continue;
      }

      if( noResidualPass ? mergeItem->noResidual : bestIsSkip )
      {
        continue;
      }

      if( isGeo )
      {
        if( m_pcEncCfg->m_Geo > 2 && geoWasTested && !bestCS->cus.empty() && !bestCS->getCU( partitioner.chType, partitioner.treeType )->geo )
        {
          continue;
        }

        geoWasTested = true;
      }

      cu = getCuForInterPrediction( tempCS, encTestMode );
      partitioner.setCUData( *cu );
      const bool resetCiip2Regular = mergeItem->exportMergeInfo( *cu, forceNoResidual );

      if( isRglr || resetCiip2Regular )
      {
        if( CU::checkDMVRCondition( *cu ) ) std::copy_n( m_subPuMvOffset[mergeItem->mergeIdx].data(), getDmvrMvdNum( *cu ), cu->mvdL0SubPu );
      }

      if( isMmvd && mergeItem->noBdofRefine )
      {
        // no BDOF refinement was made for the luma prediction, need to have luma prediction again
        mergeItem->lumaPredReady = false;
      }

      PelUnitBuf *predBuf1   = nullptr, *predBuf2 = isCiip ? &ciipBuf : nullptr;
      PelUnitBuf  dstPredBuf = tempCS->getPredBuf( *cu );

      if( isGeo )
      {
        predBuf1 = &geoBuffer[cu->geoMergeIdx[0]];
        predBuf2 = &geoBuffer[cu->geoMergeIdx[1]];
      }

      if( resetCiip2Regular )
      {
        dstPredBuf.copyFrom( mrgPredBufNoCiip[mergeItem->mergeIdx] );
      }
      else
      {
        if( isCiip && !resetCiip2Regular && isChromaEnabled( cu->chromaFormat ) && cu->chromaSize().width > 2 )
        {
          if( !ciipChromaDone )
          {
            cu->intraDir[0] = PLANAR_IDX;
            cu->intraDir[1] = DM_CHROMA_IDX;

            m_cIntraSearch  . initIntraPatternChType( *cu, cu->Cb() );
            m_cIntraSearch  . predIntraAng          ( COMP_Cb, ciipBuf.Cb(), *cu );
            m_cIntraSearch  . initIntraPatternChType( *cu, cu->Cr() );
            m_cIntraSearch  . predIntraAng          ( COMP_Cr, ciipBuf.Cr(), *cu );

            ciipChromaDone  = true;
          }
        }

        if(  mergeItem->lumaPredReady ||  mergeItem->chromaPredReady )
          dstPredBuf.copyFrom( mergeItem->getPredBuf( localUnitArea ), mergeItem->lumaPredReady, mergeItem->chromaPredReady );
        if( !mergeItem->lumaPredReady || !mergeItem->chromaPredReady )
          generateMergePrediction( localUnitArea, mergeItem, *cu, !mergeItem->lumaPredReady, !mergeItem->chromaPredReady, dstPredBuf, true, forceNoResidual, predBuf1, predBuf2 );
      }

      if( !cu->mmvdSkip && !cu->ciip && !cu->affine && !cu->geo && noResidualPass != 0 )
      {
        CHECK( mergeItem->mergeIdx >= mergeCtx.numValidMergeCand, "out of normal merge" );
        isRegularTestedAsSkip[mergeItem->mergeIdx] = true;
      }

      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, noResidualPass, noResidualPass == 0 ? &mergeItem->noResidual : nullptr );

      if( m_pcEncCfg->m_useFastMrg >= 2 )
      {
        if( cu->ciip && bestCS->cost == MAX_DOUBLE && mrgHadIdx + 1 == numMergeSatdCand )
        {
          numMergeSatdCand = ( unsigned ) m_mergeItemList.size();
        }
      
        if( mrgHadIdx > 0 && tempCS->cost >= bestEndCost && !cu->ciip && !isGeo )
        {
          stopCand = mrgHadIdx + 1;
        }
      
        if( noResidualPass == 0 )
        {
          bestEndCost = std::min( bestEndCost, tempCS->cost );
        }
      }

      if( m_pcEncCfg->m_useFastDecisionForMerge && !bestIsSkip && !cu->ciip )
      {
        bestIsSkip = !bestCS->cus.empty() && bestCS->getCU( partitioner.chType, partitioner.treeType )->rootCbf == 0;
      }

      tempCS->initStructData( encTestMode.qp );
    }   // end loop mrgHadIdx
  }
}

unsigned int EncCu::updateRdCheckingNum( MergeItemList &mergeItemList, double threshold, unsigned int numMergeSatdCand )
{
  for( uint32_t i = 0; i < mergeItemList.size(); i++ )
  {
    const auto mergeItem = mergeItemList.getMergeItemInList( i );
    if( mergeItem == nullptr || mergeItem->cost > threshold )
    {
      numMergeSatdCand = i;
      break;
    }
  }
  return std::min( numMergeSatdCand, ( unsigned ) mergeItemList.size() );
}

void EncCu::generateMergePrediction( const UnitArea &unitArea, MergeItem *mergeItem, CodingUnit &pu, bool luma, bool chroma,
                                     PelUnitBuf &dstBuf, bool finalRd, bool forceNoResidual, PelUnitBuf *predBuf1, PelUnitBuf *predBuf2 )
{
  CHECK( ( luma && mergeItem->lumaPredReady ) || ( chroma && mergeItem->chromaPredReady ), "Prediction has been avaiable" );

  pu.mcControl = ( !luma ? 4 : 0 ) | ( !chroma ? 2 : 0 );

  switch( mergeItem->mergeItemType )
  {
  case MergeItem::MergeItemType::REGULAR:
    // here predBuf1 is predBufNoCiip
    pu.mvRefine = true;
    m_cInterSearch.motionCompensation( pu, dstBuf, REF_PIC_LIST_X );
    pu.mvRefine = false;
    if( predBuf1 != nullptr )
    {
      predBuf1->copyFrom( dstBuf, luma, chroma );
    }
    break;

  case MergeItem::MergeItemType::CIIP:
    m_cInterSearch.motionCompensation( pu, dstBuf, REF_PIC_LIST_X );

    if( luma )
    {
      const ReshapeData& reshapeData = pu.cs->picture->reshapeData;
      if( pu.cs->slice->lmcsEnabled && reshapeData.getCTUFlag() )
      {
        dstBuf.Y().rspSignal( reshapeData.getFwdLUT() );
      }
      // generate intrainter Y prediction
      dstBuf.Y().weightCiip( predBuf2->Y(), mergeItem->numCiipIntra );
    }

    if( chroma )
    {
      if( pu.chromaSize().width > 2 )
      {
        dstBuf.Cb().weightCiip( predBuf2->Cb(), mergeItem->numCiipIntra );
        dstBuf.Cr().weightCiip( predBuf2->Cr(), mergeItem->numCiipIntra );
      }
    }

    break;

  case MergeItem::MergeItemType::MMVD:
    pu.mcControl           |= finalRd ? 0 : ( pu.mmvdMergeIdx.pos.step > 2 || m_pcEncCfg->m_MMVD > 1 ) ? 1 : 0;
    mergeItem->noBdofRefine = pu.mccNoBdof() && pu.cs->sps->BDOF && !pu.cs->picHeader->disBdofFlag;
    m_cInterSearch.motionCompensation( pu, dstBuf, REF_PIC_LIST_X );
    break;

  case MergeItem::MergeItemType::SBTMVP:
    m_cInterSearch.motionCompensation( pu, dstBuf, REF_PIC_LIST_X );
    break;

  case MergeItem::MergeItemType::AFFINE:
    m_cInterSearch.motionCompensation( pu, dstBuf, REF_PIC_LIST_X );
    break;

  case MergeItem::MergeItemType::GPM:
    // here predBuf1 and predBuf2 point to geoBuffer[mergeCand0] and geoBuffer[mergeCand1], respectively
    CHECK( predBuf1 == nullptr || predBuf2 == nullptr, "Invalid input buffer to GPM" );
    m_cInterSearch.weightedGeoBlk( pu.slice->clpRngs, pu, pu.geoSplitDir, luma && chroma ? MAX_NUM_CH : luma ? CH_L : CH_C, dstBuf, *predBuf1, *predBuf2 );
    break;

  default:
    THROW("Wrong merge item type");
  }

  auto mergeItemPredBuf = mergeItem->getPredBuf( unitArea );

  if( dstBuf.Y().buf == mergeItemPredBuf.Y().buf )
  {
    // dst is the internal buffer
    mergeItem->lumaPredReady   |= luma;
    mergeItem->chromaPredReady |= chroma;
  }
  else if( finalRd && !forceNoResidual )
  {
    // at final RD stage, with and without residuals are both checked
    // it makes sense to buffer the prediction
    mergeItemPredBuf.copyFrom( dstBuf, luma, chroma );
    mergeItem->lumaPredReady   |= luma;
    mergeItem->chromaPredReady |= chroma;
  }
}

void EncCu::addRegularCandsToPruningList( const MergeCtx &mergeCtx, const UnitArea &localUnitArea, double sqrtLambdaForFirstPassIntra, const TempCtx &ctxStart,
                                          DistParam& distParam, CodingUnit& pu, bool* sameMv, MergeBufVector& regularPred )
{
  pu.geo = pu.affine
         = pu.mmvdMergeFlag = pu.mmvdSkip
         = pu.ciip
         = false;

  for( uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++ )
  {
    if( sameMv[uiMergeCand] ) continue;

    mergeCtx.setMergeInfo   ( pu, uiMergeCand );

    if( m_pcEncCfg->m_ifpLines && // what about DMVR?
        ( ( pu.refIdx[L0] >= 0 && !CU::isMvInRangeFPP( pu.ly(), pu.lheight(), pu.mv[L0][0].ver, m_pcEncCfg->m_ifpLines, *pu.cs->pcv ) ) ||
          ( pu.refIdx[L1] >= 0 && !CU::isMvInRangeFPP( pu.ly(), pu.lheight(), pu.mv[L1][0].ver, m_pcEncCfg->m_ifpLines, *pu.cs->pcv ) ) ) )
    {
      continue;
    }

    pu.interDir             = mergeCtx.interDirNeighbours[uiMergeCand];
    pu.BcwIdx               = pu.interDir == 3 ? mergeCtx.BcwIdx[uiMergeCand] : BCW_DEFAULT;
    pu.imv                  = mergeCtx.useAltHpelIf[uiMergeCand] ? IMV_HPEL : 0;
    CU::spanMotionInfo      ( pu );

    MergeItem *regularMerge = m_mergeItemList.allocateNewMergeItem();
    regularMerge->importMergeInfo( mergeCtx, uiMergeCand, MergeItem::MergeItemType::REGULAR, pu );
    auto dstBuf             = regularMerge->getPredBuf( localUnitArea );
    generateMergePrediction ( localUnitArea, regularMerge, pu, true, true, dstBuf, false, false, &regularPred[uiMergeCand], nullptr );
    regularMerge->cost      = calcLumaCost4MergePrediction( ctxStart, dstBuf, sqrtLambdaForFirstPassIntra, pu, distParam );
    if( CU::checkDMVRCondition( pu ) ) std::copy_n( pu.mvdL0SubPu, getDmvrMvdNum( pu ), m_subPuMvOffset[uiMergeCand].data() );
    m_mergeItemList         . insertMergeItemToList( regularMerge );
  }
}

void EncCu::addCiipCandsToPruningList( const MergeCtx &mergeCtx, const UnitArea &localUnitArea, double sqrtLambdaForFirstPassIntra, const TempCtx &ctxStart, DistParam &distParam, CodingUnit &pu, bool* sameMv )
{
  const ReshapeData& reshapeData  = pu.cs->picture->reshapeData;
  int                numCiipIntra = -1;
  PelUnitBuf         rspBuffer    = m_aTmpStorageLCU[0].getCompactBuf( pu );
  PelUnitBuf         ciipBuf      = m_aTmpStorageLCU[1].getCompactBuf( pu );

  pu.ciip        = true;
  pu.intraDir[0] = PLANAR_IDX;
  pu.geo         = pu.affine
                 = pu.mmvdMergeFlag = pu.mmvdSkip
                 = false;
  m_cIntraSearch . initIntraPatternChType        ( pu, pu.Y() );
  m_cIntraSearch . predIntraAng                  ( COMP_Y, ciipBuf.Y(), pu );
  numCiipIntra   = m_cIntraSearch.getNumIntraCiip( pu );

  int nonCiipMrgCnds[MRG_MAX_NUM_CANDS] = { 0, };
  int numNonCiipCnds                    =   0;
  for( ; numNonCiipCnds < m_mergeItemList.size(); numNonCiipCnds++ ) nonCiipMrgCnds[numNonCiipCnds] = m_mergeItemList.getMergeItemInList( numNonCiipCnds )->mergeIdx;

  for( int i = 0; i < numNonCiipCnds; i++ )
  {
    const unsigned int uiMergeCand = nonCiipMrgCnds[i];

    if( sameMv[uiMergeCand] ) continue;

    mergeCtx.setMergeInfo     ( pu, uiMergeCand );

    if( m_pcEncCfg->m_ifpLines && 
        ( ( pu.refIdx[L0] >= 0 && !CU::isMvInRangeFPP( pu.ly(), pu.lheight(), pu.mv[L0][0].ver, m_pcEncCfg->m_ifpLines, *pu.cs->pcv ) ) ||
          ( pu.refIdx[L1] >= 0 && !CU::isMvInRangeFPP( pu.ly(), pu.lheight(), pu.mv[L1][0].ver, m_pcEncCfg->m_ifpLines, *pu.cs->pcv ) ) ) )
    {
      continue;
    }

    pu.interDir               = mergeCtx.interDirNeighbours[uiMergeCand];
    pu.BcwIdx                 = pu.interDir == 3 ? mergeCtx.BcwIdx[uiMergeCand] : BCW_DEFAULT;
    pu.imv                    = mergeCtx.useAltHpelIf[uiMergeCand] ? IMV_HPEL : 0;
    CU::spanMotionInfo        ( pu );

    MergeItem* ciipMerge      = m_mergeItemList.allocateNewMergeItem();
    ciipMerge->importMergeInfo( mergeCtx, uiMergeCand, MergeItem::MergeItemType::CIIP, pu );
    ciipMerge->numCiipIntra   = numCiipIntra;
    auto dstBuf               = ciipMerge->getPredBuf( localUnitArea );
    generateMergePrediction   ( localUnitArea, ciipMerge, pu, true, false, dstBuf, false, false, nullptr, &ciipBuf );

    if( pu.cs->slice->lmcsEnabled && reshapeData.getCTUFlag() )
    {
      // distortion is calculated in the original domain
      rspBuffer.Y()           . rspSignal( dstBuf.Y(), reshapeData.getInvLUT() );
      ciipMerge->cost         = calcLumaCost4MergePrediction( ctxStart, rspBuffer, sqrtLambdaForFirstPassIntra, pu, distParam );
    }
    else
    {
      ciipMerge->cost         = calcLumaCost4MergePrediction( ctxStart, dstBuf, sqrtLambdaForFirstPassIntra, pu, distParam );
    }
    if( !m_mergeItemList      . insertMergeItemToList( ciipMerge ) && m_pcEncCfg->m_CIIP > 1 )
    {
      break;
    }
  }
}

void EncCu::addMmvdCandsToPruningList( const MergeCtx &mergeCtx, const UnitArea &localUnitArea, double sqrtLambdaForFirstPassIntra, const TempCtx& ctxStart,
                                       DistParam& distParam, CodingUnit& pu )
{
  pu.mmvdSkip              = true;
  pu.affine                = pu.geo
                           = pu.ciip
                           = false;

  int       mmvdTestNum    = mergeCtx.numValidMergeCand > 1 ? MmvdIdx::ADD_NUM : MmvdIdx::ADD_NUM >> 1;
  int       bestDir        = 0;
  size_t    curListSize    = m_mergeItemList.size();
  double    bestCostMerge  = m_mergeItemList.getMergeItemInList( curListSize - 1 )->cost;
  double    bestCostOffset = MAX_DOUBLE;
  int       shiftCandStart = 0;

  if( m_pcEncCfg->m_MMVD == 4 )
  {
    const int cnd1idx = m_mergeItemList.size() == 1 ? 0 : 1;
    const int mrgCnd0 = m_mergeItemList.getMergeItemInList(       0 )->mergeIdx;
    const int mrgCnd1 = m_mergeItemList.getMergeItemInList( cnd1idx )->mergeIdx;

    if( mrgCnd0 > 1 && mrgCnd1 > 1 )
    {
      mmvdTestNum = 0;
    }
    else if( mrgCnd0 > 1 || mrgCnd1 > 1 )
    {
      int shiftCand = mrgCnd0 < 2 ? mrgCnd0 : mrgCnd1;

      if( shiftCand )
      {
        shiftCandStart = MMVD_MAX_REFINE_NUM;
      }
      else
      {
        mmvdTestNum    = MMVD_MAX_REFINE_NUM;
      }
    }
  }

  for( int mmvdMergeCand = shiftCandStart; mmvdMergeCand < mmvdTestNum; mmvdMergeCand++ )
  {
    MmvdIdx mmvdIdx;
    mmvdIdx.val = mmvdMergeCand;

    if( mmvdIdx.pos.step >= m_pcEncCfg->m_MmvdDisNum )
    {
      continue;
    }

    if( m_pcEncCfg->m_MMVD > 1 )
    {
      int checkMMVD = xCheckMMVDCand( mmvdIdx, bestDir, mmvdTestNum, bestCostOffset, bestCostMerge, m_mergeItemList.getMergeItemInList( curListSize - 1 )->cost );
      mmvdMergeCand = mmvdIdx.val;

      if( checkMMVD )
      {
        if( checkMMVD == 2 )
        {
          break;
        }
        continue;
      }
    }

    mergeCtx.setMmvdMergeCandiInfo( pu, mmvdIdx );

    if( m_pcEncCfg->m_ifpLines &&
        ( ( pu.refIdx[L0] >= 0 && !CU::isMvInRangeFPP( pu.ly(), pu.lheight(), pu.mv[L0][0].ver, m_pcEncCfg->m_ifpLines, *pu.cs->pcv ) ) ||
          ( pu.refIdx[L1] >= 0 && !CU::isMvInRangeFPP( pu.ly(), pu.lheight(), pu.mv[L1][0].ver, m_pcEncCfg->m_ifpLines, *pu.cs->pcv ) ) ) )
    {
      // skip candidate
      continue;
    }

    pu.interDir               = mergeCtx.interDirNeighbours[mmvdIdx.pos.baseIdx];
    pu.BcwIdx                 = pu.interDir == 3 ? mergeCtx.BcwIdx[mmvdIdx.pos.baseIdx] : BCW_DEFAULT;
    pu.imv                    = mergeCtx.useAltHpelIf[mmvdIdx.pos.baseIdx] ? IMV_HPEL : 0;
    CU::spanMotionInfo        ( pu );

    MergeItem *mmvdMerge      = m_mergeItemList.allocateNewMergeItem();
    mmvdMerge->importMergeInfo( mergeCtx, mmvdIdx.val, MergeItem::MergeItemType::MMVD, pu );
    auto dstBuf               = mmvdMerge->getPredBuf( localUnitArea );
    generateMergePrediction   ( localUnitArea, mmvdMerge, pu, true, false, dstBuf, false, false, nullptr, nullptr );
    mmvdMerge->cost           = calcLumaCost4MergePrediction( ctxStart, dstBuf, sqrtLambdaForFirstPassIntra, pu, distParam );
    m_mergeItemList           . insertMergeItemToList( mmvdMerge );

    if( m_pcEncCfg->m_MMVD > 1 && mmvdMerge->cost < bestCostOffset )
    {
      bestCostOffset          = mmvdMerge->cost;
      int CandCur             = mmvdIdx.val - MMVD_MAX_REFINE_NUM * mmvdIdx.pos.baseIdx;
      if( CandCur < 4 )
        bestDir               = CandCur;
    }
  }

  if( m_pcEncCfg->m_useFastMrg >= 2 )
  {
    m_mergeItemList           . shrinkList( curListSize );
  }
}

void EncCu::addAffineCandsToPruningList( AffineMergeCtx &affineMergeCtx, const UnitArea &localUnitArea, double sqrtLambdaForFirstPass,
                                         const TempCtx& ctxStart, DistParam& distParam, CodingUnit& pu)
{
  bool sameMV[AFFINE_MRG_MAX_NUM_CANDS + 1]
                      = { false, };
  size_t curListSize  = m_mergeItemList.size();

  pu.mergeFlag = true;
  pu.affine    = true;
  pu.imv       = 0;
  pu.geo       = pu.mmvdMergeFlag = pu.mmvdSkip
               = pu.ciip
               = false;

  if( m_pcEncCfg->m_Affine > 1 )
  {
    for( int m = 0; m < affineMergeCtx.numValidMergeCand; m++ )
    {
      if( pu.cs->slice->TLayer > 3 && affineMergeCtx.mergeType[m] != MRG_TYPE_SUBPU_ATMVP )
      {
        sameMV[m] = m != 0;
      }
      else if( !sameMV[m + 1] )
      {
        for( int n = m + 1; n < affineMergeCtx.numValidMergeCand; n++ )
        {
          sameMV[n] |= affineMergeCtx.mvFieldNeighbours[m][0][0] == affineMergeCtx.mvFieldNeighbours[n][0][0]
                    && affineMergeCtx.mvFieldNeighbours[m][1][0] == affineMergeCtx.mvFieldNeighbours[n][1][0];
        }
      }
    }
  }

  for( uint32_t mergeIdx = 0; mergeIdx < affineMergeCtx.numValidMergeCand; mergeIdx++ )
  {
    if( ( affineMergeCtx.mergeType[mergeIdx] != MRG_TYPE_SUBPU_ATMVP && m_pcEncCfg->m_Affine == 0 ) || sameMV[mergeIdx] )
    {
      continue;
    }

    pu.mergeType              = affineMergeCtx.mergeType[mergeIdx];
    pu.affineType             = affineMergeCtx.affineType[mergeIdx];
    pu.interDir               = affineMergeCtx.interDirNeighbours[mergeIdx];
    pu.BcwIdx                 = pu.interDir == 3 ? affineMergeCtx.BcwIdx[mergeIdx] : BCW_DEFAULT;

    // generate motion buf for IFP
    if( affineMergeCtx.mergeType[mergeIdx] == MRG_TYPE_SUBPU_ATMVP )
    {
      pu.refIdx[L0]           = affineMergeCtx.mvFieldNeighbours[mergeIdx][L0][0].refIdx;
      pu.refIdx[L1]           = affineMergeCtx.mvFieldNeighbours[mergeIdx][L1][0].refIdx;
      pu.mv    [L0][0]        = affineMergeCtx.mvFieldNeighbours[mergeIdx][L0][0].mv;
      pu.mv    [L1][0]        = affineMergeCtx.mvFieldNeighbours[mergeIdx][L1][0].mv;
      CU::spanMotionInfo      ( pu, &affineMergeCtx );
    }
    else
    {
      CU::setAllAffineMvField ( pu, affineMergeCtx.mvFieldNeighbours[mergeIdx][L0], L0 );
      CU::setAllAffineMvField ( pu, affineMergeCtx.mvFieldNeighbours[mergeIdx][L1], L1 );
      CU::spanMotionInfo      ( pu );
    }

    if( m_pcEncCfg->m_ifpLines && !CU::isMotionBufInRangeFPP( pu, m_pcEncCfg->m_ifpLines ) )
    {
      continue;
    }

    MergeItem *mergeItem   = m_mergeItemList.allocateNewMergeItem();
    mergeItem->importMergeInfo( affineMergeCtx, mergeIdx, affineMergeCtx.mergeType[mergeIdx] == MRG_TYPE_SUBPU_ATMVP ? MergeItem::MergeItemType::SBTMVP : MergeItem::MergeItemType::AFFINE, pu );
    auto dstBuf            = mergeItem->getPredBuf( localUnitArea );
    generateMergePrediction( localUnitArea, mergeItem, pu, true, false, dstBuf, false, false, nullptr, nullptr );
    mergeItem->cost        = calcLumaCost4MergePrediction( ctxStart, dstBuf, sqrtLambdaForFirstPass, pu, distParam );
    m_mergeItemList        . insertMergeItemToList( mergeItem );
  }
  if( m_pcEncCfg->m_useFastMrg >= 2 )
  {
    m_mergeItemList        . shrinkList( curListSize );
  }
}

void EncCu::addGpmCandsToPruningList( const MergeCtx &mergeCtx, const UnitArea &localUnitArea, double sqrtLambdaForFirstPass,
                                      const TempCtx& ctxStart, const GeoComboCostList& comboList, MergeBufVector& geoBuffer, DistParam& distParam, CodingUnit& pu)
{
  int geoNumMrgSadCand    = std::min( GEO_MAX_TRY_WEIGHTED_SAD, ( int ) comboList.list.size() );
  geoNumMrgSadCand        = std::min( geoNumMrgSadCand, m_pcEncCfg->m_Geo > 2 ? 10 : GEO_MAX_TRY_WEIGHTED_SAD );
  double bestGeoCost      = MAX_DOUBLE / 2.0;
  MergeItem* best2geo[2]  = { nullptr, nullptr };

  pu.mergeFlag = true;
  pu.geo       = true;
  pu.mergeType = MRG_TYPE_DEFAULT_N;
  pu.BcwIdx    = BCW_DEFAULT;
  pu.interDir  = 3;
  pu.imv       = 0;
  pu.affine    = pu.mmvdMergeFlag = pu.mmvdSkip
               = pu.ciip
               = false;

  for( int candidateIdx = 0; candidateIdx < geoNumMrgSadCand; candidateIdx++ )
  {
    const int          splitDir     = comboList.list[candidateIdx].splitDir;
    const MergeIdxPair mergeIdxPair { comboList.list[candidateIdx].mergeIdx0, comboList.list[candidateIdx].mergeIdx1 };
    const int          gpmIndex     = MergeItem::getGpmUnfiedIndex( splitDir, mergeIdxPair );

    pu.mergeIdx            = gpmIndex;
    pu.geoMergeIdx         = mergeIdxPair;
    pu.geoSplitDir         = splitDir;
    CU::spanGeoMotionInfo  ( pu, mergeCtx, pu.geoSplitDir, pu.geoMergeIdx[0], pu.geoMergeIdx[1] );

    MergeItem *mergeItem   = m_mergeItemList.allocateNewMergeItem();
    mergeItem->importMergeInfo( mergeCtx, gpmIndex, MergeItem::MergeItemType::GPM, pu );
    auto dstBuf            = mergeItem->getPredBuf( localUnitArea );
    generateMergePrediction( localUnitArea, mergeItem, pu, true, false, dstBuf, false, false, &geoBuffer[mergeIdxPair[0]], &geoBuffer[mergeIdxPair[1]] );
    mergeItem->cost        = calcLumaCost4MergePrediction( ctxStart, dstBuf, sqrtLambdaForFirstPass, pu, distParam );
    bestGeoCost            = std::min( mergeItem->cost, bestGeoCost );

    if( mergeItem->cost > MRG_FAST_RATIO[0] * bestGeoCost || mergeItem->cost > m_mergeBestSATDCost )
    {
      m_mergeItemList      . giveBackMergeItem( mergeItem );

      if( m_pcEncCfg->m_Geo > 2 ) break;
    }
    else if( m_pcEncCfg->m_Geo < 2 )
    {
      m_mergeItemList      . insertMergeItemToList( mergeItem );
    }
    else
    {
      if( ( m_mergeItemList.size() > 0 && m_mergeItemList.getMergeItemInList( m_mergeItemList.size() - 1 )->cost <= mergeItem->cost ) ||
        ( best2geo[1] && best2geo[1]->cost <= mergeItem->cost ) )
      {
        m_mergeItemList    . giveBackMergeItem( mergeItem );
      }
      else
      {
        if( !best2geo[0] || mergeItem->cost < best2geo[0]->cost )
        {
          if( best2geo[1] )
            m_mergeItemList. giveBackMergeItem( best2geo[1] );

          best2geo[1] = best2geo[0]; best2geo[0] = mergeItem;
        }
        else
        {
          if( best2geo[1] ) 
            m_mergeItemList. giveBackMergeItem( best2geo[1] );

          best2geo[1] = mergeItem;
        }
      }
    }
  }

  if( best2geo[0] )
    m_mergeItemList        . insertMergeItemToList( best2geo[0] );
  if( best2geo[1] )
    m_mergeItemList        . insertMergeItemToList( best2geo[1] );
}

bool EncCu::prepareGpmComboList( const MergeCtx &mergeCtx, const UnitArea &localUnitArea, double sqrtLambdaForFirstPass,
                                 GeoComboCostList& comboList, MergeBufVector& geoBuffer, CodingUnit& pu )
{
          sqrtLambdaForFirstPass /= FRAC_BITS_SCALE;
  const int bitsForPartitionIdx   = floorLog2(GEO_NUM_PARTITION_MODE);
  const int maxNumMergeCandidates = std::min( ( int ) pu.cs->sps->maxNumGeoCand, MRG_MAX_NUM_CANDS );
  DistParam distParam;
  // the second arguments to setDistParam is dummy and will be updated before being used
  DistParam  distParamWholeBlk     = m_cRdCost.setDistParam( pu.cs->getOrgBuf().Y(), pu.cs->getOrgBuf().Y(), pu.cs->sps->bitDepths[ CH_L ], DF_SAD );
  Distortion bestWholeBlkSad       = MAX_UINT64;
  double     bestWholeBlkCost      = MAX_DOUBLE;
  const ClpRng&  lclpRng           = pu.slice->clpRngs[COMP_Y];
  const unsigned rshift            = std::max<int>( 2, ( IF_INTERNAL_PREC - lclpRng.bd ) );
  const int      offset            = ( 1 << ( rshift - 1 ) ) + IF_INTERNAL_OFFS;
  const int      numSamples        = pu.Y().area();
  Distortion sadWholeBlk            [GEO_MAX_NUM_UNI_CANDS];
  int        pocMrg                 [GEO_MAX_NUM_UNI_CANDS];
  Mv         mergeMv                [GEO_MAX_NUM_UNI_CANDS];
  bool       isSkipThisCand         [GEO_MAX_NUM_UNI_CANDS]
                                   = { false, };
  bool       sameMV                 [MRG_MAX_NUM_CANDS]
                                   = { false, };
  MergeBufVector geoTempBuf;

  if( m_pcEncCfg->m_Geo > 2 )
  {
    for( int m = 0; m < maxNumMergeCandidates; m++ )
    {
      if( !sameMV[m] )
      {
        for( int n = m + 1; n < maxNumMergeCandidates; n++ )
        {
          sameMV[n] |= mergeCtx.mvFieldNeighbours[m][0] == mergeCtx.mvFieldNeighbours[n][0]
                    && mergeCtx.mvFieldNeighbours[m][1] == mergeCtx.mvFieldNeighbours[n][1];
        }
      }
    }
  }

  for( uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++ )
  {
    geoBuffer .push_back ( m_aTmpStorageLCU[2                         + mergeCand].getCompactBuf( localUnitArea ) );
    geoTempBuf.push_back ( m_aTmpStorageLCU[2 + GEO_MAX_NUM_UNI_CANDS + mergeCand].getCompactBuf( localUnitArea ) );

    const int  listIdx    = mergeCtx.mvFieldNeighbours[mergeCand][0]      .refIdx == -1 ? 1 : 0;
    const auto refPicList = RefPicList(listIdx);
    const int  refIdx     = mergeCtx.mvFieldNeighbours[mergeCand][listIdx].refIdx;

    pocMrg [mergeCand]    = pu.cs->slice->getRefPic( refPicList, refIdx )->poc;
    mergeMv[mergeCand]    = mergeCtx.mvFieldNeighbours[mergeCand][listIdx].mv;

    for( int i = 0; i < mergeCand; i++ )
    {
      if( pocMrg[mergeCand] == pocMrg[i] && mergeMv[mergeCand] == mergeMv[i] )
      {
        isSkipThisCand[mergeCand] = true;
        break;
      }
    }

    if( sameMV[mergeCand] )
    {
      continue;
    }

    if( m_pcEncCfg->m_ifpLines ) 
    {
      bool isOutOfRange  = !CU::isMvInRangeFPP( pu.ly(), pu.lheight(), mergeCtx.mvFieldNeighbours[mergeCand][0].mv.ver, m_pcEncCfg->m_ifpLines, *pu.cs->pcv );
           isOutOfRange |= !CU::isMvInRangeFPP( pu.ly(), pu.lheight(), mergeCtx.mvFieldNeighbours[mergeCand][1].mv.ver, m_pcEncCfg->m_ifpLines, *pu.cs->pcv );

      // use sameMV to surpress processing of this cand later on...
      sameMV[mergeCand] |= isOutOfRange;

      if( isOutOfRange )
        continue;
    }

    mergeCtx.setMergeInfo            ( pu, mergeCand );
    CU::spanMotionInfo               ( pu );
    m_cInterSearch.motionCompensation( pu, geoBuffer[mergeCand], REF_PIC_LIST_X );

    g_pelBufOP.roundGeo( geoBuffer[mergeCand].Y().buf, geoTempBuf[mergeCand].Y().buf, numSamples, rshift, offset, lclpRng );

    distParamWholeBlk.cur  = geoTempBuf[mergeCand].Y();
    sadWholeBlk[mergeCand] = distParamWholeBlk.distFunc( distParamWholeBlk );

    if( sadWholeBlk[mergeCand] < bestWholeBlkSad )
    {
      bestWholeBlkSad  = sadWholeBlk[mergeCand];
      int bitsCand     = mergeCand + 1;
      bestWholeBlkCost = ( double ) bestWholeBlkSad + ( double ) bitsCand * sqrtLambdaForFirstPass;
    }
  }

  bool allCandsAreSame = true;
  for( uint8_t mergeCand = 1; mergeCand < maxNumMergeCandidates; mergeCand++ )
  {
    allCandsAreSame &= isSkipThisCand[mergeCand];
  }
  if( allCandsAreSame )
  {
    return false;
  }

  const int wIdx = floorLog2( pu.lwidth() )  - GEO_MIN_CU_LOG2;
  const int hIdx = floorLog2( pu.lheight() ) - GEO_MIN_CU_LOG2;

  for( int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; )
  {
    int maskStride = 0, maskStride2 = 0;
    int stepX = 1;
    Pel *sadMask;
    int16_t angle = g_GeoParams[splitDir][0];
    
    if( g_angle2mirror[angle] == 2 )
    {
      maskStride  = -GEO_WEIGHT_MASK_SIZE;
      maskStride2 = -( int ) pu.lwidth();
      sadMask     = &g_globalGeoEncSADmask[g_angle2mask[g_GeoParams[splitDir][0]]]
                      [( GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[hIdx][wIdx][splitDir][1] ) * GEO_WEIGHT_MASK_SIZE
                                                  + g_weightOffset[hIdx][wIdx][splitDir][0]
                      ];
    }
    else if( g_angle2mirror[angle] == 1 )
    {
      stepX       = -1;
      maskStride2 = pu.lwidth();
      maskStride  = GEO_WEIGHT_MASK_SIZE;
      sadMask     = &g_globalGeoEncSADmask[g_angle2mask[g_GeoParams[splitDir][0]]]
                      [     GEO_WEIGHT_MASK_SIZE *     g_weightOffset[hIdx][wIdx][splitDir][1]
                        + ( GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[hIdx][wIdx][splitDir][0] )
                      ];
    }
    else
    {
      maskStride  = GEO_WEIGHT_MASK_SIZE;
      maskStride2 = -( int ) pu.lwidth();
      sadMask     = &g_globalGeoEncSADmask[g_angle2mask[g_GeoParams[splitDir][0]]]
                      [   g_weightOffset[hIdx][wIdx][splitDir][1] * GEO_WEIGHT_MASK_SIZE
                        + g_weightOffset[hIdx][wIdx][splitDir][0]
                      ];
    }

    m_cRdCost.setDistParamGeo ( distParam, pu.cs->getOrgBuf().Y(),
                                nullptr, 0,
                                sadMask, maskStride, stepX, maskStride2,
                                pu.cs->sps->bitDepths[CH_L], COMP_Y );

    for( uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++ )
    {
      if( sameMV[mergeCand] )
      {
        continue;
      }

      distParam.cur.buf         = geoTempBuf[mergeCand].Y().buf;
      distParam.cur.stride      = geoTempBuf[mergeCand].Y().stride;
      const Distortion sadLarge = distParam.distFunc( distParam );
      const Distortion sadSmall = sadWholeBlk[mergeCand] - sadLarge;

      const int bitsCand        = mergeCand + 1;

      const double cost0        = ( double ) sadLarge + ( double ) bitsCand * sqrtLambdaForFirstPass;
      const double cost1        = ( double ) sadSmall + ( double ) bitsCand * sqrtLambdaForFirstPass;

      m_GeoCostList.insert( splitDir, 0, mergeCand, cost0 );
      m_GeoCostList.insert( splitDir, 1, mergeCand, cost1 );
    }

    if( m_pcEncCfg->m_Geo == 4 )
    {
      if( splitDir == 1 )
      {
        splitDir += 7;
      }
      else if( splitDir == 35 || ( splitDir + 1 ) % 4 != 0 )
      {
        splitDir++;
      }
      else
      {
        splitDir += 5;
      }
    }
    else
    {
      splitDir++;
    }
  }

  comboList.list.clear();

  for( int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; )
  {
    for( int geoMotionIdx = 0; geoMotionIdx < maxNumMergeCandidates * ( maxNumMergeCandidates - 1 ); geoMotionIdx++ )
    {
      const MergeIdxPair mergeIdxPair = m_GeoModeTest[geoMotionIdx];

      if( sameMV[mergeIdxPair[0]] || sameMV[mergeIdxPair[1]] )
      {
        continue;
      }

      double tempCost = m_GeoCostList.getCost( splitDir, mergeIdxPair[0], mergeIdxPair[1] );

      if( tempCost > bestWholeBlkCost )
      {
        continue;
      }

      tempCost = tempCost + ( double ) bitsForPartitionIdx * sqrtLambdaForFirstPass;
      comboList.list.push_back( GeoMergeCombo{ splitDir, mergeIdxPair[0], mergeIdxPair[1], tempCost } );
    }

    if( m_pcEncCfg->m_Geo == 4 )
    {
      if( splitDir == 1 )
      {
        splitDir += 7;
      }
      else if( splitDir == 35 || ( splitDir + 1 ) % 4 != 0 )
      {
        splitDir++;
      }
      else
      {
        splitDir += 5;
      }
    }
    else
    {
      splitDir++;
    }
  }

  if( comboList.list.empty() )
  {
    return false;
  }

  comboList.sortByCost();
  return true;
}

double EncCu::calcLumaCost4MergePrediction( const TempCtx &ctxStart, const PelUnitBuf &predBuf, double lambda, CodingUnit &cu, DistParam &distParam )
{
  distParam.cur = predBuf.Y();
  auto dist     = distParam.distFunc(distParam);

  m_CABACEstimator->getCtx() = ctxStart;
  auto fracBits = xCalcPuMeBits( cu );

  double cost   = ( double ) dist + ( double ) fracBits * lambda;

  m_uiSadBestForQPA = std::min( dist, m_uiSadBestForQPA );

  return cost;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// ibc merge/skip mode check
void EncCu::xCheckRDCostIBCModeMerge2Nx2N(CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& partitioner, const EncTestMode& encTestMode)
{
  assert(partitioner.chType != CH_C); // chroma IBC is derived
  if (tempCS->area.lwidth() == 128 || tempCS->area.lheight() == 128) // disable IBC mode larger than 64x64
  {
    return;
  }

  if ((m_pcEncCfg->m_IBCFastMethod > 1) && !bestCS->slice->isIntra() && (bestCS->cus.size() != 0))
  {
    if (bestCS->getCU(partitioner.chType, partitioner.treeType)->skip)
    {
      return;
    }
  }

  const SPS& sps = *tempCS->sps;

  tempCS->initStructData(encTestMode.qp);
  MergeCtx mergeCtx;

  {
    // first get merge candidates
    CodingUnit cu(tempCS->area);
    cu.cs = tempCS;
    cu.predMode = MODE_IBC;
    cu.slice = tempCS->slice;
    cu.tileIdx = m_tileIdx;
    cu.initPuData();
    cu.cs = tempCS;
    cu.mmvdSkip = false;
    cu.mmvdMergeFlag = false;
    cu.geo = false;
    CU::getIBCMergeCandidates(cu, mergeCtx);
  }
  int candHasNoResidual[MRG_MAX_NUM_CANDS];
  for (unsigned int ui = 0; ui < mergeCtx.numValidMergeCand; ui++)
  {
    candHasNoResidual[ui] = 0;
  }

  bool                                        bestIsSkip = false;
  unsigned                                    numMrgSATDCand = mergeCtx.numValidMergeCand;
  static_vector<unsigned, MRG_MAX_NUM_CANDS>  RdModeList(MRG_MAX_NUM_CANDS);
  for (unsigned i = 0; i < MRG_MAX_NUM_CANDS; i++)
  {
    RdModeList[i] = i;
  }

  //{
  static_vector<double, MRG_MAX_NUM_CANDS>  candCostList(MRG_MAX_NUM_CANDS, MAX_DOUBLE);
  // 1. Pass: get SATD-cost for selected candidates and reduce their count
  {
    const double sqrtLambdaForFirstPass = m_cRdCost.getMotionLambda();

    CodingUnit& cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType,partitioner.treeType), partitioner.chType);

    partitioner.setCUData(cu);
    cu.slice = tempCS->slice;
    cu.tileIdx = m_tileIdx;
    cu.skip = false;
    cu.predMode = MODE_IBC;
    cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
    cu.qp = encTestMode.qp;
    cu.mmvdSkip = false;
    cu.geo = false;
    DistParam distParam;
    cu.initPuData();
    cu.mmvdMergeFlag = false;
    Picture* refPic = cu.slice->pic;
    const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(cu.blocks[COMP_Y].x, cu.blocks[COMP_Y].y, tempCS->area.Y().width, tempCS->area.Y().height));
    const CompArea& compArea = localUnitArea.block(COMP_Y);
    const CPelBuf refBuf = refPic->getRecoBuf(compArea);
    const Pel* piRefSrch = refBuf.buf;
    const ReshapeData& reshapeData = cu.cs->picture->reshapeData;
    if (cu.cs->slice->lmcsEnabled && reshapeData.getCTUFlag())
    {
      PelBuf tmpLmcs = m_aTmpStorageLCU[0].getCompactBuf(cu.Y());
      tmpLmcs.rspSignal(tempCS->getOrgBuf().Y(), reshapeData.getFwdLUT());
      distParam = m_cRdCost.setDistParam( tmpLmcs, refBuf, sps.bitDepths[CH_L], DF_HAD);
    }
    else
    {
      distParam = m_cRdCost.setDistParam(tempCS->getOrgBuf(COMP_Y), refBuf, sps.bitDepths[CH_L], DF_HAD);
    }
    int refStride = refBuf.stride;

    int numValidBv = mergeCtx.numValidMergeCand;
    for (unsigned int mergeCand = 0; mergeCand < mergeCtx.numValidMergeCand; mergeCand++)
    {
      mergeCtx.setMergeInfo(cu, mergeCand); // set bv info in merge mode
      const int cuPelX = cu.Y().x;
      const int cuPelY = cu.Y().y;
      int roiWidth     = cu.lwidth();
      int roiHeight    = cu.lheight();
      const int picWidth  = cu.cs->slice->pps->picWidthInLumaSamples;
      const int picHeight = cu.cs->slice->pps->picHeightInLumaSamples;
      const unsigned int lcuWidth = cu.cs->slice->sps->CTUSize;

      Mv bv = cu.mv[0][0];
      bv.changePrecision( MV_PRECISION_INTERNAL, MV_PRECISION_INT);
      int xPred = bv.hor;
      int yPred = bv.ver;
      
      if( !m_cInterSearch.searchBvIBC( cu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth ) ) // not valid bv derived
      {
        numValidBv--;
        continue;
      }
      CU::spanMotionInfo(cu);
      distParam.cur.buf = piRefSrch + refStride * yPred + xPred;

      Distortion sad = distParam.distFunc(distParam);
      unsigned int bitsCand = mergeCand + 1;
      if (mergeCand == tempCS->sps->maxNumIBCMergeCand - 1)
      {
        bitsCand--;
      }
      double cost = (double)sad + (double)bitsCand * sqrtLambdaForFirstPass;

      updateCandList( mergeCand, cost, RdModeList, candCostList, numMrgSATDCand );
    }

    // Try to limit number of candidates using SATD-costs
    if (numValidBv)
    {
      numMrgSATDCand = numValidBv;
      for (unsigned int i = 1; i < numValidBv; i++)
      {
        if (candCostList[i] > MRG_FAST_RATIO[0] * candCostList[0])
        {
          numMrgSATDCand = i;
          break;
        }
      }
    }
    else
    {
      tempCS->dist = 0;
      tempCS->fracBits = 0;
      tempCS->cost = MAX_DOUBLE;
      tempCS->costDbOffset = 0;
      tempCS->initStructData(encTestMode.qp);
      return;
    }

    tempCS->initStructData(encTestMode.qp);
  }
  //}


  const unsigned int iteration = 2;
 // m_bestModeUpdated = tempCS->cost = bestCS->cost = false;
  // 2. Pass: check candidates using full RD test
  for (unsigned int numResidualPass = 0; numResidualPass < iteration; numResidualPass++)
  {
    for (unsigned int mrgHADIdx = 0; mrgHADIdx < numMrgSATDCand; mrgHADIdx++)
    {
      unsigned int mergeCand = RdModeList[mrgHADIdx];
      if (!(numResidualPass == 1 && candHasNoResidual[mergeCand] == 1))
      {
        if (!(bestIsSkip && (numResidualPass == 0)))
        {
          {

            // first get merge candidates
            CodingUnit& cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType)partitioner.chType,partitioner.treeType), (const ChannelType)partitioner.chType);

            partitioner.setCUData(cu);
            cu.slice = tempCS->slice;
            cu.tileIdx = m_tileIdx;
            cu.skip = false;
            cu.predMode = MODE_IBC;
            cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
            cu.qp = encTestMode.qp;
            cu.sbtInfo = 0;
            cu.initPuData();
            cu.intraDir[0] = DC_IDX; // set intra pred for ibc block
            cu.intraDir[1] = PLANAR_IDX; // set intra pred for ibc block
            cu.mmvdSkip = false;
            cu.mmvdMergeFlag = false;
            cu.geo = false;
            mergeCtx.setMergeInfo(cu, mergeCand);
            CU::spanMotionInfo(cu);

            assert(mergeCtx.mrgTypeNeighbours[mergeCand] == MRG_TYPE_IBC);
            const bool chroma = !CU::isSepTree(cu);

            //  MC
            cu.mcControl = chroma ? 0: 2;
            m_cInterSearch.motionCompensationIBC(cu, tempCS->getPredBuf());
            m_CABACEstimator->getCtx() = m_CurrCtx->start;

            m_cInterSearch.encodeResAndCalcRdInterCU(*tempCS, partitioner, (numResidualPass != 0));
            cu.mcControl = 0;
            xEncodeDontSplit(*tempCS, partitioner);
            xCheckDQP(*tempCS, partitioner);
            xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);

            tempCS->initStructData(encTestMode.qp);
          }

          if (m_pcEncCfg->m_useFastDecisionForMerge && !bestIsSkip)
          {
            if (bestCS->getCU(partitioner.chType, partitioner.treeType) == NULL)
              bestIsSkip = 0;
            else
              bestIsSkip = bestCS->getCU(partitioner.chType, partitioner.treeType)->rootCbf == 0;
          }
        }
      }
    }
  }
}

void EncCu::xCheckRDCostIBCMode(CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& partitioner,
  const EncTestMode& encTestMode)
{
  if (tempCS->area.lwidth() == 128 || tempCS->area.lheight() == 128)   // disable IBC mode larger than 64x64
  {
    return;
  }
  if ((m_pcEncCfg->m_IBCFastMethod > 1) && !bestCS->slice->isIntra() && (bestCS->cus.size() != 0))
  {
    if (bestCS->getCU(partitioner.chType, partitioner.treeType)->skip)
    {
      return;
    }
  }

  tempCS->initStructData(encTestMode.qp);

  CodingUnit& cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType, partitioner.treeType), partitioner.chType);

  partitioner.setCUData(cu);
  cu.slice = tempCS->slice;
  cu.tileIdx = m_tileIdx;
  cu.skip = false;
  cu.predMode = MODE_IBC;
  cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
  cu.qp = encTestMode.qp;
  cu.initPuData();
  cu.imv = 0;
  cu.sbtInfo = 0;
  cu.mmvdSkip = false;
  cu.mmvdMergeFlag = false;

  cu.intraDir[0] = DC_IDX; // set intra pred for ibc block
  cu.intraDir[1] = PLANAR_IDX; // set intra pred for ibc block

  cu.interDir = 1; // use list 0 for IBC mode
  cu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF; // last idx in the list
  bool bValid = m_cInterSearch.predIBCSearch(cu, partitioner);

  if (bValid)
  {
    CU::spanMotionInfo(cu);
    const bool chroma = !CU::isSepTree(cu);
    //  MC
    cu.mcControl = chroma ? 0 : 2;
    m_cInterSearch.motionCompensationIBC(cu, tempCS->getPredBuf());

    m_cInterSearch.encodeResAndCalcRdInterCU(*tempCS, partitioner, false);
    cu.mcControl = 0;

    xEncodeDontSplit(*tempCS, partitioner);
    xCheckDQP(*tempCS, partitioner);
    xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
  } // bValid
  else
  {
    tempCS->dist = 0;
    tempCS->fracBits = 0;
    tempCS->cost = MAX_DOUBLE;
    tempCS->costDbOffset = 0;
  }
}

void EncCu::xCheckRDCostInter( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, _TPROF, P_INTER_MVD, tempCS, partitioner.chType );
  tempCS->initStructData( encTestMode.qp );

  m_cInterSearch.setAffineModeSelected( false );

  m_cInterSearch.resetBufferedUniMotions();

  int bcwLoopNum = BCW_NUM;

  if( tempCS->area.Y().area() < BCW_SIZE_CONSTRAINT || !tempCS->slice->isInterB() || !tempCS->sps->BCW )
  {
    bcwLoopNum = 1;
  }
  
  double curBestCost = bestCS->cost;
  double equBcwCost = MAX_DOUBLE;

  for( int bcwLoopIdx = 0; bcwLoopIdx < bcwLoopNum; bcwLoopIdx++ )
  {
    if( m_pcEncCfg->m_BCW == 2 )
    {
      bool isBestInter   = m_modeCtrl.getBlkInfo( bestCS->area ).isInter;
      uint8_t bestBcwIdx = m_modeCtrl.getBlkInfo( bestCS->area).BcwIdx;

      if( isBestInter && g_BcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT && g_BcwSearchOrder[bcwLoopIdx] != bestBcwIdx )
      {
        continue;
      }
    }
    
    if( !tempCS->slice->checkLDC )
    {
      if( bcwLoopIdx != 0 && bcwLoopIdx != 3 && bcwLoopIdx != 4 )
      {
        continue;
      }
    }
  
    CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

    partitioner.setCUData( cu );
    cu.slice            = tempCS->slice;
    cu.tileIdx          = m_tileIdx;
    cu.skip             = false;
    cu.mmvdSkip         = false;
    cu.predMode         = MODE_INTER;
    cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
    cu.qp               = encTestMode.qp;
    cu.initPuData();

    cu.BcwIdx = g_BcwSearchOrder[bcwLoopIdx];
    uint8_t bcwIdx = cu.BcwIdx;
    bool testBcw = (bcwIdx != BCW_DEFAULT);

    bool StopInterRes = (m_pcEncCfg->m_FastInferMerge >> 3) & 1;
    StopInterRes &= bestCS->slice->TLayer > (m_pcEncCfg->m_maxTLayer - (m_pcEncCfg->m_FastInferMerge & 7));
    double bestCostInter = StopInterRes ? m_mergeBestSATDCost : MAX_DOUBLE;

    bool stopTest = m_cInterSearch.predInterSearch(cu, partitioner, bestCostInter);

    if (StopInterRes && (bestCostInter != m_mergeBestSATDCost))
    {
      int L = (cu.slice->TLayer <= 2) ? 0 : (cu.slice->TLayer - 2);
      if ((bestCostInter > MRG_FAST_RATIOMYV[L] * m_mergeBestSATDCost))
      {
        stopTest = true;
      }
    }

    if( !stopTest )
    {
      bcwIdx   = CU::getValidBcwIdx(cu);
      stopTest = testBcw && bcwIdx == BCW_DEFAULT;
    }
    
    if( stopTest )
    {
      tempCS->initStructData(encTestMode.qp);
      continue;
    }

    CHECK(!(testBcw || (!testBcw && bcwIdx == BCW_DEFAULT)), " !( bTestBcw || (!bTestBcw && bcwIdx == BCW_DEFAULT ) )");
        
    xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0, 0, &equBcwCost);
    
    if( bcwIdx == BCW_DEFAULT )
    {
      m_cInterSearch.setAffineModeSelected( bestCS->cus.front()->affine && !bestCS->cus.front()->mergeFlag );
    }

    tempCS->initStructData(encTestMode.qp);
  
    double skipTH = MAX_DOUBLE;
    skipTH = (m_pcEncCfg->m_BCW == 2 ? 1.05 : MAX_DOUBLE);
    if( equBcwCost > curBestCost * skipTH )
    {
      break;
    }

    if( m_pcEncCfg->m_BCW == 2 )
    {
      if( ( cu.interDir != 3 && testBcw == 0 && ! m_pcEncCfg->m_picReordering )
         || ( g_BcwSearchOrder[bcwLoopIdx] == BCW_DEFAULT && xIsBcwSkip( cu ) ) )
      {
        break;
      }
    }
  }
  STAT_COUNT_CU_MODES( partitioner.chType == CH_L, g_cuCounters1D[CU_MODES_TESTED][0][!tempCS->slice->isIntra() + tempCS->slice->depth] );
  STAT_COUNT_CU_MODES( partitioner.chType == CH_L && !tempCS->slice->isIntra(), g_cuCounters2D[CU_MODES_TESTED][Log2( tempCS->area.lheight() )][Log2( tempCS->area.lwidth() )] );
}

void EncCu::xCheckRDCostInterIMV(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, _TPROF, P_INTER_MVD_IMV, tempCS, partitioner.chType );
  bool Test_AMVR = m_pcEncCfg->m_AMVRspeed ? true: false;
  if (m_pcEncCfg->m_AMVRspeed > 2 && m_pcEncCfg->m_AMVRspeed < 5 && !bestCS->cus.empty() && bestCS->getCU(partitioner.chType, partitioner.treeType)->skip)
  {
    Test_AMVR = false;
  }
  else if (m_pcEncCfg->m_AMVRspeed > 4 && !bestCS->cus.empty() && bestCS->getCU(partitioner.chType, partitioner.treeType)->mergeFlag && !bestCS->getCU(partitioner.chType, partitioner.treeType)->ciip)
  {
    Test_AMVR = false;
  }
  bool Do_Limit = !bestCS->cus.empty() && (m_pcEncCfg->m_AMVRspeed == 4 || m_pcEncCfg->m_AMVRspeed == 6) ? true : false;
  bool Do_OnceRes = !bestCS->cus.empty() && (m_pcEncCfg->m_AMVRspeed == 7) ? true : false;

  if( Test_AMVR )
  {
    double Fpel_cost    = m_pcEncCfg->m_AMVRspeed == 1 ? MAX_DOUBLE*0.5 : MAX_DOUBLE;
    double costCurStart = m_pcEncCfg->m_AMVRspeed == 1 ? m_modeCtrl.comprCUCtx->bestCostNoImv : bestCS->cost;
    double costCur      = MAX_DOUBLE;
    double bestCostIMV  = MAX_DOUBLE;

    if (Do_OnceRes)
    {
      costCurStart = xCalcDistortion(bestCS, partitioner.chType, bestCS->sps->bitDepths[CH_L], 0);
      Fpel_cost = costCurStart;
      tempCS->initSubStructure(*m_pTempCS2, partitioner.chType, partitioner.currArea(), false);
    }

    CodingStructure *tempCSbest = m_pTempCS2;

    m_cInterSearch.setAffineModeSelected( false );

    m_cInterSearch.resetBufferedUniMotions();

    int bcwLoopNum = (tempCS->slice->isInterB() ? BCW_NUM : 1);
    bcwLoopNum = (tempCS->sps->BCW ? bcwLoopNum : 1);

    if( tempCS->area.lwidth() * tempCS->area.lheight() < BCW_SIZE_CONSTRAINT )
    {
      bcwLoopNum = 1;
    }

    for (int i = 1; i <= IMV_HPEL; i++)
    {
      double curBestCost = bestCS->cost;
      double equBcwCost  = MAX_DOUBLE;

      for( int bcwLoopIdx = 0; bcwLoopIdx < bcwLoopNum; bcwLoopIdx++ )
      {
        if( m_pcEncCfg->m_BCW == 2 )
        {
          bool isBestInter   = m_modeCtrl.getBlkInfo( bestCS->area ).isInter;
          uint8_t bestBcwIdx = m_modeCtrl.getBlkInfo( bestCS->area).BcwIdx;

          if( isBestInter && g_BcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT && g_BcwSearchOrder[bcwLoopIdx] != bestBcwIdx )
          {
            continue;
          }
          
          if( tempCS->slice->checkLDC && g_BcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT
            && (m_bestBcwIdx[0] >= 0 && g_BcwSearchOrder[bcwLoopIdx] != m_bestBcwIdx[0])
            && (m_bestBcwIdx[1] >= 0 && g_BcwSearchOrder[bcwLoopIdx] != m_bestBcwIdx[1]))
          {
            continue;
          }
        }

        if( !tempCS->slice->checkLDC )
        {
          if( bcwLoopIdx != 0 && bcwLoopIdx != 3 && bcwLoopIdx != 4 )
          {
            continue;
          }
        }

        bool testBcw;
        uint8_t bcwIdx;
        bool isEqualUni = false;

        if (i > IMV_FPEL)
        {
          bool nextimv = false;
          double stopCost = i == IMV_HPEL ? 1.25 : 1.06;
          if (Fpel_cost > stopCost * costCurStart)
          {
            nextimv = true;
          }
          if ( m_pcEncCfg->m_AMVRspeed == 1 )
          {
            costCurStart = bestCS->cost;
          }
          if (nextimv)
          {
            continue;
          }
        }

        bool Do_Search = Do_OnceRes ? false : true;

        if (Do_Limit)
        {
          Do_Search = i == IMV_FPEL ? true : false;

          if (i == IMV_HPEL)
          {
            if (bestCS->slice->TLayer > 3)
            {
              continue;
            }
            if (bestCS->getCU(partitioner.chType, partitioner.treeType)->imv != 0)
            {
              Do_Search = true; //do_est
            }
          }
          if (bestCS->getCU(partitioner.chType, partitioner.treeType)->mmvdMergeFlag || bestCS->getCU(partitioner.chType, partitioner.treeType)->geo)
          {
            Do_Search = true;
          }
        }
        tempCS->initStructData(encTestMode.qp);

        if (!Do_Search)
        {
          tempCS->copyStructure(*bestCS, partitioner.chType, TREE_D);
        }
        tempCS->dist = 0;
        tempCS->fracBits = 0;
        tempCS->cost = MAX_DOUBLE;
        CodingUnit &cu = (Do_Search) ? tempCS->addCU(tempCS->area, partitioner.chType) : *tempCS->getCU(partitioner.chType, partitioner.treeType);
        if (Do_Search)
        {
          partitioner.setCUData(cu);
          cu.slice = tempCS->slice;
          cu.tileIdx = m_tileIdx;
          cu.skip = false;
          cu.mmvdSkip = false;
          cu.predMode = MODE_INTER;
          cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
          cu.qp = encTestMode.qp;

          cu.initPuData();

          cu.imv = i;

          cu.BcwIdx = g_BcwSearchOrder[bcwLoopIdx];
          bcwIdx    = cu.BcwIdx;
          testBcw   = (bcwIdx != BCW_DEFAULT);

          cu.interDir = 10;
          
          double bestCostInter = MAX_DOUBLE;
          m_cInterSearch.predInterSearch(cu, partitioner, bestCostInter);
          
          if ( cu.interDir <= 3 )
          {
            bcwIdx = CU::getValidBcwIdx(cu);
          }
          else
          {
            continue;
          }
          
          if( testBcw && bcwIdx == BCW_DEFAULT ) // Enabled Bcw but the search results is uni.
          {
            continue;
          }
          CHECK(!(testBcw || (!testBcw && bcwIdx == BCW_DEFAULT)), " !( bTestBcw || (!bTestBcw && bcwIdx == BCW_DEFAULT ) )");

          if( m_pcEncCfg->m_BCW == 2 )
          {
            if( cu.interDir != 3 && testBcw == 0 )
            {
              isEqualUni = true;
            }
          }

          if (!CU::hasSubCUNonZeroMVd(cu))
          {
            continue;
          }
        }
        else
        {
          cu.smvdMode = 0;
          cu.affine = false;
          cu.imv = i ;
          CU::resetMVDandMV2Int(cu);
          if (!CU::hasSubCUNonZeroMVd(cu))
          {
            continue;
          }

          cu.BcwIdx = g_BcwSearchOrder[bcwLoopIdx];

          cu.mvRefine = true;
          m_cInterSearch.motionCompensation(cu, tempCS->getPredBuf() );
          cu.mvRefine = false;
        }

        if( Do_OnceRes )
        {
          costCur = xCalcDistortion(tempCS, partitioner.chType, tempCS->sps->bitDepths[CH_L], cu.imv );
          if (costCur < bestCostIMV)
          {
            bestCostIMV = costCur;
            tempCSbest->getPredBuf().copyFrom(tempCS->getPredBuf());
            tempCSbest->clearCUs();
            tempCSbest->clearTUs();
            tempCSbest->copyStructure(*tempCS, partitioner.chType, TREE_D);
          }
          if (i > IMV_FPEL)
          {
            costCurStart = costCurStart > costCur ? costCur : costCurStart;
          }
        }
        else
        {
          xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0, 0, &equBcwCost);
          costCur = tempCS->cost;

          if (i > IMV_FPEL)
          {
            costCurStart = bestCS->cost;
          }
        }

        if (i == IMV_FPEL)
        {
           Fpel_cost = costCur;
        }

        double skipTH = MAX_DOUBLE;
        skipTH = (m_pcEncCfg->m_BCW == 2 ? 1.05 : MAX_DOUBLE);
        if( equBcwCost > curBestCost * skipTH )
        {
          break;
        }

        if( m_pcEncCfg->m_BCW == 2 )
        {
          if( isEqualUni == true && ! m_pcEncCfg->m_picReordering )
          {
            break;
          }
          if( g_BcwSearchOrder[bcwLoopIdx] == BCW_DEFAULT && xIsBcwSkip( cu ) )
          {
            break;
          }
        }
      }
    }

    if (Do_OnceRes && (bestCostIMV != MAX_DOUBLE))
    {
      CodingStructure* CSCandBest = tempCSbest;
      tempCS->initStructData(bestCS->currQP[partitioner.chType]);
      tempCS->copyStructure(*CSCandBest, partitioner.chType, TREE_D);
      tempCS->getPredBuf().copyFrom(tempCSbest->getPredBuf());
      tempCS->dist = 0;
      tempCS->fracBits = 0;
      tempCS->cost = MAX_DOUBLE;

      xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0, 0, NULL);
    }

    tempCS->initStructData(encTestMode.qp);
  }
  STAT_COUNT_CU_MODES( partitioner.chType == CH_L, g_cuCounters1D[CU_MODES_TESTED][0][!tempCS->slice->isIntra() + tempCS->slice->depth] );
  STAT_COUNT_CU_MODES( partitioner.chType == CH_L && !tempCS->slice->isIntra(), g_cuCounters2D[CU_MODES_TESTED][Log2( tempCS->area.lheight() )][Log2( tempCS->area.lwidth() )] );
}

void EncCu::xCalDebCost( CodingStructure &cs, Partitioner &partitioner )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, _TPROF, P_DEBLOCK_FILTER, &cs, partitioner.chType );
  if ( cs.slice->deblockingFilterDisable )
  {
    return;
  }

  const ChromaFormat format = cs.area.chromaFormat;
  CodingUnit*            cu = cs.getCU(partitioner.chType, partitioner.treeType);
  const Position    lumaPos = cu->Y().valid() ? cu->Y().pos() : recalcPosition( format, cu->chType, CH_L, cu->blocks[cu->chType].pos() );
  bool    topEdgeAvai = lumaPos.y > 0 && ((lumaPos.y % 4) == 0);
  bool   leftEdgeAvai = lumaPos.x > 0 && ((lumaPos.x % 4) == 0);

  if( ! ( topEdgeAvai || leftEdgeAvai ))
  {
    return;
  }

  ComponentID compStr = ( CU::isSepTree(*cu) && !isLuma( partitioner.chType ) ) ? COMP_Cb : COMP_Y;
  ComponentID compEnd = (( CU::isSepTree(*cu) && isLuma( partitioner.chType )) || cu->chromaFormat == VVENC_CHROMA_400 ) ? COMP_Y : COMP_Cr;
  const UnitArea currCsArea = clipArea( CS::getArea( cs, cs.area, partitioner.chType, partitioner.treeType ), *cs.picture );

  PelStorage&  picDbBuf = m_dbBuffer; //th we could reduce the buffer size and do some relocate

  //deblock neighbour pixels
  const Size     lumaSize = cu->Y().valid() ? cu->Y().size() : recalcSize( format, cu->chType, CH_L, cu->blocks[cu->chType].size() );

  int verOffset = lumaPos.y > 7 ? 8 : 4;
  int horOffset = lumaPos.x > 7 ? 8 : 4;

  LoopFilter::calcFilterStrengths( *cu, true );

  if( m_EDO == 2 && CS::isDualITree( cs ) && isLuma( partitioner.chType ) )
  {
    m_cLoopFilter.getMaxFilterLength( *cu, verOffset, horOffset );

    if( 0== (verOffset + horOffset) )
    {
      return;
    }

    topEdgeAvai  &= verOffset != 0;
    leftEdgeAvai &= horOffset != 0;
  }

  const UnitArea  areaTop  = UnitArea( format, Area( lumaPos.x,             lumaPos.y - verOffset, lumaSize.width, verOffset       ) );
  const UnitArea  areaLeft = UnitArea( format, Area( lumaPos.x - horOffset, lumaPos.y,             horOffset,      lumaSize.height ) );

  for ( int compIdx = compStr; compIdx <= compEnd; compIdx++ )
  {
    ComponentID compId = (ComponentID)compIdx;

    //Copy current CU's reco to Deblock Pic Buffer
    const ReshapeData& reshapeData = cs.picture->reshapeData;
    const CompArea&  compArea = currCsArea.block( compId );
    CompArea         locArea  = compArea;
    locArea.x -= cu->blocks[compIdx].x;
    locArea.y -= cu->blocks[compIdx].y;
    PelBuf dbReco = picDbBuf.getBuf( locArea );
    if (cs.slice->lmcsEnabled && isLuma(compId) )
    {
      if ((!cs.sps->LFNST) && (!cs.sps->MTS) && (!cs.sps->ISP)&& reshapeData.getCTUFlag())
      {
        PelBuf rspReco = cs.getRspRecoBuf();
        dbReco.copyFrom( rspReco );
      }
      else
      {
        PelBuf reco = cs.getRecoBuf( compId );
        dbReco.rspSignal( reco, reshapeData.getInvLUT() );
      }
    }
    else
    {
      PelBuf reco = cs.getRecoBuf( compId );
      dbReco.copyFrom( reco );
    }
    //left neighbour
    if ( leftEdgeAvai )
    {
      const CompArea&  compArea = areaLeft.block(compId);
      CompArea         locArea = compArea;
      locArea.x -= cu->blocks[compIdx].x;
      locArea.y -= cu->blocks[compIdx].y;
      PelBuf dbReco = picDbBuf.getBuf( locArea );
      if (cs.slice->lmcsEnabled && isLuma(compId))
      {
        dbReco.rspSignal( cs.picture->getRecoBuf( compArea ), reshapeData.getInvLUT() );
      }
      else
      {
        dbReco.copyFrom( cs.picture->getRecoBuf( compArea ) );
      }
    }
    //top neighbour
    if ( topEdgeAvai )
    {
      const CompArea&  compArea = areaTop.block( compId );
      CompArea         locArea = compArea;
      locArea.x -= cu->blocks[compIdx].x;
      locArea.y -= cu->blocks[compIdx].y;
      PelBuf dbReco = picDbBuf.getBuf( locArea );
      if (cs.slice->lmcsEnabled && isLuma(compId))
      {
        dbReco.rspSignal( cs.picture->getRecoBuf( compArea ), reshapeData.getInvLUT() );
      }
      else
      {
        dbReco.copyFrom( cs.picture->getRecoBuf( compArea ) );
      }
    }
  }

  ChannelType dbChType = CU::isSepTree(*cu) ? partitioner.chType : MAX_NUM_CH;

  CHECK( CU::isSepTree(*cu) && !cu->Y().valid() && partitioner.chType == CH_L, "xxx" );

  if( cu->Y() .valid() ) m_cLoopFilter.setOrigin( CH_L, cu->lumaPos() );
  if( cu->chromaFormat != VVENC_CHROMA_400 && cu->Cb().valid() ) m_cLoopFilter.setOrigin( CH_C, cu->chromaPos() );

  //deblock
  if( leftEdgeAvai )
  {
    m_cLoopFilter.loopFilterCu( *cu, dbChType, EDGE_VER, m_dbBuffer );
  }

  if( topEdgeAvai )
  {
    m_cLoopFilter.loopFilterCu( *cu, dbChType, EDGE_HOR, m_dbBuffer );
  }

  //calculate difference between DB_before_SSE and DB_after_SSE for neighbouring CUs
  Distortion distBeforeDb = 0, distAfterDb = 0, distCur = 0;
  for (int compIdx = compStr; compIdx <= compEnd; compIdx++)
  {
    ComponentID compId = (ComponentID)compIdx;
    {
      CompArea compArea = currCsArea.block( compId );
      CompArea         locArea  = compArea;
      locArea.x -= cu->blocks[compIdx].x;
      locArea.y -= cu->blocks[compIdx].y;
      CPelBuf reco      = picDbBuf.getBuf( locArea );
      CPelBuf org       = cs.getOrgBuf( compId );
      distCur += xGetDistortionDb( cs, org, reco, compArea, false );
    }

    if ( leftEdgeAvai )
    {
      const CompArea&  compArea = areaLeft.block( compId );
      CompArea         locArea  = compArea;
      locArea.x -= cu->blocks[compIdx].x;
      locArea.y -= cu->blocks[compIdx].y;
      CPelBuf org    = cs.picture->getOrigBuf( compArea );
      if ( cs.picture->getFilteredOrigBuffer().valid() )
      {
        org = cs.picture->getRspOrigBuf( compArea );
      }
      CPelBuf reco   = cs.picture->getRecoBuf( compArea );
      CPelBuf recoDb = picDbBuf.getBuf( locArea );
      distBeforeDb  += xGetDistortionDb( cs, org, reco,   compArea, true );
      distAfterDb   += xGetDistortionDb( cs, org, recoDb, compArea, false  );
    }

    if ( topEdgeAvai )
    {
      const CompArea&  compArea = areaTop.block( compId );
      CompArea         locArea  = compArea;
      locArea.x -= cu->blocks[compIdx].x;
      locArea.y -= cu->blocks[compIdx].y;
      CPelBuf org    = cs.picture->getOrigBuf( compArea );
      if ( cs.picture->getFilteredOrigBuffer().valid() )
      {
        org = cs.picture->getRspOrigBuf( compArea );
      }
      CPelBuf reco   = cs.picture->getRecoBuf( compArea );
      CPelBuf recoDb = picDbBuf.getBuf( locArea );
      distBeforeDb  += xGetDistortionDb( cs, org, reco,   compArea, true );
      distAfterDb   += xGetDistortionDb( cs, org, recoDb, compArea, false  );
    }
  }

  //updated cost
  int64_t distTmp = distCur - cs.dist + distAfterDb - distBeforeDb;
  cs.costDbOffset = distTmp < 0 ? -m_cRdCost.calcRdCost( 0, -distTmp ) : m_cRdCost.calcRdCost( 0, distTmp );
}

Distortion EncCu::xGetDistortionDb(CodingStructure &cs, CPelBuf& org, CPelBuf& reco, const CompArea& compArea, bool beforeDb)
{
  Distortion dist;
  const ReshapeData& reshapeData = cs.picture->reshapeData;
  const ComponentID compID = compArea.compID;
  if( (cs.slice->lmcsEnabled && reshapeData.getCTUFlag()) || m_pcEncCfg->m_lumaLevelToDeltaQPEnabled)
  {
    if ( compID == COMP_Y && !m_pcEncCfg->m_lumaLevelToDeltaQPEnabled)
    {
      CPelBuf tmpReco;
      if( beforeDb )
      {
        PelBuf tmpLmcs = m_aTmpStorageLCU[0].getCompactBuf( compArea );
        tmpLmcs.rspSignal( reco, reshapeData.getInvLUT() );
        tmpReco = tmpLmcs;
      }
      else
      {
        tmpReco = reco;
      }
      dist = m_cRdCost.getDistPart( org, tmpReco, cs.sps->bitDepths[CH_L], compID, DF_SSE_WTD, &org );
    }
    else if( m_EDO == 2)
    {
      // use the correct luma area to scale chroma
      const int csx = getComponentScaleX( compID, cs.area.chromaFormat );
      const int csy = getComponentScaleY( compID, cs.area.chromaFormat );
      CompArea lumaArea = CompArea( COMP_Y, cs.area.chromaFormat, Area( compArea.x << csx, compArea.y << csy, compArea.width << csx, compArea.height << csy), true);
      CPelBuf orgLuma = cs.picture->getFilteredOrigBuffer().valid() ? cs.picture->getRspOrigBuf( lumaArea ): cs.picture->getOrigBuf( lumaArea );
      dist = m_cRdCost.getDistPart( org, reco, cs.sps->bitDepths[toChannelType( compID )], compID, DF_SSE_WTD, &orgLuma );
    }
    else
    {
      const int csx = getComponentScaleX( compID, cs.area.chromaFormat );
      const int csy = getComponentScaleY( compID, cs.area.chromaFormat );
      CompArea lumaArea = compArea.compID ? CompArea( COMP_Y, cs.area.chromaFormat, Area( compArea.x << csx, compArea.y << csy, compArea.width << csx, compArea.height << csy), true) : cs.area.blocks[COMP_Y];
      CPelBuf orgLuma = cs.picture->getFilteredOrigBuffer().valid() ? cs.picture->getRspOrigBuf( lumaArea ): cs.picture->getOrigBuf( lumaArea );
//      CPelBuf orgLuma = cs.picture->getFilteredOrigBuffer().valid() ? cs.picture->getRspOrigBuf( cs.area.blocks[COMP_Y] ): cs.picture->getOrigBuf( cs.area.blocks[COMP_Y] );
      dist = m_cRdCost.getDistPart( org, reco, cs.sps->bitDepths[toChannelType( compID )], compID, DF_SSE_WTD, &orgLuma );
    }
    return dist;
  }

  if ( cs.slice->lmcsEnabled && cs.slice->isIntra() && compID == COMP_Y && !beforeDb ) //intra slice
  {
    PelBuf tmpLmcs = m_aTmpStorageLCU[0].getCompactBuf( compArea );
    tmpLmcs.rspSignal( reco, reshapeData.getFwdLUT() );
    dist = m_cRdCost.getDistPart( org, tmpLmcs, cs.sps->bitDepths[CH_L], compID, DF_SSE );
    return dist;
  }
  dist = m_cRdCost.getDistPart(org, reco, cs.sps->bitDepths[toChannelType(compID)], compID, DF_SSE);
  return dist;
}

bool checkValidMvs( const CodingUnit& cu)
{
  // clang-format off
  const int affineShiftTab[3] =
  {
    MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER,
    MV_PRECISION_INTERNAL - MV_PRECISION_SIXTEENTH,
    MV_PRECISION_INTERNAL - MV_PRECISION_INT
  };

  const int normalShiftTab[NUM_IMV_MODES] =
  {
    MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER,
    MV_PRECISION_INTERNAL - MV_PRECISION_INT,
    MV_PRECISION_INTERNAL - MV_PRECISION_4PEL,
    MV_PRECISION_INTERNAL - MV_PRECISION_HALF,
  };
  // clang-format on

  int mvShift;

  for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if (cu.refIdx[refList] >= 0)
    {
      if (!cu.affine)
      {
        mvShift = normalShiftTab[cu.imv];
        Mv signaledmvd(cu.mvd[refList][0].hor >> mvShift, cu.mvd[refList][0].ver >> mvShift);
        if (!((signaledmvd.hor >= MVD_MIN) && (signaledmvd.hor <= MVD_MAX)) || !((signaledmvd.ver >= MVD_MIN) && (signaledmvd.ver <= MVD_MAX)))
          return false;
      }
      else
      {
        for (int ctrlP = 1 + (cu.affineType == AFFINEMODEL_6PARAM); ctrlP >= 0; ctrlP--)
        {
          mvShift = affineShiftTab[cu.imv];
          Mv signaledmvd(cu.mvd[refList][ctrlP].hor >> mvShift, cu.mvd[refList][ctrlP].ver >> mvShift);
          if (!((signaledmvd.hor >= MVD_MIN) && (signaledmvd.hor <= MVD_MAX)) || !((signaledmvd.ver >= MVD_MIN) && (signaledmvd.ver <= MVD_MAX)))
            return false;;
        }
      }
    }
  }
  // avoid MV exceeding 18-bit dynamic range
  const int maxMv = 1 << 17;
  if (!cu.affine && !cu.mergeFlag)
  {
    if(    ( cu.refIdx[ 0 ] >= 0 && ( cu.mv[ 0 ][ 0 ].getAbsHor() >= maxMv || cu.mv[ 0 ][ 0 ].getAbsVer() >= maxMv ) )
        || ( cu.refIdx[ 1 ] >= 0 && ( cu.mv[ 1 ][ 0 ].getAbsHor() >= maxMv || cu.mv[ 1 ][ 0 ].getAbsVer() >= maxMv ) ) )
    {
      return false;
    }
  }
  if( cu.affine && !cu.mergeFlag )
  {
    for( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
    {
      if( cu.refIdx[ refList ] >= 0 )
      {
        for( int ctrlP = 1 + ( cu.affineType == AFFINEMODEL_6PARAM ); ctrlP >= 0; ctrlP-- )
        {
          if( cu.mv[ refList ][ ctrlP ].getAbsHor() >= maxMv || cu.mv[ refList ][ ctrlP ].getAbsVer() >= maxMv )
          {
            return false;
          }
        }
      }
    }
  }
  return true;
}


void EncCu::xEncodeInterResidual( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, int residualPass, bool* bestHasNonResi, double* equBcwCost )
{
  if( residualPass == 1 && encTestMode.lossless )
  {
    return;
  }

  CodingUnit*            cu        = tempCS->getCU( partitioner.chType, partitioner.treeType );
  double   bestCostInternal        = MAX_DOUBLE;

  if( !checkValidMvs( *cu ) )
    return;

  double  currBestCost = MAX_DOUBLE;

  // For SBT
  double     bestCost          = bestCS->cost;
  double     bestCostBegin     = bestCS->cost;
  const CodingUnit* prevBestCU = bestCS->getCU( partitioner.chType, partitioner.treeType );
  uint8_t    prevBestSbt       = ( prevBestCU == nullptr ) ? 0 : prevBestCU->sbtInfo;
  Distortion sbtOffDist        = 0;
  bool       sbtOffRootCbf     = 0;
  double     sbtOffCost        = MAX_DOUBLE;
  uint8_t    currBestSbt       = 0;
  uint8_t    histBestSbt       = MAX_UCHAR;
  Distortion curPuSse          = MAX_DISTORTION;
  uint8_t    numRDOTried       = 0;
  bool       doPreAnalyzeResi  = false;
  const bool mtsAllowed        =   tempCS->sps->MTSInter && cu->Y().maxDim() <= MTS_INTER_MAX_CU_SIZE;
  const uint8_t sbtAllowed     = ( tempCS->pps->picWidthInLumaSamples < SBT_FAST64_WIDTH_THRESHOLD || m_pcEncCfg->m_SBT > 1 ) && cu->Y().maxDim() > 32 ? 0 : CU::checkAllowedSbt(*cu);

  if( sbtAllowed )
  {
    //SBT resolution-dependent fast algorithm: not try size-64 SBT in RDO for low-resolution sequences (now resolution below HD)
    doPreAnalyzeResi = ( sbtAllowed || mtsAllowed ) && residualPass == 0;
    m_cInterSearch.getBestSbt( tempCS, cu, histBestSbt, curPuSse, sbtAllowed, doPreAnalyzeResi, mtsAllowed );
  }

  cu->skip    = false;
  cu->sbtInfo = 0;

  const bool skipResidual = residualPass == 1;
  if( skipResidual || histBestSbt == MAX_UCHAR || !CU::isSbtMode( histBestSbt ) )
  {
    m_cInterSearch.encodeResAndCalcRdInterCU( *tempCS, partitioner, skipResidual );
    xEncodeDontSplit( *tempCS, partitioner );
    xCheckDQP       ( *tempCS, partitioner );

    if( NULL != bestHasNonResi && (bestCostInternal > tempCS->cost) )
    {
      bestCostInternal = tempCS->cost;
      if( !cu->ciip )
        *bestHasNonResi = !cu->rootCbf;
    }

    if( cu->rootCbf == false )
    {
      if( cu->ciip )
      {
        tempCS->cost = MAX_DOUBLE;
        tempCS->costDbOffset = 0;
        return;
      }
    }
    currBestCost = tempCS->cost;
    if( sbtAllowed )
    {
      sbtOffCost    = tempCS->cost;
      sbtOffDist    = tempCS->dist;
      sbtOffRootCbf = cu->rootCbf;
      currBestSbt   = cu->firstTU->mtsIdx[COMP_Y] > MTS_SKIP ? SBT_OFF_MTS : SBT_OFF_DCT;
      numRDOTried  += mtsAllowed ? 2 : 1;
    }

    DTRACE_MODE_COST( *tempCS, m_cRdCost.getLambda( true ) );
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

    STAT_COUNT_CU_MODES( partitioner.chType == CH_L, g_cuCounters1D[CU_RD_TESTS][0][!tempCS->slice->isIntra() + tempCS->slice->depth] );
    STAT_COUNT_CU_MODES( partitioner.chType == CH_L && !tempCS->slice->isIntra(), g_cuCounters2D[CU_RD_TESTS][Log2( tempCS->area.lheight() )][Log2( tempCS->area.lwidth() )] );
  }

  if( sbtAllowed && ( m_pcEncCfg->m_SBT == 1 || sbtOffRootCbf ) )
  {
    bool swapped = false; // avoid unwanted data copy
    uint8_t numSbtRdo = CU::numSbtModeRdo( sbtAllowed );
    //early termination if all SBT modes are not allowed
    //normative
    if( !sbtAllowed || skipResidual )
    {
      numSbtRdo = 0;
    }
    //fast algorithm
    if( ( histBestSbt != MAX_UCHAR && !CU::isSbtMode( histBestSbt ) ) || m_cInterSearch.getSkipSbtAll() )
    {
      numSbtRdo = 0;
    }
    if( bestCost != MAX_DOUBLE && sbtOffCost != MAX_DOUBLE )
    {
      double th = 1.07;
      if( !( prevBestSbt == 0 || m_sbtCostSave[0] == MAX_DOUBLE ) )
      {
        assert( m_sbtCostSave[1] <= m_sbtCostSave[0] );
        th *= ( m_sbtCostSave[0] / m_sbtCostSave[1] );
      }
      if( sbtOffCost > bestCost * th )
      {
        numSbtRdo = 0;
      }
    }
    if( !sbtOffRootCbf && sbtOffCost != MAX_DOUBLE )
    {
      double th = Clip3( 0.05, 0.55, ( 27 - cu->qp ) * 0.02 + 0.35 );
      if( sbtOffCost < m_cRdCost.calcRdCost( ( cu->lwidth() * cu->lheight() ) << SCALE_BITS, 0 ) * th )
      {
        numSbtRdo = 0;
      }
    }

    if( histBestSbt != MAX_UCHAR && numSbtRdo != 0 )
    {
      numSbtRdo = 1;
      m_cInterSearch.initSbtRdoOrder( CU::getSbtMode( CU::getSbtIdx( histBestSbt ), CU::getSbtPos( histBestSbt ) ) );
    }

    for( int sbtModeIdx = 0; sbtModeIdx < numSbtRdo; sbtModeIdx++ )
    {
      uint8_t sbtMode = m_cInterSearch.getSbtRdoOrder( sbtModeIdx );
      uint8_t sbtIdx = CU::getSbtIdxFromSbtMode( sbtMode );
      uint8_t sbtPos = CU::getSbtPosFromSbtMode( sbtMode );

      //fast algorithm (early skip, save & load)
      if( histBestSbt == MAX_UCHAR )
      {
        uint8_t skipCode = m_cInterSearch.skipSbtByRDCost( cu->lwidth(), cu->lheight(), cu->mtDepth, sbtIdx, sbtPos, bestCS->cost, sbtOffDist, sbtOffCost, sbtOffRootCbf );
        if( skipCode != MAX_UCHAR )
        {
          continue;
        }

        if( sbtModeIdx > 0 )
        {
          uint8_t prevSbtMode = m_cInterSearch.getSbtRdoOrder( sbtModeIdx - 1 );
          //make sure the prevSbtMode is the same size as the current SBT mode (otherwise the estimated dist may not be comparable)
          if( CU::isSameSbtSize( prevSbtMode, sbtMode ) )
          {
            Distortion currEstDist = m_cInterSearch.getEstDistSbt( sbtMode );
            Distortion prevEstDist = m_cInterSearch.getEstDistSbt( prevSbtMode );
            if( currEstDist > prevEstDist * 1.15 )
            {
              continue;
            }
          }
        }
      }

      //init tempCS and TU
      if( bestCost == bestCS->cost ) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
      {
        tempCS->clearTUs();
      }
      else if( !swapped )
      {
        tempCS->initStructData( encTestMode.qp );
        tempCS->copyStructure( *bestCS, partitioner.chType, partitioner.treeType );
        tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );
        bestCost = bestCS->cost;
        cu = tempCS->getCU( partitioner.chType, partitioner.treeType );
        swapped = true;
      }
      else
      {
        tempCS->clearTUs();
        bestCost = bestCS->cost;
        cu = tempCS->getCU( partitioner.chType, partitioner.treeType );
      }

      //we need to restart the distortion for the new tempCS, the bit count and the cost
      tempCS->dist     = 0;
      tempCS->fracBits = 0;
      tempCS->cost     = MAX_DOUBLE;
      cu->skip         = false;


      //set SBT info
      cu->sbtInfo = (sbtPos << 4) + sbtIdx;

      //try residual coding
      m_cInterSearch.encodeResAndCalcRdInterCU( *tempCS, partitioner, skipResidual );
      numRDOTried++;

      xEncodeDontSplit( *tempCS, partitioner );
      xCheckDQP( *tempCS, partitioner );

      if( NULL != bestHasNonResi && ( bestCostInternal > tempCS->cost ) )
      {
        bestCostInternal = tempCS->cost;
        if( !( cu->ciip ) )
          *bestHasNonResi = !cu->rootCbf;
      }

      if( tempCS->cost < currBestCost )
      {
        currBestSbt = cu->sbtInfo;
        currBestCost = tempCS->cost;
      }
      else if( m_pcEncCfg->m_SBT > 2 )
      {
        sbtModeIdx = numSbtRdo;
      }

      DTRACE_MODE_COST( *tempCS, m_cRdCost.getLambda( true ) );
      xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
      STAT_COUNT_CU_MODES( partitioner.chType == CH_L, g_cuCounters1D[CU_RD_TESTS][0][!tempCS->slice->isIntra() + tempCS->slice->depth] );
      STAT_COUNT_CU_MODES( partitioner.chType == CH_L && !tempCS->slice->isIntra(), g_cuCounters2D[CU_RD_TESTS][Log2( tempCS->area.lheight() )][Log2( tempCS->area.lwidth() )] );
    }

    if( bestCostBegin != bestCS->cost )
    {
      m_sbtCostSave[0] = sbtOffCost;
      m_sbtCostSave[1] = currBestCost;
    }

    if( histBestSbt == MAX_UCHAR && doPreAnalyzeResi && numRDOTried > 1 )
    {
      auto slsSbt = static_cast<CacheBlkInfoCtrl&>( m_modeCtrl );
      int slShift = 4 + std::min( Log2( cu->lwidth() ) + Log2( cu->lheight() ), 9 );
      slsSbt.saveBestSbt( cu->cs->area, (uint32_t)( curPuSse >> slShift ), currBestSbt );
    }
    
    if( ETM_INTER_ME == encTestMode.type )
    {
      if( equBcwCost != NULL )
      {
        if( tempCS->cost < ( *equBcwCost ) && cu->BcwIdx == BCW_DEFAULT )
        {
          ( *equBcwCost ) = tempCS->cost;
        }
      }
      else
      {
        CHECK( equBcwCost == NULL, "equBcwCost == NULL" );
      }
      if( tempCS->slice->checkLDC && !cu->imv && cu->BcwIdx != BCW_DEFAULT && tempCS->cost < m_bestBcwCost[1] )
      {
        if( tempCS->cost < m_bestBcwCost[0] )
        {
          m_bestBcwCost[1] = m_bestBcwCost[0];
          m_bestBcwCost[0] = tempCS->cost;
          m_bestBcwIdx[1] = m_bestBcwIdx[0];
          m_bestBcwIdx[0] = cu->BcwIdx;
        }
        else
        {
          m_bestBcwCost[1] = tempCS->cost;
          m_bestBcwIdx[1] = cu->BcwIdx;
        }
      }
    }
  }

  tempCS->cost = currBestCost;
}

void EncCu::xEncodeDontSplit( CodingStructure &cs, Partitioner &partitioner )
{
  m_CABACEstimator->resetBits();

  m_CABACEstimator->split_cu_mode( CU_DONT_SPLIT, cs, partitioner );
  if( partitioner.treeType == TREE_C )
    CHECK( m_CABACEstimator->getEstFracBits() != 0, "must be 0 bit" );

  cs.fracBits += m_CABACEstimator->getEstFracBits(); // split bits
  cs.cost      = m_cRdCost.calcRdCost( cs.fracBits, cs.dist );
}

void EncCu::xReuseCachedResult( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner )
{
  EncTestMode cachedMode;

  if( ! m_modeCtrl.setCsFrom( *tempCS, cachedMode, partitioner ) )
  {
    THROW( "Should never happen!" );
  }

  CodingUnit& cu = *tempCS->cus.front();
  partitioner.setCUData( cu );

  if( CU::isIntra( cu ) )
  {
    if( isLuma( cu.chType ) )
    {
      cu.getMotionBuf().memset( -1 ); // clear motion buf
    }
    xReconIntraQT( cu );
  }
  else
  {
    xDeriveCUMV( cu );
    xReconInter( cu );
  }

  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CABACEstimator->resetBits();

  CUCtx cuCtx;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;
  m_CABACEstimator->coding_unit( cu, partitioner, cuCtx );

  tempCS->fracBits = m_CABACEstimator->getEstFracBits();
  tempCS->cost     = m_cRdCost.calcRdCost( tempCS->fracBits, tempCS->dist );

  xEncodeDontSplit( *tempCS,         partitioner );
  xCheckDQP       ( *tempCS,         partitioner );
  xCheckBestMode  (  tempCS, bestCS, partitioner, cachedMode, m_EDO );
}

uint64_t EncCu::xCalcPuMeBits( const CodingUnit &cu )
{
  CHECK( !cu.mergeFlag, "Should only be used for merge!" );
  CHECK( CU::isIBC( cu ), "Shound not be used for IBC" );

  m_CABACEstimator->resetBits();
  m_CABACEstimator->merge_flag(cu);
  if( cu.mergeFlag )
  {
    m_CABACEstimator->merge_data( cu );
  }
  return m_CABACEstimator->getEstFracBits();
}

double EncCu::xCalcDistortion(CodingStructure *&cur_CS, ChannelType chType, int BitDepth, int imv)
{
  const auto currDist1 = m_cRdCost.getDistPart(cur_CS->getOrgBuf( COMP_Y ), cur_CS->getPredBuf( COMP_Y ), BitDepth, COMP_Y, m_pcEncCfg->m_fastHad ? DF_HAD_fast : DF_HAD );
  unsigned int uiMvBits = 0;
  unsigned imvShift = imv == IMV_HPEL ? 1 : (imv << 1);
  const CodingUnit& cu = *cur_CS->getCU( chType, TREE_D);
  if (cu.interDir != 2)
  {
    uiMvBits += m_cRdCost.getBitsOfVectorWithPredictor(cu.mvd[0][0].hor, cu.mvd[0][0].ver, imvShift + MV_FRACTIONAL_BITS_DIFF);
  }
  if (cu.interDir != 1)
  {
    uiMvBits += m_cRdCost.getBitsOfVectorWithPredictor(cu.mvd[1][0].hor, cu.mvd[1][0].ver, imvShift + MV_FRACTIONAL_BITS_DIFF);
  }
  return (double(currDist1) + (double)m_cRdCost.getCost(uiMvBits));
}

int EncCu::xCheckMMVDCand(MmvdIdx& mmvdMergeCand, int& bestDir, int tempNum, double& bestCostOffset, double& bestCostMerge, double bestCostList )
{
  int baseIdx = mmvdMergeCand.val / MMVD_MAX_REFINE_NUM;
  int CandCur = mmvdMergeCand.val - MMVD_MAX_REFINE_NUM * baseIdx;

  if( m_pcEncCfg->m_MMVD > 2 )
  {
    if( CandCur % 4 == 0 )
    {
      if( ( bestCostOffset >= bestCostMerge ) && ( CandCur >= 4 ) )
      {
        if( mmvdMergeCand.val > MMVD_MAX_REFINE_NUM )
        {
          return 2;
        }
        else
        {
          mmvdMergeCand.val = MMVD_MAX_REFINE_NUM;
          if( tempNum == mmvdMergeCand.val )
          {
            return 2;
          }
        }
      }
      //reset
      bestCostOffset = MAX_DOUBLE;
      bestCostMerge  = bestCostList;
    }
  }

  if( mmvdMergeCand.val == MMVD_MAX_REFINE_NUM )
  {
    bestDir = 0;
  }
  if( CandCur >= 4 )
  {
    if( CandCur % 4 != bestDir )
    {
      return 1;
    }
  }
  return 0;
}


MergeItem::MergeItem()
{

}
MergeItem::~MergeItem()
{

}

void MergeItem::create( ChromaFormat chromaFormat, const Area &area )
{
  if( m_pelStorage.bufs.empty() )
  {
    m_pelStorage.create( chromaFormat, area );
    m_mvStorage .resize( area.area() >> ( MIN_CU_LOG2 << 1 ) );
  }

  init();
}

void MergeItem::init()
{
  // reset data
  cost        = MAX_DOUBLE;
  mergeIdx    = 0;
  bcwIdx      = 0;
  interDir    = 0;
  useAltHpelIf  = false;
  affineType    = AFFINEMODEL_4PARAM;
  mergeItemType = MergeItemType::NUM;

  noBdofRefine  = false;
  noResidual    = false;

  lumaPredReady   = false;
  chromaPredReady = false;
}

void MergeItem::importMergeInfo(const MergeCtx& mergeCtx, int _mergeIdx, MergeItemType _mergeItemType, CodingUnit& pu)
{
  mergeIdx      = _mergeIdx;
  mergeItemType = _mergeItemType;

  if( mergeItemType != MergeItemType::GPM && mergeItemType != MergeItemType::MMVD )
  {
    mvField[REF_PIC_LIST_0][0] = mergeCtx.mvFieldNeighbours [mergeIdx][REF_PIC_LIST_0];
    mvField[REF_PIC_LIST_1][0] = mergeCtx.mvFieldNeighbours [mergeIdx][REF_PIC_LIST_1];
    interDir                   = mergeCtx.interDirNeighbours[mergeIdx];
    bcwIdx                     = mergeCtx.BcwIdx            [mergeIdx];
    useAltHpelIf               = mergeCtx.useAltHpelIf      [mergeIdx];
  }

  switch( _mergeItemType )
  {
  case MergeItemType::REGULAR:
  case MergeItemType::CIIP:
    break;

  case MergeItemType::MMVD:
  {
    MmvdIdx candIdx;

    candIdx.val                = mergeIdx;
    mvField[L0][0]             . setMvField( pu.mv[L0][0], pu.refIdx[0] );
    mvField[L1][0]             . setMvField( pu.mv[L1][0], pu.refIdx[1] );
    interDir                   = pu.interDir;
    bcwIdx                     = pu.BcwIdx;
    useAltHpelIf               = mergeCtx.useAltHpelIf[candIdx.pos.baseIdx];

    break;
  }

  case MergeItemType::GPM:
    mvField[L0][0]             . setMvField( Mv( 0, 0 ), -1 );
    mvField[L1][0]             . setMvField( Mv( 0, 0 ), -1 );
    bcwIdx                     = BCW_DEFAULT;
    useAltHpelIf               = false;

    break;

  case MergeItemType::IBC:
  default:
    THROW( "Wrong merge item type" );
  }

  getMvBuf( pu ).copyFrom( pu.getMotionBuf() );
}

void MergeItem::importMergeInfo( const AffineMergeCtx &mergeCtx, int _mergeIdx, MergeItemType _mergeItemType, CodingUnit& pu )
{
  mergeIdx      = _mergeIdx;
  mergeItemType = _mergeItemType;

  affineType    = mergeCtx.affineType         [mergeIdx];
  interDir      = mergeCtx.interDirNeighbours [mergeIdx];
  bcwIdx        = mergeCtx.BcwIdx             [mergeIdx];
  useAltHpelIf  = false;

  switch( _mergeItemType )
  {
  case MergeItemType::SBTMVP:
    // the pu motion was already generated preparing for IFP check (unconditional)
    mvField[L0][0] . setMvField( pu.mv[L0][0], pu.refIdx[L0] );
    mvField[L1][0] . setMvField( pu.mv[L1][0], pu.refIdx[L1] );

    break;

  case MergeItemType::AFFINE:
    // the pu motion was already generated preparing for IFP check (unconditional)
    mvField[L0][0] . setMvField( pu.mv[L0][0], pu.refIdx[L0] );
    mvField[L0][1] . setMvField( pu.mv[L0][1], pu.refIdx[L0] );
    mvField[L0][2] . setMvField( pu.mv[L0][2], pu.refIdx[L0] );
    mvField[L1][0] . setMvField( pu.mv[L1][0], pu.refIdx[L1] );
    mvField[L1][1] . setMvField( pu.mv[L1][1], pu.refIdx[L1] );
    mvField[L1][2] . setMvField( pu.mv[L1][2], pu.refIdx[L1] );

    break;

  default:
    THROW( "Wrong merge item type" );
  }

  // the MI buf was already generated preparing for IFP check (unconditional)
  getMvBuf( pu ).copyFrom( pu.getMotionBuf() );
}

bool MergeItem::exportMergeInfo( CodingUnit &pu, bool forceNoResidual ) const
{
  pu.mergeFlag        = true;
  pu.mmvdMergeFlag    = false;
  pu.interDir         = interDir;
  pu.mergeIdx         = mergeIdx;
  pu.mergeType        = MRG_TYPE_DEFAULT_N;
  pu.mv[REF_PIC_LIST_0][0]  = mvField[REF_PIC_LIST_0][0].mv;
  pu.mv[REF_PIC_LIST_1][0]  = mvField[REF_PIC_LIST_1][0].mv;
  pu.refIdx[REF_PIC_LIST_0] = mvField[REF_PIC_LIST_0][0].refIdx;
  pu.refIdx[REF_PIC_LIST_1] = mvField[REF_PIC_LIST_1][0].refIdx;
  pu.mvd[REF_PIC_LIST_0][0] = Mv();
  pu.mvd[REF_PIC_LIST_1][0] = Mv();
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
  pu.BcwIdx         = ( interDir == 3 ) ? bcwIdx : BCW_DEFAULT;
  pu.mcControl      = 0;
  pu.mmvdSkip       = false;
  pu.affine         = false;
  pu.affineType     = AFFINEMODEL_4PARAM;
  pu.geo            = false;
  pu.mtsFlag        = false;
  pu.ciip           = false;
  pu.imv            = ( !pu.geo && useAltHpelIf ) ? IMV_HPEL : 0;
  pu.mvRefine       = false;

  const bool resetCiip2Regular = mergeItemType == MergeItemType::CIIP && forceNoResidual;
  MergeItemType updatedType    = resetCiip2Regular ? MergeItemType::REGULAR : mergeItemType;

  switch( updatedType )
  {
  case MergeItemType::REGULAR:
    CU::restrictBiPredMergeCandsOne( pu );
    break;

  case MergeItemType::CIIP:
    CHECK( forceNoResidual, "Cannot force no residuals for CIIP" );
    pu.ciip           = true;
    pu.intraDir[CH_L] = PLANAR_IDX;
    pu.intraDir[CH_C] = DM_CHROMA_IDX;
    break;

  case MergeItemType::MMVD:
    pu.mmvdMergeFlag    = true;
    pu.mmvdMergeIdx.val = mergeIdx;
    if( forceNoResidual )
    {
      pu.mmvdSkip       = true;
    }
    CU::restrictBiPredMergeCandsOne( pu );
    break;

  case MergeItemType::SBTMVP:
    pu.affine    = true;
    pu.mergeType = MRG_TYPE_SUBPU_ATMVP;
    break;

  case MergeItemType::AFFINE:
    pu.affine     = true;
    pu.affineType = affineType;
    pu.mv[L0][0]  = mvField[L0][0].mv;
    pu.mv[L1][0]  = mvField[L1][0].mv;
    pu.mv[L0][1]  = mvField[L0][1].mv;
    pu.mv[L1][1]  = mvField[L1][1].mv;
    pu.mv[L0][2]  = mvField[L0][2].mv;
    pu.mv[L1][2]  = mvField[L1][2].mv;
    pu.refIdx[L0] = mvField[L0][0].refIdx;
    pu.refIdx[L1] = mvField[L1][0].refIdx;
    break;

  case MergeItemType::GPM:
    pu.mergeIdx = -1;
    pu.geo      = true;
    pu.BcwIdx   = BCW_DEFAULT;
    updateGpmIdx( mergeIdx, pu.geoSplitDir, pu.geoMergeIdx );
    pu.imv      = 0;
    break;

  case MergeItemType::IBC:
  default:
    THROW( "Wrong merge item type" );
  }

  pu.getMotionBuf().copyFrom( getMvBuf( pu ) );

  return resetCiip2Regular;
}

MergeItemList::MergeItemList()
{

}

MergeItemList::~MergeItemList()
{
  for( MergeItem* p : m_list )
  {
    delete p;
  }
  m_list.clear();

  for( MergeItem *p : m_mergeItems )
  {
    delete p;
  }
  m_mergeItems.clear();
}

void MergeItemList::init( size_t maxSize, size_t maxExtSize, ChromaFormat chromaFormat, SizeType ctuWidth, SizeType ctuHeight )
{
  CHECK( !m_mergeItems.empty() || !m_list.empty(), "MergeItemList already initialized" );

  m_list      . reserve( maxSize + 1 ); // to avoid reallocation when inserting a new item
  m_mergeItems. reserve( maxSize + 1 );
  m_maxSize   = maxSize;
  m_maxExtSize= maxExtSize;
  m_numExt    = 0;

  for( int i = 0; i < maxSize + m_maxExtSize; i++ )
  {
    MergeItem *p = new MergeItem;
    p->create( chromaFormat, Area{ 0, 0, ctuWidth, ctuHeight } );
    m_mergeItems.push_back( p );
  }
}

MergeItem *MergeItemList::allocateNewMergeItem()
{
  m_numExt++;
  CHECK( m_mergeItems.empty(), "Missing merge items!" );
  CHECK( m_numExt > m_maxExtSize, "Taking out more external items than specified during list allocation!" );
  MergeItem *p = m_mergeItems.back();
  m_mergeItems.pop_back();
  p->init();
  return p;
}

bool MergeItemList::insertMergeItemToList( MergeItem *p )
{
  CHECK( m_list.size() + m_mergeItems.size() + m_numExt != m_maxSize + m_maxExtSize, "Wrong number of items held" );

  m_numExt--;

  if( m_list.empty() )
  {
    m_list.push_back( p );
  }
  else if( m_list.size() == m_maxTrackingNum && p->cost >= m_list.back()->cost )
  {
    m_mergeItems.push_back( p );
    return false;
  }
  else
  {
    if( m_list.size() == m_maxTrackingNum )
    {
      m_mergeItems.push_back( m_list.back() );
      m_list      .pop_back();
    }
    auto it = std::find_if( m_list.begin(), m_list.end(), [&p]( const MergeItem *mi ) { return p->cost < mi->cost; } );
    m_list.insert( it, p );
  }

  return true;
}

void MergeItemList::giveBackMergeItem( MergeItem *p )
{
  CHECK( m_list.size() + m_mergeItems.size() + m_numExt != m_maxSize + m_maxExtSize, "Wrong number of items held" );

  m_numExt--;

  m_mergeItems.push_back( p );
}

MergeItem *MergeItemList::getMergeItemInList( size_t index )
{
  return index < m_maxTrackingNum ? m_list[index] : nullptr;
}

void MergeItemList::resetList( size_t maxTrackingNum )
{
  CHECK( maxTrackingNum > m_maxSize, "Not enough items allocated to track " << maxTrackingNum << " items" );

  for( auto p : m_list )
  {
    m_mergeItems.push_back( p );
  }
  m_list.clear  ();

  m_maxTrackingNum = maxTrackingNum;
}

void MergeItemList::shrinkList( size_t reduceTo )
{
  CHECK( reduceTo > m_maxSize, "Not enough items allocated to track " << reduceTo << " items" );

  while( m_list.size() > reduceTo )
  {
    m_mergeItems.push_back( m_list.back() );
    m_list      .pop_back();
  }
}

} // namespace vvenc

//! \}

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


/** \file     EncSlice.cpp
    \brief    slice encoder class
*/

#include "EncSlice.h"
#include "EncStage.h"
#include "EncLib.h"
#include "EncPicture.h"
#include "BitAllocation.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/Picture.h"
#include "CommonLib/TimeProfiler.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "Utilities/NoMallocThreadPool.h"

#include <math.h>
#include "vvenc/vvencCfg.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

#ifdef TRACE_ENABLE_ITT
static const __itt_domain* itt_domain_encode              = __itt_domain_create( "Encode" );
static const __itt_string_handle* itt_handle_ctuEncode    = __itt_string_handle_create( "Encode_CTU" );
static const __itt_string_handle* itt_handle_rspLfVer     = __itt_string_handle_create( "RspLfVer_CTU" );
static const __itt_string_handle* itt_handle_lfHor        = __itt_string_handle_create( "LfHor_CTU" );
static const __itt_string_handle* itt_handle_sao          = __itt_string_handle_create( "SAO_CTU" );
static const __itt_string_handle* itt_handle_alf_stat     = __itt_string_handle_create( "ALF_CTU_STAT" );
static const __itt_string_handle* itt_handle_alf_derive   = __itt_string_handle_create( "ALF_DERIVE" );
static const __itt_string_handle* itt_handle_alf_recon    = __itt_string_handle_create( "ALF_RECONSTRUCT" );
static const __itt_string_handle* itt_handle_ccalf_stat   = __itt_string_handle_create( "CCALF_CTU_STAT" );
static const __itt_string_handle* itt_handle_ccalf_derive = __itt_string_handle_create( "CCALF_DERIVE" );
static const __itt_string_handle* itt_handle_ccalf_recon  = __itt_string_handle_create( "CCALF_RECONSTRUCT" );
#endif

void setArbitraryWppPattern( const PreCalcValues& pcv, std::vector<int>& ctuAddrMap, int stepX = 1 )
{
  ctuAddrMap.resize( pcv.sizeInCtus, 0 );
  std::vector<int> x_in_line( pcv.heightInCtus, 0 );
  int x = 0, y = 0, addr = 0;
  int y_top = 0;
  const int step = stepX; // number of CTUs in x-direction to scan 
  ctuAddrMap[addr++] = x++; // first entry (can be omitted)
  while( addr < pcv.sizeInCtus )
  {
    // fill entries in x-direction
    int x1 = x;
    while( x < std::min(x1 + step, (int)pcv.widthInCtus) )
    {
      // general WPP condition (top-right CTU availability)
      if( y > 0 && !( x_in_line[y - 1] - x >= 2 ) && x != pcv.widthInCtus - 1 )
        break;
      ctuAddrMap[addr++] = y*pcv.widthInCtus + x;
      x++;
    }
    x_in_line[y] = x;
        
    y += 1;

    if( y >= pcv.heightInCtus )
    {
      // go up
      if( x_in_line[y_top] >= pcv.widthInCtus )
      {
        y_top++;
        if( y_top >= pcv.heightInCtus )
        {
          // done
          break;
        }
      }
      y = y_top;
    }
    x = x_in_line[y];

    CHECK( y >= pcv.heightInCtus, "Height in CTUs is exceeded" );
  }
}

struct TileLineEncRsrc
{
  BitEstimator            m_BitEstimator;
  CABACWriter             m_CABACEstimator;
  BitEstimator            m_SaoBitEstimator;
  CABACWriter             m_SaoCABACEstimator;
  BitEstimator            m_AlfBitEstimator;
  CABACWriter             m_AlfCABACEstimator;
  ReuseUniMv              m_ReuseUniMv;
  BlkUniMvInfoBuffer      m_BlkUniMvInfoBuffer;
  AffineProfList          m_AffineProfList;
  IbcBvCand               m_CachedBvs;
  EncSampleAdaptiveOffset m_encSao;
  int                     m_prevQp[ MAX_NUM_CH ];
  TileLineEncRsrc( const VVEncCfg& encCfg ) : m_CABACEstimator( m_BitEstimator ), m_SaoCABACEstimator( m_SaoBitEstimator ), m_AlfCABACEstimator( m_AlfBitEstimator ) { m_AffineProfList.init( ! encCfg.m_picReordering ); }
};

struct PerThreadRsrc
{
  CtxCache  m_CtxCache;
  EncCu     m_encCu;
  PelStorage m_alfTempCtuBuf;
};

struct CtuEncParam
{
  Picture*  pic;
  EncSlice* encSlice;
  int       ctuRsAddr;
  int       ctuPosX;
  int       ctuPosY;
  UnitArea  ctuArea;
  int       tileLineResIdx;

  CtuEncParam() : pic( nullptr ), encSlice( nullptr ), ctuRsAddr( 0 ), ctuPosX( 0 ), ctuPosY( 0 ), ctuArea(), tileLineResIdx( 0 ) {}
  CtuEncParam( Picture* _p, EncSlice* _s, const int _r, const int _x, const int _y, const int _tileLineResIdx )
    : pic( _p )
    , encSlice( _s )
    , ctuRsAddr( _r )
    , ctuPosX( _x )
    , ctuPosY( _y )
    , ctuArea( pic->chromaFormat, pic->slices[0]->pps->pcv->getCtuArea( _x, _y ) )
    , tileLineResIdx( _tileLineResIdx ) {}
};

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncSlice::EncSlice()
  : m_pcEncCfg           ( nullptr)
  , m_threadPool         ( nullptr )
  , m_ctuTasksDoneCounter( nullptr )
  , m_ctuEncDelay        ( 1 )
  , m_pLoopFilter        ( nullptr )
  , m_pALF               ( nullptr )
  , m_pcRateCtrl         ( nullptr )
  , m_CABACWriter        ( m_BinEncoder )
  , m_encCABACTableIdx   ( VVENC_I_SLICE )
{
}


EncSlice::~EncSlice()
{
  for( auto* lnRsc : m_TileLineEncRsrc )
  {
    delete lnRsc;
  }
  m_TileLineEncRsrc.clear();

  for( auto* taskRsc: m_ThreadRsrc )
  {
    taskRsc->m_alfTempCtuBuf.destroy();
    delete taskRsc;
  }
  m_ThreadRsrc.clear();

  m_saoReconParams.clear();

  for( int i = 0; i < m_saoStatData.size(); i++ )
  {
    for( int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++ )
    {
      delete[] m_saoStatData[ i ][ compIdx ];
    }
    delete[] m_saoStatData[ i ];
  }
  m_saoStatData.clear();
}

void EncSlice::init( const VVEncCfg& encCfg,
                     const SPS& sps,
                     const PPS& pps,
                     std::vector<int>* const globalCtuQpVector,
                     LoopFilter& loopFilter,
                     EncAdaptiveLoopFilter& alf,
                     RateCtrl& rateCtrl,
                     NoMallocThreadPool* threadPool,
                     WaitCounter* ctuTasksDoneCounter )
{
  m_pcEncCfg            = &encCfg;
  m_pLoopFilter         = &loopFilter;
  m_pALF                = &alf;
  m_pcRateCtrl          = &rateCtrl;
  m_threadPool          = threadPool;
  m_ctuTasksDoneCounter = ctuTasksDoneCounter;
  m_syncPicCtx.resize( encCfg.m_entropyCodingSyncEnabled ? pps.getNumTileLineIds() : 0 );

  
  const int maxCntRscr = ( encCfg.m_numThreads > 0 ) ? pps.getNumTileLineIds() : 1;
  const int maxCtuEnc  = ( encCfg.m_numThreads > 0 && threadPool ) ? threadPool->numThreads() : 1;

  m_ThreadRsrc.resize( maxCtuEnc,  nullptr );
  m_TileLineEncRsrc.resize( maxCntRscr, nullptr );

  for( PerThreadRsrc*& taskRsc : m_ThreadRsrc )
  {
    taskRsc = new PerThreadRsrc();
    taskRsc->m_encCu.init( encCfg,
                           sps,
                           globalCtuQpVector,
                           m_syncPicCtx.data(),
                           &rateCtrl );
    taskRsc->m_alfTempCtuBuf.create( pps.pcv->chrFormat, Area( 0, 0, pps.pcv->maxCUSize + (MAX_ALF_PADDING_SIZE << 1), pps.pcv->maxCUSize + (MAX_ALF_PADDING_SIZE << 1) ), pps.pcv->maxCUSize, MAX_ALF_PADDING_SIZE, 0, false );
  }

  for( TileLineEncRsrc*& lnRsc : m_TileLineEncRsrc )
  {
    lnRsc = new TileLineEncRsrc( encCfg );
    if( sps.saoEnabled )
    {
      lnRsc->m_encSao.init( encCfg );
    }
  }

  const int sizeInCtus = pps.pcv->sizeInCtus;
  m_processStates = std::vector<ProcessCtuState>( sizeInCtus );
  m_saoReconParams.resize( sizeInCtus );

  ::memset( m_saoDisabledRate, 0, sizeof( m_saoDisabledRate ) );

  // sao statistics
  if( encCfg.m_bUseSAO )
  {
    m_saoStatData.resize( sizeInCtus );
    for( int i = 0; i < sizeInCtus; i++ )
    {
      m_saoStatData[ i ] = new SAOStatData*[ MAX_NUM_COMP ];
      for( int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++ )
      {
        m_saoStatData[ i ][ compIdx ] = new SAOStatData[ NUM_SAO_NEW_TYPES ];
      }
    }
  }
  ctuEncParams.resize( sizeInCtus );
  setArbitraryWppPattern( *pps.pcv, m_ctuAddrMap, 3 );

  const unsigned asuHeightInCtus = m_pALF->getAsuHeightInCtus();
  const unsigned numDeriveLines  = encCfg.m_ifpLines ? 
    std::min( ((encCfg.m_ifpLines & (~(asuHeightInCtus - 1))) + asuHeightInCtus), pps.pcv->heightInCtus ) : pps.pcv->heightInCtus;
  m_alfDeriveLine = numDeriveLines - 1;
  m_alfDeriveCtu  = numDeriveLines * pps.pcv->widthInCtus - 1;
}


void EncSlice::initPic( Picture* pic )
{
  Slice* slice = pic->cs->slice;

  if( slice->pps->numTileCols * slice->pps->numTileRows > 1 )
  {
    slice->sliceMap = slice->pps->sliceMap[0];
  }
  else
  {
    slice->sliceMap.addCtusToSlice( 0, pic->cs->pcv->widthInCtus, 0, pic->cs->pcv->heightInCtus, pic->cs->pcv->widthInCtus);
  }

  // this ensures that independently encoded bitstream chunks can be combined to bit-equal
  const SliceType cabacTableIdx = ! slice->pps->cabacInitPresent || slice->pendingRasInit ? slice->sliceType : m_encCABACTableIdx;
  slice->encCABACTableIdx = cabacTableIdx;

  // set QP and lambda values
  xInitSliceLambdaQP( slice );

  for( auto* thrRsc : m_ThreadRsrc )
  {
    thrRsc->m_encCu.initPic( pic );
  }

  for( auto* lnRsc : m_TileLineEncRsrc )
  {
    lnRsc->m_ReuseUniMv.resetReusedUniMvs();
  }

  m_ctuEncDelay = 1;
  if( pic->useIBC )
  {
    // IBC needs unfiltered samples up to max IBC search range
    // therefore ensure that numCtuDelayLUT CTU's have been enocded first
    // assuming IBC localSearchRangeX / Y = 128
    const int numCtuDelayLUT[ 3 ] = { 15, 3, 1 };
    CHECK( pic->cs->pcv->maxCUSizeLog2 < 5 || pic->cs->pcv->maxCUSizeLog2 > 7, "invalid max CTUSize" );
    m_ctuEncDelay = numCtuDelayLUT[ pic->cs->pcv->maxCUSizeLog2 - 5 ];
  }
}



void EncSlice::xInitSliceLambdaQP( Slice* slice )
{
  // pre-compute lambda and QP
  const bool rcp = (m_pcEncCfg->m_RCTargetBitrate > 0 && slice->pic->picInitialQP >= 0); // 2nd pass
  int  iQP = Clip3 (-slice->sps->qpBDOffset[CH_L], MAX_QP, slice->pic->picInitialQP); // RC start QP
  double dQP     = (rcp ? (double) slice->pic->picInitialQP : xGetQPForPicture (slice));
  double dLambda = (rcp ? slice->pic->picInitialLambda : xCalculateLambda (slice, slice->TLayer, dQP, dQP, iQP));
  int sliceChromaQpOffsetIntraOrPeriodic[2] = { m_pcEncCfg->m_sliceChromaQpOffsetIntraOrPeriodic[0], m_pcEncCfg->m_sliceChromaQpOffsetIntraOrPeriodic[1] };
  const int lookAheadRCCQpOffset = 0;   // was (m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_LookAhead && CS::isDualITree (*slice->pic->cs) ? 1 : 0);
  int cbQP = 0, crQP = 0, cbCrQP = 0;

  if (m_pcEncCfg->m_usePerceptQPA) // adapt sliceChromaQpOffsetIntraOrPeriodic and pic->ctuAdaptedQP
  {
    const bool cqp = (slice->isIntra() && !slice->sps->IBC) || (m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity > 0 && (slice->poc % m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity) == 0);
    const uint32_t startCtuTsAddr    = slice->sliceMap.ctuAddrInSlice[0];
    const uint32_t boundingCtuTsAddr = slice->pic->cs->pcv->sizeInCtus;

    if ((iQP = BitAllocation::applyQPAdaptationSlice (slice, m_pcEncCfg, iQP, dLambda, &slice->pic->picVisActY, // updates pic->picInitialQP
                                                      *m_ThreadRsrc[0]->m_encCu.getQpPtr(), m_pcRateCtrl->getIntraPQPAStats(),
                                                      (slice->pps->sliceChromaQpFlag && cqp ? sliceChromaQpOffsetIntraOrPeriodic : nullptr),
                                                      m_pcRateCtrl->getMinNoiseLevels(), startCtuTsAddr, boundingCtuTsAddr)) >= 0) // QP OK?
    {
      dLambda *= pow (2.0, ((double) iQP - dQP) / 3.0); // adjust lambda based on change of slice QP
    }
    else iQP = (int) dQP; // revert to unadapted slice QP
  }
  else if (rcp)
  {
    slice->pic->picInitialQP = -1; // no QPA - unused now
  }

  if (slice->pps->sliceChromaQpFlag && CS::isDualITree (*slice->pic->cs) && !m_pcEncCfg->m_usePerceptQPA && (m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity == 0))
  {
    cbQP = m_pcEncCfg->m_chromaCbQpOffsetDualTree + lookAheadRCCQpOffset; // QP offset for dual-tree
    crQP = m_pcEncCfg->m_chromaCrQpOffsetDualTree + lookAheadRCCQpOffset;
    cbCrQP = m_pcEncCfg->m_chromaCbCrQpOffsetDualTree + lookAheadRCCQpOffset;
  }
  else if (slice->pps->sliceChromaQpFlag)
  {
    const GOPEntry &gopEntry             = *(slice->pic->gopEntry);
    const bool bUseIntraOrPeriodicOffset = (slice->isIntra() && !slice->sps->IBC) || (m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity > 0 && (slice->poc % m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity) == 0);

    cbQP = (bUseIntraOrPeriodicOffset ? sliceChromaQpOffsetIntraOrPeriodic[0] : gopEntry.m_CbQPoffset) + lookAheadRCCQpOffset;
    crQP = (bUseIntraOrPeriodicOffset ? sliceChromaQpOffsetIntraOrPeriodic[1] : gopEntry.m_CrQPoffset) + lookAheadRCCQpOffset;
    cbCrQP = (cbQP + crQP) >> 1; // use floor of average CbCr chroma QP offset for joint-CbCr coding

    cbQP = Clip3 (-12, 12, cbQP + slice->pps->chromaQpOffset[COMP_Cb]) - slice->pps->chromaQpOffset[COMP_Cb];
    crQP = Clip3 (-12, 12, crQP + slice->pps->chromaQpOffset[COMP_Cr]) - slice->pps->chromaQpOffset[COMP_Cr];
    cbCrQP = Clip3 (-12, 12, cbCrQP + slice->pps->chromaQpOffset[COMP_JOINT_CbCr]) - slice->pps->chromaQpOffset[COMP_JOINT_CbCr];
  }

  slice->sliceChromaQpDelta[COMP_Cb] = Clip3 (-12, 12, cbQP);
  slice->sliceChromaQpDelta[COMP_Cr] = Clip3 (-12, 12, crQP);
  slice->sliceChromaQpDelta[COMP_JOINT_CbCr] = (slice->sps->jointCbCr ? Clip3 (-12, 12, cbCrQP) : 0);

  for( auto& thrRsc : m_ThreadRsrc )
  {
    thrRsc->m_encCu.setUpLambda( *slice, dLambda, iQP, true, true );
  }

  slice->sliceQp            = iQP;
  slice->chromaQpAdjEnabled = slice->pps->chromaQpOffsetListLen > 0;
}

static const int highTL[6] = { -1, 0, 0, 2, 4, 5 };

int EncSlice::xGetQPForPicture( const Slice* slice )
{
  const int lumaQpBDOffset = slice->sps->qpBDOffset[ CH_L ];
  int qp;

  if ( m_pcEncCfg->m_costMode == VVENC_COST_LOSSLESS_CODING )
  {
    qp = LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
  }
  else
  {
    qp = m_pcEncCfg->m_QP + slice->pic->gopAdaptedQP;

    if (m_pcEncCfg->m_usePerceptQPA)
    {
      qp = (slice->isIntra() ? std::min (qp, ((qp - std::min (3, floorLog2 (m_pcEncCfg->m_GOPSize) - 4/*TODO 3 with JVET-AC0149?*/)) * 15 + 3) >> 4) : highTL[slice->TLayer] + ((qp * (16 + std::min (2u, slice->TLayer))) >> 4) + 0/*TODO +-1?*/);
    }
    else if( slice->isIntra() )
    {
      qp += m_pcEncCfg->m_intraQPOffset;
    }
    else
    {
      if( qp != -lumaQpBDOffset )
      {
        const GOPEntry &gopEntry = *(slice->pic->gopEntry);
        // adjust QP according to the QP offset for the GOP entry.
        qp += gopEntry.m_QPOffset;

        // adjust QP according to QPOffsetModel for the GOP entry.
        double dqpOffset = qp * gopEntry.m_QPOffsetModelScale + gopEntry.m_QPOffsetModelOffset + 0.5;
        int qpOffset = (int)floor( Clip3<double>( 0.0, 3.0, dqpOffset ) );
        qp += qpOffset;
      }
    }

    if( m_pcEncCfg->m_blockImportanceMapping && !slice->pic->m_picShared->m_ctuBimQpOffset.empty() )
    {
      qp += slice->pic->m_picShared->m_picAuxQpOffset;
    }
  }
  qp = Clip3( -lumaQpBDOffset, MAX_QP, qp );
  return qp;
}


double EncSlice::xCalculateLambda( const Slice* slice,
                                   const int    depth, // slice GOP hierarchical depth.
                                   const double refQP, // initial slice-level QP
                                   const double dQP,   // initial double-precision QP
                                         int&   iQP )  // returned integer QP.
{
  const GOPEntry &gopEntry = *(slice->pic->gopEntry);
  const int SHIFT_QP       = 12;
  const int temporalId     = gopEntry.m_temporalId;
  std::vector<double> intraLambdaModifiers;
  for ( int i = 0; i < VVENC_MAX_TLAYER; i++ )
  {
    if( m_pcEncCfg->m_adIntraLambdaModifier[i] != 0.0 ) intraLambdaModifiers.push_back( m_pcEncCfg->m_adIntraLambdaModifier[i] );
    else break;
  }

  int bitdepth_luma_qp_scale = 6
                               * (slice->sps->bitDepths[ CH_L ] - 8
                                  - DISTORTION_PRECISION_ADJUSTMENT(slice->sps->bitDepths[ CH_L ]));
  double qp_temp = dQP + bitdepth_luma_qp_scale - SHIFT_QP;
  // Case #1: I or P-slices (key-frame)
  double dQPFactor = gopEntry.m_QPFactor;
  if( slice->sliceType == VVENC_I_SLICE )
  {
    if (m_pcEncCfg->m_dIntraQpFactor>=0.0 && gopEntry.m_sliceType != 'I')
    {
      dQPFactor = m_pcEncCfg->m_dIntraQpFactor;
    }
    else
    {
      dQPFactor = 0.57;
      if( ! m_pcEncCfg->m_lambdaFromQPEnable )
      {
        const int NumberBFrames = ( m_pcEncCfg->m_GOPSize - 1 );
        const double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05 * (double)NumberBFrames );
        dQPFactor *= dLambda_scale;
      }
    }
  }
  else if( m_pcEncCfg->m_lambdaFromQPEnable )
  {
    dQPFactor=0.57;
  }

  double dLambda = dQPFactor*pow( 2.0, qp_temp/3.0 );

  if( !(m_pcEncCfg->m_lambdaFromQPEnable) && depth>0 )
  {
    double qp_temp_ref = refQP + bitdepth_luma_qp_scale - SHIFT_QP;
    dLambda *= Clip3(2.00, 4.00, (qp_temp_ref / 6.0));   // (j == B_SLICE && p_cur_frm->layer != 0 )
  }

  // if hadamard is used in ME process
  if ( !m_pcEncCfg->m_bUseHADME && slice->sliceType != VVENC_I_SLICE )
  {
    dLambda *= 0.95;
  }

  double lambdaModifier;
  if( slice->sliceType != VVENC_I_SLICE || intraLambdaModifiers.empty())
  {
    lambdaModifier = m_pcEncCfg->m_adLambdaModifier[ temporalId ];
  }
  else
  {
    lambdaModifier = intraLambdaModifiers[ (temporalId < intraLambdaModifiers.size()) ? temporalId : (intraLambdaModifiers.size()-1) ];
  }
  dLambda *= lambdaModifier;

  iQP = Clip3( -slice->sps->qpBDOffset[ CH_L ], MAX_QP, (int) floor( dQP + 0.5 ) );

  if( m_pcEncCfg->m_DepQuantEnabled )
  {
    dLambda *= pow( 2.0, 0.25/3.0 ); // slight lambda adjustment for dependent quantization (due to different slope of quantizer)
  }

  // NOTE: the lambda modifiers that are sometimes applied later might be best always applied in here.
  return dLambda;
}


// ====================================================================================================================
// Public member functions
// ====================================================================================================================


/** \param pic   picture class
 */
void EncSlice::compressSlice( Picture* pic )
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_COMPRESS_SLICE );
  CodingStructure& cs         = *pic->cs;
  Slice* const slice          = cs.slice;
  uint32_t  startCtuTsAddr    = slice->sliceMap.ctuAddrInSlice[0];
  uint32_t  boundingCtuTsAddr = pic->cs->pcv->sizeInCtus;

  cs.pcv      = slice->pps->pcv;
  cs.fracBits = 0;

  if( startCtuTsAddr == 0 )
  {
    cs.initStructData( slice->sliceQp );
  }

  for( auto* thrRsrc : m_ThreadRsrc )
  {
    thrRsrc->m_encCu.initSlice( slice );
  }

  for( auto* lnRsrc : m_TileLineEncRsrc )
  {
    lnRsrc->m_CABACEstimator    .initCtxModels( *slice );
    lnRsrc->m_SaoCABACEstimator .initCtxModels( *slice );
    lnRsrc->m_AlfCABACEstimator .initCtxModels( *slice );
    lnRsrc->m_AffineProfList    .resetAffineMVList();
    lnRsrc->m_BlkUniMvInfoBuffer.resetUniMvList();
    lnRsrc->m_CachedBvs         .resetIbcBvCand();

    if( slice->sps->saoEnabled && pic->useSAO )
    {
      lnRsrc->m_encSao          .initSlice( slice );
    }
  }

  if( slice->sps->fpelMmvd && !slice->picHeader->disFracMMVD )
  {
    slice->picHeader->disFracMMVD = ( pic->lwidth() * pic->lheight() > 1920 * 1080 ) ? true : false;
  }

  xProcessCtus( pic, startCtuTsAddr, boundingCtuTsAddr );
}

void setJointCbCrModes( CodingStructure& cs, const Position topLeftLuma, const Size sizeLuma )
{
  bool              sgnFlag = true;

  if( isChromaEnabled( cs.picture->chromaFormat) )
  {
    const CompArea  cbArea  = CompArea( COMP_Cb, cs.picture->chromaFormat, Area(topLeftLuma,sizeLuma), true );
    const CompArea  crArea  = CompArea( COMP_Cr, cs.picture->chromaFormat, Area(topLeftLuma,sizeLuma), true );

    const CPelBuf   orgCb   = cs.picture->getFilteredOrigBuffer().valid() ? cs.picture->getRspOrigBuf( cbArea ): cs.picture->getOrigBuf( cbArea );
    const CPelBuf   orgCr   = cs.picture->getFilteredOrigBuffer().valid() ? cs.picture->getRspOrigBuf( crArea ): cs.picture->getOrigBuf( crArea );
    const int       x0      = ( cbArea.x > 0 ? 0 : 1 );
    const int       y0      = ( cbArea.y > 0 ? 0 : 1 );
    const int       x1      = ( cbArea.x + cbArea.width  < cs.picture->Cb().width  ? cbArea.width  : cbArea.width  - 1 );
    const int       y1      = ( cbArea.y + cbArea.height < cs.picture->Cb().height ? cbArea.height : cbArea.height - 1 );
    const int       cbs     = orgCb.stride;
    const int       crs     = orgCr.stride;
    const Pel*      pCb     = orgCb.buf + y0 * cbs;
    const Pel*      pCr     = orgCr.buf + y0 * crs;
    int64_t         sumCbCr = 0;

    // determine inter-chroma transform sign from correlation between high-pass filtered (i.e., zero-mean) Cb and Cr planes
    for( int y = y0; y < y1; y++, pCb += cbs, pCr += crs )
    {
      for( int x = x0; x < x1; x++ )
      {
        int cb = ( 12*(int)pCb[x] - 2*((int)pCb[x-1] + (int)pCb[x+1] + (int)pCb[x-cbs] + (int)pCb[x+cbs]) - ((int)pCb[x-1-cbs] + (int)pCb[x+1-cbs] + (int)pCb[x-1+cbs] + (int)pCb[x+1+cbs]) );
        int cr = ( 12*(int)pCr[x] - 2*((int)pCr[x-1] + (int)pCr[x+1] + (int)pCr[x-crs] + (int)pCr[x+crs]) - ((int)pCr[x-1-crs] + (int)pCr[x+1-crs] + (int)pCr[x-1+crs] + (int)pCr[x+1+crs]) );
        sumCbCr += cb*cr;
      }
    }

    sgnFlag = ( sumCbCr < 0 );
  }

  cs.slice->picHeader->jointCbCrSign = sgnFlag;
}

struct CtuPos
{
  const int ctuPosX;
  const int ctuPosY;
  const int ctuRsAddr;

  CtuPos( int _x, int _y, int _a ) : ctuPosX( _x ), ctuPosY( _y ), ctuRsAddr( _a ) {}
};

class CtuTsIterator
{
  private:
    const CodingStructure& cs;
    const int        m_startTsAddr;
    const int        m_endTsAddr;
    std::vector<int> m_ctuAddrMap;
          int        m_ctuTsAddr;

  private:
    int getNextTsAddr( const int _tsAddr ) const
    {
      const PreCalcValues& pcv  = *cs.pcv;
      const int startSliceRsRow = m_startTsAddr / pcv.widthInCtus;
      const int startSliceRsCol = m_startTsAddr % pcv.widthInCtus;
      const int endSliceRsRow   = (m_endTsAddr - 1) / pcv.widthInCtus;
      const int endSliceRsCol   = (m_endTsAddr - 1) % pcv.widthInCtus;
            int ctuTsAddr = _tsAddr;
      CHECK( ctuTsAddr > m_endTsAddr, "error: array index out of bounds" );
      while( ctuTsAddr < m_endTsAddr )
      {
        ctuTsAddr++;
        const int ctuRsAddr = ctuTsAddr; 
        if( cs.slice->pps->rectSlice
            && ( (ctuRsAddr / pcv.widthInCtus) < startSliceRsRow
              || (ctuRsAddr / pcv.widthInCtus) > endSliceRsRow
              || (ctuRsAddr % pcv.widthInCtus) < startSliceRsCol
              || (ctuRsAddr % pcv.widthInCtus) > endSliceRsCol ) )
          continue;
        break;
      }
      return ctuTsAddr;
    }

    int mapAddr( const int _addr ) const
    {
      if( _addr < 0 )
        return _addr;
      if( _addr >= m_ctuAddrMap.size() )
        return _addr;
      return m_ctuAddrMap[ _addr ];
    }

  public:
    CtuTsIterator( const CodingStructure& _cs, int _s, int _e,       std::vector<int>& _m         ) : cs( _cs ), m_startTsAddr( _s ), m_endTsAddr( _e ), m_ctuAddrMap( _m ), m_ctuTsAddr( _s ) {}
    CtuTsIterator( const CodingStructure& _cs, int _s, int _e, bool _wpp                          ) : cs( _cs ), m_startTsAddr( _s ), m_endTsAddr( _e ),                     m_ctuTsAddr( _s ) { if( _wpp ) setWppPattern(); }
    CtuTsIterator( const CodingStructure& _cs, int _s, int _e, const std::vector<int>& _m         ) : cs( _cs ), m_startTsAddr( _s ), m_endTsAddr( _e ), m_ctuAddrMap( _m ), m_ctuTsAddr( _s ) {}
    CtuTsIterator( const CodingStructure& _cs, int _s, int _e, const std::vector<int>& _m, int _c ) : cs( _cs ), m_startTsAddr( _s ), m_endTsAddr( _e ), m_ctuAddrMap( _m ), m_ctuTsAddr( std::max( _s, _c ) ) {}
    CtuTsIterator( const CodingStructure& _cs, int _s, int _e, const std::vector<int>* _m, bool _wpp ) : cs( _cs ), m_startTsAddr( _s ), m_endTsAddr( _e ), m_ctuTsAddr( _s ) {  if( _wpp ) m_ctuAddrMap = *_m;  }

    virtual ~CtuTsIterator() { m_ctuAddrMap.clear(); }

    CtuTsIterator& operator++()                { m_ctuTsAddr = getNextTsAddr( m_ctuTsAddr ); return *this; }
    CtuTsIterator  operator++(int)             { auto retval = *this; ++(*this); return retval; }
    bool operator==(CtuTsIterator other) const { return m_ctuTsAddr == other.m_ctuTsAddr; }
    bool operator!=(CtuTsIterator other) const { return m_ctuTsAddr != other.m_ctuTsAddr; }
    CtuPos operator*()                   const { const int ctuRsAddr = mapAddr( m_ctuTsAddr );  return CtuPos( ctuRsAddr % cs.pcv->widthInCtus, ctuRsAddr / cs.pcv->widthInCtus, ctuRsAddr ); }

    CtuTsIterator begin() { return CtuTsIterator( cs, m_startTsAddr, m_endTsAddr, m_ctuAddrMap ); };
    CtuTsIterator end()   { return CtuTsIterator( cs, m_startTsAddr, m_endTsAddr, m_ctuAddrMap, m_endTsAddr ); };

    using iterator_category = std::forward_iterator_tag;
    using value_type        = int;
    using pointer           = int*;
    using reference         = int&;
    using difference_type   = ptrdiff_t;

    void setWppPattern()
    {
      const PreCalcValues& pcv = *cs.pcv;
      m_ctuAddrMap.resize( pcv.sizeInCtus, 0 );
      int addr = 0;
      for( int i = 1; i < pcv.sizeInCtus; i++ )
      {
        int x = addr % pcv.widthInCtus;
        int y = addr / pcv.widthInCtus;
        x -= 1;
        y += 1;
        if( x < 0 || y >= pcv.heightInCtus )
        {
          x += 1 + y;
          y  = 0;
        }
        if( x >= pcv.widthInCtus )
        {
          y += ( x - pcv.widthInCtus ) + 1;
          x  = pcv.widthInCtus - 1;
        }
        addr = y * pcv.widthInCtus + x;
        m_ctuAddrMap[ i ] = addr;
      }
    }
};

void EncSlice::saoDisabledRate( CodingStructure& cs, SAOBlkParam* reconParams )
{
  EncSampleAdaptiveOffset::disabledRate( cs, m_saoDisabledRate, reconParams, m_pcEncCfg->m_saoEncodingRate, m_pcEncCfg->m_saoEncodingRateChroma, m_pcEncCfg->m_internChromaFormat );
}

void EncSlice::finishCompressSlice( Picture* pic, Slice& slice )
{
  CodingStructure& cs = *pic->cs;

  // finalize
  if( slice.sps->saoEnabled && pic->useSAO )
  {
    // store disabled statistics
    if( !m_pcEncCfg->m_numThreads )
      saoDisabledRate( cs, &m_saoReconParams[ 0 ] );

    // set slice header flags
    CHECK( m_saoEnabled[ COMP_Cb ] != m_saoEnabled[ COMP_Cr ], "Unspecified error");
    for( auto s : pic->slices )
    {
      s->saoEnabled[ CH_L ] = m_saoEnabled[ COMP_Y  ];
      s->saoEnabled[ CH_C ] = m_saoEnabled[ COMP_Cb ];
    }
  }
}

void EncSlice::xProcessCtus( Picture* pic, const unsigned startCtuTsAddr, const unsigned boundingCtuTsAddr )
{
  PROFILER_SCOPE_TOP_LEVEL_EXT( 1, g_timeProfiler, P_IGNORE, pic->cs );
  CodingStructure& cs      = *pic->cs;
  Slice&           slice   = *cs.slice;
  const PreCalcValues& pcv = *cs.pcv;

  // initialization
  if( slice.sps->jointCbCr )
  {
    setJointCbCrModes( cs, Position(0, 0), cs.area.lumaSize() );
  }

  if( slice.sps->saoEnabled && pic->useSAO )
  {
    // check SAO enabled or disabled
    EncSampleAdaptiveOffset::decidePicParams( cs, m_saoDisabledRate, m_saoEnabled, m_pcEncCfg->m_saoEncodingRate, m_pcEncCfg->m_saoEncodingRateChroma, m_pcEncCfg->m_internChromaFormat );

    m_saoAllDisabled = true;
    for( int compIdx = 0; compIdx < getNumberValidComponents( pcv.chrFormat ); compIdx++ )
    {
      m_saoAllDisabled &= ! m_saoEnabled[ compIdx ];
    }

    std::fill( m_saoReconParams.begin(), m_saoReconParams.end(), SAOBlkParam() );
  }
  else
  {
    m_saoAllDisabled = true;
  }

  if( slice.sps->alfEnabled )
  {
    m_pALF->initEncProcess( slice );
  }

  std::fill( m_processStates.begin(), m_processStates.end(), CTU_ENCODE );

  // fill encoder parameter list
  int idx = 0;
  const std::vector<int> base = slice.sliceMap.ctuAddrInSlice;
  auto ctuIter = CtuTsIterator( cs, startCtuTsAddr, boundingCtuTsAddr, &m_ctuAddrMap, m_pcEncCfg->m_numThreads > 0 );
  for( auto ctuPos : ctuIter )
  {
    ctuEncParams[ idx ].pic       = pic;
    ctuEncParams[ idx ].encSlice  = this;
    ctuEncParams[ idx ].ctuRsAddr = ctuPos.ctuRsAddr;
    ctuEncParams[ idx ].ctuPosX   = ctuPos.ctuPosX;
    ctuEncParams[ idx ].ctuPosY   = ctuPos.ctuPosY;
    ctuEncParams[ idx ].ctuArea   = UnitArea( pic->chromaFormat, slice.pps->pcv->getCtuArea( ctuPos.ctuPosX, ctuPos.ctuPosY ) );

    if( m_pcEncCfg->m_numThreads > 0 )
    {
      ctuEncParams[idx].tileLineResIdx = slice.pps->getTileLineId( ctuPos.ctuPosX, ctuPos.ctuPosY );
    }
    else
    {
      ctuEncParams[idx].tileLineResIdx = 0;
    }
    idx++;
  }

  //for( int i = 0; i < idx; i++ )
  //{
  //  for( int j = i; j < idx; j++ )
  //  {
  //    if( ctuEncParams[i].tileLineResIdx != ctuEncParams[j].tileLineResIdx ) continue;
  //
  //    CHECK( ctuEncParams[i].ctuPosY != ctuEncParams[j].ctuPosY, "Not the same CTU line!" );
  //    CHECK( slice.pps->getTileIdx( ctuEncParams[i].ctuPosX, ctuEncParams[i].ctuPosY ) != slice.pps->getTileIdx( ctuEncParams[j].ctuPosX, ctuEncParams[j].ctuPosY ), "Not the same tile!" );
  //  }
  //}

  CHECK( idx != pcv.sizeInCtus, "array index out of bounds" );

  // process ctu's until last ctu is done
  if( m_pcEncCfg->m_numThreads > 0 )
  {
    for( auto& ctuEncParam : ctuEncParams )
    {
      m_threadPool->addBarrierTask<CtuEncParam>( EncSlice::xProcessCtuTask<false>,
                                                 &ctuEncParam,
                                                 m_ctuTasksDoneCounter,
                                                 nullptr,
                                                 {},
                                                 EncSlice::xProcessCtuTask<true> );
    }
  }
  else
  {
    do
    {
      for( auto& ctuEncParam : ctuEncParams )
      {
        if( m_processStates[ctuEncParam.ctuRsAddr] != PROCESS_DONE )
          EncSlice::xProcessCtuTask<false>( 0, &ctuEncParam );
      }
      DTRACE_PIC_COMP_COND( m_processStates[ 0 ] == SAO_FILTER && m_processStates[ boundingCtuTsAddr - 1 ] == SAO_FILTER, D_REC_CB_LUMA_LF,   cs, cs.getRecoBuf(), COMP_Y  );
      DTRACE_PIC_COMP_COND( m_processStates[ 0 ] == SAO_FILTER && m_processStates[ boundingCtuTsAddr - 1 ] == SAO_FILTER, D_REC_CB_CHROMA_LF, cs, cs.getRecoBuf(), COMP_Cb );
      DTRACE_PIC_COMP_COND( m_processStates[ 0 ] == SAO_FILTER && m_processStates[ boundingCtuTsAddr - 1 ] == SAO_FILTER, D_REC_CB_CHROMA_LF, cs, cs.getRecoBuf(), COMP_Cr );
      DTRACE_PIC_COMP_COND( m_processStates[ 0 ] == ALF_GET_STATISTICS && m_processStates[ boundingCtuTsAddr - 1 ] == ALF_GET_STATISTICS, D_REC_CB_LUMA_SAO,   cs, cs.getRecoBuf(), COMP_Y  );
      DTRACE_PIC_COMP_COND( m_processStates[ 0 ] == ALF_GET_STATISTICS && m_processStates[ boundingCtuTsAddr - 1 ] == ALF_GET_STATISTICS, D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMP_Cb );
      DTRACE_PIC_COMP_COND( m_processStates[ 0 ] == ALF_GET_STATISTICS && m_processStates[ boundingCtuTsAddr - 1 ] == ALF_GET_STATISTICS, D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMP_Cr );
    }
    while( m_processStates[ boundingCtuTsAddr - 1 ] != PROCESS_DONE );
  }
}

inline bool checkCtuTaskNbTop( const PPS& pps, const int& ctuPosX, const int& ctuPosY, const int& ctuRsAddr, const ProcessCtuState* processStates, const TaskType tskType, bool override = false )
{
  return ctuPosY > 0 && ( override || pps.canFilterCtuBdry( ctuPosX, ctuPosY, 0, -1 ) ) && processStates[ ctuRsAddr - pps.pcv->widthInCtus ] <= tskType;
}

inline bool checkCtuTaskNbBot( const PPS& pps, const int& ctuPosX, const int& ctuPosY, const int& ctuRsAddr, const ProcessCtuState* processStates, const TaskType tskType, bool override = false )
{
  return ctuPosY + 1 < pps.pcv->heightInCtus && ( override || pps.canFilterCtuBdry( ctuPosX, ctuPosY, 0, 1 ) ) && processStates[ ctuRsAddr     + pps.pcv->widthInCtus ] <= tskType;
}

inline bool checkCtuTaskNbRgt( const PPS& pps, const int& ctuPosX, const int& ctuPosY, const int& ctuRsAddr, const ProcessCtuState* processStates, const TaskType tskType, bool override = false )
{
  return ctuPosX + 1 < pps.pcv->widthInCtus && ( override || pps.canFilterCtuBdry( ctuPosX, ctuPosY, 1, 0 ) ) && processStates[ ctuRsAddr + 1 ] <= tskType;
}

inline bool checkCtuTaskNbTopRgt( const PPS& pps, const int& ctuPosX, const int& ctuPosY, const int& ctuRsAddr, const ProcessCtuState* processStates, const TaskType tskType, bool override = false )
{
  return ctuPosY > 0 && ctuPosX + 1 < pps.pcv->widthInCtus && ( override || pps.canFilterCtuBdry( ctuPosX, ctuPosY, 1, -1 ) ) && processStates[ ctuRsAddr - pps.pcv->widthInCtus + 1 ] <= tskType;
}

inline bool checkCtuTaskNbBotRgt( const PPS& pps, const int& ctuPosX, const int& ctuPosY, const int& ctuRsAddr, const ProcessCtuState* processStates, const TaskType tskType, const int rightOffset = 1, bool override = false )
{
  return ctuPosX + rightOffset < pps.pcv->widthInCtus && ctuPosY + 1 < pps.pcv->heightInCtus && ( override || pps.canFilterCtuBdry( ctuPosX, ctuPosY, rightOffset, 1 ) ) && processStates[ ctuRsAddr + rightOffset + pps.pcv->widthInCtus ] <= tskType;
}

template<bool checkReadyState>
bool EncSlice::xProcessCtuTask( int threadIdx, CtuEncParam* ctuEncParam )
{
  Picture* pic                   = ctuEncParam->pic;
  EncSlice* encSlice             = ctuEncParam->encSlice;
  CodingStructure& cs            = *pic->cs;
  Slice&           slice         = *cs.slice;
  const PPS&       pps           = *slice.pps;
  const PreCalcValues& pcv       = *cs.pcv;
  const int ctuRsAddr            = ctuEncParam->ctuRsAddr;
  const int ctuPosX              = ctuEncParam->ctuPosX;
  const int ctuPosY              = ctuEncParam->ctuPosY;
  const int x                    = ctuPosX << pcv.maxCUSizeLog2;
  const int y                    = ctuPosY << pcv.maxCUSizeLog2;
  const int width                = std::min( pcv.maxCUSize, pcv.lumaWidth  - x );
  const int height               = std::min( pcv.maxCUSize, pcv.lumaHeight - y );
  const int ctuStride            = pcv.widthInCtus;
  const int lineIdx              = ctuEncParam->tileLineResIdx;
  ProcessCtuState* processStates = encSlice->m_processStates.data();
  const UnitArea& ctuArea        = ctuEncParam->ctuArea;
  const bool wppSyncEnabled      = cs.sps->entropyCodingSyncEnabled;
  const TaskType currState       = processStates[ ctuRsAddr ];
  const unsigned syncLines       = encSlice->m_pcEncCfg->m_ifpLines;

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "poc", cs.slice->poc ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", processStates[ ctuRsAddr ] == CTU_ENCODE ? 0 : 1 ) );

  // process ctu's line wise from left to right
  const bool tileParallel = encSlice->m_pcEncCfg->m_tileParallelCtuEnc;
  if( tileParallel && currState == CTU_ENCODE && ctuPosX > 0 && slice.pps->getTileIdx( ctuPosX, ctuPosY ) != slice.pps->getTileIdx( ctuPosX - 1, ctuPosY ) )
    ; // for CTU_ENCODE on tile boundaries, allow parallel processing of tiles
  else if( ctuPosX > 0 && processStates[ ctuRsAddr - 1 ] <= currState && currState < PROCESS_DONE )
    return false;

  switch( currState )
  {
    // encode
    case CTU_ENCODE:
      {
        // CTU line-wise inter-frame parallel processing synchronization
        if( syncLines )
        {
          const bool lineStart = ctuPosX == 0 || ( tileParallel && slice.pps->getTileIdx( ctuPosX, ctuPosY ) != slice.pps->getTileIdx( ctuPosX - 1, ctuPosY ) );
          if( lineStart && !refPicCtuLineReady( slice, ctuPosY + (int)syncLines, pcv ) )
          {
            return false;
          }
        }

        // general wpp conditions, top and top-right ctu have to be encoded
        if( encSlice->m_pcEncCfg->m_tileParallelCtuEnc && ctuPosY > 0 && slice.pps->getTileIdx( ctuPosX, ctuPosY ) != slice.pps->getTileIdx( ctuPosX, ctuPosY - 1 ) )
          ; // allow parallel processing of CTU-encoding on independent tiles
        else if( ctuPosY > 0                                  && processStates[ ctuRsAddr - ctuStride     ] <= CTU_ENCODE )
          return false;
        else if( ctuPosY > 0 && ctuPosX + 1 < pcv.widthInCtus && processStates[ ctuRsAddr - ctuStride + 1 ] <= CTU_ENCODE && !wppSyncEnabled )
          return false;
        
        if( checkReadyState )
          return true;

#ifdef TRACE_ENABLE_ITT
        std::stringstream ss;
        ss << "Encode_" << slice.poc << "_CTU_" << ctuPosY << "_" << ctuPosX;
        __itt_string_handle* itt_handle_ctuEncode = __itt_string_handle_create( ss.str().c_str() );
#endif
        ITT_TASKSTART( itt_domain_encode, itt_handle_ctuEncode );

        TileLineEncRsrc* lineEncRsrc = encSlice->m_TileLineEncRsrc[ lineIdx ];
        PerThreadRsrc* taskRsrc      = encSlice->m_ThreadRsrc[ threadIdx ];
        EncCu& encCu                 = taskRsrc->m_encCu;

        encCu.setCtuEncRsrc( &lineEncRsrc->m_CABACEstimator, &taskRsrc->m_CtxCache, &lineEncRsrc->m_ReuseUniMv, &lineEncRsrc->m_BlkUniMvInfoBuffer, &lineEncRsrc->m_AffineProfList, &lineEncRsrc->m_CachedBvs );
        encCu.encodeCtu( pic, lineEncRsrc->m_prevQp, ctuPosX, ctuPosY );

        // cleanup line memory when last ctu in line done to reduce overall memory consumption
        if( encSlice->m_pcEncCfg->m_ensureWppBitEqual && ( ctuPosX == pcv.widthInCtus - 1 || slice.pps->getTileIdx( ctuPosX, ctuPosY ) != slice.pps->getTileIdx( ctuPosX + 1, ctuPosY ) ) )
        {
          lineEncRsrc->m_AffineProfList    .resetAffineMVList();
          lineEncRsrc->m_BlkUniMvInfoBuffer.resetUniMvList();
          lineEncRsrc->m_ReuseUniMv        .resetReusedUniMvs();
          lineEncRsrc->m_CachedBvs         .resetIbcBvCand();
        }

        DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 1 ) );
        ITT_TASKEND( itt_domain_encode, itt_handle_ctuEncode );

        processStates[ ctuRsAddr ] = RESHAPE_LF_VER;
      }
      break;

    // reshape + vertical loopfilter
    case RESHAPE_LF_VER:
      {
        // clip check to right tile border (CTU_ENCODE pre-processing delay due to IBC)
        const int tileCol = slice.pps->ctuToTileCol[ctuPosX];
        const int lastCtuPosXInTile = slice.pps->tileColBd[tileCol] + slice.pps->tileColWidth[tileCol] - 1;
        const int checkRight = std::min<int>( encSlice->m_ctuEncDelay, lastCtuPosXInTile - ctuPosX );

        const bool hasTiles = encSlice->m_pcEncCfg->m_tileParallelCtuEnc && slice.pps->getNumTiles() > 1;

        // need to check line above bcs of tiling, which allows CTU_ENCODE to run independently across tiles
        if( hasTiles )
        {
          if( ctuPosY > 0 )
          {
            for( int i = -!!ctuPosX; i <= checkRight; i++ )
              if( pps.canFilterCtuBdry( ctuPosX, ctuPosY, i, -1 ) && processStates[ctuRsAddr - ctuStride + i] <= CTU_ENCODE )
                return false;
          }
        }
        
        // ensure all surrounding ctu's are encoded (intra pred requires non-reshaped and unfiltered residual, IBC requires unfiltered samples too)
        // check right with max offset (due to WPP condition above, this implies top-right has been already encoded)
        for( int i = hasTiles ? -!!ctuPosX : checkRight; i <= checkRight; i++ )
          if( pps.canFilterCtuBdry( ctuPosX, ctuPosY, i, 0 ) && processStates[ctuRsAddr + i] <= CTU_ENCODE )
            return false;

        // check bottom right with 1 CTU delay (this is only required for intra pred)
        // at the right picture border this will check the bottom CTU
        const int checkBottomRight = std::min<int>( 1, lastCtuPosXInTile - ctuPosX );
        if( checkCtuTaskNbBotRgt( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, CTU_ENCODE, checkBottomRight ) ) 
          return false;

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_rspLfVer );

        // reshape
        if( slice.sps->lumaReshapeEnable && slice.picHeader->lmcsEnabled )
        {
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_RESHAPER, &cs, CH_L );
          PelBuf reco = pic->getRecoBuf( COMP_Y ).subBuf( x, y, width, height );
          reco.rspSignal( pic->reshapeData.getInvLUT() );
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_IGNORE, &cs, CH_L );
        }

        // loopfilter
        if( !cs.pps->deblockingFilterControlPresent || !cs.pps->deblockingFilterDisabled || cs.pps->deblockingFilterOverrideEnabled )
        {
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_DEBLOCK_FILTER, &cs, CH_L );
          // calculate filter strengths
          encSlice->m_pLoopFilter->calcFilterStrengthsCTU( cs, ctuArea, true );

          // vertical filter
          PelUnitBuf reco = cs.picture->getRecoBuf();
          encSlice->m_pLoopFilter->xDeblockArea<EDGE_VER>( cs, ctuArea, MAX_NUM_CH, reco );
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_IGNORE, &cs, CH_L );
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_rspLfVer );

        processStates[ ctuRsAddr ] = LF_HOR;
      }
      break;

    // horizontal loopfilter
    case LF_HOR:
      {
        // ensure horizontal ordering (from top to bottom)
        if( checkCtuTaskNbTop   ( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, LF_HOR ) )         
          return false;

        // ensure vertical loop filter of neighbor ctu's will not modify current residual
        // check top, top-right and right ctu
        // (top, top-right checked implicitly due to ordering check above)
        if( checkCtuTaskNbRgt   ( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, RESHAPE_LF_VER ) ) 
          return false;

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_lfHor );

        if( !cs.pps->deblockingFilterControlPresent || !cs.pps->deblockingFilterDisabled || cs.pps->deblockingFilterOverrideEnabled )
        {
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_DEBLOCK_FILTER, &cs, CH_L );
          PelUnitBuf reco = cs.picture->getRecoBuf();
          encSlice->m_pLoopFilter->xDeblockArea<EDGE_HOR>( cs, ctuArea, MAX_NUM_CH, reco );
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_IGNORE, &cs, CH_L );
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_lfHor );

        processStates[ ctuRsAddr ] = SAO_FILTER;
      }
      break;

    // SAO filter
    case SAO_FILTER:
      {
        // general wpp conditions, top and top-right ctu have to be filtered
        if( checkCtuTaskNbTop   ( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, SAO_FILTER, true ) ) return false;
        if( checkCtuTaskNbTopRgt( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, SAO_FILTER, true ) ) return false;

        // ensure loop filter of neighbor ctu's will not modify current residual
        // sao processing dependents on +1 pixel to each side
        // due to wpp condition above, only right, bottom and bottom-right ctu have to be checked
        if( checkCtuTaskNbRgt   ( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, LF_HOR,    true ) ) return false;
        if( checkCtuTaskNbBot   ( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, LF_HOR,    true ) ) return false;
        if( checkCtuTaskNbBotRgt( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, LF_HOR, 1, true ) ) return false;

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_sao );

        // SAO filter
        if( slice.sps->saoEnabled && pic->useSAO )
        {
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_SAO, &cs, CH_L );
          TileLineEncRsrc* lineEncRsrc    = encSlice->m_TileLineEncRsrc[ lineIdx ];
          PerThreadRsrc* taskRsrc         = encSlice->m_ThreadRsrc[ threadIdx ];
          EncSampleAdaptiveOffset& encSao = lineEncRsrc->m_encSao;

          encSao.setCtuEncRsrc( &lineEncRsrc->m_SaoCABACEstimator, &taskRsrc->m_CtxCache );
          encSao.storeCtuReco( cs, ctuArea, ctuPosX, ctuPosY );
          encSao.getCtuStatistics( cs, encSlice->m_saoStatData, ctuArea, ctuRsAddr );
          encSao.decideCtuParams( cs, encSlice->m_saoStatData, encSlice->m_saoEnabled, encSlice->m_saoAllDisabled, ctuArea, ctuRsAddr, &encSlice->m_saoReconParams[ 0 ], cs.picture->getSAO() );
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_IGNORE, &cs, CH_L );
        }

        // ALF border extension
        if( cs.sps->alfEnabled )
        {
          // we have to do some kind of position aware boundary padding
          // it's done here because the conditions are readable
          PelUnitBuf recoBuf = cs.picture->getRecoBuf();
          const int fltSize  = ( MAX_ALF_FILTER_LENGTH + 1 ) >> 1;
          const int xL       = ( ctuPosX == 0 )                 ? ( x-fltSize       ) : ( x );
          const int xR       = ( ctuPosX+1 == pcv.widthInCtus ) ? ( x+width+fltSize ) : ( x+width );

          if( ctuPosX == 0 )                  recoBuf.extendBorderPelLft( y, height, fltSize );
          if( ctuPosX+1 == pcv.widthInCtus )  recoBuf.extendBorderPelRgt( y, height, fltSize );
          if( ctuPosY == 0 )                  recoBuf.extendBorderPelTop( xL, xR-xL, fltSize );
          if( ctuPosY+1 == pcv.heightInCtus ) recoBuf.extendBorderPelBot( xL, xR-xL, fltSize );

          encSlice->m_pALF->copyCTUforALF(cs, ctuPosX, ctuPosY);
        }

        // DMVR refinement can be stored now
        if( slice.sps->DMVR && !slice.picHeader->disDmvrFlag )
        {
          CS::setRefinedMotionFieldCTU( cs, ctuPosX, ctuPosY );
        }
        ITT_TASKEND( itt_domain_encode, itt_handle_sao );

        const int tileCol = slice.pps->ctuToTileCol[ctuPosX];
        const int lastCtuColInTileRow = slice.pps->tileColBd[tileCol] + slice.pps->tileColWidth[tileCol] - 1;
        if( ctuPosX == lastCtuColInTileRow )
        {
          processStates[ctuRsAddr] = ALF_GET_STATISTICS;
        }
        else
        {
          processStates[ctuRsAddr] = PROCESS_DONE;
          return true;
        }
      }
      break;

    case ALF_GET_STATISTICS:
      {
        // ensure all surrounding ctu's are filtered (ALF will use pixels of adjacent CTU's)
        // due to wpp condition above in SAO_FILTER, only right, bottom and bottom-right ctu have to be checked
        if( checkCtuTaskNbRgt   ( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, SAO_FILTER ) ) return false;
        if( checkCtuTaskNbBot   ( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, SAO_FILTER ) ) return false;
        if( checkCtuTaskNbBotRgt( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, SAO_FILTER ) ) return false;

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_alf_stat );

        // ALF pre-processing
        if( slice.sps->alfEnabled )
        {
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_ALF, &cs, CH_L );
          PelUnitBuf recoBuf = cs.picture->getRecoBuf();
          const int firstCtuInRow = ctuRsAddr + 1 - slice.pps->tileColWidth[slice.pps->ctuToTileCol[ctuPosX]];
          for( int ctu = firstCtuInRow; ctu <= ctuRsAddr; ctu++ )
          {
            encSlice->m_pALF->getStatisticsCTU( *cs.picture, cs, recoBuf, ctu, encSlice->m_ThreadRsrc[ threadIdx ]->m_alfTempCtuBuf );
          }
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_IGNORE, &cs, CH_L );
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_alf_stat );

        // start alf filter derivation either for a sub-set of CTUs (syncLines mode) or for the whole picture (regular mode)
        const unsigned deriveFilterCtu = encSlice->m_alfDeriveCtu;
        processStates[ctuRsAddr] = (ctuRsAddr < deriveFilterCtu) ? ALF_RECONSTRUCT: ALF_DERIVE_FILTER;
      }
      break;

    case ALF_DERIVE_FILTER:
      {
        const unsigned deriveFilterCtu = encSlice->m_alfDeriveCtu;
        if( ctuRsAddr == deriveFilterCtu )
        {
          // ensure statistics from all previous ctu's have been collected
          for( int y = 0; y <= encSlice->m_alfDeriveLine; y++ )
          {
            for( int tileCol = 0; tileCol < slice.pps->numTileCols; tileCol++ )
            {
              const int lastCtuInTileRow = y * pcv.widthInCtus + slice.pps->tileColBd[tileCol] + slice.pps->tileColWidth[tileCol] - 1;
              if( processStates[lastCtuInTileRow] <= ALF_GET_STATISTICS )
                return false;
            }
          }
        }
        else if( syncLines )
        {
          // ALF bitstream coding dependency for the sub-sequent ctu-lines
          if( processStates[deriveFilterCtu] < ALF_RECONSTRUCT || checkCtuTaskNbTop( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, ALF_DERIVE_FILTER ) ) 
            return false;
        }
        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_alf_derive );
        // ALF post-processing
        if( slice.sps->alfEnabled )
        {
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_ALF, &cs, CH_L );
          if( ctuRsAddr == deriveFilterCtu )
          {
            encSlice->m_pALF->initDerivation( slice );
            encSlice->m_pALF->deriveFilter( *cs.picture, cs, slice.getLambdas(), deriveFilterCtu + 1 );
            encSlice->m_pALF->reconstructCoeffAPSs( cs, cs.slice->alfEnabled[COMP_Y], cs.slice->alfEnabled[COMP_Cb] || cs.slice->alfEnabled[COMP_Cr], false );
          }
          else if( syncLines )
          {
            // in sync lines mode: derive/select filter for the remaining lines
            TileLineEncRsrc* lineEncRsrc = encSlice->m_TileLineEncRsrc[ lineIdx ];
            PerThreadRsrc*   taskRsrc    = encSlice->m_ThreadRsrc[ threadIdx ];
            const int firstCtuInRow = ctuRsAddr + 1 - slice.pps->tileColWidth[slice.pps->ctuToTileCol[ctuPosX]];
            for(int ctu = firstCtuInRow; ctu <= ctuRsAddr; ctu++)
            {
              encSlice->m_pALF->selectFilterForCTU( cs, &lineEncRsrc->m_AlfCABACEstimator, &taskRsrc->m_CtxCache, ctu );
            }
          }
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_IGNORE, &cs, CH_L );
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_alf_derive );
        processStates[ ctuRsAddr ] = ALF_RECONSTRUCT;
      }
      break;

    case ALF_RECONSTRUCT:
      {
        // start alf filter derivation either for a sub-set of CTUs (syncLines mode) or for the whole picture (regular mode)
        const unsigned deriveFilterCtu = encSlice->m_alfDeriveCtu;
        if( processStates[deriveFilterCtu] < ALF_RECONSTRUCT )
          return false;
        else if( syncLines && ctuRsAddr > deriveFilterCtu && encSlice->m_pALF->getAsuHeightInCtus() > 1 )
        {
          const int asuHeightInCtus = encSlice->m_pALF->getAsuHeightInCtus();
          const int botCtuLineInAsu = std::min( (( ctuPosY & ( ~(asuHeightInCtus - 1) ) ) + asuHeightInCtus - 1), (int)pcv.heightInCtus - 1 );
          if( processStates[botCtuLineInAsu * ctuStride + ctuPosX] < ALF_RECONSTRUCT ) 
            return false;
        }

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_alf_recon );

        if( slice.sps->alfEnabled )
        {
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_ALF, &cs, CH_L );
          const int firstCtuInRow = ctuRsAddr + 1 - slice.pps->tileColWidth[slice.pps->ctuToTileCol[ctuPosX]];
          for( int ctu = firstCtuInRow; ctu <= ctuRsAddr; ctu++ )
          {
            encSlice->m_pALF->reconstructCTU_MT( *cs.picture, cs, ctu, encSlice->m_ThreadRsrc[ threadIdx ]->m_alfTempCtuBuf );
          }
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_IGNORE, &cs, CH_L );
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_alf_recon );
        processStates[ctuRsAddr] = CCALF_GET_STATISTICS;
      }
      // dont break, no additional deps, can continue straigt away!
      //break;

    case CCALF_GET_STATISTICS:
      {
        if( checkCtuTaskNbTop   ( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, ALF_RECONSTRUCT ) ) return false;
        if( checkCtuTaskNbBot   ( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, ALF_RECONSTRUCT ) ) return false;

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_ccalf_stat );

        // ALF pre-processing
        if( slice.sps->ccalfEnabled )
        {
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_ALF, &cs, CH_L);
          const int firstCtuInRow = ctuRsAddr + 1 - slice.pps->tileColWidth[slice.pps->ctuToTileCol[ctuPosX]];
          for( int ctu = firstCtuInRow; ctu <= ctuRsAddr; ctu++ )
          {
            encSlice->m_pALF->deriveStatsForCcAlfFilteringCTU( cs, COMP_Cb, ctu, encSlice->m_ThreadRsrc[ threadIdx ]->m_alfTempCtuBuf );
            encSlice->m_pALF->deriveStatsForCcAlfFilteringCTU( cs, COMP_Cr, ctu, encSlice->m_ThreadRsrc[ threadIdx ]->m_alfTempCtuBuf );
          }
          PROFILER_EXT_ACCUM_AND_START_NEW_SET( 1, _TPROF, P_IGNORE, &cs, CH_L );
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_ccalf_stat );

        // start alf filter derivation either for a sub-set of CTUs (syncLines mode) or for the whole picture (regular mode)
        const unsigned deriveFilterCtu = syncLines ? pcv.widthInCtus * std::min(syncLines + 1, pcv.heightInCtus) - 1: pcv.sizeInCtus - 1;
        processStates[ctuRsAddr] = (ctuRsAddr < deriveFilterCtu) ? CCALF_RECONSTRUCT: CCALF_DERIVE_FILTER;
      }
      break;

    case CCALF_DERIVE_FILTER:
      {
        // synchronization dependencies
        const unsigned deriveFilterCtu = syncLines ? pcv.widthInCtus * std::min(syncLines + 1, pcv.heightInCtus) - 1: pcv.sizeInCtus - 1;
        if( ctuRsAddr == deriveFilterCtu )
        {
          // ensure statistics from all previous ctu's have been collected
          int numCheckLines = syncLines ? std::min(syncLines + 1, pcv.heightInCtus): pcv.heightInCtus;
          for( int y = 0; y < numCheckLines; y++ )
          {
            for( int tileCol = 0; tileCol < slice.pps->numTileCols; tileCol++ )
            {
              const int lastCtuInTileRow = y * pcv.widthInCtus + slice.pps->tileColBd[tileCol] + slice.pps->tileColWidth[tileCol] - 1;
              if( processStates[lastCtuInTileRow] <= CCALF_GET_STATISTICS )
                return false;
            }
          }
        }
        else if( syncLines )
        {
          // ALF bitstream coding dependency for the sub-sequent CTU-lines
          if( processStates[deriveFilterCtu] < CCALF_RECONSTRUCT || checkCtuTaskNbTop( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, CCALF_DERIVE_FILTER ) ) 
            return false;
        }
        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_ccalf_derive );

        // start task
        if( slice.sps->ccalfEnabled )
        {
          if( ctuRsAddr == deriveFilterCtu )
          {
            encSlice->m_pALF->deriveCcAlfFilter( *cs.picture, cs, syncLines ? pcv.widthInCtus * std::min(syncLines + 1, pcv.heightInCtus): pcv.sizeInCtus );
          }
          else if( syncLines )
          {
            // in sync lines mode: derive/select filter for the remaining lines
            TileLineEncRsrc* lineEncRsrc = encSlice->m_TileLineEncRsrc[ lineIdx ];
            PerThreadRsrc*   taskRsrc    = encSlice->m_ThreadRsrc[ threadIdx ];
            const int firstCtuInRow = ctuRsAddr + 1 - slice.pps->tileColWidth[slice.pps->ctuToTileCol[ctuPosX]];
            encSlice->m_pALF->selectCcAlfFilterForCtuLine( cs, COMP_Cb, cs.getRecoBuf(), &lineEncRsrc->m_AlfCABACEstimator, &taskRsrc->m_CtxCache, firstCtuInRow, ctuRsAddr );
            encSlice->m_pALF->selectCcAlfFilterForCtuLine( cs, COMP_Cr, cs.getRecoBuf(), &lineEncRsrc->m_AlfCABACEstimator, &taskRsrc->m_CtxCache, firstCtuInRow, ctuRsAddr );
          }
        }
        ITT_TASKEND( itt_domain_encode, itt_handle_ccalf_derive );

        processStates[ctuRsAddr] = CCALF_RECONSTRUCT;
      }
      break;

    case CCALF_RECONSTRUCT:
      {
        // start ccalf filter derivation either for a sub-set of CTUs (syncLines mode) or for the whole picture (regular mode)
        const unsigned deriveFilterCtu = syncLines ? pcv.widthInCtus * std::min(syncLines + 1, pcv.heightInCtus) - 1: pcv.sizeInCtus - 1;
        if( processStates[deriveFilterCtu] < CCALF_RECONSTRUCT )
          return false;

        if( syncLines )
        {
          // ensure line-by-line reconstruction due to line synchronization
          if( checkCtuTaskNbTop( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, CCALF_RECONSTRUCT ) ) return false;
          // check bottom due to rec. buffer usage in ccalf statistics
          if( checkCtuTaskNbBot( pps, ctuPosX, ctuPosY, ctuRsAddr, processStates, CCALF_GET_STATISTICS ) ) return false;
        }

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_ccalf_recon );

        if( slice.sps->ccalfEnabled )
        {
          const int firstCtuInRow = ctuRsAddr + 1 - slice.pps->tileColWidth[slice.pps->ctuToTileCol[ctuPosX]];
          for( int ctu = firstCtuInRow; ctu <= ctuRsAddr; ctu++ )
          {
            encSlice->m_pALF->applyCcAlfFilterCTU( cs, COMP_Cb, ctu, encSlice->m_ThreadRsrc[ threadIdx ]->m_alfTempCtuBuf );
            encSlice->m_pALF->applyCcAlfFilterCTU( cs, COMP_Cr, ctu, encSlice->m_ThreadRsrc[ threadIdx ]->m_alfTempCtuBuf );
          }
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_ccalf_recon );

        // extend pic border
        // CCALF reconstruction stage is done per tile, ensure that all tiles in current CTU row are done  
        if( ++(pic->m_tileColsDone->at(ctuPosY)) >= pps.numTileCols )
        {
          PelUnitBuf recoBuf = cs.picture->getRecoBuf();
          const int margin = cs.picture->margin;
          recoBuf.extendBorderPelLft( y, height, margin );
          recoBuf.extendBorderPelRgt( y, height, margin );
          if(ctuPosY == 0)
            recoBuf.extendBorderPelTop( -margin, pcv.lumaWidth + 2 * margin, margin );
          if(ctuPosY + 1 == pcv.heightInCtus)
            recoBuf.extendBorderPelBot( -margin, pcv.lumaWidth + 2 * margin, margin );

          // for IFP lines synchro, do an additional increment signaling that CTU row is ready
          if( syncLines )
            ++(pic->m_tileColsDone->at( ctuPosY ));
        }

        // perform finish only once for whole picture
        const unsigned finishCtu = pcv.sizeInCtus - 1;
        if( ctuRsAddr < finishCtu )
        {
          processStates[ctuRsAddr] = PROCESS_DONE;
          // processing done => terminate thread
          return true;
        }
        processStates[ctuRsAddr] = FINISH_SLICE;
      }

    case FINISH_SLICE:
      {
        CHECK( ctuRsAddr != pcv.sizeInCtus - 1, "invalid state, finish slice only once for last ctu" );

        // ensure all coding tasks have been done for all previous ctu's
        for( int i = 0; i < ctuRsAddr; i++ )
          if( processStates[ i ] < FINISH_SLICE )
            return false;

        if( checkReadyState )
          return true;

        encSlice->finishCompressSlice( cs.picture, slice );

        processStates[ ctuRsAddr ] = PROCESS_DONE;
        // processing done => terminate thread
        return true;
      }

    case PROCESS_DONE:
      CHECK( true, "process state is PROCESS_DONE, but thread is still running" );
      return true;

    default:
      CHECK( true, "unknown process state" );
      return true;
  }

  return false;
}

void EncSlice::encodeSliceData( Picture* pic )
{
  CodingStructure& cs              = *pic->cs;
  Slice* const slice               = cs.slice;
  const uint32_t startCtuTsAddr    = slice->sliceMap.ctuAddrInSlice[0];
  const uint32_t boundingCtuTsAddr = cs.pcv->sizeInCtus;
  const bool wavefrontsEnabled     = slice->sps->entropyCodingSyncEnabled;

  // this ensures that independently encoded bitstream chunks can be combined to bit-equal
  const SliceType cabacTableIdx = ! slice->pps->cabacInitPresent || slice->pendingRasInit ? slice->sliceType : m_encCABACTableIdx;
  slice->encCABACTableIdx = cabacTableIdx;

  // initialise entropy coder for the slice
  m_CABACWriter.initCtxModels( *slice );

  DTRACE( g_trace_ctx, D_HEADER, "=========== POC: %d ===========\n", slice->poc );

  int prevQP[MAX_NUM_CH];
  prevQP[0] = prevQP[1] = slice->sliceQp;

  const PreCalcValues& pcv        = *cs.pcv;
  const uint32_t widthInCtus      = pcv.widthInCtus;
  uint32_t uiSubStrm              = 0;
  const int numSubstreamsColumns  = slice->pps->numTileCols;
  const int numSubstreamRows      = slice->sps->entropyCodingSyncEnabled ? pic->cs->pcv->heightInCtus : slice->pps->numTileRows;
  const int numSubstreams         = std::max<int>( numSubstreamRows * numSubstreamsColumns, 0/*(int)pic->brickMap->bricks.size()*/ );
  std::vector<OutputBitstream> substreamsOut( numSubstreams );

  slice->clearSubstreamSizes();

  for( uint32_t ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++ )
  {
    const uint32_t ctuRsAddr            = slice->sliceMap.ctuAddrInSlice[ctuTsAddr];
    const uint32_t ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus        = ctuRsAddr / widthInCtus;
    const uint32_t tileXPosInCtus       = slice->pps->tileColBd[cs.pps->ctuToTileCol[ctuXPosInCtus]];
    const uint32_t tileYPosInCtus       = slice->pps->tileRowBd[cs.pps->ctuToTileRow[ctuYPosInCtus]];

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    const Position pos (ctuXPosInCtus * pcv.maxCUSize, ctuYPosInCtus * pcv.maxCUSize);
    const UnitArea ctuArea (cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUSize, pcv.maxCUSize));
    CHECK( uiSubStrm >= numSubstreams, "array index out of bounds" );
    m_CABACWriter.initBitstream( &substreamsOut[ uiSubStrm ] );

    // set up CABAC contexts' state for this CTU
    if (ctuXPosInCtus == tileXPosInCtus && ctuYPosInCtus == tileYPosInCtus )
    {
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_CABACWriter.initCtxModels( *slice );
      }
      prevQP[0] = prevQP[1] = slice->sliceQp;
    }
    else if (ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled)
    {
      // Synchronize cabac probabilities with upper-right CTU if it's available and at the start of a line.
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_CABACWriter.initCtxModels( *slice );
      }
      if( cs.getCURestricted( pos.offset( 0, -1 ), pos, slice->independentSliceIdx, slice->pps->getTileIdx( ctuXPosInCtus, ctuYPosInCtus ), CH_L, TREE_D ) )
      {
        // Top-right is available, so use it.
        m_CABACWriter.getCtx() = m_entropyCodingSyncContextState;
      }
      prevQP[0] = prevQP[1] = slice->sliceQp;
    }

    m_CABACWriter.coding_tree_unit( cs, ctuArea, prevQP, ctuRsAddr );

    // store probabilities of second CTU in line into buffer
    if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState = m_CABACWriter.getCtx();
    }

    // terminate the sub-stream, if required (end of slice-segment, end of tile, end of wavefront-CTU-row):
    bool isMoreCTUsinSlice = ctuTsAddr != (boundingCtuTsAddr - 1);
    bool isLastCTUinTile   = isMoreCTUsinSlice && slice->pps->getTileIdx( ctuRsAddr ) != slice->pps->getTileIdx( slice->sliceMap.ctuAddrInSlice[ctuTsAddr+1] );
    bool isLastCTUinWPP    = wavefrontsEnabled && isMoreCTUsinSlice && !isLastCTUinTile && ( (slice->sliceMap.ctuAddrInSlice[ctuTsAddr+1] % widthInCtus) == cs.pps->tileColBd[cs.pps->ctuToTileCol[slice->sliceMap.ctuAddrInSlice[ctuTsAddr+1] % widthInCtus]] ); //TODO: adjust tile bound condition

    if (isLastCTUinWPP || !isMoreCTUsinSlice || isLastCTUinTile )         // this the the last CTU of either tile/brick/WPP/slice
    {
      m_CABACWriter.end_of_slice();

      // Byte-alignment in slice_data() when new tile
      substreamsOut[ uiSubStrm ].writeByteAlignment();

      if (isMoreCTUsinSlice) //Byte alignment only when it is not the last substream in the slice
      {
        // write sub-stream size
        slice->addSubstreamSize( ( substreamsOut[ uiSubStrm ].getNumberOfWrittenBits() >> 3 ) + substreamsOut[ uiSubStrm ].countStartCodeEmulations() );
      }
      uiSubStrm++;
    }
  } // CTU-loop

  if(slice->pps->cabacInitPresent)
  {
    m_encCABACTableIdx = m_CABACWriter.getCtxInitId( *slice );
  }
  else
  {
    m_encCABACTableIdx = slice->sliceType;
  }

  // concatenate substreams
  OutputBitstream& outStream = pic->sliceDataStreams[ 0/*slice->sliceIdx*/ ];
  for ( int i = 0; i < slice->getNumberOfSubstreamSizes() + 1; i++ )
  {
    outStream.addSubstream( &(substreamsOut[ i ]) );
  }
  pic->sliceDataNumBins += m_CABACWriter.getNumBins();
}

} // namespace vvenc

//! \}


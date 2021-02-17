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


/** \file     EncSlice.cpp
    \brief    slice encoder class
*/

#include "EncSlice.h"
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
static const __itt_domain* itt_domain_encode            = __itt_domain_create( "Encode" );
static const __itt_string_handle* itt_handle_ctuEncode  = __itt_string_handle_create( "Encode_CTU" );
static const __itt_string_handle* itt_handle_rspLfVer   = __itt_string_handle_create( "RspLfVer_CTU" );
static const __itt_string_handle* itt_handle_lfHor      = __itt_string_handle_create( "LfHor_CTU" );
static const __itt_string_handle* itt_handle_sao        = __itt_string_handle_create( "SAO_CTU" );
static const __itt_string_handle* itt_handle_alf_stat   = __itt_string_handle_create( "ALF_CTU_STAT" );
static const __itt_string_handle* itt_handle_alf_derive = __itt_string_handle_create( "ALF_DERIVE" );
static const __itt_string_handle* itt_handle_alf_recon  = __itt_string_handle_create( "ALF_RECONSTRUCT" );
#endif

struct LineEncRsrc
{
  BitEstimator            m_BitEstimator;
  CABACWriter             m_CABACEstimator;
  BitEstimator            m_SaoBitEstimator;
  CABACWriter             m_SaoCABACEstimator;
  ReuseUniMv              m_ReuseUniMv;
  BlkUniMvInfoBuffer      m_BlkUniMvInfoBuffer;
  AffineProfList          m_AffineProfList;
  EncCu                   m_encCu;
  EncSampleAdaptiveOffset m_encSao;
  int                     m_prevQp[ MAX_NUM_CH ];
  LineEncRsrc( const VVEncCfg& encCfg ) : m_CABACEstimator( m_BitEstimator ), m_SaoCABACEstimator( m_SaoBitEstimator ) { m_AffineProfList.init( encCfg.m_IntraPeriod); }
};

struct PerThreadRsrc
{
  CtxCache  m_CtxCache;
};

struct CtuEncParam
{
  Picture*  pic;
  EncSlice* encSlice;
  int       ctuRsAddr;
  int       ctuPosX;
  int       ctuPosY;

  CtuEncParam() : pic( nullptr ), encSlice( nullptr ), ctuRsAddr( 0 ), ctuPosX( 0 ), ctuPosY( 0 ) {}
  CtuEncParam( Picture* _p, EncSlice* _s, const int _r, const int _x, const int _y ) : pic( _p ), encSlice( _s ), ctuRsAddr( _r ), ctuPosX( _x ), ctuPosY( _y ) {}
};

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncSlice::EncSlice()
  : m_pcEncCfg           ( nullptr)
  , m_threadPool         ( nullptr )
  , m_ctuTasksDoneCounter( nullptr )
  , m_pLoopFilter        ( nullptr )
  , m_pALF               ( nullptr )
  , m_pcRateCtrl         ( nullptr )
  , m_CABACWriter        ( m_BinEncoder )
  , m_encCABACTableIdx   ( I_SLICE )
  , m_appliedSwitchDQQ   ( 0 )
{
}


EncSlice::~EncSlice()
{
  for( auto* lnRsc : m_LineEncRsrc )
  {
    delete lnRsc;
  }
  m_LineEncRsrc.clear();

  for( auto* taskRsc: m_CtuTaskRsrc )
  {
    delete taskRsc;
  }
  m_CtuTaskRsrc.clear();

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
  m_syncPicCtx.resize( encCfg.m_entropyCodingSyncEnabled ? pps.pcv->heightInCtus : 0 );

  const int maxCntRscr = ( encCfg.m_numThreads > 0 ) ? pps.pcv->heightInCtus : 1;
  const int maxCtuEnc  = ( encCfg.m_numThreads > 0 && threadPool ) ? threadPool->numThreads() : 1;

  m_CtuTaskRsrc.resize( maxCtuEnc,  nullptr );
  m_LineEncRsrc.resize( maxCntRscr, nullptr );

  for( PerThreadRsrc*& taskRsc : m_CtuTaskRsrc )
  {
    taskRsc = new PerThreadRsrc();
  }

  for( LineEncRsrc*& lnRsc : m_LineEncRsrc )
  {
    lnRsc = new LineEncRsrc( encCfg );
    lnRsc->m_encCu.init( encCfg,
                         sps,
                         globalCtuQpVector,
                         m_syncPicCtx.data(),
                         &rateCtrl );
    if( sps.saoEnabled )
    {
      lnRsc->m_encSao.init( encCfg );
    }
  }

  m_appliedSwitchDQQ = 0;

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
}


void EncSlice::initPic( Picture* pic, int gopId )
{
  Slice* slice = pic->cs->slice;

  slice->sliceMap.addCtusToSlice( 0, pic->cs->pcv->widthInCtus, 0, pic->cs->pcv->heightInCtus, pic->cs->pcv->widthInCtus);

  // this ensures that independently encoded bitstream chunks can be combined to bit-equal
  const SliceType cabacTableIdx = ! slice->pps->cabacInitPresent || slice->pendingRasInit ? slice->sliceType : m_encCABACTableIdx;
  slice->encCABACTableIdx = cabacTableIdx;

  // set QP and lambda values
  xInitSliceLambdaQP( slice, gopId );

  for( auto* lnRsc : m_LineEncRsrc )
  {
    lnRsc->m_ReuseUniMv.resetReusedUniMvs();
    lnRsc->m_encCu.initPic( pic );
  }
}



void EncSlice::xInitSliceLambdaQP( Slice* slice, int gopId )
{
  const GOPEntry* gopList = m_pcEncCfg->m_GOPList;

  // pre-compute lambda and qp
  int  iQP, adaptedLumaQP = -1;
  double dQP     = xGetQPForPicture( slice, gopId );
  double dLambda = xCalculateLambda( slice, gopId, slice->depth, dQP, dQP, iQP );
  int sliceChromaQpOffsetIntraOrPeriodic[ 2 ] = { m_pcEncCfg->m_sliceChromaQpOffsetIntraOrPeriodic[ 0 ], m_pcEncCfg->m_sliceChromaQpOffsetIntraOrPeriodic[ 1 ] };

  if (slice->pps->sliceChromaQpFlag && m_pcEncCfg->m_usePerceptQPA && m_pcEncCfg->m_RCRateControlMode != RCM_CTU_LEVEL &&
      ((slice->isIntra() && !slice->sps->IBC) || (m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity > 0 && (slice->poc % m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity) == 0)))
  {
    adaptedLumaQP = BitAllocation::applyQPAdaptationChroma (slice, m_pcEncCfg, iQP, *m_LineEncRsrc[ 0 ]->m_encCu.getQpPtr(),
                                                            sliceChromaQpOffsetIntraOrPeriodic, m_pcEncCfg->m_usePerceptQPA > 2); // adapts sliceChromaQpOffsetIntraOrPeriodic[]
  }
  if (m_pcEncCfg->m_usePerceptQPA)
  {
    const bool rcIsFirstPassOf2 = (m_pcEncCfg->m_RCRateControlMode == RCM_OFF && slice->pps->useDQP && (m_pcEncCfg->m_usePerceptQPATempFiltISlice == 2) ? m_pcEncCfg->m_RCNumPasses == 2 && !m_pcRateCtrl->rcIsFinalPass : false);
    uint32_t  startCtuTsAddr    = slice->sliceMap.ctuAddrInSlice[0];
    uint32_t  boundingCtuTsAddr = slice->pic->cs->pcv->sizeInCtus;

    if ((m_pcEncCfg->m_RCNumPasses == 2) && m_pcRateCtrl->rcIsFinalPass && slice->pps->useDQP && (m_pcEncCfg->m_usePerceptQPATempFiltISlice == 2) && slice->isIntra() && (boundingCtuTsAddr > startCtuTsAddr))
    {
      const int nCtu = int (boundingCtuTsAddr - startCtuTsAddr);
      const int offs = (slice->poc / m_pcEncCfg->m_IntraPeriod) * ((nCtu + 1) >> 1);
      std::vector<uint8_t>& ctuQPMem = *m_pcRateCtrl->getIntraPQPAStats(); // unpack pass-1 red. QPs
      std::vector<int>& ctuPumpRedQP = *m_LineEncRsrc[0]->m_encCu.getQpPtr();

      if ((ctuPumpRedQP.size() >= nCtu) && (ctuQPMem.size() >= offs + ((nCtu + 1) >> 1)))
      {
        for (uint32_t ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++)
        {
          const uint32_t ctuRsAddr = /*tileMap.getCtuBsToRsAddrMap*/ (ctuTsAddr);

          ctuPumpRedQP[ctuRsAddr] = int ((ctuRsAddr & 1) ? ctuQPMem[offs + (ctuRsAddr >> 1)] >> 4 : ctuQPMem[offs + (ctuRsAddr >> 1)] & 15) - 8;
        }
      }
    }

    slice->sliceQp = iQP; // start slice QP for reference
    slice->pic->picInitialQP = iQP;

    if ((iQP = BitAllocation::applyQPAdaptationLuma (slice, m_pcEncCfg, adaptedLumaQP, dLambda, *m_LineEncRsrc[ 0 ]->m_encCu.getQpPtr(),
                                                     (rcIsFirstPassOf2 && slice->poc > 0 ? m_pcRateCtrl->getIntraPQPAStats() : nullptr),
                                                     startCtuTsAddr, boundingCtuTsAddr, m_pcEncCfg->m_usePerceptQPA > 2)) >= 0) // sets pic->ctuAdaptedQP[] & ctuQpaLambda[]
    {
      dLambda *= pow (2.0, ((double) iQP - dQP) / 3.0); // adjust lambda based on change of slice QP
    }
    else iQP = (int) dQP; // revert to unadapted slice QP
  }

  if ( slice->pps->sliceChromaQpFlag )
  {
    const bool bUseIntraOrPeriodicOffset = (slice->isIntra() && !slice->sps->IBC) || (m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity > 0 && (slice->poc % m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity) == 0);
    int cbQP = bUseIntraOrPeriodicOffset ? sliceChromaQpOffsetIntraOrPeriodic[ 0 ] : gopList[ gopId ].m_CbQPoffset;
    int crQP = bUseIntraOrPeriodicOffset ? sliceChromaQpOffsetIntraOrPeriodic[ 1 ] : gopList[ gopId ].m_CrQPoffset;
    int cbCrQP = (cbQP + crQP) >> 1; // use floor of average chroma QP offset for joint-Cb/Cr coding

    cbQP = Clip3( -12, 12, cbQP + slice->pps->chromaQpOffset[COMP_Cb] ) - slice->pps->chromaQpOffset[COMP_Cb];
    crQP = Clip3( -12, 12, crQP + slice->pps->chromaQpOffset[COMP_Cr] ) - slice->pps->chromaQpOffset[COMP_Cr];
    slice->sliceChromaQpDelta[COMP_Cb] = Clip3( -12, 12, cbQP);
    CHECK(!(slice->sliceChromaQpDelta[COMP_Cb]+slice->pps->chromaQpOffset[COMP_Cb]<=12 && slice->sliceChromaQpDelta[COMP_Cb]+slice->pps->chromaQpOffset[COMP_Cb]>=-12), "Unspecified error");
    slice->sliceChromaQpDelta[COMP_Cr] = Clip3( -12, 12, crQP);
    CHECK(!(slice->sliceChromaQpDelta[COMP_Cr]+slice->pps->chromaQpOffset[COMP_Cr]<=12 && slice->sliceChromaQpDelta[COMP_Cr]+slice->pps->chromaQpOffset[COMP_Cr]>=-12), "Unspecified error");
    if (slice->sps->jointCbCr)
    {
      cbCrQP = Clip3 (-12, 12, cbCrQP + slice->pps->chromaQpOffset[COMP_JOINT_CbCr]) - slice->pps->chromaQpOffset[COMP_JOINT_CbCr];
      slice->sliceChromaQpDelta[COMP_JOINT_CbCr] = Clip3 (-12, 12, cbCrQP);
    }
  }
  else
  {
    slice->sliceChromaQpDelta[COMP_Cb] = 0;
    slice->sliceChromaQpDelta[COMP_Cr] = 0;
    slice->sliceChromaQpDelta[COMP_JOINT_CbCr] = 0;
  }

  for( auto& lineRsc : m_LineEncRsrc )
  {
    lineRsc->m_encCu.setUpLambda( *slice, dLambda, iQP, true, true );
  }

  slice->sliceQp       = iQP;
  slice->chromaQpAdjEnabled = slice->pps->chromaQpOffsetListLen>0;

  if (slice->pps->sliceChromaQpFlag && CS::isDualITree (*slice->pic->cs) && (m_pcEncCfg->m_usePerceptQPA == 0) && (m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity == 0))
  {
    // overwrite chroma qp offset for dual tree
    slice->sliceChromaQpDelta[ COMP_Cb ] = m_pcEncCfg->m_chromaCbQpOffsetDualTree;
    slice->sliceChromaQpDelta[ COMP_Cr ] = m_pcEncCfg->m_chromaCrQpOffsetDualTree;
    if ( slice->sps->jointCbCr )
    {
      slice->sliceChromaQpDelta[ COMP_JOINT_CbCr ] = m_pcEncCfg->m_chromaCbCrQpOffsetDualTree;
    }
    for( auto& lineRsc : m_LineEncRsrc )
    {
      lineRsc->m_encCu.setUpLambda( *slice, slice->getLambdas()[0], slice->sliceQp, true, false );
    }
  }
}

void EncSlice::resetQP( Picture* pic, int sliceQP, double lambda )
{
  Slice* slice = pic->cs->slice;
  if ( RCM_GOP_LEVEL == m_pcEncCfg->m_RCRateControlMode )
  {
    int gopQp = sliceQP - (( slice->sliceType == I_SLICE ) ? m_pcEncCfg->m_intraQPOffset : 1);
    m_pcRateCtrl->encRCGOP->gopQP = gopQp;
  }

  if ( m_pcEncCfg->m_usePerceptQPA )
  {
    pic->encRCPic->picQPOffsetQPA = sliceQP - slice->sliceQp;
    pic->encRCPic->picLambdaOffsetQPA = lambda / slice->getLambdas()[ 0 ];
  }

  // store lambda
  slice->sliceQp = sliceQP;
  for( auto& lineRsc : m_LineEncRsrc )
  {
    lineRsc->m_encCu.setUpLambda( *slice, lambda, sliceQP, true, true, true );
  }
}

int EncSlice::xGetQPForPicture( const Slice* slice, unsigned gopId )
{
  const int lumaQpBDOffset = slice->sps->qpBDOffset[ CH_L ];
  int qp;

  if ( m_pcEncCfg->m_costMode == COST_LOSSLESS_CODING )
  {
    qp = LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP;
  }
  else
  {
    const SliceType sliceType = slice->sliceType;

    qp = m_pcEncCfg->m_QP;
    if ( RCM_GOP_LEVEL == m_pcEncCfg->m_RCRateControlMode )
    {
      m_pcRateCtrl->encRCSeq->setQpInGOP( gopId, m_pcRateCtrl->encRCGOP->gopQP, qp );
    }

    // switch at specific qp and keep this qp offset
    if( slice->poc == m_pcEncCfg->m_switchPOC )
    {
      m_appliedSwitchDQQ = m_pcEncCfg->m_switchDQP;
    }
    qp += m_appliedSwitchDQQ;

    if( sliceType == I_SLICE )
    {
      qp += m_pcEncCfg->m_intraQPOffset;
    }
    else
    {
      if ( ! ( qp == -lumaQpBDOffset ) )
      {
        const GOPEntry &gopEntry = m_pcEncCfg->m_GOPList[ gopId ];
        // adjust QP according to the QP offset for the GOP entry.
        qp += gopEntry.m_QPOffset;

        // adjust QP according to QPOffsetModel for the GOP entry.
        double dqpOffset = qp * gopEntry.m_QPOffsetModelScale + gopEntry.m_QPOffsetModelOffset + 0.5;
        int qpOffset = (int)floor( Clip3<double>( 0.0, 3.0, dqpOffset ) );
        qp += qpOffset ;
      }
    }
  }
  qp = Clip3( -lumaQpBDOffset, MAX_QP, qp );
  return qp;
}


double EncSlice::xCalculateLambda( const Slice*     slice,
                                  const int        GOPid, // entry in the GOP table
                                  const int        depth, // slice GOP hierarchical depth.
                                  const double     refQP, // initial slice-level QP
                                  const double     dQP,   // initial double-precision QP
                                          int&     iQP )  // returned integer QP.
{
  const  GOPEntry* gopList       = m_pcEncCfg->m_GOPList;
  const  int       NumberBFrames = ( m_pcEncCfg->m_GOPSize - 1 );
  const  int       SHIFT_QP      = 12;
  const int temporalId           = gopList[ GOPid ].m_temporalId;
  const std::vector<double> &intraLambdaModifiers = m_pcEncCfg->m_adIntraLambdaModifier;

  int bitdepth_luma_qp_scale = 6
                               * (slice->sps->bitDepths[ CH_L ] - 8
                                  - DISTORTION_PRECISION_ADJUSTMENT(slice->sps->bitDepths[ CH_L ]));
  double qp_temp = dQP + bitdepth_luma_qp_scale - SHIFT_QP;
  // Case #1: I or P-slices (key-frame)
  double dQPFactor = gopList[ GOPid ].m_QPFactor;
  if( slice->sliceType == I_SLICE )
  {
    if (m_pcEncCfg->m_dIntraQpFactor>=0.0 && gopList[ GOPid ].m_sliceType != I_SLICE)
    {
      dQPFactor = m_pcEncCfg->m_dIntraQpFactor;
    }
    else
    {
      dQPFactor = 0.57;
      if( ! m_pcEncCfg->m_lambdaFromQPEnable)
      {
        double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05 * (double)NumberBFrames );
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
  if ( !m_pcEncCfg->m_bUseHADME && slice->sliceType != I_SLICE )
  {
    dLambda *= 0.95;
  }

  double lambdaModifier;
  if( slice->sliceType != I_SLICE || intraLambdaModifiers.empty())
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
    cs.initStructData (slice->sliceQp );
  }

  for( auto* lnRsrc : m_LineEncRsrc )
  {
    lnRsrc->m_CABACEstimator.initCtxModels( *slice );
    lnRsrc->m_SaoCABACEstimator.initCtxModels( *slice );
    lnRsrc->m_AffineProfList.resetAffineMVList();
    lnRsrc->m_BlkUniMvInfoBuffer.resetUniMvList();
    lnRsrc->m_encCu.initSlice( slice );
    if( slice->sps->saoEnabled )
    {
      lnRsrc->m_encSao.initSlice( slice );
    }
  }

  if (slice->sps->fpelMmvd && !slice->picHeader->disFracMMVD)
  {
    slice->picHeader->disFracMMVD = (pic->lwidth() * pic->lheight() > 1920 * 1080) ? true : false;
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

class CtuTsIterator : public std::iterator<std::forward_iterator_tag, int>
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
    CtuTsIterator( const CodingStructure& _cs, int _s, int _e, bool _wpp                          ) : cs( _cs ), m_startTsAddr( _s ), m_endTsAddr( _e ),                     m_ctuTsAddr( _s ) { if( _wpp ) setWppPattern(); }
    CtuTsIterator( const CodingStructure& _cs, int _s, int _e, const std::vector<int>& _m         ) : cs( _cs ), m_startTsAddr( _s ), m_endTsAddr( _e ), m_ctuAddrMap( _m ), m_ctuTsAddr( _s ) {}
    CtuTsIterator( const CodingStructure& _cs, int _s, int _e, const std::vector<int>& _m, int _c ) : cs( _cs ), m_startTsAddr( _s ), m_endTsAddr( _e ), m_ctuAddrMap( _m ), m_ctuTsAddr( std::max( _s, _c ) ) {}

    virtual ~CtuTsIterator() { m_ctuAddrMap.clear(); }

    CtuTsIterator& operator++()                { m_ctuTsAddr = getNextTsAddr( m_ctuTsAddr ); return *this; }
    CtuTsIterator  operator++(int)             { auto retval = *this; ++(*this); return retval; }
    bool operator==(CtuTsIterator other) const { return m_ctuTsAddr == other.m_ctuTsAddr; }
    bool operator!=(CtuTsIterator other) const { return m_ctuTsAddr != other.m_ctuTsAddr; }
    CtuPos operator*()                   const { const int ctuRsAddr = mapAddr( m_ctuTsAddr );  return CtuPos( ctuRsAddr % cs.pcv->widthInCtus, ctuRsAddr / cs.pcv->widthInCtus, ctuRsAddr ); }

    CtuTsIterator begin() { return CtuTsIterator( cs, m_startTsAddr, m_endTsAddr, m_ctuAddrMap ); };
    CtuTsIterator end()   { return CtuTsIterator( cs, m_startTsAddr, m_endTsAddr, m_ctuAddrMap, m_endTsAddr ); };

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
  if( slice.sps->saoEnabled )
  {
    // store disabled statistics
    saoDisabledRate( cs, &m_saoReconParams[ 0 ] );

    // set slice header flags
    CHECK( m_saoEnabled[ COMP_Cb ] != m_saoEnabled[ COMP_Cr ], "Unspecified error");
    for( auto s : pic->slices )
    {
      s->saoEnabled[ CH_L ] = m_saoEnabled[ COMP_Y  ];
      s->saoEnabled[ CH_C ] = m_saoEnabled[ COMP_Cb ];
    }
  }

  CS::setRefinedMotionField( cs );

  // cleanup
  pic->getFilteredOrigBuffer().destroy();
}

void EncSlice::xProcessCtus( Picture* pic, const unsigned startCtuTsAddr, const unsigned boundingCtuTsAddr )
{
  CodingStructure& cs      = *pic->cs;
  Slice&           slice   = *cs.slice;
  const PreCalcValues& pcv = *cs.pcv;

  // initialization
  if( slice.sps->jointCbCr )
  {
    setJointCbCrModes( cs, Position(0, 0), cs.area.lumaSize() );
  }

  if( slice.sps->saoEnabled )
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

  if( slice.sps->alfEnabled )
  {
    m_pALF->resetFrameStats();
  }

  std::fill( m_processStates.begin(), m_processStates.end(), CTU_ENCODE );

  // fill encoder parameter list
  int idx = 0;
  auto ctuIter = CtuTsIterator( cs, startCtuTsAddr, boundingCtuTsAddr, m_pcEncCfg->m_numThreads > 0 );
  for( auto ctuPos : ctuIter )
  {
    ctuEncParams[ idx ].pic       = pic;
    ctuEncParams[ idx ].encSlice  = this;
    ctuEncParams[ idx ].ctuRsAddr = ctuPos.ctuRsAddr;
    ctuEncParams[ idx ].ctuPosX   = ctuPos.ctuPosX;
    ctuEncParams[ idx ].ctuPosY   = ctuPos.ctuPosY;
    idx++;
  }
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

template<bool checkReadyState>
bool EncSlice::xProcessCtuTask( int threadIdx, CtuEncParam* ctuEncParam )
{
  Picture* pic                   = ctuEncParam->pic;
  EncSlice* encSlice             = ctuEncParam->encSlice;
  CodingStructure& cs            = *pic->cs;
  Slice&           slice         = *cs.slice;
  const PreCalcValues& pcv       = *cs.pcv;
  const int ctuRsAddr            = ctuEncParam->ctuRsAddr;
  const int ctuPosX              = ctuEncParam->ctuPosX;
  const int ctuPosY              = ctuEncParam->ctuPosY;
  const int x                    = ctuPosX << pcv.maxCUSizeLog2;
  const int y                    = ctuPosY << pcv.maxCUSizeLog2;
  const int width                = std::min( pcv.maxCUSize, pcv.lumaWidth  - x );
  const int height               = std::min( pcv.maxCUSize, pcv.lumaHeight - y );
  const int ctuStride            = pcv.widthInCtus;
  ProcessCtuState* processStates = encSlice->m_processStates.data();
  const UnitArea ctuArea( pcv.chrFormat, Area( x, y, width, height ) );
  const bool wppSyncEnabled      = cs.sps->entropyCodingSyncEnabled;

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "poc", cs.slice->poc ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", processStates[ ctuRsAddr ] == CTU_ENCODE ? 0 : 1 ) );

  // process ctu's line wise from left to right
  if( ctuPosX > 0 && processStates[ ctuRsAddr - 1 ] <= processStates[ ctuRsAddr ] && processStates[ ctuRsAddr ] < PROCESS_DONE )
    return false;

  switch( processStates[ ctuRsAddr ].load() )
  {
    // encode
    case CTU_ENCODE:
      {
        // general wpp conditions, top and top-right ctu have to be encoded
        if( ctuPosY > 0                                  && processStates[ ctuRsAddr - ctuStride     ] <= CTU_ENCODE )
          return false;
        if( ctuPosY > 0 && ctuPosX + 1 < pcv.widthInCtus && processStates[ ctuRsAddr - ctuStride + 1 ] <= CTU_ENCODE && !wppSyncEnabled )
          return false;

        if( checkReadyState )
          return true;

#ifdef TRACE_ENABLE_ITT
        std::stringstream ss;
        ss << "Encode_" << slice.poc << "_CTU_" << ctuPosY << "_" << ctuPosX;
        __itt_string_handle* itt_handle_ctuEncode = __itt_string_handle_create( ss.str().c_str() );
#endif
        ITT_TASKSTART( itt_domain_encode, itt_handle_ctuEncode );

        const int lineIdx        = std::min( (int)( encSlice->m_LineEncRsrc.size() ) - 1, ctuPosY );
        LineEncRsrc* lineEncRsrc = encSlice->m_LineEncRsrc[ lineIdx ];
        PerThreadRsrc* taskRsrc  = encSlice->m_CtuTaskRsrc[ threadIdx ];
        EncCu& encCu             = lineEncRsrc->m_encCu;

        encCu.setCtuEncRsrc( &lineEncRsrc->m_CABACEstimator, &taskRsrc->m_CtxCache, &lineEncRsrc->m_ReuseUniMv, &lineEncRsrc->m_BlkUniMvInfoBuffer, &lineEncRsrc->m_AffineProfList );
        encCu.encodeCtu( pic, lineEncRsrc->m_prevQp, ctuPosX, ctuPosY );

        // cleanup line memory when last ctu in line done to reduce overall memory consumption
        if( encSlice->m_pcEncCfg->m_ensureWppBitEqual && ctuPosX == pcv.widthInCtus - 1 )
        {
          lineEncRsrc->m_AffineProfList.resetAffineMVList();
          lineEncRsrc->m_BlkUniMvInfoBuffer.resetUniMvList();
          lineEncRsrc->m_ReuseUniMv.resetReusedUniMvs();
          pic->cs->motionLutBuf[ ctuPosY ].lut.resize(0);
        }

        DTRACE_UPDATE( g_trace_ctx, std::make_pair( "final", 1 ) );
        ITT_TASKEND( itt_domain_encode, itt_handle_ctuEncode );

        processStates[ ctuRsAddr ] = RESHAPE_LF_VER;
      }
      break;

    // reshape + vertical loopfilter
    case RESHAPE_LF_VER:
      {
        // ensure all surrounding ctu's are encoded (intra pred requires non-reshaped and unfiltered residual)
        // due to wpp condition above, only right, bottom and bottom-right ctu have to be checked
        if( ctuPosX + 1 < pcv.widthInCtus                                   && processStates[ ctuRsAddr + 1             ] <= CTU_ENCODE )
          return false;
        if(                                  ctuPosY + 1 < pcv.heightInCtus && processStates[ ctuRsAddr     + ctuStride ] <= CTU_ENCODE )
          return false;
        if( ctuPosX + 1 < pcv.widthInCtus && ctuPosY + 1 < pcv.heightInCtus && processStates[ ctuRsAddr + 1 + ctuStride ] <= CTU_ENCODE )
          return false;

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_rspLfVer );

        // reshape
        if( slice.sps->lumaReshapeEnable && slice.picHeader->lmcsEnabled )
        {
          PelBuf reco = pic->getRecoBuf( COMP_Y ).subBuf( x, y, width, height );
          reco.rspSignal( pic->reshapeData.getInvLUT() );
        }

        // loopfilter
        if( !cs.pps->deblockingFilterControlPresent || !cs.pps->deblockingFilterDisabled || cs.pps->deblockingFilterOverrideEnabled )
        {
          // calculate filter strengths
          encSlice->m_pLoopFilter->calcFilterStrengthsCTU( cs, ctuArea, true );

          // vertical filter
          PelUnitBuf reco = cs.picture->getRecoBuf();
          encSlice->m_pLoopFilter->xDeblockArea<EDGE_VER>( cs, ctuArea, MAX_NUM_CH, reco );
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_rspLfVer );

        processStates[ ctuRsAddr ] = LF_HOR;
      }
      break;

    // horizontal loopfilter
    case LF_HOR:
      {
        // ensure horizontal ordering (from top to bottom)
        if( ctuPosY > 0 && processStates[ ctuRsAddr - ctuStride ] <= LF_HOR )
          return false;

        // ensure vertical loop filter of neighbor ctu's will not modify current residual
        // check top, top-right and right ctu
        // (top, top-right checked implicitly due to ordering check above)
        if( ctuPosX + 1 < pcv.widthInCtus && processStates[ ctuRsAddr + 1 ] <= RESHAPE_LF_VER )
          return false;

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_lfHor );

        if( !cs.pps->deblockingFilterControlPresent || !cs.pps->deblockingFilterDisabled || cs.pps->deblockingFilterOverrideEnabled )
        {
          PelUnitBuf reco = cs.picture->getRecoBuf();
          encSlice->m_pLoopFilter->xDeblockArea<EDGE_HOR>( cs, ctuArea, MAX_NUM_CH, reco );
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_lfHor );

        processStates[ ctuRsAddr ] = SAO_FILTER;
      }
      break;

    // SAO filter
    case SAO_FILTER:
      {
        // general wpp conditions, top and top-right ctu have to be filtered
        if( ctuPosY > 0                                  && processStates[ ctuRsAddr - ctuStride     ] <= SAO_FILTER )
          return false;
        if( ctuPosY > 0 && ctuPosX + 1 < pcv.widthInCtus && processStates[ ctuRsAddr - ctuStride + 1 ] <= SAO_FILTER )
          return false;

        // ensure loop filter of neighbor ctu's will not modify current residual
        // sao processing dependents on +1 pixel to each side
        // due to wpp condition above, only right, bottom and bottom-right ctu have to be checked
        if( ctuPosX + 1 < pcv.widthInCtus                                   && processStates[ ctuRsAddr + 1             ] <= LF_HOR )
          return false;
        if(                                  ctuPosY + 1 < pcv.heightInCtus && processStates[ ctuRsAddr     + ctuStride ] <= LF_HOR )
          return false;
        if( ctuPosX + 1 < pcv.widthInCtus && ctuPosY + 1 < pcv.heightInCtus && processStates[ ctuRsAddr + 1 + ctuStride ] <= LF_HOR )
          return false;

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_sao );

        // SAO filter
        if( slice.sps->saoEnabled )
        {
          const int lineIdx               = std::min( (int)( encSlice->m_LineEncRsrc.size() ) - 1, ctuPosY );
          LineEncRsrc* lineEncRsrc        = encSlice->m_LineEncRsrc[ lineIdx ];
          PerThreadRsrc* taskRsrc         = encSlice->m_CtuTaskRsrc[ threadIdx ];
          EncSampleAdaptiveOffset& encSao = lineEncRsrc->m_encSao;

          encSao.setCtuEncRsrc( &lineEncRsrc->m_SaoCABACEstimator, &taskRsrc->m_CtxCache );
          encSao.storeCtuReco( cs, ctuArea );
          encSao.getCtuStatistics( cs, encSlice->m_saoStatData, ctuArea, ctuRsAddr );
          encSao.decideCtuParams( cs, encSlice->m_saoStatData, encSlice->m_saoEnabled, encSlice->m_saoAllDisabled, ctuArea, ctuRsAddr, &encSlice->m_saoReconParams[ 0 ], cs.picture->getSAO() );
        }

        // ALF border extension
        if( slice.sps->alfEnabled )
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
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_sao );

        processStates[ ctuRsAddr ] = ALF_GET_STATISTICS;
      }
      break;

    case ALF_GET_STATISTICS:
      {
        // ensure all surrounding ctu's are filtered (ALF will use pixels of adjacent CTU's)
        // due to wpp condition above in SAO_FILTER, only right, bottom and bottom-right ctu have to be checked
        if( ctuPosX + 1 < pcv.widthInCtus                                   && processStates[ ctuRsAddr + 1             ] <= SAO_FILTER )
          return false;
        if(                                  ctuPosY + 1 < pcv.heightInCtus && processStates[ ctuRsAddr     + ctuStride ] <= SAO_FILTER )
          return false;
        if( ctuPosX + 1 < pcv.widthInCtus && ctuPosY + 1 < pcv.heightInCtus && processStates[ ctuRsAddr + 1 + ctuStride ] <= SAO_FILTER )
          return false;

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_alf_stat );

        // ALF pre-processing
        unsigned lastPreProcCTU = ( pcv.heightInCtus * pcv.widthInCtus ) - 1;
        if( ctuRsAddr <= lastPreProcCTU )
        {
          if( slice.sps->alfEnabled )
          {
            PelUnitBuf recoBuf = cs.picture->getRecoBuf();
            encSlice->m_pALF->getStatisticsCTU( *cs.picture, cs, recoBuf, ctuRsAddr );
          }
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_alf_stat );

        // derive alf filter only once for whole picture
        const unsigned deriveFilterCtu = pcv.sizeInCtus - 1;
        processStates[ ctuRsAddr ] = ( ctuRsAddr == deriveFilterCtu ) ? ALF_DERIVE_FILTER : ALF_RECONSTRUCT;
      }
      break;

    case ALF_DERIVE_FILTER:
      {
        CHECK( ctuRsAddr != pcv.sizeInCtus - 1, "invalid state, derive alf filter only once for last ctu" );

        // ensure statistics from all previous ctu's have been collected
        for( int i = 0; i < ctuRsAddr; i++ )
          if( processStates[ i ] <= ALF_GET_STATISTICS )
            return false;

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_alf_derive );

        // ALF post-processing
        if( slice.sps->alfEnabled )
        {
          encSlice->m_pALF->deriveFilter( *cs.picture, cs, slice.getLambdas() );
          encSlice->m_pALF->reconstructCoeffAPSs( cs, true, cs.slice->tileGroupAlfEnabled[COMP_Cb] || cs.slice->tileGroupAlfEnabled[COMP_Cr], false );
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_alf_derive );

        processStates[ ctuRsAddr ] = ALF_RECONSTRUCT;
      }
      break;

    case ALF_RECONSTRUCT:
      {
        const unsigned deriveFilterCtu = pcv.sizeInCtus - 1;

        // start alf reconstruct, when derive filter is done
        if( processStates[ deriveFilterCtu ] < ALF_RECONSTRUCT )
          return false;

        // general wpp conditions, top and top-right ctu have to be encoded
        if( ctuPosY > 0                                  && processStates[ ctuRsAddr - ctuStride     ] <= ALF_RECONSTRUCT )
          return false;
        if( ctuPosY > 0 && ctuPosX + 1 < pcv.widthInCtus && processStates[ ctuRsAddr - ctuStride + 1 ] <= ALF_RECONSTRUCT )
          return false;

        if( checkReadyState )
          return true;

        ITT_TASKSTART( itt_domain_encode, itt_handle_alf_recon );

        if( slice.sps->alfEnabled )
        {
          encSlice->m_pALF->reconstructCTU_MT( *cs.picture, cs, ctuRsAddr );
        }

        ITT_TASKEND( itt_domain_encode, itt_handle_alf_recon );

        // perform finish only once for whole picture
        const unsigned finishCtu = pcv.sizeInCtus - 1;
        if( ctuRsAddr < finishCtu )
        {
          processStates[ ctuRsAddr ] = PROCESS_DONE;
          // processing done => terminate thread
          return true;
        }
        processStates[ ctuRsAddr ] = FINISH_SLICE;
      }

    case FINISH_SLICE:
      {
        CHECK( ctuRsAddr != pcv.sizeInCtus - 1, "invalid state, finish slice only once for last ctu" );

        // ensure ALF has been done for all previous ctu's
        for( int i = 0; i < ctuRsAddr; i++ )
          if( processStates[ i ] <= ALF_RECONSTRUCT )
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
  const int numSubstreamsColumns  = slice->pps->numExpTileCols;
  const int numSubstreamRows      = slice->sps->entropyCodingSyncEnabled ? pic->cs->pcv->heightInCtus : slice->pps->numExpTileRows;
  const int numSubstreams         = std::max<int>( numSubstreamRows * numSubstreamsColumns, 0/*(int)pic->brickMap->bricks.size()*/ );
  std::vector<OutputBitstream> substreamsOut( numSubstreams );

  slice->clearSubstreamSizes();

  for( uint32_t ctuTsAddr = startCtuTsAddr; ctuTsAddr < boundingCtuTsAddr; ctuTsAddr++ )
  {
    const uint32_t ctuRsAddr = (ctuTsAddr);
    const uint32_t firstCtuRsAddrOfTile = 0;
    const uint32_t tileXPosInCtus       = firstCtuRsAddrOfTile % widthInCtus;
    const uint32_t ctuXPosInCtus        = ctuRsAddr % widthInCtus;
    const uint32_t ctuYPosInCtus        = ctuRsAddr / widthInCtus;

    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "ctu", ctuRsAddr ) );

    const Position pos (ctuXPosInCtus * pcv.maxCUSize, ctuYPosInCtus * pcv.maxCUSize);
    const UnitArea ctuArea (cs.area.chromaFormat, Area(pos.x, pos.y, pcv.maxCUSize, pcv.maxCUSize));
    CHECK( uiSubStrm >= numSubstreams, "array index out of bounds" );
    m_CABACWriter.initBitstream( &substreamsOut[ uiSubStrm ] );

    // set up CABAC contexts' state for this CTU
    if (ctuRsAddr == firstCtuRsAddrOfTile)
    {
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_CABACWriter.initCtxModels( *slice );
      }
    }
    else if (ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled)
    {
      // Synchronize cabac probabilities with upper-right CTU if it's available and at the start of a line.
      if (ctuTsAddr != startCtuTsAddr) // if it is the first CTU, then the entropy coder has already been reset
      {
        m_CABACWriter.initCtxModels( *slice );
      }
      if( cs.getCURestricted( pos.offset( 0, -1 ), pos, slice->independentSliceIdx, 0, CH_L, TREE_D ) )
      {
        // Top-right is available, so use it.
        m_CABACWriter.getCtx() = m_entropyCodingSyncContextState;
      }
    }

    m_CABACWriter.coding_tree_unit( cs, ctuArea, prevQP, ctuRsAddr );

    // store probabilities of second CTU in line into buffer
    if( ctuXPosInCtus == tileXPosInCtus && wavefrontsEnabled )
    {
      m_entropyCodingSyncContextState = m_CABACWriter.getCtx();
    }

    // terminate the sub-stream, if required (end of slice-segment, end of tile, end of wavefront-CTU-row):
    bool isLastCTUinWPP = wavefrontsEnabled && (((ctuRsAddr + 1) % widthInCtus) == tileXPosInCtus);
    bool isMoreCTUsinSlice = ctuRsAddr != (boundingCtuTsAddr - 1);
    if (isLastCTUinWPP || !isMoreCTUsinSlice)         // this the the last CTU of either tile/brick/WPP/slice
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


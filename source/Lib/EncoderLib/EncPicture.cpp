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

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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


/** \file     EncPicture.cpp
    \brief    encode picture
*/

#include "EncPicture.h"
#include "EncGOP.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "vvenc/vvencCfg.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

#ifdef TRACE_ENABLE_ITT
static __itt_string_handle* itt_handle_start = __itt_string_handle_create( "PicEnc" );
static __itt_domain* itt_domain_picEncoder   = __itt_domain_create( "PictureEncoder" );
#endif

// ---------------------------------------------------------------------------------------------------------------------

void EncPicture::init( const VVEncCfg& encCfg,
                       std::vector<int>* const globalCtuQpVector,
                       const SPS& sps,
                       const PPS& pps,
                       RateCtrl& rateCtrl,
                       NoMallocThreadPool* threadPool )
{
  m_pcEncCfg = &encCfg;

  if( encCfg.m_alf || encCfg.m_ccalf )
    m_ALF       .init( encCfg, m_CABACEstimator, m_CtxCache, threadPool );

  m_SliceEncoder.init( encCfg, sps, pps, globalCtuQpVector, m_LoopFilter, m_ALF, rateCtrl, threadPool, &m_ctuTasksDoneCounter );
  m_pcRateCtrl = &rateCtrl;
}


void EncPicture::compressPicture( Picture& pic, EncGOP& gopEncoder )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_TOP_LEVEL, pic.cs, CH_L );
  ITT_TASKSTART( itt_domain_picEncoder, itt_handle_start );

  pic.encTime.startTimer();

  pic.createTempBuffers( pic.cs->pcv->maxCUSize );
  pic.cs->createCoeffs();
  pic.cs->createTempBuffers( true );
  pic.cs->initStructData( MAX_INT, false, nullptr, true );

  if( pic.useScLMCS && m_pcEncCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ && m_pcEncCfg->m_alf )
  {
    const double *weights = gopEncoder.getReshaper().getlumaLevelToWeightPLUT();
    auto& vec = m_ALF.getLumaLevelWeightTable();
    const size_t numEl = size_t( 1 ) << m_pcEncCfg->m_internalBitDepth[0];

    vec.resize( numEl );
    std::copy( weights, weights + numEl, vec.begin() );

    m_ALF.setAlfWSSD( 1 );
  }
  else
  {
    m_ALF.setAlfWSSD( 0 );
  }

  // compress picture
  xInitPicEncoder ( pic );
  if( m_pcEncCfg->m_RCTargetBitrate > 0 )
  {
    gopEncoder.picInitRateControl( pic, pic.slices[0], this );
  }

  // compress current slice
  pic.cs->slice = pic.slices[0];
  m_SliceEncoder.compressSlice( &pic );

  ITT_TASKEND( itt_domain_picEncoder, itt_handle_start );
}

void EncPicture::finalizePicture( Picture& pic )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_TOP_LEVEL, pic.cs, CH_L );
  CodingStructure& cs = *(pic.cs);
  Slice* slice        = pic.slices[0];
  // ALF
  if( slice->sps->alfEnabled && pic.encPic )
  {
#ifdef TRACE_ENABLE_ITT
    std::stringstream ss;
    ss << "ALF_post_" << slice->poc;
    __itt_string_handle* itt_handle_post = __itt_string_handle_create( ss.str().c_str() );
#endif
    pic.picApsMap.setApsIdStart( m_ALF.getApsIdStart() );

    cs.slice->ccAlfFilterParam      = m_ALF.getCcAlfFilterParam();
    cs.slice->ccAlfFilterControl[0] = m_ALF.getCcAlfControlIdc(COMP_Cb);
    cs.slice->ccAlfFilterControl[1] = m_ALF.getCcAlfControlIdc(COMP_Cr);

    DTRACE( g_trace_ctx, D_CRC, "ALF" );
    DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );
    DTRACE_PIC_COMP( D_REC_CB_LUMA_ALF,   cs, cs.getRecoBuf(), COMP_Y  );
    DTRACE_PIC_COMP( D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMP_Cb );
    DTRACE_PIC_COMP( D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMP_Cr );
  }

  if( pic.writePic )
  {
    // write picture
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "bsfinal", 1 ) );
    xWriteSliceData( pic );
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "bsfinal", 0 ) );
  }

  // finalize
  pic.extendPicBorder();
  if ( m_pcEncCfg->m_useAMaxBT )
  {
    pic.picBlkStat.storeBlkSize( pic );
  }
  // cleanup
  if( pic.encPic )
  {
    pic.cs->releaseIntermediateData();
  }
  pic.cs->destroyTempBuffers();
  pic.cs->destroyCoeffs();
  pic.destroyTempBuffers();

  pic.encTime.stopTimer();
}

void EncPicture::xInitPicEncoder( Picture& pic )
{
  m_SliceEncoder.initPic( &pic, pic.gopId);

  Slice* slice = pic.cs->slice;

  xInitSliceColFromL0Flag( slice );
  xInitSliceCheckLDC     ( slice );

  if( slice->sps->alfEnabled )
  {
    for (int s = 0; s < (int)pic.slices.size(); s++)
    {
      pic.slices[s]->tileGroupAlfEnabled[COMP_Y] = false;
    }
  }
}


void EncPicture::xInitSliceColFromL0Flag( Slice* slice ) const
{
  if( m_pcEncCfg->m_rprRASLtoolSwitch )
  {
    return;
  }
  
  if ( slice->sliceType == VVENC_B_SLICE )
  {
    const int refIdx = 0; // Zero always assumed
    const Picture* refPicL0 = slice->getRefPic( REF_PIC_LIST_0, refIdx );
    const Picture* refPicL1 = slice->getRefPic( REF_PIC_LIST_1, refIdx );
    slice->colFromL0Flag = ( refPicL0->slices[ 0 ]->sliceQp > refPicL1->slices[ 0 ]->sliceQp );
  }
}


void EncPicture::xInitSliceCheckLDC( Slice* slice ) const
{
  if ( slice->sliceType == VVENC_B_SLICE )
  {
    bool bLowDelay = true;
    int  iCurrPOC  = slice->poc;
    int  iRefIdx   = 0;

    for ( iRefIdx = 0; iRefIdx < slice->numRefIdx[ REF_PIC_LIST_0 ] && bLowDelay; iRefIdx++ )
    {
      if ( slice->getRefPic( REF_PIC_LIST_0, iRefIdx )->getPOC() > iCurrPOC )
      {
        bLowDelay = false;
      }
    }
    for ( iRefIdx = 0; iRefIdx < slice->numRefIdx[ REF_PIC_LIST_1 ] && bLowDelay; iRefIdx++ )
    {
      if ( slice->getRefPic( REF_PIC_LIST_1, iRefIdx )->getPOC() > iCurrPOC )
      {
        bLowDelay = false;
      }
    }

    slice->checkLDC = bLowDelay;
  }
  else
  {
    slice->checkLDC = true;
  }
}


void EncPicture::skipCompressPicture( Picture& pic, ParameterSetMap<APS>& shrdApsMap )
{
  CodingStructure& cs = *(pic.cs);
  Slice* slice        = pic.slices[ 0 ];

  if( slice->sps->saoEnabled )
  {
    m_SliceEncoder.saoDisabledRate( cs, pic.getSAO( 1 ) );
  }

  if ( slice->sps->alfEnabled && ( slice->tileGroupAlfEnabled[COMP_Y] || slice->tileGroupCcAlfCbEnabled || slice->tileGroupCcAlfCrEnabled ) )
  {
    // IRAP AU: reset APS map
    int layerIdx = slice->vps == nullptr ? 0 : slice->vps->generalLayerIdx[ pic.layerId ];
    if( !layerIdx && ( slice->pendingRasInit || slice->isIDRorBLA() ) )
    {
      // We have to reset all APS on IRAP, but in not encoding case we have to keep the parsed APS of current slice
      // Get active ALF APSs from picture/slice header
      const std::vector<int> sliceApsIdsLuma = slice->tileGroupLumaApsId;

      m_ALF.setApsIdStart( ALF_CTB_MAX_NUM_APS );

      ParameterSetMap<APS>* apsMap = &pic.picApsMap;
      apsMap->clearActive();

      for( int apsId = 0; apsId < ALF_CTB_MAX_NUM_APS; apsId++ )
      {
        int psId = ( apsId << NUM_APS_TYPE_LEN ) + ALF_APS;
        APS* aps = apsMap->getPS( psId );
        if( aps )
        {
          // Check if this APS is currently the active one (used in current slice)
          bool activeAps = false;
          bool activeApsCcAlf = false;

          // Luma
          for( int i = 0; i < sliceApsIdsLuma.size(); i++ )
          {
            if( aps->apsId == sliceApsIdsLuma[i] )
            {
              activeAps = true;
              break;
            }
          }
          // Chroma
          activeAps |= ( slice->tileGroupAlfEnabled[COMP_Cb] || slice->tileGroupAlfEnabled[COMP_Cr] ) && aps->apsId == slice->tileGroupChromaApsId;
          // CC-ALF
          activeApsCcAlf |= slice->tileGroupCcAlfCbEnabled && aps->apsId == slice->tileGroupCcAlfCbApsId;
          activeApsCcAlf |= slice->tileGroupCcAlfCrEnabled && aps->apsId == slice->tileGroupCcAlfCrApsId;

          if( !activeAps && !activeApsCcAlf )
          {
            apsMap->clearChangedFlag( psId );
          }
          if( !activeAps  )
          {
            aps->alfParam.reset();
          }
          if( !activeApsCcAlf )
          {
            aps->ccAlfParam.reset();
          }
        }
      }
    }

    // Assign tne correct APS to slice and emulate the setting of ALF start APS ID
    int changedApsId = -1;
    for( int apsId = ALF_CTB_MAX_NUM_APS - 1; apsId >= 0; apsId-- )
    {
      ParameterSetMap<APS>* apsMap = &pic.picApsMap;
      int psId = ( apsId << NUM_APS_TYPE_LEN ) + ALF_APS;
      APS* aps = apsMap->getPS( psId );
      if( aps )
      {
        DTRACE( g_trace_ctx, D_ALF, "%d: id=%d, apsId=%d, t=%d, ch=%d, lid=%d, tid=%d, nf=%d,%d\n", slice->poc, psId, aps->apsId, aps->apsType, apsMap->getChangedFlag( psId ), aps->layerId, aps->temporalId, aps->alfParam.newFilterFlag[CH_L], aps->alfParam.newFilterFlag[CH_C] );
        // In slice, replace the old APS (from decoder map) with the APS from encoder map due to later checks while bitstream writing
        if( slice->alfAps[apsId] )
        {
          slice->alfAps[apsId] = aps;
        }

        if( apsMap->getChangedFlag( psId ) )
        {
          changedApsId = apsId;
        }
      }
    }

    if( changedApsId >= 0 )
    {
      m_ALF.setApsIdStart( changedApsId );
    }

    DTRACE( g_trace_ctx, D_ALF, "m_apsIdStart=%d\n", m_ALF.getApsIdStart() );
  }

  // LMCS APS
  // Here the decoded LMCS-APS is contained in the global shared APS map
  // We have to sync the picture LMCS-APS with the global shared LMCS-APS
  // NOTE: the "ChangedFlag" is also synced, enabling the APS writing later
  if( slice->sps->lumaReshapeEnable )
  {
    ParameterSetMap<APS>& apsMap = shrdApsMap;
    const int lmcsApsId          = slice->picHeader->lmcsApsId;
    const int apsMapIdx          = ( lmcsApsId << NUM_APS_TYPE_LEN ) + LMCS_APS;
    APS* aps                     = apsMap.getPS( apsMapIdx );
    if( aps )
    {
      // LMCS parameters are not completely coming from APS, some data have to come from picture/slice header
      // Better structuring of parameters is suggested
      if( slice->picHeader->lmcsEnabled )
      {
        aps->lmcsParam.sliceReshaperModelPresent = true;
        aps->lmcsParam.sliceReshaperEnabled      = slice->picHeader->lmcsEnabled;
        aps->lmcsParam.enableChromaAdj           = slice->picHeader->lmcsChromaResidualScale;
      }
      else
      {
        aps->lmcsParam.sliceReshaperModelPresent = false;
        aps->lmcsParam.sliceReshaperEnabled      = false;
      }

      ParameterSetMap<APS>& picApsMap = pic.picApsMap;
      APS* picAps                     = picApsMap.getPS( apsMapIdx );
      if ( picAps == nullptr )
      {
        picAps = picApsMap.allocatePS( apsMapIdx );
        picAps->apsType     = LMCS_APS;
        picAps->apsId       = lmcsApsId;
      }
      picAps->lmcsParam = aps->lmcsParam;
      picApsMap.setChangedFlag( apsMapIdx, apsMap.getChangedFlag( apsMapIdx ) );
      apsMap.clearChangedFlag( apsMapIdx ); // clear flag in the global map, preventing the reusing of it
      slice->picHeader->lmcsAps = picAps; // just to be sure
    }
  }
}

void EncPicture::xWriteSliceData( Picture& pic )
{
  const int numSlices = (int)pic.slices.size();

  pic.sliceDataStreams.clear();
  pic.sliceDataStreams.resize( numSlices );
  pic.sliceDataNumBins = 0;

  for ( int i = 0; i < numSlices; i++ )
  {
    // set current slice
    pic.cs->slice = pic.slices[i];
    m_SliceEncoder.encodeSliceData( &pic );
  }
}

} // namespace vvenc

//! \}


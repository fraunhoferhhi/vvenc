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


/** \file     EncPicture.cpp
    \brief    encode picture
*/

#include "EncPicture.h"
#include "EncGOP.h"
#include "UnitTools.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "vvenc/EncCfg.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

#ifdef TRACE_ENABLE_ITT
static __itt_string_handle* itt_handle_start = __itt_string_handle_create( "PicEnc" );
static __itt_domain* itt_domain_picEncoder   = __itt_domain_create( "PictureEncoder" );
static __itt_string_handle* itt_handle_post  = __itt_string_handle_create( "ALF_post" );
static __itt_domain* itt_domain_ALF_post     = __itt_domain_create( "ALFPost" );
#endif

// ---------------------------------------------------------------------------------------------------------------------

void EncPicture::init( const EncCfg& encCfg, std::vector<int>* const globalCtuQpVector,
                       const SPS& sps, const PPS& pps, RateCtrl& rateCtrl, NoMallocThreadPool* threadPool )
{
  m_pcEncCfg = &encCfg;

  m_ALF.init         ( encCfg, m_CABACEstimator, m_CtxCache, threadPool );
  m_SliceEncoder.init( encCfg, sps, pps, globalCtuQpVector, m_LoopFilter, m_ALF, rateCtrl, threadPool );
}


void EncPicture::encodePicture( Picture& pic, ParameterSetMap<APS>& shrdApsMap, EncGOP& gopEncoder )
{
  ITT_TASKSTART( itt_domain_picEncoder, itt_handle_start );

  pic.encTime.startTimer();

  // compress picture
  if ( pic.encPic )
  {
    xInitPicEncoder ( pic );
    gopEncoder.picInitRateControl( pic.gopId, pic, pic.slices[ 0 ] );
    xCompressPicture( pic );
  }
  else
  {
    xSkipCompressPicture( pic, shrdApsMap );
  }

  if( pic.writePic)
  {
    // write picture
    xWriteSliceData( pic );
  }

  // finalize
  pic.extendPicBorder();
  pic.slices[ 0 ]->updateRefPicCounter( -1 );
  if ( m_pcEncCfg->m_useAMaxBT )
  {
    pic.picBlkStat.storeBlkSize( pic );
  }
  // cleanup
  pic.destroyTempBuffers();
  pic.cs->destroyCoeffs();
  pic.cs->releaseIntermediateData();

  pic.encTime.stopTimer();

  gopEncoder.finishEncPicture( this, pic );

  ITT_TASKEND( itt_domain_picEncoder, itt_handle_start );
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
  if ( slice->sliceType == B_SLICE )
  {
    const int refIdx = 0; // Zero always assumed
    const Picture* refPicL0 = slice->getRefPic( REF_PIC_LIST_0, refIdx );
    const Picture* refPicL1 = slice->getRefPic( REF_PIC_LIST_1, refIdx );
    slice->colFromL0Flag = ( refPicL0->slices[ 0 ]->sliceQp > refPicL1->slices[ 0 ]->sliceQp );
  }
}


void EncPicture::xInitSliceCheckLDC( Slice* slice ) const
{
  if ( slice->sliceType == B_SLICE )
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


void EncPicture::xCompressPicture( Picture& pic )
{
  Slice* slice             = pic.slices[0];

  // set current slice
  pic.cs->slice = slice;

  m_SliceEncoder.compressSlice( &pic );

  CodingStructure& cs = *(pic.cs);

  // ALF
  if( slice->sps->alfEnabled )
  {
    ITT_TASKSTART( itt_domain_ALF_post, itt_handle_post );
    if( m_pcEncCfg->m_ccalf )
    {
      m_ALF.performCCALF( pic, cs );
    }
    ITT_TASKEND( itt_domain_ALF_post, itt_handle_post );

    cs.slice->ccAlfFilterParam      = m_ALF.getCcAlfFilterParam();
    cs.slice->ccAlfFilterControl[0] = m_ALF.getCcAlfControlIdc(COMP_Cb);
    cs.slice->ccAlfFilterControl[1] = m_ALF.getCcAlfControlIdc(COMP_Cr);

    DTRACE( g_trace_ctx, D_CRC, "ALF" );
    DTRACE_CRC( g_trace_ctx, D_CRC, cs, cs.getRecoBuf() );
    DTRACE_PIC_COMP( D_REC_CB_LUMA_ALF,   cs, cs.getRecoBuf(), COMP_Y  );
    DTRACE_PIC_COMP( D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMP_Cb );
    DTRACE_PIC_COMP( D_REC_CB_CHROMA_ALF, cs, cs.getRecoBuf(), COMP_Cr );
  }
}

void EncPicture::xSkipCompressPicture( Picture& pic, ParameterSetMap<APS>& shrdApsMap )
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
      apsMap->clear();

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
          activeAps |= aps->apsId == slice->tileGroupChromaApsId;
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


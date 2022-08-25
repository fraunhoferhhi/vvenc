/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/** \file     EncGOP.cpp
    \brief    GOP encoder class
*/

#include "EncGOP.h"
#include "CommonLib/SEI.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/TimeProfiler.h"
#include "CommonLib/MD5.h"
#include "NALwrite.h"
#include "DecoderLib/DecLib.h"
#include "BitAllocation.h"
#include "EncHRD.h"
#include "GOPCfg.h"

#include <list>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

#ifdef TRACE_ENABLE_ITT
static __itt_string_handle* itt_handle_start = __itt_string_handle_create( "Start" );
static __itt_domain* itt_domain_gopEncoder   = __itt_domain_create( "GOPEncoder" );
#endif

// ====================================================================================================================
// fast forward decoder in encoder
// ====================================================================================================================
bool isPicEncoded( int targetPoc, int curPoc, int curTLayer, int gopSize, int intraPeriod )
{
  CHECK( intraPeriod % gopSize != 0, "broken for aip" );
  int  tarGop = targetPoc / gopSize;
  int  curGop = curPoc / gopSize;

  if( tarGop + 1 == curGop )
  {
    // part of next GOP only for tl0 pics
    return curTLayer == 0;
  }

  int  tarIFr = ( targetPoc / intraPeriod ) * intraPeriod;
  int  curIFr = ( curPoc / intraPeriod ) * intraPeriod;

  if( curIFr != tarIFr )
  {
    return false;
  }

  int  tarId = targetPoc - tarGop * gopSize;

  if( tarGop > curGop )
  {
    return ( tarId == 0 ) ? ( 0 == curTLayer ) : ( 1 >= curTLayer );
  }

  if( tarGop + 1 < curGop )
  {
    return false;
  }

  int  curId = curPoc - curGop * gopSize;
  int  tarTL = 0;

  while( tarId != 0 )
  {
    gopSize /= 2;
    if( tarId >= gopSize )
    {
      tarId -= gopSize;
      if( curId != 0 ) curId -= gopSize;
    }
    else if( curId == gopSize )
    {
      curId = 0;
    }
    tarTL++;
  }

  return curTLayer <= tarTL && curId == 0;
}

void trySkipOrDecodePicture( bool& decPic, bool& encPic, const VVEncCfg& cfg, Picture* pic, FFwdDecoder& ffwdDecoder, ParameterSetMap<APS>& apsMap, MsgLog& msg )
{
  // check if we should decode a leading bitstream
  if( cfg.m_decodeBitstreams[0][0] != '\0' )
  {
    if( ffwdDecoder.bDecode1stPart )
    {
      if( cfg.m_forceDecodeBitstream1 )
      {
        if( 0 != ( ffwdDecoder.bDecode1stPart = tryDecodePicture( pic, pic->getPOC(), cfg.m_decodeBitstreams[ 0 ], ffwdDecoder, &apsMap, msg, false ) ) )
        {
          decPic = ffwdDecoder.bDecode1stPart;
        }
      }
      else
      {
        // update decode decision
        if( (0 != ( ffwdDecoder.bDecode1stPart = ( cfg.m_switchPOC != pic->getPOC() )  )) && ( 0 != ( ffwdDecoder.bDecode1stPart = tryDecodePicture( pic, pic->getPOC(), cfg.m_decodeBitstreams[ 0 ], ffwdDecoder, &apsMap, msg, false, cfg.m_switchPOC ) ) ) )
        {
          decPic = ffwdDecoder.bDecode1stPart;
          return;
        }
        else if( pic->getPOC() )
        {
          // reset decoder if used and not required any further
          tryDecodePicture( NULL, 0, std::string( "" ), ffwdDecoder, &apsMap, msg );
        }
      }
    }

    encPic |= cfg.m_forceDecodeBitstream1 && !decPic;
    if( cfg.m_forceDecodeBitstream1 ) { return; }
  }


  // check if we should decode a trailing bitstream
  if( cfg.m_decodeBitstreams[1][0] != '\0' )
  {
    CHECK( cfg.m_IntraPeriod % cfg.m_GOPSize != 0, "broken for aip" );
    const int  iNextKeyPOC    = (1+cfg.m_switchPOC  / cfg.m_GOPSize)     *cfg.m_GOPSize;
    const int  iNextIntraPOC  = (1+(cfg.m_switchPOC / cfg.m_IntraPeriod))*cfg.m_IntraPeriod;
    const int  iRestartIntraPOC   = iNextIntraPOC + (((iNextKeyPOC == iNextIntraPOC) && cfg.m_switchDQP ) ? cfg.m_IntraPeriod : 0);

    bool bDecode2ndPart = (pic->getPOC() >= iRestartIntraPOC);
    int expectedPoc = pic->getPOC();
    Slice slice0;
    if ( cfg.m_bs2ModPOCAndType )
    {
      expectedPoc = pic->getPOC() - iRestartIntraPOC;
      slice0.copySliceInfo( pic->slices[ 0 ], false );
    }
    if( bDecode2ndPart && (0 != (bDecode2ndPart = tryDecodePicture( pic, expectedPoc, cfg.m_decodeBitstreams[ 1 ], ffwdDecoder, &apsMap, msg, true )) ))
    {
      decPic = bDecode2ndPart;
      if ( cfg.m_bs2ModPOCAndType )
      {
        for( int i = 0; i < (int)pic->slices.size(); i++ )
        {
          pic->slices[ i ]->poc = slice0.poc;
          if ( pic->slices[ i ]->nalUnitType != slice0.nalUnitType
              && pic->slices[ i ]->getIdrPicFlag()
              && slice0.getRapPicFlag()
              && slice0.isIntra() )
          {
            // patch IDR-slice to CRA-Intra-slice
            pic->slices[ i ]->nalUnitType   = slice0.nalUnitType;
            pic->slices[ i ]->lastIDR       = slice0.lastIDR;
            pic->slices[ i ]->colFromL0Flag = slice0.colFromL0Flag;
            pic->slices[ i ]->colRefIdx     = slice0.colRefIdx;
          }
        }
      }
      return;
    }
  }

  // leave here if we do not use forward to poc
  if( cfg.m_fastForwardToPOC < 0 )
  {
    // let's encode
    encPic = true;
    return;
  }

  // this is the forward to poc section
  if( ffwdDecoder.bHitFastForwardPOC || isPicEncoded( cfg.m_fastForwardToPOC, pic->getPOC(), pic->TLayer, cfg.m_GOPSize, cfg.m_IntraPeriod ) )
  {
    ffwdDecoder.bHitFastForwardPOC |= cfg.m_fastForwardToPOC == pic->getPOC(); // once we hit the poc we continue encoding

    if( ffwdDecoder.bHitFastForwardPOC && cfg.m_stopAfterFFtoPOC && cfg.m_fastForwardToPOC != pic->getPOC() )
    {
      return;
    }

    //except if FastForwardtoPOC is meant to be a SwitchPOC in thist case drop all preceding pictures
    if( ffwdDecoder.bHitFastForwardPOC && ( cfg.m_switchPOC == cfg.m_fastForwardToPOC ) && ( cfg.m_fastForwardToPOC > pic->getPOC() ) )
    {
      return;
    }
    // let's encode
    encPic   = true;
  }
}


// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

EncGOP::EncGOP( MsgLog& logger )
  : msg                  ( logger )
  , m_recYuvBufFunc      ( nullptr )
  , m_recYuvBufCtx       ( nullptr )
  , m_threadPool         ( nullptr )
  , m_pcEncCfg           ( nullptr )
  , m_gopCfg             ( nullptr )
  , m_pcRateCtrl         ( nullptr )
  , m_gopApsMap          ( MAX_NUM_APS * MAX_NUM_APS_TYPE )
  , m_spsMap             ( MAX_NUM_SPS )
  , m_ppsMap             ( MAX_NUM_PPS )
  , m_isPreAnalysis      ( false )
  , m_bFirstWrite        ( true )
  , m_bRefreshPending    ( false )
  , m_disableLMCSIP      ( false )
  , m_lastCodingNum      ( -1 )
  , m_numPicsCoded       ( 0 )
  , m_pocRecOut          ( 0 )
  , m_ticksPerFrameMul4  ( 0 )
  , m_lastIDR            ( 0 )
  , m_lastRasPoc         ( MAX_INT )
  , m_pocCRA             ( 0 )
  , m_appliedSwitchDQP   ( 0 )
  , m_associatedIRAPPOC  ( 0 )
  , m_associatedIRAPType ( VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP )
  , m_trySkipOrDecodePicture( false )
{
}

EncGOP::~EncGOP()
{
  if( m_pcEncCfg && ( m_pcEncCfg->m_decodeBitstreams[0][0] != '\0' || m_pcEncCfg->m_decodeBitstreams[1][0] != '\0' ) )
  {
    // reset potential decoder resources
    tryDecodePicture( NULL, 0, std::string(""), m_ffwdDecoder, &m_gopApsMap, msg );
  }

  freePicList();

  for( auto& picEncoder : m_freePicEncoderList )
  {
    if( picEncoder )
    {
      delete picEncoder;
    }
  }
  m_freePicEncoderList.clear();
  m_threadPool = nullptr;

  // cleanup parameter sets
  m_spsMap.clearMap();
  m_ppsMap.clearMap();
}

void EncGOP::init( const VVEncCfg& encCfg, const GOPCfg* gopCfg, RateCtrl& rateCtrl, NoMallocThreadPool* threadPool, bool isPreAnalysis )
{
  m_pcEncCfg      = &encCfg;
  m_gopCfg        = gopCfg;
  m_pcRateCtrl    = &rateCtrl;
  m_threadPool    = threadPool;
  m_isPreAnalysis = isPreAnalysis;

  // setup parameter sets
  const int dciId = m_pcEncCfg->m_decodingParameterSetEnabled ? 1 : 0;
  SPS& sps0       = *( m_spsMap.allocatePS( 0 ) ); // NOTE: implementations that use more than 1 SPS need to be aware of activation issues.
  PPS& pps0       = *( m_ppsMap.allocatePS( 0 ) );

  xInitSPS( sps0 );
  sps0.dciId = m_DCI.dciId;
  xInitVPS( m_VPS );
  xInitDCI( m_DCI, sps0, dciId );
  xInitPPS( pps0, sps0 );
  xInitRPL( sps0 );
  xInitHrdParameters( sps0 );

  if( m_pcEncCfg->m_DecodingRefreshType == VVENC_DRT_IDR2 )
  {
    m_associatedIRAPType = VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL;
  }
  m_seiEncoder.init( encCfg, gopCfg, m_EncHRD );
  m_Reshaper.init  ( encCfg );

  m_appliedSwitchDQP = 0;
  const int maxPicEncoder = ( encCfg.m_maxParallelFrames ) ? encCfg.m_maxParallelFrames : 1;
  for ( int i = 0; i < maxPicEncoder; i++ )
  {
    EncPicture* picEncoder = new EncPicture;
    picEncoder->init( encCfg, &m_globalCtuQpVector, sps0, pps0, rateCtrl, threadPool );
    m_freePicEncoderList.push_back( picEncoder );
  }

  if (encCfg.m_usePerceptQPA)
  {
    m_globalCtuQpVector.resize( pps0.useDQP && (encCfg.m_usePerceptQPATempFiltISlice == 2) ? pps0.picWidthInCtu * pps0.picHeightInCtu + 1 : 1 );
  }

  if( m_pcEncCfg->m_FrameRate )
  {
    m_ticksPerFrameMul4 = (int)((int64_t)4 *(int64_t)m_pcEncCfg->m_TicksPerSecond * (int64_t)m_pcEncCfg->m_FrameScale/(int64_t)m_pcEncCfg->m_FrameRate);
  }

  m_trySkipOrDecodePicture = ( m_pcEncCfg->m_decodeBitstreams[0][0] != '\0' || m_pcEncCfg->m_decodeBitstreams[1][0] != '\0' )
                            && m_pcRateCtrl->rcIsFinalPass
                            && ( m_pcEncCfg->m_RCTargetBitrate > 0 || !m_isPreAnalysis );
}


// ====================================================================================================================
// Class interface
// ====================================================================================================================


void EncGOP::setRecYUVBufferCallback( void* ctx, std::function<void( void*, vvencYUVBuffer* )> func )
{
  m_recYuvBufCtx  = ctx;
  m_recYuvBufFunc = func;
}

void EncGOP::initPicture( Picture* pic )
{
  pic->encTime.startTimer();

  pic->TLayer = pic->gopEntry->m_temporalId;

  pic->setSccFlags( m_pcEncCfg );

  CHECK( m_ppsMap.getFirstPS() == nullptr || m_spsMap.getPS( m_ppsMap.getFirstPS()->spsId ) == nullptr, "picture set not initialised" );

  const PPS& pps = *( m_ppsMap.getFirstPS() );
  const SPS& sps = *( m_spsMap.getPS( pps.spsId ) );

  if( pic->cs && pic->cs->picHeader )
  {
    delete pic->cs->picHeader;
    pic->cs->picHeader = nullptr;
  }

  std::mutex* mutex = ( m_pcEncCfg->m_maxParallelFrames ) ? &m_unitCacheMutex : nullptr;
  pic->finalInit( m_VPS, sps, pps, nullptr, m_shrdUnitCache, mutex, nullptr, nullptr );

  pic->vps = &m_VPS;
  pic->dci = &m_DCI;

  // filter data initialization
  const uint32_t numberOfCtusInFrame = pic->cs->pcv->sizeInCtus;

  if( m_pcEncCfg->m_usePerceptQPA )
  {
    pic->ctuQpaLambda.resize (numberOfCtusInFrame);
    pic->ctuAdaptedQP.resize (numberOfCtusInFrame);
  }

  if( pic->cs->sps->saoEnabled )
  {
    pic->resizeSAO( numberOfCtusInFrame, 0 );
    pic->resizeSAO( numberOfCtusInFrame, 1 );
  }

  if( pic->cs->sps->alfEnabled )
  {
    pic->resizeAlfCtuBuffers( numberOfCtusInFrame );
  }

  pic->encTime.stopTimer();
}

void EncGOP::waitForFreeEncoders()
{
  {
    std::unique_lock<std::mutex> lock( m_gopEncMutex );
    if( ! xEncodersFinished() )
    {
      CHECK( m_pcEncCfg->m_numThreads <= 0, "run into MT code, but no threading enabled" );
      m_gopEncCond.wait( lock );
    }
  }
}

void EncGOP::processPictures( const PicList& picList, bool flush, AccessUnitList& auList, PicList& doneList, PicList& freeList )
{
  CHECK( picList.empty(), "empty input picture list given" );

  // create list of pictures ordered in coding order and ready to be encoded
  xInitPicsInCodingOrder( picList, flush );

  // encode pictures
  xProcessPictures( flush, auList, doneList );
  // output reconstructed YUV
  xOutputRecYuv( picList );

  // release pictures not needed anymore
  const bool allDone = flush && m_numPicsCoded >= m_picCount;
  xReleasePictures( picList, freeList, allDone );

  // clear output access unit
  if( m_isPreAnalysis )
  {
    auList.clearAu();
  }
}

void EncGOP::xProcessPictures( bool flush, AccessUnitList& auList, PicList& doneList )
{
  // in lockstep mode, process all pictures in processing list
  const bool lockStepMode = (m_pcEncCfg->m_RCTargetBitrate > 0 || (m_pcEncCfg->m_LookAhead > 0 && !m_isPreAnalysis)) && (m_pcEncCfg->m_maxParallelFrames > 0);

  // get list of pictures to be encoded and used for RC update
  if( m_procList.empty() && ! m_gopEncListInput.empty() )
  {
    xGetProcessingLists( m_procList, m_rcUpdateList, lockStepMode );
  }

  if( ! m_procList.empty() )
  {
    // encode one picture in serial mode / multiple pictures in FPP mode
    PROFILER_ACCUM_AND_START_NEW_SET( 1, g_timeProfiler, P_IGNORE );
    while( true )
    {
      Picture* pic           = nullptr;
      EncPicture* picEncoder = nullptr;

      // fetch next picture to be encoded and next free picture encoder
      {
        std::unique_lock<std::mutex> lock( m_gopEncMutex, std::defer_lock );
        if( m_pcEncCfg->m_numThreads > 0) lock.lock();

        // leave the loop when nothing to do (when all encoders are finished or in non-blocking mode)
        if( m_procList.empty() && ( isNonBlocking() || xEncodersFinished() ) ) 
        {
          break;
        }

        // get next picture ready to be encoded
        auto picItr             = find_if( m_procList.begin(), m_procList.end(), []( auto pic ) { return pic->slices[ 0 ]->checkRefPicsReconstructed(); } );
        const bool nextPicReady = picItr != m_procList.end();

        // check at least one picture and one pic encoder ready
        if( m_freePicEncoderList.empty() || ! nextPicReady )
        {
          // non-blocking stage: wait on top level, let other stages do their jobs
          // in non-lockstep mode, check if next picture can be output
          if( isNonBlocking() || ( ! lockStepMode && m_gopEncListOutput.front()->isReconstructed ) )
          {
            break;
          }
          CHECK( m_pcEncCfg->m_numThreads <= 0, "run into MT code, but no threading enabled" );
          CHECK( xEncodersFinished(), "wait for picture to be finished, but no pic encoder running" );
          m_gopEncCond.wait( lock );
          continue;
        }

        pic = *picItr;
        picEncoder = m_freePicEncoderList.front();

        // rate-control with look-ahead: init next chunk
        if( m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_LookAhead )
        {
          CHECK( m_isPreAnalysis, "rate control enabled for pre analysis" );

          if( pic->gopEntry->m_isStartOfGop )
          {
            // check the RC final pass requirement for availability of preprocessed pictures (GOP + 1)
            if( m_pcRateCtrl->lastPOCInCache() <= pic->poc && !flush )
            {
              break;
            }
            m_pcRateCtrl->processFirstPassData( flush, pic->poc );
          }
        }

        m_freePicEncoderList.pop_front();
      }

      CHECK( picEncoder == nullptr, "no free picture encoder available" );
      CHECK( pic        == nullptr, "no picture to be encoded, ready for encoding" );
      m_procList.remove( pic );

      xEncodePicture( pic, picEncoder );
    }
  }

  // picture/AU output
  // 
  // in lock-step mode:
  // the output of a picture is connected to evaluation of the lock-step-chunk
  // if the next picture to output belongs to the current chunk, do output (evaluation) when all pictures of the chunk are finished

  if( m_gopEncListOutput.empty() || !m_gopEncListOutput.front()->isReconstructed ||
    ( lockStepMode && !m_rcUpdateList.empty() && m_gopEncListOutput.front() == m_rcUpdateList.front() && !xEncodersFinished() ) )
  {
    return;
  }
  PROFILER_ACCUM_AND_START_NEW_SET( 1, g_timeProfiler, P_TOP_LEVEL );

  // AU output
  Picture* outPic = m_gopEncListOutput.front();
  m_gopEncListOutput.pop_front();

  if( outPic->writePic )
  {
    xWritePicture( *outPic, auList, false );
  }

  if( m_pcEncCfg->m_alfTempPred )
  {
    xSyncAlfAps( *outPic, m_gopApsMap, outPic->picApsMap );
  }

  // update pending RC
  // first pic has been written to bitstream
  // therefore we have at least for this picture a valid total bit and head bit count
  if( !m_rcUpdateList.empty() && m_rcUpdateList.front() == outPic )
  {
    if( m_pcEncCfg->m_RCTargetBitrate > 0 )
    {
      for( auto pic : m_rcUpdateList )
      {
        if( pic != outPic )
        {
          pic->actualHeadBits = outPic->actualHeadBits;
          pic->actualTotalBits = pic->sliceDataStreams[0].getNumberOfWrittenBits();
        }
        m_pcRateCtrl->xUpdateAfterPicRC( pic );
      }
    }

    if( lockStepMode )
      m_rcUpdateList.clear();
    else
      m_rcUpdateList.pop_front();
  }

  if( m_pcEncCfg->m_useAMaxBT )
  {
    m_BlkStat.updateMaxBT( *outPic->slices[0], outPic->picBlkStat );
  }

  outPic->slices[ 0 ]->updateRefPicCounter( -1 );
  outPic->isFinished = true;

  if( ! m_isPreAnalysis )
  {
    outPic->getFilteredOrigBuffer().destroy();
  }

  doneList.push_back( outPic );

  m_numPicsCoded += 1;
}

void EncGOP::xEncodePicture( Picture* pic, EncPicture* picEncoder )
{
  // decoder in encoder
  bool decPic = false;
  bool encPic = false;
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "finalpass", m_pcRateCtrl->rcIsFinalPass ? 1: 0 ) );
  if( m_trySkipOrDecodePicture )
  {
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "encdec", 1 ) );
    trySkipOrDecodePicture( decPic, encPic, *m_pcEncCfg, pic, m_ffwdDecoder, m_gopApsMap, msg );
    if( !encPic && m_pcEncCfg->m_RCTargetBitrate > 0 )
    {
      pic->picInitialQP     = -1;
      pic->picInitialLambda = -1.0;
      m_pcRateCtrl->initRateControlPic( *pic, pic->slices[0], pic->picInitialQP, pic->picInitialLambda );
    }
  }
  else
  {
    encPic = true;
  }
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "encdec", 0 ) );
  pic->writePic = decPic || encPic;
  pic->encPic   = encPic;
  pic->isPreAnalysis = m_isPreAnalysis;

  if( m_pcEncCfg->m_alfTempPred || !encPic )
  {
    xSyncAlfAps( *pic, pic->picApsMap, m_gopApsMap );
  }

  // compress next picture
  if( pic->encPic )
  {
    picEncoder->compressPicture( *pic, *this );
  }
  else
  {
    picEncoder->skipCompressPicture( *pic, m_gopApsMap );
  }

  // finish picture encoding and cleanup
  if( pic->encPic && m_pcEncCfg->m_numThreads > 0 )
  {
    static auto finishTask = []( int, FinishTaskParam* param ) {
      param->picEncoder->finalizePicture( *param->pic );
      {
        std::lock_guard<std::mutex> lock( param->gopEncoder->m_gopEncMutex );
        param->pic->isReconstructed = true;
        param->gopEncoder->m_freePicEncoderList.push_back( param->picEncoder );
        param->gopEncoder->m_gopEncCond.notify_one();
      }
      delete param;
      return true;
    };
    FinishTaskParam* param = new FinishTaskParam( this, picEncoder, pic );
    m_threadPool->addBarrierTask<FinishTaskParam>( finishTask, param, nullptr, nullptr, { &picEncoder->m_ctuTasksDoneCounter.done } );
  }
  else
  {
    picEncoder->finalizePicture( *pic );
    pic->isReconstructed = true;
    m_freePicEncoderList.push_back( picEncoder );
  }
}

void EncGOP::xOutputRecYuv( const PicList& picList )
{
  if( m_pcRateCtrl->rcIsFinalPass && m_recYuvBufFunc )
  {
    CHECK( m_isPreAnalysis, "yuv output enabled for pre analysis" );
    // ordered YUV output
    bool bRun = true;
    while( bRun )
    {
      bRun = false;
      for( auto pic : picList )
      {
        if( pic->poc != m_pocRecOut )
          continue;
        if( ! pic->isReconstructed )
          return;

        const PPS& pps = *(pic->cs->pps);
        vvencYUVBuffer yuvBuffer;
        vvenc_YUVBuffer_default( &yuvBuffer );
        setupYuvBuffer( pic->getRecoBuf(), yuvBuffer, &pps.conformanceWindow );
        m_recYuvBufFunc( m_recYuvBufCtx, &yuvBuffer );

        m_pocRecOut += 1;
        pic->isNeededForOutput = false;
        bRun = true;
        break;
      }
    }
  }
  else
  {
    // no output needed, simply unmark pictures
    for( auto pic : picList )
    {
      if( pic->isReconstructed && pic->isNeededForOutput )
        pic->isNeededForOutput = false;
    }
  }
}

void EncGOP::xReleasePictures( const PicList& picList, PicList& freeList, bool allDone )
{
  for( auto pic : picList )
  {
    if( allDone || ( pic->isFinished && ! pic->isNeededForOutput && ! pic->isReferenced && pic->refCounter <= 0 ) )
      freeList.push_back( pic );
  }
}

void EncGOP::printOutSummary( const bool printMSEBasedSNR, const bool printSequenceMSE, const bool printHexPsnr )
{

  if( m_pcEncCfg->m_decodeBitstreams[0][0] != '\0' && m_pcEncCfg->m_decodeBitstreams[1][0] != '\0' && m_pcEncCfg->m_fastForwardToPOC < 0 )
  {
    CHECK( !( m_numPicsCoded == m_AnalyzeAll.getNumPic() ), "Unspecified error" );
  }

  //--CFG_KDY
  //const int rateMultiplier = 1;
  double fps = m_pcEncCfg->m_FrameRate/(double)m_pcEncCfg->m_FrameScale / (double)m_pcEncCfg->m_temporalSubsampleRatio;
  m_AnalyzeAll.setFrmRate( fps );
  m_AnalyzeI.setFrmRate( fps );
  m_AnalyzeP.setFrmRate( fps );
  m_AnalyzeB.setFrmRate( fps );

  const ChromaFormat chFmt = m_pcEncCfg->m_internChromaFormat;

  const BitDepths& bitDepths = m_spsMap.getFirstPS()->bitDepths;
  //-- all
  std::string summary = "\n";
  if( m_pcEncCfg->m_verbosity >= VVENC_DETAILS )
    summary.append("\nSUMMARY --------------------------------------------------------\n");

  summary.append( m_AnalyzeAll.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths));

  if( m_pcEncCfg->m_verbosity < VVENC_DETAILS )
  {
    msg.log( VVENC_INFO,summary.c_str() );
  }
  else
  {
    summary.append( "\n\nI Slices--------------------------------------------------------\n" );
    summary.append( m_AnalyzeI.printOut('i', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths));

    summary.append( "\n\nP Slices--------------------------------------------------------\n" );
    summary.append( m_AnalyzeP.printOut('p', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths));

    summary.append( "\n\nB Slices--------------------------------------------------------\n" );
    summary.append( m_AnalyzeB.printOut('b', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths));
    msg.log( VVENC_DETAILS,summary.c_str() );
  }

  if (m_pcEncCfg->m_summaryOutFilename[0] != '\0' )
  {
    std::string summaryOutFilename(m_pcEncCfg->m_summaryOutFilename);
    m_AnalyzeAll.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, summaryOutFilename);
  }

  if (m_pcEncCfg->m_summaryPicFilenameBase[0] != '\0' )
  {
    std::string summaryPicFilenameBase(m_pcEncCfg->m_summaryPicFilenameBase);

    m_AnalyzeI.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, summaryPicFilenameBase+"I.txt");
    m_AnalyzeP.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, summaryPicFilenameBase+"P.txt");
    m_AnalyzeB.printSummary(chFmt, printSequenceMSE, printHexPsnr, bitDepths, summaryPicFilenameBase+"B.txt");
  }
}

void EncGOP::getParameterSets( AccessUnitList& accessUnit )
{
  CHECK( m_ppsMap.getFirstPS() == nullptr || m_spsMap.getPS( m_ppsMap.getFirstPS()->spsId ) == nullptr, "sps/pps not initialised" );

  const PPS& pps = *( m_ppsMap.getFirstPS() );
  const SPS& sps = *( m_spsMap.getPS( pps.spsId ) );

  if (sps.vpsId != 0)
  {
    xWriteVPS( accessUnit, &m_VPS, m_HLSWriter );
  }
  xWriteDCI( accessUnit, &m_DCI, m_HLSWriter );
  xWriteSPS( accessUnit, &sps, m_HLSWriter );
  xWritePPS( accessUnit, &pps, &sps, m_HLSWriter );
}

void EncGOP::xUpdateRasInit( Slice* slice )
{
  slice->pendingRasInit = false;
  if ( slice->poc > m_lastRasPoc )
  {
    m_lastRasPoc = MAX_INT;
    slice->pendingRasInit = true;
  }
  if ( slice->isIRAP() )
  {
    m_lastRasPoc = slice->poc;
  }
}

void EncGOP::xInitVPS(VPS &vps) const
{
  // The SPS must have already been set up.
  // set the VPS profile information.
  vps.maxLayers                   = 1;
  vps.maxSubLayers                = 1;
  vps.vpsId                       = 0;
  vps.allLayersSameNumSubLayers   = true;
  vps.allIndependentLayers        = true;
  vps.eachLayerIsAnOls            = true;
  vps.olsModeIdc                  = 0;
  vps.numOutputLayerSets          = 1;
  vps.numPtls                     = 1;
  vps.extension                   = false;
  vps.totalNumOLSs                = 0;
  vps.numDpbParams                = 0;
  vps.sublayerDpbParamsPresent    = false;
  vps.targetOlsIdx                = -1;

  for (int i = 0; i < MAX_VPS_LAYERS; i++)
  {
    vps.layerId[i]                = 0;
    vps.independentLayer[i]       = true;
    for (int j = 0; j < MAX_VPS_LAYERS; j++)
    {
      vps.directRefLayer[i][j]    = 0;
      vps.directRefLayerIdx[i][j] = MAX_VPS_LAYERS;
      vps.interLayerRefIdx[i][i]  = NOT_VALID;
    }
  }

  for (int i = 0; i < MAX_NUM_OLSS; i++)
  {
    for (int j = 0; j < MAX_VPS_LAYERS; j++)
    {
      vps.olsOutputLayer[i][j]    = 0;
    }
    vps.ptPresent[i]              = (i == 0) ? 1 : 0;
    vps.ptlMaxTemporalId[i]       = vps.maxSubLayers - 1;
    vps.olsPtlIdx[i]              = 0;
  }

  vps.profileTierLevel.resize( 1 );
}

void EncGOP::xInitDCI(DCI &dci, const SPS &sps, const int dciId) const
{
  // The SPS must have already been set up.
  // set the DPS profile information.
  dci.dciId                 = dciId;

  dci.profileTierLevel.resize(1);
  // copy profile level tier info
  dci.profileTierLevel[0]   = sps.profileTierLevel;
}

void EncGOP::xInitConstraintInfo(ConstraintInfo &ci) const
{
  ci.intraOnlyConstraintFlag                      = m_pcEncCfg->m_intraOnlyConstraintFlag;
  ci.maxBitDepthConstraintIdc                     = m_pcEncCfg->m_bitDepthConstraintValue - 8;
  ci.maxChromaFormatConstraintIdc                 = m_pcEncCfg->m_internChromaFormat;
  ci.onePictureOnlyConstraintFlag                 = false;
  ci.lowerBitRateConstraintFlag                   = false;
  ci.allLayersIndependentConstraintFlag           = false;
  ci.noMrlConstraintFlag                          = false;
  ci.noIspConstraintFlag                          = false;
  ci.noMipConstraintFlag                          = false;
  ci.noLfnstConstraintFlag                        = false;
  ci.noMmvdConstraintFlag                         = false;
  ci.noSmvdConstraintFlag                         = false;
  ci.noProfConstraintFlag                         = false;
  ci.noPaletteConstraintFlag                      = false;
  ci.noActConstraintFlag                          = false;
  ci.noLmcsConstraintFlag                         = false;
  ci.noQtbttDualTreeIntraConstraintFlag           = ! m_pcEncCfg->m_dualITree;
  ci.noPartitionConstraintsOverrideConstraintFlag = false;
  ci.noSaoConstraintFlag                          = ! m_pcEncCfg->m_bUseSAO;
  ci.noAlfConstraintFlag                          = ! m_pcEncCfg->m_alf;
  ci.noCCAlfConstraintFlag                        = ! m_pcEncCfg->m_ccalf;
  ci.noRefWraparoundConstraintFlag                = false;
  ci.noTemporalMvpConstraintFlag                  = m_pcEncCfg->m_TMVPModeId == 0;
  ci.noSbtmvpConstraintFlag                       = !m_pcEncCfg->m_SbTMVP;
  ci.noAmvrConstraintFlag                         = false;
  ci.noBdofConstraintFlag                         = ! m_pcEncCfg->m_BDOF;
  ci.noDmvrConstraintFlag                         = ! m_pcEncCfg->m_DMVR;
  ci.noCclmConstraintFlag                         = ! m_pcEncCfg->m_LMChroma;
  ci.noMtsConstraintFlag                          = !(m_pcEncCfg->m_MTSImplicit || m_pcEncCfg->m_MTS);
  ci.noSbtConstraintFlag                          = m_pcEncCfg->m_SBT == 0;
  ci.noAffineMotionConstraintFlag                 = ! m_pcEncCfg->m_Affine;
  ci.noBcwConstraintFlag                          = true;
  ci.noIbcConstraintFlag                          = m_pcEncCfg->m_IBCMode == 0;
  ci.noCiipConstraintFlag                         = m_pcEncCfg->m_CIIP == 0;
  ci.noGeoConstraintFlag                          = m_pcEncCfg->m_Geo == 0;
  ci.noLadfConstraintFlag                         = true;
  ci.noTransformSkipConstraintFlag                = m_pcEncCfg->m_TS == 0;
  ci.noBDPCMConstraintFlag                        = m_pcEncCfg->m_useBDPCM==0;
  ci.noJointCbCrConstraintFlag                    = ! m_pcEncCfg->m_JointCbCrMode;
  ci.noMrlConstraintFlag                          = ! m_pcEncCfg->m_MRL;
  ci.noIspConstraintFlag                          = true;
  ci.noMipConstraintFlag                          = ! m_pcEncCfg->m_MIP;
  ci.noQpDeltaConstraintFlag                      = false;
  ci.noDepQuantConstraintFlag                     = ! m_pcEncCfg->m_DepQuantEnabled;
  ci.noMixedNaluTypesInPicConstraintFlag          = false;
  ci.noSignDataHidingConstraintFlag               = ! m_pcEncCfg->m_SignDataHidingEnabled;
  ci.noLfnstConstraintFlag                        = ! m_pcEncCfg->m_LFNST;
  ci.noMmvdConstraintFlag                         = ! m_pcEncCfg->m_MMVD;
  ci.noSmvdConstraintFlag                         = ! m_pcEncCfg->m_SMVD;
  ci.noProfConstraintFlag                         = ! m_pcEncCfg->m_PROF;
  ci.noPaletteConstraintFlag                      = true;
  ci.noActConstraintFlag                          = true;
  ci.noLmcsConstraintFlag                         = m_pcEncCfg->m_lumaReshapeEnable == 0;
  ci.noTrailConstraintFlag                        = m_pcEncCfg->m_IntraPeriod == 1;
  ci.noStsaConstraintFlag                         = m_pcEncCfg->m_IntraPeriod == 1 || ! m_gopCfg->hasNonZeroTemporalId();
  ci.noRaslConstraintFlag                         = m_pcEncCfg->m_IntraPeriod == 1 || ! m_gopCfg->hasLeadingPictures();
  ci.noRadlConstraintFlag                         = m_pcEncCfg->m_IntraPeriod == 1 || ! m_gopCfg->hasLeadingPictures();
  ci.noIdrConstraintFlag                          = false;
  ci.noCraConstraintFlag                          = (m_pcEncCfg->m_DecodingRefreshType != VVENC_DRT_CRA && m_pcEncCfg->m_DecodingRefreshType != VVENC_DRT_CRA_CRE);
  ci.noGdrConstraintFlag                          = false;
  ci.noApsConstraintFlag                          = ( !m_pcEncCfg->m_alf && m_pcEncCfg->m_lumaReshapeEnable == 0 /*&& m_useScalingListId == SCALING_LIST_OFF*/);
}

void EncGOP::xInitSPS(SPS &sps) const
{
  ProfileTierLevel* profileTierLevel = &sps.profileTierLevel;

  xInitConstraintInfo( profileTierLevel->constraintInfo );

  profileTierLevel->levelIdc      = m_pcEncCfg->m_level;
  profileTierLevel->tierFlag      = m_pcEncCfg->m_levelTier;
  profileTierLevel->profileIdc    = m_pcEncCfg->m_profile;
  profileTierLevel->subProfileIdc.clear();
  profileTierLevel->subProfileIdc.push_back( m_pcEncCfg->m_subProfile );

  sps.maxPicWidthInLumaSamples      = m_pcEncCfg->m_PadSourceWidth;
  sps.maxPicHeightInLumaSamples     = m_pcEncCfg->m_PadSourceHeight;
  sps.conformanceWindow.setWindow( m_pcEncCfg->m_confWinLeft, m_pcEncCfg->m_confWinRight, m_pcEncCfg->m_confWinTop, m_pcEncCfg->m_confWinBottom );
  sps.chromaFormatIdc               = m_pcEncCfg->m_internChromaFormat;
  sps.CTUSize                       = m_pcEncCfg->m_CTUSize;
  sps.maxMTTDepth[0]                = m_pcEncCfg->m_maxMTTDepthI;
  sps.maxMTTDepth[1]                = m_pcEncCfg->m_maxMTTDepth >= 10 ? 3 : m_pcEncCfg->m_maxMTTDepth;
  sps.maxMTTDepth[2]                = m_pcEncCfg->m_maxMTTDepthIChroma;
  for( int i = 0; i < 3; i++)
  {
    sps.minQTSize[i]                = m_pcEncCfg->m_MinQT[i];
    sps.maxBTSize[i]                = m_pcEncCfg->m_maxBT[i];
    sps.maxTTSize[i]                = m_pcEncCfg->m_maxTT[i];
  }
  sps.minQTSize[2]                <<= getChannelTypeScaleX(CH_C, m_pcEncCfg->m_internChromaFormat);

  sps.maxNumMergeCand               = m_pcEncCfg->m_maxNumMergeCand;
  sps.maxNumAffineMergeCand         = m_pcEncCfg->m_Affine ? m_pcEncCfg->m_maxNumAffineMergeCand : 0;
  sps.maxNumGeoCand                 = m_pcEncCfg->m_maxNumGeoCand;
  sps.IBC                           = m_pcEncCfg->m_IBCMode != 0;
  sps.maxNumIBCMergeCand            = 6;

  sps.idrRefParamList               = m_pcEncCfg->m_idrRefParamList;
  sps.dualITree                     = m_pcEncCfg->m_dualITree && m_pcEncCfg->m_internChromaFormat != VVENC_CHROMA_400;
  sps.MTS                           = m_pcEncCfg->m_MTS || m_pcEncCfg->m_MTSImplicit;
  sps.SMVD                          = m_pcEncCfg->m_SMVD;
  sps.AMVR                          = m_pcEncCfg->m_AMVRspeed != IMV_OFF;
  sps.LMChroma                      = m_pcEncCfg->m_LMChroma;
  sps.horCollocatedChroma           = m_pcEncCfg->m_horCollocatedChromaFlag;
  sps.verCollocatedChroma           = m_pcEncCfg->m_verCollocatedChromaFlag;
  sps.BDOF                          = m_pcEncCfg->m_BDOF;
  sps.DMVR                          = m_pcEncCfg->m_DMVR;
  sps.lumaReshapeEnable             = m_pcEncCfg->m_lumaReshapeEnable != 0;
  sps.Affine                        = m_pcEncCfg->m_Affine;
  sps.PROF                          = m_pcEncCfg->m_PROF;
  sps.ProfPresent                   = m_pcEncCfg->m_PROF;
  sps.AffineType                    = m_pcEncCfg->m_AffineType;
  sps.MMVD                          = m_pcEncCfg->m_MMVD != 0;
  sps.fpelMmvd                      = m_pcEncCfg->m_allowDisFracMMVD;
  sps.GEO                           = m_pcEncCfg->m_Geo != 0;
  sps.MIP                           = m_pcEncCfg->m_MIP;
  sps.MRL                           = m_pcEncCfg->m_MRL;
  sps.BdofPresent                   = m_pcEncCfg->m_BDOF;
  sps.DmvrPresent                   = m_pcEncCfg->m_DMVR;
  sps.partitionOverrideEnabled      = true; // needed for the new MaxMTTDepth logic
  sps.resChangeInClvsEnabled        = m_pcEncCfg->m_resChangeInClvsEnabled;
  sps.rprEnabled                    = m_pcEncCfg->m_rprEnabledFlag != 0;
  sps.log2MinCodingBlockSize        = m_pcEncCfg->m_log2MinCodingBlockSize;
  sps.log2MaxTbSize                 = m_pcEncCfg->m_log2MaxTbSize;
  sps.temporalMVPEnabled            = m_pcEncCfg->m_TMVPModeId == 2 || m_pcEncCfg->m_TMVPModeId == 1;
  sps.LFNST                         = m_pcEncCfg->m_LFNST != 0;
  sps.entropyCodingSyncEnabled      = m_pcEncCfg->m_entropyCodingSyncEnabled;
  sps.entryPointsPresent            = m_pcEncCfg->m_entryPointsPresent;
  sps.depQuantEnabled               = m_pcEncCfg->m_DepQuantEnabled;
  sps.signDataHidingEnabled         = m_pcEncCfg->m_SignDataHidingEnabled;
  sps.MTSIntra                      = m_pcEncCfg->m_MTS ;
  sps.ISP                           = m_pcEncCfg->m_ISP;
  sps.transformSkip                 = m_pcEncCfg->m_TS != 0;
  sps.log2MaxTransformSkipBlockSize = m_pcEncCfg->m_TSsize;
  sps.BDPCM                         = m_pcEncCfg->m_useBDPCM != 0;
  sps.BCW                           = m_pcEncCfg->m_BCW;

  for (uint32_t chType = 0; chType < MAX_NUM_CH; chType++)
  {
    sps.bitDepths.recon[chType]     = m_pcEncCfg->m_internalBitDepth[chType];
    sps.qpBDOffset[chType]          = 6 * (m_pcEncCfg->m_internalBitDepth[chType] - 8);
    sps.internalMinusInputBitDepth[chType] = std::max(0, (m_pcEncCfg->m_internalBitDepth[chType] - m_pcEncCfg->m_inputBitDepth[chType]));
  }

  sps.alfEnabled                    = m_pcEncCfg->m_alf;
  sps.ccalfEnabled                  = m_pcEncCfg->m_ccalf && m_pcEncCfg->m_internChromaFormat != VVENC_CHROMA_400;

  sps.saoEnabled                    = m_pcEncCfg->m_bUseSAO;
  sps.jointCbCr                     = m_pcEncCfg->m_JointCbCrMode;
  sps.maxTLayers                    = m_pcEncCfg->m_maxTLayer + 1;
  sps.rpl1CopyFromRpl0              = ! m_pcEncCfg->m_picReordering;
  sps.SbtMvp                        = m_pcEncCfg->m_SbTMVP;
  sps.CIIP                          = m_pcEncCfg->m_CIIP != 0;
  sps.SBT                           = m_pcEncCfg->m_SBT != 0;

  CHECK( sps.maxTLayers > VVENC_MAX_TLAYER, "array index out of bounds" );
  for( int i = 0; i < sps.maxTLayers; i++ )
  {
    sps.maxDecPicBuffering[ i ]     = m_gopCfg->getMaxDecPicBuffering()[ i ];
    sps.numReorderPics[ i ]         = m_gopCfg->getNumReorderPics()[ i ];
  }

  sps.vuiParametersPresent          = m_pcEncCfg->m_vuiParametersPresent;

  if (sps.vuiParametersPresent)
  {
    VUI& vui = sps.vuiParameters;
    vui.aspectRatioInfoPresent        = m_pcEncCfg->m_aspectRatioInfoPresent;
    vui.aspectRatioIdc                = m_pcEncCfg->m_aspectRatioIdc;
    vui.sarWidth                      = m_pcEncCfg->m_sarWidth;
    vui.sarHeight                     = m_pcEncCfg->m_sarHeight;
    vui.colourDescriptionPresent      = m_pcEncCfg->m_colourDescriptionPresent;
    vui.colourPrimaries               = m_pcEncCfg->m_colourPrimaries;
    vui.transferCharacteristics       = m_pcEncCfg->m_transferCharacteristics;
    vui.matrixCoefficients            = m_pcEncCfg->m_matrixCoefficients;
    vui.chromaLocInfoPresent          = m_pcEncCfg->m_chromaLocInfoPresent;
    vui.chromaSampleLocTypeTopField   = m_pcEncCfg->m_chromaSampleLocTypeTopField;
    vui.chromaSampleLocTypeBottomField= m_pcEncCfg->m_chromaSampleLocTypeBottomField;
    vui.chromaSampleLocType           = m_pcEncCfg->m_chromaSampleLocType;
    vui.overscanInfoPresent           = m_pcEncCfg->m_overscanInfoPresent;
    vui.overscanAppropriateFlag       = m_pcEncCfg->m_overscanAppropriateFlag;
    vui.videoFullRangeFlag            = m_pcEncCfg->m_videoFullRangeFlag;
  }

  sps.hrdParametersPresent            = m_pcEncCfg->m_hrdParametersPresent;

  sps.numLongTermRefPicSPS            = NUM_LONG_TERM_REF_PIC_SPS;
  CHECK(!(NUM_LONG_TERM_REF_PIC_SPS <= MAX_NUM_LONG_TERM_REF_PICS), "Unspecified error");
  for (int k = 0; k < NUM_LONG_TERM_REF_PIC_SPS; k++)
  {
    sps.ltRefPicPocLsbSps[k]          = 0;
    sps.usedByCurrPicLtSPS[k]         = 0;
  }
  sps.chromaQpMappingTable.m_numQpTables = (m_pcEncCfg->m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag ? 1 : (sps.jointCbCr ? 3 : 2));
  sps.chromaQpMappingTable.setParams(m_pcEncCfg->m_chromaQpMappingTableParams, sps.qpBDOffset[ CH_C ]);
  sps.chromaQpMappingTable.derivedChromaQPMappingTables();
}

void EncGOP::xInitPPS(PPS &pps, const SPS &sps) const
{
  bool bUseDQP = m_pcEncCfg->m_cuQpDeltaSubdiv > 0;
  bUseDQP |= m_pcEncCfg->m_lumaLevelToDeltaQPEnabled;
  bUseDQP |= m_pcEncCfg->m_usePerceptQPA;

  if (m_pcEncCfg->m_costMode==VVENC_COST_SEQUENCE_LEVEL_LOSSLESS || m_pcEncCfg->m_costMode==VVENC_COST_LOSSLESS_CODING)
  {
    bUseDQP = false;
  }

  // pps ID already initialised.
  pps.spsId                         = sps.spsId;
  pps.jointCbCrQpOffsetPresent      = m_pcEncCfg->m_JointCbCrMode;
  pps.picWidthInLumaSamples         = m_pcEncCfg->m_PadSourceWidth;
  pps.picHeightInLumaSamples        = m_pcEncCfg->m_PadSourceHeight;
  if( pps.picWidthInLumaSamples == sps.maxPicWidthInLumaSamples && pps.picHeightInLumaSamples == sps.maxPicHeightInLumaSamples )
  {
    pps.conformanceWindow           = sps.conformanceWindow;
  }
  else
  {
    pps.conformanceWindow.setWindow( m_pcEncCfg->m_confWinLeft, m_pcEncCfg->m_confWinRight, m_pcEncCfg->m_confWinTop, m_pcEncCfg->m_confWinBottom );
  }

  pps.picWidthInCtu                 = (pps.picWidthInLumaSamples + (sps.CTUSize-1)) / sps.CTUSize;
  pps.picHeightInCtu                = (pps.picHeightInLumaSamples + (sps.CTUSize-1)) / sps.CTUSize;
  pps.subPics.clear();
  pps.subPics.resize(1);
  pps.subPics[0].init( pps.picWidthInCtu, pps.picHeightInCtu, pps.picWidthInLumaSamples, pps.picHeightInLumaSamples);
  pps.useDQP                        = m_pcEncCfg->m_RCTargetBitrate > 0 ? true : bUseDQP;

  if ( m_pcEncCfg->m_cuChromaQpOffsetSubdiv >= 0 )
  {
    //th check how this is configured now    pps.cuChromaQpOffsetSubdiv = m_pcEncCfg->m_cuChromaQpOffsetSubdiv;
    pps.chromaQpOffsetListLen = 0;
    pps.setChromaQpOffsetListEntry(1, 6, 6, 6);
  }

  {
    int baseQp = m_pcEncCfg->m_QP-26;
    if( 16 == m_pcEncCfg->m_GOPSize )
    {
      baseQp += 2;
    }

    const int maxDQP = 37;
    const int minDQP = -26 + sps.qpBDOffset[ CH_L ];
    pps.picInitQPMinus26 = std::min( maxDQP, std::max( minDQP, baseQp ) );
  }

  if (m_pcEncCfg->m_wcgChromaQpControl.enabled )
  {
    const int baseQp      = m_pcEncCfg->m_QP + pps.ppsId;
    const double chromaQp = m_pcEncCfg->m_wcgChromaQpControl.chromaQpScale * baseQp + m_pcEncCfg->m_wcgChromaQpControl.chromaQpOffset;
    const double dcbQP    = m_pcEncCfg->m_wcgChromaQpControl.chromaCbQpScale * chromaQp;
    const double dcrQP    = m_pcEncCfg->m_wcgChromaQpControl.chromaCrQpScale * chromaQp;
    const int cbQP        = std::min(0, (int)(dcbQP + ( dcbQP < 0 ? -0.5 : 0.5) ));
    const int crQP        = std::min(0, (int)(dcrQP + ( dcrQP < 0 ? -0.5 : 0.5) ));
    pps.chromaQpOffset[COMP_Y]          = 0;
    pps.chromaQpOffset[COMP_Cb]         = Clip3( -12, 12, cbQP + m_pcEncCfg->m_chromaCbQpOffset);
    pps.chromaQpOffset[COMP_Cr]         = Clip3( -12, 12, crQP + m_pcEncCfg->m_chromaCrQpOffset);
    pps.chromaQpOffset[COMP_JOINT_CbCr] = Clip3( -12, 12, ( cbQP + crQP ) / 2 + m_pcEncCfg->m_chromaCbCrQpOffset);
  }
  else
  {
    pps.chromaQpOffset[COMP_Y]          = 0;
    pps.chromaQpOffset[COMP_Cb]         = m_pcEncCfg->m_chromaCbQpOffset;
    pps.chromaQpOffset[COMP_Cr]         = m_pcEncCfg->m_chromaCrQpOffset;
    pps.chromaQpOffset[COMP_JOINT_CbCr] = m_pcEncCfg->m_chromaCbCrQpOffset;
  }

  bool bChromaDeltaQPEnabled = false;
  {
    bChromaDeltaQPEnabled = ( m_pcEncCfg->m_sliceChromaQpOffsetIntraOrPeriodic[ 0 ] || m_pcEncCfg->m_sliceChromaQpOffsetIntraOrPeriodic[ 1 ] );
    bChromaDeltaQPEnabled |= (m_pcEncCfg->m_usePerceptQPA || (m_pcEncCfg->m_LookAhead && m_pcRateCtrl->m_pcEncCfg->m_RCTargetBitrate) || m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity > 0) && (m_pcEncCfg->m_internChromaFormat != VVENC_CHROMA_400);
    if( ! bChromaDeltaQPEnabled && sps.dualITree && ( m_pcEncCfg->m_internChromaFormat != VVENC_CHROMA_400 ) )
    {
      bChromaDeltaQPEnabled = (m_pcEncCfg->m_chromaCbQpOffsetDualTree != 0 || m_pcEncCfg->m_chromaCrQpOffsetDualTree != 0 || m_pcEncCfg->m_chromaCbCrQpOffsetDualTree != 0);
    }
    if( ! bChromaDeltaQPEnabled )
    {
      bChromaDeltaQPEnabled = m_gopCfg->isChromaDeltaQPEnabled();
    }
  }
  pps.sliceChromaQpFlag                 = bChromaDeltaQPEnabled;
  pps.outputFlagPresent                 = false;
  pps.deblockingFilterOverrideEnabled   = !m_pcEncCfg->m_loopFilterOffsetInPPS;
  pps.deblockingFilterDisabled          = m_pcEncCfg->m_bLoopFilterDisable;
  pps.dbfInfoInPh                       = m_pcEncCfg->m_picPartitionFlag && !m_pcEncCfg->m_loopFilterOffsetInPPS && !m_pcEncCfg->m_bLoopFilterDisable;

  if (! pps.deblockingFilterDisabled)
  {
    for( int comp = 0; comp < MAX_NUM_COMP; comp++)
    {
      pps.deblockingFilterBetaOffsetDiv2[comp]  = m_pcEncCfg->m_loopFilterBetaOffsetDiv2[comp];
      pps.deblockingFilterTcOffsetDiv2[comp]    = m_pcEncCfg->m_loopFilterTcOffsetDiv2[comp];
    }
  }

  // deblockingFilterControlPresent is true if any of the settings differ from the inferred values:
  bool deblockingFilterControlPresent   = pps.deblockingFilterOverrideEnabled ||
                                          pps.deblockingFilterDisabled     ||
                                          pps.deblockingFilterBetaOffsetDiv2[COMP_Y] != 0 ||
                                          pps.deblockingFilterTcOffsetDiv2  [COMP_Y] != 0 ||
                                          pps.deblockingFilterBetaOffsetDiv2[COMP_Cb] != 0 ||
                                          pps.deblockingFilterTcOffsetDiv2  [COMP_Cb] != 0 ||
                                          pps.deblockingFilterBetaOffsetDiv2[COMP_Cr] != 0 ||
                                          pps.deblockingFilterTcOffsetDiv2  [COMP_Cr] != 0;

  pps.deblockingFilterControlPresent    = deblockingFilterControlPresent;
  pps.cabacInitPresent                  = m_pcEncCfg->m_cabacInitPresent != 0;
  pps.loopFilterAcrossTilesEnabled      = !m_pcEncCfg->m_bDisableLFCrossTileBoundaryFlag;
  pps.loopFilterAcrossSlicesEnabled     = !m_pcEncCfg->m_bDisableLFCrossSliceBoundaryFlag;
  pps.rpl1IdxPresent                    = sps.rpl1IdxPresent;

  const uint32_t chromaArrayType = (int)sps.separateColourPlane ? CHROMA_400 : sps.chromaFormatIdc;
  if( chromaArrayType != CHROMA_400  )
  {
    bool chromaQPOffsetNotZero = ( pps.chromaQpOffset[COMP_Cb] != 0 || pps.chromaQpOffset[COMP_Cr] != 0 || pps.jointCbCrQpOffsetPresent || pps.sliceChromaQpFlag || pps.chromaQpOffsetListLen );
    bool chromaDbfOffsetNotAsLuma = ( pps.deblockingFilterBetaOffsetDiv2[COMP_Cb] != pps.deblockingFilterBetaOffsetDiv2[COMP_Y]
                                   || pps.deblockingFilterBetaOffsetDiv2[COMP_Cr] != pps.deblockingFilterBetaOffsetDiv2[COMP_Y]
                                   || pps.deblockingFilterTcOffsetDiv2[COMP_Cb] != pps.deblockingFilterTcOffsetDiv2[COMP_Y]
                                   || pps.deblockingFilterTcOffsetDiv2[COMP_Cr] != pps.deblockingFilterTcOffsetDiv2[COMP_Y]);
    pps.usePPSChromaTool = chromaQPOffsetNotZero || chromaDbfOffsetNotAsLuma;
  }

  pps.numRefIdxL0DefaultActive = std::max( m_gopCfg->getDefaultNumActive( 0 ), 1 );
  pps.numRefIdxL1DefaultActive = std::max( m_gopCfg->getDefaultNumActive( 1 ), 1 );
  CHECK( pps.numRefIdxL0DefaultActive > 15, "num default ref index active exceeds maximum value");
  CHECK( pps.numRefIdxL1DefaultActive > 15, "num default ref index active exceeds maximum value");

  pps.noPicPartition = !m_pcEncCfg->m_picPartitionFlag;
  pps.ctuSize        = sps.CTUSize;
  pps.log2CtuSize    = Log2( sps.CTUSize );

  xInitPPSforTiles( pps, sps );

  pps.pcv            = new PreCalcValues( sps, pps, true );
}

void EncGOP::xInitPPSforTiles(PPS &pps,const SPS &sps) const
{
  pps.numExpTileCols = m_pcEncCfg->m_numExpTileCols;
  pps.numExpTileRows = m_pcEncCfg->m_numExpTileRows;
  pps.numSlicesInPic = m_pcEncCfg->m_numSlicesInPic;

  if( pps.noPicPartition )
  {
    pps.tileColWidth.resize( 1, pps.picWidthInCtu );
    pps.tileRowHeight.resize( 1, pps.picHeightInCtu );
    pps.initTiles();
    pps.sliceMap.clear();
    pps.sliceMap.resize(1);
    pps.sliceMap[0].addCtusToSlice(0, pps.picWidthInCtu, 0, pps.picHeightInCtu, pps.picWidthInCtu);
  }
  else
  {
    for( int i = 0; i < pps.numExpTileCols; i++ )
    {
      pps.tileColWidth.push_back( m_pcEncCfg->m_tileColumnWidth[i] );
    }
    for( int i = 0; i < pps.numExpTileRows; i++ )
    {
      pps.tileRowHeight.push_back( m_pcEncCfg->m_tileRowHeight[i] );
    }
    pps.initTiles();
    pps.rectSlice            = true;
    pps.tileIdxDeltaPresent  = false;
    pps.initRectSliceMap( &sps );
  }
}

void EncGOP::xInitRPL(SPS &sps) const
{
  m_gopCfg->getDefaultRPLLists( sps.rplList[ 0 ], sps.rplList[ 1 ] );

  sps.rpl1IdxPresent = ( sps.rplList[ 0 ].size() != sps.rplList[ 1 ].size() );

  //Check if all delta POC of STRP in each RPL has the same sign
  //Check RPLL0 first
  bool isAllEntriesinRPLHasSameSignFlag = true;
  for( int list = 0; list < 2; list++)
  {
    const RPLList& rplList = sps.rplList[list];
    uint32_t numRPL        = (uint32_t)rplList.size();

    bool isFirstEntry = true;
    bool lastSign = true;        //true = positive ; false = negative
    for (uint32_t ii = 0; isAllEntriesinRPLHasSameSignFlag && ii < numRPL; ii++)
    {
      const ReferencePictureList& rpl = rplList[ii];
      for (uint32_t jj = 0; jj < rpl.numberOfActivePictures; jj++)
      {
        if(rpl.isLongtermRefPic[jj])
          continue;

        if( isFirstEntry )
        {
          lastSign = (rpl.refPicIdentifier[jj] >= 0) ? true : false;
          isFirstEntry = false;
        }
        else
        {
          int ref = ( jj == 0 && !isFirstEntry ) ? 0 : rpl.refPicIdentifier[jj-1];
          if (((rpl.refPicIdentifier[jj] - ref) >= 0 ) != lastSign)
          {
            isAllEntriesinRPLHasSameSignFlag = false;
            break;  // break the inner loop
          }
        }
      }
    }
  }

  sps.allRplEntriesHasSameSign = isAllEntriesinRPLHasSameSignFlag;

  bool isRpl1CopiedFromRpl0 = ( sps.rplList[ 0 ].size() == sps.rplList[ 1 ].size() );
  for( int i = 0; isRpl1CopiedFromRpl0 && i < (int)sps.rplList[ 0 ].size(); i++)
  {
    isRpl1CopiedFromRpl0 = ( sps.rplList[0][i].getNumRefEntries() == sps.rplList[1][i].getNumRefEntries() );
    if( isRpl1CopiedFromRpl0 )
    {
      for( int j = 0; j < sps.rplList[0][i].getNumRefEntries(); j++ )
      {
        if( sps.rplList[0][i].refPicIdentifier[j] != sps.rplList[1][i].refPicIdentifier[j] )
        {
          isRpl1CopiedFromRpl0 = false;
          break;
        }
      }
    }
  }
  sps.rpl1CopyFromRpl0 = isRpl1CopiedFromRpl0;
}

void EncGOP::xInitHrdParameters(SPS &sps)
{
  m_EncHRD.initHRDParameters( *m_pcEncCfg, sps );

  sps.generalHrdParams = m_EncHRD.generalHrdParams;

  for(int i = 0; i < VVENC_MAX_TLAYER; i++)
  {
    sps.olsHrdParams[i] = m_EncHRD.olsHrdParams[i];
  }
}

/** Function for deciding the nal_unit_type.
 */
vvencNalUnitType EncGOP::xGetNalUnitType( const Slice* slice ) const
{
  const GOPEntry& gopEntry = *slice->pic->gopEntry;

  if( gopEntry.m_POC == 0 && m_pcEncCfg->m_DecodingRefreshType != VVENC_DRT_IDR2 )
  {
    return VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP;
  }

  if( gopEntry.m_isStartOfIntra )
  {
    if( m_pcEncCfg->m_DecodingRefreshType == VVENC_DRT_CRA || m_pcEncCfg->m_DecodingRefreshType == VVENC_DRT_CRA_CRE )
    {
      return VVENC_NAL_UNIT_CODED_SLICE_CRA;
    }
    if( m_pcEncCfg->m_DecodingRefreshType == VVENC_DRT_IDR || m_pcEncCfg->m_DecodingRefreshType == VVENC_DRT_IDR2  )
    {
      return VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL;
    }
  }

  if( m_pocCRA > 0 && gopEntry.m_POC < m_pocCRA )
  {
    // All leading pictures are being marked as TFD pictures here since current encoder uses all
    // reference pictures while encoding leading pictures. An encoder can ensure that a leading
    // picture can be still decodable when random accessing to a CRA/CRANT/BLA/BLANT picture by
    // controlling the reference pictures used for encoding that leading picture. Such a leading
    // picture need not be marked as a TFD picture.
    return VVENC_NAL_UNIT_CODED_SLICE_RASL;
  }

  if( m_lastIDR > 0 && gopEntry.m_POC < m_lastIDR )
  {
    return VVENC_NAL_UNIT_CODED_SLICE_RADL;
  }

  return VVENC_NAL_UNIT_CODED_SLICE_TRAIL;
}

bool EncGOP::xIsSliceTemporalSwitchingPoint( const Slice* slice, const PicList& picList ) const
{
  if( slice->TLayer <= 0
      || slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RADL
      || slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RASL
      || ! slice->isStepwiseTemporalLayerSwitchingPointCandidate( picList ) )
  {
    return false;
  }

  const GOPEntry& gopEntry = *(slice->pic->gopEntry);
  const bool isSTSA        = gopEntry.m_isSTSA;
  return isSTSA;
}

void EncGOP::xInitPicsInCodingOrder( const PicList& picList, bool flush )
{
  CHECK( m_pcEncCfg->m_maxParallelFrames <= 0 && m_gopEncListInput.size() > 0,  "no frame parallel processing enabled, but multiple pics in flight" );
  CHECK( m_pcEncCfg->m_maxParallelFrames <= 0 && m_gopEncListOutput.size() > 0, "no frame parallel processing enabled, but multiple pics in flight" );

  // loop over pic list, which is sorted in coding number order 
  for( auto pic : picList )
  {
    // skip pics, which have already been initialised
    if( pic->isInitDone )
      continue;
    // continue with next pic in increasing coding number order
    if( ! flush && pic->gopEntry->m_codingNum != m_lastCodingNum + 1 )
      break;

    CHECK( m_lastCodingNum == -1 && ! pic->gopEntry->m_isStartOfIntra, "encoding should start with an I-Slice" );

    // initialize slice header
    pic->encTime.startTimer();
    xInitFirstSlice( *pic, picList, false );
    pic->encTime.stopTimer();

    // pictures ready for encoding
    m_gopEncListInput.push_back( pic );
    m_gopEncListOutput.push_back( pic );

    // continue with next picture
    m_lastCodingNum = pic->gopEntry->m_codingNum;

    // in single threading initialize only one picture per encoding loop
    if( m_pcEncCfg->m_maxParallelFrames <= 0 )
      break;
  }

  CHECK( picList.size() && m_pcEncCfg->m_maxParallelFrames <= 0 && m_gopEncListInput.size() != 1,  "no new picture for encoding found" );
  CHECK( picList.size() && m_pcEncCfg->m_maxParallelFrames <= 0 && m_gopEncListOutput.size() != 1, "no new picture for encoding found" );
}

void EncGOP::xGetProcessingLists( std::list<Picture*>& procList, std::list<Picture*>& rcUpdateList, const bool lockStepMode )
{
  // decoder in encoder and FPP: restrict processing list to the default sequential coding order
  if( m_trySkipOrDecodePicture && m_pcEncCfg->m_maxParallelFrames > 0 )
  {
    auto& pic = m_gopEncListInput.front();
    if( pic->poc == m_pcEncCfg->m_switchPOC )
    {
      m_trySkipOrDecodePicture = false;
    }
    else
    {
      // try to skip or decode picture
      procList.push_back( pic );
      rcUpdateList.push_back( pic );
      m_gopEncListInput.pop_front();
      return;
    }
  }

  // in lockstep mode, process only pics of same temporal layer
  if( lockStepMode )
  {
    // start new parallel chunk only, if next output picture is not reconstructed
    if( rcUpdateList.empty() )
    {
      const int procTL         = m_gopEncListInput.size() ? m_gopEncListInput.front()->TLayer             : -1;
      const int gopNum         = m_gopEncListInput.size() ? m_gopEncListInput.front()->gopEntry->m_gopNum : -1;
      const int minSerialDepth = m_pcEncCfg->m_maxParallelFrames > 2 ? 1 : 2;  // up to this temporal layer encode pictures only in serial mode
      const int maxSize        = procTL <= minSerialDepth ? 1 : m_pcEncCfg->m_maxParallelFrames;
      for( auto it = m_gopEncListInput.begin(); it != m_gopEncListInput.end(); )
      {
        auto pic = *it;
        if( pic->gopEntry->m_gopNum == gopNum
            && pic->TLayer == procTL
            && pic->slices[ 0 ]->checkRefPicsReconstructed() )
        {
          procList.push_back    ( pic );
          rcUpdateList.push_back( pic );
          it = m_gopEncListInput.erase( it );
        }
        else
        {
          ++it;
        }
        if( (int)procList.size() >= maxSize )
          break;
      }
    }
  }
  else
  {
    procList.splice( procList.end(), m_gopEncListInput );
    m_gopEncListInput.clear();
    if( ! m_gopEncListOutput.empty() )
      rcUpdateList.push_back( m_gopEncListOutput.front() );
  }
  CHECK( ! rcUpdateList.empty() && m_gopEncListOutput.empty(),                                                         "first picture in RC update and in output list have to be the same" );
}

void EncGOP::xInitFirstSlice( Picture& pic, const PicList& picList, bool isEncodeLtRef )
{
  memset( pic.cs->alfAps, 0, sizeof(pic.cs->alfAps));

  const int curPoc          = pic.getPOC();
  Slice* slice              = pic.allocateNewSlice();
  pic.cs->picHeader         = new PicHeader;
  const SPS& sps            = *(slice->sps);
  vvencNalUnitType naluType = xGetNalUnitType( slice );
  const GOPEntry& gopEntry  = *pic.gopEntry;
  SliceType sliceType       = gopEntry.m_sliceType == 'B' ? VVENC_B_SLICE : ( gopEntry.m_sliceType == 'P' ? VVENC_P_SLICE : VVENC_I_SLICE );

  // correct slice type at start of intra period
  if( gopEntry.m_isStartOfIntra )
  {
    sliceType = VVENC_I_SLICE;
  }

  // update IRAP
  if( naluType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || naluType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP
      || naluType == VVENC_NAL_UNIT_CODED_SLICE_CRA )
  {
    m_associatedIRAPType = naluType;
    m_associatedIRAPPOC  = curPoc;
  }

  // update last IDR
  if( naluType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL || naluType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP )
  {
    m_lastIDR = curPoc;
  }

  slice->picHeader                 = pic.cs->picHeader;
  slice->independentSliceIdx       = 0;
  slice->sliceType                 = sliceType;
  slice->poc                       = curPoc;
  slice->TLayer                    = gopEntry.m_temporalId;
  slice->nalUnitType               = naluType;
  slice->lastIDR                   = m_lastIDR;
  slice->depQuantEnabled           = m_pcEncCfg->m_DepQuantEnabled;
  slice->signDataHidingEnabled     = m_pcEncCfg->m_SignDataHidingEnabled;

  slice->picHeader->splitConsOverride = false;
  for( int i = 0; i < 3; i++ )
  {
    slice->picHeader->minQTSize[i]   = sps.minQTSize[i];
    slice->picHeader->maxMTTDepth[i] = sps.maxMTTDepth[i];
    slice->picHeader->maxBTSize[i]   = sps.maxBTSize[i];
    slice->picHeader->maxTTSize[i]   = sps.maxTTSize[i];
    if( ( i == 1 ) && ( m_pcEncCfg->m_maxMTTDepth >= 10 ) )
    {
      slice->picHeader->maxMTTDepth[i]    = int( m_pcEncCfg->m_maxMTTDepth / pow( 10, sps.maxTLayers - slice->TLayer - 1 ) ) % 10;
      slice->picHeader->splitConsOverride = true;
    }
  }

  slice->associatedIRAPType        = m_associatedIRAPType;
  slice->associatedIRAP            = m_associatedIRAPPOC;
  CHECK( MAX_REF_PICS <= gopEntry.m_numRefPicsActive[ 0 ], "number of ref pics out of supported range" );
  CHECK( MAX_REF_PICS <= gopEntry.m_numRefPicsActive[ 1 ], "number of ref pics out of supported range" );
  slice->numRefIdx[REF_PIC_LIST_0] = gopEntry.m_numRefPicsActive[ 0 ];
  slice->numRefIdx[REF_PIC_LIST_1] = gopEntry.m_numRefPicsActive[ 1 ];
  slice->setDecodingRefreshMarking ( m_pocCRA, m_bRefreshPending, picList );
  slice->setDefaultClpRng          ( sps );

  // reference list
  xSelectReferencePictureList( slice );
  int missingPoc;
  if ( slice->isRplPicMissing( picList, REF_PIC_LIST_0, missingPoc ) || slice->isRplPicMissing( picList, REF_PIC_LIST_1, missingPoc ) )
  {
    slice->createExplicitReferencePictureSetFromReference( picList, slice->rpl[0], slice->rpl[1] );
  }
  slice->applyReferencePictureListBasedMarking( picList, slice->rpl[0], slice->rpl[1], 0, *slice->pps );

  // nalu type refinement
  if ( xIsSliceTemporalSwitchingPoint( slice, picList ) )
  {
    naluType = VVENC_NAL_UNIT_CODED_SLICE_STSA;
    slice->nalUnitType = naluType;
  }

  // reference list
  slice->numRefIdx[REF_PIC_LIST_0] = sliceType == VVENC_I_SLICE ? 0 : slice->rpl[0]->numberOfActivePictures;
  slice->numRefIdx[REF_PIC_LIST_1] = sliceType != VVENC_B_SLICE ? 0 : slice->rpl[1]->numberOfActivePictures;
  slice->constructRefPicList  ( picList, false );

  if ( BitAllocation::isTempLayer0IntraFrame( slice, m_pcEncCfg, picList, m_pcRateCtrl->rcIsFinalPass ) ) // T-Layer-0 B frame adaptation and some preparations for rate control
  {
    slice->sliceType = sliceType = VVENC_I_SLICE;
    slice->numRefIdx[REF_PIC_LIST_0] = 0;
    slice->numRefIdx[REF_PIC_LIST_1] = 0;
    slice->constructRefPicList( picList, false );
  }
  slice->setRefPOCList        ();
  slice->setList1IdxToList0Idx();
  slice->updateRefPicCounter  ( +1 );
  slice->setSMVDParam();

  // slice type refinement
  if ( sliceType == VVENC_B_SLICE && slice->numRefIdx[ REF_PIC_LIST_1 ] == 0 )
  {
    sliceType = VVENC_P_SLICE;
    slice->sliceType = sliceType;
  }

  slice->picHeader->gdrPic      = false;
  slice->picHeader->disBdofFlag = false;
  slice->picHeader->disDmvrFlag = false;
  slice->picHeader->disProfFlag = false;

  slice->picHeader->gdrOrIrapPic = slice->picHeader->gdrPic || slice->isIRAP();
  slice->picHeader->picInterSliceAllowed = sliceType != VVENC_I_SLICE;
  slice->picHeader->picIntraSliceAllowed = sliceType == VVENC_I_SLICE;

  slice->deblockingFilterOverride = sliceType != VVENC_I_SLICE && (gopEntry.m_betaOffsetDiv2 || gopEntry.m_tcOffsetDiv2);

  if( m_pcEncCfg->m_deblockLastTLayers > 0 && slice->TLayer <= m_pcEncCfg->m_maxTLayer - m_pcEncCfg->m_deblockLastTLayers )
  {
    slice->deblockingFilterOverride = true;
    slice->deblockingFilterDisable  = true;
  }

  if( slice->deblockingFilterOverride )
  {
    for( int comp = 0; comp < MAX_NUM_COMP; comp++)
    {
      slice->deblockingFilterTcOffsetDiv2[comp]    = slice->picHeader->deblockingFilterTcOffsetDiv2[comp]   = gopEntry.m_tcOffsetDiv2   + m_pcEncCfg->m_loopFilterTcOffsetDiv2[comp];
      slice->deblockingFilterBetaOffsetDiv2[comp]  = slice->picHeader->deblockingFilterBetaOffsetDiv2[comp] = gopEntry.m_betaOffsetDiv2 +   m_pcEncCfg->m_loopFilterBetaOffsetDiv2[comp];
    }
  }
  else
  {
    for( int comp = 0; comp < MAX_NUM_COMP; comp++)
    {
      slice->deblockingFilterTcOffsetDiv2[comp]    = slice->picHeader->deblockingFilterTcOffsetDiv2[comp]   = slice->pps->deblockingFilterTcOffsetDiv2[comp];
      slice->deblockingFilterBetaOffsetDiv2[comp]  = slice->picHeader->deblockingFilterBetaOffsetDiv2[comp] = slice->pps->deblockingFilterBetaOffsetDiv2[comp];
    }
  }

  if (slice->pps->useDQP)
  {
    const uint32_t cuLumaQpSubdiv = (m_pcEncCfg->m_cuQpDeltaSubdiv > 0 ? (uint32_t) m_pcEncCfg->m_cuQpDeltaSubdiv : 0);

    slice->picHeader->cuQpDeltaSubdivInter = m_pcEncCfg->m_usePerceptQPA ? 0 : cuLumaQpSubdiv;
    slice->picHeader->cuQpDeltaSubdivIntra = cuLumaQpSubdiv;
  }
  if( slice->pps->chromaQpOffsetListLen > 0)
  {
    const uint32_t cuChromaQpSubdiv = (m_pcEncCfg->m_cuChromaQpOffsetSubdiv > 0 ? (uint32_t) m_pcEncCfg->m_cuChromaQpOffsetSubdiv : 0);

    slice->picHeader->cuChromaQpOffsetSubdivInter = m_pcEncCfg->m_usePerceptQPA ? 0 : cuChromaQpSubdiv;
    slice->picHeader->cuChromaQpOffsetSubdivIntra = cuChromaQpSubdiv;
  }

  slice->picHeader->ppsId = slice->pps->ppsId;
  slice->picHeader->spsId = slice->sps->spsId;

  pic.cs->picHeader->pic = &pic;
  xInitSliceTMVPFlag ( pic.cs->picHeader, slice );
  xInitSliceMvdL1Zero( pic.cs->picHeader, slice );
  slice->picHeader->maxNumAffineMergeCand = sps.Affine ? sps.maxNumAffineMergeCand : ( sps.SbtMvp && slice->picHeader->enableTMVP ? 1 : 0 );

  if( slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RASL && m_pcEncCfg->m_rprRASLtoolSwitch )
  {
    slice->lmChromaCheckDisable = true;
    pic.cs->picHeader->disDmvrFlag = true;
    xUpdateRPRtmvp( pic.cs->picHeader, slice );
  }

  // update RAS
  xUpdateRasInit( slice );

  if( m_pcEncCfg->m_useAMaxBT )
  {
    m_BlkStat.setSliceMaxBT( *slice );
  }

  {
    bool identicalToSPS=true;
    const SPS* sps =slice->sps;
    PicHeader* picHeader = slice->picHeader;
    if (picHeader->picInterSliceAllowed)
    {
      identicalToSPS = (picHeader->minQTSize[1] == sps->minQTSize[1] &&
                        picHeader->maxMTTDepth[1] == sps->maxMTTDepth[1] &&
                        picHeader->maxBTSize[1] == sps->maxBTSize[1] &&
                        picHeader->maxTTSize[1] == sps->maxTTSize[1] );
    }

    if (identicalToSPS && picHeader->picIntraSliceAllowed)
    {
      identicalToSPS = (picHeader->minQTSize[0] == sps->minQTSize[0] &&
                        picHeader->maxMTTDepth[0] == sps->maxMTTDepth[0] &&
                        picHeader->maxBTSize[0] == sps->maxBTSize[0] &&
                        picHeader->maxTTSize[0] == sps->maxTTSize[0] );
    }

    if (identicalToSPS && sps->dualITree)
    {
      identicalToSPS = (picHeader->minQTSize[2] == sps->minQTSize[2] &&
                        picHeader->maxMTTDepth[2] == sps->maxMTTDepth[2] &&
                        picHeader->maxBTSize[2] == sps->maxBTSize[2] &&
                        picHeader->maxTTSize[2] == sps->maxTTSize[2] );
    }

    if (identicalToSPS)
    {
      picHeader->splitConsOverride = false;
    }

  }

  CHECK( slice->TLayer != 0 && slice->sliceType == VVENC_I_SLICE, "Unspecified error" );

  pic.cs->slice = slice;
  pic.cs->allocateVectorsAtPicLevel();
  pic.isReferenced = true;

  // reshaper
  xInitLMCS( pic );

  pic.picApsMap.clearActive();
  for ( int i = 0; i < ALF_CTB_MAX_NUM_APS; i++ )
  {
    const int apsMapIdx = ( i << NUM_APS_TYPE_LEN ) + ALF_APS;
    APS* alfAPS = pic.picApsMap.getPS( apsMapIdx );
    if ( alfAPS )
    {
      pic.picApsMap.clearChangedFlag( apsMapIdx );
      alfAPS->alfParam.reset();
      alfAPS->ccAlfParam.reset();
    }
  }
  CHECK( slice->enableDRAPSEI && m_pcEncCfg->m_maxParallelFrames, "Dependent Random Access Point is not supported by Frame Parallel Processing" );

  if( pic.poc == m_pcEncCfg->m_switchPOC )
  {
    m_appliedSwitchDQP = m_pcEncCfg->m_switchDQP;
  }
  pic.seqBaseQp = m_pcEncCfg->m_QP + m_appliedSwitchDQP;

  pic.isInitDone = true;
}

void EncGOP::xInitSliceTMVPFlag( PicHeader* picHeader, const Slice* slice )
{
  if( m_pcEncCfg->m_TMVPModeId == 2 )
  {
    const GOPEntry& gopEntry = *(slice->pic->gopEntry);
    picHeader->enableTMVP    = ! gopEntry.m_useBckwdOnly;
  }
  else if( m_pcEncCfg->m_TMVPModeId == 1 )
  {
    picHeader->enableTMVP = true;
  }
  else
  {
    picHeader->enableTMVP = false;
  }

  // disable TMVP when current picture is the only ref picture
  if( slice->isIRAP() && slice->sps->IBC )
  {
    picHeader->enableTMVP = false;
  }
}

void EncGOP::xUpdateRPRtmvp( PicHeader* picHeader, Slice* slice )
{
  if( slice->sliceType != VVENC_I_SLICE && picHeader->enableTMVP && m_pcEncCfg->m_rprRASLtoolSwitch )
  {
    int colRefIdxL0 = -1, colRefIdxL1 = -1;

    for( int refIdx = 0; refIdx < slice->numRefIdx[REF_PIC_LIST_0]; refIdx++ )
    {
      if( !( slice->getRefPic( REF_PIC_LIST_0, refIdx )->slices[0]->nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_RASL &&
             slice->getRefPic( REF_PIC_LIST_0, refIdx )->poc <= m_pocCRA ) )
      {
        colRefIdxL0 = refIdx;
        break;
      }
    }

    if( slice->sliceType == VVENC_B_SLICE )
    {
      for( int refIdx = 0; refIdx < slice->numRefIdx[REF_PIC_LIST_1]; refIdx++ )
      {
        if( !( slice->getRefPic( REF_PIC_LIST_1, refIdx )->slices[0]->nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_RASL &&
               slice->getRefPic( REF_PIC_LIST_1, refIdx )->poc <= m_pocCRA ) )
        {
          colRefIdxL1 = refIdx;
          break;
        }
      }
    }

    if( colRefIdxL0 >= 0 && colRefIdxL1 >= 0 )
    {
      const Picture *refPicL0 = slice->getRefPic( REF_PIC_LIST_0, colRefIdxL0 );
      const Picture *refPicL1 = slice->getRefPic( REF_PIC_LIST_1, colRefIdxL1 );

      CHECK( !refPicL0->slices.size(), "Wrong L0 reference picture" );
      CHECK( !refPicL1->slices.size(), "Wrong L1 reference picture" );

      const uint32_t uiColFromL0 = refPicL0->slices[0]->sliceQp > refPicL1->slices[0]->sliceQp;
      picHeader->picColFromL0 = uiColFromL0;
      slice->colFromL0Flag = uiColFromL0;
      slice->colRefIdx = uiColFromL0 ? colRefIdxL0 : colRefIdxL1;
      picHeader->colRefIdx = uiColFromL0 ? colRefIdxL0 : colRefIdxL1;
    }
    else if( colRefIdxL0 < 0 && colRefIdxL1 >= 0 )
    {
      picHeader->picColFromL0 = false;
      slice->colFromL0Flag = false;
      slice->colRefIdx = colRefIdxL1;
      picHeader->colRefIdx = colRefIdxL1;
    }
    else if( colRefIdxL0 >= 0 && colRefIdxL1 < 0 )
    {
      picHeader->picColFromL0 = true;
      slice->colFromL0Flag = true;
      slice->colRefIdx = colRefIdxL0;
      picHeader->colRefIdx = colRefIdxL0;
    }
    else
    {
      picHeader->enableTMVP = false;
    }
  }
}

void EncGOP::xInitSliceMvdL1Zero( PicHeader* picHeader, const Slice* slice )
{
  bool bGPBcheck = false;
  if ( slice->sliceType == VVENC_B_SLICE)
  {
    if ( slice->numRefIdx[ 0 ] == slice->numRefIdx[ 1 ] )
    {
      bGPBcheck = true;
      int i;
      for ( i=0; i < slice->numRefIdx[ 1 ]; i++ )
      {
        if ( slice->getRefPOC( RefPicList( 1 ), i ) != slice->getRefPOC( RefPicList( 0 ), i ) )
        {
          bGPBcheck = false;
          break;
        }
      }
    }
  }

  if ( bGPBcheck )
  {
    picHeader->mvdL1Zero = true;
  }
  else
  {
    picHeader->mvdL1Zero = false;
  }
}

void EncGOP::xInitLMCS( Picture& pic )
{
  Slice* slice = pic.cs->slice;
  const SliceType sliceType = slice->sliceType;

  if( ! pic.useScLMCS || (!slice->isIntra() && m_disableLMCSIP) )
  {
    pic.reshapeData.copyReshapeData( m_Reshaper );
    m_Reshaper.setCTUFlag     ( false );
    pic.reshapeData.setCTUFlag( false );
    if( slice->isIntra() )  m_disableLMCSIP = true;
    return;
  }
  if( slice->isIntra() ) m_disableLMCSIP = false;

  m_Reshaper.getReshapeCW()->rspTid = slice->TLayer + (slice->isIntra() ? 0 : 1);
  m_Reshaper.getSliceReshaperInfo().chrResScalingOffset = m_pcEncCfg->m_LMCSOffset;

  if ( m_pcEncCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ )
  {
    m_Reshaper.preAnalyzerHDR( pic, sliceType, m_pcEncCfg->m_reshapeCW );
  }
  else if ( m_pcEncCfg->m_reshapeSignalType == RESHAPE_SIGNAL_SDR || m_pcEncCfg->m_reshapeSignalType == RESHAPE_SIGNAL_HLG )
  {
    m_Reshaper.preAnalyzerLMCS( pic, m_pcEncCfg->m_reshapeSignalType, sliceType, m_pcEncCfg->m_reshapeCW );
  }
  else
  {
    THROW("Reshaper for other signal currently not defined!");
  }

  if ( sliceType == VVENC_I_SLICE )
  {
    if ( m_pcEncCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ )
    {
      m_Reshaper.initLUTfromdQPModel();
      m_Reshaper.updateReshapeLumaLevelToWeightTableChromaMD( m_Reshaper.getInvLUT() );
    }
    else if ( m_pcEncCfg->m_reshapeSignalType == RESHAPE_SIGNAL_SDR || m_pcEncCfg->m_reshapeSignalType == RESHAPE_SIGNAL_HLG )
    {
      if ( m_Reshaper.getReshapeFlag() )
      {
        m_Reshaper.constructReshaperLMCS();
        m_Reshaper.updateReshapeLumaLevelToWeightTable( m_Reshaper.getSliceReshaperInfo(), m_Reshaper.getWeightTable(), m_Reshaper.getCWeight() );
      }
    }
    else
    {
      THROW( "Reshaper for other signal currently not defined!" );
    }

        //reshape original signal
    if( m_Reshaper.getSliceReshaperInfo().sliceReshaperEnabled )
    {
      CPelUnitBuf origBuf = pic.getOrigBuf();
      if( pic.getFilteredOrigBuffer().valid() )
      {
        pic.getRspOrigBuf().get(COMP_Y).rspSignal( m_Reshaper.getFwdLUT() );
      }
      else
      {
        pic.getFilteredOrigBuffer().create( pic.cs->pcv->chrFormat, Area( 0, 0, origBuf.get( COMP_Y ).width, origBuf.get( COMP_Y ).height) );
        PelUnitBuf rspOrigBuf = pic.getRspOrigBuf();
        rspOrigBuf.get(COMP_Y).rspSignal( origBuf.get(COMP_Y), m_Reshaper.getFwdLUT() );
        if( CHROMA_400 != pic.cs->pcv->chrFormat )
        {
          rspOrigBuf.get(COMP_Cb).copyFrom( origBuf.get(COMP_Cb) );
          rspOrigBuf.get(COMP_Cr).copyFrom( origBuf.get(COMP_Cr) );
        }
      }
    }

    m_Reshaper.setCTUFlag( false );
  }
  else
  {
    m_Reshaper.setCTUFlag( m_Reshaper.getReshapeFlag() );
    m_Reshaper.getSliceReshaperInfo().sliceReshaperModelPresent = false;

    if ( m_pcEncCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ )
    {
      m_Reshaper.restoreReshapeLumaLevelToWeightTable();
    }
    else if ( m_pcEncCfg->m_reshapeSignalType == RESHAPE_SIGNAL_SDR || m_pcEncCfg->m_reshapeSignalType == RESHAPE_SIGNAL_HLG )
    {
      int modIP = pic.getPOC() - pic.getPOC() / m_pcEncCfg->m_reshapeCW.rspFpsToIp * m_pcEncCfg->m_reshapeCW.rspFpsToIp;
      if (m_Reshaper.getReshapeFlag() && m_pcEncCfg->m_reshapeCW.updateCtrl == 2 && modIP == 0)
      {
        m_Reshaper.getSliceReshaperInfo().sliceReshaperModelPresent = true;
        m_Reshaper.constructReshaperLMCS();
        m_Reshaper.updateReshapeLumaLevelToWeightTable(m_Reshaper.getSliceReshaperInfo(), m_Reshaper.getWeightTable(), m_Reshaper.getCWeight());
      }
    }
    else
    {
      THROW("Reshaper for other signal currently not defined!");
    }
  }

  //set all necessary information in LMCS APS and slice
  slice->lmcsEnabled = slice->picHeader->lmcsEnabled = m_Reshaper.getSliceReshaperInfo().sliceReshaperEnabled;
  slice->picHeader->lmcsChromaResidualScale          = ( m_Reshaper.getSliceReshaperInfo().enableChromaAdj == 1 );
  if ( m_Reshaper.getSliceReshaperInfo().sliceReshaperModelPresent )
  {
    ParameterSetMap<APS>& picApsMap = pic.picApsMap;
    const int apsId0                = 0;
    const int apsMapIdx             = ( apsId0 << NUM_APS_TYPE_LEN ) + LMCS_APS;
    APS* picAps                     = picApsMap.getPS( apsMapIdx );
    if ( picAps == nullptr )
    {
      picAps = picApsMap.allocatePS( apsMapIdx );
      picAps->apsType     = LMCS_APS;
      picAps->apsId       = apsId0;
    }
    picAps->lmcsParam = m_Reshaper.getSliceReshaperInfo();
    picApsMap.setChangedFlag( apsMapIdx );
    slice->picHeader->lmcsAps    = picAps;
    slice->picHeader->lmcsApsId  = apsId0;
  }

  if ( slice->picHeader->lmcsEnabled )
  {
    slice->picHeader->lmcsApsId = 0;
  }

  pic.reshapeData.copyReshapeData( m_Reshaper );
}

void EncGOP::xSelectReferencePictureList( Slice* slice ) const
{
  const GOPEntry& gopEntry = *(slice->pic->gopEntry);

  for( int l = 0; l < 2; l++ )
  {
    slice->rplIdx[ l ] = gopEntry.m_defaultRPLIdx;
    if( slice->rplIdx[ l ] >= 0 )
    {
      slice->rpl[ l ] = &(slice->sps->rplList[ l ][ slice->rplIdx[ l ] ]);
    }
    else
    {
      slice->rplLocal[ l ].initFromGopEntry( gopEntry, l );
      slice->rpl[ l ] = &slice->rplLocal[ l ];
    }
  }
}

void EncGOP::xSyncAlfAps( Picture& pic, ParameterSetMap<APS>& dst, const ParameterSetMap<APS>& src )
{
  Slice& slice   = *pic.cs->slice;
  const SPS& sps = *slice.sps;

  if ( sps.alfEnabled )
  {
    // cleanup first
    dst.clearActive();
    for ( int i = 0; i < ALF_CTB_MAX_NUM_APS; i++ )
    {
      const int apsMapIdx = ( i << NUM_APS_TYPE_LEN ) + ALF_APS;
      APS* alfAPS = dst.getPS( apsMapIdx );
      if ( alfAPS )
      {
        dst.clearChangedFlag( apsMapIdx );
        alfAPS->alfParam.reset();
      }
    }
    // copy
    for ( int i = 0; i < ALF_CTB_MAX_NUM_APS; i++ )
    {
      const int apsMapIdx = ( i << NUM_APS_TYPE_LEN ) + ALF_APS;
      const APS* srcAPS = src.getPS( apsMapIdx );
      if ( srcAPS )
      {
        APS* dstAPS = dst.getPS( apsMapIdx );
        if ( ! dstAPS )
        {
          dstAPS = dst.allocatePS( apsMapIdx );
          dst.clearChangedFlag( apsMapIdx );
        }
        dst.setChangedFlag( apsMapIdx, src.getChangedFlag( apsMapIdx ) );
        dstAPS->alfParam    = srcAPS->alfParam;
        dstAPS->ccAlfParam  = srcAPS->ccAlfParam;
        dstAPS->apsId       = srcAPS->apsId;
        dstAPS->apsType     = srcAPS->apsType;
        dstAPS->layerId     = srcAPS->layerId;
        dstAPS->temporalId  = srcAPS->temporalId;
        dstAPS->poc         = srcAPS->poc;
      }
    }
    dst.setApsIdStart( src.getApsIdStart() );
  }
}

void EncGOP::xWritePicture( Picture& pic, AccessUnitList& au, bool isEncodeLtRef )
{
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "bsfinal", 1 ) );
  pic.encTime.startTimer();

  au.poc           = pic.poc;
  au.temporalLayer = pic.TLayer;
  au.refPic        = pic.isReferenced;
  if ( ! pic.slices.empty() )
  {
    au.sliceType = pic.slices[ 0 ]->sliceType;
  }

  if( pic.ctsValid )
  {
    const int64_t iDiffFrames = m_numPicsCoded - pic.poc;
    au.cts      = pic.cts;
    au.ctsValid = pic.ctsValid;
    au.dts      = ( ( iDiffFrames - m_pcEncCfg->m_maxTLayer ) * m_ticksPerFrameMul4 ) / 4 + au.cts;
    au.dtsValid = pic.ctsValid;
  }

  pic.actualTotalBits += xWriteParameterSets( pic, au, m_HLSWriter );
  xWriteLeadingSEIs( pic, au );
  pic.actualTotalBits += xWritePictureSlices( pic, au, m_HLSWriter );

  pic.encTime.stopTimer();

  std::string digestStr;
  xWriteTrailingSEIs( pic, au, digestStr );
  xPrintPictureInfo ( pic, au, digestStr, m_pcEncCfg->m_printFrameMSE, isEncodeLtRef );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "bsfinal", 0 ) );
}

int EncGOP::xWriteParameterSets( Picture& pic, AccessUnitList& accessUnit, HLSWriter& hlsWriter )
{
  Slice* slice        = pic.slices[0];
  const SPS& sps      = *(slice->sps);
  const PPS& pps      = *(slice->pps);
  int actualTotalBits = 0;

  if ( m_bFirstWrite || ( m_pcEncCfg->m_rewriteParamSets && slice->isIRAP() ) )
  {
    if (slice->sps->vpsId != 0)
    {
      actualTotalBits += xWriteVPS( accessUnit, pic.vps, hlsWriter );
    }
    actualTotalBits += xWriteDCI( accessUnit, pic.dci, hlsWriter );
    actualTotalBits += xWriteSPS( accessUnit, &sps, hlsWriter );
    actualTotalBits += xWritePPS( accessUnit, &pps, &sps, hlsWriter );
    m_bFirstWrite = false;
  }

  bool IrapOrGdrAu = slice->picHeader->gdrPic || (slice->isIRAP() && !slice->pps->mixedNaluTypesInPic);
  if ((( slice->vps->maxLayers > 1 && IrapOrGdrAu) || m_pcEncCfg->m_AccessUnitDelimiter) && !slice->nuhLayerId )
  {
    xWriteAccessUnitDelimiter( accessUnit, slice, IrapOrGdrAu, hlsWriter );
  }

  // send LMCS APS when LMCSModel is updated. It can be updated even current slice does not enable reshaper.
  // For example, in RA, update is on intra slice, but intra slice may not use reshaper
  if ( sps.lumaReshapeEnable && slice->picHeader->lmcsApsId >= 0 )
  {
    // only 1 LMCS data for 1 picture
    ParameterSetMap<APS>& apsMap = pic.picApsMap;
    const int apsId              = slice->picHeader->lmcsApsId;
    const int apsMapIdx          = apsId >= 0 ?  ( apsId << NUM_APS_TYPE_LEN ) + LMCS_APS : 0;
    APS* aps                     = apsId >= 0 ?  apsMap.getPS( apsMapIdx ) : nullptr;
    const bool doAPS             = aps && apsMap.getChangedFlag( apsMapIdx );
    if ( doAPS )
    {
      aps->chromaPresent = slice->sps->chromaFormatIdc != CHROMA_400;
      aps->temporalId = slice->TLayer;
      actualTotalBits += xWriteAPS( accessUnit, aps, hlsWriter, VVENC_NAL_UNIT_PREFIX_APS );
      apsMap.clearChangedFlag( apsMapIdx );
      CHECK( aps != slice->picHeader->lmcsAps, "Wrong LMCS APS pointer" );
    }
  }

  // send ALF APS
  if ( sps.alfEnabled && (slice->alfEnabled[COMP_Y] || slice->ccAlfCbEnabled || slice->ccAlfCrEnabled ))
  {
    for ( int apsId = 0; apsId < ALF_CTB_MAX_NUM_APS; apsId++ )
    {
      ParameterSetMap<APS>& apsMap = pic.picApsMap;
      const int apsMapIdx          = ( apsId << NUM_APS_TYPE_LEN ) + ALF_APS;
      APS* aps                     = apsMap.getPS( apsMapIdx );
      bool writeAps                = aps && apsMap.getChangedFlag( apsMapIdx );
      if ( !aps && slice->alfAps[ apsId ] )
      {
        aps   = apsMap.allocatePS( apsMapIdx );
        *aps  = *slice->alfAps[ apsId ]; // copy aps from slice header
        writeAps = true;
      }
      else if (slice->ccAlfCbEnabled && !aps && apsId == slice->ccAlfCbApsId)
      {
        writeAps = true;
        aps = apsMap.getPS((slice->ccAlfCbApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      }
      else if (slice->ccAlfCrEnabled && !aps && apsId == slice->ccAlfCrApsId)
      {
        writeAps = true;
        aps = apsMap.getPS((slice->ccAlfCrApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      }

      if ( writeAps )
      {
        aps->chromaPresent = slice->sps->chromaFormatIdc != CHROMA_400;
        aps->temporalId = slice->TLayer;
        actualTotalBits += xWriteAPS( accessUnit, aps, hlsWriter, VVENC_NAL_UNIT_PREFIX_APS );
        apsMap.clearChangedFlag( apsMapIdx );
      }
    }
  }

  return actualTotalBits;
}

int EncGOP::xWritePictureSlices( Picture& pic, AccessUnitList& accessUnit, HLSWriter& hlsWriter )
{
  Slice* slice        = pic.slices[ 0 ];
  const int numSlices = (int)( pic.slices.size() );
  unsigned  numBytes  = 0;

  for ( int sliceIdx = 0; sliceIdx < numSlices; sliceIdx++ )
  {
    slice = pic.slices[ sliceIdx ];

    if ( sliceIdx > 0 && slice->sliceType != VVENC_I_SLICE )
    {
      slice->checkColRefIdx( sliceIdx, &pic );
    }

    // start slice NALUnit
    OutputNALUnit nalu( slice->nalUnitType, slice->TLayer );
    hlsWriter.setBitstream( &nalu.m_Bitstream );

    // slice header and data
    int bitsBeforeWriting = hlsWriter.getNumberOfWrittenBits();
    hlsWriter.codeSliceHeader( slice );
    pic.actualHeadBits += ( hlsWriter.getNumberOfWrittenBits() - bitsBeforeWriting );
    hlsWriter.codeTilesWPPEntryPoint( slice );
    xAttachSliceDataToNalUnit( nalu, &pic.sliceDataStreams[ sliceIdx ] );

    accessUnit.push_back( new NALUnitEBSP( nalu ) );
    numBytes += unsigned( accessUnit.back()->m_nalUnitData.str().size() );
  }

  xCabacZeroWordPadding( pic, slice, pic.sliceDataNumBins, numBytes, accessUnit.back()->m_nalUnitData );

  return numBytes * 8;
}

void EncGOP::xWriteLeadingSEIs( const Picture& pic, AccessUnitList& accessUnit )
{
  const Slice* slice = pic.slices[ 0 ];
  SEIMessages leadingSeiMessages;

  bool bpPresentInAU = false;

  if((m_pcEncCfg->m_bufferingPeriodSEIEnabled) && (slice->isIRAP() || slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_GDR) &&
    slice->nuhLayerId==slice->vps->layerId[0] && (slice->sps->hrdParametersPresent))
  {
    SEIBufferingPeriod *bufferingPeriodSEI = new SEIBufferingPeriod();
    bool noLeadingPictures = ( (slice->nalUnitType!= VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL) && (slice->nalUnitType!= VVENC_NAL_UNIT_CODED_SLICE_CRA) );
    m_seiEncoder.initBufferingPeriodSEI(*bufferingPeriodSEI, noLeadingPictures);
    m_EncHRD.bufferingPeriodSEI = *bufferingPeriodSEI;
    m_EncHRD.bufferingPeriodInitialized = true;

    leadingSeiMessages.push_back(bufferingPeriodSEI);
    bpPresentInAU = true;
  }

//  if (m_pcEncCfg->m_dependentRAPIndicationSEIEnabled && slice->isDRAP )
//  {
//    SEIDependentRAPIndication *dependentRAPIndicationSEI = new SEIDependentRAPIndication();
//    m_seiEncoder.initDrapSEI( dependentRAPIndicationSEI );
//    leadingSeiMessages.push_back(dependentRAPIndicationSEI);
//  }

  if( m_pcEncCfg->m_pictureTimingSEIEnabled && m_pcEncCfg->m_bufferingPeriodSEIEnabled )
  {
    SEIMessages nestedSeiMessages;
    SEIMessages duInfoSeiMessages;
    uint32_t numDU = 1;
    m_seiEncoder.initPictureTimingSEI( leadingSeiMessages, nestedSeiMessages, duInfoSeiMessages, slice, numDU, bpPresentInAU );
  }

  if( m_pcEncCfg->m_preferredTransferCharacteristics )
  {
    SEIAlternativeTransferCharacteristics *seiAlternativeTransferCharacteristics = new SEIAlternativeTransferCharacteristics;
    m_seiEncoder.initSEIAlternativeTransferCharacteristics( seiAlternativeTransferCharacteristics );
    leadingSeiMessages.push_back(seiAlternativeTransferCharacteristics);
  }

  // mastering display colour volume
  if( (m_pcEncCfg->m_masteringDisplay[0] != 0 && m_pcEncCfg->m_masteringDisplay[1] != 0) ||
      m_pcEncCfg->m_masteringDisplay[8] )
  {
    SEIMasteringDisplayColourVolume *sei = new SEIMasteringDisplayColourVolume;
    m_seiEncoder.initSEIMasteringDisplayColourVolume(sei);
    leadingSeiMessages.push_back(sei);
  }

  // content light level
  if( m_pcEncCfg->m_contentLightLevel[0] != 0 && m_pcEncCfg->m_contentLightLevel[1] != 0 )
  {
    SEIContentLightLevelInfo *seiCLL = new SEIContentLightLevelInfo;
    m_seiEncoder.initSEIContentLightLevel(seiCLL);
    leadingSeiMessages.push_back(seiCLL);
  }


  // Note: using accessUnit.end() works only as long as this function is called after slice coding and before EOS/EOB NAL units
  AccessUnitList::iterator pos = accessUnit.end();
  xWriteSEISeparately( VVENC_NAL_UNIT_PREFIX_SEI, leadingSeiMessages, accessUnit, pos, slice->TLayer, slice->sps );

  deleteSEIs( leadingSeiMessages );
}

void EncGOP::xWriteTrailingSEIs( const Picture& pic, AccessUnitList& accessUnit, std::string& digestStr )
{
  const Slice* slice = pic.slices[ 0 ];
  SEIMessages trailingSeiMessages;

  if ( m_pcEncCfg->m_decodedPictureHashSEIType != VVENC_HASHTYPE_NONE )
  {
    SEIDecodedPictureHash *decodedPictureHashSei = new SEIDecodedPictureHash();
    const CPelUnitBuf recoBuf = pic.cs->getRecoBuf();
    m_seiEncoder.initDecodedPictureHashSEI( *decodedPictureHashSei, recoBuf, digestStr, slice->sps->bitDepths );
    trailingSeiMessages.push_back( decodedPictureHashSei );
  }

  // Note: using accessUnit.end() works only as long as this function is called after slice coding and before EOS/EOB NAL units
  AccessUnitList::iterator pos = accessUnit.end();
  xWriteSEISeparately( VVENC_NAL_UNIT_SUFFIX_SEI, trailingSeiMessages, accessUnit, pos, slice->TLayer, slice->sps );

  deleteSEIs( trailingSeiMessages );
}

int EncGOP::xWriteVPS ( AccessUnitList &accessUnit, const VPS *vps, HLSWriter& hlsWriter )
{
  OutputNALUnit nalu(VVENC_NAL_UNIT_VPS);
  hlsWriter.setBitstream( &nalu.m_Bitstream );
  hlsWriter.codeVPS( vps );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWriteDCI ( AccessUnitList &accessUnit, const DCI *dci, HLSWriter& hlsWriter )
{
  if (dci->dciId ==0)
  {
    return 0;
  }

  OutputNALUnit nalu(VVENC_NAL_UNIT_DCI);
  hlsWriter.setBitstream( &nalu.m_Bitstream );
  hlsWriter.codeDCI( dci );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWriteSPS ( AccessUnitList &accessUnit, const SPS *sps, HLSWriter& hlsWriter )
{
  OutputNALUnit nalu(VVENC_NAL_UNIT_SPS);
  hlsWriter.setBitstream( &nalu.m_Bitstream );
  hlsWriter.codeSPS( sps );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWritePPS ( AccessUnitList &accessUnit, const PPS *pps, const SPS *sps, HLSWriter& hlsWriter )
{
  OutputNALUnit nalu(VVENC_NAL_UNIT_PPS);
  hlsWriter.setBitstream( &nalu.m_Bitstream );
  hlsWriter.codePPS( pps, sps );
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

int EncGOP::xWriteAPS( AccessUnitList &accessUnit, const APS *aps, HLSWriter& hlsWriter, vvencNalUnitType eNalUnitType )
{
  OutputNALUnit nalu(eNalUnitType, aps->temporalId);
  hlsWriter.setBitstream(&nalu.m_Bitstream);
  hlsWriter.codeAPS(aps);
  accessUnit.push_back(new NALUnitEBSP(nalu));
  return (int)(accessUnit.back()->m_nalUnitData.str().size()) * 8;
}

void EncGOP::xWriteAccessUnitDelimiter ( AccessUnitList &accessUnit, Slice* slice, bool IrapOrGdr, HLSWriter& hlsWriter )
{
  OutputNALUnit nalu(VVENC_NAL_UNIT_ACCESS_UNIT_DELIMITER, slice->TLayer);
  hlsWriter.setBitstream(&nalu.m_Bitstream);
  hlsWriter.codeAUD( IrapOrGdr, 2-slice->sliceType );
  accessUnit.push_front(new NALUnitEBSP(nalu));
}

void EncGOP::xWriteSEI (vvencNalUnitType naluType, SEIMessages& seiMessages, AccessUnitList &accessUnit, AccessUnitList::iterator &auPos, int temporalId, const SPS *sps)
{
  if (seiMessages.empty())
  {
    return;
  }
  OutputNALUnit nalu(naluType, temporalId);
  m_seiWriter.writeSEImessages(nalu.m_Bitstream, seiMessages, m_EncHRD, false, temporalId);
  auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
  auPos++;
}

void EncGOP::xWriteSEISeparately (vvencNalUnitType naluType, SEIMessages& seiMessages, AccessUnitList &accessUnit, AccessUnitList::iterator &auPos, int temporalId, const SPS *sps)
{
  if (seiMessages.empty())
  {
    return;
  }
  for (SEIMessages::const_iterator sei = seiMessages.begin(); sei!=seiMessages.end(); sei++ )
  {
    SEIMessages tmpMessages;
    tmpMessages.push_back(*sei);
    OutputNALUnit nalu(naluType, temporalId);
    m_seiWriter.writeSEImessages(nalu.m_Bitstream, tmpMessages, m_EncHRD, false, temporalId);
    auPos = accessUnit.insert(auPos, new NALUnitEBSP(nalu));
    auPos++;
  }
}

/** Attaches the input bitstream to the stream in the output NAL unit
    Updates rNalu to contain concatenated bitstream. rpcBitstreamRedirect is cleared at the end of this function call.
 *  \param codedSliceData contains the coded slice data (bitstream) to be concatenated to rNalu
 *  \param rNalu          target NAL unit
 */
void EncGOP::xAttachSliceDataToNalUnit( OutputNALUnit& rNalu, const OutputBitstream* codedSliceData )
{
  // Byte-align
  rNalu.m_Bitstream.writeByteAlignment();   // Slice header byte-alignment

  // Perform bitstream concatenation
  if (codedSliceData->getNumberOfWrittenBits() > 0)
  {
    rNalu.m_Bitstream.addSubstream(codedSliceData);
  }
}

void EncGOP::xCabacZeroWordPadding( const Picture& pic, const Slice* slice, uint32_t binCountsInNalUnits, uint32_t numBytesInVclNalUnits, std::ostringstream &nalUnitData )
{
  const PPS &pps                     = *(slice->pps);
  const SPS &sps                     = *(slice->sps);
  const ChromaFormat format          = sps.chromaFormatIdc;
  const int log2subWidthCxsubHeightC = getComponentScaleX( COMP_Cb, format ) + getComponentScaleY( COMP_Cb, format );
  const int minCUSize                = pic.cs->pcv->minCUSize;
  const int paddedWidth              = ( (pps.picWidthInLumaSamples  + minCUSize - 1) / minCUSize) * minCUSize;
  const int paddedHeight             = ( (pps.picHeightInLumaSamples + minCUSize - 1) / minCUSize) * minCUSize;
  const int rawBits                  = paddedWidth * paddedHeight * ( sps.bitDepths[ CH_L ] + 2 * ( sps.bitDepths[ CH_C ] >> log2subWidthCxsubHeightC ) );
  const uint32_t threshold           = ( 32/3 ) * numBytesInVclNalUnits + ( rawBits/32 );
  if ( binCountsInNalUnits >= threshold )
  {
    // need to add additional cabac zero words (each one accounts for 3 bytes (=00 00 03)) to increase numBytesInVclNalUnits
    const uint32_t targetNumBytesInVclNalUnits = ( ( binCountsInNalUnits - ( rawBits/32 ) ) * 3 + 31 ) / 32;

    if ( targetNumBytesInVclNalUnits>numBytesInVclNalUnits ) // It should be!
    {
      const uint32_t numberOfAdditionalBytesNeeded    = targetNumBytesInVclNalUnits - numBytesInVclNalUnits;
      const uint32_t numberOfAdditionalCabacZeroWords = ( numberOfAdditionalBytesNeeded + 2 ) / 3;
      const uint32_t numberOfAdditionalCabacZeroBytes = numberOfAdditionalCabacZeroWords * 3;
      if ( m_pcEncCfg->m_cabacZeroWordPaddingEnabled )
      {
        std::vector<uint8_t> zeroBytesPadding(numberOfAdditionalCabacZeroBytes, uint8_t(0));
        for( uint32_t i = 0; i < numberOfAdditionalCabacZeroWords; i++ )
        {
          zeroBytesPadding[ i * 3 + 2 ] = 3;  // 00 00 03
        }
        nalUnitData.write( reinterpret_cast<const char*>(&(zeroBytesPadding[ 0 ])), numberOfAdditionalCabacZeroBytes );
        msg.log( VVENC_NOTICE, "Adding %d bytes of padding\n", numberOfAdditionalCabacZeroWords * 3 );
      }
      else
      {
        msg.log( VVENC_NOTICE, "Standard would normally require adding %d bytes of padding\n", numberOfAdditionalCabacZeroWords * 3 );
      }
    }
  }
}

void EncGOP::xAddPSNRStats( const Picture* pic, CPelUnitBuf cPicD, AccessUnitList& accessUnit, bool printFrameMSE, double* PSNR_Y, bool isEncodeLtRef )
{
  const Slice* slice         = pic->slices[0];

  double dPSNR[MAX_NUM_COMP];
  double MSEyuvframe[MAX_NUM_COMP];
  for (int i = 0; i < MAX_NUM_COMP; i++)
  {
    dPSNR[i]       = pic->psnr[i];
    MSEyuvframe[i] = pic->mse[i];
  }

  /* calculate the size of the access unit, excluding:
   *  - any AnnexB contributions (start_code_prefix, zero_byte, etc.,)
   *  - SEI NAL units
   */
  uint32_t numRBSPBytes = 0;
  for (AccessUnitList::const_iterator it = accessUnit.begin(); it != accessUnit.end(); it++)
  {
    uint32_t numRBSPBytes_nal = uint32_t((*it)->m_nalUnitData.str().size());
    if (m_pcEncCfg->m_summaryVerboseness > 0)
    {
      msg.log( VVENC_NOTICE, "*** %s numBytesInNALunit: %u\n", nalUnitTypeToString((*it)->m_nalUnitType), numRBSPBytes_nal);
    }
    if( ( *it )->m_nalUnitType != VVENC_NAL_UNIT_PREFIX_SEI && ( *it )->m_nalUnitType != VVENC_NAL_UNIT_SUFFIX_SEI )
    {
      numRBSPBytes += numRBSPBytes_nal;
      if (it == accessUnit.begin() || (*it)->m_nalUnitType == VVENC_NAL_UNIT_VPS || (*it)->m_nalUnitType == VVENC_NAL_UNIT_DCI || (*it)->m_nalUnitType == VVENC_NAL_UNIT_SPS || (*it)->m_nalUnitType == VVENC_NAL_UNIT_PPS || (*it)->m_nalUnitType == VVENC_NAL_UNIT_PREFIX_APS || (*it)->m_nalUnitType == VVENC_NAL_UNIT_SUFFIX_APS)
      {
        numRBSPBytes += 4;
      }
      else
      {
        numRBSPBytes += 3;
      }
    }
  }

  const uint32_t uibits = numRBSPBytes * 8;

  if (m_isPreAnalysis || !m_pcRateCtrl->rcIsFinalPass)
  {
    m_pcRateCtrl->addRCPassStats( slice->poc,
                                  slice->sliceQp,
                                  slice->getLambdas()[0],
                                  pic->picVisActY,
                                  uibits,
                                  dPSNR[COMP_Y],
                                  slice->isIntra(),
                                  slice->TLayer,
                                  pic->gopEntry->m_isStartOfIntra,
                                  pic->gopEntry->m_isStartOfGop,
                                  pic->gopEntry->m_gopNum,
                                  pic->minNoiseLevels );
  }

  //===== add PSNR =====
  m_AnalyzeAll.addResult(dPSNR, (double)uibits, MSEyuvframe
    , isEncodeLtRef
  );
  if ( slice->isIntra() )
  {
    m_AnalyzeI.addResult(dPSNR, (double)uibits, MSEyuvframe
      , isEncodeLtRef
    );
    *PSNR_Y = dPSNR[COMP_Y];
  }
  if ( slice->isInterP() )
  {
    m_AnalyzeP.addResult(dPSNR, (double)uibits, MSEyuvframe
      , isEncodeLtRef
    );
    *PSNR_Y = dPSNR[COMP_Y];
  }
  if ( slice->isInterB() )
  {
    m_AnalyzeB.addResult(dPSNR, (double)uibits, MSEyuvframe
      , isEncodeLtRef
    );
    *PSNR_Y = dPSNR[COMP_Y];
  }

  char c = (slice->isIntra() ? 'I' : slice->isInterP() ? 'P' : 'B');
  if ( ! pic->isReferenced && pic->refCounter == 0 && ! m_pcEncCfg->m_maxParallelFrames )
  {
    c += 32;
  }

  if( m_pcEncCfg->m_verbosity >= VVENC_NOTICE )
  {
    if ((m_isPreAnalysis && m_pcRateCtrl->m_pcEncCfg->m_RCTargetBitrate) || !m_pcRateCtrl->rcIsFinalPass)
    {
      std::string cInfo;
      if( m_pcRateCtrl->rcIsFinalPass ) // single pass RC
      {
        cInfo = prnt("RC analyze poc %5d", slice->poc );
      }
      else
      {
        cInfo = prnt("RC pass %d/%d, analyze poc %5d",
            m_pcRateCtrl->rcPass + 1,
            m_pcEncCfg->m_RCNumPasses,
            slice->poc );
      }
      accessUnit.InfoString.append( cInfo );
    }
    else
    {
      std::stringstream sMctf;
      if( pic->gopEntry->m_mctfIndex >= 0 )
        sMctf << ", TF " << pic->gopEntry->m_mctfIndex << ")";
      else
        sMctf << ")      ";

      std::string cInfo = prnt("POC %5d TId: %1d (%10s, %c-SLICE, QP %d%s %10d bits",
          slice->poc,
          slice->TLayer,
          nalUnitTypeToString( slice->nalUnitType ),
          c,
          slice->sliceQp,
          sMctf.str().c_str(),
          uibits );

      std::string cPSNR = prnt(" [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", dPSNR[COMP_Y], dPSNR[COMP_Cb], dPSNR[COMP_Cr] );

      accessUnit.InfoString.append( cInfo );
      accessUnit.InfoString.append( cPSNR );

      if ( m_pcEncCfg->m_printHexPsnr )
      {
        uint64_t xPsnr[MAX_NUM_COMP];
        for (int i = 0; i < MAX_NUM_COMP; i++)
        {
          std::copy(reinterpret_cast<uint8_t *>(&dPSNR[i]),
              reinterpret_cast<uint8_t *>(&dPSNR[i]) + sizeof(dPSNR[i]),
              reinterpret_cast<uint8_t *>(&xPsnr[i]));
        }

        std::string cPSNRHex = prnt(" [xY %16" PRIx64 " xU %16" PRIx64 " xV %16" PRIx64 "]", xPsnr[COMP_Y], xPsnr[COMP_Cb], xPsnr[COMP_Cr]);

        accessUnit.InfoString.append( cPSNRHex );
      }

      if( printFrameMSE )
      {
        std::string cFrameMSE = prnt( " [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", MSEyuvframe[COMP_Y], MSEyuvframe[COMP_Cb], MSEyuvframe[COMP_Cr]);
        accessUnit.InfoString.append( cFrameMSE );
      }

      std::string cEncTime = prnt(" [ET %5d ]", pic->encTime.getTimerInSec() );
      accessUnit.InfoString.append( cEncTime );

      std::string cRefPics;
      for( int iRefList = 0; iRefList < 2; iRefList++ )
      {
        std::string tmp = prnt(" [L%d ", iRefList);
        cRefPics.append( tmp );
        for( int iRefIndex = 0; iRefIndex < slice->numRefIdx[ iRefList ]; iRefIndex++ )
        {
          tmp = prnt("%d ", slice->getRefPOC( RefPicList( iRefList ), iRefIndex));
          cRefPics.append( tmp );
        }
        cRefPics.append( "]" );
      }
      accessUnit.InfoString.append( cRefPics );
    }
  }
}

uint64_t EncGOP::xFindDistortionPlane( const CPelBuf& pic0, const CPelBuf& pic1, uint32_t rshift ) const
{
  uint64_t uiTotalDiff;
  const  Pel*  pSrc0 = pic0.bufAt(0, 0);
  const  Pel*  pSrc1 = pic1.bufAt(0, 0);

  CHECK(pic0.width  != pic1.width , "Unspecified error");
  CHECK(pic0.height != pic1.height, "Unspecified error");

  if( rshift > 0 )
  {
    uiTotalDiff = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        uiTotalDiff += uint64_t((iTemp * iTemp) >> rshift);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
    }
  }
  else
  {
    uiTotalDiff = 0;
    for (int y = 0; y < pic0.height; y++)
    {
      for (int x = 0; x < pic0.width; x++)
      {
        Intermediate_Int iTemp = pSrc0[x] - pSrc1[x];
        uiTotalDiff += uint64_t(iTemp * iTemp);
      }
      pSrc0 += pic0.stride;
      pSrc1 += pic1.stride;
    }
  }

  return uiTotalDiff;
}

void EncGOP::xPrintPictureInfo( const Picture& pic, AccessUnitList& accessUnit, const std::string& digestStr, bool printFrameMSE, bool isEncodeLtRef )
{
  double PSNR_Y;
  xAddPSNRStats( &pic, pic.getRecoBuf(), accessUnit, printFrameMSE, &PSNR_Y, isEncodeLtRef );

  if( ! m_isPreAnalysis && m_pcRateCtrl->rcIsFinalPass )
  {
    std::string modeName;
    switch ( m_pcEncCfg->m_decodedPictureHashSEIType )
    {
      case VVENC_HASHTYPE_MD5:
        modeName = "MD5";
        break;
      case VVENC_HASHTYPE_CRC:
        modeName = "CRC";
        break;
      case VVENC_HASHTYPE_CHECKSUM:
        modeName = "Checksum";
        break;
      default:
        break;
    }

    if ( modeName.length() )
    {
      std::string cDigist = prnt(" [%s:%s]", modeName.c_str(), digestStr.empty() ? "?" : digestStr.c_str() );
      accessUnit.InfoString.append( cDigist );
    }
  }

  if( !accessUnit.InfoString.empty() )
  {
    std::string cPicInfo = accessUnit.InfoString;
    cPicInfo.append("\n");
    const vvencMsgLevel msgLevel = m_isPreAnalysis ? VVENC_DETAILS : VVENC_NOTICE;
    msg.log( msgLevel, cPicInfo.c_str() );
    if( m_pcEncCfg->m_verbosity >= msgLevel ) fflush( stdout );
  }
}

} // namespace vvenc

//! \}


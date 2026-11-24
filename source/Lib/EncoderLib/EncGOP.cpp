/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
#include "BitAllocation.h"
#include "EncHRD.h"
#include "GOPCfg.h"

#include <algorithm>
#include <limits>
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

void initPicAuxQPOffsets( const Slice* slice, const bool isBIM ) // get m_picShared->m_picAuxQpOffset and m_picShared->m_ctuBimQpOffset if unavailable
{
  const Picture* slicePic = slice->pic;

  if (isBIM && slicePic && slicePic->m_picShared->m_ctuBimQpOffset.empty())
  {
    const Picture* refPicL0 = slice->getRefPic (REF_PIC_LIST_0, 0);
    const Picture* refPicL1 = slice->getRefPic (REF_PIC_LIST_1, 0);

    if (refPicL0 && !refPicL0->m_picShared->m_ctuBimQpOffset.empty() &&
        refPicL1 && !refPicL1->m_picShared->m_ctuBimQpOffset.empty() &&
        refPicL0->m_picShared->m_ctuBimQpOffset.size() == refPicL1->m_picShared->m_ctuBimQpOffset.size())
    {
      const PicShared* pic0 = refPicL0->m_picShared;
      const PicShared* pic1 = refPicL1->m_picShared;
      PicShared* const picC = slicePic->m_picShared;
      const int32_t  numCtu = (int32_t) pic0->m_ctuBimQpOffset.size();
      int i, sumCtuQpOffset = 0;

      picC->m_ctuBimQpOffset.resize (numCtu);

      for (i = 0; i < numCtu; i++) // scale and merge QPs
      {
        const int qpOffset0 = pic0->m_ctuBimQpOffset[i] + pic0->m_picAuxQpOffset; // CTU delta-QP #1
        const int qpOffset1 = pic1->m_ctuBimQpOffset[i] + pic1->m_picAuxQpOffset; // CTU delta-QP #2
        const int qpOffsetC = (3 * qpOffset0 + 3 * qpOffset1 + (qpOffset0 + qpOffset1 < 0 ? 3 : 4)) >> 3; // 3 instead of 4 for correct rounding to -2

        picC->m_ctuBimQpOffset[i] = qpOffsetC;
        sumCtuQpOffset += qpOffsetC;
      }

      picC->m_picAuxQpOffset = (sumCtuQpOffset + (sumCtuQpOffset < 0 ? -(numCtu >> 1) : numCtu >> 1)) / numCtu; // pic average; delta-QP scaling: 0.75
      for (i = 0; i < numCtu; i++) // excl. average again
      {
        picC->m_ctuBimQpOffset[i] -= picC->m_picAuxQpOffset; // delta-QP relative to the aux average
      }
    }
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
  , m_spsMap             ( MAX_NUM_SPS )
  , m_ppsMap             ( MAX_NUM_PPS )
  , m_isPreAnalysis      ( false )
  , m_bFirstWrite        ( true )
  , m_bRefreshPending    ( false )
  , m_lastCodingNum      ( -1 )
  , m_numPicsCoded       ( 0 )
  , m_numPicsInMissing   ( 0 )
  , m_numPicsOutOffset   ( 0 )
  , m_lastCts            ( 0 )
  , m_pocRecOut          ( 0 )
  , m_ticksPerFrameMul4  ( 0 )
  , m_lastIDR            ( 0 )
  , m_lastRasPoc         ( MAX_INT )
  , m_pocCRA             ( 0 )
  , m_associatedIRAPPOC  ( 0 )
  , m_associatedIRAPType ( VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP )
  , m_picSetupTasks      ( false )
  , m_numSetupsQueued    ( 0 )
  , m_numSetupsDone      ( 0 )
  , m_numPicsStarted     ( 0 )
{
}

EncGOP::~EncGOP()
{
  xWaitFgaTasks();
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

  for( auto& p : m_globalApsList ) delete p;
  m_globalApsList.clear();
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

  if ( encCfg.m_fga )
  {
    // --fga 1: VTM's analysis with its bugs fixed; --fga 2: with the improvements beyond VTM; --fga 3: --fga 2 with the
    // auto-regressive grain model
    m_fgAnalyzer.init( m_pcEncCfg->m_PadSourceWidth, m_pcEncCfg->m_PadSourceHeight, m_pcEncCfg->m_internChromaFormat,
                         m_pcEncCfg->m_internalBitDepth, m_pcEncCfg->m_fg.m_fgcSEICompModelPresent, encCfg.m_fga == 2, encCfg.m_fga == 3 );
    // the analyses of a GOP: its key picture and the pictures FgaMaxTempDepth adds
    const int fgaPicsPerGop = gopCfg->getNumFgaPicsPerGop();
    m_fgAnalyzer.setAnalysesPerGop( fgaPicsPerGop );
    m_fgcParamSets = std::max<size_t>( 8, 2 * fgaPicsPerGop + 2 );
    m_fgcByStartPoc[ std::numeric_limits<int>::min() ] = FilmGrainParam( m_fgAnalyzer, m_pcEncCfg->m_fgaStrength );   // no grain, until the first analysis measures it
  }

  if( !m_pcEncCfg->m_poc0idr )
  {
    m_associatedIRAPType = VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL;
  }
  m_seiEncoder.init( encCfg, gopCfg, m_EncHRD );

  // One CU encoder per worker thread is enough: a worker encodes one CTU at a time, whichever
  // of the parallel pictures it belongs to.
  const int numThreadRsrc = ( encCfg.m_numThreads > 0 && threadPool ) ? threadPool->numThreads() : 1;
  m_threadRsrc.init( encCfg, sps0, pps0, &m_globalCtuQpVector, rateCtrl, numThreadRsrc );

  // IFP: a picture's ALF APSs reach the global list at its ALF derivation; from then on the pictures predicting their
  // APSs from it may start (see the start condition in xProcessPictures)
  auto apsReady = [this]( Picture& pic )
  {
    std::lock_guard<std::mutex> lock( m_gopEncMutex );
    pic.picApsGlobal->initalized = true;
    m_gopEncCond.notify_one();
  };

  const int maxPicEncoder = ( encCfg.m_maxParallelFrames ) ? encCfg.m_maxParallelFrames : 1;
  for ( int i = 0; i < maxPicEncoder; i++ )
  {
    EncPicture* picEncoder = new EncPicture;
    picEncoder->init( encCfg, &m_globalCtuQpVector, sps0, pps0, rateCtrl, threadPool, m_threadRsrc, apsReady );
    m_freePicEncoderList.push_back( picEncoder );
  }

  if (encCfg.m_usePerceptQPA)
  {
    m_globalCtuQpVector.resize( pps0.useDQP && (encCfg.m_internalUsePerceptQPATempFiltISlice == 2) && encCfg.m_salienceBasedOpt ? pps0.picWidthInCtu * pps0.picHeightInCtu + 1 : 1 );
  }

  // With pictures coded in parallel, a picture's setup (EncPicture::compressPicture up to its CTU tasks) runs as a
  // thread pool task instead of on the calling thread, which also runs the pre-processing stages (MCTF). Its results
  // are read only by the picture's own tasks, except with rate control and look-ahead (lock-step, the rate cap, two
  // passes and the pre-analysis encoder read them on the calling thread) and with the pumping reduction of the QPA,
  // whose CTU QP vector is shared by all pictures; those keep the setup on the calling thread. MaxParallelFrames > 0 is
  // needed because only then do the pictures share their unit cache under a mutex.
  m_picSetupTasks = encCfg.m_numThreads > 0 && encCfg.m_maxParallelFrames > 0 && ! isPreAnalysis
                    && encCfg.m_RCTargetBitrate == 0 && encCfg.m_RCNumPasses != 2 && ! encCfg.m_rateCap
                    && encCfg.m_LookAhead <= 0 && m_globalCtuQpVector.size() <= 1;

  if( m_pcEncCfg->m_FrameRate && m_pcEncCfg->m_TicksPerSecond > 0 )
  {
    m_ticksPerFrameMul4 = (int)((int64_t)4 *(int64_t)m_pcEncCfg->m_TicksPerSecond * (int64_t)m_pcEncCfg->m_FrameScale/(int64_t)m_pcEncCfg->m_FrameRate);
  }
  m_forceSCC = false;
  m_rcap.reset();
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
  if( pic->ctsValid )
  {
    if( m_lastCts )
    {
      int64_t ticksPerFrame = m_ticksPerFrameMul4/4;
      int64_t expectedCtsDiff = (m_pcEncCfg->m_TicksPerSecond > 0 ) ? ticksPerFrame : 1;
      int64_t ctsDiff = pic->cts - m_lastCts;

      if( ctsDiff >= (expectedCtsDiff<<1) || ctsDiff < 0 )
      {
        // signalize that frames are missing at that particular picture
        pic->picOutOffset = (m_pcEncCfg->m_TicksPerSecond > 0 ) ? (ctsDiff - ticksPerFrame)/ticksPerFrame : ctsDiff-1;
        m_numPicsInMissing += pic->picOutOffset;
      }
    }
    m_lastCts = pic->cts;
  }
  if( m_numPicsInMissing )
  {
    pic->picsInMissing = m_numPicsInMissing;
  }

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
  pic->finalInit( m_VPS, sps, pps, nullptr, m_shrdUnitCache, mutex, nullptr );

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

bool EncGOP::waitForFreeEncoders()
{
  std::unique_lock<std::mutex> lock( m_gopEncMutex );
  if( xEncodersFinished() )
  {
    return false;
  }
  CHECK( m_pcEncCfg->m_numThreads <= 0, "run into MT code, but no threading enabled" );
  // Return as soon as a picture finished since the last return. A finish after the GOP pass released the lock and
  // before this wait notifies nobody; without the predicate the main thread would sleep until the next finish while
  // the freed encoder and the pictures waiting for that one stay idle.
  m_gopEncCond.wait( lock, [this] { return m_numPicsFinished != m_numPicsFinishedWaited; } );
  m_numPicsFinishedWaited = m_numPicsFinished;
  return true;
}

bool EncGOP::hasEnoughPictures( const PicList& picList ) const
{
  if( EncStage::hasEnoughPictures( picList ) )
  {
    return true;
  }

  // GOPSize + 1 pictures ensure that the GOP of the next picture to initialize is complete (the GOP QP cascade reads all
  // of it). Pictures arrive in POC order and a GOP covers consecutive POCs, so that GOP is also complete once any picture
  // of a later GOP has arrived; the list is in coding order, so its last picture has the highest GOP number. A GOP shorter
  // than GOPSize (the first one, when the intra period is not a multiple of the GOP size) then need not wait for
  // GOPSize + 1 pictures. With look-ahead, the GOP encoder gets its pictures in coding order from the pre-analysis
  // encoder, so the size test alone decides.
  if( m_isPreAnalysis || m_pcEncCfg->m_LookAhead )
  {
    return false;
  }
  const auto nextPic = std::find_if( picList.begin(), picList.end(), []( const Picture* pic ) { return ! pic->isInitDone; } );
  return nextPic != picList.end() && picList.back()->gopEntry->m_gopNum > ( *nextPic )->gopEntry->m_gopNum;
}

void EncGOP::processPictures( const PicList& picList, AccessUnitList& auList, PicList& doneList, PicList& freeList )
{
  CHECK( picList.empty(), "empty input picture list given" );

  // create list of pictures ordered in coding order and ready to be encoded
  xInitPicsInCodingOrder( picList );

  // encode pictures
  xProcessPictures( auList, doneList );

  // output reconstructed YUV
  xOutputRecYuv( picList );

  // release pictures not needed anymore
  xReleasePictures( picList, freeList );

  // clear output access unit
  if( m_isPreAnalysis )
  {
    auList.clearAu();
  }
}

// Whether a reference picture of pic has not started yet (it is still in the processing list).
static bool hasUnstartedRef( const Picture& pic, const std::list<Picture*>& procList )
{
  const Slice& slice = *pic.slices[ 0 ];
  for( int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++ )
  {
    for( int i = 0; i < slice.numRefIdx[ refList ]; i++ )
    {
      if( std::find( procList.begin(), procList.end(), slice.refPicList[ refList ][ i ] ) != procList.end() )
      {
        return true;
      }
    }
  }
  return false;
}

// The ready picture to start next: the key pictures (temporal layer 0 or 1) of the list front's GOP and the next one
// first, the lower layer first, then the others in list order. The key pictures are the long ones every later picture
// depends on; started early, they overlap the previous GOP's upper layers instead of running nearly alone after them.
// Only one GOP ahead: ranking further ahead would keep overtaking the next picture to output (the output would stall).
// With IFP a picture may start before its references are reconstructed, but only after they have started (else every
// picture encoder could wait on rows nobody codes), and after any picture whose references are reconstructed: a chain
// of row-locked pictures must not hold every picture encoder while independent work waits.
template<typename IsReady>
static std::list<Picture*>::iterator findNextPicToStart( std::list<Picture*>& procList, bool ifp, IsReady isReady )
{
  // ranks: 0 and 1 the key pictures' layers, NON_KEY the others; ROW_LOCKED is added for a row-locked start
  const int NON_KEY = 2, ROW_LOCKED = 3;
  if( procList.empty() )
  {
    return procList.end();
  }
  const int maxRankedGop = procList.front()->gopEntry->m_gopNum + 1;   // the last GOP whose key pictures go first
  auto best     = procList.end();
  int  bestRank = std::numeric_limits<int>::max();
  for( auto it = procList.begin(); it != procList.end(); ++it )
  {
    const Picture* pic     = *it;
    const int      tlRank  = pic->gopEntry->m_gopNum <= maxRankedGop ? std::min<int>( pic->TLayer, NON_KEY ) : NON_KEY;
    // the rank is at least tlRank; equal ranks keep the first in list order
    if( tlRank >= bestRank || ! isReady( pic ) )
    {
      continue;
    }
    const bool rowLocked = ifp && ! pic->slices[ 0 ]->checkAllRefPicsReconstructed();
    if( rowLocked && hasUnstartedRef( *pic, procList ) )
    {
      continue;
    }
    const int rank = ( rowLocked ? ROW_LOCKED : 0 ) + tlRank;
    if( rank < bestRank )
    {
      best     = it;
      bestRank = rank;
      if( rank == 0 )
      {
        break;
      }
    }
  }
  CHECKD( best != procList.end() && ifp && ! ( *best )->slices[ 0 ]->checkAllRefPicsReconstructed()
            && hasUnstartedRef( **best, procList ), "a row-locked picture must not start before its references" );
  return best;
}

// In lock-step mode (rate control with frame-parallel coding) pictures are coded in chunks, each after the previous
// chunk's rate-control update.
bool EncGOP::xIsLockStep() const
{
  return ( m_pcEncCfg->m_RCTargetBitrate > 0 || ( m_pcEncCfg->m_LookAhead > 0 && ! m_isPreAnalysis ) ) && m_pcEncCfg->m_maxParallelFrames > 0;
}

// Whether new pictures join the processing list on every pass and start by temporal layer (findNextPicToStart), so that
// the long key pictures of the next GOP overlap the current GOP. Otherwise the list is refilled when empty and pictures
// start in list order: with one picture encoder (its start order is the coding order, which AMaxBT follows), lock-step
// RC, rate cap, and the I pictures' temporal pumping reduction, whose CTU QP memory (sized only then) the key pictures
// fill and the next I picture reads. With IFP and NumParallelGOPs 0 the list still takes one GOP at a time
// (xGetProcessingLists), so the next GOP's key pictures start early only from the pictures already held.
bool EncGOP::xFetchOnEveryPass( bool lockStepMode ) const
{
  const bool ctuQpMemory = m_globalCtuQpVector.size() > 1;
  return m_pcEncCfg->m_maxParallelFrames > 0 && ! lockStepMode && ! m_pcEncCfg->m_rateCap && ! ctuQpMemory;
}

// Starts ready pictures of the processing list while picture encoders are free, at most maxStarts of them (0: no limit).
// In blocking mode it waits for a picture encoder or a reference as needed.
void EncGOP::xStartPictures( bool lockStepMode, bool fetchOnEveryPass, int maxStarts )
{
  int numStarted = 0;
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
      // if ALF enabled and ALFTempPred is used, ensure that refAps is initialized
      // rate capping and MT frame parallel: in the first GOP after scene cut, ensure that the two first frames of 
      //                                     this GOP are finished. Their data will be used to adjust the QP of
      //                                     remaining frames of this scene-cut-GOP.
      const std::list<Picture *>* rcUpdateList = &m_rcUpdateList;
      const VVEncCfg* encCfg = m_pcEncCfg;
      auto isReady = [encCfg, rcUpdateList]( const Picture* cand ) {
        return ( encCfg->m_ifp || cand->slices[ 0 ]->checkAllRefPicsReconstructed() )
          && ( !encCfg->m_alf || ( !cand->refApsGlobal || cand->refApsGlobal->initalized ) )
          && ( !encCfg->m_rateCap || !encCfg->m_maxParallelFrames || !cand->isSceneCutGOP || (!rcUpdateList->front()->isSceneCutCheckAdjQP && !rcUpdateList->front()->gopEntry->m_isStartOfGop ) )
          ; };
      auto picItr = fetchOnEveryPass ? findNextPicToStart( m_procList, m_pcEncCfg->m_ifp, isReady )
                                     : find_if( m_procList.begin(), m_procList.end(), isReady );

      const bool nextPicReady = picItr != m_procList.end();

      // check at least one picture and one pic encoder ready
      if( m_freePicEncoderList.empty() || ! nextPicReady )
      {
        // non-blocking stage: wait on top level, let other stages do their jobs
        // in non-lockstep mode, check if next picture can be output
        if( isNonBlocking() || ( ! lockStepMode && m_gopEncListOutput.front()->isEncoded ) )
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

        if( pic->isFlush )
        {
          m_pcRateCtrl->setRCRateSavingState(0); // tell budget estimation that end of video is near
        }
        if( pic->gopEntry->m_isStartOfGop )
        {
          // check the RC final pass requirement for availability of preprocessed pictures (GOP + 1)
          if( m_pcRateCtrl->lastPOCInCache() <= pic->poc && ! pic->isFlush )
          {
            break;
          }
          m_pcRateCtrl->processFirstPassData( pic->isFlush, pic->poc );
        }
      }

      m_freePicEncoderList.pop_front();
    }

    CHECK( picEncoder == nullptr, "no free picture encoder available" );
    CHECK( pic        == nullptr, "no picture to be encoded, ready for encoding" );
    m_procList.remove( pic );

    xEncodePicture( pic, picEncoder );
    if( ++numStarted == maxStarts )
    {
      break;
    }
  }
}

// The poll of the earlier stages (EncStage::pollStartStage) while the main thread waits in them: starts up to two
// ready pictures (so the stage's wait is not delayed much), never outputs or updates rate control. Not with rate
// control or look-ahead (a picture's RC init reads the updates made at output) or the rate cap, whose starts wait for
// those updates.
void EncGOP::startReadyPictures()
{
  CHECKD( m_isPreAnalysis, "the pre-analysis encoder is not polled" );
  if( ! isNonBlocking() || m_pcEncCfg->m_RCTargetBitrate > 0 || m_pcEncCfg->m_LookAhead > 0 || m_pcEncCfg->m_rateCap || m_procList.empty() )
  {
    return;
  }
  xStartPictures( false, xFetchOnEveryPass( false ), 2 );
}

void EncGOP::xProcessPictures( AccessUnitList& auList, PicList& doneList )
{
  // in lockstep mode, process all pictures in processing list
  const bool lockStepMode = xIsLockStep();

  // get list of pictures to be encoded and used for RC update
  CHECK( m_pcEncCfg->m_rateCap && lockStepMode, "Rate capping should not be used in lockstep mode" );
  // rate cap and MT: finish the previous GOP before processing the next one
  const bool rateCapPrevGopConstr = m_pcEncCfg->m_rateCap && !m_rcUpdateList.empty();

  const bool fetchOnEveryPass = xFetchOnEveryPass( lockStepMode );
  if( ( m_procList.empty() || fetchOnEveryPass ) && (!m_gopEncListInput.empty() || !m_rcInputReorderList.empty()) && !rateCapPrevGopConstr )
  {
    xGetProcessingLists( m_procList, m_rcUpdateList, lockStepMode );
  }

  if( ! m_procList.empty() )
  {
    // encode one picture in serial mode / multiple pictures in FPP mode
    PROFILER_ACCUM_AND_START_NEW_SET( 1, g_timeProfiler, P_IGNORE );
    xStartPictures( lockStepMode, fetchOnEveryPass, 0 );
  }

  if( lockStepMode && m_pcEncCfg->m_ifpLines && !m_rcUpdateList.empty() )
  {
    xUpdateRcIfp();
  }

  if( m_pcEncCfg->m_rateCap )
  {
    xUpdateRateCap();
  }

  // picture/AU output
  // 
  // in lock-step mode:
  // the output of a picture is connected to evaluation of the lock-step-chunk
  // if the next picture to output belongs to the current chunk, do output (evaluation) when all pictures of the chunk are finished

  if( m_gopEncListOutput.empty() || !m_gopEncListOutput.front()->isEncoded ||
    ( lockStepMode && !m_pcEncCfg->m_ifpLines && !m_rcUpdateList.empty() && m_gopEncListOutput.front() == m_rcUpdateList.front() && !xLockStepPicsFinished() ) )
  {
    return;
  }
  PROFILER_ACCUM_AND_START_NEW_SET( 1, g_timeProfiler, P_TOP_LEVEL );

  // AU output
  Picture* outPic = m_gopEncListOutput.front();
  m_gopEncListOutput.pop_front();

  xWritePicture( *outPic, auList, false );

  // update pending RC
  // first pic has been written to bitstream
  // therefore we have at least for this picture a valid total bit and head bit count
  if( !m_rcUpdateList.empty() && m_rcUpdateList.front() == outPic && (!lockStepMode || !m_pcEncCfg->m_ifpLines)  )
  {
    if( m_pcEncCfg->m_RCTargetBitrate > 0 )
    {
      for( auto pic : m_rcUpdateList )
      {
        if( pic != outPic )
        {
          pic->actualHeadBits  = outPic->actualHeadBits;
          pic->actualTotalBits = pic->sliceDataStreams[0].getNumberOfWrittenBits();
        }
        m_pcRateCtrl->updateAfterPicEncRC( pic );
      }
    }

    if( lockStepMode )
      m_rcUpdateList.clear();
    else
      m_rcUpdateList.pop_front();
  }

  const bool skipFirstPass = ( ! m_pcRateCtrl->rcIsFinalPass || m_isPreAnalysis ) && outPic->gopEntry->m_skipFirstPass;
  if( m_pcEncCfg->m_useAMaxBT && ! skipFirstPass )
  {
    m_BlkStat.updateMaxBT( *outPic->slices[0], outPic->picBlkStat );
  }

  outPic->slices[ 0 ]->updateRefPicCounter( -1 );
  outPic->isFinished = true;

  if( ! m_isPreAnalysis )
  {
    // a film grain analysis may still read the filtered picture
    for( auto& task : m_fgaTasks )
    {
      if( task->pic == outPic )
      {
        task->counter.wait();
      }
    }
    outPic->getFilteredOrigBuffer().destroy();
  }

  doneList.push_back( outPic );

  m_numPicsCoded += 1;
}

void EncGOP::xSyncAlfAps( Picture& pic )
{
  Slice& slice = *pic.cs->slice;
  const bool mtPicParallel = m_pcEncCfg->m_numThreads > 0;

  if( mtPicParallel && slice.isIntra() )
  {
    // reset APS propagation on Intra-Slice in MT-mode
    return;
  }

  const PicApsGlobal* refAps = pic.refApsGlobal;
  if( !refAps )
    return;
  CHECK( !refAps->initalized, "Attempt referencing from an uninitialized APS" );
  pic.refApsGlobal->refCnt--;
  CHECK( pic.refApsGlobal->refCnt < 0, "Not expected APS ref. counter\n" );

  // copy ref APSs to current picture
  const ParameterSetMap<APS>& src = refAps->apsMap;
  ParameterSetMap<APS>&       dst = pic.picApsMap;
  if( mtPicParallel && pic.TLayer == 0 )
  {
    // in pic.parallel case, due to limited number of APS IDs, limit propagation of TID-0 APS
    CHECK( slice.sps->maxTLayers > ALF_CTB_MAX_NUM_APS, "Not enough space for ALF APSs in MT mode: not supported"  )
    int numApsTID0 = ALF_CTB_MAX_NUM_APS - (int)slice.sps->maxTLayers;
    int lastTakenApsPOC = pic.poc;
    while( numApsTID0 > 0 )
    {
      const APS* candAPS = nullptr;
      int candMapIdx = 0;
      for( int i = 0; i < ALF_CTB_MAX_NUM_APS; i++ )
      {
        const int mapIdx = ( i << NUM_APS_TYPE_LEN ) + ALF_APS;
        const APS* srcAPS = src.getPS( mapIdx );
        if( srcAPS && srcAPS->apsId != MAX_UINT && srcAPS->poc < lastTakenApsPOC && ( !candAPS || srcAPS->poc > candAPS->poc ) )
        {
          candAPS = srcAPS;
          candMapIdx = mapIdx;
        }
      }
      if( !candAPS )
        break;

      APS* dstAPS = dst.allocatePS( candMapIdx );
      *dstAPS = *candAPS;
      dst.clearChangedFlag( candMapIdx );
      lastTakenApsPOC = candAPS->poc;
      numApsTID0--;
    }
  }
  else
  {
    for( int i = 0; i < ALF_CTB_MAX_NUM_APS; i++ )
    {
      const int apsMapIdx = ( i << NUM_APS_TYPE_LEN ) + ALF_APS;
      const APS* srcAPS = src.getPS( apsMapIdx );
      if( srcAPS )
      {
        APS* dstAPS = dst.allocatePS( apsMapIdx );
        *dstAPS = *srcAPS;
        dst.clearChangedFlag( apsMapIdx );
      }
    }
  }
  dst.setApsIdStart( src.getApsIdStart() );
}

void EncGOP::xEncodePicture( Picture* pic, EncPicture* picEncoder )
{
  // first pass temporal down-sampling
  if( ( ! m_pcRateCtrl->rcIsFinalPass || m_isPreAnalysis ) && pic->gopEntry->m_skipFirstPass )
  {
    pic->isReconstructed = true;
    pic->isEncoded       = true;
    m_freePicEncoderList.push_back( picEncoder );
    return;
  }

  // decoder in encoder
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "finalpass", m_pcRateCtrl->rcIsFinalPass ? 1: 0 ) );

  if( m_pcEncCfg->m_alf && m_pcEncCfg->m_alfTempPred )
  {
    // Establish reference APS for current picture
    xSyncAlfAps( *pic );
  }

  // initialize next picture
  pic->isPreAnalysis = m_isPreAnalysis;
  // before any task of the picture is queued: its tasks' priority (EncSlice::xProcessCtus)
  pic->encStartOrder = ++m_numPicsStarted;

  if( pic->slices[0]->TLayer + 1 < m_pcEncCfg->m_maxTLayer ) // skip for highest two temporal levels
  {
    initPicAuxQPOffsets( pic->slices[0], m_pcEncCfg->hasBlockQpOffsets() );
  }

  if( m_pcEncCfg->m_RCTargetBitrate > 0 )
  {
    pic->picInitialQP     = -1;
    pic->picInitialLambda = -1.0;

    m_pcRateCtrl->initRateControlPic( *pic, pic->slices[0], pic->picInitialQP, pic->picInitialLambda );
  }

  if( pic->isSceneCutGOP && !pic->isSceneCutCheckAdjQP && !pic->gopEntry->m_isStartOfGop && m_rcap.gopAdaptedQPAdj )
  {
    pic->gopAdaptedQP += m_rcap.gopAdaptedQPAdj;
  }

  // finish picture encoding and cleanup, once all CTU tasks of the picture are done
  static auto finishTask = []( int, void* task_param )
  {
    FinishTaskParam* param = static_cast<FinishTaskParam*>( task_param );
    // The pictures that reference this one may start as soon as what they read is final: the reconstruction and
    // motion (done by the CTU tasks) and the ALF APSs. Writing the slice data, the distortion and the cleanup run on
    // this one thread after that and do not change the reconstruction, motion or APSs.
    param->picEncoder->finalizeReference( *param->pic );
    {
      std::lock_guard<std::mutex> lock( param->gopEncoder->m_gopEncMutex );
      param->pic->isReconstructed = true;
      if( param->pic->picApsGlobal )
        param->pic->picApsGlobal->initalized = true;
      // counted like a finish, so that waitForFreeEncoders returns for it: the pictures referencing this one may start
      param->gopEncoder->m_numPicsFinished++;
      param->gopEncoder->m_gopEncCond.notify_one();
    }
    param->picEncoder->finalizePicture( *param->pic );
    param->gopEncoder->xStoreLostGrainSpectra( *param->pic );
    {
      std::lock_guard<std::mutex> lock( param->gopEncoder->m_gopEncMutex );
      param->pic->isEncoded = true;
      param->gopEncoder->m_freePicEncoderList.push_back( param->picEncoder );
      param->gopEncoder->m_numPicsFinished++;
      param->gopEncoder->m_gopEncCond.notify_one();
    }
    delete param;
    return true;
  };

  if( m_picSetupTasks )
  {
    // compress next picture in a thread pool task, see init(). The setups run one at a time in start order, as they did on
    // the calling thread: with inter-frame parallelism, a picture's CTU tasks wait for CTU lines of its references, which
    // only exist once the references' setups have run.
    static auto setupTask = []( int, void* task_param )
    {
      FinishTaskParam* param = static_cast<FinishTaskParam*>( task_param );
      EncGOP* gopEncoder     = param->gopEncoder;
      param->picEncoder->compressPicture( *param->pic, *gopEncoder );
      gopEncoder->m_numSetupsDone++;
      // The finish task is added only now: until compressPicture added the CTU tasks, the done barrier of the picture
      // encoder's CTU task counter is still open from its previous picture. Once it is added, the finish task may run,
      // delete param and let the encoder finish, so nothing is touched after it. It has the same priority as the
      // picture's CTU tasks (EncSlice::xProcessCtus).
      gopEncoder->m_threadPool->addBarrierTask( finishTask, param, nullptr, nullptr, { &param->picEncoder->m_ctuTasksDoneCounter.done }, nullptr,
                                                1 + param->pic->encStartOrder );
      return true;
    };
    static auto setupReady = []( int, void* task_param )
    {
      const FinishTaskParam* param = static_cast<const FinishTaskParam*>( task_param );
      return param->gopEncoder->m_numSetupsDone == param->setupNum;
    };
    FinishTaskParam* param = new FinishTaskParam( this, picEncoder, pic );
    param->setupNum        = m_numSetupsQueued++;
    // the most urgent priority (0), as the pre-analysis rows: a setup is short and holds back all of its picture's tasks
    m_threadPool->addBarrierTask( setupTask, param, nullptr, nullptr, {}, setupReady, 0 );
    return;
  }

  // compress next picture
  picEncoder->compressPicture( *pic, *this );

  // finish picture encoding and cleanup
  if( m_pcEncCfg->m_numThreads > 0 )
  {
    FinishTaskParam* param = new FinishTaskParam( this, picEncoder, pic );
    // the priority of the picture's CTU tasks (EncSlice::xProcessCtus): it frees the picture encoder, so it ranks with them
    m_threadPool->addBarrierTask( finishTask, param, nullptr, nullptr, { &picEncoder->m_ctuTasksDoneCounter.done }, nullptr,
                                  1 + pic->encStartOrder );
  }
  else
  {
    picEncoder->finalizeReference( *pic );
    picEncoder->finalizePicture( *pic );
    xStoreLostGrainSpectra( *pic );
    pic->isReconstructed = true;
    pic->isEncoded       = true;
    if( pic->picApsGlobal ) pic->picApsGlobal->initalized = true;
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
        if( ! pic->isEncoded )
          return;

        const PPS& pps = *(pic->cs->pps);
        vvencYUVBuffer yuvBuffer;
        vvenc_YUVBuffer_default( &yuvBuffer );
        setupYuvBuffer( pic->getRecoBuf(), yuvBuffer, &pps.conformanceWindow );
        yuvBuffer.sequenceNumber = pic->poc;
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
      if( pic->isEncoded && pic->isNeededForOutput )
        pic->isNeededForOutput = false;
    }
  }
}

void EncGOP::xReleasePictures( const PicList& picList, PicList& freeList )
{
  const bool allPicsDone = m_numPicsCoded >= m_picCount && ( picList.empty() || picList.back()->isFlush );
  // Writing a picture reads its reconstruction only for the decoded picture hash SEI. Without it, a
  // picture that no other picture references and that is not waiting for reconstruction output needs
  // neither its reconstruction nor its motion field once it is reconstructed.
  const bool writeReadsReco = m_pcEncCfg->m_decodedPictureHashSEIType != VVENC_HASHTYPE_NONE;
  // Once encoded, a picture of the final encoder (the last stage; never the pre-analysis encoder, whose pictures the
  // final pass reads again) no longer reads its source: finalizePicture computes PSNR before isEncoded is set (while
  // isReconstructed is set before it, see xEncodePicture). It may then lend the source buffer
  // to new input (EncLib::xGetFreePicShared). Its temporally filtered copy and its links to the two previous source
  // pictures (QPA, read only while it is coded) are released here instead of at output, so those pictures can lend their
  // source buffers sooner. Not with rate control or rate cap, whose updates after reconstruction are kept as they are,
  // and not before the picture's film grain analysis has read its source.
  const bool lendSources = ! m_isPreAnalysis && m_pcEncCfg->m_RCTargetBitrate == 0 && ! m_pcEncCfg->m_rateCap;
  for( auto pic : picList )
  {
    if( lendSources && pic->isEncoded && pic->m_picShared && ! pic->m_picShared->isSourceDone()
        && ! ( m_pcEncCfg->m_fga && xFgaReadsSource( *pic ) ) )
    {
      pic->m_picShared->setSourceDone();
      pic->getFilteredOrigBuffer().destroy();
      pic->m_picShared->releasePrevBuffers( pic );
    }
    const bool isUnused = ! pic->isNeededForOutput && ! pic->isReferenced && pic->refCounter <= 0;
    if( ( pic->isFinished && isUnused ) || allPicsDone )
    {
      // Nothing reads a released picture any more. Its buffers are allocated again when the
      // picture object is reused and its next picture starts encoding.
      pic->releaseEncodingBuffers();
      if( pic->m_picShared )
      {
        // a later picture's QPA reference may still hold the source: it must not be lent any more
        pic->m_picShared->clearSourceDone();
      }
      freeList.push_back( pic );
    }
    else if( pic->isEncoded && isUnused && ! writeReadsReco )
    {
      pic->releaseEncodingBuffers();
    }
  }
}

void EncGOP::printOutSummary( const bool printMSEBasedSNR, const bool printSequenceMSE, const bool printHexPsnr )
{
  //--CFG_KDY
  //const int rateMultiplier = 1;
  double fps = m_pcEncCfg->m_FrameRate/(double)m_pcEncCfg->m_FrameScale;
  m_AnalyzeAll.setFrmRate( fps );
  m_AnalyzeI.setFrmRate( fps );
  m_AnalyzeP.setFrmRate( fps );
  m_AnalyzeB.setFrmRate( fps );

  const ChromaFormat chFmt = m_pcEncCfg->m_internChromaFormat;

  const BitDepths& bitDepths = m_spsMap.getFirstPS()->bitDepths;
  //-- all
  std::string summary( "\n" );
  if( m_pcEncCfg->m_verbosity >= VVENC_DETAILS )
    summary.append("\nvvenc [info]: SUMMARY --------------------------------------------------------\n");

  summary.append( m_AnalyzeAll.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths));

  if( m_pcEncCfg->m_verbosity < VVENC_DETAILS )
  {
    msg.log( VVENC_INFO,summary.c_str() );
  }
  else
  {
    summary.append( "\n\nvvenc [info]: I Slices--------------------------------------------------------\n" );
    summary.append( m_AnalyzeI.printOut('i', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths));

    summary.append( "\n\nvvenc [info]: P Slices--------------------------------------------------------\n" );
    summary.append( m_AnalyzeP.printOut('p', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths));

    summary.append( "\n\nvvenc [info]: B Slices--------------------------------------------------------\n" );
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
  ci.noBcwConstraintFlag                          = m_pcEncCfg->m_BCW == 0;
  ci.noIbcConstraintFlag                          = m_pcEncCfg->m_IBCMode == 0;
  ci.noCiipConstraintFlag                         = m_pcEncCfg->m_CIIP == 0;
  ci.noGeoConstraintFlag                          = m_pcEncCfg->m_Geo == 0;
  ci.noLadfConstraintFlag                         = true;
  ci.noTransformSkipConstraintFlag                = m_pcEncCfg->m_TS == 0;
  ci.noBDPCMConstraintFlag                        = m_pcEncCfg->m_useBDPCM==0;
  ci.noJointCbCrConstraintFlag                    = ! m_pcEncCfg->m_JointCbCrMode;
  ci.noMrlConstraintFlag                          = ! m_pcEncCfg->m_MRL;
  ci.noIspConstraintFlag                          = m_pcEncCfg->m_ISP == 0;
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
  ci.noLmcsConstraintFlag                         = true;
  ci.noTrailConstraintFlag                        = m_pcEncCfg->m_IntraPeriod == 1;
  ci.noStsaConstraintFlag                         = m_pcEncCfg->m_IntraPeriod == 1 || ! m_gopCfg->hasNonZeroTemporalId();
  ci.noRaslConstraintFlag                         = m_pcEncCfg->m_IntraPeriod == 1 || ! m_gopCfg->hasLeadingPictures();
  ci.noRadlConstraintFlag                         = m_pcEncCfg->m_IntraPeriod == 1 || ! m_gopCfg->hasLeadingPictures();
  ci.noIdrConstraintFlag                          = false;
  ci.noCraConstraintFlag                          = (m_pcEncCfg->m_DecodingRefreshType != VVENC_DRT_CRA && m_pcEncCfg->m_DecodingRefreshType != VVENC_DRT_CRA_CRE);
  ci.noGdrConstraintFlag                          = false;
  ci.noApsConstraintFlag                          = !m_pcEncCfg->m_alf;
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

  if( m_pcEncCfg->m_maxPicWidth != 0 && m_pcEncCfg->m_maxPicHeight != 0 )
  {
    const int minCuSize = std::max( 1 << ( vvenc::MIN_CU_LOG2 + 1 ), 1 << m_pcEncCfg->m_log2MinCodingBlockSize );
    int padRight = 0, padBottom = 0;
    if( m_pcEncCfg->m_maxPicWidth % minCuSize )
    {
      padRight = ( ( m_pcEncCfg->m_maxPicWidth / minCuSize) + 1 ) * minCuSize - m_pcEncCfg->m_maxPicWidth;
    }
    if( m_pcEncCfg->m_maxPicHeight % minCuSize )
    {
      padBottom = ( ( m_pcEncCfg->m_maxPicHeight / minCuSize) + 1 ) * minCuSize - m_pcEncCfg->m_maxPicHeight;
    }
    sps.maxPicWidthInLumaSamples      = m_pcEncCfg->m_maxPicWidth + padRight;
    sps.maxPicHeightInLumaSamples     = m_pcEncCfg->m_maxPicHeight + padBottom;
    
    sps.conformanceWindow.setWindow( 0, padRight, 0, padBottom );
  }
  else
  {
    sps.maxPicWidthInLumaSamples      = m_pcEncCfg->m_PadSourceWidth;
    sps.maxPicHeightInLumaSamples     = m_pcEncCfg->m_PadSourceHeight;
    sps.conformanceWindow.setWindow( m_pcEncCfg->m_confWinLeft, m_pcEncCfg->m_confWinRight, m_pcEncCfg->m_confWinTop, m_pcEncCfg->m_confWinBottom );
  }
  sps.chromaFormatIdc               = m_pcEncCfg->m_internChromaFormat;
  sps.CTUSize                       = m_pcEncCfg->m_CTUSize;
  sps.maxMTTDepth[0]                = m_pcEncCfg->m_maxMTTDepthI;
  int maxMTTDepthVal = m_pcEncCfg->m_maxMTTDepth;
  int minMaxMttD = maxMTTDepthVal % 10;
  while( maxMTTDepthVal )
  {
    minMaxMttD      = std::min( minMaxMttD, maxMTTDepthVal % 10 );
    maxMTTDepthVal /= 10;
  }
  sps.maxMTTDepth[1]                = minMaxMttD;
  sps.maxMTTDepth[2]                = m_pcEncCfg->m_maxMTTDepthIChroma;
  for( int i = 0; i < 3; i++)
  {
    sps.minQTSize[i]                = m_pcEncCfg->m_MinQT[i];
    sps.maxBTSize[i]                = m_pcEncCfg->m_maxBT[i];
    sps.maxTTSize[i]                = m_pcEncCfg->m_maxTT[i];
  }
  sps.minQTSize[2]                <<= getChannelTypeScaleX(CH_C, m_pcEncCfg->m_internChromaFormat);

  sps.maxNumMergeCand               = m_pcEncCfg->m_maxNumMergeCand;
  sps.maxNumAffineMergeCand         = !!m_pcEncCfg->m_Affine ? m_pcEncCfg->m_maxNumAffineMergeCand : 0;
  sps.maxNumGeoCand                 = !!m_pcEncCfg->m_Geo    ? m_pcEncCfg->m_maxNumGeoCand : 0;
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
  sps.ccalfEnabled                  = m_pcEncCfg->m_ccalf && sps.alfEnabled && m_pcEncCfg->m_internChromaFormat != VVENC_CHROMA_400;

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
    vui.aspectRatioConstantFlag       = true; // false if SampleAspectRatioInfoSEIEnabled, but this SEI is not used
    vui.aspectRatioIdc                = m_pcEncCfg->m_aspectRatioIdc;
    vui.sarWidth                      = m_pcEncCfg->m_sarWidth;
    vui.sarHeight                     = m_pcEncCfg->m_sarHeight;
    vui.colourDescriptionPresent      = m_pcEncCfg->m_colourDescriptionPresent;
    vui.colourPrimaries               = m_pcEncCfg->m_colourPrimaries;
    vui.transferCharacteristics       = m_pcEncCfg->m_transferCharacteristics;
    vui.matrixCoefficients            = m_pcEncCfg->m_matrixCoefficients;
    vui.chromaLocInfoPresent          = m_pcEncCfg->m_chromaLocInfoPresent;
    vui.chromaSampleLocType           = m_pcEncCfg->m_chromaSampleLocType;
    vui.chromaSampleLocTypeTopField   = 0;
    vui.chromaSampleLocTypeBottomField= 0;
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
  bUseDQP |= m_pcEncCfg->m_lumaLevelToDeltaQPEnabled == 1;
  bUseDQP |= m_pcEncCfg->m_usePerceptQPA;
  bUseDQP |= m_pcEncCfg->hasBlockQpOffsets();

  if (m_pcEncCfg->m_costMode==VVENC_COST_SEQUENCE_LEVEL_LOSSLESS || m_pcEncCfg->m_costMode==VVENC_COST_LOSSLESS_CODING)
  {
    bUseDQP = false;
  }
  if( m_pcEncCfg->m_maxDeltaQP == 0 )
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
  pps.useDQP                        = bUseDQP;

  if ( m_pcEncCfg->m_cuChromaQpOffsetSubdiv >= 0 )
  {
    //th check how this is configured now    pps.cuChromaQpOffsetSubdiv = m_pcEncCfg->m_cuChromaQpOffsetSubdiv;
    pps.chromaQpOffsetListLen = 0;
    pps.setChromaQpOffsetListEntry(1, 6, 6, 6);
  }

  // fix PPS init QP to 26 or 32 (depending on BD) to make concatenating bitstreams more robust
  pps.picInitQPMinus26 = 6 - sps.qpBDOffset[CH_L] / 2;

  pps.chromaQpOffset[COMP_Y]          = 0;
  pps.chromaQpOffset[COMP_Cb]         = m_pcEncCfg->m_chromaCbQpOffset;
  pps.chromaQpOffset[COMP_Cr]         = m_pcEncCfg->m_chromaCrQpOffset;
  pps.chromaQpOffset[COMP_JOINT_CbCr] = m_pcEncCfg->m_chromaCbCrQpOffset;

  bool bChromaDeltaQPEnabled = false;
  {
    bChromaDeltaQPEnabled = ( m_pcEncCfg->m_sliceChromaQpOffsetIntraOrPeriodic[ 0 ] || m_pcEncCfg->m_sliceChromaQpOffsetIntraOrPeriodic[ 1 ] );
    bChromaDeltaQPEnabled |= (m_pcEncCfg->m_usePerceptQPA || (m_pcEncCfg->m_LookAhead && m_pcRateCtrl->m_pcEncCfg->m_RCTargetBitrate > 0) || m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity > 0) && (m_pcEncCfg->m_internChromaFormat != VVENC_CHROMA_400);
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

  pps.pcv            = new PreCalcValues( sps, pps, m_pcEncCfg->m_MaxQT );
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

vvencNalUnitType EncGOP::xGetNalUnitType( const GOPEntry* _gopEntry ) const
{
  const GOPEntry& gopEntry = *_gopEntry;

  if( gopEntry.m_POC == 0 && m_pcEncCfg->m_poc0idr )
  {
    return VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP;
  }

  if( gopEntry.m_isStartOfIntra )
  {
    if( m_pcEncCfg->m_DecodingRefreshType == VVENC_DRT_CRA || m_pcEncCfg->m_DecodingRefreshType == VVENC_DRT_CRA_CRE )
    {
      if( m_lastIDR == 0 && !m_pcEncCfg->m_poc0idr )
      {
        return VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL;
      }
      else
      {
        return VVENC_NAL_UNIT_CODED_SLICE_CRA;
      }
    }
    else if( m_pcEncCfg->m_DecodingRefreshType == VVENC_DRT_IDR_NO_RADL )
    {
      return VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP;
    }
    else
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

  if( m_lastIDR > 0 && gopEntry.m_POC < m_lastIDR && m_pcEncCfg->m_DecodingRefreshType != VVENC_DRT_IDR_NO_RADL )
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

void EncGOP::xSetupPicAps( Picture* pic )
{
  // manage global APS list
  m_globalApsList.push_back( new PicApsGlobal( pic->poc, pic->TLayer ) );
  CHECK( pic->picApsGlobal != nullptr, "Top level APS ptr must be nullptr" );

  // The list is kept near this size; the +2 covers up to two GOPs of processing delay (Threads=1 mode). The pictures in
  // flight are not bounded by this size (GOP read-ahead, look-ahead stages), so an entry is deleted only after its last
  // use: no picture set up later references it, and its own picture has filled it (initalized: finalizeReference wrote
  // it before, or with IFP the copy at the ALF derivation; nothing writes it after). (Pictures skipped in a first pass
  // never finish, but they have no entry: the first pass runs without ALF.)
  const size_t maxListSize = std::max( (int)MAX_NUM_APS, m_pcEncCfg->m_GOPSize ) * ( m_pcEncCfg->m_maxParallelFrames + 2 );
  if( m_globalApsList.size() > maxListSize )
  {
    // the finish task (or, with IFP, the APS copy at the ALF derivation) sets initalized under this mutex
    std::unique_lock<std::mutex> lock( m_gopEncMutex, std::defer_lock );
    if( m_pcEncCfg->m_numThreads > 0 ) lock.lock();
    while( m_globalApsList.size() > maxListSize && m_globalApsList.front()->refCnt == 0 && m_globalApsList.front()->initalized )
    {
      delete m_globalApsList.front();
      m_globalApsList.pop_front();
    }
  }

  pic->picApsGlobal = m_globalApsList.back();

  // determine reference APS
  const bool mtPicParallel = m_pcEncCfg->m_numThreads > 0;
  if( mtPicParallel && pic->slices[0]->isIntra() )
  {
    // reset APS propagation on Intra-Slice in MT-mode
    return;
  }

  // get previous APS (in coding order) to propagate from it
  // in parallelization case (parallel pictures), we refer to APS from lower temporal layer
  // NOTE: elements in the global APS list are following in coding order

  PicApsGlobal* refAps = nullptr;
  auto curApsItr = std::find_if( m_globalApsList.begin(), m_globalApsList.end(), [pic]( auto p ) { return p->poc == pic->poc; } );
  CHECK( curApsItr == m_globalApsList.end(), "Should not happen" );

  if( curApsItr != m_globalApsList.begin() )
  {
    if( mtPicParallel )
    {
      auto r_begin = std::reverse_iterator<std::deque<PicApsGlobal*>::iterator>(curApsItr);
      auto r_end   = std::reverse_iterator<std::deque<PicApsGlobal*>::iterator>(m_globalApsList.begin());
      auto refApsItr = ( pic->TLayer > 0 ) ? std::find_if( r_begin, r_end, [pic]( auto p ) { return p->tid  < pic->TLayer; } ):
                                             std::find_if( r_begin, r_end, [pic]( auto p ) { return p->tid == pic->TLayer; } );
      if( refApsItr == r_end )
        return;
      refAps = *refApsItr;
    }
    else
    {
      curApsItr--;
      refAps = *curApsItr;
    }
    if( refAps )
      refAps->refCnt++;
  }

  //CHECK( !refAps, "Faied to get reference APS" );
  pic->refApsGlobal = refAps;
}

// Pictures come here in coding order, but the analyses run in POC order: each adds to the statistics of the previous
// ones, and the parameter sets apply in output order. So at a picture, every picture that MCTF prepared for the
// analysis (the pictures up to the temporal layer FgaMaxTempDepth, see GOPCfg::xSetFgaLayer) after the last analysed
// one, up to this picture's POC, is analysed, in POC order. They are in picList then, as the pictures reach this stage
// in POC order (with the look-ahead, which passes them on in coding order, only GOP starts are analysed: the
// configuration check allows no other depth than 0 there).
// Their results are taken with this picture's access unit, the first of them written. Analyzing them here rather than
// when their encoding starts keeps the choice of analyzed pictures, and the parameters each access unit signals,
// independent of the parallel encoding order.
void EncGOP::xAnalyzeFilmGrain( const Picture& current, const PicList& picList )
{
  std::vector<Picture*> pics;
  for( Picture* pic : picList )
  {
    const bool prepared = pic->m_picShared->getFgaFilteredBuf().valid() || pic->m_picShared->fgaUsesFilteredBuf();
    if( prepared && pic->poc > m_fgaPrevAnalysedPoc && pic->poc <= current.poc )
    {
      pics.push_back( pic );
    }
  }
  std::sort( pics.begin(), pics.end(), []( const Picture* a, const Picture* b ) { return a->poc < b->poc; } );
  for( Picture* pic : pics )
  {
    xIssueFgaTask( *pic, current.gopEntry->m_codingNum );
  }
}

void EncGOP::xIssueFgaTask( Picture& pic, int codingNum )
{
  CHECK( pic.poc <= m_fgaPrevAnalysedPoc, "film grain analyses must run in POC order" );
  m_fgaTasks.emplace_back( new FgaTask );
  FgaTask* task  = m_fgaTasks.back().get();
  task->gop        = this;
  task->pic        = &pic;
  task->codingNum  = codingNum;

  // The result applies from the picture after the previously analysed one; when PreProcess found a scene cut in
  // between, from the cut, so that the pictures before it keep the parameters of their scene. The first analysis
  // applies to all pictures before it as well. Pictures already written keep theirs (xAddFilmGrainParameters). The
  // analysis of the picture after a shot start restarts the shot's statistics without the start's one-sided
  // analysis (MCTF::filter), from this picture on.
  const bool first  = m_fgaPrevAnalysedPoc == std::numeric_limits<int>::min();
  const int  cutPoc = pic.m_picShared->fgaSceneCut() ? pic.poc : pic.m_picShared->fgaLastSceneCutPoc();
  const bool newCut = cutPoc > m_fgaPrevAnalysedPoc;
  task->sceneCut    = !first && ( newCut || pic.m_picShared->fgaShotRestart() );
  task->startPoc    = first ? std::numeric_limits<int>::min() : newCut ? cutPoc : m_fgaPrevAnalysedPoc + 1;
  m_fgaPrevAnalysedPoc = pic.poc;
  const Barrier* previous = m_fgaLast ? &m_fgaLast->counter.done : nullptr;
  m_fgaLast = task;

  static auto runTask = []( int, void* param )
  {
    FgaTask* t = static_cast<FgaTask*>( param );
    t->gop->xRunFgaTask( *t );
    return true;
  };
  if( m_threadPool )
  {
    // off the encoding path: the result is only needed when the access unit of the picture is written
    m_threadPool->addBarrierTask( runTask, task, &task->counter, nullptr, { previous } );
  }
  else
  {
    runTask( 0, task );
  }
}

void EncGOP::xRunFgaTask( FgaTask& task )
{
  PicShared* shared         = task.pic->m_picShared;
  PelStorage& fgaBuf        = shared->getFgaFilteredBuf();
  PelStorage& residualBuf   = shared->getFgaResidualBuf();
  const int residualWeight  = shared->fgaResidualWeight();
  const bool ownBuf         = fgaBuf.valid();
  const CPelUnitBuf denoised = ownBuf ? CPelUnitBuf( fgaBuf ) : CPelUnitBuf( task.pic->getFilteredOrigBuffer() );
  if( residualWeight )
  {
    const CPelUnitBuf residual( residualBuf );
    m_fgAnalyzer.estimateGrainParameters( task.pic->getOrigBuffer(), denoised, &residual, residualWeight, task.sceneCut );
  }
  else
  {
    m_fgAnalyzer.estimateGrainParameters( task.pic->getOrigBuffer(), denoised, nullptr, 0, task.sceneCut );
  }
  residualBuf.destroy();
  if( ownBuf )
  {
    fgaBuf.destroy();
  }
  task.result = FilmGrainParam( m_fgAnalyzer, m_pcEncCfg->m_fgaStrength );
}

// Whether a film grain analysis of the picture may still read its source.
bool EncGOP::xFgaReadsSource( const Picture& pic ) const
{
  for( const auto& task : m_fgaTasks )
  {
    if( task->pic == &pic && task->counter.isBlocked() )
    {
      return true;
    }
  }
  return false;
}

void EncGOP::xWaitFgaTasks()
{
  for( auto& task : m_fgaTasks )
  {
    task->counter.wait();
  }
}

void EncGOP::xInitPicsInCodingOrder( const PicList& picList )
{
  CHECK( m_pcEncCfg->m_maxParallelFrames <= 0 && m_gopEncListInput.size() > 0,  "no frame parallel processing enabled, but multiple pics in flight" );
  CHECK( m_pcEncCfg->m_maxParallelFrames <= 0 && m_gopEncListOutput.size() > 0, "no frame parallel processing enabled, but multiple pics in flight" );

  // loop over pic list, which is sorted in coding number order 
  for( auto it = picList.begin(); it != picList.end(); ++it )
  {
    auto pic = (*it);
    // skip pics, which have already been initialized
    if( pic->isInitDone )
      continue;

    // update visual activity for last start of GOP picture
    // this may have been changed in the shared picture data due to fixStartOfLastGop()
    if( pic->gopEntry->m_isStartOfGop && picList.back()->isFlush )
    {
      xUpdateVAStartOfLastGop( *pic );
    }

    // GOP QP adjustments
    if( (m_pcEncCfg->m_rateCap || m_pcEncCfg->m_GOPQPA || m_pcEncCfg->m_usePerceptQPA) && pic->gopEntry->m_isStartOfGop )
    {
      // note: in case of rate cap, wait until the complete GOP is in the list and update-list is empty
      if( !m_pcEncCfg->m_rateCap ||
        ((pic->gopEntry->m_gopNum != picList.back()->gopEntry->m_gopNum || picList.back()->isFlush) && m_rcUpdateList.empty()) )
      {
        xInitGopQpCascade( *pic, it, picList );
      }
      else
      {
        // rate cap: wait until the condition is met
        break;
      }
    }

    // continue with next pic in increasing coding number order
    if( pic->gopEntry->m_codingNum != m_lastCodingNum + 1 && ! picList.back()->isFlush )
      break;

    CHECK( m_lastCodingNum == -1 && ! pic->gopEntry->m_isStartOfIntra, "encoding should start with an I-Slice" );

    xForceScc( *pic );

    // initialize slice header
    pic->encTime.startTimer();
    xInitFirstSlice( *pic, picList, false );
    pic->encTime.stopTimer();

    // pictures ready for encoding
    m_gopEncListInput.push_back( pic );
    m_gopEncListOutput.push_back( pic );

    if( m_pcEncCfg->m_alf && m_pcEncCfg->m_alfTempPred )
    {
        xSetupPicAps( pic );
    }

    if( m_pcEncCfg->m_fga && !m_isPreAnalysis && m_pcRateCtrl->rcIsFinalPass )
    {
      xAnalyzeFilmGrain( *pic, picList );
    }

    // continue with next picture
    m_lastCodingNum = pic->gopEntry->m_codingNum;

    // in single threading initialize only one picture per encoding loop
    if( m_pcEncCfg->m_maxParallelFrames <= 0 )
      break;
  }

  CHECK( !(m_pcEncCfg->m_rateCap || m_pcEncCfg->m_GOPQPA || m_pcEncCfg->m_usePerceptQPA) && picList.size() && m_pcEncCfg->m_maxParallelFrames <= 0 && m_gopEncListInput.size() != 1,  "no new picture for encoding found" );
  CHECK( !(m_pcEncCfg->m_rateCap || m_pcEncCfg->m_GOPQPA || m_pcEncCfg->m_usePerceptQPA) && picList.size() && m_pcEncCfg->m_maxParallelFrames <= 0 && m_gopEncListOutput.size() != 1, "no new picture for encoding found" );
}

void EncGOP::xUpdateRcIfp()
{
  // deterministic behavior: RC update on next finished frame in sliding window coding order,
  //                         evaluate only one finished frame at front of the list that makes place for the next frame
  //                         whose parameters can be set using the finished frame bits info
  //
  // non-deterministic behavior: RC update on any finished frame

#if IFP_RC_DETERMINISTIC
  if( m_rcUpdateList.front()->isEncoded && m_rcUpdateList.back()->encRCPic && ( m_rcUpdateList.front()->isFlush || m_rcUpdateList.size() == m_pcEncCfg->m_maxParallelFrames ) )
  {   
#endif
    for( auto it = m_rcUpdateList.begin(); it != m_rcUpdateList.end(); )
    {
      auto pic = *it;
      if( pic->isEncoded )
      {
        pic->actualTotalBits = pic->sliceDataStreams[0].getNumberOfWrittenBits();
        pic->refCounter--;
        m_pcRateCtrl->updateAfterPicEncRC( pic );
        it = m_rcUpdateList.erase( it );
      }
      else
      {
        ++it;
      }
#if IFP_RC_DETERMINISTIC
      // in deterministic case, only one frame is allowed to update the RC
      break;
#endif
    }
#if IFP_RC_DETERMINISTIC
  }
#endif
}

inline bool getReorderedProcList( std::list<Picture*>& inputList, std::list<Picture*>& procList, const int maxSize, bool isIFP, bool restrictToGOP )
{
  // deliver frames of the same TID (temporal layer) and from the same GOP
  const int procTL = inputList.size() ? inputList.front()->TLayer             : -1;
  const int gopNum = inputList.size() ? inputList.front()->gopEntry->m_gopNum : -1;
  bool added = false;
  for( auto it = inputList.begin(); it != inputList.end(); )
  {
    auto pic = *it;
    if( ( pic->gopEntry->m_gopNum == gopNum || !restrictToGOP )
        && pic->TLayer == procTL
        && ( isIFP ? pic->slices[ 0 ]->checkAllRefPicsAccessible(): pic->slices[ 0 ]->checkAllRefPicsReconstructed() ) )
    {
      pic->isInProcessList = true;
      procList.push_back  ( pic );
      it = inputList.erase( it );
      added = true;
    }
    else
    {
      ++it;
    }
    if( (int)procList.size() >= maxSize )
      break;
  }
  return added;
}

inline void getProcListForOneGOP( std::list<Picture*>& inputList, std::list<Picture*>& procList )
{
  // provide frames of the same GOP
  const int gopNum = inputList.size() ? inputList.front()->gopEntry->m_gopNum : -1;
  for( auto it = inputList.begin(); it != inputList.end(); )
  {
    auto pic = *it;
    if( pic->gopEntry->m_gopNum == gopNum )
    {
      procList.push_back  ( pic );
      it = inputList.erase( it );
    }
    else
    {
      ++it;
    }
  }
}

void EncGOP::xGetProcessingLists( std::list<Picture*>& procList, std::list<Picture*>& rcUpdateList, const bool lockStepMode )
{
  // in lockstep mode, frames are reordered in a specific processing order
  if( lockStepMode )
  {
    if( m_pcEncCfg->m_ifpLines )
    {
      // prepare reordered list
      // we need an additional reordering list to ensure causality of the coding order (ref.pics) on irregular GOP structures
      // in the first step, the reordered list is filled
      // in the second, the frames from reordered list are moved to proc. list up to required update-list size
      const int maxUpdateListSize = m_pcEncCfg->m_maxParallelFrames;
      if( rcUpdateList.size() < maxUpdateListSize && ( !m_gopEncListInput.empty() || !m_rcInputReorderList.empty()))
      {
        while( rcUpdateList.size() < maxUpdateListSize && ( !m_gopEncListInput.empty() || !m_rcInputReorderList.empty()) )
        {
          if( !m_rcInputReorderList.empty() )
          {
            auto pic = m_rcInputReorderList.front();
            m_rcInputReorderList.pop_front();
            pic->refCounter++;
            procList.push_back( pic );
            rcUpdateList.push_back( pic );
          }
          else
          {
            while( m_rcInputReorderList.size() < maxUpdateListSize && !m_gopEncListInput.empty() )
            {
              getReorderedProcList( m_gopEncListInput, m_rcInputReorderList, maxUpdateListSize, true, true );
            }
          }
        }
      }
    }
    else if( rcUpdateList.empty() )
    {
      // retrieve next lockstep chunk from reordered list
      const int procTL         = m_gopEncListInput.size() ? m_gopEncListInput.front()->TLayer : -1;
      const int minSerialDepth = m_pcEncCfg->m_maxParallelFrames > 2 ? 1 : 2;  // up to this temporal layer encode pictures only in serial mode
      const int maxSize        = procTL <= minSerialDepth ? 1 : m_pcEncCfg->m_maxParallelFrames;
      getReorderedProcList( m_gopEncListInput, procList, maxSize, false, true );
      std::copy( procList.begin(), procList.end(), std::back_inserter(rcUpdateList) );
    }
  }
  else
  {
    // regular coding mode (non-RC); the processing list may still hold pictures (fetched on every pass)
    const size_t numBefore = procList.size();
    // in case of IFP, using the reordered list brings an additional speedup
    if( m_pcEncCfg->m_ifpLines )
    {
      const size_t inputListSize = m_gopEncListInput.size();

      // in case of GOP parallel processing, we do not put all the frames from the current GOP in proc.list, but top it
      // up to MaxParallelFrames pictures on every pass: we want to add frames from the next GOP as soon as possible.
      const size_t targetProcListSize = m_pcEncCfg->m_numParallelGOPs ? std::max<size_t>( numBefore, m_pcEncCfg->m_maxParallelFrames )
                                                                      : numBefore + inputListSize;

      while( !m_gopEncListInput.empty() && procList.size() < targetProcListSize )
      {
        if( !getReorderedProcList( m_gopEncListInput, procList, (int)procList.size() + m_pcEncCfg->m_maxParallelFrames, true, !m_pcEncCfg->m_numParallelGOPs ) )
          break;
      }
      // nothing to process and nothing added: no picture can ever start (pictures still in the list may just have to
      // start first, so that their dependents become accessible)
      if( m_gopEncListInput.size() == inputListSize && numBefore == 0 )
        msg.log( VVENC_WARNING, "Processing list derivation: attempting to run in a deadlock" );
    }
    else
    {
      if( m_pcEncCfg->m_rateCap ) // TODO helmrich: what about || *->m_GOPQPA?
      {
        // ensure that procList contains only pictures from one GOP
        getProcListForOneGOP( m_gopEncListInput, procList );
      }
      else
      {
        // just pass the input list to processing list
        procList.splice( procList.end(), m_gopEncListInput );
        m_gopEncListInput.clear();
      }
    }
    if( m_pcEncCfg->m_RCTargetBitrate > 0 || m_pcEncCfg->m_rateCap )
    {
      // update-list is read only by RC and RateCapping here; with IFP alone its entries, taken in processing order but
      // released in output order, stop being removed after the first pictures and the list grows for the whole encode.
      // It takes the newly added pictures.
      std::copy( std::next( procList.begin(), static_cast<std::ptrdiff_t>( numBefore ) ), procList.end(), std::back_inserter( rcUpdateList ) );
    }
  }
  CHECK( ! rcUpdateList.empty() && m_gopEncListOutput.empty(), "first picture in RC update and in output list have to be the same" );
}

void EncGOP::xUpdateRateCap()
{
  for( auto it = m_rcUpdateList.begin(); it != m_rcUpdateList.end(); )
  {
    auto pic = *it;
    if( pic->isEncoded )
    {
      const unsigned uibits = pic->sliceDataStreams[0].getNumberOfWrittenBits();

      if( !pic->gopEntry->m_isStartOfIntra && pic->gopEntry->m_scType == SCT_NONE )
      {
        xUpdateRateCapBits( pic, uibits );
      }
      else if( pic->gopEntry->m_isStartOfIntra && pic->gopEntry->m_gopNum == 0 && pic->poc < m_pcEncCfg->m_GOPSize && m_rcap.accumTargetBits * (uint32_t) m_pcEncCfg->m_GOPSize < uibits )
      {
        m_rcap.accumActualBits += uibits - m_rcap.accumTargetBits * (uint32_t) m_pcEncCfg->m_GOPSize; // capped CQF: compensate for overspending in first I-frame
      }

      it = m_rcUpdateList.erase( it );
    }
    else
    {
      ++it;
    }
  }
}

void EncGOP::xUpdateRateCapBits( const Picture* pic, const uint32_t uibits )
{
  // try to adjust the rate for the first GOP on the scene-cut (or start of the sequence)
  if( pic->gopEntry->m_isStartOfGop )
  {
    m_rcap.gopAdaptedQPAdj = 0;
  }
  else if( pic->isSceneCutCheckAdjQP )
  {
    CHECK( uibits == 0 || m_rcap.accumTargetBits == 0, "Not expected" );
    const double f = std::min (1.5, pow (uibits / double (3u * m_rcap.accumTargetBits), 0.25));
    if( f > 1.0 )
    {
      const Slice* slice = pic->slices[0];
      const double d = (105.0 / 128.0) * sqrt( (double)std::max( 1, slice->sliceQp ) ) * log( f ) / log( 2.0 );
      m_rcap.gopAdaptedQPAdj = int(d + 0.5);
      m_rcap.nonRateCapEstim = f;
      //uibits = 3u * m_rcap.AccumTargetBits; // can be used to avoid overweighting of TL1 picture
    }
  }
  m_rcap.accumActualBits += unsigned (0.5 + uibits * m_rcap.nonRateCapEstim);
}

void EncGOP::xUpdateVAStartOfLastGop( Picture& keyPic ) const
{
  keyPic.picVA = keyPic.m_picShared->m_picVA;
}

void EncGOP::xInitGopQpCascade( Picture& keyPic, PicList::const_iterator picListBegin, const PicList& picList )
{
  CHECK( !keyPic.gopEntry->m_isStartOfGop, "Expecting key picture as start of GOP")
  uint32_t gopMotEstCount = 0, gopMotEstError = 0;
  const double resRatio4K = double (m_pcEncCfg->m_SourceWidth * m_pcEncCfg->m_SourceHeight) / (3840.0 * 2160.0);
  const bool isHighRes    = (std::min (m_pcEncCfg->m_SourceWidth, m_pcEncCfg->m_SourceHeight) > 1280);
  const int gopNum        = keyPic.gopEntry->m_gopNum;
  const bool keyPicIsIdrNLP      = xGetNalUnitType(keyPic.gopEntry) == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP;
  PicList::const_iterator picItr = picListBegin;
  const bool nextKeyPicAfterIDR  = keyPicIsIdrNLP && (++picItr != picList.end()) && (*picItr)->gopEntry->m_isStartOfGop;

  int dQP = 0;
  double qpStart = 24.0;
  unsigned num = 0, sum = 0;
  unsigned nSC = 0, sSC = 0;
  uint8_t gopMinNoiseLevels[QPA_MAX_NOISE_LEVELS];

  std::fill_n (gopMinNoiseLevels, QPA_MAX_NOISE_LEVELS, 255u);

  // get spatial activity of current and previous TL0 pic
  int spVisActTL0[2] = { 0, 0 };
  for( auto ch : { CH_L, CH_C } )
  {
    const int count = ( keyPic.picVA.spatAct[ ch ] > 0 && keyPic.picVA.prevTL0spatAct[ ch ] > 0 ) ? 2 : 1;
    spVisActTL0[ch] = ( keyPic.picVA.spatAct[ ch ] + keyPic.picVA.prevTL0spatAct[ ch ] + ( count >> 1 ) ) / count;
  }

  for (auto picItr = picListBegin; picItr != picList.end(); ++picItr)
  {
    auto pic = (*picItr);
    if( pic->gopEntry->m_gopNum == gopNum )
    {
      if( pic->m_picShared->m_picMotEstError > 0 )
      {
        CHECK( pic->isInitDone, "try to modify GOP qp of picture, which has already been initialized" );
        // summarize motion errors of all MCTF filtered pictures in GOP
        gopMotEstCount++;
        gopMotEstError += pic->m_picShared->m_picMotEstError;
        // go through ranges, search per-range minimum in GOP
        for (int i = 0; i < QPA_MAX_NOISE_LEVELS; i++)
        {
          gopMinNoiseLevels[i] = std::min<uint8_t> (gopMinNoiseLevels[i], pic->m_picShared->m_minNoiseLevels[i]);
        }
      }
      nSC++;
      sSC += (pic->isSccStrong ? 1 : 0) + (pic->isSccWeak ? 1 : 0);

      if( pic == &keyPic && nextKeyPicAfterIDR ) // consider a virtual GOP containing only one IDR pic
        break;
    }
  }

  gopMotEstError = (gopMotEstError + (gopMotEstCount >> 1)) / std::max (1u, gopMotEstCount);

  for (int i = 0; i < QPA_MAX_NOISE_LEVELS; i++) // go through ranges again, find overall min-average in GOP
  {
    if (gopMinNoiseLevels[i] < 255)
    {
      num++;
      sum += gopMinNoiseLevels[i];
    }
  }

  // force 2nd-order filter
  const bool f2O = (m_pcEncCfg->m_usePerceptQPA) && (sSC >= (nSC >> 1)) && (sum < 18 * num); // low-noise SC
  
  // adapt GOP's QP offsets
  if (num > 0 && sum > 0)
  {
    qpStart += 0.5 * (6.0 * log ((double) sum / (double) num) / log (2.0) - 1.0 - 24.0); // see RateCtrl.cpp
    if (m_pcEncCfg->m_GOPQPA)
    {
      if (((qpStart > 29) && (spVisActTL0[CH_L] > 600)) ||
          ((qpStart > 27) && (spVisActTL0[CH_L] > 850)) || (spVisActTL0[CH_L] > 1300))
      {
        dQP += 1;
      }
      if ((qpStart < 24) && (spVisActTL0[CH_L] < 400))
      {
        dQP -= 1;
      }
    }
  }
  qpStart += log (resRatio4K) / log (2.0); // ICIP23 paper

  if (keyPic.gopEntry->m_scType == SCT_TL0_SCENE_CUT)
  {
    m_rcap.reset();
  }

  // derive rate capping parameters
  if (m_pcEncCfg->m_rateCap)
  {
    const int bDepth = m_pcEncCfg->m_internalBitDepth[CH_L];
    const int intraP = Clip3(m_pcEncCfg->m_GOPSize, 4 * VVENC_MAX_GOP, m_pcEncCfg->m_IntraPeriod);
    const int visAct = std::max(uint16_t(spVisActTL0[CH_L] >> (12 - bDepth)), keyPic.picVA.visAct); // when vaY=0
    const double apa = sqrt((m_pcEncCfg->m_internalUsePerceptQPATempFiltISlice ? 32.0 : 16.0) * double(1 << (2 * bDepth - 10)) / sqrt(resRatio4K)); // average picture activity
    const int auxOff = (m_pcEncCfg->hasBlockQpOffsets() && !keyPic.m_picShared->m_ctuBimQpOffset.empty() ? keyPic.m_picShared->m_picAuxQpOffset : 0) + dQP;
    const int iFrmQP = std::min(MAX_QP, m_pcEncCfg->m_QP + m_pcEncCfg->m_intraQPOffset + auxOff + int(floor(m_pcEncCfg->m_QPFraction + 3.0 * log(visAct / apa) / log(2.0) + 0.5)));
    const int qp32BC = int(16384.0 + 7.21875 * pow((double)spVisActTL0[CH_L], 4.0 / 3.0) + 1.46875 * pow((double)spVisActTL0[CH_C], 4.0 / 3.0)) * (isHighRes ? 96 : 24); // TODO hlm
    const int iFrmBC = int(0.5 + qp32BC * pow(2.0, (32.0 - iFrmQP) * 11.0 / 64.0) * pow(resRatio4K, 2.0 / 3.0)); // * HD tuning
    const int  shift = (gopMotEstError < 32 ? 5 - (gopMotEstError >> 4) : 3);
    if (keyPic.m_picShared->m_picMotEstError >= 256) gopMotEstError >>= 2; else // avoid 2 much capping at cuts
    if (gopMotEstError >= 120) /*TODO tune this*/ gopMotEstError >>= 1;
    const int bFrmBC = int((4.0 * iFrmBC * (intraP - 1)) / sqrt((double)std::max(spVisActTL0[CH_L], spVisActTL0[CH_C])) * std::max(int(gopMotEstError * gopMotEstError) >> (bDepth / 2), (keyPic.picVA.visActTL0 - visAct) >> shift) * pow(2.0, -1.0 * bDepth));
    const int meanGopSizeInIntraP = intraP / ((intraP + m_pcEncCfg->m_GOPSize - 1) / m_pcEncCfg->m_GOPSize);

    const double eps = 1.0 - 1.0 / double(1u << std::min(31u, m_rcap.accumGopCounter));
    const double nonKeyPicsFactor = (m_rcap.accumTargetBits == 0) ? 1.0 : pow((double)m_rcap.accumActualBits / ((meanGopSizeInIntraP - 1.0) * m_rcap.accumTargetBits), eps);
    const unsigned bFrmBC_final = bFrmBC * nonKeyPicsFactor;
    const unsigned targetBits = (unsigned)((bFrmBC + (intraP >> 1)) / (intraP - 1));
    m_rcap.accumTargetBits += targetBits;
    if (keyPic.gopEntry->m_isStartOfIntra && keyPic.gopEntry->m_gopNum == 0 && keyPic.poc < m_pcEncCfg->m_GOPSize && m_rcap.accumTargetBits * (int64_t)intraP < iFrmBC)
    {
      m_rcap.accumTargetBits = (iFrmBC + (intraP >> 1)) / intraP;
    }
    m_rcap.nonRateCapEstim = 1.0;     // changed in case of capping
    m_rcap.gopAdaptedQPAdj = 0;       // changed in first GOP of scene

    const int  gopQP = (iFrmQP + MAX_QP + 1) >> 1;
    const double fac = double(m_pcEncCfg->m_FrameScale * intraP) / m_pcEncCfg->m_FrameRate;
    const double mBC = (m_pcEncCfg->m_RCMaxBitrate > 0 && m_pcEncCfg->m_RCMaxBitrate != INT32_MAX ? m_pcEncCfg->m_RCMaxBitrate * fac : 0.0);

    if (mBC > 0.0 && iFrmBC + bFrmBC_final > mBC) // max. I-period bit-count exceeded
    {
      m_rcap.nonRateCapEstim = double(iFrmBC + bFrmBC_final) / mBC;
      const double d = std::max(0, gopQP) + (105.0 / 128.0) * sqrt((double)std::max(1, gopQP)) * log(m_rcap.nonRateCapEstim) / log(2.0);

      dQP += Clip3(0, MAX_QP, int(0.5 + d + 0.5 * std::max(0.0, qpStart - d))) - std::max(0, gopQP);
    }
  }

  // assign dQP to pictures 
  for (auto picItr = picListBegin; picItr != picList.end(); ++picItr)
  {
    auto pic = (*picItr);
    if( pic->gopEntry->m_gopNum == gopNum )
    {
      pic->gopAdaptedQP = dQP;
      pic->force2ndOrder = f2O;
    }
    if( pic == &keyPic && nextKeyPicAfterIDR ) // consider a virtual GOP containing only one IDR pic
      break;
  }

  keyPic.gopAdaptedQP = dQP; // TODO: add any additional key-frame offset here
  keyPic.force2ndOrder = f2O;
  if( m_pcEncCfg->m_disableForce2ndOderFilter )
  {
    keyPic.force2ndOrder = false;
  }

  if(m_pcEncCfg->m_rateCap)
  {
    // enable QP adjustment after coded Intra in the first GOP or on a scene cut
    // NOTE: on some scene cuts, in case of low motion activity, targetBits equals zero (QPA)
    if(m_rcap.accumGopCounter == 0 && m_rcap.accumTargetBits > 0 && !nextKeyPicAfterIDR)
    {
      for(auto picItr = picListBegin; picItr != picList.end(); ++picItr)
      {
        auto pic = (*picItr);
        // just on the next picture in decoding order after start of GOP
        if(pic->gopEntry->m_gopNum == gopNum && !pic->gopEntry->m_isStartOfGop)
        {
          pic->isSceneCutCheckAdjQP = true;
          break;
        }
      }
      for(auto picItr = picListBegin; picItr != picList.end(); ++picItr)
      {
        auto pic = (*picItr);
        if(pic->gopEntry->m_gopNum == gopNum && !pic->gopEntry->m_isStartOfGop && !pic->isSceneCutCheckAdjQP)
        {
          pic->isSceneCutGOP = true;
        }
      }
    }
    m_rcap.accumGopCounter++;
  }
}

void EncGOP::xInitFirstSlice( Picture& pic, const PicList& picList, bool isEncodeLtRef )
{
  memset( pic.cs->alfAps, 0, sizeof(pic.cs->alfAps));

  const int curPoc          = pic.getPOC();
  Slice* slice              = pic.allocateNewSlice();
  pic.cs->picHeader         = new PicHeader;
  const SPS& sps            = *(slice->sps);
  vvencNalUnitType naluType = xGetNalUnitType( pic.gopEntry );
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
      slice->picHeader->splitConsOverride = slice->picHeader->maxMTTDepth[i] != sps.maxMTTDepth[i];
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
  int ipc = ( m_pcEncCfg->m_DecodingRefreshType == VVENC_DRT_IDR_NO_RADL ) ? m_pcEncCfg->m_IntraPeriod : 0;
  if ( slice->isRplPicMissing( picList, REF_PIC_LIST_0, missingPoc, ipc ) || slice->isRplPicMissing( picList, REF_PIC_LIST_1, missingPoc, ipc ) )
  {
    slice->createExplicitReferencePictureSetFromReference( picList, slice->rpl[0], slice->rpl[1], ipc );
  }
  slice->applyReferencePictureListBasedMarking( picList, slice->rpl[0], slice->rpl[1], 0, *slice->pps, m_pcEncCfg->m_numThreads == 0 );

  // nalu type refinement
  if ( xIsSliceTemporalSwitchingPoint( slice, picList ) )
  {
    naluType = VVENC_NAL_UNIT_CODED_SLICE_STSA;
    slice->nalUnitType = naluType;
  }

  const int maxTLayer  = m_pcEncCfg->m_picReordering && m_pcEncCfg->m_GOPSize > 1 ? vvenc::ceilLog2( m_pcEncCfg->m_GOPSize ) : 0;
  const int numRefCode = pic.useNumRefs ? m_pcEncCfg->m_numRefPicsSCC : m_pcEncCfg->m_numRefPics;
  const int tLayer     = slice->TLayer;
  const int numRefs    = numRefCode < 10 ? numRefCode : ( int( numRefCode / pow( 10, maxTLayer - tLayer ) ) % 10 );

  // reference list
  slice->numRefIdx[REF_PIC_LIST_0] = sliceType == VVENC_I_SLICE ? 0 : ( numRefs ? std::min( numRefs, slice->rpl[0]->numberOfActivePictures ) : slice->rpl[0]->numberOfActivePictures );
  slice->numRefIdx[REF_PIC_LIST_1] = sliceType != VVENC_B_SLICE ? 0 : ( numRefs ? std::min( numRefs, slice->rpl[1]->numberOfActivePictures ) : slice->rpl[1]->numberOfActivePictures );
  slice->constructRefPicList  ( picList, false, m_pcEncCfg->m_numThreads == 0 );

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

  // the slices' deblocking control, as in VTM: a slice is deblocked as the PPS says (a decoder infers
  // sh_deblocking_filter_disabled_flag from the PPS when the slice does not override it), and a GOP entry's offsets
  // apply only when the PPS lets slices override its parameters (LoopFilterOffsetInPPS 0) and deblocking is enabled
  slice->deblockingFilterDisable  = slice->pps->deblockingFilterDisabled;
  slice->deblockingFilterOverride = slice->pps->deblockingFilterOverrideEnabled && !slice->pps->deblockingFilterDisabled
                                    && sliceType != VVENC_I_SLICE && (gopEntry.m_betaOffsetDiv2 || gopEntry.m_tcOffsetDiv2);

  // only the highest DeblockLastTLayers temporal layers are deblocked; the signed tLayer is compared, since without
  // temporal layers m_maxTLayer - m_deblockLastTLayers is negative and no layer is at or below it (the unsigned
  // slice->TLayer would compare it as a huge value)
  if( m_pcEncCfg->m_deblockLastTLayers > 0 && tLayer <= m_pcEncCfg->m_maxTLayer - m_pcEncCfg->m_deblockLastTLayers )
  {
    slice->deblockingFilterOverride = true;
    slice->deblockingFilterDisable  = true;
  }

  if( slice->deblockingFilterOverride )
  {
    for( int comp = 0; comp < MAX_NUM_COMP; comp++)
    {
      //TODO: gopEntry.m_tcOffsetDiv2 and gopEntry.m_betaOffsetDiv2 are set with the luma value also for the chroma components (currently not used or all values are equal)
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
  pic.isReferenced = true;

  pic.picApsMap.clearActive();
  pic.picApsMap.setApsIdStart( ALF_CTB_MAX_NUM_APS );
  for ( int i = 0; i < ALF_CTB_MAX_NUM_APS; i++ )
  {
    const int apsMapIdx = ( i << NUM_APS_TYPE_LEN ) + ALF_APS;
    APS* alfAPS = pic.picApsMap.getPS( apsMapIdx );
    if ( alfAPS )
    {
      alfAPS->apsId      = MAX_UINT;
      alfAPS->temporalId = MAX_INT;
      alfAPS->poc        = MAX_INT;
      pic.picApsMap.clearChangedFlag( apsMapIdx );
      alfAPS->alfParam.reset();
      alfAPS->ccAlfParam.reset();
    }
  }
  CHECK( slice->enableDRAPSEI && m_pcEncCfg->m_maxParallelFrames, "Dependent Random Access Point is not supported by Frame Parallel Processing" );

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

void EncGOP::xWritePicture( Picture& pic, AccessUnitList& au, bool isEncodeLtRef )
{
  // first pass temporal down-sampling
  if( ( ! m_pcRateCtrl->rcIsFinalPass || m_isPreAnalysis ) && pic.gopEntry->m_skipFirstPass )
  {
    m_pcRateCtrl->addRCPassStats( pic.cs->slice->poc,
        0,                /* qp */
        0,                /* lambda */
        pic.picVA.visAct,
        0,                /* numBits */
        0,                /* psnrY */
        pic.cs->slice->isIntra(),
        pic.cs->slice->TLayer,
        pic.gopEntry->m_isStartOfIntra,
        pic.gopEntry->m_isStartOfGop,
        pic.gopEntry->m_gopNum,
        pic.gopEntry->m_scType,
        pic.picVA.spatAct[CH_L],
        pic.m_picShared->m_picMotEstError,
        pic.m_picShared->m_minNoiseLevels );
    return;
  }

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "bsfinal", 1 ) );
  pic.encTime.startTimer();

  au.poc           = pic.poc;
  au.temporalLayer = pic.TLayer;
  au.refPic        = pic.isReferenced;
  au.userData      = pic.userData;
  if( ! pic.slices.empty() )
  {
    au.sliceType = pic.slices[ 0 ]->sliceType;
  }

  if( pic.ctsValid )
  {
    const int64_t iDiffFrames = m_numPicsCoded - pic.poc - pic.picsInMissing;
    au.cts      = pic.cts;
    au.ctsValid = pic.ctsValid;
    if ( pic.picOutOffset )
    {
      m_numPicsOutOffset += pic.picOutOffset;
    }
    if( m_pcEncCfg->m_TicksPerSecond > 0 )
      au.dts      = ( ( iDiffFrames - m_pcEncCfg->m_maxTLayer + m_numPicsOutOffset ) * m_ticksPerFrameMul4 ) / 4 + au.cts;
    else
      au.dts      = ( ( iDiffFrames - m_pcEncCfg->m_maxTLayer + m_numPicsOutOffset )) + au.cts;
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

  // send ALF APS
  if ( sps.alfEnabled && (slice->alfEnabled[COMP_Y] || slice->ccAlfCbEnabled || slice->ccAlfCrEnabled ))
  {
    for ( int apsId = 0; apsId < ALF_CTB_MAX_NUM_APS; apsId++ )
    {
      ParameterSetMap<APS>& apsMap = pic.picApsMap;
      const int apsMapIdx          = ( apsId << NUM_APS_TYPE_LEN ) + ALF_APS;
      APS* aps                     = apsMap.getPS( apsMapIdx );
      bool writeAps                = aps && apsMap.getChangedFlag( apsMapIdx );
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

// True when the two film grain characteristics SEI messages would be written with the same syntax element values.
static bool sameFilmGrainCharacteristics( const SeiFgc& a, const SeiFgc& b )
{
  if( a.fgcCancelFlag || b.fgcCancelFlag )
  {
    return a.fgcCancelFlag == b.fgcCancelFlag;
  }
  if( a.filmGrainModelId != b.filmGrainModelId || a.separateColourDescriptionPresent != b.separateColourDescriptionPresent
      || a.blendingModeId != b.blendingModeId || a.log2ScaleFactor != b.log2ScaleFactor || a.fgcPersistenceFlag != b.fgcPersistenceFlag )
  {
    return false;
  }
  if( a.separateColourDescriptionPresent
      && ( a.filmGrainBitDepthLumaMinus8 != b.filmGrainBitDepthLumaMinus8 || a.filmGrainBitDepthChromaMinus8 != b.filmGrainBitDepthChromaMinus8
           || a.filmGrainFullRangeFlag != b.filmGrainFullRangeFlag || a.filmGrainColourPrimaries != b.filmGrainColourPrimaries
           || a.filmGrainTransferCharacteristics != b.filmGrainTransferCharacteristics || a.filmGrainMatrixCoeffs != b.filmGrainMatrixCoeffs ) )
  {
    return false;
  }
  for( int c = 0; c < MAX_NUM_COMP; c++ )
  {
    const SeiFgc::CompModel& ca = a.compModel[ c ];
    const SeiFgc::CompModel& cb = b.compModel[ c ];
    // as in SEIWriter: a component is written as present only with intervals and model values
    const bool presentA = ca.presentFlag && ca.numIntensityIntervals > 0 && ca.numModelValues > 0;
    const bool presentB = cb.presentFlag && cb.numIntensityIntervals > 0 && cb.numModelValues > 0;
    if( presentA != presentB )
    {
      return false;
    }
    if( !presentA )
    {
      continue;
    }
    if( ca.numIntensityIntervals != cb.numIntensityIntervals || ca.numModelValues != cb.numModelValues )
    {
      return false;
    }
    for( int i = 0; i < ca.numIntensityIntervals; i++ )
    {
      const SeiFgc::CompModelIntensityValues& va = ca.intensityValues[ i ];
      const SeiFgc::CompModelIntensityValues& vb = cb.intensityValues[ i ];
      if( va.intensityIntervalLowerBound != vb.intensityIntervalLowerBound || va.intensityIntervalUpperBound != vb.intensityIntervalUpperBound
          || !std::equal( va.compModelValue.begin(), va.compModelValue.begin() + ca.numModelValues, vb.compModelValue.begin() ) )
      {
        return false;
      }
    }
  }
  return true;
}

// Adds the parameter set of a finished analysis. vvdec keeps synthesizing the grain of a component that a later SEI
// marks as absent (fixed by its pull request 414, open in October 2026), so once a component had grain, a parameter set
// without it signals one interval with scaling factor 0 instead.
void EncGOP::xAddFilmGrainParameters( const FgaTask& task )
{
  FilmGrainParam param = task.result;
  for( int c = 0; c < MAX_NUM_COMP; c++ )
  {
    SeiFgc::CompModel& cm = param.compModel[ c ];
    if( m_pcEncCfg->m_fga && !cm.presentFlag && m_fgcCompSignalled[ c ] )
    {
      cm.presentFlag           = true;
      cm.numIntensityIntervals = 1;
      cm.numModelValues        = 1;
      cm.intensityValues.resize( 1 );
      cm.intensityValues[ 0 ].intensityIntervalLowerBound = 0;
      cm.intensityValues[ 0 ].intensityIntervalUpperBound = 255;
      cm.intensityValues[ 0 ].compModelValue.assign( 1, 0 );
    }
    m_fgcCompSignalled[ c ] = m_fgcCompSignalled[ c ] || cm.presentFlag;
  }
  // A set cannot apply to pictures whose access units are written already, so that xFilmGrainSeiNeeded, which compares
  // a picture's set with the previous picture's, sees every change. In low delay (coding order = output order) the
  // analysed picture is the last coded of its GOP and its set would otherwise start among the written pictures before
  // it, compare equal with itself and never be signalled. In random access the analysed picture is coded before the
  // rest of its GOP, so its set already starts after every written picture; the one exception, an IDR without leading
  // pictures (idr_no_radl), coded after the pictures before it in output order, carries the message anyway.
  int startPoc = task.startPoc;
  if( !m_fgcWrittenPocs.empty() )
  {
    startPoc = std::max( startPoc, *m_fgcWrittenPocs.rbegin() + 1 );
  }
  m_fgcByStartPoc.erase( m_fgcByStartPoc.lower_bound( startPoc ), m_fgcByStartPoc.end() );
  m_fgcByStartPoc[ startPoc ] = param;
  while( m_fgcByStartPoc.size() > m_fgcParamSets )
  {
    // the oldest set is dropped; the next one then applies to all earlier pictures, all of them written long ago
    m_fgcByStartPoc.erase( m_fgcByStartPoc.begin() );
    FilmGrainParam oldest = m_fgcByStartPoc.begin()->second;
    m_fgcByStartPoc.erase( m_fgcByStartPoc.begin() );
    m_fgcByStartPoc[ std::numeric_limits<int>::min() ] = oldest;
  }
}

const FilmGrainParam& EncGOP::xFilmGrainParametersAt( int poc ) const
{
  return std::prev( m_fgcByStartPoc.upper_bound( poc ) )->second;   // the first set starts at the lowest int
}

// The film grain SEI of a parameter set; the components the configuration marks absent stay absent.
SeiFgc EncGOP::xFilmGrainSei( const FilmGrainParam& param, ChromaFormat chromaFormat )
{
  SeiFgc sei;
  m_seiEncoder.initSeiFgc( &sei );
  sei.log2ScaleFactor = param.log2ScaleFactor;
  for( int c = 0; c < getNumberValidComponents( chromaFormat ); c++ )
  {
    if( sei.compModel[ c ].presentFlag )
    {
      sei.compModel[ c ] = param.compModel[ c ];
    }
  }
  return sei;
}

// A persistent film grain SEI message applies to its picture and the following pictures in output order, up to the next
// picture with a message, so repeating it in every access unit (45-60 bytes, 2-3.5% of a low rate encode) is not needed:
// a picture needs it when the picture before it in output order signals other parameters. Random access points, where
// decoding can start, always get the message, and so does a picture with no picture before it written yet in this coded
// layer video sequence (the leading pictures of an IDR, of which the first in output order is not known in coding
// order). Decoding from a CRA, vvdec skips its RASL pictures but applies their messages to the next decoded picture, so
// the first picture after RASL pictures with a message gets one as well.
bool EncGOP::xFilmGrainSeiNeeded( const Slice& slice, const SeiFgc& sei, ChromaFormat chromaFormat )
{
  if( slice.isIDRorBLA() )
  {
    m_fgcWrittenPocs.clear();   // a new coded layer video sequence, where no earlier message persists
  }
  const bool rasl         = slice.nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RASL;
  const bool randomAccess = slice.isIRAP() || slice.picHeader->gdrPic;
  const bool firstWritten = m_fgcWrittenPocs.lower_bound( slice.poc ) == m_fgcWrittenPocs.begin();
  const bool afterRasl    = !rasl && m_fgcRaslSeiPending;
  const bool changed      = !sameFilmGrainCharacteristics( xFilmGrainSei( xFilmGrainParametersAt( slice.poc - 1 ), chromaFormat ), sei );
  const bool needed       = !sei.fgcPersistenceFlag || randomAccess || firstWritten || afterRasl || changed;

  m_fgcWrittenPocs.insert( slice.poc );
  while( m_fgcWrittenPocs.size() > FGC_SEI_HISTORY )
  {
    m_fgcWrittenPocs.erase( m_fgcWrittenPocs.begin() );
  }
  m_fgcRaslSeiPending = rasl && ( m_fgcRaslSeiPending || needed );
  return needed;
}

// The grain the coding of a picture lost, for its film grain SEI: per component, the summed power per band of periods of
// the difference between the original's smoothest 64x64 tiles and the reconstruction, and the sum of its squares, per
// 8-bit tile mean (the intensity intervals are known only when the SEI is written). Runs when the picture is finished, before its reconstruction can be released; only where
// xScaleToLostGrain uses them (xFitsLostGrain).
void EncGOP::xStoreLostGrainSpectra( Picture& pic ) const
{
  if( !xFitsLostGrain() )
  {
    return;
  }
  for( int c = 0; c < getNumberValidComponents( pic.chromaFormat ); c++ )
  {
    const ComponentID compId = ComponentID( c );
    m_fgAnalyzer.lostGrainSpectra( pic.getOrigBuf( compId ), pic.getRecoBuf( compId ),
                                   m_pcEncCfg->m_internalBitDepth[ toChannelType( compId ) ], pic.lostGrainSpectra[ c ] );
  }
}

// The film grain SEIs are fitted to the grain the coding lost: with the improved analysis (--fga 2 and 3), in the final
// pass, when the SEI is written with the pictures (not as one SEI for the whole sequence).
bool EncGOP::xFitsLostGrain() const
{
  return m_pcEncCfg->m_fga >= 2 && m_pcEncCfg->m_fg.m_fgcSEIEnabled && !m_pcEncCfg->m_fg.m_fgcSEIPerPictureSEI
         && m_pcRateCtrl->rcIsFinalPass && !m_isPreAnalysis;
}

// Drops the fitted values of this component and temporal layer whose intensity interval the picture's SEI no longer
// has (a new analysis can change the intervals within a random access period), so the map stays small.
template<typename LostGrainMap>
static void eraseOtherIntervals( LostGrainMap& map, int c, unsigned layer, const SeiFgc::CompModel& cm )
{
  for( auto it = map.begin(); it != map.end(); )
  {
    bool keep = std::get<0>( it->first ) != c || std::get<3>( it->first ) != layer;
    for( int i = 0; !keep && i < cm.numIntensityIntervals; i++ )
    {
      keep = std::get<1>( it->first ) == cm.intensityValues[ i ].intensityIntervalLowerBound
             && std::get<2>( it->first ) == cm.intensityValues[ i ].intensityIntervalUpperBound;
    }
    it = keep ? std::next( it ) : map.erase( it );
  }
}

// Both cut-offs are signalled (numModelValues 3), so that the fit can move them; missing values get their defaults
// (horizontal 8, vertical = horizontal).
static void signalBothCutoffs( SeiFgc::CompModel& cm )
{
  if( cm.numModelValues >= 3 )
  {
    return;
  }
  for( int i = 0; i < cm.numIntensityIntervals; i++ )
  {
    std::vector<int>& v = cm.intensityValues[ i ].compModelValue;
    if( v.size() < 3 )
    {
      v.resize( 3 );
    }
    if( cm.numModelValues < 2 )
    {
      v[ 1 ] = 8;
    }
    v[ 2 ] = v[ 1 ];
  }
  cm.numModelValues = 3;
}

// How the bands of periods count in the lost-grain fit of an intensity interval: weighted by the eye's sensitivity at the
// display luminance of the interval's middle. A chroma interval is one of chroma values, which say nothing about the
// luminance: mid grey stands in. PQ is told from the transfer characteristics; every other curve (HLG too) is taken as
// an SDR display's.
static FGAnalyzer::LostGrainBands lostGrainBandsOf( const VVEncCfg& cfg, bool luma, bool chroma420, const SeiFgc::CompModelIntensityValues& iv )
{
  constexpr int TRANSFER_PQ = 16;    // SMPTE ST 2084
  constexpr int MID_GREY    = 128;   // 8 bits
  const int    luma8     = luma ? ( iv.intensityIntervalLowerBound + iv.intensityIntervalUpperBound ) / 2 : MID_GREY;
  const double luminance = FGAnalyzer::displayLuminance( luma8, cfg.m_transferCharacteristics == TRANSFER_PQ );
  return FGAnalyzer::lostGrainBands( chroma420, cfg.m_SourceHeight, luminance );
}

// The state a picture's lost-grain fit goes on with. A random access point starts over: its pooled spectra and scaling
// factors are its own. Its leading pictures are coded after it but shown before it, as the end of what came before, so
// they go on with the state from before it: with the state started over they took the cut-offs fitted to the random
// access point's one picture, often the first of another scene (Top Gun, a real encode at QP 31: pictures 154 to 185
// at cut-offs 5 / 5, the 15 leading pictures 186 to 200 of the clean random access picture 201 at its 2 / 3, a step
// of 30 % in the decoded grain's standard deviation between pictures 185 and 186 of one scene; from 202 on 5 / 4).
// The cut-offs go on over the random access point, and so do the costs they are chosen by (LostGrainStates): they
// change there only as at any layer-0 picture (xFitLostGrain). With the costs summed anew the random access point's
// picture alone could change them, and the first layer-0 picture after it chose them for the next 31 pictures from
// those two pictures. The first is an intra picture, which loses coarser grain than the inter pictures after it: in
// real encodes of two Top Gun scenes it alone took 3 / 3 where the pictures before it had 5 / 5, and the two together
// 4 / 4 (kept for the scene's 143 pictures) and 3 / 3 (then 3 / 4), where the same two pictures' costs summed with
// those of the 17 and 31 leading pictures coded between them gave 5 / 5 and 4 / 5 (the decoded grain's standard
// deviation at periods of up to 25 samples: 0.80-0.87 of the lost grain's against 0.93-0.99).
// The stream's first pictures, leading pictures of its first random access point, have no state from before it: they
// take its cut-offs.
EncGOP::LostGrainState& EncGOP::xLostGrainStateOf( const Picture& pic, const Slice& slice )
{
  LostGrainStates& states = m_lostGrain;
  if( slice.isIRAP() || slice.picHeader->gdrPic )
  {
    states.leading = std::move( states.current );
    states.current = LostGrainState();
    states.current.cutoffs = states.leading.cutoffs;
    states.irapPoc = pic.poc;
  }
  if( pic.poc >= states.irapPoc )
  {
    return states.current;
  }
  if( states.leading.cutoffs.empty() )
  {
    states.leading.cutoffs = states.current.cutoffs;
  }
  return states.leading;
}

// The pictures whose lost grain the latest coded picture (the last of pictures) is fitted to: the LOST_GRAIN_POOL
// coded pictures of its scene nearest to it in output order, itself first; of two equally near, the one coded later.
// A picture of another scene lost another scene's grain.
std::vector<const EncGOP::LostGrainPicture*> EncGOP::xLostGrainPool( const std::deque<LostGrainPicture>& pictures )
{
  const LostGrainPicture& current = pictures.back();
  std::vector<const LostGrainPicture*> pool;
  for( auto p = pictures.rbegin(); p != pictures.rend(); ++p )
  {
    if( p->scene == current.scene )
    {
      pool.push_back( &*p );
    }
  }
  std::stable_sort( pool.begin(), pool.end(), [ &current ]( const LostGrainPicture* a, const LostGrainPicture* b )
                    { return std::abs( a->poc - current.poc ) < std::abs( b->poc - current.poc ); } );
  if( pool.size() > LOST_GRAIN_POOL )
  {
    pool.resize( LOST_GRAIN_POOL );
  }
  return pool;
}

// Which pictures of a pool (xLostGrainPool; the first is the picture the pool is of) are also fitted alone, each a limit
// of the picture's scaling factor (lowestPictureFit; LOST_GRAIN_POOL_SPAN): the near ones, the picture itself among
// them; if it has no other near one, also the far ones of higher temporal layers than its own.
std::vector<char> EncGOP::xLostGrainFittedAlone( const std::vector<const LostGrainPicture*>& pool )
{
  const LostGrainPicture& current = *pool.front();
  auto isNear = [ &current ]( const LostGrainPicture* p ) { return std::abs( p->poc - current.poc ) <= LOST_GRAIN_POOL_SPAN; };
  const bool hasNearPicture = std::any_of( pool.begin() + 1, pool.end(), isNear );
  std::vector<char> fittedAlone( pool.size() );
  for( size_t p = 0; p < pool.size(); p++ )
  {
    fittedAlone[ p ] = isNear( pool[ p ] ) || ( !hasNearPicture && pool[ p ]->layer > current.layer );
  }
  return fittedAlone;
}

// A picture's spectra summed per intensity interval of cm: a tile mean belongs to the first interval holding it.
static std::vector<LostGrainSpectrum> lostGrainIntervalSums( const std::vector<LostGrainSpectrum>& spectra, const SeiFgc::CompModel& cm )
{
  std::vector<LostGrainSpectrum> sums( cm.numIntensityIntervals );
  for( const LostGrainSpectrum& s : spectra )
  {
    for( int i = 0; i < cm.numIntensityIntervals; i++ )
    {
      const SeiFgc::CompModelIntensityValues& iv = cm.intensityValues[ i ];
      if( s.mean8 >= iv.intensityIntervalLowerBound && s.mean8 <= iv.intensityIntervalUpperBound )
      {
        sums[ i ].add( s );
        break;
      }
    }
  }
  return sums;
}

// Fits each intensity interval's grain to the grain the coding lost: the analyzer measures the source's grain, but the
// coding keeps part of it (the full synthesis would add it twice) and loses the fine grain first (the analyzer's cut-offs
// are those of the source's grain). Per component: the summed spectra of each interval over the picture's pool
// (xLostGrainPool), over the pool's near pictures (LOST_GRAIN_POOL_SPAN) and of each of its pictures that is fitted
// alone (xLostGrainFittedAlone), then xFitLostGrain or xFitLostGrainAr, on the picture's state (xLostGrainStateOf).
void EncGOP::xScaleToLostGrain( Picture& pic, const Slice& slice, SeiFgc& sei )
{
  LostGrainState& state = xLostGrainStateOf( pic, slice );
  // The picture's spectra are kept for the pictures coded after it, those of two groups of pictures (the nearest coded
  // pictures of a group's first picture are the last ones of the group before it), at least of two pools.
  const size_t keptPictures = std::max<size_t>( 2 * m_pcEncCfg->m_GOPSize, 2 * LOST_GRAIN_POOL );
  state.pictures.emplace_back();
  state.pictures.back().poc   = pic.poc;
  state.pictures.back().scene = pic.m_picShared->fgaSceneCut() ? pic.poc : pic.m_picShared->fgaLastSceneCutPoc();
  state.pictures.back().layer = slice.TLayer;
  for( int c = 0; c < getNumberValidComponents( pic.chromaFormat ); c++ )
  {
    state.pictures.back().spectra[ c ] = std::move( pic.lostGrainSpectra[ c ] );   // the picture needs them no more
    pic.lostGrainSpectra[ c ].clear();
  }
  if( state.pictures.size() > keptPictures )
  {
    state.pictures.pop_front();
  }
  const std::vector<const LostGrainPicture*> pool        = xLostGrainPool( state.pictures );
  const std::vector<char>                    fittedAlone = xLostGrainFittedAlone( pool );
  for( int c = 0; c < getNumberValidComponents( pic.chromaFormat ); c++ )
  {
    SeiFgc::CompModel& cm = sei.compModel[ c ];
    if( !cm.presentFlag || cm.numIntensityIntervals == 0 )
    {
      continue;
    }
    const bool arModel = sei.filmGrainModelId == 1;
    if( !arModel )
    {
      signalBothCutoffs( cm );
    }
    const int  n         = cm.numIntensityIntervals;
    const bool chroma420 = c != COMP_Y && pic.chromaFormat == CHROMA_420;
    // the intervals' sums of all pictures of the pool, of the near ones and of each picture that is fitted alone, reduced
    // to the grain power the coding lost per band (of a single picture: the most its tiles say it lost)
    std::vector<LostGrainSpectrum> poolSums( n ), nearSums( n );
    std::vector<std::vector<FGAnalyzer::LostGrainPower>> picturePower;   // [ picture fitted alone ][ interval ]
    for( size_t p = 0; p < pool.size(); p++ )
    {
      const std::vector<LostGrainSpectrum> sums   = lostGrainIntervalSums( pool[ p ]->spectra[ c ], cm );
      const bool                           isNear = std::abs( pool[ p ]->poc - pic.poc ) <= LOST_GRAIN_POOL_SPAN;
      if( fittedAlone[ p ] )
      {
        picturePower.emplace_back();
      }
      for( int i = 0; i < n; i++ )
      {
        poolSums[ i ].add( sums[ i ] );
        if( isNear )
        {
          nearSums[ i ].add( sums[ i ] );
        }
        if( fittedAlone[ p ] )
        {
          picturePower.back().push_back( FGAnalyzer::lostGrainPowerAtMost( sums[ i ] ) );
        }
      }
    }
    std::vector<FGAnalyzer::LostGrainPower> power( n ), nearPower( n );
    std::vector<FGAnalyzer::LostGrainBands> bands( n );
    for( int i = 0; i < n; i++ )
    {
      power[ i ]     = FGAnalyzer::lostGrainPower( poolSums[ i ] );
      nearPower[ i ] = FGAnalyzer::lostGrainPower( nearSums[ i ] );
      bands[ i ]     = lostGrainBandsOf( *m_pcEncCfg, c == COMP_Y, chroma420, cm.intensityValues[ i ] );
    }
    const ComponentID compId   = ComponentID( c );
    const int         bitDepth = m_pcEncCfg->m_internalBitDepth[ toChannelType( compId ) ];
    if( arModel )
    {
      xFitLostGrainAr( state, c, slice.TLayer, bitDepth, sei.log2ScaleFactor, power, nearPower, bands, cm );
    }
    else
    {
      xFitLostGrain( state, m_lostGrain.cutoffCosts[ c ], c, slice.TLayer, bitDepth, chroma420, sei.log2ScaleFactor, power, picturePower,
                     bands, cm );
    }
    eraseOtherIntervals( state.fitted, c, slice.TLayer, cm );
  }
  if( sei.filmGrainModelId == 0 )   // the AR model's log2 scale factor is the unit of its filter too: its fit stays in range
  {
    sei.log2ScaleFactor -= FGAnalyzer::lostGrainSignalledScales( sei.log2ScaleFactor, sei.compModel );
  }
}

// An interval needs this many pooled 64x64 tiles to be fitted (xFitLostGrain): the band of the longest fitted periods
// has about 31 coefficients per tile and direction, so 12 tiles give its power to about 7 % (its amplitude to 4 %).
static constexpr int MIN_POOLED_TILES = 12;

// The lowest of poolFit, the fit to interval i of the pool, and the fits to that interval of each picture fitted alone
// (picturePower, xLostGrainFittedAlone) that has MIN_POOLED_TILES tiles of it; fitTo( power, scale ) fits a scaling
// factor to a lost grain power and returns whether it could.
template<typename FitTo>
static int lowestPictureFit( int poolFit, const std::vector<std::vector<FGAnalyzer::LostGrainPower>>& picturePower, int i, FitTo fitTo )
{
  int lowest = poolFit;
  for( const std::vector<FGAnalyzer::LostGrainPower>& ofPicture : picturePower )
  {
    int fit;
    if( ofPicture[ i ].tiles >= MIN_POOLED_TILES && fitTo( ofPicture[ i ], fit ) )
    {
      lowest = std::min( lowest, fit );
    }
  }
  return lowest;
}

// An interval that was never fitted (too few flat tiles at its brightness) would keep the analyzer's grain, all of the
// source's grain on top of the grain the coding kept, beside intervals reduced to the grain the coding lost. It takes
// the share of the analyzer's grain that the nearest fitted interval got, but never more than the analyzer's grain.
// analyzerScale: the analyzer's scaling factors; hasFit: the intervals with a fit of their own, now or before.
static void scaleUnfittedLikeNeighbours( const std::vector<int>& analyzerScale, const std::vector<char>& hasFit, std::vector<int>& scale )
{
  const int n = (int) scale.size();
  for( int i = 0; i < n; i++ )
  {
    if( hasFit[ i ] )
    {
      continue;
    }
    int nearest = -1;
    for( int j = 0; j < n; j++ )
    {
      if( hasFit[ j ] && analyzerScale[ j ] > 0 && ( nearest < 0 || std::abs( j - i ) < std::abs( nearest - i ) ) )
      {
        nearest = j;
      }
    }
    if( nearest >= 0 )
    {
      const int like = (int) std::lround( double( analyzerScale[ i ] ) * scale[ nearest ] / analyzerScale[ nearest ] );
      scale[ i ] = std::min( scale[ i ], like );
    }
  }
}

// The cut-offs are chosen by every cut-off pair's costs summed over the pictures, where a picture's sum counts this
// much of its value one picture (in coding order) later: about the last ten pictures decide, of all temporal layers.
// With a choice per temporal layer, each following its own pictures by a step per picture, neighbouring pictures were
// synthesized at different cut-offs, and a change of cut-offs is a change of the grain's look and of its strength (the
// scaling factor moves with it only at equal power in a 16x16 block): on meridian_talk the cut-offs changed between
// 112 of 360 consecutive pictures and the decoded grain's standard deviation stepped by up to 10 % with them, on
// nocturne_grain between 102 of 600 (9 %).
static constexpr double LOST_GRAIN_CUTOFF_MEMORY = 0.9;
// The cut-offs are chosen in two steps, with margins in shares of the cost of no grain at all, so that they mean the
// same whatever the grain's power: a share of 0.01 is the band the eye weighs most missing 10 % of its amplitude.
// The eye's cost decides first: only the pairs whose seen cost is within LOST_GRAIN_SEEN_MARGIN of the least are
// candidates. Among them the cost that counts every fitted band decides: the fine grain is kept where that costs the
// coarse grain little, and pairs the eye cannot tell apart do not take turns with the pictures' noise. The current
// pair stays while its seen cost is within LOST_GRAIN_SEEN_KEEP_MARGIN of the least and no candidate's all-bands
// cost is lower by LOST_GRAIN_ALL_BANDS_MARGIN.
// The seen margin is for what the eye's weights hardly count but a viewer sees. At 4K they count the grain of periods
// of 6.25-3.125 samples 0.02-0.05 of that of 25-12.5 samples, yet on Top Gun (4K HDR, QP 31) it is 0.28-0.33 of the
// lost grain's power at periods of up to 25 samples, and no pair holds both: the lost grain's amplitude rises 1.3-1.6
// times from 25-12.5 to 12.5-6.25 samples, the synthesis's is flat up to its cut-off. Replayed on the spectra of real
// encodes' pictures (seen cost above the least; synthesized over lost amplitude at periods of 25-12.5 / 12.5-6.25 /
// 6.25-3.125 samples; synthesized over lost standard deviation at periods of up to 25 samples), 62 pictures of one
// scene: 3 / 3 +0.002, 1.00 / 0.94 / 0.43, 0.77; 4 / 4 +0.000, 0.90 / 1.03 / 0.76, 0.88; 4 / 5 +0.003, 0.88 / 1.03 /
// 0.96, 0.94; 5 / 5 +0.008, 0.85 / 1.03 / 1.14, 1.00; in two other scenes 4 / 4 is at +0.011 and +0.033 (0.85 and 0.78
// at 25-12.5 samples). With a seen margin of 0.005 the choice among these fell to the first pictures of a scene and
// stayed by the keep margin: in real encodes the decoded grain had 0.71-0.87 of the lost grain's standard deviation
// at 3 / 3 to 4 / 4 (grain of 0.85 looked a little weak) and 0.83-0.99 at 4 / 5 to 5 / 5, with 0.85-0.94 of its
// amplitude at 25-12.5 samples. There 5 / 5 stayed by a keep margin of 0.02, but in the replay it is at +0.020 and
// +0.024 at two of the scene's layer-0 pictures, so the keep margin is above that.
// meridian_talk, where the eye's cost alone moved the cut-offs from 12 / 8 to 7 / 7 with a step of 29 % in the
// grain's standard deviation, stays at 14 / 10 over its 240 pictures (0.85 / 0.99 / 1.07, and 1.01 at finer periods;
// by the eye's cost alone 0.87 / 1.02 / 1.02 and 0.60). A keep margin of 0.01 let meridian_talk change once (7.5 %).
static constexpr double LOST_GRAIN_SEEN_MARGIN      = 0.02;
static constexpr double LOST_GRAIN_SEEN_KEEP_MARGIN = 0.03;
static constexpr double LOST_GRAIN_ALL_BANDS_MARGIN = 0.02;

// The pair of cut-offs (its index in sums) the component should have by the costs summed so far; current: the pair it
// has, below 0: none yet. See LOST_GRAIN_SEEN_MARGIN.
int EncGOP::xBestLostGrainCutoffs( const LostGrainCosts& sums, int current )
{
  const double leastSeen = *std::min_element( sums.seen.begin(), sums.seen.end() );
  int best = -1;   // the candidate of the least all-bands cost, the first of equal ones
  for( int k = 0; k < (int) sums.seen.size(); k++ )
  {
    const bool candidate = sums.seen[ k ] <= leastSeen + LOST_GRAIN_SEEN_MARGIN * sums.seenOfNone;
    if( candidate && ( best < 0 || sums.allBands[ k ] < sums.allBands[ best ] - 1e-12 ) )
    {
      best = k;
    }
  }
  if( current < 0 )
  {
    return best;
  }
  const bool seenAlike     = sums.seen[ current ] <= leastSeen + LOST_GRAIN_SEEN_KEEP_MARGIN * sums.seenOfNone;
  const bool clearlyBetter = sums.allBands[ best ] < sums.allBands[ current ] - LOST_GRAIN_ALL_BANDS_MARGIN * sums.allBandsOfNone;
  return seenAlike && !clearlyBetter ? current : best;
}

// One pair of cut-offs for all intensity intervals of the component, whatever the analyzer's. The analyzer's cut-offs
// are those of the source's grain, which can be finer than the grain the coding lost: on Top Gun (4K HDR, QP 31) the
// eye sees the lost grain mostly at periods of 6 to 25 samples, where cut-offs of 5 to 10 put back 0.44-0.48 of its
// amplitude; the synthesis's grain is flat up to ( cut-off + 1 ) / 32 cycles per sample, and at cut-offs 4 to 5 it can
// match the lost grain there within the tolerances. And they change with every new analysis and from interval to
// interval: cut-offs at a shift from them would change with them, and the summed costs of a shift would be those of
// different cut-offs from one analysis to the next. (Replayed on the spectra of meridian_talk, whose intervals have
// cut-offs from 13 / 13 to 10 / 10, one pair for all costs next to nothing: the eye-weighted synthesized over lost
// amplitude is 0.93 against 0.94 with one shift.)
// The pair is chosen by the tiles-weighted sums of the intervals' fits at every pair (FGAnalyzer::lostGrainFit,
// xBestLostGrainCutoffs). So that the synthesized grain does not pulse, the costs of the latest pictures of all temporal
// layers decide together (LOST_GRAIN_CUTOFF_MEMORY), and the pair changes only at a picture of the lowest temporal
// layer: the pictures between two of those are coded after the later one, so in output order the cut-offs change once,
// there, while a change in the middle of a group of pictures would alternate between neighbouring pictures coded before
// and after it. The component's first fit sets the pair at once (until then the analyzer's cut-offs stay). A random
// access point starts neither the pair nor the summed costs (costs) over, and its leading pictures add their costs
// like every picture but keep the pair from before it (xLostGrainStateOf).
// Then per interval the scaling factor fitted at the pair, the lowest of the fits to the pool and to each of its pictures
// that is fitted alone (picturePower, lowestPictureFit), never more grain power than FGAnalyzer::LOST_GRAIN_CAP_POWER
// times the analyzer's.
// An interval needs MIN_POOLED_TILES pooled tiles to be fitted.
// Per picture of a temporal layer the scaling factor rises at most max(1, s0 / 8) (s0 the analyzer's scaling factor)
// from its last value (a picture the coding served badly loses far more than grain, and the limit keeps that out of
// its neighbours); a lower fit is taken at once (a limit on the way down held the grain above the fit for many
// pictures, see xFitLostGrainAr), and so is the first fit at new cut-offs: the last value, moved to them at equal power
// in a 16x16 block, is not the same grain to the eye, and rising from it would dim every layer's next pictures in
// turn. A picture with nothing to fit keeps the pair, an interval without a fit keeps its last grain power, and one
// that never had a fit follows its neighbours (scaleUnfittedLikeNeighbours).
void EncGOP::xFitLostGrain( LostGrainState& state, LostGrainCosts& costs, int c, unsigned layer, int bitDepth, bool chroma420,
                            int log2ScaleFactor, const std::vector<FGAnalyzer::LostGrainPower>& power,
                            const std::vector<std::vector<FGAnalyzer::LostGrainPower>>& picturePower,
                            const std::vector<FGAnalyzer::LostGrainBands>& bands, SeiFgc::CompModel& cm ) const
{
  // static: the lambdas below read them without a capture (MSVC C3493). The cut-offs' range, which
  // FGAnalyzer::lostGrainShape clips to as well (2 .. 14).
  static constexpr int MIN_CUTOFF  = 2;
  static constexpr int MAX_CUTOFF  = 14;
  static constexpr int NUM_CUTOFFS = MAX_CUTOFF - MIN_CUTOFF + 1;
  static constexpr int NUM_PAIRS   = NUM_CUTOFFS * NUM_CUTOFFS;
  auto cutoff = []( int x ) { return Clip3( MIN_CUTOFF, MAX_CUTOFF, x ); };
  auto pairOf = []( int h, int w ) { return ( h - MIN_CUTOFF ) * NUM_CUTOFFS + w - MIN_CUTOFF; };
  const int n = cm.numIntensityIntervals;
  std::vector<int> s0( n ), h0( n ), w0( n );   // the analyzer's values
  for( int i = 0; i < n; i++ )
  {
    const std::vector<int>& v = cm.intensityValues[ i ].compModelValue;
    s0[ i ] = v[ 0 ];
    h0[ i ] = v[ 1 ];
    w0[ i ] = v[ 2 ];
  }
  // the scaling factors fitted here can be above the largest one signalled: xScaleToLostGrain lowers the SEI's log2
  // scale factor for them; the values kept for the next pictures stay in the analyzer's units
  const int largestScale = FGAnalyzer::lostGrainLargestScale( log2ScaleFactor );

  // Every interval with enough tiles is fitted at every pair ([ interval ][ pair ]; the grain's shape at a pair is the
  // same for all intervals), and the picture's costs of a pair are the sums of its fits; an interval is fitted or not
  // whatever the pair.
  std::vector<std::vector<FGAnalyzer::LostGrainFit>> fits( n );
  LostGrainCosts pictureCosts( NUM_PAIRS );
  bool           anyFit      = false;
  const bool     enoughTiles = std::any_of( power.begin(), power.end(), []( const FGAnalyzer::LostGrainPower& p ) { return p.tiles >= MIN_POOLED_TILES; } );
  for( int pair = 0; enoughTiles && pair < NUM_PAIRS; pair++ )
  {
    const int h = MIN_CUTOFF + pair / NUM_CUTOFFS, w = MIN_CUTOFF + pair % NUM_CUTOFFS;
    const FGAnalyzer::LostGrainShape shape = m_fgAnalyzer.lostGrainShape( chroma420, h, w );
    for( int i = 0; i < n; i++ )
    {
      if( power[ i ].tiles < MIN_POOLED_TILES )
      {
        continue;
      }
      fits[ i ].push_back( FGAnalyzer::lostGrainFit( power[ i ], bands[ i ], bitDepth, chroma420, log2ScaleFactor, s0[ i ], h0[ i ], w0[ i ],
                                                     shape, largestScale ) );
      const FGAnalyzer::LostGrainFit& fit = fits[ i ].back();
      if( fit.seenCost < 0 )
      {
        continue;
      }
      anyFit = true;
      pictureCosts.seen    [ pair ] += fit.seenCost     * power[ i ].tiles;
      pictureCosts.allBands[ pair ] += fit.allBandsCost * power[ i ].tiles;
      if( pair == 0 )   // the costs of no grain are the same at every pair
      {
        pictureCosts.seenOfNone     += fit.seenCostOfNone     * power[ i ].tiles;
        pictureCosts.allBandsOfNone += fit.allBandsCostOfNone * power[ i ].tiles;
      }
    }
  }
  const auto lastCutoffs = state.cutoffs.find( c );
  const bool firstFit    = lastCutoffs == state.cutoffs.end();
  int pair = firstFit ? -1 : pairOf( lastCutoffs->second.h, lastCutoffs->second.w );   // below 0: the analyzer's cut-offs
  if( anyFit )   // a picture with nothing to fit records nothing: the component's first fit ever then sets the pair
  {
    costs.fadeAndAdd( pictureCosts, LOST_GRAIN_CUTOFF_MEMORY );
    if( firstFit || layer == 0 )
    {
      pair = xBestLostGrainCutoffs( costs, pair );
    }
    LostGrainCutoffs& cutoffs = state.cutoffs[ c ];
    cutoffs.h = MIN_CUTOFF + pair / NUM_CUTOFFS;
    cutoffs.w = MIN_CUTOFF + pair % NUM_CUTOFFS;
  }
  std::vector<int>  analyzerScale( n ), scale( n );
  std::vector<char> hasFit( n );
  for( int i = 0; i < n; i++ )
  {
    SeiFgc::CompModelIntensityValues& iv = cm.intensityValues[ i ];
    const int h = pair < 0 ? cutoff( h0[ i ] ) : MIN_CUTOFF + pair / NUM_CUTOFFS;
    const int w = pair < 0 ? cutoff( w0[ i ] ) : MIN_CUTOFF + pair % NUM_CUTOFFS;
    analyzerScale[ i ] = FGAnalyzer::equalPowerScale( s0[ i ], chroma420, h0[ i ], w0[ i ], h, w );   // the analyzer's power
    const int cap      = FGAnalyzer::lostGrainCapScale( s0[ i ], chroma420, h0[ i ], w0[ i ], h, w, largestScale );
    const LostGrainKey key( c, iv.intensityIntervalLowerBound, iv.intensityIntervalUpperBound, layer );
    // the interval's last value moved to the new cut-offs at equal power; before its first fit, the analyzer's grain, which
    // is not recorded: the first fit is then free instead of starting max(1, s0 / 8) per picture down from the analyzer's
    const auto last   = state.fitted.find( key );
    const bool fitted = last != state.fitted.end();
    const int  held   = fitted ? FGAnalyzer::equalPowerScale( last->second.scale, chroma420, last->second.h, last->second.w, h, w ) : analyzerScale[ i ];
    scale[ i ] = held;
    const FGAnalyzer::LostGrainFit* lostFit = fits[ i ].empty() ? nullptr : &fits[ i ][ pairOf( h, w ) ];
    const bool fitsNow = lostFit && lostFit->seenCost >= 0;
    if( fitsNow )
    {
      const FGAnalyzer::LostGrainShape shape = m_fgAnalyzer.lostGrainShape( chroma420, h, w );
      const int fit = lowestPictureFit( lostFit->scale, picturePower, i, [ & ]( const FGAnalyzer::LostGrainPower& ofPicture, int& pictureFit )
      {
        const FGAnalyzer::LostGrainFit single = FGAnalyzer::lostGrainFit( ofPicture, bands[ i ], bitDepth, chroma420, log2ScaleFactor, s0[ i ],
                                                                          h0[ i ], w0[ i ], shape, largestScale );
        pictureFit = single.scale;
        return single.seenCost >= 0;
      } );
      const bool sameCutoffs = fitted && last->second.h == h && last->second.w == w;
      const int  step        = std::max( 1, s0[ i ] / 8 );
      scale[ i ] = sameCutoffs ? std::min( fit, held + step ) : fit;
    }
    scale[ i ]  = std::min( scale[ i ], cap );
    hasFit[ i ] = fitsNow || fitted;
    if( hasFit[ i ] )
    {
      state.fitted[ key ] = { h, w, scale[ i ] };
    }
    iv.compModelValue[ 1 ] = h;
    iv.compModelValue[ 2 ] = w;
  }
  scaleUnfittedLikeNeighbours( analyzerScale, hasFit, scale );
  for( int i = 0; i < n; i++ )
  {
    cm.intensityValues[ i ].compModelValue[ 0 ] = scale[ i ];
  }
}

// The lost-grain fit of the auto-regressive model (--fga 3): the filter stays the analyzer's (one per component; fitting
// its shape to the lost grain as well gained little when the fit compared 16x16 blocks: on Top Gun's sky the groups
// u + v 3-5 / 6-8 / 9-12 / 13-30 of reconstruction plus synthesis reached 0.22 / 0.38 / 1.39 / 1.21 of the original's
// power against 0.21 / 0.31 / 1.32 / 1.33), and every interval's scaling factor is fitted to the grain the coding lost as
// the eye sees it (FGAnalyzer::lostGrainFitAr): the fit to the pool, never above the fit to the pool's near pictures
// (nearPower; not the lowest fit of single pictures as in xFitLostGrain, whose steps are less than half this
// model's: LOST_GRAIN_POOL_SPAN), at most FGAnalyzer::LOST_GRAIN_CAP_POWER times the analyzer's grain power. Without
// it the whole analysed grain was synthesized on top of the grain the coding kept. As in xFitLostGrain, an interval
// needs MIN_POOLED_TILES pooled tiles to be fitted, an interval without a fit keeps its last value, and one that never
// had a fit follows its neighbours. A fit below the last value of the
// interval and temporal layer is taken at once: a limit on the way down held the grain far above what the fit asked
// for, and it did so for many pictures (gangster_chase: after one fit of 112 the fits fell to 84 and 48, the value only
// to 105 and 97, and that picture had 1.96 times the source's finest band between neighbours at 1.0). On the way up the
// scaling factor rises at most an eighth of its last value per picture of the layer (at least 1, at most s0 / 8), so
// that the grain does not pulse; the price is that a value near 0 rises slowly.
void EncGOP::xFitLostGrainAr( LostGrainState& state, int c, unsigned layer, int bitDepth, int log2ScaleFactor,
                              const std::vector<FGAnalyzer::LostGrainPower>& power,
                              const std::vector<FGAnalyzer::LostGrainPower>& nearPower,
                              const std::vector<FGAnalyzer::LostGrainBands>& bands, SeiFgc::CompModel& cm ) const
{
  if( cm.numIntensityIntervals == 0 )
  {
    return;
  }
  // the grain of the component's filter, computed anew only when the filter changed
  const std::vector<int>& first = cm.intensityValues[ 0 ].compModelValue;
  std::vector<int> filter( first.begin() + 1, first.end() );
  filter.insert( filter.begin(), log2ScaleFactor );
  if( state.arGrain[ c ].filter != filter )
  {
    const FGAnalyzer::ArGrain newGrain = FGAnalyzer::arGrain( first, log2ScaleFactor, bitDepth, c == COMP_Y );
    // the scaling factors last written are in units of the old filter's grain: keep their grain level, so that a new
    // analysis does not make the grain jump
    for( auto& fitted : state.fitted )
    {
      if( std::get<0>( fitted.first ) == c && !state.arGrain[ c ].filter.empty() && newGrain.stddevPerScale > 0.0 )
      {
        fitted.second.scale = (int) std::lround( fitted.second.scale * state.arGrain[ c ].grain.stddevPerScale / newGrain.stddevPerScale );
      }
    }
    state.arGrain[ c ].grain  = newGrain;
    state.arGrain[ c ].filter = filter;
  }
  const FGAnalyzer::ArGrain& grain = state.arGrain[ c ].grain;

  const int n = cm.numIntensityIntervals;
  std::vector<int>  analyzerScale( n ), scale( n );
  std::vector<char> hasFit( n );
  for( int i = 0; i < n; i++ )
  {
    const SeiFgc::CompModelIntensityValues& iv = cm.intensityValues[ i ];
    const int s0 = iv.compModelValue[ 0 ];   // the analyzer's
    const LostGrainKey key( c, iv.intensityIntervalLowerBound, iv.intensityIntervalUpperBound, layer );
    const auto last   = state.fitted.find( key );
    const bool fitted = last != state.fitted.end();
    analyzerScale[ i ] = s0;
    scale[ i ]         = fitted ? last->second.scale : s0;
    int  fit;
    const bool fits = power[ i ].tiles >= MIN_POOLED_TILES && FGAnalyzer::lostGrainFitAr( power[ i ], bands[ i ], grain, s0, fit ) >= 0;
    // the near pictures have a fit of their own with enough tiles, and fewer than the pool's (with as many, they are the pool)
    int  nearFit;
    if( fits && nearPower[ i ].tiles >= MIN_POOLED_TILES && nearPower[ i ].tiles < power[ i ].tiles
        && FGAnalyzer::lostGrainFitAr( nearPower[ i ], bands[ i ], grain, s0, nearFit ) >= 0 )
    {
      fit = std::min( fit, nearFit );
    }
    if( fits )
    {
      const int step = std::max( 1, std::min( s0, scale[ i ] ) / 8 );
      scale[ i ] = fitted ? std::min( fit, scale[ i ] + step ) : fit;   // the first fit is free
    }
    scale[ i ]  = std::min( scale[ i ], FGAnalyzer::lostGrainCapScaleAr( s0 ) );
    hasFit[ i ] = fits || fitted;
    if( hasFit[ i ] )
    {
      state.fitted[ key ] = { 0, 0, scale[ i ] };
    }
  }
  scaleUnfittedLikeNeighbours( analyzerScale, hasFit, scale );
  for( int i = 0; i < n; i++ )
  {
    cm.intensityValues[ i ].compModelValue[ 0 ] = scale[ i ];
  }
}

void EncGOP::xWriteLeadingSEIs( Picture& pic, AccessUnitList& accessUnit )
{
  const Slice* slice = pic.slices[ 0 ];
  SEIMessages leadingSeiMessages;

  bool bpPresentInAU = false;

  if((m_pcEncCfg->m_bufferingPeriodSEIEnabled) && (slice->isIRAP() || slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_GDR) &&
    slice->nuhLayerId==slice->vps->layerId[0] && (slice->sps->hrdParametersPresent) && m_pcRateCtrl->rcIsFinalPass && !m_isPreAnalysis )
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

  if( m_pcEncCfg->m_pictureTimingSEIEnabled && m_pcEncCfg->m_bufferingPeriodSEIEnabled && m_pcRateCtrl->rcIsFinalPass && !m_isPreAnalysis )
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

  // film grain SEI
  if ( m_pcEncCfg->m_fg.m_fgcSEIEnabled && !m_pcEncCfg->m_fg.m_fgcSEIPerPictureSEI )
  {
    // Access units are written in coding order. The results of the analyses issued with a picture (xAnalyzeFilmGrain:
    // the analysed pictures up to its POC, in POC order) are added with its access unit, each applying from its start
    // picture on (xAddFilmGrainParameters).
    while( !m_fgaTasks.empty() && m_fgaTasks.front()->codingNum <= pic.gopEntry->m_codingNum )
    {
      m_fgaTasks.front()->counter.wait();
      xAddFilmGrainParameters( *m_fgaTasks.front() );
      // the consumed task has run, so the task after it no longer looks at its barrier once that one has run too
      m_fgaConsumed = std::move( m_fgaTasks.front() );
      m_fgaTasks.pop_front();
    }

    SeiFgc sei = xFilmGrainSei( xFilmGrainParametersAt( slice->poc ), pic.chromaFormat );
    // With the improved analysis (--fga 2 and 3), every picture's message is scaled to the grain its coding lost, so every
    // picture carries one.
    const bool scaled = xFitsLostGrain();
    if( scaled )
    {
      xScaleToLostGrain( pic, *slice, sei );
    }
    if( xFilmGrainSeiNeeded( *slice, sei, pic.chromaFormat ) || scaled )
    {
      leadingSeiMessages.push_back( new SeiFgc( sei ) );
    }
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
    if ( m_pcEncCfg->m_decodedPictureHashSEIType < VVENC_HASHTYPE_MD5_LOG )
    {
    trailingSeiMessages.push_back( decodedPictureHashSei );
  }
    else
    {
      delete decodedPictureHashSei;
    }
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
                                  pic->picVA.visAct,
                                  uibits,
                                  dPSNR[COMP_Y],
                                  slice->isIntra(),
                                  slice->TLayer,
                                  pic->gopEntry->m_isStartOfIntra,
                                  pic->gopEntry->m_isStartOfGop,
                                  pic->gopEntry->m_gopNum,
                                  pic->gopEntry->m_scType,
                                  pic->picVA.spatAct[CH_L],
                                  pic->m_picShared->m_picMotEstError,
                                  pic->m_picShared->m_minNoiseLevels );
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

  // create info string
  {
    if ((m_isPreAnalysis && m_pcRateCtrl->m_pcEncCfg->m_RCTargetBitrate > 0) || !m_pcRateCtrl->rcIsFinalPass)
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

      std::string yPSNR = dPSNR[COMP_Y]  == MAX_DOUBLE ? prnt(" [Y %7s dB    ", "inf" ) : prnt(" [Y %6.4lf dB    ", dPSNR[COMP_Y] );
      std::string uPSNR = dPSNR[COMP_Cb] == MAX_DOUBLE ? prnt("U %7s dB    ", "inf" ) : prnt("U %6.4lf dB    ", dPSNR[COMP_Cb] );
      std::string vPSNR = dPSNR[COMP_Cr] == MAX_DOUBLE ? prnt("V %7s dB]", "inf" ) : prnt("V %6.4lf dB]", dPSNR[COMP_Cr] );

      accessUnit.InfoString.append( cInfo );
      accessUnit.InfoString.append( yPSNR );
      accessUnit.InfoString.append( uPSNR );
      accessUnit.InfoString.append( vPSNR );

      if ( m_pcEncCfg->m_printHexPsnr )
      {
        uint64_t xPsnr[MAX_NUM_COMP];
        for (int i = 0; i < MAX_NUM_COMP; i++)
        {
          std::copy(reinterpret_cast<uint8_t *>(&dPSNR[i]),
              reinterpret_cast<uint8_t *>(&dPSNR[i]) + sizeof(dPSNR[i]),
              reinterpret_cast<uint8_t *>(&xPsnr[i]));
        }

        std::string yPSNRHex = dPSNR[COMP_Y]  == MAX_DOUBLE ? prnt(" [xY %16s", "inf") : prnt(" [xY %16" PRIx64,  xPsnr[COMP_Y] );
        std::string uPSNRHex = dPSNR[COMP_Cb] == MAX_DOUBLE ? prnt(" xU %16s", "inf") : prnt(" xU %16" PRIx64, xPsnr[COMP_Cb] ) ;
        std::string vPSNRHex = dPSNR[COMP_Cr] == MAX_DOUBLE ? prnt(" xV %16s]", "inf") : prnt(" xV %16" PRIx64 "]", xPsnr[COMP_Cr]);

        accessUnit.InfoString.append( yPSNRHex );
        accessUnit.InfoString.append( uPSNRHex );
        accessUnit.InfoString.append( vPSNRHex );
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
      case VVENC_HASHTYPE_MD5_LOG:
        modeName = "MD5";
        break;
      case VVENC_HASHTYPE_CRC:
      case VVENC_HASHTYPE_CRC_LOG:
        modeName = "CRC";
        break;
      case VVENC_HASHTYPE_CHECKSUM:
      case VVENC_HASHTYPE_CHECKSUM_LOG:
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

  if( !accessUnit.InfoString.empty() && m_pcEncCfg->m_verbosity >= VVENC_NOTICE )
  {
    std::string cPicInfo = accessUnit.InfoString;
    cPicInfo.append("\n");
    const vvencMsgLevel msgLevel = m_isPreAnalysis ? VVENC_DETAILS : VVENC_NOTICE;
    msg.log( msgLevel, cPicInfo.c_str() );
    if( m_pcEncCfg->m_verbosity >= msgLevel ) fflush( stdout );
  }
}

void EncGOP::xForceScc( Picture& pic )
{
  if( pic.gopEntry->m_isStartOfGop )
  {
    m_forceSCC = pic.m_picShared->m_forceSCC;
  }
  if( m_forceSCC && (!pic.isSccStrong || !pic.isSccWeak) )
  {
    pic.isSccStrong = true;
    pic.isSccWeak = true;
    pic.setSccFlags(m_pcEncCfg);
  }
}

} // namespace vvenc

//! \}


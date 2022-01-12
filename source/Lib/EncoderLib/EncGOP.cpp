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


/** \file     EncGOP.cpp
    \brief    GOP encoder class
*/

#include "EncGOP.h"
#include "NALwrite.h"
#include "CommonLib/SEI.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/TimeProfiler.h"
#include "CommonLib/MD5.h"
#include "DecoderLib/DecLib.h"
#include "BitAllocation.h"
#include "EncHRD.h"
#include "Utilities/MsgLog.h"

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
  , m_pcRateCtrl         ( nullptr )
  , m_gopApsMap          ( MAX_NUM_APS * MAX_NUM_APS_TYPE )
  , m_spsMap             ( MAX_NUM_SPS )
  , m_ppsMap             ( MAX_NUM_PPS )
  , m_isPreAnalysis      ( false )
  , m_bFirstInit         ( true )
  , m_bFirstWrite        ( true )
  , m_bRefreshPending    ( false )
  , m_disableLMCSIP      ( false )
  , m_numPicsCoded       ( 0 )
  , m_pocEncode          ( -1 )
  , m_pocRecOut          ( 0 )
  , m_gopSizeLog2        ( -1 )
  , m_ticksPerFrameMul4  ( 0 )
  , m_codingOrderIdx     ( 0 )
  , m_lastIDR            ( 0 )
  , m_lastRasPoc         ( MAX_INT )
  , m_pocCRA             ( 0 )
  , m_appliedSwitchDQQ   ( 0 )
  , m_associatedIRAPPOC  ( 0 )
  , m_associatedIRAPType ( VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP )
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

  m_nextPocOffset.clear();
  m_pocToGopId.clear();

  // cleanup parameter sets
  m_spsMap.clearMap();
  m_ppsMap.clearMap();
}

void EncGOP::init( const VVEncCfg& encCfg, RateCtrl& rateCtrl, NoMallocThreadPool* threadPool, bool isPreAnalysis )
{
  m_pcEncCfg      = &encCfg;
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

  if( m_pcEncCfg->m_DecodingRefreshType == 4 )
  {
    m_associatedIRAPType = VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL;
  }
  m_seiEncoder.init( encCfg, m_EncHRD );
  m_Reshaper.init  ( encCfg );

  m_appliedSwitchDQQ = 0;
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

  int iOffset = -1;
  while( ( 1<< ( ++iOffset ) ) < m_pcEncCfg->m_GOPSize );
  m_gopSizeLog2 = iOffset;

  if( m_pcEncCfg->m_FrameRate )
  {
    m_ticksPerFrameMul4 = (int)((int64_t)4 *(int64_t)m_pcEncCfg->m_TicksPerSecond * (int64_t)m_pcEncCfg->m_FrameScale/(int64_t)m_pcEncCfg->m_FrameRate);
  }

  m_pocToGopId.resize( m_pcEncCfg->m_GOPSize, -1 );
  m_nextPocOffset.resize( m_pcEncCfg->m_GOPSize, 0 );

  int gopPOCadj = m_pcEncCfg->m_DecodingRefreshType == 4 ? 1 : 0;

  for ( int i = 0; i < m_pcEncCfg->m_GOPSize; i++ )
  {
    const int poc = (m_pcEncCfg->m_GOPList[ i ].m_POC-gopPOCadj) % m_pcEncCfg->m_GOPSize;
    CHECK( m_pcEncCfg->m_GOPList[ i ].m_POC > m_pcEncCfg->m_GOPSize, "error: poc greater than gop size" );
    CHECK( m_pocToGopId[ poc ] != -1, "error: multiple entries in gop list map to same poc modulo gop size" );
    m_pocToGopId[ poc ] = i;
    const int nextGopNum = ( i + 1 ) / m_pcEncCfg->m_GOPSize;
    const int nextGopId  = ( i + 1 ) % m_pcEncCfg->m_GOPSize;
    const int nextPoc    = nextGopNum * m_pcEncCfg->m_GOPSize + m_pcEncCfg->m_GOPList[ nextGopId ].m_POC-gopPOCadj;
    m_nextPocOffset[ poc ] = nextPoc - (m_pcEncCfg->m_GOPList[ i ].m_POC-gopPOCadj);
  }
  for ( int i = 0; i < m_pcEncCfg->m_GOPSize; i++ )
  {
    CHECK( m_pocToGopId[ i ] < 0 || m_nextPocOffset[ i ] == 0, "error: poc not found in gop list" );
  }
}

void EncGOP::picInitRateControl( Picture& pic, Slice* slice, EncPicture* picEncoder )
{
  int sliceQP   = MAX_QP;
  double lambda = m_pcRateCtrl->encRCSeq->maxEstLambda;

  m_pcRateCtrl->initRateControlPic (pic, slice, sliceQP, lambda);

  picEncoder->getEncSlice()->resetQP (&pic, sliceQP, lambda);
  m_pcRateCtrl->setFinalLambda (lambda);
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

  pic->gopId  = xGetGopIdFromPoc( pic->poc );
  pic->TLayer = m_pcEncCfg->m_GOPList[ pic->gopId ].m_temporalId;

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

//#define RC_LOOKAHEAD_PERIOD m_pcEncCfg->m_GOPSize
#define RC_LOOKAHEAD_PERIOD m_pcEncCfg->m_IntraPeriod

void EncGOP::processPictures( const PicList& picList, bool flush, AccessUnitList& auList, PicList& doneList, PicList& freeList )
{
  CHECK( picList.empty(), "empty input picture list given" );

  std::vector<Picture*> encList;
#if HIGH_LEVEL_MT_OPT
  // Rate Control with Look Ahead, two variants:
  // - Chunk-wise 2.Pass: encode all pictures from pre-coded Look-Ahead chunk (GOPs) using their data only
  // - GOP-wise sliding window final pass processing using data of future pre-coded GOPs from Look-Ahead
  // NOTE: In second pass, picList contains pictures coming from Look-Ahead (1.pass)
#if MT_RC_LA_GOP_SW
  if( !isNonBlocking() || flush ||  ( m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_RCLookAhead && ( ( ( m_picCount - 1 ) / m_pcEncCfg->m_GOPSize ) > m_lookAheadGOPCnt ) ) )
#else
  if( !isNonBlocking() || flush || ( m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_RCLookAhead && ( m_picCount - 1 ) % RC_LOOKAHEAD_PERIOD == 0 ) )
#endif
  {
#endif
  // create list of pictures ordered in coding order and ready to be encoded
  xCreateCodingOrder( picList, flush, encList );
#if HIGH_LEVEL_MT_OPT
#if DEBUG_PRINT
  if( !encList.empty() && isNonBlocking() )
  {
    DPRINT( "#%d NumPicRecvd=%d PicCount=%d\n", stageId(), m_numPicsRecvd, m_picCount );
    DPRINT( "#%d    ", stageId() );
    debug_print_pic_list_vector( encList, " encList" );
  }
#endif
  _CASE( m_picCount == 65 && isNonBlocking() )
    _BREAK;

  if( !encList.empty() )
  {
#else
#if DEBUG_PRINT
  if( !encList.empty() )
  {
    DPRINT( "#%d    ", stageId() );
    debug_print_pic_list_vector( encList, " encList" );
  }
#endif
#endif
  CHECK( encList.empty(), "no pictures to be encoded found" );
#if HIGH_LEVEL_MT_OPT
#if MT_RC_LA_GOP_SW
  if( m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_RCLookAhead && ( flush || (int)encList.front()->poc % m_pcEncCfg->m_GOPSize == 0 ) )
#else
  if( m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_RCLookAhead && ( flush || ( ( m_picCount - 1 ) / RC_LOOKAHEAD_PERIOD ) > m_lookAheadGOPCnt ) )
#endif
  {
#if MT_RC_LA_GOP_SW
    m_lookAheadGOPCnt += m_lookAheadGOPCnt == 0 ? 2: 1;
#else
    m_lookAheadGOPCnt++;
#endif
    m_pcRateCtrl->processFirstPassData( flush );
  }
#else
  // init rate control GOP
  if( m_pcEncCfg->m_RCTargetBitrate > 0 )
  {
    CHECK( m_isPreAnalysis, "rate control enabled for pre analysis" );
    if( m_numPicsCoded == 0 )
    {
      if ( m_pcEncCfg->m_RCLookAhead )
      {
        m_pcRateCtrl->processFirstPassData( flush );
      }
      // very first RC GOP
      m_pcRateCtrl->initRCGOP( 1 );
    }
    else if( 1 == m_numPicsCoded % m_pcEncCfg->m_GOPSize )
    {
      if ( m_pcEncCfg->m_RCLookAhead && encList.front()->poc % RC_LOOKAHEAD_PERIOD == 0 )
      {
        m_pcRateCtrl->processFirstPassData( flush );
      }
      m_pcRateCtrl->destroyRCGOP();
      const int rcGopSize = flush ? std::min( m_pcEncCfg->m_GOPSize, (int)encList.size() ) : m_pcEncCfg->m_GOPSize;
      m_pcRateCtrl->initRCGOP( rcGopSize );
    }
  }
#endif

  // encode pictures
  xInitPicsInCodingOrder( encList, picList, false );
#if HIGH_LEVEL_MT_OPT
  }
  CHECK( m_gopEncListInput.empty() && m_gopEncListOutput.empty() && m_procList.empty() , "running with empty lists into encoder" );
#endif
#if HIGH_LEVEL_MT_OPT
  } // non-blocking rc access period
#endif

#if HIGH_LEVEL_MT_OPT
  if( isNonBlocking() )
    xEncodePicturesNonBlocking( flush, auList, doneList );
  else
#endif
  xEncodePictures( flush, auList, doneList );

#if HIGH_LEVEL_MT_OPT
  if( !isNonBlocking() ){
#endif
  CHECK( doneList.empty(), "no picture encoded" );
#if HIGH_LEVEL_MT_OPT
  }
#endif

  // output reconstructed YUV
  xOutputRecYuv( picList );

  // release pictures not needed andmore
#if HIGH_LEVEL_MT_OPT
  const bool allDone = flush && ( isNonBlocking() ? m_gopEncListInput.empty() && m_gopEncListOutput.empty() && m_procList.empty(): encList.back()->poc == doneList.back()->poc );
#else
  const bool allDone = flush && encList.back()->poc == doneList.back()->poc;
#endif
  xReleasePictures( picList, freeList, allDone );

  // clear output access unit
  if( m_isPreAnalysis )
  {
    auList.clearAu();
  }
}

#if HIGH_LEVEL_MT_OPT
void EncGOP::checkState()
{
  //if( !m_procList.empty() || !m_gopEncListOutput.empty() )
  {
    std::unique_lock<std::mutex> lock( m_gopEncMutex );
    bool rcPicOnTheFly = !m_rcUpdateList.empty() && ( (int)m_freePicEncoderList.size() < m_pcEncCfg->m_maxParallelFrames ); 
    if( !m_procList.empty() || rcPicOnTheFly || m_freePicEncoderList.empty() )
    {
      bool nextPicReady = true;
      if( !m_procList.empty() )
      {
        auto picItr = find_if( m_procList.begin(), m_procList.end(), []( auto pic ) { return pic->slices[0]->checkRefPicsReconstructed(); } );
        nextPicReady = picItr != m_procList.end();
      }
      if( m_freePicEncoderList.empty() || rcPicOnTheFly || !nextPicReady )
      {
        CHECK( m_pcEncCfg->m_numThreads <= 0, "run into MT code, but no threading enabled" );
        CHECK( (int)m_freePicEncoderList.size() >= std::max( 1, m_pcEncCfg->m_maxParallelFrames ), "wait for picture to be finished, but no pic encoder running" );
        m_gopEncCond.wait( lock );
      }
    }
  }
}

void EncGOP::xEncodePicturesNonBlocking( bool flush, AccessUnitList& auList, PicList& doneList )
{
  // in lockstep mode, process all pictures in processing list
  const bool lockStepMode = m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_maxParallelFrames > 0;

#if HIGH_LEVEL_MT_OPT
  if( isNonBlocking() && lockStepMode && m_procList.empty() && (int)m_freePicEncoderList.size() < m_pcEncCfg->m_maxParallelFrames && !m_gopEncListOutput.front()->isReconstructed )
  {
    // Non-blocking: Encoder is still busy, wait outside
    return;
  }

  // get list of pictures to be encoded and used for RC update
  if( m_procList.empty() /*&& m_rcUpdateList.empty()*/ )
#else
  // get list of pictures to be encoded and used for RC update
  std::list<Picture*> m_procList;
  std::list<Picture*> m_rcUpdateList;
#endif
  xGetProcessingListsNonBlocking( m_procList, m_rcUpdateList );

#if DEBUG_PRINT
  if( !m_procList.empty() )
  {
    DPRINT( "#%d    ", stageId() );
    debug_print_pic_list( m_procList, "procList" );
  }
#endif

#if HIGH_LEVEL_MT_OPT
  // Non-blocking: If nothing to do, wait outside
  if( m_procList.empty() && !m_gopEncListOutput.front()->isReconstructed )
    return;
#endif

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

      // in non-lockstep mode, check encoding of output picture done
      // in lockstep mode, check all pictures encoded
      if( ( ! lockStepMode && ( m_gopEncListOutput.empty() || m_gopEncListOutput.front()->isReconstructed ) )
          || ( lockStepMode && m_procList.empty() && (int)m_freePicEncoderList.size() >= m_pcEncCfg->m_maxParallelFrames ) )
      {
        break;
      }

      // get next picture ready to be encoded
      auto picItr             = find_if( m_procList.begin(), m_procList.end(), []( auto pic ) { return pic->slices[ 0 ]->checkRefPicsReconstructed(); } );
      const bool nextPicReady = picItr != m_procList.end();

      // check at least one picture and one pic encoder ready
      if( m_freePicEncoderList.empty()
          || ! nextPicReady )
      {
#if HIGH_LEVEL_MT_OPT
        if( isNonBlocking() )
          return;
#endif
        CHECK( m_pcEncCfg->m_numThreads <= 0, "run into MT code, but no threading enabled" );
        CHECK( (int)m_freePicEncoderList.size() >= std::max( 1, m_pcEncCfg->m_maxParallelFrames ), "wait for picture to be finished, but no pic encoder running" );
        m_gopEncCond.wait( lock );
        continue;
      }

      pic        = *picItr;
      picEncoder = m_freePicEncoderList.front();
      m_freePicEncoderList.pop_front();
    }

    CHECK( picEncoder == nullptr, "no free picture encoder available" );
    CHECK( pic        == nullptr, "no picture to be encoded, ready for encoding" );

// #if HIGH_LEVEL_MT_OPT
// #if MT_RC_LA_GOP_SW
//   if( m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_RCLookAhead && pic->poc % m_pcEncCfg->m_GOPSize == 0 )
// #else
//   if( m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_RCLookAhead && pic->poc % RC_LOOKAHEAD_PERIOD == 0 )
// #endif
//   {
//     //m_lookAheadGOPCnt++;
//     m_pcRateCtrl->processFirstPassData( flush );
//   }
// #endif
    // picture will be encoded -> remove from input list
#if !HIGH_LEVEL_MT_OPT
    m_gopEncListInput.remove( pic );
#endif
    m_procList.remove( pic );

    // decoder in encoder
    bool decPic = false;
    bool encPic = false;
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "encdec", 1 ) );
    trySkipOrDecodePicture( decPic, encPic, *m_pcEncCfg, pic, m_ffwdDecoder, m_gopApsMap, msg );
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "encdec", 0 ) );
    pic->writePic = decPic || encPic;
    pic->encPic   = encPic;

    if( m_pcEncCfg->m_alfTempPred )
    {
      xSyncAlfAps( *pic, pic->picApsMap, m_gopApsMap );
    }
#if DEBUG_PRINT
    DPRINT( "#%d           compress: %2d <t%2d>\n", stageId(), pic->getPOC(), pic->TLayer );
#endif
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

  CHECK( m_gopEncListOutput.empty(),                    "try to output picture, but no output picture available" );
  CHECK( ! m_gopEncListOutput.front()->isReconstructed, "try to output picture, but picture not reconstructed" );
  PROFILER_ACCUM_AND_START_NEW_SET( 1, g_timeProfiler, P_TOP_LEVEL );
#if HIGH_LEVEL_MT_OPT
  _CASE( isNonBlocking() )
    _BREAK;
#endif
  // AU output
  Picture* outPic = m_gopEncListOutput.front();
  m_gopEncListOutput.pop_front();
#if DEBUG_PRINT
    DPRINT( "#%d             output:               %2d <t%2d>\n", stageId(), outPic->getPOC(), outPic->TLayer );
#endif

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
  for( auto pic : m_rcUpdateList )
  {
    if( pic != outPic )
    {
      pic->actualHeadBits  = outPic->actualHeadBits;
      pic->actualTotalBits = pic->sliceDataStreams[0].getNumberOfWrittenBits();
    }
    if( m_pcEncCfg->m_RCTargetBitrate > 0 )
    {
      m_pcRateCtrl->xUpdateAfterPicRC( pic );
    }
  }
#if HIGH_LEVEL_MT_OPT
  if( !m_rcUpdateList.empty() )
  {
    if( lockStepMode )
      m_rcUpdateList.clear();
    else
      m_rcUpdateList.pop_front();
  }
#endif

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
#if !HIGH_LEVEL_MT_OPT
  m_pocEncode     = outPic->poc;
#endif
  m_numPicsCoded += 1;
}
#endif

void EncGOP::xOutputRecYuv( const PicList& picList )
{
  if( m_pcRateCtrl->rcIsFinalPass && m_recYuvBufFunc )
  {
    CHECK( m_isPreAnalysis, "yuv output enabled for pre analysis" );
    // ordered YUV output
    for( auto pic : picList )
    {
      if( pic->poc < m_pocRecOut )
        continue;
      if( ! pic->isReconstructed || pic->poc != m_pocRecOut )
        return;

      const PPS& pps = *(pic->cs->pps);
      vvencYUVBuffer yuvBuffer;
      vvenc_YUVBuffer_default( &yuvBuffer );
      setupYuvBuffer( pic->getRecoBuf(), yuvBuffer, &pps.conformanceWindow );
      m_recYuvBufFunc( m_recYuvBufCtx, &yuvBuffer );

      m_pocRecOut += 1;
      pic->isNeededForOutput = false;
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

void EncGOP::xEncodePictures( bool flush, AccessUnitList& auList, PicList& doneList )
{
  // get list of pictures to be encoded and used for RC update
  std::list<Picture*> procList;
  std::list<Picture*> rcUpdateList;
  xGetProcessingLists( procList, rcUpdateList );
#if DEBUG_PRINT
  if( !procList.empty() )
  {
    DPRINT( "#%d    ", stageId() );
    debug_print_pic_list( procList, "procList" );
  }
#endif
  // in lockstep mode, process all pictures in processing list
  const bool lockStepMode = m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_maxParallelFrames > 0;

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

      // in non-lockstep mode, check encoding of output picture done
      // in lockstep mode, check all pictures encoded
      if( ( ! lockStepMode && ( m_gopEncListOutput.empty() || m_gopEncListOutput.front()->isReconstructed ) )
          || ( lockStepMode && procList.empty() && (int)m_freePicEncoderList.size() >= m_pcEncCfg->m_maxParallelFrames ) )
      {
        break;
      }

      // get next picture ready to be encoded
      auto picItr             = find_if( procList.begin(), procList.end(), []( auto pic ) { return pic->slices[ 0 ]->checkRefPicsReconstructed(); } );
      const bool nextPicReady = picItr != procList.end();

      // check at least one picture and one pic encoder ready
      if( m_freePicEncoderList.empty()
          || ! nextPicReady )
      {
        CHECK( m_pcEncCfg->m_numThreads <= 0, "run into MT code, but no threading enabled" );
        CHECK( (int)m_freePicEncoderList.size() >= std::max( 1, m_pcEncCfg->m_maxParallelFrames ), "wait for picture to be finished, but no pic encoder running" );
        m_gopEncCond.wait( lock );
        continue;
      }

      pic        = *picItr;
      picEncoder = m_freePicEncoderList.front();
      m_freePicEncoderList.pop_front();
    }

    CHECK( picEncoder == nullptr, "no free picture encoder available" );
    CHECK( pic        == nullptr, "no picture to be encoded, ready for encoding" );

    // picture will be encoded -> remove from input list
    m_gopEncListInput.remove( pic );
    procList.remove( pic );

    // decoder in encoder
    bool decPic = false;
    bool encPic = false;
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "encdec", 1 ) );
    trySkipOrDecodePicture( decPic, encPic, *m_pcEncCfg, pic, m_ffwdDecoder, m_gopApsMap, msg );
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "encdec", 0 ) );
    pic->writePic = decPic || encPic;
    pic->encPic   = encPic;

    if( m_pcEncCfg->m_alfTempPred )
    {
      xSyncAlfAps( *pic, pic->picApsMap, m_gopApsMap );
    }
#if DEBUG_PRINT
    DPRINT( "#%d           compress: %2d <t%2d>\n", stageId(), pic->getPOC(), pic->TLayer );
#endif

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

  CHECK( m_gopEncListOutput.empty(),                    "try to output picture, but no output picture available" );
  CHECK( ! m_gopEncListOutput.front()->isReconstructed, "try to output picture, but picture not reconstructed" );
  PROFILER_ACCUM_AND_START_NEW_SET( 1, g_timeProfiler, P_TOP_LEVEL );

  // AU output
  Picture* outPic = m_gopEncListOutput.front();
  m_gopEncListOutput.pop_front();
#if DEBUG_PRINT
    DPRINT( "#%d             output:               %2d <t%2d>\n", stageId(), outPic->getPOC(), outPic->TLayer );
#endif

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
  for( auto pic : rcUpdateList )
  {
    if( pic != outPic )
    {
      pic->actualHeadBits  = outPic->actualHeadBits;
      pic->actualTotalBits = pic->sliceDataStreams[0].getNumberOfWrittenBits();
    }
    if( m_pcEncCfg->m_RCTargetBitrate > 0 )
    {
      m_pcRateCtrl->xUpdateAfterPicRC( pic );
    }
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

  m_pocEncode     = outPic->poc;
  m_numPicsCoded += 1;
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

int EncGOP::xGetNextPocICO( int poc, int max, bool altGOP ) const
{
  if( poc < 0 )
  {
    int first = 0;
    if( altGOP )
    {
      first = m_pcEncCfg->m_GOPSize;
      while( first > max )
      {
        first >>= 1;
      }
      first -= 1;
      CHECK( first < 0, "get next poc failed" );
    }
    return first;
  }

  int next = ( poc == 0 && ! altGOP ) ? m_pcEncCfg->m_GOPList[ 0 ].m_POC : poc + m_nextPocOffset[ poc % m_pcEncCfg->m_GOPSize ];

  return next;
}

Picture* EncGOP::xFindPicture( const PicList& picList, int poc ) const
{
  for( auto& picItr : picList )
  {
    if( picItr->poc == poc )
    {
      return picItr;
    }
  }
  return nullptr;
}

void EncGOP::xCreateCodingOrder( const PicList& picList, bool flush, std::vector<Picture*>& encList ) /*const*/
{
  const bool altGOP = m_pcEncCfg->m_DecodingRefreshType == 4;
  const int  max    = picList.back()->poc;
  int poc           = m_pocEncode;
  int chk           = 0;
  while( chk < m_pcEncCfg->m_GOPSize && encList.size() < picList.size() )
  {
    poc = xGetNextPocICO( poc, max, altGOP );
    if( poc > max && flush )
    {
      chk += 1;
      continue;
    }

    Picture* pic = xFindPicture( picList, poc );
#if DEBUG_PRINT
    if( !pic && encList.size() == 0 )
    {
      DPRINT( "\nNext pic to encode not found %d\n", poc );
    }
#endif
    if( ! pic )
      break;
    encList.push_back( pic );
#if HIGH_LEVEL_MT_OPT
    m_numPicsRecvd++;
#if MT_RC_LA_GOP_SW
    // GOP-wise sliding processing window with Look-Ahead Period: Process only one GOP using data from LA
    if( m_pcEncCfg->m_RCTargetBitrate > 0 && encList.size() >= m_pcEncCfg->m_GOPSize + ( m_lookAheadGOPCnt == 0 ) ? 1: 0 )
      break;
#endif
#endif
  }
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
  bool hasNonZeroTemporalId = false;
  bool hasLeadingPictures   = false;
  for (unsigned int i = 0; i < m_pcEncCfg->m_GOPSize; i++)
  {
    if ( m_pcEncCfg->m_GOPList[i].m_temporalId != 0 )
    {
      hasNonZeroTemporalId = true;
    }
    for( int l = 0; l < 2; l++ )
    {
      for ( unsigned int j = 0; !hasLeadingPictures && j < m_pcEncCfg->m_GOPList[i].m_numRefPics[l]; j++)
      {
        if ( m_pcEncCfg->m_GOPList[i].m_deltaRefPics[l][j] < 0 )
        {
          hasLeadingPictures = true;
        }
      }
    }
  }

  ci.intraOnlyConstraintFlag                      = m_pcEncCfg->m_intraOnlyConstraintFlag;
  ci.maxBitDepthConstraintIdc                     = m_pcEncCfg->m_bitDepthConstraintValue - 8;
  ci.maxChromaFormatConstraintIdc                 = m_pcEncCfg->m_internChromaFormat;
  ci.onePictureOnlyConstraintFlag             = false;
  ci.lowerBitRateConstraintFlag               = false;
  ci.allLayersIndependentConstraintFlag       = false;
  ci.noMrlConstraintFlag                      = false;
  ci.noIspConstraintFlag                      = false;
  ci.noMipConstraintFlag                      = false;
  ci.noLfnstConstraintFlag                    = false;
  ci.noMmvdConstraintFlag                     = false;
  ci.noSmvdConstraintFlag                     = false;
  ci.noProfConstraintFlag                     = false;
  ci.noPaletteConstraintFlag                  = false;
  ci.noActConstraintFlag                      = false;
  ci.noLmcsConstraintFlag                     = false;
  ci.noQtbttDualTreeIntraConstraintFlag           = ! m_pcEncCfg->m_dualITree;
  ci.noPartitionConstraintsOverrideConstraintFlag = m_pcEncCfg->m_useAMaxBT == 0;
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
  ci.noStsaConstraintFlag                         = m_pcEncCfg->m_IntraPeriod == 1 || !hasNonZeroTemporalId;
  ci.noRaslConstraintFlag                         = m_pcEncCfg->m_IntraPeriod == 1 || !hasLeadingPictures;
  ci.noRadlConstraintFlag                         = m_pcEncCfg->m_IntraPeriod == 1 || !hasLeadingPictures;
  ci.noIdrConstraintFlag                          = false;
  ci.noCraConstraintFlag                          = (m_pcEncCfg->m_DecodingRefreshType != 1 && m_pcEncCfg->m_DecodingRefreshType != 5);
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
  sps.maxMTTDepth[1]                = m_pcEncCfg->m_maxMTTDepth;
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
  sps.partitionOverrideEnabled      = m_pcEncCfg->m_useAMaxBT != 0;
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
  sps.maxTLayers                    = m_pcEncCfg->m_maxTempLayer;
  sps.rpl1CopyFromRpl0              = m_pcEncCfg->m_IntraPeriod < 0;
  sps.SbtMvp                        = m_pcEncCfg->m_SbTMVP;
  sps.CIIP                          = m_pcEncCfg->m_CIIP != 0;
  sps.SBT                           = m_pcEncCfg->m_SBT != 0;

  for (int i = 0; i < std::min(sps.maxTLayers, (uint32_t) VVENC_MAX_TLAYER); i++ )
  {
    sps.maxDecPicBuffering[i]       = m_pcEncCfg->m_maxDecPicBuffering[i];
    sps.numReorderPics[i]           = m_pcEncCfg->m_maxNumReorderPics[i];
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
    bChromaDeltaQPEnabled |= (m_pcEncCfg->m_usePerceptQPA || m_pcEncCfg->m_RCLookAhead || m_pcEncCfg->m_sliceChromaQpOffsetPeriodicity > 0) && (m_pcEncCfg->m_internChromaFormat != VVENC_CHROMA_400);
    if ( !bChromaDeltaQPEnabled && sps.dualITree && ( m_pcEncCfg->m_internChromaFormat != VVENC_CHROMA_400) )
    {
      bChromaDeltaQPEnabled = (m_pcEncCfg->m_chromaCbQpOffsetDualTree != 0 || m_pcEncCfg->m_chromaCrQpOffsetDualTree != 0 || m_pcEncCfg->m_chromaCbCrQpOffsetDualTree != 0);
    }

    for( int i = 0; !bChromaDeltaQPEnabled && i < m_pcEncCfg->m_GOPSize; i++ )
    {
      if( m_pcEncCfg->m_GOPList[ i ].m_CbQPoffset || m_pcEncCfg->m_GOPList[ i ].m_CrQPoffset )
      {
        bChromaDeltaQPEnabled = true;
      }
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

  int histogram[MAX_NUM_REF + 1];
  for( int i = 0; i <= MAX_NUM_REF; i++ )
  {
    histogram[i]=0;
  }
  for( int i = 0; i < m_pcEncCfg->m_GOPSize; i++)
  {
    CHECK(!(m_pcEncCfg->m_RPLList0[ i ].m_numRefPicsActive >= 0 && m_pcEncCfg->m_RPLList0[ i ].m_numRefPicsActive <= MAX_NUM_REF), "Unspecified error");
    histogram[m_pcEncCfg->m_RPLList0[ i ].m_numRefPicsActive]++;
  }

  int maxHist=-1;
  int bestPos=0;
  for( int i = 0; i <= MAX_NUM_REF; i++ )
  {
    if(histogram[i]>maxHist)
    {
      maxHist=histogram[i];
      bestPos=i;
    }
  }
  CHECK(!(bestPos <= 15), "Unspecified error");
  pps.numRefIdxL0DefaultActive = bestPos;
  pps.numRefIdxL1DefaultActive = bestPos;

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
  const int numRPLCandidates = m_pcEncCfg->m_numRPLList0;
  sps.rplList[0].resize(numRPLCandidates+1);
  sps.rplList[1].resize(numRPLCandidates+1);
  sps.rpl1IdxPresent = (sps.rplList[0].size() != sps.rplList[1].size());

  for (int i = 0; i < 2; i++)
  {
    const vvencRPLEntry* rplCfg = ( i == 0 ) ? m_pcEncCfg->m_RPLList0 : m_pcEncCfg->m_RPLList1;
    for (int j = 0; j < numRPLCandidates; j++)
    {
      const vvencRPLEntry &ge = rplCfg[ j ];
      ReferencePictureList&rpl = sps.rplList[i][j];
      rpl.numberOfShorttermPictures = ge.m_numRefPics;
      rpl.numberOfLongtermPictures = 0;   //Hardcoded as 0 for now. need to update this when implementing LTRP
      rpl.numberOfActivePictures = ge.m_numRefPicsActive;

      for (int k = 0; k < ge.m_numRefPics; k++)
      {
        rpl.setRefPicIdentifier(k, -ge.m_deltaRefPics[k], 0, false, 0);
      }
    }
  }

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

  bool isRpl1CopiedFromRpl0 = true;
  for( int i = 0; isRpl1CopiedFromRpl0 && i < numRPLCandidates; i++)
  {
    if( sps.rplList[0][i].getNumRefEntries() == sps.rplList[1][i].getNumRefEntries() )
    {
      for( int j = 0; isRpl1CopiedFromRpl0 && j < sps.rplList[0][i].getNumRefEntries(); j++ )
      {
        if( sps.rplList[0][i].refPicIdentifier[j] != sps.rplList[1][i].refPicIdentifier[j] )
        {
          isRpl1CopiedFromRpl0 = false;
        }
      }
    }
    else
    {
      isRpl1CopiedFromRpl0 = false;
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
 * \param pocCurr POC of the current picture
 * \param lastIDR  POC of the last IDR picture
 * \returns the NAL unit type of the picture
 * This function checks the configuration and returns the appropriate nal_unit_type for the picture.
 */
vvencNalUnitType EncGOP::xGetNalUnitType( int pocCurr, int lastIDR ) const
{
  if (pocCurr == 0)
  {
    if( m_pcEncCfg->m_DecodingRefreshType == 4 )
    {
      return m_bFirstInit ? VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL :  VVENC_NAL_UNIT_CODED_SLICE_RADL;
    }
    else
    {
      return VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP;
    }
  }

  if (m_pcEncCfg->m_DecodingRefreshType == 4 && ((pocCurr+1+(m_pcEncCfg->m_IntraPeriod - m_pcEncCfg->m_GOPSize)) % m_pcEncCfg->m_IntraPeriod == 0 || m_bFirstInit ) )
  {
    return VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL;
  }
  if ((m_pcEncCfg->m_DecodingRefreshType < 3 || m_pcEncCfg->m_DecodingRefreshType == 5) && pocCurr % m_pcEncCfg->m_IntraPeriod == 0)
  {
    if (m_pcEncCfg->m_DecodingRefreshType == 1 || m_pcEncCfg->m_DecodingRefreshType == 5)
    {
      return VVENC_NAL_UNIT_CODED_SLICE_CRA;
    }
    else if (m_pcEncCfg->m_DecodingRefreshType == 2)
    {
      return VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL;
    }
  }
  if(m_pocCRA>0)
  {
    if(pocCurr<m_pocCRA)
    {
      // All leading pictures are being marked as TFD pictures here since current encoder uses all
      // reference pictures while encoding leading pictures. An encoder can ensure that a leading
      // picture can be still decodable when random accessing to a CRA/CRANT/BLA/BLANT picture by
      // controlling the reference pictures used for encoding that leading picture. Such a leading
      // picture need not be marked as a TFD picture.
      return VVENC_NAL_UNIT_CODED_SLICE_RASL;
    }
  }
  if (lastIDR>0)
  {
    if (pocCurr < lastIDR)
    {
      return VVENC_NAL_UNIT_CODED_SLICE_RADL;
    }
  }
  return VVENC_NAL_UNIT_CODED_SLICE_TRAIL;
}


int EncGOP::xGetSliceDepth( int poc ) const
{
  int depth = 0;

  poc = poc % m_pcEncCfg->m_GOPSize;

  if ( poc != 0 )
  {
    int step = m_pcEncCfg->m_GOPSize;
    for( int i=step>>1; i>=1; i>>=1 )
    {
      for ( int j = i; j<m_pcEncCfg->m_GOPSize; j += step )
      {
        if ( j == poc )
        {
          i=0;
          break;
        }
      }
      step >>= 1;
      depth++;
    }
  }

  return depth;
}

bool EncGOP::xIsSliceTemporalSwitchingPoint( const Slice* slice, const PicList& picList, int gopId ) const
{
  if ( slice->TLayer > 0
      && !(slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RADL     // Check if not a leading picture
        || slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RASL) )
  {
    if ( slice->isStepwiseTemporalLayerSwitchingPointCandidate( picList ) )
    {
      const vvencRPLEntry* rplList0 = m_pcEncCfg->m_RPLList0;
      const vvencRPLEntry* rplList1 = m_pcEncCfg->m_RPLList1;
      bool isSTSA              = true;

      for( int ii = gopId + 1; ii < m_pcEncCfg->m_GOPSize && isSTSA == true; ii++ )
      {
        int lTid = rplList0[ ii ].m_temporalId;

        if ( lTid == slice->TLayer )
        {
          const ReferencePictureList* rpl0 = &slice->sps->rplList[0][ii];
          for ( int jj = 0; jj < slice->rpl[0]->numberOfActivePictures; jj++ )
          {
            int tPoc = rplList0[ ii ].m_POC + rpl0->refPicIdentifier[ jj ];
            int kk   = 0;
            for ( kk = 0; kk < m_pcEncCfg->m_GOPSize; kk++ )
            {
              if ( rplList0[ kk ].m_POC == tPoc )
              {
                break;
              }
            }
            int tTid = rplList0[ kk ].m_temporalId;
            if ( tTid >= slice->TLayer )
            {
              isSTSA = false;
              break;
            }
          }
          const ReferencePictureList* rpl1 = &slice->sps->rplList[1][ii];
          for ( int jj = 0; jj < slice->rpl[1]->numberOfActivePictures; jj++ )
          {
            int tPoc = rplList1[ ii ].m_POC + rpl1->refPicIdentifier[ jj ];
            int kk   = 0;
            for ( kk = 0; kk < m_pcEncCfg->m_GOPSize; kk++ )
            {
              if ( rplList1[ kk ].m_POC == tPoc )
              {
                break;
              }
            }
            int tTid = rplList1[ kk ].m_temporalId;
            if ( tTid >= slice->TLayer )
            {
              isSTSA = false;
              break;
            }
          }
        }
      }

      if ( isSTSA == true )
      {
        return true;
      }
    }
  }
  return false;
}

void EncGOP::xInitPicsInCodingOrder( const std::vector<Picture*>& encList, const PicList& picList, bool isEncodeLtRef )
{
  const size_t size = m_pcEncCfg->m_maxParallelFrames > 0 ? encList.size() : 1;
  for( int i = 0; i < size; i++ )
  {
    Picture* pic = encList[ i ];
    if ( ! pic->isInitDone )
    {
      pic->encTime.startTimer();

      xInitFirstSlice( *pic, picList, isEncodeLtRef );

      pic->encTime.stopTimer();

      m_gopEncListInput.push_back( pic );
      m_gopEncListOutput.push_back( pic );
#if HIGH_LEVEL_MT_OPT
      m_pocEncode     = pic->poc;
#endif
    }
  }
}

#if HIGH_LEVEL_MT_OPT
void EncGOP::xGetProcessingListsNonBlocking( std::list<Picture*>& procList, std::list<Picture*>& rcUpdateList )
{
  // in lockstep mode, process only pics of same temporal layer
  const bool lockStepMode = m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_maxParallelFrames > 0;
  if( lockStepMode )
  {
    // start new parallel chunk only, if next output picture is not reconstructed
    if( m_gopEncListOutput.empty() || ! m_gopEncListOutput.front()->isReconstructed )
    {
      const int procTL         = m_gopEncListInput.size() ? m_gopEncListInput.front()->TLayer                      : -1;
      const int gopId          = m_gopEncListInput.size() ? m_gopEncListInput.front()->poc / m_pcEncCfg->m_GOPSize : -1;
      const int rcIdxInGop     = m_gopEncListInput.size() ? m_gopEncListInput.front()->rcIdxInGop                  : -1;
      const int minSerialDepth = m_pcEncCfg->m_maxParallelFrames > 2 ? 1 : 2;  // up to this temporal layer encode pictures only in serial mode
      const int maxSize        = procTL <= minSerialDepth ? 1 : m_pcEncCfg->m_maxParallelFrames;
#if HIGH_LEVEL_MT_OPT
      for( auto it = m_gopEncListInput.begin(); it != m_gopEncListInput.end(); )
      {
        auto pic = *it;
        if( pic->poc / m_pcEncCfg->m_GOPSize == gopId
            && pic->TLayer == procTL
            && pic->slices[ 0 ]->checkRefPicsReconstructed() 
          )
        {
          procList.push_back    ( pic );
          rcUpdateList.push_back( pic );
          // map all pics in a parallel chunk to the same index, improves RC performance
          pic->rcIdxInGop = rcIdxInGop;
          it = m_gopEncListInput.erase( it );
        }
        else
        {
          ++it;
        }
        if( (int)procList.size() >= maxSize )
          break;
      }
#else
      for( auto pic : m_gopEncListInput )
      {
        if( pic->poc / m_pcEncCfg->m_GOPSize == gopId
            && pic->TLayer == procTL
            && pic->slices[ 0 ]->checkRefPicsReconstructed() )
        {
          procList.push_back    ( pic );
          rcUpdateList.push_back( pic );
          // map all pics in a parallel chunk to the same index, improves RC performance
          pic->rcIdxInGop = rcIdxInGop;
        }
        if( (int)procList.size() >= maxSize )
          break;
      }
#endif
    }
  }
  else
  {
#if HIGH_LEVEL_MT_OPT
    procList.splice( procList.end(), m_gopEncListInput );
    m_gopEncListInput.clear();
#else
    procList = m_gopEncListInput;
#endif
    // TODO: Is it correct here, to put just one picture into update list?
    if( ! m_gopEncListOutput.empty() )
      rcUpdateList.push_back( m_gopEncListOutput.front() );
  }
  CHECK( ! rcUpdateList.empty() && m_gopEncListOutput.empty(),                                                         "first picture in RC update and in output list have to be the same" );
  CHECK( ! rcUpdateList.empty() && ! m_gopEncListOutput.empty() && rcUpdateList.front() != m_gopEncListOutput.front(), "first picture in RC update and in output list have to be the same" );
}
#endif

void EncGOP::xGetProcessingLists( std::list<Picture*>& procList, std::list<Picture*>& rcUpdateList )
{
  // in lockstep mode, process only pics of same temporal layer
  const bool lockStepMode = m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_maxParallelFrames > 0;
  if( lockStepMode )
  {
    // start new parallel chunk only, if next output picture is not reconstructed
    if( m_gopEncListOutput.empty() || ! m_gopEncListOutput.front()->isReconstructed )
    {
      const int procTL         = m_gopEncListInput.size() ? m_gopEncListInput.front()->TLayer                      : -1;
      const int gopId          = m_gopEncListInput.size() ? m_gopEncListInput.front()->poc / m_pcEncCfg->m_GOPSize : -1;
      const int rcIdxInGop     = m_gopEncListInput.size() ? m_gopEncListInput.front()->rcIdxInGop                  : -1;
      const int minSerialDepth = m_pcEncCfg->m_maxParallelFrames > 2 ? 1 : 2;  // up to this temporal layer encode pictures only in serial mode
      const int maxSize        = procTL <= minSerialDepth ? 1 : m_pcEncCfg->m_maxParallelFrames;
      for( auto pic : m_gopEncListInput )
      {
        if( pic->poc / m_pcEncCfg->m_GOPSize == gopId
            && pic->TLayer == procTL
            && pic->slices[ 0 ]->checkRefPicsReconstructed() )
        {
          procList.push_back    ( pic );
          rcUpdateList.push_back( pic );
          // map all pics in a parallel chunk to the same index, improves RC performance
          pic->rcIdxInGop = rcIdxInGop;
        }
        if( (int)procList.size() >= maxSize )
          break;
      }
    }
  }
  else
  {
    procList = m_gopEncListInput;
    if( ! m_gopEncListOutput.empty() )
      rcUpdateList.push_back( m_gopEncListOutput.front() );
  }
  CHECK( ! rcUpdateList.empty() && m_gopEncListOutput.empty(),                                                         "first picture in RC update and in output list have to be the same" );
  CHECK( ! rcUpdateList.empty() && ! m_gopEncListOutput.empty() && rcUpdateList.front() != m_gopEncListOutput.front(), "first picture in RC update and in output list have to be the same" );
}

void EncGOP::xInitFirstSlice( Picture& pic, const PicList& picList, bool isEncodeLtRef )
{
  const int curPoc      = pic.getPOC();
  const int gopId       = pic.gopId;
  const int depth       = xGetSliceDepth( curPoc );
  memset( pic.cs->alfAps, 0, sizeof(pic.cs->alfAps));
  Slice* slice          = pic.allocateNewSlice();
  pic.cs->picHeader     = new PicHeader;
  const SPS& sps        = *(slice->sps);
  const int drtIPoffset = m_pcEncCfg->m_DecodingRefreshType == 4 ? 1 + (m_pcEncCfg->m_IntraPeriod - m_pcEncCfg->m_GOPSize) : 0;
  SliceType sliceType   = ( (curPoc+drtIPoffset) % (unsigned)m_pcEncCfg->m_IntraPeriod == 0 || m_pcEncCfg->m_GOPList[ gopId ].m_sliceType== 'I' || ( m_pcEncCfg->m_DecodingRefreshType == 4 && m_bFirstInit ) ) ? ( VVENC_I_SLICE ) : ( m_pcEncCfg->m_GOPList[ gopId ].m_sliceType== 'P' ? VVENC_P_SLICE : VVENC_B_SLICE );
  vvencNalUnitType naluType  = xGetNalUnitType( curPoc, m_lastIDR );

  if( curPoc == 0 && m_pcEncCfg->m_DecodingRefreshType == 4 && !m_bFirstInit )
  {
    sliceType = m_pcEncCfg->m_GOPList[ gopId ].m_sliceType== 'P' ? VVENC_P_SLICE : VVENC_B_SLICE;
  }

  pic.rcIdxInGop = std::max( 0, m_codingOrderIdx - 1 ) % m_pcEncCfg->m_GOPSize;
  m_codingOrderIdx += 1;

  // update IRAP
  if ( naluType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || naluType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP
      || naluType == VVENC_NAL_UNIT_CODED_SLICE_CRA )
  {
    m_associatedIRAPType = naluType;
    m_associatedIRAPPOC  = curPoc;
  }

  // update last IDR
  if ( naluType == VVENC_NAL_UNIT_CODED_SLICE_IDR_W_RADL || naluType == VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP )
  {
    m_lastIDR = curPoc;
  }

  slice->picHeader                 = pic.cs->picHeader;
  slice->independentSliceIdx       = 0;
  slice->sliceType                 = sliceType;
  slice->poc                       = curPoc;
  slice->TLayer                    = ( m_pcEncCfg->m_DecodingRefreshType == 4 && m_bFirstInit ) ? 0 : m_pcEncCfg->m_GOPList[ gopId ].m_temporalId;
  slice->nalUnitType               = naluType;
  slice->depth                     = depth;
  slice->lastIDR                   = m_lastIDR;

  slice->depQuantEnabled           = m_pcEncCfg->m_DepQuantEnabled;
  slice->signDataHidingEnabled     = m_pcEncCfg->m_SignDataHidingEnabled;

  slice->disableSATDForRd          = false;
  slice->picHeader->splitConsOverride     = false;
  for( int i = 0; i < 3; i++ )
  {
    slice->picHeader->minQTSize[i]   = sps.minQTSize[i];
    slice->picHeader->maxMTTDepth[i] = sps.maxMTTDepth[i];
    slice->picHeader->maxBTSize[i]   = sps.maxBTSize[i];
    slice->picHeader->maxTTSize[i]   = sps.maxTTSize[i];
  }

  slice->associatedIRAPType        = m_associatedIRAPType;
  slice->associatedIRAP            = m_associatedIRAPPOC;
  CHECK( MAX_REF_PICS <= m_pcEncCfg->m_RPLList0[ gopId ].m_numRefPicsActive, "number of ref pics out of supported range" );
  slice->numRefIdx[REF_PIC_LIST_0] = m_pcEncCfg->m_RPLList0[ gopId ].m_numRefPicsActive;
  slice->numRefIdx[REF_PIC_LIST_1] = m_pcEncCfg->m_RPLList1[ gopId ].m_numRefPicsActive;
  slice->setDecodingRefreshMarking ( m_pocCRA, m_bRefreshPending, picList );
  slice->setDefaultClpRng          ( sps );
  if (!slice->sps->Affine)
  {
    slice->picHeader->maxNumAffineMergeCand = m_pcEncCfg->m_SbTMVP ? 1 : 0;
  }

  // reference list
  int poc;
  xSelectReferencePictureList( slice, curPoc, gopId, -1 );
  if ( slice->checkThatAllRefPicsAreAvailable( picList, slice->rpl[0], 0, poc ) || slice->checkThatAllRefPicsAreAvailable( picList, slice->rpl[1], 1, poc ) )
  {
    slice->createExplicitReferencePictureSetFromReference( picList, slice->rpl[0], slice->rpl[1] );
  }
  slice->applyReferencePictureListBasedMarking( picList, slice->rpl[0], slice->rpl[1], 0, *slice->pps );

  // nalu type refinement
  if ( xIsSliceTemporalSwitchingPoint( slice, picList, gopId ) )
  {
    naluType = VVENC_NAL_UNIT_CODED_SLICE_STSA;
    slice->nalUnitType = naluType;
  }

  // reference list
  slice->numRefIdx[REF_PIC_LIST_0] = sliceType == VVENC_I_SLICE ? 0 : slice->rpl[0]->numberOfActivePictures;
  slice->numRefIdx[REF_PIC_LIST_1] = sliceType != VVENC_B_SLICE ? 0 : slice->rpl[1]->numberOfActivePictures;
  slice->constructRefPicList  ( picList, false );
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

  const vvencGOPEntry& gopEntry = m_pcEncCfg->m_GOPList[pic.gopId];
  slice->deblockingFilterOverride = sliceType != VVENC_I_SLICE && (gopEntry.m_betaOffsetDiv2 || gopEntry.m_tcOffsetDiv2);
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
  xInitSliceTMVPFlag ( pic.cs->picHeader, slice, gopId );
  xInitSliceMvdL1Zero( pic.cs->picHeader, slice );

  if( slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_RASL && m_pcEncCfg->m_rprRASLtoolSwitch )
  {
    slice->lmChromaCheckDisable = true;
    pic.cs->picHeader->disDmvrFlag = true;
    xUpdateRPRtmvp( pic.cs->picHeader, slice );
  }

  // update RAS
  xUpdateRasInit( slice );

  if ( m_pcEncCfg->m_useAMaxBT )
  {
    m_BlkStat.setSliceMaxBT( *slice );

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
    m_appliedSwitchDQQ = m_pcEncCfg->m_switchDQP;
  }
  pic.seqBaseQp = m_pcEncCfg->m_QP + m_appliedSwitchDQQ;

  pic.isInitDone = true;

  m_bFirstInit = false;
}

void EncGOP::xInitSliceTMVPFlag( PicHeader* picHeader, const Slice* slice, int gopId )
{
  if ( m_pcEncCfg->m_TMVPModeId == 2 )
  {
    if ( gopId == 0 ) // first picture in SOP (i.e. forward B)
    {
      picHeader->enableTMVP = false;
    }
    else
    {
      // Note: slice->colFromL0Flag is assumed to be always 0 and getcolRefIdx() is always 0.
      picHeader->enableTMVP = true;
    }
  }
  else if ( m_pcEncCfg->m_TMVPModeId == 1 )
  {
    picHeader->enableTMVP = true;
  }
  else
  {
    picHeader->enableTMVP = false;
  }

  // disable TMVP when current picture is the only ref picture
  if ( slice->isIRAP() && slice->sps->IBC )
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

void EncGOP::xSelectReferencePictureList( Slice* slice, int curPoc, int gopId, int ltPoc )
{
  slice->rplIdx[0] = gopId;
  slice->rplIdx[1] = gopId;

  int fullListNum = m_pcEncCfg->m_GOPSize;
  int partialListNum = m_pcEncCfg->m_numRPLList0 - m_pcEncCfg->m_GOPSize;
  int extraNum = fullListNum;
  if ( m_pcEncCfg->m_IntraPeriod < 0 )
  {
    if (curPoc < (2 * m_pcEncCfg->m_GOPSize + 2))
    {
      slice->rplIdx[0] = (curPoc + m_pcEncCfg->m_GOPSize - 1);
      slice->rplIdx[1] = (curPoc + m_pcEncCfg->m_GOPSize - 1);
    }
    else
    {
      slice->rplIdx[0] = ((curPoc%m_pcEncCfg->m_GOPSize == 0) ? m_pcEncCfg->m_GOPSize - 1 : curPoc%m_pcEncCfg->m_GOPSize - 1);
      slice->rplIdx[1] = ((curPoc%m_pcEncCfg->m_GOPSize == 0) ? m_pcEncCfg->m_GOPSize - 1 : curPoc%m_pcEncCfg->m_GOPSize - 1);
    }
    extraNum = fullListNum + partialListNum;
  }
  for (; extraNum < fullListNum + partialListNum; extraNum++)
  {
    if ( m_pcEncCfg->m_IntraPeriod > 0 && m_pcEncCfg->m_DecodingRefreshType > 0 )
    {
      int pocMod = m_pcEncCfg->m_DecodingRefreshType == 4 ? 1 : 0;
      int POCIndex = (curPoc+pocMod)%m_pcEncCfg->m_IntraPeriod;
      if (POCIndex == 0)
        POCIndex = m_pcEncCfg->m_IntraPeriod;
      if (POCIndex == m_pcEncCfg->m_RPLList0[extraNum].m_POC)
      {
        slice->rplIdx[0] = extraNum;
        slice->rplIdx[1] = extraNum;
        extraNum++;
      }
    }
  }

  const ReferencePictureList* rpl0 = &(slice->sps->rplList[0][slice->rplIdx[0]]);
  const ReferencePictureList* rpl1 = &(slice->sps->rplList[1][slice->rplIdx[1]]);
  slice->rpl[0] = rpl0;
  slice->rpl[1] = rpl1;
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
    au.dts      = ( ( iDiffFrames - m_gopSizeLog2 ) * m_ticksPerFrameMul4 ) / 4 + au.cts;
    au.dtsValid = true;
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
    const int apsMapIdx          = ( apsId << NUM_APS_TYPE_LEN ) + LMCS_APS;
    APS* aps                     = apsMap.getPS( apsMapIdx );
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
  if ( sps.alfEnabled && (slice->tileGroupAlfEnabled[COMP_Y] || slice->tileGroupCcAlfCbEnabled || slice->tileGroupCcAlfCrEnabled ))
  {
    for ( int apsId = 0; apsId < ALF_CTB_MAX_NUM_APS; apsId++ )
    {
      ParameterSetMap<APS>& apsMap = pic.picApsMap;
      const int apsMapIdx          = ( apsId << NUM_APS_TYPE_LEN ) + ALF_APS;
      APS* aps                     = apsMap.getPS( apsMapIdx );
      bool writeAps                = aps && apsMap.getChangedFlag( apsMapIdx );
      if ( !aps && slice->alfAps[ apsId ] && slice->alfAps[ apsId ])
      {
        aps   = apsMap.allocatePS( apsMapIdx );
        *aps  = *slice->alfAps[ apsId ]; // copy aps from slice header
        writeAps = true;
      }
      else if (slice->tileGroupCcAlfCbEnabled && !aps && apsId == slice->tileGroupCcAlfCbApsId)
      {
        writeAps = true;
        aps = apsMap.getPS((slice->tileGroupCcAlfCbApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      }
      else if (slice->tileGroupCcAlfCrEnabled && !aps && apsId == slice->tileGroupCcAlfCrApsId)
      {
        writeAps = true;
        aps = apsMap.getPS((slice->tileGroupCcAlfCrApsId << NUM_APS_TYPE_LEN) + ALF_APS);
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

void EncGOP::xCalculateAddPSNR( const Picture* pic, CPelUnitBuf cPicD, AccessUnitList& accessUnit, bool printFrameMSE, double* PSNR_Y, bool isEncodeLtRef )
{
  const SPS&         sps = *pic->cs->sps;
  const CPelUnitBuf& org = pic->getOrigBuf();
  double  dPSNR[MAX_NUM_COMP];

  for (int i = 0; i < MAX_NUM_COMP; i++)
  {
    dPSNR[i] = 0.0;
  }

  //===== calculate PSNR =====
  double MSEyuvframe[MAX_NUM_COMP] = {0, 0, 0};
  const ChromaFormat formatD = cPicD.chromaFormat;
  const ChromaFormat format  = sps.chromaFormatIdc;
  const Slice* slice         = pic->slices[0];

  for (int comp = 0; comp < getNumberValidComponents(formatD); comp++)
  {
    const ComponentID compID = ComponentID(comp);
    const CPelBuf&    p = cPicD.get(compID);
    const CPelBuf&    o = org.get(compID);

    CHECK(!( p.width  == o.width), "Unspecified error");
    CHECK(!( p.height == o.height), "Unspecified error");

    const uint32_t   width  = p.width  - (m_pcEncCfg->m_aiPad[ 0 ] >> getComponentScaleX(compID, format));
    const uint32_t   height = p.height - (m_pcEncCfg->m_aiPad[ 1 ] >> getComponentScaleY(compID, format));

    // create new buffers with correct dimensions
    const CPelBuf recPB(p.bufAt(0, 0), p.stride, width, height);
    const CPelBuf orgPB(o.bufAt(0, 0), o.stride, width, height);
    const uint32_t    bitDepth = sps.bitDepths[toChannelType(compID)];
    const uint64_t uiSSDtemp = xFindDistortionPlane(recPB, orgPB, 0);
    const uint32_t maxval = 255 << (bitDepth - 8);
    const uint32_t size   = width * height;
    const double fRefValue = (double)maxval * maxval * size;
    dPSNR[comp]       = uiSSDtemp ? 10.0 * log10(fRefValue / (double)uiSSDtemp) : 999.99;
    MSEyuvframe[comp] = (double)uiSSDtemp / size;
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

  if( m_isPreAnalysis || ! m_pcRateCtrl->rcIsFinalPass )
  {
    const double visualActivity = (pic->picVisActY > 0.0 ? pic->picVisActY : BitAllocation::getPicVisualActivity (slice, m_pcEncCfg));

    m_pcRateCtrl->addRCPassStats( slice->poc,
                                  slice->sliceQp,
                                  slice->getLambdas()[0],
                                  ClipBD( uint16_t (0.5 + visualActivity), m_pcEncCfg->m_internalBitDepth[CH_L] ),
                                  uibits,
                                  dPSNR[COMP_Y],
                                  slice->isIntra(),
                                  slice->TLayer );
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
    if( m_isPreAnalysis || ! m_pcRateCtrl->rcIsFinalPass )
    {
      std::string cInfo = prnt("RC pass %d/%d, analyze poc %4d",
          m_pcRateCtrl->rcPass + 1,
          m_pcEncCfg->m_RCNumPasses,
          slice->poc );

          accessUnit.InfoString.append( cInfo );
    }
    else
    {
      std::string cInfo = prnt("POC %4d TId: %1d (%10s, %c-SLICE, QP %d ) %10d bits",
          slice->poc,
          slice->TLayer,
          nalUnitTypeToString( slice->nalUnitType ),
          c,
          slice->sliceQp,
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
  xCalculateAddPSNR( &pic, pic.getRecoBuf(), accessUnit, printFrameMSE, &PSNR_Y, isEncodeLtRef );

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

  std::string cPicInfo = accessUnit.InfoString;
  cPicInfo.append("\n");

  const vvencMsgLevel msgLevel = m_isPreAnalysis ? VVENC_DETAILS : VVENC_NOTICE;
  msg.log( msgLevel, cPicInfo.c_str() );
  if( m_pcEncCfg->m_verbosity >= msgLevel ) fflush( stdout );
}

} // namespace vvenc

//! \}


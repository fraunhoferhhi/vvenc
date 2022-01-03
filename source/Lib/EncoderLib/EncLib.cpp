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


/** \file     EncLib.cpp
    \brief    encoder class
*/

#include "EncLib.h"
#include "CommonLib/Picture.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/TimeProfiler.h"
#include "CommonLib/Rom.h"
#include "Utilities/NoMallocThreadPool.h"
#include "Utilities/MsgLog.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {


// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncLib::EncLib( MsgLog& logger )
  : msg             ( logger )
  , m_recYuvBufFunc  ( nullptr )
  , m_recYuvBufCtx   ( nullptr )
  , m_encCfg         ()
  , m_orgCfg         ()
  , m_firstPassCfg   ()
  , m_rateCtrl       ( nullptr )
  , m_MCTF           ( nullptr )
  , m_preEncoder     ( nullptr )
  , m_gopEncoder     ( nullptr )
  , m_threadPool     ( nullptr )
  , m_picsRcvd       ( 0 )
  , m_passInitialized( -1 )
{
}

EncLib::~EncLib()
{
  if( m_rateCtrl )
  {
    delete m_rateCtrl;
    m_rateCtrl = nullptr;
  }
}

void EncLib::setRecYUVBufferCallback( void* ctx, vvencRecYUVBufferCallback func )
{
  m_recYuvBufCtx  = ctx;
  m_recYuvBufFunc = func;
  if( m_rateCtrl && m_rateCtrl->rcIsFinalPass && m_gopEncoder )
  {
    m_gopEncoder->setRecYUVBufferCallback( m_recYuvBufCtx, m_recYuvBufFunc );
  }
}

void EncLib::initEncoderLib( const VVEncCfg& encCfg )
{
  // copy config parameter
  const_cast<VVEncCfg&>(m_encCfg) = encCfg;

  // setup modified configs for rate control
  if( m_encCfg.m_RCNumPasses > 1 || m_encCfg.m_LookAhead )
  {
    xInitRCCfg();
  }

  // initialize pass
  initPass( 0, nullptr );

#if ENABLE_TRACING
  g_trace_ctx = tracing_init( m_encCfg.m_traceFile, m_encCfg.m_traceRule, msg );
  if( g_trace_ctx && m_encCfg.m_listTracingChannels )
  {
    std::string sChannelsList;
    g_trace_ctx->getChannelsList( sChannelsList );
    msg.log( VVENC_INFO, "\n Using tracing channels:\n\n%s\n", sChannelsList.c_str() );
  }
#endif

#if ENABLE_TIME_PROFILING
  if( g_timeProfiler == nullptr )
  {
    g_timeProfiler = timeProfilerCreate( encCfg );
  }
#endif
}

void EncLib::uninitEncoderLib()
{
#if ENABLE_TRACING
  if( g_trace_ctx )
  {
    tracing_uninit( g_trace_ctx );
  }
#endif

#if ENABLE_CU_MODE_COUNTERS
  std::cout << std::endl;
  std::cout << "CU Modes statistic across picture types and temporal levels (0:Intra, >0:Inter, luma only)" << std::endl;
  for( size_t j = 0; j < g_cuCounters1D.getNumCntTypes(); j++ )
  {
    for( size_t i = 0; i < g_cuCounters1D.getDimHor() - 1; i++ )
    {
      g_cuCounters1D[j][0][g_cuCounters1D.getDimHor() - 1] += g_cuCounters1D[j][0][i];
    }
  }
  StatCounters::report2D( std::cout, g_cuCounters1D, false, true, false, true, true, CU_MODES_TESTED );

  std::cout << std::endl;
  std::cout << "CU Modes statistic across block-shapes (Non-I-Slices, luma only)" << std::endl;
  StatCounters::report2D( std::cout, g_cuCounters2D, true, true, false, true, true, CU_MODES_TESTED );
#endif

#if ENABLE_TIME_PROFILING
#if ENABLE_TIME_PROFILING_MT_MODE
  for( auto& p : m_threadPool->getProfilers() )
  {
    *g_timeProfiler += *p;
  }
#endif
  timeProfilerResults( g_timeProfiler );
#endif
  xUninitLib();
}

void EncLib::initPass( int pass, const char* statsFName )
{
  CHECK( m_passInitialized != pass && m_passInitialized + 1 != pass, "initialization of passes only in successive order possible" );

  if( m_rateCtrl == nullptr )
  {
    if( m_encCfg.m_RCNumPasses == 1 && !m_encCfg.m_LookAhead )
    {
      m_rateCtrl = new LegacyRateCtrl(msg);
    }
    else
    {
      m_rateCtrl = new RateCtrl(msg);
    }
  }

  m_rateCtrl->setRCPass( m_encCfg, pass, statsFName );

  if( m_passInitialized + 1 != pass )
  {
    return;
  }

  // reset
  xUninitLib();

  // enable encoder config based on rate control pass
  if( m_encCfg.m_RCNumPasses > 1 || (m_encCfg.m_LookAhead && m_orgCfg.m_RCTargetBitrate) )
  {
    if (!m_rateCtrl->rcIsFinalPass)
    {
      // set encoder config for 1st rate control pass
      const_cast<VVEncCfg&>(m_encCfg) = m_firstPassCfg;
    }
    else
    {
      // restore encoder config for final 2nd RC pass
      const_cast<VVEncCfg&>(m_encCfg) = m_orgCfg;
      const_cast<VVEncCfg&>(m_encCfg).m_QP = m_rateCtrl->getBaseQP();
    }
  }

  // thread pool
  if( m_encCfg.m_numThreads > 0 )
  {
    m_threadPool = new NoMallocThreadPool( m_encCfg.m_numThreads, "EncSliceThreadPool", &m_encCfg );
  }

  // MCTF
  if( m_encCfg.m_vvencMCTF.MCTF )
  {
    m_MCTF = new MCTF();
    //m_MCTF->initStage( MCTF_ADD_QUEUE_DELAY, true, true, m_encCfg.m_CTUSize );
    const int minDelay = m_encCfg.m_vvencMCTF.MCTFFutureReference ? ( m_encCfg.m_vvencMCTF.MCTFNumLeadFrames + 1 + VVENC_MCTF_RANGE ) : ( m_encCfg.m_vvencMCTF.MCTFNumLeadFrames + 1 );
    m_MCTF->initStage( minDelay, true, true, m_encCfg.m_CTUSize );
    m_MCTF->init( m_encCfg, m_threadPool );
    m_encStages.push_back( m_MCTF );
  }

  // pre analysis encoder
  if( m_encCfg.m_LookAhead )
  {
    m_preEncoder = new EncGOP( msg );
    m_preEncoder->initStage( m_firstPassCfg.m_GOPSize + 1, true, false, m_firstPassCfg.m_CTUSize );
    m_preEncoder->init( m_firstPassCfg, *m_rateCtrl, m_threadPool, true );
    m_encStages.push_back( m_preEncoder );
  }

  // gop encoder
  m_gopEncoder = new EncGOP( msg );
  const int encDelay = m_encCfg.m_LookAhead ? m_encCfg.m_IntraPeriod + 1 : m_encCfg.m_GOPSize + 1;
  m_gopEncoder->initStage( encDelay, false, false, m_encCfg.m_CTUSize );
  m_gopEncoder->init( m_encCfg, *m_rateCtrl, m_threadPool, false );
  if( m_rateCtrl->rcIsFinalPass )
  {
    m_gopEncoder->setRecYUVBufferCallback( m_recYuvBufCtx, m_recYuvBufFunc );
  }
  m_encStages.push_back( m_gopEncoder );

  // link encoder stages
  for( int i = 0; i < (int)m_encStages.size() - 1; i++ )
  {
    m_encStages[ i ]->linkNextStage( m_encStages[ i + 1 ] );
  }

  // rate control
  if( m_encCfg.m_RCTargetBitrate > 0 )
  {
    m_rateCtrl->init( m_encCfg );
    if( pass == 1 )
    {
      m_rateCtrl->processFirstPassData( false );
    }
  }

  // prepare prev shared data
  while( m_prevSharedQueue.size() < QPA_PREV_FRAMES )
  {
    m_prevSharedQueue.push_back( nullptr );
  }

  m_picsRcvd        = m_encCfg.m_vvencMCTF.MCTF ? -m_encCfg.m_vvencMCTF.MCTFNumLeadFrames : 0;
  m_passInitialized = pass;
}

void EncLib::xUninitLib()
{
  // sub modules
  if( m_rateCtrl != nullptr )
  {
    m_rateCtrl->destroy();
  }
  if( m_MCTF )
  {
    delete m_MCTF;
    m_MCTF = nullptr;
  }
  if( m_preEncoder )
  {
    delete m_preEncoder;
    m_preEncoder = nullptr;
  }
  if( m_gopEncoder )
  {
    delete m_gopEncoder;
    m_gopEncoder = nullptr;
  }
  m_encStages.clear();

  // shared data
  for( auto picShared : m_picSharedList )
  {
    delete picShared;
  }
  m_picSharedList.clear();
  m_prevSharedQueue.clear();

  // thread pool
  if( m_threadPool )
  {
    m_threadPool->shutdown( true );
    delete m_threadPool;
    m_threadPool = nullptr;
  }
}

void EncLib::xInitRCCfg()
{
  // backup original configuration
  const_cast<VVEncCfg&>(m_orgCfg) = m_encCfg;

  // initialize first pass configuration
  m_firstPassCfg = m_encCfg;
  vvenc_init_preset( &m_firstPassCfg, vvencPresetMode::VVENC_FIRSTPASS );

  // fixed-QP encoding in first rate control pass
  const double d = (3840.0 * 2160.0) / double (m_encCfg.m_SourceWidth * m_encCfg.m_SourceHeight);
  m_firstPassCfg.m_RCTargetBitrate = 0;
  m_firstPassCfg.m_QP /*base QP*/  = (m_encCfg.m_RCInitialQP > 0 ? Clip3 (17, MAX_QP, m_encCfg.m_RCInitialQP) : std::max (17, MAX_QP_PERCEPT_QPA - 2 - int (0.5 + sqrt ((d * m_encCfg.m_RCTargetBitrate) / 500000.0))));

  // preserve some settings
  if( m_firstPassCfg.m_usePerceptQPA && ( m_firstPassCfg.m_QP <= MAX_QP_PERCEPT_QPA || m_firstPassCfg.m_framesToBeEncoded == 1 ) )
  {
    m_firstPassCfg.m_CTUSize       = m_encCfg.m_CTUSize;
  }
  m_firstPassCfg.m_vvencMCTF.MCTF  = m_encCfg.m_vvencMCTF.MCTF;
  m_firstPassCfg.m_IBCMode         = m_encCfg.m_IBCMode;

  // clear MaxCuDQPSubdiv
  if( m_firstPassCfg.m_CTUSize < 128 && ( m_firstPassCfg.m_PadSourceWidth > 1024 || m_firstPassCfg.m_PadSourceHeight > 640 ) )
  {
    m_firstPassCfg.m_cuQpDeltaSubdiv = 0;
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncLib::encodePicture( bool flush, const vvencYUVBuffer* yuvInBuf, AccessUnitList& au, bool& isQueueEmpty )
{
  PROFILER_ACCUM_AND_START_NEW_SET( 1, g_timeProfiler, P_TOP_LEVEL );

  CHECK( yuvInBuf == nullptr && ! flush, "no input picture given" );

  // clear output access unit
  au.clearAu();

  // send new YUV input buffer to first encoder stage
  if( yuvInBuf )
  {
    PicShared* picShared = xGetFreePicShared();
    picShared->reuse( m_picsRcvd, yuvInBuf );
    if (m_encCfg.m_usePerceptQPA || m_encCfg.m_RCNumPasses == 2 || (m_encCfg.m_LookAhead && m_rateCtrl->m_pcEncCfg->m_RCTargetBitrate) )
    {
      xAssignPrevQpaBufs( picShared );
    }
    xDetectScc( picShared );
    m_encStages[ 0 ]->addPicSorted( picShared );
    m_picsRcvd += 1;
  }

  PROFILER_EXT_UPDATE( g_timeProfiler, P_TOP_LEVEL, pic->TLayer );

  // trigger stages
  isQueueEmpty = true;
  for( auto encStage : m_encStages )
  {
    encStage->runStage( flush, au );
    isQueueEmpty &= encStage->isStageDone();
  }

  // reset output access unit, if not final pass
  if( ! m_rateCtrl->rcIsFinalPass )
  {
    au.clearAu();
  }
}

void EncLib::printSummary()
{
  if( m_gopEncoder )
  {
    m_gopEncoder->printOutSummary( m_encCfg.m_printMSEBasedSequencePSNR, m_encCfg.m_printSequenceMSE, m_encCfg.m_printHexPsnr );
  }
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

PicShared* EncLib::xGetFreePicShared()
{
  PicShared* picShared = nullptr;
  for( auto itr : m_picSharedList )
  {
    if( ! itr->isUsed() )
    {
      picShared = itr;
      break;
    }
  }

  if( ! picShared )
  {
    picShared = new PicShared();
    picShared->create( m_encCfg.m_framesToBeEncoded, m_encCfg.m_internChromaFormat, Size( m_encCfg.m_PadSourceWidth, m_encCfg.m_PadSourceHeight ), m_encCfg.m_vvencMCTF.MCTF );
    m_picSharedList.push_back( picShared );
  }
  CHECK( picShared == nullptr, "out of memory" );

  return picShared;
}

void EncLib::xAssignPrevQpaBufs( PicShared* picShared )
{
  for( int i = 0; i < QPA_PREV_FRAMES; i++ )
  {
    picShared->m_prevShared[ i ] = m_prevSharedQueue[ i ];
  }
  m_prevSharedQueue.pop_back();
  m_prevSharedQueue.push_front( picShared );
  // for very first frame link itself
  if( picShared->m_prevShared[ 0 ] == nullptr )
  {
    picShared->m_prevShared[ 0 ] = picShared;
  }
}

#if FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED && defined( __GNUC__ ) && __GNUC__ == 5
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-overflow"
#endif

void EncLib::xDetectScc( PicShared* picShared )
{
  Picture pic;
  picShared->shareData( &pic );
  CPelUnitBuf yuvOrgBuf = pic.getOrigBuf();

  bool isSccWeak   = false;
  bool isSccStrong = false;

  int SIZE_BL = 4;
  int K_SC = 25;
  const Pel* piSrc = yuvOrgBuf.Y().buf;
  uint32_t   uiStride = yuvOrgBuf.Y().stride;
  uint32_t   uiWidth = yuvOrgBuf.Y().width;
  uint32_t   uiHeight = yuvOrgBuf.Y().height;
  int size = SIZE_BL;
  unsigned   hh, ww;
  int SizeS = SIZE_BL << 1;
  int sR[4] = { 0,0,0,0 };
  int AmountBlock = (uiWidth >> 2) * (uiHeight >> 2);
  for( hh = 0; hh < uiHeight; hh += SizeS )
  {
    for( ww = 0; ww < uiWidth; ww += SizeS )
    {
      int Rx = ww > (uiWidth >> 1) ? 1 : 0;
      int Ry = hh > (uiHeight >> 1) ? 1 : 0;
      Ry = Ry << 1 | Rx;

      int i = ww;
      int j = hh;
      int n = 0;
      int Var[4];
      for( j = hh; (j < hh + SizeS) && (j < uiHeight); j += size )
      {
        for( i = ww; (i < ww + SizeS) && (i < uiWidth); i += size )
        {
          int sum = 0;
          int Mit = 0;
          int V = 0;
          int h = j;
          int w = i;
          for( h = j; (h < j + size) && (h < uiHeight); h++ )
          {
            for( w = i; (w < i + size) && (w < uiWidth); w++ )
            {
              sum += int(piSrc[h * uiStride + w]);
            }
          }
          int sizeEnd = ((h - j) * (w - i));
          Mit = sum / sizeEnd;
          for( h = j; (h < j + size) && (h < uiHeight); h++ )
          {
            for( w = i; (w < i + size) && (w < uiWidth); w++ )
            {
              V += abs(Mit - int(piSrc[h * uiStride + w]));
            }
          }
          // Variance in Block (SIZE_BL*SIZE_BL)
          V = V / sizeEnd;
          Var[n] = V;
          n++;
        }
      }
      for( int i = 0; i < 2; i++ )
      {
        if( Var[i] == Var[i + 2] )
        {
          sR[Ry] += 1;
        }
        if( Var[i << 1] == Var[(i << 1) + 1] )
        {
          sR[Ry] += 1;
        }
      }
    }
  }
  int s = 0;
  isSccStrong = true;
  for( int r = 0; r < 4; r++ )
  {
    s += sR[r];
    if( ((sR[r] * 100 / (AmountBlock >> 2)) <= K_SC) )
    {
      isSccStrong = false;
    }
  }
  isSccWeak = ((s * 100 / AmountBlock) > K_SC);

  picShared->m_isSccWeak   = isSccWeak;
  picShared->m_isSccStrong = isSccStrong;

  picShared->releaseShared( &pic );
}

#if FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED && defined( __GNUC__ ) && __GNUC__ == 5
#pragma GCC diagnostic pop
#endif

} // namespace vvenc

//! \}

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


/** \file     EncLib.cpp
    \brief    encoder class
*/

#include "EncLib.h"
#include "CommonLib/Picture.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/TimeProfiler.h"
#include "CommonLib/Rom.h"
#include "CommonLib/MCTF.h"
#include "Utilities/NoMallocThreadPool.h"
#include "Utilities/MsgLog.h"
#include "EncStage.h"
#include "PreProcess.h"
#include "EncGOP.h"

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
  , m_preProcess     ( nullptr )
  , m_MCTF           ( nullptr )
  , m_preEncoder     ( nullptr )
  , m_gopEncoder     ( nullptr )
  , m_threadPool     ( nullptr )
  , m_picsRcvd       ( 0 )
  , m_passInitialized( -1 )
  , m_maxNumPicShared( MAX_INT )
  , m_accessUnitOutputStarted( false )
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

void EncLib::initEncoderLib( const vvenc_config& encCfg )
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
    m_rateCtrl = new RateCtrl(msg);
  }

  m_rateCtrl->setRCPass( m_encCfg, pass, statsFName );

  if( m_passInitialized + 1 != pass )
  {
    return;
  }

  // reset
  xUninitLib();

  // enable encoder config based on rate control pass
  if( m_encCfg.m_RCNumPasses > 1 || ( m_encCfg.m_LookAhead && m_orgCfg.m_RCTargetBitrate > 0 ) )
  {
    if( !m_rateCtrl->rcIsFinalPass )
    {
      // set encoder config for 1st rate control pass
      const_cast<VVEncCfg&>(m_encCfg) = m_firstPassCfg;
    }
    else
    {
      // restore encoder config for final 2nd RC pass
      const_cast<VVEncCfg&>(m_encCfg) = m_orgCfg;
      m_rateCtrl->init( m_encCfg );
      const_cast<VVEncCfg&>(m_encCfg).m_QP = m_rateCtrl->getBaseQP();
    }
    if( m_encCfg.m_RCTargetBitrate > 0 && !m_encCfg.m_LookAhead )
    {
      m_rateCtrl->processFirstPassData( false );
    }
  }
  else if( m_encCfg.m_usePerceptQPA && m_encCfg.m_LookAhead )
  {
    m_rateCtrl->init( m_encCfg );
  }

  // thread pool
  if( m_encCfg.m_numThreads > 0 )
  {
    m_threadPool = new NoMallocThreadPool( m_encCfg.m_numThreads, "EncSliceThreadPool", &m_encCfg );
  }
  m_maxNumPicShared = 0;

  // pre processing
  m_preProcess = new PreProcess( msg );
  m_preProcess->initStage( m_encCfg, 1, -m_encCfg.m_leadFrames, true, true, false );
  m_preProcess->init( m_encCfg, m_rateCtrl->rcIsFinalPass );
  m_encStages.push_back( m_preProcess );
  m_maxNumPicShared += 1;

  // MCTF
  if( m_encCfg.m_vvencMCTF.MCTF )
  {
    m_MCTF = new MCTF();
    const int leadFrames   = std::min( VVENC_MCTF_RANGE, m_encCfg.m_leadFrames );
    const int minQueueSize = m_encCfg.m_vvencMCTF.MCTFFutureReference ? ( leadFrames + 1 + VVENC_MCTF_RANGE ) : ( leadFrames + 1 );
    m_MCTF->initStage( m_encCfg, minQueueSize, -leadFrames, true, true, false );
    m_MCTF->init( m_encCfg, m_threadPool );
    m_encStages.push_back( m_MCTF );
    m_maxNumPicShared += minQueueSize - leadFrames;
  }

  // pre analysis encoder
  if( m_encCfg.m_LookAhead )
  {
    m_preEncoder = new EncGOP( msg );
    const int minQueueSize = m_firstPassCfg.m_GOPSize + 1;
    m_preEncoder->initStage( m_firstPassCfg, minQueueSize, 0, false, false, false );
    m_preEncoder->init( m_firstPassCfg, m_preProcess->getGOPCfg(), *m_rateCtrl, m_threadPool, true );
    m_encStages.push_back( m_preEncoder );
    m_maxNumPicShared += minQueueSize;
  }

  // gop encoder
  m_gopEncoder = new EncGOP( msg );
  const int minQueueSize = m_encCfg.m_GOPSize + 1;
  m_gopEncoder->initStage( m_encCfg, minQueueSize, 0, false, false, m_encCfg.m_stageParallelProc );
  m_gopEncoder->init( m_encCfg, m_preProcess->getGOPCfg(), *m_rateCtrl, m_threadPool, false );
  m_encStages.push_back( m_gopEncoder );
  m_maxNumPicShared += minQueueSize;

  // additional pictures due to structural delay
  m_maxNumPicShared += m_preProcess->getGOPCfg()->getNumReorderPics()[ m_encCfg.m_maxTLayer ];
  m_maxNumPicShared += 3;

  if( m_rateCtrl->rcIsFinalPass )
  {
    m_gopEncoder->setRecYUVBufferCallback( m_recYuvBufCtx, m_recYuvBufFunc );
  }

  // link encoder stages
  for( int i = 0; i < (int)m_encStages.size() - 1; i++ )
  {
    m_encStages[ i ]->linkNextStage( m_encStages[ i + 1 ] );
  }

  m_picsRcvd                = -m_encCfg.m_leadFrames;
  m_accessUnitOutputStarted = false;
  m_passInitialized         = pass;
}

void EncLib::xUninitLib()
{
  // make sure all processing threads are stopped before releasing data
  if( m_threadPool )
  {
    m_threadPool->shutdown( true );
  }

  // sub modules
  if( m_rateCtrl != nullptr )
  {
    m_rateCtrl->destroy();
  }
  if( m_preProcess )
  {
    delete m_preProcess;
    m_preProcess = nullptr;
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

  for( auto picShared : m_picSharedList )
  {
    delete picShared;
  }
  m_picSharedList.clear();

  // thread pool
  if( m_threadPool )
  {
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

  // NOTE regarding the stage parallel processing
  // The next input yuv-frame must be passed to the encoding process (1.Stage).
  // Following should be considered:
  // 1. The final stage is non-blocking, so it dosen't wait until picture is reconstructed.
  // 2. Generally the stages have different throughput, last stage is the slowest.
  // 3. The number of picture-units, required for the input frames, is limited.
  // 4. Due to chunk-mode and non-blockiness, it's possible that we can run out of picture-units.
  // 5. Then we have to wait for the next available picture-unit and the input frame can be passed to the 1.stage.

  PicShared* picShared = nullptr;
  bool inputPending    = ( yuvInBuf != nullptr );
  while( true )
  {
    // send new YUV input buffer to first encoder stage
    if( inputPending )
    {
      picShared = xGetFreePicShared();
      if( picShared )
      {
        picShared->reuse( m_picsRcvd, yuvInBuf );
        m_encStages[ 0 ]->addPicSorted( picShared );
        m_picsRcvd  += 1;
        inputPending = false;
      }
    }

    PROFILER_EXT_UPDATE( g_timeProfiler, P_TOP_LEVEL, pic->TLayer );

    // trigger stages
    isQueueEmpty = m_picsRcvd > 0 || ( m_picsRcvd <= 0 && flush );
    for( auto encStage : m_encStages )
    {
      encStage->runStage( flush, m_encCfg.m_RCNumPasses > 1 ? m_orgCfg.m_CTUSize : m_encCfg.m_CTUSize, au );
      isQueueEmpty &= encStage->isStageDone();
    }

    if( !au.empty() )
    {
      m_AuList.push_back( au );
      au.detachNalUnitList();
      au.clearAu();
      // NOTE: delay AU output in stage parallel mode only
      if( !m_accessUnitOutputStarted )
        m_accessUnitOutputStarted = !m_encCfg.m_stageParallelProc || m_AuList.size() > 4 || flush;
    }

    // wait if input picture hasn't been stored yet or if encoding is running and no new output access unit has been encoded
    bool waitAndStay = inputPending || ( m_AuList.empty() && ! isQueueEmpty && ( m_accessUnitOutputStarted || flush ) );
    if( ! waitAndStay )
    {
      break;
    }

    if( m_encCfg.m_stageParallelProc )
    {
      for( auto encStage : m_encStages )
      {
        if( encStage->isNonBlocking() )
          encStage->waitForFreeEncoders();
      }
    }
  }

  // check if we have an AU to output
  if( !m_AuList.empty() && m_accessUnitOutputStarted )
  {
    au = m_AuList.front();
    m_AuList.front().detachNalUnitList();
    m_AuList.pop_front();
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

void EncLib::getParameterSets( AccessUnitList& au )
{
  if( m_gopEncoder )
  {
    m_gopEncoder->getParameterSets( au );
  }
}

int EncLib::getCurPass() const
{
  return m_passInitialized;
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
    if( m_encCfg.m_stageParallelProc && ( m_picSharedList.size() >= m_maxNumPicShared ) )
      return nullptr;

    picShared = new PicShared();
    picShared->create( m_encCfg.m_framesToBeEncoded, m_encCfg.m_internChromaFormat, Size( m_encCfg.m_PadSourceWidth, m_encCfg.m_PadSourceHeight ), m_encCfg.m_vvencMCTF.MCTF );
    m_picSharedList.push_back( picShared );
  }
  CHECK( picShared == nullptr, "out of memory" );

  return picShared;
}

} // namespace vvenc

//! \}

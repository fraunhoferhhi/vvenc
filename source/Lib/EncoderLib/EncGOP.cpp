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
#include "CommonLib/MD5.h"
#include "DecoderLib/DecLib.h"
#include "BitAllocation.h"
#include "EncHRD.h"

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

void trySkipOrDecodePicture( bool& decPic, bool& encPic, const VVEncCfg& cfg, Picture* pic, FFwdDecoder& ffwdDecoder, ParameterSetMap<APS>& apsMap )
{
  // check if we should decode a leading bitstream
  if( cfg.m_decodeBitstreams[0][0] != '\0' )
  {
    if( ffwdDecoder.bDecode1stPart )
    {
      if( cfg.m_forceDecodeBitstream1 )
      {
        if( 0 != ( ffwdDecoder.bDecode1stPart = tryDecodePicture( pic, pic->getPOC(), cfg.m_decodeBitstreams[ 0 ], ffwdDecoder, &apsMap, false ) ) )
        {
          decPic = ffwdDecoder.bDecode1stPart;
        }
      }
      else
      {
        // update decode decision
        if( (0 != ( ffwdDecoder.bDecode1stPart = ( cfg.m_switchPOC != pic->getPOC() )  )) && ( 0 != ( ffwdDecoder.bDecode1stPart = tryDecodePicture( pic, pic->getPOC(), cfg.m_decodeBitstreams[ 0 ], ffwdDecoder, &apsMap, false, cfg.m_switchPOC ) ) ) )
        {
          decPic = ffwdDecoder.bDecode1stPart;
          return;
        }
        else if( pic->getPOC() )
        {
          // reset decoder if used and not required any further
          tryDecodePicture( NULL, 0, std::string( "" ), ffwdDecoder, &apsMap );
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
    if( bDecode2ndPart && (0 != (bDecode2ndPart = tryDecodePicture( pic, expectedPoc, cfg.m_decodeBitstreams[ 1 ], ffwdDecoder, &apsMap, true )) ))
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
EncGOP::EncGOP()
  : m_bFirstInit         ( true )
  , m_bFirstWrite        ( true )
  , m_bRefreshPending    ( false )
  , m_disableLMCSIP      ( false )
  , m_codingOrderIdx     ( 0 )
  , m_lastIDR            ( 0 )
  , m_lastRasPoc         ( MAX_INT )
  , m_pocCRA             ( 0 )
  , m_appliedSwitchDQQ   ( 0 )
  , m_associatedIRAPPOC  ( 0 )
  , m_associatedIRAPType ( VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP )
  , m_pcEncCfg           ( nullptr )
  , m_pcRateCtrl         ( nullptr )
  , m_pcEncHRD           ( nullptr )
  , m_gopApsMap          ( MAX_NUM_APS * MAX_NUM_APS_TYPE )
  , m_threadPool         ( nullptr )
{
}


EncGOP::~EncGOP()
{
  if( m_pcEncCfg && ( m_pcEncCfg->m_decodeBitstreams[0][0] != '\0' || m_pcEncCfg->m_decodeBitstreams[1][0] != '\0' ) )
  {
    // reset potential decoder resources
    tryDecodePicture( NULL, 0, std::string(""), m_ffwdDecoder, &m_gopApsMap );
  }

  for( auto& picEncoder : m_freePicEncoderList )
  {
    if( picEncoder )
    {
      delete picEncoder;
    }
  }
  m_freePicEncoderList.clear();
  m_threadPool = nullptr;
}


void EncGOP::init( const VVEncCfg& encCfg, const SPS& sps, const PPS& pps, RateCtrl& rateCtrl, EncHRD& encHrd, NoMallocThreadPool* threadPool )
{
  m_pcEncCfg   = &encCfg;
  m_pcRateCtrl = &rateCtrl;
  m_pcEncHRD   = &encHrd;
  m_threadPool = threadPool;

  m_seiEncoder.init( encCfg, encHrd );
  m_Reshaper.init  ( encCfg );

  m_appliedSwitchDQQ = 0;
  const int maxPicEncoder = ( encCfg.m_maxParallelFrames ) ? encCfg.m_maxParallelFrames : 1;
  for ( int i = 0; i < maxPicEncoder; i++ )
  {
    EncPicture* picEncoder = new EncPicture;
    picEncoder->init( encCfg, &m_globalCtuQpVector, sps, pps, rateCtrl, threadPool );
    m_freePicEncoderList.push_back( picEncoder );
  }

  if (encCfg.m_usePerceptQPA)
  {
    m_globalCtuQpVector.resize( pps.useDQP && (encCfg.m_usePerceptQPATempFiltISlice == 2) ? pps.picWidthInCtu * pps.picHeightInCtu + 1 : 1 );
  }
}


// ====================================================================================================================
// Class interface
// ====================================================================================================================

void EncGOP::xGetProcessingLists( std::list<Picture*>& procList, std::list<Picture*>& rcUpdateList )
{
  // in lockstep mode, process only pics of same temporal layer
  const bool lockStepMode  = m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_maxParallelFrames > 0;
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

void EncGOP::encodePictures( const std::vector<Picture*>& encList, PicList& picList, AccessUnitList& au, bool isEncodeLtRef )
{
  CHECK( encList.size() == 0 && m_gopEncListOutput.size() == 0, "error: no pictures to be encoded given" );

  // init pictures and first slice (in coding order)
  if( encList.size() )
  {
    xInitPicsInCodingOrder( encList, picList, isEncodeLtRef );
  }

  // get list of pictures to be encoded and used for RC update
  std::list<Picture*> procList;
  std::list<Picture*> rcUpdateList;
  xGetProcessingLists( procList, rcUpdateList );

  // in lockstep mode, process all pictures in processing list
  const bool lockStepMode = m_pcEncCfg->m_RCTargetBitrate > 0 && m_pcEncCfg->m_maxParallelFrames > 0;

  // encode one picture in serial mode / multiple pictures in FPP mode
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
    trySkipOrDecodePicture( decPic, encPic, *m_pcEncCfg, pic, m_ffwdDecoder, m_gopApsMap );
    DTRACE_UPDATE( g_trace_ctx, std::make_pair( "encdec", 0 ) );
    pic->writePic = decPic || encPic;
    pic->encPic   = encPic;

    if( m_pcEncCfg->m_alfTempPred )
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

  CHECK( m_gopEncListOutput.empty(),                    "try to output picture, but no output picture available" );
  CHECK( ! m_gopEncListOutput.front()->isReconstructed, "try to output picture, but picture not reconstructed" );

  // AU output
  Picture* outPic = m_gopEncListOutput.front();
  m_gopEncListOutput.pop_front();

  if( outPic->writePic )
  {
    xWritePicture( *outPic, au, isEncodeLtRef );
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
    xUpdateAfterPicRC( pic );
  }

  if( m_pcEncCfg->m_useAMaxBT )
  {
    m_BlkStat.updateMaxBT( *outPic->slices[0], outPic->picBlkStat );
  }

  outPic->slices[ 0 ]->updateRefPicCounter( -1 );
  outPic->isFinished = true;
}

void EncGOP::printOutSummary( int numAllPicCoded, const bool printMSEBasedSNR, const bool printSequenceMSE, const bool printHexPsnr, const BitDepths &bitDepths )
{

  if( m_pcEncCfg->m_decodeBitstreams[0][0] != '\0' && m_pcEncCfg->m_decodeBitstreams[1][0] != '\0' && m_pcEncCfg->m_fastForwardToPOC < 0 )
  {
    CHECK( !( numAllPicCoded == m_AnalyzeAll.getNumPic() ), "Unspecified error" );
  }

  //--CFG_KDY
  const int rateMultiplier = 1;
  m_AnalyzeAll.setFrmRate( m_pcEncCfg->m_FrameRate*rateMultiplier / (double)m_pcEncCfg->m_temporalSubsampleRatio);
  m_AnalyzeI.setFrmRate( m_pcEncCfg->m_FrameRate*rateMultiplier / (double)m_pcEncCfg->m_temporalSubsampleRatio);
  m_AnalyzeP.setFrmRate( m_pcEncCfg->m_FrameRate*rateMultiplier / (double)m_pcEncCfg->m_temporalSubsampleRatio);
  m_AnalyzeB.setFrmRate( m_pcEncCfg->m_FrameRate*rateMultiplier / (double)m_pcEncCfg->m_temporalSubsampleRatio);

  const ChromaFormat chFmt = m_pcEncCfg->m_internChromaFormat;

  //-- all
  msg( VVENC_INFO, "\n" );
  msg( VVENC_DETAILS,"\nSUMMARY --------------------------------------------------------\n" );
  m_AnalyzeAll.printOut('a', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths
                          );
  msg( VVENC_DETAILS,"\n\nI Slices--------------------------------------------------------\n" );
  m_AnalyzeI.printOut('i', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths);

  msg( VVENC_DETAILS,"\n\nP Slices--------------------------------------------------------\n" );
  m_AnalyzeP.printOut('p', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths);

  msg( VVENC_DETAILS,"\n\nB Slices--------------------------------------------------------\n" );
  m_AnalyzeB.printOut('b', chFmt, printMSEBasedSNR, printSequenceMSE, printHexPsnr, bitDepths);

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


// ====================================================================================================================
// protected
// ====================================================================================================================


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
    return VVENC_NAL_UNIT_CODED_SLICE_IDR_N_LP;
  }

  if (m_pcEncCfg->m_DecodingRefreshType != 3 && pocCurr % m_pcEncCfg->m_IntraPeriod == 0)
  {
    if (m_pcEncCfg->m_DecodingRefreshType == 1)
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


bool EncGOP::xIsSliceTemporalSwitchingPoint( const Slice* slice, PicList& picList, int gopId ) const
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


void EncGOP::xInitPicsInCodingOrder( const std::vector<Picture*>& encList, PicList& picList, bool isEncodeLtRef )
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
    }
  }
}


void EncGOP::xInitFirstSlice( Picture& pic, PicList& picList, bool isEncodeLtRef )
{
  const int curPoc      = pic.getPOC();
  const int gopId       = pic.gopId;
  const int depth       = xGetSliceDepth( curPoc );
  memset( pic.cs->alfAps, 0, sizeof(pic.cs->alfAps));
  Slice* slice          = pic.allocateNewSlice();
  pic.cs->picHeader     = new PicHeader;
  const SPS& sps        = *(slice->sps);
  SliceType sliceType   = ( curPoc % (unsigned)(m_pcEncCfg->m_IntraPeriod) == 0 || m_pcEncCfg->m_GOPList[ gopId ].m_sliceType== 'I' ) ? ( VVENC_I_SLICE ) : ( m_pcEncCfg->m_GOPList[ gopId ].m_sliceType== 'P' ? VVENC_P_SLICE : VVENC_B_SLICE );
  vvencNalUnitType naluType  = xGetNalUnitType( curPoc, m_lastIDR );

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
  slice->TLayer                    = m_pcEncCfg->m_GOPList[ gopId ].m_temporalId;
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
  xSelectReferencePictureList( slice, curPoc, gopId, -1 );
  if ( slice->checkThatAllRefPicsAreAvailable( picList, slice->rpl[0], 0, false ) != 0 || slice->checkThatAllRefPicsAreAvailable( picList, slice->rpl[1], 1, false ) != 0 )
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

  for( int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
    slice->deblockingFilterTcOffsetDiv2[comp]    = slice->picHeader->deblockingFilterTcOffsetDiv2[comp]   = slice->pps->deblockingFilterTcOffsetDiv2[comp];
    slice->deblockingFilterBetaOffsetDiv2[comp]  = slice->picHeader->deblockingFilterBetaOffsetDiv2[comp] = slice->pps->deblockingFilterBetaOffsetDiv2[comp];
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
    if( sliceType == VVENC_B_SLICE )
    {
      pic.cs->picHeader->disDmvrFlag = true;

      xUpdateRPRtmvp( pic.cs->picHeader, slice );
      xUpdateRPRToolCtrl( pic.cs->picHeader, slice );
    }
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

  if ((m_pcEncCfg->m_usePerceptQPA || (m_pcEncCfg->m_RCNumPasses == 2)) && (m_pcEncCfg->m_usePerceptQPATempFiltISlice || !slice->isIntra()))
  {
    for (auto& picItr : picList) // find previous frames
    {
      if (picItr->poc + 1 == curPoc) pic.m_bufsOrigPrev[0] = &picItr->m_bufs[PIC_ORIGINAL];
      if (picItr->poc + 2 == curPoc) pic.m_bufsOrigPrev[1] = &picItr->m_bufs[PIC_ORIGINAL];
    }
  }

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

void EncGOP::xUpdateRPRToolCtrl( PicHeader* picHeader, Slice* slice )
{
  for( int refIdx = 0; refIdx < slice->numRefIdx[REF_PIC_LIST_0]; refIdx++ )
  {
    if( slice->getRefPic( REF_PIC_LIST_0, refIdx )->poc <= m_pocCRA &&
        slice->getRefPic( REF_PIC_LIST_0, refIdx )->slices[0]->nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_RASL )
    {
      picHeader->disBdofFlag = true;
      picHeader->disProfFlag = true;

      return;
    }
}

  for( int refIdx = 0; refIdx < slice->numRefIdx[REF_PIC_LIST_1]; refIdx++ )
  {
    if( slice->getRefPic( REF_PIC_LIST_1, refIdx )->poc <= m_pocCRA &&
        slice->getRefPic( REF_PIC_LIST_1, refIdx )->slices[0]->nalUnitType != VVENC_NAL_UNIT_CODED_SLICE_RASL )
    {
      picHeader->disBdofFlag = true;
      picHeader->disProfFlag = true;

      return;
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
      CPelUnitBuf origBuf   = pic.getOrigBuf();
      if( pic.getFilteredOrigBuffer().valid() )
      {
        pic.getRspOrigBuf().get(COMP_Y).rspSignal( m_Reshaper.getFwdLUT());
      }
      else
      {
        pic.getFilteredOrigBuffer().create( pic.cs->pcv->chrFormat, Area( 0, 0, origBuf.get( COMP_Y ).width, origBuf.get( COMP_Y ).height) );
        PelUnitBuf rspOrigBuf = pic.getRspOrigBuf();
        rspOrigBuf.get(COMP_Y).rspSignal( origBuf.get(COMP_Y), m_Reshaper.getFwdLUT());
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
      int POCIndex = curPoc%m_pcEncCfg->m_IntraPeriod;
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
    m_pcEncHRD->bufferingPeriodSEI = *bufferingPeriodSEI; 
    m_pcEncHRD->bufferingPeriodInitialized = true;
    
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
  m_seiWriter.writeSEImessages(nalu.m_Bitstream, seiMessages, *m_pcEncHRD, false, temporalId);
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
    m_seiWriter.writeSEImessages(nalu.m_Bitstream, tmpMessages, *m_pcEncHRD, false, temporalId);
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
        msg( VVENC_NOTICE, "Adding %d bytes of padding\n", numberOfAdditionalCabacZeroWords * 3 );
      }
      else
      {
        msg( VVENC_NOTICE, "Standard would normally require adding %d bytes of padding\n", numberOfAdditionalCabacZeroWords * 3 );
      }
    }
  }
}

void EncGOP::picInitRateControl( int gopId, Picture& pic, Slice* slice, EncPicture* picEncoder )
{
  if( m_pcEncCfg->m_RCTargetBitrate == 0 ) // TODO: does this work with multiple slices and slice-segments?
  {
    return;
  }

  const int frameLevel = (slice->isIntra() ? 0 : slice->TLayer + 1);
  EncRCPic* encRCPic   = pic.encRCPic;
  double lambda = (m_pcEncCfg->m_RCNumPasses != 2 ? encRCPic->finalLambda : m_pcRateCtrl->encRCGOP->maxEstLambda);
  int   sliceQP = (m_pcEncCfg->m_RCNumPasses != 2 ? m_pcEncCfg->m_RCInitialQP : MAX_QP);

  if ((m_pcEncCfg->m_RCNumPasses != 2) && ((slice->poc == 0 && m_pcEncCfg->m_RCInitialQP > 0) || (frameLevel == 0 && m_pcEncCfg->m_RCForceIntraQP))) // QP is specified
  {
    int    NumberBFrames = ( m_pcEncCfg->m_GOPSize - 1 );
    double dLambda_scale = 1.0 - Clip3( 0.0, 0.5, 0.05 * (double)NumberBFrames );
    double dQPFactor = 0.57 * dLambda_scale;
    int    SHIFT_QP = 12;
    int bitdepth_luma_qp_scale = 6 * ( slice->sps->bitDepths[CH_L] - 8
      - DISTORTION_PRECISION_ADJUSTMENT( slice->sps->bitDepths[CH_L] ) );
    double qp_temp = (double)sliceQP + bitdepth_luma_qp_scale - SHIFT_QP;
    lambda = dQPFactor * pow( 2.0, qp_temp / 3.0 );
  }
  else if (frameLevel <= 7)
  {
    if (m_pcEncCfg->m_RCNumPasses == 2)
    {
      EncRCSeq* encRCSeq = m_pcRateCtrl->encRCSeq;
      std::list<TRCPassStats>::iterator it;

      for (it = encRCSeq->firstPassData.begin(); it != encRCSeq->firstPassData.end(); it++)
      {
        if ((it->poc == slice->poc) && (encRCPic->targetBits > 0) && (it->numBits > 0))
        {
          double d = (double) encRCPic->targetBits;
          const int firstPassSliceQP = it->qp;
          const int log2HeightMinus7 = int (0.5 + log ((double) std::max (128, m_pcEncCfg->m_SourceHeight)) / log (2.0)) - 7;
          uint16_t visAct = it->visActY;

          if (it->isNewScene) // spatiotemporal visual activity is transient at camera/scene change, find next steady-state activity
          {
            std::list<TRCPassStats>::iterator itNext = it;

            itNext++;
            while (itNext != encRCSeq->firstPassData.end() && !itNext->isIntra)
            {
              if (itNext->poc == it->poc + 2)
              {
                visAct = itNext->visActY;
                break;
              }
              itNext++;
            }
          }
          if (it->refreshParameters) encRCSeq->qpCorrection[frameLevel] = encRCSeq->actualBitCnt[frameLevel] = encRCSeq->targetBitCnt[frameLevel] = 0;
          CHECK (slice->TLayer >= 7, "analyzed RC frame must have TLayer < 7");

          // try to hit target rate more aggressively in last coded frames, lambda/QP clipping below will ensure smooth value change
          if (it->poc >= m_pcRateCtrl->flushPOC)
          {
            d = std::max (1.0, d + (encRCSeq->estimatedBitUsage - encRCSeq->bitsUsed) * 0.5 * it->frameInGopRatio);
            encRCPic->targetBits = int (d + 0.5); // update the member to be on the safe side
          }
          d /= (double) it->numBits;
          d = firstPassSliceQP - (105.0 / 128.0) * sqrt ((double) std::max (1, firstPassSliceQP)) * log (d) / log (2.0);
          sliceQP = int (0.5 + d + 0.125 * log2HeightMinus7 * std::max (0.0, 24.0 + 0.001/*log2HeightMinus7*/ * (log ((double) visAct) / log (2.0) - 0.5 * encRCSeq->bitDepth - 3.0) - d) + encRCSeq->qpCorrection[frameLevel]);
          encRCPic->clipTargetQP (m_pcRateCtrl->getPicList(), sliceQP);  // temp. level based
          lambda  = it->lambda * pow (2.0, double (sliceQP - firstPassSliceQP) / 3.0);
          lambda  = Clip3 (m_pcRateCtrl->encRCGOP->minEstLambda, m_pcRateCtrl->encRCGOP->maxEstLambda, lambda);

          if (it->isIntra) // update history, for parameter clipping in subsequent key frames
          {
            encRCSeq->lastIntraLambda = lambda;
            encRCSeq->lastIntraQP     = sliceQP;
          }
          break;
        }
      }
    }
    else // single-pass rate control
    {
      if (frameLevel == 0)
      {
        encRCPic->calCostSliceI( &pic );

        if (m_pcEncCfg->m_IntraPeriod != 1) // don't refine allocated bits for All-Intra case
        {
          int bits = m_pcRateCtrl->encRCSeq->totalFrames > 0 ? m_pcRateCtrl->encRCSeq->getLeftAverageBits() : m_pcRateCtrl->encRCSeq->averageBits;
          bits = encRCPic->getRefineBitsForIntra( bits );

          if( bits < 200 )
          {
            bits = 200;
          }
          encRCPic->targetBits = bits;
          encRCPic->bitsLeft = bits;
        }
      } // (frameLevel == 0)

      std::list<EncRCPic*> listPreviousPicture = m_pcRateCtrl->getPicList();
      lambda = encRCPic->estimatePicLambda( listPreviousPicture, slice->isIRAP() );
      sliceQP = encRCPic->estimatePicQP( lambda, listPreviousPicture );
      sliceQP = Clip3 (0, MAX_QP, sliceQP);
    } // 1-pass rate control
  }

  picEncoder->getEncSlice()->resetQP (&pic, sliceQP, lambda);
  encRCPic->finalLambda = lambda;
}

void EncGOP::xUpdateAfterPicRC( const Picture* pic )
{
  EncRCPic* encRCPic = pic->encRCPic;
  if ( m_pcEncCfg->m_RCTargetBitrate == 0 )
  {
    return;
  }

  if (!m_pcRateCtrl->encRCSeq->twoPass)
  {
    encRCPic->calPicMSE();
  }
  encRCPic->updateAfterPicture( pic->actualHeadBits, pic->actualTotalBits, pic->slices[ 0 ]->sliceQp, pic->slices[ 0 ]->lambdas[ COMP_Y ], pic->slices[ 0 ]->isIRAP() );
  encRCPic->addToPictureList( m_pcRateCtrl->getPicList() );

  m_pcRateCtrl->encRCSeq->updateAfterPic( pic->actualTotalBits, encRCPic->tmpTargetBits );
  if (!m_pcRateCtrl->encRCSeq->twoPass)
  {
    if ( !pic->slices[ 0 ]->isIRAP() )
    {
      m_pcRateCtrl->encRCGOP->updateAfterPicture( pic->actualTotalBits );
    }
    else    // for intra picture, the estimated bits are used to update the current status in the GOP
    {
      m_pcRateCtrl->encRCGOP->updateAfterPicture( encRCPic->estimatedBits );
    }
  }

  return;
}

void EncGOP::xCalculateAddPSNR( const Picture* pic, CPelUnitBuf cPicD, AccessUnitList& accessUnit, bool printFrameMSE, double* PSNR_Y, bool isEncodeLtRef )
{
  const SPS&         sps = *pic->cs->sps;
  const CPelUnitBuf& org = pic->getOrigBuf();
  double  dPSNR[MAX_NUM_COMP];
  double  visualActivity = 0.0;

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
      msg( VVENC_NOTICE, "*** %s numBytesInNALunit: %u\n", nalUnitTypeToString((*it)->m_nalUnitType), numRBSPBytes_nal);
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

  if ((m_pcEncCfg->m_RCNumPasses == 2) && (m_pcRateCtrl->rcPass < m_pcRateCtrl->rcMaxPass))
  {
    visualActivity = (pic->picVisActY > 0.0 ? pic->picVisActY : BitAllocation::getPicVisualActivity (slice, m_pcEncCfg));
  }
  m_pcRateCtrl->addRCPassStats (slice->poc, slice->sliceQp, slice->getLambdas()[0], ClipBD (uint16_t (0.5 + visualActivity), m_pcEncCfg->m_internalBitDepth[CH_L]),
                                uibits, dPSNR[COMP_Y], slice->isIntra(), slice->TLayer);

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
    if( ! m_pcRateCtrl->rcIsFinalPass )
    {
      std::string cInfo = print("RC pass %d/%d, analyze poc %d",
          m_pcRateCtrl->rcPass + 1,
          m_pcRateCtrl->rcMaxPass + 1,
          slice->poc );

          accessUnit.InfoString.append( cInfo );

          msg( VVENC_NOTICE, cInfo.c_str() );
    }
    else
    {
      std::string cInfo = print("POC %4d TId: %1d (%10s, %c-SLICE, QP %d ) %10d bits",
          slice->poc,
          slice->TLayer,
          nalUnitTypeToString( slice->nalUnitType ),
          c,
          slice->sliceQp,
          uibits );

      std::string cPSNR = print(" [Y %6.4lf dB    U %6.4lf dB    V %6.4lf dB]", dPSNR[COMP_Y], dPSNR[COMP_Cb], dPSNR[COMP_Cr] );

      accessUnit.InfoString.append( cInfo );
      accessUnit.InfoString.append( cPSNR );

      msg( VVENC_NOTICE, cInfo.c_str() );
      msg( VVENC_NOTICE, cPSNR.c_str() );


      if ( m_pcEncCfg->m_printHexPsnr )
      {
        uint64_t xPsnr[MAX_NUM_COMP];
        for (int i = 0; i < MAX_NUM_COMP; i++)
        {
          std::copy(reinterpret_cast<uint8_t *>(&dPSNR[i]),
              reinterpret_cast<uint8_t *>(&dPSNR[i]) + sizeof(dPSNR[i]),
              reinterpret_cast<uint8_t *>(&xPsnr[i]));
        }

        std::string cPSNRHex = print(" [xY %16" PRIx64 " xU %16" PRIx64 " xV %16" PRIx64 "]", xPsnr[COMP_Y], xPsnr[COMP_Cb], xPsnr[COMP_Cr]);

        accessUnit.InfoString.append( cPSNRHex );
        msg(VVENC_NOTICE, cPSNRHex.c_str() );
      }

      if( printFrameMSE )
      {
        std::string cFrameMSE = print( " [Y MSE %6.4lf  U MSE %6.4lf  V MSE %6.4lf]", MSEyuvframe[COMP_Y], MSEyuvframe[COMP_Cb], MSEyuvframe[COMP_Cr]);
        accessUnit.InfoString.append( cFrameMSE );
        msg(VVENC_NOTICE, cFrameMSE.c_str() );
      }

      std::string cEncTime = print(" [ET %5d ]", pic->encTime.getTimerInSec() );
      accessUnit.InfoString.append( cEncTime );
      msg(VVENC_NOTICE, cEncTime.c_str() );

      std::string cRefPics;
      for( int iRefList = 0; iRefList < 2; iRefList++ )
      {
        std::string tmp = print(" [L%d ", iRefList);
        cRefPics.append( tmp );
        for( int iRefIndex = 0; iRefIndex < slice->numRefIdx[ iRefList ]; iRefIndex++ )
        {
          tmp = print("%d ", slice->getRefPOC( RefPicList( iRefList ), iRefIndex));
          cRefPics.append( tmp );
        }
        cRefPics.append( "]" );
      }
      accessUnit.InfoString.append( cRefPics );
      msg(VVENC_NOTICE, cRefPics.c_str() );
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

  if( m_pcRateCtrl->rcIsFinalPass )
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
      if ( digestStr.empty() )
      {
        msg( VVENC_NOTICE, " [%s:%s]", modeName.c_str(), "?" );
      }
      else
      {
        msg( VVENC_NOTICE, " [%s:%s]", modeName.c_str(), digestStr.c_str() );
      }
    }
  }

  msg( VVENC_NOTICE, "\n" );
  fflush( stdout );
}

} // namespace vvenc

//! \}


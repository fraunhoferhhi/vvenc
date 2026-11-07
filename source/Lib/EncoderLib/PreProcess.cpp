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


/** \file     PreProcess.cpp
    \brief    
*/


#include "PreProcess.h"
#include "BitAllocation.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {


PreProcess::PreProcess( MsgLog& _m )
  : m_encCfg     ( nullptr )
  , m_gopCfg     ( _m )
  , m_lastPoc    ( 0 )
  , m_isHighRes  ( false )
  , m_doSTA      ( false )
  , m_doTempDown ( false )
  , m_doVisAct   ( false )
  , m_doVisActQpa( false )
  , m_cappedCQF  ( false )
{
}


PreProcess::~PreProcess()
{
}


void PreProcess::init( const VVEncCfg& encCfg, bool isFinalPass, NoMallocThreadPool* threadPool )
{
  m_gopCfg.initGopList( encCfg.m_DecodingRefreshType, encCfg.m_poc0idr, encCfg.m_IntraPeriod, encCfg.m_GOPSize, encCfg.m_leadFrames, encCfg.m_picReordering, encCfg.m_GOPList, encCfg.m_vvencMCTF, encCfg.m_FirstPassMode, encCfg.m_minIntraDist );
  CHECK( m_gopCfg.getMaxTLayer() != encCfg.m_maxTLayer, "max temporal layer of gop configuration does not match pre-configured value" );
  if( encCfg.m_adaptiveIntra == 1 )
  {
    m_gopCfg.checkCutGopLists();
  }

  m_encCfg             = &encCfg;

  m_lastPoc            = std::numeric_limits<int>::min();
  m_isHighRes          = (std::min (m_encCfg->m_SourceWidth, m_encCfg->m_SourceHeight) > 1280);

  m_doSTA              = m_encCfg->m_sliceTypeAdapt > 0;
  m_cappedCQF          = m_encCfg->m_RCNumPasses != 2 && m_encCfg->m_rateCap;
  m_doTempDown         = m_encCfg->m_FirstPassMode == 2 || m_encCfg->m_FirstPassMode == 4;
  m_doVisAct           = m_encCfg->m_usePerceptQPA
                         || (m_encCfg->m_LookAhead && m_encCfg->m_RCTargetBitrate > 0)
                         || (m_encCfg->m_RCNumPasses > 1 && (!isFinalPass));
  m_doVisActQpa        = m_encCfg->m_usePerceptQPA;
  m_lookahead          = m_encCfg->m_adaptiveIntra == 1 ? m_encCfg->m_GOPSize + 1 : 0;   // mode 2 keeps the GOP structure: no look-ahead, no cuts
  m_lastSeenPoc        = std::numeric_limits<int>::min();
  if( m_encCfg->m_adaptiveIntra > 0 )
  {
    // the cost tests' motion search with one setting (the search pattern of MCTFSpeed 4, as in presets fast and faster
    // and in every first pass), so that both passes of a two-pass encode and every preset decide alike
    m_motionSearchCfg = encCfg;
    m_motionSearchCfg.m_vvencMCTF.MCTFSpeed = 4;
    m_motionSearch.init( m_motionSearchCfg, true, threadPool );
  }
}


void PreProcess::initPicture( Picture* pic )
{
}


void PreProcess::processPictures( const PicList& picList, AccessUnitList& auList, PicList& doneList, PicList& freeList )
{
  // the scene cut detector sees every picture as it arrives, ahead of the GOP structure
  if( ! picList.empty() && picList.back()->poc > m_lastSeenPoc )
  {
    Picture* newest = picList.back();
    m_lastSeenPoc   = newest->poc;
    if( ( m_encCfg->m_fga || m_lookahead > 0 ) && ! newest->m_picShared->isLeadTrail() )
    {
      xDetectSceneCut( newest, picList );
    }
  }

  // continue with next poc: the oldest picture without a GOP entry, once the look-ahead behind it is filled (or at flush)
  Picture* next = nullptr;
  for( auto p : picList )
  {
    if( p->poc > m_lastPoc && ( ! next || p->poc < next->poc ) ) next = p;
  }
  if( next && ( picList.back()->poc >= next->poc + m_lookahead || picList.back()->isFlush ) )
  {
    auto pic = next;

    if( m_encCfg->m_adaptiveIntra > 0 && m_gopCfg.nextStartsGop() )
    {
      // the costs of pictures before this GOP are not needed any more
      m_predCosts.erase( m_predCosts.begin(), m_predCosts.lower_bound( { pic->poc, std::numeric_limits<int>::min() } ) );
      m_dcIntraCosts.erase( m_dcIntraCosts.begin(), m_dcIntraCosts.lower_bound( pic->poc ) );
    }

    // set gop entry
    m_gopCfg.getNextGopEntry( pic->m_picShared->m_gopEntry );
    CHECK( pic->m_picShared->m_gopEntry.m_POC != pic->poc, "invalid state" );

    if( ! pic->m_picShared->isLeadTrail() )
    {
      // link previous frames
      xLinkPrevQpaBufs( pic, picList );

      // compute visual activity
      xGetVisualActivity( pic, picList );

      // detect scc
      xDetectScc( pic );

      // slice type adaptation
      if( m_doSTA && pic->gopEntry->m_temporalId == 0 )
      {
        // detect scene cut in gop and adapt slice type
        xDetectSTA( pic, picList );

        // disable temporal downsampling for gop with scene cut
        if( m_doTempDown && pic->gopEntry->m_scType == SCT_TL0_SCENE_CUT )
        {
          xDisableTempDown( pic, picList );
        }
      }

      // disable temporal downsampling in lowest 3 temp. layers
      // with one-pass RC and QPA; this stabilizes the RC a bit
      if( m_doTempDown && m_doVisActQpa && pic->gopEntry->m_temporalId <= 2 && m_encCfg->m_LookAhead && m_encCfg->m_RCTargetBitrate > 0 )
      {
        xDisableTempDown( pic, picList, 0 /*for faster - TODO: 2 for fast*/ );
      }
    }
    else if( pic->gopEntry->m_temporalId == 0 )
    {
      xGetVisualActivity( pic, picList );
    }

    // cleanup pic list
    xFreeUnused( pic, picList, doneList, freeList );

    m_lastPoc = pic->poc;
  }
  else if( ! picList.empty() && picList.back()->isFlush  )
  {
    // first flush call, fix start of last gop
    if( ! picList.empty() )
    {
      Picture* pic = xGetStartOfLastGop( picList );
      if( pic )
      {
        m_gopCfg.fixStartOfLastGop( pic->m_picShared->m_gopEntry );
        // compute visual activity for start of last GOP
        // this works in combination with xUpdateVAStartOfLastGop()
        xGetVisualActivity( pic, picList );
      }
    }

    // cleanup when flush is set and all pics are done
    for( auto pic : picList )
    {
      pic->m_picShared->freeLowRes();
      freeList.push_back( pic );
    }
  }
}


void PreProcess::xFreeUnused( Picture* pic, const PicList& picList, PicList& doneList, PicList& freeList ) const
{
  // current picture is done
  doneList.push_back( pic );

  // free unused previous frames
  bool foundTl0       = ! m_doSTA; // is sta is off, not need to keep previous tl0 pic
  int idx             = 0;
  Picture* startOfGop = xGetStartOfLastGop( picList, pic->poc );
  for( auto itr = picList.rbegin(); itr != picList.rend(); itr++ )
  {
    Picture* tp  = *itr;
    if( tp->poc > pic->poc ) continue;   // still waiting for its GOP entry (adaptive intra refresh look-ahead)
    bool keepPic = false;

    // keep previous frames for visual activity
    keepPic  |= ( m_doVisAct || m_cappedCQF || m_encCfg->m_GOPQPA ) && idx < NUM_QPA_PREV_FRAMES;
    // keep previous (first) tl0 pic for sta
    keepPic  |= ! foundTl0 && ( tp->gopEntry->m_temporalId == 0 || m_doTempDown );
    foundTl0 |=                 tp->gopEntry->m_temporalId == 0;  // update found tl0
    // keep start of last gop
    keepPic  |= ( tp == startOfGop );

    if( ! keepPic )
    {
      tp->m_picShared->freeLowRes();
      freeList.push_back( tp );
    }
    idx++;
  }
}


void PreProcess::xGetPrevPics( const Picture* pic, const PicList& picList, const Picture* prevPics[ NUM_QPA_PREV_FRAMES ] ) const
{
  std::fill_n( prevPics, NUM_QPA_PREV_FRAMES, nullptr );

  // find previous pics
  int prevPoc = pic->poc;
  int prevIdx = 0;
  for( auto itr = picList.rbegin(); itr != picList.rend() && prevIdx < NUM_QPA_PREV_FRAMES; itr++ )
  {
    Picture* tp = *itr;
    if( tp->poc >= pic->poc )
      continue;
    if( tp->poc != prevPoc - 1 )
      break;
    prevPics[ prevIdx ] = tp;
    prevPoc -= 1;
    prevIdx += 1;
  }
  // if first prev not found, set link to picture itself
  if( prevPics[ 0 ] == nullptr )
  {
    prevPics[ 0 ] = pic;
  }
}


Picture* PreProcess::xGetPrevTL0Pic( const Picture* pic, const PicList& picList ) const
{
  // find previous tl0 picture
  Picture* prevTL0 = nullptr;
  for( auto itr = picList.rbegin(); itr != picList.rend(); itr++ )
  {
    Picture* tp = *itr;
    if( tp == pic || tp->poc > pic->poc )   // newer pictures have no GOP entry yet
      continue;
    if( tp->gopEntry->m_temporalId == 0 )
    {
      prevTL0 = tp;
      break;
    }
  }
  return prevTL0;
}


Picture* PreProcess::xGetStartOfLastGop( const PicList& picList, int maxPoc ) const
{
  // use only non lead trail pics
  std::vector<Picture*> cnList;
  cnList.reserve( picList.size() );
  for( auto pic : picList )
  {
    if( ! pic->m_picShared->isLeadTrail() && pic->poc <= maxPoc )   // only pictures with a GOP entry
    {
      cnList.push_back( pic );
    }
  }

  if( cnList.empty() )
  {
    return nullptr;
  }

  // sort pics by coding number
  std::sort( cnList.begin(), cnList.end(), []( auto& a, auto& b ){ return a->gopEntry->m_codingNum < b->gopEntry->m_codingNum; } );

  // find start of current gop
  Picture* pic = cnList.back();
  const int poc0Offset = (m_encCfg->m_poc0idr ? -1 : 0); // place leading poc 0 idr in GOP -1
  const int lastGopNum = pic->gopEntry->m_gopNum + (pic->gopEntry->m_POC == 0 ? poc0Offset : 0);
  for( auto itr = cnList.rbegin(); itr != cnList.rend(); itr++ )
  {
    Picture* tp = *itr;
    const int tpGopNum = tp->gopEntry->m_gopNum + (tp->gopEntry->m_POC == 0 ? poc0Offset : 0);
    if( tpGopNum != lastGopNum )
    {
      return pic;
    }
    pic = tp;
  }
  return pic;
}


void PreProcess::xLinkPrevQpaBufs( Picture* pic, const PicList& picList ) const
{
  PicShared* picShared = pic->m_picShared;

  // find max previous pictures
  if( m_doVisAct )
  {
    const Picture* prevPics[ NUM_QPA_PREV_FRAMES ];
    xGetPrevPics( pic, picList, prevPics );
    for( int i = 0; i < NUM_QPA_PREV_FRAMES; i++ )
    {
      if( prevPics[ i ] )
      {
        picShared->m_prevShared[ i ] = prevPics[ i ]->m_picShared;
      }
    }
  }
}


void PreProcess::xGetVisualActivity( Picture* pic, const PicList& picList ) const
{
  VisAct va[ MAX_NUM_CH ];
  VisAct vaTL0;

  const bool doChroma           = m_cappedCQF && pic->gopEntry->m_isStartOfGop && isChromaEnabled( pic->chromaFormat );
  // for the time being qpa activity done on ctu basis in applyQPAdaptationSlice(), which for now sums up luma activity
  const bool doVisAct           = m_doVisAct && !m_doVisActQpa;
  const bool doSpatAct          = pic->gopEntry->m_isStartOfGop && m_encCfg->m_GOPQPA;
  const bool doVisActStartOfGOP = pic->gopEntry->m_isStartOfGop && m_cappedCQF;
  const bool doVisActTL0        = pic->gopEntry->m_temporalId == 0 && ( m_doSTA || m_cappedCQF || !doVisAct );

  // spatial activity
  if( doSpatAct || doVisAct || doVisActStartOfGOP || doVisActTL0 )
  {
    xGetSpatialActivity( pic, true, doChroma, va );
    // copy luma spatial activity for prev TL0
    if( doVisActTL0 )
    {
      vaTL0 = va[ CH_L ];
    }
  }

  // visual activity to previous pics (luma only)
  if( doVisAct || doVisActStartOfGOP )
  {
    // get temporal activity for picture
    const Picture* prevPics[ NUM_QPA_PREV_FRAMES ];
    xGetPrevPics( pic, picList, prevPics );
    xGetTemporalActivity( pic, prevPics[ 0 ], prevPics[ 1 ], va[ CH_L ] );
    // visual activity for picture
    updateVisAct( va[ CH_L ], m_encCfg->m_internalBitDepth[ CH_L ] );
  }

  // visual activity to previous TL0 pic (luma only)
  const Picture* prevTL0 = xGetPrevTL0Pic( pic, picList );
  if( doVisActTL0 && prevTL0 )
  {
    // get temporal activity for picture
    xGetTemporalActivity( pic, prevTL0, nullptr, vaTL0 );
    // visual activity for picture
    updateVisAct( vaTL0, m_encCfg->m_internalBitDepth[ CH_L ] );
  }

  // store visual activity
  PicShared* picShared               = pic->m_picShared;
  picShared->m_picVA.spatAct[ CH_L ] = ClipBD( (uint16_t)va[ CH_L ].spatAct, 12 );
  picShared->m_picVA.spatAct[ CH_C ] = ClipBD( (uint16_t)va[ CH_C ].spatAct, 12 );
  picShared->m_picVA.visAct          = va[ CH_L ].visAct;
  picShared->m_picVA.visActTL0       = vaTL0.visAct;
  if( prevTL0 )
  {
    picShared->m_picVA.prevTL0spatAct[ CH_L ] = prevTL0->m_picShared->m_picVA.spatAct[ CH_L ];
    picShared->m_picVA.prevTL0spatAct[ CH_C ] = prevTL0->m_picShared->m_picVA.spatAct[ CH_C ];
  }
  // update visual activity in pic, this is needed for STA detection in this stage
  pic->picVA = picShared->m_picVA;
}


void PreProcess::xGetSpatialActivity( Picture* pic, bool doLuma, bool doChroma, VisAct va[ MAX_NUM_CH ] ) const
{
  // luma part
  if( doLuma )
  {
    const int bitDepth = m_encCfg->m_internalBitDepth[ CH_L ];
    CPelBuf origBuf    = pic->getOrigBuf( COMP_Y );
    calcSpatialVisAct( origBuf.buf, origBuf.stride, origBuf.height, origBuf.width, bitDepth, m_isHighRes, va[ CH_L ] );
  }

  // chroma part
  if( doChroma )
  {
    const int bitDepth = m_encCfg->m_internalBitDepth[ CH_C ];
    const bool isUHD   = m_isHighRes && ( pic->chromaFormat == CHROMA_444 );
    const int numComp  = getNumberValidComponents( pic->chromaFormat );
    // accumulate spatial activity over chroma components
    for( int comp = 1; comp < numComp; comp++ )
    {
      VisAct chVA;
      const ComponentID compID = (ComponentID) comp;
      CPelBuf origBuf          = pic->getOrigBuf( compID );
      calcSpatialVisAct( origBuf.buf, origBuf.stride, origBuf.height, origBuf.width, bitDepth, isUHD, chVA );
      va[ CH_C ].hpSpatAct += chVA.hpSpatAct;
    }
    // mean value over chroma components
    va[ CH_C ].hpSpatAct = va[ CH_C ].hpSpatAct / (double)( numComp - 1 );
    // spatial in 12 bit
    va[ CH_C ].spatAct   = unsigned (0.5 + va[ CH_C ].hpSpatAct * double (bitDepth < 12 ? 1 << (12 - bitDepth) : 1));
  }
}


void PreProcess::xGetTemporalActivity( Picture* curPic, const Picture* refPic1, const Picture* refPic2, VisAct& va ) const
{
  CHECK( curPic == nullptr || refPic1 == nullptr, "no pictures given to compute visual activity" );

  const int bitDepth = m_encCfg->m_internalBitDepth[ CH_L ];

  CPelBuf origBufs[ 3 ];
  origBufs[ 0 ] = curPic->getOrigBuf( COMP_Y );
  origBufs[ 1 ] = refPic1->getOrigBuf( COMP_Y );
  if( refPic2 )
  {
    origBufs[ 2 ] = refPic2->getOrigBuf( COMP_Y );
  }

  calcTemporalVisAct( origBufs[0].buf, origBufs[0].stride, origBufs[0].height, origBufs[0].width,
                      origBufs[1].buf, origBufs[1].stride,
                      origBufs[2].buf, origBufs[2].stride,
                      m_encCfg->m_FrameRate / m_encCfg->m_FrameScale,
                      bitDepth,
                      m_isHighRes,
                      va
                    );
}


void PreProcess::xDetectSTA( Picture* pic, const PicList& picList )
{
  const Picture* prevTL0 = xGetPrevTL0Pic( pic, picList );

  int picMemorySTA  = 0;
  bool isSta        = false;
  bool intraAllowed = m_gopCfg.isSTAallowed( pic->poc );

  if( prevTL0 && prevTL0->m_picShared->sceneCutStart() )
  {
    intraAllowed = false;   // a scene cut picture's activity is measured across the cut; no reference for the next key picture
  }
  if( prevTL0 && prevTL0->picVA.visActTL0 > 0 && intraAllowed )
  {
    const int scThreshold = ( ( pic->isSccStrong ? 6 : ( pic->isSccWeak ? 5 : 4 ) ) * ( m_isHighRes ? 19 : 15 ) ) >> 2;

    if(        pic->picVA.visActTL0 * 11 > prevTL0->picVA.visActTL0 * scThreshold
        || prevTL0->picVA.visActTL0 * 11 > pic->picVA.visActTL0     * ( scThreshold + 1 ) )
    {
      const int dir = pic->picVA.visActTL0 < prevTL0->picVA.visActTL0 ? -1 : 1;
      picMemorySTA  = prevTL0->picVA.visActTL0 * dir;
      isSta         = ( picMemorySTA * prevTL0->picMemorySTA ) >= 0;
    }
  }

  // adaptive intra refresh: an activity jump alone does not say that an I picture is cheaper than predicting the
  // picture; turn a B picture into an I picture only when that pays (a picture that is an I picture already keeps the
  // scene cut marking as before)
  if( isSta && m_encCfg->m_adaptiveIntra > 0 && pic->gopEntry->m_sliceType != 'I' && prevTL0 && ! prevTL0->m_picShared->isLeadTrail()
      && ! xIntraPays( pic, prevTL0 ) )
  {
    isSta = false;
  }
  if( isSta )
  {
    PicShared* picShared              = pic->m_picShared;
    pic->picMemorySTA                 = picMemorySTA;
    picShared->m_picMemorySTA         = picMemorySTA;
    picShared->m_gopEntry.m_sliceType = 'I';
    picShared->m_gopEntry.m_scType    = SCT_TL0_SCENE_CUT;
    m_gopCfg.setLastIntraSTA( pic->poc );

    if( m_encCfg->m_sliceTypeAdapt == 2 )
    {
      m_gopCfg.startIntraPeriod( picShared->m_gopEntry );
    }
  }
}


// Adaptive intra refresh thresholds (the cost functions are below)
static constexpr double INTRA_COST_RATIO  = 1.0;   // an I picture pays when the prediction error exceeds this times the DC-intra estimate
static constexpr double CUT_STAY_RATIO    = 0.25;  // the picture after a cut stays in the scene when its prediction error is below this times its DC-intra estimate (the histogram detector's 0.075 against 0.3)

// The luma histogram of every 4th sample of every 4th row in 64 bins (the top 6 bits of a sample), counted into hist,
// which holds the 64 bins; returns the number of samples.
static int lumaHistogram( const CPelBuf& y, int bitDepth, std::vector<int>& hist )
{
  const int shift  = std::max( 0, bitDepth - 6 );
  const int topBin = (int)hist.size() - 1;
  int numSamples = 0;
  for( int row = 0; row < (int) y.height; row += 4 )
  {
    const Pel* src = y.bufAt( 0, row );
    for( int x = 0; x < (int) y.width; x += 4, numSamples++ )
    {
      hist[ std::min( topBin, std::max( 0, src[ x ] >> shift ) ) ]++;
    }
  }
  return numSamples;
}

// A picture starts a new scene when its luma histogram (64 bins, every 4th sample of every 4th row) differs from that
// of the picture before it by more than 0.3 (L1 distance of the normalized histograms, 0 to 2) and the next picture's
// stays within a quarter of that of it, so that a single flash frame is no cut. On three minutes of The Beekeeper (4316
// pictures) the distance is 0.014 at the median; the 41 cuts found are the same from thresholds 0.15 to 0.3, and
// no GOP of 32 pictures has two of them. The detector runs on every picture as it arrives; the cut is marked on the
// picture once the next one has arrived, and every picture carries the latest cut found before it, so that the film
// grain analysis of a GOP's first coded picture knows the cuts among the pictures it covers. With the adaptive intra
// refresh the cut is also reported to the GOP structure, which the look-ahead keeps a GOP behind the arrivals, and a
// second test on the look-ahead's motion search finds the cuts between shots of alike brightness, which the histogram
// does not see; those are the adaptive intra refresh's alone, the film grain analysis keeps the histogram's cuts.
void PreProcess::xDetectSceneCut( Picture* pic, const PicList& picList )
{
  std::vector<int> hist( FGA_HIST_BINS, 0 );
  const int numSamples = lumaHistogram( pic->getOrigBuf( COMP_Y ), m_encCfg->m_internalBitDepth[ CH_L ], hist );

  const bool follows  = pic->poc == m_fgaPrevPoc + 1;
  int        distance = 0;   // in samples: the L1 distance times numSamples
  for( int b = 0; follows && b < FGA_HIST_BINS; b++ )
  {
    distance += std::abs( hist[ b ] - m_fgaPrevHist[ b ] );
  }
  // the previous picture is a cut when it jumped from its predecessor (above 0.3) and this one stays (below 0.075)
  const bool histCut = follows && m_fgaPrevShared && m_fgaPrevDistance * 10 > 3 * numSamples && distance * 40 < 3 * numSamples;
  if( histCut )
  {
    m_fgaPrevShared->fgaSceneCut() = true;
    m_fgaLastCutPoc                = m_fgaPrevPoc;
  }
  // both cut rules of the adaptive intra refresh need the histogram to move at least half the detector's threshold,
  // so no other picture costs a motion search
  if( m_lookahead > 0 && follows && m_fgaPrevShared && m_fgaPrevDistance * 20 > 3 * numSamples )
  {
    xAdaptiveIntraCut( pic, picList, histCut );
  }
  pic->m_picShared->fgaLastSceneCutPoc() = m_fgaLastCutPoc;

  m_fgaPrevHist     = std::move( hist );
  m_fgaPrevPoc      = pic->poc;
  m_fgaPrevDistance = follows ? distance : 0;
  m_fgaPrevShared   = pic->m_picShared;
}

// Adaptive intra refresh: is the picture before pic a cut? The GOP before a cut ends at it, the cut picture is no
// reference for the slice type adaptation, and it starts a new intra period when an I picture pays there (a change in
// a flat scene predicts well enough across the cut and gets none). Besides the histogram's cuts, the previous picture
// is a cut when the motion search cannot predict it from its predecessor (an I picture pays) while pic predicts well
// from it (so a single flash frame is no cut): shots of alike brightness have no full histogram jump.
void PreProcess::xAdaptiveIntraCut( const Picture* pic, const PicList& picList, bool histCut )
{
  const Picture* cutPic   = xFindPoc( picList, m_fgaPrevPoc );
  const Picture* before   = xFindPoc( picList, m_fgaPrevPoc - 1 );
  const bool     haveBoth = cutPic && before;
  const bool     pays     = ! haveBoth || xIntraPays( cutPic, before );   // a histogram cut gets its I picture unless shown not to pay
  const bool     costCut  = haveBoth && pays && xPredCost( pic, cutPic ) <= CUT_STAY_RATIO * xDcIntraCost( pic );
  if( histCut || costCut )
  {
    m_fgaPrevShared->sceneCutStart() = true;
    m_gopCfg.addCut( m_fgaPrevPoc, pays );
  }
}

// ====================================================================================================================
// Adaptive intra refresh: does an I picture pay?
// ====================================================================================================================

static constexpr int    LOOKAHEAD_PAD     = 32;    // border of the low-resolution pictures, the motion search reads into it
static constexpr int    LOOKAHEAD_BLK     = 16;    // motion search block at quarter resolution (64x64 at full)

// Halves the luma resolution into dst, allocated when it has no buffer or one of another size (the half resolution
// stage buffer is kept from picture to picture, a picture's low-resolution planes are freed with it).
static void downsampleLuma( const CPelBuf& in, PelStorage& dst )
{
  // (a destroyed storage has no buffers; its Y() would still show the freed plane)
  if( dst.bufs.empty() || dst.Y().buf == nullptr || dst.Y().width != (int)in.width / 2 || dst.Y().height != (int)in.height / 2 )
  {
    dst.destroy();
    dst.create( CHROMA_400, Area( 0, 0, in.width / 2, in.height / 2 ), 0, LOOKAHEAD_PAD );
  }
  const PelBuf out = dst.Y();
  g_pelBufOP.downsample2x2( in.buf, in.stride, out.buf, out.stride, out.width, out.height );
  dst.extendBorderPel( LOOKAHEAD_PAD, LOOKAHEAD_PAD );
}

// Quarter, eighth and sixteenth resolution luma of a picture the cost tests use, made on first use (the half
// resolution step is only a stage); they are freed with the picture (xFreeUnused).
void PreProcess::xMakeLowRes( const Picture* pic )
{
  if( pic->m_picShared->hasLowRes() )
  {
    return;
  }
  downsampleLuma( pic->getOrigBuf( COMP_Y ), m_halfRes );
  downsampleLuma( m_halfRes.Y(), pic->m_picShared->lowRes( 0 ) );
  downsampleLuma( pic->m_picShared->lowRes( 0 ).Y(), pic->m_picShared->lowRes( 1 ) );
  downsampleLuma( pic->m_picShared->lowRes( 1 ).Y(), pic->m_picShared->lowRes( 2 ) );
}

// How well ref predicts cur: the summed squared error of the MCTF's integer motion search, started at a sixteenth of
// the resolution (its range of 8 there covers 128 samples of motion) and refined at an eighth and a quarter. Kept by
// (POC, reference POC).
double PreProcess::xPredCost( const Picture* cur, const Picture* ref )
{
  const std::pair<int, int> key( cur->poc, ref->poc );
  auto it = m_predCosts.find( key );
  if( it != m_predCosts.end() )
  {
    return it->second;
  }
  xMakeLowRes( cur );
  xMakeLowRes( ref );
  PicShared* c = cur->m_picShared;
  PicShared* r = ref->m_picShared;
  Array2D<MotionVector> mvs[ 3 ];
  for( int level = 2; level >= 0; level-- )
  {
    const CPelBuf y = c->lowRes( level ).Y();
    mvs[ level ].allocate( y.width / LOOKAHEAD_BLK + 1, y.height / LOOKAHEAD_BLK + 1 );
    if( level == 2 )
    {
      m_motionSearch.motionEstimationLuma( mvs[ level ], c->lowRes( level ), r->lowRes( level ), LOOKAHEAD_PAD, LOOKAHEAD_BLK );
    }
    else
    {
      m_motionSearch.motionEstimationLuma( mvs[ level ], c->lowRes( level ), r->lowRes( level ), LOOKAHEAD_PAD, LOOKAHEAD_BLK, &mvs[ level + 1 ], 2 );
    }
  }
  double cost = 0.0;
  for( int y = 0; y < mvs[ 0 ].h(); y++ )
  {
    for( int x = 0; x < mvs[ 0 ].w(); x++ )
    {
      const MotionVector& mv = mvs[ 0 ].get( x, y );
      if( mv.error != INT_LEAST32_MAX )
      {
        cost += mv.error;
      }
    }
  }
  m_predCosts[ key ] = cost;
  return cost;
}

// DC-intra estimate of a picture: the summed squared error of the quarter-resolution blocks against their mean, over
// the same blocks the motion search scores (16x16, at the right and bottom edges the remaining multiple of 8), so the
// two are in the same units. Kept by POC.
double PreProcess::xDcIntraCost( const Picture* pic )
{
  auto it = m_dcIntraCosts.find( pic->poc );
  if( it != m_dcIntraCosts.end() )
  {
    return it->second;
  }
  xMakeLowRes( pic );
  const CPelBuf y = pic->m_picShared->lowRes( 0 ).Y();
  double cost = 0.0;
  for( int by = 0; by + 8 <= (int) y.height; by += LOOKAHEAD_BLK )
  {
    const int h = std::min<int>( LOOKAHEAD_BLK, y.height - by ) & ~7;
    for( int bx = 0; bx + 8 <= (int) y.width; bx += LOOKAHEAD_BLK )
    {
      const int w = std::min<int>( LOOKAHEAD_BLK, y.width - bx ) & ~7;
      int64_t sum = 0, sq = 0;
      for( int j = 0; j < h; j++ )
      {
        const Pel* row = y.buf + ( by + j ) * y.stride + bx;
        for( int i = 0; i < w; i++ )
        {
          sum += row[ i ];
          sq  += row[ i ] * row[ i ];
        }
      }
      cost += double( sq ) - double( sum ) * double( sum ) / ( w * h );
    }
  }
  m_dcIntraCosts[ pic->poc ] = cost;
  return cost;
}

// An I picture pays at pic when predicting it from ref costs more than INTRA_COST_RATIO times its DC-intra estimate.
// (Hard cuts are typically 2 to 12 times the estimate, activity jumps that predict well below 0.4.)
bool PreProcess::xIntraPays( const Picture* pic, const Picture* ref )
{
  return xPredCost( pic, ref ) > INTRA_COST_RATIO * xDcIntraCost( pic );
}

const Picture* PreProcess::xFindPoc( const PicList& picList, int poc ) const
{
  for( auto p : picList )
  {
    if( p->poc == poc && ! p->m_picShared->isLeadTrail() )
    {
      return p;
    }
  }
  return nullptr;
}

void PreProcess::xDisableTempDown( Picture* pic, const PicList& picList, const int thresh /*= INT32_MAX*/ )
{
  for( auto itr = picList.rbegin(); itr != picList.rend(); itr++ )
  {
    Picture* tp = *itr;
    if( tp->poc > pic->poc ) continue;   // newer pictures have no GOP entry yet
    if( pic->gopEntry->m_gopNum != tp->gopEntry->m_gopNum )
      break;
    if( tp->gopEntry->m_temporalId <= thresh )
      tp->m_picShared->m_gopEntry.m_skipFirstPass = false;
  }
}


#if FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED && defined( __GNUC__ ) && __GNUC__ == 5
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-overflow"
#endif


void PreProcess::xDetectScc( Picture* pic ) const
{
  if( m_encCfg->m_forceScc > 0 )
  {
    pic->isSccStrong = pic->m_picShared->m_isSccStrong = m_encCfg->m_forceScc >= 3;
    pic->isSccWeak   = pic->m_picShared->m_isSccWeak   = m_encCfg->m_forceScc >= 2;
    return;
  }

  CPelBuf yuvOrgBuf = pic->getOrigBuf().Y();

  // blocksize and threshold
  static constexpr int SIZE_BL =  4;
  static constexpr int K_SC    = 23;
  static constexpr int K_noSC  =  8;

  // mean and variance fixed point accuracy
  static constexpr int accM = 4;
  static constexpr int accV = 2;

  static_assert( accM <= 4 && accV <= 4, "Maximum Mean and Variance accuracy of 4 allowed!" );
  static constexpr int shfM = 4 - accM;
  static constexpr int shfV = 4 + accM - accV;
  static constexpr int addM = 1 << shfM >> 1;
  static constexpr int addV = 1 << shfV >> 1;

  static constexpr int SizeS = SIZE_BL << 1;

  const int minLevel = 1 << ( m_encCfg->m_internalBitDepth[CH_L] - ( m_encCfg->m_videoFullRangeFlag ? 6 : 4 ) ); // 1/16th or 1/64th of range

  const Pel*     piSrc    = yuvOrgBuf.buf;
  const uint32_t uiStride = yuvOrgBuf.stride;
  const uint32_t uiWidth  = yuvOrgBuf.width;
  const uint32_t uiHeight = yuvOrgBuf.height;

  CHECK( ( uiWidth & 7 ) != 0 || ( uiHeight & 7 ) != 0, "Width and height have to be multiples of 8!" );

  const int amountBlock = ( uiWidth >> 2 ) * ( uiHeight >> 2 );

  int sR[4] = { 0, 0, 0, 0 }; // strong SCC data
  int zR[4] = { 0, 0, 0, 0 }; // zero input data

  // sums and absolute deviations of the 4x4 blocks of the two block rows of an 8x8 block row
  static_assert( SIZE_BL == 4 && accM == 4, "blockStats4x4 computes 4x4 block sums and deviations from 16 * sample" );
  const int numBlocks = uiWidth / SIZE_BL;
  std::vector<int> blkSum[2] = { std::vector<int>( numBlocks ), std::vector<int>( numBlocks ) };
  std::vector<int> blkDev[2] = { std::vector<int>( numBlocks ), std::vector<int>( numBlocks ) };
  const int bitDepth = m_encCfg->m_internalBitDepth[CH_L];

  for( int hh = 0; hh < uiHeight; hh += SizeS )
  {
    for( int r = 0; r < 2; r++ )
    {
      g_pelBufOP.blockStats4x4( &piSrc[( hh + r * SIZE_BL ) * uiStride], uiStride, numBlocks, bitDepth, blkSum[r].data(), blkDev[r].data() );
    }

    for( int ww = 0; ww < uiWidth; ww += SizeS )
    {
      int Rx = ww >= ( uiWidth  >> 1 ) ? 1 : 0;
      int Ry = hh >= ( uiHeight >> 1 ) ? 2 : 0;
      Ry = Ry | Rx;

      int n = 0;
      int Var[4];

      for( int j = 0; j < 2; j++ )
      {
        for( int i = ( ww / SIZE_BL ); i < ( ww / SIZE_BL ) + 2; i++ )
        {
          const int Mit = ( blkSum[j][i] + addM ) >> shfM;
          const int V   = blkDev[j][i];

          // if variance is lower than 1 and mean is lower/equal to minLevel
          if( V < ( 1 << ( accM + 4 ) ) && Mit <= ( minLevel << accM ) )
          {
            Var[n] = -1;
          }
          else
          {
            Var[n] = ( V + addV ) >> shfV;
          }

          n++;
        }
      }

      for( int i = 0; i < 2; i++ )
      {
        const int var0 = Var[ i];
        const int var1 = Var[ i + 2];
        const int var2 = Var[ i << 1];
        const int var3 = Var[(i << 1) + 1];

        if( var0 < 0 && var1 < 0 && zR[Ry] * 20 < amountBlock )
        {
          zR[Ry]++;
        }
        else if( var0 == var1 )
        {
          sR[Ry]++;
        }

        if( var2 < 0 && var3 < 0 && zR[Ry] * 20 < amountBlock )
        {
          zR[Ry]++;
        }
        else if( var2 == var3 )
        {
          sR[Ry]++;
        }
      }
    }
  }

  bool isSccWeak     = false;
  bool isSccStrong   = false;
  bool isNoSccStrong = false;

  int numAll   = 0;
  int numMin   = amountBlock, numMax = 0;
  int numBelow = 0;

  for( int r = 0; r < 4; r++ )
  {
    numAll   += sR[r];
    numMax    = std::max( numMax, sR[r] );
    numMin    = std::min( numMin, sR[r] );
    numBelow += sR[r] * 100 <= K_SC * ( amountBlock >> 2 ) ? 1 : 0;
  }

  // lowest quarter is above K_SC threshold
  isSccStrong   = numMin * 100 >  K_SC *   ( amountBlock >> 2 );
  // lowest quarter is below K_noSC threshold and theres more than one quarter below K_SC threshold
  isNoSccStrong = numMin * 100 <= K_noSC * ( amountBlock >> 2 ) && numBelow > 1;
  // overall is above K_SC threshold
  isSccWeak     = numAll * 100 >  K_SC *     amountBlock;
  // peak quarter is above 2.15*K_SC threshold
  isSccStrong  |= isSccWeak && !isNoSccStrong && numMax * 186 > K_SC * amountBlock;

  PicShared* picShared     = pic->m_picShared;
  pic->isSccWeak           = isSccWeak;
  pic->isSccStrong         = isSccStrong;
  picShared->m_isSccWeak   = isSccWeak;
  picShared->m_isSccStrong = isSccStrong;
}


#if FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED && defined( __GNUC__ ) && __GNUC__ == 5
#pragma GCC diagnostic pop
#endif


} // namespace vvenc

//! \}


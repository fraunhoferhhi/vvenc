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
  , m_doVisAct   ( false )
  , m_doVisActQpa( false )
{
}


PreProcess::~PreProcess()
{
}


void PreProcess::init( const VVEncCfg& encCfg, bool isFinalPass )
{
  m_gopCfg.initGopList( encCfg.m_DecodingRefreshType, encCfg.m_IntraPeriod, encCfg.m_GOPSize, encCfg.m_leadFrames, encCfg.m_picReordering, encCfg.m_GOPList, encCfg.m_vvencMCTF );
  CHECK( m_gopCfg.getMaxTLayer() != encCfg.m_maxTLayer, "max temporal layer of gop configuration does not match pre-configured value" );

  m_encCfg      = &encCfg;

  m_lastPoc     = std::numeric_limits<int>::min();
  m_isHighRes   = ( m_encCfg->m_PadSourceWidth > 2048 || m_encCfg->m_PadSourceHeight > 1280 );

  m_doSTA       = m_encCfg->m_sliceTypeAdapt > 0;
  m_doVisAct    =    m_encCfg->m_usePerceptQPA
                  || ( m_encCfg->m_LookAhead && m_encCfg->m_RCTargetBitrate )
                  || ( m_encCfg->m_RCNumPasses > 1 && ! isFinalPass );
  m_doVisActQpa = m_encCfg->m_usePerceptQPA;


}


void PreProcess::initPicture( Picture* pic )
{
}


void PreProcess::processPictures( const PicList& picList, bool flush, AccessUnitList& auList, PicList& doneList, PicList& freeList )
{
  // continue with next poc
  if( ! picList.empty() && picList.back()->poc > m_lastPoc )
  {
    auto pic = picList.back();

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

      // detect STA picture
      if( m_doSTA && pic->gopEntry->m_temporalId == 0 )
      {
        xDetectSTA( pic, picList );
      }
    }

    // cleanup pic list
    xFreeUnused( pic, picList, doneList, freeList );

    m_lastPoc = picList.back()->poc;
  }
  else if( flush )
  {
    // first flush call, fix start of last gop
    if( ! picList.empty() )
    {
      Picture* pic = xGetStartOfLastGop( picList );
      if( pic )
      {
        m_gopCfg.fixStartOfLastGop( pic->m_picShared->m_gopEntry );
      }
    }

    // cleanup when flush is set and all pics are done
    for( auto pic : picList )
    {
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
  Picture* startOfGop = xGetStartOfLastGop( picList );
  for( auto itr = picList.rbegin(); itr != picList.rend(); itr++, idx++ )
  {
    Picture* tp  = *itr;
    bool keepPic = false;

    // keep previous frames for visual activity
    keepPic  |= m_doVisAct && idx < NUM_QPA_PREV_FRAMES;
    // keep previous (first) tl0 pic for sta
    keepPic  |= ! foundTl0 && tp->gopEntry->m_temporalId == 0;
    foundTl0 |=               tp->gopEntry->m_temporalId == 0;  // update found tl0
    // keep start of last gop
    keepPic  |= ( tp == startOfGop );

    if( ! keepPic )
    {
      freeList.push_back( tp );
    }
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
    if( tp == pic )
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


Picture* PreProcess::xGetPrevTl0Pic( const Picture* pic, const PicList& picList ) const
{
  // find previous tl0 picture
  Picture* prevTl0 = nullptr;
  for( auto itr = picList.rbegin(); itr != picList.rend(); itr++ )
  {
    Picture* tp = *itr;
    if( tp == pic )
      continue;
    if( tp->gopEntry->m_temporalId == 0 )
    {
      prevTl0 = tp;
      break;
    }
  }
  return prevTl0;
}


Picture* PreProcess::xGetStartOfLastGop( const PicList& picList ) const
{
  // use only non lead trail pics
  std::vector<Picture*> cnList;
  cnList.reserve( picList.size() );
  for( auto pic : picList )
  {
    if( ! pic->m_picShared->isLeadTrail() )
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
  const int lastGopNum = pic->gopEntry->m_gopNum;
  for( auto itr = cnList.rbegin(); itr != cnList.rend(); itr++ )
  {
    Picture* tp = *itr;
    if( tp->gopEntry->m_gopNum != lastGopNum )
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
  uint16_t picVisActTL0 = 0;
  uint16_t picVisActY   = 0;

  if( m_doVisAct && ! m_doVisActQpa ) // for the time being qpa activity done on ctu basis in applyQPAdaptationSlice(), which for now sums up luma activity
  {
    // find previous pictures
    const Picture* prevPics[ NUM_QPA_PREV_FRAMES ];
    xGetPrevPics( pic, picList, prevPics );
    // get luma visual activity for whole picture
    CHECK( NUM_QPA_PREV_FRAMES < 2, "access out of array index" );
    picVisActY = xGetPicVisualActivity( pic, prevPics[ 0 ], prevPics[ 1 ] );
  }

  if( m_doSTA && pic->gopEntry->m_temporalId == 0 )
  {
    // find previous tl0 picture
    const Picture* prevTl0 = xGetPrevTl0Pic( pic, picList );
    // get visual activity to previous tl0 frame
    if( prevTl0 )
    {
      picVisActTL0 = xGetPicVisualActivity( pic, prevTl0, nullptr );
    }
  }

  PicShared* picShared      = pic->m_picShared;
  pic->picVisActTL0         = picVisActTL0;
  pic->picVisActY           = picVisActY;
  picShared->m_picVisActTL0 = picVisActTL0;
  picShared->m_picVisActY   = picVisActY;
}

uint16_t PreProcess::xGetPicVisualActivity( const Picture* curPic, const Picture* refPic1, const Picture* refPic2 ) const
{
  CHECK( curPic == nullptr || refPic1 == nullptr, "no pictures given to compute visual activity" );

  const int bitDepth = m_encCfg->m_internalBitDepth[ CH_L ];

  CPelBuf orig[ 3 ];
  orig[ 0 ] = curPic->getOrigBuf( COMP_Y );
  orig[ 1 ] = refPic1->getOrigBuf( COMP_Y );
  if( refPic2 )
    orig[ 2 ] = refPic2->getOrigBuf( COMP_Y );

  double visActY = filterAndCalculateAverageActivity(
      orig[ 0 ].buf,
      orig[ 0 ].stride,
      orig[ 0 ].height,
      orig[ 0 ].width,
      orig[ 1 ].buf,
      orig[ 1 ].stride,
      orig[ 2 ].buf,
      orig[ 2 ].stride,
      m_encCfg->m_FrameRate / m_encCfg->m_FrameScale,
      bitDepth,
      m_isHighRes,
      nullptr );

  uint16_t ret = ClipBD( (uint16_t)( 0.5 + visActY ), bitDepth );
  return ret;
}


void PreProcess::xDetectSTA( Picture* pic, const PicList& picList )
{
  const Picture* prevTl0 = xGetPrevTl0Pic( pic, picList );

  int picMemorySTA = 0;
  bool isSta       = false;

  if( prevTl0 && prevTl0->picVisActTL0 > 0 )
  {
    const int scThreshold = ( ( pic->isSccStrong ? 6 : ( pic->isSccWeak ? 5 : 4 ) ) * ( m_isHighRes ? 19 : 15 ) ) >> 2;

    if(        pic->picVisActTL0 * 11 > prevTl0->picVisActTL0 * scThreshold
        || prevTl0->picVisActTL0 * 11 > pic->picVisActTL0     * ( scThreshold + 1 ) )
    {
      const int dir = pic->picVisActTL0 < prevTl0->picVisActTL0 ? -1 : 1;
      picMemorySTA  = prevTl0->picVisActTL0 * dir;
      isSta         = ( picMemorySTA * prevTl0->picMemorySTA ) >= 0;
    }
  }

  if( isSta )
  {
    PicShared* picShared              = pic->m_picShared;
    pic->picMemorySTA                 = picMemorySTA;
    picShared->m_picMemorySTA         = picMemorySTA;
    picShared->m_gopEntry.m_sliceType = 'I';
    picShared->m_gopEntry.m_scType    = SCT_TL0_SCENE_CUT;

    if( m_encCfg->m_sliceTypeAdapt == 2 )
    {
      m_gopCfg.startIntraPeriod( picShared->m_gopEntry );
    }
  }
}


#if FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED && defined( __GNUC__ ) && __GNUC__ == 5
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-overflow"
#endif


void PreProcess::xDetectScc( Picture* pic ) const
{
  CPelUnitBuf yuvOrgBuf = pic->getOrigBuf();

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


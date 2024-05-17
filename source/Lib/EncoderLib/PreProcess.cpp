/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
{
}


PreProcess::~PreProcess()
{
}


void PreProcess::init( const VVEncCfg& encCfg, bool isFinalPass )
{
  m_gopCfg.initGopList( encCfg.m_DecodingRefreshType, encCfg.m_poc0idr, encCfg.m_IntraPeriod, encCfg.m_GOPSize, encCfg.m_leadFrames, encCfg.m_picReordering, encCfg.m_GOPList, encCfg.m_vvencMCTF, encCfg.m_FirstPassMode, encCfg.m_minIntraDist );
  CHECK( m_gopCfg.getMaxTLayer() != encCfg.m_maxTLayer, "max temporal layer of gop configuration does not match pre-configured value" );

  m_encCfg      = &encCfg;

  m_lastPoc     = std::numeric_limits<int>::min();
  m_isHighRes   = (std::min (m_encCfg->m_SourceWidth, m_encCfg->m_SourceHeight) > 1280);

  m_doSTA       = m_encCfg->m_sliceTypeAdapt > 0;
  m_doTempDown  = m_encCfg->m_FirstPassMode == 2 || m_encCfg->m_FirstPassMode == 4;
  m_doVisAct    = m_encCfg->m_usePerceptQPA
                  || (m_encCfg->m_LookAhead && m_encCfg->m_RCTargetBitrate > 0)
                  || (m_encCfg->m_RCNumPasses > 1 && (!isFinalPass));
  m_doVisActQpa = m_encCfg->m_usePerceptQPA;


}


void PreProcess::initPicture( Picture* pic )
{
}


void PreProcess::processPictures( const PicList& picList, AccessUnitList& auList, PicList& doneList, PicList& freeList )
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
    }

    // cleanup pic list
    xFreeUnused( pic, picList, doneList, freeList );

    m_lastPoc = picList.back()->poc;
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
        xGetVisualActivity( pic, picList );
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
    keepPic  |= ! foundTl0 && ( tp->gopEntry->m_temporalId == 0 || m_doTempDown );
    foundTl0 |=                 tp->gopEntry->m_temporalId == 0;  // update found tl0
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
  const bool cappedCRF  = ( m_encCfg->m_RCNumPasses != 2 && m_encCfg->m_rateCap );
  uint16_t picVisActTL0 = 0;
  uint16_t picVisActY   = 0;

  if( ( m_doVisAct && !m_doVisActQpa ) || ( cappedCRF && m_encCfg->m_usePerceptQPA && pic->gopEntry->m_isStartOfGop ) ) // for the time being qpa activity done on ctu basis in applyQPAdaptationSlice(), which for now sums up luma activity
  {
    // find previous pictures
    const Picture* prevPics[ NUM_QPA_PREV_FRAMES ];
    xGetPrevPics( pic, picList, prevPics );
    // get luma visual activity for whole picture
    CHECK( NUM_QPA_PREV_FRAMES < 2, "access out of array index" );
    picVisActY = xGetPicVisualActivity( pic, prevPics[ 0 ], prevPics[ 1 ], false );
  }

  if( ( m_doSTA || cappedCRF || !m_doVisAct || m_doVisActQpa ) && pic->gopEntry->m_temporalId == 0 )
  {
    // find previous tl0 picture
    const Picture* prevTl0 = xGetPrevTl0Pic( pic, picList );
    // get visual activity to previous tl0 frame
    if( prevTl0 )
    {
      picVisActTL0 = xGetPicVisualActivity( pic, prevTl0, nullptr, cappedCRF );
    }
    else if( !m_doVisAct || m_doVisActQpa )
    {
      xGetPicVisualActivity( pic, pic, nullptr, cappedCRF );
    }
  }

  PicShared* picShared           = pic->m_picShared;
  pic->picVisActTL0              = picVisActTL0;
  pic->picVisActY                = picVisActY;
  picShared->m_picVisActTL0      = picVisActTL0;
  picShared->m_picVisActY        = picVisActY;
  picShared->m_picSpVisAct[CH_L] = pic->picSpVisAct;
}

uint16_t PreProcess::xGetPicVisualActivity( Picture* curPic, const Picture* refPic1, const Picture* refPic2, const bool doChroma ) const
{
  CHECK( curPic == nullptr || refPic1 == nullptr, "no pictures given to compute visual activity" );

  const int bitDepth = m_encCfg->m_internalBitDepth[ CH_L ];
  unsigned minVisAct = 0, picSpVisAct = 0;

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
      &minVisAct,
      &picSpVisAct);

  if( doChroma )
  {
    unsigned sumVisActC = 0, picSpVisActC = 0;

    for (uint32_t comp = 1; comp < getNumberValidComponents (curPic->chromaFormat); comp++)
    {
      const ComponentID compID = (ComponentID) comp;

      orig[0] = curPic->getOrigBuf (compID);
      orig[1] = refPic1->getOrigBuf(compID);
      filterAndCalculateAverageActivity(orig[0].buf, orig[0].stride, orig[0].height, orig[0].width,
                                        orig[1].buf, orig[1].stride, orig[2].buf, orig[2].stride, 24,
                                        bitDepth, m_isHighRes && (curPic->chromaFormat == CHROMA_444), nullptr, &picSpVisActC);
      sumVisActC += picSpVisActC;
    }
    curPic->m_picShared->m_picSpVisAct[CH_C] = ClipBD (uint16_t ((sumVisActC + 1) >> 1), 12); // when available, get average CbCr chroma spatial visual activity
    curPic->m_picShared->m_minNoiseLevels[0] = uint8_t (minVisAct > 0 && minVisAct < 255 ? minVisAct : 255); // temporary storage of TL0 minimum visual activity
  }

  curPic->picSpVisAct = ClipBD( (uint16_t) picSpVisAct, 12 );

  return ClipBD( uint16_t( 0.5 + visActY ), bitDepth );
}


void PreProcess::xDetectSTA( Picture* pic, const PicList& picList )
{
  const Picture* prevTl0 = xGetPrevTl0Pic( pic, picList );

  int picMemorySTA  = 0;
  bool isSta        = false;
  bool intraAllowed = m_gopCfg.isSTAallowed( pic->poc );
  
  if( prevTl0 && prevTl0->picVisActTL0 > 0 && intraAllowed )
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
    m_gopCfg.setLastIntraSTA( pic->poc );

    if( m_encCfg->m_sliceTypeAdapt == 2 )
    {
      m_gopCfg.startIntraPeriod( picShared->m_gopEntry );
    }
  }
}


void PreProcess::xDisableTempDown( Picture* pic, const PicList& picList )
{
  for( auto itr = picList.rbegin(); itr != picList.rend(); itr++ )
  {
    Picture* tp = *itr;
    if( pic->gopEntry->m_gopNum != tp->gopEntry->m_gopNum )
      break;
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

  for( int hh = 0; hh < uiHeight; hh += SizeS )
  {
    for( int ww = 0; ww < uiWidth; ww += SizeS )
    {
      int Rx = ww >= ( uiWidth  >> 1 ) ? 1 : 0;
      int Ry = hh >= ( uiHeight >> 1 ) ? 2 : 0;
      Ry = Ry | Rx;

      int n = 0;
      int Var[4];

      for( int j = hh; j < hh + SizeS; j += SIZE_BL )
      {
        for( int i = ww; i < ww + SizeS; i += SIZE_BL )
        {
          const Pel *p0 = &piSrc[j * uiStride + i];

          int Mit = 0;
          int V   = 0;

          for( int h = 0; h < SIZE_BL; h++, p0 += uiStride )
          {
            for( int w = 0; w < SIZE_BL; w++ )
            {
              Mit += p0[w];
            }
          }

          Mit = ( Mit + addM ) >> shfM;

          p0 = &piSrc[j * uiStride + i];

          for( int h = 0; h < SIZE_BL; h++, p0 += uiStride )
          {
            for( int w = 0; w < SIZE_BL; w++ )
            {
              V += abs( Mit - ( int( p0[w] ) << accM ) );
            }
          }

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


/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

/** \file     EncStage.h
    \brief
*/

#pragma once

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
#include "CommonLib/Nal.h"

#include <vector>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

class PicShared
{
public:
  PicShared*       m_prevShared[ NUM_QPA_PREV_FRAMES ];
  GOPEntry         m_gopEntry;
  bool             m_isSccWeak;
  bool             m_isSccStrong;
  uint16_t         m_picVisActTL0;
  uint16_t         m_picVisActY;
  uint16_t         m_picSpatVisAct;
  int              m_picMemorySTA;
  uint16_t         m_picMotEstError;
  uint8_t          m_minNoiseLevels[QPA_MAX_NOISE_LEVELS];
  std::vector<int> m_ctuBimQpOffset;
  int              m_picAuxQpOffset; // auxiliary QP offset per frame, for combination of RC and BIM (and possibly other tools)

private:
  PelStorage       m_origBuf;
  PelStorage       m_filteredBuf;
  uint64_t         m_cts;
  int              m_maxFrames;
  int              m_poc;
  int              m_refCount;
  bool             m_isLead;
  bool             m_isTrail;
  bool             m_ctsValid;

public:
  PicShared()
  : m_isSccWeak     ( false )
  , m_isSccStrong   ( false )
  , m_picVisActTL0  ( 0 )
  , m_picVisActY    ( 0 )
  , m_picSpatVisAct   ( 0 )
  , m_picMemorySTA  ( 0 )
  , m_picMotEstError( 0 )
  , m_picAuxQpOffset( 0 )
  , m_cts           ( 0 )
  , m_maxFrames     ( -1 )
  , m_poc           ( -1 )
  , m_refCount      ( -1 )
  , m_isLead        ( false )
  , m_isTrail       ( false )
  , m_ctsValid      ( false )
  {
    std::fill_n( m_prevShared, NUM_QPA_PREV_FRAMES, nullptr );
    std::fill_n( m_minNoiseLevels, QPA_MAX_NOISE_LEVELS, 255u );
    m_gopEntry.setDefaultGOPEntry();
  };

  ~PicShared() {};

  bool         isUsed()          const { return m_refCount > 0; }
  void         incUsed()               { m_refCount += 1; }
  void         decUsed()               { CHECK( m_refCount <= 0, "invalid state: release unused picture" ); if( m_refCount > 0 ) m_refCount -= 1; }
  bool         isLeadTrail()     const { return m_isLead || m_isTrail; }
  int          getPOC()          const { return m_poc; }
  ChromaFormat getChromaFormat() const { return m_origBuf.chromaFormat; }
  Size         getLumaSize()     const { return m_origBuf.Y(); }

  void create( int maxFrames, ChromaFormat chromaFormat, const Size& size, bool useFilter )
  {
    CHECK( m_refCount >= 0, "PicShared already created" );

    m_maxFrames = maxFrames;
    m_refCount  = 0;

    const int padding = useFilter ? MCTF_PADDING : 0;
    m_origBuf.create( chromaFormat, Area( Position(), size ), 0, padding );
  }

  void reuse( int poc, const vvencYUVBuffer* yuvInBuf )
  {
    CHECK( m_refCount < 0, "PicShared not created" );
    CHECK( isUsed(),       "PicShared still in use" );

    if(m_origBuf.bufs[0].width < yuvInBuf->planes[0].width)
    {
      copyPadToPelUnitBufDown(m_origBuf, *yuvInBuf, getChromaFormat());
    }
    else
    {
      copyPadToPelUnitBuf(m_origBuf, *yuvInBuf, getChromaFormat());
    }

    m_isSccWeak    = false;
    m_isSccStrong  = false;
    m_picVisActTL0 = 0;
    m_picVisActY   = 0;
    m_picSpatVisAct = 0;
    m_picMemorySTA = 0;
    m_cts          = yuvInBuf->cts;
    m_poc          = poc;
    m_refCount     = 0;
    m_isLead       = poc < 0;
    m_isTrail      = m_maxFrames > 0 && poc >= m_maxFrames;
    m_ctsValid     = yuvInBuf->ctsValid;
    m_ctuBimQpOffset.resize( 0 );
    m_picMotEstError = 0;
    m_picAuxQpOffset = 0;
    std::fill_n( m_prevShared, NUM_QPA_PREV_FRAMES, nullptr );
    std::fill_n( m_minNoiseLevels, QPA_MAX_NOISE_LEVELS, 255u );
    m_gopEntry.setDefaultGOPEntry();
  }

  void shareData( Picture* pic )
  {
    PelStorage* prevOrigBufs[ NUM_QPA_PREV_FRAMES ];
    sharePrevBuffers( prevOrigBufs );
    pic->linkSharedBuffers( &m_origBuf, &m_filteredBuf, prevOrigBufs, this );
    pic->isSccWeak      = m_isSccWeak;
    pic->isSccStrong    = m_isSccStrong;
    pic->picVisActTL0   = m_picVisActTL0;
    pic->picVisActY     = m_picVisActY;
    pic->picSpatVisAct = m_picSpatVisAct;
    pic->picMemorySTA   = m_picMemorySTA;
    pic->poc            = m_poc;
    pic->cts            = m_cts;
    pic->ctsValid       = m_ctsValid;
    pic->gopEntry       = &m_gopEntry;
    incUsed();
  }

  void releaseShared( Picture* pic )
  {
    releasePrevBuffers( pic );
    pic->releaseSharedBuffers();
    decUsed();
  };

  void sharePrevBuffers( PelStorage* prevOrigBufs[ NUM_QPA_PREV_FRAMES ] )
  {
    for( int i = 0; i < NUM_QPA_PREV_FRAMES; i++ )
    {
      prevOrigBufs[ i ] = nullptr;
      if( m_prevShared[ i ] )
      {
        m_prevShared[ i ]->incUsed();
        prevOrigBufs[ i ] = &( m_prevShared[ i ]->m_origBuf );
      }
    }
  }

  void releasePrevBuffers( Picture* pic )
  {
    for( int i = 0; i < NUM_QPA_PREV_FRAMES; i++ )
    {
      if( m_prevShared[ i ] && pic->m_bufsOrigPrev[ i ] )
      {
        m_prevShared[ i ]->decUsed();
      }
    }
    pic->releasePrevBuffers();
  }
};

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class EncStage
{
private:
  EncStage* m_nextStage        { nullptr };
  PicList   m_procList         { };
  PicList   m_freeList         { };
  int       m_minQueueSize     { 0 };
  int       m_startPoc         { 0 };
  bool      m_flushAll         { false };
  bool      m_processLeadTrail { false };
  bool      m_sortByPoc        { false };
  int       m_ctuSize          { MAX_CU_SIZE };
  bool      m_isNonBlocking    { false };
  bool      m_flush            { false };

protected:
  int       m_picCount         { 0 };

public:
  EncStage()
  {
  };

  virtual ~EncStage()
  {
    freePicList();
  };

protected:
  virtual void initPicture    ( Picture* pic ) = 0;
  virtual void processPictures( const PicList& picList, AccessUnitList& auList, PicList& doneList, PicList& freeList ) = 0;

public:
  virtual void waitForFreeEncoders() {}

  void freePicList()
  {
    for( auto pic : m_procList )
    {
      pic->destroy( true );
      delete pic;
    }
    m_procList.clear();
    for( auto pic : m_freeList )
    {
      pic->destroy( true );
      delete pic;
    }
    m_freeList.clear();
  }

  bool isStageDone() const   { return m_procList.empty(); }
  bool isNonBlocking() const { return m_isNonBlocking; }

  void initStage( const VVEncCfg& encCfg, int minQueueSize, int startPoc, bool processLeadTrail, bool sortByPoc, bool nonBlocking )
  {
    CHECK( processLeadTrail && ! sortByPoc, "sort by coding number only for non lead trail pics supported" );
    m_minQueueSize     = minQueueSize;
    m_startPoc         = startPoc;
    m_processLeadTrail = processLeadTrail;
    m_sortByPoc        = sortByPoc;
    m_ctuSize          = encCfg.m_CTUSize;
    m_isNonBlocking    = nonBlocking;
  }

  void linkNextStage( EncStage* nextStage )
  {
    m_nextStage = nextStage;
    m_flushAll  = m_nextStage != nullptr;
    CHECK( m_flushAll && m_isNonBlocking, "only last stage is allowed to be a non-blocking stage" );
  }

  void addPicSorted( PicShared* picShared, bool flush )
  {
    // send lead trail data to next stage if not requested
    if( picShared->getPOC() < m_startPoc
        || ( ! m_processLeadTrail && picShared->isLeadTrail() ) )
    {
      if( m_nextStage )
      {
        m_nextStage->addPicSorted( picShared, flush );
      }
      return;
    }

    // setup new picture or recycle old one
    const ChromaFormat chromaFormat = picShared->getChromaFormat();
    const Size lumaSize             = picShared->getLumaSize();
    Picture* pic                    = nullptr;
    if( m_freeList.size() )
    {
      pic = m_freeList.front();
      m_freeList.pop_front();
    }
    else
    {
      pic = new Picture();
      pic->create( chromaFormat, lumaSize, m_ctuSize, m_ctuSize + 16, false );
    }
    CHECK( pic == nullptr, "out of memory" );
    CHECK( pic->chromaFormat != chromaFormat || pic->Y().size() != lumaSize, "resolution or format changed" );

    pic->reset();
    picShared->shareData( pic );

    // call first picture init
    initPicture( pic );

    // sort picture into processing queue
    PicList::iterator picItr;
    if( m_sortByPoc )
    {
      for( picItr = m_procList.begin(); picItr != m_procList.end(); picItr++ )
      {
        if( pic->poc < ( *picItr )->poc )
          break;
      }
    }
    else
    {
      for( picItr = m_procList.begin(); picItr != m_procList.end(); picItr++ )
      {
        CHECK( ! pic->gopEntry->m_isValid, "try to sort picture by invalid gop entry" );
        if( pic->gopEntry->m_codingNum < ( *picItr )->gopEntry->m_codingNum )
          break;
      }
    }

    // if flush is signalled, insert all new pictures with flush signal set
    // for non-blocking stage, mark all pictures after first flush picture in coding order as "have seen flush" themselves
    // assume this to work deterministically, because the coding of these pictures shouldn't have been started yet
    pic->isFlush = flush;
    if( m_isNonBlocking && pic->isFlush )
    {
      for( PicList::iterator cpyItr = picItr; cpyItr != m_procList.end(); cpyItr++ )
      {
        CHECK( ( *cpyItr )->isInitDone, "set flush signal on a picture for which the coding process has already started" );
        ( *cpyItr )->isFlush = true;
      }
    }

    m_procList.insert( picItr, pic );
    m_picCount++;
  }

  void runStage( bool flush, AccessUnitList& auList )
  {
    // check for first flush signal
    if( flush && ! m_flush )
    {
      if( ! m_isNonBlocking )
      {
        // assume blocking stages are at the front and behave deterministically with respect to the input YUV pictures
        // therefore mark all pictures in blocking stages as "have seen flush signal"
        for( auto pic : m_procList )
          pic->isFlush = true;
      }
      else
      {
        // for the non-blocking stage, mark at least the last picture in coding order as "have seen flush signal"
        if( ! m_procList.empty() )
          m_procList.back()->isFlush = true;
      }
      m_flush = true;
    }

    // ready to go?
    if( ( (int)m_procList.size() >= m_minQueueSize )
        || ( m_procList.size() && flush ) )
    {
      // process always one picture or all if encoder should be flushed
      do
      {
        // process pictures
        PicList doneList;
        PicList freeList;
        processPictures( m_procList, auList, doneList, freeList );

        // send processed/finalized pictures to next stage
        for( auto pic : doneList )
        {
          // release previous pictures original buffers
          // will not be needed by this picture and this stage anymore
          // helps reducing overall memory footprint
          PicShared* picShared = pic->m_picShared;
          picShared->releasePrevBuffers( pic );
          if( m_nextStage )
          {
            m_nextStage->addPicSorted( picShared, flush );
          }
        }

        // release unused pictures
        for( auto pic : freeList )
        {
          // release shared buffer
          PicShared* picShared = pic->m_picShared;
          picShared->releaseShared( pic );
          // remove pic from own processing queue
          m_procList.remove( pic );
          m_freeList.push_back( pic );
        }
      } while( m_flushAll && flush && m_procList.size() );
    }
  }
};

} // namespace vvenc

//! \}


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

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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

/** \file     EncStage.h
    \brief
*/

#pragma once

#include "CommonLib/CommonDef.h"
#include "CommonLib/Picture.h"
#include "CommonLib/Nal.h"

//! \ingroup EncoderLib
//! \{

namespace vvenc {

class PicShared
{
public:
  PicShared()
  : m_isSccWeak  ( false )
  , m_isSccStrong( false )
  , m_cts        ( 0 )
  , m_maxFrames  ( -1 )
  , m_poc        ( -1 )
  , m_refCount   ( -1 )
  , m_isLead     ( false )
  , m_isTrail    ( false )
  , m_ctsValid   ( false )
  {
    std::fill_n( m_prevShared, QPA_PREV_FRAMES, nullptr );
  };

  ~PicShared() {};

  bool         isUsed()          const { return m_refCount > 0; }
  bool         isLeadTrail()     const { return m_isLead || m_isTrail; }
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

    copyPadToPelUnitBuf( m_origBuf, *yuvInBuf, getChromaFormat() );

    m_isSccWeak   = false;
    m_isSccStrong = false;
    m_cts         = yuvInBuf->cts;
    m_poc         = poc;
    m_refCount    = 0;
    m_isLead      = poc < 0;
    m_isTrail     = m_maxFrames > 0 && poc >= m_maxFrames;
    m_ctsValid    = yuvInBuf->ctsValid;
    std::fill_n( m_prevShared, QPA_PREV_FRAMES, nullptr );
  }

  void shareData( Picture* pic )
  {
    PelStorage* prevOrigBufs[ QPA_PREV_FRAMES ];
    for( int i = 0; i < QPA_PREV_FRAMES; i++ )
    {
      prevOrigBufs[ i ] = sharePrevOrigBuffer( i );
    }
    pic->linkSharedBuffers( &m_origBuf, &m_filteredBuf, prevOrigBufs, this );
    pic->isSccWeak   = m_isSccWeak;
    pic->isSccStrong = m_isSccStrong;
    pic->poc         = m_poc;
    pic->cts         = m_cts;
    pic->ctsValid    = m_ctsValid;
    m_refCount      += 1;
  }

  void releaseShared( Picture* pic )
  {
    pic->releaseSharedBuffers();
    for( int i = 0; i < QPA_PREV_FRAMES; i++ )
    {
      releasePrevOrigBuffer( i );
    }
    m_refCount -= 1;
    CHECK( m_refCount < 0, "PicShared invalid state" );
  };

private:
  PelStorage* sharePrevOrigBuffer( int idx )
  {
    CHECK( idx >= QPA_PREV_FRAMES, "array access out of bounds" );
    if( m_prevShared[ idx ] )
    {
      m_prevShared[ idx ]->m_refCount += 1;
      return &( m_prevShared[ idx ]->m_origBuf );
    }
    return nullptr;
  }

  void releasePrevOrigBuffer( int idx )
  {
    CHECK( idx >= QPA_PREV_FRAMES, "array access out of bounds" );
    if( m_prevShared[ idx ] )
    {
      m_prevShared[ idx ]->m_refCount -= 1;
      CHECK( m_refCount < 0, "PicShared invalid state" );
    }
  }

public:
  PicShared* m_prevShared[ QPA_PREV_FRAMES ];
  bool       m_isSccWeak;
  bool       m_isSccStrong;

private:
  PelStorage m_origBuf;
  PelStorage m_filteredBuf;
  uint64_t   m_cts;
  int        m_maxFrames;
  int        m_poc;
  int        m_refCount;
  bool       m_isLead;
  bool       m_isTrail;
  bool       m_ctsValid;
};

// ====================================================================================================================
// Class definition
// ====================================================================================================================

class EncStage
{
public:
  EncStage()
  : m_picCount        ( 0 )
  , m_nextStage       ( nullptr )
  , m_minQueueSize    ( 0 )
  , m_flushAll        ( false )
  , m_processLeadTrail( false )
  , m_ctuSize         ( MAX_CU_SIZE )
  , m_isNonBlocking   ( false )
  {
  };

  virtual ~EncStage()
  {
    freePicList();
  };

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

  bool isStageDone() const { return m_procList.empty(); }

  void initStage( const VVEncCfg& encCfg, int minQueueSize, bool flushAll, bool processLeadTrail, int ctuSize, bool nonBlocking = false )
  {
    m_minQueueSize     = minQueueSize;
    m_flushAll         = flushAll;
    m_processLeadTrail = processLeadTrail;
    m_ctuSize          = ctuSize;
    m_isNonBlocking    = nonBlocking;
    m_pcEncCfg         = &encCfg;
  }

  void linkNextStage( EncStage* nextStage )
  {
    m_nextStage = nextStage;
  }

  void addPic( PicShared* picShared )
  {
    // send lead trail data to next stage if not requested
    if( ! m_processLeadTrail && picShared->isLeadTrail() )
    {
      if( m_nextStage )
      {
        m_nextStage->addPic( picShared );
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
    for( picItr = m_procList.begin(); picItr != m_procList.end(); picItr++ )
    {
      if( pic->poc < ( *picItr )->poc )
        break;
    }
    m_procList.insert( picItr, pic );
    m_picCount++;
  }

  void runStage( bool flush, AccessUnitList& auList )
  {
    {
      // process always one picture or all if encoder should be flushed
      do
      {
        // process pictures
        PicList doneList;
        PicList freeList;
        processPictures( m_procList, flush, auList, doneList, freeList );

        // send processed/finalized pictures to next stage
        if( m_nextStage )
        {
          for( auto pic : doneList )
          {
            m_nextStage->addPic( pic->m_picShared );
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

  bool         isNonBlocking()     { return m_isNonBlocking; }
  virtual int  picOutputDelay()    { return m_minQueueSize; }
  virtual bool canRunStage( bool flush, bool picSharedAvail ) { return canRunStage( flush ); }
  virtual bool canRunStage( bool flush )
  {
    return ( ( (int)m_procList.size() >= m_minQueueSize ) || ( m_procList.size() && flush ) );
  }
  virtual void checkState()  {}
protected:
  virtual void initPicture    ( Picture* pic ) = 0;
  virtual void processPictures( const PicList& picList, bool flush, AccessUnitList& auList, PicList& doneList, PicList& freeList ) = 0;
  int64_t               m_picCount;
private:
  const VVEncCfg* m_pcEncCfg;
  EncStage* m_nextStage;
  PicList   m_procList;
  PicList   m_freeList;
  int       m_minQueueSize;
  bool      m_flushAll;
  bool      m_processLeadTrail;
  int       m_ctuSize;
  bool      m_isNonBlocking;
};

} // namespace vvenc

//! \}


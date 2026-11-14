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
  PicVisAct        m_picVA;
  bool             m_isSccWeak;
  bool             m_isSccStrong;
  bool             m_forceSCC;
  int              m_picMemorySTA;
  uint16_t         m_picMotEstError;
  uint8_t          m_minNoiseLevels[QPA_MAX_NOISE_LEVELS];
  std::vector<int> m_ctuBimQpOffset;
  int              m_picAuxQpOffset; // auxiliary QP offset per frame, for combination of RC and BIM (and possibly other tools)

private:
  PelStorage       m_origBuf;
  PelStorage       m_filteredBuf;
  PelStorage       m_fgaFilteredBuf;   // temporally filtered picture for the film grain analysis, until analysed
  bool             m_fgaUsesFilteredBuf = false;   // the encoding filtered picture is the analysis picture
  PelStorage       m_fgaResidualBuf;   // temporal residual for the improved film grain analysis, until analysed
  int              m_fgaResidualWeight = 0;        // its variance relative to the grain variance, 0 without residual
  bool             m_fgaSceneCut = false;          // the picture starts a new scene (film grain analysis)
  int              m_fgaLastSceneCutPoc = std::numeric_limits<int>::min();   // the latest scene cut before it
  uint64_t         m_cts;
  int              m_maxFrames;
  int              m_poc;
  int              m_refCount;
  bool             m_isLead;
  bool             m_isTrail;
  bool             m_ctsValid;
  void*            m_userData;
  // The source buffer's layout, kept apart from m_origBuf, which a picture gives back (lends) once it is reconstructed.
  ChromaFormat     m_chromaFormat = NUM_CHROMA_FORMAT;
  Size             m_lumaSize;
  int              m_padding = 0;
  bool             m_sourceDone = false;   // the final encoder no longer reads m_origBuf (see canLendSource); set and cleared on the main thread only

public:
  PicShared()
  : m_picVA          ()
  , m_isSccWeak      ( false )
  , m_isSccStrong    ( false )
  , m_forceSCC       ( false )
  , m_picMemorySTA   ( 0 )
  , m_picMotEstError ( 0 )
  , m_picAuxQpOffset ( 0 )
  , m_cts            ( 0 )
  , m_maxFrames      ( -1 )
  , m_poc            ( -1 )
  , m_refCount       ( -1 )
  , m_isLead         ( false )
  , m_isTrail        ( false )
  , m_ctsValid       ( false )
  , m_userData       ( nullptr )
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
  ChromaFormat getChromaFormat() const { return m_chromaFormat; }
  PelStorage&  getFgaFilteredBuf()     { return m_fgaFilteredBuf; }
  bool&        fgaUsesFilteredBuf()    { return m_fgaUsesFilteredBuf; }
  PelStorage&  getFgaResidualBuf()     { return m_fgaResidualBuf; }
  int&         fgaResidualWeight()     { return m_fgaResidualWeight; }
  bool&        fgaSceneCut()           { return m_fgaSceneCut; }
  int&         fgaLastSceneCutPoc()    { return m_fgaLastSceneCutPoc; }
  Size         getLumaSize()     const { return m_lumaSize; }

  // withSource false: the source buffer comes later, from createSource() or takeSourceFrom()
  void create( int maxFrames, ChromaFormat chromaFormat, const Size& size, bool useFilter, bool withSource )
  {
    CHECK( m_refCount >= 0, "PicShared already created" );

    m_maxFrames    = maxFrames;
    m_refCount     = 0;
    m_chromaFormat = chromaFormat;
    m_lumaSize     = size;
    m_padding      = useFilter ? MCTF_FRAME_MARGIN : 0;
    if( withSource )
    {
      createSource();
    }
  }

  bool hasSource() const { return m_origBuf.getOrigin( 0 ) != nullptr; }
  void createSource()    { m_origBuf.create( m_chromaFormat, Area( Position(), m_lumaSize ), 0, m_padding ); }

  // Set by the final GOP encoder once it has reconstructed the picture and nothing of its own reads the source any
  // more (PSNR is computed before reconstruction ends, output reads the reconstruction only); cleared when it releases
  // its picture. So while it is set, the final encoder's picture is one of the holders.
  void setSourceDone()      { m_sourceDone = true; }
  void clearSourceDone()    { m_sourceDone = false; }
  bool isSourceDone() const { return m_sourceDone; }
  // The source may go to a new input picture when nothing but the final encoder's picture holds this picture: no
  // earlier stage (MCTF neighbours, pre-processing) and no later picture's QPA previous-frame reference. Once the final
  // encoder has released its picture, m_refCount == 1 would be such a reference, hence the flag is cleared then.
  bool canLendSource() const { return m_sourceDone && m_refCount == 1 && hasSource(); }
  // Moves the lender's source buffer here. The lender keeps an empty PelStorage; a read through it is a bug (its
  // cleared buffer list may still point at the moved storage, now holding another picture).
  void takeSourceFrom( PicShared& lender )
  {
    CHECK( hasSource() || ! lender.canLendSource(), "invalid source buffer hand-over" );
    m_origBuf.takeOwnership( lender.m_origBuf );
  }

  void reuse( int poc, const vvencYUVBuffer* yuvInBuf )
  {
    CHECK( m_refCount < 0, "PicShared not created" );
    CHECK( isUsed(),       "PicShared still in use" );
    CHECK( ! hasSource(),  "PicShared has no source buffer" );

    m_sourceDone = false;
    copyPadToPelUnitBuf( m_origBuf, *yuvInBuf, getChromaFormat() );
    m_fgaFilteredBuf.destroy();
    m_fgaUsesFilteredBuf = false;
    m_fgaResidualBuf.destroy();
    m_fgaResidualWeight = 0;
    m_fgaSceneCut        = false;
    m_fgaLastSceneCutPoc = std::numeric_limits<int>::min();

    m_picVA.reset();
    m_isSccWeak    = false;
    m_isSccStrong  = false;
    m_forceSCC     = false;
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
#if VVENC_USE_UNSTABLE_API
    m_userData       = yuvInBuf->userData;
#endif
  }

  void shareData( Picture* pic )
  {
    PelStorage* prevOrigBufs[ NUM_QPA_PREV_FRAMES ];
    sharePrevBuffers( prevOrigBufs );
    pic->linkSharedBuffers( &m_origBuf, &m_filteredBuf, prevOrigBufs, this );
    pic->picVA          = m_picVA;
    pic->isSccWeak      = m_isSccWeak;
    pic->isSccStrong    = m_isSccStrong;
    pic->picMemorySTA   = m_picMemorySTA;
    pic->poc            = m_poc;
    pic->cts            = m_cts;
    pic->ctsValid       = m_ctsValid;
    pic->gopEntry       = &m_gopEntry;
    pic->userData       = m_userData;
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
        CHECK( ! m_prevShared[ i ]->hasSource(), "previous picture's source buffer was lent" );
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
  EncStage* m_startStage       { nullptr };

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

  // Called by a stage before it waits for its own pool tasks (see setStartStage).
  void pollStartStage() const { if( m_startStage ) m_startStage->startReadyPictures(); }

public:
  // Waits until a picture in coding is finished; returns false at once when no picture is in coding.
  virtual bool waitForFreeEncoders() { return false; }
  virtual void startReadyPictures() {}

  // Pictures start only on the main thread, in the last stage. While the main thread runs an earlier stage, that stage
  // lets the last one start ready pictures (startReadyPictures, without waiting) each time it waits for its own pool
  // tasks, so a picture encoder freed meanwhile does not stay idle.
  void setStartStage( EncStage* startStage ) { m_startStage = startStage; }

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
  // pictures of this stage that are encoded but still hold their source buffer (read on the main thread only)
  int  numEncodedWithSource() const
  {
    int n = 0;
    for( auto pic : m_procList )
    {
      n += ( pic->isEncoded && pic->m_picShared && pic->m_picShared->hasSource() ) ? 1 : 0;
    }
    return n;
  }
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

  // Returns whether a picture was passed on to the next stage or released.
  bool runStage( bool flush, AccessUnitList& auList )
  {
    bool progress = false;

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
        progress |= ! doneList.empty() || ! freeList.empty();

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
    return progress;
  }
};

} // namespace vvenc

//! \}


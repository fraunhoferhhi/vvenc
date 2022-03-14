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
/** \file     Picture.h
 *  \brief    Description of a coded picture
 */

#pragma once

#include "CommonDef.h"
#include "Common.h"
#include "Unit.h"
#include "Slice.h"
#include "CodingStructure.h"
#include "BitStream.h"
#include "Reshape.h"

#include <deque>
#include <chrono>
#include <atomic>

//! \ingroup CommonLib
//! \{

namespace vvenc {

class SEI;
class SEIDecodedPictureHash;
class EncRCPic;
class PicShared;
class MsgLog;

typedef std::list<SEI*> SEIMessages;


struct StopClock
{
  StopClock() : m_startTime(), m_timer() {}

  int  getTimerInSec() const { return std::chrono::duration_cast<std::chrono::seconds>( m_timer ).count(); };
  void resetTimer()          { m_timer = std::chrono::steady_clock::duration::zero(); }
  void startTimer()          { m_startTime  = std::chrono::steady_clock::now(); }
  void stopTimer()           { auto endTime = std::chrono::steady_clock::now(); m_timer += endTime - m_startTime; m_startTime = endTime; }

  std::chrono::steady_clock::time_point m_startTime;
  std::chrono::steady_clock::duration   m_timer;
};


struct Picture;

class BlkStat
{
public:
  BlkStat()
    : m_bResetAMaxBT( true )
  {
  }

  ~BlkStat()
  {
  }

  void storeBlkSize ( const Picture& pic );
  void updateMaxBT  ( const Slice& slice, const BlkStat& blkStat );
  void setSliceMaxBT( Slice& slice );

protected:
  uint32_t m_uiBlkSize[NUM_AMAXBT_LAYER];
  uint32_t m_uiNumBlk[NUM_AMAXBT_LAYER];
  uint32_t m_uiPrevISlicePOC;
  bool     m_bResetAMaxBT;
};

struct Picture : public UnitArea
{
  uint32_t margin;
  Picture();

  void create( ChromaFormat _chromaFormat, const Size& size, unsigned _maxCUSize, unsigned _margin, bool _decoder );
  void reset();
  void destroy( bool bPicHeader );

  void linkSharedBuffers( PelStorage* origBuf, PelStorage* filteredBuf, PelStorage* prevOrigBufs[ NUM_PREV_FRAMES ], PicShared* picShared );
  void releaseSharedBuffers();

  void createTempBuffers( unsigned _maxCUSize );
  void destroyTempBuffers();

  void extendPicBorder();
  void finalInit( const VPS& vps, const SPS& sps, const PPS& pps, PicHeader* picHeader, XUCache& unitCache, std::mutex* mutex, APS** alfAps, APS* lmcsAps );
  void setSccFlags( const VVEncCfg* encCfg );

  int  getPOC()                                                     const { return poc; }

  const PelStorage& getOrigBuffer()                                       { return *m_sharedBufs[       PIC_ORIGINAL]; }
         PelUnitBuf getOrigBuf()                                          { return getSharedBuf(        PIC_ORIGINAL); }
  const CPelUnitBuf getOrigBuf()                                    const { return getSharedBuf(        PIC_ORIGINAL); }
  const CPelBuf     getOrigBuf(const ComponentID compID)            const { return getSharedBuf(compID, PIC_ORIGINAL); }
  const CPelBuf     getOrigBuf(const CompArea& blk)                 const { return getSharedBuf(blk,    PIC_ORIGINAL); }
  const CPelUnitBuf getOrigBuf(const UnitArea& unit)                const { return getSharedBuf(unit,   PIC_ORIGINAL); }

         PelBuf     getRecoBuf(const ComponentID compID)                  { return getPicBuf(compID, PIC_RECONSTRUCTION); }
  const CPelBuf     getRecoBuf(const ComponentID compID)            const { return getPicBuf(compID, PIC_RECONSTRUCTION); }
         PelBuf     getRecoBuf(const CompArea& blk)                       { return getPicBuf(blk,    PIC_RECONSTRUCTION); }
  const CPelBuf     getRecoBuf(const CompArea& blk)                 const { return getPicBuf(blk,    PIC_RECONSTRUCTION); }
         PelUnitBuf getRecoBuf(const UnitArea& unit)                      { return getPicBuf(unit,   PIC_RECONSTRUCTION); }
  const CPelUnitBuf getRecoBuf(const UnitArea& unit)                const { return getPicBuf(unit,   PIC_RECONSTRUCTION); }
         PelUnitBuf getRecoBuf()                                          { return getPicBuf(        PIC_RECONSTRUCTION); }
  const CPelUnitBuf getRecoBuf()                                    const { return getPicBuf(        PIC_RECONSTRUCTION); }

  const CPelBuf     getRecoWrapBuf(const ComponentID compID)        const { return getPicBuf(compID, PIC_RECON_WRAP); }
  const CPelBuf     getRecoWrapBuf(const CompArea& blk)             const { return getPicBuf(blk,    PIC_RECON_WRAP); }
  const CPelUnitBuf getRecoWrapBuf(const UnitArea& unit)            const { return getPicBuf(unit,   PIC_RECON_WRAP); }
         PelUnitBuf getRecoWrapBuf()                                      { return getPicBuf(        PIC_RECON_WRAP); }
  const CPelUnitBuf getRecoWrapBuf()                                const { return getPicBuf(        PIC_RECON_WRAP); }

         PelUnitBuf getSaoBuf()                                           { return getPicBuf(        PIC_SAO_TEMP); }
  const CPelUnitBuf getSaoBuf()                                     const { return getPicBuf(        PIC_SAO_TEMP); }

        PelStorage& getFilteredOrigBuffer()                               { return *m_sharedBufs[       PIC_ORIGINAL_RSP]; }
         PelUnitBuf getRspOrigBuf()                                       { return getSharedBuf(        PIC_ORIGINAL_RSP); }
  const CPelUnitBuf getRspOrigBuf()                                 const { return getSharedBuf(        PIC_ORIGINAL_RSP); }
  const CPelBuf     getRspOrigBuf(const ComponentID compID)         const { return getSharedBuf(compID, PIC_ORIGINAL_RSP); }
  const CPelBuf     getRspOrigBuf(const CompArea& blk)              const { return getSharedBuf(blk,    PIC_ORIGINAL_RSP); }
  const CPelUnitBuf getRspOrigBuf(const UnitArea& unit)             const { return getSharedBuf(unit,   PIC_ORIGINAL_RSP); }

  const CPelBuf     getOrigBufPrev(const CompArea &blk, const PrevFrameType type) const;
  const CPelUnitBuf getOrigBufPrev(const PrevFrameType type) const;
  const CPelBuf     getOrigBufPrev(const ComponentID compID, const PrevFrameType type) const;

private:
         PelUnitBuf getPicBuf(                          const PictureType type)         { return m_picBufs[ type ]; }
  const CPelUnitBuf getPicBuf(                          const PictureType type)   const { return m_picBufs[ type ]; }
         PelBuf     getPicBuf(const ComponentID compID, const PictureType type)         { return m_picBufs[ type ].getBuf( compID ); }
  const CPelBuf     getPicBuf(const ComponentID compID, const PictureType type)   const { return m_picBufs[ type ].getBuf( compID ); }
         PelBuf     getPicBuf(const CompArea& blk,      const PictureType type)         { return ( !blk.valid() ) ? PelBuf() : m_picBufs[ type ].getBuf( blk ); }
  const CPelBuf     getPicBuf(const CompArea& blk,      const PictureType type)   const { return ( !blk.valid() ) ? PelBuf() : m_picBufs[ type ].getBuf( blk ); }
         PelUnitBuf getPicBuf(const UnitArea& unit,     const PictureType type);
  const CPelUnitBuf getPicBuf(const UnitArea& unit,     const PictureType type) const;

         PelUnitBuf getSharedBuf(                          const PictureType type)         { return *m_sharedBufs[ type ]; }
  const CPelUnitBuf getSharedBuf(                          const PictureType type)   const { return *m_sharedBufs[ type ]; }
         PelBuf     getSharedBuf(const ComponentID compID, const PictureType type)         { return  m_sharedBufs[ type ]->getBuf( compID ); }
  const CPelBuf     getSharedBuf(const ComponentID compID, const PictureType type)   const { return  m_sharedBufs[ type ]->getBuf( compID ); }
         PelBuf     getSharedBuf(const CompArea& blk,      const PictureType type)         { return ( !blk.valid() ) ? PelBuf() : m_sharedBufs[ type ]->getBuf( blk ); }
  const CPelBuf     getSharedBuf(const CompArea& blk,      const PictureType type)   const { return ( !blk.valid() ) ? PelBuf() : m_sharedBufs[ type ]->getBuf( blk ); }
         PelUnitBuf getSharedBuf(const UnitArea& unit,     const PictureType type);
  const CPelUnitBuf getSharedBuf(const UnitArea& unit,     const PictureType type) const;

public:
  CodingStructure*              cs;
  const VPS*                    vps;
  const DCI*                    dci;
  ParameterSetMap<APS>          picApsMap;
  std::deque<Slice*>            slices;
  ReshapeData                   reshapeData;
  SEIMessages                   SEIs;
  BlkStat                       picBlkStat;
  std::vector<OutputBitstream>  sliceDataStreams;

  bool                          isInitDone;
  std::atomic_bool              isReconstructed;
  bool                          isBorderExtended;
  bool                          isReferenced;
  bool                          isNeededForOutput;
  bool                          isFinished;
  bool                          isLongTerm;
  bool                          encPic;
  bool                          writePic;
  bool                          precedingDRAP; // preceding a DRAP picture in decoding order

  int                           refCounter;
  int                           poc;
  int                           gopId;
  int                           rcIdxInGop;
  unsigned                      TLayer;
  int                           layerId;
  bool                          isSubPicBorderSaved;
  int                           sliceDataNumBins;
  uint64_t                      cts;
  bool                          ctsValid;
  bool                          isPreAnalysis;

  PicShared*                    m_picShared;

  PelStorage                    m_picBufs[ NUM_PIC_TYPES ];
  PelStorage*                   m_sharedBufs[ NUM_PIC_TYPES ];
  PelStorage*                   m_bufsOrigPrev[ NUM_PREV_FRAMES ];

  std::vector<double>           ctuQpaLambda;
  std::vector<Pel>              ctuAdaptedQP;
  std::mutex                    wppMutex;
  int                           picInitialQP;
  uint16_t                      picVisActTL0;
  uint16_t                      picVisActY;

  StopClock                     encTime;
  bool                          isSccWeak;
  bool                          isSccStrong;
  bool                          useScME;
  bool                          useScMCTF;
  bool                          useScTS;
  bool                          useScBDPCM;
  bool                          useScIBC;
  bool                          useScLMCS;
  int                           useQtbttSpeedUpMode;
  int                           seqBaseQp;
  int                           actualHeadBits;
  int                           actualTotalBits;
  EncRCPic*                     encRCPic;

  std::vector<SAOBlkParam>      m_sao[ 2 ];
  std::vector<uint8_t>          m_alfCtuEnabled[ MAX_NUM_COMP ];
  std::vector<short>            m_alfCtbFilterIndex;
  std::vector<uint8_t>          m_alfCtuAlternative[ MAX_NUM_COMP ];

public:
  Slice*          allocateNewSlice();
  Slice*          swapSliceObject( Slice* p, uint32_t i );

  SAOBlkParam    *getSAO    (int id = 0)                     { return &m_sao[id][0]; };
  void            resizeSAO (unsigned numEntries, int dstid) { m_sao[dstid].resize(numEntries); }
  void            copySAO   (const Picture& src, int dstid)  { std::copy(src.m_sao[0].begin(), src.m_sao[0].end(), m_sao[dstid].begin()); }

  void            resizeAlfCtuBuffers( int numEntries );
};

int calcAndPrintHashStatus(const CPelUnitBuf& pic, const SEIDecodedPictureHash* pictureHashSEI, const BitDepths &bitDepths, const vvencMsgLevel msgl, MsgLog& logger );

typedef std::list<Picture*> PicList;

} // namespace vvenc

//! \}


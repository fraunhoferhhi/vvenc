/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVenc

(c) Copyright (2019-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVenc (“Fraunhofer Versatile Video Encoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Encoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Encoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Encoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Encoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Encoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Encoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Encoding Library. If You use the Fraunhofer Versatile Video Encoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Encoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Encoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Encoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Encoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
----------------------------------------------------------------------------- */
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

//! \ingroup CommonLib
//! \{

namespace vvenc {

class SEI;
class SEIDecodedPictureHash;

typedef std::list<SEI*> SEIMessages;

class Brick
{
private:
  uint32_t      m_widthInCtus;
  uint32_t      m_heightInCtus;
  uint32_t      m_colBd;
  uint32_t      m_rowBd;
  uint32_t      m_firstCtuRsAddr;

public:
  Brick();
  virtual ~Brick();

  void      setWidthInCtus         ( uint32_t i )            { m_widthInCtus = i; }
  uint32_t  getWidthInCtus         () const                  { return m_widthInCtus; }
  void      setHeightInCtus        ( uint32_t i )            { m_heightInCtus = i; }
  uint32_t  getHeightInCtus        () const                  { return m_heightInCtus; }
  void      setColBd  ( uint32_t i )                         { m_colBd = i; }
  uint32_t  getColBd  () const                               { return m_colBd; }
  void      setRowBd ( uint32_t i )                          { m_rowBd = i; }
  uint32_t  getRowBd () const                                { return m_rowBd; }

  void      setFirstCtuRsAddr      ( uint32_t i )            { m_firstCtuRsAddr = i; }
  uint32_t  getFirstCtuRsAddr      () const                  { return m_firstCtuRsAddr; }
};


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

  void create( ChromaFormat _chromaFormat, const Size& size, unsigned _maxCUSize, unsigned _margin, bool _decoder, int _padding );
  void destroy();

  void createTempBuffers( unsigned _maxCUSize );
  void destroyTempBuffers();

  void extendPicBorder();
  void finalInit( const VPS& vps, const SPS& sps, const PPS& pps, PicHeader& picHeader, XUCache& unitCache, std::mutex* mutex, APS** alfAps, APS* lmcsAps );

  int  getPOC()                               const { return poc; }
  Pel* getOrigin( const PictureType& type, const ComponentID compID ) const;

         PelUnitBuf getOrigBuf()                                          { return getBuf(        PIC_ORIGINAL); }
  const CPelUnitBuf getOrigBuf()                                    const { return getBuf(        PIC_ORIGINAL); }
  const CPelBuf     getOrigBuf(const ComponentID compID)            const { return getBuf(compID, PIC_ORIGINAL); }
  const CPelBuf     getOrigBuf(const CompArea& blk)                 const { return getBuf(blk,    PIC_ORIGINAL); }
  const CPelUnitBuf getOrigBuf(const UnitArea& unit)                const { return getBuf(unit,   PIC_ORIGINAL); }
 
         PelBuf     getRecoBuf(const ComponentID compID)                  { return getBuf(compID, PIC_RECONSTRUCTION); }
  const CPelBuf     getRecoBuf(const ComponentID compID)            const { return getBuf(compID, PIC_RECONSTRUCTION); }
         PelBuf     getRecoBuf(const CompArea& blk)                       { return getBuf(blk,    PIC_RECONSTRUCTION); }
  const CPelBuf     getRecoBuf(const CompArea& blk)                 const { return getBuf(blk,    PIC_RECONSTRUCTION); }
         PelUnitBuf getRecoBuf(const UnitArea& unit)                      { return getBuf(unit,   PIC_RECONSTRUCTION); }
  const CPelUnitBuf getRecoBuf(const UnitArea& unit)                const { return getBuf(unit,   PIC_RECONSTRUCTION); }
         PelUnitBuf getRecoBuf()                                          { return getBuf(        PIC_RECONSTRUCTION); }
  const CPelUnitBuf getRecoBuf()                                    const { return getBuf(        PIC_RECONSTRUCTION); }

  const CPelBuf     getRecoWrapBuf(const ComponentID compID)        const { return getBuf(compID, PIC_RECON_WRAP); }
  const CPelBuf     getRecoWrapBuf(const CompArea& blk)             const { return getBuf(blk,    PIC_RECON_WRAP); }
  const CPelUnitBuf getRecoWrapBuf(const UnitArea& unit)            const { return getBuf(unit,   PIC_RECON_WRAP); }
         PelUnitBuf getRecoWrapBuf()                                      { return getBuf(        PIC_RECON_WRAP); }
  const CPelUnitBuf getRecoWrapBuf()                                const { return getBuf(        PIC_RECON_WRAP); }
  
         PelUnitBuf getSaoBuf()                                           { return getBuf(        PIC_SAO_TEMP ); }
  const CPelUnitBuf getSaoBuf()                                     const { return getBuf(        PIC_SAO_TEMP ); }
        PelStorage& getFilteredOrigBuffer()                               { return m_bufs[        PIC_ORIGINAL_RSP]; }
  
         PelUnitBuf getRspOrigBuf()                                       { return getBuf(        PIC_ORIGINAL_RSP); }
  const CPelUnitBuf getRspOrigBuf()                                 const { return getBuf(        PIC_ORIGINAL_RSP); }
  const CPelBuf     getRspOrigBuf(const ComponentID compID)         const { return getBuf(compID, PIC_ORIGINAL_RSP); }
  const CPelBuf     getRspOrigBuf(const CompArea& blk)              const { return getBuf(blk,    PIC_ORIGINAL_RSP); }
  const CPelUnitBuf getRspOrigBuf(const UnitArea& unit)             const { return getBuf(unit,   PIC_ORIGINAL_RSP); }

  const CPelBuf     getOrigBufPrev(const CompArea &blk, const bool minus2) const;
  const CPelUnitBuf getOrigBufPrev(const bool minus2) const;
  const CPelBuf     getOrigBufPrev(const ComponentID compID, const bool minus2) const;

private: 
         PelUnitBuf getBuf(                          const PictureType type)         { return m_bufs[ type ]; }
  const CPelUnitBuf getBuf(                          const PictureType type)   const { return m_bufs[ type ]; }
         PelBuf     getBuf(const ComponentID compID, const PictureType type)         { return m_bufs[ type ].getBuf( compID ); }
  const CPelBuf     getBuf(const ComponentID compID, const PictureType type)   const { return m_bufs[ type ].getBuf( compID ); }
         PelBuf     getBuf(const CompArea& blk,      const PictureType type)         { return ( !blk.valid() ) ? PelBuf() : m_bufs[ type ].getBuf( blk ); }
  const CPelBuf     getBuf(const CompArea& blk,      const PictureType type)   const { return ( !blk.valid() ) ? PelBuf() : m_bufs[ type ].getBuf( blk ); }
         PelUnitBuf getBuf(const UnitArea& unit,     const PictureType type);
  const CPelUnitBuf getBuf(const UnitArea& unit,     const PictureType type) const;

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

  bool                          isMctfFiltered;
  bool                          isInitDone;
  bool                          isReconstructed;
  bool                          isBorderExtended;
  bool                          isReferenced;
  bool                          isNeededForOutput;
  bool                          isLongTerm;
  bool                          encPic;
  bool                          writePic;
  bool                          precedingDRAP; // preceding a DRAP picture in decoding order

  int                           refCounter;
  int                           poc;
  int                           gopId;
  unsigned                      TLayer;
  int                           layerId;
  bool                          isSubPicBorderSaved;
  int                           sliceDataNumBins;
  uint64_t                      cts;
  bool                          ctsValid;

  PelStorage                    m_bufs[ NUM_PIC_TYPES ];
  PelStorage*                   m_bufsOrigPrev[2];

  std::vector<double>           ctuQpaLambda;
  std::vector<Pel>              ctuAdaptedQP;
  std::mutex                    wppMutex;
  int                           picInitialQP;
  StopClock                     encTime;
#if DETECT_SC
  bool                          useSC;
#endif

private:
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

  uint8_t* getAlfCtuEnabled( int compIdx ) { return m_alfCtuEnabled[compIdx].data(); }
  std::vector<uint8_t>* getAlfCtuEnabled() { return m_alfCtuEnabled; }
  void resizeAlfCtuEnabled( int numEntries )
  {
    for( int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++ )
    {
      m_alfCtuEnabled[compIdx].resize( numEntries );
      std::fill( m_alfCtuEnabled[compIdx].begin(), m_alfCtuEnabled[compIdx].end(), 0 );
    }
  }
  short* getAlfCtbFilterIndex() { return m_alfCtbFilterIndex.data(); }
  std::vector<short>& getAlfCtbFilterIndexVec() { return m_alfCtbFilterIndex; }
  void resizeAlfCtbFilterIndex(int numEntries)
  {
    m_alfCtbFilterIndex.resize(numEntries);
    for (int i = 0; i < numEntries; i++)
    {
      m_alfCtbFilterIndex[i] = 0;
    }
  }
  std::vector<uint8_t>& getAlfCtuAlternative( int compIdx ) { return m_alfCtuAlternative[compIdx]; }
  uint8_t* getAlfCtuAlternativeData( int compIdx ) { return m_alfCtuAlternative[compIdx].data(); }
  void resizeAlfCtuAlternative( int numEntries )
  {
    for( int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++ )
    {
      m_alfCtuAlternative[compIdx].resize( numEntries );
      std::fill( m_alfCtuAlternative[compIdx].begin(), m_alfCtuAlternative[compIdx].end(), 0 );
    }
  }
};

int calcAndPrintHashStatus(const CPelUnitBuf& pic, const SEIDecodedPictureHash* pictureHashSEI, const BitDepths &bitDepths, const MsgLevel msgl);


typedef std::list<Picture*> PicList;

} // namespace vvenc

//! \}


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


/** \file     Picture.cpp
 *  \brief    Description of a coded picture
 */

#include "Picture.h"
#include "SEI.h"

#include <math.h>

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ---------------------------------------------------------------------------
// picture methods
// ---------------------------------------------------------------------------




// ====================================================================================================================
// AMaxBT block statistics
// ====================================================================================================================


void BlkStat::storeBlkSize( const Picture& pic )
{
  const Slice& slice = *(pic.slices[ 0 ]);

  ::memset( m_uiBlkSize, 0, sizeof( m_uiBlkSize ) );
  ::memset( m_uiNumBlk,  0, sizeof( m_uiNumBlk ) );

  if ( ! slice.isIRAP() )
  {
    const int refLayer = slice.depth < NUM_AMAXBT_LAYER ? slice.depth: NUM_AMAXBT_LAYER - 1;
    for ( const CodingUnit *cu : pic.cs->cus )
    {
      m_uiBlkSize[ refLayer ] += cu->Y().area();
      m_uiNumBlk [ refLayer ] += 1;
    }
  }
}

void BlkStat::updateMaxBT( const Slice& slice, const BlkStat& blkStat )
{
  if ( ! slice.isIRAP() )
  {
    const int refLayer = slice.depth < NUM_AMAXBT_LAYER ? slice.depth: NUM_AMAXBT_LAYER - 1;
    m_uiBlkSize[ refLayer ] += blkStat.m_uiBlkSize[ refLayer ];
    m_uiNumBlk [ refLayer ] += blkStat.m_uiNumBlk [ refLayer ];
  }
}

void BlkStat::setSliceMaxBT( Slice& slice )
{
  if( ! slice.isIRAP() )
  {
    int refLayer = slice.depth < NUM_AMAXBT_LAYER ? slice.depth: NUM_AMAXBT_LAYER - 1;
    if( m_bResetAMaxBT && slice.poc > m_uiPrevISlicePOC )
    {
      ::memset( m_uiBlkSize, 0, sizeof( m_uiBlkSize ) );
      ::memset( m_uiNumBlk,  0, sizeof( m_uiNumBlk ) );
      m_bResetAMaxBT = false;
    }

    if( refLayer >= 0 && m_uiNumBlk[ refLayer ] != 0 )
    {
      slice.picHeader->splitConsOverride = true;
      double dBlkSize = sqrt( ( double ) m_uiBlkSize[refLayer] / m_uiNumBlk[refLayer] );
      if( dBlkSize < AMAXBT_TH32 || slice.sps->CTUSize == 32 )
      {
        slice.picHeader->maxBTSize[1] = ( 32 > MAX_BT_SIZE_INTER ? MAX_BT_SIZE_INTER : 32 );
      }
      else if( dBlkSize < AMAXBT_TH64 || slice.sps->CTUSize == 64 )
      {
        slice.picHeader->maxBTSize[1] = ( 64 > MAX_BT_SIZE_INTER ? MAX_BT_SIZE_INTER : 64 );
      }
      else
      {
        slice.picHeader->maxBTSize[1] = ( 128 > MAX_BT_SIZE_INTER ? MAX_BT_SIZE_INTER : 128 );
      }

      m_uiBlkSize[ refLayer ] = 0;
      m_uiNumBlk [ refLayer ] = 0;
    }
  }
  else
  {
    if( m_bResetAMaxBT )
    {
      ::memset( m_uiBlkSize, 0, sizeof( m_uiBlkSize ) );
      ::memset( m_uiNumBlk,  0, sizeof( m_uiNumBlk ) );
    }

    m_uiPrevISlicePOC = slice.poc;
    m_bResetAMaxBT    = true;
  }
}


// ====================================================================================================================
// Picture
// ====================================================================================================================


Picture::Picture()
    : cs                ( nullptr )
    , vps               ( nullptr )
    , dci               ( nullptr )
    , picApsMap         ( MAX_NUM_APS * MAX_NUM_APS_TYPE )
    , isMctfProcessed   ( false )
    , isInitDone        ( false )
    , isReconstructed   ( false )
    , isBorderExtended  ( false )
    , isReferenced      ( false )
    , isNeededForOutput ( false )
    , isFinished        ( false )
    , isLongTerm        ( false )
    , encPic            ( true )
    , writePic          ( true )
    , precedingDRAP     ( false )
    , refCounter        ( 0 )
    , poc               ( 0 )
    , gopId             ( 0 )
    , rcIdxInGop        ( 0 )
    , TLayer            ( std::numeric_limits<uint32_t>::max() )
    , layerId           ( 0 )
    , isSubPicBorderSaved (false)
    , sliceDataNumBins  ( 0 )
    , cts               ( 0 )
    , ctsValid          ( false )
    , m_bufsOrigPrev    { nullptr, nullptr }
    , picInitialQP      ( 0 )
    , picVisActY        ( 0.0 )
    , useScME           ( false )
    , useScMCTF         ( false )
    , useScTS           ( false )
    , useScBDPCM        ( false )
    , useScIBC          ( false )
    , useScLMCS         ( false )
#if QTBTT_SPEED3
    , useQtbttSpeedUpMode( 0 )
#endif
    , seqBaseQp         ( 0 )
    , actualHeadBits    ( 0 )
    , actualTotalBits   ( 0 )
    , encRCPic          ( nullptr )
{
}

void Picture::create( ChromaFormat _chromaFormat, const Size& size, unsigned _maxCUSize, unsigned _margin, bool _decoder, int _padding )
{
  UnitArea::operator=( UnitArea( _chromaFormat, Area( Position{ 0, 0 }, size ) ) );
  margin            =  _margin;
  const Area a      = Area( Position(), size );
  m_bufs[ PIC_RECONSTRUCTION ].create( _chromaFormat, a, _maxCUSize, _margin, MEMORY_ALIGN_DEF_SIZE );

  if( _decoder )
  {
    m_bufs[ PIC_RESIDUAL ].create( _chromaFormat, Area( 0, 0, _maxCUSize, _maxCUSize ) );
    m_bufs[PIC_PREDICTION].create( _chromaFormat, Area( 0, 0, _maxCUSize, _maxCUSize ) );
  }
  else
  {
    m_bufs[ PIC_ORIGINAL ].create( _chromaFormat, a, 0, _padding );
  }
}

void Picture::destroy()
{
  for (uint32_t t = 0; t < NUM_PIC_TYPES; t++)
  {
    m_bufs[  t ].destroy();
  }
  if( cs )
  {
    cs->picHeader = nullptr;
    cs->destroy();
    delete cs;
    cs = nullptr;
  }

  for( auto &ps : slices )
  {
    delete ps;
  }
  slices.clear();

  for( auto &psei : SEIs )
  {
    delete psei;
  }
  SEIs.clear();

}

void Picture::createTempBuffers( unsigned _maxCUSize )
{
  CHECK( !cs, "Coding structure is required a this point!" );

  m_bufs[PIC_SAO_TEMP].create( chromaFormat, Y(), cs->pcv->maxCUSize, 0, MEMORY_ALIGN_DEF_SIZE );

  if( cs ) cs->rebindPicBufs();
}

void Picture::destroyTempBuffers()
{
  m_bufs[PIC_SAO_TEMP].destroy();

  if( cs ) cs->rebindPicBufs();
}

const CPelBuf     Picture::getOrigBufPrev (const CompArea &blk, const bool minus2) const { return (m_bufsOrigPrev[minus2 ? 1 : 0] && blk.valid() ? m_bufsOrigPrev[minus2 ? 1 : 0]->getBuf (blk) : PelBuf()); }
const CPelUnitBuf Picture::getOrigBufPrev (const bool minus2)   const { return (m_bufsOrigPrev[minus2 ? 1 : 0] ? *m_bufsOrigPrev[minus2 ? 1 : 0] : PelUnitBuf()); }
const CPelBuf     Picture::getOrigBufPrev (const ComponentID compID, const bool minus2) const { return (m_bufsOrigPrev[minus2 ? 1 : 0] ? m_bufsOrigPrev[minus2 ? 1 : 0]->getBuf (compID) : PelBuf()); }

void Picture::finalInit( const VPS& _vps, const SPS& sps, const PPS& pps, PicHeader* picHeader, XUCache& unitCache, std::mutex* mutex, APS** alfAps, APS* lmcsAps )
{
  for( auto &sei : SEIs )
  {
    delete sei;
  }
  SEIs.clear();

  for (size_t i = 0; i < slices.size(); i++)
  {
    delete slices[i];
  }
  slices.clear();

  const ChromaFormat chromaFormatIDC = sps.chromaFormatIdc;
  const int          iWidth = pps.picWidthInLumaSamples;
  const int          iHeight = pps.picHeightInLumaSamples;

  if( cs )
  {
    CHECK( cs->sps != &sps, "picture initialization error: sps changed" );
    CHECK( cs->vps != &_vps, "picture initialization error: vps changed" );
  }
  else
  {
    cs = new CodingStructure( unitCache, mutex );
    cs->sps = &sps;
    cs->vps = &_vps;
    cs->create( UnitArea( chromaFormatIDC, Area( 0, 0, iWidth, iHeight )), true, pps.pcv );
  }

  cs->picture   = this;
  cs->refCS     = cs;
  cs->slice     = nullptr;  // the slices for this picture have not been set at this point. update cs->slice after swapSliceObject()
  cs->pps       = &pps;
  cs->picHeader = picHeader;
  if ( alfAps )
  {
    memcpy(cs->alfAps, alfAps, sizeof(cs->alfAps));
  }
  cs->lmcsAps = lmcsAps;
  cs->pcv     = pps.pcv;
  vps         = &_vps;
  dci         = nullptr;

  sliceDataStreams.clear();
  sliceDataNumBins = 0;
}

Slice* Picture::allocateNewSlice()
{
  slices.push_back( new Slice );
  Slice& slice = *slices.back();

  slice.pic     = this;
  slice.pps     = cs->pps;
  slice.sps     = cs->sps;
  slice.vps     = cs->vps;

  memcpy( slice.alfAps, cs->alfAps, sizeof(cs->alfAps) );

  if ( slices.size() >= 2 )
  {
    slice.copySliceInfo( slices[ slices.size() - 2 ] );
  }

  return slices.back();
}

Slice* Picture::swapSliceObject( Slice*  p, uint32_t i )
{
  p->pic     = this;
  p->pps     = cs->pps;
  p->sps     = cs->sps;
  p->vps     = cs->vps;
  memcpy( p->alfAps, cs->alfAps, sizeof(cs->alfAps) );

  Slice*  pTmp = slices[ i ];
  slices[ i ] = p;

  pTmp->pic = ( nullptr );
  pTmp->sps = ( nullptr );
  pTmp->pps = ( nullptr );
  memset( pTmp->alfAps, 0, sizeof( *pTmp->alfAps ) * ALF_CTB_MAX_NUM_APS );

  return pTmp;
}

void Picture::extendPicBorder()
{
  if ( isBorderExtended )
  {
    return;
  }

  for(int comp=0; comp<getNumberValidComponents( cs->area.chromaFormat ); comp++)
  {
    ComponentID compID = ComponentID( comp );
    PelBuf p = m_bufs[ PIC_RECONSTRUCTION ].get( compID );
    Pel* piTxt = p.bufAt(0,0);
    int xmargin = margin >> getComponentScaleX( compID, cs->area.chromaFormat );
    int ymargin = margin >> getComponentScaleY( compID, cs->area.chromaFormat );

    Pel*  pi = piTxt;
    // do left and right margins
      for (int y = 0; y < p.height; y++)
      {
        for (int x = 0; x < xmargin; x++ )
        {
          pi[ -xmargin + x ] = pi[0];
          pi[  p.width + x ] = pi[p.width-1];
        }
        pi += p.stride;
      }

    // pi is now the (0,height) (bottom left of image within bigger picture
    pi -= (p.stride + xmargin);
    // pi is now the (-marginX, height-1)
    for (int y = 0; y < ymargin; y++ )
    {
      ::memcpy( pi + (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin << 1)));
    }

    // pi is still (-marginX, height-1)
    pi -= ((p.height-1) * p.stride);
    // pi is now (-marginX, 0)
    for (int y = 0; y < ymargin; y++ )
    {
      ::memcpy( pi - (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin<<1)) );
    }

    // reference picture with horizontal wrapped boundary
    if (cs->sps->wrapAroundEnabled)
    {
      p = m_bufs[ PIC_RECON_WRAP ].get( compID );
      p.copyFrom(m_bufs[ PIC_RECONSTRUCTION ].get( compID ));
      piTxt = p.bufAt(0,0);
      pi = piTxt;
      int xoffset = cs->pps->wrapAroundOffset >> getComponentScaleX( compID, cs->area.chromaFormat );
      for (int y = 0; y < p.height; y++)
      {
        for (int x = 0; x < xmargin; x++ )
        {
          if( x < xoffset )
          {
            pi[ -x - 1 ] = pi[ -x - 1 + xoffset ];
            pi[  p.width + x ] = pi[ p.width + x - xoffset ];
          }
          else
          {
            pi[ -x - 1 ] = pi[ 0 ];
            pi[  p.width + x ] = pi[ p.width - 1 ];
          }
        }
        pi += p.stride;
      }
      pi -= (p.stride + xmargin);
      for (int y = 0; y < ymargin; y++ )
      {
        ::memcpy( pi + (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin << 1)));
      }
      pi -= ((p.height-1) * p.stride);
      for (int y = 0; y < ymargin; y++ )
      {
        ::memcpy( pi - (y+1)*p.stride, pi, sizeof(Pel)*(p.width + (xmargin<<1)) );
      }
    }
  }

  isBorderExtended = true;
}

PelUnitBuf Picture::getBuf( const UnitArea& unit, const PictureType type )
{
  if( chromaFormat == CHROMA_400 )
  {
    return PelUnitBuf( chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return PelUnitBuf( chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

const CPelUnitBuf Picture::getBuf( const UnitArea& unit, const PictureType type ) const
{
  if( chromaFormat == CHROMA_400 )
  {
    return CPelUnitBuf( chromaFormat, getBuf( unit.Y(), type ) );
  }
  else
  {
    return CPelUnitBuf( chromaFormat, getBuf( unit.Y(), type ), getBuf( unit.Cb(), type ), getBuf( unit.Cr(), type ) );
  }
}

Pel* Picture::getOrigin( const PictureType& type, const ComponentID compID ) const
{
  return m_bufs[ type ].getOrigin( compID );

}

void Picture::resizeAlfCtuBuffers( int numEntries )
{
  for( int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++ )
  {
    m_alfCtuEnabled[compIdx].resize( numEntries );
    std::fill( m_alfCtuEnabled[compIdx].begin(), m_alfCtuEnabled[compIdx].end(), 0 );
  }

  m_alfCtbFilterIndex.resize(numEntries);
  for (int i = 0; i < numEntries; i++)
  {
    m_alfCtbFilterIndex[i] = 0;
  }

  for( int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++ )
  {
    m_alfCtuAlternative[compIdx].resize( numEntries );
    std::fill( m_alfCtuAlternative[compIdx].begin(), m_alfCtuAlternative[compIdx].end(), 0 );
  }
}


} // namespace vvenc

//! \}


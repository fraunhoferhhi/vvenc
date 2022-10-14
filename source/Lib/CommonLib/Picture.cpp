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


/** \file     Picture.cpp
 *  \brief    Description of a coded picture
 */

#include "Picture.h"
#include "SEI.h"

#include <algorithm>
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
    const int refLayer = std::min<int>( slice.TLayer, NUM_AMAXBT_LAYER - 1 );
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
    const int refLayer = std::min<int>( slice.TLayer, NUM_AMAXBT_LAYER - 1 );
    m_uiBlkSize[ refLayer ] += blkStat.m_uiBlkSize[ refLayer ];
    m_uiNumBlk [ refLayer ] += blkStat.m_uiNumBlk [ refLayer ];
  }
}

void BlkStat::setSliceMaxBT( Slice& slice )
{
  if( ! slice.isIRAP() )
  {
    const int refLayer = std::min<int>( slice.TLayer, NUM_AMAXBT_LAYER - 1 );
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
    , gopEntry          ( nullptr )
    , refCounter        ( 0 )
    , poc               ( 0 )
    , TLayer            ( std::numeric_limits<uint32_t>::max() )
    , layerId           ( 0 )
    , isSubPicBorderSaved (false)
    , sliceDataNumBins  ( 0 )
    , cts               ( 0 )
    , ctsValid          ( false )
    , isPreAnalysis     ( false )
    , m_picShared       ( nullptr )
    , picInitialQP      ( -1 )
    , picInitialLambda  ( -1.0 )
    , picMemorySTA      ( -1 )
    , picVisActTL0      ( 0 )
    , picVisActY        ( 0 )
    , isSccWeak         ( false )
    , isSccStrong       ( false )
    , useScME           ( false )
    , useScMCTF         ( false )
    , useScTS           ( false )
    , useScBDPCM        ( false )
    , useScIBC          ( false )
    , useScLMCS         ( false )
    , useScSAO          ( false )
    , useScNumRefs      ( false )
    , useScFastMrg      ( 0 )
    , useQtbttSpeedUpMode( 0 )
    , seqBaseQp         ( 0 )
    , actualHeadBits    ( 0 )
    , actualTotalBits   ( 0 )
    , encRCPic          ( nullptr )
{
  std::fill_n( m_sharedBufs, (int)NUM_PIC_TYPES, nullptr );
  std::fill_n( m_bufsOrigPrev, NUM_PREV_FRAMES, nullptr );
  std::fill_n( minNoiseLevels, QPA_MAX_NOISE_LEVELS, 255u );
}

void Picture::create( ChromaFormat _chromaFormat, const Size& size, unsigned _maxCUSize, unsigned _margin, bool _decoder )
{
  UnitArea::operator=( UnitArea( _chromaFormat, Area( Position{ 0, 0 }, size ) ) );
  margin            =  _margin;

  if( _decoder )
  {
    m_picBufs[ PIC_RESIDUAL   ].create( _chromaFormat, Area( 0, 0, _maxCUSize, _maxCUSize ) );
    m_picBufs[ PIC_PREDICTION ].create( _chromaFormat, Area( 0, 0, _maxCUSize, _maxCUSize ) );
  }
}

void Picture::reset()
{
  // reset picture
  isInitDone          = false;
  isReconstructed     = false;
  isBorderExtended    = false;
  isReferenced        = true;
  isNeededForOutput   = true;
  isFinished          = false;
  isLongTerm          = false;
  encPic              = false;
  writePic            = false;
  precedingDRAP       = false;

  gopEntry            = nullptr;
  refCounter          = 0;
  poc                 = -1;
  TLayer              = std::numeric_limits<uint32_t>::max();

  actualHeadBits      = 0;
  actualTotalBits     = 0;

  std::fill_n( m_sharedBufs, (int)NUM_PIC_TYPES, nullptr );
  std::fill_n( m_bufsOrigPrev, NUM_PREV_FRAMES, nullptr );
  std::fill_n( minNoiseLevels, QPA_MAX_NOISE_LEVELS, 255u );

  encTime.resetTimer();
}

void Picture::destroy( bool bPicHeader )
{
  for (uint32_t t = 0; t < NUM_PIC_TYPES; t++)
  {
    m_picBufs[  t ].destroy();
  }
  if( cs )
  {
    if( bPicHeader && cs->picHeader )
    {
      delete cs->picHeader;
    }
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

void Picture::linkSharedBuffers( PelStorage* origBuf, PelStorage* filteredBuf, PelStorage* prevOrigBufs[ NUM_PREV_FRAMES ], PicShared* picShared )
{
  m_picShared                      = picShared;
  m_sharedBufs[ PIC_ORIGINAL ]     = origBuf;
  m_sharedBufs[ PIC_ORIGINAL_RSP ] = filteredBuf;
  for( int i = 0; i < NUM_PREV_FRAMES; i++ )
    m_bufsOrigPrev[ i ] = prevOrigBufs[ i ];
}

void Picture::releasePrevBuffers()
{
  for( int i = 0; i < NUM_PREV_FRAMES; i++ )
    m_bufsOrigPrev[ i ] = nullptr;
}

void Picture::releaseSharedBuffers()
{
  m_picShared                      = nullptr;
  m_sharedBufs[ PIC_ORIGINAL ]     = nullptr;
  m_sharedBufs[ PIC_ORIGINAL_RSP ] = nullptr;
}

void Picture::createTempBuffers( unsigned _maxCUSize )
{
  CHECK( !cs, "Coding structure is required a this point!" );

  // SAO reads/writes +-1 sample, especially SIMD
  m_picBufs[PIC_SAO_TEMP].create( chromaFormat, Y(), cs->pcv->maxCUSize, 2, MEMORY_ALIGN_DEF_SIZE );

  if( cs ) cs->rebindPicBufs();
}

void Picture::destroyTempBuffers()
{
  m_picBufs[PIC_SAO_TEMP].destroy();

  if( cs ) cs->rebindPicBufs();
}

const CPelBuf     Picture::getOrigBufPrev (const CompArea &blk, const PrevFrameType type) const { return (m_bufsOrigPrev[ type ] && blk.valid() ? m_bufsOrigPrev[ type ]->getBuf (blk) : PelBuf()); }
const CPelUnitBuf Picture::getOrigBufPrev (const PrevFrameType type) const { return (m_bufsOrigPrev[ type ] ? *m_bufsOrigPrev[ type ] : PelUnitBuf()); }
const CPelBuf     Picture::getOrigBufPrev (const ComponentID compID, const PrevFrameType type) const { return (m_bufsOrigPrev[ type ] ? m_bufsOrigPrev[ type ]->getBuf (compID) : PelBuf()); }

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
    CHECK( cs->sps != &sps,  "picture initialization error: sps changed" );
    CHECK( cs->vps != &_vps, "picture initialization error: vps changed" );
  }
  else
  {
    cs = new CodingStructure( unitCache, mutex );
    cs->pps = &pps;
    cs->sps = &sps;
    cs->vps = &_vps;
    cs->create( UnitArea( chromaFormatIDC, Area( 0, 0, iWidth, iHeight )), true, pps.pcv );
  }

  cs->picture   = this;
  cs->refCS     = cs;
  cs->slice     = nullptr;  // the slices for this picture have not been set at this point. update cs->slice after swapSliceObject()
  cs->picHeader = picHeader;
  if ( alfAps )
  {
    memcpy(cs->alfAps, alfAps, sizeof(cs->alfAps));
  }
  cs->lmcsAps = lmcsAps;
  cs->pcv     = pps.pcv;
  vps         = &_vps;
  dci         = nullptr;

  if( ! m_picBufs[ PIC_RECONSTRUCTION ].valid() )
  {
    m_picBufs[ PIC_RECONSTRUCTION ].create( chromaFormat, Area( lumaPos(), lumaSize() ), sps.CTUSize, margin, MEMORY_ALIGN_DEF_SIZE );
  }

  sliceDataStreams.clear();
  sliceDataNumBins = 0;
}

void Picture::setSccFlags( const VVEncCfg* encCfg )
{
  useScME      = encCfg->m_motionEstimationSearchMethodSCC > 0                          && isSccStrong;
  useScTS      = encCfg->m_TS == 1                || ( encCfg->m_TS == 2                && isSccWeak );
  useScBDPCM   = encCfg->m_useBDPCM == 1          || ( encCfg->m_useBDPCM == 2          && isSccWeak );
  useScMCTF    = encCfg->m_vvencMCTF.MCTF == 1    || ( encCfg->m_vvencMCTF.MCTF == 2    && ! isSccStrong );
  useScLMCS    = encCfg->m_lumaReshapeEnable == 1 || ( encCfg->m_lumaReshapeEnable == 2 && ! isSccStrong );
  useScIBC     = encCfg->m_IBCMode == 1           || ( encCfg->m_IBCMode == 2           && isSccStrong );
  useScSAO     = encCfg->m_bUseSAO                && ( !encCfg->m_saoScc                || isSccWeak );
  useScNumRefs = isSccStrong;
  useScFastMrg = isSccStrong ? std::max(0, encCfg->m_useFastMrg - 3) : std::max(0, encCfg->m_useFastMrg - 2);
  useQtbttSpeedUpMode = encCfg->m_qtbttSpeedUpMode;

  if( ( encCfg->m_qtbttSpeedUpMode & 2 ) && isSccStrong )
  {
    useQtbttSpeedUpMode &= ~1;
  }
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
    PelBuf p = m_picBufs[ PIC_RECONSTRUCTION ].get( compID );
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
      p = m_picBufs[ PIC_RECON_WRAP ].get( compID );
      p.copyFrom(m_picBufs[ PIC_RECONSTRUCTION ].get( compID ));
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

PelUnitBuf Picture::getPicBuf( const UnitArea& unit, const PictureType type )
{
  if( chromaFormat == CHROMA_400 )
  {
    return PelUnitBuf( chromaFormat, getPicBuf( unit.Y(), type ) );
  }
  else
  {
    return PelUnitBuf( chromaFormat, getPicBuf( unit.Y(), type ), getPicBuf( unit.Cb(), type ), getPicBuf( unit.Cr(), type ) );
  }
}

const CPelUnitBuf Picture::getPicBuf( const UnitArea& unit, const PictureType type ) const
{
  if( chromaFormat == CHROMA_400 )
  {
    return CPelUnitBuf( chromaFormat, getPicBuf( unit.Y(), type ) );
  }
  else
  {
    return CPelUnitBuf( chromaFormat, getPicBuf( unit.Y(), type ), getPicBuf( unit.Cb(), type ), getPicBuf( unit.Cr(), type ) );
  }
}

PelUnitBuf Picture::getSharedBuf( const UnitArea& unit, const PictureType type )
{
  if( chromaFormat == CHROMA_400 )
  {
    return PelUnitBuf( chromaFormat, getSharedBuf( unit.Y(), type ) );
  }
  else
  {
    return PelUnitBuf( chromaFormat, getSharedBuf( unit.Y(), type ), getSharedBuf( unit.Cb(), type ), getSharedBuf( unit.Cr(), type ) );
  }
}

const CPelUnitBuf Picture::getSharedBuf( const UnitArea& unit, const PictureType type ) const
{
  if( chromaFormat == CHROMA_400 )
  {
    return CPelUnitBuf( chromaFormat, getSharedBuf( unit.Y(), type ) );
  }
  else
  {
    return CPelUnitBuf( chromaFormat, getSharedBuf( unit.Y(), type ), getSharedBuf( unit.Cb(), type ), getSharedBuf( unit.Cr(), type ) );
  }
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


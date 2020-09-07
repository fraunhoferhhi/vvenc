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
    , isMctfFiltered    ( false )
    , isInitDone        ( false )
    , isReconstructed   ( false )
    , isBorderExtended  ( false )
    , isReferenced      ( false )
    , isNeededForOutput ( false )
    , isLongTerm        ( false )
    , encPic            ( true )
    , writePic          ( true )
    , precedingDRAP     ( false )
    , refCounter        ( 0 )
    , poc               ( 0 )
    , gopId             ( 0 )
    , TLayer            ( std::numeric_limits<uint32_t>::max() )
    , layerId           ( 0 )
    , isSubPicBorderSaved (false)
    , sliceDataNumBins  ( 0 )
    , cts               ( 0 )
    , ctsValid          ( false )
    , m_bufsOrigPrev    { nullptr, nullptr }
    , picInitialQP    ( 0 )
{
}

void Picture::create( ChromaFormat _chromaFormat, const Size& size, unsigned _maxCUSize, unsigned _margin, bool _decoder, int _padding )
{
  UnitArea::operator=( UnitArea( _chromaFormat, Area( Position{ 0, 0 }, size ) ) );
  margin            =  _margin;
  const Area a      = Area( Position(), size );
  m_bufs[ PIC_RECONSTRUCTION ].create( _chromaFormat, a, _maxCUSize, _margin, MEMORY_ALIGN_DEF_SIZE );
  m_bufs[ PIC_SAO_TEMP ].create( _chromaFormat, a, _maxCUSize, 0, MEMORY_ALIGN_DEF_SIZE );

  if( _decoder )
  {
    m_bufs[ PIC_RESIDUAL ].create( _chromaFormat, Area( 0, 0, _maxCUSize, _maxCUSize ) );
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
  if( cs ) cs->rebindPicBufs();
}

void Picture::destroyTempBuffers()
{
  if( cs ) cs->rebindPicBufs();
}

const CPelBuf     Picture::getOrigBufPrev (const CompArea &blk, const bool minus2) const { return (m_bufsOrigPrev[minus2 ? 1 : 0] && blk.valid() ? m_bufsOrigPrev[minus2 ? 1 : 0]->getBuf (blk) : PelBuf()); }
const CPelUnitBuf Picture::getOrigBufPrev (const bool minus2)   const { return (m_bufsOrigPrev[minus2 ? 1 : 0] ? *m_bufsOrigPrev[minus2 ? 1 : 0] : PelUnitBuf()); }
const CPelBuf     Picture::getOrigBufPrev (const ComponentID compID, const bool minus2) const { return (m_bufsOrigPrev[minus2 ? 1 : 0] ? m_bufsOrigPrev[minus2 ? 1 : 0]->getBuf (compID) : PelBuf()); }

void Picture::finalInit( const VPS& _vps, const SPS& sps, const PPS& pps, PicHeader& picHeader, XUCache& unitCache, std::mutex* mutex, APS** alfAps, APS* lmcsAps )
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
    cs->initStructData();
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
  cs->picHeader = &picHeader;
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

} // namespace vvenc

//! \}


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

#include "DepQuant.h"
#include "TrQuant.h"
#include "CodingStructure.h"
#include "UnitTools.h"

#include <bitset>

//! \ingroup CommonLib
//! \{

namespace vvenc {


namespace DQIntern
{
  static void findFirstPos( int& firstTestPos, const TCoeff* tCoeff, const DQIntern::TUParameters& tuPars, int defaultTh,
                            bool zeroOutForThres, int zeroOutWidth, int zeroOutHeight )
  {
    for( ; firstTestPos >= 0; firstTestPos-- )
    {
      if( zeroOutForThres && ( tuPars.m_scanId2BlkPos[firstTestPos].x >= zeroOutWidth ||
                              tuPars.m_scanId2BlkPos[firstTestPos].y >= zeroOutHeight ) )
      {
        continue;
      }
      if( abs( tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) > defaultTh )
      {
        break;
      }
    }
  }

  void Rom::xInitScanArrays()
  {
    if( m_scansInitialized )
    {
      return;
    }
    ::memset( m_scanId2NbInfoSbbArray, 0, sizeof(m_scanId2NbInfoSbbArray) );
    ::memset( m_scanId2NbInfoOutArray, 0, sizeof(m_scanId2NbInfoOutArray) );
    ::memset( m_tuParameters,          0, sizeof(m_tuParameters) );

    uint32_t raster2id[ MAX_CU_SIZE * MAX_CU_SIZE ];
    ::memset(raster2id, 0, sizeof(raster2id));

    for( int hd = 0; hd < MAX_TU_SIZE_IDX; hd++ )
    {
      for( int vd = 0; vd < MAX_TU_SIZE_IDX; vd++ )
      {
        if( (hd == 0 && vd <= 1) || (hd <= 1 && vd == 0) )
        {
          continue;
        }
        const uint32_t      blockWidth    = (1 << hd);
        const uint32_t      blockHeight   = (1 << vd);
        const uint32_t      log2CGWidth   = g_log2SbbSize[hd][vd][0];
        const uint32_t      log2CGHeight  = g_log2SbbSize[hd][vd][1];
        const uint32_t      groupWidth    = 1 << log2CGWidth;
        const uint32_t      groupHeight   = 1 << log2CGHeight;
        const uint32_t      groupSize     = groupWidth * groupHeight;
        const SizeType      blkWidthIdx   = Log2( blockWidth );
        const SizeType      blkHeightIdx  = Log2( blockHeight );
        const ScanElement * scanId2RP     = getScanOrder( SCAN_GROUPED_4x4, blkWidthIdx, blkHeightIdx );
        NbInfoSbb*&         sId2NbSbb     = m_scanId2NbInfoSbbArray[hd][vd];
        NbInfoOut*&         sId2NbOut     = m_scanId2NbInfoOutArray[hd][vd];
        // consider only non-zero-out region
        const uint32_t      blkWidthNZOut = std::min<unsigned>( JVET_C0024_ZERO_OUT_TH, blockWidth  );
        const uint32_t      blkHeightNZOut= std::min<unsigned>( JVET_C0024_ZERO_OUT_TH, blockHeight );
        const uint32_t      totalValues   = blkWidthNZOut * blkHeightNZOut;

        sId2NbSbb = new NbInfoSbb[ totalValues ];
        sId2NbOut = new NbInfoOut[ totalValues ];

        for( uint32_t scanId = 0; scanId < totalValues; scanId++ )
        {
          raster2id[scanId2RP[scanId].idx] = scanId;
          sId2NbSbb[scanId].numInv = 0;
        }

        for( unsigned scanId = 0; scanId < totalValues; scanId++ )
        {
          const int posX = scanId2RP[scanId].x;
          const int posY = scanId2RP[scanId].y;
          const int rpos = scanId2RP[scanId].idx;
          {
            //===== inside subband neighbours =====
            const int      begSbb = scanId - ( scanId & (groupSize-1) ); // first pos in current subblock
            int            cpos[5];

            cpos[0] = ( posX + 1 < blkWidthNZOut                              ? ( raster2id[rpos+1           ] < groupSize + begSbb ? raster2id[rpos+1           ] - begSbb : 0 ) : 0 );
            cpos[1] = ( posX + 2 < blkWidthNZOut                              ? ( raster2id[rpos+2           ] < groupSize + begSbb ? raster2id[rpos+2           ] - begSbb : 0 ) : 0 );
            cpos[2] = ( posX + 1 < blkWidthNZOut && posY + 1 < blkHeightNZOut ? ( raster2id[rpos+1+blockWidth] < groupSize + begSbb ? raster2id[rpos+1+blockWidth] - begSbb : 0 ) : 0 );
            cpos[3] = ( posY + 1 < blkHeightNZOut                             ? ( raster2id[rpos+  blockWidth] < groupSize + begSbb ? raster2id[rpos+  blockWidth] - begSbb : 0 ) : 0 );
            cpos[4] = ( posY + 2 < blkHeightNZOut                             ? ( raster2id[rpos+2*blockWidth] < groupSize + begSbb ? raster2id[rpos+2*blockWidth] - begSbb : 0 ) : 0 );

            int num = 0;
            int inPos[5] = { 0, };

            while( true )
            {
              int nk = -1;
              for( int k = 0; k < 5; k++ )
              {
                if( cpos[k] != 0 && ( nk < 0 || cpos[k] < cpos[nk] ) )
                {
                  nk = k;
                }
              }
              if( nk < 0 )
              {
                break;
              }
              inPos[ num++ ] = uint8_t( cpos[nk] );
              cpos[nk] = 0;
            }
            for( int k = num; k < 5; k++ )
            {
              inPos[k] = 0;
            }
            for( int k = 0; k < num; k++ )
            {
              CHECK( sId2NbSbb[begSbb + inPos[k]].numInv >= 5, "" );
              sId2NbSbb[begSbb + inPos[k]].invInPos[sId2NbSbb[begSbb + inPos[k]].numInv++] = scanId & ( groupSize - 1 );
            }
          }
          {
            //===== outside subband neighbours =====
            NbInfoOut&     nbOut  = sId2NbOut[ scanId ];
            const int      begSbb = scanId - ( scanId & (groupSize-1) ); // first pos in current subblock
            int            cpos[5];

            cpos[0] = ( posX + 1 < blkWidthNZOut                              ? ( raster2id[rpos+1           ] >= groupSize + begSbb ? raster2id[rpos+1           ] : 0 ) : 0 );
            cpos[1] = ( posX + 2 < blkWidthNZOut                              ? ( raster2id[rpos+2           ] >= groupSize + begSbb ? raster2id[rpos+2           ] : 0 ) : 0 );
            cpos[2] = ( posX + 1 < blkWidthNZOut && posY + 1 < blkHeightNZOut ? ( raster2id[rpos+1+blockWidth] >= groupSize + begSbb ? raster2id[rpos+1+blockWidth] : 0 ) : 0 );
            cpos[3] = ( posY + 1 < blkHeightNZOut                             ? ( raster2id[rpos+  blockWidth] >= groupSize + begSbb ? raster2id[rpos+  blockWidth] : 0 ) : 0 );
            cpos[4] = ( posY + 2 < blkHeightNZOut                             ? ( raster2id[rpos+2*blockWidth] >= groupSize + begSbb ? raster2id[rpos+2*blockWidth] : 0 ) : 0 );

            for( nbOut.num = 0; true; )
            {
              int nk = -1;
              for( int k = 0; k < 5; k++ )
              {
                if( cpos[k] != 0 && ( nk < 0 || cpos[k] < cpos[nk] ) )
                {
                  nk = k;
                }
              }
              if( nk < 0 )
              {
                break;
              }
              nbOut.outPos[ nbOut.num++ ] = uint16_t( cpos[nk] );
              cpos[nk] = 0;
            }
            for( int k = nbOut.num; k < 5; k++ )
            {
              nbOut.outPos[k] = 0;
            }
            nbOut.maxDist = ( scanId == 0 ? 0 : sId2NbOut[scanId-1].maxDist );
            for( int k = 0; k < nbOut.num; k++ )
            {
              if( nbOut.outPos[k] > nbOut.maxDist )
              {
                nbOut.maxDist = nbOut.outPos[k];
              }
            }
          }
        }

        // make it relative
        for( unsigned scanId = 0; scanId < totalValues; scanId++ )
        {
          NbInfoOut& nbOut  = sId2NbOut[scanId];
          const int  begSbb = scanId - ( scanId & (groupSize-1) ); // first pos in current subblock
          for( int k = 0; k < nbOut.num; k++ )
          {
            CHECK(begSbb > nbOut.outPos[k], "Position must be past sub block begin");
            nbOut.outPos[k] -= begSbb;
          }
          nbOut.maxDist -= scanId;
        }

        for( int chId = 0; chId < MAX_NUM_CH; chId++ )
        {
          m_tuParameters[hd][vd][chId] = new TUParameters( *this, blockWidth, blockHeight, ChannelType(chId) );
        }
      }
    }
    m_scansInitialized = true;
  }

  void Rom::xUninitScanArrays()
  {
    if( !m_scansInitialized )
    {
      return;
    }
    for( int hd = 0; hd < MAX_TU_SIZE_IDX; hd++ )
    {
      for( int vd = 0; vd < MAX_TU_SIZE_IDX; vd++ )
      {
        NbInfoSbb*& sId2NbSbb = m_scanId2NbInfoSbbArray[hd][vd];
        NbInfoOut*& sId2NbOut = m_scanId2NbInfoOutArray[hd][vd];
        if( sId2NbSbb )
        {
          delete [] sId2NbSbb;
        }
        if( sId2NbOut )
        {
          delete [] sId2NbOut;
        }
        for( int chId = 0; chId < MAX_NUM_CH; chId++ )
        {
          TUParameters*& tuPars = m_tuParameters[hd][vd][chId];
          if( tuPars )
          {
            delete tuPars;
          }
        }
      }
    }
    m_scansInitialized = false;
  }


  TUParameters::TUParameters( const Rom& rom, const unsigned width, const unsigned height, const ChannelType chType )
  {
    m_chType              = chType;
    m_width               = width;
    m_height              = height;
    const uint32_t nonzeroWidth  = std::min<uint32_t>(JVET_C0024_ZERO_OUT_TH, m_width);
    const uint32_t nonzeroHeight = std::min<uint32_t>(JVET_C0024_ZERO_OUT_TH, m_height);
    m_numCoeff                   = nonzeroWidth * nonzeroHeight;
    m_log2SbbWidth        = g_log2SbbSize[ Log2(m_width) ][ Log2(m_height) ][0];
    m_log2SbbHeight       = g_log2SbbSize[ Log2(m_width) ][ Log2(m_height) ][1];
    m_log2SbbSize         = m_log2SbbWidth + m_log2SbbHeight;
    m_sbbSize             = ( 1 << m_log2SbbSize );
    m_sbbMask             = m_sbbSize - 1;
    m_widthInSbb  = nonzeroWidth >> m_log2SbbWidth;
    m_heightInSbb = nonzeroHeight >> m_log2SbbHeight;
    m_numSbb              = m_widthInSbb * m_heightInSbb;
    SizeType        hsbb  = Log2( m_widthInSbb  );
    SizeType        vsbb  = Log2( m_heightInSbb );
    SizeType        hsId  = Log2( m_width  );
    SizeType        vsId  = Log2( m_height );
    m_scanSbbId2SbbPos    = getScanOrder( SCAN_UNGROUPED   , hsbb , vsbb );
    m_scanId2BlkPos       = getScanOrder( SCAN_GROUPED_4x4 , hsId , vsId );
    int log2W             = Log2( m_width  );
    int log2H             = Log2( m_height );
    m_scanId2NbInfoSbb    = rom.getNbInfoSbb( log2W, log2H );
    m_scanId2NbInfoOut    = rom.getNbInfoOut( log2W, log2H );
    m_scanInfo            = new ScanInfo[ m_numCoeff ];
    for( int scanIdx = 0; scanIdx < m_numCoeff; scanIdx++ )
    {
      xSetScanInfo( m_scanInfo[scanIdx], scanIdx );
    }
  }


  void TUParameters::xSetScanInfo( ScanInfo& scanInfo, int scanIdx )
  {
    scanInfo.sbbSize    = m_sbbSize;
    scanInfo.numSbb     = m_numSbb;
    scanInfo.scanIdx    = scanIdx;
    scanInfo.rasterPos  = m_scanId2BlkPos[scanIdx].idx;
    scanInfo.sbbPos     = m_scanSbbId2SbbPos[scanIdx >> m_log2SbbSize].idx;
    scanInfo.insidePos  = scanIdx & m_sbbMask;
    scanInfo.spt        = SCAN_ISCSBB;
    if(  scanInfo.insidePos == m_sbbMask && scanIdx > scanInfo.sbbSize && scanIdx < m_numCoeff - 1 )
      scanInfo.spt      = SCAN_SOCSBB;
    else if( scanInfo.insidePos == 0 && scanIdx > 0 && scanIdx < m_numCoeff - m_sbbSize )
      scanInfo.spt      = SCAN_EOCSBB;
    scanInfo.posX = m_scanId2BlkPos[scanIdx].x;
    scanInfo.posY = m_scanId2BlkPos[scanIdx].y;
    if( scanIdx )
    {
      const int nextScanIdx = scanIdx - 1;
      const int diag        = m_scanId2BlkPos[nextScanIdx].x + m_scanId2BlkPos[nextScanIdx].y;
      if( m_chType == CH_L )
      {
        scanInfo.sigCtxOffsetNext = ( diag < 2 ? 8 : diag < 5 ?  4 : 0 );
        scanInfo.gtxCtxOffsetNext = ( diag < 1 ? 16 : diag < 3 ? 11 : diag < 10 ? 6 : 1 );
      }
      else
      {
        scanInfo.sigCtxOffsetNext = ( diag < 2 ? 4 : 0 );
        scanInfo.gtxCtxOffsetNext = ( diag < 1 ? 6 : 1 );
      }
      scanInfo.nextInsidePos      = nextScanIdx & m_sbbMask;
      scanInfo.currNbInfoSbb      = m_scanId2NbInfoSbb[ scanIdx ];
      if( scanInfo.insidePos == 0 )
      {
        const int nextSbbPos  = m_scanSbbId2SbbPos[nextScanIdx >> m_log2SbbSize].idx;
        const int nextSbbPosY = nextSbbPos               / m_widthInSbb;
        const int nextSbbPosX = nextSbbPos - nextSbbPosY * m_widthInSbb;
        scanInfo.nextSbbRight = ( nextSbbPosX < m_widthInSbb  - 1 ? nextSbbPos + 1            : 0 );
        scanInfo.nextSbbBelow = ( nextSbbPosY < m_heightInSbb - 1 ? nextSbbPos + m_widthInSbb : 0 );
      }
    }
  }

  void RateEstimator::initCtx( const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID, const FracBitsAccess& fracBitsAccess )
  {
    m_scanId2Pos = tuPars.m_scanId2BlkPos;
    xSetSigSbbFracBits  ( fracBitsAccess, tuPars.m_chType );
    xSetSigFlagBits     ( fracBitsAccess, tuPars.m_chType );
    xSetGtxFlagBits     ( fracBitsAccess, tuPars.m_chType );
    xSetLastCoeffOffset ( fracBitsAccess, tuPars, tu, compID );
  }

  void RateEstimator::xSetLastCoeffOffset( const FracBitsAccess& fracBitsAccess, const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID )
  {
    const ChannelType chType = ( compID == COMP_Y ? CH_L : CH_C );
    int32_t cbfDeltaBits = 0;
    if( compID == COMP_Y && !CU::isIntra(*tu.cu) && !tu.depth )
    {
      const BinFracBits bits  = fracBitsAccess.getFracBitsArray( Ctx::QtRootCbf() );
      cbfDeltaBits            = int32_t( bits.intBits[1] ) - int32_t( bits.intBits[0] );
    }
    else
    {
      BinFracBits bits;
      bool prevLumaCbf           = false;
      bool lastCbfIsInferred     = false;
      bool useIntraSubPartitions = tu.cu->ispMode && isLuma(chType);
      if( useIntraSubPartitions )
      {
        bool rootCbfSoFar = false;
        bool isLastSubPartition = CU::isISPLast(*tu.cu, tu.Y(), compID);
        uint32_t nTus = tu.cu->ispMode == HOR_INTRA_SUBPARTITIONS ? tu.cu->lheight() >> Log2(tu.lheight()) : tu.cu->lwidth() >> Log2(tu.lwidth());
        if( isLastSubPartition )
        {
          TransformUnit* tuPointer = tu.cu->firstTU;
          for( int tuIdx = 0; tuIdx < nTus - 1; tuIdx++ )
          {
            rootCbfSoFar |= TU::getCbfAtDepth(*tuPointer, COMP_Y, tu.depth);
            tuPointer     = tuPointer->next;
          }
          if( !rootCbfSoFar )
          {
            lastCbfIsInferred = true;
          }
        }
        if( !lastCbfIsInferred )
        {
          prevLumaCbf = TU::getPrevTuCbfAtDepth(tu, compID, tu.depth);
        }
        bits = fracBitsAccess.getFracBitsArray(Ctx::QtCbf[compID](DeriveCtx::CtxQtCbf(compID, prevLumaCbf, true)));
      }
      else
      {
        bits = fracBitsAccess.getFracBitsArray(Ctx::QtCbf[compID](DeriveCtx::CtxQtCbf(compID, tu.cbf[COMP_Cb])));
      }
      cbfDeltaBits = lastCbfIsInferred ? 0 : int32_t(bits.intBits[1]) - int32_t(bits.intBits[0]);
    }

    static const unsigned prefixCtx[] = { 0, 0, 0, 3, 6, 10, 15, 21 };
    uint32_t              ctxBits  [ LAST_SIGNIFICANT_GROUPS ];
    for( unsigned xy = 0; xy < 2; xy++ )
    {
      int32_t             bitOffset   = ( xy ? cbfDeltaBits : 0 );
      int32_t*            lastBits    = ( xy ? m_lastBitsY : m_lastBitsX );
      const unsigned      size        = ( xy ? tuPars.m_height : tuPars.m_width );
      const unsigned      log2Size    = Log2( size );
      const bool          useYCtx     = ( xy != 0 );
      const CtxSet&       ctxSetLast  = ( useYCtx ? Ctx::LastY : Ctx::LastX )[ chType ];
      const unsigned      lastShift   = ( compID == COMP_Y ? (log2Size+1)>>2 : Clip3<unsigned>(0,2,size>>3) );
      const unsigned      lastOffset  = ( compID == COMP_Y ? ( prefixCtx[log2Size] ) : 0 );
      uint32_t            sumFBits    = 0;
      unsigned            maxCtxId    = g_uiGroupIdx[std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, size) - 1];
      for( unsigned ctxId = 0; ctxId < maxCtxId; ctxId++ )
      {
        const BinFracBits bits  = fracBitsAccess.getFracBitsArray( ctxSetLast( lastOffset + ( ctxId >> lastShift ) ) );
        ctxBits[ ctxId ]        = sumFBits + bits.intBits[0] + ( ctxId>3 ? ((ctxId-2)>>1)<<SCALE_BITS : 0 ) + bitOffset;
        sumFBits               +=            bits.intBits[1];
      }
      ctxBits  [ maxCtxId ]     = sumFBits + ( maxCtxId>3 ? ((maxCtxId-2)>>1)<<SCALE_BITS : 0 ) + bitOffset;
      for (unsigned pos = 0; pos < std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, size); pos++)
      {
        lastBits[ pos ]         = ctxBits[ g_uiGroupIdx[ pos ] ];
      }
    }
  }

  void RateEstimator::xSetSigSbbFracBits( const FracBitsAccess& fracBitsAccess, ChannelType chType )
  {
    const CtxSet& ctxSet = Ctx::SigCoeffGroup[ chType ];
    for( unsigned ctxId = 0; ctxId < sm_maxNumSigSbbCtx; ctxId++ )
    {
      m_sigSbbFracBits[ ctxId ] = fracBitsAccess.getFracBitsArray( ctxSet( ctxId ) );
    }
  }

  void RateEstimator::xSetSigFlagBits( const FracBitsAccess& fracBitsAccess, ChannelType chType )
  {
    for( unsigned ctxSetId = 0; ctxSetId < sm_numCtxSetsSig; ctxSetId++ )
    {
      BinFracBits*    bits    = m_sigFracBits [ ctxSetId ];
      const CtxSet&   ctxSet  = Ctx::SigFlag  [ chType + 2*ctxSetId ];
      const unsigned  numCtx  = ( chType == CH_L ? 12 : 8 );
      for( unsigned ctxId = 0; ctxId < numCtx; ctxId++ )
      {
        bits[ ctxId ] = fracBitsAccess.getFracBitsArray( ctxSet( ctxId ) );
      }
    }
  }

  void RateEstimator::xSetGtxFlagBits( const FracBitsAccess& fracBitsAccess, ChannelType chType )
  {
    const CtxSet&   ctxSetPar   = Ctx::ParFlag [     chType ];
    const CtxSet&   ctxSetGt1   = Ctx::GtxFlag [ 2 + chType ];
    const CtxSet&   ctxSetGt2   = Ctx::GtxFlag [     chType ];
    const unsigned  numCtx      = ( chType == CH_L ? 21 : 11 );
    for( unsigned ctxId = 0; ctxId < numCtx; ctxId++ )
    {
      BinFracBits     fbPar = fracBitsAccess.getFracBitsArray( ctxSetPar( ctxId ) );
      BinFracBits     fbGt1 = fracBitsAccess.getFracBitsArray( ctxSetGt1( ctxId ) );
      BinFracBits     fbGt2 = fracBitsAccess.getFracBitsArray( ctxSetGt2( ctxId ) );
      CoeffFracBits&  cb    = m_gtxFracBits[ ctxId ];
      int32_t         par0  = (1<<SCALE_BITS) + int32_t(fbPar.intBits[0]);
      int32_t         par1  = (1<<SCALE_BITS) + int32_t(fbPar.intBits[1]);
      cb.bits[0] = 0;
      cb.bits[1] = fbGt1.intBits[0] + (1 << SCALE_BITS);
      cb.bits[2] = fbGt1.intBits[1] + par0 + fbGt2.intBits[0];
      cb.bits[3] = fbGt1.intBits[1] + par1 + fbGt2.intBits[0];
      cb.bits[4] = fbGt1.intBits[1] + par0 + fbGt2.intBits[1];
      cb.bits[5] = fbGt1.intBits[1] + par1 + fbGt2.intBits[1];
    }
  }

  void CommonCtx::update( const ScanInfo& scanInfo, const int prevId, int stateId, StateMem& curr )
  {
    uint8_t*    sbbFlags  = m_currSbbCtx[stateId].sbbFlags;
    uint8_t*    levels    = m_currSbbCtx[stateId].levels;
    uint16_t    maxDist   = m_nbInfo[scanInfo.scanIdx - 1].maxDist;
    uint16_t    sbbSize   = scanInfo.sbbSize;
    std::size_t setCpSize = ( maxDist > sbbSize ? maxDist - sbbSize : 0 ) * sizeof( uint8_t );
    if( prevId >= 0 )
    {
      ::memcpy( sbbFlags, m_prevSbbCtx[prevId].sbbFlags, scanInfo.numSbb * sizeof( uint8_t ) );
      ::memcpy( levels + scanInfo.scanIdx + sbbSize, m_prevSbbCtx[prevId].levels + scanInfo.scanIdx + sbbSize, setCpSize );
    }
    else
    {
      ::memset( sbbFlags, 0, scanInfo.numSbb * sizeof( uint8_t ) );
      ::memset( levels + scanInfo.scanIdx + sbbSize, 0, setCpSize );
    }
    sbbFlags[scanInfo.sbbPos] = !!curr.numSig[stateId];

    const int       sigNSbb = ( ( scanInfo.nextSbbRight ? sbbFlags[scanInfo.nextSbbRight] : false ) || ( scanInfo.nextSbbBelow ? sbbFlags[scanInfo.nextSbbBelow] : false ) ? 1 : 0 );
    curr.refSbbCtxId[stateId] = stateId;
    const BinFracBits sbbBits = m_sbbFlagBits[sigNSbb];

    curr.sbbBits0[stateId] = sbbBits.intBits[0];
    curr.sbbBits1[stateId] = sbbBits.intBits[1];

    if( sigNSbb || ( ( scanInfo.nextSbbRight && scanInfo.nextSbbBelow ) ? sbbFlags[scanInfo.nextSbbBelow + 1] : false ) )
    {
      const int         scanBeg = scanInfo.scanIdx - scanInfo.sbbSize;
      const NbInfoOut* nbOut = m_nbInfo + scanBeg;
      const uint8_t* absLevels = levels + scanBeg;

      for( int id = 0; id < scanInfo.sbbSize; id++, nbOut++ )
      {
        if( nbOut->num )
        {
          TCoeff sumAbs = 0, sumAbs1 = 0, sumNum = 0;
#define UPDATE(k) {TCoeff t=absLevels[nbOut->outPos[k]]; sumAbs+=t; sumAbs1+=std::min<TCoeff>(4+(t&1),t); sumNum+=!!t; }
          switch( nbOut->num )
          {
          default:
          case 5:
            UPDATE( 4 );
          case 4:
            UPDATE( 3 );
          case 3:
            UPDATE( 2 );
          case 2:
            UPDATE( 1 );
          case 1:
            UPDATE( 0 );
          }
#undef UPDATE
          curr.tplAcc[id][stateId] = ( sumNum << 5 ) | sumAbs1;
          curr.sum1st[id][stateId] = ( uint8_t ) std::min( 255, sumAbs );
        }
      }
    }
  }

  void Quantizer::initQuantBlock(const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda, int gValue)
  {
    CHECKD( lambda <= 0.0, "Lambda must be greater than 0" );

    const int         qpDQ                  = cQP.Qp(tu.mtsIdx[compID]==MTS_SKIP) + 1;
    const int         qpPer                 = qpDQ / 6;
    const int         qpRem                 = qpDQ - 6 * qpPer;
    const SPS&        sps                   = *tu.cs->sps;
    const CompArea&   area                  = tu.blocks[ compID ];
    const ChannelType chType                = toChannelType( compID );
    const int         channelBitDepth       = sps.bitDepths[ chType ];
    const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange();
    const int         nomTransformShift     = getTransformShift( channelBitDepth, area.size(), maxLog2TrDynamicRange );
    const bool    needsSqrt2ScaleAdjustment = TU::needsSqrt2Scale(tu, compID);
    const int         transformShift        = nomTransformShift + (needsSqrt2ScaleAdjustment?-1:0);
    // quant parameters
    m_QShift                    = QUANT_SHIFT  - 1 + qpPer + transformShift;
    m_QAdd                      = -( ( 3 << m_QShift ) >> 1 );
    Intermediate_Int  invShift  = IQUANT_SHIFT + 1 - qpPer - transformShift;
    m_QScale                    = g_quantScales[needsSqrt2ScaleAdjustment?1:0][ qpRem ];
    const unsigned    qIdxBD    = std::min<unsigned>( maxLog2TrDynamicRange + 1, 8*sizeof(Intermediate_Int) + invShift - IQUANT_SHIFT - 1 );
    m_maxQIdx                   = ( 1 << (qIdxBD-1) ) - 4;
    if( m_QShift )
      m_thresLast               = TCoeff((int64_t(m_DqThrVal) << (m_QShift-1)));
    else
      m_thresLast               = TCoeff((int64_t(m_DqThrVal>>1) << m_QShift));
    m_thresSSbb                 = TCoeff((int64_t(3) << m_QShift));
    // distortion calculation parameters
    const int64_t qScale        = (gValue==-1) ? m_QScale : gValue;
    const int nomDShift =
      SCALE_BITS - 2 * (nomTransformShift + DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth)) + m_QShift + (needsSqrt2ScaleAdjustment ? 1 : 0);
    const double  qScale2       = double( qScale * qScale );
    const double  nomDistFactor = ( nomDShift < 0 ? 1.0/(double(int64_t(1)<<(-nomDShift))*qScale2*lambda) : double(int64_t(1)<<nomDShift)/(qScale2*lambda) );
    const uint32_t pow2dfShift   = (uint32_t)( nomDistFactor * qScale2 ) + 1;
    const int     dfShift       = ceilLog2( pow2dfShift );
    m_DistShift                 = 62 + m_QShift - 2*maxLog2TrDynamicRange - dfShift;
    m_DistAdd                   = (int64_t(1) << m_DistShift) >> 1;
    m_DistStepAdd               = ((m_DistShift+m_QShift)>=64 ? (int64_t)( nomDistFactor * pow(2,m_DistShift+m_QShift) + .5 ) : (int64_t)( nomDistFactor * double(int64_t(1)<<(m_DistShift+m_QShift)) + .5 ));
    m_DistOrgFact               = (int64_t)( nomDistFactor * double(int64_t(1)<<(m_DistShift+1       )) + .5 );
  }

  void Quantizer::dequantBlock( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff, bool enableScalingLists, int* piDequantCoef) const
  {

    //----- set basic parameters -----
    const CompArea&     area      = tu.blocks[ compID ];
    const int           numCoeff  = area.area();
    const SizeType      hsId      = Log2( area.width );
    const SizeType      vsId      = Log2( area.height );
    const ScanElement  *scan      = getScanOrder( SCAN_GROUPED_4x4, hsId, vsId );
    const TCoeffSig*    qCoeff    = tu.getCoeffs( compID ).buf;
          TCoeff*       tCoeff    = recCoeff.buf;

    //----- reset coefficients and get last scan index -----
    ::memset( tCoeff, 0, numCoeff * sizeof( TCoeff ) );
    int lastScanIdx = tu.lastPos[compID];
    if( lastScanIdx < 0 )
    {
      return;
    }

    //----- set dequant parameters -----
    const int         qpDQ                  = cQP.Qp(tu.mtsIdx[compID]==MTS_SKIP) + 1;
    const int         qpPer                 = qpDQ / 6;
    const int         qpRem                 = qpDQ - 6 * qpPer;
    const SPS&        sps                   = *tu.cs->sps;
    const ChannelType chType                = toChannelType( compID );
    const int         channelBitDepth       = sps.bitDepths[ chType ];
    const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange();
    const TCoeff      minTCoeff             = -( 1 << maxLog2TrDynamicRange );
    const TCoeff      maxTCoeff             =  ( 1 << maxLog2TrDynamicRange ) - 1;
    const int         nomTransformShift     = getTransformShift( channelBitDepth, area.size(), maxLog2TrDynamicRange );
    const bool    needsSqrt2ScaleAdjustment = TU::needsSqrt2Scale(tu, compID);
    const int         transformShift        = nomTransformShift + (needsSqrt2ScaleAdjustment?-1:0);
    Intermediate_Int  shift                 = IQUANT_SHIFT + 1 - qpPer - transformShift + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);
    Intermediate_Int  invQScale             = g_invQuantScales[needsSqrt2ScaleAdjustment?1:0][ qpRem ];
    Intermediate_Int  add                   = (shift < 0) ? 0 : ((1 << shift) >> 1);
    //----- dequant coefficients -----
    for( int state = 0, scanIdx = lastScanIdx; scanIdx >= 0; scanIdx-- )
    {
      const unsigned   rasterPos = scan[scanIdx].idx;
      const TCoeffSig& level     = qCoeff[ rasterPos ];
      if( level )
      {
        if (enableScalingLists)
          invQScale = piDequantCoef[rasterPos];//scalingfactor*levelScale
        if (shift < 0 && (enableScalingLists || scanIdx == lastScanIdx))
        {
          invQScale <<= -shift;
        }
        Intermediate_Int qIdx = 2 * level + (level > 0 ? -(state>>1) : (state>>1));
        int64_t  nomTCoeff          = ((int64_t)qIdx * (int64_t)invQScale + add) >> ((shift < 0) ? 0 : shift);
        tCoeff[rasterPos]           = (TCoeff)Clip3<int64_t>(minTCoeff, maxTCoeff, nomTCoeff);
      }
      state = ( 32040 >> ((state<<2)+((level&1)<<1)) ) & 3;   // the 16-bit value "32040" represent the state transition table
    }
  }

  bool Quantizer::preQuantCoeff( const TCoeff absCoeff, PQData* pqData, int quanCoeff ) const
  {
    int64_t scaledOrg = int64_t( absCoeff ) * quanCoeff;
    TCoeff  qIdx      = TCoeff( ( scaledOrg + m_QAdd ) >> m_QShift );

    if( qIdx < 0 )
    {
      int64_t scaledAdd = m_DistStepAdd - scaledOrg * m_DistOrgFact;
      PQData& pq_a      = pqData[1];
      PQData& pq_b      = pqData[2];

      pq_a.deltaDist    = ( ( scaledAdd + 0 * m_DistStepAdd ) * 1 + m_DistAdd ) >> m_DistShift;
      pq_a.absLevel     = 1;

      pq_b.deltaDist    = ( ( scaledAdd + 1 * m_DistStepAdd ) * 2 + m_DistAdd ) >> m_DistShift;
      pq_b.absLevel     = 1;
      
      return true;
    }
     
    qIdx              = std::max<TCoeff>( 1, std::min<TCoeff>( m_maxQIdx, qIdx ) );
    int64_t scaledAdd = qIdx * m_DistStepAdd - scaledOrg * m_DistOrgFact;

    PQData& pq_a      = pqData[( qIdx + 0 ) & 3];
    PQData& pq_b      = pqData[( qIdx + 1 ) & 3];
    PQData& pq_c      = pqData[( qIdx + 2 ) & 3];
    PQData& pq_d      = pqData[( qIdx + 3 ) & 3];

    pq_a.deltaDist    = ( ( scaledAdd + 0 * m_DistStepAdd ) * ( qIdx + 0 ) + m_DistAdd ) >> m_DistShift;
    pq_a.absLevel     = ( qIdx + 1 ) >> 1;

    pq_b.deltaDist    = ( ( scaledAdd + 1 * m_DistStepAdd ) * ( qIdx + 1 ) + m_DistAdd ) >> m_DistShift;
    pq_b.absLevel     = ( qIdx + 2 ) >> 1;

    pq_c.deltaDist    = ( ( scaledAdd + 2 * m_DistStepAdd ) * ( qIdx + 2 ) + m_DistAdd ) >> m_DistShift;
    pq_c.absLevel     = ( qIdx + 3 ) >> 1;

    pq_d.deltaDist    = ( ( scaledAdd + 3 * m_DistStepAdd ) * ( qIdx + 3 ) + m_DistAdd ) >> m_DistShift;
    pq_d.absLevel     = ( qIdx + 4 ) >> 1;

    return false;
  }

  const int32_t g_goRiceBits[4][RICEMAX] =
  {
    { 32768,  65536,  98304, 131072, 163840, 196608, 262144, 262144, 327680, 327680, 327680, 327680, 393216, 393216, 393216, 393216, 393216, 393216, 393216, 393216, 458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752},
    { 65536,  65536,  98304,  98304, 131072, 131072, 163840, 163840, 196608, 196608, 229376, 229376, 294912, 294912, 294912, 294912, 360448, 360448, 360448, 360448, 360448, 360448, 360448, 360448, 425984, 425984, 425984, 425984, 425984, 425984, 425984, 425984},
    { 98304,  98304,  98304,  98304, 131072, 131072, 131072, 131072, 163840, 163840, 163840, 163840, 196608, 196608, 196608, 196608, 229376, 229376, 229376, 229376, 262144, 262144, 262144, 262144, 327680, 327680, 327680, 327680, 327680, 327680, 327680, 327680},
    {131072, 131072, 131072, 131072, 131072, 131072, 131072, 131072, 163840, 163840, 163840, 163840, 163840, 163840, 163840, 163840, 196608, 196608, 196608, 196608, 196608, 196608, 196608, 196608, 229376, 229376, 229376, 229376, 229376, 229376, 229376, 229376}
  };

  static inline void initStates( const int stateId, DQIntern::StateMem& state )
  {
    state.rdCost[stateId]         = DQIntern::rdCostInit;
    state.ctx.cff[stateId]        =  0;
    state.ctx.sig[stateId]        =  0;
    state.numSig[stateId]         =  0;
    state.refSbbCtxId[stateId]    = -1;
    state.remRegBins[stateId]     =  4;
    state.cffBitsCtxOffset        =  0;
    state.m_goRicePar[stateId]    =  0;
    state.m_goRiceZero[stateId]   =  0;
    state.sbbBits0[stateId]       =  0;
    state.sbbBits1[stateId]       =  0;
  }

  template<bool rrgEnsured = false>
  static inline void checkRdCosts( const int stateId, const DQIntern::ScanPosType spt, const DQIntern::PQData& pqDataA, const DQIntern::PQData& pqDataB, DQIntern::Decisions& decisions, int idxAZ, int idxB, const DQIntern::StateMem& state )
  {
    const int32_t* goRiceTab = DQIntern::g_goRiceBits[state.m_goRicePar[stateId]];
    int64_t         rdCostA = state.rdCost[stateId] + pqDataA.deltaDist;
    int64_t         rdCostB = state.rdCost[stateId] + pqDataB.deltaDist;
    int64_t         rdCostZ = state.rdCost[stateId];

    if( rrgEnsured || state.remRegBins[stateId] >= 4 )
    {
      const CoeffFracBits& cffBits = state.m_gtxFracBitsArray[state.ctx.cff[stateId]];
      const BinFracBits    sigBits = state.m_sigFracBitsArray[stateId][state.ctx.sig[stateId]];

      if( pqDataA.absLevel < 4 )
        rdCostA += cffBits.bits[pqDataA.absLevel];
      else
      {
        const unsigned value = ( pqDataA.absLevel - 4 ) >> 1;
        rdCostA += cffBits.bits[pqDataA.absLevel - ( value << 1 )] + goRiceTab[std::min<unsigned>( value, RICEMAX - 1 )];
      }

      if( pqDataB.absLevel < 4 )
        rdCostB += cffBits.bits[pqDataB.absLevel];
      else
      {
        const unsigned value = ( pqDataB.absLevel - 4 ) >> 1;
        rdCostB += cffBits.bits[pqDataB.absLevel - ( value << 1 )] + goRiceTab[std::min<unsigned>( value, RICEMAX - 1 )];
      }

      if( spt == SCAN_ISCSBB )
      {
        rdCostA += sigBits.intBits[1];
        rdCostB += sigBits.intBits[1];
        rdCostZ += sigBits.intBits[0];
      }
      else if( spt == SCAN_SOCSBB )
      {
        rdCostA += state.sbbBits1[stateId] + sigBits.intBits[1];
        rdCostB += state.sbbBits1[stateId] + sigBits.intBits[1];
        rdCostZ += state.sbbBits1[stateId] + sigBits.intBits[0];
      }
      else if( state.numSig[stateId] )
      {
        rdCostA += sigBits.intBits[1];
        rdCostB += sigBits.intBits[1];
        rdCostZ += sigBits.intBits[0];
      }
      else
      {
        rdCostZ = rdCostInit;
      }
    }
    else
    {
      rdCostA += ( 1 << SCALE_BITS ) + goRiceTab[pqDataA.absLevel <= state.m_goRiceZero[stateId] ? pqDataA.absLevel - 1 : std::min<int>( pqDataA.absLevel, RICEMAX - 1 )];
      rdCostB += ( 1 << SCALE_BITS ) + goRiceTab[pqDataB.absLevel <= state.m_goRiceZero[stateId] ? pqDataB.absLevel - 1 : std::min<int>( pqDataB.absLevel, RICEMAX - 1 )];
      rdCostZ += goRiceTab[state.m_goRiceZero[stateId]];
    }

    if( rdCostA < rdCostZ && rdCostA < decisions.rdCost[idxAZ] )
    {
      decisions.rdCost[idxAZ] = rdCostA;
      decisions.absLevel[idxAZ] = pqDataA.absLevel;
      decisions.prevId[idxAZ] = stateId;
    }
    else if( rdCostZ < decisions.rdCost[idxAZ] )
    {
      decisions.rdCost[idxAZ] = rdCostZ;
      decisions.absLevel[idxAZ] = 0;
      decisions.prevId[idxAZ] = stateId;
    }

    if( rdCostB < decisions.rdCost[idxB] )
    {
      decisions.rdCost[idxB] = rdCostB;
      decisions.absLevel[idxB] = pqDataB.absLevel;
      decisions.prevId[idxB] = stateId;
    }
  }

  void checkAllRdCosts( const DQIntern::ScanPosType spt, const DQIntern::PQData* pqData, DQIntern::Decisions& decisions, const DQIntern::StateMem& state )
  {
    checkRdCosts<true>( 0, spt, pqData[0], pqData[2], decisions, 0, 2, state );
    checkRdCosts<true>( 1, spt, pqData[0], pqData[2], decisions, 2, 0, state );
    checkRdCosts<true>( 2, spt, pqData[3], pqData[1], decisions, 1, 3, state );
    checkRdCosts<true>( 3, spt, pqData[3], pqData[1], decisions, 3, 1, state );
  }

  template<bool rrgEnsured = false>
  static void checkRdCostsOdd1( const int stateId, const ScanPosType spt, const int64_t deltaDist, Decisions& decisions, int idxA, int idxZ, const StateMem& state )
  {
    int64_t         rdCostA = state.rdCost[stateId] + deltaDist;
    int64_t         rdCostZ = state.rdCost[stateId];

    if( rrgEnsured || state.remRegBins[stateId] >= 4 )
    {
      const BinFracBits sigBits = state.m_sigFracBitsArray[stateId][state.ctx.sig[stateId]];

      rdCostA += state.cffBits1[state.ctx.cff[stateId]];

      if( spt == SCAN_ISCSBB )
      {
        rdCostA += sigBits.intBits[1];
        rdCostZ += sigBits.intBits[0];
      }
      else if( spt == SCAN_SOCSBB )
      {
        rdCostA += state.sbbBits1[stateId] + sigBits.intBits[1];
        rdCostZ += state.sbbBits1[stateId] + sigBits.intBits[0];
      }
      else if( state.numSig[stateId] )
      {
        rdCostA += sigBits.intBits[1];
        rdCostZ += sigBits.intBits[0];
      }
      else
      {
        rdCostZ = rdCostInit;
      }
    }
    else
    {
      const int32_t* goRiceTab = g_goRiceBits[state.m_goRicePar[stateId]];

      rdCostA += ( 1 << SCALE_BITS ) + goRiceTab[0];
      rdCostZ += goRiceTab[state.m_goRiceZero[stateId]];
    }

    if( rdCostA < decisions.rdCost[idxA] )
    {
      decisions.rdCost[idxA] = rdCostA;
      decisions.absLevel[idxA] = 1;
      decisions.prevId[idxA] = stateId;
    }

    if( rdCostZ < decisions.rdCost[idxZ] )
    {
      decisions.rdCost[idxZ] = rdCostZ;
      decisions.absLevel[idxZ] = 0;
      decisions.prevId[idxZ] = stateId;
    }
  }

  static void checkAllRdCostsOdd1( const DQIntern::ScanPosType spt, const int64_t pq_a_dist, const int64_t pq_b_dist, DQIntern::Decisions& decisions, const DQIntern::StateMem& state )
  {
    checkRdCostsOdd1<true>( 0, spt, pq_b_dist, decisions, 2, 0, state );
    checkRdCostsOdd1<true>( 1, spt, pq_b_dist, decisions, 0, 2, state );
    checkRdCostsOdd1<true>( 2, spt, pq_a_dist, decisions, 3, 1, state );
    checkRdCostsOdd1<true>( 3, spt, pq_a_dist, decisions, 1, 3, state );
  }

  static inline void checkRdCostStart( int32_t lastOffset, const PQData& pqData, Decisions& decisions, int idx, const StateMem& state )
  {
    const CoeffFracBits& cffBits = state.m_gtxFracBitsArray[0];

    int64_t rdCost = pqData.deltaDist + lastOffset;
    if( pqData.absLevel < 4 )
    {
      rdCost += cffBits.bits[pqData.absLevel];
    }
    else
    {
      const unsigned value = ( pqData.absLevel - 4 ) >> 1;
      rdCost += cffBits.bits[pqData.absLevel - ( value << 1 )] + g_goRiceBits[0][value < RICEMAX ? value : RICEMAX - 1];
    }

    if( rdCost < decisions.rdCost[idx] )
    {
      decisions.rdCost[idx]   = rdCost;
      decisions.absLevel[idx] = pqData.absLevel;
      decisions.prevId[idx]   = -1;
    }
  }

  static inline void checkRdCostSkipSbb( const int stateId, Decisions& decisions, int idx, const StateMem& state )
  {
    int64_t rdCost = state.rdCost[stateId] + state.sbbBits0[stateId];
    if( rdCost < decisions.rdCost[idx] )
    {
      decisions.rdCost[idx]   = rdCost;
      decisions.absLevel[idx] = 0;
      decisions.prevId[idx]   = 4 | stateId;
    }
  }

  static inline void checkRdCostSkipSbbZeroOut( const int stateId, Decisions& decisions, int idx, const StateMem& state )
  {
    int64_t rdCost          = state.rdCost[stateId] + state.sbbBits0[stateId];
    decisions.rdCost[idx]   = rdCost;
    decisions.absLevel[idx] = 0;
    decisions.prevId[idx]   = 4 | stateId;
  }

  static inline void setRiceParam( const int stateId, const ScanInfo& scanInfo, StateMem& state, bool ge4 )
  {
    if( state.remRegBins[stateId] < 4 || ge4 )
    {
      TCoeff  sumAbs = state.sum1st[scanInfo.insidePos][stateId];
      int sumSub     = state.remRegBins[stateId] < 4 ? 0 : 4 * 5;
      int sumAll     = std::max( std::min( 31, ( int ) sumAbs - sumSub ), 0 );
      state.m_goRicePar[stateId]
                     = g_auiGoRiceParsCoeff[sumAll];

      if( state.remRegBins[stateId] < 4 )
      {
        state.m_goRiceZero[stateId] = g_auiGoRicePosCoeff0( stateId, state.m_goRicePar[stateId] );
      }
    }
  }

  static void update1State( int stateId, const DQIntern::ScanInfo& scanInfo, const DQIntern::Decisions& decisions, DQIntern::StateMem& curr, DQIntern::StateMem& prev )
  {
    curr.rdCost[stateId] = decisions.rdCost[stateId];
    if( decisions.prevId[stateId] > -2 )
    {
      if( decisions.prevId[stateId] >= 0 )
      {
        const int prevId          = decisions.prevId[stateId];
        curr.numSig[stateId]      = prev.numSig[prevId] + !!decisions.absLevel[stateId];
        curr.refSbbCtxId[stateId] = prev.refSbbCtxId[prevId];
        curr.sbbBits0[stateId]    = prev.sbbBits0[prevId];
        curr.sbbBits1[stateId]    = prev.sbbBits1[prevId];
        curr.remRegBins[stateId]  = prev.remRegBins[prevId] - 1;

        if( curr.remRegBins[stateId] >= 4 )
        {
          curr.remRegBins[stateId] -= ( decisions.absLevel[stateId] < 2 ? decisions.absLevel[stateId] : 3 );
        }

        for( int i = 0; i < 16; i++ )
        {
          curr.tplAcc[i][stateId] = prev.tplAcc[i][prevId];
          curr.sum1st[i][stateId] = prev.sum1st[i][prevId];
          curr.absVal[i][stateId] = prev.absVal[i][prevId];
        }
      }
      else
      {
        curr.numSig[stateId]      =  1;
        curr.refSbbCtxId[stateId] = -1;
        curr.remRegBins[stateId]  = prev.initRemRegBins;
        curr.remRegBins[stateId] -= ( decisions.absLevel[stateId] < 2 ? decisions.absLevel[stateId] : 3 );

        for( int i = 0; i < 16; i++ )
        {
          curr.tplAcc[i][stateId] = 0;
          curr.sum1st[i][stateId] = 0;
          curr.absVal[i][stateId] = 0;
        }
      }

      if( decisions.absLevel[stateId] )
      {
        curr.absVal[scanInfo.insidePos][stateId] = ( uint8_t ) std::min<TCoeff>( 126 + ( decisions.absLevel[stateId] & 1 ), decisions.absLevel[stateId] );

        if( scanInfo.currNbInfoSbb.numInv )
        {
          int min4_or_5 = std::min<TCoeff>( 4 + ( decisions.absLevel[stateId] & 1 ), decisions.absLevel[stateId] );

          auto adds8 = []( uint8_t a, uint8_t b )
          {
            uint8_t c = a + b;
            if( c < a ) c = -1;
            return c;
          };

          auto update_deps = [&]( int k )
          {
            curr.tplAcc[scanInfo.currNbInfoSbb.invInPos[k]][stateId] += 32 + min4_or_5;
            curr.sum1st[scanInfo.currNbInfoSbb.invInPos[k]][stateId] = adds8( curr.sum1st[scanInfo.currNbInfoSbb.invInPos[k]][stateId], decisions.absLevel[stateId] );
          };

          switch( scanInfo.currNbInfoSbb.numInv )
          {
          default:
          case 5:
            update_deps( 4 );
          case 4:
            update_deps( 3 );
          case 3:
            update_deps( 2 );
          case 2:
            update_deps( 1 );
          case 1:
            update_deps( 0 );
          }
        }
      }

      if( curr.remRegBins[stateId] >= 4 )
      {
        TCoeff  sumAbs1 = curr.tplAcc[scanInfo.nextInsidePos][stateId] & 31;
        TCoeff  sumNum  = curr.tplAcc[scanInfo.nextInsidePos][stateId] >> 5u;
        int sumGt1 = sumAbs1 - sumNum;

        curr.ctx.sig[stateId] = scanInfo.sigCtxOffsetNext + std::min( ( sumAbs1 + 1 ) >> 1, 3 );
        curr.ctx.cff[stateId] = scanInfo.gtxCtxOffsetNext + std::min( sumGt1, 4 );
      }
      else
      {
        curr.anyRemRegBinsLt4 = true;
      }
    }
  }

  static void update1StateEOS( const int stateId, const DQIntern::ScanInfo& scanInfo, const DQIntern::Decisions& decisions, const DQIntern::StateMem& skip, DQIntern::StateMem& curr, DQIntern::StateMem& prev, DQIntern::CommonCtx& commonCtx )
  {
    curr.rdCost[stateId] = decisions.rdCost[stateId];

    if( decisions.prevId[stateId] > -2 )
    {
      if( decisions.prevId[stateId] >= 4 )
      {
        CHECK( decisions.absLevel[stateId] != 0, "cannot happen" );

        const int prevId          = decisions.prevId[stateId] - 4;
        curr.numSig    [stateId]  = 0;
        curr.remRegBins[stateId]  = skip.remRegBins[prevId];
        curr.refSbbCtxId[stateId] = prevId;

        for( int i = 0; i < 16; i++ )
        {
          curr.absVal[i][stateId] = 0;
        }
      }
      else if( decisions.prevId[stateId] >= 0 )
      {
        const int prevId          = decisions.prevId[stateId];
        curr.numSig[stateId]      = prev.numSig[prevId] + !!decisions.absLevel[stateId];
        curr.refSbbCtxId[stateId] = prev.refSbbCtxId[prevId];
        curr.remRegBins[stateId]  = prev.remRegBins[prevId] - 1;

        if( curr.remRegBins[stateId] >= 4 )
        {
          curr.remRegBins[stateId] -= ( decisions.absLevel[stateId] < 2 ? decisions.absLevel[stateId] : 3 );
        }

        for( int i = 0; i < 16; i++ )
        {
          curr.absVal[i][stateId] = prev.absVal[i][prevId];
        }
      }
      else
      {
        curr.numSig[stateId]      =  1;
        curr.refSbbCtxId[stateId] = -1;
        curr.remRegBins[stateId]  = prev.initRemRegBins;
        curr.remRegBins[stateId] -= ( decisions.absLevel[stateId] < 2 ? decisions.absLevel[stateId] : 3 );

        for( int i = 0; i < 16; i++ )
        {
          curr.absVal[i][stateId] = 0;
        }
      }

      curr.absVal[scanInfo.insidePos][stateId] = ( uint8_t ) std::min<TCoeff>( 126 + ( decisions.absLevel[stateId] & 1 ), decisions.absLevel[stateId] );

      uint8_t* levels[4];
      commonCtx.getLevelPtrs( scanInfo, levels[0], levels[1], levels[2], levels[3] );
      for( int i = 0; i < 16; i++ )
      {
        // save abs levels to commonCtx
        levels[stateId][i] = curr.absVal[i][stateId];
        // clean the SBB ctx
        curr.tplAcc[i][stateId] = 0;
        curr.sum1st[i][stateId] = 0;
        curr.absVal[i][stateId] = 0;
      }

      commonCtx.update( scanInfo, curr.refSbbCtxId[stateId], stateId, curr );

      curr.numSig[stateId] = 0;

      if( curr.remRegBins[stateId] >= 4 )
      {
        TCoeff  sumAbs1 = curr.tplAcc[scanInfo.nextInsidePos][stateId] & 31;
        TCoeff  sumNum  = curr.tplAcc[scanInfo.nextInsidePos][stateId] >> 5u;
        int sumGt1 = sumAbs1 - sumNum;

        curr.ctx.sig[stateId] = scanInfo.sigCtxOffsetNext + std::min( ( sumAbs1 + 1 ) >> 1, 3 );
        curr.ctx.cff[stateId] = scanInfo.gtxCtxOffsetNext + std::min( sumGt1, 4 );
      }
      else
      {
        curr.anyRemRegBinsLt4 = true;
      }
    }
  }

  static void updateStates( const DQIntern::ScanInfo& scanInfo, const DQIntern::Decisions& decisions, DQIntern::StateMem& curr )
  {
    DQIntern::StateMem prev = curr;
    curr.anyRemRegBinsLt4   = false;

    update1State( 0, scanInfo, decisions, curr, prev );
    update1State( 1, scanInfo, decisions, curr, prev );
    update1State( 2, scanInfo, decisions, curr, prev );
    update1State( 3, scanInfo, decisions, curr, prev );

    curr.cffBitsCtxOffset = scanInfo.gtxCtxOffsetNext;
  }

  static void updateStatesEOS( const DQIntern::ScanInfo& scanInfo, const DQIntern::Decisions& decisions, const DQIntern::StateMem& skip, DQIntern::StateMem& curr, DQIntern::CommonCtx& commonCtx )
  {
    DQIntern::StateMem prev = curr;
    curr.anyRemRegBinsLt4   = false;

    update1StateEOS( 0, scanInfo, decisions, skip, curr, prev, commonCtx );
    update1StateEOS( 1, scanInfo, decisions, skip, curr, prev, commonCtx );
    update1StateEOS( 2, scanInfo, decisions, skip, curr, prev, commonCtx );
    update1StateEOS( 3, scanInfo, decisions, skip, curr, prev, commonCtx );

    curr.cffBitsCtxOffset = scanInfo.gtxCtxOffsetNext;
  }
}; // namespace DQIntern

static const DQIntern::Decisions startDec[2] =
{
  DQIntern::Decisions
  {
    { DQIntern::rdCostInit >> 2, DQIntern::rdCostInit >> 2, DQIntern::rdCostInit >> 2, DQIntern::rdCostInit >> 2 },
    { -1, -1, -1, -1 },
    { -2, -2, -2, -2 },
  },
  DQIntern::Decisions
  {
    { DQIntern::rdCostInit >> 2, DQIntern::rdCostInit >> 2, DQIntern::rdCostInit >> 2, DQIntern::rdCostInit >> 2 },
    { 0, 0, 0, 0 },
    { 4, 5, 6, 7 },
  }
};

void DepQuant::xQuantDQ( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP, const double lambda, const Ctx& ctx, TCoeff& absSum, bool enableScalingLists, int* quantCoeff )
{
  using namespace DQIntern;
  
  //===== reset / pre-init =====
  const TUParameters& tuPars  = *m_scansRom->getTUPars( tu.blocks[compID], compID );
  m_quant.initQuantBlock    ( tu, compID, cQP, lambda );
  TCoeffSig*    qCoeff      = tu.getCoeffs( compID ).buf;
  const TCoeff* tCoeff      = srcCoeff.buf;
  const int     numCoeff    = tu.blocks[compID].area();
  ::memset( qCoeff, 0x00, numCoeff * sizeof( TCoeffSig ) );
  absSum                    = 0;

  const CompArea& area      = tu.blocks[ compID ];
  const uint32_t  width     = area.width;
  const uint32_t  height    = area.height;
  const uint32_t  lfnstIdx  = tu.cu->lfnstIdx;
  //===== scaling matrix ====
  //const int         qpDQ = cQP.Qp + 1;
  //const int         qpPer = qpDQ / 6;
  //const int         qpRem = qpDQ - 6 * qpPer;

  //TCoeff thresTmp = thres;
  bool zeroOut = false;
  bool zeroOutforThres = false;
  int effWidth = tuPars.m_width, effHeight = tuPars.m_height;
  if( ( tu.mtsIdx[compID] > MTS_SKIP || ( tu.cs->sps->MTS && tu.cu->sbtInfo != 0 && tuPars.m_height <= 32 && tuPars.m_width <= 32 ) ) && compID == COMP_Y )
  {
    effHeight = ( tuPars.m_height == 32 ) ? 16 : tuPars.m_height;
    effWidth  = ( tuPars.m_width  == 32 ) ? 16 : tuPars.m_width;
    zeroOut   = ( effHeight < tuPars.m_height || effWidth < tuPars.m_width );
  }
  zeroOutforThres = zeroOut || ( 32 < tuPars.m_height || 32 < tuPars.m_width );
  //===== find first test position =====
  int firstTestPos = std::min<int>( tuPars.m_width, JVET_C0024_ZERO_OUT_TH ) * std::min<int>( tuPars.m_height, JVET_C0024_ZERO_OUT_TH ) - 1;
  if( lfnstIdx > 0 && tu.mtsIdx[compID] != MTS_SKIP && width >= 4 && height >= 4 )
  {
    firstTestPos = ( ( width == 4 && height == 4 ) || ( width == 8 && height == 8 ) )  ? 7 : 15 ;
  }

  const TCoeff defaultQuantisationCoefficient = (TCoeff)m_quant.getQScale();
  const TCoeff thres = m_quant.getLastThreshold();
  const int zeroOutWidth  = ( tuPars.m_width  == 32 && zeroOut ) ? 16 : 32;
  const int zeroOutHeight = ( tuPars.m_height == 32 && zeroOut ) ? 16 : 32;

  if( enableScalingLists )
  {
    for( ; firstTestPos >= 0; firstTestPos-- )
    {
      if( zeroOutforThres && ( tuPars.m_scanId2BlkPos[firstTestPos].x >= zeroOutWidth || tuPars.m_scanId2BlkPos[firstTestPos].y >= zeroOutHeight ) ) continue;

      const TCoeff thresTmp = TCoeff( thres / ( 4 * quantCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) );

      if( abs( tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) > thresTmp ) break;
    }
  }
  else
  {
    const TCoeff defaultTh = TCoeff( thres / ( defaultQuantisationCoefficient << 2 ) );

    m_findFirstPos( firstTestPos, tCoeff, tuPars, defaultTh, zeroOutforThres, zeroOutWidth, zeroOutHeight );
  }

  if( firstTestPos < 0 )
  {
    tu.lastPos[compID] = -1;
    return;
  }

  //===== real init =====
  RateEstimator::initCtx( tuPars, tu, compID, ctx.getFracBitsAcess() );
  m_commonCtx.reset( tuPars, *this );
  for( int k = 0; k < 4; k++ )
  {
    DQIntern::initStates( k, m_state_curr );
    DQIntern::initStates( k, m_state_skip );
    m_state_curr.m_sigFracBitsArray[k] = RateEstimator::sigFlagBits(k);
  }

  m_state_curr.m_gtxFracBitsArray = RateEstimator::gtxFracBits();
  //memset( m_state_curr.tplAcc, 0, sizeof( m_state_curr.tplAcc ) ); // will be set in updateStates{,EOS} before first access
  memset( m_state_curr.sum1st, 0, sizeof( m_state_curr.sum1st ) );   // will be accessed in setRiceParam before updateState{,EOS}
  //memset( m_state_curr.absVal, 0, sizeof( m_state_curr.absVal ) ); // will be set in updateStates{,EOS} before first access

  const int numCtx = isLuma( compID ) ? 21 : 11;
  const CoeffFracBits* const cffBits = gtxFracBits();
  for( int i = 0; i < numCtx; i++ )
  {
    m_state_curr.cffBits1[i] = cffBits[i].bits[1];
  }

  int effectWidth  = std::min( 32, effWidth );
  int effectHeight = std::min( 32, effHeight );
  m_state_curr.initRemRegBins   = ( effectWidth * effectHeight * MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT ) / 16;
  m_state_curr.anyRemRegBinsLt4 = true; // for the first coeff use scalar impl., because it check against the init state, which
                                        // prohibits some paths

  //===== populate trellis =====
  for( int scanIdx = firstTestPos; scanIdx >= 0; scanIdx-- )
  {
    const ScanInfo& scanInfo = tuPars.m_scanInfo[ scanIdx ];
    if( enableScalingLists )
    {
      m_quant.initQuantBlock( tu, compID, cQP, lambda, quantCoeff[scanInfo.rasterPos] );
      xDecideAndUpdate( abs( tCoeff[scanInfo.rasterPos] ), scanInfo, zeroOut && ( scanInfo.posX >= effWidth || scanInfo.posY >= effHeight ), quantCoeff[scanInfo.rasterPos] );
    }
    else
      xDecideAndUpdate( abs( tCoeff[scanInfo.rasterPos] ), scanInfo, zeroOut && ( scanInfo.posX >= effWidth || scanInfo.posY >= effHeight ), defaultQuantisationCoefficient );
  }

  //===== find best path =====
  int       prevId      = -1;
  int64_t   minPathCost =  0;
  for( int8_t stateId = 0; stateId < 4; stateId++ )
  {
    int64_t pathCost = m_trellis[0][0].rdCost[stateId];
    if( pathCost < minPathCost )
    {
      prevId      = stateId;
      minPathCost = pathCost;
    }
  }

  //===== backward scanning =====
  int scanIdx = 0;
  for( ; prevId >= 0; scanIdx++ )
  {
    TCoeffSig absLevel = m_trellis[scanIdx][prevId >> 2].absLevel[prevId & 3];
    int32_t blkpos     = tuPars.m_scanId2BlkPos[scanIdx].idx;
    qCoeff[ blkpos ]   = TCoeffSig( tCoeff[blkpos] < 0 ? -absLevel : absLevel );
    absSum            += absLevel;
    prevId             = m_trellis[scanIdx][prevId >> 2].prevId[prevId & 3];
  }

  tu.lastPos[compID] = scanIdx - 1;
}

void DepQuant::xDecide( const DQIntern::ScanInfo& scanInfo, const TCoeff absCoeff, const int lastOffset, DQIntern::Decisions& decisions, bool zeroOut, int quantCoeff )
{
  using namespace DQIntern;

  ::memcpy( &decisions, startDec, sizeof( Decisions ) );

  StateMem& skip = m_state_skip;

  if( zeroOut )
  {
    if( scanInfo.spt == SCAN_EOCSBB )
    {
      checkRdCostSkipSbbZeroOut( 0, decisions, 0, skip );
      checkRdCostSkipSbbZeroOut( 1, decisions, 1, skip );
      checkRdCostSkipSbbZeroOut( 2, decisions, 2, skip );
      checkRdCostSkipSbbZeroOut( 3, decisions, 3, skip );
    }
    return;
  }

  StateMem& prev = m_state_curr;

  /// start inline prequant
  int64_t scaledOrg = int64_t( absCoeff ) * quantCoeff;
  TCoeff  qIdx      = TCoeff( ( scaledOrg + m_quant.m_QAdd ) >> m_quant.m_QShift );

  if( qIdx < 0 )
  {
    int64_t scaledAdd = m_quant.m_DistStepAdd - scaledOrg * m_quant.m_DistOrgFact;
    int64_t pq_a_dist = ( ( scaledAdd + 0 * m_quant.m_DistStepAdd ) * 1 + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
    int64_t pq_b_dist = ( ( scaledAdd + 1 * m_quant.m_DistStepAdd ) * 2 + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
    /// stop inline prequant

    if( prev.anyRemRegBinsLt4 )
    {
      setRiceParam( 0, scanInfo, prev, false );
      checkRdCostsOdd1( 0, scanInfo.spt, pq_b_dist, decisions, 2, 0, prev );

      setRiceParam( 1, scanInfo, prev, false );
      checkRdCostsOdd1( 1, scanInfo.spt, pq_b_dist, decisions, 0, 2, prev );

      setRiceParam( 2, scanInfo, prev, false );
      checkRdCostsOdd1( 2, scanInfo.spt, pq_a_dist, decisions, 3, 1, prev );

      setRiceParam( 3, scanInfo, prev, false );
      checkRdCostsOdd1( 3, scanInfo.spt, pq_a_dist, decisions, 1, 3, prev );
    }
    else
    {
      // has to be called as a first check, assumes no decision has been made yet
      m_checkAllRdCostsOdd1( scanInfo.spt, pq_a_dist, pq_b_dist, decisions, prev );
    }

    checkRdCostStart( lastOffset, PQData{ 1, pq_b_dist }, decisions, 2, prev );
  }
  else
  {
    /// start inline prequant
    qIdx = std::max<TCoeff>( 1, std::min<TCoeff>( m_quant.m_maxQIdx, qIdx ) );
    int64_t scaledAdd = qIdx * m_quant.m_DistStepAdd - scaledOrg * m_quant.m_DistOrgFact;

    PQData  pqData[4];

    PQData& pq_a = pqData[( qIdx + 0 ) & 3];
    PQData& pq_b = pqData[( qIdx + 1 ) & 3];
    PQData& pq_c = pqData[( qIdx + 2 ) & 3];
    PQData& pq_d = pqData[( qIdx + 3 ) & 3];

    pq_a.deltaDist = ( ( scaledAdd + 0 * m_quant.m_DistStepAdd ) * ( qIdx + 0 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
    pq_a.absLevel = ( qIdx + 1 ) >> 1;

    pq_b.deltaDist = ( ( scaledAdd + 1 * m_quant.m_DistStepAdd ) * ( qIdx + 1 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
    pq_b.absLevel = ( qIdx + 2 ) >> 1;

    pq_c.deltaDist = ( ( scaledAdd + 2 * m_quant.m_DistStepAdd ) * ( qIdx + 2 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
    pq_c.absLevel = ( qIdx + 3 ) >> 1;

    pq_d.deltaDist = ( ( scaledAdd + 3 * m_quant.m_DistStepAdd ) * ( qIdx + 3 ) + m_quant.m_DistAdd ) >> m_quant.m_DistShift;
    pq_d.absLevel = ( qIdx + 4 ) >> 1;
    /// stop inline prequant

    bool cff02ge4 = pqData[0].absLevel >= 4/* || pqData[2].absLevel >= 4 */;
    bool cff13ge4 = /* pqData[1].absLevel >= 4 || */ pqData[3].absLevel >= 4;

    if( cff02ge4 || cff13ge4 || prev.anyRemRegBinsLt4 )
    {
      if( prev.anyRemRegBinsLt4 || cff02ge4 )
      {
        setRiceParam( 0, scanInfo, prev, cff02ge4 );
        setRiceParam( 1, scanInfo, prev, cff02ge4 );
      }

      if( prev.anyRemRegBinsLt4 || cff13ge4 )
      {
        setRiceParam( 2, scanInfo, prev, cff13ge4 );
        setRiceParam( 3, scanInfo, prev, cff13ge4 );
      }

      checkRdCosts( 0, scanInfo.spt, pqData[0], pqData[2], decisions, 0, 2, prev );
      checkRdCosts( 1, scanInfo.spt, pqData[0], pqData[2], decisions, 2, 0, prev );
      checkRdCosts( 2, scanInfo.spt, pqData[3], pqData[1], decisions, 1, 3, prev );
      checkRdCosts( 3, scanInfo.spt, pqData[3], pqData[1], decisions, 3, 1, prev );
    }
    else
    {
      // has to be called as a first check, assumes no decision has been made yet
      m_checkAllRdCosts( scanInfo.spt, pqData, decisions, prev );
    }

    checkRdCostStart( lastOffset, pqData[0], decisions, 0, prev );
    checkRdCostStart( lastOffset, pqData[2], decisions, 2, prev );
  }

  if( scanInfo.spt == SCAN_EOCSBB )
  {
    checkRdCostSkipSbb( 0, decisions, 0, skip );
    checkRdCostSkipSbb( 1, decisions, 1, skip );
    checkRdCostSkipSbb( 2, decisions, 2, skip );
    checkRdCostSkipSbb( 3, decisions, 3, skip );
  }
}

void DepQuant::xDecideAndUpdate( const TCoeff absCoeff, const DQIntern::ScanInfo& scanInfo, bool zeroOut, int quantCoeff )
{
  using namespace DQIntern;

  Decisions* decisions = &m_trellis[scanInfo.scanIdx][0];

  xDecide( scanInfo, absCoeff, lastOffset( scanInfo.scanIdx ), *decisions, zeroOut, quantCoeff );

  if( scanInfo.scanIdx )
  {
    if( scanInfo.spt == SCAN_SOCSBB )
    {
      memcpy( &m_state_skip, &m_state_curr, DQIntern::StateMemSkipCpySize );
    }

    if( scanInfo.insidePos == 0 )
    {
      m_commonCtx.swap();
      m_updateStatesEOS( scanInfo, *decisions, m_state_skip, m_state_curr, m_commonCtx );
      ::memcpy( decisions + 1, decisions, sizeof( Decisions ) );
    }
    else if( !zeroOut )
    {
      m_updateStates( scanInfo, *decisions, m_state_curr );
    }
  }
}

void DepQuant::xDequantDQ( const TransformUnit& tu,  CoeffBuf& recCoeff, const ComponentID compID, const QpParam& cQP, bool enableScalingLists, int* piDequantCoef )
{
  m_quant.dequantBlock( tu, compID, cQP, recCoeff, enableScalingLists, piDequantCoef );
}

DepQuant::DepQuant( const Quant* other, bool enc, bool useScalingLists, bool enableOpt ) : QuantRDOQ2( other, useScalingLists ), RateEstimator(), m_commonCtx()
{
  const DepQuant* dq = dynamic_cast<const DepQuant*>( other );
  CHECK( other && !dq, "The DepQuant cast must be successfull!" );

  if( !dq )
  {
    m_scansRom = std::make_shared<DQIntern::Rom>();
    m_scansRom->init();
  }
  else
  {
    m_scansRom = dq->m_scansRom;
  }

  for( int t = 0; t < ( MAX_TB_SIZEY * MAX_TB_SIZEY ); t++ )
  {
    memcpy( m_trellis[t], startDec, sizeof( startDec ) );
  }

  m_checkAllRdCosts     = DQIntern::checkAllRdCosts;
  m_checkAllRdCostsOdd1 = DQIntern::checkAllRdCostsOdd1;
  m_updateStatesEOS     = DQIntern::updateStatesEOS;
  m_updateStates        = DQIntern::updateStates;
  m_findFirstPos        = DQIntern::findFirstPos;

  if( enableOpt )
  {
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_QUANT
    initDepQuantX86();
#endif
#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_QUANT
    initDepQuantARM();
#endif
  }
}

DepQuant::~DepQuant()
{
}

void DepQuant::quant( TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff& uiAbsSum, const QpParam& cQP, const Ctx& ctx )
{
  if( tu.cs->picture->useSelectiveRdoq && !xNeedRDOQ( tu, compID, pSrc, cQP ) )
  {
    tu.lastPos[compID] = -1;
    uiAbsSum           =  0;
  }
  else if( tu.cs->slice->depQuantEnabled && tu.mtsIdx[compID] != MTS_SKIP )
  {
    //===== scaling matrix ====
    const int         qpDQ            = cQP.Qp(tu.mtsIdx[compID]==MTS_SKIP) + 1;
    const int         qpPer           = qpDQ / 6;
    const int         qpRem           = qpDQ - 6 * qpPer;
    const CompArea    &rect           = tu.blocks[compID];
    const int         width           = rect.width;
    const int         height          = rect.height;
    uint32_t          scalingListType = getScalingListType(tu.cu->predMode, compID);
    CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");
    const uint32_t    log2TrWidth     = Log2(width);
    const uint32_t    log2TrHeight    = Log2(height);
    const bool isLfnstApplied         = tu.cu->lfnstIdx > 0 && (CU::isSepTree(*tu.cu) ? true : isLuma(compID));
    const bool enableScalingLists     = getUseScalingList(width, height, (tu.mtsIdx[compID] == MTS_SKIP), isLfnstApplied);
    xQuantDQ( tu, pSrc, compID, cQP, Quant::m_dLambda, ctx, uiAbsSum, enableScalingLists, Quant::getQuantCoeff(scalingListType, qpRem, log2TrWidth, log2TrHeight) );
  }
  else
  {
    QuantRDOQ2::quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
  }
}

void DepQuant::dequant( const TransformUnit& tu, CoeffBuf& dstCoeff, const ComponentID compID, const QpParam& cQP )
{
  if( tu.cs->slice->depQuantEnabled && (tu.mtsIdx[compID] != MTS_SKIP) )
  {
    const int         qpDQ            = cQP.Qp(tu.mtsIdx[compID]==MTS_SKIP) + 1;
    const int         qpPer           = qpDQ / 6;
    const int         qpRem           = qpDQ - 6 * qpPer;
    const CompArea    &rect           = tu.blocks[compID];
    const int         width           = rect.width;
    const int         height          = rect.height;
    uint32_t          scalingListType = getScalingListType(tu.cu->predMode, compID);
    CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");
    const uint32_t    log2TrWidth    = Log2(width);
    const uint32_t    log2TrHeight   = Log2(height);
    const bool isLfnstApplied        = tu.cu->lfnstIdx > 0 && (CU::isSepTree(*tu.cu) ? true : isLuma(compID));
    const bool enableScalingLists    = getUseScalingList(width, height, (tu.mtsIdx[compID] == MTS_SKIP), isLfnstApplied);
    xDequantDQ( tu, dstCoeff, compID, cQP, enableScalingLists, Quant::getDequantCoeff(scalingListType, qpRem, log2TrWidth, log2TrHeight) );
  }
  else
  {
    QuantRDOQ::dequant( tu, dstCoeff, compID, cQP );
  }
}

void DepQuant::init( int rdoq, bool useRDOQTS, int thrVal )
{
  QuantRDOQ2::init( rdoq, useRDOQTS, thrVal );
  m_quant.init( thrVal );
}

} // namespace vvenc

//! \}


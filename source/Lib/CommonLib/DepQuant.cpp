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
#include "DepQuantDecide.h"
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
    ::memset( m_scanSbb2NbSbbStartsArray, 0, sizeof(m_scanSbb2NbSbbStartsArray) );
    ::memset( m_nbOutShuffle16.idx,    0x80, sizeof(m_nbOutShuffle16.idx) );
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

        NbSbbStarts*&       sbb2NbStarts  = m_scanSbb2NbSbbStartsArray[hd][vd];
        if( groupWidth == 4 && groupHeight == 4 )
        {
          sbb2NbStarts = new NbSbbStarts[ totalValues / groupSize ];
          for( uint32_t sbb = 0; sbb < totalValues / groupSize; sbb++ )
          {
            std::fill_n( sbb2NbStarts[sbb].start, NUM_NB_SBB, -1 );
          }
        }

        for( uint32_t scanId = 0; scanId < totalValues; scanId++ )
        {
          raster2id[scanId2RP[scanId].idx] = scanId;
          sId2NbSbb[scanId].numInv = 0;
          sId2NbSbb[scanId].invInMask = 0;
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
              sId2NbSbb[begSbb + inPos[k]].invInMask |= uint16_t( 1 << ( scanId & ( groupSize - 1 ) ) );
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

            if( sbb2NbStarts )
            {
              xSetNbSbb16( sbb2NbStarts[scanId / groupSize], cpos, posX & 3, posY & 3, scanId & 15, begSbb );
            }

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

    // the shuffle table is complete only now
    for( int hd = 0; hd < MAX_TU_SIZE_IDX; hd++ )
    {
      for( int vd = 0; vd < MAX_TU_SIZE_IDX; vd++ )
      {
        if( m_scanSbb2NbSbbStartsArray[hd][vd] )
        {
          const unsigned numCoeff = std::min<unsigned>( JVET_C0024_ZERO_OUT_TH, 1 << hd ) * std::min<unsigned>( JVET_C0024_ZERO_OUT_TH, 1 << vd );
          xCheckNbSbb16( m_scanSbb2NbSbbStartsArray[hd][vd], m_scanId2NbInfoOutArray[hd][vd], numCoeff );
        }
      }
    }
    m_scansInitialized = true;
  }

  // Records for a position of a 4x4 SBB (x, y and insidePos inside it) in which neighbouring SBB each of its outside
  // neighbours cpos[k] lies (absolute scan positions, 0 = none) and where inside it.
  void Rom::xSetNbSbb16( NbSbbStarts& starts, const int cpos[5], int x, int y, int insidePos, int begSbb )
  {
    static const int dx[5] = { 1, 2, 1, 0, 0 };   // the order of cpos[]
    static const int dy[5] = { 0, 0, 1, 1, 2 };
    for( int k = 0; k < 5; k++ )
    {
      if( !cpos[k] )
      {
        continue;
      }
      const bool right = x + dx[k] >= 4;
      const bool below = y + dy[k] >= 4;
      CHECK( !right && !below, "An outside neighbour must lie in another SBB" );
      const int nbSbb = right && below ? NB_SBB_DIAG : ( right ? NB_SBB_RIGHT : NB_SBB_BELOW );
      const int start = ( cpos[k] & ~15 ) - begSbb;
      CHECK( starts.start[nbSbb] >= 0 && starts.start[nbSbb] != start, "A neighbouring SBB must have one start" );
      starts.start[nbSbb] = int16_t( start );
      uint8_t& idx = m_nbOutShuffle16.idx[k][nbSbb][insidePos];
      CHECK( idx != 0x80 && idx != ( cpos[k] & 15 ), "The scan inside a 4x4 SBB must be the same for every TU shape" );
      idx = uint8_t( cpos[k] & 15 );
    }
  }

  // Checks that the starts and the shuffle table give exactly the outside neighbours of NbInfoOut for every position,
  // and that only the neighbours the SIMD code reads are used: (x+1,y) and (x+2,y) from the right SBB, (x,y+1) and
  // (x,y+2) from the one below, (x+1,y+1) from any of the three.
  void Rom::xCheckNbSbb16( const NbSbbStarts* starts, const NbInfoOut* nbOut, unsigned numCoeff ) const
  {
    static const bool used[5][NUM_NB_SBB] = { { true, false, false }, { true, false, false }, { true, true, true },
                                              { false, true, false }, { false, true, false } };
    for( int k = 0; k < 5; k++ )
    {
      for( int nbSbb = 0; nbSbb < NUM_NB_SBB; nbSbb++ )
      {
        for( int p = 0; p < 16 && !used[k][nbSbb]; p++ )
        {
          CHECK( m_nbOutShuffle16.idx[k][nbSbb][p] != 0x80, "Neighbour lies in an SBB the SIMD code does not read" );
        }
      }
    }
    for( unsigned scanId = 0; scanId < numCoeff; scanId++ )
    {
      const NbSbbStarts& sbb = starts[scanId >> 4];
      std::vector<int> fromTable;
      for( int k = 0; k < 5; k++ )
      {
        for( int nbSbb = 0; nbSbb < NUM_NB_SBB; nbSbb++ )
        {
          const uint8_t idx = m_nbOutShuffle16.idx[k][nbSbb][scanId & 15];
          if( sbb.start[nbSbb] >= 0 && idx != 0x80 )
          {
            fromTable.push_back( sbb.start[nbSbb] + idx );
          }
        }
      }
      std::sort( fromTable.begin(), fromTable.end() );
      const std::vector<int> expected( nbOut[scanId].outPos, nbOut[scanId].outPos + nbOut[scanId].num );
      CHECK( fromTable != expected, "The neighbouring SBBs must give the outside neighbours" );
    }
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
        delete [] m_scanSbb2NbSbbStartsArray[hd][vd];
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
    m_scanSbb2NbSbbStarts = rom.getNbSbbStarts( log2W, log2H );
    m_nbOutShuffle16      = &rom.getNbOutShuffle16();
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
        // the templates of the next SBB reach maxDist past its last position; the first sbbSize levels of that reach are
        // this SBB's own, which updateStatesEOS stores, the rest a context copies from its previous one
        scanInfo.levelsCopySize = std::max<int>( m_scanId2NbInfoOut[nextScanIdx].maxDist - int( m_sbbSize ), 0 );
      }
    }
  }

  RateTablesMemo::RateTablesMemo()
  {
    // the coefficient-coding contexts must lie back to back in the context store, in the span initCtx compares
    const unsigned first = Ctx::SigCoeffGroup[ 0 ].Offset;
    const unsigned end   = Ctx::GtxFlag[ 3 ].Offset + Ctx::GtxFlag[ 3 ].Size;
    CHECK( end - first != RateEstimator::sm_numCoeffCtx, "coefficient-coding contexts are not one span" );
#if _DEBUG
    for( const CtxSet* set : { &Ctx::SigCoeffGroup[ 0 ], &Ctx::SigCoeffGroup[ 1 ], &Ctx::SigFlag[ 0 ], &Ctx::SigFlag[ 1 ],
                               &Ctx::SigFlag[ 2 ], &Ctx::SigFlag[ 3 ], &Ctx::SigFlag[ 4 ], &Ctx::SigFlag[ 5 ], &Ctx::ParFlag[ 0 ],
                               &Ctx::ParFlag[ 1 ], &Ctx::GtxFlag[ 0 ], &Ctx::GtxFlag[ 1 ], &Ctx::GtxFlag[ 2 ], &Ctx::GtxFlag[ 3 ] } )
    {
      CHECKD( set->Offset < first || set->Offset + set->Size > end, "a coefficient-coding context lies outside the span" );
    }
#endif
    // no real context has these bytes (a state's high byte is at most 0x7f), so the first call per channel type builds
    // the tables (and a kept table set is only read after it was built)
    for( Channel& c : channel ) memset( c.ctxStates, 0xff, sizeof( c.ctxStates ) );

    // where equalCoeffCtxStates expects each set of contexts (the end of the last one is checked above)
    CHECK( Ctx::SigCoeffGroup[ 1 ].Offset - first !=   2 || Ctx::SigFlag[ 0 ].Offset - first !=   4 || Ctx::SigFlag[ 1 ].Offset - first !=  16
        || Ctx::SigFlag[ 2 ].Offset - first !=  24 || Ctx::SigFlag[ 3 ].Offset - first !=  36 || Ctx::SigFlag[ 4 ].Offset - first !=  44
        || Ctx::SigFlag[ 5 ].Offset - first !=  56 || Ctx::ParFlag[ 0 ].Offset - first !=  64 || Ctx::ParFlag[ 1 ].Offset - first !=  85
        || Ctx::GtxFlag[ 0 ].Offset - first !=  96 || Ctx::GtxFlag[ 1 ].Offset - first != 117 || Ctx::GtxFlag[ 2 ].Offset - first != 128
        || Ctx::GtxFlag[ 3 ].Offset - first != 149, "the coefficient-coding contexts are not where equalCoeffCtxStates reads them" );
  }

  // The OR of the XORed 64-bit words of numCtx context models from firstCtx on (0 when they are equal), which the compiler
  // can vectorize; unlike memcmp it needs no ordering and no library call (whose speed differs between C runtimes). The
  // last word may overlap the one before it.
  template<unsigned firstCtx, unsigned numCtx>
  static inline uint64_t diffOfCtxs( const uint8_t* a, const uint8_t* b )
  {
    static_assert( numCtx * sizeof( BinProbModel ) >= 8, "diffOfCtxs compares at least one 64-bit word" );
    const unsigned first = firstCtx * unsigned( sizeof( BinProbModel ) );
    const unsigned last8 = first + numCtx * unsigned( sizeof( BinProbModel ) ) - 8;
    uint64_t diff = 0, x, y;
    for( unsigned i = first; i < last8; i += 8 )
    {
      memcpy( &x, a + i, 8 );
      memcpy( &y, b + i, 8 );
      diff |= x ^ y;
    }
    memcpy( &x, a + last8, 8 );
    memcpy( &y, b + last8, 8 );
    return diff | ( x ^ y );
  }

  // true when the context bytes the tables of this channel type are built from are equal. The contexts of the other
  // channel type take turns with these ones in the store, so each set is read on its own (at the indices checked in
  // RateTablesMemo's constructor).
  static inline bool equalCoeffCtxStates( const uint8_t* a, const uint8_t* b, const ChannelType chType )
  {
    if( chType == CH_L )
    {
      return ( diffOfCtxs<  0,  2>( a, b )     // SigCoeffGroup[0]
             | diffOfCtxs<  4, 12>( a, b )     // SigFlag[0]
             | diffOfCtxs< 24, 12>( a, b )     // SigFlag[2]
             | diffOfCtxs< 44, 12>( a, b )     // SigFlag[4]
             | diffOfCtxs< 64, 21>( a, b )     // ParFlag[0]
             | diffOfCtxs< 96, 21>( a, b )     // GtxFlag[0]
             | diffOfCtxs<128, 21>( a, b ) ) == 0;   // GtxFlag[2]
    }
    return ( diffOfCtxs<  2,  2>( a, b )       // SigCoeffGroup[1]
           | diffOfCtxs< 16,  8>( a, b )       // SigFlag[1]
           | diffOfCtxs< 36,  8>( a, b )       // SigFlag[3]
           | diffOfCtxs< 56,  8>( a, b )       // SigFlag[5]
           | diffOfCtxs< 85, 11>( a, b )       // ParFlag[1]
           | diffOfCtxs<117, 11>( a, b )       // GtxFlag[1]
           | diffOfCtxs<149, 11>( a, b ) ) == 0;     // GtxFlag[3]
  }

  void RateEstimator::initCtx( const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID, const FracBitsAccess& fracBitsAccess,
                               RateTablesMemo& memo )
  {
    // The sig and gtx tables depend only on the channel type and the states of the coefficient-coding contexts. The
    // memo keeps, per channel type, the last tables built and the raw context bytes they came from: when those are
    // unchanged, as they mostly are, the kept tables are still valid. The result is identical to building them every time.
    const ChannelType chType = tuPars.m_chType;
    RateTablesMemo::Channel& kept = memo.channel[ chType ];
    const uint8_t* ctxStates = reinterpret_cast<const uint8_t*>( &fracBitsAccess[ Ctx::SigCoeffGroup[ 0 ].Offset ] );
    if( !equalCoeffCtxStates( kept.ctxStates, ctxStates, chType ) )
    {
      xSetSigSbbFracBits( fracBitsAccess, chType, kept.tables );
      xSetSigFlagBits   ( fracBitsAccess, chType, kept.tables );
      xSetGtxFlagBits   ( fracBitsAccess, chType, kept.tables );
      memcpy( kept.ctxStates, ctxStates, sizeof( kept.ctxStates ) );
    }
    m_tables = &kept.tables;
    xSetLastCoeffOffset( fracBitsAccess, tuPars, tu, compID );
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

  void RateEstimator::xSetSigSbbFracBits( const FracBitsAccess& fracBitsAccess, ChannelType chType, RateTables& tables )
  {
    const CtxSet& ctxSet = Ctx::SigCoeffGroup[ chType ];
    for( unsigned ctxId = 0; ctxId < sm_maxNumSigSbbCtx; ctxId++ )
    {
      tables.sigSbb[ ctxId ] = fracBitsAccess.getFracBitsArray( ctxSet( ctxId ) );
    }
  }

  void RateEstimator::xSetSigFlagBits( const FracBitsAccess& fracBitsAccess, ChannelType chType, RateTables& tables )
  {
    for( unsigned ctxSetId = 0; ctxSetId < sm_numCtxSetsSig; ctxSetId++ )
    {
      BinFracBits*    bits    = tables.sig    [ ctxSetId ];
      const CtxSet&   ctxSet  = Ctx::SigFlag  [ chType + 2*ctxSetId ];
      const unsigned  numCtx  = ( chType == CH_L ? 12 : 8 );
      for( unsigned ctxId = 0; ctxId < numCtx; ctxId++ )
      {
        bits[ ctxId ] = fracBitsAccess.getFracBitsArray( ctxSet( ctxId ) );
      }
    }
  }

  void RateEstimator::xSetGtxFlagBits( const FracBitsAccess& fracBitsAccess, ChannelType chType, RateTables& tables )
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
      CoeffFracBits&  cb    = tables.gtx[ ctxId ];
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

  void CommonCtx::updateTemplates( const ScanInfo& scanInfo, const uint8_t* absLevels, int stateId, StateMem& curr ) const
  {
    const NbInfoOut* nbOut = m_nbInfo + scanInfo.scanIdx - scanInfo.sbbSize;

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

  // The coefficient rows dequantBlock and DepQuant::xQuantDQ clear. A block of 64 rows has coefficients only in its first
  // 32 (JVET-C0024 zero-out), and nothing reads the rows past them. For the dequantized buffer: neither the inverse
  // transform (TrQuant::xIT) nor the encoder's next forward transform and quantization of the buffer. For the quantized
  // levels: the dequantization and the residual coding read only scan positions, which lie in the first 32 rows. The LFNST
  // cut (xIT reads 4 or 8 rows) can't be used the same way: the encoder reuses the dequantized buffer in its next forward
  // LFNST pass, and the streams change without the full clear there.
  static int rowsToClear( const int height )
  {
    return std::min( height, JVET_C0024_ZERO_OUT_TH );
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

  void Quantizer::initQuantBlock( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda, QuantBlockMemo& memo )
  {
    CHECKD( tu.mtsIdx[compID] == MTS_SKIP, "the memo does not cover transform skip" );

    const int       qpDQ = cQP.Qp( false ) + 1;
    const CompArea& area = tu.blocks[compID];
    QuantBlockMemo::Entry& kept = memo.entry[toChannelType( compID )][Log2( area.width ) + Log2( area.height )];
    if( kept.qpDQ == qpDQ && kept.lambda == lambda )
    {
      static_cast<QuantBlockParams&>( *this ) = kept.params;
      return;
    }
    initQuantBlock( tu, compID, cQP, lambda );
    m_thresDefault = TCoeff( m_thresLast / ( TCoeff( m_QScale ) << 2 ) );
    kept.qpDQ      = qpDQ;
    kept.lambda    = lambda;
    kept.params    = *this;
  }

  void Quantizer::dequantBlock( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff, bool enableScalingLists, int* piDequantCoef) const
  {

    //----- set basic parameters -----
    const CompArea&     area      = tu.blocks[ compID ];
    const SizeType      hsId      = Log2( area.width );
    const SizeType      vsId      = Log2( area.height );
    const ScanElement  *scan      = getScanOrder( SCAN_GROUPED_4x4, hsId, vsId );
    const TCoeffSig*    qCoeff    = tu.getCoeffs( compID ).buf;
          TCoeff*       tCoeff    = recCoeff.buf;

    //----- reset the coefficients that are read and get last scan index -----
    ::memset( tCoeff, 0, rowsToClear( area.height ) * area.width * sizeof( TCoeff ) );
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
      // zero levels are not skipped by a (often mispredicted) branch: their qIdx is 0, so they give the 0 of the memset
      if (enableScalingLists)
        invQScale = piDequantCoef[rasterPos];//scalingfactor*levelScale
      if (shift < 0 && level && (enableScalingLists || scanIdx == lastScanIdx))
      {
        invQScale <<= -shift;
      }
      const int        sign      = (level > 0) - (level < 0);   // -1, 0 or 1
      Intermediate_Int qIdx      = 2 * level - sign * (state >> 1);
      int64_t  nomTCoeff          = ((int64_t)qIdx * (int64_t)invQScale + add) >> ((shift < 0) ? 0 : shift);
      tCoeff[rasterPos]           = (TCoeff)Clip3<int64_t>(minTCoeff, maxTCoeff, nomTCoeff);
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

  void checkAllRdCosts( const DQIntern::ScanPosType spt, const DQIntern::PQData* pqData, DQIntern::Decisions& decisions, const DQIntern::StateMem& state )
  {
    ::memcpy( &decisions, startDec, sizeof( Decisions ) );
    checkRdCosts<true>( 0, spt, pqData[0], pqData[2], decisions, 0, 2, state );
    checkRdCosts<true>( 1, spt, pqData[0], pqData[2], decisions, 2, 0, state );
    checkRdCosts<true>( 2, spt, pqData[3], pqData[1], decisions, 1, 3, state );
    checkRdCosts<true>( 3, spt, pqData[3], pqData[1], decisions, 3, 1, state );
  }

  static void checkAllRdCostsOdd1( const DQIntern::ScanPosType spt, const int64_t pq_a_dist, const int64_t pq_b_dist, DQIntern::Decisions& decisions, const DQIntern::StateMem& state )
  {
    ::memcpy( &decisions, startDec, sizeof( Decisions ) );
    checkRdCostsOdd1<true>( 0, spt, pq_b_dist, decisions, 2, 0, state );
    checkRdCostsOdd1<true>( 1, spt, pq_b_dist, decisions, 0, 2, state );
    checkRdCostsOdd1<true>( 2, spt, pq_a_dist, decisions, 3, 1, state );
    checkRdCostsOdd1<true>( 3, spt, pq_a_dist, decisions, 1, 3, state );
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

// The per-coefficient kernels through DepQuant's function pointers: the scalar trellis, and every scaling-list trellis
struct DepQuant::PointerKernels
{
  static void checkAllRdCosts( const DepQuant& dq, const DQIntern::ScanPosType spt, const DQIntern::PQData* pqData, DQIntern::Decisions& decisions, const DQIntern::StateMem& state )
  {
    dq.m_checkAllRdCosts( spt, pqData, decisions, state );
  }
  static void checkAllRdCostsGe4( const DepQuant& dq, const DQIntern::ScanPosType spt, const DQIntern::PQData* pqData, DQIntern::Decisions& decisions, const DQIntern::StateMem& state )
  {
    dq.m_checkAllRdCostsGe4( spt, pqData, decisions, state );
  }
  static void checkAllRdCostsOdd1( const DepQuant& dq, const DQIntern::ScanPosType spt, const int64_t pq_a_dist, const int64_t pq_b_dist, DQIntern::Decisions& decisions, const DQIntern::StateMem& state )
  {
    dq.m_checkAllRdCostsOdd1( spt, pq_a_dist, pq_b_dist, decisions, state );
  }
  static void updateStates( const DepQuant& dq, const DQIntern::ScanInfo& scanInfo, const DQIntern::Decisions& decisions, DQIntern::StateMem& curr )
  {
    dq.m_updateStates( scanInfo, decisions, curr );
  }
  static void updateStatesEOS( const DepQuant& dq, const DQIntern::ScanInfo& scanInfo, const DQIntern::Decisions& decisions, const DQIntern::StateMem& skip, DQIntern::StateMem& curr, DQIntern::CommonCtx& commonCtx )
  {
    dq.m_updateStatesEOS( scanInfo, decisions, skip, curr, commonCtx );
  }
};

void DepQuant::xQuantDQ( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP, const double lambda, const Ctx& ctx, TCoeff& absSum, bool enableScalingLists, int* quantCoeff )
{
  using namespace DQIntern;
  
  //===== reset / pre-init =====
  const TUParameters& tuPars  = *m_scansRom->getTUPars( tu.blocks[compID], compID );
  m_quant.initQuantBlock    ( tu, compID, cQP, lambda, m_quantBlockMemo );
  TCoeffSig*    qCoeff      = tu.getCoeffs( compID ).buf;
  const TCoeff* tCoeff      = srcCoeff.buf;
  ::memset( qCoeff, 0x00, rowsToClear( tu.blocks[compID].height ) * tu.blocks[compID].width * sizeof( TCoeffSig ) );
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
    m_findFirstPos( firstTestPos, tCoeff, tuPars, m_quant.m_thresDefault, zeroOutforThres, zeroOutWidth, zeroOutHeight );
  }

  if( firstTestPos < 0 )
  {
    tu.lastPos[compID] = -1;
    return;
  }

  //===== real init =====
  RateEstimator::initCtx( tuPars, tu, compID, ctx.getFracBitsAcess(), m_rateTablesMemo );
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
  m_firstScanIdx = firstTestPos;
  if( enableScalingLists )
  {
    for( int scanIdx = firstTestPos; scanIdx >= 0; scanIdx-- )
    {
      const ScanInfo& scanInfo = tuPars.m_scanInfo[ scanIdx ];
      m_quant.initQuantBlock( tu, compID, cQP, lambda, quantCoeff[scanInfo.rasterPos] );
      xDecideAndUpdate<PointerKernels>( abs( tCoeff[scanInfo.rasterPos] ), scanInfo, zeroOut && ( scanInfo.posX >= effWidth || scanInfo.posY >= effHeight ), quantCoeff[scanInfo.rasterPos] );
    }
  }
  else
  {
    m_populateTrellis( *this, tuPars, firstTestPos, tCoeff, zeroOut, effWidth, effHeight, defaultQuantisationCoefficient );
  }

  //===== find best path =====
  int       prevId      = -1;
  int64_t   minPathCost =  0;
  for( int stateId = 0; stateId < 4; stateId++ )
  {
    // No branch, as which state wins is close to random. It does the same as
    //   if( rdCost[stateId] < minPathCost ) { prevId = stateId; minPathCost = rdCost[stateId]; }
    const int64_t costDiff = m_trellis[0][0].rdCost[stateId] - minPathCost;
    const int64_t lower    = costDiff >> 63;
    prevId                += int( ( stateId - prevId ) & lower );
    minPathCost           += costDiff & lower;
  }

  //===== backward scanning =====
  const Decisions* row = m_trellis[0];
  TCoeff           sum = 0;
  int              scanIdx = 0;
  for( ; prevId >= 0; scanIdx++, row += 2 )
  {
    // Not row[prevId >> 2]: that multiplies by sizeof(Decisions) on the prevId load chain, and the walk runs 10-12 % slower.
    const Decisions& dec = prevId < 4 ? row[0] : row[1];
    TCoeffSig absLevel = dec.absLevel[prevId & 3];
    int32_t blkpos     = tuPars.m_scanId2BlkPos[scanIdx].idx;
    // apply the coefficient's sign without a branch: (absLevel ^ m) - m is -absLevel for m = -1 and absLevel for m = 0
    const int signMask = -int( tCoeff[blkpos] < 0 );
    qCoeff[ blkpos ]   = TCoeffSig( ( absLevel ^ signMask ) - signMask );
    sum               += absLevel;
    prevId             = dec.prevId[prevId & 3];
  }
  absSum = sum;

  tu.lastPos[compID] = scanIdx - 1;
}

void DepQuant::xDequantDQ( const TransformUnit& tu,  CoeffBuf& recCoeff, const ComponentID compID, const QpParam& cQP, bool enableScalingLists, int* piDequantCoef )
{
  m_quant.dequantBlock( tu, compID, cQP, recCoeff, enableScalingLists, piDequantCoef );
}

DepQuant::DepQuant( const Quant* other, bool enc, bool useScalingLists, bool enableOpt ) : QuantRDOQ2( other, useScalingLists, enableOpt ), RateEstimator(), m_commonCtx()
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
    memcpy( m_trellis[t], DQIntern::startDec, sizeof( DQIntern::startDec ) );
  }

  m_checkAllRdCosts     = DQIntern::checkAllRdCosts;
  m_checkAllRdCostsGe4  = DQIntern::checkAllRdCosts;   // the scalar checks add the Rice code's bits themselves
  m_checkAllRdCostsOdd1 = DQIntern::checkAllRdCostsOdd1;
  m_updateStatesEOS     = DQIntern::updateStatesEOS;
  m_updateStates        = DQIntern::updateStates;
  m_findFirstPos        = DQIntern::findFirstPos;
  m_populateTrellis     = xPopulateTrellis<PointerKernels>;

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
    // without scaling lists there is no table to find, and this runs for every transform block
    bool enableScalingLists = false;
    int* quantCoeff         = nullptr;
    if( getScalingListEnabled() )
    {
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
      enableScalingLists                = getUseScalingList(width, height, (tu.mtsIdx[compID] == MTS_SKIP), isLfnstApplied);
      quantCoeff                        = Quant::getQuantCoeff(scalingListType, qpRem, log2TrWidth, log2TrHeight);
    }
    xQuantDQ( tu, pSrc, compID, cQP, Quant::m_dLambda, ctx, uiAbsSum, enableScalingLists, quantCoeff );
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
    // without scaling lists there is no table to find, and this runs for every transform block
    bool enableScalingLists = false;
    int* dequantCoeff       = nullptr;
    if( getScalingListEnabled() )
    {
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
      enableScalingLists                = getUseScalingList(width, height, (tu.mtsIdx[compID] == MTS_SKIP), isLfnstApplied);
      dequantCoeff                      = Quant::getDequantCoeff(scalingListType, qpRem, log2TrWidth, log2TrHeight);
    }
    xDequantDQ( tu, dstCoeff, compID, cQP, enableScalingLists, dequantCoeff );
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
  m_quantBlockMemo = DQIntern::QuantBlockMemo();   // the threshold value is part of what it keeps
}

} // namespace vvenc

//! \}


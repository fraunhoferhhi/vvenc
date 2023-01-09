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

#include "DepQuant.h"
#include "TrQuant.h"
#include "CodingStructure.h"
#include "UnitTools.h"
#ifdef TARGET_SIMD_X86
#  include "x86/CommonDefX86.h"
#  include <simde/x86/sse4.1.h>
#endif

#include <bitset>

//! \ingroup CommonLib
//! \{

namespace vvenc {


namespace DQIntern
{
  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   R A T E   E S T I M A T O R                                        =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  struct NbInfoSbb
  {
    //uint8_t   num;
    uint8_t   numInv;
    //uint8_t   inPos[5];
    uint8_t   invInPos[5];
  };
  struct NbInfoOut
  {
    uint16_t  maxDist;
    uint16_t  num;
    uint16_t  outPos[5];
  };
  struct CoeffFracBits
  {
    int32_t   bits[6];
  };


  enum ScanPosType { SCAN_ISCSBB = 0, SCAN_SOCSBB = 1, SCAN_EOCSBB = 2 };

  struct ScanInfo
  {
    ScanInfo() {}
    int           sbbSize;
    int           numSbb;
    int           scanIdx;
    int           rasterPos;
    int           sbbPos;
    int           insidePos;
    ScanPosType   spt;
    unsigned      sigCtxOffsetNext;
    unsigned      gtxCtxOffsetNext;
    int           nextInsidePos;
    NbInfoSbb     currNbInfoSbb;
    int           nextSbbRight;
    int           nextSbbBelow;
    int           posX;
    int           posY;
  };

  class Rom;
  struct TUParameters
  {
    TUParameters ( const Rom& rom, const unsigned width, const unsigned height, const ChannelType chType );
    ~TUParameters()
    {
      delete [] m_scanInfo;
    }

    ChannelType       m_chType;
    unsigned          m_width;
    unsigned          m_height;
    unsigned          m_numCoeff;
    unsigned          m_numSbb;
    unsigned          m_log2SbbWidth;
    unsigned          m_log2SbbHeight;
    unsigned          m_log2SbbSize;
    unsigned          m_sbbSize;
    unsigned          m_sbbMask;
    unsigned          m_widthInSbb;
    unsigned          m_heightInSbb;
    const ScanElement *m_scanSbbId2SbbPos;
    const ScanElement *m_scanId2BlkPos;
    const NbInfoSbb*  m_scanId2NbInfoSbb;
    const NbInfoOut*  m_scanId2NbInfoOut;
    ScanInfo*         m_scanInfo;
  private:
    void xSetScanInfo( ScanInfo& scanInfo, int scanIdx );
  };

  class Rom
  {
  public:
    Rom() : m_scansInitialized(false) {}
    ~Rom() { xUninitScanArrays(); }
    void                init        ()                       { xInitScanArrays(); }
    const NbInfoSbb*    getNbInfoSbb( int hd, int vd ) const { return m_scanId2NbInfoSbbArray[hd][vd]; }
    const NbInfoOut*    getNbInfoOut( int hd, int vd ) const { return m_scanId2NbInfoOutArray[hd][vd]; }
    const TUParameters* getTUPars   ( const CompArea& area, const ComponentID compID ) const
    {
      return m_tuParameters[Log2(area.width)][Log2(area.height)][toChannelType(compID)];
    }
  private:
    void  xInitScanArrays   ();
    void  xUninitScanArrays ();
  private:
    bool          m_scansInitialized;
    NbInfoSbb*    m_scanId2NbInfoSbbArray[ MAX_TU_SIZE_IDX ][ MAX_TU_SIZE_IDX ];
    NbInfoOut*    m_scanId2NbInfoOutArray[ MAX_TU_SIZE_IDX ][ MAX_TU_SIZE_IDX ];
    TUParameters* m_tuParameters         [ MAX_TU_SIZE_IDX ][ MAX_TU_SIZE_IDX ][ MAX_NUM_CH ];
  };

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



  class RateEstimator
  {
  public:
    RateEstimator () {}
    ~RateEstimator() {}
    void initCtx  ( const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID, const FracBitsAccess& fracBitsAccess );

    inline const BinFracBits *sigSbbFracBits() const { return m_sigSbbFracBits; }
    inline const BinFracBits *sigFlagBits(unsigned stateId) const
    {
      return m_sigFracBits[std::max(((int) stateId) - 1, 0)];
    }
    inline const CoeffFracBits *gtxFracBits(unsigned stateId) const { return m_gtxFracBits; }
    inline int32_t              lastOffset(unsigned scanIdx) const
    {
      return m_lastBitsX[m_scanId2Pos[scanIdx].x] + m_lastBitsY[m_scanId2Pos[scanIdx].y];
    }

  private:
    void  xSetLastCoeffOffset ( const FracBitsAccess& fracBitsAccess, const TUParameters& tuPars, const TransformUnit& tu, const ComponentID compID );
    void  xSetSigSbbFracBits  ( const FracBitsAccess& fracBitsAccess, ChannelType chType );
    void  xSetSigFlagBits     ( const FracBitsAccess& fracBitsAccess, ChannelType chType );
    void  xSetGtxFlagBits     ( const FracBitsAccess& fracBitsAccess, ChannelType chType );

  private:
    static const unsigned sm_numCtxSetsSig    = 3;
    static const unsigned sm_numCtxSetsGtx    = 2;
    static const unsigned sm_maxNumSigSbbCtx  = 2;
    static const unsigned sm_maxNumSigCtx     = 12;
    static const unsigned sm_maxNumGtxCtx     = 21;

  private:
    const ScanElement * m_scanId2Pos;
    int32_t             m_lastBitsX      [ MAX_TB_SIZEY ];
    int32_t             m_lastBitsY      [ MAX_TB_SIZEY ];
    BinFracBits         m_sigSbbFracBits [ sm_maxNumSigSbbCtx ];
    BinFracBits         m_sigFracBits    [ sm_numCtxSetsSig   ][ sm_maxNumSigCtx ];
    CoeffFracBits       m_gtxFracBits                          [ sm_maxNumGtxCtx ];
  };

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





  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   D A T A   S T R U C T U R E S                                      =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/


  struct PQData
  {
    TCoeff  absLevel;
    int64_t deltaDist;
  };


  struct Decision
  {
    int64_t rdCost;
    TCoeff  absLevel;
    int     prevId;
  };




  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   P R E - Q U A N T I Z E R                                          =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  class Quantizer
  {
  public:
    Quantizer() {}
    void  init                 ( int dqThrVal ) { m_DqThrVal = dqThrVal; }
    void  dequantBlock         ( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, CoeffBuf& recCoeff, bool enableScalingLists, int* piDequantCoef ) const;
    void  initQuantBlock       ( const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda, int gValue );
    inline void   preQuantCoeff( const TCoeff absCoeff, PQData *pqData, int quanCoeff ) const;
    inline TCoeff getLastThreshold() const { return m_thresLast; }
    inline TCoeff getSSbbThreshold() const { return m_thresSSbb; }

    inline int64_t getQScale()       const { return m_QScale; }
  private:
    // quantization
    int               m_DqThrVal;
    int               m_QShift;
    int64_t           m_QAdd;
    int64_t           m_QScale;
    TCoeff            m_maxQIdx;
    TCoeff            m_thresLast;
    TCoeff            m_thresSSbb;
    // distortion normalization
    int               m_DistShift;
    int64_t           m_DistAdd;
    int64_t           m_DistStepAdd;
    int64_t           m_DistOrgFact;
  };

  void Quantizer::initQuantBlock(const TransformUnit& tu, const ComponentID compID, const QpParam& cQP, const double lambda, int gValue = -1)
  {
    CHECKD( lambda <= 0.0, "Lambda must be greater than 0" );

    const int         qpDQ                  = cQP.Qp(tu.mtsIdx[compID]==MTS_SKIP) + 1;
    const int         qpPer                 = qpDQ / 6;
    const int         qpRem                 = qpDQ - 6 * qpPer;
    const SPS&        sps                   = *tu.cs->sps;
    const CompArea&   area                  = tu.blocks[ compID ];
    const ChannelType chType                = toChannelType( compID );
    const int         channelBitDepth       = sps.bitDepths[ chType ];
    const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange( chType );
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
    const int         maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange( chType );
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

  inline void Quantizer::preQuantCoeff( const TCoeff absCoeff, PQData* pqData, int quanCoeff ) const
  {
    int64_t scaledOrg = int64_t( absCoeff ) * quanCoeff;
    TCoeff  qIdx      = std::max<TCoeff>( 1, std::min<TCoeff>( m_maxQIdx, TCoeff( ( scaledOrg + m_QAdd ) >> m_QShift ) ) );
    int64_t scaledAdd = qIdx * m_DistStepAdd - scaledOrg * m_DistOrgFact;

    PQData& pq_a      = pqData[ ( qIdx + 0 ) & 3 ];
    PQData& pq_b      = pqData[ ( qIdx + 1 ) & 3 ];
    PQData& pq_c      = pqData[ ( qIdx + 2 ) & 3 ];
    PQData& pq_d      = pqData[ ( qIdx + 3 ) & 3 ];

    pq_a.deltaDist    = ( ( scaledAdd + 0 * m_DistStepAdd ) * ( qIdx + 0 ) + m_DistAdd ) >> m_DistShift;
    pq_a.absLevel     = ( qIdx + 1 ) >> 1;

    pq_b.deltaDist    = ( ( scaledAdd + 1 * m_DistStepAdd ) * ( qIdx + 1 ) + m_DistAdd ) >> m_DistShift;
    pq_b.absLevel     = ( qIdx + 2 ) >> 1;

    pq_c.deltaDist    = ( ( scaledAdd + 2 * m_DistStepAdd ) * ( qIdx + 2 ) + m_DistAdd ) >> m_DistShift;
    pq_c.absLevel     = ( qIdx + 3 ) >> 1;

    pq_d.deltaDist    = ( ( scaledAdd + 3 * m_DistStepAdd ) * ( qIdx + 3 ) + m_DistAdd ) >> m_DistShift;
    pq_d.absLevel     = ( qIdx + 4 ) >> 1;
  }







  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q   S T A T E                                                  =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/

  class State;

  struct SbbCtx
  {
    uint8_t*  sbbFlags;
    uint8_t*  levels;
  };

  class CommonCtx
  {
  public:
    CommonCtx() : m_currSbbCtx( m_allSbbCtx ), m_prevSbbCtx( m_currSbbCtx + 4 ) {}

    inline void swap() { std::swap(m_currSbbCtx, m_prevSbbCtx); }

    inline void reset( const TUParameters& tuPars, const RateEstimator &rateEst)
    {
      m_nbInfo = tuPars.m_scanId2NbInfoOut;
      ::memcpy( m_sbbFlagBits, rateEst.sigSbbFracBits(), 2*sizeof(BinFracBits) );
      const int numSbb    = tuPars.m_numSbb;
      const int chunkSize = numSbb + tuPars.m_numCoeff;
      uint8_t*  nextMem   = m_memory;
      for( int k = 0; k < 8; k++, nextMem += chunkSize )
      {
        m_allSbbCtx[k].sbbFlags = nextMem;
        m_allSbbCtx[k].levels   = nextMem + numSbb;
      }
    }

    inline void update(const ScanInfo &scanInfo, const State *prevState, State &currState);

  private:
    const NbInfoOut*            m_nbInfo;
    BinFracBits                 m_sbbFlagBits[2];
    SbbCtx                      m_allSbbCtx  [8];
    SbbCtx*                     m_currSbbCtx;
    SbbCtx*                     m_prevSbbCtx;
    uint8_t                     m_memory[ 8 * ( MAX_TB_SIZEY * MAX_TB_SIZEY + MLS_GRP_NUM ) ];
  };

#define RICEMAX 32
  const int32_t g_goRiceBits[4][RICEMAX] =
  {
    { 32768,  65536,  98304, 131072, 163840, 196608, 262144, 262144, 327680, 327680, 327680, 327680, 393216, 393216, 393216, 393216, 393216, 393216, 393216, 393216, 458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752, 458752},
    { 65536,  65536,  98304,  98304, 131072, 131072, 163840, 163840, 196608, 196608, 229376, 229376, 294912, 294912, 294912, 294912, 360448, 360448, 360448, 360448, 360448, 360448, 360448, 360448, 425984, 425984, 425984, 425984, 425984, 425984, 425984, 425984},
    { 98304,  98304,  98304,  98304, 131072, 131072, 131072, 131072, 163840, 163840, 163840, 163840, 196608, 196608, 196608, 196608, 229376, 229376, 229376, 229376, 262144, 262144, 262144, 262144, 327680, 327680, 327680, 327680, 327680, 327680, 327680, 327680},
    {131072, 131072, 131072, 131072, 131072, 131072, 131072, 131072, 163840, 163840, 163840, 163840, 163840, 163840, 163840, 163840, 196608, 196608, 196608, 196608, 196608, 196608, 196608, 196608, 229376, 229376, 229376, 229376, 229376, 229376, 229376, 229376}
  };

  class State
  {
    friend class CommonCtx;
  public:
    State( const RateEstimator& rateEst, CommonCtx& commonCtx, const int stateId );

    inline void updateState(const ScanInfo &scanInfo, const State *prevStates, const Decision &decision);
    inline void updateStateEOS(const ScanInfo &scanInfo, const State *prevStates, const State *skipStates,
                               const Decision &decision);

    inline void init()
    {
      m_rdCost        = rdCostInit;
      m_numSigSbb     = 0;
      m_remRegBins    = 4;  // just large enough for last scan pos
      m_refSbbCtxId   = -1;
      m_sigFracBits   = m_sigFracBitsArray[ 0 ];
      m_coeffFracBits = m_gtxFracBitsArray[ 0 ];
      m_goRicePar     = 0;
      m_goRiceZero    = 0;
    }

    void checkRdCosts( const ScanPosType spt, const PQData& pqDataA, const PQData& pqDataB, Decision& decisionA, Decision& decisionB ) const
    {
      const int32_t*  goRiceTab = g_goRiceBits[m_goRicePar];
      int64_t         rdCostA   = m_rdCost + pqDataA.deltaDist;
      int64_t         rdCostB   = m_rdCost + pqDataB.deltaDist;
      int64_t         rdCostZ   = m_rdCost;

      if( m_remRegBins >= 4 )
      {
        if( pqDataA.absLevel < 4 )
          rdCostA += m_coeffFracBits.bits[ pqDataA.absLevel ];
        else
        {
          const unsigned value = ( pqDataA.absLevel - 4 ) >> 1;
          rdCostA += m_coeffFracBits.bits[ pqDataA.absLevel - ( value << 1 ) ] + goRiceTab[ std::min<unsigned>( value, RICEMAX - 1 ) ];
        }

        if( pqDataB.absLevel < 4 )
          rdCostB += m_coeffFracBits.bits[ pqDataB.absLevel ];
        else
        {
          const unsigned value = ( pqDataB.absLevel - 4 ) >> 1;
          rdCostB += m_coeffFracBits.bits[ pqDataB.absLevel - ( value << 1 ) ] + goRiceTab[std::min<unsigned>( value, RICEMAX - 1 )];
        }

        if( spt == SCAN_ISCSBB )
        {
          rdCostA += m_sigFracBits.intBits[ 1 ];
          rdCostB += m_sigFracBits.intBits[ 1 ];
          rdCostZ += m_sigFracBits.intBits[ 0 ];
        }
        else if( spt == SCAN_SOCSBB )
        {
          rdCostA += m_sbbFracBits.intBits[ 1 ] + m_sigFracBits.intBits[ 1 ];
          rdCostB += m_sbbFracBits.intBits[ 1 ] + m_sigFracBits.intBits[ 1 ];
          rdCostZ += m_sbbFracBits.intBits[ 1 ] + m_sigFracBits.intBits[ 0 ];
        }
        else if( m_numSigSbb )
        {
          rdCostA += m_sigFracBits.intBits[ 1 ];
          rdCostB += m_sigFracBits.intBits[ 1 ];
          rdCostZ += m_sigFracBits.intBits[ 0 ];
        }
        else
        {
          rdCostZ = decisionA.rdCost;
        }
      }
      else
      {
        rdCostA += ( 1 << SCALE_BITS ) + goRiceTab[ pqDataA.absLevel <= m_goRiceZero ? pqDataA.absLevel - 1 : std::min<int>( pqDataA.absLevel, RICEMAX - 1 ) ];
        rdCostB += ( 1 << SCALE_BITS ) + goRiceTab[ pqDataB.absLevel <= m_goRiceZero ? pqDataB.absLevel - 1 : std::min<int>( pqDataB.absLevel, RICEMAX - 1 ) ];
        rdCostZ += goRiceTab[ m_goRiceZero ];
      }

      if( rdCostA < rdCostZ && rdCostA < decisionA.rdCost )
      {
        decisionA.rdCost    = rdCostA;
        decisionA.absLevel  = pqDataA.absLevel;
        decisionA.prevId    = m_stateId;
      }
      else if( rdCostZ < decisionA.rdCost )
      {
        decisionA.rdCost    = rdCostZ;
        decisionA.absLevel  = 0;
        decisionA.prevId    = m_stateId;
      }

      if( rdCostB < decisionB.rdCost )
      {
        decisionB.rdCost    = rdCostB;
        decisionB.absLevel  = pqDataB.absLevel;
        decisionB.prevId    = m_stateId;
      }
    }

#if 0
    void checkRdCostsZero( const ScanPosType spt, Decision& decisionA ) const
    {
      const int32_t* goRiceTab = g_goRiceBits[m_goRicePar];
      int64_t         rdCostZ  = m_rdCost;

      if( m_remRegBins >= 4 )
      {
        if( spt == SCAN_ISCSBB )
        {
          rdCostZ += m_sigFracBits.intBits[0];
        }
        else if( spt == SCAN_SOCSBB )
        {
          rdCostZ += m_sbbFracBits.intBits[1] + m_sigFracBits.intBits[0];
        }
        else if( m_numSigSbb )
        {
          rdCostZ += m_sigFracBits.intBits[0];
        }
        else
        {
          rdCostZ = decisionA.rdCost;
        }
      }
      else
      {
        rdCostZ += goRiceTab[m_goRiceZero];
      }

      if( rdCostZ < decisionA.rdCost )
      {
        decisionA.rdCost   = rdCostZ;
        decisionA.absLevel = 0;
        decisionA.prevId   = m_stateId;
      }
    }

#endif
    inline void checkRdCostStart(int32_t lastOffset, const PQData &pqData, Decision &decision) const
    {
      int64_t rdCost = pqData.deltaDist + lastOffset;
      if (pqData.absLevel < 4)
      {
        rdCost += m_coeffFracBits.bits[pqData.absLevel];
      }
      else
      {
        const unsigned value = (pqData.absLevel - 4) >> 1;
        rdCost += m_coeffFracBits.bits[pqData.absLevel - (value << 1)] + g_goRiceBits[m_goRicePar][value < RICEMAX ? value : RICEMAX-1];
      }
      if( rdCost < decision.rdCost )
      {
        decision.rdCost   = rdCost;
        decision.absLevel = pqData.absLevel;
        decision.prevId   = -1;
      }
    }

    inline void checkRdCostSkipSbb(Decision &decision) const
    {
      int64_t rdCost = m_rdCost + m_sbbFracBits.intBits[0];
      if( rdCost < decision.rdCost )
      {
        decision.rdCost   = rdCost;
        decision.absLevel = 0;
        decision.prevId   = 4 | m_stateId;
      }
    }

    inline void checkRdCostSkipSbbZeroOut(Decision &decision) const
    {
      int64_t rdCost    = m_rdCost + m_sbbFracBits.intBits[0];
      decision.rdCost   = rdCost;
      decision.absLevel = 0;
      decision.prevId   = 4 | m_stateId;
    }

    struct CtxAcc
    {
      // tplAcc: lower 5 bits are absSum1, upper 3 bits are numPos
      uint8_t tplAcc, sumAbs;
    };

  private:

    int64_t                   m_rdCost;
    union
    {
      uint8_t                 m_state[48];
      struct
      {
        uint8_t               absLevels[16];
        CtxAcc                ctx[16];
      } m_sbb;
    };
    int8_t                    m_numSigSbb;
    int                       m_remRegBins;
    int8_t                    m_refSbbCtxId;
    BinFracBits               m_sbbFracBits;
    BinFracBits               m_sigFracBits;
    CoeffFracBits             m_coeffFracBits;
    int8_t                    m_goRicePar;
    int8_t                    m_goRiceZero;
    const int8_t              m_stateId;
    const BinFracBits*const   m_sigFracBitsArray;
    const CoeffFracBits*const m_gtxFracBitsArray;
    CommonCtx&                m_commonCtx;
  public:
    static const int64_t      rdCostInit = std::numeric_limits<int64_t>::max() >> 1;
    unsigned                  effWidth;
    unsigned                  effHeight;
  };


  State::State( const RateEstimator& rateEst, CommonCtx& commonCtx, const int stateId )
    : m_sbbFracBits     { { 0, 0 } }
    , m_stateId         ( stateId )
    , m_sigFracBitsArray( rateEst.sigFlagBits(stateId) )
    , m_gtxFracBitsArray( rateEst.gtxFracBits(stateId) )
    , m_commonCtx       ( commonCtx )
  {
  }

  inline void State::updateState(const ScanInfo &scanInfo, const State *prevStates, const Decision &decision)
  {
    m_rdCost = decision.rdCost;
    if( decision.prevId > -2 )
    {
      if( decision.prevId >= 0 )
      {
        const State*  prvState  = prevStates            +   decision.prevId;
        m_numSigSbb             = prvState->m_numSigSbb + !!decision.absLevel;
        m_refSbbCtxId           = prvState->m_refSbbCtxId;
        m_sbbFracBits           = prvState->m_sbbFracBits;
        m_remRegBins            = prvState->m_remRegBins - 1;
        if( m_remRegBins >= 4 )
        {
          m_remRegBins -= (decision.absLevel < 2 ? decision.absLevel : 3);
        }
        ::memcpy( m_state, prvState->m_state, sizeof( m_state ) );
      }
      else
      {
        m_numSigSbb     =  1;
        m_refSbbCtxId   = -1;
        int ctxBinSampleRatio = MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT;
        m_remRegBins = (effWidth * effHeight *ctxBinSampleRatio) / 16 - (decision.absLevel < 2 ? decision.absLevel : 3);
        ::memset( m_state, 0, sizeof( m_state ) );
      }

      if( decision.absLevel )
      {
        m_sbb.absLevels[scanInfo.insidePos] = ( uint8_t ) std::min<TCoeff>( 255, decision.absLevel );
        
        if( scanInfo.currNbInfoSbb.numInv )
        {
          int min4_or_5 = std::min<TCoeff>( 4 + ( decision.absLevel & 1 ), decision.absLevel );

          auto adds8 = []( uint8_t a, uint8_t b )
          {
            uint8_t c = a + b;
            if( c < a ) c = -1;
            return c;
          };

          auto update_deps = [&]( int k )
          {
            auto& ctx = m_sbb.ctx[scanInfo.currNbInfoSbb.invInPos[k]];
            ctx.tplAcc += 32 + min4_or_5;
            ctx.sumAbs  = adds8( ctx.sumAbs, decision.absLevel );
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

      if (m_remRegBins >= 4)
      {
        TCoeff  sumAbs  = m_sbb.ctx[scanInfo.nextInsidePos].sumAbs;
        TCoeff  sumAbs1 = m_sbb.ctx[scanInfo.nextInsidePos].tplAcc & 31;
        TCoeff  sumNum  = m_sbb.ctx[scanInfo.nextInsidePos].tplAcc >> 5u;
        int sumGt1 = sumAbs1 - sumNum;
        int sumAll = std::max( std::min( 31, ( int ) sumAbs - 4 * 5 ), 0 );

        m_sigFracBits   = m_sigFracBitsArray  [scanInfo.sigCtxOffsetNext + std::min( (sumAbs1+1)>>1, 3 )];
        m_coeffFracBits = m_gtxFracBitsArray  [scanInfo.gtxCtxOffsetNext + std::min(  sumGt1,        4 )];
        m_goRicePar     = g_auiGoRiceParsCoeff[sumAll];
      }
      else
      {
        TCoeff  sumAbs = m_sbb.ctx[scanInfo.nextInsidePos].sumAbs;
        sumAbs       = std::min<TCoeff>(31, sumAbs);
        m_goRicePar  = g_auiGoRiceParsCoeff[sumAbs];
        m_goRiceZero = g_auiGoRicePosCoeff0(m_stateId, m_goRicePar);
      }
    }
  }

  inline void State::updateStateEOS(const ScanInfo &scanInfo, const State *prevStates, const State *skipStates,
                                    const Decision &decision)
  {
    m_rdCost = decision.rdCost;
    if( decision.prevId > -2 )
    {
      const State* prvState = 0;
      if( decision.prevId  >= 4 )
      {
        CHECK( decision.absLevel != 0, "cannot happen" );
        prvState    = skipStates + ( decision.prevId - 4 );
        m_numSigSbb = 0;
        ::memset( m_sbb.absLevels, 0, sizeof( m_sbb.absLevels ) );
      }
      else if( decision.prevId  >= 0 )
      {
        prvState     = prevStates            +   decision.prevId;
        m_numSigSbb  = prvState->m_numSigSbb + !!decision.absLevel;
        ::memcpy( m_sbb.absLevels, prvState->m_sbb.absLevels, sizeof( m_sbb.absLevels ) );
      }
      else
      {
        m_numSigSbb   = 1;
        ::memset( m_sbb.absLevels, 0, sizeof( m_sbb.absLevels ) );
      }

      m_sbb.absLevels[ scanInfo.insidePos ] = (uint8_t)std::min<TCoeff>( 255, decision.absLevel );

      m_commonCtx.update( scanInfo, prvState, *this );

      if (m_remRegBins >= 4)
      {
        TCoeff  sumAbs  = m_sbb.ctx[scanInfo.nextInsidePos].sumAbs;
        TCoeff  sumAbs1 = m_sbb.ctx[scanInfo.nextInsidePos].tplAcc & 31;
        TCoeff  sumNum  = m_sbb.ctx[scanInfo.nextInsidePos].tplAcc >> 5u;
        int sumGt1 = sumAbs1 - sumNum;
        int sumAll = std::max( std::min( 31, ( int ) sumAbs - 4 * 5 ), 0 );

        m_sigFracBits   = m_sigFracBitsArray  [scanInfo.sigCtxOffsetNext + std::min( (sumAbs1+1)>>1, 3 )];
        m_coeffFracBits = m_gtxFracBitsArray  [scanInfo.gtxCtxOffsetNext + std::min(  sumGt1,        4 )];
        m_goRicePar     = g_auiGoRiceParsCoeff[sumAll];
      }
      else
      {
        TCoeff  sumAbs = m_sbb.ctx[scanInfo.nextInsidePos].sumAbs;
        sumAbs       = std::min<TCoeff>(31, sumAbs);
        m_goRicePar  = g_auiGoRiceParsCoeff[sumAbs];
        m_goRiceZero = g_auiGoRicePosCoeff0(m_stateId, m_goRicePar);
      }
    }
  }

  inline void CommonCtx::update(const ScanInfo &scanInfo, const State *prevState, State &currState)
  {
    uint8_t*    sbbFlags  = m_currSbbCtx[ currState.m_stateId ].sbbFlags;
    uint8_t*    levels    = m_currSbbCtx[ currState.m_stateId ].levels;
    std::size_t setCpSize = m_nbInfo[ scanInfo.scanIdx - 1 ].maxDist * sizeof(uint8_t);
    if( prevState && prevState->m_refSbbCtxId >= 0 )
    {
      ::memcpy( sbbFlags,                  m_prevSbbCtx[prevState->m_refSbbCtxId].sbbFlags,                  scanInfo.numSbb*sizeof(uint8_t) );
      ::memcpy( levels + scanInfo.scanIdx, m_prevSbbCtx[prevState->m_refSbbCtxId].levels + scanInfo.scanIdx, setCpSize );
    }
    else
    {
      ::memset( sbbFlags,                  0, scanInfo.numSbb*sizeof(uint8_t) );
      ::memset( levels + scanInfo.scanIdx, 0, setCpSize );
    }
    sbbFlags[ scanInfo.sbbPos ] = !!currState.m_numSigSbb;
    ::memcpy( levels + scanInfo.scanIdx, currState.m_sbb.absLevels, scanInfo.sbbSize*sizeof(uint8_t) );

    const int       sigNSbb   = ( ( scanInfo.nextSbbRight ? sbbFlags[ scanInfo.nextSbbRight ] : false ) || ( scanInfo.nextSbbBelow ? sbbFlags[ scanInfo.nextSbbBelow ] : false ) ? 1 : 0 );
    currState.m_numSigSbb     = 0;
    if (prevState)
    {
      currState.m_remRegBins = prevState->m_remRegBins;
    }
    else
    {
      int ctxBinSampleRatio = MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT;
      currState.m_remRegBins = (currState.effWidth * currState.effHeight *ctxBinSampleRatio) / 16;
    }
    currState.m_goRicePar     = 0;
    currState.m_refSbbCtxId   = currState.m_stateId;
    currState.m_sbbFracBits   = m_sbbFlagBits[ sigNSbb ];

    ::memset( currState.m_state, 0, sizeof( currState.m_state ) );

    if( sigNSbb || ( ( scanInfo.nextSbbRight && scanInfo.nextSbbBelow ) ? sbbFlags[ scanInfo.nextSbbBelow  + 1 ] : false ) )
    {
      const int         scanBeg   = scanInfo.scanIdx - scanInfo.sbbSize;
      const NbInfoOut*  nbOut     = m_nbInfo + scanBeg;
      const uint8_t*    absLevels = levels   + scanBeg;

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
            UPDATE(4);
          case 4:
            UPDATE(3);
          case 3:
            UPDATE(2);
          case 2:
            UPDATE(1);
          case 1:
            UPDATE(0);
          }
  #undef UPDATE
          currState.m_sbb.ctx[id].tplAcc = ( sumNum << 5 ) | sumAbs1;
          currState.m_sbb.ctx[id].sumAbs = ( uint8_t ) std::min( 127, sumAbs );
        }
      }
    }
  }



  /*================================================================================*/
  /*=====                                                                      =====*/
  /*=====   T C Q                                                              =====*/
  /*=====                                                                      =====*/
  /*================================================================================*/
  class DepQuant : private RateEstimator
  {
  public:
    DepQuant( bool enc );

    void    quant   ( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP, const double lambda, const Ctx& ctx, TCoeff& absSum, bool enableScalingLists, int* quantCoeff );
    void    dequant ( const TransformUnit& tu, CoeffBuf& recCoeff, const ComponentID compID, const QpParam& cQP, bool enableScalingLists, int* quantCoeff );
    void    init    ( int dqTrVal );

  private:
    void    xDecideAndUpdate  ( const TCoeff absCoeff, const ScanInfo& scanInfo, bool zeroOut, int quantCoeff);
    void    xDecide           ( const ScanPosType spt, const TCoeff absCoeff, const int lastOffset, Decision* decisions, bool zeroOut, int quantCoeff );

  private:
    CommonCtx   m_commonCtx;
    State       m_allStates[ 12 ];
    State*      m_currStates;
    State*      m_prevStates;
    State*      m_skipStates;
    State       m_startState;
    Quantizer   m_quant;
    Decision    m_trellis[ MAX_TB_SIZEY * MAX_TB_SIZEY ][ 8 ];
    Rom         m_scansRom;
  };
  

#define DINIT(l,p) {std::numeric_limits<int64_t>::max()>>2,l,p}
  static const Decision startDec[8] = {DINIT(-1,-2),DINIT(-1,-2),DINIT(-1,-2),DINIT(-1,-2),DINIT(0,4),DINIT(0,5),DINIT(0,6),DINIT(0,7)};
#undef  DINIT

#define TINIT(x) {*this,m_commonCtx,x}
  DepQuant::DepQuant( bool enc )
    : RateEstimator ()
    , m_commonCtx   ()
    , m_allStates   {TINIT(0),TINIT(1),TINIT(2),TINIT(3),TINIT(0),TINIT(1),TINIT(2),TINIT(3),TINIT(0),TINIT(1),TINIT(2),TINIT(3)}
    , m_currStates  (  m_allStates      )
    , m_prevStates  (  m_currStates + 4 )
    , m_skipStates  (  m_prevStates + 4 )
    , m_startState  TINIT(0)
  {
    if( enc )
    {
      m_scansRom.init();

      for( int t = 0; t < ( MAX_TB_SIZEY * MAX_TB_SIZEY ); t++ )
      {
        memcpy( m_trellis[t] + 4, startDec + 4, 4 * sizeof( Decision ) );
      }
    }
  }
#undef TINIT


  void DepQuant::dequant( const TransformUnit& tu,  CoeffBuf& recCoeff, const ComponentID compID, const QpParam& cQP, bool enableScalingLists, int* piDequantCoef )
  {
    m_quant.dequantBlock( tu, compID, cQP, recCoeff, enableScalingLists, piDequantCoef );
  }

  void DepQuant::init( int dqTrVal )
  {
    m_quant.init( dqTrVal );
  }

  void DepQuant::xDecide( const ScanPosType spt, const TCoeff absCoeff, const int lastOffset, Decision* decisions, bool zeroOut, int quanCoeff )
  {
    ::memcpy( decisions, startDec, 4*sizeof(Decision) );

    if( zeroOut )
    {
      if( spt==SCAN_EOCSBB )
      {
        m_skipStates[0].checkRdCostSkipSbbZeroOut( decisions[0] );
        m_skipStates[1].checkRdCostSkipSbbZeroOut( decisions[1] );
        m_skipStates[2].checkRdCostSkipSbbZeroOut( decisions[2] );
        m_skipStates[3].checkRdCostSkipSbbZeroOut( decisions[3] );
      }
      return;
    }

    PQData  pqData[4];
    m_quant.preQuantCoeff( absCoeff, pqData, quanCoeff );

    m_prevStates[0].checkRdCosts( spt, pqData[0], pqData[2], decisions[0], decisions[2] );
    m_prevStates[1].checkRdCosts( spt, pqData[0], pqData[2], decisions[2], decisions[0] );
    m_prevStates[2].checkRdCosts( spt, pqData[3], pqData[1], decisions[1], decisions[3] );
    m_prevStates[3].checkRdCosts( spt, pqData[3], pqData[1], decisions[3], decisions[1] );

    if( spt==SCAN_EOCSBB )
    {
        m_skipStates[0].checkRdCostSkipSbb( decisions[0] );
        m_skipStates[1].checkRdCostSkipSbb( decisions[1] );
        m_skipStates[2].checkRdCostSkipSbb( decisions[2] );
        m_skipStates[3].checkRdCostSkipSbb( decisions[3] );
    }

    m_startState.checkRdCostStart( lastOffset, pqData[0], decisions[0] );
    m_startState.checkRdCostStart( lastOffset, pqData[2], decisions[2] );
  }

  void DepQuant::xDecideAndUpdate( const TCoeff absCoeff, const ScanInfo& scanInfo, bool zeroOut, int quantCoeff )
  {
    Decision* decisions = m_trellis[ scanInfo.scanIdx ];

    std::swap( m_prevStates, m_currStates );

    xDecide( scanInfo.spt, absCoeff, lastOffset(scanInfo.scanIdx), decisions, zeroOut, quantCoeff );

    if( scanInfo.scanIdx )
    {
      if( scanInfo.insidePos == 0 )
      {
        m_commonCtx.swap();
        m_currStates[0].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[0] );
        m_currStates[1].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[1] );
        m_currStates[2].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[2] );
        m_currStates[3].updateStateEOS( scanInfo, m_prevStates, m_skipStates, decisions[3] );
        ::memcpy( decisions+4, decisions, 4*sizeof(Decision) );
      }
      else if( !zeroOut )
      {
        m_currStates[0].updateState( scanInfo, m_prevStates, decisions[0] );
        m_currStates[1].updateState( scanInfo, m_prevStates, decisions[1] );
        m_currStates[2].updateState( scanInfo, m_prevStates, decisions[2] );
        m_currStates[3].updateState( scanInfo, m_prevStates, decisions[3] );
      }

      if( scanInfo.spt == SCAN_SOCSBB )
      {
        std::swap( m_prevStates, m_skipStates );
      }
    }
  }

  void DepQuant::quant( TransformUnit& tu, const CCoeffBuf& srcCoeff, const ComponentID compID, const QpParam& cQP, const double lambda, const Ctx& ctx, TCoeff& absSum, bool enableScalingLists, int* quantCoeff )
  {
    //===== reset / pre-init =====
    const TUParameters& tuPars  = *m_scansRom.getTUPars( tu.blocks[compID], compID );
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

#if ENABLE_SIMD_OPT_QUANT && defined( TARGET_SIMD_X86 )
      // if more than one 4x4 coding subblock is available, use SIMD to find first subblock with coefficient larger than threshold
      if( firstTestPos >= 16 && tuPars.m_log2SbbWidth == 2 && tuPars.m_log2SbbHeight == 2 && read_x86_extension_flags() > SCALAR )
      {
        const int sbbSize = tuPars.m_sbbSize;
        // move the pointer to the beginning of the current subblock
        firstTestPos -= ( sbbSize - 1 );

        const __m128i xdfTh = _mm_set1_epi32( defaultTh );

        // for each subblock
        for( ; firstTestPos >= 0; firstTestPos -= sbbSize )
        {
          // skip zeroed out blocks
          // for 64-point transformation the coding order takes care of that
          if( zeroOutforThres && ( tuPars.m_scanId2BlkPos[firstTestPos].x >= zeroOutWidth || tuPars.m_scanId2BlkPos[firstTestPos].y >= zeroOutHeight ) )
          {
            continue;
          }

          // read first line of the subblock and check for coefficients larger than the threshold
          // assumming the subblocks are dense 4x4 blocks in raster scan order with the stride of tuPars.m_width
          int pos = tuPars.m_scanId2BlkPos[firstTestPos].idx;
          __m128i xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &tCoeff[pos] ) );
          __m128i xdf = _mm_cmpgt_epi32( xl0, xdfTh );

          // same for the next line in the subblock
          pos += tuPars.m_width;
          xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &tCoeff[pos] ) );
          xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

          // and the third line
          pos += tuPars.m_width;
          xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &tCoeff[pos] ) );
          xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

          // and the last line
          pos += tuPars.m_width;
          xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &tCoeff[pos] ) );
          xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

          // if any of the 16 comparisons were true, break, because this subblock contains a coefficient larger than threshold
          if( !_mm_testz_si128( xdf, xdf ) ) break;
        }

        if( firstTestPos >= 0 )
        {
          // if a coefficient was found, advance the pointer to the end of the current subblock
          // for the subsequent coefficient-wise refinement (C-impl after endif)
          firstTestPos += sbbSize - 1;
        }
      }

#endif
      for( ; firstTestPos >= 0; firstTestPos-- )
      {
        if( zeroOutforThres && ( tuPars.m_scanId2BlkPos[firstTestPos].x >= zeroOutWidth || tuPars.m_scanId2BlkPos[firstTestPos].y >= zeroOutHeight ) ) continue;
        if( abs( tCoeff[tuPars.m_scanId2BlkPos[firstTestPos].idx] ) > defaultTh ) break;
      }
    }

    if( firstTestPos < 0 )
    {
      tu.lastPos[compID] = -1;
      return;
    }

    //===== real init =====
    RateEstimator::initCtx( tuPars, tu, compID, ctx.getFracBitsAcess() );
    m_commonCtx.reset( tuPars, *this );
    for( int k = 0; k < 12; k++ )
    {
      m_allStates[k].init();
    }
    m_startState.init();
    
    int effectWidth  = std::min( 32, effWidth );
    int effectHeight = std::min( 32, effHeight );
    for (int k = 0; k < 12; k++)
    {
      m_allStates[k].effWidth  = effectWidth;
      m_allStates[k].effHeight = effectHeight;
    }
    m_startState.effWidth  = effectWidth;
    m_startState.effHeight = effectHeight;

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
    Decision  decision    = { std::numeric_limits<int64_t>::max(), -1, -2 };
    int64_t   minPathCost =  0;
    for( int8_t stateId = 0; stateId < 4; stateId++ )
    {
      int64_t pathCost = m_trellis[0][stateId].rdCost;
      if( pathCost < minPathCost )
      {
        decision.prevId = stateId;
        minPathCost     = pathCost;
      }
    }

    //===== backward scanning =====
    int scanIdx = 0;
    for( ; decision.prevId >= 0; scanIdx++ )
    {
      decision          = m_trellis[ scanIdx ][ decision.prevId ];
      int32_t blkpos    = tuPars.m_scanId2BlkPos[scanIdx].idx;
      qCoeff[ blkpos ]  = TCoeffSig( tCoeff[ blkpos ] < 0 ? -decision.absLevel : decision.absLevel );
      absSum           += decision.absLevel;
    }

    tu.lastPos[compID] = scanIdx - 1;
  }

}; // namespace DQIntern




//===== interface class =====
DepQuant::DepQuant( const Quant* other, bool enc, bool useScalingLists ) : QuantRDOQ2( other, useScalingLists )
{
  const DepQuant* dq = dynamic_cast<const DepQuant*>( other );
  CHECK( other && !dq, "The DepQuant cast must be successfull!" );
  p = new DQIntern::DepQuant( enc );
}

DepQuant::~DepQuant()
{
  delete static_cast<DQIntern::DepQuant*>(p);
}

void DepQuant::quant( TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx& ctx )
{
  if( tu.cs->slice->depQuantEnabled && (tu.mtsIdx[compID] != MTS_SKIP) )
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
    static_cast<DQIntern::DepQuant*>(p)->quant( tu, pSrc, compID, cQP, Quant::m_dLambda, ctx, uiAbsSum, enableScalingLists, Quant::getQuantCoeff(scalingListType, qpRem, log2TrWidth, log2TrHeight) );
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
    static_cast<DQIntern::DepQuant*>(p)->dequant( tu, dstCoeff, compID, cQP, enableScalingLists, Quant::getDequantCoeff(scalingListType, qpRem, log2TrWidth, log2TrHeight) );
  }
  else
  {
    QuantRDOQ::dequant( tu, dstCoeff, compID, cQP );
  }
}

void DepQuant::init( int rdoq, bool useRDOQTS, bool useSelectiveRDOQ, int thrVal )
{
  QuantRDOQ2::init( rdoq, useRDOQTS, useSelectiveRDOQ, thrVal );

  static_cast<DQIntern::DepQuant*>(p)->init( thrVal );
}

} // namespace vvenc

//! \}


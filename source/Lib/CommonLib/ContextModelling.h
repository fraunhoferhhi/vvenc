/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     ContextModelling.h
 *  \brief    Classes providing probability descriptions and contexts (header)
 */

#pragma once

#include "CommonDef.h"
#include "Contexts.h"
#include "Slice.h"
#include "Unit.h"
#include "UnitPartitioner.h"

#include <bitset>

//! \ingroup CommonLib
//! \{

namespace vvenc
{

struct CtxTpl
{
  // lower 5 bits are absSum1, upper 3 bits are numPos
  uint8_t ctxTpl;
};

struct CoeffCodingContext
{
public:
  CoeffCodingContext( const TransformUnit& tu, ComponentID component, bool signHide, bool bdpcm = false, CtxTpl* tplBuf = nullptr );
public:
  void  initSubblock    ( int SubsetId, bool sigGroupFlag = false );
public:
  void  resetSigGroup   ()                      { m_sigCoeffGroupFlag.reset( m_subSetPos ); }
  void  setSigGroup     ()                      { m_sigCoeffGroupFlag.set( m_subSetPos ); }
  bool  noneSigGroup    ()                      { return m_sigCoeffGroupFlag.none(); }
  int   lastSubSet      ()                      { return ( maxNumCoeff() - 1 ) >> log2CGSize(); }
  bool  isLastSubSet    ()                      { return lastSubSet() == m_subSetId; }
  bool  only1stSigGroup ()                      { return m_sigCoeffGroupFlag.count()-m_sigCoeffGroupFlag[lastSubSet()]==0; }
  void  setScanPosLast  ( int       posLast )   { m_scanPosLast = posLast; }
public:
  ComponentID     compID          ()                        const { return m_compID; }
  int             subSetId        ()                        const { return m_subSetId; }
  int             subSetPos       ()                        const { return m_subSetPos; }
  int             cgPosY          ()                        const { return m_subSetPosY; }
  int             cgPosX          ()                        const { return m_subSetPosX; }
  unsigned        width           ()                        const { return m_width; }
  unsigned        height          ()                        const { return m_height; }
  unsigned        log2CGWidth     ()                        const { return m_log2CGWidth; }
  unsigned        log2CGHeight    ()                        const { return m_log2CGHeight; }
  unsigned        log2CGSize      ()                        const { return m_log2CGSize; }
  int             maxLog2TrDRange ()                        const { return m_maxLog2TrDynamicRange; }
  unsigned        maxNumCoeff     ()                        const { return m_maxNumCoeff; }
  int             scanPosLast     ()                        const { return m_scanPosLast; }
  int             minSubPos       ()                        const { return m_minSubPos; }
  int             maxSubPos       ()                        const { return m_maxSubPos; }
  bool            isLast          ()                        const { return ( ( m_scanPosLast >> m_log2CGSize ) == m_subSetId ); }
  bool            isNotFirst      ()                        const { return ( m_subSetId != 0 ); }
  bool            isSigGroup      ( int scanPosCG )         const { return m_sigCoeffGroupFlag[m_scanCG[scanPosCG].idx]; }
  bool            isSigGroup      ()                        const { return m_sigCoeffGroupFlag[ m_subSetPos ]; }
  bool            signHiding      ()                        const { return m_signHiding; }
  bool            hideSign        ( int       posFirst,
                                    int       posLast   )   const { return ( m_signHiding && ( posLast - posFirst >= SBH_THRESHOLD ) ); }
  unsigned        blockPos        ( int scanPos )           const { return m_scan[scanPos].idx; }
  unsigned        posX            ( int scanPos )           const { return m_scan[scanPos].x; }
  unsigned        posY            ( int scanPos )           const { return m_scan[scanPos].y; }
  unsigned        maxLastPosX     ()                        const { return m_maxLastPosX; }
  unsigned        maxLastPosY     ()                        const { return m_maxLastPosY; }
  unsigned        lastXCtxId      ( unsigned  posLastX  )   const { return m_CtxSetLastX( m_lastOffsetX + ( posLastX >> m_lastShiftX ) ); }
  unsigned        lastYCtxId      ( unsigned  posLastY  )   const { return m_CtxSetLastY( m_lastOffsetY + ( posLastY >> m_lastShiftY ) ); }
  int             numCtxBins      ()                        const { return   m_remainingContextBins;      }
  void            setNumCtxBins   ( int n )                       {          m_remainingContextBins  = n; }
  unsigned        sigGroupCtxId   ( bool ts = false     )   const { return ts ? m_sigGroupCtxIdTS : m_sigGroupCtxId; }
  bool            bdpcm           ()                        const { return m_bdpcm; }

  void            decimateNumCtxBins(int n) { m_remainingContextBins -= n; }
  void            increaseNumCtxBins(int n) { m_remainingContextBins += n; }

  unsigned sigCtxIdAbs( int scanPos, const TCoeffSig* coeff, const int state )
  {
    const uint32_t   posY      = m_scan[scanPos].y;
    const uint32_t   posX      = m_scan[scanPos].x;
    const TCoeffSig* pData     = coeff + posX + posY * m_width;
    const int        diag      = posX + posY;
    int              numPos    = 0;
    int              sumAbs    = 0;
#define UPDATE(x) {int a=abs(x);sumAbs+=std::min(4+(a&1),a);numPos+=!!a;}
    if( posX < m_width-1 )
    {
      UPDATE( pData[1] );
      if( posX < m_width-2 )
      {
        UPDATE( pData[2] );
      }
      if( posY < m_height-1 )
      {
        UPDATE( pData[m_width+1] );
      }
    }
    if( posY < m_height-1 )
    {
      UPDATE( pData[m_width] );
      if( posY < m_height-2 )
      {
        UPDATE( pData[m_width<<1] );
      }
    }
#undef UPDATE

    int ctxOfs = std::min((sumAbs+1)>>1, 3) + ( diag < 2 ? 4 : 0 );

    if( m_chType == CH_L )
    {
      ctxOfs += diag < 5 ? 4 : 0;
    }

    m_tmplCpDiag = diag;
    m_tmplCpSum1 = sumAbs - numPos;
    return m_sigFlagCtxSet[std::max( 0, state-1 )]( ctxOfs );
  }

  unsigned sigCtxIdAbsWithAcc( const int scanPos, const int state )
  {
    const auto scanEl       = m_scan[scanPos];
    const uint32_t posY     = scanEl.y;
    const uint32_t posX     = scanEl.x;
    const int32_t blkPos    = scanEl.idx;
    const int      diag     = posX + posY;
    const int      tplVal   = m_tplBuf[-blkPos].ctxTpl;
    const int      numPos   = tplVal >> 5u;
    const int      sumAbs   = tplVal & 31;

    int ctxOfs = std::min( ( sumAbs + 1 ) >> 1, 3 ) + ( diag < 2 ? 4 : 0 );

    if( isLuma( m_chType ) )
    {
      ctxOfs += diag < 5 ? 4 : 0;
    }
    m_tmplCpDiag = diag;
    m_tmplCpSum1 = sumAbs - numPos;
    return m_sigFlagCtxSet[std::max( 0, state-1 )]( ctxOfs );
  }

  void absVal1stPass( const int scanPos, const TCoeffSig absLevel1 )
  {
    CHECKD( !absLevel1, "Shound not be called if '0'!" );

    const auto scanEl     = m_scan[scanPos];
    const uint32_t posY   = scanEl.y;
    const uint32_t posX   = scanEl.x;
    const int32_t  blkPos = scanEl.idx;

    auto update_deps = [&]( int offset )
    {
      auto& ctx   = m_tplBuf[-blkPos + offset];
      ctx.ctxTpl += uint8_t( 32 + absLevel1 );
    };

    if( posY > 1 ) update_deps( 2 * m_width );
    if( posY > 0
     && posX > 0 ) update_deps( m_width + 1 );
    if( posY > 0 ) update_deps( m_width );
    if( posX > 1 ) update_deps( 2 );
    if( posX > 0 ) update_deps( 1 );
  }
  

  void remAbsVal1stPass( const int scanPos, const TCoeffSig absLevel1 )
  {
    CHECKD( !absLevel1, "Shound not be called if '0'!" );

    const auto scanEl     = m_scan[scanPos];
    const uint32_t posY   = scanEl.y;
    const uint32_t posX   = scanEl.x;
    const int32_t  blkPos = scanEl.idx;

    auto update_deps = [&]( int offset )
    {
      auto& ctx   = m_tplBuf[-blkPos + offset];
      ctx.ctxTpl -= uint8_t( 32 + absLevel1 );
    };

    if( posY > 1 ) update_deps( 2 * m_width );
    if( posY > 0
     && posX > 0 ) update_deps( m_width + 1 );
    if( posY > 0 ) update_deps( m_width );
    if( posX > 1 ) update_deps( 2 );
    if( posX > 0 ) update_deps( 1 );
  }

  uint8_t ctxOffsetAbs()
  {
    int offset = 0;
    if( m_tmplCpDiag != -1 )
    {
      offset  = std::min( m_tmplCpSum1, 4 ) + 1;
      offset += ( !m_tmplCpDiag ? ( m_chType == CH_L ? 15 : 5 ) : m_chType == CH_L ? m_tmplCpDiag < 3 ? 10 : ( m_tmplCpDiag < 10 ? 5 : 0 ) : 0 );
    }
    return uint8_t(offset);
  }

  unsigned parityCtxIdAbs   ( uint8_t offset )  const { return m_parFlagCtxSet   ( offset ); }
  unsigned greater1CtxIdAbs ( uint8_t offset )  const { return m_gtxFlagCtxSet[1]( offset ); }
  unsigned greater2CtxIdAbs ( uint8_t offset )  const { return m_gtxFlagCtxSet[0]( offset ); }

  unsigned templateAbsSum( int scanPos, const TCoeffSig* coeff, int baseLevel )
  {
    const uint32_t   posY = m_scan[scanPos].y;
    const uint32_t   posX = m_scan[scanPos].x;
    const TCoeffSig* pData = coeff + posX + posY * m_width;
    int              sum   = 0;
    if (posX < m_width - 1)
    {
      sum += abs(pData[1]);
      if (posX < m_width - 2)
      {
        sum += abs(pData[2]);
      }
      if (posY < m_height - 1)
      {
        sum += abs(pData[m_width + 1]);
      }
    }
    if (posY < m_height - 1)
    {
      sum += abs(pData[m_width]);
      if (posY < m_height - 2)
      {
        sum += abs(pData[m_width << 1]);
      }
    }
    return std::max(std::min(sum - 5 * baseLevel, 31), 0);
  }

  unsigned sigCtxIdAbsTS( int scanPos, const TCoeffSig* coeff )
  {
    const uint32_t   posY   = m_scan[scanPos].y;
    const uint32_t   posX   = m_scan[scanPos].x;
    const TCoeffSig* posC   = coeff + posX + posY * m_width;
    int              numPos = 0;
#define UPDATE(x) {int a=abs(x);numPos+=!!a;}
    if( posX > 0 )
    {
      UPDATE( posC[-1] );
    }
    if( posY > 0 )
    {
      UPDATE( posC[-(int)m_width] );
    }
#undef UPDATE

    return m_tsSigFlagCtxSet( numPos );
  }

  unsigned parityCtxIdAbsTS   ()                  const { return m_tsParFlagCtxSet(      0 ); }
  unsigned greaterXCtxIdAbsTS ( uint8_t offset )  const { return m_tsGtxFlagCtxSet( offset ); }

  unsigned lrg1CtxIdAbsTS(int scanPos, const TCoeffSig* coeff, int bdpcm)
  {
    const uint32_t   posY = m_scan[scanPos].y;
    const uint32_t   posX = m_scan[scanPos].x;
    const TCoeffSig* posC = coeff + posX + posY * m_width;

    int             numPos = 0;
#define UPDATE(x) {int a=abs(x);numPos+=!!a;}

    if (bdpcm)
    {
      numPos = 3;
    }
    else
    {
      if (posX > 0)
      {
        UPDATE(posC[-1]);
      }
      if (posY > 0)
      {
        UPDATE(posC[-(int)m_width]);
      }
    }

#undef UPDATE
    return m_tsLrg1FlagCtxSet(numPos);
  }

  unsigned signCtxIdAbsTS(int scanPos, const TCoeffSig* coeff, int bdpcm)
  {
    const uint32_t   posY  = m_scan[scanPos].y;
    const uint32_t   posX  = m_scan[scanPos].x;
    const TCoeffSig* pData = coeff + posX + posY * m_width;

    int rightSign = 0, belowSign = 0;
    unsigned signCtx = 0;

    if (posX > 0)
    {
      rightSign = pData[-1];
    }
    if (posY > 0)
    {
      belowSign = pData[-(int)m_width];
    }

    if ((rightSign == 0 && belowSign == 0) || ((rightSign*belowSign) < 0))
    {
      signCtx = 0;
    }
    else if (rightSign >= 0 && belowSign >= 0)
    {
      signCtx = 1;
    }
    else
    {
      signCtx = 2;
    }
    if (bdpcm)
    {
      signCtx += 3;
    }
    return m_tsSignFlagCtxSet(signCtx);
  }

  void neighTS(int& rightPixel, int& belowPixel, int scanPos, const TCoeffSig* coeff)
  {
    const uint32_t   posY = m_scan[scanPos].y;
    const uint32_t   posX = m_scan[scanPos].x;
    const TCoeffSig* data = coeff + posX + posY * m_width;

    rightPixel = belowPixel = 0;

    if (posX > 0)
    {
      rightPixel = data[-1];
    }
    if (posY > 0)
    {
      belowPixel = data[-(int)m_width];
    }
  }

  int deriveModCoeff(int rightPixel, int belowPixel, int absCoeff, int bdpcm = 0)
  {
    if (absCoeff == 0)
      return 0;

    int pred1, absBelow = abs(belowPixel), absRight = abs(rightPixel);

    int absCoeffMod = absCoeff;

    if (bdpcm == 0)
    {
      pred1 = std::max(absBelow, absRight);

      if (absCoeff == pred1)
      {
        absCoeffMod = 1;
      }
      else
      {
        absCoeffMod = absCoeff < pred1 ? absCoeff + 1 : absCoeff;
      }
    }

    return(absCoeffMod);
  }

  int decDeriveModCoeff(int rightPixel, int belowPixel, int absCoeff)
  {
    if (absCoeff == 0)
      return 0;

    int pred1, absBelow = abs(belowPixel), absRight = abs(rightPixel);
    pred1 = std::max(absBelow, absRight);

    int absCoeffMod;

    if (absCoeff == 1 && pred1 > 0)
    {
      absCoeffMod = pred1;
    }
    else
    {
      absCoeffMod = absCoeff - (absCoeff <= pred1);
    }
    return(absCoeffMod);
  }

  unsigned templateAbsSumTS( int scanPos, const TCoeffSig* coeff )
  {
    return 1;
  }

  int                       regBinLimit;

private:
  // constant
  const ComponentID         m_compID;
  const ChannelType         m_chType;
  const unsigned            m_width;
  const unsigned            m_height;
  const unsigned            m_log2CGWidth;
  const unsigned            m_log2CGHeight;
  const unsigned            m_log2CGSize;
  const unsigned            m_widthInGroups;
  const unsigned            m_heightInGroups;
  const unsigned            m_log2WidthInGroups;
  const unsigned            m_log2BlockWidth;
  const unsigned            m_log2BlockHeight;
  const unsigned            m_maxNumCoeff;
  const bool                m_signHiding;
  const int                 m_maxLog2TrDynamicRange;
  const ScanElement *       m_scan;
  const ScanElement *       m_scanCG;
  const CtxSet              m_CtxSetLastX;
  const CtxSet              m_CtxSetLastY;
  const unsigned            m_maxLastPosX;
  const unsigned            m_maxLastPosY;
  const int                 m_lastOffsetX;
  const int                 m_lastOffsetY;
  const int                 m_lastShiftX;
  const int                 m_lastShiftY;
  // modified
  int                       m_scanPosLast;
  int                       m_subSetId;
  int                       m_subSetPos;
  int                       m_subSetPosX;
  int                       m_subSetPosY;
  int                       m_minSubPos;
  int                       m_maxSubPos;
  unsigned                  m_sigGroupCtxId;
  int                       m_tmplCpSum1;
  int                       m_tmplCpDiag;
  CtxSet                    m_sigFlagCtxSet[3];
  CtxSet                    m_parFlagCtxSet;
  CtxSet                    m_gtxFlagCtxSet[2];
  unsigned                  m_sigGroupCtxIdTS;
  CtxSet                    m_tsSigFlagCtxSet;
  CtxSet                    m_tsParFlagCtxSet;
  CtxSet                    m_tsGtxFlagCtxSet;
  CtxSet                    m_tsLrg1FlagCtxSet;
  CtxSet                    m_tsSignFlagCtxSet;
  int                       m_remainingContextBins;
  std::bitset<MLS_GRP_NUM>  m_sigCoeffGroupFlag;
  const bool                m_bdpcm;
  CtxTpl*                   m_tplBuf;
};


struct CUCtx
{
  CUCtx()              : isDQPCoded(false), isChromaQpAdjCoded(false),
                         qgStart(false)
                         {
                           violatesLfnstConstrained[CH_L] = false;
                           violatesLfnstConstrained[CH_C] = false;
                           violatesMtsCoeffConstraint      = false;
                           mtsLastScanPos                  = false;
                           lfnstLastScanPos                = false;
                         }
  CUCtx(int _qp)       : isDQPCoded(false), isChromaQpAdjCoded(false),
                         qgStart(false),
                         qp(_qp)
                         {
                           violatesLfnstConstrained[CH_L] = false;
                           violatesLfnstConstrained[CH_C] = false;
                           violatesMtsCoeffConstraint      = false;
                           mtsLastScanPos                  = false;
                           lfnstLastScanPos                = false;
                         }

  bool      isDQPCoded;
  bool      isChromaQpAdjCoded;
  bool      qgStart;
  bool      lfnstLastScanPos;
  int8_t    qp;                   // used as a previous(last) QP and for QP prediction
  bool      violatesLfnstConstrained[MAX_NUM_CH];
  bool      violatesMtsCoeffConstraint;
  bool      mtsLastScanPos;
};

struct MergeCtx
{
  MergeCtx() : numValidMergeCand( 0 ), hasMergedCandList( false ) { for( unsigned i = 0; i < MRG_MAX_NUM_CANDS; i++ ) mrgTypeNeighbours[i] = MRG_TYPE_DEFAULT_N; }

  void          setMmvdMergeCandiInfo ( CodingUnit& cu, const MmvdIdx candIdx) const;
  void          setMergeInfo          ( CodingUnit& cu, const int     candIdx) const;
  void          getMmvdDeltaMv        ( const Slice& slice, const MmvdIdx candIdx, Mv deltaMv[NUM_REF_PIC_LIST_01] ) const;

  MvField       mvFieldNeighbours [MRG_MAX_NUM_CANDS][NUM_REF_PIC_LIST_01];
  uint8_t       BcwIdx            [MRG_MAX_NUM_CANDS];
  unsigned char interDirNeighbours[MRG_MAX_NUM_CANDS];
  MergeType     mrgTypeNeighbours [MRG_MAX_NUM_CANDS];
  int           numValidMergeCand;
  bool          hasMergedCandList;

  MvField       mmvdBaseMv        [MMVD_BASE_MV_NUM][NUM_REF_PIC_LIST_01];
  bool          mmvdUseAltHpelIf  [MMVD_BASE_MV_NUM];
  bool          useAltHpelIf      [MRG_MAX_NUM_CANDS];
};

struct AffineMergeCtx
{
  AffineMergeCtx() : numValidMergeCand( 0 ) { for ( unsigned i = 0; i < AFFINE_MRG_MAX_NUM_CANDS; i++ ) affineType[i] = AFFINEMODEL_4PARAM; }

  MvField       mvFieldNeighbours [AFFINE_MRG_MAX_NUM_CANDS][NUM_REF_PIC_LIST_01][3];
  unsigned char interDirNeighbours[AFFINE_MRG_MAX_NUM_CANDS];
  EAffineModel  affineType        [AFFINE_MRG_MAX_NUM_CANDS];
  uint8_t       BcwIdx            [AFFINE_MRG_MAX_NUM_CANDS];
  int           numValidMergeCand;
  int           maxNumMergeCand;

  MergeType     mergeType[AFFINE_MRG_MAX_NUM_CANDS];
  MotionBuf     subPuMvpMiBuf;
};


struct DeriveCtx
{
  const CodingUnit* cuRestrictedLeft[MAX_NUM_CH];
  const CodingUnit* cuRestrictedAbove[MAX_NUM_CH];

  void determineNeighborCus       ( const CodingStructure& cs, const UnitArea& ua, const ChannelType ch, const TreeType treeType );

  inline static unsigned CtxQtCbf ( const ComponentID compID, const bool prevCbf = false, const int ispIdx = 0 )   { return ( ispIdx && isLuma( compID ) ) ? (2 + (int)prevCbf) : (((compID == COMP_Cr) && prevCbf) ? 1 : 0); }
  void     CtxSplit               ( const Partitioner& partitioner, unsigned& ctxSpl, unsigned& ctxQt, unsigned& ctxHv, unsigned& ctxHorBt, unsigned& ctxVerBt, const bool canSplit[6] ) const;
  unsigned CtxMipFlag             ( const CodingUnit& cu ) const;
  unsigned CtxInterDir            ( const CodingUnit& cu ) const { return ( 7 - ((Log2(cu.lumaSize().area()) + 1) >> 1) ); }

  unsigned CtxModeConsFlag() const
  {
    const CodingUnit* cuLeft  = cuRestrictedLeft[CH_L]; const CodingUnit* cuAbove = cuRestrictedAbove[CH_L];
    return ((cuAbove && cuAbove->predMode == MODE_INTRA) || (cuLeft && cuLeft->predMode == MODE_INTRA)) ? 1 : 0;
  }

  unsigned CtxAffineFlag() const
  {
    const CodingUnit *cuLeft = cuRestrictedLeft[CH_L]; const CodingUnit *cuAbove = cuRestrictedAbove[CH_L];
    return (( cuLeft && cuLeft->affine ) ? 1 : 0) + (( cuAbove && cuAbove->affine ) ? 1 : 0);
  }

  unsigned CtxSkipFlag() const
  {
    const CodingUnit *cuLeft = cuRestrictedLeft[CH_L]; const CodingUnit *cuAbove = cuRestrictedAbove[CH_L];
    return (( cuLeft && cuLeft->skip ) ? 1 : 0) + (( cuAbove && cuAbove->skip ) ? 1 : 0);
  }

  unsigned CtxPredModeFlag() const
  {
    const CodingUnit *cuLeft = cuRestrictedLeft[CH_L]; const CodingUnit *cuAbove = cuRestrictedAbove[CH_L];
    return ((cuAbove && cuAbove->predMode == MODE_INTRA) || (cuLeft && cuLeft->predMode == MODE_INTRA)) ? 1 : 0;
  }

  unsigned CtxIBCFlag(const CodingUnit& cu) const
  {
    const CodingUnit *cuLeft = cuRestrictedLeft[cu.chType]; const CodingUnit *cuAbove = cuRestrictedAbove[cu.chType];
    return ((cuLeft && cuLeft->predMode == MODE_IBC) ? 1 : 0) + ((cuAbove && cuAbove->predMode == MODE_IBC) ? 1 : 0);
  }

};

} // namespace vvenc

//! \}


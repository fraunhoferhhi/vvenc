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


/** \file     TrQuant.cpp
    \brief    transform and quantization class
*/

#include "TrQuant.h"
#include "TrQuant_EMT.h"
#include "QuantRDOQ.h"
#include "DepQuant.h"
#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "dtrace_buffer.h"
#include "TimeProfiler.h"
#include "SearchSpaceCounter.h"

#include <stdlib.h>
#include <memory.h>

//! \ingroup CommonLib
//! \{

namespace vvenc {

struct coeffGroupRDStats
{
  int    iNNZbeforePos0;
  double d64CodedLevelandDist; // distortion and level cost only
  double d64UncodedDist;    // all zero coded block distortion
  double d64SigCost;
  double d64SigCost_0;
};

FwdTrans *const fastFwdTrans[NUM_TRANS_TYPE][g_numTransformMatrixSizes] =
{
  { fastForwardDCT2_B2, fastForwardDCT2_B4, fastForwardDCT2_B8, fastForwardDCT2_B16, fastForwardDCT2_B32, fastForwardDCT2_B64 },
  { nullptr,            fastForwardDCT8_B4, fastForwardDCT8_B8, fastForwardDCT8_B16, fastForwardDCT8_B32, nullptr },
  { nullptr,            fastForwardDST7_B4, fastForwardDST7_B8, fastForwardDST7_B16, fastForwardDST7_B32, nullptr },
};

InvTrans *const fastInvTrans[NUM_TRANS_TYPE][g_numTransformMatrixSizes] =
{
  { fastInverseDCT2_B2, fastInverseDCT2_B4, fastInverseDCT2_B8, fastInverseDCT2_B16, fastInverseDCT2_B32, fastInverseDCT2_B64 },
  { nullptr,            fastInverseDCT8_B4, fastInverseDCT8_B8, fastInverseDCT8_B16, fastInverseDCT8_B32, nullptr },
  { nullptr,            fastInverseDST7_B4, fastInverseDST7_B8, fastInverseDST7_B16, fastInverseDST7_B32, nullptr },
};

//! \ingroup CommonLib
//! \{

static inline int64_t square( const int d ) { return d * (int64_t)d; }

template<int signedMode> std::pair<int64_t,int64_t> fwdTransformCbCr( const PelBuf& resCb, const PelBuf& resCr, PelBuf& resC1, PelBuf& resC2 )
{
  const Pel*  cb  = resCb.buf;
  const Pel*  cr  = resCr.buf;
  Pel*        c1  = resC1.buf;
  Pel*        c2  = resC2.buf;
  int64_t     d1  = 0;
  int64_t     d2  = 0;
  for( SizeType y = 0; y < resCb.height; y++, cb += resCb.stride, cr += resCr.stride, c1 += resC1.stride, c2 += resC2.stride )
  {
    for( SizeType x = 0; x < resCb.width; x++ )
    {
      int cbx = cb[x], crx = cr[x];
      if      ( signedMode ==  1 )
      {
        c1[x] = Pel( ( 4*cbx + 2*crx ) / 5 );
        d1   += square( cbx - c1[x] ) + square( crx - (c1[x]>>1) );
      }
      else if ( signedMode == -1 )
      {
        c1[x] = Pel( ( 4*cbx - 2*crx ) / 5 );
        d1   += square( cbx - c1[x] ) + square( crx - (-c1[x]>>1) );
      }
      else if ( signedMode ==  2 )
      {
        c1[x] = Pel( ( cbx + crx ) / 2 );
        d1   += square( cbx - c1[x] ) + square( crx - c1[x] );
      }
      else if ( signedMode == -2 )
      {
        c1[x] = Pel( ( cbx - crx ) / 2 );
        d1   += square( cbx - c1[x] ) + square( crx + c1[x] );
      }
      else if ( signedMode ==  3 )
      {
        c2[x] = Pel( ( 4*crx + 2*cbx ) / 5 );
        d1   += square( cbx - (c2[x]>>1) ) + square( crx - c2[x] );
      }
      else if ( signedMode == -3 )
      {
        c2[x] = Pel( ( 4*crx - 2*cbx ) / 5 );
        d1   += square( cbx - (-c2[x]>>1) ) + square( crx - c2[x] );
      }
      else
      {
        d1   += square( cbx );
        d2   += square( crx );
      }
    }
  }
  return std::make_pair(d1,d2);
}

template<int signedMode> void invTransformCbCr( PelBuf& resCb, PelBuf& resCr )
{
  Pel*  cb  = resCb.buf;
  Pel*  cr  = resCr.buf;
  for( SizeType y = 0; y < resCb.height; y++, cb += resCb.stride, cr += resCr.stride )
  {
    for( SizeType x = 0; x < resCb.width; x++ )
    {
      if      ( signedMode ==  1 )  { cr[x] =  cb[x] >> 1;  }
      else if ( signedMode == -1 )  { cr[x] = -cb[x] >> 1;  }
      else if ( signedMode ==  2 )  { cr[x] =  cb[x]; }
      else if ( signedMode == -2 )  { cr[x] = -cb[x]; }
      else if ( signedMode ==  3 )  { cb[x] =  cr[x] >> 1; }
      else if ( signedMode == -3 )  { cb[x] = -cr[x] >> 1; }
    }
  }
}

// ====================================================================================================================
// TrQuant class member functions
// ====================================================================================================================
TrQuant::TrQuant() : m_scalingListEnabled(false), m_quant( nullptr )
{
  // allocate temporary buffers
  m_plTempCoeff = ( TCoeff* ) xMalloc( TCoeff, MAX_CU_SIZE * MAX_CU_SIZE );
  m_tmp         = ( TCoeff* ) xMalloc( TCoeff, MAX_CU_SIZE * MAX_CU_SIZE );
  m_blk         = ( TCoeff* ) xMalloc( TCoeff, MAX_CU_SIZE * MAX_CU_SIZE );

  {
    m_invICT      = m_invICTMem + maxAbsIctMode;
    m_invICT[ 0]  = invTransformCbCr< 0>;
    m_invICT[ 1]  = invTransformCbCr< 1>;
    m_invICT[-1]  = invTransformCbCr<-1>;
    m_invICT[ 2]  = invTransformCbCr< 2>;
    m_invICT[-2]  = invTransformCbCr<-2>;
    m_invICT[ 3]  = invTransformCbCr< 3>;
    m_invICT[-3]  = invTransformCbCr<-3>;
    m_fwdICT      = m_fwdICTMem + maxAbsIctMode;
    m_fwdICT[ 0]  = fwdTransformCbCr< 0>;
    m_fwdICT[ 1]  = fwdTransformCbCr< 1>;
    m_fwdICT[-1]  = fwdTransformCbCr<-1>;
    m_fwdICT[ 2]  = fwdTransformCbCr< 2>;
    m_fwdICT[-2]  = fwdTransformCbCr<-2>;
    m_fwdICT[ 3]  = fwdTransformCbCr< 3>;
    m_fwdICT[-3]  = fwdTransformCbCr<-3>;
  }
}

TrQuant::~TrQuant()
{
  if( m_quant )
  {
    delete m_quant;
    m_quant = nullptr;
  }

  // delete temporary buffers
  if( m_plTempCoeff )
  {
    xFree( m_plTempCoeff );
    m_plTempCoeff = nullptr;
  }

  if( m_blk )
  {
    xFree( m_blk );
    m_blk = nullptr;
  }

  if( m_tmp )
  {
    xFree( m_tmp );
    m_tmp = nullptr;
  }
}

void TrQuant::xDeQuant(const TransformUnit& tu,
                             CoeffBuf      &dstCoeff,
                       const ComponentID   &compID,
                       const QpParam       &cQP)
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_QUANT );
  m_quant->dequant( tu, dstCoeff, compID, cQP );
}

void TrQuant::init( const Quant* otherQuant,
                    const int  rdoq,
                    const bool bUseRDOQTS,
                    const bool useSelectiveRDOQ,
                    const bool scalingListsEnabled,
                    const bool bEnc,
                    const bool useTransformSkipFast,
                    const int  thrVal
)
{
  m_bEnc = bEnc;

  delete m_quant;
  m_quant = nullptr;

  {
    m_quant = new DepQuant( otherQuant, bEnc, scalingListsEnabled );
  }

  if( m_quant )
  {
    m_quant->init( rdoq, bUseRDOQTS, useSelectiveRDOQ, thrVal );
  }
}


void TrQuant::invTransformNxN( TransformUnit& tu, const ComponentID compID, PelBuf& pResi, const QpParam& cQP )
{
  const CompArea& area    = tu.blocks[compID];
  const uint32_t uiWidth  = area.width;
  const uint32_t uiHeight = area.height;

  CHECK( uiWidth > tu.cs->sps->getMaxTbSize() || uiHeight > tu.cs->sps->getMaxTbSize(), "Maximal allowed transformation size exceeded!" );

  {
    CoeffBuf tempCoeff = CoeffBuf( m_plTempCoeff, area );
    xDeQuant( tu, tempCoeff, compID, cQP );

    DTRACE_COEFF_BUF( D_TCOEFF, tempCoeff, tu, tu.cu->predMode, compID );

    if (tu.cs->sps->LFNST)
    {
      xInvLfnst(tu, compID);
    }
    if (tu.mtsIdx[compID] == MTS_SKIP)
    {
      xITransformSkip(tempCoeff, pResi, tu, compID);
    }
    else
    {
      xIT(tu, compID, tempCoeff, pResi);
    }
  }

  //DTRACE_BLOCK_COEFF(tu.getCoeffs(compID), tu, tu.cu->predMode, compID);
  DTRACE_PEL_BUF( D_RESIDUALS, pResi, tu, tu.cu->predMode, compID);
}

std::pair<int64_t,int64_t> TrQuant::fwdTransformICT( const TransformUnit& tu, const PelBuf& resCb, const PelBuf& resCr, PelBuf& resC1, PelBuf& resC2, int jointCbCr )
{
  CHECK( Size(resCb) != Size(resCr), "resCb and resCr have different sizes" );
  CHECK( Size(resCb) != Size(resC1), "resCb and resC1 have different sizes" );
  CHECK( Size(resCb) != Size(resC2), "resCb and resC2 have different sizes" );
  return (*m_fwdICT[ TU::getICTMode(tu, jointCbCr) ])( resCb, resCr, resC1, resC2 );
}

void TrQuant::invTransformICT( const TransformUnit& tu, PelBuf& resCb, PelBuf& resCr )
{
  CHECK( Size(resCb) != Size(resCr), "resCb and resCr have different sizes" );
  (*m_invICT[ TU::getICTMode(tu) ])( resCb, resCr );
}

std::vector<int> TrQuant::selectICTCandidates( const TransformUnit& tu, CompStorage* resCb, CompStorage* resCr )
{
  CHECK( !resCb[0].valid() || !resCr[0].valid(), "standard components are not valid" );

  if( !CU::isIntra( *tu.cu ) )
  {
    int cbfMask = 3;
    resCb[cbfMask].create( tu.blocks[COMP_Cb] );
    resCr[cbfMask].create( tu.blocks[COMP_Cr] );
    fwdTransformICT( tu, resCb[0], resCr[0], resCb[cbfMask], resCr[cbfMask], cbfMask );
    std::vector<int> cbfMasksToTest;
    cbfMasksToTest.push_back( cbfMask );
    return cbfMasksToTest;
  }

  std::pair<int64_t,int64_t> pairDist[4];
  for( int cbfMask = 0; cbfMask < 4; cbfMask++ )
  {
    if( cbfMask )
    {
      if (tu.cu->lfnstIdx)
      {
        if (resCb[cbfMask].valid())
        {
          resCb[cbfMask].destroy();
        }
        if (resCr[cbfMask].valid())
        {
          resCr[cbfMask].destroy();
        }
      }
      CHECK( resCb[cbfMask].valid() || resCr[cbfMask].valid(), "target components for cbfMask=" << cbfMask << " are already present" );
      resCb[cbfMask].create( tu.blocks[COMP_Cb] );
      resCr[cbfMask].create( tu.blocks[COMP_Cr] );
    }
    pairDist[cbfMask] = fwdTransformICT( tu, resCb[0], resCr[0], resCb[cbfMask], resCr[cbfMask], cbfMask );
  }

  std::vector<int> cbfMasksToTest;
  int64_t minDist1  = std::min<int64_t>( pairDist[0].first, pairDist[0].second );
  int64_t minDist2  = std::numeric_limits<int64_t>::max();
  int     cbfMask1  = 0;
  int     cbfMask2  = 0;
  for( int cbfMask : { 1, 2, 3 } )
  {
    if( pairDist[cbfMask].first < minDist1 )
    {
      cbfMask2  = cbfMask1; minDist2  = minDist1;
      cbfMask1  = cbfMask;  minDist1  = pairDist[cbfMask1].first;
    }
    else if( pairDist[cbfMask].first < minDist2 )
    {
      cbfMask2  = cbfMask;  minDist2  = pairDist[cbfMask2].first;
    }
  }
  if( cbfMask1 )
  {
    cbfMasksToTest.push_back( cbfMask1 );
  }
  if( cbfMask2 && ( ( minDist2 < (9*minDist1)/8 ) || ( !cbfMask1 && minDist2 < (3*minDist1)/2 ) ) )
  {
    cbfMasksToTest.push_back( cbfMask2 );
  }

  return cbfMasksToTest;
}



// ------------------------------------------------------------------------------------------------
// Logical transform
// ------------------------------------------------------------------------------------------------
void TrQuant::xSetTrTypes( const TransformUnit& tu, const ComponentID compID, const int width, const int height, int &trTypeHor, int &trTypeVer )
{
  const bool isISP = CU::isIntra(*tu.cu) && tu.cu->ispMode && isLuma(compID);
  if (isISP && tu.cu->lfnstIdx)
  {
    return;
  }
  if (!tu.cs->sps->MTS)
  {
    return;
  }
  if (CU::isIntra(*tu.cu) && isLuma(compID) && ((tu.cs->sps->getUseImplicitMTS() && tu.cu->lfnstIdx == 0 && tu.cu->mipFlag == 0) || tu.cu->ispMode))
  {
    if (width >= 4 && width <= 16)
      trTypeHor = DST7;
    if (height >= 4 && height <= 16)
      trTypeVer = DST7;
  }
  else if( tu.cs->sps->MTS && tu.cu->sbtInfo && isLuma(compID)/*isSBT*/ )
  {
    const uint8_t sbtIdx = CU::getSbtIdx( tu.cu->sbtInfo );
    const uint8_t sbtPos = CU::getSbtPos( tu.cu->sbtInfo );

    if( sbtIdx == SBT_VER_HALF || sbtIdx == SBT_VER_QUAD )
    {
      assert( tu.lwidth() <= MTS_INTER_MAX_CU_SIZE );
      if( tu.lheight() > MTS_INTER_MAX_CU_SIZE )
      {
        trTypeHor = trTypeVer = DCT2;
      }
      else
      {
        if( sbtPos == SBT_POS0 )  { trTypeHor = DCT8;  trTypeVer = DST7; }
        else                      { trTypeHor = DST7;  trTypeVer = DST7; }
      }
    }
    else
    {
      assert( tu.lheight() <= MTS_INTER_MAX_CU_SIZE );
      if( tu.lwidth() > MTS_INTER_MAX_CU_SIZE )
      {
        trTypeHor = trTypeVer = DCT2;
      }
      else
      {
        if( sbtPos == SBT_POS0 )  { trTypeHor = DST7;  trTypeVer = DCT8; }
        else                      { trTypeHor = DST7;  trTypeVer = DST7; }
      }
    }
  }
  const bool isExplicitMTS = (CU::isIntra(*tu.cu) ? tu.cs->sps->MTS : tu.cs->sps->MTSInter && CU::isInter(*tu.cu)) && isLuma(compID);
  if (isExplicitMTS)
  {
    if (tu.mtsIdx[compID] > MTS_SKIP)
    {
      int indHor = (tu.mtsIdx[compID] - MTS_DST7_DST7) & 1;
      int indVer = (tu.mtsIdx[compID] - MTS_DST7_DST7) >> 1;
      trTypeHor  = indHor ? DCT8 : DST7;
      trTypeVer  = indVer ? DCT8 : DST7;
    }
  }
}


void TrQuant::xT( const TransformUnit& tu, const ComponentID compID, const CPelBuf& resi, CoeffBuf& dstCoeff, const int width, const int height )
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_TRAFO );

  const unsigned maxLog2TrDynamicRange  = tu.cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const unsigned bitDepth               = tu.cs->sps->bitDepths[               toChannelType( compID ) ];
  const int      TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_FORWARD];
  const uint32_t transformWidthIndex    = Log2(width ) - 1;  // nLog2WidthMinus1, since transform start from 2-point
  const uint32_t transformHeightIndex   = Log2(height) - 1;  // nLog2HeightMinus1, since transform start from 2-point

  int trTypeHor = DCT2;
  int trTypeVer = DCT2;

  xSetTrTypes( tu, compID, width, height, trTypeHor, trTypeVer );

  int  skipWidth  = ( trTypeHor != DCT2 && width  == 32 ) ? 16 : width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  int  skipHeight = ( trTypeVer != DCT2 && height == 32 ) ? 16 : height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;

  if( tu.cu->lfnstIdx )
  {
    if ((width == 4 && height > 4) || (width > 4 && height == 4))
    {
      skipWidth  = width - 4;
      skipHeight = height - 4;
    }
    else if ((width >= 8 && height >= 8))
    {
      skipWidth  = width - 8;
      skipHeight = height - 8;
    }
  }

  TCoeff* block = m_blk;
  TCoeff* tmp   = m_tmp;

  const Pel* resiBuf    = resi.buf;
  const int  resiStride = resi.stride;

#if ENABLE_SIMD_TRAFO
  if( width & 3 )
#endif
  {
    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        block[( y * width ) + x] = resiBuf[( y * resiStride ) + x];
      }
    }
  }
#if ENABLE_SIMD_TRAFO
  else if( width & 7 )
  {
    g_tCoeffOps.cpyCoeff4( resiBuf, resiStride, block, width, height );
  }
  else
  {
    g_tCoeffOps.cpyCoeff8( resiBuf, resiStride, block, width, height );
  }
#endif //ENABLE_SIMD_TRAFO

  if (width > 1 && height > 1)
  {
    const int shift_1st = ((Log2(width )) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    const int shift_2nd =  (Log2(height))            + TRANSFORM_MATRIX_SHIFT                          + COM16_C806_TRANS_PREC;
    CHECK( shift_1st < 0, "Negative shift" );
    CHECK( shift_2nd < 0, "Negative shift" );
    fastFwdTrans[trTypeHor][transformWidthIndex](block, tmp, shift_1st, height, 0, skipWidth);
    fastFwdTrans[trTypeVer][transformHeightIndex](tmp, dstCoeff.buf, shift_2nd, width, skipWidth, skipHeight);
  }
  else if (height == 1)   // 1-D horizontal transform
  {
    const int shift = ((Log2(width )) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    CHECK( shift < 0, "Negative shift" );
    fastFwdTrans[trTypeHor][transformWidthIndex](block, dstCoeff.buf, shift, 1, 0, skipWidth);
  }
  else   // if (iWidth == 1) //1-D vertical transform
  {
    int shift = ((floorLog2(height)) + bitDepth + TRANSFORM_MATRIX_SHIFT) - maxLog2TrDynamicRange + COM16_C806_TRANS_PREC;
    CHECK(shift < 0, "Negative shift");
    CHECKD((transformHeightIndex < 0), "There is a problem with the height.");
    fastFwdTrans[trTypeVer][transformHeightIndex](block, dstCoeff.buf, shift, 1, 0, skipHeight);
  }
}


void TrQuant::xIT( const TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pCoeff, PelBuf& pResidual )
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_TRAFO );

  const int      width                  = pCoeff.width;
  const int      height                 = pCoeff.height;
  const unsigned maxLog2TrDynamicRange  = tu.cs->sps->getMaxLog2TrDynamicRange( toChannelType( compID ) );
  const unsigned bitDepth               = tu.cs->sps->bitDepths[               toChannelType( compID ) ];
  const int      TRANSFORM_MATRIX_SHIFT = g_transformMatrixShift[TRANSFORM_INVERSE];
  const TCoeff   clipMinimum            = -( 1 << maxLog2TrDynamicRange );
  const TCoeff   clipMaximum            =  ( 1 << maxLog2TrDynamicRange ) - 1;
  const uint32_t transformWidthIndex    = Log2(width )- 1;                                // nLog2WidthMinus1, since transform start from 2-point
  const uint32_t transformHeightIndex   = Log2(height) - 1;                                // nLog2HeightMinus1, since transform start from 2-point


  int trTypeHor = DCT2;
  int trTypeVer = DCT2;

  xSetTrTypes( tu, compID, width, height, trTypeHor, trTypeVer );

  int skipWidth  = ( trTypeHor != DCT2 && width  == 32 ) ? 16 : width  > JVET_C0024_ZERO_OUT_TH ? width  - JVET_C0024_ZERO_OUT_TH : 0;
  int skipHeight = ( trTypeVer != DCT2 && height == 32 ) ? 16 : height > JVET_C0024_ZERO_OUT_TH ? height - JVET_C0024_ZERO_OUT_TH : 0;

  if (tu.cs->sps->LFNST && tu.cu->lfnstIdx)
  {
    if ((width == 4 && height > 4) || (width > 4 && height == 4))
    {
      skipWidth = width - 4;
      skipHeight = height - 4;
    }
    else if ((width >= 8 && height >= 8))
    {
      skipWidth = width - 8;
      skipHeight = height - 8;
    }
  }

  TCoeff *block = m_blk;
  TCoeff *tmp   = m_tmp;
  if (width > 1 && height > 1)   // 2-D transform
  {
    const int shift_1st =   TRANSFORM_MATRIX_SHIFT + 1 + COM16_C806_TRANS_PREC; // 1 has been added to shift_1st at the expense of shift_2nd
    const int shift_2nd = ( TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1 ) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK( shift_1st < 0, "Negative shift" );
    CHECK( shift_2nd < 0, "Negative shift" );
    fastInvTrans[trTypeVer][transformHeightIndex](pCoeff.buf, tmp, shift_1st, width, skipWidth, skipHeight, clipMinimum, clipMaximum);
    fastInvTrans[trTypeHor][transformWidthIndex](tmp, block, shift_2nd, height, 0, skipWidth, clipMinimum, clipMaximum);
  }
  else if (width == 1)   // 1-D vertical transform
  {
    int shift = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK(shift < 0, "Negative shift");
    CHECK((transformHeightIndex < 0), "There is a problem with the height.");
    fastInvTrans[trTypeVer][transformHeightIndex](pCoeff.buf, block, shift + 1, 1, 0, skipHeight, clipMinimum, clipMaximum);
  }
  else   // if(iHeight == 1) //1-D horizontal transform
  {
    const int shift = (TRANSFORM_MATRIX_SHIFT + maxLog2TrDynamicRange - 1) - bitDepth + COM16_C806_TRANS_PREC;
    CHECK(shift < 0, "Negative shift");
    CHECK((transformWidthIndex < 0), "There is a problem with the width.");
    fastInvTrans[trTypeHor][transformWidthIndex](pCoeff.buf, block, shift + 1, 1, 0, skipWidth, clipMinimum, clipMaximum);
  }

#if ENABLE_SIMD_TRAFO
  if( width & 3 )
#endif //ENABLE_SIMD_TRAFO
  {
    Pel       *dst    = pResidual.buf;
    ptrdiff_t  stride = pResidual.stride;

    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        dst[x] = ( Pel ) *block++;
      }

      dst += stride;
    }
  }
#if ENABLE_SIMD_TRAFO
  else if( width & 7 )
  {
    g_tCoeffOps.cpyResi4( block, pResidual.buf, pResidual.stride, width, height );
  }
  else
  {
    g_tCoeffOps.cpyResi8( block, pResidual.buf, pResidual.stride, width, height );
  }
#endif //ENABLE_SIMD_TRAFO
}

/** Wrapper function between HM interface and core NxN transform skipping
 */
void TrQuant::xITransformSkip(const CCoeffBuf& pCoeff,
  PelBuf& pResidual,
  const TransformUnit& tu,
  const ComponentID compID)
{
  const CompArea& area = tu.blocks[compID];
  const int width = area.width;
  const int height = area.height;

  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      pResidual.at(x, y) = Pel(pCoeff.at(x, y));
    }
  }
}

void TrQuant::xQuant(TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx& ctx)
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, g_timeProfiler, P_QUANT, tu.cs, toChannelType(compID) );
  m_quant->quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
#if ENABLE_MEASURE_SEARCH_SPACE

  g_searchSpaceAcc.addQuant( tu, toChannelType( compID ) );
#endif
}


void TrQuant::transformNxN(TransformUnit &tu, const ComponentID compID, const QpParam &cQP, TCoeff &uiAbsSum, const Ctx &ctx, const bool loadTr)
{
        CodingStructure &cs = *tu.cs;
  const CompArea& rect      = tu.blocks[compID];
  const uint32_t uiWidth        = rect.width;
  const uint32_t uiHeight       = rect.height;

  const CPelBuf resiBuf     = cs.getResiBuf(rect);

  if( tu.noResidual )
  {
    uiAbsSum = 0;
    TU::setCbfAtDepth( tu, compID, tu.depth, uiAbsSum > 0 );
    return;
  }
  if (tu.cu->bdpcmM[toChannelType(compID)])
  {
    tu.mtsIdx[compID] = MTS_SKIP;
  }

  uiAbsSum = 0;
  CHECK( cs.sps->getMaxTbSize() < uiWidth, "Unsupported transformation size" );

  CoeffBuf tempCoeff(loadTr ? m_mtsCoeffs[tu.mtsIdx[compID]] : m_plTempCoeff, rect);
  if (!loadTr)
  {
    DTRACE_PEL_BUF( D_RESIDUALS, resiBuf, tu, tu.cu->predMode, compID );
    if (tu.mtsIdx[compID] == MTS_SKIP)
    {
      xTransformSkip(tu, compID, resiBuf, tempCoeff.buf);
    }
    else
    {
      xT(tu, compID, resiBuf, tempCoeff, uiWidth, uiHeight);
    }
  }
  if (cs.sps->LFNST)
  {
    xFwdLfnst(tu, compID, loadTr);
  }
  DTRACE_COEFF_BUF( D_TCOEFF, tempCoeff, tu, tu.cu->predMode, compID );

  xQuant( tu, compID, tempCoeff, uiAbsSum, cQP, ctx );

  DTRACE_COEFF_BUF( D_TCOEFF, tu.getCoeffs( compID ), tu, tu.cu->predMode, compID );

  // set coded block flag (CBF)
  TU::setCbfAtDepth (tu, compID, tu.depth, uiAbsSum > 0);
}

void TrQuant::checktransformsNxN( TransformUnit &tu, std::vector<TrMode> *trModes, const int maxCand, const ComponentID compID)
{
  CodingStructure &cs     = *tu.cs;
  const CompArea& rect    = tu.blocks[compID];
  const uint32_t   width  = rect.width;
  const uint32_t   height = rect.height;

  const CPelBuf resiBuf = cs.getResiBuf(rect);

  CHECK(cs.sps->getMaxTbSize() < width, "Unsupported transformation size");
  int                           pos = 0;
  std::vector<TrCost>           trCosts;
  std::vector<TrMode>::iterator it      = trModes->begin();
  const double                  facBB[] = { 1.2, 1.3, 1.3, 1.4, 1.5 };
  while (it != trModes->end())
  {
    tu.mtsIdx[compID] = it->first;
    CoeffBuf tempCoeff(m_mtsCoeffs[tu.mtsIdx[compID]], rect);
    if (tu.noResidual)
    {
      int sumAbs = 0;
      trCosts.push_back(TrCost(sumAbs, pos++));
      it++;
      continue;
    }
    if (tu.mtsIdx[compID] == MTS_SKIP)
    {
      xTransformSkip(tu, compID, resiBuf, tempCoeff.buf);
    }
    else
    {
      xT(tu, compID, resiBuf, tempCoeff, width, height);
    }

    int sumAbs = 0;
    for (int pos = 0; pos < width * height; pos++)
    {
      sumAbs += abs(tempCoeff.buf[pos]);
    }

    double scaleSAD = 1.0;
    if (tu.mtsIdx[compID] == MTS_SKIP && ((floorLog2(width) + floorLog2(height)) & 1) == 1)
    {
      scaleSAD = 1.0 / 1.414213562;   // compensate for not scaling transform skip coefficients by 1/sqrt(2)
    }
    if (tu.mtsIdx[compID] == MTS_SKIP)
    {
      int trShift = getTransformShift(tu.cu->slice->sps->bitDepths[CH_L], rect.size(), tu.cu->slice->sps->getMaxLog2TrDynamicRange(toChannelType(compID)));
      scaleSAD *= pow(2, trShift);
    }
    trCosts.push_back(TrCost(int(std::min<double>(sumAbs * scaleSAD, std::numeric_limits<int>::max())), pos++));
    it++;
  }

  int                           numTests = 0;
  std::vector<TrCost>::iterator itC      = trCosts.begin();
  const double                  fac      = facBB[std::max(0, floorLog2(std::max(width, height)) - 2)];
  const double                  thr      = fac * trCosts.begin()->first;
  const double                  thrTS    = trCosts.begin()->first;
  while (itC != trCosts.end())
  {
    const bool testTr               = itC->first <= (itC->second == 1 ? thrTS : thr) && numTests <= maxCand;
    trModes->at(itC->second).second = testTr;
    numTests += testTr;
    itC++;
  }
}


void TrQuant::xFwdLfnstNxN(int *src, int *dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize)
{
  const int8_t *trMat  = (size > 4) ? g_lfnst8x8[mode][index][0] : g_lfnst4x4[mode][index][0];
  const int     trSize = (size > 4) ? 48 : 16;
  int           coef;
  int *         out = dst;

  assert(index < 3);

  for (int j = 0; j < zeroOutSize; j++)
  {
    int *         srcPtr   = src;
    const int8_t *trMatTmp = trMat;
    coef                   = 0;
    for (int i = 0; i < trSize; i++)
    {
      coef += *srcPtr++ * *trMatTmp++;
    }
    *out++ = (coef + 64) >> 7;
    trMat += trSize;
  }

  ::memset(out, 0, (trSize - zeroOutSize) * sizeof(int));
}


void TrQuant::xInvLfnstNxN(int *src, int *dst, const uint32_t mode, const uint32_t index, const uint32_t size, int zeroOutSize)
{
  int           maxLog2TrDynamicRange = 15;
  const TCoeff  outputMinimum         = -(1 << maxLog2TrDynamicRange);
  const TCoeff  outputMaximum         = (1 << maxLog2TrDynamicRange) - 1;
  const int8_t *trMat                 = (size > 4) ? g_lfnst8x8[mode][index][0] : g_lfnst4x4[mode][index][0];
  const int     trSize                = (size > 4) ? 48 : 16;
  int           resi;
  int *         out = dst;

  assert(index < 3);

  for (int j = 0; j < trSize; j++)
  {
    resi                   = 0;
    const int8_t *trMatTmp = trMat;
    int *         srcPtr   = src;
    for (int i = 0; i < zeroOutSize; i++)
    {
      resi += *srcPtr++ * *trMatTmp;
      trMatTmp += trSize;
    }
    *out++ = Clip3(outputMinimum, outputMaximum, (int) (resi + 64) >> 7);
    trMat++;
  }
}

uint32_t TrQuant::xGetLFNSTIntraMode( const Area& tuArea, const uint32_t dirMode )
{
  if (dirMode < 2)
  {
    return dirMode;
  }

  static const int modeShift[] = { 0, 6, 10, 12, 14, 15 };

  const int width  = int(tuArea.width);
  const int height = int(tuArea.height);

  if (width > height && dirMode < 2 + modeShift[floorLog2(width) - floorLog2(height)])
  {
    return dirMode + (VDIA_IDX - 1) + (NUM_EXT_LUMA_MODE >> 1);
  }
  else if (height > width && dirMode > VDIA_IDX - modeShift[floorLog2(height) - floorLog2(width)])
  {
    return dirMode - (VDIA_IDX + 1) + (NUM_EXT_LUMA_MODE >> 1) + NUM_LUMA_MODE;
  }

  return dirMode;
}


bool TrQuant::xGetTransposeFlag(uint32_t intraMode)
{
  return ((intraMode >= NUM_LUMA_MODE) && (intraMode >= (NUM_LUMA_MODE + (NUM_EXT_LUMA_MODE >> 1))))
         || ((intraMode < NUM_LUMA_MODE) && (intraMode > DIA_IDX));
}


void TrQuant::xInvLfnst(const TransformUnit &tu, const ComponentID compID)
{
  const CompArea &area     = tu.blocks[compID];
  const uint32_t  width    = area.width;
  const uint32_t  height   = area.height;
  const uint32_t  lfnstIdx = tu.cu->lfnstIdx;
  if (lfnstIdx && tu.mtsIdx[compID] != MTS_SKIP && (CU::isSepTree(*tu.cu) ? true : isLuma(compID)))
  {
    const CodingUnit& cu = *tu.cs->getCU(area.pos(), toChannelType(compID), TREE_D);
    const bool         whge3 = width >= 8 && height >= 8;
    const ScanElement *scan =
      whge3
        ? g_coefTopLeftDiagScan8x8[Log2(width)] 
        : getScanOrder(SCAN_GROUPED_4x4, Log2(area.width), Log2(area.height));
    uint32_t intraMode = CU::getFinalIntraMode(cu, toChannelType(compID));

    if (CU::isLMCMode( cu.intraDir[toChannelType(compID)]))
    {
      intraMode = CU::getCoLocatedIntraLumaMode(cu);
    }
    if (CU::isMIP(cu, toChannelType(compID)))
    {
      intraMode = PLANAR_IDX;
    }
    CHECK(intraMode >= NUM_INTRA_MODE - 1, "Invalid intra mode");

    if (lfnstIdx < 3)
    {
      if (tu.cu->ispMode && isLuma(compID))
      {
        intraMode = xGetLFNSTIntraMode(tu.cu->blocks[compID], intraMode);
      }
      else
        intraMode = xGetLFNSTIntraMode(tu.blocks[compID], intraMode);
      bool      transposeFlag = xGetTransposeFlag(intraMode);
      const int sbSize        = whge3 ? 8 : 4;
      bool      tu4x4Flag     = (width == 4 && height == 4);
      bool      tu8x8Flag     = (width == 8 && height == 8);
      TCoeff *  lfnstTemp;
      TCoeff *  coeffTemp;
      int       y;
      lfnstTemp                  = m_tempInMatrix;   // inverse spectral rearrangement
      coeffTemp                  = m_plTempCoeff;
      TCoeff *           dst     = lfnstTemp;
      const ScanElement *scanPtr = scan;
      for (y = 0; y < 16; y++)
      {
        *dst++ = coeffTemp[scanPtr->idx];
        scanPtr++;
      }

      xInvLfnstNxN(m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[intraMode], lfnstIdx - 1, sbSize,
                  (tu4x4Flag || tu8x8Flag) ? 8 : 16);

      lfnstTemp = m_tempOutMatrix;   // inverse spectral rearrangement

      if (transposeFlag)
      {
        if (sbSize == 4)
        {
          for (y = 0; y < 4; y++)
          {
            coeffTemp[0] = lfnstTemp[0];
            coeffTemp[1] = lfnstTemp[4];
            coeffTemp[2] = lfnstTemp[8];
            coeffTemp[3] = lfnstTemp[12];
            lfnstTemp++;
            coeffTemp += width;
          }
        }
        else   // ( sbSize == 8 )
        {
          for (y = 0; y < 8; y++)
          {
            coeffTemp[0] = lfnstTemp[0];
            coeffTemp[1] = lfnstTemp[8];
            coeffTemp[2] = lfnstTemp[16];
            coeffTemp[3] = lfnstTemp[24];
            if (y < 4)
            {
              coeffTemp[4] = lfnstTemp[32];
              coeffTemp[5] = lfnstTemp[36];
              coeffTemp[6] = lfnstTemp[40];
              coeffTemp[7] = lfnstTemp[44];
            }
            lfnstTemp++;
            coeffTemp += width;
          }
        }
      }
      else
      {
        for (y = 0; y < sbSize; y++)
        {
          uint32_t uiStride = (y < 4) ? sbSize : 4;
          ::memcpy(coeffTemp, lfnstTemp, uiStride * sizeof(TCoeff));
          lfnstTemp += uiStride;
          coeffTemp += width;
        }
      }
    }
  }
}


void TrQuant::xFwdLfnst(const TransformUnit &tu, const ComponentID compID, const bool loadTr)
{
  const CompArea &area     = tu.blocks[compID];
  const uint32_t  width    = area.width;
  const uint32_t  height   = area.height;
  const uint32_t  lfnstIdx = tu.cu->lfnstIdx;
  if (lfnstIdx && tu.mtsIdx[compID] != MTS_SKIP && (CU::isSepTree(*tu.cu) ? true : isLuma(compID)))
  {
    const CodingUnit& cu = *tu.cs->getCU(area.pos(), toChannelType(compID), TREE_D);
    const bool         whge3 = width >= 8 && height >= 8;
    const ScanElement *scan =
      whge3
        ? g_coefTopLeftDiagScan8x8[Log2(width)] 
        : getScanOrder(SCAN_GROUPED_4x4, Log2(area.width), Log2(area.height));   
    uint32_t intraMode = CU::getFinalIntraMode(cu, toChannelType(compID));

    if (CU::isLMCMode(cu.intraDir[toChannelType(compID)]))
    {
      intraMode = CU::getCoLocatedIntraLumaMode(cu);
    }
    if (CU::isMIP(cu, toChannelType(compID)))
    {
      intraMode = PLANAR_IDX;
    }
    CHECK(intraMode >= NUM_INTRA_MODE - 1, "Invalid intra mode");

    if (lfnstIdx < 3)
    {
      if (tu.cu->ispMode && isLuma(compID))
      {
        intraMode = xGetLFNSTIntraMode(tu.cu->blocks[compID], intraMode);
      }
      else
      {
        intraMode = xGetLFNSTIntraMode(tu.blocks[compID], intraMode);
      }
      bool      transposeFlag = xGetTransposeFlag(intraMode);
      const int sbSize        = whge3 ? 8 : 4;
      bool      tu4x4Flag     = (width == 4 && height == 4);
      bool      tu8x8Flag     = (width == 8 && height == 8);
      TCoeff*   lfnstTemp;
      TCoeff*   coeffTemp;
      TCoeff*   tempCoeff     = loadTr ? m_mtsCoeffs[tu.mtsIdx[compID]] : m_plTempCoeff;

      int y;
      lfnstTemp = m_tempInMatrix;   // forward low frequency non-separable transform
      coeffTemp = tempCoeff;

      if (transposeFlag)
      {
        if (sbSize == 4)
        {
          for (y = 0; y < 4; y++)
          {
            lfnstTemp[0]  = coeffTemp[0];
            lfnstTemp[4]  = coeffTemp[1];
            lfnstTemp[8]  = coeffTemp[2];
            lfnstTemp[12] = coeffTemp[3];
            lfnstTemp++;
            coeffTemp += width;
          }
        }
        else   // ( sbSize == 8 )
        {
          for (y = 0; y < 8; y++)
          {
            lfnstTemp[0]  = coeffTemp[0];
            lfnstTemp[8]  = coeffTemp[1];
            lfnstTemp[16] = coeffTemp[2];
            lfnstTemp[24] = coeffTemp[3];
            if (y < 4)
            {
              lfnstTemp[32] = coeffTemp[4];
              lfnstTemp[36] = coeffTemp[5];
              lfnstTemp[40] = coeffTemp[6];
              lfnstTemp[44] = coeffTemp[7];
            }
            lfnstTemp++;
            coeffTemp += width;
          }
        }
      }
      else
      {
        for (y = 0; y < sbSize; y++)
        {
          uint32_t uiStride = (y < 4) ? sbSize : 4;
          ::memcpy(lfnstTemp, coeffTemp, uiStride * sizeof(TCoeff));
          lfnstTemp += uiStride;
          coeffTemp += width;
        }
      }

      xFwdLfnstNxN(m_tempInMatrix, m_tempOutMatrix, g_lfnstLut[intraMode], lfnstIdx - 1, sbSize,
                  (tu4x4Flag || tu8x8Flag) ? 8 : 16);

      lfnstTemp                        = m_tempOutMatrix;   // forward spectral rearrangement
      coeffTemp                        = tempCoeff;
      const ScanElement *scanPtr       = scan;
      int                lfnstCoeffNum = (sbSize == 4) ? sbSize * sbSize : 48;
      for (y = 0; y < lfnstCoeffNum; y++)
      {
        coeffTemp[scanPtr->idx] = *lfnstTemp++;
        scanPtr++;
      }
    }
  }
}

void TrQuant::xTransformSkip(const TransformUnit& tu, const ComponentID& compID, const CPelBuf& resi, TCoeff* psCoeff)
{
  const CompArea& rect = tu.blocks[compID];
  const uint32_t width = rect.width;
  const uint32_t height = rect.height;

  for (uint32_t y = 0, coefficientIndex = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++, coefficientIndex++)
    {
      psCoeff[coefficientIndex] = TCoeff(resi.at(x, y));
    }
  }
}
} // namespace vvenc

//! \}


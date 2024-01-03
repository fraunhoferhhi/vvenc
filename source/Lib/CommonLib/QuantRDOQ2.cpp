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


/** \file     QuantRDOQ2.cpp
    \brief    transform and quantization class
*/

#include "QuantRDOQ2.h"
#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "dtrace_next.h"
#include "dtrace_buffer.h"

#include <stdlib.h>
#include <memory.h>

#if defined( TARGET_SIMD_X86 )
#  include "CommonDefX86.h"
#  include <simde/x86/sse4.1.h>
#endif

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


//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================
#define COEFF_ERR_SCALE_PRECISION_BITS 20

//! \}

QuantRDOQ2::QuantRDOQ2( const Quant* other, bool useScalingLists ) : QuantRDOQ( other, useScalingLists ), m_isErrScaleListOwner( false ), m_iLambda( 0 )
{
  const QuantRDOQ2 *rdoq2 = dynamic_cast<const QuantRDOQ2*>( other );
  CHECK( other && !rdoq2, "The RDOQ cast must be successfull!" );
  xInitScalingList( rdoq2 );
}

QuantRDOQ2::~QuantRDOQ2()
{
  xDestroyScalingList();
}


/** initialization process of scaling list array
*/
void QuantRDOQ2::xInitScalingList( const QuantRDOQ2* other )
{
  m_isErrScaleListOwner = other == nullptr;

  const bool useScalingLists = getScalingListEnabled();

  for(uint32_t sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++)
  {
    for(uint32_t sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++)
    {
      for(uint32_t qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
      {
        for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
        {
          if( m_isErrScaleListOwner )
          {
            m_errScale[sizeIdX][sizeIdY][listId][qp] = useScalingLists ? new int[g_scalingListSizeX[sizeIdX] * g_scalingListSizeX[sizeIdY]] : nullptr;
          }
          else
          {
            m_errScale[sizeIdX][sizeIdY][listId][qp] = other->m_errScale[sizeIdX][sizeIdY][listId][qp];
          }
        } // listID loop
      }
    }
  }
}

/** destroy quantization matrix array
*/
void QuantRDOQ2::xDestroyScalingList()
{
  if( !m_isErrScaleListOwner ) return;

  for(uint32_t sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++)
  {
    for(uint32_t sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++)
    {
      for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
      {
        for(uint32_t qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
        {
          if(m_errScale[sizeIdX][sizeIdY][listId][qp])
          {
            delete [] m_errScale[sizeIdX][sizeIdY][listId][qp];
          }
        }
      }
    }
  }
  //   Quant::destroyScalingList();
}

int QuantRDOQ2::xGetErrScaleCoeff( const bool needsSqrt2, SizeType width, SizeType height, int qp, const int maxLog2TrDynamicRange, const int channelBitDepth )
{
  const int iTransformShift = getTransformShift(channelBitDepth, Size(width, height), maxLog2TrDynamicRange);
  double    dErrScale = (double)(1 << SCALE_BITS);                                // Compensate for scaling of bitcount in Lagrange cost function
  double    dTransShift = (double)iTransformShift + (needsSqrt2 ? -0.5 : 0.0);
  dErrScale = dErrScale * pow(2.0, (-2.0*dTransShift));                     // Compensate for scaling through forward transform
  const int  QStep = g_quantScales[needsSqrt2 ? 1 : 0][qp];
  double    finalErrScale = dErrScale / QStep / QStep / (1 << (DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth) << 1));
  return    finalErrScale;
}

/** set error scale coefficients
* \param list                   list ID
* \param size
* \param qp                     quantization parameter
* \param maxLog2TrDynamicRange
* \param bitDepths              reference to bit depth array for all channels
*/
void QuantRDOQ2::xSetErrScaleCoeff( unsigned list, unsigned sizeX, unsigned sizeY, int qp, const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths )
{
  const int width               = g_scalingListSizeX[sizeX];
  const int height              = g_scalingListSizeX[sizeY];
  const ChannelType channelType = ((list == 0) || (list == MAX_NUM_COMP)) ? CH_L : CH_C;
  const int channelBitDepth     = bitDepths.recon[channelType];
  const int iTransformShift     = getTransformShift( channelBitDepth, Size( width, height ), maxLog2TrDynamicRange[channelType] );
  const double dTransShift      = (double)iTransformShift;

  double dErrScale = pow( 2.0, ( (double)SCALE_BITS / 2.0 ) );    // Compensate for scaling of bitcount in Lagrange cost function
  dErrScale = dErrScale*pow( 2.0, ( -/*2.0**/( dTransShift ) ) );   // Compensate for scaling through forward transform

  if( getScalingListEnabled() )
  {
    const unsigned uiMaxNumCoeff  = g_scalingListSizeX[sizeX] * g_scalingListSizeX[sizeY];
    const int *piQuantCoeff       = getQuantCoeff( list, qp, sizeX, sizeY );
    int *piErrScale               = xGetErrScaleCoeffSL( list, sizeX, sizeY, qp );

    for( unsigned i = 0; i < uiMaxNumCoeff; i++ )
    {
      int QStep = piQuantCoeff[i];
      double errScale = dErrScale / QStep / (1 << (DISTORTION_PRECISION_ADJUSTMENT( channelBitDepth ) /*<< 1*/)); // (1 << ( /*2 **/ (bitDepths.recon[channelType] - 8)));
      piErrScale[i] = ( int ) (errScale * ( double ) (1 << COEFF_ERR_SCALE_PRECISION_BITS));
    }
  }

  xSetErrScaleCoeffNoScalingList( list, sizeX, sizeY, qp, maxLog2TrDynamicRange, bitDepths );
}

void QuantRDOQ2::xSetErrScaleCoeffNoScalingList( unsigned list, unsigned wIdx, unsigned hIdx, int qp, const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths )
{
  const int width               = g_scalingListSizeX[wIdx];
  const int height              = g_scalingListSizeX[hIdx];
  const ChannelType channelType = ( ( list == 0 ) || ( list == MAX_NUM_COMP ) ) ? CH_L : CH_C;
  const int channelBitDepth     = bitDepths.recon[channelType];
  const int iTransformShift     = getTransformShift( channelBitDepth, Size( width, height ), maxLog2TrDynamicRange[channelType] );
  const bool needsSqrt2         = ((Log2(width*height)) & 1) == 1;
  double dTransShift            = (double)iTransformShift + ( needsSqrt2 ? -0.5 : 0.0 );

  double dErrScale   = pow( 2.0, ( (double)SCALE_BITS / 2.0 ) );             // Compensate for scaling of bitcount in Lagrange cost function
  dErrScale          = dErrScale*pow( 2.0, ( -( dTransShift ) ) );          // Compensate for scaling through forward transform
  int QStep          = g_quantScales[needsSqrt2][qp];

  double errScale = dErrScale / QStep /*/ QStep*/ / (1 << (DISTORTION_PRECISION_ADJUSTMENT( channelBitDepth ) /*<< 1*/));
  xGetErrScaleCoeffNoScalingList( list, wIdx, hIdx, qp ) = (int)( errScale * (double)( 1 << COEFF_ERR_SCALE_PRECISION_BITS ) );
}


/** set flat matrix value to quantized coefficient
*/
void QuantRDOQ2::setFlatScalingList(const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths)
{
  QuantRDOQ::setFlatScalingList( maxLog2TrDynamicRange, bitDepths );

  const int minimumQp = 0;
  const int maximumQp = SCALING_LIST_REM_NUM;

  for(uint32_t sizeX = 0; sizeX < SCALING_LIST_SIZE_NUM; sizeX++)
  {
    for(uint32_t sizeY = 0; sizeY < SCALING_LIST_SIZE_NUM; sizeY++)
    {
      for(uint32_t list = 0; list < SCALING_LIST_NUM; list++)
      {
        for(int qp = minimumQp; qp < maximumQp; qp++)
        {
          xSetErrScaleCoeff( list, sizeX, sizeY, qp, maxLog2TrDynamicRange, bitDepths );
        }
      }
    }
  }
}


void QuantRDOQ2::quant( TransformUnit &tu, const ComponentID compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx& ctx )
{
  if( m_RDOQ == 1 )
  {
    QuantRDOQ::quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
    return;
  }

  const CompArea& rect      = tu.blocks[compID];
  const uint32_t uiWidth    = rect.width;
  const uint32_t uiHeight   = rect.height;

  const CCoeffBuf&  piCoef   = pSrc;

  const bool useTransformSkip = tu.mtsIdx[compID]==MTS_SKIP;

  bool useRDOQ = useTransformSkip ? m_useRDOQTS : m_RDOQ > 0;

  if( !tu.cu->ispMode || !isLuma(compID) )
  {
    useRDOQ &= uiWidth > 2;
    useRDOQ &= uiHeight > 2;
  }

  if( useRDOQ )
  {
    if( !tu.cs->picture->useSelectiveRdoq || xNeedRDOQ( tu, compID, piCoef, cQP ) )
    {
      if( useTransformSkip )
      {
        if( tu.cu->bdpcmM[toChannelType( compID )] )
        {
          forwardRDPCM( tu, compID, pSrc, uiAbsSum, cQP, ctx );
        }
        else
        {
          rateDistOptQuantTS( tu, compID, pSrc, uiAbsSum, cQP, ctx );
        }
      }
      else
      {
        xRateDistOptQuant( tu, compID, pSrc, uiAbsSum, cQP, ctx, getScalingListEnabled() );
      }
    }
    else
    {
      uiAbsSum    = 0;
      tu.lastPos[compID] = -1;
    }
  }
  else
  {
    Quant::quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
  }
}

inline cost_t QuantRDOQ2::xiGetICost(int iRate ) const
{
  return (cost_t)(m_dLambda * iRate);
}

inline cost_t QuantRDOQ2::xGetIEPRate() const
{
  return 1 << SCALE_BITS;
}

/** Calculates the cost for specific absolute transform level
* \param uiAbsLevel scaled quantized level
* \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
* \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
* \param ui16AbsGoRice Rice parameter for coeff_abs_level_minus3
* \returns cost of given absolute transform level
*/
inline cost_t QuantRDOQ2::xiGetICRateCost( const uint32_t     uiAbsLevel,
                                            const BinFracBits& fracBitsPar,
                                            const BinFracBits& fracBitsGt1,
                                            const BinFracBits& fracBitsGt2,
                                            const int          remRegBins,
                                            unsigned           goRiceZero,
                                            const uint16_t     ui16AbsGoRice,
                                            const int          maxLog2TrDynamicRange ) const
{
  cost_t iRate = xGetIEPRate();
  if( remRegBins < 4 )
  {
    uint32_t  symbol  = ( uiAbsLevel == 0 ? goRiceZero : uiAbsLevel <= goRiceZero ? uiAbsLevel-1 : uiAbsLevel );
    uint32_t  length;
    const int threshold = COEF_REMAIN_BIN_REDUCTION;
    if( symbol < ( threshold << ui16AbsGoRice ) )
    {
      length = symbol >> ui16AbsGoRice;
      iRate += ( length + 1 + ui16AbsGoRice ) << SCALE_BITS;
    }
    else
    {
      length = ui16AbsGoRice;
      symbol = symbol - ( threshold << ui16AbsGoRice );
      while( symbol >= ( 1 << length ) )
      {
        symbol -= ( 1 << ( length++ ) );
      }
      iRate += ( threshold + length + 1 - ui16AbsGoRice + length ) << SCALE_BITS;
    }
  }
  else
  {
    const uint32_t cthres = 4;
    if( uiAbsLevel >= cthres )
    {
      uint32_t symbol = ( uiAbsLevel - cthres ) >> 1;
      uint32_t length;
      const int threshold = COEF_REMAIN_BIN_REDUCTION;
      if( symbol < ( threshold << ui16AbsGoRice ) )
      {
        length = symbol >> ui16AbsGoRice;
        iRate += ( length + 1 + ui16AbsGoRice ) << SCALE_BITS;
      }
      else
      {
        length = ui16AbsGoRice;
        symbol = symbol - ( threshold << ui16AbsGoRice );
        while( symbol >= ( 1 << length ) )
        {
          symbol -= ( 1 << ( length++ ) );
        }
        iRate += ( threshold + length + 1 - ui16AbsGoRice + length ) << SCALE_BITS;
      }

      iRate += fracBitsGt1.intBits[1];
      iRate += fracBitsPar.intBits[( uiAbsLevel - 2 ) & 1];
      iRate += fracBitsGt2.intBits[1];
    }
    else if( uiAbsLevel == 1 )
    {
      iRate += fracBitsGt1.intBits[0];
    }
    else if( uiAbsLevel == 2 )
    {
      iRate += fracBitsGt1.intBits[1];
      iRate += fracBitsPar.intBits[0];
      iRate += fracBitsGt2.intBits[0];
    }
    else if( uiAbsLevel == 3 )
    {
      iRate += fracBitsGt1.intBits[1];
      iRate += fracBitsPar.intBits[1];
      iRate += fracBitsGt2.intBits[0];
    }
    else
    {
      iRate = 0;
    }
  }
  return xiGetICost( (int)iRate );
}

inline cost_t QuantRDOQ2::xiGetCostSigCoeffGroup( const BinFracBits& fracBitsSigCG, unsigned uiSignificanceCoeffGroup ) const
{
  return xiGetICost( fracBitsSigCG.intBits[uiSignificanceCoeffGroup] );
}

void QuantRDOQ2::xInitLastPosBitsTab( const CoeffCodingContext& cctx, const uint32_t uiWidth, const uint32_t uiHeight, const ChannelType chType, const FracBitsAccess& fracBits )
{
  int dim1 = std::min<int>(JVET_C0024_ZERO_OUT_TH, uiWidth);
  int dim2 = std::min<int>(JVET_C0024_ZERO_OUT_TH, uiHeight);

  int bitsX = 0;
  int bitsY = 0;
  int ctxId;

  //X-coordinate
  for( ctxId = 0; ctxId < g_uiGroupIdx[dim1 - 1]; ctxId++ )
  {
    const BinFracBits fB = fracBits.getFracBitsArray( cctx.lastXCtxId( ctxId ) );
    m_lastBitsX[chType][ctxId] = bitsX + fB.intBits[0];
    bitsX += fB.intBits[1];
  }
  m_lastBitsX[chType][ctxId] = bitsX;

  //Y-coordinate
  for( ctxId = 0; ctxId < g_uiGroupIdx[dim2 - 1]; ctxId++ )
  {
    const BinFracBits fB = fracBits.getFracBitsArray( cctx.lastYCtxId( ctxId ) );
    m_lastBitsY[chType][ctxId] = bitsY + fB.intBits[0];
    bitsY += fB.intBits[1];
  }
  m_lastBitsY[chType][ctxId] = bitsY;
}


/** Calculates the cost of signaling the last significant coefficient in the block
* \param uiPosX X coordinate of the last significant coefficient
* \param uiPosY Y coordinate of the last significant coefficient
* \returns cost of last significant coefficient
*/
/*
* \param uiWidth width of the transform unit (TU)
*/
inline cost_t QuantRDOQ2::xiGetCostLast( const uint32_t uiPosX, const uint32_t uiPosY, const ChannelType chType ) const
{
  uint32_t uiCtxX = g_uiGroupIdx[uiPosX];
  uint32_t uiCtxY = g_uiGroupIdx[uiPosY];

  uint32_t uiCost = m_lastBitsX[chType][uiCtxX] + m_lastBitsY[chType][uiCtxY];

  if( uiCtxX > 3 )
  {
    uiCost += xGetIEPRate() * ( ( uiCtxX - 2 ) >> 1 );
  }
  if( uiCtxY > 3 )
  {
    uiCost += xGetIEPRate() * ( ( uiCtxY - 2 ) >> 1 );
  }
  return xiGetICost( (int)uiCost );
}

inline cost_t QuantRDOQ2::xiGetCostSigCoef( const BinFracBits& fracBitsSig, unsigned uiSignificance ) const
{
  return xiGetICost( fracBitsSig.intBits[uiSignificance] );
}

static inline cost_t _dist( cost_t iErr, cost_t iErrScale, int64_t iErrScaleShift )
{
  int64_t iSqrtErrCost = ( iErr*iErrScale ) >> iErrScaleShift;
  int64_t iDist = iSqrtErrCost*iSqrtErrCost;
  return iDist;
}

template< bool bSBH, bool bUseScalingList >
int QuantRDOQ2::xRateDistOptQuantFast( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx &ctx )
{
  CoeffCodingContext cctx( tu, compID, bSBH, false, m_tplBuf );
  const FracBitsAccess& fracBits = ctx.getFracBitsAcess();

  const SPS &sps            = *tu.cs->sps;
  const CompArea &rect      = tu.blocks[compID];
  const uint32_t uiWidth    = rect.width;
  const uint32_t uiHeight   = rect.height;
  const ChannelType chType  = toChannelType( compID );
  const int channelBitDepth = sps.bitDepths[ chType ];

  const int  maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(chType);

  if( compID != COMP_Cr || !tu.cbf[COMP_Cb] )
    xInitLastPosBitsTab( cctx, uiWidth, uiHeight, chType, fracBits );

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
  * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
  * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
  * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
  */

  // Represents scaling through forward transform
  const int iTransformShift = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

  const uint32_t uiLog2BlockWidth   = Log2(uiWidth);
  const uint32_t uiLog2BlockHeight  = Log2(uiHeight);
  const uint32_t uiMaxNumCoeff      = uiWidth * uiHeight;
  const uint32_t log2CGSize         = cctx.log2CGSize();

  int scalingListType = getScalingListType( tu.cu->predMode, compID );
  CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");

  const TCoeff    *plSrcCoeff   = pSrc.buf;
        TCoeffSig *piDstCoeff   = tu.getCoeffs( compID ).buf;

  memset( piDstCoeff, 0, sizeof( *piDstCoeff ) * uiMaxNumCoeff );

  const bool needSqrtAdjustment = TU::needsSqrt2Scale( tu, compID );
  const bool isTransformSkip    = tu.mtsIdx[compID] == MTS_SKIP;
  const int  *quantScaleList    = getQuantCoeff( scalingListType, cQP.rem( isTransformSkip ), uiLog2BlockWidth, uiLog2BlockHeight );
  const int  defaultQuantScale  = g_quantScales[ needSqrtAdjustment ?1:0][cQP.rem( isTransformSkip )];
  const int  defaultErrScale    = xGetErrScaleCoeffNoScalingList( scalingListType, uiLog2BlockWidth, uiLog2BlockHeight, cQP.rem( isTransformSkip ) );
  const int  *piErrScale        = xGetErrScaleCoeffSL           ( scalingListType, uiLog2BlockWidth, uiLog2BlockHeight, cQP.rem( isTransformSkip ) );
  const int  iErrScaleShift     = COEFF_ERR_SCALE_PRECISION_BITS;
  int iQBits                    = QUANT_SHIFT + cQP.per( isTransformSkip ) + iTransformShift + (needSqrtAdjustment?-1:0);    // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits
  int iQOffset                  = 1 << ( iQBits - 1 );

  cost_t piCostCoeff   [16];
  cost_t piCostSig     [16];
  cost_t piCostCoeff0  [16];
  cost_t piCostDeltaSBH[16];
  int    piAddSBH      [16];

  cost_t iCodedCostBlock   = 0;
  cost_t iUncodedCostBlock = 0;
  int    iLastScanPos      = -1;
  int    lastSubSetId      = -1;
  bool   lastOptFinished   = false;
  cost_t bestTotalCost  = std::numeric_limits<cost_t>::max() / 2;

  int ctxBinSampleRatio = MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT;
  int remRegBins = ( tu.getTbAreaAfterCoefZeroOut( compID ) * ctxBinSampleRatio ) >> 4;
  uint32_t  goRiceParam   = 0;

#if ENABLE_TRACING
  bool  bFirstNZSeen = false;
  DTRACE( g_trace_ctx, D_RDOQ, "%d: %3d, %3d, %dx%d, comp=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ ), rect.x, rect.y, rect.width, rect.height, compID );
#endif

  uiAbsSum = 0;

  const int iCGSize   = 1 << log2CGSize;
  const int iCGSizeM1 = iCGSize - 1;

  const uint32_t lfnstIdx = tu.cu->lfnstIdx;
  const int iCGNum   = lfnstIdx > 0 ? 1 : std::min<int>(JVET_C0024_ZERO_OUT_TH, uiWidth) * std::min<int>(JVET_C0024_ZERO_OUT_TH, uiHeight) >> cctx.log2CGSize();
  int       iScanPos = ( iCGNum << log2CGSize ) - 1;

  if( lfnstIdx > 0 && ( ( uiWidth == 4 && uiHeight == 4 ) || ( uiWidth == 8 && uiHeight == 8 ) ) )
  {
    iScanPos = 7;
  }

  // Find first non-zero coeff
  for( ; iScanPos > 0; iScanPos-- )
  {
    uint32_t uiBlkPos = cctx.blockPos( iScanPos );
    if( plSrcCoeff[uiBlkPos] )
      break;
  }

  //////////////////////////////////////////////////////////////////////////
  //  Loop over sub-sets (coefficient groups)
  //////////////////////////////////////////////////////////////////////////
  
  TCoeff thres = 0, useThres = 0;
  
  if( iQBits )
    thres = TCoeff( ( int64_t( m_thrVal ) << ( iQBits - 1 ) ) );
  else
    thres = TCoeff( ( int64_t( m_thrVal >> 1 ) << iQBits ) );

  if( !bUseScalingList )
  {
    useThres = thres / ( defaultQuantScale << 2 );
  }

  const bool scanFirstBlk = !bUseScalingList && log2CGSize == 4 && cctx.log2CGWidth() == 2;
#if ENABLE_SIMD_OPT_QUANT && defined( TARGET_SIMD_X86 )
  const bool isSimd       = read_x86_extension_flags() > x86_simd::SCALAR;
#endif

  int subSetId = iScanPos >> log2CGSize;
  for( ; subSetId >= 0; subSetId-- )
  {
    int    iNZbeforePos0  = 0;
    int    uiAbsSumCG     = 0;
    cost_t iCodedCostCG   = 0;
    cost_t iUncodedCostCG = 0;

    int iScanPosinCG = iScanPos & ( iCGSize - 1 );
    if( iLastScanPos < 0 )
    {
#if ENABLE_SIMD_OPT_QUANT && defined( TARGET_SIMD_X86 )
      // if more than one 4x4 coding subblock is available, use SIMD to find first subblock with coefficient larger than threshold
      if( scanFirstBlk && iScanPos >= 16 && isSimd )
      {
        // move the pointer to the beginning of the current subblock
        const int firstTestPos  = iScanPos - iScanPosinCG;
        uint32_t  uiBlkPos      = cctx.blockPos( firstTestPos );

        const __m128i xdfTh = _mm_set1_epi32( useThres );

        // read first line of the subblock and check for coefficients larger than the threshold
        // assumming the subblocks are dense 4x4 blocks in raster scan order with the stride of tuPars.m_width
        __m128i xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &plSrcCoeff[uiBlkPos] ) );
        __m128i xdf = _mm_cmpgt_epi32( xl0, xdfTh );

        // same for the next line in the subblock
        uiBlkPos += uiWidth;
        xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &plSrcCoeff[uiBlkPos] ) );
        xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

        // and the third line
        uiBlkPos += uiWidth;
        xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &plSrcCoeff[uiBlkPos] ) );
        xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

        // and the last line
        uiBlkPos += uiWidth;
        xl0 = _mm_abs_epi32( _mm_loadu_si128( ( const __m128i* ) &plSrcCoeff[uiBlkPos] ) );
        xdf = _mm_or_si128( xdf, _mm_cmpgt_epi32( xl0, xdfTh ) );

        if( _mm_testz_si128( xdf, xdf ) )
        {
          iScanPos    -= iScanPosinCG + 1;
          iScanPosinCG = -1;
          continue;
        }
      }
      else
#endif
      if( scanFirstBlk && iScanPos >= 16 )
      {
        bool allSmaller = true;

        for( int xScanPosinCG = iScanPosinCG, xScanPos = iScanPos; allSmaller && xScanPosinCG >= 0; xScanPosinCG--, xScanPos-- )
        {
          const uint32_t uiBlkPos = cctx.blockPos( xScanPos );
          allSmaller &= std::abs( plSrcCoeff[uiBlkPos] ) <= useThres;
        }

        if( allSmaller )
        {
          iScanPos    -= iScanPosinCG + 1;
          iScanPosinCG = -1;
          continue;
        }
      }

    findlast2:
      // Fast loop to find last-pos.
      // No need to add distortion to cost as it would be added to both the coded and uncoded cost
      for( ; iScanPosinCG >= 0; iScanPosinCG--, iScanPos-- )
      {
        const uint32_t uiBlkPos = cctx.blockPos( iScanPos );

        //===== quantization =====
        int quantScale;
        if( bUseScalingList ){ quantScale = quantScaleList[uiBlkPos]; }
        else{                  quantScale = defaultQuantScale; }
        
        const uint32_t uiMaxAbsLevel = ( std::abs( plSrcCoeff[uiBlkPos] ) * quantScale + iQOffset ) >> iQBits;

        if( uiMaxAbsLevel )
        {
          iLastScanPos = iScanPos;
          lastSubSetId = subSetId;
          break;
        }
#if ENABLE_TRACING
        if( bFirstNZSeen )
        {
          DTRACE( g_trace_ctx, D_RDOQ, "%d [%d][%d][%2d:%2d][%2d:%2d]", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ ), iScanPos, cctx.blockPos( iScanPos ), cctx.cgPosX(), cctx.cgPosY(), cctx.posX( iScanPos ), cctx.posY( iScanPos ) );
          DTRACE( g_trace_ctx, D_RDOQ, " remRegBins=%d \n", remRegBins );
          DTRACE( g_trace_ctx, D_RDOQ, " Lev=%d \n", 0 );
        }
#endif
      }
    }

    //////////////////////////////////////////////////////////////////////////
    //  Loop over coefficients
    //////////////////////////////////////////////////////////////////////////

    cctx.initSubblock( subSetId );

    for( ; iScanPosinCG >= 0; iScanPosinCG--, iScanPos-- )
    {
      const uint32_t uiBlkPos = cctx.blockPos( iScanPos );
      int quantScale;
      int iErrScale;
      //===== quantization =====
      if( bUseScalingList ){
        quantScale = quantScaleList[uiBlkPos];
        iErrScale  = piErrScale[uiBlkPos];
      }
      else{
        quantScale = defaultQuantScale;
        iErrScale  = defaultErrScale;
      }
      const int iScaledLevel = std::abs( plSrcCoeff[uiBlkPos] ) * quantScale;
      const int iAbsLevel    = ( iScaledLevel + iQOffset ) >> iQBits;

      //============ Set context models ===============
      unsigned ctxIdSig = 0;

      if( iScanPos != iLastScanPos )
      {
        ctxIdSig = cctx.sigCtxIdAbsWithAcc( iScanPos, 0 );
      }
      uint8_t     ctxOffset     = cctx.ctxOffsetAbs();
      uint32_t    uiParCtx      = cctx.parityCtxIdAbs   ( ctxOffset );
      uint32_t    uiGt1Ctx      = cctx.greater1CtxIdAbs ( ctxOffset );
      uint32_t    uiGt2Ctx      = cctx.greater2CtxIdAbs ( ctxOffset );
      uint32_t    goRiceZero    = 0;

      const BinFracBits& fracBitsPar = fracBits.getFracBitsArray( uiParCtx );
      const BinFracBits& fracBitsGt1 = fracBits.getFracBitsArray( uiGt1Ctx );
      const BinFracBits& fracBitsGt2 = fracBits.getFracBitsArray( uiGt2Ctx );

      if( remRegBins < 4 )
      {
        unsigned  sumAbs = cctx.templateAbsSum( iScanPos, piDstCoeff, 0 );
        goRiceParam      = g_auiGoRiceParsCoeff   [ sumAbs ];
        goRiceZero       = g_auiGoRicePosCoeff0(0, goRiceParam);
      }

#if ENABLE_TRACING
      DTRACE( g_trace_ctx, D_RDOQ, "%d [%d][%d][%2d:%2d][%2d:%2d]", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ ), iScanPos, cctx.blockPos( iScanPos ), cctx.cgPosX(), cctx.cgPosY(), cctx.posX( iScanPos ), cctx.posY( iScanPos ) );
      DTRACE( g_trace_ctx, D_RDOQ, " remRegBins=%d \n", remRegBins );
      bFirstNZSeen = true;
#endif

      // Cost for zero coeff
      piCostCoeff0[iScanPosinCG] = _dist( iScaledLevel, iErrScale, iErrScaleShift );

      uint32_t uiLevel = 0;
      if( iAbsLevel == 0 )
      {
        // ----------------- ABS LEVEL 0 ----------------
        const BinFracBits fracBitsSig = fracBits.getFracBitsArray( ctxIdSig );
        piCostSig  [iScanPosinCG] = xiGetCostSigCoef( fracBitsSig, 0 );
        piCostCoeff[iScanPosinCG] = piCostCoeff0[iScanPosinCG] + piCostSig[iScanPosinCG];

        if( bSBH )
        {
          cost_t iErr1        = iScaledLevel - ( (int64_t)1 << iQBits );
          cost_t iDist1       = _dist( iErr1, iErrScale, iErrScaleShift );
          cost_t iRate1       = remRegBins < 4 ? 
                                 xiGetICRateCost( 1, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, maxLog2TrDynamicRange ) -
                                 xiGetICRateCost( 0, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, maxLog2TrDynamicRange ):
                                 fracBitsGt1.intBits[ 0 ];

          cost_t iCost1       = iDist1 + iRate1 + xiGetCostSigCoef( fracBitsSig, 1 );

          piCostDeltaSBH[iScanPosinCG] = iCost1 - piCostCoeff[iScanPosinCG];
          piAddSBH      [iScanPosinCG] = 1;
        }
        DTRACE( g_trace_ctx, D_RDOQ, " Lev=%d \n", 0 );
      }
      else
      {
        //===== coefficient level estimation =====
        const int iFloor = (int)( iScaledLevel >> iQBits );
        const int iCeil  = iFloor + 1;

        if( remRegBins >= 4 && iScanPos != iLastScanPos && iCeil >= 4 )
        {
          int  sumAll = cctx.templateAbsSum( iScanPos, piDstCoeff, 4 );
          goRiceParam = g_auiGoRiceParsCoeff[ sumAll ];
        }

        if( iScanPos == iLastScanPos )
        {
          // =======================             =======================
          // ======================= LAST LEVEL  =======================
          // =======================             =======================
          piCostSig[ iScanPosinCG ] = 0;
          // Floor = 0, Uncoded
          cost_t iCurrCostF = piCostCoeff0[ iScanPosinCG ];

          if( iFloor )
          {
            // ----------------- LEVEL > 0  ----------------
            cost_t iErrF       = iScaledLevel - (iFloor << iQBits);
            cost_t iDistF      = _dist( iErrF, iErrScale, iErrScaleShift ); //(iErrF*iErrScale) >> iErrScaleShift;
            iCurrCostF         = iDistF + xiGetICRateCost( iFloor, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, maxLog2TrDynamicRange );
          }

          // ----------------- LEVEL + 1 ----------------
          cost_t iErrC         = iScaledLevel - (iCeil << iQBits);
          cost_t iDistC        = _dist( iErrC, iErrScale, iErrScaleShift ); //(iErrC*iErrScale) >> iErrScaleShift;
          cost_t iCurrCostC    = iDistC + xiGetICRateCost( iCeil, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, maxLog2TrDynamicRange );

          if( iCurrCostC < iCurrCostF )
          {
            uiLevel                   = iCeil;
            piCostCoeff[iScanPosinCG] = iCurrCostC;
            if( bSBH ){
              piCostDeltaSBH[iScanPosinCG] = iCurrCostF - iCurrCostC;
              piAddSBH      [iScanPosinCG] = -1;
            }
          }
          else
          {
            if( iFloor == 0 )
            {
              DTRACE( g_trace_ctx, D_RDOQ, " Lev=%d \n", 0 );
              DTRACE( g_trace_ctx, D_RDOQ, " CostC0=%lld\n", (int64_t)piCostCoeff0[iScanPosinCG] );
              DTRACE( g_trace_ctx, D_RDOQ, " CostC =%lld\n", (int64_t)iCurrCostC        );

              iLastScanPos = -1;
              lastSubSetId = -1;
              iScanPos--;
              iScanPosinCG--;
              goto findlast2;
            }
            uiLevel = iFloor;
            piCostCoeff[iScanPosinCG] = iCurrCostF;
            if( bSBH ){
              piCostDeltaSBH[iScanPosinCG] = iCurrCostC - iCurrCostF;
              piAddSBH      [iScanPosinCG] = 1;
            }
          }
        }
        else
        {
          const BinFracBits& fracBitsSig = fracBits.getFracBitsArray( ctxIdSig );
          cost_t iCostSig1 = xiGetCostSigCoef( fracBitsSig, 1 );
          if( iCeil < 3 )
          {
            // =======================                 =======================
            // ======================= LEVELS 0, 1, 2  =======================
            // =======================                 =======================
            
            // ----------------- BEST LEVEL = 0 ----------------
            cost_t iCostSig0    = xiGetCostSigCoef( fracBitsSig, 0 );
            cost_t iBestCost    = piCostCoeff0[iScanPosinCG] + iCostSig0;
            cost_t iBestCostSig = iCostSig0;
            cost_t iCostF       = iBestCost;
            uiLevel = 0;

            if( iFloor == 1 )
            {
              // ----------------- LEVEL = 1 ----------------
              cost_t iErrF      = iScaledLevel - ( iFloor << iQBits );
              cost_t iDistF     = _dist( iErrF, iErrScale, iErrScaleShift ); //( iErrF*iErrScale ) >> iErrScaleShift;
              iCostF            = iDistF + iCostSig1 + xiGetICRateCost( iFloor, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, maxLog2TrDynamicRange );

              if( iCostF < iBestCost )
              {
                uiLevel      = iFloor;
                iBestCost    = iCostF;
                iBestCostSig = iCostSig1;
                if( bSBH )
                {
                  piCostDeltaSBH[iScanPosinCG] = iBestCost - iCostF;
                  piAddSBH      [iScanPosinCG] = -1;
                }
              }
              else
              {
                if( bSBH )
                {
                  piCostDeltaSBH[iScanPosinCG] = iCostF - iBestCost;
                  piAddSBH      [iScanPosinCG] = 1;
                }
              }
            }

            // ----------------- LEVELS = 1, 2 ----------------
            cost_t iErrC         = iScaledLevel - ( iCeil << iQBits );
            cost_t iDistC        = _dist( iErrC, iErrScale, iErrScaleShift ); //( iErrC*iErrScale ) >> iErrScaleShift;
            cost_t iCostC        = iDistC + iCostSig1 + xiGetICRateCost( iCeil, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, maxLog2TrDynamicRange );

            if( iCostC < iBestCost )
            {
              uiLevel                   = iCeil;
              piCostCoeff[iScanPosinCG] = iCostC;
              piCostSig[iScanPosinCG]   = iCostSig1;
              if( bSBH )
              {
                piCostDeltaSBH[iScanPosinCG] = iCostF - iCostC;
                piAddSBH[iScanPosinCG]       = -1;
              }
            }
            else
            {
              piCostCoeff[iScanPosinCG] = iBestCost;
              piCostSig[iScanPosinCG] = iBestCostSig;
              if( bSBH )
              {
                piCostDeltaSBH[iScanPosinCG] = iCostC - iCostF;
                piAddSBH      [iScanPosinCG] = 1;
              }
            }
          }
          else
          {
            // ----------------- LEVEL X, X+1 ----------------
            cost_t iErrF        = iScaledLevel - (iFloor << iQBits);
            cost_t iDistF       = _dist( iErrF, iErrScale, iErrScaleShift ); //(iErrF*iErrScale) >> iErrScaleShift;
            cost_t iCostF       = iDistF + iCostSig1 + xiGetICRateCost( iFloor, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, maxLog2TrDynamicRange );

            cost_t iErrC        = iScaledLevel - ( iCeil << iQBits );
            cost_t iDistC       = _dist( iErrC, iErrScale, iErrScaleShift ); //( iErrC*iErrScale ) >> iErrScaleShift;
            cost_t iCostC       = iDistC + iCostSig1 + xiGetICRateCost( iCeil, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, maxLog2TrDynamicRange );

            piCostSig[iScanPosinCG] = iCostSig1;
            if( iCostC < iCostF )
            {
              uiLevel = iCeil;
              piCostCoeff[iScanPosinCG] = iCostC;
              if( bSBH )
              {
                piCostDeltaSBH[iScanPosinCG] = iCostF - iCostC;
                piAddSBH[iScanPosinCG]       = -1;
              }
            }
            else
            {
              uiLevel = iFloor;
              piCostCoeff[iScanPosinCG] = iCostF;
              if( bSBH )
              {
                piCostDeltaSBH[iScanPosinCG] = iCostC - iCostF;
                piAddSBH[iScanPosinCG] = 1;
              }
            }
          }
        }
        piDstCoeff[uiBlkPos] = uiLevel;
        DTRACE( g_trace_ctx, D_RDOQ, " Lev=%d \n", uiLevel );
        DTRACE( g_trace_ctx, D_RDOQ, " CostC0=%lld\n", (int64_t)piCostCoeff0[iScanPosinCG] );
        DTRACE( g_trace_ctx, D_RDOQ, " CostC =%lld\n", (int64_t)piCostCoeff [iScanPosinCG] );
        if( uiLevel )
        {
          uiAbsSumCG    += uiLevel;
          iNZbeforePos0 += iScanPosinCG; // hack-> just add instead of checking iScanPosinCG >0 and increment
          cctx.absVal1stPass( iScanPos, std::min<TCoeff>( 4 + ( uiLevel & 1 ), uiLevel ) );
          cctx.setSigGroup();
        }
      }


      if( ( (iScanPos & iCGSizeM1) == 0 ) && ( iScanPos > 0 ) )
      {
        goRiceParam   = 0;
      }
      else if( remRegBins >= 4 )
      {
        remRegBins -= (uiLevel < 2 ? uiLevel : 3) + (iScanPos != iLastScanPos);
      }

      iUncodedCostCG += piCostCoeff0[iScanPosinCG];
      iCodedCostCG   += piCostCoeff[iScanPosinCG];
      DTRACE( g_trace_ctx, D_RDOQ_MORE, "Uncoded=%lld\n", (long long)( iUncodedCostBlock + iUncodedCostCG ) );
      DTRACE( g_trace_ctx, D_RDOQ_MORE, "Coded  =%lld\n", (long long)( iCodedCostBlock   + iCodedCostCG   ) );
    } // for (iScanPosinCG)

    //================== Group sig. flag ===================
    cost_t iCostCoeffGroupSig = 0;
    if( lastSubSetId >= 0 )
    {
      if( subSetId )
      {
        const BinFracBits fracBitsSigGroup = fracBits.getFracBitsArray( cctx.sigGroupCtxId() );
        cost_t iCostCoeffGroupSig0 = xiGetCostSigCoeffGroup( fracBitsSigGroup, 0 );

        // if no coeff in CG
        if( !cctx.isSigGroup() )
        {
          iCodedCostCG = iUncodedCostCG + iCostCoeffGroupSig0;
          iCostCoeffGroupSig  = iCostCoeffGroupSig0;
        }
        else
        {
          // if not topleft CG
          if( subSetId < lastSubSetId )
          {
            cost_t iCostCoeffGroupSig1 = xiGetCostSigCoeffGroup( fracBitsSigGroup, 1 );
            iCostCoeffGroupSig = iCostCoeffGroupSig1;

            // if only one coeff in CG
            if( !iNZbeforePos0 ) {
              iCodedCostCG -= piCostSig[0];
            }
            cost_t iUncodedCostCGTmp = iUncodedCostCG + iCostCoeffGroupSig0;
            iCodedCostCG += iCostCoeffGroupSig1;

            // if we can save cost, change this block to all-zero block
            if( iUncodedCostCGTmp < iCodedCostCG )
            {
              cctx.resetSigGroup();
              iCodedCostCG = iUncodedCostCGTmp;
              iCostCoeffGroupSig = iCostCoeffGroupSig0;

              // reset coeffs to 0 in this block
              for( iScanPosinCG = iCGSize - 1; iScanPosinCG >= 0; iScanPosinCG-- )
              {
                int iScanPosTmp = subSetId * iCGSize + iScanPosinCG;
                uint32_t uiBlkPos = cctx.blockPos( iScanPosTmp );
                if( piDstCoeff[uiBlkPos] )
                {
                  int absLevel = std::abs( piDstCoeff[uiBlkPos] );
                  cctx.remAbsVal1stPass( iScanPosTmp, std::min( absLevel, 4 + ( absLevel & 1 ) ) );
                  piDstCoeff[uiBlkPos] = 0;
                }
              }
              uiAbsSumCG = 0;
              if( lastSubSetId == subSetId ) {
                iCodedCostCG   = 0;
                iUncodedCostCG = 0;
                iLastScanPos   = -1;
                lastSubSetId   = -1;
              }
            }
          }
          else
          {
            cctx.setSigGroup();
          }
        }
      }
    }

    //===== estimate last position cost =====
    bestTotalCost += iCodedCostCG;
    if( !lastOptFinished )
    {
      if( cctx.isSigGroup( subSetId ) )
      {
        cost_t codedCostBlockTmp = iUncodedCostBlock + iCodedCostCG - iCostCoeffGroupSig;
        int startPosInCG  = subSetId == lastSubSetId ? iLastScanPos % iCGSize: iCGSizeM1;
        int newAbsSumCG   = uiAbsSumCG;
        int bestLastIdxP1 = iLastScanPos + 1;
        for( int iScanPosinCGTmp = startPosInCG; iScanPosinCGTmp >= 0; iScanPosinCGTmp-- )
        {
          uint32_t iScanPosTmp = ( subSetId << log2CGSize ) + iScanPosinCGTmp;
          uint32_t uiBlkPos    = cctx.blockPos( iScanPosTmp );

          if( piDstCoeff[uiBlkPos] )
          {
            uint32_t  uiPosY = uiBlkPos >> uiLog2BlockWidth;
            uint32_t  uiPosX = uiBlkPos - (uiPosY << uiLog2BlockWidth);
            const cost_t iCostLast = xiGetCostLast( uiPosX, uiPosY, chType );
            const cost_t totalCost = codedCostBlockTmp + iCostLast - piCostSig[iScanPosinCGTmp];

            if( totalCost < bestTotalCost )
            {
              bestLastIdxP1 = iScanPosTmp + 1;
              bestTotalCost = totalCost;
              lastSubSetId  = subSetId;
              uiAbsSumCG    = newAbsSumCG;
              uiAbsSum      = 0;
            }

            if( piDstCoeff[uiBlkPos] > 1 )
            {
              lastOptFinished = true;
              break;
            }
            newAbsSumCG -= 1;
            codedCostBlockTmp -= piCostCoeff [ iScanPosinCGTmp ];
            codedCostBlockTmp += piCostCoeff0[ iScanPosinCGTmp ];
          }
          else
          {
            codedCostBlockTmp -= piCostSig[ iScanPosinCGTmp ];
          }
        } //end for
        for( int iScanPosTmp = bestLastIdxP1; iScanPosTmp <= iLastScanPos; iScanPosTmp++ )
        {
          const int uiBlkPos = cctx.blockPos( iScanPosTmp );
          if( piDstCoeff[uiBlkPos] )
          {
            int absLevel = std::abs( piDstCoeff[uiBlkPos] );
            cctx.remAbsVal1stPass( iScanPosTmp, std::min( absLevel, 4 + ( absLevel & 1 ) ) );
            piDstCoeff[uiBlkPos] = 0;
          }
        }
        iLastScanPos = bestLastIdxP1 - 1;
      }
    }

    //=============== estimate Sign Bit Hiding ================
    if( bSBH )
    {
      if( uiAbsSumCG >= 2 /*&& cctx.isSigGroup()*/ )
      {
        int iSubPos         = subSetId*iCGSize;
        int iLastNZPosInCG  = -1;
        int iFirstNZPosInCG = iCGSize;

        for( int n = 0; n <iCGSize; n++ ) {
          if( piDstCoeff[ cctx.blockPos( n + iSubPos ) ] ) {
            iFirstNZPosInCG = n;
            break;
          }
        }
        if( lastSubSetId == subSetId ){
          iLastNZPosInCG = ( iLastScanPos )%iCGSize;
          if( piDstCoeff[ cctx.blockPos( iLastScanPos ) ] == 1 && ( piAddSBH[iLastNZPosInCG] == -1 ) )
          {
            piCostDeltaSBH[iLastNZPosInCG] -= (4<<SCALE_BITS);
          }
        }
        else{
          for( int n = iCGSize - 1; n >= 0; n-- ) {
            if( piDstCoeff[ cctx.blockPos( n + iSubPos ) ] ) {
              iLastNZPosInCG = n;
              break;
            }
          }
        }
        if( iLastNZPosInCG - iFirstNZPosInCG >= SBH_THRESHOLD )
        {
          iCodedCostCG -= xiGetICost( (int)xGetIEPRate() ); //subtract cost for one sign bin
          bool bSign    = plSrcCoeff[ cctx.blockPos( iSubPos + iFirstNZPosInCG) ] < 0;

          if( bSign != ( uiAbsSumCG & 0x1 ) ) {
            int iLastPosInCG    = ( lastSubSetId == subSetId ) ? iLastNZPosInCG : iCGSize - 1;
            int64_t iMinCostDelta = std::numeric_limits<int64_t>::max();
            int iMinCostPos     = -1;

            if( piDstCoeff[ cctx.blockPos( iFirstNZPosInCG + iSubPos ) ] >1 ){
              iMinCostDelta = piCostDeltaSBH[iFirstNZPosInCG];
              iMinCostPos   = iFirstNZPosInCG;
            }

            for( int n = 0; n<iFirstNZPosInCG; n++ ){
              if( ( plSrcCoeff[ cctx.blockPos( iSubPos + n ) ] < 0 ) == bSign ){
                if( piCostDeltaSBH[n] < iMinCostDelta ){
                  iMinCostDelta = piCostDeltaSBH[n];
                  iMinCostPos   = n;
                }
              }
            }

            for( int n = iFirstNZPosInCG + 1; n <= iLastPosInCG; n++ ){
              if( piCostDeltaSBH[n] < iMinCostDelta ){
                iMinCostDelta = piCostDeltaSBH[n];
                iMinCostPos   = n;
              }
            }
            const int oldAbsVal = std::abs( piDstCoeff[cctx.blockPos( iMinCostPos + iSubPos )] );
            if( oldAbsVal ) cctx.remAbsVal1stPass( iMinCostPos + iSubPos, std::min( oldAbsVal, 4 + ( oldAbsVal & 1 ) ) );
            piDstCoeff[ cctx.blockPos( iMinCostPos + iSubPos ) ] += piAddSBH[iMinCostPos];
            const int absVal = std::abs( piDstCoeff[cctx.blockPos( iMinCostPos + iSubPos )] );
            if( absVal ) cctx.absVal1stPass( iMinCostPos + iSubPos, std::min( absVal, 4 + ( absVal & 1 ) ) );
            uiAbsSumCG   += piAddSBH[iMinCostPos];
            iCodedCostCG += iMinCostDelta;
          }
        }
      }
    }

    iCodedCostBlock   += iCodedCostCG;
    iUncodedCostBlock += iUncodedCostCG;
    uiAbsSum += uiAbsSumCG;
    DTRACE( g_trace_ctx, D_RDOQ_COST, "%d: [%2d:%2d]\n", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ_COST ), cctx.cgPosX(), cctx.cgPosY() );
    DTRACE( g_trace_ctx, D_RDOQ_MORE, "Uncoded=%lld\n", (long long)( iUncodedCostBlock ) );
    DTRACE( g_trace_ctx, D_RDOQ_MORE, "Coded  =%lld\n", (long long)( iCodedCostBlock ) );
  } //end for (iCGScanPos)

  iCodedCostBlock = bestTotalCost;

  if( iLastScanPos < 0 )
  {
    CHECK( uiAbsSum != 0, "Illegal" );
    return 0;
  }

  if( !CU::isIntra( *tu.cu ) && isLuma( compID ) )
  {
    const BinFracBits fracBitsQtRootCbf = fracBits.getFracBitsArray( Ctx::QtRootCbf() );
    iUncodedCostBlock += xiGetICost( fracBitsQtRootCbf.intBits[0] );
    iCodedCostBlock   += xiGetICost( fracBitsQtRootCbf.intBits[1] );
  }
  else
  {
    bool previousCbf       = tu.cbf[COMP_Cb];
    bool lastCbfIsInferred = false;
    const bool useIntraSubPartitions = tu.cu->ispMode && isLuma(compID);
    if( useIntraSubPartitions )
    {
      bool rootCbfSoFar       = false;
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
        previousCbf = TU::getPrevTuCbfAtDepth(tu, compID, tu.depth);
      }
    }
    BinFracBits fracBitsQtCbf = fracBits.getFracBitsArray( Ctx::QtCbf[compID]( DeriveCtx::CtxQtCbf( rect.compID, previousCbf, useIntraSubPartitions ) ) );

    if( !lastCbfIsInferred )
    {
      iUncodedCostBlock += xiGetICost(fracBitsQtCbf.intBits[0]);
      iCodedCostBlock   += xiGetICost(fracBitsQtCbf.intBits[1]);
    }
  }

  if( iUncodedCostBlock <= iCodedCostBlock )
  {
    iCodedCostBlock = iUncodedCostBlock;
    uiAbsSum = 0;
    ::memset( piDstCoeff, 0, uiMaxNumCoeff*sizeof( TCoeffSig ) );
  }
  else
  {
    // Check due to saving of last pos. Sign data hiding can change the position of last coef.
    if( bSBH )
    {
      if( piDstCoeff[cctx.blockPos( iLastScanPos )] == 0 )
      {
        int scanPos = iLastScanPos - 1;
        for( ; scanPos >= 0; scanPos-- )
        {
          if( piDstCoeff[cctx.blockPos( scanPos )] )
            break;
        }
        iLastScanPos = scanPos;
      }
    }

    for ( int scanPos = 0; scanPos <= iLastScanPos; scanPos++ )
    {
      int blkPos = cctx.blockPos( scanPos );
      TCoeff level = piDstCoeff[ blkPos ];
      int iSign = plSrcCoeff[blkPos] >> ( sizeof(TCoeff)*8 - 1 );
      piDstCoeff[blkPos] = ( iSign^level ) - iSign;
    }
    tu.lastPos[compID] = iLastScanPos;
  }

#if ENABLE_TRACING
  for ( int scanPos = iCGNum * iCGSize-1; scanPos >= 0; scanPos-- )
  {
    if(( scanPos & iCGSizeM1) == iCGSizeM1 )
    {
      DTRACE(g_trace_ctx, D_RDOQ, "%d:", scanPos >> cctx.log2CGSize() );
    }
    int blkPos = cctx.blockPos( scanPos );
    DTRACE( g_trace_ctx, D_RDOQ, "%3d ", piDstCoeff[blkPos] );
    if( scanPos % iCGSize == 0 )
    {
      DTRACE(g_trace_ctx, D_RDOQ, "\n");
    }
  }
#endif

  DTRACE( g_trace_ctx, D_RDOQ_MORE, "Uncoded=%lld\n", (long long)( iUncodedCostBlock ) );
  DTRACE( g_trace_ctx, D_RDOQ_MORE, "Coded  =%lld\n", (long long)( iCodedCostBlock ) );
  DTRACE( g_trace_ctx, D_RDOQ, "%d: %3d, %3d, %dx%d, comp=%d, lastScanPos=%d, absSum=%d, cost=%lld \n", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ ), rect.x, rect.y, rect.width, rect.height, compID, iLastScanPos, uiAbsSum,  (long long)iCodedCostBlock );
  return 0;
}

int QuantRDOQ2::xRateDistOptQuant( TransformUnit &tu, const ComponentID &compID, const CCoeffBuf &pSrc, TCoeff &uiAbsSum, const QpParam &cQP, const Ctx &ctx, bool bUseScalingList )
{
  if( tu.cs->slice->signDataHidingEnabled/*m_bSBH*/ )
  {
    if( bUseScalingList ) return xRateDistOptQuantFast<true, true >( tu, compID, pSrc, uiAbsSum, cQP, ctx );
    else                  return xRateDistOptQuantFast<true, false>( tu, compID, pSrc, uiAbsSum, cQP, ctx );
  }
  else
  {
    if( bUseScalingList ) return xRateDistOptQuantFast<false, true >( tu, compID, pSrc, uiAbsSum, cQP, ctx );
    else                  return xRateDistOptQuantFast<false, false>( tu, compID, pSrc, uiAbsSum, cQP, ctx );
  }

}


} // namespace vvenc

//! \}

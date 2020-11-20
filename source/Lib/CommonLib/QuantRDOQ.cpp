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


/** \file     QuantRDOQ.cpp
    \brief    transform and quantization class
*/

#include "QuantRDOQ.h"
#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "dtrace_next.h"
#include "dtrace_buffer.h"

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
 int   iNumSbbCtxBins;
};


//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Constants
// ====================================================================================================================


// ====================================================================================================================
// Static functions
// ====================================================================================================================

// ====================================================================================================================
// QuantRDOQ class member functions
// ====================================================================================================================


QuantRDOQ::QuantRDOQ( const Quant* other ) : Quant( other )
{

  const QuantRDOQ *rdoq = dynamic_cast<const QuantRDOQ*>( other );
  CHECK( other && !rdoq, "The RDOQ cast must be successfull!" );
  xInitScalingList( rdoq );
}

QuantRDOQ::~QuantRDOQ()
{
  xDestroyScalingList();
}




/** Get the best level in RD sense
 *
 * \returns best quantized transform level for given scan position
 *
 * This method calculates the best quantized transform level for a given scan position.
 */
inline uint32_t QuantRDOQ::xGetCodedLevel( double&            rd64CodedCost,
                                       double&            rd64CodedCost0,
                                       double&            rd64CodedCostSig,
                                       Intermediate_Int   lLevelDouble,
                                       uint32_t               uiMaxAbsLevel,
                                       const BinFracBits* fracBitsSig,
                                       const BinFracBits& fracBitsPar,
                                       const BinFracBits& fracBitsGt1,
                                       const BinFracBits& fracBitsGt2,
                                       const int          remRegBins,
                                       unsigned           goRiceZero,
                                       uint16_t             ui16AbsGoRice,
                                       int                iQBits,
                                       double             errorScale,
                                       bool               bLast,
                                       bool               useLimitedPrefixLength,
                                       const int          maxLog2TrDynamicRange
                                     ) const
{
  double dCurrCostSig   = 0;
  uint32_t   uiBestAbsLevel = 0;

  if( !bLast && uiMaxAbsLevel < 3 )
  {
    rd64CodedCostSig    = xGetRateSigCoef( *fracBitsSig, 0 );
    rd64CodedCost       = rd64CodedCost0 + rd64CodedCostSig;
    if( uiMaxAbsLevel == 0 )
    {
      return uiBestAbsLevel;
    }
  }
  else
  {
    rd64CodedCost       = MAX_DOUBLE;
  }

  if( !bLast )
  {
    dCurrCostSig        = xGetRateSigCoef( *fracBitsSig, 1 );
  }

  uint32_t uiMinAbsLevel    = ( uiMaxAbsLevel > 1 ? uiMaxAbsLevel - 1 : 1 );
  for( int uiAbsLevel  = uiMaxAbsLevel; uiAbsLevel >= uiMinAbsLevel ; uiAbsLevel-- )
  {
    double dErr         = double( lLevelDouble  - ( Intermediate_Int(uiAbsLevel) << iQBits ) );

    double dCurrCost    = dErr * dErr * errorScale + xGetICost( xGetICRate( uiAbsLevel, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, ui16AbsGoRice, true, maxLog2TrDynamicRange ) );
    dCurrCost          += dCurrCostSig;

    if( dCurrCost < rd64CodedCost )
    {
      uiBestAbsLevel    = uiAbsLevel;
      rd64CodedCost     = dCurrCost;
      rd64CodedCostSig  = dCurrCostSig;
    }
  }

  return uiBestAbsLevel;
}

/** Calculates the cost for specific absolute transform level
 * \param uiAbsLevel scaled quantized level
 * \param ui16CtxNumOne current ctxInc for coeff_abs_level_greater1 (1st bin of coeff_abs_level_minus1 in AVC)
 * \param ui16CtxNumAbs current ctxInc for coeff_abs_level_greater2 (remaining bins of coeff_abs_level_minus1 in AVC)
 * \param ui16AbsGoRice Rice parameter for coeff_abs_level_minus3
 * \param c1Idx
 * \param c2Idx
 * \param useLimitedPrefixLength
 * \param maxLog2TrDynamicRange
 * \returns cost of given absolute transform level
 */
inline int QuantRDOQ::xGetICRate( const uint32_t         uiAbsLevel,
                                  const BinFracBits& fracBitsPar,
                                  const BinFracBits& fracBitsGt1,
                                  const BinFracBits& fracBitsGt2,
                                  const int          remRegBins,
                                  unsigned           goRiceZero,
                                  const uint16_t       ui16AbsGoRice,
                                  const bool         useLimitedPrefixLength,
                                  const int          maxLog2TrDynamicRange  ) const
{
  if( remRegBins < 4 )
  {
    int       iRate   = int( xGetIEPRate() ); // cost of sign bit
    uint32_t  symbol  = ( uiAbsLevel == 0 ? goRiceZero : uiAbsLevel <= goRiceZero ? uiAbsLevel-1 : uiAbsLevel );
    uint32_t  length;
    const int threshold = COEF_REMAIN_BIN_REDUCTION;
    if( symbol < ( threshold << ui16AbsGoRice ) )
    {
      length = symbol >> ui16AbsGoRice;
      iRate += ( length + 1 + ui16AbsGoRice ) << SCALE_BITS;
    }
    else if( useLimitedPrefixLength )
    {
      const uint32_t maximumPrefixLength = ( 32 - ( COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange ) );

      uint32_t prefixLength = 0;
      uint32_t suffix = ( symbol >> ui16AbsGoRice ) - COEF_REMAIN_BIN_REDUCTION;

      while( ( prefixLength < maximumPrefixLength ) && ( suffix > ( ( 2 << prefixLength ) - 2 ) ) )
      {
        prefixLength++;
      }

      const uint32_t suffixLength = ( prefixLength == maximumPrefixLength ) ? ( maxLog2TrDynamicRange - ui16AbsGoRice ) : ( prefixLength + 1/*separator*/ );

      iRate += ( COEF_REMAIN_BIN_REDUCTION + prefixLength + suffixLength + ui16AbsGoRice ) << SCALE_BITS;
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
    return iRate;
  }

  int iRate = int( xGetIEPRate() ); // cost of sign bit
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
    else if( useLimitedPrefixLength )
    {
      const uint32_t maximumPrefixLength = ( 32 - ( COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange ) );

      uint32_t prefixLength = 0;
      uint32_t suffix = ( symbol >> ui16AbsGoRice ) - COEF_REMAIN_BIN_REDUCTION;

      while( ( prefixLength < maximumPrefixLength ) && ( suffix > ( ( 2 << prefixLength ) - 2 ) ) )
      {
        prefixLength++;
      }

      const uint32_t suffixLength = ( prefixLength == maximumPrefixLength ) ? ( maxLog2TrDynamicRange - ui16AbsGoRice ) : ( prefixLength + 1/*separator*/ );

      iRate += ( COEF_REMAIN_BIN_REDUCTION + prefixLength + suffixLength + ui16AbsGoRice ) << SCALE_BITS;
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
  return  iRate;
}

inline double QuantRDOQ::xGetRateSigCoeffGroup( const BinFracBits& fracBitsSigCG, unsigned uiSignificanceCoeffGroup ) const
{
  return xGetICost( fracBitsSigCG.intBits[uiSignificanceCoeffGroup] );
}

/** Calculates the cost of signaling the last significant coefficient in the block
 * \param uiPosX X coordinate of the last significant coefficient
 * \param uiPosY Y coordinate of the last significant coefficient
 * \param component colour component ID
 * \returns cost of last significant coefficient
 */
/*
 * \param uiWidth width of the transform unit (TU)
*/
inline double QuantRDOQ::xGetRateLast( const int* lastBitsX, const int* lastBitsY, unsigned PosX, unsigned PosY ) const
{
  uint32_t    CtxX  = g_uiGroupIdx[PosX];
  uint32_t    CtxY  = g_uiGroupIdx[PosY];
  double  Cost  = lastBitsX[ CtxX ] + lastBitsY[ CtxY ];
  if( CtxX > 3 )
  {
    Cost += xGetIEPRate() * ((CtxX-2)>>1);
  }
  if( CtxY > 3 )
  {
    Cost += xGetIEPRate() * ((CtxY-2)>>1);
  }
  return xGetICost( Cost );
}


inline double QuantRDOQ::xGetRateSigCoef( const BinFracBits& fracBitsSig, unsigned uiSignificance ) const
{
  return xGetICost( fracBitsSig.intBits[uiSignificance] );
}

/** Get the cost for a specific rate
 * \param dRate rate of a bit
 * \returns cost at the specific rate
 */
inline double QuantRDOQ::xGetICost        ( double                          dRate         ) const
{
  return m_dLambda * dRate;
}

/** Get the cost of an equal probable bit
 * \returns cost of equal probable bit
 */
inline double QuantRDOQ::xGetIEPRate() const
{
  return 32768;
}


double QuantRDOQ::xGetErrScaleCoeff(const bool needsSqrt2, SizeType width, SizeType height, int qp, const int maxLog2TrDynamicRange, const int channelBitDepth, bool bTransformSkip=false)
{
  const int iTransformShift = bTransformSkip ? 0 : getTransformShift(channelBitDepth, Size(width, height), maxLog2TrDynamicRange);
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
void QuantRDOQ::xSetErrScaleCoeff( uint32_t list, uint32_t sizeX, uint32_t sizeY, int qp, const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths )
{
  const int width = g_scalingListSizeX[sizeX];
  const int height = g_scalingListSizeX[sizeY];
  const ChannelType channelType = ( ( list == 0 ) || ( list == MAX_NUM_COMP ) ) ? CH_L : CH_C;
  const int channelBitDepth = bitDepths[channelType];
  const int iTransformShift = getTransformShift( channelBitDepth, Size( g_scalingListSizeX[sizeX], g_scalingListSizeX[sizeY] ), maxLog2TrDynamicRange[channelType] );  // Represents scaling through forward transform

  uint32_t i, uiMaxNumCoeff = width * height;
  int *piQuantcoeff;
  double *pdErrScale;
  piQuantcoeff = getQuantCoeff( list, qp, sizeX, sizeY );
  pdErrScale   = xGetErrScaleCoeffSL( list, sizeX, sizeY, qp);

  double dErrScale = (double)( 1 << SCALE_BITS );                                // Compensate for scaling of bitcount in Lagrange cost function

  const bool needsSqrt2 = ((Log2(width*height)) & 1) == 1;
  double dTransShift = (double)iTransformShift + ( needsSqrt2 ? -0.5 : 0.0 );
  dErrScale = dErrScale*pow( 2.0, ( -2.0*dTransShift ) );                     // Compensate for scaling through forward transform

  for( i = 0; i < uiMaxNumCoeff; i++ )
  {
    pdErrScale[i] = dErrScale / piQuantcoeff[i] / piQuantcoeff[i]
                    / (1 << (DISTORTION_PRECISION_ADJUSTMENT(bitDepths[channelType]) << 1));
  }

  int QStep = g_quantScales[needsSqrt2][qp];

  xGetErrScaleCoeffNoScalingList(list, sizeX, sizeY, qp) =
    dErrScale / QStep / QStep / (1 << (DISTORTION_PRECISION_ADJUSTMENT(bitDepths[channelType]) << 1));
}

/** set flat matrix value to quantized coefficient
 */
void QuantRDOQ::setFlatScalingList(const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths)
{
  Quant::setFlatScalingList( maxLog2TrDynamicRange, bitDepths );

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

/** initialization process of scaling list array
 */
void QuantRDOQ::xInitScalingList( const QuantRDOQ* other )
{
  m_isErrScaleListOwner = other == nullptr;

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
            m_errScale[sizeIdX][sizeIdY][listId][qp] = new double[g_scalingListSizeX[sizeIdX] * g_scalingListSizeX[sizeIdY]];
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
void QuantRDOQ::xDestroyScalingList()
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


void QuantRDOQ::quant(TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx& ctx)
{
  const CompArea& rect      = tu.blocks[compID];
  const uint32_t uiWidth        = rect.width;
  const uint32_t uiHeight       = rect.height;

  const CCoeffBuf& piCoef   = pSrc;
        CoeffBuf   piQCoef  = tu.getCoeffs(compID);

  const bool useTransformSkip      = tu.mtsIdx[compID]==MTS_SKIP;

  bool useRDOQ = useTransformSkip ? m_useRDOQTS : m_RDOQ > 0;

  if( !tu.cu->ispMode || !isLuma(compID) )
  {
    useRDOQ &= uiWidth > 2;
    useRDOQ &= uiHeight > 2;
  }

  if( useRDOQ )
  {
    if (!m_useSelectiveRDOQ || xNeedRDOQ(tu, compID, piCoef, cQP))
    {
      if( useTransformSkip )
      {
        if((isLuma(compID) && tu.cu->bdpcmMode) || (isChroma(compID) && tu.cu->bdpcmModeChroma))
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
        xRateDistOptQuant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
      }
    }
    else
    {
      piQCoef.fill(0);
      uiAbsSum = 0;
      tu.lastPos[compID] = -1;
    }
  }
  else
  {
    Quant::quant( tu, compID, pSrc, uiAbsSum, cQP, ctx );
  }
}



void QuantRDOQ::xRateDistOptQuant(TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx &ctx)
{
  const FracBitsAccess& fracBits = ctx.getFracBitsAcess();

  const SPS &sps            = *tu.cs->sps;
  const CompArea& rect      = tu.blocks[compID];
  const uint32_t uiWidth        = rect.width;
  const uint32_t uiHeight       = rect.height;
  const ChannelType chType  = toChannelType(compID);
  const int channelBitDepth = sps.bitDepths[ chType ];

  const bool extendedPrecision     = sps.spsRExt.extendedPrecisionProcessing;
  const int  maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(chType);

  const bool useIntraSubPartitions = tu.cu->ispMode && isLuma(compID);
  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
  * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
  * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
  * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
  */

  // Represents scaling through forward transform
  int iTransformShift = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

  if (tu.mtsIdx[compID]==MTS_SKIP && extendedPrecision)
  {
    iTransformShift = std::max<int>(0, iTransformShift);
  }

  double     d64BlockUncodedCost               = 0;
  const uint32_t uiLog2BlockWidth                  = Log2(uiWidth);
  const uint32_t uiLog2BlockHeight                 = Log2(uiHeight);
  const uint32_t uiMaxNumCoeff                     = rect.area();

  CHECK(compID >= MAX_NUM_TBLOCKS, "Invalid component ID");

  int scalingListType = getScalingListType(tu.cu->predMode, compID);

  CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");

  const TCoeff *plSrcCoeff = pSrc.buf;
        TCoeff *piDstCoeff = tu.getCoeffs(compID).buf;

  double *pdCostCoeff  = m_pdCostCoeff;
  double *pdCostSig    = m_pdCostSig;
  double *pdCostCoeff0 = m_pdCostCoeff0;
  int    *rateIncUp    = m_rateIncUp;
  int    *rateIncDown  = m_rateIncDown;
  int    *sigRateDelta = m_sigRateDelta;
  TCoeff *deltaU       = m_deltaU;

  memset(piDstCoeff, 0, sizeof(*piDstCoeff) * uiMaxNumCoeff);
  memset( m_pdCostCoeff,  0, sizeof( double ) *  uiMaxNumCoeff );
  memset( m_pdCostSig,    0, sizeof( double ) *  uiMaxNumCoeff );
  memset( m_rateIncUp,    0, sizeof( int    ) *  uiMaxNumCoeff );
  memset( m_rateIncDown,  0, sizeof( int    ) *  uiMaxNumCoeff );
  memset( m_sigRateDelta, 0, sizeof( int    ) *  uiMaxNumCoeff );
  memset( m_deltaU,       0, sizeof( TCoeff ) *  uiMaxNumCoeff );


  const bool   needSqrtAdjustment = TU::needsSqrt2Scale( tu, compID );
  const bool   isTransformSkip    = tu.mtsIdx[compID]==MTS_SKIP;
  const double *const pdErrScale  = xGetErrScaleCoeffSL(scalingListType, uiLog2BlockWidth, uiLog2BlockHeight, cQP.rem(isTransformSkip));
  const int    *const piQCoef     = getQuantCoeff(scalingListType, cQP.rem(isTransformSkip), uiLog2BlockWidth, uiLog2BlockHeight);
  const bool isLfnstApplied       = tu.cu->lfnstIdx > 0 && (CU::isSepTree(*tu.cu) ? true : isLuma(compID));
  const bool enableScalingLists   = getUseScalingList(uiWidth, uiHeight, isTransformSkip, isLfnstApplied);
  const int    defaultQuantisationCoefficient = g_quantScales[ needSqrtAdjustment ?1:0][cQP.rem(isTransformSkip)];
  const double defaultErrorScale              = xGetErrScaleCoeffNoScalingList(scalingListType, uiLog2BlockWidth, uiLog2BlockHeight, cQP.rem(isTransformSkip));
  const int iQBits = QUANT_SHIFT + cQP.per(isTransformSkip) + iTransformShift + (needSqrtAdjustment?-1:0);                   // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits


  const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  CoeffCodingContext cctx(tu, compID, tu.cs->slice->signDataHidingEnabled);
  const int    iCGSizeM1      = (1 << cctx.log2CGSize()) - 1;

  int     iCGLastScanPos      = -1;
  double  d64BaseCost         = 0;
  int     iLastScanPos        = -1;

  int ctxBinSampleRatio = MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT;
  int remRegBins = (uiWidth * uiHeight * ctxBinSampleRatio) >> 4;
  uint32_t  goRiceParam   = 0;

  double *pdCostCoeffGroupSig = m_pdCostCoeffGroupSig;
  memset( pdCostCoeffGroupSig, 0, ( uiMaxNumCoeff >> cctx.log2CGSize() ) * sizeof( double ) );
  int iScanPos;
  coeffGroupRDStats rdStats;

#if ENABLE_TRACING
  DTRACE( g_trace_ctx, D_RDOQ, "%d: %3d, %3d, %dx%d, comp=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ ), rect.x, rect.y, rect.width, rect.height, compID );
#endif

  const uint32_t lfnstIdx = tu.cu->lfnstIdx;

  const int iCGNum = lfnstIdx > 0 ? 1 : std::min<int>(JVET_C0024_ZERO_OUT_TH, uiWidth) * std::min<int>(JVET_C0024_ZERO_OUT_TH, uiHeight) >> cctx.log2CGSize();

  for (int subSetId = iCGNum - 1; subSetId >= 0; subSetId--)
  {
    cctx.initSubblock( subSetId );

    uint32_t maxNonZeroPosInCG = iCGSizeM1;
    if( lfnstIdx > 0 && ( ( uiWidth == 4 && uiHeight == 4 ) || ( uiWidth == 8 && uiHeight == 8 && cctx.cgPosX() == 0 && cctx.cgPosY() == 0 ) ) )
    {
      maxNonZeroPosInCG = 7;
    }

    memset( &rdStats, 0, sizeof (coeffGroupRDStats));

    for( int iScanPosinCG = iCGSizeM1; iScanPosinCG > maxNonZeroPosInCG; iScanPosinCG-- )
    {
      iScanPos = cctx.minSubPos() + iScanPosinCG;
      uint32_t    blkPos = cctx.blockPos( iScanPos );
      piDstCoeff[ blkPos ] = 0;
    }
    for( int iScanPosinCG = maxNonZeroPosInCG; iScanPosinCG >= 0; iScanPosinCG-- )
    {
      iScanPos = cctx.minSubPos() + iScanPosinCG;
      //===== quantization =====
      uint32_t    uiBlkPos          = cctx.blockPos(iScanPos);

      // set coeff
      const int    quantisationCoefficient = (enableScalingLists) ? piQCoef   [uiBlkPos]               : defaultQuantisationCoefficient;
      const double errorScale              = (enableScalingLists) ? pdErrScale[uiBlkPos]               : defaultErrorScale;
      const int64_t  tmpLevel                = int64_t(abs(plSrcCoeff[ uiBlkPos ])) * quantisationCoefficient;

      const Intermediate_Int lLevelDouble  = (Intermediate_Int)std::min<int64_t>(tmpLevel, std::numeric_limits<Intermediate_Int>::max() - (Intermediate_Int(1) << (iQBits - 1)));

      uint32_t uiMaxAbsLevel        = std::min<uint32_t>(uint32_t(entropyCodingMaximum), uint32_t((lLevelDouble + (Intermediate_Int(1) << (iQBits - 1))) >> iQBits));

      const double dErr         = double( lLevelDouble );
      pdCostCoeff0[ iScanPos ]  = dErr * dErr * errorScale;
      d64BlockUncodedCost      += pdCostCoeff0[ iScanPos ];
      piDstCoeff[ uiBlkPos ]    = uiMaxAbsLevel;

      if ( uiMaxAbsLevel > 0 && iLastScanPos < 0 )
      {
        iLastScanPos            = iScanPos;
        iCGLastScanPos          = cctx.subSetId();
      }

      if ( iLastScanPos >= 0 )
      {

#if ENABLE_TRACING
        uint32_t uiCGPosY = cctx.cgPosY();
        uint32_t uiCGPosX = cctx.cgPosX();
        uint32_t uiPosY = cctx.posY( iScanPos );
        uint32_t uiPosX = cctx.posX( iScanPos );
        DTRACE( g_trace_ctx, D_RDOQ, "%d [%d][%d][%2d:%2d][%2d:%2d]", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ ), iScanPos, uiBlkPos, uiCGPosX, uiCGPosY, uiPosX, uiPosY );
#endif
        //===== coefficient level estimation =====
        unsigned ctxIdSig = 0;
        if( iScanPos != iLastScanPos )
        {
          ctxIdSig = cctx.sigCtxIdAbs( iScanPos, piDstCoeff, 0 );
        }
        uint32_t    uiLevel;
        uint8_t ctxOffset     = cctx.ctxOffsetAbs     ();
        uint32_t    uiParCtx      = cctx.parityCtxIdAbs   ( ctxOffset );
        uint32_t    uiGt1Ctx      = cctx.greater1CtxIdAbs ( ctxOffset );
        uint32_t    uiGt2Ctx      = cctx.greater2CtxIdAbs ( ctxOffset );
        uint32_t    goRiceZero    = 0;
        if( remRegBins < 4 )
        {
          unsigned  sumAbs = cctx.templateAbsSum( iScanPos, piDstCoeff, 0 );
          goRiceParam             = g_auiGoRiceParsCoeff   [ sumAbs ];
          goRiceZero              = g_auiGoRicePosCoeff0(0, goRiceParam);
        }

        const BinFracBits fracBitsPar = fracBits.getFracBitsArray( uiParCtx );
        const BinFracBits fracBitsGt1 = fracBits.getFracBitsArray( uiGt1Ctx );
        const BinFracBits fracBitsGt2 = fracBits.getFracBitsArray( uiGt2Ctx );

        if( iScanPos == iLastScanPos )
        {
          uiLevel = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ],
                                    lLevelDouble, uiMaxAbsLevel, nullptr, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, iQBits, errorScale, 1, extendedPrecision, maxLog2TrDynamicRange );
        }
        else
        {
          DTRACE_COND( ( uiMaxAbsLevel != 0 ), g_trace_ctx, D_RDOQ_MORE, " uiCtxSig=%d", ctxIdSig );

          const BinFracBits fracBitsSig = fracBits.getFracBitsArray( ctxIdSig );
          uiLevel = xGetCodedLevel( pdCostCoeff[ iScanPos ], pdCostCoeff0[ iScanPos ], pdCostSig[ iScanPos ],
                                    lLevelDouble, uiMaxAbsLevel, &fracBitsSig, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, iQBits, errorScale, 0, extendedPrecision, maxLog2TrDynamicRange );
          sigRateDelta[ uiBlkPos ] = ( remRegBins < 4 ? 0 : fracBitsSig.intBits[1] - fracBitsSig.intBits[0] );
        }

        DTRACE( g_trace_ctx, D_RDOQ, " Lev=%d \n", uiLevel );
        DTRACE_COND( ( uiMaxAbsLevel != 0 ), g_trace_ctx, D_RDOQ, " CostC0=%d\n", (int64_t)( pdCostCoeff0[iScanPos] ) );
        DTRACE_COND( ( uiMaxAbsLevel != 0 ), g_trace_ctx, D_RDOQ, " CostC =%d\n", (int64_t)( pdCostCoeff[iScanPos] ) );

        deltaU[ uiBlkPos ]        = TCoeff((lLevelDouble - (Intermediate_Int(uiLevel) << iQBits)) >> (iQBits-8));

        if( uiLevel > 0 )
        {
          int rateNow              = xGetICRate( uiLevel,   fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, extendedPrecision, maxLog2TrDynamicRange );
          rateIncUp   [ uiBlkPos ] = xGetICRate( uiLevel+1, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, extendedPrecision, maxLog2TrDynamicRange ) - rateNow;
          rateIncDown [ uiBlkPos ] = xGetICRate( uiLevel-1, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, extendedPrecision, maxLog2TrDynamicRange ) - rateNow;
        }
        else // uiLevel == 0
        {
          if( remRegBins < 4 )
          {
            int rateNow            = xGetICRate( uiLevel,   fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, extendedPrecision, maxLog2TrDynamicRange );
            rateIncUp [ uiBlkPos ] = xGetICRate( uiLevel+1, fracBitsPar, fracBitsGt1, fracBitsGt2, remRegBins, goRiceZero, goRiceParam, extendedPrecision, maxLog2TrDynamicRange ) - rateNow;
          }
          else
          {
            rateIncUp [ uiBlkPos ] = fracBitsGt1.intBits[ 0 ];
          }
        }
        piDstCoeff[ uiBlkPos ] = uiLevel;
        d64BaseCost           += pdCostCoeff [ iScanPos ];

        if( ( (iScanPos & iCGSizeM1) == 0 ) && ( iScanPos > 0 ) )
        {
          goRiceParam   = 0;
        }
        else if( remRegBins >= 4 )
        {
          int  sumAll = cctx.templateAbsSum(iScanPos, piDstCoeff, 4);
          goRiceParam = g_auiGoRiceParsCoeff[sumAll];
          remRegBins -= (uiLevel < 2 ? uiLevel : 3) + (iScanPos != iLastScanPos);
        }
      }
      else
      {
        d64BaseCost    += pdCostCoeff0[ iScanPos ];
      }
      rdStats.d64SigCost += pdCostSig[ iScanPos ];
      if (iScanPosinCG == 0 )
      {
        rdStats.d64SigCost_0 = pdCostSig[ iScanPos ];
      }
      if (piDstCoeff[ uiBlkPos ] )
      {
        cctx.setSigGroup();
        rdStats.d64CodedLevelandDist += pdCostCoeff[ iScanPos ] - pdCostSig[ iScanPos ];
        rdStats.d64UncodedDist += pdCostCoeff0[ iScanPos ];
        if ( iScanPosinCG != 0 )
        {
          rdStats.iNNZbeforePos0++;
        }
      }
    } //end for (iScanPosinCG)

    if (iCGLastScanPos >= 0)
    {
      if( cctx.subSetId() )
      {
        if( !cctx.isSigGroup() )
        {
          const BinFracBits fracBitsSigGroup = fracBits.getFracBitsArray( cctx.sigGroupCtxId() );
          d64BaseCost += xGetRateSigCoeffGroup(fracBitsSigGroup, 0) - rdStats.d64SigCost;
          pdCostCoeffGroupSig[ cctx.subSetId() ] = xGetRateSigCoeffGroup(fracBitsSigGroup, 0);
        }
        else
        {
          if (cctx.subSetId() < iCGLastScanPos) //skip the last coefficient group, which will be handled together with last position below.
          {
            if ( rdStats.iNNZbeforePos0 == 0 )
            {
              d64BaseCost -= rdStats.d64SigCost_0;
              rdStats.d64SigCost -= rdStats.d64SigCost_0;
            }
            // rd-cost if SigCoeffGroupFlag = 0, initialization
            double d64CostZeroCG = d64BaseCost;

            const BinFracBits fracBitsSigGroup = fracBits.getFracBitsArray( cctx.sigGroupCtxId() );

            if (cctx.subSetId() < iCGLastScanPos)
            {
              d64BaseCost  += xGetRateSigCoeffGroup(fracBitsSigGroup,1);
              d64CostZeroCG += xGetRateSigCoeffGroup(fracBitsSigGroup,0);
              pdCostCoeffGroupSig[ cctx.subSetId() ] = xGetRateSigCoeffGroup(fracBitsSigGroup,1);
            }

            // try to convert the current coeff group from non-zero to all-zero
            d64CostZeroCG += rdStats.d64UncodedDist;  // distortion for resetting non-zero levels to zero levels
            d64CostZeroCG -= rdStats.d64CodedLevelandDist;   // distortion and level cost for keeping all non-zero levels
            d64CostZeroCG -= rdStats.d64SigCost;     // sig cost for all coeffs, including zero levels and non-zerl levels

                                                     // if we can save cost, change this block to all-zero block
            if ( d64CostZeroCG < d64BaseCost )
            {
              cctx.resetSigGroup();
              d64BaseCost = d64CostZeroCG;
              if (cctx.subSetId() < iCGLastScanPos)
              {
                pdCostCoeffGroupSig[ cctx.subSetId() ] = xGetRateSigCoeffGroup(fracBitsSigGroup,0);
              }
              // reset coeffs to 0 in this block
              for( int iScanPosinCG = maxNonZeroPosInCG; iScanPosinCG >= 0; iScanPosinCG-- )
              {
                iScanPos      = cctx.minSubPos() + iScanPosinCG;
                uint32_t uiBlkPos = cctx.blockPos( iScanPos );

                if (piDstCoeff[ uiBlkPos ])
                {
                  piDstCoeff [ uiBlkPos ] = 0;
                  pdCostCoeff[ iScanPos ] = pdCostCoeff0[ iScanPos ];
                  pdCostSig  [ iScanPos ] = 0;
                }
              }
            } // end if ( d64CostAllZeros < d64BaseCost )
          }
        } // end if if (uiSigCoeffGroupFlag[ uiCGBlkPos ] == 0)
      }
      else
      {
        cctx.setSigGroup();
      }
    }
  } //end for (cctx.subSetId)


  //===== estimate last position =====
  if ( iLastScanPos < 0 )
  {
    return;
  }

  double  d64BestCost         = 0;
  int     iBestLastIdxP1      = 0;


  if( !CU::isIntra( *tu.cu ) && isLuma( compID ) && tu.depth == 0 )
  {
    const BinFracBits fracBitsQtRootCbf = fracBits.getFracBitsArray( Ctx::QtRootCbf() );
    d64BestCost  = d64BlockUncodedCost + xGetICost( fracBitsQtRootCbf.intBits[ 0 ] );
    d64BaseCost += xGetICost( fracBitsQtRootCbf.intBits[ 1 ] );
  }
  else
  {
    bool previousCbf       = tu.cbf[COMP_Cb];
    bool lastCbfIsInferred = false;
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
      d64BestCost  = d64BlockUncodedCost + xGetICost(fracBitsQtCbf.intBits[0]);
      d64BaseCost += xGetICost(fracBitsQtCbf.intBits[1]);
    }
    else
    {
      d64BestCost  = d64BlockUncodedCost;
    }
  }

  int lastBitsX[LAST_SIGNIFICANT_GROUPS] = { 0 };
  int lastBitsY[LAST_SIGNIFICANT_GROUPS] = { 0 };
  {
    int dim1 = std::min<int>(JVET_C0024_ZERO_OUT_TH, uiWidth);
    int dim2 = std::min<int>(JVET_C0024_ZERO_OUT_TH, uiHeight);
    int bitsX = 0;
    int bitsY = 0;
    int ctxId;
    //X-coordinate
    for ( ctxId = 0; ctxId < g_uiGroupIdx[dim1-1]; ctxId++)
    {
      const BinFracBits fB = fracBits.getFracBitsArray( cctx.lastXCtxId(ctxId) );
      lastBitsX[ ctxId ]   = bitsX + fB.intBits[ 0 ];
      bitsX               +=         fB.intBits[ 1 ];
    }
    lastBitsX[ctxId] = bitsX;
    //Y-coordinate
    for ( ctxId = 0; ctxId < g_uiGroupIdx[dim2-1]; ctxId++)
    {
      const BinFracBits fB = fracBits.getFracBitsArray( cctx.lastYCtxId(ctxId) );
      lastBitsY[ ctxId ]   = bitsY + fB.intBits[ 0 ];
      bitsY               +=         fB.intBits[ 1 ];
    }
    lastBitsY[ctxId] = bitsY;
  }


  bool bFoundLast = false;
  for (int iCGScanPos = iCGLastScanPos; iCGScanPos >= 0; iCGScanPos--)
  {
    d64BaseCost -= pdCostCoeffGroupSig [ iCGScanPos ];
    if (cctx.isSigGroup( iCGScanPos ) )
    {
      uint32_t maxNonZeroPosInCG = iCGSizeM1;
      if( lfnstIdx > 0 && ( ( uiWidth == 4 && uiHeight == 4 ) || ( uiWidth == 8 && uiHeight == 8 && cctx.cgPosX() == 0 && cctx.cgPosY() == 0 ) ) )
      {
        maxNonZeroPosInCG = 7;
      }
      for( int iScanPosinCG = maxNonZeroPosInCG; iScanPosinCG >= 0; iScanPosinCG-- )
      {
        iScanPos = iCGScanPos * (iCGSizeM1 + 1) + iScanPosinCG;

        if (iScanPos > iLastScanPos)
        {
          continue;
        }
        uint32_t   uiBlkPos     = cctx.blockPos( iScanPos );

        if( piDstCoeff[ uiBlkPos ] )
        {
          uint32_t   uiPosY = uiBlkPos >> uiLog2BlockWidth;
          uint32_t   uiPosX = uiBlkPos - ( uiPosY << uiLog2BlockWidth );
          double d64CostLast  = xGetRateLast( lastBitsX, lastBitsY, uiPosX, uiPosY );

          double totalCost = d64BaseCost + d64CostLast - pdCostSig[ iScanPos ];

          if( totalCost < d64BestCost )
          {
            iBestLastIdxP1  = iScanPos + 1;
            d64BestCost     = totalCost;
          }
          if( piDstCoeff[ uiBlkPos ] > 1 )
          {
            bFoundLast = true;
            break;
          }
          d64BaseCost      -= pdCostCoeff[ iScanPos ];
          d64BaseCost      += pdCostCoeff0[ iScanPos ];
        }
        else
        {
          d64BaseCost      -= pdCostSig[ iScanPos ];
        }
      } //end for
      if (bFoundLast)
      {
        break;
      }
    } // end if (uiSigCoeffGroupFlag[ uiCGBlkPos ])
    DTRACE( g_trace_ctx, D_RDOQ_COST, "%d: %3d, %3d, %dx%d, comp=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_RDOQ_COST ), rect.x, rect.y, rect.width, rect.height, compID );
    DTRACE( g_trace_ctx, D_RDOQ_COST, "Uncoded=%d\n", (int64_t)( d64BlockUncodedCost ) );
    DTRACE( g_trace_ctx, D_RDOQ_COST, "Coded  =%d\n", (int64_t)( d64BaseCost ) );

  } // end for


  for ( int scanPos = 0; scanPos < iBestLastIdxP1; scanPos++ )
  {
    int blkPos = cctx.blockPos( scanPos );
    TCoeff level = piDstCoeff[ blkPos ];
    uiAbsSum += level;
    piDstCoeff[ blkPos ] = ( plSrcCoeff[ blkPos ] < 0 ) ? -level : level;
  }

  //===== clean uncoded coefficients =====
  for ( int scanPos = iBestLastIdxP1; scanPos <= iLastScanPos; scanPos++ )
  {
    piDstCoeff[ cctx.blockPos( scanPos ) ] = 0;
  }
  iLastScanPos = iBestLastIdxP1 - 1;

  if( cctx.signHiding() && uiAbsSum>=2)
  {
    const double inverseQuantScale = double(g_invQuantScales[0][cQP.rem(isTransformSkip)]);
    int64_t rdFactor = (int64_t)(inverseQuantScale * inverseQuantScale * (1 << (2 * cQP.per(isTransformSkip))) / m_dLambda / 16
                                  / (1 << (2 * DISTORTION_PRECISION_ADJUSTMENT(channelBitDepth)))
                             + 0.5);

    int lastCG = -1;
    int absSum = 0 ;
    int n ;
    for (int subSet = iCGNum - 1; subSet >= 0; subSet--)
    {
      int  subPos         = subSet << cctx.log2CGSize();
      int  firstNZPosInCG = iCGSizeM1 + 1, lastNZPosInCG = -1;
      absSum = 0 ;

      for( n = iCGSizeM1; n >= 0; --n )
      {
        if( piDstCoeff[ cctx.blockPos( n + subPos )] )
        {
          lastNZPosInCG = n;
          break;
        }
      }

      for( n = 0; n <= iCGSizeM1; n++ )
      {
        if( piDstCoeff[ cctx.blockPos( n + subPos )] )
        {
          firstNZPosInCG = n;
          break;
        }
      }

      for( n = firstNZPosInCG; n <= lastNZPosInCG; n++ )
      {
        absSum += int(piDstCoeff[ cctx.blockPos( n + subPos )]);
      }

      if(lastNZPosInCG>=0 && lastCG==-1)
      {
        lastCG = 1;
      }

      if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )
      {
        uint32_t signbit = (piDstCoeff[cctx.blockPos(subPos+firstNZPosInCG)]>0?0:1);
        if( signbit!=(absSum&0x1) )  // hide but need tune
        {
          // calculate the cost
          int64_t minCostInc = std::numeric_limits<int64_t>::max(), curCost = std::numeric_limits<int64_t>::max();
          int minPos = -1, finalChange = 0, curChange = 0;

          for( n = (lastCG == 1 ? lastNZPosInCG : iCGSizeM1); n >= 0; --n )
          {
            uint32_t uiBlkPos   = cctx.blockPos( n + subPos );
            if(piDstCoeff[ uiBlkPos ] != 0 )
            {
              int64_t costUp   = rdFactor * ( - deltaU[uiBlkPos] ) + rateIncUp[uiBlkPos];
              int64_t costDown = rdFactor * (   deltaU[uiBlkPos] ) + rateIncDown[uiBlkPos]
                -   ((abs(piDstCoeff[uiBlkPos]) == 1) ? sigRateDelta[uiBlkPos] : 0);

              if(lastCG==1 && lastNZPosInCG==n && abs(piDstCoeff[uiBlkPos])==1)
              {
                costDown -= (4<<SCALE_BITS);
              }

              if(costUp<costDown)
              {
                curCost = costUp;
                curChange =  1;
              }
              else
              {
                curChange = -1;
                if(n==firstNZPosInCG && abs(piDstCoeff[uiBlkPos])==1)
                {
                  curCost = std::numeric_limits<int64_t>::max();
                }
                else
                {
                  curCost = costDown;
                }
              }
            }
            else
            {
              curCost = rdFactor * ( - (abs(deltaU[uiBlkPos])) ) + (1<<SCALE_BITS) + rateIncUp[uiBlkPos] + sigRateDelta[uiBlkPos] ;
              curChange = 1 ;

              if(n<firstNZPosInCG)
              {
                uint32_t thissignbit = (plSrcCoeff[uiBlkPos]>=0?0:1);
                if(thissignbit != signbit )
                {
                  curCost = std::numeric_limits<int64_t>::max();
                }
              }
            }

            if( curCost<minCostInc)
            {
              minCostInc = curCost;
              finalChange = curChange;
              minPos = uiBlkPos;
            }
          }

          if(piDstCoeff[minPos] == entropyCodingMaximum || piDstCoeff[minPos] == entropyCodingMinimum)
          {
            finalChange = -1;
          }

          if(plSrcCoeff[minPos]>=0)
          {
            piDstCoeff[minPos] += finalChange ;
          }
          else
          {
            piDstCoeff[minPos] -= finalChange ;
          }
        }
      }

      if(lastCG==1)
      {
        lastCG=0 ;
      }
    }

    // Check due to saving of last pos. Sign data hiding can change the position of last coef.
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
  tu.lastPos[compID] = iLastScanPos;
}

void QuantRDOQ::rateDistOptQuantTS( TransformUnit& tu, const ComponentID compID, const CCoeffBuf& coeffs, TCoeff &absSum, const QpParam& qp, const Ctx &ctx )
{
  const FracBitsAccess& fracBits = ctx.getFracBitsAcess();

  const SPS &sps            = *tu.cs->sps;
  const CompArea& rect      = tu.blocks[compID];
  const uint32_t width      = rect.width;
  const uint32_t height     = rect.height;
  const ChannelType chType  = toChannelType(compID);
  const int channelBitDepth = sps.bitDepths[ chType ];

  const bool extendedPrecision     = sps.spsRExt.extendedPrecisionProcessing;
  const int  maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(chType);

  int transformShift = getTransformShift( channelBitDepth, rect.size(), maxLog2TrDynamicRange );

  if( extendedPrecision )
  {
    transformShift = std::max<int>( 0, transformShift );
  }

        double   blockUncodedCost                   = 0;
  const uint32_t maxNumCoeff                        = rect.area();

  CHECK( compID >= MAX_NUM_TBLOCKS, "Invalid component ID" );

  int scalingListType = getScalingListType( tu.cu->predMode, compID );
  CHECK( scalingListType >= SCALING_LIST_NUM, "Invalid scaling list" );

  const TCoeff *srcCoeff = coeffs.buf;
        TCoeff *dstCoeff = tu.getCoeffs( compID ).buf;

  double *costCoeff  = m_pdCostCoeff;
  double *costSig    = m_pdCostSig;
  double *costCoeff0 = m_pdCostCoeff0;

  memset( m_pdCostCoeff,  0, sizeof( double ) *  maxNumCoeff );
  memset( m_pdCostSig,    0, sizeof( double ) *  maxNumCoeff );

  m_bdpcm = 0;

  const bool   needsSqrt2Scale = TU::needsSqrt2Scale( tu, compID );  // should always be false - transform-skipped blocks don't require sqrt(2) compensation.
  const bool   isTransformSkip = tu.mtsIdx[compID]==MTS_SKIP;
  const int    qBits = QUANT_SHIFT + qp.per(isTransformSkip) + (isTransformSkip ? 0 : transformShift) + (needsSqrt2Scale ? -1 : 0);  // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits
  const int    quantisationCoefficient = g_quantScales[needsSqrt2Scale?1:0][qp.rem(isTransformSkip)];
  const double errorScale              = xGetErrScaleCoeff( TU::needsSqrt2Scale(tu, compID), width, height, qp.rem(isTransformSkip), maxLog2TrDynamicRange, channelBitDepth, isTransformSkip);

  const TCoeff entropyCodingMaximum = ( 1 << maxLog2TrDynamicRange ) - 1;

  uint32_t coeffLevels[3];
  double   coeffLevelError[4];

  CoeffCodingContext cctx( tu, compID, tu.cs->slice->signDataHidingEnabled );
  const int sbSizeM1    = ( 1 << cctx.log2CGSize() ) - 1;
  double    baseCost    = 0;
  uint32_t  goRiceParam = 0;

  double *costSigSubBlock = m_pdCostCoeffGroupSig;
  memset( costSigSubBlock, 0, ( maxNumCoeff >> cctx.log2CGSize() ) * sizeof( double ) );

  const int sbNum = width * height >> cctx.log2CGSize();
  int scanPos;
  coeffGroupRDStats rdStats;

  bool anySigCG = false;

  int maxCtxBins = (cctx.maxNumCoeff() * 7) >> 2;
  cctx.setNumCtxBins(maxCtxBins);

  for( int sbId = 0; sbId < sbNum; sbId++ )
  {
    cctx.initSubblock( sbId );

    int noCoeffCoded = 0;
    baseCost = 0.0;
    memset( &rdStats, 0, sizeof (coeffGroupRDStats));

    rdStats.iNumSbbCtxBins = 0;

    for( int scanPosInSB = 0; scanPosInSB <= sbSizeM1; scanPosInSB++ )
    {
      int lastPosCoded = sbSizeM1;
      scanPos = cctx.minSubPos() + scanPosInSB;
      //===== quantization =====
      uint32_t blkPos = cctx.blockPos( scanPos );

      // set coeff
      const int64_t          tmpLevel    = int64_t( abs( srcCoeff[blkPos] ) ) * quantisationCoefficient;
      const Intermediate_Int levelDouble = (Intermediate_Int)std::min<int64_t>( tmpLevel, std::numeric_limits<Intermediate_Int>::max() - ( Intermediate_Int( 1 ) << ( qBits - 1 ) ) );

      uint32_t roundAbsLevel = std::min<uint32_t>(uint32_t(entropyCodingMaximum), uint32_t((levelDouble + (Intermediate_Int(1) << (qBits - 1))) >> qBits));
      uint32_t minAbsLevel = (roundAbsLevel > 1 ? roundAbsLevel - 1 : 1);

      uint32_t downAbsLevel = std::min<uint32_t>(uint32_t(entropyCodingMaximum), uint32_t(levelDouble >> qBits));
      uint32_t upAbsLevel = std::min<uint32_t>(uint32_t(entropyCodingMaximum), downAbsLevel + 1);

      m_testedLevels = 0;
      coeffLevels[m_testedLevels++] = roundAbsLevel;

      if (minAbsLevel != roundAbsLevel)
        coeffLevels[m_testedLevels++] = minAbsLevel;

      int rightPixel, belowPixel, predPixel;

      cctx.neighTS(rightPixel, belowPixel, scanPos, dstCoeff);
      predPixel = cctx.deriveModCoeff(rightPixel, belowPixel, upAbsLevel, 0);

      if (upAbsLevel != roundAbsLevel && upAbsLevel != minAbsLevel && predPixel == 1)
        coeffLevels[m_testedLevels++] = upAbsLevel;

      double dErr = double(levelDouble);
      coeffLevelError[0] = dErr * dErr * errorScale;

      costCoeff0[scanPos] = coeffLevelError[0];
      blockUncodedCost   += costCoeff0[ scanPos ];
      dstCoeff[blkPos]    = coeffLevels[0];

      //===== coefficient level estimation =====
            unsigned    ctxIdSig = cctx.sigCtxIdAbsTS( scanPos, dstCoeff );
            uint32_t    cLevel;
      const BinFracBits fracBitsPar = fracBits.getFracBitsArray( cctx.parityCtxIdAbsTS() );

      goRiceParam = cctx.templateAbsSumTS( scanPos, dstCoeff );
      unsigned ctxIdSign = cctx.signCtxIdAbsTS(scanPos, dstCoeff, 0);
      const BinFracBits fracBitsSign = fracBits.getFracBitsArray(ctxIdSign);
      const uint8_t     sign         = srcCoeff[ blkPos ] < 0 ? 1 : 0;

      DTRACE_COND( ( coeffLevels[0] != 0 ), g_trace_ctx, D_RDOQ_MORE, " uiCtxSig=%d", ctxIdSig );

      unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(scanPos, dstCoeff, 0);
      const BinFracBits fracBitsGr1 = fracBits.getFracBitsArray(gt1CtxId);

      const BinFracBits fracBitsSig = fracBits.getFracBitsArray( ctxIdSig );
      bool lastCoeff = false; //
      if (scanPosInSB == lastPosCoded && noCoeffCoded == 0)
      {
        lastCoeff = true;
      }
      int numUsedCtxBins = 0;
      cLevel = xGetCodedLevelTSPred(costCoeff[scanPos], costCoeff0[scanPos], costSig[scanPos], levelDouble, qBits, errorScale, coeffLevels, coeffLevelError,
                                    &fracBitsSig, fracBitsPar, cctx, fracBits, fracBitsSign, fracBitsGr1, sign, rightPixel, belowPixel, goRiceParam, lastCoeff, extendedPrecision, maxLog2TrDynamicRange, numUsedCtxBins);

      cctx.decimateNumCtxBins(numUsedCtxBins);
      rdStats.iNumSbbCtxBins += numUsedCtxBins;

      if (cLevel > 0)
      {
        noCoeffCoded++;
      }

      TCoeff level = cLevel;
      dstCoeff[blkPos] = (level != 0 && srcCoeff[blkPos] < 0) ? -level : level;
      baseCost           += costCoeff[ scanPos ];
      rdStats.d64SigCost += costSig[ scanPos ];

      if( dstCoeff[ blkPos ] )
      {
        cctx.setSigGroup();
        rdStats.d64CodedLevelandDist += costCoeff [ scanPos ] - costSig[ scanPos ];
        rdStats.d64UncodedDist       += costCoeff0[ scanPos ];
      }
    } //end for (iScanPosinCG)

    if( !cctx.isSigGroup() )
    {
      const BinFracBits fracBitsSigGroup = fracBits.getFracBitsArray( cctx.sigGroupCtxId( true ) );
      baseCost += xGetRateSigCoeffGroup( fracBitsSigGroup, 0 ) - rdStats.d64SigCost;
      costSigSubBlock[cctx.subSetId()] = xGetRateSigCoeffGroup( fracBitsSigGroup, 0 );
      cctx.increaseNumCtxBins(rdStats.iNumSbbCtxBins); // skip sub-block
    }
    else if( sbId != sbNum - 1 || anySigCG )
    {
      // rd-cost if SigCoeffGroupFlag = 0, initialization
      double costZeroSB = baseCost;

      const BinFracBits fracBitsSigGroup = fracBits.getFracBitsArray( cctx.sigGroupCtxId( true ) );

      baseCost   += xGetRateSigCoeffGroup( fracBitsSigGroup, 1 );
      costZeroSB += xGetRateSigCoeffGroup( fracBitsSigGroup, 0 );
      costSigSubBlock[ cctx.subSetId() ] = xGetRateSigCoeffGroup( fracBitsSigGroup, 1 );

      costZeroSB += rdStats.d64UncodedDist;         // distortion for resetting non-zero levels to zero levels
      costZeroSB -= rdStats.d64CodedLevelandDist;   // distortion and level cost for keeping all non-zero levels
      costZeroSB -= rdStats.d64SigCost;             // sig cost for all coeffs, including zero levels and non-zerl levels

      if( costZeroSB < baseCost )
      {
        cctx.resetSigGroup();
        baseCost = costZeroSB;
        costSigSubBlock[ cctx.subSetId() ] = xGetRateSigCoeffGroup( fracBitsSigGroup, 0 );
        cctx.increaseNumCtxBins(rdStats.iNumSbbCtxBins); // skip sub-block

        for( int scanPosInSB = 0; scanPosInSB <= sbSizeM1; scanPosInSB++ )
        {
          scanPos = cctx.minSubPos() + scanPosInSB;
          uint32_t blkPos = cctx.blockPos( scanPos );

          if( dstCoeff[ blkPos ] )
          {
            dstCoeff[ blkPos ] = 0;
            costCoeff[ scanPos ] = costCoeff0[ scanPos ];
            costSig[ scanPos] = 0;
          }
        }
      }
      else
      {
        anySigCG = true;
      }
    }
  }

  //===== estimate last position =====
  for( int scanPos = 0; scanPos < maxNumCoeff; scanPos++ )
  {
    int blkPos = cctx.blockPos( scanPos );
    TCoeff level = dstCoeff[ blkPos ];
    absSum += abs(level);
  }
}

void QuantRDOQ::forwardRDPCM( TransformUnit& tu, const ComponentID compID, const CCoeffBuf& coeffs, TCoeff &absSum, const QpParam& qp, const Ctx &ctx )
{
  const FracBitsAccess& fracBits = ctx.getFracBitsAcess();

  const SPS &sps = *tu.cs->sps;
  const CompArea& rect = tu.blocks[compID];
  const uint32_t width = rect.width;
  const uint32_t height = rect.height;
  const ChannelType chType = toChannelType(compID);
  const int channelBitDepth = sps.bitDepths[chType];

  const bool extendedPrecision = sps.spsRExt.extendedPrecisionProcessing;
  const int  maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(chType);
  const int  dirMode = isLuma(compID) ? tu.cu->bdpcmMode : tu.cu->bdpcmModeChroma;

  int transformShift = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange);

  if (extendedPrecision)
  {
    transformShift = std::max<int>(0, transformShift);
  }

  double   blockUncodedCost = 0;
  const uint32_t maxNumCoeff = rect.area();

  CHECK(compID >= MAX_NUM_TBLOCKS, "Invalid component ID");

  int scalingListType = getScalingListType(tu.cu->predMode, compID);
  CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");

  const TCoeff *srcCoeff = coeffs.buf;
  TCoeff *dstCoeff = tu.getCoeffs(compID).buf;

  double *costCoeff = m_pdCostCoeff;
  double *costSig = m_pdCostSig;
  double *costCoeff0 = m_pdCostCoeff0;

  memset(m_pdCostCoeff, 0, sizeof(double) *  maxNumCoeff);
  memset(m_pdCostSig, 0, sizeof(double) *  maxNumCoeff);
  memset(m_fullCoeff, 0, sizeof(TCoeff) * maxNumCoeff);

  m_bdpcm = dirMode;

  const bool   needsSqrt2Scale = TU::needsSqrt2Scale(tu, compID);  // should always be false - transform-skipped blocks don't require sqrt(2) compensation.
  const bool   isTransformSkip = tu.mtsIdx[compID]==MTS_SKIP;
  const int    qBits = QUANT_SHIFT + qp.per(isTransformSkip) + (isTransformSkip? 0 : transformShift) + ( needsSqrt2Scale ? -1 : 0);  // Right shift of non-RDOQ quantizer;  level = (coeff*uiQ + offset)>>q_bits
  const int    quantisationCoefficient = g_quantScales[needsSqrt2Scale ? 1 : 0][qp.rem(isTransformSkip)];
  const double errorScale = xGetErrScaleCoeff(TU::needsSqrt2Scale(tu, compID), width, height, qp.rem(isTransformSkip), maxLog2TrDynamicRange, channelBitDepth, isTransformSkip);

  TrQuantParams trQuantParams;
  trQuantParams.rightShift = (IQUANT_SHIFT - ((isTransformSkip ? 0 : transformShift) + qp.per(isTransformSkip)));
  trQuantParams.qScale = g_invQuantScales[needsSqrt2Scale ? 1 : 0][qp.rem(isTransformSkip)];

  const TCoeff entropyCodingMaximum = (1 << maxLog2TrDynamicRange) - 1;

  uint32_t coeffLevels[3];
  double   coeffLevelError[4];

  CoeffCodingContext cctx(tu, compID, tu.cs->slice->signDataHidingEnabled);
  const int sbSizeM1 = (1 << cctx.log2CGSize()) - 1;
  double    baseCost = 0;
  uint32_t  goRiceParam = 0;

  double *costSigSubBlock = m_pdCostCoeffGroupSig;
  memset(costSigSubBlock, 0, (maxNumCoeff >> cctx.log2CGSize()) * sizeof(double));

  const int sbNum = width * height >> cctx.log2CGSize();
  int scanPos;
  coeffGroupRDStats rdStats;

  bool anySigCG = false;

  int maxCtxBins = (cctx.maxNumCoeff() * 7) >> 2;
  cctx.setNumCtxBins(maxCtxBins);

  for (int sbId = 0; sbId < sbNum; sbId++)
  {
    cctx.initSubblock(sbId);

    int noCoeffCoded = 0;
    baseCost = 0.0;
    memset(&rdStats, 0, sizeof(coeffGroupRDStats));
    rdStats.iNumSbbCtxBins = 0;

    for (int scanPosInSB = 0; scanPosInSB <= sbSizeM1; scanPosInSB++)
    {
      int lastPosCoded = sbSizeM1;
      scanPos = cctx.minSubPos() + scanPosInSB;
      //===== quantization =====
      uint32_t blkPos = cctx.blockPos(scanPos);

      const int posX = cctx.posX(scanPos);
      const int posY = cctx.posY(scanPos);
      const int posS = (1 == dirMode) ? posX : posY;
      const int posNb = (1 == dirMode) ? (posX - 1) + posY * coeffs.stride : posX + (posY - 1) * coeffs.stride;
      TCoeff predCoeff = (0 != posS) ? m_fullCoeff[posNb] : 0;

      // set coeff
      const int64_t          tmpLevel = int64_t(abs(srcCoeff[blkPos] - predCoeff)) * quantisationCoefficient;
      const Intermediate_Int levelDouble = (Intermediate_Int)std::min<int64_t>(tmpLevel, std::numeric_limits<Intermediate_Int>::max() - (Intermediate_Int(1) << (qBits - 1)));
      uint32_t roundAbsLevel = std::min<uint32_t>(uint32_t(entropyCodingMaximum), uint32_t((levelDouble + (Intermediate_Int(1) << (qBits - 1))) >> qBits));
      uint32_t minAbsLevel = (roundAbsLevel > 1 ? roundAbsLevel - 1 : 1);

      m_testedLevels = 0;
      coeffLevels[m_testedLevels++] = roundAbsLevel;

      if (minAbsLevel != roundAbsLevel)
        coeffLevels[m_testedLevels++] = minAbsLevel;

      double dErr = double(levelDouble);
      coeffLevelError[0]  = dErr * dErr * errorScale;

      costCoeff0[scanPos] = coeffLevelError[0];
      blockUncodedCost   += costCoeff0[scanPos];
      dstCoeff[blkPos]    = coeffLevels[0];

      //===== coefficient level estimation =====
      unsigned    ctxIdSig = cctx.sigCtxIdAbsTS(scanPos, dstCoeff);
      uint32_t    cLevel;
      const BinFracBits fracBitsPar = fracBits.getFracBitsArray(cctx.parityCtxIdAbsTS());

      goRiceParam = cctx.templateAbsSumTS(scanPos, dstCoeff);
      unsigned ctxIdSign = cctx.signCtxIdAbsTS(scanPos, dstCoeff, dirMode);
      const BinFracBits fracBitsSign = fracBits.getFracBitsArray(ctxIdSign);
      const uint8_t     sign = srcCoeff[blkPos] - predCoeff < 0 ? 1 : 0;
      unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(scanPos, dstCoeff, dirMode);
      const BinFracBits fracBitsGr1 = fracBits.getFracBitsArray(gt1CtxId);

      DTRACE_COND((dstCoeff[blkPos] != 0), g_trace_ctx, D_RDOQ_MORE, " uiCtxSig=%d", ctxIdSig);

      const BinFracBits fracBitsSig = fracBits.getFracBitsArray(ctxIdSig);
      bool lastCoeff = false; //
      if (scanPosInSB == lastPosCoded && noCoeffCoded == 0)
      {
        lastCoeff = true;
      }
      int rightPixel, belowPixel;
      cctx.neighTS(rightPixel, belowPixel, scanPos, dstCoeff);
      int numUsedCtxBins = 0;
      cLevel = xGetCodedLevelTSPred(costCoeff[scanPos], costCoeff0[scanPos], costSig[scanPos], levelDouble, qBits, errorScale, coeffLevels, coeffLevelError,
        &fracBitsSig, fracBitsPar, cctx, fracBits, fracBitsSign, fracBitsGr1, sign, rightPixel, belowPixel, goRiceParam, lastCoeff, extendedPrecision, maxLog2TrDynamicRange, numUsedCtxBins);
      cctx.decimateNumCtxBins(numUsedCtxBins);
      rdStats.iNumSbbCtxBins += numUsedCtxBins;

      if (cLevel > 0)
      {
        noCoeffCoded++;
      }
      dstCoeff[blkPos] = cLevel;

      if (sign)
      {
        dstCoeff[blkPos] = -dstCoeff[blkPos];
      }
      xDequantSample( m_fullCoeff[blkPos], dstCoeff[blkPos], trQuantParams );
      m_fullCoeff[blkPos] += predCoeff;

      baseCost += costCoeff[scanPos];
      rdStats.d64SigCost += costSig[scanPos];

      if (dstCoeff[blkPos])
      {
        cctx.setSigGroup();
        rdStats.d64CodedLevelandDist += costCoeff[scanPos] - costSig[scanPos];
        rdStats.d64UncodedDist += costCoeff0[scanPos];
      }
    } //end for (iScanPosinCG)

    if (!cctx.isSigGroup())
    {
      const BinFracBits fracBitsSigGroup = fracBits.getFracBitsArray(cctx.sigGroupCtxId(true));
      baseCost += xGetRateSigCoeffGroup(fracBitsSigGroup, 0) - rdStats.d64SigCost;
      costSigSubBlock[cctx.subSetId()] = xGetRateSigCoeffGroup(fracBitsSigGroup, 0);
      cctx.increaseNumCtxBins(rdStats.iNumSbbCtxBins); // skip sub-block
    }
    else if (sbId != sbNum - 1 || anySigCG)
    {
      // rd-cost if SigCoeffGroupFlag = 0, initialization
      double costZeroSB = baseCost;

      const BinFracBits fracBitsSigGroup = fracBits.getFracBitsArray(cctx.sigGroupCtxId(true));

      baseCost += xGetRateSigCoeffGroup(fracBitsSigGroup, 1);
      costZeroSB += xGetRateSigCoeffGroup(fracBitsSigGroup, 0);
      costSigSubBlock[cctx.subSetId()] = xGetRateSigCoeffGroup(fracBitsSigGroup, 1);

      costZeroSB += rdStats.d64UncodedDist;         // distortion for resetting non-zero levels to zero levels
      costZeroSB -= rdStats.d64CodedLevelandDist;   // distortion and level cost for keeping all non-zero levels
      costZeroSB -= rdStats.d64SigCost;             // sig cost for all coeffs, including zero levels and non-zerl levels

      if (costZeroSB < baseCost)
      {
        cctx.resetSigGroup();
        baseCost = costZeroSB;
        costSigSubBlock[cctx.subSetId()] = xGetRateSigCoeffGroup(fracBitsSigGroup, 0);
        cctx.increaseNumCtxBins(rdStats.iNumSbbCtxBins); // skip sub-block

        for (int scanPosInSB = 0; scanPosInSB <= sbSizeM1; scanPosInSB++)
        {
          scanPos = cctx.minSubPos() + scanPosInSB;
          uint32_t blkPos = cctx.blockPos(scanPos);

          const int posX = cctx.posX(scanPos);
          const int posY = cctx.posY(scanPos);
          const int posS = (1 == dirMode) ? posX : posY;
          const int posNb = (1 == dirMode) ? (posX - 1) + posY * coeffs.stride : posX + (posY - 1) * coeffs.stride;
          m_fullCoeff[scanPos] = (0 != posS) ? m_fullCoeff[posNb] : 0;

          if (dstCoeff[blkPos])
          {
            dstCoeff[blkPos] = 0;
            costCoeff[scanPos] = costCoeff0[scanPos];
            costSig[scanPos] = 0;
          }
        }
      }
      else
      {
        anySigCG = true;
      }
    }
  }

  //===== estimate last position =====
  for (int scanPos = 0; scanPos < maxNumCoeff; scanPos++)
  {
    int blkPos = cctx.blockPos(scanPos);
    TCoeff level = dstCoeff[blkPos];
    absSum += abs(level);
  }
}

void QuantRDOQ::xDequantSample(TCoeff& pRes, TCoeff& coeff, const TrQuantParams& trQuantParams)
{
  // xDequant
  if (trQuantParams.rightShift > 0)
  {
    const Intermediate_Int qAdd = Intermediate_Int(1) << (trQuantParams.rightShift - 1);
    pRes = TCoeff((Intermediate_Int(coeff) * trQuantParams.qScale + qAdd) >> trQuantParams.rightShift);
  }
  else
  {
    pRes = TCoeff((Intermediate_Int(coeff) * trQuantParams.qScale) << -trQuantParams.rightShift);
  }
}

inline uint32_t QuantRDOQ::xGetCodedLevelTSPred(double&            rd64CodedCost,
  double&            rd64CodedCost0,
  double&            rd64CodedCostSig,
  Intermediate_Int    levelDouble,
  int                 qBits,
  double              errorScale,
  uint32_t coeffLevels[],
  double coeffLevelError[],
  const BinFracBits* fracBitsSig,
  const BinFracBits& fracBitsPar,
  CoeffCodingContext& cctx,
  const FracBitsAccess& fracBitsAccess,
  const BinFracBits& fracBitsSign,
  const BinFracBits& fracBitsGt1,
  const uint8_t      sign,
  int                rightPixel,
  int                belowPixel,
  uint16_t           ricePar,
  bool               isLast,
  bool               useLimitedPrefixLength,
  const int          maxLog2TrDynamicRange,
  int&               numUsedCtxBins
) const
{
  double currCostSig = 0;
  uint32_t   bestAbsLevel = 0;
  numUsedCtxBins = 0;
  int numBestCtxBin = 0;
  if (!isLast && coeffLevels[0] < 3)
  {
    if (cctx.numCtxBins() >= 4)
    rd64CodedCostSig = xGetRateSigCoef(*fracBitsSig, 0);
    else
      rd64CodedCostSig = xGetICost(1 << SCALE_BITS);
    rd64CodedCost = rd64CodedCost0 + rd64CodedCostSig;
    if (cctx.numCtxBins() >= 4)
      numUsedCtxBins++;
    if (coeffLevels[0] == 0)
    {
      return bestAbsLevel;
    }
  }
  else
  {
    rd64CodedCost = MAX_DOUBLE;
  }

  if (!isLast)
  {
    if (cctx.numCtxBins() >= 4)
      currCostSig = xGetRateSigCoef(*fracBitsSig, 1);
    else
      currCostSig = xGetICost(1 << SCALE_BITS);
    if (coeffLevels[0] >= 3 && cctx.numCtxBins() >= 4)
      numUsedCtxBins++;
  }

  for (int errorInd = 1; errorInd <= m_testedLevels; errorInd++)
  {
    int absLevel = coeffLevels[errorInd - 1];
    double dErr = 0.0;
    dErr = double(levelDouble - (Intermediate_Int(absLevel) << qBits));
    coeffLevelError[errorInd] = dErr * dErr * errorScale;
    int modAbsLevel = absLevel;
    if (cctx.numCtxBins() >= 4) 
    {
      modAbsLevel = cctx.deriveModCoeff(rightPixel, belowPixel, absLevel, m_bdpcm);
    }
    int numCtxBins = 0;
    double dCurrCost = coeffLevelError[errorInd] + xGetICost(xGetICRateTS(modAbsLevel, fracBitsPar, cctx, fracBitsAccess, fracBitsSign, fracBitsGt1, numCtxBins, sign, ricePar, useLimitedPrefixLength, maxLog2TrDynamicRange));

    if (cctx.numCtxBins() >= 4)
      dCurrCost += currCostSig; // if cctx.numCtxBins < 4, xGetICRateTS return rate including sign cost. dont need to add any more

    if (dCurrCost < rd64CodedCost)
    {
      bestAbsLevel = absLevel;
      rd64CodedCost = dCurrCost;
      rd64CodedCostSig = currCostSig;
      numBestCtxBin = numCtxBins;
    }
  }
  numUsedCtxBins += numBestCtxBin;
  return bestAbsLevel;
}

inline int QuantRDOQ::xGetICRateTS( const uint32_t            absLevel,
                                    const BinFracBits&        fracBitsPar,
                                    const CoeffCodingContext& cctx,
                                    const FracBitsAccess&     fracBitsAccess,
                                    const BinFracBits&        fracBitsSign,
                                    const BinFracBits&        fracBitsGt1,
                                    int&                      numCtxBins,
                                    const uint8_t             sign,
                                    const uint16_t            ricePar,
                                    const bool                useLimitedPrefixLength,
                                    const int                 maxLog2TrDynamicRange  ) const
{
 
  if (cctx.numCtxBins() < 4) // Full by-pass coding 
  {
    int rate = absLevel ? (1 << SCALE_BITS) : 0; // 1 bit to signal sign of non-zero 

    uint32_t symbol = absLevel;

    uint32_t length;
    const int threshold = COEF_REMAIN_BIN_REDUCTION;
    if (symbol < (threshold << ricePar))
    {
      length = symbol >> ricePar;
      rate += (length + 1 + ricePar) << SCALE_BITS;
    }
    else if (useLimitedPrefixLength)
    {
      const uint32_t maximumPrefixLength = (32 - (COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange));

      uint32_t prefixLength = 0;
      uint32_t suffix = (symbol >> ricePar) - COEF_REMAIN_BIN_REDUCTION;

      while ((prefixLength < maximumPrefixLength) && (suffix > ((2 << prefixLength) - 2)))
      {
        prefixLength++;
      }

      const uint32_t suffixLength = (prefixLength == maximumPrefixLength) ? (maxLog2TrDynamicRange - ricePar) : (prefixLength + 1/*separator*/);

      rate += (COEF_REMAIN_BIN_REDUCTION + prefixLength + suffixLength + ricePar) << SCALE_BITS;
    }
    else
    {
      length = ricePar;
      symbol = symbol - (threshold << ricePar);
      while (symbol >= (1 << length))
      {
        symbol -= (1 << (length++));
      }
      rate += (threshold + length + 1 - ricePar + length) << SCALE_BITS;
    }

    return rate;
  }

  else if (cctx.numCtxBins() >= 4 && cctx.numCtxBins() < 8) // First pass context coding and all by-pass coding ( Sign flag is not counted here)
  {
    int rate = fracBitsSign.intBits[sign]; // sign bits
    if (absLevel)
      numCtxBins++;

    if (absLevel > 1)
    {
      rate += fracBitsGt1.intBits[1];
      rate += fracBitsPar.intBits[(absLevel - 2) & 1];

      numCtxBins += 2;

      int cutoffVal = 2;

      if (absLevel >= cutoffVal)
      {
        uint32_t symbol = (absLevel - cutoffVal) >> 1;
        uint32_t length;
        const int threshold = COEF_REMAIN_BIN_REDUCTION;
        if (symbol < (threshold << ricePar))
        {
          length = symbol >> ricePar;
          rate += (length + 1 + ricePar) << SCALE_BITS;
        }
        else if (useLimitedPrefixLength)
        {
          const uint32_t maximumPrefixLength = (32 - (COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange));

          uint32_t prefixLength = 0;
          uint32_t suffix = (symbol >> ricePar) - COEF_REMAIN_BIN_REDUCTION;

          while ((prefixLength < maximumPrefixLength) && (suffix > ((2 << prefixLength) - 2)))
          {
            prefixLength++;
          }

          const uint32_t suffixLength = (prefixLength == maximumPrefixLength) ? (maxLog2TrDynamicRange - ricePar) : (prefixLength + 1/*separator*/);

          rate += (COEF_REMAIN_BIN_REDUCTION + prefixLength + suffixLength + ricePar) << SCALE_BITS;
        }
        else
        {
          length = ricePar;
          symbol = symbol - (threshold << ricePar);
          while (symbol >= (1 << length))
          {
            symbol -= (1 << (length++));
          }
          rate += (threshold + length + 1 - ricePar + length) << SCALE_BITS;
        }
      }
    }
    else if (absLevel == 1)
    {
      rate += fracBitsGt1.intBits[0];
      numCtxBins++;
    }
    else
    {
      rate = 0;
    }
    return rate;
  }
    
  int rate = fracBitsSign.intBits[sign];

  if (absLevel)
    numCtxBins++;

  if( absLevel > 1 )
  {
    rate += fracBitsGt1.intBits[1];
    rate += fracBitsPar.intBits[( absLevel - 2 ) & 1];
    numCtxBins += 2;

          int cutoffVal = 2;
    const int numGtBins = 4;
    for( int i = 0; i < numGtBins; i++ )
    {
      if( absLevel >= cutoffVal )
      {
        const uint16_t ctxGtX = cctx.greaterXCtxIdAbsTS( cutoffVal>>1 );
        const BinFracBits &fracBitsGtX = fracBitsAccess.getFracBitsArray( ctxGtX );
        unsigned gtX = ( absLevel >= ( cutoffVal + 2 ) );
        rate += fracBitsGtX.intBits[gtX];
        numCtxBins++;
      }
      cutoffVal += 2;
    }

    if( absLevel >= cutoffVal )
    {
      uint32_t symbol = ( absLevel - cutoffVal ) >> 1;
      uint32_t length;
      const int threshold = COEF_REMAIN_BIN_REDUCTION;
      if( symbol < ( threshold << ricePar ) )
      {
        length = symbol >> ricePar;
        rate  += ( length + 1 + ricePar ) << SCALE_BITS;
      }
      else if( useLimitedPrefixLength )
      {
        const uint32_t maximumPrefixLength = ( 32 - ( COEF_REMAIN_BIN_REDUCTION + maxLog2TrDynamicRange ) );

        uint32_t prefixLength = 0;
        uint32_t suffix = ( symbol >> ricePar ) - COEF_REMAIN_BIN_REDUCTION;

        while( ( prefixLength < maximumPrefixLength ) && ( suffix > ( ( 2 << prefixLength ) - 2 ) ) )
        {
          prefixLength++;
        }

        const uint32_t suffixLength = ( prefixLength == maximumPrefixLength ) ? ( maxLog2TrDynamicRange - ricePar ) : ( prefixLength + 1/*separator*/ );

        rate += ( COEF_REMAIN_BIN_REDUCTION + prefixLength + suffixLength + ricePar ) << SCALE_BITS;
      }
      else
      {
        length = ricePar;
        symbol = symbol - ( threshold << ricePar );
        while( symbol >= ( 1 << length ) )
        {
          symbol -= ( 1 << ( length++ ) );
        }
        rate += ( threshold + length + 1 - ricePar + length ) << SCALE_BITS;
      }
    }
  }
  else if( absLevel == 1 )
  {
    rate += fracBitsGt1.intBits[0];
    numCtxBins++;
  }
  else
  {
    rate = 0;
  }
  return rate;
}

} // namespace vvenc

//! \}


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


/** \file     Quant.cpp
    \brief    transform and quantization class
*/

#include "Quant.h"
#include "UnitTools.h"
#include "ContextModelling.h"
#include "CodingStructure.h"
#include "dtrace_buffer.h"

#include <stdlib.h>
#include <memory.h>

//! \ingroup CommonLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Constants
// ====================================================================================================================


// ====================================================================================================================
// QpParam constructor
// ====================================================================================================================
 
QpParam::QpParam(const TransformUnit& tu, const ComponentID &compID, const bool allowACTQpoffset)
{
  const ChannelType chType = toChannelType( compID );
  const SPS        &sps    = *tu.cu->cs->sps;
  const int     qpBdOffset = sps.qpBDOffset[chType];
  const bool useJQP        = isChroma( compID ) && abs( TU::getICTMode( tu ) ) == 2;
  const ComponentID jCbCr  = useJQP ? COMP_JOINT_CbCr : compID;
  
        int chromaQpOffset = 0;

  if( isChroma( compID ) )
  {
    const PPS &pps  = *tu.cu->slice->pps;
    chromaQpOffset  = pps.chromaQpOffset              [jCbCr];
    chromaQpOffset += tu.cu->slice->sliceChromaQpDelta[jCbCr];
    chromaQpOffset += pps.getChromaQpOffsetListEntry( tu.cu->chromaQpAdj ).u.offset[int( jCbCr ) - 1];
  }
  
  int baseQp;
  int qpy        = tu.cu->qp;
  //bool skip      = tu.mtsIdx[compID] == MTS_SKIP;

  if( isLuma( compID ) )
  {
    baseQp = tu.cu->qp + qpBdOffset;
  }
  else
  {
    int qpi = Clip3( -qpBdOffset, MAX_QP, qpy );
    baseQp  = sps.chromaQpMappingTable.getMappedChromaQpValue( jCbCr, qpi );
    baseQp  = Clip3( -qpBdOffset, MAX_QP, baseQp + chromaQpOffset ) + qpBdOffset;
  }

  if( allowACTQpoffset && tu.cu->colorTransform )
  {
    baseQp += DELTA_QP_ACT[jCbCr];
  }

  baseQp = Clip3( 0, MAX_QP + qpBdOffset, baseQp );

  //if( !skip )
  {
    Qps [0] = baseQp;
    pers[0] = baseQp / 6;
    rems[0] = baseQp % 6;
  }
  //else
  {
    int internalMinusInputBitDepth = sps.internalMinusInputBitDepth[chType];
    int baseQpTS           = std::max( baseQp, 4 + 6 * internalMinusInputBitDepth );

    Qps [1] = baseQpTS;
    pers[1] = baseQpTS / 6;
    rems[1] = baseQpTS % 6;
  }
}


// ====================================================================================================================
// Quant class member functions
// ====================================================================================================================
static void DeQuantCore(const int maxX,const int maxY,const int scale,const TCoeff*const piQCoef,const size_t piQCfStride,TCoeff   *const piCoef,const int rightShift,const int inputMaximum,const TCoeff transformMaximum)
{
  const int inputMinimum = -(inputMaximum+1);
  const TCoeff transformMinimum = -(transformMaximum+1);
  if (rightShift>0)
  {
    const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);
    for( int y = 0, n = 0; y <= maxY; y++)
    {
      for( int x = 0; x <= maxX; x++, n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[x + y * piQCfStride]));
        Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale + iAdd) >> rightShift;
        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
  }
  else  // rightshift <0
  {
    int leftShift = -rightShift;
    for( int y = 0, n = 0; y <= maxY; y++)
    {
      for( int x = 0; x <= maxX; x++, n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[x + y * piQCfStride]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * scale) << leftShift;
        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
  }
}

Quant::Quant( const Quant* other ) : m_RDOQ( 0 ), m_useRDOQTS( false ), m_useSelectiveRDOQ( false ), m_dLambda( 0.0 )
{
  xInitScalingList( other );
  DeQuant=DeQuantCore;
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_QUANT
  initQuantX86();
#endif

}

Quant::~Quant()
{
  xDestroyScalingList();
}

void invResDPCM( const TransformUnit& tu, const ComponentID compID, CoeffBuf& dstBuf )
{
  const CompArea& rect = tu.blocks[compID];
  const int      wdt = rect.width;
  const int      hgt = rect.height;
  const CCoeffBuf coeffs = tu.getCoeffs(compID);

  const int      maxLog2TrDynamicRange = tu.cs->sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const TCoeff   inputMinimum   = -(1 << maxLog2TrDynamicRange);
  const TCoeff   inputMaximum   =  (1 << maxLog2TrDynamicRange) - 1;

  const TCoeff* coef = &coeffs.buf[0];
  TCoeff* dst = &dstBuf.buf[0];
  if ( tu.cu->bdpcmM[toChannelType(compID)] == 1)
  {
    for( int y = 0; y < hgt; y++ )
    {
      dst[0] = coef[0];
      for( int x = 1; x < wdt; x++ )
      {
        dst[x] = Clip3(inputMinimum, inputMaximum, dst[x - 1] + coef[x]);
      }
      coef += coeffs.stride;
      dst += dstBuf.stride;
    }
  }
  else
  {
    for( int x = 0; x < wdt; x++ )
    {
      dst[x] = coef[x];
    }
    for( int y = 0; y < hgt - 1; y++ )
    {
      for( int x = 0; x < wdt; x++ )
      {
        dst[dstBuf.stride + x] = Clip3(inputMinimum, inputMaximum, dst[x] + coef[coeffs.stride + x]);
      }
      coef += coeffs.stride;
      dst += dstBuf.stride;
    }
  }
}

void fwdResDPCM( TransformUnit& tu, const ComponentID compID )
{
  const CompArea& rect = tu.blocks[compID];
  const int      wdt = rect.width;
  const int      hgt = rect.height;
  CoeffBuf       coeffs = tu.getCoeffs(compID);

  TCoeff* coef = &coeffs.buf[0];
  if (tu.cu->bdpcmM[toChannelType(compID)] == 1)
  {
    for( int y = 0; y < hgt; y++ )
    {
      for( int x = wdt - 1; x > 0; x-- )
      {
        coef[x] -= coef[x - 1];
      }
      coef += coeffs.stride;
    }
  }
  else
  {
    coef += coeffs.stride * (hgt - 1);
    for( int y = 0; y < hgt - 1; y++ )
    {
      for ( int x = 0; x < wdt; x++ )
      {
        coef[x] -= coef[x - coeffs.stride];
      }
      coef -= coeffs.stride;
    }
  }
}

// To minimize the distortion only. No rate is considered.
void Quant::xSignBitHidingHDQ( TCoeff* pQCoef, const TCoeff* pCoef, TCoeff* deltaU, const CoeffCodingContext& cctx, const int maxLog2TrDynamicRange )
{
  const uint32_t width     = cctx.width();
  const uint32_t height    = cctx.height();
  const uint32_t groupSize = 1 << cctx.log2CGSize();

  const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
  const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

  int lastCG = -1;
  int absSum = 0 ;
  int n ;

  for( int subSet = (width*height-1) >> cctx.log2CGSize(); subSet >= 0; subSet-- )
  {
    int  subPos = subSet << cctx.log2CGSize();
    int  firstNZPosInCG=groupSize , lastNZPosInCG=-1 ;
    absSum = 0 ;

    for(n = groupSize-1; n >= 0; --n )
    {
      if( pQCoef[ cctx.blockPos( n + subPos ) ] )
      {
        lastNZPosInCG = n;
        break;
      }
    }

    for(n = 0; n <groupSize; n++ )
    {
      if( pQCoef[ cctx.blockPos( n + subPos ) ] )
      {
        firstNZPosInCG = n;
        break;
      }
    }

    for(n = firstNZPosInCG; n <=lastNZPosInCG; n++ )
    {
      absSum += int(pQCoef[ cctx.blockPos( n + subPos ) ]);
    }

    if(lastNZPosInCG>=0 && lastCG==-1)
    {
      lastCG = 1 ;
    }

    if( lastNZPosInCG-firstNZPosInCG>=SBH_THRESHOLD )
    {
      uint32_t signbit = (pQCoef[cctx.blockPos(subPos+firstNZPosInCG)]>0?0:1) ;
      if( signbit!=(absSum&0x1) )  //compare signbit with sum_parity
      {
        TCoeff curCost    = std::numeric_limits<TCoeff>::max();
        TCoeff minCostInc = std::numeric_limits<TCoeff>::max();
        int minPos =-1, finalChange=0, curChange=0;

        for( n = (lastCG==1?lastNZPosInCG:groupSize-1) ; n >= 0; --n )
        {
          uint32_t blkPos   = cctx.blockPos( n+subPos );
          if(pQCoef[ blkPos ] != 0 )
          {
            if(deltaU[blkPos]>0)
            {
              curCost = - deltaU[blkPos];
              curChange=1 ;
            }
            else
            {
              //curChange =-1;
              if(n==firstNZPosInCG && abs(pQCoef[blkPos])==1)
              {
                curCost = std::numeric_limits<TCoeff>::max();
              }
              else
              {
                curCost = deltaU[blkPos];
                curChange =-1;
              }
            }
          }
          else
          {
            if(n<firstNZPosInCG)
            {
              uint32_t thisSignBit = (pCoef[blkPos]>=0?0:1);
              if(thisSignBit != signbit )
              {
                curCost = std::numeric_limits<TCoeff>::max();
              }
              else
              {
                curCost = - (deltaU[blkPos])  ;
                curChange = 1 ;
              }
            }
            else
            {
              curCost = - (deltaU[blkPos])  ;
              curChange = 1 ;
            }
          }

          if( curCost<minCostInc)
          {
            minCostInc = curCost ;
            finalChange = curChange ;
            minPos = blkPos ;
          }
        } //CG loop

        if(pQCoef[minPos] == entropyCodingMaximum || pQCoef[minPos] == entropyCodingMinimum)
        {
          finalChange = -1;
        }

        if(pCoef[minPos]>=0)
        {
          pQCoef[minPos] += finalChange ;
        }
        else
        {
          pQCoef[minPos] -= finalChange ;
        }
      } // Hide
    }
    if(lastCG==1)
    {
      lastCG=0 ;
    }
  } // TU loop

  return;
}

void Quant::dequant(const TransformUnit& tu,
                             CoeffBuf&   dstCoeff,
                       const ComponentID compID,
                       const QpParam&    cQP)
{
  const SPS       *sps                  = tu.cs->sps;
  const CompArea  &area                 = tu.blocks[compID];
  const uint32_t  uiWidth               = area.width;
  const uint32_t  uiHeight              = area.height;
  TCoeff *const   piCoef                = dstCoeff.buf;
  size_t          piStride;
  const uint32_t  numSamplesInBlock     = uiWidth * uiHeight;
  const int       maxLog2TrDynamicRange = sps->getMaxLog2TrDynamicRange(toChannelType(compID));
  const TCoeff    transformMinimum      = -(1 << maxLog2TrDynamicRange);
  const TCoeff    transformMaximum      =  (1 << maxLog2TrDynamicRange) - 1;
  const bool      isTransformSkip       = tu.mtsIdx[compID] == MTS_SKIP;
  const bool      isLfnstApplied        = tu.cu->lfnstIdx > 0 && (CU::isSepTree(*tu.cu) ? true : isLuma(compID));
  const bool      enableScalingLists    = getUseScalingList(uiWidth, uiHeight, isTransformSkip, isLfnstApplied);
  const int       scalingListType       = getScalingListType(tu.cu->predMode, compID);
  const int       channelBitDepth       = sps->bitDepths[toChannelType(compID)];

  const TCoeff          *coef;
  if (tu.cu->bdpcmM[toChannelType(compID)])
  {
    invResDPCM( tu, compID, dstCoeff );
    coef = piCoef;
    piStride=dstCoeff.stride;
  }
  else
  {
    coef = tu.getCoeffs(compID).buf;
    piStride=tu.getCoeffs(compID).stride;
  }
  const TCoeff          *const piQCoef = coef;
  CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");

  // Represents scaling through forward transform
  const bool bClipTransformShiftTo0 = tu.mtsIdx[compID]!=MTS_SKIP && sps->spsRExt.extendedPrecisionProcessing;
  const int  originalTransformShift = getTransformShift(channelBitDepth, area.size(), maxLog2TrDynamicRange);
  const bool needSqrtAdjustment     = TU::needsSqrt2Scale( tu, compID );
  const int  iTransformShift        = (bClipTransformShiftTo0 ? std::max<int>(0, originalTransformShift) : originalTransformShift) + (needSqrtAdjustment?-1:0);

  const int QP_per = cQP.per(isTransformSkip);
  const int QP_rem = cQP.rem(isTransformSkip);

  const int  rightShift = (IQUANT_SHIFT - ((isTransformSkip ? 0 : iTransformShift) + QP_per)) + (enableScalingLists ? LOG2_SCALING_LIST_NEUTRAL_VALUE : 0);

  if(enableScalingLists)
  {
    //from the dequantization equation:
    //iCoeffQ                         = ((Intermediate_Int(clipQCoef) * piDequantCoef[deQuantIdx]) + iAdd ) >> rightShift
    //(sizeof(Intermediate_Int) * 8)  =              inputBitDepth    +    dequantCoefBits                   - rightShift
    const uint32_t             dequantCoefBits     = 1 + IQUANT_SHIFT + SCALING_LIST_BITS;
    const uint32_t             targetInputBitDepth = std::min<uint32_t>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - dequantCoefBits));

    const Intermediate_Int inputMinimum        = -(1 << (targetInputBitDepth - 1));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;

    const uint32_t uiLog2TrWidth  = Log2(uiWidth);
    const uint32_t uiLog2TrHeight = Log2(uiHeight);
    int* piDequantCoef            = getDequantCoeff(scalingListType, QP_rem, uiLog2TrWidth, uiLog2TrHeight);

    if(rightShift > 0)
    {
      const Intermediate_Int iAdd = (Intermediate_Int) 1 << (rightShift - 1);
      for( int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = ((Intermediate_Int(clipQCoef) * piDequantCoef[n]) + iAdd ) >> rightShift;
        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
    else
    {
      const int leftShift = -rightShift;
      for( int n = 0; n < numSamplesInBlock; n++ )
      {
        const TCoeff           clipQCoef = TCoeff(Clip3<Intermediate_Int>(inputMinimum, inputMaximum, piQCoef[n]));
        const Intermediate_Int iCoeffQ   = (Intermediate_Int(clipQCoef) * piDequantCoef[n]) << leftShift;
        piCoef[n] = TCoeff(Clip3<Intermediate_Int>(transformMinimum,transformMaximum,iCoeffQ));
      }
    }
  }
  else
  {
    const int scale     = g_invQuantScales[needSqrtAdjustment?1:0][QP_rem];
    const int scaleBits = ( IQUANT_SHIFT + 1 );
    //from the dequantisation equation:
    //iCoeffQ                         = Intermediate_Int((int64_t(clipQCoef) * scale + iAdd) >> rightShift);
    //(sizeof(Intermediate_Int) * 8)  =                    inputBitDepth   + scaleBits      - rightShift
    const uint32_t             targetInputBitDepth = std::min<uint32_t>((maxLog2TrDynamicRange + 1), (((sizeof(Intermediate_Int) * 8) + rightShift) - scaleBits));
    const Intermediate_Int inputMaximum        =  (1 << (targetInputBitDepth - 1)) - 1;
    DeQuant(uiWidth-1,uiHeight-1,scale,piQCoef,piStride,piCoef,rightShift,inputMaximum,transformMaximum);
  }
}

void Quant::init( int rdoq, bool bUseRDOQTS, bool useSelectiveRDOQ, int thrDqVal )
{

  // TODO: pass to init() a single variable containing (quantization) flags,
  //       instead of variables that don't have to do with this class

  m_RDOQ             = rdoq;
  m_useRDOQTS        = bUseRDOQTS;
  m_useSelectiveRDOQ = useSelectiveRDOQ;
}

/** set flat matrix value to quantized coefficient
 */
void Quant::setFlatScalingList(const int maxLog2TrDynamicRange[MAX_NUM_CH], const BitDepths &bitDepths)
{
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
          xSetFlatScalingList( list, sizeX, sizeY, qp );
        }
      }
    }
  }
}

/** set flat matrix value to quantized coefficient
 * \param list List ID
 * \param size size index
 * \param qp Quantization parameter
 * \param format chroma format
 */
void Quant::xSetFlatScalingList(uint32_t list, uint32_t sizeX, uint32_t sizeY, int qp)
{
  uint32_t i,num = g_scalingListSizeX[sizeX]*g_scalingListSizeX[sizeY];
  int *quantcoeff;
  int *dequantcoeff;

  const bool blockIsNotPowerOf4 = ((Log2(g_scalingListSizeX[sizeX] * g_scalingListSizeX[sizeY])) & 1) == 1;
  int quantScales    = g_quantScales   [blockIsNotPowerOf4?1:0][qp];
  int invQuantScales = g_invQuantScales[blockIsNotPowerOf4?1:0][qp] << 4;

  quantcoeff   = getQuantCoeff(list, qp, sizeX, sizeY);
  dequantcoeff = getDequantCoeff(list, qp, sizeX, sizeY);

  for(i=0;i<num;i++)
  {
    *quantcoeff++ = quantScales;
    *dequantcoeff++ = invQuantScales;
  }
}


/** initialization process of scaling list array
 */
void Quant::xInitScalingList( const Quant* other )
{
  m_isScalingListOwner = other == nullptr;
  m_scalingListEnabled = false;

  for(uint32_t sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++)
  {
    for(uint32_t sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++)
    {
      for(uint32_t qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
      {
        for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
        {
          if( m_isScalingListOwner )
          {
            m_quantCoef   [sizeIdX][sizeIdY][listId][qp] = new int    [g_scalingListSizeX[sizeIdX]*g_scalingListSizeX[sizeIdY]];
            m_dequantCoef [sizeIdX][sizeIdY][listId][qp] = new int    [g_scalingListSizeX[sizeIdX]*g_scalingListSizeX[sizeIdY]];
          }
          else
          {
            m_quantCoef   [sizeIdX][sizeIdY][listId][qp] = other->m_quantCoef   [sizeIdX][sizeIdY][listId][qp];
            m_dequantCoef [sizeIdX][sizeIdY][listId][qp] = other->m_dequantCoef [sizeIdX][sizeIdY][listId][qp];
          }
        } // listID loop
      }
    }
  }
}

/** destroy quantization matrix array
 */
void Quant::xDestroyScalingList()
{
  if( !m_isScalingListOwner ) return;

  for(uint32_t sizeIdX = 0; sizeIdX < SCALING_LIST_SIZE_NUM; sizeIdX++)
  {
    for(uint32_t sizeIdY = 0; sizeIdY < SCALING_LIST_SIZE_NUM; sizeIdY++)
    {
      for(uint32_t listId = 0; listId < SCALING_LIST_NUM; listId++)
      {
        for(uint32_t qp = 0; qp < SCALING_LIST_REM_NUM; qp++)
        {
          if(m_quantCoef[sizeIdX][sizeIdY][listId][qp])
          {
            delete [] m_quantCoef[sizeIdX][sizeIdY][listId][qp];
          }
          if(m_dequantCoef[sizeIdX][sizeIdY][listId][qp])
          {
            delete [] m_dequantCoef[sizeIdX][sizeIdY][listId][qp];
          }
        }
      }
    }
  }
}

void Quant::quant(TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, TCoeff &uiAbsSum, const QpParam& cQP, const Ctx& ctx)
{
  const SPS &sps            = *tu.cs->sps;
  const CompArea& rect      = tu.blocks[compID];
  const uint32_t uiWidth        = rect.width;
  const uint32_t uiHeight       = rect.height;
  const int channelBitDepth = sps.bitDepths[toChannelType(compID)];

  const CCoeffBuf& piCoef   = pSrc;
        CoeffBuf   piQCoef  = tu.getCoeffs(compID);

  const bool useTransformSkip = tu.mtsIdx[compID] == MTS_SKIP;
  const int  maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(toChannelType(compID));

  {
    CoeffCodingContext cctx(tu, compID, tu.cs->slice->signDataHidingEnabled);

    const TCoeff entropyCodingMinimum = -(1 << maxLog2TrDynamicRange);
    const TCoeff entropyCodingMaximum =  (1 << maxLog2TrDynamicRange) - 1;

    TCoeff deltaU[MAX_TB_SIZEY * MAX_TB_SIZEY];
    int scalingListType           = getScalingListType(tu.cu->predMode, compID);
    CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");
    const uint32_t uiLog2TrWidth  = Log2(uiWidth);
    const uint32_t uiLog2TrHeight = Log2(uiHeight);
    int *piQuantCoeff             = getQuantCoeff(scalingListType, cQP.rem(useTransformSkip), uiLog2TrWidth, uiLog2TrHeight);

    const bool isLfnstApplied     = tu.cu->lfnstIdx > 0 && (CU::isSepTree(*tu.cu) ? true : isLuma(compID));
    const bool enableScalingLists = getUseScalingList(uiWidth, uiHeight, useTransformSkip, isLfnstApplied);

    // for blocks that where width*height != 4^N, the effective scaling applied during transformation cannot be
    // compensated by a bit-shift (the quantised result will be sqrt(2) * larger than required).
    // The quantScale table and shift is used to compensate for this.
    const bool needSqrtAdjustment= TU::needsSqrt2Scale( tu, compID );
    const int defaultQuantisationCoefficient    = g_quantScales[needSqrtAdjustment?1:0][cQP.rem(useTransformSkip)];
    int iTransformShift = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange) + ( needSqrtAdjustment?-1:0);

    if (useTransformSkip && sps.spsRExt.extendedPrecisionProcessing)
    {
      iTransformShift = std::max<int>(0, iTransformShift);
    }

    const int iQBits = QUANT_SHIFT + cQP.per(useTransformSkip) + (useTransformSkip ? 0 : iTransformShift);
    // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

    const int64_t iAdd = int64_t(tu.cs->slice->isIRAP() ? 171 : 85) << int64_t(iQBits - 9);
    const int qBits8 = iQBits - 8;

    const uint32_t lfnstIdx = tu.cu->lfnstIdx;
    const int maxNumberOfCoeffs = lfnstIdx > 0 ? ((( uiWidth == 4 && uiHeight == 4 ) || ( uiWidth == 8 && uiHeight == 8) ) ? 8 : 16) : piQCoef.area();
    memset( piQCoef.buf, 0, sizeof(TCoeff) * piQCoef.area() );
    for (int uiBlockPos = 0; uiBlockPos < maxNumberOfCoeffs; uiBlockPos++ )
    {
      const TCoeff iLevel   = piCoef.buf[uiBlockPos];
      const TCoeff iSign    = (iLevel < 0 ? -1: 1);

      const int64_t  tmpLevel = (int64_t)abs(iLevel) * (enableScalingLists ? piQuantCoeff[uiBlockPos] : defaultQuantisationCoefficient);

      const TCoeff quantisedMagnitude = TCoeff((tmpLevel + iAdd ) >> iQBits);
      deltaU[uiBlockPos] = (TCoeff)((tmpLevel - ((int64_t)quantisedMagnitude<<iQBits) )>> qBits8);

      uiAbsSum += quantisedMagnitude;
      const TCoeff quantisedCoefficient = quantisedMagnitude * iSign;

      piQCoef.buf[uiBlockPos] = Clip3<TCoeff>( entropyCodingMinimum, entropyCodingMaximum, quantisedCoefficient );
    } // for n
    if (tu.cu->bdpcmM[toChannelType(compID)])
    {
      fwdResDPCM( tu, compID );
    }
    if( cctx.signHiding() )
    {
      if(uiAbsSum >= 2) //this prevents TUs with only one coefficient of value 1 from being tested
      {
        xSignBitHidingHDQ(piQCoef.buf, piCoef.buf, deltaU, cctx, maxLog2TrDynamicRange);
      }
    }
    int lastScanPos = -1;
    for( int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
    {
      unsigned blkPos = cctx.blockPos( scanPos );
      if( piQCoef.buf[blkPos] )
      {
        lastScanPos = scanPos;
      }
    }
    tu.lastPos[compID] = lastScanPos;
  } //if RDOQ
  //return;
}

bool Quant::xNeedRDOQ(TransformUnit& tu, const ComponentID compID, const CCoeffBuf& pSrc, const QpParam& cQP)
{
  const SPS &sps            = *tu.cs->sps;
  const CompArea& rect      = tu.blocks[compID];
  const uint32_t uiWidth        = rect.width;
  const uint32_t uiHeight       = rect.height;
  const int channelBitDepth = sps.bitDepths[toChannelType(compID)];

  const CCoeffBuf piCoef    = pSrc;

  const bool useTransformSkip = tu.mtsIdx[compID] == MTS_SKIP;
  const int  maxLog2TrDynamicRange = sps.getMaxLog2TrDynamicRange(toChannelType(compID));

  int scalingListType = getScalingListType(tu.cu->predMode, compID);
  CHECK(scalingListType >= SCALING_LIST_NUM, "Invalid scaling list");

  const uint32_t uiLog2TrWidth  = Log2(uiWidth);
  const uint32_t uiLog2TrHeight = Log2(uiHeight);
  int *piQuantCoeff             = getQuantCoeff(scalingListType, cQP.rem(useTransformSkip), uiLog2TrWidth, uiLog2TrHeight);

  const bool isLfnstApplied     = tu.cu->lfnstIdx > 0 && (CU::isSepTree(*tu.cu) ? true : isLuma(compID));
  const bool enableScalingLists = getUseScalingList(uiWidth, uiHeight, (useTransformSkip != 0), isLfnstApplied);

  /* for 422 chroma blocks, the effective scaling applied during transformation is not a power of 2, hence it cannot be
    * implemented as a bit-shift (the quantised result will be sqrt(2) * larger than required). Alternatively, adjust the
    * uiLog2TrSize applied in iTransformShift, such that the result is 1/sqrt(2) the required result (i.e. smaller)
    * Then a QP+3 (sqrt(2)) or QP-3 (1/sqrt(2)) method could be used to get the required result
    */
  const bool needSqrtAdjustment= TU::needsSqrt2Scale( tu, compID );
  const int defaultQuantisationCoefficient    = g_quantScales[needSqrtAdjustment?1:0][cQP.rem(useTransformSkip)];
  int iTransformShift = getTransformShift(channelBitDepth, rect.size(), maxLog2TrDynamicRange) + (needSqrtAdjustment?-1:0);

  if (useTransformSkip && sps.spsRExt.extendedPrecisionProcessing)
  {
    iTransformShift = std::max<int>(0, iTransformShift);
  }


  const int iQBits = QUANT_SHIFT + cQP.per(useTransformSkip) + iTransformShift;
  assert(iQBits>=0);
  // QBits will be OK for any internal bit depth as the reduction in transform shift is balanced by an increase in Qp_per due to QpBDOffset

  // iAdd is different from the iAdd used in normal quantization
  const int64_t iAdd = int64_t(compID == COMP_Y ? 171 : 256) << (iQBits - 9);

  for (int uiBlockPos = 0; uiBlockPos < rect.area(); uiBlockPos++)
  {
    const TCoeff iLevel   = piCoef.buf[uiBlockPos];
    const int64_t  tmpLevel = (int64_t)abs(iLevel) * (enableScalingLists ? piQuantCoeff[uiBlockPos] : defaultQuantisationCoefficient);
    const TCoeff quantisedMagnitude = TCoeff((tmpLevel + iAdd ) >> iQBits);

    if (quantisedMagnitude != 0)
    {
      return true;
    }
  } // for n
  return false;
}

} // namespace vvenc

//! \}


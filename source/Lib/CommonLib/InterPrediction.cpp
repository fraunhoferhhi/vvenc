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


/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "InterPrediction.h"
#include "Unit.h"
#include "UnitTools.h"
#include "dtrace_next.h"
#include "dtrace_buffer.h"
#include "CommonLib/TimeProfiler.h"

#include <memory.h>
#include <algorithm>

//! \ingroup CommonLib
//! \{

namespace vvenc {

void addBDOFAvgCore(const Pel* src0, int src0Stride, const Pel* src1, int src1Stride, Pel* dst, int dstStride, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0, const Pel*gradY1, int gradStride, int width, int height, int tmpx, int tmpy, unsigned shift, int offset, const ClpRng& clpRng)
{
  int b = 0;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x += 4)
    {
      b = tmpx * (gradX0[x] - gradX1[x]) + tmpy * (gradY0[x] - gradY1[x]);
      dst[x] = ClipPel((int16_t)rightShiftU((src0[x] + src1[x] + b + offset), shift), clpRng);

      b = tmpx * (gradX0[x + 1] - gradX1[x + 1]) + tmpy * (gradY0[x + 1] - gradY1[x + 1]);
      dst[x + 1] = ClipPel((int16_t)rightShiftU((src0[x + 1] + src1[x + 1] + b + offset), shift), clpRng);

      b = tmpx * (gradX0[x + 2] - gradX1[x + 2]) + tmpy * (gradY0[x + 2] - gradY1[x + 2]);
      dst[x + 2] = ClipPel((int16_t)rightShiftU((src0[x + 2] + src1[x + 2] + b + offset), shift), clpRng);

      b = tmpx * (gradX0[x + 3] - gradX1[x + 3]) + tmpy * (gradY0[x + 3] - gradY1[x + 3]);
      dst[x + 3] = ClipPel((int16_t)rightShiftU((src0[x + 3] + src1[x + 3] + b + offset), shift), clpRng);
    }
    dst += dstStride;       src0 += src0Stride;     src1 += src1Stride;
    gradX0 += gradStride; gradX1 += gradStride; gradY0 += gradStride; gradY1 += gradStride;
  }
}

template<bool PAD = true>
void gradFilterCore(const Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY, const int bitDepth)
{
  const Pel* srcTmp = pSrc + srcStride + 1;
  Pel* gradXTmp = gradX + gradStride + 1;
  Pel* gradYTmp = gradY + gradStride + 1;
  int  shift1 = 6;

  for (int y = 0; y < (height - 2 * BDOF_EXTEND_SIZE); y++)
  {
    for (int x = 0; x < (width - 2 * BDOF_EXTEND_SIZE); x++)
    {
      gradYTmp[x] = ( srcTmp[x + srcStride] >> shift1 ) - ( srcTmp[x - srcStride] >> shift1 );
      gradXTmp[x] = ( srcTmp[x + 1] >> shift1 ) - ( srcTmp[x - 1] >> shift1 );
    }
    gradXTmp += gradStride;
    gradYTmp += gradStride;
    srcTmp += srcStride;
  }

  if (PAD)
  {
    gradXTmp = gradX + gradStride + 1;
    gradYTmp = gradY + gradStride + 1;
    for (int y = 0; y < (height - 2 * BDOF_EXTEND_SIZE); y++)
    {
      gradXTmp[-1] = gradXTmp[0];
      gradXTmp[width - 2 * BDOF_EXTEND_SIZE] = gradXTmp[width - 2 * BDOF_EXTEND_SIZE - 1];
      gradXTmp += gradStride;

      gradYTmp[-1] = gradYTmp[0];
      gradYTmp[width - 2 * BDOF_EXTEND_SIZE] = gradYTmp[width - 2 * BDOF_EXTEND_SIZE - 1];
      gradYTmp += gradStride;
    }

    gradXTmp = gradX + gradStride;
    gradYTmp = gradY + gradStride;
    ::memcpy(gradXTmp - gradStride, gradXTmp, sizeof(Pel)*(width));
    ::memcpy(gradXTmp + (height - 2 * BDOF_EXTEND_SIZE)*gradStride, gradXTmp + (height - 2 * BDOF_EXTEND_SIZE - 1)*gradStride, sizeof(Pel)*(width));
    ::memcpy(gradYTmp - gradStride, gradYTmp, sizeof(Pel)*(width));
    ::memcpy(gradYTmp + (height - 2 * BDOF_EXTEND_SIZE)*gradStride, gradYTmp + (height - 2 * BDOF_EXTEND_SIZE - 1)*gradStride, sizeof(Pel)*(width));
  }
}

void calcBDOFSumsCore(const Pel* srcY0Tmp, const Pel* srcY1Tmp, Pel* gradX0, Pel* gradX1, Pel* gradY0, Pel* gradY1, int xu, int yu, const int src0Stride, const int src1Stride, const int widthG, const int bitDepth, int* sumAbsGX, int* sumAbsGY, int* sumDIX, int* sumDIY, int* sumSignGY_GX)
{
  int shift4 = 4;
  int shift5 = 1;

  for (int y = 0; y < 6; y++)
  {
    for (int x = 0; x < 6; x++)
    {
      int tmpGX = (gradX0[x] + gradX1[x]) >> shift5;
      int tmpGY = (gradY0[x] + gradY1[x]) >> shift5;
      int tmpDI = (int)((srcY1Tmp[x] >> shift4) - (srcY0Tmp[x] >> shift4));
      *sumAbsGX += (tmpGX < 0 ? -tmpGX : tmpGX);
      *sumAbsGY += (tmpGY < 0 ? -tmpGY : tmpGY);
      *sumDIX += (tmpGX < 0 ? -tmpDI : (tmpGX == 0 ? 0 : tmpDI));
      *sumDIY += (tmpGY < 0 ? -tmpDI : (tmpGY == 0 ? 0 : tmpDI));
      *sumSignGY_GX += (tmpGY < 0 ? -tmpGX : (tmpGY == 0 ? 0 : tmpGX));

    }
    srcY1Tmp += src1Stride;
    srcY0Tmp += src0Stride;
    gradX0 += widthG;
    gradX1 += widthG;
    gradY0 += widthG;
    gradY1 += widthG;
  }
}

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

InterPrediction::InterPrediction()
  : m_currChromaFormat( NUM_CHROMA_FORMAT )
  , m_subPuMC(false)
{
}

InterPrediction::~InterPrediction()
{
  destroy();
}

void InterPrediction::destroy()
{
  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    m_yuvPred[i].destroy();
  }
  m_geoPartBuf[0].destroy();
  m_geoPartBuf[1].destroy();
}

void InterPrediction::init( RdCost* pcRdCost, ChromaFormat chFormat, const int ctuSize )
{
  // if it has been initialised before, but the chroma format has changed, release the memory and start again.
  if( m_yuvPred[L0].getOrigin( COMP_Y ) != nullptr && m_currChromaFormat != chFormat )
  {
    destroy();
    DMVR::destroy();
    InterPredInterpolation::destroy();
  }

  m_currChromaFormat = chFormat;

  if( m_yuvPred[L0].getOrigin( COMP_Y ) == nullptr )
  {
    for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
    {
      m_yuvPred[i].create( chFormat, Area{ 0, 0, (int)MAX_CU_SIZE, (int)MAX_CU_SIZE }, 0, 0, 32 );
    }

    InterPredInterpolation::init();
    DMVR::init( pcRdCost, chFormat );
    m_geoPartBuf[0].create(UnitArea(chFormat, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
    m_geoPartBuf[1].create(UnitArea(chFormat, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool InterPrediction::xCheckIdenticalMotion( const PredictionUnit &pu ) const
{
  const Slice &slice = *pu.cs->slice;

  if( slice.isInterB() && !pu.cs->pps->weightedBiPred )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->getPOC();
      int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
        if( !pu.cu->affine )
        {
          if( pu.mv[0] == pu.mv[1] )
          {
            return true;
          }
        }
        else
        {
          if ( (pu.cu->affineType == AFFINEMODEL_4PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]))
            || (pu.cu->affineType == AFFINEMODEL_6PARAM && (pu.mvAffi[0][0] == pu.mvAffi[1][0]) && (pu.mvAffi[0][1] == pu.mvAffi[1][1]) && (pu.mvAffi[0][2] == pu.mvAffi[1][2])) )
          {
            return true;
          }
        }
      }
    }
  }

  return false;
}

void InterPrediction::xSubPuBDOF( const PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList& refPicList /*= REF_PIC_LIST_X*/)
{
  // compute the location of the current PU
  Position puPos = pu.lumaPos();
  Size puSize = pu.lumaSize();

  PredictionUnit subPu;
  subPu.cs             = pu.cs;
  subPu.cu             = pu.cu;
  subPu.mergeType      = pu.mergeType;
  subPu.mmvdMergeFlag  = pu.mmvdMergeFlag;
  subPu.mcControl      = pu.mcControl;
  subPu.mergeFlag      = pu.mergeFlag;
  subPu.ciip           = pu.ciip;
  subPu.mvRefine       = pu.mvRefine;
  subPu.refIdx[0]      = pu.refIdx[0];
  subPu.refIdx[1]      = pu.refIdx[1];

  const int  yEnd      = puPos.y + puSize.height;
  const int  xEnd      = puPos.x + puSize.width;
  const int  dy        = std::min((int)MAX_BDOF_APPLICATION_REGION, (int)puSize.height);
  const int  dx        = std::min((int)MAX_BDOF_APPLICATION_REGION, (int)puSize.width);
  for (int y = puPos.y; y < yEnd; y += dy)
  {
    for (int x = puPos.x; x < xEnd; x += dx)
    {
      const MotionInfo &curMi = pu.getMotionInfo(Position{ x, y });

      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
      subPu = curMi;
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(pu, subPu));

      motionCompensation(subPu, subPredBuf, refPicList);
    }
  }
}
void InterPrediction::xPredInterUni(const PredictionUnit& pu, const RefPicList& refPicList, PelUnitBuf& pcYuvPred, const bool bi, const bool bdofApplied)
{
  int iRefIdx = pu.refIdx[refPicList];
  Mv mv[3];
  bool isIBC = false;
  if (pu.cu->affine)
  {
    CHECK(iRefIdx < 0, "iRefIdx incorrect.");

    mv[0] = pu.mvAffi[refPicList][0];
    mv[1] = pu.mvAffi[refPicList][1];
    mv[2] = pu.mvAffi[refPicList][2];
  }
  else
  {
    mv[0] = pu.mv[refPicList];
    clipMv(mv[0], pu.cu->lumaPos(), pu.cu->lumaSize(), *pu.cs->pcv);
  }

  for( uint32_t comp = COMP_Y; comp < pcYuvPred.bufs.size(); comp++ )
  {
    bool luma   = pu.mcControl > 3         ? false : true;
    bool chroma = (pu.mcControl >> 1) == 1 ? false : true;
    const ComponentID compID = ComponentID( comp );
    if (compID == COMP_Y && !luma)
      continue;
    if (compID != COMP_Y && !chroma)
      continue;
    if (pu.cu->affine)
    {
      xPredAffineBlk(compID, pu, pu.cu->slice->getRefPic(refPicList, iRefIdx), mv, pcYuvPred, bi, pu.cu->slice->clpRngs[compID], refPicList);
    }
    else
    {
      xPredInterBlk(compID, pu, pu.cu->slice->getRefPic(refPicList, iRefIdx), mv[0], pcYuvPred, bi, pu.cu->slice->clpRngs[compID], bdofApplied, isIBC, refPicList);
    }
  }
}

void InterPrediction::xPredInterBi( const PredictionUnit& pu, PelUnitBuf& yuvPred, const bool bdofApplied )
{
  CHECK( !pu.cu->affine && pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 && ( pu.lwidth() + pu.lheight() == 12 ), "invalid 4x8/8x4 bi-predicted blocks" );

  PelUnitBuf puBuf[NUM_REF_PIC_LIST_01];
  for (uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if( pu.refIdx[refList] < 0)
    {
      continue;
    }

    RefPicList refPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

    CHECK(CU::isIBC(*pu.cu) && refPicList != REF_PIC_LIST_0, "Invalid interdir for ibc mode");
    CHECK(CU::isIBC(*pu.cu) && pu.refIdx[refList] != MAX_NUM_REF, "Invalid reference index for ibc mode");
    CHECK((CU::isInter(*pu.cu) && pu.refIdx[refList] >= pu.cs->slice->numRefIdx[ refPicList ]), "Invalid reference index");

    puBuf[refList] = m_yuvPred[refList].getCompactBuf( pu );

    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      xPredInterUni ( pu, refPicList, puBuf[refList], true, bdofApplied );
    }
    else
    {
      xPredInterUni( pu, refPicList, puBuf[refList], pu.cu->geo, bdofApplied );
    }
  }

  xWeightedAverage( pu, puBuf[0], puBuf[1], yuvPred, bdofApplied );
}

void InterPrediction::motionCompensationIBC( PredictionUnit &pu, PelUnitBuf& predBuf )
{
  // dual tree handling for IBC as the only ref
  xPredInterUni(pu, REF_PIC_LIST_0, predBuf, false, false );
}

bool InterPrediction::motionCompensation( PredictionUnit &pu, PelUnitBuf& predBuf, const RefPicList refPicList)
{
  bool ret = false;
  if( refPicList != REF_PIC_LIST_X )
  {
    xPredInterUni( pu, refPicList, predBuf, false, false );
  }
  else
  {
    CHECK( !pu.cu->affine && pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 && ( pu.lwidth() + pu.lheight() == 12 ), "invalid 4x8/8x4 bi-predicted blocks" );
    bool bdofApplied = false;
    if( pu.cs->sps->BDOF && ( !pu.cs->picHeader->disBdofFlag ) )
    {
      if (pu.cu->affine || m_subPuMC || pu.ciip )
      {
        bdofApplied = false;
      }
      else
      {
        if (PU::isBiPredFromDifferentDirEqDistPoc(pu)
          && (pu.Y().height >= 8)
          && (pu.Y().width >= 8)
          && ((pu.Y().height * pu.Y().width) >= 128)
          && !(pu.cu->smvdMode)
          && !(pu.cu->cs->sps->BCW && pu.cu->BcwIdx != BCW_DEFAULT)
          && !(((pu.mcControl & 1) == 1) && pu.mmvdMergeFlag)
          )
        {
          bdofApplied = true;
        }
      }
    }

    bool dmvrApplied = false;
    dmvrApplied = (pu.mvRefine) && PU::checkDMVRCondition(pu);
    if ((pu.lumaSize().width > MAX_BDOF_APPLICATION_REGION || pu.lumaSize().height > MAX_BDOF_APPLICATION_REGION) && pu.mergeType != MRG_TYPE_SUBPU_ATMVP && (bdofApplied && !dmvrApplied))
    {
      xSubPuBDOF( pu, predBuf, refPicList );
    }
    else if (pu.mergeType != MRG_TYPE_DEFAULT_N /*&& pu.mergeType != MRG_TYPE_IBC*/)
    {
      xSubPuMC(pu, predBuf, refPicList);
    }
    else if( xCheckIdenticalMotion( pu ) )
    {
      xPredInterUni( pu, REF_PIC_LIST_0, predBuf, false, false );
    }
    else if (dmvrApplied)
    {
      xProcessDMVR( pu, predBuf, pu.cu->slice->clpRngs, bdofApplied );
    }
    else
    {
      xPredInterBi( pu, predBuf, bdofApplied );
    }
    DTRACE( g_trace_ctx, D_MOT_COMP, "BIDOF=%d, DMVR=%d\n", bdofApplied, dmvrApplied );
    ret = bdofApplied || dmvrApplied;
  }
  DTRACE( g_trace_ctx, D_MOT_COMP, "MV=%d,%d\n", pu.mv[0].hor, pu.mv[0].ver );
  DTRACE( g_trace_ctx, D_MOT_COMP, "MV=%d,%d\n", pu.mv[1].hor, pu.mv[1].ver );
  DTRACE_PEL_BUF( D_MOT_COMP, predBuf.Y(), pu, pu.cu->predMode, COMP_Y );
  DTRACE_PEL_BUF( D_MOT_COMP, predBuf.Cb(), pu, pu.cu->predMode, COMP_Cb );
  DTRACE_PEL_BUF( D_MOT_COMP, predBuf.Cr(), pu, pu.cu->predMode, COMP_Cr );
  return ret;
}

void InterPrediction::xSubPuMC(PredictionUnit& pu, PelUnitBuf& predBuf, const RefPicList& eRefPicList /*= REF_PIC_LIST_X*/)
{
  // compute the location of the current PU
  Position puPos = pu.lumaPos();
  Size puSize = pu.lumaSize();

  int numPartLine, numPartCol, puHeight, puWidth;
  {
    numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
    numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
    puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
    puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;
  }

  PredictionUnit subPu = pu;

  subPu.cs = pu.cs;
//  subPu.cu = pu.cu;
  subPu.mergeType = MRG_TYPE_DEFAULT_N;

  bool isAffine = pu.cu->affine;
  subPu.cu->affine = false;

  // join sub-pus containing the same motion
  bool verMC = puSize.height > puSize.width;
  int  fstStart = (!verMC ? puPos.y : puPos.x);
  int  secStart = (!verMC ? puPos.x : puPos.y);
  int  fstEnd = (!verMC ? puPos.y + puSize.height : puPos.x + puSize.width);
  int  secEnd = (!verMC ? puPos.x + puSize.width : puPos.y + puSize.height);
  int  fstStep = (!verMC ? puHeight : puWidth);
  int  secStep = (!verMC ? puWidth : puHeight);

  pu.refIdx[0] = 0;
  pu.refIdx[1] = pu.cs->slice->sliceType == B_SLICE ? 0 : -1;
  bool scaled = false;//!PU::isRefPicSameSize(pu);

  m_subPuMC = true;

  for (int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep)
  {
    for (int secDim = secStart; secDim < secEnd; secDim += secStep)
    {
      int x = !verMC ? secDim : fstDim;
      int y = !verMC ? fstDim : secDim;
      const MotionInfo &curMi = pu.getMotionInfo(Position{ x, y });

      int length = secStep;
      int later = secDim + secStep;

      while (later < secEnd)
      {
        const MotionInfo &laterMi = !verMC ? pu.getMotionInfo(Position{ later, fstDim }) : pu.getMotionInfo(Position{ fstDim, later });
        if (!scaled && laterMi == curMi)
        {
          length += secStep;
        }
        else
        {
          break;
        }
        later += secStep;
      }
      int dx = !verMC ? length : puWidth;
      int dy = !verMC ? puHeight : length;

      subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
      subPu = curMi;
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(pu, subPu));
      subPu.mcControl = (pu.mcControl >> 1) << 1;
      subPu.mvRefine = false;
      motionCompensation(subPu, subPredBuf, eRefPicList);
      secDim = later - secStep;
    }
  }
  m_subPuMC = false;

  pu.cu->affine = isAffine;
}

InterPredInterpolation::InterPredInterpolation()
  : m_storedMv(nullptr)
  , m_skipPROF(false)
  , m_encOnly(false)
  , m_isBi(false)
{

}

InterPredInterpolation::~InterPredInterpolation()
{
  destroy();
}

void InterPredInterpolation::destroy()
{
  for( uint32_t c = 0; c < MAX_NUM_COMP; c++ )
  {
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++ )
    {
      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++ )
      {
        xFree( m_filteredBlock[i][j][c] );
        m_filteredBlock[i][j][c] = nullptr;
      }

      xFree( m_filteredBlockTmp[i][c] );
      m_filteredBlockTmp[i][c] = nullptr;
    }
  }
  xFree(m_gradX0);   m_gradX0 = nullptr;
  xFree(m_gradY0);   m_gradY0 = nullptr;
  xFree(m_gradX1);   m_gradX1 = nullptr;
  xFree(m_gradY1);   m_gradY1 = nullptr;

  if (m_storedMv != nullptr)
  {
    delete[] m_storedMv;
    m_storedMv = nullptr;
  }
}

void InterPredInterpolation::init()
{
  for( uint32_t c = 0; c < MAX_NUM_COMP; c++ )
  {
    int extWidth = MAX_CU_SIZE + (2 * BDOF_EXTEND_SIZE + 2) + 16;
    int extHeight = MAX_CU_SIZE + (2 * BDOF_EXTEND_SIZE + 2) + 1;
    extWidth = extWidth > (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 16) ? extWidth : MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 16;
    extHeight = extHeight > (MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 1) ? extHeight : MAX_CU_SIZE + (2 * DMVR_NUM_ITERATION) + 1;
    for( uint32_t i = 0; i < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; i++ )
    {
      m_filteredBlockTmp[i][c] = ( Pel* ) xMalloc( Pel, ( extWidth + 4 ) * ( extHeight + 7 + 4 ) );

      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++ )
      {
        m_filteredBlock[i][j][c] = ( Pel* ) xMalloc( Pel, extWidth * extHeight );
      }
    }
  }

  m_gradX0 = (Pel*)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
  m_gradY0 = (Pel*)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
  m_gradX1 = (Pel*)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
  m_gradY1 = (Pel*)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);

  m_if.initInterpolationFilter( true );

  xFpAddBDOFAvg4    = addBDOFAvgCore;
  xFpBDOFGradFilter = gradFilterCore;
  xFpCalcBDOFSums   = calcBDOFSumsCore;
#if ENABLE_SIMD_OPT_BDOF
  initInterPredictionX86();
#endif

  if (m_storedMv == nullptr)
  {
    const int MVBUFFER_SIZE = MAX_CU_SIZE / MIN_PU_SIZE;
    m_storedMv = new Mv[MVBUFFER_SIZE*MVBUFFER_SIZE];
  }
}

void InterPredInterpolation::xPredInterBlk ( const ComponentID compID, const PredictionUnit& pu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool bi, const ClpRng& clpRng
                                           , const bool bdofApplied
                                           , const bool isIBC
                                           , const RefPicList refPicList
                                           , const SizeType dmvrWidth
                                           , const SizeType dmvrHeight
                                           , const bool bilinearMC
                                           , const Pel* srcPadBuf
                                           , const int32_t srcPadStride
                                          )
{
  const ChromaFormat  chFmt = pu.chromaFormat;
  const bool          rndRes = !bi;

  int shiftHor = MV_FRACTIONAL_BITS_INTERNAL + getComponentScaleX(compID, chFmt);
  int shiftVer = MV_FRACTIONAL_BITS_INTERNAL + getComponentScaleY(compID, chFmt);

  bool  wrapRef = false;
  Mv    mv(_mv);
  if( !isIBC && pu.cs->pcv->wrapArround )
  {
    wrapRef = wrapClipMv( mv, pu.blocks[0].pos(), pu.blocks[0].size(), *pu.cs);
  }

  int xFrac = mv.hor & ((1 << shiftHor) - 1);
  int yFrac = mv.ver & ((1 << shiftVer) - 1);

  PelBuf& dstBuf  = dstPic.bufs[compID];
  unsigned width  = dstBuf.width;
  unsigned height = dstBuf.height;

  CPelBuf refBuf;
  if( srcPadBuf )
  {
    refBuf.buf = srcPadBuf;
    refBuf.stride = srcPadStride;
  }
  else
  {
    Position offset = pu.blocks[compID].pos().offset( mv.hor >> shiftHor, mv.ver >> shiftVer );
    refBuf      = (wrapRef) ? refPic->getRecoWrapBuf( compID ) : refPic->getRecoBuf( compID );
    refBuf.buf += offset.x;
    refBuf.buf += offset.y * refBuf.stride;
  }

  if( dmvrWidth )
  {
    width = dmvrWidth;
    height = dmvrHeight;
  }
  // backup data
  const int backupWidth = width;
  const int backupHeight = height;
  Pel* backupDstBufPtr = dstBuf.buf;
  int backupDstBufStride = dstBuf.stride;
  if( bdofApplied && compID == COMP_Y )
  {
    width = width + 2 * BDOF_EXTEND_SIZE + 2;
    height = height + 2 * BDOF_EXTEND_SIZE + 2;

    // change MC output
    CHECK( refPicList >= NUM_REF_PIC_LIST_01, "Wrong refpiclist" );
    dstBuf.stride = width;
    dstBuf.buf = m_filteredBlockTmp[2 + refPicList][compID] + 2 * dstBuf.stride + 2;
  }
  bool useAltHpelIf = pu.cu->imv == IMV_HPEL;

  if( yFrac == 0 )
  {
    m_if.filterHor(compID, (Pel*)refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight, xFrac, rndRes, chFmt, clpRng, useAltHpelIf, bilinearMC, bilinearMC);
  }
  else if( xFrac == 0 )
  {
    m_if.filterVer(compID, (Pel*)refBuf.buf, refBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight, yFrac, true, rndRes, chFmt, clpRng, useAltHpelIf, bilinearMC, bilinearMC);
  }
  else
  {
    int vFilterSize = bilinearMC ? NTAPS_BILINEAR : isLuma( compID ) ? NTAPS_LUMA : NTAPS_CHROMA;

    if( backupWidth == 4 && backupHeight == 4 && !bilinearMC )
    {
      m_if.filter4x4( compID, (Pel*)refBuf.buf, refBuf.stride ,(Pel*)dstBuf.buf, dstBuf.stride, 4, 4, xFrac, yFrac, rndRes, chFmt, clpRng, useAltHpelIf );
    }
    else if( !bilinearMC && ( backupWidth & 15 ) == 0 )
    {
      for( int dx = 0; dx < backupWidth; dx += 16 )
      {
        m_if.filter16x16( compID, refBuf.buf + dx, refBuf.stride, dstBuf.buf + dx, dstBuf.stride, 16, backupHeight, xFrac, yFrac, rndRes, chFmt, clpRng, useAltHpelIf );
      }
    }
    else if( !bilinearMC && ( backupWidth & 7 ) == 0 )
    {
      for( int dx = 0; dx < backupWidth; dx += 8 )
      {
        m_if.filter8x8( compID, refBuf.buf + dx, refBuf.stride, dstBuf.buf + dx, dstBuf.stride, 8, backupHeight, xFrac, yFrac, rndRes, chFmt, clpRng, useAltHpelIf );
      }
    }
    else
    {
      PelBuf tmpBuf( m_filteredBlockTmp[0][compID], dmvrWidth ? dmvrWidth : dstBuf.stride, dmvrWidth ? Size( dmvrWidth, dmvrHeight ) : pu.blocks[compID].size() );

      m_if.filterHor(compID, (Pel*)refBuf.buf - ((vFilterSize >> 1) - 1) * refBuf.stride, refBuf.stride, tmpBuf.buf, tmpBuf.stride, backupWidth, backupHeight + vFilterSize - 1, xFrac, false, chFmt, clpRng, useAltHpelIf, bilinearMC, bilinearMC);
      m_if.filterVer(compID, (Pel*)tmpBuf.buf + ((vFilterSize >> 1) - 1) * tmpBuf.stride, tmpBuf.stride, dstBuf.buf, dstBuf.stride, backupWidth, backupHeight, yFrac, false, rndRes, chFmt, clpRng, useAltHpelIf, bilinearMC, bilinearMC);
    }
  }
  if (bdofApplied && compID == COMP_Y)
  {
    const unsigned shift = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
    int xOffset = (xFrac < 8) ? 1 : 0;
    int yOffset = (yFrac < 8) ? 1 : 0;
    const Pel* refPel = refBuf.buf - yOffset * refBuf.stride - xOffset;
    Pel* dstPel = m_filteredBlockTmp[2 + refPicList][compID] + dstBuf.stride + 1;
    for (int w = 0; w < (width - 2 * BDOF_EXTEND_SIZE); w++)
    {
      Pel val = leftShiftU(refPel[w], shift);
      dstPel[w] = val - (Pel)IF_INTERNAL_OFFS;
    }

    refPel = refBuf.buf + (1 - yOffset)*refBuf.stride - xOffset;
    dstPel = m_filteredBlockTmp[2 + refPicList][compID] + 2 * dstBuf.stride + 1;
    for (int h = 0; h < (height - 2 * BDOF_EXTEND_SIZE - 2); h++)
    {
      Pel val = leftShiftU(refPel[0], shift);
      dstPel[0] = val - (Pel)IF_INTERNAL_OFFS;

      val = leftShiftU(refPel[width - 3], shift);
      dstPel[width - 3] = val - (Pel)IF_INTERNAL_OFFS;

      refPel += refBuf.stride;
      dstPel += dstBuf.stride;
    }

    refPel = refBuf.buf + (height - 2 * BDOF_EXTEND_SIZE - 2 + 1 - yOffset)*refBuf.stride - xOffset;
    dstPel = m_filteredBlockTmp[2 + refPicList][compID] + (height - 2 * BDOF_EXTEND_SIZE)*dstBuf.stride + 1;
    for (int w = 0; w < (width - 2 * BDOF_EXTEND_SIZE); w++)
    {
      Pel val = leftShiftU(refPel[w], shift);
      dstPel[w] = val - (Pel)IF_INTERNAL_OFFS;
    }

    // restore data
    width         = backupWidth;
    height        = backupHeight;
    dstBuf.buf    = backupDstBufPtr;
    dstBuf.stride = backupDstBufStride;
  }
}

int InterPredInterpolation::xRightShiftMSB( int numer, int denom )
{
  return ( numer >> floorLog2( denom ) );
}

void InterPredInterpolation::xApplyBDOF( PelBuf& yuvDst, const ClpRng& clpRng )
{
  const int     bitDepth  = clpRng.bd;

  const int     height    = yuvDst.height;
  const int     width     = yuvDst.width;
  int           heightG   = height + 2 * BDOF_EXTEND_SIZE;
  int           widthG    = width + 2 * BDOF_EXTEND_SIZE;
  int           offsetPos = widthG*BDOF_EXTEND_SIZE + BDOF_EXTEND_SIZE;

  Pel*          gradX0 = m_gradX0;
  Pel*          gradX1 = m_gradX1;
  Pel*          gradY0 = m_gradY0;
  Pel*          gradY1 = m_gradY1;

  int           stridePredMC = widthG + 2;
  const Pel*    srcY0 = m_filteredBlockTmp[2][COMP_Y] + stridePredMC + 1;
  const Pel*    srcY1 = m_filteredBlockTmp[3][COMP_Y] + stridePredMC + 1;
  const int     src0Stride = stridePredMC;
  const int     src1Stride = stridePredMC;

  Pel*          dstY = yuvDst.buf;
  const int     dstStride = yuvDst.stride;
  const Pel*    srcY0Temp = srcY0;
  const Pel*    srcY1Temp = srcY1;

  for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    Pel* dstTempPtr = m_filteredBlockTmp[2 + refList][COMP_Y] + stridePredMC + 1;
    Pel* gradY = (refList == 0) ? m_gradY0 : m_gradY1;
    Pel* gradX = (refList == 0) ? m_gradX0 : m_gradX1;

    xFpBDOFGradFilter(dstTempPtr, stridePredMC, widthG, heightG, widthG, gradX, gradY, bitDepth );
    Pel* padStr = m_filteredBlockTmp[2 + refList][COMP_Y] + 2 * stridePredMC + 2;
    for (int y = 0; y< height; y++)
    {
      padStr[-1] = padStr[0];
      padStr[width] = padStr[width - 1];
      padStr += stridePredMC;
    }

    padStr = m_filteredBlockTmp[2 + refList][COMP_Y] + 2 * stridePredMC + 1;
    ::memcpy(padStr - stridePredMC, padStr, sizeof(Pel)*(widthG));
    ::memcpy(padStr + height*stridePredMC, padStr + (height - 1)*stridePredMC, sizeof(Pel)*(widthG));
  }

  const unsigned shiftNum = IF_INTERNAL_PREC + 1 - bitDepth;
  const int   offset = (1 << (shiftNum - 1)) + 2 * IF_INTERNAL_OFFS;
  const int   limit = (1 << 4) - 1;

  int xUnit = (width >> 2);
  int yUnit = (height >> 2);

  Pel* dstY0 = dstY;
  gradX0 = m_gradX0; gradX1 = m_gradX1;
  gradY0 = m_gradY0; gradY1 = m_gradY1;

  for (int yu = 0; yu < yUnit; yu++)
  {
    for (int xu = 0; xu < xUnit; xu++)
    {
      int tmpx = 0, tmpy = 0;
      int sumAbsGX = 0, sumAbsGY = 0, sumDIX = 0, sumDIY = 0;
      int sumSignGY_GX = 0;

      Pel* pGradX0Tmp = m_gradX0 + (xu << 2) + (yu << 2) * widthG;
      Pel* pGradX1Tmp = m_gradX1 + (xu << 2) + (yu << 2) * widthG;
      Pel* pGradY0Tmp = m_gradY0 + (xu << 2) + (yu << 2) * widthG;
      Pel* pGradY1Tmp = m_gradY1 + (xu << 2) + (yu << 2) * widthG;
      const Pel* SrcY1Tmp = srcY1 + (xu << 2) + (yu << 2) * src1Stride;
      const Pel* SrcY0Tmp = srcY0 + (xu << 2) + (yu << 2) * src0Stride;

      xFpCalcBDOFSums(SrcY0Tmp, SrcY1Tmp, pGradX0Tmp, pGradX1Tmp, pGradY0Tmp, pGradY1Tmp, xu, yu, src0Stride, src1Stride, widthG, bitDepth, &sumAbsGX, &sumAbsGY, &sumDIX, &sumDIY, &sumSignGY_GX);
      tmpx = (sumAbsGX == 0 ? 0 : xRightShiftMSB(sumDIX << 2, sumAbsGX));
      tmpx = Clip3(-limit, limit, tmpx);

      int     mainsGxGy = sumSignGY_GX >> 12;
      int     secsGxGy  = sumSignGY_GX & ((1 << 12) - 1);
      int     tmpData   = tmpx * mainsGxGy;
      tmpData           = ((tmpData << 12) + tmpx*secsGxGy) >> 1;
      tmpy = (sumAbsGY == 0 ? 0 : xRightShiftMSB(((sumDIY << 2) - tmpData), sumAbsGY));
      tmpy = Clip3(-limit, limit, tmpy);

      srcY0Temp = srcY0 + (stridePredMC + 1) + ((yu*src0Stride + xu) << 2);
      srcY1Temp = srcY1 + (stridePredMC + 1) + ((yu*src0Stride + xu) << 2);
      gradX0 = m_gradX0 + offsetPos + ((yu*widthG + xu) << 2);
      gradX1 = m_gradX1 + offsetPos + ((yu*widthG + xu) << 2);
      gradY0 = m_gradY0 + offsetPos + ((yu*widthG + xu) << 2);
      gradY1 = m_gradY1 + offsetPos + ((yu*widthG + xu) << 2);

      dstY0 = dstY + ((yu*dstStride + xu) << 2);
      xFpAddBDOFAvg4(srcY0Temp, src0Stride, srcY1Temp, src1Stride, dstY0, dstStride, gradX0, gradX1, gradY0, gradY1, widthG, (1 << 2), (1 << 2), tmpx, tmpy, shiftNum, offset, clpRng);
    }  // xu
  }  // yu
}

void InterPredInterpolation::xWeightedAverage( const PredictionUnit& pu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const bool bdofApplied )
{
  const bool lumaOnly = (pu.mcControl >> 1) == 1;
  const bool chromaOnly = pu.mcControl > 3;
  CHECK((chromaOnly && lumaOnly), "should not happen");

  const ClpRngs& clpRngs = pu.cu->slice->clpRngs;
  const int iRefIdx0 = pu.refIdx[0];
  const int iRefIdx1 = pu.refIdx[1];

  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
    if (bdofApplied)
    {
      xApplyBDOF( pcYuvDst.Y(), clpRngs[COMP_Y] );
    }
    if (!bdofApplied && (lumaOnly || chromaOnly))
    {
      pcYuvDst.addAvg(pcYuvSrc0, pcYuvSrc1, clpRngs, chromaOnly, lumaOnly);
    }
    else
    {
      pcYuvDst.addAvg( pcYuvSrc0, pcYuvSrc1, clpRngs, bdofApplied );
    }
  }
  else if( iRefIdx0 >= 0 && iRefIdx1 < 0 )
  {
    if (pu.cu->geo)
    {
      pcYuvDst.copyFrom(pcYuvSrc0);
    }
    else
    {
      pcYuvDst.copyClip(pcYuvSrc0, clpRngs, lumaOnly, chromaOnly);
    }
  }
  else if( iRefIdx0 < 0 && iRefIdx1 >= 0 )
  {
    if (pu.cu->geo)
    {
      pcYuvDst.copyFrom(pcYuvSrc1);
    }
    else
    {
      pcYuvDst.copyClip(pcYuvSrc1, clpRngs, lumaOnly, chromaOnly);
    }
  }
}

void InterPrediction::motionCompensationGeo(PredictionUnit &pu, PelUnitBuf& predBuf, const MergeCtx &geoMrgCtx)
{
  const ClpRngs & clpRngs  = pu.cu->slice->clpRngs;
  const UnitArea localUnitArea(pu.chromaFormat, Area(0, 0, pu.lwidth(), pu.lheight()));

  PelUnitBuf     tmpGeoBuf0 = m_geoPartBuf[0].getBuf(localUnitArea);
  PelUnitBuf     tmpGeoBuf1 = m_geoPartBuf[1].getBuf(localUnitArea);

  geoMrgCtx.setMergeInfo(pu, pu.geoMergeIdx0);
  PU::spanMotionInfo(pu);
  motionCompensation(pu, tmpGeoBuf0, REF_PIC_LIST_X);   // TODO: check 4:0:0 interaction with weighted prediction.

  geoMrgCtx.setMergeInfo(pu, pu.geoMergeIdx1);
  PU::spanMotionInfo(pu);
  motionCompensation(pu, tmpGeoBuf1, REF_PIC_LIST_X);   // TODO: check 4:0:0 interaction with weighted prediction.

  weightedGeoBlk(clpRngs, pu, pu.geoSplitDir, isChromaEnabled(pu.chromaFormat) ? MAX_NUM_CH : CH_L, predBuf, tmpGeoBuf0,
                  tmpGeoBuf1);
}

void InterPredInterpolation::weightedGeoBlk(const ClpRngs &clpRngs, PredictionUnit &pu, const uint8_t splitDir,
                                            int32_t channel, PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1)
{
  if (channel == CH_L)
  {
    m_if.weightedGeoBlk(clpRngs,pu, pu.lumaSize().width, pu.lumaSize().height, COMP_Y, splitDir, predDst, predSrc0,
                        predSrc1);
  }
  else if (channel == CH_C)
  {
    m_if.weightedGeoBlk(clpRngs, pu, pu.chromaSize().width, pu.chromaSize().height, COMP_Cb, splitDir, predDst, predSrc0,
                        predSrc1);
    m_if.weightedGeoBlk(clpRngs, pu, pu.chromaSize().width, pu.chromaSize().height, COMP_Cr, splitDir, predDst, predSrc0,
                        predSrc1);
  }
  else
  {
    m_if.weightedGeoBlk(clpRngs, pu, pu.lumaSize().width, pu.lumaSize().height, COMP_Y, splitDir, predDst, predSrc0,
                        predSrc1);
    if (isChromaEnabled(pu.chromaFormat))
    {
      m_if.weightedGeoBlk(clpRngs, pu, pu.chromaSize().width, pu.chromaSize().height, COMP_Cb, splitDir, predDst,
                          predSrc0, predSrc1);
      m_if.weightedGeoBlk(clpRngs, pu, pu.chromaSize().width, pu.chromaSize().height, COMP_Cr, splitDir, predDst,
                          predSrc0, predSrc1);
    }
  }
}

DMVR::DMVR() : m_pcRdCost( nullptr )
{
}

DMVR::~DMVR()
{
  destroy();
}

void DMVR::destroy()
{
  for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    m_yuvPred[i].destroy();
    m_yuvPad[i].destroy();
    m_yuvTmp[i].destroy();
    m_yuvRef[i].destroy();
  }
  m_pcRdCost = nullptr;
}

void DMVR::init( RdCost* pcRdCost, const ChromaFormat chFormat )
{
  if( m_pcRdCost == nullptr )
  {
    m_pcRdCost = pcRdCost;

    Area predArea = Area( 0, 0, DMVR_SUBCU_SIZE, DMVR_SUBCU_SIZE );
    Area refArea  = Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE );
    for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
    {
      m_yuvPred[i].create( chFormat, predArea );
      m_yuvTmp[i].create( CHROMA_400, refArea, 0, DMVR_NUM_ITERATION );
      m_yuvPad[i].create( chFormat, predArea, 0, DMVR_NUM_ITERATION + (NTAPS_LUMA>>1), 32 );
      m_yuvRef[i].create( chFormat, refArea, 0, DMVR_NUM_ITERATION + (NTAPS_LUMA>>1), 32 );
    }
  }
}

void DMVR::xPrefetch( const PredictionUnit& pu, PelUnitBuf& dstBuf, RefPicList refId, bool isPadding, bool forLuma, bool forChroma )
{
  const Picture* refPic = pu.cu->slice->getRefPic(refId, pu.refIdx[refId]);
  int mvShift = (MV_FRACTIONAL_BITS_INTERNAL);
  int start = forLuma ? 0 : 1;
  int end = forChroma ? MAX_NUM_COMP : 1;

  for (int compID = start; compID < end; compID++)
  {
    int mvshiftTemp      = mvShift + getComponentScaleX((ComponentID)compID, pu.chromaFormat);
    int filtersize       = (compID == (COMP_Y)) ? NTAPS_LUMA : NTAPS_CHROMA;
    int width            = dstBuf.bufs[compID].width + (filtersize - 1);
    int height           = dstBuf.bufs[compID].height + (filtersize - 1);
    int leftTopFilterExt = ( filtersize >> 1 ) - 1;
    Mv cMv               = Mv( pu.mv[refId].hor, pu.mv[refId].ver );
    cMv                 += Mv(-(leftTopFilterExt << mvshiftTemp), -(leftTopFilterExt << mvshiftTemp));
    bool wrapRef         = false;
    if( pu.cs->pcv->wrapArround )
    {
      wrapRef = wrapClipMv( cMv, pu.blocks[COMP_Y].pos(), pu.blocks[COMP_Y].size(), *pu.cs);
    }
    else
    {
      clipMv( cMv, pu.lumaPos(), pu.lumaSize(),*pu.cs->pcv );
    }
    /* Pre-fetch similar to HEVC*/
    {
      Position recOffset  = pu.blocks[compID].pos().offset( cMv.hor >> mvshiftTemp, cMv.ver >> mvshiftTemp );
      CompArea ca((ComponentID)compID, pu.chromaFormat, recOffset, pu.blocks[compID].size());
      CPelBuf refBuf      = wrapRef ? refPic->getRecoWrapBuf(ca) : refPic->getRecoBuf(ca);
      PelBuf& padBuf      = dstBuf.bufs[compID];
      int padOffset       = leftTopFilterExt * padBuf.stride + leftTopFilterExt;
      g_pelBufOP.copyBuffer( ( const char * ) refBuf.buf, refBuf.stride * sizeof( Pel ), ( char* ) ( padBuf.buf - padOffset ), padBuf.stride * sizeof( Pel ), width * sizeof( Pel ), height );
      if( isPadding )
      {
        const int padSize = (DMVR_NUM_ITERATION) >> getComponentScaleX((ComponentID)compID, pu.chromaFormat);
        g_pelBufOP.padding( padBuf.buf - padOffset, padBuf.stride, width, height, padSize );
      }
    }
  }
}

void DMVR::xCopyAndPad( const PredictionUnit& pu, const PelUnitBuf& srcBuf, const PelUnitBuf& dstBuf )
{
  for( int compID = 0; compID < MAX_NUM_COMP; compID++ )
  {
    const int filtersize = ( compID == ( COMP_Y ) ) ? NTAPS_LUMA : NTAPS_CHROMA;

    const int width   = dstBuf.bufs[compID].width  + filtersize - 1;
    const int height  = dstBuf.bufs[compID].height + filtersize - 1;

    const PelBuf& refBuf = srcBuf.bufs[compID];
    const PelBuf& padBuf = dstBuf.bufs[compID];
    const int leftTopFilterExt = ( ( filtersize >> 1 ) - 1 );
    const int refOffset = leftTopFilterExt * refBuf.stride + leftTopFilterExt;
    const int padOffset = leftTopFilterExt * padBuf.stride + leftTopFilterExt;
    const int padSize   = ( DMVR_NUM_ITERATION ) >> getComponentScaleX( ( ComponentID ) compID, pu.chromaFormat );
    g_pelBufOP.copyBuffer( ( const char* ) ( refBuf.buf - refOffset ), refBuf.stride * sizeof( Pel ), ( char* ) ( padBuf.buf - padOffset ), padBuf.stride * sizeof( Pel ), width * sizeof( Pel ), height );
    g_pelBufOP.padding( padBuf.buf - padOffset, padBuf.stride, width, height, padSize );
  }
}

inline int32_t div_for_maxq7(int64_t N, int64_t D)
{
  int32_t sign, q;
  sign = 0;
  if (N < 0)
  {
    sign = 1;
    N = -N;
  }

  q = 0;
  D = (D << 3);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  D = (D >> 1);
  if (N >= D)
  {
    N -= D;
    q++;
  }
  q = (q << 1);

  if (N >= (D >> 1))
    q++;

  if (sign)
    return (-q);
  return(q);
}

void xSubPelErrorSrfc(uint64_t *sadBuffer, int32_t *deltaMv)
{
  for( int hv = 0; hv < 2; hv++)
  {
    const int32_t mvSubPelLvl = 4;/*1: half pel, 2: Qpel, 3:1/8, 4: 1/16*/
    int64_t numerator   = (int64_t)((sadBuffer[hv+1] - sadBuffer[hv+3]) << mvSubPelLvl);
    int64_t denominator = (int64_t)((sadBuffer[hv+1] + sadBuffer[hv+3] - (sadBuffer[0] << 1)));

    if (0 != denominator)
    {
      if ((sadBuffer[hv+1] != sadBuffer[0]) && (sadBuffer[hv+3] != sadBuffer[0]))
      {
        deltaMv[hv] = div_for_maxq7(numerator, denominator);
      }
      else
      {
        deltaMv[hv] = (sadBuffer[hv+1] == sadBuffer[0]) ? -8 : 8;
      }
    }
  }
}

void DMVR::xFinalPaddedMCForDMVR( const PredictionUnit& pu, PelUnitBuf* dstBuf, const PelUnitBuf *refBuf, const bool bioApplied, const Mv mergeMv[NUM_REF_PIC_LIST_01], const Mv& refMv )
{
  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;
  Mv mv[2];
  mv[L0] = mergeMv[L0] + refMv;
  mv[L1] = mergeMv[L1] - refMv;

  for (int k = 0; k < NUM_REF_PIC_LIST_01; k++)
  {
    RefPicList refId = (RefPicList)k;
    const Mv& cMv = mv[refId];
    Mv cMvClipped( cMv );
    clipMv(cMvClipped, pu.lumaPos(), pu.lumaSize(), *pu.cs->pcv);

    const Mv& startMv = mergeMv[refId];
    for (int compID = 0; compID < MAX_NUM_COMP; compID++)
    {
      int mvshiftTemp = mvShift + getComponentScaleX((ComponentID)compID, pu.chromaFormat);
      int deltaIntMvX = (cMv.hor >> mvshiftTemp) - (startMv.hor >> mvshiftTemp);
      int deltaIntMvY = (cMv.ver >> mvshiftTemp) - (startMv.ver >> mvshiftTemp);
      CHECK((abs(deltaIntMvX) > DMVR_NUM_ITERATION) || (abs(deltaIntMvY) > DMVR_NUM_ITERATION), "not expected DMVR movement");

      const PelBuf& srcBuf = refBuf[refId].bufs[compID];
      int offset = (deltaIntMvY) * srcBuf.stride + (deltaIntMvX);

      xPredInterBlk( (ComponentID)compID, pu, nullptr, cMvClipped, dstBuf[refId], true, pu.cs->slice->clpRngs.comp[compID],
        bioApplied, false, refId, 0, 0, 0, srcBuf.buf + offset, srcBuf.stride );
    }
  }
}

void xDMVRSubPixelErrorSurface( bool notZeroCost, int16_t *totalDeltaMV, int16_t *deltaMV, uint64_t *pSADsArray )
{
  int sadStride = (((2 * DMVR_NUM_ITERATION) + 1));
  uint64_t sadbuffer[5];
  if (notZeroCost
    && (abs(totalDeltaMV[0]) != (2 << MV_FRACTIONAL_BITS_INTERNAL))
    && (abs(totalDeltaMV[1]) != (2 << MV_FRACTIONAL_BITS_INTERNAL)))
  {
    int32_t tempDeltaMv[2] = { 0,0 };
    sadbuffer[0] = pSADsArray[0];
    sadbuffer[1] = pSADsArray[-1];
    sadbuffer[2] = pSADsArray[-sadStride];
    sadbuffer[3] = pSADsArray[1];
    sadbuffer[4] = pSADsArray[sadStride];
    xSubPelErrorSrfc(sadbuffer, tempDeltaMv);
    totalDeltaMV[0] += tempDeltaMv[0];
    totalDeltaMV[1] += tempDeltaMv[1];
  }
}

void DMVR::xProcessDMVR( const PredictionUnit& pu, PelUnitBuf& pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied )
{
  PROFILER_SCOPE_AND_STAGE( 1, g_timeProfiler, P_INTER_MRG_DMVR );
  int iterationCount = 1;
  /*Always High Precision*/
  const int mvShift = MV_FRACTIONAL_BITS_INTERNAL;

  /*use merge MV as starting MV*/
  const Mv mergeMv[] = { pu.mv[REF_PIC_LIST_0] , pu.mv[REF_PIC_LIST_1] };


  const int dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_SIZE);
  const int dx = std::min<int>(pu.lumaSize().width,  DMVR_SUBCU_SIZE);
  const bool usingSubPU = pu.lwidth() > DMVR_SUBCU_SIZE || pu.lheight() > DMVR_SUBCU_SIZE;

  const Position& puPos = pu.lumaPos();

  bool bioAppliedType[MAX_NUM_SUBCU_DMVR];
  const int refBufStride   = pu.Y().width + 2 * ( DMVR_NUM_ITERATION + ( NTAPS_LUMA >> 1 ) );
  const int refBufStrideCr = pu.Cb().width + 2 * ( DMVR_NUM_ITERATION + ( NTAPS_CHROMA >> 1 ) );
  PelUnitBuf yuvRefPu[NUM_REF_PIC_LIST_01];
  for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
    yuvRefPu[i] = m_yuvRef[i].getBuf( refBufStride, refBufStrideCr, refBufStrideCr, pu );

  // Do refinement search
  {
    const int bilinearBufStride = (pu.Y().width + (2 * DMVR_NUM_ITERATION));
    const int padSize = DMVR_NUM_ITERATION << 1;
    const int srcOffset = -( DMVR_NUM_ITERATION * yuvRefPu[L0].bufs[COMP_Y].stride + DMVR_NUM_ITERATION );
    const int dstOffset = -( DMVR_NUM_ITERATION * bilinearBufStride + DMVR_NUM_ITERATION );

    for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
    {
      RefPicList refId = (RefPicList)i;
      xPrefetch( pu, yuvRefPu[i], RefPicList( i ), !usingSubPU );

      // generate bilinear interpolated reference for the search
      const PelBuf& srcBuf = yuvRefPu[refId].bufs[COMP_Y];
      PelUnitBuf yuvTmp = PelUnitBuf( pu.chromaFormat, PelBuf( m_yuvTmp[refId].getBuf( COMP_Y ).buf + dstOffset, bilinearBufStride, pu.lwidth() + padSize, pu.lheight() + padSize ) );
      xPredInterBlk( COMP_Y, pu, nullptr, mergeMv[refId], yuvTmp, true, clpRngs.comp[COMP_Y], false, false, refId, pu.lwidth() + padSize, pu.lheight() + padSize, true, srcBuf.buf + srcOffset, srcBuf.stride );
    }

    // point mc buffer to center point to avoid multiplication to reach each iteration to the beginning
    const Pel* biLinearPredL0 = m_yuvTmp[0].getBuf( COMP_Y ).buf;
    const Pel* biLinearPredL1 = m_yuvTmp[1].getBuf( COMP_Y ).buf;
    const int bioEnabledThres = 2 * dy * dx;
    const int bd = pu.cs->slice->clpRngs.comp[COMP_Y].bd;

    DistParam distParam = m_pcRdCost->setDistParam( nullptr, nullptr, bilinearBufStride, bilinearBufStride, bd, COMP_Y, dx, dy, 1 );

    int num = 0;
    int yStart = 0;
    uint64_t sadArray[((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)];

    for (int y = puPos.y; y < (puPos.y + pu.lumaSize().height); y = y + dy, yStart = yStart + dy)
    {
      for (int x = puPos.x, xStart = 0; x < (puPos.x + pu.lumaSize().width); x = x + dx, xStart = xStart + dx)
      {
        uint64_t minCost = MAX_UINT64;
        bool notZeroCost = true;
        int16_t totalDeltaMV[2] = { 0, 0 };
        int16_t deltaMV[2]      = { 0, 0 };

        // set all entries to MAX_UNIT64
        memset( sadArray, 0xff, sizeof( sadArray ) );
        uint64_t *pSADsArray = &sadArray[( ( ( 2 * DMVR_NUM_ITERATION ) + 1 ) * ( ( 2 * DMVR_NUM_ITERATION ) + 1 ) ) >> 1];

        const Pel* addrL0Centre = biLinearPredL0 + yStart * bilinearBufStride + xStart;
        const Pel* addrL1Centre = biLinearPredL1 + yStart * bilinearBufStride + xStart;

        for( int i = 0; i < iterationCount; i++ )
        {
          const ptrdiff_t diff0 = totalDeltaMV[0] + (totalDeltaMV[1] * bilinearBufStride);
          const Pel* addrL0 = addrL0Centre + diff0;
          const Pel* addrL1 = addrL1Centre - diff0;
          if (i == 0)
          {
            distParam.org.buf = addrL0;
            distParam.cur.buf = addrL1;
            minCost = distParam.distFunc(distParam)>>1;
            minCost -= (minCost >>2);
            if (minCost < (dx * dy))
            {
              notZeroCost = false;
              break;
            }
            pSADsArray[0] = minCost;
          }
          if (!minCost)
          {
            notZeroCost = false;
            break;
          }

          //xBIPMVRefine
          {
            deltaMV[0] = 0;
            deltaMV[1] = 0;

            for (int nIdx = 0; (nIdx < 25); ++nIdx)
            {
              int32_t sadOffset = ((m_pSearchOffset[nIdx].ver * ((2 * DMVR_NUM_ITERATION) + 1)) + m_pSearchOffset[nIdx].hor);
              if (*(pSADsArray + sadOffset) == MAX_UINT64)
              {
                const ptrdiff_t diff = m_pSearchOffset[nIdx].hor + (m_pSearchOffset[nIdx].ver * bilinearBufStride);
                distParam.org.buf = addrL0 + diff;
                distParam.cur.buf = addrL1 - diff;
                const uint64_t cost = distParam.distFunc(distParam)>>1;
                *(pSADsArray + sadOffset) = cost;
              }
              if (*(pSADsArray + sadOffset) < minCost)
              {
                minCost = *(pSADsArray + sadOffset);
                deltaMV[0] = m_pSearchOffset[nIdx].hor;
                deltaMV[1] = m_pSearchOffset[nIdx].ver;
              }
            }
          }

          if (deltaMV[0] == 0 && deltaMV[1] == 0)
          {
            break;
          }
          totalDeltaMV[0] += deltaMV[0];
          totalDeltaMV[1] += deltaMV[1];
          pSADsArray += ((deltaMV[1] * (((2 * DMVR_NUM_ITERATION) + 1))) + deltaMV[0]);
        }

        bioAppliedType[num] = (minCost < bioEnabledThres) ? false : bioApplied;
        totalDeltaMV[0] = (totalDeltaMV[0] << mvShift);
        totalDeltaMV[1] = (totalDeltaMV[1] << mvShift);
        xDMVRSubPixelErrorSurface(notZeroCost, totalDeltaMV, deltaMV, pSADsArray);

        pu.mvdL0SubPu[num] = Mv(totalDeltaMV[0], totalDeltaMV[1]);

        num++;
      }
    }
  }

  // Final MC
  if( usingSubPU )
  {
    PredictionUnit subPu = pu;
    subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(puPos.x, puPos.y, dx, dy)));
    PelUnitBuf subPredBuf = pcYuvDst.subBuf(UnitAreaRelative(pu, subPu));

    PelUnitBuf predBuf[NUM_REF_PIC_LIST_01];
    PelUnitBuf padBuf[NUM_REF_PIC_LIST_01];
    predBuf[L0] = m_yuvPred[L0].getCompactBuf( subPu );
    predBuf[L1] = m_yuvPred[L1].getCompactBuf( subPu );

    /* For padding */
    padBuf[L0] = m_yuvPad[L0].getBufPart( subPu );
    padBuf[L1] = m_yuvPad[L1].getBufPart( subPu );

    int x = 0, y = 0;
    int xStart = 0, yStart = 0;
    int num = 0;
    const int scaleX = getComponentScaleX(COMP_Cb, pu.chromaFormat);
    const int scaleY = getComponentScaleY(COMP_Cb, pu.chromaFormat);

    const ptrdiff_t dstStride[MAX_NUM_COMP] = { pcYuvDst.bufs[COMP_Y].stride, pcYuvDst.bufs[COMP_Cb].stride, pcYuvDst.bufs[COMP_Cr].stride };
    for (y = puPos.y; y < (puPos.y + pu.lumaSize().height); y = y + dy, yStart = yStart + dy)
    {
      for (x = puPos.x, xStart = 0; x < (puPos.x + pu.lumaSize().width); x = x + dx, xStart = xStart + dx)
      {
        subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
        PelUnitBuf yuvRefSubPu[NUM_REF_PIC_LIST_01];
        yuvRefSubPu[L0] = yuvRefPu[L0].subBuf(UnitAreaRelative(pu, subPu));
        yuvRefSubPu[L1] = yuvRefPu[L1].subBuf(UnitAreaRelative(pu, subPu));

        if( pu.mvdL0SubPu[num] != Mv(0, 0) )
        {
          xCopyAndPad( subPu, yuvRefSubPu[L0], padBuf[L0] );
          xCopyAndPad( subPu, yuvRefSubPu[L1], padBuf[L1] );
          xFinalPaddedMCForDMVR( subPu, predBuf, padBuf, bioAppliedType[num], mergeMv, pu.mvdL0SubPu[num] );
        }
        else
        {
          xFinalPaddedMCForDMVR( subPu, predBuf, yuvRefSubPu, bioAppliedType[num], mergeMv, pu.mvdL0SubPu[num] );
        }

        subPredBuf.bufs[COMP_Y].buf  = pcYuvDst.bufs[COMP_Y].buf + xStart + yStart * dstStride[COMP_Y];
        subPredBuf.bufs[COMP_Cb].buf = pcYuvDst.bufs[COMP_Cb].buf + (xStart >> scaleX) + ((yStart >> scaleY) * dstStride[COMP_Cb]);
        subPredBuf.bufs[COMP_Cr].buf = pcYuvDst.bufs[COMP_Cr].buf + (xStart >> scaleX) + ((yStart >> scaleY) * dstStride[COMP_Cr]);

        xWeightedAverage(subPu, predBuf[L0], predBuf[L1], subPredBuf, bioAppliedType[num] );
        num++;
      }
    }
  }
  else
  {
    PelUnitBuf predBuf[NUM_REF_PIC_LIST_01];
    predBuf[L0] = m_yuvPred[L0].getCompactBuf( pu );
    predBuf[L1] = m_yuvPred[L1].getCompactBuf( pu );
    xFinalPaddedMCForDMVR( pu, predBuf, yuvRefPu, bioAppliedType[0], mergeMv, pu.mvdL0SubPu[0] );
    xWeightedAverage( pu, predBuf[L0], predBuf[L1], pcYuvDst, bioAppliedType[0] );
  }
}

bool InterPredInterpolation::isSubblockVectorSpreadOverLimit(int a, int b, int c, int d, int predType)
{
  int s4 = (4 << 11);
  int filterTap = 6;

  if (predType == 3)
  {
    int refBlkWidth = std::max(std::max(0, 4 * a + s4), std::max(4 * c, 4 * a + 4 * c + s4)) - std::min(std::min(0, 4 * a + s4), std::min(4 * c, 4 * a + 4 * c + s4));
    int refBlkHeight = std::max(std::max(0, 4 * b), std::max(4 * d + s4, 4 * b + 4 * d + s4)) - std::min(std::min(0, 4 * b), std::min(4 * d + s4, 4 * b + 4 * d + s4));
    refBlkWidth = (refBlkWidth >> 11) + filterTap + 3;
    refBlkHeight = (refBlkHeight >> 11) + filterTap + 3;

    if (refBlkWidth * refBlkHeight > (filterTap + 9) * (filterTap + 9))
    {
      return true;
    }
  }
  else
  {
    int refBlkWidth = std::max(0, 4 * a + s4) - std::min(0, 4 * a + s4);
    int refBlkHeight = std::max(0, 4 * b) - std::min(0, 4 * b);
    refBlkWidth = (refBlkWidth >> 11) + filterTap + 3;
    refBlkHeight = (refBlkHeight >> 11) + filterTap + 3;
    if (refBlkWidth * refBlkHeight > (filterTap + 9) * (filterTap + 5))
    {
      return true;
    }

    refBlkWidth = std::max(0, 4 * c) - std::min(0, 4 * c);
    refBlkHeight = std::max(0, 4 * d + s4) - std::min(0, 4 * d + s4);
    refBlkWidth = (refBlkWidth >> 11) + filterTap + 3;
    refBlkHeight = (refBlkHeight >> 11) + filterTap + 3;
    if (refBlkWidth * refBlkHeight > (filterTap + 5) * (filterTap + 9))
    {
      return true;
    }
  }
  return false;
}

void InterPredInterpolation::xPredAffineBlk(const ComponentID compID, const PredictionUnit& pu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool bi, const ClpRng& clpRng, const RefPicList refPicList)
{
  const ChromaFormat chFmt = pu.chromaFormat;
  int iScaleX = getComponentScaleX(compID, chFmt);
  int iScaleY = getComponentScaleY(compID, chFmt);

  Mv mvLT = _mv[0];
  Mv mvRT = _mv[1];
  Mv mvLB = _mv[2];

  // get affine sub-block width and height
  const int width = pu.Y().width;
  const int height = pu.Y().height;
  int blockWidth = AFFINE_MIN_BLOCK_SIZE;
  int blockHeight = AFFINE_MIN_BLOCK_SIZE;

  CHECK(blockWidth  > (width >> iScaleX), "Sub Block width  > Block width");
  CHECK(blockHeight > (height >> iScaleY), "Sub Block height > Block height");
  const int MVBUFFER_SIZE = MAX_CU_SIZE / MIN_PU_SIZE;

  const int cxWidth = width >> iScaleX;
  const int cxHeight = height >> iScaleY;
  const int iHalfBW = blockWidth >> 1;
  const int iHalfBH = blockHeight >> 1;

  const int iBit = MAX_CU_DEPTH;
  int iDMvHorX = 0;
  int iDMvHorY = 0;
  int iDMvVerX = 0;
  int iDMvVerY = 0;

  iDMvHorX = (mvRT - mvLT).hor << (iBit - Log2(cxWidth));
  iDMvHorY = (mvRT - mvLT).ver << (iBit - Log2(cxWidth));
  if (pu.cu->affineType == AFFINEMODEL_6PARAM)
  {
    iDMvVerX = (mvLB - mvLT).hor << (iBit - Log2(cxHeight));
    iDMvVerY = (mvLB - mvLT).ver << (iBit - Log2(cxHeight));
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  int iMvScaleHor = mvLT.hor << iBit;
  int iMvScaleVer = mvLT.ver << iBit;
  const PPS &pps = *pu.cs->pps;
  const SPS &sps = *pu.cs->sps;
  const int iMvShift = 4;
  const int iOffset = 8;
  const int iHorMax = (pps.picWidthInLumaSamples + iOffset - pu.Y().x - 1) << iMvShift;
  const int iHorMin = (-(int)pu.cs->pcv->maxCUSize - iOffset - (int)pu.Y().x + 1) << iMvShift;
  const int iVerMax = (pps.picHeightInLumaSamples + iOffset - pu.Y().y - 1) << iMvShift;
  const int iVerMin = (-(int)pu.cs->pcv->maxCUSize - iOffset - (int)pu.Y().y + 1) << iMvShift;

  const int shift = iBit - 4 + MV_FRACTIONAL_BITS_INTERNAL;
  bool      wrapRef = false;
  const bool subblkMVSpreadOverLimit = isSubblockVectorSpreadOverLimit(iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY, pu.interDir);

  bool enablePROF = sps.PROF && (!m_skipPROF) && (compID == COMP_Y);
  enablePROF &= !((pu.cu->affineType == AFFINEMODEL_6PARAM && _mv[0] == _mv[1] && _mv[0] == _mv[2]) || (pu.cu->affineType == AFFINEMODEL_4PARAM && _mv[0] == _mv[1]));
  enablePROF &= !subblkMVSpreadOverLimit;
  const int profThres = 1 << (iBit + (m_isBi ? 1 : 0));
  enablePROF &= !m_encOnly || pu.cu->slice->checkLDC || iDMvHorX > profThres || iDMvHorY > profThres || iDMvVerX > profThres || iDMvVerY > profThres || iDMvHorX < -profThres || iDMvHorY < -profThres || iDMvVerX < -profThres || iDMvVerY < -profThres;
  enablePROF &= pps.picWidthInLumaSamples == refPic->cs->pps->picWidthInLumaSamples && pps.picHeightInLumaSamples == refPic->cs->pps->picHeightInLumaSamples;

  bool isLast = enablePROF ? false : !bi;

  const int cuExtW = AFFINE_MIN_BLOCK_SIZE + PROF_BORDER_EXT_W * 2;
  const int cuExtH = AFFINE_MIN_BLOCK_SIZE + PROF_BORDER_EXT_H * 2;

  PelBuf gradXExt(m_gradBuf[0], cuExtW, cuExtH);
  PelBuf gradYExt(m_gradBuf[1], cuExtW, cuExtH);

  int dstExtW = (((blockWidth + PROF_BORDER_EXT_W * 2 + 7) >> 3) << 3);
  int dstExtH = (blockHeight + PROF_BORDER_EXT_H * 2);

  PelBuf dstExtBuf(m_filteredBlockTmp[1][compID], dstExtW, dstExtH);

  PelBuf& dstBuf = dstPic.bufs[compID];

  int *dMvScaleHor = m_dMvBuf[refPicList];
  int *dMvScaleVer = m_dMvBuf[refPicList] + 16;

  if (enablePROF)
  {
    int* dMvH = dMvScaleHor;
    int* dMvV = dMvScaleVer;
    int quadHorX = iDMvHorX << 2;
    int quadHorY = iDMvHorY << 2;
    int quadVerX = iDMvVerX << 2;
    int quadVerY = iDMvVerY << 2;

    dMvH[0] = ((iDMvHorX + iDMvVerX) << 1) - ((quadHorX + quadVerX) << 1);
    dMvV[0] = ((iDMvHorY + iDMvVerY) << 1) - ((quadHorY + quadVerY) << 1);

    for (int w = 1; w < blockWidth; w++)
    {
      dMvH[w] = dMvH[w - 1] + quadHorX;
      dMvV[w] = dMvV[w - 1] + quadHorY;
    }

    dMvH += blockWidth;
    dMvV += blockWidth;
    for (int h = 1; h < blockHeight; h++)
    {
      for (int w = 0; w < blockWidth; w++)
      {
        dMvH[w] = dMvH[w - blockWidth] + quadVerX;
        dMvV[w] = dMvV[w - blockWidth] + quadVerY;
      }
      dMvH += blockWidth;
      dMvV += blockWidth;
    }

    const int mvShift  = 8;
    const int dmvLimit = ( 1 << 5 ) - 1;

    if (!g_pelBufOP.roundIntVector)
    {
      for (int idx = 0; idx < blockWidth * blockHeight; idx++)
      {
        roundAffineMv(dMvScaleHor[idx], dMvScaleVer[idx], mvShift);
        dMvScaleHor[idx] = Clip3(-dmvLimit, dmvLimit, dMvScaleHor[idx]);
        dMvScaleVer[idx] = Clip3(-dmvLimit, dmvLimit, dMvScaleVer[idx]);
      }
    }
    else
    {
      int sz = blockWidth * blockHeight;
      g_pelBufOP.roundIntVector(dMvScaleHor, sz, mvShift, dmvLimit);
      g_pelBufOP.roundIntVector(dMvScaleVer, sz, mvShift, dmvLimit);
    }
  }

  int scaleXLuma = getComponentScaleX(COMP_Y, chFmt);
  int scaleYLuma = getComponentScaleY(COMP_Y, chFmt);
  if ((pu.mcControl > 3) && (compID == COMP_Cb) && pu.chromaFormat != CHROMA_444)
  {
    CHECK(compID == COMP_Y, "Chroma only subblock MV calculation should not apply to Luma");
    int lumaBlockWidth = AFFINE_MIN_BLOCK_SIZE;
    int lumaBlockHeight = AFFINE_MIN_BLOCK_SIZE;

    CHECK(lumaBlockWidth > (width >> scaleXLuma), "Sub Block width  > Block width");
    CHECK(lumaBlockHeight > (height >> scaleYLuma), "Sub Block height > Block height");

    const int cxWidthLuma = width >> scaleXLuma;
    const int cxHeightLuma = height >> scaleYLuma;
    const int halfBWLuma = lumaBlockWidth >> 1;
    const int halfBHLuma = lumaBlockHeight >> 1;

    int dMvHorXLuma, dMvHorYLuma, dMvVerXLuma, dMvVerYLuma;
    dMvHorXLuma = (mvRT - mvLT).hor << (iBit - floorLog2(cxWidthLuma));
    dMvHorYLuma = (mvRT - mvLT).ver << (iBit - floorLog2(cxWidthLuma));
    if (pu.cu->affineType == AFFINEMODEL_6PARAM)
    {
      dMvVerXLuma = (mvLB - mvLT).hor << (iBit - floorLog2(cxHeightLuma));
      dMvVerYLuma = (mvLB - mvLT).ver << (iBit - floorLog2(cxHeightLuma));
    }
    else
    {
      dMvVerXLuma = -dMvHorYLuma;
      dMvVerYLuma = dMvHorXLuma;
    }

    const bool subblkMVSpreadOverLimitLuma = isSubblockVectorSpreadOverLimit(dMvHorXLuma, dMvHorYLuma, dMvVerXLuma, dMvVerYLuma, pu.interDir);

    // get luma MV block by block
    for (int h = 0; h < cxHeightLuma; h += lumaBlockHeight)
    {
      for (int w = 0; w < cxWidthLuma; w += lumaBlockWidth)
      {
        int mvScaleTmpHor, mvScaleTmpVer;
        if (!subblkMVSpreadOverLimitLuma)
        {
          mvScaleTmpHor = iMvScaleHor + dMvHorXLuma * (halfBWLuma + w) + dMvVerXLuma * (halfBHLuma + h);
          mvScaleTmpVer = iMvScaleVer + dMvHorYLuma * (halfBWLuma + w) + dMvVerYLuma * (halfBHLuma + h);
        }
        else
        {
          mvScaleTmpHor = iMvScaleHor + dMvHorXLuma * (cxWidthLuma >> 1) + dMvVerXLuma * (cxHeightLuma >> 1);
          mvScaleTmpVer = iMvScaleVer + dMvHorYLuma * (cxWidthLuma >> 1) + dMvVerYLuma * (cxHeightLuma >> 1);
        }

        roundAffineMv(mvScaleTmpHor, mvScaleTmpVer, shift);
        Mv tmpMv(mvScaleTmpHor, mvScaleTmpVer);
        tmpMv.clipToStorageBitDepth();
        mvScaleTmpHor = tmpMv.hor;
        mvScaleTmpVer = tmpMv.ver;

        m_storedMv[h / AFFINE_MIN_BLOCK_SIZE * MVBUFFER_SIZE + w / AFFINE_MIN_BLOCK_SIZE].set(mvScaleTmpHor, mvScaleTmpVer);
      }
    }
  }
  // get prediction block by block
  const CPelBuf refBuf     = refPic->getRecoBuf(compID);
  const CPelBuf refBufWrap; //no support = refPic->getRecoWrapBuf(compID);

  const int puX = pu.blocks[compID].x;
  const int puY = pu.blocks[compID].y;

  for (int h = 0; h < cxHeight; h += blockHeight)
  {
    for (int w = 0; w < cxWidth; w += blockWidth)
    {
      int iMvScaleTmpHor, iMvScaleTmpVer;
      if (compID == COMP_Y || pu.chromaFormat == CHROMA_444)
      {
        if (!subblkMVSpreadOverLimit)
        {
          iMvScaleTmpHor = iMvScaleHor + iDMvHorX * (iHalfBW + w) + iDMvVerX * (iHalfBH + h);
          iMvScaleTmpVer = iMvScaleVer + iDMvHorY * (iHalfBW + w) + iDMvVerY * (iHalfBH + h);
        }
        else
        {
          iMvScaleTmpHor = iMvScaleHor + iDMvHorX * (cxWidth >> 1) + iDMvVerX * (cxHeight >> 1);
          iMvScaleTmpVer = iMvScaleVer + iDMvHorY * (cxWidth >> 1) + iDMvVerY * (cxHeight >> 1);
        }

        roundAffineMv(iMvScaleTmpHor, iMvScaleTmpVer, shift);
        Mv tmpMv(iMvScaleTmpHor, iMvScaleTmpVer);
        tmpMv.clipToStorageBitDepth();
        iMvScaleTmpHor = tmpMv.hor;
        iMvScaleTmpVer = tmpMv.ver;

        // clip and scale
        if( pu.cs->sps->wrapAroundEnabled )
        {
          m_storedMv[h / AFFINE_MIN_BLOCK_SIZE * MVBUFFER_SIZE + w / AFFINE_MIN_BLOCK_SIZE].set(iMvScaleTmpHor, iMvScaleTmpVer);
          Mv tmpMv(iMvScaleTmpHor, iMvScaleTmpVer);
          wrapRef = wrapClipMv(tmpMv, Position(pu.Y().x + w, pu.Y().y + h), Size(blockWidth, blockHeight), *pu.cs);
          iMvScaleTmpHor = tmpMv.hor;
          iMvScaleTmpVer = tmpMv.ver;
        }
        else
        {
          wrapRef = false;
          m_storedMv[h / AFFINE_MIN_BLOCK_SIZE * MVBUFFER_SIZE + w / AFFINE_MIN_BLOCK_SIZE].set(iMvScaleTmpHor, iMvScaleTmpVer);
       //   if( scalingRatio == SCALE_1X ) 
          {
            iMvScaleTmpHor = std::min<int>(iHorMax, std::max<int>(iHorMin, iMvScaleTmpHor));
            iMvScaleTmpVer = std::min<int>(iVerMax, std::max<int>(iVerMin, iMvScaleTmpVer));
          }
        }
      }
      else
      {
        Mv curMv = m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE) * MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE)] +
          m_storedMv[((h << iScaleY) / AFFINE_MIN_BLOCK_SIZE + iScaleY)* MVBUFFER_SIZE + ((w << iScaleX) / AFFINE_MIN_BLOCK_SIZE + iScaleX)];
        roundAffineMv(curMv.hor, curMv.ver, 1);
        if (pu.cs->sps->wrapAroundEnabled)
        {
          wrapRef = wrapClipMv(curMv, Position(pu.Y().x + (w << iScaleX), pu.Y().y + (h << iScaleY)), Size(blockWidth << iScaleX, blockHeight << iScaleY), *pu.cs);
        }
        else
        {
          wrapRef = false;
//          if( scalingRatio == SCALE_1X ) 
          {
            curMv.hor = std::min<int>(iHorMax, std::max<int>(iHorMin, curMv.hor));
            curMv.ver = std::min<int>(iVerMax, std::max<int>(iVerMin, curMv.ver));
          }
        }
        iMvScaleTmpHor = curMv.hor;
        iMvScaleTmpVer = curMv.ver;
      }

      // get the MV in high precision
      int xFrac, yFrac, xInt, yInt;

      if (!iScaleX)
      {
        xInt = iMvScaleTmpHor >> 4;
        xFrac = iMvScaleTmpHor & 15;
      }
      else
      {
        xInt = iMvScaleTmpHor >> 5;
        xFrac = iMvScaleTmpHor & 31;
      }
      if (!iScaleY)
      {
        yInt = iMvScaleTmpVer >> 4;
        yFrac = iMvScaleTmpVer & 15;
      }
      else
      {
        yInt = iMvScaleTmpVer >> 5;
        yFrac = iMvScaleTmpVer & 31;
      }

      Pel* ref = wrapRef ? ( Pel* ) refBufWrap.buf : ( Pel* ) refBuf.buf;
      ref     +=   puX + xInt + w;
      ref     += ( puY + yInt + h ) * ( wrapRef ? refBufWrap.stride : refBuf.stride );
      Pel* dst = dstBuf.buf + w + h * dstBuf.stride;

      int refStride = refBuf.stride;
      int dstStride = dstBuf.stride;

      int bw = blockWidth;
      int bh = blockHeight;

      if( enablePROF )
      {
        dst = dstExtBuf.bufAt(PROF_BORDER_EXT_W, PROF_BORDER_EXT_H);
        dstStride = dstExtBuf.stride;
      }

      if( xFrac && yFrac )
      {
        m_if.filter4x4( compID, ref, refStride, dst, dstStride, 4, 4, xFrac, yFrac, isLast, chFmt, clpRng );
      }
      else if( !yFrac )
      {
        m_if.filterHor( compID, ( Pel* ) ref, refStride, dst, dstStride, bw, bh, xFrac, isLast, chFmt, clpRng );
      }
      else if( xFrac == 0 )
      {
        m_if.filterVer( compID, ( Pel* ) ref, refStride, dst, dstStride, bw, bh, yFrac, true, isLast, chFmt, clpRng );
      }
      
      if (enablePROF)
      {
        const unsigned shift = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
        const int xOffset = xFrac >> 3;
        const int yOffset = yFrac >> 3;

        const int refOffset = (blockHeight + 1) * refStride;
        const int dstOffset = (blockHeight + 1)* dstStride;

        const Pel* refPel = ref - (1 - yOffset) * refStride + xOffset - 1;
        Pel* dstPel = dst - dstStride - 1;
        for (int pw = 0; pw < blockWidth + 2; pw++)
        {
          dstPel[pw] = leftShiftU(refPel[pw], shift) - (Pel)IF_INTERNAL_OFFS;
          dstPel[pw + dstOffset] = leftShiftU(refPel[pw + refOffset], shift) - (Pel)IF_INTERNAL_OFFS;
        }

        refPel = ref + yOffset * refBuf.stride + xOffset;
        dstPel = dst;
        for (int ph = 0; ph < blockHeight; ph++, refPel += refStride, dstPel += dstStride)
        {
          dstPel[-1] = leftShiftU(refPel[-1], shift) - (Pel)IF_INTERNAL_OFFS;
          dstPel[blockWidth] = leftShiftU(refPel[blockWidth], shift) - (Pel)IF_INTERNAL_OFFS;
        }

        PelBuf gradXBuf = gradXExt.subBuf(0, 0, blockWidth + 2, blockHeight + 2);
        PelBuf gradYBuf = gradYExt.subBuf(0, 0, blockWidth + 2, blockHeight + 2);

        g_pelBufOP.profGradFilter(dstExtBuf.buf, dstExtBuf.stride, blockWidth + 2, blockHeight + 2, gradXBuf.stride, gradXBuf.buf, gradYBuf.buf, clpRng.bd);
        const int shiftNum = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
        const Pel offset = (1 << (shiftNum - 1)) + IF_INTERNAL_OFFS;
        Pel* src = dstExtBuf.bufAt(PROF_BORDER_EXT_W, PROF_BORDER_EXT_H);
        Pel* gX = gradXBuf.bufAt(PROF_BORDER_EXT_W, PROF_BORDER_EXT_H);
        Pel* gY = gradYBuf.bufAt(PROF_BORDER_EXT_W, PROF_BORDER_EXT_H);

        Pel*  dstY = dstBuf.bufAt(w, h);

        g_pelBufOP.applyPROF(dstY, dstBuf.stride, src, dstExtBuf.stride, blockWidth, blockHeight, gX, gY, gradXBuf.stride, dMvScaleHor, dMvScaleVer, blockWidth, bi, shiftNum, offset, clpRng);
      }
    }
  }

}

} // namespace vvenc

//! \}


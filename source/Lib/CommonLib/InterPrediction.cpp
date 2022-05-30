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

void applyPROFCore(Pel* dst, int dstStride, const Pel* src, int srcStride, int width, int height, const Pel* gradX, const Pel* gradY, int gradStride, const int* dMvX, const int* dMvY, int dMvStride, const bool& bi, int shiftNum, Pel offset, const ClpRng& clpRng)
{
  int idx = 0;
  const int dILimit = 1 << std::max<int>(clpRng.bd + 1, 13);
  for (int h = 0; h < height; h++)
  {
    for (int w = 0; w < width; w++)
    {
      int32_t dI = dMvX[idx] * gradX[w] + dMvY[idx] * gradY[w];
      dI = Clip3(-dILimit, dILimit - 1, dI);
      dst[w] = src[w] + dI;
      if (!bi)
      {
        dst[w] = (dst[w] + offset) >> shiftNum;
        dst[w] = ClipPel(dst[w], clpRng);
      }
      idx++;
    }
    gradX += gradStride;
    gradY += gradStride;
    dst += dstStride;
    src += srcStride;
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


template<int padSize>
void paddingCore(Pel *ptr, int stride, int width, int height)
{
  /*left and right padding*/
  Pel *ptrTemp1 = ptr;
  Pel *ptrTemp2 = ptr + (width - 1);
  ptrdiff_t offset = 0;
  for (int i = 0; i < height; i++)
  {
    offset = stride * i;
    for (int j = 1; j <= padSize; j++)
    {
      *(ptrTemp1 - j + offset) = *(ptrTemp1 + offset);
      *(ptrTemp2 + j + offset) = *(ptrTemp2 + offset);
    }
  }
  /*Top and Bottom padding*/
  int numBytes = (width + padSize + padSize) * sizeof(Pel);
  ptrTemp1 = (ptr - padSize);
  ptrTemp2 = (ptr + (stride * (height - 1)) - padSize);
  for (int i = 1; i <= padSize; i++)
  {
    memcpy(ptrTemp1 - (i * stride), (ptrTemp1), numBytes);
    memcpy(ptrTemp2 + (i * stride), (ptrTemp2), numBytes);
  }
}

void padDmvrCore( const Pel* src, const int srcStride, Pel* dst, const int dstStride, int width, int height, int padSize )
{
  g_pelBufOP.copyBuffer( ( const char* ) src, srcStride * sizeof( Pel ), ( char* ) dst, dstStride * sizeof( Pel ), width * sizeof( Pel ), height );
  if( padSize == 1 )
    paddingCore<1>( dst, dstStride, width, height );
  else
    paddingCore<2>( dst, dstStride, width, height );
}

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

InterPrediction::InterPrediction()
  : m_currChromaFormat( NUM_CHROMA_FORMAT )
  , m_subPuMC(false)
  , m_IBCBufferWidth(0)
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
  m_IBCBuffer.destroy();
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
  if (m_IBCBufferWidth != g_IBCBufferSize / ctuSize)
  {
    m_IBCBuffer.destroy();
  }
  if (m_IBCBuffer.bufs.empty())
  {
    m_IBCBufferWidth = g_IBCBufferSize / ctuSize;
    m_IBCBuffer.create(UnitArea(chFormat, Area(0, 0, m_IBCBufferWidth, ctuSize)));
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool InterPrediction::xCheckIdenticalMotion( const CodingUnit& cu ) const
{
  const Slice &slice = *cu.cs->slice;

  if( slice.isInterB() && !cu.cs->pps->weightedBiPred )
  {
    if( cu.refIdx[0] >= 0 && cu.refIdx[1] >= 0 )
    {
      int RefPOCL0 = slice.getRefPic( REF_PIC_LIST_0, cu.refIdx[0] )->getPOC();
      int RefPOCL1 = slice.getRefPic( REF_PIC_LIST_1, cu.refIdx[1] )->getPOC();

      if( RefPOCL0 == RefPOCL1 )
      {
        if( !cu.affine )
        {
          if( cu.mv[0][0] == cu.mv[1][0] )
          {
            return true;
          }
        }
        else
        {
          if( cu.mv[0][0] == cu.mv[1][0] && cu.mv[0][1] == cu.mv[1][1] && ( cu.affineType == AFFINEMODEL_4PARAM || cu.mv[0][2] == cu.mv[1][2] ) )
          {
            return true;
          }
        }
      }
    }
  }

  return false;
}

void InterPrediction::xSubPuBDOF( const CodingUnit& cu, PelUnitBuf& predBuf, const RefPicList& refPicList /*= REF_PIC_LIST_X*/)
{
  Position puPos = cu.lumaPos();
  Size puSize = cu.lumaSize();

  CodingUnit subCu = cu;  // th we do not need all that stuff 
  subCu.cs             = cu.cs;
  subCu.mergeType      = cu.mergeType;
  subCu.mmvdMergeFlag  = cu.mmvdMergeFlag;
  subCu.mcControl      = cu.mcControl;
  subCu.mergeFlag      = cu.mergeFlag;
  subCu.ciip           = cu.ciip;
  subCu.mvRefine       = cu.mvRefine;
  subCu.refIdx[0]      = cu.refIdx[0];
  subCu.refIdx[1]      = cu.refIdx[1];

  const int  yEnd      = puPos.y + puSize.height;
  const int  xEnd      = puPos.x + puSize.width;
  const int  dy        = std::min((int)MAX_BDOF_APPLICATION_REGION, (int)puSize.height);
  const int  dx        = std::min((int)MAX_BDOF_APPLICATION_REGION, (int)puSize.width);
  for (int y = puPos.y; y < yEnd; y += dy)
  {
    for (int x = puPos.x; x < xEnd; x += dx)
    {
      const MotionInfo &curMi = cu.getMotionInfo(Position{ x, y });

      subCu.UnitArea::operator=(UnitArea(cu.chromaFormat, Area(x, y, dx, dy)));
      subCu = curMi;
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(cu, subCu));

      motionCompensation(subCu, subPredBuf, refPicList);
    }
  }
}
void InterPrediction::xPredInterUni(const CodingUnit& cu, const RefPicList& refPicList, PelUnitBuf& pcYuvPred, const bool bi, const bool bdofApplied)
{
  int iRefIdx = cu.refIdx[refPicList];
  Mv mv[3];
  bool isIBC = false;
  CHECK(!CU::isIBC(cu) && cu.lwidth() == 4 && cu.lheight() == 4, "invalid 4x4 inter blocks");
  if (CU::isIBC(cu))
  {
    isIBC = true;
  }
  if (cu.affine)
  {
    CHECK(iRefIdx < 0, "iRefIdx incorrect.");

    mv[0] = cu.mv[refPicList][0];
    mv[1] = cu.mv[refPicList][1];
    mv[2] = cu.mv[refPicList][2];
  }
  else
  {
    mv[0] = cu.mv[refPicList][0];
    if (!isIBC )
      clipMv(mv[0], cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
  }

  for( uint32_t comp = COMP_Y; comp < pcYuvPred.bufs.size(); comp++ )
  {
    bool luma = cu.mcControl <= 3;
    bool chroma = (cu.mcControl >> 1) != 1 ;
    const ComponentID compID = ComponentID( comp );
    if (compID == COMP_Y && !luma)
      continue;
    if (compID != COMP_Y && !chroma)
      continue;
    if (cu.affine)
    {
      xPredAffineBlk(compID, cu, cu.slice->getRefPic(refPicList, iRefIdx), mv, pcYuvPred, bi, cu.slice->clpRngs[compID], refPicList);
    }
    else
    {
      if (isIBC)
      {
        xPredInterBlk(compID, cu, cu.slice->pic, mv[0], pcYuvPred, bi, cu.slice->clpRngs[compID], bdofApplied, isIBC);
      }
      else
      {
        xPredInterBlk(compID, cu, cu.slice->getRefPic(refPicList, iRefIdx), mv[0], pcYuvPred, bi, cu.slice->clpRngs[compID], bdofApplied, isIBC, refPicList);
      }
    }
  }
}

void InterPrediction::xPredInterBi( const CodingUnit& cu, PelUnitBuf& yuvPred, const bool bdofApplied, PelUnitBuf *yuvPredTmp )
{
  CHECK( !cu.affine && cu.refIdx[0] >= 0 && cu.refIdx[1] >= 0 && ( cu.lwidth() + cu.lheight() == 12 ), "invalid 4x8/8x4 bi-predicted blocks" );

  PelUnitBuf puBuf[NUM_REF_PIC_LIST_01];
  for (uint32_t refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if( cu.refIdx[refList] < 0)
    {
      continue;
    }

    RefPicList refPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

    CHECK(CU::isIBC(cu) && refPicList != REF_PIC_LIST_0, "Invalid interdir for ibc mode");
    CHECK(CU::isIBC(cu) && cu.refIdx[refList] != MAX_NUM_REF, "Invalid reference index for ibc mode");
    CHECK((CU::isInter(cu) && cu.refIdx[refList] >= cu.cs->slice->numRefIdx[ refPicList ]), "Invalid reference index");

    puBuf[refList] = m_yuvPred[refList].getCompactBuf( cu );

    if( cu.refIdx[0] >= 0 && cu.refIdx[1] >= 0 )
    {
      xPredInterUni ( cu, refPicList, puBuf[refList], true, bdofApplied );
    }
    else
    {
      xPredInterUni( cu, refPicList, puBuf[refList], cu.geo, bdofApplied );
    }
  }

  xWeightedAverage( cu, puBuf[0], puBuf[1], yuvPred, bdofApplied, yuvPredTmp );
}

void InterPrediction::motionCompensationIBC( CodingUnit& cu, PelUnitBuf& predBuf )
{
  if (!cu.cs->pcv->isEncoder)
  {
    if (CU::isIBC(cu))
    {
      bool luma = cu.mcControl <= 3;
      bool chroma = (cu.mcControl >> 1) != 1;
      CHECK(!luma, "IBC only for Chroma is not allowed.");
      xIntraBlockCopyIBC(cu, predBuf, COMP_Y);
      if (chroma && isChromaEnabled(cu.chromaFormat))
      {
        xIntraBlockCopyIBC(cu, predBuf, COMP_Cb);
        xIntraBlockCopyIBC(cu, predBuf, COMP_Cr);
      }
      return;
    }
  }
  // dual tree handling for IBC as the only ref
  xPredInterUni(cu, REF_PIC_LIST_0, predBuf, false, false );
}

bool InterPrediction::motionCompensation( CodingUnit& cu, PelUnitBuf& predBuf, const RefPicList refPicList, PelUnitBuf* predBufDfltWght )
{
  bool ret = false;
  if( refPicList != REF_PIC_LIST_X )
  {
    xPredInterUni( cu, refPicList, predBuf, false, false );
  }
  else
  {
    CHECK( !cu.affine && cu.refIdx[0] >= 0 && cu.refIdx[1] >= 0 && ( cu.lwidth() + cu.lheight() == 12 ), "invalid 4x8/8x4 bi-predicted blocks" );
    bool bdofApplied = false;
    if( cu.cs->sps->BDOF && ( !cu.cs->picHeader->disBdofFlag ) )
    {
      if (cu.affine || m_subPuMC || cu.ciip )
      {
        bdofApplied = false;
      }
      else
      {
        if (CU::isBiPredFromDifferentDirEqDistPoc(cu)
          && (cu.Y().height >= 8)
          && (cu.Y().width >= 8)
          && ((cu.Y().height * cu.Y().width) >= 128)
          && !(cu.smvdMode)
          && !(cu.cs->sps->BCW && cu.BcwIdx != BCW_DEFAULT)
          && !(((cu.mcControl & 1) == 1) && cu.mmvdMergeFlag)
          )
        {
          bdofApplied = true;
        }
      }
    }

    bool dmvrApplied = false;
    dmvrApplied = (cu.mvRefine) && CU::checkDMVRCondition(cu);
    if ((cu.lumaSize().width > MAX_BDOF_APPLICATION_REGION || cu.lumaSize().height > MAX_BDOF_APPLICATION_REGION) && cu.mergeType != MRG_TYPE_SUBPU_ATMVP && (bdofApplied && !dmvrApplied))
    {
      xSubPuBDOF( cu, predBuf, refPicList );
    }
    else if (cu.mergeType != MRG_TYPE_DEFAULT_N && cu.mergeType != MRG_TYPE_IBC)
    {
      xSubPuMC(cu, predBuf, refPicList);
    }
    else if( xCheckIdenticalMotion( cu ) )
    {
      xPredInterUni( cu, REF_PIC_LIST_0, predBuf, false, false );
      if( predBufDfltWght )
      {
        predBufDfltWght->copyFrom( predBuf );
      }
    }
    else if (dmvrApplied)
    {
      xProcessDMVR( cu, predBuf, cu.slice->clpRngs, bdofApplied );
    }
    else
    {
      xPredInterBi( cu, predBuf, bdofApplied, predBufDfltWght );
    }
    DTRACE( g_trace_ctx, D_MOT_COMP, "BIDOF=%d, DMVR=%d\n", bdofApplied, dmvrApplied );
    ret = bdofApplied || dmvrApplied;
  }
  DTRACE( g_trace_ctx, D_MOT_COMP, "MV=%d,%d\n", cu.mv[0][0].hor, cu.mv[0][0].ver );
  DTRACE( g_trace_ctx, D_MOT_COMP, "MV=%d,%d\n", cu.mv[1][0].hor, cu.mv[1][0].ver );
  DTRACE_PEL_BUF( D_MOT_COMP, predBuf.Y(), cu, cu.predMode, COMP_Y );
  if( cu.chromaFormat != VVENC_CHROMA_400 )
  {
    DTRACE_PEL_BUF( D_MOT_COMP, predBuf.Cb(), cu, cu.predMode, COMP_Cb );
    DTRACE_PEL_BUF( D_MOT_COMP, predBuf.Cr(), cu, cu.predMode, COMP_Cr );
  }
  return ret;
}

void InterPrediction::xSubPuMC(CodingUnit& cu, PelUnitBuf& predBuf, const RefPicList& eRefPicList /*= REF_PIC_LIST_X*/)
{
  Position puPos = cu.lumaPos();
  Size puSize = cu.lumaSize();

  int numPartLine, numPartCol, puHeight, puWidth;
  {
    numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
    numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
    puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
    puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;
  }

  CodingUnit subCu = cu;
  subCu.cs = cu.cs;
  subCu.mergeType = MRG_TYPE_DEFAULT_N;

  bool isAffine = cu.affine;
  subCu.affine = false;

  // join sub-pus containing the same motion
  bool verMC = puSize.height > puSize.width;
  int  fstStart = (!verMC ? puPos.y : puPos.x);
  int  secStart = (!verMC ? puPos.x : puPos.y);
  int  fstEnd = (!verMC ? puPos.y + puSize.height : puPos.x + puSize.width);
  int  secEnd = (!verMC ? puPos.x + puSize.width : puPos.y + puSize.height);
  int  fstStep = (!verMC ? puHeight : puWidth);
  int  secStep = (!verMC ? puWidth : puHeight);

  cu.refIdx[0] = 0;
  cu.refIdx[1] = cu.cs->slice->sliceType == VVENC_B_SLICE ? 0 : -1;
  bool scaled = false;//!CU::isRefPicSameSize(cu);

  m_subPuMC = true;

  for (int fstDim = fstStart; fstDim < fstEnd; fstDim += fstStep)
  {
    for (int secDim = secStart; secDim < secEnd; secDim += secStep)
    {
      int x = !verMC ? secDim : fstDim;
      int y = !verMC ? fstDim : secDim;
      const MotionInfo &curMi = cu.getMotionInfo(Position{ x, y });

      int length = secStep;
      int later = secDim + secStep;

      while (later < secEnd)
      {
        const MotionInfo &laterMi = !verMC ? cu.getMotionInfo(Position{ later, fstDim }) : cu.getMotionInfo(Position{ fstDim, later });
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

      subCu.UnitArea::operator=(UnitArea(cu.chromaFormat, Area(x, y, dx, dy)));
      subCu = curMi;
      PelUnitBuf subPredBuf = predBuf.subBuf(UnitAreaRelative(cu, subCu));
      subCu.mcControl = (cu.mcControl >> 1) << 1;
      subCu.mvRefine = false;
      motionCompensation(subCu, subPredBuf, eRefPicList);
      secDim = later - secStep;
    }
  }
  m_subPuMC = false;

  cu.affine = isAffine;
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
      VALGRIND_MEMCLEAR( m_filteredBlockTmp[i][c], sizeof( Pel ) * (extWidth + 4) * (extHeight + 7 + 4) );

      for( uint32_t j = 0; j < LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS_SIGNAL; j++ )
      {
        m_filteredBlock[i][j][c] = ( Pel* ) xMalloc( Pel, extWidth * extHeight );
        VALGRIND_MEMCLEAR( m_filteredBlock[i][j][c], sizeof( Pel ) * extWidth * extHeight );
      }
    }
  }

  m_gradX0 = (Pel*)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
  m_gradY0 = (Pel*)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
  m_gradX1 = (Pel*)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);
  m_gradY1 = (Pel*)xMalloc(Pel, BDOF_TEMP_BUFFER_SIZE);

  VALGRIND_MEMCLEAR( m_gradX0, sizeof( Pel ) * BDOF_TEMP_BUFFER_SIZE );
  VALGRIND_MEMCLEAR( m_gradY0, sizeof( Pel ) * BDOF_TEMP_BUFFER_SIZE );
  VALGRIND_MEMCLEAR( m_gradX1, sizeof( Pel ) * BDOF_TEMP_BUFFER_SIZE );
  VALGRIND_MEMCLEAR( m_gradY1, sizeof( Pel ) * BDOF_TEMP_BUFFER_SIZE );

  m_if.initInterpolationFilter( true );

  xFpBDOFGradFilter = gradFilterCore;
  xFpProfGradFilter = gradFilterCore<false>;
  xFpApplyPROF      = applyPROFCore;
  xFpPadDmvr        = padDmvrCore;

#if ENABLE_SIMD_OPT_BDOF && defined( TARGET_SIMD_X86 )
  initInterPredictionX86();
#endif

  if (m_storedMv == nullptr)
  {
    const int MVBUFFER_SIZE = MAX_CU_SIZE / MIN_PU_SIZE;
    m_storedMv = new Mv[MVBUFFER_SIZE*MVBUFFER_SIZE];
#if ENABLE_VALGRIND_CODE
    for( int i = 0; i < MVBUFFER_SIZE * MVBUFFER_SIZE; i++ )
    {
      m_storedMv[i].setZero();
    }
#endif
  }
}

void InterPredInterpolation::xPredInterBlk ( const ComponentID compID, const CodingUnit& cu, const Picture* refPic, const Mv& _mv, PelUnitBuf& dstPic, const bool bi, const ClpRng& clpRng
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
  const ChromaFormat  chFmt = cu.chromaFormat;
  const bool          rndRes = !bi;

  int shiftHor = MV_FRACTIONAL_BITS_INTERNAL + getComponentScaleX(compID, chFmt);
  int shiftVer = MV_FRACTIONAL_BITS_INTERNAL + getComponentScaleY(compID, chFmt);

  bool  wrapRef = false;
  Mv    mv(_mv);
  if( !isIBC && cu.cs->pcv->wrapArround )
  {
    wrapRef = wrapClipMv( mv, cu.blocks[0].pos(), cu.blocks[0].size(), *cu.cs);
  }
  int xFrac = mv.hor & ((1 << shiftHor) - 1);
  int yFrac = mv.ver & ((1 << shiftVer) - 1);
  if (isIBC)
  {
    xFrac = yFrac = 0;
  }

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
    Position offset = cu.blocks[compID].pos().offset( mv.hor >> shiftHor, mv.ver >> shiftVer );
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
  bool useAltHpelIf = cu.imv == IMV_HPEL;

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
      PelBuf tmpBuf( m_filteredBlockTmp[0][compID], dmvrWidth ? dmvrWidth : dstBuf.stride, dmvrWidth ? Size( dmvrWidth, dmvrHeight ) : cu.blocks[compID].size() );

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

  if( xFpBiDirOptFlow )
  {
    xFpBiDirOptFlow( srcY0, srcY1, gradX0, gradX1, gradY0, gradY1, width, height, dstY, dstStride, shiftNum, offset, limit, clpRng, bitDepth );
    return;
  }

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

      calcBDOFSumsCore(SrcY0Tmp, SrcY1Tmp, pGradX0Tmp, pGradX1Tmp, pGradY0Tmp, pGradY1Tmp, xu, yu, src0Stride, src1Stride, widthG, bitDepth, &sumAbsGX, &sumAbsGY, &sumDIX, &sumDIY, &sumSignGY_GX);
      tmpx = (sumAbsGX == 0 ? 0 : xRightShiftMSB(4 * sumDIX, sumAbsGX));
      tmpx = Clip3(-limit, limit, tmpx);

      const int tmpData = sumSignGY_GX * tmpx >> 1;
      tmpy = (sumAbsGY == 0 ? 0 : xRightShiftMSB((4 * sumDIY - tmpData), sumAbsGY));
      tmpy = Clip3(-limit, limit, tmpy);

      srcY0Temp = srcY0 + (stridePredMC + 1) + ((yu*src0Stride + xu) << 2);
      srcY1Temp = srcY1 + (stridePredMC + 1) + ((yu*src0Stride + xu) << 2);
      gradX0 = m_gradX0 + offsetPos + ((yu*widthG + xu) << 2);
      gradX1 = m_gradX1 + offsetPos + ((yu*widthG + xu) << 2);
      gradY0 = m_gradY0 + offsetPos + ((yu*widthG + xu) << 2);
      gradY1 = m_gradY1 + offsetPos + ((yu*widthG + xu) << 2);

      dstY0 = dstY + ((yu*dstStride + xu) << 2);
      addBDOFAvgCore(srcY0Temp, src0Stride, srcY1Temp, src1Stride, dstY0, dstStride, gradX0, gradX1, gradY0, gradY1, widthG, (1 << 2), (1 << 2), tmpx, tmpy, shiftNum, offset, clpRng);
    }  // xu
  }  // yu
}

void InterPredInterpolation::xWeightedAverage( const CodingUnit& cu, const CPelUnitBuf& pcYuvSrc0, const CPelUnitBuf& pcYuvSrc1, PelUnitBuf& pcYuvDst, const bool bdofApplied, PelUnitBuf *yuvPredTmp )
{
  const bool lumaOnly = (cu.mcControl >> 1) == 1;
  const bool chromaOnly = cu.mcControl > 3;
  CHECK((chromaOnly && lumaOnly), "should not happen");

  const ClpRngs& clpRngs = cu.slice->clpRngs;
  const int iRefIdx0 = cu.refIdx[0];
  const int iRefIdx1 = cu.refIdx[1];

  if( iRefIdx0 >= 0 && iRefIdx1 >= 0 )
  {
    if( cu.BcwIdx != BCW_DEFAULT && ( yuvPredTmp || !cu.ciip) )
    {
      CHECK( bdofApplied, "Bcw is disallowed with BIO" );
      pcYuvDst.addWeightedAvg( pcYuvSrc0, pcYuvSrc1, clpRngs, cu.BcwIdx, chromaOnly, lumaOnly );
      if( yuvPredTmp )
      {
        yuvPredTmp->addAvg( pcYuvSrc0, pcYuvSrc1, clpRngs, chromaOnly, lumaOnly );
      }
      return;
    }
    
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
    if (cu.geo)
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
    if (cu.geo)
    {
      pcYuvDst.copyFrom(pcYuvSrc1);
    }
    else
    {
      pcYuvDst.copyClip(pcYuvSrc1, clpRngs, lumaOnly, chromaOnly);
    }
  }
}

void InterPrediction::motionCompensationGeo(CodingUnit& cu, PelUnitBuf& predBuf, const MergeCtx &geoMrgCtx)
{
  const ClpRngs & clpRngs  = cu.slice->clpRngs;
  const UnitArea localUnitArea(cu.chromaFormat, Area(0, 0, cu.lwidth(), cu.lheight()));

  PelUnitBuf     tmpGeoBuf0 = m_geoPartBuf[0].getBuf(localUnitArea);
  PelUnitBuf     tmpGeoBuf1 = m_geoPartBuf[1].getBuf(localUnitArea);

  geoMrgCtx.setMergeInfo(cu, cu.geoMergeIdx0);
  CU::spanMotionInfo(cu);
  motionCompensation(cu, tmpGeoBuf0, REF_PIC_LIST_X);   // TODO: check 4:0:0 interaction with weighted prediction.

  geoMrgCtx.setMergeInfo(cu, cu.geoMergeIdx1);
  CU::spanMotionInfo(cu);
  motionCompensation(cu, tmpGeoBuf1, REF_PIC_LIST_X);   // TODO: check 4:0:0 interaction with weighted prediction.

  weightedGeoBlk(clpRngs, cu, cu.geoSplitDir, isChromaEnabled(cu.chromaFormat) ? MAX_NUM_CH : CH_L, predBuf, tmpGeoBuf0,
                  tmpGeoBuf1);
}

void InterPredInterpolation::weightedGeoBlk(const ClpRngs &clpRngs, CodingUnit& cu, const uint8_t splitDir,
                                            int32_t channel, PelUnitBuf &predDst, PelUnitBuf &predSrc0, PelUnitBuf &predSrc1)
{
  if (channel == CH_L)
  {
    m_if.weightedGeoBlk(clpRngs,cu, cu.lumaSize().width, cu.lumaSize().height, COMP_Y, splitDir, predDst, predSrc0,
                        predSrc1);
  }
  else if (channel == CH_C)
  {
    m_if.weightedGeoBlk(clpRngs, cu, cu.chromaSize().width, cu.chromaSize().height, COMP_Cb, splitDir, predDst, predSrc0,
                        predSrc1);
    m_if.weightedGeoBlk(clpRngs, cu, cu.chromaSize().width, cu.chromaSize().height, COMP_Cr, splitDir, predDst, predSrc0,
                        predSrc1);
  }
  else
  {
    m_if.weightedGeoBlk(clpRngs, cu, cu.lumaSize().width, cu.lumaSize().height, COMP_Y, splitDir, predDst, predSrc0,
                        predSrc1);
    if (isChromaEnabled(cu.chromaFormat))
    {
      m_if.weightedGeoBlk(clpRngs, cu, cu.chromaSize().width, cu.chromaSize().height, COMP_Cb, splitDir, predDst,
                          predSrc0, predSrc1);
      m_if.weightedGeoBlk(clpRngs, cu, cu.chromaSize().width, cu.chromaSize().height, COMP_Cr, splitDir, predDst,
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
      // the buffer m_yuvPad[i].bufs[0].buf is aligned to 32
      // the actual begin of the written to buffer is m_yuvPad[i].bufs[0].buf - 3 * stride - 3 = m_yuvPad[i].bufs[0].buf - 99,
      // which is not aligned with int. Since the margin on the left side is 1 sample too big, moving the buffer within the
      // allocated memory 1 to the left doesn't cause problems
      m_yuvPad[i].bufs[0].buf--;
    }
  }
}

void DMVR::xCopyAndPad( const CodingUnit& cu, PelUnitBuf& pcPad, RefPicList refId, bool forLuma)
{
  int width, height;
  Mv cMv;

  const Picture* refPic = cu.slice->getRefPic(refId, cu.refIdx[refId]);

  static constexpr int mvShift = MV_FRACTIONAL_BITS_INTERNAL;

  const int start = forLuma ? 0 : 1;
  const int end   = forLuma ? 1 : MAX_NUM_COMP;

  for (int compID = start; compID < end; compID++)
  {
    int filtersize = compID == COMP_Y ? NTAPS_LUMA : NTAPS_CHROMA;
    cMv            = cu.mv[refId][0];
    width          = pcPad.bufs[compID].width;
    height         = pcPad.bufs[compID].height;

    int mvshiftTemp = mvShift + getComponentScaleX((ComponentID)compID, cu.chromaFormat);

    width  += filtersize - 1;
    height += filtersize - 1;
    cMv    += Mv(-(((filtersize >> 1) - 1) << mvshiftTemp), -(((filtersize >> 1) - 1) << mvshiftTemp));
    bool wrapRef = false;

    if (cu.cs->sps->wrapAroundEnabled)
    {
      wrapRef = wrapClipMv(cMv, cu.lumaPos(), cu.lumaSize(), *cu.cs);
    }
    else
    {
      clipMv(cMv, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
    }

    /* Pre-fetch similar to HEVC*/
    {
      CPelBuf refBuf      = wrapRef ? refPic->getRecoWrapBuf(ComponentID(compID)) : refPic->getRecoBuf(ComponentID(compID));
      Position Rec_offset = cu.blocks[compID].pos().offset(cMv.hor >> mvshiftTemp, cMv.ver >> mvshiftTemp);
      const Pel* refBufPtr = refBuf.bufAt(Rec_offset);

      PelBuf& dstBuf = pcPad.bufs[compID];

      const int leftTopFilterExt = ((filtersize >> 1) - 1);
      const int padOffset        = leftTopFilterExt * dstBuf.stride + leftTopFilterExt;
      const int padSize          = (DMVR_NUM_ITERATION) >> getComponentScaleX((ComponentID)compID, cu.chromaFormat);

      xFpPadDmvr( refBufPtr, refBuf.stride, dstBuf.buf - padOffset, dstBuf.stride, width, height, padSize );
    }
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

void DMVR::xFinalPaddedMCForDMVR( const CodingUnit& cu, PelUnitBuf* dstBuf, const PelUnitBuf *refBuf, const bool bioApplied, const Mv mergeMv[NUM_REF_PIC_LIST_01], const Mv& refMv )
{
  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;
  Mv mv[2];
  mv[L0] = mergeMv[L0] + refMv; mv[L0].clipToStorageBitDepth();
  mv[L1] = mergeMv[L1] - refMv; mv[L1].clipToStorageBitDepth();

  for (int k = 0; k < NUM_REF_PIC_LIST_01; k++)
  {
    RefPicList refId = (RefPicList)k;
    const Mv& cMv = mv[refId];
    Mv cMvClipped( cMv );
    clipMv(cMvClipped, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
    const Picture* refPic = cu.slice->getRefPic(refId, cu.refIdx[refId]);
    const Mv& startMv = mergeMv[refId];
    for (int compID = 0; compID < getNumberValidComponents(cu.chromaFormat); compID++)
    {
      int mvshiftTemp = mvShift + getComponentScaleX((ComponentID)compID, cu.chromaFormat);
      int deltaIntMvX = (cMv.hor >> mvshiftTemp) - (startMv.hor >> mvshiftTemp);
      int deltaIntMvY = (cMv.ver >> mvshiftTemp) - (startMv.ver >> mvshiftTemp);

      CHECK((abs(deltaIntMvX) > DMVR_NUM_ITERATION) || (abs(deltaIntMvY) > DMVR_NUM_ITERATION), "not expected DMVR movement");

      if (deltaIntMvX || deltaIntMvY)
      {
        const PelBuf& srcBuf = refBuf[refId].bufs[compID];
        int offset = (deltaIntMvY)*srcBuf.stride + (deltaIntMvX);

        xPredInterBlk((ComponentID)compID, cu, nullptr, cMvClipped, dstBuf[refId], true, cu.cs->slice->clpRngs.comp[compID],
          bioApplied, false, refId, 0, 0, 0, srcBuf.buf + offset, srcBuf.stride);
      }
      else
      {
        xPredInterBlk((ComponentID)compID, cu, refPic, cMvClipped, dstBuf[refId], true, cu.cs->slice->clpRngs.comp[compID],
          bioApplied, false, refId, 0, 0, 0);
      }
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

void DMVR::xProcessDMVR( const CodingUnit& cu, PelUnitBuf& pcYuvDst, const ClpRngs &clpRngs, const bool bioApplied )
{
  PROFILER_SCOPE_AND_STAGE_EXT( 1, _TPROF, P_INTER_MRG_DMVR, cu.cs, CH_L );
  int iterationCount = 1;
  /*Always High Precision*/
  const int mvShift  = MV_FRACTIONAL_BITS_INTERNAL;
  const int mvShiftC = mvShift + getChannelTypeScaleX(CH_C, cu.chromaFormat);

  /*use merge MV as starting MV*/
  const Mv mergeMv[] = { cu.mv[REF_PIC_LIST_0][0], cu.mv[REF_PIC_LIST_1][0] };


  const int dy = std::min<int>(cu.lumaSize().height, DMVR_SUBCU_SIZE);
  const int dx = std::min<int>(cu.lumaSize().width,  DMVR_SUBCU_SIZE);

  const Position& puPos = cu.lumaPos();

  bool bioAppliedType[MAX_NUM_SUBCU_DMVR];

  // Do refinement search
  {
    const int bilinearBufStride = (cu.Y().width + (2 * DMVR_NUM_ITERATION));
    const int padSize = DMVR_NUM_ITERATION << 1;
    const int dstOffset = -( DMVR_NUM_ITERATION * bilinearBufStride + DMVR_NUM_ITERATION );

    /*use merge MV as starting MV*/
    Mv mergeMVL0 = cu.mv[L0][0];
    Mv mergeMVL1 = cu.mv[L1][0];

    /*Clip the starting MVs*/
    clipMv(mergeMVL0, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);
    clipMv(mergeMVL1, cu.lumaPos(), cu.lumaSize(), *cu.cs->pcv);

    /*L0 MC for refinement*/
    {
      const Picture* refPic = cu.slice->getRefPic(L0, cu.refIdx[L0]);

      PelUnitBuf yuvTmp = PelUnitBuf(cu.chromaFormat, PelBuf(m_yuvTmp[L0].getBuf(COMP_Y).buf + dstOffset, bilinearBufStride, cu.lwidth() + padSize, cu.lheight() + padSize));

      mergeMVL0.hor -= (DMVR_NUM_ITERATION << MV_FRACTIONAL_BITS_INTERNAL);
      mergeMVL0.ver -= (DMVR_NUM_ITERATION << MV_FRACTIONAL_BITS_INTERNAL);

      xPredInterBlk(COMP_Y, cu, refPic, mergeMVL0, yuvTmp, true, clpRngs.comp[COMP_Y], false, false, L0, cu.lwidth() + padSize, cu.lheight() + padSize, true);
    }

    /*L1 MC for refinement*/
    {
      const Picture* refPic = cu.slice->getRefPic(L1, cu.refIdx[L1]);

      PelUnitBuf yuvTmp = PelUnitBuf(cu.chromaFormat, PelBuf(m_yuvTmp[L1].getBuf(COMP_Y).buf + dstOffset, bilinearBufStride, cu.lwidth() + padSize, cu.lheight() + padSize));

      mergeMVL1.hor -= (DMVR_NUM_ITERATION << MV_FRACTIONAL_BITS_INTERNAL);
      mergeMVL1.ver -= (DMVR_NUM_ITERATION << MV_FRACTIONAL_BITS_INTERNAL);

      xPredInterBlk(COMP_Y, cu, refPic, mergeMVL1, yuvTmp, true, clpRngs.comp[COMP_Y], false, false, L1, cu.lwidth() + padSize, cu.lheight() + padSize, true);
    }

    // point mc buffer to center point to avoid multiplication to reach each iteration to the beginning
    const Pel* biLinearPredL0 = m_yuvTmp[0].getBuf( COMP_Y ).buf;
    const Pel* biLinearPredL1 = m_yuvTmp[1].getBuf( COMP_Y ).buf;
    const int bioEnabledThres = 2 * dy * dx;
    const int bd = cu.cs->slice->clpRngs.comp[COMP_Y].bd;

    DistParam distParam = m_pcRdCost->setDistParam( nullptr, nullptr, bilinearBufStride, bilinearBufStride, bd, COMP_Y, dx, dy, 1, true );

    int num = 0;
    int yStart = 0;
    uint64_t sadArray[((2 * DMVR_NUM_ITERATION) + 1) * ((2 * DMVR_NUM_ITERATION) + 1)];

    for (int y = puPos.y; y < (puPos.y + cu.lumaSize().height); y = y + dy, yStart = yStart + dy)
    {
      for (int x = puPos.x, xStart = 0; x < (puPos.x + cu.lumaSize().width); x = x + dx, xStart = xStart + dx)
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
        totalDeltaMV[0] = totalDeltaMV[0] * (1 << mvShift);
        totalDeltaMV[1] = totalDeltaMV[1] * (1 << mvShift);
        xDMVRSubPixelErrorSurface(notZeroCost, totalDeltaMV, deltaMV, pSADsArray);

        cu.mvdL0SubPu[num] = Mv(totalDeltaMV[0], totalDeltaMV[1]);

        num++;
      }
    }
  }

  // Final MC
  CodingUnit subCu = cu;
  subCu.UnitArea::operator=(UnitArea(cu.chromaFormat, Area(puPos.x, puPos.y, dx, dy)));
  PelUnitBuf subPredBuf = pcYuvDst.subBuf(UnitAreaRelative(cu, subCu));

  PelUnitBuf predBuf[NUM_REF_PIC_LIST_01];
  predBuf[L0] = m_yuvPred[L0].getCompactBuf( subCu );
  predBuf[L1] = m_yuvPred[L1].getCompactBuf( subCu );
  /* For padding */
  PelUnitBuf padBuf[NUM_REF_PIC_LIST_01];
  padBuf[L0] = m_yuvPad[L0].getBufPart(subCu);
  padBuf[L1] = m_yuvPad[L1].getBufPart(subCu);

  int x = 0, y = 0;
  int xStart = 0, yStart = 0;
  int num = 0;
  const int scaleX = getComponentScaleX(COMP_Cb, cu.chromaFormat);
  const int scaleY = getComponentScaleY(COMP_Cb, cu.chromaFormat);

  const ptrdiff_t dstStride[MAX_NUM_COMP] = { pcYuvDst.bufs[COMP_Y].stride, cu.chromaFormat != CHROMA_400 ? pcYuvDst.bufs[COMP_Cb].stride : 0, cu.chromaFormat != CHROMA_400 ? pcYuvDst.bufs[COMP_Cr].stride : 0 };
  for (y = puPos.y; y < (puPos.y + cu.lumaSize().height); y = y + dy, yStart = yStart + dy)
  {
    for (x = puPos.x, xStart = 0; x < (puPos.x + cu.lumaSize().width); x = x + dx, xStart = xStart + dx)
    {
      new (&subCu) UnitArea(cu.chromaFormat, Area(x, y, dx, dy));

      Mv mv0 = mergeMv[REF_PIC_LIST_0] + cu.mvdL0SubPu[num]; mv0.clipToStorageBitDepth();
      Mv mv1 = mergeMv[REF_PIC_LIST_1] - cu.mvdL0SubPu[num]; mv1.clipToStorageBitDepth();

      bool padBufL0  = (mv0.hor >> mvShift)  != (mergeMv[0].hor >> mvShift)  || (mv0.ver >> mvShift)  != (mergeMv[0].ver >> mvShift);
      bool padBufL0C = (mv0.hor >> mvShiftC) != (mergeMv[0].hor >> mvShiftC) || (mv0.ver >> mvShiftC) != (mergeMv[0].ver >> mvShiftC);
        
      bool padBufL1  = (mv1.hor >> mvShift)  != (mergeMv[1].hor >> mvShift)  || (mv1.ver >> mvShift)  != (mergeMv[1].ver >> mvShift);
      bool padBufL1C = (mv1.hor >> mvShiftC) != (mergeMv[1].hor >> mvShiftC) || (mv1.ver >> mvShiftC) != (mergeMv[1].ver >> mvShiftC);

      padBufL0C &= cu.chromaFormat != CHROMA_400;
      padBufL1C &= cu.chromaFormat != CHROMA_400;

      if (padBufL0)  xCopyAndPad(subCu, padBuf[L0], L0, true);
      if (padBufL0C) xCopyAndPad(subCu, padBuf[L0], L0, false);
      if (padBufL1)  xCopyAndPad(subCu, padBuf[L1], L1, true);
      if (padBufL1C) xCopyAndPad(subCu, padBuf[L1], L1, false);

      xFinalPaddedMCForDMVR( subCu, predBuf, padBuf, bioAppliedType[num], mergeMv, cu.mvdL0SubPu[num] );

      subPredBuf.bufs[COMP_Y].buf  = pcYuvDst.bufs[COMP_Y].buf + xStart + yStart * dstStride[COMP_Y];
      if( cu.chromaFormat != CHROMA_400 )
      {
        subPredBuf.bufs[COMP_Cb].buf = pcYuvDst.bufs[COMP_Cb].buf + (xStart >> scaleX) + ((yStart >> scaleY) * dstStride[COMP_Cb]);
        subPredBuf.bufs[COMP_Cr].buf = pcYuvDst.bufs[COMP_Cr].buf + (xStart >> scaleX) + ((yStart >> scaleY) * dstStride[COMP_Cr]);
      }

      xWeightedAverage(subCu, predBuf[L0], predBuf[L1], subPredBuf, bioAppliedType[num] );
      num++;
    }
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

void InterPredInterpolation::xPredAffineBlk(const ComponentID compID, const CodingUnit& cu, const Picture* refPic, const Mv* _mv, PelUnitBuf& dstPic, const bool bi, const ClpRng& clpRng, const RefPicList refPicList)
{
  const ChromaFormat chFmt = cu.chromaFormat;
  int iScaleX = getComponentScaleX(compID, chFmt);
  int iScaleY = getComponentScaleY(compID, chFmt);

  Mv mvLT = _mv[0];
  Mv mvRT = _mv[1];
  Mv mvLB = _mv[2];

  // get affine sub-block width and height
  const int width = cu.Y().width;
  const int height = cu.Y().height;
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

  iDMvHorX = (mvRT - mvLT).hor * (1 << (iBit - Log2(cxWidth)));
  iDMvHorY = (mvRT - mvLT).ver * (1 <<(iBit - Log2(cxWidth)));
  if (cu.affineType == AFFINEMODEL_6PARAM)
  {
    iDMvVerX = (mvLB - mvLT).hor * (1 <<(iBit - Log2(cxHeight)));
    iDMvVerY = (mvLB - mvLT).ver * (1 <<(iBit - Log2(cxHeight)));
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  int iMvScaleHor = mvLT.hor * (1 << iBit);
  int iMvScaleVer = mvLT.ver * (1 << iBit);
  const PPS &pps = *cu.cs->pps;
  const SPS &sps = *cu.cs->sps;
  const int iMvShift = 4;
  const int iOffset = 8;
  const int iHorMax = (pps.picWidthInLumaSamples + iOffset - cu.Y().x - 1) << iMvShift;
  const int iHorMin = (-(int)cu.cs->pcv->maxCUSize - iOffset - (int)cu.Y().x + 1) * (1 << iMvShift);
  const int iVerMax = (pps.picHeightInLumaSamples + iOffset - cu.Y().y - 1) << iMvShift;
  const int iVerMin = (-(int)cu.cs->pcv->maxCUSize - iOffset - (int)cu.Y().y + 1) * (1 << iMvShift);

  const int shift = iBit - 4 + MV_FRACTIONAL_BITS_INTERNAL;
  bool      wrapRef = false;
  const bool subblkMVSpreadOverLimit = isSubblockVectorSpreadOverLimit(iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY, cu.interDir);

  bool enablePROF = sps.PROF && (!m_skipPROF) && (compID == COMP_Y);
  enablePROF &= (!cu.cs->picHeader->disProfFlag);
  enablePROF &= !((cu.affineType == AFFINEMODEL_6PARAM && _mv[0] == _mv[1] && _mv[0] == _mv[2]) || (cu.affineType == AFFINEMODEL_4PARAM && _mv[0] == _mv[1]));
  enablePROF &= !subblkMVSpreadOverLimit;
  const int profThres = 1 << (iBit + (m_isBi ? 1 : 0));
  enablePROF &= !m_encOnly || cu.slice->checkLDC || iDMvHorX > profThres || iDMvHorY > profThres || iDMvVerX > profThres || iDMvVerY > profThres || iDMvHorX < -profThres || iDMvHorY < -profThres || iDMvVerX < -profThres || iDMvVerY < -profThres;
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
    int quadHorX = 4 * iDMvHorX ;
    int quadHorY = 4 * iDMvHorY ;
    int quadVerX = 4 * iDMvVerX ;
    int quadVerY = 4 * iDMvVerY ;

    dMvH[0] = ((iDMvHorX + iDMvVerX) * 2) - ((quadHorX + quadVerX)  * 2);
    dMvV[0] = ((iDMvHorY + iDMvVerY) * 2) - ((quadHorY + quadVerY)  * 2);

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
  if ((cu.mcControl > 3) && (compID == COMP_Cb) && cu.chromaFormat != CHROMA_444)
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
    dMvHorXLuma = (mvRT - mvLT).hor * (1 << (iBit - floorLog2(cxWidthLuma)));
    dMvHorYLuma = (mvRT - mvLT).ver * (1 <<  (iBit - floorLog2(cxWidthLuma)));
    if (cu.affineType == AFFINEMODEL_6PARAM)
    {
      dMvVerXLuma = (mvLB - mvLT).hor * (1 << (iBit - floorLog2(cxHeightLuma)));
      dMvVerYLuma = (mvLB - mvLT).ver * (1 << (iBit - floorLog2(cxHeightLuma)));
    }
    else
    {
      dMvVerXLuma = -dMvHorYLuma;
      dMvVerYLuma = dMvHorXLuma;
    }

    const bool subblkMVSpreadOverLimitLuma = isSubblockVectorSpreadOverLimit(dMvHorXLuma, dMvHorYLuma, dMvVerXLuma, dMvVerYLuma, cu.interDir);

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

  const int puX = cu.blocks[compID].x;
  const int puY = cu.blocks[compID].y;

  for (int h = 0; h < cxHeight; h += blockHeight)
  {
    for (int w = 0; w < cxWidth; w += blockWidth)
    {
      int iMvScaleTmpHor, iMvScaleTmpVer;
      if (compID == COMP_Y || cu.chromaFormat == CHROMA_444)
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
        if( cu.cs->sps->wrapAroundEnabled )
        {
          m_storedMv[h / AFFINE_MIN_BLOCK_SIZE * MVBUFFER_SIZE + w / AFFINE_MIN_BLOCK_SIZE].set(iMvScaleTmpHor, iMvScaleTmpVer);
          Mv tmpMv(iMvScaleTmpHor, iMvScaleTmpVer);
          wrapRef = wrapClipMv(tmpMv, Position(cu.Y().x + w, cu.Y().y + h), Size(blockWidth, blockHeight), *cu.cs);
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
        if (cu.cs->sps->wrapAroundEnabled)
        {
          wrapRef = wrapClipMv(curMv, Position(cu.Y().x + (w << iScaleX), cu.Y().y + (h << iScaleY)), Size(blockWidth << iScaleX, blockHeight << iScaleY), *cu.cs);
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

        xFpProfGradFilter(dstExtBuf.buf, dstExtBuf.stride, blockWidth + 2, blockHeight + 2, gradXBuf.stride, gradXBuf.buf, gradYBuf.buf, clpRng.bd);

        const int shiftNum = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
        const Pel offset = (1 << (shiftNum - 1)) + IF_INTERNAL_OFFS;
        Pel* src = dstExtBuf.bufAt(PROF_BORDER_EXT_W, PROF_BORDER_EXT_H);
        Pel* gX = gradXBuf.bufAt(PROF_BORDER_EXT_W, PROF_BORDER_EXT_H);
        Pel* gY = gradYBuf.bufAt(PROF_BORDER_EXT_W, PROF_BORDER_EXT_H);

        Pel*  dstY = dstBuf.bufAt(w, h);

        xFpApplyPROF(dstY, dstBuf.stride, src, dstExtBuf.stride, blockWidth, blockHeight, gX, gY, gradXBuf.stride, dMvScaleHor, dMvScaleVer, blockWidth, bi, shiftNum, offset, clpRng);
      }
    }
  }

}

void InterPrediction::xFillIBCBuffer(CodingUnit& cu)
{
  for (auto& currPU : CU::traverseTUs(cu))
  {
    for (const CompArea& area : currPU.blocks)
    {
      if (!area.valid())
      {
        continue;
      }
      const unsigned int lcuWidth = cu.cs->slice->sps->CTUSize;
      const int shiftSampleHor = getComponentScaleX(area.compID, cu.chromaFormat);
      const int shiftSampleVer = getComponentScaleY(area.compID, cu.chromaFormat);
      const int ctuSizeLog2Ver = floorLog2(lcuWidth) - shiftSampleVer;
      const int pux = area.x & ((m_IBCBufferWidth >> shiftSampleHor) - 1);
      const int puy = area.y & ((1 << ctuSizeLog2Ver) - 1);
      const CompArea dstArea = CompArea(area.compID, cu.chromaFormat, Position(pux, puy), Size(area.width, area.height));
      CPelBuf srcBuf = cu.cs->getRecoBuf(area);
      PelBuf dstBuf = m_IBCBuffer.getBuf(dstArea);

      dstBuf.copyFrom(srcBuf);
    }
  }
}

void InterPrediction::xIntraBlockCopyIBC(CodingUnit& cu, PelUnitBuf& predBuf, const ComponentID compID)
{
  const unsigned int lcuWidth = cu.cs->slice->sps->CTUSize;
  const int shiftSampleHor = getComponentScaleX(compID, cu.chromaFormat);
  const int shiftSampleVer = getComponentScaleY(compID, cu.chromaFormat);
  const int ctuSizeLog2Ver = floorLog2(lcuWidth) - shiftSampleVer;
  cu.bv = cu.mv[REF_PIC_LIST_0][0];
  cu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
  int refx, refy;
  if (compID == COMP_Y)
  {
    refx = cu.Y().x + cu.bv.hor;
    refy = cu.Y().y + cu.bv.ver;
  }
  else
  {//Cb or Cr
    refx = cu.Cb().x + (cu.bv.hor >> shiftSampleHor);
    refy = cu.Cb().y + (cu.bv.ver >> shiftSampleVer);
  }
  refx &= ((m_IBCBufferWidth >> shiftSampleHor) - 1);
  refy &= ((1 << ctuSizeLog2Ver) - 1);

  if (refx + predBuf.bufs[compID].width <= (m_IBCBufferWidth >> shiftSampleHor))
  {
    const CompArea srcArea = CompArea(compID, cu.chromaFormat, Position(refx, refy), Size(predBuf.bufs[compID].width, predBuf.bufs[compID].height));
    const CPelBuf refBuf = m_IBCBuffer.getBuf(srcArea);
    predBuf.bufs[compID].copyFrom(refBuf);
  }
  else
  {//wrap around
    int width = (m_IBCBufferWidth >> shiftSampleHor) - refx;
    CompArea srcArea = CompArea(compID, cu.chromaFormat, Position(refx, refy), Size(width, predBuf.bufs[compID].height));
    CPelBuf srcBuf = m_IBCBuffer.getBuf(srcArea);
    PelBuf dstBuf = PelBuf(predBuf.bufs[compID].bufAt(Position(0, 0)), predBuf.bufs[compID].stride, Size(width, predBuf.bufs[compID].height));
    dstBuf.copyFrom(srcBuf);

    width = refx + predBuf.bufs[compID].width - (m_IBCBufferWidth >> shiftSampleHor);
    srcArea = CompArea(compID, cu.chromaFormat, Position(0, refy), Size(width, predBuf.bufs[compID].height));
    srcBuf = m_IBCBuffer.getBuf(srcArea);
    dstBuf = PelBuf(predBuf.bufs[compID].bufAt(Position((m_IBCBufferWidth >> shiftSampleHor) - refx, 0)), predBuf.bufs[compID].stride, Size(width, predBuf.bufs[compID].height));
    dstBuf.copyFrom(srcBuf);
  }
}

void InterPrediction::resetIBCBuffer(const ChromaFormat chromaFormatIDC, const int ctuSize)
{
  const UnitArea area = UnitArea(chromaFormatIDC, Area(0, 0, m_IBCBufferWidth, ctuSize));
  m_IBCBuffer.getBuf(area).fill(-1);
}

void InterPrediction::resetVPDUforIBC(const ChromaFormat chromaFormatIDC, const int ctuSize, const int vSize, const int xPos, const int yPos)
{
  const UnitArea area = UnitArea(chromaFormatIDC, Area(xPos & (m_IBCBufferWidth - 1), yPos & (ctuSize - 1), vSize, vSize));
  m_IBCBuffer.getBuf(area).fill(-1);
}
bool InterPrediction::isLumaBvValidIBC(const int ctuSize, const int xCb, const int yCb, const int width, const int height, const int xBv, const int yBv)
{
  if (((yCb + yBv) & (ctuSize - 1)) + height > ctuSize)
  {
    return false;
  }
  int refTLx = xCb + xBv;
  int refTLy = (yCb + yBv) & (ctuSize - 1);
  PelBuf buf = m_IBCBuffer.Y();
  for (int x = 0; x < width; x += 4)
  {
    for (int y = 0; y < height; y += 4)
    {
      if (buf.at((x + refTLx) & (m_IBCBufferWidth - 1), y + refTLy) == -1) return false;
      if (buf.at((x + 3 + refTLx) & (m_IBCBufferWidth - 1), y + refTLy) == -1) return false;
      if (buf.at((x + refTLx) & (m_IBCBufferWidth - 1), y + 3 + refTLy) == -1) return false;
      if (buf.at((x + 3 + refTLx) & (m_IBCBufferWidth - 1), y + 3 + refTLy) == -1) return false;
    }
  }
  return true;
}

} // namespace vvenc

//! \}


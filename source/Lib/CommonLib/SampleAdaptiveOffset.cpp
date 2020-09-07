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


/** \file     SampleAdaptiveOffset.cpp
    \brief    sample adaptive offset class
*/

#include "SampleAdaptiveOffset.h"
#include "UnitTools.h"
#include "UnitPartitioner.h"
#include "CodingStructure.h"
#include "dtrace_codingstruct.h"
#include "dtrace_buffer.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>

//! \ingroup CommonLib
//! \{

namespace vvenc {

void offsetBlock_core(const int channelBitDepth, const ClpRng& clpRng, int typeIdx, int* offset, int startIdx, 
                      const Pel* srcBlk, Pel* resBlk, ptrdiff_t srcStride, ptrdiff_t resStride, int width, int height, 
                      uint8_t availMask, std::vector<int8_t> &signLineBuf1, std::vector<int8_t> &signLineBuf2)
{
  int x, y, startX, startY, endX, endY, edgeType;
  int firstLineStartX, firstLineEndX, lastLineStartX, lastLineEndX;
  int8_t signLeft, signRight, signDown;

  const Pel* srcLine = srcBlk;
  Pel* resLine = resBlk;

  switch (typeIdx)
  {
  case SAO_TYPE_EO_0:
  {
    offset += 2;
    startX = availMask&LeftAvail ? 0 : 1;
    endX = availMask&RightAvail ? width : (width - 1);
    for (y = 0; y < height; y++)
    {
      signLeft = (int8_t)sgn(srcLine[startX] - srcLine[startX - 1]);
      for (x = startX; x < endX; x++)
      {
        signRight = (int8_t)sgn(srcLine[x] - srcLine[x + 1]);
        edgeType = signRight + signLeft;
        signLeft = -signRight;

        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
      }
      srcLine += srcStride;
      resLine += resStride;
    }

  }
  break;
  case SAO_TYPE_EO_90:
  {
    offset += 2;
    int8_t *signUpLine = &signLineBuf1[0];

    startY = availMask&AboveAvail ? 0 : 1;
    endY = availMask&BelowAvail ? height : height - 1;
    if (!(availMask&AboveAvail))
    {
      srcLine += srcStride;
      resLine += resStride;
    }

    const Pel* srcLineAbove = srcLine - srcStride;
    for (x = 0; x < width; x++)
    {
      signUpLine[x] = (int8_t)sgn(srcLine[x] - srcLineAbove[x]);
    }

    const Pel* srcLineBelow;
    for (y = startY; y < endY; y++)
    {
      srcLineBelow = srcLine + srcStride;

      for (x = 0; x < width; x++)
      {
        signDown = (int8_t)sgn(srcLine[x] - srcLineBelow[x]);
        edgeType = signDown + signUpLine[x];
        signUpLine[x] = -signDown;

        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
      }
      srcLine += srcStride;
      resLine += resStride;
    }

  }
  break;
  case SAO_TYPE_EO_135:
  {
    offset += 2;
    int8_t *signUpLine, *signDownLine, *signTmpLine;

    signUpLine = &signLineBuf1[0];
    signDownLine = &signLineBuf2[0];

    startX = availMask&LeftAvail ? 0 : 1;
    endX = availMask&RightAvail ? width : (width - 1);

    //prepare 2nd line's upper sign
    const Pel* srcLineBelow = srcLine + srcStride;
    for (x = startX; x < endX + 1; x++)
    {
      signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x - 1]);
    }

    //1st line
    const Pel* srcLineAbove = srcLine - srcStride;
    firstLineStartX = availMask&AboveLeftAvail ? 0 : 1;
    firstLineEndX = availMask&AboveAvail ? endX : 1;
    for (x = firstLineStartX; x < firstLineEndX; x++)
    {
      edgeType = sgn(srcLine[x] - srcLineAbove[x - 1]) - signUpLine[x + 1];

      resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
    }
    srcLine += srcStride;
    resLine += resStride;


    //middle lines
    for (y = 1; y < height - 1; y++)
    {
      srcLineBelow = srcLine + srcStride;

      for (x = startX; x < endX; x++)
      {
        signDown = (int8_t)sgn(srcLine[x] - srcLineBelow[x + 1]);
        edgeType = signDown + signUpLine[x];
        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);

        signDownLine[x + 1] = -signDown;
      }
      signDownLine[startX] = (int8_t)sgn(srcLineBelow[startX] - srcLine[startX - 1]);

      signTmpLine = signUpLine;
      signUpLine = signDownLine;
      signDownLine = signTmpLine;

      srcLine += srcStride;
      resLine += resStride;
    }

    //last line
    srcLineBelow = srcLine + srcStride;
    lastLineStartX = availMask&BelowAvail ? startX : (width - 1);
    lastLineEndX = availMask&BelowRightAvail ? width : (width - 1);
    for (x = lastLineStartX; x < lastLineEndX; x++)
    {
      edgeType = sgn(srcLine[x] - srcLineBelow[x + 1]) + signUpLine[x];
      resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);

    }
  }
  break;
  case SAO_TYPE_EO_45:
  {
    offset += 2;
    int8_t *signUpLine = &signLineBuf1[1];

    startX = availMask&LeftAvail ? 0 : 1;
    endX = availMask&RightAvail ? width : (width - 1);

    //prepare 2nd line upper sign
    const Pel* srcLineBelow = srcLine + srcStride;
    for (x = startX - 1; x < endX; x++)
    {
      signUpLine[x] = (int8_t)sgn(srcLineBelow[x] - srcLine[x + 1]);
    }


    //first line
    const Pel* srcLineAbove = srcLine - srcStride;
    firstLineStartX = availMask&AboveAvail ? startX : (width - 1);
    firstLineEndX = availMask&AboveRightAvail ? width : (width - 1);
    for (x = firstLineStartX; x < firstLineEndX; x++)
    {
      edgeType = sgn(srcLine[x] - srcLineAbove[x + 1]) - signUpLine[x - 1];
      resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
    }
    srcLine += srcStride;
    resLine += resStride;

    //middle lines
    for (y = 1; y < height - 1; y++)
    {
      srcLineBelow = srcLine + srcStride;

      for (x = startX; x < endX; x++)
      {
        signDown = (int8_t)sgn(srcLine[x] - srcLineBelow[x - 1]);
        edgeType = signDown + signUpLine[x];
        resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);
        signUpLine[x - 1] = -signDown;
      }
      signUpLine[endX - 1] = (int8_t)sgn(srcLineBelow[endX - 1] - srcLine[endX]);
      srcLine += srcStride;
      resLine += resStride;
    }

    //last line
    srcLineBelow = srcLine + srcStride;
    lastLineStartX = availMask&BelowLeftAvail ? 0 : 1;
    lastLineEndX = availMask&BelowAvail ? endX : 1;
    for (x = lastLineStartX; x < lastLineEndX; x++)
    {
      edgeType = sgn(srcLine[x] - srcLineBelow[x - 1]) + signUpLine[x];
      resLine[x] = ClipPel<int>(srcLine[x] + offset[edgeType], clpRng);

    }
  }
  break;
  case SAO_TYPE_BO:
  {
    const int shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
    for (y = 0; y < height; y++)
    {
      for (x = 0; x < width; x++)
      {
        resLine[x] = ClipPel<int>(srcLine[x] + offset[srcLine[x] >> shiftBits], clpRng);
      }
      srcLine += srcStride;
      resLine += resStride;
    }
  }
  break;
  default:
  {
    THROW("Not a supported SAO types\n");
  }
  }
}

void SAOOffset::reset()
{
  modeIdc = SAO_MODE_OFF;
  typeIdc = -1;
  typeAuxInfo = -1;
  ::memset(offset, 0, sizeof(int)* MAX_NUM_SAO_CLASSES);
}

void SAOBlkParam::reset()
{
  for(int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    SAOOffsets[compIdx].reset();
  }
}



SampleAdaptiveOffset::SampleAdaptiveOffset()
{
}


SampleAdaptiveOffset::~SampleAdaptiveOffset()
{
  m_signLineBuf1.clear();
  m_signLineBuf2.clear();
}

void SampleAdaptiveOffset::init( ChromaFormat format, uint32_t maxCUWidth, uint32_t maxCUHeight, uint32_t lumaBitShift, uint32_t chromaBitShift )
{
  offsetBlock = offsetBlock_core;
#if   ENABLE_SIMD_OPT_SAO
  initSampleAdaptiveOffsetX86();
#endif

  //bit-depth related
  for(int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    m_offsetStepLog2  [compIdx] = isLuma(ComponentID(compIdx))? lumaBitShift : chromaBitShift;
  }
  m_numberOfComponents = getNumberValidComponents(format);

  size_t lineBufferSize = std::max( maxCUWidth, maxCUHeight ) + 1;
  if( m_signLineBuf1.size() < lineBufferSize )
  {
    m_signLineBuf1.resize( lineBufferSize );
    m_signLineBuf2.resize( lineBufferSize );
  }
}

void SampleAdaptiveOffset::invertQuantOffsets(ComponentID compIdx, int typeIdc, int typeAuxInfo, int* dstOffsets, int* srcOffsets)
{
  int codedOffset[MAX_NUM_SAO_CLASSES];

  ::memcpy(codedOffset, srcOffsets, sizeof(int)*MAX_NUM_SAO_CLASSES);
  ::memset(dstOffsets, 0, sizeof(int)*MAX_NUM_SAO_CLASSES);

  if(typeIdc == SAO_TYPE_START_BO)
  {
    for(int i=0; i< 4; i++)
    {
      dstOffsets[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES] = codedOffset[(typeAuxInfo+ i)%NUM_SAO_BO_CLASSES]*(1<<m_offsetStepLog2[compIdx]);
    }
  }
  else //EO
  {
    for(int i=0; i< NUM_SAO_EO_CLASSES; i++)
    {
      dstOffsets[i] = codedOffset[i] *(1<<m_offsetStepLog2[compIdx]);
    }
    CHECK(dstOffsets[SAO_CLASS_EO_PLAIN] != 0, "EO offset is not '0'"); //keep EO plain offset as zero
  }

}

int SampleAdaptiveOffset::getMergeList(CodingStructure& cs, int ctuRsAddr, SAOBlkParam* blkParams, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{
  const PreCalcValues& pcv = *cs.pcv;

  int ctuX = ctuRsAddr % pcv.widthInCtus;
  int ctuY = ctuRsAddr / pcv.widthInCtus;
  const CodingUnit& cu = *cs.getCU(Position(ctuX*pcv.maxCUSize, ctuY*pcv.maxCUSize), CH_L, TREE_D);
  int mergedCTUPos;
  int numValidMergeCandidates = 0;

  for(int mergeType=0; mergeType< NUM_SAO_MERGE_TYPES; mergeType++)
  {
    SAOBlkParam* mergeCandidate = NULL;

    switch(mergeType)
    {
    case SAO_MERGE_ABOVE:
      {
        if(ctuY > 0)
        {
          mergedCTUPos = ctuRsAddr- pcv.widthInCtus;
          if(cs.getCURestricted(Position(ctuX*pcv.maxCUSize, (ctuY-1)*pcv.maxCUSize), cu, cu.chType))
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);
          }
        }
      }
      break;
    case SAO_MERGE_LEFT:
      {
        if(ctuX > 0)
        {
          mergedCTUPos = ctuRsAddr- 1;
          if(cs.getCURestricted(Position((ctuX-1)*pcv.maxCUSize, ctuY*pcv.maxCUSize), cu, cu.chType))
          {
            mergeCandidate = &(blkParams[mergedCTUPos]);
          }
        }
      }
      break;
    default:
      {
        THROW("not a supported merge type");
      }
    }

    mergeList[mergeType]=mergeCandidate;
    if (mergeCandidate != NULL)
    {
      numValidMergeCandidates++;
    }
  }

  return numValidMergeCandidates;
}


void SampleAdaptiveOffset::reconstructBlkSAOParam(SAOBlkParam& recParam, SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES])
{
  const int numberOfComponents = m_numberOfComponents;
  for(int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID component = ComponentID(compIdx);
    SAOOffset& offsetParam = recParam[component];

    if(offsetParam.modeIdc == SAO_MODE_OFF)
    {
      continue;
    }

    switch(offsetParam.modeIdc)
    {
    case SAO_MODE_NEW:
      {
        invertQuantOffsets(component, offsetParam.typeIdc, offsetParam.typeAuxInfo, offsetParam.offset, offsetParam.offset);
      }
      break;
    case SAO_MODE_MERGE:
      {
        SAOBlkParam* mergeTarget = mergeList[offsetParam.typeIdc];
        CHECK(mergeTarget == NULL, "Merge target does not exist");

        offsetParam = (*mergeTarget)[component];
      }
      break;
    default:
      {
        THROW("Not a supported mode");
      }
    }
  }
}

void SampleAdaptiveOffset::xReconstructBlkSAOParams(CodingStructure& cs, SAOBlkParam* saoBlkParams)
{
  for(uint32_t compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++)
  {
    m_picSAOEnabled[compIdx] = false;
  }

  const uint32_t numberOfComponents = getNumberValidComponents(cs.pcv->chrFormat);

  for(int ctuRsAddr=0; ctuRsAddr< cs.pcv->sizeInCtus; ctuRsAddr++)
  {
    SAOBlkParam* mergeList[NUM_SAO_MERGE_TYPES] = { NULL };
    getMergeList(cs, ctuRsAddr, saoBlkParams, mergeList);

    reconstructBlkSAOParam(saoBlkParams[ctuRsAddr], mergeList);

    for(uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
    {
      if(saoBlkParams[ctuRsAddr][compIdx].modeIdc != SAO_MODE_OFF)
      {
        m_picSAOEnabled[compIdx] = true;
      }
    }
  }
}

void SampleAdaptiveOffset::offsetCTU( const UnitArea& area, const CPelUnitBuf& src, PelUnitBuf& res, SAOBlkParam& saoblkParam, CodingStructure& cs)
{
  const uint32_t numberOfComponents = getNumberValidComponents( area.chromaFormat );
  bool bAllOff=true;
  for( uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (saoblkParam[compIdx].modeIdc != SAO_MODE_OFF)
    {
      bAllOff=false;
    }
  }
  if (bAllOff)
  {
    return;
  }

  uint8_t availMask;
  //block boundary availability
  deriveLoopFilterBoundaryAvailibility(cs, area.Y(), availMask);

  const size_t lineBufferSize = area.Y().width + 1;
  if (m_signLineBuf1.size() < lineBufferSize)
  {
    m_signLineBuf1.resize(lineBufferSize);
    m_signLineBuf2.resize(lineBufferSize);
  }

  for(int compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    const ComponentID compID = ComponentID(compIdx);
    const CompArea& compArea = area.block(compID);
    SAOOffset& ctbOffset     = saoblkParam[compIdx];

    if(ctbOffset.modeIdc != SAO_MODE_OFF)
    {
      int  srcStride    = src.get(compID).stride;
      const Pel* srcBlk = src.get(compID).bufAt(compArea);
      int  resStride    = res.get(compID).stride;
      Pel* resBlk       = res.get(compID).bufAt(compArea);

      offsetBlock( cs.sps->bitDepths[toChannelType(compID)],
                   cs.slice->clpRngs[compID],
                   ctbOffset.typeIdc, ctbOffset.offset, ctbOffset.typeAuxInfo
                  , srcBlk, resBlk, srcStride, resStride, compArea.width, compArea.height, availMask
                  , m_signLineBuf1, m_signLineBuf2 );
    }
  } //compIdx
}

void SampleAdaptiveOffset::SAOProcess( CodingStructure& cs, SAOBlkParam* saoBlkParams )
{
  CHECK(!saoBlkParams, "No parameters present");

  xReconstructBlkSAOParams(cs, saoBlkParams);

  const uint32_t numberOfComponents = getNumberValidComponents(cs.area.chromaFormat);
  bool bAllDisabled = true;
  for (uint32_t compIdx = 0; compIdx < numberOfComponents; compIdx++)
  {
    if (m_picSAOEnabled[compIdx])
    {
      bAllDisabled = false;
    }
  }
  if (bAllDisabled)
  {
    return;
  }

  const PreCalcValues& pcv = *cs.pcv;
  Picture& pic             = *cs.picture;
  PelUnitBuf recBuf        = pic.getRecoBuf();
  PelUnitBuf saoBuf        = pic.getSaoBuf();
  saoBuf.copyFrom( recBuf );

  int ctuRsAddr = 0;
  for( uint32_t yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUSize )
  {
    const uint32_t height = (yPos + pcv.maxCUSize > pcv.lumaHeight) ? (pcv.lumaHeight - yPos) : pcv.maxCUSize;
    for( uint32_t xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUSize )
    {
      const uint32_t width  = (xPos + pcv.maxCUSize  > pcv.lumaWidth)  ? (pcv.lumaWidth - xPos)  : pcv.maxCUSize;
      const UnitArea area( cs.area.chromaFormat, Area(xPos , yPos, width, height) );

      offsetCTU( area, saoBuf, recBuf, cs.picture->getSAO()[ctuRsAddr], cs);
      ctuRsAddr++;
    }
  }

  DTRACE_PIC_COMP(D_REC_CB_LUMA_SAO, cs, cs.getRecoBuf(), COMP_Y);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMP_Cb);
  DTRACE_PIC_COMP(D_REC_CB_CHROMA_SAO, cs, cs.getRecoBuf(), COMP_Cr);
}

void SampleAdaptiveOffset::deriveLoopFilterBoundaryAvailibility(CodingStructure& cs, const Position& pos, uint8_t& availMask ) const
{
  const int cuSize = cs.pcv->maxCUSize;
  const CodingUnit* cuCurr = cs.getCU(pos, CH_L, TREE_D);
  const CodingUnit* cuLeft = cs.getCU(pos.offset(-cuSize, 0), CH_L, TREE_D);
  const CodingUnit* cuRight = cs.getCU(pos.offset(cuSize, 0), CH_L, TREE_D);
  const CodingUnit* cuAbove = cs.getCU(pos.offset(0, -cuSize), CH_L, TREE_D);
  const CodingUnit* cuBelow = cs.getCU(pos.offset(0, cuSize), CH_L, TREE_D);
  const CodingUnit* cuAboveLeft = cs.getCU(pos.offset(-cuSize, -cuSize), CH_L, TREE_D);
  const CodingUnit* cuAboveRight = cs.getCU(pos.offset(cuSize, -cuSize), CH_L, TREE_D);
  const CodingUnit* cuBelowLeft = cs.getCU(pos.offset(-cuSize, cuSize), CH_L, TREE_D);
  const CodingUnit* cuBelowRight = cs.getCU(pos.offset(cuSize, cuSize), CH_L, TREE_D);
  availMask = 0;

  // check cross slice flags
  if( cs.pps->loopFilterAcrossSlicesEnabled )
  {
    availMask |= (cuLeft       != NULL) ? LeftAvail : 0;
    availMask |= (cuAbove      != NULL) ? AboveAvail : 0;
    availMask |= (cuRight      != NULL) ? RightAvail : 0;
    availMask |= (cuBelow      != NULL) ? BelowAvail : 0;
    availMask |= (cuAboveLeft  != NULL) ? AboveLeftAvail : 0;
    availMask |= (cuBelowRight != NULL) ? BelowRightAvail : 0;
    availMask |= (cuAboveRight != NULL) ? AboveRightAvail : 0;
    availMask |= (cuBelowLeft  != NULL) ? BelowLeftAvail : 0;
  }
  else
  {
    availMask |= ((cuLeft       != NULL) && CU::isSameSlice(*cuCurr, *cuLeft) ) ? LeftAvail : 0;
    availMask |= ((cuAbove      != NULL) && CU::isSameSlice(*cuCurr, *cuAbove) ) ? AboveAvail : 0;
    availMask |= ((cuRight      != NULL) && CU::isSameSlice(*cuCurr, *cuRight)) ? RightAvail : 0;
    availMask |= ((cuBelow      != NULL) && CU::isSameSlice(*cuCurr, *cuBelow) ) ? BelowAvail : 0;
    availMask |= ((cuAboveLeft  != NULL) && CU::isSameSlice(*cuCurr, *cuAboveLeft)) ?  AboveLeftAvail : 0;
    availMask |= ((cuBelowRight != NULL) && CU::isSameSlice(*cuCurr, *cuBelowRight) ) ? BelowRightAvail : 0;
    availMask |= ((cuAboveRight != NULL) && CU::isSameSlice(*cuCurr, *cuAboveRight) ) ? AboveRightAvail : 0;
    availMask |= ( (cuBelowLeft != NULL) && CU::isSameSlice(*cuCurr, *cuBelowLeft) ) ? BelowLeftAvail : 0;
  }

  // check cross tile flags
  if (!cs.pps->loopFilterAcrossTilesEnabled)
  {
    uint8_t availMaskTile = 0;
    availMaskTile |= (availMask&LeftAvail       && CU::isSameTile(*cuCurr, *cuLeft)) ? LeftAvail : 0;
    availMaskTile |= (availMask&AboveAvail      && CU::isSameTile(*cuCurr, *cuAbove)) ? AboveAvail : 0;
    availMaskTile |= (availMask&RightAvail      && CU::isSameTile(*cuCurr, *cuRight)) ? RightAvail : 0;
    availMaskTile |= (availMask&BelowAvail      && CU::isSameTile(*cuCurr, *cuBelow)) ? BelowAvail : 0;
    availMaskTile |= (availMask&AboveLeftAvail  && CU::isSameTile(*cuCurr, *cuAboveLeft)) ? AboveLeftAvail : 0;
    availMaskTile |= (availMask&AboveRightAvail &&CU::isSameTile(*cuCurr, *cuAboveRight)) ? AboveRightAvail : 0;
    availMaskTile |= (availMask&BelowLeftAvail  && CU::isSameTile(*cuCurr, *cuBelowLeft)) ? BelowLeftAvail : 0;
    availMaskTile |= (availMask&BelowRightAvail && CU::isSameTile(*cuCurr, *cuBelowRight)) ? BelowRightAvail : 0;
    availMask = availMaskTile;
  }

  if (!cs.pps->getSubPicFromCU(*cuCurr).loopFilterAcrossSubPicEnabled)
  {
    THROW("no support");
  }
}

} // namespace vvenc

//! \}


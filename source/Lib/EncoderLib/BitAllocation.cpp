/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2023, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/** \file     BitAllocation.cpp
\brief    Bit allocation class for QP adaptation and, possibly, rate control
*/

#include "BitAllocation.h"
#include "EncStage.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include <math.h>

#include "vvenc/vvencCfg.h"


//! \ingroup EncoderLib
//! \{

namespace vvenc {

// static functions

static inline int apprI3Log2 (const double d) // rounded 3*log2(d)
{
  return d < 1.5e-13 ? -128 : int (floor (3.0 * log (d) / log (2.0) + 0.5));
}

static inline int lumaDQPOffset (const uint32_t avgLumaValue, const uint32_t bitDepth)
{
  if (bitDepth > 16 || avgLumaValue >= (1u << bitDepth)) return 0;
#if 0
  // mapping for peak luminance of ca. 3*400 = 1200 nits
  return (2 - int ((9 * uint64_t (avgLumaValue * avgLumaValue)) >> uint64_t (2 * bitDepth)));
#else
  // mapping for peak luminance of ca. 2*400 =  800 nits
  return (1 - int ((6 * uint64_t (avgLumaValue * avgLumaValue)) >> uint64_t (2 * bitDepth)));
#endif
}

double filterAndCalculateAverageActivity (const Pel* pSrc, const int iSrcStride, const int height, const int width,
                                          const Pel* pSM1, const int iSM1Stride, const Pel* pSM2, const int iSM2Stride,
                                          uint32_t frameRate, const uint32_t bitDepth, const bool isUHD, unsigned* minVA = nullptr
#if USE_SP_ACT
  , unsigned* spaVA = nullptr
#endif
)
{
  double spatAct = 0.0, tempAct = 0.0;
  uint64_t saAct = 0;   // spatial absolute activity sum
  uint64_t taAct = 0;  // temporal absolute activity sum
  const Pel* pS0 = pSrc;

  if (pSrc == nullptr || iSrcStride <= 0) return 0.0;
  // force 1st-order delta if only prev. frame available
  if (pSM2 == nullptr || iSM2Stride <= 0) frameRate = 24;

  // skip first row as there may be a black border frame
  pSrc += iSrcStride;
  // center rows
  if (isUHD) // high-pass with downsampling
  {
    pSrc += iSrcStride;
    saAct = g_pelBufOP.AvgHighPassWithDownsampling (width, height, pSrc, iSrcStride);

    spatAct = double (saAct) / double ((width - 4) * (height - 4));
  }
  else // HD high-pass without downsampling
  {
    saAct = g_pelBufOP.AvgHighPass (width, height, pSrc, iSrcStride);

    spatAct = double (saAct) / double ((width - 2) * (height - 2));
  }

  if (minVA) // spatial pt scaled to 12 bit
  {
    *minVA = unsigned (0.5 + spatAct * double (bitDepth < 12 ? 1 << (12 - bitDepth) : 1));
  }
#if USE_SP_ACT
  if (spaVA) // spatial pt scaled to 12 bit
  {
    *spaVA = unsigned(0.5 + spatAct * double(bitDepth < 12 ? 1 << (12 - bitDepth) : 1));
  }
#endif

  // skip first row as there may be a black border frame
  pSrc = pS0 + iSrcStride;
  // center rows
  if (isUHD) // high-pass with downsampling
  {
    const int i2M1Stride = iSM1Stride * 2;

    CHECK (pSM1 == nullptr || iSM1Stride <= 0 || iSM1Stride < width, "Pel buffer pointer pSM1 must not be null!");

    pSrc += iSrcStride;
    pSM1 += i2M1Stride;
    if (frameRate <= 31) // 1st-order delta
    {
      taAct = g_pelBufOP.AvgHighPassWithDownsamplingDiff1st (width, height, pSrc, pSM1, iSrcStride, iSM1Stride);
    }
    else // 2nd-order delta (diff of diffs)
    {
      const int i2M2Stride = iSM2Stride * 2;

      CHECK (pSM2 == nullptr || iSM2Stride <= 0 || iSM2Stride < width, "Pel buffer pointer pSM2 must not be null!");

      pSM2 += i2M2Stride;
      taAct = g_pelBufOP.AvgHighPassWithDownsamplingDiff2nd (width, height, pSrc, pSM1, pSM2, iSrcStride, iSM1Stride, iSM2Stride);
    }

    tempAct = double (taAct) / double ((width - 4) * (height - 4));
  }
  else // HD high-pass without downsampling
  {
    CHECK (pSM1 == nullptr || iSM1Stride <= 0 || iSM1Stride < width, "Pel buffer pointer pSM1 must not be null!");

    pSM1 += iSM1Stride;
    if (frameRate <= 31) // 1st-order delta
    {
      taAct = g_pelBufOP.HDHighPass (width, height, pSrc, pSM1, iSrcStride, iSM1Stride);
    }
    else // 2nd-order delta (diff of diffs)
    {
      CHECK (pSM2 == nullptr || iSM2Stride <= 0 || iSM2Stride < width, "Pel buffer pointer pSM2 must not be null!");

      pSM2 += iSM2Stride;
      taAct = g_pelBufOP.HDHighPass2 (width, height, pSrc, pSM1, pSM2, iSrcStride, iSM1Stride, iSM2Stride);
    }

    tempAct = double (taAct) / double ((width - 2) * (height - 2));
  }

  if (minVA) // minimum pt scaled to 12 bit
  {
    taAct = uint64_t (0.5 + tempAct * double (bitDepth < 12 ? 1 << (12 - bitDepth) : 1) * (frameRate <= 31 ? 1.15625 : 1.0));

    if (*minVA > taAct) *minVA = (unsigned) taAct;
  }

  // lower limit, compensate for high-pass amplification
  return std::max (double (1 << (bitDepth - 6)), spatAct + 2.0 * tempAct);
}

static double getAveragePictureActivity (const uint32_t picWidth,  const uint32_t picHeight,
                                         const int scaledAverageGopActivity,
                                         const bool tempFiltering, const uint32_t bitDepth)
{
  if (scaledAverageGopActivity > 0) 
  { 
    return (double (scaledAverageGopActivity) / double (1 << (24 - bitDepth)));
  }
  const double hpEnerPic = (tempFiltering ? 32.0 : 16.0) * double (1 << (2 * bitDepth - 10)) * sqrt ((3840.0 * 2160.0) / double (picWidth * picHeight));

  return sqrt (hpEnerPic); // square-root of a_pic value
}

static int getGlaringColorQPOffset (Picture* const pic, const int ctuAddr, const int bitDepth, uint32_t &avgLumaValue)
{
  const PreCalcValues& pcv  = *pic->cs->pcv;
  const ChromaFormat chrFmt = pic->chromaFormat;
  const SizeType chrWidth   = pcv.maxCUSize >> getChannelTypeScaleX (CH_C, chrFmt);
  const SizeType chrHeight  = pcv.maxCUSize >> getChannelTypeScaleY (CH_C, chrFmt);
  const unsigned w          = pcv.widthInCtus;
  const int      midLevel   = 1 << (bitDepth - 1);
  int chrValue = MAX_INT;

  avgLumaValue = uint32_t ((ctuAddr >= 0) ? pic->ctuAdaptedQP[ctuAddr] : pic->getOrigBuf().Y().getAvg());

  for (uint32_t comp = COMP_Cb; comp < MAX_NUM_COMP; comp++)
  {
    const ComponentID compID = (ComponentID) comp;
    int avgCompValue;

    if (ctuAddr >= 0) // chroma
    {
      const CompArea chrArea = clipArea (CompArea (compID, chrFmt, Area ((ctuAddr % w) * chrWidth, (ctuAddr / w) * chrHeight, chrWidth, chrHeight)), pic->block (compID));

      avgCompValue = pic->getOrigBuf (chrArea).getAvg();
    }
    else avgCompValue = pic->getOrigBuf (pic->block (compID)).getAvg();

    if (chrValue > avgCompValue) chrValue = avgCompValue; // minimum of the DC offsets
  }
  CHECK (chrValue < 0, "mean chroma value cannot be negative!");

  chrValue = (int) avgLumaValue - chrValue;

  if (chrValue > midLevel) return apprI3Log2 (double (chrValue * chrValue) / double (midLevel * midLevel));

  return 0;
}

static int getGlaringColorQPOffsetSubCtu (Picture* const pic, const CompArea& lumaArea, const int bitDepth, uint32_t &avgLumaValue)
{
  const ChromaFormat chrFmt = pic->chromaFormat;
  const SizeType chrWidth   = lumaArea.width  >> getChannelTypeScaleX (CH_C, chrFmt);
  const SizeType chrHeight  = lumaArea.height >> getChannelTypeScaleY (CH_C, chrFmt);
  const PosType  chrPosX    = lumaArea.x >> getChannelTypeScaleX (CH_C, chrFmt);
  const PosType  chrPosY    = lumaArea.y >> getChannelTypeScaleY (CH_C, chrFmt);
  const int      midLevel   = 1 << (bitDepth - 1);
  int chrValue = MAX_INT;

  avgLumaValue = pic->getOrigBuf (lumaArea).getAvg();

  for (uint32_t comp = COMP_Cb; comp < MAX_NUM_COMP; comp++)
  {
    const ComponentID compID = (ComponentID) comp;
    const CompArea   chrArea = clipArea (CompArea (compID, chrFmt, Area (chrPosX, chrPosY, chrWidth, chrHeight)), pic->block (compID));

    int avgCompValue = pic->getOrigBuf (chrArea).getAvg();

    if (chrValue > avgCompValue) chrValue = avgCompValue; // minimum of the DC offsets
  }
  CHECK (chrValue < 0, "mean chroma value cannot be negative!");

  chrValue = (int) avgLumaValue - chrValue;

  if (chrValue > midLevel) return apprI3Log2 (double (chrValue * chrValue) / double (midLevel * midLevel));

  return 0;
}

static void updateMinNoiseLevelsPic (uint8_t* const minNoiseLevels, const int bitDepth, const unsigned avgValue, const unsigned noise)
{
  const unsigned avgIndex = avgValue >> (bitDepth - 3); // one of 8 mean level regions

  CHECK (avgIndex >= QPA_MAX_NOISE_LEVELS, "array index out of bounds");

  if (noise < (unsigned) minNoiseLevels[avgIndex])
  {
    minNoiseLevels[avgIndex] = (uint8_t) noise;
  }
}

static void clipQPValToEstimatedMinimStats (const uint8_t* minNoiseLevels, const int bitDepth, const unsigned avgValue,
                                            const double resFac, const int extraQPOffset, int& QP) // output QP
{
  const unsigned avgIndex = avgValue >> (bitDepth - 3); // one of 8 mean level regions
  const unsigned x = (1 << 3) - 1;
  const int32_t dQPOffset = -15;

  CHECK (avgIndex >= QPA_MAX_NOISE_LEVELS, "array index out of bounds");

  int i = minNoiseLevels[avgIndex];

  // try to "fill in the blanks" in luma range (also results in peak smoothing, as described in PCS 2022 paper)
  if (avgIndex == 0 && i > minNoiseLevels[0 + 1]) i = minNoiseLevels[0 + 1];
  if (avgIndex == x && i > minNoiseLevels[x - 1]) i = minNoiseLevels[x - 1];

  if (avgIndex > 0 && avgIndex < x)
  {
    const uint8_t maxNeighborNoiseLevel = std::max (minNoiseLevels[avgIndex - 1], minNoiseLevels[avgIndex + 1]);

    if (i > maxNeighborNoiseLevel) i = maxNeighborNoiseLevel;
  }
  if (i >= 255)
  {
    return;
  }

  i = std::max (0, apprI3Log2 (std::min (16.0, resFac) * i * i) + dQPOffset + extraQPOffset); // min QP, 6*log2
  if (QP < i)
  {
    QP = i;
  }
}

static int refineDeltaQpDistribution (Picture* const pic, const VVEncCfg* encCfg,   const int sliceQP,
                                      const double sliceLambda, const int rcQpDiff, const int bitDepth,
                                      const uint32_t startAddr, const uint32_t endAddr, const int qpSum,
                                      const uint32_t tempLayer, const bool isIntra, const bool isEncPass,
                                      const uint8_t* minNoiseLevels, std::vector<int>& ctuAvgLuma)
{
  const double resRatio = (isEncPass ? sqrt (double (encCfg->m_SourceWidth * encCfg->m_SourceHeight) / (3840.0 * 2160.0)) : 0.0);
  const int ctusInSlice = int (endAddr - startAddr);
  const int targetQpSum = (encCfg->m_RCTargetBitrate > 0 ? sliceQP * ctusInSlice : qpSum);
  int blockQpSum = 0, tempLumaQP;
  double blockLambda;
  bool isLimited = false;

  for (uint32_t ctuRsAddr = startAddr; ctuRsAddr < endAddr; ctuRsAddr++)
  {
    int clippedLumaQP = std::max (0, pic->ctuAdaptedQP[ctuRsAddr] + rcQpDiff);

    if (isEncPass)
    {
      tempLumaQP = clippedLumaQP; // CTU QP before clipping for diff calculation below

      clipQPValToEstimatedMinimStats (minNoiseLevels, bitDepth, ctuAvgLuma[ctuRsAddr - startAddr], resRatio, (isIntra ? encCfg->m_intraQPOffset >> 1 : std::min (4, (int) tempLayer)), clippedLumaQP);
      if (clippedLumaQP > tempLumaQP)
      {
        ctuAvgLuma[ctuRsAddr - startAddr] = -1; // mark CTU as being processed already
        isLimited = isEncPass;
      }
    }

    clippedLumaQP = std::min (MAX_QP, clippedLumaQP);

    blockLambda = sliceLambda * pow (2.0, double (clippedLumaQP - sliceQP) / 3.0);
    blockQpSum += clippedLumaQP;

    pic->ctuQpaLambda[ctuRsAddr] = blockLambda;  // store modified CTU lambdas and QPs
    pic->ctuAdaptedQP[ctuRsAddr] = clippedLumaQP;
  }

  if (blockQpSum > targetQpSum && isLimited) // CTU QPs limited, so distribute saved rate among nonlimited CTUs
  {
    int maxCtuQP = 0, minCtuQP = MAX_QP;

    for (uint32_t ctuRsAddr = startAddr; ctuRsAddr < endAddr; ctuRsAddr++) // find max
    {
      if (ctuAvgLuma[ctuRsAddr - startAddr] >= 0 && pic->ctuAdaptedQP[ctuRsAddr] > maxCtuQP) // nonlimited CTUs
      {
        maxCtuQP = pic->ctuAdaptedQP[ctuRsAddr];
      }
      if (pic->ctuAdaptedQP[ctuRsAddr] < minCtuQP)
      {
        minCtuQP = pic->ctuAdaptedQP[ctuRsAddr];
      }
    }

    minCtuQP = std::max (0, minCtuQP);

    while (maxCtuQP > minCtuQP && blockQpSum > targetQpSum) // spend rate starting at max QPs, then go downward
    {
      for (uint32_t ctuRsAddr = startAddr; ctuRsAddr < endAddr; ctuRsAddr++) // reduce
      {
        if (ctuAvgLuma[ctuRsAddr - startAddr] >= 0 && pic->ctuAdaptedQP[ctuRsAddr] == maxCtuQP)
        {
          tempLumaQP = std::max (0, pic->ctuAdaptedQP[ctuRsAddr] - 1);

          ctuAvgLuma[ctuRsAddr - startAddr] = -1; // mark CTU as being reduced already
          blockLambda = sliceLambda * pow (2.0, double (tempLumaQP - sliceQP) / 3.0);
          if (tempLumaQP < pic->ctuAdaptedQP[ctuRsAddr]) blockQpSum--;

          pic->ctuQpaLambda[ctuRsAddr] = blockLambda; // store reduced lambdas and QPs
          pic->ctuAdaptedQP[ctuRsAddr] = tempLumaQP;
        }

        if (blockQpSum <= targetQpSum) break;
      }

      maxCtuQP--;
    }
  }

  return (blockQpSum - 1 + ((ctusInSlice + 1) >> 1)) / ctusInSlice;
}

// public functions

int BitAllocation::applyQPAdaptationSlice (const Slice* slice, const VVEncCfg* encCfg, const int sliceQP,
                                           const double sliceLambda, uint16_t* const picVisActLuma,
                                           std::vector<int>& ctuPumpRedQP, std::vector<uint8_t>* ctuRCQPMemory,
                                           int* const optChromaQPOffsets, const uint8_t* minNoiseLevels,
                                           const uint32_t ctuStartAddr, const uint32_t ctuBoundingAddr)
{
  Picture* const pic          = (slice != nullptr ? slice->pic : nullptr);
  double hpEner[MAX_NUM_COMP] = {0.0, 0.0, 0.0};
  double averageAdaptedLambda = 0.0;
  int    averageAdaptedLumaQP = -1;
  uint32_t meanLuma           = MAX_UINT;
  std::vector<int> ctuAvgLuma;

  if (pic == nullptr || pic->cs == nullptr || encCfg == nullptr || ctuStartAddr >= ctuBoundingAddr)
  {
    return -1;
  }

  const bool isEncPass        = (encCfg->m_LookAhead > 0 && !slice->pic->isPreAnalysis);
  const bool isHDR            = (encCfg->m_HdrMode != vvencHDRMode::VVENC_HDR_OFF) && !(encCfg->m_lumaReshapeEnable != 0 && encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ);
  const bool isBIM            = (encCfg->m_blockImportanceMapping && !pic->m_picShared->m_ctuBimQpOffset.empty());
  const bool isHighResolution = (encCfg->m_PadSourceWidth > 2048 || encCfg->m_PadSourceHeight > 1280);
  const bool useFrameWiseQPA  = (encCfg->m_QP > MAX_QP_PERCEPT_QPA) && (encCfg->m_framesToBeEncoded != 1) && (slice->TLayer > 0);
  const int  bitDepth         = slice->sps->bitDepths[CH_L];
  const double hpEnerPicNorm  = 1.0 / getAveragePictureActivity (encCfg->m_PadSourceWidth, encCfg->m_PadSourceHeight, (encCfg->m_RCNumPasses == 2 ? 0 : ctuPumpRedQP.back()),
                                                                 (encCfg->m_usePerceptQPATempFiltISlice || !slice->isIntra()), bitDepth);
  const PreCalcValues& pcv    = *pic->cs->pcv;

  pic->picInitialQP = sliceQP;  // modified below and used in applyQPAdaptationSubCtu
  if ((encCfg->m_RCTargetBitrate > 0) && useFrameWiseQPA)
  {
    averageAdaptedLumaQP = Clip3 (0, MAX_QP, sliceQP - 1); // one will be added again
  }

  for (uint32_t comp = 0; comp < getNumberValidComponents (pic->chromaFormat); comp++)
  {
    const ComponentID compID  = (ComponentID) comp;

    if (isLuma (compID)) // luma: CTU-wise QPA operation
    {
#if USE_SP_ACT
      uint32_t spatVisAct = 0;
#endif
      const PosType guardSize = (isHighResolution ? 2 : 1);

      for (uint32_t ctuRsAddr = ctuStartAddr; ctuRsAddr < ctuBoundingAddr; ctuRsAddr++)
      {
        const Position pos ((ctuRsAddr % pcv.widthInCtus) * pcv.maxCUSize, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUSize);
        const CompArea ctuArea   = clipArea (CompArea (COMP_Y, pic->chromaFormat, Area (pos.x, pos.y, pcv.maxCUSize, pcv.maxCUSize)), pic->Y());
        const SizeType fltWidth  = pcv.maxCUSize + guardSize * (pos.x > 0 ? 2 : 1);
        const SizeType fltHeight = pcv.maxCUSize + guardSize * (pos.y > 0 ? 2 : 1);
        const CompArea fltArea   = clipArea (CompArea (COMP_Y, pic->chromaFormat, Area (pos.x > 0 ? pos.x - guardSize : 0, pos.y > 0 ? pos.y - guardSize : 0, fltWidth, fltHeight)), pic->Y());
        const CPelBuf  picOrig   = pic->getOrigBuf (fltArea);
        const CPelBuf  picPrv1   = pic->getOrigBufPrev (fltArea, PREV_FRAME_1);
        const CPelBuf  picPrv2   = pic->getOrigBufPrev (fltArea, PREV_FRAME_2);
        unsigned minActivityPart = 0;
#if USE_SP_ACT
        unsigned spatActivityCTU = 0;
#endif

        hpEner[1] = filterAndCalculateAverageActivity (picOrig.buf, picOrig.stride, picOrig.height, picOrig.width,
                                                       picPrv1.buf, picPrv1.stride, picPrv2.buf, picPrv2.stride, encCfg->m_FrameRate / encCfg->m_FrameScale,
                                                       bitDepth, isHighResolution, &minActivityPart
#if USE_SP_ACT
          , &spatActivityCTU
#endif
        );
        hpEner[comp] += hpEner[1] * double (ctuArea.width * ctuArea.height);
        pic->ctuQpaLambda[ctuRsAddr] = hpEner[1]; // temporary backup of CTU mean visual activity
        pic->ctuAdaptedQP[ctuRsAddr] = (int) pic->getOrigBuf (ctuArea).getAvg(); // and mean luma

        if (picOrig.buf == picPrv1.buf) // replace temporal visual activity with min motion error
        {
          hpEner[1] = pic->m_picShared->m_minNoiseLevels[pic->ctuAdaptedQP[ctuRsAddr] >> (bitDepth - 3)] * (bitDepth >= 10 ? 1.5 : 0.375);

          if (hpEner[1] < (bitDepth >= 10 ? 382.5 : 95.625)) // levels in first frame
          {
            hpEner[comp] += hpEner[1] * double (ctuArea.width * ctuArea.height);
            pic->ctuQpaLambda[ctuRsAddr] += hpEner[1]; // add noise level to mean visual activity
          }
        }
        else if (!isEncPass)
        {
          updateMinNoiseLevelsPic (pic->m_picShared->m_minNoiseLevels, bitDepth, pic->ctuAdaptedQP[ctuRsAddr], minActivityPart);
        }
#if USE_SP_ACT
        spatVisAct += spatActivityCTU;
#endif
      }

      hpEner[comp] /= double (encCfg->m_SourceWidth * encCfg->m_SourceHeight);
      if (picVisActLuma != nullptr)
      {
        *picVisActLuma = ClipBD (uint16_t (0.5 + hpEner[comp]), bitDepth);
      }
#if USE_SP_ACT
      if (ctuBoundingAddr > ctuStartAddr)
      {
        const uint32_t nCtu = ctuBoundingAddr - ctuStartAddr;

        pic->picSpatVisAct = (spatVisAct + (nCtu >> 1)) / nCtu;
      }
#endif
    }
    else // chroma: only picture-wise operation required
    {
      const CPelBuf picOrig = pic->getOrigBuf (compID);
      const CPelBuf picPrv1 = pic->getOrigBufPrev (compID, PREV_FRAME_1);
      const CPelBuf picPrv2 = pic->getOrigBufPrev (compID, PREV_FRAME_2);

      hpEner[comp] = filterAndCalculateAverageActivity (picOrig.buf, picOrig.stride, picOrig.height, picOrig.width,
                                                        picPrv1.buf, picPrv1.stride, picPrv2.buf, picPrv2.stride, encCfg->m_FrameRate / encCfg->m_FrameScale,
                                                        bitDepth, isHighResolution && (pic->chromaFormat == CHROMA_444));

      const int adaptChromaQPOffset = 2.0 * hpEner[comp] <= hpEner[0] ? 0 : apprI3Log2 (2.0 * hpEner[comp] / hpEner[0]);

      if (averageAdaptedLumaQP < 0) // YUV is not 4:0:0!
      {
        averageAdaptedLumaQP = Clip3 (0, MAX_QP, sliceQP + apprI3Log2 (hpEner[0] * hpEnerPicNorm));

        if (isChromaEnabled (pic->chromaFormat) && (averageAdaptedLumaQP < MAX_QP))
        {
          averageAdaptedLumaQP += getGlaringColorQPOffset (pic, -1 /*ctuAddr*/, slice->sps->bitDepths[CH_C], meanLuma);

          if ((averageAdaptedLumaQP > MAX_QP) && !isHDR) averageAdaptedLumaQP = MAX_QP;
        }
        // change mean picture QP index based on picture's average luma value (Sharp)
        if (isHDR)
        {
          if (meanLuma == MAX_UINT) meanLuma = pic->getOrigBuf().Y().getAvg();

          averageAdaptedLumaQP = Clip3 (0, MAX_QP, averageAdaptedLumaQP + lumaDQPOffset (meanLuma, bitDepth));
        }
      }

      if (optChromaQPOffsets != nullptr) // adapts sliceChromaQpOffsetIntraOrPeriodic
      {
        GCC_WARNING_DISABLE_maybe_uninitialized // probably spurious warning, when building with -fsanitize=undefined: "error: 'encCfg.33' may be used uninitialized in this function"
        const int lumaChromaMappingDQP = (averageAdaptedLumaQP - slice->sps->chromaQpMappingTable.getMappedChromaQpValue (compID, averageAdaptedLumaQP)) >> (encCfg->m_QP >= MAX_QP_PERCEPT_QPA ? 1 : 0);
        GCC_WARNING_RESET
        optChromaQPOffsets[comp - 1] = std::min (3 + lumaChromaMappingDQP, adaptChromaQPOffset + lumaChromaMappingDQP);
      }
    } // isLuma or isChroma
  }

  if (averageAdaptedLumaQP < 0) // only if YUV is 4:0:0!
  {
    averageAdaptedLumaQP = Clip3 (0, MAX_QP, sliceQP + apprI3Log2 (hpEner[0] * hpEnerPicNorm));

    // change mean picture QP index based on the picture's average luma value (Sharp)
    if (isHDR)
    {
      if (meanLuma == MAX_UINT) meanLuma = pic->getOrigBuf().Y().getAvg();

      averageAdaptedLumaQP = Clip3 (0, MAX_QP, averageAdaptedLumaQP + lumaDQPOffset (meanLuma, bitDepth));
    }
  }

  if (encCfg->m_RCNumPasses == 2 && (encCfg->m_RCTargetBitrate > 0) && (ctuRCQPMemory != nullptr) && slice->pps->useDQP && (encCfg->m_usePerceptQPATempFiltISlice == 2) && slice->isIntra())
  {
    const int nCtu = int (ctuBoundingAddr - ctuStartAddr);
    const int offs = (slice->poc / encCfg->m_IntraPeriod) * ((nCtu + 1) >> 1);
    std::vector<uint8_t>& ctuQPMem = *ctuRCQPMemory; // unpack 1st-pass reduction QPs

    if ((ctuPumpRedQP.size() >= nCtu) && (ctuQPMem.size() >= offs + ((nCtu + 1) >> 1)))
    {
      for (uint32_t ctuRsAddr = ctuStartAddr; ctuRsAddr < ctuBoundingAddr; ctuRsAddr++)
      {
        ctuPumpRedQP[ctuRsAddr] = int ((ctuRsAddr & 1) ? ctuQPMem[offs + (ctuRsAddr >> 1)] >> 4 : ctuQPMem[offs + (ctuRsAddr >> 1)] & 15) - 8;
      }
    }
  }

  if (useFrameWiseQPA || (averageAdaptedLumaQP >= MAX_QP)) // store the CTU-wise QP/lambda values
  {
    averageAdaptedLumaQP = std::min (MAX_QP, averageAdaptedLumaQP + 1);
    averageAdaptedLambda = sliceLambda * pow (2.0, double (averageAdaptedLumaQP - sliceQP) / 3.0);

    for (uint32_t ctuRsAddr = ctuStartAddr; ctuRsAddr < ctuBoundingAddr; ctuRsAddr++)
    {
      pic->ctuQpaLambda[ctuRsAddr] = averageAdaptedLambda; // save adapted lambda, QP
      pic->ctuAdaptedQP[ctuRsAddr] = averageAdaptedLumaQP;
    }
  }
  else // use CTU-level QPA
  {
    const int nCtu = int (ctuBoundingAddr - ctuStartAddr);
    const int dvsr = encCfg->m_IntraPeriod - encCfg->m_GOPSize;
    const int aaQP = averageAdaptedLumaQP; // backup of initial average QP from above
    const bool rcIsFirstPassOf2 = ((encCfg->m_RCTargetBitrate == 0) && (ctuRCQPMemory != nullptr) && slice->pps->useDQP && (slice->poc > 0) ? encCfg->m_RCNumPasses == 2 : false);

    if (isEncPass) ctuAvgLuma.resize (nCtu);

    averageAdaptedLumaQP = -1;
    for (uint32_t ctuRsAddr = ctuStartAddr; ctuRsAddr < ctuBoundingAddr; ctuRsAddr++)
    {
      const double hpEnerCTU = pic->ctuQpaLambda[ctuRsAddr];
      int adaptedLumaQP = Clip3 (0, MAX_QP, sliceQP + apprI3Log2 (hpEnerCTU * hpEnerPicNorm));

      if ((encCfg->m_usePerceptQPATempFiltISlice == 2) && slice->isIntra() && (ctuPumpRedQP.size() > ctuRsAddr))
      {
        if (rcIsFirstPassOf2) // backup 1st-pass I-frame QP for 2nd rate control pass
        {
          if (ctuRsAddr & 1) ctuRCQPMemory->back() |= (Clip3 (-8, 7, ctuPumpRedQP[ctuRsAddr]) + 8) << 4;
          else /*even addr*/ ctuRCQPMemory->push_back (Clip3 (-8, 7, ctuPumpRedQP[ctuRsAddr]) + 8);
          if (adaptedLumaQP > 0)
          {
            adaptedLumaQP -= (aaQP >> 4); // some first-pass tuning for stabilization
          }
        }
        if (ctuPumpRedQP[ctuRsAddr] < 0) adaptedLumaQP = Clip3 (0, MAX_QP, adaptedLumaQP + (ctuPumpRedQP[ctuRsAddr] * encCfg->m_GOPSize - (dvsr >> 1)) / dvsr);
        else /*ctuPumpRedQP[addr] >= 0*/ adaptedLumaQP = Clip3 (0, MAX_QP, adaptedLumaQP + (ctuPumpRedQP[ctuRsAddr] * encCfg->m_GOPSize + (dvsr >> 1)) / dvsr);

        ctuPumpRedQP[ctuRsAddr] = 0; // reset QP memory for temporal pumping analysis
      }
      if ((encCfg->m_usePerceptQPATempFiltISlice == 2) && !slice->isIntra() && (slice->TLayer == 0) && rcIsFirstPassOf2 && (adaptedLumaQP < MAX_QP))
      {
        adaptedLumaQP++; // this is a first-pass tuning to stabilize the rate control
      }
      meanLuma = MAX_UINT;
      if (isChromaEnabled (pic->chromaFormat) && (adaptedLumaQP < MAX_QP))
      {
        adaptedLumaQP += getGlaringColorQPOffset (pic, (int) ctuRsAddr, slice->sps->bitDepths[CH_C], meanLuma);

        if ((adaptedLumaQP > MAX_QP) && !isHDR) adaptedLumaQP = MAX_QP;
      }
      // change the CTU-level QP index based on CTU area's average luma value (Sharp)
      if (isHDR)
      {
        if (meanLuma == MAX_UINT) meanLuma = pic->ctuAdaptedQP[ctuRsAddr];

        adaptedLumaQP = Clip3 (0, MAX_QP, adaptedLumaQP + lumaDQPOffset (meanLuma, bitDepth));
      }
      // add further delta-QP of block importance mapping (BIM) detector if available
      if (isBIM)
      {
        adaptedLumaQP = Clip3 (-slice->sps->qpBDOffset[CH_L], MAX_QP, adaptedLumaQP + pic->m_picShared->m_ctuBimQpOffset[ctuRsAddr]);
      }
      // reduce delta-QP variance, avoid wasting precious bit budget at low bit-rates
      if ((encCfg->m_RCTargetBitrate == 0) && (3 + encCfg->m_QP > MAX_QP_PERCEPT_QPA) && (encCfg->m_framesToBeEncoded != 1))
      {
        const int retunedAdLumaQP = adaptedLumaQP + 1;

        adaptedLumaQP = (std::max (0, 1 + MAX_QP_PERCEPT_QPA - encCfg->m_QP) * adaptedLumaQP + std::min (4, 3 + encCfg->m_QP - MAX_QP_PERCEPT_QPA) * aaQP + 2) >> 2;
        if (adaptedLumaQP > retunedAdLumaQP) adaptedLumaQP = retunedAdLumaQP;
        if (adaptedLumaQP < MAX_QP && encCfg->m_QP == MAX_QP_PERCEPT_QPA && slice->TLayer > 1) adaptedLumaQP++; // a fine-tuning
      }
      if (isEncPass) ctuAvgLuma[ctuRsAddr - ctuStartAddr] = pic->ctuAdaptedQP[ctuRsAddr];

      averageAdaptedLambda = sliceLambda * pow (2.0, double (adaptedLumaQP - sliceQP) / 3.0);
      averageAdaptedLumaQP += adaptedLumaQP;

      pic->ctuQpaLambda[ctuRsAddr] = averageAdaptedLambda; // save adapted lambda, QP
      pic->ctuAdaptedQP[ctuRsAddr] = adaptedLumaQP;
    }

    meanLuma = std::max (0, averageAdaptedLumaQP + 1);
    averageAdaptedLumaQP = (averageAdaptedLumaQP + ((nCtu + 1) >> 1)) / nCtu;

    if ((encCfg->m_RCTargetBitrate > 0 && averageAdaptedLumaQP != sliceQP) || (isEncPass)) // QP/rate control
    {
      const int rcQpDiff = (encCfg->m_RCTargetBitrate > 0 ? sliceQP - averageAdaptedLumaQP : 0);

      averageAdaptedLumaQP = refineDeltaQpDistribution (pic, encCfg, sliceQP, sliceLambda, rcQpDiff, bitDepth, ctuStartAddr, ctuBoundingAddr,
                                                        meanLuma, slice->TLayer, slice->isIntra(), isEncPass, minNoiseLevels, ctuAvgLuma);

      pic->picInitialQP = Clip3 (0, MAX_QP, pic->picInitialQP + rcQpDiff); // used in applyQPAdaptationSubCtu
      pic->isMeanQPLimited = (encCfg->m_RCTargetBitrate > 0) && isEncPass && (averageAdaptedLumaQP > sliceQP);
    }
    else if ((encCfg->m_RCTargetBitrate == 0) && (3 + encCfg->m_QP > MAX_QP_PERCEPT_QPA) && (encCfg->m_framesToBeEncoded != 1) && (averageAdaptedLumaQP + 1 < aaQP))
    {
      const int lrQpDiff = (aaQP - averageAdaptedLumaQP) >> (encCfg->m_QP <= MAX_QP_PERCEPT_QPA ? 2 : 1); // for monotonous rate change at low rates

      averageAdaptedLambda = pow (2.0, double (lrQpDiff) / 3.0);
      for (uint32_t ctuRsAddr = ctuStartAddr; ctuRsAddr < ctuBoundingAddr; ctuRsAddr++)
      {
        pic->ctuQpaLambda[ctuRsAddr] *= averageAdaptedLambda; // scale adapted lambda
        pic->ctuAdaptedQP[ctuRsAddr] = std::min (MAX_QP, pic->ctuAdaptedQP[ctuRsAddr] + lrQpDiff);
      }

      pic->picInitialQP = Clip3 (0, MAX_QP, pic->picInitialQP + lrQpDiff); // used in applyQPAdaptationSubCtu
      averageAdaptedLumaQP = aaQP; // TODO hlm: += lrQpDiff?

      pic->isMeanQPLimited = false;
    }

    if (isEncPass) ctuAvgLuma.clear();
  } // CTU-/frame-level QPA

  return averageAdaptedLumaQP;
}

int BitAllocation::applyQPAdaptationSubCtu (const Slice* slice, const VVEncCfg* encCfg, const Area& lumaArea, const uint8_t* minNoiseLevels)
{
  Picture* const pic          = (slice != nullptr ? slice->pic : nullptr);
  uint32_t meanLuma           = MAX_UINT;

  if (pic == nullptr || encCfg == nullptr)
  {
    return -1;
  }

  const bool isEncPass        = (encCfg->m_LookAhead > 0 && !slice->pic->isPreAnalysis);
  const bool isHDR            = (encCfg->m_HdrMode != vvencHDRMode::VVENC_HDR_OFF) && !(encCfg->m_lumaReshapeEnable != 0 && encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ);
  const bool isBIM            = (encCfg->m_blockImportanceMapping && !pic->m_picShared->m_ctuBimQpOffset.empty());
  const bool isHighResolution = (encCfg->m_PadSourceWidth > 2048 || encCfg->m_PadSourceHeight > 1280);
  const int  bitDepth         = slice->sps->bitDepths[CH_L];
  const PosType     guardSize = (isHighResolution ? 2 : 1);
  const Position    pos       = lumaArea.pos();
  const CompArea    subArea   = clipArea (CompArea (COMP_Y, pic->chromaFormat, Area (pos.x, pos.y, lumaArea.width, lumaArea.height)), pic->Y());
  const SizeType    fltWidth  = lumaArea.width  + guardSize * (pos.x > 0 ? 2 : 1);
  const SizeType    fltHeight = lumaArea.height + guardSize * (pos.y > 0 ? 2 : 1);
  const CompArea    fltArea   = clipArea (CompArea (COMP_Y, pic->chromaFormat, Area (pos.x > 0 ? pos.x - guardSize : 0, pos.y > 0 ? pos.y - guardSize : 0, fltWidth, fltHeight)), pic->Y());
  const CPelBuf     picOrig   = pic->getOrigBuf (fltArea);
  const CPelBuf     picPrv1   = pic->getOrigBufPrev (fltArea, PREV_FRAME_1);
  const CPelBuf     picPrv2   = pic->getOrigBufPrev (fltArea, PREV_FRAME_2);
  const double hpEnerSubCTU   = filterAndCalculateAverageActivity (picOrig.buf, picOrig.stride, picOrig.height, picOrig.width,
                                                                   picPrv1.buf, picPrv1.stride, picPrv2.buf, picPrv2.stride, encCfg->m_FrameRate / encCfg->m_FrameScale,
                                                                   bitDepth, isHighResolution);
  const double hpEnerPicNorm  = 1.0 / getAveragePictureActivity (encCfg->m_PadSourceWidth, encCfg->m_PadSourceHeight, 0,
                                                                 (encCfg->m_usePerceptQPATempFiltISlice || !slice->isIntra()), bitDepth);
  int adaptedSubCtuQP = Clip3 (0, MAX_QP, pic->picInitialQP + apprI3Log2 (hpEnerSubCTU * hpEnerPicNorm));

  if (isChromaEnabled (pic->chromaFormat) && (adaptedSubCtuQP < MAX_QP))
  {
    adaptedSubCtuQP += getGlaringColorQPOffsetSubCtu (pic, subArea, slice->sps->bitDepths[CH_C], meanLuma);

    if ((adaptedSubCtuQP > MAX_QP) && !isHDR) adaptedSubCtuQP = MAX_QP;
  }
  // change the sub-CTU-level QP index based on sub-area's average luma value (Sharp)
  if (isHDR)
  {
    if (meanLuma == MAX_UINT) meanLuma = pic->getOrigBuf (subArea).getAvg();

    adaptedSubCtuQP = Clip3 (0, MAX_QP, adaptedSubCtuQP + lumaDQPOffset (meanLuma, bitDepth));
  }
  // add additional delta-QP of block importance mapping (BIM) detection if available
  if (isBIM)
  {
    adaptedSubCtuQP = Clip3 (-slice->sps->qpBDOffset[CH_L], MAX_QP, adaptedSubCtuQP + pic->m_picShared->m_ctuBimQpOffset[getCtuAddr (pos, *pic->cs->pcv)]);
  }
  // reduce the delta-QP variance, avoid wasting precious bit budget at low bit-rates
  if ((encCfg->m_RCTargetBitrate == 0) && (3 + encCfg->m_QP > MAX_QP_PERCEPT_QPA) && (slice->sliceQp >= 0) && (encCfg->m_framesToBeEncoded != 1))
  {
    const int retunedAdLumaQP = adaptedSubCtuQP + 1;

    adaptedSubCtuQP = (std::max (0, 1 + MAX_QP_PERCEPT_QPA - encCfg->m_QP) * adaptedSubCtuQP + std::min (4, 3 + encCfg->m_QP - MAX_QP_PERCEPT_QPA) * slice->sliceQp + 2) >> 2;
    if (adaptedSubCtuQP > retunedAdLumaQP) adaptedSubCtuQP = retunedAdLumaQP;
    if (adaptedSubCtuQP < MAX_QP && encCfg->m_QP >= MAX_QP_PERCEPT_QPA) adaptedSubCtuQP++; // for monotonous rate change, l. 507
  }
  if (isEncPass)
  {
    const double resRatio = sqrt (double (encCfg->m_SourceWidth * encCfg->m_SourceHeight) / (3840.0 * 2160.0));

    if (meanLuma == MAX_UINT) meanLuma = pic->getOrigBuf (subArea).getAvg();
    clipQPValToEstimatedMinimStats (minNoiseLevels, bitDepth, meanLuma, resRatio, (slice->isIntra() ? encCfg->m_intraQPOffset >> 1 : std::min (4, (int) slice->TLayer)), adaptedSubCtuQP);
  }

  return adaptedSubCtuQP;
}

int BitAllocation::getCtuPumpingReducingQP (const Slice* slice, const CPelBuf& origY, const Distortion uiSadBestForQPA,
                                            std::vector<int>& ctuPumpRedQP, const uint32_t ctuRsAddr, const int baseQP)
{
  if (slice == nullptr || !slice->pps->useDQP || ctuPumpRedQP.size() <= ctuRsAddr) return 0;

  const int32_t avgOrig = origY.getAvg();
  uint32_t sumAbsZmOrig = 0; // zero-mean
  const Pel* src = origY.buf;

  for (SizeType y = 0; y < origY.height; y++) // sum up the zero-mean absolute values
  {
    for (SizeType x = 0; x < origY.width; x++)
    {
      sumAbsZmOrig += (uint32_t) abs (src[x] - avgOrig);
    }
    src += origY.stride;
  }

  const double sumAbsRatio = double (uiSadBestForQPA * 3 /*TODO: or 4? fine-tune!*/) / double (sumAbsZmOrig == 0 ? 1 : sumAbsZmOrig);
  const int pumpingReducQP = (int (log (Clip3 (0.25, 4.0, sumAbsRatio)) / log (2.0) + (sumAbsRatio < 1.0 ? -0.5 : 0.5))) >> (baseQP >= 38/*MAX_QP_PERCEPT_QPA*/ ? 1 : 0);

  ctuPumpRedQP[ctuRsAddr] += pumpingReducQP;

  return pumpingReducQP;
}

} // namespace vvenc

//! \}

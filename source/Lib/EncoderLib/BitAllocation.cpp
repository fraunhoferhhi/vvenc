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


/** \file     BitAllocation.cpp
\brief    Bit allocation class for QP adaptation and, possibly, rate control
*/

#include "BitAllocation.h"
#include "CommonLib/Picture.h"
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

static double filterAndCalculateAverageActivity (const Pel* pSrc, const int iSrcStride, const int height, const int width,
                                                 const Pel* pSM1, const int iSM1Stride, const Pel* pSM2, const int iSM2Stride,
                                                 uint32_t frameRate, const uint32_t bitDepth, const bool isUHD, unsigned* minVA = nullptr)
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

static void updateMinNoiseLevelsPic( Picture* pic, const int bitDepth, const unsigned avgValue, const unsigned noise )
{
  const unsigned avgIndex = avgValue >> (bitDepth - 3); // one of 8 mean level regions

  CHECK( avgIndex >= QPA_MAX_NOISE_LEVELS, "array index out of bounds" );

  if (noise < (unsigned) pic->minNoiseLevels [avgIndex])
  {
    pic->minNoiseLevels [avgIndex] = (uint8_t) noise;
  }
}

static void clipQPValToEstimatedMinimStats( const uint8_t* minNoiseLevels, const int bitDepth, const unsigned avgValue,
                                            const double resFac, const int extraQPOffset, int& QP ) // output QP
{
  const unsigned avgIndex = avgValue >> (bitDepth - 3); // one of 8 mean level regions
  const int32_t dQPOffset = -9;

  CHECK( avgIndex >= QPA_MAX_NOISE_LEVELS, "array index out of bounds" );

  int i = minNoiseLevels[avgIndex];

  if (i >= 255 && avgIndex > 0 && avgIndex < (1 << 3) - 1)
  {
    i = std::min (minNoiseLevels [avgIndex - 1], minNoiseLevels [avgIndex + 1]); // try neighbors
  }
  else if (i && avgIndex > 0 && avgIndex < (1 << 3) - 1)
  {
    if (minNoiseLevels [avgIndex - 1] < 255 && minNoiseLevels [avgIndex + 1] < 255)
    {
      i = (2 * i + minNoiseLevels [avgIndex - 1] + minNoiseLevels [avgIndex + 1] + 2) >> 2;
    }
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

// public functions

int BitAllocation::applyQPAdaptationChroma (const Slice* slice, const VVEncCfg* encCfg, const int sliceQP, uint16_t* const picVisActLuma,
                                            std::vector<int>& ctuPumpRedQP, int* const optChromaQPOffsets,
                                            const uint32_t ctuStartAddr, const uint32_t ctuBoundingAddr)
{
  Picture* const pic          = (slice != nullptr ? slice->pic : nullptr);
  double hpEner[MAX_NUM_COMP] = {0.0, 0.0, 0.0};
  int    savedLumaQP          = -1;
  uint32_t meanLuma           = MAX_UINT;

  if (pic == nullptr || encCfg == nullptr) return -1;

  const bool isHDR            = (encCfg->m_HdrMode != vvencHDRMode::VVENC_HDR_OFF) && !(encCfg->m_lumaReshapeEnable != 0 && encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ);
  const bool isHighResolution = (encCfg->m_PadSourceWidth > 2048 || encCfg->m_PadSourceHeight > 1280);
  const int          bitDepth = slice->sps->bitDepths[CH_L];
  const PreCalcValues&    pcv = *pic->cs->pcv;

  for (uint32_t comp = 0; comp < getNumberValidComponents (pic->chromaFormat); comp++)
  {
    const ComponentID  compID = (ComponentID) comp;

    if (isLuma (compID)) // luma: CTU-wise QPA operation
    {
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

        hpEner[1] = filterAndCalculateAverageActivity (picOrig.buf, picOrig.stride, picOrig.height, picOrig.width,
                                                       picPrv1.buf, picPrv1.stride, picPrv2.buf, picPrv2.stride, encCfg->m_FrameRate/encCfg->m_FrameScale,
                                                       bitDepth, isHighResolution, &minActivityPart);
        hpEner[comp] += hpEner[1] * double (ctuArea.width * ctuArea.height);
        pic->ctuQpaLambda[ctuRsAddr] = hpEner[1]; // temporary backup of CTU mean visual activity
        pic->ctuAdaptedQP[ctuRsAddr] = pic->getOrigBuf (ctuArea).getAvg(); // and mean luma value

        updateMinNoiseLevelsPic( pic, bitDepth, pic->ctuAdaptedQP[ ctuRsAddr ], minActivityPart );
      }

      hpEner[comp] /= double (encCfg->m_SourceWidth * encCfg->m_SourceHeight);
      if (picVisActLuma != nullptr) *picVisActLuma = ClipBD (uint16_t (0.5 + hpEner[comp]), bitDepth);
    }
    else // chroma: only picture-wise operation required
    {
      const CPelBuf picOrig = pic->getOrigBuf (compID);
      const CPelBuf picPrv1 = pic->getOrigBufPrev (compID, PREV_FRAME_1);
      const CPelBuf picPrv2 = pic->getOrigBufPrev (compID, PREV_FRAME_2);

      hpEner[comp] = filterAndCalculateAverageActivity (picOrig.buf, picOrig.stride, picOrig.height, picOrig.width,
                                                        picPrv1.buf, picPrv1.stride, picPrv2.buf, picPrv2.stride, encCfg->m_FrameRate/encCfg->m_FrameScale,
                                                        bitDepth, isHighResolution && (pic->chromaFormat == CHROMA_444));

      const int adaptChromaQPOffset = 2.0 * hpEner[comp] <= hpEner[0] ? 0 : apprI3Log2 (2.0 * hpEner[comp] / hpEner[0]);

      if (savedLumaQP < 0)
      {
        int averageAdaptedLumaQP = Clip3 (0, MAX_QP, sliceQP + apprI3Log2 (hpEner[0] / getAveragePictureActivity (encCfg->m_PadSourceWidth, encCfg->m_PadSourceHeight,
                                                                                                                  encCfg->m_RCNumPasses == 2 ? 0 : ctuPumpRedQP.back(),
                                                                                                                  (encCfg->m_usePerceptQPATempFiltISlice || !slice->isIntra()), bitDepth)));
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

        savedLumaQP = averageAdaptedLumaQP;
      } // savedLumaQP < 0

      GCC_WARNING_DISABLE_maybe_uninitialized   // probably spurious warning, when building with -fsanitize=undefined: "error: 'encCfg.33' may be used uninitialized in this function"
      const int lumaChromaMappingDQP = (savedLumaQP - slice->sps->chromaQpMappingTable.getMappedChromaQpValue (compID, savedLumaQP)) >> (encCfg->m_QP >= MAX_QP_PERCEPT_QPA ? 1 : 0);
      GCC_WARNING_RESET

      if (optChromaQPOffsets != nullptr) optChromaQPOffsets[comp - 1] = std::min (3 + lumaChromaMappingDQP, adaptChromaQPOffset + lumaChromaMappingDQP);
    } // isChroma (compID)
  }

  return savedLumaQP;
}

int BitAllocation::applyQPAdaptationLuma (const Slice* slice, const VVEncCfg* encCfg, const int savedQP, const double lambda,
                                          std::vector<int>& ctuPumpRedQP, std::vector<uint8_t>* ctuRCQPMemory,
                                          const uint8_t* minNoiseLevels, const uint32_t ctuStartAddr, const uint32_t ctuBoundingAddr)
{
  Picture* const pic          = (slice != nullptr ? slice->pic : nullptr);
  double hpEnerPic, hpEnerAvg = 0.0;
  int    adaptedSliceQP       = -1;
  uint32_t meanLuma           = MAX_UINT;
  std::vector<Pel> ctuAvgLuma;

  if (pic == nullptr || pic->cs == nullptr || encCfg == nullptr || ctuStartAddr >= ctuBoundingAddr) return -1;

  const bool isEncPass        = (encCfg->m_LookAhead > 0 && !slice->pic->isPreAnalysis);
  const bool isHDR            = (encCfg->m_HdrMode != vvencHDRMode::VVENC_HDR_OFF) && !(encCfg->m_lumaReshapeEnable != 0 && encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ);
  const bool isHighResolution = (encCfg->m_PadSourceWidth > 2048 || encCfg->m_PadSourceHeight > 1280);
  const bool useFrameWiseQPA  = (encCfg->m_QP > MAX_QP_PERCEPT_QPA) && (encCfg->m_framesToBeEncoded != 1) && (slice->TLayer > 0);
  const int          bitDepth = slice->sps->bitDepths[CH_L];
  const int           sliceQP = (savedQP < 0 ? slice->sliceQp : savedQP);
  const PreCalcValues&    pcv = *pic->cs->pcv;

  if (!useFrameWiseQPA || (savedQP < 0)) // mean visual activity value and luma value in each CTU
  {
    if (savedQP >= 0) // data available from chroma QPA!
    {
      for (uint32_t ctuRsAddr = ctuStartAddr; ctuRsAddr < ctuBoundingAddr; ctuRsAddr++)
      {
        const Position pos ((ctuRsAddr % pcv.widthInCtus) * pcv.maxCUSize, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUSize);
        const CompArea ctuArea   = clipArea (CompArea (COMP_Y, pic->chromaFormat, Area (pos.x, pos.y, pcv.maxCUSize, pcv.maxCUSize)), pic->Y());

        hpEnerAvg += pic->ctuQpaLambda[ctuRsAddr] * double (ctuArea.width * ctuArea.height);
      }
    }
    else // per-CTU data missing, CTU-wise QPA operation
    {
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

        hpEnerPic = filterAndCalculateAverageActivity (picOrig.buf, picOrig.stride, picOrig.height, picOrig.width,
                                                       picPrv1.buf, picPrv1.stride, picPrv2.buf, picPrv2.stride, encCfg->m_FrameRate/encCfg->m_FrameScale,
                                                       bitDepth, isHighResolution, &minActivityPart);
        hpEnerAvg += hpEnerPic * double (ctuArea.width * ctuArea.height);
        pic->ctuQpaLambda[ctuRsAddr] = hpEnerPic; // temporary backup of CTU mean visual activity
        pic->ctuAdaptedQP[ctuRsAddr] = pic->getOrigBuf (ctuArea).getAvg(); // and mean luma value

        if (!isEncPass)
        {
          updateMinNoiseLevelsPic( pic, bitDepth, pic->ctuAdaptedQP[ ctuRsAddr ], minActivityPart );
        }
      }
    }

    hpEnerAvg /= double (encCfg->m_SourceWidth * encCfg->m_SourceHeight);
  }

  hpEnerPic = 1.0 / getAveragePictureActivity (encCfg->m_PadSourceWidth, encCfg->m_PadSourceHeight, encCfg->m_RCNumPasses == 2 ? 0 : ctuPumpRedQP.back(),
                                               (encCfg->m_usePerceptQPATempFiltISlice || !slice->isIntra()), bitDepth);

  if (useFrameWiseQPA || (sliceQP >= MAX_QP))
  {
    int averageAdaptedLumaQP = (savedQP < 0 ? Clip3 (0, MAX_QP, slice->sliceQp + apprI3Log2 (hpEnerAvg * hpEnerPic)) : std::min (MAX_QP, savedQP + 1));

    if (isChromaEnabled (pic->chromaFormat) && (averageAdaptedLumaQP < MAX_QP) && (savedQP < 0))
    {
      averageAdaptedLumaQP += getGlaringColorQPOffset (pic, -1 /*ctuAddr*/, slice->sps->bitDepths[CH_C], meanLuma);

      if ((averageAdaptedLumaQP > MAX_QP) && !isHDR) averageAdaptedLumaQP = MAX_QP;
    }
    // change mean picture QP index depending on picture's average luma value (Sharp)
    if (isHDR && (savedQP < 0))
    {
      if (meanLuma == MAX_UINT) meanLuma = pic->getOrigBuf().Y().getAvg();

      averageAdaptedLumaQP = Clip3 (0, MAX_QP, averageAdaptedLumaQP + lumaDQPOffset (meanLuma, bitDepth));
    }

    hpEnerAvg = lambda * pow (2.0, double (averageAdaptedLumaQP - slice->sliceQp) / 3.0);

    for (uint32_t ctuRsAddr = ctuStartAddr; ctuRsAddr < ctuBoundingAddr; ctuRsAddr++)
    {
      pic->ctuQpaLambda[ctuRsAddr] = hpEnerAvg; // store adapted slice-level lambda
      pic->ctuAdaptedQP[ctuRsAddr] = (Pel) averageAdaptedLumaQP; // store fixed QPs
    }

    adaptedSliceQP = averageAdaptedLumaQP;
  }
  else // use CTU-wise QPA
  {
    const int nCtu = int (ctuBoundingAddr - ctuStartAddr);
    const int dvsr = encCfg->m_IntraPeriod - encCfg->m_GOPSize;

    if (isEncPass) ctuAvgLuma.resize (nCtu);

    for (uint32_t ctuRsAddr = ctuStartAddr; ctuRsAddr < ctuBoundingAddr; ctuRsAddr++)
    {
      int adaptedLumaQP = Clip3 (0, MAX_QP, slice->sliceQp + apprI3Log2 (pic->ctuQpaLambda[ctuRsAddr]/*hpEner*/ * hpEnerPic));

      if ((encCfg->m_usePerceptQPATempFiltISlice == 2) && slice->isIntra() && (ctuPumpRedQP.size() > ctuRsAddr))
      {
        if (ctuRCQPMemory != nullptr) // save I-frame QP for second rate control pass
        {
          if (ctuRsAddr & 1) ctuRCQPMemory->back() |= (Clip3 (-8, 7, ctuPumpRedQP[ctuRsAddr]) + 8) << 4;
          else /*even addr*/ ctuRCQPMemory->push_back (Clip3 (-8, 7, ctuPumpRedQP[ctuRsAddr]) + 8);
          if (adaptedLumaQP > 0)
          {
            adaptedLumaQP -= (sliceQP >> 4); // a first-pass tuning for stabilization
          }
        }
        if (ctuPumpRedQP[ctuRsAddr] < 0) adaptedLumaQP = Clip3 (0, MAX_QP, adaptedLumaQP + (ctuPumpRedQP[ctuRsAddr] * encCfg->m_GOPSize - (dvsr >> 1)) / dvsr);
        else /*ctuPumpRedQP[addr] >= 0*/ adaptedLumaQP = Clip3 (0, MAX_QP, adaptedLumaQP + (ctuPumpRedQP[ctuRsAddr] * encCfg->m_GOPSize + (dvsr >> 1)) / dvsr);

        ctuPumpRedQP[ctuRsAddr] = 0; // reset QP memory for temporal pumping analysis
      }
      if ((encCfg->m_usePerceptQPATempFiltISlice == 2) && !slice->isIntra() && (slice->TLayer == 0) && (ctuRCQPMemory != nullptr) && (adaptedLumaQP < MAX_QP))
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
      // reduce delta-QP variance, avoid wasting precious bit budget at low bit-rates
      if ((encCfg->m_RCTargetBitrate == 0) && (3 + encCfg->m_QP > MAX_QP_PERCEPT_QPA) && (savedQP >= 0) && (encCfg->m_framesToBeEncoded != 1))
      {
        const int retunedAdLumaQP = adaptedLumaQP + 1;

        adaptedLumaQP = (std::max (0, 1 + MAX_QP_PERCEPT_QPA - encCfg->m_QP) * adaptedLumaQP + std::min (4, 3 + encCfg->m_QP - MAX_QP_PERCEPT_QPA) * sliceQP + 2) >> 2;
        if (adaptedLumaQP > retunedAdLumaQP) adaptedLumaQP = retunedAdLumaQP;
        if (adaptedLumaQP < MAX_QP && encCfg->m_QP == MAX_QP_PERCEPT_QPA && slice->TLayer > 1) adaptedLumaQP++; // a fine-tuning
      }
      if (isEncPass) ctuAvgLuma[ctuRsAddr - ctuStartAddr] = pic->ctuAdaptedQP[ctuRsAddr];

      hpEnerAvg = lambda * pow (2.0, double (adaptedLumaQP - slice->sliceQp) / 3.0);

      pic->ctuQpaLambda[ctuRsAddr] = hpEnerAvg;  // store adapted CTU-level lambdas
      pic->ctuAdaptedQP[ctuRsAddr] = (Pel) adaptedLumaQP;  // store adapted CTU QPs

      adaptedSliceQP += adaptedLumaQP;

      // try to reduce local bitrate peaks using minimum smoothing of the adapted QPs
      if ((pcv.widthInCtus > 1) && (encCfg->m_PadSourceWidth <= 640 && encCfg->m_PadSourceHeight <= 480))
      {
        const uint32_t ctuIdxX = ctuRsAddr % pcv.widthInCtus; // horizontal CTU index

        if (ctuIdxX == 0)
        {
          adaptedLumaQP = (ctuRsAddr > 1 ? pic->ctuAdaptedQP[ctuRsAddr - 2] : 0);
        }
        else // ctuIdxX > 0
        {
          adaptedLumaQP = (ctuIdxX > 1 ? std::min (pic->ctuAdaptedQP[ctuRsAddr - 2], pic->ctuAdaptedQP[ctuRsAddr]) : pic->ctuAdaptedQP[ctuRsAddr]);
        }
        if (ctuRsAddr > pcv.widthInCtus) // previous vertical
        {
          adaptedLumaQP = std::min (adaptedLumaQP, (int) pic->ctuAdaptedQP[ctuRsAddr - 1 - pcv.widthInCtus]);
        }
        if ((ctuRsAddr > 0) && (pic->ctuAdaptedQP[ctuRsAddr - 1] < (Pel) adaptedLumaQP))
        {
          hpEnerAvg = lambda * pow (2.0, double (adaptedLumaQP - slice->sliceQp) / 3.0);

          pic->ctuQpaLambda[ctuRsAddr - 1] = hpEnerAvg;   // lambda after smoothing
          pic->ctuAdaptedQP[ctuRsAddr - 1] = (Pel) adaptedLumaQP; // min. smoothing
        }

        if ((ctuRsAddr == ctuBoundingAddr - 1) && (ctuRsAddr > pcv.widthInCtus)) // last CTU in the given slice
        {
          adaptedLumaQP = std::min (pic->ctuAdaptedQP[ctuRsAddr - 1], pic->ctuAdaptedQP[ctuRsAddr - pcv.widthInCtus]);

          if (pic->ctuAdaptedQP[ctuRsAddr] < (Pel) adaptedLumaQP)
          {
            hpEnerAvg = lambda * pow (2.0, double (adaptedLumaQP - slice->sliceQp) / 3.0);

            pic->ctuQpaLambda[ctuRsAddr] = hpEnerAvg;
            pic->ctuAdaptedQP[ctuRsAddr] = (Pel) adaptedLumaQP;
          }
        }
      }
    } // end CTU-wise loop

    adaptedSliceQP = (ctuBoundingAddr > ctuStartAddr ? (adaptedSliceQP < 0 ? adaptedSliceQP - ((nCtu + 1) >> 1) : adaptedSliceQP + ((nCtu + 1) >> 1)) / nCtu : sliceQP); // mean adapted luma QP

    if ((encCfg->m_RCTargetBitrate > 0 && adaptedSliceQP != slice->sliceQp) || (isEncPass)) // QP, rate control
    {
      const int rcQpDiff = (encCfg->m_RCTargetBitrate > 0 ? slice->sliceQp - adaptedSliceQP : 0);

      adaptedSliceQP = -1;
      for (uint32_t ctuRsAddr = ctuStartAddr; ctuRsAddr < ctuBoundingAddr; ctuRsAddr++)
      {
        int clippedLumaQP = pic->ctuAdaptedQP[ctuRsAddr] + rcQpDiff;

        if (isEncPass)
        {
          const double resRatio = double (encCfg->m_SourceWidth * encCfg->m_SourceHeight) / (3840.0 * 2160.0 * (1.0 + encCfg->m_RCTargetBitrate));

          meanLuma = ctuAvgLuma[ctuRsAddr - ctuStartAddr];
          clipQPValToEstimatedMinimStats (minNoiseLevels, bitDepth, meanLuma, resRatio, (encCfg->m_QP >> 3) + (slice->isIntra() ? encCfg->m_intraQPOffset >> 1 : slice->TLayer), clippedLumaQP);
        }
        clippedLumaQP = Clip3 (0, MAX_QP, clippedLumaQP);

        hpEnerAvg = lambda * pow (2.0, double (clippedLumaQP - slice->sliceQp) / 3.0);

        pic->ctuQpaLambda[ctuRsAddr] = hpEnerAvg; // store clipped CTU-level lambda
        pic->ctuAdaptedQP[ctuRsAddr] = (Pel) clippedLumaQP; // store clipped CTU QP

        adaptedSliceQP += clippedLumaQP;
      }

      pic->picInitialQP = Clip3 (0, MAX_QP, pic->picInitialQP + rcQpDiff); // used in applyQPAdaptationSubCtu()
      adaptedSliceQP = (ctuBoundingAddr > ctuStartAddr ? (adaptedSliceQP < 0 ? adaptedSliceQP - ((nCtu + 1) >> 1) : adaptedSliceQP + ((nCtu + 1) >> 1)) / nCtu : sliceQP); // mean final luma QP
    }
    else if ((3 + encCfg->m_QP > MAX_QP_PERCEPT_QPA) && (savedQP >= 0) && (encCfg->m_framesToBeEncoded != 1) && (adaptedSliceQP + 1 < sliceQP))
    {
      const int lrQpDiff = (sliceQP - adaptedSliceQP) >> (encCfg->m_QP <= MAX_QP_PERCEPT_QPA ? 2 : 1); // for monotonous rate change at low rates

      hpEnerAvg = pow (2.0, double (lrQpDiff) / 3.0);
      for (uint32_t ctuRsAddr = ctuStartAddr; ctuRsAddr < ctuBoundingAddr; ctuRsAddr++)
      {
        pic->ctuQpaLambda[ctuRsAddr] *= hpEnerAvg;
        pic->ctuAdaptedQP[ctuRsAddr] = (Pel) std::min (MAX_QP, pic->ctuAdaptedQP[ctuRsAddr] + lrQpDiff);
      }

      pic->picInitialQP = Clip3 (0, MAX_QP, pic->picInitialQP + lrQpDiff); // used in applyQPAdaptationSubCtu()
      adaptedSliceQP = sliceQP;
    }
    if (isEncPass) ctuAvgLuma.clear();
  }

  return adaptedSliceQP;
}

int BitAllocation::applyQPAdaptationSubCtu (const Slice* slice, const VVEncCfg* encCfg, const Area& lumaArea, const uint8_t* minNoiseLevels)
{
  Picture* const pic          = (slice != nullptr ? slice->pic : nullptr);
  double hpEnerPic, hpEnerSub = 0.0;
  int    adaptedSubCtuQP      = -1;
  uint32_t meanLuma           = MAX_UINT;

  if (pic == nullptr || encCfg == nullptr) return -1;

  const bool isEncPass        = (encCfg->m_LookAhead > 0 && !slice->pic->isPreAnalysis);
  const bool isHDR            = (encCfg->m_HdrMode != vvencHDRMode::VVENC_HDR_OFF) && !(encCfg->m_lumaReshapeEnable != 0 && encCfg->m_reshapeSignalType == RESHAPE_SIGNAL_PQ);
  const bool isHighResolution = (encCfg->m_PadSourceWidth > 2048 || encCfg->m_PadSourceHeight > 1280);
  const int         bitDepth  = slice->sps->bitDepths[CH_L];
  const PosType     guardSize = (isHighResolution ? 2 : 1);
  const Position    pos       = lumaArea.pos();
  const CompArea    subArea   = clipArea (CompArea (COMP_Y, pic->chromaFormat, Area (pos.x, pos.y, lumaArea.width, lumaArea.height)), pic->Y());
  const SizeType    fltWidth  = lumaArea.width  + guardSize * (pos.x > 0 ? 2 : 1);
  const SizeType    fltHeight = lumaArea.height + guardSize * (pos.y > 0 ? 2 : 1);
  const CompArea    fltArea   = clipArea (CompArea (COMP_Y, pic->chromaFormat, Area (pos.x > 0 ? pos.x - guardSize : 0, pos.y > 0 ? pos.y - guardSize : 0, fltWidth, fltHeight)), pic->Y());
  const CPelBuf     picOrig   = pic->getOrigBuf (fltArea);
  const CPelBuf     picPrv1   = pic->getOrigBufPrev (fltArea, PREV_FRAME_1);
  const CPelBuf     picPrv2   = pic->getOrigBufPrev (fltArea, PREV_FRAME_2);

  hpEnerSub = filterAndCalculateAverageActivity (picOrig.buf, picOrig.stride, picOrig.height, picOrig.width,
                                                 picPrv1.buf, picPrv1.stride, picPrv2.buf, picPrv2.stride, encCfg->m_FrameRate/encCfg->m_FrameScale,
                                                 bitDepth, isHighResolution);
  hpEnerPic = 1.0 / getAveragePictureActivity (encCfg->m_PadSourceWidth, encCfg->m_PadSourceHeight, 0,
                                               (encCfg->m_usePerceptQPATempFiltISlice || !slice->isIntra()), bitDepth);
  adaptedSubCtuQP = Clip3 (0, MAX_QP, pic->picInitialQP + apprI3Log2 (hpEnerSub * hpEnerPic));

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
  // reduce the delta-QP variance, avoid wasting precious bit budget at low bit-rates
  if ((encCfg->m_RCTargetBitrate == 0) && (3 + encCfg->m_QP > MAX_QP_PERCEPT_QPA) && (slice->sliceQp >= 0) && (encCfg->m_framesToBeEncoded != 1))
  {
    const int retunedAdLumaQP = adaptedSubCtuQP + 1;

    adaptedSubCtuQP = (std::max (0, 1 + MAX_QP_PERCEPT_QPA - encCfg->m_QP) * adaptedSubCtuQP + std::min (4, 3 + encCfg->m_QP - MAX_QP_PERCEPT_QPA) * slice->sliceQp + 2) >> 2;
    if (adaptedSubCtuQP > retunedAdLumaQP) adaptedSubCtuQP = retunedAdLumaQP;
    if (adaptedSubCtuQP < MAX_QP && encCfg->m_QP >= MAX_QP_PERCEPT_QPA) adaptedSubCtuQP++; // for monotonous rate change, l. 563
  }
  if (isEncPass)
  {
    const double resRatio = double (encCfg->m_SourceWidth * encCfg->m_SourceHeight) / (3840.0 * 2160.0 * (1.0 + encCfg->m_RCTargetBitrate));

    if (meanLuma == MAX_UINT) meanLuma = pic->getOrigBuf (subArea).getAvg();
    clipQPValToEstimatedMinimStats (minNoiseLevels, bitDepth, meanLuma, resRatio, (encCfg->m_QP >> 3) + (slice->isIntra() ? encCfg->m_intraQPOffset >> 1 : slice->TLayer), adaptedSubCtuQP);
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

double BitAllocation::getPicVisualActivity (const Slice* slice, const VVEncCfg* encCfg, const CPelBuf* origPrev /*= nullptr*/)
{
  Picture* const pic    = (slice != nullptr ? slice->pic : nullptr);

  if (pic == nullptr || encCfg == nullptr) return 0.0;

  const bool isHighRes  = (encCfg->m_PadSourceWidth > 2048 || encCfg->m_PadSourceHeight > 1280);
  const CPelBuf picOrig = pic->getOrigBuf (COMP_Y);
  const CPelBuf picPrv1 = (origPrev != nullptr ? *origPrev : pic->getOrigBufPrev (COMP_Y, PREV_FRAME_1));
  const CPelBuf picPrv2 = pic->getOrigBufPrev (COMP_Y, PREV_FRAME_2);

  return filterAndCalculateAverageActivity (picOrig.buf, picOrig.stride, picOrig.height, picOrig.width,
                                            picPrv1.buf, picPrv1.stride, picPrv2.buf, picPrv2.stride,
                                            (origPrev != nullptr ? 24 : encCfg->m_FrameRate/encCfg->m_FrameScale),
                                            slice->sps->bitDepths[CH_L], isHighRes);
}

bool BitAllocation::isTempLayer0IntraFrame (const Slice* slice, const VVEncCfg* encCfg, const PicList& picList, const bool rcIsFinalPass)
{
  Picture* const curPic = (slice != nullptr ? slice->pic : nullptr);

  if (curPic == nullptr || encCfg == nullptr) return false;

  const GOPEntry& gopEntry = *(slice->pic->gopEntry);
  const int curPoc         = slice->poc;
  const bool isHighRes     = (encCfg->m_PadSourceWidth > 2048 || encCfg->m_PadSourceHeight > 1280);

  curPic->picVisActTL0 = curPic->picVisActY = 0;

  if (((encCfg->m_LookAhead > 0 && encCfg->m_RCTargetBitrate == 0) || (encCfg->m_RCNumPasses > 1 && !rcIsFinalPass)) && !(slice->pps->sliceChromaQpFlag && encCfg->m_usePerceptQPA &&
      ((slice->isIntra() && !slice->sps->IBC) || (encCfg->m_sliceChromaQpOffsetPeriodicity > 0 && (curPoc % encCfg->m_sliceChromaQpOffsetPeriodicity) == 0))))
  {
    const double visActY = getPicVisualActivity (slice, encCfg);

    curPic->picVisActY   = ClipBD (uint16_t (0.5 + visActY), slice->sps->bitDepths[CH_L]);
  }

  if (encCfg->m_sliceTypeAdapt && curPoc >= 0 && encCfg->m_GOPSize > 8 && gopEntry.m_temporalId == 0)
  {
    const CPelBuf prvTL0 = curPic->getOrigBufPrev (COMP_Y, PREV_FRAME_TL0);
    const double visActY = (prvTL0.buf == nullptr ? 0.0 : getPicVisualActivity (slice, encCfg, &prvTL0));

    curPic->picVisActTL0 = ClipBD (uint16_t (0.5 + visActY), slice->sps->bitDepths[CH_L]);

    if (!slice->isIntra() && slice->getRefPic (REF_PIC_LIST_0, 0)) // detect scene change if comparison is possible
    {
      const Picture* refPic = slice->getRefPic (REF_PIC_LIST_0, 0);
      const int scThreshold = ((curPic->isSccStrong ? 6 : (curPic->isSccWeak ? 5 : 4)) * (isHighRes ? 19 : 15)) >> 2;

      if ((curPic->picVisActTL0 * 11 > refPic->picVisActTL0 * scThreshold ||
           refPic->picVisActTL0 * 11 > curPic->picVisActTL0 * (scThreshold + 1)) && refPic->picVisActTL0 > 0)
      {
        curPic->picMemorySTA = refPic->picVisActTL0 * (curPic->picVisActTL0 < refPic->picVisActTL0 ? -1 : 1);

        if (curPic->picMemorySTA * refPic->picMemorySTA >= 0) return true;
      }
    }
    curPic->picMemorySTA = 0;
  }

  return false;
}

} // namespace vvenc

//! \}

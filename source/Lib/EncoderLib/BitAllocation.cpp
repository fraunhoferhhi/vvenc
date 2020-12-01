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


/** \file     BitAllocation.cpp
\brief    Bit allocation class for QP adaptation and, possibly, rate control
*/

#include "BitAllocation.h"
#include "CommonLib/Picture.h"
#include <math.h>


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
                                                 uint32_t frameRate, const uint32_t bitDepth, const bool isUHD)
{
  double meanAct = 0.0;
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
    const int i2ndStride = iSrcStride * 2;
    const int i3rdStride = iSrcStride * 3;

    pSrc += iSrcStride;
    for (int y = 2; y < height - 2; y += 2)
    {
      for (int x = 2; x < width - 2; x += 2) // cnt cols
      {
        const int s = 12 * ((int) pSrc[x             ] + (int) pSrc[x+1           ] + (int) pSrc[x  +iSrcStride] + (int) pSrc[x+1+iSrcStride])
                     - 3 * ((int) pSrc[x-1           ] + (int) pSrc[x+2           ] + (int) pSrc[x-1+iSrcStride] + (int) pSrc[x+2+iSrcStride]
                          + (int) pSrc[x  -iSrcStride] + (int) pSrc[x+1-iSrcStride] + (int) pSrc[x  +i2ndStride] + (int) pSrc[x+1+i2ndStride])
                     - 2 * ((int) pSrc[x-1-iSrcStride] + (int) pSrc[x+2-iSrcStride] + (int) pSrc[x-1+i2ndStride] + (int) pSrc[x+2+i2ndStride])
                         - ((int) pSrc[x-1-i2ndStride] + (int) pSrc[x  -i2ndStride] + (int) pSrc[x+1-i2ndStride] + (int) pSrc[x+2-i2ndStride]
                          + (int) pSrc[x-1+i3rdStride] + (int) pSrc[x  +i3rdStride] + (int) pSrc[x+1+i3rdStride] + (int) pSrc[x+2+i3rdStride]
                          + (int) pSrc[x-2-iSrcStride] + (int) pSrc[x-2           ] + (int) pSrc[x-2+iSrcStride] + (int) pSrc[x-2+i2ndStride]
                          + (int) pSrc[x+3-iSrcStride] + (int) pSrc[x+3           ] + (int) pSrc[x+3+iSrcStride] + (int) pSrc[x+3+i2ndStride]);
        saAct += abs (s);
      }
      pSrc += i2ndStride;
    }

    meanAct = double (saAct) / double ((width - 4) * (height - 4));
  }
  else // HD high-pass without downsampling
  {
    for (int y = 1; y < height - 1; y++)
    {
      for (int x = 1; x < width - 1; x++) // center cols
      {
        const int s = 12 * (int) pSrc[x  ] - 2 * ((int) pSrc[x-1] + (int) pSrc[x+1] + (int) pSrc[x  -iSrcStride] + (int) pSrc[x  +iSrcStride])
                         - ((int) pSrc[x-1-iSrcStride] + (int) pSrc[x+1-iSrcStride] + (int) pSrc[x-1+iSrcStride] + (int) pSrc[x+1+iSrcStride]);
        saAct += abs (s);
      }
      pSrc += iSrcStride;
    }

    meanAct = double (saAct) / double ((width - 2) * (height - 2));
  }

  // skip first row as there may be a black border frame
  pSrc = pS0 + iSrcStride;
  // center rows
  if (isUHD) // high-pass with downsampling
  {
    const int i2M0Stride = iSrcStride * 2;
    const int i2M1Stride = iSM1Stride * 2;

    CHECK (pSM1 == nullptr || iSM1Stride <= 0 || iSM1Stride < width, "Pel buffer pointer pSM1 must not be null!");

    pSrc += iSrcStride;
    pSM1 += i2M1Stride;
    if (frameRate <= 31) // 1st-order delta
    {
      for (int y = 2; y < height - 2; y += 2)
      {
        for (int x = 2; x < width - 2; x += 2) // c cols
        {
          const int t = (int) pSrc[x] + (int) pSrc[x+1] + (int) pSrc[x+iSrcStride] + (int) pSrc[x+1+iSrcStride]
                     - ((int) pSM1[x] + (int) pSM1[x+1] + (int) pSM1[x+iSM1Stride] + (int) pSM1[x+1+iSM1Stride]);
          taAct += (1 + 3 * abs (t)) >> 1;
        }
        pSrc += i2M0Stride;
        pSM1 += i2M1Stride;
      }
    }
    else // 2nd-order delta (diff of diffs)
    {
      const int i2M2Stride = iSM2Stride * 2;

      CHECK (pSM2 == nullptr || iSM2Stride <= 0 || iSM2Stride < width, "Pel buffer pointer pSM2 must not be null!");

      pSM2 += i2M2Stride;
      for (int y = 2; y < height - 2; y += 2)
      {
        for (int x = 2; x < width - 2; x += 2) // c cols
        {
          const int t = (int) pSrc[x] + (int) pSrc[x+1] + (int) pSrc[x+iSrcStride] + (int) pSrc[x+1+iSrcStride]
                 - 2 * ((int) pSM1[x] + (int) pSM1[x+1] + (int) pSM1[x+iSM1Stride] + (int) pSM1[x+1+iSM1Stride])
                      + (int) pSM2[x] + (int) pSM2[x+1] + (int) pSM2[x+iSM2Stride] + (int) pSM2[x+1+iSM2Stride];
          taAct += abs (t);
        }
        pSrc += i2M0Stride;
        pSM1 += i2M1Stride;
        pSM2 += i2M2Stride;
      }
    }

    meanAct += (2.0 * taAct) / double ((width - 4) * (height - 4));
  }
  else // HD high-pass without downsampling
  {
    CHECK (pSM1 == nullptr || iSM1Stride <= 0 || iSM1Stride < width, "Pel buffer pointer pSM1 must not be null!");

    pSM1 += iSM1Stride;
    if (frameRate <= 31) // 1st-order delta
    {
      for (int y = 1; y < height - 1; y++)
      {
        for (int x = 1; x < width - 1; x++)  // cnt cols
        {
          const int t = (int) pSrc[x] - (int) pSM1[x];

          taAct += (1 + 3 * abs (t)) >> 1;
        }
        pSrc += iSrcStride;
        pSM1 += iSM1Stride;
      }
    }
    else // 2nd-order delta (diff of diffs)
    {
      CHECK (pSM2 == nullptr || iSM2Stride <= 0 || iSM2Stride < width, "Pel buffer pointer pSM2 must not be null!");

      pSM2 += iSM2Stride;
      for (int y = 1; y < height - 1; y++)
      {
        for (int x = 1; x < width - 1; x++)  // cnt cols
        {
          const int t = (int) pSrc[x] - 2 * (int) pSM1[x] + (int) pSM2[x];

          taAct += abs (t);
        }
        pSrc += iSrcStride;
        pSM1 += iSM1Stride;
        pSM2 += iSM2Stride;
      }
    }

    meanAct += (2.0 * taAct) / double ((width - 2) * (height - 2));
  }

  // lower limit, compensate for high-pass amplification
  return std::max (meanAct, double (1 << (bitDepth - 6)));
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

static int getLumaLevelBasedDeltaQP (const Pel avgLumaValue, const uint32_t bitDepth)
{
  if (bitDepth > 16 || avgLumaValue < 0 || avgLumaValue >= (1 << bitDepth)) return 0;

  const int meanLumaIndex = (bitDepth <= 10 ? avgLumaValue << (10 - bitDepth) : avgLumaValue >> (bitDepth - 10));

  if (meanLumaIndex >= 834) return -6; // Sharp's default curve with stepping of 66.67
  if (meanLumaIndex >= 767) return -5;
  if (meanLumaIndex >= 701) return -4;
  if (meanLumaIndex >= 634) return -3;
  if (meanLumaIndex >= 567) return -2;
  if (meanLumaIndex >= 501) return -1;
  if (meanLumaIndex >= 434) return  0;
  if (meanLumaIndex >= 367) return  1;
  if (meanLumaIndex >= 301) return  2;
  /* (meanLumaIndex >= 0) */return  3;
}

// public functions

int BitAllocation::applyQPAdaptationChroma (const Slice* slice, const EncCfg* encCfg, const int sliceQP, std::vector<int>& ctuPumpRedQP,
                                            int optChromaQPOffset[2], const bool isHDR /*= false*/)
{
  Picture* const pic          = (slice != nullptr ? slice->pic : nullptr);
  double hpEner[MAX_NUM_COMP] = {0.0, 0.0, 0.0};
  int    savedLumaQP          = -1;
  uint32_t meanLuma           = MAX_UINT;

  if (pic == nullptr || encCfg == nullptr || optChromaQPOffset == nullptr || encCfg->m_usePerceptQPA > 4) return -1;

  const bool isXPSNRBasedQPA  = (encCfg->m_usePerceptQPA & 1) == 0 && encCfg->m_RCNumPasses != 2;
  const bool isHighResolution = (encCfg->m_SourceWidth > 2048 || encCfg->m_SourceHeight > 1280) && ( encCfg->m_usePerceptQPA & 1 ) == 0;
  const int          bitDepth = slice->sps->bitDepths[CH_L];

  optChromaQPOffset[0] = optChromaQPOffset[1] = 0;

  for (uint32_t comp = 0; comp < getNumberValidComponents (pic->chromaFormat); comp++)
  {
    const ComponentID  compID = (ComponentID) comp;
    const CPelBuf     picOrig = pic->getOrigBuf (compID);
    const CPelBuf     picPrv1 = (isXPSNRBasedQPA ? pic->getOrigBufPrev (compID, false) : picOrig);
    const CPelBuf     picPrv2 = pic->getOrigBufPrev (compID, true );

    hpEner[comp] = filterAndCalculateAverageActivity (picOrig.buf, picOrig.stride, picOrig.height, picOrig.width,
                                                      picPrv1.buf, picPrv1.stride, (isXPSNRBasedQPA ? picPrv2.buf : nullptr), picPrv2.stride, encCfg->m_FrameRate,
                                                      bitDepth, isHighResolution && (isLuma (compID) || pic->chromaFormat == CHROMA_444));
    if (isChroma (compID))
    {
      const int adaptChromaQPOffset = 2.0 * hpEner[comp] <= hpEner[0] ? 0 : apprI3Log2 (2.0 * hpEner[comp] / hpEner[0]);

      if (savedLumaQP < 0)
      {
        int averageAdaptedLumaQP = ((encCfg->m_RCRateControlMode > 0) && (encCfg->m_RCRateControlMode < 3) ? Clip3 (0, MAX_QP, sliceQP) :
                                   Clip3 (0, MAX_QP, sliceQP + apprI3Log2 (hpEner[0] / getAveragePictureActivity (encCfg->m_SourceWidth, encCfg->m_SourceHeight,
                                                                                                                  encCfg->m_RCNumPasses == 2 ? 0 : ctuPumpRedQP.back(),
                                                                                                                  (encCfg->m_usePerceptQPATempFiltISlice || !slice->isIntra()) && isXPSNRBasedQPA, bitDepth))));
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

      const int lumaChromaMappingDQP = (savedLumaQP - slice->sps->chromaQpMappingTable.getMappedChromaQpValue (compID, savedLumaQP)) >> (encCfg->m_QP >= MAX_QP_PERCEPT_QPA ? 1 : 0);

      optChromaQPOffset[comp-1] = std::min (3 + lumaChromaMappingDQP, adaptChromaQPOffset + lumaChromaMappingDQP);
    } // isChroma (compID)
  }

  return savedLumaQP;
}

int BitAllocation::applyQPAdaptationLuma (const Slice* slice, const EncCfg* encCfg, const int savedQP, const double lambda, std::vector<int>& ctuPumpRedQP,
                                          const uint32_t ctuStartAddr, const uint32_t ctuBoundingAddr, const bool isHDR /*= false*/)
{
  Picture* const pic          = (slice != nullptr ? slice->pic : nullptr);
  double hpEnerPic, hpEnerAvg = 0.0;
  int    adaptedSliceQP       = -1;
  uint32_t meanLuma           = MAX_UINT;

  if (pic == nullptr || pic->cs == nullptr || encCfg == nullptr || ctuStartAddr >= ctuBoundingAddr) return -1;

  const bool isXPSNRBasedQPA  = (encCfg->m_usePerceptQPA & 1) == 0 && encCfg->m_RCNumPasses != 2;
  const bool isHighResolution = (encCfg->m_SourceWidth > 2048 || encCfg->m_SourceHeight > 1280) && ( encCfg->m_usePerceptQPA & 1 ) == 0;
  const bool useFrameWiseQPA  = (encCfg->m_QP > MAX_QP_PERCEPT_QPA);
  const int          bitDepth = slice->sps->bitDepths[CH_L];
  const int           sliceQP = (savedQP < 0 ? slice->sliceQp : savedQP);
  const PreCalcValues&    pcv = *pic->cs->pcv;

  if (!useFrameWiseQPA || (savedQP < 0)) // mean visual activity value and luma value in each CTU
  {
    const PosType guardSize = (isHighResolution ? 2 : 1);
    for (uint32_t ctuTsAddr = ctuStartAddr; ctuTsAddr < ctuBoundingAddr; ctuTsAddr++)
    {
      const uint32_t ctuRsAddr = /*tileMap.getCtuBsToRsAddrMap*/ (ctuTsAddr);
      const Position pos ((ctuRsAddr % pcv.widthInCtus) * pcv.maxCUSize, (ctuRsAddr / pcv.widthInCtus) * pcv.maxCUSize);
      const CompArea ctuArea   = clipArea (CompArea (COMP_Y, pic->chromaFormat, Area (pos.x, pos.y, pcv.maxCUSize, pcv.maxCUSize)), pic->Y());
      const SizeType fltWidth  = pcv.maxCUSize + guardSize * (pos.x > 0 ? 2 : 1);
      const SizeType fltHeight = pcv.maxCUSize + guardSize * (pos.y > 0 ? 2 : 1);
      const CompArea fltArea   = clipArea (CompArea (COMP_Y, pic->chromaFormat, Area (pos.x > 0 ? pos.x - guardSize : 0, pos.y > 0 ? pos.y - guardSize : 0, fltWidth, fltHeight)), pic->Y());
      const CPelBuf  picOrig   = pic->getOrigBuf (fltArea);
      const CPelBuf  picPrv1   = (isXPSNRBasedQPA ? pic->getOrigBufPrev (fltArea, false) : picOrig);
      const CPelBuf  picPrv2   = pic->getOrigBufPrev (fltArea, true );

      hpEnerPic = filterAndCalculateAverageActivity (picOrig.buf, picOrig.stride, picOrig.height, picOrig.width,
                                                     picPrv1.buf, picPrv1.stride, (isXPSNRBasedQPA ? picPrv2.buf : nullptr), picPrv2.stride, encCfg->m_FrameRate,
                                                     bitDepth, isHighResolution);
      hpEnerAvg += hpEnerPic;
      pic->ctuQpaLambda[ctuRsAddr] = hpEnerPic; // temporary backup of CTU mean visual activity
      pic->ctuAdaptedQP[ctuRsAddr] = pic->getOrigBuf (ctuArea).getAvg(); // and mean luma value
    }

    hpEnerAvg /= double (ctuBoundingAddr - ctuStartAddr);
  }

  if ((encCfg->m_RCRateControlMode > 0) && (encCfg->m_RCRateControlMode < 3) && (!useFrameWiseQPA || (savedQP < 0)))
  {
    hpEnerPic = 1.0 / hpEnerAvg;
  }
  else
  {
    hpEnerPic = 1.0 / getAveragePictureActivity (encCfg->m_SourceWidth, encCfg->m_SourceHeight, encCfg->m_RCNumPasses == 2 ? 0 : ctuPumpRedQP.back(),
                                                 (encCfg->m_usePerceptQPATempFiltISlice || !slice->isIntra()) && isXPSNRBasedQPA, bitDepth);
  }

  if (encCfg->m_usePerceptQPA > 4) // only set CTU-level QPs based on CTU mean luma value (Sharp)
  {
    CHECK (!isHDR, "HDR configuration error in perceptual QPA function");

    for (uint32_t ctuTsAddr = ctuStartAddr; ctuTsAddr < ctuBoundingAddr; ctuTsAddr++)
    {
      const uint32_t ctuRsAddr = /*tileMap.getCtuBsToRsAddrMap*/ (ctuTsAddr);
      const int  adaptedLumaQP = Clip3 (0, MAX_QP, slice->sliceQp + getLumaLevelBasedDeltaQP (pic->ctuAdaptedQP[ctuRsAddr], bitDepth));

      hpEnerAvg = lambda * pow (2.0, double (adaptedLumaQP - slice->sliceQp) / 3.0);

      pic->ctuQpaLambda[ctuRsAddr] = hpEnerAvg;  // store adapted CTU-level lambdas
      pic->ctuAdaptedQP[ctuRsAddr] = (Pel) adaptedLumaQP;  // store adapted CTU QPs
    }

    adaptedSliceQP = sliceQP;
  }
  else if (useFrameWiseQPA || (sliceQP >= MAX_QP))
  {
    int averageAdaptedLumaQP = (savedQP < 0 ? Clip3 (0, MAX_QP, slice->sliceQp + apprI3Log2 (hpEnerAvg * hpEnerPic)) : savedQP);

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

    for (uint32_t ctuTsAddr = ctuStartAddr; ctuTsAddr < ctuBoundingAddr; ctuTsAddr++)
    {
      const uint32_t ctuRsAddr = /*tileMap.getCtuBsToRsAddrMap*/ (ctuTsAddr);

      pic->ctuQpaLambda[ctuRsAddr] = hpEnerAvg; // store adapted slice-level lambda
      pic->ctuAdaptedQP[ctuRsAddr] = (Pel) averageAdaptedLumaQP; // store fixed QPs
    }

    adaptedSliceQP = averageAdaptedLumaQP;
  }
  else // use CTU-wise QPA
  {
    const int nCtu = int (ctuBoundingAddr - ctuStartAddr);

    for (uint32_t ctuTsAddr = ctuStartAddr; ctuTsAddr < ctuBoundingAddr; ctuTsAddr++)
    {
      const uint32_t ctuRsAddr = /*tileMap.getCtuBsToRsAddrMap*/ (ctuTsAddr);
      int adaptedLumaQP = Clip3 (0, MAX_QP, slice->sliceQp + apprI3Log2 (pic->ctuQpaLambda[ctuRsAddr]/*hpEner*/ * hpEnerPic));
      if (encCfg->m_usePerceptQPATempFiltISlice && slice->isIntra() && (ctuPumpRedQP.size() > ctuRsAddr))
      {
        adaptedLumaQP = Clip3 (0, MAX_QP, adaptedLumaQP + (ctuPumpRedQP[ctuRsAddr] * encCfg->m_GOPSize + ((encCfg->m_IntraPeriod - encCfg->m_GOPSize) >> 1)) / (encCfg->m_IntraPeriod - encCfg->m_GOPSize));
        ctuPumpRedQP[ctuRsAddr] = 0; // reset QP memory for temporal pumping analysis
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
      if ((3 + encCfg->m_QP > MAX_QP_PERCEPT_QPA) && (savedQP >= 0))
      {
        adaptedLumaQP = ((1 + MAX_QP_PERCEPT_QPA - encCfg->m_QP) * adaptedLumaQP + (3 + encCfg->m_QP - MAX_QP_PERCEPT_QPA) * sliceQP + 1) >> 2;
      }

      hpEnerAvg = lambda * pow (2.0, double (adaptedLumaQP - slice->sliceQp) / 3.0);

      pic->ctuQpaLambda[ctuRsAddr] = hpEnerAvg;  // store adapted CTU-level lambdas
      pic->ctuAdaptedQP[ctuRsAddr] = (Pel) adaptedLumaQP;  // store adapted CTU QPs

      adaptedSliceQP += adaptedLumaQP;

      // try to reduce local bitrate peaks using minimum smoothing of the adapted QPs
      if ((pcv.widthInCtus > 1) && (encCfg->m_SourceWidth <= 640 && encCfg->m_SourceHeight <= 480))
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

        if ((ctuTsAddr == ctuBoundingAddr - 1) && (ctuRsAddr > pcv.widthInCtus)) // last CTU in the given slice
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

    adaptedSliceQP = ((savedQP < 0) && (ctuBoundingAddr > ctuStartAddr) ? (adaptedSliceQP + ((nCtu + 1) >> 1)) / nCtu : sliceQP); // mean adapted luma QP
  }

  return adaptedSliceQP;
}

int BitAllocation::applyQPAdaptationSubCtu (const Slice* slice, const EncCfg* encCfg, const Area& lumaArea, const bool isHDR /*= false*/)
{
  Picture* const pic          = (slice != nullptr ? slice->pic : nullptr);
  double hpEnerPic, hpEnerSub = 0.0;
  int    adaptedSubCtuQP      = -1;
  uint32_t meanLuma           = MAX_UINT;

  if (pic == nullptr || encCfg == nullptr) return -1;

  const bool isXPSNRBasedQPA  = (encCfg->m_usePerceptQPA & 1) == 0 && encCfg->m_RCNumPasses != 2;
  const bool isHighResolution = (encCfg->m_SourceWidth > 2048 || encCfg->m_SourceHeight > 1280) && ( encCfg->m_usePerceptQPA & 1 ) == 0;
  const int         bitDepth  = slice->sps->bitDepths[CH_L];
  const PosType     guardSize = (isHighResolution ? 2 : 1);
  const Position    pos       = lumaArea.pos();
  const CompArea    subArea   = clipArea (CompArea (COMP_Y, pic->chromaFormat, Area (pos.x, pos.y, lumaArea.width, lumaArea.height)), pic->Y());
  const SizeType    fltWidth  = lumaArea.width  + guardSize * (pos.x > 0 ? 2 : 1);
  const SizeType    fltHeight = lumaArea.height + guardSize * (pos.y > 0 ? 2 : 1);
  const CompArea    fltArea   = clipArea (CompArea (COMP_Y, pic->chromaFormat, Area (pos.x > 0 ? pos.x - guardSize : 0, pos.y > 0 ? pos.y - guardSize : 0, fltWidth, fltHeight)), pic->Y());
  const CPelBuf     picOrig   = pic->getOrigBuf (fltArea);
  const CPelBuf     picPrv1   = (isXPSNRBasedQPA ? pic->getOrigBufPrev (fltArea, false) : picOrig);
  const CPelBuf     picPrv2   = pic->getOrigBufPrev (fltArea, true );

  if (encCfg->m_usePerceptQPA > 4) // only set CTU-level QPs based on CTU mean luma value (Sharp)
  {
    CHECK (!isHDR, "HDR configuration error in perceptual QPA function");

    return Clip3 (0, MAX_QP, pic->picInitialQP + getLumaLevelBasedDeltaQP (pic->getOrigBuf (subArea).getAvg(), bitDepth));
  }

  hpEnerSub = filterAndCalculateAverageActivity (picOrig.buf, picOrig.stride, picOrig.height, picOrig.width,
                                                 picPrv1.buf, picPrv1.stride, (isXPSNRBasedQPA ? picPrv2.buf : nullptr), picPrv2.stride, encCfg->m_FrameRate,
                                                 bitDepth, isHighResolution);
  hpEnerPic = 1.0 / getAveragePictureActivity (encCfg->m_SourceWidth, encCfg->m_SourceHeight, 0,
                                               (encCfg->m_usePerceptQPATempFiltISlice || !slice->isIntra()) && isXPSNRBasedQPA, bitDepth);
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

double BitAllocation::getPicVisualActivity (const Slice* slice, const EncCfg* encCfg, const PelBuf* origBuf /*= nullptr*/)
{
  Picture* const pic    = (slice != nullptr ? slice->pic : nullptr);

  if (pic == nullptr || encCfg == nullptr) return 0.0;

  const bool isXPSNRQPA = (encCfg->m_usePerceptQPA & 1) == 0 && encCfg->m_RCNumPasses != 2;
  const bool isHighRes  = ( encCfg->m_SourceWidth > 2048 || encCfg->m_SourceHeight > 1280 ) && ( encCfg->m_usePerceptQPA & 1 ) == 0;
  const CPelBuf picOrig = (origBuf != nullptr ? *origBuf : pic->getOrigBuf (COMP_Y));
  const CPelBuf picPrv1 = (isXPSNRQPA ? pic->getOrigBufPrev (COMP_Y, false) : picOrig);
  const CPelBuf picPrv2 = pic->getOrigBufPrev (COMP_Y, true );

  return filterAndCalculateAverageActivity (picOrig.buf, picOrig.stride, picOrig.height, picOrig.width,
                                            picPrv1.buf, picPrv1.stride, (isXPSNRQPA ? picPrv2.buf : nullptr), picPrv2.stride, encCfg->m_FrameRate,
                                            slice->sps->bitDepths[CH_L], isHighRes);
}

} // namespace vvenc

//! \}

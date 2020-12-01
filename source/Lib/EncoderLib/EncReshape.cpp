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

Copyright (c) 2019-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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


/** \file     EncReshape.cpp
\brief    encoder reshaper class
*/
#include "EncReshape.h"
#include "CommonLib/Picture.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
//! \ingroup EncLib
//! \{

namespace vvenc {

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

EncReshape::EncReshape()
  : m_exceedSTD     (false)
  , m_binImportance ()
  , m_tcase         (0)
  , m_rateAdpMode   (0)
  , m_useAdpCW      (false)
  , m_initCWAnalyze (0)
  , m_reshapeCW     ()
  , m_cwLumaWeight  {0}
  , m_chromaWeight  (1.0)
  , m_chromaAdj     (0)
  , m_binNum        (0)
  , m_srcSeqStats   ()
  , m_rspSeqStats   ()
{
  m_CTUFlag      = false;
  m_reshape      = true;
  m_exceedSTD    = false;
  m_signalType   = RESHAPE_SIGNAL_NULL;
}

EncReshape::~EncReshape()
{
}

void  EncReshape::init( const EncCfg& encCfg )
{
  if ( encCfg.m_lumaReshapeEnable )
  {
    m_lumaBD = encCfg.m_internalBitDepth[ CH_L ];
    m_reshapeLUTSize = 1 << m_lumaBD;
    m_initCWAnalyze = m_reshapeLUTSize / PIC_ANALYZE_CW_BINS;
    m_initCW = m_reshapeLUTSize / PIC_CODE_CW_BINS;

    if (m_fwdLUT.empty())
      m_fwdLUT.resize(m_reshapeLUTSize, 0);
    if (m_invLUT.empty())
      m_invLUT.resize(m_reshapeLUTSize,0);
    if (m_binCW.empty())
      m_binCW.resize(PIC_ANALYZE_CW_BINS);
    if (m_binImportance.empty())
      m_binImportance.resize(PIC_ANALYZE_CW_BINS);
    if (m_reshapePivot.empty())
      m_reshapePivot.resize(PIC_CODE_CW_BINS + 1, 0);
    if (m_inputPivot.empty())
      m_inputPivot.resize(PIC_CODE_CW_BINS + 1, 0);
    if (m_fwdScaleCoef.empty())
      m_fwdScaleCoef.resize(PIC_CODE_CW_BINS, 1 << FP_PREC);
    if (m_invScaleCoef.empty())
      m_invScaleCoef.resize(PIC_CODE_CW_BINS, 1 << FP_PREC);
    if (m_chromaAdjHelpLUT.empty())
      m_chromaAdjHelpLUT.resize(PIC_CODE_CW_BINS, 1<<CSCALE_FP_PREC);

    m_sliceReshapeInfo.sliceReshaperEnabled      = true;
    m_sliceReshapeInfo.sliceReshaperModelPresent = true;
    m_sliceReshapeInfo.enableChromaAdj           = true;
    m_sliceReshapeInfo.reshaperModelMinBinIdx    = 0;
    m_sliceReshapeInfo.reshaperModelMaxBinIdx    = PIC_CODE_CW_BINS - 1;
    memset(m_sliceReshapeInfo.reshaperModelBinCWDelta, 0, (PIC_CODE_CW_BINS) * sizeof(int));
    m_sliceReshapeInfo.maxNbitsNeededDeltaCW     = 0;
    m_sliceReshapeInfo.chrResScalingOffset       = 0;

    m_binNum = PIC_CODE_CW_BINS;
    m_srcSeqStats = SeqInfo();
    m_rspSeqStats = SeqInfo();

    m_signalType = encCfg.m_reshapeSignalType;
    m_chromaWeight = 1.0;

    initLumaLevelToWeightTableReshape();
  }
  else if (encCfg.m_lumaLevelToDeltaQPEnabled )
  {
    m_lumaBD = encCfg.m_internalBitDepth[ CH_L ];
    m_reshapeLUTSize = 1 << m_lumaBD;
    m_initCWAnalyze = m_reshapeLUTSize / PIC_ANALYZE_CW_BINS;
    m_initCW = m_reshapeLUTSize / PIC_CODE_CW_BINS;
    m_signalType = RESHAPE_SIGNAL_PQ;
    m_chromaWeight = 1.0;

    initLumaLevelToWeightTableReshape();
  }
}

void  EncReshape::destroy()
{
}

/**
-Perform HDR set up
\param   pcPic describe pointer of current coding picture
\param   sliceType describe the slice type
*/
void EncReshape::preAnalyzerHDR(Picture& pic, const SliceType sliceType, const ReshapeCW& reshapeCW, bool isDualT)
{
  if (m_lumaBD >= 10)
  {
    m_sliceReshapeInfo.sliceReshaperEnabled = true;
      if (sliceType == I_SLICE )                                              { m_sliceReshapeInfo.sliceReshaperModelPresent = true;  }
      else                                                                    { m_sliceReshapeInfo.sliceReshaperModelPresent = false; }
    { m_sliceReshapeInfo.enableChromaAdj = 1;                   }
  }
  else
  {
    m_sliceReshapeInfo.sliceReshaperEnabled = false;
    m_sliceReshapeInfo.sliceReshaperModelPresent = false;
  }
}

/**
-Perform picture analysis for SDR
\param   pcPic describe pointer of current coding picture
\param   sliceType describe the slice type
\param   reshapeCW describe some input info
*/

void EncReshape::calcSeqStats(Picture& pic, SeqInfo &stats)
{
  CPelBuf picY     = pic.getOrigBuf(COMP_Y);
  const int width  = picY.width;
  const int height = picY.height;
  const int stride = picY.stride;
  uint32_t winLens = (m_binNum == PIC_CODE_CW_BINS) ? (std::min(height, width) / 240) : 2;
  winLens = winLens > 0 ? winLens : 1;

  int64_t tempSq  = 0;
  int64_t topSum  = 0,  topSumSq = 0;
  int64_t leftSum = 0, leftSumSq = 0;
  int64_t *leftColSum   = new int64_t[width];
  int64_t *leftColSumSq = new int64_t[width];
  int64_t *topRowSum    = new int64_t[height];
  int64_t *topRowSumSq  = new int64_t[height];
  int64_t *topColSum    = new int64_t[width];
  int64_t *topColSumSq  = new int64_t[width];
  uint32_t *binCnt      = new uint32_t[m_binNum];
  memset(leftColSum,   0, width * sizeof(int64_t));
  memset(leftColSumSq, 0, width * sizeof(int64_t));
  memset(topRowSum,    0, height * sizeof(int64_t));
  memset(topRowSumSq,  0, height * sizeof(int64_t));
  memset(topColSum,    0, width * sizeof(int64_t));
  memset(topColSumSq,  0, width * sizeof(int64_t));
  memset(binCnt,       0, m_binNum * sizeof(uint32_t));

  stats = SeqInfo();
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      const Pel pxlY = picY.buf[x];
      int64_t sum = 0, sumSq = 0;
      uint32_t numPixInPart = 0;
      uint32_t y1 = std::max((int)(y - winLens), 0);
      uint32_t y2 = std::min((int)(y + winLens), (height - 1));
      uint32_t x1 = std::max((int)(x - winLens), 0);
      uint32_t x2 = std::min((int)(x + winLens), (width - 1));
      uint32_t bx = 0, by = 0;
      const Pel* pWinY = &picY.buf[0];
      numPixInPart = (x2 - x1 + 1) * (y2 - y1 + 1);

      if (x == 0 && y == 0)
      {
        for (by = y1; by <= y2; by++)
        {
          for (bx = x1; bx <= x2; bx++)
          {
            tempSq = pWinY[bx] * pWinY[bx];
            leftSum += pWinY[bx];
            leftSumSq += tempSq;
            leftColSum[bx] += pWinY[bx];
            leftColSumSq[bx] += tempSq;
            topColSum[bx] += pWinY[bx];
            topColSumSq[bx] += tempSq;
            topRowSum[by] += pWinY[bx];
            topRowSumSq[by] += tempSq;
          }
          pWinY += stride;
        }
        topSum = leftSum;
        topSumSq = leftSumSq;
        sum = leftSum;
        sumSq = leftSumSq;
      }
      else if (x == 0 && y > 0)
      {
        if (y < height - winLens)
        {
          pWinY += winLens*stride;
          topRowSum[y + winLens] = 0;
          topRowSumSq[y + winLens] = 0;
          for (bx = x1; bx <= x2; bx++)
          {
            topRowSum[y + winLens] += pWinY[bx];
            topRowSumSq[y + winLens] += pWinY[bx] * pWinY[bx];
          }
          topSum += topRowSum[y + winLens];
          topSumSq += topRowSumSq[y + winLens];
        }
        if (y > winLens)
        {
          topSum -= topRowSum[y - 1 - winLens];
          topSumSq -= topRowSumSq[y - 1 - winLens];
        }
        memset(leftColSum, 0, width * sizeof(int64_t));
        memset(leftColSumSq, 0, width * sizeof(int64_t));
        pWinY = &picY.buf[0];
        pWinY -= (y <= winLens ? y : winLens)*stride;
        for (by = y1; by <= y2; by++)
        {
          for (bx = x1; bx <= x2; bx++)
          {
            leftColSum[bx] += pWinY[bx];
            leftColSumSq[bx] += pWinY[bx] * pWinY[bx];
          }
          pWinY += stride;
        }
        leftSum = topSum;
        leftSumSq = topSumSq;
        sum = topSum;
        sumSq = topSumSq;
      }
      else if (x > 0)
      {
        if (x < width - winLens)
        {
          pWinY -= (y <= winLens ? y : winLens)*stride;
          if (y == 0)
          {
            leftColSum[x + winLens] = 0;
            leftColSumSq[x + winLens] = 0;
            for (by = y1; by <= y2; by++)
            {
              leftColSum[x + winLens] += pWinY[x + winLens];
              leftColSumSq[x + winLens] += pWinY[x + winLens] * pWinY[x + winLens];
              pWinY += stride;
            }
          }
          else
          {
            leftColSum[x + winLens] = topColSum[x + winLens];
            leftColSumSq[x + winLens] = topColSumSq[x + winLens];
            if (y < height - winLens)
            {
              pWinY = &picY.buf[0];
              pWinY += winLens * stride;
              leftColSum[x + winLens] += pWinY[x + winLens];
              leftColSumSq[x + winLens] += pWinY[x + winLens] * pWinY[x + winLens];
            }
            if (y > winLens)
            {
              pWinY = &picY.buf[0];
              pWinY -= (winLens + 1) * stride;
              leftColSum[x + winLens] -= pWinY[x + winLens];
              leftColSumSq[x + winLens] -= pWinY[x + winLens] * pWinY[x + winLens];
            }
          }
          topColSum[x + winLens] = leftColSum[x + winLens];
          topColSumSq[x + winLens] = leftColSumSq[x + winLens];
          leftSum += leftColSum[x + winLens];
          leftSumSq += leftColSumSq[x + winLens];
        }
        if (x > winLens)
        {
          leftSum -= leftColSum[x - 1 - winLens];
          leftSumSq -= leftColSumSq[x - 1 - winLens];
        }
        sum = leftSum;
        sumSq = leftSumSq;
      }

      double average = double(sum) / numPixInPart;
      double variance = double(sumSq) / numPixInPart - average * average;
      int binLen = m_reshapeLUTSize / m_binNum;
      uint32_t binIdx = (uint32_t)(pxlY / binLen);
      average = average / (double)(1 << (m_lumaBD - 10));
      variance = variance / (double)(1 << (2 * (m_lumaBD - 10)));
      binIdx = (uint32_t)((pxlY >> (m_lumaBD - 10)) / binLen);
      double varLog10 = log10(variance + 1.0);
      stats.binVar[binIdx] += varLog10;
      binCnt[binIdx]++;
    }
    picY.buf += stride;
  }

  for (int b = 0; b < m_binNum; b++)
  {
    stats.binHist[b] = (double)binCnt[b] / (double)(m_reshapeCW.rspPicSize);
    stats.binVar[b] = (binCnt[b] > 0) ? (stats.binVar[b] / binCnt[b]) : 0.0;
  }
  delete[] binCnt;
  delete[] topColSum;
  delete[] topColSumSq;
  delete[] topRowSum;
  delete[] topRowSumSq;
  delete[] leftColSum;
  delete[] leftColSumSq;

  stats.minBinVar = 5.0;
  stats.maxBinVar = 0.0;
  stats.meanBinVar = 0.0;
  stats.nonZeroCnt = 0;
  for (int b = 0; b < m_binNum; b++)
  {
    if (stats.binHist[b] > 0.001)
    {
      stats.nonZeroCnt++;
      stats.meanBinVar += stats.binVar[b];
      if (stats.binVar[b] > stats.maxBinVar) { stats.maxBinVar = stats.binVar[b]; }
      if (stats.binVar[b] < stats.minBinVar) { stats.minBinVar = stats.binVar[b]; }
    }
  }
  stats.meanBinVar /= (double)stats.nonZeroCnt;
  for (int b = 0; b < m_binNum; b++)
  {
    if (stats.meanBinVar > 0.0)
      stats.normVar[b] = stats.binVar[b] / stats.meanBinVar;
    stats.weightVar += stats.binHist[b] * stats.binVar[b];
    stats.weightNorm += stats.binHist[b] * stats.normVar[b];
  }

  picY = pic.getOrigBuf(COMP_Y);
  CPelBuf picU = pic.getOrigBuf(COMP_Cb);
  CPelBuf picV = pic.getOrigBuf(COMP_Cr);
  const int widthC = picU.width;
  const int heightC = picU.height;
  const int strideC = picU.stride;
  double avgY = 0.0, avgU = 0.0, avgV = 0.0;
  double varY = 0.0, varU = 0.0, varV = 0.0;
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      avgY += picY.buf[x];
      varY += picY.buf[x] * picY.buf[x];
    }
    picY.buf += stride;
  }
  for (int y = 0; y < heightC; y++)
  {
    for (int x = 0; x < widthC; x++)
    {
      avgU += picU.buf[x];
      avgV += picV.buf[x];
      varU += picU.buf[x] * picU.buf[x];
      varV += picV.buf[x] * picV.buf[x];
    }
    picU.buf += strideC;
    picV.buf += strideC;
  }
  avgY = avgY / (width * height);
  avgU = avgU / (widthC * heightC);
  avgV = avgV / (widthC * heightC);
  varY = varY / (width * height) - avgY * avgY;
  varU = varU / (widthC * heightC) - avgU * avgU;
  varV = varV / (widthC * heightC) - avgV * avgV;
  if (varY > 0)
  {
    stats.ratioStdU = sqrt(varU) / sqrt(varY);
    stats.ratioStdV = sqrt(varV) / sqrt(varY);
  }
}

void EncReshape::preAnalyzerLMCS(Picture& pic, const uint32_t signalType, const SliceType sliceType, const ReshapeCW& reshapeCW)
{
  m_sliceReshapeInfo.sliceReshaperModelPresent = true;
  m_sliceReshapeInfo.sliceReshaperEnabled = true;
  int modIP = pic.getPOC() - pic.getPOC() / reshapeCW.rspFpsToIp * reshapeCW.rspFpsToIp;
  if (sliceType == I_SLICE || (reshapeCW.updateCtrl == 2 && modIP == 0))
  {
    if (m_sliceReshapeInfo.sliceReshaperModelPresent == true)
    {
      m_reshapeCW = reshapeCW;
      m_binNum = PIC_CODE_CW_BINS;
      int stdMin = 16 << (m_lumaBD - 8);
      int stdMax = 235 << (m_lumaBD - 8);
      int binLen = m_reshapeLUTSize / m_binNum;
      int startBinIdx = stdMin / binLen;
      int endBinIdx = stdMax / binLen;
      m_sliceReshapeInfo.reshaperModelMinBinIdx = startBinIdx;
      m_sliceReshapeInfo.reshaperModelMaxBinIdx = endBinIdx;
      m_initCWAnalyze = m_lumaBD > 10 ? (binLen >> (m_lumaBD - 10)) : m_lumaBD < 10 ? (binLen << (10 - m_lumaBD)) : binLen;
      for (int b = 0; b < m_binNum; b++) { m_binCW[b] = m_initCWAnalyze; }

      m_reshape = true;
      m_useAdpCW = false;
      m_exceedSTD = false;
      m_chromaWeight = 1.0;
      m_sliceReshapeInfo.enableChromaAdj = 1;
      m_rateAdpMode = 0;  m_tcase = 0;
      bool intraAdp = true, interAdp = true;

      calcSeqStats(pic, m_srcSeqStats);
      if (m_binNum == PIC_CODE_CW_BINS)
      {
        if ((m_srcSeqStats.binHist[0] + m_srcSeqStats.binHist[m_binNum - 1]) > 0.005) { m_exceedSTD = true; }
        if (m_srcSeqStats.binHist[m_binNum - 1] > 0.0003) { intraAdp = false;  interAdp = false; }
        if (m_srcSeqStats.binHist[0] > 0.03) { intraAdp = false;  interAdp = false; }
      }
      else if (m_binNum == PIC_ANALYZE_CW_BINS)
      {
        if ((m_srcSeqStats.binHist[0] + m_srcSeqStats.binHist[1] + m_srcSeqStats.binHist[m_binNum - 2] + m_srcSeqStats.binHist[m_binNum - 1]) > 0.01) { m_exceedSTD = true; }
        if ((m_srcSeqStats.binHist[m_binNum - 2] + m_srcSeqStats.binHist[m_binNum - 1]) > 0.0003) { intraAdp = false;  interAdp = false; }
        if ((m_srcSeqStats.binHist[0] + m_srcSeqStats.binHist[1]) > 0.03) { intraAdp = false;  interAdp = false; }
      }
      if (m_exceedSTD)
      {
        for (int i = 0; i < m_binNum; i++)
        {
          if (m_srcSeqStats.binHist[i] > 0 && i < startBinIdx) { startBinIdx = i; }
          if (m_srcSeqStats.binHist[i] > 0 && i > endBinIdx) { endBinIdx = i; }
        }
        m_sliceReshapeInfo.reshaperModelMinBinIdx = startBinIdx;
        m_sliceReshapeInfo.reshaperModelMaxBinIdx = endBinIdx;
      }

      if ((m_srcSeqStats.ratioStdU + m_srcSeqStats.ratioStdV) > 1.5 && m_srcSeqStats.binHist[1] > 0.5) { intraAdp = false;  interAdp = false; }
      if (m_srcSeqStats.ratioStdU > 0.36 && m_srcSeqStats.ratioStdV > 0.2 && m_reshapeCW.rspPicSize > 5184000)
      {
        m_sliceReshapeInfo.enableChromaAdj = 0; m_chromaWeight = 1.05;
        if ((m_srcSeqStats.ratioStdU + m_srcSeqStats.ratioStdV) < 0.69) { m_chromaWeight = 0.95; }
      }

      if (interAdp)
      {
        if (m_reshapeCW.adpOption)
        {
          m_reshapeCW.binCW[0] = 0; m_reshapeCW.binCW[1] = m_reshapeCW.initialCW;
          m_rateAdpMode = m_reshapeCW.adpOption - 2 * (m_reshapeCW.adpOption / 2);
          if (m_reshapeCW.adpOption == 2) { m_tcase = 9; }
          else if (m_reshapeCW.adpOption > 2) { intraAdp = false; }
        }
        else if (signalType == RESHAPE_SIGNAL_SDR)
        {
          m_reshapeCW.binCW[0] = 0; m_reshapeCW.binCW[1] = 1022;
          deriveReshapeParametersSDR(&intraAdp, &interAdp);
        }
        else if (signalType == RESHAPE_SIGNAL_HLG)
        {
          if (m_reshapeCW.updateCtrl == 0)
          {
            m_rateAdpMode = 0;  m_tcase = 9;
            m_reshapeCW.binCW[1] = 952;
            if (m_srcSeqStats.meanBinVar < 2.5) { m_reshapeCW.binCW[1] = 840; }
          }
          else
          {
            m_useAdpCW = true;
            m_rateAdpMode = 2;
            if (m_binNum == PIC_CODE_CW_BINS) { m_reshapeCW.binCW[0] = 72;  m_reshapeCW.binCW[1] = 58; }
            else if (m_binNum == PIC_ANALYZE_CW_BINS) { m_reshapeCW.binCW[0] = 36;  m_reshapeCW.binCW[1] = 30; }
            if (m_srcSeqStats.meanBinVar < 2.5) { intraAdp = false; interAdp = false; }
          }
        }
      }

      if (m_rateAdpMode == 2 && reshapeCW.rspBaseQP <= 22) { intraAdp = false; interAdp = false; }
      m_sliceReshapeInfo.sliceReshaperEnabled = intraAdp;
      if (!intraAdp && !interAdp)
      {
        m_sliceReshapeInfo.sliceReshaperModelPresent = false;
        m_reshape = false;
        return;
      }

      if (m_rateAdpMode == 1 && reshapeCW.rspBaseQP <= 22)
      {
        for (int i = 0; i < m_binNum; i++)
        {
          if (i >= startBinIdx && i <= endBinIdx) { m_binCW[i] = m_initCWAnalyze + 2; }
          else { m_binCW[i] = 0; }
        }
      }
      else if (m_useAdpCW)
      {
        if (signalType == RESHAPE_SIGNAL_SDR && m_reshapeCW.updateCtrl == 2)
        {
          m_binNum = PIC_ANALYZE_CW_BINS;
          startBinIdx = startBinIdx * 2;
          endBinIdx = endBinIdx * 2 + 1;
          calcSeqStats(pic, m_srcSeqStats);
        }
        double alpha = 1.0, beta = 0.0;
        deriveReshapeParameters(m_srcSeqStats.binVar, startBinIdx, endBinIdx, m_reshapeCW, alpha, beta);
        for (int i = 0; i < m_binNum; i++)
        {
          if (i >= startBinIdx && i <= endBinIdx) { m_binCW[i] = (uint32_t)round(alpha*m_srcSeqStats.binVar[i] + beta); }
          else { m_binCW[i] = 0; }
        }
      }
      else
      {
        cwPerturbation(startBinIdx, endBinIdx, (uint16_t)m_reshapeCW.binCW[1]);
      }
      cwReduction(startBinIdx, endBinIdx);
    }
    m_chromaAdj = m_sliceReshapeInfo.enableChromaAdj;
  }
  else // Inter slices
  {
    m_sliceReshapeInfo.sliceReshaperModelPresent = false;
    m_sliceReshapeInfo.enableChromaAdj = m_chromaAdj;
    if (!m_reshape) { m_sliceReshapeInfo.sliceReshaperEnabled = false; }
    else
    {
      const int cTid = m_reshapeCW.rspTid;
      bool enableRsp = m_tcase == 5 ? false : (m_tcase < 5 ? (cTid < m_tcase + 1 ? false : true) : (cTid <= 10 - m_tcase ? true : false));
      m_sliceReshapeInfo.sliceReshaperEnabled = enableRsp;
    }
  }
}

// Bubble Sort to  descending order with index
void EncReshape::bubbleSortDsd(double* array, int * idx, int n)
{
  int i, j;
  bool swapped;
  for (i = 0; i < n - 1; i++)
  {
    swapped = false;
    for (j = 0; j < n - i - 1; j++)
    {
      if (array[j] < array[j + 1])
      {
        std::swap(array[j], array[j + 1]);
        std::swap(idx[j], idx[j + 1]);
        swapped = true;
      }
    }
    if (swapped == false)
      break;
  }
}

void EncReshape::cwPerturbation(int startBinIdx, int endBinIdx, uint16_t maxCW)
{
  for (int i = 0; i < m_binNum; i++)
  {
    if (i >= startBinIdx && i <= endBinIdx) { m_binCW[i] = (uint32_t)round((double)maxCW / (endBinIdx - startBinIdx + 1)); }
    else { m_binCW[i] = 0; }
  }

  double hist = 0.0;
  uint16_t delta1 = 0, delta2 = 0;
  for (int i = 0; i < m_binNum; i++)
  {
    if (m_srcSeqStats.binHist[i] > 0.001)
    {
      hist = m_srcSeqStats.binHist[i] > 0.4 ? 0.4 : m_srcSeqStats.binHist[i];
      delta1 = (uint16_t)(10.0 * hist + 0.5);
      delta2 = (uint16_t)(20.0 * hist + 0.5);
      if (m_srcSeqStats.normVar[i] < 0.8) { m_binCW[i] = m_binCW[i] + delta2; }
      else if (m_srcSeqStats.normVar[i] < 0.9) { m_binCW[i] = m_binCW[i] + delta1; }
      if (m_srcSeqStats.normVar[i] > 1.2) { m_binCW[i] = m_binCW[i] - delta2; }
      else if (m_srcSeqStats.normVar[i] > 1.1) { m_binCW[i] = m_binCW[i] - delta1; }
    }
  }
}

void EncReshape::cwReduction(int startBinIdx, int endBinIdx)
{
  int bdShift = m_lumaBD - 10;
  int totCW = bdShift != 0 ? (bdShift > 0 ? m_reshapeLUTSize / (1 << bdShift) : m_reshapeLUTSize * (1 << (-bdShift))) : m_reshapeLUTSize;
  int maxAllowedCW = totCW - 1, usedCW = 0;
  for (int i = 0; i < m_binNum; i++) { usedCW += m_binCW[i]; }
  if (usedCW > maxAllowedCW)
  {
    int deltaCW = usedCW - maxAllowedCW;
    int divCW = deltaCW / (endBinIdx - startBinIdx + 1);
    int modCW = deltaCW - divCW * (endBinIdx - startBinIdx + 1);
    if (divCW > 0)
    {
      for (int i = startBinIdx; i <= endBinIdx; i++) { m_binCW[i] -= divCW; }
    }
    for (int i = startBinIdx; i <= endBinIdx; i++)
    {
      if (modCW == 0)  break;
      if (m_binCW[i] > 0) { m_binCW[i]--; modCW--; }
    }
  }
}

void EncReshape::deriveReshapeParametersSDR(bool *intraAdp, bool *interAdp)
{
  bool   isSkipCase = false;
  bool   isLowCase = false;
  int    firstBinVarLessThanVal1 = 0;
  int    firstBinVarLessThanVal2 = 0;
  int    firstBinVarLessThanVal3 = 0;
  double percBinVarLessThenVal1 = 0.0;
  double percBinVarLessThenVal2 = 0.0;
  double percBinVarLessThenVal3 = 0.0;
  int    *binIdxSortDsd = new int[m_binNum];
  double *binVarSortDsd = new double[m_binNum];
  double *binVarSortDsdCDF = new double[m_binNum];
  double ratioWeiVar = 0.0, ratioWeiVarNorm = 0.0;
  int startBinIdx = m_sliceReshapeInfo.reshaperModelMinBinIdx;
  int endBinIdx = m_sliceReshapeInfo.reshaperModelMaxBinIdx;

  for (int b = 0; b < m_binNum; b++)
  {
    binVarSortDsd[b] = m_srcSeqStats.binVar[b];
    binIdxSortDsd[b] = b;
  }
  bubbleSortDsd(binVarSortDsd, binIdxSortDsd, m_binNum);
  binVarSortDsdCDF[0] = m_srcSeqStats.binHist[binIdxSortDsd[0]];
  for (int b = 1; b < m_binNum; b++) { binVarSortDsdCDF[b] = binVarSortDsdCDF[b - 1] + m_srcSeqStats.binHist[binIdxSortDsd[b]]; }
  for (int b = 0; b < m_binNum - 1; b++)
  {
    if (binVarSortDsd[b] > 3.4) { firstBinVarLessThanVal1 = b + 1; }
    if (binVarSortDsd[b] > 2.8) { firstBinVarLessThanVal2 = b + 1; }
    if (binVarSortDsd[b] > 2.5) { firstBinVarLessThanVal3 = b + 1; }
  }
  percBinVarLessThenVal1 = binVarSortDsdCDF[firstBinVarLessThanVal1];
  percBinVarLessThenVal2 = binVarSortDsdCDF[firstBinVarLessThanVal2];
  percBinVarLessThenVal3 = binVarSortDsdCDF[firstBinVarLessThanVal3];
  delete[] binIdxSortDsd;
  delete[] binVarSortDsd;
  delete[] binVarSortDsdCDF;

  cwPerturbation(startBinIdx, endBinIdx, (uint16_t)m_reshapeCW.binCW[1]);
  cwReduction(startBinIdx, endBinIdx);
  m_rspSeqStats = SeqInfo();
  for (int b = 0; b < m_binNum; b++)
  {
    double scale = (m_binCW[b] > 0) ? ((double)m_binCW[b] / (double)m_initCWAnalyze) : 1.0;
    m_rspSeqStats.binHist[b] = m_srcSeqStats.binHist[b];
    m_rspSeqStats.binVar[b] = m_srcSeqStats.binVar[b] + 2.0 * log10(scale);
  }
  m_rspSeqStats.minBinVar = 5.0;
  m_rspSeqStats.maxBinVar = 0.0;
  m_rspSeqStats.meanBinVar = 0.0;
  m_rspSeqStats.nonZeroCnt = 0;
  for (int b = 0; b < m_binNum; b++)
  {
    if (m_rspSeqStats.binHist[b] > 0.001)
    {
      m_rspSeqStats.nonZeroCnt++;
      m_rspSeqStats.meanBinVar += m_rspSeqStats.binVar[b];
      if (m_rspSeqStats.binVar[b] > m_rspSeqStats.maxBinVar) { m_rspSeqStats.maxBinVar = m_rspSeqStats.binVar[b]; }
      if (m_rspSeqStats.binVar[b] < m_rspSeqStats.minBinVar) { m_rspSeqStats.minBinVar = m_rspSeqStats.binVar[b]; }
    }
  }
  m_rspSeqStats.meanBinVar /= (double)m_rspSeqStats.nonZeroCnt;
  for (int b = 0; b < m_binNum; b++)
  {
    if (m_rspSeqStats.meanBinVar > 0.0)
      m_rspSeqStats.normVar[b] = m_rspSeqStats.binVar[b] / m_rspSeqStats.meanBinVar;
    m_rspSeqStats.weightVar += m_rspSeqStats.binHist[b] * m_rspSeqStats.binVar[b];
    m_rspSeqStats.weightNorm += m_rspSeqStats.binHist[b] * m_rspSeqStats.normVar[b];
  }
  ratioWeiVar = m_rspSeqStats.weightVar / m_srcSeqStats.weightVar;
  ratioWeiVarNorm = m_rspSeqStats.weightNorm / m_srcSeqStats.weightNorm;

  if ((m_srcSeqStats.binHist[0] + m_srcSeqStats.binHist[m_binNum - 1]) > 0.0001 && m_srcSeqStats.binHist[m_binNum - 2] < 0.001)
  {
    if (percBinVarLessThenVal3 > 0.8 && percBinVarLessThenVal2 > 0.4 && m_srcSeqStats.binVar[m_binNum - 2] > 4.8) { isSkipCase = true; }
    else if (percBinVarLessThenVal3 < 0.1 && percBinVarLessThenVal1 < 0.05 && m_srcSeqStats.binVar[m_binNum - 2] < 4.0) { isSkipCase = true; }
  }
  if (isSkipCase) { *intraAdp = false;  *interAdp = false;  return; }

  if (m_reshapeCW.rspPicSize > 5184000) { isLowCase = true; }
  else if (m_srcSeqStats.binVar[1] > 4.0) { isLowCase = true; }
  else if (m_rspSeqStats.meanBinVar > 3.4 && ratioWeiVarNorm > 1.005 && ratioWeiVar > 1.02) { isLowCase = true; }
  else if (m_rspSeqStats.meanBinVar > 3.1 && ratioWeiVarNorm > 1.005 && ratioWeiVar > 1.04) { isLowCase = true; }
  else if (m_rspSeqStats.meanBinVar > 2.8 && ratioWeiVarNorm > 1.01 && ratioWeiVar > 1.04) { isLowCase = true; }

  if (m_reshapeCW.updateCtrl == 0)
  {
    m_reshapeCW.binCW[1] = 1022;
    if (isLowCase)
    {
      *intraAdp = false;
      m_rateAdpMode = 1;
      m_reshapeCW.binCW[1] = 980;
      if (m_srcSeqStats.binHist[m_binNum - 2] > 0.05)
      {
        m_reshapeCW.binCW[1] = 896;
        if (m_srcSeqStats.binVar[m_binNum - 2] < 1.2) { m_reshapeCW.binCW[1] = 938; }
      }
      else if (percBinVarLessThenVal2 < 0.8 && percBinVarLessThenVal3 == 1.0)
      {
        m_rateAdpMode = 1;
        m_reshapeCW.binCW[1] = 938;
      }
    }
    if (m_srcSeqStats.binHist[m_binNum - 2] < 0.001)
    {
      if (m_srcSeqStats.binHist[1] > 0.05 && m_srcSeqStats.binVar[1] > 3.0)
      {
        *intraAdp = true;
        m_rateAdpMode = 1;
        m_reshapeCW.binCW[1] = 784;
      }
      else if (m_srcSeqStats.binHist[1] < 0.006)
      {
        *intraAdp = false;
        m_rateAdpMode = 0;
        m_reshapeCW.binCW[1] = 1008;
      }
      else if (percBinVarLessThenVal3 < 0.5)
      {
        *intraAdp = true;
        m_rateAdpMode = 0;
        m_reshapeCW.binCW[1] = 1022;
      }
    }
    else if ((m_srcSeqStats.maxBinVar > 4.0 && m_rspSeqStats.meanBinVar > 3.2 && percBinVarLessThenVal2 < 0.25) || ratioWeiVar < 1.03)
    {
      *intraAdp = true;
      m_rateAdpMode = 0;
      m_reshapeCW.binCW[1] = 1022;
    }
    if (*intraAdp == true && m_rateAdpMode == 0) { m_tcase = 9; }
  }
  else if (m_reshapeCW.updateCtrl == 1)
  {
    m_reshapeCW.binCW[1] = 952;
    if (isLowCase)
    {
      if (m_reshapeCW.rspPicSize > 5184000)
      {
        m_rateAdpMode = 1;
        m_reshapeCW.binCW[1] = 812;
      }
      if (m_srcSeqStats.binHist[m_binNum - 2] > 0.05)
      {
        m_rateAdpMode = 1;
        m_reshapeCW.binCW[1] = 812;
        if (m_srcSeqStats.binHist[m_binNum - 2] > 0.1 || m_srcSeqStats.binHist[1] > 0.1)
        {
          m_rateAdpMode = 0;
          m_reshapeCW.binCW[1] = 924;
        }
      }
      else if (percBinVarLessThenVal2 < 0.8 && percBinVarLessThenVal3 == 1.0)
      {
        m_rateAdpMode = 1;
        m_reshapeCW.binCW[1] = 896;
      }
      else if (percBinVarLessThenVal2 > 0.98 && m_srcSeqStats.binHist[1] > 0.05)
      {
        m_rateAdpMode = 0;
        m_reshapeCW.binCW[1] = 784;
      }
      else if (percBinVarLessThenVal2 < 0.1)
      {
        m_rateAdpMode = 0;
        m_reshapeCW.binCW[1] = 1022;
      }
    }
    if (m_srcSeqStats.binHist[1] > 0.1 && (m_srcSeqStats.binVar[1] > 1.8 && m_srcSeqStats.binVar[1] < 3.0))
    {
      m_rateAdpMode = 1;
      if (m_srcSeqStats.binVar[m_binNum - 2] > 1.2 && m_srcSeqStats.binVar[m_binNum - 2] < 4.0) { m_reshapeCW.binCW[1] = 784; }
    }
    else if (m_srcSeqStats.binHist[m_binNum - 2] < 0.001)
    {
      if (m_srcSeqStats.binHist[1] > 0.05 && m_srcSeqStats.binVar[1] > 3.0)
      {
        m_rateAdpMode = 1;
        m_reshapeCW.binCW[1] = 784;
      }
      else if (m_srcSeqStats.binHist[1] < 0.006)
      {
        m_rateAdpMode = 0;
        m_reshapeCW.binCW[1] = 980;
      }
      else if (percBinVarLessThenVal3 < 0.5)
      {
        m_rateAdpMode = 0;
        m_reshapeCW.binCW[1] = 924;
      }
    }
    else if ((m_srcSeqStats.maxBinVar > 4.0 && m_rspSeqStats.meanBinVar > 3.2 && percBinVarLessThenVal2 < 0.25) || ratioWeiVar < 1.03)
    {
      m_rateAdpMode = 0;
      m_reshapeCW.binCW[1] = 980;
    }
  }
  else
  {
    m_useAdpCW = true;
    m_reshapeCW.binCW[0] = 36;  m_reshapeCW.binCW[1] = 30;
    if (isLowCase)
    {
      if (m_srcSeqStats.binHist[m_binNum - 2] > 0.05)
      {
        m_useAdpCW = false;
        m_rateAdpMode = 1;
        m_reshapeCW.binCW[1] = 896;
        if (m_srcSeqStats.binHist[1] > 0.005) { m_rateAdpMode = 0; }
      }
      else if (percBinVarLessThenVal2 < 0.8 && percBinVarLessThenVal3 == 1.0) { m_reshapeCW.binCW[1] = 28; }
    }
    if (m_srcSeqStats.binHist[1] > 0.1 && m_srcSeqStats.binVar[1] > 1.8 && m_srcSeqStats.binVar[1] < 3.0)
    {
      m_useAdpCW = false;
      m_rateAdpMode = 1;
      m_reshapeCW.binCW[1] = 952;
    }
    else if (m_srcSeqStats.binHist[1] > 0.05 && m_srcSeqStats.binHist[m_binNum - 2] < 0.001 && m_srcSeqStats.binVar[1] > 3.0)
    {
      m_useAdpCW = false;
      m_rateAdpMode = 1;
      m_reshapeCW.binCW[1] = 784;
    }
    else if (m_srcSeqStats.binHist[1] > 0.05 && m_srcSeqStats.binHist[m_binNum - 2] < 0.005 && m_srcSeqStats.binVar[1] > 1.0 && m_srcSeqStats.binVar[1] < 1.5)
    {
      m_rateAdpMode = 2;
      m_reshapeCW.binCW[0] = 38;
    }
    else if (m_srcSeqStats.binHist[1] < 0.005 && m_srcSeqStats.binHist[m_binNum - 2] > 0.05 && m_srcSeqStats.binVar[m_binNum - 2] > 1.0 && m_srcSeqStats.binVar[m_binNum - 2] < 1.5)
    {
      m_rateAdpMode = 2;
      m_reshapeCW.binCW[0] = 36;
    }
    else if (m_srcSeqStats.binHist[1] > 0.02 && m_srcSeqStats.binHist[m_binNum - 2] > 0.04 && m_srcSeqStats.binVar[1] < 2.0 && m_srcSeqStats.binVar[m_binNum - 2] < 1.5)
    {
      m_rateAdpMode = 2;
      m_reshapeCW.binCW[0] = 34;
    }
    else if ((m_srcSeqStats.binHist[1] > 0.05 && m_srcSeqStats.binHist[m_binNum - 2] > 0.2 && m_srcSeqStats.binVar[1] > 3.0 && m_srcSeqStats.binVar[1] < 4.0) || ratioWeiVar < 1.03)
    {
      m_rateAdpMode = 1;
      m_reshapeCW.binCW[0] = 34;
    }
    else if (m_srcSeqStats.binVar[1] < 4.0 && percBinVarLessThenVal2 == 1.0 && percBinVarLessThenVal3 == 1.0)
    {
      m_rateAdpMode = 0;
      m_reshapeCW.binCW[0] = 34;
    }
    if (m_useAdpCW && !isLowCase) { m_reshapeCW.binCW[1] = 66 - m_reshapeCW.binCW[0]; }
  }
}

void EncReshape::deriveReshapeParameters(double *array, int start, int end, ReshapeCW respCW, double &alpha, double &beta)
{
  double minVar = 10.0, maxVar = 0.0;
  for (int b = start; b <= end; b++)
  {
    if (array[b] < minVar)       minVar = array[b];
    if (array[b] > maxVar)       maxVar = array[b];
  }
  double maxCW = (double)respCW.binCW[0];
  double minCW = (double)respCW.binCW[1];
  alpha = (minCW - maxCW) / (maxVar - minVar);
  beta = (maxCW*maxVar - minCW*minVar) / (maxVar - minVar);
}

/**
-Init reshaping LUT  from dQP model
*/
void EncReshape::initLUTfromdQPModel()
{
  int pwlFwdLUTsize = PIC_CODE_CW_BINS;
  int pwlFwdBinLen = m_reshapeLUTSize / PIC_CODE_CW_BINS;
  double lumaDQP = 0.0;
  double * slopeLUT = new double[m_reshapeLUTSize]();
  double * fwdLUTHighPrec = new double[m_reshapeLUTSize]();
  for (int i = 0; i < m_reshapeLUTSize; i++)
  {
    int inputY = m_lumaBD < 10 ? i << (10 - m_lumaBD) : m_lumaBD > 10 ? i >> (m_lumaBD - 10) : i;
    lumaDQP = 0.015*(double)inputY - 7.5;
    lumaDQP = lumaDQP<-3 ? -3 : (lumaDQP>6 ? 6 : lumaDQP);
    slopeLUT[i] = pow(2.0, lumaDQP / 6.0);
  }
  for (int i = 0; i < (16 << (m_lumaBD - 8)); i++) { slopeLUT[i] = 0.0; }
  for (int i = (235 << (m_lumaBD - 8)); i < m_reshapeLUTSize; i++) { slopeLUT[i] = 0.0; }
  for (int i = 0; i < m_reshapeLUTSize - 1; i++)
    fwdLUTHighPrec[i + 1] = fwdLUTHighPrec[i] + slopeLUT[i];
  if (slopeLUT != nullptr) { delete[] slopeLUT;    slopeLUT = nullptr; }

  double maxY = fwdLUTHighPrec[m_reshapeLUTSize - 1];
  for (int i = 0; i < m_reshapeLUTSize; i++)
  {
    m_fwdLUT[i] = (int16_t)((fwdLUTHighPrec[i] / maxY * (double)(m_reshapeLUTSize - 1)) + 0.5);
  }

  if (fwdLUTHighPrec != nullptr)   {    delete[] fwdLUTHighPrec;    fwdLUTHighPrec = nullptr;  }
  m_sliceReshapeInfo.reshaperModelMinBinIdx = 1;
  m_sliceReshapeInfo.reshaperModelMaxBinIdx = PIC_CODE_CW_BINS-2;

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    int16_t X1 = i * pwlFwdBinLen;
    m_reshapePivot[i] = m_fwdLUT[X1];
  }
  m_reshapePivot[pwlFwdLUTsize] = ((1 << m_lumaBD) - 1);

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    m_binCW[i] = m_reshapePivot[i + 1] - m_reshapePivot[i];
  }
  for (int i = 0; i <= PIC_CODE_CW_BINS; i++)
  {
    m_inputPivot[i] = m_initCW * i;
  }

  adjustLmcsPivot();

  int maxAbsDeltaCW = 0, absDeltaCW = 0, deltaCW = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMinBinIdx; i <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; i++)
  {
    deltaCW = (int)m_binCW[i] - (int)m_initCW;
    m_sliceReshapeInfo.reshaperModelBinCWDelta[i] = deltaCW;
    absDeltaCW = (deltaCW < 0) ? (-deltaCW) : deltaCW;
    if (absDeltaCW > maxAbsDeltaCW)     {      maxAbsDeltaCW = absDeltaCW;    }
  }
  m_sliceReshapeInfo.maxNbitsNeededDeltaCW = std::max(1, 1 + floorLog2(maxAbsDeltaCW));

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    m_fwdScaleCoef[i] = ((int32_t)m_binCW[i] * (1 << FP_PREC) + (1 << (floorLog2(pwlFwdBinLen) - 1))) >> floorLog2(pwlFwdBinLen);
    if (m_binCW[i] == 0)
    {
      m_invScaleCoef[i] = 0;
      m_chromaAdjHelpLUT[i] = 1 << CSCALE_FP_PREC;
    }
    else
    {
      m_invScaleCoef[i] = (int32_t)(m_initCW * (1 << FP_PREC) / m_binCW[i]);
      m_chromaAdjHelpLUT[i] = (int32_t)(m_initCW * (1 << FP_PREC) / (m_binCW[i] + m_sliceReshapeInfo.chrResScalingOffset));
    }
  }
  for (int lumaSample = 0; lumaSample < m_reshapeLUTSize; lumaSample++)
  {
    int idxY = lumaSample / m_initCW;
    int tempVal = m_reshapePivot[idxY] + ((m_fwdScaleCoef[idxY] * (lumaSample - m_inputPivot[idxY]) + (1 << (FP_PREC - 1))) >> FP_PREC);
    m_fwdLUT[lumaSample] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)(tempVal));

    int idxYInv = getPWLIdxInv(lumaSample);
    int invSample = m_inputPivot[idxYInv] + ((m_invScaleCoef[idxYInv] * (lumaSample - m_reshapePivot[idxYInv]) + (1 << (FP_PREC - 1))) >> FP_PREC);
    m_invLUT[lumaSample] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)(invSample));
  }
}

void EncReshape::constructReshaperLMCS()
{
  int bdShift = m_lumaBD - 10;
  int totCW = bdShift != 0 ? (bdShift > 0 ? m_reshapeLUTSize / (1 << bdShift) : m_reshapeLUTSize * (1 << (-bdShift))) : m_reshapeLUTSize;
  int histLenth = totCW / m_binNum;
  int log2HistLenth = floorLog2(histLenth);
  int i;

  if (bdShift != 0)
  {
    for (int i = 0; i < PIC_ANALYZE_CW_BINS; i++)
    {
      m_binCW[i] = bdShift > 0 ? m_binCW[i] * (1 << bdShift) : m_binCW[i] / (1 << (-bdShift));
    }
  }
  if (m_binNum == PIC_ANALYZE_CW_BINS)
  {
    for (int i = 0; i < PIC_CODE_CW_BINS; i++)
    {
      m_binCW[i] = m_binCW[2 * i] + m_binCW[2 * i + 1];
    }
  }
  for (int i = 0; i <= PIC_CODE_CW_BINS; i++)
  {
    m_inputPivot[i] = m_initCW * i;
  }

  m_sliceReshapeInfo.reshaperModelMinBinIdx = 0;
  m_sliceReshapeInfo.reshaperModelMaxBinIdx = PIC_CODE_CW_BINS - 1;
  for (int i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    if (m_binCW[i] > 0)
    {
      m_sliceReshapeInfo.reshaperModelMinBinIdx = i;
      break;
    }
  }
  for (int i = PIC_CODE_CW_BINS - 1; i >= 0; i--)
  {
    if (m_binCW[i] > 0)
    {
      m_sliceReshapeInfo.reshaperModelMaxBinIdx = i;
      break;
    }
  }

  adjustLmcsPivot();

  int maxAbsDeltaCW = 0, absDeltaCW = 0, deltaCW = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMinBinIdx; i <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; i++)
  {
    deltaCW = (int)m_binCW[i] - (int)m_initCW;
    m_sliceReshapeInfo.reshaperModelBinCWDelta[i] = deltaCW;
    absDeltaCW = (deltaCW < 0) ? (-deltaCW) : deltaCW;
    if (absDeltaCW > maxAbsDeltaCW) { maxAbsDeltaCW = absDeltaCW; }
  }
  m_sliceReshapeInfo.maxNbitsNeededDeltaCW = std::max(1, 1 + floorLog2(maxAbsDeltaCW));

  histLenth = m_initCW;
  log2HistLenth = floorLog2(histLenth);

  int sumBins = 0;
  for (i = 0; i < PIC_CODE_CW_BINS; i++) { sumBins += m_binCW[i]; }
  CHECK(sumBins >= m_reshapeLUTSize, "SDR CW assignment is wrong!!");
  for (int i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    m_reshapePivot[i + 1] = m_reshapePivot[i] + m_binCW[i];
    m_fwdScaleCoef[i] = ((int32_t)m_binCW[i] * (1 << FP_PREC) + (1 << (log2HistLenth - 1))) >> log2HistLenth;
    if (m_binCW[i] == 0)
    {
      m_invScaleCoef[i] = 0;
      m_chromaAdjHelpLUT[i] = 1 << CSCALE_FP_PREC;
    }
    else
    {
      m_invScaleCoef[i] = (int32_t)(m_initCW * (1 << FP_PREC) / m_binCW[i]);
      m_chromaAdjHelpLUT[i] = (int32_t)(m_initCW * (1 << FP_PREC) / (m_binCW[i] + m_sliceReshapeInfo.chrResScalingOffset));
    }
  }
  for (int lumaSample = 0; lumaSample < m_reshapeLUTSize; lumaSample++)
  {
    int idxY = lumaSample / m_initCW;
    int tempVal = m_reshapePivot[idxY] + ((m_fwdScaleCoef[idxY] * (lumaSample - m_inputPivot[idxY]) + (1 << (FP_PREC - 1))) >> FP_PREC);
    m_fwdLUT[lumaSample] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)(tempVal));

    int idxYInv = getPWLIdxInv(lumaSample);
    int invSample = m_inputPivot[idxYInv] + ((m_invScaleCoef[idxYInv] * (lumaSample - m_reshapePivot[idxYInv]) + (1 << (FP_PREC - 1))) >> FP_PREC);
    m_invLUT[lumaSample] = Clip3((Pel)0, (Pel)((1 << m_lumaBD) - 1), (Pel)(invSample));
  }
  for (i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    int start = i*histLenth;
    int end = (i + 1)*histLenth - 1;
    m_cwLumaWeight[i] = m_fwdLUT[end] - m_fwdLUT[start];
  }
}

void EncReshape::adjustLmcsPivot()
{
  int bdShift = m_lumaBD - 10;
  int totCW = bdShift != 0 ? (bdShift > 0 ? m_reshapeLUTSize / (1 << bdShift) : m_reshapeLUTSize * (1 << (-bdShift))) : m_reshapeLUTSize;
  int orgCW = totCW / PIC_CODE_CW_BINS;
  int log2SegSize = Log2(LMCS_SEG_NUM);

  m_reshapePivot[0] = 0;
  for (int i = 0; i < PIC_CODE_CW_BINS; i++)
  {
    m_reshapePivot[i + 1] = m_reshapePivot[i] + m_binCW[i];
  }
  int segIdxMax = (m_reshapePivot[m_sliceReshapeInfo.reshaperModelMaxBinIdx + 1] >> log2SegSize);
  for (int i = m_sliceReshapeInfo.reshaperModelMinBinIdx; i <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; i++)
  {
    m_reshapePivot[i + 1] = m_reshapePivot[i] + m_binCW[i];
    int segIdxCurr = (m_reshapePivot[i]     >> log2SegSize);
    int segIdxNext = (m_reshapePivot[i + 1] >> log2SegSize);

    if ((segIdxCurr == segIdxNext) && (m_reshapePivot[i] != (segIdxCurr << log2SegSize)))
    {
      if (segIdxCurr == segIdxMax)
      {
        m_reshapePivot[i] = m_reshapePivot[m_sliceReshapeInfo.reshaperModelMaxBinIdx + 1];
        for (int j = i; j <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; j++)
        {
          m_reshapePivot[j + 1] = m_reshapePivot[i];
          m_binCW[j] = 0;
        }
        m_binCW[i - 1] = m_reshapePivot[i] - m_reshapePivot[i - 1];
        break;
      }
      else
      {
        int16_t adjustVal = ((segIdxCurr + 1) << log2SegSize) - m_reshapePivot[i + 1];
        m_reshapePivot[i + 1] += adjustVal;
        m_binCW[i] += adjustVal;

        for (int j = i + 1; j <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; j++)
        {
          if (m_binCW[j] < (adjustVal + (orgCW >> 3)))
          {
            adjustVal -= (m_binCW[j] - (orgCW >> 3));
            m_binCW[j] = (orgCW >> 3);
          }
          else
          {
            m_binCW[j] -= adjustVal;
            adjustVal = 0;
          }
          if (adjustVal == 0)
            break;
        }
      }
    }
  }

  for (int i = PIC_CODE_CW_BINS - 1; i >= 0; i--)
  {
    if (m_binCW[i] > 0)
    {
      m_sliceReshapeInfo.reshaperModelMaxBinIdx = i;
      break;
    }
  }
}


} // namespace vvenc

//
//! \}

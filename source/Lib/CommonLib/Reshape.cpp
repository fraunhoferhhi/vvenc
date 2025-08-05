/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/** \file     Reshape.cpp
    \brief    common reshaper class
*/
#include "Reshape.h"
#include "UnitTools.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
 //! \ingroup CommonLib
 //! \{

namespace vvenc {

// =====================================================================================================================
// ReshapeData
// =====================================================================================================================

int ReshapeData::getPWLIdxInv( int lumaVal ) const
{
  for (int idxS = m_sliceReshapeInfo.reshaperModelMinBinIdx; (idxS <= m_sliceReshapeInfo.reshaperModelMaxBinIdx); idxS++)
  {
    if (lumaVal < m_reshapePivot[idxS + 1]) 
      return idxS;    
  }
  return PIC_CODE_CW_BINS-1; 
}

int ReshapeData::calculateChromaAdj( Pel avgLuma ) const
{
  int iAdj = m_chromaAdjHelpLUT[getPWLIdxInv(avgLuma)];
  return(iAdj);
}

int ReshapeData::calculateChromaAdjVpduNei( const TransformUnit& tu, const CompArea& areaY, const TreeType _treeType )
{
  CodingStructure &cs = *tu.cs;
  int xPos = areaY.lumaPos().x;
  int yPos = areaY.lumaPos().y;
  int numNeighborLog = std::min<unsigned>(6,cs.pcv->maxCUSizeLog2);
  int numNeighbor    =  1<<numNeighborLog;
  const int mask     = 1-numNeighbor-1;
  xPos &= mask;
  yPos &= mask;

  if( !true/*isEncoder*/ )
  {
    if( isVPDUprocessed( xPos, yPos ) )
    {
      return m_chromaScale;
    }

    setVPDULoc( xPos, yPos );
  }
  Position topLeft(xPos, yPos);
  CodingStructure* pcs =  (CS::isDualITree(cs) && cs.slice->sliceType == VVENC_I_SLICE) ? tu.cs->picture->cs : tu.cs;
  CodingUnit *topLeftLuma   = pcs->getCU(topLeft, CH_L, _treeType);
  const CodingUnit *cuAbove = pcs->getCURestricted( topLeftLuma->lumaPos().offset(0, -1), topLeftLuma->lumaPos(), topLeftLuma->slice->independentSliceIdx, topLeftLuma->tileIdx, CH_L, _treeType );
  const CodingUnit *cuLeft  = pcs->getCURestricted( topLeftLuma->lumaPos().offset(-1, 0), topLeftLuma->lumaPos(), topLeftLuma->slice->independentSliceIdx, topLeftLuma->tileIdx, CH_L, _treeType ); 

  xPos = topLeftLuma->lumaPos().x;
  yPos = topLeftLuma->lumaPos().y;

  const CPelBuf piRecoY = cs.picture->getRecoBuf(topLeftLuma->Y());
  const int strideY = piRecoY.stride;

  const Pel* recSrc0 = piRecoY.bufAt(0, 0);
  const uint32_t picH = tu.cs->picture->lheight();
  const uint32_t picW = tu.cs->picture->lwidth();
  int32_t recLuma = 0;

  int pelnum = 0;
  if (cuLeft != nullptr)
  {
    pelnum ++;
    for (int i = 0; i < numNeighbor; i++)
    {
      int k = (yPos + i) >= picH ? (picH - yPos - 1) : i;
      recLuma += recSrc0[-1 + k * strideY];
    }
  }
  if (cuAbove != nullptr)
  {
    pelnum++;
    for (int i = 0; i < numNeighbor; i++)
    {
      int k = (xPos + i) >= picW ? (picW - xPos - 1) : i;
      recLuma += recSrc0[-strideY + k];
    }
  }

  int lumaValue;
  if( pelnum )
  {
    const int shift = numNeighborLog + pelnum - 1;
    lumaValue = (recLuma + (1 << (shift - 1))) >> shift;
  }
  else
  {
    lumaValue = 1 << (cs.sps->bitDepths[CH_L] - 1);
  }

  int chromaScale = calculateChromaAdj(lumaValue);
  if( !true/*isEncoder*/)
  {
    m_chromaScale = chromaScale;
  }
  return chromaScale;
}

// =====================================================================================================================
// Reshape
// =====================================================================================================================

void Reshape::createDec(int bitDepth)
{
  m_lumaBD = bitDepth;
  m_reshapeLUTSize = 1 << m_lumaBD;
  m_initCW = m_reshapeLUTSize / PIC_CODE_CW_BINS;
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_BUFFER
  if( m_fwdLUT.empty() )
    m_fwdLUT.resize( m_reshapeLUTSize + 2, 0 );
  if( m_invLUT.empty() )
    m_invLUT.resize( m_reshapeLUTSize + 2, 0 );
#else
  if( m_fwdLUT.empty() )
    m_fwdLUT.resize( m_reshapeLUTSize, 0 );
  if( m_invLUT.empty() )
    m_invLUT.resize( m_reshapeLUTSize, 0 );
#endif
  if (m_binCW.empty())
    m_binCW.resize(PIC_CODE_CW_BINS, 0);
  if (m_inputPivot.empty())
    m_inputPivot.resize(PIC_CODE_CW_BINS + 1, 0);
  if (m_fwdScaleCoef.empty())
    m_fwdScaleCoef.resize(PIC_CODE_CW_BINS, 1 << FP_PREC);
  if (m_invScaleCoef.empty())
    m_invScaleCoef.resize(PIC_CODE_CW_BINS, 1 << FP_PREC);
  if (m_reshapePivot.empty())
    m_reshapePivot.resize(PIC_CODE_CW_BINS + 1, 0);
  if (m_chromaAdjHelpLUT.empty())
    m_chromaAdjHelpLUT.resize(PIC_CODE_CW_BINS, 1<<CSCALE_FP_PREC);
}

/** Construct reshaper from syntax
* \param void
* \return void
*/
void Reshape::constructReshaper()
{
  int pwlFwdLUTsize = PIC_CODE_CW_BINS;
  int pwlFwdBinLen = m_reshapeLUTSize / PIC_CODE_CW_BINS;

  for (int i = 0; i < m_sliceReshapeInfo.reshaperModelMinBinIdx; i++)
    m_binCW[i] = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMaxBinIdx + 1; i < PIC_CODE_CW_BINS; i++)
    m_binCW[i] = 0;
  for (int i = m_sliceReshapeInfo.reshaperModelMinBinIdx; i <= m_sliceReshapeInfo.reshaperModelMaxBinIdx; i++)
    m_binCW[i] = (uint16_t)(m_sliceReshapeInfo.reshaperModelBinCWDelta[i] + (int)m_initCW);

  for (int i = 0; i < pwlFwdLUTsize; i++)
  {
    m_reshapePivot[i + 1] = m_reshapePivot[i] + m_binCW[i];
    m_inputPivot[i + 1] = m_inputPivot[i] + m_initCW;
    m_fwdScaleCoef[i] = ((int32_t)m_binCW[i] * (1 << FP_PREC) + (1 << (floorLog2(pwlFwdBinLen) - 1))) >> floorLog2(pwlFwdBinLen);
    if (m_binCW[i] == 0)
    {
      m_invScaleCoef[i] = 0;
      m_chromaAdjHelpLUT[i] = 1 << CSCALE_FP_PREC;
    }
    else
    {
      m_invScaleCoef[i] = (int32_t)(m_initCW * (1 << FP_PREC) / m_binCW[i]);
      m_chromaAdjHelpLUT[i] = (int32_t)(m_initCW * (1 << FP_PREC) / ( m_binCW[i] + m_sliceReshapeInfo.chrResScalingOffset ) );
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



void Reshape::initLumaLevelToWeightTableReshape()
{
  const int lutSize = 1 << m_lumaBD;
  if (m_reshapeLumaLevelToWeightPLUT.empty())
    m_reshapeLumaLevelToWeightPLUT.resize(lutSize, 1.0);

  if (m_lumaLevelToWeightPLUT.empty())
    m_lumaLevelToWeightPLUT.resize(lutSize, 1.0);

  if (m_signalType == RESHAPE_SIGNAL_PQ)
  {
    for (int i = 0; i < lutSize; i++)
    {
      double x = m_lumaBD < 10 ? i << (10 - m_lumaBD) : m_lumaBD > 10 ? i >> (m_lumaBD - 10) : i;
      double y;
      y = 0.015*x - 1.5 - 6;
      y = y < -3 ? -3 : (y > 6 ? 6 : y);
      m_lumaLevelToWeightPLUT[i] = pow(2.0, y / 3.0);
      m_reshapeLumaLevelToWeightPLUT[i] = (uint32_t)(m_lumaLevelToWeightPLUT[i]* (double)(1 << 16));
    }
  }
}

void Reshape::updateReshapeLumaLevelToWeightTableChromaMD( const Pel* ILUT)
{
  const int lutSize = 1 << m_lumaBD;
  for( int i = 0; i < lutSize; i++ )
  {
    m_reshapeLumaLevelToWeightPLUT[i] = (uint32_t)(m_lumaLevelToWeightPLUT[ILUT[i]]* (double)(1 << 16));
  }
}

void Reshape::restoreReshapeLumaLevelToWeightTable()
{
  const int lutSize = 1 << m_lumaBD;
  for (int i = 0; i < lutSize; i++)
  {
    m_reshapeLumaLevelToWeightPLUT[i] = (uint32_t)(m_lumaLevelToWeightPLUT[i]* (double)(1 << 16));
  }
}

void Reshape::updateReshapeLumaLevelToWeightTable(LmcsParam &sliceReshape, Pel* wtTable, double cwt)
{
  if (m_signalType == RESHAPE_SIGNAL_SDR || m_signalType == RESHAPE_SIGNAL_HLG)
  {
    if (sliceReshape.sliceReshaperModelPresent )
    {
      double wBin = 1.0;
      double weight = 1.0;
      int histLens = (1 << m_lumaBD) / PIC_CODE_CW_BINS;

      for (int i = 0; i < PIC_CODE_CW_BINS; i++)
      {
        if ((i < sliceReshape.reshaperModelMinBinIdx) || (i > sliceReshape.reshaperModelMaxBinIdx))
          weight = 1.0;
        else
        {
          if (sliceReshape.reshaperModelBinCWDelta[i] == 1 || (sliceReshape.reshaperModelBinCWDelta[i] == -1 * histLens))
            weight = wBin;
          else
          {
            weight = (double)wtTable[i] / (double)histLens;
            weight = weight*weight;
          }
        }
        for (int j = 0; j < histLens; j++)
        {
          int ii = i*histLens + j;
          m_reshapeLumaLevelToWeightPLUT[ii] = (uint32_t)(weight* (double)(1 << 16));
        }
      }
      m_chromaWeightRS = cwt;
    }
    else
    {
      THROW("updateReshapeLumaLevelToWeightTable ERROR!!");
    }
  }
  else
  {
    THROW("updateReshapeLumaLevelToWeightTable not support other signal types!!");
  }
}


} // namespace vvenc

//
//! \}

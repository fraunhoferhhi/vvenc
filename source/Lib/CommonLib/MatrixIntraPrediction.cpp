/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/** \file     MatrixIntraPrediction.cpp
\brief    matrix-based intra prediction class
*/


#include "MatrixIntraPrediction.h"
#include "dtrace_next.h"

#include "UnitTools.h"
#include "MipData.h"

namespace vvenc {

static const int MIP_MAX_INPUT_SIZE             =  8;
static const int MIP_MAX_REDUCED_OUTPUT_SAMPLES = 64;

MatrixIntraPrediction::MatrixIntraPrediction()
  : m_reducedBoundary       (nullptr)
  , m_reducedBoundaryTransp (nullptr)
  , m_inputOffset           ( 0 )
  , m_inputOffsetTransp     ( 0 )
  , m_refSamplesTop         (nullptr)
  , m_refSamplesLeft        (nullptr)
  , m_blockSize             ( 0, 0 )
  , m_sizeId                ( 0 )
  , m_reducedBdrySize       ( 0 )
  , m_reducedPredSize       ( 0 )
  , m_upsmpFactorHor        ( 0 )
  , m_upsmpFactorVer        ( 0 )
{
  m_reducedBoundary       = (Pel*)xMalloc( Pel, MIP_MAX_INPUT_SIZE ); 
  m_reducedBoundaryTransp = (Pel*)xMalloc( Pel, MIP_MAX_INPUT_SIZE );
}

MatrixIntraPrediction::~MatrixIntraPrediction()
{
  xFree( m_reducedBoundary );       m_reducedBoundary = nullptr;
  xFree( m_reducedBoundaryTransp ); m_reducedBoundaryTransp = nullptr;
}

void MatrixIntraPrediction::prepareInputForPred(const CPelBuf &pSrc, const Area& block, const int bitDepth)
{
  // Step 1: Save block size and calculate dependent values
  initPredBlockParams(block);

  m_refSamplesTop  = pSrc.bufAt(1, 0);
  m_refSamplesLeft = pSrc.bufAt(1, 1);

  // Step 3: Compute the reduced boundary via Haar-downsampling (input for the prediction)
  const int inputSize = 2 * m_reducedBdrySize;

  Pel* const topReduced = m_reducedBoundary;
  boundaryDownsampling1D( topReduced, m_refSamplesTop, block.width, m_reducedBdrySize );

  Pel* const leftReduced = m_reducedBoundary + m_reducedBdrySize;
  boundaryDownsampling1D( leftReduced, m_refSamplesLeft, block.height, m_reducedBdrySize );

  Pel* const leftReducedTransposed = m_reducedBoundaryTransp;
  Pel* const topReducedTransposed  = m_reducedBoundaryTransp + m_reducedBdrySize;
  for( int x = 0; x < m_reducedBdrySize; x++ )
  {
    topReducedTransposed[x] = topReduced[x];
  }
  for( int y = 0; y < m_reducedBdrySize; y++ )
  {
    leftReducedTransposed[y] = leftReduced[y];
  }

  // Step 4: Rebase the reduced boundary
  m_inputOffset       = m_reducedBoundary[0];
  m_inputOffsetTransp = m_reducedBoundaryTransp[0];

  const bool hasFirstCol = (m_sizeId < 2);
  m_reducedBoundary      [0] = hasFirstCol ? ((1 << (bitDepth - 1)) - m_inputOffset      ) : 0; // first column of matrix not needed for large blocks
  m_reducedBoundaryTransp[0] = hasFirstCol ? ((1 << (bitDepth - 1)) - m_inputOffsetTransp) : 0;
  for (int i = 1; i < inputSize; i++)
  {
    m_reducedBoundary      [i] -= m_inputOffset;
    m_reducedBoundaryTransp[i] -= m_inputOffsetTransp;
  }
}

void MatrixIntraPrediction::predBlock(Pel* const result, const int modeIdx, const bool transpose, const int bitDepth)
{
  ALIGN_DATA( MEMORY_ALIGN_DEF_SIZE, Pel bufReducedPred[MIP_MAX_REDUCED_OUTPUT_SAMPLES] );

  const bool       needUpsampling  = ( m_upsmpFactorHor > 1 ) || ( m_upsmpFactorVer > 1 );
  Pel* const       reducedPred     = needUpsampling ? bufReducedPred : result;
  const Pel* const reducedBoundary = transpose ? m_reducedBoundaryTransp : m_reducedBoundary;

  {
    const int outputSize = m_reducedPredSize;
    const int inputSize  = 2 * m_reducedBdrySize;
    const int offset     = transpose ? m_inputOffsetTransp : m_inputOffset;
    const int maxVal     = ( 1 << bitDepth ) - 1;

    if( outputSize == 8)
    {
      g_pelBufOP.mipMatrixMul_8_8( reducedPred, reducedBoundary, &mipMatrix16x16[modeIdx][0][0], maxVal, offset, transpose );
    }
    else
    {
      if( inputSize == 4)
      {
        g_pelBufOP.mipMatrixMul_4_4( reducedPred, reducedBoundary, &mipMatrix4x4[modeIdx][0][0], maxVal, offset, transpose );
      }
      else
      {
        g_pelBufOP.mipMatrixMul_8_4( reducedPred, reducedBoundary, &mipMatrix8x8[modeIdx][0][0], maxVal, offset, transpose );
      }
    }
  }

  // Reduced prediction is transposed if ( transpose && needUpsampling ).
  if( needUpsampling )
  {
    const Pel* verSrc   = reducedPred;
    SizeType verSrcStep = m_blockSize.width;

    // upsampling factors 2, 4 or 8: MIP blocks are at most 64x64 with an aspect ratio of at most 4
    auto log2Factor = []( unsigned factor ) { return factor == 2 ? 1u : factor == 4 ? 2u : 3u; };

    if( m_upsmpFactorHor > 1 )
    {
      Pel* const horDst = result + (m_upsmpFactorVer - 1) * m_blockSize.width;
      verSrc = horDst;
      verSrcStep *= m_upsmpFactorVer;

      g_pelBufOP.mipUpsampleHor( horDst, reducedPred, &m_refSamplesLeft[0], verSrcStep, m_upsmpFactorVer, m_reducedPredSize, log2Factor( m_upsmpFactorHor ) );
    }

    if( m_upsmpFactorVer > 1 )
    {
      g_pelBufOP.mipUpsampleVer( result, verSrc, &m_refSamplesTop[0], m_blockSize.width, verSrcStep, m_reducedPredSize, log2Factor( m_upsmpFactorVer ) );
    }
  }
}

void MatrixIntraPrediction::initPredBlockParams(const Size& block)
{
  m_blockSize = block;
  // init size index
  m_sizeId = getMipSizeId( m_blockSize );

  // init reduced boundary size
  m_reducedBdrySize = (m_sizeId == 0) ? 2 : 4;

  // init reduced prediction size
  m_reducedPredSize = ( m_sizeId < 2 ) ? 4 : 8;

  // init upsampling factors
  m_upsmpFactorHor = m_blockSize.width  / m_reducedPredSize;
  m_upsmpFactorVer = m_blockSize.height / m_reducedPredSize;

  CHECKD( (m_upsmpFactorHor < 1) || ((m_upsmpFactorHor & (m_upsmpFactorHor - 1)) != 0), "Need power of two horizontal upsampling factor." );
  CHECKD( (m_upsmpFactorVer < 1) || ((m_upsmpFactorVer & (m_upsmpFactorVer - 1)) != 0), "Need power of two vertical upsampling factor." );
}

void MatrixIntraPrediction::boundaryDownsampling1D(Pel* reducedDst, const Pel* const fullSrc, const SizeType srcLen, const SizeType dstLen)
{
  if (dstLen < srcLen)
  {
    // Create reduced boundary by downsampling
    const SizeType downsmpFactor = srcLen / dstLen;
    const int log2DownsmpFactor = floorLog2(downsmpFactor);
    const int roundingOffset = (1 << (log2DownsmpFactor - 1));

    SizeType srcIdx = 0;
    for( SizeType dstIdx = 0; dstIdx < dstLen; dstIdx++ )
    {
      int sum = 0;
      for( int k = 0; k < downsmpFactor; k++ )
      {
        sum += fullSrc[srcIdx++];
      }
      reducedDst[dstIdx] = (sum + roundingOffset) >> log2DownsmpFactor;
    }
  }
  else
  {
    // Copy boundary if no downsampling is needed
    for (SizeType i = 0; i < dstLen; ++i)
    {
      reducedDst[i] = fullSrc[i];
    }
  }
}



} // namespace vvenc

//! \}

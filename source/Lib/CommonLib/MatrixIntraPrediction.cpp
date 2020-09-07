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

    if( m_upsmpFactorHor > 1 )
    {
      Pel* const horDst = result + (m_upsmpFactorVer - 1) * m_blockSize.width;
      verSrc = horDst;
      verSrcStep *= m_upsmpFactorVer;

      if( m_reducedPredSize == 4)
      {
        if( m_upsmpFactorHor == 2 )
          predictionUpsampling1DHor<4,1>( horDst, reducedPred, &m_refSamplesLeft[0], verSrcStep, m_upsmpFactorVer );
        else if( m_upsmpFactorHor == 4 )
          predictionUpsampling1DHor<4,2>( horDst, reducedPred, &m_refSamplesLeft[0], verSrcStep, m_upsmpFactorVer );
        else
          predictionUpsampling1DHor<4,3>( horDst, reducedPred, &m_refSamplesLeft[0], verSrcStep, m_upsmpFactorVer );
      }
      else
      {
        if( m_upsmpFactorHor == 2 )
          predictionUpsampling1DHor<8,1>( horDst, reducedPred, &m_refSamplesLeft[0], verSrcStep, m_upsmpFactorVer );
        else if( m_upsmpFactorHor == 4 )
          predictionUpsampling1DHor<8,2>( horDst, reducedPred, &m_refSamplesLeft[0], verSrcStep, m_upsmpFactorVer );
        else
          predictionUpsampling1DHor<8,3>( horDst, reducedPred, &m_refSamplesLeft[0], verSrcStep, m_upsmpFactorVer );
      }
    }

    if( m_upsmpFactorVer > 1 )
    {
      if( m_reducedPredSize == 4)
      {
        if( m_upsmpFactorVer == 2 )
          predictionUpsampling1DVer<4,1>( result, verSrc, &m_refSamplesTop[0], m_blockSize.width, verSrcStep );
        else if( m_upsmpFactorVer == 4 )
          predictionUpsampling1DVer<4,2>( result, verSrc, &m_refSamplesTop[0], m_blockSize.width, verSrcStep );
        else
          predictionUpsampling1DVer<4,3>( result, verSrc, &m_refSamplesTop[0], m_blockSize.width, verSrcStep );
      }
      else
      {
        if( m_upsmpFactorVer == 2 )
          predictionUpsampling1DVer<8,1>( result, verSrc, &m_refSamplesTop[0], m_blockSize.width, verSrcStep );
        else if( m_upsmpFactorVer == 4 )
          predictionUpsampling1DVer<8,2>( result, verSrc, &m_refSamplesTop[0], m_blockSize.width, verSrcStep );
        else
          predictionUpsampling1DVer<8,3>( result, verSrc, &m_refSamplesTop[0], m_blockSize.width, verSrcStep );
      }
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

template< SizeType predPredSize, unsigned log2UpsmpFactor>
void MatrixIntraPrediction::predictionUpsampling1DHor(Pel* const dst, const Pel* const src, const Pel* const bndry, const SizeType dstStride, const SizeType bndryStep )
{
  const int roundingOffset   = 1 << (log2UpsmpFactor - 1);
  const SizeType upsmpFactor = 1 << log2UpsmpFactor;

        Pel* dstLine   = dst;
  const Pel* srcLine   = src;
  const Pel* bndryLine = bndry + bndryStep - 1;

  for( SizeType idxOrthDim = 0; idxOrthDim < predPredSize; idxOrthDim++ )
  {
    const Pel* before  = bndryLine;
    const Pel* behind  = srcLine;
          Pel* currDst = dstLine;
    for( SizeType idxUpsmpDim = 0; idxUpsmpDim < predPredSize; idxUpsmpDim++ )
    {
      int scaledVal = ( *before ) << log2UpsmpFactor;
      for( SizeType pos = 0; pos < upsmpFactor; pos++)
      {
        scaledVal -= *before;
        scaledVal += *behind;
        *currDst = (scaledVal + roundingOffset) >> log2UpsmpFactor;
        currDst ++;
      }
      before = behind;
      behind ++;
    }

    srcLine   += predPredSize;
    dstLine   += dstStride;
    bndryLine += bndryStep;
  }
}

template< SizeType inHeight, unsigned log2UpsmpFactor>
void MatrixIntraPrediction::predictionUpsampling1DVer(Pel* const dst, const Pel* const src, const Pel* const bndry, const SizeType outWidth, const SizeType srcStep  )
{
  const int roundingOffset   = 1 << (log2UpsmpFactor - 1);
  const SizeType upsmpFactor = 1 << log2UpsmpFactor;

        Pel* dstLine   = dst;
  const Pel* srcLine   = src;
  const Pel* bndryLine = bndry;

  for( SizeType idxOrthDim = 0; idxOrthDim < outWidth; idxOrthDim++ )
  {
    const Pel* before  = bndryLine;
    const Pel* behind  = srcLine;
          Pel* currDst = dstLine;
    for( SizeType idxUpsmpDim = 0; idxUpsmpDim < inHeight; idxUpsmpDim++ )
    {
      int scaledVal = ( *before ) << log2UpsmpFactor;
      for( SizeType pos = 0; pos < upsmpFactor; pos++)
      {
        scaledVal -= *before;
        scaledVal += *behind;
        *currDst = (scaledVal + roundingOffset) >> log2UpsmpFactor;
        currDst += outWidth;
      }
      before = behind;
      behind += srcStep;
    }

    srcLine ++;
    dstLine ++;
    bndryLine ++;
  }
}


} // namespace vvenc

//! \}

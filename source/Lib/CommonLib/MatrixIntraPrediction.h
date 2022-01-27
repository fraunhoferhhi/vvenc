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

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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
/** \file     MatrixIntraPrediction.h
\brief    matrix-based intra prediction class (header)
*/

#pragma once


#include "Unit.h"

namespace vvenc {

#define MIP_UPSAMPLING_TEMPLATE 1

class MatrixIntraPrediction
{
public:
  MatrixIntraPrediction();
  ~MatrixIntraPrediction();

  void prepareInputForPred (const CPelBuf &pSrc, const Area& block, const int bitDepth);
  void predBlock           (Pel* const result, const int modeIdx, const bool transpose, const int bitDepth);

private:
  Pel*          m_reducedBoundary;            // downsampled             boundary of a block
  Pel*          m_reducedBoundaryTransp;      // downsampled, transposed boundary of a block
  int           m_inputOffset;
  int           m_inputOffsetTransp;
  const Pel*    m_refSamplesTop;              // top  reference samples for upsampling
  const Pel*    m_refSamplesLeft;             // left reference samples for upsampling
  Size          m_blockSize;
  int           m_sizeId;
  int           m_reducedBdrySize;
  int           m_reducedPredSize;
  unsigned int  m_upsmpFactorHor;
  unsigned int  m_upsmpFactorVer;

  void initPredBlockParams            (const Size& block);

  static void boundaryDownsampling1D  (Pel* reducedDst, const Pel* const fullSrc, const SizeType srcLen, const SizeType dstLen);

  template< SizeType predPredSize, unsigned log2UpsmpFactor>
  void predictionUpsampling1DHor       (Pel* const dst, const Pel* const src, const Pel* const bndry, const SizeType dstStride, const SizeType bndryStep );

  template< SizeType inHeight, unsigned log2UpsmpFactor>
  void predictionUpsampling1DVer       (Pel* const dst, const Pel* const src, const Pel* const bndry, const SizeType outWidth, const SizeType srcStep );
};

} // namespace vvenc

//! \}

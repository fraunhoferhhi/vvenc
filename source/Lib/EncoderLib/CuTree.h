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


/** \file     CuTree.h
    \brief    look-ahead stage for CU-tree adaptive quantization
*/

#pragma once

#include "CommonLib/CommonDef.h"
#include "CommonLib/MCTF.h"
#include "CommonLib/RdCost.h"
#include "EncStage.h"

#include <functional>
#include <map>
#include <memory>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

class NoMallocThreadPool;

// Quantizer of the block coder, in the unnormalised 8x8 Hadamard domain (8 times the orthonormal one):
// level = ( |c| * scale + round ) >> 16, dequantised |c| = ( level * deqStep + 8 ) >> 4.
struct TplQuant
{
  int32_t scale;      // 2^16 / step
  int32_t round;      // dead-zone rounding offset, 2^16 units
  int32_t deqStep;    // step, 2^4 units
};

// Codes the 8x8 residual org - pred: Hadamard transform, quantization, and when recon is given the inverse
// transform added to pred. Returns the squared coefficient error in the unnormalised domain (64 times the error
// in samples) and the estimated bits (Exp-Golomb length of each level plus sign and significance).
typedef void ( *TplCodeBlockFn )( const Pel* org, ptrdiff_t orgStride, const Pel* pred, Pel* recon, ptrdiff_t reconStride,
                                  const TplQuant& quant, int maxVal, int64_t& dist, int& bits );

void tplCodeBlockCore( const Pel* org, ptrdiff_t orgStride, const Pel* pred, Pel* recon, ptrdiff_t reconStride,
                       const TplQuant& quant, int maxVal, int64_t& dist, int& bits );

// Bilinear prediction of an 8x8 block from the 9x9 reference samples at src, at the fractional position
// ( fx, fy ) in 1/16 sample.
typedef void ( *TplPredBlockFn )( const Pel* src, ptrdiff_t srcStride, int fx, int fy, Pel* dst );

void tplPredBlockCore( const Pel* src, ptrdiff_t srcStride, int fx, int fy, Pel* dst );

// One row of 2x2 averages, ( a + b + c + d + 2 ) >> 2.
typedef void ( *TplDownsampleRowFn )( const Pel* src, ptrdiff_t srcStride, Pel* dst, int dstWidth );

void tplDownsampleRowCore( const Pel* src, ptrdiff_t srcStride, Pel* dst, int dstWidth );

// ====================================================================================================================
// CU-tree adaptive quantization
//
// CU-tree (the macroblock-tree of x264, J. Garrett-Glaser 2009) lowers the QP of blocks that later pictures predict
// from, so the bits spent on them improve every picture that copies them. This stage estimates that dependency
// with the temporal dependency model (TPL) of J. Han, P. Wilkins, Y. Xu, J. Bankoski, "A quantitative approach to
// the temporal dependency in video coding", arXiv:2108.11586, 2021, which libaom uses. MB-tree measures the
// dependency on source samples, which overstates it where quantization removes the detail (film grain, low rates).
// TPL codes every block at its picture's QP from both the original and a reconstructed reference; the extra
// distortion and rate caused by the reference's coding error is what propagates.
//
// The analysis runs on half-resolution luma in 8x8 blocks (16x16 at full resolution), in coding order. The stage
// holds back the pictures of a GOP until the next GOP has arrived, because the GOP's key picture is also
// referenced by the next one. The result is a QP offset per CTU, applied where QPA and the frame QP apply the
// block importance mapping (BIM) offsets, which CU-tree replaces.
// ====================================================================================================================

class CuTree : public EncStage
{
public:
  CuTree();
  virtual ~CuTree();

  void init( const VVEncCfg& encCfg, NoMallocThreadPool* threadPool );

  // number of pictures the stage holds in addition to the encoder's own GOP queue
  static int getNumLookAheadPics( const VVEncCfg& encCfg );

  TplCodeBlockFn     m_codeBlock;       // kernels, SIMD where available
  TplPredBlockFn     m_predBlock;
  TplDownsampleRowFn m_downsampleRow;

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT
  void initCuTree_X86();
  template <X86_VEXT vext>
  void _initCuTree_X86();
#endif

protected:
  virtual void initPicture    ( Picture* pic );
  virtual void processPictures( const PicList& picList, AccessUnitList& auList, PicList& doneList, PicList& freeList );

private:
  struct Frame;

  const VVEncCfg*                       m_encCfg;
  NoMallocThreadPool*                   m_threadPool;
  MCTF                                  m_motionSearch;  // provides the integer motion search of the coarsest level
  RdCost                                m_rdCost;        // provides the SIMD Hadamard SATD
  FpDistFunc                            m_hadFunc;
  int                                   m_lookAheadPics;
  int                                   m_blkWidth;      // width and height of the picture in analysis blocks
  int                                   m_blkHeight;
  int                                   m_qpBDOffset;
  double                                m_lambdaLeaf;    // Lagrange multiplier at the QP of unreferenced pictures
  std::map<int, std::unique_ptr<Frame>> m_frames;        // analysis data by POC, kept while it can still be referenced

  void   xForEachRow         ( int numRows, const std::function<void( int )>& fn ) const;
  void   xDownsample         ( const PelStorage& src, PelStorage& dst ) const;
  bool   xIsBatchReady       ( const PicList& picList, int& batchEndPoc ) const;
  Frame* xFindFrame          ( int poc ) const;
  void   xAnalyse            ( Frame& frame, Picture* pic, bool flush );
  void   xEstimateMotion     ( const Frame& cur, const Frame& ref, Array2D<MotionVector>& mvs ) const;
  void   xSearchInteger      ( const PelStorage& curPic, const PelStorage& refPic, const Array2D<MotionVector>& coarse, Array2D<MotionVector>& mvs ) const;
  void   xSearchFractional   ( const Frame& cur, const Frame& ref, const Array2D<MotionVector>& coarse, Array2D<MotionVector>& mvs ) const;
  void   xPropagate          ( int firstPoc );
  void   xDistribute         ( Frame& ref, int bx, int by, const MotionVector& mv, double dist, double rate ) const;
  void   xSetQpOffsets       ( const Frame& frame, Picture* pic ) const;
};

} // namespace vvenc

//! \}


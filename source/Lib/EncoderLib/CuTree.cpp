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


/** \file     CuTree.cpp
    \brief    look-ahead stage for CU-tree adaptive quantization

    Implements the temporal dependency model (TPL) of
    J. Han, P. Wilkins, Y. Xu, J. Bankoski, "A quantitative approach to the temporal dependency in video coding",
    arXiv:2108.11586, 2021 (https://arxiv.org/abs/2108.11586). The equation numbers in the comments refer to it.
*/

#include "CuTree.h"
#include "EncSlice.h"
#include "CommonLib/Picture.h"
#include "Utilities/NoMallocThreadPool.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <vector>

//! \ingroup EncoderLib
//! \{

namespace vvenc {

static constexpr int BLK_SIZE     = 8;                     // analysis block, half resolution
static constexpr int MV_PREC      = 16;                    // motion vectors are in 1/16 sample, as in MCTF
static constexpr int BLK_MV_UNITS = BLK_SIZE * MV_PREC;    // one block width in motion vector units
static constexpr int NUM_LEVELS   = 3;                     // half, quarter and eighth resolution
static constexpr int NO_REF       = MAX_INT;
// Border of the half-resolution pictures. Motion search and prediction clamp reads that go beyond it.
static constexpr int LOWRES_PAD   = 32;

enum CuTreePred : uint8_t
{
  CTP_INTRA = 0,
  CTP_L0    = 1,
  CTP_L1    = 2,
  CTP_BI    = 3
};

// Analysis data of one picture. Distortions are squared errors of full-resolution samples.
struct CuTree::Frame
{
  int                   poc       = 0;
  int                   codingNum = 0;
  int                   qp        = 0;       // the picture's QP from the temporal-layer cascade
  bool                  isIntra   = false;
  bool                  analysed  = false;
  bool                  hasImages = true;    // lowRes and recon are kept while other pictures can reference them
  int                   refPoc[2] = { NO_REF, NO_REF };
  PelStorage            lowRes[NUM_LEVELS];
  PelStorage            recon;               // half-resolution reconstruction at the picture's QP
  Array2D<MotionVector> mvs[2];              // half-resolution motion towards refPoc[0] and refPoc[1]
  std::vector<uint8_t>  predMode;
  std::vector<float>    distRec;             // distortion of the coded block, predicted from reconstructed references
  std::vector<float>    distSrc[2];          // the same with reference l replaced by its original
  std::vector<float>    deltaDist[2];        // extra distortion and rate caused by the coding error of reference l
  std::vector<float>    deltaRate[2];
  std::vector<double>   propDist;            // distortion and rate that later pictures owe to each block
  std::vector<double>   propRate;
};

// ====================================================================================================================
// Block helpers
// ====================================================================================================================

// The encoder's own (SIMD) 8x8 Hadamard SATD of the difference between two blocks.
struct Satd8x8
{
  FpDistFunc func;
  int        bitDepth;

  int operator()( const Pel* org, ptrdiff_t orgStride, const Pel* pred, ptrdiff_t predStride ) const
  {
    const DistParam dp( CPelBuf( org, ( int ) orgStride, BLK_SIZE, BLK_SIZE ), CPelBuf( pred, ( int ) predStride, BLK_SIZE, BLK_SIZE ),
                        func, bitDepth, 0, COMP_Y );
    return ( int ) func( dp );
  }
};

void tplPredBlockCore( const Pel* src, ptrdiff_t srcStride, int fx, int fy, Pel* dst )
{
  for( int j = 0; j < BLK_SIZE; j++ )
  {
    const Pel* row0 = src + j * srcStride;
    const Pel* row1 = row0 + srcStride;
    for( int i = 0; i < BLK_SIZE; i++ )
    {
      const int top    = row0[i] * ( MV_PREC - fx ) + row0[i + 1] * fx;
      const int bottom = row1[i] * ( MV_PREC - fx ) + row1[i + 1] * fx;
      dst[j * BLK_SIZE + i] = Pel( ( top * ( MV_PREC - fy ) + bottom * fy + 128 ) >> 8 );
    }
  }
}

void tplDownsampleRowCore( const Pel* src, ptrdiff_t srcStride, Pel* dst, int dstWidth )
{
  const Pel* below = src + srcStride;
  for( int x = 0; x < dstWidth; x++ )
  {
    dst[x] = Pel( ( src[2 * x] + src[2 * x + 1] + below[2 * x] + below[2 * x + 1] + 2 ) >> 2 );
  }
}

// Bilinear prediction of the 8x8 block at (x, y) displaced by mv (1/16 sample). Positions outside the padded
// reference repeat its edge, as the motion search does.
static void predictBlock( TplPredBlockFn predBlock, const CPelBuf& ref, int x, int y, const MotionVector& mv, Pel* dst )
{
  const int fx = mv.x & ( MV_PREC - 1 );
  const int fy = mv.y & ( MV_PREC - 1 );
  const int x0 = x + ( mv.x >> 4 );
  const int y0 = y + ( mv.y >> 4 );
  const int minX = -LOWRES_PAD, maxX = ( int ) ref.width  + LOWRES_PAD - 1;
  const int minY = -LOWRES_PAD, maxY = ( int ) ref.height + LOWRES_PAD - 1;

  if( x0 >= minX && x0 + BLK_SIZE <= maxX && y0 >= minY && y0 + BLK_SIZE <= maxY )
  {
    predBlock( ref.buf + y0 * ref.stride + x0, ref.stride, fx, fy, dst );
    return;
  }

  for( int j = 0; j < BLK_SIZE; j++ )
  {
    const Pel* row0 = ref.buf + Clip3( minY, maxY, y0 + j     ) * ref.stride;
    const Pel* row1 = ref.buf + Clip3( minY, maxY, y0 + j + 1 ) * ref.stride;
    for( int i = 0; i < BLK_SIZE; i++ )
    {
      const int c0 = Clip3( minX, maxX, x0 + i );
      const int c1 = Clip3( minX, maxX, x0 + i + 1 );
      const int top    = row0[c0] * ( MV_PREC - fx ) + row0[c1] * fx;
      const int bottom = row1[c0] * ( MV_PREC - fx ) + row1[c1] * fx;
      dst[j * BLK_SIZE + i] = Pel( ( top * ( MV_PREC - fy ) + bottom * fy + 128 ) >> 8 );
    }
  }
}

// The best of DC, vertical, horizontal and planar prediction from the neighbouring source samples, by SATD.
static void predictIntra( const CPelBuf& pic, int x, int y, int bitDepth, const Satd8x8& satd8x8, Pel* bestPred )
{
  const Pel* org     = pic.buf + y * pic.stride + x;
  const bool hasTop  = y > 0;
  const bool hasLeft = x > 0;
  Pel top[BLK_SIZE], left[BLK_SIZE];
  Pel pred[BLK_SIZE * BLK_SIZE];

  int sum = 0;
  for( int i = 0; i < BLK_SIZE; i++ )
  {
    top[i]  = hasTop  ? org[i - pic.stride]     : 0;
    left[i] = hasLeft ? org[i * pic.stride - 1] : 0;
    sum    += ( hasTop ? top[i] : 0 ) + ( hasLeft ? left[i] : 0 );
  }
  const int num = ( hasTop ? BLK_SIZE : 0 ) + ( hasLeft ? BLK_SIZE : 0 );

  int best = MAX_INT;
  auto keepBest = [&]()
  {
    const int cost = satd8x8( org, pic.stride, pred, BLK_SIZE );
    if( cost < best )
    {
      best = cost;
      std::copy_n( pred, BLK_SIZE * BLK_SIZE, bestPred );
    }
  };

  const Pel dc = num ? Pel( ( sum + ( num >> 1 ) ) / num ) : Pel( 1 << ( bitDepth - 1 ) );
  std::fill_n( pred, BLK_SIZE * BLK_SIZE, dc );
  keepBest();

  if( hasTop )
  {
    for( int j = 0; j < BLK_SIZE; j++ ) std::copy_n( top, BLK_SIZE, pred + j * BLK_SIZE );
    keepBest();
  }
  if( hasLeft )
  {
    for( int j = 0; j < BLK_SIZE; j++ ) std::fill_n( pred + j * BLK_SIZE, BLK_SIZE, left[j] );
    keepBest();
  }
  if( hasTop && hasLeft )
  {
    for( int j = 0; j < BLK_SIZE; j++ )
    {
      for( int i = 0; i < BLK_SIZE; i++ )
      {
        const int hor = ( BLK_SIZE - 1 - i ) * left[j] + ( i + 1 ) * top[BLK_SIZE - 1];
        const int ver = ( BLK_SIZE - 1 - j ) * top[i]  + ( j + 1 ) * left[BLK_SIZE - 1];
        pred[j * BLK_SIZE + i] = Pel( ( hor + ver + BLK_SIZE ) >> 4 );
      }
    }
    keepBest();
  }
}

// 8-point Hadamard butterflies on v[0], v[s], ..., v[7 s].
static void hadamard8( int32_t* v, int s )
{
  const int32_t a0 = v[0] + v[4 * s], a1 = v[s] + v[5 * s], a2 = v[2 * s] + v[6 * s], a3 = v[3 * s] + v[7 * s];
  const int32_t a4 = v[0] - v[4 * s], a5 = v[s] - v[5 * s], a6 = v[2 * s] - v[6 * s], a7 = v[3 * s] - v[7 * s];
  const int32_t b0 = a0 + a2, b1 = a1 + a3, b2 = a0 - a2, b3 = a1 - a3;
  const int32_t b4 = a4 + a6, b5 = a5 + a7, b6 = a4 - a6, b7 = a5 - a7;
  v[0] = b0 + b1; v[s] = b0 - b1; v[2 * s] = b2 + b3; v[3 * s] = b2 - b3;
  v[4 * s] = b4 + b5; v[5 * s] = b4 - b5; v[6 * s] = b6 + b7; v[7 * s] = b6 - b7;
}

// The scalar block coder. Integer Hadamard butterflies are exact in any order, so the SIMD versions give the
// same numbers.
void tplCodeBlockCore( const Pel* org, ptrdiff_t orgStride, const Pel* pred, Pel* recon, ptrdiff_t reconStride,
                       const TplQuant& quant, int maxVal, int64_t& dist, int& bits )
{
  int32_t c[BLK_SIZE * BLK_SIZE];
  for( int y = 0; y < BLK_SIZE; y++ )
  {
    for( int x = 0; x < BLK_SIZE; x++ )
    {
      c[y * BLK_SIZE + x] = org[y * orgStride + x] - pred[y * BLK_SIZE + x];
    }
  }
  for( int y = 0; y < BLK_SIZE; y++ ) hadamard8( c + y * BLK_SIZE, 1 );
  for( int x = 0; x < BLK_SIZE; x++ ) hadamard8( c + x, BLK_SIZE );

  dist = 0;
  bits = 0;
  for( int i = 0; i < BLK_SIZE * BLK_SIZE; i++ )
  {
    const int32_t absCoeff = abs( c[i] );
    const int32_t level    = ( absCoeff * quant.scale + quant.round ) >> 16;
    const int32_t absDeq   = ( level * quant.deqStep + 8 ) >> 4;
    const int64_t err      = absCoeff - absDeq;
    dist += err * err;
    bits += level > 0 ? 2 * floorLog2( level ) + 3 : 0;
    c[i]  = c[i] < 0 ? -absDeq : absDeq;
  }

  if( recon )
  {
    for( int y = 0; y < BLK_SIZE; y++ ) hadamard8( c + y * BLK_SIZE, 1 );
    for( int x = 0; x < BLK_SIZE; x++ ) hadamard8( c + x, BLK_SIZE );
    for( int y = 0; y < BLK_SIZE; y++ )
    {
      for( int x = 0; x < BLK_SIZE; x++ )
      {
        recon[y * reconStride + x] = Pel( Clip3( 0, maxVal, pred[y * BLK_SIZE + x] + ( ( c[y * BLK_SIZE + x] + 32 ) >> 6 ) ) );
      }
    }
  }
}

// Codes an 8x8 half-resolution residual the way TPL does with a Walsh-Hadamard transform. Distortion and rate are
// scaled to the 16x16 full-resolution block it stands for: its low frequencies have twice the coefficient
// amplitude, so the step is halved and the squared error counts four times.
struct BlockCoder
{
  TplCodeBlockFn codeBlock;
  TplQuant       quant;

  // step of the orthonormal half-resolution coefficients, dead-zone rounding offset below one half
  BlockCoder( TplCodeBlockFn fn, double step, double deadZone ) : codeBlock( fn )
  {
    const double unnormStep = step * BLK_SIZE;
    quant.scale   = int32_t( std::lround( 65536.0 / unnormStep ) );
    quant.round   = int32_t( std::lround( 65536.0 * deadZone ) );
    quant.deqStep = int32_t( std::lround( 16.0 * unnormStep ) );
  }

  struct Result
  {
    double dist;      // squared error, full-resolution scale
    double rate;      // estimated bits
  };

  Result code( const Pel* org, ptrdiff_t orgStride, const Pel* pred, Pel* recon, ptrdiff_t reconStride, int maxVal ) const
  {
    int64_t dist = 0;
    int     bits = 0;
    codeBlock( org, orgStride, pred, recon, reconStride, quant, maxVal, dist, bits );
    // unnormalised domain: 64 times the sample error; full resolution: 4 times the half-resolution error
    return { double( dist ) / 16.0, double( bits ) };
  }
};

// Quantization step and Lagrange multiplier of a QP, in sample units of the internal bit depth.
static double qpToStep( int qp, int bitDepth )
{
  return std::pow( 2.0, ( qp - 4 ) / 6.0 ) * double( 1 << ( bitDepth - 8 ) );
}

static double qpToLambda( int qp, int bitDepth )
{
  return 0.57 * std::pow( 2.0, ( qp - 12 ) / 3.0 ) * double( 1 << ( 2 * ( bitDepth - 8 ) ) );
}

// ====================================================================================================================
// Stage
// ====================================================================================================================

CuTree::CuTree()
  : m_codeBlock    ( tplCodeBlockCore )
  , m_predBlock    ( tplPredBlockCore )
  , m_downsampleRow( tplDownsampleRowCore )
  , m_encCfg       ( nullptr )
  , m_threadPool   ( nullptr )
  , m_motionSearch ( true )
  , m_hadFunc      ( nullptr )
  , m_lookAheadPics( 0 )
  , m_blkWidth     ( 0 )
  , m_blkHeight    ( 0 )
  , m_qpBDOffset   ( 0 )
  , m_lambdaLeaf   ( 0.0 )
{
}

CuTree::~CuTree()
{
}

int CuTree::getNumLookAheadPics( const VVEncCfg& encCfg )
{
  // the key picture of a GOP is referenced by the whole next GOP
  return encCfg.m_GOPSize;
}

void CuTree::init( const VVEncCfg& encCfg, NoMallocThreadPool* threadPool )
{
  m_encCfg        = &encCfg;
  m_threadPool    = threadPool;
  m_lookAheadPics = getNumLookAheadPics( encCfg );
  m_blkWidth      = ( encCfg.m_PadSourceWidth  / 2 ) / BLK_SIZE;
  m_blkHeight     = ( encCfg.m_PadSourceHeight / 2 ) / BLK_SIZE;
  m_qpBDOffset    = 6 * ( encCfg.m_internalBitDepth[CH_L] - 8 );
  m_motionSearch.init( encCfg, true, threadPool );
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT
  initCuTree_X86();
#endif
  m_rdCost.create();
  // the 8x8 Hadamard SATD; setDistParam picks it by the block width
  m_hadFunc       = m_rdCost.setDistParam( CPelBuf( nullptr, BLK_SIZE, BLK_SIZE ), CPelBuf( nullptr, BLK_SIZE, BLK_SIZE ), encCfg.m_internalBitDepth[CH_L], DF_HAD ).distFunc;

  // TPL weighs rate with the Lagrange multiplier of the unreferenced pictures, the top temporal layer.
  GOPEntry topEntry;
  topEntry.setDefaultGOPEntry();
  topEntry.copyFromGopCfg( encCfg.m_GOPList[0] );
  for( int i = 1; i < encCfg.m_GOPSize && i < VVENC_MAX_GOP; i++ )
  {
    if( encCfg.m_GOPList[i].m_temporalId > topEntry.m_temporalId ) topEntry.copyFromGopCfg( encCfg.m_GOPList[i] );
  }
  topEntry.m_vtl = std::min( 5, encCfg.m_maxTLayer );
  const int leafQp = Clip3( -m_qpBDOffset, MAX_QP, EncSlice::getCascadeQP( encCfg, topEntry, false, encCfg.m_QP, m_qpBDOffset ) );
  m_lambdaLeaf    = qpToLambda( leafQp, encCfg.m_internalBitDepth[CH_L] );
}

// The rows of one xForEachRow call, shared by the stage thread and the helper tasks. A helper that starts after
// the call has returned still holds them and finds no row left.
struct CuTreeRows
{
  std::atomic<int>                  next{ 0 };
  int                               done    = 0;   // rows finished, guarded by mutex
  int                               numRows = 0;
  const std::function<void( int )>* fn      = nullptr;
  std::mutex                        mutex;
  std::condition_variable           allDone;

  void work()
  {
    for( int row = next++; row < numRows; row = next++ )
    {
      ( *fn )( row );
      std::lock_guard<std::mutex> lock( mutex );
      if( ++done == numRows )
      {
        allDone.notify_one();
      }
    }
  }

  void waitAllDone()
  {
    std::unique_lock<std::mutex> lock( mutex );
    allDone.wait( lock, [this] { return done == numRows; } );
  }
};

// Runs fn( row ) for rows 0 .. numRows - 1. The stage thread works through the rows itself and workers of the
// thread pool join in when they are free. Waiting for row tasks instead stalled the stage thread, and with it the
// stages before the encoder, behind the encoder's queued CTU tasks. Rows write disjoint data, so the result does
// not depend on which thread runs which row.
void CuTree::xForEachRow( int numRows, const std::function<void( int )>& fn ) const
{
  auto rows     = std::make_shared<CuTreeRows>();
  rows->numRows = numRows;
  rows->fn      = &fn;

  const int numHelpers = m_threadPool ? std::min( m_threadPool->numThreads(), numRows - 1 ) : 0;
  for( int i = 0; i < numHelpers; i++ )
  {
    m_threadPool->addBarrierTask( []( int, void* param )
    {
      std::unique_ptr<std::shared_ptr<CuTreeRows>> held( static_cast<std::shared_ptr<CuTreeRows>*>( param ) );
      ( *held )->work();
      return true;
    }, new std::shared_ptr<CuTreeRows>( rows ) );
  }
  rows->work();
  rows->waitAllDone(); // for the rows helpers took and still run
}

// Halves the luma resolution with 2x2 averages and pads the result.
void CuTree::xDownsample( const PelStorage& src, PelStorage& dst ) const
{
  const CPelBuf in      = src.Y();
  const int width       = in.width / 2;
  const int height      = in.height / 2;
  const int rowsPerTask = 16;
  dst.create( CHROMA_400, Area( 0, 0, width, height ), 0, LOWRES_PAD );
  const PelBuf out      = dst.Y();

  xForEachRow( ( height + rowsPerTask - 1 ) / rowsPerTask, [&]( int task )
  {
    for( int y = task * rowsPerTask; y < std::min( height, ( task + 1 ) * rowsPerTask ); y++ )
    {
      m_downsampleRow( in.buf + 2 * y * in.stride, in.stride, out.buf + y * out.stride, width );
    }
  } );
  dst.extendBorderPel( LOWRES_PAD, LOWRES_PAD );
}

void CuTree::initPicture( Picture* pic )
{
  std::unique_ptr<Frame> frame( new Frame );
  frame->poc = pic->poc;

  xDownsample( pic->getOrigBuffer(), frame->lowRes[0] );
  for( int level = 1; level < NUM_LEVELS; level++ )
  {
    xDownsample( frame->lowRes[level - 1], frame->lowRes[level] );
  }

  m_frames[pic->poc] = std::move( frame );
}

void CuTree::processPictures( const PicList& picList, AccessUnitList& auList, PicList& doneList, PicList& freeList )
{
  int batchEndPoc = 0;
  if( picList.empty() || ! xIsBatchReady( picList, batchEndPoc ) )
  {
    return;
  }

  // Every picture is coded from reconstructed references, so the references go first: analyse in coding order.
  const bool flush = picList.back()->isFlush;
  std::vector<Picture*> codingOrder( picList.begin(), picList.end() );
  std::sort( codingOrder.begin(), codingOrder.end(), []( const Picture* a, const Picture* b ) { return a->gopEntry->m_codingNum < b->gopEntry->m_codingNum; } );
  for( auto pic : codingOrder )
  {
    Frame* frame = xFindFrame( pic->poc );
    if( frame->analysed )
    {
      continue;
    }
    xAnalyse( *frame, pic, flush );

    // In a hierarchical GOP the top temporal layer is never referenced, its pictures are not needed any more.
    if( frame->analysed && m_encCfg->m_maxTLayer > 0 && pic->gopEntry->m_temporalId >= m_encCfg->m_maxTLayer )
    {
      for( auto& level : frame->lowRes ) level.destroy();
      frame->recon.destroy();
      frame->hasImages = false;
    }
  }

  xPropagate( picList.front()->poc );

  for( auto pic : picList )
  {
    if( pic->poc > batchEndPoc )
    {
      break;
    }
    xSetQpOffsets( *xFindFrame( pic->poc ), pic );
    doneList.push_back( pic );
    freeList.push_back( pic );
  }

  // Keep the pictures from the batch's last one on, which the next GOP references, and every picture a cached
  // later picture references. Pictures that have not arrived reference only pictures that are still cached.
  int oldestPocNeeded = batchEndPoc;
  for( auto itr = m_frames.upper_bound( batchEndPoc ); itr != m_frames.end(); itr++ )
  {
    for( int refPoc : itr->second->refPoc )
    {
      if( refPoc != NO_REF ) oldestPocNeeded = std::min( oldestPocNeeded, refPoc );
    }
  }
  m_frames.erase( m_frames.begin(), m_frames.lower_bound( oldestPocNeeded ) );
}

// A batch is the pictures of the GOP that comes next. It is ready when that GOP and the look-ahead after it
// have arrived, or when the encoder is flushed.
bool CuTree::xIsBatchReady( const PicList& picList, int& batchEndPoc ) const
{
  const int  gopNum = picList.front()->gopEntry->m_gopNum;
  const bool flush  = picList.back()->isFlush;
  bool gopComplete  = false;

  batchEndPoc = picList.front()->poc;
  for( auto pic : picList )
  {
    if( pic->gopEntry->m_gopNum != gopNum )
    {
      gopComplete = true;
      break;
    }
    batchEndPoc = pic->poc;
  }

  return flush || ( gopComplete && picList.back()->poc >= batchEndPoc + m_lookAheadPics );
}

CuTree::Frame* CuTree::xFindFrame( int poc ) const
{
  auto itr = m_frames.find( poc );
  return itr == m_frames.end() ? nullptr : itr->second.get();
}

// Codes every block of the picture as TPL does: picks intra, L0, L1 or bi-prediction by rate-distortion cost with
// the original references, then codes the chosen mode with the reconstructed references and keeps the
// reconstruction for the pictures that reference this one (Han et al., Eqs. 6 to 11).
void CuTree::xAnalyse( Frame& frame, Picture* pic, bool flush )
{
  const GOPEntry& gopEntry = *pic->gopEntry;
  const int bitDepth       = m_encCfg->m_internalBitDepth[CH_L];
  const int maxVal         = ( 1 << bitDepth ) - 1;
  const int numBlocks      = m_blkWidth * m_blkHeight;

  frame.codingNum = gopEntry.m_codingNum;
  frame.isIntra   = gopEntry.m_sliceType == 'I';
  frame.qp        = Clip3( -m_qpBDOffset, MAX_QP, EncSlice::getCascadeQP( *m_encCfg, gopEntry, frame.isIntra, m_encCfg->m_QP, m_qpBDOffset ) );

  // the nearest reference picture of each list, which must be reconstructed already
  Frame* refs[2] = { nullptr, nullptr };
  if( ! frame.isIntra )
  {
    const int lastPoc = m_frames.rbegin()->first;
    for( int l = 0; l < 2; l++ )
    {
      if( gopEntry.m_numRefPicsActive[l] <= 0 )
      {
        continue;
      }
      const int refPoc = frame.poc - gopEntry.m_deltaRefPics[l][0];
      if( l == 1 && refPoc == frame.refPoc[0] )
      {
        continue;
      }
      if( refPoc > lastPoc && ! flush )
      {
        return; // the reference has not arrived yet, analyse the picture later
      }
      refs[l] = xFindFrame( refPoc );
      if( refs[l] && ! refs[l]->hasImages )
      {
        refs[l] = nullptr; // not expected: only unreferenced pictures drop their images
      }
      if( refs[l] && ! refs[l]->analysed )
      {
        return; // the reference is not reconstructed yet
      }
      if( refs[l] )
      {
        frame.refPoc[l] = refPoc;
      }
    }
  }
  for( int l = 0; l < 2; l++ )
  {
    if( refs[l] )
    {
      xEstimateMotion( frame, *refs[l], frame.mvs[l] );
    }
  }

  // samples outside the block grid keep their source values
  const CPelBuf org = frame.lowRes[0].Y();
  frame.recon.create( CHROMA_400, Area( 0, 0, org.width, org.height ), 0, LOWRES_PAD );
  frame.recon.Y().copyFrom( org );
  frame.predMode.assign( numBlocks, CTP_INTRA );
  frame.distRec.assign( numBlocks, 0.f );
  for( int l = 0; l < 2; l++ )
  {
    frame.distSrc  [l].assign( numBlocks, 0.f );
    frame.deltaDist[l].assign( numBlocks, 0.f );
    frame.deltaRate[l].assign( numBlocks, 0.f );
  }

  const Satd8x8    satd8x8 = { m_hadFunc, bitDepth };
  const double     step    = qpToStep( frame.qp, bitDepth ) / 2;
  const double     lambda  = qpToLambda( frame.qp, bitDepth );
  const BlockCoder intraCoder( m_codeBlock, step, 1.0 / 3 );   // rounding offsets of HM's quantizer without RDOQ
  const BlockCoder interCoder( m_codeBlock, step, 1.0 / 6 );
  const PelBuf     recon   = frame.recon.Y();

  xForEachRow( m_blkHeight, [&]( int by )
  {
    Pel predSrc[2][BLK_SIZE * BLK_SIZE], predRec[2][BLK_SIZE * BLK_SIZE], pred[BLK_SIZE * BLK_SIZE], predIntra[BLK_SIZE * BLK_SIZE];

    for( int bx = 0; bx < m_blkWidth; bx++ )
    {
      const int  idx    = by * m_blkWidth + bx;
      const int  x      = bx * BLK_SIZE, y = by * BLK_SIZE;
      const Pel* orgBlk = org.buf + y * org.stride + x;
      Pel*       recBlk = recon.buf + y * recon.stride + x;
      auto cost = [&]( const BlockCoder::Result& r ) { return r.dist + lambda * r.rate; };

      predictIntra( org, x, y, bitDepth, satd8x8, predIntra );
      const BlockCoder::Result intra = intraCoder.code( orgBlk, org.stride, predIntra, nullptr, 0, maxVal );

      // mode decision with the original references
      int    bestMode = CTP_INTRA;
      double bestCost = cost( intra );
      BlockCoder::Result src[2] = {};
      for( int l = 0; l < 2; l++ )
      {
        if( ! refs[l] ) continue;
        predictBlock( m_predBlock, refs[l]->lowRes[0].Y(), x, y, frame.mvs[l].get( bx, by ), predSrc[l] );
        src[l] = interCoder.code( orgBlk, org.stride, predSrc[l], nullptr, 0, maxVal );
        if( cost( src[l] ) < bestCost )
        {
          bestCost = cost( src[l] );
          bestMode = l == 0 ? CTP_L0 : CTP_L1;
        }
      }
      if( refs[0] && refs[1] )
      {
        for( int i = 0; i < BLK_SIZE * BLK_SIZE; i++ ) pred[i] = Pel( ( predSrc[0][i] + predSrc[1][i] + 1 ) >> 1 );
        const BlockCoder::Result srcBi = interCoder.code( orgBlk, org.stride, pred, nullptr, 0, maxVal );
        if( cost( srcBi ) < bestCost )
        {
          bestCost = cost( srcBi );
          bestMode = CTP_BI;
        }
      }
      frame.predMode[idx] = uint8_t( bestMode );

      if( bestMode == CTP_INTRA )
      {
        // intra cuts the dependency
        frame.distRec[idx] = float( intraCoder.code( orgBlk, org.stride, predIntra, recBlk, recon.stride, maxVal ).dist );
        continue;
      }

      // code the chosen mode from the reconstructed references
      for( int l = 0; l < 2; l++ )
      {
        if( refs[l] ) predictBlock( m_predBlock, refs[l]->recon.Y(), x, y, frame.mvs[l].get( bx, by ), predRec[l] );
      }
      if( bestMode != CTP_BI )
      {
        const int l = bestMode == CTP_L0 ? 0 : 1;
        const BlockCoder::Result rec = interCoder.code( orgBlk, org.stride, predRec[l], recBlk, recon.stride, maxVal );
        frame.distRec  [idx]    = float( rec.dist );
        frame.distSrc  [l][idx] = float( std::min( src[l].dist, rec.dist ) );
        frame.deltaDist[l][idx] = float( std::max( 0.0, rec.dist - src[l].dist ) );
        frame.deltaRate[l][idx] = float( std::max( 0.0, rec.rate - src[l].rate ) );
        continue;
      }

      // bi-prediction: the dependency on each reference is measured by keeping the other one reconstructed
      for( int i = 0; i < BLK_SIZE * BLK_SIZE; i++ ) pred[i] = Pel( ( predRec[0][i] + predRec[1][i] + 1 ) >> 1 );
      const BlockCoder::Result rec = interCoder.code( orgBlk, org.stride, pred, recBlk, recon.stride, maxVal );
      frame.distRec[idx] = float( rec.dist );
      for( int l = 0; l < 2; l++ )
      {
        const Pel* other = predRec[1 - l];
        for( int i = 0; i < BLK_SIZE * BLK_SIZE; i++ ) pred[i] = Pel( ( predSrc[l][i] + other[i] + 1 ) >> 1 );
        const BlockCoder::Result mixed = interCoder.code( orgBlk, org.stride, pred, nullptr, 0, maxVal );
        frame.distSrc  [l][idx] = float( std::min( mixed.dist, rec.dist ) );
        frame.deltaDist[l][idx] = float( std::max( 0.0, rec.dist - mixed.dist ) );
        frame.deltaRate[l][idx] = float( std::max( 0.0, rec.rate - mixed.rate ) );
      }
    }
  } );

  frame.recon.extendBorderPel( LOWRES_PAD, LOWRES_PAD );
  frame.propDist.assign( numBlocks, 0.0 );
  frame.propRate.assign( numBlocks, 0.0 );
  frame.analysed = true;
}

// Hierarchical motion search: MCTF's integer search of 16x16 blocks at eighth resolution, refined by xSearchInteger
// at quarter and half resolution, then the 8x8 analysis blocks to quarter sample by xSearchFractional.
void CuTree::xEstimateMotion( const Frame& cur, const Frame& ref, Array2D<MotionVector>& mvs ) const
{
  const int searchSize = 2 * BLK_SIZE;
  Array2D<MotionVector> integerMvs[NUM_LEVELS];

  for( int level = NUM_LEVELS - 1; level >= 0; level-- )
  {
    const CPelBuf picLevel = cur.lowRes[level].Y();
    integerMvs[level].allocate( picLevel.width / searchSize + 1, picLevel.height / searchSize + 1 );
    if( level == NUM_LEVELS - 1 )
    {
      m_motionSearch.motionEstimationLuma( integerMvs[level], cur.lowRes[level], ref.lowRes[level], LOWRES_PAD, searchSize, nullptr, 2 );
    }
    else
    {
      xSearchInteger( cur.lowRes[level], ref.lowRes[level], integerMvs[level + 1], integerMvs[level] );
    }
  }

  const CPelBuf pic = cur.lowRes[0].Y();
  mvs.allocate( pic.width / BLK_SIZE + 1, pic.height / BLK_SIZE + 1 );
  xSearchFractional( cur, ref, integerMvs[0], mvs );
}

// The positions one block's motion search has checked. Checking a position again cannot change the result: its error
// is the same, and only a smaller error replaces the best. Neighbouring blocks often share a vector, and the
// refinement steps come back to positions checked before, so the searches skip them.
class CheckedPositions
{
public:
  // Records the position; false when it was checked before.
  bool add( int x, int y )
  {
    for( int i = 0; i < m_num; i++ )
    {
      if( m_x[i] == x && m_y[i] == y )
      {
        return false;
      }
    }
    if( m_num < CAPACITY ) // more positions than any search checks, see xSearchInteger
    {
      m_x[m_num]   = x;
      m_y[m_num++] = y;
    }
    return true;
  }

private:
  static constexpr int CAPACITY = 48;
  int m_x[CAPACITY], m_y[CAPACITY];
  int m_num = 0;
};

// Integer motion of the 16x16 blocks of a level from the coarser level: the best of the parent's and its
// neighbours' vectors (doubled) and zero, refined by a small diamond search; at most 1 + 9 + 8 * 4 positions.
void CuTree::xSearchInteger( const PelStorage& curPic, const PelStorage& refPic, const Array2D<MotionVector>& coarse, Array2D<MotionVector>& mvs ) const
{
  const int     bs     = 2 * BLK_SIZE;
  const CPelBuf org    = curPic.Y();
  const CPelBuf refBuf = refPic.Y();
  if( org.height < BLK_SIZE || org.width < BLK_SIZE )
  {
    return;
  }
  const int     rows   = ( org.height - BLK_SIZE ) / bs + 1;
  const int     cols   = ( org.width  - BLK_SIZE ) / bs + 1;

  xForEachRow( rows, [&]( int by )
  {
    for( int bx = 0; bx < cols; bx++ )
    {
      const int  x  = bx * bs, y = by * bs;
      const int  bw = std::min<int>( bs, org.width  - x ) & ~7;
      const int  bh = std::min<int>( bs, org.height - y ) & ~7;
      const Pel* orgBlk = org.buf + y * org.stride + x;
      // keep the displaced block inside the padded reference
      const int  minX = -LOWRES_PAD - x, maxX = int( refBuf.width )  + LOWRES_PAD - bw - x;
      const int  minY = -LOWRES_PAD - y, maxY = int( refBuf.height ) + LOWRES_PAD - bh - y;
      MotionVector     best;
      CheckedPositions checked;
      auto check = [&]( int dx, int dy )
      {
        dx = Clip3( minX, maxX, dx );
        dy = Clip3( minY, maxY, dy );
        if( ! checked.add( dx, dy ) )
        {
          return;
        }
        const Pel* refBlk = refBuf.buf + ( y + dy ) * refBuf.stride + x + dx;
        const int  error  = m_motionSearch.m_motionErrorLumaInt8( orgBlk, org.stride, refBlk, refBuf.stride, bw, bh, best.error );
        if( error < best.error )
        {
          best.set( dx * MV_PREC, dy * MV_PREC, error );
        }
      };

      check( 0, 0 );
      for( int py = by / 2 - 1; py <= by / 2 + 1; py++ )
      {
        for( int px = bx / 2 - 1; px <= bx / 2 + 1; px++ )
        {
          if( px >= 0 && py >= 0 && px < coarse.w() && py < coarse.h() && coarse.get( px, py ).error != INT_LEAST32_MAX )
          {
            const MotionVector& c = coarse.get( px, py );
            check( 2 * c.x / MV_PREC, 2 * c.y / MV_PREC );
          }
        }
      }
      for( int iter = 0; iter < 8; iter++ )
      {
        const MotionVector center = best;
        check( center.x / MV_PREC - 1, center.y / MV_PREC );
        check( center.x / MV_PREC + 1, center.y / MV_PREC );
        check( center.x / MV_PREC, center.y / MV_PREC - 1 );
        check( center.x / MV_PREC, center.y / MV_PREC + 1 );
        if( best.x == center.x && best.y == center.y )
        {
          break;
        }
      }
      mvs.get( bx, by ) = best;
    }
  } );
}

// Motion of the 8x8 analysis blocks from the integer motion of the 16x16 blocks: the best of the parent's and
// its neighbours' vectors and zero, refined to half and then quarter sample with the bilinear predictor.
void CuTree::xSearchFractional( const Frame& cur, const Frame& ref, const Array2D<MotionVector>& coarse, Array2D<MotionVector>& mvs ) const
{
  const CPelBuf org    = cur.lowRes[0].Y();
  const CPelBuf refBuf = ref.lowRes[0].Y();
  const int     rows   = org.height / BLK_SIZE;
  const int     cols   = org.width  / BLK_SIZE;

  xForEachRow( rows, [&]( int by )
  {
    Pel pred[BLK_SIZE * BLK_SIZE];
    for( int bx = 0; bx < cols; bx++ )
    {
      const int  x      = bx * BLK_SIZE, y = by * BLK_SIZE;
      const Pel* orgBlk = org.buf + y * org.stride + x;
      MotionVector     best;
      CheckedPositions checked;
      auto check = [&]( int mvx, int mvy )
      {
        if( ! checked.add( mvx, mvy ) )
        {
          return;
        }
        MotionVector mv;
        mv.set( mvx, mvy, 0 );
        predictBlock( m_predBlock, refBuf, x, y, mv, pred );
        const int error = m_motionSearch.m_motionErrorLumaInt8( orgBlk, org.stride, pred, BLK_SIZE, BLK_SIZE, BLK_SIZE, best.error );
        if( error < best.error )
        {
          best.set( mvx, mvy, error );
        }
      };

      check( 0, 0 );
      for( int py = by / 2 - 1; py <= by / 2 + 1; py++ )
      {
        for( int px = bx / 2 - 1; px <= bx / 2 + 1; px++ )
        {
          if( px >= 0 && py >= 0 && px < coarse.w() && py < coarse.h() )
          {
            const MotionVector& c = coarse.get( px, py );
            check( c.x, c.y );
          }
        }
      }
      for( int step = MV_PREC / 2; step >= MV_PREC / 4; step /= 2 )
      {
        const MotionVector center = best;
        for( int dy = -step; dy <= step; dy += step )
        {
          for( int dx = -step; dx <= step; dx += step )
          {
            if( dx || dy )
            {
              check( center.x + dx, center.y + dy );
            }
          }
        }
      }
      mvs.get( bx, by ) = best;
    }
  } );
}

// Walks the analysed pictures from firstPoc on in reverse coding order and passes each block's dependency to the
// reference blocks it predicts from (Han et al., Eqs. 12, 16 and 17):
//   dD_ref += dD + dD / D_rec * D_prop
//   dR_ref += dR + log2( 2^(2 R_prop) / ( D_src / D_rec * 2^(2 R_prop) + 1 - D_src / D_rec ) )
void CuTree::xPropagate( int firstPoc )
{
  std::vector<Frame*> order;
  for( auto& entry : m_frames )
  {
    Frame& frame = *entry.second;
    std::fill( frame.propDist.begin(), frame.propDist.end(), 0.0 );
    std::fill( frame.propRate.begin(), frame.propRate.end(), 0.0 );
    if( frame.poc >= firstPoc && frame.analysed && ! frame.isIntra )
    {
      order.push_back( &frame );
    }
  }
  std::sort( order.begin(), order.end(), []( const Frame* a, const Frame* b ) { return a->codingNum > b->codingNum; } );

  for( Frame* frame : order )
  {
    Frame* refs[2] = { xFindFrame( frame->refPoc[0] ), xFindFrame( frame->refPoc[1] ) };

    for( int by = 0; by < m_blkHeight; by++ )
    {
      for( int bx = 0; bx < m_blkWidth; bx++ )
      {
        const int idx  = by * m_blkWidth + bx;
        const int mode = frame->predMode[idx];
        if( mode == CTP_INTRA || frame->distRec[idx] <= 0.f )
        {
          continue;
        }
        const double distRec  = frame->distRec[idx];
        const double propRate = frame->propRate[idx];
        const double growth   = propRate > 0.0 ? std::pow( 2.0, 2.0 * propRate ) : 1.0;

        for( int l = 0; l < 2; l++ )
        {
          const bool usesRef = mode == CTP_BI || mode == ( l == 0 ? CTP_L0 : CTP_L1 );
          if( ! usesRef || ! refs[l] )
          {
            continue;
          }
          const double deltaDist = frame->deltaDist[l][idx];
          const double srcRatio  = frame->distSrc[l][idx] / distRec;
          const double dist      = deltaDist + deltaDist / distRec * frame->propDist[idx];
          const double rate      = frame->deltaRate[l][idx] + ( propRate > 0.0 ? std::log2( growth / ( srcRatio * growth + 1.0 - srcRatio ) ) : 0.0 );
          if( dist > 0.0 || rate > 0.0 )
          {
            xDistribute( *refs[l], bx, by, frame->mvs[l].get( bx, by ), dist, rate );
          }
        }
      }
    }
  }
}

// Adds the dependency to the up to four reference blocks that the displaced block overlaps, in proportion to the
// overlapping area. Parts outside the picture are dropped.
void CuTree::xDistribute( Frame& ref, int bx, int by, const MotionVector& mv, double dist, double rate ) const
{
  const int posX = bx * BLK_MV_UNITS + mv.x;
  const int posY = by * BLK_MV_UNITS + mv.y;
  // floor division, positions can be negative
  const int x0 = ( posX >= 0 ? posX : posX - BLK_MV_UNITS + 1 ) / BLK_MV_UNITS;
  const int y0 = ( posY >= 0 ? posY : posY - BLK_MV_UNITS + 1 ) / BLK_MV_UNITS;
  const int fx = posX - x0 * BLK_MV_UNITS;
  const int fy = posY - y0 * BLK_MV_UNITS;
  const double distScale = dist / double( BLK_MV_UNITS * BLK_MV_UNITS );
  const double rateScale = rate / double( BLK_MV_UNITS * BLK_MV_UNITS );

  const int xs[2] = { x0, x0 + 1 };
  const int ys[2] = { y0, y0 + 1 };
  const int ws[2] = { BLK_MV_UNITS - fx, fx };
  const int hs[2] = { BLK_MV_UNITS - fy, fy };

  for( int j = 0; j < 2; j++ )
  {
    if( ys[j] < 0 || ys[j] >= m_blkHeight || hs[j] == 0 )
    {
      continue;
    }
    for( int i = 0; i < 2; i++ )
    {
      if( xs[i] < 0 || xs[i] >= m_blkWidth || ws[i] == 0 )
      {
        continue;
      }
      const int    idx  = ys[j] * m_blkWidth + xs[i];
      const double area = double( ws[i] * hs[j] );
      ref.propDist[idx] += distScale * area;
      ref.propRate[idx] += rateScale * area;
    }
  }
}

// The QP offset of each CTU (Han et al., Eqs. 19 to 21): with the dependency factor
//   alpha = ( distortion + lambda * rate owed by later pictures ) / own distortion
// of the CTU and of the picture, lambda_ctu = lambda * ( 1 + alpha_pic ) / ( 1 + alpha_ctu ). As lambda goes with
// 2^(QP / 3), the QP offset is a multiple of -log2( beta ), beta = ( 1 + alpha_ctu ) / ( 1 + alpha_pic ). Like
// libaom's superblock QP, the change is damped to the square root of beta for intra pictures and for CTUs below
// the picture's average, and to the fourth root for CTUs above it in inter pictures. The offsets go where QPA
// and the non-QPA path pick up the BIM offsets; the picture QP stays that of the temporal-layer cascade.
void CuTree::xSetQpOffsets( const Frame& frame, Picture* pic ) const
{
  const int ctuSize      = m_encCfg->m_bimCtuSize;
  const int widthInCtus  = ( m_encCfg->m_PadSourceWidth  + ctuSize - 1 ) / ctuSize;
  const int heightInCtus = ( m_encCfg->m_PadSourceHeight + ctuSize - 1 ) / ctuSize;
  const int blksPerCtu   = ctuSize / ( 2 * BLK_SIZE );

  PicShared* picShared = pic->m_picShared;
  picShared->m_ctuBimQpOffset.assign( widthInCtus * heightInCtus, 0 );
  picShared->m_picAuxQpOffset = 0;
  if( m_blkWidth == 0 || m_blkHeight == 0 || ! frame.analysed )
  {
    return; // picture smaller than one analysis block, or not analysed
  }

  auto blockDep = [&]( int idx ) { return frame.propDist[idx] + m_lambdaLeaf * frame.propRate[idx]; };

  double picDep = 0.0, picDist = 0.0;
  for( int idx = 0; idx < m_blkWidth * m_blkHeight; idx++ )
  {
    picDep  += blockDep( idx );
    picDist += frame.distRec[idx];
  }
  const double alphaPic = picDep / std::max( picDist, 1.0 );

  for( int cy = 0; cy < heightInCtus; cy++ )
  {
    // CTUs at the right and bottom edge may lie beyond the last full analysis block
    const int by0 = std::min( cy * blksPerCtu, m_blkHeight - 1 );
    const int by1 = std::max( by0 + 1, std::min( ( cy + 1 ) * blksPerCtu, m_blkHeight ) );
    for( int cx = 0; cx < widthInCtus; cx++ )
    {
      const int bx0 = std::min( cx * blksPerCtu, m_blkWidth - 1 );
      const int bx1 = std::max( bx0 + 1, std::min( ( cx + 1 ) * blksPerCtu, m_blkWidth ) );

      double dep = 0.0, dist = 0.0;
      for( int by = by0; by < by1; by++ )
      {
        for( int bx = bx0; bx < bx1; bx++ )
        {
          dep  += blockDep( by * m_blkWidth + bx );
          dist += frame.distRec[by * m_blkWidth + bx];
        }
      }
      const double alphaCtu = dep / std::max( dist, 1.0 );
      const double beta     = ( 1.0 + alphaCtu ) / ( 1.0 + alphaPic );
      const double damping  = beta < 1.0 || frame.isIntra ? 0.5 : 0.25;
      picShared->m_ctuBimQpOffset[cy * widthInCtus + cx] = int( std::lround( -m_encCfg->m_cuTreeScale * damping * std::log2( beta ) ) );
    }
  }
}

} // namespace vvenc

//! \}

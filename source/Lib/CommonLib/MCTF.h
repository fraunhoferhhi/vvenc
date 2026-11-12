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
/** \file     MCTF.h
\brief    MCTF class (header)
*/

#pragma once

#include "CommonLib/Unit.h"
#include "EncoderLib/EncStage.h"
#include "EncoderLib/SEIFilmGrainAnalyzer.h"
#include <sstream>
#include <map>
#include <deque>
#include <algorithm>
#include <atomic>
#include <cstdint>

namespace vvenc {

#if defined(TARGET_SIMD_X86)  && ENABLE_SIMD_OPT_MCTF
using namespace x86_simd;
#endif
#if defined(TARGET_SIMD_ARM)  && ENABLE_SIMD_OPT_MCTF
using namespace arm_simd;
#endif

class NoMallocThreadPool;

//! \ingroup EncoderLib
//! \{

double calcVarCore( const Pel* org, const ptrdiff_t origStride, const int w, const int h );


struct MotionVector
{
  int x, y;
  int error;
  uint16_t rmsme;
  double overlap;

  MotionVector() : x(0), y(0), error(INT_LEAST32_MAX), rmsme(UINT16_MAX) {}

  void set(int vectorX, int vectorY, int errorValue) { x = vectorX; y = vectorY; error = errorValue; }
};

template <class T>
struct Array2D
{
private:
  int m_width, m_height;
  std::vector< T > v;
public:
  Array2D() : m_width(0), m_height(0), v() { }
  Array2D(int width, int height, const T& value=T()) : m_width(0), m_height(0), v() { allocate(width, height, value); }

  int w() const { return m_width; }
  int h() const { return m_height; }

  void allocate(int width, int height, const T& value=T())
  {
    m_width=width;
    m_height=height;
    v.resize(std::size_t(m_width*m_height), value);
  }

  T& get(int x, int y)
  {
    assert(x<m_width && y<m_height);
    return v[y*m_width+x];
  }

  const T& get(int x, int y) const
  {
    assert(x<m_width && y<m_height);
    return v[y*m_width+x];
  }
};

struct TemporalFilterSourcePicInfo
{
  TemporalFilterSourcePicInfo() : picBuffer(), mvs(), index(0) { }
  PelStorage            picBuffer;
  Array2D<MotionVector> mvs;
  int                   index;
};
// ====================================================================================================================
// Class definition
// ====================================================================================================================

struct Picture;

class MCTF : public EncStage
{
public:
  MCTF( bool enableOpt = true );
  virtual ~MCTF();

  void init( const VVEncCfg& encCfg, bool isFinalPass, NoMallocThreadPool* threadPool );

protected:
  virtual void initPicture    ( Picture* pic );
  virtual void processPictures( const PicList& picList, AccessUnitList& auList, PicList& doneList, PicList& freeList );
private:
  void filter( const std::deque<Picture*>& picFifo, int filterIdx );

#if defined(TARGET_SIMD_X86) && ENABLE_SIMD_OPT_MCTF
  void initMCTF_X86();
  template <X86_VEXT vext>
  void _initMCTF_X86();
#endif

#if defined(TARGET_SIMD_ARM) && ENABLE_SIMD_OPT_MCTF
  void initMCTF_ARM();
  template <ARM_VEXT vext>
  void _initMCTF_ARM();
#endif

public:
  static const int16_t m_interpolationFilter4[16][4];
  static const int16_t m_interpolationFilter8[16][8];

  // The largest block whose squared error cannot overflow 32 bits: 2048 * 1023^2 < 2^31 (samples of at most 10 bits).
  static constexpr int MAX_EXACT_INT_AREA = 2048;

  int ( *m_motionErrorLumaIntX )( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int besterror );
  int ( *m_motionErrorLumaInt8 )( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int besterror );
  // m_motionErrorLumaInt8 for the four reference positions refs[0..3] in one call, into errors[0..3]: w is a multiple of
  // 16 and w * h at most MAX_EXACT_INT_AREA. An error above besterror may be a partial sum, as with the one-position
  // kernels. nullptr where no such kernel exists.
  void ( *m_motionErrorLumaInt4 )( const Pel* org, const ptrdiff_t origStride, const Pel* const* refs, const ptrdiff_t buffStride, const int w, const int h, const int besterror, int* errors );

  int ( *m_motionErrorLumaFracX[2] )( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth, const int besterror );
  int ( *m_motionErrorLumaFrac8[2] )( const Pel* org, const ptrdiff_t origStride, const Pel* buf, const ptrdiff_t buffStride, const int w, const int h, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth, const int besterror );
  // The horizontal 4-tap pass of a column of vectors: numRows rows of w samples into filtered (stride w), row r from
  // source row r - 1 of buf, rounded and clipped to the sample range; w is a multiple of 16 and at most 32 (MCTFUnitSize),
  // numRows at most 36. nullptr where no such kernel exists.
  void ( *m_filterColumn4 )( const Pel* buf, const ptrdiff_t buffStride, const int w, const int numRows, const int16_t* xFilter, const int bitDepth, Pel* filtered );
  // The exact errors (no early exit) of num <= 4 vectors that share their horizontal position, for the 4-tap filters,
  // from the rows of m_filterColumn4: vector i uses yFilter[i] and starts rowOffset[i] (0 or 1) rows below filtered; w is
  // a multiple of 16, w and h are at most 32 (MCTFUnitSize). nullptr where no such kernel exists.
  void ( *m_motionErrorLumaFracColumn )( const Pel* org, const ptrdiff_t origStride, const Pel* filtered, const int w, const int h, const int num, const int16_t* const* yFilter, const int* rowOffset, const int bitDepth, int* error );

  void( *m_applyFrac[MAX_NUM_CH][2] )( const Pel* org, const ptrdiff_t origStride, Pel* dst, const ptrdiff_t dstStride, const int bsx, const int bsy, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth );

  void( *m_applyPlanarCorrection )( const Pel* refPel, const ptrdiff_t refStride, Pel* dstPel, const ptrdiff_t dstStride, const int32_t w, const int32_t h, const ClpRng& clpRng, const uint16_t motionError );
  void( *m_applyBlock )( const CPelBuf& src, PelBuf& dst, const CompArea& blk, const ClpRng& clpRng, const Pel** correctedPics, int numRefs, const int* verror, const double* refStrenghts, double weightScaling, double sigmaSq );
  double( *m_calcVar ) ( const Pel* org, const ptrdiff_t origStride, const int w, const int h );

  // The CU-tree look-ahead reuses the motion search.
  void motionEstimationLuma(Array2D<MotionVector> &mvs, const PelStorage &orig, const PelStorage &buffer, const int refMargin, const int bs,
    const Array2D<MotionVector> *previous=0, const int factor = 1, const bool doubleRes = false) const;

private:
  static const double   m_chromaFactor;
  static const double   m_sigmaMultiplier;
  static const int      m_range;
  static const int      m_motionVectorFactor;
  static const int      m_padding;
  static const double   m_refStrengths[2][6];
  static const int      m_cuTreeThresh[4];
  static const double   m_cuTreeCenter;

  const VVEncCfg*       m_encCfg;
  NoMallocThreadPool*   m_threadPool;
  bool                  m_isFinalPass;
  int                   m_filterPoc;
  Area                  m_area;
  int                   m_MCTFSpeedVal;
  Picture*              m_lastPicIn;
  bool                  m_lowResFltSearch = false;  // TODO: use this to select high/low-res filter (6/4 tap) for motion search
  bool                  m_lowResFltApply  = false;  // TODO: use this to select high/low-res filter (6/4 tap) for actual application
  int                   m_searchPttrn     = 0;
  int                   m_mctfUnitSize;
  FgaTemporalResidual   m_fgaResidual;      // grain measurement of the improved film grain analysis (--fga 2)

  void subsampleLuma    (const PelStorage &input, PelStorage &output, const int factor = 2) const;

  // refMargin is the width of the replicated border around the reference frame buffer.
  int motionErrorLuma   (const PelStorage &orig, const PelStorage &buffer, const int refMargin, const int x, const int y, int dx, int dy, const int bs, const int besterror) const;
  // The horizontally filtered columns (m_filterColumn4) of one block's subpel search, kept across its refinement steps:
  // a later step's column with the same horizontal position and rows inside a kept one reuses its rows.
  class FilteredColumns
  {
  public:
    static constexpr int MAX_WIDTH = 32;   // MCTFUnitSize

    // The w-wide rows of the column at horizontal position dx (1/16 samples) whose vectors start top to bottom full
    // rows below the block (dy >> 4), from a kept column of the same position that covers them, or nullptr.
    const Pel* find( const int dx, const int top, const int bottom, const int w )
    {
      for( int c = 0; c < m_num; c++ )
      {
        if( m_dx[c] == dx && m_top[c] <= top && m_bottom[c] >= bottom )
        {
          return rows( c ) + ( top - m_top[c] ) * w;
        }
      }
      return nullptr;
    }

    // The buffer for the rows of a new column, a free slot or, with none left, the last one.
    Pel* add( const int dx, const int top, const int bottom )
    {
      const int c = std::min( m_num, MAX_COLUMNS - 1 );
      m_num       = c + 1;
      m_dx[c]     = dx;
      m_top[c]    = top;
      m_bottom[c] = bottom;
      return rows( c );
    }

  private:
    static constexpr int MAX_COLUMNS = 8;
    static constexpr int MAX_ROWS    = MAX_WIDTH + 4;          // h + 3 rows per vector plus the one-row spread of a column
    static constexpr int COLUMN_SIZE = MAX_ROWS * MAX_WIDTH;   // a multiple of 16 samples (32 bytes)
    static constexpr int ALIGNMENT   = 32;                     // bytes: the 256-bit loads of the vertical pass do not split

    Pel* rows( const int c )
    {
      const uintptr_t start = ( reinterpret_cast<uintptr_t>( m_storage ) + ALIGNMENT - 1 ) & ~uintptr_t( ALIGNMENT - 1 );
      return reinterpret_cast<Pel*>( start ) + c * COLUMN_SIZE;
    }

    int m_dx[MAX_COLUMNS], m_top[MAX_COLUMNS], m_bottom[MAX_COLUMNS];
    int m_num = 0;
    Pel m_storage[MAX_COLUMNS * COLUMN_SIZE + ALIGNMENT / sizeof( Pel )];
  };
  void testSubpelLattice( const PelStorage &orig, const PelStorage &buffer, const int refMargin, const int x, const int y, const int bs,
                          const MotionVector &center, const int range, const int step, FilteredColumns &columns, MotionVector &best ) const;
  void testIntegerLattice( const PelStorage &orig, const PelStorage &buffer, const int refMargin, const int x, const int y, const int bs,
                           const int centerX, const int centerY, const int range, const int step, const MotionVector &tested, MotionVector &best ) const;

  bool estimateLumaLn   ( std::atomic_int& blockX, std::atomic_int* prevLineX, Array2D<MotionVector> &mvs, const PelStorage &orig, const PelStorage &buffer, const int refMargin, const int blockSize,
    const Array2D<MotionVector> *previous, const int factor, const bool doubleRes, int blockY, int bitDepth ) const;

  void bilateralFilter  (const PelStorage &orgPic, std::deque<TemporalFilterSourcePicInfo> &srcFrameInfo, PelStorage &newOrgPic, double overallStrength) const;

  void xFinalizeBlkLine (const PelStorage &orgPic, std::deque<TemporalFilterSourcePicInfo> &srcFrameInfo, PelStorage &newOrgPic, int yStart, const double sigmaSqCh[MAX_NUM_CH], double overallStrenght) const;

  void motionEstimationMCTF(Picture* curPic, std::deque<TemporalFilterSourcePicInfo>& srcFrameInfo, const PelStorage& origBuf, PelStorage& origSubsampled2, PelStorage& origSubsampled4, PelStorage& origSubsampled8, std::vector<double>& mvErr, double& minError, bool addLevel, bool calcErr);

}; // END CLASS DEFINITION MCTF

//! \}

} // namespace vvenc



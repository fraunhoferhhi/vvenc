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
/** \file     MCTF.h
\brief    MCTF class (header)
*/

#pragma once

#include "CommonLib/Unit.h"
#include <sstream>
#include <map>
#include <deque>

namespace vvenc {

class NoMallocThreadPool;

//! \ingroup EncoderLib
//! \{


struct MotionVector
{
  int x, y;
  int error;
  MotionVector() : x(0), y(0), error(INT_LEAST32_MAX) {}
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

class MCTF
{
public:
  MCTF();
  ~MCTF();

  void init( const int internalBitDepth[MAX_NUM_CH],
             const int width,
             const int height,
             const int ctuSize,
             const ChromaFormat inputChroma,
             const int qp,
             const std::vector<int>&    filterFrames,
             const std::vector<double>& filterStrengths,
             const bool filterFutureReference,
             const int MCTFMode,
             const int numLeadFrames,
             const int numTrailFrames,
             const int framesToBeEncoded,
             NoMallocThreadPool* threadPool );
  void uninit();

  void addLeadFrame ( const YUVBuffer& yuvInBuf );
  void addTrailFrame( const YUVBuffer& yuvInBuf );

  int getNumLeadFrames()  const { return (int)m_leadFifo.size(); };
  int getNumTrailFrames() const { return (int)m_trailFifo.size(); };

  int getCurDelay() const { return m_cur_delay; }

  void filter( Picture* pic );
 
private:
#ifdef TARGET_SIMD_X86
  void initMCTF_X86();
  template <X86_VEXT vext>
  void _initMCTF_X86();
#endif

  int ( *m_motionErrorLumaIntX )( const Pel* origOrigin, const ptrdiff_t origStride, const Pel* buffOrigin, const ptrdiff_t buffStride, const int bs, const int x, const int y, const int dx, const int dy, const int besterror );
  int ( *m_motionErrorLumaInt8 )( const Pel* origOrigin, const ptrdiff_t origStride, const Pel* buffOrigin, const ptrdiff_t buffStride, const int bs, const int x, const int y, const int dx, const int dy, const int besterror );
  
  int ( *m_motionErrorLumaFracX )( const Pel* origOrigin, const ptrdiff_t origStride, const Pel* buffOrigin, const ptrdiff_t buffStride, const int bs, const int x, const int y, const int dx, const int dy, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth, const int besterror );
  int ( *m_motionErrorLumaFrac8 )( const Pel* origOrigin, const ptrdiff_t origStride, const Pel* buffOrigin, const ptrdiff_t buffStride, const int bs, const int x, const int y, const int dx, const int dy, const int16_t* xFilter, const int16_t* yFilter, const int bitDepth, const int besterror );

private:
  // Private static member variables
  static const double   m_chromaFactor;
  static const double   m_sigmaMultiplier;
  static const double   m_sigmaZeroPoint;
  static const int      m_range;
  static const int      m_motionVectorFactor;
  static const int      m_padding;
  static const int16_t  m_interpolationFilter[16][8];
  static const double   m_refStrengths[3][2];

  // Private member variables
  int64_t               m_input_cnt;
  int                   m_cur_delay;
  int                   m_internalBitDepth[MAX_NUM_CH];
  ChromaFormat          m_chromaFormatIDC;
  int                   m_QP;
  std::vector<int>      m_FilterFrames;
  std::vector<double>   m_FilterStrengths;
  Area                  m_area;
  int                   m_ctuSize;
  bool                  m_filterFutureReference;
  int                   m_MCTFMode;
  int                   m_numLeadFrames;
  int                   m_numTrailFrames;
  int                   m_framesToBeEncoded;
  NoMallocThreadPool*   m_threadPool;

  std::deque<Picture*>  m_picFifo;
  std::deque<Picture*>  m_leadFifo;
  std::deque<Picture*>  m_trailFifo;

  // Private functions
  Picture* createLeadTrailPic( const YUVBuffer& yuvInBuf, const int poc );
  void subsampleLuma(const PelStorage &input, PelStorage &output, const int factor = 2) const;

  int motionErrorLuma(const PelStorage &orig, const PelStorage &buffer, const int x, const int y, int dx, int dy, const int bs, const int besterror) const;

  void estimateLumaLn( Array2D<MotionVector> &mvs, const PelStorage &orig, const PelStorage &buffer, const int blockSize,
    const Array2D<MotionVector> *previous, const int factor, const bool doubleRes, int blockY ) const;

  void motionEstimationLuma(Array2D<MotionVector> &mvs, const PelStorage &orig, const PelStorage &buffer, const int bs,
    const Array2D<MotionVector> *previous=0, const int factor = 1, const bool doubleRes = false) const;

  void bilateralFilter(const PelStorage &orgPic, const std::deque<TemporalFilterSourcePicInfo> &srcFrameInfo, PelStorage &newOrgPic, double overallStrength) const;

  void applyMotionLn(const Array2D<MotionVector> &mvs, const PelStorage &input, PelStorage &output, int blockNumY, int comp ) const;

  void xFinalizeBlkLine( const PelStorage &orgPic, const std::deque<TemporalFilterSourcePicInfo> &srcFrameInfo, PelStorage &newOrgPic,
    std::vector<PelStorage>& correctedPics, int yStart, const double sigmaSqCh[MAX_NUM_CH], const std::vector<double> refStrengthCh[MAX_NUM_CH] ) const;

}; // END CLASS DEFINITION MCTF

//! \}

} // namespace vvenc



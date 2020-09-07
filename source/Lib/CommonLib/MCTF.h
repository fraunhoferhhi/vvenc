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



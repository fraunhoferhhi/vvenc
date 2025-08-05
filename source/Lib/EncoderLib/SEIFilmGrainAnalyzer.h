/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2025, Fraunhofer-Gesellschaft zur F�rderung der angewandten Forschung e.V. & The VVenC Authors.
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


#ifndef __SEIFILMGRAINANALYZER__
#define __SEIFILMGRAINANALYZER__

#pragma once

#include "CommonLib/Picture.h"
#include "CommonLib/SEI.h"

#include <numeric>
#include <cmath>
#include <algorithm>
#include <fstream>  // Include this for std::ofstream

//using namespace vvenc;
namespace vvenc {

#if defined(TARGET_SIMD_X86)  && ENABLE_SIMD_OPT_FGA
using namespace x86_simd;
#endif
  
static constexpr double   PI                     = 3.14159265358979323846;
static constexpr double   PI_2                   = 3.14159265358979323846 / 2.0;
static constexpr double   pi_8                   = PI / 8.0;
static constexpr double   pi_3_8                 = 3.0 * PI / 8.0;
static constexpr double   pi_5_8                 = 5.0 * PI / 8.0;
static constexpr double   pi_7_8                 = 7.0 * PI / 8.0;

static constexpr int      DATA_BASE_SIZE         = 64;
static constexpr int      INTERVAL_SIZE          = 16;
static constexpr int      MAX_INTERVAL_NUMBER    = (1 << 16) / INTERVAL_SIZE;
static constexpr int      MAXPAIRS               = 256;

static constexpr int      KERNELSIZE             = 3;     // Dilation and erosion kernel size
static constexpr int      CONV_WIDTH_S           = 3;
static constexpr int      CONV_HEIGHT_S          = 3;
// ====================================================================================================================
// Class definition
// ====================================================================================================================

class Canny
{
public:
  Canny();
  ~Canny();

  unsigned int      m_convWidthG = 5, m_convHeightG = 5;		  // Pixel's row and col positions for Gauss filtering
  
  void init ( unsigned int width,
              unsigned int height,
              ChromaFormat inputChroma );

  void destroy ();

  void detect_edges ( const PelStorage* orig,
                      PelStorage* dest,
                      unsigned int uiBitDepth,
                      ComponentID compID );

private:
  double            m_lowThresholdRatio   = 0.1;               // low threshold rato
  int               m_highThresholdRatio  = 3;                 // high threshold rato

  PelStorage *m_orientationBuf = nullptr;
  PelStorage* m_gradientBufX = nullptr;
  PelStorage* m_gradientBufY = nullptr;

  void suppressNonMax ( PelStorage* buff1,
                        PelStorage* buff2,
                        unsigned int width,
                        unsigned int height,
                        ComponentID compID );

  void doubleThreshold ( PelStorage *buff,
                         unsigned int width,
                         unsigned int height,
                         unsigned int bitDepth,
                         ComponentID compID );

  void edgeTracking ( PelStorage* buff1,
                      unsigned int width,
                      unsigned int height,
                      unsigned int windowWidth,
                      unsigned int windowHeight,
                      unsigned int bitDepth,
                      ComponentID compID );

  void (*gradient) ( PelStorage* buff1,
                     PelStorage* buff2,
                     PelStorage *tmpBuf1,
                     PelStorage *tmpBuf2,
                     unsigned int width,
                     unsigned int height,
                     unsigned int bitDepth,
                     ComponentID compID);

#if ENABLE_SIMD_OPT_FGA && defined( TARGET_SIMD_X86 )
  void initFGACannyX86();
  template <X86_VEXT vext>
  void _initFGACannyX86();
#endif
};

class Morph
{
public:
  Morph();
  ~Morph();

  void init ( uint32_t width,
              uint32_t height );

  void destroy ();

  int (*dilation) ( PelStorage *buff,
                    PelStorage *Wbuf,
                    uint32_t bitDepth,
                    ComponentID compID,
                    int numIter,
                    int iter,
                    Pel Value );

#if ENABLE_SIMD_OPT_FGA && defined( TARGET_SIMD_X86 )
  void initFGAMorphX86();
  template <X86_VEXT vext>
  void _initFGAMorphX86();
#endif

  PelStorage* m_dilationBuf = nullptr;
  PelStorage* m_dilationBuf2 = nullptr;
  PelStorage* m_dilationBuf4 = nullptr;
};


class FGAnalyzer
{
public:
  FGAnalyzer();
  ~FGAnalyzer();

  int                             prevAnalysisPoc                = -1;

  void init( const int                        width,
             const int                        height,
             const ChromaFormat               inputChroma,
             const int                        *outputBitDepths,
             const bool doAnalysis[] );
  void destroy        ();

  void estimateGrainParameters ( Picture* pic );

  int getLog2scaleFactor()  { return m_log2ScaleFactor; };

  SeiFgc::CompModel  getCompModel( int idx ) { return m_compModel[idx];  };

private:
  int                             *m_bitDepths;
  ChromaFormat                    m_inputChromaFormat;
  bool                            m_doAnalysis[ComponentID::MAX_NUM_COMP] = { true, true, true };

  Canny                           m_edgeDetector;
  Morph                           m_morphOperation;
  double                          m_lowIntensityRatio            = 0.1;           // supress everything below 0.1*maxIntensityOffset

  CoeffBuf                        * m_dctGrainBlockList;
  TCoeff                          * m_coeffBuf;
  int                             m_numDctGrainBlocks;

  std::vector<double>             coeffs;
  std::vector<double>             scalingVec;
  std::vector<int>                quantVec;

  std::vector<int>                vecMean;
  std::vector<int>                vecVar;

  double                          meanSquaredDctGrain[DATA_BASE_SIZE][DATA_BASE_SIZE];

  /* Interval points for fitFunction */
  std::vector<int>                vec_mean_intensity;
  std::vector<int>                vec_variance_intensity;
  std::vector<int>                element_number_per_interval;
  std::vector<int>                tmp_data_x;
  std::vector<int>                tmp_data_y;

  static constexpr double         m_tapFilter[3]                = { 1, 2, 1 };
  static constexpr double         m_normTap                     = 4.0;

  // fg model parameters
  int                             m_log2ScaleFactor;
  std::vector<std::array<int, 3>> finalIntervalsandScalingFactors;   // lower_bound, upper_bound, scaling_factor
  SeiFgc::CompModel               m_compModel[ComponentID::MAX_NUM_COMP];

  const PelStorage                *m_originalBuf             = nullptr;
  const PelStorage                *m_workingBuf              = nullptr;
  PelStorage                      *m_maskBuf                 = nullptr;
  PelStorage                      *m_grainEstimateBuf        = nullptr;
  PelStorage                      *m_workingBufSubsampled2   = nullptr;
  PelStorage                      *m_maskSubsampled2         = nullptr;
  PelStorage                      *m_workingBufSubsampled4   = nullptr;
  PelStorage                      *m_maskSubsampled4         = nullptr;
  PelStorage                      *m_maskUpsampled           = nullptr;
  // for DCT
  TCoeff                          *m_DCTinout                = nullptr;
  TCoeff                          *m_DCTtemp                 = nullptr;

  void findMask ( ComponentID compID );

  void blockTransform ( CoeffBuf& currentCoeffBuf,
                        int offsetX,
                        int offsetY,
                        uint32_t bitDepth,
                        ComponentID compId );

  void adaptiveSampling ( int bins,
                          double threshold,
                          std::vector<int>& significantIndices,
                          bool isRow,
                          int startIdx );

  void estimateCutoffFreqAdaptive ( ComponentID compID );

  void estimateScalingFactors ( uint32_t bitDepth,
                                ComponentID compID );

  bool fitFunction ( int order,
                     int bitDepth,
                     bool second_pass );

  void avgScalingVec ( int bitDepth );

  bool lloydMax ( double& distortion,
                  int bitDepth );

  void quantize ( std::vector<double>& quantizedVec,
                  double& distortion,
                  double partition[],
                  double codebook[] );

  void extendPoints ( int bitDepth );

  void setEstimatedParameters ( uint32_t bitDepth,
                                ComponentID compID );

  void defineIntervalsAndScalings ( int bitDepth );

  void scaleDown ( int bitDepth );

  void confirmIntervals ( );

  long double ldpow ( long double n,
                      unsigned p );

  int countEdges ( int windowSize,
                   int offsetX,
                   int offsetY,
                   ComponentID compID );

  void subsample ( PelStorage& output,
                   const int factor = 2,
                   const int padding = 0, 
                   ComponentID compID = COMP_Y ) const;

  void upsample ( const PelStorage& input,
                  const int factor = 2,
                  const int padding = 0,
                  ComponentID compID = COMP_Y ) const;

  void combineMasks ( ComponentID compId );

  void suppressLowIntensity ( const PelStorage& buff1,
                              PelStorage& buff2,
                              uint32_t bitDepth,
                              ComponentID compId );

  double (*calcVar) ( const Pel* org,
                      const ptrdiff_t origStride,
                      const int w,
                      const int h );

  int (*calcMean) ( const Pel* org,
                    const ptrdiff_t origStride,
                    const int w,
                    const int h );

  void (*fastDCT2_64) ( const TCoeff* src,
                        TCoeff* dst,
                        int shift,
                        int line,
                        int iSkipLine,
                        int iSkipLine2 );

#if ENABLE_SIMD_OPT_FGA && defined( TARGET_SIMD_X86 )
  void initFGAnalyzerX86();
  template <X86_VEXT vext>
  void _initFGAnalyzerX86();
#endif
};

} // namespace vvenc

#endif // __SEIFILMGRAINANALYZER__



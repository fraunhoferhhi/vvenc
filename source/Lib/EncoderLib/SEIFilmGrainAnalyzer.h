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

/* Film grain analysis (--fga 1): a port of the film grain analysis of the VTM reference software (VTM master,
 * including JVET-AL0282 and JVET-AN0237 by InterDigital).
 *
 * For every analysed picture, the grain is estimated as the difference between the original and a temporally filtered
 * (denoised) version of it, as in VTM.
 * Flat blocks are found with a multi-scale Canny edge detector and morphological dilation on the denoised picture. On
 * 16x16 flat blocks the analysis collects
 *  - the block mean and the grain strength (twice the standard deviation of the grain), from which a polynomial
 *    scaling function is fitted and quantized with Lloyd-Max into intensity intervals and scaling factors, and
 *  - the squared DCT of the grain, per block mean, from which horizontal and vertical cut-off frequencies are
 *    estimated per intensity interval.
 * Both are accumulated over the analysed pictures with exponentially decreasing weights, so that the estimate follows
 * the current scene.
 */

#pragma once

#include "CommonLib/CommonDef.h"
#include "CommonLib/Unit.h"
#include "CommonLib/SEI.h"

#include <utility>
#include <vector>

namespace vvenc {

// Row and block kernels of the analysis, with SIMD versions in CommonLib/x86/FGAX86.h and
// CommonLib/arm/neon/FGA_neon.cpp. A row is processed from x = 0 to width - 1; neighbours outside the picture repeat
// the border sample.
struct FgaRowKernels
{
  // Sobel gradients of the row "cur": magnitude, (|gx| + |gy|) / 2 limited to maxVal, and edge direction (0, 45, 90
  // or 135 degrees).
  void ( *sobel )   ( const Pel* above, const Pel* cur, const Pel* below, int width, Pel maxVal, Pel* mag, Pel* dir );
  // Non-maximum suppression: the magnitude where it is not smaller than both neighbours along the direction, else 0.
  void ( *nonMax )  ( const Pel* magAbove, const Pel* mag, const Pel* magBelow, const Pel* dir, int width, Pel* dst );
  // 3-tap maximum (dilate) or minimum (erode) along a row, and over three rows.
  void ( *morphRow )( const Pel* src, int width, bool dilate, Pel* dst );
  void ( *morphCol )( const Pel* above, const Pel* cur, const Pel* below, int width, bool dilate, Pel* dst );
  // Largest of n non-negative samples.
  Pel  ( *rowMax )  ( const Pel* src, int n );
  // Double threshold: strong above high, weak above low, else 0.
  void ( *classify )( Pel* row, int n, Pel low, Pel high, Pel weak, Pel strong );
  // Positions x of the samples equal to value, in increasing order; returns their number.
  int  ( *findEqual )( const Pel* row, int n, Pel value, int* positions );

  // Block kernels, on 16x16 blocks.
  bool ( *isZeroBlock ) ( const Pel* src, int stride );
  void ( *blockSums )   ( const Pel* src, int stride, int64_t& sum, int64_t& sumSq );
  // 16x16 DCT-II of VVC (rounding shift 8 after each stage, the second stage clipped to +-maxRange); the squared
  // coefficients are added to sum[ horizontal * 16 + vertical ].
  void ( *squaredDct16 )( const Pel* src, int stride, int maxRange, double* sum );

};

// Scalar row kernels for x = xStart to xEnd - 1; the SIMD kernels use them at the picture borders.
void fgaSobelCore   ( const Pel* above, const Pel* cur, const Pel* below, int width, int xStart, int xEnd, Pel maxVal, Pel* mag, Pel* dir );
void fgaNonMaxCore  ( const Pel* magAbove, const Pel* mag, const Pel* magBelow, const Pel* dir, int width, int xStart, int xEnd, Pel* dst );
void fgaMorphRowCore( const Pel* src, int width, int xStart, int xEnd, bool dilate, Pel* dst );
void fgaMorphColCore( const Pel* above, const Pel* cur, const Pel* below, int xStart, int xEnd, bool dilate, Pel* dst );
Pel  fgaRowMaxCore  ( const Pel* src, int n );
int  fgaFindEqualCore( const Pel* row, int n, Pel value, int* positions );
void fgaClassifyCore( Pel* row, int n, Pel low, Pel high, Pel weak, Pel strong );
bool fgaIsZeroBlockCore ( const Pel* src, int stride );
void fgaBlockSumsCore   ( const Pel* src, int stride, int64_t& sum, int64_t& sumSq );
void fgaSquaredDct16Core( const Pel* src, int stride, int maxRange, double* sum );

class FGAnalyzer
{
public:
  FGAnalyzer();

  void init( int width, int height, ChromaFormat chromaFormat, const int bitDepths[ MAX_NUM_CH ], const bool doAnalysis[ MAX_NUM_COMP ] );

  // Analyses one picture: orig is the original, denoised its temporally filtered version (same size); the grain is
  // estimated as denoised minus orig. With sceneCut the statistics of the previous pictures are dropped first, as they
  // describe another scene.
  void estimateGrainParameters( const CPelUnitBuf& orig, const CPelUnitBuf& denoised, bool sceneCut = false );

  // Sets the row kernels: the scalar ones, then the SIMD ones the CPU supports.
  static void initKernels( FgaRowKernels& k );

  // The parameters to signal in the FGC SEI, with the grain strength set to strength percent of the analysed grain:
  // the log2 scale factor and the component models.
  void getSignalledParameters( double strength, int& log2ScaleFactor, SeiFgc::CompModel compModel[ MAX_NUM_COMP ] ) const;

  // A plane of samples with its own storage; the analysis works on whole planes.
  struct Plane
  {
    int              width  = 0;
    int              height = 0;
    std::vector<Pel> samples;

    // Sets the size; the storage only grows, so planes reused for smaller sizes are not cleared again.
    void resize( int w, int h )                     { width = w; height = h; if( samples.size() < (size_t) w * h ) samples.resize( (size_t) w * h ); }
    Pel* row   ( int y )                            { return samples.data() + (size_t) y * width; }
    const Pel* row( int y ) const                   { return samples.data() + (size_t) y * width; }
    Pel  at    ( int x, int y ) const               { return samples[ (size_t) y * width + x ]; }
    // Sample at (x, y) with the coordinates clamped to the plane, as on a border-extended buffer.
    Pel  clampedAt( int x, int y ) const;
  };

private:
  static constexpr int    BLOCK_SIZE         = 16;    // analysis block size, for scaling function and cut-off frequencies
  // Weight of the data accumulated so far when a picture is added. VTM sums the data of all pictures without limit,
  // so a new scene hardly changes the estimate; halving the old data lets it follow the scene within a few analyses.
  static constexpr double ACCUMULATION_DECAY = 0.5;

  // Squared DCT coefficients of the grain, summed over the blocks with one block mean.
  typedef std::vector<double> SquaredDctSum;   // BLOCK_SIZE * BLOCK_SIZE entries, [horizontal][vertical]

  struct CutoffPair
  {
    std::pair<int, int> p;      // horizontal and vertical cut-off
    int                 w = 0;  // weight (interval width)
    std::vector<int>    idx;    // intensity intervals having this pair
  };

  int               m_width  = 0;
  int               m_height = 0;
  ChromaFormat      m_chromaFormat = CHROMA_420;
  int               m_bitDepths[ MAX_NUM_CH ] = { 10, 10 };
  bool              m_doAnalysis[ MAX_NUM_COMP ] = { false, false, false };

  // The analysed parameters: intensities at 8 bits, compModelValue[ 0 ] the grain strength (twice the standard deviation
  // of the grain on 16x16 blocks, at the component's bit depth), cut-off frequencies at the component's resolution.
  // getSignalledParameters() converts them for the synthesis.
  SeiFgc::CompModel m_compModel[ MAX_NUM_COMP ];
  int               m_cutoffPairs[ MAX_NUM_COMP ] = { 0, 0, 0 };

  // Data accumulated over the analysed pictures, weighted (see ACCUMULATION_DECAY).
  std::vector<SquaredDctSum> m_squaredDctSums[ MAX_NUM_COMP ];   // per block mean
  std::vector<double>        m_numDctBlocks  [ MAX_NUM_COMP ];   // per block mean
  std::vector<double>        m_storedMeanIntensity   [ MAX_NUM_COMP ];
  std::vector<double>        m_storedStrengthIntensity[ MAX_NUM_COMP ];
  std::vector<double>        m_storedElementsPerInterval[ MAX_NUM_COMP ];

  // Planes of the picture being analysed.
  Plane             m_denoised[ MAX_NUM_COMP ];
  Plane             m_grain   [ MAX_NUM_COMP ];   // denoised minus original
  Plane             m_mask    [ MAX_NUM_COMP ];   // non-zero where the picture is not flat

  // Scratch planes and lists of the mask search, kept to avoid allocating and clearing them for every picture.
  struct Scratch
  {
    Plane            magnitude, direction, morph, half, quarter, lowMask;
    std::vector<int> positions, weakX, weakY, columnStart;
    std::vector<Pel> wide, top, bottom, left, right;
  };
  Scratch           m_scratch;

  // flat area mask
  void resetStatistics       ( int c );
  void findMask              ( ComponentID compId );
  void maskForScale          ( const Plane& picture, Plane& mask, int bitDepth, int numDilations );

  // parameter estimation
  void estimateComponent     ( ComponentID compId );
  void estimateScalingFactors( std::vector<int>& dataX, std::vector<int>& dataY, int bitDepth, ComponentID compId );
  bool fitFunction           ( std::vector<int>& dataX, std::vector<int>& dataY, std::vector<double>& coeffs, std::vector<double>& scalingVec,
                               int order, int bitDepth, bool secondPass, ComponentID compId );
  void extendPoints          ( std::vector<int>& dataX, std::vector<int>& dataY, int bitDepth ) const;
  bool lloydMax              ( std::vector<double>& scalingVec, std::vector<int>& quantizedVec, int numQuantizedLevels, int bitDepth ) const;
  void setEstimatedParameters( std::vector<int>& quantizedVec, int bitDepth, ComponentID compId );
  void mergeIntervals        ( std::vector<std::vector<int>>& parameters, int bitDepth ) const;
  void scaleDown             ( std::vector<std::vector<int>>& parameters, int bitDepth ) const;

  void estimateCutoffFreq    ( ComponentID compId, int bitDepth );
  int  cutoffFrequency       ( const std::vector<double>& mean ) const;
  void replaceCutoff         ( ComponentID compId, int& replacementH, int& replacementV );
  void limitCutoff           ( ComponentID compId, int avgH, int avgV );
  void limitCutoffConsecutive( ComponentID compId );
  int  limitCutoffPairs      ( std::vector<CutoffPair>& pairs, int maxUnique ) const;

  FgaRowKernels     m_kernels;

#if ENABLE_SIMD_OPT_FGA && defined( TARGET_SIMD_X86 )
  static void initFGAnalyzerX86( FgaRowKernels& k );
  template <x86_simd::X86_VEXT vext>
  static void _initFGAnalyzerX86( FgaRowKernels& k );
#endif
#if ENABLE_SIMD_OPT_FGA && defined( TARGET_SIMD_ARM )
  static void initFGAnalyzerARM( FgaRowKernels& k );
  template <arm_simd::ARM_VEXT vext>
  static void _initFGAnalyzerARM( FgaRowKernels& k );
#endif
};

} // namespace vvenc

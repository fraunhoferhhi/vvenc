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

/* Film grain analysis (--fga 1, 2 and 3): a port of the film grain analysis of the VTM reference software (VTM master,
 * including JVET-AL0282 and JVET-AN0237 by InterDigital), with improvements (--fga 2) and the auto-regressive grain
 * model (--fga 3, which signals an AR filter fitted to the grain spectrum instead of cut-off frequencies).
 *
 * For every analysed picture, the grain is estimated as the difference between the original and a temporally filtered
 * (denoised) version of it, as in VTM (--fga 1). With the improvements (--fga 2) it is estimated from the temporal
 * residual instead, twice the picture minus its motion compensated neighbours (FgaTemporalResidual), which also holds
 * the part of the grain that the filter keeps.
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

  // Temporal residual kernels (FgaTemporalResidual).
  // 4x downsampling of n samples: dst[ x ] = ( sum of the 4x4 samples from src[ 4 x ] on + 8 ) >> 4.
  void ( *downsample4 ) ( const Pel* src, int stride, int n, Pel* dst );
  // Sums of absolute differences of n 4x4 blocks next to each other: sad[ b ] of the blocks at x = 4 b.
  void ( *sad4x4Row )   ( const Pel* cur, int curStride, const Pel* ref, int refStride, int n, int* sad );
  // Temporal residual of n samples: 2 cur - prev - next, or cur - prev when next is null.
  void ( *residualRow ) ( const Pel* cur, const Pel* prev, const Pel* next, int n, Pel* dst );
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
void fgaDownsample4Core ( const Pel* src, int stride, int n, Pel* dst );
void fgaSad4x4RowCore   ( const Pel* cur, int curStride, const Pel* ref, int refStride, int n, int* sad );
void fgaResidualRowCore ( const Pel* cur, const Pel* prev, const Pel* next, int n, Pel* dst );

class FGAnalyzer
{
public:
  FGAnalyzer();

  // improved: the analysis with the improvements beyond VTM (--fga 2), else VTM's analysis with its bugs fixed
  // (--fga 1). arModel: the improved analysis with the auto-regressive grain model (--fga 3). See m_improved.
  void init( int width, int height, ChromaFormat chromaFormat, const int bitDepths[ MAX_NUM_CH ], const bool doAnalysis[ MAX_NUM_COMP ], bool improved, bool arModel = false );

  // Analyses one picture: orig is the original, denoised its temporally filtered version (same size). residual is the
  // temporal residual of FgaTemporalResidual, whose variance is residualWeight times the grain variance; without it
  // the grain is estimated as denoised minus orig.
  void estimateGrainParameters( const CPelUnitBuf& orig, const CPelUnitBuf& denoised, const CPelUnitBuf* residual = nullptr, int residualWeight = 0 );

  // Sets the row kernels: the scalar ones, then the SIMD ones the CPU supports.
  static void initKernels( FgaRowKernels& k );

  // The parameters to signal in the FGC SEI, with the grain strength set to strength percent of the analysed grain:
  // the log2 scale factor and the component models, with scaling factors calibrated against the synthesis.
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

  // The analysed parameters: intensities at 8 bits, compModelValue[ 0 ] the grain strength (m_strengthScale times twice
  // the standard deviation of the grain on 16x16 blocks, at the component's bit depth), cut-off frequencies at the
  // component's resolution. getSignalledParameters() converts them for the synthesis.
  SeiFgc::CompModel m_compModel[ MAX_NUM_COMP ];
  int               m_cutoffPairs[ MAX_NUM_COMP ] = { 0, 0, 0 };
  int               m_failedFits [ MAX_NUM_COMP ] = { 0, 0, 0 };   // analyses in a row whose scaling function fit failed

  // With the improvements beyond VTM (--fga 2): the grain is measured from the temporal residual when MCTF provides
  // it, the strengths are kept in finer units, the scaling function fit works with few intensity intervals (chroma),
  // the scaling factors are calibrated against the synthesis, and the statistics follow grain that changes or
  // disappears. Without them (--fga 1) the analysis is VTM's, with its bugs fixed.
  bool              m_improved = true;
  int               m_strengthScale = 1;   // units of the strengths: twice the standard deviation / m_strengthScale

  // The auto-regressive grain model (--fga 3, FGC model_id 1): instead of cut-off frequencies, every component
  // signals the filter of vvdec's SEI AR mode, x = a1 * left + a1 * r * top + a3 * r * ( top-left + top-right )
  // + a5 * left-left + a5 * r^2 * top-top + noise, fitted to the measured grain spectrum. One filter per component,
  // the same for all intensity intervals, so that the synthesis makes one pattern per component.
  struct ArFilter
  {
    double a1 = 0.0, a3 = 0.0, a5 = 0.0, r = 1.0;
    double nonDcShare = 1.0;   // share of the grain variance outside the DC coefficient of 16x16 blocks
    bool   valid = false;
  };
  bool              m_arModel = false;
  ArFilter          m_arFilter[ MAX_NUM_COMP ];

  // Data accumulated over the analysed pictures, weighted (see ACCUMULATION_DECAY).
  std::vector<SquaredDctSum> m_squaredDctSums[ MAX_NUM_COMP ];   // per block mean
  std::vector<double>        m_numDctBlocks  [ MAX_NUM_COMP ];   // per block mean
  std::vector<double>        m_storedMeanIntensity   [ MAX_NUM_COMP ];
  std::vector<double>        m_storedStrengthIntensity[ MAX_NUM_COMP ];
  std::vector<double>        m_storedElementsPerInterval[ MAX_NUM_COMP ];

  // Planes of the picture being analysed.
  Plane             m_denoised[ MAX_NUM_COMP ];
  Plane             m_grain   [ MAX_NUM_COMP ];   // temporal residual, or denoised minus original
  int               m_grainWeight = 1;            // variance of m_grain relative to the grain variance
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
  void holdEdgeStrengths     ( std::vector<int>& quantizedVec, int bitDepth ) const;
  void mergeIntervals        ( std::vector<std::vector<int>>& parameters, int bitDepth ) const;
  void scaleDown             ( std::vector<std::vector<int>>& parameters, int bitDepth ) const;

  void getVtmParameters      ( double strength, int& log2ScaleFactor, SeiFgc::CompModel compModel[ MAX_NUM_COMP ] ) const;
  void getArParameters       ( double strength, int& log2ScaleFactor, SeiFgc::CompModel compModel[ MAX_NUM_COMP ] ) const;
  void fitArFilter           ( ComponentID compId );

  void estimateCutoffFreq    ( ComponentID compId, int bitDepth );
  int  cutoffFrequency       ( const std::vector<double>& mean ) const;
  void fitCutoffPair         ( const std::vector<double>& meanSq, bool chroma420, int& horizontal, int& vertical ) const;
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

// Temporal residual of the film grain for the improved analysis (--fga 2). For grain that is independent from picture
// to picture, the residual e = 2 O - P - N of a picture O and its motion compensated neighbours P and N has six times
// the grain variance and the spectrum of the grain, whatever the denoiser would have kept of it. MCTF computes it for
// the analysed pictures, while their neighbours are available.
class FgaTemporalResidual
{
public:
  FgaTemporalResidual();

  // Computes the residual of cur from the previous and next picture (null when missing) into residual (created here).
  // The motion is searched per 16x16 block on 4x downsampled luma, with a bias towards zero motion, so that the vectors
  // do not follow the grain in flat areas. A neighbour that matches much worse than the other (a scene cut) is left
  // out. Returns the variance of the residual relative to the grain variance: 6 with both neighbours, 2 with one, and
  // 0, with no residual made, without neighbours or for pictures smaller than one block.
  int compute( const CPelUnitBuf& cur, const CPelUnitBuf* prev, const CPelUnitBuf* next, PelStorage& residual ) const;

private:
  // Block motion in samples of the 4x downsampled pictures, per 16x16 block of the full picture.
  struct Motion
  {
    std::vector<int8_t> dx, dy;
    double              meanSad = 0.0;   // mean absolute difference per downsampled sample, at the chosen vectors
  };

  void downsample( const CPelBuf& src, FGAnalyzer::Plane& dst ) const;
  void search    ( const FGAnalyzer::Plane& cur, const FGAnalyzer::Plane& ref, int numBlocksX, int numBlocksY, Motion& motion ) const;

  FgaRowKernels m_kernels;
};

} // namespace vvenc

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
 * residual instead, twice the picture minus its neighbours motion compensated with the vectors of the temporal filter
 * (MCTF::fgaTemporalResidual), which also holds the part of the grain that the filter keeps.
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

  // Temporal residual kernel (MCTF::fgaTemporalResidual).
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
void fgaResidualRowCore ( const Pel* cur, const Pel* prev, const Pel* next, int n, Pel* dst );

struct LostGrainSpectrum;   // CommonLib/Picture.h

class FGAnalyzer
{
public:
  FGAnalyzer();

  // improved: the analysis with the improvements beyond VTM (--fga 2), else VTM's analysis with its bugs fixed
  // (--fga 1). arModel: the improved analysis with the auto-regressive grain model (--fga 3). See m_improved.
  void init( int width, int height, ChromaFormat chromaFormat, const int bitDepths[ MAX_NUM_CH ], const bool doAnalysis[ MAX_NUM_COMP ], bool improved, bool arModel = false );

  // The number of pictures analysed in a GOP (1: its key picture only). The failed fits that end the grain
  // (m_maxFailedFits, three with one analysis per GOP) are scaled with it, so that the grain ends after as many
  // pictures as with one analysis per GOP. The data's ageing stays per analysis (ACCUMULATION_DECAY): it then follows
  // a change of the grain faster, which measured better than keeping its span in pictures.
  void setAnalysesPerGop( int numAnalyses );

  // Analyses one picture: orig is the original, denoised its temporally filtered version (same size). residual is the
  // temporal residual of MCTF::fgaTemporalResidual, whose variance is residualWeight times the grain variance; without it
  // the grain is estimated as denoised minus orig. With sceneCut the statistics of the previous pictures are dropped
  // first, as they describe another scene.
  void estimateGrainParameters( const CPelUnitBuf& orig, const CPelUnitBuf& denoised, const CPelUnitBuf* residual = nullptr, int residualWeight = 0,
                                bool sceneCut = false );

  // Sets the row kernels: the scalar ones, then the SIMD ones the CPU supports.
  static void initKernels( FgaRowKernels& k );

  // The parameters to signal in the FGC SEI, with the grain strength set to strength percent of the analysed grain:
  // the log2 scale factor and the component models, with scaling factors calibrated against the synthesis.
  void getSignalledParameters( double strength, int& log2ScaleFactor, SeiFgc::CompModel compModel[ MAX_NUM_COMP ] ) const;

  // The film grain SEI fitted to the grain the coding lost, as the eye sees it. lostGrainSpectra: for one component of
  // a coded picture, what the coding took from the original's smoothest 64x64 tiles (the 30 % with the least
  // roughness per level of tile means): the power of original minus reconstruction per band of periods and direction
  // and per 8-bit tile mean, in ascending order (read by several threads at once: it changes no state). Empty before
  // init() with the improved analysis.
  void lostGrainSpectra( const CPelBuf& org, const CPelBuf& rec, int bitDepth, std::vector<LostGrainSpectrum>& spectra ) const;
  // An intensity interval's summed spectra reduced to the mean grain power per tile, in squared sample values, that
  // the coding took in each band of periods (50-25, 25-12.5, 12.5-6.25, 6.25-3.125 samples of the component, and
  // finer) and direction (once per interval: it does not depend on the cut-offs tried).
  struct LostGrainPower
  {
    static constexpr int BANDS      = 5;
    static constexpr int DIRECTIONS = 2;   // 0: the horizontal frequency is the higher one, 1: the vertical one
    double lost[ BANDS ][ DIRECTIONS ] = {};   // the power of original minus reconstruction
    int    tiles                       = 0;
  };
  static LostGrainPower lostGrainPower( const LostGrainSpectrum& pooled );
  // The most one picture's tiles of an interval say the coding took: their mean power per band and direction plus two
  // standard errors of that mean, from how far the tiles' powers differ (LostGrainSpectrum::lostSquared).
  static LostGrainPower lostGrainPowerAtMost( const LostGrainSpectrum& ofPicture );
  // How the bands count in the fit of one intensity interval of a component.
  struct LostGrainBands
  {
    static constexpr int BANDS = LostGrainPower::BANDS;
    double weight        [ BANDS ] = {};   // how much grain power in the band counts to the eye, the largest 1; 0: the band is not fitted
    double allBandsWeight[ BANDS ] = {};   // the same, but no fitted band counts less than a fixed share of the largest
    double tolerance     [ BANDS ] = {};   // the most synthesized power per lost power; 0: the band does not count at all
  };
  // The bands' weights are Barten's contrast sensitivity, squared, at the given display luminance (cd/m2), for a
  // picture of pictureHeight luma rows seen from 1.5 and from 3 picture heights (the mean of both). By them the grain
  // finer than about 6 samples of a 4K picture hardly counts; allBandsWeight is for where it must (see
  // LOST_GRAIN_MIN_WEIGHT). Bands of periods of up to 25 luma samples are fitted, those up to 50 only limit the
  // synthesis, longer ones do not count; the synthesis may exceed the lost grain by 1.05 times in amplitude.
  static LostGrainBands lostGrainBands( bool chroma420, int pictureHeight, double luminance );
  // The luminance in cd/m2 a display gives an 8-bit limited-range luma value: PQ (SMPTE ST 2084), or else gamma 2.4 with
  // white at 100 cd/m2; at least 0.1.
  static double displayLuminance( int luma8, bool pq );
  // What lostGrainFit needs of the grain synthesized at cut-offs (h horizontal, w vertical), each clipped to 2 .. 14.
  // It depends on nothing else, so the caller computes it once per pair of cut-offs and fits every intensity interval
  // with it.
  struct LostGrainShape
  {
    double patternStddev = 0.0;   // the grain pattern's standard deviation
    double blockShare    = 0.0;   // the share of the grain's variance a 16x16 block keeps
    double power[ LostGrainPower::BANDS ][ LostGrainPower::DIRECTIONS ] = {};   // the shares of the grain's variance
  };
  LostGrainShape lostGrainShape( bool chroma420, int h, int w ) const;
  // A fit of the frequency model to the lost grain of an intensity interval at one pair of cut-offs. The costs are in
  // squared sample values, so the costs of intervals can be added; those of no grain do not depend on the cut-offs and
  // are what the others are a share of.
  struct LostGrainFit
  {
    int    scale              = 0;      // the fitted scaling factor
    double seenCost           = -1.0;   // the eye's cost (LostGrainBands::weight) of its grain; below 0: nothing to fit
    double allBandsCost       = 0.0;    // the same grain's cost by LostGrainBands::allBandsWeight
    double seenCostOfNone     = 0.0;    // the two costs of no grain at all
    double allBandsCostOfNone = 0.0;
  };
  // The scaling factor (0 .. largestScale) at the cut-offs of shape whose synthesized grain the eye sees
  // closest to the lost grain: the least sum over the fitted bands and directions of the band's weight times the squared
  // difference of the amplitudes, synthesized grain beyond the lost grain counting four times; among the scaling factors
  // that leave every band within its tolerance (plus a small share of all the lost grain's power). The synthesized
  // power is capped at LOST_GRAIN_CAP_POWER times that of the analyzer's scale s0 at its cut-offs (h0, w0). With
  // nothing to fit the scale is s0. largestScale: lostGrainLargestScale().
  static LostGrainFit lostGrainFit( const LostGrainPower& power, const LostGrainBands& bands, int bitDepth, bool chroma420,
                                    int log2ScaleFactor, int s0, int h0, int w0, const LostGrainShape& shape, int largestScale );
  // The largest scaling factor the fit of the frequency model may give at a log2 scale factor: above the largest one
  // signalled (127) as far as a lower log2 scale factor can signal it. lostGrainSignalledScales then brings the scaling
  // factors of all components of a film grain SEI into the signalled range: it halves them as often as needed and
  // returns the number of halvings, by which the caller lowers the SEI's log2 scale factor.
  static int lostGrainLargestScale( int log2ScaleFactor );
  static int lostGrainSignalledScales( int log2ScaleFactor, SeiFgc::CompModel compModel[ MAX_NUM_COMP ] );
  // The scaling factor at cut-offs (h, w) whose synthesized in-block power equals that of scale s at (h0, w0); it can
  // be larger than any that is signalled.
  static int equalPowerScale( int s, bool chroma420, int h0, int w0, int h, int w );
  // The lost-grain fit may put back up to this many times the analyzer's grain power: the coding can lose more than that
  // (the analyzer's scale falls toward bright areas while the lost grain does not). At 2x Top Gun's mid-bright band
  // (8-bit 97-128) got back 0.68-0.76 of its lost fine grain and 0.52-0.56 of its medium grain, at 4x 0.93-0.94 and
  // 0.71-0.72 (measured with the fit on 16x16 blocks, before it tolerated overshoot). It also bounds grain fitted to
  // texture the coding smoothed away, which the lost grain cannot be told from.
  static constexpr double LOST_GRAIN_CAP_POWER = 4.0;
  // The largest scaling factor the fit may give at cut-offs (h, w): LOST_GRAIN_CAP_POWER times the power of the analyzer's
  // scale s0 at (h0, w0), at most largestScale (lostGrainLargestScale()).
  static int lostGrainCapScale( int s0, bool chroma420, int h0, int w0, int h, int w, int largestScale );

  // The grain the synthesis makes with a filter of the auto-regressive model (--fga 3): its standard deviation per unit
  // of the scaling factor, the shares of its variance in each band and direction of the lost-grain fit, and those of
  // white noise (the error of rounding the grain to whole sample values).
  struct ArGrain
  {
    double stddevPerScale = 0.0;
    double bandPower [ LostGrainPower::BANDS ][ LostGrainPower::DIRECTIONS ] = {};
    double whitePower[ LostGrainPower::BANDS ][ LostGrainPower::DIRECTIONS ] = {};
  };
  // modelValues: an intensity interval's signalled model values (scaling factor, a1, 0, a3, r, a5; the filter is the same
  // in all intervals of a component).
  static ArGrain arGrain( const std::vector<int>& modelValues, int log2ScaleFactor, int bitDepth, bool luma );
  // The scaling factor (0 to twice s0, the analyzer's: LOST_GRAIN_CAP_POWER in power) whose synthesized grain, rounded
  // to whole sample values as the synthesis rounds it, is closest to the lost grain: lostGrainFit's cost and tolerances,
  // by LostGrainBands::allBandsWeight. Returns the cost, -1: nothing to fit.
  static double lostGrainFitAr( const LostGrainPower& power, const LostGrainBands& bands, const ArGrain& grain, int s0, int& scale );
  // The largest scaling factor that fit may give: LOST_GRAIN_CAP_POWER times the power of the analyzer's s0.
  static int lostGrainCapScaleAr( int s0 );

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
  // After this many analyses in a row whose scaling function fit failed, a component's grain is no longer signalled
  // (three GOPs, see setAnalysesPerGop).
  int    m_maxFailedFits     = 3;

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
    double powerScale = 1.0;   // share of the measured grain power the synthesized grain gets (fitArFilter)
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
  std::vector<uint8_t> m_motionFailed[ MAX_NUM_COMP ];   // per 16x16 block: the temporal residual's compensation failed
  std::vector<double> m_fineEnergy[ MAX_NUM_COMP ];   // per 16x16 block: the original's energy at the finest frequencies
  std::vector<double> m_repeatedGrainRatio[ MAX_NUM_COMP ];   // per intensity bin: grain variance per variance the residual sees
  std::vector<int>  m_plateaus;   // luma values of the picture's grain-free plateaus (findPlateaus), most blocks first

  // Scratch planes and lists of the mask search, kept to avoid allocating and clearing them for every picture.
  struct Scratch
  {
    Plane            magnitude, direction, morph, half, quarter, lowMask;
    Plane            spatial;   // denoised minus original of the component being prepared, beside a temporal residual
    std::vector<int> positions, weakX, weakY, columnStart;
    std::vector<Pel> wide, top, bottom, left, right;
    std::vector<double> repeatedGrainRatios;   // per flat block, measureRepeatedGrain
    std::vector<std::vector<double>> repeatedGrainBinRatios;   // the same, per intensity bin
  };
  Scratch           m_scratch;

  // flat area mask
  void resetStatistics       ( int c );
  void findMask              ( ComponentID compId );
  void maskForScale          ( const Plane& picture, Plane& mask, int bitDepth, int numDilations );
  bool markMotionFailures    ( ComponentID compId );
  void measureFineDetail     ( ComponentID compId, const Plane& spatialGrain );
  void measureRepeatedGrain  ( ComponentID compId );
  void addSmoothBlockPoints  ( ComponentID compId, std::vector<int>& means, std::vector<int>& strengths ) const;

  // parameter estimation
  void estimateComponent     ( ComponentID compId );
  void estimateScalingFactors( std::vector<int>& dataX, std::vector<int>& dataY, int bitDepth, ComponentID compId );
  bool fitFunction           ( std::vector<int>& dataX, std::vector<int>& dataY, std::vector<double>& coeffs, std::vector<double>& scalingVec,
                               int order, int bitDepth, bool secondPass, ComponentID compId );
  void extendPoints          ( std::vector<int>& dataX, std::vector<int>& dataY, int bitDepth ) const;
  bool lloydMax              ( std::vector<double>& scalingVec, std::vector<int>& quantizedVec, int numQuantizedLevels, int bitDepth ) const;
  void setEstimatedParameters( std::vector<int>& quantizedVec, int bitDepth, ComponentID compId );
  void holdDarkStrength      ( std::vector<int>& quantizedVec, int bitDepth ) const;
  void mergeIntervals        ( std::vector<std::vector<int>>& parameters, int bitDepth ) const;
  void cutPlateauHoles       ( std::vector<std::vector<int>>& parameters, int bitDepth ) const;
  void scaleDown             ( std::vector<std::vector<int>>& parameters, int bitDepth ) const;

  void getVtmParameters      ( double strength, int& log2ScaleFactor, SeiFgc::CompModel compModel[ MAX_NUM_COMP ] ) const;
  void getArParameters       ( double strength, int& log2ScaleFactor, SeiFgc::CompModel compModel[ MAX_NUM_COMP ] ) const;
  bool arParametersAt        ( double strength, int log2ScaleFactor, SeiFgc::CompModel compModel[ MAX_NUM_COMP ], bool& usable ) const;
  void fitArFilter           ( ComponentID compId );

  void estimateCutoffFreq    ( ComponentID compId, int bitDepth );
  int  cutoffFrequency       ( const std::vector<double>& mean ) const;
  void fitCutoffPair         ( const std::vector<double>& meanSq, bool chroma420, int& horizontal, int& vertical ) const;
  void replaceCutoff         ( ComponentID compId, int& replacementH, int& replacementV );
  void limitCutoff           ( ComponentID compId, int avgH, int avgV );
  void limitCutoffConsecutive( ComponentID compId );
  int  limitCutoffPairs      ( std::vector<CutoffPair>& pairs, int maxUnique ) const;

  void initLostGrainTables   ();

  // Tables of the fit to the grain the coding lost, set by init() with the improved analysis: the basis functions of the
  // 64x64 tiles' DCT ([ sample ][ frequency ]), and per component type (luma, 4:2:0 chroma) and cut-off frequency the
  // shares of the synthesized grain's variance in the tile's coefficients along one direction.
  std::vector<float>  m_tileBasis;
  std::vector<double> m_synthesisMarginals;

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

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

#include "SEIFilmGrainAnalyzer.h"
#include "SEIFilmGrainSpectra.h"

#include "CommonLib/Rom.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace vvenc {

namespace {

// Constants of the VTM analysis.
constexpr int    MAXPAIRS                  = 256;
constexpr int    MAXORDER                  = 8;     // maximum order of the polynomial fit
constexpr int    MAX_REAL_SCALE            = 16;
constexpr int    ORDER                     = 4;     // order of the polynomial function
constexpr int    QUANT_LEVELS              = 4;     // quantization levels of the Lloyd-Max quantizer
constexpr int    INTERVAL_SIZE             = 8;
constexpr int    MIN_ELEMENTS_PER_INTERVAL = 8;
constexpr int    POINT_STEP                = 16;    // step size of the point extension
constexpr int    MAX_POINTS_TO_EXTEND      = 4;
constexpr double POINT_SCALE               = 1.25;  // scaling of the point extension (VTM; the improved analysis keeps the level)
constexpr double VAR_SCALE_DOWN            = 1.2;   // filter out points far below the fitted function
constexpr double VAR_SCALE_UP              = 0.6;   // filter out points far above the fitted function
constexpr int    NUM_PASSES                = 2;     // the second pass of the fit filters the data points
constexpr int    NBRS                      = 1;     // neighbours a point needs within WINDOW to be kept
constexpr int    WINDOW                    = 1;
constexpr int    MIN_MEASURED_POINTS       = 2;     // measured points the improved fit needs, before the point extension
constexpr int    MIN_INTENSITY             = 40;
constexpr int    MAX_INTENSITY             = 950;
constexpr int    MAX_STANDARD_DEVIATION    = 255;   // at 8 bits
// With the improvements, the grain strength is kept in units of a quarter of twice the standard deviation (VTM: twice
// the standard deviation), so that the scaling factors calibrated from it keep their precision for weak grain.
constexpr int    IMPROVED_STRENGTH_SCALE   = 4;
constexpr int    BIT_DEPTH_8               = 8;
constexpr int    MAX_ALLOWED_MODEL_VALUES  = 3;
constexpr int    MAX_NUM_INTENSITIES       = 256;
constexpr int    MAX_CUTOFF_PAIRS          = 10;    // different cut-off pairs the synthesis supports per picture
constexpr double MIN_BLOCK_WEIGHT          = 1.0;   // accumulated block weight an interval needs for cut-off estimation

// Calibration of the scaling factors, per component type (luma, 4:2:0 chroma) and cut-off pair, measured with the
// vvdec film grain synthesis on flat grey 1920x1080 10-bit pictures:
//  - PATTERN_STDDEV: standard deviation of the synthesized grain for scaling factor 64 at log2_scale_factor 2, so that
//    a scaling factor s at log2 scale factor l gives grain of standard deviation
//    s * PATTERN_STDDEV / 2^( l + 14 - bitDepth ) (the synthesis of VTM gives the same);
//  - BLOCK_VARIANCE_SHARE: the share of that grain's variance that remains after removing the mean of every 16x16
//    block, as the analysis measures it.
constexpr int    NUM_CUTOFFS               = 13;    // cut-off frequencies 2 to 14
// Largest scaling factor signalled. The syntax allows 255 and so does the scalar synthesis of vvdec, but its x86 SIMD
// synthesis sign-extends the 8-bit scaling factors, so 128 and above would make weaker grain of the wrong sign.
constexpr int    MAX_SCALING_FACTOR        = 127;
constexpr double PATTERN_STDDEV[ 2 ][ NUM_CUTOFFS ][ NUM_CUTOFFS ] =
{
  { // luma: [ horizontal - 2 ][ vertical - 2 ]
    { 11.3471, 13.1953, 14.4713, 16.0922, 17.5548, 18.7151, 20.0700, 20.9957, 21.8035, 22.8604, 23.8162, 24.4313, 25.4741 },
    { 12.7110, 14.8901, 16.3847, 18.3015, 19.9240, 21.3337, 22.8999, 24.0157, 25.0655, 26.3700, 27.3986, 28.3434, 29.3805 },
    { 14.1172, 16.5524, 18.2611, 20.5952, 22.4983, 24.0530, 25.9973, 27.3273, 28.5572, 30.0424, 31.1997, 32.1487, 33.2355 },
    { 15.5021, 18.0297, 20.0899, 22.4546, 24.4364, 26.2005, 28.2301, 29.8060, 31.3123, 32.9112, 34.1466, 35.3053, 36.4918 },
    { 16.4720, 19.2467, 21.3980, 23.9320, 26.0343, 28.0799, 30.3483, 32.0788, 33.6981, 35.4062, 36.7268, 37.9420, 39.4344 },
    { 17.4095, 20.4055, 22.9677, 25.5371, 27.8608, 30.1310, 32.3993, 34.2167, 35.9951, 37.9442, 39.4321, 40.6859, 42.2433 },
    { 18.1305, 21.4469, 24.0610, 26.8472, 29.2790, 31.7692, 34.0672, 36.0396, 37.8454, 39.9101, 41.3417, 42.7916, 44.3514 },
    { 19.2211, 22.7276, 25.4476, 28.2473, 30.8452, 33.3751, 35.7479, 37.6866, 39.5467, 41.7627, 43.3952, 44.7700, 46.4703 },
    { 20.4457, 24.1150, 27.0148, 29.9963, 32.7611, 35.2320, 37.6624, 39.8350, 41.6697, 43.8564, 45.5852, 46.9930, 48.9238 },
    { 21.6430, 25.4942, 28.5270, 31.8942, 34.6005, 37.0724, 39.4975, 41.7995, 43.5896, 45.7560, 47.5483, 49.0574, 50.9609 },
    { 22.5390, 26.4786, 29.5484, 32.9731, 35.8114, 38.4931, 41.0125, 43.2637, 45.0366, 47.2199, 49.1412, 50.7580, 52.6695 },
    { 23.4841, 27.5594, 30.6919, 34.2237, 37.1031, 39.7887, 42.3244, 44.6668, 46.6079, 48.6607, 50.7871, 52.3455, 54.2592 },
    { 24.0446, 28.2479, 31.5268, 35.1441, 38.0267, 40.7036, 43.3165, 45.7409, 47.7803, 49.8510, 51.9981, 53.5674, 55.5688 },
  },
  { // 4:2:0 chroma: [ horizontal - 2 ][ vertical - 2 ]
    {  9.8329, 11.6354, 12.8245, 13.6625, 15.3470, 16.4060, 16.4060, 16.4060, 16.4060, 16.4060, 16.4060, 16.4060, 16.4060 },
    { 11.7467, 14.0110, 15.6190, 16.6803, 18.4311, 19.4934, 19.4934, 19.4934, 19.4934, 19.4934, 19.4934, 19.4934, 19.4934 },
    { 12.8707, 15.3081, 17.2100, 18.5999, 20.4333, 21.5392, 21.5392, 21.5392, 21.5392, 21.5392, 21.5392, 21.5392, 21.5392 },
    { 14.0068, 16.3715, 18.4529, 19.9943, 21.8958, 23.3361, 23.3361, 23.3361, 23.3361, 23.3361, 23.3361, 23.3361, 23.3361 },
    { 15.2300, 17.7825, 20.0028, 21.6539, 23.5309, 25.4114, 25.4114, 25.4114, 25.4114, 25.4114, 25.4114, 25.4114, 25.4114 },
    { 16.5225, 19.3558, 21.5353, 23.2748, 25.1079, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849 },
    { 16.5225, 19.3558, 21.5353, 23.2748, 25.1079, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849 },
    { 16.5225, 19.3558, 21.5353, 23.2748, 25.1079, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849 },
    { 16.5225, 19.3558, 21.5353, 23.2748, 25.1079, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849 },
    { 16.5225, 19.3558, 21.5353, 23.2748, 25.1079, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849 },
    { 16.5225, 19.3558, 21.5353, 23.2748, 25.1079, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849 },
    { 16.5225, 19.3558, 21.5353, 23.2748, 25.1079, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849 },
    { 16.5225, 19.3558, 21.5353, 23.2748, 25.1079, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849, 27.1849 },
  },
};

constexpr double BLOCK_VARIANCE_SHARE[ 2 ][ NUM_CUTOFFS ][ NUM_CUTOFFS ] =
{
  { // luma: [ horizontal - 2 ][ vertical - 2 ]
    {  0.9137,  0.9350,  0.9456,  0.9547,  0.9619,  0.9646,  0.9688,  0.9716,  0.9733,  0.9758,  0.9777,  0.9789,  0.9805 },
    {  0.9299,  0.9481,  0.9568,  0.9642,  0.9699,  0.9724,  0.9756,  0.9778,  0.9795,  0.9813,  0.9828,  0.9839,  0.9850 },
    {  0.9429,  0.9577,  0.9650,  0.9715,  0.9762,  0.9780,  0.9809,  0.9828,  0.9840,  0.9855,  0.9867,  0.9875,  0.9883 },
    {  0.9520,  0.9641,  0.9708,  0.9757,  0.9796,  0.9815,  0.9837,  0.9854,  0.9867,  0.9879,  0.9888,  0.9895,  0.9902 },
    {  0.9568,  0.9680,  0.9740,  0.9784,  0.9818,  0.9836,  0.9858,  0.9873,  0.9884,  0.9894,  0.9902,  0.9908,  0.9915 },
    {  0.9609,  0.9709,  0.9770,  0.9807,  0.9839,  0.9856,  0.9874,  0.9886,  0.9896,  0.9907,  0.9914,  0.9919,  0.9925 },
    {  0.9634,  0.9733,  0.9786,  0.9822,  0.9851,  0.9868,  0.9883,  0.9896,  0.9904,  0.9914,  0.9920,  0.9926,  0.9931 },
    {  0.9674,  0.9761,  0.9809,  0.9839,  0.9866,  0.9881,  0.9894,  0.9905,  0.9913,  0.9921,  0.9928,  0.9933,  0.9938 },
    {  0.9711,  0.9788,  0.9830,  0.9857,  0.9881,  0.9893,  0.9904,  0.9914,  0.9921,  0.9929,  0.9934,  0.9939,  0.9945 },
    {  0.9744,  0.9811,  0.9848,  0.9875,  0.9894,  0.9904,  0.9914,  0.9923,  0.9929,  0.9936,  0.9941,  0.9945,  0.9950 },
    {  0.9762,  0.9824,  0.9859,  0.9883,  0.9901,  0.9911,  0.9920,  0.9929,  0.9934,  0.9940,  0.9945,  0.9948,  0.9953 },
    {  0.9782,  0.9838,  0.9869,  0.9891,  0.9907,  0.9916,  0.9925,  0.9933,  0.9938,  0.9943,  0.9948,  0.9951,  0.9956 },
    {  0.9792,  0.9846,  0.9876,  0.9897,  0.9911,  0.9920,  0.9928,  0.9936,  0.9941,  0.9946,  0.9951,  0.9954,  0.9959 },
  },
  { // 4:2:0 chroma: [ horizontal - 2 ][ vertical - 2 ]
    {  0.9488,  0.9625,  0.9686,  0.9723,  0.9781,  0.9805,  0.9805,  0.9805,  0.9805,  0.9805,  0.9805,  0.9805,  0.9805 },
    {  0.9621,  0.9727,  0.9776,  0.9805,  0.9840,  0.9855,  0.9855,  0.9855,  0.9855,  0.9855,  0.9855,  0.9855,  0.9855 },
    {  0.9684,  0.9770,  0.9817,  0.9842,  0.9870,  0.9881,  0.9881,  0.9881,  0.9881,  0.9881,  0.9881,  0.9881,  0.9881 },
    {  0.9733,  0.9798,  0.9839,  0.9862,  0.9886,  0.9897,  0.9897,  0.9897,  0.9897,  0.9897,  0.9897,  0.9897,  0.9897 },
    {  0.9772,  0.9829,  0.9862,  0.9881,  0.9901,  0.9914,  0.9914,  0.9914,  0.9914,  0.9914,  0.9914,  0.9914,  0.9914 },
    {  0.9807,  0.9855,  0.9882,  0.9899,  0.9914,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925 },
    {  0.9807,  0.9855,  0.9882,  0.9899,  0.9914,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925 },
    {  0.9807,  0.9855,  0.9882,  0.9899,  0.9914,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925 },
    {  0.9807,  0.9855,  0.9882,  0.9899,  0.9914,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925 },
    {  0.9807,  0.9855,  0.9882,  0.9899,  0.9914,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925 },
    {  0.9807,  0.9855,  0.9882,  0.9899,  0.9914,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925 },
    {  0.9807,  0.9855,  0.9882,  0.9899,  0.9914,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925 },
    {  0.9807,  0.9855,  0.9882,  0.9899,  0.9914,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925,  0.9925 },
  },
};

// After this many analyses in a row whose scaling function fit failed, a component's grain is no longer signalled.
constexpr int    MAX_FAILED_FITS           = 3;
// A picture whose mean grain strength differs from that of the accumulated data by more than this factor starts the
// statistics anew (a new scene with different grain); it needs this many blocks for a reliable mean.
constexpr double GRAIN_CHANGE_RATIO        = 2.0;
constexpr int    GRAIN_CHANGE_MIN_BLOCKS   = 64;

// The auto-regressive model (--fga 3): vvdec's synthesis makes grain of standard deviation
// scale * AR_PATTERN_STDDEV * sqrt( energy ) / 2^( log2ScaleFactor + bitDepth - 10 ) for the AR filter whose impulse
// response has the given energy (its quantized coefficients, as the synthesis computes them). Measured with vvdec on
// flat grey 1920x1088 10-bit pictures over 24 filters (left coefficient 0 to 0.375) and log2 scale factors 3 to 7:
// the ratio stays within 3% of these values.
constexpr double AR_PATTERN_STDDEV[ 2 ] = { 3.83, 3.60 };   // luma, chroma
constexpr int    AR_SPECTRUM_BINS         = 64;    // frequency bins per direction of the AR spectrum computation
constexpr int    AR_FIT_ITERATIONS        = 200;   // largest number of Nelder-Mead iterations of the AR filter fit
constexpr double AR_FIT_TOLERANCE         = 1e-5;  // the fit stops when its simplex spans less than this L1 error

// Outliers of the temporal residual: per intensity bin (16 bins), blocks above this ratio of the median grain variance
// are motion compensation failures; bins need this many blocks for the median.
constexpr int    NUM_OUTLIER_BINS          = 16;
constexpr double OUTLIER_VARIANCE_RATIO    = 4.0;
constexpr int    MIN_OUTLIER_BIN_BLOCKS    = 8;

// Temporal residual: motion search range in samples of the 4x downsampled pictures (12 or 20 samples at full
// resolution), and the share of the zero motion SAD a vector must stay below, as 17 / 20 = 0.85.
constexpr int    RESIDUAL_RANGE            = 3;
constexpr int    RESIDUAL_RANGE_WIDE       = 5;     // pictures wider than 2048
constexpr int    ZERO_BIAS_NUM             = 17;
constexpr int    ZERO_BIAS_DEN             = 20;
// A neighbour whose mean absolute difference is above twice that of the other neighbour, plus this many sample
// values, is left out of the residual.
constexpr double NEIGHBOUR_SAD_MARGIN      = 1.0;

constexpr double LOW_THRESHOLD_RATIO       = 0.1;   // Canny: low threshold relative to the strongest gradient
constexpr int    HIGH_THRESHOLD_RATIO      = 3;     // Canny: high threshold relative to the low threshold
constexpr double LOW_INTENSITY_RATIO       = 0.1;   // samples below this share of the range are not analysed

constexpr double TAP_FILTER[ 3 ]           = { 1, 2, 1 };
constexpr double NORM_TAP                  = 4.0;

typedef FGAnalyzer::Plane Plane;

// ====================================================================================================================
// Flat area detection: Canny edge detector and morphological operations
// ====================================================================================================================

// Classifies the magnitudes as strong, weak or no edge, with thresholds relative to the strongest gradient.
void doubleThreshold( const FgaRowKernels& k, Plane& mag, int bitDepth )
{
  const Pel strongPel = (Pel) ( ( 1 << bitDepth ) - 1 );
  const Pel weakPel   = (Pel) ( ( 1 << ( bitDepth - 1 ) ) - 1 );
  const int n = mag.width * mag.height;   // the storage can be larger than the plane

  const Pel highest       = k.rowMax( mag.samples.data(), n );
  const Pel lowThreshold  = (Pel) ( LOW_THRESHOLD_RATIO * highest );
  const Pel highThreshold = (Pel) Clip3( 0, ( 1 << bitDepth ) - 1, HIGH_THRESHOLD_RATIO * lowThreshold );
  k.classify( mag.samples.data(), n, lowThreshold, highThreshold, weakPel, strongPel );
}

// Hysteresis: a weak edge becomes strong when a strong edge is in its 3x3 neighbourhood, otherwise it is removed.
// As in VTM, the picture is updated in place, column by column, so a weak edge turned strong can promote weak edges
// processed after it. Neighbours outside the picture repeat the border as it was before the tracking started.
template<class ScratchT>
void edgeTracking( const FgaRowKernels& k, Plane& edges, int bitDepth, ScratchT& s )
{
  const Pel strongPel = (Pel) ( ( 1 << bitDepth ) - 1 );
  const Pel weakPel   = (Pel) ( ( 1 << ( bitDepth - 1 ) ) - 1 );
  const int w = edges.width, h = edges.height;

  // The weak edges in column-major order: found row by row, then sorted by column (counting sort, stable, so the
  // rows of every column stay in increasing order).
  s.positions.resize( w );
  s.weakX.clear();
  s.weakY.clear();
  for( int y = 0; y < h; y++ )
  {
    const int n = k.findEqual( edges.row( y ), w, weakPel, s.positions.data() );
    for( int i = 0; i < n; i++ )
    {
      s.weakX.push_back( s.positions[ i ] );
      s.weakY.push_back( y );
    }
  }
  if( s.weakX.empty() )
  {
    return;
  }
  std::vector<int>& columnStart = s.columnStart;
  columnStart.assign( w + 1, 0 );
  for( int px : s.weakX )
  {
    columnStart[ px + 1 ]++;
  }
  for( int px = 0; px < w; px++ )
  {
    columnStart[ px + 1 ] += columnStart[ px ];
  }
  std::vector<int>& weakRows = s.positions;
  weakRows.resize( s.weakX.size() );
  {
    std::vector<int> fill( columnStart.begin(), columnStart.end() - 1 );
    for( size_t i = 0; i < s.weakX.size(); i++ )
    {
      weakRows[ fill[ s.weakX[ i ] ]++ ] = s.weakY[ i ];
    }
  }

  // Neighbours outside the picture are border samples as they were before the tracking.
  s.top.assign( edges.row( 0 ), edges.row( 0 ) + w );
  s.bottom.assign( edges.row( h - 1 ), edges.row( h - 1 ) + w );
  s.left.resize( h );
  s.right.resize( h );
  for( int y = 0; y < h; y++ )
  {
    s.left[ y ]  = edges.row( y )[ 0 ];
    s.right[ y ] = edges.row( y )[ w - 1 ];
  }
  auto borderAt = [ & ]( int bx, int by )
  {
    const int cx = Clip3( 0, w - 1, bx ), cy = Clip3( 0, h - 1, by );
    return cy == 0 ? s.top[ cx ] : cy == h - 1 ? s.bottom[ cx ] : cx == 0 ? s.left[ cy ] : s.right[ cy ];
  };

  for( int x = 0; x < w; x++ )
  {
    for( int i = columnStart[ x ]; i < columnStart[ x + 1 ]; i++ )
    {
      const int y = weakRows[ i ];
      bool strong = false;
      for( int ny = y - 1; ny <= y + 1 && !strong; ny++ )
      {
        for( int nx = x - 1; nx <= x + 1 && !strong; nx++ )
        {
          const bool inside = nx >= 0 && nx < w && ny >= 0 && ny < h;
          strong = ( inside ? edges.row( ny )[ nx ] : borderAt( nx, ny ) ) == strongPel;
        }
      }
      edges.row( y )[ x ] = strong ? strongPel : 0;
    }
  }
}

// Sets the binary mask to "edge" where the picture is too dark to be analysed.
void suppressLowIntensity( const Plane& p, Plane& mask, int bitDepth )
{
  const Pel maxIntensity = (Pel) ( ( 1 << bitDepth ) - 1 );
  const Pel threshold    = (Pel) ( LOW_INTENSITY_RATIO * maxIntensity );
  const size_t n = (size_t) mask.width * mask.height;
  const Pel* src = p.samples.data();
  Pel* dst = mask.samples.data();
  for( size_t i = 0; i < n; i++ )
  {
    dst[ i ] = src[ i ] < threshold ? maxIntensity : dst[ i ];   // a select, so that the loop vectorizes
  }
}

// Subsampling as in VTM: the average of the top-left 2x2 samples of every factor x factor block.
void subsample( const Plane& in, Plane& out, int factor )
{
  out.resize( in.width / factor, in.height / factor );
  for( int y = 0; y < out.height; y++ )
  {
    const Pel* r0 = in.row( y * factor );
    const Pel* r1 = in.row( y * factor + 1 );
    Pel* dst = out.row( y );
    for( int x = 0; x < out.width; x++ )
    {
      const int sx = x * factor;
      dst[ x ] = (Pel) ( ( r0[ sx ] + r1[ sx ] + r0[ sx + 1 ] + r1[ sx + 1 ] + 2 ) >> 2 );
    }
  }
}

// Adds a mask found at a lower resolution to the full-resolution mask (sample replication, then bitwise or).
void combineUpsampled( Plane& mask, const Plane& low, int factor, std::vector<Pel>& wide )
{
  const int w = low.width * factor;
  wide.resize( w );
  for( int ly = 0; ly < low.height; ly++ )
  {
    const Pel* src = low.row( ly );
    for( int lx = 0; lx < low.width; lx++ )
    {
      for( int f = 0; f < factor; f++ )
      {
        wide[ lx * factor + f ] = src[ lx ];
      }
    }
    for( int y = ly * factor; y < ( ly + 1 ) * factor; y++ )
    {
      Pel* dst = mask.row( y );
      for( int x = 0; x < w; x++ )
      {
        dst[ x ] |= wide[ x ];
      }
    }
  }
}

// ====================================================================================================================
// Block statistics
// ====================================================================================================================

struct BlockStats
{
  int    mean;     // truncated, as in VTM (it selects the intensity)
  double stddev;   // exact; VTM truncates it to an integer, which loses up to a third of the grain strength
};

// Mean and standard deviation of a 16x16 block.
BlockStats meanStdDev( const FgaRowKernels& k, const Plane& p, int x0, int y0 )
{
  int64_t sum = 0, sumSq = 0;
  k.blockSums( p.row( y0 ) + x0, p.width, sum, sumSq );
  const int n = 16 * 16;
  const double variance = ( (double) sumSq - (double) sum * sum / n ) / n;
  return { (int) ( sum / n ), std::sqrt( std::max( variance, 0.0 ) ) };
}

// Expected power of the coefficients of the orthonormal 16x16 DCT-II of grain made by the AR filter of vvdec's SEI AR
// mode (see FGAnalyzer::ArFilter), as spec[ vertical * 16 + horizontal ]: its power spectrum 1 / |1 - A(w)|^2,
// integrated against the squared spectra of the DCT basis vectors on AR_SPECTRUM_BINS x AR_SPECTRUM_BINS frequencies.
class ArSpectrum
{
public:
  ArSpectrum()
  {
    constexpr int    N  = 16;
    constexpr double PI = 3.14159265358979323846;
    for( int k = 0; k < AR_SPECTRUM_BINS; k++ )
    {
      const double w = 2.0 * PI * k / AR_SPECTRUM_BINS;
      m_cos[ k ] = std::cos( w );
      m_sin[ k ] = std::sin( w );
      for( int u = 0; u < N; u++ )
      {
        double re = 0.0, im = 0.0;
        for( int n = 0; n < N; n++ )
        {
          const double b = std::sqrt( ( u ? 2.0 : 1.0 ) / N ) * std::cos( PI * ( 2 * n + 1 ) * u / ( 2 * N ) );
          re += b * std::cos( w * n );
          im -= b * std::sin( w * n );
        }
        m_basisPower[ u ][ k ] = re * re + im * im;
      }
    }
  }

  // Scratch space of compute(), reused by the caller over the calls of a fit.
  static constexpr size_t SCRATCH_SIZE = AR_SPECTRUM_BINS * AR_SPECTRUM_BINS + 16 * AR_SPECTRUM_BINS;

  // spec: 256 values, the DC coefficient set to 0 and the rest normalized to sum 1; returns the share of the power
  // outside the DC coefficient. scratch: SCRATCH_SIZE values.
  double compute( double a1, double a3, double a5, double r, double* spec, std::vector<double>& scratch ) const
  {
    constexpr int N = 16, K = AR_SPECTRUM_BINS;
    scratch.resize( SCRATCH_SIZE );
    double* psd  = scratch.data();
    double* rows = psd + K * K;
    for( int ky = 0; ky < K; ky++ )
    {
      for( int kx = 0; kx < K; kx++ )
      {
        // A(w) = sum of coefficient * e^(-i w . offset) over the filter taps
        const int sum = ( kx + ky ) % K, diff = ( ky - kx + K ) % K, x2 = ( 2 * kx ) % K, y2 = ( 2 * ky ) % K;
        const double re = a1 * m_cos[ kx ] + a1 * r * m_cos[ ky ] + a3 * r * ( m_cos[ sum ] + m_cos[ diff ] )
                        + a5 * m_cos[ x2 ] + a5 * r * r * m_cos[ y2 ];
        const double im = -( a1 * m_sin[ kx ] + a1 * r * m_sin[ ky ] + a3 * r * ( m_sin[ sum ] + m_sin[ diff ] )
                        + a5 * m_sin[ x2 ] + a5 * r * r * m_sin[ y2 ] );
        psd[ ky * K + kx ] = 1.0 / std::max( ( 1.0 - re ) * ( 1.0 - re ) + im * im, 1e-6 );
      }
    }
    for( int v = 0; v < N; v++ )
    {
      for( int kx = 0; kx < K; kx++ )
      {
        double s = 0.0;
        for( int ky = 0; ky < K; ky++ )
        {
          s += m_basisPower[ v ][ ky ] * psd[ ky * K + kx ];
        }
        rows[ v * K + kx ] = s;
      }
    }
    double total = 0.0;
    for( int v = 0; v < N; v++ )
    {
      for( int h = 0; h < N; h++ )
      {
        double s = 0.0;
        for( int kx = 0; kx < K; kx++ )
        {
          s += rows[ v * K + kx ] * m_basisPower[ h ][ kx ];
        }
        spec[ v * N + h ] = s;
        total += s;
      }
    }
    const double nonDc = total - spec[ 0 ];
    spec[ 0 ] = 0.0;
    for( int i = 0; i < N * N; i++ )
    {
      spec[ i ] /= nonDc;
    }
    return nonDc / total;
  }

private:
  double m_cos[ AR_SPECTRUM_BINS ], m_sin[ AR_SPECTRUM_BINS ];
  double m_basisPower[ 16 ][ AR_SPECTRUM_BINS ];
};

// The filter coefficients of vvdec's SEI AR mode as it computes them, in units of 2^-log2: left, top, diagonal
// (top-left and top-right), left-left and top-top.
struct ArTaps
{
  int left, top, diag, leftLeft, topTop;
  ArTaps( const int c[ 6 ], int log2 )
    : left( c[ 1 ] ), top( ( c[ 1 ] * c[ 4 ] ) >> log2 ), diag( ( c[ 3 ] * c[ 4 ] ) >> log2 ), leftLeft( c[ 5 ] ),
      topTop( ( c[ 5 ] * c[ 4 ] * c[ 4 ] ) >> ( 2 * log2 ) ) {}
};

// Energy of the impulse response of vvdec's AR filter with the signalled (integer) coefficients c[ 1 ] to c[ 5 ].
double arEnergy( const int c[ 6 ], int log2 )
{
  constexpr int SIZE = 96;
  const ArTaps t( c, log2 );
  const double one = 1 << log2;
  std::vector<double> h( SIZE * SIZE, 0.0 );
  double energy = 0.0;
  for( int y = 0; y < SIZE; y++ )
  {
    for( int x = 2; x < SIZE - 2; x++ )
    {
      double g = t.left * h[ y * SIZE + x - 1 ] + t.leftLeft * h[ y * SIZE + x - 2 ];
      if( y >= 1 )
      {
        g += t.top * h[ ( y - 1 ) * SIZE + x ] + t.diag * ( h[ ( y - 1 ) * SIZE + x - 1 ] + h[ ( y - 1 ) * SIZE + x + 1 ] );
      }
      if( y >= 2 )
      {
        g += t.topTop * h[ ( y - 2 ) * SIZE + x ];
      }
      h[ y * SIZE + x ] = g / one + ( y == 0 && x == SIZE / 2 ? 1.0 : 0.0 );
      energy += h[ y * SIZE + x ] * h[ y * SIZE + x ];
    }
  }
  return energy;
}

double ldpow( double n, unsigned p )
{
  double x = 1;
  for( unsigned i = 0; i < p; i++ )
  {
    x = x * n;
  }
  return x;
}

Pel rowMax( const Pel* src, int n )
{
  return fgaRowMaxCore( src, n );
}

void classify( Pel* row, int n, Pel low, Pel high, Pel weak, Pel strong )
{
  fgaClassifyCore( row, n, low, high, weak, strong );
}

void sobelRow( const Pel* above, const Pel* cur, const Pel* below, int width, Pel maxVal, Pel* mag, Pel* dir )
{
  fgaSobelCore( above, cur, below, width, 0, width, maxVal, mag, dir );
}

void nonMaxRow( const Pel* magAbove, const Pel* mag, const Pel* magBelow, const Pel* dir, int width, Pel* dst )
{
  fgaNonMaxCore( magAbove, mag, magBelow, dir, width, 0, width, dst );
}

void morphRow( const Pel* src, int width, bool dilate, Pel* dst )
{
  fgaMorphRowCore( src, width, 0, width, dilate, dst );
}

void morphCol( const Pel* above, const Pel* cur, const Pel* below, int width, bool dilate, Pel* dst )
{
  fgaMorphColCore( above, cur, below, 0, width, dilate, dst );
}

// Gradient magnitude after non-maximum suppression along the edge direction.
void gradientWithNonMaxSuppression( const FgaRowKernels& k, const Plane& p, Plane& out, int bitDepth, Plane& mag, Plane& dir )
{
  const int w = p.width, h = p.height;
  const Pel maxVal = (Pel) ( ( 1 << bitDepth ) - 1 );
  mag.resize( w, h );
  dir.resize( w, h );
  for( int y = 0; y < h; y++ )
  {
    k.sobel( p.row( std::max( y - 1, 0 ) ), p.row( y ), p.row( std::min( y + 1, h - 1 ) ), w, maxVal, mag.row( y ), dir.row( y ) );
  }
  out.resize( w, h );
  for( int y = 0; y < h; y++ )
  {
    k.nonMax( mag.row( std::max( y - 1, 0 ) ), mag.row( y ), mag.row( std::min( y + 1, h - 1 ) ), dir.row( y ), w, out.row( y ) );
  }
}

// One 3x3 dilation or erosion of a binary mask (values 0 and "edge"): on such a mask, dilation is the 3x3 maximum and
// erosion the 3x3 minimum, and both are separable. Borders are replicated.
void morphStep( const FgaRowKernels& k, Plane& mask, Plane& tmp, bool dilate )
{
  const int w = mask.width, h = mask.height;
  tmp.resize( w, h );
  for( int y = 0; y < h; y++ )
  {
    k.morphRow( mask.row( y ), w, dilate, tmp.row( y ) );
  }
  for( int y = 0; y < h; y++ )
  {
    k.morphCol( tmp.row( std::max( y - 1, 0 ) ), tmp.row( y ), tmp.row( std::min( y + 1, h - 1 ) ), w, dilate, mask.row( y ) );
  }
}

} // anonymous namespace

// ====================================================================================================================
// Scalar kernels
// ====================================================================================================================

Pel fgaRowMaxCore( const Pel* src, int n )
{
  Pel highest = 0;
  for( int i = 0; i < n; i++ )
  {
    highest = std::max( highest, src[ i ] );
  }
  return highest;
}

void fgaClassifyCore( Pel* row, int n, Pel low, Pel high, Pel weak, Pel strong )
{
  for( int i = 0; i < n; i++ )
  {
    row[ i ] = row[ i ] > high ? strong : row[ i ] > low ? weak : 0;
  }
}

int fgaFindEqualCore( const Pel* row, int n, Pel value, int* positions )
{
  int count = 0;
  for( int i = 0; i < n; i++ )
  {
    if( row[ i ] == value )
    {
      positions[ count++ ] = i;
    }
  }
  return count;
}

bool fgaIsZeroBlockCore( const Pel* src, int stride )
{
  for( int y = 0; y < 16; y++, src += stride )
  {
    for( int x = 0; x < 16; x++ )
    {
      if( src[ x ] )
      {
        return false;
      }
    }
  }
  return true;
}

void fgaBlockSumsCore( const Pel* src, int stride, int64_t& sum, int64_t& sumSq )
{
  sum = sumSq = 0;
  for( int y = 0; y < 16; y++, src += stride )
  {
    for( int x = 0; x < 16; x++ )
    {
      sum   += src[ x ];
      sumSq += src[ x ] * src[ x ];
    }
  }
}

void fgaDownsample4Core( const Pel* src, int stride, int n, Pel* dst )
{
  for( int x = 0; x < n; x++, src += 4 )
  {
    int sum = 8;
    for( int y = 0; y < 4; y++ )
    {
      sum += src[ y * stride ] + src[ y * stride + 1 ] + src[ y * stride + 2 ] + src[ y * stride + 3 ];
    }
    dst[ x ] = (Pel) ( sum >> 4 );
  }
}

void fgaSad4x4RowCore( const Pel* cur, int curStride, const Pel* ref, int refStride, int n, int* sad )
{
  for( int b = 0; b < n; b++ )
  {
    int sum = 0;
    for( int y = 0; y < 4; y++ )
    {
      for( int x = 4 * b; x < 4 * b + 4; x++ )
      {
        sum += abs( cur[ y * curStride + x ] - ref[ y * refStride + x ] );
      }
    }
    sad[ b ] = sum;
  }
}

void fgaResidualRowCore( const Pel* cur, const Pel* prev, const Pel* next, int n, Pel* dst )
{
  if( next )
  {
    for( int x = 0; x < n; x++ )
    {
      dst[ x ] = (Pel) ( 2 * cur[ x ] - prev[ x ] - next[ x ] );
    }
  }
  else
  {
    for( int x = 0; x < n; x++ )
    {
      dst[ x ] = (Pel) ( cur[ x ] - prev[ x ] );
    }
  }
}

// VVC's 16-point DCT-II as a matrix multiplication, as VTM's film grain analysis computes it.
void fgaSquaredDct16Core( const Pel* src, int stride, int maxRange, double* sum )
{
  constexpr int N = 16;
  const TMatrixCoeff* tr = g_trCoreDCT2P16[ TRANSFORM_FORWARD ][ 0 ];
  int horizontal[ N ][ N ];   // [horizontal frequency][row]
  for( int u = 0; u < N; u++ )
  {
    for( int y = 0; y < N; y++ )
    {
      const Pel* row = src + y * stride;
      int acc = 0;
      for( int k = 0; k < N; k++ )
      {
        acc += tr[ u * N + k ] * row[ k ];
      }
      horizontal[ u ][ y ] = ( acc + 128 ) >> 8;
    }
  }
  for( int u = 0; u < N; u++ )
  {
    for( int v = 0; v < N; v++ )
    {
      int acc = 0;
      for( int k = 0; k < N; k++ )
      {
        acc += horizontal[ u ][ k ] * tr[ v * N + k ];
      }
      const int64_t c = Clip3( -maxRange, maxRange, ( acc + 128 ) >> 8 );
      sum[ u * N + v ] += (double) ( c * c );
    }
  }
}

// Sobel gradients as in VTM: "gx" is the difference of the rows below and above (a vertical derivative), "gy" the
// difference of the columns right and left. VTM quantizes the angle atan2(gx, gy) to 0, 45, 90 or 135 degrees with
// bounds at 22.5 + k * 45 degrees. As tan(22.5) = sqrt(2) - 1 and tan(67.5) = sqrt(2) + 1, with a = |gx| and b = |gy|
//   a <= (sqrt(2) - 1) * b  <=>  (a + b)^2 <= 2 * b^2           : 0 degrees,
//   a >  (sqrt(2) + 1) * b  <=>  a > b and (a - b)^2 > 2 * b^2  : 90 degrees,
// otherwise 45 degrees when gx and gy have the same sign and 135 when not. This integer rule gives the same direction
// as atan2 for every 10-bit gradient pair (checked exhaustively) and needs no floating point.
void fgaSobelCore( const Pel* above, const Pel* cur, const Pel* below, int width, int xStart, int xEnd, Pel maxVal, Pel* mag, Pel* dir )
{
  for( int x = xStart; x < xEnd; x++ )
  {
    const int l = std::max( x - 1, 0 ), r = std::min( x + 1, width - 1 );
    const int gx = ( below[ l ] + 2 * below[ x ] + below[ r ] ) - ( above[ l ] + 2 * above[ x ] + above[ r ] );
    const int gy = ( above[ r ] + 2 * cur[ r ] + below[ r ] ) - ( above[ l ] + 2 * cur[ l ] + below[ l ] );
    const int a = abs( gx ), b = abs( gy );
    mag[ x ] = (Pel) std::min<int>( ( a + b ) / 2, maxVal );
    if( ( a + b ) * ( a + b ) <= 2 * b * b )
    {
      dir[ x ] = 0;
    }
    else if( a > b && ( a - b ) * ( a - b ) > 2 * b * b )
    {
      dir[ x ] = 90;
    }
    else
    {
      dir[ x ] = ( gx ^ gy ) >= 0 ? 45 : 135;
    }
  }
}

// Neighbours along the direction: 0 degrees compares left and right, 45 lower right and upper left, 90 above and
// below, 135 lower left and upper right.
void fgaNonMaxCore( const Pel* magAbove, const Pel* mag, const Pel* magBelow, const Pel* dir, int width, int xStart, int xEnd, Pel* dst )
{
  for( int x = xStart; x < xEnd; x++ )
  {
    const int l = std::max( x - 1, 0 ), r = std::min( x + 1, width - 1 );
    Pel n1, n2;
    switch( dir[ x ] )
    {
    case 0:  n1 = mag[ r ];      n2 = mag[ l ];      break;
    case 45: n1 = magBelow[ r ]; n2 = magAbove[ l ]; break;
    case 90: n1 = magBelow[ x ]; n2 = magAbove[ x ]; break;
    default: n1 = magBelow[ l ]; n2 = magAbove[ r ]; break;
    }
    dst[ x ] = mag[ x ] < n1 || mag[ x ] < n2 ? 0 : mag[ x ];
  }
}

void fgaMorphRowCore( const Pel* src, int width, int xStart, int xEnd, bool dilate, Pel* dst )
{
  for( int x = xStart; x < xEnd; x++ )
  {
    const Pel l = src[ std::max( x - 1, 0 ) ], r = src[ std::min( x + 1, width - 1 ) ];
    dst[ x ] = dilate ? std::max( { l, src[ x ], r } ) : std::min( { l, src[ x ], r } );
  }
}

void fgaMorphColCore( const Pel* above, const Pel* cur, const Pel* below, int xStart, int xEnd, bool dilate, Pel* dst )
{
  for( int x = xStart; x < xEnd; x++ )
  {
    dst[ x ] = dilate ? std::max( { above[ x ], cur[ x ], below[ x ] } ) : std::min( { above[ x ], cur[ x ], below[ x ] } );
  }
}

Pel FGAnalyzer::Plane::clampedAt( int x, int y ) const
{
  return at( Clip3( 0, width - 1, x ), Clip3( 0, height - 1, y ) );
}

// ====================================================================================================================
// Initialization
// ====================================================================================================================

FGAnalyzer::FGAnalyzer()
{
  initKernels( m_kernels );
}

void FGAnalyzer::initKernels( FgaRowKernels& k )
{
  k.sobel    = sobelRow;
  k.nonMax   = nonMaxRow;
  k.morphRow = morphRow;
  k.morphCol = morphCol;
  k.rowMax   = rowMax;
  k.classify = classify;
  k.findEqual = fgaFindEqualCore;
  k.isZeroBlock  = fgaIsZeroBlockCore;
  k.blockSums    = fgaBlockSumsCore;
  k.squaredDct16 = fgaSquaredDct16Core;
  k.downsample4  = fgaDownsample4Core;
  k.sad4x4Row    = fgaSad4x4RowCore;
  k.residualRow  = fgaResidualRowCore;
#if ENABLE_SIMD_OPT_FGA && defined( TARGET_SIMD_X86 )
  initFGAnalyzerX86( k );
#endif
#if ENABLE_SIMD_OPT_FGA && defined( TARGET_SIMD_ARM )
  initFGAnalyzerARM( k );
#endif
}

void FGAnalyzer::init( int width, int height, ChromaFormat chromaFormat, const int bitDepths[ MAX_NUM_CH ], const bool doAnalysis[ MAX_NUM_COMP ], bool improved, bool arModel )
{
  m_improved      = improved || arModel;
  m_arModel       = arModel;
  for( ArFilter& f : m_arFilter )
  {
    f = ArFilter();
  }
  m_strengthScale = m_improved ? IMPROVED_STRENGTH_SCALE : 1;
  m_width        = width;
  m_height       = height;
  m_chromaFormat = chromaFormat;
  for( int ch = 0; ch < MAX_NUM_CH; ch++ )
  {
    m_bitDepths[ ch ] = bitDepths[ ch ];
  }

  // No grain is signalled until an analysis has measured it. VTM signals default parameters (scaling factor 16 for
  // luma and 8 for chroma) until its first successful fit, which adds strong grain to video without grain, where the
  // fit never succeeds.
  for( int c = 0; c < MAX_NUM_COMP; c++ )
  {
    SeiFgc::CompModel& cm   = m_compModel[ c ];
    cm.presentFlag           = false;
    cm.numModelValues        = 1;
    cm.numIntensityIntervals = 0;
    cm.intensityValues.resize( MAX_NUM_INTENSITIES );
    for( auto& iv : cm.intensityValues )
    {
      iv.intensityIntervalLowerBound = 0;
      iv.intensityIntervalUpperBound = 0;
      iv.compModelValue.assign( MAX_ALLOWED_MODEL_VALUES, 0 );
    }
    m_doAnalysis[ c ]  = doAnalysis[ c ];
    m_cutoffPairs[ c ] = 0;
    m_failedFits [ c ] = 0;
    resetStatistics( c );
  }
}

// Drops the data accumulated over the analysed pictures of a component.
void FGAnalyzer::resetStatistics( int c )
{
  const int numMeans = 1 << m_bitDepths[ toChannelType( ComponentID( c ) ) ];
  m_squaredDctSums[ c ].assign( numMeans, SquaredDctSum( BLOCK_SIZE * BLOCK_SIZE, 0.0 ) );
  m_numDctBlocks  [ c ].assign( numMeans, 0.0 );
  m_storedMeanIntensity[ c ].clear();
  m_storedStrengthIntensity[ c ].clear();
  m_storedElementsPerInterval[ c ].clear();
}

// ====================================================================================================================
// Temporal residual
// ====================================================================================================================

FgaTemporalResidual::FgaTemporalResidual()
{
  FGAnalyzer::initKernels( m_kernels );
}

void FgaTemporalResidual::downsample( const CPelBuf& src, FGAnalyzer::Plane& dst ) const
{
  dst.resize( src.width / 4, src.height / 4 );
  for( int y = 0; y < dst.height; y++ )
  {
    m_kernels.downsample4( src.bufAt( 0, 4 * y ), (int) src.stride, dst.width, dst.row( y ) );
  }
}

void FgaTemporalResidual::search( const FGAnalyzer::Plane& cur, const FGAnalyzer::Plane& ref, int numBlocksX, int numBlocksY, Motion& motion ) const
{
  const int numBlocks = numBlocksX * numBlocksY;
  const int range     = cur.width * 4 > 2048 ? RESIDUAL_RANGE_WIDE : RESIDUAL_RANGE;
  motion.dx.assign( numBlocks, 0 );
  motion.dy.assign( numBlocks, 0 );
  std::vector<int> bestSad( numBlocks ), bestScore( numBlocks ), sad( numBlocksX );

  // zero motion; another vector must have a SAD below ZERO_BIAS_NUM / ZERO_BIAS_DEN of it
  for( int by = 0; by < numBlocksY; by++ )
  {
    int* s = &bestSad[ by * numBlocksX ];
    m_kernels.sad4x4Row( cur.row( 4 * by ), cur.width, ref.row( 4 * by ), ref.width, numBlocksX, s );
    for( int bx = 0; bx < numBlocksX; bx++ )
    {
      bestScore[ by * numBlocksX + bx ] = s[ bx ] * ZERO_BIAS_NUM;
    }
  }
  for( int dy = -range; dy <= range; dy++ )
  {
    for( int dx = -range; dx <= range; dx++ )
    {
      if( dx == 0 && dy == 0 )
      {
        continue;
      }
      // the blocks whose displaced block lies inside the reference
      const int bx0 = dx < 0 ? ( 3 - dx ) / 4 : 0;
      const int bx1 = ref.width - 4 - dx < 0 ? 0 : std::min( numBlocksX, ( ref.width - 4 - dx ) / 4 + 1 );
      if( bx1 <= bx0 )
      {
        continue;
      }
      for( int by = 0; by < numBlocksY; by++ )
      {
        const int ry = 4 * by + dy;
        if( ry < 0 || ry + 4 > ref.height )
        {
          continue;
        }
        m_kernels.sad4x4Row( cur.row( 4 * by ) + 4 * bx0, cur.width, ref.row( ry ) + 4 * bx0 + dx, ref.width, bx1 - bx0, sad.data() );
        for( int bx = bx0; bx < bx1; bx++ )
        {
          const int b = by * numBlocksX + bx;
          const int score = sad[ bx - bx0 ] * ZERO_BIAS_DEN;
          if( score < bestScore[ b ] )
          {
            bestScore[ b ] = score;
            bestSad  [ b ] = sad[ bx - bx0 ];
            motion.dx[ b ] = (int8_t) dx;
            motion.dy[ b ] = (int8_t) dy;
          }
        }
      }
    }
  }
  int64_t sum = 0;
  for( int s : bestSad )
  {
    sum += s;
  }
  motion.meanSad = (double) sum / ( 16.0 * numBlocks );
}

int FgaTemporalResidual::compute( const CPelUnitBuf& cur, const CPelUnitBuf* prev, const CPelUnitBuf* next, PelStorage& residual ) const
{
  const CPelBuf curY = cur.get( COMP_Y );
  const int numBlocksX = curY.width  / 16;
  const int numBlocksY = curY.height / 16;
  if( ( !prev && !next ) || numBlocksX == 0 || numBlocksY == 0 )
  {
    return 0;
  }

  FGAnalyzer::Plane curQ, refQ;
  downsample( curY, curQ );
  Motion motionPrev, motionNext;
  if( prev )
  {
    downsample( prev->get( COMP_Y ), refQ );
    search( curQ, refQ, numBlocksX, numBlocksY, motionPrev );
  }
  if( next )
  {
    downsample( next->get( COMP_Y ), refQ );
    search( curQ, refQ, numBlocksX, numBlocksY, motionNext );
  }
  if( prev && next )
  {
    if( motionPrev.meanSad > 2 * motionNext.meanSad + NEIGHBOUR_SAD_MARGIN )
    {
      prev = nullptr;
    }
    else if( motionNext.meanSad > 2 * motionPrev.meanSad + NEIGHBOUR_SAD_MARGIN )
    {
      next = nullptr;
    }
  }
  // the first neighbour of the residual, and the second one if both are used
  const CPelUnitBuf* first  = prev ? prev : next;
  const CPelUnitBuf* second = prev ? next : nullptr;
  const Motion& motionFirst  = prev ? motionPrev : motionNext;
  const Motion& motionSecond = motionNext;

  residual.destroy();
  residual.create( cur.chromaFormat, Area( 0, 0, curY.width, curY.height ) );
  residual.fill( 0 );   // the samples outside the 16x16 blocks are not analysed
  for( int c = 0; c < getNumberValidComponents( cur.chromaFormat ); c++ )
  {
    const ComponentID compId = ComponentID( c );
    const int csx = getComponentScaleX( compId, cur.chromaFormat );
    const int csy = getComponentScaleY( compId, cur.chromaFormat );
    const int bw  = 16 >> csx;
    const int bh  = 16 >> csy;
    const CPelBuf o  = cur.get( compId );
    const CPelBuf f  = first->get( compId );
    const CPelBuf s  = second ? second->get( compId ) : CPelBuf();
    PelBuf        r  = residual.get( compId );
    for( int by = 0; by < numBlocksY; by++ )
    {
      for( int bx = 0; bx < numBlocksX; bx++ )
      {
        const int b  = by * numBlocksX + bx;
        const int x0 = bx * bw;
        const int y0 = by * bh;
        // vectors in samples of this component (4 full resolution samples per downsampled sample)
        const int fx = ( 4 * motionFirst.dx[ b ] ) >> csx, fy = ( 4 * motionFirst.dy[ b ] ) >> csy;
        const int sx = second ? ( 4 * motionSecond.dx[ b ] ) >> csx : 0;
        const int sy = second ? ( 4 * motionSecond.dy[ b ] ) >> csy : 0;
        for( int y = 0; y < bh; y++ )
        {
          m_kernels.residualRow( o.bufAt( x0, y0 + y ), f.bufAt( x0 + fx, y0 + y + fy ),
                                 second ? s.bufAt( x0 + sx, y0 + y + sy ) : nullptr, bw, r.bufAt( x0, y0 + y ) );
        }
      }
    }
  }
  return second ? 6 : 2;
}

// VTM's scaling factors: twice the standard deviation, at log2 scale factor 2. As in getSignalledParameters(), the 4:2:0
// chroma cut-off frequencies are halved and the scaling factors stay at MAX_SCALING_FACTOR or below.
void FGAnalyzer::getVtmParameters( double strength, int& log2ScaleFactor, SeiFgc::CompModel compModel[ MAX_NUM_COMP ] ) const
{
  log2ScaleFactor = 2;
  for( int c = 0; c < MAX_NUM_COMP; c++ )
  {
    SeiFgc::CompModel& cm = compModel[ c ];
    cm = m_compModel[ c ];
    if( !cm.presentFlag || strength <= 0.0 )
    {
      cm.presentFlag = false;
      continue;
    }
    const bool chroma420 = c != COMP_Y && m_chromaFormat == CHROMA_420;
    const int  bitDepth  = m_bitDepths[ toChannelType( ComponentID( c ) ) ];
    bool anyGrain = false;
    for( int i = 0; i < cm.numIntensityIntervals; i++ )
    {
      std::vector<int>& v = cm.intensityValues[ i ].compModelValue;
      for( int k = 1; chroma420 && k < cm.numModelValues; k++ )
      {
        v[ k ] = Clip3( 2, 14, ( v[ k ] + 1 ) >> 1 );
      }
      const int scale = ( v[ 0 ] << log2ScaleFactor ) >> ( bitDepth - BIT_DEPTH_8 );
      v[ 0 ]   = std::min( MAX_SCALING_FACTOR, (int) ( scale * strength + 0.5 ) );
      anyGrain = anyGrain || v[ 0 ] > 0;
    }
    cm.presentFlag = anyGrain;
  }
}

// The parameters of the auto-regressive model: model values scale, a1, 0 (no correlation between the components), a3,
// r and a5, the last four in units of 2^-log2ScaleFactor. The scaling factors are calibrated with the impulse response
// energy of the quantized filter (AR_PATTERN_STDDEV); the log2 scale factor is the largest (2 to 7) at which they stay
// at MAX_SCALING_FACTOR or below, for the finest steps of both the scaling factors and the filter coefficients.
void FGAnalyzer::getArParameters( double strength, int& log2ScaleFactor, SeiFgc::CompModel compModel[ MAX_NUM_COMP ] ) const
{
  for( log2ScaleFactor = 7; log2ScaleFactor >= 2; log2ScaleFactor-- )
  {
    const int one = 1 << log2ScaleFactor;
    bool fits = true;
    for( int c = 0; c < MAX_NUM_COMP; c++ )
    {
      SeiFgc::CompModel& cm = compModel[ c ];
      cm = m_compModel[ c ];
      const ArFilter& f = m_arFilter[ c ];
      if( !cm.presentFlag || strength <= 0.0 || !f.valid )
      {
        cm.presentFlag = false;
        continue;
      }
      const int coef[ 6 ] = { 0, (int) std::lround( f.a1 * one ), 0, (int) std::lround( f.a3 * one ), (int) std::lround( f.r * one ),
                              (int) std::lround( f.a5 * one ) };
      const int bitDepth = m_bitDepths[ toChannelType( ComponentID( c ) ) ];
      const double patternStddev = AR_PATTERN_STDDEV[ c == COMP_Y ? 0 : 1 ] * std::sqrt( arEnergy( coef, log2ScaleFactor ) );
      bool anyGrain = false;
      cm.numModelValues = 6;
      for( int i = 0; i < cm.numIntensityIntervals; i++ )
      {
        std::vector<int>& v = cm.intensityValues[ i ].compModelValue;
        const double stddev = v[ 0 ] / ( 2.0 * m_strengthScale ) / std::sqrt( f.nonDcShare );
        const double scale  = std::ldexp( stddev * strength, log2ScaleFactor + 10 - bitDepth ) / patternStddev;
        fits     = fits && scale <= MAX_SCALING_FACTOR + 0.5;
        v.assign( 6, 0 );
        v[ 0 ]   = std::min( MAX_SCALING_FACTOR, (int) ( scale + 0.5 ) );
        std::copy( coef + 1, coef + 6, v.begin() + 1 );
        anyGrain = anyGrain || v[ 0 ] > 0;
      }
      cm.presentFlag = anyGrain;
    }
    if( fits || log2ScaleFactor == 2 )
    {
      return;
    }
  }
}

// The scaling factors are calibrated, because the standard deviation of the synthesized grain depends on the cut-off
// frequencies by a factor of up to 5 (PATTERN_STDDEV), and the block measurement misses up to 9% of the grain
// variance (BLOCK_VARIANCE_SHARE). VTM signals twice the standard deviation, which fits only cut-off frequencies near 8.
// The log2 scale factor is the largest (2 to 7, the range of the synthesis) at which all scaling factors stay at
// MAX_SCALING_FACTOR or below, for the finest steps.
void FGAnalyzer::getSignalledParameters( double strength, int& log2ScaleFactor, SeiFgc::CompModel compModel[ MAX_NUM_COMP ] ) const
{
  if( !m_improved )
  {
    getVtmParameters( strength, log2ScaleFactor, compModel );
    return;
  }
  if( m_arModel )
  {
    getArParameters( strength, log2ScaleFactor, compModel );
    return;
  }
  std::vector<double> scales[ MAX_NUM_COMP ];   // per interval, at log2 scale factor 0
  double maxScale = 0.0;
  for( int c = 0; c < MAX_NUM_COMP; c++ )
  {
    SeiFgc::CompModel& cm = compModel[ c ];
    cm = m_compModel[ c ];
    if( !cm.presentFlag || strength <= 0.0 )
    {
      cm.presentFlag = false;
      continue;
    }
    // The analysis measures the chroma grain on the subsampled chroma planes, but for 4:2:0 the synthesis (VTM, vvdec)
    // treats the chroma cut-off frequencies as luma resolution values and doubles them: they are halved here. Its
    // halving of the chroma scaling factors is part of the chroma calibration tables.
    const bool chroma420 = c != COMP_Y && m_chromaFormat == CHROMA_420;
    const int  table     = chroma420 ? 1 : 0;
    const int  bitDepth  = m_bitDepths[ toChannelType( ComponentID( c ) ) ];
    for( int i = 0; i < cm.numIntensityIntervals; i++ )
    {
      std::vector<int>& v = cm.intensityValues[ i ].compModelValue;
      for( int k = 1; chroma420 && k < cm.numModelValues; k++ )
      {
        v[ k ] = Clip3( 2, 14, ( v[ k ] + 1 ) >> 1 );
      }
      const int h = cm.numModelValues > 1 ? Clip3( 2, 14, v[ 1 ] ) : 8;
      const int w = cm.numModelValues > 2 ? Clip3( 2, 14, v[ 2 ] ) : h;
      const double stddev = v[ 0 ] / ( 2.0 * m_strengthScale ) / std::sqrt( BLOCK_VARIANCE_SHARE[ table ][ h - 2 ][ w - 2 ] );
      scales[ c ].push_back( stddev * strength * ( 1 << ( 14 - bitDepth ) ) / PATTERN_STDDEV[ table ][ h - 2 ][ w - 2 ] );
      maxScale = std::max( maxScale, scales[ c ].back() );
    }
  }

  log2ScaleFactor = 7;
  while( log2ScaleFactor > 2 && maxScale * ( 1 << log2ScaleFactor ) > MAX_SCALING_FACTOR + 0.5 )
  {
    log2ScaleFactor--;
  }
  for( int c = 0; c < MAX_NUM_COMP; c++ )
  {
    SeiFgc::CompModel& cm = compModel[ c ];
    if( !cm.presentFlag )
    {
      continue;
    }
    bool anyGrain = false;
    for( int i = 0; i < cm.numIntensityIntervals; i++ )
    {
      int& scale = cm.intensityValues[ i ].compModelValue[ 0 ];
      scale    = std::min( MAX_SCALING_FACTOR, (int) ( scales[ c ][ i ] * ( 1 << log2ScaleFactor ) + 0.5 ) );
      anyGrain = anyGrain || scale > 0;
    }
    cm.presentFlag = anyGrain;
  }
}

// ====================================================================================================================
// Analysis of one picture
// ====================================================================================================================

void FGAnalyzer::estimateGrainParameters( const CPelUnitBuf& orig, const CPelUnitBuf& denoised, const CPelUnitBuf* residual, int residualWeight,
                                          bool sceneCut )
{
  const int numComp = getNumberValidComponents( m_chromaFormat );
  m_grainWeight = residual ? residualWeight : 1;
  for( int c = 0; sceneCut && c < numComp; c++ )
  {
    resetStatistics( c );
  }
  for( int c = 0; c < numComp; c++ )
  {
    const ComponentID compId = ComponentID( c );
    const CPelBuf o = orig.get( compId );
    const CPelBuf d = denoised.get( compId );
    m_denoised[ c ].resize( d.width, d.height );
    m_grain   [ c ].resize( d.width, d.height );
    for( int y = 0; y < (int) o.height; y++ )
    {
      const Pel* ro = o.bufAt( 0, y );
      const Pel* rd = d.bufAt( 0, y );
      Pel* td = m_denoised[ c ].row( y );
      Pel* tg = m_grain   [ c ].row( y );
      std::copy( rd, rd + o.width, td );
      if( residual )
      {
        const Pel* rr = residual->get( compId ).bufAt( 0, y );
        std::copy( rr, rr + o.width, tg );
      }
      else
      {
        for( int x = 0; x < (int) o.width; x++ )
        {
          tg[ x ] = (Pel) ( rd[ x ] - ro[ x ] );
        }
      }
    }
    if( m_doAnalysis[ c ] )
    {
      findMask( compId );
    }
  }

  m_cutoffPairs[ COMP_Y ] = m_cutoffPairs[ COMP_Cb ] = m_cutoffPairs[ COMP_Cr ] = 0;
  for( int c = 0; c < numComp; c++ )
  {
    estimateComponent( ComponentID( c ) );
  }
}

// Mask of the samples that are not part of a flat area: edges found at full, half and quarter resolution, dilated.
void FGAnalyzer::findMask( ComponentID compId )
{
  const int bitDepth  = m_bitDepths[ toChannelType( compId ) ];
  const Plane& picture = m_denoised[ compId ];
  Plane& mask = m_mask[ compId ];

  Scratch& s = m_scratch;
  subsample( picture, s.half, 2 );
  subsample( picture, s.quarter, 4 );

  maskForScale( picture, mask, bitDepth, 4 );
  maskForScale( s.half, s.lowMask, bitDepth, 3 );
  combineUpsampled( mask, s.lowMask, 2, s.wide );
  maskForScale( s.quarter, s.lowMask, bitDepth, 2 );
  combineUpsampled( mask, s.lowMask, 4, s.wide );

  // final dilation to fill the holes, then erosion
  morphStep( m_kernels, mask, s.morph, true );
  morphStep( m_kernels, mask, s.morph, true );
  morphStep( m_kernels, mask, s.morph, false );
}

void FGAnalyzer::maskForScale( const Plane& picture, Plane& mask, int bitDepth, int numDilations )
{
  Scratch& s = m_scratch;
  gradientWithNonMaxSuppression( m_kernels, picture, mask, bitDepth, s.magnitude, s.direction );
  doubleThreshold( m_kernels, mask, bitDepth );
  edgeTracking( m_kernels, mask, bitDepth, s );
  suppressLowIntensity( picture, mask, bitDepth );
  for( int i = 0; i < numDilations; i++ )
  {
    morphStep( m_kernels, mask, s.morph, true );
  }
}

// Collects the block statistics of one component and estimates its parameters.
void FGAnalyzer::estimateComponent( ComponentID compId )
{
  if( !m_doAnalysis[ compId ] || ( compId > COMP_Y && !m_compModel[ COMP_Y ].presentFlag ) )
  {
    m_compModel[ compId ].presentFlag = false;
    m_cutoffPairs[ compId ] = 0;
    return;
  }

  const int bitDepth = m_bitDepths[ toChannelType( compId ) ];
  const Plane& denoised = m_denoised[ compId ];
  const Plane& grain    = m_grain[ compId ];
  const Plane& mask     = m_mask[ compId ];
  const int maxStrength = MAX_REAL_SCALE << ( bitDepth - BIT_DEPTH_8 );

  static_assert( BLOCK_SIZE == 16, "the block kernels work on 16x16 blocks" );
  std::vector<int> vecMean;
  std::vector<int> vecStrength;

  // older pictures count less
  for( size_t m = 0; m < m_numDctBlocks[ compId ].size(); m++ )
  {
    if( m_numDctBlocks[ compId ][ m ] > 0 )
    {
      m_numDctBlocks[ compId ][ m ] *= ACCUMULATION_DECAY;
      for( double& v : m_squaredDctSums[ compId ][ m ] )
      {
        v *= ACCUMULATION_DECAY;
      }
    }
  }

  // The flat blocks, visited column by column as in VTM; the order matters for the floating point sums of the fit.
  struct FlatBlock
  {
    int    x, y, mean;
    double stddev;     // grain standard deviation
  };
  std::vector<FlatBlock> blocks;
  const double weightStddev = std::sqrt( (double) m_grainWeight );   // standard deviation of m_grain per grain standard deviation
  for( int x = 0; x + BLOCK_SIZE <= denoised.width; x += BLOCK_SIZE )
  {
    for( int y = 0; y + BLOCK_SIZE <= denoised.height; y += BLOCK_SIZE )
    {
      if( !m_kernels.isZeroBlock( mask.row( y ) + x, mask.width ) )
      {
        continue;   // not flat
      }
      const BlockStats picStats = meanStdDev( m_kernels, denoised, x, y );
      if( 2 * picStats.stddev >= ( maxStrength >> 1 ) )
      {
        continue;   // the denoised block is not flat enough: the mask missed some texture
      }
      blocks.push_back( { x, y, picStats.mean, meanStdDev( m_kernels, grain, x, y ).stddev / weightStddev } );
    }
  }

  // Where the motion compensation of the temporal residual failed, the residual of a block is much larger than the
  // grain of the other blocks of its intensity: such blocks are left out.
  std::vector<double> maxVariance( NUM_OUTLIER_BINS, std::numeric_limits<double>::max() );
  if( m_grainWeight > 1 )
  {
    const int binShift = bitDepth - 4;
    std::vector<std::vector<double>> binVariances( NUM_OUTLIER_BINS );
    for( const FlatBlock& b : blocks )
    {
      binVariances[ b.mean >> binShift ].push_back( b.stddev * b.stddev );
    }
    for( int i = 0; i < NUM_OUTLIER_BINS; i++ )
    {
      std::vector<double>& v = binVariances[ i ];
      if( (int) v.size() >= MIN_OUTLIER_BIN_BLOCKS )
      {
        std::nth_element( v.begin(), v.begin() + v.size() / 2, v.end() );
        maxVariance[ i ] = OUTLIER_VARIANCE_RATIO * v[ v.size() / 2 ];
      }
    }
  }

  std::vector<const FlatBlock*> grainBlocks;
  for( const FlatBlock& b : blocks )
  {
    if( b.stddev * b.stddev > maxVariance[ b.mean >> ( bitDepth - 4 ) ] )
    {
      continue;   // motion compensation failure
    }
    const int strength = (int) ( 2 * m_strengthScale * b.stddev + 0.5 );
    if( strength >= maxStrength * m_strengthScale )
    {
      continue;   // too strong to be grain: probably texture or motion left in the grain estimate
    }
    vecMean.push_back( b.mean );
    vecStrength.push_back( strength );
    grainBlocks.push_back( &b );
  }

  // When the grain of this picture is much stronger or weaker than that of the accumulated data, a new scene began:
  // the accumulated data are dropped. Otherwise the second pass of the fit would discard the new scene's points as
  // outliers of the old fit for several analyses, and the old grain would stay on.
  double storedStrength = 0.0, storedElements = 0.0;
  for( size_t i = 0; i < m_storedStrengthIntensity[ compId ].size(); i++ )
  {
    storedStrength += m_storedStrengthIntensity  [ compId ][ i ];
    storedElements += m_storedElementsPerInterval[ compId ][ i ];
  }
  if( m_improved && (int) vecStrength.size() >= GRAIN_CHANGE_MIN_BLOCKS && storedElements > 0.0 )
  {
    const double mean   = std::accumulate( vecStrength.begin(), vecStrength.end(), 0.0 ) / vecStrength.size();
    const double stored = storedStrength / storedElements;
    if( mean > GRAIN_CHANGE_RATIO * stored || stored > GRAIN_CHANGE_RATIO * mean )
    {
      resetStatistics( compId );
    }
  }

  SquaredDctSum blockDct( BLOCK_SIZE * BLOCK_SIZE );
  for( const FlatBlock* bp : grainBlocks )
  {
    const FlatBlock& b = *bp;
    double* dctSum = m_squaredDctSums[ compId ][ b.mean ].data();
    if( m_grainWeight == 1 )
    {
      m_kernels.squaredDct16( grain.row( b.y ) + b.x, grain.width, ( 1 << ( bitDepth + 4 ) ) - 1, dctSum );
    }
    else
    {
      // the spectrum in grain units, so that pictures with one or two neighbours add up correctly
      std::fill( blockDct.begin(), blockDct.end(), 0.0 );
      m_kernels.squaredDct16( grain.row( b.y ) + b.x, grain.width, ( 1 << ( bitDepth + 4 ) ) - 1, blockDct.data() );
      for( int i = 0; i < BLOCK_SIZE * BLOCK_SIZE; i++ )
      {
        dctSum[ i ] += blockDct[ i ] / m_grainWeight;
      }
    }
    m_numDctBlocks[ compId ][ b.mean ]++;
  }

  estimateScalingFactors( vecMean, vecStrength, bitDepth, compId );
  if( m_arModel )
  {
    fitArFilter( compId );
  }
  else
  {
    estimateCutoffFreq( compId, bitDepth );
  }
}

// ====================================================================================================================
// Scaling function
// ====================================================================================================================

void FGAnalyzer::estimateScalingFactors( std::vector<int>& dataX, std::vector<int>& dataY, int bitDepth, ComponentID compId )
{
  std::vector<double> coeffs;
  std::vector<double> scalingVec;
  std::vector<int>    quantVec;

  // fit a polynomial to the points; the second pass removes points far from the first fit and stores the data
  bool valid  = false;
  bool stored = false;
  for( int i = 0; i < NUM_PASSES; i++ )
  {
    valid  = fitFunction( dataX, dataY, coeffs, scalingVec, ORDER, bitDepth, i == 1, compId );
    stored = i == 1;
    if( !valid )
    {
      break;
    }
  }
  if( valid )
  {
    valid = lloydMax( scalingVec, quantVec, QUANT_LEVELS, bitDepth );
  }
  if( valid )
  {
    setEstimatedParameters( quantVec, bitDepth, compId );
    m_failedFits[ compId ] = 0;
    return;
  }
  if( !m_improved )
  {
    return;   // VTM keeps the previous parameters
  }

  // Without a fit, for example because there were too few points, the previous parameters are kept for a while (VTM
  // keeps them for good). The stored data age as if a picture without data had been added, and after MAX_FAILED_FITS
  // failures in a row the grain is no longer signalled, as it was measured on pictures too long ago: otherwise the
  // grain of a grainy scene stays on in the following clean scene when the fit fails there.
  if( !stored )
  {
    for( auto* data : { &m_storedMeanIntensity[ compId ], &m_storedStrengthIntensity[ compId ], &m_storedElementsPerInterval[ compId ] } )
    {
      for( double& v : *data )
      {
        v *= ACCUMULATION_DECAY;
      }
    }
  }
  if( ++m_failedFits[ compId ] >= MAX_FAILED_FITS )
  {
    m_compModel  [ compId ].presentFlag = false;
    m_cutoffPairs[ compId ] = 0;
  }
}

// Fits a polynomial of the given order to the strength over intensity points, averaged per intensity interval and
// accumulated with the points of the previous pictures. scalingVec receives the function sampled from xmin to xmax,
// followed by xmax and xmin.
bool FGAnalyzer::fitFunction( std::vector<int>& dataX, std::vector<int>& dataY, std::vector<double>& coeffs, std::vector<double>& scalingVec,
                                int order, int bitDepth, bool secondPass, ComponentID compId )
{
  const int intervalSize = INTERVAL_SIZE << ( ( bitDepth - BIT_DEPTH_8 ) >> 1 );
  const int numIntervals = ( 1 << bitDepth ) / intervalSize;
  std::vector<double> meanIntensity( numIntervals, 0.0 );
  std::vector<double> strengthIntensity( numIntervals, 0.0 );
  std::vector<double> elementsPerInterval( numIntervals, 0.0 );
  std::vector<int> tmpDataX;
  std::vector<int> tmpDataY;
  double xmin = 0.0, xmax = 0.0, ymin = 0.0, ymax = 0.0;
  double mn = 0.0, sd = 0.0;

  if( secondPass )
  {
    xmin = scalingVec.back();
    scalingVec.pop_back();
    xmax = scalingVec.back();
    scalingVec.pop_back();
    const int n = (int) dataY.size();
    if( n != 0 )
    {
      mn = std::accumulate( dataY.begin(), dataY.end(), 0.0 ) / n;
      for( int i = 0; i < n; i++ )
      {
        sd += ( dataY[ i ] - mn ) * ( dataY[ i ] - mn );
      }
      sd /= n;
      sd = std::sqrt( sd );
    }
  }

  for( size_t i = 0; i < dataX.size(); i++ )
  {
    if( secondPass )
    {
      if( dataX[ i ] >= xmin && dataX[ i ] <= xmax )
      {
        const double fitted = scalingVec[ dataX[ i ] - (int) xmin ];
        if( dataY[ i ] < fitted + sd * VAR_SCALE_UP && dataY[ i ] > fitted - sd * VAR_SCALE_DOWN )
        {
          const int idx = dataX[ i ] / intervalSize;
          meanIntensity[ idx ]     += dataX[ i ];
          strengthIntensity[ idx ] += dataY[ i ];
          elementsPerInterval[ idx ]++;
        }
      }
    }
    else
    {
      const int idx = dataX[ i ] / intervalSize;
      meanIntensity[ idx ]     += dataX[ i ];
      strengthIntensity[ idx ] += dataY[ i ];
      elementsPerInterval[ idx ]++;
    }
  }

  // the data of the previous pictures, with decreasing weight
  if( !m_storedMeanIntensity[ compId ].empty() && !m_storedStrengthIntensity[ compId ].empty() )
  {
    for( int i = 0; i < numIntervals; i++ )
    {
      elementsPerInterval[ i ] += ACCUMULATION_DECAY * m_storedElementsPerInterval[ compId ][ i ];
      meanIntensity[ i ]       += ACCUMULATION_DECAY * m_storedMeanIntensity[ compId ][ i ];
      strengthIntensity[ i ]   += ACCUMULATION_DECAY * m_storedStrengthIntensity[ compId ][ i ];
    }
  }

  for( int i = 0; i < numIntervals; i++ )
  {
    if( elementsPerInterval[ i ] >= MIN_ELEMENTS_PER_INTERVAL )
    {
      tmpDataX.push_back( (int) ( meanIntensity[ i ] / elementsPerInterval[ i ] ) );
      tmpDataY.push_back( (int) ( strengthIntensity[ i ] / elementsPerInterval[ i ] + 0.5 ) );   // VTM truncates
    }
  }

  if( secondPass )
  {
    // keep the data to improve the estimate of the next pictures
    m_storedMeanIntensity[ compId ]       = meanIntensity;
    m_storedStrengthIntensity[ compId ]   = strengthIntensity;
    m_storedElementsPerInterval[ compId ] = elementsPerInterval;
  }

  // remove single points
  for( int i = 0; i < (int) tmpDataX.size(); i++ )
  {
    int check = 0;
    for( int j = -WINDOW; j <= WINDOW; j++ )
    {
      const int idx = i + j;
      if( idx >= 0 && idx < (int) tmpDataX.size() && j != 0 )
      {
        check += abs( tmpDataX[ i ] / intervalSize - tmpDataX[ idx ] / intervalSize ) <= WINDOW ? 1 : 0;
      }
    }
    if( check < NBRS )
    {
      tmpDataX.erase( tmpDataX.begin() + i );
      tmpDataY.erase( tmpDataY.begin() + i );
      i--;
    }
  }

  // VTM needs order + 1 measured points here. Chroma block means usually fall into only a few intervals around the
  // middle of the range, so the chroma fit failed for most content and VTM signalled its default chroma parameters.
  // The points added by extendPoints() model the fall-off on both sides of the measured range, which is enough for the
  // improved fit when at least two measured points are left.
  if( (int) tmpDataX.size() < ( m_improved ? MIN_MEASURED_POINTS : order + 1 ) )
  {
    return false;   // too few points to fit the function
  }
  extendPoints( tmpDataX, tmpDataY, bitDepth );
  CHECK( tmpDataX.size() > MAXPAIRS, "Maximum dataset size exceeded." );
  if( (int) tmpDataX.size() < order + 1 )
  {
    return false;
  }

  xmin = xmax = tmpDataX[ 0 ];
  ymin = ymax = tmpDataY[ 0 ];
  for( size_t i = 0; i < tmpDataX.size(); i++ )
  {
    xmin = std::min<double>( xmin, tmpDataX[ i ] );
    xmax = std::max<double>( xmax, tmpDataX[ i ] );
    ymin = std::min<double>( ymin, tmpDataY[ i ] );
    ymax = std::max<double>( ymax, tmpDataY[ i ] );
  }
  double xlow = xmax;
  double ylow = ymax;

  const int dataPairs = (int) tmpDataX.size();
  std::vector<double> dataXd( dataPairs + 1 ), dataYd( dataPairs + 1 );
  for( int i = 0; i < dataPairs; i++ )
  {
    dataXd[ i + 1 ] = (double) tmpDataX[ i ];
    dataYd[ i + 1 ] = (double) tmpDataY[ i ];
  }
  if( secondPass )
  {
    std::vector<int>().swap( dataX );
    std::vector<int>().swap( dataY );
    std::vector<double>().swap( coeffs );
    std::vector<double>().swap( scalingVec );
  }

  for( int i = 1; i <= dataPairs; i++ )
  {
    if( dataXd[ i ] < xlow && dataXd[ i ] != 0 )
    {
      xlow = dataXd[ i ];
    }
    if( dataYd[ i ] < ylow && dataYd[ i ] != 0 )
    {
      ylow = dataYd[ i ];
    }
  }
  const double xscale = ( xlow < .001 && xmax < 1000 ) ? 1 / xlow : ( xmax > 1000 && xlow > .001 ) ? 1 / xmax : 1;
  const double yscale = ( ylow < .001 && ymax < 1000 ) ? 1 / ylow : ( ymax > 1000 && ylow > .001 ) ? 1 / ymax : 1;

  // least squares normal equations, solved by Gaussian elimination with partial pivoting (indices from 1)
  double a[ MAXORDER + 1 ][ MAXORDER + 1 ] = { { 0 } };
  double B[ MAXORDER + 1 ] = { 0 }, S[ MAXORDER + 1 ] = { 0 };
  double polycoefs[ MAXORDER + 1 ] = { 0 };
  double Y1 = 0;
  const double maxVal = std::numeric_limits<double>::max();
  for( int j = 1; j <= dataPairs; j++ )
  {
    for( int i = 1; i <= order; i++ )
    {
      B[ i ] = B[ i ] + dataYd[ j ] * yscale * ldpow( dataXd[ j ] * xscale, i );
      if( B[ i ] == maxVal )
      {
        return false;
      }
      for( int k = 1; k <= order; k++ )
      {
        a[ i ][ k ] = a[ i ][ k ] + ldpow( dataXd[ j ] * xscale, i + k );
        if( a[ i ][ k ] == maxVal )
        {
          return false;
        }
      }
      S[ i ] = S[ i ] + ldpow( dataXd[ j ] * xscale, i );
      if( S[ i ] == maxVal )
      {
        return false;
      }
    }
    Y1 = Y1 + dataYd[ j ] * yscale;
    if( Y1 == maxVal )
    {
      return false;
    }
  }

  for( int i = 1; i <= order; i++ )
  {
    for( int j = 1; j <= order; j++ )
    {
      a[ i ][ j ] = a[ i ][ j ] - S[ i ] * S[ j ] / (double) dataPairs;
      if( a[ i ][ j ] == maxVal )
      {
        return false;
      }
    }
    B[ i ] = B[ i ] - Y1 * S[ i ] / (double) dataPairs;
    if( B[ i ] == maxVal )
    {
      return false;
    }
  }

  for( int k = 1; k <= order; k++ )
  {
    int    R  = k;
    double A1 = 0;
    for( int L = k; L <= order; L++ )
    {
      const double A2 = std::fabs( a[ L ][ k ] );
      if( A2 > A1 )
      {
        A1 = A2;
        R  = L;
      }
    }
    if( A1 == 0 )
    {
      return false;
    }
    if( R != k )
    {
      for( int j = k; j <= order; j++ )
      {
        std::swap( a[ R ][ j ], a[ k ][ j ] );
      }
      std::swap( B[ R ], B[ k ] );
    }
    for( int i = k; i <= order; i++ )
    {
      const double m = a[ i ][ k ];
      for( int j = k; j <= order; j++ )
      {
        a[ i ][ j ] = i == k ? a[ i ][ j ] / m : a[ i ][ j ] - m * a[ k ][ j ];
      }
      B[ i ] = i == k ? B[ i ] / m : B[ i ] - m * B[ k ];
    }
  }

  polycoefs[ order ] = B[ order ];
  for( int k = 1; k <= order - 1; k++ )
  {
    const int i = order - k;
    double S1 = 0;
    for( int j = 1; j <= order; j++ )
    {
      S1 = S1 + a[ i ][ j ] * polycoefs[ j ];
      if( S1 == maxVal )
      {
        return false;
      }
    }
    polycoefs[ i ] = B[ i ] - S1;
  }

  double S1 = 0;
  for( int i = 1; i <= order; i++ )
  {
    S1 = S1 + polycoefs[ i ] * S[ i ] / (double) dataPairs;
    if( S1 == maxVal )
    {
      return false;
    }
  }
  polycoefs[ 0 ] = Y1 / (double) dataPairs - S1;

  // zero all coefficients smaller than +-1e-11 (avoids -0)
  for( int i = 0; i <= order; i++ )
  {
    if( std::fabs( polycoefs[ i ] * 100000000000 ) < 1 )
    {
      polycoefs[ i ] = 0;
    }
  }

  for( int i = 0; i <= order; i++ )
  {
    polycoefs[ i ] = ( 1 / yscale ) * polycoefs[ i ] * ldpow( xscale, i );
    coeffs.push_back( polycoefs[ i ] );
  }

  // sample the scaling function from xmin to xmax
  for( int i = (int) xmin; i <= (int) xmax; i++ )
  {
    double val = coeffs[ 0 ];
    for( size_t j = 1; j < coeffs.size(); j++ )
    {
      val += coeffs[ j ] * ldpow( i, (unsigned) j );
    }
    scalingVec.push_back( Clip3( 0.0, (double) ( 1 << bitDepth ) - 1, val ) );
  }
  scalingVec.push_back( xmax );
  scalingVec.push_back( xmin );
  return true;
}

// Adds up to four points beyond the lowest and the highest intensity, each a step further and weaker, then drops the
// points outside the usable intensity range.
void FGAnalyzer::extendPoints( std::vector<int>& dataX, std::vector<int>& dataY, int bitDepth ) const
{
  const int minInt = bitDepth < 10 ? MIN_INTENSITY >> ( 10 - bitDepth ) : MIN_INTENSITY << ( bitDepth - 10 );
  const int maxInt = bitDepth < 10 ? MAX_INTENSITY >> ( 10 - bitDepth ) : MAX_INTENSITY << ( bitDepth - 10 );

  int xmin = dataX[ 0 ], xmax = dataX[ 0 ];
  int ymin = dataY[ 0 ], ymax = dataY[ 0 ];   // strengths at xmin and xmax
  for( size_t i = 0; i < dataX.size(); i++ )
  {
    if( dataX[ i ] < xmin )
    {
      xmin = dataX[ i ];
      ymin = dataY[ i ];
    }
    if( dataX[ i ] > xmax )
    {
      xmax = dataX[ i ];
      ymax = dataY[ i ];
    }
  }

  int numLeft = MAX_POINTS_TO_EXTEND, numRight = MAX_POINTS_TO_EXTEND;
  while( xmin >= POINT_STEP && ymin > m_strengthScale && numLeft > 0 )
  {
    xmin -= POINT_STEP;
    ymin = m_improved ? ymin : (int) ( ymin / POINT_SCALE );
    dataX.push_back( xmin );
    dataY.push_back( ymin );
    numLeft--;
  }
  while( xmax + POINT_STEP <= ( 1 << bitDepth ) - 1 && ymax > m_strengthScale && numRight > 0 )
  {
    xmax += POINT_STEP;
    ymax = m_improved ? ymax : (int) ( ymax / POINT_SCALE );
    dataX.push_back( xmax );
    dataY.push_back( ymax );
    numRight--;
  }
  for( int i = 0; i < (int) dataX.size(); i++ )
  {
    if( dataX[ i ] < minInt || dataX[ i ] > maxInt )
    {
      dataX.erase( dataX.begin() + i );
      dataY.erase( dataY.begin() + i );
      i--;
    }
  }
}

// Lloyd-Max quantization of the sampled scaling function into numQuantizedLevels levels; quantizedVec receives the
// quantized function for every intensity.
bool FGAnalyzer::lloydMax( std::vector<double>& scalingVec, std::vector<int>& quantizedVec, int numQuantizedLevels, int bitDepth ) const
{
  if( scalingVec.size() <= 2 )
  {
    return false;   // no sampled function (only xmax and xmin)
  }

  const int xmin = (int) scalingVec.back();
  scalingVec.pop_back();
  scalingVec.pop_back();   // xmax

  const double tolerance = 0.0000001;
  double ymin = scalingVec[ 0 ], ymax = scalingVec[ 0 ];
  for( double v : scalingVec )
  {
    ymin = std::min( ymin, v );
    ymax = std::max( ymax, v );
  }
  const double initTraining = ( ymax - ymin ) / numQuantizedLevels;
  if( initTraining <= 0 )
  {
    // A constant scaling function needs no quantization. VTM returns failure here, so grain of the same strength at
    // all measured intensities was never signalled.
    quantizedVec.assign( (size_t) 1 << bitDepth, 0 );
    for( size_t i = 0; i < scalingVec.size(); i++ )
    {
      quantizedVec[ i + xmin ] = Clip3( 0, ( MAX_STANDARD_DEVIATION * m_strengthScale ) << ( bitDepth - BIT_DEPTH_8 ), (int) ( ymin + .5 ) );
    }
    return true;
  }

  std::vector<double> codebook( numQuantizedLevels );
  std::vector<double> partition( numQuantizedLevels - 1 );
  const double step = initTraining / 2;
  for( int i = 0; i < numQuantizedLevels; i++ )
  {
    codebook[ i ] = ymin + i * initTraining + step;
  }
  for( int i = 0; i < numQuantizedLevels - 1; i++ )
  {
    partition[ i ] = ( codebook[ i ] + codebook[ i + 1 ] ) / 2;
  }

  // Quantizes the scaling function with the current partition and codebook and returns the mean squared error.
  std::vector<double> quantized( scalingVec.size(), 0.0 );
  auto quantize = [ & ]()
  {
    for( size_t i = 0; i < scalingVec.size(); i++ )
    {
      double level = 0.0;
      for( size_t j = 0; j < partition.size(); j++ )
      {
        level = level + ( scalingVec[ i ] > partition[ j ] );
      }
      quantized[ i ] = codebook[ (int) level ];
    }
    double distortion = 0.0;
    for( size_t i = 0; i < scalingVec.size(); i++ )
    {
      distortion += ( scalingVec[ i ] - quantized[ i ] ) * ( scalingVec[ i ] - quantized[ i ] );
    }
    return distortion / scalingVec.size();
  };

  double distortion = quantize();
  double lastDistortion = 0.0;
  const double tolerance2 = std::numeric_limits<double>::epsilon() * ymax;
  double relDistortion = distortion > tolerance2 ? std::fabs( distortion - lastDistortion ) / distortion : distortion;

  while( relDistortion > tolerance && relDistortion > tolerance2 )
  {
    for( int i = 0; i < numQuantizedLevels; i++ )
    {
      int    count = 0;
      double sum   = 0.0;
      for( size_t j = 0; j < quantized.size(); j++ )
      {
        if( codebook[ i ] == quantized[ j ] )
        {
          count++;
          sum += scalingVec[ j ];
        }
      }
      if( count )
      {
        codebook[ i ] = sum / (double) count;
        continue;
      }
      // empty cell: use the mean of the samples in its partition range, or the middle of the range
      sum = 0.0;
      count = 0;
      const bool first = i == 0, last = i == numQuantizedLevels - 1;
      for( size_t j = 0; j < quantized.size(); j++ )
      {
        const bool inCell = first ? scalingVec[ j ] <= partition[ i ]
                          : last  ? scalingVec[ j ] >= partition[ i - 1 ]
                                  : scalingVec[ j ] >= partition[ i - 1 ] && scalingVec[ j ] <= partition[ i ];
        if( inCell )
        {
          count++;
          sum += scalingVec[ j ];
        }
      }
      if( count )
      {
        codebook[ i ] = sum / (double) count;
      }
      else
      {
        codebook[ i ] = first ? ( partition[ i ] + ymin ) / 2 : last ? ( partition[ i - 1 ] + ymax ) / 2 : ( partition[ i - 1 ] + partition[ i ] ) / 2;
      }
    }

    for( int i = 0; i < numQuantizedLevels - 1; i++ )
    {
      partition[ i ] = ( codebook[ i ] + codebook[ i + 1 ] ) / 2;
    }
    std::sort( partition.begin(), partition.end() );

    lastDistortion = distortion;
    distortion = quantize();
    relDistortion = distortion > tolerance2 ? std::fabs( distortion - lastDistortion ) / distortion : distortion;
  }

  quantizedVec.assign( (size_t) 1 << bitDepth, 0 );
  for( size_t i = 0; i < quantized.size(); i++ )
  {
    quantizedVec[ i + xmin ] = Clip3( 0, ( MAX_STANDARD_DEVIATION * m_strengthScale ) << ( bitDepth - BIT_DEPTH_8 ), (int) ( quantized[ i ] + .5 ) );
  }
  return true;
}

// Turns the quantized scaling function into intensity intervals and scaling factors of the SEI message (8-bit
// intensities). The cut-off frequencies are set afterwards.
void FGAnalyzer::setEstimatedParameters( std::vector<int>& quantizedVec, int bitDepth, ComponentID compId )
{
  // strengths below half of VTM's unit count as no grain, as they did in VTM
  for( int& q : quantizedVec )
  {
    q = q < m_strengthScale / 2 ? 0 : q;
  }
  if( m_improved )
  {
    holdEdgeStrengths( quantizedVec, bitDepth );
  }

  std::vector<std::vector<int>> parameters( 3 );   // lower bounds, upper bounds, scaling factors

  // intervals of constant value
  parameters[ 0 ].push_back( 0 );
  parameters[ 2 ].push_back( quantizedVec[ 0 ] );
  for( size_t i = 0; i + 1 < quantizedVec.size(); i++ )
  {
    if( quantizedVec[ i ] != quantizedVec[ i + 1 ] )
    {
      parameters[ 0 ].push_back( (int) i + 1 );
      parameters[ 1 ].push_back( (int) i );
      parameters[ 2 ].push_back( quantizedVec[ i + 1 ] );
    }
  }
  parameters[ 1 ].push_back( ( 1 << bitDepth ) - 1 );

  mergeIntervals( parameters, bitDepth );
  scaleDown( parameters, bitDepth );

  // the scaling to 8 bits can make intervals overlap: make the bounds strictly increasing
  std::vector<int> bounds;
  for( size_t i = 0; i < parameters[ 2 ].size(); i++ )
  {
    bounds.push_back( parameters[ 0 ][ i ] );
    bounds.push_back( parameters[ 1 ][ i ] );
  }
  for( size_t i = 0; i + 1 < bounds.size(); i++ )
  {
    if( bounds[ i ] >= bounds[ i + 1 ] )
    {
      bounds[ i + 1 ] = bounds[ i ] + 1;
    }
  }
  for( size_t i = 0; i < parameters[ 2 ].size(); i++ )
  {
    parameters[ 0 ][ i ] = bounds[ 2 * i ];
    parameters[ 1 ][ i ] = bounds[ 2 * i + 1 ];
  }

  // Intervals with scaling factor 0 are not signalled. The component is signalled again as soon as a later picture
  // gives a scaling factor above 0 (in VTM a component stays off for the rest of the sequence once all its scaling
  // factors were 0, and then the chroma components are no longer analysed either).
  SeiFgc::CompModel& cm = m_compModel[ compId ];
  cm.numIntensityIntervals = (uint8_t) ( parameters[ 2 ].size() - std::count( parameters[ 2 ].begin(), parameters[ 2 ].end(), 0 ) );
  cm.presentFlag = cm.numIntensityIntervals > 0;
  if( !cm.presentFlag )
  {
    m_cutoffPairs[ compId ] = 0;
    return;
  }
  int j = 0;
  for( size_t i = 0; i < parameters[ 2 ].size(); i++ )
  {
    if( parameters[ 2 ][ i ] != 0 )
    {
      auto& iv = cm.intensityValues[ j++ ];
      iv.intensityIntervalLowerBound = (uint8_t) parameters[ 0 ][ i ];
      iv.intensityIntervalUpperBound = (uint8_t) parameters[ 1 ][ i ];
      iv.compModelValue[ 0 ] = parameters[ 2 ][ i ];
      iv.compModelValue[ 1 ] = -1;   // cut-off frequencies are estimated later
      iv.compModelValue[ 2 ] = -1;
    }
  }
}

// The scaling function only covers the intensities with flat blocks, plus a few extension points. VTM lets it fall off
// by a factor of 1.25 per extension point and signals no grain beyond, but film grain goes on there: on Beekeeper the
// darkest intensities have the strongest grain of the scene, and they got half of it. The improved analysis keeps the
// level of the extension points (extendPoints()) and here holds the first and last strength out to the intensity
// limits of the analysis (MIN_INTENSITY and MAX_INTENSITY).
void FGAnalyzer::holdEdgeStrengths( std::vector<int>& quantizedVec, int bitDepth ) const
{
  const int minInt = bitDepth < 10 ? MIN_INTENSITY >> ( 10 - bitDepth ) : MIN_INTENSITY << ( bitDepth - 10 );
  const int maxInt = bitDepth < 10 ? MAX_INTENSITY >> ( 10 - bitDepth ) : MAX_INTENSITY << ( bitDepth - 10 );
  int first = -1, last = -1;
  for( int i = 0; i < (int) quantizedVec.size(); i++ )
  {
    if( quantizedVec[ i ] > 0 )
    {
      first = first < 0 ? i : first;
      last  = i;
    }
  }
  if( first < 0 )
  {
    return;   // no grain
  }
  for( int i = minInt; i < first; i++ )
  {
    quantizedVec[ i ] = quantizedVec[ first ];
  }
  for( int i = last + 1; i <= maxInt && i < (int) quantizedVec.size(); i++ )
  {
    quantizedVec[ i ] = quantizedVec[ last ];
  }
}

// Merges intervals narrower than 2 (at 8 bits) into the neighbour with the closer scaling factor.
void FGAnalyzer::mergeIntervals( std::vector<std::vector<int>>& parameters, int bitDepth ) const
{
  for( int i = 0; i < (int) parameters[ 2 ].size(); i++ )
  {
    const int width = parameters[ 1 ][ i ] - parameters[ 0 ][ i ];
    if( width >= ( 2 << ( bitDepth - BIT_DEPTH_8 ) ) )
    {
      continue;
    }
    const int last = (int) parameters[ 2 ].size() - 1;
    const int diffRight = ( i == last || parameters[ 2 ][ i + 1 ] == 0 ) ? std::numeric_limits<int>::max() : abs( parameters[ 2 ][ i ] - parameters[ 2 ][ i + 1 ] );
    const int diffLeft  = ( i == 0    || parameters[ 2 ][ i - 1 ] == 0 ) ? std::numeric_limits<int>::max() : abs( parameters[ 2 ][ i ] - parameters[ 2 ][ i - 1 ] );
    // VTM merges to the right when both neighbours are unusable; at the last interval there is no right neighbour.
    const bool mergeLeft = diffLeft < diffRight || ( i == last && i > 0 );
    if( !mergeLeft && i == last )
    {
      continue;   // a single interval cannot be merged
    }
    const int n = mergeLeft ? i - 1 : i + 1;
    const int otherWidth = parameters[ 1 ][ n ] - parameters[ 0 ][ n ];
    // the scales weighted by the interval widths; two intervals of one intensity each (width 0) weigh the same
    const int newScale = otherWidth + width > 0
                       ? ( otherWidth * parameters[ 2 ][ n ] + width * parameters[ 2 ][ i ] ) / ( otherWidth + width )
                       : ( parameters[ 2 ][ n ] + parameters[ 2 ][ i ] ) / 2;
    if( mergeLeft )
    {
      parameters[ 1 ][ i - 1 ] = parameters[ 1 ][ i ];
      parameters[ 2 ][ i - 1 ] = newScale;
      for( auto& p : parameters )
      {
        p.erase( p.begin() + i );
      }
    }
    else
    {
      parameters[ 1 ][ i ] = parameters[ 1 ][ i + 1 ];
      parameters[ 2 ][ i ] = newScale;
      for( auto& p : parameters )
      {
        p.erase( p.begin() + i + 1 );
      }
    }
    i--;
  }
}

// Scales intensities to 8 bits and the scaling factors to the SEI representation.
void FGAnalyzer::scaleDown( std::vector<std::vector<int>>& parameters, int bitDepth ) const
{
  for( size_t i = 0; i < parameters[ 2 ].size(); i++ )
  {
    // the intensities to 8 bits; the strengths stay as they are, getSignalledParameters() converts them
    parameters[ 0 ][ i ] >>= ( bitDepth - BIT_DEPTH_8 );
    parameters[ 1 ][ i ] >>= ( bitDepth - BIT_DEPTH_8 );
  }
}

// ====================================================================================================================
// Cut-off frequencies
// ====================================================================================================================

// The cut-off pair whose synthesized grain has the spectrum closest to the measured one, for the improved analysis:
// the horizontal and vertical marginals of the mean squared DCT (without the DC coefficient), normalized, against those
// of the synthesis (GRAIN_SPECTRUM_MARGINALS), by the sum of squared differences. AN0237's estimator instead takes,
// per direction, the frequency where the marginal falls closest to a fixed fraction of its low-frequency level, which
// reads about one step too high on grain of the model itself (9/9 for grain synthesized with 8/8, 13/6 for 12/5), and
// the horizontal and vertical cut-offs together decide the shape of both marginals, which a per-direction rule cannot
// take into account. For 4:2:0 chroma the measured spectrum is that of the chroma plane, where the
// synthesis doubles the signalled cut-offs; the result is returned at chroma plane resolution like AN0237's, twice the
// signalled pair, which getSignalledParameters() halves.
void FGAnalyzer::fitCutoffPair( const std::vector<double>& meanSq, bool chroma420, int& horizontal, int& vertical ) const
{
  constexpr int N = BLOCK_SIZE;
  double measured[ 2 ][ N ] = { { 0 } };   // horizontal and vertical marginal
  double total = 0.0;
  for( int x = 0; x < N; x++ )
  {
    for( int y = 0; y < N; y++ )
    {
      const double p = x || y ? meanSq[ x * N + y ] : 0.0;
      measured[ 0 ][ x ] += p;
      measured[ 1 ][ y ] += p;
      total              += p;
    }
  }
  horizontal = vertical = -1;
  if( total <= 0.0 )
  {
    return;
  }
  for( int k = 0; k < N; k++ )
  {
    measured[ 0 ][ k ] /= total;
    measured[ 1 ][ k ] /= total;
  }

  const int table = chroma420 ? 1 : 0;
  double bestError = std::numeric_limits<double>::max();
  for( int h = 0; h < NUM_CUTOFFS; h++ )
  {
    for( int v = 0; v < NUM_CUTOFFS; v++ )
    {
      double error = 0.0;
      for( int d = 0; d < 2; d++ )
      {
        for( int k = 0; k < N; k++ )
        {
          const double diff = measured[ d ][ k ] - GRAIN_SPECTRUM_MARGINALS[ table ][ h ][ v ][ d ][ k ];
          error += diff * diff;
        }
      }
      if( error < bestError )
      {
        bestError  = error;
        horizontal = h + 2;
        vertical   = v + 2;
      }
    }
  }
  if( chroma420 )
  {
    horizontal *= 2;
    vertical   *= 2;
  }
}

// Fits the AR filter of a component to the spectrum of its grain, accumulated over all intensities: the Nelder-Mead
// minimum of the L1 distance between the normalized spectra, starting from the previous filter, or from a mild
// low-pass one for the first fit. Filters that are not stable (sum of the absolute tap weights 0.98 or more) are
// excluded. For chroma the model spectrum is the one vvdec synthesizes, the analytic spectrum times
// AR_SYNTHESIS_RATIO_CHROMA, renormalized: the fitted filters then give 0.02-0.06 lower chroma spectrum errors on
// Beekeeper. Luma is fitted to the analytic spectrum: the same correction for luma brought no gain (there the fixed
// pattern of the synthesis is the limit) and made one 4K clip worse.
void FGAnalyzer::fitArFilter( ComponentID compId )
{
  constexpr int N = BLOCK_SIZE;
  const SeiFgc::CompModel& cm = m_compModel[ compId ];
  if( !cm.presentFlag )
  {
    return;
  }
  double measured[ N * N ] = { 0 };   // [ vertical * N + horizontal ]
  double total = 0.0;
  for( const SquaredDctSum& s : m_squaredDctSums[ compId ] )
  {
    for( int x = 0; x < N; x++ )
    {
      for( int y = 0; y < N; y++ )
      {
        measured[ y * N + x ] += x || y ? s[ x * N + y ] : 0.0;
      }
    }
  }
  for( double v : measured )
  {
    total += v;
  }
  if( total <= 0.0 )
  {
    return;
  }
  for( double& v : measured )
  {
    v /= total;
  }

  static const ArSpectrum arSpectrum;
  const float* ratio = compId == COMP_Y ? nullptr : &AR_SYNTHESIS_RATIO_CHROMA[ 0 ][ 0 ];   // [ vertical * N + horizontal ]
  double model[ N * N ];
  std::vector<double> scratch( ArSpectrum::SCRATCH_SIZE );
  auto error = [ & ]( const double p[ 4 ] )
  {
    const double a1 = p[ 0 ], a3 = p[ 1 ], a5 = p[ 2 ], r = p[ 3 ];
    if( r < 0.0 || std::fabs( a1 ) * ( 1 + r ) + 2 * std::fabs( a3 * r ) + std::fabs( a5 ) * ( 1 + r * r ) >= 0.98 )
    {
      return std::numeric_limits<double>::max();
    }
    arSpectrum.compute( a1, a3, a5, r, model, scratch );
    double sum = 1.0;   // the analytic spectrum is normalized; the one weighted by the ratio is renormalized
    if( ratio )
    {
      sum = 0.0;
      for( int i = 0; i < N * N; i++ )
      {
        model[ i ] *= ratio[ i ];
        sum        += model[ i ];
      }
    }
    double e = 0.0;
    for( int i = 0; i < N * N; i++ )
    {
      e += std::fabs( model[ i ] / sum - measured[ i ] );
    }
    return e;
  };

  ArFilter& f = m_arFilter[ compId ];
  const double starts[ 2 ][ 4 ] = { { f.a1, f.a3, f.a5, f.r }, { 0.3, 0.0, 0.0, 1.0 } };
  double best[ 4 ] = { 0 }, bestError = std::numeric_limits<double>::max();
  for( int s = f.valid ? 0 : 1; s < ( f.valid ? 1 : 2 ); s++ )
  {
    // Nelder-Mead on the four parameters
    double pts[ 5 ][ 4 ], vals[ 5 ];
    for( int i = 0; i < 5; i++ )
    {
      for( int j = 0; j < 4; j++ )
      {
        pts[ i ][ j ] = starts[ s ][ j ] + ( i == j + 1 ? 0.15 : 0.0 );
      }
      vals[ i ] = error( pts[ i ] );
    }
    for( int it = 0; it < AR_FIT_ITERATIONS; it++ )
    {
      int order[ 5 ] = { 0, 1, 2, 3, 4 };
      std::sort( order, order + 5, [ & ]( int a, int b ) { return vals[ a ] < vals[ b ]; } );
      const int worst = order[ 4 ], second = order[ 3 ], bestIdx = order[ 0 ];
      if( vals[ worst ] - vals[ bestIdx ] < AR_FIT_TOLERANCE )
      {
        break;   // converged
      }
      double c[ 4 ] = { 0 }, xr[ 4 ], xe[ 4 ], xc[ 4 ];
      for( int i = 0; i < 5; i++ )
      {
        for( int j = 0; j < 4 && i != worst; j++ )
        {
          c[ j ] += pts[ i ][ j ] / 4;
        }
      }
      for( int j = 0; j < 4; j++ )
      {
        xr[ j ] = c[ j ] + ( c[ j ] - pts[ worst ][ j ] );
      }
      const double fr = error( xr );
      if( fr < vals[ bestIdx ] )
      {
        for( int j = 0; j < 4; j++ )
        {
          xe[ j ] = c[ j ] + 2 * ( c[ j ] - pts[ worst ][ j ] );
        }
        const double fe = error( xe );
        std::copy( fe < fr ? xe : xr, ( fe < fr ? xe : xr ) + 4, pts[ worst ] );
        vals[ worst ] = std::min( fe, fr );
      }
      else if( fr < vals[ second ] )
      {
        std::copy( xr, xr + 4, pts[ worst ] );
        vals[ worst ] = fr;
      }
      else
      {
        for( int j = 0; j < 4; j++ )
        {
          xc[ j ] = c[ j ] + 0.5 * ( pts[ worst ][ j ] - c[ j ] );
        }
        const double fc = error( xc );
        if( fc < vals[ worst ] )
        {
          std::copy( xc, xc + 4, pts[ worst ] );
          vals[ worst ] = fc;
        }
        else
        {
          for( int i = 0; i < 5; i++ )
          {
            if( i == bestIdx )
            {
              continue;
            }
            for( int j = 0; j < 4; j++ )
            {
              pts[ i ][ j ] = pts[ bestIdx ][ j ] + 0.5 * ( pts[ i ][ j ] - pts[ bestIdx ][ j ] );
            }
            vals[ i ] = error( pts[ i ] );
          }
        }
      }
    }
    for( int i = 0; i < 5; i++ )
    {
      if( vals[ i ] < bestError )
      {
        bestError = vals[ i ];
        std::copy( pts[ i ], pts[ i ] + 4, best );
      }
    }
  }
  if( bestError == std::numeric_limits<double>::max() )
  {
    return;
  }
  f.a1 = best[ 0 ];
  f.a3 = best[ 1 ];
  f.a5 = best[ 2 ];
  f.r  = best[ 3 ];
  f.nonDcShare = arSpectrum.compute( f.a1, f.a3, f.a5, f.r, model, scratch );
  f.valid = true;
}

// Estimates horizontal and vertical cut-off frequencies per intensity interval from the accumulated squared DCT of the
// grain, then fills missing values, limits the spread and the number of different pairs.
void FGAnalyzer::estimateCutoffFreq( ComponentID compId, int bitDepth )
{
  SeiFgc::CompModel& cm = m_compModel[ compId ];
  if( !cm.presentFlag )
  {
    m_cutoffPairs[ compId ] = 0;
    return;
  }
  constexpr int N = BLOCK_SIZE;
  const int intervals = cm.numIntensityIntervals;

  // intensity interval of every 8-bit intensity
  int intensityInterval[ MAX_NUM_INTENSITIES ];
  std::fill_n( intensityInterval, MAX_NUM_INTENSITIES, -1 );
  for( int i = 0; i < intervals; i++ )
  {
    for( int v = cm.intensityValues[ i ].intensityIntervalLowerBound; v <= cm.intensityValues[ i ].intensityIntervalUpperBound; v++ )
    {
      intensityInterval[ v ] = i;
    }
  }

  const std::vector<SquaredDctSum>& blocks = m_squaredDctSums[ compId ];
  const std::vector<double>&        numEl  = m_numDctBlocks[ compId ];
  std::vector<double> blocksPerInterval( intervals, 0.0 );
  for( size_t i = 0; i < blocks.size(); i++ )
  {
    const int idx = intensityInterval[ i >> ( bitDepth - BIT_DEPTH_8 ) ];
    if( idx != -1 )
    {
      blocksPerInterval[ idx ] += numEl[ i ];
    }
  }

  // mean squared DCT per interval, and its row and column averages without the DC coefficient
  std::vector<std::vector<double>> meanSq( intervals, std::vector<double>( N * N, 0.0 ) );
  std::vector<std::vector<double>> rowMean( intervals, std::vector<double>( N, 0.0 ) );   // per horizontal frequency
  std::vector<std::vector<double>> colMean( intervals, std::vector<double>( N, 0.0 ) );   // per vertical frequency
  for( int x = 0; x < N; x++ )
  {
    for( int y = 0; y < N; y++ )
    {
      for( size_t i = 0; i < blocks.size(); i++ )
      {
        const int idx = intensityInterval[ i >> ( bitDepth - BIT_DEPTH_8 ) ];
        if( idx != -1 )
        {
          meanSq[ idx ][ x * N + y ] += blocks[ i ][ x * N + y ];
        }
      }
      for( int i = 0; i < intervals; i++ )
      {
        if( blocksPerInterval[ i ] >= MIN_BLOCK_WEIGHT )
        {
          meanSq[ i ][ x * N + y ] /= blocksPerInterval[ i ];
          rowMean[ i ][ x ] += ( x != 0 || y != 0 ) ? meanSq[ i ][ x * N + y ] : 0.0;
          colMean[ i ][ y ] += ( x != 0 || y != 0 ) ? meanSq[ i ][ x * N + y ] : 0.0;
        }
      }
    }
  }

  int avgCutoffHorizontal = 0, avgCutoffVertical = 0;
  int numCutoffHorizontal = 0, numCutoffVertical = 0;
  cm.numModelValues = 3;   // always signal all three values
  for( int i = 0; i < intervals; i++ )
  {
    auto& iv = cm.intensityValues[ i ];
    if( blocksPerInterval[ i ] < MIN_BLOCK_WEIGHT )
    {
      iv.compModelValue[ 1 ] = -1;
      iv.compModelValue[ 2 ] = -1;
      continue;
    }
    for( int x = 0; x < N; x++ )
    {
      rowMean[ i ][ x ] /= x == 0 ? N - 1 : N;
      colMean[ i ][ x ] /= x == 0 ? N - 1 : N;
    }
    // rowMean is indexed by the horizontal frequency, colMean by the vertical one. VTM takes the vertical cut-off from
    // rowMean and the horizontal one from colMean, so its grain comes out rotated by 90 degrees (comp_model_value[1]
    // is the horizontal cut-off, for VTM's own synthesis as well).
    int cutoffHorizontal = -1, cutoffVertical = -1;
    if( m_improved )
    {
      fitCutoffPair( meanSq[ i ], compId != COMP_Y && m_chromaFormat == CHROMA_420, cutoffHorizontal, cutoffVertical );
    }
    else
    {
      cutoffHorizontal = cutoffFrequency( rowMean[ i ] );
      cutoffVertical   = cutoffFrequency( colMean[ i ] );
    }
    const int intervalWidth    = iv.intensityIntervalUpperBound - iv.intensityIntervalLowerBound;
    if( cutoffHorizontal != -1 )
    {
      iv.compModelValue[ 1 ] = cutoffHorizontal;
      avgCutoffHorizontal   += intervalWidth * cutoffHorizontal;
      numCutoffHorizontal   += intervalWidth;
    }
    if( cutoffVertical != -1 )
    {
      iv.compModelValue[ 2 ] = cutoffVertical;
      avgCutoffVertical     += intervalWidth * cutoffVertical;
      numCutoffVertical     += intervalWidth;
    }
  }

  avgCutoffHorizontal = numCutoffHorizontal ? avgCutoffHorizontal / numCutoffHorizontal : -1;
  if( numCutoffVertical )
  {
    avgCutoffVertical = avgCutoffVertical / numCutoffVertical;
  }
  else
  {
    avgCutoffVertical = -1;   // VTM master sets avgCutoffHorizontal here, losing the valid horizontal average
  }

  replaceCutoff( compId, avgCutoffHorizontal, avgCutoffVertical );
  limitCutoff( compId, avgCutoffHorizontal, avgCutoffVertical );
  limitCutoffConsecutive( compId );
  if( !cm.presentFlag )
  {
    m_cutoffPairs[ compId ] = 0;
    return;
  }

  // limit the number of different cut-off pairs to what the synthesis supports, shared by the components
  std::vector<CutoffPair> pairs( intervals );
  for( int i = 0; i < intervals; i++ )
  {
    pairs[ i ].p = { cm.intensityValues[ i ].compModelValue[ 1 ], cm.intensityValues[ i ].compModelValue[ 2 ] };
    pairs[ i ].w = cm.intensityValues[ i ].intensityIntervalUpperBound - cm.intensityValues[ i ].intensityIntervalLowerBound;
  }
  const int numLumaPairs = MAX_CUTOFF_PAIRS - 2 + !m_doAnalysis[ COMP_Cb ] + !m_doAnalysis[ COMP_Cr ];
  int maxUnique = numLumaPairs;
  if( compId == COMP_Cb )
  {
    maxUnique = 1 + !m_doAnalysis[ COMP_Cr ] + ( numLumaPairs - m_cutoffPairs[ COMP_Y ] ) / ( m_doAnalysis[ COMP_Cb ] + m_doAnalysis[ COMP_Cr ] );
  }
  else if( compId == COMP_Cr )
  {
    maxUnique = MAX_CUTOFF_PAIRS - m_cutoffPairs[ COMP_Y ] - m_cutoffPairs[ COMP_Cb ];
  }
  m_cutoffPairs[ compId ] = limitCutoffPairs( pairs, maxUnique );
  for( int i = 0; i < intervals; i++ )
  {
    cm.intensityValues[ i ].compModelValue[ 1 ] = pairs[ i ].p.first;
    cm.intensityValues[ i ].compModelValue[ 2 ] = pairs[ i ].p.second;
  }
}

// Cut-off frequency of a 16-sample row or column average: the smoothed average is compared with a threshold derived
// from its low frequencies; the frequency closest to the threshold is the cut-off. Returns -1 when no cut-off is found.
int FGAnalyzer::cutoffFrequency( const std::vector<double>& mean ) const
{
  constexpr int NUM = 13;   // cut-off frequencies 2 to 14
  constexpr int a = 6;
  constexpr int w[ NUM ] = { 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  double sum[ NUM ];
  for( int j = 2; j < 15; j++ )
  {
    sum[ j - 2 ] = ( TAP_FILTER[ 0 ] * mean[ j - 1 ] + TAP_FILTER[ 1 ] * mean[ j ] + TAP_FILTER[ 2 ] * mean[ j + 1 ] ) / NORM_TAP;
  }
  double target = 0;
  for( int j = 0; j < NUM; j++ )
  {
    target += w[ j ] * sum[ j ];
  }
  target /= a;

  double bestDiff = std::numeric_limits<double>::max();
  int    bestIdx  = -1;
  for( int j = 0; j < NUM; j++ )
  {
    const double d = std::fabs( sum[ j ] - target );
    if( d < bestDiff || ( d == bestDiff && j > bestIdx ) )   // the larger index when equally close
    {
      bestDiff = d;
      bestIdx  = j;
    }
  }
  if( bestIdx == -1 || ( bestIdx == NUM - 1 && target < sum[ NUM - 1 ] && bestDiff > target / 2 ) )
  {
    return -1;
  }
  return Clip3( 2, 14, bestIdx + 2 );   // RDD 5 range
}

// Replaces missing cut-offs (-1) with the averages; one missing average is replaced by the other.
void FGAnalyzer::replaceCutoff( ComponentID compId, int& replacementH, int& replacementV )
{
  SeiFgc::CompModel& cm = m_compModel[ compId ];
  if( !cm.presentFlag )
  {
    return;
  }
  if( replacementH == -1 && replacementV == -1 )
  {
    cm.presentFlag = false;
    return;
  }
  if( replacementH == -1 )
  {
    replacementH = replacementV;
  }
  else if( replacementV == -1 )
  {
    replacementV = replacementH;
  }
  for( int i = 0; i < cm.numIntensityIntervals; i++ )
  {
    auto& v = cm.intensityValues[ i ].compModelValue;
    if( v[ 1 ] == -1 )
    {
      v[ 1 ] = replacementH;
    }
    if( v[ 2 ] == -1 )
    {
      v[ 2 ] = replacementV;
    }
  }
}

// Limits every cut-off to the average +-2.
void FGAnalyzer::limitCutoff( ComponentID compId, int avgH, int avgV )
{
  SeiFgc::CompModel& cm = m_compModel[ compId ];
  if( !cm.presentFlag )
  {
    return;
  }
  constexpr int delta = 2;
  for( int i = 0; i < cm.numIntensityIntervals; i++ )
  {
    auto& v = cm.intensityValues[ i ].compModelValue;
    v[ 1 ] = Clip3( avgH - delta, avgH + delta, v[ 1 ] );
    v[ 2 ] = Clip3( avgV - delta, avgV + delta, v[ 2 ] );
  }
}

// Smooths the cut-offs over consecutive intervals with a [1 2 1] filter ([3 1] at the ends), in place as in VTM.
void FGAnalyzer::limitCutoffConsecutive( ComponentID compId )
{
  SeiFgc::CompModel& cm = m_compModel[ compId ];
  if( !cm.presentFlag )
  {
    return;
  }
  const int n = cm.numIntensityIntervals;
  for( int i = 0; i < n; i++ )
  {
    for( int k = 1; k <= 2; k++ )
    {
      int& cur = cm.intensityValues[ i ].compModelValue[ k ];
      if( i == 0 )
      {
        cur = ( 3 * cur + cm.intensityValues[ i + 1 ].compModelValue[ k ] + 2 ) / 4;
      }
      else if( i == n - 1 )
      {
        cur = ( cm.intensityValues[ i - 1 ].compModelValue[ k ] + 3 * cur + 2 ) / 4;
      }
      else
      {
        cur = ( cm.intensityValues[ i - 1 ].compModelValue[ k ] + 2 * cur + cm.intensityValues[ i + 1 ].compModelValue[ k ] + 2 ) / 4;
      }
    }
  }
}

// Reduces the number of different cut-off pairs to maxUnique with a weighted k-medoids clustering (Manhattan distance,
// weights are interval widths) and maps every interval to its medoid. Returns the number of different pairs.
int FGAnalyzer::limitCutoffPairs( std::vector<CutoffPair>& pairs, int maxUnique ) const
{
  const int s = (int) pairs.size();
  if( s == 0 )
  {
    return 0;
  }

  // distinct pairs in order of first appearance, with summed weights and the intervals using them
  std::vector<CutoffPair> nodes;
  for( int i = 0; i < s; i++ )
  {
    auto it = std::find_if( nodes.begin(), nodes.end(), [ & ]( const CutoffPair& n ) { return n.p == pairs[ i ].p; } );
    if( it == nodes.end() )
    {
      nodes.push_back( CutoffPair() );
      nodes.back().p = pairs[ i ].p;
      nodes.back().w = pairs[ i ].w;
      nodes.back().idx.push_back( i );
    }
    else
    {
      it->w += pairs[ i ].w;
      it->idx.push_back( i );
    }
  }
  const int numNodes = (int) nodes.size();
  if( numNodes <= maxUnique )
  {
    return numNodes;
  }

  std::vector<std::vector<int>> dist( numNodes, std::vector<int>( numNodes, 0 ) );
  for( int u = 0; u < numNodes; u++ )
  {
    for( int v = u + 1; v < numNodes; v++ )
    {
      dist[ u ][ v ] = dist[ v ][ u ] = abs( nodes[ u ].p.first - nodes[ v ].p.first ) + abs( nodes[ u ].p.second - nodes[ v ].p.second );
    }
  }

  // start with the heaviest pair, then add heavy pairs far from the medoids chosen so far
  std::vector<int> md;
  int first = 0;
  for( int u = 1; u < numNodes; u++ )
  {
    if( nodes[ u ].w > nodes[ first ].w )
    {
      first = u;
    }
  }
  md.push_back( first );
  auto isMedoid = [ & ]( int u ) { return std::find( md.begin(), md.end(), u ) != md.end(); };
  while( (int) md.size() < maxUnique )
  {
    long long bestScore = -1;
    int bestU = -1;
    for( int u = 0; u < numNodes; u++ )
    {
      if( isMedoid( u ) )
      {
        continue;
      }
      int dmin = std::numeric_limits<int>::max();
      for( int m : md )
      {
        dmin = std::min( dmin, dist[ u ][ m ] );
      }
      const long long score = 1LL * nodes[ u ].w * dmin;
      if( score > bestScore )
      {
        bestScore = score;
        bestU = u;
      }
    }
    if( bestU < 0 )
    {
      break;
    }
    md.push_back( bestU );
  }

  std::vector<int> assignTo( numNodes, -1 );
  auto assignAll = [ & ]()
  {
    for( int u = 0; u < numNodes; u++ )
    {
      int bestM = md.front();
      int bestD = dist[ u ][ bestM ];
      for( int m : md )
      {
        const int d = dist[ u ][ m ];
        if( d < bestD || ( d == bestD && m < bestM ) )
        {
          bestD = d;
          bestM = m;
        }
      }
      assignTo[ u ] = bestM;
    }
  };
  assignAll();

  // swap a medoid with a non-medoid while that lowers the total weighted distance (at most 15 times)
  bool improved = true;
  for( int iter = 0; improved && iter < 15; iter++ )
  {
    improved = false;
    long long bestDelta = 0;
    int bestOut = -1, bestIn = -1;
    std::vector<char> isM( numNodes, 0 );
    for( int m : md )
    {
      isM[ m ] = 1;
    }
    for( int mOut : md )
    {
      for( int o = 0; o < numNodes; o++ )
      {
        if( isM[ o ] )
        {
          continue;
        }
        long long delta = 0;
        for( int u = 0; u < numNodes; u++ )
        {
          const int duOld = dist[ u ][ assignTo[ u ] ];
          const int dToO  = dist[ u ][ o ];
          int duNew;
          if( assignTo[ u ] == mOut )
          {
            int dAlt = std::numeric_limits<int>::max();
            for( int m : md )
            {
              if( m != mOut )
              {
                dAlt = std::min( dAlt, dist[ u ][ m ] );
              }
            }
            duNew = std::min( dAlt, dToO );
          }
          else
          {
            duNew = std::min( duOld, dToO );
          }
          delta += 1LL * nodes[ u ].w * ( duNew - duOld );
        }
        if( delta < bestDelta )
        {
          bestDelta = delta;
          bestOut = mOut;
          bestIn = o;
        }
      }
    }
    if( bestDelta < 0 )
    {
      *std::find( md.begin(), md.end(), bestOut ) = bestIn;
      assignAll();
      improved = true;
    }
  }

  for( int u = 0; u < numNodes; u++ )
  {
    for( int idx : nodes[ u ].idx )
    {
      pairs[ idx ].p = nodes[ assignTo[ u ] ].p;
    }
  }
  return (int) md.size();
}

} // namespace vvenc

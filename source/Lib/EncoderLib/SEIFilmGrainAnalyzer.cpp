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
constexpr double POINT_SCALE               = 1.25;  // the strength falls by this factor per extension point
constexpr double VAR_SCALE_DOWN            = 1.2;   // filter out points far below the fitted function
constexpr double VAR_SCALE_UP              = 0.6;   // filter out points far above the fitted function
constexpr int    NUM_PASSES                = 2;     // the second pass of the fit filters the data points
constexpr int    NBRS                      = 1;     // neighbours a point needs within WINDOW to be kept
constexpr int    WINDOW                    = 1;
constexpr int    MIN_INTENSITY             = 40;
constexpr int    MAX_INTENSITY             = 950;
constexpr int    MAX_STANDARD_DEVIATION    = 255;   // at 8 bits
constexpr int    BIT_DEPTH_8               = 8;
constexpr int    MAX_ALLOWED_MODEL_VALUES  = 3;
constexpr int    MAX_NUM_INTENSITIES       = 256;
constexpr int    MAX_CUTOFF_PAIRS          = 10;    // different cut-off pairs the synthesis supports per picture
constexpr double MIN_BLOCK_WEIGHT          = 1.0;   // accumulated block weight an interval needs for cut-off estimation

// Largest scaling factor signalled. The syntax allows 255 and so does the scalar synthesis of vvdec, but its x86 SIMD
// synthesis sign-extends the 8-bit scaling factors, so 128 and above would make weaker grain of the wrong sign.
constexpr int    MAX_SCALING_FACTOR        = 127;
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
#if ENABLE_SIMD_OPT_FGA && defined( TARGET_SIMD_X86 )
  initFGAnalyzerX86( k );
#endif
#if ENABLE_SIMD_OPT_FGA && defined( TARGET_SIMD_ARM )
  initFGAnalyzerARM( k );
#endif
}

void FGAnalyzer::init( int width, int height, ChromaFormat chromaFormat, const int bitDepths[ MAX_NUM_CH ], const bool doAnalysis[ MAX_NUM_COMP ] )
{
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

    const int numMeans = 1 << m_bitDepths[ toChannelType( ComponentID( c ) ) ];
    m_squaredDctSums[ c ].assign( numMeans, SquaredDctSum( BLOCK_SIZE * BLOCK_SIZE, 0.0 ) );
    m_numDctBlocks  [ c ].assign( numMeans, 0.0 );
    m_storedMeanIntensity[ c ].clear();
    m_storedStrengthIntensity[ c ].clear();
    m_storedElementsPerInterval[ c ].clear();
  }
}

// VTM's scaling factors: twice the standard deviation, at log2 scale factor 2, scaled by the strength and kept at
// MAX_SCALING_FACTOR or below. The analysis measures the chroma grain on the subsampled chroma planes, but for 4:2:0 the
// synthesis (VTM, vvdec) treats the chroma cut-off frequencies as luma resolution values and doubles them: they are
// halved here.
void FGAnalyzer::getSignalledParameters( double strength, int& log2ScaleFactor, SeiFgc::CompModel compModel[ MAX_NUM_COMP ] ) const
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
      v[ 0 ]   = std::min( MAX_SCALING_FACTOR, (int) ( scale * strength / 100.0 + 0.5 ) );
      anyGrain = anyGrain || v[ 0 ] > 0;
    }
    cm.presentFlag = anyGrain;
  }
}

// ====================================================================================================================
// Analysis of one picture
// ====================================================================================================================

void FGAnalyzer::estimateGrainParameters( const CPelUnitBuf& orig, const CPelUnitBuf& denoised )
{
  const int numComp = getNumberValidComponents( m_chromaFormat );
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
      for( int x = 0; x < (int) o.width; x++ )
      {
        tg[ x ] = (Pel) ( rd[ x ] - ro[ x ] );
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
      blocks.push_back( { x, y, picStats.mean, meanStdDev( m_kernels, grain, x, y ).stddev } );
    }
  }

  std::vector<const FlatBlock*> grainBlocks;
  for( const FlatBlock& b : blocks )
  {
    const int strength = (int) ( 2 * b.stddev + 0.5 );
    if( strength >= maxStrength )
    {
      continue;   // too strong to be grain: probably texture or motion left in the grain estimate
    }
    vecMean.push_back( b.mean );
    vecStrength.push_back( strength );
    grainBlocks.push_back( &b );
  }

  for( const FlatBlock* bp : grainBlocks )
  {
    const FlatBlock& b = *bp;
    m_kernels.squaredDct16( grain.row( b.y ) + b.x, grain.width, ( 1 << ( bitDepth + 4 ) ) - 1, m_squaredDctSums[ compId ][ b.mean ].data() );
    m_numDctBlocks[ compId ][ b.mean ]++;
  }

  estimateScalingFactors( vecMean, vecStrength, bitDepth, compId );
  estimateCutoffFreq( compId, bitDepth );
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
  bool valid = false;
  for( int i = 0; i < NUM_PASSES; i++ )
  {
    valid = fitFunction( dataX, dataY, coeffs, scalingVec, ORDER, bitDepth, i == 1, compId );
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
  }
  // without a fit, VTM keeps the previous parameters
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

  if( (int) tmpDataX.size() < order + 1 )
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
  while( xmin >= POINT_STEP && ymin > 1 && numLeft > 0 )
  {
    xmin -= POINT_STEP;
    ymin = (int) ( ymin / POINT_SCALE );
    dataX.push_back( xmin );
    dataY.push_back( ymin );
    numLeft--;
  }
  while( xmax + POINT_STEP <= ( 1 << bitDepth ) - 1 && ymax > 1 && numRight > 0 )
  {
    xmax += POINT_STEP;
    ymax = (int) ( ymax / POINT_SCALE );
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
      quantizedVec[ i + xmin ] = Clip3( 0, MAX_STANDARD_DEVIATION << ( bitDepth - BIT_DEPTH_8 ), (int) ( ymin + .5 ) );
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
    quantizedVec[ i + xmin ] = Clip3( 0, MAX_STANDARD_DEVIATION << ( bitDepth - BIT_DEPTH_8 ), (int) ( quantized[ i ] + .5 ) );
  }
  return true;
}

// Turns the quantized scaling function into intensity intervals and scaling factors of the SEI message (8-bit
// intensities). The cut-off frequencies are set afterwards.
void FGAnalyzer::setEstimatedParameters( std::vector<int>& quantizedVec, int bitDepth, ComponentID compId )
{
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
    const int newScale = ( otherWidth * parameters[ 2 ][ n ] + width * parameters[ 2 ][ i ] ) / ( otherWidth + width );
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
    const int cutoffHorizontal = cutoffFrequency( rowMean[ i ] );
    const int cutoffVertical   = cutoffFrequency( colMean[ i ] );
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

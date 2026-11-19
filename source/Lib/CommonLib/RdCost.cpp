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


/** \file     RdCost.cpp
    \brief    RD cost computation class
*/

#define DONT_UNDEF_SIZE_AWARE_PER_EL_OP

#include "RdCost.h"
#include "Rom.h"
#include "Picture.h"
#include "UnitPartitioner.h"
#include "SearchSpaceCounter.h"


//! \ingroup CommonLib
//! \{

namespace vvenc {


static Distortion fixWeightedSSE_Core( const DistParam& rcDtParam, uint32_t fixedWeight );

static void bvBitsLineCore( const Mv* predictors, const int x0, const int xStep, const int count, const int y, const bool useIMV, unsigned* bits );

RdCost::RdCost()
  : m_afpDistortFunc{ { nullptr, }, { nullptr, } }
{
}

RdCost::~RdCost()
{
}

void RdCost::setLambda( double dLambda, const BitDepths &bitDepths )
{
  m_dLambda          = dLambda;
  m_DistScale        = double(1<<SCALE_BITS) / m_dLambda;
  m_dLambdaMotionSAD = sqrt(m_dLambda);
}


// in place: the unnormalized Walsh-Hadamard transform of an n x n block (natural order)
static void psyHadamard( int* blk, int n )
{
  for( int len = 1; len < n; len <<= 1 )   // along the rows
    for( int y = 0; y < n; y++ )
      for( int x = 0; x < n; x += 2 * len )
        for( int k = x; k < x + len; k++ )
        {
          const int a = blk[y * n + k], b = blk[y * n + k + len];
          blk[y * n + k] = a + b; blk[y * n + k + len] = a - b;
        }
  for( int len = 1; len < n; len <<= 1 )   // along the columns
    for( int y = 0; y < n; y += 2 * len )
      for( int k = y; k < y + len; k++ )
        for( int x = 0; x < n; x++ )
        {
          const int a = blk[k * n + x], b = blk[( k + len ) * n + x];
          blk[k * n + x] = a + b; blk[( k + len ) * n + x] = a - b;
        }
}

// the transforms of the n x n source block and of the reconstruction rec + resi (resi may be null; unclipped)
static void psyTransformPair( const Pel* org, ptrdiff_t orgStride, const Pel* rec, ptrdiff_t recStride, const Pel* resi, ptrdiff_t resiStride, int n, int* o, int* r )
{
  for( int y = 0; y < n; y++ )
    for( int x = 0; x < n; x++ )
    {
      o[y * n + x] = org[y * orgStride + x];
      r[y * n + x] = rec[y * recStride + x] + ( resi ? resi[y * resiStride + x] : 0 );
    }
  psyHadamard( o, n );
  psyHadamard( r, n );
}

// per group of masks: the L1 of the transform coefficients of the n x n block a, or a + b summed as Pel when b is given
static void psyGroupSums( const Pel* a, ptrdiff_t aStride, const Pel* b, ptrdiff_t bStride, int n, const PsyGroupMasks& masks, int numGroups, int64_t* sums )
{
  int blk[64];
  for( int y = 0; y < n; y++ )
    for( int x = 0; x < n; x++ )
    {
      blk[y * n + x] = b ? Pel( a[y * aStride + x] + b[y * bStride + x] ) : a[y * aStride + x];
    }
  psyHadamard( blk, n );
  for( int g = 0; g < numGroups; g++ )
  {
    sums[g] = 0;
    for( int i = 0; i < n * n; i++ )
    {
      sums[g] += std::abs( blk[i] ) & masks.keep[g][i];
    }
  }
}

void RdCost::xPsyGroupSums8x8( const Pel* a, ptrdiff_t aStride, const Pel* b, ptrdiff_t bStride, int64_t sums[PSY_NUM_GROUPS] )
{
  psyGroupSums( a, aStride, b, bStride, 8, PSY_GROUPS_8x8, PSY_NUM_GROUPS, sums );
}

void RdCost::xPsyGroupSums4x4( const Pel* a, ptrdiff_t aStride, const Pel* b, ptrdiff_t bStride, int64_t sums[PSY_NUM_GROUPS_4x4] )
{
  psyGroupSums( a, aStride, b, bStride, 4, PSY_GROUPS_4x4, PSY_NUM_GROUPS_4x4, sums );
}

// the grouped band difference of one block against the source's group sums, and its band L1 (m_psyGroupDiff8x8)
static int64_t psyGroupDiff( const int64_t* recSums, const int32_t* orgGroups, int numGroups, int64_t* recL1 )
{
  int64_t groupDiff = 0;
  *recL1 = 0;
  for( int g = 0; g < numGroups; g++ )
  {
    *recL1    += recSums[g];
    groupDiff += std::abs( orgGroups[g] - recSums[g] );
  }
  return groupDiff;
}

int64_t RdCost::xPsyGroupDiff8x8( const Pel* rec, ptrdiff_t recStride, const Pel* resi, ptrdiff_t resiStride, const int32_t orgGroups[PSY_NUM_GROUPS], int64_t* recL1 )
{
  int64_t recSums[PSY_NUM_GROUPS];
  xPsyGroupSums8x8( rec, recStride, resi, resiStride, recSums );
  return psyGroupDiff( recSums, orgGroups, PSY_NUM_GROUPS, recL1 );
}

int64_t RdCost::xPsyGroupDiff4x4( const Pel* rec, ptrdiff_t recStride, const Pel* resi, ptrdiff_t resiStride, const int32_t orgGroups[PSY_NUM_GROUPS_4x4], int64_t* recL1 )
{
  int64_t recSums[PSY_NUM_GROUPS_4x4];
  xPsyGroupSums4x4( rec, recStride, resi, resiStride, recSums );
  return psyGroupDiff( recSums, orgGroups, PSY_NUM_GROUPS_4x4, recL1 );
}

void RdCost::xPsyTransform4x4Tiles( const Pel* src, ptrdiff_t stride, int numTiles, int32_t* coefs )
{
  for( int t = 0; t < numTiles; t++, src += 4, coefs += 16 )
  {
    int blk[16];
    for( int y = 0; y < 4; y++ )
      for( int x = 0; x < 4; x++ )
        blk[y * 4 + x] = src[y * stride + x];
    psyHadamard( blk, 4 );
    std::copy( blk, blk + 16, coefs );
  }
}

void RdCost::xPsyTileGroupSums4x4( const int32_t* coefs, int32_t sums[PSY_NUM_GROUPS_4x4] )
{
  for( int g = 0; g < PSY_NUM_GROUPS_4x4; g++ )
  {
    sums[g] = 0;
    for( int i = 0; i < 16; i++ )
    {
      sums[g] += std::abs( coefs[i] ) & PSY_GROUPS_4x4.keep[g][i];
    }
  }
}

// In the natural order the 8-point transform is H8 = [[H4, H4], [H4, -H4]]: coefficient 4 h + l of 8 samples is
// coefficient l of their first four plus (h = 0) or minus (h = 1) coefficient l of their last four. Along both axes,
// coefficient ( 4 vHigh + vLow, 4 uHigh + uLow ) of the 8x8 block is ( topLeft +- topRight ) +- ( bottomLeft +-
// bottomRight ) at ( vLow, uLow ) of the quarters' transforms: + for a high bit of 0, - for 1.
void RdCost::xPsyTileGroupSums8x8( const int32_t* topLeft, const int32_t* topRight, const int32_t* bottomLeft, const int32_t* bottomRight, int32_t sums[PSY_NUM_GROUPS] )
{
  std::fill( sums, sums + PSY_NUM_GROUPS, 0 );
  for( int i = 0; i < 16; i++ )   // i = 4 vLow + uLow
  {
    const int topSum    = topLeft[i] + topRight[i], topDiff    = topLeft[i] - topRight[i];
    const int bottomSum = bottomLeft[i] + bottomRight[i], bottomDiff = bottomLeft[i] - bottomRight[i];
    const int coef[2][2] = { { topSum + bottomSum, topDiff + bottomDiff }, { topSum - bottomSum, topDiff - bottomDiff } };   // [vHigh][uHigh]
    for( int vHigh = 0; vHigh < 2; vHigh++ )
      for( int uHigh = 0; uHigh < 2; uHigh++ )
      {
        const int pos = ( 4 * vHigh + ( i >> 2 ) ) * 8 + 4 * uHigh + ( i & 3 );
        for( int g = 0; g < PSY_NUM_GROUPS; g++ )
        {
          sums[g] += std::abs( coef[vHigh][uHigh] ) & PSY_GROUPS_8x8.keep[g][pos];
        }
      }
  }
}

// One 8x8 or 4x4 block of the psy difference from ONE transform of the reconstruction (rec, or rec + resi): recL1 is its
// band L1 (the group sums add up to it: what m_psyBand8x8 / m_psyBand4x4 return), and groupDiff the
// grouped band difference to the source: the sum over the groups of | source's group L1 - credit x reconstruction's
// group L1 | (see PSY_GROUPS_8x8). The source's group sums come from the picture's table (computed here for a position
// off the grid or without a table).
void RdCost::xPsyBlock( const Picture& pic, const CPelBuf& org, int x, int y, int xInArea, int yInArea, int n, const CPelBuf& rec, const CPelBuf& resi, double credit, int64_t& recL1, int64_t& groupDiff ) const
{
  const bool                  is8x8     = n == 8;
  const int                   numGroups = is8x8 ? PSY_NUM_GROUPS : PSY_NUM_GROUPS_4x4;
  const auto                  groupSums = is8x8 ? m_psyGroupSums8x8 : m_psyGroupSums4x4;
  const std::vector<int32_t>& table     = is8x8 ? pic.psyOrgGroups : pic.psyOrgGroups4x4;
  int64_t recSums[PSY_NUM_GROUPS], orgSums[PSY_NUM_GROUPS];
  groupSums( rec.bufAt( xInArea, yInArea ), rec.stride, resi.buf ? resi.bufAt( xInArea, yInArea ) : nullptr, resi.stride, recSums );
  if( !table.empty() && !( ( x | y ) & 3 ) )
  {
    const int32_t* kept = &table[( size_t( y >> 2 ) * pic.psyGridW + ( x >> 2 ) ) * numGroups];
    std::copy( kept, kept + numGroups, orgSums );
  }
  else
  {
    groupSums( org.bufAt( xInArea, yInArea ), org.stride, nullptr, 0, orgSums );
  }
  recL1     = 0;
  groupDiff = 0;
  for( int g = 0; g < numGroups; g++ )
  {
    recL1     += recSums[g];
    // without a credit (most calls) the product below is the sum itself: no conversions
    groupDiff += std::abs( orgSums[g] - ( credit == 1.0 ? recSums[g] : int64_t( credit * double( recSums[g] ) + 0.5 ) ) );
  }
}

// The same difference of an n x n block under 4 samples (ISP slivers), which has no band: every AC coefficient is its
// own group there.
int64_t RdCost::xPsySliverDiff( const Pel* org, ptrdiff_t orgStride, const Pel* rec, ptrdiff_t recStride, const Pel* resi, ptrdiff_t resiStride, int n, double credit )
{
  int64_t diff = 0;
  int     o[4], r[4];
  psyTransformPair( org, orgStride, rec, recStride, resi, resiStride, n, o, r );
  for( int i = 1; i < n * n; i++ )
  {
    diff += std::abs( std::abs( o[i] ) - ( credit == 1.0 ? std::abs( r[i] ) : int( credit * std::abs( r[i] ) + 0.5 ) ) );
  }
  return diff;
}

// the L1 norm of an n x n block's unnormalized Walsh-Hadamard transform over its psy band: for n = 8 / 4 the fine band
// PSY_BAND_8x8 / _4x4 (RdCost.h), for n = 1 / 2 (ISP sub-partitions, too small for a band) every AC coefficient. The
// transform is linear and maps a constant block to its DC coefficient alone, which no band holds, so no mean needs to be
// removed.
int64_t RdCost::xPsyBandL1( const Pel* src, ptrdiff_t stride, int n )
{
  int blk[64];
  for( int y = 0; y < n; y++ )
    for( int x = 0; x < n; x++ )
      blk[y * n + x] = src[y * stride + x];
  psyHadamard( blk, n );
  int64_t energy = 0;
  if( n >= 4 )
  {
    const int32_t* keep = n == 8 ? PSY_BAND_8x8.keep : PSY_BAND_4x4.keep;
    for( int i = 0; i < n * n; i++ )
      energy += std::abs( blk[i] ) & keep[i];
  }
  else
  {
    for( int i = 1; i < n * n; i++ )
      energy += std::abs( blk[i] );
  }
  return energy;
}

int64_t RdCost::xPsyEnergy( const Pel* src, ptrdiff_t stride, int n, int shift ) const
{
  const int64_t l1 = n == 8 ? m_psyBand8x8( src, stride )
                   : n == 4 ? m_psyBand4x4( src, stride )
                            : xPsyBandL1( src, stride, n );
  return ( l1 / n ) >> shift;   // / n: the unnormalized transform's gain; >> shift: to 8-bit sample units
}

// The structure tensors ( Jxx, Jyy, Jxy ) of the fine band of the 16x16 tiles at every 4-sample grid position that lies
// inside the picture: in a tile, the band is f = 25 x sample - its 5x5 sum on the inner 12x12, and the tensor sums the
// products of f's central differences over the inner 10x10. A tile's inner 5x5 sums never reach outside the tile, so f and
// its gradient products are per-sample quantities of the picture: each is computed once, in one pass down the rows (ring
// buffers of 5 rows of horizontal 5-sums and 3 rows of f), and a tile's 10x10 sum is a difference of prefix sums along the
// row (per tile column) and of running sums down the columns (snapshots at the tiles' first rows). onTile( x, y, J ) is
// called for every tile, row by row.
template<typename OnTile>
static void forEachPsyTileTensor( const CPelBuf& org, OnTile onTile )
{
  const int W = int( org.width ), H = int( org.height );
  if( W < 16 || H < 16 )
  {
    return;
  }
  const int tilesX = ( W - 16 ) / 4 + 1, tilesY = ( H - 16 ) / 4 + 1;
  const int lastX  = 4 * ( tilesX - 1 ) + 12, lastY = 4 * ( tilesY - 1 ) + 12;   // the last gradient column and row used
  std::vector<int>     h5( 5 * W ), f( 3 * W );                 // rings: h5 row r at ( r % 5 ), f row r at ( r % 3 )
  std::vector<int64_t> rowPrefix( 3 * ( lastX + 2 ), 0 );       // this row's prefix sums of the three products (at 3: 0)
  std::vector<int64_t> colSum( 3 * tilesX, 0 ), firstRowSum( 3 * 3 * tilesX );   // running sums; ring of 3 tile rows
  auto h5Row = [&]( int r ) {
    const Pel* v = org.bufAt( 0, r );
    int*       h = &h5[( r % 5 ) * W];
    for( int x = 2; x < W - 2; x++ )
    {
      h[x] = v[x - 2] + v[x - 1] + v[x] + v[x + 1] + v[x + 2];
    }
  };
  for( int r = 0; r < 4; r++ )
  {
    h5Row( r );
  }
  for( int r = 2; r <= lastY + 1; r++ )   // f row r, then the gradient row r - 1
  {
    h5Row( r + 2 );
    const Pel* v = org.bufAt( 0, r );
    int*       fr = &f[( r % 3 ) * W];
    for( int x = 2; x <= lastX + 1; x++ )
    {
      fr[x] = 25 * v[x] - ( h5[( ( r - 2 ) % 5 ) * W + x] + h5[( ( r - 1 ) % 5 ) * W + x] + h5[( r % 5 ) * W + x]
                            + h5[( ( r + 1 ) % 5 ) * W + x] + h5[( ( r + 2 ) % 5 ) * W + x] );
    }
    const int y = r - 1;
    if( y < 3 )
    {
      continue;
    }
    if( ( y - 3 ) % 4 == 0 && ( y - 3 ) / 4 < tilesY )   // the first row of tile row ( y - 3 ) / 4: keep the sums above it
    {
      std::copy( colSum.begin(), colSum.end(), firstRowSum.begin() + 3 * tilesX * ( ( ( y - 3 ) / 4 ) % 3 ) );
    }
    const int* fu = &f[( ( y - 1 ) % 3 ) * W];
    const int* fc = &f[( y % 3 ) * W];
    const int* fd = &f[( ( y + 1 ) % 3 ) * W];
    int64_t    pxx = 0, pyy = 0, pxy = 0;
    for( int x = 3; x <= lastX; x++ )   // rowPrefix[ x + 1 ] = the sums over columns 3 .. x
    {
      const int64_t gx = fc[x + 1] - fc[x - 1];
      const int64_t gy = fd[x] - fu[x];
      pxx += gx * gx;
      pyy += gy * gy;
      pxy += gx * gy;
      rowPrefix[3 * ( x + 1 )] = pxx;
      rowPrefix[3 * ( x + 1 ) + 1] = pyy;
      rowPrefix[3 * ( x + 1 ) + 2] = pxy;
    }
    for( int tx = 0; tx < tilesX; tx++ )   // columns 4 tx + 3 .. 4 tx + 12
      for( int k = 0; k < 3; k++ )
      {
        colSum[3 * tx + k] += rowPrefix[3 * ( 4 * tx + 13 ) + k] - rowPrefix[3 * ( 4 * tx + 3 ) + k];
      }
    if( ( y - 12 ) % 4 == 0 && y >= 12 )   // the last row of tile row ( y - 12 ) / 4
    {
      const int      ty    = ( y - 12 ) / 4;
      const int64_t* above = &firstRowSum[3 * tilesX * ( ty % 3 )];
      for( int tx = 0; tx < tilesX; tx++ )
      {
        const int64_t J[3] = { colSum[3 * tx] - above[3 * tx], colSum[3 * tx + 1] - above[3 * tx + 1], colSum[3 * tx + 2] - above[3 * tx + 2] };
        onTile( tx, ty, J );
      }
    }
  }
}

// Psy-rd rewards the reconstruction's fine energy; in grain an angular intra mode earns that reward with a directional
// streak (a brush stroke). IntraSearch withholds part of the reward from angular modes, but not near real source edges,
// where a directional mode is right. A 16x16 tile at a 4-sample grid position is an edge when its fine band is coherent
// (c = ( l1 - l2 ) / ( l1 + l2 ) of the structure tensor > 0.6) and strong (RMS gradient > 4 x the median of the
// picture's grain-like tiles, c < 0.43, sampled every 16 samples); every position with an edge tile within 4 grid units
// (16 samples) in both directions is gated. Tiles crossing the right or bottom picture edge are never edges.
static void buildPsyEdgeGate( Picture& pic, const CPelBuf& org, int gridW, int gridH )
{
  const size_t        n = size_t( gridW ) * gridH;
  std::vector<double> g( n, 0.0 ), c( n, 0.0 ), grainG;
  forEachPsyTileTensor( org, [&]( int x, int y, const int64_t t[3] ) {
    const double tr = double( t[0] + t[1] );
    const size_t i  = size_t( y ) * gridW + x;
    g[i] = sqrt( std::max( tr, 0.0 ) );
    c[i] = tr > 0 ? sqrt( double( t[0] - t[1] ) * double( t[0] - t[1] ) + 4.0 * double( t[2] ) * double( t[2] ) ) / tr : 0.0;
    if( !( ( x | y ) & 3 ) && c[i] < 0.43 )
    {
      grainG.push_back( g[i] );
    }
  } );
  double grainLevel = 0;
  if( !grainG.empty() )
  {
    std::nth_element( grainG.begin(), grainG.begin() + grainG.size() / 2, grainG.end() );
    grainLevel = grainG[grainG.size() / 2];
  }
  // dilation by 4 grid units, separable: along rows into rowGate, then along columns
  const int            reach = 4;
  std::vector<uint8_t> rowGate( n, 0 );
  for( int y = 0; y < gridH; y++ )
    for( int x = 0; x < gridW; x++ )
    {
      const size_t i = size_t( y ) * gridW + x;
      if( c[i] > 0.6 && g[i] > 4.0 * grainLevel )
      {
        for( int xx = std::max( 0, x - reach ); xx <= std::min( gridW - 1, x + reach ); xx++ )
        {
          rowGate[size_t( y ) * gridW + xx] = 1;
        }
      }
    }
  pic.psyEdgeGate.assign( n, 0 );
  for( int y = 0; y < gridH; y++ )
    for( int x = 0; x < gridW; x++ )
    {
      if( rowGate[size_t( y ) * gridW + x] )
      {
        for( int yy = std::max( 0, y - reach ); yy <= std::min( gridH - 1, y + reach ); yy++ )
        {
          pic.psyEdgeGate[size_t( yy ) * gridW + x] = 1;
        }
      }
    }
}

// The median of a histogram of numBlocks energies (the last bin holds everything from 255), placed inside its bin by
// linear interpolation so that it moves smoothly from picture to picture; 0 without blocks
static double psyMedianEnergy( const int histogram[256], int numBlocks )
{
  if( numBlocks == 0 )
  {
    return 0.0;
  }
  const double half = 0.5 * numBlocks;
  int energy = 0, numBelow = 0;
  while( energy < 255 && numBelow + histogram[energy] < half )
  {
    numBelow += histogram[energy++];
  }
  return energy + ( half - numBelow ) / histogram[energy];
}

// The source side of the psy distortion is the same for every candidate at a position, so its energies are computed
// once per picture, on the 4-sample grid every psy block starts on (coding areas start on it; blocks are 4 or 8 apart).
// 8x8 blocks crossing the right or bottom picture edge are never looked up: their entries stay 0.
// Every 8x8 block on the grid is made of four 4x4 blocks on it, and its transform is a butterfly of their transforms
// (xPsyTileGroupSums8x8): each 4x4 block is transformed once, for its own sums and those of the four 8x8 blocks it is a
// quarter of. Two rows of 4x4 transforms are kept (128 bytes per 4 columns of the picture).
void RdCost::buildPsyOrgEnergies( Picture& pic, int bitDepth ) const
{
  const CPelBuf org   = pic.getOrigBuf( COMP_Y );
  const int     shift = bitDepth - 8;
  const int     gridW = int( org.width ) / 4, gridH = int( org.height ) / 4;
  pic.psyGridW = gridW;
  pic.psyOrgEnergy[0].assign( size_t( gridW ) * gridH, 0 );
  pic.psyOrgEnergy[1].assign( size_t( gridW ) * gridH, 0 );
  pic.psyOrgGroups.assign( size_t( gridW ) * gridH * PSY_NUM_GROUPS, 0 );
  pic.psyOrgGroups4x4.assign( size_t( gridW ) * gridH * PSY_NUM_GROUPS_4x4, 0 );
  std::vector<int32_t> tileRows( 2 * size_t( gridW ) * 16 );
  int32_t*             tilesHere  = tileRows.data();                    // the 4x4 transforms of grid row y
  int32_t*             tilesBelow = tilesHere + size_t( gridW ) * 16;   // and of grid row y + 1
  if( gridH > 0 )
  {
    m_psyTransform4x4Tiles( org.bufAt( 0, 0 ), org.stride, gridW, tilesHere );
  }
  for( int y = 0; y < gridH; y++ )
  {
    const bool has8x8 = 4 * y + 8 <= int( org.height );   // the row of 4x4 blocks below is in the picture
    if( has8x8 )
    {
      m_psyTransform4x4Tiles( org.bufAt( 0, 4 * y + 4 ), org.stride, gridW, tilesBelow );
    }
    for( int x = 0; x < gridW; x++ )
    {
      const size_t   pos    = size_t( y ) * gridW + x;
      const int32_t* tile   = tilesHere + 16 * size_t( x );
      int32_t*       groups = &pic.psyOrgGroups4x4[pos * PSY_NUM_GROUPS_4x4];
      int64_t        l1     = 0;   // the sum of the groups is the band L1 of xPsyEnergy
      m_psyTileGroupSums4x4( tile, groups );
      for( int g = 0; g < PSY_NUM_GROUPS_4x4; g++ )
      {
        l1 += groups[g];
      }
      pic.psyOrgEnergy[1][pos] = int32_t( ( l1 / 4 ) >> shift );
      if( has8x8 && 4 * x + 8 <= int( org.width ) )
      {
        const int32_t* tileBelow = tilesBelow + 16 * size_t( x );
        groups = &pic.psyOrgGroups[pos * PSY_NUM_GROUPS];
        l1     = 0;
        m_psyTileGroupSums8x8( tile, tile + 16, tileBelow, tileBelow + 16, groups );
        for( int g = 0; g < PSY_NUM_GROUPS; g++ )
        {
          l1 += groups[g];
        }
        pic.psyOrgEnergy[0][pos] = int32_t( ( l1 / 8 ) >> shift );
      }
    }
    std::swap( tilesHere, tilesBelow );
  }
  buildPsyEdgeGate( pic, org, gridW, gridH );
  // The picture's grain floor: the 10th percentile of the source's 8x8 band energies, blocks of exactly zero energy
  // (black bars, flat graphics) left out. A source with grain has it everywhere, also in its flattest tenth; a clean
  // source reads next to nothing there. The convexity of the lost-grain term follows it (see PSY_GRAIN_FLOOR_NONE).
  int histogram[256] = {};
  int numBlocks = 0;
  for( int y = 0; 4 * y + 8 <= int( org.height ); y++ )
    for( int x = 0; 4 * x + 8 <= int( org.width ); x++ )
    {
      const int energy = pic.psyOrgEnergy[0][y * gridW + x];
      if( energy > 0 )
      {
        histogram[std::min( energy, 255 )]++;
        numBlocks++;
      }
    }
  int floorEnergy = 0, numBelow = 0;   // the smallest energy with more than a tenth of the blocks at or below it
  while( floorEnergy < 255 && ( numBelow + histogram[floorEnergy] ) * 10 <= numBlocks )
  {
    numBelow += histogram[floorEnergy++];
  }
  const double grainShare  = Clip3( 0.0, 1.0, double( floorEnergy - PSY_GRAIN_FLOOR_NONE ) / double( PSY_GRAIN_FLOOR_FULL - PSY_GRAIN_FLOOR_NONE ) );
  const double changeShare = Clip3( 0.0, 1.0, ( PSY_STABLE_SHARE_NONE - double( pic.psyStableShare ) ) / ( PSY_STABLE_SHARE_NONE - PSY_STABLE_SHARE_FULL ) );
  const double termShare  = numBlocks ? grainShare * changeShare : 0.0;
  pic.psyGrainShare       = termShare;
  pic.psyDeficitConvexity = PSY_DEFICIT_CONVEXITY * termShare;
  // The picture's grain level, for the rate ramp's grain delay (see PSY_RATE_QP_FULL): the median, not the floor,
  // which moves twice as much from picture to picture where the source is itself an encode (Top Gun: every 6th
  // picture carries more grain)
  pic.psyGrainLevel       = psyMedianEnergy( histogram, numBlocks );
}

// the source's energy of the n x n block at picture position ( x, y ), from the picture's table; ISP sub-partitions
// 1 or 2 samples wide give n = 1 or 2 at positions off the 4-sample grid: those are computed here
int64_t RdCost::xPsyOrgEnergy( const Picture& pic, const CPelBuf& org, int x, int y, int xInArea, int yInArea, int n, int shift ) const
{
  const std::vector<int32_t>& table = pic.psyOrgEnergy[n == 8 ? 0 : 1];
  if( table.empty() || ( n != 8 && n != 4 ) || ( ( x | y ) & 3 ) )
  {
    return xPsyEnergy( org.bufAt( xInArea, yInArea ), org.stride, n, shift );
  }
  return table[( y >> 2 ) * pic.psyGridW + ( x >> 2 )];
}

// The band-energy difference of one block, | orgEnergy - recEnergy |, made convex where the block lost energy beyond the
// picture's usual loss (see PSY_DEFICIT_CONVEXITY). At or below the usual loss the convex term is exactly 0.
static inline int64_t psyEnergyDiff( const Picture& pic, int64_t orgEnergy, int64_t recEnergy )
{
  int64_t energyDiff = std::abs( orgEnergy - recEnergy );
  if( recEnergy < orgEnergy && pic.psyDeficitConvexity > 0.0 )   // lost grain
  {
    const double lost = double( energyDiff ) / double( orgEnergy );
    if( lost > pic.psyUsualLoss )
    {
      const double beyondUsual = ( lost - pic.psyUsualLoss ) / ( 1.0 - pic.psyUsualLoss );
      energyDiff += int64_t( pic.psyDeficitConvexity * double( energyDiff ) * beyondUsual + 0.5 );
    }
  }
  return energyDiff;
}

// A block's share of the psy difference: the band-energy difference and, with PSY_GROUP_WEIGHT, the group-by-group
// difference (see PSY_GROUPS_8x8), rounded: int64( 0.25 g + 0.75 e + 0.5 ), which is ( g + 3 e + 2 ) / 4 exactly in
// integers (both are never negative, and far below 2^50, where the double sum has no rounding)
static_assert( PSY_GROUP_WEIGHT == 0.25, "psyBlockDiff computes the weighted sum in integers for this weight" );
static inline int64_t psyBlockDiff( int64_t groupDiff, int64_t energyDiff )
{
  return ( groupDiff + 3 * energyDiff + 2 ) >> 2;
}

// true when every n x n block of the area (n = 8 or 4) finds the source's energy and group sums in the picture's tables:
// the tables exist and the area starts on their 4-sample grid (so does every block in it). Areas lie inside the picture.
static bool psyTablesCover( const Picture& pic, const CompArea& area, int n )
{
  return n >= 4 && !pic.psyOrgGroups.empty() && !( ( area.x | area.y ) & 3 );
}

// getPsyDistortion's sum over the blocks (before strength and lambda) where the tables cover the area, the case of
// nearly every call: the source side comes from the tables, and one transform per block gives the reconstruction's
// band L1 and its groups (without a credit, the usual case, one kernel call also gives the grouped difference). The
// same values as xPsyDiffAnyBlock: the sums are never negative, so its divisions by n are these shifts.
template<int n>
int64_t RdCost::xPsyDiffFromTables( const Picture& pic, const CompArea& area, const CPelBuf& rec, const CPelBuf& resi, int shift ) const
{
  static_assert( n == 8 || n == 4, "the tables hold 8x8 and 4x4 blocks" );
  constexpr int  log2n     = n == 8 ? 3 : 2;
  constexpr int  numGroups = n == 8 ? PSY_NUM_GROUPS : PSY_NUM_GROUPS_4x4;
  constexpr int  gridStep  = n / 4;   // grid positions from one block to the next
  const auto     groupDiff = n == 8 ? m_psyGroupDiff8x8 : m_psyGroupDiff4x4;
  const auto     groupSums = n == 8 ? m_psyGroupSums8x8 : m_psyGroupSums4x4;
  const double   credit    = m_psyRecCredit;
  const size_t   gridW     = pic.psyGridW;
  const size_t   gridPos   = size_t( area.y >> 2 ) * gridW + ( area.x >> 2 );
  const int32_t* orgEnergy = &pic.psyOrgEnergy[n == 8 ? 0 : 1][gridPos];
  const int32_t* orgGroups = &( n == 8 ? pic.psyOrgGroups : pic.psyOrgGroups4x4 )[gridPos * numGroups];
  const int      unitShift = log2n + shift;   // / n: the unnormalized transform's gain; >> shift: to 8-bit sample units
  int64_t        diff      = 0;
  for( int y = 0; y + n <= int( area.height ); y += n )
  {
    for( int x = 0, gx = 0; x + n <= int( area.width ); x += n, gx += gridStep )
    {
      const Pel*     recBlk  = rec.bufAt( x, y );
      const Pel*     resiBlk = resi.buf ? resi.bufAt( x, y ) : nullptr;
      const int32_t* org     = orgGroups + size_t( gx ) * numGroups;
      int64_t        recL1, blockGroupDiff, recEnergy;
      if( credit == 1.0 )
      {
        blockGroupDiff = groupDiff( recBlk, rec.stride, resiBlk, resi.stride, org, &recL1 );
        recEnergy      = recL1 >> unitShift;
      }
      else   // only the credited share of the reconstruction's energy counts, as in xPsyBlock and xPsyDiffAnyBlock
      {
        int64_t recSums[numGroups];
        groupSums( recBlk, rec.stride, resiBlk, resi.stride, recSums );
        recL1          = 0;
        blockGroupDiff = 0;
        for( int g = 0; g < numGroups; g++ )
        {
          recL1          += recSums[g];
          blockGroupDiff += std::abs( org[g] - int64_t( credit * double( recSums[g] ) + 0.5 ) );
        }
        recEnergy = int64_t( double( recL1 >> unitShift ) * credit + 0.5 );
      }
      diff += psyBlockDiff( blockGroupDiff >> unitShift, psyEnergyDiff( pic, orgEnergy[gx], recEnergy ) );
    }
    orgEnergy += gridStep * gridW;
    orgGroups += gridStep * gridW * numGroups;
  }
  return diff;
}

int64_t RdCost::getPsyDiff( const Picture& pic, const CompArea& area, const CPelBuf& rec, const CPelBuf& resi, int bitDepth ) const
{
  const int n     = std::min<int>( 8, std::min<int>( area.width, area.height ) );
  const int shift = bitDepth - 8;
  if( psyTablesCover( pic, area, n ) )
  {
    return n == 8 ? xPsyDiffFromTables<8>( pic, area, rec, resi, shift ) : xPsyDiffFromTables<4>( pic, area, rec, resi, shift );
  }
  return xPsyDiffAnyBlock( pic, area, rec, resi, shift );
}

Distortion RdCost::getPsyDistortion( int64_t psyDiff ) const
{
  return Distortion( m_psyStrength * sqrt( m_dLambda ) * double( psyDiff ) + 0.5 );
}

// getPsyDistortion's sum over the blocks in every case: ISP slivers, areas off the tables' grid, pictures without tables
int64_t RdCost::xPsyDiffAnyBlock( const Picture& pic, const CompArea& area, const CPelBuf& rec, const CPelBuf& resi, int shift ) const
{
  const CPelBuf org   = pic.getOrigBuf( area );
  const int     n     = std::min<int>( 8, std::min<int>( org.width, org.height ) );
  int64_t       diff  = 0;
  Pel           sum[64];
  for( int y = 0; y + n <= int( org.height ); y += n )
    for( int x = 0; x + n <= int( org.width ); x += n )
    {
      int64_t recEnergy, groupDiff;
      if( n >= 4 )   // one transform of the reconstruction gives its band L1 and its groups
      {
        int64_t recL1;
        xPsyBlock( pic, org, area.x + x, area.y + y, x, y, n, rec, resi, m_psyRecCredit, recL1, groupDiff );
        recEnergy = ( recL1 / n ) >> shift;
        groupDiff = ( groupDiff / n ) >> shift;
      }
      else if( resi.buf )
      {
        for( int j = 0; j < n; j++ )
          for( int i = 0; i < n; i++ )
            sum[j * n + i] = Pel( rec.at( x + i, y + j ) + resi.at( x + i, y + j ) );   // not clipped: an estimate
        recEnergy = xPsyEnergy( sum, n, n, shift );
      }
      else
      {
        recEnergy = xPsyEnergy( rec.bufAt( x, y ), rec.stride, n, shift );
      }
      if( m_psyRecCredit != 1.0 )
      {
        recEnergy = int64_t( double( recEnergy ) * m_psyRecCredit + 0.5 );
      }
      const int64_t energyDiff = psyEnergyDiff( pic, xPsyOrgEnergy( pic, org, area.x + x, area.y + y, x, y, n, shift ), recEnergy );
      if( n < 4 )
      {
        groupDiff = ( xPsySliverDiff( org.bufAt( x, y ), org.stride, rec.bufAt( x, y ), rec.stride, resi.buf ? resi.bufAt( x, y ) : nullptr, resi.stride, n, m_psyRecCredit ) / n ) >> shift;
      }
      diff += psyBlockDiff( groupDiff, energyDiff );
    }
  return diff;
}

// getPsyGroupGap where the tables cover the area: the source's group sums from the tables, one kernel call per block
template<int n>
int64_t RdCost::xPsyGroupGapFromTables( const Picture& pic, const CompArea& area, const CPelBuf& pred, const CPelBuf& resi ) const
{
  constexpr int  numGroups = n == 8 ? PSY_NUM_GROUPS : PSY_NUM_GROUPS_4x4;
  constexpr int  gridStep  = n / 4;
  const auto     groupDiff = n == 8 ? m_psyGroupDiff8x8 : m_psyGroupDiff4x4;
  const size_t   gridW     = pic.psyGridW;
  const int32_t* orgGroups = &( n == 8 ? pic.psyOrgGroups : pic.psyOrgGroups4x4 )[( size_t( area.y >> 2 ) * gridW + ( area.x >> 2 ) ) * numGroups];
  int64_t        gap       = 0;
  for( int y = 0; y + n <= int( area.height ); y += n )
  {
    for( int x = 0, gx = 0; x + n <= int( area.width ); x += n, gx += gridStep )
    {
      int64_t recL1;
      gap += groupDiff( pred.bufAt( x, y ), pred.stride, resi.buf ? resi.bufAt( x, y ) : nullptr, resi.stride, orgGroups + size_t( gx ) * numGroups, &recL1 );
    }
    orgGroups += gridStep * gridW * numGroups;
  }
  return gap;
}

int64_t RdCost::getPsyGroupGap( const Picture& pic, const CompArea& area, const CPelBuf& pred, const CPelBuf& resi ) const
{
  const CPelBuf org = pic.getOrigBuf( area );
  const int     n   = std::min<int>( 8, std::min<int>( org.width, org.height ) );
  if( psyTablesCover( pic, area, n ) )
  {
    return n == 8 ? xPsyGroupGapFromTables<8>( pic, area, pred, resi ) : xPsyGroupGapFromTables<4>( pic, area, pred, resi );
  }
  int64_t gap = 0;
  for( int y = 0; y + n <= int( org.height ); y += n )
    for( int x = 0; x + n <= int( org.width ); x += n )
    {
      if( n >= 4 )
      {
        int64_t recL1, groupDiff;
        xPsyBlock( pic, org, area.x + x, area.y + y, x, y, n, pred, resi, 1.0, recL1, groupDiff );
        gap += groupDiff;
      }
      else
      {
        gap += xPsySliverDiff( org.bufAt( x, y ), org.stride, pred.bufAt( x, y ), pred.stride, resi.buf ? resi.bufAt( x, y ) : nullptr, resi.stride, n, 1.0 );
      }
    }
  return gap;
}

// The share of the source's 8x8 band energy that the picture's final reconstruction lost: what the pictures referencing
// it take as their usual loss (see PSY_USUAL_LOSS_SCALE_LOW). Needs the picture's source tables (before finalizePicture).
double RdCost::getPsyLostShare( const Picture& pic, int bitDepth ) const
{
  const CompArea area  = pic.Y();
  const CPelBuf  org   = pic.getOrigBuf( area );
  const CPelBuf  rec   = pic.getRecoBuf( area );
  const int      shift = bitDepth - 8;
  int64_t        lost  = 0;
  int64_t        total = 0;
  for( int y = 0; y + 8 <= int( org.height ); y += 8 )
    for( int x = 0; x + 8 <= int( org.width ); x += 8 )
    {
      const int64_t orgEnergy = xPsyOrgEnergy( pic, org, area.x + x, area.y + y, x, y, 8, shift );
      const int64_t recEnergy = xPsyEnergy( rec.bufAt( x, y ), rec.stride, 8, shift );
      total += orgEnergy;
      lost  += std::max<int64_t>( 0, orgEnergy - recEnergy );
    }
  return total ? double( lost ) / double( total ) : 0.0;
}

int64_t RdCost::getPsyEnergyGap( const Picture& pic, const CompArea& area, const CPelBuf& rec, int bitDepth ) const
{
  const CPelBuf org   = pic.getOrigBuf( area );
  const int     shift = bitDepth - 8;
  int64_t       gap   = 0;
  for( int y = 0; y + 8 <= int( org.height ); y += 8 )
    for( int x = 0; x + 8 <= int( org.width ); x += 8 )
    {
      const int64_t recEnergy = xPsyEnergy( rec.bufAt( x, y ), rec.stride, 8, shift );
      gap += std::abs( xPsyOrgEnergy( pic, org, area.x + x, area.y + y, x, y, 8, shift ) - recEnergy );
    }
  return gap;
}

// Initialize Function Pointer by [eDFunc]
void RdCost::create( bool enableOpt )
{
  m_chromaWeight               = 1.0;
  m_afpDistortFunc[0][DF_SSE    ] = RdCost::xGetSSE;
  m_afpDistortFunc[0][DF_SSE2   ] = RdCost::xGetSSE;
  m_afpDistortFunc[0][DF_SSE4   ] = RdCost::xGetSSE4;
  m_afpDistortFunc[0][DF_SSE8   ] = RdCost::xGetSSE8;
  m_afpDistortFunc[0][DF_SSE16  ] = RdCost::xGetSSE16;
  m_afpDistortFunc[0][DF_SSE32  ] = RdCost::xGetSSE32;
  m_afpDistortFunc[0][DF_SSE64  ] = RdCost::xGetSSE64;
  m_afpDistortFunc[0][DF_SSE128 ] = RdCost::xGetSSE128;

  m_afpDistortFunc[0][DF_SAD    ] = RdCost::xGetSAD;
  m_afpDistortFunc[0][DF_SAD2   ] = RdCost::xGetSAD;
  m_afpDistortFunc[0][DF_SAD4   ] = RdCost::xGetSAD4;
  m_afpDistortFunc[0][DF_SAD8   ] = RdCost::xGetSAD8;
  m_afpDistortFunc[0][DF_SAD16  ] = RdCost::xGetSAD16;
  m_afpDistortFunc[0][DF_SAD32  ] = RdCost::xGetSAD32;
  m_afpDistortFunc[0][DF_SAD64  ] = RdCost::xGetSAD64;
  m_afpDistortFunc[0][DF_SAD128 ] = RdCost::xGetSAD128;

  m_afpDistortFunc[0][DF_HAD    ] = RdCost::xGetHADs<false>;
  m_afpDistortFunc[0][DF_HAD2   ] = RdCost::xGetHADs<false>;
  m_afpDistortFunc[0][DF_HAD4   ] = RdCost::xGetHADs<false>;
  m_afpDistortFunc[0][DF_HAD8   ] = RdCost::xGetHADs<false>;
  m_afpDistortFunc[0][DF_HAD16  ] = RdCost::xGetHADs<false>;
  m_afpDistortFunc[0][DF_HAD32  ] = RdCost::xGetHADs<false>;
  m_afpDistortFunc[0][DF_HAD64  ] = RdCost::xGetHADs<false>;
  m_afpDistortFunc[0][DF_HAD128 ] = RdCost::xGetHADs<false>;

  m_afpDistortFunc[0][DF_HAD_fast    ] = RdCost::xGetHADs<true>;
  m_afpDistortFunc[0][DF_HAD2_fast   ] = RdCost::xGetHADs<true>;
  m_afpDistortFunc[0][DF_HAD4_fast   ] = RdCost::xGetHADs<true>;
  m_afpDistortFunc[0][DF_HAD8_fast   ] = RdCost::xGetHADs<true>;
  m_afpDistortFunc[0][DF_HAD16_fast  ] = RdCost::xGetHADs<true>;
  m_afpDistortFunc[0][DF_HAD32_fast  ] = RdCost::xGetHADs<true>;
  m_afpDistortFunc[0][DF_HAD64_fast  ] = RdCost::xGetHADs<true>;
  m_afpDistortFunc[0][DF_HAD128_fast ] = RdCost::xGetHADs<true>;

  //  m_afpDistortFunc[0][DF_SAD_INTERMEDIATE_BITDEPTH] = RdCost::xGetSAD;
  m_afpDistortFunc[0][DF_HAD_2SAD ] = RdCost::xGetHAD2SADs;

  m_afpDistortFunc[0][DF_SAD_WITH_MASK] = RdCost::xGetSADwMask;
  // m_afpDistortFunc[1] can be used in any case
  memcpy( m_afpDistortFunc[1], m_afpDistortFunc[0], sizeof(m_afpDistortFunc)/2);

  m_fxdWtdPredPtr = fixWeightedSSE_Core;
  m_bvBitsLineIBC = bvBitsLineCore;
  m_psyBand8x8    = RdCost::xPsyBand8x8;
  m_psyBand4x4    = RdCost::xPsyBand4x4;
  m_psyGroupSums8x8 = RdCost::xPsyGroupSums8x8;
  m_psyGroupSums4x4 = RdCost::xPsyGroupSums4x4;
  m_psyGroupDiff8x8 = RdCost::xPsyGroupDiff8x8;
  m_psyGroupDiff4x4 = RdCost::xPsyGroupDiff4x4;
  m_psyTransform4x4Tiles = RdCost::xPsyTransform4x4Tiles;
  m_psyTileGroupSums4x4  = RdCost::xPsyTileGroupSums4x4;
  m_psyTileGroupSums8x8  = RdCost::xPsyTileGroupSums8x8;

  m_afpDistortFuncX5[0] = RdCost::xGetSAD8X5;
  m_afpDistortFuncX5[1] = RdCost::xGetSAD16X5;

#if ENABLE_SIMD_OPT_DIST
  if( enableOpt )
  {
#ifdef TARGET_SIMD_X86
    initRdCostX86();
#endif
#ifdef TARGET_SIMD_ARM
    initRdCostARM();
#endif
  }
#endif

  m_costMode      = VVENC_COST_STANDARD_LOSSY;
  setMotionLambda( 0 );
  m_iCostScale    = 0;
}

#if ENABLE_MEASURE_SEARCH_SPACE
static Distortion xMeasurePredSearchSpaceInterceptor( const DistParam& dp )
{
  g_searchSpaceAcc.addPrediction( dp.org.width, dp.org.height, toChannelType( dp.compID ) );
  return dp.xDistFunc( dp );
}

#endif
void RdCost::setDistParam( DistParam &rcDP, const CPelBuf& org, const Pel* piRefY, int iRefStride, int bitDepth, ComponentID compID, int subShiftMode, int useHadamard )
{
  rcDP.bitDepth   = bitDepth;
  rcDP.compID     = compID;

  // set Original & Curr Pointer / Stride
  rcDP.org        = org;

  rcDP.cur.buf    = piRefY;
  rcDP.cur.stride = iRefStride;

  // set Block Width / Height
  rcDP.cur.width    = org.width;
  rcDP.cur.height   = org.height;
  rcDP.maximumDistortionForEarlyExit = MAX_DISTORTION;

  const int base = (rcDP.bitDepth > 10 || rcDP.applyWeight) ? 1 : 0;
  if( !useHadamard )
  {
    rcDP.distFunc = m_afpDistortFunc[base][ DF_SAD + Log2( org.width ) ];
    rcDP.sadFourPositions = base == 0 ? m_afpSadFourPositions[Log2( org.width )] : nullptr;
  }
  else
  {
    rcDP.distFunc = m_afpDistortFunc[base][( useHadamard == 1 ? DF_HAD : DF_HAD_fast ) + Log2( org.width ) ];
    rcDP.sadFourPositions = nullptr;
  }

  // initialize
  rcDP.subShift  = 0;

  if( subShiftMode == 1 )
  {
    if( rcDP.org.height > 8 && rcDP.org.width <= 128 )
    {
      rcDP.subShift = 1;
    }
  }
  else if( subShiftMode == 2 )
  {
    if (rcDP.org.height > 8)
    {
      rcDP.subShift = 1;
    }
  }

#if ENABLE_MEASURE_SEARCH_SPACE
  rcDP.xDistFunc = rcDP.distFunc;
  rcDP.distFunc  = xMeasurePredSearchSpaceInterceptor;
#endif
}


DistParam RdCost::setDistParam( const CPelBuf& org, const CPelBuf& cur, int bitDepth, DFunc dfunc )
{
  int index = dfunc;
  if( dfunc != DF_HAD_2SAD )   // DF_HAD_2SAD has a single entry for all sizes
  {
    index += Log2(org.width);
  }

  const int base = bitDepth > 10 ? 1:0; //TBD: check does SDA ever overflow
#if ENABLE_MEASURE_SEARCH_SPACE
  DistParam rcDP( org.buf, org.stride, cur.buf, cur.stride, org.width, org.height, m_afpDistortFunc[base][index], bitDepth, 0, COMP_Y );
  rcDP.xDistFunc = rcDP.distFunc;
  rcDP.distFunc  = xMeasurePredSearchSpaceInterceptor;
  return rcDP;
#else
  return DistParam( org.buf, org.stride, cur.buf, cur.stride, org.width, org.height, m_afpDistortFunc[base][index], bitDepth, 0, COMP_Y );
#endif
}

DistParam RdCost::setDistParam( const Pel* pOrg, const Pel* piRefY, int iOrgStride, int iRefStride, int bitDepth, ComponentID compID, int width, int height, int subShift, bool isDMVR )
{
  DistParam rcDP;
  rcDP.bitDepth   = bitDepth;
  rcDP.compID     = compID;

  rcDP.org.buf    = pOrg;
  rcDP.org.stride = iOrgStride;
  rcDP.org.width  = width;
  rcDP.org.height = height;

  rcDP.cur.buf    = piRefY;
  rcDP.cur.stride = iRefStride;
  rcDP.cur.width  = width;
  rcDP.cur.height = height;
  rcDP.subShift   = subShift;

  //  CHECK( useHadamard || rcDP.useMR, "only used in xDMVRCost with these default parameters (so far...)" );
  const int base = (rcDP.bitDepth > 10) ? 1 : 0;

  rcDP.distFunc = m_afpDistortFunc[base][ DF_SAD + Log2( width ) ];
  
  if( isDMVR )
  {
    rcDP.dmvrSadX5 = m_afpDistortFuncX5[Log2( width ) - 3];
  }

#if ENABLE_MEASURE_SEARCH_SPACE
  if( !isDMVR )
  {
    // DMVT is part of the decoder complexity
    rcDP.xDistFunc = rcDP.distFunc;
    rcDP.distFunc = xMeasurePredSearchSpaceInterceptor;
  }

#endif
  return rcDP;
}

Distortion RdCost::getDistPart( const CPelBuf& org, const CPelBuf& cur, int bitDepth, const ComponentID compId, DFunc eDFunc )
{
  DistParam dp( org.buf, org.stride, cur.buf, cur.stride, org.width, org.height, nullptr, bitDepth, 0, compId );
# if ENABLE_MEASURE_SEARCH_SPACE
  g_searchSpaceAcc.addPrediction( dp.org.width, dp.org.height, toChannelType( dp.compID ) );
#endif
  Distortion dist;
  if( ( org.width == 1 ) )
  {
    dist = xGetSSE( dp );
  }
  else
  {
    const int base = (bitDepth > 10) ? 1 : 0;
    dist = m_afpDistortFunc[base][eDFunc + Log2(org.width)](dp);
  }
  if (isChroma(compId))
  {
    return ((Distortion) (m_distortionWeight[ compId ] * dist));
  }
  else
  {
    return dist;
  }
}

// ====================================================================================================================
// Distortion functions
// ====================================================================================================================

// --------------------------------------------------------------------------------------------------------------------
// SAD
// --------------------------------------------------------------------------------------------------------------------

Distortion RdCost::xGetSAD( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg           = rcDtParam.org.buf;
  const Pel* piCur           = rcDtParam.cur.buf;
  const int  iCols           = rcDtParam.org.width;
        int  iRows           = rcDtParam.org.height;
  const int  iSubShift       = rcDtParam.subShift;
  const int  iSubStep        = ( 1 << iSubShift );
  const int  iStrideCur      = rcDtParam.cur.stride * iSubStep;
  const int  iStrideOrg      = rcDtParam.org.stride * iSubStep;
  const uint32_t distortionShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows -= iSubStep )
  {
    for (int n = 0; n < iCols; n++ )
    {
      uiSum += abs( piOrg[n] - piCur[n] );
    }
    if (rcDtParam.maximumDistortionForEarlyExit < ( uiSum >> distortionShift ))
    {
      return ( uiSum >> distortionShift );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return ( uiSum >> distortionShift );
}

Distortion RdCost::xGetSAD4( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iSubShift     = rcDtParam.subShift;
  int  iSubStep      = ( 1 << iSubShift );
  int  iStrideCur    = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg    = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows -= iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD8( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD16( const DistParam& rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows -= iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}


Distortion RdCost::xGetSAD128( const DistParam &rcDtParam )
{
  const Pel* piOrg  = rcDtParam.org.buf;
  const Pel* piCur  = rcDtParam.cur.buf;
  int  iRows        = rcDtParam.org.height;
  int  iCols        = rcDtParam.org.width;
  int  iSubShift    = rcDtParam.subShift;
  int  iSubStep     = ( 1 << iSubShift );
  int  iStrideCur   = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg   = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    for (int n = 0; n < iCols; n+=16 )
    {
      uiSum += abs( piOrg[n+ 0] - piCur[n+ 0] );
      uiSum += abs( piOrg[n+ 1] - piCur[n+ 1] );
      uiSum += abs( piOrg[n+ 2] - piCur[n+ 2] );
      uiSum += abs( piOrg[n+ 3] - piCur[n+ 3] );
      uiSum += abs( piOrg[n+ 4] - piCur[n+ 4] );
      uiSum += abs( piOrg[n+ 5] - piCur[n+ 5] );
      uiSum += abs( piOrg[n+ 6] - piCur[n+ 6] );
      uiSum += abs( piOrg[n+ 7] - piCur[n+ 7] );
      uiSum += abs( piOrg[n+ 8] - piCur[n+ 8] );
      uiSum += abs( piOrg[n+ 9] - piCur[n+ 9] );
      uiSum += abs( piOrg[n+10] - piCur[n+10] );
      uiSum += abs( piOrg[n+11] - piCur[n+11] );
      uiSum += abs( piOrg[n+12] - piCur[n+12] );
      uiSum += abs( piOrg[n+13] - piCur[n+13] );
      uiSum += abs( piOrg[n+14] - piCur[n+14] );
      uiSum += abs( piOrg[n+15] - piCur[n+15] );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}

Distortion RdCost::xGetSAD32( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}


Distortion RdCost::xGetSAD64( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iSubShift        = rcDtParam.subShift;
  int  iSubStep         = ( 1 << iSubShift );
  int  iStrideCur       = rcDtParam.cur.stride * iSubStep;
  int  iStrideOrg       = rcDtParam.org.stride * iSubStep;

  Distortion uiSum = 0;

  for( ; iRows != 0; iRows-=iSubStep )
  {
    uiSum += abs( piOrg[0] - piCur[0] );
    uiSum += abs( piOrg[1] - piCur[1] );
    uiSum += abs( piOrg[2] - piCur[2] );
    uiSum += abs( piOrg[3] - piCur[3] );
    uiSum += abs( piOrg[4] - piCur[4] );
    uiSum += abs( piOrg[5] - piCur[5] );
    uiSum += abs( piOrg[6] - piCur[6] );
    uiSum += abs( piOrg[7] - piCur[7] );
    uiSum += abs( piOrg[8] - piCur[8] );
    uiSum += abs( piOrg[9] - piCur[9] );
    uiSum += abs( piOrg[10] - piCur[10] );
    uiSum += abs( piOrg[11] - piCur[11] );
    uiSum += abs( piOrg[12] - piCur[12] );
    uiSum += abs( piOrg[13] - piCur[13] );
    uiSum += abs( piOrg[14] - piCur[14] );
    uiSum += abs( piOrg[15] - piCur[15] );
    uiSum += abs( piOrg[16] - piCur[16] );
    uiSum += abs( piOrg[17] - piCur[17] );
    uiSum += abs( piOrg[18] - piCur[18] );
    uiSum += abs( piOrg[19] - piCur[19] );
    uiSum += abs( piOrg[20] - piCur[20] );
    uiSum += abs( piOrg[21] - piCur[21] );
    uiSum += abs( piOrg[22] - piCur[22] );
    uiSum += abs( piOrg[23] - piCur[23] );
    uiSum += abs( piOrg[24] - piCur[24] );
    uiSum += abs( piOrg[25] - piCur[25] );
    uiSum += abs( piOrg[26] - piCur[26] );
    uiSum += abs( piOrg[27] - piCur[27] );
    uiSum += abs( piOrg[28] - piCur[28] );
    uiSum += abs( piOrg[29] - piCur[29] );
    uiSum += abs( piOrg[30] - piCur[30] );
    uiSum += abs( piOrg[31] - piCur[31] );
    uiSum += abs( piOrg[32] - piCur[32] );
    uiSum += abs( piOrg[33] - piCur[33] );
    uiSum += abs( piOrg[34] - piCur[34] );
    uiSum += abs( piOrg[35] - piCur[35] );
    uiSum += abs( piOrg[36] - piCur[36] );
    uiSum += abs( piOrg[37] - piCur[37] );
    uiSum += abs( piOrg[38] - piCur[38] );
    uiSum += abs( piOrg[39] - piCur[39] );
    uiSum += abs( piOrg[40] - piCur[40] );
    uiSum += abs( piOrg[41] - piCur[41] );
    uiSum += abs( piOrg[42] - piCur[42] );
    uiSum += abs( piOrg[43] - piCur[43] );
    uiSum += abs( piOrg[44] - piCur[44] );
    uiSum += abs( piOrg[45] - piCur[45] );
    uiSum += abs( piOrg[46] - piCur[46] );
    uiSum += abs( piOrg[47] - piCur[47] );
    uiSum += abs( piOrg[48] - piCur[48] );
    uiSum += abs( piOrg[49] - piCur[49] );
    uiSum += abs( piOrg[50] - piCur[50] );
    uiSum += abs( piOrg[51] - piCur[51] );
    uiSum += abs( piOrg[52] - piCur[52] );
    uiSum += abs( piOrg[53] - piCur[53] );
    uiSum += abs( piOrg[54] - piCur[54] );
    uiSum += abs( piOrg[55] - piCur[55] );
    uiSum += abs( piOrg[56] - piCur[56] );
    uiSum += abs( piOrg[57] - piCur[57] );
    uiSum += abs( piOrg[58] - piCur[58] );
    uiSum += abs( piOrg[59] - piCur[59] );
    uiSum += abs( piOrg[60] - piCur[60] );
    uiSum += abs( piOrg[61] - piCur[61] );
    uiSum += abs( piOrg[62] - piCur[62] );
    uiSum += abs( piOrg[63] - piCur[63] );

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  uiSum <<= iSubShift;
  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}


// --------------------------------------------------------------------------------------------------------------------
// SSE
// --------------------------------------------------------------------------------------------------------------------

Distortion RdCost::xGetSSE( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg      = rcDtParam.org.buf;
  const Pel* piCur      = rcDtParam.cur.buf;
  int  iRows            = rcDtParam.org.height;
  int  iCols            = rcDtParam.org.width;
  int  iStrideCur       = rcDtParam.cur.stride;
  int  iStrideOrg       = rcDtParam.org.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int iTemp;

  for( ; iRows != 0; iRows-- )
  {
    for (int n = 0; n < iCols; n++ )
    {
      iTemp = piOrg[n  ] - piCur[n  ];
      uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE4( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    CHECK( rcDtParam.org.width != 4, "Invalid size" );
    THROW(" no support");
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {

    iTemp = piOrg[0] - piCur[0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[1] - piCur[1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[2] - piCur[2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[3] - piCur[3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE8( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    CHECK( rcDtParam.org.width != 8, "Invalid size" );
    THROW(" no support");
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    iTemp = piOrg[0] - piCur[0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[1] - piCur[1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[2] - piCur[2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[3] - piCur[3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[4] - piCur[4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[5] - piCur[5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[6] - piCur[6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[7] - piCur[7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE16( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    CHECK( rcDtParam.org.width != 16, "Invalid size" );
    THROW(" no support");
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {

    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE128( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }
  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iCols         = rcDtParam.org.width;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    for (int n = 0; n < iCols; n+=16 )
    {

      iTemp = piOrg[n+ 0] - piCur[n+ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 1] - piCur[n+ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 2] - piCur[n+ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 3] - piCur[n+ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 4] - piCur[n+ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 5] - piCur[n+ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 6] - piCur[n+ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 7] - piCur[n+ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 8] - piCur[n+ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+ 9] - piCur[n+ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+10] - piCur[n+10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+11] - piCur[n+11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+12] - piCur[n+12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+13] - piCur[n+13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+14] - piCur[n+14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
      iTemp = piOrg[n+15] - piCur[n+15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE32( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {

    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[16] - piCur[16]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[17] - piCur[17]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[18] - piCur[18]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[19] - piCur[19]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[20] - piCur[20]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[21] - piCur[21]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[22] - piCur[22]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[23] - piCur[23]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[24] - piCur[24]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[25] - piCur[25]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[26] - piCur[26]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[27] - piCur[27]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[28] - piCur[28]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[29] - piCur[29]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[30] - piCur[30]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[31] - piCur[31]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

Distortion RdCost::xGetSSE64( const DistParam &rcDtParam )
{
  if ( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  const Pel* piOrg   = rcDtParam.org.buf;
  const Pel* piCur   = rcDtParam.cur.buf;
  int  iRows         = rcDtParam.org.height;
  int  iStrideOrg    = rcDtParam.org.stride;
  int  iStrideCur    = rcDtParam.cur.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1;

  Intermediate_Int  iTemp;

  for( ; iRows != 0; iRows-- )
  {
    iTemp = piOrg[ 0] - piCur[ 0]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 1] - piCur[ 1]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 2] - piCur[ 2]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 3] - piCur[ 3]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 4] - piCur[ 4]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 5] - piCur[ 5]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 6] - piCur[ 6]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 7] - piCur[ 7]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 8] - piCur[ 8]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[ 9] - piCur[ 9]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[10] - piCur[10]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[11] - piCur[11]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[12] - piCur[12]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[13] - piCur[13]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[14] - piCur[14]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[15] - piCur[15]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[16] - piCur[16]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[17] - piCur[17]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[18] - piCur[18]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[19] - piCur[19]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[20] - piCur[20]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[21] - piCur[21]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[22] - piCur[22]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[23] - piCur[23]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[24] - piCur[24]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[25] - piCur[25]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[26] - piCur[26]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[27] - piCur[27]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[28] - piCur[28]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[29] - piCur[29]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[30] - piCur[30]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[31] - piCur[31]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[32] - piCur[32]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[33] - piCur[33]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[34] - piCur[34]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[35] - piCur[35]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[36] - piCur[36]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[37] - piCur[37]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[38] - piCur[38]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[39] - piCur[39]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[40] - piCur[40]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[41] - piCur[41]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[42] - piCur[42]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[43] - piCur[43]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[44] - piCur[44]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[45] - piCur[45]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[46] - piCur[46]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[47] - piCur[47]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[48] - piCur[48]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[49] - piCur[49]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[50] - piCur[50]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[51] - piCur[51]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[52] - piCur[52]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[53] - piCur[53]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[54] - piCur[54]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[55] - piCur[55]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[56] - piCur[56]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[57] - piCur[57]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[58] - piCur[58]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[59] - piCur[59]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[60] - piCur[60]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[61] - piCur[61]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[62] - piCur[62]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);
    iTemp = piOrg[63] - piCur[63]; uiSum += Distortion(( iTemp * iTemp ) >> uiShift);

    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum );
}

// --------------------------------------------------------------------------------------------------------------------
// HADAMARD with step (used in fractional search)
// --------------------------------------------------------------------------------------------------------------------

Distortion RdCost::xCalcHADs2x2( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  Distortion satd = 0;
  TCoeff diff[4], m[4];

  diff[0] = piOrg[0             ] - piCur[0];
  diff[1] = piOrg[1             ] - piCur[1];
  diff[2] = piOrg[iStrideOrg    ] - piCur[0 + iStrideCur];
  diff[3] = piOrg[iStrideOrg + 1] - piCur[1 + iStrideCur];
  m[0] = diff[0] + diff[2];
  m[1] = diff[1] + diff[3];
  m[2] = diff[0] - diff[2];
  m[3] = diff[1] - diff[3];
  
  satd += abs(m[0] + m[1]) >> 2;
  satd += abs(m[0] - m[1]);
  satd += abs(m[2] + m[3]);
  satd += abs(m[2] - m[3]);

  return satd;
}

static Distortion xCalcHADs4x4( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int k;
  Distortion satd = 0;
  TCoeff diff[16], m[16], d[16];

  for( k = 0; k < 16; k+=4 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  /*===== hadamard transform =====*/
  m[ 0] = diff[ 0] + diff[12];
  m[ 1] = diff[ 1] + diff[13];
  m[ 2] = diff[ 2] + diff[14];
  m[ 3] = diff[ 3] + diff[15];
  m[ 4] = diff[ 4] + diff[ 8];
  m[ 5] = diff[ 5] + diff[ 9];
  m[ 6] = diff[ 6] + diff[10];
  m[ 7] = diff[ 7] + diff[11];
  m[ 8] = diff[ 4] - diff[ 8];
  m[ 9] = diff[ 5] - diff[ 9];
  m[10] = diff[ 6] - diff[10];
  m[11] = diff[ 7] - diff[11];
  m[12] = diff[ 0] - diff[12];
  m[13] = diff[ 1] - diff[13];
  m[14] = diff[ 2] - diff[14];
  m[15] = diff[ 3] - diff[15];

  d[ 0] = m[ 0] + m[ 4];
  d[ 1] = m[ 1] + m[ 5];
  d[ 2] = m[ 2] + m[ 6];
  d[ 3] = m[ 3] + m[ 7];
  d[ 4] = m[ 8] + m[12];
  d[ 5] = m[ 9] + m[13];
  d[ 6] = m[10] + m[14];
  d[ 7] = m[11] + m[15];
  d[ 8] = m[ 0] - m[ 4];
  d[ 9] = m[ 1] - m[ 5];
  d[10] = m[ 2] - m[ 6];
  d[11] = m[ 3] - m[ 7];
  d[12] = m[12] - m[ 8];
  d[13] = m[13] - m[ 9];
  d[14] = m[14] - m[10];
  d[15] = m[15] - m[11];

  m[ 0] = d[ 0] + d[ 3];
  m[ 1] = d[ 1] + d[ 2];
  m[ 2] = d[ 1] - d[ 2];
  m[ 3] = d[ 0] - d[ 3];
  m[ 4] = d[ 4] + d[ 7];
  m[ 5] = d[ 5] + d[ 6];
  m[ 6] = d[ 5] - d[ 6];
  m[ 7] = d[ 4] - d[ 7];
  m[ 8] = d[ 8] + d[11];
  m[ 9] = d[ 9] + d[10];
  m[10] = d[ 9] - d[10];
  m[11] = d[ 8] - d[11];
  m[12] = d[12] + d[15];
  m[13] = d[13] + d[14];
  m[14] = d[13] - d[14];
  m[15] = d[12] - d[15];

  d[ 0] = m[ 0] + m[ 1];
  d[ 1] = m[ 0] - m[ 1];
  d[ 2] = m[ 2] + m[ 3];
  d[ 3] = m[ 3] - m[ 2];
  d[ 4] = m[ 4] + m[ 5];
  d[ 5] = m[ 4] - m[ 5];
  d[ 6] = m[ 6] + m[ 7];
  d[ 7] = m[ 7] - m[ 6];
  d[ 8] = m[ 8] + m[ 9];
  d[ 9] = m[ 8] - m[ 9];
  d[10] = m[10] + m[11];
  d[11] = m[11] - m[10];
  d[12] = m[12] + m[13];
  d[13] = m[12] - m[13];
  d[14] = m[14] + m[15];
  d[15] = m[15] - m[14];

  for (k=0; k<16; ++k)
  {
    satd += abs(d[k]);
  }

  satd -= abs( d[0] );
  satd += abs( d[0] ) >> 2;
  satd = ((satd+1)>>1);

  return satd;
}

static Distortion xCalcHADs16x16_fast( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int k, i, j, jj;
  Distortion sad = 0;
  TCoeff diff[64], m1[8][8], m2[8][8], m3[8][8];

  for( k = 0; k < 64; k += 8 )
  {
    diff[k+0] = ( ( piOrg[ 0] + piOrg[ 0+1] + piOrg[ 0+iStrideOrg] + piOrg[ 0+1+iStrideOrg] + 2 ) >> 2 ) - ( ( piCur[ 0] + piCur[ 0+1] + piCur[ 0+iStrideCur] + piCur[ 0+1+iStrideCur] + 2 ) >> 2 );
    diff[k+1] = ( ( piOrg[ 2] + piOrg[ 2+1] + piOrg[ 2+iStrideOrg] + piOrg[ 2+1+iStrideOrg] + 2 ) >> 2 ) - ( ( piCur[ 2] + piCur[ 2+1] + piCur[ 2+iStrideCur] + piCur[ 2+1+iStrideCur] + 2 ) >> 2 );
    diff[k+2] = ( ( piOrg[ 4] + piOrg[ 4+1] + piOrg[ 4+iStrideOrg] + piOrg[ 4+1+iStrideOrg] + 2 ) >> 2 ) - ( ( piCur[ 4] + piCur[ 4+1] + piCur[ 4+iStrideCur] + piCur[ 4+1+iStrideCur] + 2 ) >> 2 );
    diff[k+3] = ( ( piOrg[ 6] + piOrg[ 6+1] + piOrg[ 6+iStrideOrg] + piOrg[ 6+1+iStrideOrg] + 2 ) >> 2 ) - ( ( piCur[ 6] + piCur[ 6+1] + piCur[ 6+iStrideCur] + piCur[ 6+1+iStrideCur] + 2 ) >> 2 );
    diff[k+4] = ( ( piOrg[ 8] + piOrg[ 8+1] + piOrg[ 8+iStrideOrg] + piOrg[ 8+1+iStrideOrg] + 2 ) >> 2 ) - ( ( piCur[ 8] + piCur[ 8+1] + piCur[ 8+iStrideCur] + piCur[ 8+1+iStrideCur] + 2 ) >> 2 );
    diff[k+5] = ( ( piOrg[10] + piOrg[10+1] + piOrg[10+iStrideOrg] + piOrg[10+1+iStrideOrg] + 2 ) >> 2 ) - ( ( piCur[10] + piCur[10+1] + piCur[10+iStrideCur] + piCur[10+1+iStrideCur] + 2 ) >> 2 );
    diff[k+6] = ( ( piOrg[12] + piOrg[12+1] + piOrg[12+iStrideOrg] + piOrg[12+1+iStrideOrg] + 2 ) >> 2 ) - ( ( piCur[12] + piCur[12+1] + piCur[12+iStrideCur] + piCur[12+1+iStrideCur] + 2 ) >> 2 );
    diff[k+7] = ( ( piOrg[14] + piOrg[14+1] + piOrg[14+iStrideOrg] + piOrg[14+1+iStrideOrg] + 2 ) >> 2 ) - ( ( piCur[14] + piCur[14+1] + piCur[14+iStrideCur] + piCur[14+1+iStrideCur] + 2 ) >> 2 );

    piCur += 2 * iStrideCur;
    piOrg += 2 * iStrideOrg;
  }

  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i=0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      sad += abs(m2[i][j]);
    }
  }
  
  sad -= abs( m2[0][0] );
  sad += abs( m2[0][0] ) >> 2;
  sad=((sad+2)>>2);

  return (sad << 2);
}

static Distortion xCalcHADs8x8( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int k, i, j, jj;
  Distortion sad = 0;
  TCoeff diff[64], m1[8][8], m2[8][8], m3[8][8];

  for( k = 0; k < 64; k += 8 )
  {
    diff[k+0] = piOrg[0] - piCur[0];
    diff[k+1] = piOrg[1] - piCur[1];
    diff[k+2] = piOrg[2] - piCur[2];
    diff[k+3] = piOrg[3] - piCur[3];
    diff[k+4] = piOrg[4] - piCur[4];
    diff[k+5] = piOrg[5] - piCur[5];
    diff[k+6] = piOrg[6] - piCur[6];
    diff[k+7] = piOrg[7] - piCur[7];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for (j=0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj  ] + diff[jj+4];
    m2[j][1] = diff[jj+1] + diff[jj+5];
    m2[j][2] = diff[jj+2] + diff[jj+6];
    m2[j][3] = diff[jj+3] + diff[jj+7];
    m2[j][4] = diff[jj  ] - diff[jj+4];
    m2[j][5] = diff[jj+1] - diff[jj+5];
    m2[j][6] = diff[jj+2] - diff[jj+6];
    m2[j][7] = diff[jj+3] - diff[jj+7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i=0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      sad += abs(m2[i][j]);
    }
  }
  
  sad -= abs( m2[0][0] );
  sad += abs( m2[0][0] ) >> 2;
  sad=((sad+2)>>2);

  return sad;
}

static Distortion xCalcHADs16x8( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{   //need to add SIMD implementation ,JCA
  int k, i, j, jj, sad = 0;
  int diff[128], m1[8][16], m2[8][16];
  for( k = 0; k < 128; k += 16 )
  {
    diff[k + 0] = piOrg[0] - piCur[0];
    diff[k + 1] = piOrg[1] - piCur[1];
    diff[k + 2] = piOrg[2] - piCur[2];
    diff[k + 3] = piOrg[3] - piCur[3];
    diff[k + 4] = piOrg[4] - piCur[4];
    diff[k + 5] = piOrg[5] - piCur[5];
    diff[k + 6] = piOrg[6] - piCur[6];
    diff[k + 7] = piOrg[7] - piCur[7];

    diff[k + 8] = piOrg[8] - piCur[8];
    diff[k + 9] = piOrg[9] - piCur[9];
    diff[k + 10] = piOrg[10] - piCur[10];
    diff[k + 11] = piOrg[11] - piCur[11];
    diff[k + 12] = piOrg[12] - piCur[12];
    diff[k + 13] = piOrg[13] - piCur[13];
    diff[k + 14] = piOrg[14] - piCur[14];
    diff[k + 15] = piOrg[15] - piCur[15];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for( j = 0; j < 8; j++ )
  {
    jj = j << 4;

    m2[j][0] = diff[jj    ] + diff[jj + 8];
    m2[j][1] = diff[jj + 1] + diff[jj + 9];
    m2[j][2] = diff[jj + 2] + diff[jj + 10];
    m2[j][3] = diff[jj + 3] + diff[jj + 11];
    m2[j][4] = diff[jj + 4] + diff[jj + 12];
    m2[j][5] = diff[jj + 5] + diff[jj + 13];
    m2[j][6] = diff[jj + 6] + diff[jj + 14];
    m2[j][7] = diff[jj + 7] + diff[jj + 15];
    m2[j][8] = diff[jj    ] - diff[jj + 8];
    m2[j][9] = diff[jj + 1] - diff[jj + 9];
    m2[j][10] = diff[jj + 2] - diff[jj + 10];
    m2[j][11] = diff[jj + 3] - diff[jj + 11];
    m2[j][12] = diff[jj + 4] - diff[jj + 12];
    m2[j][13] = diff[jj + 5] - diff[jj + 13];
    m2[j][14] = diff[jj + 6] - diff[jj + 14];
    m2[j][15] = diff[jj + 7] - diff[jj + 15];

    m1[j][0] = m2[j][0] + m2[j][4];
    m1[j][1] = m2[j][1] + m2[j][5];
    m1[j][2] = m2[j][2] + m2[j][6];
    m1[j][3] = m2[j][3] + m2[j][7];
    m1[j][4] = m2[j][0] - m2[j][4];
    m1[j][5] = m2[j][1] - m2[j][5];
    m1[j][6] = m2[j][2] - m2[j][6];
    m1[j][7] = m2[j][3] - m2[j][7];
    m1[j][8] = m2[j][8] + m2[j][12];
    m1[j][9] = m2[j][9] + m2[j][13];
    m1[j][10] = m2[j][10] + m2[j][14];
    m1[j][11] = m2[j][11] + m2[j][15];
    m1[j][12] = m2[j][8] - m2[j][12];
    m1[j][13] = m2[j][9] - m2[j][13];
    m1[j][14] = m2[j][10] - m2[j][14];
    m1[j][15] = m2[j][11] - m2[j][15];

    m2[j][0] = m1[j][0] + m1[j][2];
    m2[j][1] = m1[j][1] + m1[j][3];
    m2[j][2] = m1[j][0] - m1[j][2];
    m2[j][3] = m1[j][1] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][6];
    m2[j][5] = m1[j][5] + m1[j][7];
    m2[j][6] = m1[j][4] - m1[j][6];
    m2[j][7] = m1[j][5] - m1[j][7];
    m2[j][8] = m1[j][8] + m1[j][10];
    m2[j][9] = m1[j][9] + m1[j][11];
    m2[j][10] = m1[j][8] - m1[j][10];
    m2[j][11] = m1[j][9] - m1[j][11];
    m2[j][12] = m1[j][12] + m1[j][14];
    m2[j][13] = m1[j][13] + m1[j][15];
    m2[j][14] = m1[j][12] - m1[j][14];
    m2[j][15] = m1[j][13] - m1[j][15];

    m1[j][0] = m2[j][0] + m2[j][1];
    m1[j][1] = m2[j][0] - m2[j][1];
    m1[j][2] = m2[j][2] + m2[j][3];
    m1[j][3] = m2[j][2] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][5];
    m1[j][5] = m2[j][4] - m2[j][5];
    m1[j][6] = m2[j][6] + m2[j][7];
    m1[j][7] = m2[j][6] - m2[j][7];
    m1[j][8] = m2[j][8] + m2[j][9];
    m1[j][9] = m2[j][8] - m2[j][9];
    m1[j][10] = m2[j][10] + m2[j][11];
    m1[j][11] = m2[j][10] - m2[j][11];
    m1[j][12] = m2[j][12] + m2[j][13];
    m1[j][13] = m2[j][12] - m2[j][13];
    m1[j][14] = m2[j][14] + m2[j][15];
    m1[j][15] = m2[j][14] - m2[j][15];
  }

  //vertical
  for( i = 0; i < 16; i++ )
  {
    m2[0][i] = m1[0][i] + m1[4][i];
    m2[1][i] = m1[1][i] + m1[5][i];
    m2[2][i] = m1[2][i] + m1[6][i];
    m2[3][i] = m1[3][i] + m1[7][i];
    m2[4][i] = m1[0][i] - m1[4][i];
    m2[5][i] = m1[1][i] - m1[5][i];
    m2[6][i] = m1[2][i] - m1[6][i];
    m2[7][i] = m1[3][i] - m1[7][i];

    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];
    m1[4][i] = m2[4][i] + m2[6][i];
    m1[5][i] = m2[5][i] + m2[7][i];
    m1[6][i] = m2[4][i] - m2[6][i];
    m1[7][i] = m2[5][i] - m2[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for( i = 0; i < 8; i++ )
  {
    for( j = 0; j < 16; j++ )
    {
      sad += abs( m2[i][j] );
    }
  }
  
  sad -= abs( m2[0][0] );
  sad += abs( m2[0][0] ) >> 2;
  sad = ( int ) ( sad / sqrt( 16.0 * 8 ) * 2 );

  return sad;
}

static Distortion xCalcHADs8x16( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int k, i, j, jj, sad = 0;
  int diff[128], m1[16][8], m2[16][8];
  for( k = 0; k < 128; k += 8 )
  {
    diff[k + 0] = piOrg[0] - piCur[0];
    diff[k + 1] = piOrg[1] - piCur[1];
    diff[k + 2] = piOrg[2] - piCur[2];
    diff[k + 3] = piOrg[3] - piCur[3];
    diff[k + 4] = piOrg[4] - piCur[4];
    diff[k + 5] = piOrg[5] - piCur[5];
    diff[k + 6] = piOrg[6] - piCur[6];
    diff[k + 7] = piOrg[7] - piCur[7];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for( j = 0; j < 16; j++ )
  {
    jj = j << 3;

    m2[j][0] = diff[jj] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for( i = 0; i < 8; i++ )
  {
    m1[0][i] = m2[0][i] + m2[8][i];
    m1[1][i] = m2[1][i] + m2[9][i];
    m1[2][i] = m2[2][i] + m2[10][i];
    m1[3][i] = m2[3][i] + m2[11][i];
    m1[4][i] = m2[4][i] + m2[12][i];
    m1[5][i] = m2[5][i] + m2[13][i];
    m1[6][i] = m2[6][i] + m2[14][i];
    m1[7][i] = m2[7][i] + m2[15][i];
    m1[8][i] = m2[0][i] - m2[8][i];
    m1[9][i] = m2[1][i] - m2[9][i];
    m1[10][i] = m2[2][i] - m2[10][i];
    m1[11][i] = m2[3][i] - m2[11][i];
    m1[12][i] = m2[4][i] - m2[12][i];
    m1[13][i] = m2[5][i] - m2[13][i];
    m1[14][i] = m2[6][i] - m2[14][i];
    m1[15][i] = m2[7][i] - m2[15][i];

    m2[0][i] = m1[0][i] + m1[4][i];
    m2[1][i] = m1[1][i] + m1[5][i];
    m2[2][i] = m1[2][i] + m1[6][i];
    m2[3][i] = m1[3][i] + m1[7][i];
    m2[4][i] = m1[0][i] - m1[4][i];
    m2[5][i] = m1[1][i] - m1[5][i];
    m2[6][i] = m1[2][i] - m1[6][i];
    m2[7][i] = m1[3][i] - m1[7][i];
    m2[8][i] = m1[8][i] + m1[12][i];
    m2[9][i] = m1[9][i] + m1[13][i];
    m2[10][i] = m1[10][i] + m1[14][i];
    m2[11][i] = m1[11][i] + m1[15][i];
    m2[12][i] = m1[8][i] - m1[12][i];
    m2[13][i] = m1[9][i] - m1[13][i];
    m2[14][i] = m1[10][i] - m1[14][i];
    m2[15][i] = m1[11][i] - m1[15][i];

    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];
    m1[4][i] = m2[4][i] + m2[6][i];
    m1[5][i] = m2[5][i] + m2[7][i];
    m1[6][i] = m2[4][i] - m2[6][i];
    m1[7][i] = m2[5][i] - m2[7][i];
    m1[8][i] = m2[8][i] + m2[10][i];
    m1[9][i] = m2[9][i] + m2[11][i];
    m1[10][i] = m2[8][i] - m2[10][i];
    m1[11][i] = m2[9][i] - m2[11][i];
    m1[12][i] = m2[12][i] + m2[14][i];
    m1[13][i] = m2[13][i] + m2[15][i];
    m1[14][i] = m2[12][i] - m2[14][i];
    m1[15][i] = m2[13][i] - m2[15][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
    m2[8][i] = m1[8][i] + m1[9][i];
    m2[9][i] = m1[8][i] - m1[9][i];
    m2[10][i] = m1[10][i] + m1[11][i];
    m2[11][i] = m1[10][i] - m1[11][i];
    m2[12][i] = m1[12][i] + m1[13][i];
    m2[13][i] = m1[12][i] - m1[13][i];
    m2[14][i] = m1[14][i] + m1[15][i];
    m2[15][i] = m1[14][i] - m1[15][i];
  }

  for( i = 0; i < 16; i++ )
  {
    for( j = 0; j < 8; j++ )
    {
      sad += abs( m2[i][j] );
    }
  }
  
  sad -= abs( m2[0][0] );
  sad += abs( m2[0][0] ) >> 2;
  sad = ( int ) ( sad / sqrt( 16.0 * 8 ) * 2 );

  return sad;
}

static Distortion xCalcHADs4x8( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int k, i, j, jj, sad = 0;
  int diff[32], m1[8][4], m2[8][4];
  for( k = 0; k < 32; k += 4 )
  {
    diff[k + 0] = piOrg[0] - piCur[0];
    diff[k + 1] = piOrg[1] - piCur[1];
    diff[k + 2] = piOrg[2] - piCur[2];
    diff[k + 3] = piOrg[3] - piCur[3];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for( j = 0; j < 8; j++ )
  {
    jj = j << 2;
    m2[j][0] = diff[jj] + diff[jj + 2];
    m2[j][1] = diff[jj + 1] + diff[jj + 3];
    m2[j][2] = diff[jj] - diff[jj + 2];
    m2[j][3] = diff[jj + 1] - diff[jj + 3];

    m1[j][0] = m2[j][0] + m2[j][1];
    m1[j][1] = m2[j][0] - m2[j][1];
    m1[j][2] = m2[j][2] + m2[j][3];
    m1[j][3] = m2[j][2] - m2[j][3];
  }

  //vertical
  for( i = 0; i < 4; i++ )
  {
    m2[0][i] = m1[0][i] + m1[4][i];
    m2[1][i] = m1[1][i] + m1[5][i];
    m2[2][i] = m1[2][i] + m1[6][i];
    m2[3][i] = m1[3][i] + m1[7][i];
    m2[4][i] = m1[0][i] - m1[4][i];
    m2[5][i] = m1[1][i] - m1[5][i];
    m2[6][i] = m1[2][i] - m1[6][i];
    m2[7][i] = m1[3][i] - m1[7][i];

    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];
    m1[4][i] = m2[4][i] + m2[6][i];
    m1[5][i] = m2[5][i] + m2[7][i];
    m1[6][i] = m2[4][i] - m2[6][i];
    m1[7][i] = m2[5][i] - m2[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for( i = 0; i < 8; i++ )
  {
    for( j = 0; j < 4; j++ )
    {
      sad += abs( m2[i][j] );
    }
  }
  
  sad -= abs( m2[0][0] );
  sad += abs( m2[0][0] ) >> 2;
  sad = ( int ) ( sad / sqrt( 4.0 * 8 ) * 2 );

  return sad;
}

static Distortion xCalcHADs8x4( const Pel* piOrg, const Pel* piCur, int iStrideOrg, int iStrideCur )
{
  int k, i, j, jj, sad = 0;
  int diff[32], m1[4][8], m2[4][8];
  for( k = 0; k < 32; k += 8 )
  {
    diff[k + 0] = piOrg[0] - piCur[0];
    diff[k + 1] = piOrg[1] - piCur[1];
    diff[k + 2] = piOrg[2] - piCur[2];
    diff[k + 3] = piOrg[3] - piCur[3];
    diff[k + 4] = piOrg[4] - piCur[4];
    diff[k + 5] = piOrg[5] - piCur[5];
    diff[k + 6] = piOrg[6] - piCur[6];
    diff[k + 7] = piOrg[7] - piCur[7];

    piCur += iStrideCur;
    piOrg += iStrideOrg;
  }

  //horizontal
  for( j = 0; j < 4; j++ )
  {
    jj = j << 3;

    m2[j][0] = diff[jj] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for( i = 0; i < 8; i++ )
  {
    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
  }

  for( i = 0; i < 4; i++ )
  {
    for( j = 0; j < 8; j++ )
    {
      sad += abs( m2[i][j] );
    }
  }
  
  sad -= abs( m2[0][0] );
  sad += abs( m2[0][0] ) >> 2;
  sad = ( int ) ( sad / sqrt( 4.0 * 8 ) * 2 );

  return sad;
}

Distortion RdCost::xGetHAD2SADs( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }

  Distortion distHad = xGetHADs<false>( rcDtParam );
  Distortion distSad = 0;
  {
    CHECKD( (rcDtParam.org.width != rcDtParam.org.stride) || (rcDtParam.cur.stride != rcDtParam.org.stride) , "this functions assumes compact, aligned buffering");

    const Pel* piOrg  = rcDtParam.org.buf;
    const Pel* piCur  = rcDtParam.cur.buf;
    int  iRows        = rcDtParam.org.height>>2;
    int  iCols        = rcDtParam.org.width<<2;

    Distortion uiSum = 0;

    for( int y = 0; y < iRows;  y++ )
    {
      for (int n = 0; n < iCols; n+=16 )
      {
        uiSum += abs( piOrg[n+ 0] - piCur[n+ 0] );
        uiSum += abs( piOrg[n+ 1] - piCur[n+ 1] );
        uiSum += abs( piOrg[n+ 2] - piCur[n+ 2] );
        uiSum += abs( piOrg[n+ 3] - piCur[n+ 3] );
        uiSum += abs( piOrg[n+ 4] - piCur[n+ 4] );
        uiSum += abs( piOrg[n+ 5] - piCur[n+ 5] );
        uiSum += abs( piOrg[n+ 6] - piCur[n+ 6] );
        uiSum += abs( piOrg[n+ 7] - piCur[n+ 7] );
        uiSum += abs( piOrg[n+ 8] - piCur[n+ 8] );
        uiSum += abs( piOrg[n+ 9] - piCur[n+ 9] );
        uiSum += abs( piOrg[n+10] - piCur[n+10] );
        uiSum += abs( piOrg[n+11] - piCur[n+11] );
        uiSum += abs( piOrg[n+12] - piCur[n+12] );
        uiSum += abs( piOrg[n+13] - piCur[n+13] );
        uiSum += abs( piOrg[n+14] - piCur[n+14] );
        uiSum += abs( piOrg[n+15] - piCur[n+15] );
      }
      piOrg += iCols;
      piCur += iCols;
    }

    distSad = (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
  }

  return std::min( distHad, 2*distSad);
}

template<bool fastHad>
Distortion RdCost::xGetHADs( const DistParam &rcDtParam )
{
  if( rcDtParam.applyWeight )
  {
    THROW(" no support");
  }
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iRows = rcDtParam.org.height;
  const int  iCols = rcDtParam.org.width;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;

  int  x = 0, y = 0;

  Distortion uiSum = 0;

  if( iCols > iRows && ( iRows & 7 ) == 0 && ( iCols & 15 ) == 0 )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHADs16x8( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += iStrideOrg * 8;
      piCur += iStrideCur * 8;
    }
  }
  else if( iCols < iRows && ( iCols & 7 ) == 0 && ( iRows & 15 ) == 0 )
  {
    for( y = 0; y < iRows; y += 16 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHADs8x16( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += iStrideOrg * 16;
      piCur += iStrideCur * 16;
    }
  }
  else if( iCols > iRows && ( iRows & 3 ) == 0 && ( iCols & 7 ) == 0 )
  {
    for( y = 0; y < iRows; y += 4 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHADs8x4( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += iStrideOrg * 4;
      piCur += iStrideCur * 4;
    }
  }
  else if( iCols < iRows && ( iCols & 3 ) == 0 && ( iRows & 7 ) == 0 )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHADs4x8( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += iStrideOrg * 8;
      piCur += iStrideCur * 8;
    }
  }
  else if( fastHad && ( ( iRows % 32 == 0 ) && ( iCols % 32 == 0 ) ) && iRows == iCols )
  {
    for( y = 0; y < iRows; y += 16 )
    {
      for( x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHADs16x16_fast( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 16 * iStrideOrg;
      piCur += 16 * iStrideCur;
    }
  }
  else if( ( iRows % 8 == 0 ) && ( iCols % 8 == 0 ) )
  {
    for( y = 0; y < iRows; y += 8 )
    {
      for( x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHADs8x8( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8*iStrideOrg;
      piCur += 8*iStrideCur;
    }
  }
  else if( ( iRows % 4 == 0 ) && ( iCols % 4 == 0 ) )
  {
    for( y = 0; y < iRows; y += 4 )
    {
      for( x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHADs4x4( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 4*iStrideOrg;
      piCur += 4*iStrideCur;
    }
  }
  else if( ( iRows % 2 == 0 ) && ( iCols % 2 == 0 ) )
  {
    for( y = 0; y < iRows; y += 2 )
    {
      for( x = 0; x < iCols; x += 2 )
      {
        uiSum += xCalcHADs2x2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 2*iStrideOrg;
      piCur += 2*iStrideCur;
    }
  }
  else
  {
    THROW( "Invalid size" );
  }

  return (uiSum >> DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth));
}


void RdCost::setMotionLambda( double motionLambda )
{
  m_motionLambda = motionLambda;
  for( uint32_t bits = 0; bits < MOTION_COST_TABLE_SIZE; bits++ )
  {
    m_motionCost[bits] = Distortion( motionLambda * bits );
  }
}

void RdCost::saveUnadjustedLambda()
{
  m_dLambda_unadjusted = m_dLambda;
  m_DistScaleUnadjusted = m_DistScale;
}


inline Distortion getWeightedMSE(const Pel org, const Pel cur, const int64_t fixedPTweight, unsigned uiShift)
{
  const Intermediate_Int iTemp = org - cur;
  return Intermediate_Int((fixedPTweight*(iTemp*iTemp) + (1 << 15)) >> uiShift);
}

static Distortion fixWeightedSSE_Core( const DistParam& rcDtParam, uint32_t fixedPTweight )
{
        int  iRows = rcDtParam.org.height;
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  const int  iCols = rcDtParam.org.width;
  const int  iStrideCur = rcDtParam.cur.stride;
  const int  iStrideOrg = rcDtParam.org.stride;

  Distortion uiSum   = 0;
  uint32_t uiShift = 16 + (DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth) << 1);

  // cf, column factor, offset of the second column, to be set to '0' for width of '1'
  const int cf =  1 - ( iCols & 1 );
  CHECK( ( iCols & 1 ) && iCols != 1, "Width can only be even or equal to '1'!" );
  
  for( ; iRows != 0; iRows-- )
  {
    for (int n = 0; n < iCols; n+=2 )
    {
      uiSum += getWeightedMSE( piOrg[n   ], piCur[n   ], fixedPTweight, uiShift );
      uiSum += getWeightedMSE( piOrg[n+cf], piCur[n+cf], fixedPTweight, uiShift );
    }
    piOrg += iStrideOrg;
    piCur += iStrideCur;
  }

  return ( uiSum >> ( 1 - cf ) );
}

void RdCost::xGetSAD8X5(const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos) {
  DistParam rcDtParamTmp0 = rcDtParam;

  DistParam rcDtParamTmp1 = rcDtParam;
  rcDtParamTmp1.org.buf += 1;
  rcDtParamTmp1.cur.buf -= 1;

  DistParam rcDtParamTmp2 = rcDtParam;
  rcDtParamTmp2.org.buf += 2;
  rcDtParamTmp2.cur.buf -= 2;

  DistParam rcDtParamTmp3 = rcDtParam;
  rcDtParamTmp3.org.buf += 3;
  rcDtParamTmp3.cur.buf -= 3;

  DistParam rcDtParamTmp4 = rcDtParam;
  rcDtParamTmp4.org.buf += 4;
  rcDtParamTmp4.cur.buf -= 4;
  
  cost[0] = (RdCost::xGetSAD8(rcDtParamTmp0)) >> 1;
  cost[1] = (RdCost::xGetSAD8(rcDtParamTmp1)) >> 1;
  if (isCalCentrePos) cost[2] = (RdCost::xGetSAD8(rcDtParamTmp2)) >> 1;
  cost[3] = (RdCost::xGetSAD8(rcDtParamTmp3)) >> 1;
  cost[4] = (RdCost::xGetSAD8(rcDtParamTmp4)) >> 1;
}

void RdCost::xGetSAD16X5(const DistParam& rcDtParam, Distortion* cost, bool isCalCentrePos) {
  DistParam rcDtParamTmp0 = rcDtParam;

  DistParam rcDtParamTmp1 = rcDtParam;
  rcDtParamTmp1.org.buf += 1;
  rcDtParamTmp1.cur.buf -= 1;

  DistParam rcDtParamTmp2 = rcDtParam;
  rcDtParamTmp2.org.buf += 2;
  rcDtParamTmp2.cur.buf -= 2;

  DistParam rcDtParamTmp3 = rcDtParam;
  rcDtParamTmp3.org.buf += 3;
  rcDtParamTmp3.cur.buf -= 3;

  DistParam rcDtParamTmp4 = rcDtParam;
  rcDtParamTmp4.org.buf += 4;
  rcDtParamTmp4.cur.buf -= 4;
  
  cost[0] = (RdCost::xGetSAD16(rcDtParamTmp0)) >> 1;
  cost[1] = (RdCost::xGetSAD16(rcDtParamTmp1)) >> 1;
  if (isCalCentrePos) cost[2] = (RdCost::xGetSAD16(rcDtParamTmp2)) >> 1;
  cost[3] = (RdCost::xGetSAD16(rcDtParamTmp3)) >> 1;
  cost[4] = (RdCost::xGetSAD16(rcDtParamTmp4)) >> 1;
}

void RdCost::setDistParamGeo(DistParam &rcDP, const CPelBuf &org, const Pel *piRefY, int iRefStride, const Pel *mask,
                          int iMaskStride, int stepX, int iMaskStride2, int bitDepth, ComponentID compID)
{
  rcDP.bitDepth = bitDepth;
  rcDP.compID   = compID;

  // set Original & Curr Pointer / Stride
  rcDP.org        = org;
  rcDP.cur.buf    = piRefY;
  rcDP.cur.stride = iRefStride;

  // set Mask
  rcDP.mask        = mask;
  rcDP.maskStride  = iMaskStride;
  rcDP.stepX       = stepX;
  rcDP.maskStride2 = iMaskStride2;

  // set Block Width / Height
  rcDP.cur.width                     = org.width;
  rcDP.cur.height                    = org.height;
  rcDP.maximumDistortionForEarlyExit = MAX_DISTORTION;

  // set Cost function for motion estimation with Mask
  rcDP.distFunc = m_afpDistortFunc[0][DF_SAD_WITH_MASK];
}

Distortion RdCost::xGetSADwMask(const DistParam &rcDtParam)
{
  const Pel *    org             = rcDtParam.org.buf;
  const Pel *    cur             = rcDtParam.cur.buf;
  const Pel *    mask            = rcDtParam.mask;
  const int      cols            = rcDtParam.org.width;
  int            rows            = rcDtParam.org.height;
  const int      subShift        = rcDtParam.subShift;
  const int      subStep         = (1 << subShift);
  const int      strideCur       = rcDtParam.cur.stride * subStep;
  const int      strideOrg       = rcDtParam.org.stride * subStep;
  const int      strideMask      = rcDtParam.maskStride * subStep;
  const int      stepX           = rcDtParam.stepX;
  const int      strideMask2     = rcDtParam.maskStride2;
  const uint32_t distortionShift = DISTORTION_PRECISION_ADJUSTMENT(rcDtParam.bitDepth);

  Distortion sum = 0;
  for (; rows != 0; rows -= subStep)
  {
    for (int n = 0; n < cols; n++)
    {
      sum += abs(org[n] - cur[n]) * *mask;
      mask += stepX;
    }
    org += strideOrg;
    cur += strideCur;
    mask += strideMask;
    mask += strideMask2;
  }
  sum <<= subShift;
  return (sum >> distortionShift);
}

Distortion RdCost::getBvCostMultiplePredsIBC(int x, int y, bool useIMV)
{
  return getBvCostIBC( getBitsMultiplePredsIBC( x, y, useIMV ) );
}

static inline unsigned getIComponentBitsIBC( int val )
{
  if( !val ) return 1;

  const unsigned int l2 = floorLog2( (val <= 0) ? (-val << 1) + 1 : (val << 1) );

  return (l2 << 1) + 1;
}

// the bits of the block vector ( x, y ) with the cheaper of the two predictors, as estimated by the IBC search
static unsigned bvBitsMultiplePreds( const Mv* predictors, const int x, const int y, const bool useIMV )
{
  int rmvH[2];
  int rmvV[2];
  rmvH[0] = x - predictors[0].hor;
  rmvH[1] = x - predictors[1].hor;

  rmvV[0] = y - predictors[0].ver;
  rmvV[1] = y - predictors[1].ver;
  int absCand[2];
  absCand[0] = abs(rmvH[0]) + abs(rmvV[0]);
  absCand[1] = abs(rmvH[1]) + abs(rmvV[1]);

  if (useIMV && x % 4 == 0 && y % 4 == 0)
  {
    int rmvHQP[2];
    int rmvVQP[2];

    int imvShift = 2;
    int offset = 1 << (imvShift - 1);

    rmvHQP[0] = (x >> 2) - ((predictors[0].hor + offset) >> 2);
    rmvHQP[1] = (x >> 2) - ((predictors[1].hor + offset) >> 2);
    rmvVQP[0] = (y >> 2) - ((predictors[0].ver + offset) >> 2);
    rmvVQP[1] = (y >> 2) - ((predictors[1].ver + offset) >> 2);

    int absCandQP[2];
    absCandQP[0] = abs(rmvHQP[0]) + abs(rmvVQP[0]);
    absCandQP[1] = abs(rmvHQP[1]) + abs(rmvVQP[1]);
    unsigned int candBits0QP, candBits1QP;
    if (absCand[0] < absCand[1])
    {
      unsigned int candBits0 = getIComponentBitsIBC(rmvH[0]) + getIComponentBitsIBC(rmvV[0]);
      if (absCandQP[0] < absCandQP[1])
      {
        candBits0QP = getIComponentBitsIBC(rmvHQP[0]) + getIComponentBitsIBC(rmvVQP[0]);
        return candBits0QP < candBits0 ? candBits0QP : candBits0;
      }
      else
      {
        candBits1QP = getIComponentBitsIBC(rmvHQP[1]) + getIComponentBitsIBC(rmvVQP[1]);
        return candBits1QP < candBits0 ? candBits1QP : candBits0;
      }
    }
    else
    {
      unsigned int candBits1 = getIComponentBitsIBC(rmvH[1]) + getIComponentBitsIBC(rmvV[1]);
      if (absCandQP[0] < absCandQP[1])
      {
        candBits0QP = getIComponentBitsIBC(rmvHQP[0]) + getIComponentBitsIBC(rmvVQP[0]);
        return candBits0QP < candBits1 ? candBits0QP : candBits1;
      }
      else
      {
        candBits1QP = getIComponentBitsIBC(rmvHQP[1]) + getIComponentBitsIBC(rmvVQP[1]);
        return candBits1QP < candBits1 ? candBits1QP : candBits1;
      }
    }
  }
  else
  {
    if (absCand[0] < absCand[1])
    {
      return getIComponentBitsIBC(rmvH[0]) + getIComponentBitsIBC(rmvV[0]);
    }
    else
    {
      return getIComponentBitsIBC(rmvH[1]) + getIComponentBitsIBC(rmvV[1]);
    }
  }
}

unsigned int RdCost::getBitsMultiplePredsIBC(int x, int y, bool useIMV)
{
  return bvBitsMultiplePreds( m_bvPredictors, x, y, useIMV );
}

static void bvBitsLineCore( const Mv* predictors, const int x0, const int xStep, const int count, const int y, const bool useIMV, unsigned* bits )
{
  for( int i = 0; i < count; i++ )
  {
    bits[i] = bvBitsMultiplePreds( predictors, x0 + i * xStep, y, useIMV );
  }
}

void RdCost::initLumaLevelToWeightTable( int lumaBD )
{
  const int lutSize = 1 << lumaBD;
  if (m_lumaLevelToWeightPLUT.empty())
    m_lumaLevelToWeightPLUT.resize(lutSize, 1.0);

  for (int i = 0; i < lutSize; i++)
  {
    double x = lumaBD < 10 ? i << (10 - lumaBD) : lumaBD > 10 ? i >> (lumaBD - 10) : i;
    double y;
    y = 0.015*x - 1.5 - 6;
    y = y < -3 ? -3 : (y > 6 ? 6 : y);
    m_lumaLevelToWeightPLUT[i] = (uint32_t)(pow(2.0, y / 3.0) * (double)(1 << 16));
  }
}

} // namespace vvenc

//! \}


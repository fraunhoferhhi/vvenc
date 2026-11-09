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
/**
 * \file AdaptiveLoopFilter_neon.cpp
 * \brief Neon implementation of ALF for AArch64.
 */
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include <arm_neon.h>

#include "CommonLib/AdaptiveLoopFilter.h"
#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "permute_neon.h"
#include "sum_neon.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_ALF
namespace vvenc
{

// ALF classification of a block, as AdaptiveLoopFilter::deriveClassificationBlk and the x86 kernel
// simdDeriveClassificationBlk, whose data flow this follows. The first pass forms, for every second row and column,
// the vertical, horizontal and two diagonal Laplacians | 2 c - a - b | (at most 8190 for 12-bit samples) and sums them
// per pair of rows and group of 4 columns in 16 bit (at most 8 * 8190, unsigned). The second pass adds those of each
// 4x4 block in 32 bit and derives its activity class, direction and transpose index with the C code's integer
// arithmetic: the products d1 * hv0 and hv1 * d0 are compared as uint32, as there.
void DeriveClassificationBlk_neon( AlfClassifier* classifier, const CPelBuf& srcLuma, const Area& blkDst,
                                   const Area& blk, const int shift, const int vbCTUHeight, int vbPos )
{
  CHECK( ( blk.height & 7 ) != 0, "Block height must be a multiple of 8" );
  CHECK( ( blk.width & 7 ) != 0, "Block width must be a multiple of 8" );
  CHECK( ( vbCTUHeight & ( vbCTUHeight - 1 ) ) != 0, "vbCTUHeight must be a power of 2" );

  const ptrdiff_t stride    = srcLuma.stride;
  const int       heightExt = blk.height + 4;
  const int       widthExt  = blk.width + 4;

  // per pair of rows and group of 4 columns: the sums of the four Laplacians, in the order V, H, D0, D1
  uint16_t colSums[( AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 4 ) >> 1]
                  [AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 8];

  static const uint16_t evenLanes16[8] = { 0xFFFF, 0, 0xFFFF, 0, 0xFFFF, 0, 0xFFFF, 0 };
  const uint16x8_t      evenLanes      = vld1q_u16( evenLanes16 );

  for( int i = 0; i < heightExt; i += 2 )
  {
    const Pel* row0 = srcLuma.buf + ( i + blk.pos().y - 3 ) * stride + blk.pos().x - 3;
    const Pel* row1 = row0 + stride;
    const Pel* row2 = row0 + 2 * stride;
    const Pel* row3 = row0 + 3 * stride;

    // the rows beyond the virtual boundary are replaced by the nearest row before it
    const int pos      = blkDst.pos().y - 2 + i;
    const int posInCTU = pos & ( vbCTUHeight - 1 );
    if( pos > 0 && posInCTU == vbPos - 2 )
    {
      row3 = row2;
    }
    else if( pos > 0 && posInCTU == vbPos )
    {
      row0 = row1;
    }

    int16x8_t prev = vdupq_n_s16( 0 );
    for( int j = 0; j < widthExt; j += 8 )
    {
      const int16x8_t x0 = vld1q_s16( row0 + j );
      const int16x8_t x1 = vld1q_s16( row1 + j );
      const int16x8_t x2 = vld1q_s16( row2 + j );
      const int16x8_t x3 = vld1q_s16( row3 + j );
      const int16x8_t x4 = vld1q_s16( row0 + j + 2 );
      const int16x8_t x5 = vld1q_s16( row1 + j + 2 );
      const int16x8_t x6 = vld1q_s16( row2 + j + 2 );
      const int16x8_t x7 = vld1q_s16( row3 + j + 2 );

      // even lanes: the Laplacians at the sample of row 1, odd lanes: those at the sample of row 2 one column on
      const int16x8_t nw     = vbslq_s16( evenLanes, x0, x1 );
      const int16x8_t n      = vbslq_s16( evenLanes, x5, x0 );
      const int16x8_t ne     = vbslq_s16( evenLanes, x4, x5 );
      const int16x8_t w      = vbslq_s16( evenLanes, x1, x2 );
      const int16x8_t e      = vbslq_s16( evenLanes, x5, x6 );
      const int16x8_t sw     = vbslq_s16( evenLanes, x2, x3 );
      const int16x8_t s      = vbslq_s16( evenLanes, x7, x2 );
      const int16x8_t se     = vbslq_s16( evenLanes, x6, x7 );
      const int16x8_t centre = vbslq_s16( evenLanes, x6, x1 );
      const int16x8_t c      = vaddq_s16( centre, centre );
      const int16x8_t d      = vrev32q_s16( c );   // the lanes of each pair swapped

      const int16x8_t ver   = vabdq_s16( c, vaddq_s16( n, s ) );
      const int16x8_t hor   = vabdq_s16( d, vaddq_s16( w, e ) );
      const int16x8_t diag0 = vabdq_s16( d, vaddq_s16( nw, se ) );
      const int16x8_t diag1 = vabdq_s16( d, vaddq_s16( ne, sw ) );

      // per group of two columns V, H, D0, D1 (for columns j ... j + 3 and j + 4 ... j + 7), then the groups of 4
      // columns, the first one with the last group of the vector before
      const int16x8_t verHor   = pairwise_add_s16x8( ver, hor );
      const int16x8_t diags    = pairwise_add_s16x8( diag0, diag1 );
      const int16x8_t all      = pairwise_add_s16x8( verHor, diags );
      const int16x8_t withPrev = vbslq_s16( evenLanes, all, prev );
      vst1q_u16( &colSums[i >> 1][j], vreinterpretq_u16_s16( pairwise_add_s16x8( withPrev, all ) ) );
      prev = all;
    }
  }

  const int32x4_t  negShift = vdupq_n_s32( -shift );
  const uint16x8_t zero     = vdupq_n_u16( 0 );
  const uint32x4_t five     = vdupq_n_u32( 5 );

  for( int i = 0; i < ( blk.height >> 1 ); i += 4 )
  {
    // two rows of 4x4 blocks; at the virtual boundary their outer pair of rows is left out and the activity scaled
    const int        z      = ( 2 * i + blkDst.pos().y ) & ( vbCTUHeight - 1 );
    const int        z2     = ( 2 * i + 4 + blkDst.pos().y ) & ( vbCTUHeight - 1 );
    const uint32_t   scale1 = ( z == vbPos - 4 || z == vbPos ) ? 96 : 64;
    const uint32_t   scale2 = ( z2 == vbPos - 4 || z2 == vbPos ) ? 96 : 64;
    const uint32x4_t scales = vcombine_u32( vdup_n_u32( scale1 ), vdup_n_u32( scale2 ) );

    AlfClassifier* classes1 = &classifier[( i >> 1 ) * ( MAX_CU_SIZE / 4 )];
    AlfClassifier* classes2 = &classifier[( ( i >> 1 ) + 1 ) * ( MAX_CU_SIZE / 4 )];

    for( int j = 0; j < blk.width; j += 8 )
    {
      const uint16x8_t r0 = z == vbPos ? zero : vld1q_u16( &colSums[i][j + 4] );
      const uint16x8_t r1 = vld1q_u16( &colSums[i + 1][j + 4] );
      const uint16x8_t r2 = vld1q_u16( &colSums[i + 2][j + 4] );
      const uint16x8_t r3 = z == vbPos - 4 ? zero : vld1q_u16( &colSums[i + 3][j + 4] );
      const uint16x8_t r4 = z2 == vbPos ? zero : vld1q_u16( &colSums[i + 2][j + 4] );
      const uint16x8_t r5 = vld1q_u16( &colSums[i + 3][j + 4] );
      const uint16x8_t r6 = vld1q_u16( &colSums[i + 4][j + 4] );
      const uint16x8_t r7 = z2 == vbPos - 4 ? zero : vld1q_u16( &colSums[i + 5][j + 4] );

      // V, H, D0, D1 of the left and right block of each block row
      const uint32x4_t left1  = vaddq_u32( vaddl_u16( vget_low_u16( r0 ), vget_low_u16( r1 ) ),
                                           vaddl_u16( vget_low_u16( r2 ), vget_low_u16( r3 ) ) );
      const uint32x4_t right1 = vaddq_u32( vaddl_u16( vget_high_u16( r0 ), vget_high_u16( r1 ) ),
                                           vaddl_u16( vget_high_u16( r2 ), vget_high_u16( r3 ) ) );
      const uint32x4_t left2  = vaddq_u32( vaddl_u16( vget_low_u16( r4 ), vget_low_u16( r5 ) ),
                                           vaddl_u16( vget_low_u16( r6 ), vget_low_u16( r7 ) ) );
      const uint32x4_t right2 = vaddq_u32( vaddl_u16( vget_high_u16( r4 ), vget_high_u16( r5 ) ),
                                           vaddl_u16( vget_high_u16( r6 ), vget_high_u16( r7 ) ) );

      // transposed: one direction per vector, lanes left1, right1, left2, right2
      const uint32x4x2_t upper = vtrnq_u32( left1, right1 );
      const uint32x4x2_t lower = vtrnq_u32( left2, right2 );
      const uint32x4_t sumV  = vcombine_u32( vget_low_u32( upper.val[0] ), vget_low_u32( lower.val[0] ) );
      const uint32x4_t sumH  = vcombine_u32( vget_low_u32( upper.val[1] ), vget_low_u32( lower.val[1] ) );
      const uint32x4_t sumD0 = vcombine_u32( vget_high_u32( upper.val[0] ), vget_high_u32( lower.val[0] ) );
      const uint32x4_t sumD1 = vcombine_u32( vget_high_u32( upper.val[1] ), vget_high_u32( lower.val[1] ) );

      // activity = min( 15, ( sumV + sumH ) * scale >> shift ), class th[activity] = ( activity >= 1 ) +
      // ( activity >= 2 ) + ( activity >= 7 ) + ( activity >= 15 ), adding the all-ones compare masks as -1
      const uint32x4_t activity =
        vminq_u32( vshlq_u32( vmulq_u32( vaddq_u32( sumV, sumH ), scales ), negShift ), vdupq_n_u32( 15 ) );
      uint32x4_t classIdx = vdupq_n_u32( 0 );
      classIdx            = vsubq_u32( classIdx, vcgeq_u32( activity, vdupq_n_u32( 1 ) ) );
      classIdx            = vsubq_u32( classIdx, vcgeq_u32( activity, vdupq_n_u32( 2 ) ) );
      classIdx            = vsubq_u32( classIdx, vcgeq_u32( activity, vdupq_n_u32( 7 ) ) );
      classIdx            = vsubq_u32( classIdx, vcgeq_u32( activity, vdupq_n_u32( 15 ) ) );

      const uint32x4_t dirTempHV = vcgtq_u32( sumV, sumH );   // all ones where dirTempHV is 0
      const uint32x4_t hv1       = vmaxq_u32( sumV, sumH );
      const uint32x4_t hv0       = vminq_u32( sumV, sumH );
      const uint32x4_t dirTempD  = vcgtq_u32( sumD0, sumD1 );   // all ones where dirTempD is 0
      const uint32x4_t d1        = vmaxq_u32( sumD0, sumD1 );
      const uint32x4_t d0        = vminq_u32( sumD0, sumD1 );

      const uint32x4_t diagonal = vcgtq_u32( vmulq_u32( d1, hv0 ), vmulq_u32( hv1, d0 ) );   // dirIdx 0
      const uint32x4_t hvd1     = vbslq_u32( diagonal, d1, hv1 );
      const uint32x4_t hvd0     = vbslq_u32( diagonal, d0, hv0 );

      // + 5 where hvd1 > 2 hvd0, + 5 more where 2 hvd1 > 9 hvd0, and for HV + 10 more where hvd1 > 2 hvd0
      const uint32x4_t strength1 = vcgtq_u32( hvd1, vaddq_u32( hvd0, hvd0 ) );
      const uint32x4_t strength2 = vcgtq_u32( vaddq_u32( hvd1, hvd1 ), vaddq_u32( hvd0, vshlq_n_u32( hvd0, 3 ) ) );
      const uint32x4_t offset    = vandq_u32( strength1, five );
      const uint32x4_t offsetHV  = vbicq_u32( offset, diagonal );
      classIdx                   = vaddq_u32( classIdx, offset );
      classIdx                   = vaddq_u32( classIdx, vandq_u32( strength2, five ) );
      classIdx                   = vaddq_u32( classIdx, vaddq_u32( offsetHV, offsetHV ) );

      // transposeIdx = 2 * dirTempD + dirTempHV = 3 - 2 * ( dirTempD == 0 ) - ( dirTempHV == 0 )
      const uint32x4_t transposeIdx =
        vaddq_u32( vaddq_u32( vdupq_n_u32( 3 ), dirTempHV ), vaddq_u32( dirTempD, dirTempD ) );

      // AlfClassifier { uint8_t classIdx, transposeIdx }: the left and right block of each block row as 32 bit
      static_assert( sizeof( AlfClassifier ) == 2, "AlfClassifier must be 16 bits wide" );
      const uint16x4_t classes = vorr_u16( vmovn_u32( classIdx ), vshl_n_u16( vmovn_u32( transposeIdx ), 8 ) );
      vst1_lane_u32( reinterpret_cast<uint32_t*>( classes1 + ( j >> 2 ) ), vreinterpret_u32_u16( classes ), 0 );
      vst1_lane_u32( reinterpret_cast<uint32_t*>( classes2 + ( j >> 2 ) ), vreinterpret_u32_u16( classes ), 1 );
    }
  }
}

// CC-ALF of 4 chroma samples of a 4:2:0 row, as AdaptiveLoopFilter::filterBlkCcAlf. The 7 luma differences to the
// sample at the chroma position come from the even / odd lanes (vuzp) of 8-sample loads of the luma rows; they are
// weighted by the coefficients in 32 bit and rounded as there, ( sum + 64 ) >> 7, by vrshrn: for 12-bit samples and
// coefficients up to 64 the result fits 16 bits. The clips to [0, max] add the signed delta to unsigned values with
// saturation (vsqadd) and take the minimum with max.
static inline void filterCcAlfRow4_neon( const Pel* srcCross, Pel* srcSelf, const ptrdiff_t offset1,
                                         const ptrdiff_t offset2, const ptrdiff_t offset3, const int16x8_t coeff,
                                         const uint16x4_t offset, const uint16x4_t maxVal )
{
  static constexpr int scaleBits = 7;   // AdaptiveLoopFilter::m_scaleBits

  const int16x8_t above     = vld1q_s16( srcCross + offset2 );
  const int16x8_t left      = vld1q_s16( srcCross - 1 );
  const int16x8_t centre    = vld1q_s16( srcCross );
  const int16x8_t belowLeft = vld1q_s16( srcCross + offset1 - 1 );
  const int16x8_t below     = vld1q_s16( srcCross + offset1 );
  const int16x8_t below2    = vld1q_s16( srcCross + offset3 );

  // low half: taps 0, 2, 3, 6; high half: taps 1, 5, 4
  const int16x8_t curr       = vuzpq_s16( centre, centre ).val[0];
  const int16x8_t taps01     = vuzpq_s16( above, left ).val[0];
  const int16x8_t taps25     = vuzpq_s16( centre, below ).val[1];
  const int16x8_t taps34     = vuzpq_s16( belowLeft, below ).val[0];
  const int16x8_t tap6       = vuzpq_s16( below2, below2 ).val[0];
  const int16x8_t diff01     = vsubq_s16( taps01, curr );
  const int16x8_t diff25     = vsubq_s16( taps25, curr );
  const int16x8_t diff34     = vsubq_s16( taps34, curr );
  const int16x8_t diff6      = vsubq_s16( tap6, curr );
  const int16x4_t coeffLow   = vget_low_s16( coeff );
  const int16x4_t coeffHigh  = vget_high_s16( coeff );

  int32x4_t sum = vmull_lane_s16( vget_low_s16( diff01 ), coeffLow, 0 );
  sum           = vmlal_lane_s16( sum, vget_high_s16( diff01 ), coeffLow, 1 );
  sum           = vmlal_lane_s16( sum, vget_low_s16( diff25 ), coeffLow, 2 );
  sum           = vmlal_lane_s16( sum, vget_low_s16( diff34 ), coeffLow, 3 );
  sum           = vmlal_lane_s16( sum, vget_high_s16( diff34 ), coeffHigh, 0 );
  sum           = vmlal_lane_s16( sum, vget_high_s16( diff25 ), coeffHigh, 1 );
  sum           = vmlal_lane_s16( sum, vget_low_s16( diff6 ), coeffHigh, 2 );

  // delta = clip( sum + offset, 0, max ) - offset
  const int16x4_t rounded = vrshrn_n_s32( sum, scaleBits );
  const int16x4_t delta =
    vreinterpret_s16_u16( vsub_u16( vmin_u16( vvenc_vsqadd_u16( offset, rounded ), maxVal ), offset ) );

  const uint16x4_t self = vreinterpret_u16_s16( vld1_s16( srcSelf ) );
  vst1_s16( srcSelf, vreinterpret_s16_u16( vmin_u16( vvenc_vsqadd_u16( self, delta ), maxVal ) ) );
}

// CC-ALF of a chroma block, as AdaptiveLoopFilter::filterBlkCcAlf, for 4:2:0; other formats take the C code. Rows next
// to the virtual boundary use the same reduced luma neighbourhood as there.
void FilterBlkCcAlf_neon( const PelBuf& dstBuf, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blkSrc,
                          const ComponentID compId, const int16_t* filterCoeff, const ClpRngs& clpRngs,
                          CodingStructure& cs, int vbCTUHeight, int vbPos )
{
  CHECKD( 1 << floorLog2( vbCTUHeight ) != vbCTUHeight, "Not a power of 2" );
  CHECKD( !isChroma( compId ), "Must be chroma" );

  const ChromaFormat chromaFormat = cs.slice->sps->chromaFormatIdc;
  const int          scaleX       = getComponentScaleX( compId, chromaFormat );
  const int          scaleY       = getComponentScaleY( compId, chromaFormat );
  if( scaleX != 1 || scaleY != 1 )
  {
    AdaptiveLoopFilter::filterBlkCcAlf<CC_ALF>( dstBuf, recSrc, blkDst, blkSrc, compId, filterCoeff, clpRngs, cs,
                                                vbCTUHeight, vbPos );
    return;
  }

  CHECKD( blkDst.y % 4 || blkDst.x % 4 || blkDst.height % 4 || blkDst.width % 4, "Wrong block in filtering" );
  CHECKD( blkDst.height <= 0 || blkDst.width <= 0, "Empty block in filtering" );

  const CPelBuf&  srcBuf       = recSrc.get( COMP_Y );
  const ptrdiff_t lumaStride   = srcBuf.stride;
  const Pel*      lumaPtr      = srcBuf.buf + blkSrc.y * lumaStride + blkSrc.x;
  const ptrdiff_t chromaStride = dstBuf.stride;
  Pel*            chromaPtr    = dstBuf.buf + blkDst.y * chromaStride + blkDst.x;

  const uint16x4_t offset = vdup_n_u16( static_cast<uint16_t>( 1 << ( clpRngs.bd - 1 ) ) );
  const uint16x4_t maxVal = vdup_n_u16( static_cast<uint16_t>( clpRngs.max() ) );
  const int16x8_t  coeff  = vld1q_s16( filterCoeff );

  for( int i = 0; i < blkDst.height; i += 4, chromaPtr += 4 * chromaStride, lumaPtr += 8 * lumaStride )
  {
    // distance of the group's first luma row from the virtual boundary; its 4 chroma rows cover 8 luma rows
    const int firstDistance = vbPos - ( ( ( blkDst.y + i ) << 1 ) & ( vbCTUHeight - 1 ) );
    for( int ii = 0; ii < 4; ii++ )
    {
      const int distance = firstDistance - 2 * ii;
      ptrdiff_t offset1  = lumaStride;
      ptrdiff_t offset2  = -lumaStride;
      ptrdiff_t offset3  = 2 * lumaStride;
      if( distance == 2 || distance == -1 )
      {
        offset3 = offset1;
      }
      else if( distance == 1 || distance == 0 )
      {
        offset1 = 0;
        offset2 = 0;
        offset3 = 0;
      }

      const Pel* srcCrossRow = lumaPtr + 2 * ii * lumaStride;
      Pel*       srcSelfRow  = chromaPtr + ii * chromaStride;
      for( int j = 0; j < blkDst.width; j += 4 )
      {
        filterCcAlfRow4_neon( srcCrossRow + 2 * j, srcSelfRow + j, offset1, offset2, offset3, coeff, offset, maxVal );
      }
    }
  }
}

constexpr int ALF_7x7_VB_FOLD_MIN_DIST = -2;
constexpr int ALF_7x7_VB_FOLD_MAX_DIST = 3;

// Lookup table for ALF coefficient shuffling.
static const uint8_t g_alf_shuffle_table[4][32] = {
    // transposeIdx = 0
    {  0,  1,  2,  3,  4,  5,  6,  7,   8,   9,  10,  11,  12,  13,  14,  15,
      16, 17, 18, 19, 20, 21, 22, 23, 255, 255, 255, 255, 255, 255, 255, 255 },
    // transposeIdx = 1
    { 18, 19, 8, 9, 20, 21, 16, 17,   2,   3,  10,  11,  22,  23,  14,  15,
       6,  7, 0, 1,  4,  5, 12, 13, 255, 255, 255, 255, 255, 255, 255, 255 },
    // transposeIdx = 2
    { 0, 1,  6,  7,  4,  5,  2,  3,  16,  17,  14,  15,  12,  13,  10,  11,
      8, 9, 18, 19, 20, 21, 22, 23, 255, 255, 255, 255, 255, 255, 255, 255 },
    // transposeIdx = 3
    { 18, 19, 16, 17, 20, 21,  8,  9,  6,    7,  14,  15,  22,  23,  10,  11,
       2,  3,  0,  1,  4,  5, 12, 13, 255, 255, 255, 255, 255, 255, 255, 255 } };

static inline void load_shuffle_coeffs_neon( const uint8_t* shuffleTable0, const uint8_t* shuffleTable1,
                                             const short* coeff0, const short* coeff1, int16x4_t coeff[6] )
{
  const uint8x16_t c0_04 = vreinterpretq_u8_s16( vld1q_s16( coeff0 ) );
  const uint8x16_t c1_04 = vreinterpretq_u8_s16( vld1q_s16( coeff1 ) );
  const uint8x16_t c0_8 = vreinterpretq_u8_s16( vcombine_s16( vld1_s16( coeff0 + 8 ), vdup_n_s16( 0 ) ) );
  const uint8x16_t c1_8 = vreinterpretq_u8_s16( vcombine_s16( vld1_s16( coeff1 + 8 ), vdup_n_s16( 0 ) ) );

  const uint8x16_t shuffleIndices0_04 = vld1q_u8( shuffleTable0 + 0 );
  const uint8x16_t shuffleIndices1_04 = vld1q_u8( shuffleTable1 + 0 );
  const uint8x16_t shuffleIndices0_8 = vld1q_u8( shuffleTable0 + 16 );
  const uint8x16_t shuffleIndices1_8 = vld1q_u8( shuffleTable1 + 16 );

  const uint8x16x2_t raw_coeff0 = { c0_04, c0_8 };
  const uint8x16x2_t raw_coeff1 = { c1_04, c1_8 };

  const int16x8_t coeff02 = vreinterpretq_s16_u8( vvenc_vqtbl2q_u8( raw_coeff0, shuffleIndices0_04 ) );
  const int16x8_t coeff13 = vreinterpretq_s16_u8( vvenc_vqtbl2q_u8( raw_coeff1, shuffleIndices1_04 ) );
  const int16x8_t coeff4 = vreinterpretq_s16_u8( vvenc_vqtbl2q_u8( raw_coeff0, shuffleIndices0_8 ) );
  const int16x8_t coeff5 = vreinterpretq_s16_u8( vvenc_vqtbl2q_u8( raw_coeff1, shuffleIndices1_8 ) );

  coeff[0] = vget_low_s16( coeff02 );
  coeff[2] = vget_high_s16( coeff02 );
  coeff[1] = vget_low_s16( coeff13 );
  coeff[3] = vget_high_s16( coeff13 );
  coeff[4] = vget_low_s16( coeff4 );
  coeff[5] = vget_low_s16( coeff5 );
}

static inline int16x8_t clip3_neon( const int16x8_t val, const int16x8_t minVal, const int16x8_t maxVal )
{
  return vmaxq_s16( vminq_s16( val, maxVal ), minVal );
}

static inline void processALF_CoeffPair_neon( const Pel* ptr0, const Pel* ptr1, const Pel* ptr2, const Pel* ptr3,
                                              const int16x8_t curr, int16x8_t& accA, int16x8_t& accB )
{
  const int16x8_t v0 = vld1q_s16( ptr0 );
  const int16x8_t v1 = vld1q_s16( ptr1 );
  const int16x8_t v2 = vld1q_s16( ptr2 );
  const int16x8_t v3 = vld1q_s16( ptr3 );

  const int16x8_t diff0 = vsubq_s16( v0, curr );
  const int16x8_t diff1 = vsubq_s16( v1, curr );
  const int16x8_t diff2 = vsubq_s16( v2, curr );
  const int16x8_t diff3 = vsubq_s16( v3, curr );

  accA = vaddq_s16( diff0, diff1 );
  accB = vaddq_s16( diff2, diff3 );
}

template<bool isFoldingRequired>
static inline int16x8_t processALF7x7Row_neon( const Pel* pImg0, const int distance, const ptrdiff_t srcStride,
                                               const int16x4_t coeff[6], const int clpRngMax )
{
  constexpr int SHIFT = AdaptiveLoopFilter::m_NUM_BITS - 1;

  const Pel* pImg1 = pImg0 + srcStride; // y+1
  const Pel* pImg2 = pImg0 - srcStride; // y-1
  const Pel* pImg3 = pImg1 + srcStride; // y+2
  const Pel* pImg4 = pImg2 - srcStride; // y-2
  const Pel* pImg5 = pImg3 + srcStride; // y+3
  const Pel* pImg6 = pImg4 - srcStride; // y-3

  if( isFoldingRequired )
  {
    // When the current line is near the VB (vbPos), some of rows access could point across the CTU boundary.
    // Distance = 0 or 1, reuse current row (pImg0).
    // Distance = 2 or -1, reuse previous folded row (pImg1/pImg2).
    // Distance = 3 or -2, reuse previous folded row (pImg3/pImg4).
    if( distance > 0 && distance <= ALF_7x7_VB_FOLD_MAX_DIST ) // Above.
    {
      pImg1 = distance == 1 ? pImg0 : pImg1;
      pImg3 = distance <= 2 ? pImg1 : pImg3;
      pImg5 = distance <= 3 ? pImg3 : pImg5;

      pImg2 = distance == 1 ? pImg0 : pImg2;
      pImg4 = distance <= 2 ? pImg2 : pImg4;
      pImg6 = distance <= 3 ? pImg4 : pImg6;
    }
    else if( distance <= 0 && distance >= ALF_7x7_VB_FOLD_MIN_DIST ) // Bottom.
    {
      pImg1 = distance == 0 ? pImg0 : pImg1;
      pImg3 = distance >= -1 ? pImg1 : pImg3;
      pImg5 = distance >= -2 ? pImg3 : pImg5;

      pImg2 = distance == 0 ? pImg0 : pImg2;
      pImg4 = distance >= -1 ? pImg2 : pImg4;
      pImg6 = distance >= -2 ? pImg4 : pImg6;
    }
  }

  const int16x8_t curr = vld1q_s16( pImg0 );

  int16x8_t a0, a1, a2, a3, a4, a5;
  int16x8_t b0, b1, b2, b3, b4, b5;
  processALF_CoeffPair_neon( pImg5 + 0, pImg6 + 0, pImg3 + 1, pImg4 - 1, curr, a0, b0 );
  processALF_CoeffPair_neon( pImg3 + 0, pImg4 + 0, pImg3 - 1, pImg4 + 1, curr, a1, b1 );
  processALF_CoeffPair_neon( pImg1 + 2, pImg2 - 2, pImg1 + 1, pImg2 - 1, curr, a2, b2 );
  processALF_CoeffPair_neon( pImg1 + 0, pImg2 + 0, pImg1 - 1, pImg2 + 1, curr, a3, b3 );
  processALF_CoeffPair_neon( pImg1 - 2, pImg2 + 2, pImg0 + 3, pImg0 - 3, curr, a4, b4 );
  processALF_CoeffPair_neon( pImg0 + 2, pImg0 - 2, pImg0 + 1, pImg0 - 1, curr, a5, b5 );

  int32x4_t accLo = vmull_lane_s16( vget_low_s16( a0 ), coeff[0], 0 );
  int32x4_t accHi = vmull_lane_s16( vget_high_s16( a0 ), coeff[1], 0 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b0 ), coeff[0], 1 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b0 ), coeff[1], 1 );

  accLo = vmlal_lane_s16( accLo, vget_low_s16( a1 ), coeff[0], 2 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a1 ), coeff[1], 2 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b1 ), coeff[0], 3 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b1 ), coeff[1], 3 );

  accLo = vmlal_lane_s16( accLo, vget_low_s16( a2 ), coeff[2], 0 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a2 ), coeff[3], 0 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b2 ), coeff[2], 1 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b2 ), coeff[3], 1 );

  accLo = vmlal_lane_s16( accLo, vget_low_s16( a3 ), coeff[2], 2 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a3 ), coeff[3], 2 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b3 ), coeff[2], 3 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b3 ), coeff[3], 3 );

  accLo = vmlal_lane_s16( accLo, vget_low_s16( a4 ), coeff[4], 0 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a4 ), coeff[5], 0 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b4 ), coeff[4], 1 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b4 ), coeff[5], 1 );

  accLo = vmlal_lane_s16( accLo, vget_low_s16( a5 ), coeff[4], 2 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( a5 ), coeff[5], 2 );
  accLo = vmlal_lane_s16( accLo, vget_low_s16( b5 ), coeff[4], 3 );
  accHi = vmlal_lane_s16( accHi, vget_high_s16( b5 ), coeff[5], 3 );

  int16x8_t acc;
  if( isFoldingRequired && distance >= 0 && distance <= 1 )
  {
    // Weaker filter, closer to VB.
    acc = vcombine_s16( vrshrn_n_s32( accLo, SHIFT + 3 ), vrshrn_n_s32( accHi, SHIFT + 3 ) );
  }
  else
  {
    // Regular filter strength.
    acc = vcombine_s16( vrshrn_n_s32( accLo, SHIFT ), vrshrn_n_s32( accHi, SHIFT ) );
  }

  acc = vqaddq_s16( acc, curr );
  return clip3_neon( acc, vdupq_n_s16( 0 ), vdupq_n_s16( clpRngMax ) );
}

void Filter7x7Blk_neon( const AlfClassifier* classifier, const PelUnitBuf& recDst, const CPelUnitBuf& recSrc,
                        const Area& blkDst, const Area& blk, const ComponentID compId, const short* filterSet,
                        const short* fClipSet, const ClpRng& clpRng, const CodingStructure& cs, const int vbCTUHeight,
                        int vbPos )
{
  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const ptrdiff_t srcStride = srcLuma.stride;
  const ptrdiff_t dstStride = dstLuma.stride;

  const int width = blk.width;
  const int height = blk.height;

  const Pel* src = srcLuma.buf + blk.y * srcStride + blk.x;
  Pel* dst = dstLuma.buf + blkDst.y * dstStride + blkDst.x;

  constexpr size_t STEP_X = 8;
  constexpr size_t STEP_Y = 4;

  CHECKD( width % STEP_X, "Width must be multiple of 8!" );
  CHECKD( height % STEP_Y, "Height must be multiple of 4!" );
  CHECKD( width <= 0, "Width must be greater than 0!" );
  CHECKD( height <= 0, "Height must be greater than 0!" );

  const int clpRngMax = clpRng.max();

  int i = 0;
  do
  {
    int yVbPos = ( blkDst.y + i ) & ( vbCTUHeight - 1 ); // Row’s position inside its CTU.

    auto calculateNextVbPosDist = [&yVbPos, vbPos, vbCTUHeight]() -> int
    {
      int distance = vbPos - yVbPos;
      if( ++yVbPos == vbCTUHeight )
      {
        yVbPos = 0;
      }
      return distance;
    };

    int VbDistance[STEP_Y];
    VbDistance[0] = calculateNextVbPosDist();
    VbDistance[1] = calculateNextVbPosDist();
    VbDistance[2] = calculateNextVbPosDist();
    VbDistance[3] = calculateNextVbPosDist();
    const bool foldingRequired =
        ( VbDistance[0] >= ALF_7x7_VB_FOLD_MIN_DIST && VbDistance[0] <= ALF_7x7_VB_FOLD_MAX_DIST ) ||
        ( VbDistance[1] >= ALF_7x7_VB_FOLD_MIN_DIST && VbDistance[1] <= ALF_7x7_VB_FOLD_MAX_DIST ) ||
        ( VbDistance[2] >= ALF_7x7_VB_FOLD_MIN_DIST && VbDistance[2] <= ALF_7x7_VB_FOLD_MAX_DIST ) ||
        ( VbDistance[3] >= ALF_7x7_VB_FOLD_MIN_DIST && VbDistance[3] <= ALF_7x7_VB_FOLD_MAX_DIST );

    int cl_index = ( i / 4 ) * ( MAX_CU_SIZE / 4 );
    const Pel* pImg0 = src;
    Pel* pDst = dst;

    int j = width;
    do
    {
      const AlfClassifier& cl0 = classifier[cl_index + 0];
      const AlfClassifier& cl1 = classifier[cl_index + 1];

      const int index0 = cl0.classIdx * MAX_NUM_ALF_LUMA_COEFF;
      const int index1 = cl1.classIdx * MAX_NUM_ALF_LUMA_COEFF;

      int16x4_t coeff[MAX_NUM_ALF_LUMA_COEFF / 2];

      // Copy all 12 coeffs.
      load_shuffle_coeffs_neon( g_alf_shuffle_table[cl0.transposeIdx], g_alf_shuffle_table[cl1.transposeIdx],
                                filterSet + index0, filterSet + index1, coeff );

      int16x8_t dst0, dst1, dst2, dst3;
      if( foldingRequired )
      {
        dst0 = processALF7x7Row_neon<true>( pImg0 + 0 * srcStride, VbDistance[0], srcStride, coeff, clpRngMax );
        dst1 = processALF7x7Row_neon<true>( pImg0 + 1 * srcStride, VbDistance[1], srcStride, coeff, clpRngMax );
        dst2 = processALF7x7Row_neon<true>( pImg0 + 2 * srcStride, VbDistance[2], srcStride, coeff, clpRngMax );
        dst3 = processALF7x7Row_neon<true>( pImg0 + 3 * srcStride, VbDistance[3], srcStride, coeff, clpRngMax );
      }
      else
      {
        dst0 = processALF7x7Row_neon<false>( pImg0 + 0 * srcStride, 0, srcStride, coeff, clpRngMax );
        dst1 = processALF7x7Row_neon<false>( pImg0 + 1 * srcStride, 0, srcStride, coeff, clpRngMax );
        dst2 = processALF7x7Row_neon<false>( pImg0 + 2 * srcStride, 0, srcStride, coeff, clpRngMax );
        dst3 = processALF7x7Row_neon<false>( pImg0 + 3 * srcStride, 0, srcStride, coeff, clpRngMax );
      }

      vst1q_s16( pDst + 0 * dstStride, dst0 );
      vst1q_s16( pDst + 1 * dstStride, dst1 );
      vst1q_s16( pDst + 2 * dstStride, dst2 );
      vst1q_s16( pDst + 3 * dstStride, dst3 );

      cl_index += 2;
      pImg0 += STEP_X;
      pDst += STEP_X;
      j -= STEP_X;
    } while( j != 0 );

    src += srcStride * STEP_Y;
    dst += dstStride * STEP_Y;
    i += STEP_Y;
  } while( i < height );
}

template<>
void AdaptiveLoopFilter::_initAdaptiveLoopFilterARM<NEON>()
{
  m_filter7x7Blk[0] = Filter7x7Blk_neon;
  m_deriveClassificationBlk = DeriveClassificationBlk_neon;
  m_filterCcAlf             = FilterBlkCcAlf_neon;
}

} // namespace vvenc
#endif
//! \}

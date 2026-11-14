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
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */

/** \file     LoopFilter_neon.cpp
    \brief    deblocking filter, Neon version
*/

#include <arm_neon.h>

#include "CommonDefARM.h"
#include "CommonLib/LoopFilter.h"
#include "CommonLib/Rom.h"
#include "sum_neon.h"
#include "transpose_neon.h"

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_DBLF

namespace vvenc
{

// ====================================================================================================================
// Tables (mirror x86 side)
// ====================================================================================================================

static const int dbCoeffs7[7] = { 59, 50, 41, 32, 23, 14,  5 };
static const int dbCoeffs5[5] = { 58, 45, 32, 19,  6 };
static const int dbCoeffs3[3] = { 53, 32, 11 };
static const int tcCoeffs7[7] = { 6, 5, 4, 3, 2, 1, 1 };
static const int tcCoeffs3[3] = { 6, 4, 2 };

// ====================================================================================================================
// Long (bilinear) filter -- horizontal edge (step == 1)
// ====================================================================================================================

static inline void xFilteringPandQHor_neon( Pel* src, ptrdiff_t step, const ptrdiff_t offset,
                                            int numberPSide, int numberQSide, int tc )
{
  CHECKD( step != 1, "step must be 1 for horizontal edge" );

  const int* dbCoeffsP = numberPSide == 7 ? dbCoeffs7 : ( numberPSide == 5 ? dbCoeffs5 : dbCoeffs3 );
  const int* dbCoeffsQ = numberQSide == 7 ? dbCoeffs7 : ( numberQSide == 5 ? dbCoeffs5 : dbCoeffs3 );
  const int* tcP       = numberPSide == 3 ? tcCoeffs3 : tcCoeffs7;
  const int* tcQ       = numberQSide == 3 ? tcCoeffs3 : tcCoeffs7;

  // Scalar reference computation per row (4 rows = DEBLOCK_SMALLEST_BLOCK/2)
  uint64_t refPx4 = 0, refQx4 = 0, refMiddlex4 = 0;

  for( int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++ )
  {
    const Pel* srcP = src + step * i - offset;
    const Pel* srcQ = src + step * i;

    const Pel refP = ( srcP[-numberPSide * offset] + srcP[-( numberPSide - 1 ) * offset] + 1 ) >> 1;
    const Pel refQ = ( srcQ[ numberQSide * offset] + srcQ[  ( numberQSide - 1 ) * offset] + 1 ) >> 1;

    Pel refMiddle = 0;
    if( numberPSide == numberQSide )
    {
      if( numberPSide == 5 )
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2*offset] + srcQ[2*offset] )
                      + srcP[-3*offset] + srcQ[3*offset] + srcP[-4*offset] + srcQ[4*offset] + 8 ) >> 4;
      else
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] ) + srcP[-offset] + srcQ[offset] + srcP[-2*offset] + srcQ[2*offset]
                      + srcP[-3*offset] + srcQ[3*offset] + srcP[-4*offset] + srcQ[4*offset]
                      + srcP[-5*offset] + srcQ[5*offset] + srcP[-6*offset] + srcQ[6*offset] + 8 ) >> 4;
    }
    else
    {
      const Pel* srcPt = srcP, *srcQt = srcQ;
      ptrdiff_t  offP  = -offset, offQ = offset;
      int        nP    = numberPSide, nQ = numberQSide;
      if( nQ > nP ) { std::swap( srcPt, srcQt ); std::swap( offP, offQ ); std::swap( nP, nQ ); }

      if( nP == 7 && nQ == 5 )
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] )
                      + srcP[-2*offset] + srcQ[2*offset] + srcP[-3*offset] + srcQ[3*offset]
                      + srcP[-4*offset] + srcQ[4*offset] + srcP[-5*offset] + srcQ[5*offset] + 8 ) >> 4;
      else if( nP == 7 && nQ == 3 )
        refMiddle = ( 2 * ( srcPt[0] + srcQt[0] ) + srcQt[0]
                      + 2 * ( srcQt[offQ] + srcQt[2*offQ] ) + srcPt[offP] + srcQt[offQ]
                      + srcPt[2*offP] + srcPt[3*offP] + srcPt[4*offP] + srcPt[5*offP] + srcPt[6*offP] + 8 ) >> 4;
      else
        refMiddle = ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset]
                      + srcP[-2*offset] + srcQ[2*offset] + srcP[-3*offset] + srcQ[3*offset] + 4 ) >> 3;
    }

    refPx4      |= ( (uint64_t)(uint16_t)refP      << ( i * 16 ) );
    refQx4      |= ( (uint64_t)(uint16_t)refQ      << ( i * 16 ) );
    refMiddlex4 |= ( (uint64_t)(uint16_t)refMiddle << ( i * 16 ) );
  }

  // NEON weighted blend for P side
  int16x4_t vref_mid = vreinterpret_s16_u64( vcreate_u64( refMiddlex4 ) );
  int16x4_t vref_p   = vreinterpret_s16_u64( vcreate_u64( refPx4 ) );
  int16x4_t vref_q   = vreinterpret_s16_u64( vcreate_u64( refQx4 ) );

  Pel* srcP = src - offset;
  Pel* srcQ = src;

  for( int pos = 0; pos < numberPSide; pos++ )
  {
    int16x4_t vsrc   = vld1_s16( srcP - offset * pos );
    int16x4_t cval   = vdup_n_s16( (int16_t)( ( tc * tcP[pos] ) >> 1 ) );
    int16x4_t vlo    = vsub_s16( vsrc, cval );
    int16x4_t vhi    = vadd_s16( vsrc, cval );
    int32x4_t vacc   = vmull_s16( vref_mid, vdup_n_s16( (int16_t)dbCoeffsP[pos] ) );
    vacc              = vmlal_s16( vacc, vref_p, vdup_n_s16( (int16_t)( 64 - dbCoeffsP[pos] ) ) );
    vacc              = vaddq_s32( vacc, vdupq_n_s32( 32 ) );
    int16x4_t vresult = vqmovn_s32( vshrq_n_s32( vacc, 6 ) );
    vresult            = vmin_s16( vmax_s16( vresult, vlo ), vhi );
    vst1_s16( srcP - offset * pos, vresult );
  }

  for( int pos = 0; pos < numberQSide; pos++ )
  {
    int16x4_t vsrc   = vld1_s16( srcQ + offset * pos );
    int16x4_t cval   = vdup_n_s16( (int16_t)( ( tc * tcQ[pos] ) >> 1 ) );
    int16x4_t vlo    = vsub_s16( vsrc, cval );
    int16x4_t vhi    = vadd_s16( vsrc, cval );
    int32x4_t vacc   = vmull_s16( vref_mid, vdup_n_s16( (int16_t)dbCoeffsQ[pos] ) );
    vacc              = vmlal_s16( vacc, vref_q, vdup_n_s16( (int16_t)( 64 - dbCoeffsQ[pos] ) ) );
    vacc              = vaddq_s32( vacc, vdupq_n_s32( 32 ) );
    int16x4_t vresult = vqmovn_s32( vshrq_n_s32( vacc, 6 ) );
    vresult            = vmin_s16( vmax_s16( vresult, vlo ), vhi );
    vst1_s16( srcQ + offset * pos, vresult );
  }
}

// Vertical edge long filter -- lane loads/stores across 4 rows
static inline void xFilteringPandQVer_neon( Pel* src, ptrdiff_t step, const ptrdiff_t offset,
                                            int numberPSide, int numberQSide, int tc )
{
  CHECKD( offset != 1, "offset must be 1 for vertical edge" );

  const int* dbCoeffsP = numberPSide == 7 ? dbCoeffs7 : ( numberPSide == 5 ? dbCoeffs5 : dbCoeffs3 );
  const int* dbCoeffsQ = numberQSide == 7 ? dbCoeffs7 : ( numberQSide == 5 ? dbCoeffs5 : dbCoeffs3 );
  const int* tcP       = numberPSide == 3 ? tcCoeffs3 : tcCoeffs7;
  const int* tcQ       = numberQSide == 3 ? tcCoeffs3 : tcCoeffs7;

  uint64_t refPx4 = 0, refQx4 = 0, refMiddlex4 = 0;

  for( int i = 0; i < DEBLOCK_SMALLEST_BLOCK / 2; i++ )
  {
    const Pel* srcP = src + step * i - offset;
    const Pel* srcQ = src + step * i;

    const Pel refP = ( srcP[-numberPSide * offset] + srcP[-( numberPSide - 1 ) * offset] + 1 ) >> 1;
    const Pel refQ = ( srcQ[ numberQSide * offset] + srcQ[  ( numberQSide - 1 ) * offset] + 1 ) >> 1;

    Pel refMiddle = 0;
    if( numberPSide == numberQSide )
    {
      if( numberPSide == 5 )
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] + srcP[-2*offset] + srcQ[2*offset] )
                      + srcP[-3*offset] + srcQ[3*offset] + srcP[-4*offset] + srcQ[4*offset] + 8 ) >> 4;
      else
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] ) + srcP[-offset] + srcQ[offset] + srcP[-2*offset] + srcQ[2*offset]
                      + srcP[-3*offset] + srcQ[3*offset] + srcP[-4*offset] + srcQ[4*offset]
                      + srcP[-5*offset] + srcQ[5*offset] + srcP[-6*offset] + srcQ[6*offset] + 8 ) >> 4;
    }
    else
    {
      const Pel* srcPt = srcP, *srcQt = srcQ;
      ptrdiff_t  offP  = -offset, offQ = offset;
      int        nP    = numberPSide, nQ = numberQSide;
      if( nQ > nP ) { std::swap( srcPt, srcQt ); std::swap( offP, offQ ); std::swap( nP, nQ ); }

      if( nP == 7 && nQ == 5 )
        refMiddle = ( 2 * ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset] )
                      + srcP[-2*offset] + srcQ[2*offset] + srcP[-3*offset] + srcQ[3*offset]
                      + srcP[-4*offset] + srcQ[4*offset] + srcP[-5*offset] + srcQ[5*offset] + 8 ) >> 4;
      else if( nP == 7 && nQ == 3 )
        refMiddle = ( 2 * ( srcPt[0] + srcQt[0] ) + srcQt[0]
                      + 2 * ( srcQt[offQ] + srcQt[2*offQ] ) + srcPt[offP] + srcQt[offQ]
                      + srcPt[2*offP] + srcPt[3*offP] + srcPt[4*offP] + srcPt[5*offP] + srcPt[6*offP] + 8 ) >> 4;
      else
        refMiddle = ( srcP[0] + srcQ[0] + srcP[-offset] + srcQ[offset]
                      + srcP[-2*offset] + srcQ[2*offset] + srcP[-3*offset] + srcQ[3*offset] + 4 ) >> 3;
    }

    refPx4      |= ( (uint64_t)(uint16_t)refP      << ( i * 16 ) );
    refQx4      |= ( (uint64_t)(uint16_t)refQ      << ( i * 16 ) );
    refMiddlex4 |= ( (uint64_t)(uint16_t)refMiddle << ( i * 16 ) );
  }

  int16x4_t vref_mid = vreinterpret_s16_u64( vcreate_u64( refMiddlex4 ) );
  int16x4_t vref_p   = vreinterpret_s16_u64( vcreate_u64( refPx4 ) );
  int16x4_t vref_q   = vreinterpret_s16_u64( vcreate_u64( refQx4 ) );

  Pel* srcP = src - offset;
  Pel* srcQ = src;

  for( int pos = 0; pos < numberPSide; pos++ )
  {
    Pel* base      = srcP - offset * pos;
    int16x4_t vsrc = vld1_lane_s16( base,          vdup_n_s16( 0 ), 0 );
    vsrc            = vld1_lane_s16( base + step,   vsrc,            1 );
    vsrc            = vld1_lane_s16( base + step*2, vsrc,            2 );
    vsrc            = vld1_lane_s16( base + step*3, vsrc,            3 );
    int16x4_t cval   = vdup_n_s16( (int16_t)( ( tc * tcP[pos] ) >> 1 ) );
    int16x4_t vlo    = vsub_s16( vsrc, cval );
    int16x4_t vhi    = vadd_s16( vsrc, cval );
    int32x4_t vacc   = vmull_s16( vref_mid, vdup_n_s16( (int16_t)dbCoeffsP[pos] ) );
    vacc              = vmlal_s16( vacc, vref_p, vdup_n_s16( (int16_t)( 64 - dbCoeffsP[pos] ) ) );
    vacc              = vaddq_s32( vacc, vdupq_n_s32( 32 ) );
    int16x4_t vresult = vqmovn_s32( vshrq_n_s32( vacc, 6 ) );
    vresult            = vmin_s16( vmax_s16( vresult, vlo ), vhi );
    vst1_lane_s16( base,          vresult, 0 );
    vst1_lane_s16( base + step,   vresult, 1 );
    vst1_lane_s16( base + step*2, vresult, 2 );
    vst1_lane_s16( base + step*3, vresult, 3 );
  }

  for( int pos = 0; pos < numberQSide; pos++ )
  {
    Pel* base      = srcQ + offset * pos;
    int16x4_t vsrc = vld1_lane_s16( base,          vdup_n_s16( 0 ), 0 );
    vsrc            = vld1_lane_s16( base + step,   vsrc,            1 );
    vsrc            = vld1_lane_s16( base + step*2, vsrc,            2 );
    vsrc            = vld1_lane_s16( base + step*3, vsrc,            3 );
    int16x4_t cval   = vdup_n_s16( (int16_t)( ( tc * tcQ[pos] ) >> 1 ) );
    int16x4_t vlo    = vsub_s16( vsrc, cval );
    int16x4_t vhi    = vadd_s16( vsrc, cval );
    int32x4_t vacc   = vmull_s16( vref_mid, vdup_n_s16( (int16_t)dbCoeffsQ[pos] ) );
    vacc              = vmlal_s16( vacc, vref_q, vdup_n_s16( (int16_t)( 64 - dbCoeffsQ[pos] ) ) );
    vacc              = vaddq_s32( vacc, vdupq_n_s32( 32 ) );
    int16x4_t vresult = vqmovn_s32( vshrq_n_s32( vacc, 6 ) );
    vresult            = vmin_s16( vmax_s16( vresult, vlo ), vhi );
    vst1_lane_s16( base,          vresult, 0 );
    vst1_lane_s16( base + step,   vresult, 1 );
    vst1_lane_s16( base + step*2, vresult, 2 );
    vst1_lane_s16( base + step*3, vresult, 3 );
  }
}

static void xFilteringPandQNeon( Pel* src, ptrdiff_t step, const ptrdiff_t offset,
                                 int numberPSide, int numberQSide, int tc )
{
  CHECKD( numberPSide <= 3 && numberQSide <= 3, "Short filtering in long filtering function" );

  if( step == 1 )
    xFilteringPandQHor_neon( src, step, offset, numberPSide, numberQSide, tc );
  else
    xFilteringPandQVer_neon( src, step, offset, numberPSide, numberQSide, tc );
}

// ====================================================================================================================
// Short (strong and weak) filter of 4 luma lines
// ====================================================================================================================

// Clips each lane of f to x - range .. x + range, as Clip3( x - range, x + range, f ).
static inline int16x8_t clipAround_s16( const int16x8_t f, const int16x8_t x, const int16x8_t range )
{
  return vminq_s16( vmaxq_s16( f, vsubq_s16( x, range ) ), vaddq_s16( x, range ) );
}

// The 4 lines are held in pairs: x0 = [ p0 | q0 ], x1 = [ p1 | q1 ], x2 = [ p2 | q2 ], x3 = [ p3 | q3 ], the P side in
// the low half and the Q side in the high half, one lane per line. The formulas of the two sides are mirror images
// (p_i <-> q_i), so each vector operation filters both sides, and vextq_s16( x, x, 4 ) is the other side of the edge.
// The sums stay below 8 * 4095 for samples of up to 12 bits, so 16-bit lanes are exact.
static inline void xPelFilterLumaStrong_neon( int16x8_t& x0, int16x8_t& x1, int16x8_t& x2, const int16x8_t x3,
                                              const int tc )
{
  const int16x8_t y0 = vextq_s16( x0, x0, 4 );
  const int16x8_t y1 = vextq_s16( x1, x1, 4 );
  const int16x8_t s  = vaddq_s16( vaddq_s16( x1, x0 ), y0 );   // p1 + p0 + q0

  // ( p2 + 2 p1 + 2 p0 + 2 q0 + q1 + 4 ) >> 3, ( p2 + p1 + p0 + q0 + 2 ) >> 2, ( 2 p3 + 3 p2 + p1 + p0 + q0 + 4 ) >> 3
  const int16x8_t f0 = vrshrq_n_s16( vaddq_s16( vaddq_s16( x2, y1 ), vshlq_n_s16( s, 1 ) ), 3 );
  const int16x8_t f1 = vrshrq_n_s16( vaddq_s16( x2, s ), 2 );
  const int16x8_t f2 = vrshrq_n_s16( vmlaq_n_s16( vaddq_s16( x2, s ), vaddq_s16( x3, x2 ), 2 ), 3 );

  // each output stays within 3, 2 and 1 times tc of the sample it replaces
  x0 = clipAround_s16( f0, x0, vdupq_n_s16( 3 * tc ) );
  x1 = clipAround_s16( f1, x1, vdupq_n_s16( 2 * tc ) );
  x2 = clipAround_s16( f2, x2, vdupq_n_s16( tc ) );
}

// The weak filter of the pairs above: delta = ( 9 ( q0 - p0 ) - 3 ( q1 - p1 ) + 8 ) >> 4 per line, applied where
// | delta | < thrCut, clamped to +-tc, added to p0 and subtracted from q0. p1 and q1, if their flags are set, move by
// ( ( ( p2 + p0 + 1 ) >> 1 ) - p1 + delta ) >> 1 clamped to +-( tc >> 1 ), the Q side with -delta. A line that is not
// filtered adds 0, which leaves its samples as they are; vvenc_vsqaddq_u16 clips the sums at 0, vminq_u16 at the
// maximum.
static inline void xPelFilterLumaWeak_neon( int16x8_t& x0, int16x8_t& x1, const int16x8_t x2, const int tc,
                                            const int thrCut, const bool secondP, const bool secondQ,
                                            const ClpRng& clpRng )
{
  const int16x4_t  d0       = vsub_s16( vget_high_s16( x0 ), vget_low_s16( x0 ) );   // q0 - p0
  const int16x4_t  d1       = vsub_s16( vget_high_s16( x1 ), vget_low_s16( x1 ) );   // q1 - p1
  const int32x4_t  sum      = vmlsl_n_s16( vmull_n_s16( d0, 9 ), d1, 3 );
  int16x4_t        delta    = vrshrn_n_s32( sum, 4 );
  const uint16x4_t passMask = vclt_s16( vabs_s16( delta ), vdup_n_s16( thrCut ) );
  delta                     = vmin_s16( vmax_s16( delta, vdup_n_s16( -tc ) ), vdup_n_s16( tc ) );
  delta                     = vand_s16( delta, vreinterpret_s16_u16( passMask ) );

  const int16x8_t  deltaPQ = vcombine_s16( delta, vneg_s16( delta ) );   // [ delta | -delta ]
  const uint16x8_t maxVal  = vdupq_n_u16( clpRng.max() );

  if( secondP || secondQ )
  {
    const int16x8_t  tcHalf     = vdupq_n_s16( tc >> 1 );
    const int16x8_t  mid        = vsubq_s16( vrhaddq_s16( x2, x0 ), x1 );   // ( ( p2 + p0 + 1 ) >> 1 ) - p1
    const int16x8_t  stepAll    = vminq_s16( vmaxq_s16( vhaddq_s16( mid, deltaPQ ), vnegq_s16( tcHalf ) ), tcHalf );
    const uint16x4_t passP      = vand_u16( passMask, vdup_n_u16( secondP ? 0xffff : 0 ) );
    const uint16x4_t passQ      = vand_u16( passMask, vdup_n_u16( secondQ ? 0xffff : 0 ) );
    const int16x8_t  step       = vandq_s16( stepAll, vreinterpretq_s16_u16( vcombine_u16( passP, passQ ) ) );
    x1 = vreinterpretq_s16_u16( vminq_u16( vvenc_vsqaddq_u16( vreinterpretq_u16_s16( x1 ), step ), maxVal ) );
  }
  x0 = vreinterpretq_s16_u16( vminq_u16( vvenc_vsqaddq_u16( vreinterpretq_u16_s16( x0 ), deltaPQ ), maxVal ) );
}

// A horizontal edge: the lines are the 4 columns from piSrc on, and p_k, q_k are rows -1 - k and k.
static void xPelFilterLumaHor_neon( Pel* piSrc, const ptrdiff_t offset, const int tc, const bool sw, const int thrCut,
                                    const bool secondP, const bool secondQ, const ClpRng& clpRng )
{
  const int16x4_t p2 = vld1_s16( piSrc - 3 * offset );
  const int16x4_t p1 = vld1_s16( piSrc - 2 * offset );
  const int16x4_t p0 = vld1_s16( piSrc - 1 * offset );
  const int16x4_t q0 = vld1_s16( piSrc );
  const int16x4_t q1 = vld1_s16( piSrc + 1 * offset );
  const int16x4_t q2 = vld1_s16( piSrc + 2 * offset );
  int16x8_t       x0 = vcombine_s16( p0, q0 );
  int16x8_t       x1 = vcombine_s16( p1, q1 );
  int16x8_t       x2 = vcombine_s16( p2, q2 );

  if( sw )
  {
    const int16x4_t p3 = vld1_s16( piSrc - 4 * offset );
    const int16x4_t q3 = vld1_s16( piSrc + 3 * offset );
    xPelFilterLumaStrong_neon( x0, x1, x2, vcombine_s16( p3, q3 ), tc );
    vst1_s16( piSrc - 3 * offset, vget_low_s16( x2 ) );
    vst1_s16( piSrc + 2 * offset, vget_high_s16( x2 ) );
  }
  else
  {
    xPelFilterLumaWeak_neon( x0, x1, x2, tc, thrCut, secondP, secondQ, clpRng );
  }
  vst1_s16( piSrc - 2 * offset, vget_low_s16( x1 ) );
  vst1_s16( piSrc - 1 * offset, vget_low_s16( x0 ) );
  vst1_s16( piSrc,              vget_high_s16( x0 ) );
  vst1_s16( piSrc + 1 * offset, vget_high_s16( x1 ) );
}

// A vertical edge: the lines are the 4 rows from piSrc on, each [ p3 p2 p1 p0 q0 q1 q2 q3 ] around piSrc. Transposing
// both halves gives a0 = [ p3 | q0 ], a1 = [ p2 | q1 ], a2 = [ p1 | q2 ], a3 = [ p0 | q3 ], and x_k takes the low half
// of a_3-k and the high half of a_k. That map and the transpose are their own inverses, so the same steps write back
// the 8 samples of each line (those the filter does not change with the values they had).
static void xPelFilterLumaVer_neon( Pel* piSrc, const ptrdiff_t step, const int tc, const bool sw, const int thrCut,
                                    const bool secondP, const bool secondQ, const ClpRng& clpRng )
{
  int16x8_t a0 = vld1q_s16( piSrc - 4 );
  int16x8_t a1 = vld1q_s16( piSrc - 4 + step );
  int16x8_t a2 = vld1q_s16( piSrc - 4 + 2 * step );
  int16x8_t a3 = vld1q_s16( piSrc - 4 + 3 * step );
  transpose_4x4_s16_halves( a0, a1, a2, a3 );

  int16x8_t       x0 = vcombine_s16( vget_low_s16( a3 ), vget_high_s16( a0 ) );
  int16x8_t       x1 = vcombine_s16( vget_low_s16( a2 ), vget_high_s16( a1 ) );
  int16x8_t       x2 = vcombine_s16( vget_low_s16( a1 ), vget_high_s16( a2 ) );
  const int16x8_t x3 = vcombine_s16( vget_low_s16( a0 ), vget_high_s16( a3 ) );

  if( sw )
    xPelFilterLumaStrong_neon( x0, x1, x2, x3, tc );
  else
    xPelFilterLumaWeak_neon( x0, x1, x2, tc, thrCut, secondP, secondQ, clpRng );

  a0 = vcombine_s16( vget_low_s16( x3 ), vget_high_s16( x0 ) );
  a1 = vcombine_s16( vget_low_s16( x2 ), vget_high_s16( x1 ) );
  a2 = vcombine_s16( vget_low_s16( x1 ), vget_high_s16( x2 ) );
  a3 = vcombine_s16( vget_low_s16( x0 ), vget_high_s16( x3 ) );
  transpose_4x4_s16_halves( a0, a1, a2, a3 );
  vst1q_s16( piSrc - 4,            a0 );
  vst1q_s16( piSrc - 4 + step,     a1 );
  vst1q_s16( piSrc - 4 + 2 * step, a2 );
  vst1q_s16( piSrc - 4 + 3 * step, a3 );
}

static void xPelFilterLumaNeon( Pel* piSrc, const ptrdiff_t step, const ptrdiff_t offset, const int tc, const bool sw,
                                const int iThrCut, const bool bFilterSecondP, const bool bFilterSecondQ,
                                const ClpRng& clpRng )
{
  CHECKD( clpRng.bd > 12, "the 16-bit sums need samples of at most 12 bits" );

  if( offset == 1 )
    xPelFilterLumaVer_neon( piSrc, step, tc, sw, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );
  else
    xPelFilterLumaHor_neon( piSrc, offset, tc, sw, iThrCut, bFilterSecondP, bFilterSecondQ, clpRng );
}

// ====================================================================================================================
// Init
// ====================================================================================================================

template <ARM_VEXT vext>
void LoopFilter::_initLoopFilterARM()
{
  xFilteringPandQ = xFilteringPandQNeon;
  xPelFilterLuma  = xPelFilterLumaNeon;
}

template void LoopFilter::_initLoopFilterARM<NEON>();

void xFilteringPandQNeonEntry( Pel* src, ptrdiff_t step, const ptrdiff_t offset,
                               int numberPSide, int numberQSide, int tc )
{
  xFilteringPandQNeon( src, step, offset, numberPSide, numberQSide, tc );
}

}  // namespace vvenc

#endif  // TARGET_SIMD_ARM && ENABLE_SIMD_DBLF

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
 * \file InterPred_neon.cpp
 * \brief Neon implementation of InterPrediction for Arm.
 */
//  ====================================================================================================================
//  Includes
//  ====================================================================================================================

#include "CommonDefARM.h"
#include "InterPrediction.h"
#include "Rom.h"
#include "neon/permute_neon.h"
#include "neon/sum_neon.h"

#if ENABLE_SIMD_OPT_BDOF && defined( TARGET_SIMD_ARM )

namespace vvenc {

static inline int rightShiftMSB( int numer, int denom )
{
  int shiftIdx = bit_scan_reverse( denom );
  return numer >> shiftIdx;
}

static inline int16x8_t signum_neon( int16x8_t x )
{
  x = vqshlq_n_s16( x, 15 );
  return vrshrq_n_s16( x, 15 );
}

static inline void calcBIOSums_neon( const Pel* srcY0Tmp, const Pel* srcY1Tmp, const Pel* gradX0, const Pel* gradX1,
                                     const Pel* gradY0, const Pel* gradY1, int gradOfs, const int widthG,
                                     const int bitDepth, int limit, int& tmpx, int& tmpy )
{
  const int srcStride = widthG + 2;
  int16x8_t sumAbsGXTmp = vdupq_n_s16( 0 );
  int16x8_t sumDIXTmp = vdupq_n_s16( 0 );
  int16x8_t sumAbsGYTmp = vdupq_n_s16( 0 );
  int16x8_t sumDIYTmp = vdupq_n_s16( 0 );
  int16x8_t sumSignGyGxTmp = vdupq_n_s16( 0 );
  int16_t mask6_arr[8] = { ~0, ~0, ~0, ~0, ~0, ~0, 0, 0 };
  int16x8_t mask6 = vld1q_s16( mask6_arr );

  for( int y = 0; y < 6; y++ )
  {
    int16x8_t shiftSrcY0Tmp = vshrq_n_s16( vld1q_s16( srcY0Tmp ), 4 );
    int16x8_t shiftSrcY1Tmp = vshrq_n_s16( vld1q_s16( srcY1Tmp ), 4 );

    int16x8_t loadGradX0 = vld1q_s16( gradX0 + gradOfs );
    int16x8_t loadGradX1 = vld1q_s16( gradX1 + gradOfs );
    int16x8_t loadGradY0 = vld1q_s16( gradY0 + gradOfs );
    int16x8_t loadGradY1 = vld1q_s16( gradY1 + gradOfs );
    int16x8_t subTemp1 = vsubq_s16( shiftSrcY1Tmp, shiftSrcY0Tmp );
    int16x8_t packTempX = vhaddq_s16( loadGradX0, loadGradX1 );
    int16x8_t packTempY = vhaddq_s16( loadGradY0, loadGradY1 );

    int16x8_t signX = signum_neon( packTempX );
    int16x8_t signY = signum_neon( packTempY );

    sumAbsGXTmp = vabaq_s16( sumAbsGXTmp, packTempX, vdupq_n_s16( 0 ) );
    sumAbsGYTmp = vabaq_s16( sumAbsGYTmp, packTempY, vdupq_n_s16( 0 ) );
    sumDIXTmp = vmlaq_s16( sumDIXTmp, subTemp1, signX );
    sumDIYTmp = vmlaq_s16( sumDIYTmp, subTemp1, signY );
    sumSignGyGxTmp = vmlaq_s16( sumSignGyGxTmp, packTempX, signY );

    srcY0Tmp += srcStride;
    srcY1Tmp += srcStride;
    gradOfs += widthG;
  }

  // Each lane holds the sum of 6 rows, which fits 16 bits; the sum over the 6 columns does not always (dI up to 1023 per
  // sample for black and white predictions: 36 * 1023 > 32767), so the lanes are added in 32 bits.
  int sumAbsGX = horizontal_add_long_s16x8( vandq_s16( sumAbsGXTmp, mask6 ) );
  int sumAbsGY = horizontal_add_long_s16x8( vandq_s16( sumAbsGYTmp, mask6 ) );
  int sumDIX = horizontal_add_long_s16x8( vandq_s16( sumDIXTmp, mask6 ) );
  int sumDIY = horizontal_add_long_s16x8( vandq_s16( sumDIYTmp, mask6 ) );
  int sumSignGY_GX = horizontal_add_long_s16x8( vandq_s16( sumSignGyGxTmp, mask6 ) );

  tmpx = sumAbsGX == 0 ? 0 : rightShiftMSB( sumDIX << 2, sumAbsGX );
  tmpx = Clip3( -limit, limit, tmpx );

  int mainsGxGy = sumSignGY_GX >> 12;
  int secsGxGy = sumSignGY_GX & ( ( 1 << 12 ) - 1 );
  int tmpData = tmpx * mainsGxGy;
  tmpData = ( ( tmpData << 12 ) + tmpx * secsGxGy ) >> 1;
  tmpy = sumAbsGY == 0 ? 0 : rightShiftMSB( ( sumDIY << 2 ) - tmpData, sumAbsGY );
  tmpy = Clip3( -limit, limit, tmpy );
}

static inline void addBIOAvg4_x2_neon( const int16_t* src0, const int16_t* src1, int16_t* dst, ptrdiff_t dstStride,
                                       const int16_t* gradX0, const int16_t* gradX1, const int16_t* gradY0,
                                       const int16_t* gradY1, int gradOfs, ptrdiff_t widthG, int tmpx[2], int tmpy[2],
                                       int shift, int offset, const ClpRng& clpRng )
{
  const ptrdiff_t srcStride = widthG + 2;
  const ptrdiff_t gradStride = widthG;
  const int32x4_t voffset = vdupq_n_s32( offset );
  const uint16x8_t vibdimax = vdupq_n_u16( clpRng.max() );

  for( int y = 0; y < 4; y++ )
  {
    int16x8_t a = vsubq_s16( vld1q_s16( gradX0 + gradOfs ), vld1q_s16( gradX1 + gradOfs ) );
    int16x8_t b = vsubq_s16( vld1q_s16( gradY0 + gradOfs ), vld1q_s16( gradY1 + gradOfs ) );

    int16x8_t s0 = vld1q_s16( src0 );
    int16x8_t s1 = vld1q_s16( src1 );

    int32x4_t s01_lo = vaddl_s16( vget_low_s16( s0 ), vget_low_s16( s1 ) );
    int32x4_t s01_hi = vaddl_s16( vget_high_s16( s0 ), vget_high_s16( s1 ) );

    int32x4_t sum_lo = vaddq_s32( voffset, s01_lo );
    int32x4_t sum_hi = vaddq_s32( voffset, s01_hi );

    sum_lo = vmlal_n_s16( sum_lo, vget_low_s16( a ), tmpx[0] );
    sum_lo = vmlal_n_s16( sum_lo, vget_low_s16( b ), tmpy[0] );
    sum_hi = vmlal_n_s16( sum_hi, vget_high_s16( a ), tmpx[1] );
    sum_hi = vmlal_n_s16( sum_hi, vget_high_s16( b ), tmpy[1] );

    uint16x4_t sum_u16_lo = vqmovun_s32( vshlq_s32( sum_lo, vdupq_n_s32( -shift ) ) );
    uint16x4_t sum_u16_hi = vqmovun_s32( vshlq_s32( sum_hi, vdupq_n_s32( -shift ) ) );
    uint16x8_t sum_u16 = vcombine_u16( sum_u16_lo, sum_u16_hi );

    int16x8_t sum_s16 = vreinterpretq_s16_u16( vminq_u16( vibdimax, sum_u16 ) );

    vst1q_s16( dst, sum_s16 );

    dst += dstStride;
    src0 += srcStride;
    src1 += srcStride;
    gradOfs += gradStride;
  }
}

template<int width, int height>
static void BiOptFlow_neon_impl( const Pel* srcY0, const Pel* srcY1, const Pel* gradX0, const Pel* gradX1,
                                 const Pel* gradY0, const Pel* gradY1, Pel* dstY, const ptrdiff_t dstStride,
                                 const int shiftNum, const int offset, const int limit, const ClpRng& clpRng,
                                 const int bitDepth )
{
  constexpr int widthG = width + 2 * BDOF_EXTEND_SIZE;
  constexpr int stridePredMC = widthG + 2;
  constexpr int xUnit = width >> 2;
  int yUnit = height >> 2;
  int offsetPos = widthG * BDOF_EXTEND_SIZE + BDOF_EXTEND_SIZE;
  int offsetPad = 0;

  static_assert( width >= 8 && height >= 4, "Invalid width or height" );

  do
  {
    const Pel* srcY0Temp = srcY0;
    const Pel* srcY1Temp = srcY1;
    Pel* dstY0 = dstY;

    int offPos = offsetPos;
    int offPad = offsetPad;

    for( int xu = 0; xu < xUnit; xu += 2 )
    {
      int tmpx[2], tmpy[2];
      calcBIOSums_neon( srcY0Temp, srcY1Temp, gradX0, gradX1, gradY0, gradY1, offPad, widthG, bitDepth, limit, tmpx[0],
                        tmpy[0] );

      calcBIOSums_neon( srcY0Temp + 4, srcY1Temp + 4, gradX0, gradX1, gradY0, gradY1, offPad + 4, widthG, bitDepth,
                        limit, tmpx[1], tmpy[1] );

      addBIOAvg4_x2_neon( srcY0Temp + stridePredMC + 1, srcY1Temp + stridePredMC + 1, dstY0, dstStride, gradX0, gradX1,
                          gradY0, gradY1, offPos, widthG, tmpx, tmpy, shiftNum, offset, clpRng );
      srcY0Temp += 8;
      srcY1Temp += 8;
      dstY0 += 8;
      offPos += 8;
      offPad += 8;
    }

    srcY0 += stridePredMC << 2;
    srcY1 += stridePredMC << 2;
    dstY += dstStride << 2;
    offsetPos += widthG << 2;
    offsetPad += widthG << 2;
  } while( --yUnit != 0 );
}

void BiOptFlow_neon( const Pel* srcY0, const Pel* srcY1, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0,
                     const Pel* gradY1, const int width, const int height, Pel* dstY, const ptrdiff_t dstStride,
                     const int shiftNum, const int offset, const int limit, const ClpRng& clpRng, const int bitDepth )
{
  if( width == 8 && height == 16 )
  {
    BiOptFlow_neon_impl<8, 16>( srcY0, srcY1, gradX0, gradX1, gradY0, gradY1, dstY, dstStride, shiftNum, offset, limit,
                                clpRng, bitDepth );
  }
  else if( width == 16 && height == 8 )
  {
    BiOptFlow_neon_impl<16, 8>( srcY0, srcY1, gradX0, gradX1, gradY0, gradY1, dstY, dstStride, shiftNum, offset, limit,
                                clpRng, bitDepth );
  }
  else if( width == 16 && height == 16 )
  {
    BiOptFlow_neon_impl<16, 16>( srcY0, srcY1, gradX0, gradX1, gradY0, gradY1, dstY, dstStride, shiftNum, offset, limit,
                                 clpRng, bitDepth );
  }
  else
  {
    CHECKD( true, "Unsupported height and width combination" );
  }
}

template<bool PAD = true>
void gradFilter_neon( const Pel* pSrc, int srcStride, int width, int height, int gradStride, Pel* gradX, Pel* gradY,
                      const int bitDepth )
{
  const Pel* srcTmp = pSrc + srcStride + 1;
  Pel* gradXTmp = gradX + gradStride + 1;
  Pel* gradYTmp = gradY + gradStride + 1;
  int widthInside = width - 2 * BDOF_EXTEND_SIZE;
  int heightInside = height - 2 * BDOF_EXTEND_SIZE;
  static constexpr int shift = 6;

  CHECK( widthInside < 4, "(Width - 2) must be greater than or equal to 4!" );
  CHECK( heightInside % 2 != 0, "(Height - 2) must be multiple of 2!" );

  if( widthInside % 8 == 0 )
  {
    int y = heightInside;
    do
    {
      int x = widthInside;
      do
      {
        int16x8_t srcRight = vld1q_s16( srcTmp + 1 );
        int16x8_t srcLeft = vld1q_s16( srcTmp - 1 );

        int16x8_t srcBottom = vld1q_s16( srcTmp + srcStride );
        int16x8_t srcTop = vld1q_s16( srcTmp - srcStride );

        srcRight = vshrq_n_s16( srcRight, shift );
        srcLeft = vshrq_n_s16( srcLeft, shift );
        srcBottom = vshrq_n_s16( srcBottom, shift );
        srcTop = vshrq_n_s16( srcTop, shift );

        const int16x8_t grad_x = vsubq_s16( srcRight, srcLeft );
        const int16x8_t grad_y = vsubq_s16( srcBottom, srcTop );

        vst1q_s16( gradXTmp, grad_x );
        vst1q_s16( gradYTmp, grad_y );

        srcTmp += 8;
        gradXTmp += 8;
        gradYTmp += 8;
        x -= 8;
      } while( x != 0 );

      gradXTmp += gradStride - widthInside;
      gradYTmp += gradStride - widthInside;
      srcTmp += srcStride - widthInside;
    } while( --y != 0 );
  }
  else
  {
    CHECK( widthInside != 4, "(Width - 2) must be equal to 4!" );
    int y = heightInside >> 1;

    int16x8_t srcTop = vcombine_s16( vld1_s16( srcTmp - srcStride ), vld1_s16( srcTmp ) );
    srcTop = vshrq_n_s16( srcTop, shift );

    do
    {
      int16x8_t srcRight = vcombine_s16( vld1_s16( srcTmp + 1 ), vld1_s16( srcTmp + srcStride + 1 ) );
      int16x8_t srcLeft = vcombine_s16( vld1_s16( srcTmp - 1 ), vld1_s16( srcTmp + srcStride - 1 ) );
      int16x8_t srcBottom = vcombine_s16( vld1_s16( srcTmp + srcStride ), vld1_s16( srcTmp + ( srcStride << 1 ) ) );

      srcRight = vshrq_n_s16( srcRight, shift );
      srcLeft = vshrq_n_s16( srcLeft, shift );
      srcBottom = vshrq_n_s16( srcBottom, shift );

      const int16x8_t grad_x = vsubq_s16( srcRight, srcLeft );
      const int16x8_t grad_y = vsubq_s16( srcBottom, srcTop );

      vst1_s16( gradXTmp, vget_low_s16( grad_x ) );
      vst1_s16( gradXTmp + gradStride, vget_high_s16( grad_x ) );
      vst1_s16( gradYTmp, vget_low_s16( grad_y ) );
      vst1_s16( gradYTmp + gradStride, vget_high_s16( grad_y ) );

      gradXTmp += gradStride << 1;
      gradYTmp += gradStride << 1;
      srcTmp += srcStride << 1;
      srcTop = srcBottom; // For next iteration.
    } while( --y != 0 );
  }

  if( PAD )
  {
    gradXTmp = gradX + gradStride + 1;
    gradYTmp = gradY + gradStride + 1;
    int y = heightInside;
    do
    {
      gradXTmp[-1] = gradXTmp[0];
      gradXTmp[widthInside] = gradXTmp[widthInside - 1];
      gradXTmp += gradStride;

      gradYTmp[-1] = gradYTmp[0];
      gradYTmp[widthInside] = gradYTmp[widthInside - 1];
      gradYTmp += gradStride;
    } while( --y != 0 );

    gradXTmp = gradX + gradStride;
    gradYTmp = gradY + gradStride;
    memcpy( gradXTmp - gradStride, gradXTmp, sizeof( Pel ) * width );
    memcpy( gradXTmp + heightInside * gradStride, gradXTmp + ( heightInside - 1 ) * gradStride, sizeof( Pel ) * width );
    memcpy( gradYTmp - gradStride, gradYTmp, sizeof( Pel ) * width );
    memcpy( gradYTmp + heightInside * gradStride, gradYTmp + ( heightInside - 1 ) * gradStride, sizeof( Pel ) * width );
  }
}

// Duplicate the right element once: 0, 1, 2, 3, 4, 5, 6, 6
static const uint8_t kPad1RightTbl[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 12, 13 };
// Duplicate the left element once: 0, 0, 1, 2, 3, 4, 5, 6
static const uint8_t kPad1LeftTbl[] = { 0, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };
// Duplicate the right element twice: 0, 1, 2, 3, 4, 5, 5, 5
static const uint8_t kPad2RightTbl[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 10, 11, 10, 11 };
// Duplicate the left element twice: 0, 0, 0, 1, 2, 3, 4, 5
static const uint8_t kPad2LeftTbl[] = { 0, 1, 0, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

static inline void padDmvr_1_7( const Pel* src, const int srcStride, Pel* dst, const int dstStride, int height )
{
  uint8x16_t idx = vld1q_u8( kPad1RightTbl );

  int16x8_t s = vld1q_s16( src );
  int16x8_t r = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s ), idx ) );
  dst[-dstStride - 1] = vgetq_lane_s16( r, 0 );
  vst1q_s16( dst - dstStride, r );
  dst[-1] = vgetq_lane_s16( r, 0 );
  vst1q_s16( dst, r );

  do
  {
    src += srcStride;
    dst += dstStride;

    s = vld1q_s16( src );
    r = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s ), idx ) );
    dst[-1] = vgetq_lane_s16( r, 0 );
    vst1q_s16( dst, r );
  } while( --height != 1 );

  dst[dstStride - 1] = vgetq_lane_s16( r, 0 );
  vst1q_s16( dst + dstStride, r );
}

static inline void padDmvr_1_11( const Pel* src, const int srcStride, Pel* dst, const int dstStride, int height )
{
  uint8x16_t idx0 = vld1q_u8( kPad1LeftTbl );
  uint8x16_t idx1 = vld1q_u8( kPad1RightTbl );

  int16x8_t s0 = vld1q_s16( src + 0 );
  int16x8_t s1 = vld1q_s16( src + 4 );
  int16x8_t r0 = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s0 ), idx0 ) );
  int16x8_t r1 = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s1 ), idx1 ) );
  vst1q_s16( dst - dstStride - 1, r0 );
  vst1q_s16( dst - dstStride + 4, r1 );
  vst1q_s16( dst - 1, r0 );
  vst1q_s16( dst + 4, r1 );

  do
  {
    src += srcStride;
    dst += dstStride;

    s0 = vld1q_s16( src + 0 );
    s1 = vld1q_s16( src + 4 );
    r0 = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s0 ), idx0 ) );
    r1 = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s1 ), idx1 ) );
    vst1q_s16( dst - 1, r0 );
    vst1q_s16( dst + 4, r1 );
  } while( --height != 1 );

  vst1q_s16( dst + dstStride - 1, r0 );
  vst1q_s16( dst + dstStride + 4, r1 );
}

static inline void padDmvr_2_15( const Pel* src, const int srcStride, Pel* dst, const int dstStride, int height )
{
  uint8x16_t idx0 = vld1q_u8( kPad2LeftTbl );
  uint8x16_t idx1 = vld1q_u8( kPad2RightTbl );

  int16x8_t s0 = vld1q_s16( src + 0 );
  int16x8_t s1 = vld1q_s16( src + 6 );
  int16x8_t s2 = vld1q_s16( src + 9 );
  int16x8_t r0 = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s0 ), idx0 ) );
  int16x8_t r1 = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s2 ), idx1 ) );
  vst1q_s16( dst - 2 * dstStride - 2, r0 );
  vst1q_s16( dst - 2 * dstStride + 6, s1 );
  vst1q_s16( dst - 2 * dstStride + 9, r1 );
  vst1q_s16( dst - 1 * dstStride - 2, r0 );
  vst1q_s16( dst - 1 * dstStride + 6, s1 );
  vst1q_s16( dst - 1 * dstStride + 9, r1 );
  vst1q_s16( dst - 0 * dstStride - 2, r0 );
  vst1q_s16( dst - 0 * dstStride + 6, s1 );
  vst1q_s16( dst - 0 * dstStride + 9, r1 );

  do
  {
    src += srcStride;
    dst += dstStride;

    s0 = vld1q_s16( src + 0 );
    s1 = vld1q_s16( src + 6 );
    s2 = vld1q_s16( src + 9 );

    r0 = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s0 ), idx0 ) );
    r1 = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s2 ), idx1 ) );
    vst1q_s16( dst - 2, r0 );
    vst1q_s16( dst + 6, s1 );
    vst1q_s16( dst + 9, r1 );
  } while( --height != 1 );

  vst1q_s16( dst + 1 * dstStride - 2, r0 );
  vst1q_s16( dst + 1 * dstStride + 6, s1 );
  vst1q_s16( dst + 1 * dstStride + 9, r1 );
  vst1q_s16( dst + 2 * dstStride - 2, r0 );
  vst1q_s16( dst + 2 * dstStride + 6, s1 );
  vst1q_s16( dst + 2 * dstStride + 9, r1 );
}

static inline void padDmvr_2_23( const Pel* src, const int srcStride, Pel* dst, const int dstStride, int height )
{
  uint8x16_t idx0 = vld1q_u8( kPad2LeftTbl );
  uint8x16_t idx1 = vld1q_u8( kPad2RightTbl );

  int16x8_t s0 = vld1q_s16( src + 0 );
  int16x8_t s1 = vld1q_s16( src + 6 );
  int16x8_t s2 = vld1q_s16( src + 14 );
  int16x8_t s3 = vld1q_s16( src + 17 );
  int16x8_t r0 = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s0 ), idx0 ) );
  int16x8_t r1 = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s3 ), idx1 ) );
  vst1q_s16( dst - 2 * dstStride - 2, r0 );
  vst1q_s16( dst - 2 * dstStride + 6, s1 );
  vst1q_s16( dst - 2 * dstStride + 14, s2 );
  vst1q_s16( dst - 2 * dstStride + 17, r1 );
  vst1q_s16( dst - 1 * dstStride - 2, r0 );
  vst1q_s16( dst - 1 * dstStride + 6, s1 );
  vst1q_s16( dst - 1 * dstStride + 14, s2 );
  vst1q_s16( dst - 1 * dstStride + 17, r1 );
  vst1q_s16( dst - 0 * dstStride - 2, r0 );
  vst1q_s16( dst - 0 * dstStride + 6, s1 );
  vst1q_s16( dst - 0 * dstStride + 14, s2 );
  vst1q_s16( dst - 0 * dstStride + 17, r1 );

  do
  {
    src += srcStride;
    dst += dstStride;

    s0 = vld1q_s16( src + 0 );
    s1 = vld1q_s16( src + 6 );
    s2 = vld1q_s16( src + 14 );
    s3 = vld1q_s16( src + 17 );

    r0 = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s0 ), idx0 ) );
    r1 = vreinterpretq_s16_s8( vvenc_vqtbl1q_s8( vreinterpretq_s8_s16( s3 ), idx1 ) );
    vst1q_s16( dst - 2, r0 );
    vst1q_s16( dst + 6, s1 );
    vst1q_s16( dst + 14, s2 );
    vst1q_s16( dst + 17, r1 );
  } while( --height != 1 );

  vst1q_s16( dst + 1 * dstStride - 2, r0 );
  vst1q_s16( dst + 1 * dstStride + 6, s1 );
  vst1q_s16( dst + 1 * dstStride + 14, s2 );
  vst1q_s16( dst + 1 * dstStride + 17, r1 );
  vst1q_s16( dst + 2 * dstStride - 2, r0 );
  vst1q_s16( dst + 2 * dstStride + 6, s1 );
  vst1q_s16( dst + 2 * dstStride + 14, s2 );
  vst1q_s16( dst + 2 * dstStride + 17, r1 );
}

void padDmvr_neon( const Pel* src, const int srcStride, Pel* dst, const int dstStride, int width, int height,
                   int padSize )
{
  if( padSize == 1 && width == 7 )
  {
    padDmvr_1_7( src, srcStride, dst, dstStride, height );
  }
  else if( padSize == 1 && width == 11 )
  {
    padDmvr_1_11( src, srcStride, dst, dstStride, height );
  }
  else if( padSize == 2 && width == 15 )
  {
    padDmvr_2_15( src, srcStride, dst, dstStride, height );
  }
  else if( padSize == 2 && width == 23 )
  {
    padDmvr_2_23( src, srcStride, dst, dstStride, height );
  }
  else
  {
    CHECKD( true, "Unsupported padSize and width combination" );
  }
}

// The five per-sample terms BDOF sums, for the 16 columns of a row (int16, columns 0-7 in [0] and 8-15 in [1])
struct BdofTerms16
{
  int16x8_t absGX[2], absGY[2], dIX[2], dIY[2], signGYGX[2];
};

static inline void addBdofTerms16( BdofTerms16& sum, const BdofTerms16& terms )
{
  for( int h = 0; h < 2; h++ )
  {
    sum.absGX[h]    = vaddq_s16( sum.absGX[h], terms.absGX[h] );
    sum.absGY[h]    = vaddq_s16( sum.absGY[h], terms.absGY[h] );
    sum.dIX[h]      = vaddq_s16( sum.dIX[h], terms.dIX[h] );
    sum.dIY[h]      = vaddq_s16( sum.dIY[h], terms.dIY[h] );
    sum.signGYGX[h] = vaddq_s16( sum.signGYGX[h], terms.signGYGX[h] );
  }
}

// the gradient differences of one row that the average needs
struct BdofGradDiff16
{
  int16_t x[16];   // gradX0 - gradX1
  int16_t y[16];   // gradY0 - gradY1
};

// a * signum( b ), as _mm_sign_epi16: 0 where b is 0
static inline int16x8_t bdofSign( const int16x8_t a, const int16x8_t b )
{
  return vmulq_s16( a, signum_neon( b ) );
}

// The terms of one row and its gradient differences: gradients as gradFilter_neon (differences of the neighbours
// >> 6), the terms as calcBIOSums_neon.
static inline BdofTerms16 bdofRowTerms16( const Pel* pred0, const Pel* pred1, const ptrdiff_t stride, BdofGradDiff16& gradDiff )
{
  BdofTerms16 terms;
  for( int h = 0; h < 2; h++ )
  {
    const Pel*      p0     = pred0 + 8 * h;
    const Pel*      p1     = pred1 + 8 * h;
    const int16x8_t left0  = vld1q_s16( p0 - 1 );
    const int16x8_t right0 = vld1q_s16( p0 + 1 );
    const int16x8_t above0 = vld1q_s16( p0 - stride );
    const int16x8_t below0 = vld1q_s16( p0 + stride );
    const int16x8_t left1  = vld1q_s16( p1 - 1 );
    const int16x8_t right1 = vld1q_s16( p1 + 1 );
    const int16x8_t above1 = vld1q_s16( p1 - stride );
    const int16x8_t below1 = vld1q_s16( p1 + stride );
    const int16x8_t s0     = vld1q_s16( p0 );
    const int16x8_t s1     = vld1q_s16( p1 );

    const int16x8_t gradX0 = vsubq_s16( vshrq_n_s16( right0, 6 ), vshrq_n_s16( left0, 6 ) );
    const int16x8_t gradY0 = vsubq_s16( vshrq_n_s16( below0, 6 ), vshrq_n_s16( above0, 6 ) );
    const int16x8_t gradX1 = vsubq_s16( vshrq_n_s16( right1, 6 ), vshrq_n_s16( left1, 6 ) );
    const int16x8_t gradY1 = vsubq_s16( vshrq_n_s16( below1, 6 ), vshrq_n_s16( above1, 6 ) );
    vst1q_s16( gradDiff.x + 8 * h, vsubq_s16( gradX0, gradX1 ) );
    vst1q_s16( gradDiff.y + 8 * h, vsubq_s16( gradY0, gradY1 ) );

    const int16x8_t gX = vhaddq_s16( gradX0, gradX1 );
    const int16x8_t gY = vhaddq_s16( gradY0, gradY1 );
    const int16x8_t dI = vsubq_s16( vshrq_n_s16( s1, 4 ), vshrq_n_s16( s0, 4 ) );
    terms.absGX[h]    = vabsq_s16( gX );
    terms.absGY[h]    = vabsq_s16( gY );
    terms.dIX[h]      = bdofSign( dI, gX );
    terms.dIY[h]      = bdofSign( dI, gY );
    terms.signGYGX[h] = bdofSign( gX, gY );
  }
  return terms;
}

// The window sums of the four subblocks of a subblock row (int32, lane k = subblock k) from the 16 column sums:
// subblock k covers columns 4k-1 .. 4k+4, with column -1 = column 0 and column 16 = column 15. With the extended row
// e = c0, c0, c1, .., c15, c15 and its pair sums p_j = e_2j + e_2j+1, the window of subblock k is p_2k + p_2k+1 +
// p_2k+2.
static inline int32x4_t bdofWindowSums16( const int16x8_t* v )
{
  const int16x8_t  first   = vextq_s16( vdupq_lane_s16( vget_low_s16( v[0] ), 0 ), v[0], 7 );   // c0, c0, c1 .. c6
  const int16x8_t  second  = vextq_s16( v[0], v[1], 7 );                                        // c7 .. c14
  const int32x4_t  pairs03 = vpaddlq_s16( first );                                              // p0 .. p3
  const int32x4_t  pairs47 = vpaddlq_s16( second );                                             // p4 .. p7
  const int32x4_t  p8      = vdupq_n_s32( 2 * vgetq_lane_s16( v[1], 7 ) );                       // 2 c15
  const int32x4x2_t evenOdd = vuzpq_s32( pairs03, pairs47 );   // p0, p2, p4, p6 | p1, p3, p5, p7
  const int32x4_t  next    = vextq_s32( evenOdd.val[0], p8, 1 );                                 // p2, p4, p6, p8
  return vaddq_s32( vaddq_s32( evenOdd.val[0], evenOdd.val[1] ), next );
}

// n >> floor( log2( d ) ) per lane, clipped to [-limit, limit], 0 where d is 0 (calcBIOSums_neon's rightShiftMSB)
static inline int32x4_t bdofRefine( const int32x4_t numer, const int32x4_t denom, const int32x4_t vLimit )
{
  const int32x4_t  msb    = vsubq_s32( vdupq_n_s32( 31 ), vclzq_s32( denom ) );
  const int32x4_t  shifted = vshlq_s32( numer, vnegq_s32( msb ) );
  const int32x4_t  clipped = vminq_s32( vLimit, vmaxq_s32( vnegq_s32( vLimit ), shifted ) );
  const uint32x4_t isZero  = vceqq_s32( denom, vdupq_n_s32( 0 ) );
  return vbicq_s32( clipped, vreinterpretq_s32_u32( isZero ) );
}

// the corrected average of 4 samples of subblock k (columns 4k .. 4k+3) of one row (addBIOAvg4_x2_neon's arithmetic)
template<int k>
static inline int16x4_t bdofAverage4( const Pel* pred0, const Pel* pred1, const BdofGradDiff16& gradDiff, const int16x4_t tmpx,
                                      const int16x4_t tmpy, const int32x4_t vOffset, const int32x4_t vShift,
                                      const int32x4_t vMin, const int32x4_t vMax )
{
  const int16x4_t gradDiffX = vld1_s16( gradDiff.x + 4 * k );
  const int16x4_t gradDiffY = vld1_s16( gradDiff.y + 4 * k );
  const int16x4_t src0      = vld1_s16( pred0 + 4 * k );
  const int16x4_t src1      = vld1_s16( pred1 + 4 * k );
  int32x4_t sum = vaddl_s16( src0, src1 );
  sum = vmlal_lane_s16( sum, gradDiffX, tmpx, k );
  sum = vmlal_lane_s16( sum, gradDiffY, tmpy, k );
  sum = vshlq_s32( vaddq_s32( sum, vOffset ), vShift );
  return vmovn_s32( vminq_s32( vMax, vmaxq_s32( vMin, sum ) ) );
}

// the corrected average of 4 rows; tmpx, tmpy: lane k = subblock k
static inline void bdofAverage4Rows16( const Pel* pred0, const Pel* pred1, const ptrdiff_t predStride, const BdofGradDiff16* gradDiff,
                                       Pel* dst, const ptrdiff_t dstStride, const int16x4_t tmpx, const int16x4_t tmpy,
                                       const int shiftNum, const int offset, const ClpRng& clpRng )
{
  const int32x4_t vOffset = vdupq_n_s32( offset );
  const int32x4_t vShift  = vdupq_n_s32( -shiftNum );
  const int32x4_t vMin    = vdupq_n_s32( clpRng.min() );
  const int32x4_t vMax    = vdupq_n_s32( clpRng.max() );

  for( int y = 0; y < 4; y++, pred0 += predStride, pred1 += predStride, gradDiff++, dst += dstStride )
  {
    const int16x4_t avg0 = bdofAverage4<0>( pred0, pred1, *gradDiff, tmpx, tmpy, vOffset, vShift, vMin, vMax );
    const int16x4_t avg1 = bdofAverage4<1>( pred0, pred1, *gradDiff, tmpx, tmpy, vOffset, vShift, vMin, vMax );
    const int16x4_t avg2 = bdofAverage4<2>( pred0, pred1, *gradDiff, tmpx, tmpy, vOffset, vShift, vMin, vMax );
    const int16x4_t avg3 = bdofAverage4<3>( pred0, pred1, *gradDiff, tmpx, tmpy, vOffset, vShift, vMin, vMax );
    vst1q_s16( dst, vcombine_s16( avg0, avg1 ) );
    vst1q_s16( dst + 8, vcombine_s16( avg2, avg3 ) );
  }
}

// BDOF of a 16-wide block (16x16 or 16x8) in one pass, the NEON version of BDOF16_AVX2 (InterPredX86.h, which
// explains why it equals the gradient filter + padding + BiOptFlow path): the gradients, the window sums of every 4x4
// subblock and the corrected average, without gradient planes or padded predictions. Each row's terms are computed
// once; a window (rows 4yu-1 .. 4yu+4) counts row 0 twice at the top and row H-1 twice at the bottom.
// pred0, pred1: the blocks' first samples in the unpadded predictions (one-sample border around them)
static void BDOF16_neon( const Pel* pred0, const Pel* pred1, const ptrdiff_t predStride, const int height, Pel* dst, const ptrdiff_t dstStride,
                         const int shiftNum, const int offset, const int limit, const ClpRng& clpRng )
{
  CHECKD( height != 8 && height != 16, "BDOF16_neon: height" );
  BdofGradDiff16 gradDiff[MAX_BDOF_APPLICATION_REGION];
  const int32x4_t vLimit = vdupq_n_s32( limit );

  // the sum of rows y0-1 and y0 of the next window; row -1 is a copy of row 0
  BdofTerms16 carry = bdofRowTerms16( pred0, pred1, predStride, gradDiff[0] );
  addBdofTerms16( carry, carry );

  for( int y0 = 0; y0 < height; y0 += 4 )
  {
    BdofTerms16 window = carry;
    for( int y = y0 + 1; y < y0 + 3; y++ )
    {
      addBdofTerms16( window, bdofRowTerms16( pred0 + y * predStride, pred1 + y * predStride, predStride, gradDiff[y] ) );
    }
    const BdofTerms16 lastRow = bdofRowTerms16( pred0 + ( y0 + 3 ) * predStride, pred1 + ( y0 + 3 ) * predStride, predStride, gradDiff[y0 + 3] );
    addBdofTerms16( window, lastRow );
    if( y0 + 4 < height )
    {
      const BdofTerms16 below = bdofRowTerms16( pred0 + ( y0 + 4 ) * predStride, pred1 + ( y0 + 4 ) * predStride, predStride, gradDiff[y0 + 4] );
      addBdofTerms16( window, below );
      carry = lastRow;
      addBdofTerms16( carry, below );
    }
    else
    {
      addBdofTerms16( window, lastRow );   // row H is a copy of row H-1
    }

    const int32x4_t sumAbsGX    = bdofWindowSums16( window.absGX );
    const int32x4_t sumAbsGY    = bdofWindowSums16( window.absGY );
    const int32x4_t sumDIX      = bdofWindowSums16( window.dIX );
    const int32x4_t sumDIY      = bdofWindowSums16( window.dIY );
    const int32x4_t sumSignGYGX = bdofWindowSums16( window.signGYGX );

    // calcBIOSums_neon's motion: tmpData = sumSignGYGX * tmpx >> 1 (|product| < 2^20: the split form there is equal)
    const int32x4_t tmpx    = bdofRefine( vshlq_n_s32( sumDIX, 2 ), sumAbsGX, vLimit );
    const int32x4_t tmpData = vshrq_n_s32( vmulq_s32( sumSignGYGX, tmpx ), 1 );
    const int32x4_t tmpy    = bdofRefine( vsubq_s32( vshlq_n_s32( sumDIY, 2 ), tmpData ), sumAbsGY, vLimit );

    bdofAverage4Rows16( pred0 + y0 * predStride, pred1 + y0 * predStride, predStride, gradDiff + y0, dst + y0 * dstStride, dstStride,
                        vmovn_s32( tmpx ), vmovn_s32( tmpy ), shiftNum, offset, clpRng );
  }
}

template<>
void InterPredInterpolation::_initInterPredictionARM<NEON>()
{
  xFpBiDirOptFlow = BiOptFlow_neon;
  xFpBDOF16 = BDOF16_neon;
  xFpBDOFGradFilter = gradFilter_neon;
  xFpProfGradFilter = gradFilter_neon<false>;
  xFpPadDmvr = padDmvr_neon;
}

} // namespace vvenc

#endif // ENABLE_SIMD_OPT_BDOF && defined( TARGET_SIMD_ARM )

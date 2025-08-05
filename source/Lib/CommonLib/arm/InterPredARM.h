/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
/** \file     InterPredX86.h
    \brief    SIMD for InterPrediction
*/

//! \ingroup CommonLib
//! \{

#include "CommonDefARM.h"
#include "InterPrediction.h"
#include "Rom.h"
#include "neon/sum_neon.h"

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if ENABLE_SIMD_OPT_BDOF && defined( TARGET_SIMD_ARM )

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
    int16x8_t shiftSrcY0Tmp = vshrq_n_s16( vld1q_s16( ( int16_t* )( srcY0Tmp ) ), 4 );
    int16x8_t shiftSrcY1Tmp = vshrq_n_s16( vld1q_s16( ( int16_t* )( srcY1Tmp ) ), 4 );

    int16x8_t loadGradX0 = vld1q_s16( ( int16_t* )( gradX0 + gradOfs ) );
    int16x8_t loadGradX1 = vld1q_s16( ( int16_t* )( gradX1 + gradOfs ) );
    int16x8_t loadGradY0 = vld1q_s16( ( int16_t* )( gradY0 + gradOfs ) );
    int16x8_t loadGradY1 = vld1q_s16( ( int16_t* )( gradY1 + gradOfs ) );
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

  int sumAbsGX = horizontal_add_s16x8( vandq_s16( sumAbsGXTmp, mask6 ) );
  int sumAbsGY = horizontal_add_s16x8( vandq_s16( sumAbsGYTmp, mask6 ) );
  int sumDIX = horizontal_add_s16x8( vandq_s16( sumDIXTmp, mask6 ) );
  int sumDIY = horizontal_add_s16x8( vandq_s16( sumDIYTmp, mask6 ) );
  int sumSignGY_GX = horizontal_add_s16x8( vandq_s16( sumSignGyGxTmp, mask6 ) );

  tmpx = sumAbsGX == 0 ? 0 : rightShiftMSB( sumDIX << 2, sumAbsGX );
  tmpx = Clip3( -limit, limit, tmpx );

  int mainsGxGy = sumSignGY_GX >> 12;
  int secsGxGy = sumSignGY_GX & ( ( 1 << 12 ) - 1 );
  int tmpData = tmpx * mainsGxGy;
  tmpData = ( ( tmpData << 12 ) + tmpx * secsGxGy ) >> 1;
  tmpy = sumAbsGY == 0 ? 0 : rightShiftMSB( ( ( sumDIY << 2 ) - tmpData ), sumAbsGY );
  tmpy = Clip3( -limit, limit, tmpy );
}

static inline void addBIOAvg4_neon( const int16_t* src0, const int16_t* src1, int16_t* dst, ptrdiff_t dstStride,
                                    const int16_t* gradX0, const int16_t* gradX1, const int16_t* gradY0,
                                    const int16_t* gradY1, int gradOfs, ptrdiff_t widthG, int tmpx, int tmpy, int shift,
                                    int offset, const ClpRng& clpRng )
{
  const ptrdiff_t srcStride = widthG + 2;
  const ptrdiff_t gradStride = widthG;
  const int32x4_t voffset = vdupq_n_s32( offset );
  const uint16x4_t vibdimax = vdup_n_u16( clpRng.max() );

  for( int y = 0; y < 4; y++ )
  {

    int16x4_t a = vsub_s16( vld1_s16( ( const int16_t* )( gradX0 + gradOfs ) ),
                            vld1_s16( ( const int16_t* )( gradX1 + gradOfs ) ) );
    int16x4_t b = vsub_s16( vld1_s16( ( const int16_t* )( gradY0 + gradOfs ) ),
                            vld1_s16( ( const int16_t* )( gradY1 + gradOfs ) ) );

    int16x4_t s0 = vld1_s16( ( const int16_t* )( src0 ) );
    int16x4_t s1 = vld1_s16( ( const int16_t* )( src1 ) );
    int32x4_t s01 = vaddl_s16( s0, s1 );

    int32x4_t sum = vaddq_s32( voffset, s01 );
    sum = vmlal_n_s16( sum, a, tmpx );
    sum = vmlal_n_s16( sum, b, tmpy );
    int16x4_t sum3 =
        vreinterpret_s16_u16( vmin_u16( vibdimax, vqmovun_s32( vshlq_s32( sum, vdupq_n_s32( -shift ) ) ) ) );

    vst1_s16( ( int16_t* )dst, sum3 );

    dst += dstStride;
    src0 += srcStride;
    src1 += srcStride;
    gradOfs += gradStride;
  }
}

template<ARM_VEXT vext>
void BiOptFlowCoreARMSIMD( const Pel* srcY0, const Pel* srcY1, const Pel* gradX0, const Pel* gradX1, const Pel* gradY0,
                           const Pel* gradY1, const int width, const int height, Pel* dstY, const ptrdiff_t dstStride,
                           const int shiftNum, const int offset, const int limit, const ClpRng& clpRng,
                           const int bitDepth )
{
  const int widthG = width + 2 * BDOF_EXTEND_SIZE;
  const int stridePredMC = widthG + 2;
  const int xUnit = width >> 2;
  const int yUnit = height >> 2;
  int offsetPos = widthG * BDOF_EXTEND_SIZE + BDOF_EXTEND_SIZE;

  for( int yu = 0; yu < yUnit; yu++ )
  {
    const Pel* srcY0Temp = srcY0;
    const Pel* srcY1Temp = srcY1;
    Pel* dstY0 = dstY;

    int OffPos = offsetPos;
    int OffPad = ( yu * widthG ) << 2;
    for( int xu = 0; xu < xUnit; xu++ )
    {
      int tmpx, tmpy;
      calcBIOSums_neon( srcY0Temp, srcY1Temp, gradX0, gradX1, gradY0, gradY1, OffPad, widthG, bitDepth, limit, tmpx,
                        tmpy );
      addBIOAvg4_neon( srcY0Temp + stridePredMC + 1, srcY1Temp + stridePredMC + 1, dstY0, dstStride, gradX0, gradX1,
                       gradY0, gradY1, OffPos, widthG, tmpx, tmpy, shiftNum, offset, clpRng );

      srcY0Temp += 4;
      srcY1Temp += 4;
      dstY0 += 4;
      OffPos += 4;
      OffPad += 4;
    }

    srcY0 += stridePredMC << 2;
    srcY1 += stridePredMC << 2;
    dstY += dstStride << 2;
    offsetPos += widthG << 2;
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

template<ARM_VEXT vext>
void InterPredInterpolation::_initInterPredictionARM()
{
  xFpBiDirOptFlow = BiOptFlowCoreARMSIMD<vext>;
  xFpBDOFGradFilter = gradFilter_neon;
  xFpProfGradFilter = gradFilter_neon<false>;
}

template void InterPredInterpolation::_initInterPredictionARM<SIMDARM>();

#endif
} // namespace vvenc

//! \}

// #endif // TARGET_SIMD_X86
//! \}

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

/** \file     SampleAdaptiveOffset_neon.cpp
    \brief    Sample adaptive offset class, Neon versions of calcSaoStatisticsBo and the calcSaoStatisticsEo* family.
*/

#include "../CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "sum_neon.h"
#include <arm_neon.h>

namespace vvenc
{

#if ENABLE_SIMD_OPT_SAO && defined( TARGET_SIMD_ARM )

void calcSaoStatisticsBo_neon( int width, int endX, int endY, Pel* srcLine, Pel* orgLine, int srcStride, int orgStride,
                               int channelBitDepth, int64_t* count, int64_t* diff )
{
  const int shift = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;

  int32_t countTmp[2][32] = { { 0 }, { 0 } };
  int32_t diffTmp[2][32] = { { 0 }, { 0 } };

  do
  {
    for( int x = 0; x < endX; ++x )
    {
      int idx = x & 1;
      int bandIdx = srcLine[x] >> shift;
      diffTmp[idx][bandIdx] += orgLine[x] - srcLine[x];
      countTmp[idx][bandIdx]++;
    }

    srcLine += srcStride;
    orgLine += orgStride;
  } while( --endY != 0 );

  for( int i = 0; i < 32; i += 4 )
  {
    int32x4_t c0 = vld1q_s32( countTmp[0] + i );
    int32x4_t c1 = vld1q_s32( countTmp[1] + i );
    int32x4_t d0 = vld1q_s32( diffTmp[0] + i );
    int32x4_t d1 = vld1q_s32( diffTmp[1] + i );

    int64x2_t cTotal_lo = vaddl_s32( vget_low_s32( c0 ), vget_low_s32( c1 ) );
    int64x2_t cTotal_hi = vaddl_s32( vget_high_s32( c0 ), vget_high_s32( c1 ) );
    int64x2_t dTotal_lo = vaddl_s32( vget_low_s32( d0 ), vget_low_s32( d1 ) );
    int64x2_t dTotal_hi = vaddl_s32( vget_high_s32( d0 ), vget_high_s32( d1 ) );

    vst1q_s64( count + i + 0, cTotal_lo );
    vst1q_s64( count + i + 2, cTotal_hi );
    vst1q_s64( diff + i + 0, dTotal_lo );
    vst1q_s64( diff + i + 2, dTotal_hi );
  }
}

// sign(a - b) without materializing the subtraction
static inline int16x8_t sign_s16( const int16x8_t a, const int16x8_t b )
{
  // vcgtq_s16 returns all-ones (-1) on true, so the difference needs this order to land on +1/-1
  const int16x8_t isGreater = vreinterpretq_s16_u16( vcgtq_s16( a, b ) );
  const int16x8_t isLess    = vreinterpretq_s16_u16( vcgtq_s16( b, a ) );
  return vsubq_s16( isLess, isGreater );
}

// right-edge exclusion mask for 0..8 samples
static const int16_t kEoRightMaskTbl[16] = { 0, 0, 0, 0, 0, 0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1 };

void calcSaoStatisticsEo0_neon( int width, int startX, int endX, int endY, Pel* srcLine, Pel* orgLine, int srcStride,
                                int orgStride, int64_t* count, int64_t* diff )
{
  if( width % 16 != 0 )
  {
    const int naWidth = startX + ( width - endX );
    srcLine += startX;
    orgLine += startX;
    diff += 2;
    count += 2;

    for( int y = 0; y < endY; y++ )
    {
      int signLeft = sgn( srcLine[0] - srcLine[-1] );
      for( int x = 0; x < width - naWidth; x++, srcLine++, orgLine++ )
      {
        int signRight = sgn( srcLine[0] - srcLine[1] );
        int type      = signLeft + signRight;
        signLeft      = -signRight;
        diff[type]  += *orgLine - *srcLine;
        count[type] += 1;
      }
      srcLine += srcStride - ( width - naWidth );
      orgLine += orgStride - ( width - naWidth );
    }
    return;
  }

  const int16x8_t vrightMask = vld1q_s16( kEoRightMaskTbl + ( width - endX ) );
  const int16x8_t vleftMask  = startX ? vsetq_lane_s16( -1, vdupq_n_s16( 0 ), 0 ) : vdupq_n_s16( 0 );

  int32x4_t vdiffSum[NUM_SAO_EO_CLASSES];
  int32x4_t vcountSum[NUM_SAO_EO_CLASSES];
  for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
  {
    vdiffSum[i]  = vdupq_n_s32( 0 );
    vcountSum[i] = vdupq_n_s32( 0 );
  }

  for( int y = 0; y < endY; y++ )
  {
    int16x8_t vmaskFirst = vleftMask;

    for( int x = 0; x < width; x += 16 )
    {
      const int16x8_t vmaskLast = x >= width - 16 ? vrightMask : vdupq_n_s16( 0 );

      const int16x8_t vcurA = vld1q_s16( srcLine + x );
      const int16x8_t vcurB = vld1q_s16( srcLine + x + 8 );
      const int16x8_t vorgA = vld1q_s16( orgLine + x );
      const int16x8_t vorgB = vld1q_s16( orgLine + x + 8 );

      const int16x8_t vleftA  = vld1q_s16( srcLine + x - 1 );
      const int16x8_t vrightA = vld1q_s16( srcLine + x + 1 );
      const int16x8_t vleftB  = vld1q_s16( srcLine + x + 8 - 1 );
      const int16x8_t vrightB = vld1q_s16( srcLine + x + 8 + 1 );

      int16x8_t vtypeA = vaddq_s16( sign_s16( vcurA, vleftA ), sign_s16( vcurA, vrightA ) );
      int16x8_t vtypeB = vaddq_s16( sign_s16( vcurB, vleftB ), sign_s16( vcurB, vrightB ) );
      vtypeA = vorrq_s16( vaddq_s16( vtypeA, vdupq_n_s16( 2 ) ), vmaskFirst );
      vtypeB = vorrq_s16( vaddq_s16( vtypeB, vdupq_n_s16( 2 ) ), vmaskLast );

      const int16x8_t vdiffA = vsubq_s16( vorgA, vcurA );
      const int16x8_t vdiffB = vsubq_s16( vorgB, vcurB );

      for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
      {
        const int16x8_t vbin   = vdupq_n_s16( i );
        const uint16x8_t vcmpA = vceqq_s16( vtypeA, vbin );
        const uint16x8_t vcmpB = vceqq_s16( vtypeB, vbin );

        vdiffSum[i] = vpadalq_s16( vdiffSum[i], vandq_s16( vreinterpretq_s16_u16( vcmpA ), vdiffA ) );
        vdiffSum[i] = vpadalq_s16( vdiffSum[i], vandq_s16( vreinterpretq_s16_u16( vcmpB ), vdiffB ) );

        // vcmpA/B are all-ones (-1) on a match, so this sum is the negated count; negate once at the end
        vcountSum[i] = vpadalq_s16( vcountSum[i], vreinterpretq_s16_u16( vcmpA ) );
        vcountSum[i] = vpadalq_s16( vcountSum[i], vreinterpretq_s16_u16( vcmpB ) );
      }

      vmaskFirst = vdupq_n_s16( 0 );
    }

    srcLine += srcStride;
    orgLine += orgStride;
  }

  for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
  {
    diff[i]  = horizontal_add_s32x4( vdiffSum[i] );
    count[i] = -horizontal_add_s32x4( vcountSum[i] );
  }
}

void calcSaoStatisticsEo90_neon( int width, int endX, int startY, int endY, Pel* srcLine, Pel* orgLine, int srcStride,
                                  int orgStride, int64_t* count, int64_t* diff, int8_t* signUpLine )
{
  if( width % 16 != 0 )
  {
    diff += 2;
    count += 2;

    const Pel* srcLineAbove = srcLine - srcStride;
    for( int x = 0; x < endX; x++ )
    {
      signUpLine[x] = (int8_t)sgn( srcLine[x] - srcLineAbove[x] );
    }

    for( int y = startY; y < endY; y++ )
    {
      const Pel* srcLineBelow = srcLine + srcStride;
      for( int x = 0; x < endX; x++ )
      {
        int8_t signDown = (int8_t)sgn( srcLine[x] - srcLineBelow[x] );
        int    type     = signDown + signUpLine[x];
        signUpLine[x]   = -signDown;
        diff[type]  += orgLine[x] - srcLine[x];
        count[type] += 1;
      }
      srcLine += srcStride;
      orgLine += orgStride;
    }
    return;
  }

  const int16x8_t vrightMask = vld1q_s16( kEoRightMaskTbl + ( width - endX ) );

  // carry sign(above - cur) between rows; subtract it from sign(cur - below)
  int16x8_t vsignAboveA[MAX_CU_SIZE / 16];
  int16x8_t vsignAboveB[MAX_CU_SIZE / 16];
  {
    const Pel* srcLineAbove = srcLine - srcStride;
    for( int x = 0; x < width; x += 16 )
    {
      vsignAboveA[x / 16] = sign_s16( vld1q_s16( srcLineAbove + x ), vld1q_s16( srcLine + x ) );
      vsignAboveB[x / 16] = sign_s16( vld1q_s16( srcLineAbove + x + 8 ), vld1q_s16( srcLine + x + 8 ) );
    }
  }

  int32x4_t vdiffSum[NUM_SAO_EO_CLASSES];
  int32x4_t vcountSum[NUM_SAO_EO_CLASSES];
  for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
  {
    vdiffSum[i]  = vdupq_n_s32( 0 );
    vcountSum[i] = vdupq_n_s32( 0 );
  }

  for( int y = startY; y < endY; y++ )
  {
    for( int x = 0; x < width; x += 16 )
    {
      const int16x8_t vmaskLast = x >= width - 16 ? vrightMask : vdupq_n_s16( 0 );

      const int16x8_t vcurA = vld1q_s16( srcLine + x );
      const int16x8_t vcurB = vld1q_s16( srcLine + x + 8 );
      const int16x8_t vorgA = vld1q_s16( orgLine + x );
      const int16x8_t vorgB = vld1q_s16( orgLine + x + 8 );

      const int16x8_t vsignDownA = sign_s16( vcurA, vld1q_s16( srcLine + x + srcStride ) );
      const int16x8_t vsignDownB = sign_s16( vcurB, vld1q_s16( srcLine + x + 8 + srcStride ) );

      int16x8_t vtypeA = vaddq_s16( vsubq_s16( vsignDownA, vsignAboveA[x / 16] ), vdupq_n_s16( 2 ) );
      int16x8_t vtypeB = vorrq_s16( vaddq_s16( vsubq_s16( vsignDownB, vsignAboveB[x / 16] ), vdupq_n_s16( 2 ) ),
                                     vmaskLast );

      vsignAboveA[x / 16] = vsignDownA;
      vsignAboveB[x / 16] = vsignDownB;

      const int16x8_t vdiffA = vsubq_s16( vorgA, vcurA );
      const int16x8_t vdiffB = vsubq_s16( vorgB, vcurB );

      for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
      {
        const int16x8_t vbin   = vdupq_n_s16( i );
        const uint16x8_t vcmpA = vceqq_s16( vtypeA, vbin );
        const uint16x8_t vcmpB = vceqq_s16( vtypeB, vbin );

        vdiffSum[i] = vpadalq_s16( vdiffSum[i], vandq_s16( vreinterpretq_s16_u16( vcmpA ), vdiffA ) );
        vdiffSum[i] = vpadalq_s16( vdiffSum[i], vandq_s16( vreinterpretq_s16_u16( vcmpB ), vdiffB ) );

        // vcmpA/B are all-ones (-1) on a match, so this sum is the negated count; negate once at the end
        vcountSum[i] = vpadalq_s16( vcountSum[i], vreinterpretq_s16_u16( vcmpA ) );
        vcountSum[i] = vpadalq_s16( vcountSum[i], vreinterpretq_s16_u16( vcmpB ) );
      }
    }

    srcLine += srcStride;
    orgLine += orgStride;
  }

  for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
  {
    diff[i]  = horizontal_add_s32x4( vdiffSum[i] );
    count[i] = -horizontal_add_s32x4( vcountSum[i] );
  }
}

void calcSaoStatisticsEo135_neon( int width, int startX, int endX, int endY, Pel* srcLine, Pel* orgLine,
                                   int srcStride, int orgStride, int64_t* count, int64_t* diff, int8_t* signUpLine,
                                   int8_t* signDownLine )
{
  if( width % 16 != 0 )
  {
    for( int y = 1; y < endY; y++ )
    {
      const Pel* srcLineBelow = srcLine + srcStride;
      int8_t tmpSign = (int8_t)sgn( srcLineBelow[startX] - srcLine[startX - 1] );
      for( int x = startX; x < endX; x++ )
      {
        int8_t signDown = (int8_t)sgn( srcLine[x] - srcLineBelow[x + 1] );
        int    type     = signDown + signUpLine[x];
        signUpLine[x]   = tmpSign;
        tmpSign         = -signDown;
        diff[type]  += orgLine[x] - srcLine[x];
        count[type] += 1;
      }
      srcLine += srcStride;
      orgLine += orgStride;
    }
    return;
  }

  // the caller passes bin-2 pointers and has already accumulated the first row
  diff -= 2;
  count -= 2;

  const int16x8_t vrightMask = vld1q_s16( kEoRightMaskTbl + ( width - endX ) );
  const int16x8_t vleftMask  = startX ? vsetq_lane_s16( -1, vdupq_n_s16( 0 ), 0 ) : vdupq_n_s16( 0 );

  int32x4_t vdiffSum[NUM_SAO_EO_CLASSES];
  int32x4_t vcountSum[NUM_SAO_EO_CLASSES];
  for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
  {
    vdiffSum[i]  = vdupq_n_s32( 0 );
    vcountSum[i] = vdupq_n_s32( 0 );
  }

  for( int y = 1; y < endY; y++ )
  {
    int16x8_t vmaskFirst = vleftMask;

    for( int x = 0; x < width; x += 16 )
    {
      const int16x8_t vmaskLast = x >= width - 16 ? vrightMask : vdupq_n_s16( 0 );

      const int16x8_t vcurA = vld1q_s16( srcLine + x );
      const int16x8_t vcurB = vld1q_s16( srcLine + x + 8 );
      const int16x8_t vorgA = vld1q_s16( orgLine + x );
      const int16x8_t vorgB = vld1q_s16( orgLine + x + 8 );

      // type = sign(cur - aboveLeft) + sign(cur - belowRight) + 2
      const int16x8_t vaboveLeftA  = vld1q_s16( srcLine + x - 1 - srcStride );
      const int16x8_t vbelowRightA = vld1q_s16( srcLine + x + 1 + srcStride );
      const int16x8_t vaboveLeftB  = vld1q_s16( srcLine + x + 8 - 1 - srcStride );
      const int16x8_t vbelowRightB = vld1q_s16( srcLine + x + 8 + 1 + srcStride );

      int16x8_t vtypeA = vaddq_s16( sign_s16( vcurA, vaboveLeftA ), sign_s16( vcurA, vbelowRightA ) );
      int16x8_t vtypeB = vaddq_s16( sign_s16( vcurB, vaboveLeftB ), sign_s16( vcurB, vbelowRightB ) );
      vtypeA = vorrq_s16( vaddq_s16( vtypeA, vdupq_n_s16( 2 ) ), vmaskFirst );
      vtypeB = vorrq_s16( vaddq_s16( vtypeB, vdupq_n_s16( 2 ) ), vmaskLast );

      const int16x8_t vdiffA = vsubq_s16( vorgA, vcurA );
      const int16x8_t vdiffB = vsubq_s16( vorgB, vcurB );

      for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
      {
        const int16x8_t vbin   = vdupq_n_s16( i );
        const uint16x8_t vcmpA = vceqq_s16( vtypeA, vbin );
        const uint16x8_t vcmpB = vceqq_s16( vtypeB, vbin );

        vdiffSum[i] = vpadalq_s16( vdiffSum[i], vandq_s16( vreinterpretq_s16_u16( vcmpA ), vdiffA ) );
        vdiffSum[i] = vpadalq_s16( vdiffSum[i], vandq_s16( vreinterpretq_s16_u16( vcmpB ), vdiffB ) );

        // vcmpA/B are all-ones (-1) on a match, so this sum is the negated count; negate once at the end
        vcountSum[i] = vpadalq_s16( vcountSum[i], vreinterpretq_s16_u16( vcmpA ) );
        vcountSum[i] = vpadalq_s16( vcountSum[i], vreinterpretq_s16_u16( vcmpB ) );
      }

      vmaskFirst = vdupq_n_s16( 0 );
    }

    srcLine += srcStride;
    orgLine += orgStride;
  }

  for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
  {
    diff[i]  += horizontal_add_s32x4( vdiffSum[i] );
    count[i] -= horizontal_add_s32x4( vcountSum[i] );
  }
}

void calcSaoStatisticsEo45_neon( int width, int startX, int endX, int endY, Pel* srcLine, Pel* orgLine, int srcStride,
                                  int orgStride, int64_t* count, int64_t* diff, int8_t* signUpLine )
{
  if( width % 16 != 0 )
  {
    for( int y = 1; y < endY; y++ )
    {
      const Pel* srcLineBelow = srcLine + srcStride;
      for( int x = startX; x < endX; x++ )
      {
        int8_t signDown = (int8_t)sgn( srcLine[x] - srcLineBelow[x - 1] );
        int    type     = signDown + signUpLine[x];
        diff[type]  += orgLine[x] - srcLine[x];
        count[type] += 1;
        signUpLine[x - 1] = -signDown;
      }
      signUpLine[endX - 1] = (int8_t)sgn( srcLineBelow[endX - 1] - srcLine[endX] );
      srcLine += srcStride;
      orgLine += orgStride;
    }
    return;
  }

  // the caller passes bin-2 pointers and has already accumulated the first row
  diff -= 2;
  count -= 2;

  const int16x8_t vrightMask = vld1q_s16( kEoRightMaskTbl + ( width - endX ) );
  const int16x8_t vleftMask  = startX ? vsetq_lane_s16( -1, vdupq_n_s16( 0 ), 0 ) : vdupq_n_s16( 0 );

  int32x4_t vdiffSum[NUM_SAO_EO_CLASSES];
  int32x4_t vcountSum[NUM_SAO_EO_CLASSES];
  for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
  {
    vdiffSum[i]  = vdupq_n_s32( 0 );
    vcountSum[i] = vdupq_n_s32( 0 );
  }

  for( int y = 1; y < endY; y++ )
  {
    int16x8_t vmaskFirst = vleftMask;

    for( int x = 0; x < width; x += 16 )
    {
      const int16x8_t vmaskLast = x >= width - 16 ? vrightMask : vdupq_n_s16( 0 );

      const int16x8_t vcurA = vld1q_s16( srcLine + x );
      const int16x8_t vcurB = vld1q_s16( srcLine + x + 8 );
      const int16x8_t vorgA = vld1q_s16( orgLine + x );
      const int16x8_t vorgB = vld1q_s16( orgLine + x + 8 );

      // type = sign(cur - aboveRight) + sign(cur - belowLeft) + 2
      const int16x8_t vaboveRightA = vld1q_s16( srcLine + x - srcStride + 1 );
      const int16x8_t vbelowLeftA  = vld1q_s16( srcLine + x + srcStride - 1 );
      const int16x8_t vaboveRightB = vld1q_s16( srcLine + x + 8 - srcStride + 1 );
      const int16x8_t vbelowLeftB  = vld1q_s16( srcLine + x + 8 + srcStride - 1 );

      int16x8_t vtypeA = vaddq_s16( sign_s16( vcurA, vaboveRightA ), sign_s16( vcurA, vbelowLeftA ) );
      int16x8_t vtypeB = vaddq_s16( sign_s16( vcurB, vaboveRightB ), sign_s16( vcurB, vbelowLeftB ) );
      vtypeA = vorrq_s16( vaddq_s16( vtypeA, vdupq_n_s16( 2 ) ), vmaskFirst );
      vtypeB = vorrq_s16( vaddq_s16( vtypeB, vdupq_n_s16( 2 ) ), vmaskLast );

      const int16x8_t vdiffA = vsubq_s16( vorgA, vcurA );
      const int16x8_t vdiffB = vsubq_s16( vorgB, vcurB );

      for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
      {
        const int16x8_t vbin   = vdupq_n_s16( i );
        const uint16x8_t vcmpA = vceqq_s16( vtypeA, vbin );
        const uint16x8_t vcmpB = vceqq_s16( vtypeB, vbin );

        vdiffSum[i] = vpadalq_s16( vdiffSum[i], vandq_s16( vreinterpretq_s16_u16( vcmpA ), vdiffA ) );
        vdiffSum[i] = vpadalq_s16( vdiffSum[i], vandq_s16( vreinterpretq_s16_u16( vcmpB ), vdiffB ) );

        // vcmpA/B are all-ones (-1) on a match, so this sum is the negated count; negate once at the end
        vcountSum[i] = vpadalq_s16( vcountSum[i], vreinterpretq_s16_u16( vcmpA ) );
        vcountSum[i] = vpadalq_s16( vcountSum[i], vreinterpretq_s16_u16( vcmpB ) );
      }

      vmaskFirst = vdupq_n_s16( 0 );
    }

    srcLine += srcStride;
    orgLine += orgStride;
  }

  for( int i = 0; i < NUM_SAO_EO_CLASSES; i++ )
  {
    diff[i]  += horizontal_add_s32x4( vdiffSum[i] );
    count[i] -= horizontal_add_s32x4( vcountSum[i] );
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// offsetBlock_core: every edge offset class is, per sample, edgeType = sign( c - a ) + sign( c - b ) with a and b its
// two neighbours along the class direction (the C code's sign lines only keep the previous row's signs), so each row is
// one run over the C code's x range. The offsets of a block are an 8-entry int8 table (exact: SAO offsets are at most
// 31, scaled by 2^(bitDepth - 10) above 10 bits, so at most 124 up to 12 bits; the encoder takes 8 or 10 bits): edge
// offset entry edgeType + 2, band offset entry ( band - startIdx ) mod 32 for the four signalled bands (entries 4 to 7
// zero; vtbl gives 0 for 8 and more). Sample + offset fits int16 without saturation.
// ---------------------------------------------------------------------------------------------------------------------

static inline int8x8_t saoOffsets_neon( const uint8x8_t table, const uint16x8_t idx )
{
  return vreinterpret_s8_u8( vtbl1_u8( table, vmovn_u16( idx ) ) );
}

static inline int16x8_t saoApply_neon( const int16x8_t c, const int8x8_t offset, const int16x8_t vMax )
{
  return vminq_s16( vmaxq_s16( vaddw_s8( c, offset ), vdupq_n_s16( 0 ) ), vMax );
}

// one row of an edge offset class over x in [xStart, xEnd): a at src + x + aOff, b at src + x + bOff. Runs of 8 and
// more in 8-sample steps, the last overlapping the one before (src and res are separate buffers), shorter runs scalar.
static inline void saoEoRun_neon( const Pel* src, Pel* res, const ptrdiff_t aOff, const ptrdiff_t bOff,
                                  const int xStart, const int xEnd, const uint8x8_t table, const int* offsets,
                                  const int16x8_t vMax, const ClpRng& clpRng )
{
  if( xEnd - xStart < 8 )
  {
    for( int x = xStart; x < xEnd; x++ )
    {
      const int edgeType = sgn( src[x] - src[x + aOff] ) + sgn( src[x] - src[x + bOff] );
      res[x]             = ClipPel<int>( src[x] + offsets[edgeType + 2], clpRng );
    }
    return;
  }
  auto step = [&]( const int x )
  {
    const int16x8_t c   = vld1q_s16( src + x );
    const int16x8_t a   = vld1q_s16( src + x + aOff );
    const int16x8_t b   = vld1q_s16( src + x + bOff );
    const int16x8_t sum = vaddq_s16( sign_s16( c, a ), sign_s16( c, b ) );
    const uint16x8_t idx = vreinterpretq_u16_s16( vaddq_s16( sum, vdupq_n_s16( 2 ) ) );
    vst1q_s16( res + x, saoApply_neon( c, saoOffsets_neon( table, idx ), vMax ) );
  };
  int x = xStart;
  for( ; x + 8 <= xEnd; x += 8 )
  {
    step( x );
  }
  if( x < xEnd )
  {
    step( xEnd - 8 );
  }
}

static void offsetBlock_neon( const int channelBitDepth, const ClpRng& clpRng, int typeIdx, int* offset, int startIdx,
                              const Pel* srcBlk, Pel* resBlk, ptrdiff_t srcStride, ptrdiff_t resStride, int width,
                              int height, uint8_t availMask, std::vector<int8_t>& /*signLineBuf1*/,
                              std::vector<int8_t>& /*signLineBuf2*/ )
{
  CHECKD( srcBlk == resBlk, "SAO works from a copy of the samples" );
  const int16x8_t vMax = vdupq_n_s16( clpRng.max() );
  int8_t          table[8] = { 0 };

  if( typeIdx == SAO_TYPE_BO )
  {
    // the vectors read only the four bands from startIdx: the other bands' offsets are 0 (invertQuantOffsets), as the
    // SSE kernel assumes too; the scalar rows read the whole table, as the C code
    for( int k = 0; k < 4; k++ )
    {
      const int bandOffset = offset[( startIdx + k ) % MAX_NUM_SAO_CLASSES];
      CHECKD( bandOffset != int8_t( bandOffset ), "SAO offsets fit 8 bits" );
      table[k] = int8_t( bandOffset );
    }
    const int        shiftBits = channelBitDepth - NUM_SAO_BO_CLASSES_LOG2;
    const uint8x8_t  vTable    = vreinterpret_u8_s8( vld1_s8( table ) );
    const int16x8_t  vShift    = vdupq_n_s16( -shiftBits );
    const uint16x8_t vStart    = vdupq_n_u16( uint16_t( startIdx ) );
    const uint16x8_t vBandMask = vdupq_n_u16( MAX_NUM_SAO_CLASSES - 1 );
    auto step = [&]( const Pel* src, Pel* res, const int x )
    {
      const int16x8_t  c    = vld1q_s16( src + x );
      const uint16x8_t band = vreinterpretq_u16_s16( vshlq_s16( c, vShift ) );
      const uint16x8_t idx  = vandq_u16( vsubq_u16( band, vStart ), vBandMask );
      vst1q_s16( res + x, saoApply_neon( c, saoOffsets_neon( vTable, idx ), vMax ) );
    };
    for( int y = 0; y < height; y++, srcBlk += srcStride, resBlk += resStride )
    {
      if( width < 8 )
      {
        for( int x = 0; x < width; x++ )
        {
          resBlk[x] = ClipPel<int>( srcBlk[x] + offset[srcBlk[x] >> shiftBits], clpRng );
        }
        continue;
      }
      int x = 0;
      for( ; x + 8 <= width; x += 8 )
      {
        step( srcBlk, resBlk, x );
      }
      if( x < width )
      {
        step( srcBlk, resBlk, width - 8 );
      }
    }
    return;
  }

  for( int k = 0; k < 5; k++ )
  {
    CHECKD( offset[k] != int8_t( offset[k] ), "SAO offsets fit 8 bits" );
    table[k] = int8_t( offset[k] );
  }
  const uint8x8_t vTable = vreinterpret_u8_s8( vld1_s8( table ) );
  const int       startX = availMask & LeftAvail ? 0 : 1;
  const int       endX   = availMask & RightAvail ? width : width - 1;
  auto row = [&]( const int y, const ptrdiff_t aOff, const ptrdiff_t bOff, const int xStart, const int xEnd )
  {
    saoEoRun_neon( srcBlk + y * srcStride, resBlk + y * resStride, aOff, bOff, xStart, xEnd, vTable, offset, vMax,
                   clpRng );
  };

  switch( typeIdx )
  {
  case SAO_TYPE_EO_0:
    for( int y = 0; y < height; y++ )
    {
      row( y, -1, 1, startX, endX );
    }
    break;
  case SAO_TYPE_EO_90:
  {
    const int startY = availMask & AboveAvail ? 0 : 1;
    const int endY   = availMask & BelowAvail ? height : height - 1;
    for( int y = startY; y < endY; y++ )
    {
      row( y, -srcStride, srcStride, 0, width );
    }
    break;
  }
  case SAO_TYPE_EO_135:
    // above-left and below-right; the first and last rows have their own ranges (the corner availabilities)
    row( 0, -srcStride - 1, srcStride + 1, availMask & AboveLeftAvail ? 0 : 1, availMask & AboveAvail ? endX : 1 );
    for( int y = 1; y < height - 1; y++ )
    {
      row( y, -srcStride - 1, srcStride + 1, startX, endX );
    }
    row( height - 1, -srcStride - 1, srcStride + 1, availMask & BelowAvail ? startX : width - 1,
         availMask & BelowRightAvail ? width : width - 1 );
    break;
  case SAO_TYPE_EO_45:
    // above-right and below-left
    row( 0, -srcStride + 1, srcStride - 1, availMask & AboveAvail ? startX : width - 1,
         availMask & AboveRightAvail ? width : width - 1 );
    for( int y = 1; y < height - 1; y++ )
    {
      row( y, -srcStride + 1, srcStride - 1, startX, endX );
    }
    row( height - 1, -srcStride + 1, srcStride - 1, availMask & BelowLeftAvail ? 0 : 1,
         availMask & BelowAvail ? endX : 1 );
    break;
  default:
    THROW( "Not a supported SAO type" );
  }
}

template<>
void SampleAdaptiveOffset::_initSampleAdaptiveOffsetARM<NEON>()
{
  calcSaoStatisticsBo    = calcSaoStatisticsBo_neon;
  calcSaoStatisticsEo0   = calcSaoStatisticsEo0_neon;
  calcSaoStatisticsEo90  = calcSaoStatisticsEo90_neon;
  calcSaoStatisticsEo135 = calcSaoStatisticsEo135_neon;
  calcSaoStatisticsEo45  = calcSaoStatisticsEo45_neon;
  offsetBlock            = offsetBlock_neon;
}

#endif // ENABLE_SIMD_OPT_SAO && defined( TARGET_SIMD_ARM )

} // namespace vvenc

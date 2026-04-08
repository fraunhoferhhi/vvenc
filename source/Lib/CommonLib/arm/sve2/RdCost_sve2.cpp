/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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

/** \file     RdCost_sve2.cpp
    \brief    RD cost computation class, SVE version
*/

#include <arm_neon.h>
#include <math.h>

#include "CommonLib/CommonDef.h"
#include "CommonLib/RdCost.h"
#include "neon/RdCost_neon.h"
#include "neon/sum_neon.h"
#include "neon_sve2_bridge.h"
#include "sve/neon_sve_bridge.h"

#include <arm_sve.h>

namespace vvenc
{

#if ENABLE_SIMD_OPT_DIST && defined( TARGET_SIMD_ARM )

static const uint16_t kZip1DTbl[] = { 0, 2, 4, 6, 1, 3, 5, 7 };

Distortion xCalcHAD16x16_fast_sve2( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                                    const ptrdiff_t iStrideCur )
{
  int16x8_t m1[8], m2[8];

  for( int i = 0; i < 8; i++ )
  {
    int16x8_t r0 = vld1q_s16( piOrg );
    int16x8_t r1 = vld1q_s16( piCur );
    int16x8_t r2 = vld1q_s16( piOrg + iStrideOrg );
    int16x8_t r3 = vld1q_s16( piCur + iStrideCur );

    r0 = vaddq_s16( r0, r2 );
    r1 = vaddq_s16( r1, r3 );

    r2 = vld1q_s16( piOrg + 8 );
    r3 = vld1q_s16( piCur + 8 );

    int16x8_t r4 = vld1q_s16( piOrg + iStrideOrg + 8 );
    int16x8_t r5 = vld1q_s16( piCur + iStrideCur + 8 );

    r2 = vaddq_s16( r2, r4 );
    r3 = vaddq_s16( r3, r5 );

    r0 = pairwise_add_s16x8( r0, r2 );
    r1 = pairwise_add_s16x8( r1, r3 );

    r0 = vrshrq_n_s16( r0, 2 );
    r1 = vrshrq_n_s16( r1, 2 );

    m1[i] = vsubq_s16( r0, r1 ); // 11-bit

    piCur += iStrideCur * 2;
    piOrg += iStrideOrg * 2;
  }

  const uint16x8_t idxs = vld1q_u16( kZip1DTbl );

  // Horizontal.
  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] ); // 12-bit

  m1[0] = vvenc_svtbl_s16( m2[0], idxs );
  m1[1] = vvenc_svtbl_s16( m2[1], idxs );
  m1[2] = vvenc_svtbl_s16( m2[2], idxs );
  m1[3] = vvenc_svtbl_s16( m2[3], idxs );
  m1[4] = vvenc_svtbl_s16( m2[4], idxs );
  m1[5] = vvenc_svtbl_s16( m2[5], idxs );
  m1[6] = vvenc_svtbl_s16( m2[6], idxs );
  m1[7] = vvenc_svtbl_s16( m2[7], idxs );

  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] ); // 13-bit

  m1[0] = vvenc_svtbl_s16( m2[0], idxs );
  m1[1] = vvenc_svtbl_s16( m2[1], idxs );
  m1[2] = vvenc_svtbl_s16( m2[2], idxs );
  m1[3] = vvenc_svtbl_s16( m2[3], idxs );
  m1[4] = vvenc_svtbl_s16( m2[4], idxs );
  m1[5] = vvenc_svtbl_s16( m2[5], idxs );
  m1[6] = vvenc_svtbl_s16( m2[6], idxs );
  m1[7] = vvenc_svtbl_s16( m2[7], idxs );

  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] ); // 14-bit

  // Vertical.
  m1[0] = vaddq_s16( m2[0], m2[4] );
  m1[1] = vaddq_s16( m2[1], m2[5] );
  m1[2] = vaddq_s16( m2[2], m2[6] );
  m1[3] = vaddq_s16( m2[3], m2[7] );
  m1[4] = vsubq_s16( m2[0], m2[4] );
  m1[5] = vsubq_s16( m2[1], m2[5] );
  m1[6] = vsubq_s16( m2[2], m2[6] );
  m1[7] = vsubq_s16( m2[3], m2[7] ); // 15-bit

  // The last butterfly uses |x+y|+|x-y| = 2*max(|x|,|y|); we delay the "*2".
  // vabsq_s16 of -2^15 returns -2^15, which gets reinterpreted to 2^15.
  int16x8_t dcVec0 = vaddq_s16( m1[0], m1[2] );
  int16x8_t dcVec1 = vaddq_s16( m1[1], m1[3] );
  uint16x8_t r0 = vreinterpretq_u16_s16( vabsq_s16( dcVec0 ) );
  uint16x8_t r1 = vreinterpretq_u16_s16( vabsq_s16( dcVec1 ) );
  uint16x8_t r2 = vreinterpretq_u16_s16( vabdq_s16( m1[0], m1[2] ) );
  uint16x8_t r3 = vreinterpretq_u16_s16( vabdq_s16( m1[1], m1[3] ) );
  uint16x8_t r4 = vreinterpretq_u16_s16( vabsq_s16( vaddq_s16( m1[4], m1[6] ) ) );
  uint16x8_t r5 = vreinterpretq_u16_s16( vabsq_s16( vaddq_s16( m1[5], m1[7] ) ) );
  uint16x8_t r6 = vreinterpretq_u16_s16( vabdq_s16( m1[4], m1[6] ) );
  uint16x8_t r7 = vreinterpretq_u16_s16( vabdq_s16( m1[5], m1[7] ) ); // 16-bit

  const int32x4_t dcVec = vabsq_s32( vaddl_s16( vget_high_s16( dcVec0 ), vget_high_s16( dcVec1 ) ) );
  const uint32_t absDC = ( uint32_t )vgetq_lane_s32( dcVec, 3 );

  const uint16x8_t max0 = vmaxq_u16( r0, r1 );
  const uint16x8_t max1 = vmaxq_u16( r2, r3 );
  const uint16x8_t max2 = vmaxq_u16( r4, r5 );
  const uint16x8_t max3 = vmaxq_u16( r6, r7 );

  const uint32x4_t sum = horizontal_add_long_4d_u16x8( max0, max1, max2, max3 );

  uint32_t sad = horizontal_add_u32x4( sum );
  sad <<= 1; // Apply the deferred doubling from the last butterfly.
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( sad + 2 ) >> 2;

  return sad << 2;
}

Distortion xCalcHAD8x8_sve2( const Pel* piOrg, const Pel* piCur, const ptrdiff_t iStrideOrg,
                             const ptrdiff_t iStrideCur )
{
  int16x8_t m1[8], m2[8];

  for( int i = 0; i < 8; i++ )
  {
    const int16x8_t org = vld1q_s16( piOrg + iStrideOrg * i );
    const int16x8_t cur = vld1q_s16( piCur + iStrideCur * i );
    m1[i] = vsubq_s16( org, cur ); // 11-bit
  }

  const uint16x8_t idxs = vld1q_u16( kZip1DTbl );

  // Horizontal.
  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] ); // 12-bit

  m1[0] = vvenc_svtbl_s16( m2[0], idxs );
  m1[1] = vvenc_svtbl_s16( m2[1], idxs );
  m1[2] = vvenc_svtbl_s16( m2[2], idxs );
  m1[3] = vvenc_svtbl_s16( m2[3], idxs );
  m1[4] = vvenc_svtbl_s16( m2[4], idxs );
  m1[5] = vvenc_svtbl_s16( m2[5], idxs );
  m1[6] = vvenc_svtbl_s16( m2[6], idxs );
  m1[7] = vvenc_svtbl_s16( m2[7], idxs );

  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] ); // 13-bit

  m1[0] = vvenc_svtbl_s16( m2[0], idxs );
  m1[1] = vvenc_svtbl_s16( m2[1], idxs );
  m1[2] = vvenc_svtbl_s16( m2[2], idxs );
  m1[3] = vvenc_svtbl_s16( m2[3], idxs );
  m1[4] = vvenc_svtbl_s16( m2[4], idxs );
  m1[5] = vvenc_svtbl_s16( m2[5], idxs );
  m1[6] = vvenc_svtbl_s16( m2[6], idxs );
  m1[7] = vvenc_svtbl_s16( m2[7], idxs );

  m2[0] = vvenc_cadd_s16<90>( m1[0], m1[0] );
  m2[1] = vvenc_cadd_s16<90>( m1[1], m1[1] );
  m2[2] = vvenc_cadd_s16<90>( m1[2], m1[2] );
  m2[3] = vvenc_cadd_s16<90>( m1[3], m1[3] );
  m2[4] = vvenc_cadd_s16<90>( m1[4], m1[4] );
  m2[5] = vvenc_cadd_s16<90>( m1[5], m1[5] );
  m2[6] = vvenc_cadd_s16<90>( m1[6], m1[6] );
  m2[7] = vvenc_cadd_s16<90>( m1[7], m1[7] ); // 14-bit

  // Vertical.
  m1[0] = vaddq_s16( m2[0], m2[4] );
  m1[1] = vaddq_s16( m2[1], m2[5] );
  m1[2] = vaddq_s16( m2[2], m2[6] );
  m1[3] = vaddq_s16( m2[3], m2[7] );
  m1[4] = vsubq_s16( m2[0], m2[4] );
  m1[5] = vsubq_s16( m2[1], m2[5] );
  m1[6] = vsubq_s16( m2[2], m2[6] );
  m1[7] = vsubq_s16( m2[3], m2[7] ); // 15-bit

  // The last butterfly uses |x+y|+|x-y| = 2*max(|x|,|y|); we delay the "*2".
  int16x8_t dcVec0 = vaddq_s16( m1[0], m1[2] );
  int16x8_t dcVec1 = vaddq_s16( m1[1], m1[3] );

  // vabsq_s16 of -2^15 returns -2^15, which gets reinterpreted to 2^15.
  uint16x8_t r0 = vreinterpretq_u16_s16( vabsq_s16( dcVec0 ) );
  uint16x8_t r1 = vreinterpretq_u16_s16( vabsq_s16( dcVec1 ) );
  uint16x8_t r2 = vreinterpretq_u16_s16( vabdq_s16( m1[0], m1[2] ) );
  uint16x8_t r3 = vreinterpretq_u16_s16( vabdq_s16( m1[1], m1[3] ) );
  uint16x8_t r4 = vreinterpretq_u16_s16( vabsq_s16( vaddq_s16( m1[4], m1[6] ) ) );
  uint16x8_t r5 = vreinterpretq_u16_s16( vabsq_s16( vaddq_s16( m1[5], m1[7] ) ) );
  uint16x8_t r6 = vreinterpretq_u16_s16( vabdq_s16( m1[4], m1[6] ) );
  uint16x8_t r7 = vreinterpretq_u16_s16( vabdq_s16( m1[5], m1[7] ) ); // 16-bit

  const int32x4_t dcVec = vabsq_s32( vaddl_s16( vget_high_s16( dcVec0 ), vget_high_s16( dcVec1 ) ) );
  const uint32_t absDC = ( uint32_t )vgetq_lane_s32( dcVec, 3 );

  const uint16x8_t max0 = vmaxq_u16( r0, r1 );
  const uint16x8_t max1 = vmaxq_u16( r2, r3 );
  const uint16x8_t max2 = vmaxq_u16( r4, r5 );
  const uint16x8_t max3 = vmaxq_u16( r6, r7 );

  const uint32x4_t sum = horizontal_add_long_4d_u16x8( max0, max1, max2, max3 );

  uint64_t sad = horizontal_add_long_u32x4( sum );
  sad <<= 1; // Apply the deferred doubling from the last butterfly.
  sad -= absDC;
  sad += absDC >> 2;
  sad = ( sad + 2 ) >> 2;

  return sad;
}

Distortion xGetHADs_generic_sve2( const DistParam& rcDtParam )
{
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  int iRows = rcDtParam.org.height;
  const int iCols = rcDtParam.org.width;
  const ptrdiff_t iStrideCur = rcDtParam.cur.stride;
  const ptrdiff_t iStrideOrg = rcDtParam.org.stride;

  CHECKD( rcDtParam.bitDepth > 10, "Only bit-depths of up to 10 bits supported!" );

  Distortion uiSum = 0;

  if( iCols > iRows && iCols % 16 == 0 && iRows % 8 == 0 )
  {
    do
    {
      for( int x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHAD16x8_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
      iRows -= 8;
    } while( iRows != 0 );
  }
  else if( iCols < iRows && iRows % 16 == 0 && iCols % 8 == 0 )
  {
    do
    {
      for( int x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHAD8x16_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 16 * iStrideOrg;
      piCur += 16 * iStrideCur;
      iRows -= 16;
    } while( iRows != 0 );
  }
  else if( iCols > iRows && iCols % 8 == 0 && iRows % 4 == 0 )
  {
    for( int x = 0; x < iCols; x += 8 )
    {
      uiSum += xCalcHAD8x4_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
    }
  }
  else if( iCols < iRows && iRows % 8 == 0 && iCols % 4 == 0 )
  {
    do
    {
      uiSum += xCalcHAD4x8_neon( piOrg, piCur, iStrideOrg, iStrideCur );

      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
      iRows -= 8;
    } while( iRows != 0 );
  }
  else if( iCols % 8 == 0 && iRows == iCols )
  {
    do
    {
      for( int x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHAD8x8_sve2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
      iRows -= 8;
    } while( iRows != 0 );
  }
  else if( iRows % 4 == 0 && iCols % 4 == 0 )
  {
    do
    {
      for( int x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHAD4x4_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 4 * iStrideOrg;
      piCur += 4 * iStrideCur;
      iRows -= 4;
    } while( iRows != 0 );
  }
  else
  {
    do
    {
      for( int x = 0; x < iCols; x += 2 )
      {
        uiSum += RdCost::xCalcHADs2x2( &piOrg[x], &piCur[x], ( int )iStrideOrg, ( int )iStrideCur );
      }
      piOrg += 2 * iStrideOrg;
      piCur += 2 * iStrideCur;
      iRows -= 2;
    } while( iRows != 0 );
  }

  return uiSum;
}

template<int width>
Distortion xGetHADs_sve2( const DistParam& rcDtParam )
{
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  int iRows = rcDtParam.org.height;
  const int iCols = width;
  const ptrdiff_t iStrideCur = rcDtParam.cur.stride;
  const ptrdiff_t iStrideOrg = rcDtParam.org.stride;

  CHECKD( iRows < 2 || iRows > 128 || iCols < 2 || iCols > 128 || iRows % 2 != 0 || iCols % 2 != 0,
          "Invalid row or column count!" );
  CHECKD( rcDtParam.bitDepth > 10, "Only bit-depths of up to 10 bits supported!" );

  Distortion uiSum = 0;

  if( iCols > iRows && iCols % 16 == 0 && iRows % 8 == 0 )
  {
    do
    {
      for( int x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHAD16x8_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
      iRows -= 8;
    } while( iRows != 0 );
  }
  else if( iCols < iRows && iRows % 16 == 0 && iCols % 8 == 0 )
  {
    do
    {
      for( int x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHAD8x16_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 16 * iStrideOrg;
      piCur += 16 * iStrideCur;
      iRows -= 16;
    } while( iRows != 0 );
  }
  else if( iCols > iRows && iCols % 8 == 0 && iRows % 4 == 0 )
  {
    for( int x = 0; x < iCols; x += 8 )
    {
      uiSum += xCalcHAD8x4_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
    }
  }
  else if( iCols < iRows && iRows % 8 == 0 && iCols % 4 == 0 )
  {
    do
    {
      uiSum += xCalcHAD4x8_neon( piOrg, piCur, iStrideOrg, iStrideCur );

      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
      iRows -= 8;
    } while( iRows != 0 );
  }
  else if( iCols % 8 == 0 && iRows == iCols )
  {
    do
    {
      for( int x = 0; x < iCols; x += 8 )
      {
        uiSum += xCalcHAD8x8_sve2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 8 * iStrideOrg;
      piCur += 8 * iStrideCur;
      iRows -= 8;
    } while( iRows != 0 );
  }
  else if( iRows % 4 == 0 && iCols % 4 == 0 )
  {
    do
    {
      for( int x = 0; x < iCols; x += 4 )
      {
        uiSum += xCalcHAD4x4_neon( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 4 * iStrideOrg;
      piCur += 4 * iStrideCur;
      iRows -= 4;
    } while( iRows != 0 );
  }
  else
  {
    do
    {
      for( int x = 0; x < iCols; x += 2 )
      {
        uiSum += RdCost::xCalcHADs2x2( &piOrg[x], &piCur[x], ( int )iStrideOrg, ( int )iStrideCur );
      }
      piOrg += 2 * iStrideOrg;
      piCur += 2 * iStrideCur;
      iRows -= 2;
    } while( iRows != 0 );
  }

  return uiSum;
}

template<int width>
Distortion xGetHADs_fast_sve2( const DistParam& rcDtParam )
{
  const Pel* piOrg = rcDtParam.org.buf;
  const Pel* piCur = rcDtParam.cur.buf;
  int iRows = rcDtParam.org.height;
  int iCols = width;
  const ptrdiff_t iStrideCur = rcDtParam.cur.stride;
  const ptrdiff_t iStrideOrg = rcDtParam.org.stride;

  CHECKD( rcDtParam.bitDepth > 10, "Only bit-depths of up to 10 bits supported!" );
  CHECKD( iRows < 2 || iRows > 128 || iCols < 2 || iCols > 128 || iRows % 2 != 0 || iCols % 2 != 0,
          "Invalid row or column count!" );

  Distortion uiSum = 0;
  if( iCols % 32 == 0 && iRows == iCols )
  {
    do
    {
      for( int x = 0; x < iCols; x += 16 )
      {
        uiSum += xCalcHAD16x16_fast_sve2( &piOrg[x], &piCur[x], iStrideOrg, iStrideCur );
      }
      piOrg += 16 * iStrideOrg;
      piCur += 16 * iStrideCur;
      iRows -= 16;
    } while( iRows != 0 );
  }
  else
  {
    return xGetHADs_sve2<width>( rcDtParam );
  }

  return uiSum;
}

template<>
void RdCost::_initRdCostARM<SVE2>()
{
  m_afpDistortFunc[0][DF_HAD] = xGetHADs_generic_sve2;
  m_afpDistortFunc[0][DF_HAD2] = xGetHADs_sve2<2>;
  m_afpDistortFunc[0][DF_HAD4] = xGetHADs_sve2<4>;
  m_afpDistortFunc[0][DF_HAD8] = xGetHADs_sve2<8>;
  m_afpDistortFunc[0][DF_HAD16] = xGetHADs_sve2<16>;
  m_afpDistortFunc[0][DF_HAD32] = xGetHADs_sve2<32>;
  m_afpDistortFunc[0][DF_HAD64] = xGetHADs_sve2<64>;
  m_afpDistortFunc[0][DF_HAD128] = xGetHADs_sve2<128>;

  m_afpDistortFunc[0][DF_HAD_fast] = xGetHADs_generic_sve2;
  m_afpDistortFunc[0][DF_HAD2_fast] = xGetHADs_sve2<2>;
  m_afpDistortFunc[0][DF_HAD4_fast] = xGetHADs_sve2<4>;
  m_afpDistortFunc[0][DF_HAD8_fast] = xGetHADs_sve2<8>;
  m_afpDistortFunc[0][DF_HAD16_fast] = xGetHADs_sve2<16>;
  m_afpDistortFunc[0][DF_HAD32_fast] = xGetHADs_fast_sve2<32>;
  m_afpDistortFunc[0][DF_HAD64_fast] = xGetHADs_fast_sve2<64>;
  m_afpDistortFunc[0][DF_HAD128_fast] = xGetHADs_fast_sve2<128>;
}

#endif // defined( TARGET_SIMD_ARM )

} // namespace vvenc

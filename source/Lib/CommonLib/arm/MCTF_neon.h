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
/** \file     MCTF_neon.h
    \brief    SIMD for MCTF
*/

#pragma once

#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "MCTF.h"

#include <arm_neon.h>

namespace vvenc
{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCTF

static const int32_t xSzm[6] = { 0, 1, 20, 336, 5440, 87296 };

static inline void applyPlanarDeblockingCorrection_common( Pel* dstPel, const ptrdiff_t dstStride, const int32_t x1yzm,
                                                           const int32_t x2yzm, const int32_t ySum, const int32_t w,
                                                           const int32_t h, const ClpRng& clpRng,
                                                           const uint16_t motionError )
{
  const int32_t blockSize = w * h;
  const int32_t log2Width = floorLog2( w );
  const int32_t maxPelVal = clpRng.max();
  const int32_t mWeight = std::min( 512u, ( uint32_t )motionError * motionError );
  const int32_t xSum = ( blockSize * ( w - 1 ) ) >> 1;
  int32_t b0, b1, b2;
  int64_t numer, denom;

  CHECK( w != h, "Width must be same as height!" );
  CHECK( w < 4, "Width must be greater than or equal to 4!" );
  CHECK( ( w & ( w - 1 ) ) != 0, "Width can only be power of two!" );

  denom = blockSize * xSzm[log2Width]; // Plane-fit parameters, in fixed-point arithmetic.
  numer = ( int64_t )mWeight * ( ( int64_t )x1yzm * blockSize - xSum * ySum );
  b1 = int32_t( ( numer < 0 ? numer - ( denom >> 1 ) : numer + ( denom >> 1 ) ) / denom );
  b1 = b1 < INT16_MIN ? INT16_MIN : b1 > INT16_MAX ? INT16_MAX : b1;
  numer = ( int64_t )mWeight * ( ( int64_t )x2yzm * blockSize - xSum * ySum );
  b2 = int32_t( ( numer < 0 ? numer - ( denom >> 1 ) : numer + ( denom >> 1 ) ) / denom );
  b2 = b2 > INT16_MAX ? INT16_MAX : b2 < INT16_MIN ? INT16_MIN : b2;
  b0 = ( mWeight * ySum - ( b1 + b2 ) * xSum + ( blockSize >> 1 ) ) >> ( log2Width << 1 );

  if( b0 == 0 && b1 == 0 && b2 == 0 )
  {
    return;
  }

  if( w % 8 == 0 )
  {
    Pel* pDst = dstPel;
    const int32_t idx_off[4] = { 0, 1, 2, 3 };
    int32x4_t idxVec = vld1q_s32( idx_off );
    int32x4_t b1x = vmulq_s32( idxVec, vdupq_n_s32( b1 ) ); // {0*b1,1*b1,2*b1,3*b1}

    int32x4_t step4 = vdupq_n_s32( b1 << 2 );
    int32x4_t step8 = vdupq_n_s32( b1 << 3 );

    for( int32_t y = 0; y < h; y++ )
    {
      int32x4_t pc0 = vaddq_s32( vdupq_n_s32( b0 + b2 * y ), b1x ); // Plane corrector for x…x+3.
      int32x4_t pc1 = vaddq_s32( pc0, step4 );                      // Plane corrector for x+4…x+7.
      int32_t x = w;

      do
      {
        int16x8_t dst = vld1q_s16( pDst );
        int16x4_t p_lo = vqrshrn_n_s32( pc0, 9 ); // With saturation.
        int16x4_t p_hi = vqrshrn_n_s32( pc1, 9 );
        int16x8_t p = vcombine_s16( p_lo, p_hi );
        int16x8_t z = vqsubq_s16( dst, p ); // With saturation.
        z = vmaxq_s16( vdupq_n_s16( 0 ), vminq_s16( z, vdupq_n_s16( maxPelVal ) ) );
        vst1q_s16( pDst, z );

        // Increment the plane corrector by 8 * b1.
        pc0 = vaddq_s32( pc0, step8 );
        pc1 = vaddq_s32( pc1, step8 );
        pDst += 8;
        x -= 8;
      } while( x != 0 );

      pDst += dstStride - w;
    }
  }
  else
  {
    Pel* pDst = dstPel;
    const int32_t b1_idx[] = { 0, b1, b1 << 1, b1 * 3 };
    int32x4_t b1x = vld1q_s32( b1_idx );
    int32x4_t pc = vaddq_s32( b1x, vdupq_n_s32( b0 ) ); // Plane corrector.
    int32_t y = 4;

    do
    {
      // Perform deblocking by adding fitted correction plane.
      int16x4_t dst = vld1_s16( pDst );
      int16x4_t p = vqrshrn_n_s32( pc, 9 );
      int16x4_t z = vqsub_s16( dst, p );
      z = vmax_s16( vdup_n_s16( 0 ), vmin_s16( z, vdup_n_s16( maxPelVal ) ) );
      vst1_s16( pDst, z );

      // b0 + b1 * x + b2 * y for next iteration.
      pc = vaddq_s32( pc, vdupq_n_s32( b2 ) );
      pDst += dstStride;
    } while( --y != 0 );
  }
}
#endif
} // namespace vvenc

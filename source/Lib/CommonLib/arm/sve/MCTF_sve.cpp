/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2025, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
 * \file MCTF_sve.cpp
 * \brief SVE implementation of MCTF for AArch64.
 */

//  ====================================================================================================================
//  Includes
//  ====================================================================================================================
#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"

#include "MCTF.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_MCTF

#include "MCTF_neon.h"
#include "neon_sve_bridge.h"
#include <arm_neon.h>
#include <arm_sve.h>

namespace vvenc
{
void applyPlanarCorrection_sve( const Pel* refPel, const ptrdiff_t refStride, Pel* dstPel, const ptrdiff_t dstStride,
                                const int32_t w, const int32_t h, const ClpRng& clpRng, const uint16_t motionError )
{
  int32_t x1yzm = 0, x2yzm = 0, ySum = 0;

  CHECK( w != h, "Width must be same as height!" );
  CHECK( w < 4, "Width must be greater than or equal to 4!" );
  CHECK( ( w & ( w - 1 ) ) != 0, "Width can only be power of two!" );

  if( w % 8 == 0 )
  {
    Pel* pDst = dstPel;
    const Pel* pRef = refPel;

    const int16_t idx_off[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    int16x8_t vidx_off = vld1q_s16( idx_off );
    int64x2_t acc_x = vdupq_n_s64( 0 );

    for( int32_t y = 0; y < h; y += 2 ) // Sum up dot-products between indices and sample diffs.
    {
      int32_t x = 0;
      int16x8_t acc_z0 = vdupq_n_s16( 0 ); // Σ(z) for one iteration of y.
      int16x8_t acc_z1 = vdupq_n_s16( 0 );
      int16x8_t xv = vidx_off;

      do
      {
        int16x8_t dst0 = vld1q_s16( pDst + x );
        int16x8_t ref0 = vld1q_s16( pRef + x );
        int16x8_t dst1 = vld1q_s16( pDst + dstStride + x );
        int16x8_t ref1 = vld1q_s16( pRef + refStride + x );

        int16x8_t diff0 = vsubq_s16( dst0, ref0 );
        int16x8_t diff1 = vsubq_s16( dst1, ref1 );

        acc_z0 = vaddq_s16( acc_z0, diff0 );
        acc_z1 = vaddq_s16( acc_z1, diff1 );

        acc_x = vvenc_sdotq_s16( acc_x, diff0, xv );
        acc_x = vvenc_sdotq_s16( acc_x, diff1, xv );

        xv = vaddq_s16( xv, vdupq_n_s16( 8 ) );
        x += 8;
      } while( x != w );

      int32_t zAcc0 = vaddlvq_s16( acc_z0 );
      int32_t zAcc1 = vaddlvq_s16( acc_z1 );

      x2yzm += y * zAcc0 + ( y + 1 ) * zAcc1; // Σ(y*z) calculated as y * Σ(z).
      ySum += zAcc0 + zAcc1;                  // Σ(z)

      pDst += dstStride << 1;
      pRef += refStride << 1;
    }
    x1yzm = ( int32_t )vaddvq_s64( acc_x ); // Σ(x*z)
  }
  else // w = h = 4
  {
    Pel* pDst = dstPel;
    const Pel* pRef = refPel;

    // Sum up dot-products between indices and sample diffs.
    int16x8_t dst = vcombine_s16( vld1_s16( pDst ), vld1_s16( pDst + dstStride ) ); // 10 bit
    int16x8_t ref = vcombine_s16( vld1_s16( pRef ), vld1_s16( pRef + refStride ) ); // 10 bit
    int16x8_t diff0 = vsubq_s16( dst, ref );
    int16x8_t acc_y = vextq_s16( diff0, vdupq_n_s16( 0 ), 4 );

    dst = vcombine_s16( vld1_s16( pDst + 2 * dstStride ), vld1_s16( pDst + 3 * dstStride ) );
    ref = vcombine_s16( vld1_s16( pRef + 2 * refStride ), vld1_s16( pRef + 3 * refStride ) );
    int16x8_t diff1 = vsubq_s16( dst, ref );
    int16x8_t acc_z = vaddq_s16( diff0, diff1 );

    const int16_t idx_off1[] = { 2, 2, 2, 2, 3, 3, 3, 3 };
    int16x8_t v_idx_off1 = vld1q_s16( idx_off1 );
    const int16_t idx_off2[] = { 0, 1, 2, 3, 0, 1, 2, 3 };
    int16x8_t v_idx_off2 = vld1q_s16( idx_off2 );
    acc_y = vmlaq_s16( acc_y, diff1, v_idx_off1 );
    int16x8_t acc_x = vmulq_s16( acc_z, v_idx_off2 );

    ySum = vaddlvq_s16( acc_z );  // Σ(z)
    x2yzm = vaddlvq_s16( acc_y ); // Σ(y*z)
    x1yzm = vaddlvq_s16( acc_x ); // Σ(x*z)
  }

  applyPlanarDeblockingCorrection_common( dstPel, dstStride, x1yzm, x2yzm, ySum, w, h, clpRng, motionError );
}

template<>
void MCTF::_initMCTF_ARM<SVE>()
{
  m_applyPlanarCorrection = applyPlanarCorrection_sve;
}

} // namespace vvenc

#endif
//! \}

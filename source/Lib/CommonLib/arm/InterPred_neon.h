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
/** \file     InterPred_neon.h
    \brief    Affine luma CU kernels shared by the NEON and SVE inter prediction inits
*/

#pragma once

#include "CommonDefARM.h"
#include "InterPrediction.h"

#include <algorithm>
#include <arm_neon.h>

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_BDOF && ENABLE_SIMD_OPT_MCIF

namespace vvenc
{

// The 4x4 interpolation filter the affine CU kernels take per sub-block (the NEON or the SVE simdFilter4x4_N6). The
// kernels are templated on it, so the call inlines; everything here is static, so that the NEON and the SVE file each
// get their own copy compiled for their architecture.
using AffineFilter4x4 = void ( * )( const ClpRng&, Pel const*, int, Pel*, int, int, int, TFilterCoeff const*,
                                     TFilterCoeff const* );

// The luma sub-block loop of xPredAffineBlk without PROF in one call: the same sub-block MVs (stored for the chroma
// pass) and the same samples. Every sub-block takes the 6-tap 4x4 filter, also where xPredAffineBlk filters in one
// direction only because a fraction is 0: the filter of fraction 0 is the single tap 1 << IF_FILTER_PREC, which the
// first stage scales exactly and the second stage rounds as the one-direction filter does. filter4x4 is the 4x4 filter
// with isLast = !bi.
template<bool bi, AffineFilter4x4 filter4x4>
static void affineLumaCU_arm( const InterPredInterpolation::AffineLumaCU& cu )
{
  for( int h = 0; h < cu.height; h += AFFINE_MIN_BLOCK_SIZE )
  {
    Pel* dst = cu.dst + h * cu.dstStride;
    for( int w = 0; w < cu.width; w += AFFINE_MIN_BLOCK_SIZE )
    {
      const AffineLumaSubblockRef r = affineLumaSubblockRef( cu, w, h );
      filter4x4( cu.clpRng, r.ref, cu.refStride, dst + w, cu.dstStride, AFFINE_MIN_BLOCK_SIZE, AFFINE_MIN_BLOCK_SIZE,
                 InterpolationFilter::m_lumaFilter4x4[r.xFrac], InterpolationFilter::m_lumaFilter4x4[r.yFrac] );
    }
  }
}

// PROF of one 4x4 affine sub-block, as affinePROF4x4_SSE does on x86. pred holds the 4x4 filter's 14-bit output, ref
// is the sub-block's nearest integer position in the reference, which supplies the one-sample border. Each window row
// lives in one register (columns -1 .. 6 in lanes 0 .. 7), so both gradients are lane shifts: gradX = right - left two
// lanes apart, gradY = the row below minus the row above. The arithmetic matches gradFilter_neon< false > followed by
// applyPROF: 16-bit wrapping border scale and >> 6, the products of the dMv and gradient pairs summed in 32 bit, the
// clip to the PROF limit, the saturating narrowing and, without bi, the saturating offset add, shift and clip.
template<bool bi>
static ALWAYS_INLINE void affinePROF4x4_neon( Pel* dst, int dstStride, const Pel* pred, const Pel* ref, int refStride,
                                              const int16_t* dMvXY, int shiftNum, Pel offset, const ClpRng& clpRng )
{
  static const uint16_t predLanes[8] = { 0, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0, 0, 0 };
  const uint16x8_t predMask    = vld1q_u16( predLanes );
  const int16x8_t  borderShift = vdupq_n_s16( shiftNum );
  const int16x8_t  internalOff = vdupq_n_s16( IF_INTERNAL_OFFS );
  const int        dILimit     = 1 << std::max<int>( clpRng.bd + 1, 13 );
  const int32x4_t  dIMin       = vdupq_n_s32( -dILimit );
  const int32x4_t  dIMax       = vdupq_n_s32( dILimit - 1 );

  // Window rows -1 .. 4: the reference scaled to the prediction's range, the inner rows with the prediction in lanes
  // 1 .. 4, each >> 6 as the gradient filter does.
  int16x4_t predRow[4];
  int16x8_t window[6];
  for( int r = -1; r < 5; r++ )
  {
    const int16x8_t refRow = vld1q_s16( ref + r * refStride - 1 );
    int16x8_t       row    = vsubq_s16( vshlq_s16( refRow, borderShift ), internalOff );
    if( r >= 0 && r < 4 )
    {
      predRow[r]            = vld1_s16( pred + 4 * r );
      const int16x8_t moved = vextq_s16( vdupq_n_s16( 0 ), vcombine_s16( predRow[r], predRow[r] ), 7 );
      row                   = vbslq_s16( predMask, moved, row );
    }
    window[r + 1] = vshrq_n_s16( row, 6 );
  }

  const int16x8_t vOffset = vdupq_n_s16( offset );
  const int16x8_t vMin    = vdupq_n_s16( clpRng.min() );
  const int16x8_t vMax    = vdupq_n_s16( clpRng.max() );
  const int16x8_t vShift  = vdupq_n_s16( -shiftNum );
  for( int r = 0; r < 4; r += 2 )
  {
    int16x4_t dI[2];
    for( int k = 0; k < 2; k++ )
    {
      const int16x8_t w     = window[r + k + 1];
      const int16x8_t gradX = vsubq_s16( vextq_s16( w, w, 2 ), w );
      const int16x8_t dY    = vsubq_s16( window[r + k + 2], window[r + k] );
      const int16x8_t gradY = vextq_s16( dY, dY, 1 );
      const int16x4x2_t dMv = vld2_s16( dMvXY + 8 * ( r + k ) );   // the (dMvX, dMvY) pairs of the row
      int32x4_t sum         = vmull_s16( dMv.val[0], vget_low_s16( gradX ) );
      sum                   = vmlal_s16( sum, dMv.val[1], vget_low_s16( gradY ) );
      sum                   = vminq_s32( dIMax, vmaxq_s32( dIMin, sum ) );
      dI[k]                 = vqmovn_s32( sum );
    }
    int16x8_t res = vaddq_s16( vcombine_s16( dI[0], dI[1] ), vcombine_s16( predRow[r], predRow[r + 1] ) );
    if( !bi )
    {
      res = vshlq_s16( vqaddq_s16( res, vOffset ), vShift );
      res = vminq_s16( vMax, vmaxq_s16( vMin, res ) );
    }
    vst1_s16( dst + r * dstStride, vget_low_s16( res ) );
    vst1_s16( dst + ( r + 1 ) * dstStride, vget_high_s16( res ) );
  }
}

// The luma sub-block loop of xPredAffineBlk with PROF in one call: the 4x4 filter into a small buffer (14-bit, as PROF
// takes it), then the PROF of the sub-block. Same MVs and samples as the per-sub-block path. filter4x4 is the 4x4
// filter with isLast = false.
template<bool bi, AffineFilter4x4 filter4x4>
static void affineLumaCUProf_arm( const InterPredInterpolation::AffineLumaCU& cu )
{
  for( int h = 0; h < cu.height; h += AFFINE_MIN_BLOCK_SIZE )
  {
    Pel* dst = cu.dst + h * cu.dstStride;
    for( int w = 0; w < cu.width; w += AFFINE_MIN_BLOCK_SIZE )
    {
      const AffineLumaSubblockRef r = affineLumaSubblockRef( cu, w, h );
      Pel pred[AFFINE_MIN_BLOCK_SIZE * AFFINE_MIN_BLOCK_SIZE];
      filter4x4( cu.clpRng, r.ref, cu.refStride, pred, AFFINE_MIN_BLOCK_SIZE, AFFINE_MIN_BLOCK_SIZE,
                 AFFINE_MIN_BLOCK_SIZE, InterpolationFilter::m_lumaFilter4x4[r.xFrac],
                 InterpolationFilter::m_lumaFilter4x4[r.yFrac] );
      affinePROF4x4_neon<bi>( dst + w, cu.dstStride, pred, r.nearest, cu.refStride, cu.dMvXY, cu.shiftNum, cu.offset,
                              cu.clpRng );
    }
  }
}

} // namespace vvenc

#endif

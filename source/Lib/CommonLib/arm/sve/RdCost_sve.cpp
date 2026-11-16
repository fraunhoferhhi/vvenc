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

/** \file     RdCost_sve.cpp
    \brief    RD cost computation class, SVE version
*/

#include <limits>
#include <math.h>

#include "../neon/reverse_neon.h"
#include "../neon/sum_neon.h"
#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "CommonLib/RdCost.h"
#include "neon_sve_bridge.h"

#include <arm_sve.h>

namespace vvenc
{

#if ENABLE_SIMD_OPT_DIST && defined( TARGET_SIMD_ARM )

static inline int64x2_t xGetSADwMask_sve_step( const short* src1, const short* src2, const short* weightMask, int stepX,
                                               int x, int64x2_t sum )
{
  int16x8_t vsrc1 = vld1q_s16( src1 + x );
  int16x8_t vsrc2 = vld1q_s16( src2 + x );
  int16x8_t vmask;
  if( stepX == -1 )
  {
    vmask = vld1q_s16( weightMask - x - 7 );
    vmask = reverse_vector_s16x8( vmask );
  }
  else
  {
    vmask = vld1q_s16( weightMask + x );
  }
  int16x8_t diff = vabdq_s16( vsrc1, vsrc2 );
  return vvenc_sdotq_s16( sum, diff, vmask );
}

Distortion xGetSADwMask_sve( const DistParam& rcDtParam )
{
  if( rcDtParam.org.width < 4 || rcDtParam.bitDepth > 10 || rcDtParam.applyWeight )
  {
    return RdCost::xGetSADwMask( rcDtParam );
  }

  const short* src1 = ( const short* )rcDtParam.org.buf;
  const short* src2 = ( const short* )rcDtParam.cur.buf;
  const short* weightMask = ( const short* )rcDtParam.mask;
  int rows = rcDtParam.org.height;
  int cols = rcDtParam.org.width;
  int subShift = rcDtParam.subShift;
  int subStep = 1 << subShift;
  const int strideSrc1 = rcDtParam.org.stride * subStep;
  const int strideSrc2 = rcDtParam.cur.stride * subStep;
  const int strideMask = rcDtParam.maskStride * subStep;

  int64x2_t sum0 = vdupq_n_s64( 0 );

  if( cols == 8 )
  {
    do
    {
      sum0 = xGetSADwMask_sve_step( src1, src2, weightMask, rcDtParam.stepX, 0, sum0 );

      src1 += strideSrc1;
      src2 += strideSrc2;
      weightMask += strideMask;
      rows -= subStep;
    } while( rows != 0 );
  }
  else
  {
    int64x2_t sum1 = vdupq_n_s64( 0 );

    do
    {
      int x = 0;
      do
      {
        sum0 = xGetSADwMask_sve_step( src1, src2, weightMask, rcDtParam.stepX, x + 0, sum0 );
        sum1 = xGetSADwMask_sve_step( src1, src2, weightMask, rcDtParam.stepX, x + 8, sum1 );
        x += 16;
      } while( x != cols );

      src1 += strideSrc1;
      src2 += strideSrc2;
      weightMask += strideMask;
      rows -= subStep;
    } while( rows != 0 );

    sum0 = vaddq_s64( sum0, sum1 );
  }

  Distortion sum = horizontal_add_s64x2( sum0 );
  sum <<= subShift;
  return sum >> DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth );
}

// The squared differences of one vector of 8, added to acc in one 16-bit dot product (64-bit lanes).
static inline int64x2_t sseAccumulate8_sve( const Pel* org, const Pel* cur, const int64x2_t acc )
{
  const int16x8_t o    = vld1q_s16( org );
  const int16x8_t c    = vld1q_s16( cur );
  const int16x8_t diff = vsubq_s16( o, c );
  return vvenc_sdotq_s16( acc, diff, diff );
}

// DF_SSE8 .. DF_SSE128 as xGetSSE_NxN_neon, with each vector's eight squares added in one dot product into four
// independent accumulators: the exact sum in 64 bits, as the C code. The NEON kernel's wrapping 32-bit lanes give the
// same sum for samples of up to 10 bits (a lane of a 128 x 128 block takes 512 squares below 2^20), but not for deeper
// samples, where this kernel stays exact.
template<int iWidth>
Distortion xGetSSE_NxN_sve( const DistParam& rcDtParam )
{
  const Pel*     org       = rcDtParam.org.buf;
  const Pel*     cur       = rcDtParam.cur.buf;
  const int      rows      = rcDtParam.org.height;
  const int      orgStride = rcDtParam.org.stride;
  const int      curStride = rcDtParam.cur.stride;
  const uint32_t shift     = DISTORTION_PRECISION_ADJUSTMENT( rcDtParam.bitDepth ) << 1;

  int64x2_t acc[4];
  for( int k = 0; k < 4; k++ )
  {
    acc[k] = vdupq_n_s64( 0 );
  }

  if( iWidth == 8 || iWidth == 16 )
  {
    // two rows per step: 2 (W 8) or 4 (W 16) chains
    int y = 0;
    for( ; y + 2 <= rows; y += 2, org += 2 * orgStride, cur += 2 * curStride )
    {
      acc[0] = sseAccumulate8_sve( org, cur, acc[0] );
      acc[1] = sseAccumulate8_sve( org + orgStride, cur + curStride, acc[1] );
      if( iWidth == 16 )
      {
        acc[2] = sseAccumulate8_sve( org + 8, cur + 8, acc[2] );
        acc[3] = sseAccumulate8_sve( org + orgStride + 8, cur + curStride + 8, acc[3] );
      }
    }
    if( y < rows )
    {
      acc[0] = sseAccumulate8_sve( org, cur, acc[0] );
      if( iWidth == 16 )
      {
        acc[2] = sseAccumulate8_sve( org + 8, cur + 8, acc[2] );
      }
    }
  }
  else
  {
    // one row per step, the vectors of 8 in turn on 4 chains
    for( int y = 0; y < rows; y++, org += orgStride, cur += curStride )
    {
      for( int x = 0; x < iWidth; x += 32 )
      {
        acc[0] = sseAccumulate8_sve( org + x + 0, cur + x + 0, acc[0] );
        acc[1] = sseAccumulate8_sve( org + x + 8, cur + x + 8, acc[1] );
        acc[2] = sseAccumulate8_sve( org + x + 16, cur + x + 16, acc[2] );
        acc[3] = sseAccumulate8_sve( org + x + 24, cur + x + 24, acc[3] );
      }
    }
  }

  const int64x2_t sum = vaddq_s64( vaddq_s64( acc[0], acc[1] ), vaddq_s64( acc[2], acc[3] ) );
  return Distortion( horizontal_add_s64x2( sum ) ) >> shift;
}

template<>
void RdCost::_initRdCostARM<SVE>()
{
  m_afpDistortFunc[0][DF_SAD_WITH_MASK] = xGetSADwMask_sve;

  // DF_SSE4 stays with xGetSSE_NxN_neon<4>
  m_afpDistortFunc[0][DF_SSE8]   = xGetSSE_NxN_sve<8>;
  m_afpDistortFunc[0][DF_SSE16]  = xGetSSE_NxN_sve<16>;
  m_afpDistortFunc[0][DF_SSE32]  = xGetSSE_NxN_sve<32>;
  m_afpDistortFunc[0][DF_SSE64]  = xGetSSE_NxN_sve<64>;
  m_afpDistortFunc[0][DF_SSE128] = xGetSSE_NxN_sve<128>;
}

#endif // defined( TARGET_SIMD_ARM )

} // namespace vvenc

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

template<>
void RdCost::_initRdCostARM<SVE>()
{
  m_afpDistortFunc[0][DF_SAD_WITH_MASK] = xGetSADwMask_sve;
}

#endif // defined( TARGET_SIMD_ARM )

} // namespace vvenc

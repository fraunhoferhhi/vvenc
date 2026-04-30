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

/** \file     LoopFilter_neon.cpp
    \brief    deblocking filter, Neon version
*/

#include <arm_neon.h>

#include "CommonLib/LoopFilter.h"
#include "CommonDefARM.h"
#include "CommonLib/Rom.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_DBLF

namespace vvenc
{

void xFilteringPandQCore( Pel* src, ptrdiff_t step, const ptrdiff_t offset, int numberPSide, int numberQSide, int tc );

static const int dbCoeffs7[7] = { 59, 50, 41, 32, 23, 14,  5 };
static const int dbCoeffs5[5] = { 58, 45, 32, 19,  6 };
static const int dbCoeffs3[3] = { 53, 32, 11 };
static const int tcCoeffs7[7] = { 6, 5, 4, 3, 2, 1, 1 };
static const int tcCoeffs3[3] = { 6, 4, 2 };

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

static void xFilteringPandQNeon( Pel* src, ptrdiff_t step, const ptrdiff_t offset,
                                 int numberPSide, int numberQSide, int tc )
{
  CHECKD( numberPSide <= 3 && numberQSide <= 3, "Short filtering in long filtering function" );

  if( step == 1 )
    xFilteringPandQHor_neon( src, step, offset, numberPSide, numberQSide, tc );
  else
    xFilteringPandQCore( src, step, offset, numberPSide, numberQSide, tc );
}

template <ARM_VEXT vext>
void LoopFilter::_initLoopFilterARM()
{
  xFilteringPandQ = xFilteringPandQNeon;
}

template void LoopFilter::_initLoopFilterARM<NEON>();

}  // namespace vvenc

#endif  // TARGET_SIMD_ARM && ENABLE_SIMD_DBLF

//! \}

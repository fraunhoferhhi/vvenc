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

/** \file     EncAdaptiveLoopFilterX86.h
 \brief    x86 helpers for adaptive loop filter
 */

#include "CommonDefX86.h"

#include "../EncAdaptiveLoopFilter.h"

//! \ingroup EncoderLib
//! \{

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_ALF

namespace vvenc
{

#define NL_COVAR_bstride ( MAX_NUM_ALF_LUMA_COEFF << 4 )
#define NL_COVAR_kstride ( 1 << 4 )
#define NL_COVAR_xstride ( 1 )
#define GET_ALF_COVAR( elocal, b, k, x )                                                                               \
  elocal[( b ) * NL_COVAR_bstride + ( k ) * NL_COVAR_kstride + ( x ) * NL_COVAR_xstride]

template<X86_VEXT vext>
void simdGetPreBlkStatsWeightedAccum( AlfCovariance& alfCovariance, const AlfFilterShape& shape, const Pel* ELocal,
                                      const Pel yLocal[4][4], const alf_float_t weight[4][4], const int numBins )
{
  for( int b0 = 0; b0 < numBins; b0++ )
  {
    for( int k = 0; k < shape.numCoeff; k++ )
    {
      const Pel* Elocalk = &GET_ALF_COVAR( ELocal, b0, k, 0 );

      __m128 xprodwk[4];
      for( int ii = 0; ii < 4; ii++ )
      {
        const __m128i xlockx32 = _mm_cvtepi16_epi32( _vv_loadl_epi64( ( const __m128i* )&Elocalk[( ii << 2 )] ) );

        __m128 xlocd = _mm_cvtepi32_ps( xlockx32 );
        __m128 xwght = _mm_loadu_ps( &weight[ii][0] );
        __m128 xprdct = _mm_mul_ps( xwght, xlocd );
        xprodwk[ii] = xprdct;
      }

      for( int b1 = 0; b1 < numBins; b1++ )
      {
        alf_float_t* cov = &alfCovariance.E[b0][b1][k][k];

        for( int l = k; l < shape.numCoeff; l++ )
        {
          const Pel* Elocall = &GET_ALF_COVAR( ELocal, b1, l, 0 );

          __m128 xacc = _mm_setzero_ps();

          for( int ii = 0; ii < 4; ii++ )
          {
            const __m128i xloclx32 = _mm_cvtepi16_epi32( _vv_loadl_epi64( ( const __m128i* )&Elocall[( ii << 2 )] ) );

            __m128 xlockd = _mm_cvtepi32_ps( xloclx32 );
            xacc = _mm_add_ps( xacc, _mm_mul_ps( xprodwk[ii], xlockd ) );
          }

          xacc = _mm_hadd_ps( xacc, xacc );
          xacc = _mm_hadd_ps( xacc, xacc );

          *cov++ += _mm_cvtss_f32( xacc );
        }
      }

      __m128 xacc = _mm_setzero_ps();

      for( int ii = 0; ii < 4; ii++ )
      {
        const __m128i yloc32 = _mm_cvtepi16_epi32( _vv_loadl_epi64( ( const __m128i* )&yLocal[ii][0] ) );

        __m128 ylocd = _mm_cvtepi32_ps( yloc32 );
        __m128 xprdct = _mm_mul_ps( ylocd, xprodwk[ii] );

        xacc = _mm_add_ps( xacc, xprdct );
      }

      xacc = _mm_hadd_ps( xacc, xacc );
      xacc = _mm_hadd_ps( xacc, xacc );

      alfCovariance.y[b0][k] += _mm_cvtss_f32( xacc );
    }
  }

  __m128 xacc = _mm_setzero_ps();

  for( int ii = 0; ii < 4; ii++ )
  {
    const __m128i yloc32 = _mm_cvtepi16_epi32( _vv_loadl_epi64( ( const __m128i* )&yLocal[ii][0] ) );

    __m128 ylocd = _mm_cvtepi32_ps( yloc32 );
    __m128 xwght = _mm_loadu_ps( &weight[ii][0] );
    __m128 xprdct = _mm_mul_ps( xwght, _mm_mul_ps( ylocd, ylocd ) );

    xacc = _mm_add_ps( xacc, xprdct );
  }

  xacc = _mm_hadd_ps( xacc, xacc );
  xacc = _mm_hadd_ps( xacc, xacc );

  alfCovariance.pixAcc += _mm_cvtss_f32( xacc );
}

template<X86_VEXT vext>
void simdGetPreBlkStatsAccum( AlfCovariance& alfCovariance, const AlfFilterShape& shape, const Pel* ELocal,
                              const Pel yLocal[4][4], const int numBins )
{
  const __m128i mylocal0 = _mm_loadu_si128( ( const __m128i* )&yLocal[0][0] );
  const __m128i mylocal8 = _mm_loadu_si128( ( const __m128i* )&yLocal[2][0] );

  for( int b0 = 0; b0 < numBins; b0++ )
  {
    for( int k = 0; k < shape.numCoeff; k++ )
    {
      const Pel* Elocalk = &GET_ALF_COVAR( ELocal, b0, k, 0 );

      const __m128i melocalk0 = _mm_loadu_si128( ( const __m128i* )&Elocalk[0] );
      const __m128i melocalk8 = _mm_loadu_si128( ( const __m128i* )&Elocalk[8] );

      for( int b1 = 0; b1 < numBins; b1++ )
      {
        alf_float_t* cov = &alfCovariance.E[b0][b1][k][k];

        int l = k;

        for( ; l < ( shape.numCoeff - 3 ); l += 4 )
        {
          __m128i vmacc[4];

          for( int ll = 0; ll < 4; ll++ )
          {
            const Pel* Elocall = &GET_ALF_COVAR( ELocal, b1, l + ll, 0 );

            __m128i melocall0 = _mm_loadu_si128( ( const __m128i* )&Elocall[0] );
            __m128i melocall8 = _mm_loadu_si128( ( const __m128i* )&Elocall[8] );

            __m128i mmacc0 = _mm_madd_epi16( melocalk0, melocall0 );
            __m128i mmacc8 = _mm_madd_epi16( melocalk8, melocall8 );

            __m128i mmacc = _mm_add_epi32( mmacc0, mmacc8 );

            vmacc[ll] = mmacc;
          }

          __m128i mmacc = _mm_hadd_epi32( _mm_hadd_epi32( vmacc[0], vmacc[1] ), _mm_hadd_epi32( vmacc[2], vmacc[3] ) );

          __m128 mmaccf = _mm_cvtepi32_ps( mmacc );

          __m128 mcov = _mm_loadu_ps( cov );
          mcov = _mm_add_ps( mcov, mmaccf );
          _mm_storeu_ps( cov, mcov );

          cov += 4;
        }

        for( ; l < shape.numCoeff; l++ )
        {
          const Pel* Elocall = &GET_ALF_COVAR( ELocal, b1, l, 0 );

          __m128i melocall0 = _mm_loadu_si128( ( const __m128i* )&Elocall[0] );
          __m128i melocall8 = _mm_loadu_si128( ( const __m128i* )&Elocall[8] );

          __m128i mmacc0 = _mm_madd_epi16( melocalk0, melocall0 );
          __m128i mmacc8 = _mm_madd_epi16( melocalk8, melocall8 );

          __m128i mmacc = _mm_add_epi32( mmacc0, mmacc8 );
          mmacc = _mm_hadd_epi32( mmacc, mmacc );
          mmacc = _mm_hadd_epi32( mmacc, mmacc );

          __m128 mmaccf = _mm_cvtepi32_ps( mmacc );

          *cov++ += _mm_cvtss_f32( mmaccf );
        }
      }

      const __m128i mmacc0 = _mm_madd_epi16( melocalk0, mylocal0 );
      const __m128i mmacc8 = _mm_madd_epi16( melocalk8, mylocal8 );

      __m128i mmacc = _mm_add_epi32( mmacc0, mmacc8 );
      mmacc = _mm_hadd_epi32( mmacc, mmacc );
      mmacc = _mm_hadd_epi32( mmacc, mmacc );

      alfCovariance.y[b0][k] += _mm_cvtsi128_si32( mmacc );
    }
  }

  const __m128i mmacc0 = _mm_madd_epi16( mylocal0, mylocal0 );
  const __m128i mmacc8 = _mm_madd_epi16( mylocal8, mylocal8 );

  __m128i mmacc = _mm_add_epi32( mmacc0, mmacc8 );
  mmacc = _mm_hadd_epi32( mmacc, mmacc );
  mmacc = _mm_hadd_epi32( mmacc, mmacc );

  alfCovariance.pixAcc += _mm_cvtsi128_si32( mmacc );
}

template<X86_VEXT vext>
void EncAdaptiveLoopFilter::_initEncAdaptiveLoopFilter_X86()
{
  m_getPreBlkStatsAccum = simdGetPreBlkStatsAccum<vext>;
  m_getPreBlkStatsWeightedAccum = simdGetPreBlkStatsWeightedAccum<vext>;
}

template void EncAdaptiveLoopFilter::_initEncAdaptiveLoopFilter_X86<SIMDX86>();

#undef GET_ALF_COVAR
#undef NL_COVAR_xstride
#undef NL_COVAR_kstride
#undef NL_COVAR_bstride

} // namespace vvenc

#endif // TARGET_SIMD_X86

//! \}


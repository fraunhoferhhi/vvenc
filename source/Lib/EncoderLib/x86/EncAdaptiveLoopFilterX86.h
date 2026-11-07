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

// A row of 16 Pels of ELocal is loaded once and multiplied with many other rows.
// alfRowDotPartials: its products with the 16 Pels at row, summed pairwise into 4 int32 partial sums of their dot product.
// alfRowDots4: its dot products with the 4 rows of 16 Pels from rows on, one in each int32.
// Integer sums don't depend on their order, so both builds give the same values.
#ifdef USE_AVX2
typedef __m256i AlfRow;

static inline AlfRow alfLoadRow( const Pel* row )
{
  return _mm256_loadu_si256( ( const __m256i* )row );
}

static inline __m128i alfAddLanes( const __m256i value )
{
  return _mm_add_epi32( _mm256_castsi256_si128( value ), _mm256_extracti128_si256( value, 1 ) );
}

static inline __m128i alfRowDotPartials( const AlfRow rowK, const Pel* row )
{
  return alfAddLanes( _mm256_madd_epi16( rowK, alfLoadRow( row ) ) );
}

static inline __m128i alfRowDots4( const AlfRow rowK, const Pel* rows )
{
  const __m256i dot0 = _mm256_madd_epi16( rowK, alfLoadRow( rows + 0 * NL_COVAR_kstride ) );
  const __m256i dot1 = _mm256_madd_epi16( rowK, alfLoadRow( rows + 1 * NL_COVAR_kstride ) );
  const __m256i dot2 = _mm256_madd_epi16( rowK, alfLoadRow( rows + 2 * NL_COVAR_kstride ) );
  const __m256i dot3 = _mm256_madd_epi16( rowK, alfLoadRow( rows + 3 * NL_COVAR_kstride ) );
  return alfAddLanes( _mm256_hadd_epi32( _mm256_hadd_epi32( dot0, dot1 ), _mm256_hadd_epi32( dot2, dot3 ) ) );
}
#else
struct AlfRow
{
  __m128i pels0, pels8;
};

static inline AlfRow alfLoadRow( const Pel* row )
{
  return { _mm_loadu_si128( ( const __m128i* )&row[0] ), _mm_loadu_si128( ( const __m128i* )&row[8] ) };
}

static inline __m128i alfRowDotPartials( const AlfRow& rowK, const Pel* row )
{
  const __m128i mmacc0 = _mm_madd_epi16( rowK.pels0, _mm_loadu_si128( ( const __m128i* )&row[0] ) );
  const __m128i mmacc8 = _mm_madd_epi16( rowK.pels8, _mm_loadu_si128( ( const __m128i* )&row[8] ) );
  return _mm_add_epi32( mmacc0, mmacc8 );
}

static inline __m128i alfRowDots4( const AlfRow& rowK, const Pel* rows )
{
  const __m128i dot0 = alfRowDotPartials( rowK, rows + 0 * NL_COVAR_kstride );
  const __m128i dot1 = alfRowDotPartials( rowK, rows + 1 * NL_COVAR_kstride );
  const __m128i dot2 = alfRowDotPartials( rowK, rows + 2 * NL_COVAR_kstride );
  const __m128i dot3 = alfRowDotPartials( rowK, rows + 3 * NL_COVAR_kstride );
  return _mm_hadd_epi32( _mm_hadd_epi32( dot0, dot1 ), _mm_hadd_epi32( dot2, dot3 ) );
}
#endif

template<X86_VEXT vext>
void simdGetPreBlkStatsAccum( AlfCovariance& alfCovariance, const AlfFilterShape& shape, const Pel* ELocal,
                              const Pel yLocal[4][4], const int numBins )
{
  for( int b0 = 0; b0 < numBins; b0++ )
  {
    for( int k = 0; k < shape.numCoeff; k++ )
    {
      const Pel* Elocalk = &GET_ALF_COVAR( ELocal, b0, k, 0 );

      const AlfRow rowK = alfLoadRow( Elocalk );

      for( int b1 = 0; b1 < numBins; b1++ )
      {
        alf_float_t* cov = &alfCovariance.E[b0][b1][k][k];

        int l = k;

        for( ; l < ( shape.numCoeff - 3 ); l += 4 )
        {
          const __m128i mmacc = alfRowDots4( rowK, &GET_ALF_COVAR( ELocal, b1, l, 0 ) );

          __m128 mmaccf = _mm_cvtepi32_ps( mmacc );

          __m128 mcov = _mm_loadu_ps( cov );
          mcov = _mm_add_ps( mcov, mmaccf );
          _mm_storeu_ps( cov, mcov );

          cov += 4;
        }

        for( ; l < shape.numCoeff; l++ )
        {
          __m128i mmacc = alfRowDotPartials( rowK, &GET_ALF_COVAR( ELocal, b1, l, 0 ) );
          mmacc = _mm_hadd_epi32( mmacc, mmacc );
          mmacc = _mm_hadd_epi32( mmacc, mmacc );

          __m128 mmaccf = _mm_cvtepi32_ps( mmacc );

          *cov++ += _mm_cvtss_f32( mmaccf );
        }
      }

      __m128i mmacc = alfRowDotPartials( rowK, &yLocal[0][0] );
      mmacc = _mm_hadd_epi32( mmacc, mmacc );
      mmacc = _mm_hadd_epi32( mmacc, mmacc );

      alfCovariance.y[b0][k] += _mm_cvtsi128_si32( mmacc );
    }
  }

  __m128i mmacc = alfRowDotPartials( alfLoadRow( &yLocal[0][0] ), &yLocal[0][0] );
  mmacc = _mm_hadd_epi32( mmacc, mmacc );
  mmacc = _mm_hadd_epi32( mmacc, mmacc );

  alfCovariance.pixAcc += _mm_cvtsi128_si32( mmacc );
}

// simdGetPreBlkStatsAccum with all clipping bins for a block whose equal bins form the runs of equalBinsMask
// (AlfEqualBinRuns): each sum is computed once per pair of runs and added, with the same instructions as in
// simdGetPreBlkStatsAccum, to every cell it stands for, so the result is identical
template<X86_VEXT vext>
void simdGetPreBlkStatsAccumEqualBins( AlfCovariance& alfCovariance, const AlfFilterShape& shape, const Pel* ELocal,
                                       const Pel yLocal[4][4], const int equalBinsMask )
{
  const AlfEqualBinRuns runs( equalBinsMask );

  for( int r0 = 0; r0 < runs.numRuns; r0++ )
  {
    const int firstBin0 = runs.first[r0], endBin0 = firstBin0 + runs.size[r0];

    for( int k = 0; k < shape.numCoeff; k++ )
    {
      const Pel* Elocalk = &GET_ALF_COVAR( ELocal, firstBin0, k, 0 );

      const AlfRow rowK = alfLoadRow( Elocalk );

      for( int r1 = 0; r1 < runs.numRuns; r1++ )
      {
        const int firstBin1 = runs.first[r1], endBin1 = firstBin1 + runs.size[r1];

        int l = k;

        for( ; l < ( shape.numCoeff - 3 ); l += 4 )
        {
          const __m128i mmacc = alfRowDots4( rowK, &GET_ALF_COVAR( ELocal, firstBin1, l, 0 ) );

          const __m128 mmaccf = _mm_cvtepi32_ps( mmacc );

          for( int b0 = firstBin0; b0 < endBin0; b0++ )
          {
            for( int b1 = firstBin1; b1 < endBin1; b1++ )
            {
              alf_float_t* cov = &alfCovariance.E[b0][b1][k][l];
              _mm_storeu_ps( cov, _mm_add_ps( _mm_loadu_ps( cov ), mmaccf ) );
            }
          }
        }

        for( ; l < shape.numCoeff; l++ )
        {
          __m128i mmacc = alfRowDotPartials( rowK, &GET_ALF_COVAR( ELocal, firstBin1, l, 0 ) );
          mmacc = _mm_hadd_epi32( mmacc, mmacc );
          mmacc = _mm_hadd_epi32( mmacc, mmacc );

          const float sum = _mm_cvtss_f32( _mm_cvtepi32_ps( mmacc ) );

          for( int b0 = firstBin0; b0 < endBin0; b0++ )
          {
            for( int b1 = firstBin1; b1 < endBin1; b1++ )
            {
              alfCovariance.E[b0][b1][k][l] += sum;
            }
          }
        }
      }

      __m128i mmacc = alfRowDotPartials( rowK, &yLocal[0][0] );
      mmacc = _mm_hadd_epi32( mmacc, mmacc );
      mmacc = _mm_hadd_epi32( mmacc, mmacc );

      const int sum = _mm_cvtsi128_si32( mmacc );

      for( int b0 = firstBin0; b0 < endBin0; b0++ )
      {
        alfCovariance.y[b0][k] += sum;
      }
    }
  }

  __m128i mmacc = alfRowDotPartials( alfLoadRow( &yLocal[0][0] ), &yLocal[0][0] );
  mmacc = _mm_hadd_epi32( mmacc, mmacc );
  mmacc = _mm_hadd_epi32( mmacc, mmacc );

  alfCovariance.pixAcc += _mm_cvtsi128_si32( mmacc );
}

template<X86_VEXT vext>
void EncAdaptiveLoopFilter::_initEncAdaptiveLoopFilter_X86()
{
  m_getPreBlkStatsAccum = simdGetPreBlkStatsAccum<vext>;
  m_getPreBlkStatsWeightedAccum = simdGetPreBlkStatsWeightedAccum<vext>;
  m_getPreBlkStatsAccumEqualBins = simdGetPreBlkStatsAccumEqualBins<vext>;
}

template void EncAdaptiveLoopFilter::_initEncAdaptiveLoopFilter_X86<SIMDX86>();

#undef GET_ALF_COVAR
#undef NL_COVAR_xstride
#undef NL_COVAR_kstride
#undef NL_COVAR_bstride

} // namespace vvenc

#endif // TARGET_SIMD_X86

//! \}


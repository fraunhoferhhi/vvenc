/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2022, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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


/** \file     EncAdaptiveLoopFilter.cpp
 \brief    estimation part of adaptive loop filter class
 */

#include "EncAdaptiveLoopFilter.h"
#include "CommonLib/Picture.h"
#include "CommonLib/CodingStructure.h"
#include "CommonLib/dtrace_buffer.h"
#include "CommonLib/TimeProfiler.h"

#include <algorithm>

#include "vvenc/vvencCfg.h"
#include "Utilities/NoMallocThreadPool.h"


//! \ingroup EncoderLib
//! \{

namespace vvenc {

#define OPTIMIZE_ORIG_CASE 0

#ifdef TRACE_ENABLE_ITT
static __itt_string_handle* itt_handle_pre = __itt_string_handle_create( "ALF_pre" );
static __itt_domain* itt_domain_ALF_pre   = __itt_domain_create( "ALFPre" );
#endif

struct FilterIdxCount
{
  uint64_t count;
  uint8_t filterIdx;
};

bool compareCounts(FilterIdxCount a, FilterIdxCount b) { return a.count > b.count; }

#define AlfCtx(c) SubCtx( Ctx::Alf, c)

void AlfCovariance::getClipMax(const AlfFilterShape& alfShape, int *clip_max) const
{
  for( int k = 0; k < numCoeff-1; ++k )
  {
    clip_max[k] = 0;

    bool inc = true;
    while( inc && clip_max[k]+1 < numBins && y[clip_max[k]+1][k] == y[clip_max[k]][k] )
    {
      for( int l = 0; inc && l < numCoeff; ++l )
        if( E[clip_max[k]][0][k][l] != E[clip_max[k]+1][0][k][l] )
        {
          inc = false;
        }
      if( inc )
      {
        ++clip_max[k];
      }
    }
  }
  clip_max[numCoeff-1] = 0;
}

void AlfCovariance::reduceClipCost(const AlfFilterShape& alfShape, int *clip) const
{
  for( int k = 0; k < numCoeff-1; ++k )
  {
    bool dec = true;
    while( dec && clip[k] > 0 && y[clip[k]-1][k] == y[clip[k]][k] )
    {
      for( int l = 0; dec && l < numCoeff; ++l )
        if( E[clip[k]][clip[l]][k][l] != E[clip[k]-1][clip[l]][k][l] )
        {
          dec = false;
        }
      if( dec )
      {
        --clip[k];
      }
    }
  }
}

alf_float_t AlfCovariance::optimizeFilter(const AlfFilterShape& alfShape, int* clip, alf_float_t *f, bool optimize_clip) const
{
  const int size = alfShape.numCoeff;
  int clip_max[MAX_NUM_ALF_LUMA_COEFF];

  alf_float_t err_best, err_last;

  TE kE;
  Ty ky;

  if( optimize_clip )
  {
    // Start by looking for min clipping that has no impact => max_clipping
    getClipMax(alfShape, clip_max);
    for (int k=0; k<size; ++k)
    {
      clip[k] = std::max(clip_max[k], clip[k]);
      clip[k] = std::min(clip[k], numBins-1);
    }
  }

  setEyFromClip( clip, kE, ky, size );

  gnsSolveByChol( kE, ky, f, size );
  err_best = calculateError( clip, f, size );

  int step = optimize_clip ? (numBins+1)/2 : 0;

  while( step > 0 )
  {
    alf_float_t err_min = err_best;
    int idx_min = -1;
    int inc_min = 0;

    for( int k = 0; k < size-1; ++k )
    {
      if( clip[k] - step >= clip_max[k] )
      {
        clip[k] -= step;
        ky[k] = y[clip[k]][k];
        for( int l = 0; l < size; l++ )
        {
          kE[k][l] = E[clip[k]][clip[l]][k][l];
          kE[l][k] = E[clip[l]][clip[k]][l][k];
        }

        gnsSolveByChol( kE, ky, f, size );
        err_last = calculateError( clip, f, size );

        if( err_last < err_min )
        {
          err_min = err_last;
          idx_min = k;
          inc_min = -step;
        }
        clip[k] += step;
      }
      if( clip[k] + step < numBins )
      {
        clip[k] += step;
        ky[k] = y[clip[k]][k];
        for( int l = 0; l < size; l++ )
        {
          kE[k][l] = E[clip[k]][clip[l]][k][l];
          kE[l][k] = E[clip[l]][clip[k]][l][k];
        }

        gnsSolveByChol( kE, ky, f, size );
        err_last = calculateError( clip, f, size );

        if( err_last < err_min )
        {
          err_min = err_last;
          idx_min = k;
          inc_min = step;
        }
        clip[k] -= step;

      }
      ky[k] = y[clip[k]][k];
      for( int l = 0; l < size; l++ )
      {
        kE[k][l] = E[clip[k]][clip[l]][k][l];
        kE[l][k] = E[clip[l]][clip[k]][l][k];
      }
    }

    if( idx_min >= 0 )
    {
      err_best = err_min;
      clip[idx_min] += inc_min;
      ky[idx_min] = y[clip[idx_min]][idx_min];
      for( int l = 0; l < size; l++ )
      {
        kE[idx_min][l] = E[clip[idx_min]][clip[l]][idx_min][l];
        kE[l][idx_min] = E[clip[l]][clip[idx_min]][l][idx_min];
      }
    }
    else
    {
      --step;
    }
  }

  if( optimize_clip ) {
    // test all max
    for( int k = 0; k < size-1; ++k )
    {
      clip_max[k] = 0;
    }
    TE kE_max;
    Ty ky_max;
    setEyFromClip( clip_max, kE_max, ky_max, size );

    gnsSolveByChol( kE_max, ky_max, f, size );
    err_last = calculateError( clip_max, f, size );
    if( err_last < err_best )
    {
      err_best = err_last;
      for (int k=0; k<size; ++k)
      {
        clip[k] = clip_max[k];
      }
    }
    else
    {
      // update clip to reduce coding cost
      reduceClipCost(alfShape, clip);

      // update f with best solution
      gnsSolveByChol( kE, ky, f, size );
    }
  }

  return err_best;
}

template<>
alf_float_t AlfCovariance::calcDiffErrorForCoeffs<false>( const int *clip, const int *coeff, const int numCoeff, const int coeffBefore, const int coeffAfter, const int coeffPos, const alf_float_t invFactor ) const
{
  alf_float_t error = 0;
  const int coeffDiff = (coeffAfter-coeffBefore);
  for( int i = 0; i < numCoeff; i++ )
  {
    if( i == coeffPos )
    {
      alf_float_t sum = 0;
      for( int j = i + 1; j < numCoeff; j++ )
      {
        sum += E[0][0][i][j] * coeff[j];
      }
      error += (invFactor * sum - y[0][i]) * coeffDiff * 2;
      error += invFactor * ( E[0][0][i][i] * (coeffAfter * coeffAfter - coeffBefore * coeffBefore));
      break;
    }
    else
    {
      for( int j = i + 1; j < numCoeff; j++ )
      {
        if( j == coeffPos )
        {
          error += 2  * invFactor * E[0][0][i][j] * coeffDiff  *  coeff[i];
        }
      }
    }
  }

  return error * invFactor;
}

template<>
alf_float_t AlfCovariance::calcDiffErrorForCoeffs<true>( const int *clip, const int *coeff, const int numCoeff, const int coeffBefore, const int coeffAfter, const int coeffPos, const alf_float_t invFactor ) const
{
  alf_float_t error = 0;
  const int coeffDiff = (coeffAfter-coeffBefore);
  for( int i = 0; i < numCoeff; i++ )
  {
    if( i == coeffPos )
    {
      alf_float_t sum = 0;
      for( int j = i + 1; j < numCoeff; j++ )
      {
        sum += E[clip[i]][clip[j]][i][j] * coeff[j];
      }
      error += (invFactor * sum - y[clip[i]][i]) * coeffDiff * 2;
      error += invFactor * ( E[clip[i]][clip[i]][i][i] * (coeffAfter * coeffAfter - coeffBefore * coeffBefore));
      break;
    }
    else
    {
      for( int j = i + 1; j < numCoeff; j++ )
      {
        if( j == coeffPos )
        {
          error += 2  * invFactor * E[clip[i]][clip[j]][i][j] * coeffDiff  *  coeff[i];
        }
      }
    }
  }

  return error * invFactor;
}



template<int numCoeff>
static alf_float_t calcErrorForCoeffsLin( const AlfCovariance::TKE& E, const AlfCovariance::TKy& y, const int* coeff, const alf_float_t invFactor )
{
  alf_float_t error = 0;

  for( int i = 0; i < numCoeff; i++ )   //diagonal
  {
    alf_float_t sum = 0;
    for( int j = i + 1; j < numCoeff; j++ )
    {
      // E[j][i] = E[i][j], sum will be multiplied by 2 later
      sum += E[0][0][i][j] * coeff[j];
    }
    error += ( ( E[0][0][i][i] * coeff[i] + sum * 2 ) * invFactor - 2 * y[0][i] ) * coeff[i];
  }

  return error * invFactor;
}

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_ALF
static alf_float_t calcErrorForCoeffsLin_13_SSE( const AlfCovariance::TKE& E, const AlfCovariance::TKy& y, const int* coeff, const alf_float_t invFactor )
{
#if ALF_SINGLE_PREC_FLOAT
  //alf_float_t orgerror = 0;
  //
  //for( int i = 0; i < 13; i++ )   //diagonal
  //{
  //  alf_float_t sum = 0;
  //  for( int j = i + 1; j < 13; j++ )
  //  {
  //    // E[j][i] = E[i][j], sum will be multiplied by 2 later
  //    sum += E[0][0][i][j] * coeff[j];
  //  }
  //  orgerror += ( ( E[0][0][i][i] * coeff[i] + sum * 2 ) * invFactor - 2 * y[0][i] ) * coeff[i];
  //}
  //
  //orgerror *= invFactor;

  alf_float_t error = 0;

  const __m128 mzero = _mm_setzero_ps();
  const __m128 minvf = _mm_set1_ps( invFactor );
  const __m128 mtwo  = _mm_set1_ps( 2.0 );

  __m128 merror  = _mm_setzero_ps(), msum0, msum1, msum2, msum3;
  __m128 mcoef1  = _mm_cvtepi32_ps( _mm_loadu_si128( ( const __m128i* ) &coeff[1] ) );
  __m128 mcoef5  = _mm_cvtepi32_ps( _mm_loadu_si128( ( const __m128i* ) &coeff[5] ) );
  __m128 mcoef9  = _mm_cvtepi32_ps( _mm_loadu_si128( ( const __m128i* ) &coeff[9] ) );

  // i = 0

  __m128 mE1  = _mm_loadu_ps( &E[0][0][0][ 1] );
  __m128 mE5  = _mm_loadu_ps( &E[0][0][0][ 5] );
  __m128 mE9  = _mm_loadu_ps( &E[0][0][0][ 9] );
  
  mE1  = _mm_mul_ps( mcoef1, mE1 );
  mE5  = _mm_mul_ps( mcoef5, mE5 );
  mE9  = _mm_mul_ps( mcoef9, mE9 );

  msum0 = _mm_add_ps( _mm_add_ps( mE1, mE5 ), mE9 );

  // i = 1

  mcoef1 = _mm_blend_ps( mcoef1, mzero, 1 );

  mE1  = _mm_loadu_ps( &E[0][0][1][ 1] );
  mE5  = _mm_loadu_ps( &E[0][0][1][ 5] );
  mE9  = _mm_loadu_ps( &E[0][0][1][ 9] );
  
  mE1  = _mm_mul_ps( mcoef1,  mE1 );
  mE5  = _mm_mul_ps( mcoef5,  mE5 );
  mE9  = _mm_mul_ps( mcoef9,  mE9 );

  msum1 = _mm_add_ps( _mm_add_ps( mE1, mE5 ), mE9 );

  // i = 2

  mcoef1 = _mm_blend_ps( mcoef1, mzero, 2 );

  mE1 = _mm_loadu_ps( &E[0][0][2][1] );
  mE5 = _mm_loadu_ps( &E[0][0][2][5] );
  mE9 = _mm_loadu_ps( &E[0][0][2][9] );

  mE1 = _mm_mul_ps( mcoef1, mE1 );
  mE5 = _mm_mul_ps( mcoef5, mE5 );
  mE9 = _mm_mul_ps( mcoef9, mE9 );

  msum2 = _mm_add_ps( _mm_add_ps( mE1, mE5 ), mE9 );

  // i = 3

  mcoef1 = _mm_blend_ps( mcoef1, mzero, 4 );

  mE1 = _mm_loadu_ps( &E[0][0][3][1] );
  mE5 = _mm_loadu_ps( &E[0][0][3][5] );
  mE9 = _mm_loadu_ps( &E[0][0][3][9] );

  mE1 = _mm_mul_ps( mcoef1, mE1 );
  mE5 = _mm_mul_ps( mcoef5, mE5 );
  mE9 = _mm_mul_ps( mcoef9, mE9 );

  msum3 = _mm_add_ps( _mm_add_ps( mE1, mE5 ), mE9 );

  msum0 = _mm_hadd_ps( msum0, msum1 );
  msum2 = _mm_hadd_ps( msum2, msum3 );
  msum0 = _mm_hadd_ps( msum0, msum2 );

  // combine i = 0...3

  __m128 mcoef   = _mm_cvtepi32_ps( _mm_loadu_si128( ( const __m128i* ) &coeff[0] ) );
  __m128 my      =                  _mm_loadu_ps(                       &y[0][0] );
  __m128 mE      =                  _mm_setr_ps ( E[0][0][0][0], E[0][0][1][1], E[0][0][2][2], E[0][0][3][3] );
  
  mE1 = _mm_mul_ps( mE, mcoef );
  mE5 = _mm_mul_ps( msum0, mtwo );
  mE1 = _mm_add_ps( mE1, mE5 );
  mE1 = _mm_mul_ps( mE1, minvf );
  mE5 = _mm_mul_ps( mtwo, my );
  mE1 = _mm_sub_ps( mE1, mE5 );
  mE1 = _mm_mul_ps( mE1, mcoef );
  merror = _mm_add_ps( merror, mE1 );

  //error += ( ( E[0][0][0][0] * coeff[0] + sum0 * 2 ) * invFactor - 2 * y[0][0] ) * coeff[0];
  //error += ( ( E[0][0][1][1] * coeff[1] + sum1 * 2 ) * invFactor - 2 * y[0][1] ) * coeff[1];
  //error += ( ( E[0][0][2][2] * coeff[2] + sum2 * 2 ) * invFactor - 2 * y[0][2] ) * coeff[2];
  //error += ( ( E[0][0][3][3] * coeff[3] + sum3 * 2 ) * invFactor - 2 * y[0][3] ) * coeff[3];

  // i = 4
  
  //__m128 mE1 = _mm_loadu_ps( &E[0][0][4][1] );
  mE5 = _mm_loadu_ps( &E[0][0][4][5] );
  mE9 = _mm_loadu_ps( &E[0][0][4][9] );

  //mE1 = _mm_mul_ps( mcoef1, mE1 );
  mE5 = _mm_mul_ps( mcoef5, mE5 );
  mE9 = _mm_mul_ps( mcoef9, mE9 );

  msum0 = _mm_add_ps( mE5, mE9 );

  // i = 5

  mcoef5 = _mm_blend_ps( mcoef5, mzero, 1 );

  //mE1 = _mm_loadu_ps( &E[0][0][5][1] );
  mE5 = _mm_loadu_ps( &E[0][0][5][5] );
  mE9 = _mm_loadu_ps( &E[0][0][5][9] );

  //mE1 = _mm_mul_ps( mcoef1, mE1 );
  mE5 = _mm_mul_ps( mcoef5, mE5 );
  mE9 = _mm_mul_ps( mcoef9, mE9 );

  msum1 = _mm_add_ps( mE5, mE9 );

  // i = 6

  mcoef5 = _mm_blend_ps( mcoef5, mzero, 2 );

  //mE1 = _mm_loadu_ps( &E[0][0][6][1] );
  mE5 = _mm_loadu_ps( &E[0][0][6][5] );
  mE9 = _mm_loadu_ps( &E[0][0][6][9] );

  //mE1 = _mm_mul_ps( mcoef1, mE1 );
  mE5 = _mm_mul_ps( mcoef5, mE5 );
  mE9 = _mm_mul_ps( mcoef9, mE9 );

  msum2 = _mm_add_ps( mE5, mE9 );

  // i = 7

  mcoef5 = _mm_blend_ps( mcoef5, mzero, 4 );

  //mE1 = _mm_loadu_ps( &E[0][0][7][1] );
  mE5 = _mm_loadu_ps( &E[0][0][7][5] );
  mE9 = _mm_loadu_ps( &E[0][0][7][9] );

//  mE1 = _mm_mul_ps( mcoef1, mE1 );
  mE5 = _mm_mul_ps( mcoef5, mE5 );
  mE9 = _mm_mul_ps( mcoef9, mE9 );

  msum3 = _mm_add_ps( mE5, mE9 );

  msum0 = _mm_hadd_ps( msum0, msum1 );
  msum2 = _mm_hadd_ps( msum2, msum3 );
  msum0 = _mm_hadd_ps( msum0, msum2 );

  // combine i = 4...7

  mcoef = _mm_cvtepi32_ps( _mm_loadu_si128( ( const __m128i* ) &coeff[4] ) );
  my = _mm_loadu_ps( &y[0][4] );
  mE = _mm_setr_ps( E[0][0][4][4], E[0][0][5][5], E[0][0][6][6], E[0][0][7][7] );

  mE1 = _mm_mul_ps( mE, mcoef );
  mE5 = _mm_mul_ps( msum0, mtwo );
  mE1 = _mm_add_ps( mE1, mE5 );
  mE1 = _mm_mul_ps( mE1, minvf );
  mE5 = _mm_mul_ps( mtwo, my );
  mE1 = _mm_sub_ps( mE1, mE5 );
  mE1 = _mm_mul_ps( mE1, mcoef );
  merror = _mm_add_ps( merror, mE1 );

  //error += ( ( E[0][0][4][4] * coeff[4] + sum0 * 2 ) * invFactor - 2 * y[0][4] ) * coeff[4];
  //error += ( ( E[0][0][5][5] * coeff[5] + sum1 * 2 ) * invFactor - 2 * y[0][5] ) * coeff[5];
  //error += ( ( E[0][0][6][6] * coeff[6] + sum2 * 2 ) * invFactor - 2 * y[0][6] ) * coeff[6];
  //error += ( ( E[0][0][7][7] * coeff[7] + sum3 * 2 ) * invFactor - 2 * y[0][7] ) * coeff[7];

  // i = 8
  
  //__m128 mE1 = _mm_loadu_ps( &E[0][0][8][1] );
  //__m128 mE5 = _mm_loadu_ps( &E[0][0][8][5] );
  mE9 = _mm_loadu_ps( &E[0][0][8][9] );

  //mE1 = _mm_mul_ps( mcoef1, mE1 );
  //mE5 = _mm_mul_ps( mcoef5, mE5 );
  mE9 = _mm_mul_ps( mcoef9, mE9 );

  msum0 = mE9;

  // i = 9

  mcoef9 = _mm_blend_ps( mcoef9, mzero, 1 );

  //mE1 = _mm_loadu_ps( &E[0][0][9][1] );
  //mE5 = _mm_loadu_ps( &E[0][0][9][5] );
  mE9 = _mm_loadu_ps( &E[0][0][9][9] );

  //mE1 = _mm_mul_ps( mcoef1, mE1 );
  //mE5 = _mm_mul_ps( mcoef5, mE5 );
  mE9 = _mm_mul_ps( mcoef9, mE9 );

  msum1 = mE9;

  // i = 10

  mcoef9 = _mm_blend_ps( mcoef9, mzero, 2 );

  //mE1 = _mm_loadu_ps( &E[0][0][10][1] );
  //mE5 = _mm_loadu_ps( &E[0][0][10][5] );
  mE9 = _mm_loadu_ps( &E[0][0][10][9] );

  //mE1 = _mm_mul_ps( mcoef1, mE1 );
  //mE5 = _mm_mul_ps( mcoef5, mE5 );
  mE9 = _mm_mul_ps( mcoef9, mE9 );

  msum2 = mE9;

  // i = 11

  mcoef9 = _mm_blend_ps( mcoef9, mzero, 4 );

  //mE1 = _mm_loadu_ps( &E[0][0][11][1] );
  //mE5 = _mm_loadu_ps( &E[0][0][11][5] );
  mE9 = _mm_loadu_ps( &E[0][0][11][9] );

  //mE1 = _mm_mul_ps( mcoef1, mE1 );
  //mE5 = _mm_mul_ps( mcoef5, mE5 );
  mE9 = _mm_mul_ps( mcoef9, mE9 );

  msum3 = mE9;

  msum0 = _mm_hadd_ps( msum0, msum1 );
  msum2 = _mm_hadd_ps( msum2, msum3 );
  msum0 = _mm_hadd_ps( msum0, msum2 );

  // combine i = 8...11

  mcoef = _mm_cvtepi32_ps( _mm_loadu_si128( ( const __m128i* ) & coeff[8] ) );
  my = _mm_loadu_ps( &y[0][8] );
  mE = _mm_setr_ps( E[0][0][8][8], E[0][0][9][9], E[0][0][10][10], E[0][0][11][11] );

  mE1 = _mm_mul_ps( mE, mcoef );
  mE5 = _mm_mul_ps( msum0, mtwo );
  mE1 = _mm_add_ps( mE1, mE5 );
  mE1 = _mm_mul_ps( mE1, minvf );
  mE5 = _mm_mul_ps( mtwo, my );
  mE1 = _mm_sub_ps( mE1, mE5 );
  mE1 = _mm_mul_ps( mE1, mcoef );
  merror = _mm_add_ps( merror, mE1 );

  //error += ( ( E[0][0][ 8][ 8] * coeff[ 8] + sum0 * 2 ) * invFactor - 2 * y[0][ 8] ) * coeff[ 8];
  //error += ( ( E[0][0][ 9][ 9] * coeff[ 9] + sum1 * 2 ) * invFactor - 2 * y[0][ 9] ) * coeff[ 9];
  //error += ( ( E[0][0][10][10] * coeff[10] + sum2 * 2 ) * invFactor - 2 * y[0][10] ) * coeff[10];
  //error += ( ( E[0][0][11][11] * coeff[11] + sum3 * 2 ) * invFactor - 2 * y[0][11] ) * coeff[11];

  merror = _mm_hadd_ps( merror, mzero );
  merror = _mm_hadd_ps( merror, mzero );
  error  = _mm_cvtss_f32( merror );
  error += ( ( E[0][0][12][12] * coeff[12] ) * invFactor - 2 * y[0][12] ) * coeff[12];

  return error * invFactor;
#else
  alf_float_t error = 0;

  const __m128d mzero = _mm_setzero_pd();
  const __m128d minvf = _mm_set1_pd( invFactor );
  const __m128d mtwo  = _mm_set1_pd( 2.0 );

  __m128d merror  = _mm_setzero_pd(), msum0, msum1;
  __m128d mcoef1  = _mm_cvtepi32_pd( _mm_loadl_epi64( ( const __m128i* ) &coeff [1] ) );
  __m128d mcoef3  = _mm_cvtepi32_pd( _mm_loadl_epi64( ( const __m128i* ) &coeff [3] ) );
  __m128d mcoef5  = _mm_cvtepi32_pd( _mm_loadl_epi64( ( const __m128i* ) &coeff [5] ) );
  __m128d mcoef7  = _mm_cvtepi32_pd( _mm_loadl_epi64( ( const __m128i* ) &coeff [7] ) );
  __m128d mcoef9  = _mm_cvtepi32_pd( _mm_loadl_epi64( ( const __m128i* ) &coeff [9] ) );
  __m128d mcoef11 = _mm_cvtepi32_pd( _mm_loadl_epi64( ( const __m128i* ) &coeff[11] ) );


  __m128d mE1  = _mm_loadu_pd( &E[0][0][0][ 1] );
  __m128d mE3  = _mm_loadu_pd( &E[0][0][0][ 3] );
  __m128d mE5  = _mm_loadu_pd( &E[0][0][0][ 5] );
  __m128d mE7  = _mm_loadu_pd( &E[0][0][0][ 7] );
  __m128d mE9  = _mm_loadu_pd( &E[0][0][0][ 9] );
  __m128d mE11 = _mm_loadu_pd( &E[0][0][0][11] );
  
  mE1  = _mm_mul_pd( mcoef1,  mE1 );
  mE3  = _mm_mul_pd( mcoef3,  mE3 );
  mE5  = _mm_mul_pd( mcoef5,  mE5 );
  mE7  = _mm_mul_pd( mcoef7,  mE7 );
  mE9  = _mm_mul_pd( mcoef9,  mE9 );
  mE11 = _mm_mul_pd( mcoef11, mE11 );

  msum0 = _mm_add_pd( _mm_add_pd( mE1, mE3 ), _mm_add_pd( _mm_add_pd( mE5, mE7 ), _mm_add_pd( mE9, mE11 ) ) );


  mcoef1 = _mm_blend_pd( mcoef1, mzero, 1 );

  mE1  = _mm_loadu_pd( &E[0][0][1][ 1] );
  mE3  = _mm_loadu_pd( &E[0][0][1][ 3] );
  mE5  = _mm_loadu_pd( &E[0][0][1][ 5] );
  mE7  = _mm_loadu_pd( &E[0][0][1][ 7] );
  mE9  = _mm_loadu_pd( &E[0][0][1][ 9] );
  mE11 = _mm_loadu_pd( &E[0][0][1][11] );
  
  mE1  = _mm_mul_pd( mcoef1,  mE1 );
  mE3  = _mm_mul_pd( mcoef3,  mE3 );
  mE5  = _mm_mul_pd( mcoef5,  mE5 );
  mE7  = _mm_mul_pd( mcoef7,  mE7 );
  mE9  = _mm_mul_pd( mcoef9,  mE9 );
  mE11 = _mm_mul_pd( mcoef11, mE11 );

  msum1 = _mm_add_pd( _mm_add_pd( mE1, mE3 ), _mm_add_pd( _mm_add_pd( mE5, mE7 ), _mm_add_pd( mE9, mE11 ) ) );

  msum0 = _mm_hadd_pd( msum0, msum1 );
  

  
  __m128d mcoef   = _mm_cvtepi32_pd( _mm_loadl_epi64( ( const __m128i* ) &coeff[0] ) );
  __m128d my      =                  _mm_loadu_pd(                       &y[0][0] );
  __m128d mE      =                  _mm_setr_pd( E[0][0][0][0], E[0][0][1][1] );
  
  mE1 = _mm_mul_pd( mE, mcoef );
  mE3 = _mm_mul_pd( msum0, mtwo );
  mE1 = _mm_add_pd( mE1, mE3 );
  mE1 = _mm_mul_pd( mE1, minvf );
  mE3 = _mm_mul_pd( mtwo, my );
  mE1 = _mm_sub_pd( mE1, mE3 );
  mE1 = _mm_mul_pd( mE1, mcoef );
  merror = _mm_add_pd( merror, mE1 );

  //error += ( ( E[0][0][0][0] * coeff[0] + sum0 * 2 ) * invFactor - 2 * y[0][0] ) * coeff[0];
  //error += ( ( E[0][0][1][1] * coeff[1] + sum1 * 2 ) * invFactor - 2 * y[0][1] ) * coeff[1];
  
  
  
  //mcoef1 = _mm_blend_pd( mcoef1, mzero, 1 );

  //mE1  = _mm_loadu_pd( &E[0][0][2][ 1] );
  mE3  = _mm_loadu_pd( &E[0][0][2][ 3] );
  mE5  = _mm_loadu_pd( &E[0][0][2][ 5] );
  mE7  = _mm_loadu_pd( &E[0][0][2][ 7] );
  mE9  = _mm_loadu_pd( &E[0][0][2][ 9] );
  mE11 = _mm_loadu_pd( &E[0][0][2][11] );
  
  //mE1  = _mm_mul_pd( mcoef1,  mE1 );
  mE3  = _mm_mul_pd( mcoef3,  mE3 );
  mE5  = _mm_mul_pd( mcoef5,  mE5 );
  mE7  = _mm_mul_pd( mcoef7,  mE7 );
  mE9  = _mm_mul_pd( mcoef9,  mE9 );
  mE11 = _mm_mul_pd( mcoef11, mE11 );

  msum0 = _mm_add_pd( mE3, _mm_add_pd( _mm_add_pd( mE5, mE7 ), _mm_add_pd( mE9, mE11 ) ) );


  mcoef3 = _mm_blend_pd( mcoef3, mzero, 1 );

  //mE1  = _mm_loadu_pd( &E[0][0][3][ 1] );
  mE3  = _mm_loadu_pd( &E[0][0][3][ 3] );
  mE5  = _mm_loadu_pd( &E[0][0][3][ 5] );
  mE7  = _mm_loadu_pd( &E[0][0][3][ 7] );
  mE9  = _mm_loadu_pd( &E[0][0][3][ 9] );
  mE11 = _mm_loadu_pd( &E[0][0][3][11] );
  
  //mE1  = _mm_mul_pd( mcoef1,  mE1 );
  mE3  = _mm_mul_pd( mcoef3,  mE3 );
  mE5  = _mm_mul_pd( mcoef5,  mE5 );
  mE7  = _mm_mul_pd( mcoef7,  mE7 );
  mE9  = _mm_mul_pd( mcoef9,  mE9 );
  mE11 = _mm_mul_pd( mcoef11, mE11 );

  msum1 = _mm_add_pd( mE3, _mm_add_pd( _mm_add_pd( mE5, mE7 ), _mm_add_pd( mE9, mE11 ) ) );
  msum0 = _mm_hadd_pd( msum0, msum1 );

  
  
  mcoef   = _mm_cvtepi32_pd( _mm_loadl_epi64( ( const __m128i* ) &coeff[2] ) );
  my      =                  _mm_loadu_pd(                       &y[0][2] );
  mE      =                  _mm_setr_pd( E[0][0][2][2], E[0][0][3][3] );
  
  mE1 = _mm_mul_pd( mE, mcoef );
  mE3 = _mm_mul_pd( msum0, mtwo );
  mE1 = _mm_add_pd( mE1, mE3 );
  mE1 = _mm_mul_pd( mE1, minvf );
  mE3 = _mm_mul_pd( mtwo, my );
  mE1 = _mm_sub_pd( mE1, mE3 );
  mE1 = _mm_mul_pd( mE1, mcoef );
  merror = _mm_add_pd( merror, mE1 );
  
  //error += ( ( E[0][0][2][2] * coeff[2] + sum0 * 2 ) * invFactor - 2 * y[0][2] ) * coeff[2];
  //error += ( ( E[0][0][3][3] * coeff[3] + sum1 * 2 ) * invFactor - 2 * y[0][3] ) * coeff[3];


  
  //mcoef3 = _mm_blend_pd( mcoef3, mzero, 1 );

  //mE1  = _mm_loadu_pd( &E[0][0][4][ 1] );
  //mE3  = _mm_loadu_pd( &E[0][0][4][ 3] );
  mE5  = _mm_loadu_pd( &E[0][0][4][ 5] );
  mE7  = _mm_loadu_pd( &E[0][0][4][ 7] );
  mE9  = _mm_loadu_pd( &E[0][0][4][ 9] );
  mE11 = _mm_loadu_pd( &E[0][0][4][11] );
  
  //mE1  = _mm_mul_pd( mcoef1,  mE1 );
  //mE3  = _mm_mul_pd( mcoef3,  mE3 );
  mE5  = _mm_mul_pd( mcoef5,  mE5 );
  mE7  = _mm_mul_pd( mcoef7,  mE7 );
  mE9  = _mm_mul_pd( mcoef9,  mE9 );
  mE11 = _mm_mul_pd( mcoef11, mE11 );

  msum0 = _mm_add_pd( _mm_add_pd( mE5, mE7 ), _mm_add_pd( mE9, mE11 ) );


  mcoef5 = _mm_blend_pd( mcoef5, mzero, 1 );

  //mE1  = _mm_loadu_pd( &E[0][0][5][ 1] );
  //mE3  = _mm_loadu_pd( &E[0][0][5][ 3] );
  mE5  = _mm_loadu_pd( &E[0][0][5][ 5] );
  mE7  = _mm_loadu_pd( &E[0][0][5][ 7] );
  mE9  = _mm_loadu_pd( &E[0][0][5][ 9] );
  mE11 = _mm_loadu_pd( &E[0][0][5][11] );
  
  //mE1  = _mm_mul_pd( mcoef1,  mE1 );
  //mE3  = _mm_mul_pd( mcoef3,  mE3 );
  mE5  = _mm_mul_pd( mcoef5,  mE5 );
  mE7  = _mm_mul_pd( mcoef7,  mE7 );
  mE9  = _mm_mul_pd( mcoef9,  mE9 );
  mE11 = _mm_mul_pd( mcoef11, mE11 );

  msum1 = _mm_add_pd( _mm_add_pd( mE5, mE7 ), _mm_add_pd( mE9, mE11 ) );
  msum0 = _mm_hadd_pd( msum0, msum1 );



  mcoef   = _mm_cvtepi32_pd( _mm_loadl_epi64( ( const __m128i* ) &coeff[4] ) );
  my      =                  _mm_loadu_pd(                       &y[0][4] );
  mE      =                  _mm_setr_pd( E[0][0][4][4], E[0][0][5][5] );
  
  mE1 = _mm_mul_pd( mE, mcoef );
  mE3 = _mm_mul_pd( msum0, mtwo );
  mE1 = _mm_add_pd( mE1, mE3 );
  mE1 = _mm_mul_pd( mE1, minvf );
  mE3 = _mm_mul_pd( mtwo, my );
  mE1 = _mm_sub_pd( mE1, mE3 );
  mE1 = _mm_mul_pd( mE1, mcoef );
  merror = _mm_add_pd( merror, mE1 );
  
  //error += ( ( E[0][0][4][4] * coeff[4] + sum0 * 2 ) * invFactor - 2 * y[0][4] ) * coeff[4];
  //error += ( ( E[0][0][5][5] * coeff[5] + sum1 * 2 ) * invFactor - 2 * y[0][5] ) * coeff[5];
  

  
  //mcoef5 = _mm_blend_pd( mcoef5, mzero, 1 );

  //mE1  = _mm_loadu_pd( &E[0][0][6][ 1] );
  //mE3  = _mm_loadu_pd( &E[0][0][6][ 3] );
  //mE5  = _mm_loadu_pd( &E[0][0][6][ 5] );
  mE7  = _mm_loadu_pd( &E[0][0][6][ 7] );
  mE9  = _mm_loadu_pd( &E[0][0][6][ 9] );
  mE11 = _mm_loadu_pd( &E[0][0][6][11] );
  
  //mE1  = _mm_mul_pd( mcoef1,  mE1 );
  //mE3  = _mm_mul_pd( mcoef3,  mE3 );
  //mE5  = _mm_mul_pd( mcoef5,  mE5 );
  mE7  = _mm_mul_pd( mcoef7,  mE7 );
  mE9  = _mm_mul_pd( mcoef9,  mE9 );
  mE11 = _mm_mul_pd( mcoef11, mE11 );

  msum0 = _mm_add_pd( mE7, _mm_add_pd( mE9, mE11 ) );


  mcoef7 = _mm_blend_pd( mcoef7, mzero, 1 );

  //mE1  = _mm_loadu_pd( &E[0][0][7][ 1] );
  //mE3  = _mm_loadu_pd( &E[0][0][7][ 3] );
  //mE5  = _mm_loadu_pd( &E[0][0][7][ 5] );
  mE7  = _mm_loadu_pd( &E[0][0][7][ 7] );
  mE9  = _mm_loadu_pd( &E[0][0][7][ 9] );
  mE11 = _mm_loadu_pd( &E[0][0][7][11] );
  
  //mE1  = _mm_mul_pd( mcoef1,  mE1 );
  //mE3  = _mm_mul_pd( mcoef3,  mE3 );
  //mE5  = _mm_mul_pd( mcoef5,  mE5 );
  mE7  = _mm_mul_pd( mcoef7,  mE7 );
  mE9  = _mm_mul_pd( mcoef9,  mE9 );
  mE11 = _mm_mul_pd( mcoef11, mE11 );

  msum1 = _mm_add_pd( mE7, _mm_add_pd( mE9, mE11 ) );
  msum0 = _mm_hadd_pd( msum0, msum1 );



  mcoef   = _mm_cvtepi32_pd( _mm_loadl_epi64( ( const __m128i* ) &coeff[6] ) );
  my      =                  _mm_loadu_pd(                       &y[0][6] );
  mE      =                  _mm_setr_pd( E[0][0][6][6], E[0][0][7][7] );
  
  mE1 = _mm_mul_pd( mE, mcoef );
  mE3 = _mm_mul_pd( msum0, mtwo );
  mE1 = _mm_add_pd( mE1, mE3 );
  mE1 = _mm_mul_pd( mE1, minvf );
  mE3 = _mm_mul_pd( mtwo, my );
  mE1 = _mm_sub_pd( mE1, mE3 );
  mE1 = _mm_mul_pd( mE1, mcoef );
  merror = _mm_add_pd( merror, mE1 );
  
  //error += ( ( E[0][0][6][6] * coeff[6] + sum0 * 2 ) * invFactor - 2 * y[0][6] ) * coeff[6];
  //error += ( ( E[0][0][7][7] * coeff[7] + sum1 * 2 ) * invFactor - 2 * y[0][7] ) * coeff[7];
  

  
  //mcoef7 = _mm_blend_pd( mcoef7, mzero, 1 );

  //mE1  = _mm_loadu_pd( &E[0][0][8][ 1] );
  //mE3  = _mm_loadu_pd( &E[0][0][8][ 3] );
  //mE5  = _mm_loadu_pd( &E[0][0][8][ 5] );
  //mE7  = _mm_loadu_pd( &E[0][0][8][ 7] );
  mE9  = _mm_loadu_pd( &E[0][0][8][ 9] );
  mE11 = _mm_loadu_pd( &E[0][0][8][11] );
  
  //mE1  = _mm_mul_pd( mcoef1,  mE1 );
  //mE3  = _mm_mul_pd( mcoef3,  mE3 );
  //mE5  = _mm_mul_pd( mcoef5,  mE5 );
  //mE7  = _mm_mul_pd( mcoef7,  mE7 );
  mE9  = _mm_mul_pd( mcoef9,  mE9 );
  mE11 = _mm_mul_pd( mcoef11, mE11 );

  msum0 = _mm_add_pd( mE9, mE11 );

  
  mcoef9 = _mm_blend_pd( mcoef9, mzero, 1 );

  //mE1  = _mm_loadu_pd( &E[0][0][9][ 1] );
  //mE3  = _mm_loadu_pd( &E[0][0][9][ 3] );
  //mE5  = _mm_loadu_pd( &E[0][0][9][ 5] );
  //mE7  = _mm_loadu_pd( &E[0][0][9][ 7] );
  mE9  = _mm_loadu_pd( &E[0][0][9][ 9] );
  mE11 = _mm_loadu_pd( &E[0][0][9][11] );
  
  //mE1  = _mm_mul_pd( mcoef1,  mE1 );
  //mE3  = _mm_mul_pd( mcoef3,  mE3 );
  //mE5  = _mm_mul_pd( mcoef5,  mE5 );
  //mE7  = _mm_mul_pd( mcoef7,  mE7 );
  mE9  = _mm_mul_pd( mcoef9,  mE9 );
  mE11 = _mm_mul_pd( mcoef11, mE11 );

  msum1 = _mm_add_pd( mE9, mE11 );
  msum0 = _mm_hadd_pd( msum0, msum1 );



  mcoef   = _mm_cvtepi32_pd( _mm_loadl_epi64( ( const __m128i* ) &coeff[8] ) );
  my      =                  _mm_loadu_pd(                       &y[0][8] );
  mE      =                  _mm_setr_pd( E[0][0][8][8], E[0][0][9][9] );
  
  mE1 = _mm_mul_pd( mE, mcoef );
  mE3 = _mm_mul_pd( msum0, mtwo );
  mE1 = _mm_add_pd( mE1, mE3 );
  mE1 = _mm_mul_pd( mE1, minvf );
  mE3 = _mm_mul_pd( mtwo, my );
  mE1 = _mm_sub_pd( mE1, mE3 );
  mE1 = _mm_mul_pd( mE1, mcoef );
  merror = _mm_add_pd( merror, mE1 );
  
  //error += ( ( E[0][0][8][8] * coeff[8] + sum0 * 2 ) * invFactor - 2 * y[0][8] ) * coeff[8];
  //error += ( ( E[0][0][9][9] * coeff[9] + sum1 * 2 ) * invFactor - 2 * y[0][9] ) * coeff[9];
  

  
  //mcoef9 = _mm_blend_pd( mcoef9, mzero, 1 );

  //mE1  = _mm_loadu_pd( &E[0][0][10][ 1] );
  //mE3  = _mm_loadu_pd( &E[0][0][10][ 3] );
  //mE5  = _mm_loadu_pd( &E[0][0][10][ 5] );
  //mE7  = _mm_loadu_pd( &E[0][0][10][ 7] );
  //mE9  = _mm_loadu_pd( &E[0][0][10][ 9] );
  mE11 = _mm_loadu_pd( &E[0][0][10][11] );
  
  //mE1  = _mm_mul_pd( mcoef1,  mE1 );
  //mE3  = _mm_mul_pd( mcoef3,  mE3 );
  //mE5  = _mm_mul_pd( mcoef5,  mE5 );
  //mE7  = _mm_mul_pd( mcoef7,  mE7 );
  //mE9  = _mm_mul_pd( mcoef9,  mE9 );
  mE11 = _mm_mul_pd( mcoef11, mE11 );

  msum0 = mE11;


  mcoef11 = _mm_blend_pd( mcoef11, mzero, 1 );

  //mE1  = _mm_loadu_pd( &E[0][0][1][ 1] );
  //mE3  = _mm_loadu_pd( &E[0][0][1][ 3] );
  //mE5  = _mm_loadu_pd( &E[0][0][1][ 5] );
  //mE7  = _mm_loadu_pd( &E[0][0][1][ 7] );
  //mE9  = _mm_loadu_pd( &E[0][0][1][ 9] );
  mE11 = _mm_loadu_pd( &E[0][0][11][11] );

  //mE1  = _mm_mul_pd( mcoef1,  mE1 );
  //mE3  = _mm_mul_pd( mcoef3,  mE3 );
  //mE5  = _mm_mul_pd( mcoef5,  mE5 );
  //mE7  = _mm_mul_pd( mcoef7,  mE7 );
  //mE9  = _mm_mul_pd( mcoef9,  mE9 );
  mE11 = _mm_mul_pd( mcoef11, mE11 );

  msum1 = mE11;
  msum0 = _mm_hadd_pd( msum0, msum1 );



  mcoef   = _mm_cvtepi32_pd( _mm_loadl_epi64( ( const __m128i* ) &coeff[10] ) );
  my      =                  _mm_loadu_pd(                       &y[0][10] );
  mE      =                  _mm_setr_pd( E[0][0][10][10], E[0][0][11][11] );
  
  mE1 = _mm_mul_pd( mE, mcoef );
  mE3 = _mm_mul_pd( msum0, mtwo );
  mE1 = _mm_add_pd( mE1, mE3 );
  mE1 = _mm_mul_pd( mE1, minvf );
  mE3 = _mm_mul_pd( mtwo, my );
  mE1 = _mm_sub_pd( mE1, mE3 );
  mE1 = _mm_mul_pd( mE1, mcoef );
  merror = _mm_add_pd( merror, mE1 );
  
  //error += ( ( E[0][0][10][10] * coeff[10] + sum0 * 2 ) * invFactor - 2 * y[0][10] ) * coeff[10];
  //error += ( ( E[0][0][11][11] * coeff[11] + sum1 * 2 ) * invFactor - 2 * y[0][11] ) * coeff[11];

  error  = _mm_cvtsd_f64( _mm_hadd_pd( merror, mzero ) );
  error += ( ( E[0][0][12][12] * coeff[12] ) * invFactor - 2 * y[0][12] ) * coeff[12];

  return error * invFactor;
#endif
}
#endif

const AlfCovariance& AlfCovariance::operator+= ( const AlfCovariance& src )
{
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_ALF && ALF_SINGLE_PREC_FLOAT
  if( numCoeff == 13 && read_x86_extension_flags() > SCALAR )
  {
    for( int b0 = 0; b0 < numBins; b0++ )
    {
      for( int b1 = 0; b1 < numBins; b1++ )
      {
        //for( int j = 0; j < numCoeff; j++ )
        //{
        //  for( int i = 0; i < numCoeff; i++ )
        //  {
        //    E[b0][b1][j][i] += src.E[b0][b1][j][i];
        //  }
        //}
        const int sumLen = numCoeff * numCoeff;

        CHECKD( ( sumLen & 3 ) != 1, "sumLen has to have 1 extra element over multiples of 4" );
        
              alf_float_t* d = &    E[b0][b1][0][0];
        const alf_float_t* s = &src.E[b0][b1][0][0];
        
        int i = 0;
        for( ; i < ( sumLen - 1 ); i += 4 )
        {
          __m128 m1 = _mm_loadu_ps( &d[i] );
          __m128 m2 = _mm_loadu_ps( &s[i] );
          _mm_storeu_ps( &d[i], _mm_add_ps( m1, m2 ) );
        }

        d[sumLen - 1] += s[sumLen - 1];
      }
    }

    for( int b = 0; b < numBins; b++ )
    {
            alf_float_t* d = &    y[b][0];
      const alf_float_t* s = &src.y[b][0];
      int i = 0;
      for( ; i < ( numCoeff - 1 ); i += 4 )
      {
        __m128 m1 = _mm_loadu_ps( &d[i] );
        __m128 m2 = _mm_loadu_ps( &s[i] );
        _mm_storeu_ps( &d[i], _mm_add_ps( m1, m2 ) );
      }

      d[numCoeff - 1] += s[numCoeff - 1];
    }
  }
  else
#endif
  {
    for( int b0 = 0; b0 < numBins; b0++ )
    {
      for( int b1 = 0; b1 < numBins; b1++ )
      {
        for( int j = 0; j < numCoeff; j++ )
        {
          for( int i = 0; i < numCoeff; i++ )
          {
            E[b0][b1][j][i] += src.E[b0][b1][j][i];
          }
        }
      }
    }

    for( int b = 0; b < numBins; b++ )
    {
      for( int j = 0; j < numCoeff; j++ )
      {
        y[b][j] += src.y[b][j];
      }
    }
  }

  pixAcc += src.pixAcc;
  all0 &= src.all0;
  return *this;
}

template<int numCoeff>
static alf_float_t calcErrorForCoeffsNonLin( const AlfCovariance::TKE& E, const AlfCovariance::TKy& y, const int* clip, const int* coeff, const alf_float_t invFactor )
{
  alf_float_t error = 0;

  for( int i = 0; i < numCoeff; i++ )   //diagonal
  {
    alf_float_t sum = 0;
    for( int j = i + 1; j < numCoeff; j++ )
    {
      // E[j][i] = E[i][j], sum will be multiplied by 2 later
      sum += E[clip[i]][clip[j]][i][j] * coeff[j];
    }
    error += ( ( E[clip[i]][clip[i]][i][i] * coeff[i] + sum * 2 ) * invFactor - 2 * y[clip[i]][i] ) * coeff[i];
  }

  return error * invFactor;
}

template<>
alf_float_t AlfCovariance::calcErrorForCoeffs<true>( const int *clip, const int *coeff, const int numCoeff, const alf_float_t invFactor ) const
{
  if( numCoeff ==  7 ) return calcErrorForCoeffsNonLin< 7>( E, y, clip, coeff, invFactor );
  if( numCoeff == 13 ) return calcErrorForCoeffsNonLin<13>( E, y, clip, coeff, invFactor );

  THROW( "Unexpected number of coefficients: " << numCoeff );
  return 0.0;
}

template<>
alf_float_t AlfCovariance::calcErrorForCoeffs<false>( const int *clip, const int *coeff, const int numCoeff, const alf_float_t invFactor ) const
{
  if( numCoeff ==  7 ) return calcErrorForCoeffsLin< 7>( E, y, coeff, invFactor );
  if( numCoeff == 13 )
  {
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_ALF
    if( read_x86_extension_flags() > SCALAR )
      return calcErrorForCoeffsLin_13_SSE( E, y, coeff, invFactor );
    else
#endif
      return calcErrorForCoeffsLin<13>( E, y, coeff, invFactor );
  }

  THROW( "Unexpected number of coefficients: " << numCoeff );
  return 0.0;
}

alf_float_t AlfCovariance::calcErrorForCcAlfCoeffs(const int16_t* coeff, const int numCoeff, const alf_float_t invFactor) const
{
  alf_float_t error = 0;

  for (int i = 0; i < numCoeff; i++)   // diagonal
  {
    alf_float_t sum = 0;
    for (int j = i + 1; j < numCoeff; j++)
    {
      // E[j][i] = E[i][j], sum will be multiplied by 2 later
      sum += E[0][0][i][j] * coeff[j];
    }
    error += ((E[0][0][i][i] * coeff[i] + sum * 2) * invFactor - 2 * y[0][i]) * coeff[i];
  }

  return error * invFactor;
}

alf_float_t AlfCovariance::calculateError( const int *clip, const alf_float_t*coeff, const int numCoeff ) const
{
  alf_float_t sum = 0;
  for( int i = 0; i < numCoeff; i++ )
  {
    sum += coeff[i] * y[clip[i]][i];
  }

  return pixAcc - sum;
}

alf_float_t AlfCovariance::calculateError( const int *clip ) const
{
  Ty c;
  return optimizeFilter( clip, c, numCoeff );
}

//********************************
// Cholesky decomposition
//********************************

#define ROUND(a)  (((a) < 0)? (int)((a) - 0.5) : (int)((a) + 0.5))
#define REG              alf_float_t( 0.0001    )
#define REG_SQR          alf_float_t( 0.0000001 )

//Find filter coeff related
int AlfCovariance::gnsCholeskyDec( TE inpMatr, TE outMatr, int numEq ) const
{
  Ty invDiag;  /* Vector of the inverse of diagonal entries of outMatr */

  for( int i = 0; i < numEq; i++ )
  {
    for( int j = i; j < numEq; j++ )
    {
      /* Compute the scaling factor */
      alf_float_t scale = inpMatr[i][j];
      if( i > 0 )
      {
        for( int k = i - 1; k >= 0; k-- )
        {
          scale -= outMatr[k][j] * outMatr[k][i];
        }
      }

      /* Compute i'th row of outMatr */
      if( i == j )
      {
        if( scale <= REG_SQR ) // if(scale <= 0 )  /* If inpMatr is singular */
        {
          return 0;
        }
        else              /* Normal operation */
          invDiag[i] = 1.0 / ( outMatr[i][i] = sqrt( scale ) );
      }
      else
      {
        outMatr[i][j] = scale * invDiag[i]; /* Upper triangular part          */
        outMatr[j][i] = 0.0;              /* Lower triangular part set to 0 */
      }
    }
  }
  return 1; /* Signal that Cholesky factorization is successfully performed */
}

void AlfCovariance::gnsTransposeBacksubstitution( TE U, alf_float_t* rhs, alf_float_t* x, int order ) const
{
  /* Backsubstitution starts */
  x[0] = rhs[0] / U[0][0];               /* First row of U'                   */
  for( int i = 1; i < order; i++ )
  {         /* For the rows 1..order-1           */

    alf_float_t sum = 0; //Holds backsubstitution from already handled rows

    for( int j = 0; j < i; j++ ) /* Backsubst already solved unknowns */
    {
      sum += x[j] * U[j][i];
    }

    x[i] = ( rhs[i] - sum ) / U[i][i];       /* i'th component of solution vect.  */
  }
}

void AlfCovariance::gnsBacksubstitution( TE R, alf_float_t* z, int size, alf_float_t* A ) const
{
  size--;
  A[size] = z[size] / R[size][size];

  for( int i = size - 1; i >= 0; i-- )
  {
    alf_float_t sum = 0;

    for( int j = i + 1; j <= size; j++ )
    {
      sum += R[i][j] * A[j];
    }

    A[i] = ( z[i] - sum ) / R[i][i];
  }
}

int AlfCovariance::gnsSolveByChol( const int *clip, alf_float_t*x, int numEq ) const
{
  TE LHS;
  Ty rhs;

  setEyFromClip( clip, LHS, rhs, numEq );
  return gnsSolveByChol( LHS, rhs, x, numEq );
}

int AlfCovariance::gnsSolveByChol( TE LHS, alf_float_t* rhs, alf_float_t*x, int numEq ) const
{
  Ty aux;     /* Auxiliary vector */
  TE U;    /* Upper triangular Cholesky factor of LHS */

  int res = 1;  // Signal that Cholesky factorization is successfully performed

                /* The equation to be solved is LHSx = rhs */

                /* Compute upper triangular U such that U'*U = LHS */
  if( gnsCholeskyDec( LHS, U, numEq ) ) /* If Cholesky decomposition has been successful */
  {
    /* Now, the equation is  U'*U*x = rhs, where U is upper triangular
    * Solve U'*aux = rhs for aux
    */
    gnsTransposeBacksubstitution( U, rhs, aux, numEq );

    /* The equation is now U*x = aux, solve it for x (new motion coefficients) */
    gnsBacksubstitution( U, aux, numEq, x );

  }
  else /* LHS was singular */
  {
    res = 0;

    /* Regularize LHS */
    for( int i = 0; i < numEq; i++ )
    {
      LHS[i][i] += REG;
    }

    /* Compute upper triangular U such that U'*U = regularized LHS */
    res = gnsCholeskyDec( LHS, U, numEq );

    if( !res )
    {
      std::memset( x, 0, sizeof( alf_float_t )*numEq );
      return 0;
    }

    /* Solve  U'*aux = rhs for aux */
    gnsTransposeBacksubstitution( U, rhs, aux, numEq );

    /* Solve U*x = aux for x */
    gnsBacksubstitution( U, aux, numEq, x );
  }
  return res;
}
//////////////////////////////////////////////////////////////////////////////////////////

EncAdaptiveLoopFilter::EncAdaptiveLoopFilter()
  : m_encCfg         ( nullptr )
  , m_apsMap         ( nullptr )
  , m_CABACEstimator ( nullptr )
  , m_CtxCache       ( nullptr )
  , m_apsIdStart     ( ALF_CTB_MAX_NUM_APS )
  , m_bestFilterCount( 0 )
  , m_threadpool     ( nullptr )
 {
  for( int i = 0; i < MAX_NUM_COMP; i++ )
  {
    m_alfCovariance[i] = nullptr;
  }
  for( int i = 0; i < MAX_NUM_CH; i++ )
  {
    m_alfCovarianceFrame[i] = nullptr;
  }
  m_filterCoeffSet = nullptr;
  m_filterClippSet = nullptr;
  m_diffFilterCoeff = nullptr;

  m_alfWSSD = 0;

  m_alfCovarianceCcAlf[0] = nullptr;
  m_alfCovarianceCcAlf[1] = nullptr;
  m_alfCovarianceFrameCcAlf[0] = nullptr;
  m_alfCovarianceFrameCcAlf[1] = nullptr;
}

void EncAdaptiveLoopFilter::init( const VVEncCfg& encCfg, CABACWriter& cabacEstimator, CtxCache& ctxCache, NoMallocThreadPool* threadpool )
{

  AdaptiveLoopFilter::create( encCfg.m_PadSourceWidth, encCfg.m_PadSourceHeight, encCfg.m_internChromaFormat, encCfg.m_CTUSize, encCfg.m_CTUSize, encCfg.m_MaxCodingDepth, encCfg.m_internalBitDepth );

  m_encCfg = &encCfg;
  m_CABACEstimator = &cabacEstimator;
  m_CtxCache = &ctxCache;

#if ENABLE_ALF_MAX_APU_SIZE
  m_maxApuSize    = MAX_CU_SIZE;
  m_maxApuWidth   = m_maxApuSize;
  m_maxApuHeight  = m_maxApuSize;
#else
  m_maxApuSize    = encCfg.m_CTUSize;
  m_maxApuWidth   = m_maxCUWidth;
  m_maxApuHeight  = m_maxCUHeight;
#endif
  m_numApusInWidth  = ( m_picWidth / m_maxApuWidth ) + ( ( m_picWidth % m_maxApuWidth ) ? 1 : 0 );
  m_numApusInHeight = ( m_picHeight / m_maxApuHeight ) + ( ( m_picHeight % m_maxApuHeight ) ? 1 : 0 );
  m_numApusInPic    = m_numApusInHeight * m_numApusInWidth;
  m_numCtusInApuWidth  = m_maxApuWidth  / m_maxCUWidth;
  m_numCtusInApuHeight = m_maxApuHeight / m_maxCUHeight;

  const int numBins = m_encCfg->m_useNonLinearAlfLuma || m_encCfg->m_useNonLinearAlfChroma ? MaxAlfNumClippingValues : 1;

  for( int channelIdx = 0; channelIdx < MAX_NUM_CH; channelIdx++ )
  {
    ChannelType chType = (ChannelType)channelIdx;
    int numClasses = channelIdx ? VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA : MAX_NUM_ALF_CLASSES;
    m_alfCovarianceFrame[chType] = new AlfCovariance*[m_filterShapes[chType].size()];
    for( int i = 0; i != m_filterShapes[chType].size(); i++ )
    {
      m_alfCovarianceFrame[chType][i] = new AlfCovariance[numClasses];
      for( int k = 0; k < numClasses; k++ )
      {
        m_alfCovarianceFrame[chType][i][k].create( m_filterShapes[chType][i].numCoeff, numBins );
      }
    }
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++ )
  {
    m_ctuEnableFlagTmp[compIdx] = new uint8_t[m_numCTUsInPic];
    m_ctuEnableFlagTmp2[compIdx] = new uint8_t[m_numCTUsInPic];
    if( isLuma( ComponentID(compIdx) ) )
    {
      m_ctuAlternativeTmp[compIdx] = nullptr;
    }
    else
    {
      m_ctuAlternativeTmp[compIdx] = new uint8_t[m_numCTUsInPic];
      std::fill_n( m_ctuAlternativeTmp[compIdx], m_numCTUsInPic, 0 );
    }
    ChannelType chType = toChannelType( ComponentID( compIdx ) );
    int numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;

    m_alfCovariance[compIdx] = new AlfCovariance**[m_filterShapes[chType].size()];

    for( int i = 0; i != m_filterShapes[chType].size(); i++ )
    {
      m_alfCovariance[compIdx][i] = new AlfCovariance*[m_numApusInPic];
      for( int j = 0; j < m_numApusInPic; j++ )
      {
        m_alfCovariance[compIdx][i][j] = new AlfCovariance[numClasses];
        for( int k = 0; k < numClasses; k++ )
        {
          m_alfCovariance[compIdx][i][j][k].create( m_filterShapes[chType][i].numCoeff, numBins );
        }
      }
    }
  }

  for( int i = 0; i != m_filterShapes[COMP_Y].size(); i++ )
  {
    for (int j = 0; j <= MAX_NUM_ALF_CLASSES + 1; j++)
    {
      m_alfCovarianceMerged[i][j].create( m_filterShapes[COMP_Y][i].numCoeff, numBins );
    }
  }

  m_filterCoeffSet = new int*[std::max(MAX_NUM_ALF_CLASSES, VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA)];
  m_filterClippSet = new int*[std::max(MAX_NUM_ALF_CLASSES, VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA)];
  m_diffFilterCoeff = new int*[MAX_NUM_ALF_CLASSES];

  for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
  {
    m_filterCoeffSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
    m_filterClippSet[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
    m_diffFilterCoeff[i] = new int[MAX_NUM_ALF_LUMA_COEFF];
  }

//  m_apsIdStart = ALF_CTB_MAX_NUM_APS;
  m_ctbDistortionFixedFilter = new double[m_numCTUsInPic];
  for (int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
    m_ctbDistortionUnfilter[comp] = new double[m_numCTUsInPic];
  }
  m_alfCtbFilterSetIndexTmp.resize(m_numCTUsInPic);
  memset(m_clipDefaultEnc, 0, sizeof(m_clipDefaultEnc));
  m_apsIdCcAlfStart[0] = (int) MAX_NUM_APS;
  m_apsIdCcAlfStart[1] = (int) MAX_NUM_APS;
  for( int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++ )
  {
    int numFilters = MAX_NUM_CC_ALF_FILTERS;
    m_alfCovarianceCcAlf[compIdx-1] = new AlfCovariance**[m_filterShapesCcAlf[compIdx-1].size()];
    m_alfCovarianceFrameCcAlf[compIdx-1] = new AlfCovariance*[m_filterShapesCcAlf[compIdx-1].size()];
    for( int i = 0; i != m_filterShapesCcAlf[compIdx-1].size(); i++ )
    {
      m_alfCovarianceFrameCcAlf[compIdx - 1][i] = new AlfCovariance[numFilters];
      for (int k = 0; k < numFilters; k++)
      {
        m_alfCovarianceFrameCcAlf[compIdx - 1][i][k].create(m_filterShapesCcAlf[compIdx - 1][i].numCoeff, numBins);
      }

      m_alfCovarianceCcAlf[compIdx - 1][i] = new AlfCovariance *[numFilters];
      for (int j = 0; j < numFilters; j++)
      {
        m_alfCovarianceCcAlf[compIdx - 1][i][j] = new AlfCovariance[m_numCTUsInPic];
        for (int k = 0; k < m_numCTUsInPic; k++)
        {
          m_alfCovarianceCcAlf[compIdx - 1][i][j][k].create(m_filterShapesCcAlf[compIdx - 1][i].numCoeff, numBins);
        }
      }
    }
  }
  m_trainingCovControl   = new uint8_t[m_numCTUsInPic];
  for ( int i = 0; i < MAX_NUM_CC_ALF_FILTERS; i++ )
  {
    m_trainingDistortion[i] = new uint64_t[m_numCTUsInPic];
  }
  m_filterControl         = new uint8_t[m_numCTUsInPic];
  m_bestFilterControl     = new uint8_t[m_numCTUsInPic];
  uint32_t area           = (encCfg.m_PadSourceWidth >> getComponentScaleX(COMP_Cb,encCfg.m_internChromaFormat))*(encCfg.m_PadSourceHeight >> getComponentScaleY(COMP_Cb,encCfg.m_internChromaFormat));
  m_bufOrigin             = ( Pel* ) xMalloc( Pel, area );
  m_buf                   = new PelBuf( m_bufOrigin, encCfg.m_PadSourceWidth >> getComponentScaleX(COMP_Cb,encCfg.m_internChromaFormat), encCfg.m_PadSourceWidth >> getComponentScaleX(COMP_Cb,encCfg.m_internChromaFormat), encCfg.m_PadSourceHeight >> getComponentScaleY(COMP_Cb,encCfg.m_internChromaFormat) );
  m_lumaSwingGreaterThanThresholdCount = new uint64_t[m_numCTUsInPic];
  m_chromaSampleCountNearMidPoint = new uint64_t[m_numCTUsInPic];
  m_threadpool = threadpool;
#if ALF_CTU_PAR_TRACING
    m_traceStreams = new std::stringstream[m_numCTUsInPic];
#endif
}

void EncAdaptiveLoopFilter::destroy()
{
  if (!m_created)
  {
    return;
  }
  for( int channelIdx = 0; channelIdx < MAX_NUM_CH; channelIdx++ )
  {
    if( m_alfCovarianceFrame[channelIdx] )
    {
      ChannelType chType = (ChannelType)channelIdx;
      int numClasses = channelIdx ? VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA : MAX_NUM_ALF_CLASSES;
      for( int i = 0; i != m_filterShapes[chType].size(); i++ )
      {
        for( int k = 0; k < numClasses; k++ )
        {
          m_alfCovarianceFrame[channelIdx][i][k].destroy();
        }
        delete[] m_alfCovarianceFrame[channelIdx][i];
        m_alfCovarianceFrame[channelIdx][i] = nullptr;
      }
      delete[] m_alfCovarianceFrame[channelIdx];
      m_alfCovarianceFrame[channelIdx] = nullptr;
    }
  }

  for( int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++ )
  {
    if( m_ctuEnableFlagTmp[compIdx] )
    {
      delete[] m_ctuEnableFlagTmp[compIdx];
      m_ctuEnableFlagTmp[compIdx] = nullptr;
    }

    if( m_ctuEnableFlagTmp2[compIdx] )
    {
      delete[] m_ctuEnableFlagTmp2[compIdx];
      m_ctuEnableFlagTmp2[compIdx] = nullptr;
    }

    if( m_ctuAlternativeTmp[compIdx] )
    {
      delete[] m_ctuAlternativeTmp[compIdx];
      m_ctuAlternativeTmp[compIdx] = nullptr;
    }

    if( m_alfCovariance[compIdx] )
    {
      ChannelType chType = toChannelType( ComponentID( compIdx ) );
      int numClasses = compIdx ? 1 : MAX_NUM_ALF_CLASSES;

      for( int i = 0; i != m_filterShapes[chType].size(); i++ )
      {
        for( int j = 0; j < m_numApusInPic; j++ )
        {
          for( int k = 0; k < numClasses; k++ )
          {
            m_alfCovariance[compIdx][i][j][k].destroy();
          }
          delete[] m_alfCovariance[compIdx][i][j];
          m_alfCovariance[compIdx][i][j] = nullptr;

        }
        delete[] m_alfCovariance[compIdx][i];
        m_alfCovariance[compIdx][i] = nullptr;

      }
      delete[] m_alfCovariance[compIdx];
      m_alfCovariance[compIdx] = nullptr;
    }
  }

  for( int i = 0; i != m_filterShapes[COMP_Y].size(); i++ )
  {
    for (int j = 0; j <= MAX_NUM_ALF_CLASSES + 1; j++)
    {
      m_alfCovarianceMerged[i][j].destroy();
    }
  }

  if( m_filterCoeffSet )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      delete[] m_filterCoeffSet[i];
      m_filterCoeffSet[i] = nullptr;
    }
    delete[] m_filterCoeffSet;
    m_filterCoeffSet = nullptr;
  }

  if( m_filterClippSet )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      delete[] m_filterClippSet[i];
      m_filterClippSet[i] = nullptr;
    }
    delete[] m_filterClippSet;
    m_filterClippSet = nullptr;
  }

  if( m_diffFilterCoeff )
  {
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      delete[] m_diffFilterCoeff[i];
      m_diffFilterCoeff[i] = nullptr;
    }
    delete[] m_diffFilterCoeff;
    m_diffFilterCoeff = nullptr;
  }


  delete[] m_ctbDistortionFixedFilter;
  m_ctbDistortionFixedFilter = nullptr;
  for (int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
    delete[] m_ctbDistortionUnfilter[comp];
    m_ctbDistortionUnfilter[comp] = nullptr;
  }
  for (int compIdx = 1; compIdx < MAX_NUM_COMP; compIdx++)
  {
    int numFilters = MAX_NUM_CC_ALF_FILTERS;
    if (m_alfCovarianceFrameCcAlf[compIdx - 1])
    {
      for (int i = 0; i != m_filterShapesCcAlf[compIdx - 1].size(); i++)
      {
        for (int k = 0; k < numFilters; k++)
        {
          m_alfCovarianceFrameCcAlf[compIdx - 1][i][k].destroy();
        }
        delete[] m_alfCovarianceFrameCcAlf[compIdx - 1][i];
      }
      delete[] m_alfCovarianceFrameCcAlf[compIdx - 1];
      m_alfCovarianceFrameCcAlf[compIdx - 1] = nullptr;
    }

    if (m_alfCovarianceCcAlf[compIdx - 1])
    {
      for (int i = 0; i != m_filterShapesCcAlf[compIdx - 1].size(); i++)
      {
        for (int j = 0; j < numFilters; j++)
        {
          for (int k = 0; k < m_numCTUsInPic; k++)
          {
            m_alfCovarianceCcAlf[compIdx - 1][i][j][k].destroy();
          }
          delete[] m_alfCovarianceCcAlf[compIdx - 1][i][j];
        }
        delete[] m_alfCovarianceCcAlf[compIdx - 1][i];
      }
      delete[] m_alfCovarianceCcAlf[compIdx - 1];
      m_alfCovarianceCcAlf[compIdx - 1] = nullptr;
    }
  }

  if (m_trainingCovControl)
  {
    delete[] m_trainingCovControl;
    m_trainingCovControl = nullptr;
  }

  for ( int i = 0; i < MAX_NUM_CC_ALF_FILTERS; i++ )
  {
    if (m_trainingDistortion[i])
    {
      delete[] m_trainingDistortion[i];
      m_trainingDistortion[i] = nullptr;
    }
  }

  if (m_filterControl)
  {
    delete[] m_filterControl;
    m_filterControl = nullptr;
  }

  if (m_bestFilterControl)
  {
    delete[] m_bestFilterControl;
    m_bestFilterControl = nullptr;
  }

  if (m_bufOrigin)
  {
    xFree(m_bufOrigin);
    m_bufOrigin = nullptr;
  }

  if (m_buf)
  {
    delete m_buf;
    m_buf = nullptr;
  }

  if (m_lumaSwingGreaterThanThresholdCount)
  {
    delete[] m_lumaSwingGreaterThanThresholdCount;
    m_lumaSwingGreaterThanThresholdCount = nullptr;
  }
  if (m_chromaSampleCountNearMidPoint)
  {
    delete[] m_chromaSampleCountNearMidPoint;
    m_chromaSampleCountNearMidPoint = nullptr;
  }
#if ALF_CTU_PAR_TRACING
  delete[] m_traceStreams;
#endif
  AdaptiveLoopFilter::destroy();
}


void EncAdaptiveLoopFilter::initCABACEstimator( Slice* pcSlice, ParameterSetMap<APS>* apsMap )
{
  m_apsMap         = apsMap;
  m_CABACEstimator->initCtxModels( *pcSlice );
  m_CABACEstimator->resetBits();
}

void EncAdaptiveLoopFilter::xSetupCcAlfAPS( CodingStructure &cs )
{
  if (m_ccAlfFilterParam.ccAlfFilterEnabled[COMP_Cb - 1])
  {
    int  ccAlfCbApsId = cs.slice->ccAlfCbApsId;
    APS* aps = m_apsMap->getPS((cs.slice->ccAlfCbApsId << NUM_APS_TYPE_LEN) + ALF_APS);
    if (aps == NULL)
    {
      aps = m_apsMap->allocatePS((ccAlfCbApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      aps->temporalId = cs.slice->TLayer;
    }
    aps->ccAlfParam.ccAlfFilterEnabled[COMP_Cb - 1] = 1;
    aps->ccAlfParam.ccAlfFilterCount[COMP_Cb - 1] = m_ccAlfFilterParam.ccAlfFilterCount[COMP_Cb - 1];
    for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
    {
      aps->ccAlfParam.ccAlfFilterIdxEnabled[COMP_Cb - 1][filterIdx] =
        m_ccAlfFilterParam.ccAlfFilterIdxEnabled[COMP_Cb - 1][filterIdx];
      memcpy(aps->ccAlfParam.ccAlfCoeff[COMP_Cb - 1][filterIdx],
             m_ccAlfFilterParam.ccAlfCoeff[COMP_Cb - 1][filterIdx], sizeof(short) * MAX_NUM_CC_ALF_CHROMA_COEFF);
    }
    aps->apsId   = ccAlfCbApsId;
    aps->apsType = ALF_APS;
    aps->poc     = cs.slice->poc;
    if (m_reuseApsId[COMP_Cb - 1] < 0)
    {
      aps->ccAlfParam.newCcAlfFilter[COMP_Cb - 1] = 1;
      m_apsMap->setChangedFlag((ccAlfCbApsId << NUM_APS_TYPE_LEN) + ALF_APS, true);
      aps->temporalId = cs.slice->TLayer;
    }
    cs.slice->ccAlfCbEnabled = true;
  }
  else
  {
    cs.slice->ccAlfCbEnabled = false;
  }
  if (m_ccAlfFilterParam.ccAlfFilterEnabled[COMP_Cr - 1])
  {
    int  ccAlfCrApsId = cs.slice->ccAlfCrApsId;
    APS* aps = m_apsMap->getPS((cs.slice->ccAlfCrApsId << NUM_APS_TYPE_LEN) + ALF_APS);
    if (aps == NULL)
    {
      aps = m_apsMap->allocatePS((ccAlfCrApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      aps->temporalId = cs.slice->TLayer;
    }
    aps->ccAlfParam.ccAlfFilterEnabled[COMP_Cr - 1] = 1;
    aps->ccAlfParam.ccAlfFilterCount[COMP_Cr - 1] = m_ccAlfFilterParam.ccAlfFilterCount[COMP_Cr - 1];
    for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
    {
      aps->ccAlfParam.ccAlfFilterIdxEnabled[COMP_Cr - 1][filterIdx] =
        m_ccAlfFilterParam.ccAlfFilterIdxEnabled[COMP_Cr - 1][filterIdx];
      memcpy(aps->ccAlfParam.ccAlfCoeff[COMP_Cr - 1][filterIdx],
             m_ccAlfFilterParam.ccAlfCoeff[COMP_Cr - 1][filterIdx], sizeof(short) * MAX_NUM_CC_ALF_CHROMA_COEFF);
    }
    aps->apsId =  ccAlfCrApsId;
    aps->poc   = cs.slice->poc;
    if (m_reuseApsId[COMP_Cr - 1] < 0)
    {
      aps->ccAlfParam.newCcAlfFilter[COMP_Cr - 1] = 1;
      m_apsMap->setChangedFlag((ccAlfCrApsId << NUM_APS_TYPE_LEN) + ALF_APS, true);
      aps->temporalId = cs.slice->TLayer;
    }
    aps->apsType = ALF_APS;
    cs.slice->ccAlfCrEnabled = true;
  }
  else
  {
    cs.slice->ccAlfCrEnabled = false;
  }
}

void EncAdaptiveLoopFilter::getStatisticsCTU( Picture& pic, CodingStructure& cs, PelUnitBuf& recYuv, const int ctuRsAddr )
{
  PROFILER_SCOPE_AND_STAGE( 0, _TPROF, P_ALF_STATS );

  if( cs.sps->maxTLayers > 1 && cs.sps->maxTLayers - m_encCfg->m_alfSpeed <= pic.TLayer )
  {
    return;
  }

  PelUnitBuf orgYuv = cs.picture->getOrigBuf();
  const int numClassBlocksInCTU = ( MAX_CU_SIZE * MAX_CU_SIZE ) >> 4;

  const CPelBuf& recLuma = recYuv.get( COMP_Y );
  const PreCalcValues& pcv = *cs.pcv;
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };

//  const int numberOfComponents = getNumberValidComponents( m_chromaFormat );

  const int xPos = ( ctuRsAddr % pcv.widthInCtus ) << pcv.maxCUSizeLog2;
  const int yPos = ( ctuRsAddr / pcv.widthInCtus ) << pcv.maxCUSizeLog2;

  const int width = ( xPos + pcv.maxCUSize > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUSize;
  const int height = ( yPos + pcv.maxCUSize > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUSize;

  // init APU stats buffer
  const int apuRsAddr = ( yPos / m_maxApuHeight ) * m_numApusInWidth + xPos / m_maxApuWidth;
  const int numberOfComponents = getNumberValidComponents( m_chromaFormat );
  if( xPos % m_maxApuWidth == 0 && yPos % m_maxApuHeight == 0 )
  {
    for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
    {
      const ComponentID compID = ComponentID( compIdx );
      const int numClasses = isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1;

      for( int shape = 0; shape != m_filterShapes[toChannelType( compID )].size(); shape++ )
      {
        for( int classIdx = 0; classIdx < numClasses; classIdx++ )
        {
          m_alfCovariance[compIdx][shape][apuRsAddr][classIdx].reset();
        }
      }
    }
  }
  int rasterSliceAlfPad = 0;
  if( isCrossedByVirtualBoundaries( cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, rasterSliceAlfPad ) )
  {
    int yStart = yPos;
    for( int i = 0; i <= numHorVirBndry; i++ )
    {
      const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
      const int h = yEnd - yStart;
      const bool clipT = ( i == 0 && clipTop ) || ( i > 0 ) || ( yStart == 0 );
      const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );

      int xStart = xPos;
      for( int j = 0; j <= numVerVirBndry; j++ )
      {
        // Classification
        const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
        const int w = xEnd - xStart;
        const bool clipL = ( j == 0 && clipLeft ) || ( j > 0 ) || ( xStart == 0 );
        const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );

        const int wBuf = w + ( clipL ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipR ? 0 : MAX_ALF_PADDING_SIZE );
        const int hBuf = h + ( clipT ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipB ? 0 : MAX_ALF_PADDING_SIZE );
        PelUnitBuf  dstBuf = m_tempBuf2.subBuf( UnitArea( cs.area.chromaFormat, Area( 0, 0, wBuf, hBuf ) ) );
        CPelUnitBuf srcBuf = recYuv.subBuf( UnitArea( cs.area.chromaFormat, Area( xStart - ( clipL ? 0 : MAX_ALF_PADDING_SIZE ), yStart - ( clipT ? 0 : MAX_ALF_PADDING_SIZE ), wBuf, hBuf ) ) );
        dstBuf.copyFrom( srcBuf );
        // pad top-left unavailable samples for raster slice
        if( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
        {
          dstBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 1 );
        }

        // pad bottom-right unavailable samples for raster slice
        if( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
        {
          dstBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 2 );
        }
        dstBuf.extendBorderPel( MAX_ALF_PADDING_SIZE );
        PelUnitBuf buf = dstBuf.subBuf( UnitArea ( cs.area.chromaFormat, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );

        const Area blkSrc( 0, 0, w, h );
        const Area blkDst( xStart, yStart, w, h );
        deriveClassification( &m_classifier[numClassBlocksInCTU * ctuRsAddr], buf.get( COMP_Y ), blkDst, blkSrc );

        // CTU statistic
        const UnitArea area( m_chromaFormat, Area( 0, 0, w, h ) );
        const UnitArea areaDst( m_chromaFormat, Area( xStart, yStart, w, h ) );
        for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
        {
          const ComponentID compID = ComponentID( compIdx );
          const ChannelType chType = toChannelType( compID );
          const CompArea& compArea = area.block( compID );

          int  recStride = buf.get( compID ).stride;
          Pel* rec = buf.get( compID ).bufAt( compArea );

          int  orgStride = orgYuv.get( compID ).stride;
          Pel* org = orgYuv.get( compID ).bufAt( xStart >> getComponentScaleX( compID, m_chromaFormat ), yStart >> getComponentScaleY( compID, m_chromaFormat ) );


          for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
          {
            const CompArea& compAreaDst = areaDst.block( compID );
            getPreBlkStats( m_alfCovariance[compIdx][shape][apuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : &m_classifier[numClassBlocksInCTU * ctuRsAddr], org, orgStride, rec, recStride, compAreaDst, compArea, chType
              , ( ( compIdx == 0 ) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight )
              , ( compIdx == 0 ) ? m_alfVBLumaPos : m_alfVBChmaPos
            );
          }
        }

        xStart = xEnd;
      }

      yStart = yEnd;
    }
  }
  else
  {
    Area blk( xPos, yPos, width, height );
    deriveClassification( &m_classifier[numClassBlocksInCTU * ctuRsAddr], recLuma, blk, blk );

    // CTU statistic
    const UnitArea area( m_chromaFormat, Area( xPos, yPos, width, height ) );
    for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
    {
      const ComponentID compID = ComponentID( compIdx );
      const CompArea& compArea = area.block( compID );

      int  recStride = recYuv.get( compID ).stride;
      Pel* rec = recYuv.get( compID ).bufAt( compArea );

      int  orgStride = orgYuv.get( compID ).stride;
      Pel* org = orgYuv.get( compID ).bufAt( compArea );

      ChannelType chType = toChannelType( compID );

      for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
      {
        getPreBlkStats( m_alfCovariance[compIdx][shape][apuRsAddr], m_filterShapes[chType][shape], compIdx ? nullptr : &m_classifier[numClassBlocksInCTU * ctuRsAddr], org, orgStride, rec, recStride, compArea, compArea, chType
          , ( ( compIdx == 0 ) ? m_alfVBLumaCTUHeight : m_alfVBChmaCTUHeight )
          , ( compIdx == 0 ) ? m_alfVBLumaPos : m_alfVBChmaPos
        );
      }
    }
  }
}

void EncAdaptiveLoopFilter::copyCTUForCCALF( CodingStructure& cs, int ctuPosX, int ctuPosY )
{
  UnitArea   ctuArea   = UnitArea( m_chromaFormat, cs.pcv->getCtuArea( ctuPosX, ctuPosY ) );
  PelUnitBuf ctuBufSrc = cs.getRecoBuf( ctuArea );
  PelUnitBuf ctuBufDst  = m_tempBuf.getBuf( ctuArea );
  ctuBufDst.get( COMP_Cb ).copyFrom( ctuBufSrc.get( COMP_Cb ) );
  ctuBufDst.get( COMP_Cr ).copyFrom( ctuBufSrc.get( COMP_Cr ) );
}

void EncAdaptiveLoopFilter::getStatisticsFrame( Picture& pic, CodingStructure& cs )
{
  PelUnitBuf recoBuf = cs.picture->getRecoBuf();
  resetFrameStats( cs.slice->sps->ccalfEnabled );
  for( int ctuRsAddr = 0; ctuRsAddr < m_numCTUsInPic; ctuRsAddr++ )
  {
    getStatisticsCTU( pic, cs, recoBuf, ctuRsAddr );
  }
}

void EncAdaptiveLoopFilter::deriveFilter( Picture& pic, CodingStructure& cs, const double* lambdas )
{
  PROFILER_SCOPE_AND_STAGE( 0, _TPROF, P_ALF_STATS );

  if( cs.sps->maxTLayers > 1 && cs.sps->maxTLayers - m_encCfg->m_alfSpeed <= pic.TLayer )
  {
    memset( cs.slice->alfEnabled, 0, sizeof( cs.slice->alfEnabled ) );
    return;
  }

  initCABACEstimator( cs.slice, &pic.picApsMap );
  if( m_encCfg->m_maxParallelFrames )
  {
    setApsIdStart( pic.picApsMap.getApsIdStart() );
  }

  // On TL0 and pending RAS: reset APS
  int layerIdx = cs.vps == nullptr ? 0 : cs.vps->generalLayerIdx[ cs.slice->pic->layerId ];
  if ( !layerIdx && ( cs.slice->pendingRasInit || cs.slice->isIDRorBLA() || ( cs.slice->nalUnitType == VVENC_NAL_UNIT_CODED_SLICE_CRA && m_encCfg->m_craAPSreset ) ) )
  {
    memset(cs.slice->alfAps, 0, sizeof(*cs.slice->alfAps)*ALF_CTB_MAX_NUM_APS);
    m_apsIdStart = ALF_CTB_MAX_NUM_APS;
    m_apsMap->clearActive();
    for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
    {
      APS* alfAPS = m_apsMap->getPS((i << NUM_APS_TYPE_LEN) + ALF_APS);
      m_apsMap->clearChangedFlag((i << NUM_APS_TYPE_LEN) + ALF_APS);
      if (alfAPS)
      {
        alfAPS->alfParam.reset();
        alfAPS->ccAlfParam.reset();
        alfAPS = nullptr;
      }
    }
  }

  // Accumulate ALF statistic
  const int numberOfComponents = getNumberValidComponents( m_chromaFormat );
  for( int compIdx = 0; compIdx < numberOfComponents; compIdx++ )
  {
    const ComponentID compID = ComponentID( compIdx );
    const ChannelType chType = toChannelType( compID );
    for( int shape = 0; shape != m_filterShapes[chType].size(); shape++ )
    {
      for( int apuRsAddr = 0; apuRsAddr < m_numApusInPic; apuRsAddr++ )
      {
        for( int classIdx = 0; classIdx < ( isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1 ); classIdx++ )
        {
          m_alfCovarianceFrame[chType][shape][isLuma( compID ) ? classIdx : 0] += m_alfCovariance[compIdx][shape][apuRsAddr][classIdx];
#if ENABLE_TRACING
          m_alfCovarianceFrame[chType][shape][isLuma( compID ) ? classIdx : 0].trace();
#endif
        }
      }
    }
  }


  AlfParam alfParam;
  alfParam.reset();
  const TempCtx ctxStart( m_CtxCache, AlfCtx( m_CABACEstimator->getCtx() ) );

  // set available filter shapes
  alfParam.filterShapes = m_filterShapes;

  for( int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++ )
  {
    pic.m_alfCtbFilterIndex[ctbIdx] = NUM_FIXED_FILTER_SETS;
  }

  // set CTU ALF enable flags, it was already reset before ALF process
  for( int compIdx = 0; compIdx < MAX_NUM_COMP; compIdx++ )
  {
    m_ctuEnableFlag[compIdx] = cs.picture->m_alfCtuEnabled[ compIdx ].data();
    m_ctuAlternative[compIdx] = cs.picture->m_alfCtuAlternative[ compIdx ].data();
  }

  // reset ALF parameters
  alfParam.reset();
  int shiftLuma   = 2 * DISTORTION_PRECISION_ADJUSTMENT( m_inputBitDepth[CH_L] );
  int shiftChroma = 2 * DISTORTION_PRECISION_ADJUSTMENT( m_inputBitDepth[CH_C] );
  m_lambda[COMP_Y]  = lambdas[COMP_Y]  * double( 1 << shiftLuma );
  m_lambda[COMP_Cb] = lambdas[COMP_Cb] * double( 1 << shiftChroma );
  m_lambda[COMP_Cr] = lambdas[COMP_Cr] * double( 1 << shiftChroma );

  // consider using new filter (only)
  alfParam.newFilterFlag[CH_L] = true;
  alfParam.newFilterFlag[CH_C] = true;
  cs.slice->numAps = ( 1 ); // Only new filter for RD cost optimization

  const bool useCtuWiseLambda     = m_encCfg->m_usePerceptQPA && cs.slice->pps->useDQP;
  const double lambdaChromaWeight = useCtuWiseLambda && ( m_lambda[COMP_Y] > 0.0 ) ? ( m_lambda[COMP_Cb] + m_lambda[COMP_Cr] ) / ( 2.0 * m_lambda[COMP_Y] ) : 0.0;

  // derive filter (luma)
  alfEncoder( cs, alfParam, CH_L, lambdaChromaWeight );
  // derive filter (chroma)
  alfEncoder( cs, alfParam, CH_C, lambdaChromaWeight );

  // let alfEncoderCtb decide now
  alfParam.newFilterFlag[CH_L] = false;
  alfParam.newFilterFlag[CH_C] = false;
  cs.slice->numAps = ( 0 );
  m_CABACEstimator->getCtx() = AlfCtx( ctxStart );
  alfEncoderCtb( cs, alfParam, lambdaChromaWeight );
}

void EncAdaptiveLoopFilter::reconstructCTU_MT( Picture& pic, CodingStructure& cs, int ctuRsAddr )
{
  PROFILER_SCOPE_AND_STAGE( 0, _TPROF, P_ALF_REC );

  if( cs.sps->maxTLayers > 1 && cs.sps->maxTLayers - m_encCfg->m_alfSpeed <= pic.TLayer )
  {
    return;
  }

  const int nCtuX = ctuRsAddr % cs.pcv->widthInCtus;
  const int nCtuY = ctuRsAddr / cs.pcv->widthInCtus;

  // copy unfiltered reco (including padded / extented area)
  const ChromaFormat chromaFormat = cs.area.chromaFormat;
  UnitArea ctuArea                = UnitArea( chromaFormat, cs.pcv->getCtuArea( nCtuX, nCtuY ) );
  const int extMargin             = ( MAX_ALF_FILTER_LENGTH + 1 ) >> 1;
  for( int i = 0; i < getNumberValidComponents( chromaFormat ); i++ )
  {
    const int extX   = extMargin >> getComponentScaleX( ComponentID( i ), chromaFormat );
    const int extY   = extMargin >> getComponentScaleY( ComponentID( i ), chromaFormat );
    CompArea cpyArea = ctuArea.blocks[ i ];
    cpyArea.x += extX;
    cpyArea.y += extY;
    if ( nCtuX == 0 )
    {
      cpyArea.x     -= 2 * extX;
      cpyArea.width += 2 * extX;
    }
    if ( nCtuY == 0 )
    {
      cpyArea.y      -= 2 * extY;
      cpyArea.height += 2 * extY;
    }
    PelBuf dstBuf = m_tempBuf.getBuf( cpyArea );
    PelBuf srcBuf = pic.getRecoBuf( cpyArea );
    dstBuf.copyFrom( srcBuf );
  }

  // Perform filtering
  PelUnitBuf ctuBuf = m_tempBuf.getBuf( ctuArea );
  reconstructCTU( pic, cs, ctuBuf, ctuRsAddr );
}

#if ENABLE_TRACING
template< typename Tsrc >
void traceStreamBlock( std::stringstream& str, Tsrc *buf, unsigned stride, unsigned block_w, unsigned block_h )
{
  unsigned i, j;
  for( j = 0; j < block_h; j++ )
  {
    for( i = 0; i < block_w; i++ )
    {
      str << std::setw( 4 ) << std::hex << buf[j * stride + i];
      //trace_ctx->dtrace<false>( channel, "%4d ", buf[j*stride + i] );
    }
    str << std::endl;
  }
  str << std::endl;
}
#endif

void EncAdaptiveLoopFilter::reconstructCTU( Picture& pic, CodingStructure& cs, const CPelUnitBuf& recExtBufCTU, int ctuRsAddr )
{
  bool ctuEnableFlag = m_ctuEnableFlag[COMP_Y][ctuRsAddr];
  const int numberOfComponents = getNumberValidComponents( m_chromaFormat );
  for( int compIdx = 1; compIdx < numberOfComponents; compIdx++ )
    ctuEnableFlag |= m_ctuEnableFlag[compIdx][ctuRsAddr] > 0;

#if ALF_CTU_PAR_TRACING
  {
    const PreCalcValues& pcv = *cs.pcv;
    const int xPos            = ( ctuRsAddr % pcv.widthInCtus ) << pcv.maxCUSizeLog2;
    const int yPos            = ( ctuRsAddr / pcv.widthInCtus ) << pcv.maxCUSizeLog2;
    const int width           = ( xPos + pcv.maxCUSize > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUSize;
    const int height          = ( yPos + pcv.maxCUSize > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUSize;

    traceStreamBlock( m_traceStreams[ctuRsAddr], recExtBufCTU.bufs[0].buf - ( MAX_ALF_FILTER_LENGTH >> 1 ) * recExtBufCTU.bufs[0].stride - ( MAX_ALF_FILTER_LENGTH >> 1 ),
      recExtBufCTU.bufs[0].stride,
      width + 2 * ( MAX_ALF_FILTER_LENGTH >> 1 ),
      height + 2 * ( MAX_ALF_FILTER_LENGTH >> 1 ) );
    if( ctuRsAddr == m_numCTUsToProcess - 1 )
    {
      for( int i = 0; i < m_numCTUsToProcess; i++ )
      {
        DTRACE( g_trace_ctx, D_MISC, "CTU=%d \n", i );
        DTRACE( g_trace_ctx, D_MISC, "%s", m_traceStreams[i].str().c_str() );
      }
    }
  }
#endif

  if( ctuEnableFlag )
  {
    PelUnitBuf& recBuf = cs.getRecoBufRef();
    const ClpRngs& clpRngs = cs.slice->clpRngs;

    const short* alfCtuFilterIndex = cs.picture->m_alfCtbFilterIndex.data();

    const PreCalcValues& pcv = *cs.pcv;
    const int xPos            = ( ctuRsAddr % pcv.widthInCtus ) << pcv.maxCUSizeLog2;
    const int yPos            = ( ctuRsAddr / pcv.widthInCtus ) << pcv.maxCUSizeLog2;
    const int width           = ( xPos + pcv.maxCUSize > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUSize;
    const int height          = ( yPos + pcv.maxCUSize > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUSize;
    const int nonLinAlfLuma   = m_encCfg->m_useNonLinearAlfLuma ? 1 : 0;
    const int nonLinAlfChroma = m_encCfg->m_useNonLinearAlfChroma ? 1 : 0;

    const int numClassBlocksInCTU = ( MAX_CU_SIZE * MAX_CU_SIZE ) >> 4;

    bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
    int numHorVirBndry = 0, numVerVirBndry = 0;
    int horVirBndryPos[] = { 0, 0, 0 };
    int verVirBndryPos[] = { 0, 0, 0 };
    int rasterSliceAlfPad = 0;
    if( isCrossedByVirtualBoundaries( cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, rasterSliceAlfPad ) )
    {
      int yStart = yPos;
      for( int i = 0; i <= numHorVirBndry; i++ )
      {
        const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
        const int h = yEnd - yStart;
        const bool clipT = ( i == 0 && clipTop ) || ( i > 0 ) || ( yStart == 0 );
        const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );

        int xStart = xPos;
        for( int j = 0; j <= numVerVirBndry; j++ )
        {
          const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
          const int w = xEnd - xStart;
          const bool clipL = ( j == 0 && clipLeft ) || ( j > 0 ) || ( xStart == 0 );
          const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );

          const int wBuf = w + ( clipL ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipR ? 0 : MAX_ALF_PADDING_SIZE );
          const int hBuf = h + ( clipT ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipB ? 0 : MAX_ALF_PADDING_SIZE );
          PelUnitBuf  dstBuf = m_tempBuf2.subBuf( UnitArea( cs.area.chromaFormat, Area( 0, 0, wBuf, hBuf ) ) );
          dstBuf.copyFrom( m_tempBuf.subBuf( UnitArea( cs.area.chromaFormat, Area( xStart - (clipL ? 0 : MAX_ALF_PADDING_SIZE), yStart - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf ) ) ) );
          // pad top-left unavailable samples for raster slice
          if( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
          {
            dstBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 1 );
          }

          // pad bottom-right unavailable samples for raster slice
          if( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
          {
            dstBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 2 );
          }
          dstBuf.extendBorderPel( MAX_ALF_PADDING_SIZE );
          PelUnitBuf buf = dstBuf.subBuf( UnitArea( cs.area.chromaFormat, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );

          if( m_ctuEnableFlag[COMP_Y][ctuRsAddr] )
          {
            const Area blkSrc( 0, 0, w, h );
            const Area blkDst( xStart, yStart, w, h );
            short filterSetIndex = alfCtuFilterIndex[ctuRsAddr];
            short* coeff;
            short* clip;
            if( filterSetIndex >= NUM_FIXED_FILTER_SETS )
            {
              coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
              clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
            }
            else
            {
              coeff = m_fixedFilterSetCoeffDec[filterSetIndex];
              clip = m_clipDefault;
            }
            m_filter7x7Blk[nonLinAlfLuma]( &m_classifier[numClassBlocksInCTU * ctuRsAddr], recBuf, buf, blkDst, blkSrc, COMP_Y, coeff, clip, clpRngs.comp[COMP_Y], cs
              , m_alfVBLumaCTUHeight
              , m_alfVBLumaPos
              );
          }

          for( int compIdx = 1; compIdx < numberOfComponents; compIdx++ )
          {
            ComponentID compID = ComponentID( compIdx );
            const int chromaScaleX = getComponentScaleX( compID, recBuf.chromaFormat );
            const int chromaScaleY = getComponentScaleY( compID, recBuf.chromaFormat );
            if( m_ctuEnableFlag[compIdx][ctuRsAddr] )
            {
              const Area blkSrc( 0, 0, w >> chromaScaleX, h >> chromaScaleY );
              const Area blkDst( xStart >> chromaScaleX, yStart >> chromaScaleY, w >> chromaScaleX, h >> chromaScaleY );
              const int alt_num = m_ctuAlternative[compID][ctuRsAddr];
              m_filter5x5Blk[nonLinAlfChroma]( m_classifier, recBuf, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], clpRngs.comp[compIdx], cs
                , m_alfVBChmaCTUHeight
                , m_alfVBChmaPos
                );
            }
          }

          xStart = xEnd;
        }

        yStart = yEnd;
      }
    }
    else
    {
      const UnitArea area( cs.area.chromaFormat, Area( xPos, yPos, width, height ) );
      if( m_ctuEnableFlag[COMP_Y][ctuRsAddr] )
      {
        const Area blk( xPos, yPos, width, height );
        const Area blkSrc( 0, 0, width, height );
        short filterSetIndex = alfCtuFilterIndex[ctuRsAddr];
        short* coeff;
        short* clip;
        if( filterSetIndex >= NUM_FIXED_FILTER_SETS )
        {
          coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
          clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
        }
        else
        {
          coeff = m_fixedFilterSetCoeffDec[filterSetIndex];
          clip = m_clipDefault;
        }
        m_filter7x7Blk[nonLinAlfLuma]( &m_classifier[numClassBlocksInCTU * ctuRsAddr], recBuf, recExtBufCTU, blk, blkSrc, COMP_Y, coeff, clip, clpRngs.comp[COMP_Y], cs
          , m_alfVBLumaCTUHeight
          , m_alfVBLumaPos
          );
      }

      for( int compIdx = 1; compIdx < numberOfComponents; compIdx++ )
      {
        ComponentID compID = ComponentID( compIdx );
        const int chromaScaleX = getComponentScaleX( compID, recBuf.chromaFormat );
        const int chromaScaleY = getComponentScaleY( compID, recBuf.chromaFormat );
        if( m_ctuEnableFlag[compIdx][ctuRsAddr] )
        {
          Area blk( xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY );
          const Area blkSrc( 0, 0, width >> chromaScaleX, height >> chromaScaleY );
          const int alt_num = m_ctuAlternative[compID][ctuRsAddr];
          m_filter5x5Blk[nonLinAlfChroma]( m_classifier, recBuf, recExtBufCTU, blk, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], clpRngs.comp[compIdx], cs
            , m_alfVBChmaCTUHeight
            , m_alfVBChmaPos
            );
        }
      }
    }
  }

}

void EncAdaptiveLoopFilter::alfReconstructor( CodingStructure& cs )
{
  PROFILER_SCOPE_AND_STAGE( 0, _TPROF, P_ALF_REC );
  if( !cs.slice->alfEnabled[COMP_Y] )
  {
    return;
  }


  reconstructCoeffAPSs( cs, true, cs.slice->alfEnabled[COMP_Cb] || cs.slice->alfEnabled[COMP_Cr], false );
  const PreCalcValues& pcv = *cs.pcv;

  int ctuIdx = 0;
  for( int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUSize )
  {
    for( int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUSize )
    {
      reconstructCTU_MT( *cs.picture, cs, ctuIdx );
      ctuIdx++;
    }
  }
}

void EncAdaptiveLoopFilter::resetFrameStats( bool ccAlfEnabled )
{
  // init Frame stats buffers
  const int numberOfChannels = getNumberValidChannels( m_chromaFormat );
  for( int channelIdx = 0; channelIdx < numberOfChannels; channelIdx++ )
  {
    const ChannelType channelID = ChannelType( channelIdx );
    const int numAlts = channelID == CH_L ? 1 : VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA;
    const int numClasses = isLuma( channelID ) ? MAX_NUM_ALF_CLASSES : 1;

    for( int altIdx = 0; altIdx < numAlts; ++altIdx )
    {
      for( int shape = 0; shape != m_filterShapes[channelIdx].size(); shape++ )
      {
        for( int classIdx = 0; classIdx < numClasses; classIdx++ )
        {
          m_alfCovarianceFrame[channelIdx][shape][isLuma( channelID ) ? classIdx : altIdx].reset();
        }
      }
    }
  }

  if( ccAlfEnabled )
  {
    const int filterIdx = 0;
    const int numberOfComponents = getNumberValidComponents( m_chromaFormat );
    for( int compIdx = 1; compIdx < numberOfComponents; compIdx++ )
    {
      for( int shape = 0; shape != m_filterShapesCcAlf[compIdx - 1].size(); shape++ )
      {
        m_alfCovarianceFrameCcAlf[compIdx - 1][shape][filterIdx].reset();
      }
    }
  }
}

void EncAdaptiveLoopFilter::xStoreAlfApuEnabledFlag( CodingStructure& cs, int ctuX, int ctuY, int ctuIdx, const int compIdx, bool flag )
{
#if OPTIMIZE_ORIG_CASE
  if( m_numApusInPic == m_numCTUsInPic )
  {
    m_ctuEnableFlag[compIdx][ctuIdx] = flag;
  }
  else
#endif
  {
    int ctuMaxX = getApuMaxCtuX( ctuX );
    int ctuMaxY = getApuMaxCtuY( ctuY );

    for( int cY = ctuY; cY < ctuMaxY; cY++ )
    {
      for( int cX = ctuX; cX < ctuMaxX; cX++ )
      {
        int curCtuIdx = cY * cs.pcv->widthInCtus + cX;
        m_ctuEnableFlag[compIdx][curCtuIdx] = flag;
      }
    }
  }
}

double EncAdaptiveLoopFilter::xCodeAlfApuEnabledFlag( CodingStructure& cs, int ctuX, int ctuY, int ctuIdx, const int compIdx, AlfParam* alfParam, const double chromaWeight )
{
  double rate = 0;
#if OPTIMIZE_ORIG_CASE
  if( m_numApusInPic == m_numCTUsInPic )
  {
    m_CABACEstimator->codeAlfCtuEnabledFlag( cs, ctuIdx, compIdx, alfParam );
    const double ctuLambda = chromaWeight > 0.0 ? (isLuma (ComponentID(compIdx)) ? cs.picture->ctuQpaLambda[ctuIdx] : cs.picture->ctuQpaLambda[ctuIdx] * chromaWeight) : m_lambda[compIdx];
    rate += ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  }
  else
#endif
  {
    int ctuMaxX = getApuMaxCtuX( ctuX );
    int ctuMaxY = getApuMaxCtuY( ctuY );

    const uint8_t flag = m_ctuEnableFlag[compIdx][ctuIdx];
    for( int cY = ctuY; cY < ctuMaxY; cY++ )
    {
      for( int cX = ctuX; cX < ctuMaxX; cX++ )
      {
        int curCtuIdx = cY * cs.pcv->widthInCtus + cX;
        m_ctuEnableFlag[compIdx][curCtuIdx] = flag;
        m_CABACEstimator->codeAlfCtuEnabledFlag( cs, curCtuIdx, compIdx, alfParam );
        const double ctuLambda = chromaWeight > 0.0 ? (isLuma (ComponentID(compIdx)) ? cs.picture->ctuQpaLambda[curCtuIdx] : cs.picture->ctuQpaLambda[curCtuIdx] * chromaWeight) : m_lambda[compIdx];
        rate += ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
      }
    }
  }
  return rate;
}

double EncAdaptiveLoopFilter::xCodeAlfApuLuma( CodingStructure& cs, int ctuX, int ctuY, int ctuIdx, const int compIdx, AlfParam* alfParam, const double chromaWeight )
{
  double rate = 0;
#if OPTIMIZE_ORIG_CASE
  if( m_numApusInPic == m_numCTUsInPic )
  {
    m_CABACEstimator->codeAlfCtuEnabledFlag( cs, ctuIdx, compIdx, alfParam );
    m_CABACEstimator->codeAlfCtuFilterIndex( cs, ctuIdx, m_alfParamTemp.alfEnabled[COMP_Y] );
    const double ctuLambda = chromaWeight > 0.0 ? (isLuma (ComponentID(compIdx)) ? cs.picture->ctuQpaLambda[ctuIdx] : cs.picture->ctuQpaLambda[ctuIdx] * chromaWeight) : m_lambda[compIdx];
    rate += ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  }
  else
#endif
  {
    int ctuMaxX = getApuMaxCtuX( ctuX );
    int ctuMaxY = getApuMaxCtuY( ctuY );

    const uint8_t flag = m_ctuEnableFlag[compIdx][ctuIdx];
    const short fltIdx = cs.picture->m_alfCtbFilterIndex[ctuIdx];

    for( int cY = ctuY; cY < ctuMaxY; cY++ )
    {
      for( int cX = ctuX; cX < ctuMaxX; cX++ )
      {
        int curCtuIdx = cY * cs.pcv->widthInCtus + cX;
        m_ctuEnableFlag[compIdx][curCtuIdx] = flag;
        cs.picture->m_alfCtbFilterIndex[curCtuIdx] = fltIdx;

        m_CABACEstimator->codeAlfCtuEnabledFlag( cs, curCtuIdx, compIdx, alfParam );
        m_CABACEstimator->codeAlfCtuFilterIndex( cs, curCtuIdx, m_alfParamTemp.alfEnabled[COMP_Y] );
        const double ctuLambda = chromaWeight > 0.0 ? (isLuma (ComponentID(compIdx)) ? cs.picture->ctuQpaLambda[curCtuIdx] : cs.picture->ctuQpaLambda[curCtuIdx] * chromaWeight) : m_lambda[compIdx];
        rate += ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
      }
    }
  }
  return rate;
}

double EncAdaptiveLoopFilter::xCodeAlfApuAlternative( CodingStructure& cs, int ctuX, int ctuY, int ctuIdx, const int compIdx, AlfParam* alfParam, const double chromaWeight )
{
  double rate = 0;
#if OPTIMIZE_ORIG_CASE
  if( m_numApusInPic == m_numCTUsInPic )
  {
    m_CABACEstimator->codeAlfCtuEnabledFlag( cs, ctuIdx, compIdx, alfParam );
    const double ctuLambda = chromaWeight > 0.0 ? (isLuma (ComponentID(compIdx)) ? cs.picture->ctuQpaLambda[ctuIdx] : cs.picture->ctuQpaLambda[ctuIdx] * chromaWeight) : m_lambda[compIdx];
    rate += ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  }
  else
#endif
  {
    int ctuMaxX = getApuMaxCtuX( ctuX );
    int ctuMaxY = getApuMaxCtuY( ctuY );

    const uint8_t alt = m_ctuAlternative[compIdx][ctuIdx];
    for( int cY = ctuY; cY < ctuMaxY; cY++ )
    {
      for( int cX = ctuX; cX < ctuMaxX; cX++ )
      {
        int curCtuIdx = cY * cs.pcv->widthInCtus + cX;
        m_ctuAlternative[compIdx][curCtuIdx] = alt;
        m_CABACEstimator->codeAlfCtuEnabledFlag( cs, curCtuIdx, compIdx, alfParam );
        const double ctuLambda = chromaWeight > 0.0 ? (isLuma (ComponentID(compIdx)) ? cs.picture->ctuQpaLambda[curCtuIdx] : cs.picture->ctuQpaLambda[curCtuIdx] * chromaWeight) : m_lambda[compIdx];
        rate += ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
      }
    }
  }
  return rate;
}


double EncAdaptiveLoopFilter::deriveCtbAlfEnableFlags( CodingStructure& cs, const int iShapeIdx, ChannelType channel, const double chromaWeight,
                                                           const int numClasses, const int numCoeff, double& distUnfilter )
{
  TempCtx        ctxTempStart( m_CtxCache );
  TempCtx        ctxTempBest( m_CtxCache );
  TempCtx        ctxTempAltStart( m_CtxCache );
  TempCtx        ctxTempAltBest( m_CtxCache );
  const ComponentID compIDFirst = isLuma( channel ) ? COMP_Y : COMP_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMP_Y : COMP_Cr;
  const int numAlts = isLuma( channel ) ? 1 : m_alfParamTemp.numAlternativesChroma;
  const double invFactor = 1.0/((double)(1<<(m_NUM_BITS-1)));
  const bool doClip = isLuma( channel ) ? m_encCfg->m_useNonLinearAlfLuma : m_encCfg->m_useNonLinearAlfChroma;
  double cost = 0;
  distUnfilter = 0;

  setEnableFlag(m_alfParamTemp, channel, true);

  reconstructCoeff(m_alfParamTemp, channel, true, isLuma(channel));
  for( int altIdx = 0; altIdx < (isLuma(channel) ? 1 : VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA); altIdx++)
  {
    for (int classIdx = 0; classIdx < (isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1); classIdx++)
    {
      for (int i = 0; i < (isLuma(channel) ? MAX_NUM_ALF_LUMA_COEFF : MAX_NUM_ALF_CHROMA_COEFF); i++)
      {
        m_filterCoeffSet[isLuma(channel) ? classIdx : altIdx][i] = isLuma(channel) ? m_coeffFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaCoeffFinal[altIdx][i];
        m_filterClippSet[isLuma(channel) ? classIdx : altIdx][i] = isLuma(channel) ? m_clippFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + i] : m_chromaClippFinal[altIdx][i];
      }
    }
  }

#if ENC_ALF_MODS
  for( int apuIdx = 0; apuIdx < m_numApusInPic; apuIdx++ )
#else
  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
#endif
  {
    int ctuX, ctuY;
    getApuCtuXY( apuIdx, ctuX, ctuY );
    int ctuIdx = ctuY * cs.pcv->widthInCtus + ctuX;

    for( int compID = compIDFirst; compID <= compIDLast; compID++ )
    {
      const double ctuLambda = chromaWeight > 0.0 ? (isLuma (channel) ? cs.picture->ctuQpaLambda[ctuIdx] : cs.picture->ctuQpaLambda[ctuIdx] * chromaWeight) : m_lambda[compID];
      double distUnfilterCtu = getUnfilteredDistortion( m_alfCovariance[compID][iShapeIdx][apuIdx], numClasses );

      ctxTempStart = AlfCtx( m_CABACEstimator->getCtx() );
      m_CABACEstimator->resetBits();
      m_ctuEnableFlag[compID][ctuIdx] = 1;
#if ENC_ALF_MODS
      double costOn = distUnfilterCtu;
      if( isLuma( channel ) )
        costOn += xCodeAlfApuLuma( cs, ctuX, ctuY, ctuIdx, compID, &m_alfParamTemp, chromaWeight );
      else
        costOn += xCodeAlfApuEnabledFlag( cs, ctuX, ctuY, ctuIdx, compID, &m_alfParamTemp, chromaWeight );
#else
      m_CABACEstimator->codeAlfCtuEnabledFlag( cs, ctuIdx, compID, &m_alfParamTemp );
      if( isLuma( channel ) )
      {
        // Evaluate cost of signaling filter set index for convergence of filters enabled flag / filter derivation
        assert( cs.picture->m_alfCtbFilterIndex[ctuIdx] == NUM_FIXED_FILTER_SETS );
        assert( cs.slice->numAps == 1 );
        m_CABACEstimator->codeAlfCtuFilterIndex(cs, ctuIdx, m_alfParamTemp.alfEnabled[COMP_Y]);
      }
      double costOn = distUnfilterCtu + ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#endif

      ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );
      if( isLuma( channel ) )
      {
        costOn += doClip ? getFilteredDistortion<true >( m_alfCovariance[compID][iShapeIdx][apuIdx], numClasses, m_alfParamTemp.numLumaFilters - 1, numCoeff )
                         : getFilteredDistortion<false>( m_alfCovariance[compID][iShapeIdx][apuIdx], numClasses, m_alfParamTemp.numLumaFilters - 1, numCoeff );;
      }
      else
      {
        double bestAltCost = MAX_DOUBLE;
        int bestAltIdx = -1;
        ctxTempAltStart = AlfCtx( ctxTempBest );
        for( int altIdx = 0; altIdx < numAlts; ++altIdx )
        {
          if( altIdx )
            m_CABACEstimator->getCtx() = AlfCtx( ctxTempAltStart );
          m_CABACEstimator->resetBits();
          m_ctuAlternative[compID][ctuIdx] = altIdx;
#if ENC_ALF_MODS
          double r_altCost = xCodeAlfApuAlternative( cs, ctuX, ctuY, ctuIdx, compID, &m_alfParamTemp, chromaWeight );
#else
          m_CABACEstimator->codeAlfCtuAlternative( cs, ctuIdx, compID, &m_alfParamTemp );
          double r_altCost = ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#endif
          double altDist = 0.;
          altDist += doClip ? m_alfCovariance[compID][iShapeIdx][apuIdx][0].calcErrorForCoeffs<true >( m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], numCoeff, invFactor )
                            : m_alfCovariance[compID][iShapeIdx][apuIdx][0].calcErrorForCoeffs<false>( m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], numCoeff, invFactor );

          double altCost = altDist + r_altCost;
          if( altCost < bestAltCost )
          {
            bestAltCost = altCost;
            bestAltIdx = altIdx;
            ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );
          }
        }
        m_ctuAlternative[compID][ctuIdx] = bestAltIdx;
        costOn += bestAltCost;
      }

      m_CABACEstimator->getCtx() = AlfCtx( ctxTempStart );
      m_CABACEstimator->resetBits();
      m_ctuEnableFlag[compID][ctuIdx] = 0;
#if ENC_ALF_MODS
      double costOff = distUnfilterCtu + xCodeAlfApuEnabledFlag( cs, ctuX, ctuY, ctuIdx, compID, &m_alfParamTemp, chromaWeight );
#else
      m_CABACEstimator->codeAlfCtuEnabledFlag( cs, ctuIdx, compID, &m_alfParamTemp);
      double costOff = distUnfilterCtu + ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#endif

      uint8_t enable = 0;
      if( costOn < costOff )
      {
        cost += costOn;
        m_CABACEstimator->getCtx() = AlfCtx( ctxTempBest );
        enable = 1;
      }
      else
      {
        cost += costOff;
        distUnfilter += distUnfilterCtu;
      }
#if ENC_ALF_MODS
      xStoreAlfApuEnabledFlag( cs, ctuX, ctuY, ctuIdx, compID, enable );
#else
      m_ctuEnableFlag[compID][ctuIdx] = enable;
#endif
    }
  }

  if( isChroma( channel ) )
  {
    setEnableFlag(m_alfParamTemp, channel, m_ctuEnableFlag);
  }

  return cost;
}

void EncAdaptiveLoopFilter::alfEncoder( CodingStructure& cs, AlfParam& alfParam, const ChannelType channel, const double lambdaChromaWeight )
{
  PROFILER_SCOPE_AND_STAGE( 0, _TPROF, P_ALF_ENC );
  const TempCtx  ctxStart( m_CtxCache, AlfCtx( m_CABACEstimator->getCtx() ) );
  TempCtx        ctxBest( m_CtxCache );

  double costMin = MAX_DOUBLE;

  std::vector<AlfFilterShape>& alfFilterShape = alfParam.filterShapes[channel];
  m_bitsNewFilter[channel] = 0;
  const int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  int uiCoeffBits = 0;

  for( int iShapeIdx = 0; iShapeIdx < alfFilterShape.size(); iShapeIdx++ )
  {
    m_alfParamTemp = alfParam;
    //1. get unfiltered distortion
    if( isChroma(channel) )
      m_alfParamTemp.numAlternativesChroma = 1;
    double cost = getUnfilteredDistortion( m_alfCovarianceFrame[channel][iShapeIdx], channel );
    cost /= 1.001; // slight preference for unfiltered choice

    if( cost < costMin )
    {
      costMin = cost;
      setEnableFlag( alfParam, channel, false );
      // no CABAC signalling
      ctxBest = AlfCtx( ctxStart );
      setCtuEnableFlag( m_ctuEnableFlagTmp, channel, 0 );
      if( isChroma(channel) )
        setCtuAlternativeChroma( m_ctuAlternativeTmp, 0 );
    }

    const int nonLinearFlagMax =
      ( isLuma( channel ) ? m_encCfg->m_useNonLinearAlfLuma : m_encCfg->m_useNonLinearAlfChroma ) // For Chroma non linear flag is check for each alternative filter
      ? 2 : 1;

    for( int nonLinearFlag = 0; nonLinearFlag < nonLinearFlagMax; nonLinearFlag++ )
    {
      for( int numAlternatives = isLuma( channel ) ? 1 : getMaxNumAlternativesChroma(); numAlternatives > 0; numAlternatives-- )
      {
        if( isChroma( channel ) )
          m_alfParamTemp.numAlternativesChroma = numAlternatives;
        //2. all CTUs are on
        setEnableFlag( m_alfParamTemp, channel, true );
        m_alfParamTemp.nonLinearFlag[channel] = nonLinearFlag;
        m_CABACEstimator->getCtx() = AlfCtx( ctxStart );
        setCtuEnableFlag( m_ctuEnableFlag, channel, 1 );
        // all alternatives are on
        if( isChroma( channel ) )
          initCtuAlternativeChroma( m_ctuAlternative );
        cost = getFilterCoeffAndCost( cs, 0, channel, true, iShapeIdx, uiCoeffBits );

        if( cost < costMin )
        {
          m_bitsNewFilter[channel] = uiCoeffBits;
          costMin = cost;
          copyAlfParam( alfParam, m_alfParamTemp, channel );
          ctxBest = AlfCtx( m_CABACEstimator->getCtx() );
          setCtuEnableFlag( m_ctuEnableFlagTmp, channel, 1 );
          if( isChroma(channel) )
            copyCtuAlternativeChroma( m_ctuAlternativeTmp, m_ctuAlternative );
        }

        //3. CTU decision
        double distUnfilter = 0;
        double prevItCost = MAX_DOUBLE;
        const int iterNum = isLuma(channel) ? (2 * 4 + 1) : (2 * (2 + m_alfParamTemp.numAlternativesChroma - 1) + 1);

        for( int iter = 0; iter < iterNum; iter++ )
        {
          if ((iter & 0x01) == 0)
          {
            m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
            cost = m_lambda[channel] * uiCoeffBits;
            cost += deriveCtbAlfEnableFlags(cs, iShapeIdx, channel, lambdaChromaWeight,
                                            numClasses, alfFilterShape[iShapeIdx].numCoeff, distUnfilter);
            if (cost < costMin)
            {
              m_bitsNewFilter[channel] = uiCoeffBits;
              costMin = cost;
              ctxBest = AlfCtx(m_CABACEstimator->getCtx());
              copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, channel);
              if( isChroma(channel) )
                copyCtuAlternativeChroma( m_ctuAlternativeTmp, m_ctuAlternative );
              copyAlfParam(alfParam, m_alfParamTemp, channel);
            }
            else if ( cost >= prevItCost  )
            {
              // High probability that we have converged or we are diverging
              break;
            }
            prevItCost = cost;
          }
          else
          {
            // unfiltered distortion is added due to some CTBs may not use filter
            // no need to reset CABAC here, since uiCoeffBits is not affected
            /*cost = */getFilterCoeffAndCost( cs, distUnfilter, channel, true, iShapeIdx, uiCoeffBits );
          }
        }//for iter
        // Decrease number of alternatives and reset ctu params and filters
      }
    }// for nonLineaFlag
  }//for shapeIdx
  m_CABACEstimator->getCtx() = AlfCtx( ctxBest );
  if( isChroma(channel) )
    copyCtuAlternativeChroma( m_ctuAlternative, m_ctuAlternativeTmp );
  copyCtuEnableFlag( m_ctuEnableFlag, m_ctuEnableFlagTmp, channel );
}

void EncAdaptiveLoopFilter::copyAlfParam( AlfParam& alfParamDst, AlfParam& alfParamSrc, ChannelType channel )
{
  if( isLuma( channel ) )
  {
    alfParamDst = alfParamSrc;
  }
  else
  {
    alfParamDst.alfEnabled[COMP_Cb] = alfParamSrc.alfEnabled[COMP_Cb];
    alfParamDst.alfEnabled[COMP_Cr] = alfParamSrc.alfEnabled[COMP_Cr];
    alfParamDst.numAlternativesChroma = alfParamSrc.numAlternativesChroma;
    alfParamDst.nonLinearFlag[CH_C] = alfParamSrc.nonLinearFlag[CH_C];
    memcpy( alfParamDst.chromaCoeff, alfParamSrc.chromaCoeff, sizeof( alfParamDst.chromaCoeff ) );
    memcpy( alfParamDst.chromaClipp, alfParamSrc.chromaClipp, sizeof( alfParamDst.chromaClipp ) );
  }
}

double EncAdaptiveLoopFilter::getFilterCoeffAndCost( CodingStructure& cs, double distUnfilter, ChannelType channel, bool bReCollectStat, int iShapeIdx, int& uiCoeffBits, bool onlyFilterCost )
{
  //collect stat based on CTU decision
  if( bReCollectStat )
  {
    getFrameStats( channel, iShapeIdx );
  }

  double dist = distUnfilter;
  uiCoeffBits = 0;
  AlfFilterShape& alfFilterShape = m_alfParamTemp.filterShapes[channel][iShapeIdx];
  //get filter coeff
  if( isLuma( channel ) )
  {
    std::fill_n(m_alfClipMerged[iShapeIdx][0][0], MAX_NUM_ALF_LUMA_COEFF*MAX_NUM_ALF_CLASSES*MAX_NUM_ALF_CLASSES, m_alfParamTemp.nonLinearFlag[channel] ? AlfNumClippingValues / 2 : 0);
    // Reset Merge Tmp Cov
    m_alfCovarianceMerged[iShapeIdx][MAX_NUM_ALF_CLASSES].reset();
    m_alfCovarianceMerged[iShapeIdx][MAX_NUM_ALF_CLASSES + 1].reset();
    //distortion
    dist += mergeFiltersAndCost( m_alfParamTemp, alfFilterShape, m_alfCovarianceFrame[channel][iShapeIdx], m_alfCovarianceMerged[iShapeIdx], m_alfClipMerged[iShapeIdx], uiCoeffBits );
  }
  else
  {
    //distortion
    for( int altIdx = 0; altIdx < m_alfParamTemp.numAlternativesChroma; ++altIdx )
    {
      assert(alfFilterShape.numCoeff == m_alfCovarianceFrame[channel][iShapeIdx][altIdx].numCoeff);
      AlfParam bestSliceParam;
      double bestCost = MAX_DOUBLE;
      double bestDist = MAX_DOUBLE;
      int bestCoeffBits = 0;
      const int nonLinearFlagMax = m_encCfg->m_useNonLinearAlfChroma ? 2 : 1;

      for( int nonLinearFlag = 0; nonLinearFlag < nonLinearFlagMax; nonLinearFlag++ )
      {
        int currentNonLinearFlag = m_alfParamTemp.nonLinearFlag[channel] ? 1 : 0;
        if (nonLinearFlag != currentNonLinearFlag)
        {
          continue;
        }

        std::fill_n(m_filterClippSet[altIdx], MAX_NUM_ALF_CHROMA_COEFF, nonLinearFlag ? AlfNumClippingValues / 2 : 0 );
        double dist = m_alfCovarianceFrame[channel][iShapeIdx][altIdx].pixAcc + deriveCoeffQuant( m_filterClippSet[altIdx], m_filterCoeffSet[altIdx], m_alfCovarianceFrame[channel][iShapeIdx][altIdx], alfFilterShape, m_NUM_BITS, nonLinearFlag );
        for( int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++ )
        {
          m_alfParamTemp.chromaCoeff[altIdx][i] = m_filterCoeffSet[altIdx][i];
          m_alfParamTemp.chromaClipp[altIdx][i] = m_filterClippSet[altIdx][i];
        }
        int coeffBits = getChromaCoeffRate( m_alfParamTemp, altIdx );
        double cost = dist + m_lambda[channel] * coeffBits;
        if( cost < bestCost )
        {
          bestCost = cost;
          bestDist = dist;
          bestCoeffBits = coeffBits;
          bestSliceParam = m_alfParamTemp;
        }
      }
      uiCoeffBits += bestCoeffBits;
      dist += bestDist;
      m_alfParamTemp = bestSliceParam;
    }
    uiCoeffBits += lengthUvlc( m_alfParamTemp.numAlternativesChroma-1 );
    uiCoeffBits++;
  }
  if (onlyFilterCost)
  {
    return dist + m_lambda[channel] * uiCoeffBits;
  }
  double rate = uiCoeffBits;
  m_CABACEstimator->resetBits();
  m_CABACEstimator->codeAlfCtuEnabled( cs, channel, &m_alfParamTemp);
  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
  {
    if( isLuma( channel ) )
    {
      // Evaluate cost of signaling filter set index for convergence of filters enabled flag / filter derivation
      assert( cs.picture->m_alfCtbFilterIndex[ctuIdx] == NUM_FIXED_FILTER_SETS );
      assert( cs.slice->numAps == 1 );
      m_CABACEstimator->codeAlfCtuFilterIndex(cs, ctuIdx, m_alfParamTemp.alfEnabled[COMP_Y]);
    }
  }
  m_CABACEstimator->codeAlfCtuAlternatives( cs, channel, &m_alfParamTemp );
  rate += FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
  return dist + m_lambda[channel] * rate;
}

int EncAdaptiveLoopFilter::getChromaCoeffRate( AlfParam& alfParam, int altIdx )
{
  int iBits = 0;

  AlfFilterShape alfShape(5);
  // Filter coefficients
  for( int i = 0; i < alfShape.numCoeff - 1; i++ )
  {
   iBits += lengthUvlc( abs( alfParam.chromaCoeff[ altIdx ][ i ] ) );  // alf_coeff_chroma[altIdx][i]
    if( ( alfParam.chromaCoeff[ altIdx ][ i ] ) != 0 )
      iBits += 1;
   }

  if( m_alfParamTemp.nonLinearFlag[CH_C] )
  {
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      if( !abs( alfParam.chromaCoeff[altIdx][i] ) )
      {
        alfParam.chromaClipp[altIdx][i] = 0;
      }
    }
    iBits += ((alfShape.numCoeff - 1) << 1);
  }
  return iBits;
}

double EncAdaptiveLoopFilter::getUnfilteredDistortion( AlfCovariance* cov, ChannelType channel )
{
  double dist = 0;
  if( isLuma( channel ) )
  {
    dist = getUnfilteredDistortion( cov, MAX_NUM_ALF_CLASSES );
  }
  else
  {
    dist = getUnfilteredDistortion( cov, 1 );
  }
  return dist;
}

double EncAdaptiveLoopFilter::getUnfilteredDistortion( AlfCovariance* cov, const int numClasses )
{
  double dist = 0;
  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    dist += cov[classIdx].pixAcc;
  }
  return dist;
}

template<bool doClip>
double EncAdaptiveLoopFilter::getFilteredDistortion( AlfCovariance* cov, const int numClasses, const int numFiltersMinus1, const int numCoeff )
{
  const double invFactor = 1.0/((double)(1<<(m_NUM_BITS-1)));
  double dist = 0;

  for( int classIdx = 0; classIdx < numClasses; classIdx++ )
  {
    dist += cov[classIdx].calcErrorForCoeffs<doClip>(m_filterClippSet[classIdx], m_filterCoeffSet[classIdx], numCoeff, invFactor);
  }

  return dist;
}

double EncAdaptiveLoopFilter::mergeFiltersAndCost( AlfParam& alfParam, AlfFilterShape& alfShape, AlfCovariance* covFrame, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], int& uiCoeffBits )
{
  int numFiltersBest = 0;
  int numFilters = MAX_NUM_ALF_CLASSES;
  bool codedVarBins[MAX_NUM_ALF_CLASSES];
  alf_float_t errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2];

  double cost, cost0, dist, distForce0, costMin = MAX_DOUBLE;
  int coeffBits, coeffBitsForce0;

  mergeClasses( alfShape, covFrame, covMerged, clipMerged, MAX_NUM_ALF_CLASSES, m_filterIndices );

  while( numFilters >= 1 )
  {
    // filter coeffs are stored in m_filterCoeffSet
    dist      = deriveFilterCoeffs( covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFilters - 1], numFilters, errorForce0CoeffTab, alfParam );
    coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFilters );
    distForce0      = getDistForce0( alfShape, numFilters, errorForce0CoeffTab, codedVarBins );
    coeffBitsForce0 = getCostFilterCoeffForce0( alfShape, m_filterCoeffSet, numFilters, codedVarBins );

    cost = dist + m_lambda[COMP_Y] * coeffBits;
    cost0 = distForce0 + m_lambda[COMP_Y] * coeffBitsForce0;

    if( cost0 < cost )
    {
      cost = cost0;
    }

    if( cost <= costMin )
    {
      costMin = cost;
      numFiltersBest = numFilters;
    }
    numFilters--;
  }

  dist = deriveFilterCoeffs( covFrame, covMerged, clipMerged, alfShape, m_filterIndices[numFiltersBest - 1], numFiltersBest, errorForce0CoeffTab, alfParam );
  coeffBits = deriveFilterCoefficientsPredictionMode( alfShape, m_filterCoeffSet, m_diffFilterCoeff, numFiltersBest );
  distForce0 = getDistForce0( alfShape, numFiltersBest, errorForce0CoeffTab, codedVarBins );
  coeffBitsForce0 = getCostFilterCoeffForce0( alfShape, m_filterCoeffSet, numFiltersBest, codedVarBins );

  cost = dist + m_lambda[COMP_Y] * coeffBits;
  cost0 = distForce0 + m_lambda[COMP_Y] * coeffBitsForce0;

  alfParam.numLumaFilters = numFiltersBest;
  double distReturn;
  if (cost <= cost0)
  {
    distReturn = dist;
    alfParam.alfLumaCoeffDeltaFlag = 0;
    uiCoeffBits = coeffBits;
  }
  else
  {
    distReturn = distForce0;
    alfParam.alfLumaCoeffDeltaFlag = 1;
    uiCoeffBits = coeffBitsForce0;
    memcpy( alfParam.alfLumaCoeffFlag, codedVarBins, sizeof( codedVarBins ) );

    for( int varInd = 0; varInd < numFiltersBest; varInd++ )
    {
      if( codedVarBins[varInd] == 0 )
      {
        memset( m_filterCoeffSet[varInd], 0, sizeof( int )*MAX_NUM_ALF_LUMA_COEFF );
        memset( m_filterClippSet[varInd], 0, sizeof( int )*MAX_NUM_ALF_LUMA_COEFF );
      }
    }
  }

  for( int ind = 0; ind < alfParam.numLumaFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff; i++ )
    {
      alfParam.lumaCoeff[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterCoeffSet[ind][i];
      alfParam.lumaClipp[ind * MAX_NUM_ALF_LUMA_COEFF + i] = m_filterClippSet[ind][i];
    }
  }

  memcpy( alfParam.filterCoeffDeltaIdx, m_filterIndices[numFiltersBest - 1], sizeof( short ) * MAX_NUM_ALF_CLASSES );
  uiCoeffBits += getNonFilterCoeffRate( alfParam );
  return distReturn;
}

int EncAdaptiveLoopFilter::getNonFilterCoeffRate( AlfParam& alfParam )
{
  int len = 2 + lengthUvlc (alfParam.numLumaFilters - 1);
  if( alfParam.numLumaFilters > 1 )
  {
    const int coeffLength = ceilLog2(alfParam.numLumaFilters);
    for( int i = 0; i < MAX_NUM_ALF_CLASSES; i++ )
    {
      len += coeffLength;                              // alf_luma_coeff_delta_idx   u(v)
    }
  }
  return len;
}


int EncAdaptiveLoopFilter::getCostFilterCoeffForce0( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters, bool* codedVarBins )
{
  int len = 0;
  // Filter coefficients
  for( int ind = 0; ind < numFilters; ++ind )
  {
    if( codedVarBins[ind] )
    {
      for( int i = 0; i < alfShape.numCoeff - 1; i++ )
      {
        len += lengthUvlc( abs( pDiffQFilterCoeffIntPP[ ind ][ i ] ) ); // alf_coeff_luma_delta[i][j]
        if( ( abs( pDiffQFilterCoeffIntPP[ ind ][ i ] ) != 0 ) )
          len += 1;
      }
    }
    else
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        len += lengthUvlc(0); // alf_coeff_luma_delta[i][j]
      }
    }
  }

  if( m_alfParamTemp.nonLinearFlag[CH_L] )
  {
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        if (!abs(pDiffQFilterCoeffIntPP[ind][i]))
        {
          m_filterClippSet[ind][i] = 0;
        }
        len += 2;
      }
    }
  }

  return len;
}

int EncAdaptiveLoopFilter::deriveFilterCoefficientsPredictionMode( AlfFilterShape& alfShape, int **filterSet, int** filterCoeffDiff, const int numFilters )
{
  return (m_alfParamTemp.nonLinearFlag[CH_L] ? getCostFilterClipp(alfShape, filterSet, numFilters) : 0) + getCostFilterCoeff(alfShape, filterSet, numFilters);
}

int EncAdaptiveLoopFilter::getCostFilterCoeff( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters )
{
  return lengthFilterCoeffs( alfShape, numFilters, pDiffQFilterCoeffIntPP );  // alf_coeff_luma_delta[i][j];
}

int EncAdaptiveLoopFilter::getCostFilterClipp( AlfFilterShape& alfShape, int **pDiffQFilterCoeffIntPP, const int numFilters )
{
  for (int filterIdx = 0; filterIdx < numFilters; ++filterIdx)
  {
    for (int i = 0; i < alfShape.numCoeff - 1; i++)
    {
      if (!abs(pDiffQFilterCoeffIntPP[filterIdx][i]))
      {
        m_filterClippSet[filterIdx][i] = 0;
      }
    }
  }
  return (numFilters * (alfShape.numCoeff - 1)) << 1;
}

int EncAdaptiveLoopFilter::lengthFilterCoeffs( AlfFilterShape& alfShape, const int numFilters, int **FilterCoeff )
{
  int bitCnt = 0;

  for( int ind = 0; ind < numFilters; ++ind )
  {
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      bitCnt += lengthUvlc( abs( FilterCoeff[ ind ][ i ] ) );
      if( abs( FilterCoeff[ ind ][ i ] ) != 0 )
        bitCnt += 1;
    }
  }
  return bitCnt;
}


alf_float_t EncAdaptiveLoopFilter::getDistForce0( AlfFilterShape& alfShape, const int numFilters, alf_float_t errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], bool* codedVarBins )
{
  int bitsVarBin[MAX_NUM_ALF_CLASSES];

  for( int ind = 0; ind < numFilters; ++ind )
  {
    bitsVarBin[ind] = 0;
    for( int i = 0; i < alfShape.numCoeff - 1; i++ )
    {
      bitsVarBin[ ind ] += lengthUvlc( abs( m_filterCoeffSet[ ind ][ i ] ) );
      if( abs( m_filterCoeffSet[ ind ][ i ] ) != 0 )
        bitsVarBin[ ind ] += 1;
    }
  }

  int zeroBitsVarBin = 0;
  for (int i = 0; i < alfShape.numCoeff - 1; i++)
  {
    zeroBitsVarBin += lengthUvlc( 0 );
  }
  if( m_alfParamTemp.nonLinearFlag[CH_L] )
  {
    for (int ind = 0; ind < numFilters; ++ind)
    {
      for (int i = 0; i < alfShape.numCoeff - 1; i++)
      {
        if (!abs(m_filterCoeffSet[ind][i]))
        {
          m_filterClippSet[ind][i] = 0;
        }
      }
    }
  }

  double distForce0 = getDistCoeffForce0( codedVarBins, errorTabForce0Coeff, bitsVarBin, zeroBitsVarBin, numFilters);

  return distForce0;
}
alf_float_t EncAdaptiveLoopFilter::getDistCoeffForce0( bool* codedVarBins, alf_float_t errorForce0CoeffTab[MAX_NUM_ALF_CLASSES][2], int* bitsVarBin, int zeroBitsVarBin, const int numFilters)
{
  double distForce0 = 0;
  std::memset( codedVarBins, 0, sizeof( *codedVarBins ) * MAX_NUM_ALF_CLASSES );

  for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
  {
    double costDiff = (errorForce0CoeffTab[filtIdx][0] + m_lambda[COMP_Y] * zeroBitsVarBin) - (errorForce0CoeffTab[filtIdx][1] + m_lambda[COMP_Y] * bitsVarBin[filtIdx]);
    codedVarBins[filtIdx] = costDiff > 0 ? true : false;
    distForce0 += errorForce0CoeffTab[filtIdx][codedVarBins[filtIdx] ? 1 : 0];
  }

  return distForce0;
}

int EncAdaptiveLoopFilter::lengthUvlc( int uiCode )
{
  int uiLength = 1;
  int uiTemp = ++uiCode;

  CHECK( !uiTemp, "Integer overflow" );

  while( 1 != uiTemp )
  {
    uiTemp >>= 1;
    uiLength += 2;
  }
  // Take care of cases where uiLength > 32
  return ( uiLength >> 1 ) + ( ( uiLength + 1 ) >> 1 );
}


double EncAdaptiveLoopFilter::deriveFilterCoeffs( AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], AlfFilterShape& alfShape, short* filterIndices, int numFilters, alf_float_t errorTabForce0Coeff[MAX_NUM_ALF_CLASSES][2], AlfParam& alfParam )
{
  PROFILER_SCOPE_AND_STAGE( 0, _TPROF, P_ALF_DERIVE_COEF );
  double error = 0.0;
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];

  for( int filtIdx = 0; filtIdx < numFilters; filtIdx++ )
  {
    tmpCov.reset();
    bool found_clip = false;
    for( int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++ )
    {
      if( filterIndices[classIdx] == filtIdx )
      {
        tmpCov += cov[classIdx];
        if( !found_clip )
        {
          found_clip = true; // clip should be at the adress of shortest one
          memcpy(m_filterClippSet[filtIdx], clipMerged[numFilters-1][classIdx], sizeof(int[MAX_NUM_ALF_LUMA_COEFF]));
        }
      }
    }

    // Find coeffcients
    assert(alfShape.numCoeff == tmpCov.numCoeff);
    errorTabForce0Coeff[filtIdx][1] = tmpCov.pixAcc + deriveCoeffQuant( m_filterClippSet[filtIdx], m_filterCoeffSet[filtIdx], tmpCov, alfShape, m_NUM_BITS, false );
    errorTabForce0Coeff[filtIdx][0] = tmpCov.pixAcc;
    error += errorTabForce0Coeff[filtIdx][1];
  }
  return error;
}

double EncAdaptiveLoopFilter::deriveCoeffQuant( int *filterClipp, int *filterCoeffQuant, const AlfCovariance& cov, const AlfFilterShape& shape, const int bitDepth, const bool optimizeClip )
{
  const int factor = 1 << ( bitDepth - 1 );
  const int max_value = factor - 1;
  const int min_value = -factor + 1;
  const alf_float_t invFactor = 1.0 /((double)factor);
  const bool doClip = m_encCfg->m_useNonLinearAlfLuma || m_encCfg->m_useNonLinearAlfChroma;

  const int numCoeff = shape.numCoeff;
  alf_float_t filterCoeff[MAX_NUM_ALF_LUMA_COEFF];

  cov.optimizeFilter( shape, filterClipp, filterCoeff, optimizeClip );
  roundFiltCoeff( filterCoeffQuant, filterCoeff, numCoeff, factor );

  for ( int i = 0; i < numCoeff - 1; i++ )
  {
    filterCoeffQuant[i] = std::min( max_value, std::max( min_value, filterCoeffQuant[i] ) );
  }
  filterCoeffQuant[numCoeff - 1] = 0;

  int modified=1;

  double errRef = doClip ? cov.calcErrorForCoeffs<true >( filterClipp, filterCoeffQuant, numCoeff, invFactor )
                         : cov.calcErrorForCoeffs<false>( filterClipp, filterCoeffQuant, numCoeff, invFactor );
  while( modified )
  {
    modified=0;
    for( int sign: {1, -1} )
    {
      double errMin = MAX_DOUBLE;
      int minInd = -1;

      for( int k = 0; k < numCoeff-1; k++ )
      {
        if( filterCoeffQuant[k] - sign > max_value || filterCoeffQuant[k] - sign < min_value )
          continue;

        double error = errRef + (doClip ? cov.calcDiffErrorForCoeffs<true >( filterClipp, filterCoeffQuant, numCoeff, filterCoeffQuant[k], filterCoeffQuant[k] - sign, k, invFactor )
                                        : cov.calcDiffErrorForCoeffs<false>( filterClipp, filterCoeffQuant, numCoeff, filterCoeffQuant[k], filterCoeffQuant[k] - sign, k, invFactor ));

        if( error < errMin )
        {
          errMin = error;
          minInd = k;
        }
      }
      if( errMin < errRef )
      {
        filterCoeffQuant[minInd] -= sign;
        modified++;
        errRef = errMin;
      }
    }
  }

  return errRef;
}

void EncAdaptiveLoopFilter::roundFiltCoeff( int *filterCoeffQuant, alf_float_t*filterCoeff, const int numCoeff, const int factor )
{
  for( int i = 0; i < numCoeff; i++ )
  {
    int sign = filterCoeff[i] > 0 ? 1 : -1;
    filterCoeffQuant[i] = int( filterCoeff[i] * sign * factor + 0.5 ) * sign;
  }
}

void EncAdaptiveLoopFilter::roundFiltCoeffCCALF( int16_t *filterCoeffQuant, alf_float_t*filterCoeff, const int numCoeff, const int factor )
{
  for( int i = 0; i < numCoeff; i++ )
  {
    int sign = filterCoeff[i] > 0 ? 1 : -1;
    double best_err = 128.0*128.0;
    int best_index = 0;
    for(int k = 0; k < CCALF_CANDS_COEFF_NR; k++)
    {
      double err = (filterCoeff[i] * sign * factor - CCALF_SMALL_TAB[k]);
      err = err*err;
      if(err < best_err)
      {
        best_err = err;
        best_index = k;
      }
    }
    filterCoeffQuant[i] = CCALF_SMALL_TAB[best_index] * sign;
  }
}

void EncAdaptiveLoopFilter::mergeClasses( const AlfFilterShape& alfShape, AlfCovariance* cov, AlfCovariance* covMerged, int clipMerged[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF], const int numClasses, short filterIndices[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_CLASSES] )
{
  PROFILER_SCOPE_AND_STAGE( 0, _TPROF, P_ALF_MERGE );
  int       tmpClip          [MAX_NUM_ALF_LUMA_COEFF];
  int       bestMergeClip    [MAX_NUM_ALF_LUMA_COEFF];
  double    err              [MAX_NUM_ALF_CLASSES];
  double    bestMergeErr     =MAX_DOUBLE;
  bool      availableClass   [MAX_NUM_ALF_CLASSES];
  uint8_t   indexList        [MAX_NUM_ALF_CLASSES];
  uint8_t   indexListTemp    [MAX_NUM_ALF_CLASSES];
  int numRemaining = numClasses;

  memset( filterIndices, 0, sizeof( short ) * MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_CLASSES );

  for( int i = 0; i < numClasses; i++ )
  {
    filterIndices[numRemaining - 1][i] = i;
    indexList[i] = i;
    availableClass[i] = true;
    covMerged[i] = cov[i];
    covMerged[i].numBins = m_alfParamTemp.nonLinearFlag[CH_L] ? AlfNumClippingValues : 1;
  }

  // Try merging different covariance matrices

  // temporal AlfCovariance structure is allocated as the last element in covMerged array, the size of covMerged is MAX_NUM_ALF_CLASSES + 1
  AlfCovariance& tmpCov = covMerged[MAX_NUM_ALF_CLASSES];
  tmpCov.numBins = m_alfParamTemp.nonLinearFlag[CH_L] ? AlfNumClippingValues : 1;

  // init Clip
  for( int i = 0; i < numClasses; i++ )
  {
    std::fill_n(clipMerged[numRemaining-1][i], MAX_NUM_ALF_LUMA_COEFF, m_alfParamTemp.nonLinearFlag[CH_L] ? AlfNumClippingValues / 2 : 0);
    if ( m_alfParamTemp.nonLinearFlag[CH_L] )
    {
      err[i] = covMerged[i].optimizeFilterClip( alfShape, clipMerged[numRemaining-1][i] );
    }
    else
    {
      err[i] = covMerged[i].calculateError( clipMerged[numRemaining-1][i] );
    }
  }

  while( numRemaining >= 2 )
  {
    double errorMin = std::numeric_limits<double>::max();
    int bestToMergeIdx1 = 0, bestToMergeIdx2 = 1;

    for( int i = 0; i < numClasses - 1; i++ )
    {
      if( availableClass[i] )
      {
        for( int j = i + 1; j < numClasses; j++ )
        {
          if( availableClass[j] )
          {
            double error1 = err[i];
            double error2 = err[j];

            tmpCov.add( covMerged[i], covMerged[j] );
            for( int l = 0; l < MAX_NUM_ALF_LUMA_COEFF; ++l )
            {
              tmpClip[l] = (clipMerged[numRemaining-1][i][l] + clipMerged[numRemaining-1][j][l] + 1 ) >> 1;
            }
            double errorMerged = m_alfParamTemp.nonLinearFlag[CH_L] ? tmpCov.optimizeFilterClip( alfShape, tmpClip ) : tmpCov.calculateError( tmpClip );
            double error = errorMerged - error1 - error2;

            if( error < errorMin )
            {
              bestMergeErr = errorMerged;
              memcpy(bestMergeClip, tmpClip, sizeof(bestMergeClip));
              errorMin = error;
              bestToMergeIdx1 = i;
              bestToMergeIdx2 = j;
            }
          }
        }
      }
    }

    covMerged[bestToMergeIdx1] += covMerged[bestToMergeIdx2];
    memcpy(clipMerged[numRemaining-2], clipMerged[numRemaining-1], sizeof(int[MAX_NUM_ALF_CLASSES][MAX_NUM_ALF_LUMA_COEFF]));
    memcpy(clipMerged[numRemaining-2][bestToMergeIdx1], bestMergeClip, sizeof(bestMergeClip));
    err[bestToMergeIdx1] = bestMergeErr;
    availableClass[bestToMergeIdx2] = false;

    for( int i = 0; i < numClasses; i++ )
    {
      if( indexList[i] == bestToMergeIdx2 )
      {
        indexList[i] = bestToMergeIdx1;
      }
    }

    numRemaining--;
    if( numRemaining <= numClasses )
    {
      std::memcpy( indexListTemp, indexList, sizeof( uint8_t ) * numClasses );

      bool exist = false;
      int ind = 0;

      for( int j = 0; j < numClasses; j++ )
      {
        exist = false;
        for( int i = 0; i < numClasses; i++ )
        {
          if( indexListTemp[i] == j )
          {
            exist = true;
            break;
          }
        }

        if( exist )
        {
          for( int i = 0; i < numClasses; i++ )
          {
            if( indexListTemp[i] == j )
            {
              filterIndices[numRemaining - 1][i] = ind;
              indexListTemp[i] = -1;
            }
          }
          ind++;
        }
      }
    }
  }
}

void EncAdaptiveLoopFilter::getFrameStats( ChannelType channel, int iShapeIdx )
{
  int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  int numAlternatives = isLuma( channel ) ? 1 : m_alfParamTemp.numAlternativesChroma;
  // When calling this function m_ctuEnableFlag shall be set to 0 for CTUs using alternative APS
  // Here we compute frame stats for building new alternative filters
  for( int altIdx = 0; altIdx < numAlternatives; ++altIdx )
  {
    for( int i = 0; i < numClasses; i++ )
    {
      m_alfCovarianceFrame[channel][iShapeIdx][isLuma( channel ) ? i : altIdx].reset();
    }
    if( isLuma( channel ) )
    {
      getFrameStat( m_alfCovarianceFrame[CH_L][iShapeIdx], m_alfCovariance[COMP_Y][iShapeIdx], m_ctuEnableFlag[COMP_Y], nullptr, numClasses, altIdx );
    }
    else
    {
      getFrameStat( m_alfCovarianceFrame[CH_C][iShapeIdx], m_alfCovariance[COMP_Cb][iShapeIdx], m_ctuEnableFlag[COMP_Cb], m_ctuAlternative[COMP_Cb], numClasses, altIdx );
      getFrameStat( m_alfCovarianceFrame[CH_C][iShapeIdx], m_alfCovariance[COMP_Cr][iShapeIdx], m_ctuEnableFlag[COMP_Cr], m_ctuAlternative[COMP_Cr], numClasses, altIdx );
    }
  }
}

void EncAdaptiveLoopFilter::getFrameStat( AlfCovariance* frameCov, AlfCovariance** ctbCov, uint8_t* ctbEnableFlags, uint8_t* ctbAltIdx, const int numClasses, int altIdx )
{
  if( !ctbAltIdx )
  {
#if ENC_ALF_MODS
    for( int ctuIdx = 0; ctuIdx < m_numApusInPic; ctuIdx++ )
#else
    for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
#endif
    {
      if( ctbEnableFlags[ctuIdx] )
      {
        for( int classIdx = 0; classIdx < numClasses; classIdx++ )
        {
          frameCov[classIdx] += ctbCov[ctuIdx][classIdx];
        }
      }
    }
  }
  else
  {
#if ENC_ALF_MODS
    for( int apuIdx = 0; apuIdx < m_numApusInPic; apuIdx++ )
    {
      int ctuX, ctuY;
      getApuCtuXY( apuIdx, ctuX, ctuY );
      int ctuIdx = ctuY * m_numCTUsInWidth/*cs.pcv->widthInCtus*/ + ctuX;
      if( ctbEnableFlags[ctuIdx]  && ( altIdx == ctbAltIdx[ctuIdx] ))
      {
        for( int classIdx = 0; classIdx < numClasses; classIdx++ )
        {
          frameCov[altIdx] += ctbCov[apuIdx][classIdx];
        }
      }
    }
#else
    for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++ )
    {
      if( ctbEnableFlags[ctuIdx]  && ( altIdx == ctbAltIdx[ctuIdx] ))
      {
        for( int classIdx = 0; classIdx < numClasses; classIdx++ )
        {
          frameCov[altIdx] += ctbCov[ctuIdx][classIdx];
        }
      }
    }
#endif
  }
}


void EncAdaptiveLoopFilter::getPreBlkStats(AlfCovariance* alfCovariance, const AlfFilterShape& shape, AlfClassifier* classifier, Pel* org, const int orgStride, Pel* rec, const int recStride, const CompArea& areaDst, const CompArea& area, const ChannelType channel, int vbCTUHeight, int vbPos)
{
  constexpr int numBins = AlfNumClippingValues;
  int transposeIdx = 0;
  int classIdx = 0;

  const int halfFilterLength = shape.filterLength >> 1;
  int filterPattern[25]; // max entries in shape.pattern

  if( !m_encCfg->m_useNonLinearAlfLuma && !m_encCfg->m_useNonLinearAlfChroma && ( area.width & 3 ) == 0 )
  {
    for( int i = 0; i < shape.filterSize; i++ ) filterPattern[i] = shape.pattern[i] << 4;
  }

  if( !m_encCfg->m_useNonLinearAlfLuma && !m_encCfg->m_useNonLinearAlfChroma && ( area.width & 3 ) == 0 )
  {
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_ALF
    bool useSimd = read_x86_extension_flags() > SCALAR;
#else
    bool useSimd = false;
#endif
    for( int i = 0; i < area.height; i += 4 )
    {
      const int maxFilterSamples = 4;

      int clipTopRow[4] = { -maxFilterSamples, -maxFilterSamples, -maxFilterSamples, -maxFilterSamples };
      int clipBotRow[4] = {  maxFilterSamples,  maxFilterSamples,  maxFilterSamples,  maxFilterSamples };

      for( int ii = 0; ii < 4; ii++ )
      {
        int vbDistance = ((areaDst.y + i + ii) % vbCTUHeight) - vbPos;
        if( vbDistance >= -3 && vbDistance < 0 )
        {
          clipBotRow[ii] = -vbDistance - 1;
          clipTopRow[ii] = -clipBotRow[ii]; // symmetric
        }
        else if( vbDistance >= 0 && vbDistance < 3 )
        {
          clipTopRow[ii] = -vbDistance;
          clipBotRow[ii] = -clipTopRow[ii]; // symmetric
        }
      }

      for( int j = 0; j < area.width; j += 4 )
      {
        const int idx = ( i / 4 ) * ( MAX_CU_SIZE / 4 ) + j / 4;
        if( classifier && classifier[idx].classIdx == m_ALF_UNUSED_CLASSIDX && classifier[idx].transposeIdx == m_ALF_UNUSED_TRANSPOSIDX )
        {
          continue;
        }
        if( classifier )
        {
          AlfClassifier& cl = classifier[idx];
          transposeIdx = cl.transposeIdx;
          classIdx = cl.classIdx;
        }

        Pel yLocal[4][4];
        for( int ii = 0; ii < 4; ii++ ) for( int jj = 0; jj < 4; jj++ )
        {
          yLocal[ii][jj] = org[j + jj + ii * orgStride] - rec[j + jj + ii * recStride];
        }

        //      std::memset( ELocal, 0, sizeof( ELocal ) );
        Pel ELocal[MAX_NUM_ALF_LUMA_COEFF << 4];

        if( useSimd )
        {
          for( int ii = 0; ii < 4; ii++ )
          {
            if( clipBotRow[ii] == 4 ) 
              calcLinCovariance4<false, true>( ELocal + ( ii << 2 ), rec + j + ii * recStride, recStride, filterPattern, halfFilterLength, transposeIdx, clipTopRow[ii], clipBotRow[ii] );
            else 
              calcLinCovariance4<true,  true>( ELocal + ( ii << 2 ), rec + j + ii * recStride, recStride, filterPattern, halfFilterLength, transposeIdx, clipTopRow[ii], clipBotRow[ii] );
          }
        }
        else
        {
          for( int ii = 0; ii < 4; ii++ )
          {
            if( clipBotRow[ii] == 4 ) 
              calcLinCovariance4<false, false>( ELocal + ( ii << 2 ), rec + j + ii * recStride, recStride, filterPattern, halfFilterLength, transposeIdx, clipTopRow[ii], clipBotRow[ii] );
            else 
              calcLinCovariance4<true,  false>( ELocal + ( ii << 2 ), rec + j + ii * recStride, recStride, filterPattern, halfFilterLength, transposeIdx, clipTopRow[ii], clipBotRow[ii] );
          }
        }

        if( m_alfWSSD )
        {
          alf_float_t weight[4][4];
          for( int ii = 0; ii < 4; ii++ ) for( int jj = 0; jj < 4; jj++ )
          {
            weight[ii][jj] = m_lumaLevelToWeightPLUT[org[j + jj + ii * orgStride]];
          }

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_ALF
          if( useSimd )
          {
#if ALF_SINGLE_PREC_FLOAT
            for( int k = 0; k < shape.numCoeff; k++ )
            {
              const Pel* Elocalk  = &ELocal[k << 4];
              alf_float_t* cov     = &alfCovariance[classIdx].E[0][0][k][k];

              for( int l = k; l < shape.numCoeff; l++ )
              {
                const Pel* Elocall = &ELocal[l << 4];

                //double sum = 0.0;
                __m128 xacc = _mm_setzero_ps();

                for( int ii = 0; ii < 4; ii++ )
                {
                  const __m128i xloclx32 = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i* ) &Elocall[(ii << 2)] ) );
                  const __m128i xlockx32 = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i* ) &Elocalk[(ii << 2)] ) );
                  
                  __m128 xlocld = _mm_cvtepi32_ps( xloclx32 );
                  __m128 xlockd = _mm_cvtepi32_ps( xlockx32 );
                  __m128 xwght  = _mm_loadu_ps   ( &weight[ii][0] );
                  __m128 xprdct = _mm_mul_ps     ( xwght, _mm_mul_ps     ( xlocld, xlockd ) );
                  
                  xacc = _mm_add_ps( xacc, xprdct );

                  //for( int jj = 0; jj < 4; jj++ ) sum += weight[ii][jj] * Elocall[(ii << 2) + jj] * Elocalk[(ii << 2) + jj];
                }

                xacc = _mm_hadd_ps( xacc, xacc );
                xacc = _mm_hadd_ps( xacc, xacc );

                *cov++ += _mm_cvtss_f32( xacc );
                //*cov++ += sum;
              }

              //double sum = 0.0;
              __m128 xacc = _mm_setzero_ps();

              for( int ii = 0; ii < 4; ii++ )
              {
                const __m128i xlockx32 = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i* ) &Elocalk[(ii << 2)] ) );
                const __m128i yloc32   = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i* ) &yLocal [ii][0] ) );
                
                __m128 ylocd  = _mm_cvtepi32_ps( yloc32 );
                __m128 xlockd = _mm_cvtepi32_ps( xlockx32 );
                __m128 xwght  = _mm_loadu_ps   ( &weight[ii][0] );
                __m128 xprdct = _mm_mul_ps     ( xwght, _mm_mul_ps( ylocd, xlockd ) );
                
                xacc    = _mm_add_ps      ( xacc, xprdct );
                //for( int jj = 0; jj < 4; jj++ ) sum += weight[ii][jj] * Elocalk[(ii << 2) + jj] * yLocal[ii][jj];
              }

              xacc = _mm_hadd_ps( xacc, xacc );
              xacc = _mm_hadd_ps( xacc, xacc );

              alfCovariance[classIdx].y[0][k] += _mm_cvtss_f32( xacc );
              //alfCovariance[classIdx].y[0][k] += sum;
            }

            //double sum = 0.0;
            __m128 xacc = _mm_setzero_ps();

            for( int ii = 0; ii < 4; ii++ )
            {
              const __m128i yloc32   = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i* ) &yLocal[ii][0] ) );
              
              __m128 ylocd  = _mm_cvtepi32_ps( yloc32 );
              __m128 xwght  = _mm_loadu_ps   ( &weight[ii][0] );
              __m128 xprdct = _mm_mul_ps     ( xwght, _mm_mul_ps( ylocd, ylocd ) );
              
              xacc    = _mm_add_ps      ( xacc, xprdct );
              //for( int jj = 0; jj < 4; jj++ ) sum += weight[ii][jj] * yLocal[ii][jj] * yLocal[ii][jj];
            }

            xacc = _mm_hadd_ps( xacc, xacc );
            xacc = _mm_hadd_ps( xacc, xacc );

            alfCovariance[classIdx].pixAcc += _mm_cvtss_f32( xacc );
            //alfCovariance[classIdx].pixAcc += sum;
#else
            for( int k = 0; k < shape.numCoeff; k++ )
            {
              const Pel* Elocalk  = &ELocal[k << 4];
              alf_float_t* cov     = &alfCovariance[classIdx].E[0][0][k][k];

              for( int l = k; l < shape.numCoeff; l++ )
              {
                const Pel* Elocall = &ELocal[l << 4];

                //double sum = 0.0;
                __m128d xacc = _mm_setzero_pd();

                for( int ii = 0; ii < 4; ii++ )
                {
                  const __m128i xloclx32 = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i* ) &Elocall[(ii << 2)] ) );
                  const __m128i xlockx32 = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i* ) &Elocalk[(ii << 2)] ) );
                  
                  __m128d xlocld = _mm_cvtepi32_pd( xloclx32 );
                  __m128d xlockd = _mm_cvtepi32_pd( xlockx32 );
                  __m128d xwght  = _mm_loadu_pd   ( &weight[ii][0] );
                  __m128d xprdct = _mm_mul_pd     ( xwght, _mm_mul_pd     ( xlocld, xlockd ) );
                  
                  xacc = _mm_add_pd( xacc, xprdct );
                  
                  xlocld = _mm_cvtepi32_pd( _mm_unpackhi_epi64( xloclx32, xloclx32 ) );
                  xlockd = _mm_cvtepi32_pd( _mm_unpackhi_epi64( xlockx32, xlockx32 ) );
                  xwght  = _mm_loadu_pd   ( &weight[ii][2] );
                  xprdct = _mm_mul_pd     ( xwght, _mm_mul_pd     ( xlocld, xlockd ) );
                  
                  xacc = _mm_add_pd( xacc, xprdct );

                  //for( int jj = 0; jj < 4; jj++ ) sum += weight[ii][jj] * Elocall[(ii << 2) + jj] * Elocalk[(ii << 2) + jj];
                }

                xacc = _mm_hadd_pd( xacc, xacc );

                *cov++ += _mm_cvtsd_f64( xacc );
                //*cov++ += sum;
              }

              //double sum = 0.0;
              __m128d xacc = _mm_setzero_pd();

              for( int ii = 0; ii < 4; ii++ )
              {
                const __m128i xlockx32 = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i* ) &Elocalk[(ii << 2)] ) );
                const __m128i yloc32   = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i* ) &yLocal [ii][0] ) );
                
                __m128d ylocd  = _mm_cvtepi32_pd( yloc32 );
                __m128d xlockd = _mm_cvtepi32_pd( xlockx32 );
                __m128d xwght  = _mm_loadu_pd   ( &weight[ii][0] );
                __m128d xprdct = _mm_mul_pd     ( xwght, _mm_mul_pd( ylocd, xlockd ) );
                
                xacc    = _mm_add_pd      ( xacc, xprdct );
                
                ylocd   = _mm_cvtepi32_pd ( _mm_unpackhi_epi64( yloc32, yloc32 ) );
                xlockd  = _mm_cvtepi32_pd ( _mm_unpackhi_epi64( xlockx32, xlockx32 ) );
                xwght   = _mm_loadu_pd    ( &weight[ii][2] );
                xprdct  = _mm_mul_pd      ( xwght, _mm_mul_pd( ylocd, xlockd ) );
                
                xacc    = _mm_add_pd      ( xacc, xprdct );

                //for( int jj = 0; jj < 4; jj++ ) sum += weight[ii][jj] * Elocalk[(ii << 2) + jj] * yLocal[ii][jj];
              }

              xacc = _mm_hadd_pd( xacc, xacc );

              alfCovariance[classIdx].y[0][k] += _mm_cvtsd_f64( xacc );
              //alfCovariance[classIdx].y[0][k] += sum;
            }

            //double sum = 0.0;
            __m128d xacc = _mm_setzero_pd();

            for( int ii = 0; ii < 4; ii++ )
            {
              const __m128i yloc32   = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( const __m128i* ) &yLocal[ii][0] ) );
              
              __m128d ylocd  = _mm_cvtepi32_pd( yloc32 );
              __m128d xwght  = _mm_loadu_pd   ( &weight[ii][0] );
              __m128d xprdct = _mm_mul_pd     ( xwght, _mm_mul_pd( ylocd, ylocd ) );
              
              xacc    = _mm_add_pd      ( xacc, xprdct );
              
              ylocd   = _mm_cvtepi32_pd ( _mm_unpackhi_epi64( yloc32, yloc32 ) );
              xwght   = _mm_loadu_pd    ( &weight[ii][2] );
              xprdct  = _mm_mul_pd      ( xwght, _mm_mul_pd( ylocd, ylocd ) );
              
              xacc    = _mm_add_pd      ( xacc, xprdct );

              //for( int jj = 0; jj < 4; jj++ ) sum += weight[ii][jj] * yLocal[ii][jj] * yLocal[ii][jj];
            }

            xacc = _mm_hadd_pd( xacc, xacc );

            alfCovariance[classIdx].pixAcc += _mm_cvtsd_f64( xacc );
            //alfCovariance[classIdx].pixAcc += sum;
#endif
          }
          else
#endif
          {
            for( int k = 0; k < shape.numCoeff; k++ )
            {
              const Pel* Elocalk  = &ELocal[k << 4];
              alf_float_t* cov     = &alfCovariance[classIdx].E[0][0][k][k];

              for( int l = k; l < shape.numCoeff; l++ )
              {
                const Pel* Elocall = &ELocal[l << 4];

                alf_float_t sum = 0.0;
                for( int ii = 0; ii < 4; ii++ ) for( int jj = 0; jj < 4; jj++ )
                {
                  sum += weight[ii][jj] * Elocall[(ii << 2) + jj] * Elocalk[(ii << 2) + jj];
                }

                *cov++ += sum;
              }

              alf_float_t sum = 0.0;
              for( int ii = 0; ii < 4; ii++ ) for( int jj = 0; jj < 4; jj++ )
              {
                sum += weight[ii][jj] * Elocalk[(ii << 2) + jj] * yLocal[ii][jj];
              }

              alfCovariance[classIdx].y[0][k] += sum;
            }

            alf_float_t sum = 0.0;
            for( int ii = 0; ii < 4; ii++ ) for( int jj = 0; jj < 4; jj++ )
            {
              sum += weight[ii][jj] * yLocal[ii][jj] * yLocal[ii][jj];
            }

            alfCovariance[classIdx].pixAcc += sum;
          }
        }
        else
        {
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_ALF
          if( useSimd )
          {
#if ALF_SINGLE_PREC_FLOAT
            const __m128i mylocal0 = _mm_loadu_si128( ( const __m128i* ) &yLocal[0][0] );
            const __m128i mylocal8 = _mm_loadu_si128( ( const __m128i* ) &yLocal[2][0] );

            for( int k = 0; k < shape.numCoeff; k++ )
            {
              const Pel* Elocalk = &ELocal[k << 4];
              alf_float_t* cov = &alfCovariance[classIdx].E[0][0][k][k];

              const __m128i melocalk0 = _mm_loadu_si128( ( const __m128i* ) &Elocalk[0] );
              const __m128i melocalk8 = _mm_loadu_si128( ( const __m128i* ) &Elocalk[8] );

              int l = k;
              
              
              for( ; l < ( shape.numCoeff - 3 ); l += 4 )
              {
                __m128i vmacc[4];

                for( int ll = 0; ll < 4; ll++ )
                {
                  const Pel *Elocall = &ELocal[( l + ll ) << 4];

                  __m128i melocall0 = _mm_loadu_si128( ( const __m128i * ) &Elocall[0] );
                  __m128i melocall8 = _mm_loadu_si128( ( const __m128i * ) &Elocall[8] );

                  __m128i mmacc0 = _mm_madd_epi16( melocalk0, melocall0 );
                  __m128i mmacc8 = _mm_madd_epi16( melocalk8, melocall8 );

                  __m128i mmacc = _mm_add_epi32( mmacc0, mmacc8 );

                  vmacc[ll] = mmacc;
                }

                __m128i
                mmacc = _mm_hadd_epi32( _mm_hadd_epi32( vmacc[0], vmacc[1] ),
                                        _mm_hadd_epi32( vmacc[2], vmacc[3] ) );
                
                __m128 mmaccf = _mm_cvtepi32_ps( mmacc );

                __m128 mcov = _mm_loadu_ps( cov );
                mcov = _mm_add_ps( mcov, mmaccf );
                _mm_storeu_ps( cov, mcov );

                cov += 4;
              }
              
              for( ; l < shape.numCoeff; l++ )
              {
                const Pel* Elocall = &ELocal[l << 4];
                
                __m128i melocall0 = _mm_loadu_si128( ( const __m128i* ) &Elocall[0] );
                __m128i melocall8 = _mm_loadu_si128( ( const __m128i* ) &Elocall[8] );

                __m128i mmacc0 = _mm_madd_epi16( melocalk0, melocall0 );
                __m128i mmacc8 = _mm_madd_epi16( melocalk8, melocall8 );
                
                __m128i mmacc = _mm_add_epi32( mmacc0, mmacc8 );
                mmacc = _mm_hadd_epi32( mmacc, mmacc );
                mmacc = _mm_hadd_epi32( mmacc, mmacc );
                
                __m128 mmaccf = _mm_cvtepi32_ps( mmacc );

                *cov++ += _mm_cvtss_f32( mmaccf );
              }

              const __m128i mmacc0 = _mm_madd_epi16( melocalk0, mylocal0 );
              const __m128i mmacc8 = _mm_madd_epi16( melocalk8, mylocal8 );

              __m128i mmacc = _mm_add_epi32( mmacc0, mmacc8 );
              mmacc = _mm_hadd_epi32( mmacc, mmacc );
              mmacc = _mm_hadd_epi32( mmacc, mmacc );

              alfCovariance[classIdx].y[0][k] += _mm_cvtsi128_si32( mmacc );
            }

            const __m128i mmacc0 = _mm_madd_epi16( mylocal0, mylocal0 );
            const __m128i mmacc8 = _mm_madd_epi16( mylocal8, mylocal8 );

            __m128i mmacc = _mm_add_epi32( mmacc0, mmacc8 );
            mmacc = _mm_hadd_epi32( mmacc, mmacc );
            mmacc = _mm_hadd_epi32( mmacc, mmacc );

            alfCovariance[classIdx].pixAcc += _mm_cvtsi128_si32( mmacc );
#else
            const __m128i mylocal0 = _mm_loadu_si128( ( const __m128i* ) &yLocal[0][0] );
            const __m128i mylocal8 = _mm_loadu_si128( ( const __m128i* ) &yLocal[2][0] );

            for( int k = 0; k < shape.numCoeff; k++ )
            {
              const Pel* Elocalk = &ELocal[k << 4];
              alf_float_t* cov = &alfCovariance[classIdx].E[0][0][k][k];

              const __m128i melocalk0 = _mm_loadu_si128( ( const __m128i* ) &Elocalk[0] );
              const __m128i melocalk8 = _mm_loadu_si128( ( const __m128i* ) &Elocalk[8] );

              int l = k;
              
              for( ; l < ( shape.numCoeff - 1 ); l += 2 )
              {
                const Pel* Elocall = &ELocal[l << 4];
                
                __m128i melocall0 = _mm_loadu_si128( ( const __m128i* ) &Elocall[0] );
                __m128i melocall8 = _mm_loadu_si128( ( const __m128i* ) &Elocall[8] );

                __m128i mmacc0 = _mm_madd_epi16( melocalk0, melocall0 );
                __m128i mmacc8 = _mm_madd_epi16( melocalk8, melocall8 );
                
                __m128i mmacca = _mm_add_epi32( mmacc0, mmacc8 );

                Elocall = &ELocal[(l + 1) << 4];

                melocall0 = _mm_loadu_si128( ( const __m128i* ) &Elocall[0] );
                melocall8 = _mm_loadu_si128( ( const __m128i* ) &Elocall[8] );

                mmacc0 = _mm_madd_epi16( melocalk0, melocall0 );
                mmacc8 = _mm_madd_epi16( melocalk8, melocall8 );

                __m128i mmaccb = _mm_add_epi32( mmacc0, mmacc8 );

                __m128i mmacc = _mm_hadd_epi32( mmacca, mmaccb );
                mmacc = _mm_hadd_epi32( mmacc, mmacc );

                __m128d madd = _mm_cvtepi32_pd( mmacc );
                __m128d mcov = _mm_loadu_pd( cov );
                mcov = _mm_add_pd( mcov, madd );
                _mm_storeu_pd( cov, mcov );

                cov += 2;
              }

              if( l != shape.numCoeff )
              {
                const Pel* Elocall = &ELocal[l << 4];
                
                const __m128i melocall0 = _mm_loadu_si128( ( const __m128i* ) &Elocall[0] );
                const __m128i melocall8 = _mm_loadu_si128( ( const __m128i* ) &Elocall[8] );

                const __m128i mmacc0 = _mm_madd_epi16( melocalk0, melocall0 );
                const __m128i mmacc8 = _mm_madd_epi16( melocalk8, melocall8 );
                
                __m128i mmacc = _mm_add_epi32( mmacc0, mmacc8 );
                mmacc = _mm_hadd_epi32( mmacc, mmacc );
                mmacc = _mm_hadd_epi32( mmacc, mmacc );

                *cov += _mm_cvtsi128_si32( mmacc );
              }

              const __m128i mmacc0 = _mm_madd_epi16( melocalk0, mylocal0 );
              const __m128i mmacc8 = _mm_madd_epi16( melocalk8, mylocal8 );

              __m128i mmacc = _mm_add_epi32( mmacc0, mmacc8 );
              mmacc = _mm_hadd_epi32( mmacc, mmacc );
              mmacc = _mm_hadd_epi32( mmacc, mmacc );

              alfCovariance[classIdx].y[0][k] += _mm_cvtsi128_si32( mmacc );
            }

            const __m128i mmacc0 = _mm_madd_epi16( mylocal0, mylocal0 );
            const __m128i mmacc8 = _mm_madd_epi16( mylocal8, mylocal8 );

            __m128i mmacc = _mm_add_epi32( mmacc0, mmacc8 );
            mmacc = _mm_hadd_epi32( mmacc, mmacc );
            mmacc = _mm_hadd_epi32( mmacc, mmacc );

            alfCovariance[classIdx].pixAcc += _mm_cvtsi128_si32( mmacc );
#endif
          }
          else
#endif
          {
            for( int k = 0; k < shape.numCoeff; k++ )
            {
              const Pel* Elocalk = &ELocal[k << 4];
              alf_float_t* cov    = &alfCovariance[classIdx].E[0][0][k][k];

              for( int l = k; l < shape.numCoeff; l++ )
              {
                const Pel* Elocall = &ELocal[l << 4];

                int64_t sum = 0;
                for( int ii = 0; ii < 4; ii++ ) for( int jj = 0; jj < 4; jj++ )
                {
                  sum += int( Elocall[(ii << 2) + jj] ) * Elocalk[(ii << 2) + jj];
                }

                *cov++ += sum;
              }

              int64_t sum = 0;
              for( int ii = 0; ii < 4; ii++ ) for( int jj = 0; jj < 4; jj++ )
              {
                sum += int( Elocalk[(ii << 2) + jj] ) * yLocal[ii][jj];
              }

              alfCovariance[classIdx].y[0][k] += sum;
            }

            int64_t sum = 0;
            for( int ii = 0; ii < 4; ii++ ) for( int jj = 0; jj < 4; jj++ )
            {
              sum += int( yLocal[ii][jj] ) * yLocal[ii][jj];
            }

            alfCovariance[classIdx].pixAcc += sum;
          }
        }

        alfCovariance[classIdx].all0 = false;
      }

      org += ( orgStride << 2 );
      rec += ( recStride << 2 );
    }
  }
  else
  {
    for( int i = 0; i < area.height; i++ )
    {
      const int maxFilterSamples = 4;

      int clipTopRow = -maxFilterSamples;
      int clipBotRow =  maxFilterSamples;

      int vbDistance = ((areaDst.y + i) % vbCTUHeight) - vbPos;
      if( vbDistance >= -3 && vbDistance < 0 )
      {
        clipBotRow = -vbDistance - 1;
        clipTopRow = -clipBotRow; // symmetric
      }
      else if( vbDistance >= 0 && vbDistance < 3 )
      {
        clipTopRow = -vbDistance;
        clipBotRow = -clipTopRow; // symmetric
      }

      for( int j = 0; j < area.width; j++ )
      {
        const int idx = ( i / 4 ) * ( MAX_CU_SIZE / 4 ) + j / 4;
        if( classifier && classifier[idx].classIdx == m_ALF_UNUSED_CLASSIDX && classifier[idx].transposeIdx == m_ALF_UNUSED_TRANSPOSIDX )
        {
          continue;
        }
        if( classifier )
        {
          AlfClassifier& cl = classifier[idx];
          transposeIdx = cl.transposeIdx;
          classIdx = cl.classIdx;
        }

        int yLocal = org[j] - rec[j];

        int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues];
        //      std::memset( ELocal, 0, sizeof( ELocal ) );
        calcCovariance( ELocal, rec + j, recStride, shape, transposeIdx, channel, vbDistance );

        if( m_alfWSSD )
        {
          double weight = m_lumaLevelToWeightPLUT[org[j]];

          for( int k = 0; k < shape.numCoeff; k++ )
          {
            for( int l = k; l < shape.numCoeff; l++ )
            {
              int Elocalkb0 = ELocal[k][0];
              int Elocalkb1 = ELocal[k][1];
              int Elocalkb2 = ELocal[k][2];
              int Elocalkb3 = ELocal[k][3];

              int Elocallb0 = ELocal[l][0];
              int Elocallb1 = ELocal[l][1];
              int Elocallb2 = ELocal[l][2];
              int Elocallb3 = ELocal[l][3];

              alfCovariance[classIdx].E[0][0][k][l] += weight * (Elocalkb0 * ( double ) Elocallb0);
              alfCovariance[classIdx].E[0][1][k][l] += weight * (Elocalkb0 * ( double ) Elocallb1);
              alfCovariance[classIdx].E[0][2][k][l] += weight * (Elocalkb0 * ( double ) Elocallb2);
              alfCovariance[classIdx].E[0][3][k][l] += weight * (Elocalkb0 * ( double ) Elocallb3);

              alfCovariance[classIdx].E[1][0][k][l] += weight * (Elocalkb1 * ( double ) Elocallb0);
              alfCovariance[classIdx].E[1][1][k][l] += weight * (Elocalkb1 * ( double ) Elocallb1);
              alfCovariance[classIdx].E[1][2][k][l] += weight * (Elocalkb1 * ( double ) Elocallb2);
              alfCovariance[classIdx].E[1][3][k][l] += weight * (Elocalkb1 * ( double ) Elocallb3);

              alfCovariance[classIdx].E[2][0][k][l] += weight * (Elocalkb2 * ( double ) Elocallb0);
              alfCovariance[classIdx].E[2][1][k][l] += weight * (Elocalkb2 * ( double ) Elocallb1);
              alfCovariance[classIdx].E[2][2][k][l] += weight * (Elocalkb2 * ( double ) Elocallb2);
              alfCovariance[classIdx].E[2][3][k][l] += weight * (Elocalkb2 * ( double ) Elocallb3);

              alfCovariance[classIdx].E[3][0][k][l] += weight * (Elocalkb3 * ( double ) Elocallb0);
              alfCovariance[classIdx].E[3][1][k][l] += weight * (Elocalkb3 * ( double ) Elocallb1);
              alfCovariance[classIdx].E[3][2][k][l] += weight * (Elocalkb3 * ( double ) Elocallb2);
              alfCovariance[classIdx].E[3][3][k][l] += weight * (Elocalkb3 * ( double ) Elocallb3);
            }

            for( int b = 0; b < numBins; b++ )
            {
              alfCovariance[classIdx].y[b][k] += weight * (ELocal[k][b] * ( double ) yLocal);
            }
          }

          alfCovariance[classIdx].pixAcc += weight * (yLocal * ( double ) yLocal);
        }
        else
        {
          for( int k = 0; k < shape.numCoeff; k++ )
          {
            for( int l = k; l < shape.numCoeff; l++ )
            {
              int Elocalkb0 = ELocal[k][0];
              int Elocalkb1 = ELocal[k][1];
              int Elocalkb2 = ELocal[k][2];
              int Elocalkb3 = ELocal[k][3];

              int Elocallb0 = ELocal[l][0];
              int Elocallb1 = ELocal[l][1];
              int Elocallb2 = ELocal[l][2];
              int Elocallb3 = ELocal[l][3];

              alfCovariance[classIdx].E[0][0][k][l] += Elocalkb0 * Elocallb0;
              alfCovariance[classIdx].E[0][1][k][l] += Elocalkb0 * Elocallb1;
              alfCovariance[classIdx].E[0][2][k][l] += Elocalkb0 * Elocallb2;
              alfCovariance[classIdx].E[0][3][k][l] += Elocalkb0 * Elocallb3;

              alfCovariance[classIdx].E[1][0][k][l] += Elocalkb1 * Elocallb0;
              alfCovariance[classIdx].E[1][1][k][l] += Elocalkb1 * Elocallb1;
              alfCovariance[classIdx].E[1][2][k][l] += Elocalkb1 * Elocallb2;
              alfCovariance[classIdx].E[1][3][k][l] += Elocalkb1 * Elocallb3;

              alfCovariance[classIdx].E[2][0][k][l] += Elocalkb2 * Elocallb0;
              alfCovariance[classIdx].E[2][1][k][l] += Elocalkb2 * Elocallb1;
              alfCovariance[classIdx].E[2][2][k][l] += Elocalkb2 * Elocallb2;
              alfCovariance[classIdx].E[2][3][k][l] += Elocalkb2 * Elocallb3;

              alfCovariance[classIdx].E[3][0][k][l] += Elocalkb3 * Elocallb0;
              alfCovariance[classIdx].E[3][1][k][l] += Elocalkb3 * Elocallb1;
              alfCovariance[classIdx].E[3][2][k][l] += Elocalkb3 * Elocallb2;
              alfCovariance[classIdx].E[3][3][k][l] += Elocalkb3 * Elocallb3;
            }
            for( int b = 0; b < numBins; b++ )
            {
              alfCovariance[classIdx].y[b][k] += ELocal[k][b] * yLocal;
            }
          }

          alfCovariance[classIdx].pixAcc += yLocal * yLocal;
        }

        alfCovariance[classIdx].all0 = false;
      }

      org += orgStride;
      rec += recStride;
    }
  }

  int numClasses = classifier ? MAX_NUM_ALF_CLASSES : 1;

  if( !m_encCfg->m_useNonLinearAlfLuma && !m_encCfg->m_useNonLinearAlfChroma )
  {
    for( classIdx = 0; classIdx < numClasses; classIdx++ )
    {
      for( int k = 1; k < shape.numCoeff; k++ )
      {
        for( int l = 0; l < k; l++ )
        {
#if FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED && defined( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
#endif
          alfCovariance[classIdx].E[0][0][k][l] = alfCovariance[classIdx].E[0][0][l][k];
#if FIX_FOR_TEMPORARY_COMPILER_ISSUES_ENABLED && defined( __GNUC__ )
#pragma GCC diagnostic pop
#endif
        }
      }
    }
  }
  else
  {
    for( classIdx = 0; classIdx < numClasses; classIdx++ )
    {
      for( int k = 1; k < shape.numCoeff; k++ )
      {
        for( int l = 0; l < k; l++ )
        {
          for( int b0 = 0; b0 < numBins; b0++ )
          {
            for( int b1 = 0; b1 < numBins; b1++ )
            {
              alfCovariance[classIdx].E[b0][b1][k][l] = alfCovariance[classIdx].E[b1][b0][l][k];
            }
          }
        }
      }
    }
  }
}

template < bool clipToBdry >
inline int clipIdx( int i, int clip )
{
  if( clipToBdry )
    return std::max( i, clip );
  else
    return i;
}

template <bool clipToBdry>
void EncAdaptiveLoopFilter::calcLinCovariance( int* ELocal, const Pel *rec, const int stride, const int* filterPattern, const int halfFilterLength, const int transposeIdx, int clipTopRow, int clipBotRow )
{
  int k = 0;

  const short curr = rec[0];

  if( transposeIdx == 0 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      const Pel* rec0 = rec + clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
      const Pel* rec1 = rec - clipIdx<clipToBdry>( i, -clipBotRow ) * stride;
      for( int j = -halfFilterLength - i; j <= halfFilterLength + i; j++, k++ )
      {
        const int val0 = rec0[j] - curr;
        const int val1 = rec1[-j] - curr;

        ELocal[filterPattern[k]] = val0 + val1;
      }
    }
    for( int j = -halfFilterLength; j < 0; j++, k++ )
    {
      const int val0 = rec[j] - curr;
      const int val1 = rec[-j] - curr;

      ELocal[filterPattern[k]] = val0 + val1;
    }
  }
  else if( transposeIdx == 1 )
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;
      for( int i = -halfFilterLength - j; i <= halfFilterLength + j; i++, k++ )
      {
        const int val0 = rec0[ clipIdx<clipToBdry>( i,  clipTopRow ) * stride] - curr;
        const int val1 = rec1[-clipIdx<clipToBdry>( i, -clipBotRow ) * stride] - curr;

        ELocal[filterPattern[k]] = val0 + val1;
      }
    }
    for( int i = -halfFilterLength; i < 0; i++, k++ )
    {
      const int val0 = rec[ clipIdx<clipToBdry>( i,  clipTopRow ) * stride] - curr;
      const int val1 = rec[-clipIdx<clipToBdry>( i, -clipBotRow ) * stride] - curr;

      ELocal[filterPattern[k]] = val0 + val1;
    }
  }
  else if( transposeIdx == 2 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      const Pel* rec0 = rec + clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
      const Pel* rec1 = rec - clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

      for( int j = halfFilterLength + i; j >= -halfFilterLength - i; j--, k++ )
      {
        const int val0 = rec0[j] - curr;
        const int val1 = rec1[-j] - curr;

        ELocal[filterPattern[k]] = val0 + val1;
      }
    }
    for( int j = -halfFilterLength; j < 0; j++, k++ )
    {
      const int val0 = rec[j] - curr;
      const int val1 = rec[-j] - curr;

      ELocal[filterPattern[k]] = val0 + val1;
    }
  }
  else
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;
      for( int i = halfFilterLength + j; i >= -halfFilterLength - j; i--, k++ )
      {
        const int val0 = rec0[ clipIdx<clipToBdry>( i,  clipTopRow ) * stride] - curr;
        const int val1 = rec1[-clipIdx<clipToBdry>( i, -clipBotRow ) * stride] - curr;

        ELocal[filterPattern[k]] = val0 + val1;
      }
    }
    for( int i = -halfFilterLength; i < 0; i++, k++ )
    {
      const int val0 = rec[ clipIdx<clipToBdry>( i,  clipTopRow ) * stride] - curr;
      const int val1 = rec[-clipIdx<clipToBdry>( i, -clipBotRow ) * stride] - curr;

      ELocal[filterPattern[k]] = val0 + val1;
    }
  }

  ELocal[filterPattern[k]] = curr;
}

template void EncAdaptiveLoopFilter::calcLinCovariance<true >( int* ELocal, const Pel* rec, const int stride, const int* filterPattern, const int halfFilterLength, const int transposeIdx, int clipTopRow, int clipBotRow );
template void EncAdaptiveLoopFilter::calcLinCovariance<false>( int* ELocal, const Pel* rec, const int stride, const int* filterPattern, const int halfFilterLength, const int transposeIdx, int clipTopRow, int clipBotRow );

template<bool clipToBdry, bool simd>
void EncAdaptiveLoopFilter::calcLinCovariance4( Pel* ELocal, const Pel *rec, const int stride, const int* filterPattern, const int halfFilterLength, const int transposeIdx, int clipTopRow, int clipBotRow )
{
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_ALF
  if( simd )
  {
    int k = 0;
  
    const Pel* rec0 = &rec[0];

    __m128i xrec = _mm_loadl_epi64( ( const __m128i* ) rec );
    __m128i xcur = _mm_slli_epi16( xrec, 1 );

    if( transposeIdx == 0 )
    {
      for( int i = -halfFilterLength; i < 0; i++ )
      {
        const int off0 =  clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
        const int off1 = -clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

        const Pel* rec00 = rec0 + off0;
        const Pel* rec01 = rec0 + off1;

        for( int j = -halfFilterLength - i; j <= halfFilterLength + i; j++, k++ )
        {
          __m128i xval0 = _mm_loadl_epi64( ( const __m128i* ) &rec00[ j] );
          __m128i xval1 = _mm_loadl_epi64( ( const __m128i* ) &rec01[-j] );

          xval0 = _mm_add_epi16( xval0, xval1 );
          xval0 = _mm_sub_epi16( xval0, xcur );

          _mm_storel_epi64( ( __m128i* ) &ELocal[filterPattern[k]], xval0 );
        }
      }

      for( int j = -halfFilterLength; j < 0; j++, k++ )
      {
        __m128i xval0 = _mm_loadl_epi64( ( const __m128i* ) &rec[ j] );
        __m128i xval1 = _mm_loadl_epi64( ( const __m128i* ) &rec[-j] );

        xval0 = _mm_add_epi16( xval0, xval1 );
        xval0 = _mm_sub_epi16( xval0, xcur );

        _mm_storel_epi64( ( __m128i* ) & ELocal[filterPattern[k]], xval0 );
      }
    }
    else if( transposeIdx == 1 )
    {
      for( int j = -halfFilterLength; j < 0; j++ )
      {
        const Pel* rec00 = rec0 + j;
        const Pel* rec01 = rec0 - j;

        for( int i = -halfFilterLength - j; i <= halfFilterLength + j; i++, k++ )
        {
          const int off0 =  clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
          const int off1 = -clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

          const Pel *vptr0 = &rec00[off0];
          const Pel *vptr1 = &rec01[off1];

          __m128i xval0 = _mm_loadl_epi64( ( const __m128i* ) vptr0 );
          __m128i xval1 = _mm_loadl_epi64( ( const __m128i* ) vptr1 );

          xval0 = _mm_add_epi16( xval0, xval1 );
          xval0 = _mm_sub_epi16( xval0, xcur );

          _mm_storel_epi64( ( __m128i* ) &ELocal[filterPattern[k]], xval0 );
        }
      }
      for( int i = -halfFilterLength; i < 0; i++, k++ )
      {
        const int off0 =  clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
        const int off1 = -clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

        const Pel *vptr0 = &rec0[off0];
        const Pel *vptr1 = &rec0[off1];

        __m128i xval0 = _mm_loadl_epi64( ( const __m128i* ) vptr0 );
        __m128i xval1 = _mm_loadl_epi64( ( const __m128i* ) vptr1 );

        xval0 = _mm_add_epi16( xval0, xval1 );
        xval0 = _mm_sub_epi16( xval0, xcur );

        _mm_storel_epi64( ( __m128i* ) &ELocal[filterPattern[k]], xval0 );
      }
    }
    else if( transposeIdx == 2 )
    {
      for( int i = -halfFilterLength; i < 0; i++ )
      {
        const int off0 =  clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
        const int off1 = -clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

        const Pel* rec00 = rec0 + off0;
        const Pel* rec01 = rec0 + off1;

        for( int j = halfFilterLength + i; j >= -halfFilterLength - i; j--, k++ )
        {
          __m128i xval0 = _mm_loadl_epi64( ( const __m128i* ) &rec00[ j] );
          __m128i xval1 = _mm_loadl_epi64( ( const __m128i* ) &rec01[-j] );

          xval0 = _mm_add_epi16( xval0, xval1 );
          xval0 = _mm_sub_epi16( xval0, xcur );

          _mm_storel_epi64( ( __m128i* ) &ELocal[filterPattern[k]], xval0 );
        }
      }
      for( int j = -halfFilterLength; j < 0; j++, k++ )
      {
        __m128i xval0 = _mm_loadl_epi64( ( const __m128i* ) &rec0[ j] );
        __m128i xval1 = _mm_loadl_epi64( ( const __m128i* ) &rec0[-j] );

        xval0 = _mm_add_epi16( xval0, xval1 );
        xval0 = _mm_sub_epi16( xval0, xcur );

        _mm_storel_epi64( ( __m128i* ) &ELocal[filterPattern[k]], xval0 );
      }
    }
    else
    {
      for( int j = -halfFilterLength; j < 0; j++ )
      {
        const Pel* rec00 = rec0 + j;
        const Pel* rec01 = rec0 - j;

        for( int i = halfFilterLength + j; i >= -halfFilterLength - j; i--, k++ )
        {
          const int off0 =  clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
          const int off1 = -clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

          const Pel *vptr0 = &rec00[off0];
          const Pel *vptr1 = &rec01[off1];
          
          __m128i xval0 = _mm_loadl_epi64( ( const __m128i* ) vptr0 );
          __m128i xval1 = _mm_loadl_epi64( ( const __m128i* ) vptr1 );

          xval0 = _mm_add_epi16( xval0, xval1 );
          xval0 = _mm_sub_epi16( xval0, xcur );

          _mm_storel_epi64( ( __m128i* ) &ELocal[filterPattern[k]], xval0 );
        }
      }
      for( int i = -halfFilterLength; i < 0; i++, k++ )
      {
        const int off0 =  clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
        const int off1 = -clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

        const Pel *vptr0 = &rec0[off0];
        const Pel *vptr1 = &rec0[off1];
        
        __m128i xval0 = _mm_loadl_epi64( ( const __m128i* ) vptr0 );
        __m128i xval1 = _mm_loadl_epi64( ( const __m128i* ) vptr1 );

        xval0 = _mm_add_epi16( xval0, xval1 );
        xval0 = _mm_sub_epi16( xval0, xcur );

        _mm_storel_epi64( ( __m128i* ) &ELocal[filterPattern[k]], xval0 );
      }
    }

    _mm_storel_epi64( ( __m128i* ) &ELocal[filterPattern[k]], xrec );
  }
  else
#endif
  {
    int k = 0;
  
    const Pel* rec0 = &rec[0];
    const Pel* rec1 = &rec[1];
    const Pel* rec2 = &rec[2];
    const Pel* rec3 = &rec[3];
  
    const Pel curr0 = *rec0 << 1;
    const Pel curr1 = *rec1 << 1;
    const Pel curr2 = *rec2 << 1;
    const Pel curr3 = *rec3 << 1;

    if( transposeIdx == 0 )
    {
      for( int i = -halfFilterLength; i < 0; i++ )
      {
        const int off0 =  clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
        const int off1 = -clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

        const Pel* rec00 = rec0 + off0;
        const Pel* rec01 = rec0 + off1;

        const Pel* rec10 = rec1 + off0;
        const Pel* rec11 = rec1 + off1;

        const Pel* rec20 = rec2 + off0;
        const Pel* rec21 = rec2 + off1;

        const Pel* rec30 = rec3 + off0;
        const Pel* rec31 = rec3 + off1;

        for( int j = -halfFilterLength - i; j <= halfFilterLength + i; j++, k++ )
        {
          const Pel val00 = rec00[ j];
          const Pel val01 = rec01[-j];
        
          const Pel val10 = rec10[ j];
          const Pel val11 = rec11[-j];
        
          const Pel val20 = rec20[ j];
          const Pel val21 = rec21[-j];

          const Pel val30 = rec30[ j];
          const Pel val31 = rec31[-j];

          ELocal[filterPattern[k] + 0] = val00 + val01 - curr0;
          ELocal[filterPattern[k] + 1] = val10 + val11 - curr1;
          ELocal[filterPattern[k] + 2] = val20 + val21 - curr2;
          ELocal[filterPattern[k] + 3] = val30 + val31 - curr3;
        }
      }

      for( int j = -halfFilterLength; j < 0; j++, k++ )
      {
        const Pel val00 = rec0[ j];
        const Pel val01 = rec0[-j];
      
        const Pel val10 = rec1[ j];
        const Pel val11 = rec1[-j];
      
        const Pel val20 = rec2[ j];
        const Pel val21 = rec2[-j];
      
        const Pel val30 = rec3[ j];
        const Pel val31 = rec3[-j];

        ELocal[filterPattern[k] + 0] = val00 + val01 - curr0;
        ELocal[filterPattern[k] + 1] = val10 + val11 - curr1;
        ELocal[filterPattern[k] + 2] = val20 + val21 - curr2;
        ELocal[filterPattern[k] + 3] = val30 + val31 - curr3;
      }
    }
    else if( transposeIdx == 1 )
    {
      for( int j = -halfFilterLength; j < 0; j++ )
      {
        const Pel* rec00 = rec0 + j;
        const Pel* rec01 = rec0 - j;

        for( int i = -halfFilterLength - j; i <= halfFilterLength + j; i++, k++ )
        {
          const int off0 =  clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
          const int off1 = -clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

          const Pel *vptr0 = &rec00[off0];
          const Pel *vptr1 = &rec01[off1];
          const Pel* vptr2 =  rec0;

          ELocal[filterPattern[k] + 0] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
          ELocal[filterPattern[k] + 1] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
          ELocal[filterPattern[k] + 2] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
          ELocal[filterPattern[k] + 3] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
        }
      }
      for( int i = -halfFilterLength; i < 0; i++, k++ )
      {
        const int off0 =  clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
        const int off1 = -clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

        const Pel *vptr0 = &rec0[off0];
        const Pel *vptr1 = &rec0[off1];
        const Pel* vptr2 =  rec0;

        ELocal[filterPattern[k] + 0] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
        ELocal[filterPattern[k] + 1] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
        ELocal[filterPattern[k] + 2] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
        ELocal[filterPattern[k] + 3] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
      }
    }
    else if( transposeIdx == 2 )
    {
      for( int i = -halfFilterLength; i < 0; i++ )
      {
        const int off0 =  clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
        const int off1 = -clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

        const Pel* rec00 = rec0 + off0;
        const Pel* rec01 = rec0 + off1;

        const Pel* rec10 = rec1 + off0;
        const Pel* rec11 = rec1 + off1;

        const Pel* rec20 = rec2 + off0;
        const Pel* rec21 = rec2 + off1;

        const Pel* rec30 = rec3 + off0;
        const Pel* rec31 = rec3 + off1;

        for( int j = halfFilterLength + i; j >= -halfFilterLength - i; j--, k++ )
        {
          const Pel val00 = rec00[ j];
          const Pel val01 = rec01[-j];
        
          const Pel val10 = rec10[ j];
          const Pel val11 = rec11[-j];
        
          const Pel val20 = rec20[ j];
          const Pel val21 = rec21[-j];
        
          const Pel val30 = rec30[ j];
          const Pel val31 = rec31[-j];

          ELocal[filterPattern[k] + 0] = val00 + val01 - curr0;
          ELocal[filterPattern[k] + 1] = val10 + val11 - curr1;
          ELocal[filterPattern[k] + 2] = val20 + val21 - curr2;
          ELocal[filterPattern[k] + 3] = val30 + val31 - curr3;
        }
      }
      for( int j = -halfFilterLength; j < 0; j++, k++ )
      {
        const Pel val00 = rec0[ j];
        const Pel val01 = rec0[-j];

        const Pel val10 = rec1[ j];
        const Pel val11 = rec1[-j];

        const Pel val20 = rec2[ j];
        const Pel val21 = rec2[-j];

        const Pel val30 = rec3[ j];
        const Pel val31 = rec3[-j];

        ELocal[filterPattern[k] + 0] = val00 + val01 - curr0;
        ELocal[filterPattern[k] + 1] = val10 + val11 - curr1;
        ELocal[filterPattern[k] + 2] = val20 + val21 - curr2;
        ELocal[filterPattern[k] + 3] = val30 + val31 - curr3;
      }
    }
    else
    {
      for( int j = -halfFilterLength; j < 0; j++ )
      {
        const Pel* rec00 = rec0 + j;
        const Pel* rec01 = rec0 - j;

        for( int i = halfFilterLength + j; i >= -halfFilterLength - j; i--, k++ )
        {
          const int off0 =  clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
          const int off1 = -clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

          const Pel *vptr0 = &rec00[off0];
          const Pel *vptr1 = &rec01[off1];
          const Pel* vptr2 =  rec0;

          ELocal[filterPattern[k] + 0] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
          ELocal[filterPattern[k] + 1] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
          ELocal[filterPattern[k] + 2] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
          ELocal[filterPattern[k] + 3] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
        }
      }
      for( int i = -halfFilterLength; i < 0; i++, k++ )
      {
        const int off0 =  clipIdx<clipToBdry>( i,  clipTopRow ) * stride;
        const int off1 = -clipIdx<clipToBdry>( i, -clipBotRow ) * stride;

        const Pel *vptr0 = &rec0[off0];
        const Pel *vptr1 = &rec0[off1];
        const Pel* vptr2 =  rec0;

        ELocal[filterPattern[k] + 0] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
        ELocal[filterPattern[k] + 1] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
        ELocal[filterPattern[k] + 2] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
        ELocal[filterPattern[k] + 3] = *vptr0++ + *vptr1++ - ( *vptr2++ << 1 );
      }
    }

    ELocal[filterPattern[k] + 0] = *rec0;
    ELocal[filterPattern[k] + 1] = *rec1;
    ELocal[filterPattern[k] + 2] = *rec2;
    ELocal[filterPattern[k] + 3] = *rec3;
  }
}

template void EncAdaptiveLoopFilter::calcLinCovariance4<true,  false>( Pel* ELocal, const Pel* rec, const int stride, const int* filterPattern, const int halfFilterLength, const int transposeIdx, int clipTopRow, int clipBotRow );
template void EncAdaptiveLoopFilter::calcLinCovariance4<false, false>( Pel* ELocal, const Pel* rec, const int stride, const int* filterPattern, const int halfFilterLength, const int transposeIdx, int clipTopRow, int clipBotRow );
template void EncAdaptiveLoopFilter::calcLinCovariance4<true,  true> ( Pel* ELocal, const Pel* rec, const int stride, const int* filterPattern, const int halfFilterLength, const int transposeIdx, int clipTopRow, int clipBotRow );
template void EncAdaptiveLoopFilter::calcLinCovariance4<false, true> ( Pel* ELocal, const Pel* rec, const int stride, const int* filterPattern, const int halfFilterLength, const int transposeIdx, int clipTopRow, int clipBotRow );

void EncAdaptiveLoopFilter::calcCovariance( int ELocal[MAX_NUM_ALF_LUMA_COEFF][MaxAlfNumClippingValues], const Pel *rec, const int stride, const AlfFilterShape& shape, const int transposeIdx, const ChannelType channel, int vbDistance )
{
  int clipTopRow = -4;
  int clipBotRow = 4;
  if (vbDistance >= -3 && vbDistance < 0)
  {
    clipBotRow = -vbDistance - 1;
    clipTopRow = -clipBotRow; // symmetric
  }
  else if (vbDistance >= 0 && vbDistance < 3)
  {
    clipTopRow = -vbDistance;
    clipBotRow = -clipTopRow; // symmetric
  }

  const int *filterPattern   = shape.pattern.data();
  const int halfFilterLength = shape.filterLength >> 1;
  const Pel* clip            = m_alfClippingValues[channel];
  constexpr int numBins      = AlfNumClippingValues;

  int k = 0;

  const short curr = rec[0];

  if( transposeIdx == 0 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      const Pel* rec0 = rec + std::max(i, clipTopRow) * stride;
      const Pel* rec1 = rec - std::max(i, -clipBotRow) * stride;
      for( int j = -halfFilterLength - i; j <= halfFilterLength + i; j++, k++ )
      {
        const int val0 = rec0[ j] - curr;
        const int val1 = rec1[-j] - curr;
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] = clipALF( clip[b], val0, val1 );
        }
      }
    }
    for( int j = -halfFilterLength; j < 0; j++, k++ )
    {
      const int val0 = rec[ j] - curr;
      const int val1 = rec[-j] - curr;
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] = clipALF( clip[b], val0, val1 );
      }
    }
  }
  else if( transposeIdx == 1 )
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;
      for (int i = -halfFilterLength - j; i <= halfFilterLength + j; i++, k++)
      {
        const int val0 = rec0[std::max(i, clipTopRow) * stride] - curr;
        const int val1 = rec1[-std::max(i, -clipBotRow) * stride] - curr;
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] = clipALF(clip[b], val0, val1);
        }
      }
    }
    for (int i = -halfFilterLength; i < 0; i++, k++)
    {
      const int val0 = rec[std::max(i, clipTopRow) * stride] - curr;
      const int val1 = rec[-std::max(i, -clipBotRow) * stride] - curr;
      for (int b = 0; b < numBins; b++)
      {
        ELocal[filterPattern[k]][b] = clipALF(clip[b], val0 , val1);
      }
    }
  }
  else if( transposeIdx == 2 )
  {
    for( int i = -halfFilterLength; i < 0; i++ )
    {
      const Pel* rec0 = rec + std::max(i, clipTopRow) * stride;
      const Pel* rec1 = rec - std::max(i, -clipBotRow) * stride;

      for( int j = halfFilterLength + i; j >= -halfFilterLength - i; j--, k++ )
      {
        const int val0 = rec0[j] - curr;
        const int val1 = rec1[-j] - curr;
        for( int b = 0; b < numBins; b++ )
        {
          ELocal[filterPattern[k]][b] = clipALF( clip[b], val0, val1 );
        }
      }
    }
    for( int j = -halfFilterLength; j < 0; j++, k++ )
    {
      const int val0 = rec[j] - curr;
      const int val1 = rec[-j] - curr;
      for( int b = 0; b < numBins; b++ )
      {
        ELocal[filterPattern[k]][b] = clipALF( clip[b], val0, val1 );
      }
    }
  }
  else
  {
    for( int j = -halfFilterLength; j < 0; j++ )
    {
      const Pel* rec0 = rec + j;
      const Pel* rec1 = rec - j;
      for (int i = halfFilterLength + j; i >= -halfFilterLength - j; i--, k++)
      {
        const int val0 = rec0[std::max(i, clipTopRow) * stride] - curr;
        const int val1 = rec1[-std::max(i, -clipBotRow) * stride] - curr;
        for (int b = 0; b < numBins; b++)
        {
          ELocal[filterPattern[k]][b] = clipALF(clip[b], val0 ,val1 );
        }
      }
    }
    for (int i = -halfFilterLength; i < 0; i++, k++)
    {
      const int val0 = rec[std::max(i, clipTopRow) * stride] - curr;
      const int val1 = rec[-std::max(i, -clipBotRow) * stride] - curr;
      for (int b = 0; b < numBins; b++)
      {
        ELocal[filterPattern[k]][b] = clipALF(clip[b], val0 ,val1 );
      }
    }
  }
  for( int b = 0; b < numBins; b++ )
  {
    ELocal[filterPattern[k]][b] = curr;
  }
}

void EncAdaptiveLoopFilter::setEnableFlag( AlfParam& alfSlicePara, ChannelType channel, bool val )
{
  if( channel == CH_L )
  {
    alfSlicePara.alfEnabled[COMP_Y] = val;
  }
  else
  {
    alfSlicePara.alfEnabled[COMP_Cb] = alfSlicePara.alfEnabled[COMP_Cr] = val;
  }
}


void EncAdaptiveLoopFilter::setEnableFlag( AlfParam& alfSlicePara, ChannelType channel, uint8_t** ctuFlags )
{
  const ComponentID compIDFirst = isLuma( channel ) ? COMP_Y : COMP_Cb;
  const ComponentID compIDLast = isLuma( channel ) ? COMP_Y : COMP_Cr;
  for( int compId = compIDFirst; compId <= compIDLast; compId++ )
  {
    alfSlicePara.alfEnabled[compId] = false;
    for( int i = 0; i < m_numCTUsInPic; i++ )
    {
      if( ctuFlags[compId][i] )
      {
        alfSlicePara.alfEnabled[compId] = true;
        break;
      }
    }
  }
}

void EncAdaptiveLoopFilter::copyCtuEnableFlag( uint8_t** ctuFlagsDst, uint8_t** ctuFlagsSrc, ChannelType channel )
{
  if( isLuma( channel ) )
  {
    memcpy( ctuFlagsDst[COMP_Y], ctuFlagsSrc[COMP_Y], sizeof( uint8_t ) * m_numCTUsInPic );
  }
  else
  {
    memcpy( ctuFlagsDst[COMP_Cb], ctuFlagsSrc[COMP_Cb], sizeof( uint8_t ) * m_numCTUsInPic );
    memcpy( ctuFlagsDst[COMP_Cr], ctuFlagsSrc[COMP_Cr], sizeof( uint8_t ) * m_numCTUsInPic );
  }
}

void EncAdaptiveLoopFilter::setCtuEnableFlag( uint8_t** ctuFlags, ChannelType channel, uint8_t val )
{
  if( isLuma( channel ) )
  {
    memset( ctuFlags[COMP_Y], val, sizeof( uint8_t ) * m_numCTUsInPic );
  }
  else
  {
    memset( ctuFlags[COMP_Cb], val, sizeof( uint8_t ) * m_numCTUsInPic );
    memset( ctuFlags[COMP_Cr], val, sizeof( uint8_t ) * m_numCTUsInPic );
  }
}

std::vector<int> EncAdaptiveLoopFilter::getAvaiApsIdsLuma(CodingStructure& cs, int& newApsId)
{
  APS** apss = cs.slice->alfAps;
  for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
  {
    apss[i] = m_apsMap->getPS((i << NUM_APS_TYPE_LEN) + ALF_APS);
  }

  std::vector<int> result;
  if( m_encCfg->m_alfTempPred )
  {
    int apsIdChecked = 0, curApsId = m_apsIdStart;
    if (curApsId < ALF_CTB_MAX_NUM_APS)
    {
      while (apsIdChecked < ALF_CTB_MAX_NUM_APS && !cs.slice->isIntra() && result.size() < ALF_CTB_MAX_NUM_APS && !cs.slice->pendingRasInit && !cs.slice->isIDRorBLA())
      {
        APS* curAPS = cs.slice->alfAps[curApsId];
        if (curAPS && curAPS->layerId <= cs.slice->pic->layerId && curAPS->temporalId <= cs.slice->TLayer && curAPS->alfParam.newFilterFlag[CH_L])
        {
          result.push_back( curApsId );
        }
        apsIdChecked++;
        curApsId = (curApsId + 1) % ALF_CTB_MAX_NUM_APS;
      }
    }
  }
  cs.slice->numAps = ((int)result.size());
  cs.slice->setAlfApsIds(result);
  newApsId = m_apsIdStart - 1;
  if( !m_encCfg->m_alfTempPred )
  {
    newApsId = m_apsIdStart = m_encCfg->m_explicitAPSid;
  }

  if( newApsId < 0 )
  {
    newApsId = ALF_CTB_MAX_NUM_APS - 1;
  }
  CHECK(newApsId >= ALF_CTB_MAX_NUM_APS, "Wrong APS index assignment in getAvaiApsIdsLuma");
  return result;
}
void  EncAdaptiveLoopFilter::initDistortion()
{
  for (int comp = 0; comp < MAX_NUM_COMP; comp++)
  {
#if ENC_ALF_MODS
    for( int apuIdx = 0; apuIdx < m_numApusInPic; apuIdx++ )
    {
      m_ctbDistortionUnfilter[comp][apuIdx] = getUnfilteredDistortion(m_alfCovariance[comp][0][apuIdx], comp == 0 ? MAX_NUM_ALF_CLASSES : 1);
    }
#else
    for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
    {
      m_ctbDistortionUnfilter[comp][ctbIdx] = getUnfilteredDistortion(m_alfCovariance[comp][0][ctbIdx], comp == 0 ? MAX_NUM_ALF_CLASSES : 1);
    }
#endif
  }
}

void  EncAdaptiveLoopFilter::initDistortionCcalf()
{
  for (int comp = 1; comp < MAX_NUM_COMP; comp++)
  {
    for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
    {
      m_ctbDistortionUnfilter[comp][ctbIdx] = m_alfCovarianceCcAlf[comp - 1][0][0][ctbIdx].pixAcc;
    }
  }
}
#define ENC_ALF_MODS_LUMA   ( ENC_ALF_MODS && 1 )
#define ENC_ALF_MODS_CHROMA ( ENC_ALF_MODS && 1 )
void  EncAdaptiveLoopFilter::alfEncoderCtb( CodingStructure& cs, AlfParam& alfParamNewFilters, const double lambdaChromaWeight )
{
  PROFILER_SCOPE_AND_STAGE( 0, _TPROF, P_ALF_ENC_CTB );
  TempCtx   ctxStart       ( m_CtxCache, AlfCtx(m_CABACEstimator->getCtx()));
  TempCtx   ctxBest        ( m_CtxCache );
  TempCtx   ctxTempStart   ( m_CtxCache );
  TempCtx   ctxTempBest    ( m_CtxCache );
  TempCtx   ctxTempAltStart( m_CtxCache );
  TempCtx   ctxTempAltBest ( m_CtxCache );
  AlfParam  alfParamNewFiltersBest = alfParamNewFilters;
  APS**     apss = cs.slice->alfAps;
  short*    alfCtbFilterSetIndex = cs.picture->m_alfCtbFilterIndex.data();
  bool      hasNewFilters[2] = { alfParamNewFilters.alfEnabled[COMP_Y] , alfParamNewFilters.alfEnabled[COMP_Cb] || alfParamNewFilters.alfEnabled[COMP_Cr] };
  const double invFactor = 1.0/((double)(1<<(m_NUM_BITS-1)));
  const bool doClip      = m_encCfg->m_useNonLinearAlfLuma || m_encCfg->m_useNonLinearAlfChroma;
  initDistortion();

  //luma
  m_alfParamTemp = alfParamNewFilters;
  setCtuEnableFlag(m_ctuEnableFlag, CH_L, 1);
  getFrameStats(CH_L, 0);
  setCtuEnableFlag(m_ctuEnableFlag, CH_L, 0);
  double costOff = getUnfilteredDistortion(m_alfCovarianceFrame[CH_L][0], CH_L);

  int newApsId;
  std::vector<int> apsIds = getAvaiApsIdsLuma(cs, newApsId);
  std::vector<int> bestApsIds;
  double costMin = MAX_DOUBLE;
  reconstructCoeffAPSs(cs, true, false, true);

  DTRACE( g_trace_ctx, D_MISC, "POC=%d\n", cs.slice->poc );

  int numLoops = hasNewFilters[CH_L] ? 2 : 1;
  for (int useNewFilter = 0; useNewFilter < numLoops; useNewFilter++)
  {
    DTRACE( g_trace_ctx, D_MISC, "\t useNewFilter=%d\n", useNewFilter );
    int bitsNewFilter = 0;
    if (useNewFilter == 1)
    {
      if (!hasNewFilters[CH_L])
      {
        continue;
      }
      else
      {
        bitsNewFilter = m_bitsNewFilter[CH_L];
        reconstructCoeff(alfParamNewFilters, CH_L, true, true);
      }
    }
    int numIter = useNewFilter ? 2 : 1;
    for (int numTemporalAps = 0; numTemporalAps <= apsIds.size(); numTemporalAps++)
    {
      DTRACE( g_trace_ctx, D_MISC, "\t\t numTemporalAps=%d\n", numTemporalAps );
      if (numTemporalAps + useNewFilter >= ALF_CTB_MAX_NUM_APS)
      {
        continue;
      }
      cs.slice->numAps = (numTemporalAps + useNewFilter);
      int numFilterSet = NUM_FIXED_FILTER_SETS + numTemporalAps + useNewFilter;
      if (numTemporalAps == apsIds.size() && numTemporalAps > 0 && useNewFilter && newApsId == apsIds.back()) //last temporalAPS is occupied by new filter set and this temporal APS becomes unavailable
      {
        continue;
      }
      for (int iter = 0; iter < numIter; iter++)
      {
        DTRACE( g_trace_ctx, D_MISC, "\t\t\t iter=%d\n", iter );
        m_alfParamTemp = alfParamNewFilters;
        m_alfParamTemp.alfEnabled[CH_L] = true;
        double curCost = 3 * m_lambda[CH_L];
        if (iter > 0)  //re-derive new filter-set
        {
          double dDistOrgNewFilter = 0;
          int blocksUsingNewFilter = 0;
#if ENC_ALF_MODS_LUMA
          for( int apuIdx = 0; apuIdx < m_numApusInPic; apuIdx++ )
          {
            int ctuX, ctuY;
            getApuCtuXY( apuIdx, ctuX, ctuY );
            int ctbIdx = ctuY * cs.pcv->widthInCtus + ctuX;
#else
          for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
          {
#if ENC_ALF_MODS
            int apuIdx = ctbIdx;
#endif
#endif

            if (m_ctuEnableFlag[COMP_Y][apuIdx] && alfCtbFilterSetIndex[ctbIdx] != NUM_FIXED_FILTER_SETS)
            {
              m_ctuEnableFlag[COMP_Y][ctbIdx] = 0;
            }
            else if (m_ctuEnableFlag[COMP_Y][ctbIdx] && alfCtbFilterSetIndex[ctbIdx] == NUM_FIXED_FILTER_SETS)
            {
              DTRACE( g_trace_ctx, D_MISC, "\t\t\t\t re-derive ctbIdx=%d\n", apuIdx );
              blocksUsingNewFilter++;
              dDistOrgNewFilter += m_ctbDistortionUnfilter[COMP_Y][apuIdx];
              for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
              {
                short* pCoeff = m_coeffFinal;
                short* pClipp = m_clippFinal;
                for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
                {
                  m_filterTmp[i] = pCoeff[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                  m_clipTmp[i]   = pClipp[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                }
                dDistOrgNewFilter += doClip ? m_alfCovariance[COMP_Y][0][apuIdx][classIdx].calcErrorForCoeffs<true >(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, invFactor)
                                            : m_alfCovariance[COMP_Y][0][apuIdx][classIdx].calcErrorForCoeffs<false>(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, invFactor);
              }
            }
          } //for(ctbIdx)
          if (blocksUsingNewFilter > 0 && blocksUsingNewFilter < m_numCTUsInPic)
          {
            int bitNL[2] = { 0, 0 };
            double errNL[2] = { 0.0, 0.0 };
            m_alfParamTemp.nonLinearFlag[CH_L] = 1;
            if (m_encCfg->m_useNonLinearAlfLuma)
            {
              errNL[1] = getFilterCoeffAndCost(cs, 0, CH_L, true, 0, bitNL[1], true);
              m_alfParamTempNL = m_alfParamTemp;
            }
            else
            {
              errNL[1] = MAX_DOUBLE;
            }
            m_alfParamTemp.nonLinearFlag[CH_L] = 0;
            errNL[0] = getFilterCoeffAndCost(cs, 0, CH_L, true, 0, bitNL[0], true);

            int bitsNewFilterTempLuma = bitNL[0];
            double err = errNL[0];
            if (errNL[1]  < errNL[0])
            {
              err = errNL[1];
              bitsNewFilterTempLuma = bitNL[1];
              m_alfParamTemp = m_alfParamTempNL;
            }

            DTRACE( g_trace_ctx, D_MISC, " dDistOrgNewFilter=%.2f, rate=%.2f, cost=%.2f, rederived cost=%2.f ",
              dDistOrgNewFilter, m_lambda[CH_L] * m_bitsNewFilter[CH_L], dDistOrgNewFilter + m_lambda[CH_L] * m_bitsNewFilter[CH_L], err );

            if (dDistOrgNewFilter + m_lambda[CH_L] * m_bitsNewFilter[CH_L] < err) //re-derived filter is not good, skip
            {
              DTRACE( g_trace_ctx, D_MISC, "\n" );
              continue;
            }

            DTRACE( g_trace_ctx, D_MISC, ", use it\n" );

            reconstructCoeff(m_alfParamTemp, CH_L, true, true);
            bitsNewFilter = bitsNewFilterTempLuma;
          }
          else //no blocks using new filter, skip
          {
            continue;
          }
        }

        m_CABACEstimator->getCtx() = ctxStart;
#if ENC_ALF_MODS_LUMA
        for( int apuIdx = 0; apuIdx < m_numApusInPic; apuIdx++ )
        {
          int ctuX, ctuY;
          getApuCtuXY( apuIdx, ctuX, ctuY );
          int ctbIdx = ctuY * cs.pcv->widthInCtus + ctuX;
#else
        for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
        {
#if ENC_ALF_MODS
          int apuIdx = ctbIdx;
#endif
          const double ctuLambda = lambdaChromaWeight > 0.0 ? cs.picture->ctuQpaLambda[ctbIdx] : m_lambda[COMP_Y];
#endif
          const double distUnfilterCtb = m_ctbDistortionUnfilter[COMP_Y][apuIdx];
          DTRACE( g_trace_ctx, D_MISC, "\t\t\t ctbIdx=%d\n", ctbIdx );
          //ctb on
          m_ctuEnableFlag[COMP_Y][ctbIdx] = 1;
          double         costOn = MAX_DOUBLE;
          ctxTempStart = AlfCtx(m_CABACEstimator->getCtx());
          int iBestFilterSetIdx = 0;
          for (int filterSetIdx = 0; filterSetIdx < numFilterSet; filterSetIdx++)
          {
            DTRACE( g_trace_ctx, D_MISC, "\t\t\t\t filterSetIdx=%d ", filterSetIdx );
            //rate
            m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
            m_CABACEstimator->resetBits();
#if ENC_ALF_MODS_LUMA
            alfCtbFilterSetIndex[ctbIdx] = filterSetIdx;
            double rateOn = xCodeAlfApuLuma( cs, ctuX, ctuY, ctbIdx, COMP_Y, &m_alfParamTemp, lambdaChromaWeight );
#else
            m_CABACEstimator->codeAlfCtuEnabledFlag(cs, ctbIdx, COMP_Y, &m_alfParamTemp);
            alfCtbFilterSetIndex[ctbIdx] = filterSetIdx;
            m_CABACEstimator->codeAlfCtuFilterIndex(cs, ctbIdx, m_alfParamTemp.alfEnabled[COMP_Y]);
            double rateOn = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#endif
            //distortion
            double dist = distUnfilterCtb;

            for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
            {
              if (filterSetIdx < NUM_FIXED_FILTER_SETS)
              {
                // fixed filter set
                int filterIdx = m_classToFilterMapping[filterSetIdx][classIdx];

                if( !m_alfCovariance[COMP_Y][0][apuIdx][classIdx].all0 )
                  dist += doClip ? m_alfCovariance[COMP_Y][0][apuIdx][classIdx].calcErrorForCoeffs<true >(m_clipDefaultEnc, m_fixedFilterSetCoeff[filterIdx], MAX_NUM_ALF_LUMA_COEFF, invFactor)
                                 : m_alfCovariance[COMP_Y][0][apuIdx][classIdx].calcErrorForCoeffs<false>(m_clipDefaultEnc, m_fixedFilterSetCoeff[filterIdx], MAX_NUM_ALF_LUMA_COEFF, invFactor);
              }
              else
              {
                short *pCoeff;
                short *pClipp;
                if (useNewFilter && filterSetIdx == NUM_FIXED_FILTER_SETS)
                {
                  // New filter, no APS
                  pCoeff = m_coeffFinal;
                  pClipp = m_clippFinal;
                }
                else if (useNewFilter)
                {
                  // New filter after APS
                  pCoeff = m_coeffApsLuma[filterSetIdx - 1 - NUM_FIXED_FILTER_SETS];
                  pClipp = m_clippApsLuma[filterSetIdx - 1 - NUM_FIXED_FILTER_SETS];
                }
                else
                {
                  // filter from APS
                  pCoeff = m_coeffApsLuma[filterSetIdx - NUM_FIXED_FILTER_SETS];
                  pClipp = m_clippApsLuma[filterSetIdx - NUM_FIXED_FILTER_SETS];
                }
                for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF; i++)
                {
                  m_filterTmp[i] = pCoeff[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                  m_clipTmp[i]   = pClipp[classIdx * MAX_NUM_ALF_LUMA_COEFF + i];
                }

                if( !m_alfCovariance[COMP_Y][0][apuIdx][classIdx].all0 )
                  dist += doClip ? m_alfCovariance[COMP_Y][0][apuIdx][classIdx].calcErrorForCoeffs<true >(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, invFactor)
                                 : m_alfCovariance[COMP_Y][0][apuIdx][classIdx].calcErrorForCoeffs<false>(m_clipTmp, m_filterTmp, MAX_NUM_ALF_LUMA_COEFF, invFactor);
              }
            } //for(classIdx)
            //cost
#if ENC_ALF_MODS_LUMA
            const double costOnTmp = dist + rateOn;
#else
            const double costOnTmp = dist + ctuLambda * rateOn;
#endif
            DTRACE( g_trace_ctx, D_MISC, "\t cost = %.2f, rate = %.2f, dist = %.2f", costOnTmp, rateOn, dist );

            if (costOnTmp < costOn)
            {
              ctxTempBest = AlfCtx(m_CABACEstimator->getCtx());
              costOn = costOnTmp;
              iBestFilterSetIdx = filterSetIdx;
            }
            DTRACE( g_trace_ctx, D_MISC, "\n" );
          } //for(filterSetIdx)

          DTRACE( g_trace_ctx, D_MISC, "\t\t\t costOn =%.2f\n", costOn );
          //ctb off
          m_ctuEnableFlag[COMP_Y][ctbIdx] = 0;
          //rate
          m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
          m_CABACEstimator->resetBits();
#if ENC_ALF_MODS_LUMA
          double rateCost = xCodeAlfApuEnabledFlag( cs, ctuX, ctuY, ctbIdx, COMP_Y, &m_alfParamTemp, lambdaChromaWeight );
          //cost
          const double costOff = distUnfilterCtb + rateCost;
#else
          m_CABACEstimator->codeAlfCtuEnabledFlag(cs, ctbIdx, COMP_Y, &m_alfParamTemp);
          //cost
          const double costOff = distUnfilterCtb + ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#endif
          DTRACE( g_trace_ctx, D_MISC, "\t\t\t costOff=%.2f\n", costOff );

          uint8_t enable = 0;
          if (costOn < costOff)
          {
            m_CABACEstimator->getCtx() = AlfCtx(ctxTempBest);
            alfCtbFilterSetIndex[ctbIdx] = iBestFilterSetIdx;
            enable = 1;
            curCost += costOn;
          }
          else
          {
            curCost += costOff;
          }
#if ENC_ALF_MODS_LUMA
          xStoreAlfApuEnabledFlag( cs, ctuX, ctuY, ctbIdx, COMP_Y, enable );
#else
          m_ctuEnableFlag[COMP_Y][ctbIdx] = enable;
#endif
        } //for(ctbIdx)
        int tmpBits = bitsNewFilter + 3 * (numFilterSet - NUM_FIXED_FILTER_SETS);
        curCost += tmpBits * m_lambda[COMP_Y];
        if (curCost < costMin)
        {
          costMin = curCost;
          bestApsIds.resize(numFilterSet - NUM_FIXED_FILTER_SETS);
          for (int i = 0; i < bestApsIds.size(); i++)
          {
            if (i == 0 && useNewFilter)
            {
              bestApsIds[i] = newApsId;
            }
            else
            {
              bestApsIds[i] = apsIds[i - useNewFilter];
            }
          }
          alfParamNewFiltersBest = m_alfParamTemp;
          ctxBest = AlfCtx(m_CABACEstimator->getCtx());
          copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CH_L);
          for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
          {
            m_alfCtbFilterSetIndexTmp[ctuIdx] = alfCtbFilterSetIndex[ctuIdx];
          }
          alfParamNewFiltersBest.newFilterFlag[CH_L] = useNewFilter;
        }
        DTRACE( g_trace_ctx, D_MISC, "\t\t curCost=%.2f\n", curCost );
        DTRACE( g_trace_ctx, D_MISC, "\t\t costMin=%.2f\n", costMin );
      }//for (int iter = 0; iter < numIter; iter++)
    }// for (int numTemporalAps = 0; numTemporalAps < apsIds.size(); numTemporalAps++)
  }//for (int useNewFilter = 0; useNewFilter <= 1; useNewFilter++)

  cs.slice->ccAlfCbApsId = newApsId;
  cs.slice->ccAlfCrApsId = newApsId;

  if (costOff <= costMin)
  {
    memset( cs.slice->alfEnabled, 0, sizeof( cs.slice->alfEnabled ) );
    cs.slice->numAps = (0);
    setCtuEnableFlag(m_ctuEnableFlag, CH_L, 0);
    setCtuEnableFlag(m_ctuEnableFlag, CH_C, 0);
    return;
  }
  else
  {
    cs.slice->alfEnabled[COMP_Y] = true;
    cs.slice->numAps = ((int)bestApsIds.size());
    cs.slice->setAlfApsIds(bestApsIds);
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CH_L);
    for (int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ctuIdx++)
    {
      alfCtbFilterSetIndex[ctuIdx] = m_alfCtbFilterSetIndexTmp[ctuIdx];
    }
    if (alfParamNewFiltersBest.newFilterFlag[CH_L])
    {
      APS* newAPS = m_apsMap->getPS((newApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      if (newAPS == NULL)
      {
        newAPS = m_apsMap->allocatePS((newApsId << NUM_APS_TYPE_LEN) + ALF_APS);
        newAPS->apsId = newApsId;
        newAPS->apsType = ALF_APS;
      }
      newAPS->alfParam = alfParamNewFiltersBest;
      newAPS->temporalId = cs.slice->TLayer;
      newAPS->alfParam.newFilterFlag[CH_C] = false;
      newAPS->poc = cs.slice->poc;
      m_apsMap->setChangedFlag((newApsId << NUM_APS_TYPE_LEN) + ALF_APS);
      m_apsIdStart = newApsId;
    }

    const std::vector<int>& apsIds = cs.slice->lumaApsId;
    for (int i = 0; i < (int)cs.slice->numAps; i++)
    {
      apss[apsIds[i]] = m_apsMap->getPS((apsIds[i] << NUM_APS_TYPE_LEN) + ALF_APS);
    }
  }

  //chroma
  if (isChromaEnabled(cs.pcv->chrFormat))
  {
  m_alfParamTemp = alfParamNewFiltersBest;
  if( m_alfParamTemp.numAlternativesChroma < 1 )
  {
    m_alfParamTemp.numAlternativesChroma = 1;
  }
  setCtuAlternativeChroma( m_ctuAlternative, 0 );
  setCtuEnableFlag(m_ctuEnableFlag, CH_C, 1);
  getFrameStats(CH_C, 0);
  costOff = getUnfilteredDistortion(m_alfCovarianceFrame[CH_C][0], CH_C)/* + m_lambda[CH_C] * 1.0*/;
  costMin = MAX_DOUBLE;
  m_CABACEstimator->getCtx() = AlfCtx(ctxBest);
  ctxStart = AlfCtx(m_CABACEstimator->getCtx());
  int newApsIdChroma = -1;
  if (alfParamNewFiltersBest.newFilterFlag[CH_L] && (alfParamNewFiltersBest.alfEnabled[COMP_Cb] || alfParamNewFiltersBest.alfEnabled[COMP_Cr]))
  {
    newApsIdChroma = newApsId;
  }
  else if (alfParamNewFiltersBest.alfEnabled[COMP_Cb] || alfParamNewFiltersBest.alfEnabled[COMP_Cr])
  {
    int curId = m_apsIdStart;
    while (newApsIdChroma < 0)
    {
      curId--;
      if (curId < 0)
      {
        curId = ALF_CTB_MAX_NUM_APS - 1;
      }
      if (std::find(bestApsIds.begin(), bestApsIds.end(), curId) == bestApsIds.end())
      {
        newApsIdChroma = !m_encCfg->m_alfTempPred ? newApsId: curId;
      }
    }
  }
  for (int curApsId = 0; curApsId < ALF_CTB_MAX_NUM_APS; curApsId++)
  {
    if ((cs.slice->pendingRasInit || cs.slice->isIDRorBLA() || cs.slice->isIntra()) && curApsId != newApsIdChroma)
    {
      continue;
    }
    APS* curAPS = m_apsMap->getPS((curApsId << NUM_APS_TYPE_LEN) + ALF_APS);

    if( curAPS && curAPS->layerId != cs.slice->pic->layerId )
    {
      continue;
    }
    const double lambdaChroma = (m_lambda[COMP_Cb] + m_lambda[COMP_Cr]) * 0.5;
    double curCost = lambdaChroma * 3;
    int maxTID = cs.slice->TLayer;
    if (curApsId == newApsIdChroma)
    {
      m_alfParamTemp = alfParamNewFilters;
      curCost += lambdaChroma * m_bitsNewFilter[CH_C];
    }
    else if (m_encCfg->m_alfTempPred && curAPS && curAPS->temporalId <= maxTID && curAPS->alfParam.newFilterFlag[CH_C])
    {
      m_alfParamTemp = curAPS->alfParam;
    }
    else
    {
      continue;
    }
    reconstructCoeff(m_alfParamTemp, CH_C, true, true);
    m_CABACEstimator->getCtx() = AlfCtx(ctxStart);
    for (int compId = 1; compId < MAX_NUM_COMP; compId++)
    {
      m_alfParamTemp.alfEnabled[compId] = true;
#if ENC_ALF_MODS_CHROMA
      for( int apuIdx = 0; apuIdx < m_numApusInPic; apuIdx++ )
      {
        int ctuX, ctuY;
        getApuCtuXY( apuIdx, ctuX, ctuY );
        int ctbIdx = ctuY * cs.pcv->widthInCtus + ctuX;
#else
      for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
      {
#if ENC_ALF_MODS
        int apuIdx = ctbIdx;
#endif
#endif
        const double ctuLambda = lambdaChromaWeight > 0.0 ? cs.picture->ctuQpaLambda[ctbIdx] * lambdaChromaWeight : m_lambda[compId];
        double distUnfilterCtu = m_ctbDistortionUnfilter[compId][apuIdx];
        //cost on
        m_ctuEnableFlag[compId][ctbIdx] = 1;
        ctxTempStart = AlfCtx(m_CABACEstimator->getCtx());
        //rate
        m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
        m_CABACEstimator->resetBits();
        //ctb flag
#if ENC_ALF_MODS_CHROMA
        double rateCostOn = xCodeAlfApuEnabledFlag( cs, ctuX, ctuY, ctbIdx, compId, &m_alfParamTemp, lambdaChromaWeight );
#else
        m_CABACEstimator->codeAlfCtuEnabledFlag(cs, ctbIdx, compId, &m_alfParamTemp);
        double rateOn = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#endif
        double dist = MAX_DOUBLE;
        int numAlts = m_alfParamTemp.numAlternativesChroma;
        ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );
#if ENC_ALF_MODS_CHROMA
        double bestRateAltCost = 0;
#else
        double bestAltRate = 0;
#endif
        double bestAltCost = MAX_DOUBLE;
        int bestAltIdx = -1;
        ctxTempAltStart = AlfCtx( ctxTempBest );
        for( int altIdx = 0; altIdx < numAlts; ++altIdx )
        {
          if( altIdx )
            m_CABACEstimator->getCtx() = AlfCtx( ctxTempAltStart );
          m_CABACEstimator->resetBits();
          m_ctuAlternative[compId][ctbIdx] = altIdx;
#if ENC_ALF_MODS_CHROMA
          double r_altCost = xCodeAlfApuAlternative( cs, ctuX, ctuY, ctbIdx, compId, &m_alfParamTemp, lambdaChromaWeight );
#else
          m_CABACEstimator->codeAlfCtuAlternative( cs, ctbIdx, compId, &m_alfParamTemp );
          double altRate   = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
          double r_altCost = ctuLambda * altRate;
#endif

          //distortion
          for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
          {
            m_filterTmp[i] = m_chromaCoeffFinal[altIdx][i];
            m_clipTmp[i]   = m_chromaClippFinal[altIdx][i];
          }
          double altDist = doClip ? m_alfCovariance[compId][0][apuIdx][0].calcErrorForCoeffs<true >( m_clipTmp, m_filterTmp, MAX_NUM_ALF_CHROMA_COEFF, invFactor )
                                  : m_alfCovariance[compId][0][apuIdx][0].calcErrorForCoeffs<false>( m_clipTmp, m_filterTmp, MAX_NUM_ALF_CHROMA_COEFF, invFactor );
          double altCost = altDist + r_altCost;
          if( altCost < bestAltCost )
          {
            bestAltCost = altCost;
            bestAltIdx = altIdx;
#if ENC_ALF_MODS_CHROMA
            bestRateAltCost = r_altCost;
#else
            bestAltRate = altRate;
#endif
            ctxTempBest = AlfCtx( m_CABACEstimator->getCtx() );
            dist = altDist;
          }
        }
        m_ctuAlternative[compId][ctbIdx] = bestAltIdx;
#if ENC_ALF_MODS_CHROMA
        rateCostOn += bestRateAltCost;
#else
        rateOn += bestAltRate;
#endif
        dist += distUnfilterCtu;
        //cost
#if ENC_ALF_MODS_CHROMA
        const double costOn = dist + rateCostOn;
#else
        const double costOn = dist + ctuLambda * rateOn;
#endif
        //cost off
        m_ctuEnableFlag[compId][ctbIdx] = 0;
        //rate
        m_CABACEstimator->getCtx() = AlfCtx(ctxTempStart);
        m_CABACEstimator->resetBits();
#if ENC_ALF_MODS_CHROMA
        double rateCost = xCodeAlfApuEnabledFlag( cs, ctuX, ctuY, ctbIdx, compId, &m_alfParamTemp, lambdaChromaWeight );
        //cost
        const double costOff = distUnfilterCtu + rateCost;
#else
        m_CABACEstimator->codeAlfCtuEnabledFlag(cs, ctbIdx, compId, &m_alfParamTemp);
        //cost
        const double costOff = distUnfilterCtu + ctuLambda * FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
#endif
        
        uint8_t enable = 0;
        if (costOn < costOff)
        {
          m_CABACEstimator->getCtx() = AlfCtx(ctxTempBest);
          enable = 1;
          curCost += costOn;
        }
        else
        {
          curCost += costOff;
        }
#if ENC_ALF_MODS_CHROMA
        xStoreAlfApuEnabledFlag( cs, ctuX, ctuY, ctbIdx, compId, enable );
#else
        m_ctuEnableFlag[compId][ctbIdx] = enable;
#endif

      } //for(ctbIdx)
    }
    //chroma idc
    setEnableFlag(m_alfParamTemp, CH_C, m_ctuEnableFlag);

    if (curCost < costMin)
    {
      costMin = curCost;
      cs.slice->chromaApsId = curApsId;
      cs.slice->alfEnabled[COMP_Cb] = m_alfParamTemp.alfEnabled[COMP_Cb];
      cs.slice->alfEnabled[COMP_Cr] = m_alfParamTemp.alfEnabled[COMP_Cr];
      copyCtuEnableFlag(m_ctuEnableFlagTmp, m_ctuEnableFlag, CH_C);
      copyCtuAlternativeChroma(m_ctuAlternativeTmp, m_ctuAlternative);
    }
  }
  if (newApsIdChroma >= 0)
  {
    cs.slice->ccAlfCbApsId = newApsIdChroma;
    cs.slice->ccAlfCrApsId = newApsIdChroma;
  }
  if (costOff < costMin)
  {
    cs.slice->alfEnabled[COMP_Cb] = false;
    cs.slice->alfEnabled[COMP_Cr] = false;
    setCtuEnableFlag(m_ctuEnableFlag, CH_C, 0);
  }
  else
  {
    copyCtuEnableFlag(m_ctuEnableFlag, m_ctuEnableFlagTmp, CH_C);
    copyCtuAlternativeChroma(m_ctuAlternative, m_ctuAlternativeTmp);
    if (cs.slice->chromaApsId == newApsIdChroma)  //new filter
    {
      APS* newAPS = m_apsMap->getPS((newApsIdChroma << NUM_APS_TYPE_LEN) + ALF_APS);
      if (newAPS == NULL)
      {
        newAPS = m_apsMap->allocatePS((newApsIdChroma << NUM_APS_TYPE_LEN) + ALF_APS);
        newAPS->apsType = ALF_APS;
        newAPS->apsId   = newApsIdChroma;
        newAPS->alfParam.reset();
      }
      newAPS->alfParam.newFilterFlag[CH_C] = true;
      if (!alfParamNewFiltersBest.newFilterFlag[CH_L])
      {
        newAPS->alfParam.newFilterFlag[CH_L] = false;
      }
      newAPS->alfParam.numAlternativesChroma = alfParamNewFilters.numAlternativesChroma;
      newAPS->alfParam.nonLinearFlag[CH_C] = alfParamNewFilters.nonLinearFlag[CH_C];
      newAPS->temporalId = cs.slice->TLayer;
      newAPS->poc = cs.slice->poc;
      for (int altIdx = 0; altIdx  < VVENC_MAX_NUM_ALF_ALTERNATIVES_CHROMA; ++altIdx )
      for (int i = 0; i < MAX_NUM_ALF_CHROMA_COEFF; i++)
      {
        newAPS->alfParam.chromaCoeff[altIdx][i] = alfParamNewFilters.chromaCoeff[altIdx][i];
        newAPS->alfParam.chromaClipp[altIdx][i] = alfParamNewFilters.chromaClipp[altIdx][i];
      }
      m_apsMap->setChangedFlag((newApsIdChroma << NUM_APS_TYPE_LEN) + ALF_APS);
      m_apsIdStart = newApsIdChroma;
    }
    apss[cs.slice->chromaApsId] = m_apsMap->getPS((cs.slice->chromaApsId << NUM_APS_TYPE_LEN) + ALF_APS);
  }
  }
}

void EncAdaptiveLoopFilter::copyCtuAlternativeChroma( uint8_t* ctuAltsDst[MAX_NUM_COMP], uint8_t* ctuAltsSrc[MAX_NUM_COMP] )
{
  std::copy_n( ctuAltsSrc[COMP_Cb], m_numCTUsInPic, ctuAltsDst[COMP_Cb] );
  std::copy_n( ctuAltsSrc[COMP_Cr], m_numCTUsInPic, ctuAltsDst[COMP_Cr] );
}

void EncAdaptiveLoopFilter::setCtuAlternativeChroma( uint8_t* ctuAlts[MAX_NUM_COMP], uint8_t val )
{
  std::fill_n( ctuAlts[COMP_Cb], m_numCTUsInPic, val );
  std::fill_n( ctuAlts[COMP_Cr], m_numCTUsInPic, val );
}

void EncAdaptiveLoopFilter::initCtuAlternativeChroma( uint8_t* ctuAlts[MAX_NUM_COMP] )
{
  uint8_t altIdx = 0;
#if ENC_ALF_MODS
#if OPTIMIZE_ORIG_CASE
  if( m_numApusInPic == m_numCTUsInPic )
  {
    for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ++ctuIdx )
    {
      ctuAlts[COMP_Cb][ctuIdx] = altIdx;
      ctuAlts[COMP_Cr][ctuIdx] = altIdx;
      if( (ctuIdx+1) * m_alfParamTemp.numAlternativesChroma >= (altIdx+1)*m_numCTUsInPic )
        ++altIdx;
    }
  }
  else
#endif
  {
    for( int apuIdx = 0; apuIdx < m_numApusInPic; apuIdx++ )
    {
      int ctuX, ctuY;
      getApuCtuXY( apuIdx, ctuX, ctuY );
      int ctuMaxX = getApuMaxCtuX( ctuX );
      int ctuMaxY = getApuMaxCtuY( ctuY );
      for( int cY = ctuY; cY < ctuMaxY; cY++ )
      {
        for( int cX = ctuX; cX < ctuMaxX; cX++ )
        {
          int ctuIdx = cY * m_numCTUsInWidth/*cs.pcv->widthInCtus*/ + cX;
          ctuAlts[COMP_Cb][ctuIdx] = altIdx;
          ctuAlts[COMP_Cr][ctuIdx] = altIdx;
        }
      }
      if( (apuIdx+1) * m_alfParamTemp.numAlternativesChroma >= (altIdx+1)*m_numApusInPic )
        ++altIdx;
    }
  }
#else
  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ++ctuIdx )
  {
    ctuAlts[COMP_Cb][ctuIdx] = altIdx;
    if( (ctuIdx+1) * m_alfParamTemp.numAlternativesChroma >= (altIdx+1)*m_numCTUsInPic )
      ++altIdx;
  }
  altIdx = 0;
  for( int ctuIdx = 0; ctuIdx < m_numCTUsInPic; ++ctuIdx )
  {
    ctuAlts[COMP_Cr][ctuIdx] = altIdx;
    if( (ctuIdx+1) * m_alfParamTemp.numAlternativesChroma >= (altIdx+1)*m_numCTUsInPic )
      ++altIdx;
  }
#endif
}

int EncAdaptiveLoopFilter::getMaxNumAlternativesChroma( )
{
  return std::min<int>( m_numCTUsInPic * 2, m_encCfg->m_maxNumAlfAlternativesChroma );
}

int EncAdaptiveLoopFilter::getCoeffRateCcAlf(short chromaCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], bool filterEnabled[MAX_NUM_CC_ALF_FILTERS], uint8_t filterCount, ComponentID compID)
{
  int bits = 0;

  if ( filterCount > 0 )
  {
    bits += lengthUvlc(filterCount - 1);
    int signaledFilterCount = 0;
    for ( int filterIdx=0; filterIdx<MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
    {
      if (filterEnabled[filterIdx])
      {
        AlfFilterShape alfShape(size_CC_ALF);
        // Filter coefficients
        for (int i = 0; i < alfShape.numCoeff - 1; i++)
        {
          bits += CCALF_BITS_PER_COEFF_LEVEL + (chromaCoeff[filterIdx][i] == 0 ? 0 : 1);
        }

        signaledFilterCount++;
      }
    }
    CHECK(signaledFilterCount != filterCount, "Number of filter signaled not same as indicated");
  }

  return bits;
}

void EncAdaptiveLoopFilter::deriveCcAlfFilterCoeff( ComponentID compID, const PelUnitBuf& recYuv, const PelUnitBuf& recYuvExt, short filterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], const uint8_t filterIdx )
{
  int forward_tab[CCALF_CANDS_COEFF_NR * 2 - 1] = {0};
  for (int i = 0; i < CCALF_CANDS_COEFF_NR; i++)
  {
    forward_tab[CCALF_CANDS_COEFF_NR - 1 + i] = CCALF_SMALL_TAB[i];
    forward_tab[CCALF_CANDS_COEFF_NR - 1 - i] = (-1) * CCALF_SMALL_TAB[i];
  }
  using TE = alf_float_t[MAX_NUM_ALF_LUMA_COEFF][MAX_NUM_ALF_LUMA_COEFF];
  using Ty = alf_float_t[MAX_NUM_ALF_LUMA_COEFF];

  alf_float_t filterCoeffDbl[MAX_NUM_CC_ALF_CHROMA_COEFF];
  int16_t filterCoeffInt[MAX_NUM_CC_ALF_CHROMA_COEFF];

  std::fill_n(filterCoeffInt, MAX_NUM_CC_ALF_CHROMA_COEFF, 0);

  TE        kE;
  Ty        ky;
  const int size = m_filterShapesCcAlf[compID - 1][0].numCoeff - 1;

  for (int k = 0; k < size; k++)
  {
    ky[k] = m_alfCovarianceFrameCcAlf[compID - 1][0][filterIdx].y[0][k];
    for (int l = 0; l < size; l++)
    {
      kE[k][l] = m_alfCovarianceFrameCcAlf[compID - 1][0][filterIdx].E[0][0][k][l];
    }
  }

  m_alfCovarianceFrameCcAlf[compID - 1][0][filterIdx].gnsSolveByChol(kE, ky, filterCoeffDbl, size);
  roundFiltCoeffCCALF(filterCoeffInt, filterCoeffDbl, size, (1 << m_scaleBits));

  for (int k = 0; k < size; k++)
  {
    CHECK( filterCoeffInt[k] < -(1 << CCALF_DYNAMIC_RANGE), "this is not possible: filterCoeffInt[k] <  -(1 << CCALF_DYNAMIC_RANGE)");
    CHECK( filterCoeffInt[k] > (1 << CCALF_DYNAMIC_RANGE), "this is not possible: filterCoeffInt[k] >  (1 << CCALF_DYNAMIC_RANGE)");
  }

  const alf_float_t invFactor = 1.0 / ( alf_float_t )( 1 << m_scaleBits );
  // Refine quanitzation
  int modified       = 1;
  double errRef      = m_alfCovarianceFrameCcAlf[compID - 1][0][filterIdx].calcErrorForCcAlfCoeffs(filterCoeffInt, size, invFactor);
  while (modified)
  {
    modified = 0;
    for (int delta : { 1, -1 })
    {
      double errMin = MAX_DOUBLE;
      int    idxMin = -1;
      int minIndex = -1;

      for (int k = 0; k < size; k++)
      {
        int org_idx = -1;
        for (int i = 0; i < CCALF_CANDS_COEFF_NR * 2 - 1; i++)
        {
          if (forward_tab[i] == filterCoeffInt[k])
          {
            org_idx = i;
            break;
          }
        }
        CHECK( org_idx < 0, "this is wrong, does not find coeff from forward_tab");
        if ( (org_idx - delta < 0) || (org_idx - delta >= CCALF_CANDS_COEFF_NR * 2 - 1) )
          continue;

        filterCoeffInt[k] = forward_tab[org_idx - delta];
        double error = m_alfCovarianceFrameCcAlf[compID - 1][0][filterIdx].calcErrorForCcAlfCoeffs(filterCoeffInt, size, invFactor);
        if( error < errMin )
        {
          errMin = error;
          idxMin = k;
          minIndex = org_idx;
        }
        filterCoeffInt[k] = forward_tab[org_idx];
      }
      if (errMin < errRef)
      {
        minIndex -= delta;
        CHECK( minIndex < 0, "this is wrong, index - delta < 0");
        CHECK( minIndex >= CCALF_CANDS_COEFF_NR * 2 - 1, "this is wrong, index - delta >= CCALF_CANDS_COEFF_NR * 2 - 1");
        filterCoeffInt[idxMin] = forward_tab[minIndex];
        modified++;
        errRef = errMin;
      }
    }
  }


  for (int k = 0; k < (size + 1); k++)
  {
    CHECK((filterCoeffInt[k] < -(1 << CCALF_DYNAMIC_RANGE)) || (filterCoeffInt[k] > (1 << CCALF_DYNAMIC_RANGE)), "Exceeded valid range for CC ALF coefficient");
    filterCoeff[filterIdx][k] = filterCoeffInt[k];
  }
}


void EncAdaptiveLoopFilter::determineControlIdcValues(CodingStructure &cs, const ComponentID compID, const PelBuf *buf,
                                                      const int ctuWidthC, const int ctuHeightC, const int picWidthC,
                                                      const int picHeightC, double **unfilteredDistortion,
                                                      uint64_t *trainingDistortion[MAX_NUM_CC_ALF_FILTERS],
                                                      uint64_t *lumaSwingGreaterThanThresholdCount,
                                                      uint64_t *chromaSampleCountNearMidPoint,
                                                      bool reuseTemporalFilterCoeff, uint8_t *trainingCovControl,
                                                      uint8_t *filterControl, uint64_t &curTotalDistortion,
                                                      double &curTotalRate, bool filterEnabled[MAX_NUM_CC_ALF_FILTERS],
                                                      uint8_t  mapFilterIdxToFilterIdc[MAX_NUM_CC_ALF_FILTERS + 1],
                                                      uint8_t &ccAlfFilterCount)
{
  bool curFilterEnabled[MAX_NUM_CC_ALF_FILTERS];
  std::fill_n(curFilterEnabled, MAX_NUM_CC_ALF_FILTERS, false);

  FilterIdxCount filterIdxCount[MAX_NUM_CC_ALF_FILTERS];
  for (int i = 0; i < MAX_NUM_CC_ALF_FILTERS; i++)
  {
    filterIdxCount[i].count     = 0;
    filterIdxCount[i].filterIdx = i;
  }

  double prevRate = curTotalRate;

  TempCtx ctxInitial(m_CtxCache);
  TempCtx ctxBest(m_CtxCache);
  TempCtx ctxStart(m_CtxCache);
  ctxInitial = SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx());
  ctxBest    = SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx());

  int ctuIdx = 0;
  for (int yCtu = 0; yCtu < buf->height; yCtu += ctuHeightC)
  {
    for (int xCtu = 0; xCtu < buf->width; xCtu += ctuWidthC)
    {
      uint64_t ssd;
      double   rate;
      double   cost;

      uint64_t bestSSD       = MAX_UINT64;
      double   bestRate      = MAX_DOUBLE;
      double   bestCost      = MAX_DOUBLE;
      uint8_t  bestFilterIdc = 0;
      uint8_t  bestFilterIdx = 0;
      const uint32_t thresholdS = std::min<int>(buf->height - yCtu, ctuHeightC) << getComponentScaleY(COMP_Cb, m_chromaFormat);
      const uint32_t numberOfChromaSamples = std::min<int>(buf->height - yCtu, ctuHeightC) * std::min<int>(buf->width - xCtu, ctuWidthC);
      const uint32_t thresholdC = (numberOfChromaSamples >> 2);

      m_CABACEstimator->getCtx() = ctxBest;
      ctxStart                   = SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx());

      for (int filterIdx = 0; filterIdx <= MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        uint8_t filterIdc = mapFilterIdxToFilterIdc[filterIdx];
        if (filterIdx < MAX_NUM_CC_ALF_FILTERS && !filterEnabled[filterIdx])
        {
          continue;
        }

        if (filterIdx == MAX_NUM_CC_ALF_FILTERS)
        {
          ssd = unfilteredDistortion[compID][ctuIdx];   // restore saved distortion computation
        }
        else
        {
          ssd = trainingDistortion[filterIdx][ctuIdx];
        }
        m_CABACEstimator->getCtx() = ctxStart;
        m_CABACEstimator->resetBits();
        const Position lumaPos = Position({ xCtu << getComponentScaleX(compID, cs.pcv->chrFormat),
          yCtu << getComponentScaleY(compID, cs.pcv->chrFormat) });
        m_CABACEstimator->codeCcAlfFilterControlIdc(filterIdc, cs, compID, ctuIdx, filterControl, lumaPos,
                                                    ccAlfFilterCount);
        rate = FRAC_BITS_SCALE * m_CABACEstimator->getEstFracBits();
        cost = rate * m_lambda[compID] + ssd;

        bool limitationExceeded = false;
        if (m_limitCcAlf && filterIdx < MAX_NUM_CC_ALF_FILTERS)
        {
          limitationExceeded = limitationExceeded || (lumaSwingGreaterThanThresholdCount[ctuIdx] >= thresholdS);
          limitationExceeded = limitationExceeded || (chromaSampleCountNearMidPoint[ctuIdx] >= thresholdC);
        }
        if (cost < bestCost && !limitationExceeded)
        {
          bestCost      = cost;
          bestRate      = rate;
          bestSSD       = ssd;
          bestFilterIdc = filterIdc;
          bestFilterIdx = filterIdx;

          ctxBest = SubCtx(Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx());

          trainingCovControl[ctuIdx] = (filterIdx == MAX_NUM_CC_ALF_FILTERS) ? 0 : (filterIdx + 1);
          filterControl[ctuIdx]      = (filterIdx == MAX_NUM_CC_ALF_FILTERS) ? 0 : (filterIdx + 1);
        }
      }
      if (bestFilterIdc != 0)
      {
        curFilterEnabled[bestFilterIdx] = true;
        filterIdxCount[bestFilterIdx].count++;
      }
      curTotalRate += bestRate;
      curTotalDistortion += bestSSD;
      ctuIdx++;
    }
  }

  if (!reuseTemporalFilterCoeff)
  {
    std::copy_n(curFilterEnabled, MAX_NUM_CC_ALF_FILTERS, filterEnabled);

    std::stable_sort(filterIdxCount, filterIdxCount + MAX_NUM_CC_ALF_FILTERS, compareCounts);

    int filterIdc = 1;
    ccAlfFilterCount = 0;
    for ( FilterIdxCount &s : filterIdxCount )
    {
      const int filterIdx = s.filterIdx;
      if (filterEnabled[filterIdx])
      {
        mapFilterIdxToFilterIdc[filterIdx] = filterIdc;
        filterIdc++;
        ccAlfFilterCount++;
      }
    }

    curTotalRate = prevRate;
    m_CABACEstimator->getCtx() = ctxInitial;
    m_CABACEstimator->resetBits();
    int ctuIdx = 0;
    for (int y = 0; y < buf->height; y += ctuHeightC)
    {
      for (int x = 0; x < buf->width; x += ctuWidthC)
      {
        const int filterIdxPlus1 = filterControl[ctuIdx];

        const Position lumaPos = Position(
                                          { x << getComponentScaleX(compID, cs.pcv->chrFormat), y << getComponentScaleY(compID, cs.pcv->chrFormat) });

        m_CABACEstimator->codeCcAlfFilterControlIdc(filterIdxPlus1 == 0 ? 0
                                                    : mapFilterIdxToFilterIdc[filterIdxPlus1 - 1],
                                                    cs, compID, ctuIdx, filterControl, lumaPos, ccAlfFilterCount);

        ctuIdx++;
      }
    }
    curTotalRate += FRAC_BITS_SCALE*m_CABACEstimator->getEstFracBits();
  }

  // restore for next iteration
  m_CABACEstimator->getCtx() = ctxInitial;
}

std::vector<int> EncAdaptiveLoopFilter::getAvailableCcAlfApsIds(CodingStructure& cs, ComponentID compID)
{
  APS** apss = cs.slice->alfAps;
  for (int i = 0; i < ALF_CTB_MAX_NUM_APS; i++)
  {
    apss[i] = m_apsMap->getPS((i << NUM_APS_TYPE_LEN) + ALF_APS);
  }

  std::vector<int> result;
  if( m_encCfg->m_alfTempPred )
  {
    int apsIdChecked = 0, curApsId = m_apsIdStart;
    if (curApsId < ALF_CTB_MAX_NUM_APS)
    {
      while (apsIdChecked < ALF_CTB_MAX_NUM_APS && !cs.slice->isIntra() && result.size() < ALF_CTB_MAX_NUM_APS && !cs.slice->pendingRasInit && !cs.slice->isIDRorBLA())
      {
        APS* curAPS = cs.slice->alfAps[curApsId];
        if (curAPS && curAPS->layerId == cs.picture->layerId && curAPS->temporalId <= cs.slice->TLayer && curAPS->ccAlfParam.newCcAlfFilter[compID - 1])
        {
          result.push_back(curApsId);
        }
        apsIdChecked++;
        curApsId = (curApsId + 1) % ALF_CTB_MAX_NUM_APS;
      }
    }
  }
  return result;
}

void EncAdaptiveLoopFilter::getFrameStatsCcalf(ComponentID compIdx, int filterIdc)
{
        int ctuRsAddr = 0;
  const int filterIdx = filterIdc - 1;

  // init Frame stats buffers
  for (int shape = 0; shape != m_filterShapesCcAlf[compIdx - 1].size(); shape++)
  {
    m_alfCovarianceFrameCcAlf[compIdx - 1][shape][filterIdx].reset();
  }

  for (int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight)
  {
    for (int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth)
    {
      if (m_trainingCovControl[ctuRsAddr] == filterIdc)
      {
        for (int shape = 0; shape != m_filterShapesCcAlf[compIdx - 1].size(); shape++)
        {
          m_alfCovarianceFrameCcAlf[compIdx - 1][shape][filterIdx] +=
            m_alfCovarianceCcAlf[compIdx - 1][shape][0][ctuRsAddr];
        }
      }
      ctuRsAddr++;
    }
  }
}

void EncAdaptiveLoopFilter::deriveCcAlfFilter( Picture& pic, CodingStructure& cs )
{
  initCABACEstimator( cs.slice, &pic.picApsMap );
  initDistortionCcalf();

  // Do not transmit CC ALF if it is unchanged
  if( cs.slice->alfEnabled[COMP_Y] )
  {
    for( int32_t lumaAlfApsId : cs.slice->lumaApsId )
    {
      APS* aps = ( lumaAlfApsId >= 0 ) ? m_apsMap->getPS( ( lumaAlfApsId << NUM_APS_TYPE_LEN ) + ALF_APS ) : nullptr;
      if( aps && m_apsMap->getChangedFlag( ( lumaAlfApsId << NUM_APS_TYPE_LEN ) + ALF_APS ) )
      {
        aps->ccAlfParam.newCcAlfFilter[0] = false;
        aps->ccAlfParam.newCcAlfFilter[1] = false;
      }
    }
  }
  int chromaAlfApsId = ( cs.slice->alfEnabled[COMP_Cb] || cs.slice->alfEnabled[COMP_Cr] ) ? cs.slice->chromaApsId : -1;
  APS* aps = ( chromaAlfApsId >= 0 ) ? m_apsMap->getPS( ( chromaAlfApsId << NUM_APS_TYPE_LEN ) + ALF_APS ) : nullptr;
  if( aps && m_apsMap->getChangedFlag( ( chromaAlfApsId << NUM_APS_TYPE_LEN ) + ALF_APS ) )
  {
    aps->ccAlfParam.newCcAlfFilter[0] = false;
    aps->ccAlfParam.newCcAlfFilter[1] = false;
  }

  // Accumulate ALF statistic
  const int filterIdx = 0;
  const int numberOfComponents = getNumberValidComponents( m_chromaFormat );
  for( int compIdx = 1; compIdx < numberOfComponents; compIdx++ )
  {
    const ComponentID compID = ComponentID( compIdx );
    for( int shape = 0; shape != m_filterShapesCcAlf[compIdx - 1].size(); shape++ )
    {
      for( int ctuRsAddr = 0; ctuRsAddr < m_numCTUsInPic; ctuRsAddr++ )
      {
        for( int classIdx = 0; classIdx < ( isLuma( compID ) ? MAX_NUM_ALF_CLASSES : 1 ); classIdx++ )
        {
          m_alfCovarianceFrameCcAlf[compIdx - 1][shape][filterIdx] += m_alfCovarianceCcAlf[compIdx - 1][shape][filterIdx][ctuRsAddr];
        }
      }
    }
  }

  const TempCtx ctxStartCcAlf( m_CtxCache, SubCtx( Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx() ) );
  const PelUnitBuf orgYuv = cs.picture->getOrigBuf();
  const PelUnitBuf tmpYuv = m_tempBuf.getBuf( cs.area );
  const PelUnitBuf recYuv = cs.getRecoBuf();

  m_CABACEstimator->getCtx() = SubCtx( Ctx::CcAlfFilterControlFlag, ctxStartCcAlf );
  deriveCcAlfFilter( cs, COMP_Cb, orgYuv, tmpYuv, recYuv );

  m_CABACEstimator->getCtx() = SubCtx( Ctx::CcAlfFilterControlFlag, ctxStartCcAlf );
  deriveCcAlfFilter( cs, COMP_Cr, orgYuv, tmpYuv, recYuv );

  xSetupCcAlfAPS( cs );
}

void EncAdaptiveLoopFilter::deriveCcAlfFilter( CodingStructure& cs, ComponentID compID, const PelUnitBuf& orgYuv, const PelUnitBuf& tempDecYuvBuf, const PelUnitBuf& dstYuv )
{
  if (!cs.slice->alfEnabled[COMP_Y])
  {
    m_ccAlfFilterParam.ccAlfFilterEnabled[compID - 1] = false;
    return;
  }

  m_limitCcAlf = m_encCfg->m_QP >= m_encCfg->m_ccalfQpThreshold;
  if (m_limitCcAlf && cs.slice->sliceQp <= m_encCfg->m_QP + 1)
  {
    m_ccAlfFilterParam.ccAlfFilterEnabled[compID - 1] = false;
    return;
  }

  uint8_t bestMapFilterIdxToFilterIdc[MAX_NUM_CC_ALF_FILTERS+1];
  const int scaleX               = getComponentScaleX(compID, cs.pcv->chrFormat);
  const int scaleY               = getComponentScaleY(compID, cs.pcv->chrFormat);
  const int ctuWidthC            = cs.pcv->maxCUSize >> scaleX;
  const int ctuHeightC           = cs.pcv->maxCUSize >> scaleY;
  const int picWidthC            = cs.pcv->lumaWidth >> scaleX;
  const int picHeightC           = cs.pcv->lumaHeight >> scaleY;
  const int maxTrainingIterCount = 15;

  if (m_limitCcAlf)
  {
    countLumaSwingGreaterThanThreshold(dstYuv.get(COMP_Y).bufAt(0, 0), dstYuv.get(COMP_Y).stride, dstYuv.get(COMP_Y).height, dstYuv.get(COMP_Y).width, cs.pcv->maxCUSizeLog2, cs.pcv->maxCUSizeLog2, m_lumaSwingGreaterThanThresholdCount, m_numCTUsInWidth);
  }
  if (m_limitCcAlf)
  {
    countChromaSampleValueNearMidPoint(dstYuv.get(compID).bufAt(0, 0), dstYuv.get(compID).stride, dstYuv.get(compID).height, dstYuv.get(compID).width, cs.pcv->maxCUSizeLog2- scaleX, cs.pcv->maxCUSizeLog2 - scaleY, m_chromaSampleCountNearMidPoint, m_numCTUsInWidth);
  }

  for ( int filterIdx = 0; filterIdx <= MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
  {
    if ( filterIdx < MAX_NUM_CC_ALF_FILTERS)
    {
      memset( m_bestFilterCoeffSet[filterIdx], 0, sizeof(m_bestFilterCoeffSet[filterIdx]) );
      bestMapFilterIdxToFilterIdc[filterIdx] = filterIdx + 1;
    }
    else
    {
      bestMapFilterIdxToFilterIdc[filterIdx] = 0;
    }
  }
  memset(m_bestFilterControl, 0, sizeof(uint8_t) * m_numCTUsInPic);
  int ccalfReuseApsId      = -1;
  m_reuseApsId[compID - 1] = -1;

  const TempCtx ctxStartCcAlfFilterControlFlag  ( m_CtxCache, SubCtx( Ctx::CcAlfFilterControlFlag, m_CABACEstimator->getCtx() ) );

  // compute cost of not filtering
  uint64_t unfilteredDistortion = 0;
  for (int ctbIdx = 0; ctbIdx < m_numCTUsInPic; ctbIdx++)
  {
    unfilteredDistortion += (uint64_t)m_alfCovarianceCcAlf[compID - 1][0][0][ctbIdx].pixAcc;
  }
  double bestUnfilteredTotalCost = 1 * m_lambda[compID] + unfilteredDistortion;   // 1 bit is for gating flag

  bool  ccAlfFilterIdxEnabled[MAX_NUM_CC_ALF_FILTERS];
  short ccAlfFilterCoeff[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF];
  uint8_t ccAlfFilterCount     = MAX_NUM_CC_ALF_FILTERS;
  double bestFilteredTotalCost        = MAX_DOUBLE;
  bool   bestreuseTemporalFilterCoeff = false;
  std::vector<int> apsIds             = getAvailableCcAlfApsIds(cs, compID);
  const double invFactor = 1.0 / (double)(1 << m_scaleBits );

  for (int testFilterIdx = 0; testFilterIdx < ( apsIds.size() + 1 ); testFilterIdx++ )
  {
    bool referencingExistingAps   = (testFilterIdx < apsIds.size()) ? true : false;
    int maxNumberOfFiltersBeingTested = MAX_NUM_CC_ALF_FILTERS - (testFilterIdx - static_cast<int>(apsIds.size()));

    if (maxNumberOfFiltersBeingTested < 0)
    {
      maxNumberOfFiltersBeingTested = 1;
    }

    {
      // Instead of rewriting the control buffer for every training iteration just keep a mapping from filterIdx to filterIdc
      uint8_t mapFilterIdxToFilterIdc[MAX_NUM_CC_ALF_FILTERS + 1];
      for (int filterIdx = 0; filterIdx <= MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        if (filterIdx == MAX_NUM_CC_ALF_FILTERS)
        {
          mapFilterIdxToFilterIdc[filterIdx] = 0;
        }
        else
        {
          mapFilterIdxToFilterIdc[filterIdx] = filterIdx + 1;
        }
      }

      // initialize filters
      for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
      {
        ccAlfFilterIdxEnabled[filterIdx] = false;
        memset(ccAlfFilterCoeff[filterIdx], 0, sizeof(ccAlfFilterCoeff[filterIdx]));
      }
      if ( referencingExistingAps )
      {
        maxNumberOfFiltersBeingTested = m_apsMap->getPS((apsIds[testFilterIdx] << NUM_APS_TYPE_LEN) + ALF_APS)->ccAlfParam.ccAlfFilterCount[compID - 1];
        ccAlfFilterCount = maxNumberOfFiltersBeingTested;
        for (int filterIdx = 0; filterIdx < maxNumberOfFiltersBeingTested; filterIdx++)
        {
          ccAlfFilterIdxEnabled[filterIdx] = true;
          memcpy(ccAlfFilterCoeff[filterIdx], m_ccAlfFilterParam.ccAlfCoeff[compID - 1][filterIdx],
                 sizeof(ccAlfFilterCoeff[filterIdx]));
        }
        memcpy( ccAlfFilterCoeff, m_apsMap->getPS((apsIds[testFilterIdx] << NUM_APS_TYPE_LEN) + ALF_APS)->ccAlfParam.ccAlfCoeff[compID - 1], sizeof(ccAlfFilterCoeff) );
      }
      else
      {
        for (int i = 0; i < maxNumberOfFiltersBeingTested; i++)
        {
          ccAlfFilterIdxEnabled[i] = true;
        }
        ccAlfFilterCount = maxNumberOfFiltersBeingTested;
      }

      // initialize
      int controlIdx = 0;
      const int columnSize = ( m_buf->width / maxNumberOfFiltersBeingTested);
      for (int y = 0; y < m_buf->height; y += ctuHeightC)
      {
        for (int x = 0; x < m_buf->width; x += ctuWidthC)
        {
          m_trainingCovControl[controlIdx] = ( x / columnSize ) + 1;
          controlIdx++;
        }
      }

      // compute cost of filtering
      int    trainingIterCount = 0;
      bool   keepTraining      = true;
      bool   improvement       = false;
      double prevTotalCost     = MAX_DOUBLE;
      while (keepTraining)
      {
        improvement = false;
        for (int filterIdx = 0; filterIdx < maxNumberOfFiltersBeingTested; filterIdx++)
        {
          if (ccAlfFilterIdxEnabled[filterIdx])
          {
            if (!referencingExistingAps)
            {
              getFrameStatsCcalf(compID, (filterIdx + 1));
              deriveCcAlfFilterCoeff(compID, dstYuv, tempDecYuvBuf, ccAlfFilterCoeff, filterIdx);
            }
            const int numCoeff  = m_filterShapesCcAlf[compID - 1][0].numCoeff - 1;
            int log2BlockWidth  = cs.pcv->maxCUSizeLog2 - scaleX;
            int log2BlockHeight = cs.pcv->maxCUSizeLog2 - scaleY;
            for (int y = 0; y < m_buf->height; y += (1 << log2BlockHeight))
            {
              for (int x = 0; x < m_buf->width; x += (1 << log2BlockWidth))
              {
                int ctuIdx = (y >> log2BlockHeight) * m_numCTUsInWidth + (x >> log2BlockWidth);
                m_trainingDistortion[filterIdx][ctuIdx] =
                  int(m_ctbDistortionUnfilter[compID][ctuIdx]
                      + m_alfCovarianceCcAlf[compID - 1][0][0][ctuIdx].calcErrorForCcAlfCoeffs(
                        ccAlfFilterCoeff[filterIdx], numCoeff, invFactor));
              }
            }
          }
        }

        m_CABACEstimator->getCtx() = ctxStartCcAlfFilterControlFlag;

        uint64_t curTotalDistortion = 0;
        double curTotalRate = 0;
        determineControlIdcValues(cs, compID, m_buf, ctuWidthC, ctuHeightC, picWidthC, picHeightC,
                                  m_ctbDistortionUnfilter, m_trainingDistortion,
                                  m_lumaSwingGreaterThanThresholdCount,
                                  m_chromaSampleCountNearMidPoint,
                                  (referencingExistingAps == true),
                                  m_trainingCovControl, m_filterControl, curTotalDistortion, curTotalRate,
                                  ccAlfFilterIdxEnabled, mapFilterIdxToFilterIdc, ccAlfFilterCount);

        // compute coefficient coding bit cost
        if (ccAlfFilterCount > 0)
        {
          if (referencingExistingAps)
          {
            curTotalRate += 1 + 3; // +1 for enable flag, +3 APS ID in slice header
          }
          else
          {
            curTotalRate += getCoeffRateCcAlf(ccAlfFilterCoeff, ccAlfFilterIdxEnabled, ccAlfFilterCount, compID) + 1
            + 9;   // +1 for the enable flag, +9  3-bit for APS ID, one in slice header, 5-bit for APS ID in APS, a 1-bit
            // new filter flags (ignore shared cost such as other new-filter flags/NALU header/RBSP
            // terminating bit/byte alignment bits)
          }

          double curTotalCost = curTotalRate * m_lambda[compID] + curTotalDistortion;

          if (curTotalCost < prevTotalCost)
          {
            prevTotalCost = curTotalCost;
            improvement = true;
          }

          if (curTotalCost < bestFilteredTotalCost)
          {
            bestFilteredTotalCost = curTotalCost;
            memcpy(m_bestFilterIdxEnabled, ccAlfFilterIdxEnabled, sizeof(ccAlfFilterIdxEnabled));
            memcpy(m_bestFilterCoeffSet, ccAlfFilterCoeff, sizeof(ccAlfFilterCoeff));
            memcpy(m_bestFilterControl, m_filterControl, sizeof(uint8_t) * m_numCTUsInPic);
            m_bestFilterCount = ccAlfFilterCount;
            ccalfReuseApsId = referencingExistingAps ? apsIds[testFilterIdx] : -1;
            memcpy(bestMapFilterIdxToFilterIdc, mapFilterIdxToFilterIdc, sizeof(mapFilterIdxToFilterIdc));
          }
        }

        trainingIterCount++;
        if (!improvement || trainingIterCount > maxTrainingIterCount || referencingExistingAps)
        {
          keepTraining = false;
        }
      }
    }
  }

  if (bestUnfilteredTotalCost < bestFilteredTotalCost)
  {
    memset(m_bestFilterControl, 0, sizeof(uint8_t) * m_numCTUsInPic);
  }

  // save best coeff and control
  bool atleastOneBlockUndergoesFitlering = false;
  for (int controlIdx = 0; m_bestFilterCount > 0 && controlIdx < m_numCTUsInPic; controlIdx++)
  {
    if (m_bestFilterControl[controlIdx])
    {
      atleastOneBlockUndergoesFitlering = true;
      break;
    }
  }
  m_ccAlfFilterParam.numberValidComponents          = getNumberValidComponents(m_chromaFormat);
  m_ccAlfFilterParam.ccAlfFilterEnabled[compID - 1] = atleastOneBlockUndergoesFitlering;
  if (atleastOneBlockUndergoesFitlering)
  {
    // update the filter control indicators
    if (bestreuseTemporalFilterCoeff!=1)
    {
      short storedBestFilterCoeffSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF];
      for (int filterIdx=0; filterIdx<MAX_NUM_CC_ALF_FILTERS; filterIdx++)
      {
        memcpy(storedBestFilterCoeffSet[filterIdx], m_bestFilterCoeffSet[filterIdx], sizeof(m_bestFilterCoeffSet[filterIdx]));
      }
      memcpy(m_filterControl, m_bestFilterControl, sizeof(uint8_t) * m_numCTUsInPic);

      int filterCount = 0;
      for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
      {
        uint8_t curFilterIdc = bestMapFilterIdxToFilterIdc[filterIdx];
        if (m_bestFilterIdxEnabled[filterIdx])
        {
          for (int controlIdx = 0; controlIdx < m_numCTUsInPic; controlIdx++)
          {
            if (m_filterControl[controlIdx] == (filterIdx+1) )
            {
              m_bestFilterControl[controlIdx] = curFilterIdc;
            }
          }
          memcpy( m_bestFilterCoeffSet[curFilterIdc-1], storedBestFilterCoeffSet[filterIdx], sizeof(storedBestFilterCoeffSet[filterIdx]) );
          filterCount++;
        }
        m_bestFilterIdxEnabled[filterIdx] = ( filterIdx < m_bestFilterCount ) ? true : false;
      }
      CHECK( filterCount != m_bestFilterCount, "Number of filters enabled did not match the filter count");
    }

    m_ccAlfFilterParam.ccAlfFilterCount[compID - 1] = m_bestFilterCount;
    // cleanup before copying
    memset(m_ccAlfFilterControl[compID - 1], 0, sizeof(uint8_t) * m_numCTUsInPic);
    for ( int filterIdx = 0; filterIdx < MAX_NUM_CC_ALF_FILTERS; filterIdx++ )
    {
      memset(m_ccAlfFilterParam.ccAlfCoeff[compID - 1][filterIdx], 0,
             sizeof(m_ccAlfFilterParam.ccAlfCoeff[compID - 1][filterIdx]));
    }
    memset(m_ccAlfFilterParam.ccAlfFilterIdxEnabled[compID - 1], false,
           sizeof(m_ccAlfFilterParam.ccAlfFilterIdxEnabled[compID - 1]));
    for ( int filterIdx = 0; filterIdx < m_bestFilterCount; filterIdx++ )
    {
      m_ccAlfFilterParam.ccAlfFilterIdxEnabled[compID - 1][filterIdx] = m_bestFilterIdxEnabled[filterIdx];
      memcpy(m_ccAlfFilterParam.ccAlfCoeff[compID - 1][filterIdx], m_bestFilterCoeffSet[filterIdx],
             sizeof(m_bestFilterCoeffSet[filterIdx]));
    }
    memcpy(m_ccAlfFilterControl[compID - 1], m_bestFilterControl, sizeof(uint8_t) * m_numCTUsInPic);
    if ( ccalfReuseApsId >= 0 )
    {
      m_reuseApsId[compID - 1] = ccalfReuseApsId;
      if (compID == COMP_Cb)
      {
        cs.slice->ccAlfCbApsId = ccalfReuseApsId;
      }
      else
      {
        cs.slice->ccAlfCrApsId = ccalfReuseApsId;
      }
    }
  }
}

void EncAdaptiveLoopFilter::deriveStatsForCcAlfFilteringCTU( CodingStructure& cs, const int compIdx, int ctuRsAddr )
{
  const int filterIdx = 0;

  // init CTU stats buffers
  for( int shape = 0; shape != m_filterShapesCcAlf[compIdx - 1].size(); shape++ )
  {
      m_alfCovarianceCcAlf[compIdx - 1][shape][filterIdx][ctuRsAddr].reset();
  }

  const PreCalcValues& pcv = *cs.pcv;
  bool                 clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int                  numHorVirBndry = 0, numVerVirBndry = 0;
  int                  horVirBndryPos[] = { 0, 0, 0 };
  int                  verVirBndryPos[] = { 0, 0, 0 };

  const int xPos = ( ctuRsAddr % pcv.widthInCtus ) << pcv.maxCUSizeLog2;
  const int yPos = ( ctuRsAddr / pcv.widthInCtus ) << pcv.maxCUSizeLog2;
  const int width = ( xPos + m_maxCUWidth > m_picWidth ) ? ( m_picWidth - xPos ) : m_maxCUWidth;
  const int height = ( yPos + m_maxCUHeight > m_picHeight ) ? ( m_picHeight - yPos ) : m_maxCUHeight;
  int       rasterSliceAlfPad = 0;

  const PelUnitBuf orgYuv = cs.picture->getOrigBuf();
  const PelUnitBuf recYuv = PelUnitBuf( m_chromaFormat, m_tempBuf.getBuf( COMP_Y ), cs.getRecoBuf( COMP_Cb ), cs.getRecoBuf( COMP_Cr ) );

  if( isCrossedByVirtualBoundaries( cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight,
    numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos,
    rasterSliceAlfPad ) )
  {
    int yStart = yPos;
    for( int i = 0; i <= numHorVirBndry; i++ )
    {
      const int  yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
      const int  h = yEnd - yStart;
      const bool clipT = ( i == 0 && clipTop ) || ( i > 0 ) || ( yStart == 0 );
      const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );
      int        xStart = xPos;
      for( int j = 0; j <= numVerVirBndry; j++ )
      {
        const int  xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
        const int  w = xEnd - xStart;
        const bool clipL = ( j == 0 && clipLeft ) || ( j > 0 ) || ( xStart == 0 );
        const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );
        const int  wBuf = w + ( clipL ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipR ? 0 : MAX_ALF_PADDING_SIZE );
        const int  hBuf = h + ( clipT ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipB ? 0 : MAX_ALF_PADDING_SIZE );
        PelUnitBuf recBuf = m_tempBuf2.subBuf( UnitArea( cs.area.chromaFormat, Area( 0, 0, wBuf, hBuf ) ) );
        recBuf.copyFrom( recYuv.subBuf(
          UnitArea( cs.area.chromaFormat, Area( xStart - ( clipL ? 0 : MAX_ALF_PADDING_SIZE ),
            yStart - ( clipT ? 0 : MAX_ALF_PADDING_SIZE ), wBuf, hBuf ) ) ) );
        // pad top-left unavailable samples for raster slice
        if( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
        {
          recBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 1 );
        }

        // pad bottom-right unavailable samples for raster slice
        if( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
        {
          recBuf.padBorderPel( MAX_ALF_PADDING_SIZE, 2 );
        }
        recBuf.extendBorderPel( MAX_ALF_PADDING_SIZE );
        recBuf = recBuf.subBuf( UnitArea(
          cs.area.chromaFormat, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );

        const UnitArea area( m_chromaFormat, Area( 0, 0, w, h ) );
        const UnitArea areaDst( m_chromaFormat, Area( xStart, yStart, w, h ) );

        const ComponentID compID = ComponentID( compIdx );

        for( int shape = 0; shape != m_filterShapesCcAlf[compIdx - 1].size(); shape++ )
        {
          getBlkStatsCcAlf( m_alfCovarianceCcAlf[compIdx - 1][0][filterIdx][ctuRsAddr],
            m_filterShapesCcAlf[compIdx - 1][shape], orgYuv, recBuf, areaDst, area, compID, yPos );
        }

        xStart = xEnd;
      }

      yStart = yEnd;
    }
  }
  else
  {
    const UnitArea area( m_chromaFormat, Area( xPos, yPos, width, height ) );

    const ComponentID compID = ComponentID( compIdx );

    for( int shape = 0; shape != m_filterShapesCcAlf[compIdx - 1].size(); shape++ )
    {
      getBlkStatsCcAlf( m_alfCovarianceCcAlf[compIdx - 1][0][filterIdx][ctuRsAddr],
        m_filterShapesCcAlf[compIdx - 1][shape], orgYuv, recYuv, area, area, compID, yPos );
    }
  }
}

void EncAdaptiveLoopFilter::getBlkStatsCcAlf(AlfCovariance &alfCovariance, const AlfFilterShape &shape,
                                             const PelUnitBuf &orgYuv, const PelUnitBuf &recYuv,
                                             const UnitArea &areaDst, const UnitArea &area, const ComponentID compID,
                                             const int yPos)
{
  const CompArea &compArea = areaDst.block(compID);

  int        recStride = recYuv.get(COMP_Y).stride;
  const Pel* rec       = recYuv.get(COMP_Y).bufAt(area.lumaPos());
  
  int        slfStride = recYuv.get(compID).stride;
  const Pel* slf       = recYuv.get(compID).bufAt(area.chromaPos());

  int        orgStride = orgYuv.get(compID).stride;
  const Pel *org       = orgYuv.get(compID).bufAt(compArea);

  int vbCTUHeight = m_alfVBLumaCTUHeight;
  int vbPos       = m_alfVBLumaPos;

  if ((yPos + m_maxCUHeight) >= m_picHeight)
  {
    vbPos = m_picHeight;
  }

  CHECK( ( compArea.width  & 3 ) != 0, "Area width has to be a multiple of 4!" );
  CHECK( ( compArea.height & 3 ) != 0, "Area width has to be a multiple of 4!" );
  CHECK( isLuma( compID ), "Only chroma can be analysed in CCALF!" );

  int effStride = recStride << getComponentScaleY(compID, m_chromaFormat);

  Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][16];
  Pel yLocal[4][4];
  double weight[4][4];

  for (int i = 0; i < compArea.height; i += 4)
  {
    int vbDistance0 = (((i+0) << getComponentScaleX(compID, m_chromaFormat)) % vbCTUHeight) - vbPos;
    int vbDistance1 = (((i+1) << getComponentScaleX(compID, m_chromaFormat)) % vbCTUHeight) - vbPos;
    int vbDistance2 = (((i+2) << getComponentScaleX(compID, m_chromaFormat)) % vbCTUHeight) - vbPos;
    int vbDistance3 = (((i+3) << getComponentScaleX(compID, m_chromaFormat)) % vbCTUHeight) - vbPos;

    const Pel* recLine[4] = { &rec[0], &rec[effStride], &rec[2 * effStride], &rec[3 * effStride] };
    const Pel* orgLine[4] = { &org[0], &org[orgStride], &org[2 * orgStride], &org[3 * orgStride] };
    const Pel* slfLine[4] = { &slf[0], &slf[slfStride], &slf[2 * slfStride], &slf[3 * slfStride] };

    for (int j = 0; j < compArea.width; j += 4)
    {
      for (int ii = 0; ii < 4; ii++) for (int jj = 0; jj < 4; jj++)
      {
        yLocal[ii][jj] = orgLine[ii][j + jj] - slfLine[ii][j + jj];
      }

      calcCovariance4CcAlf(ELocal,  0, recLine[0] + (j << getChannelTypeScaleX(CH_C, m_chromaFormat)), recStride, shape, vbDistance0);
      calcCovariance4CcAlf(ELocal,  4, recLine[1] + (j << getChannelTypeScaleX(CH_C, m_chromaFormat)), recStride, shape, vbDistance1);
      calcCovariance4CcAlf(ELocal,  8, recLine[2] + (j << getChannelTypeScaleX(CH_C, m_chromaFormat)), recStride, shape, vbDistance2);
      calcCovariance4CcAlf(ELocal, 12, recLine[3] + (j << getChannelTypeScaleX(CH_C, m_chromaFormat)), recStride, shape, vbDistance3);

      if( m_alfWSSD )
      {
        for (int ii = 0; ii < 4; ii++) for (int jj = 0; jj < 4; jj++)
        {
          weight[ii][jj] = m_lumaLevelToWeightPLUT[orgLine[ii][j + jj]];
        }

        for( int k = 0; k < ( shape.numCoeff - 1 ); k++ )
        {
          for( int l = k; l < ( shape.numCoeff - 1 ); l++ )
          {
            double sum = 0;
            for (int ii = 0; ii < 4; ii++) for (int jj = 0; jj < 4; jj++)
            {
              sum += weight[ii][jj] * int( ELocal[k][(ii << 2) + jj] ) * ELocal[l][(ii << 2) + jj];
            }
            alfCovariance.E[0][0][k][l] += sum;
          }

          double sum = 0;
          for (int ii = 0; ii < 4; ii++) for (int jj = 0; jj < 4; jj++)
          {
            sum += weight[ii][jj] * int( ELocal[k][(ii << 2) + jj] ) * yLocal[ii][jj];
          }
          alfCovariance.y[0][k] += sum;
        } 


        double sum = 0;
        for (int ii = 0; ii < 4; ii++) for (int jj = 0; jj < 4; jj++)
        {
          sum += weight[ii][jj] * int( yLocal[ii][jj] ) * yLocal[ii][jj];
        }
        alfCovariance.pixAcc += sum;
      }
      else
      {
#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT_ALF
        if (read_x86_extension_flags() > SCALAR)
        {
          const __m128i mylocal0 = _mm_loadu_si128((const __m128i*) & yLocal[0][0]);
          const __m128i mylocal8 = _mm_loadu_si128((const __m128i*) & yLocal[2][0]);

          for (int k = 0; k < shape.numCoeff; k++)
          {
            const Pel* Elocalk = &ELocal[k][0];
            alf_float_t* cov = &alfCovariance.E[0][0][k][k];

            const __m128i melocalk0 = _mm_loadu_si128((const __m128i*) & Elocalk[0]);
            const __m128i melocalk8 = _mm_loadu_si128((const __m128i*) & Elocalk[8]);

            for (int l = k; l < shape.numCoeff; l++)
            {
              const Pel* Elocall = &ELocal[l][0];

              const __m128i melocall0 = _mm_loadu_si128((const __m128i*) & Elocall[0]);
              const __m128i melocall8 = _mm_loadu_si128((const __m128i*) & Elocall[8]);

              const __m128i mmacc0 = _mm_madd_epi16(melocalk0, melocall0);
              const __m128i mmacc8 = _mm_madd_epi16(melocalk8, melocall8);

              __m128i mmacc = _mm_add_epi32(mmacc0, mmacc8);
              mmacc = _mm_hadd_epi32(mmacc, mmacc);
              mmacc = _mm_hadd_epi32(mmacc, mmacc);

              *cov++ += _mm_cvtsi128_si32(mmacc);
            }

            const __m128i mmacc0 = _mm_madd_epi16(melocalk0, mylocal0);
            const __m128i mmacc8 = _mm_madd_epi16(melocalk8, mylocal8);

            __m128i mmacc = _mm_add_epi32(mmacc0, mmacc8);
            mmacc = _mm_hadd_epi32(mmacc, mmacc);
            mmacc = _mm_hadd_epi32(mmacc, mmacc);

            alfCovariance.y[0][k] += _mm_cvtsi128_si32(mmacc);
          }

          const __m128i mmacc0 = _mm_madd_epi16(mylocal0, mylocal0);
          const __m128i mmacc8 = _mm_madd_epi16(mylocal8, mylocal8);

          __m128i mmacc = _mm_add_epi32(mmacc0, mmacc8);
          mmacc = _mm_hadd_epi32(mmacc, mmacc);
          mmacc = _mm_hadd_epi32(mmacc, mmacc);

          alfCovariance.pixAcc += _mm_cvtsi128_si32(mmacc);
        }
        else
#endif
        {
          for( int k = 0; k < ( shape.numCoeff - 1 ); k++ )
          {
            for( int l = k; l < ( shape.numCoeff - 1 ); l++ )
            {
              int sum = 0;
              for (int ii = 0; ii < 4; ii++) for (int jj = 0; jj < 4; jj++)
              {
                sum += int( ELocal[k][(ii << 2) + jj] ) * ELocal[l][(ii << 2) + jj];
              }
              alfCovariance.E[0][0][k][l] += sum;
            }

            int sum = 0;
            for (int ii = 0; ii < 4; ii++) for (int jj = 0; jj < 4; jj++)
            {
              sum += int( ELocal[k][(ii << 2) + jj] ) * yLocal[ii][jj];
            }
            alfCovariance.y[0][k] += sum;
          } 


          int sum = 0;
          for (int ii = 0; ii < 4; ii++) for (int jj = 0; jj < 4; jj++)
          {
            sum += int( yLocal[ii][jj] ) * yLocal[ii][jj];
          }
          alfCovariance.pixAcc += sum;
        }
      }
    }
    
    slf += (slfStride << 2);
    org += (orgStride << 2);
    rec += (effStride << 2);
  }

  for (int k = 1; k < (MAX_NUM_CC_ALF_CHROMA_COEFF - 1); k++)
  {
    for (int l = 0; l < k; l++)
    {
      alfCovariance.E[0][0][k][l] = alfCovariance.E[0][0][l][k];
    }
  }
}

void EncAdaptiveLoopFilter::calcCovariance4CcAlf(Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF][16], const int N, const Pel* rec, const int stride, const AlfFilterShape& shape, int vbDistance)
{
  CHECK(shape.filterType != CC_ALF, "Bad CC ALF shape");

  const Pel* recYM1 = rec - 1 * stride;
  const Pel* recY0  = rec;
  const Pel* recYP1 = rec + 1 * stride;
  const Pel* recYP2 = rec + 2 * stride;

  if (vbDistance == -2 || vbDistance == +1)
  {
    recYP2 = recYP1;
  }
  else if (vbDistance == -1 || vbDistance == 0)
  {
    recYM1 = recY0;
    recYP2 = recYP1 = recY0;
}

  const int dx0 = 0;
  const int dx1 = 1 << getChannelTypeScaleX( CH_C, m_chromaFormat );
  const int dx2 = dx1 << 1;
  const int dx3 = dx1 + dx2;

  const Pel centerValue0 = recY0[+dx0];
  const Pel centerValue1 = recY0[+dx1];
  const Pel centerValue2 = recY0[+dx2];
  const Pel centerValue3 = recY0[+dx3];

  ELocal[0][N+0] = recYM1[+0+dx0] - centerValue0;
  ELocal[0][N+1] = recYM1[+0+dx1] - centerValue1;
  ELocal[0][N+2] = recYM1[+0+dx2] - centerValue2;
  ELocal[0][N+3] = recYM1[+0+dx3] - centerValue3;
                 
  ELocal[1][N+0] = recY0[ -1+dx0] - centerValue0;
  ELocal[1][N+1] = recY0[ -1+dx1] - centerValue1;
  ELocal[1][N+2] = recY0[ -1+dx2] - centerValue2;
  ELocal[1][N+3] = recY0[ -1+dx3] - centerValue3;
                 
  ELocal[2][N+0] = recY0[ +1+dx0] - centerValue0;
  ELocal[2][N+1] = recY0[ +1+dx1] - centerValue1;
  ELocal[2][N+2] = recY0[ +1+dx2] - centerValue2;
  ELocal[2][N+3] = recY0[ +1+dx3] - centerValue3;
                 
  ELocal[3][N+0] = recYP1[-1+dx0] - centerValue0;
  ELocal[3][N+1] = recYP1[-1+dx1] - centerValue1;
  ELocal[3][N+2] = recYP1[-1+dx2] - centerValue2;
  ELocal[3][N+3] = recYP1[-1+dx3] - centerValue3;
                 
  ELocal[4][N+0] = recYP1[+0+dx0] - centerValue0;
  ELocal[4][N+1] = recYP1[+0+dx1] - centerValue1;
  ELocal[4][N+2] = recYP1[+0+dx2] - centerValue2;
  ELocal[4][N+3] = recYP1[+0+dx3] - centerValue3;
                 
  ELocal[5][N+0] = recYP1[+1+dx0] - centerValue0;
  ELocal[5][N+1] = recYP1[+1+dx1] - centerValue1;
  ELocal[5][N+2] = recYP1[+1+dx2] - centerValue2;
  ELocal[5][N+3] = recYP1[+1+dx3] - centerValue3;
                 
  ELocal[6][N+0] = recYP2[+0+dx0] - centerValue0;
  ELocal[6][N+1] = recYP2[+0+dx1] - centerValue1;
  ELocal[6][N+2] = recYP2[+0+dx2] - centerValue2;
  ELocal[6][N+3] = recYP2[+0+dx3] - centerValue3;
}

void EncAdaptiveLoopFilter::calcCovarianceCcAlf(Pel ELocal[MAX_NUM_CC_ALF_CHROMA_COEFF], const Pel *rec, const int stride, const AlfFilterShape& shape, int vbDistance)
{
  CHECK(shape.filterType != CC_ALF, "Bad CC ALF shape");

  const Pel *recYM1 = rec - 1 * stride;
  const Pel *recY0  = rec;
  const Pel *recYP1 = rec + 1 * stride;
  const Pel *recYP2 = rec + 2 * stride;

  if (vbDistance == -2 || vbDistance == +1)
  {
    recYP2 = recYP1;
  }
  else if (vbDistance == -1 || vbDistance == 0)
  {
    recYM1 = recY0;
    recYP2 = recYP1 = recY0;
  }

    const Pel centerValue = recY0[+0];

  ELocal[0] += recYM1[+0] - centerValue;
  ELocal[1] += recY0 [-1] - centerValue;
  ELocal[2] += recY0 [+1] - centerValue;
  ELocal[3] += recYP1[-1] - centerValue;
  ELocal[4] += recYP1[+0] - centerValue;
  ELocal[5] += recYP1[+1] - centerValue;
  ELocal[6] += recYP2[+0] - centerValue;
  }

void EncAdaptiveLoopFilter::countLumaSwingGreaterThanThreshold(const Pel* luma, int lumaStride, int height, int width, int log2BlockWidth, int log2BlockHeight, uint64_t* lumaSwingGreaterThanThresholdCount, int lumaCountStride)
{
  const int lumaBitDepth = m_inputBitDepth[CH_L];
  const int threshold = (1 << ( m_inputBitDepth[CH_L] - 2 )) - 1;

  // 3x4 Diamond
  int xSupport[] = {  0, -1, 0, 1, -1, 0, 1, 0 };
  int ySupport[] = { -1,  0, 0, 0,  1, 1, 1, 2 };

  for (int y = 0; y < height; y += (1 << log2BlockHeight))
  {
    for (int x = 0; x < width; x += (1 << log2BlockWidth))
    {
      lumaSwingGreaterThanThresholdCount[(y >> log2BlockHeight) * lumaCountStride + (x >> log2BlockWidth)] = 0;

      for (int yOff = 0; yOff < (1 << log2BlockHeight); yOff++)
      {
        for (int xOff = 0; xOff < (1 << log2BlockWidth); xOff++)
        {
          if ((y + yOff) >= (height - 2) || (x + xOff) >= (width - 1) || (y + yOff) < 1 || (x + xOff) < 1) // only consider samples that are fully supported by picture
          {
            continue;
          }

          int minVal = ((1 << lumaBitDepth) - 1);
          int maxVal = 0;
          for (int i = 0; i < 8; i++)
          {
            Pel p = luma[(yOff + ySupport[i]) * lumaStride + x + xOff + xSupport[i]];

            if ( p < minVal )
            {
              minVal = p;
            }
            if ( p > maxVal )
            {
              maxVal = p;
            }
          }

          if ((maxVal - minVal) > threshold)
          {
            lumaSwingGreaterThanThresholdCount[(y >> log2BlockHeight) * lumaCountStride + (x >> log2BlockWidth)]++;
          }
        }
      }
    }
    luma += (lumaStride << log2BlockHeight);
  }
}

void EncAdaptiveLoopFilter::countChromaSampleValueNearMidPoint(const Pel* chroma, int chromaStride, int height, int width, int log2BlockWidth, int log2BlockHeight, uint64_t* chromaSampleCountNearMidPoint, int chromaSampleCountNearMidPointStride)
{
  const int midPoint  = (1 << m_inputBitDepth[CH_C]) >> 1;
  const int threshold = 16;

  for (int y = 0; y < height; y += (1 << log2BlockHeight))
  {
    for (int x = 0; x < width; x += (1 << log2BlockWidth))
    {
      chromaSampleCountNearMidPoint[(y >> log2BlockHeight)* chromaSampleCountNearMidPointStride + (x >> log2BlockWidth)] = 0;

      for (int yOff = 0; yOff < (1 << log2BlockHeight); yOff++)
      {
        for (int xOff = 0; xOff < (1 << log2BlockWidth); xOff++)
        {
          if ((y + yOff) >= height || (x + xOff) >= width)
          {
            continue;
          }

          int distanceToMidPoint = abs(chroma[yOff * chromaStride + x + xOff] - midPoint);
          if (distanceToMidPoint < threshold)
          {
            chromaSampleCountNearMidPoint[(y >> log2BlockHeight)* chromaSampleCountNearMidPointStride + (x >> log2BlockWidth)]++;
          }
        }
      }
    }
    chroma += (chromaStride << log2BlockHeight);
  }
}

} // namespace vvenc

//! \}

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

/**
 * \file EncAdaptiveLoopFilter_neon.cpp
 * \brief Neon helpers for encoder-side adaptive loop filter code.
 */

#include "EncoderLib/EncAdaptiveLoopFilter.h"
#include <arm_neon.h>

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_OPT_ALF

namespace vvenc
{

#define REG alf_float_t( 0.0001 )
#define REG_SQR alf_float_t( 0.0000001 )

template<int NumEq>
static bool checkForZero( const AlfCovariance::TE lhs, const alf_float_t* rhs )
{
  for( int i = 0; i < NumEq; i++ )
  {
    if( rhs[i] != 0.0f )
    {
      return false;
    }
    for( int j = 0; j < NumEq; j++ )
    {
      if( lhs[i][j] != 0.0f )
      {
        return false;
      }
    }
  }

  return true;
}

template<int NumEq>
static int gnsCholeskyDec_neon( const AlfCovariance::TE inpMatr, AlfCovariance::TE outMatr );

template<>
int gnsCholeskyDec_neon<7>( const AlfCovariance::TE inpMatr, AlfCovariance::TE outMatr )
{
  static constexpr int NumEq = 7;
  for( int i = 0; i < NumEq; i++ )
  {
    alf_float_t invDiag = 0.0f;

    for( int j = i; j < NumEq; j++ )
    {
      // Compute the scaling factor.
      alf_float_t scale = inpMatr[i][j];
      if( i > 0 )
      {
        for( int k = i - 1; k >= 0; k-- )
        {
          scale -= outMatr[k][j] * outMatr[k][i];
        }
      }

      // Compute i'th row of outMatr.
      if( i == j )
      {
        if( scale <= REG_SQR ) // If inpMatr is singular.
        {
          return 0;
        }
        else
        {
          // Normal operation.
          invDiag = alf_float_t( 1.0 ) / ( outMatr[i][i] = sqrtf( scale ) );
        }
      }
      else
      {
        outMatr[i][j] = scale * invDiag;    // Upper triangular part.
        outMatr[j][i] = alf_float_t( 0.0 ); // Lower triangular part set to 0.
      }
    }
  }
  return 1; // Signal that Cholesky factorization is successfully performed.
}

template<int Row>
static inline bool choleskyStep13( const AlfCovariance::TE inpMatr, AlfCovariance::TE outMatr )
{
  static constexpr int NumEq = 13;

  // Computes scale from inpMatr[Row][Row] and outMatr[0..Row-1][Row].
  alf_float_t scale = inpMatr[Row][Row];

  for( int k = Row - 1; k >= 0; --k )
  {
    scale -= outMatr[k][Row] * outMatr[k][Row];
  }

  if( scale <= REG_SQR )
  {
    return false;
  }

  // Updates outMatr[Row][Row] and outMatr[Row][Row+1..12].
  outMatr[Row][Row] = sqrtf( scale );
  const alf_float_t invDiag = alf_float_t( 1.0 ) / outMatr[Row][Row];

  if( Row % 4 == 0 )
  {
    for( int j = Row + 1; j < NumEq; j += 4 )
    {
      float32x4_t scale = vld1q_f32( &inpMatr[Row][j] );

      for( int k = Row - 1; k >= 0; k -= 4 )
      {
        const float32x4_t v1 = vld1q_f32( &outMatr[k][j] );
        const float32x4_t v2 = vld1q_f32( &outMatr[k - 1][j] );
        const float32x4_t v3 = vld1q_f32( &outMatr[k - 2][j] );
        const float32x4_t v4 = vld1q_f32( &outMatr[k - 3][j] );

        scale = vmlsq_n_f32( scale, v1, outMatr[k][Row] );
        scale = vmlsq_n_f32( scale, v2, outMatr[k - 1][Row] );
        scale = vmlsq_n_f32( scale, v3, outMatr[k - 2][Row] );
        scale = vmlsq_n_f32( scale, v4, outMatr[k - 3][Row] );
      }

      const float32x4_t out = vmulq_n_f32( scale, invDiag );
      vst1q_f32( &outMatr[Row][j], out );
    }
  }
  else if( Row % 2 == 0 )
  {
    for( int j = Row + 1; j < NumEq; j += 2 )
    {
      float32x2_t scale = vld1_f32( &inpMatr[Row][j] );

      for( int k = Row - 1; k >= 0; k -= 2 )
      {
        const float32x2_t v1 = vld1_f32( &outMatr[k][j] );
        const float32x2_t v2 = vld1_f32( &outMatr[k - 1][j] );

        scale = vmls_n_f32( scale, v1, outMatr[k][Row] );
        scale = vmls_n_f32( scale, v2, outMatr[k - 1][Row] );
      }

      const float32x2_t out = vmul_n_f32( scale, invDiag );
      vst1_f32( &outMatr[Row][j], out );
    }
  }
  else
  {
    for( int j = Row + 1; j < NumEq; j++ )
    {
      alf_float_t scale = inpMatr[Row][j];
      for( int k = Row - 1; k >= 0; k-- )
      {
        scale -= outMatr[k][j] * outMatr[k][Row];
      }
      outMatr[Row][j] = scale * invDiag;
    }
  }
  return true;
}

template<>
int gnsCholeskyDec_neon<13>( const AlfCovariance::TE inpMatr, AlfCovariance::TE outMatr )
{
  static constexpr int NumEq = 13;

  if( !choleskyStep13<0>( inpMatr, outMatr ) || !choleskyStep13<1>( inpMatr, outMatr ) ||
      !choleskyStep13<2>( inpMatr, outMatr ) || !choleskyStep13<3>( inpMatr, outMatr ) ||
      !choleskyStep13<4>( inpMatr, outMatr ) || !choleskyStep13<5>( inpMatr, outMatr ) ||
      !choleskyStep13<6>( inpMatr, outMatr ) || !choleskyStep13<7>( inpMatr, outMatr ) ||
      !choleskyStep13<8>( inpMatr, outMatr ) || !choleskyStep13<9>( inpMatr, outMatr ) ||
      !choleskyStep13<10>( inpMatr, outMatr ) || !choleskyStep13<11>( inpMatr, outMatr ) ||
      !choleskyStep13<12>( inpMatr, outMatr ) )
  {
    return 0;
  }

  // Lower triangular part set to 0.
  for( int i = 1; i < NumEq; ++i )
  {
    for( int j = 0; j < i; ++j )
    {
      outMatr[i][j] = alf_float_t( 0.0 );
    }
  }

  return 1; // Signal that Cholesky factorization is successfully performed.
}

template<int order>
static void gnsTransposeBacksubstitution_neon( const AlfCovariance::TE U, const alf_float_t* rhs, alf_float_t* x );

template<>
void gnsTransposeBacksubstitution_neon<7>( const AlfCovariance::TE U, const alf_float_t* rhs, alf_float_t* x )
{
  x[0] = rhs[0] / U[0][0];
  alf_float_t s1 = x[0] * U[0][1];
  alf_float_t s2 = x[0] * U[0][2];
  alf_float_t s3 = x[0] * U[0][3];
  alf_float_t s4 = x[0] * U[0][4];
  alf_float_t s5 = x[0] * U[0][5];
  alf_float_t s6 = x[0] * U[0][6];

  // sum = x[0] * U[0][1];
  x[1] = ( rhs[1] - s1 ) / U[1][1];
  s2 += x[1] * U[1][2];
  s3 += x[1] * U[1][3];
  s4 += x[1] * U[1][4];
  s5 += x[1] * U[1][5];
  s6 += x[1] * U[1][6];

  // sum = x[0] * U[0][2] + x[1] * U[1][2];
  x[2] = ( rhs[2] - s2 ) / U[2][2];
  s3 += x[2] * U[2][3];
  s4 += x[2] * U[2][4];
  s5 += x[2] * U[2][5];
  s6 += x[2] * U[2][6];

  // sum = x[0] * U[0][3] + x[1] * U[1][3] + x[2] * U[2][3];
  x[3] = ( rhs[3] - s3 ) / U[3][3];
  s4 += x[3] * U[3][4];
  s5 += x[3] * U[3][5];
  s6 += x[3] * U[3][6];

  // sum = x[0] * U[0][4] + x[1] * U[1][4] + x[2] * U[2][4] + x[3] * U[3][4];
  x[4] = ( rhs[4] - s4 ) / U[4][4];
  s5 += x[4] * U[4][5];
  s6 += x[4] * U[4][6];

  // sum = x[0] * U[0][5] + x[1] * U[1][5] + x[2] * U[2][5] + x[3] * U[3][5] + x[4] * U[4][5];
  x[5] = ( rhs[5] - s5 ) / U[5][5];
  s6 += x[5] * U[5][6];

  // sum = x[0] * U[0][6] + x[1] * U[1][6] + x[2] * U[2][6] + x[3] * U[3][6] + x[4] * U[4][6] + x[5] * U[5][6];
  x[6] = ( rhs[6] - s6 ) / U[6][6];
}

static inline void accum4( float32x4_t& acc, const alf_float_t* u, const alf_float_t x )
{
  const float32x4_t uv = vld1q_f32( u );
  acc = vmlaq_n_f32( acc, uv, x );
}

template<>
void gnsTransposeBacksubstitution_neon<13>( const AlfCovariance::TE U, const alf_float_t* rhs, alf_float_t* x )
{
  float32x4_t sum1 = vdupq_n_f32( 0.0f );
  float32x4_t sum2 = vdupq_n_f32( 0.0f );
  float32x4_t sum3 = vdupq_n_f32( 0.0f );

  x[0] = rhs[0] / U[0][0];
  accum4( sum1, &U[0][1], x[0] );
  accum4( sum2, &U[0][5], x[0] );
  accum4( sum3, &U[0][9], x[0] );

  alf_float_t s1 = vgetq_lane_f32( sum1, 0 );
  alf_float_t s2 = vgetq_lane_f32( sum1, 1 );
  alf_float_t s3 = vgetq_lane_f32( sum1, 2 );
  alf_float_t s4 = vgetq_lane_f32( sum1, 3 );

  // sum = x[0] * U[0][1];
  x[1] = ( rhs[1] - s1 ) / U[1][1];
  s2 += x[1] * U[1][2];
  s3 += x[1] * U[1][3];
  s4 += x[1] * U[1][4];
  accum4( sum2, &U[1][5], x[1] );
  accum4( sum3, &U[1][9], x[1] );

  // sum = x[0] * U[0][2] + x[1] * U[1][2];
  x[2] = ( rhs[2] - s2 ) / U[2][2];
  s3 += x[2] * U[2][3];
  s4 += x[2] * U[2][4];
  accum4( sum2, &U[2][5], x[2] );
  accum4( sum3, &U[2][9], x[2] );

  // sum = x[0] * U[0][3] + x[1] * U[1][3] + x[2] * U[2][3];
  x[3] = ( rhs[3] - s3 ) / U[3][3];
  s4 += x[3] * U[3][4];
  accum4( sum2, &U[3][5], x[3] );
  accum4( sum3, &U[3][9], x[3] );

  // sum = x[0] * U[0][4] + x[1] * U[1][4] + x[2] * U[2][4] + x[3] * U[3][4];
  x[4] = ( rhs[4] - s4 ) / U[4][4];
  accum4( sum2, &U[4][5], x[4] );
  accum4( sum3, &U[4][9], x[4] );

  alf_float_t s5 = vgetq_lane_f32( sum2, 0 );
  alf_float_t s6 = vgetq_lane_f32( sum2, 1 );
  alf_float_t s7 = vgetq_lane_f32( sum2, 2 );
  alf_float_t s8 = vgetq_lane_f32( sum2, 3 );

  // sum = x[0] * U[0][5] + x[1] * U[1][5] + x[2] * U[2][5] + x[3] * U[3][5] + x[4] * U[4][5];
  x[5] = ( rhs[5] - s5 ) / U[5][5];
  s6 += x[5] * U[5][6];
  s7 += x[5] * U[5][7];
  s8 += x[5] * U[5][8];
  accum4( sum3, &U[5][9], x[5] );

  // sum = x[0] * U[0][6] + x[1] * U[1][6] + x[2] * U[2][6] + x[3] * U[3][6] + x[4] * U[4][6] + x[5] * U[5][6];
  x[6] = ( rhs[6] - s6 ) / U[6][6];
  s7 += x[6] * U[6][7];
  s8 += x[6] * U[6][8];
  accum4( sum3, &U[6][9], x[6] );

  // sum = x[0] * U[0][7] + x[1] * U[1][7] + x[2] * U[2][7] + x[3] * U[3][7] + x[4] * U[4][7] + x[5] * U[5][7] +
  //             x[6] * U[6][7];
  x[7] = ( rhs[7] - s7 ) / U[7][7];
  s8 += x[7] * U[7][8];
  accum4( sum3, &U[7][9], x[7] );

  // sum = x[0] * U[0][8] + x[1] * U[1][8] + x[2] * U[2][8] + x[3] * U[3][8] + x[4] * U[4][8] + x[5] * U[5][8] +
  //             x[6] * U[6][8] + x[7] * U[7][8];
  x[8] = ( rhs[8] - s8 ) / U[8][8];
  accum4( sum3, &U[8][9], x[8] );

  alf_float_t s9 = vgetq_lane_f32( sum3, 0 );
  alf_float_t s10 = vgetq_lane_f32( sum3, 1 );
  alf_float_t s11 = vgetq_lane_f32( sum3, 2 );
  alf_float_t s12 = vgetq_lane_f32( sum3, 3 );

  // sum = x[0] * U[0][9] + x[1] * U[1][9] + x[2] * U[2][9] + x[3] * U[3][9] + x[4] * U[4][9] + x[5] * U[5][9] +
  //             x[6] * U[6][9] + x[7] * U[7][9] + x[8] * U[8][9];
  x[9] = ( rhs[9] - s9 ) / U[9][9];
  s10 += x[9] * U[9][10];
  s11 += x[9] * U[9][11];
  s12 += x[9] * U[9][12];

  // sum = x[0] * U[0][10] + x[1] * U[1][10] + x[2] * U[2][10] + x[3] * U[3][10] + x[4] * U[4][10] +
  //              x[5] * U[5][10] + x[6] * U[6][10] + x[7] * U[7][10] + x[8] * U[8][10] + x[9] * U[9][10];
  x[10] = ( rhs[10] - s10 ) / U[10][10];
  s11 += x[10] * U[10][11];
  s12 += x[10] * U[10][12];

  // sum = x[0] * U[0][11] + x[1] * U[1][11] + x[2] * U[2][11] + x[3] * U[3][11] + x[4] * U[4][11] +
  //              x[5] * U[5][11] + x[6] * U[6][11] + x[7] * U[7][11] + x[8] * U[8][11] + x[9] * U[9][11] +
  //              x[10] * U[10][11];
  x[11] = ( rhs[11] - s11 ) / U[11][11];
  s12 += x[11] * U[11][12];

  // sum = x[0] * U[0][12] + x[1] * U[1][12] + x[2] * U[2][12] + x[3] * U[3][12] + x[4] * U[4][12] +
  //              x[5] * U[5][12] + x[6] * U[6][12] + x[7] * U[7][12] + x[8] * U[8][12] + x[9] * U[9][12] +
  //              x[10] * U[10][12] + x[11] * U[11][12];
  x[12] = ( rhs[12] - s12 ) / U[12][12];
}

template<int NumEq>
static void gnsBacksubstitution( const AlfCovariance::TE R, const alf_float_t* z, alf_float_t* A )
{
  const int size = NumEq - 1;

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

template<int NumEq>
static int solveByChol_neon( AlfCovariance::TE lhs, const alf_float_t* rhs, alf_float_t* x )
{
  static_assert( NumEq == 7 || NumEq == 13, "Unsupported ALF Cholesky order" );

  AlfCovariance::Ty aux; // Auxiliary vector.
  AlfCovariance::TE U;   // Upper triangular Cholesky factor of lhs.

  int res = 1; // Signal that Cholesky factorization is successfully performed

  // The equation to be solved is LHSx = rhs.

  // Compute upper triangular U such that U'*U = LHS.
  if( gnsCholeskyDec_neon<NumEq>( lhs, U ) ) // If Cholesky decomposition has been successful.
  {
    // Now, the equation is  U'*U*x = rhs, where U is upper triangular
    // Solve U'*aux = rhs for aux.
    gnsTransposeBacksubstitution_neon<NumEq>( U, rhs, aux );

    // The equation is now U*x = aux, solve it for x (new motion coefficients).
    gnsBacksubstitution<NumEq>( U, aux, x );
  }
  else // lhs was singular.
  {
    // Check if lhs and rhs are all zero, in which the solution is also all zero.
    if( checkForZero<NumEq>( lhs, rhs ) )
    {
      std::memset( x, 0, sizeof( alf_float_t ) * NumEq );
      return 1;
    }

    // Regularize lhs.
    for( int i = 0; i < NumEq; i++ )
    {
      lhs[i][i] += REG;
    }

    // Compute upper triangular U such that U'*U = regularized lhs.
    res = gnsCholeskyDec_neon<NumEq>( lhs, U );

    if( !res )
    {
      std::memset( x, 0, sizeof( alf_float_t ) * NumEq );
      return 0;
    }

    // Solve  U'*aux = rhs for aux.
    gnsTransposeBacksubstitution_neon<NumEq>( U, rhs, aux );

    // Solve U*x = aux for x.
    gnsBacksubstitution<NumEq>( U, aux, x );
  }
  return res;
}

int gnsSolveByChol_neon( AlfCovariance::TE lhs, alf_float_t* rhs, alf_float_t* x, int numEq )
{
  if( numEq == 7 )
  {
    return solveByChol_neon<7>( lhs, rhs, x );
  }
  else if( numEq == 13 )
  {
    return solveByChol_neon<13>( lhs, rhs, x );
  }
  CHECK( true, "Unsupported ALF Cholesky order" );
  return 0;
}

template<>
void AlfCovariance::_initAlfCovarianceARM<NEON>()
{
  m_gnsSolveByChol = gnsSolveByChol_neon;
}

} // namespace vvenc

#endif

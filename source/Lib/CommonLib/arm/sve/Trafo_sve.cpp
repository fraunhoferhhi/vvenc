/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2024-2026, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
 * \file Trafo_sve.cpp
 * \brief SVE implementation of TCoeffOps for AArch64.
 */

//  ====================================================================================================================
//  Includes
//  ====================================================================================================================
#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"

#include "TrQuant.h"
#include "TrQuant_EMT.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_TRAFO

#include <arm_neon_sve_bridge.h>
#include <arm_sve.h>

namespace vvenc
{

static svint16_t load_narrow_to_s16( const int32_t* src )
{
  svint32_t lo = svld1_vnum_s32( svptrue_b32(), src, 0 );
  svint32_t hi = svld1_vnum_s32( svptrue_b32(), src, 1 );
  return svuzp1_s16( svreinterpret_s16_s32( lo ), svreinterpret_s16_s32( hi ) );
}

static int64_t shift_and_round( int64_t x, int shift )
{
  return ( x + ( 1 << ( shift - 1 ) ) ) >> shift;
}

template<int vlBits>
static inline void fastFwdCore_reduce_x4_sve( TCoeff* dst, svint64_t v0, svint64_t v1, svint64_t v2, svint64_t v3,
                                              int shift );

template<>
inline void fastFwdCore_reduce_x4_sve<128>( TCoeff* dst, svint64_t v0, svint64_t v1, svint64_t v2, svint64_t v3,
                                            int shift )
{
  // For a 128-bit vector length we do not need to reduce the sum down, use
  // svget_neonq to operate on the Neon vectors directly so we can use pairwise
  // additions to incrementally sum each vector.
  int64x2_t v01   = vpaddq_s64( svget_neonq_s64( v0 ), svget_neonq_s64( v1 ) );
  int64x2_t v23   = vpaddq_s64( svget_neonq_s64( v2 ), svget_neonq_s64( v3 ) );
  int32x4_t v0123 = vuzp1q_s32( vreinterpretq_s32_s64( v01 ), vreinterpretq_s32_s64( v23 ) );
  v0123           = vrshlq_s32( v0123, vdupq_n_s32( -shift ) );
  vst1q_s32( dst, v0123 );
}

template<>
inline void fastFwdCore_reduce_x4_sve<256>( TCoeff* dst, svint64_t v0, svint64_t v1, svint64_t v2, svint64_t v3,
                                            int shift )
{
  // Halve the data width such that we only utilise the low half (128 bits) of each vector.
  svint32_t v0_s32 = svuzp1_s32( svreinterpret_s32_s64( v0 ), svreinterpret_s32_s64( v0 ) );
  svint32_t v1_s32 = svuzp1_s32( svreinterpret_s32_s64( v1 ), svreinterpret_s32_s64( v1 ) );
  svint32_t v2_s32 = svuzp1_s32( svreinterpret_s32_s64( v2 ), svreinterpret_s32_s64( v2 ) );
  svint32_t v3_s32 = svuzp1_s32( svreinterpret_s32_s64( v3 ), svreinterpret_s32_s64( v3 ) );

  // Now that we have data in the low 128 bits of each vector, use svget_neonq
  // to operate on the Neon vectors directly and use pairwise additions to
  // incrementally sum each vector.
  int32x4_t v01   = vpaddq_s32( svget_neonq_s32( v0_s32 ), svget_neonq_s32( v1_s32 ) );
  int32x4_t v23   = vpaddq_s32( svget_neonq_s32( v2_s32 ), svget_neonq_s32( v3_s32 ) );
  int32x4_t v0123 = vpaddq_s32( v01, v23 );
  v0123           = vrshlq_s32( v0123, vdupq_n_s32( -shift ) );
  vst1q_s32( dst, v0123 );
}

template<int vlBits, unsigned trVecs>
static void fastFwdCore_nVec_sve( const TMatrixCoeff* tc, const TCoeff* src, TCoeff* dst, unsigned line,
                                  unsigned reducedLine, unsigned cutoff, int shift )
{
  CHECK( cutoff % 4 != 0, "Cutoff should be a multiple of four" );
  CHECK( cutoff == 0, "Cutoff should be non-zero" );
  CHECK( shift == 0, "Shift must be at least one" );

  unsigned trSize = trVecs * (unsigned)svcnth();
  unsigned i      = 0;
  for( ; i < ( reducedLine & ~3U ); i += 4 )
  {
    for( int j = 0; j < cutoff; j += 4 )
    {
      const TMatrixCoeff* tcj = tc + j * trSize;
      const TCoeff* srci      = src + i * trSize;

      svint64_t sum00 = svdup_n_s64( 0 );
      svint64_t sum01 = svdup_n_s64( 0 );
      svint64_t sum02 = svdup_n_s64( 0 );
      svint64_t sum03 = svdup_n_s64( 0 );
      svint64_t sum10 = svdup_n_s64( 0 );
      svint64_t sum11 = svdup_n_s64( 0 );
      svint64_t sum12 = svdup_n_s64( 0 );
      svint64_t sum13 = svdup_n_s64( 0 );
      svint64_t sum20 = svdup_n_s64( 0 );
      svint64_t sum21 = svdup_n_s64( 0 );
      svint64_t sum22 = svdup_n_s64( 0 );
      svint64_t sum23 = svdup_n_s64( 0 );
      svint64_t sum30 = svdup_n_s64( 0 );
      svint64_t sum31 = svdup_n_s64( 0 );
      svint64_t sum32 = svdup_n_s64( 0 );
      svint64_t sum33 = svdup_n_s64( 0 );
      for( int k = 0; k < trVecs; ++k )
      {
        svint16_t s0 = load_narrow_to_s16( srci + 0 * trSize );
        svint16_t s1 = load_narrow_to_s16( srci + 1 * trSize );
        svint16_t s2 = load_narrow_to_s16( srci + 2 * trSize );
        svint16_t s3 = load_narrow_to_s16( srci + 3 * trSize );
        svint16_t c0 = svld1_s16( svptrue_b16(), tcj + 0 * trSize );
        svint16_t c1 = svld1_s16( svptrue_b16(), tcj + 1 * trSize );
        svint16_t c2 = svld1_s16( svptrue_b16(), tcj + 2 * trSize );
        svint16_t c3 = svld1_s16( svptrue_b16(), tcj + 3 * trSize );
        sum00        = svdot_s64( sum00, s0, c0 );
        sum01        = svdot_s64( sum01, s0, c1 );
        sum02        = svdot_s64( sum02, s0, c2 );
        sum03        = svdot_s64( sum03, s0, c3 );
        sum10        = svdot_s64( sum10, s1, c0 );
        sum11        = svdot_s64( sum11, s1, c1 );
        sum12        = svdot_s64( sum12, s1, c2 );
        sum13        = svdot_s64( sum13, s1, c3 );
        sum20        = svdot_s64( sum20, s2, c0 );
        sum21        = svdot_s64( sum21, s2, c1 );
        sum22        = svdot_s64( sum22, s2, c2 );
        sum23        = svdot_s64( sum23, s2, c3 );
        sum30        = svdot_s64( sum30, s3, c0 );
        sum31        = svdot_s64( sum31, s3, c1 );
        sum32        = svdot_s64( sum32, s3, c2 );
        sum33        = svdot_s64( sum33, s3, c3 );

        srci += svcnth();
        tcj += svcnth();
      }
      TCoeff* dstij = dst + j * line + i;
      fastFwdCore_reduce_x4_sve<vlBits>( dstij + 0 * line, sum00, sum10, sum20, sum30, shift );
      fastFwdCore_reduce_x4_sve<vlBits>( dstij + 1 * line, sum01, sum11, sum21, sum31, shift );
      fastFwdCore_reduce_x4_sve<vlBits>( dstij + 2 * line, sum02, sum12, sum22, sum32, shift );
      fastFwdCore_reduce_x4_sve<vlBits>( dstij + 3 * line, sum03, sum13, sum23, sum33, shift );
    }
  }
  for( ; i < reducedLine; ++i )
  {
    for( int j = 0; j < cutoff; j += 4 )
    {
      const TMatrixCoeff* tcj = tc + j * trSize;
      const TCoeff* srci      = src + i * trSize;

      svint64_t sum00 = svdup_n_s64( 0 );
      svint64_t sum01 = svdup_n_s64( 0 );
      svint64_t sum02 = svdup_n_s64( 0 );
      svint64_t sum03 = svdup_n_s64( 0 );
      for( int k = 0; k < trVecs; ++k )
      {
        svint16_t s0 = load_narrow_to_s16( srci + 0 * trSize );
        svint16_t c0 = svld1_s16( svptrue_b16(), tcj + 0 * trSize );
        svint16_t c1 = svld1_s16( svptrue_b16(), tcj + 1 * trSize );
        svint16_t c2 = svld1_s16( svptrue_b16(), tcj + 2 * trSize );
        svint16_t c3 = svld1_s16( svptrue_b16(), tcj + 3 * trSize );
        sum00        = svdot_s64( sum00, s0, c0 );
        sum01        = svdot_s64( sum01, s0, c1 );
        sum02        = svdot_s64( sum02, s0, c2 );
        sum03        = svdot_s64( sum03, s0, c3 );

        srci += svcnth();
        tcj += svcnth();
      }
      TCoeff* dstij         = dst + j * line + i;
      dstij[ 0 * line + 0 ] = (TCoeff)shift_and_round( svaddv_s64( svptrue_b64(), sum00 ), shift );
      dstij[ 1 * line + 0 ] = (TCoeff)shift_and_round( svaddv_s64( svptrue_b64(), sum01 ), shift );
      dstij[ 2 * line + 0 ] = (TCoeff)shift_and_round( svaddv_s64( svptrue_b64(), sum02 ), shift );
      dstij[ 3 * line + 0 ] = (TCoeff)shift_and_round( svaddv_s64( svptrue_b64(), sum03 ), shift );
    }
  }
}

template<>
void TCoeffOps::_initTCoeffOpsARM<SVE>()
{
  // Wire up kernels based on how many vector iterations we need in the inner
  // loop. Use Neon if we don't have at least one vector of work to do. Arm
  // Neoverse micro-architectures only currently exist with vector lengths of
  // 128 and 256 bits, so don't bother specialising for other vector lengths.
  switch( svcnth() )
  {
  case 8:  // SVE VL = 128-bits
    fastFwdCore_2D[ 1 ] = fastFwdCore_nVec_sve<128, 1>;
    fastFwdCore_2D[ 2 ] = fastFwdCore_nVec_sve<128, 2>;
    fastFwdCore_2D[ 3 ] = fastFwdCore_nVec_sve<128, 4>;
    fastFwdCore_2D[ 4 ] = fastFwdCore_nVec_sve<128, 8>;
    break;
  case 16:  // SVE VL = 256-bits
    fastFwdCore_2D[ 2 ] = fastFwdCore_nVec_sve<256, 1>;
    fastFwdCore_2D[ 3 ] = fastFwdCore_nVec_sve<256, 2>;
    fastFwdCore_2D[ 4 ] = fastFwdCore_nVec_sve<256, 4>;
    break;
  default:
    // Don't use SVE for other vector lengths, fall back to Neon.
    break;
  }
}

}  // namespace vvenc

#endif
//! \}

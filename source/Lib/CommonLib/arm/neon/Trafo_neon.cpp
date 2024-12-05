/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the Clear BSD
License, included below. No patent rights, trademark rights and/or
other Intellectual Property Rights other than the copyrights concerning
the Software are granted under this license.

The Clear BSD License

Copyright (c) 2019-2024, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. & The VVenC Authors.
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
 * \file Trafo_neon.cpp
 * \brief Neon implementation of TCoeffOps for AArch64.
 */

//  ====================================================================================================================
//  Includes
//  ====================================================================================================================
#include "CommonDefARM.h"
#include "CommonLib/CommonDef.h"
#include "TrQuant.h"
#include "TrQuant_EMT.h"
#include "sum_neon.h"

//! \ingroup CommonLib
//! \{

#if defined( TARGET_SIMD_ARM ) && ENABLE_SIMD_TRAFO

namespace vvenc
{

static inline int64_t shift_and_round( int64_t x, int shift )
{
  return ( x + ( static_cast<int64_t>(1) << ( shift - 1 ) ) ) >> shift;
}

template<unsigned trSize>
static void fastInvCore_neon( const TMatrixCoeff* it, const TCoeff* src, TCoeff* dst, unsigned lines,
                              unsigned reducedLines, unsigned rows )
{
  static_assert( trSize % 4 == 0, "trSize should be a multiple of four" );
  CHECK( rows % 4 != 0, "rows should be a multiple of four" );
  CHECK( rows == 0, "rows should be non-zero" );

  unsigned i = 0;
  for( ; i != ( reducedLines & ~3U ); i += 4 )
  {
    unsigned j = 0;
    do
    {
      const TCoeff* srci      = src + i;
      const TMatrixCoeff* itj = it + j;
      TCoeff* dstij           = dst + i * trSize + j;

      int32x4_t d0 = vld1q_s32( dstij + 0 * trSize );
      int32x4_t d1 = vld1q_s32( dstij + 1 * trSize );
      int32x4_t d2 = vld1q_s32( dstij + 2 * trSize );
      int32x4_t d3 = vld1q_s32( dstij + 3 * trSize );

      unsigned k = rows;
      do
      {
        int16x4_t s = vmovn_s32( vld1q_s32( srci ) );
        int16x4_t c = vld1_s16( itj );

        d0 = vmlal_lane_s16( d0, c, s, 0 );
        d1 = vmlal_lane_s16( d1, c, s, 1 );
        d2 = vmlal_lane_s16( d2, c, s, 2 );
        d3 = vmlal_lane_s16( d3, c, s, 3 );

        srci += lines;
        itj += trSize;
      } while( --k != 0 );

      vst1q_s32( dstij + 0 * trSize, d0 );
      vst1q_s32( dstij + 1 * trSize, d1 );
      vst1q_s32( dstij + 2 * trSize, d2 );
      vst1q_s32( dstij + 3 * trSize, d3 );

      j += 4;
    } while( j != trSize );
  }

  for( ; i != reducedLines; ++i )
  {
    unsigned j = 0;
    do
    {
      const TCoeff* srci      = src + i;
      const TMatrixCoeff* itj = it + j;
      TCoeff* dstij           = dst + i * trSize + j;

      int32_t d0 = *dstij;
      unsigned k = rows;
      do
      {
        d0 += *srci * *itj;

        srci += lines;
        itj += trSize;
      } while( --k != 0 );

      *dstij = d0;
    } while( ++j != trSize );
  }
}

template<unsigned trSize>
static void fastFwdCore_neon( const TMatrixCoeff* tc, const TCoeff* src, TCoeff* dst, unsigned line,
                              unsigned reducedLine, unsigned cutoff, int shift )
{
  static_assert( trSize % 4 == 0, "trSize should be a multiple of four" );
  CHECK( cutoff % 4 != 0, "cutoff should be a multiple of four" );
  CHECK( cutoff == 0, "cutoff should be non-zero" );
  CHECK( shift == 0, "shift must be at least one" );

  unsigned i = 0;
  for( ; i < ( reducedLine & ~3U ); i += 4 )
  {
    unsigned j = 0;
    do
    {
      const TMatrixCoeff* tcj = tc + j * trSize;
      const TCoeff* srci      = src + i * trSize;

      int32x4_t sum00 = vdupq_n_s32( 0 );
      int32x4_t sum01 = vdupq_n_s32( 0 );
      int32x4_t sum02 = vdupq_n_s32( 0 );
      int32x4_t sum03 = vdupq_n_s32( 0 );
      int32x4_t sum10 = vdupq_n_s32( 0 );
      int32x4_t sum11 = vdupq_n_s32( 0 );
      int32x4_t sum12 = vdupq_n_s32( 0 );
      int32x4_t sum13 = vdupq_n_s32( 0 );
      int32x4_t sum20 = vdupq_n_s32( 0 );
      int32x4_t sum21 = vdupq_n_s32( 0 );
      int32x4_t sum22 = vdupq_n_s32( 0 );
      int32x4_t sum23 = vdupq_n_s32( 0 );
      int32x4_t sum30 = vdupq_n_s32( 0 );
      int32x4_t sum31 = vdupq_n_s32( 0 );
      int32x4_t sum32 = vdupq_n_s32( 0 );
      int32x4_t sum33 = vdupq_n_s32( 0 );
      for( unsigned k = 0; k < trSize; k += 4 )
      {
        int16x4_t s0 = vmovn_s32( vld1q_s32( srci + 0 * trSize ) );
        int16x4_t s1 = vmovn_s32( vld1q_s32( srci + 1 * trSize ) );
        int16x4_t s2 = vmovn_s32( vld1q_s32( srci + 2 * trSize ) );
        int16x4_t s3 = vmovn_s32( vld1q_s32( srci + 3 * trSize ) );
        int16x4_t c0 = vld1_s16( tcj + 0 * trSize );
        int16x4_t c1 = vld1_s16( tcj + 1 * trSize );
        int16x4_t c2 = vld1_s16( tcj + 2 * trSize );
        int16x4_t c3 = vld1_s16( tcj + 3 * trSize );

        sum00 = vmlal_s16( sum00, s0, c0 );
        sum01 = vmlal_s16( sum01, s0, c1 );
        sum02 = vmlal_s16( sum02, s0, c2 );
        sum03 = vmlal_s16( sum03, s0, c3 );
        sum10 = vmlal_s16( sum10, s1, c0 );
        sum11 = vmlal_s16( sum11, s1, c1 );
        sum12 = vmlal_s16( sum12, s1, c2 );
        sum13 = vmlal_s16( sum13, s1, c3 );
        sum20 = vmlal_s16( sum20, s2, c0 );
        sum21 = vmlal_s16( sum21, s2, c1 );
        sum22 = vmlal_s16( sum22, s2, c2 );
        sum23 = vmlal_s16( sum23, s2, c3 );
        sum30 = vmlal_s16( sum30, s3, c0 );
        sum31 = vmlal_s16( sum31, s3, c1 );
        sum32 = vmlal_s16( sum32, s3, c2 );
        sum33 = vmlal_s16( sum33, s3, c3 );

        srci += 4;
        tcj += 4;
      }
      int32x4_t sum0 = horizontal_add_4d_s32x4( sum00, sum10, sum20, sum30 );
      int32x4_t sum1 = horizontal_add_4d_s32x4( sum01, sum11, sum21, sum31 );
      int32x4_t sum2 = horizontal_add_4d_s32x4( sum02, sum12, sum22, sum32 );
      int32x4_t sum3 = horizontal_add_4d_s32x4( sum03, sum13, sum23, sum33 );

      sum0 = vrshlq_s32( sum0, vdupq_n_s32( -shift ) );
      sum1 = vrshlq_s32( sum1, vdupq_n_s32( -shift ) );
      sum2 = vrshlq_s32( sum2, vdupq_n_s32( -shift ) );
      sum3 = vrshlq_s32( sum3, vdupq_n_s32( -shift ) );

      TCoeff* dstij = dst + j * line + i;
      vst1q_s32( dstij + 0 * line, sum0 );
      vst1q_s32( dstij + 1 * line, sum1 );
      vst1q_s32( dstij + 2 * line, sum2 );
      vst1q_s32( dstij + 3 * line, sum3 );

      j += 4;
    } while( j != cutoff );
  }
  for( ; i < reducedLine; ++i )
  {
    int j = 0;
    do
    {
      const TMatrixCoeff* tcj = tc + j * trSize;
      const TCoeff* srci      = src + i * trSize;

      int32x4_t sum00 = vdupq_n_s32( 0 );
      int32x4_t sum01 = vdupq_n_s32( 0 );
      int32x4_t sum02 = vdupq_n_s32( 0 );
      int32x4_t sum03 = vdupq_n_s32( 0 );
      for( int k = 0; k < trSize; k += 4 )
      {
        int16x4_t s0 = vmovn_s32( vld1q_s32( srci ) );
        int16x4_t c0 = vld1_s16( tcj + 0 * trSize );
        int16x4_t c1 = vld1_s16( tcj + 1 * trSize );
        int16x4_t c2 = vld1_s16( tcj + 2 * trSize );
        int16x4_t c3 = vld1_s16( tcj + 3 * trSize );

        sum00 = vmlal_s16( sum00, s0, c0 );
        sum01 = vmlal_s16( sum01, s0, c1 );
        sum02 = vmlal_s16( sum02, s0, c2 );
        sum03 = vmlal_s16( sum03, s0, c3 );

        srci += 4;
        tcj += 4;
      }
      TCoeff* dstij = dst + j * line + i;

      dstij[ 0 * line ] = (TCoeff) shift_and_round( horizontal_add_s32x4( sum00 ), shift );
      dstij[ 1 * line ] = (TCoeff) shift_and_round( horizontal_add_s32x4( sum01 ), shift );
      dstij[ 2 * line ] = (TCoeff) shift_and_round( horizontal_add_s32x4( sum02 ), shift );
      dstij[ 3 * line ] = (TCoeff) shift_and_round( horizontal_add_s32x4( sum03 ), shift );

      j += 4;
    } while( j != cutoff );
  }
}

template<>
void TCoeffOps::_initTCoeffOpsARM<NEON>()
{
  fastInvCore[ 0 ]    = fastInvCore_neon<4>;
  fastInvCore[ 1 ]    = fastInvCore_neon<8>;
  fastInvCore[ 2 ]    = fastInvCore_neon<16>;
  fastInvCore[ 3 ]    = fastInvCore_neon<32>;
  fastInvCore[ 4 ]    = fastInvCore_neon<64>;
  fastFwdCore_2D[ 0 ] = fastFwdCore_neon<4>;
  fastFwdCore_2D[ 1 ] = fastFwdCore_neon<8>;
  fastFwdCore_2D[ 2 ] = fastFwdCore_neon<16>;
  fastFwdCore_2D[ 3 ] = fastFwdCore_neon<32>;
  fastFwdCore_2D[ 4 ] = fastFwdCore_neon<64>;
}

}  // namespace vvenc

#endif
//! \}

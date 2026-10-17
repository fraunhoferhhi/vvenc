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
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED UNDER
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

/** \file     InterSearchX86.h
 \brief    x86 helpers for the IBC block vector search
 */

#include "CommonDefX86.h"

#include "../BvSearchOps.h"

//! \ingroup EncoderLib
//! \{

#if defined( TARGET_SIMD_X86 ) && ENABLE_SIMD_OPT

namespace vvenc
{

// true if mv is one of the first n entries of list. A motion vector consists of two 32-bit components, so one 64-bit
// lane of _mm_cmpeq_epi64 compares one candidate; eight (SSE4.1) or sixteen (AVX2) candidates are tested per branch.
template<X86_VEXT vext>
static bool containsMv_SIMD( const Mv* list, const unsigned n, const Mv& mv )
{
  static_assert( sizeof( Mv ) == 2 * sizeof( int32_t ), "an Mv has to consist of its two 32-bit components only" );

  const __m128i mv64 = _vv_loadl_epi64( ( const __m128i* ) &mv );
  const __m128i key  = _mm_unpacklo_epi64( mv64, mv64 );
  auto equal = [&key]( const Mv* candidates ) { return _mm_cmpeq_epi64( _mm_loadu_si128( ( const __m128i* ) candidates ), key ); };

  unsigned i = 0;
#if USE_AVX2
  if( vext >= AVX2 )
  {
    const __m256i key256 = _mm256_broadcastq_epi64( mv64 );
    auto equal256 = [&key256]( const Mv* candidates ) { return _mm256_cmpeq_epi64( _mm256_loadu_si256( ( const __m256i* ) candidates ), key256 ); };
    for( ; i + 16 <= n; i += 16 )
    {
      const __m256i found = _mm256_or_si256( _mm256_or_si256( equal256( &list[i] ), equal256( &list[i + 4] ) ),
                                             _mm256_or_si256( equal256( &list[i + 8] ), equal256( &list[i + 12] ) ) );
      if( !_mm256_testz_si256( found, found ) )
      {
        return true;
      }
    }
  }
#endif
  for( ; i + 8 <= n; i += 8 )
  {
    const __m128i found = _mm_or_si128( _mm_or_si128( equal( &list[i] ), equal( &list[i + 2] ) ),
                                        _mm_or_si128( equal( &list[i + 4] ), equal( &list[i + 6] ) ) );
    if( !_mm_testz_si128( found, found ) )
    {
      return true;
    }
  }
  for( ; i + 2 <= n; i += 2 )
  {
    const __m128i found = equal( &list[i] );
    if( !_mm_testz_si128( found, found ) )
    {
      return true;
    }
  }
  return i < n && list[i] == mv;
}

// sum over the block of ( |org - cur| >> shift ), for samples in 0 ... 2^15 - 1: the differences fit 16 bits, and the
// 32-bit lane sums cannot overflow for blocks of up to 2^16 samples
template<X86_VEXT vext>
static Distortion sumAbsDiffShifted_SIMD( const Pel* org, const ptrdiff_t orgStride, const Pel* cur, const ptrdiff_t curStride, const int width, const int height, const int shift )
{
  CHECKD( width * height > ( 1 << 16 ), "block too large for 32-bit lane sums" );

  const __m128i ones   = _mm_set1_epi16( 1 );
  const __m128i vshift = _mm_cvtsi32_si128( shift );
  __m128i       acc    = _mm_setzero_si128();
  auto addTerms = [&]( const __m128i o, const __m128i c )
  {
    acc = _mm_add_epi32( acc, _mm_madd_epi16( _mm_srl_epi16( _mm_abs_epi16( _mm_sub_epi16( c, o ) ), vshift ), ones ) );
  };
  // the samples of up to four 2-wide or two 4-wide rows in one vector; the lanes of missing rows are zero
  auto loadRows = []( const Pel* p, const ptrdiff_t stride, const int rowWidth, const int numRows )
  {
    if( rowWidth == 4 )
    {
      const __m128i r0 = _vv_loadl_epi64( ( const __m128i* ) p );
      return numRows > 1 ? _mm_unpacklo_epi64( r0, _vv_loadl_epi64( ( const __m128i* ) &p[stride] ) ) : r0;
    }
    __m128i r[4];
    for( int i = 0; i < 4; i++ )
    {
      r[i] = i < numRows ? _mm_loadu_si32( &p[i * stride] ) : _mm_setzero_si128();
    }
    return _mm_unpacklo_epi64( _mm_unpacklo_epi32( r[0], r[1] ), _mm_unpacklo_epi32( r[2], r[3] ) );
  };

  Distortion tail = 0;
  if( width == 2 || width == 4 )
  {
    const int rowsPerVector = 8 / width;
    for( int y = 0; y < height; y += rowsPerVector, org += rowsPerVector * orgStride, cur += rowsPerVector * curStride )
    {
      const int numRows = std::min( rowsPerVector, height - y );
      addTerms( loadRows( org, orgStride, width, numRows ), loadRows( cur, curStride, width, numRows ) );
    }
  }
  else
  {
    for( int y = 0; y < height; y++, org += orgStride, cur += curStride )
    {
      int x = 0;
      for( ; x + 8 <= width; x += 8 )
      {
        addTerms( _mm_loadu_si128( ( const __m128i* ) &org[x] ), _mm_loadu_si128( ( const __m128i* ) &cur[x] ) );
      }
      for( ; x < width; x++ )
      {
        tail += abs( cur[x] - org[x] ) >> shift;
      }
    }
  }

  acc = _mm_add_epi32( acc, _mm_unpackhi_epi64( acc, acc ) );
  acc = _mm_add_epi32( acc, _mm_shuffle_epi32( acc, 1 ) );
  return tail + Distortion( _mm_cvtsi128_si32( acc ) );
}

template<X86_VEXT vext>
void BvSearchOps::_initBvSearchOpsX86()
{
  containsMv        = containsMv_SIMD<vext>;
  sumAbsDiffShifted = sumAbsDiffShifted_SIMD<vext>;
}

template void BvSearchOps::_initBvSearchOpsX86<SIMDX86>();

} // namespace vvenc

#endif // TARGET_SIMD_X86 && ENABLE_SIMD_OPT

//! \}

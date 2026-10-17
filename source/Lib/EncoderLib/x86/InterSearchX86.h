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

template<X86_VEXT vext>
void BvSearchOps::_initBvSearchOpsX86()
{
  containsMv = containsMv_SIMD<vext>;
}

template void BvSearchOps::_initBvSearchOpsX86<SIMDX86>();

} // namespace vvenc

#endif // TARGET_SIMD_X86 && ENABLE_SIMD_OPT

//! \}

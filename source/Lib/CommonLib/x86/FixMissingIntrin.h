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

#pragma once

#ifdef TARGET_SIMD_X86

#include <simde/x86/sse2.h>
#ifdef USE_AVX
#include <simde/x86/avx.h>
#endif
#ifdef USE_AVX2
#include <simde/x86/avx2.h>
#endif

#include <cstdint>

namespace vvenc
{

#ifdef MISSING_INTRIN_mm_storeu_si16
static inline void _mm_storeu_si16( void* p, __m128i a )
{
  *(short*) ( p ) = (short) _mm_cvtsi128_si32( a );
}
#endif

#ifdef MISSING_INTRIN_mm_storeu_si32
static inline void _mm_storeu_si32( void* p, __m128i a )
{
  *(int32_t*)p = _mm_cvtsi128_si32( a );
}
#endif

#ifdef MISSING_INTRIN_mm_storeu_si64
static inline void _mm_storeu_si64( void* p, __m128i a )
{
  _mm_storel_epi64( (__m128i*)p, a);
}
#endif

#ifdef MISSING_INTRIN_mm_loadu_si32
static inline __m128i _mm_loadu_si32( const void* p )
{
  return _mm_cvtsi32_si128( *(int32_t*)p );
}
#endif

#ifdef MISSING_INTRIN_mm_loadu_si64
static inline __m128i _mm_loadu_si64( const void* p )
{
  return _mm_loadl_epi64( (const __m128i*)p );
}
#endif

#ifdef MISSING_INTRIN_mm_bslli_si128
static inline __m128i __m128i _mm_bslli_si128( __m128i a, int i )
{
  return _mm_slli_si128( a, i )
}
#endif

#ifdef MISSING_INTRIN_mm_bsrli_si128
static inline __m128i _mm_bsrli_si128( __m128i a, int i )
{
  return _mm_srli_si128( a, i );
}
#endif


#if defined( USE_AVX ) || defined( USE_AVX2 )

// this should only be true for non-x86 architectures
#ifdef MISSING_INTRIN_mm256_zeroupper
#if defined( __x86_64__ ) || defined( _M_X64 ) || defined( __i386__ ) || defined( __i386 ) || defined( _M_IX86 )
#error MISSING_INTRIN_mm256_zeroupper should not be defined on x86
#endif

static inline void _mm256_zeroupper() {}  // NOOP
#endif

#ifdef MISSING_INTRIN_mm256_loadu2_m128i
static inline __m256i _mm256_loadu2_m128i( __m128i const* hiaddr, __m128i const* loaddr )
{
  return _mm256_inserti128_si256( _mm256_castsi128_si256( _mm_loadu_si128( hiaddr ) ), _mm_loadu_si128( loaddr ), 1 );
}
#endif

#ifdef MISSING_INTRIN_mm256_extract_epi32
static inline uint32_t _mm256_extract_epi32( __m256i vec, const int i )
{
  __m128i indx = _mm_cvtsi32_si128( i );
  __m256i val  = _mm256_permutevar8x32_epi32( vec, _mm256_castsi128_si256( indx ) );
  return _mm_cvtsi128_si32( _mm256_castsi256_si128( val ) );
}
#endif

#ifdef MISSING_INTRIN_mm256_set_m128i
static inline __m256i _mm256_set_m128i( __m128i hi, __m128i lo )
{
  return _mm256_insertf128_si256( _mm256_castsi128_si256( lo ), hi, 1 );
}
#endif

#endif   // USE_AVX


#ifdef USE_AVX2

#ifdef MISSING_INTRIN_mm256_bsrli_epi128
static inline __m256i _mm256_bsrli_epi128( __m256i a, const int i )
{
  return _mm256_srli_si256( a, i ) :
}
#endif

#endif   // USE_AVX2

}   // namespace vvenc

#endif   // TARGET_SIMD_X86

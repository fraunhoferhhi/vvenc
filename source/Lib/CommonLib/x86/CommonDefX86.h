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

/** \file     CommonDefX86.h
*/

#pragma once

#include "CommonDef.h"

//! \ingroup CommonLib
//! \{

#if defined(TARGET_SIMD_X86)  && ENABLE_SIMD_OPT

#  if REAL_TARGET_X86 || REAL_TARGET_WASM
#    ifdef _WIN32
#      include <intrin.h>
#    else
#      include <immintrin.h>
#    endif
#  else    // !REAL_TARGET_X86 && !REAL_TARGET_WASM
#    define SIMDE_ENABLE_NATIVE_ALIASES
#  endif   // !REAL_TARGET_X86 && !REAL_TARGET_WASM

#  include "FixMissingIntrin.h"

#  ifdef USE_AVX512
#    define SIMDX86 AVX512
#    include <simde/x86/avx512.h>
#  elif defined USE_AVX2
#    define SIMDX86 AVX2
#    include <simde/x86/avx2.h>
#  elif defined USE_AVX
#    define SIMDX86 AVX
#    include <simde/x86/avx.h>
#  elif defined USE_SSE42
#    define SIMDX86 SSE42
#    include <simde/x86/sse4.2.h>
#  elif defined USE_SSE41
#    define SIMDX86 SSE41
#    include <simde/x86/sse4.1.h>
#  endif

namespace vvenc
{

using namespace x86_simd;

const std::string& x86_vext_to_string( X86_VEXT vext );
X86_VEXT           string_to_x86_vext( const std::string& ext_name );

X86_VEXT           read_x86_extension_flags( X86_VEXT request = x86_simd::UNDEFINED );
const std::string& read_x86_extension_name();

#if (defined REAL_TARGET_ARM && !defined REAL_TARGET_AARCH64)
// _mm_loadl_epi64 / _mm_storel_epi64 could cause memory crash in 32 Bit ARM Architecture 
#define _vv_loadl_epi64 _mm_loadu_si64
#define _vv_storel_epi64 _mm_storeu_si64
#else
#define _vv_loadl_epi64 _mm_loadl_epi64
#define _vv_storel_epi64 _mm_storel_epi64
#endif

#ifdef USE_AVX2

static inline __m128i _mm256_cvtepi32_epi16x( __m256i& v )
{
  return  _mm_packs_epi32( _mm256_castsi256_si128( v ), _mm256_extracti128_si256( v, 1 ) );
}

#endif

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


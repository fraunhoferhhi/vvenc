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

/*
 * \ingroup CommonLib
 * \file    CommondefX86.cpp
 * \brief   This file contains the SIMD x86 common used functions.
 */

#include "CommonDefX86.h"

#include <map>
#include <cstdint>
#include "CommonLib/CommonDef.h"


#ifdef REAL_TARGET_X86
# if defined( _WIN32 ) && !defined( __MINGW32__ )
#  include <intrin.h>
# else
#  include <x86intrin.h>  // needed for _xgetbv, which is not provided by simd-everywhere
#  include <cpuid.h>
# endif
#endif

#ifdef TARGET_SIMD_X86

namespace vvenc
{

#  if REAL_TARGET_X86
const static std::map<X86_VEXT, std::string> vext_names{ { UNDEFINED, "" }, { SCALAR, "SCALAR" }, { SSE41, "SSE41" }, { SSE42, "SSE42" }, { AVX, "AVX" }, { AVX2, "AVX2" }, { AVX512, "AVX512" } };
#  else   // !REAL_TARGET_X86
#    if defined( REAL_TARGET_ARM )
const static std::map<X86_VEXT, std::string> vext_names{ { UNDEFINED, "" }, { SCALAR, "SCALAR" }, { SIMD_EVERYWHERE_EXTENSION_LEVEL, "NEON" } };
#    elif defined( REAL_TARGET_WASM )
const static std::map<X86_VEXT, std::string> vext_names{ { UNDEFINED, "" }, { SCALAR, "SCALAR" }, { SIMD_EVERYWHERE_EXTENSION_LEVEL, "WASM" } };
#    elif defined( REAL_TARGET_LOONGARCH )
const static std::map<X86_VEXT, std::string> vext_names{ { UNDEFINED, "" }, { SCALAR, "SCALAR" }, { SIMD_EVERYWHERE_EXTENSION_LEVEL, "LSX" } };
#    else
const static std::map<X86_VEXT, std::string> vext_names{ { UNDEFINED, "" }, { SCALAR, "SCALAR" }, { SIMD_EVERYWHERE_EXTENSION_LEVEL, "SIMDE" } };
#    endif
#  endif   // !REAL_TARGET_X86

const std::string& x86_vext_to_string( X86_VEXT vext )
{
  try
  {
    return vext_names.at( vext );
  }
  catch( std::out_of_range& )
  {
    THROW( "Invalid SIMD extension value " << vext );
  }
}

X86_VEXT string_to_x86_vext( const std::string& ext_name )
{
  if( ext_name.empty() )
  {
    return UNDEFINED;
  }

  for( auto& it: vext_names )
  {
    if( it.second == ext_name )
    {
      return it.first;
    }
  }

  THROW( "Invalid SIMD Mode string: \"" << ext_name << "\"" );
}

#if __GNUC__   // valid for GCC and clang
# define NO_OPT_SIMD __attribute__( ( optimize( "no-tree-vectorize" ) ) )
#else
# define NO_OPT_SIMD
#endif

#ifdef REAL_TARGET_X86

#if defined( __MINGW32__ ) && !defined( __MINGW64__ )
# define SIMD_UP_TO_SSE42 1
#else
# define SIMD_UP_TO_SSE42 0
#endif


/* use __cpuid for windows or inline assembler for gcc and clang */
#if defined( _WIN32 ) && !defined( __MINGW32__ )
# define doCpuid   __cpuid
# define doCpuidex __cpuidex
#else   // !_WIN32
static inline void doCpuid( int CPUInfo[4], int InfoType )
{
  __get_cpuid( (unsigned) InfoType, (unsigned*) &CPUInfo[0], (unsigned*) &CPUInfo[1], (unsigned*) &CPUInfo[2], (unsigned*) &CPUInfo[3] );
}
# if !SIMD_UP_TO_SSE42
static inline void doCpuidex( int CPUInfo[4], int InfoType0, int InfoType1 )
{
  __cpuid_count( InfoType0, InfoType1, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3] );
}
# endif  // !SIMD_UP_TO_SSE42
#endif  // !_WIN32

static inline int64_t xgetbv( int ctr )
{
#if( defined( _MSC_FULL_VER ) && _MSC_FULL_VER >= 160040000 )    \
  || ( defined( __INTEL_COMPILER ) && __INTEL_COMPILER >= 1200 ) \
  || GCC_VERSION_AT_LEAST( 8, 0 )                                \
  || CLANG_VERSION_AT_LEAST( 9, 0 )   // Microsoft, Intel, newer GCC or newer Clang compiler supporting _xgetbv intrinsic

  return _xgetbv( ctr );   // intrinsic function for XGETBV

#elif defined( __GNUC__ )   // use inline assembly, Gnu/AT&T syntax

  uint32_t a, d;
#if GCC_VERSION_AT_LEAST( 4, 4 ) || CLANG_VERSION_AT_LEAST( 3, 3 )
  __asm( "xgetbv" : "=a"( a ), "=d"( d ) : "c"( ctr ) : );
#else
  __asm( ".byte 0x0f, 0x01, 0xd0" : "=a"( a ), "=d"( d ) : "c"( ctr ) : );
#endif
  return a | ( uint64_t( d ) << 32 );

#else  // #elif defined (_MSC_FULL_VER) || (defined (__INTEL_COMPILER)...) // other compiler. try inline assembly with masm/intel/MS syntax

  uint32_t a, d;
  __asm {
        mov ecx, ctr
        _emit 0x0f
        _emit 0x01
        _emit 0xd0 ; // xgetbv
        mov a, eax
        mov d, edx
  }
  return a | ( uint64_t( d ) << 32 );

#endif
}


#define BIT_HAS_MMX                    (1 << 23)
#define BIT_HAS_SSE                    (1 << 25)
#define BIT_HAS_SSE2                   (1 << 26)
#define BIT_HAS_SSE3                   (1 <<  0)
#define BIT_HAS_SSSE3                  (1 <<  9)
#define BIT_HAS_SSE41                  (1 << 19)
#define BIT_HAS_SSE42                  (1 << 20)
#define BIT_HAS_SSE4a                  (1 <<  6)
#define BIT_HAS_OSXSAVE                (1 << 27)
#define BIT_HAS_AVX                   ((1 << 28)|BIT_HAS_OSXSAVE)
#define BIT_HAS_AVX2                   (1 <<  5)
#define BIT_HAS_AVX512F                (1 << 16)
#define BIT_HAS_AVX512DQ               (1 << 17)
#define BIT_HAS_AVX512BW               (1 << 30)
#define BIT_HAS_FMA3                   (1 << 12)
#define BIT_HAS_FMA4                   (1 << 16)
#define BIT_HAS_X64                    (1 << 29)
#define BIT_HAS_XOP                    (1 << 11)

/**
 * \brief Read instruction set extension support flags from CPU register;
 */
NO_OPT_SIMD
static X86_VEXT _get_x86_extensions()
{
  int      regs[4] = { 0, 0, 0, 0 };
  X86_VEXT ext;
  ext = SCALAR;

  doCpuid( regs, 0 );
  if( regs[0] == 0 )
    return ext;

  doCpuid( regs, 1 );
  if( !( regs[2] & BIT_HAS_SSE41 ) )
    return ext;
  ext = SSE41;

  if( !( regs[2] & BIT_HAS_SSE42 ) )
    return ext;
  ext = SSE42;

#  if SIMD_UP_TO_SSE42
  return ext;
#  else   //  !SIMD_UP_TO_SSE42

  doCpuidex( regs, 1, 1 );
  if( !( ( regs[2] & BIT_HAS_AVX ) == BIT_HAS_AVX ) )
    return ext;   // first check if the cpu supports avx
  if( ( xgetbv( 0 ) & 6 ) != 6 )
    return ext;   // then see if the os uses YMM state management via XSAVE etc...
#    ifndef _WIN32
  // don't detect AVX, as there are problems with MSVC production illegal ops for AVX
  ext = AVX;
#    endif

// #ifdef USE_AVX2
  doCpuidex( regs, 7, 0 );
  if( !( regs[1] & BIT_HAS_AVX2 ) )
    return ext;
  ext = AVX2;
// #endif

#    ifdef USE_AVX512
  if( ( xgetbv( 0 ) & 0xE0 ) != 0xE0 )
    return ext;   // see if OPMASK state and ZMM are availabe and enabled
  doCpuidex( regs, 7, 0 );
  if( !( regs[1] & BIT_HAS_AVX512F ) )
    return ext;
  if( !( regs[1] & BIT_HAS_AVX512DQ ) )
    return ext;
  if( !( regs[1] & BIT_HAS_AVX512BW ) )
    return ext;
  ext = AVX512;
#    endif   //  USE_AVX512
#  endif     // !SIMD_UP_TO_SSE42

  return ext;
}

#endif   // REAL_TARGET_X86

NO_OPT_SIMD
X86_VEXT read_x86_extension_flags( X86_VEXT request )
{
#ifdef REAL_TARGET_X86
  static const X86_VEXT max_supported = _get_x86_extensions();
  static X86_VEXT       ext_flags     = max_supported;
#else
  static const X86_VEXT max_supported = AVX2;                               // disable AVX2 for non-x86 because the SIMD-Everywhere implementation is buggy
  static X86_VEXT       ext_flags     = SIMD_EVERYWHERE_EXTENSION_LEVEL;   // default to SSE42 for WASM and SIMD-everywhere
#endif

  if( request != UNDEFINED )
  {
    if( request > max_supported )
    {
#ifdef REAL_TARGET_X86
      THROW( "requested SIMD level (" << x86_vext_to_string( request ) << ") not supported by current CPU (max " << x86_vext_to_string( max_supported ) << ")." );
#endif
    }

    ext_flags = request;
  }

#ifdef REAL_TARGET_X86
  if( max_supported < X86_SIMD_SSE41 )
  {
    THROW( "maximum SIMD level of current CPU is " << x86_vext_to_string( max_supported ) << " but at least SSE4.1 is required." );
  }
#endif

  return ext_flags;
}

const std::string& read_x86_extension_name()
{
  return x86_vext_to_string( read_x86_extension_flags() );
}

}   // namespace vvenc

#endif // TARGET_SIMD_X86

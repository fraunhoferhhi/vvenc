/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software,
especially patent licenses, a separate Agreement needs to be closed. 
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2019-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */


/*
 * \ingroup CommonLib
 * \file    CommondefX86.cpp
 * \brief   This file contains the SIMD x86 common used functions.
 */

#include "CommonDefX86.h"

#include <sstream>
#include <map>
#include <iostream>
#include <stdint.h>
#include <string>

#if defined( _WIN32 ) && !defined( __MINGW32__ )
#include <intrin.h>
#else
#include <cpuid.h>
#endif

#ifdef TARGET_SIMD_X86

//! \ingroup CommonLib
//! \{

namespace vvenc {

#if __GNUC__ // valid for GCC and clang
#define NO_USE_SIMD __attribute__((optimize("no-tree-vectorize")))
#else
#define NO_USE_SIMD
#endif


#if defined ( __MINGW32__ ) && !defined (  __MINGW64__ )
# define SIMD_UP_TO_SSE42 1
#else
# define SIMD_UP_TO_SSE42 0
#endif

/* use __cpuid for windows or inline assembler for gcc and clang */
#if defined( _WIN32 ) && !defined( __MINGW32__ )
#define do_cpuid    __cpuid
#define do_cpuidex  __cpuidex
#else
void do_cpuid(int CPUInfo[4], int InfoType)
{
    __get_cpuid( (unsigned)InfoType, (unsigned*)&CPUInfo[0], (unsigned*)&CPUInfo[1], (unsigned*)&CPUInfo[2], (unsigned*)&CPUInfo[3] );
}
#if !SIMD_UP_TO_SSE42
#define do_cpuidex(cd, v0, v1) __cpuid_count(v0, v1, cd[0], cd[1], cd[2], cd[3])
#endif
#endif

static inline int64_t xgetbv (int ctr) {
#if (defined (_MSC_FULL_VER) && _MSC_FULL_VER >= 160040000) || (defined (__INTEL_COMPILER) && __INTEL_COMPILER >= 1200) // Microsoft or Intel compiler supporting _xgetbv intrinsic

    return _xgetbv(ctr);                                   // intrinsic function for XGETBV

#elif defined(__GNUC__)                                    // use inline assembly, Gnu/AT&T syntax

   uint32_t a, d;
#if GCC_VERSION_AT_LEAST(4,4) || CLANG_VERSION_AT_LEAST(3,3)
   __asm("xgetbv" : "=a"(a),"=d"(d) : "c"(ctr) : );
#else
   __asm(".byte 0x0f, 0x01, 0xd0" : "=a"(a),"=d"(d) : "c"(ctr) : );
#endif
   return a | (uint64_t(d) << 32);

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
   return a | (uint64_t(d) << 32);

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
NO_USE_SIMD
X86_VEXT _get_x86_extensions()
{
    int regs[4] = {0, 0, 0, 0};
    X86_VEXT ext;
    ext = SCALAR;

    do_cpuid( regs, 0 );
    if( regs[0] == 0 ) return ext;
    do_cpuid( regs, 1 );
    if (!(regs[2] & BIT_HAS_SSE41)) return ext;
    ext = SSE41;
    if (!(regs[2] & BIT_HAS_SSE42)) return ext;
    ext = SSE42;
#if !SIMD_UP_TO_SSE42
    do_cpuidex( regs, 1, 1 );
    if (!((regs[2] & BIT_HAS_AVX) == BIT_HAS_AVX ))   return ext; // first check if the cpu supports avx
    if ((xgetbv(0) & 6) != 6)       return ext; // then see if the os uses YMM state management via XSAVE etc...
    ext = AVX;
// #ifdef USE_AVX2
    do_cpuidex( regs, 7, 0 );
    if (!(regs[1] & BIT_HAS_AVX2))  return ext;
    ext = AVX2;
// #endif
#ifdef USE_AVX512
    if ((xgetbv(0) & 0xE0) != 0xE0) return ext; // see if OPMASK state and ZMM are availabe and enabled
    do_cpuidex( regs, 7, 0 );
    if (!(regs[1] & BIT_HAS_AVX512F ))  return ext;
    if (!(regs[1] & BIT_HAS_AVX512DQ))  return ext;
    if (!(regs[1] & BIT_HAS_AVX512BW))  return ext;
    ext = AVX512;
#endif
#endif

    return ext;
}

typedef std::map<std::string, X86_VEXT> translate;
const static translate m
{ { "SCALAR", SCALAR },{ "SSE41", SSE41 },{ "SSE42", SSE42 },
  { "AVX", AVX },{ "AVX2", AVX2 },{ "AVX512", AVX512 } };

NO_USE_SIMD
X86_VEXT read_x86_extension_flags(const std::string &extStrId)
{
  //static std::atomic<bool> b_detection_finished(false);
  static bool b_detection_finished( false );
  static X86_VEXT ext_flags = SCALAR;

  {
    if( !b_detection_finished )
    {
      if( !extStrId.empty() )
      {
        translate::const_iterator search = m.find( extStrId );
        if( search != m.end() )
        {
          ext_flags = search->second;
        }
        else
        {
          EXIT( "SIMD Mode not supported: " << ext_flags << "\n" );
        }
      }
      else
      {
        ext_flags = _get_x86_extensions();
      }

      b_detection_finished = true;
    }
  }

  return ext_flags;
}

const char* read_x86_extension(const std::string &extStrId)
{
  static const char extension_not_available[] = "NA";

  X86_VEXT vext = read_x86_extension_flags(extStrId);

  for( translate::const_iterator it = m.begin(); it != m.end(); ++it )
    if( it->second == vext )
      return it->first.c_str();

  return extension_not_available;
}

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86


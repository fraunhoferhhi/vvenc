/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVenc

(c) Copyright (2019-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVenc (“Fraunhofer Versatile Video Encoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Encoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Encoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Encoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Encoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Encoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Encoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Encoding Library. If You use the Fraunhofer Versatile Video Encoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Encoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Encoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Encoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Encoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
----------------------------------------------------------------------------- */


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
          EXIT( "Mode not supported: " << ext_flags << "\n" );
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


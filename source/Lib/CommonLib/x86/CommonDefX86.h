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
/** \file     CommonDefX86.h
*/

#pragma once

#include "CommonDef.h"

#ifdef TARGET_SIMD_X86

#include <immintrin.h>
#if defined _MSC_VER
#include <tmmintrin.h>
#endif

//! \ingroup CommonLib
//! \{

namespace vvenc {

#ifdef USE_AVX512
#define SIMDX86 AVX512
#elif defined USE_AVX2
#define SIMDX86 AVX2
#elif defined USE_AVX
#define SIMDX86 AVX
#elif defined USE_SSE42
#define SIMDX86 SSE42
#elif defined USE_SSE41
#define SIMDX86 SSE41
#endif

#if defined( _MSC_VER ) && _MSC_VER <= 1900
#define _mm_bsrli_si128 _mm_srli_si128
#define _mm256_bsrli_epi128 _mm256_srli_si256
#endif


#define TRANSPOSE4x4(T) \
{\
  __m128i a01b01 = _mm_unpacklo_epi32(T[0], T[1]);\
  __m128i a23b23 = _mm_unpackhi_epi32(T[0], T[1]);\
  __m128i c01d01 = _mm_unpacklo_epi32(T[2], T[3]);\
  __m128i c23d23 = _mm_unpackhi_epi32(T[2], T[3]);\
\
  T[0] = _mm_unpacklo_epi64(a01b01, c01d01);\
  T[1] = _mm_unpackhi_epi64(a01b01, c01d01);\
  T[2] = _mm_unpacklo_epi64(a23b23, c23d23);\
  T[3] = _mm_unpackhi_epi64(a23b23, c23d23);\
}\

#define TRANSPOSE8x8(T) \
{\
  __m128i a03b03 = _mm_unpacklo_epi16(T[0], T[1]);\
  __m128i c03d03 = _mm_unpacklo_epi16(T[2], T[3]);\
  __m128i e03f03 = _mm_unpacklo_epi16(T[4], T[5]);\
  __m128i g03h03 = _mm_unpacklo_epi16(T[6], T[7]);\
  __m128i a47b47 = _mm_unpackhi_epi16(T[0], T[1]);\
  __m128i c47d47 = _mm_unpackhi_epi16(T[2], T[3]);\
  __m128i e47f47 = _mm_unpackhi_epi16(T[4], T[5]);\
  __m128i g47h47 = _mm_unpackhi_epi16(T[6], T[7]);\
\
  __m128i a01b01c01d01 = _mm_unpacklo_epi32(a03b03, c03d03);\
  __m128i a23b23c23d23 = _mm_unpackhi_epi32(a03b03, c03d03);\
  __m128i e01f01g01h01 = _mm_unpacklo_epi32(e03f03, g03h03);\
  __m128i e23f23g23h23 = _mm_unpackhi_epi32(e03f03, g03h03);\
  __m128i a45b45c45d45 = _mm_unpacklo_epi32(a47b47, c47d47);\
  __m128i a67b67c67d67 = _mm_unpackhi_epi32(a47b47, c47d47);\
  __m128i e45f45g45h45 = _mm_unpacklo_epi32(e47f47, g47h47);\
  __m128i e67f67g67h67 = _mm_unpackhi_epi32(e47f47, g47h47);\
\
  T[0] = _mm_unpacklo_epi64(a01b01c01d01, e01f01g01h01);\
  T[1] = _mm_unpackhi_epi64(a01b01c01d01, e01f01g01h01);\
  T[2] = _mm_unpacklo_epi64(a23b23c23d23, e23f23g23h23);\
  T[3] = _mm_unpackhi_epi64(a23b23c23d23, e23f23g23h23);\
  T[4] = _mm_unpacklo_epi64(a45b45c45d45, e45f45g45h45);\
  T[5] = _mm_unpackhi_epi64(a45b45c45d45, e45f45g45h45);\
  T[6] = _mm_unpacklo_epi64(a67b67c67d67, e67f67g67h67);\
  T[7] = _mm_unpackhi_epi64(a67b67c67d67, e67f67g67h67);\
}\

#define TRANSPOSESAT4x4(T)\
{\
  TRANSPOSE4x4(T);\
  T[0] = _mm_cvtepi16_epi32(_mm_packs_epi32(T[0], vzero));\
  T[1] = _mm_cvtepi16_epi32(_mm_packs_epi32(T[1], vzero));\
  T[2] = _mm_cvtepi16_epi32(_mm_packs_epi32(T[2], vzero));\
  T[3] = _mm_cvtepi16_epi32(_mm_packs_epi32(T[3], vzero));\
}\

#define ADDCLIP4(dstptr, res, min, max)\
{\
  __m128i vdst = _mm_lddqu_si128((__m128i*) dstptr);\
  vdst = _mm_add_epi16(vdst,  _mm_packs_epi32(res, vzero));\
  vdst = _mm_min_epi16(max, _mm_max_epi16(min, vdst));\
  _mm_storel_epi64((__m128i*) dstptr, vdst);\
}\

#define TRANSPOSESTORE8x8_ALGN(T, D, stride)\
{\
  TRANSPOSE8x8(T); \
\
  _mm_store_si128((__m128i*)&D[0*stride], T[0]);\
  _mm_store_si128((__m128i*)&D[1*stride], T[1]);\
  _mm_store_si128((__m128i*)&D[2*stride], T[2]);\
  _mm_store_si128((__m128i*)&D[3*stride], T[3]);\
  _mm_store_si128((__m128i*)&D[4*stride], T[4]);\
  _mm_store_si128((__m128i*)&D[5*stride], T[5]);\
  _mm_store_si128((__m128i*)&D[6*stride], T[6]);\
  _mm_store_si128((__m128i*)&D[7*stride], T[7]);\
}\

#define ADDCLIP(dstptr, res, min, max)\
{\
  __m128i vdst = _mm_load_si128((__m128i*) dstptr);\
  vdst = _mm_add_epi16(vdst, res ); \
  vdst = _mm_min_epi16(max,_mm_max_epi16(min, vdst));\
  _mm_store_si128((__m128i*)dstptr, vdst);\
}\

#define TRANSPOSEADDCLIPSTORE8x8_ALGN(T, D, stride, min, max)\
{\
  TRANSPOSE8x8(T); \
\
  ADDCLIP(&D[0*stride], T[0], min, max);\
  ADDCLIP(&D[1*stride], T[1], min, max);\
  ADDCLIP(&D[2*stride], T[2], min, max);\
  ADDCLIP(&D[3*stride], T[3], min, max);\
  ADDCLIP(&D[4*stride], T[4], min, max);\
  ADDCLIP(&D[5*stride], T[5], min, max);\
  ADDCLIP(&D[6*stride], T[6], min, max);\
  ADDCLIP(&D[7*stride], T[7], min, max);\
}\


static inline __m128i _mm_sel_si128(__m128i a, __m128i b, __m128i mask)
{
#ifdef USE_SSE41
  return _mm_blendv_epi8( a, b, mask);
#else
  return _mm_or_si128( _mm_andnot_si128( mask, a ), _mm_and_si128( b, mask ));
#endif
}


static inline __m128i _mm_clip_epi8(__m128i v, __m128i low, __m128i hi)
{
#ifdef USE_SSE41
  return _mm_min_epi8(_mm_max_epi8(v, low), hi);
#else
  __m128i vlowm = _mm_cmplt_epi8(v, low);
  __m128i vhighm = _mm_cmpgt_epi8(v, hi);
  return _mm_sel_si128(_mm_sel_si128(v, low, vlowm), hi, vhighm);
#endif
}


#ifdef USE_AVX2

static inline __m128i _mm256_cvtepi32_epi16x( __m256i& v )
{
  v = _mm256_packs_epi32( v, _mm256_setzero_si256() );
  return _mm_unpacklo_epi64( _mm256_extracti128_si256( v, 0 ), _mm256_extracti128_si256( v, 1 ) );
}

#define TRANSPOSESTORE16x16_ALGN(T, D, stride)\
{\
  TRANSPOSE16x16_AVX2(T); \
\
  for (int _i_=0; _i_ < 16; _i_++)\
  _mm256_store_si256((__m256i*)&D[_i_*stride], T[_i_]);\
}\

#define ADDCLIPAVX2(dstptr, res, min, max)\
{\
  __m256i vdst = _mm256_load_si256((__m256i*) dstptr);\
  vdst = _mm256_adds_epi16(vdst, res ); \
  vdst = _mm256_min_epi16(max,_mm256_max_epi16(min, vdst));\
  _mm256_store_si256((__m256i*)dstptr, vdst);\
}\

#define TRANSPOSEADDCLIPSTORE16x16_ALGN(T, D, stride, min, max)\
{\
  TRANSPOSE16x16_AVX2(T); \
\
  for (int _i_=0; _i_ < 16; _i_++)\
    ADDCLIPAVX2(&D[_i_*stride], T[_i_], min, max);\
}\


static inline void TRANSPOSE16x16_AVX2(__m256i T[16]){
  __m256i T_03[8];
  __m256i T_47[8];
  for (int i=0; i<8; i++){
    T_03[i] = _mm256_unpacklo_epi16(T[2*i], T[2*i+1]);
    T_47[i] = _mm256_unpackhi_epi16(T[2*i], T[2*i+1]);
  }

  __m256i T_01[4];
  __m256i T_23[4];
  __m256i T_45[4];
  __m256i T_67[4];
  for (int i=0; i<4; i++){
    T_01[i] = _mm256_unpacklo_epi32(T_03[2*i], T_03[2*i+1]);
    T_23[i] = _mm256_unpackhi_epi32(T_03[2*i], T_03[2*i+1]);
    T_45[i] = _mm256_unpacklo_epi32(T_47[2*i], T_47[2*i+1]);
    T_67[i] = _mm256_unpackhi_epi32(T_47[2*i], T_47[2*i+1]);
  }
  __m256i TR[8][2];

  for (int i=0; i<2; i++){
    TR[0][i] = _mm256_unpacklo_epi64(T_01[2*i], T_01[2*i+1]);
    TR[1][i] = _mm256_unpackhi_epi64(T_01[2*i], T_01[2*i+1]);
    TR[2][i] = _mm256_unpacklo_epi64(T_23[2*i], T_23[2*i+1]);
    TR[3][i] = _mm256_unpackhi_epi64(T_23[2*i], T_23[2*i+1]);
    TR[4][i] = _mm256_unpacklo_epi64(T_45[2*i], T_45[2*i+1]);
    TR[5][i] = _mm256_unpackhi_epi64(T_45[2*i], T_45[2*i+1]);
    TR[6][i] = _mm256_unpacklo_epi64(T_67[2*i], T_67[2*i+1]);
    TR[7][i] = _mm256_unpackhi_epi64(T_67[2*i], T_67[2*i+1]);
  }
  for (int i=0; i<8; i++){
    T[i] = _mm256_permute2x128_si256(TR[i][0], TR[i][1], 0x20);
    T[i+8] = _mm256_permute2x128_si256(TR[i][0], TR[i][1], 0x31);
  }
}


static inline void TRANSPOSE16x8_AVX2(__m256i T[8]){
  __m256i T_03[4];
  __m256i T_47[4];
  for (int i=0; i<4; i++){
    T_03[i] = _mm256_unpacklo_epi16(T[2*i], T[2*i+1]);
    T_47[i] = _mm256_unpackhi_epi16(T[2*i], T[2*i+1]);
  }

  __m256i T_01[2];
  __m256i T_23[2];
  __m256i T_45[2];
  __m256i T_67[2];
  for (int i=0; i<2; i++){
    T_01[i] = _mm256_unpacklo_epi32(T_03[2*i], T_03[2*i+1]);
    T_23[i] = _mm256_unpackhi_epi32(T_03[2*i], T_03[2*i+1]);
    T_45[i] = _mm256_unpacklo_epi32(T_47[2*i], T_47[2*i+1]);
    T_67[i] = _mm256_unpackhi_epi32(T_47[2*i], T_47[2*i+1]);
  }
  T[0] = _mm256_unpacklo_epi64(T_01[0], T_01[1]);
  T[1] = _mm256_unpackhi_epi64(T_01[0], T_01[1]);
  T[2] = _mm256_unpacklo_epi64(T_23[0], T_23[1]);
  T[3] = _mm256_unpackhi_epi64(T_23[0], T_23[1]);
  T[4] = _mm256_unpacklo_epi64(T_45[0], T_45[1]);
  T[5] = _mm256_unpackhi_epi64(T_45[0], T_45[1]);
  T[6] = _mm256_unpacklo_epi64(T_67[0], T_67[1]);
  T[7] = _mm256_unpackhi_epi64(T_67[0], T_67[1]);
}


static inline void TRANSPOSE8x8_32b_AVX2(__m256i T[8])
{
  __m256i T_03[4];
  __m256i T_47[4];
  for (int i=0; i<4; i++){
    T_03[i] = _mm256_unpacklo_epi32(T[2*i], T[2*i+1]);
    T_47[i] = _mm256_unpackhi_epi32(T[2*i], T[2*i+1]);
  }
  __m256i T_01[2];
  __m256i T_23[2];
  __m256i T_45[2];
  __m256i T_67[2];
  for (int i=0; i<2; i++){
    T_01[i] = _mm256_unpacklo_epi64(T_03[2*i], T_03[2*i+1]);
    T_23[i] = _mm256_unpackhi_epi64(T_03[2*i], T_03[2*i+1]);
    T_45[i] = _mm256_unpacklo_epi64(T_47[2*i], T_47[2*i+1]);
    T_67[i] = _mm256_unpackhi_epi64(T_47[2*i], T_47[2*i+1]);
  }

  T[0] = _mm256_permute2x128_si256(T_01[0], T_01[1], 0x20);
  T[4] = _mm256_permute2x128_si256(T_01[0], T_01[1], 0x31);
  T[1] = _mm256_permute2x128_si256(T_23[0], T_23[1], 0x20);
  T[5] = _mm256_permute2x128_si256(T_23[0], T_23[1], 0x31);
  T[2] = _mm256_permute2x128_si256(T_45[0], T_45[1], 0x20);
  T[6] = _mm256_permute2x128_si256(T_45[0], T_45[1], 0x31);
  T[3] = _mm256_permute2x128_si256(T_67[0], T_67[1], 0x20);
  T[7] = _mm256_permute2x128_si256(T_67[0], T_67[1], 0x31);
}


#endif

#ifdef USE_AVX512

ALWAYS_INLINE inline __m512i
_mm512_set_epi16( int16_t x31, int16_t x30, int16_t x29, int16_t x28,
                  int16_t x27, int16_t x26, int16_t x25, int16_t x24,
                  int16_t x23, int16_t x22, int16_t x21, int16_t x20,
                  int16_t x19, int16_t x18, int16_t x17, int16_t x16,
                  int16_t x15, int16_t x14, int16_t x13, int16_t x12,
                  int16_t x11, int16_t x10,  int16_t x9,  int16_t x8,
                  int16_t  x7,  int16_t x6,  int16_t x5,  int16_t x4,
                  int16_t  x3,  int16_t x2,  int16_t x1,  int16_t x0 )
{
  return _mm512_set_epi32( (x31<<16) + (0xffff & x30), (x29<<16) + (0xffff & x28),
                           (x27<<16) + (0xffff & x26), (x25<<16) + (0xffff & x24),
                           (x23<<16) + (0xffff & x22), (x21<<16) + (0xffff & x20),
                           (x19<<16) + (0xffff & x18), (x17<<16) + (0xffff & x16),
                           (x15<<16) + (0xffff & x14), (x13<<16) + (0xffff & x12),
                           (x11<<16) + (0xffff & x10), ( x9<<16) + (0xffff &  x8),
                           ( x7<<16) + (0xffff &  x6), ( x5<<16) + (0xffff &  x4),
                           ( x3<<16) + (0xffff &  x2), ( x1<<16) + (0xffff &  x0) );
}
#endif

#ifdef ENABLE_REGISTER_PRINTING
/* note for gcc: this helper throws a compilation error
 * because of name mangling when used with different types for R at the same time,
 * the workaround is to compile with -fabi-version=4 or higher
 * (needs gcc >= 4.5) */
template< class T, class R >
static void _printReg( const R var, const char* varname, uint8_t count = sizeof( R ) )
{
    T *val = (T*)&var;
    const int varcnt = std::min(count, (uint8_t)(sizeof(R)/sizeof(T)));
    std::cout << varname << ":";
    for( int i = 0; i < varcnt; ++i )
    {
        std::cout << " " << std::setw(sizeof(T)*2 + 1)<< val[i];
    }
    std::cout << std::endl;
}

#define PREG( var, t, cnt ) \
{ \
  static unsigned c = cnt; \
  if( c ) \
  { \
    std::cout << cnt - c << " "; \
    _printReg<t>( var, #var ); \
    c--;  \
  } \
}
#else
#define PREG( var, t, cnt )
#endif

} // namespace vvenc

//! \}

#endif // TARGET_SIMD_X86

